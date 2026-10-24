//! Basic job-application entry point.
//!
//! Wires up a small in-memory example board, runs the planner, and drives
//! the `Runner` to completion while logging the resulting `Action`s to
//! stdout. The up-camera is mocked: when the runner pauses for inspection
//! we feed back a `PartObservation` with a synthetic rotation error so the
//! correction path in `PickState::CorrectingRotation` actually fires.
//!
//! No real hardware is touched; the same `Action` stream a real backend
//! would dispatch is just printed here.
//!
//! Run with: `cargo run`.

use std::str::FromStr;

use pnp_logic::feeder::PartId;
use pnp_logic::job_io::{HeadSpec, JobFile, NozzleSpec};
use pnp_logic::parts::{Footprint, Part, Polarity};
use pnp_logic::plan::{FeederRotation, PlanSearchMode};
use pnp_logic::{
    Camera, CameraKind, FeederBank, FeederCalibration, FeederKind, FeederPlacementMode, FeederSlot,
    MachineConfig, MachineContext, MachineSetup, NozzleHead, NozzleKind, NozzleSlot,
    PartObservation, PartPickup, PlacementNeed, PlacementStrategy, Point, Pose, Runner, Size,
    StepResult, plan_job_with_mode,
};

fn main() {
    // 1. Build a six-nozzle head with the actual nozzle catalogue used by
    //    this job. HWGC-SmtApp/OpenKayo use physical nozzle numbers 1..=N.
    let head = NozzleHead::new(vec![
        NozzleSlot {
            id: 1,
            offset_mm: 0.0,
            kind: NozzleKind::N503,
            rotation_deg: 0.0,
            vacuum: false,
        },
        NozzleSlot {
            id: 2,
            offset_mm: 20.0,
            kind: NozzleKind::N503,
            rotation_deg: 0.0,
            vacuum: false,
        },
        NozzleSlot {
            id: 3,
            offset_mm: 40.0,
            kind: NozzleKind::N503,
            rotation_deg: 0.0,
            vacuum: false,
        },
        NozzleSlot {
            id: 4,
            offset_mm: 60.0,
            kind: NozzleKind::Standard,
            rotation_deg: 0.0,
            vacuum: false,
        },
    ]);

    // 2. Build the feeder rail. Two banks rotated +90° (front rail) and
    //    -90° (back rail) so we can see bank rotation in the move output.
    //    Slots 0–4: front rail, y=400. Slots 5–7: back rail, y=420.
    //    Slot 8: 16mm tape on the front rail, holding the BGA (only 1
    //    placement, so the feeder swing cost is negligible).
    //    Slot 9: 12mm tape on the front rail, alternate LED slot.
    let feeders = vec![
        FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(-100.0, 400.0),
            loaded: None,
        },
        FeederSlot {
            id: 1,
            bank_id: 0,
            position: Point::new(-50.0, 400.0),
            loaded: None,
        },
        FeederSlot {
            id: 2,
            bank_id: 0,
            position: Point::new(0.0, 400.0),
            loaded: None,
        },
        FeederSlot {
            id: 3,
            bank_id: 0,
            position: Point::new(50.0, 400.0),
            loaded: None,
        },
        FeederSlot {
            id: 4,
            bank_id: 0,
            position: Point::new(100.0, 400.0),
            loaded: None,
        },
        FeederSlot {
            id: 5,
            bank_id: 1,
            position: Point::new(-100.0, 420.0),
            loaded: None,
        },
        FeederSlot {
            id: 6,
            bank_id: 1,
            position: Point::new(-50.0, 420.0),
            loaded: None,
        },
        FeederSlot {
            id: 7,
            bank_id: 1,
            position: Point::new(0.0, 420.0),
            loaded: None,
        },
        FeederSlot {
            id: 8,
            bank_id: 0,
            position: Point::new(0.0, 380.0),
            loaded: None,
        },
        FeederSlot {
            id: 9,
            bank_id: 0,
            position: Point::new(50.0, 380.0),
            loaded: None,
        },
    ];
    let banks = vec![
        FeederBank {
            id: 0,
            rotation_deg: 90.0,
        },
        FeederBank {
            id: 1,
            rotation_deg: -90.0,
        },
    ];

    // Calibration: each (bank, part) carries a per-pocket offset, the
    // rotation the part comes off the tape at, and the Z height the
    // nozzle should descend to. Without calibration the runner falls
    // back to MachineConfig::pickup_z and the Part's nominal pickup
    // (both zero-offset here).
    let mut calibration = FeederCalibration::new();
    // R1 (PartId 1) on the front bank: pickup pocket is 1 mm forward of
    // the slot centre (in the bank's frame) and the part comes off the
    // tape at 0°. Z is 0.4 mm.
    calibration.set(
        0,
        PartId(1),
        PartPickup {
            offset: Point::new(1.0, 0.0),
            rotation_deg: 0.0,
            pickup_z_mm: 0.4,
        },
    );
    // LED (PartId 2) on the back bank: pocket 2 mm back, Z 0.7 mm.
    calibration.set(
        1,
        PartId(2),
        PartPickup {
            offset: Point::new(0.0, 2.0),
            rotation_deg: 0.0,
            pickup_z_mm: 0.7,
        },
    );

    // 3. Define the parts in the BOM. Three parts, one of them appears
    //    30 times (a common 100Ω resistor), one 10 times (an LED), one
    //    once (a BGA).
    let parts = make_parts();

    // 4. Build the placement list for one example board. Mix the order
    //    so the planner has to re-burst it.
    let mut needs = Vec::new();
    // 5 R1 placements, 2 LED, 1 BGA = 8 total — keeps the BGA in scope.
    for (x, y) in [
        (20.0, 20.0),
        (60.0, 20.0),
        (100.0, 20.0),
        (20.0, 60.0),
        (60.0, 60.0),
        (70.0, 60.0),
        (80.0, 60.0),
        (90.0, 60.0),
    ] {
        needs.push(PlacementNeed {
            reference: String::new(),
            part: parts[0].clone(), // R1 (100Ω, 30 placements)
            design_position: Point::new(x, y),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        });
    }
    for (x, y) in [
        (30.0, 20.0),
        (80.0, 20.0),
        (120.0, 20.0),
        (40.0, 60.0),
        (65.0, 60.0),
    ] {
        needs.push(PlacementNeed {
            reference: String::new(),
            part: parts[1].clone(), // LED, polarised
            design_position: Point::new(x, y),
            design_rotation_deg: 0.0,
            should_place: true,
            // Rotate 30° before camera so the cathode stripe is visible
            // to the vision routine; rotate -30° after correction to land
            // the stripe at the design's polarity mark.
            strategy: PlacementStrategy::Polarized {
                before_camera_deg: 30.0,
                after_camera_deg: -30.0,
            },
            camera_kind: CameraKind::FastVision,
        });
    }
    needs.push(PlacementNeed {
        reference: String::new(),
        part: parts[2].clone(), // BGA, fine-pitch: precision mode
        design_position: Point::new(60.0, 40.0),
        design_rotation_deg: 0.0,
        should_place: true,
        strategy: PlacementStrategy::Precision {
            // Loose enough for the demo's halving mock to converge
            // within 5 attempts.
            rotation_tolerance_deg: 0.5,
            offset_tolerance_mm: 0.5,
            max_attempts: 5,
        },
        camera_kind: CameraKind::Precise,
    });

    // 5. Per-job setup. The operator doesn't declare specific tape widths
    //    — the planner reads each part's footprint and picks the
    //    smallest compatible tape width automatically. Slots 8 and 9
    //    get set here as physical 16mm tape (the BGA needs a wider
    //    reel than 8mm in production), but slots 0–7 are auto-grow
    //    to whatever the part needs.
    let mut setup = MachineSetup::new();
    setup.set_kind(8, FeederKind::Tape(16));
    setup.set_kind(9, FeederKind::Tape(16));

    let cameras = vec![
        Camera {
            id: 1,
            kind: CameraKind::FastVision,
            position: Point::new(0.0, 200.0),
            array_pitch_mm: Some(20.0),
        },
        Camera {
            id: 0,
            kind: CameraKind::Precise,
            position: Point::new(0.0, 0.0),
            array_pitch_mm: None,
        },
        Camera {
            id: 2,
            kind: CameraKind::FastVision,
            position: Point::new(20.0, 200.0),
            array_pitch_mm: Some(20.0),
        },
        Camera {
            id: 3,
            kind: CameraKind::FastVision,
            position: Point::new(40.0, 200.0),
            array_pitch_mm: Some(20.0),
        },
        Camera {
            id: 4,
            kind: CameraKind::FastVision,
            position: Point::new(60.0, 200.0),
            array_pitch_mm: Some(20.0),
        },
    ];

    // 6. Plan the job.
    let work_centroid = Point::new(60.0, 50.0);
    let config = MachineConfig::default();
    // Movable mode places compatible feeders near the board centroid. Use
    // `Fixed` here when `FeederSlot::loaded` or `setup.set_loaded(...)` has
    // been populated with the reels already mounted on the machine.
    let feeder_mode = FeederPlacementMode::Movable;
    // Algorithm selector. Change `PlanAlgorithm::Greedy` to
    // `PlanAlgorithm::AntColony` to run the ACO search.
    let plan_mode = PlanSearchMode::LocalSearch;
    if matches!(plan_mode, PlanSearchMode::AntColony) {
        println!("=== Plan algorithm: AntColony (ACO) ===");
    } else if matches!(plan_mode, PlanSearchMode::LocalSearch) {
        println!("=== Plan algorithm: LocalSearch (board-aware) ===");
    } else {
        println!("=== Plan algorithm: Greedy ===");
    }
    let planned = plan_job_with_mode(
        "DEMO-BOARD-1",
        needs,
        &head,
        &feeders,
        &setup,
        work_centroid,
        &config,
        feeder_mode,
        plan_mode,
    );

    println!("=== Feeder assignment ===");
    for (pid, slots) in &planned.feeder_assignment.slot_for_part {
        let part_mpn = planned
            .job
            .placements
            .iter()
            .find(|p| p.part.id == *pid)
            .map(|p| p.part.mpn.clone())
            .unwrap_or_default();
        println!("  {} ({:?}) -> slots {:?}", part_mpn, pid, slots);
    }
    if !planned.feeder_assignment.unassigned.is_empty() {
        println!("  unassigned: {:?}", planned.feeder_assignment.unassigned);
    }

    println!(
        "\n=== Planned run: {} placements, estimated {:.1} s ({:.0} CPH) ===\n",
        planned.job.placements.len(),
        planned.estimated_seconds,
        placements_per_hour(planned.job.placements.len(), planned.estimated_seconds),
    );

    println!("=== Nozzle assignment ===");
    for (index, placement) in planned.job.placements.iter().enumerate() {
        let nozzle = head.nozzle(placement.nozzle);
        println!(
            "  placement {:02}: {:<5} -> nozzle {} ({}) / camera {:?}",
            index,
            placement.part.mpn,
            placement.nozzle,
            nozzle.kind.name(),
            placement.camera_kind,
        );
    }
    println!("=== Multi-nozzle pickup batches ===");
    for burst in &planned.multi_nozzle_bursts {
        println!(
            "  part {:?}: placements {:?} on nozzles {:?}",
            burst.part_id, burst.placements, burst.nozzle_ids
        );
    }

    // Save the pre-optimised job to JSON so the UI can load it
    // directly. The file format is `JobFile` (see src/job_io.rs).
    let job_file = JobFile {
        board_id: planned.job.board_id.clone(),
        track_width_mm: None,
        feeders: feeders.clone(),
        banks: banks.clone(),
        head: HeadSpec {
            nozzles: head
                .nozzles
                .iter()
                .map(|n| NozzleSpec {
                    id: n.id,
                    kind: n.kind,
                    offset_mm: n.offset_mm,
                })
                .collect(),
        },
        placements: planned.job.placements.clone(),
        feeder_assignment: planned.feeder_assignment.clone(),
        fiducials: vec![],
        board_mapping: None,
        estimated_seconds: None,
        placement_columns: Vec::new(),
    };
    let json = serde_json::to_string_pretty(&job_file).unwrap();
    println!(
        "\n=== Job JSON ({} bytes) ===\n{}",
        json.len(),
        &json[..json.len().min(400)]
    );

    // Reload the JSON and run the loaded runner to verify the round-trip.
    // Machine settings ride alongside the job file, not inside it.
    let reloaded = JobFile::from_str(&json).unwrap();
    let machine = MachineContext {
        cameras: cameras.clone(),
        config: config.clone(),
        setup: setup.clone(),
        calibration: vec![],
    };
    let mut loaded_runner = reloaded.into_runner(machine).unwrap();
    loaded_runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
    let mut loaded_actions = 0;
    for _ in 0..5000 {
        match loaded_runner.step() {
            StepResult::Action(_) => loaded_actions += 1,
            StepResult::NeedPrompt(_) => loaded_runner.feed_prompt_response(),
            StepResult::NeedCamera => loaded_runner.feed_camera(PartObservation::ok()),
            StepResult::Done | StepResult::Aborted(_) => break,
        }
    }
    println!(
        "\n=== loaded runner produced {} actions ===",
        loaded_actions
    );
    assert!(loaded_runner.job().placements.iter().all(|p| p.placed));

    // 6. Build the runner with the feeder rotation, machine setup, and
    //    cameras the planner / setup produced.
    let rotation = FeederRotation::from_assignment(&planned.feeder_assignment);
    let mut runner = Runner::with_setup(
        head,
        feeders,
        banks,
        calibration,
        setup,
        cameras,
        config,
        planned.job,
        rotation,
    );

    // 7. Set a board pose (skipping the fiducial demo; just use identity).
    runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));

    //dbg!( runner.collect_actions() );

    // 8. Drive the runner to completion. Mock the camera by feeding back
    //    a synthetic rotation error every time we're asked.
    let mut step_count = 0;
    let mut last_nozzle_action = String::new();
    loop {
        step_count += 1;
        if step_count > 5000 {
            println!("\naborted: runner did not finish in 5000 steps");
            std::process::exit(1);
        }
        match runner.step() {
            StepResult::Action(a) => {
                let label = a.label();
                // Extract nozzle info from action label if present
                let nozzle_info = if label.contains("n") && label.contains("(") {
                    let start = label.find('(').unwrap_or(0);
                    let end = label.find(')').unwrap_or(label.len());
                    format!(" [nozzle:{}]", &label[start + 1..end])
                } else {
                    String::new()
                };
                println!("[step {:04}]{} {}", step_count, nozzle_info, label);
                last_nozzle_action = label;
            }
            StepResult::NeedCamera => {
                let obs = mock_camera();
                println!(
                    "[step {:04}] camera check: present={} rot_err={:+.2}° tol={} (last: {})",
                    step_count,
                    obs.present,
                    obs.rotation_error_deg,
                    obs.within_tolerance,
                    last_nozzle_action
                );
                runner.feed_camera(obs);
            }
            StepResult::NeedPrompt(msg) => {
                println!("[step {:04}] prompt: {}", step_count, msg);
                runner.feed_prompt_response();
            }
            StepResult::Done => {
                let placed = runner.job().placements.iter().filter(|p| p.placed).count();
                println!("\n=== done: {} placements placed ===", placed);
                break;
            }
            StepResult::Aborted(why) => {
                println!("\n=== aborted: {} ===", why);
                std::process::exit(1);
            }
        }
    }
}

use std::sync::atomic::{AtomicU32, Ordering};

static CAMERA_CALL: AtomicU32 = AtomicU32::new(0);

fn mock_camera() -> PartObservation {
    // Pretend the camera converges over time: every call halves the
    // reported error. This exercises the precision loop without
    // exhausting the BGA's max_attempts.
    let n = CAMERA_CALL.fetch_add(1, Ordering::Relaxed);
    let factor = 1.0 / (1u32 << n.min(6)) as f64;
    PartObservation {
        rotation_error_deg: 7.5 * factor,
        centroid_offset: Point::new(1.0 * factor, 2.0 * factor),
        present: true,
        within_tolerance: true,
    }
}

fn make_parts() -> [Part; 3] {
    let mk = |id: u32, mpn: &str, desc: &str, w: f64, h: f64, nozzle_type: Vec<NozzleKind>| Part {
        id: PartId(id),
        mpn: mpn.into(),
        description: desc.into(),
        footprint: Footprint {
            name: mpn.into(),
            centroid: Point::ORIGIN,
            size: Size {
                width: w,
                height: h,
            },
            pads: vec![],
            polarity: Polarity::None,
        },
        nozzle_type,
        pickup_rotation_deg: 0.0,
        pickup_offset: Point::ORIGIN,
        camera: None,
    };
    [
        mk(1, "R1", "100Ω 0603", 1.6, 0.8, vec![NozzleKind::N503]),
        mk(
            2,
            "LED",
            "green LED 0805",
            2.0,
            1.25,
            vec![NozzleKind::N503],
        ),
        mk(3, "BGA", "BGA-64 8x8", 8.0, 8.0, vec![NozzleKind::Standard]),
    ]
}

fn placements_per_hour(n: usize, seconds: f64) -> f64 {
    if seconds <= 0.0 {
        0.0
    } else {
        n as f64 / seconds * 3600.0
    }
}
