//! Pre-optimised job I/O.
//!
//! A `JobFile` is the on-disk format for a fully planned job. It bundles
//! the machine configuration (feeders, banks, cameras, head, calibration,
//! setup, `MachineConfig`) with the planner's output (the placement
//! sequence and the `FeederAssignment`). When loaded, the file is
//! hydrated into a `Runner` ready to drive.
//!
//! The runner's runtime state (`PickState`, `PlaceState`, etc.) is not
//! part of the file — loaders reset the job to `JobState::Setup` so the
//! runner drives it from the start.
//!
//! ```no_run
//! use pnp_logic::{JobFile, MachineContext, Runner, StepResult, PartObservation};
//! use std::str::FromStr;
//!
//! let json = std::fs::read_to_string("demo.json").unwrap();
//! let file = JobFile::from_str(&json).unwrap();
//! let mut runner = file.into_runner(MachineContext::default()).unwrap();
//! runner.set_board_pose(pnp_logic::Pose::new(0.0, 0.0, 0.0));
//!
//! loop {
//!     match runner.step() {
//!         StepResult::Action(_) => {},
//!         StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
//!         StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
//!         StepResult::Done | StepResult::Aborted(_) => break,
//!     }
//! }
//! ```

use std::str::FromStr;

use crate::config::{Camera, MachineSetup};
use crate::feeder::PartId;
use crate::feeder::{FeederBank, FeederCalibration, FeederSlot, PartPickup};
use crate::nozzle::{NozzleHead, NozzleKind, NozzleSlot};
use crate::optimizer::FeederAssignment;
use crate::plan::FeederRotation;
use crate::runner::{MachineConfig, Runner};
use crate::state::{Job, Placement};

/// One physical camera on the machine (re-export of `config::Camera`).
pub use crate::config::Camera as CameraSlot;

/// Per-bank per-part pickup calibration entry. Serialised as a flat
/// tuple so the on-disk format is stable across versions.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CalibrationEntry {
    pub bank_id: u8,
    pub part_id: PartId,
    pub pickup: PartPickup,
}

/// Serialised nozzle specification (no rotation, no vacuum — those
/// are runtime state).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NozzleSpec {
    pub id: u8,
    pub kind: NozzleKind,
    pub offset_mm: f64,
}

/// Serialised head spec.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct HeadSpec {
    pub nozzles: Vec<NozzleSpec>,
}

/// The operator-verified mapping of board design coordinates onto
/// machine coordinates, recovered from two located panel fiducials.
/// `origin` is where board (0, 0) sits on the machine; `rotation_deg`
/// is how far the board is rotated relative to the design; `scale`
/// should be ≈1 and is kept as a sanity check.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BoardMapping {
    pub origin_x: f64,
    pub origin_y: f64,
    pub rotation_deg: f64,
    pub scale: f64,
}

impl BoardMapping {
    /// Map a board design position onto machine coordinates.
    pub fn apply(&self, point: shared_types::Point) -> shared_types::Point {
        use shared_types::Point;
        let rad = self.rotation_deg.to_radians();
        let (sin, cos) = rad.sin_cos();
        let scaled = self.scale;
        Point::new(
            self.origin_x + (point.x * cos - point.y * sin) * scaled,
            self.origin_y + (point.x * sin + point.y * cos) * scaled,
        )
    }
}

/// Pre-optimised job file. Holds everything that describes *this job*:
/// the planner's placement sequence plus feeder assignment, and the
/// nozzle kinds the operator chose for it. Machine-side settings
/// (cameras, machine config, feeder setup, feeder calibration) are NOT
/// part of the job — they live in the machine's settings database and
/// are injected through [`MachineContext`] when the job is turned into
/// a runner.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct JobFile {
    pub board_id: String,
    #[serde(default)]
    pub track_width_mm: Option<f64>,
    pub feeders: Vec<FeederSlot>,
    pub banks: Vec<FeederBank>,
    pub head: HeadSpec,
    pub placements: Vec<Placement>,
    pub feeder_assignment: FeederAssignment,
    /// Design-space fiducials. The recovered machine pose is runtime state
    /// and is deliberately not serialized here.
    #[serde(default)]
    pub fiducials: Vec<crate::identifier::Fiducial>,
    /// Operator-verified board mapping from the fiducial workflow in the
    /// job configuration page. Runtime pose recovery still happens live;
    /// this records the mapping for display and sanity checks.
    #[serde(default)]
    pub board_mapping: Option<BoardMapping>,
    /// Planner's cycle-time estimate for the stored plan, in seconds.
    /// Used to display an estimated CPH (components per hour) in the
    /// process view; `None` for a job that has not been planned yet.
    #[serde(default)]
    pub estimated_seconds: Option<f64>,
    /// Column labels of the placement list this job came from, so the
    /// editor can restore the exact layout the operator imported instead
    /// of guessing a default grid. Empty when the job was never edited.
    #[serde(default)]
    pub placement_columns: Vec<String>,
}

/// Machine-side settings that accompany every run but belong to the
/// machine, not to a job: cameras, machine config, feeder setup and
/// feeder calibration. Persisted in the settings database by the host
/// application and injected when a [`JobFile`] is turned into a
/// [`Runner`].
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct MachineContext {
    pub cameras: Vec<Camera>,
    pub config: MachineConfig,
    pub setup: MachineSetup,
    pub calibration: Vec<CalibrationEntry>,
}

/// Supports loading from string
impl FromStr for JobFile {
    type Err = String;

    fn from_str(s: &str) -> Result<Self, Self::Err> {
        serde_json::from_str(s).map_err(|e| e.to_string())
    }
}

impl JobFile {
    /// Serialise the job file to a JSON string.
    pub fn to_string(&self) -> Result<String, serde_json::Error> {
        serde_json::to_string_pretty(self)
    }

    /// Recover the board pose from the first two panel fiducials that have
    /// machine-space measurements.
    pub fn board_pose(&self) -> Option<crate::geometry::Pose> {
        let fiducials = self
            .fiducials
            .iter()
            .filter(|fiducial| fiducial.kind == crate::identifier::FiducialKind::Panel)
            .filter_map(|fiducial| fiducial.machine_position.map(|machine| (fiducial, machine)))
            .collect::<Vec<_>>();
        if fiducials.len() < 2 {
            return None;
        }
        let expected = fiducials
            .iter()
            .map(|(fiducial, _)| (*fiducial).clone())
            .collect::<Vec<_>>();
        let observed = fiducials
            .iter()
            .map(|(_, machine)| crate::identifier::FiducialObservation {
                image_position: *machine,
                confidence: 1.0,
            })
            .collect::<Vec<_>>();
        crate::identifier::compute_board_pose(&expected, &observed)
    }

    /// Return the machine/world target for a placement in local board space.
    pub fn placement_world_position(
        &self,
        index: usize,
        pose: Option<crate::geometry::Pose>,
    ) -> Option<shared_types::Point> {
        let placement = self.placements.get(index)?;
        Some(
            pose.or_else(|| self.board_pose())
                .map(|pose| pose.transform(placement.design_position))
                .unwrap_or(placement.design_position),
        )
    }

    /// Hydrate the job file into a `Runner` ready to drive.
    ///
    /// The runner's state machine is reset to the start of the job; the
    /// board pose must be set with `set_board_pose` after the loader
    /// detects fiducials (or from a manual entry).
    pub fn into_runner(self, machine: MachineContext) -> Result<Runner, String> {
        let head = NozzleHead::new(
            self.head
                .nozzles
                .into_iter()
                .map(|n| NozzleSlot {
                    id: n.id,
                    kind: n.kind,
                    offset_mm: n.offset_mm,
                    rotation_deg: 0.0,
                    vacuum: false,
                })
                .collect(),
        );
        let mut calibration = FeederCalibration::new();
        for e in machine.calibration {
            calibration.set(e.bank_id, e.part_id, e.pickup);
        }
        let rotation = FeederRotation::from_assignment(&self.feeder_assignment);
        Ok(Runner::with_setup(
            head,
            self.feeders,
            self.banks,
            calibration,
            machine.setup,
            machine.cameras,
            machine.config,
            Job::new(self.board_id, self.placements),
            rotation,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::CameraKind;
    use crate::geometry::{Pose, Size};
    use crate::nozzle::NozzleKind;
    use crate::parts::{Footprint, Part, Polarity};
    use crate::placement::PartObservation;
    use crate::runner::StepResult;
    use crate::state::PlacementStrategy;
    use shared_types::Point;

    fn dummy_part(id: u32, mpn: &str) -> Part {
        Part {
            id: PartId(id),
            mpn: mpn.into(),
            description: String::new(),
            footprint: Footprint {
                name: mpn.into(),
                centroid: Point::ORIGIN,
                size: Size {
                    width: 1.0,
                    height: 1.0,
                },
                pads: vec![],
                polarity: Polarity::None,
            },
            nozzle_type: vec![NozzleKind::Standard],
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        }
    }

    fn dummy_placement(p: Part, x: f64, y: f64) -> Placement {
        Placement {
            reference: String::new(),
            part: p,
            design_position: Point::new(x, y),
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        }
    }

    fn tiny_job() -> JobFile {
        JobFile {
            estimated_seconds: None,
            placement_columns: Vec::new(),
            board_mapping: None,
            board_id: "TINY".into(),
            track_width_mm: None,
            feeders: vec![FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: None,
            }],
            banks: vec![],
            head: HeadSpec {
                nozzles: vec![NozzleSpec {
                    id: 0,
                    kind: NozzleKind::Standard,
                    offset_mm: 0.0,
                }],
            },
            placements: vec![dummy_placement(dummy_part(1, "X"), 10.0, 20.0)],
            feeder_assignment: FeederAssignment::default(),
            fiducials: vec![],
        }
    }

    /// Machine-side settings matching `tiny_job`: the camera the runner
    /// needs to inspect its placement.
    fn tiny_machine() -> MachineContext {
        MachineContext {
            cameras: vec![Camera {
                id: 0,
                kind: CameraKind::FastVision,
                position: Point::new(0.0, 200.0),
                array_pitch_mm: None,
            }],
            config: MachineConfig::default(),
            setup: MachineSetup::new(),
            calibration: vec![],
        }
    }

    #[test]
    fn json_round_trip_preserves_assignments() {
        let original = tiny_job();
        let json = original.to_string().unwrap();
        let parsed = JobFile::from_str(&json).unwrap();
        assert_eq!(parsed.board_id, "TINY");
        assert_eq!(parsed.placements.len(), 1);
        assert_eq!(parsed.placements[0].part.mpn, "X");
        assert_eq!(parsed.placements[0].design_position.x, 10.0);
        assert_eq!(parsed.placements[0].design_position.y, 20.0);
        assert_eq!(parsed.feeders.len(), 1);
        assert_eq!(parsed.feeders[0].id, 0);
    }

    #[test]
    fn job_maps_local_placement_into_world_space() {
        let mut job = tiny_job();
        job.placements[0].design_position = Point::new(100.0, 100.0);
        job.fiducials = vec![
            crate::identifier::Fiducial {
                design_position: Point::ORIGIN,
                kind: crate::identifier::FiducialKind::Panel,
                machine_position: Some(Point::new(332.0, 158.0)),
                expected_position: None,
            },
            crate::identifier::Fiducial {
                design_position: Point::new(10.0, 0.0),
                kind: crate::identifier::FiducialKind::Panel,
                machine_position: Some(Point::new(332.0, 168.0)),
                expected_position: None,
            },
        ];
        let target = job.placement_world_position(0, None).unwrap();
        assert!((target.x - 232.0).abs() < 1e-9);
        assert!((target.y - 258.0).abs() < 1e-9);
    }

    #[test]
    fn job_ignores_local_fiducials_when_recovering_global_pose() {
        let mut job = tiny_job();
        job.fiducials = vec![
            crate::identifier::Fiducial {
                design_position: Point::ORIGIN,
                kind: crate::identifier::FiducialKind::Local,
                machine_position: Some(Point::new(999.0, 999.0)),
                expected_position: None,
            },
            crate::identifier::Fiducial {
                design_position: Point::ORIGIN,
                kind: crate::identifier::FiducialKind::Panel,
                machine_position: Some(Point::new(10.0, 20.0)),
                expected_position: None,
            },
            crate::identifier::Fiducial {
                design_position: Point::new(10.0, 0.0),
                kind: crate::identifier::FiducialKind::Panel,
                machine_position: Some(Point::new(20.0, 20.0)),
                expected_position: None,
            },
        ];
        assert_eq!(job.board_pose().unwrap().position, Point::new(10.0, 20.0));
    }

    #[test]
    fn into_runner_produces_runnable_job() {
        let mut runner = tiny_job().into_runner(tiny_machine()).unwrap();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut actions = 0;
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => actions += 1,
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        assert!(actions > 0, "expected at least one action emitted");
        assert!(runner.job().placements[0].placed);
    }

    #[test]
    fn machine_context_round_trips_through_json() {
        // Feeder setup and calibration used to ride along inside the job
        // file; they are machine settings now, serialized as one context.
        let mut machine = MachineContext::default();
        machine.setup.set_kind(0, crate::feeder::FeederKind::Tape(8));
        machine.cameras = tiny_machine().cameras;
        machine.calibration = vec![
            CalibrationEntry {
                bank_id: 0,
                part_id: PartId(1),
                pickup: PartPickup {
                    offset: Point::new(1.0, 0.0),
                    rotation_deg: 0.0,
                    pickup_z_mm: 0.4,
                },
            },
            CalibrationEntry {
                bank_id: 0,
                part_id: PartId(2),
                pickup: PartPickup {
                    offset: Point::new(0.0, 2.0),
                    rotation_deg: 0.0,
                    pickup_z_mm: 0.7,
                },
            },
        ];
        let json = serde_json::to_string(&machine).unwrap();
        let parsed: MachineContext = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.cameras.len(), 1);
        assert_eq!(parsed.cameras[0].position.y, 200.0);
        assert_eq!(parsed.calibration.len(), 2);
        assert_eq!(parsed.calibration[0].bank_id, 0);
        assert_eq!(parsed.calibration[0].part_id, PartId(1));
        assert_eq!(parsed.calibration[1].pickup.offset.y, 2.0);
        assert!(parsed.setup.kind(0).is_some());
    }
}
