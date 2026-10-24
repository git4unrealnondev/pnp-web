//! Regression check: with four N501 nozzles on the head and a part that
//! allows N501, the planner must spread placements across the nozzles
//! instead of parking everything on nozzle 1.

use pnp_logic::config::MachineSetup;
use pnp_logic::feeder::{FeederBank, FeederKind, FeederSlot, PartId};
use pnp_logic::nozzle::{NozzleHead, NozzleKind, NozzleSlot};
use pnp_logic::optimizer::{FeederPlacementMode, PlacementNeed};
use pnp_logic::plan::{plan_job_with_mode, PlanSearchMode};
use pnp_logic::state::PlacementStrategy;
use pnp_logic::CameraKind;
use pnp_logic::MachineConfig;
use shared_types::Point;

#[test]
fn four_n501_nozzles_spread_across_all_four() {
    let part = pnp_logic::Part {
        id: PartId(2),
        mpn: "1nf".into(),
        description: String::new(),
        footprint: pnp_logic::Footprint {
            name: "0603".into(),
            centroid: Point::ORIGIN,
            size: pnp_logic::Size {
                width: 1.524,
                height: 0.762,
            },
            pads: vec![],
            polarity: pnp_logic::Polarity::None,
        },
        nozzle_type: vec![NozzleKind::N501],
        pickup_rotation_deg: 0.0,
        pickup_offset: Point::ORIGIN,
        camera: None,
    };

    let head = NozzleHead::new(
        (0..4)
            .map(|index| NozzleSlot {
                id: index + 1,
                kind: NozzleKind::N501,
                offset_mm: -30.0 + 20.0 * index as f64,
                rotation_deg: 0.0,
                vacuum: false,
            })
            .collect(),
    );
    assert_eq!(head.nozzles.len(), 4, "test head must have four nozzles");

    let feeders: Vec<FeederSlot> = (0..8)
        .map(|index| FeederSlot {
            id: index,
            bank_id: (index / 4) as u8,
            position: Point::new(-75.0 + 50.0 * (index % 4) as f64, 400.0 + 30.0 * (index / 4) as f64),
            loaded: if index == 1 { Some(PartId(2)) } else { None },
        })
        .collect();
    let banks = vec![FeederBank { id: 0, rotation_deg: 0.0 }, FeederBank { id: 1, rotation_deg: 0.0 }];
    let mut setup = MachineSetup::default();
    for feeder in &feeders {
        setup.set_kind(feeder.id, FeederKind::Tape(16));
        if let Some(loaded) = feeder.loaded {
            setup.set_loaded(feeder.id, loaded);
        }
    }

    let needs: Vec<PlacementNeed> = (0..9)
        .map(|index| PlacementNeed {
            reference: String::new(),
            part: part.clone(),
            design_position: Point::new(index as f64 * 2.0, index as f64 * 2.0),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        })
        .collect();

    let planned = plan_job_with_mode(
        "test",
        needs,
        &head,
        &feeders,
        &setup,
        Point::new(8.0, 8.0),
        &MachineConfig::default(),
        FeederPlacementMode::Movable,
        PlanSearchMode::LocalSearch,
    );

    let nozzles: std::collections::HashSet<u8> =
        planned.job.placements.iter().map(|p| p.nozzle).collect();
    assert_eq!(
        planned.job.placements.len(),
        9,
        "all placements must be planned"
    );
    assert!(
        nozzles.len() > 1,
        "planner parked every placement on one nozzle: {nozzles:?}"
    );
    assert!(
        !planned.feeder_assignment.slot_for_part.is_empty(),
        "feeder assignment is empty — optimiser did not allocate feeders"
    );
}
