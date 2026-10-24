//! Helpers that turn a `Placement` into concrete machine targets.
//!
//! The runner uses these to compute the XY moves it needs to issue.
//! Kept as free functions so they are easy to unit-test.

use crate::feeder::{FeederBank, FeederCalibration, FeederSlot};
use crate::geometry::Pose;
use crate::nozzle::NozzleHead;
use crate::state::Placement;
use shared_types::*;

/// What the up-camera saw of a part held by a nozzle.
#[derive(Debug, Clone, Copy)]
pub struct PartObservation {
    /// Rotation error in degrees: how far the part is rotated from the
    /// design orientation. Positive = part is CCW from intended.
    pub rotation_error_deg: f64,
    /// Centroid of the held part relative to the nozzle tip, in machine
    /// coordinates. `(0, 0)` means perfectly centred. `centroid_offset =
    /// (1.0, 2.0)` means the part's centroid sits 1 mm to the right and
    /// 2 mm down (in the machine frame) of where the tip is — a 1 mm
    /// right / 2 mm down pickup error.
    ///
    /// The runner feeds this through to placement targeting so the head
    /// compensates when landing the part on the board.
    pub centroid_offset: Point,
    /// Whether the part is present at all (vacuum sensor / vision check).
    pub present: bool,
    /// Whether the part's centroid is within the placement tolerance.
    pub within_tolerance: bool,
}

impl PartObservation {
    pub fn ok() -> Self {
        Self {
            rotation_error_deg: 0.0,
            centroid_offset: Point::ORIGIN,
            present: true,
            within_tolerance: true,
        }
    }
}

/// Pickup geometry for one specific (bank, part). Resolves calibration
/// overrides onto the part's defaults.
pub fn pickup_for(
    bank_id: u8,
    placement: &Placement,
    calibration: &FeederCalibration,
    default_pickup_z_mm: f64,
) -> crate::feeder::PartPickup {
    calibration.get_or(
        bank_id,
        placement.part.id,
        &placement.part,
        default_pickup_z_mm,
    )
}

/// Head-target position to bring the active nozzle's *tip* over the slot.
/// Doesn't account for the part's pickup offset inside the pocket — for
/// that, use `feeder_pickup_target`.
pub fn feeder_target(
    placement: &Placement,
    feeders: &[FeederSlot],
    head: &NozzleHead,
) -> Option<Point> {
    feeders
        .iter()
        .find(|s| s.id == placement.feeder_slot)
        .map(|s| head.head_position_for_tip(placement.nozzle, s.position))
}

/// Head-target position to put the nozzle tip directly above the part
/// inside the pocket, taking the bank's rotation and the (bank, part)
/// pickup offset into account.
///
/// `tip - (slot_pos + R(bank_rot) * pickup.offset)` gives the head
/// position that puts the tip on the part's pocket reference point.
///
/// Falls back to the slot's nominal position (no bank-rotation / no
/// offset) if no bank entry is found, so callers without banks set up
/// still work.
pub fn feeder_pickup_target(
    placement: &Placement,
    feeders: &[FeederSlot],
    banks: &[FeederBank],
    head: &NozzleHead,
    calibration: &FeederCalibration,
    default_pickup_z_mm: f64,
) -> Option<Point> {
    let slot = feeders.iter().find(|s| s.id == placement.feeder_slot)?;
    let bank_rot = banks
        .iter()
        .find(|b| b.id == slot.bank_id)
        .map(|b| b.rotation_deg)
        .unwrap_or(0.0);
    let pickup = pickup_for(slot.bank_id, placement, calibration, default_pickup_z_mm);
    let (s, c) = bank_rot.to_radians().sin_cos();
    let world_offset = Point::new(
        pickup.offset.x * c - pickup.offset.y * s,
        pickup.offset.x * s + pickup.offset.y * c,
    );
    let pocket = Point::new(
        slot.position.x + world_offset.x,
        slot.position.y + world_offset.y,
    );
    Some(head.head_position_for_tip(placement.nozzle, pocket))
}

/// Z height the nozzle should descend to when picking this part from
/// this feeder.
pub fn feeder_pickup_z(
    placement: &Placement,
    feeders: &[FeederSlot],
    calibration: &FeederCalibration,
    default_pickup_z_mm: f64,
) -> Option<f64> {
    let slot = feeders.iter().find(|s| s.id == placement.feeder_slot)?;
    Some(pickup_for(slot.bank_id, placement, calibration, default_pickup_z_mm).pickup_z_mm)
}

/// Head-target position to land the part centroid on the design position
/// on the board, taking the current nozzle rotation and any
/// camera-observed centroid offset into account.
///
/// `extra_centroid_offset` is the offset the camera saw, rotated by
/// `-rotation_error_deg` so it lives in the part's body frame at the
/// current rotation. The runner supplies this; for bare-bones use pass
/// `Point::ORIGIN`.
///
/// Caller is responsible for making sure the placement's physical nozzle ID
/// exists on `head`.
/// holds the rotation the part will be at when placed (i.e. after the
/// camera-correction rotation has been applied).
pub fn place_target(
    placement: &Placement,
    head: &NozzleHead,
    board_pose: Pose,
    extra_centroid_offset: Point,
) -> Point {
    let world = board_pose.transform(placement.design_position);
    let local = Point::new(
        placement.part.pickup_offset.x + extra_centroid_offset.x,
        placement.part.pickup_offset.y + extra_centroid_offset.y,
    );
    head.head_position_for_part_point(placement.nozzle, local, world)
}

/// Required nozzle rotation, in world frame, for a placement to land at
/// the right angle on the board.
pub fn required_nozzle_rotation(placement: &Placement, board_pose: Pose) -> f64 {
    board_pose.angle_deg + placement.design_rotation_deg
}

/// Shortest signed rotation from `from` to `to` (degrees, in [-180, 180]).
pub fn shortest_rotation(from: f64, to: f64) -> f64 {
    let delta = (to - from).rem_euclid(360.0);
    if delta > 180.0 { delta - 360.0 } else { delta }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NozzleKind;
    use crate::feeder::PartId;
    use crate::geometry::Size;
    use crate::nozzle::default_head;
    use crate::parts::{Footprint, Part, Polarity};
    use crate::state::Placement;

    fn dummy_placement(feeder_slot: u8, nozzle: u8, design: Point) -> Placement {
        Placement {
            reference: String::new(),
            part: Part {
                id: PartId(1),
                mpn: "R1".into(),
                description: String::new(),
                footprint: Footprint {
                    name: "R1".into(),
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
            },
            design_position: design,
            design_rotation_deg: 0.0,
            feeder_slot,
            nozzle,
            placed: false,
            should_place: true,
            strategy: crate::state::PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }
    }

    #[test]
    fn shortest_rotation_picks_short_arc() {
        assert!((shortest_rotation(0.0, 30.0) - 30.0).abs() < 1e-9);
        assert!((shortest_rotation(0.0, 350.0) + 10.0).abs() < 1e-9);
        assert!((shortest_rotation(350.0, 10.0) - 20.0).abs() < 1e-9);
    }

    #[test]
    fn feeder_target_uses_head_centre_offset() {
        // A 4-nozzle head has nozzle 0 at offset -30 mm. To put nozzle 0
        // over a feeder at (100, 50), the head reference sits at (130, 50).
        let head = default_head(4);
        let feeders = vec![FeederSlot {
            id: 5,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: None,
        }];
        let p = dummy_placement(5, 0, Point::ORIGIN);
        let target = feeder_target(&p, &feeders, &head).unwrap();
        assert_eq!(target, Point::new(130.0, 50.0));
    }

    #[test]
    fn feeder_pickup_target_applies_bank_rotation() {
        // Bank 0 is rotated +90°. A pickup offset of (1, 0) in the bank's
        // frame becomes (0, 1) in the machine frame.
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 5,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: None,
        }];
        let banks = vec![FeederBank {
            id: 0,
            rotation_deg: 90.0,
        }];
        let mut cal = FeederCalibration::new();
        cal.set(
            0,
            PartId(1),
            crate::feeder::PartPickup {
                offset: Point::new(1.0, 0.0),
                rotation_deg: 0.0,
                pickup_z_mm: 0.5,
            },
        );
        let p = dummy_placement(5, 0, Point::ORIGIN);
        let target = feeder_pickup_target(&p, &feeders, &banks, &head, &cal, 0.5).unwrap();
        // Slot at (100, 50) + rotated offset (0, 1) = (100, 51).
        assert!((target.x - 100.0).abs() < 1e-9, "x={}", target.x);
        assert!((target.y - 51.0).abs() < 1e-9, "y={}", target.y);
    }

    #[test]
    fn feeder_pickup_z_uses_calibration_override() {
        let feeders = vec![FeederSlot {
            id: 5,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: None,
        }];
        let mut cal = FeederCalibration::new();
        cal.set(
            0,
            PartId(1),
            crate::feeder::PartPickup {
                offset: Point::ORIGIN,
                rotation_deg: 0.0,
                pickup_z_mm: 1.7,
            },
        );
        let p = dummy_placement(5, 0, Point::ORIGIN);
        let z = feeder_pickup_z(&p, &feeders, &cal, 0.5).unwrap();
        assert!((z - 1.7).abs() < 1e-9);
    }

    #[test]
    fn feeder_pickup_z_falls_back_to_default() {
        let feeders = vec![FeederSlot {
            id: 5,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: None,
        }];
        let cal = FeederCalibration::new();
        let p = dummy_placement(5, 0, Point::ORIGIN);
        let z = feeder_pickup_z(&p, &feeders, &cal, 0.5).unwrap();
        assert!((z - 0.5).abs() < 1e-9);
    }

    #[test]
    fn place_target_uses_board_pose() {
        let head = default_head(1);
        let pose = Pose::new(100.0, 200.0, 0.0);
        let p = dummy_placement(0, 0, Point::new(10.0, 5.0));
        let target = place_target(&p, &head, pose, Point::ORIGIN);
        assert_eq!(target, Point::new(110.0, 205.0));
    }

    #[test]
    fn local_board_position_100_100_maps_into_world_coordinates() {
        let head = default_head(1);
        let pose = Pose::new(332.0, 158.0, 10.0);
        let p = dummy_placement(0, 0, Point::new(100.0, 100.0));
        let target = place_target(&p, &head, pose, Point::ORIGIN);
        let expected = pose.transform(Point::new(100.0, 100.0));
        assert!((target.x - expected.x).abs() < 1e-9);
        assert!((target.y - expected.y).abs() < 1e-9);
        assert!((target.x - 100.0).abs() > 1.0);
        assert!((target.y - 100.0).abs() > 1.0);
    }

    #[test]
    fn local_position_is_not_saved_as_world_position() {
        let pose = Pose::new(332.0, 158.0, 10.0);
        let design = Point::new(100.0, 100.0);
        let world = pose.transform(design);
        let recovered = pose.inverse_transform(world);
        assert!((recovered.x - design.x).abs() < 1e-9);
        assert!((recovered.y - design.y).abs() < 1e-9);
    }

    #[test]
    fn place_target_applies_centroid_offset() {
        // 4-nozzle head, nozzle 0 at X=-30. Part design at (10, 5); board
        // pose (0, 0, 0). Centroid is observed 1 mm right and 2 mm down
        // on the tip — head must shift -1 mm X and -2 mm Y to compensate.
        let head = default_head(4);
        let pose = Pose::new(0.0, 0.0, 0.0);
        let mut p = dummy_placement(0, 0, Point::new(10.0, 5.0));
        p.part.pickup_offset = Point::ORIGIN;
        let target = place_target(&p, &head, pose, Point::new(1.0, 2.0));
        // Tip lands at design (10, 5). With nozzle offset (-30, 0), head
        // reference = (40, 5). Centroid offset shifts the centroid by
        // (1, 2) — to put centroid at (10, 5), tip needs to be at (9, 3),
        // so head reference at (39, 3).
        assert!((target.x - 39.0).abs() < 1e-9, "x={}", target.x);
        assert!((target.y - 3.0).abs() < 1e-9, "y={}", target.y);
    }

    #[test]
    fn required_rotation_includes_board_rotation() {
        let pose = Pose::new(0.0, 0.0, 45.0);
        let p = dummy_placement(0, 0, Point::ORIGIN);
        assert!((required_nozzle_rotation(&p, pose) - 45.0).abs() < 1e-9);
        let pose = Pose::new(0.0, 0.0, -30.0);
        let mut p = dummy_placement(0, 0, Point::ORIGIN);
        p.design_rotation_deg = 15.0;
        assert!((required_nozzle_rotation(&p, pose) - -15.0).abs() < 1e-9);
    }

    #[test]
    fn place_target_rotates_design_with_board_pose() {
        // Board is rotated +90° CCW and translated to (100, 200). A design
        // point at (10, 0) should land at world (100, 210).
        let head = default_head(1);
        let pose = Pose::new(100.0, 200.0, 90.0);
        let p = dummy_placement(0, 0, Point::new(10.0, 0.0));
        let target = place_target(&p, &head, pose, Point::ORIGIN);
        assert!((target.x - 100.0).abs() < 1e-9, "x={}", target.x);
        assert!((target.y - 210.0).abs() < 1e-9, "y={}", target.y);
    }
}
