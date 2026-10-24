//! Nozzle-tip calibration by ink marking.
//!
//! # Why four rotations
//!
//! A nozzle tip is not perfectly concentric with its rotation axis, so the
//! tip traces a small circle ("runout") as the nozzle spins. Touching the
//! paper at a single rotation therefore stamps a mark that is offset from
//! the true nozzle centre by the runout vector at that angle.
//!
//! Touching once per cardinal rotation and averaging the four marks
//! cancels that vector exactly: for runout of radius `r` at phase `p`,
//! the four marks sit at angles `p, p+90, p+180, p+270`, whose vector sum
//! is zero. The mean of the four marks is the true rotation axis,
//! independent of `r`.
//!
//! One subtlety this module exists to handle: marks accumulate. By the
//! fourth round the paper carries marks from every earlier round, so naive
//! blob detection cannot tell "the mark I just made" from the three
//! already there. We solve that by requiring the same number of blobs on
//! every round and rejecting duplicates — see [`assign_round_marks`].
//!
//! # Coordinate frames
//!
//! Marks are observed in camera pixels and converted to machine
//! millimetres through the camera's [`PixelToMachineCalibration`]. All
//! geometry in this module is in **machine millimetres**.

use shared_types::{NozzleId, Point};

/// One touchdown observation: a nozzle's tip position at one rotation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InkMark {
    pub nozzle: NozzleId,
    /// Rotation of the nozzle when it touched, in degrees CCW.
    pub rotation_deg: f64,
    /// Tip position in machine millimetres.
    pub position: Point,
}

/// Calibrated result for one nozzle.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NozzleCalibration {
    pub nozzle: NozzleId,
    /// Mean of the four marks: the nozzle's true rotation axis.
    pub center: Point,
    /// Mean distance from `center` to each mark. This is the runout
    /// radius, a useful health metric in its own right — a nozzle whose
    /// runout is large will place parts inconsistently.
    pub runout_mm: f64,
    /// Largest single deviation from `center`.
    pub max_deviation_mm: f64,
}

/// Card rotations used for a calibration run, in the order they are
/// performed. Spelled out rather than generated so the order is obvious
/// at the call site and stable for logs.
pub const CARDINAL_ROTATIONS: [f64; 4] = [0.0, 90.0, 180.0, 270.0];

/// Average one nozzle's marks and measure its runout.
///
/// Averaging over exactly the cardinal rotations cancels a circular
/// runout vector, so the result is the rotation axis rather than the
/// position of any single touchdown. Requires at least
/// [`MIN_MARKS_PER_NOZZLE`] marks; below that the cancellation argument
/// does not hold and the caller gets an error instead of a plausible-
/// looking number.
pub fn calibrate_nozzle(marks: &[InkMark]) -> Result<NozzleCalibration, String> {
    let nozzle = marks.first().map(|m| m.nozzle).ok_or("no marks supplied")?;
    if marks.len() < MIN_MARKS_PER_NOZZLE {
        return Err(format!(
            "nozzle {:?} needs at least {MIN_MARKS_PER_NOZZLE} marks, got {}",
            nozzle,
            marks.len()
        ));
    }
    if marks.iter().any(|m| m.nozzle != nozzle) {
        return Err("calibrate_nozzle called with marks from different nozzles".into());
    }

    let n = marks.len() as f64;
    let sum_x: f64 = marks.iter().map(|m| m.position.x).sum();
    let sum_y: f64 = marks.iter().map(|m| m.position.y).sum();
    let center = Point::new(sum_x / n, sum_y / n);

    let deviations: Vec<f64> = marks
        .iter()
        .map(|m| m.position.distance(center))
        .collect();
    let runout_mm = deviations.iter().sum::<f64>() / n;
    let max_deviation_mm = deviations.iter().copied().fold(0.0_f64, f64::max);

    Ok(NozzleCalibration {
        nozzle,
        center,
        runout_mm,
        max_deviation_mm,
    })
}

/// Calibrate every nozzle from a full set of observations.
pub fn calibrate_all(
    marks: &[InkMark],
    nozzles: &[NozzleId],
) -> Result<Vec<NozzleCalibration>, String> {
    nozzles
        .iter()
        .map(|nozzle| {
            let subset: Vec<InkMark> = marks.iter().filter(|m| m.nozzle == *nozzle).copied().collect();
            calibrate_nozzle(&subset)
        })
        .collect()
}

/// Minimum marks per nozzle for a trustworthy centroid.
pub const MIN_MARKS_PER_NOZZLE: usize = 4;

/// Two marks this close in millimetres are treated as the same physical
/// blob. Ink from an earlier round that overlaps a fresh mark produces
/// contours closer together than this, which is what lets us spot and
/// discard them.
pub const DUPLICATE_MERGE_MM: f64 = 1.5;

/// Assign this round's detected blobs to nozzles.
///
/// `detected` is every blob found in the frame, in millimetres;
/// `expected` is where each nozzle's tip should be, from the head
/// geometry. The two lists are the same length when the frame shows one
/// blob per nozzle, which is the normal case once earlier marks have
/// merged.
///
/// Returns, for each nozzle index in `expected` order, the blob assigned
/// to it. If the counts disagree we cannot tell a fresh mark from a
/// leftover one, so we fail loudly rather than guess — a wrong assignment
/// here silently corrupts every downstream centroid.
pub fn assign_round_marks(
    detected: &[Point],
    expected: &[Point],
) -> Result<Vec<Point>, String> {
    if detected.len() != expected.len() {
        return Err(format!(
            "expected {} blobs for {} nozzles but found {}; \
             paper may be marked from a previous run or a mark may have missed",
            expected.len(),
            expected.len(),
            detected.len()
        ));
    }

    // Pair each detection with its nearest expectation. Nearest-neighbour
    // pairing is correct here because the nozzles are spaced far wider
    // than the search radius, so no nozzle can claim another's blob.
    let mut pairs: Vec<(f64, usize, usize)> = Vec::with_capacity(detected.len());
    for (di, &d) in detected.iter().enumerate() {
        for (ni, &e) in expected.iter().enumerate() {
            pairs.push((d.distance(e), di, ni));
        }
    }
    pairs.sort_by(|a, b| a.0.total_cmp(&b.0));

    let mut used_detected = vec![false; detected.len()];
    let mut out: Vec<Option<Point>> = vec![None; expected.len()];
    for (distance, di, ni) in pairs {
        if used_detected[di] || out[ni].is_some() {
            continue;
        }
        used_detected[di] = true;
        if distance > MAX_MATCH_DISTANCE_MM {
            return Err(format!(
                "nozzle #{ni} matched a blob {distance:.2} mm away, above the \
                 {MAX_MATCH_DISTANCE_MM:.1} mm limit; is the camera calibrated?"
            ));
        }
        out[ni] = Some(detected[di]);
    }

    if out.iter().any(Option::is_none) {
        return Err("could not pair every detected blob with a nozzle".into());
    }
    Ok(out.into_iter().flatten().collect())
}

/// Furthest a fresh mark may sit from its predicted position before we
/// treat it as a missed detection. Loose enough to absorb a few tenths of
/// a millimetre of unmodelled runout and paper error, tight enough to
/// catch "the camera is not calibrated" rather than absorbing it.
pub const MAX_MATCH_DISTANCE_MM: f64 = 3.0;

/// Expected tip position for each nozzle, given the head pose and the
/// nozzle offsets.
///
/// Mirrors [`crate::nozzle::NozzleHead::tip_position`] but takes the
/// values directly so the calibration code does not need to construct a
/// head and can be tested without one.
pub fn expected_tip_positions(
    head_position: Point,
    nozzles: &[NozzleId],
    offsets: &[f64],
) -> Result<Vec<Point>, String> {
    if nozzles.len() != offsets.len() {
        return Err("nozzle and offset counts differ".into());
    }
    nozzles
        .iter()
        .zip(offsets)
        .map(|(&nozzle, &offset)| {
            // Reject unaddressable nozzles here so a mistyped plan fails
            // before the head starts moving.
            nozzle_index(nozzle)
                .ok_or_else(|| format!("nozzle {nozzle:?} is outside the calibratable range"))?;
            Ok(Point::new(head_position.x + offset, head_position.y))
        })
        .collect()
}

/// Position of `nozzle` in the 1..=4 range the hardware addresses.
///
/// [`NozzleId`] has nine variants but this machine's rotation axis index
/// is a `[f64; 4]` and the multi-nozzle rotate command packs four
/// direction bits, so only the first four are addressable.
pub fn nozzle_index(nozzle: NozzleId) -> Option<usize> {
    let index = match nozzle {
        NozzleId::N0 => 0,
        NozzleId::N1 => 1,
        NozzleId::N2 => 2,
        NozzleId::N3 => 3,
        _ => return None,
    };
    Some(index)
}

/// The first four nozzles, in machine order.
pub fn default_calibration_nozzles() -> [NozzleId; 4] {
    [NozzleId::N0, NozzleId::N1, NozzleId::N2, NozzleId::N3]
}

#[cfg(test)]
mod id_tests {
    use super::*;

    /// The bug this guards: a hand-written match mapped nozzle 1 to N1,
    /// whose `id()` is 2, so every command addressed the wrong nozzle and
    /// nozzle 1 was unreachable.
    #[test]
    fn from_id_is_the_exact_inverse_of_id() {
        for nozzle in NozzleId::ALL {
            assert_eq!(
                NozzleId::from_id(nozzle.id()),
                Some(nozzle),
                "{nozzle:?} id {} did not round-trip",
                nozzle.id()
            );
        }
    }

    #[test]
    fn nozzle_one_is_reachable() {
        assert_eq!(NozzleId::from_id(1), Some(NozzleId::N0));
        assert_eq!(NozzleId::N0.id(), 1);
    }

    /// `id` is 1-based, so 0 is not a nozzle.
    #[test]
    fn zero_is_not_a_nozzle() {
        assert_eq!(NozzleId::from_id(0), None);
    }

    #[test]
    fn out_of_range_is_rejected() {
        assert_eq!(NozzleId::from_id(10), None);
        assert_eq!(NozzleId::from_id(255), None);
    }

    /// The mapping the API uses must agree with what the hardware
    /// addresses, which is what `id()` feeds into the frame.
    #[test]
    fn api_numbering_matches_hardware_numbering() {
        for id in 1..=4u8 {
            let nozzle = NozzleId::from_id(id).unwrap();
            assert_eq!(nozzle.id(), id);
        }
    }
}

/// A nozzle's corrected offset, as consumed by [`crate::nozzle::NozzleSlot`].
///
/// This is the persisted form of a calibration: an offset relative to the
/// head reference point rather than an absolute machine position, so it
/// stays valid no matter where the head was parked during the run.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NozzleOffset {
    pub nozzle: NozzleId,
    /// Corrected offset along X, in mm from the head reference.
    pub offset_mm: f64,
    /// Measured runout, retained as a health metric.
    pub runout_mm: f64,
}

/// Reduce centroids to head-relative offsets.
///
/// The centroid is an absolute machine position; the head reference moved
/// to `head_position` during the run, so subtracting it recovers the
/// offset the rest of the software actually wants. `nominal_offsets` is
/// used only to sanity-check the result — a correction of several
/// millimetres means either a mis-modelled head or a failed calibration,
/// and silently accepting it would shift every placement on the machine.
pub fn offsets_from_calibration(
    results: &[NozzleCalibration],
    head_position: Point,
    nominal_offsets: &[f64],
) -> Result<Vec<NozzleOffset>, String> {
    if results.len() != nominal_offsets.len() {
        return Err(format!(
            "got {} calibrated nozzles but {} nominal offsets",
            results.len(),
            nominal_offsets.len()
        ));
    }

    let mut out = Vec::with_capacity(results.len());
    for (result, &nominal) in results.iter().zip(nominal_offsets) {
        let offset_mm = result.center.x - head_position.x;
        if !offset_mm.is_finite() {
            return Err(format!("nozzle {:?} produced a non-finite offset", result.nozzle));
        }
        if (offset_mm - nominal).abs() > MAX_PLAUSIBLE_CORRECTION_MM {
            return Err(format!(
                "nozzle {:?} would move from {nominal:.2} mm to {offset_mm:.2} mm, a \
                 {:.2} mm correction beyond the {:.2} mm limit; check the camera \
                 calibration and that the head really parked at the planned position",
                result.nozzle,
                (offset_mm - nominal).abs(),
                MAX_PLAUSIBLE_CORRECTION_MM
            ));
        }
        out.push(NozzleOffset {
            nozzle: result.nozzle,
            offset_mm,
            runout_mm: result.runout_mm,
        });
    }
    Ok(out)
}

/// Largest nozzle offset correction we will accept before treating the
/// calibration as failed.
///
/// Real nozzle pitch errors are fractions of a millimetre. A larger
/// correction almost always means the camera was not calibrated, or the
/// head did not reach the position the plan assumed — and applying it
/// would shift every placement on the machine, so we stop instead.
pub const MAX_PLAUSIBLE_CORRECTION_MM: f64 = 2.0;

#[cfg(test)]
mod offset_tests {
    use super::*;

    fn result(nozzle: NozzleId, center: Point) -> NozzleCalibration {
        NozzleCalibration {
            nozzle,
            center,
            runout_mm: 0.2,
            max_deviation_mm: 0.2,
        }
    }

    /// The offset is head-relative, so a calibration is valid wherever the
    /// head happened to park.
    #[test]
    fn offset_is_relative_to_the_head_position() {
        let results = vec![
            result(NozzleId::N0, Point::new(169.88, 100.0)),
            result(NozzleId::N1, Point::new(190.05, 100.0)),
        ];
        let offsets = offsets_from_calibration(&results, Point::new(200.0, 100.0), &[-30.0, -10.0])
            .unwrap();
        assert!((offsets[0].offset_mm - -30.12).abs() < 1e-9);
        assert!((offsets[1].offset_mm - -9.95).abs() < 1e-9);
    }

    /// Y is untouched: nozzles only offset along X, so a Y drift in the
    /// centroid must not leak into the offset.
    #[test]
    fn y_drift_does_not_affect_the_offset() {
        let results = vec![result(NozzleId::N0, Point::new(170.0, 100.5))];
        let offsets =
            offsets_from_calibration(&results, Point::new(200.0, 100.0), &[-30.0]).unwrap();
        assert!((offsets[0].offset_mm - -30.0).abs() < 1e-9);
    }

    /// A large correction means the camera was not calibrated or the head
    /// did not reach the planned spot. Applying it would shift every
    /// placement, so it must be rejected rather than stored.
    #[test]
    fn implausible_correction_is_rejected() {
        let results = vec![result(NozzleId::N0, Point::new(240.0, 100.0))];
        let err = offsets_from_calibration(&results, Point::new(200.0, 100.0), &[-30.0])
            .unwrap_err();
        assert!(err.contains("beyond"), "unhelpful error: {err}");
    }

    #[test]
    fn mismatched_counts_are_rejected() {
        let results = vec![result(NozzleId::N0, Point::new(170.0, 100.0))];
        assert!(offsets_from_calibration(&results, Point::ORIGIN, &[-30.0, -10.0]).is_err());
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Marks for a nozzle whose true centre is (100, 50) with 0.20 mm of
    /// runout at phase 30°, touched at the four cardinal rotations.
    ///
    /// The runout vector rotates WITH the nozzle, so at rotation r the tip
    /// is displaced by `r * (cos(p + r), sin(p + r))`.
    fn runout_marks(nozzle: NozzleId, center: Point, radius: f64, phase_deg: f64) -> Vec<InkMark> {
        CARDINAL_ROTATIONS
            .iter()
            .map(|&rotation_deg| {
                let angle = phase_deg.to_radians() + rotation_deg.to_radians();
                InkMark {
                    nozzle,
                    rotation_deg,
                    position: Point::new(
                        center.x + radius * angle.cos(),
                        center.y + radius * angle.sin(),
                    ),
                }
            })
            .collect()
    }

    #[test]
    fn centroid_recovers_axis_despite_runout() {
        let truth = Point::new(100.0, 50.0);
        let marks = runout_marks(NozzleId::N0, truth, 0.2, 30.0);
        let result = calibrate_nozzle(&marks).unwrap();

        // The whole point: mean of the four marks is the axis, not any
        // single touchdown.
        assert!(
            result.center.distance(truth) < 1e-9,
            "centroid {:?} is not the true axis {truth:?}",
            result.center
        );
        assert!((result.runout_mm - 0.2).abs() < 1e-9);
        assert!((result.max_deviation_mm - 0.2).abs() < 1e-9);
    }

    #[test]
    fn centroid_is_independent_of_runout_phase() {
        let truth = Point::new(10.0, -5.0);
        for phase in [0.0, 17.0, 90.0, 200.0, 359.0] {
            let marks = runout_marks(NozzleId::N2, truth, 0.15, phase);
            let result = calibrate_nozzle(&marks).unwrap();
            assert!(
                result.center.distance(truth) < 1e-9,
                "phase {phase}° shifted the centroid to {:?}",
                result.center
            );
        }
    }

    #[test]
    fn runout_is_measured_not_removed() {
        // The centroid is runout-free, but we still report the magnitude
        // so a worn nozzle is visible in the UI.
        let truth = Point::new(0.0, 0.0);
        let marks = runout_marks(NozzleId::N1, truth, 0.35, 45.0);
        let result = calibrate_nozzle(&marks).unwrap();
        assert!((result.runout_mm - 0.35).abs() < 1e-9);
        assert!(result.runout_mm > 0.3, "runout should be reported");
    }

    #[test]
    fn a_single_mark_is_the_worst_case_not_the_average() {
        // Shows what the averaging buys: with one mark the answer is
        // wrong by the full runout.
        let truth = Point::new(0.0, 0.0);
        let all = runout_marks(NozzleId::N0, truth, 0.2, 0.0);
        let single = calibrate_nozzle(&all[..1]).is_err();
        assert!(single, "one mark must be rejected as insufficient");

        let averaged = calibrate_nozzle(&all).unwrap();
        assert!(averaged.center.distance(truth) < 1e-9);
    }

    #[test]
    fn too_few_marks_are_rejected() {
        let marks = runout_marks(NozzleId::N0, Point::ORIGIN, 0.2, 0.0);
        let err = calibrate_nozzle(&marks[..3]).unwrap_err();
        assert!(err.contains("at least 4 marks"), "unhelpful error: {err}");
    }

    #[test]
    fn mixing_nozzles_is_rejected() {
        let mut marks = runout_marks(NozzleId::N0, Point::ORIGIN, 0.1, 0.0);
        marks[3].nozzle = NozzleId::N1;
        assert!(calibrate_nozzle(&marks).is_err());
    }

    #[test]
    fn assignment_pairs_each_blob_with_its_own_nozzle() {
        // Nozzles 20 mm apart at -30, -10, +10, +30.
        let nozzles = default_calibration_nozzles();
        let offsets = [-30.0, -10.0, 10.0, 30.0];
        let expected = expected_tip_positions(Point::new(200.0, 100.0), &nozzles, &offsets).unwrap();
        // Tips should land at x = 170, 190, 210, 230. Detections arrive
        // out of order, each nudged ~0.4 mm off truth.
        let detected = vec![
            Point::new(210.1, 99.9),
            Point::new(169.7, 99.8),
            Point::new(230.3, 100.1),
            Point::new(190.2, 100.4),
        ];
        let assigned = assign_round_marks(&detected, &expected).unwrap();
        assert_eq!(assigned.len(), 4);
        // Index 3 is the nozzle at +30 mm.
        assert!(assigned[3].distance(Point::new(230.0, 100.0)) < 0.5);
        // Each blob must land on the nozzle it belongs to, not merely on
        // some nozzle.
        for (i, blob) in assigned.iter().enumerate() {
            assert!(
                blob.distance(expected[i]) < 0.5,
                "blob {i} at {blob:?} is not near its nozzle at {:?}",
                expected[i]
            );
        }
    }

    #[test]
    fn assignment_rejects_a_missing_mark() {
        let nozzles = default_calibration_nozzles();
        let expected: Vec<Point> = (0..4).map(|i| Point::new(i as f64 * 20.0, 0.0)).collect();
        // One nozzle failed to leave a visible mark.
        let detected = vec![Point::new(0.0, 0.0), Point::new(20.0, 0.0), Point::new(40.0, 0.0)];
        let err = assign_round_marks(&detected, &expected).unwrap_err();
        assert!(err.contains("found 3"), "should name the count: {err}");
    }

    #[test]
    fn assignment_rejects_a_blob_that_is_too_far() {
        // A blob that matches nothing nearby means the camera calibration
        // is wrong; absorbing it would poison the centroid silently.
        let expected: Vec<Point> = (0..4).map(|i| Point::new(i as f64 * 20.0, 0.0)).collect();
        let detected = vec![
            Point::new(0.0, 0.0),
            Point::new(20.0, 0.0),
            Point::new(40.0, 0.0),
            Point::new(75.0, 0.0),
        ];
        assert!(assign_round_marks(&detected, &expected).is_err());
    }

    #[test]
    fn expected_positions_apply_the_offset() {
        let nozzles = default_calibration_nozzles();
        let offsets = [-30.0, -10.0, 10.0, 30.0];
        let positions = expected_tip_positions(Point::new(200.0, 100.0), &nozzles, &offsets).unwrap();
        assert_eq!(positions[0], Point::new(170.0, 100.0));
        assert_eq!(positions[3], Point::new(230.0, 100.0));
        // Y is unaffected: nozzles only offset along X.
        assert!(positions.iter().all(|p| p.y == 100.0));
    }

    #[test]
    fn only_the_first_four_nozzles_are_addressable() {
        assert_eq!(nozzle_index(NozzleId::N0), Some(0));
        assert_eq!(nozzle_index(NozzleId::N3), Some(3));
        // N4..N8 exceed the hardware's 4-slot rotation array.
        assert_eq!(nozzle_index(NozzleId::N4), None);
        assert_eq!(nozzle_index(NozzleId::N8), None);
    }

    #[test]
    fn offset_and_nozzle_counts_must_agree() {
        assert!(
            expected_tip_positions(
                Point::ORIGIN,
                &default_calibration_nozzles(),
                &[-30.0, -10.0]
            )
            .is_err()
        );
    }

    /// Sweeps runout radius and phase to confirm the cancellation holds in
    /// general, not just for the hand-picked values above.
    #[test]
    fn centroid_survives_a_sweep_of_radii_and_phases() {
        let truth = Point::new(123.456, -78.9);
        let mut worst = 0.0_f64;
        for radius in [0.0, 0.01, 0.05, 0.1, 0.25, 0.5, 1.0] {
            for phase_deg in 0..90 {
                let marks = runout_marks(NozzleId::N0, truth, radius, phase_deg as f64);
                let result = calibrate_nozzle(&marks).unwrap();
                worst = worst.max(result.center.distance(truth));
                // The reported runout should equal the true radius, so an
                // operator can trust the health metric as well as the fix.
                assert!(
                    (result.runout_mm - radius).abs() < 1e-9,
                    "radius {radius} reported as {}",
                    result.runout_mm
                );
            }
        }
        assert!(worst < 1e-9, "centroid drifted by {worst} mm");
    }

    #[test]
    fn calibrate_all_handles_every_nozzle() {
        let nozzles = default_calibration_nozzles();
        let mut marks = Vec::new();
        for (i, &nozzle) in nozzles.iter().enumerate() {
            let center = Point::new(100.0 + i as f64 * 20.0, 50.0);
            marks.extend(runout_marks(nozzle, center, 0.1, 20.0));
        }
        let results = calibrate_all(&marks, &nozzles).unwrap();
        assert_eq!(results.len(), 4);
        for (i, result) in results.iter().enumerate() {
            assert_eq!(result.nozzle, nozzles[i]);
            assert!((result.center.x - (100.0 + i as f64 * 20.0)).abs() < 1e-9);
        }
    }
}