//! Machine-readable identifiers and fiducial-based board localisation.
//!
//! Boards carry human- or machine-readable labels (text strings, QR codes,
//! DataMatrix codes) that name the board variant or job. Fiducials are
//! optical alignment marks placed on the board at known positions;
//! observing them with the up-camera lets the machine recover the board's
//! true pose on the machine, including its rotation, which is the basis
//! for placing every part in the right orientation.

use crate::geometry::Pose;
use shared_types::*;

/// Something the up-camera has decoded off the board itself.
#[derive(Debug, Clone)]
pub enum BoardId {
    Text(String),
    QrCode(String),
    DataMatrix(String),
}

/// A fiducial mark in the board design.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Fiducial {
    pub design_position: Point,
    pub kind: FiducialKind,
    #[serde(default)]
    pub machine_position: Option<Point>,
    /// Rough machine position saved by the operator before the first
    /// fiducial run: the place the camera should be moved to before the
    /// finder refines it into `machine_position`.
    #[serde(default)]
    pub expected_position: Option<Point>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum FiducialKind {
    /// Global fiducial: used to recover the overall board pose.
    Panel,
    /// Local fiducial: used to refine placement in a specific area.
    Local,
}

/// One detected fiducial in the camera image.
#[derive(Debug, Clone)]
pub struct FiducialObservation {
    pub image_position: Point,
    /// Detection confidence, 0..=1. Observations below threshold should
    /// be rejected before passing to `compute_board_pose`.
    pub confidence: f64,
}

/// Recover the board's pose on the machine from fiducial observations.
///
/// Bare-bones rigid 2D similarity fit from two correspondences. Real
/// implementations would use a least-squares fit over N≥3 points and
/// detect / reject outliers (RANSAC).
///
/// Returns `None` if inputs are inconsistent (different lengths, fewer
/// than two fiducials, degenerate geometry).
pub fn compute_board_pose(expected: &[Fiducial], observed: &[FiducialObservation]) -> Option<Pose> {
    if expected.len() < 2 || expected.len() != observed.len() {
        return None;
    }
    let a0 = expected[0].design_position;
    let a1 = expected[1].design_position;
    let b0 = observed[0].image_position;
    let b1 = observed[1].image_position;

    let da = a1.x - a0.x;
    let db = a1.y - a0.y;
    let ea = b1.x - b0.x;
    let eb = b1.y - b0.y;

    let len_a = (da * da + db * db).sqrt();
    let len_b = (ea * ea + eb * eb).sqrt();
    if len_a < 1e-9 || len_b < 1e-9 {
        return None;
    }

    let cos = (da * ea + db * eb) / (len_a * len_b);
    let sin = (da * eb - db * ea) / (len_a * len_b);
    let angle = sin.atan2(cos).to_degrees();

    let tx = b0.x - (a0.x * cos - a0.y * sin);
    let ty = b0.y - (a0.x * sin + a0.y * cos);
    Some(Pose::new(tx, ty, angle))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn identity_pose_when_observations_match_design() {
        let expected = vec![
            Fiducial {
                design_position: Point::new(0.0, 0.0),
                kind: FiducialKind::Panel,
                machine_position: None,
                expected_position: None,
            },
            Fiducial {
                design_position: Point::new(10.0, 0.0),
                kind: FiducialKind::Panel,
                machine_position: None,
                expected_position: None,
            },
        ];
        let observed = vec![
            FiducialObservation {
                image_position: Point::new(0.0, 0.0),
                confidence: 1.0,
            },
            FiducialObservation {
                image_position: Point::new(10.0, 0.0),
                confidence: 1.0,
            },
        ];
        let pose = compute_board_pose(&expected, &observed).unwrap();
        assert_eq!(pose.position, Point::new(0.0, 0.0));
        assert_eq!(pose.angle_deg, 0.0);
    }

    #[test]
    fn recovered_pose_transforms_design_to_observed() {
        // Board is rotated 90 CCW and translated to (100, 200).
        let expected = vec![
            Fiducial {
                design_position: Point::new(0.0, 0.0),
                kind: FiducialKind::Panel,
                machine_position: None,
                expected_position: None,
            },
            Fiducial {
                design_position: Point::new(10.0, 0.0),
                kind: FiducialKind::Panel,
                machine_position: None,
                expected_position: None,
            },
        ];
        let observed = vec![
            FiducialObservation {
                image_position: Point::new(100.0, 200.0),
                confidence: 1.0,
            },
            FiducialObservation {
                image_position: Point::new(100.0, 210.0),
                confidence: 1.0,
            },
        ];
        let pose = compute_board_pose(&expected, &observed).unwrap();
        let world = pose.transform(Point::new(5.0, 3.0));
        // (5, 3) rotated 90 CCW -> (-3, 5), translated -> (97, 205)
        assert!((world.x - 97.0).abs() < 1e-9, "x was {}", world.x);
        assert!((world.y - 205.0).abs() < 1e-9, "y was {}", world.y);
    }
}
