//! Core 2D geometry for machine coordinates.
//!
//! - Positions are millimetres in the machine's XY frame. Origin at the
//!   home position, +X to the right, +Y away from the operator, +Z down.
//! - Angles are degrees, CCW positive, applied to parts (not to the gantry).

use shared_types::*;

/// 2D pose: position plus rotation angle.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Pose {
    pub position: Point,
    pub angle_deg: f64,
}

impl Pose {
    pub const fn new(x: f64, y: f64, angle_deg: f64) -> Self {
        Self {
            position: Point::new(x, y),
            angle_deg,
        }
    }

    /// Map a point from this pose's local frame into world coordinates.
    ///
    /// Equivalent to: `R(angle) * local + position`.
    pub fn transform(&self, local: Point) -> Point {
        let (s, c) = self.angle_deg.to_radians().sin_cos();
        Point::new(
            self.position.x + local.x * c - local.y * s,
            self.position.y + local.x * s + local.y * c,
        )
    }

    /// Map a machine/world point back into the design frame.
    pub fn inverse_transform(&self, world: Point) -> Point {
        let translated = world - self.position;
        let (s, c) = self.angle_deg.to_radians().sin_cos();
        Point::new(
            translated.x * c + translated.y * s,
            -translated.x * s + translated.y * c,
        )
    }
}

/// Bounding-box size.
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Size {
    pub width: f64,
    pub height: f64,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn pose_rotates_local_point() {
        let pose = Pose::new(10.0, 20.0, 90.0);
        assert_eq!(pose.transform(Point::new(1.0, 0.0)), Point::new(10.0, 21.0));
        assert_eq!(pose.transform(Point::new(0.0, 1.0)), Point::new(9.0, 20.0));
    }

    #[test]
    fn pose_transform_round_trips_through_inverse() {
        let pose = Pose::new(100.0, 50.0, 30.0);
        let design = Point::new(12.5, -4.0);
        let recovered = pose.inverse_transform(pose.transform(design));
        assert!((recovered.x - design.x).abs() < 1e-9);
        assert!((recovered.y - design.y).abs() < 1e-9);
    }
}
