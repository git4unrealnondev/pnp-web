//! Nozzle head geometry and tool assignment.
//!
//! A PnP head carries N nozzles (typically 4, 6, or 8) in a straight row
//! along the X axis. Each nozzle has:
//!   - an offset along the head from the head's reference point
//!   - a physical kind (size of the tip)
//!   - an independent rotation angle
//!   - independent vacuum / blow valves
//!
//! All nozzles share the head's XY position; the gantry moves the whole
//! head. To "use" a particular nozzle, the head is at the same XY, but
//! commands are sent for that specific nozzle's Z, rotation, and valves.

use crate::geometry::Size;
use crate::parts::Part;
use shared_types::*;

/// Catalogue of nozzle tips, following the Juki nozzle catalogue
/// (Rev C3): the standard 5xx family for 2000-series heads, numbered by
/// the component width they pick (`501` smallest → `508` largest).
///
/// Tip diameters are the catalog's tip **outer** dimensions, in 0.01 mm
/// units. Compatibility with a part is decided by the per-part
/// allowlist (`Part::nozzle_type`), which planning re-derives from the
/// footprint; `diameter()` only feeds size heuristics.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum NozzleKind {
    /// D < 0.45 mm — 0.7 mm tip.
    N501,
    /// 0.45 ≤ D < 0.75 mm — 0.7 mm tip.
    N502,
    /// 0.75 ≤ D < 1.45 mm — 1.0 mm tip (0603, 0805, SOT-23, SOD-323).
    N503,
    /// 1.45 ≤ D < 2.5 mm — 1.5 mm tip (1206, SOT-25/26, SOD-123).
    N504,
    /// 2.5 ≤ D < 4.0 mm — 3.5 mm tip (2512, SOIC, QFP, BGA).
    N505,
    /// 4.0 ≤ D < 7.0 mm — 5.0 mm tip (DPAK, electrolytics).
    N506,
    /// 7.0 ≤ D < 10.0 mm — 8.5 mm tip.
    N507,
    /// D ≥ 10.0 mm — 9.5 mm tip.
    N508,
    /// Deprecated: 3.0 mm tip from the pre-Juki catalogue. Kept so old
    /// job files still load; planning heals allowlists to 5xx nozzles.
    Standard,
    /// Deprecated: 1.5 mm tip (matches Juki 504).
    Fine,
    /// Deprecated: 1.0 mm tip (matches Juki 503).
    ExtraFine,
    /// Custom tip diameter in 0.01 mm units (e.g. `Custom(250)` = 2.50 mm).
    Custom(u16),
}

impl NozzleKind {
    /// Tip diameter, in 0.01 mm units.
    pub fn diameter(self) -> u16 {
        match self {
            NozzleKind::N501 | NozzleKind::N502 => 70,
            NozzleKind::N503 => 100,
            NozzleKind::N504 => 150,
            NozzleKind::N505 => 350,
            NozzleKind::N506 => 500,
            NozzleKind::N507 => 850,
            NozzleKind::N508 => 950,
            NozzleKind::Standard => 300,
            NozzleKind::Fine => 150,
            NozzleKind::ExtraFine => 100,
            NozzleKind::Custom(d) => d,
        }
    }

    /// Short name, useful for logs and labels.
    pub fn name(self) -> &'static str {
        match self {
            NozzleKind::N501 => "501",
            NozzleKind::N502 => "502",
            NozzleKind::N503 => "503",
            NozzleKind::N504 => "504",
            NozzleKind::N505 => "505",
            NozzleKind::N506 => "506",
            NozzleKind::N507 => "507",
            NozzleKind::N508 => "508",
            NozzleKind::Standard => "Standard",
            NozzleKind::Fine => "Fine",
            NozzleKind::ExtraFine => "ExtraFine",
            NozzleKind::Custom(_) => "Custom",
        }
    }

    /// `true` if this nozzle's tip is large enough for a part that needs
    /// `part_diameter` (also in 0.01 mm units). Bare-bones one-way size
    /// check; the per-part `accepts_nozzle` allowlist is the
    /// authoritative compatibility test.
    pub fn accepts(self, part_diameter: u16) -> bool {
        self.diameter() >= part_diameter
    }
}

/// One physical nozzle slot on the head.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct NozzleSlot {
    /// Physical nozzle number. HWGC/OpenKayo uses 1..=N; feeder IDs remain
    /// independent and may still be zero-based.
    pub id: u8,
    /// Offset from the head reference point along the row, in mm.
    /// Negative = towards -X, positive = towards +X.
    pub offset_mm: f64,
    pub kind: NozzleKind,
    /// Current rotation state, in degrees CCW.
    pub rotation_deg: f64,
    /// Current vacuum state (true = part held).
    pub vacuum: bool,
}

/// The whole head — array of nozzle slots plus the head's current XY pose.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NozzleHead {
    pub position: Point,
    pub nozzles: Vec<NozzleSlot>,
    /// Optional camera mounted on the head, used only during job setup
    /// (board preview, fiducial hunting, manual part inspection). It
    /// moves with the head and is not used by the pick/place state
    /// machine.
    pub onboard_camera: Option<OnboardCamera>,
}

/// Head-mounted setup camera. `offset_mm` is the camera's centre in
/// head-local coords (same frame as `NozzleSlot.offset_mm`); `fov_mm`
/// is the rectangular sensing window centred on that offset.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct OnboardCamera {
    pub offset_mm: Point,
    pub fov_mm: Size,
}

impl NozzleHead {
    pub fn new(nozzles: Vec<NozzleSlot>) -> Self {
        Self {
            position: Point::ORIGIN,
            nozzles,
            onboard_camera: None,
        }
    }

    pub fn with_onboard_camera(mut self, camera: OnboardCamera) -> Self {
        self.onboard_camera = Some(camera);
        self
    }

    /// World-space rectangle currently seen by the head-mounted camera,
    /// or `None` if the head has no onboard camera.
    pub fn onboard_view(&self) -> Option<(Point, Point)> {
        self.onboard_camera
            .map(|camera| camera.view_at(self.position))
    }
}

impl OnboardCamera {
    pub(crate) fn view_at(self, head_position: Point) -> (Point, Point) {
        let center = Point::new(
            head_position.x + self.offset_mm.x,
            head_position.y + self.offset_mm.y,
        );
        let half_width = self.fov_mm.width / 2.0;
        let half_height = self.fov_mm.height / 2.0;
        (
            Point::new(center.x - half_width, center.y - half_height),
            Point::new(center.x + half_width, center.y + half_height),
        )
    }
}

impl NozzleHead {
    /// World-space position of the tip of `nozzle_id`. IDs are physical
    /// nozzle numbers when supplied by hardware (HWGC uses 1..=N); the
    /// legacy zero-based vector index is accepted for existing job data.
    ///
    /// This is "where the nozzle is centred" — the reference point that
    /// moves with the gantry.
    pub fn tip_position(&self, nozzle_id: u8) -> Point {
        let n = self.nozzle(nozzle_id);
        Point::new(self.position.x + n.offset_mm, self.position.y)
    }

    /// World-space position of a point `local` on the part currently held
    /// by `nozzle_id`.
    ///
    /// `local` is in the part's local frame: where the point sits relative
    /// to the part's pickup point when the nozzle is at rotation 0. The
    /// current nozzle rotation is applied.
    ///
    /// This is the workhorse for:
    ///  - deciding where to look with the up-camera to inspect a part,
    ///  - computing the head move that puts a specific part feature
    ///    over its design position during placement,
    ///  - rotating a part in place (the pickup point stays put, the rest
    ///    of the part moves around it).
    pub fn part_point(&self, nozzle_id: u8, local: Point) -> Point {
        let n = self.nozzle(nozzle_id);
        let tip = self.tip_position(nozzle_id);
        let (s, c) = n.rotation_deg.to_radians().sin_cos();
        Point::new(
            tip.x + local.x * c - local.y * s,
            tip.y + local.x * s + local.y * c,
        )
    }

    /// Head position such that `world_target` is reached by the tip of
    /// `nozzle_id`.
    pub fn head_position_for_tip(&self, nozzle_id: u8, world_target: Point) -> Point {
        let n = self.nozzle(nozzle_id);
        Point::new(world_target.x - n.offset_mm, world_target.y)
    }

    /// Head position such that `world_target` is reached by the point
    /// `part_local` on the held part at `nozzle_id`.
    ///
    /// Inverse of `part_point`: given where you want a feature on the
    /// held part to be in the world, this returns where the head
    /// reference must move.
    pub fn head_position_for_part_point(
        &self,
        nozzle_id: u8,
        part_local: Point,
        world_target: Point,
    ) -> Point {
        let n = self.nozzle(nozzle_id);
        let (s, c) = n.rotation_deg.to_radians().sin_cos();
        let rx = part_local.x * c - part_local.y * s;
        let ry = part_local.x * s + part_local.y * c;
        Point::new(world_target.x - n.offset_mm - rx, world_target.y - ry)
    }

    /// Look up a physical nozzle number. HWGC/OpenKayo numbers axes from 1,
    /// while the vector remains zero-based for storage.
    pub fn nozzle(&self, nozzle_id: u8) -> &NozzleSlot {
        self.nozzles
            .iter()
            .find(|n| n.id == nozzle_id)
            .or_else(|| self.nozzles.get(nozzle_id as usize))
            .unwrap_or_else(|| panic!("invalid nozzle id {}", nozzle_id))
    }

    pub fn nozzle_mut(&mut self, nozzle_id: u8) -> Option<&mut NozzleSlot> {
        if let Some(index) = self.nozzles.iter().position(|n| n.id == nozzle_id) {
            self.nozzles.get_mut(index)
        } else {
            self.nozzles.get_mut(nozzle_id as usize)
        }
    }
}

/// Build a default head with `spans` nozzles at 20 mm pitch, centred
/// on the head reference point. IDs follow HWGC/OpenKayo's 1-based
/// physical nozzle numbering. Common configurations: 4, 6, or 8.
pub fn default_head(spans: u8) -> NozzleHead {
    let nozzles = (0..spans)
        .map(|i| NozzleSlot {
            id: i + 1,
            offset_mm: (i as f64 - (spans as f64 - 1.0) / 2.0) * 20.0,
            kind: NozzleKind::Standard,
            rotation_deg: 0.0,
            vacuum: false,
        })
        .collect();
    NozzleHead::new(nozzles)
}

/// Assign one nozzle id per part in `parts`, preferring the nozzle closest
/// in index to the previous choice so the head moves as little as possible
/// between picks.
///
/// Compatibility is decided in two steps:
/// 1. If the part has a non-empty `accepts_nozzle` allowlist, only
///    nozzles whose `kind` is on the list are considered.
/// 2. Otherwise, the bare size check (`nozzle.diameter() >= part.nozzle_diameter`)
///    is used.
///
/// A real optimiser would also consider the physical location of feeders
/// in a pick sequence.
pub fn assign_nozzles(parts: &[Part], head: &NozzleHead) -> Vec<u8> {
    let mut last: i16 = 0;
    let mut use_count = vec![0usize; head.nozzles.len()];
    parts
        .iter()
        .map(|p| {
            let chosen = head
                .nozzles
                .iter()
                .filter(|n| nozzle_matches_part(n, p))
                // Balance compatible tools before minimizing index travel.
                // Reusing one nozzle is attractive locally, but it leaves the
                // rest of a multi-nozzle head idle over a complete job.
                .min_by_key(|n| {
                    (
                        use_count
                            .iter()
                            .enumerate()
                            .find(|(index, _)| head.nozzles[*index].id == n.id)
                            .map(|(_, count)| *count)
                            .unwrap_or(0),
                        (n.id as i16 - last).unsigned_abs(),
                    )
                });
            let id = chosen.map(|n| n.id).unwrap_or(1);
            if let Some(index) = head.nozzles.iter().position(|n| n.id == id) {
                if let Some(count) = use_count.get_mut(index) {
                    *count += 1;
                }
            } else if let Some(count) = use_count.get_mut(id as usize) {
                *count += 1;
            }
            last = id as i16;
            id
        })
        .collect()
}

/// Is this nozzle slot compatible with the part?
fn nozzle_matches_part(n: &NozzleSlot, p: &Part) -> bool {
    // A part MUST declare its nozzle_type (non-empty). If it's empty,
    // we conservatively reject the nozzle so the caller must provide
    // an allowlist.
    if p.nozzle_type.is_empty() {
        return false;
    }
    p.nozzle_type.contains(&n.kind)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn tip_position_is_head_plus_offset() {
        let head = default_head(4);
        assert_eq!(head.tip_position(1).x, -30.0);
        assert_eq!(head.tip_position(4).x, 30.0);
    }

    #[test]
    fn onboard_view_tracks_head_position() {
        let mut head = default_head(2);
        head.onboard_camera = Some(OnboardCamera {
            offset_mm: Point::new(5.0, -2.0),
            fov_mm: Size {
                width: 10.0,
                height: 8.0,
            },
        });
        head.position = Point::new(100.0, 50.0);
        let (lo, hi) = head.onboard_view().unwrap();
        assert_eq!(lo, Point::new(100.0, 44.0));
        assert_eq!(hi, Point::new(110.0, 52.0));
        assert!(head.onboard_view().is_some());
        head.onboard_camera = None;
        assert!(head.onboard_view().is_none());
    }

    #[test]
    fn part_point_rotates_around_tip() {
        let mut head = default_head(1);
        head.nozzles[0].rotation_deg = 90.0;
        let p = head.part_point(0, Point::new(1.0, 0.0));
        assert!((p.x - 0.0).abs() < 1e-9);
        assert!((p.y - 1.0).abs() < 1e-9);
    }

    #[test]
    fn head_position_for_part_point_is_inverse() {
        let mut head = default_head(1);
        head.nozzles[0].rotation_deg = 90.0;
        let target = Point::new(100.0, 50.0);
        let head_pos = head.head_position_for_part_point(0, Point::new(1.0, 0.0), target);
        head.position = head_pos;
        let p = head.part_point(0, Point::new(1.0, 0.0));
        assert!((p.x - target.x).abs() < 1e-9);
        assert!((p.y - target.y).abs() < 1e-9);
        assert_eq!(head_pos, head.position);
    }

    fn mk_part(id: u32, nozzle_type: &[NozzleKind]) -> Part {
        use crate::feeder::PartId;
        use crate::geometry::Size;
        Part {
            id: PartId(id),
            mpn: format!("P{}", id),
            description: String::new(),
            footprint: crate::parts::Footprint {
                name: format!("P{}", id),
                centroid: Point::ORIGIN,
                size: Size {
                    width: 1.0,
                    height: 1.0,
                },
                pads: vec![],
                polarity: crate::parts::Polarity::None,
            },
            nozzle_type: nozzle_type.to_vec(),
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        }
    }

    /// 4-nozzle head: 503 at slots 1 & 3, 502 at slots 2 & 4.
    /// 0603 (size threshold ~1.6×0.8 mm) only allows 503; 0402
    /// (1.0×0.5 mm) only allows 502. The optimiser routes the 0603
    /// to the 503 closest to its previous pick (nozzle 0) and the
    /// 0402 to the 502 closest to that.
    #[test]
    fn resistors_route_to_their_own_nozzle_family() {
        let mut head = default_head(4);
        head.nozzles[0].kind = NozzleKind::Standard; // unused but plausible
        head.nozzles[1].kind = NozzleKind::N503; // 503
        head.nozzles[2].kind = NozzleKind::N502; // 502
        head.nozzles[3].kind = NozzleKind::N503; // 503

        let r_0603 = mk_part(1, &[NozzleKind::N503]); // must use 503
        let r_0402 = mk_part(2, &[NozzleKind::N502]); // must use 502

        let assignment = assign_nozzles(&[r_0603, r_0402], &head);
        assert_eq!(assignment, vec![2, 3]);
    }

    /// Multiple compatible parts are distributed across the compatible
    /// nozzles so a multi-nozzle head can form useful pickup bursts.
    #[test]
    fn multiple_0603s_are_balanced_across_compatible_nozzles() {
        let mut head = default_head(4);
        head.nozzles[1].kind = NozzleKind::N503;
        head.nozzles[2].kind = NozzleKind::N502;
        head.nozzles[3].kind = NozzleKind::N503;

        let r1 = mk_part(1, &[NozzleKind::N503]);
        let r2 = mk_part(2, &[NozzleKind::N503]);
        let r3 = mk_part(3, &[NozzleKind::N503]);
        let r4 = mk_part(4, &[NozzleKind::N503]);

        let assignment = assign_nozzles(&[r1, r2, r3, r4], &head);
        assert_eq!(assignment, vec![2, 4, 4, 2]);
    }

    /// A part with no allowlist still uses the size threshold.
    #[test]
    fn empty_accepts_nozzle_falls_back_to_size_check() {
        let mut head = default_head(2);
        head.nozzles[0].kind = NozzleKind::N503;
        head.nozzles[1].kind = NozzleKind::N502;

        // 0603 with no allowlist, size threshold 30 (0.30 mm).
        // Both 503 (30) and 502 (40) accept. Optimiser picks closest to
        // previous = 0 → nozzle 0.
        let p = mk_part(1, &[NozzleKind::Standard]);
        let assignment = assign_nozzles(&[p], &head);
        assert_eq!(assignment, vec![1]);
    }
}
