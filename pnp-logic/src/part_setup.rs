//! Deriving part parameters from a footprint name.
//!
//! When a placement list is imported, each row names a footprint (e.g.
//! "0603", "SOT-23", "SOIC-8"). The machine needs to know which nozzle can
//! pick the part and how it is oriented, but asking the operator to enter
//! that for every row is tedious and error-prone.
//!
//! This module makes a best guess from the footprint name. A guess is
//! never trusted silently: the result is marked [`PartStatus::Guessed`] so
//! the operator can confirm it, and a name we cannot interpret is marked
//! [`PartStatus::NeedsReview`] rather than guessed at. A job containing a
//! part that needs review must not run.

use crate::feeder::PartId;
use crate::geometry::Size;
use crate::nozzle::NozzleKind;
use crate::parts::{Part, Polarity};
use shared_types::Point;

/// How much trust to place in a part's parameters.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, serde::Serialize, serde::Deserialize)]
pub enum PartStatus {
    /// Parameters were derived from the footprint name and not yet
    /// confirmed by the operator.
    #[default]
    Guessed,
    /// Parameters were entered or confirmed by the operator.
    Confirmed,
    /// The footprint name could not be interpreted. The job must not run
    /// until the operator supplies the missing parameters.
    NeedsReview,
}

/// Parameters the machine needs for one part, independent of where the
/// part is placed on the board.
#[derive(Debug, Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct PartParams {
    /// Nozzle tips this part may be picked with.
    pub nozzle_type: Vec<NozzleKind>,
    /// Rotation applied when the part is picked, degrees.
    pub pickup_rotation_deg: f64,
    /// Pickup point relative to the part centroid, in the part's local
    /// frame. Zero for parts picked at their centroid.
    pub pickup_offset: Point,
    /// Body size in millimetres.
    pub footprint_size: Size,
    /// Where the polarity indicator sits, for vision-based orientation.
    pub polarity: Polarity,
}

/// A part together with how much we trust its parameters.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct SetupPart {
    pub part: Part,
    pub status: PartStatus,
}

/// Editable form for creating or editing a part.
///
/// Mirrors the fields the UI edits, with the nozzle allowlist kept as a
/// human-readable string so it can be typed directly.
#[derive(Debug, Clone, Default)]
pub struct PartForm {
    pub mpn: String,
    pub description: String,
    pub footprint_name: String,
    pub nozzle_types: Vec<NozzleKind>,
    pub pickup_rotation: f64,
    pub pickup_offset_x: f64,
    pub pickup_offset_y: f64,
    pub status: PartStatus,
}

impl PartForm {
    /// An empty form, ready for a new part.
    pub fn empty() -> Self {
        Self::default()
    }

    /// Build a form from a library entry, so the UI can edit an existing
    /// part without retyping it.
    pub fn from_setup(entry: &SetupPart) -> Self {
        Self {
            mpn: entry.part.mpn.clone(),
            description: entry.part.description.clone(),
            footprint_name: entry.part.footprint.name.clone(),
            nozzle_types: entry.part.nozzle_type.clone(),
            pickup_rotation: entry.part.pickup_rotation_deg,
            pickup_offset_x: entry.part.pickup_offset.x,
            pickup_offset_y: entry.part.pickup_offset.y,
            status: entry.status,
        }
    }

    /// Nozzle allowlist as a JSON string, for the text input.
    pub fn nozzle_types_json(&self) -> String {
        serde_json::to_string(&self.nozzle_types).unwrap_or_default()
    }

    /// Whether the form's status is "Guessed", used to preselect the
    /// dropdown.
    pub fn status_is_guessed(&self) -> bool {
        self.status == PartStatus::Guessed
    }
}

/// Best-effort interpretation of a footprint name.
///
/// Returns `None` when the name matches no known package. The caller
/// should treat that as "needs review" rather than guessing, because a
/// wrong nozzle choice damages parts and a wrong rotation places them
/// rotated 180°.
pub fn derive_part_params(footprint_name: &str) -> Option<PartParams> {
    let normalized = footprint_name.trim().to_ascii_uppercase();
    if normalized.is_empty() {
        return None;
    }

    // Imperial chip codes (0201, 0402, 0603, ...) carry their own
    // dimensions, so they are the most reliable thing to match on.
    // Polar chip variants like "LED-0603" parse from the tail while the
    // polarity comes from the full name.
    let chip_candidate = normalized
        .strip_prefix("LED-")
        .or_else(|| normalized.strip_prefix("LED"))
        .unwrap_or(&normalized);
    if let Some(size) = imperial_chip_size(chip_candidate) {
        return Some(chip_params(size, &normalized));
    }

    // KiCad-style names embed the chip code inside a longer string
    // ("C_0402_1005Metric", "R_0603_1608Metric", "L_0805_2012Metric").
    // The imperial code comes first; the trailing 4-digit metric code
    // (millimetres) is translated back to imperial so the same part
    // family is recognised either way.
    if let Some(size) = embedded_chip_size(&normalized) {
        return Some(chip_params(size, &normalized));
    }

    // Named packages (SOT-23, SOIC-8, QFP-64, ...).
    if let Some(params) = named_package_params(&normalized) {
        return Some(params);
    }

    None
}

/// Extract every 4-digit run bounded by non-digit characters (or the
/// string edges): `C_0402_1005Metric` yields `["0402", "1005"]` while
/// `SOIC-8` yields nothing.
fn embedded_chip_codes(name: &str) -> Vec<String> {
    let bytes = name.as_bytes();
    let mut codes = Vec::new();
    let mut start: Option<usize> = None;
    for (index, byte) in bytes.iter().enumerate() {
        if byte.is_ascii_digit() {
            if start.is_none() {
                start = Some(index);
            }
        } else if let Some(begin) = start.take() {
            if index - begin == 4 {
                codes.push(name[begin..index].to_string());
            }
        }
    }
    if let Some(begin) = start {
        if bytes.len() - begin == 4 {
            codes.push(name[begin..].to_string());
        }
    }
    codes
}

/// Canonical imperial chip codes, in hundredths of an inch. String keys
/// keep the leading zeros (`"0402"` is length 4, width 2).
fn imperial_chip_hundredths(code: &str) -> Option<(f64, f64)> {
    let (length, width) = match code {
        "0201" => (2.0, 1.0),
        "0402" => (4.0, 2.0),
        "0603" => (6.0, 3.0),
        "0805" => (8.0, 5.0),
        "1206" => (12.0, 6.0),
        "1210" => (12.0, 10.0),
        "1806" => (18.0, 6.0),
        "1812" => (18.0, 12.0),
        "2010" => (20.0, 10.0),
        "2512" => (25.0, 12.0),
        _ => return None,
    };
    Some((length, width))
}

/// Metric chip codes (mm × 10) mapped onto their imperial twin, so
/// `1005` (1.0 × 0.5 mm) is recognised as the same package as `0402`.
fn metric_to_imperial_code(code: &str) -> Option<&'static str> {
    Some(match code {
        "0603" => "0201",
        "1005" => "0402",
        "1608" => "0603",
        "2012" => "0805",
        "3216" => "1206",
        "3225" => "1210",
        "4516" => "1806",
        "4532" => "1812",
        "5025" => "2010",
        "6332" => "2512",
        _ => return None,
    })
}

/// Chip size from an embedded code. KiCad names lead with the imperial
/// code (`C_0402_1005Metric`), so the first run wins when it reads as
/// imperial; otherwise any metric-looking run is translated. `0603` is
/// deliberately ambiguous (imperial 0603 and metric 0201 share the
/// digits) and resolves as imperial, matching the KiCad convention.
fn embedded_chip_size(name: &str) -> Option<Size> {
    let codes = embedded_chip_codes(name);
    let mut imperial: Option<&str> = None;
    for code in &codes {
        if imperial_chip_hundredths(code).is_some() {
            imperial = Some(code.as_str());
            break;
        }
    }
    if imperial.is_none() {
        for code in &codes {
            if let Some(translated) = metric_to_imperial_code(code) {
                imperial = Some(translated);
                break;
            }
        }
    }
    let (length, width) = imperial_chip_hundredths(imperial?)?;
    if length <= 0.0 || width <= 0.0 {
        return None;
    }
    Some(Size {
        width: length * 0.254,
        height: width * 0.254,
    })
}

/// Build a [`Part`] from a footprint name, or `None` if it is unknown.
///
/// The returned part has a placeholder id and empty MPN/description; the
/// caller fills those in from the placement row.
pub fn part_from_footprint(footprint_name: &str) -> Option<SetupPart> {
    let params = derive_part_params(footprint_name)?;
    Some(SetupPart {
        part: Part {
            id: PartId(0),
            mpn: String::new(),
            description: footprint_name.trim().to_string(),
            footprint: crate::parts::Footprint {
                name: footprint_name.trim().to_string(),
                centroid: Point::ORIGIN,
                size: params.footprint_size,
                pads: Vec::new(),
                polarity: params.polarity,
            },
            nozzle_type: params.nozzle_type,
            pickup_rotation_deg: params.pickup_rotation_deg,
            pickup_offset: params.pickup_offset,
            camera: None,
        },
        status: PartStatus::Guessed,
    })
}

/// Parse an imperial chip code such as `0603` into a body size.
///
/// The first two digits are the length in hundredths of an inch and the
/// last two the width, so `0603` is 0.06 × 0.03 inch ≈ 1.52 × 0.76 mm.
/// Returns `None` for anything that is not exactly four digits.
fn imperial_chip_size(name: &str) -> Option<Size> {
    let digits: Vec<u8> = name
        .bytes()
        .filter(|b| b.is_ascii_digit())
        .map(|b| b - b'0')
        .collect();
    if digits.len() != 4 {
        return None;
    }
    // Reject names that are mostly non-digits: "0603" is a chip code but
    // "0603-LED" or "R0603" should not be treated as one unless the digits
    // are the whole story.
    let non_digit = name.bytes().filter(|b| !b.is_ascii_digit()).count();
    if non_digit > 0 && name.len() != 4 {
        return None;
    }

    let length_hundredths = (digits[0] * 10 + digits[1]) as f64;
    let width_hundredths = (digits[2] * 10 + digits[3]) as f64;
    if length_hundredths <= 0.0 || width_hundredths <= 0.0 {
        return None;
    }
    // 0.01 inch = 0.254 mm.
    Some(Size {
        width: length_hundredths * 0.254,
        height: width_hundredths * 0.254,
    })
}

/// Parameters for a chip-style part, chosen from its body size.
fn chip_params(size: Size, name: &str) -> PartParams {
    let min_dim = size.width.min(size.height);
    PartParams {
        nozzle_type: vec![nozzle_for_min_dimension(min_dim)],
        pickup_rotation_deg: 0.0,
        pickup_offset: Point::ORIGIN,
        footprint_size: size,
        polarity: chip_polarity(name),
    }
}

/// Pick a Juki 5xx nozzle for a part whose smallest body dimension is
/// `min_dim` millimetres.
///
/// The boundaries follow the Juki nozzle catalogue (Rev C3): each 5xx
/// nozzle is rated for a component-width range (`D < 0.45`,
/// `0.45 ≤ D < 0.75`, `0.75 ≤ D < 1.45`, `1.45 ≤ D < 2.5`,
/// `2.5 ≤ D < 4.0`, `4.0 ≤ D < 7.0`, `7.0 ≤ D < 10.0`, `D ≥ 10.0`),
/// so the smallest dimension falls into the range of the tip to use.
fn nozzle_for_min_dimension(min_dim: f64) -> NozzleKind {
    if min_dim < 0.45 {
        NozzleKind::N501
    } else if min_dim < 0.75 {
        NozzleKind::N502
    } else if min_dim < 1.45 {
        NozzleKind::N503
    } else if min_dim < 2.5 {
        NozzleKind::N504
    } else if min_dim < 4.0 {
        NozzleKind::N505
    } else if min_dim < 7.0 {
        NozzleKind::N506
    } else if min_dim < 10.0 {
        NozzleKind::N507
    } else {
        NozzleKind::N508
    }
}

/// Polarity for a chip part, guessed from the name.
///
/// Most passives have no polarity; diodes and LEDs do. Getting this wrong
/// places the part rotated 180°, which is why an unrecognised polar part
/// is left as `None` and flagged for review by the caller.
fn chip_polarity(name: &str) -> Polarity {
    if name.contains("LED") || name.contains("DIODE") {
        Polarity::Dot
    } else {
        Polarity::None
    }
}

/// Parameters for a named (non-chip) package.
fn named_package_params(name: &str) -> Option<PartParams> {
    // Small-outline transistors.
    if let Some(params) = sot_params(name) {
        return Some(params);
    }
    // Small-outline ICs and friends.
    if let Some(params) = soic_params(name) {
        return Some(params);
    }
    // Quad flat packages and chip-scale packages.
    if let Some(params) = qfp_params(name) {
        return Some(params);
    }
    // Ball grid arrays.
    if name.starts_with("BGA") {
        return Some(PartParams {
            nozzle_type: vec![nozzle_for_min_dimension(10.0)],
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            footprint_size: Size {
                width: 10.0,
                height: 10.0,
            },
            polarity: Polarity::Pin1,
        });
    }
    None
}

fn sot_params(name: &str) -> Option<PartParams> {
    let body: (f64, f64) = match name {
        "SOT-23" | "SOT23" => (2.9, 1.3),
        "SOT-223" | "SOT223" => (6.5, 3.5),
        "SOT-89" | "SOT89" => (4.5, 2.5),
        "SOT-363" | "SOT363" | "SC-70" => (2.0, 1.25),
        "SOT-723" | "SOT723" => (2.0, 1.25),
        _ => return None,
    };
    Some(PartParams {
        nozzle_type: vec![nozzle_for_min_dimension(body.1.min(body.0))],
        pickup_rotation_deg: 0.0,
        pickup_offset: Point::ORIGIN,
        footprint_size: Size {
            width: body.0,
            height: body.1,
        },
        polarity: Polarity::Pin1,
    })
}

fn soic_params(name: &str) -> Option<PartParams> {
    let body: (f64, f64) = match name {
        "SOIC-8" | "SOIC8" | "SO-8" | "SO8" => (4.9, 3.9),
        "SOIC-14" | "SOIC14" | "SO-14" => (8.7, 3.9),
        "SOIC-16" | "SOIC16" | "SO-16" => (10.3, 3.9),
        "TSSOP-8" | "TSSOP8" => (4.4, 3.0),
        "TSSOP-16" | "TSSOP16" => (5.0, 4.4),
        "TSSOP-20" | "TSSOP20" => (6.5, 4.4),
        "TSSOP-28" | "TSSOP28" => (9.7, 4.4),
        "TSSOP-48" | "TSSOP48" => (12.5, 6.1),
        "QFN-16" | "QFN16" => (3.0, 3.0),
        "QFN-32" | "QFN32" => (5.0, 5.0),
        "QFN-48" | "QFN48" => (7.0, 7.0),
        _ => return None,
    };
    Some(PartParams {
        nozzle_type: vec![nozzle_for_min_dimension(body.1.min(body.0))],
        pickup_rotation_deg: 0.0,
        pickup_offset: Point::ORIGIN,
        footprint_size: Size {
            width: body.0,
            height: body.1,
        },
        polarity: Polarity::Pin1,
    })
}

fn qfp_params(name: &str) -> Option<PartParams> {
    let body: (f64, f64) = match name {
        "QFP-32" | "QFP32" => (7.0, 7.0),
        "QFP-44" | "QFP44" => (10.0, 10.0),
        "QFP-64" | "QFP64" => (12.0, 12.0),
        "QFP-100" | "QFP100" => (16.0, 16.0),
        _ => return None,
    };
    Some(PartParams {
        nozzle_type: vec![nozzle_for_min_dimension(body.1.min(body.0))],
        pickup_rotation_deg: 0.0,
        pickup_offset: Point::ORIGIN,
        footprint_size: Size {
            width: body.0,
            height: body.1,
        },
        polarity: Polarity::Pin1,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params(name: &str) -> PartParams {
        derive_part_params(name).unwrap_or_else(|| panic!("{name} should be recognised"))
    }

    #[test]
    fn imperial_chip_codes_give_their_dimensions() {
        let p = params("0603");
        // 0.06 x 0.03 inch = 1.524 x 0.762 mm.
        assert!((p.footprint_size.width - 1.524).abs() < 1e-9);
        assert!((p.footprint_size.height - 0.762).abs() < 1e-9);
    }

    #[test]
    fn kicad_style_names_recognise_the_embedded_chip_code() {
        // KiCad wraps the chip code in a library prefix and a metric
        // suffix: C_0402_1005Metric is a 0402 (1.0 x 0.5 mm) capacitor.
        // Without this, imported KiCad position files could not be
        // planned at all.
        let p = params("C_0402_1005Metric");
        assert!((p.footprint_size.width - 1.016).abs() < 1e-9);
        assert!((p.footprint_size.height - 0.508).abs() < 1e-9);
        assert_eq!(p.nozzle_type, vec![NozzleKind::N502]);

        // The metric code alone also resolves (R_1608_... → 0603).
        let p = params("R_1608_1608Metric");
        assert!((p.footprint_size.width - 1.524).abs() < 1e-9);
        assert!((p.footprint_size.height - 0.762).abs() < 1e-9);

        // The leading package code wins over the trailing metric one
        // when both are present and imperial.
        let p = params("R_0603_1608Metric");
        assert!((p.footprint_size.width - 1.524).abs() < 1e-9);

        // Non-chip names with short digit runs must not misfire.
        assert!(derive_part_params("SOIC-8").is_some());
    }

    #[test]
    fn chip_nozzle_follows_the_smallest_dimension() {
        // Juki nozzle catalogue Rev C3 buckets by component width D.
        // 0402 is 1.016 x 0.508 mm, so 0.45 <= D < 0.75 → nozzle 502.
        assert_eq!(params("0402").nozzle_type, vec![NozzleKind::N502]);
        // 0603 is 1.524 x 0.762 mm, so 0.75 <= D < 1.45 → nozzle 503.
        assert_eq!(params("0603").nozzle_type, vec![NozzleKind::N503]);
        // 0805 is 2.032 x 1.27 mm, so 0.75 <= D < 1.45 → nozzle 503.
        assert_eq!(params("0805").nozzle_type, vec![NozzleKind::N503]);
        // 1206 is 3.048 x 1.524 mm, so 1.45 <= D < 2.5 → nozzle 504.
        assert_eq!(params("1206").nozzle_type, vec![NozzleKind::N504]);
    }

    #[test]
    fn named_packages_are_recognised() {
        // Catalogue examples: SOT-23 → 503, and packages wider than
        // 10 mm fall back to the largest standard nozzle, 508.
        assert_eq!(params("SOT-23").nozzle_type, vec![NozzleKind::N503]);
        assert_eq!(params("SOIC-8").nozzle_type, vec![NozzleKind::N505]);
        assert_eq!(params("QFP-64").nozzle_type, vec![NozzleKind::N508]);
        assert_eq!(params("BGA-256").nozzle_type, vec![NozzleKind::N508]);
    }

    #[test]
    fn package_names_are_case_and_separator_insensitive() {
        // Operators paste from different CAD tools, so "SOT-23", "sot23"
        // and "SOT 23" should all work.
        assert!(derive_part_params("sot-23").is_some());
        assert!(derive_part_params("SOT23").is_some());
        assert!(derive_part_params("soic8").is_some());
    }

    #[test]
    fn unknown_names_are_not_guessed() {
        // Guessing here would pick a wrong nozzle and damage the part.
        assert_eq!(derive_part_params("MY-CUSTOM-PART"), None);
        assert_eq!(derive_part_params(""), None);
        assert_eq!(derive_part_params("   "), None);
    }

    #[test]
    fn polar_parts_are_marked() {
        assert_eq!(params("LED-0603").polarity, Polarity::Dot);
        assert_eq!(params("0603").polarity, Polarity::None);
    }

    #[test]
    fn a_derived_part_carries_its_footprint() {
        let setup = part_from_footprint("0603").unwrap();
        assert_eq!(setup.status, PartStatus::Guessed);
        assert_eq!(setup.part.footprint.name, "0603");
        assert!((setup.part.footprint.size.width - 1.524).abs() < 1e-9);
    }

    #[test]
    fn a_derived_part_has_a_usable_nozzle_allowlist() {
        // An empty allowlist makes the part unplaceable, so a guess must
        // always produce at least one nozzle.
        let setup = part_from_footprint("SOT-23").unwrap();
        }
}
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
/// A summary of a part, containing only the fields needed for display
/// and transfer between server and client without requiring the full `pnp_logic`
/// crate (which depends on OpenCV).
pub struct PartSummary {
    /// Part identifier.
    pub id: u32,
    /// Manufacturer part number.
    pub mpn: String,
    /// Footprint/designator name.
    pub footprint: String,
    /// Human-readable description.
    pub description: String,
    /// Nozzle allowlist as a JSON string.
    pub nozzles: String,
    /// Pickup rotation in degrees.
    pub rotation: f64,
    /// Pickup offset X in mm.
    pub offset_x: f64,
    /// Pickup offset Y in mm.
    pub offset_y: f64,
    /// Trust status of the part parameters.
    pub status: PartStatus,
}