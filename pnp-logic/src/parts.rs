//! Parts and footprints.
//!
//! A `Part` is what the user describes in setup: an MPN, a footprint, and
//! pick constraints (which nozzle size, what pickup rotation, where the
//! pickup point sits in the part's local frame).
//!
//! A `Footprint` is what the user defines against the board: pads,
//! centroid, polarity mark. The machine matches a placed part against
//! the design footprint to identify what is going where; the camera
//! reads text / QR codes on the board itself to confirm which board it
//! is and which variant of a multi-up panel it is.

use crate::feeder::PartId;
use crate::geometry::Size;
use crate::nozzle::NozzleKind;
use shared_types::*;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Footprint {
    pub name: String,
    /// Centroid of the part in the footprint's local frame.
    pub centroid: Point,
    pub size: Size,
    pub pads: Vec<Pad>,
    pub polarity: Polarity,
}

#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Pad {
    pub position: Point,
    pub size: Size,
    pub kind: PadKind,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum PadKind {
    Smd,
    ThruHole,
}

/// Where the polarity indicator lives on the part, used by the camera
/// vision routine to determine part orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Polarity {
    None,
    /// Cathode dot (diodes).
    Dot,
    /// Pin-1 dot / bevel (ICs).
    Pin1,
    /// Cathode stripe (diodes, tantalum caps).
    Stripe,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Part {
    pub id: PartId,
    pub mpn: String,
    pub description: String,
    pub footprint: Footprint,
    /// The `NozzleKind`s this part is allowed to be picked with. The
    /// optimiser only considers head nozzles whose `kind` is in this
    /// list. Required — every part must declare which tip families it
    /// is rated for (e.g. a 0603 lists `[N503]`, a 0402 lists
    /// `[N502]`, a BGA lists `[N507, N508]`).
    pub nozzle_type: Vec<NozzleKind>,
    /// Pickup rotation, degrees. 0 = part is oriented as designed.
    pub pickup_rotation_deg: f64,
    /// Position of the part's centroid relative to the nozzle tip when
    /// the part is held at `pickup_rotation_deg`. Non-zero for parts
    /// whose pickup point is offset from the centroid (e.g. SOT-23).
    pub pickup_offset: Point,
    /// Which camera to inspect this part with, if the operator pinned
    /// one down in the part editor. `None` = automatic choice by body
    /// size (precise above the size threshold, fast vision below).
    #[serde(default)]
    pub camera: Option<crate::config::CameraKind>,
}
