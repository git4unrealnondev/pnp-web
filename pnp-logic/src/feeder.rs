//! Physical feeder slots and banks.
//!
//! A feeder holds a strip, stick, or tray of identical parts. The machine
//! has a fixed number of slots organised into one or more *banks* (e.g.
//! front rail + back rail). Each bank has its own rotation — the part's
//! "design orientation on the feeder" is the bank's frame, not the world
//! frame.
//!
//! Each (bank, part) pair also has a `PartPickup` describing where inside
//! the pocket the vacuum grabs and at what Z height. This is consulted
//! by the runner so the operator never has to re-teach the pickup after
//! changing a reel.
//!
//! Slot allocation lives in `crate::optimizer`.

use crate::parts::Part;
use shared_types::*;

/// Identifier for a part type. Typically a primary key from the parts
/// database; opaque to the machine logic.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct PartId(pub u32);

/// Form factor of a feeder.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub enum FeederKind {
    /// Tape feeder for a given width in mm (8, 12, 16, 24, 32, 44 are common).
    Tape(u8),
    Stick,
    Tray,
    Bulk,
    Tube,
    /// Vibratory bowl / inline feeder.
    Vibratory,
}

impl FeederKind {
    /// Whether this feeder can physically serve the part (size / width
    /// checks). The check is `w < tape_width` — exact fits are rejected
    /// because real tape pockets are smaller than the nominal width
    /// and an 8mm part on 8mm tape would bind or get damaged.
    pub fn accepts(self, part: &Part) -> bool {
        let w = part.footprint.size.width.max(part.footprint.size.height);
        match self {
            FeederKind::Tape(width_mm) => w < width_mm as f64,
            FeederKind::Stick => w <= 30.0,
            FeederKind::Tray => true,
            FeederKind::Bulk => w <= 10.0,
            FeederKind::Tube => w <= 8.0,
            FeederKind::Vibratory => true,
        }
    }

    /// Approximate parts-per-reel. Used by the planner to decide whether
    /// one reel is enough or whether the part must be split across
    /// multiple physical slots. Industry-typical numbers for tapewidth
    /// in mm; we differentiate widths because 12mm + 16mm + 24mm
    /// tape holders hold different numbers of parts on a single reel.
    pub fn capacity(self) -> u32 {
        match self {
            FeederKind::Tape(w) => match w {
                8 => 4000,
                12 => 2500,
                16 => 2000,
                24 => 1000,
                32 => 600,
                44 => 400,
                _ => 4000,
            },
            FeederKind::Stick => 50,
            FeederKind::Tray => 30,
            FeederKind::Bulk => 1000,
            FeederKind::Tube => 100,
            FeederKind::Vibratory => 1000,
        }
    }

    /// Provider class used by the native feeder optimizer.
    pub const fn provider_class(self) -> ProviderClass {
        match self {
            Self::Tray => ProviderClass::Tray,
            Self::Vibratory => ProviderClass::Vibratory,
            Self::Tape(_) | Self::Stick | Self::Bulk | Self::Tube => ProviderClass::Regular,
        }
    }
}

/// Provider classes are kept separate by the native optimizer because tray
/// and vibratory feeders use different internal candidate lists.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
pub enum ProviderClass {
    Regular,
    Tray,
    Vibratory,
}

impl ProviderClass {
    /// Native provider values 6 and 7 map to tray and vibratory providers;
    /// all other values are ordinary feeders.
    pub const fn from_native(provider: u8) -> Self {
        match provider {
            6 => Self::Tray,
            7 => Self::Vibratory,
            _ => Self::Regular,
        }
    }
}

/// One physical feeder position on the machine.
///
/// The physical `FeederKind` mounted here varies by job; the
/// per-job `MachineSetup` (in `crate::config`) maps this slot's `id`
/// to the `FeederKind` for the current run. This lets the same
/// physical slot host different feeder bodies across jobs.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct FeederSlot {
    pub id: u8,
    pub bank_id: u8,
    pub position: Point,
    /// Optional: the part already loaded in this slot. The planner
    /// re-uses pre-loaded reels when possible; the runner also uses
    /// this as a fallback if the per-job setup doesn't list the slot.
    pub loaded: Option<PartId>,
}

/// One bank of feeder slots. All slots in a bank share the same
/// rotation; a part loaded on slot A of bank B is picked up in bank B's
/// frame.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct FeederBank {
    pub id: u8,
    /// Rotation of the bank relative to the machine frame, in degrees.
    /// Pickup offsets are stored in this frame; the runner rotates them
    /// into the machine frame before commanding a move.
    pub rotation_deg: f64,
}

/// How the nozzle meets a specific (bank, part) combination: where the
/// pocket's reference point sits inside the bank's frame, and the Z
/// height the nozzle should descend to.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct PartPickup {
    /// Position of the pocket reference point in the *bank's* local
    /// frame. The runner applies the bank's rotation before commanding.
    pub offset: Point,
    /// Rotation of the part as loaded on the tape in the bank's frame.
    /// The runner applies this on top of the bank's rotation when
    /// initialising the nozzle for a fresh pickup.
    pub rotation_deg: f64,
    /// Z height at which the nozzle should meet this part on this
    /// feeder. Overrides the global `MachineConfig::pickup_z_mm`.
    pub pickup_z_mm: f64,
}

impl Default for PartPickup {
    fn default() -> Self {
        Self {
            offset: Point::ORIGIN,
            rotation_deg: 0.0,
            pickup_z_mm: 0.5,
        }
    }
}

/// User-supplied calibration table: per (bank, part) pickup overrides.
/// The runner consults this and falls back to the `Part`'s nominal
/// pickup when no entry is present.
#[derive(Debug, Clone, Default)]
pub struct FeederCalibration {
    entries: std::collections::HashMap<(u8, PartId), PartPickup>,
}

impl FeederCalibration {
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert / replace the pickup entry for a specific bank + part.
    pub fn set(&mut self, bank_id: u8, part: PartId, pickup: PartPickup) {
        self.entries.insert((bank_id, part), pickup);
    }

    /// Look up the pickup entry, if any.
    pub fn get(&self, bank_id: u8, part: PartId) -> Option<&PartPickup> {
        self.entries.get(&(bank_id, part))
    }

    /// Convenience: return the entry, or a default with the part's
    /// nominal offset / rotation and the machine's pickup Z.
    pub fn get_or(
        &self,
        bank_id: u8,
        part_id: PartId,
        part: &Part,
        default_z_mm: f64,
    ) -> PartPickup {
        match self.entries.get(&(bank_id, part_id)) {
            Some(p) => *p,
            None => PartPickup {
                offset: part.pickup_offset,
                rotation_deg: part.pickup_rotation_deg,
                pickup_z_mm: default_z_mm,
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Size;
    use crate::nozzle::NozzleKind;
    use crate::parts::{Footprint, Polarity};

    fn part_with_width(mm: f64) -> Part {
        Part {
            id: PartId(1),
            mpn: "X".into(),
            description: String::new(),
            footprint: Footprint {
                name: "X".into(),
                centroid: Point::ORIGIN,
                size: Size {
                    width: mm,
                    height: mm,
                },
                pads: vec![],
                polarity: Polarity::None,
            },
            nozzle_type: vec![NozzleKind::N503],
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        }
    }

    #[test]
    fn tape_width_is_strict() {
        // An 8mm part on 8mm tape is rejected — the tape pocket is
        // smaller than the nominal width and exact-fit parts bind.
        let part = part_with_width(8.0);
        assert!(!FeederKind::Tape(8).accepts(&part));
        assert!(FeederKind::Tape(12).accepts(&part));
        assert!(FeederKind::Tape(16).accepts(&part));
    }

    #[test]
    fn capacity_scales_with_tape_width() {
        assert_eq!(FeederKind::Tape(8).capacity(), 4000);
        assert_eq!(FeederKind::Tape(12).capacity(), 2500);
        assert_eq!(FeederKind::Tape(16).capacity(), 2000);
        assert_eq!(FeederKind::Tape(24).capacity(), 1000);
        assert_eq!(FeederKind::Tape(32).capacity(), 600);
        assert_eq!(FeederKind::Tape(44).capacity(), 400);
    }
}
