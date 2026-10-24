//! Machine configuration: feeder kinds (per slot, per job), cameras,
//! Z heights, speeds.
//!
//! A `FeederSlot` is just a position on the machine — it doesn't carry
//! a `FeederKind`. The physical feeder mounted in slot N varies between
//! jobs (a resistor board might use 8 mm tape slots, an LED board might
//! use 24 mm tape slots). The per-job `MachineSetup` declares which
//! `FeederKind` is mounted at each slot id for the current run.
//!
//! Cameras are similar: a job can use any of the cameras mounted on
//! the machine, and each `Placement` picks which one (so a fast vision
//! camera can handle small parts while a precise camera is reserved
//! for BGAs).

use std::collections::HashMap;

use crate::feeder::{FeederKind, PartId};
use shared_types::*;

/// One physical camera on the machine.
#[derive(Debug, Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Camera {
    pub id: u8,
    pub kind: CameraKind,
    /// XY position of the camera in machine coords. The runner drives
    /// the head here for inspection.
    pub position: Point,
    /// Sensor pitch for a FastVision line array. This is not an optical FOV.
    pub array_pitch_mm: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum CameraKind {
    /// Cheap, fast camera. Adequate for most small parts.
    FastVision,
    /// Slower, more accurate camera. Required for fine-pitch / BGA
    /// work; the precision-mode loop only runs on these.
    Precise,
    /// Used for top down vision
    HeadCamera,
}

/// Per-job declaration of which physical `FeederKind` is mounted at
/// each slot. The same physical slot can host different feeders on
/// different jobs — Job A mounts 8 mm tape at slot 0, Job B mounts
/// 16 mm tape at slot 0.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct MachineSetup {
    kind_for_slot: HashMap<u8, FeederKind>,
    /// Optional per-slot overrides for the part already loaded. When
    /// the operator has a reel in slot N for part P, the planner
    /// re-uses that slot instead of asking the operator to reload.
    loaded_for_slot: HashMap<u8, PartId>,
}

impl MachineSetup {
    pub fn new() -> Self {
        Self::default()
    }

    /// Declare that slot `slot_id` has a `kind` feeder mounted for this
    /// job.
    pub fn set_kind(&mut self, slot_id: u8, kind: FeederKind) {
        self.kind_for_slot.insert(slot_id, kind);
    }

    /// Look up the physical feeder kind for a slot. Returns `None` if
    /// the job hasn't declared a kind for this slot.
    pub fn kind(&self, slot_id: u8) -> Option<FeederKind> {
        self.kind_for_slot.get(&slot_id).copied()
    }

    /// Mark slot `slot_id` as already loaded with a reel of `part`.
    pub fn set_loaded(&mut self, slot_id: u8, part: PartId) {
        self.loaded_for_slot.insert(slot_id, part);
    }

    /// Look up the part already loaded in `slot_id`, if any.
    pub fn loaded(&self, slot_id: u8) -> Option<PartId> {
        self.loaded_for_slot.get(&slot_id).copied()
    }
}

/// Resolve the effective `FeederKind` for a slot id, falling back to
/// `Tape(8)` if the setup doesn't declare one. Used as a last-resort
/// default in the planner; the runner should normally have a setup
/// supplied.
pub fn default_kind_for(_slot_id: u8) -> FeederKind {
    FeederKind::Tape(8)
}
