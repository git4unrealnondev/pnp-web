//! Pick, place, and job-level state machines.
//!
//! Each state machine is an enum with explicit transitions. A runner
//! (out of scope here) interprets the current state, dispatches the
//! corresponding frames to `hwcontrol`, observes results, and feeds
//! them back as the next state. State machines are first-class so:
//!  - the UI can render progress;
//!  - errors are handled uniformly and can be retried / recovered;
//!  - the machine can pause / resume from any state.

use shared_types::Point;

use crate::parts::Part;
use shared_types::*;

/// Per-placement strategy. Decides what the runner does between the
/// initial pickup and the final place move, on top of the standard
/// pick / camera-inspect / correct-rotation / place flow.
#[derive(Debug, Clone, PartialEq, Default, serde::Serialize, serde::Deserialize)]
pub enum PlacementStrategy {
    /// Pick → camera → single-shot rotate correction → place.
    #[default]
    Standard,

    /// Like `Standard`, with two extra rotation knobs:
    ///
    /// - `before_camera_deg`: rotate the nozzle by this many degrees
    ///   *before* the camera inspection, so the camera sees a known
    ///   feature at a known angle. Useful for parts where the
    ///   top-down view alone can't disambiguate orientation (e.g.
    ///   symmetric packages).
    /// - `after_camera_deg`: rotate the nozzle by this many degrees
    ///   *after* the camera correction, to land a specific feature
    ///   (e.g. cathode stripe) at a specific angle on the board.
    Polarized {
        before_camera_deg: f64,
        after_camera_deg: f64,
    },

    /// Like `Standard`, but the camera → correction step is iterated:
    /// the runner re-inspects and re-corrects until both the rotation
    /// error and the centroid offset are below tolerance, or
    /// `max_attempts` inspections have been performed. Used for
    /// fine-pitch / BGA work where one-shot vision isn't enough.
    Precision {
        rotation_tolerance_deg: f64,
        offset_tolerance_mm: f64,
        max_attempts: u8,
    },
}

/// Pick cycle: from feeder to a part held under the up-camera.
#[derive(Debug, Clone, PartialEq)]
pub enum PickState {
    Idle,
    MovingToFeeder {
        slot: u8,
    },
    AdvancingFeeder {
        slot: u8,
    },
    Descending {
        nozzle: u8,
    },
    ApplyingVacuum {
        nozzle: u8,
    },
    Ascending {
        nozzle: u8,
    },
    /// Continue picking the remaining nozzles in the current bulk batch.
    BulkNext,
    MovingToCamera {
        nozzle: u8,
    },
    /// Applying the strategy's `before_camera_deg` rotation (Polarized).
    PreRotate {
        nozzle: u8,
        delta_deg: f64,
    },
    Inspecting,
    /// Camera found a rotation error; correction is being applied.
    CorrectingRotation {
        nozzle: u8,
        delta_deg: f64,
    },
    /// Applying the strategy's `after_camera_deg` rotation (Polarized),
    /// *after* the camera correction.
    PostRotate {
        nozzle: u8,
        delta_deg: f64,
    },
    /// Precision mode: re-inspecting after a correction to see if we
    /// converged.
    ReInspecting {
        attempt: u8,
        max_attempts: u8,
    },
    Done,
    Failed(PickFailure),
}

#[derive(Debug, Clone, PartialEq)]
pub enum PickFailure {
    NoPart,
    OffsetTooLarge,
    VacuumLost,
    /// Precision mode: max inspection attempts exhausted without
    /// converging below the per-placement tolerance.
    PrecisionExhausted,
}

/// Place cycle: from a part held under the camera to its design position
/// on the board.
#[derive(Debug, Clone, PartialEq)]
pub enum PlaceState {
    Idle,
    MovingToBoard { target: Point, rotation_deg: f64 },
    RotatingPart { nozzle: u8, delta_deg: f64 },
    Descending { nozzle: u8 },
    ReleasingVacuum { nozzle: u8 },
    Blowing { nozzle: u8 },
    Ascending { nozzle: u8 },
    Done,
    Failed(PlaceFailure),
}

#[derive(Debug, Clone, PartialEq)]
pub enum PlaceFailure {
    Misaligned,
    VacuumStuck,
}

/// One placement in a job, fully resolved.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Placement {
    pub part: Part,
    pub design_position: Point,
    pub design_rotation_deg: f64,
    pub feeder_slot: u8,
    pub nozzle: u8,
    /// Flipped to `true` by the runner when the part has been successfully
    /// released on the board. UI / reports use this for progress display
    /// and for resuming an interrupted job.
    pub placed: bool,
    /// If we should place this component. Missing values in older job files
    /// default to true because placement is the historical default.
    #[serde(default = "default_should_place")]
    pub should_place: bool,
    /// Per-placement behaviour: standard, polarised pre/post-rotate, or
    /// precision iterative correction.
    pub strategy: PlacementStrategy,
    /// Which camera on the machine inspects this placement. Small parts
    /// typically go to a fast vision camera (id 0); BGAs go to a
    /// precise camera (id 1). Defaults to 0.
    pub camera_kind: crate::config::CameraKind,
    /// Original designator from the placement list (`R12`, `C5`, ...).
    /// This is the operator-facing identity of the placement: value and
    /// footprint describe the *kind* of part, the reference says *which
    /// one*. Missing in older job files defaults to empty.
    #[serde(default)]
    pub reference: String,
}

fn default_should_place() -> bool {
    true
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct Job {
    pub board_id: String,
    pub placements: Vec<Placement>,
    /// Runtime state. The serialised form ignores this — loaders
    /// reset it to `JobState::Setup` so the runner drives the job
    /// from the start.
    #[serde(skip, default)]
    pub state: JobState,
}

impl Job {
    pub fn new(board_id: impl Into<String>, placements: Vec<Placement>) -> Self {
        Self {
            board_id: board_id.into(),
            placements,
            state: JobState::Setup,
        }
    }

    /// Advance the job by one outer transition. The runner is responsible
    /// for actually executing the pick / place sub-states and only calls
    /// `step` once those have completed.
    pub fn step(&mut self) -> &JobState {
        match &self.state {
            JobState::Setup => self.state = JobState::LoadingBoard,
            JobState::LoadingBoard => self.state = JobState::LocatingFiducials,
            JobState::LocatingFiducials => {
                if self.placements.is_empty() {
                    self.state = JobState::Done;
                } else {
                    self.state = JobState::Picking { index: 0 };
                }
            }
            JobState::Picking { index } => {
                let next = index + 1;
                if next >= self.placements.len() {
                    self.state = JobState::Done;
                } else {
                    self.state = JobState::Placing { index: *index };
                }
            }
            JobState::Placing { index } => {
                let next = index + 1;
                if next >= self.placements.len() {
                    self.state = JobState::Unloading;
                } else {
                    self.state = JobState::Picking { index: next };
                }
            }
            JobState::Unloading => {
                self.state = JobState::Done;
            }
            JobState::Done => {}
            JobState::Aborted { .. } => {}
        }
        &self.state
    }

    pub fn abort(&mut self, reason: impl Into<String>) {
        self.state = JobState::Aborted {
            reason: reason.into(),
        };
    }
}
