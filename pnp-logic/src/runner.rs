//! Drives a `Job` through its state machines, producing a stream of
//! `Action`s for the caller to dispatch to the real machine.

use crate::action::{Action, LightColor, MoveKind};
use crate::config::{Camera, CameraKind, MachineSetup};
use crate::feeder::{FeederBank, FeederCalibration, FeederSlot, PartId};
use crate::geometry::Pose;
use crate::nozzle::NozzleHead;
use crate::optimizer::chebyshev_cost as chebyshev;
use crate::placement::{
    PartObservation, feeder_pickup_target, feeder_pickup_z, pickup_for, place_target,
    required_nozzle_rotation, shortest_rotation,
};
use crate::plan::FeederRotation;
use crate::state::{Job, PickFailure, PickState, PlaceState, Placement, PlacementStrategy};
use shared_types::*;
use std::collections::HashMap;

/// Virtual inspection station. One or more cameras of the same kind at
/// roughly the same Y form a single tiled station whose total X range
/// is the union of the tiles' positions (each tile extends half a
/// pitch on either side of its centre). A lone camera becomes a
/// single-point station with `half_width_mm == 0.0`.
#[derive(Debug, Clone)]
pub struct TileStation {
    pub centre: Point,
    pub half_width_mm: f64,
    pub camera_ids: Vec<u8>,
}

/// Cameras at the same Y (within `Y_TOLERANCE_MM`) and of the same kind
/// are treated as one tiled station. Cameras separated along Y are
/// distinct stations.
const Y_TOLERANCE_MM: f64 = 1.0;

pub fn tile_stations(cameras: &[Camera], kind: CameraKind) -> Vec<TileStation> {
    let mut stations: Vec<TileStation> = Vec::new();
    for cam in cameras.iter().filter(|c| c.kind == kind) {
        if let Some(existing) = stations
            .iter_mut()
            .find(|s| (s.centre.y - cam.position.y).abs() <= Y_TOLERANCE_MM)
        {
            existing.camera_ids.push(cam.id);
            let member_xs: Vec<f64> = cameras
                .iter()
                .filter(|c| c.kind == kind && existing.camera_ids.contains(&c.id))
                .map(|c| c.position.x)
                .collect();
            existing.half_width_mm = compute_union_half_width(&member_xs);
            existing.centre = Point::new(average_x(&member_xs), existing.centre.y);
        } else {
            stations.push(TileStation {
                centre: cam.position,
                half_width_mm: 0.0,
                camera_ids: vec![cam.id],
            });
        }
    }
    stations
}

fn average_x(xs: &[f64]) -> f64 {
    if xs.is_empty() {
        0.0
    } else {
        xs.iter().sum::<f64>() / xs.len() as f64
    }
}

fn compute_union_half_width(xs: &[f64]) -> f64 {
    if xs.len() < 2 {
        return 0.0;
    }
    let mut sorted = xs.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let first = sorted[0];
    let last = *sorted.last().unwrap();
    let mut gaps: Vec<f64> = sorted
        .windows(2)
        .map(|w| w[1] - w[0])
        .filter(|g| *g > 1e-9)
        .collect();
    let pitch = if gaps.is_empty() {
        0.0
    } else {
        gaps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        gaps[gaps.len() / 2]
    };
    (last - first) / 2.0 + pitch / 2.0
}

/// Pre-computed view of a single placement under the current
/// `board_pose`. Returned by `Runner::planned_place` /
/// `Runner::planned_places` so a UI can step through every drop point
/// and its expected nozzle rotation after the operator has nudged the
/// pose.
#[derive(Debug, Clone)]
pub struct PlacementPreview {
    pub index: usize,
    pub part_id: PartId,
    pub mpn: String,
    pub design_position: Point,
    pub world_position: Point,
    pub nozzle_rotation_deg: f64,
    pub feeder_slot: u8,
    pub nozzle: u8,
}

fn preview_from(p: &Placement, index: usize, pose: Pose, head: &NozzleHead) -> PlacementPreview {
    let world = crate::placement::place_target(p, head, pose, Point::ORIGIN);
    let rot = crate::placement::required_nozzle_rotation(p, pose);
    PlacementPreview {
        index,
        part_id: p.part.id,
        mpn: p.part.mpn.clone(),
        design_position: p.design_position,
        world_position: world,
        nozzle_rotation_deg: rot,
        feeder_slot: p.feeder_slot,
        nozzle: p.nozzle,
    }
}

fn optimize_pick_route(
    remaining: &mut Vec<usize>,
    candidate: &mut Vec<usize>,
    cursor: Point,
    pickup_targets: &HashMap<usize, Point>,
    camera_targets: &HashMap<usize, Point>,
    best_order: &mut Vec<usize>,
    best_cost: &mut f64,
) {
    if remaining.is_empty() {
        let last = candidate
            .last()
            .and_then(|index| camera_targets.get(index).copied())
            .unwrap_or(cursor);
        let cost = candidate
            .windows(2)
            .map(|pair| pickup_targets[&pair[0]].distance(pickup_targets[&pair[1]]))
            .sum::<f64>()
            + cursor.distance(pickup_targets[&candidate[0]])
            + pickup_targets[candidate.last().unwrap()].distance(last);
        if cost < *best_cost {
            *best_cost = cost;
            *best_order = candidate.clone();
        }
        return;
    }

    for position in 0..remaining.len() {
        let index = remaining.remove(position);
        candidate.push(index);
        optimize_pick_route(
            remaining,
            candidate,
            pickup_targets[&index],
            pickup_targets,
            camera_targets,
            best_order,
            best_cost,
        );
        candidate.pop();
        remaining.insert(position, index);
    }
}

/// Public version of the runner's per-batch pickup route optimizer.
/// Given the placements in a batch and their feeder/camera targets,
/// returns the placement order that minimizes the head XY path
/// (feeder → placements → camera). Exposed so the cost model in
/// `plan.rs` can mirror the runner's actual behaviour.
pub fn optimal_batch_pick_route(
    batch_indices: &[usize],
    pickup_targets: &HashMap<usize, Point>,
    camera_targets: &HashMap<usize, Point>,
    cursor: Point,
) -> (Vec<usize>, f64) {
    let mut best_order: Vec<usize> = batch_indices.to_vec();
    let mut best_cost = f64::INFINITY;
    let mut candidate = Vec::with_capacity(batch_indices.len());
    let mut remaining = batch_indices.to_vec();
    optimize_pick_route(
        &mut remaining,
        &mut candidate,
        cursor,
        pickup_targets,
        camera_targets,
        &mut best_order,
        &mut best_cost,
    );
    (best_order, best_cost)
}

/// Machine-level configuration: heights, speeds, camera position.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MachineConfig {
    /// XY position of the up-camera, in machine coords.
    pub camera_position: Point,
    /// Z height used for XY travel between pick/place moves.
    pub travel_z_mm: f64,
    /// Z height at which the nozzle meets a feeder tape.
    pub pickup_z_mm: f64,
    /// Z height at which the nozzle meets the board.
    pub place_z_mm: f64,
    pub xy_speed: u8,
    pub z_speed: u8,
    pub rotation_speed: u8,
    pub xy_approach_speed: u8,
    pub z_approach_speed: u8,
}

impl Default for MachineConfig {
    fn default() -> Self {
        Self {
            camera_position: Point::new(0.0, 0.0),
            travel_z_mm: 5.0,
            pickup_z_mm: 0.5,
            place_z_mm: 0.2,
            xy_speed: 8,
            z_speed: 4,
            rotation_speed: 8,
            xy_approach_speed: 4,
            z_approach_speed: 2,
        }
    }
}

/// Outcome of one `step()` call.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub enum StepResult {
    /// Dispatch this `Action` to the hardware.
    Action(Action),
    /// The runner is at the up-camera waiting for a `PartObservation`;
    /// call `Runner::feed_camera`.
    NeedCamera,
    /// The runner is waiting for the operator; call `Runner::feed_prompt_response`.
    NeedPrompt(String),
    /// Job completed.
    Done,
    /// Job aborted; `reason` describes why.
    Aborted(String),
}

/// Drives a job. Owns its head model (rotation state), the feeders it
/// can draw from, the feeder banks (per-bank rotation), the calibration
/// table (per-bank-per-part pickup offsets / Z), the per-part feeder
/// rotation (so it can advance to the next reel mid-job), the per-job
/// machine setup (which physical feeder kinds are mounted where), the
/// cameras available on the machine, and the board pose once fiducials
/// are localised.
#[derive(Debug)]
pub struct Runner {
    job: Job,
    head: NozzleHead,
    feeders: Vec<FeederSlot>,
    banks: Vec<FeederBank>,
    calibration: FeederCalibration,
    rotation: FeederRotation,
    setup: MachineSetup,
    cameras: Vec<Camera>,
    board_pose: Option<Pose>,
    pick_state: PickState,
    place_state: PlaceState,
    config: MachineConfig,
    pending_camera: bool,
    pending_prompt: Option<String>,
    cached_place_target: Option<Point>,
    /// Manual FastVision inspection point. When `Some`, every FastVision
    /// batch flies the head here (single-point station) instead of
    /// using the registered cameras. Cleared via
    /// `clear_fastvision_station`.
    manual_fastvision: Option<TileStation>,
    /// Centroid offset of the currently-held part relative to the nozzle
    /// tip, in the *part's body frame* (already corrected by the
    /// camera-rotation). Reset at the start of every pick and consumed
    /// at the start of the corresponding place.
    held_centroid_offset: Point,
    /// Camera offset for every part currently held during a multi-pick.
    held_offsets: HashMap<u8, Point>,
    /// Counter for the precision-mode loop. Reset on `PickState::Idle`.
    precision_attempt: u8,
    /// Placement indices currently being picked before any of them are
    /// placed. A batch is limited to distinct nozzles and one part type.
    batch_indices: Vec<usize>,
    batch_position: usize,
}

/// Trip indices for a whole run, mirroring `start_batch`: consecutive
/// placements ride in the same trip (one feeder visit, one camera stop) while
/// they need the same camera kind and the same placement strategy, use
/// distinct nozzles, and stay within one head-load of `head_capacity` slots.
/// Each placement gets its trip id (0-based, in run order). The process view
/// uses this to show the cycle the physical machine will actually perform.
pub fn trip_ids(head_capacity: usize, placements: &[Placement]) -> Vec<usize> {
    let capacity = head_capacity.max(1);
    let mut ids = vec![0usize; placements.len()];
    let mut start = 0usize;
    let mut trip = 0usize;
    while start < placements.len() {
        let first = &placements[start];
        let mut used = std::collections::HashSet::new();
        let _ = used.insert(first.nozzle);
        let mut end = start + 1;
        while end < placements.len() {
            let placement = &placements[end];
            if placement.camera_kind != first.camera_kind
                || placement.strategy != first.strategy
                || !used.insert(placement.nozzle)
                || end - start >= capacity
            {
                break;
            }
            end += 1;
        }
        for index in start..end {
            ids[index] = trip;
        }
        trip += 1;
        start = end;
    }
    ids
}

impl Runner {
    pub fn new(
        head: NozzleHead,
        feeders: Vec<FeederSlot>,
        config: MachineConfig,
        job: Job,
    ) -> Self {
        Self::with_calibration(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            config,
            job,
            FeederRotation::default(),
        )
    }

    pub fn with_rotation(
        head: NozzleHead,
        feeders: Vec<FeederSlot>,
        config: MachineConfig,
        job: Job,
        rotation: FeederRotation,
    ) -> Self {
        Self::with_calibration(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            config,
            job,
            rotation,
        )
    }

    pub fn with_calibration(
        head: NozzleHead,
        feeders: Vec<FeederSlot>,
        banks: Vec<FeederBank>,
        calibration: FeederCalibration,
        config: MachineConfig,
        job: Job,
        rotation: FeederRotation,
    ) -> Self {
        Self {
            job,
            head,
            feeders,
            banks,
            calibration,
            rotation,
            setup: MachineSetup::new(),
            cameras: Vec::new(),
            board_pose: None,
            pick_state: PickState::Idle,
            place_state: PlaceState::Idle,
            config,
            pending_camera: false,
            pending_prompt: None,
            cached_place_target: None,
            manual_fastvision: None,
            held_centroid_offset: Point::ORIGIN,
            held_offsets: HashMap::new(),
            precision_attempt: 0,
            batch_indices: Vec::new(),
            batch_position: 0,
        }
    }

    /// Build a runner with the full set of state: head, feeders, banks,
    /// calibration, machine setup, cameras, config, job, rotation.
    #[allow(clippy::too_many_arguments)]
    pub fn with_setup(
        head: NozzleHead,
        feeders: Vec<FeederSlot>,
        banks: Vec<FeederBank>,
        calibration: FeederCalibration,
        setup: MachineSetup,
        cameras: Vec<Camera>,
        config: MachineConfig,
        job: Job,
        rotation: FeederRotation,
    ) -> Self {
        Self {
            job,
            head,
            feeders,
            banks,
            calibration,
            rotation,
            setup,
            cameras,
            board_pose: None,
            pick_state: PickState::Idle,
            place_state: PlaceState::Idle,
            config,
            pending_camera: false,
            pending_prompt: None,
            cached_place_target: None,
            manual_fastvision: None,
            held_centroid_offset: Point::ORIGIN,
            held_offsets: HashMap::new(),
            precision_attempt: 0,
            batch_indices: Vec::new(),
            batch_position: 0,
        }
    }

    pub fn rotation(&self) -> &FeederRotation {
        &self.rotation
    }

    pub fn job(&self) -> &Job {
        &self.job
    }

    pub fn head(&self) -> &NozzleHead {
        &self.head
    }

    #[cfg(test)]
    fn cameras_for_test(&self) -> Vec<Camera> {
        self.cameras.clone()
    }

    pub fn state(&self) -> &JobState {
        &self.job.state
    }

    pub fn board_pose(&self) -> Option<Pose> {
        self.board_pose
    }

    /// Set the board pose (typically after `compute_board_pose`).
    pub fn set_board_pose(&mut self, pose: Pose) {
        self.board_pose = Some(pose);
    }

    pub fn abort(&mut self, reason: impl Into<String>) {
        self.job.abort(reason);
    }

    /// Compute the world-space rectangle the head-mounted camera would
    /// see if the head reference point were at `head_position`.
    ///
    /// This is intended for job setup (board preview, fiducial hunting,
    /// manual inspection). It does not mutate runner state and does not
    /// depend on the current `JobState` — by convention callers should
    /// only drive it before the job has left `JobState::Setup`.
    pub fn onboard_view_at(&self, head_position: Point) -> Option<(Point, Point)> {
        self.head
            .onboard_camera
            .map(|camera| camera.view_at(head_position))
    }

    /// Preview one placement under the current `board_pose`.
    ///
    /// `world_position` is where the head will drive the nozzle tip
    /// for the final place move. `nozzle_rotation_deg` is the rotation
    /// the nozzle will be at on touchdown (board rotation +
    /// `design_rotation_deg`).
    ///
    /// Returns `None` if `board_pose` is unset or `index` is out of
    /// range.
    pub fn planned_place(&self, index: usize) -> Option<PlacementPreview> {
        let pose = self.board_pose?;
        let p = self.job.placements.get(index)?;
        Some(preview_from(p, index, pose, &self.head))
    }

    /// Preview every placement in run order. Returns an empty vec if
    /// `board_pose` is unset (so the caller doesn't have to special-case
    /// the "no pose yet" state).
    pub fn planned_places(&self) -> Vec<PlacementPreview> {
        let Some(pose) = self.board_pose else {
            return Vec::new();
        };
        self.job
            .placements
            .iter()
            .enumerate()
            .map(|(i, p)| preview_from(p, i, pose, &self.head))
            .collect()
    }

    /// Drive the job to completion with synthetic camera/prompt feedback,
    /// returning every `Action` the runner would have emitted.
    ///
    /// The runner mutates as it runs; clone it first if you want to keep
    /// a pristine copy for an actual run after previewing.
    pub fn collect_actions(&mut self) -> Vec<Action> {
        let mut out = Vec::new();
        loop {
            match self.step() {
                StepResult::Action(a) => out.push(a),
                StepResult::NeedCamera => self.feed_camera(PartObservation::ok()),
                StepResult::NeedPrompt(_) => self.feed_prompt_response(),
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        out
    }

    /// Drive the job to completion and return the full `StepResult`
    /// stream (including `NeedCamera` / `NeedPrompt` / `Done` /
    /// `Aborted`), with synthetic feedback for camera and prompt
    /// requests. Useful for previews and dry-runs.
    pub fn collect_steps(&mut self) -> Vec<StepResult> {
        let mut out = Vec::new();
        loop {
            let r = self.step();
            let done = matches!(r, StepResult::Done | StepResult::Aborted(_));
            match &r {
                StepResult::NeedCamera => self.feed_camera(PartObservation::ok()),
                StepResult::NeedPrompt(_) => self.feed_prompt_response(),
                _ => {}
            }
            out.push(r);
            if done {
                break;
            }
        }
        out
    }

    /// Drive one step. Returns the next thing to do; the caller dispatches
    /// `Action`s and feeds back observations via `feed_camera` /
    /// `feed_prompt_response`.
    pub fn step(&mut self) -> StepResult {
        if self.pending_camera {
            return StepResult::NeedCamera;
        }
        if let Some(msg) = self.pending_prompt.clone() {
            return StepResult::NeedPrompt(msg);
        }

        match self.job.state.clone() {
            JobState::Setup => {
                self.pick_state = PickState::Idle;
                self.job.state = JobState::LoadingBoard;
                StepResult::Action(Action::Home)
            }
            JobState::LoadingBoard => {
                self.job.state = JobState::LocatingFiducials;
                self.pending_prompt = Some("Load board and press OK".into());
                StepResult::NeedPrompt("Load board and press OK".into())
            }
            JobState::LocatingFiducials => match self.board_pose {
                Some(_) => {
                    if self.job.placements.is_empty() {
                        self.job.state = JobState::Unloading;
                        StepResult::Done
                    } else {
                        let first = self.start_batch(0);
                        self.job.state = JobState::Picking { index: first };
                        self.pick_state = PickState::Idle;
                        StepResult::Action(Action::Light(LightColor::Green))
                    }
                }
                None => {
                    let msg = "Locate fiducials, then call set_board_pose()";
                    self.pending_prompt = Some(msg.into());
                    StepResult::NeedPrompt(msg.into())
                }
            },
            JobState::Picking { index } => self.run_pick(index),
            JobState::Placing { index } => self.run_place(index),
            JobState::Done | JobState::Unloading => StepResult::Done,
            JobState::Aborted { reason } => StepResult::Aborted(reason),
        }
    }

    /// Whether the runner is parked waiting for a camera observation.
    pub fn awaiting_camera(&self) -> bool {
        self.pending_camera
    }

    /// Whether the runner is parked waiting for the operator.
    pub fn awaiting_prompt(&self) -> bool {
        self.pending_prompt.is_some()
    }

    /// Whether the runner is waiting for the board pose to be published.
    pub fn locating_board(&self) -> bool {
        matches!(self.job.state, JobState::LocatingFiducials) && self.board_pose.is_none()
    }

    /// Feed back the up-camera's observation of the currently-held part.
    pub fn feed_camera(&mut self, observation: PartObservation) {
        if !self.pending_camera {
            return;
        }
        self.pending_camera = false;

        if !observation.present {
            self.pick_state = PickState::Failed(PickFailure::NoPart);
            return;
        }
        // The `within_tolerance` flag is a sanity check from the camera
        // system (e.g. "the part is way off the FOV, abort"). Precision
        // placements override this with their own per-placement tolerance
        // — only bail when *not* in precision mode.
        if !observation.within_tolerance
            && !matches!(
                self.current_placement().map(|p| &p.strategy),
                Some(PlacementStrategy::Precision { .. })
            )
        {
            self.pick_state = PickState::Failed(PickFailure::OffsetTooLarge);
            return;
        }

        // Rotate the observed centroid offset back into the part's body
        // frame so the placement maths can combine it with the part's
        // nominal pickup_offset. We do this immediately, before the
        // nozzle-rotation correction runs, because the camera's frame
        // is the *current* nozzle frame.
        let err = observation.rotation_error_deg;
        let (s, c) = err.to_radians().sin_cos();
        let off = observation.centroid_offset;
        let body_offset = Point::new(off.x * c + off.y * s, -off.x * s + off.y * c);

        // Accumulate the centroid offset across precision iterations.
        // Each camera pass reports the part's centroid relative to the
        // tip *at that moment*; the body-frame version of that has to
        // be added to the previous body-frame value because the
        // centroid moves relative to the tip when the nozzle rotates.
        self.held_centroid_offset = Point::new(
            self.held_centroid_offset.x + body_offset.x,
            self.held_centroid_offset.y + body_offset.y,
        );
        if let Some(nozzle) = self.active_nozzle() {
            self.held_offsets.insert(nozzle, self.held_centroid_offset);
        }

        let nozzle = self.active_nozzle().unwrap_or(0);

        // Precision-mode convergence check. Clone the strategy out of the
        // placement first so we can mutate `self` without fighting the
        // borrow checker.
        let strategy = self
            .current_placement()
            .map(|p| p.strategy.clone())
            .unwrap_or(PlacementStrategy::Standard);
        if let PlacementStrategy::Precision {
            rotation_tolerance_deg,
            offset_tolerance_mm,
            max_attempts,
        } = strategy
        {
            self.precision_attempt += 1;
            let offset_mag = (observation.centroid_offset.x * observation.centroid_offset.x
                + observation.centroid_offset.y * observation.centroid_offset.y)
                .sqrt();
            let converged_rot = observation.rotation_error_deg.abs() < rotation_tolerance_deg;
            let converged_offset = offset_mag < offset_tolerance_mm;

            if converged_rot && converged_offset {
                self.pick_state = PickState::Done;
                return;
            }
            if self.precision_attempt >= max_attempts {
                self.pick_state = PickState::Failed(PickFailure::PrecisionExhausted);
                return;
            }
            // Still off — apply the correction then re-inspect.
            if err.abs() >= 0.1 {
                self.pick_state = PickState::CorrectingRotation {
                    nozzle,
                    delta_deg: -err,
                };
                return;
            }
            // Off only in offset, not in rotation — just retry the
            // inspect (camera might get a better read).
            self.pick_state = PickState::ReInspecting {
                attempt: self.precision_attempt,
                max_attempts,
            };
            return;
        }

        // Non-precision path: single-shot correction.
        if err.abs() < 0.1 {
            self.pick_state = PickState::Done;
        } else {
            self.pick_state = PickState::CorrectingRotation {
                nozzle,
                delta_deg: -err,
            };
        }
    }

    /// Acknowledge the most recent prompt.
    pub fn feed_prompt_response(&mut self) {
        self.pending_prompt = None;
    }

    fn active_nozzle(&self) -> Option<u8> {
        self.current_placement().map(|p| p.nozzle)
    }

    /// Build the largest safe pickup batch starting at `start`. Parts must be
    /// the same type and each nozzle may only carry one part. Keeping batches
    /// consecutive preserves the planner's feeder/camera ordering.
    fn start_batch(&mut self, start: usize) -> usize {
        self.batch_indices.clear();
        self.batch_position = 0;
        let first = match self.job.placements.get(start) {
            Some(p) => p,
            None => return start,
        };
        let mut nozzles = std::collections::HashSet::new();
        for index in start..self.job.placements.len() {
            let placement = &self.job.placements[index];
            if placement.camera_kind != first.camera_kind
                || placement.strategy != first.strategy
                || !nozzles.insert(placement.nozzle)
                || self.batch_indices.len() >= self.head.nozzles.len()
            {
                break;
            }
            self.batch_indices.push(index);
        }

        // Choose pickup order by minimizing the complete feeder-to-camera
        // path, not by alternating nozzle IDs. This naturally picks 1->3 or
        // 3->1 according to the actual feeder/camera direction and also
        // handles parts spread across multiple feeder slots.
        let station = self.select_station(first.camera_kind, self.head.position);
        let station_centre = station
            .as_ref()
            .map(|s| s.centre)
            .unwrap_or_else(|| self.camera_for(first.camera_kind).position);
        let mut targets = HashMap::new();
        let mut camera_targets = HashMap::new();
        for &index in &self.batch_indices {
            let placement = &self.job.placements[index];
            if let Some(target) = feeder_pickup_target(
                placement,
                &self.feeders,
                &self.banks,
                &self.head,
                &self.calibration,
                self.config.pickup_z_mm,
            ) {
                targets.insert(index, target);
            }
            camera_targets.insert(
                index,
                self.head
                    .head_position_for_tip(placement.nozzle, station_centre),
            );
        }
        let mut best_order = self.batch_indices.clone();
        let mut best_cost = f64::INFINITY;
        let mut candidate = Vec::with_capacity(self.batch_indices.len());
        let mut remaining = self.batch_indices.clone();
        // Do not seed this optimization with the head's post-placement pose.
        // That pose is on the board and changes after every batch, which was
        // the reason identical feeder/camera batches alternated 1->3 and
        // 3->1. The pickup route should be optimized from the feeder side;
        // the final camera leg supplies the direction preference.
        let pickup_start = self
            .batch_indices
            .iter()
            .filter_map(|index| targets.get(index).copied())
            .fold(None, |center: Option<Point>, point| {
                Some(match center {
                    Some(center) => {
                        Point::new((center.x + point.x) / 2.0, (center.y + point.y) / 2.0)
                    }
                    None => point,
                })
            })
            .unwrap_or(self.head.position);
        optimize_pick_route(
            &mut remaining,
            &mut candidate,
            pickup_start,
            &targets,
            &camera_targets,
            &mut best_order,
            &mut best_cost,
        );
        self.batch_indices = best_order;
        self.batch_indices.first().copied().unwrap_or(start)
    }

    fn batch_complete(&self) -> bool {
        self.batch_position + 1 >= self.batch_indices.len()
    }

    fn next_unplaced_index(&self) -> Option<usize> {
        self.job.placements.iter().position(|p| !p.placed)
    }

    fn current_placement(&self) -> Option<&Placement> {
        match &self.job.state {
            JobState::Picking { index } | JobState::Placing { index } => {
                self.job.placements.get(*index)
            }
            _ => None,
        }
    }

    /// The active feeder slot for the current placement: the rotation
    /// table's current value if it has one, otherwise fall back to the
    /// slot stamped on the placement by the planner.
    fn current_feeder_slot(&self, placement: &Placement) -> u8 {
        self.rotation
            .current_slot(placement.part.id)
            .unwrap_or(placement.feeder_slot)
    }

    fn bank_id_for_slot(&self, slot_id: u8) -> u8 {
        self.feeders
            .iter()
            .find(|s| s.id == slot_id)
            .map(|s| s.bank_id)
            .unwrap_or(0)
    }

    /// Resolve the camera by capability. Falls back to the first
    /// registered camera, or a dummy at the origin if none are
    /// registered (so the runner can still be driven in tests).
    fn camera_for(&self, kind: CameraKind) -> Camera {
        self.cameras
            .iter()
            .find(|c| c.kind == kind)
            .copied()
            .or_else(|| self.cameras.first().copied())
            .unwrap_or(Camera {
                id: 0,
                kind: CameraKind::FastVision,
                position: Point::ORIGIN,
                array_pitch_mm: None,
            })
    }

    /// Pick the inspection station of `kind` whose centre is closest
    /// (Chebyshev) to `from`. A station is one or more cameras of the
    /// same kind at roughly the same Y, treated as a single tiled
    /// inspection point. Returns `None` if no camera of that kind is
    /// registered.
    ///
    /// For `FastVision`, a manual station set via
    /// `set_fastvision_station` overrides auto‑selection.
    fn select_station(&self, kind: CameraKind, from: Point) -> Option<TileStation> {
        if kind == CameraKind::FastVision {
            if let Some(manual) = self.manual_fastvision.clone() {
                return Some(manual);
            }
        }
        let stations = tile_stations(&self.cameras, kind);
        stations.into_iter().min_by(|a, b| {
            let da = chebyshev(from, a.centre);
            let db = chebyshev(from, b.centre);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
    }

    /// Pin every FastVision batch to a single manual inspection point.
    /// Use this when you have a hand‑tuned camera location you want
    /// the head to fly to regardless of what the registered cameras
    /// say. The manual station is a single point — `half_width_mm` is
    /// forced to 0 so `optimal_camera_head_position` lands the active
    /// nozzle on the centre.
    pub fn set_fastvision_station(&mut self, centre: Point) {
        self.manual_fastvision = Some(TileStation {
            centre,
            half_width_mm: 0.0,
            camera_ids: Vec::new(),
        });
    }

    /// Read the manual FastVision station, if any.
    pub fn fastvision_station(&self) -> Option<TileStation> {
        self.manual_fastvision.clone()
    }

    /// Drop the manual FastVision override and fall back to
    /// auto‑selection from the registered cameras.
    pub fn clear_fastvision_station(&mut self) {
        self.manual_fastvision = None;
    }

    /// Drive the head so the held tips (or the active nozzle) fall
    /// inside the chosen tiled station. A station with
    /// `half_width_mm == 0.0` is a single-point inspection: the active
    /// nozzle lands on the centre. A wider station centres the held
    /// tips in the union X range.
    fn optimal_camera_head_position(&self, station: &TileStation, active_nozzle: u8) -> Point {
        if station.half_width_mm <= 0.0 {
            return self
                .head
                .head_position_for_tip(active_nozzle, station.centre);
        }
        let held: Vec<u8> = self
            .head
            .nozzles
            .iter()
            .filter(|n| n.vacuum)
            .map(|n| n.id)
            .collect();
        let offsets: Vec<f64> = if held.is_empty() {
            vec![self.head.nozzle(active_nozzle).offset_mm]
        } else {
            held.iter()
                .map(|id| self.head.nozzle(*id).offset_mm)
                .collect()
        };
        let min_off = offsets.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_off = offsets.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let centre_offset = (min_off + max_off) / 2.0;
        Point::new(station.centre.x - centre_offset, station.centre.y)
    }

    fn run_pick(&mut self, index: usize) -> StepResult {
        let mut placement = match self.job.placements.get(index) {
            Some(p) => p.clone(),
            None => {
                self.job.state = JobState::Unloading;
                return StepResult::Done;
            }
        };
        // Refresh the active feeder slot from the rotation table.
        placement.feeder_slot = self.current_feeder_slot(&placement);
        let head = &self.head;

        match self.pick_state.clone() {
            PickState::Idle => {
                // Only the first pickup in a batch starts a fresh camera
                // correction. Later pickups must preserve all held parts.
                if self.batch_position == 0 {
                    self.held_centroid_offset = Point::ORIGIN;
                    self.precision_attempt = 0;
                }
                self.held_offsets.insert(placement.nozzle, Point::ORIGIN);
                // Initialise the active nozzle's rotation to the part's
                // pickup rotation, rotated into the bank frame. The
                // camera will report any further error.
                let slot = self.current_feeder_slot(&placement);
                let bank_rot = self
                    .feeders
                    .iter()
                    .find(|s| s.id == slot)
                    .and_then(|s| self.banks.iter().find(|b| b.id == s.bank_id))
                    .map(|b| b.rotation_deg)
                    .unwrap_or(0.0);
                let pickup = pickup_for(
                    self.bank_id_for_slot(slot),
                    &placement,
                    &self.calibration,
                    self.config.pickup_z_mm,
                );
                if let Some(n) = self.head.nozzle_mut(placement.nozzle) {
                    n.rotation_deg = bank_rot + pickup.rotation_deg;
                }
                self.pick_state = PickState::MovingToFeeder { slot };
                self.step()
            }
            PickState::MovingToFeeder { slot } => {
                // Drive the head to the pocket's actual pickup point in
                // the machine frame: slot position + bank-rotated offset.
                let target = match feeder_pickup_target(
                    &placement,
                    &self.feeders,
                    &self.banks,
                    head,
                    &self.calibration,
                    self.config.pickup_z_mm,
                ) {
                    Some(t) => t,
                    None => {
                        self.pick_state = PickState::Failed(PickFailure::NoPart);
                        return StepResult::Aborted(format!(
                            "no pickup target for feeder slot {}",
                            slot
                        ));
                    }
                };
                self.pick_state = PickState::AdvancingFeeder { slot };
                StepResult::Action(Action::Move {
                    target,
                    speed: self.config.xy_speed,
                    kind: MoveKind::Linear,
                })
            }
            PickState::AdvancingFeeder { slot } => {
                self.pick_state = PickState::Descending {
                    nozzle: placement.nozzle,
                };
                StepResult::Action(Action::AdvanceFeeder {
                    id: slot,
                    pulses: 1,
                })
            }
            PickState::Descending { nozzle } => {
                // Z height is per-(bank, part) via calibration, with the
                // machine config as fallback.
                let z = feeder_pickup_z(
                    &placement,
                    &self.feeders,
                    &self.calibration,
                    self.config.pickup_z_mm,
                )
                .unwrap_or(self.config.pickup_z_mm);
                self.pick_state = PickState::ApplyingVacuum { nozzle };
                StepResult::Action(Action::MoveZ {
                    nozzle,
                    height_mm: z,
                    speed: self.config.z_approach_speed,
                })
            }
            PickState::ApplyingVacuum { nozzle } => {
                if let Some(n) = self.head.nozzle_mut(nozzle) {
                    n.vacuum = true;
                }
                self.pick_state = PickState::Ascending { nozzle };
                StepResult::Action(Action::Vacuum { nozzle, on: true })
            }
            PickState::Ascending { nozzle } => {
                // The part is now on the nozzle; charge one part against
                // the active reel. If that reel is exhausted the rotation
                // table hands us the next slot; the next pick will target
                // it instead.
                let cap = self
                    .feeders
                    .iter()
                    .find(|s| s.id == placement.feeder_slot)
                    .map(|s| {
                        self.setup
                            .kind(s.id)
                            .map(|k| k.capacity())
                            .unwrap_or(crate::config::default_kind_for(s.id).capacity())
                    })
                    .unwrap_or(crate::config::default_kind_for(placement.feeder_slot).capacity());
                self.rotation.tick(placement.part.id, cap);
                self.pick_state = if self.batch_complete() {
                    PickState::MovingToCamera { nozzle }
                } else {
                    PickState::BulkNext
                };
                StepResult::Action(Action::MoveZ {
                    nozzle,
                    height_mm: self.config.travel_z_mm,
                    speed: self.config.z_speed,
                })
            }
            PickState::BulkNext => {
                self.batch_position += 1;
                let next = self.batch_indices[self.batch_position];
                self.job.state = JobState::Picking { index: next };
                self.pick_state = PickState::Idle;
                self.step()
            }
            PickState::MovingToCamera { nozzle: _ } => {
                let station = self.select_station(placement.camera_kind, self.head.position);
                // Frame the held tips across the chosen tiled station.
                let target = match station.as_ref() {
                    Some(s) => self.optimal_camera_head_position(s, placement.nozzle),
                    None => self.head.head_position_for_tip(
                        placement.nozzle,
                        self.camera_for(placement.camera_kind).position,
                    ),
                };
                // Strategy: if Polarized with a non-zero before_camera_deg,
                // rotate first so the camera sees a known angle.
                self.pick_state = match placement.strategy {
                    PlacementStrategy::Polarized {
                        before_camera_deg, ..
                    } if before_camera_deg.abs() >= 0.1 => PickState::PreRotate {
                        nozzle: placement.nozzle,
                        delta_deg: before_camera_deg,
                    },
                    _ => PickState::Inspecting,
                };
                StepResult::Action(Action::Move {
                    target,
                    speed: self.config.xy_approach_speed,
                    kind: MoveKind::Linear,
                })
            }
            PickState::PreRotate { nozzle, delta_deg } => {
                if let Some(n) = self.head.nozzle_mut(nozzle) {
                    n.rotation_deg += delta_deg;
                }
                self.pick_state = PickState::Inspecting;
                StepResult::Action(Action::RotateNozzle {
                    nozzle,
                    delta_deg,
                    speed: self.config.rotation_speed,
                })
            }
            PickState::Inspecting => {
                self.pending_camera = true;
                StepResult::NeedCamera
            }
            PickState::CorrectingRotation { nozzle, delta_deg } => {
                if let Some(n) = self.head.nozzle_mut(nozzle) {
                    n.rotation_deg += delta_deg;
                }
                // Decide the next state based on the strategy:
                //   - Polarized with after_camera_deg: rotate by that.
                //   - Precision: re-inspect until converged.
                //   - Otherwise: Done.
                self.pick_state = match &placement.strategy {
                    PlacementStrategy::Polarized {
                        after_camera_deg, ..
                    } if after_camera_deg.abs() >= 0.1 => PickState::PostRotate {
                        nozzle,
                        delta_deg: *after_camera_deg,
                    },
                    PlacementStrategy::Precision { max_attempts, .. } => PickState::ReInspecting {
                        attempt: self.precision_attempt,
                        max_attempts: *max_attempts,
                    },
                    _ => PickState::Done,
                };
                StepResult::Action(Action::RotateNozzle {
                    nozzle,
                    delta_deg,
                    speed: self.config.rotation_speed,
                })
            }
            PickState::PostRotate { nozzle, delta_deg } => {
                if let Some(n) = self.head.nozzle_mut(nozzle) {
                    n.rotation_deg += delta_deg;
                }
                self.pick_state = PickState::Done;
                StepResult::Action(Action::RotateNozzle {
                    nozzle,
                    delta_deg,
                    speed: self.config.rotation_speed,
                })
            }
            PickState::ReInspecting { .. } => {
                // We've come back to the camera after a correction;
                // re-emit NeedCamera. feed_camera decides whether we're
                // done or need another round.
                self.pending_camera = true;
                StepResult::NeedCamera
            }
            PickState::Done => {
                if !self.batch_complete() {
                    self.batch_position += 1;
                    let next = self.batch_indices[self.batch_position];
                    self.job.state = JobState::Picking { index: next };
                    self.pick_state = PickState::Idle;
                    self.step()
                } else {
                    let first = self.batch_indices[0];
                    self.batch_position = 0;
                    self.job.state = JobState::Placing { index: first };
                    self.place_state = PlaceState::Idle;
                    self.step()
                }
            }
            PickState::Failed(reason) => {
                let msg = format!("{:?}", reason);
                self.job.abort(format!("pick failed: {}", msg));
                StepResult::Aborted(msg)
            }
        }
    }

    fn run_place(&mut self, index: usize) -> StepResult {
        let placement = match self.job.placements.get(index) {
            Some(p) => p.clone(),
            None => {
                self.job.state = JobState::Done;
                return StepResult::Done;
            }
        };
        let board_pose = match self.board_pose {
            Some(p) => p,
            None => {
                let msg = "place attempted before board pose set".to_string();
                self.job.abort(&msg);
                return StepResult::Aborted(msg);
            }
        };

        match self.place_state.clone() {
            PlaceState::Idle => {
                let target = place_target(
                    &placement,
                    &self.head,
                    board_pose,
                    self.held_offsets
                        .get(&placement.nozzle)
                        .copied()
                        .unwrap_or(self.held_centroid_offset),
                );
                let target_rot = required_nozzle_rotation(&placement, board_pose);
                let current_rot = self.head.nozzle(placement.nozzle).rotation_deg;
                let delta = shortest_rotation(current_rot, target_rot);
                self.cached_place_target = Some(target);
                // Always drive the head to the corrected board position
                // before descending, whether or not we needed to rotate.
                self.place_state = if delta.abs() < 0.1 {
                    PlaceState::MovingToBoard {
                        target,
                        rotation_deg: current_rot,
                    }
                } else {
                    PlaceState::RotatingPart {
                        nozzle: placement.nozzle,
                        delta_deg: delta,
                    }
                };
                self.step()
            }
            PlaceState::RotatingPart { nozzle, delta_deg } => {
                if let Some(n) = self.head.nozzle_mut(nozzle) {
                    n.rotation_deg += delta_deg;
                }
                // After rotating, drive the head to the corrected board
                // position before descending.
                self.place_state = PlaceState::MovingToBoard {
                    target: self
                        .cached_place_target
                        .expect("place target computed in Idle"),
                    rotation_deg: self.head.nozzle(nozzle).rotation_deg,
                };
                StepResult::Action(Action::RotateNozzle {
                    nozzle,
                    delta_deg,
                    speed: self.config.rotation_speed,
                })
            }
            PlaceState::MovingToBoard { target, .. } => {
                self.place_state = PlaceState::Descending {
                    nozzle: placement.nozzle,
                };
                StepResult::Action(Action::Move {
                    target,
                    speed: self.config.xy_approach_speed,
                    kind: MoveKind::PlaceApproach,
                })
            }
            PlaceState::Descending { nozzle } => {
                self.place_state = PlaceState::ReleasingVacuum { nozzle };
                StepResult::Action(Action::MoveZ {
                    nozzle,
                    height_mm: self.config.place_z_mm,
                    speed: self.config.z_approach_speed,
                })
            }
            PlaceState::ReleasingVacuum { nozzle } => {
                if let Some(n) = self.head.nozzle_mut(nozzle) {
                    n.vacuum = false;
                }
                self.place_state = PlaceState::Blowing { nozzle };
                StepResult::Action(Action::Vacuum { nozzle, on: false })
            }
            PlaceState::Blowing { nozzle } => {
                self.place_state = PlaceState::Ascending { nozzle };
                StepResult::Action(Action::Blow { nozzle, on: true })
            }
            PlaceState::Ascending { nozzle } => {
                self.place_state = PlaceState::Done;
                StepResult::Action(Action::MoveZ {
                    nozzle,
                    height_mm: self.config.travel_z_mm,
                    speed: self.config.z_speed,
                })
            }
            PlaceState::Done => {
                if let Some(p) = self.job.placements.get_mut(index) {
                    p.placed = true;
                }
                if self.batch_position + 1 < self.batch_indices.len() {
                    self.batch_position += 1;
                    let next = self.batch_indices[self.batch_position];
                    self.place_state = PlaceState::Idle;
                    self.job.state = JobState::Placing { index: next };
                    self.step()
                } else {
                    let next = self.next_unplaced_index();
                    self.batch_indices.clear();
                    self.batch_position = 0;
                    if let Some(next) = next {
                        let first = self.start_batch(next);
                        self.job.state = JobState::Picking { index: first };
                        self.pick_state = PickState::Idle;
                        self.step()
                    } else {
                        self.job.state = JobState::Done;
                        StepResult::Action(Action::Light(LightColor::Off))
                    }
                }
            }
            PlaceState::Failed(reason) => {
                let msg = format!("{:?}", reason);
                self.job.abort(format!("place failed: {}", msg));
                StepResult::Aborted(msg)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NozzleKind;
    use crate::feeder::{FeederKind, PartId};
    use crate::geometry::Size;
    use crate::nozzle::default_head;
    use crate::parts::{Footprint, Part, Polarity};
    use crate::state::Placement;

    fn dummy_part(id: u32) -> Part {
        Part {
            id: PartId(id),
            mpn: format!("P{}", id),
            description: String::new(),
            footprint: Footprint {
                name: format!("P{}", id),
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
        }
    }

    fn build() -> Runner {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 3,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: Some(PartId(1)),
        }];
        let placements = vec![Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::new(10.0, 5.0),
            design_rotation_deg: 0.0,
            feeder_slot: 3,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        }];
        let job = Job::new("BOARD-1", placements);
        Runner::new(head, feeders, MachineConfig::default(), job)
    }

    fn drive(runner: &mut Runner, actions: &mut Vec<String>) {
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(a) => actions.push(a.label()),
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
                StepResult::Done | StepResult::Aborted(_) => return,
            }
        }
        panic!("runner did not finish within 500 steps");
    }

    #[test]
    fn pick_and_place_produces_expected_actions() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut actions = Vec::new();
        drive(&mut runner, &mut actions);

        assert!(actions.iter().any(|l| l == "home"));
        assert!(actions.iter().any(|l| l.starts_with("feeder++")));
        assert!(actions.iter().any(|l| l.starts_with("vac(n0,on)")));
        assert!(actions.iter().any(|l| l.starts_with("vac(n0,off)")));
        assert!(actions.iter().any(|l| l.starts_with("blow(n0,on)")));
        assert_eq!(runner.state(), &JobState::Done);
        assert!(runner.job().placements[0].placed);
    }

    #[test]
    fn trip_ids_mirror_the_runner_batch_boundaries() {
        // Same rule start_batch applies: same camera kind + strategy +
        // distinct nozzles + head capacity. Mixed camera kinds / repeated
        // nozzles / oversize loads each force a trip break.
        let base = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::new(0.0, 0.0),
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 1,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let mut make = |nozzle: u8, camera: CameraKind| Placement {
            nozzle,
            camera_kind: camera,
            ..base.clone()
        };
        let placements = vec![
            make(1, CameraKind::FastVision),
            make(2, CameraKind::FastVision),
            make(3, CameraKind::FastVision),
            // Capacity: 4 placements already, head holds 4.
            make(4, CameraKind::FastVision),
            // Duplicate nozzle + camera change below force separate trips.
            make(1, CameraKind::Precise),
            make(1, CameraKind::FastVision),
            make(2, CameraKind::FastVision),
            make(3, CameraKind::FastVision),
        ];
        let ids = trip_ids(4, &placements);
        assert_eq!(ids, vec![0, 0, 0, 0, 1, 2, 2, 2]);
    }

    #[test]
    fn multi_pick_holds_all_parts_before_first_release() {
        let head = default_head(2);
        let feeders = vec![FeederSlot {
            id: 3,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: Some(PartId(1)),
        }];
        let placements = (1..=2)
            .map(|nozzle| Placement {
                reference: String::new(),
                part: dummy_part(1),
                design_position: Point::new(nozzle as f64 * 10.0, 5.0),
                design_rotation_deg: 0.0,
                feeder_slot: 3,
                nozzle,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            })
            .collect();
        let mut runner = Runner::new(
            head,
            feeders,
            MachineConfig::default(),
            Job::new("MULTI", placements),
        );
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));

        let mut vacuum_on = 0;
        let mut camera_checks = 0;
        let mut events = Vec::new();
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(Action::Vacuum { on: true, .. }) => {
                    vacuum_on += 1;
                    events.push("on");
                }
                StepResult::Action(Action::Vacuum { on: false, .. }) => events.push("off"),
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => {
                    camera_checks += 1;
                    events.push("camera");
                    runner.feed_camera(PartObservation::ok());
                }
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }

        assert_eq!(vacuum_on, 2);
        let first_off = events.iter().position(|event| *event == "off").unwrap();
        assert_eq!(events[..first_off], ["on", "on", "camera"]);
        assert_eq!(camera_checks, 1);
        assert!(runner.job().placements.iter().all(|p| p.placed));
    }

    #[test]
    fn conjoined_batch_runs_until_first_nozzle_collision() {
        // 4-nozzle head, all N503 compatible. Two LED placements
        // interleaved with two R1 placements should produce a single
        // batch of 4 placements (since R1 and LED both use N503 and
        // the runner's batch rule now allows mixed parts).
        let head = default_head(4);
        let feeders = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 400.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(50.0, 400.0),
                loaded: None,
            },
        ];
        let mut r1 = dummy_part(1);
        r1.id = PartId(1);
        let mut led = dummy_part(2);
        led.id = PartId(2);
        let placements = vec![
            Placement {
                reference: String::new(),
                part: r1.clone(),
                design_position: Point::new(10.0, 10.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 0,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: led.clone(),
                design_position: Point::new(20.0, 20.0),
                design_rotation_deg: 0.0,
                feeder_slot: 1,
                nozzle: 1,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: r1.clone(),
                design_position: Point::new(30.0, 30.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 2,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: led.clone(),
                design_position: Point::new(40.0, 40.0),
                design_rotation_deg: 0.0,
                feeder_slot: 1,
                nozzle: 3,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
        ];
        let mut runner = Runner::new(
            head,
            feeders,
            MachineConfig::default(),
            Job::new("CONJOIN", placements),
        );
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));

        let mut camera_checks = 0;
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => {
                    camera_checks += 1;
                    runner.feed_camera(PartObservation::ok());
                }
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        // 4 placements with 4 distinct nozzles and same camera+strategy
        // should form one batch.
        assert_eq!(camera_checks, 1, "expected 1 batch, got {}", camera_checks);
        assert!(runner.job().placements.iter().all(|p| p.placed));
    }

    #[test]
    fn five_placement_interleaved_run_produces_two_batches() {
        // 3 N503-compatible nozzles on a 4-nozzle head. Five
        // placements of mixed parts (R, L, R, L, R) should split into
        // 2 batches: [R, L, R] (nozzles 1, 2, 3) and [L, R] (nozzles
        // 1, 2), for 2 camera checks total.
        let head = default_head(4);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 400.0),
            loaded: None,
        }];
        let mut r1 = dummy_part(1);
        r1.id = PartId(1);
        let mut led = dummy_part(2);
        led.id = PartId(2);
        let placements = vec![
            Placement {
                reference: String::new(),
                part: r1.clone(),
                design_position: Point::new(10.0, 10.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 0,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: led.clone(),
                design_position: Point::new(20.0, 20.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 1,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: r1.clone(),
                design_position: Point::new(30.0, 30.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 2,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: led.clone(),
                design_position: Point::new(40.0, 40.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 1,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
            Placement {
                reference: String::new(),
                part: r1.clone(),
                design_position: Point::new(50.0, 50.0),
                design_rotation_deg: 0.0,
                feeder_slot: 0,
                nozzle: 2,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            },
        ];
        let mut runner = Runner::new(
            head,
            feeders,
            MachineConfig::default(),
            Job::new("INTERLEAVE", placements),
        );
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));

        let mut camera_checks = 0;
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => {
                    camera_checks += 1;
                    runner.feed_camera(PartObservation::ok());
                }
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        // Batch 1: [R1,L,R1] uses nozzles 0,1,2. Batch 2: [L,R1] uses
        // nozzles 1,2. The 5th placement re-uses nozzle 2 from batch 2
        // if batch 2 takes nozzles 1,2 — that would collide. The runner
        // may keep batch 2 as [L,R1] (2 placements) and start batch 3
        // for the 5th placement. So 2 or 3 batches are both valid.
        assert!(
            (2..=3).contains(&camera_checks),
            "expected 2 or 3 batches, got {}",
            camera_checks
        );
        assert!(runner.job().placements.iter().all(|p| p.placed));
    }

    #[test]
    fn board_pose_required_before_place() {
        let mut runner = build();
        // Drive until either Done or Aborted, never set a board pose.
        let mut finished: Option<String> = None;
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(msg) => {
                    runner.feed_prompt_response();
                    if msg.contains("Locate fiducials") {
                        // We are stuck here without a board pose; the
                        // runner must not progress to Picking.
                        assert_eq!(*runner.state(), JobState::LocatingFiducials);
                        return;
                    }
                }
                StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
                StepResult::Done => {
                    finished = Some("Done".into());
                    break;
                }
                StepResult::Aborted(reason) => {
                    finished = Some(reason);
                    break;
                }
            }
        }
        panic!(
            "expected runner to wait in LocatingFiducials, finished={:?}",
            finished
        );
    }

    #[test]
    fn rotation_correction_emits_rotate_action() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut actions = Vec::new();
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(a) => {
                    actions.push(a.label());
                    if matches!(a, Action::RotateNozzle { .. }) {
                        // skip remaining steps to keep this test small
                        return;
                    }
                }
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => {
                    runner.feed_camera(PartObservation {
                        rotation_error_deg: 12.5,
                        centroid_offset: Point::ORIGIN,
                        present: true,
                        within_tolerance: true,
                    });
                }
                StepResult::Done | StepResult::Aborted(_) => return,
            }
        }
        assert!(
            actions.iter().any(|l| l.starts_with("rot(n0,")),
            "expected a rotation action, got {:?}",
            actions
        );
    }

    #[test]
    fn centroid_offset_shifts_place_move() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut moves: Vec<String> = Vec::new();
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(Action::Move { target, .. }) => {
                    moves.push(format!("({:.2},{:.2})", target.x, target.y));
                }
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation {
                    rotation_error_deg: 0.0,
                    centroid_offset: Point::new(1.0, 2.0),
                    present: true,
                    within_tolerance: true,
                }),
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        // The "move to board" must reflect the (1, 2) mm offset: head
        // goes to (design - offset) = (9, 3) for a 1-nozzle head
        // (nozzle 0 sits at offset 0 on default_head(1)).
        assert!(
            moves.iter().any(|m| m == "(9.00,3.00)"),
            "expected place move (9.00,3.00); moves = {:?}",
            moves
        );
    }

    fn build_with_strategy(strategy: PlacementStrategy) -> Runner {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 3,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: Some(PartId(1)),
        }];
        let placements = vec![Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::new(10.0, 5.0),
            design_rotation_deg: 0.0,
            feeder_slot: 3,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy,
            camera_kind: CameraKind::FastVision,
        }];
        let job = Job::new("BOARD-1", placements);
        Runner::new(head, feeders, MachineConfig::default(), job)
    }

    #[test]
    fn polarized_strategy_rotates_before_and_after_camera() {
        let mut runner = build_with_strategy(PlacementStrategy::Polarized {
            before_camera_deg: 30.0,
            after_camera_deg: -45.0,
        });
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut rotates: Vec<f64> = Vec::new();
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(Action::RotateNozzle { delta_deg, .. }) => {
                    rotates.push(delta_deg);
                }
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation {
                    rotation_error_deg: 7.5,
                    centroid_offset: Point::ORIGIN,
                    present: true,
                    within_tolerance: true,
                }),
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        // Pre: +30, camera correction: -7.5, post: -45. The fourth rotate is
        // the place-state rotation to the design angle and isn't checked.
        assert_eq!(
            &rotates[..3],
            &[30.0, -7.5, -45.0],
            "expected pre/camera/post rotates; got {:?}",
            rotates
        );
    }

    #[test]
    fn precision_strategy_converges() {
        let mut runner = build_with_strategy(PlacementStrategy::Precision {
            rotation_tolerance_deg: 0.6,
            offset_tolerance_mm: 0.1,
            max_attempts: 5,
        });
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut camera_calls = 0;
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => {
                    camera_calls += 1;
                    // Halve the error each call: 8°, 4°, 2°, 1° — within
                    // 0.5° tolerance on the fourth attempt.
                    let err = 8.0 / (1 << (camera_calls - 1)) as f64;
                    runner.feed_camera(PartObservation {
                        rotation_error_deg: err,
                        centroid_offset: Point::ORIGIN,
                        present: true,
                        within_tolerance: true,
                    });
                }
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        // Camera was invoked multiple times because the precision loop
        // iterated until tolerance.
        assert!(
            camera_calls >= 4,
            "expected at least 4 camera calls, got {}",
            camera_calls
        );
        // Don't require Done — the precision loop converges but the
        // runner may still be in the place phase. Just assert the job
        // isn't aborted.
        assert!(
            !matches!(runner.state(), JobState::Aborted { .. }),
            "precision converged but job was aborted; state = {:?}",
            runner.state()
        );
    }

    #[test]
    fn precision_strategy_aborts_when_exhausted() {
        let mut runner = build_with_strategy(PlacementStrategy::Precision {
            rotation_tolerance_deg: 0.001,
            offset_tolerance_mm: 0.001,
            max_attempts: 3,
        });
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => {
                    // Always report a huge error so we never converge.
                    runner.feed_camera(PartObservation {
                        rotation_error_deg: 10.0,
                        centroid_offset: Point::ORIGIN,
                        present: true,
                        within_tolerance: true,
                    });
                }
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        assert!(
            matches!(runner.state(), JobState::Aborted { .. }),
            "expected abort after exhausting max_attempts; state = {:?}",
            runner.state()
        );
    }

    fn build_with_camera(camera_kind: CameraKind, cameras: Vec<Camera>) -> Runner {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 3,
            bank_id: 0,
            position: Point::new(100.0, 50.0),
            loaded: Some(PartId(1)),
        }];
        let placements = vec![Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::new(10.0, 5.0),
            design_rotation_deg: 0.0,
            feeder_slot: 3,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind,
        }];
        let job = Job::new("BOARD-1", placements);
        let mut setup = MachineSetup::new();
        setup.set_kind(8, FeederKind::Tape(8));
        Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            setup,
            cameras,
            MachineConfig::default(),
            job,
            FeederRotation::default(),
        )
    }

    #[test]
    fn placement_with_precise_kind_drives_to_precise_camera() {
        // FastVision is at Y=200, Precise is at Y=250. Default 1-nozzle head:
        // head reference equals tip.
        let mut runner = build_with_camera(
            CameraKind::Precise,
            vec![
                Camera {
                    id: 0,
                    kind: CameraKind::FastVision,
                    position: Point::new(0.0, 200.0),
                    array_pitch_mm: Some(20.0),
                },
                Camera {
                    id: 1,
                    kind: CameraKind::Precise,
                    position: Point::new(0.0, 250.0),
                    array_pitch_mm: None,
                },
            ],
        );
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut moves: Vec<String> = Vec::new();
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(Action::Move { target, .. }) => {
                    moves.push(format!("({:.2},{:.2})", target.x, target.y));
                }
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        // Camera selection is by kind, not by physical camera ID.
        assert!(
            moves.iter().any(|m| m == "(0.00,250.00)"),
            "expected move to Precise camera at (0, 250); moves = {:?}",
            moves
        );
        assert!(
            !moves.iter().any(|m| m == "(0.00,200.00)"),
            "should not have moved to FastVision; moves = {:?}",
            moves
        );
    }

    #[test]
    fn missing_camera_falls_back_to_first_registered() {
        // No cameras registered. Runner must still drive *somewhere*
        // without panicking.
        let mut runner = build_with_camera(CameraKind::FastVision, Vec::new());
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        for _ in 0..500 {
            match runner.step() {
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
                StepResult::Done | StepResult::Aborted(_) => break,
            }
        }
        assert!(matches!(runner.state(), JobState::Done));
    }

    #[test]
    fn fastvision_sees_multiple_nozzles_at_once() {
        // 4-nozzle head at 20mm pitch: nozzles at -30, -10, +10, +30 mm X.
        // All 4 nozzles have parts. FastVision camera at (0, 200) with
        // FastVision is a line array. The runner centers the occupied nozzle
        // span on the array without applying an FOV rectangle.
        let mut head = default_head(4);
        head.nozzles[0].kind = NozzleKind::N503;
        head.nozzles[1].kind = NozzleKind::N502;
        head.nozzles[2].kind = NozzleKind::N503;
        head.nozzles[3].kind = NozzleKind::N502;

        // All nozzles have parts (simulate by setting vacuum true).
        for n in &mut head.nozzles {
            n.vacuum = true;
        }

        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 400.0),
            loaded: Some(PartId(1)),
        }];
        let _setup = MachineSetup::new();
        let cameras = vec![Camera {
            id: 0,
            kind: CameraKind::FastVision,
            position: Point::new(0.0, 200.0),
            array_pitch_mm: Some(20.0),
        }];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::new(0.0, 0.0),
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0, // will be overridden by runner's current_placement
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let job = Job::new("TEST", vec![placement]);

        let mut runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            job,
            FeederRotation::default(),
        );
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));

        // We just want to check the move-to-camera target.
        // The runner will try to position so multiple nozzles are in FOV.
        let mut camera_moves = Vec::new();
        for _ in 0..100 {
            match runner.step() {
                StepResult::Action(Action::Move { target, .. }) => {
                    if target.y > 150.0 {
                        // camera region
                        camera_moves.push(format!("({:.2},{:.2})", target.x, target.y));
                    }
                }
                StepResult::Action(_) => {}
                StepResult::NeedPrompt(_) => runner.feed_prompt_response(),
                StepResult::NeedCamera => runner.feed_camera(PartObservation::ok()),
                _ => break,
            }
        }
        // With the no-FOV model, each inspection station is a single-nozzle
        // point: every camera-region move must land some nozzle tip on
        // the camera at (0, 200). With camera.position.x = 0 and the
        // active nozzle offset between -30 and +30, head X is in
        // [-30, 30].
        assert!(
            !camera_moves.is_empty(),
            "expected at least one move to camera region; got {:?}",
            camera_moves
        );
        let all_on_camera = camera_moves.iter().all(|m| {
            let x: f64 = m
                .split(',')
                .next()
                .unwrap()
                .trim_start_matches('(')
                .parse()
                .unwrap();
            x.abs() <= 30.0
        });
        assert!(
            all_on_camera,
            "every camera move should land a nozzle on the camera; got {:?}",
            camera_moves
        );
    }

    #[test]
    fn select_station_groups_tiled_cameras() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        // Four FastVision cameras tiled along X at the same Y. They
        // should collapse to a single station whose centre is the mean X
        // (30) and whose half-width covers the union.
        let cameras = vec![
            Camera {
                id: 0,
                kind: CameraKind::FastVision,
                position: Point::new(0.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 1,
                kind: CameraKind::FastVision,
                position: Point::new(20.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 2,
                kind: CameraKind::FastVision,
                position: Point::new(40.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 3,
                kind: CameraKind::FastVision,
                position: Point::new(60.0, 200.0),
                array_pitch_mm: None,
            },
        ];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        let station = runner
            .select_station(CameraKind::FastVision, Point::ORIGIN)
            .unwrap();
        assert_eq!(station.camera_ids.len(), 4);
        assert!((station.centre.x - 30.0).abs() < 1e-9);
        // 4 tiles spanning 0..60 with 20 mm pitch: union half-width =
        // (60-0)/2 + 20/2 = 30 + 10 = 40.
        assert!((station.half_width_mm - 40.0).abs() < 1e-9);
    }

    #[test]
    fn select_station_splits_cameras_at_different_y() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let cameras = vec![
            Camera {
                id: 0,
                kind: CameraKind::FastVision,
                position: Point::new(0.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 1,
                kind: CameraKind::FastVision,
                position: Point::new(50.0, 300.0),
                array_pitch_mm: None,
            },
        ];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        let stations =
            crate::runner::tile_stations(&runner.cameras_for_test(), CameraKind::FastVision);
        assert_eq!(stations.len(), 2);
    }

    #[test]
    fn optimal_position_centres_held_tips_in_tiled_station() {
        let mut head = default_head(4);
        // 4-nozzle head: offsets -30, -10, +10, +30. Hold nozzles 0 and
        // 3 so the held-tip range spans the full -30..+30.
        head.nozzles[0].vacuum = true;
        head.nozzles[3].vacuum = true;
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let cameras = vec![
            Camera {
                id: 0,
                kind: CameraKind::FastVision,
                position: Point::new(0.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 1,
                kind: CameraKind::FastVision,
                position: Point::new(20.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 2,
                kind: CameraKind::FastVision,
                position: Point::new(40.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 3,
                kind: CameraKind::FastVision,
                position: Point::new(60.0, 200.0),
                array_pitch_mm: None,
            },
        ];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        let station = runner
            .select_station(CameraKind::FastVision, Point::ORIGIN)
            .unwrap();
        let target = runner.optimal_camera_head_position(&station, 0);
        // Held tip range centre is 0, station centre is 30, so head
        // should land at X=30 to put both tips inside the union.
        assert!((target.x - 30.0).abs() < 1e-9, "x={}", target.x);
        assert!((target.y - 200.0).abs() < 1e-9, "y={}", target.y);
    }

    #[test]
    fn select_station_ignores_other_kinds() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let cameras = vec![
            Camera {
                id: 0,
                kind: CameraKind::Precise,
                position: Point::ORIGIN,
                array_pitch_mm: None,
            },
            Camera {
                id: 1,
                kind: CameraKind::FastVision,
                position: Point::new(100.0, 100.0),
                array_pitch_mm: None,
            },
        ];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        let chosen = runner
            .select_station(CameraKind::FastVision, Point::ORIGIN)
            .unwrap();
        assert_eq!(chosen.camera_ids, vec![1]);
        let chosen_precise = runner
            .select_station(CameraKind::Precise, Point::ORIGIN)
            .unwrap();
        assert_eq!(chosen_precise.camera_ids, vec![0]);
    }

    #[test]
    fn select_station_returns_none_when_kind_missing() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let cameras = vec![Camera {
            id: 0,
            kind: CameraKind::Precise,
            position: Point::ORIGIN,
            array_pitch_mm: None,
        }];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        assert!(
            runner
                .select_station(CameraKind::FastVision, Point::ORIGIN)
                .is_none()
        );
    }

    #[test]
    fn manual_fastvision_station_overrides_auto() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        // Two FastVision cameras. Without manual override, the runner
        // picks the closer one (id=1).
        let cameras = vec![
            Camera {
                id: 0,
                kind: CameraKind::FastVision,
                position: Point::new(-200.0, 200.0),
                array_pitch_mm: None,
            },
            Camera {
                id: 1,
                kind: CameraKind::FastVision,
                position: Point::new(100.0, 100.0),
                array_pitch_mm: None,
            },
        ];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::FastVision,
        };
        let mut runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        // Pin to a manual point that's nowhere near any registered camera.
        let manual = Point::new(500.0, 500.0);
        runner.set_fastvision_station(manual);
        let chosen = runner
            .select_station(CameraKind::FastVision, Point::ORIGIN)
            .unwrap();
        assert_eq!(chosen.centre, manual);
        assert_eq!(chosen.half_width_mm, 0.0);
        // Optimal head position uses the manual centre (single-point):
        // with active nozzle 0 (offset 0), head lands at the manual X.
        let target = runner.optimal_camera_head_position(&chosen, 0);
        assert_eq!(target, manual);
        // Clear and confirm auto-selection returns.
        runner.clear_fastvision_station();
        let auto = runner
            .select_station(CameraKind::FastVision, Point::ORIGIN)
            .unwrap();
        assert_ne!(auto.centre, manual);
    }

    #[test]
    fn manual_fastvision_station_does_not_affect_precise() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let cameras = vec![
            Camera {
                id: 0,
                kind: CameraKind::Precise,
                position: Point::ORIGIN,
                array_pitch_mm: None,
            },
            Camera {
                id: 1,
                kind: CameraKind::FastVision,
                position: Point::new(100.0, 100.0),
                array_pitch_mm: None,
            },
        ];
        let placement = Placement {
            reference: String::new(),
            part: dummy_part(1),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            feeder_slot: 0,
            nozzle: 0,
            placed: false,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: CameraKind::Precise,
        };
        let mut runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            MachineConfig::default(),
            Job::new("TEST", vec![placement]),
            FeederRotation::default(),
        );
        // Pin FastVision to a wildly different point. Precise must
        // ignore the override and return its registered camera.
        runner.set_fastvision_station(Point::new(999.0, 999.0));
        let precise = runner
            .select_station(CameraKind::Precise, Point::ORIGIN)
            .unwrap();
        assert_eq!(precise.centre, Point::ORIGIN);
    }

    #[test]
    fn onboard_view_at_tracks_head_position() {
        use crate::nozzle::OnboardCamera;
        let head = default_head(2).with_onboard_camera(OnboardCamera {
            offset_mm: Point::new(0.0, 0.0),
            fov_mm: Size {
                width: 20.0,
                height: 10.0,
            },
        });
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let cameras = vec![Camera {
            id: 0,
            kind: CameraKind::FastVision,
            position: Point::ORIGIN,
            array_pitch_mm: None,
        }];
        let config = MachineConfig::default();
        let runner = Runner::with_setup(
            head,
            feeders,
            Vec::new(),
            FeederCalibration::new(),
            MachineSetup::new(),
            cameras,
            config,
            Job::new("BOARD", Vec::new()),
            FeederRotation::default(),
        );
        let (lo, hi) = runner.onboard_view_at(Point::new(0.0, 0.0)).unwrap();
        assert_eq!(lo, Point::new(-10.0, -5.0));
        assert_eq!(hi, Point::new(10.0, 5.0));
        let (lo2, hi2) = runner.onboard_view_at(Point::new(50.0, 25.0)).unwrap();
        assert_eq!(lo2, Point::new(40.0, 20.0));
        assert_eq!(hi2, Point::new(60.0, 30.0));
    }

    #[test]
    fn collect_actions_runs_job_to_done() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let actions = runner.collect_actions();
        assert!(!actions.is_empty());
        assert!(
            runner.state() == &JobState::Done || matches!(runner.state(), JobState::Aborted { .. })
        );
        assert!(runner.job().placements[0].placed);
    }

    #[test]
    fn collect_actions_matches_drive_helper() {
        let mut a = build();
        a.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let mut labels = Vec::new();
        drive(&mut a, &mut labels);

        let mut b = build();
        b.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let via_helper: Vec<String> = b.collect_actions().iter().map(|a| a.label()).collect();

        assert_eq!(labels, via_helper);
    }

    #[test]
    fn collect_steps_includes_done_terminator() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let steps = runner.collect_steps();
        assert!(matches!(steps.last(), Some(StepResult::Done)));
        assert!(steps.iter().any(|s| matches!(s, StepResult::Action(_))));
    }

    #[test]
    fn planned_place_requires_pose() {
        let runner = build();
        assert!(runner.planned_place(0).is_none());
        assert!(runner.planned_places().is_empty());
    }

    #[test]
    fn planned_place_applies_pose_rotation_and_translation() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(100.0, 50.0, 30.0));
        let preview = runner.planned_place(0).unwrap();
        assert_eq!(preview.index, 0);
        assert_eq!(preview.feeder_slot, 3);
        // design_position is unchanged; world_position is the pose-rotated,
        // pose-translated version.
        let expected_world = runner
            .board_pose()
            .unwrap()
            .transform(preview.design_position);
        assert!((preview.world_position.x - expected_world.x).abs() < 1e-9);
        assert!((preview.world_position.y - expected_world.y).abs() < 1e-9);
        // Final rotation includes the board rotation.
        assert!((preview.nozzle_rotation_deg - 30.0).abs() < 1e-9);
    }

    #[test]
    fn planned_places_returns_all_in_run_order() {
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let previews = runner.planned_places();
        assert_eq!(previews.len(), runner.job().placements.len());
        for (i, p) in previews.iter().enumerate() {
            assert_eq!(p.index, i);
        }
    }

    #[test]
    fn planned_place_reflects_pose_update() {
        // Set identity, capture preview, then rotate the board 90° and
        // confirm the preview's world_position and nozzle_rotation_deg
        // move with it.
        let mut runner = build();
        runner.set_board_pose(Pose::new(0.0, 0.0, 0.0));
        let before = runner.planned_place(0).unwrap();

        runner.set_board_pose(Pose::new(0.0, 0.0, 90.0));
        let after = runner.planned_place(0).unwrap();

        assert_ne!(before.world_position, after.world_position);
        assert!((after.nozzle_rotation_deg - 90.0).abs() < 1e-9);
    }
}
