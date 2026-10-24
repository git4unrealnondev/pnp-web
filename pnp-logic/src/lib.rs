//! `pnp-logic`: high-level pick-and-place machine logic.
//!
//! Built on top of `hwcontrol`, which provides the low-level serial
//! protocol. This crate adds:
//!
//! - Geometric primitives for machine coordinates.
//! - Nozzle head modelling (position math, part-on-nozzle rotation).
//! - Feeder slot modelling and assignment optimisation.
//! - Part / footprint / polarity definitions.
//! - Board identification (text, QR codes) and fiducial-based pose recovery.
//! - Pick, place, and job-level state machines.
//! - A machine-agnostic `Action` stream + a `Runner` that drives the
//!   state machines for any caller to dispatch.

pub mod action;
pub mod config;
pub mod feeder;
pub mod geometry;
pub mod identifier;
pub mod job_io;
pub mod job_executor;
pub mod nozzle;
pub mod nozzle_calibration;
pub mod optimizer;
pub mod part_setup;
pub mod parts;
pub mod placement;
pub mod plan;
pub mod runner;
pub mod state;

use std::error::Error;
use std::sync::{Arc, Mutex};
use std::time::Duration;

/// Everything the operator must supply to run a nozzle calibration.
///
/// Kept separate from the sequence so the geometry is explicit and
/// testable, and so the UI can present exactly these fields.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NozzleCalibrationPlan {
    /// Nozzles to calibrate. This machine addresses four rotation axes,
    /// so more than four cannot be calibrated.
    pub nozzles: Vec<NozzleId>,
    /// Nominal nozzle offsets in mm, used to predict where each mark
    /// should appear. Usually `default_head(4)` offsets.
    pub nozzle_offsets_mm: Vec<f64>,
    /// Where the head parks for each round. Must put every nozzle over
    /// the paper.
    pub head_position: Point,
    /// Z height at which the tips touch the paper, in mm above deck.
    /// There is no Z probing on this machine, so this is whatever height
    /// the paper happens to sit at — a wrong value either misses the paper
    /// or misses the ink entirely.
    pub touch_z_mm: f64,
    /// Z height to retract to between rounds, in mm. Must clear the paper
    /// and any nozzle already lifted by rotation.
    pub clearance_z_mm: f64,
    pub camera_type: CameraTypes,
    pub min_mark_diameter_mm: f64,
    pub max_mark_diameter_mm: f64,
    pub speed: u8,
    /// Settle time after each move, before capture.
    #[serde(with = "duration_secs")]
    pub settle: Duration,
}

mod duration_secs {
    use super::Duration;
    use serde::{Deserialize, Deserializer, Serializer};

    pub fn serialize<S: Serializer>(value: &Duration, serializer: S) -> Result<S::Ok, S::Error> {
        serializer.serialize_f64(value.as_secs_f64())
    }

    pub fn deserialize<'de, D: Deserializer<'de>>(deserializer: D) -> Result<Duration, D::Error> {
        f64::deserialize(deserializer).map(Duration::from_secs_f64)
    }
}

impl NozzleCalibrationPlan {
    /// The four cardinal rotations, in the order performed.
    ///
    /// Cumulative: the hardware tracks absolute angle and
    /// `rotate_nozzle` sends a delta, so the sequence advances 90° at a
    /// time rather than commanding absolute angles. Asking for absolute
    /// angles would leave each run starting wherever the last one ended.
    pub const ROTATIONS: [f64; 4] = CARDINAL_ROTATIONS;

    /// A plan for the common case: four nozzles at 20 mm pitch, on
    /// paper at `touch_z_mm`, viewed by the head camera.
    ///
    /// The remaining fields still need the operator's attention — most
    /// importantly `touch_z_mm`, which cannot be discovered by the
    /// machine because there is no Z probe.
    pub fn new(head_position: Point, touch_z_mm: f64) -> Self {
        let nozzles = default_calibration_nozzles();
        Self {
            nozzle_offsets_mm: vec![-30.0, -10.0, 10.0, 30.0],
            nozzles: nozzles.to_vec(),
            head_position,
            touch_z_mm,
            clearance_z_mm: touch_z_mm + 10.0,
            camera_type: CameraTypes::Head,
            min_mark_diameter_mm: 0.3,
            max_mark_diameter_mm: 3.0,
            speed: 1,
            settle: Duration::from_millis(1000),
        }
    }
}

pub use action::{Action, LightColor, MoveKind};
pub use config::{Camera, CameraKind, MachineSetup, default_kind_for};
pub use feeder::ProviderClass;
pub use feeder::{FeederBank, FeederCalibration, FeederKind, FeederSlot, PartId, PartPickup};
pub use geometry::{Pose, Size};
pub use identifier::{BoardId, Fiducial, FiducialKind, FiducialObservation, compute_board_pose};
pub use job_io::{BoardMapping, CalibrationEntry, HeadSpec, JobFile, MachineContext, NozzleSpec};
pub use nozzle::{NozzleHead, NozzleKind, NozzleSlot, assign_nozzles, default_head};
pub use job_executor::{JobExecutor, JobProgress};
pub use nozzle_calibration::{
    CARDINAL_ROTATIONS, InkMark, NozzleCalibration, NozzleOffset, assign_round_marks,
    default_calibration_nozzles, offsets_from_calibration,
};
pub use optimizer::{
    ComponentDemand, FeederAssignment, FeederInstallation, FeederOptimization, FeederPlacementMode,
    OptimizerSlot, PlacementNeed, chebyshev_cost, optimise_feeder_model, optimise_feeder_slots,
    optimise_feeder_slots_with_mode,
};
pub use parts::{Footprint, Pad, PadKind, Part, Polarity};
pub use part_setup::{
    PartForm, PartParams, PartStatus, SetupPart, derive_part_params, part_from_footprint,
    PartSummary,
};
pub use placement::{
    PartObservation, feeder_pickup_target, feeder_pickup_z, feeder_target, pickup_for,
    place_target, required_nozzle_rotation, shortest_rotation,
};
pub use plan::{
    PlanSearchMode, PlannedJob, plan_job, plan_job_with_feeder_mode, plan_job_with_mode,
};
pub use runner::{
    MachineConfig, Runner, StepResult, optimal_batch_pick_route, trip_ids,
};
pub use shared_types::*;
pub use state::{
    Job, PickFailure, PickState, PlaceFailure, PlaceState, Placement, PlacementStrategy,
};
pub use vision_functions::*;

pub use hwcontrol::JogRequest;
use hwcontrol::MoveType;

const MAX_OFFSET_FROM_FEDUCIAL_MM: f64 = 0.3;

#[derive(Debug)]
pub enum GeneralMachineError {
    NotConnected,
    NotReady,
    UnsupportedLightColor(LightColor),
    Machine(hwcontrol::machine::Error),
}

impl From<hwcontrol::machine::Error> for GeneralMachineError {
    fn from(error: hwcontrol::machine::Error) -> Self {
        Self::Machine(error)
    }
}

impl std::fmt::Display for GeneralMachineError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::NotConnected => write!(f, "machine is not connected"),
            Self::NotReady => write!(f, "machine is not ready for movement"),
            Self::UnsupportedLightColor(color) => {
                write!(f, "hardware does not support {:?} light", color)
            }
            Self::Machine(error) => error.fmt(f),
        }
    }
}

impl std::error::Error for GeneralMachineError {}

// Generic machine to hold generics
pub struct GeneralMachine {
    machine_state: Mutex<MachineState>,
    machine_connection: Arc<Mutex<Option<Box<dyn hwcontrol::Machine>>>>,
    camera_manager: CameraManager,
    /// Nozzle offsets from the most recent calibration. Populated at
    /// startup from the database and replaced whenever a new calibration
    /// is stored, so callers can read the current offsets without
    /// reopening the database.
    nozzle_offsets: Mutex<Vec<NozzleOffset>>,
}

impl GeneralMachine {
    pub fn new() -> Self {
        GeneralMachine {
            machine_state: Mutex::new(MachineState::Unknown),
            machine_connection: Arc::new(Mutex::new(None)),
            camera_manager: CameraManager::new(),
            nozzle_offsets: Mutex::new(Vec::new()),
        }
    }

    /// Current calibrated nozzle offsets, or empty if never calibrated.
    pub fn nozzle_offsets(&self) -> Vec<NozzleOffset> {
        self.nozzle_offsets.lock().map(|o| o.clone()).unwrap_or_default()
    }

    /// Record a calibration in memory. The caller is responsible for
    /// persisting it; this only updates the in-memory copy so subsequent
    /// readers see the new values.
    pub fn store_nozzle_calibration(&self, offsets: &[NozzleOffset]) {
        if let Ok(mut guard) = self.nozzle_offsets.lock() {
            *guard = offsets.to_vec();
        }
    }

    /// Load persisted offsets into memory. Returns whether a calibration
    /// was found, so startup can report it.
    pub fn load_nozzle_calibration(&self, offsets: &[NozzleOffset]) -> bool {
        if offsets.is_empty() {
            return false;
        }
        self.store_nozzle_calibration(offsets);
        true
    }

    /// Returns true or false if camera was able to be centered on feducial
    /// Physically puts the camera on the feducial
    pub fn center_camera_on_feducial(&self, tries: u8) -> bool {
        self.center_camera_on_landmark(tries, 15.0, 80.0)
    }

    /// Centers the head camera on a large circular feeder-pocket landmark.
    pub fn center_camera_on_feeder_pocket(&self, camera_type: &CameraTypes, tries: u8) -> bool {
        for _ in 0..tries {
            // Camera frames are updated asynchronously. Give the capture
            // thread time to publish a fresh frame after startup or a move.
            std::thread::sleep(Duration::from_millis(1000));
            let Some(camera_offset) = self
                .camera_manager
                .feeder_pocket_correction_in_world_space(camera_type)
            else {
                continue;
            };
            if let Ok(mut guard) = self.machine_connection.lock()
                && let Some(machine) = guard.as_deref_mut()
            {
                let position = machine.cached_position();
                let target_x = position.x + camera_offset.x;
                let target_y = position.y + camera_offset.y;
                if machine
                    .move_xy((target_x, target_y), 1, MoveType::Normal)
                    .is_err()
                {
                    return false;
                }
                if camera_offset.x.abs() < MAX_OFFSET_FROM_FEDUCIAL_MM
                    && camera_offset.y.abs() < MAX_OFFSET_FROM_FEDUCIAL_MM
                {
                    return true;
                }
            }
        }
        false
    }

    fn center_camera_on_landmark(&self, tries: u8, min_diameter: f32, max_diameter: f32) -> bool {
        for _ in 0..tries {
            // Let the head camera settle after the previous XY move.
            std::thread::sleep(Duration::from_millis(1000));
            if let Some(camera_offset) = self.camera_manager.landmark_correction_in_world_space(
                &CameraTypes::Head,
                min_diameter,
                max_diameter,
            ) && let Ok(mut guard) = self.machine_connection.lock()
                && let Some(machine) = guard.as_deref_mut()
            {
                let position = machine.cached_position();
                let target_x = position.x + camera_offset.x;
                let target_y = position.y + camera_offset.y;
                if machine
                    .move_xy((target_x, target_y), 1, MoveType::Normal)
                    .is_err()
                {
                    return false;
                }

                // Check if the offset is now within acceptable tolerance (< instead of >=)
                if camera_offset.x.abs() < MAX_OFFSET_FROM_FEDUCIAL_MM
                    && camera_offset.y.abs() < MAX_OFFSET_FROM_FEDUCIAL_MM
                {
                    return true;
                }
            }
        }
        false
    }

    // Adds camera to machine associates the camera path with the camera type
    pub fn add_camera(
        &mut self,
        camera_path: &str,
        camera_type: shared_types::CameraTypes,
        camera_calibration: Option<PixelToMachineCalibration>,
    ) -> Result<(), Box<dyn Error>> {
        self.camera_manager
            .add_camera(camera_path, camera_type, camera_calibration)
            .map_err(Into::into)
    }

    /// Gets camera image from a camera type
    pub fn camera_png_bytes(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> Result<Vec<u8>, Box<dyn Error>> {
        Ok(self.camera_manager.get_png_bytes_kind(camera_type)?)
    }

    /// Gets camera image from a camera type
    pub fn camera_jpeg_bytes(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> Result<Vec<u8>, Box<dyn Error>> {
        Ok(self.camera_manager.get_jpeg_bytes_kind(camera_type)?)
    }

    pub fn camera_calibration(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> Option<PixelToMachineCalibration> {
        self.camera_manager.calibration(camera_type)
    }

    pub fn set_camera_calibration(
        &self,
        camera_type: &shared_types::CameraTypes,
        calibration: PixelToMachineCalibration,
    ) -> Result<(), Box<dyn Error>> {
        self.camera_manager
            .set_calibration_value(camera_type, calibration)
            .map_err(Into::into)
    }

    pub fn feeder_pocket_settings(
        &self,
        camera_type: &shared_types::CameraTypes,
    ) -> FeederPocketVisionSettings {
        self.camera_manager.feeder_pocket_settings(camera_type)
    }

    pub fn set_feeder_pocket_settings(
        &self,
        camera_type: &shared_types::CameraTypes,
        settings: FeederPocketVisionSettings,
    ) -> Result<(), Box<dyn Error>> {
        self.camera_manager
            .set_feeder_pocket_settings(camera_type, settings)
            .map_err(Into::into)
    }

    /// Calibrates a camera from known machine positions where a landmark is
    /// visible. The four points should describe the board's 3 x 4 cm grid.
    pub fn calibrate_camera_grid(
        &self,
        camera_type: &shared_types::CameraTypes,
        machine_points: &[Point],
    ) -> Result<PixelToMachineCalibration, Box<dyn Error>> {
        if machine_points.len() != 4 {
            return Err("exactly four calibration points are required".into());
        }
        self.calibrate_camera_points(camera_type, machine_points)
    }

    pub fn calibrate_camera_points(
        &self,
        camera_type: &shared_types::CameraTypes,
        machine_points: &[Point],
    ) -> Result<PixelToMachineCalibration, Box<dyn Error>> {
        if machine_points.len() < 3 {
            return Err("at least three calibration points are required".into());
        }
        let mut samples = Vec::with_capacity(machine_points.len());
        for machine_point in machine_points {
            // move_xy waits for the controller's binary completion return.
            self.move_xy(*machine_point, 1)?;
            let actual_position = self
                .cached_position()
                .ok_or(GeneralMachineError::NotConnected)?;
            std::thread::sleep(std::time::Duration::from_millis(1000));
            let frame = self.camera_manager.camera_frame_kind(camera_type)?;
            let detection = detect_fiducial_in_frame(&frame, 8.0, 180.0)?.ok_or_else(|| {
                format!(
                    "no landmark detected at ({}, {})",
                    machine_point.x, machine_point.y
                )
            })?;
            samples.push(CalibrationSample {
                machine_x_mm: actual_position.x,
                machine_y_mm: actual_position.y,
                pixel_x: detection.x as f64,
                pixel_y: detection.y as f64,
            });
        }
        let calibration = calibrate_pixel_to_machine(&samples)?;
        if calibration.rms_error_mm > 1.0 {
            return Err(format!(
                "camera calibration RMS error {:.3} mm exceeds 1.0 mm",
                calibration.rms_error_mm
            )
            .into());
        }
        Ok(self.camera_manager.set_calibration(camera_type, &samples)?)
    }

    /// Runs the full ink-mark nozzle calibration sequence.
    ///
    /// The operator inks all nozzle tips, lays paper on the deck, and
    /// positions the head so every tip is over the paper. This method then
    /// performs, once per cardinal rotation:
    ///
    /// 1. move the head to the calibration point,
    /// 2. lower every nozzle onto the paper to leave a mark,
    /// 3. lift back to the clearance height,
    /// 4. rotate every nozzle 90 degrees further,
    /// 5. detect the fresh marks and record them in machine millimetres.
    ///
    /// Rotation is cumulative: the hardware tracks each nozzle's angle and
    /// `rotate_nozzle` sends a delta, so we ask for +90 four times rather
    /// than setting absolute angles. Setting absolute angles would make a
    /// second run start from wherever the last one stopped.
    ///
    /// Returns one [`NozzleCalibration`] per nozzle.
    pub fn calibrate_nozzles(
        &self,
        plan: &NozzleCalibrationPlan,
    ) -> Result<Vec<NozzleCalibration>, Box<dyn std::error::Error>> {
        for &nozzle in &plan.nozzles {
            if nozzle_calibration::nozzle_index(nozzle).is_none() {
                return Err(format!("nozzle {nozzle:?} is not addressable on this machine").into());
            }
        }

        let mut marks: Vec<InkMark> = Vec::new();
        for &rotation_deg in &NozzleCalibrationPlan::ROTATIONS {
            self.move_xy(plan.head_position, plan.speed)?;

            // Clear the paper before descending so the tips never drag
            // across an existing mark.
            for &nozzle in &plan.nozzles {
                self.move_z(nozzle, plan.clearance_z_mm, plan.speed)?;
            }
            std::thread::sleep(plan.settle);

            for &nozzle in &plan.nozzles {
                self.rotate_nozzle(nozzle, rotation_deg, true, plan.speed)?;
            }
            std::thread::sleep(plan.settle);

            // Touch down: every nozzle descends to the same paper height,
            // so all four marks land in one frame.
            for &nozzle in &plan.nozzles {
                self.move_z(nozzle, plan.touch_z_mm, plan.speed)?;
            }
            std::thread::sleep(plan.settle);

            let frame = self.camera_manager.camera_frame_kind(&plan.camera_type)?;
            let expected = nozzle_calibration::expected_tip_positions(
                plan.head_position,
                &plan.nozzles,
                &plan.nozzle_offsets_mm,
            )?;
            let detected = self.detect_ink_marks(&frame, &expected, plan)?;

            for (index, blob) in detected.iter().enumerate() {
                marks.push(InkMark {
                    nozzle: plan.nozzles[index],
                    rotation_deg,
                    position: *blob,
                });
            }

            for &nozzle in &plan.nozzles {
                self.move_z(nozzle, plan.clearance_z_mm, plan.speed)?;
            }
        }

        Ok(nozzle_calibration::calibrate_all(&marks, &plan.nozzles)?)
    }

    /// Detect the ink marks in one frame and convert them to machine mm.
    ///
    /// Camera calibration is required; without it a pixel position cannot
    /// become a millimetre position, so we fail with an actionable message
    /// rather than emitting a silently meaningless centroid.
    fn detect_ink_marks(
        &self,
        frame: &Mat,
        expected: &[Point],
        plan: &NozzleCalibrationPlan,
    ) -> Result<Vec<Point>, Box<dyn std::error::Error>> {
        let calibration = self
            .camera_manager
            .calibration(&plan.camera_type)
            .ok_or("camera is not calibrated; calibrate it before calibrating nozzles")?;

        // Convert expectations into pixels so detection can be seeded
        // near each nozzle, which keeps the per-nozzle search local and
        // stops one nozzle's mark from being claimed by another.
        let center = PixelToMachineCalibration::image_center(frame);
        let px_per_mm = calibration.pixels_per_mm_x();
        let mut detected = Vec::with_capacity(expected.len());
        for point in expected {
            let expected_pixel = calibration
                .machine_to_pixel(point.x, point.y, center)
                .ok_or("camera calibration is degenerate and cannot be inverted")?;
            let blob = detect_ink_mark_in_frame(
                frame,
                expected_pixel,
                (plan.min_mark_diameter_mm * px_per_mm) as f32,
                (plan.max_mark_diameter_mm * px_per_mm) as f32,
            )?;
            let blob = blob.ok_or_else(|| {
                format!("no ink mark found near ({:.1}, {:.1}) mm", point.x, point.y)
            })?;
            let (mx, my) = calibration.pixel_to_machine(blob.x as f64, blob.y as f64);
            detected.push(Point::new(mx, my));
        }

        // Sanity-check against predicted positions. A mark far from where
        // its nozzle should be means either a bad camera calibration or a
        // nozzle outside the modelled offset — both worth stopping for.
        assign_round_marks(&detected, expected)
            .map_err(|error| -> Box<dyn std::error::Error> { error.into() })
    }

    /// Replace a head's nozzle offsets with a stored calibration.
    ///
    /// Offsets are the thing the runner actually uses
    /// (`NozzleSlot::offset_mm`), so this is what makes a calibration take
    /// effect on real placements. Offsets for nozzles absent from
    /// `offsets` are left untouched, so a partial calibration improves what
    /// it covers without blanking the rest.
    pub fn apply_nozzle_offsets(head: &mut NozzleHead, offsets: &[NozzleOffset]) {
        for offset in offsets {
            if let Some(slot) = head.nozzle_mut(offset.nozzle.id()) {
                slot.offset_mm = offset.offset_mm;
            }
        }
    }

    /// Offsets implied by a default head, for comparison against a
    /// calibration.
    pub fn nominal_offsets_mm(nozzles: &[NozzleId]) -> Vec<f64> {
        nozzles
            .iter()
            .map(|nozzle| {
                let index = nozzle_calibration::nozzle_index(*nozzle).unwrap_or(0);
                default_head(4)
                    .nozzles
                    .get(index)
                    .map(|slot| slot.offset_mm)
                    .unwrap_or(0.0)
            })
            .collect()
    }

    /// Connects to the kayo machine
    pub fn connect_kayo(
        &self,
        path: &str,
        baud_rate: u32,
    ) -> Result<(), Box<dyn std::error::Error>> {
        let machine = hwcontrol::connect_serial_kayo(path, baud_rate)?;
        *self
            .machine_connection
            .lock()
            .map_err(|_| "machine connection lock poisoned")? = Some(machine);
        *self.machine_state.lock().unwrap() = MachineState::Unknown;
        Ok(())
    }

    pub fn state(&self) -> MachineState {
        self.machine_state.lock().unwrap().clone()
    }

    pub fn move_xy(&self, position: Point, speed: u8) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.move_xy((position.x, position.y), speed, MoveType::Normal))
    }

    pub fn jog_xy(&self, request: JogRequest) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.jog_xy(request))
    }

    pub fn move_z(
        &self,
        nozzle: NozzleId,
        height: f64,
        speed: u8,
    ) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.move_z(nozzle, height, speed))
    }

    pub fn rotate_nozzle(
        &self,
        nozzle: NozzleId,
        angle_deg: f64,
        clockwise: bool,
        speed: u8,
    ) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.rotate_nozzle(nozzle, angle_deg, clockwise, speed))
    }

    pub fn rotate_nozzles(
        &self,
        directions: &[bool],
        speed: u8,
    ) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.rotate_nozzles(directions, speed))
    }

    /// Homes machine
    pub fn home_all(&self) -> Result<(), GeneralMachineError> {
        let result = self.call(|machine| machine.home_all());
        if result.is_ok() {
            *self.machine_state.lock().unwrap() = MachineState::Ready;
        }
        result
    }

    pub fn home_z(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.home_z())
    }

    /// Resets the machine
    pub fn reset(&self) -> Result<(), GeneralMachineError> {
        let result = self.call(|machine| machine.reset());
        if result.is_ok() {
            *self.machine_state.lock().unwrap() = MachineState::Unknown;
        }
        result
    }

    /// Returns the cached location of the machine
    pub fn report_position(&self) -> Result<hwcontrol::Position, GeneralMachineError> {
        self.call(|machine| machine.report_position())
    }

    /// Returns the last position received or commanded without querying serial.
    pub fn cached_position(&self) -> Option<hwcontrol::Position> {
        self.machine_connection
            .lock()
            .ok()?
            .as_deref()
            .map(|machine| machine.cached_position())
    }

    pub fn open_vacuum(&self, nozzle: NozzleId) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.open_vacuum(nozzle))
    }

    pub fn close_vacuum(&self, nozzle: NozzleId) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.close_vacuum(nozzle))
    }

    pub fn open_blow(&self, nozzle: NozzleId) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.open_blow(nozzle))
    }

    pub fn close_blow(&self, nozzle: NozzleId) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.close_blow(nozzle))
    }

    pub fn close_vacuum_open_blow(&self, nozzle: NozzleId) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.close_vacuum_open_blow(nozzle))
    }

    pub fn close_vacuum_blow_all(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.close_vacuum_blow_all())
    }

    pub fn open_supplement_valve(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.open_supplement_valve())
    }

    pub fn close_supplement_valve(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.close_supplement_valve())
    }

    pub fn open_feeder(&self, id: u8) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.open_feeder(id))
    }

    pub fn close_feeders(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.close_feeders())
    }

    pub fn tube_feeder_action(&self, p1: u8, p2: u8) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.tube_feeder_action(p1, p2))
    }

    // Mounts PCB to rail
    pub fn clip_pcb(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.clip_pcb())
    }

    pub fn release_pcb(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.release_pcb())
    }

    pub fn light_on(&self, id: u8, brightness: u8) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.light_on(id, brightness))
    }

    pub fn light_off(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.light_off())
    }

    pub fn error_light(&self, color: LightColor) -> Result<(), GeneralMachineError> {
        let color = match color {
            LightColor::Red => shared_types::LightColor::Red,
            LightColor::Yellow => shared_types::LightColor::Yellow,
            LightColor::Off => shared_types::LightColor::Off,
            LightColor::Green => shared_types::LightColor::Green,
        };
        self.call(|machine| machine.error_light(color))
    }

    pub fn conveyor_run(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.conveyor_run())
    }

    pub fn conveyor_stop(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.conveyor_stop())
    }

    pub fn conveyor_stop_pin_engage(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.conveyor_stop_pin_engage())
    }

    pub fn conveyor_clip_pcb(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.conveyor_clip_pcb())
    }

    pub fn conveyor_unclip_and_move(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.conveyor_unclip_and_move())
    }

    pub fn conveyor_clear(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.conveyor_clear())
    }

    /// Moves converyor a set distance if a board is their then we stop
    pub fn conveyor_check_pcb(&self) -> Result<(), GeneralMachineError> {
        self.call(|machine| machine.conveyor_check_pcb())
    }

    /// Sets the track width
    pub fn track_set_width(&self, units: u16) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.track_set_width(units))
    }

    /// Resets the track
    pub fn track_reset(&self) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.track_reset())
    }

    pub fn canned_pnp(
        &self,
        nozzle: NozzleId,
        depth: f64,
        feeder: u8,
        delay: u8,
        kind: u8,
        speed: u8,
    ) -> Result<(), GeneralMachineError> {
        self.moving(|machine| machine.canned_pnp(nozzle, depth, feeder, delay, kind, speed))
    }

    fn call<T>(
        &self,
        operation: impl FnOnce(&mut dyn hwcontrol::Machine) -> hwcontrol::machine::Result<T>,
    ) -> Result<T, GeneralMachineError> {
        let mut guard = self
            .machine_connection
            .lock()
            .map_err(|_| GeneralMachineError::NotConnected)?;

        let machine = guard
            .as_deref_mut()
            .ok_or(GeneralMachineError::NotConnected)?;

        operation(machine).map_err(Into::into)
    }

    fn moving<T>(
        &self,
        operation: impl FnOnce(&mut dyn hwcontrol::Machine) -> hwcontrol::machine::Result<T>,
    ) -> Result<T, GeneralMachineError> {
        if *self.machine_state.lock().unwrap() != MachineState::Ready {
            return Err(GeneralMachineError::NotReady);
        }
        self.call(operation)
    }
}
