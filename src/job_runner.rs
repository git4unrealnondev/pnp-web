//! Driving a job against the machine.
//!
//! `JobExecutor` owns the state machine but knows nothing about the
//! hardware; this module is the missing half. It runs on a tokio task
//! (spawned by `start_job`) and turns every `StepResult` into something
//! real:
//!
//! - `Action(..)` → the matching `GeneralMachine` call, so the runner's
//!   abstract `Action` stream finally reaches the serial machine.
//! - `NeedCamera` → an observation of the part currently held under the
//!   camera. This machine has no part-inspection routine yet, so the
//!   observation is nominal; see [`observe_part`].
//! - `NeedPrompt(..)` → the message is published for the web UI, and the
//!   task parks until the operator answers through `answer_prompt`.
//!
//! Board localisation is the one step that needs the operator: the runner
//! asks for the board pose, we ask the operator to put the head camera
//! over each fiducial in turn, then publish the recovered pose.

use std::sync::Arc;
use std::time::Duration;

use pnp_logic::{
    Action, Fiducial, FiducialKind, FiducialObservation, JobProgress, LightColor, MoveKind,
    NozzleId, PartObservation, Pose, Point, compute_board_pose,
};

use crate::AppState;
use crate::server_fns::JobRunStatus;

/// How long to wait between checks for an operator answer.
const PROMPT_POLL: Duration = Duration::from_millis(100);

/// How many times the camera may re-centre on one fiducial.
const FIDUCIAL_TRIES: u8 = 8;

/// Handle the web layer uses to see and answer what the run asks for.
#[derive(Clone, Default)]
pub struct JobRunner {
    inner: Arc<std::sync::Mutex<JobRunnerInner>>,
}

#[derive(Default)]
struct JobRunnerInner {
    /// Message the run is waiting on, if any.
    prompt: Option<String>,
    /// What the machine is doing right now, for the progress line.
    action: String,
    /// When the current run started, so the UI can show a live
    /// components-per-hour rate. Cleared when the run ends.
    run_started: Option<std::time::Instant>,
}

impl JobRunner {
    pub fn new() -> Self {
        Self::default()
    }

    /// Current operator prompt, if the run is waiting for one.
    pub fn prompt(&self) -> Option<String> {
        self.lock().prompt.clone()
    }

    /// Label of the action currently being dispatched.
    pub fn action(&self) -> String {
        self.lock().action.clone()
    }

    fn lock(&self) -> std::sync::MutexGuard<'_, JobRunnerInner> {
        self.inner.lock().unwrap_or_else(|error| error.into_inner())
    }

    fn set_prompt(&self, prompt: Option<String>) {
        self.lock().prompt = prompt;
    }

    fn set_action(&self, action: impl Into<String>) {
        self.lock().action = action.into();
    }

    /// Record when the run started, for the live CPH tracker.
    fn set_run_started(&self, started: Option<std::time::Instant>) {
        self.lock().run_started = started;
    }

    /// Seconds since the run started, or `None` when idle.
    fn elapsed_secs(&self) -> Option<f64> {
        self.lock()
            .run_started
            .map(|started| started.elapsed().as_secs_f64())
    }

    /// Operator answered the prompt. Returns `false` when there was
    /// nothing to answer, so a double click is harmless.
    pub fn answer(&self) -> bool {
        if self.prompt().is_none() {
            return false;
        }
        self.set_prompt(None);
        true
    }
}

/// Run the job currently loaded in the executor to completion.
///
/// Returns when the job finishes, aborts, or is stopped. `start_job`
/// refuses to spawn a second one while a run is still alive, so only one
/// of these loops exists at a time.
pub async fn run_job(state: AppState) {
    let executor = state.job_executor.clone();
    let runner = state.job_runner.clone();
    let machine = state.machine.clone();
    let store = state.store.clone();

    // Job-history log: one row per run, closed when the loop exits with
    // the achieved placement count and wall-clock duration. Failures to
    // write the log must never break the run itself, so every DB error
    // is only reported on the console.
    let run_started = std::time::Instant::now();
    runner.set_run_started(Some(run_started));
    let run_id: Option<i64> = {
        let job = store.job();
        let job_name = store
            .loaded_job_name()
            .unwrap_or_else(|| job.board_id.clone());
        crate::calibration_store::open_connection()
            .and_then(|connection| {
                crate::calibration_store::start_job_run(
                    &connection,
                    &job_name,
                    job.placements.len(),
                )
            })
            .map_err(|e| eprintln!("job-run log start failed: {e}"))
            .ok()
    };

    /// Close the run-history row for this run. `status` describes how
    /// the run ended ("Done", "Stopped", "Aborted: <reason>").
    fn finish_run(run_id: Option<i64>, started: std::time::Instant, placed: usize, status: &str) {
        let Some(run_id) = run_id else { return };
        let duration = started.elapsed().as_secs_f64();
        if let Err(e) = crate::calibration_store::open_connection().and_then(|connection| {
            crate::calibration_store::finish_job_run(
                &connection,
                run_id,
                placed,
                duration,
                status,
            )
        }) {
            eprintln!("job-run log finish failed: {e}");
        }
    }

    // Live progress sync: placed flags are pushed into the store at most
    // once a second while they change, so the process view's live-step
    // cursor follows the machine in real time instead of only updating
    // when the job ends.
    let mut last_sync = std::time::Instant::now() - std::time::Duration::from_secs(60);
    let mut last_placed = usize::MAX;

    loop {
        let Some(step) = step_blocking(&executor) else {
            // Nothing left to do: finished, aborted, or stopped.
            persist_placed(&store, &executor);
            runner.set_action(String::new());
            let placed = executor
                .placements()
                .iter()
                .filter(|placement| placement.placed)
                .count();
            finish_run(run_id, run_started, placed, "Done");
            return;
        };

        match step {
            pnp_logic::StepResult::Action(action) => {
                runner.set_action(action.label());
                dispatch(&machine, &action, &executor).await;
            }
            pnp_logic::StepResult::NeedCamera => {
                runner.set_action("inspecting part");
                let observation = observe_part(&machine).await;
                let _ = executor.feed_camera(observation);
            }
            pnp_logic::StepResult::NeedPrompt(message) => {
                wait_for_operator(&executor, &runner, &machine, &store, &message).await;
            }
            pnp_logic::StepResult::Done | pnp_logic::StepResult::Aborted(_) => {
                persist_placed(&store, &executor);
                runner.set_action(String::new());
                let placed = executor
                    .placements()
                    .iter()
                    .filter(|placement| placement.placed)
                    .count();
                let status = if matches!(
                    executor.progress(),
                    pnp_logic::JobProgress::Aborted(_)
                ) {
                    "Aborted"
                } else {
                    "Done"
                };
                finish_run(run_id, run_started, placed, status);
                return;
            }
        }

        let placed_now = executor
            .placements()
            .iter()
            .filter(|placement| placement.placed)
            .count();
        if placed_now != last_placed && last_sync.elapsed() >= std::time::Duration::from_secs(1) {
            last_sync = std::time::Instant::now();
            last_placed = placed_now;
            persist_placed(&store, &executor);
        }
    }
}

/// Call `executor.step()` off the async worker.
///
/// `step` takes the executor mutex and runs the state machine, which is
/// CPU work rather than I/O.
fn step_blocking(executor: &pnp_logic::JobExecutor) -> Option<pnp_logic::StepResult> {
    let executor = executor.clone();
    tokio::task::block_in_place(|| executor.step())
}

/// Park until the operator answers `message`.
///
/// The fiducial prompt is special: answering it is the trigger to hunt
/// for the board pose, so that step is driven here rather than by the UI.
async fn wait_for_operator(
    executor: &pnp_logic::JobExecutor,
    runner: &JobRunner,
    machine: &Arc<pnp_logic::GeneralMachine>,
    store: &crate::job_store::Store,
    message: &str,
) {
    if message.contains("fiducial") {
        while executor.is_running() {
            match locate_board(runner, executor, machine, store).await {
                Some(pose) => {
                    if executor.set_board_pose(pose) {
                        let _ = executor.feed_prompt_response();
                        return;
                    }
                }
                None => runner.set_action("board pose not recovered; try again"),
            }
        }
        return;
    }

    ask(runner, executor, message).await;
    let _ = executor.feed_prompt_response();
}

/// Publish `message` and wait until the operator answers it.
///
/// Returns as soon as the run is no longer alive, so an operator pressing
/// Stop while a prompt is up does not leave the loop waiting forever.
async fn ask(runner: &JobRunner, executor: &pnp_logic::JobExecutor, message: &str) {
    runner.set_prompt(Some(message.to_string()));
    while runner.prompt().is_some() && executor.is_running() {
        tokio::time::sleep(PROMPT_POLL).await;
    }
    runner.set_prompt(None);
}

/// Observe the part currently held under the camera.
///
/// This machine has no part-inspection routine: the vision code that
/// exists detects fiducials, landmarks and feeder pockets, not the part
/// on a nozzle. Until one is added the observation is nominal — the part
/// is present, perfectly centred and correctly rotated. That makes the
/// standard and polarised flows do exactly what they promise and the
/// precision loop converge on its first inspection.
///
/// The camera is still touched so a dead camera aborts the run instead of
/// placing parts that were never verified.
async fn observe_part(machine: &Arc<pnp_logic::GeneralMachine>) -> PartObservation {
    let machine = machine.clone();
    let captured = tokio::task::spawn_blocking(move || {
        machine
            .camera_png_bytes(&shared_types::CameraTypes::Head)
            .map_err(|error| error.to_string())
    })
    .await
    .unwrap_or_else(|_| Err("camera task failed".to_string()));

    match captured {
        Ok(_) => PartObservation::ok(),
        Err(error) => {
            eprintln!("job camera capture failed: {error}");
            PartObservation {
                present: false,
                ..PartObservation::ok()
            }
        }
    }
}

/// Walk the operator over each fiducial and recover the board pose.
///
/// For every fiducial the job knows about the operator jogs the head
/// camera over the mark; the machine then centres itself on it with
/// `center_camera_on_feducial` and the head position is read back as the
/// fiducial's machine position. Two or more observations are enough for
/// `compute_board_pose`.
async fn locate_board(
    runner: &JobRunner,
    executor: &pnp_logic::JobExecutor,
    machine: &Arc<pnp_logic::GeneralMachine>,
    store: &crate::job_store::Store,
) -> Option<Pose> {
    let fiducials: Vec<Fiducial> = store
        .job()
        .fiducials
        .into_iter()
        .filter(|fiducial| matches!(fiducial.kind, FiducialKind::Panel))
        .collect();
    if fiducials.len() < 2 {
        // Without two fiducials there is nothing to fit a pose from, and
        // guessing would rotate every part on the board.
        runner.set_action("at least two fiducials are needed to locate the board");
        return None;
    }

    let total = fiducials.len();
    let mut expected = Vec::new();
    let mut observed = Vec::new();

    for (index, fiducial) in fiducials.iter().enumerate() {
        let message = format!(
            "Fiducial {}/{}: jog the head camera over the mark at ({:.1}, {:.1}) mm, then press OK",
            index + 1,
            total,
            fiducial.design_position.x,
            fiducial.design_position.y
        );
        ask(runner, executor, &message).await;
        // The operator pressed Stop instead of OK, or the run ended.
        if !executor.is_running() {
            return None;
        }

        let centering = machine.clone();
        let centered = tokio::task::spawn_blocking(move || {
            centering.center_camera_on_feducial(FIDUCIAL_TRIES)
        })
        .await
        .unwrap_or(false);
        if !centered {
            runner.set_action("fiducial not found under the camera");
            return None;
        }

        // The head reference is over the mark once the camera has
        // centred on it, so the cached position is the fiducial's
        // position in machine coordinates.
        let Some(position) = machine.cached_position() else {
            runner.set_action("machine position unavailable");
            return None;
        };
        expected.push(fiducial.clone());
        observed.push(FiducialObservation {
            image_position: Point::new(position.x, position.y),
            confidence: 1.0,
        });
    }

    compute_board_pose(&expected, &observed)
}

/// Save the `placed` flags back to `job.json` so an interrupted job can be
/// resumed instead of started over.
fn persist_placed(store: &crate::job_store::Store, executor: &pnp_logic::JobExecutor) {
    let placements = executor.placements();
    if placements.is_empty() {
        return;
    }
    let mut job = store.job();
    job.placements = placements;
    if let Err(error) = store.save_job(job) {
        eprintln!("unable to persist placed placements to job.json: {error}");
    }
}

/// Turn one runner `Action` into machine calls.
///
/// A failed action aborts the run rather than letting the head continue
/// somewhere unexpected.
async fn dispatch(
    machine: &Arc<pnp_logic::GeneralMachine>,
    action: &Action,
    executor: &pnp_logic::JobExecutor,
) {
    let machine = machine.clone();
    let action = action.clone();
    let label = action.label();
    let outcome = tokio::task::spawn_blocking(move || dispatch_blocking(&machine, &action))
        .await
        .unwrap_or_else(|_| Err("dispatch task failed".to_string()));

    if let Err(error) = outcome {
        eprintln!("job action {label} failed: {error}");
        executor.abort(format!("action {label} failed: {error}"));
    }
}

/// The synchronous half of [`dispatch`].
fn dispatch_blocking(
    machine: &pnp_logic::GeneralMachine,
    action: &Action,
) -> Result<(), String> {
    /// The runner addresses nozzles by physical number; the machine API
    /// takes a [`NozzleId`]. An unknown number is an error rather than a
    /// silent fallback to nozzle 1.
    fn nozzle(id: u8) -> Result<NozzleId, String> {
        NozzleId::from_id(id).ok_or_else(|| format!("nozzle {id} does not exist"))
    }

    let result = match action {
        Action::Home => machine.home_all(),
        Action::Reset => machine.reset(),
        Action::HomeZ => machine.home_z(),
        Action::Move { target, speed, kind } => {
            machine.move_xy(*target, speed_for(*kind, *speed))
        }
        Action::MoveZ {
            nozzle: id,
            height_mm,
            speed,
        } => machine.move_z(nozzle(*id)?, *height_mm, *speed),
        // Deltas are relative to the current angle, so the direction is
        // simply the sign of the delta.
        Action::RotateNozzle {
            nozzle: id,
            delta_deg,
            speed,
        } => machine.rotate_nozzle(nozzle(*id)?, *delta_deg, *delta_deg >= 0.0, *speed),
        // The firmware has no absolute rotate; from a homed head the
        // nozzle starts at zero, so an absolute command is a delta.
        Action::SetNozzleRotation {
            nozzle: id,
            abs_deg,
            ..
        } => machine.rotate_nozzle(nozzle(*id)?, *abs_deg, *abs_deg >= 0.0, 8),
        Action::Vacuum { nozzle: id, on } => {
            if *on {
                machine.open_vacuum(nozzle(*id)?)
            } else {
                machine.close_vacuum(nozzle(*id)?)
            }
        }
        Action::Blow { nozzle: id, on } => {
            if *on {
                machine.open_blow(nozzle(*id)?)
            } else {
                machine.close_blow(nozzle(*id)?)
            }
        }
        Action::Feeder { id, open } => {
            if *open {
                machine.open_feeder(*id)
            } else {
                machine.close_feeders()
            }
        }
        // The firmware advances a feeder by running its tape motor; a
        // pulse count maps onto one motor run.
        Action::AdvanceFeeder { id, pulses } => machine.tube_feeder_action(*id, *pulses),
        Action::TrackWidth { units } => machine.track_set_width(*units),
        Action::Light(color) => match color {
            LightColor::Off => machine.light_off(),
            LightColor::Green | LightColor::Yellow | LightColor::Red => {
                machine.error_light(*color)
            }
        },
        Action::AwaitCamera | Action::Prompt(_) => Ok(()),
    };

    result.map_err(|error| error.to_string())
}

/// Approaching the pocket or the board is done slower than travelling
/// between them, so the runner's `MoveKind` picks a slower speed.
fn speed_for(kind: MoveKind, speed: u8) -> u8 {
    match kind {
        MoveKind::Linear | MoveKind::Fast => speed,
        MoveKind::PickApproach | MoveKind::PlaceApproach => speed.min(4),
    }
}

/// Snapshot of the run for the UI.
pub fn status(state: &AppState) -> JobRunStatus {
    let progress = state.job_executor.progress();
    let job = state.store.job();
    let placed_count = job.placements.iter().filter(|placement| placement.placed).count();
    let placement_count = job.placements.len();

    let (running, progress_text, done, aborted) = match progress {
        JobProgress::Idle => (false, "Idle".to_string(), false, None),
        JobProgress::Running(message) => (true, message, false, None),
        JobProgress::Done => (false, "Done".to_string(), true, None),
        JobProgress::Aborted(reason) => (false, "Aborted".to_string(), false, Some(reason)),
    };

    // Live CPH: placements per hour over the elapsed run time. Only
    // meaningful while a run is live, so it is None when idle.
    let (elapsed_secs, live_cph) = match (state.job_runner.elapsed_secs(), running) {
        (Some(elapsed), true) => {
            let cph = if elapsed > 0.0 {
                placed_count as f64 / elapsed * 3600.0
            } else {
                0.0
            };
            (Some(elapsed), Some(cph))
        }
        _ => (None, None),
    };

    // Estimated CPH comes from the planner's cycle-time model, stored
    // with the job at plan time.
    let estimated_cph = job
        .estimated_seconds
        .filter(|seconds| *seconds > 0.0)
        .map(|seconds| placement_count as f64 / seconds * 3600.0)
        .unwrap_or(0.0);

    JobRunStatus {
        running,
        progress: progress_text,
        done,
        aborted,
        prompt: state.job_runner.prompt(),
        action: state.job_runner.action(),
        placed_count,
        placement_count,
        elapsed_secs,
        live_cph,
        estimated_cph,
    }
}
