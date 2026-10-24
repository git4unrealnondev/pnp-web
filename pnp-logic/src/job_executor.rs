//! Job execution: turning a saved [`JobFile`] into machine actions.
//!
//! A job runs by repeatedly calling [`Runner::step`], which returns one of:
//! - [`StepResult::Action`] — dispatch this to the hardware
//! - [`StepResult::NeedCamera`] — wait for a part observation
//! - [`StepResult::NeedPrompt`] — wait for the operator
//! - [`StepResult::Done`] / [`StepResult::Aborted`] — finished
//!
//! The executor owns the runner and the machine connection, so the web
//! layer only needs to call [`JobExecutor::step`] and act on what comes
//! back. This keeps the state machine in one place and makes it possible
//! to resume an interrupted job.

use crate::action::Action;
use crate::job_io::JobFile;
use crate::runner::{Runner, StepResult};
use crate::state::Placement;
use crate::placement::PartObservation;
use crate::geometry::Pose;
use std::sync::{Arc, Mutex};

/// Progress of a running job, for the UI.
#[derive(Debug, Clone, PartialEq)]
pub enum JobProgress {
    /// Not running.
    Idle,
    /// Running; carries a human-readable description of the current step.
    Running(String),
    /// Finished successfully.
    Done,
    /// Stopped; the reason is usually an error or an operator abort.
    Aborted(String),
}

/// Executes a [`JobFile`] against a [`GeneralMachine`].
///
/// The runner is created from the job file on [`JobExecutor::start`], so a
/// job can be re-run or resumed by calling `start` again with a different
/// file.
#[derive(Clone)]
pub struct JobExecutor {
    inner: Arc<Mutex<JobExecutorInner>>,
}

struct JobExecutorInner {
    runner: Option<Runner>,
    progress: JobProgress,
}

impl JobExecutor {
    pub fn new() -> Self {
        Self {
            inner: Arc::new(Mutex::new(JobExecutorInner {
                runner: None,
                progress: JobProgress::Idle,
            })),
        }
    }

    /// Current progress, for display.
    pub fn progress(&self) -> JobProgress {
        self.inner.lock().unwrap().progress.clone()
    }

    /// Start running `job`, replacing any previous run.
    ///
    /// `machine` supplies the machine-side settings (cameras, config,
    /// feeder setup, feeder calibration) that no longer live in the job
    /// file.
    ///
    /// Returns an error if the job file cannot be turned into a runner,
    /// which means the job cannot run at all.
    pub fn start(&self, job: JobFile, machine: crate::job_io::MachineContext) -> Result<(), String> {
        let runner = job.into_runner(machine)?;
        let mut inner = self.inner.lock().unwrap();
        inner.runner = Some(runner);
        inner.progress = JobProgress::Running("Starting".into());
        Ok(())
    }

    /// Stop the current run, if any.
    pub fn stop(&self) {
        self.abort("Stopped by operator");
    }

    /// Abort the current run with `reason`.
    ///
    /// Used by `stop` and by the host when an action could not be
    /// dispatched, so the state machine stops instead of continuing with
    /// the head somewhere unexpected.
    pub fn abort(&self, reason: impl Into<String>) {
        let reason = reason.into();
        let mut inner = self.inner.lock().unwrap();
        if let Some(runner) = &mut inner.runner {
            runner.abort(reason.clone());
        }
        inner.progress = JobProgress::Aborted(reason);
    }

    /// Advance the job by one step, returning what the caller must do.
    ///
    /// The returned value is `None` when there is nothing to do (no job is
    /// running, or the job has finished).
    pub fn step(&self) -> Option<StepResult> {
        let mut inner = self.inner.lock().unwrap();
        let runner = inner.runner.as_mut()?;

        // Drive the state machine until it needs something from us.
        loop {
            match runner.step() {
                StepResult::Action(action) => {
                    inner.progress = JobProgress::Running(describe_action(&action));
                    return Some(StepResult::Action(action));
                }
                StepResult::NeedCamera => {
                    inner.progress = JobProgress::Running("Waiting for camera".into());
                    return Some(StepResult::NeedCamera);
                }
                StepResult::NeedPrompt(message) => {
                    inner.progress = JobProgress::Running(format!("Prompt: {message}"));
                    return Some(StepResult::NeedPrompt(message));
                }
                StepResult::Done => {
                    inner.progress = JobProgress::Done;
                    inner.runner = None;
                    return None;
                }
                StepResult::Aborted(reason) => {
                    inner.progress = JobProgress::Aborted(reason.clone());
                    inner.runner = None;
                    return None;
                }
            }
        }
    }

    /// Whether a job is currently running.
    pub fn is_running(&self) -> bool {
        matches!(self.progress(), JobProgress::Running(_))
    }

    /// Feed back the up-camera's observation of the currently held part.
    ///
    /// Returns `false` when no job is running or the runner was not waiting
    /// for the camera, so the caller can tell a stale answer from one that
    /// was actually consumed.
    pub fn feed_camera(&self, observation: PartObservation) -> bool {
        let mut inner = self.inner.lock().unwrap();
        match inner.runner.as_mut() {
            Some(runner) if runner.awaiting_camera() => {
                runner.feed_camera(observation);
                true
            }
            _ => false,
        }
    }

    /// Tell the runner the operator answered the current prompt.
    ///
    /// Returns `false` when there was no prompt to answer.
    pub fn feed_prompt_response(&self) -> bool {
        let mut inner = self.inner.lock().unwrap();
        match inner.runner.as_mut() {
            Some(runner) if runner.awaiting_prompt() => {
                runner.feed_prompt_response();
                true
            }
            _ => false,
        }
    }

    /// Publish the board pose the caller recovered from the fiducials.
    ///
    /// Called while the runner is parked in `JobState::LocatingFiducials`.
    /// Returns `false` when no job is running (so a stale answer cannot
    /// leak into the next job).
    pub fn set_board_pose(&self, pose: Pose) -> bool {
        let mut inner = self.inner.lock().unwrap();
        match inner.runner.as_mut() {
            Some(runner) if runner.locating_board() => {
                runner.set_board_pose(pose);
                true
            }
            _ => false,
        }
    }

    /// The placements of the running job, including the `placed` flags the
    /// runner has flipped. Empty when nothing is running.
    ///
    /// Used to persist which placements made it onto the board so an
    /// interrupted job can be resumed instead of restarted.
    pub fn placements(&self) -> Vec<Placement> {
        self.inner
            .lock()
            .unwrap()
            .runner
            .as_ref()
            .map(|runner| runner.job().placements.clone())
            .unwrap_or_default()
    }
}

/// Human-readable description of an action, for the progress display.
fn describe_action(action: &Action) -> String {
    action.label()
}