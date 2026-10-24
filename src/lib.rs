// The App view tree is deeply nested; rustc's layout computation needs a
// deeper recursion limit than the default on the wasm client crate.
#![recursion_limit = "512"]

#[cfg(feature = "ssr")]
use std::sync::Arc;

#[cfg(feature = "ssr")]
use axum::extract::FromRef;
#[cfg(feature = "ssr")]
use leptos::config::LeptosOptions;
#[cfg(feature = "ssr")]
use pnp_logic::*;

pub use shared_types::MachineState;

pub mod app;
#[cfg(feature = "ssr")]
pub mod calibration_store;
#[cfg(feature = "ssr")]
pub mod job_library;
#[cfg(feature = "ssr")]
pub mod job_plan;
#[cfg(feature = "ssr")]
pub mod job_runner;
#[cfg(feature = "ssr")]
pub mod job_store;
#[cfg(feature = "ssr")]
pub mod part_heights;
pub mod server_fns;

#[cfg(feature = "hydrate")]
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn hydrate() {
    console_error_panic_hook::set_once();
    leptos::mount::hydrate_islands();
}

#[cfg(feature = "ssr")]
#[derive(Clone, FromRef)]
pub struct AppState {
    pub machine: Arc<GeneralMachine>,
    pub leptos_options: LeptosOptions,
    pub camera_calibration_path: Arc<std::path::PathBuf>,
    pub store: job_store::Store,
    /// Per-footprint pickup/place heights recorded by the operator.
    pub heights: part_heights::HeightStore,
    /// Machine-side settings (cameras, machine config, feeder setup,
    /// feeder calibration). Not part of any job file; persisted in the
    /// settings database and injected when a job starts.
    pub machine_context: Arc<std::sync::RwLock<pnp_logic::MachineContext>>,
    pub job_executor: pnp_logic::JobExecutor,
    /// What the running job is asking the operator for.
    pub job_runner: job_runner::JobRunner,
}
