#![recursion_limit = "512"]

use axum::Router;
#[cfg(feature = "ssr")]
use axum::{
    body::Body,
    extract::{Path, State},
    http::{header, HeaderValue, StatusCode},
    response::{IntoResponse, Response},
    routing::get,
};
#[cfg(feature = "ssr")]
use bytes::Bytes;
use leptos::prelude::*;
#[cfg(feature = "ssr")]
use leptos_axum::{generate_route_list, LeptosRoutes};
use leptos_tailwind::app::{shell, App};
#[cfg(feature = "ssr")]
use leptos_tailwind::AppState;
#[cfg(feature = "ssr")]
use pnp_logic::GeneralMachine;
#[cfg(feature = "ssr")]
use rusqlite::Connection;
#[cfg(feature = "ssr")]
use std::sync::Arc;
#[cfg(feature = "ssr")]
use tokio_stream::wrappers::ReceiverStream;

#[cfg(feature = "ssr")]

#[tokio::main]
async fn main() {
    let conf = get_configuration(None).unwrap();
    let leptos_options = conf.leptos_options;
    let addr = leptos_options.site_addr;
    let routes = generate_route_list(App);
    let mut machine: GeneralMachine = GeneralMachine::new();

    let (serial_path, baud_rate, camera_path) = load_serial_settings("db.db");
    if let Err(error) = machine.connect_kayo(&serial_path, baud_rate) {
        eprintln!("unable to connect to machine at {serial_path} ({baud_rate}): {error}");
    } else {
        println!("connected to machine at {serial_path} ({baud_rate})");
    }

    for attempt in 1..=20 {
        match machine.add_camera(&camera_path, pnp_logic::CameraTypes::Head, None) {
            Ok(()) => break,
            Err(error) if attempt == 20 => {
                eprintln!("configured head camera {camera_path} unavailable: {error}");
            }
            Err(_) => std::thread::sleep(std::time::Duration::from_millis(250)),
        }
    }

    let camera_calibration_path = Arc::new(std::path::PathBuf::from("camera-calibration.json"));
    if let Ok(contents) = std::fs::read_to_string(&*camera_calibration_path) {
        if let Ok(calibration) =
            serde_json::from_str::<pnp_logic::PixelToMachineCalibration>(&contents)
        {
            if let Err(error) =
                machine.set_camera_calibration(&pnp_logic::CameraTypes::Head, calibration)
            {
                eprintln!("failed to load camera calibration: {error}");
            }
        }
    }

    load_nozzle_calibration(&machine);

    let feeder_pocket_settings_path = std::path::PathBuf::from("feeder-pocket-vision.json");
    if let Ok(contents) = std::fs::read_to_string(&feeder_pocket_settings_path) {
        if let Ok(settings) = serde_json::from_str::<
            std::collections::HashMap<String, pnp_logic::FeederPocketVisionSettings>,
        >(&contents)
        {
            for (camera, settings) in settings {
                let Ok(camera_type) = camera.parse::<pnp_logic::CameraTypes>() else {
                    continue;
                };
                if let Err(error) = machine.set_feeder_pocket_settings(&camera_type, settings) {
                    eprintln!("failed to load feeder-pocket settings for {camera}: {error}");
                }
            }
        }
    }

    let app_state = AppState {
        machine: Arc::new(machine),
        leptos_options: leptos_options.clone(),
        camera_calibration_path,
        store: match leptos_tailwind::job_store::Store::load() {
            Ok(store) => store,
            Err(error) => {
                eprintln!("{error}");
                std::process::exit(1);
            }
        },
        heights: match leptos_tailwind::part_heights::HeightStore::load() {
            Ok(heights) => heights,
            Err(error) => {
                eprintln!("{error}");
                std::process::exit(1);
            }
        },
        machine_context: Arc::new(std::sync::RwLock::new(load_machine_context())),
        job_executor: pnp_logic::JobExecutor::new(),
        job_runner: leptos_tailwind::job_runner::JobRunner::new(),
    };

    let app = Router::new()
        .route("/api/camera/{camera}", get(camera_stream))
        .leptos_routes(&app_state, routes, {
            let leptos_options = leptos_options.clone();
            move || shell(leptos_options.clone())
        })
        .fallback(leptos_axum::file_and_error_handler::<AppState, _>(shell))
        .with_state(app_state);

    let listener = tokio::net::TcpListener::bind(&addr).await.unwrap();
    println!("listening on http://{addr}");
    axum::serve(listener, app.into_make_service())
        .await
        .unwrap();
}

/// Load the stored nozzle calibration, if the machine has been calibrated.
///
/// Called at startup so a calibrated machine comes up already calibrated.
/// A missing database or table is not an error — it just means this
/// machine has never been calibrated, which is a normal state.
#[cfg(feature = "ssr")]
fn load_nozzle_calibration(machine: &pnp_logic::GeneralMachine) {
    let connection = match Connection::open(leptos_tailwind::calibration_store::DB_PATH) {
        Ok(connection) => connection,
        Err(error) => {
            eprintln!(
                "unable to open {}: {error}; nozzles are uncalibrated",
                leptos_tailwind::calibration_store::DB_PATH
            );
            return;
        }
    };
    let offsets = match leptos_tailwind::calibration_store::load_nozzle_offsets(&connection) {
        Ok(Some(offsets)) => offsets,
        Ok(None) => {
            println!("no nozzle calibration stored; using nominal offsets");
            return;
        }
        Err(error) => {
            eprintln!("unable to read nozzle calibration: {error}");
            return;
        }
    };

    if machine.load_nozzle_calibration(&offsets) {
        println!("loaded nozzle calibration for {} nozzles", offsets.len());
        for offset in &offsets {
            println!(
                "  nozzle {}: offset {:.3} mm, runout {:.3} mm",
                offset.nozzle.id(),
                offset.offset_mm,
                offset.runout_mm
            );
        }
    }
}

/// Load the machine-side settings (cameras, machine config, feeder
/// setup, feeder calibration) from the settings database.
///
/// Called at startup. A missing database or entry is not an error — it
/// just means the machine has never had its settings saved, so the
/// defaults apply. A corrupt blob is reported loudly: silently falling
/// back would make the machine forget its cameras and speeds.
#[cfg(feature = "ssr")]
fn load_machine_context() -> pnp_logic::MachineContext {
    let connection = match Connection::open(leptos_tailwind::calibration_store::DB_PATH) {
        Ok(connection) => connection,
        Err(error) => {
            eprintln!(
                "unable to open {}: {error}; machine settings fall back to defaults",
                leptos_tailwind::calibration_store::DB_PATH
            );
            return pnp_logic::MachineContext::default();
        }
    };
    match leptos_tailwind::calibration_store::load_machine_context(&connection) {
        Ok(Some(context)) => {
            println!(
                "loaded machine settings ({} cameras, {} feeder calibrations)",
                context.cameras.len(),
                context.calibration.len()
            );
            context
        }
        Ok(None) => {
            println!("no machine settings stored; using defaults until machine setup is saved");
            pnp_logic::MachineContext::default()
        }
        Err(error) => {
            eprintln!("unable to read machine settings: {error}; falling back to defaults");
            pnp_logic::MachineContext::default()
        }
    }
}

#[cfg(feature = "ssr")]
fn load_serial_settings(path: &str) -> (String, u32, String) {
    const DEFAULT_PATH: &str = "/dev/ttyS0";
    const DEFAULT_BAUD: u32 = 115200;

    let connection = match Connection::open(path) {
        Ok(connection) => connection,
        Err(error) => {
            eprintln!("unable to open settings database {path}: {error}");
            return (
                DEFAULT_PATH.to_string(),
                DEFAULT_BAUD,
                "/dev/video0".to_string(),
            );
        }
    };
    if let Err(error) = connection.execute_batch(
        "CREATE TABLE IF NOT EXISTS settings (key TEXT PRIMARY KEY NOT NULL, value TEXT NOT NULL)",
    ) {
        eprintln!("unable to initialize settings database: {error}");
        return (
            DEFAULT_PATH.to_string(),
            DEFAULT_BAUD,
            "/dev/video0".to_string(),
        );
    }

    let setting = |key: &str, default: &str| -> String {
        let value = connection
            .query_row("SELECT value FROM settings WHERE key = ?1", [key], |row| {
                row.get::<_, String>(0)
            })
            .ok();
        match value {
            Some(value) => value,
            None => {
                if let Err(error) = connection.execute(
                    "INSERT INTO settings (key, value) VALUES (?1, ?2)",
                    rusqlite::params![key, default],
                ) {
                    eprintln!("unable to save default setting {key}: {error}");
                }
                default.to_string()
            }
        }
    };

    let serial_path = setting("serial_path", DEFAULT_PATH);
    let baud_text = setting("serial_baud", &DEFAULT_BAUD.to_string());
    let baud_rate = match baud_text.parse::<u32>() {
        Ok(baud_rate) => baud_rate,
        Err(error) => {
            eprintln!("invalid serial_baud setting {baud_text:?}: {error}; using {DEFAULT_BAUD}");
            DEFAULT_BAUD
        }
    };
    let camera_path = setting("camera_path_head", "/dev/video0");
    (serial_path, baud_rate, camera_path)
}

#[cfg(feature = "ssr")]
async fn camera_stream(State(state): State<AppState>, Path(camera): Path<String>) -> Response {
    let Ok(camera_type) = camera.parse::<pnp_logic::CameraTypes>() else {
        return StatusCode::NOT_FOUND.into_response();
    };

    let (sender, receiver) = tokio::sync::mpsc::channel(1);
    let machine = state.machine.clone();
    tokio::spawn(async move {
        loop {
            let machine = machine.clone();
            let bytes =
                tokio::task::spawn_blocking(move || machine.camera_jpeg_bytes(&camera_type).ok())
                    .await
                    .ok()
                    .flatten();
            let Some(bytes) = bytes else {
                tokio::time::sleep(std::time::Duration::from_millis(100)).await;
                continue;
            };

            let mut part = Vec::with_capacity(bytes.len() + 80);
            part.extend_from_slice(
                b"--camera-frame\r\nContent-Type: image/jpeg\r\nContent-Length: ",
            );
            part.extend_from_slice(bytes.len().to_string().as_bytes());
            part.extend_from_slice(b"\r\n\r\n");
            part.extend_from_slice(&bytes);
            part.extend_from_slice(b"\r\n");
            if sender
                .send(Ok::<Bytes, std::convert::Infallible>(Bytes::from(part)))
                .await
                .is_err()
            {
                break;
            }
            tokio::time::sleep(std::time::Duration::from_millis(100)).await;
        }
    });

    let mut response = Body::from_stream(ReceiverStream::new(receiver)).into_response();
    response.headers_mut().insert(
        header::CONTENT_TYPE,
        HeaderValue::from_static("multipart/x-mixed-replace; boundary=camera-frame"),
    );
    response.headers_mut().insert(
        header::CACHE_CONTROL,
        HeaderValue::from_static("no-cache, no-store, must-revalidate"),
    );
    response
}

#[cfg(not(feature = "ssr"))]
pub fn main() {}
