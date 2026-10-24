use std::collections::HashMap;

#[cfg(feature = "ssr")]
use crate::AppState;
#[cfg(feature = "ssr")]
use crate::calibration_store;
#[cfg(feature = "ssr")]
use crate::job_store;
#[cfg(feature = "ssr")]
use crate::job_runner;
#[cfg(feature = "ssr")]
use rusqlite::Connection;
#[cfg(feature = "ssr")]
use base64::{engine::general_purpose::STANDARD, Engine};
#[cfg(feature = "ssr")]
use leptos::context::use_context;
use leptos::{server, server_fn::ServerFnError};
#[cfg(feature = "ssr")]
use pnp_logic::*;

pub use shared_types::MachineState;

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MachineStatus {
    pub state: MachineState,
    pub x: f64,
    pub y: f64,
    pub z: [f64; 4],
}

#[cfg(feature = "ssr")]
fn camera_type(name: &str) -> Result<shared_types::CameraTypes, ServerFnError> {
    name.parse()
        .map_err(|()| server_error(format!("unknown camera: {name}")))
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FeederPocketSettingsResponse {
    pub size_mm: f32,
    pub position_tolerance_mm: f32,
    pub size_tolerance_mm: f32,
    pub px_per_mm: f32,
}

#[server(prefix = "/api", endpoint = "get_feeder_pocket_settings")]
pub async fn get_feeder_pocket_settings(
    camera: String,
) -> Result<FeederPocketSettingsResponse, ServerFnError> {
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let selected_camera_type = camera_type(&camera)?;
    let settings = state.machine.feeder_pocket_settings(&selected_camera_type);
    Ok(FeederPocketSettingsResponse {
        size_mm: settings.size_mm,
        position_tolerance_mm: settings.position_tolerance_mm,
        size_tolerance_mm: settings.size_tolerance_mm,
        px_per_mm: settings.px_per_mm,
    })
}

#[server(prefix = "/api", endpoint = "set_feeder_pocket_settings")]
pub async fn set_feeder_pocket_settings(
    camera: String,
    size_mm: String,
    position_tolerance_mm: String,
    size_tolerance_mm: String,
    px_per_mm: String,
) -> Result<String, ServerFnError> {
    let parse = |value: &str, name: &str| {
        value
            .parse::<f32>()
            .map_err(|_| server_error(format!("invalid {name}")))
    };
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let selected_camera_type = camera_type(&camera)?;
    let settings = pnp_logic::FeederPocketVisionSettings {
        size_mm: parse(&size_mm, "pocket size")?,
        position_tolerance_mm: parse(&position_tolerance_mm, "position tolerance")?,
        size_tolerance_mm: parse(&size_tolerance_mm, "size tolerance")?,
        px_per_mm: parse(&px_per_mm, "pixels per millimeter")?,
    };
    state
        .machine
        .set_feeder_pocket_settings(&selected_camera_type, settings)
        .map_err(|error| server_error(error.to_string()))?;
    let all: std::collections::HashMap<&str, _> = shared_types::CameraTypes::ALL
        .into_iter()
        .map(|camera| {
            (
                camera.name(),
                state.machine.feeder_pocket_settings(&camera),
            )
        })
        .collect();
    let serialized =
        serde_json::to_string_pretty(&all).map_err(|error| server_error(error.to_string()))?;
    std::fs::write("feeder-pocket-vision.json", serialized)
        .map_err(|error| server_error(error.to_string()))?;
    Ok(format!("Feeder-pocket vision settings saved for {camera}"))
}

#[cfg(feature = "ssr")]
fn server_error(message: impl Into<String>) -> ServerFnError {
    ServerFnError::ServerError(message.into())
}

/// Runs action on client
#[server(prefix = "/api", endpoint = "run_action")]
pub async fn run_action(
    action: String,
    #[server(default)] actions: HashMap<String, String>,
) -> Result<(), ServerFnError> {
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let machine = &state.machine;
    let value = |key: &str| actions.get(key).map(String::as_str);
    let required = |key: &str| -> Result<&str, ServerFnError> {
        value(key).ok_or_else(|| server_error(format!("missing argument: {key}")))
    };
    let parse = |key: &str| -> Result<f64, ServerFnError> {
        required(key)?
            .parse()
            .map_err(|_| server_error(format!("invalid number for argument: {key}")))
    };
    let parse_u8 = |key: &str| -> Result<u8, ServerFnError> {
        required(key)?
            .parse()
            .map_err(|_| server_error(format!("invalid integer for argument: {key}")))
    };
    let parse_i8 = |key: &str| -> Result<i8, ServerFnError> {
        required(key)?
            .parse()
            .map_err(|_| server_error(format!("invalid signed integer for argument: {key}")))
    };
    let parse_u16 = |key: &str| -> Result<u16, ServerFnError> {
        required(key)?
            .parse()
            .map_err(|_| server_error(format!("invalid integer for argument: {key}")))
    };
    // `nozzle` is the physical 1-based nozzle number the operator sees on
    // the head, so it maps through `NozzleId::from_id` rather than being
    // matched by hand. Matching by hand previously shifted every command
    // by one and left nozzle 1 unreachable.
    let nozzle = || -> Result<NozzleId, ServerFnError> {
        let id = parse_u8("nozzle")?;
        NozzleId::from_id(id).ok_or_else(|| {
            server_error(format!("nozzle must be 1..={}, got {id}", NozzleId::ALL.len()))
        })
    };
    let boolean = |key: &str| -> Result<bool, ServerFnError> {
        match required(key)?.to_ascii_lowercase().as_str() {
            "1" | "true" | "on" | "yes" => Ok(true),
            "0" | "false" | "off" | "no" => Ok(false),
            _ => Err(server_error(format!("invalid boolean for argument: {key}"))),
        }
    };

    let result = match action.as_str() {
        "home" | "home_all" => machine.home_all(),
        "home_z" => machine.home_z(),
        "reset" => machine.reset(),
        "move_xy" => machine.move_xy(Point::new(parse("x")?, parse("y")?), parse_u8("speed")?),
        "move_relative" => {
            let position = machine
                .cached_position()
                .ok_or_else(|| server_error("machine is not connected"))?;
            let distance = parse("distance")?;
            let (x, y) = match required("direction")? {
                "up" => (0.0, distance),
                "down" => (0.0, -distance),
                "left" => (-distance, 0.0),
                "right" => (distance, 0.0),
                direction => return Err(server_error(format!("invalid direction: {direction}"))),
            };
            let final_pos = Point::new(position.x + x, position.y + y);
            machine.move_xy(final_pos, parse_u8("speed")?)
        }
        "jog_xy" => machine.jog_xy(JogRequest {
            x: parse_i8("x")?,
            y: parse_i8("y")?,
            speed: parse_u8("speed")?,
        }),
        "move_z" => machine.move_z(nozzle()?, parse("height")?, parse_u8("speed")?),
        "rotate_nozzle" => machine.rotate_nozzle(
            nozzle()?,
            parse("angle")?,
            boolean("clockwise")?,
            parse_u8("speed")?,
        ),
        "rotate_nozzles" => {
            let directions = required("directions")?
                .split(',')
                .map(|s| matches!(s.trim(), "1" | "true" | "on"))
                .collect::<Vec<_>>();
            machine.rotate_nozzles(&directions, parse_u8("speed")?)
        }
        "open_vacuum" => machine.open_vacuum(nozzle()?),
        "close_vacuum" => machine.close_vacuum(nozzle()?),
        "open_blow" => machine.open_blow(nozzle()?),
        "close_blow" => machine.close_blow(nozzle()?),
        "close_vacuum_open_blow" => machine.close_vacuum_open_blow(nozzle()?),
        "close_vacuum_blow_all" => machine.close_vacuum_blow_all(),
        "open_supplement_valve" => machine.open_supplement_valve(),
        "close_supplement_valve" => machine.close_supplement_valve(),
        "open_feeder" => machine.open_feeder(parse_u8("id")?),
        "close_feeders" => machine.close_feeders(),
        "tube_feeder_action" => machine.tube_feeder_action(parse_u8("p1")?, parse_u8("p2")?),
        "clip_pcb" => machine.clip_pcb(),
        "release_pcb" => machine.release_pcb(),
        "conveyor_run" => machine.conveyor_run(),
        "conveyor_stop" => machine.conveyor_stop(),
        "conveyor_stop_pin_engage" => machine.conveyor_stop_pin_engage(),
        "conveyor_clip_pcb" => machine.conveyor_clip_pcb(),
        "conveyor_unclip_and_move" => machine.conveyor_unclip_and_move(),
        "conveyor_clear" => machine.conveyor_clear(),
        "conveyor_check_pcb" => machine.conveyor_check_pcb(),
        "track_set_width" => machine.track_set_width(parse_u16("units")?),
        "track_reset" => machine.track_reset(),
        "light_on" => machine.light_on(parse_u8("id")?, parse_u8("brightness")?),
        "light_off" => machine.light_off(),
        "error_light" => {
            machine.error_light(match required("color")?.to_ascii_lowercase().as_str() {
                "red" => LightColor::Red,
                "yellow" => LightColor::Yellow,
                "green" => LightColor::Green,
                "off" => LightColor::Off,
                _ => return Err(server_error("invalid light color")),
            })
        }
        "canned_pnp" => machine.canned_pnp(
            nozzle()?,
            parse("depth")?,
            parse_u8("feeder")?,
            parse_u8("delay")?,
            parse_u8("kind")?,
            parse_u8("speed")?,
        ),
        _ => return Err(server_error(format!("unknown action: {action}"))),
    };

    result.map_err(|error| server_error(error.to_string()))
}

/// Calibrates nozzle tip positions by ink marking.
///
/// The operator inks all four tips, lays paper on the deck, and parks the
/// head over it. This walks the four cardinal rotations, touching down and
/// reading each mark, then reports the centroid of each nozzle's marks
/// together with its runout.
///
/// Long-running and drives real hardware, so it runs on the blocking pool.
/// Requires the selected camera to be calibrated first: ink positions are
/// observed in pixels and only become millimetres through that transform.
#[server(prefix = "/api", endpoint = "calibrate_nozzles")]
pub async fn calibrate_nozzles(
    head_x: String,
    head_y: String,
    touch_z: String,
    camera: String,
) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let parse = |value: &str, name: &str| {
            value
                .parse::<f64>()
                .map_err(|_| server_error(format!("invalid {name}")))
        };
        let head_position = shared_types::Point::new(
            parse(&head_x, "head X")?,
            parse(&head_y, "head Y")?,
        );
        let plan = pnp_logic::NozzleCalibrationPlan {
            camera_type: camera_type(&camera)?,
            ..pnp_logic::NozzleCalibrationPlan::new(head_position, parse(&touch_z, "touch Z")?)
        };

        let head_position = plan.head_position;
        let nominal = pnp_logic::GeneralMachine::nominal_offsets_mm(&plan.nozzles);

        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let machine = state.machine.clone();
        let results = tokio::task::spawn_blocking(move || {
            machine.calibrate_nozzles(&plan).map_err(|error| error.to_string())
        })
        .await
        .map_err(|error| server_error(error.to_string()))?
        .map_err(|error| server_error(format!("Nozzle calibration failed: {error}")))?;

        // Reduce centroids to head-relative offsets. This rejects an
        // implausible correction before anything is written, so a bad run
        // cannot replace a good stored calibration.
        let offsets = pnp_logic::offsets_from_calibration(&results, head_position, &nominal)
            .map_err(|error| server_error(format!("Nozzle calibration rejected: {error}")))?;

        // Persist to the database, then update the in-memory copy so anything
        // reading the machine in this process sees the new offsets without
        // waiting for a restart.
        let connection = Connection::open(calibration_store::DB_PATH).map_err(|error| {
            server_error(format!(
                "unable to open {}: {error}",
                calibration_store::DB_PATH
            ))
        })?;
        calibration_store::save_nozzle_offsets(&connection, &offsets).map_err(|error| {
            server_error(format!("unable to save nozzle calibration: {error}"))
        })?;
        state.machine.store_nozzle_calibration(&offsets);

        // Surface runout as well as the centre: a large runout is a worn
        // or bent nozzle, which no amount of re-centring will fix.
        let mut lines = Vec::new();
        for offset in &offsets {
            lines.push(format!(
                "Nozzle {}: offset {:.3} mm, runout {:.3} mm",
                nozzle_label(offset.nozzle),
                offset.offset_mm,
                offset.runout_mm
            ));
        }
        Ok(lines.join("\n"))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (head_x, head_y, touch_z, camera);
        unreachable!()
    }
}

/// 1-based label for a nozzle, matching what the operator sees on the
/// head.
#[cfg(feature = "ssr")]
fn nozzle_label(nozzle: shared_types::NozzleId) -> u8 {
    nozzle.id()
}

/// Gets machine status
#[server(prefix = "/api", endpoint = "get_status")]
pub async fn get_status() -> Result<MachineStatus, ServerFnError> {
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let position = state
        .machine
        .cached_position()
        .ok_or_else(|| server_error("machine is not connected"))?;

    Ok(MachineStatus {
        state: state.machine.state().clone(),
        x: position.x,
        y: position.y,
        z: position.z,
    })
}

/// Returns a compressed preview frame from the selected camera as a browser data URL.
#[server(prefix = "/api", endpoint = "get_camera_frame")]
pub async fn get_camera_frame(camera: String) -> Result<String, ServerFnError> {
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let camera_type = camera_type(&camera)?;
    let bytes = state
        .machine
        .camera_jpeg_bytes(&camera_type)
        .map_err(|error| server_error(error.to_string()))?;

    Ok(format!("data:image/jpeg;base64,{}", STANDARD.encode(bytes)))
}

/// Calibrate the head camera against four known machine coordinates.
#[server(prefix = "/api", endpoint = "calibrate_camera_grid")]
pub async fn calibrate_camera_grid(
    x1: String,
    y1: String,
    x2: String,
    y2: String,
    x3: String,
    y3: String,
    x4: String,
    y4: String,
) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let parse = |value: &str, name: &str| {
            value
                .parse::<f64>()
                .map_err(|_| server_error(format!("invalid {name}")))
        };
        let points = [
            Point::new(parse(&x1, "x1")?, parse(&y1, "y1")?),
            Point::new(parse(&x2, "x2")?, parse(&y2, "y2")?),
            Point::new(parse(&x3, "x3")?, parse(&y3, "y3")?),
            Point::new(parse(&x4, "x4")?, parse(&y4, "y4")?),
        ];
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let machine = state.machine.clone();
        let calibration = tokio::task::spawn_blocking(move || {
            machine
                .calibrate_camera_grid(&CameraTypes::Head, &points)
                .map_err(|error| error.to_string())
        })
        .await
        .map_err(|error| server_error(error.to_string()))?
        .map_err(|error| server_error(error.to_string()))?;
        let serialized =
            serde_json::to_string(&calibration).map_err(|error| server_error(error.to_string()))?;
        std::fs::write(&*state.camera_calibration_path, serialized)
            .map_err(|error| server_error(error.to_string()))?;
        return Ok(format!(
            "Camera calibrated (RMS error {:.3} mm)",
            calibration.rms_error_mm
        ));
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (x1, y1, x2, y2, x3, y3, x4, y4);
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "calibrate_camera_3x3")]
pub async fn calibrate_camera_3x3(
    origin_x: String,
    origin_y: String,
) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let parse = |value: &str, name: &str| {
            value
                .parse::<f64>()
                .map_err(|_| server_error(format!("invalid {name}")))
        };
        let origin_x = parse(&origin_x, "origin X")?;
        let origin_y = parse(&origin_y, "origin Y")?;
        const GRID_SPACING_MM: f64 = 10.0;
        let mut points = Vec::with_capacity(9);
        for row in 0..3 {
            for column in 0..3 {
                points.push(Point::new(
                    origin_x + (column as f64 - 1.0) * GRID_SPACING_MM,
                    origin_y + (row as f64 - 1.0) * GRID_SPACING_MM,
                ));
            }
        }
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let machine = state.machine.clone();
        let calibration = tokio::task::spawn_blocking(move || {
            machine
                .calibrate_camera_points(&CameraTypes::Head, &points)
                .map_err(|error| error.to_string())
        })
        .await
        .map_err(|error| server_error(error.to_string()))?
        .map_err(|error| server_error(error.to_string()))?;
        let serialized =
            serde_json::to_string(&calibration).map_err(|error| server_error(error.to_string()))?;
        std::fs::write(&*state.camera_calibration_path, serialized)
            .map_err(|error| server_error(error.to_string()))?;
        return Ok(format!(
            "3x3 camera calibration complete (RMS error {:.3} mm)",
            calibration.rms_error_mm
        ));
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (origin_x, origin_y);
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "center_head_fiducial")]
pub async fn center_head_fiducial() -> Result<String, ServerFnError> {
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let machine = state.machine.clone();
    tokio::task::spawn_blocking(move || {
        if machine.center_camera_on_feducial(5) {
            Ok("Fiducial centered".to_string())
        } else {
            Err(server_error("unable to detect and center fiducial"))
        }
    })
    .await
    .map_err(|error| server_error(error.to_string()))?
}

#[server(prefix = "/api", endpoint = "center_feeder_pocket")]
pub async fn center_feeder_pocket(camera: String) -> Result<String, ServerFnError> {
    let state: AppState =
        use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
    let camera_type = camera_type(&camera)?;
    if state.machine.camera_calibration(&camera_type).is_none() {
        return Err(server_error(
            "camera calibration is required before centering a feeder pocket; center a fiducial and calibrate the camera first",
        ));
    }
    let machine = state.machine.clone();
    tokio::task::spawn_blocking(move || {
        if machine.center_camera_on_feeder_pocket(&camera_type, 5) {
            Ok(format!("{camera} camera centered on feeder pocket"))
        } else {
            Err(server_error("unable to detect and center feeder pocket"))
        }
    })
    .await
    .map_err(|error| server_error(error.to_string()))?
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct JobSummary {
    pub board_id: String,
    pub placement_count: usize,
    pub placed_count: usize,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PartsSummary {
    pub parts: Vec<shared_types::PartSummary>,
    pub blockers: Vec<String>,
    /// Cleanup suggestions that do not stop the job (e.g. duplicate
    /// value-footprint parts).
    pub warnings: Vec<String>,
}

/// Everything the UI's part/job views need, in one snapshot.
///
/// The single source of truth for the client: one request, one fetch
/// time, so the parts table, blockers, warnings, job counts and
/// recorded heights can never disagree with each other the way
/// independently-fetched signals did.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct AppStateSnapshot {
    pub job: JobSummary,
    pub parts: Vec<shared_types::PartSummary>,
    pub blockers: Vec<String>,
    pub warnings: Vec<String>,
    pub heights: Vec<HeightEntry>,
}

/// Build the [`shared_types::PartSummary`] list from the library.
#[cfg(feature = "ssr")]
fn part_summaries(parts: &[pnp_logic::SetupPart]) -> Vec<shared_types::PartSummary> {
    parts
        .iter()
        .map(|part| shared_types::PartSummary {
            id: part.part.id.0,
            mpn: part.part.mpn.clone(),
            footprint: part.part.footprint.name.clone(),
            description: part.part.description.clone(),
            nozzles: serde_json::to_string(&part.part.nozzle_type).unwrap_or_default(),
            rotation: part.part.pickup_rotation_deg,
            offset_x: part.part.pickup_offset.x,
            offset_y: part.part.pickup_offset.y,
            status: format!("{:?}", part.status),
            camera: match part.part.camera {
                Some(pnp_logic::config::CameraKind::FastVision) => "fast".to_string(),
                Some(pnp_logic::config::CameraKind::Precise) => "precise".to_string(),
                Some(pnp_logic::config::CameraKind::HeadCamera) => "head".to_string(),
                None => "auto".to_string(),
            },
        })
        .collect()
}

#[server(prefix = "/api", endpoint = "get_app_state")]
pub async fn get_app_state() -> Result<AppStateSnapshot, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let library = state.store.parts();
        let job_summary = JobSummary {
            board_id: job.board_id.clone(),
            placement_count: job.placements.len(),
            placed_count: job.placements.iter().filter(|p| p.placed).count(),
        };
        Ok(AppStateSnapshot {
            job: job_summary,
            parts: part_summaries(&library),
            blockers: job_store::blockers(&job, &library),
            warnings: job_store::duplicate_warnings(&library),
            heights: {
                let mut entries: Vec<HeightEntry> = state
                    .heights
                    .map()
                    .into_iter()
                    .map(|(footprint, h)| HeightEntry {
                        footprint,
                        pickup_z_mm: h.pickup_z_mm,
                        place_z_mm: h.place_z_mm,
                        width_mm: h.width_mm,
                        height_mm: h.height_mm,
                    })
                    .collect();
                entries.sort_by(|a, b| a.footprint.cmp(&b.footprint));
                entries
            },
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "get_job")]
pub async fn get_job() -> Result<JobSummary, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let placed_count = job.placements.iter().filter(|p| p.placed).count();
        Ok(JobSummary {
            board_id: job.board_id.clone(),
            placement_count: job.placements.len(),
            placed_count,
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "get_parts")]
pub async fn get_parts() -> Result<PartsSummary, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let parts = state.store.parts();
        let blockers = job_store::blockers(&job, &parts);
        let warnings = job_store::duplicate_warnings(&parts);
        Ok(PartsSummary {
            parts: part_summaries(&parts),
            blockers,
            warnings,
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "save_job")]
pub async fn save_job(
    board_id: String,
    placements: String,
) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let mut job = state.store.job();
        job.board_id = board_id;
        job.placements = serde_json::from_str(&placements)
            .map_err(|e| server_error(format!("invalid placements JSON: {e}")))?;
        state
            .store
            .save_job(job)
            .map_err(|e| server_error(format!("Unable to save job: {e}")))?;
        Ok("Job saved".to_string())
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (board_id, placements);
        unreachable!()
    }
}

/// One row of the placement list, as the editor sends it.
///
/// The editor works in plain text; this is the resolved, numeric form the
/// planner needs. Only the fields the planner actually uses are sent.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PlacementRow {
    /// Designator, e.g. `R12`. Fiducial rows use it as a label only.
    pub reference: String,
    /// Value / MPN, e.g. `10k`.
    pub value: String,
    /// Footprint name, e.g. `0603`. Used to look the part up in the
    /// library, or to guess its parameters when it is not there yet.
    pub footprint: String,
    pub x: f64,
    pub y: f64,
    #[serde(default)]
    pub rotation: f64,
    /// `true` when the operator marked the row as a fiducial rather than
    /// a component.
    #[serde(default)]
    pub fiducial: bool,
}

/// Result of planning a placement list, for the UI to report.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PlanSummary {
    #[serde(default)]
    pub placement_count: usize,
    #[serde(default)]
    pub part_count: usize,
    /// Distinct parts the job will draw from a feeder.
    #[serde(default)]
    pub unassigned_parts: Vec<String>,
    /// Rows that could not be turned into a placement at all.
    #[serde(default)]
    pub skipped: usize,
    /// Parts whose parameters are not confirmed yet — either guessed from
    /// the footprint name or explicitly marked as needing review.
    #[serde(default)]
    pub unconfirmed_parts: Vec<String>,
    /// Components per hour the planner estimates this plan will achieve,
    /// derived from its cycle-time estimate. 0 when no estimate exists.
    #[serde(default)]
    pub estimated_cph: f64,
}

/// Turn the placement list into a planned job and store it.
///
/// This is the bridge between the editor (a grid of text) and the runner
/// (a full pick-and-place plan): parts are resolved from the library or
/// guessed from their footprint, the planner assigns feeders, nozzles and
/// the run order, and the whole thing is written to `job.json`.
#[server(prefix = "/api", endpoint = "plan_job")]
pub async fn plan_job_from_rows(
    board_id: String,
    columns: Vec<String>,
    rows: Vec<PlacementRow>,
) -> Result<PlanSummary, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        // A running job owns the machine and the placement order it is
        // executing. Re-optimising underneath it is never wanted — no
        // optimisation happens while the job runs in production — so every
        // plan request (explicit button or automatic re-opt from the
        // settings tabs) is refused while the executor is live.
        if state.job_executor.is_running() {
            return Err(server_error(
                "a job is running — no re-optimisation during a live run".to_string(),
            ));
        }
        let summary = crate::job_plan::plan_from_rows(
            &state.store,
            &machine_context(&state),
            board_id,
            columns,
            rows,
        )
        .map_err(|error| server_error(error))?;
        // Keep the library file being edited in sync with placement
        // edits, so it never lags behind job.json.
        if let Err(error) = crate::job_library::sync_loaded_job(&state.store) {
            eprintln!("syncing loaded job failed: {error}");
        }
        Ok(summary)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (board_id, columns, rows);
        unreachable!()
    }
}

/// Re-optimise the planned job without the editor grid.
///
/// Used when the operator goes to run the job and when the process view
/// opens: the planner re-runs on the stored placements, refreshing the
/// feeder assignment and run order against the current machine setup.
/// Refused while a run is live — a production run is never re-optimised.
#[server(prefix = "/api", endpoint = "reoptimize_job")]
pub async fn reoptimize_job() -> Result<PlanSummary, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        if state.job_executor.is_running() {
            return Err(server_error(
                "a job is running — no re-optimisation during a live run".to_string(),
            ));
        }
        let job = state.store.job();
        if job.placements.is_empty() && job.fiducials.is_empty() {
            return Err(server_error(
                "nothing to optimise: the job has no placements yet".to_string(),
            ));
        }
        let board_id = job.board_id.clone();
        let summary = crate::job_plan::plan_from_rows(
            &state.store,
            &machine_context(&state),
            board_id,
            job.placement_columns.clone(),
            crate::job_plan::rows_from_job(&job),
        )
        .map_err(server_error)?;
        if let Err(error) = crate::job_library::sync_loaded_job(&state.store) {
            eprintln!("syncing loaded job failed: {error}");
        }
        Ok(summary)
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "save_part")]
pub async fn save_part(
    id: String,
    mpn: String,
    description: String,
    footprint_name: String,
    nozzle_types: String,
    pickup_rotation: String,
    pickup_offset_x: String,
    pickup_offset_y: String,
    status: String,
    camera: String,
) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let parse_f64 = |v: &str, name: &str| {
            v.parse::<f64>()
                .map_err(|_| server_error(format!("invalid {name}: {v:?}")))
        };
        let mut id: u32 = id.parse().map_err(|_| server_error("invalid id"))?;
        // A zero id means "new part": pick the next free one instead of
        // storing every new part under the same id 0.
        if id == 0 {
            id = state
                .store
                .parts()
                .iter()
                .map(|part| part.part.id.0)
                .max()
                .unwrap_or(0)
                + 1;
        }
        let camera_kind = match camera.as_str() {
            "fast" => Some(pnp_logic::config::CameraKind::FastVision),
            "precise" => Some(pnp_logic::config::CameraKind::Precise),
            "head" => Some(pnp_logic::config::CameraKind::HeadCamera),
            _ => None,
        };
        let nozzle_types: Vec<pnp_logic::NozzleKind> = serde_json::from_str(&nozzle_types)
            .map_err(|e| server_error(format!("invalid nozzle_types: {e}")))?;
        let status = match status.as_str() {
            "Guessed" => pnp_logic::PartStatus::Guessed,
            "Confirmed" => pnp_logic::PartStatus::Confirmed,
            "NeedsReview" => pnp_logic::PartStatus::NeedsReview,
            _ => return Err(server_error(format!("unknown status: {status}"))),
        };
        let footprint_size = pnp_logic::derive_part_params(&footprint_name)
            .map(|p| p.footprint_size)
            .unwrap_or(pnp_logic::Size {
                width: 0.0,
                height: 0.0,
            });
        let part = pnp_logic::Part {
            id: pnp_logic::PartId(id),
            mpn,
            description,
            footprint: pnp_logic::Footprint {
                name: footprint_name.clone(),
                centroid: shared_types::Point::ORIGIN,
                size: footprint_size,
                pads: Vec::new(),
                polarity: pnp_logic::Polarity::None,
            },
            nozzle_type: nozzle_types,
            pickup_rotation_deg: parse_f64(&pickup_rotation, "pickup rotation")?,
            pickup_offset: shared_types::Point::new(
                parse_f64(&pickup_offset_x, "pickup offset x")?,
                parse_f64(&pickup_offset_y, "pickup offset y")?,
            ),
            camera: camera_kind,
        };
        let setup_part = pnp_logic::SetupPart { part, status };
        state
            .store
            .save_part(setup_part.clone())
            .map_err(|e| server_error(format!("Unable to save part: {e}")))?;

        // Placements embed a snapshot of the part they use. Refresh any
        // placement that references the saved part so the job reflects
        // the library (nozzle types, footprint, camera, ...), otherwise
        // library edits silently stop fixing blockers.
        let mut job = state.store.job();
        let mut refreshed = 0usize;
        for placement in &mut job.placements {
            if placement.part.id == setup_part.part.id {
                placement.part = setup_part.part.clone();
                refreshed += 1;
            }
        }
        if refreshed > 0 {
            state
                .store
                .save_job(job)
                .map_err(|e| server_error(format!("Unable to refresh job: {e}")))?;
        }

        Ok(if refreshed > 0 {
            format!("Part saved and {refreshed} placement(s) updated")
        } else {
            "Part saved".to_string()
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (id, mpn, description, footprint_name, nozzle_types, pickup_rotation, pickup_offset_x, pickup_offset_y, status, camera);
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "delete_part")]
pub async fn delete_part(id: String) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let id: u32 = id.parse().map_err(|_| server_error("invalid id"))?;
        // A part that placements still reference would be re-created by
        // the next re-optimisation (the planner re-derives library
        // entries from the placements), so refuse the deletion and say
        // which placements use it.
        let part = state
            .store
            .parts()
            .into_iter()
            .find(|entry| entry.part.id.0 == id)
            .map(|entry| entry.part);
        let Some(part) = part else {
            return Err(server_error(format!("part {id} is not in the library")));
        };
        // Deleting a part that placements still reference does not
        // stick — the planner re-derives library entries from the
        // placements — so remove those placements everywhere first.
        let removed = state.store.remove_placements_using(&part);
        let scrubbed = crate::job_library::scrub_part_from_saved_jobs(&part)
            .map_err(server_error)?;
        state
            .store
            .remove_part(pnp_logic::PartId(id))
            .map_err(|e| server_error(format!("Unable to delete part: {e}")))?;
        // Keep the library file being edited in sync with the removal.
        if let Err(error) = crate::job_library::sync_loaded_job(&state.store) {
            eprintln!("syncing loaded job failed: {error}");
        }
        let mut notes = vec!["Part deleted".to_string()];
        if !removed.is_empty() {
            notes.push(format!("removed placements {}", removed.join(", ")));
        }
        if !scrubbed.is_empty() {
            notes.push(format!("scrubbed saved jobs {}", scrubbed.join(", ")));
        }
        Ok(notes.join("; "))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = id;
        unreachable!()
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct JobRunStatus {
    pub running: bool,
    pub progress: String,
    pub done: bool,
    pub aborted: Option<String>,
    /// Message the runner needs the operator to act on, if any.
    #[serde(default)]
    pub prompt: Option<String>,
    /// What the machine is doing right now.
    #[serde(default)]
    pub action: String,
    #[serde(default)]
    pub placed_count: usize,
    #[serde(default)]
    pub placement_count: usize,
    /// Seconds since the run started, when a run is live.
    #[serde(default)]
    pub elapsed_secs: Option<f64>,
    /// Achieved components per hour while the run is live.
    #[serde(default)]
    pub live_cph: Option<f64>,
    /// Planner-estimated components per hour for the stored plan.
    #[serde(default)]
    pub estimated_cph: f64,
}

#[server(prefix = "/api", endpoint = "start_job")]
pub async fn start_job() -> Result<JobRunStatus, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;

        // Refuse to run a job the operator cannot place (no parts, no
        // nozzle, unreviewed part) for the same reason the UI does.
        let blockers = job_store::blockers(&state.store.job(), &state.store.parts());
        if !blockers.is_empty() {
            return Err(server_error(format!(
                "job cannot run yet: {}",
                blockers.join("; ")
            )));
        }
        // Two runs would fight over the machine, so a live run wins.
        if state.job_executor.is_running() {
            return Err(server_error("a job is already running"));
        }

        let job = state.store.job();
        // A new run starts from scratch: clear the `placed` flags the
        // previous run left behind so the progress counts mean
        // "placed in this run", not "ever placed".
        let mut job = job;
        job.placements
            .iter_mut()
            .for_each(|placement| placement.placed = false);
        // Nozzle offsets live in the calibration database, not in the
        // job. Stamp the DB's current offsets into the head so the
        // executor plans with the machine's real geometry — the job
        // only carries nozzle number and type.
        let offsets = state.machine.nozzle_offsets();
        for nozzle in &mut job.head.nozzles {
            if let Some(calibrated) = offsets
                .iter()
                .find(|offset| offset.nozzle.id() == nozzle.id)
            {
                nozzle.offset_mm = calibrated.offset_mm;
            }
        }
        if !job.placements.is_empty() {
            state
                .store
                .save_job(job.clone())
                .map_err(|e| server_error(format!("Unable to reset job: {e}")))?;
        }

        state
            .job_executor
            .start(job, machine_context(&state))
            .map_err(|e| server_error(format!("Unable to start job: {e}")))?;

        // The executor only owns the state machine; this task turns its
        // steps into machine moves and operator prompts.
        let runner_state = state.clone();
        tokio::spawn(async move { crate::job_runner::run_job(runner_state).await });

        Ok(job_runner::status(&state))
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "stop_job")]
pub async fn stop_job() -> Result<JobRunStatus, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        state.job_executor.stop();
        // Clear any prompt so the run loop does not wait on the operator
        // for an answer that will never come.
        let _ = state.job_runner.answer();
        Ok(job_runner::status(&state))
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

/// Answer whatever the running job is asking the operator.
///
/// Used for every prompt: "load the board", each fiducial, and any future
/// question the runner wants answered before it moves on.
#[server(prefix = "/api", endpoint = "answer_prompt")]
pub async fn answer_prompt() -> Result<JobRunStatus, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        if !state.job_runner.answer() {
            return Err(server_error("the job is not waiting for an answer"));
        }
        Ok(job_runner::status(&state))
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "job_status")]
pub async fn job_status() -> Result<JobRunStatus, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        Ok(job_runner::status(&state))
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

// ---------------------------------------------------------------------------
// Per-footprint heights and the process view
// ---------------------------------------------------------------------------

/// One recorded footprint: motion heights and body size.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct HeightEntry {
    pub footprint: String,
    pub pickup_z_mm: f64,
    pub place_z_mm: f64,
    pub width_mm: f64,
    pub height_mm: f64,
}

/// One step of the machine's cycle, in execution order.
///
/// `kind` is `"fiducial"`, `"pick"`, `"place"` or `"inspect"`. The cycle is
/// organised into head-loads ("trips", numbered from 1 in `trip`): every
/// pick of that trip at the feeder rail, then one camera stop to inspect
/// the carried parts (`inspect`), then that trip's placements. `trip` is 0
/// for the fiducial phase, which runs before any parts are carried.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StepInfo {
    pub index: usize,
    pub kind: String,
    pub refdes: String,
    pub mpn: String,
    pub footprint: String,
    pub feeder_slot: u8,
    pub nozzle: u8,
    /// Head-load (trip) this step belongs to, 1-based; 0 = fiducial phase.
    pub trip: u32,
    pub x: f64,
    pub y: f64,
    pub rotation: f64,
    /// Z the head travels to for this step (mm).
    pub z_mm: f64,
    /// Already completed by a previous (or the current) run.
    pub done: bool,
    /// The operator disabled this placement.
    pub skipped: bool,
}

/// A feeder slot position, for drawing the feeder rail in the process view.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FeederInfo {
    pub slot: u8,
    pub x: f64,
    pub y: f64,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct StepsResponse {
    pub board_id: String,
    pub steps: Vec<StepInfo>,
    pub feeders: Vec<FeederInfo>,
    /// Index of the first not-yet-done step, if any.
    pub live_index: Option<usize>,
    /// Planner-estimated components per hour for the stored plan.
    #[serde(default)]
    pub estimated_cph: f64,
    /// Components per hour achieved so far while a run is live.
    #[serde(default)]
    pub live_cph: Option<f64>,
    /// Seconds since the run started, when a run is live.
    #[serde(default)]
    pub elapsed_secs: Option<f64>,
}

/// Every recorded footprint height, for the part / footprint editors.
#[server(prefix = "/api", endpoint = "get_part_heights")]
pub async fn get_part_heights() -> Result<Vec<HeightEntry>, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let mut entries: Vec<HeightEntry> = state
            .heights
            .map()
            .into_iter()
            .map(|(footprint, h)| HeightEntry {
                footprint,
                pickup_z_mm: h.pickup_z_mm,
                place_z_mm: h.place_z_mm,
                width_mm: h.width_mm,
                height_mm: h.height_mm,
            })
            .collect();
        entries.sort_by(|a, b| a.footprint.cmp(&b.footprint));
        Ok(entries)
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

/// Record (or update) the motion parameters of one footprint.
#[server(prefix = "/api", endpoint = "save_part_height")]
pub async fn save_part_height(
    footprint: String,
    pickup_z_mm: String,
    place_z_mm: String,
    width_mm: String,
    height_mm: String,
) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let parse = |v: &str, name: &str| {
            v.trim()
                .parse::<f64>()
                .map_err(|_| server_error(format!("invalid {name}: {v:?}")))
        };
        let entry = crate::part_heights::FootprintHeights {
            pickup_z_mm: parse(&pickup_z_mm, "pickup z")?,
            place_z_mm: parse(&place_z_mm, "place z")?,
            width_mm: parse(&width_mm, "body width")?,
            height_mm: parse(&height_mm, "body height")?,
        };
        state
            .heights
            .save_entry(&footprint, entry)
            .map_err(|e| server_error(format!("Unable to save heights: {e}")))?;
        Ok(format!("Heights saved for {footprint}"))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (footprint, pickup_z_mm, place_z_mm, width_mm, height_mm);
        unreachable!()
    }
}

/// The full machine cycle in execution order: locate fiducials, then
/// every head-load ("trip") the machine performs — all picks of that trip
/// at the feeder rail, one camera stop to inspect the carried parts, then
/// that trip's placements. The trip grouping mirrors the runner's batch
/// rule exactly (same camera kind, same strategy, distinct nozzles, within
/// one head-load), so the view shows the real bulk-picking cycle.
///
/// The process view steps through this list in either direction; the
/// live index tracks what the machine is actually doing right now.
/// The working job exported for editing: the board id plus the
/// placement list rebuilt as editable rows.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct PlacementExport {
    pub board_id: String,
    pub rows: Vec<PlacementRow>,
    /// Column labels saved with the job, so the editor restores the
    /// exact layout the operator imported instead of a default grid.
    #[serde(default)]
    pub columns: Vec<String>,
}

/// Export the working job's placements as placement-list rows so the
/// PCB placement list can be populated from a loaded job and edited.
#[server(prefix = "/api", endpoint = "get_placement_rows")]
pub async fn get_placement_rows() -> Result<PlacementExport, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        Ok(PlacementExport {
            board_id: job.board_id.clone(),
            rows: crate::job_plan::editor_rows_from_job(&job),
            columns: job.placement_columns.clone(),
        })
    }
    #[cfg(not(feature = "ssr"))]
    unreachable!()
}

/// Clear the working job's placement list (the rows and fiducials, plus
/// the feeder assignment derived from them). Used to wipe temp or test
/// data from the system without touching the machine layout. The loaded
/// library job is unlinked so a later machine-setup save cannot write
/// an emptied job back over it.
#[server(prefix = "/api", endpoint = "clear_placements")]
pub async fn clear_placements() -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        state
            .store
            .clear_placements()
            .map(|_| "Placement list cleared".to_string())
            .map_err(server_error)
    }
    #[cfg(not(feature = "ssr"))]
    unreachable!()
}

/// Station position for a feeder slot. Never returns NaN: serde_json
/// renders NaN/Infinity as JSON `null`, which the browser cannot decode
/// back into `f64` ("invalid type: null, expected f64").
#[cfg(feature = "ssr")]
fn feeder_station(feeders: &[FeederSlot], slot: u8) -> (f64, f64) {
    feeders
        .iter()
        .find(|feeder| feeder.id == slot)
        .or_else(|| feeders.first())
        .map(|feeder| (feeder.position.x, feeder.position.y))
        .unwrap_or((0.0, 0.0))
}

/// Station position for a camera of the given kind. Falls back to the
/// first camera in the job, then to the origin — e.g. a job whose
/// placements still reference a `FastVision` camera that the saved
/// camera list no longer contains must still render.
#[cfg(feature = "ssr")]
fn camera_station(cameras: &[Camera], kind: CameraKind) -> (f64, f64) {
    cameras
        .iter()
        .find(|camera| camera.kind == kind)
        .or_else(|| cameras.first())
        .map(|camera| (camera.position.x, camera.position.y))
        .unwrap_or((0.0, 0.0))
}

/// Snapshot of the machine-side settings (cameras, machine config,
/// feeder setup, feeder calibration). These live in the settings
/// database, not in the job file; runners and the planner get them from
/// here.
#[cfg(feature = "ssr")]
fn machine_context(state: &AppState) -> pnp_logic::MachineContext {
    state
        .machine_context
        .read()
        .expect("machine context lock poisoned")
        .clone()
}

#[server(prefix = "/api", endpoint = "get_job_steps")]
pub async fn get_job_steps() -> Result<StepsResponse, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let machine = machine_context(&state);
        let heights = state.heights.map();
        let (default_pickup_z, default_place_z) =
            (machine.config.pickup_z_mm, machine.config.place_z_mm);

        let feeder_pos = |slot: u8| feeder_station(&job.feeders, slot);

        let mut steps: Vec<StepInfo> = Vec::new();
        for (index, fiducial) in job.fiducials.iter().enumerate() {
            steps.push(StepInfo {
                index: steps.len(),
                kind: "fiducial".to_string(),
                refdes: format!("FID-{}", index + 1),
                mpn: String::new(),
                footprint: String::new(),
                feeder_slot: 0,
                nozzle: 0,
                trip: 0,
                x: fiducial.design_position.x,
                y: fiducial.design_position.y,
                rotation: 0.0,
                z_mm: 0.0,
                done: fiducial.machine_position.is_some(),
                skipped: false,
            });
        }

        // Group placements into head-loads ("trips") exactly the way the
        // runner batches them (pnp_logic::trip_ids mirrors start_batch).
        let trip_of = pnp_logic::trip_ids(job.head.nozzles.len(), &job.placements);
        let mut trips: Vec<Vec<usize>> = vec![Vec::new(); trip_of.iter().copied().max().unwrap_or(0) + 1];
        for (index, &trip) in trip_of.iter().enumerate() {
            trips[trip].push(index);
        }
        let placements_of = |index: &usize| &job.placements[*index];
        for (trip_number, members) in trips.iter().enumerate() {
            if members.is_empty() {
                continue;
            }
            let trip = (trip_number + 1) as u32;
            let build = |steps: &mut Vec<StepInfo>,
                             kind: &str,
                             refdes: String,
                             x: f64,
                             y: f64,
                             rotation: f64,
                             z_mm: f64,
                             placement: &pnp_logic::Placement| {
                steps.push(StepInfo {
                    index: steps.len(),
                    kind: kind.to_string(),
                    refdes,
                    mpn: placement.part.mpn.clone(),
                    footprint: placement.part.footprint.name.clone(),
                    feeder_slot: placement.feeder_slot,
                    nozzle: placement.nozzle,
                    trip,
                    x,
                    y,
                    rotation,
                    z_mm,
                    done: placement.placed,
                    skipped: !placement.should_place,
                });
            };
            // Pickup leg: every nozzle of this trip dips into its feeder.
            for index in members {
                let placement = placements_of(index);
                let recorded = heights.get(&placement.part.footprint.name);
                let pickup_z = recorded
                    .map(|h| h.pickup_z_mm)
                    .filter(|z| *z != 0.0)
                    .unwrap_or(default_pickup_z);
                let (feeder_x, feeder_y) = feeder_pos(placement.feeder_slot);
                build(
                    &mut steps,
                    "pick",
                    String::new(),
                    feeder_x,
                    feeder_y,
                    placement.part.pickup_rotation_deg,
                    pickup_z,
                    placement,
                );
            }
            // One camera stop per trip: the freshly picked parts are
            // inspected (and corrected) together.
            let first = placements_of(&members[0]);
            let (camera_x, camera_y) = camera_station(&machine.cameras, first.camera_kind);
            steps.push(StepInfo {
                index: steps.len(),
                kind: "inspect".to_string(),
                refdes: format!("Inspect ×{}", members.len()),
                mpn: String::new(),
                footprint: String::new(),
                feeder_slot: 0,
                nozzle: 0,
                trip,
                x: camera_x,
                y: camera_y,
                rotation: 0.0,
                z_mm: 0.0,
                done: members.iter().all(|&index| job.placements[index].placed),
                skipped: false,
            });
            // Placement leg: the trip's parts go onto the board in order.
            for index in members {
                let placement = placements_of(index);
                let recorded = heights.get(&placement.part.footprint.name);
                let place_z = recorded
                    .map(|h| h.place_z_mm)
                    .filter(|z| *z != 0.0)
                    .unwrap_or(default_place_z);
                build(
                    &mut steps,
                    "place",
                    format!("{}-{}", placement.part.mpn, placement.part.id.0),
                    placement.design_position.x,
                    placement.design_position.y,
                    placement.design_rotation_deg,
                    place_z,
                    placement,
                );
            }
        }
        let live_index = steps.iter().position(|step| !step.done && !step.skipped);
        let feeders = job
            .feeders
            .iter()
            .map(|feeder| FeederInfo {
                slot: feeder.id,
                x: feeder.position.x,
                y: feeder.position.y,
            })
            .collect();
        Ok(StepsResponse {
            board_id: job.board_id.clone(),
            steps,
            feeders,
            live_index,
            estimated_cph: crate::job_runner::status(&state).estimated_cph,
            live_cph: crate::job_runner::status(&state).live_cph,
            elapsed_secs: crate::job_runner::status(&state).elapsed_secs,
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

// ---------------------------------------------------------------------------
// Machine setup + fiducial workflow
// ---------------------------------------------------------------------------

/// One feeder rail as edited on the machine setup page.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FeederEntry {
    pub id: u8,
    pub bank_id: u8,
    pub x: f64,
    pub y: f64,
    /// "tape8".."tape44", "stick", "tray", "bulk", "tube", "vibratory"
    pub kind: String,
    /// MPN of the loaded part, empty = empty feeder.
    pub loaded: String,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BankEntry {
    pub id: u8,
    pub rotation_deg: f64,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct CameraEntry {
    pub id: u8,
    /// "fast" | "precise" | "head"
    pub kind: String,
    pub x: f64,
    pub y: f64,
    /// 0 = not an array camera.
    pub pitch: f64,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NozzleEntry {
    pub id: u8,
    /// "standard" | "fine" | "extra_fine" | "n501".."n503" | "custom:<d>"
    pub kind: String,
    pub offset_mm: f64,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct ConfigEntry {
    pub camera_x: f64,
    pub camera_y: f64,
    pub travel_z: f64,
    pub pickup_z: f64,
    pub place_z: f64,
    pub xy_speed: u8,
    pub z_speed: u8,
    pub rotation_speed: u8,
    pub xy_approach_speed: u8,
    pub z_approach_speed: u8,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct MachineSetupData {
    pub feeders: Vec<FeederEntry>,
    pub banks: Vec<BankEntry>,
    pub cameras: Vec<CameraEntry>,
    pub nozzles: Vec<NozzleEntry>,
    pub config: ConfigEntry,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct FiducialEntry {
    pub index: usize,
    pub design_x: f64,
    pub design_y: f64,
    /// "panel" | "local"
    pub kind: String,
    pub expected: Option<(f64, f64)>,
    pub machine: Option<(f64, f64)>,
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct BoardMappingData {
    pub origin_x: f64,
    pub origin_y: f64,
    pub rotation_deg: f64,
    pub scale: f64,
    /// Non-empty when the mapping looks suspicious (scale far from 1).
    pub warning: String,
}

#[cfg(feature = "ssr")]
fn parse_feeder_kind(kind: &str) -> Result<pnp_logic::FeederKind, ServerFnError> {
    let tape = kind
        .strip_prefix("tape")
        .and_then(|width| width.parse::<u8>().ok());
    if let Some(width) = tape {
        return Ok(pnp_logic::FeederKind::Tape(width));
    }
    let kind = match kind {
        "stick" => pnp_logic::FeederKind::Stick,
        "tray" => pnp_logic::FeederKind::Tray,
        "bulk" => pnp_logic::FeederKind::Bulk,
        "tube" => pnp_logic::FeederKind::Tube,
        "vibratory" => pnp_logic::FeederKind::Vibratory,
        other => return Err(server_error(format!("unknown feeder kind: {other}"))),
    };
    Ok(kind)
}

#[cfg(feature = "ssr")]
fn format_feeder_kind(kind: &pnp_logic::FeederKind) -> String {
    match kind {
        pnp_logic::FeederKind::Tape(width) => format!("tape{width}"),
        pnp_logic::FeederKind::Stick => "stick".to_string(),
        pnp_logic::FeederKind::Tray => "tray".to_string(),
        pnp_logic::FeederKind::Bulk => "bulk".to_string(),
        pnp_logic::FeederKind::Tube => "tube".to_string(),
        pnp_logic::FeederKind::Vibratory => "vibratory".to_string(),
    }
}

#[cfg(feature = "ssr")]
fn parse_nozzle_kind(kind: &str) -> Result<pnp_logic::NozzleKind, ServerFnError> {
    let kind = match kind {
        "standard" => pnp_logic::NozzleKind::Standard,
        "fine" => pnp_logic::NozzleKind::Fine,
        "extra_fine" => pnp_logic::NozzleKind::ExtraFine,
        "n501" => pnp_logic::NozzleKind::N501,
        "n502" => pnp_logic::NozzleKind::N502,
        "n503" => pnp_logic::NozzleKind::N503,
        "n504" => pnp_logic::NozzleKind::N504,
        "n505" => pnp_logic::NozzleKind::N505,
        "n506" => pnp_logic::NozzleKind::N506,
        "n507" => pnp_logic::NozzleKind::N507,
        "n508" => pnp_logic::NozzleKind::N508,
        other => {
            let diameter = other
                .strip_prefix("custom:")
                .and_then(|d| d.parse::<u16>().ok())
                .ok_or_else(|| server_error(format!("unknown nozzle kind: {other}")))?;
            pnp_logic::NozzleKind::Custom(diameter)
        }
    };
    Ok(kind)
}

#[cfg(feature = "ssr")]
fn format_nozzle_kind(kind: &pnp_logic::NozzleKind) -> String {
    match kind {
        pnp_logic::NozzleKind::Standard => "standard".to_string(),
        pnp_logic::NozzleKind::Fine => "fine".to_string(),
        pnp_logic::NozzleKind::ExtraFine => "extra_fine".to_string(),
        pnp_logic::NozzleKind::N501 => "n501".to_string(),
        pnp_logic::NozzleKind::N502 => "n502".to_string(),
        pnp_logic::NozzleKind::N503 => "n503".to_string(),
        pnp_logic::NozzleKind::N504 => "n504".to_string(),
        pnp_logic::NozzleKind::N505 => "n505".to_string(),
        pnp_logic::NozzleKind::N506 => "n506".to_string(),
        pnp_logic::NozzleKind::N507 => "n507".to_string(),
        pnp_logic::NozzleKind::N508 => "n508".to_string(),
        pnp_logic::NozzleKind::Custom(d) => format!("custom:{d}"),
    }
}

#[server(prefix = "/api", endpoint = "get_machine_setup")]
pub async fn get_machine_setup() -> Result<MachineSetupData, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let parts = state.store.parts();
        let mpn_for_id = |id: u32| {
            parts
                .iter()
                .find(|entry| entry.part.id.0 == id)
                .map(|entry| entry.part.mpn.clone())
                .unwrap_or_default()
        };
        // Cameras, machine config and the feeder-kind setup are machine
        // settings now — they come from the settings database, not from
        // the job file.
        let machine = machine_context(&state);
        Ok(MachineSetupData {
            feeders: job
                .feeders
                .iter()
                .map(|feeder| FeederEntry {
                    id: feeder.id,
                    bank_id: feeder.bank_id,
                    x: feeder.position.x,
                    y: feeder.position.y,
                    kind: machine
                        .setup
                        .kind(feeder.id)
                        .map(|kind| format_feeder_kind(&kind))
                        .unwrap_or_else(|| "tape16".to_string()),
                    loaded: feeder
                        .loaded
                        .map(|part_id| mpn_for_id(part_id.0))
                        .unwrap_or_default(),
                })
                .collect(),
            banks: job
                .banks
                .iter()
                .map(|bank| BankEntry {
                    id: bank.id,
                    rotation_deg: bank.rotation_deg,
                })
                .collect(),
            cameras: machine
                .cameras
                .iter()
                .map(|camera| CameraEntry {
                    id: camera.id,
                    kind: match camera.kind {
                        pnp_logic::config::CameraKind::FastVision => "fast".to_string(),
                        pnp_logic::config::CameraKind::Precise => "precise".to_string(),
                        pnp_logic::config::CameraKind::HeadCamera => "head".to_string(),
                    },
                    x: camera.position.x,
                    y: camera.position.y,
                    pitch: camera.array_pitch_mm.unwrap_or(0.0),
                })
                .collect(),
            nozzles: {
                // Offsets are owned by the calibration database; the job
                // only carries nozzle number and type. Show the DB value.
                let offsets = state.machine.nozzle_offsets();
                job.head
                    .nozzles
                    .iter()
                    .map(|nozzle| NozzleEntry {
                        id: nozzle.id,
                        kind: format_nozzle_kind(&nozzle.kind),
                        offset_mm: offsets
                            .iter()
                            .find(|calibrated| calibrated.nozzle.id() == nozzle.id)
                            .map(|calibrated| calibrated.offset_mm)
                            .unwrap_or(nozzle.offset_mm),
                    })
                    .collect()
            },
            config: ConfigEntry {
                camera_x: machine.config.camera_position.x,
                camera_y: machine.config.camera_position.y,
                travel_z: machine.config.travel_z_mm,
                pickup_z: machine.config.pickup_z_mm,
                place_z: machine.config.place_z_mm,
                xy_speed: machine.config.xy_speed,
                z_speed: machine.config.z_speed,
                rotation_speed: machine.config.rotation_speed,
                xy_approach_speed: machine.config.xy_approach_speed,
                z_approach_speed: machine.config.z_approach_speed,
            },
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "save_machine_setup")]
pub async fn save_machine_setup(setup: MachineSetupData) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let parse_camera = |kind: &str| match kind {
            "fast" => Ok(pnp_logic::config::CameraKind::FastVision),
            "precise" => Ok(pnp_logic::config::CameraKind::Precise),
            "head" => Ok(pnp_logic::config::CameraKind::HeadCamera),
            other => Err(server_error(format!("unknown camera kind: {other}"))),
        };
        let parts = state.store.parts();

        let mut job = state.store.job();

        job.feeders = setup
            .feeders
            .iter()
            .map(|entry| {
                let loaded = if entry.loaded.trim().is_empty() {
                    None
                } else {
                    parts
                        .iter()
                        .find(|part| part.part.mpn == entry.loaded.trim())
                        .map(|part| part.part.id)
                };
                pnp_logic::FeederSlot {
                    id: entry.id,
                    bank_id: entry.bank_id,
                    position: shared_types::Point::new(entry.x, entry.y),
                    loaded,
                }
            })
            .collect();
        job.banks = setup
            .banks
            .iter()
            .map(|bank| pnp_logic::FeederBank {
                id: bank.id,
                rotation_deg: bank.rotation_deg,
            })
            .collect();
        let mut cameras = Vec::with_capacity(setup.cameras.len());
        for camera in &setup.cameras {
            cameras.push(pnp_logic::Camera {
                id: camera.id,
                kind: parse_camera(&camera.kind)?,
                position: shared_types::Point::new(camera.x, camera.y),
                array_pitch_mm: if camera.pitch > 0.0 {
                    Some(camera.pitch)
                } else {
                    None
                },
            });
        }
        // Cameras, machine config and the feeder-kind setup are machine
        // settings now: they go to the settings database, never into the
        // job file. Feeder calibration entries are not edited in this
        // form, so the existing ones are carried over untouched.
        let mut machine = machine_context(&state);
        machine.cameras = cameras;
        // The job records nozzle number and type; offsets stay in the
        // calibration database. Stamp the DB's current offsets in so the
        // job stays runnable, but never store an operator-typed offset.
        let db_offsets = state.machine.nozzle_offsets();
        let mut nozzles = Vec::with_capacity(setup.nozzles.len());
        for nozzle in &setup.nozzles {
            nozzles.push(pnp_logic::NozzleSpec {
                id: nozzle.id,
                kind: parse_nozzle_kind(&nozzle.kind)?,
                offset_mm: db_offsets
                    .iter()
                    .find(|calibrated| calibrated.nozzle.id() == nozzle.id)
                    .map(|calibrated| calibrated.offset_mm)
                    .unwrap_or(nozzle.offset_mm),
            });
        }
        job.head = pnp_logic::HeadSpec { nozzles };
        machine.config = pnp_logic::MachineConfig {
            camera_position: shared_types::Point::new(
                setup.config.camera_x,
                setup.config.camera_y,
            ),
            travel_z_mm: setup.config.travel_z,
            pickup_z_mm: setup.config.pickup_z,
            place_z_mm: setup.config.place_z,
            xy_speed: setup.config.xy_speed,
            z_speed: setup.config.z_speed,
            rotation_speed: setup.config.rotation_speed,
            xy_approach_speed: setup.config.xy_approach_speed,
            z_approach_speed: setup.config.z_approach_speed,
        };
        let mut setup_new = pnp_logic::MachineSetup::default();
        for entry in &setup.feeders {
            setup_new.set_kind(entry.id, parse_feeder_kind(&entry.kind)?);
            if let Some(part) = parts
                .iter()
                .find(|part| part.part.mpn == entry.loaded.trim())
            {
                setup_new.set_loaded(entry.id, part.part.id);
            }
        }
        machine.setup = setup_new;
        let connection = Connection::open(crate::calibration_store::DB_PATH)
            .map_err(|e| server_error(format!("unable to open the settings database: {e}")))?;
        crate::calibration_store::save_machine_context(&connection, &machine)
            .map_err(|e| server_error(format!("unable to save machine settings: {e}")))?;
        *state
            .machine_context
            .write()
            .expect("machine context lock poisoned") = machine;

        state
            .store
            .save_job(job)
            .map_err(|e| server_error(format!("Unable to save machine setup: {e}")))?;
        // Self-verifying feedback: the operator can check at a glance
        // that what they edited is what the server stored.
        let nozzle_summary = setup
            .nozzles
            .iter()
            .map(|nozzle| format!("{}={}", nozzle.id, nozzle.kind))
            .collect::<Vec<_>>()
            .join(" ");
        // The operator may be editing a named library job (loaded from
        // `jobs/`); keep its nozzle setup in sync with what was just
        // saved. A failure to write back is a warning, not a lost save.
        let mut message = format!("Machine setup saved — nozzles: {nozzle_summary}");
        match crate::job_library::sync_loaded_nozzle_kinds(&state.store) {
            Ok(Some(path)) => message.push_str(&format!(" — wrote nozzle kinds to {path}")),
            Ok(None) => {}
            Err(error) => message.push_str(&format!(" (warning: {error})")),
        }
        Ok(message)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = setup;
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "get_fiducials")]
pub async fn get_fiducials() -> Result<Vec<FiducialEntry>, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        Ok(job
            .fiducials
            .iter()
            .enumerate()
            .map(|(index, fiducial)| FiducialEntry {
                index,
                design_x: fiducial.design_position.x,
                design_y: fiducial.design_position.y,
                kind: match fiducial.kind {
                    pnp_logic::FiducialKind::Panel => "panel".to_string(),
                    pnp_logic::FiducialKind::Local => "local".to_string(),
                },
                expected: fiducial
                    .expected_position
                    .map(|point| (point.x, point.y)),
                machine: fiducial
                    .machine_position
                    .map(|point| (point.x, point.y)),
            })
            .collect())
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

/// Save the machine's *current* position as the rough spot where the
/// camera expects fiducial `index` to be. Operator jogs the head over
/// the mark first, then presses the button.
#[server(prefix = "/api", endpoint = "set_fiducial_expected")]
pub async fn set_fiducial_expected(index: u32) -> Result<(f64, f64), ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let position = state
            .machine
            .cached_position()
            .ok_or_else(|| server_error("machine has not reported a position yet"))?;
        let point = shared_types::Point::new(position.x, position.y);
        let mut job = state.store.job();
        let Some(fiducial) = job.fiducials.get_mut(index as usize) else {
            return Err(server_error(format!("no fiducial #{index}")));
        };
        fiducial.expected_position = Some(point);
        state
            .store
            .save_job(job)
            .map_err(|e| server_error(format!("Unable to save job: {e}")))?;
        Ok((point.x, point.y))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = index;
        unreachable!()
    }
}

/// Move the camera to the saved rough position, run the fiducial finder
/// and store the refined position as this fiducial's machine position.
#[server(prefix = "/api", endpoint = "locate_fiducial")]
pub async fn locate_fiducial(index: u32) -> Result<(f64, f64), ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let Some(fiducial) = job.fiducials.get(index as usize) else {
            return Err(server_error(format!("no fiducial #{index}")));
        };
        let Some(expected) = fiducial.expected_position else {
            return Err(server_error(
                "no expected position saved — jog the camera over the fiducial and save it first",
            ));
        };
        let machine = state.machine.clone();
        let found = tokio::task::spawn_blocking(move || -> Result<(f64, f64), String> {
            machine
                .move_xy(shared_types::Point::new(expected.x, expected.y), 3)
                .map_err(|e| format!("move failed: {e}"))?;
            if !machine.center_camera_on_feducial(8) {
                return Err("fiducial finder could not lock on to the mark".to_string());
            }
            let position = machine
                .cached_position()
                .ok_or_else(|| "machine did not report a position".to_string())?;
            Ok((position.x, position.y))
        })
        .await
        .map_err(|e| server_error(format!("fiducial run failed: {e}")))?
        .map_err(server_error)?;

        let point = shared_types::Point::new(found.0, found.1);
        let mut job = state.store.job();
        if let Some(fiducial) = job.fiducials.get_mut(index as usize) {
            fiducial.machine_position = Some(point);
        }
        state
            .store
            .save_job(job)
            .map_err(|e| server_error(format!("Unable to save job: {e}")))?;
        Ok(found)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = index;
        unreachable!()
    }
}

/// Recover the board mapping (origin + rotation + scale) from the
/// located panel fiducials and store it in the job.
#[server(prefix = "/api", endpoint = "map_board")]
pub async fn map_board() -> Result<BoardMappingData, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let job = state.store.job();
        let pairs: Vec<(shared_types::Point, shared_types::Point)> = job
            .fiducials
            .iter()
            .filter(|fiducial| fiducial.kind == pnp_logic::FiducialKind::Panel)
            .filter_map(|fiducial| {
                fiducial
                    .machine_position
                    .map(|machine| (fiducial.design_position, machine))
            })
            .collect();
        if pairs.len() < 2 {
            return Err(server_error(
                "need at least two located panel fiducials before mapping",
            ));
        }
        let (d1, m1) = pairs[0];
        let (d2, m2) = pairs[1];
        let (dx, dy) = (d2.x - d1.x, d2.y - d1.y);
        let (mx, my) = (m2.x - m1.x, m2.y - m1.y);
        let design_length = (dx * dx + dy * dy).sqrt();
        if design_length < 1e-9 {
            return Err(server_error(
                "the two fiducials sit on top of each other in the design — pick two spaced-apart fiducials",
            ));
        }
        let cross = dx * my - dy * mx;
        let dot = dx * mx + dy * my;
        let rotation_deg = cross.atan2(dot).to_degrees();
        let machine_length = (mx * mx + my * my).sqrt();
        let scale = machine_length / design_length;

        let rad = rotation_deg.to_radians();
        let (sin, cos) = rad.sin_cos();
        let origin_x = m1.x - (d1.x * cos - d1.y * sin) * scale;
        let origin_y = m1.y - (d1.x * sin + d1.y * cos) * scale;

        let warning = if (scale - 1.0).abs() > 0.02 {
            format!(
                "scale is {scale:.3} — fiducial positions are probably inconsistent, re-run the finder"
            )
        } else {
            String::new()
        };

        let mapping = pnp_logic::BoardMapping {
            origin_x,
            origin_y,
            rotation_deg,
            scale,
        };
        let mut job = state.store.job();
        job.board_mapping = Some(mapping.clone());
        state
            .store
            .save_job(job)
            .map_err(|e| server_error(format!("Unable to save job: {e}")))?;

        Ok(BoardMappingData {
            origin_x,
            origin_y,
            rotation_deg,
            scale,
            warning,
        })
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

/// The nozzle offsets currently stored in the calibration database.
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct NozzleOffsetEntry {
    /// 1-based label matching the operator-facing nozzle numbering.
    pub label: String,
    pub offset_mm: f64,
    pub runout_mm: f64,
}

#[server(prefix = "/api", endpoint = "get_nozzle_offsets")]
pub async fn get_nozzle_offsets() -> Result<Vec<NozzleOffsetEntry>, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let connection = rusqlite::Connection::open(calibration_store::DB_PATH)
            .map_err(|e| server_error(format!("unable to open {}: {e}", calibration_store::DB_PATH)))?;
        calibration_store::ensure_schema(&connection)
            .map_err(|e| server_error(format!("unable to ensure schema: {e}")))?;
        let stored = calibration_store::load_nozzle_offsets(&connection)
            .map_err(|e| server_error(format!("unable to load nozzle calibration: {e}")))?
            .unwrap_or_default();
        Ok(stored
            .iter()
            .map(|offset| NozzleOffsetEntry {
                label: nozzle_label(offset.nozzle).to_string(),
                offset_mm: offset.offset_mm,
                runout_mm: offset.runout_mm,
            })
            .collect())
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

// ---------------------------------------------------------------------------
// Guided pick & place height calibration
// ---------------------------------------------------------------------------

/// Move the head so the selected nozzle sits roughly over a fixed
/// camera station. The operator fine-jogs from there; the nozzle's own
/// XY offset is not applied yet, which is fine because this workflow
/// ends up recording heights relative to whatever the operator centres.
#[server(prefix = "/api", endpoint = "move_nozzle_to_camera")]
pub async fn move_nozzle_to_camera(camera: String) -> Result<(f64, f64), ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        if camera == "head" {
            return Err(server_error(
                "the head camera travels with the head — jog the nozzle over the mark directly",
            ));
        }
        let (kind, index) = if camera == "precise" {
            (pnp_logic::config::CameraKind::Precise, 0usize)
        } else if let Some(rest) = camera.strip_prefix("fast-") {
            let index: usize = rest
                .parse()
                .map_err(|_| server_error(format!("unknown camera: {camera}")))?;
            (pnp_logic::config::CameraKind::FastVision, index.saturating_sub(1))
        } else {
            return Err(server_error(format!("unknown camera: {camera}")));
        };
        let target = *machine_context(&state)
            .cameras
            .iter()
            .filter(|camera| camera.kind == kind)
            .nth(index)
            .map(|camera| &camera.position)
            .ok_or_else(|| {
                server_error(format!(
                    "no camera of that kind is defined in the machine setup"
                ))
            })?;
        let machine = state.machine.clone();
        tokio::task::spawn_blocking(move || {
            machine
                .move_xy(target, 4)
                .map_err(|e| format!("move failed: {e}"))
        })
        .await
        .map_err(|e| server_error(e.to_string()))?
        .map_err(server_error)?;
        Ok((target.x, target.y))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = camera;
        unreachable!()
    }
}

/// Small relative Z jog for one nozzle (e.g. creeping down onto paper).
#[server(prefix = "/api", endpoint = "cal_z_jog")]
pub async fn cal_z_jog(nozzle: u8, delta_mm: String) -> Result<f64, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let nozzle_id = shared_types::NozzleId::from_id(nozzle)
            .ok_or_else(|| server_error(format!("unknown nozzle {nozzle}")))?;
        let delta: f64 = delta_mm
            .trim()
            .replace(',', ".")
            .parse()
            .map_err(|_| server_error(format!("invalid delta: {delta_mm:?}")))?;
        let machine = state.machine.clone();
        let new_z = tokio::task::spawn_blocking(move || -> Result<f64, String> {
            let position = machine
                .cached_position()
                .ok_or_else(|| "machine has not reported a position".to_string())?;
            let index = (nozzle_id.id() - 1) as usize;
            let target = position.z[index] + delta;
            machine
                .move_z(nozzle_id, target, 1)
                .map_err(|e| format!("Z move failed: {e}"))?;
            Ok(target)
        })
        .await
        .map_err(|e| server_error(e.to_string()))?
        .map_err(server_error)?;
        Ok(new_z)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (nozzle, delta_mm);
        unreachable!()
    }
}

/// Manual vacuum control for the calibration workflow: "on" opens
/// suction on the nozzle, "off" closes it.
#[server(prefix = "/api", endpoint = "cal_vacuum")]
pub async fn cal_vacuum(nozzle: u8, action: String) -> Result<(), ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let nozzle_id = shared_types::NozzleId::from_id(nozzle)
            .ok_or_else(|| server_error(format!("unknown nozzle {nozzle}")))?;
        let machine = state.machine.clone();
        tokio::task::spawn_blocking(move || -> Result<(), String> {
            match action.as_str() {
                "on" => machine
                    .open_vacuum(nozzle_id)
                    .map_err(|e| format!("vacuum on failed: {e}")),
                "off" => machine
                    .close_vacuum(nozzle_id)
                    .map_err(|e| format!("vacuum off failed: {e}")),
                other => Err(format!("unknown vacuum action: {other}")),
            }
        })
        .await
        .map_err(|e| server_error(e.to_string()))?
        .map_err(server_error)?;
        Ok(())
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (nozzle, action);
        unreachable!()
    }
}

/// The pick half of the guided calibration. The operator has jogged the
/// nozzle down onto the part; this records the current Z as the
/// footprint's pickup height, opens suction and retracts the nozzle.
#[server(prefix = "/api", endpoint = "cal_pick")]
pub async fn cal_pick(nozzle: u8, footprint: String) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let nozzle_id = shared_types::NozzleId::from_id(nozzle)
            .ok_or_else(|| server_error(format!("unknown nozzle {nozzle}")))?;
        let footprint = footprint.trim().to_string();
        let machine = state.machine.clone();
        let recorded = tokio::task::spawn_blocking(move || -> Result<f64, String> {
            let position = machine
                .cached_position()
                .ok_or_else(|| "machine has not reported a position".to_string())?;
            let z = position.z[(nozzle_id.id() - 1) as usize];
            machine
                .open_vacuum(nozzle_id)
                .map_err(|e| format!("vacuum failed: {e}"))?;
            machine
                .move_z(nozzle_id, 0.0, 3)
                .map_err(|e| format!("retract failed: {e}"))?;
            Ok(z)
        })
        .await
        .map_err(|e| server_error(e.to_string()))?
        .map_err(server_error)?;

        if footprint.is_empty() {
            return Ok(format!(
                "Suction on, nozzle retracted. Z was {recorded:.3} mm — no footprint selected, height not recorded."
            ));
        }
        let mut entry = state
            .heights
            .map()
            .remove(&footprint)
            .unwrap_or_default();
        entry.pickup_z_mm = recorded;
        state
            .heights
            .save_entry(&footprint, entry)
            .map_err(|e| server_error(format!("Unable to save heights: {e}")))?;
        Ok(format!(
            "Pick recorded for {footprint}: pickup Z = {recorded:.3} mm. Suction is on and the nozzle is retracted."
        ))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (nozzle, footprint);
        unreachable!()
    }
}

/// The place half of the guided calibration. The operator has jogged
/// the nozzle (carrying the part) down to the target; this records the
/// current Z as the footprint's place height, blows the part off and
/// retracts the nozzle.
#[server(prefix = "/api", endpoint = "cal_place")]
pub async fn cal_place(nozzle: u8, footprint: String) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let nozzle_id = shared_types::NozzleId::from_id(nozzle)
            .ok_or_else(|| server_error(format!("unknown nozzle {nozzle}")))?;
        let footprint = footprint.trim().to_string();
        let machine = state.machine.clone();
        let recorded = tokio::task::spawn_blocking(move || -> Result<f64, String> {
            let position = machine
                .cached_position()
                .ok_or_else(|| "machine has not reported a position".to_string())?;
            let z = position.z[(nozzle_id.id() - 1) as usize];
            machine
                .close_vacuum_open_blow(nozzle_id)
                .map_err(|e| format!("release failed: {e}"))?;
            machine
                .move_z(nozzle_id, 0.0, 3)
                .map_err(|e| format!("retract failed: {e}"))?;
            Ok(z)
        })
        .await
        .map_err(|e| server_error(e.to_string()))?
        .map_err(server_error)?;

        if footprint.is_empty() {
            return Ok(format!(
                "Part released, nozzle retracted. Z was {recorded:.3} mm — no footprint selected, height not recorded."
            ));
        }
        let mut entry = state
            .heights
            .map()
            .remove(&footprint)
            .unwrap_or_default();
        entry.place_z_mm = recorded;
        state
            .heights
            .save_entry(&footprint, entry)
            .map_err(|e| server_error(format!("Unable to save heights: {e}")))?;
        Ok(format!(
            "Place recorded for {footprint}: place Z = {recorded:.3} mm. Part released and the nozzle is retracted."
        ))
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = (nozzle, footprint);
        unreachable!()
    }
}

/// Retract one nozzle to Z 0 (fully up).
#[server(prefix = "/api", endpoint = "cal_z_zero")]
pub async fn cal_z_zero(nozzle: u8) -> Result<(), ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        let nozzle_id = shared_types::NozzleId::from_id(nozzle)
            .ok_or_else(|| server_error(format!("unknown nozzle {nozzle}")))?;
        let machine = state.machine.clone();
        tokio::task::spawn_blocking(move || -> Result<(), String> {
            machine
                .move_z(nozzle_id, 0.0, 3)
                .map_err(|e| format!("Z move failed: {e}"))
        })
        .await
        .map_err(|e| server_error(e.to_string()))?
        .map_err(server_error)?;
        Ok(())
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = nozzle;
        unreachable!()
    }
}

// ---------------------------------------------------------------------------
// Job library: save / load / list / delete named job snapshots
// ---------------------------------------------------------------------------

/// One entry of the job library listing: name plus last-modified time
/// (Unix ms for sorting, preformatted UTC text for display).
#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct JobListing {
    pub name: String,
    pub modified_ms: u64,
    pub modified_text: String,
}

#[server(prefix = "/api", endpoint = "list_jobs")]
pub async fn list_jobs() -> Result<Vec<JobListing>, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        crate::job_library::list_jobs().map_err(server_error)
    }
    #[cfg(not(feature = "ssr"))]
    {
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "save_job_as")]
pub async fn save_job_as(name: String) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        crate::job_library::save_job_as(&state.store, &name).map_err(server_error)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = name;
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "load_job")]
pub async fn load_job(name: String) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        crate::job_library::load_job(&state.store, &name).map_err(server_error)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = name;
        unreachable!()
    }
}

#[server(prefix = "/api", endpoint = "delete_job")]
pub async fn delete_job(name: String) -> Result<String, ServerFnError> {
    #[cfg(feature = "ssr")]
    {
        let state: AppState =
            use_context::<AppState>().ok_or_else(|| server_error("missing application state"))?;
        crate::job_library::delete_job(&state.store, &name).map_err(server_error)
    }
    #[cfg(not(feature = "ssr"))]
    {
        let _ = name;
        unreachable!()
    }
}

#[cfg(all(test, feature = "ssr"))]
mod station_fallback_tests {
    use super::{camera_station, feeder_station};
    use pnp_logic::{Camera, CameraKind, FeederSlot, Point};

    fn feeder(id: u8, x: f64, y: f64) -> FeederSlot {
        FeederSlot {
            id,
            bank_id: 0,
            position: Point { x, y },
            loaded: None,
        }
    }

    fn camera(kind: CameraKind, x: f64, y: f64) -> Camera {
        Camera {
            id: 0,
            kind,
            position: Point { x, y },
            array_pitch_mm: None,
        }
    }

    #[test]
    fn feeder_station_prefers_the_exact_slot() {
        let feeders = vec![feeder(1, 10.0, 400.0), feeder(4, 80.0, 430.0)];
        assert_eq!(feeder_station(&feeders, 4), (80.0, 430.0));
    }

    #[test]
    fn feeder_station_never_emits_nan() {
        // Unknown slot -> first feeder; no feeders at all -> origin.
        let feeders = vec![feeder(3, 12.0, 34.0)];
        assert_eq!(feeder_station(&feeders, 9), (12.0, 34.0));
        let (x, y) = feeder_station(&[], 9);
        assert_eq!((x, y), (0.0, 0.0));
        assert!(x.is_finite() && y.is_finite());
    }

    #[test]
    fn camera_station_prefers_kind_then_first_then_origin() {
        let cameras = vec![
            camera(CameraKind::HeadCamera, 0.0, 200.0),
            camera(CameraKind::Precise, 1.0, 2.0),
        ];
        assert_eq!(camera_station(&cameras, CameraKind::Precise), (1.0, 2.0));
        // Regression: jobs whose placements reference a FastVision camera
        // that is no longer in the camera list (e.g. wasd.json) fell back
        // to NaN, which serialized as JSON null and crashed the browser.
        let (x, y) = camera_station(&cameras, CameraKind::FastVision);
        assert_eq!((x, y), (0.0, 200.0));
        assert!(x.is_finite() && y.is_finite());
        let (ox, oy) = camera_station(&[], CameraKind::FastVision);
        assert_eq!((ox, oy), (0.0, 0.0));
        assert!(ox.is_finite() && oy.is_finite());
    }
}
