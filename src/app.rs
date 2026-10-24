use leptos::{html::Input, prelude::*, task::spawn_local};
use leptos_meta::*;
use leptos_router::{
    components::{FlatRoutes, Route, Router},
    StaticSegment,
};
use leptos_use::{
    use_debounce_fn, use_event_listener, use_event_listener_with_options, use_interval,
    UseEventListenerOptions,
};
use wasm_bindgen::JsCast;
use wasm_bindgen_futures::JsFuture;
use web_sys::{HtmlImageElement, HtmlInputElement};

use crate::server_fns::*;

/// The client-side single source of truth for part and job data.
///
/// One resource fetches one combined snapshot (`get_app_state`) that
/// covers the job summary, the whole parts library, blockers, warnings
/// and recorded heights. Every view reads from this context instead of
/// issuing its own fetches, so the parts table, the blockers banner and
/// the job counts can never disagree with one another.
///
/// `refresh` bumps a version the resource is keyed on, forcing an
/// immediate refetch after any mutating action; the 1 s interval keeps
/// the snapshot current otherwise.
#[derive(Clone, Copy)]
pub struct AppData {
    snapshot: Resource<Result<AppStateSnapshot, ServerFnError>>,
    refresh_signal: WriteSignal<u64>,
}

impl AppData {
    /// The latest snapshot, `None` while loading or on server errors.
    pub fn data(&self) -> Option<AppStateSnapshot> {
        self.snapshot.get().and_then(|result| result.ok())
    }

    /// Force an immediate refetch (after a save / delete / plan).
    pub fn refresh(&self) {
        self.refresh_signal.update(|version| *version += 1);
    }
}

pub fn shell(options: LeptosOptions) -> impl IntoView {
    view! {
        <!DOCTYPE html>
        <html lang="en">
            <head>
                <meta charset="utf-8" />
                <meta name="viewport" content="width=device-width, initial-scale=1" />
                <AutoReload options=options.clone() />
                <HydrationScripts options islands=true />
                <link rel="stylesheet" id="leptos" href="/pkg/leptos_tailwind.css" />
                <link rel="shortcut icon" type="image/ico" href="/favicon.ico" />
                <MetaTags />
            </head>
            <body>
                <App />
            </body>
        </html>
    }
}

#[component]
pub fn App() -> impl IntoView {
    provide_meta_context();

    view! {
        <Router>
            <FlatRoutes fallback=|| "Page not found.">
                <Route path=StaticSegment("") view=Home />
            </FlatRoutes>
        </Router>
    }
}

/// Shows the specified camera in the webvide
#[component]
fn CameraView() -> impl IntoView {
    let (camera, set_camera) = signal("head".to_string());
    let (show_enlarged, set_show_enlarged) = signal(false);
    // Stream version counter: bumping it changes the src URL (cache-buster),
    // forcing the browser to reconnect when the MJPEG stream stalls or dies.
    let (stream_version, set_stream_version) = signal(0u32);
    let camera_enabled = use_context::<ReadSignal<bool>>().expect("camera state context");
    let camera_src =
        move || format!("/api/camera/{}?v={}", camera.get(), stream_version.get());
    let refresh_stream = move || set_stream_version.update(|v| *v = v.wrapping_add(1));
    view! {
        <section class="w-full min-w-0 rounded border border-slate-600 bg-slate-800 p-2 md:w-fit md:max-w-[33vw]">
            <label class="mr-2" for="camera-select">
                "Camera"
            </label>
            <select
                id="camera-select"
                class="h-7 px-1 text-sm"
                on:change=move |event| {
                    set_camera.set(event_target_value(&event));
                    refresh_stream();
                }
            >
                <option value="head">"Head"</option>
                <option value="precise">"Precise vision"</option>
                <option value="fast-1">"Fast vision 1"</option>
                <option value="fast-2">"Fast vision 2"</option>
                <option value="fast-3">"Fast vision 3"</option>
                <option value="fast-4">"Fast vision 4"</option>
                <option value="fast-5">"Fast vision 5"</option>
                <option value="fast-6">"Fast vision 6"</option>
                <option value="fast-7">"Fast vision 7"</option>
                <option value="fast-8">"Fast vision 8"</option>
            </select>
            <button
                class="ml-2 rounded border border-slate-500 px-2 py-0.5 text-xs hover:bg-slate-700"
                title="Reconnect camera stream"
                on:click=move |_| refresh_stream()
            >
                "⟳"
            </button>
            <div class="relative mt-2 w-full min-w-0 overflow-hidden bg-slate-900 md:w-fit">
                <Show when=move || camera_enabled.get()>
                    <img
                        id="camera-preview"
                        class="block h-auto w-full cursor-zoom-in md:w-auto md:max-w-full"
                        src=camera_src
                        alt="Camera feed"
                        title="Click to enlarge"
                        on:click=move |_| set_show_enlarged.set(true)
                        on:error=move |_| refresh_stream()
                    />
                </Show>
            </div>
        </section>
        {move || {
            show_enlarged
                .get()
                .then(move || {
                    view! {
                        <div
                            class="fixed inset-0 z-[70] flex items-center justify-center bg-black/80 p-4"
                            on:click=move |_| set_show_enlarged.set(false)
                        >
                            <img
                                class="max-h-full max-w-full cursor-zoom-out"
                                src=camera_src
                                alt="Camera feed (enlarged)"
                                on:error=move |_| refresh_stream()
                            />
                        </div>
                    }
                })
        }}
    }
}

/// Actual home button to tell the machine to home
#[component]
fn HomeButton(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
    set_home_prompt: Option<WriteSignal<bool>>,
) -> impl IntoView {
    let run_action_action = ServerAction::<RunAction>::new();

    Effect::new(move |_| {
        if let Some(result) = run_action_action.value().get() {
            if result.is_ok() {
                if let Some(set_home_prompt) = set_home_prompt {
                    set_home_prompt.set(false);
                }
            }
            set_last_action.set(Some(match result {
                Ok(()) => "Home completed successfully".to_string(),
                Err(error) => format!("Home failed: {error}"),
            }));
        }
    });

    view! {
        <ActionForm action=run_action_action>
            <input type="hidden" name="action" value="home" />
            <button
                class="btn-xp"
                type="submit"
                disabled=move || matches!(machine_state.get(), MachineState::RunningJob(_))
            >
                "⌂"
            </button>
        </ActionForm>
    }
}

/// Actual home button to tell the machine to home
#[component]
fn ConveyorManager(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
) -> impl IntoView {
    let run_action_action = ServerAction::<RunAction>::new();

    Effect::new(move |_| {
        if let Some(result) = run_action_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(()) => "PCB Load".to_string(),
                Err(error) => format!("Home failed: {error}"),
            }));
        }
    });

    view! {
        <div class="flex min-w-0 flex-wrap gap-2 ">
        <ActionForm action=run_action_action>
            <input type="hidden" name="action" value="conveyor_clip_pcb" />
            <button
                class="btn-xp !text-xs !p-1"
                type="submit"
                disabled=move || machine_state.get() != MachineState::Ready
            >
                "PCB In"
            </button>
        </ActionForm>
        <ActionForm action=run_action_action>
            <input type="hidden" name="action" value="conveyor_unclip_and_move" />
            <button
                class="btn-xp !text-xs !p-1"
                type="submit"
                disabled=move || machine_state.get() != MachineState::Ready
            >
                "PCB Out"
            </button>
        </ActionForm>
        </div>
    }
}

/// Main prompt to home the machine due to unknown state
#[component]
fn HomePrompt(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
    set_home_prompt: WriteSignal<bool>,
) -> impl IntoView {
    view! {
        <Popup title="Machine needs homing".to_string() set_open=set_home_prompt>
            <p class="mt-2 text-sm text-slate-300">
                "The machine position is unknown. Home the machine please."
            </p>
            <div class="mt-4 flex justify-end">
                <HomeButton
                    machine_state=machine_state
                    set_last_action=set_last_action
                    set_home_prompt=Some(set_home_prompt)
                />
            </div>
        </Popup>
    }
}

/// Reusable modal popup for actions and custom forms.
#[component]
fn Popup(title: String, set_open: WriteSignal<bool>, children: Children) -> impl IntoView {
    view! {
        <div class="fixed inset-0 z-50 flex items-center justify-center bg-black/60 p-4">
            <div class="relative w-full max-w-sm rounded border border-slate-500 bg-slate-800 p-5 text-white shadow-xl">
                <button
                    class="absolute right-2 top-2 px-2 text-lg leading-none text-slate-300 hover:text-white"
                    type="button"
                    aria-label="Close popup"
                    on:click=move |event| {
                        event.prevent_default();
                        set_open.set(false);
                    }
                >
                    "×"
                </button>
                <h2 class="pr-8 text-lg font-semibold">{title}</h2>
                {children()}
            </div>
        </div>
    }
}

#[component]
fn SettingsMenu(
    set_show_settings: WriteSignal<bool>,
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
) -> impl IntoView {
    let calibration_action = ServerAction::<CalibrateCameraGrid>::new();
    let calibration_3x3_action = ServerAction::<CalibrateCamera3x3>::new();
    let center_action = ServerAction::<CenterHeadFiducial>::new();
    let center_feeder_action = ServerAction::<CenterFeederPocket>::new();
    let pocket_settings_action = ServerAction::<SetFeederPocketSettings>::new();
    let (calibration_message, set_calibration_message) = signal::<Option<String>>(None);
    let (calibration_mode, set_calibration_mode) = signal("four".to_string());
    let (pocket_camera, set_pocket_camera) = signal("head".to_string());
    Effect::new(move |_| {
        if let Some(result) = center_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(message) => message,
                Err(error) => format!("Fiducial centering failed: {error}"),
            }));
        }
    });
    Effect::new(move |_| {
        if let Some(result) = center_feeder_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(message) => message,
                Err(error) => format!("Feeder-pocket centering failed: {error}"),
            }));
        }
    });
    Effect::new(move |_| {
        if let Some(result) = calibration_action.value().get() {
            set_calibration_message.set(Some(match result {
                Ok(message) => message,
                Err(error) => format!("Calibration failed: {error}"),
            }));
        }
    });
    Effect::new(move |_| {
        if let Some(result) = pocket_settings_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(message) => message,
                Err(error) => format!("Feeder-pocket settings failed: {error}"),
            }));
        }
    });
    view! {
        <Popup title="Settings".to_string() set_open=set_show_settings>
            <details class="mt-3 rounded border border-slate-600 p-2" open=true>
                <summary class="cursor-pointer font-semibold">"Camera Calibration"</summary>
                <details class="mt-2 rounded border border-slate-600 p-2" open=true>
                    <summary class="cursor-pointer">"Head"</summary>
                    <label class="mt-3 block text-sm">
                        "Calibration method"
                        <select
                            class="ml-2 p-1"
                            on:change=move |event| set_calibration_mode.set(event_target_value(&event))
                        >
                            <option value="four">"Four points"</option>
                            <option value="three-by-three">"3x3 grid"</option>
                        </select>
                    </label>
                    <Show when=move || calibration_mode.get() == "four">
                    <div class="mt-3 text-sm text-slate-300">
                        <p>"Enter the four machine coordinates where the board landmarks are visible."</p>
                        <p class="mt-1">"Use the same order for all four points around the grid."</p>
                    </div>
                    <ActionForm action=calibration_action attr:class="mt-3 grid grid-cols-2 gap-2">
                <label class="text-sm">"Point 1 X"<input class="w-full p-1 " name="x1" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 1 Y"<input class="w-full p-1 " name="y1" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 2 X"<input class="w-full p-1 " name="x2" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 2 Y"<input class="w-full p-1 " name="y2" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 3 X"<input class="w-full p-1 " name="x3" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 3 Y"<input class="w-full p-1 " name="y3" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 4 X"<input class="w-full p-1 " name="x4" type="number" step="any" required=true /></label>
                <label class="text-sm">"Point 4 Y"<input class="w-full p-1 " name="y4" type="number" step="any" required=true /></label>
                        <button class="btn-xp col-span-2" type="submit">"Calibrate head camera"</button>
                    </ActionForm>
                    </Show>
                    <Show when=move || calibration_mode.get() == "three-by-three">
                        <div class="mt-3 text-sm text-slate-300">
                            <p>"Enter the center point of the grid in machine coordinates."</p>
                            <p class="mt-1">"The center is visited as point 5 in a 20 x 20 mm, 3x3 grid."</p>
                        </div>
                        <ActionForm action=calibration_3x3_action attr:class="mt-3 grid grid-cols-2 gap-2">
                            <label class="text-sm">"Origin X"<input class="w-full p-1 " name="origin_x" type="number" step="any" required=true /></label>
                            <label class="text-sm">"Origin Y"<input class="w-full p-1 " name="origin_y" type="number" step="any" required=true /></label>
                            <button class="btn-xp col-span-2" type="submit">"Calibrate 3x3 grid"</button>
                        </ActionForm>
                    </Show>
                </details>
            </details>
            <details class="mt-3 rounded border border-slate-600 p-2" open=true>
                <summary class="cursor-pointer font-semibold">"Feeder-Pocket Vision"</summary>
                <p class="mt-2 text-sm text-slate-300">"Adjust detection geometry independently for each camera. Values are persisted on the server."</p>
                <ActionForm action=pocket_settings_action attr:class="mt-3 grid grid-cols-2 gap-2">
                    <label class="col-span-2 text-sm">"Camera"
                        <select class="ml-2 p-1" name="camera" on:change=move |event| set_pocket_camera.set(event_target_value(&event))>
                            <option value="head">"Head"</option>
                            <option value="precise">"Precise vision"</option>
                            <option value="fast-1">"Fast vision 1"</option>
                            <option value="fast-2">"Fast vision 2"</option>
                            <option value="fast-3">"Fast vision 3"</option>
                            <option value="fast-4">"Fast vision 4"</option>
                            <option value="fast-5">"Fast vision 5"</option>
                            <option value="fast-6">"Fast vision 6"</option>
                            <option value="fast-7">"Fast vision 7"</option>
                            <option value="fast-8">"Fast vision 8"</option>
                        </select>
                    </label>
                    <label class="text-sm">"Pocket size (mm)"<input class="w-full p-1" name="size_mm" type="number" step="any" value="10" required=true /></label>
                    <label class="text-sm">"Position tolerance (mm)"<input class="w-full p-1" name="position_tolerance_mm" type="number" step="any" value="3" min="0" required=true /></label>
                    <label class="text-sm">"Size tolerance (mm)"<input class="w-full p-1" name="size_tolerance_mm" type="number" step="any" value="6" min="0" required=true /></label>
                    <label class="text-sm">"Pixels per mm"<input class="w-full p-1" name="px_per_mm" type="number" step="any" value="20" min="0.001" required=true /></label>
                    <button class="btn-xp col-span-2" type="submit">"Save feeder-pocket settings"</button>
                </ActionForm>
                <ActionForm action=center_feeder_action attr:class="mt-2">
                    <input type="hidden" name="camera" value=move || pocket_camera.get() />
                    <button class="btn-xp w-full" type="submit">"Center selected camera on feeder pocket"</button>
                </ActionForm>
            </details>
            <div class="mt-4 border-t border-slate-600 pt-3">
                <p class="text-sm font-semibold">"Manual head movement"</p>
                <RelativeMoveButton
                    machine_state=machine_state
                    set_last_action=set_last_action
                />
                <ActionForm action=center_action attr:class="mt-2">
                    <button class="btn-xp w-full" type="submit">"Center detected fiducial"</button>
                </ActionForm>
            </div>
            {move || calibration_message.get().map(|message| view! { <p class="mt-3 text-sm">{message}</p> })}
        </Popup>
    }
}

/// Part library panel.
///
/// Lists every part the operator has defined or had derived from a
/// footprint name, lets them create and edit parts by hand, and blocks
/// the job from running while any part needs review.
#[component]
fn PartsPanel(set_last_action: WriteSignal<Option<String>>) -> impl IntoView {
    let save_action = ServerAction::<SavePart>::new();
    let delete_action = ServerAction::<DeletePart>::new();
    // All library data comes from the shared snapshot; nothing is
    // fetched or cached locally, so the table can never go stale
    // relative to the rest of the UI.
    let app = expect_context::<AppData>();
    let parts = move || -> Vec<PartClient> {
        app.data()
            .map(|data| {
                data.parts
                    .iter()
                    .map(|entry| PartClient {
                        id: entry.id,
                        mpn: entry.mpn.clone(),
                        footprint: entry.footprint.clone(),
                        description: entry.description.clone(),
                        nozzles: entry.nozzles.clone(),
                        rotation: entry.rotation.to_string(),
                        offset_x: entry.offset_x.to_string(),
                        offset_y: entry.offset_y.to_string(),
                        status: entry.status.clone(),
                        camera: entry.camera.clone(),
                    })
                    .collect()
            })
            .unwrap_or_default()
    };
    let blockers = move || app.data().map(|d| d.blockers).unwrap_or_default();
    let warnings = move || app.data().map(|d| d.warnings).unwrap_or_default();
    let heights = move || app.data().map(|d| d.heights).unwrap_or_default();
    let (editing, set_editing) = signal::<Option<u32>>(None);
    let (form_mpn, set_form_mpn) = signal(String::new());
    let (form_footprint, set_form_footprint) = signal(String::new());
    let (form_description, set_form_description) = signal(String::new());
    let (form_nozzles, set_form_nozzles) = signal(String::new());
    let (form_rotation, set_form_rotation) = signal(String::from("0"));
    let (form_offset_x, set_form_offset_x) = signal(String::from("0"));
    let (form_offset_y, set_form_offset_y) = signal(String::from("0"));
    let (form_status, set_form_status) = signal(String::from("Guessed"));
    let (form_camera, set_form_camera) = signal(String::from("auto"));
    // Part edits (nozzle allowlist, camera, ...) change what the planner
    // can do, so they ask for a re-optimisation. Debounced: a burst of
    // edits becomes one re-plan.
    let (plan_cmd, set_plan_cmd) = (
        use_context::<ReadSignal<(u64, String)>>().expect("plan command context"),
        use_context::<WriteSignal<(u64, String)>>().expect("plan command writer"),
    );
    let replan = use_debounce_fn(
        move || {
            let (sequence, _) = plan_cmd.get();
            set_plan_cmd.set((sequence + 1, "plan".to_string()));
        },
        1500.0,
    );
    // Nozzle allowlist as a set of ticked kinds; serialised to the JSON
    // the server expects in a hidden input.
    let (form_nozzle_set, set_form_nozzle_set) = signal::<Vec<String>>(vec![]);
    // Juki nozzle catalogue Rev C3 (2000-series heads): tips 501–508,
    // bucketed by component width. Legacy Standard/Fine/ExtraFine are
    // kept so parts saved before the migration still render correctly.
    let nozzle_kind_options: &[(&str, &str)] = &[
        ("n501", "501 (0.7 x 0.4 mm)"),
        ("n502", "502 (0.7 mm)"),
        ("n503", "503 (1.0 mm)"),
        ("n504", "504 (1.5 mm)"),
        ("n505", "505 (3.5 mm)"),
        ("n506", "506 (5.0 mm)"),
        ("n507", "507 (8.5 mm)"),
        ("n508", "508 (9.5 mm)"),
        ("standard", "Legacy standard (3.0 mm)"),
        ("fine", "Legacy fine (1.5 mm)"),
        ("extra_fine", "Legacy extra fine (1.0 mm)"),
    ];
    let toggle_nozzle = move |kind: &'static str| {
        set_form_nozzle_set.update(|set| {
            if set.iter().any(|k| k == kind) {
                set.retain(|k| k != kind);
            } else {
                set.push(kind.to_string());
            }
        });
    };
    let parse_nozzle_set = |json: &str| -> Vec<String> {
        serde_json::from_str::<Vec<String>>(json)
            .unwrap_or_default()
            .into_iter()
            .map(|kind| kind.trim_start_matches("NozzleKind::").to_ascii_lowercase())
            .collect()
    };
    // The server expects the serde variant names ("Standard", "N501", ...).
    let nozzle_json = move || {
        let names: Vec<String> = form_nozzle_set
            .get()
            .iter()
            .map(|kind| match kind.as_str() {
                "standard" => "Standard",
                "fine" => "Fine",
                "extra_fine" => "ExtraFine",
                "n501" => "N501",
                "n502" => "N502",
                "n503" => "N503",
                "n504" => "N504",
                "n505" => "N505",
                "n506" => "N506",
                "n507" => "N507",
                "n508" => "N508",
                other => other,
            })
            .map(|name| name.to_string())
            .collect();
        serde_json::to_string(&names).unwrap_or_else(|_| "[]".to_string())
    };
    // In-progress edits of the footprint editor, keyed by footprint name:
    // [width, height, pickup_z, place_z] as raw strings.
    let (fp_edits, set_fp_edits) = signal::<std::collections::HashMap<String, [String; 4]>>(
        std::collections::HashMap::new(),
    );

    // Client-side part summary — only the fields needed for display.
    // This type uses only simple primitive types (strings, numbers) so it
    // can be transferred to the wasm target without requiring the `pnp_logic`
    // crate (which depends on OpenCV).
    #[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
    struct PartClient {
        id: u32,
        mpn: String,
        footprint: String,
        description: String,
        nozzles: String,     // JSON string of nozzle allowlist
        rotation: String,    // pickup rotation in degrees
        offset_x: String,    // pickup offset X in mm
        offset_y: String,    // pickup offset Y in mm
        status: String,      // "Guessed", "Confirmed", "NeedsReview"
        camera: String,      // "auto", "fast", "precise", "head"
    }

    // Save one footprint's recorded parameters, then refresh.
    let save_heights = move |footprint: String, values: [String; 4]| {
        spawn_local(async move {
            match save_part_height(
                footprint.clone(),
                values[2].clone(),
                values[3].clone(),
                values[0].clone(),
                values[1].clone(),
            )
            .await
            {
                Ok(message) => set_last_action.set(Some(message)),
                Err(error) => set_last_action.set(Some(format!("Save failed: {error}"))),
            }
            app.refresh();
            set_fp_edits.update(|edits| {
                edits.remove(&footprint);
            });
        });
    };
    // Every footprint the operator can tune: those in the library plus
    // those that already have recorded heights.
    let footprints = move || {
        let mut names: Vec<String> = parts()
            .iter()
            .map(|entry| entry.footprint.clone())
            .chain(heights().iter().map(|h| h.footprint.clone()))
            .collect();
        names.sort();
        names.dedup();
        names
    };
    // Current field values for a footprint row: pending edits win over
    // the recorded values, which win over the machine defaults.
    let field = move |footprint: &str, slot: usize, recorded: Option<f64>| -> String {
        if let Some(edit) = fp_edits.get().get(footprint) {
            if !edit[slot].is_empty() {
                return edit[slot].clone();
            }
        }
        recorded.map(|value| format!("{value:.3}")).unwrap_or_default()
    };
    let set_field = move |footprint: &str, slot: usize, value: String| {
        set_fp_edits.update(|edits| {
            let entry = edits.entry(footprint.to_string()).or_insert([
                String::new(),
                String::new(),
                String::new(),
                String::new(),
            ]);
            entry[slot] = value;
        });
    };
    let effect_replan = replan.clone();
    Effect::new(move |_| {
        // Reload whenever a plan completes too: planning creates guessed
        // library parts (named value-footprint) that must show up here
        // without the operator having to save a part or switch tabs.
        let (plan_sequence, _) = plan_cmd.get();
        let _ = plan_sequence;
        app.refresh();
        let replan = effect_replan.clone();
        if let Some(result) = save_action.value().get() {
            match result {
                Ok(message) => set_last_action.set(Some(message)),
                Err(error) => set_last_action.set(Some(format!("Save failed: {error}"))),
            }
            app.refresh();
            replan();
        }
        if let Some(result) = delete_action.value().get() {
            match result {
                Ok(message) => set_last_action.set(Some(message)),
                Err(error) => set_last_action.set(Some(format!("Delete failed: {error}"))),
            }
            app.refresh();
            replan();
        }
    });

    let start_edit = move |id: u32| {
        // Look up the part from the library and populate the form strings.
        if let Some(entry) = parts().iter().find(|e| e.id == id) {
            set_form_mpn.set(entry.mpn.clone());
            set_form_footprint.set(entry.footprint.clone());
            set_form_description.set(entry.description.clone());
            set_form_nozzles.set(entry.nozzles.clone());
            set_form_nozzle_set.set(parse_nozzle_set(&entry.nozzles));
            set_form_rotation.set(entry.rotation.clone());
            set_form_offset_x.set(entry.offset_x.clone());
            set_form_offset_y.set(entry.offset_y.clone());
            set_form_status.set(entry.status.clone());
            set_form_camera.set(entry.camera.clone());
        }
        set_editing.set(Some(id));
    };

    view! {
        <section class="rounded border border-slate-600 p-2">
            <h2 class="flex items-baseline gap-2 text-lg font-semibold">
                "Parts & Footprints"
                <span class="text-xs font-normal text-slate-400">
                    {move || format!("{} parts", parts().len())}
                </span>
            </h2>

            <Show when=move || !blockers().is_empty()>
                <div class="mt-2 rounded border border-red-500 bg-red-900/40 p-2 text-xs">
                    <p class="font-semibold text-red-300">"Cannot run yet:"</p>
                    <ul class="mt-1 list-disc pl-4 text-red-200">
                        {move || blockers().into_iter().map(|b| view! { <li>{b}</li> }).collect_view()}
                    </ul>
                </div>
            </Show>

            <Show when=move || !warnings().is_empty()>
                <div class="mt-2 rounded border border-amber-500 bg-amber-900/40 p-2 text-xs">
                    <p class="font-semibold text-amber-300">"Worth fixing:"</p>
                    <ul class="mt-1 list-disc pl-4 text-amber-200">
                        {move || warnings().into_iter().map(|w| view! { <li>{w}</li> }).collect_view()}
                    </ul>
                </div>
            </Show>

            <div class="mt-3 max-h-64 overflow-auto">
                <table class="min-w-full text-left text-xs">
                    <thead class="sticky top-0 bg-slate-700">
                        <tr>
                            <th class="p-1">"ID"</th>
                            <th class="p-1">"MPN"</th>
                            <th class="p-1">"Footprint"</th>
                            <th class="p-1">"Nozzles"</th>
                            <th class="p-1">"Status"</th>
                            <th class="p-1">"Camera"</th>
                            <th class="p-1">"Actions"</th>
                        </tr>
                    </thead>
                    <tbody>
                        {move || parts().into_iter().map(|entry| {
                            let entry_snapshot = entry.clone();
                            let status_class = match entry.status.as_str() {
                                "Confirmed" => "text-green-400",
                                "Guessed" => "text-yellow-400",
                                "NeedsReview" => "text-red-400",
                                _ => "text-slate-400",
                            };
                            view! {
                                <tr class="border-t border-slate-600">
                                    <td class="p-1">{entry.id}</td>
                                    <td class="p-1">{entry.mpn}</td>
                                    <td class="p-1">{entry.footprint}</td>
                                    <td class="p-1">{entry.nozzles}</td>
                                    <td class={format!("p-1 {}", status_class)}>{entry.status.clone()}</td>
                                    <td class="p-1">
                                        {let entry_camera = entry_snapshot.clone();
                                        let row_replan = replan.clone();
                                        let save_camera = move |camera: String| {
                                            let entry = entry_camera.clone();
                                            let replan = row_replan.clone();
                                            spawn_local(async move {
                                                let _ = save_part(
                                                    entry.id.to_string(),
                                                    entry.mpn.clone(),
                                                    entry.description.clone(),
                                                    entry.footprint.clone(),
                                                    entry.nozzles.clone(),
                                                    entry.rotation.clone(),
                                                    entry.offset_x.clone(),
                                                    entry.offset_y.clone(),
                                                    entry.status.clone(),
                                                    camera,
                                                )
                                                .await;
                                                app.refresh();
                                                replan();
                                            });
                                        };
                                        view! {
                                            <select
                                                class="bg-slate-700 p-1"
                                                on:change=move |event| save_camera(event_target_value(&event))
                                            >
                                                <option value="auto" selected=entry_snapshot.camera == "auto">"Fast array (default)"</option>
                                                <option value="fast" selected=entry_snapshot.camera == "fast">"Fast (array)"</option>
                                                <option value="precise" selected=entry_snapshot.camera == "precise">"Precise"</option>
                                                <option value="head" selected=entry_snapshot.camera == "head">"No array cam (head)"</option>
                                            </select>
                                        }}
                                    </td>
                                    <td class="p-1">
                                        <button
                                            class="btn-xp !text-xs !p-1 mr-1"
                                            type="button"
                                            on:click=move |_| start_edit(entry.id)
                                        >
                                            "Edit"
                                        </button>
                                        <ActionForm action=delete_action>
                                            <input type="hidden" name="id" value=entry.id.clone() />
                                            <button class="btn-xp !text-xs !p-1" type="submit">"Del"</button>
                                        </ActionForm>
                                    </td>
                                </tr>
                            }
                        }).collect_view()}
                    </tbody>
                </table>
            </div>

            <Show when=move || editing.get().is_some()>
                <div class="mt-3 rounded border border-slate-500 bg-slate-700/50 p-2">
                    <p class="text-xs font-semibold text-slate-300">
                        {move || match editing.get() {
                            Some(id) => format!("Editing part {id}"),
                            None => "New part".to_string(),
                        }}
                    </p>
                    <ActionForm action=save_action attr:class="mt-2 grid grid-cols-2 gap-2">
                        <input type="hidden" name="id" value=move || editing.get().unwrap_or(0) />
                        <label class="text-xs">"MPN"
                            <input class="w-full p-1" name="mpn" type="text"
                                value=move || form_mpn.get() />
                        </label>
                        <label class="text-xs">"Footprint name"
                            <input class="w-full p-1" name="footprint_name" type="text"
                                value=move || form_footprint.get() />
                        </label>
                        <label class="text-xs">"Description"
                            <input class="w-full p-1" name="description" type="text"
                                value=move || form_description.get() />
                        </label>
                        <div class="col-span-2">
                            <p class="text-xs text-slate-300">"Nozzle types (pick every tip this part is rated for):"</p>
                            <input type="hidden" name="nozzle_types" value=nozzle_json />
                            <div class="mt-1 grid grid-cols-2 gap-1 text-xs sm:grid-cols-3">
                                {nozzle_kind_options
                                    .iter()
                                    .map(|(kind, label)| {
                                        view! {
                                            <label class="flex items-center gap-1">
                                                <input
                                                    type="checkbox"
                                                    prop:checked=move || {
                                                        form_nozzle_set.get().iter().any(|k| k == *kind)
                                                    }
                                                    on:change=move |_| toggle_nozzle(kind)
                                                />
                                                {label.to_string()}
                                            </label>
                                        }
                                    })
                                    .collect_view()}
                            </div>
                        </div>
                        <label class="text-xs">"Pickup rotation (deg)"
                            <input class="w-full p-1" name="pickup_rotation" type="number" step="any"
                                value=move || form_rotation.get() />
                        </label>
                        <label class="text-xs">"Pickup offset X (mm)"
                            <input class="w-full p-1" name="pickup_offset_x" type="number" step="any"
                                value=move || form_offset_x.get() />
                        </label>
                        <label class="text-xs">"Pickup offset Y (mm)"
                            <input class="w-full p-1" name="pickup_offset_y" type="number" step="any"
                                value=move || form_offset_y.get() />
                        </label>
                        <label class="text-xs col-span-2">"Status"
                            <select class="w-full p-1" name="status">
                                <option value="Guessed" selected=move || form_status.get() == "Guessed">
                                    "Guessed"
                                </option>
                                <option value="Confirmed">Confirmed</option>
                                <option value="NeedsReview">Needs Review</option>
                            </select>
                        </label>
                        <label class="text-xs col-span-2">"Camera (vision for this part)"
                            <select class="w-full p-1" name="camera">
                                <option value="auto" selected=move || form_camera.get() == "auto">
                                    "Fast array (default)"
                                </option>
                                <option value="fast" selected=move || form_camera.get() == "fast">
                                    "Fast (4-camera array)"
                                </option>
                                <option value="precise" selected=move || form_camera.get() == "precise">
                                    "Precise (single camera)"
                                </option>
                                <option value="head" selected=move || form_camera.get() == "head">
                                    "No array camera (head cam)"
                                </option>
                            </select>
                        </label>
                        <button class="btn-xp col-span-2" type="submit">"Save part"</button>
                    </ActionForm>
                </div>
            </Show>

            <button
                class="btn-xp mt-2"
                type="button"
                on:click=move |_| { set_editing.set(None); set_form_mpn.set(String::new()); set_form_footprint.set(String::new()); set_form_description.set(String::new()); set_form_nozzles.set(String::new()); set_form_rotation.set(String::from("0")); set_form_offset_x.set(String::from("0")); set_form_offset_y.set(String::from("0")); set_form_status.set(String::from("Guessed")); set_form_camera.set(String::from("auto")); set_form_nozzle_set.set(vec![]); }
            >
                "New part"
            </button>

            <details class="mt-4 rounded border border-slate-600 p-2" open=true>
                <summary class="cursor-pointer font-semibold">"Footprint editor"</summary>
                <p class="mt-1 text-xs text-slate-400">
                    "Pick and place Z heights are stored per footprint and replayed \
                     by the machine for every component with that footprint. Body \
                     size feeds the vision window and the process view."
                </p>
                <div class="mt-2 max-h-72 overflow-auto">
                    <table class="min-w-full text-left text-xs">
                        <thead class="sticky top-0 bg-slate-700">
                            <tr>
                                <th class="p-1">"Footprint"</th>
                                <th class="p-1">"Width (mm)"</th>
                                <th class="p-1">"Height (mm)"</th>
                                <th class="p-1">"Pick Z (mm)"</th>
                                <th class="p-1">"Place Z (mm)"</th>
                                <th class="p-1"></th>
                            </tr>
                        </thead>
                        <tbody>
                            {move || {
                                footprints()
                                    .into_iter()
                                    .map(|footprint| {
                                        let fp_width = footprint.clone();
                                        let fp_height = footprint.clone();
                                        let fp_pickup = footprint.clone();
                                        let fp_place = footprint.clone();
                                        let fp_save = footprint.clone();
                                        let recorded = heights()
                                            .iter()
                                            .find(|h| h.footprint == footprint)
                                            .cloned();
                                        let pickup = field(&footprint, 2, recorded.as_ref().map(|h| h.pickup_z_mm));
                                        let place = field(&footprint, 3, recorded.as_ref().map(|h| h.place_z_mm));
                                        let width = field(&footprint, 0, recorded.as_ref().map(|h| h.width_mm));
                                        let height = field(&footprint, 1, recorded.as_ref().map(|h| h.height_mm));
                                        view! {
                                            <tr class="border-t border-slate-600">
                                                <td class="p-1 font-mono">{footprint.clone()}</td>
                                                <td class="p-1">
                                                    <input
                                                        class="w-20 bg-slate-700 p-1"
                                                        type="number" step="any"
                                                        value=width
                                                        on:input=move |event| set_field(&fp_width, 0, event_target_value(&event))
                                                    />
                                                </td>
                                                <td class="p-1">
                                                    <input
                                                        class="w-20 bg-slate-700 p-1"
                                                        type="number" step="any"
                                                        value=height
                                                        on:input=move |event| set_field(&fp_height, 1, event_target_value(&event))
                                                    />
                                                </td>
                                                <td class="p-1">
                                                    <input
                                                        class="w-20 bg-slate-700 p-1"
                                                        type="number" step="any"
                                                        value=pickup
                                                        on:input=move |event| set_field(&fp_pickup, 2, event_target_value(&event))
                                                    />
                                                </td>
                                                <td class="p-1">
                                                    <input
                                                        class="w-20 bg-slate-700 p-1"
                                                        type="number" step="any"
                                                        value=place
                                                        on:input=move |event| set_field(&fp_place, 3, event_target_value(&event))
                                                    />
                                                </td>
                                                <td class="p-1">
                                                    <button
                                                        class="btn-xp !text-xs !p-1"
                                                        type="button"
                                                        on:click=move |_| {
                                                            let footprint = fp_save.clone();
                                                            let recorded = heights().iter().find(|h| h.footprint == footprint).cloned();
                                                            let edits = fp_edits.get().get(&footprint).cloned();
                                                            let value = |slot: usize, fallback: Option<f64>| -> String {
                                                                if let Some(edit) = edits.as_ref() {
                                                                    if !edit[slot].is_empty() {
                                                                        return edit[slot].clone();
                                                                    }
                                                                }
                                                                fallback.map(|v| format!("{v:.3}")).unwrap_or_else(|| "0".to_string())
                                                            };
                                                            save_heights(
                                                                footprint.clone(),
                                                                [
                                                                    value(0, recorded.as_ref().map(|h| h.width_mm)),
                                                                    value(1, recorded.as_ref().map(|h| h.height_mm)),
                                                                    value(2, recorded.as_ref().map(|h| h.pickup_z_mm)),
                                                                    value(3, recorded.as_ref().map(|h| h.place_z_mm)),
                                                                ],
                                                            );
                                                        }
                                                    >
                                                        "Save"
                                                    </button>
                                                </td>
                                            </tr>
                                        }
                                    })
                                    .collect_view()
                            }}
                        </tbody>
                    </table>
                </div>
            </details>
        </section>
    }
}

/// Job manager: turn the placement list into a plan, run it, and watch it
/// run.
///
/// This is the other half of the board editor. The placement list is the
/// operator's view of the job; this panel is the machine's:
///
/// - **Plan** resolves every footprint in the list against the part
///   library, guesses parameters for parts that are not in it yet, and
///   hands the result to the planner, which picks feeder slots, nozzles
///   and the run order before the job is stored.
/// - **Run** homes the machine, asks the operator to load the board and
///   to put the head camera over each fiducial, then drives pick and
///   place until the board is done.
/// - A job never starts while something is unresolved: the same
///   blockers the parts library reports disable the button.
#[component]
fn JobManager(
    set_last_action: WriteSignal<Option<String>>,
    plan: impl Fn(bool) + Copy + 'static,
    plan_note: ReadSignal<Option<String>>,
    job_version: ReadSignal<u64>,
    on_job_loaded: impl Fn() + Copy + Send + Sync + 'static,
) -> impl IntoView {
    // Job counts and blockers come from the shared snapshot.
    let app = expect_context::<AppData>();
    let summary = move || app.data().map(|d| d.job);
    let blockers = move || app.data().map(|d| d.blockers).unwrap_or_default();

    // Poll the server once a second. `job_status` doubles as the job
    // summary (counts) and as the live run state, so one request keeps
    // the whole panel current.
    let interval = use_interval(1000u64);
    let status = Resource::new(
        move || interval.counter.get(),
        |_| async { job_status().await },
    );

    Effect::new(move |_| {
        // Refetch the shared snapshot whenever the job version is bumped.
        job_version.get();
        app.refresh();
    });

    let start_run = move || {
        spawn_local(async move {
            // Going to run re-optimises first: the plan is rebuilt from
            // the stored placements against the current machine setup,
            // so the run always starts on a fresh optimization. The
            // server refuses this while a job is already running, in
            // which case the run is not started either.
            match reoptimize_job().await {
                Ok(planned) => {
                    set_last_action.set(Some(format!(
                        "Optimised {} placements; starting run",
                        planned.placement_count
                    )));
                }
                Err(error) => {
                    set_last_action.set(Some(format!("Start failed: {error}")));
                    return;
                }
            }
            match start_job().await {
                Ok(status) => {
                    set_last_action.set(Some(format!("Job started: {}", status.progress)))
                }
                Err(error) => set_last_action.set(Some(format!("Start failed: {error}"))),
            }
        });
    };

    let stop = move || {
        spawn_local(async move {
            match stop_job().await {
                Ok(status) => {
                    set_last_action.set(Some(format!("Job stopped: {}", status.progress)))
                }
                Err(error) => set_last_action.set(Some(format!("Stop failed: {error}"))),
            }
        });
    };

    let answer = move || {
        spawn_local(async move {
            if let Err(error) = answer_prompt().await {
                set_last_action.set(Some(format!("Answer failed: {error}")));
            }
        });
    };

    // The menu bar sends sequenced commands here: every change to the
    // (sequence, kind) pair triggers exactly one plan/run/stop.
    let job_cmd = use_context::<ReadSignal<(u64, String)>>().expect("job command context");
    Effect::new(move |_| {
        let (sequence, kind) = job_cmd.get();
        if sequence == 0 {
            return;
        }
        match kind.as_str() {
            "plan" => plan(true),
            "run" => start_run(),
            "stop" => stop(),
            _ => {}
        }
    });

    view! {
        <section class="flex min-h-0 flex-col gap-3">
            <div class="flex flex-wrap items-center justify-between gap-2">
                <h2 class="text-lg font-semibold">"Job manager"</h2>
                <div class="flex flex-wrap gap-2">
                    <button class="btn-xp" type="button" on:click=move |_| plan(true)>
                        "Plan job from placement list"
                    </button>
                    <button
                        class="btn-xp"
                        type="button"
                        on:click=move |_| start_run()
                        disabled=move || !blockers().is_empty()
                    >
                        "Run job"
                    </button>
                    <button class="btn-xp" type="button" on:click=move |_| stop()>
                        "Stop"
                    </button>
                </div>
            </div>

            <p class="text-xs text-slate-400">
                "Board ID is edited in the header above. Parts live in Settings → Parts Library."
            </p>

            {move || plan_note.get().map(|note| view! {
                <p class="whitespace-pre-line rounded border border-slate-600 bg-slate-800 p-2 text-xs text-slate-300">{note}</p>
            })}

            <Show when=move || !blockers().is_empty()>
                <div class="rounded border border-red-500 bg-red-900/40 p-2 text-xs">
                    <p class="font-semibold text-red-300">"Cannot run yet:"</p>
                    <ul class="mt-1 list-disc pl-4 text-red-200">
                        {move || blockers().into_iter().map(|b| view! { <li>{b}</li> }).collect_view()}
                    </ul>
                </div>
            </Show>

            <JobLibrary set_last_action=set_last_action on_job_loaded=on_job_loaded />

            <Suspense fallback=|| view! { <p class="text-xs text-slate-400">"Loading job..."</p> }>
                {move || {
                    status.get().map(|result| {
                        match result {
                            Ok(current) => {
                                let prompt = current.prompt.clone();
                                let action = current.action.clone();
                                view! {
                                    <div class="flex flex-col gap-2">
                                        <div class="rounded border border-slate-600 bg-slate-800 p-2 text-xs text-slate-300">
                                            {move || match summary() {
                                                Some(s) => format!(
                                                    "{}: {} placements, {} placed",
                                                    s.board_id, s.placement_count, s.placed_count
                                                ),
                                                None => "No job loaded".to_string(),
                                            }}
                                            <span class="ml-2 text-slate-400">{current.progress.clone()}</span>
                                        </div>
                                        {(!action.is_empty()).then(|| view! {
                                            <p class="text-xs text-slate-400">
                                                "Machine: " {action}
                                            </p>
                                        })}
                                        {current.aborted.map(|reason| view! {
                                            <p class="rounded border border-red-500 bg-red-900/40 p-2 text-xs text-red-200">
                                                {reason}
                                            </p>
                                        })}
                                        {prompt.map(|message| view! {
                                            <div class="rounded border border-yellow-500 bg-yellow-900/30 p-2">
                                                <p class="text-sm text-yellow-200">{message}</p>
                                                <button class="btn-xp mt-2" type="button" on:click=move |_| answer()>
                                                    "Done"
                                                </button>
                                            </div>
                                        })}
                                    </div>
                                }
                                .into_any()
                            }
                            Err(error) => view! {
                                <p class="text-xs text-red-300">{format!("Unable to load job: {error}")}</p>
                            }
                            .into_any(),
                        }
                    })
                }}
            </Suspense>
        </section>
    }
}

/// Handles absolute positioning
#[component]
fn MoveButton(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
) -> impl IntoView {
    let run_action_action = ServerAction::<RunAction>::new();

    Effect::new(move |_| {
        if let Some(result) = run_action_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(()) => "Move completed successfully".to_string(),
                Err(error) => format!("Move failed: {error}"),
            }));
        }
    });

    view! {
        <div class="min-w-0 rounded border border-slate-600 bg-slate-800 p-2">
            <ActionForm action=run_action_action attr:class="flex flex-wrap items-center gap-1">
                <input type="hidden" name="action" value="move_xy" />
                <input class="h-7 w-16 px-1 text-sm" type="number" name="actions[x]" value="100" />
                <input class="h-7 w-16 px-1 text-sm" type="number" name="actions[y]" value="200" />
                <input class="h-7 w-12 px-1 text-sm" type="number" name="actions[speed]" value="1" />
                <button
                    class="btn-xp"
                    type="submit"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    "Move"
                </button>
            </ActionForm>
            <div class="mt-2">
                <HomeButton
                    machine_state=machine_state
                    set_last_action=set_last_action
                    set_home_prompt=None
                />
            </div>
        </div>
    }
}

/// Z jog controls for a single nozzle, shown next to the XY jog pad.
///
/// Relative ±0.5 mm creeping, a full retract to Z 0, and an absolute
/// "go to height" field. The readout shows the selected nozzle's Z as
/// last reported by the machine.
#[component]
fn ZJogControls(set_last_action: WriteSignal<Option<String>>) -> impl IntoView {
    let (nozzle, set_nozzle) = signal("1".to_string());
    let (target, set_target) = signal(String::new());
    let (readout, set_readout) = signal::<Option<f64>>(None);

    let nozzle_id = move || nozzle.get().parse::<u8>().unwrap_or(1);

    let read = move || {
        spawn_local(async move {
            if let Ok(status) = get_status().await {
                let index = (nozzle_id() - 1).min(3) as usize;
                set_readout.set(Some(status.z[index]));
            }
        })
    };
    Effect::new(move |_| {
        read();
    });

    let jog = move |delta: f64| {
        let n = nozzle_id();
        spawn_local(async move {
            match cal_z_jog(n, format!("{delta}")).await {
                Ok(z) => {
                    set_readout.set(Some(z));
                    set_last_action.set(Some(format!("Nozzle {n} Z → {z:.3} mm")));
                }
                Err(error) => set_last_action.set(Some(format!("Z jog failed: {error}"))),
            }
        });
    };
    let zero = move |_| {
        let n = nozzle_id();
        spawn_local(async move {
            match cal_z_zero(n).await {
                Ok(()) => {
                    set_readout.set(Some(0.0));
                    set_last_action.set(Some(format!("Nozzle {n} retracted to Z 0")));
                }
                Err(error) => set_last_action.set(Some(format!("Z retract failed: {error}"))),
            }
        });
    };
    let go = move |_| {
        let n = nozzle_id();
        let target = target.get();
        let mut actions = std::collections::HashMap::new();
        actions.insert("nozzle".to_string(), n.to_string());
        actions.insert("height".to_string(), target.clone());
        actions.insert("speed".to_string(), "1".to_string());
        spawn_local(async move {
            match run_action("move_z".to_string(), actions).await {
                Ok(()) => set_last_action.set(Some(format!("Nozzle {n} → Z {target} mm"))),
                Err(error) => set_last_action.set(Some(format!("Z move failed: {error}"))),
            }
            if let Ok(status) = get_status().await {
                let index = (n - 1).min(3) as usize;
                set_readout.set(Some(status.z[index]));
            }
        });
    };

    view! {
        <div class="min-w-0 rounded border border-slate-600 bg-slate-800 p-2">
            <div class="flex items-center justify-between gap-2">
                <p class="text-xs font-semibold text-slate-300">"Z jog"</p>
                <select
                    class="bg-slate-700 p-1 text-xs"
                    on:change=move |event| {
                        set_nozzle.set(event_target_value(&event));
                        set_readout.set(None);
                    }
                >
                    <option value="1">"N1"</option>
                    <option value="2">"N2"</option>
                    <option value="3">"N3"</option>
                    <option value="4">"N4"</option>
                </select>
            </div>
            <p class="mt-1 font-mono text-sm text-slate-200">
                "Z " {move || readout.get().map(|z| format!("{z:.3} mm")).unwrap_or_else(|| "—".into())}
            </p>
            <div class="mt-1 grid grid-cols-3 gap-1">
                <button class="btn-xp !px-1 !text-xs" type="button" on:click=move |_| jog(-0.5)>
                    "−0.5"
                </button>
                <button class="btn-xp !px-1 !text-xs" type="button" on:click=move |_| zero(())>
                    "→ 0"
                </button>
                <button class="btn-xp !px-1 !text-xs" type="button" on:click=move |_| jog(0.5)>
                    "+0.5"
                </button>
            </div>
            <div class="mt-1 flex gap-1">
                <input
                    class="h-7 w-16 px-1 text-sm"
                    type="number"
                    step="any"
                    placeholder="mm"
                    prop:value=target
                    on:input=move |event| set_target.set(event_target_value(&event))
                />
                <button class="btn-xp !px-2 !text-xs" type="button" on:click=go>
                    "Go"
                </button>
            </div>
        </div>
    }
}

/// Handles relative XY movement.
#[component]
fn RelativeMoveButton(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
) -> impl IntoView {
    let run_action_action = ServerAction::<RunAction>::new();

    Effect::new(move |_| {
        if let Some(result) = run_action_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(()) => "Relative move completed successfully".to_string(),
                Err(error) => format!("Relative move failed: {error}"),
            }));
        }
    });

    view! {
        <ActionForm action=run_action_action attr:class="min-w-0 rounded border border-slate-600 bg-slate-800 p-2">
            <input type="hidden" name="action" value="move_relative" />
            <div class="flex flex-wrap items-center gap-1">
            <label class="mr-2" for="relative-distance">
                "Distance (mm)"
            </label>
            <select id="relative-distance" name="actions[distance]" class="h-7 px-1 text-sm">
                <option value="0.1">"0.1"</option>
                <option value="1" selected>
                    "1"
                </option>
                <option value="10">"10"</option>
                <option value="100">"100"</option>
            </select>
            </div>
            <input type="hidden" name="actions[speed]" value="1" />
            <div class="mt-2 grid w-32 grid-cols-3 gap-1">
                <span></span>
                <button
                    class="btn-xp"
                    type="submit"
                    name="actions[direction]"
                    value="up"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    "^"
                </button>
                <span></span>
                <button
                    class="btn-xp"
                    type="submit"
                    name="actions[direction]"
                    value="left"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    "<"
                </button>
                <button
                    class="btn-xp"
                    type="submit"
                    name="actions[direction]"
                    value="down"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    "v"
                </button>
                <button
                    class="btn-xp"
                    type="submit"
                    name="actions[direction]"
                    value="right"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    ">"
                </button>
            </div>
        </ActionForm>
    }
}

#[component]
fn MachineStatus(set_machine_state: WriteSignal<MachineState>) -> impl IntoView {
    let interval = use_interval(1000u64);
    let status = Resource::new(
        move || interval.counter.get(),
        |_| async { get_status().await },
    );
    let (displayed_status, set_displayed_status) = signal::<Option<String>>(None);

    Effect::new(move |_| {
        if let Some(result) = status.get() {
            match result {
                Ok(machine_status) => {
                    set_machine_state.set(machine_status.state.clone());
                    set_displayed_status.set(Some(format!(
                        "{:?} | X: {:.3} mm, Y: {:.3} mm, Z: {:.3} mm",
                        machine_status.state,
                        machine_status.x,
                        machine_status.y,
                        machine_status.z[0]
                    )));
                }
                Err(error) => {
                    set_displayed_status.set(Some(format!("Unable to load status: {error}")))
                }
            }
        }
    });

    view! {
        <div class=move || match displayed_status.get().as_deref() {
            Some(status) if status.contains("Ready") => {
                "rounded bg-green-700 px-2 py-1 text-sm font-medium"
            }
            Some(status) if status.contains("RunningJob") => {
                "rounded bg-yellow-700 px-2 py-1 text-sm font-medium"
            }
            _ => "rounded bg-red-600 px-2 py-1 text-sm font-medium",
        }>{move || displayed_status.get().unwrap_or_else(|| "Loading...".into())}</div>
    }
}

/// What a row of the placement list is: a component to place, or a
/// fiducial the machine uses to work out where the board sits.
#[derive(Clone, Debug, PartialEq)]
pub enum ComponentKind {
    Part,
    Fiducial,
}

/// What a column of the placement list holds.
#[derive(Clone, Debug, PartialEq)]
pub enum PlacementColumn {
    Reference,
    Value,
    X,
    Y,
    Rotation,
    Footprint,
    Description,
    Side,
    Other(String),
}

/// CSV writers quote fields that may contain the separator, so a cell
/// can arrive as `"C_0402_1005Metric"` with the quotes as part of the
/// value. Strip one matching pair (and doubled-quote escapes) so names
/// and MPNs don't carry quote characters into the library and the job
/// file.
fn unquote_csv(value: &str) -> String {
    let trimmed = value.trim();
    if trimmed.len() >= 2 && trimmed.starts_with('"') && trimmed.ends_with('"') {
        trimmed[1..trimmed.len() - 1].replace("\"\"", "\"")
    } else {
        trimmed.to_string()
    }
}

impl PlacementColumn {
    fn label(&self) -> &str {
        match self {
            Self::Reference => "Reference",
            Self::Value => "Value",
            Self::X => "X",
            Self::Y => "Y",
            Self::Rotation => "Rotation",
            Self::Footprint => "Footprint",
            Self::Description => "Description",
            Self::Side => "Side",
            Self::Other(name) => name,
        }
    }

    fn from_header(header: &str) -> Self {
        let normalized = header.trim().to_ascii_lowercase().replace(['_', '-'], " ");

        match normalized.as_str() {
            "reference" | "ref" | "designator" | "component" | "part" => Self::Reference,
            "value" | "val" | "comment" => Self::Value,
            "x" | "posx" | "x position" | "mid x" | "midx" | "midpoint x" => Self::X,
            "y" | "posy" | "y position" | "mid y" | "midy" | "midpoint y" => Self::Y,
            "rotation" | "rot" | "angle" => Self::Rotation,
            "footprint" | "package" => Self::Footprint,
            "description" | "desc" => Self::Description,
            "side" | "layer" => Self::Side,
            _ => Self::Other(header.trim().to_string()),
        }
    }
}

/// Columns a fresh list starts with: the ones every placement file has.
fn default_columns() -> Vec<PlacementColumn> {
    vec![
        PlacementColumn::Reference,
        PlacementColumn::Value,
        PlacementColumn::X,
        PlacementColumn::Y,
        PlacementColumn::Rotation,
    ]
}

#[component]
fn PlacementEditor(
    columns: ReadSignal<Vec<PlacementColumn>>,
    set_columns: WriteSignal<Vec<PlacementColumn>>,
    rows: ReadSignal<Vec<Vec<String>>>,
    set_rows: WriteSignal<Vec<Vec<String>>>,
    kinds: ReadSignal<Vec<ComponentKind>>,
    set_kinds: WriteSignal<Vec<ComponentKind>>,
    plan: impl Fn(bool) + Copy + 'static,
    plan_note: ReadSignal<Option<String>>,
) -> impl IntoView {
    let set_camera_enabled = use_context::<WriteSignal<bool>>().expect("camera state context");
    let (context_menu, set_context_menu) = signal::<Option<(usize, i32, i32, bool)>>(None);
    let import_input = NodeRef::<Input>::new();
    // Parts library, fetched once per mount, to offer footprint picks.
    let library = expect_context::<AppData>().snapshot;
    let library_footprints = move || {
        let mut names: Vec<String> = library
            .get()
            .and_then(|result| result.ok())
            .map(|summary| {
                summary
                    .parts
                    .iter()
                    .map(|part| part.footprint.clone())
                    .collect()
            })
            .unwrap_or_default();
        names.sort();
        names.dedup();
        names
    };
    // Import wizard state.
    let (show_import, set_show_import) = signal(false);
    let (import_text, set_import_text) = signal(String::new());
    let (delimiter, set_delimiter) = signal("auto".to_string());
    let (parsed, set_parsed) =
        signal::<Option<(Vec<PlacementColumn>, Vec<Vec<String>>)>>(None);
    let (import_error, set_import_error) = signal::<Option<String>>(None);
    // Feedback for list-wide actions (clear) that don't go through the
    // plan pipeline, which owns `plan_note`.
    let (clear_note, set_clear_note) = signal::<Option<String>>(None);

    // CPH tracker: how the run behind the job is pacing while parts land.
    // Polled independently of the editor's own plan pipeline.
    let (run_status, set_run_status) = signal::<Option<JobRunStatus>>(None);
    let status_interval = use_interval(2000u64);
    Effect::new(move |_| {
        let _ = status_interval.counter.get();
        spawn_local(async move {
            if let Ok(status) = job_status().await {
                set_run_status.set(Some(status));
            }
        });
    });
    let cph_summary = move || {
        let status = run_status.get()?;
        let mut line = String::new();
        if status.placement_count > 0 {
            line.push_str(&format!(
                "{}/{} placed",
                status.placed_count, status.placement_count
            ));
        }
        if let Some(live) = status.live_cph {
            line.push_str(&format!(" · {live:.0} CPH"));
        }
        if let Some(elapsed) = status.elapsed_secs {
            let mins = (elapsed / 60.0).floor() as u64;
            let secs = (elapsed % 60.0) as u64;
            line.push_str(&format!(" · {mins:02}:{secs:02}"));
        }
        if line.is_empty() { None } else { Some(line) }
    };

    // Live validation of the placement table so bad rows are caught
    // before they are planned into a job.
    let validation = Memo::new(move |_| {
        let columns = columns.get();
        let index_of = |wanted: &PlacementColumn| columns.iter().position(|c| c == wanted);
        let reference_index = index_of(&PlacementColumn::Reference);
        let value_index = index_of(&PlacementColumn::Value);
        let footprint_index = index_of(&PlacementColumn::Footprint);
        let x_index = index_of(&PlacementColumn::X);
        let y_index = index_of(&PlacementColumn::Y);
        let mut issues: Vec<String> = Vec::new();
        let mut seen = std::collections::HashSet::new();
        for (row_index, row) in rows.get().iter().enumerate() {
            let cell = |index: Option<usize>| {
                index
                    .and_then(|index| row.get(index))
                    .map(|value| value.trim().to_string())
                    .unwrap_or_default()
            };
            // Completely blank rows are just empty seats, not errors.
            if row.iter().all(|value| value.trim().is_empty()) {
                continue;
            }
            let number = |value: &str| {
                value.is_empty()
                    || value.trim().replace(',', ".").parse::<f64>().is_ok()
            };
            let part_name = {
                let from_footprint = cell(footprint_index);
                if from_footprint.is_empty() {
                    cell(value_index)
                } else {
                    from_footprint
                }
            };
            if part_name.is_empty() {
                issues.push(format!(
                    "Row {}: no Value or Footprint — the planner cannot tell what part this is",
                    row_index + 1
                ));
            }
            let reference = cell(reference_index);
            if reference.is_empty() {
                issues.push(format!("Row {}: missing reference", row_index + 1));
            } else if !seen.insert(reference.clone()) {
                issues.push(format!(
                    "Row {}: duplicate reference {reference}",
                    row_index + 1
                ));
            }
            let x = cell(x_index);
            let y = cell(y_index);
            if x.is_empty() || y.is_empty() {
                issues.push(format!("Row {}: missing X or Y", row_index + 1));
            } else if !number(&x) || !number(&y) {
                issues.push(format!("Row {}: X/Y is not a number", row_index + 1));
            }
        }
        issues
    });

    // Close the context menu when clicking anywhere outside of it.
    // The `click` listener runs in the bubble phase, so the menu can let the
    // click through for its own buttons. The `contextmenu` listener runs in
    // the capture phase so right-clicking another row re-opens the menu at
    // the new position instead of closing it right after it is opened.
    Effect::new(move |_| {
        if context_menu.get().is_none() {
            return;
        }

        let close_if_outside = move |event: web_sys::MouseEvent| {
            let inside_menu = event
                .target()
                .as_ref()
                .and_then(|target| target.dyn_ref::<web_sys::Element>())
                .and_then(|element| element.closest("[data-context-menu]").ok().flatten())
                .is_some();

            if !inside_menu {
                set_context_menu.set(None);
            }
        };

        let _ = use_event_listener(window(), leptos::ev::click, close_if_outside.clone());
        let _ = use_event_listener_with_options(
            window(),
            leptos::ev::contextmenu,
            close_if_outside,
            UseEventListenerOptions::default().capture(true),
        );
    });

    let placement_fields = [
        PlacementColumn::Reference,
        PlacementColumn::Value,
        PlacementColumn::X,
        PlacementColumn::Y,
        PlacementColumn::Rotation,
        PlacementColumn::Footprint,
        PlacementColumn::Description,
        PlacementColumn::Side,
    ];

    // Parse pasted or uploaded CSV/TSV text into columns + rows without
    // touching the live table, so the operator can preview first.
    let parse_import = move |text: &str, delim: &str| {
        let mut lines = text.lines().filter(|line| !line.trim().is_empty());
        let Some(header_line) = lines.next() else {
            return Err("The data is empty.".to_string());
        };
        let separator = match delim {
            "tab" => '\t',
            "comma" => ',',
            "semicolon" => ';',
            // Auto: the first separator character found in the header wins.
            _ => {
                if header_line.contains('\t') {
                    '\t'
                } else if header_line.contains(';') {
                    ';'
                } else {
                    ','
                }
            }
        };
        let headers = header_line
            .split(separator)
            .map(|header| PlacementColumn::from_header(&unquote_csv(header)))
            .collect::<Vec<_>>();
        let imported_rows = lines
            .map(|line| {
                let mut row = line
                    .split(separator)
                    .map(|value| unquote_csv(value))
                    .collect::<Vec<_>>();
                row.resize(headers.len(), String::new());
                row
            })
            .collect::<Vec<_>>();
        Ok((headers, imported_rows))
    };
    let run_parse = move |_| {
        match parse_import(import_text.get().as_str(), delimiter.get().as_str()) {
            Ok(result) => {
                set_import_error.set(None);
                set_parsed.set(Some(result));
            }
            Err(message) => {
                set_parsed.set(None);
                set_import_error.set(Some(message));
            }
        }
    };
    let apply_import = move |_| {
        if let Some((imported_columns, imported_rows)) = parsed.get() {
            set_columns.set(imported_columns);
            set_kinds
                .set(imported_rows.iter().map(|_| ComponentKind::Part).collect());
            set_rows.set(imported_rows);
            set_show_import.set(false);
            set_import_text.set(String::new());
            set_parsed.set(None);
        }
    };
    // Load a file's text into the wizard for preview before applying.
    let import_file = move |event| {
        let input = event_target::<HtmlInputElement>(&event);
        let Some(file) = input.files().and_then(|files| files.get(0)) else {
            return;
        };

        spawn_local(async move {
            let Ok(text) = JsFuture::from(file.text()).await else {
                return;
            };
            let Some(text) = text.as_string() else {
                return;
            };
            set_import_text.set(text);
            set_show_import.set(true);
        });
    };

    let add_row = move |_| {
        let column_count = columns.get().len();
        set_rows.update(|rows| rows.push(vec![String::new(); column_count]));
        set_kinds.update(|kinds| kinds.push(ComponentKind::Part));
    };
    let remove_row = move |index| {
        set_rows.update(|rows| {
            if index < rows.len() {
                rows.remove(index);
            }
        });
        set_kinds.update(|kinds| {
            if index < kinds.len() {
                kinds.remove(index);
            }
        });
    };
    let add_column = move |_| {
        let index = columns.get().len() + 1;
        set_columns
            .update(|columns| columns.push(PlacementColumn::Other(format!("Column {index}"))));
        set_rows.update(|rows| rows.iter_mut().for_each(|row| row.push(String::new())));
    };
    let remove_column = move |index| {
        if columns.get().len() <= 1 {
            return;
        }
        set_columns.update(|columns| {
            if index < columns.len() {
                columns.remove(index);
            }
        });
        set_rows.update(|rows| {
            rows.iter_mut().for_each(|row| {
                if index < row.len() {
                    row.remove(index);
                }
            });
        });
    };

    view! {
        <section class="flex min-h-0 flex-col" on:click=move |_| { set_context_menu.set(None); }>
            <div class="flex flex-wrap items-center justify-between gap-2">
                <h2 class="text-lg font-semibold">"PCB placement list"</h2>
                {move || {
                    cph_summary().map(|line| {
                        view! {
                            <span class="rounded border border-slate-600 bg-slate-700 px-2 py-0.5 text-xs text-slate-200">
                                {line}
                            </span>
                        }
                    })
                }}
                <div class="flex flex-wrap gap-2">
                    <button
                        class="btn-xp"
                        type="button"
                        on:click=move |_| {
                            set_import_text.set(String::new());
                            set_parsed.set(None);
                            set_import_error.set(None);
                            set_show_import.set(true);
                        }
                    >
                        "Import CSV/TSV"
                    </button>
                    <button class="btn-xp" type="button" on:click=add_row>
                        "Add component"
                    </button>
                    <button class="btn-xp" type="button" on:click=add_column>
                        "Add column"
                    </button>
                    <button
                        class="btn-xp !bg-rose-900"
                        type="button"
                        on:click=move |_| {
                            spawn_local(async move {
                                match clear_placements().await {
                                    Ok(message) => {
                                        set_rows.set(Vec::new());
                                        set_kinds.set(Vec::new());
                                        set_clear_note.set(Some(message));
                                    }
                                    Err(error) => {
                                        set_clear_note.set(Some(format!("Clear failed: {error}")));
                                    }
                                }
                            });
                        }
                        title="Wipe the working job's placement list (feeders, banks and head are kept)"
                    >
                        "Clear list"
                    </button>
                    <button
                        class="btn-xp !bg-emerald-800"
                        type="button"
                        on:click=move |_| plan(true)
                        title="Turn this list into a planned, runnable job"
                    >
                        "Plan job →"
                    </button>
                </div>
            </div>
            {move || {
                let note = clear_note.get().or_else(|| plan_note.get());
                note.map(|note| view! {
                    <p class="mt-2 whitespace-pre-line rounded border border-slate-600 bg-slate-800 p-2 text-xs text-slate-300">{note}</p>
                })
            }}
            <Show when=move || !validation.get().is_empty()>
                <div class="mt-2 rounded border border-yellow-600 bg-yellow-900/30 p-2 text-xs text-yellow-200">
                    <p class="font-semibold">
                        {move || format!("{} data issue(s) — these rows will be skipped when planning:", validation.get().len())}
                    </p>
                    <ul class="mt-1 max-h-32 list-disc overflow-auto pl-4">
                        {move || {
                            validation
                                .get()
                                .into_iter()
                                .take(20)
                                .map(|issue| view! { <li>{issue}</li> })
                                .collect_view()
                        }}
                    </ul>
                </div>
            </Show>
            <div class="mt-3 max-h-[calc(100vh-12rem)] overflow-auto">
                <table class="min-w-full border-collapse text-left text-xs">
                    <thead>
                        <tr>
                            {move || {
                                columns
                                    .get()
                                    .into_iter()
                                    .enumerate()
                                    .map(|(index, column)| {
                                        let column_label = column.label().to_string();
                                        let is_other = matches!(column, PlacementColumn::Other(_));
                                        let custom_label = if is_other
                                            && !placement_fields
                                                .iter()
                                                .any(|field| field.label() == column_label)
                                        {
                                            Some(column_label.clone())
                                        } else {
                                            None
                                        };
                                        view! {
                                            <th
                                                class="border border-slate-600 p-1"
                                                on:contextmenu=move |event| {
                                                    event.prevent_default();
                                                    set_context_menu
                                                        .set(
                                                            Some((index, event.client_x(), event.client_y(), true)),
                                                        );
                                                }
                                            >
                                                <select
                                                    class="w-28 bg-slate-700 p-1"
                                                    on:change=move |event| {
                                                        let value = event_target_value(&event);
                                                        set_columns
                                                            .update(|columns| {
                                                                columns[index] = PlacementColumn::from_header(&value)
                                                            });
                                                    }
                                                >
                                                    {placement_fields
                                                        .iter()
                                                        .map(|field| {
                                                            let field_label = field.label().to_string();
                                                            let selected = column == *field;
                                                            view! {
                                                                <option value=field_label.clone() selected=selected>
                                                                    {field_label.clone()}
                                                                </option>
                                                            }
                                                        })
                                                        .collect_view()}
                                                    {custom_label
                                                        .map(|label| {
                                                            view! {
                                                                <option value=label.clone() selected=true>
                                                                    {label.clone()}
                                                                </option>
                                                            }
                                                        })}
                                                </select>
                                            </th>
                                        }
                                    })
                                    .collect_view()
                            }}
                        </tr>
                    </thead>
                    <tbody>
                        {move || {
                            rows.get()
                                .into_iter()
                                .enumerate()
                                .map(|(row_index, row)| {
                                    view! {
                                        <tr
                                            class=move || match kinds.get().get(row_index) {
                                                Some(ComponentKind::Fiducial) => "bg-blue-900/40",
                                                _ => "",
                                            }
                                            on:contextmenu=move |event| {
                                                event.prevent_default();
                                                set_context_menu
                                                    .set(
                                                        Some((row_index, event.client_x(), event.client_y(), false)),
                                                    );
                                            }
                                        >
                                            {row
                                                .into_iter()
                                                .enumerate()
                                                .map(|(column_index, value)| {
                                                    // Footprint cells get a dropdown fed
                                                    // by the parts library.
                                                    let is_footprint = columns
                                                        .get()
                                                        .get(column_index)
                                                        == Some(&PlacementColumn::Footprint);
                                                    if is_footprint {
                                                        let current = value.clone();
                                                        let options = library_footprints();
                                                        (view! {
                                                            <td class="border border-slate-600 p-1">
                                                                <select
                                                                    class="w-28 bg-slate-700 p-1"
                                                                    on:change=move |event| {
                                                                        let value = event_target_value(&event);
                                                                        set_rows
                                                                            .update(|rows| rows[row_index][column_index] = value);
                                                                    }
                                                                >
                                                                    <option value="" selected=current.is_empty()>"—"</option>
                                                                    {(!current.is_empty()
                                                                        && !options.contains(&current))
                                                                        .then(|| {
                                                                            view! {
                                                                                <option value=current.clone() selected=true>
                                                                                    {current.clone()}
                                                                                </option>
                                                                            }
                                                                        })}
                                                                    {options
                                                                        .into_iter()
                                                                        .map(|name| {
                                                                            let option_value = name.clone();
                                                                            view! {
                                                                                <option value=option_value.clone() selected=option_value == current>
                                                                                    {name}
                                                                                </option>
                                                                            }
                                                                        })
                                                                        .collect_view()}
                                                                </select>
                                                            </td>
                                                        })
                                                        .into_any()
                                                    } else {
                                                        (view! {
                                                            <td class="border border-slate-600 p-1">
                                                                <input
                                                                    class="w-24 bg-slate-700 p-1"
                                                                    value=value
                                                                    on:input=move |event| {
                                                                        let value = event_target_value(&event);
                                                                        set_rows
                                                                            .update(|rows| rows[row_index][column_index] = value);
                                                                    }
                                                                />
                                                            </td>
                                                        })
                                                        .into_any()
                                                    }
                                                })
                                                .collect_view()}
                                        </tr>
                                    }
                                })
                                .collect_view()
                        }}
                    </tbody>
                </table>
            </div>
            {move || {
                context_menu
                    .get()
                    .map(|(index, x, y, is_column)| {
                        view! {
                            <div
                                class="fixed z-[60] rounded border border-slate-500 bg-slate-700 p-1 shadow-lg"
                                style=format!("left: {x}px; top: {y}px;")
                                data-context-menu=""
                                on:click=move |event| {
                                    event.stop_propagation();
                                }
                            >
                                {(!is_column)
                                    .then(|| {
                                        view! {
                                            <>
                                                <button
                                                    class="block w-full px-3 py-1 text-left text-sm hover:bg-slate-600"
                                                    type="button"
                                                    on:click=move |_| {
                                                        set_kinds
                                                            .update(|kinds| {
                                                                if index < kinds.len() {
                                                                    kinds[index] = ComponentKind::Part;
                                                                }
                                                            });
                                                        set_context_menu.set(None);
                                                    }
                                                >
                                                    "Mark as part"
                                                </button>
                                                <button
                                                    class="block w-full px-3 py-1 text-left text-sm hover:bg-slate-600"
                                                    type="button"
                                                    on:click=move |_| {
                                                        set_kinds
                                                            .update(|kinds| {
                                                                if index < kinds.len() {
                                                                    kinds[index] = ComponentKind::Fiducial;
                                                                }
                                                            });
                                                        set_context_menu.set(None);
                                                    }
                                                >
                                                    "Mark as fiducial"
                                                </button>
                                            </>
                                        }
                                    })}
                                <button
                                    class="block w-full px-3 py-1 text-left text-sm hover:bg-slate-600"
                                    type="button"
                                    on:click=move |_| {
                                        if is_column {
                                            remove_column(index);
                                        } else {
                                            remove_row(index);
                                        }
                                        set_context_menu.set(None);
                                    }
                                >
                                    {if is_column { "Delete column" } else { "Delete row" }}
                                </button>
                                <button
                                    class="block w-full px-3 py-1 text-left text-sm hover:bg-slate-600"
                                    type="button"
                                    on:click=move |_| set_context_menu.set(None)
                                >
                                    "Cancel"
                                </button>
                            </div>
                        }
                    })
            }}

            // Import wizard: paste or pick a file, preview it, then apply.
            <Show when=move || show_import.get()>
                <div class="fixed inset-0 z-[65] flex items-center justify-center bg-black/60 p-4">
                    <div class="flex max-h-[90vh] w-full max-w-2xl flex-col overflow-auto rounded border border-slate-500 bg-slate-800 p-4 text-white shadow-xl">
                        <h2 class="text-lg font-semibold">"Import CSV/TSV"</h2>
                        <p class="mt-1 text-xs text-slate-400">
                            "Pick a file or paste placement data, check the preview, then apply. \
                             Column meanings are detected from the header and can be changed in the table."
                        </p>
                        <div class="mt-3 flex flex-wrap items-center gap-2 text-sm">
                            <button
                                class="btn-xp"
                                type="button"
                                on:click=move |_| {
                                    // Stop image decoding before opening the native file picker.
                                    set_camera_enabled.set(false);
                                    if let Some(image) = web_sys::window()
                                        .and_then(|window| window.document())
                                        .and_then(|document| document.get_element_by_id("camera-preview"))
                                        .and_then(|element| element.dyn_into::<HtmlImageElement>().ok())
                                    {
                                        image.set_src("");
                                    }
                                    if let Some(input) = import_input.get() {
                                        let _ = input.click();
                                    }
                                }
                            >
                                "Choose file…"
                            </button>
                            <input
                                node_ref=import_input
                                class="hidden"
                                type="file"
                                accept=".csv,.tsv,text/csv,text/tab-separated-values"
                                on:change=move |event| {
                                    import_file(event);
                                    set_camera_enabled.set(true);
                                }
                                on:blur=move |_| set_camera_enabled.set(true)
                            />
                            <label class="text-xs">"Separator"
                                <select
                                    class="ml-1 bg-slate-700 p-1"
                                    on:change=move |event| set_delimiter.set(event_target_value(&event))
                                >
                                    <option value="auto">"Auto"</option>
                                    <option value="comma">"Comma"</option>
                                    <option value="tab">"Tab"</option>
                                    <option value="semicolon">"Semicolon"</option>
                                </select>
                            </label>
                            <button class="btn-xp" type="button" on:click=run_parse>
                                "Parse preview"
                            </button>
                        </div>
                        <textarea
                            class="mt-2 h-40 w-full rounded border border-slate-600 bg-slate-900 p-2 font-mono text-xs"
                            placeholder="Reference,Value,Footprint,X,Y,Rotation"
                            prop:value=move || import_text.get()
                            on:input=move |event| set_import_text.set(event_target_value(&event))
                        ></textarea>
                        {move || import_error.get().map(|message| view! {
                            <p class="mt-2 text-xs text-red-300">{message}</p>
                        })}
                        <Show when=move || parsed.get().is_some()>
                            {move || {
                                let (imported_columns, imported_rows) = parsed.get().expect("checked");
                                let header_labels: Vec<String> = imported_columns
                                    .iter()
                                    .map(|column| column.label().to_string())
                                    .collect();
                                view! {
                                    <div class="mt-2 rounded border border-slate-600 p-2 text-xs">
                                        <p class="font-semibold text-slate-300">
                                            {format!("{} rows, {} columns detected", imported_rows.len(), imported_columns.len())}
                                        </p>
                                        <div class="mt-1 max-h-40 overflow-auto">
                                            <table class="min-w-full text-left">
                                                <thead>
                                                    <tr>
                                                        {header_labels
                                                            .iter()
                                                            .map(|label| view! {
                                                                <th class="border border-slate-600 p-1">{label.clone()}</th>
                                                            })
                                                            .collect_view()}
                                                    </tr>
                                                </thead>
                                                <tbody>
                                                    {imported_rows
                                                        .iter()
                                                        .take(6)
                                                        .map(|row| {
                                                            view! {
                                                                <tr>
                                                                    {row
                                                                        .iter()
                                                                        .map(|value| view! {
                                                                            <td class="border border-slate-600 p-1">{value.clone()}</td>
                                                                        })
                                                                        .collect_view()}
                                                                </tr>
                                                            }
                                                        })
                                                        .collect_view()}
                                                </tbody>
                                            </table>
                                        </div>
                                    </div>
                                }
                            }}
                        </Show>
                        <div class="mt-3 flex justify-end gap-2">
                            <button
                                class="btn-xp"
                                type="button"
                                on:click=move |_| set_show_import.set(false)
                            >
                                "Cancel"
                            </button>
                            <button
                                class="btn-xp !bg-emerald-700"
                                type="button"
                                on:click=apply_import
                                disabled=move || parsed.get().is_none()
                            >
                                "Apply import"
                            </button>
                        </div>
                    </div>
                </div>
            </Show>
        </section>
    }
}

/// Process view: visualize the machine's cycle and step through it.
///
/// The server turns the planned job into an ordered step list (fiducials
/// first, then one head-load at a time: every pick of that trip, one camera
/// stop, then the trip's placements). The board map draws every placement,
/// the feeder rail draws every pick location, and the cycle tracker lets
/// the operator walk the timeline forwards into the future steps or
/// backwards over the past ones. While a job runs, "follow" keeps the
/// cursor on the step the machine is executing right now.
#[component]
fn ProcessVisualizer() -> impl IntoView {
    let (data, set_data) = signal::<Option<StepsResponse>>(None);
    let (cursor, set_cursor) = signal(0usize);
    let (follow, set_follow) = signal(true);
    let (error, set_error) = signal::<Option<String>>(None);

    let reload = move || {
        spawn_local(async move {
            match get_job_steps().await {
                Ok(response) => {
                    let live = response.live_index;
                    set_data.set(Some(response));
                    if follow.get_untracked() {
                        if let Some(index) = live {
                            set_cursor.set(index);
                        }
                    }
                }
                Err(e) => set_error.set(Some(format!("{e}"))),
            }
        })
    };

    // Opening the process view re-optimises the job first, so the step
    // list reflects the current machine setup. While a run is live the
    // server refuses the request and the view simply shows the stored
    // plan — a production job is never re-optimised while it runs.
    spawn_local(async move {
        if reoptimize_job().await.is_ok() {
            reload();
        }
    });

    // Keep the view current while a job runs.
    let interval = use_interval(1000u64);
    Effect::new(move |_| {
        let _ = interval.counter.get();
        reload();
    });

    let steps = move || data.get().map(|d| d.steps).unwrap_or_default();
    let total = move || steps().len();
    let clamped = move |index: usize| {
        if total() == 0 {
            0
        } else {
            index.min(total() - 1)
        }
    };
    let current = move || clamped(cursor.get());
    let step_at = move |index: usize| steps().get(index).cloned();
    let done_count = move || steps().iter().filter(|s| s.done).count();

    let go = move |delta: i32| {
        let next = cursor.get() as i64 + delta as i64;
        set_cursor.set(clamped(next.max(0) as usize));
    };

    // Map helpers: project machine millimetres into an SVG view box.
    let project = move |items: Vec<(f64, f64)>, w: f64, h: f64, pad: f64| -> Vec<(f64, f64, f64, f64)> {
        let min_x = items.iter().map(|i| i.0).fold(f64::INFINITY, f64::min);
        let max_x = items.iter().map(|i| i.0).fold(f64::NEG_INFINITY, f64::max);
        let min_y = items.iter().map(|i| i.1).fold(f64::INFINITY, f64::min);
        let max_y = items.iter().map(|i| i.1).fold(f64::NEG_INFINITY, f64::max);
        let span_x = (max_x - min_x).max(1.0);
        let span_y = (max_y - min_y).max(1.0);
        let scale = ((w - 2.0 * pad) / span_x).min((h - 2.0 * pad) / span_y);
        let off_x = (w - span_x * scale) / 2.0;
        let off_y = (h - span_y * scale) / 2.0;
        items
            .into_iter()
            .map(|(x, y)| {
                (
                    off_x + (x - min_x) * scale,
                    off_y + (max_y - y) * scale, // SVG y grows downwards
                    x,
                    y,
                )
            })
            .collect()
    };

    // Placement marks for the board map (machine coordinates -> svg).
    let board_marks = move || {
        let all = steps();
        let places: Vec<StepInfo> = all.iter().filter(|s| s.kind == "place").cloned().collect();
        if places.is_empty() {
            return Vec::new();
        }
        let raw: Vec<(f64, f64)> = places.iter().map(|s| (s.x, s.y)).collect();
        let projected = project(raw, 240.0, 170.0, 12.0);
        places
            .iter()
            .zip(projected)
            .map(|(step, (cx, cy, _mx, _my))| (step.clone(), cx, cy))
            .collect::<Vec<_>>()
    };
    // Pick marks for the feeder rail, one per pick step.
    let feeder_marks = move || {
        let all = steps();
        let picks: Vec<StepInfo> = all.iter().filter(|s| s.kind == "pick").cloned().collect();
        if picks.is_empty() {
            return Vec::new();
        }
        let raw: Vec<(f64, f64)> = picks.iter().map(|s| (s.x, s.y)).collect();
        let projected = project(raw, 240.0, 170.0, 12.0);
        picks
            .iter()
            .zip(projected)
            .map(|(step, (cx, cy, _mx, _my))| (step.clone(), cx, cy))
            .collect::<Vec<_>>()
    };

    let mark_class = move |step: &StepInfo, is_current: bool| -> &'static str {
        if is_current {
            "fill-amber-400 stroke-amber-200"
        } else if step.skipped {
            "fill-slate-600 stroke-slate-500"
        } else if step.done {
            "fill-emerald-500 stroke-emerald-300"
        } else {
            "fill-sky-600 stroke-sky-400"
        }
    };

    view! {
        <section class="flex min-h-0 flex-col gap-3">
            <div class="flex flex-wrap items-center justify-between gap-2">
                <h2 class="text-lg font-semibold">"Process view"</h2>
                <div class="flex items-center gap-3 text-xs text-slate-300">
                    <label class="flex items-center gap-1">
                        <input
                            type="checkbox"
                            prop:checked=move || follow.get()
                            on:change=move |event| set_follow.set(event_target_checked(&event))
                        />
                        "Follow machine"
                    </label>
                    <span>{move || format!("{} / {} steps done", done_count(), total())}</span>
                    {move || {
                        let data = data.get();
                        let estimated = data
                            .as_ref()
                            .map(|d| d.estimated_cph)
                            .unwrap_or(0.0);
                        let live = data.as_ref().and_then(|d| d.live_cph);
                        let elapsed = data.as_ref().and_then(|d| d.elapsed_secs);
                        let mut line = String::new();
                        if estimated > 0.0 {
                            line.push_str(&format!("est {estimated:.0} CPH"));
                        }
                        if let Some(live) = live {
                            if !line.is_empty() {
                                line.push_str(" · ");
                            }
                            line.push_str(&format!("live {live:.0} CPH"));
                        }
                        if let Some(elapsed) = elapsed {
                            let mins = (elapsed / 60.0).floor() as u64;
                            let secs = (elapsed % 60.0) as u64;
                            line.push_str(&format!(" · {mins:02}:{secs:02}"));
                        }
                        view! { <span>{line}</span> }
                    }}
                </div>
            </div>

            {move || error.get().map(|e| view! {
                <p class="rounded border border-red-500 bg-red-900/40 p-2 text-xs text-red-200">{e}</p>
            })}

            <div class="grid gap-3 md:grid-cols-2">
                // Board map: where each component goes.
                <div class="rounded border border-slate-600 bg-slate-800 p-2">
                    <p class="text-xs font-semibold text-slate-300">"Board placements"</p>
                    <svg viewBox="0 0 240 170" class="mt-1 w-full">
                        <rect x="6" y="6" width="228" height="158" rx="4" class="fill-slate-900 stroke-slate-600" />
                        {move || {
                            board_marks()
                                .into_iter()
                                .map(|(step, cx, cy)| {
                                    let is_current = step.index == current();
                                    view! {
                                        <g
                                            class="cursor-pointer"
                                            on:click=move |_| set_cursor.set(step.index)
                                        >
                                            <circle
                                                cx=cx cy=cy r=4.5
                                                class=mark_class(&step, is_current)
                                                stroke-width=if is_current { 2.0 } else { 1.0 }
                                            />
                                            <title>{format!("{} {} ({:.1}, {:.1})", step.refdes, step.mpn, step.x, step.y)}</title>
                                        </g>
                                    }
                                })
                                .collect_view()
                        }}
                    </svg>
                </div>
                // Feeder rail: where each component is picked from.
                <div class="rounded border border-slate-600 bg-slate-800 p-2">
                    <p class="text-xs font-semibold text-slate-300">"Feeder rail (picks)"</p>
                    <svg viewBox="0 0 240 170" class="mt-1 w-full">
                        <rect x="6" y="6" width="228" height="158" rx="4" class="fill-slate-900 stroke-slate-600" />
                        {move || {
                            feeder_marks()
                                .into_iter()
                                .map(|(step, cx, cy)| {
                                    let is_current = step.index == current();
                                    view! {
                                        <g
                                            class="cursor-pointer"
                                            on:click=move |_| set_cursor.set(step.index)
                                        >
                                            <rect
                                                x=(cx - 4.0) y=(cy - 4.0) width=8.0 height=8.0 rx=1.0
                                                class=mark_class(&step, is_current)
                                                stroke-width=if is_current { 2.0 } else { 1.0 }
                                            />
                                            <title>{format!("Slot {} {} ({:.1}, {:.1}) Z={:.2}", step.feeder_slot, step.mpn, step.x, step.y, step.z_mm)}</title>
                                        </g>
                                    }
                                })
                                .collect_view()
                        }}
                    </svg>
                </div>
            </div>

            // Cycle tracker.
            <div class="rounded border border-slate-600 bg-slate-800 p-3">
                <div class="flex flex-wrap items-center gap-2">
                    <button class="btn-xp !px-2" type="button" on:click=move |_| set_cursor.set(0) title="First step">
                        "⏮"
                    </button>
                    <button class="btn-xp !px-2" type="button" on:click=move |_| go(-1) title="Previous step">
                        "◀"
                    </button>
                    <input
                        class="min-w-40 flex-1"
                        type="range"
                        min="0"
                        max=move || total().saturating_sub(1)
                        prop:value=move || current()
                        on:input=move |event| {
                            if let Ok(value) = event_target_value(&event).parse::<usize>() {
                                set_cursor.set(value);
                            }
                        }
                    />
                    <button class="btn-xp !px-2" type="button" on:click=move |_| go(1) title="Next step">
                        "▶"
                    </button>
                    <button class="btn-xp !px-2" type="button" on:click=move |_| set_cursor.set(total().saturating_sub(1)) title="Last step">
                        "⏭"
                    </button>
                    <span class="text-xs text-slate-400">
                        {move || format!("Step {} / {}", current() + 1, total())}
                    </span>
                </div>

                // Timeline strip: a window of steps around the cursor.
                <div class="mt-3 flex gap-1 overflow-x-auto pb-1">
                    {move || {
                        let all = steps();
                        if all.is_empty() {
                            return view! { <p class="text-xs text-slate-400">"No job planned yet. Plan a job to see its cycle here."</p> }.into_any();
                        }
                        let cur = current();
                        let start = cur.saturating_sub(5);
                        let end = (start + 11).min(all.len());
                        (start..end)
                            .map(|index| {
                                let step = &all[index];
                                let label = match step.kind.as_str() {
                                    "fiducial" => "◎",
                                    "pick" => "↑",
                                    "inspect" => "⌕",
                                    _ => "↓",
                                };
                                let chip_class = if index == cur {
                                    "border-amber-400 bg-amber-900/40 text-amber-200"
                                } else if step.done {
                                    "border-emerald-700 bg-emerald-900/30 text-emerald-300"
                                } else if step.skipped {
                                    "border-slate-700 bg-slate-800 text-slate-500"
                                } else {
                                    "border-slate-600 bg-slate-700 text-slate-300"
                                };
                                let name = if step.refdes.is_empty() {
                                    step.mpn.clone()
                                } else {
                                    step.refdes.clone()
                                };
                                view! {
                                    <button
                                        class=format!("shrink-0 rounded border px-2 py-1 text-left text-xs {chip_class}")
                                        type="button"
                                        on:click=move |_| set_cursor.set(index)
                                    >
                                        <span class="font-mono">{label}</span>
                                        " "
                                        {name}
                                        <span class="ml-1 text-slate-400">{if step.trip > 0 { format!("T{}", step.trip) } else { String::new() }}</span>
                                    </button>
                                }
                            })
                            .collect_view()
                            .into_any()
                    }}
                </div>

                // Detail card for the step under the cursor.
                {move || {
                    step_at(current()).map(|step| {
                        let kind_label = match step.kind.as_str() {
                            "fiducial" => "Locate fiducial",
                            "pick" => "Pick from feeder",
                            "inspect" => "Inspect at camera",
                            _ => "Place on board",
                        };
                        let state_label = if step.skipped {
                            "skipped"
                        } else if step.done {
                            "done"
                        } else {
                            "pending"
                        };
                        let trip_label = if step.trip > 0 {
                            format!("Head-load {}", step.trip)
                        } else {
                            "Fiducial phase".to_string()
                        };
                        let (mpn, footprint) = (step.mpn.clone(), step.footprint.clone());
                        let has_part = !mpn.is_empty();
                        let (feeder_slot, nozzle, rotation) =
                            (step.feeder_slot, step.nozzle, step.rotation);
                        view! {
                            <div class="mt-3 grid gap-x-4 gap-y-1 rounded border border-slate-600 bg-slate-700/50 p-2 text-xs sm:grid-cols-2">
                                <p><span class="font-semibold">{kind_label}</span>
                                    <span class="ml-2 text-slate-400">{state_label}</span>
                                </p>
                                <p>{trip_label}</p>
                                {has_part.then(move || view! {
                                    <p>"Part: " <span class="font-mono">{mpn.clone()}</span>
                                        " · footprint " <span class="font-mono">{footprint.clone()}</span>
                                    </p>
                                })}
                                {has_part.then(move || view! {
                                    <p>"Feeder slot " {feeder_slot}
                                        " · nozzle " {nozzle}
                                        " · rotation " {format!("{:.0}°", rotation)}
                                    </p>
                                })}
                                <p>"X " {format!("{:.2}", step.x)} " · Y " {format!("{:.2}", step.y)}
                                    " · Z " {format!("{:.2}", step.z_mm)} " mm"
                                </p>
                            </div>
                        }
                    })
                }}
            </div>

            <div class="flex flex-wrap gap-4 text-xs text-slate-400">
                <span><span class="mr-1 inline-block h-2 w-2 rounded-full bg-emerald-500"></span>"done"</span>
                <span><span class="mr-1 inline-block h-2 w-2 rounded-full bg-amber-400"></span>"current"</span>
                <span><span class="mr-1 inline-block h-2 w-2 rounded-full bg-sky-600"></span>"pending"</span>
                <span><span class="mr-1 inline-block h-2 w-2 rounded-full bg-slate-600"></span>"skipped"</span>
            </div>
        </section>
    }
}

#[component]
fn BoardHeader(
    board_id: ReadSignal<String>,
    set_board_id: WriteSignal<String>,
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
) -> impl IntoView {
    view! {
        <header class="mb-3 flex flex-wrap items-center gap-4 rounded border border-slate-600 bg-slate-700 p-2">
            <div>
                <label class="mr-2 text-sm font-semibold" for="board-id">
                    "Board ID"
                </label>
                <input
                    id="board-id"
                    class="bg-slate-800 px-2 py-1 text-sm"
                    type="text"
                    value=move || board_id.get()
                    on:input=move |event| set_board_id.set(event_target_value(&event))
                />
            </div>
            <TrackController machine_state=machine_state set_last_action=set_last_action />
        </header>
    }
}

#[component]
fn TrackController(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
) -> impl IntoView {
    let set_width_action = ServerAction::<RunAction>::new();
    let reset_action = ServerAction::<RunAction>::new();

    Effect::new(move |_| {
        if let Some(result) = set_width_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(()) => "Track width set successfully".to_string(),
                Err(error) => format!("Track width failed: {error}"),
            }));
        }
    });
    Effect::new(move |_| {
        if let Some(result) = reset_action.value().get() {
            set_last_action.set(Some(match result {
                Ok(()) => "Track reset successfully".to_string(),
                Err(error) => format!("Track reset failed: {error}"),
            }));
        }
    });

    view! {
        <div class="flex flex-wrap items-center gap-2">
            <ActionForm action=set_width_action>
                <input type="hidden" name="action" value="track_set_width" />
                <label class="mr-1 text-sm font-semibold" for="track-width">
                    "Track Width (mm)"
                </label>
                <input
                    id="track-width"
                    class="h-7 w-20 px-1 text-sm"
                    type="number"
                    name="actions[units]"
                    value="0"
                    min="0"
                />
                <button
                    class="btn-xp ml-2"
                    type="submit"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    "Set"
                </button>
            </ActionForm>
            <ActionForm action=reset_action>
                <input type="hidden" name="action" value="track_reset" />
                <button
                    class="btn-xp"
                    type="submit"
                    disabled=move || machine_state.get() != MachineState::Ready
                >
                    "Reset"
                </button>
            </ActionForm>
        </div>
    }
}

#[component]
fn BoardEditor(
    machine_state: ReadSignal<MachineState>,
    set_last_action: WriteSignal<Option<String>>,
    tab: ReadSignal<String>,
    set_tab: WriteSignal<String>,
) -> impl IntoView {
    let (board_id, set_board_id) = signal(String::new());
    // Writer half of the plan context: used to tell the parts panel to
    // refresh after a plan without triggering another plan.
    let set_plan_cmd =
        use_context::<WriteSignal<(u64, String)>>().expect("plan command writer");
    let plan_cmd = use_context::<ReadSignal<(u64, String)>>().expect("plan command context");
    // The placement list and the job manager are two views of the same
    // board, so they share the list itself and the board ID.
    let (columns, set_columns) = signal(default_columns());
    let (rows, set_rows) = signal::<Vec<Vec<String>>>(vec![]);
    let (kinds, set_kinds) = signal::<Vec<ComponentKind>>(vec![]);
    // Planning lives here at the board level so both the placement list
    // and the job manager can trigger it and see its result.
    let (plan_note, set_plan_note) = signal::<Option<String>>(None);
    // Bumped after every successful plan so the job manager refetches.
    let (job_version, set_job_version) = signal(0u64);
    // Bumped only when a job is LOADED from the library (and once at
    // mount), unlike `job_version` which every plan also bumps — the
    // placement grid must not be rewritten while the operator edits it.
    let (loaded_version, set_loaded_version) = signal(0u64);

    // Build the rows the planner needs from what is on screen.
    //
    // Cells are free text, so every field is parsed here and anything
    // unparseable is left out; the server reports how many rows it had
    // to skip rather than failing the whole plan.
    let column_index_of = move |columns: &[PlacementColumn], wanted: &PlacementColumn| {
        columns.iter().position(|column| column == wanted)
    };
    // Tolerate the formats real spreadsheets produce: "12.5", "12,5",
    // "12.5 mm", stray spaces.
    let parse_number = |raw: &str| -> Option<f64> {
        let cleaned = raw
            .trim()
            .trim_end_matches("mm")
            .trim()
            .replace(',', ".");
        if cleaned.is_empty() {
            None
        } else {
            cleaned.parse::<f64>().ok()
        }
    };

    // Every reason a row will be dropped by the planner, in plain
    // words, so "9 rows skipped" never happens silently again.
    let row_problems = move || -> Vec<String> {
        let columns = columns.get();
        let reference_index = column_index_of(&columns, &PlacementColumn::Reference);
        let value_index = column_index_of(&columns, &PlacementColumn::Value);
        let footprint_index = column_index_of(&columns, &PlacementColumn::Footprint);
        let x_index = column_index_of(&columns, &PlacementColumn::X);
        let y_index = column_index_of(&columns, &PlacementColumn::Y);
        let mut problems = Vec::new();
        for (row_index, row) in rows.get().iter().enumerate() {
            let cell = |index: Option<usize>| {
                index
                    .and_then(|index| row.get(index))
                    .map(|value| value.trim().to_string())
                    .unwrap_or_default()
            };
            // Completely blank rows are just empty seats, not errors.
            if row.iter().all(|value| value.trim().is_empty()) {
                continue;
            }
            let label = cell(reference_index);
            let label = if label.is_empty() {
                format!("row {}", row_index + 1)
            } else {
                format!("row {} ({label})", row_index + 1)
            };
            // The part name: footprint column, falling back to value.
            let part_name = {
                let from_footprint = cell(footprint_index);
                if from_footprint.is_empty() {
                    cell(value_index)
                } else {
                    from_footprint
                }
            };
            if part_name.is_empty() {
                problems.push(format!(
                    "{label}: no Value or Footprint — the planner cannot tell what part this is"
                ));
            }
            let x_raw = cell(x_index);
            let y_raw = cell(y_index);
            if x_raw.is_empty() || y_raw.is_empty() {
                problems.push(format!("{label}: X or Y coordinate is missing"));
            } else if parse_number(&x_raw).is_none() || parse_number(&y_raw).is_none() {
                problems.push(format!("{label}: X/Y is not a number (got \"{x_raw}\", \"{y_raw}\")"));
            }
        }
        problems
    };

    let plan_rows = move || {
        let columns = columns.get();
        let reference_index = column_index_of(&columns, &PlacementColumn::Reference);
        let value_index = column_index_of(&columns, &PlacementColumn::Value);
        let footprint_index = column_index_of(&columns, &PlacementColumn::Footprint);
        let x_index = column_index_of(&columns, &PlacementColumn::X);
        let y_index = column_index_of(&columns, &PlacementColumn::Y);
        let rotation_index = column_index_of(&columns, &PlacementColumn::Rotation);
        let rows = rows.get();

        rows.into_iter()
            .enumerate()
            .map(|(row_index, row)| {
                let cell = |index: Option<usize>| {
                    index
                        .and_then(|index| row.get(index))
                        .cloned()
                        .unwrap_or_default()
                };
                // The part name: footprint column, falling back to value.
                let footprint = {
                    let from_footprint = cell(footprint_index);
                    if from_footprint.trim().is_empty() {
                        cell(value_index)
                    } else {
                        from_footprint
                    }
                };

                PlacementRow {
                    reference: cell(reference_index),
                    value: cell(value_index),
                    footprint,
                    // Rows without usable coordinates cannot be placed;
                    // the server reports how many it had to skip.
                    x: parse_number(&cell(x_index)).unwrap_or(f64::NAN),
                    y: parse_number(&cell(y_index)).unwrap_or(f64::NAN),
                    rotation: parse_number(&cell(rotation_index)).unwrap_or(0.0),
                    fiducial: matches!(
                        kinds.get().get(row_index),
                        Some(ComponentKind::Fiducial)
                    ),
                }
            })
            .collect::<Vec<_>>()
    };

    let plan = move |announce: bool| {
        // Pre-flight: if nothing would survive planning, say exactly why
        // instead of planning an empty job.
        let problems = row_problems();
        let usable = rows.get().iter().enumerate().count()
            - rows
                .get()
                .iter()
                .filter(|row| row.iter().all(|value| value.trim().is_empty()))
                .count();
        if usable == 0 {
            set_plan_note.set(Some(
                "Nothing to plan: the placement list is empty. Add rows or import a CSV/TSV first."
                    .to_string(),
            ));
            set_last_action.set(Some("Planning failed: list is empty".to_string()));
            return;
        }
        if problems.len() >= usable {
            let mut note = format!(
                "Cannot plan: every row has a problem ({} rows). Fix these and try again:",
                problems.len()
            );
            for problem in problems.iter().take(6) {
                note += &format!("\n· {problem}");
            }
            if problems.len() > 6 {
                note += &format!("\n… and {} more", problems.len() - 6);
            }
            set_plan_note.set(Some(note));
            set_last_action.set(Some("Planning failed: fix the flagged rows".to_string()));
            return;
        }

        let rows = plan_rows();
        let board_id = board_id.get();
        spawn_local(async move {
            let column_labels: Vec<String> =
                columns.get().iter().map(|column| column.label().to_string()).collect();
            match plan_job_from_rows(board_id, column_labels, rows).await {
                Ok(planned) => {
                    let mut note = format!(
                        "Planned {} placements from {} parts ({} rows skipped)",
                        planned.placement_count, planned.part_count, planned.skipped
                    );
                    if planned.skipped > 0 && !problems.is_empty() {
                        note.push_str(" — skipped rows:");
                        for problem in problems.iter().take(6) {
                            note.push_str(&format!("\n· {problem}"));
                        }
                    }
                    if !planned.unassigned_parts.is_empty() {
                        note += &format!(
                            "; no feeder for {}",
                            planned.unassigned_parts.join(", ")
                        );
                    }
                    if !planned.unconfirmed_parts.is_empty() {
                        note += &format!("; review: {}", planned.unconfirmed_parts.join(", "));
                    }
                    set_plan_note.set(Some(note));
                    set_last_action.set(Some("Job planned".to_string()));
                    set_job_version.update(|version| *version += 1);
                    // Planning may have created or healed library parts
                    // (value-footprint MPNs, footprint-derived nozzles).
                    // Bump the plan context with a "refresh" kind so the
                    // parts panel reloads; the "plan" kind would make the
                    // debounced auto-replan fire again.
                    let (sequence, _) = plan_cmd.get_untracked();
                    set_plan_cmd.set((sequence + 1, "refresh".to_string()));
                    // An explicit "Plan job" jumps to the result;
                    // automatic re-optimisation stays where it is.
                    if announce {
                        set_tab.set("job".to_string());
                    }
                }
                Err(error) => {
                    set_plan_note.set(Some(format!("Planning failed: {error}")));
                    set_last_action.set(Some(format!("Planning failed: {error}")));
                }
            }
        });
    };

    // Settings components bump the plan command after a save; the
    // re-optimisation is debounced and does not jump tabs.
    let plan_cmd = use_context::<ReadSignal<(u64, String)>>().expect("plan command context");
    let replan = use_debounce_fn(move || plan(false), 300.0);
    Effect::new(move |_| {
        let (sequence, kind) = plan_cmd.get();
        if sequence == 0 {
            return;
        }
        if kind == "plan" {
            replan();
        }
    });

    // Populate the placement list from the working job: once at mount
    // (so a job.json already on disk is editable right away) and every
    // time a job is loaded from the library. Columns get a Footprint
    // column appended because that is what the planner resolves parts
    // by; the numbers are written in plain text the grid parser reads.
    Effect::new(move |_| {
        loaded_version.get();
        spawn_local(async move {
            let exported = match get_placement_rows().await {
                Ok(exported) => exported,
                Err(_) => return, // no job yet — leave the grid alone
            };
            if exported.rows.is_empty() {
                return;
            }
            set_board_id.set(exported.board_id);
            // Restore the column layout saved with the job; fall back to
            // the default grid only for jobs that were never imported
            // with columns (older files).
            let columns: Vec<PlacementColumn> = if exported.columns.is_empty() {
                let mut defaults = default_columns();
                defaults.push(PlacementColumn::Footprint);
                defaults
            } else {
                exported
                    .columns
                    .iter()
                    .map(|label| PlacementColumn::from_header(label))
                    .collect()
            };
            let index_of = |wanted: &PlacementColumn| {
                columns.iter().position(|column| column == wanted)
            };
            let reference_at = index_of(&PlacementColumn::Reference).unwrap_or(0);
            let value_at = index_of(&PlacementColumn::Value).unwrap_or(1);
            let footprint_at = index_of(&PlacementColumn::Footprint);
            let x_at = index_of(&PlacementColumn::X).unwrap_or(2);
            let y_at = index_of(&PlacementColumn::Y).unwrap_or(3);
            let rotation_at = index_of(&PlacementColumn::Rotation).unwrap_or(4);
            let mut grid = Vec::with_capacity(exported.rows.len());
            let mut kinds = Vec::with_capacity(exported.rows.len());
            for row in exported.rows {
                let mut cells = vec![String::new(); columns.len()];
                cells[reference_at] = row.reference.clone();
                cells[value_at] = row.value.clone();
                if let Some(at) = footprint_at {
                    cells[at] = row.footprint.clone();
                }
                cells[x_at] = format!("{}", row.x);
                cells[y_at] = format!("{}", row.y);
                cells[rotation_at] = format!("{}", row.rotation);
                grid.push(cells);
                kinds.push(if row.fiducial {
                    ComponentKind::Fiducial
                } else {
                    ComponentKind::Part
                });
            }
            set_columns.set(columns);
            set_rows.set(grid);
            set_kinds.set(kinds);
        });
    });

    view! {
        <div class="mt-8">
            <BoardHeader
                board_id=board_id
                set_board_id=set_board_id
                machine_state=machine_state
                set_last_action=set_last_action
            />
            <div class="mt-3 flex gap-2 border-b border-slate-600 pb-2">
                {[
                    ("placements", "PCB placement list"),
                    ("process", "Process view"),
                    ("job", "Job manager"),
                    ("jobcfg", "Job config"),
                    ("parts", "Parts & footprints"),
                    ("machine", "Machine setup"),
                ]
                    .into_iter()
                    .map(|(key, label)| {
                        view! {
                            <button
                                class=move || {
                                    if tab.get() == key {
                                        "btn-xp !bg-slate-600"
                                    } else {
                                        "btn-xp"
                                    }
                                }
                                type="button"
                                on:click=move |_| set_tab.set(key.to_string())
                            >
                                {label}
                            </button>
                        }
                    })
                    .collect_view()}
            </div>
            {move || match tab.get().as_str() {
                "placements" => view! {
                    <PlacementEditor
                        columns=columns
                        set_columns=set_columns
                        rows=rows
                        set_rows=set_rows
                        kinds=kinds
                        set_kinds=set_kinds
                        plan=plan
                        plan_note=plan_note
                    />
                }
                .into_any(),
                "process" => view! { <ProcessVisualizer /> }.into_any(),
                "jobcfg" => view! { <JobConfig /> }.into_any(),
                "parts" => view! { <PartsPanel set_last_action=set_last_action /> }.into_any(),
                "machine" => view! { <MachineSetup job_version=job_version /> }.into_any(),
                _ => view! {
                    <JobManager
                        set_last_action=set_last_action
                        plan=plan
                        plan_note=plan_note
                        job_version=job_version
                        on_job_loaded=move || {
                            set_job_version.update(|version| *version += 1);
                            set_loaded_version.update(|version| *version += 1);
                        }
                    />
                }
                .into_any(),
            }}
        </div>
    }
}

/// Editable mirror of the machine setup, all fields as strings so the
/// inputs can be typed in freely and parsed on save.
#[derive(Clone, Default, Debug)]
struct FeederRow {
    id: String,
    bank: String,
    x: String,
    y: String,
    kind: String,
    loaded: String,
}

#[derive(Clone, Default, Debug)]
struct CameraRow {
    id: String,
    kind: String,
    x: String,
    y: String,
    pitch: String,
}

#[derive(Clone, Default, Debug)]
struct NozzleRow {
    id: String,
    kind: String,
    offset: String,
}

#[derive(Clone, Default, Debug)]
struct BankRow {
    id: String,
    rotation: String,
}

#[derive(Clone, Default, Debug)]
struct EditSetup {
    feeders: Vec<FeederRow>,
    banks: Vec<BankRow>,
    cameras: Vec<CameraRow>,
    nozzles: Vec<NozzleRow>,
    // (key, value) for the machine config numbers, in display order.
    config: Vec<(&'static str, String)>,
}

/// Machine setup page: feeders, banks, cameras, nozzles and motion
/// configuration, all editable and saved into the job file.
#[component]
fn MachineSetup(job_version: ReadSignal<u64>) -> impl IntoView {
    let (setup, set_setup) = signal(EditSetup::default());
    let (note, set_note) = signal::<Option<String>>(None);
    // Auto-save is only armed once the first successful load has filled
    // the form — otherwise an auto-save racing the load could overwrite
    // the real setup with the empty defaults.
    let (loaded, set_loaded) = signal(false);
    // Bumped after a successful save so the board re-optimises against
    // the new setup (feeder layout, nozzles, cameras, speeds).
    let (plan_cmd, set_plan_cmd) = (
        use_context::<ReadSignal<(u64, String)>>().expect("plan command context"),
        use_context::<WriteSignal<(u64, String)>>().expect("plan command writer"),
    );
    // Distinguishes the load-echo from a real edit: the auto-save that
    // follows a load writes back identical data and must not trigger
    // a re-plan.
    let (awaiting_echo, set_awaiting_echo) = signal(false);

    let load = move || {
        spawn_local(async move {
            match get_machine_setup().await {
                Ok(data) => {
                    set_setup.set(EditSetup {
                        feeders: data
                            .feeders
                            .iter()
                            .map(|f| FeederRow {
                                id: f.id.to_string(),
                                bank: f.bank_id.to_string(),
                                x: format!("{:.1}", f.x),
                                y: format!("{:.1}", f.y),
                                kind: f.kind.clone(),
                                loaded: f.loaded.clone(),
                            })
                            .collect(),
                        banks: data
                            .banks
                            .iter()
                            .map(|b| BankRow {
                                id: b.id.to_string(),
                                rotation: format!("{:.1}", b.rotation_deg),
                            })
                            .collect(),
                        cameras: data
                            .cameras
                            .iter()
                            .map(|c| CameraRow {
                                id: c.id.to_string(),
                                kind: c.kind.clone(),
                                x: format!("{:.1}", c.x),
                                y: format!("{:.1}", c.y),
                                pitch: format!("{:.1}", c.pitch),
                            })
                            .collect(),
                        nozzles: data
                            .nozzles
                            .iter()
                            .map(|n| NozzleRow {
                                id: n.id.to_string(),
                                kind: n.kind.clone(),
                                offset: format!("{:.2}", n.offset_mm),
                            })
                            .collect(),
                        config: vec![
                            (
                                "Camera position X (mm)",
                                format!("{:.1}", data.config.camera_x),
                            ),
                            (
                                "Camera position Y (mm)",
                                format!("{:.1}", data.config.camera_y),
                            ),
                            ("Travel Z (mm)", format!("{:.2}", data.config.travel_z)),
                            ("Pickup Z (mm)", format!("{:.2}", data.config.pickup_z)),
                            ("Place Z (mm)", format!("{:.2}", data.config.place_z)),
                            ("XY speed", data.config.xy_speed.to_string()),
                            ("Z speed", data.config.z_speed.to_string()),
                            ("Rotation speed", data.config.rotation_speed.to_string()),
                            (
                                "XY approach speed",
                                data.config.xy_approach_speed.to_string(),
                            ),
                            (
                                "Z approach speed",
                                data.config.z_approach_speed.to_string(),
                            ),
                        ],
                    });
                    set_loaded.set(true);
                    set_awaiting_echo.set(true);
                }
                Err(error) => set_note.set(Some(format!("Load failed: {error}"))),
            }
        });
    };
    Effect::new(move |_| {
        load();
    });
    // A different job may define different nozzle kinds — reload the
    // form whenever the job changes (library load, re-plan, ...).
    Effect::new(move |_| {
        job_version.get();
        if loaded.get_untracked() {
            load();
        }
    });

    let do_save = move || {
        let current = setup.get();
        // Parse everything up front so one bad field aborts the save
        // with a message instead of silently writing zeros.
        let number = |key: &str, raw: &str| -> Result<f64, String> {
            raw.trim()
                .replace(',', ".")
                .parse::<f64>()
                .map_err(|_| format!("{key}: {raw:?} is not a number"))
        };
        let int8 = |key: &str, raw: &str| -> Result<u8, String> {
            raw.trim()
                .parse::<u8>()
                .map_err(|_| format!("{key}: {raw:?} is not a whole number 0-255"))
        };
        let result = (|| -> Result<MachineSetupData, String> {
            let mut feeders = Vec::new();
            for row in &current.feeders {
                feeders.push(FeederEntry {
                    id: row
                        .id
                        .trim()
                        .parse()
                        .map_err(|_| format!("feeder id: {:?} is not a number", row.id))?,
                    bank_id: row
                        .bank
                        .trim()
                        .parse()
                        .map_err(|_| format!("bank id: {:?} is not a number", row.bank))?,
                    x: number("feeder X", &row.x)?,
                    y: number("feeder Y", &row.y)?,
                    kind: row.kind.clone(),
                    loaded: row.loaded.clone(),
                });
            }
            let mut banks = Vec::new();
            for row in &current.banks {
                banks.push(BankEntry {
                    id: row
                        .id
                        .trim()
                        .parse()
                        .map_err(|_| format!("bank id: {:?} is not a number", row.id))?,
                    rotation_deg: number("bank rotation", &row.rotation)?,
                });
            }
            let mut cameras = Vec::new();
            for row in &current.cameras {
                cameras.push(CameraEntry {
                    id: row
                        .id
                        .trim()
                        .parse()
                        .map_err(|_| format!("camera id: {:?} is not a number", row.id))?,
                    kind: row.kind.clone(),
                    x: number("camera X", &row.x)?,
                    y: number("camera Y", &row.y)?,
                    pitch: number("camera pitch", &row.pitch)?,
                });
            }
            let mut nozzles = Vec::new();
            for row in &current.nozzles {
                nozzles.push(NozzleEntry {
                    id: row
                        .id
                        .trim()
                        .parse()
                        .map_err(|_| format!("nozzle id: {:?} is not a number", row.id))?,
                    kind: row.kind.clone(),
                    offset_mm: number("nozzle offset", &row.offset)?,
                });
            }
            let get_config = |key: &str| {
                current
                    .config
                    .iter()
                    .find(|(name, _)| *name == key)
                    .map(|(_, value)| value.clone())
                    .unwrap_or_default()
            };
            let config = ConfigEntry {
                camera_x: number("camera position X", &get_config("Camera position X (mm)"))?,
                camera_y: number("camera position Y", &get_config("Camera position Y (mm)"))?,
                travel_z: number("travel Z", &get_config("Travel Z (mm)"))?,
                pickup_z: number("pickup Z", &get_config("Pickup Z (mm)"))?,
                place_z: number("place Z", &get_config("Place Z (mm)"))?,
                xy_speed: int8("XY speed", &get_config("XY speed"))?,
                z_speed: int8("Z speed", &get_config("Z speed"))?,
                rotation_speed: int8("rotation speed", &get_config("Rotation speed"))?,
                xy_approach_speed: int8(
                    "XY approach speed",
                    &get_config("XY approach speed"),
                )?,
                z_approach_speed: int8("Z approach speed", &get_config("Z approach speed"))?,
            };
            Ok(MachineSetupData {
                feeders,
                banks,
                cameras,
                nozzles,
                config,
            })
        })();
        match result {
            Ok(data) => {
                spawn_local(async move {
                    match save_machine_setup(data).await {
                        Ok(message) => {
                            set_note.set(Some(message));
                            // The setup changed — re-optimise the job.
                            let (sequence, _) = plan_cmd.get();
                            set_plan_cmd.set((sequence + 1, "plan".to_string()));
                        }
                        Err(error) => set_note.set(Some(format!("Save failed: {error}"))),
                    }
                });
            }
            Err(message) => set_note.set(Some(format!("Not saved — {message}"))),
        }
    };

    // Parts library, for the feeder "loaded part" dropdown.
    let library = expect_context::<AppData>().snapshot;
    let library_mpns = move || {
        let mut mpns: Vec<String> = library
            .get()
            .and_then(|result| result.ok())
            .map(|summary| {
                summary
                    .parts
                    .iter()
                    .filter(|part| !part.mpn.is_empty())
                    .map(|part| part.mpn.clone())
                    .collect()
            })
            .unwrap_or_default();
        mpns.sort();
        mpns.dedup();
        mpns
    };

    let save = move |_: leptos::ev::MouseEvent| {
        do_save();
    };

    // Auto-save: any edit to the setup form triggers a debounced save,
    // so a change is on disk ~1.2 s after the operator stops typing.
    // Explicit "Save setup" stays available for instant flushes.
    let auto_save = use_debounce_fn(
        move || {
            if loaded.get_untracked() {
                do_save();
            }
        },
        1200.0,
    );
    Effect::new(move |_| {
        setup.get();
        loaded.get();
        if !loaded.get_untracked() {
            return;
        }
        if awaiting_echo.get_untracked() {
            set_awaiting_echo.set(false);
            return;
        }
        auto_save();
    });

    let feeder_kinds = vec![
        ("tape8", "Tape 8 mm"),
        ("tape12", "Tape 12 mm"),
        ("tape16", "Tape 16 mm"),
        ("tape24", "Tape 24 mm"),
        ("tape32", "Tape 32 mm"),
        ("tape44", "Tape 44 mm"),
        ("stick", "Stick"),
        ("tray", "Tray"),
        ("tube", "Tube"),
        ("bulk", "Bulk"),
        ("vibratory", "Vibratory"),
    ];

    view! {
        <section class="flex min-h-0 flex-col gap-3">
            <div class="flex flex-wrap items-center justify-between gap-2">
                <h2 class="text-lg font-semibold">"Machine setup"</h2>
                <div class="flex gap-2">
                    <button class="btn-xp" type="button" on:click=move |_| load()>
                        "Reload"
                    </button>
                    <button class="btn-xp !bg-emerald-800" type="button" on:click=save>
                        "Save setup"
                    </button>
                </div>
            </div>
            <p class="text-xs text-slate-400">
                "Feeder rails, camera array, nozzle head and motion configuration — stored with the job."
            </p>
            {move || note.get().map(|n| view! {
                <p class="rounded border border-slate-600 bg-slate-800 p-2 text-xs text-slate-300">{n}</p>
            })}

            // Feeders (with the banks they mount on).
            <details class="rounded border border-slate-600 p-2" open=true>
                <summary class="cursor-pointer text-xs font-semibold text-slate-300">
                    {move || format!("Feeders ({})", setup.get().feeders.len())}
                </summary>
                    <button
                        class="btn-xp !text-xs !p-1"
                        type="button"
                        on:click=move |_| {
                            set_setup.update(|setup| {
                                let next = setup.feeders.iter().map(|f| f.id.parse::<u8>().unwrap_or(0)).max().unwrap_or(0) + 1;
                                setup.feeders.push(FeederRow {
                                    id: next.to_string(),
                                    bank: "0".to_string(),
                                    x: "0".to_string(),
                                    y: "400".to_string(),
                                    kind: "tape16".to_string(),
                                    loaded: String::new(),
                                });
                            });
                        }
                    >
                        "+ feeder"
                    </button>
                <div class="mt-2 max-h-48 overflow-auto">
                    <table class="min-w-full text-left text-xs">
                        <thead class="bg-slate-700">
                            <tr>
                                <th class="p-1">"Slot"</th>
                                <th class="p-1">"Bank"</th>
                                <th class="p-1">"X (mm)"</th>
                                <th class="p-1">"Y (mm)"</th>
                                <th class="p-1">"Kind"</th>
                                <th class="p-1">"Loaded part (MPN)"</th>
                                <th class="p-1"></th>
                            </tr>
                        </thead>
                        <tbody>
                            {move || {
                                setup.get().feeders.into_iter().enumerate().map(|(index, row)| {
                                    let row_id = row.id.clone();
                                    view! {
                                        <tr class="border-t border-slate-600">
                                            <td class="p-1">
                                                <input class="w-10 bg-slate-700 p-1" prop:value=row.id.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.feeders.get_mut(index) { r.id = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <input class="w-10 bg-slate-700 p-1" prop:value=row.bank.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.feeders.get_mut(index) { r.bank = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <input class="w-16 bg-slate-700 p-1" prop:value=row.x.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.feeders.get_mut(index) { r.x = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <input class="w-16 bg-slate-700 p-1" prop:value=row.y.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.feeders.get_mut(index) { r.y = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <select class="bg-slate-700 p-1"
                                                    on:change=move |e| {
                                                        set_setup.update(|s| { if let Some(r) = s.feeders.get_mut(index) { r.kind = event_target_value(&e); } });
                                                        do_save();
                                                    }
                                                >
                                                    {feeder_kinds
                                                        .iter()
                                                        .map(|(value, label)| {
                                                            view! {
                                                                <option value=*value selected=row.kind == *value>{*label}</option>
                                                            }
                                                        })
                                                        .collect_view()}
                                                </select>
                                            </td>
                                            <td class="p-1">
                                                <select
                                                    class="w-28 bg-slate-700 p-1"
                                                    on:change=move |e| {
                                                        set_setup.update(|s| { if let Some(r) = s.feeders.get_mut(index) { r.loaded = event_target_value(&e); } });
                                                        do_save();
                                                    }
                                                >
                                                    <option value="" selected=row.loaded.is_empty()>"— empty —"</option>
                                                    {library_mpns()
                                                        .into_iter()
                                                        .map(|mpn| {
                                                            let option_value = mpn.clone();
                                                            view! {
                                                                <option value=option_value.clone() selected=option_value == row.loaded>
                                                                    {mpn}
                                                                </option>
                                                            }
                                                        })
                                                        .collect_view()}
                                                </select>
                                            </td>
                                            <td class="p-1">
                                                <button
                                                    class="text-red-400 hover:text-red-300"
                                                    type="button"
                                                    title=format!("Remove feeder {row_id}")
                                                    on:click=move |_| set_setup.update(|s| { s.feeders.remove(index); })
                                                >
                                                    "✕"
                                                </button>
                                            </td>
                                        </tr>
                                    }
                                }).collect_view()
                            }}
                        </tbody>
                    </table>
                </div>
                <div class="mt-2 flex flex-wrap items-center gap-2 text-xs">
                    <span class="font-semibold text-slate-300">"Banks:"</span>
                    {move || {
                        setup.get().banks.into_iter().enumerate().map(|(index, row)| {
                            view! {
                                <div class="flex items-center gap-1 rounded border border-slate-600 p-1 text-xs">
                                    <span>"#"</span>
                                    <input class="w-8 bg-slate-700 p-1" prop:value=row.id.clone()
                                        on:input=move |e| set_setup.update(|s| { if let Some(r) = s.banks.get_mut(index) { r.id = event_target_value(&e); } }) />
                                    <span>"rot °"</span>
                                    <input class="w-12 bg-slate-700 p-1" prop:value=row.rotation.clone()
                                        on:input=move |e| set_setup.update(|s| { if let Some(r) = s.banks.get_mut(index) { r.rotation = event_target_value(&e); } }) />
                                    <button
                                        class="text-red-400 hover:text-red-300"
                                        type="button"
                                        on:click=move |_| set_setup.update(|s| { s.banks.remove(index); })
                                    >
                                        "✕"
                                    </button>
                                </div>
                            }
                        }).collect_view()
                    }}
                    <button
                        class="btn-xp !text-xs !p-1"
                        type="button"
                        on:click=move |_| {
                            set_setup.update(|setup| {
                                let next = setup.banks.iter().map(|b| b.id.parse::<u8>().unwrap_or(0)).max().unwrap_or(0) + 1;
                                setup.banks.push(BankRow {
                                    id: next.to_string(),
                                    rotation: "0".to_string(),
                                });
                            });
                        }
                    >
                        "+ bank"
                    </button>
                </div>
            </details>

            // Cameras.
            <details class="rounded border border-slate-600 p-2">
                <summary class="cursor-pointer text-xs font-semibold text-slate-300">
                    {move || format!("Cameras ({})", setup.get().cameras.len())}
                </summary>
                    <button
                        class="btn-xp !text-xs !p-1"
                        type="button"
                        title="Replace fast cameras with a 4-camera array at 20 mm pitch"
                        on:click=move |_| {
                            set_setup.update(|setup| {
                                setup.cameras.retain(|c| c.kind != "fast");
                                for i in 0..4 {
                                    setup.cameras.push(CameraRow {
                                        id: (10 + i).to_string(),
                                        kind: "fast".to_string(),
                                        x: (i as f64 * 20.0).to_string(),
                                        y: "200".to_string(),
                                        pitch: "20".to_string(),
                                    });
                                }
                            });
                        }
                    >
                        "Set 4-camera fast array (pitch 20 mm)"
                    </button>
                <div class="mt-2 max-h-48 overflow-auto">
                    <table class="min-w-full text-left text-xs">
                        <thead class="bg-slate-700">
                            <tr>
                                <th class="p-1">"ID"</th>
                                <th class="p-1">"Kind"</th>
                                <th class="p-1">"X (mm)"</th>
                                <th class="p-1">"Y (mm)"</th>
                                <th class="p-1">"Pitch (mm)"</th>
                                <th class="p-1"></th>
                            </tr>
                        </thead>
                        <tbody>
                            {move || {
                                setup.get().cameras.into_iter().enumerate().map(|(index, row)| {
                                    view! {
                                        <tr class="border-t border-slate-600">
                                            <td class="p-1">
                                                <input class="w-10 bg-slate-700 p-1" prop:value=row.id.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.cameras.get_mut(index) { r.id = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <select class="bg-slate-700 p-1"
                                                    on:change=move |e| {
                                                        set_setup.update(|s| { if let Some(r) = s.cameras.get_mut(index) { r.kind = event_target_value(&e); } });
                                                        do_save();
                                                    }
                                                >
                                                    <option value="fast" selected=row.kind == "fast">"Fast vision"</option>
                                                    <option value="precise" selected=row.kind == "precise">"Precise"</option>
                                                    <option value="head" selected=row.kind == "head">"Head"</option>
                                                </select>
                                            </td>
                                            <td class="p-1">
                                                <input class="w-16 bg-slate-700 p-1" prop:value=row.x.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.cameras.get_mut(index) { r.x = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <input class="w-16 bg-slate-700 p-1" prop:value=row.y.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.cameras.get_mut(index) { r.y = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <input class="w-14 bg-slate-700 p-1" prop:value=row.pitch.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.cameras.get_mut(index) { r.pitch = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <button
                                                    class="text-red-400 hover:text-red-300"
                                                    type="button"
                                                    on:click=move |_| set_setup.update(|s| { s.cameras.remove(index); })
                                                >
                                                    "✕"
                                                </button>
                                            </td>
                                        </tr>
                                    }
                                }).collect_view()
                            }}
                        </tbody>
                    </table>
                </div>
            </details>

            // Head nozzles.
            <details class="rounded border border-slate-600 p-2">
                <summary class="cursor-pointer text-xs font-semibold text-slate-300">
                    {move || format!("Head nozzles ({})", setup.get().nozzles.len())}
                </summary>
                    <button
                        class="btn-xp !text-xs !p-1"
                        type="button"
                        on:click=move |_| {
                            set_setup.update(|setup| {
                                let next = setup.nozzles.iter().map(|n| n.id.parse::<u8>().unwrap_or(0)).max().unwrap_or(0) + 1;
                                setup.nozzles.push(NozzleRow {
                                    id: next.to_string(),
                                    kind: "standard".to_string(),
                                    offset: "0".to_string(),
                                });
                            });
                        }
                    >
                        "+ nozzle"
                    </button>
                <div class="mt-2 max-h-48 overflow-auto">
                    <table class="min-w-full text-left text-xs">
                        <thead class="bg-slate-700">
                            <tr>
                                <th class="p-1">"ID"</th>
                                <th class="p-1">"Kind"</th>
                                <th class="p-1">"Offset (mm)"</th>
                                <th class="p-1"></th>
                            </tr>
                        </thead>
                        <tbody>
                            {move || {
                                setup.get().nozzles.into_iter().enumerate().map(|(index, row)| {
                                    view! {
                                        <tr class="border-t border-slate-600">
                                            <td class="p-1">
                                                <input class="w-10 bg-slate-700 p-1" prop:value=row.id.clone()
                                                    on:input=move |e| set_setup.update(|s| { if let Some(r) = s.nozzles.get_mut(index) { r.id = event_target_value(&e); } }) />
                                            </td>
                                            <td class="p-1">
                                                <select class="bg-slate-700 p-1"
                                                    on:change=move |e| {
                                                        set_setup.update(|s| { if let Some(r) = s.nozzles.get_mut(index) { r.kind = event_target_value(&e); } });
                                                        do_save();
                                                    }
                                                >
                                                    <option value="standard" selected=row.kind == "standard">"Standard (3.0)"</option>
                                                    <option value="fine" selected=row.kind == "fine">"Fine (1.5)"</option>
                                                    <option value="extra_fine" selected=row.kind == "extra_fine">"Extra fine (1.0)"</option>
                                                    <option value="n501" selected=row.kind == "n501">"N501 (0.5)"</option>
                                                    <option value="n502" selected=row.kind == "n502">"N502 (0.4)"</option>
                                                    <option value="n503" selected=row.kind == "n503">"N503 (0.3)"</option>
                                                </select>
                                            </td>
                                            <td class="p-1 font-mono" title="Measured by the nozzle calibration — run it below to change">
                                                {format!("{:.3}", row.offset.parse::<f64>().unwrap_or(0.0))}
                                            </td>
                                            <td class="p-1">
                                                <button
                                                    class="text-red-400 hover:text-red-300"
                                                    type="button"
                                                    on:click=move |_| set_setup.update(|s| { s.nozzles.remove(index); })
                                                >
                                                    "✕"
                                                </button>
                                            </td>
                                        </tr>
                                    }
                                }).collect_view()
                            }}
                        </tbody>
                    </table>
                </div>
            </details>

            // Motion configuration.
            <details class="rounded border border-slate-600 p-2">
                <summary class="cursor-pointer text-xs font-semibold text-slate-300">
                    "Motion configuration"
                </summary>
                <div class="mt-2 grid grid-cols-2 gap-2 sm:grid-cols-3">
                    {move || {
                        setup.get().config.into_iter().enumerate().map(|(index, (label, value))| {
                            view! {
                                <label class="text-xs">
                                    {label}
                                    <input
                                        class="mt-1 w-full bg-slate-700 p-1"
                                        prop:value=value
                                        on:input=move |e| {
                                            let raw = event_target_value(&e);
                                            set_setup.update(|s| {
                                                if let Some((_, slot)) = s.config.get_mut(index) {
                                                    *slot = raw;
                                                }
                                            });
                                        }
                                    />
                                </label>
                            }
                        }).collect_view()
                    }}
                </div>
            </details>

            <details class="rounded border border-slate-600 p-2">
                <summary class="cursor-pointer text-xs font-semibold text-slate-300">
                    "Nozzle XY calibration"
                </summary>
                <NozzleCalibrationPage />
            </details>
            <details class="rounded border border-slate-600 p-2">
                <summary class="cursor-pointer text-xs font-semibold text-slate-300">
                    "Pick & place height calibration"
                </summary>
                <PickPlaceCalibrationPage />
            </details>
        </section>
    }
}

/// Job configuration page: the fiducial workflow and the resulting
/// board mapping.
///
/// 1. The placement list rows marked as fiducials appear here.
/// 2. Jog the head camera over fiducial 1, press "Save current
///    position" — that rough spot is stored as the expected position.
/// 3. Press "Run finder": the machine moves to the expected position,
///    the fiducial finder locks on and the refined position is stored.
/// 4. Repeat for fiducial 2.
/// 5. Press "Map board": the design coordinates are mapped onto the
///    machine coordinates via the two located fiducials.
#[component]
fn JobConfig() -> impl IntoView {
    let (fiducials, set_fiducials) = signal::<Vec<FiducialEntry>>(vec![]);
    let (note, set_note) = signal::<Option<String>>(None);
    let (mapping, set_mapping) = signal::<Option<BoardMappingData>>(None);
    let (busy, set_busy) = signal(false);

    let load = move || {
        spawn_local(async move {
            match get_fiducials().await {
                Ok(list) => set_fiducials.set(list),
                Err(error) => set_note.set(Some(format!("Load failed: {error}"))),
            }
        });
    };
    Effect::new(move |_| {
        load();
    });

    let save_expected = move |index: usize| {
        set_busy.set(true);
        spawn_local(async move {
            match set_fiducial_expected(index as u32).await {
                Ok((x, y)) => {
                    set_note.set(Some(format!("Saved expected position of fiducial {}: ({:.2}, {:.2})", index + 1, x, y)));
                    load();
                }
                Err(error) => set_note.set(Some(format!("Failed: {error}"))),
            }
            set_busy.set(false);
        });
    };
    let run_finder = move |index: usize| {
        set_busy.set(true);
        set_note.set(Some(format!("Running fiducial finder on fiducial {}…", index + 1)));
        spawn_local(async move {
            match locate_fiducial(index as u32).await {
                Ok((x, y)) => {
                    set_note.set(Some(format!(
                        "Fiducial {} located at ({:.2}, {:.2})",
                        index + 1, x, y
                    )));
                    load();
                }
                Err(error) => set_note.set(Some(format!("Finder failed: {error}"))),
            }
            set_busy.set(false);
        });
    };
    let map = move |_| {
        set_busy.set(true);
        set_note.set(Some("Mapping the board…".to_string()));
        spawn_local(async move {
            match map_board().await {
                Ok(result) => {
                    set_mapping.set(Some(result.clone()));
                    let message = if result.warning.is_empty() {
                        format!(
                            "Board mapped: origin ({:.2}, {:.2}), rotation {:.2}°, scale {:.3}",
                            result.origin_x, result.origin_y, result.rotation_deg, result.scale
                        )
                    } else {
                        format!("Board mapped with warning: {}", result.warning)
                    };
                    set_note.set(Some(message));
                    load();
                }
                Err(error) => set_note.set(Some(format!("Mapping failed: {error}"))),
            }
            set_busy.set(false);
        });
    };

    view! {
        <section class="flex min-h-0 flex-col gap-3">
            <div class="flex flex-wrap items-center justify-between gap-2">
                <h2 class="text-lg font-semibold">"Job configuration"</h2>
                <button class="btn-xp" type="button" on:click=move |_| load()>
                    "Reload"
                </button>
            </div>
            <p class="text-xs text-slate-400">
                "Board alignment workflow: (1) jog the head camera roughly over a fiducial mark, \
                 (2) save that as the expected position, (3) run the fiducial finder to refine it, \
                 (4) repeat for the second fiducial, (5) map the board onto the machine coordinates. \
                 The runner repeats this mapping live at the start of every job."
            </p>
            {move || note.get().map(|n| view! {
                <p class="rounded border border-slate-600 bg-slate-800 p-2 text-xs text-slate-300">{n}</p>
            })}

            <div class="rounded border border-slate-600 p-2">
                <p class="text-xs font-semibold text-slate-300">"Fiducials"</p>
                <Show when=move || fiducials.get().is_empty()>
                    <p class="mt-1 text-xs text-slate-400">
                        "No fiducials yet. Mark one or two rows as fiducials in the placement list \
                         (right-click → mark as fiducial), then re-plan the job."
                    </p>
                </Show>
                <div class="mt-2 overflow-auto">
                    <table class="min-w-full text-left text-xs">
                        <thead class="bg-slate-700">
                            <tr>
                                <th class="p-1">"#"</th>
                                <th class="p-1">"Kind"</th>
                                <th class="p-1">"Design X / Y (mm)"</th>
                                <th class="p-1">"Expected (machine)"</th>
                                <th class="p-1">"Located (machine)"</th>
                                <th class="p-1">"Actions"</th>
                            </tr>
                        </thead>
                        <tbody>
                            {move || {
                                fiducials.get().into_iter().map(|fid| {
                                    let index = fid.index;
                                    view! {
                                        <tr class="border-t border-slate-600">
                                            <td class="p-1">{index + 1}</td>
                                            <td class="p-1">{fid.kind.clone()}</td>
                                            <td class="p-1 font-mono">
                                                {format!("{:.2} / {:.2}", fid.design_x, fid.design_y)}
                                            </td>
                                            <td class="p-1 font-mono">
                                                {fid.expected
                                                    .map(|(x, y)| format!("({x:.2}, {y:.2})"))
                                                    .unwrap_or_else(|| "—".to_string())}
                                            </td>
                                            <td class="p-1 font-mono">
                                                {fid.machine
                                                    .map(|(x, y)| format!("({x:.2}, {y:.2})"))
                                                    .unwrap_or_else(|| "—".to_string())}
                                            </td>
                                            <td class="p-1">
                                                <div class="flex gap-1">
                                                    <button
                                                        class="btn-xp !text-xs !p-1"
                                                        type="button"
                                                        disabled=busy.get()
                                                        title="Store the machine's current position as the rough fiducial position"
                                                        on:click=move |_| save_expected(index)
                                                    >
                                                        "Save current pos"
                                                    </button>
                                                    <button
                                                        class="btn-xp !text-xs !p-1"
                                                        type="button"
                                                        disabled=busy.get()
                                                        title="Move to the expected position and run the fiducial finder"
                                                        on:click=move |_| run_finder(index)
                                                    >
                                                        "Run finder"
                                                    </button>
                                                </div>
                                            </td>
                                        </tr>
                                    }
                                }).collect_view()
                            }}
                        </tbody>
                    </table>
                </div>
            </div>

            <div class="rounded border border-slate-600 p-2">
                <p class="text-xs font-semibold text-slate-300">"Board mapping"</p>
                {move || mapping.get().map(|m| {
                    let warning_text = m.warning.clone();
                    let warning_flag = warning_text.clone();
                    view! {
                    <div class="mt-2 rounded border border-slate-600 bg-slate-800 p-2 font-mono text-xs">
                        <p>"origin: (" {format!("{:.2}", m.origin_x)} ", " {format!("{:.2}", m.origin_y)} ") mm"</p>
                        <p>"rotation: " {format!("{:.2}", m.rotation_deg)} "°"</p>
                        <p>"scale: " {format!("{:.4}", m.scale)}</p>
                        <Show when=move || !warning_flag.is_empty()>
                            <p class="text-yellow-300">{warning_text.clone()}</p>
                        </Show>
                    </div>
                    }
                })}
                <button
                    class="btn-xp mt-2 !bg-emerald-800"
                    type="button"
                    disabled=busy.get()
                    on:click=map
                >
                    "Map board from located fiducials"
                </button>
            </div>
        </section>
    }
}

/// Nozzle XY-offset calibration page.
///
/// The paper-and-ink method: ink the tips, lay paper on the deck, lower
/// the nozzles onto the paper, then let the camera measure where each
/// nozzle actually landed. The machine repeats the touch at the four
/// cardinal rotations (0/90/180/270°), which separates the nozzle XY
/// offset from the eccentric runout of the rotated assembly.
#[component]
fn NozzleCalibrationPage() -> impl IntoView {
    let (head_x, set_head_x) = signal(String::new());
    let (head_y, set_head_y) = signal(String::new());
    let (touch_z, set_touch_z) = signal(String::new());
    let (camera, set_camera) = signal("head".to_string());
    let (running, set_running) = signal(false);
    let (result, set_result) = signal::<Option<String>>(None);
    // Bumped after a successful run so the stored table refetches.
    let (version, set_version) = signal(0u64);
    let stored = Resource::new(
        move || version.get(),
        |_| async move { get_nozzle_offsets().await },
    );

    // The operator jogs the head over the first ink mark, then captures
    // the machine's current XY as the calibration origin.
    let use_current = move |_| {
        spawn_local(async move {
            match get_status().await {
                Ok(status) => {
                    set_head_x.set(format!("{:.3}", status.x));
                    set_head_y.set(format!("{:.3}", status.y));
                    set_result.set(Some(
                        "Head position captured from the machine.".to_string(),
                    ));
                }
                Err(error) => {
                    set_result.set(Some(format!("Could not read position: {error}")))
                }
            }
        });
    };

    let run = move |_| {
        set_running.set(true);
        set_result.set(Some("Calibrating — the machine will touch the paper, rotate the nozzles and photograph the marks. Keep hands clear.".to_string()));
        let head_x = head_x.get();
        let head_y = head_y.get();
        let touch_z = touch_z.get();
        let camera = camera.get();
        spawn_local(async move {
            match calibrate_nozzles(head_x, head_y, touch_z, camera).await {
                Ok(message) => set_result.set(Some(message)),
                Err(error) => set_result.set(Some(format!("Calibration failed: {error}"))),
            }
            set_running.set(false);
            set_version.update(|version| *version += 1);
        });
    };

    let ready = move || {
        !head_x.get().trim().is_empty()
            && !head_y.get().trim().is_empty()
            && !touch_z.get().trim().is_empty()
            && !running.get()
    };

    view! {
        <section class="rounded border border-slate-600 p-2">
            <h2 class="text-lg font-semibold">"Nozzle XY calibration"</h2>
            <p class="mt-1 text-xs text-slate-400">
                "Finds the exact XY offset of every nozzle using the paper-and-ink method. \
                 The result also includes runout — how far the tip wanders as the nozzle \
                 rotates — which no offset can fix and which flags a worn nozzle."
            </p>

            <ol class="mt-2 list-decimal space-y-1 pl-5 text-xs text-slate-300">
                <li>"Ink all four nozzle tips and lay a piece of paper flat on the deck."</li>
                <li>
                    "Jog the head over the paper and slowly lower the nozzles onto it until \
                     every tip leaves a visible mark. Enter the paper height as touch Z."
                </li>
                <li>
                    "Jog the camera over the first mark, then press ""Use current position"" — \
                     that XY is the calibration origin."
                </li>
                <li>
                    "Run the calibration: the machine touches down, rotates the nozzle array \
                     90° at a time and measures the marks at all four cardinal directions."
                </li>
            </ol>

            <div class="mt-3 grid grid-cols-2 gap-2 text-xs sm:grid-cols-4">
                <label class="flex flex-col gap-1">
                    "Head X (mm)"
                    <input
                        class="bg-slate-700 p-1"
                        prop:value=head_x
                        on:input=move |e| set_head_x.set(event_target_value(&e))
                    />
                </label>
                <label class="flex flex-col gap-1">
                    "Head Y (mm)"
                    <input
                        class="bg-slate-700 p-1"
                        prop:value=head_y
                        on:input=move |e| set_head_y.set(event_target_value(&e))
                    />
                </label>
                <label class="flex flex-col gap-1">
                    "Touch Z (mm)"
                    <input
                        class="bg-slate-700 p-1"
                        prop:value=touch_z
                        on:input=move |e| set_touch_z.set(event_target_value(&e))
                    />
                </label>
                <label class="flex flex-col gap-1">
                    "Camera"
                    <select
                        class="bg-slate-700 p-1"
                        on:change=move |e| set_camera.set(event_target_value(&e))
                    >
                        <option value="head">"Head"</option>
                        <option value="precise">"Precise vision"</option>
                        <option value="fast-1">"Fast vision 1"</option>
                        <option value="fast-2">"Fast vision 2"</option>
                        <option value="fast-3">"Fast vision 3"</option>
                        <option value="fast-4">"Fast vision 4"</option>
                    </select>
                </label>
            </div>

            <div class="mt-3 flex flex-wrap gap-2">
                <button class="btn-xp" type="button" on:click=use_current>
                    "Use current position"
                </button>
                <button
                    class="btn-xp !bg-emerald-800"
                    type="button"
                    disabled=move || !ready()
                    on:click=run
                >
                    "Run calibration"
                </button>
            </div>

            {move || result.get().map(|text| view! {
                <pre class="mt-3 overflow-auto rounded border border-slate-600 bg-slate-800 p-2 text-xs whitespace-pre-wrap">{text}</pre>
            })}

            <div class="mt-3 rounded border border-slate-600 p-2">
                <p class="text-xs font-semibold text-slate-300">"Stored calibration"</p>
                <div class="mt-1">
                    {move || match stored.get() {
                        None => view! { <p class="text-xs text-slate-400">"Loading…"</p> }.into_any(),
                        Some(Err(error)) => view! {
                            <p class="text-xs text-red-300">{format!("{error}")}</p>
                        }.into_any(),
                        Some(Ok(entries)) if entries.is_empty() => view! {
                            <p class="text-xs text-slate-400">"No calibration stored yet."</p>
                        }.into_any(),
                        Some(Ok(entries)) => view! {
                            <table class="min-w-full text-left text-xs">
                                <thead class="bg-slate-700">
                                    <tr>
                                        <th class="p-1">"Nozzle"</th>
                                        <th class="p-1">"XY offset (mm)"</th>
                                        <th class="p-1">"Runout (mm)"</th>
                                    </tr>
                                </thead>
                                <tbody>
                                    {entries
                                        .into_iter()
                                        .map(|entry| {
                                            let offset_text = format!("{:+.3}", entry.offset_mm);
                                            let runout_text = format!("{:.3}", entry.runout_mm);
                                            view! {
                                                <tr class="border-t border-slate-600">
                                                    <td class="p-1">{entry.label}</td>
                                                    <td class="p-1 font-mono">{offset_text}</td>
                                                    <td class="p-1 font-mono">{runout_text}</td>
                                                </tr>
                                            }
                                        })
                                        .collect_view()}
                                </tbody>
                            </table>
                        }.into_any(),
                    }
                }
                </div>
            </div>
        </section>
    }
}

/// Guided pick & place height calibration.
///
/// The operator walks one part through a full manual pick and place:
/// camera finds the spot, nozzle is jogged over it and crept down onto
/// the part, suction grabs it and the nozzle retracts — the Z at the
/// moment of pickup is recorded automatically as the footprint's pickup
/// height. The same happens in reverse at the destination: the Z where
/// the part is released is recorded as the place height.
#[component]
fn PickPlaceCalibrationPage() -> impl IntoView {
    let (nozzle, set_nozzle) = signal("1".to_string());
    let (footprint, set_footprint) = signal(String::new());
    let (camera, set_camera) = signal("precise".to_string());
    let (busy, set_busy) = signal(false);
    let (note, set_note) = signal::<Option<String>>(None);
    // Live position readout: (x, y, z of the selected nozzle).
    let (position, set_position) = signal::<Option<(f64, f64, f64)>>(None);
    // Heights come from the shared snapshot; refresh it after changes.
    let app = expect_context::<AppData>();
    let heights = move || app.data().map(|d| d.heights).unwrap_or_default();

    // Parts library for the footprint dropdown.
    let library = expect_context::<AppData>().snapshot;
    let library_footprints = move || {
        let mut names: Vec<String> = library
            .get()
            .and_then(|result| result.ok())
            .map(|summary| {
                summary
                    .parts
                    .iter()
                    .map(|part| part.footprint.clone())
                    .collect()
            })
            .unwrap_or_default();
        names.sort();
        names.dedup();
        names
    };

    let nozzle_id = move || nozzle.get().parse::<u8>().unwrap_or(1);

    let read_position = move || {
        spawn_local(async move {
            match get_status().await {
                Ok(status) => {
                    let index = (nozzle_id() - 1).min(3) as usize;
                    set_position.set(Some((status.x, status.y, status.z[index])));
                }
                Err(error) => set_note.set(Some(format!("Position read failed: {error}"))),
            }
        })
    };
    Effect::new(move |_| {
        read_position();
    });

    let run = move |action: &'static str| {
        set_busy.set(true);
        let n = nozzle_id();
        let fp = footprint.get();
        let camera_name = camera.get();
        spawn_local(async move {
            let outcome = match action {
                "over_camera" => move_nozzle_to_camera(camera_name.clone())
                    .await
                    .map(|(x, y)| format!("Head moved to the {camera_name} station ({x:.2}, {y:.2}). Fine-jog the nozzle over the mark."))
                    .map_err(|e| e.to_string()),
                "z_down" => cal_z_jog(n, "-0.5".to_string())
                    .await
                    .map(|z| format!("Nozzle Z is now {z:.3} mm"))
                    .map_err(|e| e.to_string()),
                "z_up" => cal_z_jog(n, "0.5".to_string())
                    .await
                    .map(|z| format!("Nozzle Z is now {z:.3} mm"))
                    .map_err(|e| e.to_string()),
                "z_zero" => cal_z_zero(n)
                    .await
                    .map(|_| "Nozzle retracted to Z 0.".to_string())
                    .map_err(|e| e.to_string()),
                "pick" => cal_pick(n, fp)
                    .await
                    .map_err(|e| e.to_string()),
                "place" => cal_place(n, fp)
                    .await
                    .map_err(|e| e.to_string()),
                "vacuum_off" => cal_vacuum(n, "off".to_string())
                    .await
                    .map(|_| "Suction off.".to_string())
                    .map_err(|e| e.to_string()),
                _ => Err("unknown".to_string()),
            };
            match outcome {
                Ok(message) => set_note.set(Some(message)),
                Err(error) => set_note.set(Some(error)),
            }
            set_busy.set(false);
            app.refresh();
            // Refresh the readout after every machine action.
            if let Ok(status) = get_status().await {
                let index = (n - 1).min(3) as usize;
                set_position.set(Some((status.x, status.y, status.z[index])));
            }
        });
    };

    let recorded_for_footprint = move || {
        let footprint = footprint.get();
        heights()
            .into_iter()
            .find(|entry| entry.footprint == footprint)
    };

    view! {
        <section class="rounded border border-slate-600 p-2">
            <h2 class="text-lg font-semibold">"Pick & place height calibration"</h2>
            <p class="mt-1 text-xs text-slate-400">
                "Walks one component through a full manual pick and place and records the \
                 heights automatically: the Z at the moment suction grabs the part becomes \
                 the pickup height, the Z at the moment the part is released becomes the \
                 place height. Both are stored per footprint."
            </p>

            <ol class="mt-2 list-decimal space-y-1 pl-5 text-xs text-slate-300">
                <li>"Select the nozzle and the part's footprint below."</li>
                <li>
                    "For a feeder pick: put a component in the feeder, then use ""Move nozzle \
                     over camera"" and fine-jog until the nozzle is over the part."
                </li>
                <li>"Creep down with ""Z −0.5"" until the nozzle sits on the part."</li>
                <li>
                    "Press ""Pick"" — suction grabs the part, the nozzle retracts and the \
                     pickup height is recorded."
                </li>
                <li>
                    "Move to the destination (another camera station for nozzle-to-nozzle \
                     transfer, or a board position) and creep down again until the part \
                     touches the surface."
                </li>
                <li>
                    "Press ""Place"" — the part is blown off, the nozzle retracts and the \
                     place height is recorded."
                </li>
            </ol>

            <div class="mt-3 grid grid-cols-2 gap-2 text-xs sm:grid-cols-4">
                <label class="flex flex-col gap-1">
                    "Nozzle"
                    <select
                        class="bg-slate-700 p-1"
                        on:change=move |e| {
                            set_nozzle.set(event_target_value(&e));
                            set_position.set(None);
                        }
                    >
                        <option value="1">"Nozzle 1"</option>
                        <option value="2">"Nozzle 2"</option>
                        <option value="3">"Nozzle 3"</option>
                        <option value="4">"Nozzle 4"</option>
                    </select>
                </label>
                <label class="flex flex-col gap-1">
                    "Part footprint"
                    <select
                        class="bg-slate-700 p-1"
                        on:change=move |e| set_footprint.set(event_target_value(&e))
                    >
                        <option value="">"— (no height recording)"</option>
                        {move || {
                            library_footprints()
                                .into_iter()
                                .map(|name| {
                                    let option_value = name.clone();
                                    view! {
                                        <option value=option_value.clone() selected=option_value == footprint.get()>
                                            {name}
                                        </option>
                                    }
                                })
                                .collect_view()
                        }}
                    </select>
                </label>
                <label class="flex flex-col gap-1">
                    "Reference camera"
                    <select
                        class="bg-slate-700 p-1"
                        on:change=move |e| set_camera.set(event_target_value(&e))
                    >
                        <option value="precise">"Precise vision"</option>
                        <option value="fast-1">"Fast vision 1"</option>
                        <option value="fast-2">"Fast vision 2"</option>
                        <option value="fast-3">"Fast vision 3"</option>
                        <option value="fast-4">"Fast vision 4"</option>
                    </select>
                </label>
                <div class="flex flex-col gap-1 rounded border border-slate-600 bg-slate-800 p-2 font-mono text-xs">
                    <span>"X " {move || position.get().map(|p| format!("{:.2}", p.0)).unwrap_or_else(|| "—".into())}</span>
                    <span>"Y " {move || position.get().map(|p| format!("{:.2}", p.1)).unwrap_or_else(|| "—".into())}</span>
                    <span>"Z " {move || position.get().map(|p| format!("{:.2}", p.2)).unwrap_or_else(|| "—".into())}</span>
                </div>
            </div>

            <div class="mt-3 flex flex-wrap gap-2">
                <button class="btn-xp" type="button" disabled=move || busy.get() on:click=move |_| run("over_camera")>
                    "Move nozzle over camera"
                </button>
                <button class="btn-xp !px-2" type="button" disabled=move || busy.get() title="Nozzle down 0.5 mm" on:click=move |_| run("z_down")>
                    "Z −0.5"
                </button>
                <button class="btn-xp !px-2" type="button" disabled=move || busy.get() title="Nozzle up 0.5 mm" on:click=move |_| run("z_up")>
                    "Z +0.5"
                </button>
                <button class="btn-xp !px-2" type="button" disabled=move || busy.get() title="Retract nozzle fully to Z 0" on:click=move |_| run("z_zero")>
                    "Z → 0"
                </button>
                <button
                    class="btn-xp !bg-emerald-800"
                    type="button"
                    disabled=move || busy.get()
                    title="Suction on, record pickup height, retract"
                    on:click=move |_| run("pick")
                >
                    "Pick: suction ON + record"
                </button>
                <button
                    class="btn-xp !bg-amber-800"
                    type="button"
                    disabled=move || busy.get()
                    title="Release part, record place height, retract"
                    on:click=move |_| run("place")
                >
                    "Place: release + record"
                </button>
                <button class="btn-xp !bg-red-900" type="button" disabled=move || busy.get() on:click=move |_| run("vacuum_off")>
                    "Suction off"
                </button>
            </div>

            {move || note.get().map(|n| view! {
                <p class="mt-3 rounded border border-slate-600 bg-slate-800 p-2 text-xs text-slate-300">{n}</p>
            })}

            {move || {
                recorded_for_footprint().map(|entry| {
                    let pickup_text = format!("{:.3}", entry.pickup_z_mm);
                    let place_text = format!("{:.3}", entry.place_z_mm);
                    view! {
                        <div class="mt-3 rounded border border-slate-600 bg-slate-800 p-2 text-xs">
                            <p class="font-semibold text-slate-300">
                                "Recorded for " {footprint.get()} ":"
                            </p>
                            <p class="font-mono">
                                "pickup Z " {pickup_text} " mm · place Z " {place_text} " mm"
                            </p>
                        </div>
                    }
                })
            }}
        </section>
    }
}

/// Job library: save the current working job under a name, then pick
/// saved jobs from a sortable, selectable list to load or delete.
/// Files live server-side in `jobs/`.
#[component]
fn JobLibrary(
    set_last_action: WriteSignal<Option<String>>,
    on_job_loaded: impl Fn() + Copy + Send + Sync + 'static,
) -> impl IntoView {
    let (listings, set_listings) = signal::<Vec<JobListing>>(vec![]);
    let (name, set_name) = signal(String::new());
    let (note, set_note) = signal::<Option<String>>(None);
    // Which saved job the operator has highlighted in the list.
    let (selected, set_selected) = signal::<Option<String>>(None);
    // Sort state: key + ascending flag.
    let (sort_key, set_sort_key) = signal("date".to_string());
    let (sort_asc, set_sort_asc) = signal(false);

    let reload_list = move || {
        spawn_local(async move {
            if let Ok(list) = list_jobs().await {
                set_listings.set(list);
            }
        })
    };
    Effect::new(move |_| {
        reload_list();
    });

    let save = move |_| {
        let job_name = name.get();
        spawn_local(async move {
            let outcome = save_job_as(job_name.clone()).await;
            match &outcome {
                Ok(message) => set_last_action.set(Some(message.clone())),
                Err(error) => set_last_action.set(Some(format!("Save failed: {error}"))),
            }
            set_note.set(Some(match outcome {
                Ok(message) => message,
                Err(error) => format!("Save failed: {error}"),
            }));
            set_selected.set(Some(job_name));
            reload_list();
        });
    };
    let load = move |job_name: String| {
        spawn_local(async move {
            let outcome = load_job(job_name.clone()).await;
            match &outcome {
                Ok(message) => set_last_action.set(Some(message.clone())),
                Err(error) => set_last_action.set(Some(format!("Load failed: {error}"))),
            }
            set_note.set(Some(match outcome {
                Ok(message) => message,
                Err(error) => format!("Load failed: {error}"),
            }));
            // The job manager refetches the job summary and blockers.
            on_job_loaded();
        });
    };
    let delete = move |job_name: String| {
        spawn_local(async move {
            let outcome = delete_job(job_name.clone()).await;
            match &outcome {
                Ok(message) => set_last_action.set(Some(message.clone())),
                Err(error) => set_last_action.set(Some(format!("Delete failed: {error}"))),
            }
            set_note.set(Some(match outcome {
                Ok(message) => message,
                Err(error) => format!("Delete failed: {error}"),
            }));
            if selected.get_untracked().as_deref() == Some(job_name.as_str()) {
                set_selected.set(None);
            }
            reload_list();
        });
    };

    // Toggle a column's sort; a second click on the same column flips
    // the direction.
    let toggle_sort = move |key: &'static str| {
        if sort_key.get_untracked() == key {
            set_sort_asc.update(|ascending| *ascending = !*ascending);
        } else {
            set_sort_key.set(key.to_string());
            set_sort_asc.set(true);
        }
    };

    let sorted = move || {
        let mut rows = listings.get();
        let ascending = sort_asc.get();
        match sort_key.get().as_str() {
            "name" => {
                rows.sort_by(|a, b| {
                    a.name.to_ascii_lowercase().cmp(&b.name.to_ascii_lowercase())
                });
            }
            _ => {
                rows.sort_by(|a, b| a.modified_ms.cmp(&b.modified_ms));
            }
        }
        if !ascending {
            rows.reverse();
        }
        rows
    };

    let arrow = move |key: &'static str| -> &'static str {
        if sort_key.get() != key {
            return "";
        }
        if sort_asc.get() {
            " ▲"
        } else {
            " ▼"
        }
    };

    view! {
        <div class="rounded border border-slate-600 p-2">
            <p class="text-xs font-semibold text-slate-300">"Job library"</p>
            <p class="mt-1 text-xs text-slate-400">
                "Save the current job under a name, or select a saved job below and load it. \
                 Loading replaces the current job (placement list, feeders, fiducials — everything)."
            </p>
            <div class="mt-2 flex flex-wrap gap-2">
                <input
                    class="w-48 bg-slate-700 p-1 text-sm"
                    type="text"
                    placeholder="job name (e.g. rev-A-board)"
                    prop:value=name
                    on:input=move |event| set_name.set(event_target_value(&event))
                />
                <button
                    class="btn-xp !bg-emerald-800"
                    type="button"
                    disabled=move || name.get().trim().is_empty()
                    on:click=save
                >
                    "Save current job as…"
                </button>
            </div>
            {move || note.get().map(|message| view! {
                <p class="mt-2 text-xs text-slate-300">{message}</p>
            })}
            <Show when=move || !listings.get().is_empty()>
                <table class="mt-2 min-w-full text-left text-xs">
                    <thead class="bg-slate-700">
                        <tr>
                            <th class="p-1">
                                <button
                                    class="cursor-pointer font-semibold"
                                    type="button"
                                    on:click=move |_| toggle_sort("name")
                                >
                                    "Job name"
                                    {move || arrow("name")}
                                </button>
                            </th>
                            <th class="p-1">
                                <button
                                    class="cursor-pointer font-semibold"
                                    type="button"
                                    on:click=move |_| toggle_sort("date")
                                >
                                    "Last modified"
                                    {move || arrow("date")}
                                </button>
                            </th>
                        </tr>
                    </thead>
                    <tbody>
                        {move || {
                            sorted()
                                .into_iter()
                                .map(|entry| {
                                    let is_selected = selected.get() == Some(entry.name.clone());
                                    let select_name = entry.name.clone();
                                    let row_class = if is_selected {
                                        "cursor-pointer border-t border-slate-600 bg-slate-600/60"
                                    } else {
                                        "cursor-pointer border-t border-slate-600 hover:bg-slate-700/60"
                                    };
                                    view! {
                                        <tr
                                            class=row_class
                                            on:click=move |_| {
                                                set_selected
                                                    .update(|current| {
                                                        *current = if *current == Some(select_name.clone()) {
                                                            None
                                                        } else {
                                                            Some(select_name.clone())
                                                        };
                                                    });
                                            }
                                        >
                                            <td class="p-1 font-mono">{entry.name.clone()}</td>
                                            <td class="p-1 text-slate-400">{entry.modified_text}</td>
                                        </tr>
                                    }
                                })
                                .collect_view()
                        }}
                    </tbody>
                </table>
                <div class="mt-2 flex gap-2">
                    <button
                        class="btn-xp !bg-emerald-800"
                        type="button"
                        disabled=move || selected.get().is_none()
                        on:click=move |_| {
                            if let Some(job_name) = selected.get() {
                                load(job_name);
                            }
                        }
                    >
                        "Load selected"
                    </button>
                    <button
                        class="btn-xp !bg-red-900"
                        type="button"
                        disabled=move || selected.get().is_none()
                        on:click=move |_| {
                            if let Some(job_name) = selected.get() {
                                delete(job_name);
                            }
                        }
                    >
                        "Delete selected"
                    </button>
                </div>
            </Show>
        </div>
    }
}

/// One dropdown menu in the top menu bar.
#[component]
fn Menu(
    name: &'static str,
    open: ReadSignal<Option<&'static str>>,
    set_open: WriteSignal<Option<&'static str>>,
    children: ChildrenFn,
) -> impl IntoView {
    view! {
        <div class="relative">
            <button
                class=move || {
                    if open.get() == Some(name) {
                        "rounded bg-slate-600 px-3 py-1"
                    } else {
                        "rounded px-3 py-1 hover:bg-slate-700"
                    }
                }
                type="button"
                on:click=move |_| {
                    set_open
                        .update(|current| {
                            *current = if *current == Some(name) { None } else { Some(name) };
                        });
                }
            >
                {name}
            </button>
            <Show when=move || open.get() == Some(name)>
                <div
                    class="absolute left-0 top-full z-50 mt-1 min-w-48 rounded border border-slate-600 bg-slate-800 p-1 shadow-xl"
                    on:click=move |_| set_open.set(None)
                >
                    {children()}
                </div>
            </Show>
        </div>
    }
}

/// Top menu bar: the entry points a full PnP suite exposes, grouped the
/// way operators expect them.
#[component]
fn MenuBar(
    machine_state: ReadSignal<MachineState>,
    set_tab: WriteSignal<String>,
    set_show_settings: WriteSignal<bool>,
) -> impl IntoView {
    let (job_cmd, set_job_cmd) = (
        use_context::<ReadSignal<(u64, String)>>().expect("job command context"),
        use_context::<WriteSignal<(u64, String)>>().expect("job command writer"),
    );
    let (open, set_open) = signal::<Option<&'static str>>(None);
    let home_action = ServerAction::<RunAction>::new();
    let pcb_in_action = ServerAction::<RunAction>::new();
    let pcb_out_action = ServerAction::<RunAction>::new();

    let send_job = move |kind: &'static str| {
        let (sequence, _) = job_cmd.get();
        set_job_cmd.set((sequence + 1, kind.to_string()));
    };
    let go_tab = move |tab: &'static str| set_tab.set(tab.to_string());
    let running = move || matches!(machine_state.get(), MachineState::RunningJob(_));
    let ready = move || machine_state.get() == MachineState::Ready;
    let item = "block w-full px-3 py-1 text-left text-sm hover:bg-slate-600";

    view! {
        <nav class="flex flex-wrap items-center gap-1 rounded border border-slate-600 bg-slate-800 px-2 py-1 text-sm">
            <span class="mr-3 font-bold tracking-wide">"⬢ PnP Studio"</span>
            <Menu name="View" open=open set_open=set_open>
                <button class=item type="button" on:click=move |_| go_tab("placements")>
                    "PCB placement list"
                </button>
                <button class=item type="button" on:click=move |_| go_tab("process")>
                    "Process visualization"
                </button>
                <button class=item type="button" on:click=move |_| go_tab("job")>
                    "Job manager"
                </button>
                <button class=item type="button" on:click=move |_| go_tab("jobcfg")>
                    "Job configuration"
                </button>
                <button class=item type="button" on:click=move |_| go_tab("parts")>
                    "Parts & footprints"
                </button>
                <button class=item type="button" on:click=move |_| go_tab("machine")>
                    "Machine setup"
                </button>
            </Menu>
            <Menu name="Job" open=open set_open=set_open>
                <button class=item type="button" on:click=move |_| send_job("plan")>
                    "Plan job from placement list"
                </button>
                <button class=item type="button" on:click=move |_| send_job("run") disabled=running>
                    "Run job"
                </button>
                <button class=item type="button" on:click=move |_| send_job("stop")>
                    "Stop job"
                </button>
            </Menu>
            <Menu name="Machine" open=open set_open=set_open>
                <ActionForm action=home_action>
                    <input type="hidden" name="action" value="home" />
                    <button class=item type="submit" disabled=running>
                        "Home machine"
                    </button>
                </ActionForm>
                <ActionForm action=pcb_in_action>
                    <input type="hidden" name="action" value="conveyor_clip_pcb" />
                    <button class=item type="submit" disabled=move || !ready()>
                        "Load PCB (conveyor in)"
                    </button>
                </ActionForm>
                <ActionForm action=pcb_out_action>
                    <input type="hidden" name="action" value="conveyor_unclip_and_move" />
                    <button class=item type="submit" disabled=move || !ready()>
                        "Unload PCB (conveyor out)"
                    </button>
                </ActionForm>
            </Menu>
            <Menu name="Settings" open=open set_open=set_open>
                <button class=item type="button" on:click=move |_| set_show_settings.set(true)>
                    "Calibration & machine settings…"
                </button>
            </Menu>
        </nav>
    }
}

#[island]
fn Home() -> impl IntoView {
    // The single source of truth every tab reads from. Provided here,
    // not in `App`, because in islands mode only islands like `Home`
    // run on the client — context provided in `App` would be missing
    // after hydration.
    let (app_refresh, set_app_refresh) = signal(0u64);
    let app_interval = use_interval(1000u64);
    let app_snapshot = Resource::new(
        move || (app_interval.counter.get(), app_refresh.get()),
        |_| async move { get_app_state().await },
    );
    provide_context(AppData {
        snapshot: app_snapshot,
        refresh_signal: set_app_refresh,
    });
    let (machine_state, set_machine_state) = signal(MachineState::default());
    let (camera_enabled, set_camera_enabled) = signal(true);
    provide_context(camera_enabled);
    provide_context(set_camera_enabled);
    let (last_action, set_last_action) = signal::<Option<String>>(None);
    let (show_home_prompt, set_show_home_prompt) = signal(true);
    let (show_settings, set_show_settings) = signal(false);
    // Which board-editor tab is visible; shared with the menu bar.
    let (tab, set_tab) = signal("placements".to_string());
    // Sequenced job commands from the menu bar: (sequence, kind). The
    // job manager reacts to every sequence change by planning, running
    // or stopping the job.
    let (job_cmd, set_job_cmd) = signal((0u64, String::new()));
    // Sequenced plan commands: settings components (machine setup, part
    // library) ask for an automatic re-optimisation by bumping this.
    let (plan_cmd, set_plan_cmd) = signal((0u64, String::new()));
    provide_context(tab);
    provide_context(set_tab);
    provide_context(job_cmd);
    provide_context(set_job_cmd);
    provide_context(plan_cmd);
    provide_context(set_plan_cmd);

    Effect::new(move |_| {
        if !matches!(machine_state.get(), MachineState::Unknown) {
            set_show_home_prompt.set(false);
        }
    });

    view! {
        <Title text="Leptos + Tailwindcss" />
        <main>
            <div class="min-h-screen w-screen bg-slate-900 p-2 text-white md:flex md:h-screen md:flex-col md:gap-4 md:overflow-hidden">
                <MenuBar
                    machine_state=machine_state
                    set_tab=set_tab
                    set_show_settings=set_show_settings
                />
                <div class="flex min-w-0 flex-col gap-4 md:min-h-0 md:flex-1 md:flex-row">
                    <div class="flex min-w-0 flex-col gap-4 md:w-fit md:shrink-0 md:overflow-auto">
                        <CameraView />
                        <div class="grid min-w-0 grid-cols-[repeat(auto-fit,minmax(12rem,1fr))] gap-2 rounded border border-slate-600 bg-slate-800 p-2">
                            <MoveButton machine_state=machine_state set_last_action=set_last_action />
                            <RelativeMoveButton
                                machine_state=machine_state
                                set_last_action=set_last_action
                            />
                            <ZJogControls set_last_action=set_last_action />
                            <ConveyorManager
                                machine_state=machine_state
                                set_last_action=set_last_action
                            />
                            <div class="rounded border border-slate-600 bg-slate-800 p-2">
                                <button class="btn-xp w-full" on:click=move |_| set_show_settings.set(true)>
                                    "Settings"
                                </button>
                            </div>
                        </div>
                    </div>

                    <aside class="relative min-h-40 min-w-0 flex-none overflow-y-auto rounded border border-slate-600 bg-slate-800 p-3 md:flex-1 md:min-h-0">
                        <div class="absolute right-3 top-3">
                            <MachineStatus set_machine_state=set_machine_state />
                        </div>
                        <BoardEditor
                            machine_state=machine_state
                            set_last_action=set_last_action
                            tab=tab
                            set_tab=set_tab
                        />
                    // <p class="mt-2 text-sm text-slate-300">"Job controls will appear here."</p>
                    </aside>
                </div>
                <div class="shrink-0 rounded border border-slate-600 bg-slate-800 p-2 text-sm">
                    <span class="font-semibold">"Last action: "</span>
                    {move || last_action.get().unwrap_or_else(|| "No action yet".into())}
                </div>
            </div>
            {move || {
                show_home_prompt
                    .get()
                    .then(|| {

                        view! {
                            <HomePrompt
                                machine_state=machine_state
                                set_last_action=set_last_action
                                set_home_prompt=set_show_home_prompt
                            />
                        }
                    })
            }}
            {move || {
                show_settings
                    .get()
                    .then(|| {
                        view! {
                            <SettingsMenu
                                set_show_settings=set_show_settings
                                machine_state=machine_state
                                set_last_action=set_last_action
                            />
                        }
                    })
            }}

        </main>
    }
}

#[cfg(test)]
mod unquote_csv_tests {
    use super::{PlacementColumn, unquote_csv};

    #[test]
    fn strips_one_matching_pair_of_quotes() {
        assert_eq!(unquote_csv("\"C_0402_1005Metric\""), "C_0402_1005Metric");
        assert_eq!(unquote_csv("plain"), "plain");
        // A lone quote, or quotes that don't wrap the cell, stay as-is.
        assert_eq!(unquote_csv("\"odd"), "\"odd");
        assert_eq!(unquote_csv("a\"b"), "a\"b");
        // Doubled quotes inside a quoted field escape a real quote.
        assert_eq!(unquote_csv("\"say \"\"hi\"\"\""), "say \"hi\"");
    }

    #[test]
    fn recognizes_the_kicad_val_header_as_the_value_column() {
        // KiCad pick-and-place files call the value column "Val"; an
        // unmatched header silently drops the values, and every planned
        // part then gets a footprint-only MPN.
        assert!(matches!(
            PlacementColumn::from_header("Val"),
            PlacementColumn::Value
        ));
        assert!(matches!(
            PlacementColumn::from_header("VAL"),
            PlacementColumn::Value
        ));
    }
}
