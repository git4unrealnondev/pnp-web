//! Persistence for jobs and the part library.
//!
//! A *job* is everything needed to build a board: the placement list, the
//! feeder layout, the head geometry and the machine configuration. It is
//! stored as a [`JobFile`], which is the same structure `pnp-logic`
//! serialises for its own tests, so a job saved here can be loaded by the
//! runner without conversion.
//!
//! A *part library* is the set of parts the operator has defined or had
//! derived from a footprint name. It is kept separate from the job so a
//! part can be reused across boards, and so a part can be created and
//! tested before it is used in a placement.
//!
//! Both are written to disk after every change, so a crash or power loss
//! loses at most the last edit rather than the whole job.

use std::path::PathBuf;
use std::sync::{Arc, Mutex};

use rusqlite::Connection;

use pnp_logic::{JobFile, PartId, PartStatus, HeadSpec};

/// Where jobs are stored, relative to the working directory.
pub const JOB_PATH: &str = "job.json";

/// Where the part library *used to be* stored. The library now lives in
/// the SQLite database (`db.db`); this path is only used for a one-time
/// import of an existing library, and the file is left untouched as a
/// backup.
pub const PARTS_PATH: &str = "parts.json";

/// Test-only helper: tests that touch the store change the process-wide
/// working directory, so they must share one lock to avoid interleaving
/// relative paths. Lives here so every store-touching test module can
/// reach it (`crate::job_store::test_cwd_lock`).
#[cfg(test)]
pub(crate) fn test_cwd_lock() -> std::sync::MutexGuard<'static, ()> {
    static CWD_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());
    CWD_LOCK.lock().unwrap_or_else(|error| error.into_inner())
}

/// Persistent, shared handle to the current job and part library.
///
/// Kept behind a mutex because server functions run concurrently: one
/// request may be writing the job while another reads it for display.
#[derive(Clone)]
pub struct Store {
    inner: Arc<Mutex<StoreInner>>,
}

struct StoreInner {
    job: JobFile,
    /// Parts the operator has defined or had derived, keyed by part id.
    parts: Vec<pnp_logic::SetupPart>,
    job_dirty: bool,
    parts_dirty: bool,
    /// Library file the operator is editing (`jobs/<name>.json`), when
    /// one is loaded. Machine-setup saves write the nozzle setup back
    /// into it so the named job stays current without an explicit
    /// "Save as".
    loaded_job_name: Option<String>,
}

impl Store {
    /// Open the store, loading any previously saved job and parts.
    ///
    /// A missing file is not an error: it simply means nothing has been
    /// saved yet. A *corrupt* file is reported, because silently starting
    /// empty would let the operator believe their job was gone.
    ///
    /// The part library lives in the SQLite database. If the database has
    /// no parts yet but a legacy `parts.json` exists, it is imported once
    /// and kept as a backup file.
    pub fn load() -> Result<Self, String> {
        let job = match std::fs::read_to_string(JOB_PATH) {
            Ok(contents) => serde_json::from_str(&contents).map_err(|error| {
                format!("{JOB_PATH} is not valid JSON: {error}; fix or remove it to start a new job")
            })?,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                // No saved job — start fresh with empty job data
                let inner = StoreInner {
                    job: JobFile {
                        board_id: String::new(),
                        track_width_mm: None,
                        feeders: vec![],
                        banks: vec![],
                        head: HeadSpec { nozzles: vec![] },
                        placements: vec![],
                        feeder_assignment: pnp_logic::FeederAssignment::default(),
                        fiducials: vec![],
                        board_mapping: None,
                        estimated_seconds: None,
                        placement_columns: Vec::new(),
                    },
                    parts: vec![],
                    job_dirty: false,
                    parts_dirty: false,
                    loaded_job_name: None,
                };
                return Ok(Self {
                    inner: Arc::new(Mutex::new(inner)),
                });
            }
            Err(error) => return Err(format!("unable to read {JOB_PATH}: {error}")),
        };

        let connection = Connection::open(crate::calibration_store::DB_PATH)
            .map_err(|error| format!("unable to open {}: {error}", crate::calibration_store::DB_PATH))?;
        crate::calibration_store::ensure_schema(&connection)
            .map_err(|error| format!("unable to prepare the database: {error}"))?;

        let mut parts = crate::calibration_store::load_parts(&connection)
            .map_err(|error| format!("unable to load the part library: {error}"))?;
        if parts.is_empty() {
            // One-time import of the legacy JSON library.
            if let Ok(contents) = std::fs::read_to_string(PARTS_PATH) {
                match serde_json::from_str::<Vec<pnp_logic::SetupPart>>(&contents) {
                    Ok(imported) if !imported.is_empty() => {
                        crate::calibration_store::save_all_parts(&connection, &imported)
                            .map_err(|error| format!("unable to import {PARTS_PATH}: {error}"))?;
                        parts = imported;
                    }
                    _ => {} // empty or unreadable legacy file — ignore
                }
            }
        }

        Ok(Self {
            inner: Arc::new(Mutex::new(StoreInner {
                job,
                parts,
                job_dirty: false,
                parts_dirty: false,
                loaded_job_name: None,
            })),
        })
    }

    /// Current job, for display and editing.
    pub fn job(&self) -> JobFile {
        self.inner.lock().unwrap().job.clone()
    }

    /// Name of the library job being edited (`jobs/<name>.json`), if one
    /// is loaded.
    pub fn loaded_job_name(&self) -> Option<String> {
        self.inner.lock().unwrap().loaded_job_name.clone()
    }

    /// Record (or clear) the library job being edited. Callers pass the
    /// sanitised name, matching the file stem on disk.
    pub fn set_loaded_job_name(&self, name: Option<String>) {
        self.inner.lock().unwrap().loaded_job_name = name;
    }

    /// Replace the job and persist it.
    pub fn save_job(&self, job: JobFile) -> Result<(), String> {
        let mut inner = self.inner.lock().unwrap();
        inner.job = job;
        inner.job_dirty = true;
        drop(inner);
        self.flush_job()
    }

    /// Reset the working job to an empty placement list: clears the
    /// placements and fiducials (the "temp data" left over from a test
    /// or imported board) plus the feeder assignment derived from them,
    /// and forgets the loaded library job so a later machine-setup save
    /// cannot overwrite the library file with an emptied job. Feeders,
    /// banks and the head are kept — they are machine layout, not board
    /// data. Persists immediately.
    pub fn clear_placements(&self) -> Result<(), String> {
        let mut inner = self.inner.lock().unwrap();
        inner.job.placements.clear();
        inner.job.fiducials.clear();
        inner.job.feeder_assignment = pnp_logic::FeederAssignment::default();
        inner.loaded_job_name = None;
        inner.job_dirty = true;
        drop(inner);
        self.flush_job()
    }

    /// Parts in the library, for display and editing.
    pub fn parts(&self) -> Vec<pnp_logic::SetupPart> {
        self.inner.lock().unwrap().parts.clone()
    }

    /// Add or replace a part, then persist.
    pub fn save_part(&self, part: pnp_logic::SetupPart) -> Result<(), String> {
        let mut inner = self.inner.lock().unwrap();
        let id = part.part.id;
        match inner.parts.iter_mut().find(|entry| entry.part.id == id) {
            Some(slot) => *slot = part,
            None => inner.parts.push(part),
        }
        inner.parts_dirty = true;
        drop(inner);
        self.flush_parts()
    }

    /// Remove every placement that resolves to `part`, then persist.
    ///
    /// Returns the references of the removed placements. Used by the
    /// delete-part flow: leaving such placements behind would make the
    /// planner re-create the deleted part on the next re-optimisation.
    pub fn remove_placements_using(&self, part: &pnp_logic::Part) -> Vec<String> {
        let mut inner = self.inner.lock().unwrap();
        let mut removed = Vec::new();
        inner.job.placements.retain(|placement| {
            if placement_uses(&placement.part, part) {
                removed.push(placement.reference.clone());
                false
            } else {
                true
            }
        });
        inner.job_dirty = true;
        drop(inner);
        if let Err(error) = self.flush_job() {
            eprintln!("flushing job after placement removal failed: {error}");
        }
        removed
    }

    /// Remove a part from the library.
    pub fn remove_part(&self, id: PartId) -> Result<(), String> {
        let mut inner = self.inner.lock().unwrap();
        let before = inner.parts.len();
        inner.parts.retain(|entry| entry.part.id != id);
        if inner.parts.len() == before {
            return Err(format!("part {id:?} is not in the library"));
        }
        inner.parts_dirty = true;
        drop(inner);
        self.flush_parts()
    }

    /// Persist the job if it changed since the last write.
    fn flush_job(&self) -> Result<(), String> {
        let contents = {
            let mut inner = self.inner.lock().unwrap();
            if !inner.job_dirty {
                return Ok(());
            }
            let contents = serde_json::to_string_pretty(&inner.job)
                .map_err(|error| format!("unable to serialise job: {error}"))?;
            inner.job_dirty = false;
            contents
        };
        std::fs::write(JOB_PATH, contents).map_err(|error| format!("unable to write {JOB_PATH}: {error}"))
    }

    /// Persist the part library if it changed since the last write.
    /// The library lives in the SQLite database, not in a JSON file.
    fn flush_parts(&self) -> Result<(), String> {
        let snapshot = {
            let mut inner = self.inner.lock().unwrap();
            if !inner.parts_dirty {
                return Ok(());
            }
            let snapshot = inner.parts.clone();
            inner.parts_dirty = false;
            snapshot
        };
        let connection = Connection::open(crate::calibration_store::DB_PATH)
            .map_err(|error| format!("unable to open {}: {error}", crate::calibration_store::DB_PATH))?;
        crate::calibration_store::save_all_parts(&connection, &snapshot)
            .map_err(|error| format!("unable to save the part library: {error}"))
    }
}

/// Paths the store writes, exposed so the UI can show where data lives.
pub fn paths() -> (PathBuf, PathBuf) {
    (
        PathBuf::from(JOB_PATH),
        PathBuf::from(crate::calibration_store::DB_PATH),
    )
}

/// Human-readable label for a placement.
///
/// The MPN is the operator's own identifier for the part, but parts
/// imported from a placement list have an empty MPN, so fall back to the
/// designator.
fn placement_label(designator: &str, part: &pnp_logic::Part) -> String {
    if part.mpn.is_empty() {
        designator.to_string()
    } else {
        part.mpn.clone()
    }
}

/// Whether a job is runnable: every part must have confirmed or derived
/// parameters, and there must be at least one placement to make.
///
/// Returns the list of problems so the UI can show them all at once.
pub fn blockers(job: &JobFile, parts: &[pnp_logic::SetupPart]) -> Vec<String> {
    let mut problems = Vec::new();

    let unplaced = job
        .placements
        .iter()
        .filter(|placement| placement.should_place)
        .count();
    if unplaced == 0 {
        problems.push("no placements to place".into());
    }

    for placement in &job.placements {
        if !placement.should_place {
            continue;
        }
        // A part with no nozzle allowlist cannot be picked by anything,
        // so the planner would silently skip it.
        if placement.part.nozzle_type.is_empty() {
            problems.push(format!(
                "{} has no nozzle type",
                placement_label("", &placement.part)
            ));
        }
        // A part the operator has flagged as needing review must be
        // resolved before the job runs.
        if let Some(entry) = parts.iter().find(|entry| entry.part.id == placement.part.id) {
            if entry.status == PartStatus::NeedsReview {
                problems.push(format!(
                    "{} needs review before it can be placed",
                    placement_label("", &placement.part)
                ));
            }
        }
    }

    problems
}
/// Duplicates in the part library that the operator should clean up.
///
/// Two parts with the same (footprint, MPN) pair confuse the import
/// wizard: new placements match whichever comes first, so edits to one
/// copy appear to do nothing on the other. Unlike [`blockers`] these do
/// not stop the job from running; they are surfaced as warnings.
pub fn duplicate_warnings(parts: &[pnp_logic::SetupPart]) -> Vec<String> {
    use std::collections::BTreeMap;

    // Group part ids by (footprint, mpn), case-insensitively.
    let mut groups: BTreeMap<(String, String), Vec<String>> = BTreeMap::new();
    for entry in parts {
        let key = (
            entry.part.footprint.name.to_lowercase(),
            entry.part.mpn.to_lowercase(),
        );
        groups.entry(key).or_default().push(entry.part.id.0.to_string());
    }

    groups
        .into_iter()
        .filter_map(|((footprint, mpn), ids)| {
            if ids.len() < 2 {
                return None;
            }
            let label = if mpn.is_empty() {
                footprint.clone()
            } else {
                format!("{mpn}-{footprint}")
            };
            Some(format!(
                "duplicate part {label}: library entries {}",
                ids.join(", ")
            ))
        })
        .collect()
}

#[cfg(test)]
mod duplicate_warning_tests {
    use super::*;
    use pnp_logic::{Footprint, Part, PartId, PartStatus};

    fn part(id: u32, mpn: &str, footprint: &str) -> pnp_logic::SetupPart {
        pnp_logic::SetupPart {
            part: Part {
                id: PartId(id),
                mpn: mpn.to_string(),
                description: String::new(),
                footprint: Footprint {
                    name: footprint.to_string(),
                    centroid: pnp_logic::Point::ORIGIN,
                    size: pnp_logic::Size { width: 0.0, height: 0.0 },
                    pads: Vec::new(),
                    polarity: pnp_logic::Polarity::None,
                },
                nozzle_type: Vec::new(),
                pickup_rotation_deg: 0.0,
                pickup_offset: pnp_logic::Point::ORIGIN,
                camera: None,
            },
            status: PartStatus::Guessed,
        }
    }

    #[test]
    fn flags_two_parts_with_the_same_footprint_and_mpn() {
        let parts = vec![
            part(1, "100nF-C_0402_1005Metric", "C_0402_1005Metric"),
            part(2, "100nf-c_0402_1005metric", "C_0402_1005Metric"),
            part(3, "10k-R_0402_1005Metric", "C_0402_1005Metric"),
        ];
        let warnings = duplicate_warnings(&parts);
        assert_eq!(warnings.len(), 1);
        assert!(warnings[0].contains("library entries 1, 2"));
    }

    #[test]
    fn distinct_mpns_or_footprints_are_not_flagged() {
        let parts = vec![
            part(1, "100nF-C_0402_1005Metric", "C_0402_1005Metric"),
            part(2, "10k-R_0402_1005Metric", "C_0402_1005Metric"),
            part(3, "100nF-C_0402_1005Metric", "C_0603_1608Metric"),
        ];
        assert!(duplicate_warnings(&parts).is_empty());
    }
}

/// Why a part cannot be deleted right now.
///
/// The planner re-derives library entries from the job's placements
/// (each placement embeds its part's value and footprint, and matching
/// is by that key — placement ids can drift from library ids after a
/// re-plan), so deleting a part that placements still reference does
/// not stick: the next re-optimisation silently re-creates it under a
/// new id. Returns the designators of the placements using this part;
/// empty means deletion is safe.
pub fn delete_blockers(job: &JobFile, part: &pnp_logic::Part) -> Vec<String> {
    job.placements
        .iter()
        .filter(|placement| placement_uses(&placement.part, part))
        .map(|placement| placement.reference.clone())
        .collect()
}

/// Whether a placement's embedded part resolves to this library part
/// under the same rules `plan_from_rows` uses to look up library
/// entries (same id, same footprint+MPN key, or the MPN alone when the
/// operator renamed the footprint).
pub(crate) fn placement_uses(placement_part: &pnp_logic::Part, library_part: &pnp_logic::Part) -> bool {
    if placement_part.id == library_part.id {
        return true;
    }
    let combined = format!(
        "{}-{}",
        placement_part.mpn, library_part.footprint.name
    );
    placement_part
        .footprint
        .name
        .eq_ignore_ascii_case(&library_part.footprint.name)
        && (placement_part.mpn.eq_ignore_ascii_case(&library_part.mpn)
            || library_part.mpn.eq_ignore_ascii_case(&combined))
        || (!library_part.mpn.is_empty()
            && placement_part.mpn.eq_ignore_ascii_case(&library_part.mpn))
}

#[cfg(test)]
mod delete_blocker_tests {
    use super::*;
    use pnp_logic::{CameraKind, Fiducial, FiducialKind, PlacementStrategy};

    fn job_with_one_part(id: u32, mpn: &str) -> JobFile {
        let mut part = pnp_logic::part_from_footprint("0603").unwrap().part;
        part.id = PartId(id);
        part.mpn = mpn.to_string();
        JobFile {
            board_id: "test".to_string(),
            track_width_mm: None,
            feeders: vec![],
            banks: vec![],
            head: HeadSpec { nozzles: vec![] },
            placements: vec![pnp_logic::Placement {
                reference: "C5".to_string(),
                part,
                design_position: pnp_logic::Point::new(10.0, 20.0),
                design_rotation_deg: 0.0,
                feeder_slot: 1,
                nozzle: 0,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            }],
            feeder_assignment: pnp_logic::FeederAssignment::default(),
            fiducials: vec![Fiducial {
                design_position: pnp_logic::Point::new(1.0, 2.0),
                kind: FiducialKind::Panel,
                machine_position: None,
                expected_position: None,
            }],
            board_mapping: None,
            estimated_seconds: None,
            placement_columns: Vec::new(),
        }
    }

    #[test]
    fn lists_references_of_placements_using_the_part() {
        let job = job_with_one_part(7, "10k-R_0402_1005Metric");
        let part = job.placements[0].part.clone();
        assert_eq!(delete_blockers(&job, &part), vec!["C5".to_string()]);
        // Key matching too: a stale id (the planner re-resolves by
        // MPN/footprint) and the value-footprint MPN form both count.
        let mut stale = part.clone();
        stale.id = PartId(999);
        assert_eq!(delete_blockers(&job, &stale), vec!["C5".to_string()]);
        let mut renamed = part.clone();
        renamed.id = PartId(8);
        renamed.mpn = "4k7-R_0402_1005Metric".to_string();
        assert!(delete_blockers(&job, &renamed).is_empty());
    }

    #[test]
    fn removing_placements_drops_only_the_matching_rows() {
        // The store test helpers change the working directory, so share
        // the lock and use a scratch dir (same pattern as job_plan).
        let _lock = test_cwd_lock();
        let scratch = std::env::temp_dir().join(format!("job_store_delete_{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&scratch);
        std::fs::create_dir_all(&scratch).unwrap();
        let previous = std::env::current_dir().unwrap();
        std::env::set_current_dir(&scratch).unwrap();

        let job = job_with_one_part(7, "10k-R_0402_1005Metric");
        std::fs::write(JOB_PATH, serde_json::to_string(&job).unwrap()).unwrap();
        let store = Store::load().unwrap();
        let part = job.placements[0].part.clone();

        let removed = store.remove_placements_using(&part);
        assert_eq!(removed, vec!["C5".to_string()]);
        assert!(store.job().placements.is_empty());
        // The removal is persisted, not just in memory.
        let reread: JobFile =
            serde_json::from_str(&std::fs::read_to_string(JOB_PATH).unwrap()).unwrap();
        assert!(reread.placements.is_empty());

        // An unrelated part removes nothing.
        let mut other = part.clone();
        other.id = PartId(9);
        other.mpn = "4k7-R_0402_1005Metric".to_string();
        let job2 = job_with_one_part(7, "10k-R_0402_1005Metric");
        std::fs::write(JOB_PATH, serde_json::to_string(&job2).unwrap()).unwrap();
        let store2 = Store::load().unwrap();
        assert!(store2.remove_placements_using(&other).is_empty());
        std::env::set_current_dir(previous).unwrap();
        let _ = std::fs::remove_dir_all(&scratch);
    }
}
