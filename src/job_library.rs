//! Named job library.
//!
//! The working job lives in `job.json` (see `job_store`). This module
//! keeps named snapshots of jobs in the `jobs/` directory so operators
//! can save a setup, switch boards and come back later. Files are plain
//! `JobFile` JSON, same format as the working job.

use std::path::{Path, PathBuf};

use crate::job_store::Store;

const JOBS_DIR: &str = "jobs";

/// Keep names filesystem-safe: letters, digits, dash, underscore, dot.
/// Anything else is folded to `_`, and the result must be non-empty.
fn sanitize(name: &str) -> Result<String, String> {
    let cleaned: String = name
        .trim()
        .chars()
        .map(|c| {
            if c.is_ascii_alphanumeric() || c == '-' || c == '_' || c == '.' {
                c
            } else {
                '_'
            }
        })
        .collect();
    if cleaned.is_empty() || cleaned == "." || cleaned == ".." {
        return Err("job name must contain at least one letter or digit".to_string());
    }
    Ok(cleaned)
}

fn jobs_dir() -> PathBuf {
    PathBuf::from(JOBS_DIR)
}

fn job_path(name: &str) -> Result<PathBuf, String> {
    Ok(jobs_dir().join(format!("{}.json", sanitize(name)?)))
}

/// One entry of the job library listing. The type itself lives in
/// `server_fns` so the wasm client can see it too.
use crate::server_fns::JobListing;

/// Civil-from-days (Howard Hinnant's algorithm): day number since the
/// Unix epoch to (year, month, day) in the proleptic Gregorian calendar.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64; // [0, 146096]
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365; // [0, 399]
    let y = yoe as i64 + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // [0, 365]
    let mp = (5 * doy + 2) / 153; // [0, 11]
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // [1, 31]
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // [1, 12]
    (if m <= 2 { y + 1 } else { y }, m, d)
}

/// Format Unix milliseconds as "YYYY-MM-DD HH:MM" in UTC.
fn format_utc(ms: u64) -> String {
    let secs = (ms / 1000) as i64;
    let days = secs.div_euclid(86_400);
    let secs_of_day = secs.rem_euclid(86_400);
    let (year, month, day) = civil_from_days(days);
    format!(
        "{year:04}-{month:02}-{day:02} {:02}:{:02} UTC",
        secs_of_day / 3600,
        (secs_of_day % 3600) / 60
    )
}

/// Every saved job with its last-modified time, newest first. Reads the
/// directory; a missing directory simply means nothing saved yet.
pub fn list_jobs() -> Result<Vec<JobListing>, String> {
    let dir = jobs_dir();
    if !dir.exists() {
        return Ok(Vec::new());
    }
    let mut listings = Vec::new();
    let entries = std::fs::read_dir(&dir).map_err(|e| format!("cannot read {JOBS_DIR}: {e}"))?;
    for entry in entries {
        let path = entry.map_err(|e| format!("cannot read {JOBS_DIR}: {e}"))?.path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let Some(stem) = path.file_stem().and_then(|s| s.to_str()) else {
            continue;
        };
        let modified_ms = path
            .metadata()
            .and_then(|meta| meta.modified())
            .ok()
            .and_then(|time| {
                time.duration_since(std::time::UNIX_EPOCH)
                    .ok()
                    .map(|duration| duration.as_millis() as u64)
            })
            .unwrap_or(0);
        listings.push(JobListing {
            name: stem.to_string(),
            modified_ms,
            modified_text: format_utc(modified_ms),
        });
    }
    listings.sort_by(|a, b| b.modified_ms.cmp(&a.modified_ms).then(a.name.cmp(&b.name)));
    Ok(listings)
}

/// Save the store's current job under `name`, creating `jobs/` if needed.
/// The saved file becomes the one being edited, so later machine-setup
/// saves keep it current.
pub fn save_job_as(store: &Store, name: &str) -> Result<String, String> {
    let path = job_path(name)?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("cannot create {JOBS_DIR}: {e}"))?;
    }
    let job = store.job();
    let json = job
        .to_string()
        .map_err(|e| format!("cannot serialise job: {e}"))?;
    std::fs::write(&path, json).map_err(|e| format!("cannot write {}: {e}", path.display()))?;
    let saved = sanitize(name)?;
    store.set_loaded_job_name(Some(saved.clone()));
    Ok(format!("Job saved as {}", saved))
}

/// Load the named job into the store as the current working job, and
/// remember it as the library file being edited.
pub fn load_job(store: &Store, name: &str) -> Result<String, String> {
    let path = job_path(name)?;
    let text =
        std::fs::read_to_string(&path).map_err(|e| format!("cannot read {}: {e}", path.display()))?;
    let job: pnp_logic::JobFile =
        serde_json::from_str(&text).map_err(|e| format!("{} is corrupt: {e}", path.display()))?;
    store
        .save_job(job)
        .map_err(|e| format!("cannot store job: {e}"))?;
    let loaded = sanitize(name)?;
    store.set_loaded_job_name(Some(loaded.clone()));
    Ok(format!("Job {} loaded", loaded))
}

/// Write the working job's head — nozzle numbers, kinds and the
/// calibrated offsets — into the library file being edited. Keeps a
/// named job like `wasd.json` current when the operator changes nozzle
/// kinds in the machine setup without pressing "Save as".
///
/// Returns the relative path of the file that was updated, or `None`
/// when no library job is loaded (nothing to update).
pub fn sync_loaded_nozzle_kinds(store: &Store) -> Result<Option<String>, String> {
    let Some(name) = store.loaded_job_name() else {
        return Ok(None);
    };
    let path = job_path(&name)?;
    let text = std::fs::read_to_string(&path)
        .map_err(|e| format!("cannot update {}: {e}", path.display()))?;
    let mut job: pnp_logic::JobFile = serde_json::from_str(&text)
        .map_err(|e| format!("{} is corrupt: {e}", path.display()))?;
    job.head = store.job().head;
    let json = job
        .to_string()
        .map_err(|e| format!("cannot serialise job: {e}"))?;
    std::fs::write(&path, json).map_err(|e| format!("cannot write {}: {e}", path.display()))?;
    Ok(Some(path.display().to_string()))
}

/// Delete a saved job file. Only touches `jobs/`, never the working job.
/// If the deleted file was being edited, forget it so a later
/// machine-setup save doesn't silently recreate it.
pub fn delete_job(store: &Store, name: &str) -> Result<String, String> {
    let path = job_path(name)?;
    std::fs::remove_file(&path).map_err(|e| format!("cannot delete {}: {e}", path.display()))?;
    if let Ok(deleted) = sanitize(name) {
        if store.loaded_job_name().as_deref() == Some(deleted.as_str()) {
            store.set_loaded_job_name(None);
        }
    }
    Ok(format!("Job {} deleted", sanitize(name)?))
}

/// True when the directory exists — used by the UI to distinguish
/// "no saved jobs" from storage problems.
pub fn exists(name: &str) -> bool {
    matches!(job_path(name), Ok(path) if Path::new(&path).exists())
}

#[cfg(test)]
mod tests {
    use super::*;
    use pnp_logic::{FeederAssignment, HeadSpec, JobFile, NozzleKind, NozzleSpec};

    /// Restores the working directory and removes the scratch dir even
    /// if the test body panics. The functions under test build paths
    /// relative to the current directory (`jobs/`, `job.json`), so they
    /// must run inside the scratch dir.
    struct ScratchDir {
        scratch: std::path::PathBuf,
        previous: std::path::PathBuf,
    }
    impl ScratchDir {
        fn enter(tag: &str) -> Self {
            let scratch = std::env::temp_dir()
                .join(format!("job_library_{tag}_{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&scratch);
            std::fs::create_dir_all(&scratch).unwrap();
            let previous = std::env::current_dir().unwrap();
            std::env::set_current_dir(&scratch).unwrap();
            Self { scratch, previous }
        }
    }
    impl Drop for ScratchDir {
        fn drop(&mut self) {
            let _ = std::env::set_current_dir(&self.previous);
            let _ = std::fs::remove_dir_all(&self.scratch);
        }
    }

    /// Tests change the process-wide working directory for the whole
    /// crate; hold this while a test runs so parallel tests interleaving
    /// relative paths can't confuse each other.

    /// One-nozzle job, small enough to serialise in a few lines.
    fn sample_job(kind: NozzleKind) -> JobFile {
        JobFile {
            board_id: "TINY".into(),
            track_width_mm: None,
            feeders: vec![],
            banks: vec![],
            head: HeadSpec {
                nozzles: vec![NozzleSpec { id: 1, kind, offset_mm: -30.0 }],
            },
            placements: vec![],
            feeder_assignment: FeederAssignment::default(),
            fiducials: vec![],
            board_mapping: None,
            estimated_seconds: None,
            placement_columns: Vec::new(),
        }
    }

    /// Saving as, then syncing, keeps the library file's nozzle setup
    /// aligned with the working job; deleting the file forgets the name
    /// so a later sync cannot resurrect it.
    #[test]
    fn sync_writes_nozzle_kinds_into_loaded_job_file() {
        let _guard = crate::job_store::test_cwd_lock();
        let _scratch = ScratchDir::enter("sync");

        let store = Store::load().unwrap();
        let mut job = sample_job(NozzleKind::Standard);
        store.save_job(job.clone()).unwrap();
        save_job_as(&store, "wasd").unwrap();
        assert!(exists("wasd"));
        assert_eq!(store.loaded_job_name().as_deref(), Some("wasd"));

        // The operator changes the nozzle kind in the machine setup.
        job.head = HeadSpec {
            nozzles: vec![NozzleSpec { id: 1, kind: NozzleKind::N503, offset_mm: -30.0 }],
        };
        store.save_job(job).unwrap();

        let updated = sync_loaded_nozzle_kinds(&store).unwrap();
        assert!(updated.is_some(), "expected wasd.json to be updated");

        let text = std::fs::read_to_string("jobs/wasd.json").unwrap();
        let saved: JobFile = serde_json::from_str(&text).unwrap();
        assert!(matches!(saved.head.nozzles[0].kind, NozzleKind::N503));

        // After deletion there is nothing to edit, so sync must not
        // recreate the file.
        delete_job(&store, "wasd").unwrap();
        assert_eq!(store.loaded_job_name(), None);
        assert_eq!(sync_loaded_nozzle_kinds(&store).unwrap(), None);
        assert!(!exists("wasd"));
    }

    /// No library job loaded — machine-setup saves only touch the working
    /// job and must not create stray library files.
    #[test]
    fn sync_without_loaded_job_is_a_no_op() {
        let _guard = crate::job_store::test_cwd_lock();
        let _scratch = ScratchDir::enter("no_load");
        let store = Store::load().unwrap();
        assert_eq!(store.loaded_job_name(), None);
        assert_eq!(sync_loaded_nozzle_kinds(&store).unwrap(), None);
    }
}

/// Named jobs whose placements still resolve to `part`.
///
/// Deleting a library part does not touch saved job files: loading one
/// of these later re-derives the part from its placements and the part
/// comes back. The delete flow uses this to refuse with an actionable
/// message instead.
pub fn jobs_using_part(part: &pnp_logic::Part) -> Result<Vec<String>, String> {
    let dir = jobs_dir();
    if !dir.exists() {
        return Ok(Vec::new());
    }
    let mut using = Vec::new();
    let entries = std::fs::read_dir(&dir).map_err(|e| format!("cannot read {JOBS_DIR}: {e}"))?;
    for entry in entries {
        let path = entry.map_err(|e| format!("cannot read {JOBS_DIR}: {e}"))?.path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(_) => continue,
        };
        let Ok(job) = serde_json::from_str::<pnp_logic::JobFile>(&text) else {
            continue;
        };
        if job
            .placements
            .iter()
            .any(|p| crate::job_store::placement_uses(&p.part, part))
        {
            let stem = path
                .file_stem()
                .and_then(|s| s.to_str())
                .unwrap_or_default()
                .to_string();
            using.push(stem);
        }
    }
    Ok(using)
}

/// Write the working job back into the library file being edited.
///
/// Placement edits go to `job.json`; when a named job is loaded the
/// operator expects `jobs/<name>.json` to stay current too — otherwise
/// stale placements linger in the library file (and, e.g., keep parts
/// the operator deleted "in use"). Returns the updated path, or `None`
/// when no library job is loaded.
pub fn sync_loaded_job(store: &Store) -> Result<Option<String>, String> {
    let Some(name) = store.loaded_job_name() else {
        return Ok(None);
    };
    let path = job_path(&name)?;
    if let Some(parent) = path.parent() {
        std::fs::create_dir_all(parent).map_err(|e| format!("cannot create {JOBS_DIR}: {e}"))?;
    }
    let job = store.job();
    let json = job
        .to_string()
        .map_err(|e| format!("cannot serialise job: {e}"))?;
    std::fs::write(&path, json).map_err(|e| format!("cannot write {}: {e}", path.display()))?;
    Ok(Some(path.display().to_string()))
}

/// Remove every placement that resolves to `part` from every saved job
/// file in `jobs/`.
///
/// A saved job that still embeds the part would re-create it when the
/// job is loaded (the planner re-derives library entries from the
/// placements), so the delete-part flow scrubs them all. Returns the
/// names of the files that were changed, each with the number of
/// placements removed.
pub fn scrub_part_from_saved_jobs(part: &pnp_logic::Part) -> Result<Vec<String>, String> {
    let dir = jobs_dir();
    if !dir.exists() {
        return Ok(Vec::new());
    }
    let mut scrubbed = Vec::new();
    let entries = std::fs::read_dir(&dir).map_err(|e| format!("cannot read {JOBS_DIR}: {e}"))?;
    for entry in entries {
        let path = entry.map_err(|e| format!("cannot read {JOBS_DIR}: {e}"))?.path();
        if path.extension().and_then(|e| e.to_str()) != Some("json") {
            continue;
        }
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(_) => continue,
        };
        let Ok(mut job) = serde_json::from_str::<pnp_logic::JobFile>(&text) else {
            continue;
        };
        let before = job.placements.len();
        job.placements
            .retain(|p| !crate::job_store::placement_uses(&p.part, part));
        if job.placements.len() == before {
            continue;
        }
        let removed = before - job.placements.len();
        let json = job
            .to_string()
            .map_err(|e| format!("cannot serialise job: {e}"))?;
        std::fs::write(&path, json)
            .map_err(|e| format!("cannot write {}: {e}", path.display()))?;
        let stem = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or_default()
            .to_string();
        scrubbed.push(format!("{stem} ({removed} placements)"));
    }
    Ok(scrubbed)
}
