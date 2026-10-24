//! Persistence for machine calibration.
//!
//! Stored in the same SQLite file as the serial/camera settings so there
//! is one place to back up. `camera-calibration.json` and
//! `feeder-pocket-vision.json` predate this module and are still written as
//! files; nozzle calibration uses the database instead.
//!
//! Schema is created on demand, so pointing at a fresh or existing `db.db`
//! both work and no migration step is needed.

use rusqlite::{params, Connection};

use pnp_logic::{MachineContext, NozzleId, NozzleOffset};

/// Settings-table key under which the machine context (cameras, machine
/// config, feeder setup, feeder calibration) is stored as one JSON blob.
const MACHINE_CONTEXT_KEY: &str = "machine_setup";

/// Path of the settings database, relative to the working directory.
pub const DB_PATH: &str = "db.db";

/// Open the calibration database. Every logging path opens a short-lived
/// connection rather than sharing one across threads: SQLite handles it
/// fine and it keeps the API free of connection lifetimes.
pub fn open_connection() -> rusqlite::Result<Connection> {
    Connection::open(DB_PATH)
}

/// Ensure the tables this module needs exist.
///
/// `CREATE TABLE IF NOT EXISTS` is safe to call repeatedly, so callers can
/// just invoke it on every connection rather than tracking a version.
pub fn ensure_schema(connection: &Connection) -> rusqlite::Result<()> {
    connection.execute_batch(
        "CREATE TABLE IF NOT EXISTS settings (
             key TEXT PRIMARY KEY NOT NULL,
             value TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS nozzle_calibration (
             nozzle_id INTEGER PRIMARY KEY NOT NULL,
             offset_mm REAL NOT NULL,
             runout_mm REAL NOT NULL,
             calibrated_at TEXT NOT NULL DEFAULT (datetime('now'))
         );
         CREATE TABLE IF NOT EXISTS parts (
             id INTEGER PRIMARY KEY NOT NULL,
             status TEXT NOT NULL,
             data TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS footprint_heights (
             footprint TEXT PRIMARY KEY NOT NULL,
             data TEXT NOT NULL
         );
         CREATE TABLE IF NOT EXISTS job_runs (
             id INTEGER PRIMARY KEY AUTOINCREMENT NOT NULL,
             job_name TEXT NOT NULL DEFAULT '',
             started_at TEXT NOT NULL DEFAULT (datetime('now')),
             finished_at TEXT,
             placement_count INTEGER NOT NULL DEFAULT 0,
             placed_count INTEGER NOT NULL DEFAULT 0,
             duration_secs REAL NOT NULL DEFAULT 0,
             cph REAL NOT NULL DEFAULT 0,
             status TEXT NOT NULL DEFAULT 'Running'
         );",
    )
}

/// Load the whole parts library from the database. Each row stores the
/// part as a JSON blob plus its trust status as a column.
pub fn load_parts(connection: &Connection) -> rusqlite::Result<Vec<pnp_logic::SetupPart>> {
    ensure_schema(connection)?;
    let mut statement = connection.prepare("SELECT id, data, status FROM parts ORDER BY id")?;
    let rows = statement.query_map([], |row| {
        Ok((
            row.get::<_, i64>(0)?,
            row.get::<_, String>(1)?,
            row.get::<_, String>(2)?,
        ))
    })?;
    let mut parts = Vec::new();
    for row in rows {
        let (id, data, status) = row?;
        let mut part: pnp_logic::Part =
            serde_json::from_str(&data).map_err(|e| rusqlite::Error::FromSqlConversionFailure(
                0,
                rusqlite::types::Type::Text,
                e.into(),
            ))?;
        // The id lives in the row, not only in the blob, so a hand-edited
        // database cannot desynchronise from the index.
        part.id = pnp_logic::PartId(id as u32);
        let status = match status.as_str() {
            "Confirmed" => pnp_logic::PartStatus::Confirmed,
            "NeedsReview" => pnp_logic::PartStatus::NeedsReview,
            _ => pnp_logic::PartStatus::Guessed,
        };
        parts.push(pnp_logic::SetupPart { part, status });
    }
    Ok(parts)
}

/// Write the whole parts library to the database, replacing previous
/// contents. The in-memory library is the authority, so a full replace
/// keeps both views identical.
pub fn save_all_parts(
    connection: &Connection,
    parts: &[pnp_logic::SetupPart],
) -> rusqlite::Result<()> {
    ensure_schema(connection)?;
    let transaction = connection.unchecked_transaction()?;
    transaction.execute("DELETE FROM parts", [])?;
    {
        let mut statement = transaction.prepare(
            "INSERT INTO parts (id, status, data) VALUES (?1, ?2, ?3)",
        )?;
        for entry in parts {
            let data = serde_json::to_string(&entry.part)
                .map_err(|e| rusqlite::Error::ToSqlConversionFailure(e.into()))?;
            let status = match entry.status {
                pnp_logic::PartStatus::Confirmed => "Confirmed",
                pnp_logic::PartStatus::NeedsReview => "NeedsReview",
                pnp_logic::PartStatus::Guessed => "Guessed",
            };
            statement.execute(params![entry.part.id.0, status, data])?;
        }
    }
    transaction.commit()
}

/// Load all recorded footprint heights.
pub fn load_heights(
    connection: &Connection,
) -> rusqlite::Result<std::collections::HashMap<String, crate::part_heights::FootprintHeights>> {
    ensure_schema(connection)?;
    let mut statement =
        connection.prepare("SELECT footprint, data FROM footprint_heights ORDER BY footprint")?;
    let rows = statement.query_map([], |row| {
        Ok((row.get::<_, String>(0)?, row.get::<_, String>(1)?))
    })?;
    let mut map = std::collections::HashMap::new();
    for row in rows {
        let (footprint, data) = row?;
        let entry: crate::part_heights::FootprintHeights =
            serde_json::from_str(&data).map_err(|e| rusqlite::Error::FromSqlConversionFailure(
                0,
                rusqlite::types::Type::Text,
                e.into(),
            ))?;
        map.insert(footprint, entry);
    }
    Ok(map)
}

/// Upsert one footprint's recorded heights.
pub fn save_height(
    connection: &Connection,
    footprint: &str,
    entry: &crate::part_heights::FootprintHeights,
) -> rusqlite::Result<()> {
    ensure_schema(connection)?;
    let data = serde_json::to_string(entry)
        .map_err(|e| rusqlite::Error::ToSqlConversionFailure(e.into()))?;
    connection.execute(
        "INSERT INTO footprint_heights (footprint, data) VALUES (?1, ?2)
         ON CONFLICT(footprint) DO UPDATE SET data = excluded.data",
        params![footprint, data],
    )?;
    Ok(())
}

/// Write a complete nozzle calibration, replacing any previous one.
///
/// Replaces rather than merges: a calibration describes all four nozzles
/// as one consistent set, and a partial merge could pair new offsets with
/// stale ones from a different run.
pub fn save_nozzle_offsets(
    connection: &Connection,
    offsets: &[NozzleOffset],
) -> rusqlite::Result<()> {
    ensure_schema(connection)?;
    let transaction = connection.unchecked_transaction()?;
    transaction.execute("DELETE FROM nozzle_calibration", [])?;
    {
        let mut statement = transaction.prepare(
            "INSERT INTO nozzle_calibration (nozzle_id, offset_mm, runout_mm)
             VALUES (?1, ?2, ?3)",
        )?;
        for offset in offsets {
            statement.execute(params![
                offset.nozzle.id(),
                offset.offset_mm,
                offset.runout_mm
            ])?;
        }
    }
    transaction.commit()
}

/// Read the stored calibration, ordered by nozzle id.
///
/// Returns `Ok(None)` when nothing has been calibrated yet, which is
/// distinct from a stored empty set.
pub fn load_nozzle_offsets(
    connection: &Connection,
) -> rusqlite::Result<Option<Vec<NozzleOffset>>> {
    ensure_schema(connection)?;
    let mut statement = connection.prepare(
        "SELECT nozzle_id, offset_mm, runout_mm
         FROM nozzle_calibration
         ORDER BY nozzle_id",
    )?;
    let rows = statement.query_map([], |row| {
        let nozzle_id: u8 = row.get(0)?;
        let offset_mm: f64 = row.get(1)?;
        let runout_mm: f64 = row.get(2)?;
        Ok(NozzleOffset {
            nozzle: nozzle_id_from_db(nozzle_id).ok_or_else(|| {
                rusqlite::Error::InvalidColumnType(
                    0,
                    format!("nozzle id {nozzle_id} is outside the addressable range"),
                    rusqlite::types::Type::Integer,
                )
            })?,
            offset_mm,
            runout_mm,
        })
    })?;

    let mut offsets = Vec::new();
    for row in rows {
        offsets.push(row?);
    }
    Ok((!offsets.is_empty()).then_some(offsets))
}

/// Map the stored `nozzle_id` onto a [`NozzleId`].
///
/// `NozzleId::id()` numbers nozzles from 1, and only the first four are
/// addressable on this machine, so anything outside 1..=4 is a corrupt row
/// rather than a nozzle we should silently skip.
fn nozzle_id_from_db(id: u8) -> Option<NozzleId> {
    match id {
        1 => Some(NozzleId::N0),
        2 => Some(NozzleId::N1),
        3 => Some(NozzleId::N2),
        4 => Some(NozzleId::N3),
        _ => None,
    }
}

/// Load the stored machine context, if one has been saved.
///
/// Returns `Ok(None)` when the machine has never had its settings saved,
/// which is distinct from a stored (possibly default) context. A blob
/// that cannot be parsed is an error: silently discarding it would make
/// the machine forget its cameras and speeds.
pub fn load_machine_context(
    connection: &Connection,
) -> rusqlite::Result<Option<MachineContext>> {
    ensure_schema(connection)?;
    let value: Option<String> = connection
        .query_row(
            "SELECT value FROM settings WHERE key = ?1",
            [MACHINE_CONTEXT_KEY],
            |row| row.get(0),
        )
        .map(Some)
        .or_else(|error| match error {
            rusqlite::Error::QueryReturnedNoRows => Ok(None),
            other => Err(other),
        })?;
    match value {
        None => Ok(None),
        Some(value) => {
            let context: MachineContext = serde_json::from_str(&value).map_err(|e| {
                rusqlite::Error::FromSqlConversionFailure(
                    0,
                    rusqlite::types::Type::Text,
                    e.into(),
                )
            })?;
            Ok(Some(context))
        }
    }
}

/// Write the machine context, replacing any previous one.
pub fn save_machine_context(
    connection: &Connection,
    context: &MachineContext,
) -> rusqlite::Result<()> {
    ensure_schema(connection)?;
    let data = serde_json::to_string(context)
        .map_err(|e| rusqlite::Error::ToSqlConversionFailure(e.into()))?;
    connection.execute(
        "INSERT INTO settings (key, value) VALUES (?1, ?2)
         ON CONFLICT(key) DO UPDATE SET value = excluded.value",
        params![MACHINE_CONTEXT_KEY, data],
    )?;
    Ok(())
}

/// Begin a job-run record. Returns the row id, which the finisher needs
/// to update the same record instead of inserting a second one.
pub fn start_job_run(
    connection: &Connection,
    job_name: &str,
    placement_count: usize,
) -> rusqlite::Result<i64> {
    ensure_schema(connection)?;
    connection.execute(
        "INSERT INTO job_runs (job_name, placement_count, status) VALUES (?1, ?2, 'Running')",
        params![job_name, placement_count as i64],
    )?;
    Ok(connection.last_insert_rowid())
}

/// Close a job-run record with its outcome. `cph` is the achieved rate
/// (placements per hour over the wall-clock run time).
pub fn finish_job_run(
    connection: &Connection,
    run_id: i64,
    placed_count: usize,
    duration_secs: f64,
    status: &str,
) -> rusqlite::Result<()> {
    ensure_schema(connection)?;
    let cph = if duration_secs > 0.0 {
        placed_count as f64 / duration_secs * 3600.0
    } else {
        0.0
    };
    connection.execute(
        "UPDATE job_runs
         SET finished_at = datetime('now'), placed_count = ?1,
             duration_secs = ?2, cph = ?3, status = ?4
         WHERE id = ?5",
        params![
            placed_count as i64,
            duration_secs,
            cph,
            status,
            run_id
        ],
    )?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> Vec<NozzleOffset> {
        vec![
            NozzleOffset { nozzle: NozzleId::N0, offset_mm: -30.12, runout_mm: 0.21 },
            NozzleOffset { nozzle: NozzleId::N1, offset_mm: -9.95, runout_mm: 0.18 },
            NozzleOffset { nozzle: NozzleId::N2, offset_mm: 10.08, runout_mm: 0.25 },
            NozzleOffset { nozzle: NozzleId::N3, offset_mm: 30.01, runout_mm: 0.19 },
        ]
    }

    /// Round-trips offsets through the database.
    #[test]
    fn offsets_round_trip() {
        let connection = Connection::open_in_memory().unwrap();
        let offsets = sample();
        save_nozzle_offsets(&connection, &offsets).unwrap();
        let loaded = load_nozzle_offsets(&connection).unwrap().unwrap();
        assert_eq!(loaded, offsets);
    }

    /// No calibration yet is distinct from an empty one.
    #[test]
    fn missing_calibration_is_none() {
        let connection = Connection::open_in_memory().unwrap();
        assert_eq!(load_nozzle_offsets(&connection).unwrap(), None);
    }

    /// A re-run replaces the old set rather than accumulating rows.
    #[test]
    fn saving_twice_replaces() {
        let connection = Connection::open_in_memory().unwrap();
        save_nozzle_offsets(&connection, &sample()).unwrap();

        let corrected = vec![NozzleOffset {
            nozzle: NozzleId::N0,
            offset_mm: -29.8,
            runout_mm: 0.22,
        }];
        save_nozzle_offsets(&connection, &corrected).unwrap();

        let loaded = load_nozzle_offsets(&connection).unwrap().unwrap();
        assert_eq!(loaded, corrected, "stale rows from the first run survived");
    }

    /// Works against a file that predates this module.
    #[test]
    fn schema_is_created_on_demand() {
        let connection = Connection::open_in_memory().unwrap();
        // An existing settings table, as an older db.db would have.
        connection
            .execute_batch("CREATE TABLE settings (key TEXT PRIMARY KEY NOT NULL, value TEXT NOT NULL)")
            .unwrap();
        save_nozzle_offsets(&connection, &sample()).unwrap();
        assert_eq!(load_nozzle_offsets(&connection).unwrap().unwrap().len(), 4);
    }

    /// A nozzle id outside the addressable range is a corrupt row, not
    /// something to skip quietly.
    #[test]
    fn corrupt_nozzle_id_is_rejected() {
        let connection = Connection::open_in_memory().unwrap();
        ensure_schema(&connection).unwrap();
        connection
            .execute(
                "INSERT INTO nozzle_calibration (nozzle_id, offset_mm, runout_mm)
                 VALUES (9, 0.0, 0.1)",
                [],
            )
            .unwrap();
        assert!(load_nozzle_offsets(&connection).is_err());
    }

    /// The machine context round-trips through the settings table, and a
    /// fresh database reports "never saved" rather than a default.
    #[test]
    fn machine_context_round_trip() {
        let connection = Connection::open_in_memory().unwrap();
        assert!(load_machine_context(&connection).unwrap().is_none());

        let mut context = MachineContext::default();
        context.cameras.push(pnp_logic::Camera {
            id: 1,
            kind: pnp_logic::config::CameraKind::Precise,
            position: pnp_logic::Point::new(2.0, 3.0),
            array_pitch_mm: None,
        });
        save_machine_context(&connection, &context).unwrap();

        let loaded = load_machine_context(&connection).unwrap().unwrap();
        assert_eq!(loaded.cameras.len(), 1);
        assert_eq!(loaded.cameras[0].position.y, 3.0);

        // Saving again replaces, not duplicates.
        let empty = MachineContext::default();
        save_machine_context(&connection, &empty).unwrap();
        assert!(load_machine_context(&connection)
            .unwrap()
            .unwrap()
            .cameras
            .is_empty());
    }
}