//! Per-footprint motion heights.
//!
//! Pick-and-place machines need to know how far down the nozzle must
//! travel to pick a component off its feeder and how far down it must go
//! to seat that component on the board. Those distances depend on the
//! component body, so they are recorded per footprint and replayed by the
//! job runner whenever a placement with that footprint is processed.
//!
//! The map is keyed by footprint name and persisted as pretty JSON next
//! to the other stores (`part-heights.json`).

use std::collections::HashMap;
use std::sync::{Arc, Mutex};

use rusqlite::Connection;

/// Recorded motion parameters for one footprint.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct FootprintHeights {
    /// Z the nozzle must reach to grab the part off the feeder (mm).
    #[serde(default)]
    pub pickup_z_mm: f64,
    /// Z the nozzle must reach to seat the part on the board (mm).
    #[serde(default)]
    pub place_z_mm: f64,
    /// Body width used for drawing and vision sizing (mm).
    #[serde(default)]
    pub width_mm: f64,
    /// Body height used for drawing and vision sizing (mm).
    #[serde(default)]
    pub height_mm: f64,
}

/// Footprint name -> recorded parameters.
pub type HeightMap = HashMap<String, FootprintHeights>;

const HEIGHTS_PATH: &str = "part-heights.json";

struct HeightStoreInner {
    map: HeightMap,
    dirty: bool,
}

/// Thread-safe handle around the persisted heights map.
#[derive(Clone)]
pub struct HeightStore {
    inner: Arc<Mutex<HeightStoreInner>>,
}

impl HeightStore {
    /// Load recorded heights from the SQLite database. A legacy
    /// `part-heights.json` is imported once if the database is empty;
    /// the file is left in place as a backup.
    pub fn load() -> Result<Self, String> {
        let connection = Connection::open(crate::calibration_store::DB_PATH)
            .map_err(|error| format!("unable to open {}: {error}", crate::calibration_store::DB_PATH))?;
        let mut map = crate::calibration_store::load_heights(&connection)
            .map_err(|error| format!("unable to load recorded heights: {error}"))?;
        if map.is_empty() {
            if let Ok(contents) = std::fs::read_to_string(HEIGHTS_PATH) {
                if let Ok(imported) = serde_json::from_str::<HeightMap>(&contents) {
                    if !imported.is_empty() {
                        for (footprint, entry) in &imported {
                            crate::calibration_store::save_height(&connection, footprint, entry)
                                .map_err(|error| format!("unable to import {HEIGHTS_PATH}: {error}"))?;
                        }
                        map = imported;
                    }
                }
            }
        }
        Ok(Self {
            inner: Arc::new(Mutex::new(HeightStoreInner { map, dirty: false })),
        })
    }

    /// A snapshot of every recorded footprint.
    pub fn map(&self) -> HeightMap {
        self.inner.lock().expect("heights lock").map.clone()
    }

    /// Insert or update one footprint's parameters and flush to the
    /// database.
    pub fn save_entry(&self, footprint: &str, entry: FootprintHeights) -> Result<(), String> {
        if footprint.trim().is_empty() {
            return Err("footprint name is empty".to_string());
        }
        let mut guard = self.inner.lock().expect("heights lock");
        guard
            .map
            .insert(footprint.trim().to_string(), entry.clone());
        guard.dirty = true;
        let connection = Connection::open(crate::calibration_store::DB_PATH)
            .map_err(|error| format!("unable to open {}: {error}", crate::calibration_store::DB_PATH))?;
        crate::calibration_store::save_height(&connection, footprint.trim(), &entry)
            .map_err(|error| format!("unable to save recorded heights: {error}"))?;
        guard.dirty = false;
        Ok(())
    }
}
