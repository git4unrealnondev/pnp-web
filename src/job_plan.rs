//! Turning the placement list into a planned job.
//!
//! The editor works in a grid of text; the runner needs a complete
//! pick-and-place plan. This module is the bridge:
//!
//! 1. every distinct footprint in the list is resolved to a part — from
//!    the library when the operator has defined it, otherwise guessed
//!    from the footprint name and added to the library for review;
//! 2. the list becomes [`PlacementNeed`]s, with rows marked as fiducials
//!    split out as board landmarks;
//! 3. the planner assigns feeders, nozzles and the run order;
//! 4. the resulting [`JobFile`] is written to `job.json`.
//!
//! The machine itself (feeders, banks, cameras, head, calibration) is
//! taken from the stored job when it has been planned before, and filled
//! in with defaults otherwise, so a first board can be planned from the
//! placement list alone.

use std::collections::HashMap;

use pnp_logic::{
    Camera, CameraKind, FeederBank, FeederKind, FeederPlacementMode, FeederSlot, Fiducial,
    FiducialKind, HeadSpec, JobFile, MachineConfig, MachineSetup, NozzleHead, NozzleSlot, NozzleSpec,
    Part, PartId, PartStatus, PlanSearchMode, PlacementNeed, PlacementStrategy, Point,
    Size, default_head, part_from_footprint, plan_job_with_mode,
};

use crate::job_store::Store;
use crate::server_fns::{PlacementRow, PlanSummary};

/// Parts bigger than this are placed with the precise camera, mirroring
/// Plan a job from the placement list and store it.
///
/// `machine` supplies the machine-side settings (cameras, machine
/// config, feeder setup) that used to be read from the job file. Only
/// job-owned data is written back into the job; the machine context is
/// never copied into it.
pub fn plan_from_rows(
    store: &Store,
    machine: &pnp_logic::MachineContext,
    board_id: String,
    columns: Vec<String>,
    rows: Vec<PlacementRow>,
) -> Result<PlanSummary, String> {
    let mut job = store.job();
    let library = store.parts();

    // 1. Resolve one part per distinct (value, footprint) pair. The
    //    footprint alone is not a safe key: "10uF" and "100nF" often
    //    share a package, and merging them would pick one reel for both
    //    and place the wrong component. Ids are assigned in sorted-key
    //    order after the library's own ids, so planning the same board
    //    twice produces the same job and keeps the operator's library
    //    entries linked.
    let mut keys: Vec<(String, String)> = rows
        .iter()
        .filter(|row| !row.fiducial)
        .map(|row| {
            (
                row.footprint.trim().to_uppercase(),
                row.value.trim().to_string(),
            )
        })
        .filter(|(footprint, _)| !footprint.is_empty())
        .collect();
    keys.sort();
    keys.dedup();

    let mut next_id = library
        .iter()
        .map(|entry| entry.part.id.0)
        .max()
        .unwrap_or(0)
        + 1;

    let label = |part: &Part| -> String {
        if part.mpn.is_empty() {
            part.footprint.name.clone()
        } else {
            part.mpn.clone()
        }
    };

    let mut parts: Vec<pnp_logic::SetupPart> = Vec::with_capacity(keys.len());
    let mut unconfirmed: Vec<String> = Vec::new();
    // Legacy entries adopted (and renamed) during this run, by part id:
    // the `library` snapshot below is not refreshed between keys, so
    // without this a second value sharing the footprint would re-adopt
    // the same blank entry and overwrite the first rename.
    let mut adopted: std::collections::HashSet<u32> = std::collections::HashSet::new();
    for (footprint, value) in &keys {
        // Original-case footprint of the first row with this key, used
        // for the guessed part's naming.
        let original = rows
            .iter()
            .find(|row| {
                !row.fiducial
                    && row.footprint.trim().eq_ignore_ascii_case(footprint)
                    && row.value.trim() == value.as_str()
            })
            .map(|row| row.footprint.trim().to_string())
            .unwrap_or_else(|| footprint.clone());
        // Library lookup, most specific first:
        //   1. same footprint AND same value — the part for this key
        //      (the value may also be an entry named value-footprint by
        //      a previous import of the same board);
        //   2. same footprint with no MPN — a legacy entry from before
        //      values were part of the key. It is adopted AND renamed to
        //      value-footprint, so stale blank-MPN entries heal on the
        //      next plan instead of hiding behind a blank MPN forever;
        //   3. same value alone — the operator renamed the part's
        //      footprint after planning;
        //   4. no value in the key: any part for this footprint.
        let mut entry: Option<pnp_logic::SetupPart> = None;
        if value.is_empty() {
            entry = library
                .iter()
                .find(|entry| entry.part.footprint.name.eq_ignore_ascii_case(footprint))
                .cloned();
        } else {
            let combined = format!("{value}-{original}");
            entry = library
                .iter()
                .find(|candidate| {
                    candidate.part.footprint.name.eq_ignore_ascii_case(footprint)
                        && (candidate.part.mpn.eq_ignore_ascii_case(value)
                            || candidate.part.mpn.eq_ignore_ascii_case(&combined))
                })
                .cloned()
                .or_else(|| {
                    library
                        .iter()
                        .find(|candidate| {
                            candidate.part.footprint.name.eq_ignore_ascii_case(footprint)
                                && candidate.part.mpn.is_empty()
                                && !adopted.contains(&candidate.part.id.0)
                        })
                        .map(|legacy| {
                            let mut renamed = legacy.clone();
                            // Same dedupe as the guess below: a value
                            // that is only the footprint fallback names
                            // the entry by footprint alone.
                            renamed.part.mpn =
                                if value.eq_ignore_ascii_case(&original) {
                                    original.clone()
                                } else {
                                    combined.clone()
                                };
                            adopted.insert(renamed.part.id.0);
                            renamed
                        })
                })
                .or_else(|| {
                    library
                        .iter()
                        .find(|candidate| candidate.part.mpn.eq_ignore_ascii_case(value))
                        .cloned()
                });
        };
        if let Some(entry) = entry {
            // A part the operator has not confirmed yet is not a
            // blocker — the job still runs — but the report should
            // say so, because a guessed nozzle or rotation places
            // parts wrong rather than not at all.
            if entry.status == PartStatus::NeedsReview {
                unconfirmed.push(format!("{} (needs review)", label(&entry.part)));
            } else if entry.status == PartStatus::Guessed {
                unconfirmed.push(format!("{} (guessed, please confirm)", label(&entry.part)));
            }
            // A renamed legacy entry must be persisted, or every plan
            // would re-adopt it as blank and the MPN would never stick.
            if entry.part.mpn.starts_with(value.as_str())
                && !library
                    .iter()
                    .any(|candidate| candidate.part.mpn == entry.part.mpn)
            {
                store.save_part(entry.clone())?;
            }
            parts.push(entry.clone());
            continue;
        }
        // Not in the library: guess from the footprint name and save
        // the guess so the operator can review it.
        let mut guessed = part_from_footprint(&original)
            .unwrap_or_else(|| unreviewable_part(&original, PartId(next_id)));
        guessed.part.id = PartId(next_id);
        next_id += 1;
        // The MPN identifies the library entry as value-footprint
        // ("10uF-C_0402_1005Metric"), so two values in the same package
        // stay separate parts; footprint alone when the row has no
        // value — or when the value is only the footprint fallback
        // `rows_from_job`/`editor_rows_from_job` synthesise for rows
        // whose part carries no MPN — otherwise the "value" would just
        // duplicate the footprint ("0402-0402").
        guessed.part.mpn = if value.is_empty() || value.eq_ignore_ascii_case(&original) {
            original.clone()
        } else {
            format!("{value}-{original}")
        };
        guessed.part.description = original.clone();
        if guessed.status == PartStatus::NeedsReview {
            unconfirmed.push(format!("{} (needs review)", guessed.part.mpn));
        } else if guessed.status == PartStatus::Guessed {
            unconfirmed.push(format!("{} (guessed, please confirm)", guessed.part.mpn));
        }
        store.save_part(guessed.clone())?;
        parts.push(guessed);
    }

    // The nozzle is a property of the physical package, so the footprint
    // decides it — not the MPN, value or reference a row happened to
    // carry. Every part whose footprint the machine can interpret gets
    // its allowlist re-derived here, which also heals stale entries that
    // were guessed before this rule existed (blank or wrong allowlists).
    // Footprints the machine cannot interpret keep whatever the part
    // already has.
    for part in &mut parts {
        if let Some(guess) = part_from_footprint(&part.part.footprint.name) {
            if part.part.nozzle_type != guess.part.nozzle_type {
                part.part.nozzle_type = guess.part.nozzle_type;
                // Persist, so the parts panel and future plans see the
                // footprint-derived allowlist.
                store.save_part(part.clone())?;
            }
        }
    }

    // 2. Turn the rows into placement needs.
    let index_of: HashMap<(String, String), usize> = keys
        .iter()
        .enumerate()
        .map(|(index, key)| (key.clone(), index))
        .collect();

    let mut needs: Vec<PlacementNeed> = Vec::new();
    let mut skipped = 0usize;
    for row in &rows {
        if !row.x.is_finite() || !row.y.is_finite() {
            // Nothing can be done with a row that has no coordinates —
            // not placing it, and not locating it either. It is reported
            // so the operator can fix the cell instead of wondering why
            // the board came out short.
            skipped += 1;
            continue;
        }
        if row.fiducial {
            continue;
        }
        let key = (
            row.footprint.trim().to_uppercase(),
            row.value.trim().to_string(),
        );
        let Some(&part_index) = index_of.get(&key) else {
            skipped += 1;
            continue;
        };
        let part = &parts[part_index].part;
        needs.push(PlacementNeed {
            part: part.clone(),
            reference: row.reference.clone(),
            design_position: Point::new(row.x, row.y),
            design_rotation_deg: if row.rotation.is_finite() { row.rotation } else { 0.0 },
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: camera_for(part),
        });
    }

    // Rows the operator marked as fiducials become board landmarks. The
    // runner needs at least two to recover a pose. Re-planning the same
    // board must not throw away calibrated positions, so expected and
    // located machine positions are carried over from the previous job
    // wherever the design position matches.
    let previous_fiducials = job.fiducials.clone();
    let fiducials: Vec<Fiducial> = rows
        .iter()
        .filter(|row| row.fiducial && row.x.is_finite() && row.y.is_finite())
        .map(|row| {
            let design_position = Point::new(row.x, row.y);
            let previous = previous_fiducials.iter().find(|fiducial| {
                fiducial.kind == FiducialKind::Panel
                    && (fiducial.design_position.x - design_position.x).abs() < 0.01
                    && (fiducial.design_position.y - design_position.y).abs() < 0.01
            });
            Fiducial {
                design_position,
                kind: FiducialKind::Panel,
                machine_position: previous.and_then(|fiducial| fiducial.machine_position),
                expected_position: previous.and_then(|fiducial| fiducial.expected_position),
            }
        })
        .collect();

    // 3. Machine layout: keep whatever a previous plan produced, and fall
    //    back to defaults for a first plan.
    let head = if job.head.nozzles.is_empty() {
        default_head(4)
    } else {
        NozzleHead::new(
            job.head
                .nozzles
                .iter()
                .map(|nozzle| NozzleSlot {
                    id: nozzle.id,
                    offset_mm: nozzle.offset_mm,
                    kind: nozzle.kind,
                    rotation_deg: 0.0,
                    vacuum: false,
                })
                .collect(),
        )
    };
    let feeders: Vec<FeederSlot> = if job.feeders.is_empty() {
        default_feeders()
    } else {
        job.feeders.clone()
    };
    let banks: Vec<FeederBank> = if job.banks.is_empty() {
        default_banks(&feeders)
    } else {
        job.banks.clone()
    };
    let setup = if feeders
        .iter()
        .any(|feeder| machine.setup.kind(feeder.id).is_some())
    {
        machine.setup.clone()
    } else {
        default_setup(&feeders)
    };
    let config: MachineConfig = machine.config.clone();

    // The planner optimises around the middle of the board, so use the
    // bounding-box centre of everything that has to be placed.
    let work_centroid = centroid(&needs);

    let planned = plan_job_with_mode(
        board_id.clone(),
        needs,
        &head,
        &feeders,
        &setup,
        work_centroid,
        &config,
        FeederPlacementMode::Movable,
        PlanSearchMode::LocalSearch,
    );

    // 4. Store the plan.
    let unassigned_parts = planned
        .feeder_assignment
        .unassigned
        .iter()
        .filter_map(|part_id| {
            parts
                .iter()
                .find(|entry| entry.part.id == *part_id)
                .map(|entry| entry.part.footprint.name.clone())
        })
        .collect();

    job.board_id = board_id;
    job.feeders = feeders.clone();
    job.banks = banks;
    job.head = HeadSpec {
        nozzles: head
            .nozzles
            .iter()
            .map(|nozzle| NozzleSpec {
                id: nozzle.id,
                kind: nozzle.kind,
                offset_mm: nozzle.offset_mm,
            })
            .collect(),
    };
    // Machine settings (cameras, config, feeder setup) stay in the
    // machine context — they are deliberately not written into the job.
    job.placements = planned.job.placements.clone();
    job.feeder_assignment = planned.feeder_assignment;
    job.fiducials = fiducials;
    // Persist the editor's column layout so loading the job restores
    // the exact grid the operator imported instead of a default guess.
    job.placement_columns = columns;
    // Persist the planner's cycle-time estimate so the process view can
    // show an estimated CPH without re-running the planner.
    job.estimated_seconds = Some(planned.estimated_seconds);
    store.save_job(job)?;

    let part_count = planned
        .job
        .placements
        .iter()
        .map(|placement| placement.part.id)
        .collect::<std::collections::HashSet<_>>()
        .len();

    Ok(PlanSummary {
        placement_count: planned.job.placements.len(),
        part_count,
        unassigned_parts,
        skipped,
        unconfirmed_parts: unconfirmed,
        // Components per hour from the planner's own cycle-time model.
        estimated_cph: if planned.estimated_seconds > 0.0 {
            planned.job.placements.len() as f64 / planned.estimated_seconds * 3600.0
        } else {
            0.0
        },
    })
}

/// Rebuild the planner's input rows from the stored job.
///
/// Re-optimisation happens away from the editor (opening the process
/// view, going to run), where the text grid is not available. The rows
/// are therefore reconstructed from the placements and fiducials of the
/// last plan: parts resolve again by footprint name, the machine layout
/// carries over, and the planner produces a fresh run order for the same
/// board.
pub fn rows_from_job(job: &JobFile) -> Vec<PlacementRow> {
    let mut rows: Vec<PlacementRow> = job
        .placements
        .iter()
        .map(|placement| PlacementRow {
            // The reference column is the placement's identity; the
            // planner never reads it, but the editor relies on it.
            reference: placement.reference.clone(),
            // Fall back to the footprint name when the part carries no
            // MPN, mirroring `editor_rows_from_job`. A blank value would
            // take the planner's valueless branch, which re-adopts
            // legacy blank-MPN library entries without healing them —
            // re-planning a loaded job would keep the parts panel blank
            // forever.
            value: if placement.part.mpn.is_empty() {
                placement.part.footprint.name.clone()
            } else {
                placement.part.mpn.clone()
            },
            footprint: placement.part.footprint.name.clone(),
            x: placement.design_position.x,
            y: placement.design_position.y,
            rotation: placement.design_rotation_deg,
            fiducial: false,
        })
        .collect();
    rows.extend(job.fiducials.iter().map(|fiducial| PlacementRow {
        reference: String::new(),
        value: String::new(),
        footprint: String::new(),
        x: fiducial.design_position.x,
        y: fiducial.design_position.y,
        rotation: 0.0,
        fiducial: true,
    }));
    rows
}

/// Fast vision is the default for every part; the operator can pin
/// precise vision (or the head camera) per part in the part editor,
/// which always wins over the default.
fn camera_for(part: &Part) -> CameraKind {
    if let Some(chosen) = part.camera {
        return chosen;
    }
    CameraKind::FastVision
}

/// A part for a footprint name the machine cannot interpret.
///
/// Deliberately unusable: no nozzle allowlist, so the job refuses to run
/// until the operator fills the parameters in.
fn unreviewable_part(name: &str, id: PartId) -> pnp_logic::SetupPart {
    pnp_logic::SetupPart {
        part: Part {
            id,
            mpn: String::new(),
            description: name.to_string(),
            footprint: pnp_logic::Footprint {
                name: name.to_string(),
                centroid: Point::ORIGIN,
                size: Size {
                    width: 0.0,
                    height: 0.0,
                },
                pads: Vec::new(),
                polarity: pnp_logic::Polarity::None,
            },
            nozzle_type: Vec::new(),
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        },
        status: PartStatus::NeedsReview,
    }
}

/// Middle of the board, so the planner's travel estimate is centred.
fn centroid(needs: &[PlacementNeed]) -> Point {
    if needs.is_empty() {
        return Point::ORIGIN;
    }
    let count = needs.len() as f64;
    let sum = needs
        .iter()
        .fold(Point::ORIGIN, |sum, need| Point::new(
            sum.x + need.design_position.x,
            sum.y + need.design_position.y,
        ));
    Point::new(sum.x / count, sum.y / count)
}

/// Two banks of four slots, straight along the machine's X axis.
pub fn default_feeders() -> Vec<FeederSlot> {
    let mut feeders = Vec::new();
    for bank in 0..2u8 {
        for index in 0..4u8 {
            let id = bank * 4 + index;
            feeders.push(FeederSlot {
                id,
                bank_id: bank,
                position: Point::new(-75.0 + index as f64 * 50.0, 400.0 + bank as f64 * 30.0),
                loaded: None,
            });
        }
    }
    feeders
}

/// Both banks are mounted square to the machine frame, so a pickup
/// offset in the bank frame is also the machine-frame offset.
pub fn default_banks(feeders: &[FeederSlot]) -> Vec<FeederBank> {
    let mut banks: Vec<u8> = feeders.iter().map(|feeder| feeder.bank_id).collect();
    banks.sort();
    banks.dedup();
    banks
        .into_iter()
        .map(|id| FeederBank {
            id,
            rotation_deg: 0.0,
        })
        .collect()
}

/// Wide tape everywhere: the planner grows a slot's width to fit when it
/// needs to, and a 16 mm pocket accepts every hobby-board part.
pub fn default_setup(feeders: &[FeederSlot]) -> MachineSetup {
    let mut setup = MachineSetup::new();
    for feeder in feeders {
        setup.set_kind(feeder.id, FeederKind::Tape(16));
    }
    setup
}

/// One fast-vision camera and one precise camera.
pub fn default_cameras() -> Vec<Camera> {
    vec![
        Camera {
            id: 0,
            kind: CameraKind::FastVision,
            position: Point::new(0.0, 200.0),
            array_pitch_mm: Some(20.0),
        },
        Camera {
            id: 1,
            kind: CameraKind::Precise,
            position: Point::new(0.0, 0.0),
            array_pitch_mm: None,
        },
    ]
}

/// Turn the stored job back into editable placement rows, so the PCB
/// placement list can be populated from a loaded job (or from the
/// working job at startup) and edited again.
///
/// Designators are re-synthesised (`P1`, `P2`, … and `FID-n`): the
/// stored placements do not carry the reference labels the original
/// CSV/import had. Every component row carries the footprint name —
/// that is what the planner resolves parts by — with the MPN as the
/// value. Fiducial rows repeat the label in the value column so the
/// editor's row validation (which wants a non-empty Value/Footprint)
/// accepts them.
pub fn editor_rows_from_job(job: &JobFile) -> Vec<PlacementRow> {
    let mut rows = Vec::with_capacity(job.placements.len() + job.fiducials.len());
    for (index, placement) in job.placements.iter().enumerate() {
        // The planner resolves parts by footprint, so fall back to the
        // footprint name when the part carries no MPN — the editor's
        // row validation wants a non-empty Value.
        let value = if placement.part.mpn.is_empty() {
            placement.part.footprint.name.clone()
        } else {
            placement.part.mpn.clone()
        };
        // Designators are the placement's identity: keep the one stored
        // with the placement, and only synthesise a label when an older
        // job file has none.
        let reference = if placement.reference.is_empty() {
            format!("P{}", index + 1)
        } else {
            placement.reference.clone()
        };
        rows.push(PlacementRow {
            reference,
            value,
            footprint: placement.part.footprint.name.clone(),
            x: placement.design_position.x,
            y: placement.design_position.y,
            rotation: placement.design_rotation_deg,
            fiducial: false,
        });
    }
    for (index, fiducial) in job.fiducials.iter().enumerate() {
        let label = format!("FID-{}", index + 1);
        rows.push(PlacementRow {
            reference: label.clone(),
            value: label,
            footprint: String::new(),
            x: fiducial.design_position.x,
            y: fiducial.design_position.y,
            rotation: 0.0,
            fiducial: true,
        });
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;
    use pnp_logic::Placement;

    /// Isolated working directory for tests that touch the store.
    struct ScratchDir {
        previous: std::path::PathBuf,
    }
    impl ScratchDir {
        fn enter(tag: &str) -> Self {
            let scratch = std::env::temp_dir()
                .join(format!("job_plan_{tag}_{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&scratch);
            std::fs::create_dir_all(&scratch).unwrap();
            let previous = std::env::current_dir().unwrap();
            std::env::set_current_dir(&scratch).unwrap();
            Self { previous }
        }
    }
    impl Drop for ScratchDir {
        fn drop(&mut self) {
            let _ = std::env::set_current_dir(&self.previous);
        }
    }

    /// Tests change the process-wide working directory; hold the shared
    /// crate-wide lock (defined on the job store) while a scratch-dir
    /// test runs so parallel tests can't interleave.
    use crate::job_store::test_cwd_lock;

    fn machine() -> pnp_logic::MachineContext {
        pnp_logic::MachineContext::default()
    }

    fn row(reference: &str, value: &str, footprint: &str, x: f64, y: f64) -> PlacementRow {
        PlacementRow {
            reference: reference.to_string(),
            value: value.to_string(),
            footprint: footprint.to_string(),
            x,
            y,
            rotation: 0.0,
            fiducial: false,
        }
    }

    #[test]
    fn plan_resolves_one_part_per_value_and_footprint_pair() {
        // Two values sharing a package ("10uF" and "100nF", both 0402)
        // must become two parts with distinct feeders, not one merged
        // part that places the wrong component.
        let _guard = test_cwd_lock();
        let _scratch = ScratchDir::enter("value_footprint_parts");
        let store = Store::load().unwrap();
        let summary = plan_from_rows(
            &store,
            &machine(),
            "board".to_string(),
            Vec::new(),
            vec![
                row("C1", "10uF", "0402", 10.0, 10.0),
                row("C2", "10uF", "0402", 20.0, 10.0),
                row("C3", "100nF", "0402", 30.0, 10.0),
            ],
        )
        .unwrap();

        // Two distinct parts, nine-... three placements, none skipped.
        assert_eq!(summary.placement_count, 3);
        assert_eq!(summary.part_count, 2);
        assert_eq!(summary.skipped, 0);

        // The guessed parts carry MPNs that identify value and package.
        let library = store.parts();
        let mpns: Vec<&str> = library
            .iter()
            .map(|entry| entry.part.mpn.as_str())
            .collect();
        assert!(mpns.contains(&"10uF-0402"), "mpns: {mpns:?}");
        assert!(mpns.contains(&"100nF-0402"), "mpns: {mpns:?}");

        // Re-planning the same board resolves to the same parts (no
        // duplicates grow in the library).
        let summary2 = plan_from_rows(
            &store,
            &machine(),
            "board".to_string(),
            Vec::new(),
            vec![
                row("C1", "10uF", "0402", 10.0, 10.0),
                row("C2", "10uF", "0402", 20.0, 10.0),
                row("C3", "100nF", "0402", 30.0, 10.0),
            ],
        )
        .unwrap();
        assert_eq!(summary2.part_count, 2);
        assert_eq!(store.parts().len(), 2, "library must not grow on re-plan");
    }

    #[test]
    fn plan_renames_stale_blank_mpn_entries_to_value_footprint() {
        // Parts guessed by older builds carry an empty MPN. Planning a
        // board with values must adopt AND rename them, otherwise the
        // parts panel shows blank MPN rows forever.
        let _guard = test_cwd_lock();
        let _scratch = ScratchDir::enter("legacy_blank_mpn");
        let store = Store::load().unwrap();
        // Simulate the stale entry: right footprint, no MPN, no nozzles.
        let mut stale = pnp_logic::SetupPart {
            part: Part {
                id: PartId(9),
                mpn: String::new(),
                description: "0402".to_string(),
                footprint: pnp_logic::Footprint {
                    name: "0402".to_string(),
                    centroid: Point::ORIGIN,
                    size: Size {
                        width: 1.0,
                        height: 0.5,
                    },
                    pads: Vec::new(),
                    polarity: pnp_logic::Polarity::None,
                },
                nozzle_type: Vec::new(),
                pickup_rotation_deg: 0.0,
                pickup_offset: Point::ORIGIN,
                camera: None,
            },
            status: PartStatus::NeedsReview,
        };
        store.save_part(stale.clone());

        let summary = plan_from_rows(
            &store,
            &machine(),
            "board".to_string(),
            Vec::new(),
            vec![
                row("C1", "10uF", "0402", 10.0, 10.0),
                row("C4", "100nF", "0402", 30.0, 10.0),
            ],
        )
        .unwrap();
        assert_eq!(summary.placement_count, 2);
        // The legacy part serves one of the values; the other becomes a
        // new part. No blank MPNs may remain.
        let mpns: Vec<String> = store.parts().iter().map(|e| e.part.mpn.clone()).collect();
        assert_eq!(mpns.len(), 2, "mpns: {mpns:?}");
        assert!(
            mpns.iter().all(|mpn| !mpn.is_empty()),
            "blank MPN survived: {mpns:?}"
        );
        assert!(mpns.contains(&"10uF-0402".to_string()), "mpns: {mpns:?}");
        assert!(mpns.contains(&"100nF-0402".to_string()), "mpns: {mpns:?}");
        // Keep the stale handle from tripping unused warnings.
        stale.part.mpn = String::new();
    }

    #[test]
    fn plan_derives_nozzles_from_the_footprint_not_the_row() {
        // Nozzles are a property of the physical package: whatever MPN,
        // value or reference a row carries, the allowlist must come from
        // the footprint. A stale entry with an empty (or wrong) allowlist
        // is re-derived at plan time.
        let _guard = test_cwd_lock();
        let _scratch = ScratchDir::enter("footprint_nozzles");
        let store = Store::load().unwrap();
        store.save_part(pnp_logic::SetupPart {
            part: Part {
                id: PartId(7),
                mpn: "10uF-0402".to_string(),
                description: "0402".to_string(),
                footprint: pnp_logic::Footprint {
                    name: "0402".to_string(),
                    centroid: Point::ORIGIN,
                    size: Size {
                        width: 1.0,
                        height: 0.5,
                    },
                    pads: Vec::new(),
                    polarity: pnp_logic::Polarity::None,
                },
                // Wrong on purpose: 0402 needs the 0.4 mm tip (N502).
                nozzle_type: Vec::new(),
                pickup_rotation_deg: 0.0,
                pickup_offset: Point::ORIGIN,
                camera: None,
            },
            status: PartStatus::Confirmed,
        });

        plan_from_rows(
            &store,
            &machine(),
            "board".to_string(),
            Vec::new(),
            vec![row("C1", "10uF", "0402", 10.0, 10.0)],
        )
        .unwrap();

        let saved = store
            .parts()
            .into_iter()
            .find(|entry| entry.part.id == PartId(7))
            .expect("part kept");
        assert_eq!(saved.part.nozzle_type, vec![pnp_logic::NozzleKind::N502]);
    }

    fn test_job() -> JobFile {
        let mut placements = Vec::new();
        for (index, (x, y)) in [(10.0, 20.0), (30.0, 40.0)].iter().enumerate() {
            placements.push(Placement {
                reference: String::new(),
                part: part_from_footprint("0603").unwrap().part,
                design_position: Point::new(*x, *y),
                design_rotation_deg: if index == 0 { 90.0 } else { 0.0 },
                feeder_slot: 1,
                nozzle: 0,
                placed: false,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            });
        }
        JobFile {
            board_id: "test".to_string(),
            track_width_mm: None,
            feeders: vec![],
            banks: vec![],
            head: HeadSpec { nozzles: vec![] },
            placements,
            feeder_assignment: pnp_logic::FeederAssignment::default(),
            fiducials: vec![Fiducial {
                design_position: Point::new(1.0, 2.0),
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
    fn rows_from_job_rebuilds_planner_input() {
        let rows = rows_from_job(&test_job());
        assert_eq!(rows.len(), 3);

        // Placements first: footprint, coordinates and rotation are what
        // the planner reads.
        assert_eq!(rows[0].footprint, "0603");
        assert!((rows[0].x - 10.0).abs() < 1e-9);
        assert!((rows[0].y - 20.0).abs() < 1e-9);
        assert!((rows[0].rotation - 90.0).abs() < 1e-9);
        assert!(!rows[0].fiducial);

        // A fiducial row rounds the list off, out of grid terms.
        assert!(rows[2].fiducial);
        assert!(rows[2].footprint.is_empty());
        assert!((rows[2].x - 1.0).abs() < 1e-9);
        assert!((rows[2].y - 2.0).abs() < 1e-9);
    }

    #[test]
    fn replanning_a_loaded_job_heals_blank_mpn_parts() {
        // A job saved by an older build carries placements whose parts
        // have an empty MPN. Re-optimising it (`rows_from_job` without
        // the editor grid) must fall back to the footprint name as the
        // value, so the planner's legacy-entry adoption renames the
        // blank parts — otherwise the parts panel shows blank MPNs
        // forever after loading such a job.
        let _guard = test_cwd_lock();
        let _scratch = ScratchDir::enter("loaded_job_blank_mpn");
        let store = Store::load().unwrap();

        let mut job = test_job();
        for placement in &mut job.placements {
            placement.part.mpn = String::new();
        }
        let rows = rows_from_job(&job);
        // Fiducial rows legitimately have no value; component rows must
        // not.
        assert!(
            rows.iter()
                .filter(|row| !row.fiducial)
                .all(|row| !row.value.is_empty()),
            "rows must not carry a blank value: {rows:?}"
        );

        plan_from_rows(&store, &machine(), job.board_id.clone(), Vec::new(), rows)
            .unwrap();

        let mpns: Vec<String> = store.parts().iter().map(|e| e.part.mpn.clone()).collect();
        assert!(
            mpns.iter().all(|mpn| !mpn.is_empty()),
            "blank MPN survived re-planning a loaded job: {mpns:?}"
        );
        // The value was only the footprint fallback, so the MPN is the
        // footprint alone — not a doubled "0603-0603".
        assert_eq!(mpns, vec!["0603".to_string()], "mpns: {mpns:?}");
    }

    #[test]
    fn editor_rows_from_job_rebuilds_the_grid() {
        let rows = editor_rows_from_job(&test_job());
        assert_eq!(rows.len(), 3);

        // Component rows carry synthesised unique designators, the MPN
        // as the value and the footprint the planner resolves by.
        assert_eq!(rows[0].reference, "P1");
        assert_eq!(rows[1].reference, "P2");
        assert!(!rows[0].value.is_empty());
        assert_eq!(rows[0].footprint, "0603");
        assert!((rows[1].rotation - 0.0).abs() < 1e-9);
        assert!(!rows[1].fiducial);

        // Fiducial rows keep their label in both reference and value so
        // the editor's validation accepts them.
        assert_eq!(rows[2].reference, "FID-1");
        assert_eq!(rows[2].value, "FID-1");
        assert!(rows[2].fiducial);
        assert!((rows[2].x - 1.0).abs() < 1e-9);
    }
}
