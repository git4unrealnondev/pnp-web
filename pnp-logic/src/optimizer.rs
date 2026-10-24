//! Optimisers: where to load each reel on the feeder rail.
//!
//! The nozzle-assignment optimiser lives in `crate::nozzle` because it is
//! tightly coupled to nozzle geometry.

use std::cmp::Ordering;
use std::collections::{BTreeMap, HashMap, HashSet};

use shared_types::*;

use crate::config::MachineSetup;
use crate::feeder::{FeederSlot, PartId, ProviderClass};
use crate::parts::Part;
use crate::state::PlacementStrategy;

/// One placement requirement in a job: this part goes at this design
/// position. Frequency is encoded by repeating the entry — three 100Ω
/// resistors on a board are three `PlacementNeed`s with the same `Part`.
#[derive(Debug, Clone)]
pub struct PlacementNeed {
    pub part: Part,
    pub design_position: Point,
    pub design_rotation_deg: f64,
    /// Components marked false are excluded from feeder allocation.
    pub should_place: bool,
    pub strategy: PlacementStrategy,
    /// Which camera on the machine inspects this placement. Defaults
    /// to 0 in tests; production code should set per part (BGA → 1).
    pub camera_kind: crate::config::CameraKind,
    /// Original designator (`R12`, `C5`, ...) carried through to the
    /// placement so the job keeps the operator's identity for each
    /// component. Empty when the source list has no designators.
    pub reference: String,
}

/// Result of feeder assignment.
///
/// `slot_for_part[pid]` lists every physical slot that holds a reel of
/// `pid`. The list is ordered: the runner picks slots from front to back
/// and only advances to the next slot in the list when the current reel
/// is exhausted. If a part needs more capacity than one reel provides,
/// it gets multiple slots in this list.
///
/// `chosen_kind[slot_id]` is the FeederKind the planner picked for
/// that slot (the smallest tape width that fits the part assigned
/// there, when the operator didn't pin a specific kind). The runner
/// uses this to drive reel-advance behaviour.
#[derive(Debug, Clone, Default, serde::Serialize, serde::Deserialize)]
pub struct FeederAssignment {
    pub slot_for_part: HashMap<PartId, Vec<u8>>,
    pub unassigned: Vec<PartId>,
    pub chosen_kind: HashMap<u8, crate::feeder::FeederKind>,
}

/// Whether the optimizer may change the feeder contents for this job.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum FeederPlacementMode {
    /// Keep the currently loaded reels in their slots. Empty or incompatible
    /// slots cannot be used for this job.
    Fixed,
    /// Feeder contents may be placed in compatible slots. Existing matching
    /// reels are still preferred to avoid unnecessary operator work.
    Movable,
}

/// A slot/category input for the native-style discrete optimizer.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OptimizerSlot {
    pub id: u8,
    pub position: Point,
    pub feeder_section: u16,
    pub provider: u8,
    /// Bitmask of component categories accepted by this slot.
    pub category_mask: u64,
    /// Bitmask of feeder compatibility classes accepted by this slot.
    pub feeder_mask: u64,
    pub available: bool,
}

/// Remaining demand for a component category and feeder section.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ComponentDemand {
    pub category: u8,
    pub count: u32,
    pub feeder_section: u16,
    pub provider: u8,
    pub category_mask: u64,
    pub feeder_compatibility: u64,
}

/// One selected feeder installation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeederInstallation {
    pub slot_id: u8,
    pub category: u8,
    pub provider: ProviderClass,
    pub feeder_section: u16,
    pub component_count: u32,
    pub cost: f64,
}

/// Full result of the native-style assignment pass.
#[derive(Debug, Clone, PartialEq)]
pub struct FeederOptimization {
    pub installations: Vec<FeederInstallation>,
    pub slot_component_map: BTreeMap<u8, FeederInstallation>,
    pub unassigned: Vec<ComponentDemand>,
}

/// Gantry movement time proxy: both axes move simultaneously, so the slower
/// axis determines the move cost.
pub fn chebyshev_cost(from: Point, to: Point) -> f64 {
    (from.x - to.x).abs().max((from.y - to.y).abs())
}

fn compatible(slot: &OptimizerSlot, demand: &ComponentDemand) -> bool {
    slot.available
        && slot.feeder_section == demand.feeder_section
        && ProviderClass::from_native(slot.provider) == ProviderClass::from_native(demand.provider)
        && slot.category_mask & demand.category_mask != 0
        && slot.feeder_mask & demand.feeder_compatibility != 0
}

/// Assign component demand to slots using the native incremental nearest-
/// candidate strategy. Provider class and feeder section are separate
/// candidate groups, and each physical slot is assigned once.
pub fn optimise_feeder_model(
    slots: &[OptimizerSlot],
    demands: &[ComponentDemand],
    reference: Point,
) -> FeederOptimization {
    let mut remaining = demands.to_vec();
    let mut used = vec![false; slots.len()];
    let mut installations = Vec::new();
    let mut current = reference;

    let mut groups: Vec<(ProviderClass, u16)> = demands
        .iter()
        .map(|d| (ProviderClass::from_native(d.provider), d.feeder_section))
        .collect();
    groups.sort_unstable();
    groups.dedup();

    for (provider, section) in groups {
        loop {
            let mut best: Option<(usize, usize, f64)> = None;
            for (slot_index, slot) in slots.iter().enumerate() {
                if used[slot_index]
                    || ProviderClass::from_native(slot.provider) != provider
                    || slot.feeder_section != section
                {
                    continue;
                }
                for (demand_index, demand) in remaining.iter().enumerate() {
                    if demand.count == 0 || !compatible(slot, demand) {
                        continue;
                    }
                    let cost = chebyshev_cost(current, slot.position);
                    let replace = best.is_none_or(|(old_slot, old_demand, old_cost)| {
                        cost < old_cost
                            || (cost == old_cost
                                && (slot.id, demand.category)
                                    < (slots[old_slot].id, remaining[old_demand].category))
                    });
                    if replace {
                        best = Some((slot_index, demand_index, cost));
                    }
                }
            }

            let Some((slot_index, demand_index, cost)) = best else {
                break;
            };
            let slot = slots[slot_index];
            let demand = &mut remaining[demand_index];
            demand.count -= 1;
            used[slot_index] = true;
            installations.push(FeederInstallation {
                slot_id: slot.id,
                category: demand.category,
                provider,
                feeder_section: demand.feeder_section,
                component_count: 1,
                cost,
            });
            current = slot.position;
        }
    }

    FeederOptimization {
        slot_component_map: installations
            .iter()
            .copied()
            .map(|installation| (installation.slot_id, installation))
            .collect(),
        installations,
        unassigned: remaining.into_iter().filter(|d| d.count != 0).collect(),
    }
}

/// Greedy feeder assignment.
///
/// 1. Count placements per part (frequency).
/// 2. Split parts into sub-reel "chunks" sized to the feeder capacity
///    (a 5000-placement resistor on 4000-cap tape reels → 2 chunks).
/// 3. Sort chunks: most-needed chunks first; for ties, closest to the
///    work centroid wins (those benefit the most from proximity).
/// 4. Walk the sorted chunks; for each, take the closest slot that
///    already holds a matching reel (no operator reload) or is empty
///    and can accept the part by size. If no such slot exists, the part
///    goes into `unassigned`.
///
/// `setup` maps each slot's `id` to the physical `FeederKind`
/// mounted in that slot for this job.
///
/// Bare bones: chunking is the only refinement. A fuller optimiser would
/// co-locate reels of frequently-paired parts on adjacent slots, reserve
/// tray positions for large BGAs, and prefer same-rail clustering.
pub fn optimise_feeder_slots(
    needs: &[PlacementNeed],
    slots: &[FeederSlot],
    setup: &MachineSetup,
    work_centroid: Point,
) -> FeederAssignment {
    optimise_feeder_slots_with_mode(
        needs,
        slots,
        setup,
        work_centroid,
        FeederPlacementMode::Movable,
    )
}

/// Assign feeders using an explicit fixed or movable feeder policy.
pub fn optimise_feeder_slots_with_mode(
    needs: &[PlacementNeed],
    slots: &[FeederSlot],
    setup: &MachineSetup,
    work_centroid: Point,
    mode: FeederPlacementMode,
) -> FeederAssignment {
    let needs_to_place: Vec<PlacementNeed> = needs
        .iter()
        .filter(|need| need.should_place)
        .cloned()
        .collect();
    let needs = needs_to_place.as_slice();

    // 1. Frequency per part.
    let mut counts: HashMap<PartId, u32> = HashMap::new();
    for n in needs {
        *counts.entry(n.part.id).or_insert(0) += 1;
    }

    /// Width in mm for a feeder kind. `Tape(w)` is its width; non-tape
    /// feeders are sortable by an effective width (Tape=its width, Tray
    /// = ∞, Vibratory = ∞, others = their kind-specific max width).
    fn feeder_width(kind: crate::feeder::FeederKind) -> f64 {
        use crate::feeder::FeederKind;
        match kind {
            FeederKind::Tape(w) => w as f64,
            FeederKind::Stick => 30.0,
            FeederKind::Tray => 100.0,
            FeederKind::Bulk => 10.0,
            FeederKind::Tube => 8.0,
            FeederKind::Vibratory => 100.0,
        }
    }

    /// Smallest tape width that fits the part (strict: w < tape_width).
    /// Returns `Tape(8)` as the floor even when the part needs nothing
    /// wider.
    fn smallest_tape_for(part: &Part) -> crate::feeder::FeederKind {
        use crate::feeder::FeederKind;
        let w = part.footprint.size.width.max(part.footprint.size.height);
        for &tape in &[8u8, 12, 16, 24, 32, 44] {
            if w < tape as f64 {
                return FeederKind::Tape(tape);
            }
        }
        FeederKind::Tray
    }

    /// Resolved FeederKind for `s` when assigned to `part`. If the
    /// operator pinned a specific kind via `setup.set_kind`, that wins.
    /// Otherwise the smallest tape width that fits the part is chosen
    /// automatically — the planner picks the physical slot, not the
    /// operator.
    fn kind_for(s: &FeederSlot, setup: &MachineSetup, part: &Part) -> crate::feeder::FeederKind {
        setup.kind(s.id).unwrap_or_else(|| smallest_tape_for(part))
    }

    // 2. Build a flat list of (part, count) "chunks" — one per part,
    //    sized to one reel's worth. We don't pre-decide the chunk size
    //    here; allocation is iterative (see step 4) so a part that
    //    exceeds a slot's capacity automatically gets split across
    //    multiple reels at allocation time.
    #[derive(Clone)]
    struct Chunk<'a> {
        part: &'a Part,
        count: u32,
    }
    let parts_by_id: HashMap<PartId, &Part> = needs.iter().map(|n| (n.part.id, &n.part)).collect();
    let mut chunks: Vec<Chunk> = counts
        .into_iter()
        .map(|(pid, count)| Chunk {
            part: parts_by_id[&pid],
            count,
        })
        .collect();

    // 3. Sort: biggest chunks first; ties broken by closest part position
    //    to centroid.
    let positions_by_id: HashMap<PartId, Point> = {
        let mut m: HashMap<PartId, Point> = HashMap::new();
        for n in needs {
            m.entry(n.part.id).or_insert(n.design_position);
        }
        m
    };
    chunks.sort_by(|a, b| {
        b.count.cmp(&a.count).then_with(|| {
            let da = positions_by_id[&a.part.id].distance(work_centroid);
            let db = positions_by_id[&b.part.id].distance(work_centroid);
            da.partial_cmp(&db).unwrap_or(Ordering::Equal)
        })
    });

    // 4. Allocate chunks: each chunk is one part's full count. Pick a slot;
    //    if the slot's capacity isn't enough for the whole chunk, take
    //    what fits and re-queue the remainder as a new chunk for the
    //    same part. Iterate until the chunk is fully allocated.
    let mut used_on_slot: HashMap<(u8, PartId), u32> = HashMap::new();
    let mut slot_taken: HashSet<u8> = HashSet::new();
    let mut slot_for_part: HashMap<PartId, Vec<u8>> = HashMap::new();
    let mut unassigned: Vec<PartId> = Vec::new();
    let mut chosen_kind: HashMap<u8, crate::feeder::FeederKind> = HashMap::new();

    // Work queue: start with all chunks; each pass picks the best slot
    // for the front chunk and consumes `min(chunk.count, slot.capacity -
    // slot.used_for_this_part)` from it, re-queuing any remainder.
    let mut work: Vec<Chunk> = chunks;
    let mut current_position = work_centroid;

    while let Some(chunk) = work.first().cloned() {
        let pid = chunk.part.id;
        let mut candidates: Vec<&FeederSlot> = slots
            .iter()
            .filter(|s| {
                // Auto-pick the smallest tape width that fits the part
                // when the operator hasn't pinned a kind. This lets the
                // planner decide which slot gets which physical tape
                // body based on the part's footprint.
                let kind = kind_for(s, setup, chunk.part);
                if !kind.accepts(chunk.part) {
                    return false;
                }
                let loaded = s.loaded.or_else(|| setup.loaded(s.id));
                if mode == FeederPlacementMode::Fixed && loaded != Some(pid) {
                    return false;
                }
                let assigned_to_us = slot_for_part
                    .get(&pid)
                    .map(|v| v.contains(&s.id))
                    .unwrap_or(false);
                let used = used_on_slot.get(&(s.id, pid)).copied().unwrap_or(0);
                let cap = kind.capacity();
                if assigned_to_us {
                    return used < cap;
                }
                if loaded == Some(pid) {
                    return used < cap;
                }
                if loaded.is_none() && !slot_taken.contains(&s.id) {
                    return true;
                }
                false
            })
            .collect();
        candidates.sort_by(|a, b| {
            // Smallest feasible feeder first (e.g. an 8mm part picks
            // an 8mm slot over a 12mm slot). Within a width class,
            // pick the closest feeder swing. This keeps wider slots
            // free for wider parts and avoids over-sized swings.
            let wa = feeder_width(kind_for(a, setup, chunk.part));
            let wb = feeder_width(kind_for(b, setup, chunk.part));
            wa.partial_cmp(&wb)
                .unwrap_or(Ordering::Equal)
                .then_with(|| {
                    chebyshev_cost(current_position, a.position)
                        .partial_cmp(&chebyshev_cost(current_position, b.position))
                        .unwrap_or(Ordering::Equal)
                })
                .then_with(|| a.id.cmp(&b.id))
        });

        match candidates.into_iter().next() {
            Some(s) => {
                let kind = kind_for(s, setup, chunk.part);
                let cap = kind.capacity();
                let used = used_on_slot.get(&(s.id, pid)).copied().unwrap_or(0);
                let room = cap - used;
                let take = chunk.count.min(room);
                *used_on_slot.entry((s.id, pid)).or_insert(0) += take;
                slot_taken.insert(s.id);
                slot_for_part.entry(pid).or_default().push(s.id);
                chosen_kind.insert(s.id, kind);
                current_position = s.position;
                work.remove(0);
                if take < chunk.count {
                    // Re-queue the remainder.
                    work.insert(
                        0,
                        Chunk {
                            part: chunk.part,
                            count: chunk.count - take,
                        },
                    );
                }
            }
            None => {
                if !unassigned.contains(&pid) {
                    unassigned.push(pid);
                }
                work.remove(0);
            }
        }
    }

    // 5. Deduplicate and order slot lists (closest-to-centroid first, so
    //    the runner drains the most-accessible reel before reaching for
    //    a farther one).
    for slots_for in slot_for_part.values_mut() {
        slots_for.sort_by(|a, b| {
            let da = slots
                .iter()
                .find(|s| s.id == *a)
                .map(|s| chebyshev_cost(work_centroid, s.position))
                .unwrap_or(f64::INFINITY);
            let db = slots
                .iter()
                .find(|s| s.id == *b)
                .map(|s| chebyshev_cost(work_centroid, s.position))
                .unwrap_or(f64::INFINITY);
            da.partial_cmp(&db).unwrap_or(Ordering::Equal)
        });
        slots_for.dedup();
    }

    FeederAssignment {
        slot_for_part,
        unassigned,
        chosen_kind,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NozzleKind;
    use crate::feeder::FeederKind;
    use crate::geometry::Size;
    use crate::parts::{Footprint, Polarity};

    fn dummy_part(id: u32, mpn: &str) -> Part {
        Part {
            id: PartId(id),
            mpn: mpn.into(),
            description: String::new(),
            footprint: Footprint {
                name: mpn.into(),
                centroid: Point::ORIGIN,
                size: Size {
                    width: 1.0,
                    height: 1.0,
                },
                pads: vec![],
                polarity: Polarity::None,
            },
            nozzle_type: vec![NozzleKind::Standard],
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        }
    }

    #[test]
    fn most_used_part_gets_closest_slot() {
        let slots = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(50.0, 0.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        setup.set_kind(1, FeederKind::Tape(8));
        setup.set_kind(2, FeederKind::Tape(8));
        let mut needs: Vec<PlacementNeed> = (0..100)
            .map(|_| PlacementNeed {
                reference: String::new(),
                part: dummy_part(1, "R1"),
                design_position: Point::ORIGIN,
                design_rotation_deg: 0.0,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: crate::config::CameraKind::FastVision,
            })
            .collect();
        needs.push(PlacementNeed {
            reference: String::new(),
            part: dummy_part(2, "C1"),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        });
        let a = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);
        assert_eq!(a.slot_for_part[&PartId(1)], vec![0]);
        assert_eq!(a.slot_for_part[&PartId(2)], vec![1]);
        assert!(a.unassigned.is_empty());
    }

    #[test]
    fn already_loaded_slot_is_reused() {
        let slots = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: Some(PartId(1)),
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(1.0, 0.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        setup.set_kind(1, FeederKind::Tape(8));
        setup.set_kind(2, FeederKind::Tape(8));
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: dummy_part(1, "R1"),
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];
        let a = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);
        assert_eq!(a.slot_for_part[&PartId(1)], vec![0]);
    }

    #[test]
    fn skipped_need_does_not_consume_feeder_capacity() {
        let slots = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::ORIGIN,
            loaded: None,
        }];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        let needs = vec![
            PlacementNeed {
                reference: String::new(),
                part: dummy_part(1, "SKIP"),
                design_position: Point::ORIGIN,
                design_rotation_deg: 0.0,
                should_place: false,
                strategy: PlacementStrategy::Standard,
                camera_kind: crate::config::CameraKind::FastVision,
            },
            PlacementNeed {
                reference: String::new(),
                part: dummy_part(2, "PLACE"),
                design_position: Point::ORIGIN,
                design_rotation_deg: 0.0,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: crate::config::CameraKind::FastVision,
            },
        ];
        let assignment = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);

        assert!(!assignment.slot_for_part.contains_key(&PartId(1)));
        assert_eq!(assignment.slot_for_part[&PartId(2)], vec![0]);
        assert!(assignment.unassigned.is_empty());
    }

    #[test]
    fn fixed_mode_only_uses_matching_preloaded_slots() {
        let slots = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: Some(PartId(1)),
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(1.0, 0.0),
                loaded: Some(PartId(2)),
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        setup.set_kind(1, FeederKind::Tape(8));
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: dummy_part(1, "R1"),
            design_position: Point::new(100.0, 100.0),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];

        let result = optimise_feeder_slots_with_mode(
            &needs,
            &slots,
            &setup,
            Point::new(100.0, 100.0),
            FeederPlacementMode::Fixed,
        );

        assert_eq!(result.slot_for_part[&PartId(1)], vec![0]);
        assert!(result.unassigned.is_empty());
    }

    #[test]
    fn movable_mode_prefers_slot_nearest_to_board() {
        let slots = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(100.0, 100.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        setup.set_kind(1, FeederKind::Tape(8));
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: dummy_part(1, "R1"),
            design_position: Point::new(100.0, 100.0),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];

        let result = optimise_feeder_slots_with_mode(
            &needs,
            &slots,
            &setup,
            Point::new(100.0, 100.0),
            FeederPlacementMode::Movable,
        );

        assert_eq!(result.slot_for_part[&PartId(1)], vec![1]);
        assert!(result.unassigned.is_empty());
    }

    #[test]
    fn smallest_feasible_feeder_wins() {
        // 12mm part prefers 12mm tape over 16mm tape, even if 16mm is
        // closer to the work centroid. The narrow slot is reserved for
        // the narrow part so wider slots stay free for wider parts.
        let slots = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(100.0, 0.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(12));
        setup.set_kind(1, FeederKind::Tape(16));
        let mut part12 = dummy_part(1, "BIG12");
        part12.id = PartId(1);
        part12.footprint.size = Size {
            width: 11.0,
            height: 11.0,
        };
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: part12,
            design_position: Point::new(100.0, 0.0),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];
        let a = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);
        assert_eq!(a.slot_for_part[&PartId(1)], vec![0]);
    }

    #[test]
    fn two_slots_per_part_when_capacity_exhausted() {
        // Stick capacity is 50. Two sticks, 60 placements → 50 + 10.
        let slots = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 0.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(20.0, 0.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Stick);
        setup.set_kind(1, FeederKind::Stick);
        let needs: Vec<PlacementNeed> = (0..60)
            .map(|_| PlacementNeed {
                reference: String::new(),
                part: dummy_part(1, "R1"),
                design_position: Point::ORIGIN,
                design_rotation_deg: 0.0,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: crate::config::CameraKind::FastVision,
            })
            .collect();
        let a = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);
        assert_eq!(a.slot_for_part[&PartId(1)].len(), 2);
    }

    #[test]
    fn strict_width_excludes_8mm_part_from_8mm_slot() {
        // 8mm part on 8mm slot is rejected because the strict `w < 8`
        // rule reserves the slot for narrower parts. With no operator
        // pin, the planner computes the smallest-feasible feeder for the
        // 8mm part (Tape(12)) and assigns that kind to the slot. The
        // slot kind is then *not* 8mm, so the planner accepts. To prove
        // the strict rule, pin the slot to 8mm tape and verify the
        // 8mm part is rejected.
        let slots = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 0.0),
            loaded: None,
        }];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        let mut big = dummy_part(1, "BIG8");
        big.footprint.size = Size {
            width: 8.0,
            height: 8.0,
        };
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: big,
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];
        let a = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);
        assert!(a.unassigned.contains(&PartId(1)));
    }

    #[test]
    fn unassigned_when_no_compatible_slot() {
        let slots = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 0.0),
            loaded: None,
        }];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        setup.set_kind(1, FeederKind::Tape(8));
        setup.set_kind(2, FeederKind::Tape(8));
        // A 20 mm part can't fit on 8 mm tape.
        let big = Part {
            id: PartId(7),
            mpn: "BIG".into(),
            description: String::new(),
            footprint: Footprint {
                name: "BIG".into(),
                centroid: Point::ORIGIN,
                size: Size {
                    width: 20.0,
                    height: 20.0,
                },
                pads: vec![],
                polarity: Polarity::None,
            },
            nozzle_type: vec![NozzleKind::ExtraFine],
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        };
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: big,
            design_position: Point::ORIGIN,
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];
        let a = optimise_feeder_slots(&needs, &slots, &setup, Point::ORIGIN);
        assert!(a.unassigned.contains(&PartId(7)));
    }

    #[test]
    fn model_optimizer_uses_chebyshev_and_provider_mapping() {
        let slots = [
            OptimizerSlot {
                id: 1,
                position: Point::new(3.0, 4.0),
                feeder_section: 0,
                provider: 6,
                category_mask: 1,
                feeder_mask: 1,
                available: true,
            },
            OptimizerSlot {
                id: 2,
                position: Point::new(4.0, 4.0),
                feeder_section: 0,
                provider: 6,
                category_mask: 1,
                feeder_mask: 1,
                available: true,
            },
        ];
        let result = optimise_feeder_model(
            &slots,
            &[ComponentDemand {
                category: 10,
                count: 2,
                feeder_section: 0,
                provider: 6,
                category_mask: 1,
                feeder_compatibility: 1,
            }],
            Point::ORIGIN,
        );

        assert_eq!(result.installations[0].slot_id, 1);
        assert_eq!(result.installations[0].cost, 4.0);
        assert_eq!(result.installations[0].provider, ProviderClass::Tray);
        assert_eq!(result.installations[1].slot_id, 2);
        assert_eq!(result.slot_component_map.len(), 2);
        assert!(result.unassigned.is_empty());
    }

    #[test]
    fn model_optimizer_keeps_provider_classes_separate() {
        let slots = [
            OptimizerSlot {
                id: 1,
                position: Point::ORIGIN,
                feeder_section: 0,
                provider: 1,
                category_mask: 1,
                feeder_mask: 1,
                available: true,
            },
            OptimizerSlot {
                id: 2,
                position: Point::ORIGIN,
                feeder_section: 0,
                provider: 7,
                category_mask: 1,
                feeder_mask: 1,
                available: true,
            },
        ];
        let result = optimise_feeder_model(
            &slots,
            &[
                ComponentDemand {
                    category: 1,
                    count: 1,
                    feeder_section: 0,
                    provider: 1,
                    category_mask: 1,
                    feeder_compatibility: 1,
                },
                ComponentDemand {
                    category: 2,
                    count: 1,
                    feeder_section: 0,
                    provider: 7,
                    category_mask: 1,
                    feeder_compatibility: 1,
                },
            ],
            Point::ORIGIN,
        );

        assert_eq!(
            result.slot_component_map[&1].provider,
            ProviderClass::Regular
        );
        assert_eq!(
            result.slot_component_map[&2].provider,
            ProviderClass::Vibratory
        );
    }
}
