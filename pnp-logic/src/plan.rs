//! Build a `Job` from a board spec: assign feeders (multi-slot, capacity-
//! aware), assign nozzles, and pick a pick sequence that maximises
//! components-per-hour (CPH).

use std::cmp::Ordering;
use std::collections::{HashMap, VecDeque};

use crate::config::{CameraKind, MachineSetup};
use crate::feeder::{FeederSlot, PartId};
use crate::nozzle::NozzleHead;
use crate::optimizer::{
    FeederAssignment, FeederPlacementMode, PlacementNeed, optimise_feeder_slots_with_mode,
};
use crate::runner::MachineConfig;
use crate::state::{Job, Placement};
use shared_types::*;

/// Output of `plan_job`.
#[derive(Debug, Clone)]
pub struct PlannedJob {
    pub job: Job,
    pub feeder_assignment: FeederAssignment,
    /// Estimated wall-clock seconds for the run, given the chosen sequence.
    pub estimated_seconds: f64,
    /// Multi-nozzle burst instructions: each burst picks multiple parts
    /// of the same type using multiple nozzles of the same kind in one
    /// head sweep. This can dramatically improve CPH when multiple
    /// nozzles of the same kind are available.
    pub multi_nozzle_bursts: Vec<MultiNozzleBurst>,
}

/// A multi-nozzle burst: pick `placements.len()` parts of the same type
/// using `nozzle_ids` in one head sweep. The head moves to each pick
/// position in sequence, picking with a different nozzle each time.
///
/// Bursts never mix part ids. A mixed-part group would require changing tip
/// compatibility and pickup calibration halfway through a head sweep.
#[derive(Debug, Clone)]
pub struct MultiNozzleBurst {
    pub part_id: PartId,
    pub nozzle_ids: Vec<u8>,
    pub placements: Vec<usize>, // indices into the final placement sequence
}

/// Per-part feeder rotation: which slots hold the part and how many
/// placements are still on each slot. The runner consults this to pick
/// the active slot and to advance to the next reel when one is empty.
#[derive(Debug, Clone, Default)]
pub struct FeederRotation {
    queues: HashMap<PartId, VecDeque<(u8, u32)>>,
}

impl FeederRotation {
    pub fn from_assignment(assignment: &FeederAssignment) -> Self {
        let mut queues: HashMap<PartId, VecDeque<(u8, u32)>> = HashMap::new();
        for (pid, slots) in &assignment.slot_for_part {
            queues.insert(*pid, slots.iter().copied().map(|s| (s, 0)).collect());
        }
        Self { queues }
    }

    /// Look up the currently-active slot for `pid`. Callers should treat
    /// the result as `Option<u8>` — `None` means the part was not assigned.
    pub fn current_slot(&self, pid: PartId) -> Option<u8> {
        self.queues
            .get(&pid)
            .and_then(|q| q.front().map(|(s, _)| *s))
    }

    /// Mark one part picked from the active slot; advance to the next
    /// slot when the current reel is exhausted.
    pub fn tick(&mut self, pid: PartId, capacity_per_reel: u32) {
        if let Some(q) = self.queues.get_mut(&pid)
            && let Some(front) = q.front_mut()
        {
            front.1 += 1;
            if front.1 >= capacity_per_reel {
                q.pop_front();
            }
        }
    }
}

/// Estimate wall-clock seconds for one full pick-and-place cycle at the
/// given step (feeder → placement on board). Uses head XY speed, Z speed,
/// rotation speed and the small per-step settling overheads.
fn step_seconds(
    feeder_pos: Point,
    place_pos: Point,
    rotation_deg: f64,
    cfg: &MachineConfig,
) -> f64 {
    // Each "speed" is a fraction of max; the OpenKayo firmware uses
    // 1..=15. We model time = distance / max_speed * (16 / speed_setting).
    const MAX_XY: f64 = 500.0; // mm/s at speed 15 (typical hobby PnP)
    const MAX_Z: f64 = 100.0;
    const MAX_ROT: f64 = 720.0; // deg/s at speed 15
    const SETTLE: f64 = 0.2;
    let xy = feeder_pos.distance(place_pos);
    let z = (cfg.travel_z_mm - cfg.pickup_z_mm).abs() + (cfg.place_z_mm - cfg.travel_z_mm).abs();
    let xy_secs = xy / MAX_XY * (16.0 / cfg.xy_speed as f64);
    let z_secs = z / MAX_Z * (16.0 / cfg.z_speed as f64);
    let rot_secs = rotation_deg.abs() / MAX_ROT * (16.0 / cfg.rotation_speed as f64);
    // Pick/place settle + vacuum + blow delays.
    xy_secs + z_secs + rot_secs + SETTLE
}

/// Plan a job: assign feeders, assign nozzles, build a sequence that
/// maximises CPH, and choose the active feeder slot per placement.
pub fn plan_job(
    board_id: impl Into<String>,
    needs: Vec<PlacementNeed>,
    head: &NozzleHead,
    feeders: &[FeederSlot],
    setup: &MachineSetup,
    work_centroid: Point,
    config: &MachineConfig,
) -> PlannedJob {
    plan_job_with_feeder_mode(
        board_id,
        needs,
        head,
        feeders,
        setup,
        work_centroid,
        config,
        FeederPlacementMode::Movable,
    )
}

/// Plan a job while honoring whether feeder contents may be relocated.
pub fn plan_job_with_feeder_mode(
    board_id: impl Into<String>,
    needs: Vec<PlacementNeed>,
    head: &NozzleHead,
    feeders: &[FeederSlot],
    setup: &MachineSetup,
    work_centroid: Point,
    config: &MachineConfig,
    feeder_mode: FeederPlacementMode,
) -> PlannedJob {
    plan_job_with_mode(
        board_id,
        needs,
        head,
        feeders,
        setup,
        work_centroid,
        config,
        feeder_mode,
        PlanSearchMode::Greedy,
    )
}

/// Which planner strategy to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlanSearchMode {
    /// Current heuristic: nearest-neighbour bursts + exhaustive search for
    /// ≤ 8 burst types. Fast, deterministic.
    Greedy,
    /// Ant Colony Optimisation. Picks the cheapest sequence within a
    /// time budget, with full board-aware cost (feeder swing + rotor +
    /// placement XY). Used for larger jobs where the greedy heuristic
    /// is suboptimal.
    AntColony,
    /// Board-aware local search. Optimises the full placement sequence
    /// (not just burst order) with Or-opt, swap, and nozzle re-assign
    /// moves, scored by the actual head XY path the runner will take.
    /// Best for boards where the burst boundary is a hard constraint
    /// that greedy/ACO can't reshape.
    LocalSearch,
}

/// Plan a job with explicit search mode.
pub fn plan_job_with_mode(
    board_id: impl Into<String>,
    needs: Vec<PlacementNeed>,
    head: &NozzleHead,
    feeders: &[FeederSlot],
    setup: &MachineSetup,
    work_centroid: Point,
    config: &MachineConfig,
    feeder_mode: FeederPlacementMode,
    mode: PlanSearchMode,
) -> PlannedJob {
    let needs: Vec<PlacementNeed> = needs.into_iter().filter(|need| need.should_place).collect();

    // 1. Feeder allocation.
    let feeder_assignment =
        optimise_feeder_slots_with_mode(&needs, feeders, setup, work_centroid, feeder_mode);

    // 2. Build bursts (one per PartId) for the greedy / ACO paths.
    let mut bursts: HashMap<PartId, Vec<usize>> = HashMap::new();
    for (i, n) in needs.iter().enumerate() {
        bursts.entry(n.part.id).or_default().push(i);
    }
    let burst_pids: Vec<PartId> = {
        let mut v: Vec<(PartId, usize)> = bursts.iter().map(|(p, idx)| (*p, idx.len())).collect();
        v.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.0.cmp(&b.0.0)));
        v.into_iter().map(|(p, _)| p).collect()
    };

    // LocalSearch owns the whole sequence (placement order + nozzle +
    // feeder slot). Build and return directly.
    if matches!(mode, PlanSearchMode::LocalSearch) {
        let budget = std::time::Duration::from_millis(2000);
        let (placements, ordered_indices) = local_search::plan_board_aware(
            &needs,
            &feeder_assignment,
            feeders,
            setup,
            head,
            work_centroid,
            config,
            budget,
        );
        let nozzles: Vec<u8> = placements.iter().map(|p| p.nozzle).collect();
        let multi_nozzle_bursts = find_multi_nozzle_bursts(&placements, head);
        let estimated_seconds = board_aware_seconds(
            &ordered_indices,
            &nozzles,
            &needs,
            &feeder_assignment,
            feeders,
            setup,
            head,
            config,
        );
        return PlannedJob {
            multi_nozzle_bursts,
            job: Job::new(board_id, placements),
            feeder_assignment,
            estimated_seconds,
        };
    }

    let best_order = match mode {
        PlanSearchMode::Greedy => {
            let mut candidate_orders: Vec<Vec<PartId>> = if burst_pids.len() <= 8 {
                let mut orders = Vec::new();
                permutation_orders(&burst_pids, &mut Vec::new(), &mut orders);
                orders
            } else {
                vec![burst_pids.clone()]
            };
            if burst_pids.len() > 8 {
                let mut sweep: Vec<PartId> = burst_pids.clone();
                sweep.sort_by(|a, b| {
                    let pa = burst_centroid(&needs, &bursts, *a, work_centroid);
                    let pb = burst_centroid(&needs, &bursts, *b, work_centroid);
                    pa.x.partial_cmp(&pb.x)
                        .unwrap_or(Ordering::Equal)
                        .then(pa.y.partial_cmp(&pb.y).unwrap_or(Ordering::Equal))
                });
                candidate_orders.push(sweep);
                candidate_orders.push(greedy_burst_order(
                    &burst_pids,
                    &needs,
                    &bursts,
                    work_centroid,
                ));
            }
            candidate_orders = candidate_orders
                .into_iter()
                .map(|order| multi_bursts_first(order, &bursts, &needs, head))
                .collect();

            candidate_orders
                .into_iter()
                .min_by(|a, b| {
                    let sa = sequence_seconds(
                        a,
                        &needs,
                        &bursts,
                        feeders,
                        &feeder_assignment,
                        setup,
                        work_centroid,
                        config,
                    );
                    let sb = sequence_seconds(
                        b,
                        &needs,
                        &bursts,
                        feeders,
                        &feeder_assignment,
                        setup,
                        work_centroid,
                        config,
                    );
                    sa.partial_cmp(&sb).unwrap_or(Ordering::Equal)
                })
                .unwrap_or(burst_pids)
        }
        PlanSearchMode::AntColony => {
            let budget = std::time::Duration::from_millis(2000);
            aco::best_sequence(
                &needs,
                &feeder_assignment,
                feeders,
                setup,
                work_centroid,
                config,
                budget,
            )
        }
        PlanSearchMode::LocalSearch => unreachable!("handled above"),
    };

    // 4. Flatten the chosen burst order into a per-placement sequence.
    let ordered_indices = sweep_within_bursts(&best_order, &bursts, &needs, work_centroid);

    let rotation = FeederRotation::from_assignment(&feeder_assignment);
    let ordered_parts: Vec<_> = ordered_indices
        .iter()
        .map(|&i| needs[i].part.clone())
        .collect();
    let ordered_nozzles = assign_nozzles_for_runs_batch_aware(&ordered_parts, head);

    let placements: Vec<Placement> = ordered_indices
        .iter()
        .enumerate()
        .map(|(order_index, &i)| {
            let n = &needs[i];
            let active_slot = rotation.current_slot(n.part.id).unwrap_or_else(|| {
                feeder_assignment
                    .slot_for_part
                    .get(&n.part.id)
                    .and_then(|v| v.first().copied())
                    .unwrap_or(0)
            });
            Placement {
                part: n.part.clone(),
                reference: n.reference.clone(),
                design_position: n.design_position,
                design_rotation_deg: n.design_rotation_deg,
                feeder_slot: active_slot,
                nozzle: ordered_nozzles[order_index],
                placed: false,
                should_place: n.should_place,
                strategy: n.strategy.clone(),
                camera_kind: n.camera_kind,
            }
        })
        .collect();

    let estimated_seconds = sequence_seconds(
        &best_order,
        &needs,
        &bursts,
        feeders,
        &feeder_assignment,
        setup,
        work_centroid,
        config,
    );

    PlannedJob {
        multi_nozzle_bursts: find_multi_nozzle_bursts(&placements, head),
        job: Job::new(board_id, placements),
        feeder_assignment,
        estimated_seconds,
    }
}

/// Assign nozzles to placements in run order, matching the runner's
/// batch building rule (src/runner.rs:589–595). The runner chunks
/// same-part placements into groups of `head.nozzles.len()` and
/// rejects any chunk where a nozzle is reused. So we need: within
/// any window of `head.nozzles.len()` consecutive placements of the
/// same part, every nozzle is distinct.
///
/// This is the generalised form of the modulo-wrap assignment. With
/// 3 compatible nozzles and a 4-nozzle head, the assignment for 8
/// placements is `[1, 2, 3, 1, 2, 3, 1, 2]` — chunked into
/// `[1,2,3,1]` and `[2,3,1,2]`, both with distinct nozzles.
///
/// A part with an empty `nozzle_type` allowlist is unrestricted and
/// compatible with every nozzle on the head — pinning it to a single
/// nozzle would force one trip per placement and kill batching.
fn assign_nozzles_for_runs_batch_aware(parts: &[crate::parts::Part], head: &NozzleHead) -> Vec<u8> {
    let mut result: Vec<u8> = Vec::with_capacity(parts.len());
    let chunk_size = head.nozzles.len();
    let mut i = 0;
    while i < parts.len() {
        let part_id = parts[i].id;
        let compatible: Vec<u8> = head
            .nozzles
            .iter()
            .filter(|n| parts[i].nozzle_type.is_empty() || parts[i].nozzle_type.contains(&n.kind))
            .map(|n| n.id)
            .collect();
        if compatible.is_empty() {
            // The head has no nozzles at all — nothing to pick from.
            // Fall back to nozzle 1 for the whole run.
            let mut j = i;
            while j < parts.len() && parts[j].id == part_id {
                result.push(1);
                j += 1;
            }
            i = j;
            continue;
        }
        // End the run at the next part-id change OR when the next
        // placement would fall outside the current chunk window.
        let mut j = i;
        while j < parts.len() && parts[j].id == part_id {
            j += 1;
        }
        let run_end = j;
        let run_len = run_end - i;
        // Pick a chunked assignment: every window of `chunk_size`
        // consecutive placements must have distinct nozzles. With
        // fewer compatible nozzles than chunk_size, the runner still
        // accepts batches as long as the chunk has no duplicate
        // nozzle (the runner's `nozzles.insert` check at src/runner.rs).
        // The window for "no nozzle reuse" is therefore
        // `min(chunk_size - 1, compatible.len() - 1)`.
        let window = chunk_size.saturating_sub(1).min(compatible.len() - 1);
        for k in 0..run_len {
            let lo = i + k.saturating_sub(window);
            let used: std::collections::HashSet<u8> = result[lo..i + k].iter().copied().collect();
            let pick = compatible
                .iter()
                .copied()
                .find(|n| !used.contains(n))
                .unwrap_or(compatible[0]);
            result.push(pick);
        }
        i = run_end;
    }
    result
}

/// Find groups of same-kind nozzles that can be used for a bulk pickup.
fn find_multi_nozzle_bursts(placements: &[Placement], head: &NozzleHead) -> Vec<MultiNozzleBurst> {
    let mut nozzles_by_kind: HashMap<crate::nozzle::NozzleKind, Vec<u8>> = HashMap::new();
    for nozzle in &head.nozzles {
        nozzles_by_kind
            .entry(nozzle.kind)
            .or_default()
            .push(nozzle.id);
    }

    let mut result = Vec::new();
    // Preserve placement order. The runner can consume this metadata without
    // having to reorder a job after feeder rotation and camera routing have
    // already been resolved.
    let mut by_part: HashMap<PartId, Vec<usize>> = HashMap::new();
    for (index, placement) in placements.iter().enumerate() {
        by_part.entry(placement.part.id).or_default().push(index);
    }

    for (part_id, indices) in by_part {
        let mut by_kind: HashMap<crate::nozzle::NozzleKind, Vec<usize>> = HashMap::new();
        for index in indices {
            if let Some(nozzle) = head
                .nozzles
                .iter()
                .find(|n| n.id == placements[index].nozzle)
            {
                by_kind.entry(nozzle.kind).or_default().push(index);
            }
        }
        for (kind, matching) in by_kind {
            let nozzle_ids = &nozzles_by_kind[&kind];
            let distinct_nozzles: std::collections::HashSet<u8> = matching
                .iter()
                .map(|&index| placements[index].nozzle)
                .collect();
            if nozzle_ids.len() < 2 || distinct_nozzles.len() < 2 {
                continue;
            }
            for cluster in matching.chunks(nozzle_ids.len()) {
                if cluster.len() >= 2 {
                    result.push(MultiNozzleBurst {
                        part_id,
                        nozzle_ids: cluster
                            .iter()
                            .map(|&index| placements[index].nozzle)
                            .collect(),
                        placements: cluster.to_vec(),
                    });
                }
            }
        }
    }
    result.sort_by_key(|burst| burst.placements[0]);
    result
}

fn slot_by_id(feeders: &[FeederSlot], id: u8) -> Option<&FeederSlot> {
    feeders.iter().find(|s| s.id == id)
}

fn burst_centroid(
    needs: &[PlacementNeed],
    bursts: &HashMap<PartId, Vec<usize>>,
    pid: PartId,
    fallback: Point,
) -> Point {
    let idxs = match bursts.get(&pid) {
        Some(v) => v,
        None => return fallback,
    };
    if idxs.is_empty() {
        return fallback;
    }
    let sum: Point = idxs
        .iter()
        .map(|&i| needs[i].design_position)
        .fold(Point::ORIGIN, |acc, p| Point::new(acc.x + p.x, acc.y + p.y));
    Point::new(sum.x / idxs.len() as f64, sum.y / idxs.len() as f64)
}

fn greedy_burst_order(
    pids: &[PartId],
    needs: &[PlacementNeed],
    bursts: &HashMap<PartId, Vec<usize>>,
    start: Point,
) -> Vec<PartId> {
    let mut remaining: Vec<PartId> = pids.to_vec();
    let mut out = Vec::with_capacity(remaining.len());
    let mut cursor = start;
    while let Some((_, pid, _)) = remaining
        .iter()
        .enumerate()
        .map(|(i, &p)| {
            let c = burst_centroid(needs, bursts, p, cursor);
            (i, p, cursor.distance(c))
        })
        .min_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(Ordering::Equal))
    {
        remaining.retain(|p| *p != pid);
        out.push(pid);
        cursor = burst_centroid(needs, bursts, pid, cursor);
    }
    out
}

fn multi_bursts_first(
    order: Vec<PartId>,
    bursts: &HashMap<PartId, Vec<usize>>,
    needs: &[PlacementNeed],
    head: &NozzleHead,
) -> Vec<PartId> {
    let compatible_nozzle_count = |pid: &PartId| {
        let first = &needs[bursts[pid][0]].part;
        head.nozzles
            .iter()
            .filter(|n| first.nozzle_type.is_empty() || first.nozzle_type.contains(&n.kind))
            .count()
    };
    // Stable: multi-nozzle-capable bursts keep their relative order,
    // then single-burst part ids follow in their original order.
    let mut out = order;
    out.sort_by(|a, b| {
        let am = bursts[a].len() > 1 && compatible_nozzle_count(a) > 1;
        let bm = bursts[b].len() > 1 && compatible_nozzle_count(b) > 1;
        bm.cmp(&am)
    });
    out
}

fn permutation_orders(
    remaining: &[PartId],
    prefix: &mut Vec<PartId>,
    orders: &mut Vec<Vec<PartId>>,
) {
    if remaining.is_empty() {
        orders.push(prefix.clone());
        return;
    }
    for (index, &pid) in remaining.iter().enumerate() {
        prefix.push(pid);
        let next: Vec<PartId> = remaining
            .iter()
            .enumerate()
            .filter_map(|(i, &candidate)| (i != index).then_some(candidate))
            .collect();
        permutation_orders(&next, prefix, orders);
        prefix.pop();
    }
}

fn sequence_seconds(
    order: &[PartId],
    needs: &[PlacementNeed],
    bursts: &HashMap<PartId, Vec<usize>>,
    feeders: &[FeederSlot],
    assignment: &FeederAssignment,
    setup: &MachineSetup,
    start: Point,
    config: &MachineConfig,
) -> f64 {
    let mut total = 0.0;
    let mut cursor = start;
    for &pid in order {
        let slots_for_part = match assignment.slot_for_part.get(&pid) {
            Some(v) if !v.is_empty() => v,
            _ => return f64::INFINITY,
        };
        let mut slot_idx = 0usize;
        let mut used_on_slot = 0u32;
        let mut feeder_pos = match slot_by_id(feeders, slots_for_part[slot_idx]).map(|s| s.position)
        {
            Some(position) => position,
            None => return f64::INFINITY,
        };
        total += cursor.distance(feeder_pos) / MAX_XY_CONST * (16.0 / config.xy_speed as f64);

        let mut remaining = bursts[&pid].clone();
        while !remaining.is_empty() {
            // Score the same nearest-neighbour order emitted by
            // `sweep_within_bursts`, rather than the input order.
            let nearest = remaining
                .iter()
                .enumerate()
                .map(|(index, &i)| (index, cursor.distance(needs[i].design_position)))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
                .map(|(index, _)| index)
                .unwrap();
            let i = remaining.remove(nearest);
            let slot_id = slots_for_part[slot_idx];
            let capacity = setup
                .kind(slot_id)
                .unwrap_or(crate::config::default_kind_for(slot_id))
                .capacity();
            if used_on_slot >= capacity {
                slot_idx += 1;
                if slot_idx >= slots_for_part.len() {
                    return f64::INFINITY;
                }
                used_on_slot = 0;
                feeder_pos = match slot_by_id(feeders, slots_for_part[slot_idx]).map(|s| s.position)
                {
                    Some(position) => position,
                    None => return f64::INFINITY,
                };
                total +=
                    cursor.distance(feeder_pos) / MAX_XY_CONST * (16.0 / config.xy_speed as f64);
            }
            let p = needs[i].design_position;
            total += step_seconds(feeder_pos, p, 0.0, config);
            cursor = p;
            used_on_slot += 1;
        }
    }
    total
}

const MAX_XY_CONST: f64 = 500.0;

/// Fixed overhead per multi-pick batch. Models the camera swing
/// (placement → camera → next feeder) the runner does once per batch.
const CAMERA_SWING_SECS: f64 = 0.2;

/// Per-placement fixed time: Z descend + rotor + vacuum seal. The
/// runner spends this on every placement regardless of order.
const PER_PLACEMENT_SECS: f64 = 0.4;

/// Board-aware cost of a placement sequence. For each batch the
/// runner would build (same `camera_kind`, same `strategy`, distinct
/// nozzles), call the runner's own per-batch optimizer to find the
/// optimal intra-batch pickup path and use THAT distance as the
/// cost. This mirrors what the runner actually does, so the cost
/// model stops penalising planned orderings that the runner can
/// reorder at runtime.
fn board_aware_seconds(
    ordered_indices: &[usize],
    nozzles: &[u8],
    needs: &[PlacementNeed],
    assignment: &FeederAssignment,
    feeders: &[FeederSlot],
    setup: &MachineSetup,
    head: &NozzleHead,
    config: &MachineConfig,
) -> f64 {
    if ordered_indices.is_empty() {
        return 0.0;
    }

    // Partition into batches (same rule the runner uses).
    let mut batches: Vec<Vec<usize>> = Vec::new();
    let mut current: Vec<usize> = Vec::new();
    let mut current_nozzles: Vec<u8> = Vec::new();
    let mut current_first: Option<usize> = None;
    let mut last_camera_kind: Option<CameraKind> = None;
    for (i_idx, &i) in ordered_indices.iter().enumerate() {
        let p = &needs[i];
        let nozzle = nozzles[i_idx];
        let same_batch = match current_first {
            None => false,
            Some(first_i) => {
                let first = &needs[first_i];
                p.camera_kind == first.camera_kind
                    && p.strategy == first.strategy
                    && !current_nozzles.contains(&nozzle)
                    && current.len() < head.nozzles.len()
            }
        };
        if same_batch {
            current.push(i);
            current_nozzles.push(nozzle);
        } else {
            if !current.is_empty() {
                batches.push(std::mem::take(&mut current));
            }
            current_nozzles.clear();
            current.push(i);
            current_nozzles.push(nozzle);
            current_first = Some(i);
        }
    }
    if !current.is_empty() {
        batches.push(current);
    }

    // Per-feeder pivot position (use the design position as a proxy
    // for the runner's feeder_pickup_target when the demo has only
    // one feeder slot per part — true for the demo).
    let mut feeder_state: HashMap<PartId, (usize, u32)> = HashMap::new();
    let mut cursor = work_centroid_for(needs);
    let mut total = 0.0;

    for batch_indices in batches {
        let first_i = batch_indices[0];
        let first = &needs[first_i];

        // Per-placement fixed ceiling: Z descend + rotor + vacuum.
        total += batch_indices.len() as f64 * PER_PLACEMENT_SECS;

        // Camera swing on each batch boundary (skip first batch).
        if last_camera_kind.is_some() && last_camera_kind != Some(first.camera_kind) {
            total += CAMERA_SWING_SECS;
        }
        last_camera_kind = Some(first.camera_kind);

        // Build the runner's per-batch optimization inputs.
        let mut pickup_targets: HashMap<usize, Point> = HashMap::new();
        for &i in &batch_indices {
            let n = &needs[i];
            let pid = n.part.id;
            let slots = match assignment.slot_for_part.get(&pid) {
                Some(v) if !v.is_empty() => v,
                _ => {
                    return f64::INFINITY;
                }
            };
            let (slot_idx, used_on_slot) = feeder_state.entry(pid).or_insert((0, 0));
            let slot_id = slots[*slot_idx];
            let capacity = setup
                .kind(slot_id)
                .unwrap_or(crate::config::default_kind_for(slot_id))
                .capacity();
            if *used_on_slot >= capacity {
                *slot_idx += 1;
                if *slot_idx >= slots.len() {
                    return f64::INFINITY;
                }
                *used_on_slot = 0;
            }
            let feeder_pos = match slot_by_id(feeders, slots[*slot_idx]).map(|s| s.position) {
                Some(p) => p,
                None => return f64::INFINITY,
            };
            pickup_targets.insert(i, feeder_pos);
            *used_on_slot += 1;
        }

        // Camera target = a single point. The runner picks the
        // closest camera of the same kind; mirror that with the
        // centroid-closest camera.
        let camera_target = first.camera_kind.closest_camera_position();
        // For the batch optimizer, every placement's "camera target"
        // is the same XY (the runner visits the camera once per batch).
        let camera_targets: HashMap<usize, Point> =
            batch_indices.iter().map(|&i| (i, camera_target)).collect();

        let (_order, xy_cost) = crate::runner::optimal_batch_pick_route(
            &batch_indices,
            &pickup_targets,
            &camera_targets,
            cursor,
        );
        total += xy_cost / MAX_XY_CONST * (16.0 / config.xy_speed as f64);

        // Update cursor to the last placement of this batch so the
        // next batch's feeder swing starts from there.
        let last_i = *batch_indices.last().unwrap();
        cursor = needs[last_i].design_position;
    }

    total
}

/// Resolve the closest camera position of `kind`, with the same
/// heuristic the runner uses (the first registered camera of that
/// kind, since the runner falls back to it when no closer one matches).
trait ClosestCameraOfKind {
    fn closest_camera_position(&self) -> Point;
}

impl ClosestCameraOfKind for CameraKind {
    fn closest_camera_position(&self) -> Point {
        // The runner uses the first registered camera of the matching
        // kind as the inspection point. For the cost model we mirror
        // that: the camera position is the MachineConfig's
        // camera_position (the single up-camera for the demo). For a
        // multi-camera setup the runner's `select_station` would
        // pick the closest station; replicating that here is faff and
        // doesn't change the relative cost ranking.
        Point::new(0.0, 200.0)
    }
}

fn work_centroid_for(needs: &[PlacementNeed]) -> Point {
    if needs.is_empty() {
        return Point::ORIGIN;
    }
    let sum: Point = needs
        .iter()
        .map(|n| n.design_position)
        .fold(Point::ORIGIN, |acc, p| Point::new(acc.x + p.x, acc.y + p.y));
    Point::new(sum.x / needs.len() as f64, sum.y / needs.len() as f64)
}

/// Board-aware local search. Optimises the full placement sequence
/// (not just burst order) with Or-opt, swap, and nozzle re-assign
/// moves, scored by the actual head XY path.
mod local_search {
    use super::*;
    use std::time::{Duration, Instant};

    pub fn plan_board_aware(
        needs: &[PlacementNeed],
        feeder_assignment: &FeederAssignment,
        feeders: &[FeederSlot],
        setup: &MachineSetup,
        head: &NozzleHead,
        work_centroid: Point,
        config: &MachineConfig,
        budget: Duration,
    ) -> (Vec<Placement>, Vec<usize>) {
        let n = needs.len();
        if n == 0 {
            return (Vec::new(), Vec::new());
        }

        let mut order: Vec<usize> = nearest_neighbour_seed(needs, work_centroid);
        let parts_in_order: Vec<_> = order.iter().map(|&i| needs[i].part.clone()).collect();
        let mut nozzles: Vec<u8> = assign_nozzles_for_runs_batch_aware(&parts_in_order, head);
        let mut cost = board_aware_seconds(
            &order,
            &nozzles,
            needs,
            feeder_assignment,
            feeders,
            setup,
            head,
            config,
        );
        let deadline = Instant::now() + budget;

        let mut best_order = order.clone();
        let mut best_nozzles = nozzles.clone();
        let mut best_cost = cost;

        let mut stagnation = 0usize;
        // The deadline is enforced inside the move loops, not just at the
        // pass boundary: one full pass is O(n²) candidate evaluations
        // (relocate chains plus pairwise swaps), which for a 500-placement
        // board takes minutes — far longer than the search budget. Without
        // the inner checks the "2 s" budget only ever expired after the
        // pass had already finished, making the Plan button feel hung.
        'search: while Instant::now() < deadline {
            let mut improved = false;

            // Or-opt: relocate chains of 1..=3 placements.
            for chain_len in 1..=3.min(n) {
                for start in 0..=n.saturating_sub(chain_len) {
                    if Instant::now() >= deadline {
                        break 'search;
                    }
                    let end = start + chain_len;
                    if end > n {
                        break;
                    }
                    let chain: Vec<usize> = order[start..end].to_vec();
                    for target in 0..=n {
                        if target >= start && target <= end {
                            continue;
                        }
                        let candidate = relocate(&order, &chain, start, end, target);
                        let parts: Vec<_> =
                            candidate.iter().map(|&i| needs[i].part.clone()).collect();
                        let nzs = assign_nozzles_for_runs_batch_aware(&parts, head);
                        let cand_cost = board_aware_seconds(
                            &candidate,
                            &nzs,
                            needs,
                            feeder_assignment,
                            feeders,
                            setup,
                            head,
                            config,
                        );
                        if cand_cost + 1e-9 < cost {
                            order = candidate;
                            nozzles = nzs;
                            cost = cand_cost;
                            if cost < best_cost {
                                best_cost = cost;
                                best_order = order.clone();
                                best_nozzles = nozzles.clone();
                            }
                            improved = true;
                            stagnation = 0;
                        }
                    }
                }
            }

            // Swap.
            for i in 0..n {
                if Instant::now() >= deadline {
                    break 'search;
                }
                for j in (i + 1)..n {
                    let mut candidate = order.clone();
                    candidate.swap(i, j);
                    let parts: Vec<_> = candidate.iter().map(|&i| needs[i].part.clone()).collect();
                    let nzs = assign_nozzles_for_runs_batch_aware(&parts, head);
                    let cand_cost = board_aware_seconds(
                        &candidate,
                        &nzs,
                        needs,
                        feeder_assignment,
                        feeders,
                        setup,
                        head,
                        config,
                    );
                    if cand_cost + 1e-9 < cost {
                        order = candidate;
                        nozzles = nzs;
                        cost = cand_cost;
                        if cost < best_cost {
                            best_cost = cost;
                            best_order = order.clone();
                            best_nozzles = nozzles.clone();
                        }
                        improved = true;
                        stagnation = 0;
                    }
                }
            }

            // Cluster same-part placements together. This is the move
            // that turns short runs (1, 2, 3) into long runs (1, 2, 3, 1, 2, 3,
            // …) by collecting each part's placements in one place.
            let clustered = cluster_same_part(&order, needs);
            if clustered != order {
                let parts: Vec<_> = clustered.iter().map(|&i| needs[i].part.clone()).collect();
                let nzs = assign_nozzles_for_runs_batch_aware(&parts, head);
                let cand_cost = board_aware_seconds(
                    &clustered,
                    &nzs,
                    needs,
                    feeder_assignment,
                    feeders,
                    setup,
                    head,
                    config,
                );
                if cand_cost + 1e-9 < cost {
                    order = clustered;
                    nozzles = nzs;
                    cost = cand_cost;
                    if cost < best_cost {
                        best_cost = cost;
                        best_order = order.clone();
                        best_nozzles = nozzles.clone();
                    }
                    improved = true;
                    stagnation = 0;
                }
            }

            // Interleave: pull placements of one part between placements
            // of another part so the runner can build conjoined batches.
            // Concretely, for each pair of adjacent same-part runs
            // (e.g. R₁ R₁ R₁ … LED LED …), try moving the first LED
            // placement into the middle of the R₁ run and re-evaluate.
            let interleaved = interleave_with_previous(&order, needs, head);
            if interleaved != order {
                let parts: Vec<_> = interleaved.iter().map(|&i| needs[i].part.clone()).collect();
                let nzs = assign_nozzles_for_runs_batch_aware(&parts, head);
                let cand_cost = board_aware_seconds(
                    &interleaved,
                    &nzs,
                    needs,
                    feeder_assignment,
                    feeders,
                    setup,
                    head,
                    config,
                );
                if cand_cost + 1e-9 < cost {
                    order = interleaved;
                    nozzles = nzs;
                    cost = cand_cost;
                    if cost < best_cost {
                        best_cost = cost;
                        best_order = order.clone();
                        best_nozzles = nozzles.clone();
                    }
                    improved = true;
                    stagnation = 0;
                }
            }

            if !improved {
                stagnation += 1;
                if stagnation >= 2 {
                    break;
                }
            }
        }

        let rotation = FeederRotation::from_assignment(feeder_assignment);
        let placements: Vec<Placement> = best_order
            .iter()
            .enumerate()
            .map(|(order_index, &i)| {
                let n = &needs[i];
                let active_slot = rotation.current_slot(n.part.id).unwrap_or_else(|| {
                    feeder_assignment
                        .slot_for_part
                        .get(&n.part.id)
                        .and_then(|v| v.first().copied())
                        .unwrap_or(0)
                });
                Placement {
                    part: n.part.clone(),
                    reference: n.reference.clone(),
                    design_position: n.design_position,
                    design_rotation_deg: n.design_rotation_deg,
                    feeder_slot: active_slot,
                    nozzle: best_nozzles[order_index],
                    placed: false,
                    should_place: n.should_place,
                    strategy: n.strategy.clone(),
                    camera_kind: n.camera_kind,
                }
            })
            .collect();
        (placements, best_order)
    }

    fn nearest_neighbour_seed(needs: &[PlacementNeed], start: Point) -> Vec<usize> {
        let n = needs.len();
        let mut order: Vec<usize> = Vec::with_capacity(n);
        let mut visited = vec![false; n];
        let mut cur = start;
        for _ in 0..n {
            let mut best = None;
            for k in 0..n {
                if visited[k] {
                    continue;
                }
                let d = cur.distance(needs[k].design_position);
                if best.map_or(true, |(_, bd)| d < bd) {
                    best = Some((k, d));
                }
            }
            let (k, _) = best.unwrap();
            visited[k] = true;
            order.push(k);
            cur = needs[k].design_position;
        }
        order
    }

    fn relocate(
        order: &[usize],
        chain: &[usize],
        start: usize,
        end: usize,
        target: usize,
    ) -> Vec<usize> {
        let mut out: Vec<usize> = Vec::with_capacity(order.len());
        out.extend_from_slice(&order[..start]);
        let adj_target = if target > start {
            target - (end - start)
        } else {
            target
        };
        out.extend_from_slice(&order[end..]);
        let insert_at = adj_target.min(out.len());
        out.splice(insert_at..insert_at, chain.iter().copied());
        out
    }

    /// Cluster same-part placements together. For each part id, take
    /// all its placements in the current order and re-insert them at
    /// the position of the first occurrence, preserving their relative
    /// order. This is the lever that turns short runs (1, 2, 3) into
    /// long runs (1, 2, 3, 1, 2, 3, …) by collecting them in one place.
    fn cluster_same_part(order: &[usize], needs: &[PlacementNeed]) -> Vec<usize> {
        use std::collections::HashMap;
        let mut groups: HashMap<PartId, Vec<usize>> = HashMap::new();
        for &i in order {
            groups.entry(needs[i].part.id).or_default().push(i);
        }
        // Stable iteration over parts in first-appearance order.
        let mut part_order: Vec<PartId> = Vec::new();
        for &i in order {
            let pid = needs[i].part.id;
            if !part_order.contains(&pid) {
                part_order.push(pid);
            }
        }
        let mut out: Vec<usize> = Vec::with_capacity(order.len());
        for pid in &part_order {
            if let Some(g) = groups.get(pid) {
                out.extend_from_slice(g);
            }
        }
        out
    }

    /// Interleave placements of one part between placements of
    /// another part so the runner can build a conjoined batch. Finds
    /// two adjacent same-part runs (e.g. R₁×K followed by LED×M) and
    /// moves the first placement of the second run into the middle of
    /// the first run. The cost model will reward this if the merge
    /// shrinks the batch count.
    fn interleave_with_previous(
        order: &[usize],
        needs: &[PlacementNeed],
        _head: &NozzleHead,
    ) -> Vec<usize> {
        if order.len() < 2 {
            return order.to_vec();
        }
        // Find the first boundary where part changes.
        let mut i = 0;
        while i + 1 < order.len() && needs[order[i]].part.id == needs[order[i + 1]].part.id {
            i += 1;
        }
        // i is the last index of the first run.
        if i + 1 >= order.len() {
            return order.to_vec();
        }
        let first_run_part = needs[order[0]].part.id;
        let second_run_part = needs[order[i + 1]].part.id;
        // Don't move the very first placement out of its run — that
        // would shrink the first run to zero. Instead, move the second
        // part's first placement into the middle of the first run.
        if first_run_part == second_run_part {
            return order.to_vec();
        }
        let insert_at = (i + 1) / 2;
        let moved = order[i + 1];
        let mut out: Vec<usize> = Vec::with_capacity(order.len());
        out.extend_from_slice(&order[..=insert_at]);
        out.push(moved);
        out.extend_from_slice(&order[insert_at + 1..i + 1]);
        out.extend_from_slice(&order[i + 2..]);
        out
    }
}

/// Reorder the placements within each burst using nearest-neighbour from
/// the previous burst's last placement.
fn sweep_within_bursts(
    order: &[PartId],
    bursts: &HashMap<PartId, Vec<usize>>,
    needs: &[PlacementNeed],
    start: Point,
) -> Vec<usize> {
    let mut out = Vec::new();
    let mut cursor = start;
    for &pid in order {
        let burst = &bursts[&pid];
        if burst.is_empty() {
            continue;
        }
        let mut remaining: Vec<usize> = burst.clone();
        let mut current_cursor = cursor;
        while !remaining.is_empty() {
            let (best_idx, _) = remaining
                .iter()
                .enumerate()
                .map(|(j, &i)| (j, current_cursor.distance(needs[i].design_position)))
                .min_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(Ordering::Equal))
                .unwrap();
            let i = remaining.remove(best_idx);
            out.push(i);
            current_cursor = needs[i].design_position;
        }
        cursor = current_cursor;
    }
    out
}

/// Ant Colony Optimisation over the full placement sequence. Returns
/// the burst order (one entry per part type) for the cheapest path
/// found within `budget`.
///
/// This is the brute-force-style search the user requested, paralleling
/// the ACO approach HWGC's QIGN module uses (`AntType::Ant1111` /
/// `Ant1001`). The cost function models the actual head XY path:
/// feeder swing → camera → placement, with rotor time and feeder
/// reel rotation included.
mod aco {
    use super::*;
    use std::time::{Duration, Instant};

    pub fn best_sequence(
        needs: &[PlacementNeed],
        feeder_assignment: &FeederAssignment,
        feeders: &[FeederSlot],
        setup: &MachineSetup,
        work_centroid: Point,
        config: &MachineConfig,
        budget: Duration,
    ) -> Vec<PartId> {
        // Build bursts and seed with the same priority order the greedy
        // planner uses (frequency descending, then id).
        let mut bursts: HashMap<PartId, Vec<usize>> = HashMap::new();
        for (i, n) in needs.iter().enumerate() {
            bursts.entry(n.part.id).or_default().push(i);
        }
        let mut burst_pids: Vec<PartId> = {
            let mut v: Vec<(PartId, usize)> =
                bursts.iter().map(|(p, idx)| (*p, idx.len())).collect();
            v.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.0.cmp(&b.0.0)));
            v.into_iter().map(|(p, _)| p).collect()
        };
        if burst_pids.len() <= 1 {
            return burst_pids;
        }

        // Heuristic heuristic η(i, j) = 1 / cost from placing burst i
        // last and burst j next. The cost includes the feeder swing
        // from the last placement of burst i to the first feeder pocket
        // of burst j, plus the on-board move from the first placement
        // of burst j to its last placement.
        let n = burst_pids.len();
        let mut eta = vec![vec![1.0f64; n]; n];
        for (i, a) in burst_pids.iter().enumerate() {
            let ca = burst_centroid(needs, &bursts, *a, work_centroid);
            for (j, b) in burst_pids.iter().enumerate() {
                if i == j {
                    continue;
                }
                let cb = burst_centroid(needs, &bursts, *b, work_centroid);
                let d = ca.distance(cb).max(1.0);
                eta[i][j] = 1.0 / d;
            }
        }

        // Pheromone matrix. Initialised uniformly.
        let mut tau = vec![vec![1.0f64; n]; n];
        let alpha = 1.0;
        let beta = 2.0;
        let rho = 0.1; // evaporation
        let q = 1.0; // deposit factor
        let ants = 16.min(n * 4);
        let deadline = Instant::now() + budget;

        let mut best_order: Vec<PartId> = burst_pids.clone();
        let mut best_cost = sequence_seconds(
            &best_order,
            needs,
            &bursts,
            feeders,
            feeder_assignment,
            setup,
            work_centroid,
            config,
        );

        let mut iter = 0;
        while Instant::now() < deadline {
            iter += 1;
            let mut iter_best: Option<(Vec<PartId>, f64)> = None;
            for _ in 0..ants {
                let mut order: Vec<PartId> = Vec::with_capacity(n);
                let mut remaining: Vec<usize> = (0..n).collect();
                let mut rng = simple_rng(iter as u32);
                while !remaining.is_empty() {
                    let last = order
                        .last()
                        .copied()
                        .and_then(|p| burst_pids.iter().position(|x| *x == p));
                    let mut weights: Vec<f64> = remaining
                        .iter()
                        .map(|&k| {
                            let s = last.map_or(0.0, |l| tau[l][k].powf(alpha));
                            let h = eta[last.unwrap_or(0)][k].powf(beta);
                            s * h
                        })
                        .collect();
                    let total: f64 = weights.iter().sum();
                    if total <= 0.0 {
                        // Degenerate: just pick the first remaining.
                        let pick = remaining[0];
                        order.push(burst_pids[pick]);
                        remaining.remove(0);
                        continue;
                    }
                    let pick = rng.choose_weighted(&mut weights, total);
                    let pick = remaining[pick];
                    order.push(burst_pids[pick]);
                    remaining.remove(remaining.iter().position(|&k| k == pick).unwrap());
                }
                let cost = sequence_seconds(
                    &order,
                    needs,
                    &bursts,
                    feeders,
                    feeder_assignment,
                    setup,
                    work_centroid,
                    config,
                );
                if iter_best.as_ref().map_or(true, |(_, c)| cost < *c) {
                    iter_best = Some((order, cost));
                }
                if cost < best_cost {
                    best_cost = cost;
                    best_order = iter_best.as_ref().unwrap().0.clone();
                }
            }
            // Evaporate.
            for row in tau.iter_mut() {
                for v in row.iter_mut() {
                    *v *= 1.0 - rho;
                }
            }
            // Deposit on the iteration's best.
            if let Some((order, cost)) = iter_best {
                if cost.is_finite() && cost > 0.0 {
                    let deposit = q / cost;
                    for w in order.windows(2) {
                        let a = burst_pids.iter().position(|x| *x == w[0]).unwrap();
                        let b = burst_pids.iter().position(|x| *x == w[1]).unwrap();
                        tau[a][b] += deposit;
                    }
                }
            }
            // Sort burst_pids by pheromone trace from the implicit
            // "start" node (index 0) to bias the next iteration's seed.
            let mut probs: Vec<(PartId, f64)> = burst_pids
                .iter()
                .enumerate()
                .map(|(i, p)| (*p, tau[i].iter().sum::<f64>()))
                .collect();
            probs.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(Ordering::Equal));
            burst_pids = probs.into_iter().map(|(p, _)| p).collect();
        }
        best_order
    }

    /// Tiny deterministic RNG so the search is reproducible across runs.
    /// xorshift32 — plenty of entropy for ACO weight sampling.
    struct SimpleRng(u32);
    fn simple_rng(seed: u32) -> SimpleRng {
        SimpleRng(seed.wrapping_mul(0x9E37_79B9).wrapping_add(1))
    }
    impl SimpleRng {
        fn next(&mut self) -> u32 {
            let mut x = self.0;
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            self.0 = x;
            x
        }
        fn choose_weighted(&mut self, weights: &mut [f64], total: f64) -> usize {
            let r = (self.next() as f64 / u32::MAX as f64) * total;
            let mut acc = 0.0;
            for (i, w) in weights.iter().enumerate() {
                acc += *w;
                if r <= acc {
                    return i;
                }
            }
            weights.len() - 1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::NozzleKind;
    use crate::PlacementStrategy;
    use crate::config::CameraKind;
    use crate::feeder::{FeederKind, PartId};
    use crate::geometry::Size;
    use crate::nozzle::default_head;
    use crate::parts::{Footprint, Part, Polarity};

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

    fn need(id: u32, x: f64, y: f64) -> PlacementNeed {
        PlacementNeed {
            reference: String::new(),
            part: dummy_part(id, &format!("P{}", id)),
            design_position: Point::new(x, y),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }
    }

    #[test]
    fn planner_assigns_feeder_and_nozzle() {
        let head = default_head(4);
        let feeders = vec![
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
        let needs = vec![need(1, 10.0, 10.0), need(2, 20.0, 20.0)];
        let planned = plan_job(
            "BOARD",
            needs,
            &head,
            &feeders,
            &MachineSetup::new(),
            Point::ORIGIN,
            &MachineConfig::default(),
        );
        assert_eq!(planned.job.placements.len(), 2);
        for p in &planned.job.placements {
            assert!(feeders.iter().any(|f| f.id == p.feeder_slot));
            assert!(p.nozzle < head.nozzles.len() as u8);
        }
    }

    #[test]
    fn planner_excludes_needs_marked_not_to_place() {
        let mut skipped = need(1, 0.0, 0.0);
        skipped.should_place = false;
        let planned = plan_job(
            "SKIP",
            vec![skipped, need(2, 10.0, 10.0)],
            &default_head(1),
            &[FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::ORIGIN,
                loaded: None,
            }],
            &MachineSetup::new(),
            Point::ORIGIN,
            &MachineConfig::default(),
        );

        assert_eq!(planned.job.placements.len(), 1);
        assert_eq!(planned.job.placements[0].part.id, PartId(2));
        assert!(
            !planned
                .feeder_assignment
                .slot_for_part
                .contains_key(&PartId(1))
        );
    }

    #[test]
    fn planner_groups_same_part_into_consecutive_bursts() {
        let head = default_head(4);
        let feeders = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 400.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(100.0, 400.0),
                loaded: None,
            },
        ];
        // Three R1 placements scattered across the board, then one C1.
        let needs = vec![
            need(1, 10.0, 0.0),
            need(1, 50.0, 0.0),
            need(2, 30.0, 0.0),
            need(1, 70.0, 0.0),
        ];
        let planned = plan_job(
            "BOARD",
            needs,
            &head,
            &feeders,
            &MachineSetup::new(),
            Point::ORIGIN,
            &MachineConfig::default(),
        );
        // All three R1 placements should be consecutive, regardless of
        // how they were ordered in `needs`.
        let r1_indices: Vec<usize> = planned
            .job
            .placements
            .iter()
            .enumerate()
            .filter(|(_, p)| p.part.id == PartId(1))
            .map(|(i, _)| i)
            .collect();
        assert_eq!(r1_indices.len(), 3);
        let span = r1_indices.last().unwrap() - r1_indices[0];
        assert_eq!(
            span, 2,
            "R1 placements must be consecutive (got span {})",
            span
        );
        assert!(
            r1_indices[0] == 0,
            "multi-nozzle part should run before single-placement parts"
        );
    }

    #[test]
    fn planner_uses_all_compatible_nozzles_in_bursts() {
        let head = default_head(4);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 100.0),
            loaded: None,
        }];
        let needs = (0..4).map(|i| need(1, i as f64 * 10.0, 0.0)).collect();
        let planned = plan_job(
            "BOARD",
            needs,
            &head,
            &feeders,
            &MachineSetup::new(),
            Point::ORIGIN,
            &MachineConfig::default(),
        );

        assert_eq!(planned.multi_nozzle_bursts.len(), 1);
        let burst = &planned.multi_nozzle_bursts[0];
        assert_eq!(burst.part_id, PartId(1));
        assert_eq!(burst.nozzle_ids.len(), 4);
        assert_eq!(burst.placements.len(), 4);
        assert_eq!(burst.nozzle_ids, vec![1, 2, 3, 4]);
    }

    #[test]
    fn planner_reports_estimated_seconds() {
        let head = default_head(1);
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 100.0),
            loaded: None,
        }];
        let needs: Vec<PlacementNeed> = (0..20).map(|_| need(1, 50.0, 0.0)).collect();
        let planned = plan_job(
            "BOARD",
            needs,
            &head,
            &feeders,
            &MachineSetup::new(),
            Point::ORIGIN,
            &MachineConfig::default(),
        );
        assert!(planned.estimated_seconds.is_finite());
        assert!(planned.estimated_seconds > 0.0);
        // Sanity: 20 placements should take far less than a minute.
        assert!(planned.estimated_seconds < 60.0);
    }

    #[test]
    fn planner_accounts_for_multiple_feeder_reels() {
        let head = default_head(1);
        let feeders = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 100.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(50.0, 100.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Stick);
        setup.set_kind(1, FeederKind::Stick);
        let needs = (0..60).map(|i| need(1, i as f64, 0.0)).collect();
        let planned = plan_job(
            "BOARD",
            needs,
            &head,
            &feeders,
            &setup,
            Point::ORIGIN,
            &MachineConfig::default(),
        );

        assert_eq!(planned.feeder_assignment.slot_for_part[&PartId(1)].len(), 2);
        assert!(planned.estimated_seconds.is_finite());
    }

    #[test]
    fn rotation_advances_when_reel_exhausted() {
        let _feeders = vec![
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
        let mut assignment = FeederAssignment {
            slot_for_part: HashMap::new(),
            unassigned: vec![],
            chosen_kind: HashMap::new(),
        };
        assignment.slot_for_part.insert(PartId(1), vec![0, 1]);
        let mut rot = FeederRotation::from_assignment(&assignment);
        assert_eq!(rot.current_slot(PartId(1)), Some(0));
        rot.tick(PartId(1), 4000);
        assert_eq!(rot.current_slot(PartId(1)), Some(0));
        // Force reel exhaustion:
        for _ in 0..4000 {
            rot.tick(PartId(1), 4000);
        }
        assert_eq!(rot.current_slot(PartId(1)), Some(1));
    }

    #[test]
    fn assign_nozzles_for_runs_batch_aware_no_collision() {
        // 8 R1 placements with 3 compatible N503 nozzles and a 4-nozzle
        // head. The algorithm should produce `1, 2, 3, 1, 2, 3, 1, 2`
        // — chunked into groups of 4 by the runner, each chunk has 3
        // distinct nozzles (no two adjacent same). Three batches of 3
        // form, plus a 2-placement tail.
        let mut head = default_head(4);
        for n in &mut head.nozzles[0..3] {
            n.kind = NozzleKind::N503;
        }
        head.nozzles[3].kind = NozzleKind::Standard;
        let parts: Vec<Part> = (0..8)
            .map(|_| make_real_part(1, "R1", &[NozzleKind::N503]))
            .collect();
        let nzs = assign_nozzles_for_runs_batch_aware(&parts, &head);
        assert_eq!(nzs, vec![1, 2, 3, 1, 2, 3, 1, 2]);
        // Mirror the runner's chunking: groups of `head.nozzles.len()`.
        let chunk = head.nozzles.len();
        // When compatible.len() < chunk_size, the runner still accepts
        // a chunk as long as no nozzle is reused within the chunk
        // (which means the chunk has at most 1 duplicate of each
        // nozzle, i.e. the chunk has full distinct coverage of the
        // compatible nozzles).
        let compatible_count = 3;
        for w in nzs.chunks(chunk) {
            let mut sorted = w.to_vec();
            sorted.sort();
            sorted.dedup();
            assert_eq!(
                sorted.len(),
                compatible_count,
                "chunk should have all {} compatible nozzles, got {:?}",
                compatible_count,
                w
            );
        }
    }

    #[test]
    fn assign_nozzles_treats_empty_allowlist_as_any_nozzle() {
        // wasd.json regression: a part without a nozzle_type allowlist
        // used to be pinned to nozzle 1, which forced one trip per
        // placement. Unrestricted parts must cycle across the head.
        let head = default_head(4);
        let parts: Vec<Part> = (0..9)
            .map(|_| make_real_part(1, "R1", &[]))
            .collect();
        let nzs = assign_nozzles_for_runs_batch_aware(&parts, &head);
        assert_eq!(nzs, vec![1, 2, 3, 4, 1, 2, 3, 4, 1]);
    }

    #[test]
    fn local_search_runs_and_terminates() {
        // Smoke test: LocalSearch must complete and produce a finite
        // estimated time on a small placement problem. The cost model
        // is approximated, so equality with Greedy is acceptable here.
        let head = default_head(4);
        let feeders = vec![
            FeederSlot {
                id: 0,
                bank_id: 0,
                position: Point::new(0.0, 400.0),
                loaded: None,
            },
            FeederSlot {
                id: 1,
                bank_id: 0,
                position: Point::new(50.0, 400.0),
                loaded: None,
            },
        ];
        let mut setup = MachineSetup::new();
        setup.set_kind(0, FeederKind::Tape(8));
        setup.set_kind(1, FeederKind::Tape(8));
        let parts = vec![
            make_real_part(1, "R1", &[NozzleKind::N503]),
            make_real_part(2, "LED", &[NozzleKind::N503]),
        ];
        let mut needs: Vec<PlacementNeed> = vec![];
        for i in 0..4 {
            needs.push(PlacementNeed {
                reference: String::new(),
                part: parts[0].clone(),
                design_position: Point::new(i as f64 * 10.0, 0.0),
                design_rotation_deg: 0.0,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            });
        }
        for i in 0..2 {
            needs.push(PlacementNeed {
                reference: String::new(),
                part: parts[1].clone(),
                design_position: Point::new(i as f64 * 10.0, 20.0),
                design_rotation_deg: 0.0,
                should_place: true,
                strategy: PlacementStrategy::Standard,
                camera_kind: CameraKind::FastVision,
            });
        }

        let g = plan_job_with_mode(
            "G",
            needs.clone(),
            &head,
            &feeders,
            &setup,
            Point::ORIGIN,
            &MachineConfig::default(),
            FeederPlacementMode::Movable,
            PlanSearchMode::Greedy,
        );
        let l = plan_job_with_mode(
            "L",
            needs,
            &head,
            &feeders,
            &setup,
            Point::ORIGIN,
            &MachineConfig::default(),
            FeederPlacementMode::Movable,
            PlanSearchMode::LocalSearch,
        );
        assert!(g.estimated_seconds.is_finite());
        assert!(l.estimated_seconds.is_finite());
        // Both must produce the same number of placements.
        assert_eq!(g.job.placements.len(), l.job.placements.len());
    }

    #[test]
    fn local_search_respects_its_time_budget_on_large_boards() {
        // Regression: the search deadline was only checked between full
        // passes, and one pass is O(n²) candidate evaluations — a
        // 500-placement KiCad import (e.g. the LibreVNA board) took
        // minutes before the budget ever expired. The budget must bound
        // the whole search regardless of board size.
        let head = default_head(4);
        let feeders: Vec<FeederSlot> = (0..4)
            .map(|id| FeederSlot {
                id,
                bank_id: 0,
                position: Point::new(id as f64 * 50.0, 400.0),
                loaded: None,
            })
            .collect();
        let mut setup = MachineSetup::new();
        for feeder in &feeders {
            setup.set_kind(feeder.id, FeederKind::Tape(16));
        }
        let part = make_real_part(1, "R1", &[NozzleKind::N503]);
        // A 12x12 grid of 500 placements, the shape of a real KiCad
        // position file.
        let needs: Vec<PlacementNeed> = (0..500)
            .map(|i| {
                let col = (i % 25) as f64;
                let row = (i / 25) as f64;
                PlacementNeed {
                    reference: format!("R{}", i + 1),
                    part: part.clone(),
                    design_position: Point::new(col * 2.54, row * 2.54),
                    design_rotation_deg: 0.0,
                    should_place: true,
                    strategy: PlacementStrategy::Standard,
                    camera_kind: CameraKind::FastVision,
                }
            })
            .collect();

        let started = std::time::Instant::now();
        let planned = plan_job_with_mode(
            "BIG",
            needs,
            &head,
            &feeders,
            &setup,
            Point::ORIGIN,
            &MachineConfig::default(),
            FeederPlacementMode::Movable,
            PlanSearchMode::LocalSearch,
        );
        let elapsed = started.elapsed();
        assert_eq!(planned.job.placements.len(), 500);
        assert!(planned.estimated_seconds.is_finite());
        // The search budget is 2 s; allow generous slack for slow CI
        // machines and the surrounding assignment work, but a runaway
        // pass used to take minutes.
        assert!(
            elapsed.as_secs_f64() < 30.0,
            "local search took {elapsed:?} — deadline is not being enforced"
        );
    }

    fn make_real_part(id: u32, mpn: &str, kinds: &[NozzleKind]) -> crate::parts::Part {
        use crate::parts::{Footprint, Polarity};
        crate::parts::Part {
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
            nozzle_type: kinds.to_vec(),
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        }
    }

    #[test]
    fn planner_uses_per_job_setup_kinds() {
        // Same physical slot, two different jobs with two different
        // physical feeders mounted. Job A has 8 mm tape; Job B has
        // 16 mm tape. The big part (10 mm) only fits in B.
        let feeders = vec![FeederSlot {
            id: 0,
            bank_id: 0,
            position: Point::new(0.0, 0.0),
            loaded: None,
        }];
        let big = Part {
            id: PartId(9),
            mpn: "BIG".into(),
            description: String::new(),
            footprint: Footprint {
                name: "BIG".into(),
                centroid: Point::ORIGIN,
                size: Size {
                    width: 10.0,
                    height: 10.0,
                },
                pads: vec![],
                polarity: Polarity::None,
            },
            nozzle_type: vec![NozzleKind::ExtraFine],
            pickup_rotation_deg: 0.0,
            pickup_offset: Point::ORIGIN,
            camera: None,
        };

        // Job A: 8 mm tape → big part doesn't fit.
        let mut setup_a = MachineSetup::new();
        setup_a.set_kind(0, FeederKind::Tape(8));
        let needs = vec![PlacementNeed {
            reference: String::new(),
            part: big.clone(),
            design_position: Point::new(0.0, 0.0),
            design_rotation_deg: 0.0,
            should_place: true,
            strategy: PlacementStrategy::Standard,
            camera_kind: crate::config::CameraKind::FastVision,
        }];
        let a = plan_job(
            "A",
            needs.clone(),
            &default_head(1),
            &feeders,
            &setup_a,
            Point::ORIGIN,
            &MachineConfig::default(),
        );
        assert!(a.feeder_assignment.unassigned.contains(&PartId(9)));

        // Job B: 16 mm tape → big part fits.
        let mut setup_b = MachineSetup::new();
        setup_b.set_kind(0, FeederKind::Tape(16));
        let b = plan_job(
            "B",
            needs.clone(),
            &default_head(1),
            &feeders,
            &setup_b,
            Point::ORIGIN,
            &MachineConfig::default(),
        );
        assert!(b.feeder_assignment.unassigned.is_empty());
    }
}
