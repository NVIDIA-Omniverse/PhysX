use std::collections::{HashMap, HashSet};

use rapier3d::prelude::RigidBodyHandle;

use crate::types::SplitChild;

/// Body state before a split.
pub struct ExistingBodyState {
    pub handle: RigidBodyHandle,
    pub node_indices: HashSet<u32>,
    pub is_fixed: bool,
}

/// Plan for how to handle split children: reuse existing bodies or create new ones.
pub struct SplitMigrationPlan {
    /// Children that can reuse an existing body (same node set).
    pub reuse: Vec<ReuseEntry>,
    /// Children that need a new body created.
    pub create: Vec<CreateEntry>,
}

pub struct ReuseEntry {
    pub child_index: usize,
    pub body_handle: RigidBodyHandle,
}

pub struct CreateEntry {
    pub child_index: usize,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct PlannerChildSupport {
    pub is_support: bool,
}

/// Plan body reuse vs. creation for split children.
///
/// Uses hash-based matching: if a child's node set exactly matches an existing body,
/// reuse that body. Otherwise, create a new one. Largest children are prioritized.
pub fn plan_split_migration(
    bodies: &[ExistingBodyState],
    children: &[SplitChild],
) -> SplitMigrationPlan {
    plan_split_migration_with_support(
        bodies,
        children,
        &vec![PlannerChildSupport::default(); children.len()],
    )
}

pub fn plan_split_migration_with_support(
    bodies: &[ExistingBodyState],
    children: &[SplitChild],
    child_support: &[PlannerChildSupport],
) -> SplitMigrationPlan {
    if bodies.is_empty() || children.is_empty() {
        return SplitMigrationPlan {
            reuse: Vec::new(),
            create: (0..children.len())
                .map(|i| CreateEntry { child_index: i })
                .collect(),
        };
    }

    // Build hash index of existing bodies
    let mut body_hashes: HashMap<u64, Vec<usize>> = HashMap::new();
    for (i, body) in bodies.iter().enumerate() {
        let hash = hash_node_set(&body.node_indices);
        body_hashes.entry(hash).or_default().push(i);
    }

    let mut reuse = Vec::new();
    let mut assigned_bodies = HashSet::new();
    let mut unmatched_children = Vec::new();

    for (ci, child) in children.iter().enumerate() {
        let child_set: HashSet<u32> = child.nodes.iter().copied().collect();
        let hash = hash_node_set(&child_set);

        let mut matched = false;
        if let Some(candidates) = body_hashes.get(&hash) {
            for &bi in candidates {
                if assigned_bodies.contains(&bi) {
                    continue;
                }
                if bodies[bi].is_fixed
                    && !child_support
                        .get(ci)
                        .copied()
                        .unwrap_or_default()
                        .is_support
                {
                    continue;
                }
                if bodies[bi].node_indices == child_set {
                    reuse.push(ReuseEntry {
                        child_index: ci,
                        body_handle: bodies[bi].handle,
                    });
                    assigned_bodies.insert(bi);
                    matched = true;
                    break;
                }
            }
        }
        if !matched {
            unmatched_children.push(ci);
        }
    }

    let unmatched_bodies: Vec<usize> = bodies
        .iter()
        .enumerate()
        .filter_map(|(idx, _)| (!assigned_bodies.contains(&idx)).then_some(idx))
        .collect();

    if !unmatched_bodies.is_empty() && !unmatched_children.is_empty() {
        let overlap = build_overlap_matrix(
            bodies,
            &unmatched_bodies,
            children,
            &unmatched_children,
            child_support,
        );
        let assignments = hungarian_max(&overlap);
        let mut reused_children = HashSet::new();

        for (row_idx, assignment) in assignments.into_iter().enumerate() {
            let Some(col_idx) = assignment else {
                continue;
            };
            let overlap_score = overlap
                .get(row_idx)
                .and_then(|row| row.get(col_idx))
                .copied()
                .unwrap_or(0);
            if overlap_score == 0 {
                continue;
            }

            let body_idx = unmatched_bodies[row_idx];
            let child_index = unmatched_children[col_idx];
            reuse.push(ReuseEntry {
                child_index,
                body_handle: bodies[body_idx].handle,
            });
            reused_children.insert(child_index);
        }

        let create = unmatched_children
            .into_iter()
            .filter(|child_index| !reused_children.contains(child_index))
            .map(|child_index| CreateEntry { child_index })
            .collect();
        return SplitMigrationPlan { reuse, create };
    }

    let create = unmatched_children
        .into_iter()
        .map(|child_index| CreateEntry { child_index })
        .collect();
    SplitMigrationPlan { reuse, create }
}

fn hash_node_set(nodes: &HashSet<u32>) -> u64 {
    let mut sorted: Vec<u32> = nodes.iter().copied().collect();
    sorted.sort_unstable();
    let mut hash = 0u64;
    for n in sorted {
        // Simple FNV-1a-like hash
        hash ^= n as u64;
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

fn build_overlap_matrix(
    bodies: &[ExistingBodyState],
    unmatched_bodies: &[usize],
    children: &[SplitChild],
    unmatched_children: &[usize],
    child_support: &[PlannerChildSupport],
) -> Vec<Vec<usize>> {
    unmatched_bodies
        .iter()
        .map(|&body_idx| {
            let body = &bodies[body_idx];
            unmatched_children
                .iter()
                .map(|&child_idx| {
                    if body.is_fixed
                        && !child_support
                            .get(child_idx)
                            .copied()
                            .unwrap_or_default()
                            .is_support
                    {
                        return 0;
                    }

                    children[child_idx]
                        .nodes
                        .iter()
                        .filter(|node| body.node_indices.contains(node))
                        .count()
                })
                .collect()
        })
        .collect()
}

fn hungarian_max(matrix: &[Vec<usize>]) -> Vec<Option<usize>> {
    let rows = matrix.len();
    let cols = matrix.first().map(Vec::len).unwrap_or(0);
    if rows == 0 || cols == 0 {
        return vec![None; rows];
    }

    let size = rows.max(cols);
    let max_val = matrix
        .iter()
        .flat_map(|row| row.iter())
        .copied()
        .max()
        .unwrap_or(0) as i64;
    let mut cost = vec![vec![max_val; size]; size];
    for (i, row) in matrix.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            cost[i][j] = max_val - (*value as i64);
        }
    }

    let assignments = hungarian(&cost);
    assignments
        .into_iter()
        .take(rows)
        .map(|col| (col < cols).then_some(col))
        .collect()
}

fn hungarian(cost: &[Vec<i64>]) -> Vec<usize> {
    let size = cost.len();
    let mut u = vec![0i64; size + 1];
    let mut v = vec![0i64; size + 1];
    let mut p = vec![0usize; size + 1];
    let mut way = vec![0usize; size + 1];

    for i in 1..=size {
        p[0] = i;
        let mut minv = vec![i64::MAX; size + 1];
        let mut used = vec![false; size + 1];
        let mut j0 = 0usize;
        loop {
            used[j0] = true;
            let i0 = p[j0];
            let mut delta = i64::MAX;
            let mut j1 = 0usize;
            for j in 1..=size {
                if used[j] {
                    continue;
                }
                let cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if cur < minv[j] {
                    minv[j] = cur;
                    way[j] = j0;
                }
                if minv[j] < delta {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for j in 0..=size {
                if used[j] {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else if minv[j] != i64::MAX {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
            if p[j0] == 0 {
                break;
            }
        }
        loop {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
            if j0 == 0 {
                break;
            }
        }
    }

    let mut result = vec![usize::MAX; size];
    for j in 1..=size {
        if p[j] > 0 {
            result[p[j] - 1] = j - 1;
        }
    }
    result
}
