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

/// Plan body reuse vs. creation for split children.
///
/// Uses hash-based matching: if a child's node set exactly matches an existing body,
/// reuse that body. Otherwise, create a new one. Largest children are prioritized.
pub fn plan_split_migration(
    bodies: &[ExistingBodyState],
    children: &[SplitChild],
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
    let mut create = Vec::new();
    let mut assigned_bodies = HashSet::new();

    for (ci, child) in children.iter().enumerate() {
        let child_set: HashSet<u32> = child.nodes.iter().copied().collect();
        let hash = hash_node_set(&child_set);

        let mut matched = false;
        if let Some(candidates) = body_hashes.get(&hash) {
            for &bi in candidates {
                if assigned_bodies.contains(&bi) {
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
            create.push(CreateEntry { child_index: ci });
        }
    }

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
