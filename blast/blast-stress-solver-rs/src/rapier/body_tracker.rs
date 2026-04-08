use std::collections::{HashMap, HashSet};

use rapier3d::prelude::*;

use crate::types::*;

/// Tracks the mapping between stress graph nodes and Rapier rigid bodies.
pub struct BodyTracker {
    /// For each node index, which body it belongs to (if any).
    node_to_body: Vec<Option<RigidBodyHandle>>,
    /// For each body handle, which node indices it contains.
    body_to_nodes: HashMap<RigidBodyHandle, Vec<u32>>,
    /// Set of support (fixed, mass=0) node indices.
    support_nodes: HashSet<u32>,
    /// Per-node size used for creating colliders.
    node_sizes: Vec<Vec3>,
    /// Per-node centroid.
    node_centroids: Vec<Vec3>,
}

impl BodyTracker {
    /// Create a new tracker from a scenario description.
    pub fn new(nodes: &[ScenarioNode], node_sizes: Vec<Vec3>) -> Self {
        let support_nodes: HashSet<u32> = nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.mass == 0.0)
            .map(|(i, _)| i as u32)
            .collect();

        let node_centroids = nodes.iter().map(|n| n.centroid).collect();

        Self {
            node_to_body: vec![None; nodes.len()],
            body_to_nodes: HashMap::new(),
            support_nodes,
            node_sizes,
            node_centroids,
        }
    }

    /// Create initial Rapier bodies from the actor table.
    ///
    /// Each actor becomes one rigid body. Support-only actors become fixed bodies.
    /// Returns the list of created body handles.
    pub fn create_initial_bodies(
        &mut self,
        actors: &[Actor],
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> Vec<RigidBodyHandle> {
        let mut handles = Vec::new();

        for actor in actors {
            if actor.nodes.is_empty() {
                continue;
            }

            let all_support = actor
                .nodes
                .iter()
                .all(|n| self.support_nodes.contains(n));

            // Compute center of mass for the actor
            let com = self.compute_actor_com(&actor.nodes);

            let body = if all_support {
                bodies.insert(RigidBodyBuilder::fixed().translation(vector![com.x, com.y, com.z]))
            } else {
                bodies.insert(
                    RigidBodyBuilder::dynamic().translation(vector![com.x, com.y, com.z]),
                )
            };

            // Create colliders for each node
            for &node_idx in &actor.nodes {
                let ni = node_idx as usize;
                let size = self.node_sizes[ni];
                let centroid = self.node_centroids[ni];

                // Collider position is relative to the body
                let local_x = centroid.x - com.x;
                let local_y = centroid.y - com.y;
                let local_z = centroid.z - com.z;

                let half = vector![size.x * 0.5, size.y * 0.5, size.z * 0.5];
                let collider = ColliderBuilder::cuboid(half.x, half.y, half.z)
                    .translation(vector![local_x, local_y, local_z])
                    .density(1.0);

                colliders.insert_with_parent(collider, body, bodies);
                self.node_to_body[ni] = Some(body);
            }

            self.body_to_nodes
                .insert(body, actor.nodes.clone());
            handles.push(body);
        }

        handles
    }

    /// Handle a split event: create new bodies for child actors.
    ///
    /// Returns newly created body handles.
    pub fn handle_split(
        &mut self,
        event: &SplitEvent,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        mut policy_filter: impl FnMut(u32) -> bool,
    ) -> Vec<RigidBodyHandle> {
        let mut new_handles = Vec::new();

        // Find and remove the parent body
        // The parent actor's nodes may be spread across the children
        let parent_nodes: HashSet<u32> = event
            .children
            .iter()
            .flat_map(|c| c.nodes.iter().copied())
            .collect();

        // Find which bodies owned these nodes (could be one parent body)
        let parent_bodies: HashSet<RigidBodyHandle> = parent_nodes
            .iter()
            .filter_map(|n| self.node_to_body[*n as usize])
            .collect();

        // Remove parent body entries from tracking (but keep the bodies alive for reuse)
        for &bh in &parent_bodies {
            self.body_to_nodes.remove(&bh);
        }

        // Use split migrator to plan reuse
        let existing: Vec<super::split_migrator::ExistingBodyState> = parent_bodies
            .iter()
            .map(|&h| super::split_migrator::ExistingBodyState {
                handle: h,
                node_indices: self
                    .node_to_body
                    .iter()
                    .enumerate()
                    .filter(|(_, bh)| **bh == Some(h))
                    .map(|(i, _)| i as u32)
                    .collect(),
                is_fixed: bodies
                    .get(h)
                    .map(|b| !b.is_dynamic())
                    .unwrap_or(false),
            })
            .collect();

        let plan = super::split_migrator::plan_split_migration(&existing, &event.children);

        // Handle reused bodies
        for entry in &plan.reuse {
            let child = &event.children[entry.child_index];
            self.body_to_nodes
                .insert(entry.body_handle, child.nodes.clone());
            for &n in &child.nodes {
                self.node_to_body[n as usize] = Some(entry.body_handle);
            }
        }

        // Remove bodies that weren't reused
        let reused_handles: HashSet<RigidBodyHandle> =
            plan.reuse.iter().map(|r| r.body_handle).collect();
        for &bh in &parent_bodies {
            if !reused_handles.contains(&bh) {
                bodies.remove(bh, &mut IslandManager::new(), colliders, &mut ImpulseJointSet::new(), &mut MultibodyJointSet::new(), true);
            }
        }

        // Create new bodies for unmatched children
        for entry in &plan.create {
            let child = &event.children[entry.child_index];

            if !policy_filter(child.nodes.len() as u32) {
                continue;
            }

            let all_support = child
                .nodes
                .iter()
                .all(|n| self.support_nodes.contains(n));

            let com = self.compute_actor_com(&child.nodes);

            let body_handle = if all_support {
                bodies.insert(
                    RigidBodyBuilder::fixed().translation(vector![com.x, com.y, com.z]),
                )
            } else {
                bodies.insert(
                    RigidBodyBuilder::dynamic().translation(vector![com.x, com.y, com.z]),
                )
            };

            for &node_idx in &child.nodes {
                let ni = node_idx as usize;
                let size = self.node_sizes[ni];
                let centroid = self.node_centroids[ni];

                let local_x = centroid.x - com.x;
                let local_y = centroid.y - com.y;
                let local_z = centroid.z - com.z;

                let half = vector![size.x * 0.5, size.y * 0.5, size.z * 0.5];
                let collider = ColliderBuilder::cuboid(half.x, half.y, half.z)
                    .translation(vector![local_x, local_y, local_z])
                    .density(1.0);

                colliders.insert_with_parent(collider, body_handle, bodies);
                self.node_to_body[ni] = Some(body_handle);
            }

            self.body_to_nodes
                .insert(body_handle, child.nodes.clone());
            new_handles.push(body_handle);
        }

        new_handles
    }

    /// Get the body handle for a node.
    pub fn node_body(&self, node_index: u32) -> Option<RigidBodyHandle> {
        self.node_to_body.get(node_index as usize).copied().flatten()
    }

    /// Whether a node is a support (fixed) node.
    pub fn is_support(&self, node_index: u32) -> bool {
        self.support_nodes.contains(&node_index)
    }

    /// Number of tracked bodies.
    pub fn body_count(&self) -> usize {
        self.body_to_nodes.len()
    }

    /// Number of dynamic bodies.
    pub fn dynamic_body_count(&self, bodies: &RigidBodySet) -> usize {
        self.body_to_nodes
            .keys()
            .filter(|h| bodies.get(**h).map(|b| b.is_dynamic()).unwrap_or(false))
            .count()
    }

    fn compute_actor_com(&self, nodes: &[u32]) -> Vec3 {
        if nodes.is_empty() {
            return Vec3::ZERO;
        }
        let mut sum = Vec3::ZERO;
        for &n in nodes {
            sum += self.node_centroids[n as usize];
        }
        sum / nodes.len() as f32
    }
}
