use std::collections::{HashMap, HashSet};

use rapier3d::prelude::*;

use crate::types::*;

#[derive(Clone, Copy)]
struct BodyState {
    position: Isometry<Real>,
    linvel: Vector<Real>,
    angvel: Vector<Real>,
    linear_damping: Real,
    angular_damping: Real,
}

/// Tracks the mapping between stress graph nodes and Rapier rigid bodies.
pub struct BodyTracker {
    /// For each node index, which body it belongs to (if any).
    node_to_body: Vec<Option<RigidBodyHandle>>,
    /// For each node index, which collider it belongs to (if any).
    node_to_collider: Vec<Option<ColliderHandle>>,
    /// For each collider handle, which node index it belongs to.
    collider_to_node: HashMap<ColliderHandle, u32>,
    /// For each body handle, which node indices it contains.
    body_to_nodes: HashMap<RigidBodyHandle, Vec<u32>>,
    /// Set of support (fixed, mass=0) node indices.
    support_nodes: HashSet<u32>,
    /// Per-node size used for creating colliders.
    node_sizes: Vec<Vec3>,
    /// Per-node centroid.
    node_centroids: Vec<Vec3>,
    /// Per-node mass.
    node_masses: Vec<f32>,
    /// Per-node local offset relative to its owning body.
    node_local_offsets: Vec<Vec3>,
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
        let node_masses = nodes.iter().map(|n| n.mass).collect();

        Self {
            node_to_body: vec![None; nodes.len()],
            node_to_collider: vec![None; nodes.len()],
            collider_to_node: HashMap::new(),
            body_to_nodes: HashMap::new(),
            support_nodes,
            node_sizes,
            node_centroids,
            node_masses,
            node_local_offsets: vec![Vec3::ZERO; nodes.len()],
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

            let has_support = actor
                .nodes
                .iter()
                .any(|n| self.support_nodes.contains(n));

            // Compute center of mass for the actor
            let com = self.compute_actor_com(&actor.nodes);

            let body = if has_support {
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
                    .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
                    .contact_force_event_threshold(0.0)
                    .friction(0.25)
                    .restitution(0.0);

                let collider = if self.support_nodes.contains(&node_idx) {
                    collider.mass(0.0)
                } else {
                    collider.mass(self.node_masses[ni].max(0.0))
                };

                let collider_handle = colliders.insert_with_parent(collider, body, bodies);
                self.attach_node(node_idx, body, collider_handle, Vec3::new(local_x, local_y, local_z));
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
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        mut policy_filter: impl FnMut(u32) -> bool,
    ) -> Vec<RigidBodyHandle> {
        let mut new_handles = Vec::new();
        let previous_node_bodies = self.node_to_body.clone();
        let previous_local_offsets = self.node_local_offsets.clone();

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
        let parent_states: HashMap<RigidBodyHandle, BodyState> = parent_bodies
            .iter()
            .filter_map(|&handle| {
                let body = bodies.get(handle)?;
                Some((
                    handle,
                    BodyState {
                        position: *body.position(),
                        linvel: *body.linvel(),
                        angvel: *body.angvel(),
                        linear_damping: body.linear_damping(),
                        angular_damping: body.angular_damping(),
                    },
                ))
            })
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
                if let Some(nodes) = self.body_to_nodes.get(&bh).cloned() {
                    for node in nodes {
                        self.detach_node(node);
                    }
                } else {
                    for node in &parent_nodes {
                        if self.node_to_body[*node as usize] == Some(bh) {
                            self.detach_node(*node);
                        }
                    }
                }
                bodies.remove(
                    bh,
                    island_manager,
                    colliders,
                    impulse_joints,
                    multibody_joints,
                    true,
                );
            }
        }

        // Create new bodies for unmatched children
        for entry in &plan.create {
            let child = &event.children[entry.child_index];

            if !policy_filter(child.nodes.len() as u32) {
                continue;
            }

            let has_support = child
                .nodes
                .iter()
                .any(|n| self.support_nodes.contains(n));

            let source_body = child
                .nodes
                .first()
                .and_then(|node| previous_node_bodies[*node as usize])
                .and_then(|handle| parent_states.get(&handle).copied());
            let body_handle = if let Some(source) = source_body {
                let builder = if has_support {
                    RigidBodyBuilder::fixed()
                        .pose(source.position)
                        .linear_damping(source.linear_damping)
                        .angular_damping(source.angular_damping)
                } else {
                    RigidBodyBuilder::dynamic()
                        .pose(source.position)
                        .linvel(source.linvel)
                        .angvel(source.angvel)
                        .linear_damping(source.linear_damping)
                        .angular_damping(source.angular_damping)
                };
                bodies.insert(builder)
            } else if has_support {
                let com = self.compute_actor_com(&child.nodes);
                bodies.insert(
                    RigidBodyBuilder::fixed().translation(vector![com.x, com.y, com.z]),
                )
            } else {
                let com = self.compute_actor_com(&child.nodes);
                bodies.insert(
                    RigidBodyBuilder::dynamic().translation(vector![com.x, com.y, com.z]),
                )
            };

            for &node_idx in &child.nodes {
                let ni = node_idx as usize;
                let size = self.node_sizes[ni];
                let local_offset = previous_local_offsets[ni];

                let half = vector![size.x * 0.5, size.y * 0.5, size.z * 0.5];
                let collider = ColliderBuilder::cuboid(half.x, half.y, half.z)
                    .translation(vector![local_offset.x, local_offset.y, local_offset.z])
                    .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
                    .contact_force_event_threshold(0.0)
                    .friction(0.25)
                    .restitution(0.0);

                let collider = if self.support_nodes.contains(&node_idx) {
                    collider.mass(0.0)
                } else {
                    collider.mass(self.node_masses[ni].max(0.0))
                };

                let collider_handle = colliders.insert_with_parent(collider, body_handle, bodies);
                self.attach_node(
                    node_idx,
                    body_handle,
                    collider_handle,
                    local_offset,
                );
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

    /// Get the collider handle for a node.
    pub fn node_collider(&self, node_index: u32) -> Option<ColliderHandle> {
        self.node_to_collider.get(node_index as usize).copied().flatten()
    }

    /// Get the node owning a collider.
    pub fn collider_node(&self, collider: ColliderHandle) -> Option<u32> {
        self.collider_to_node.get(&collider).copied()
    }

    /// Get the node local offset relative to its owning body.
    pub fn node_local_offset(&self, node_index: u32) -> Option<Vec3> {
        let idx = node_index as usize;
        if self.node_to_body.get(idx).copied().flatten().is_some() {
            self.node_local_offsets.get(idx).copied()
        } else {
            None
        }
    }

    /// Get the node indices attached to a body.
    pub fn body_nodes(&self, body_handle: RigidBodyHandle) -> Vec<u32> {
        self.body_to_nodes
            .get(&body_handle)
            .cloned()
            .unwrap_or_default()
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

    fn attach_node(
        &mut self,
        node_index: u32,
        body: RigidBodyHandle,
        collider: ColliderHandle,
        local_offset: Vec3,
    ) {
        let idx = node_index as usize;
        self.node_to_body[idx] = Some(body);
        self.node_to_collider[idx] = Some(collider);
        self.collider_to_node.insert(collider, node_index);
        self.node_local_offsets[idx] = local_offset;
    }

    fn detach_node(&mut self, node_index: u32) {
        let idx = node_index as usize;
        if let Some(collider) = self.node_to_collider[idx].take() {
            self.collider_to_node.remove(&collider);
        }
        self.node_to_body[idx] = None;
        self.node_local_offsets[idx] = Vec3::ZERO;
    }
}
