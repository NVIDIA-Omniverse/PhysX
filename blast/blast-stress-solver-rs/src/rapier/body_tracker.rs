use std::collections::{HashMap, HashSet};

use rapier3d::prelude::*;

use super::collision_groups::{apply_collision_groups_for_body, DebrisCollisionMode};
use super::optimization::{
    DebrisCleanupOptions, OptimizationMode, OptimizationResult, SleepThresholdOptions,
    SmallBodyDampingOptions,
};
use crate::types::*;

#[derive(Clone, Copy)]
struct BodyState {
    position: Isometry<Real>,
    linvel: Vector<Real>,
    angvel: Vector<Real>,
    linear_damping: Real,
    angular_damping: Real,
}

#[derive(Clone, Copy, Debug)]
struct BodyMetadata {
    created_at: f32,
    first_support_contact_at: Option<f32>,
    damping_promoted: bool,
    sleep_threshold_configured: bool,
    has_support: bool,
}

#[derive(Clone, Copy)]
struct NodeSourceState {
    body_handle: RigidBodyHandle,
    collider_handle: Option<ColliderHandle>,
    world_centroid: Vec3,
}

struct SplitPlanningData {
    existing: Vec<super::split_migrator::ExistingBodyState>,
    parent_bodies: Vec<RigidBodyHandle>,
    parent_states: HashMap<RigidBodyHandle, BodyState>,
    node_sources: HashMap<u32, NodeSourceState>,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SplitCost {
    pub create_bodies: usize,
    pub collider_migrations: usize,
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
    /// Per-body lifecycle metadata used for replay stabilization.
    body_metadata: HashMap<RigidBodyHandle, BodyMetadata>,
    /// Whether newly created dynamic bodies should enable CCD.
    dynamic_body_ccd_enabled: bool,
    /// Collision filtering mode for small debris bodies.
    debris_collision_mode: DebrisCollisionMode,
    /// Optional ground body used by debris-ground-only filtering.
    ground_body_handle: Option<RigidBodyHandle>,
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
            body_metadata: HashMap::new(),
            dynamic_body_ccd_enabled: false,
            debris_collision_mode: DebrisCollisionMode::All,
            ground_body_handle: None,
        }
    }

    pub fn set_dynamic_body_ccd_enabled(&mut self, enabled: bool) {
        self.dynamic_body_ccd_enabled = enabled;
    }

    pub fn debris_collision_mode(&self) -> DebrisCollisionMode {
        self.debris_collision_mode
    }

    pub fn set_debris_collision_mode(&mut self, mode: DebrisCollisionMode) {
        self.debris_collision_mode = mode;
    }

    pub fn ground_body_handle(&self) -> Option<RigidBodyHandle> {
        self.ground_body_handle
    }

    pub fn set_ground_body_handle(&mut self, handle: Option<RigidBodyHandle>) {
        self.ground_body_handle = handle;
    }

    pub fn refresh_collision_groups(
        &self,
        bodies: &RigidBodySet,
        colliders: &mut ColliderSet,
        max_colliders_for_debris: usize,
    ) {
        if let Some(ground_body_handle) = self.ground_body_handle {
            apply_collision_groups_for_body(
                ground_body_handle,
                bodies,
                colliders,
                self.ground_body_handle,
                self.debris_collision_mode,
                true,
                max_colliders_for_debris,
            );
        }

        for &body_handle in self.body_to_nodes.keys() {
            apply_collision_groups_for_body(
                body_handle,
                bodies,
                colliders,
                self.ground_body_handle,
                self.debris_collision_mode,
                self.debris_collision_active(body_handle),
                max_colliders_for_debris,
            );
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
        created_at: f32,
        small_body_damping: SmallBodyDampingOptions,
        sleep_thresholds: SleepThresholdOptions,
        max_colliders_for_debris: usize,
    ) -> Vec<RigidBodyHandle> {
        let mut handles = Vec::new();

        for actor in actors {
            if actor.nodes.is_empty() {
                continue;
            }

            let has_support = actor.nodes.iter().any(|n| self.support_nodes.contains(n));

            // Compute center of mass for the actor
            let com = self.compute_actor_com(&actor.nodes);

            let body = if has_support {
                bodies.insert(RigidBodyBuilder::fixed().translation(vector![com.x, com.y, com.z]))
            } else {
                bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(vector![com.x, com.y, com.z])
                        .ccd_enabled(self.dynamic_body_ccd_enabled),
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
                    .active_events(
                        ActiveEvents::CONTACT_FORCE_EVENTS | ActiveEvents::COLLISION_EVENTS,
                    )
                    .contact_force_event_threshold(0.0)
                    .friction(0.25)
                    .restitution(0.0);

                let collider = if self.support_nodes.contains(&node_idx) {
                    collider.mass(0.0)
                } else {
                    collider.mass(self.node_masses[ni].max(0.0))
                };

                let collider_handle = colliders.insert_with_parent(collider, body, bodies);
                self.attach_node(
                    node_idx,
                    body,
                    collider_handle,
                    Vec3::new(local_x, local_y, local_z),
                );
            }

            self.body_to_nodes.insert(body, actor.nodes.clone());
            self.update_body_metadata(body, &actor.nodes, created_at);
            self.promote_small_body_damping(body, bodies, &small_body_damping);
            self.configure_sleep_thresholds(body, bodies, &sleep_thresholds);
            self.apply_collision_groups(body, bodies, colliders, max_colliders_for_debris);
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
        created_at: f32,
        small_body_damping: SmallBodyDampingOptions,
        sleep_thresholds: SleepThresholdOptions,
        max_colliders_for_debris: usize,
    ) -> Vec<RigidBodyHandle> {
        let mut new_handles = Vec::new();
        let planning = self.collect_split_planning_data(event, bodies);
        let child_support: Vec<super::split_migrator::PlannerChildSupport> = event
            .children
            .iter()
            .map(|child| super::split_migrator::PlannerChildSupport {
                is_support: child.nodes.iter().any(|n| self.support_nodes.contains(n)),
            })
            .collect();
        let plan = super::split_migrator::plan_split_migration_with_support(
            &planning.existing,
            &event.children,
            &child_support,
        );
        let reused_by_body: HashMap<RigidBodyHandle, usize> = plan
            .reuse
            .iter()
            .map(|entry| (entry.body_handle, entry.child_index))
            .collect();

        // First detach nodes that no longer belong to reused bodies while the
        // original body/collider mappings are still intact.
        for entry in &plan.reuse {
            let child = &event.children[entry.child_index];
            let previous_nodes = self
                .body_to_nodes
                .get(&entry.body_handle)
                .cloned()
                .unwrap_or_default();
            let retained: HashSet<u32> = child.nodes.iter().copied().collect();

            for node in previous_nodes {
                if retained.contains(&node) {
                    continue;
                }
                self.remove_node_collider_from_body(node, bodies, colliders, island_manager);
                self.detach_node(node);
            }
        }

        // Remove bodies that aren't reused. Nodes that survive this split will
        // be reattached below using the saved source state.
        let reused_handles: HashSet<RigidBodyHandle> = reused_by_body.keys().copied().collect();
        for &bh in &planning.parent_bodies {
            if !reused_handles.contains(&bh) {
                if let Some(nodes) = self.body_to_nodes.remove(&bh) {
                    for node in nodes {
                        if self.node_to_body[node as usize] == Some(bh) {
                            self.detach_node(node);
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
                self.body_metadata.remove(&bh);
            }
        }

        // Reattach adopted nodes to reused bodies and refresh metadata.
        for entry in &plan.reuse {
            let child = &event.children[entry.child_index];
            let target_body = match bodies.get(entry.body_handle) {
                Some(body) => body,
                None => continue,
            };
            let target_pose = *target_body.position();

            for &n in &child.nodes {
                let already_on_target = self.node_body(n) == Some(entry.body_handle);
                if already_on_target {
                    continue;
                }
                let Some(source) = planning.node_sources.get(&n) else {
                    continue;
                };
                self.remove_stale_collider_mapping(source.collider_handle);
                let local_offset = self.local_offset_for_pose(source, &target_pose);
                let collider_handle = self.insert_node_collider(
                    n,
                    entry.body_handle,
                    local_offset,
                    bodies,
                    colliders,
                );
                self.attach_node(n, entry.body_handle, collider_handle, local_offset);
            }

            self.body_to_nodes
                .insert(entry.body_handle, child.nodes.clone());
            self.update_body_metadata(entry.body_handle, &child.nodes, created_at);
            self.promote_small_body_damping(entry.body_handle, bodies, &small_body_damping);
            self.configure_sleep_thresholds(entry.body_handle, bodies, &sleep_thresholds);
            self.apply_collision_groups(
                entry.body_handle,
                bodies,
                colliders,
                max_colliders_for_debris,
            );
        }

        // Create new bodies for unmatched children
        for entry in &plan.create {
            let child = &event.children[entry.child_index];

            let has_support = child.nodes.iter().any(|n| self.support_nodes.contains(n));

            let source_body = child
                .nodes
                .first()
                .and_then(|node| planning.node_sources.get(node))
                .and_then(|source| planning.parent_states.get(&source.body_handle).copied());
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
                        .ccd_enabled(self.dynamic_body_ccd_enabled)
                };
                bodies.insert(builder)
            } else if has_support {
                let com = self.compute_actor_com(&child.nodes);
                bodies.insert(RigidBodyBuilder::fixed().translation(vector![com.x, com.y, com.z]))
            } else {
                let com = self.compute_actor_com(&child.nodes);
                bodies.insert(
                    RigidBodyBuilder::dynamic()
                        .translation(vector![com.x, com.y, com.z])
                        .ccd_enabled(self.dynamic_body_ccd_enabled),
                )
            };
            let body_pose = bodies
                .get(body_handle)
                .map(|body| *body.position())
                .unwrap_or_else(Isometry::identity);

            for &node_idx in &child.nodes {
                let Some(source) = planning.node_sources.get(&node_idx) else {
                    continue;
                };
                self.remove_stale_collider_mapping(source.collider_handle);
                let local_offset = self.local_offset_for_pose(source, &body_pose);
                let collider_handle = self.insert_node_collider(
                    node_idx,
                    body_handle,
                    local_offset,
                    bodies,
                    colliders,
                );
                self.attach_node(node_idx, body_handle, collider_handle, local_offset);
            }

            self.body_to_nodes.insert(body_handle, child.nodes.clone());
            self.update_body_metadata(body_handle, &child.nodes, created_at);
            self.promote_small_body_damping(body_handle, bodies, &small_body_damping);
            self.configure_sleep_thresholds(body_handle, bodies, &sleep_thresholds);
            self.apply_collision_groups(body_handle, bodies, colliders, max_colliders_for_debris);
            new_handles.push(body_handle);
        }

        new_handles
    }

    /// Get the body handle for a node.
    pub fn node_body(&self, node_index: u32) -> Option<RigidBodyHandle> {
        self.node_to_body
            .get(node_index as usize)
            .copied()
            .flatten()
    }

    /// Get the collider handle for a node.
    pub fn node_collider(&self, node_index: u32) -> Option<ColliderHandle> {
        self.node_to_collider
            .get(node_index as usize)
            .copied()
            .flatten()
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

    pub fn body_nodes_slice(&self, body_handle: RigidBodyHandle) -> &[u32] {
        self.body_to_nodes
            .get(&body_handle)
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    pub fn body_node_count(&self, body_handle: RigidBodyHandle) -> usize {
        self.body_to_nodes
            .get(&body_handle)
            .map(Vec::len)
            .unwrap_or(0)
    }

    pub fn body_has_support(&self, body_handle: RigidBodyHandle) -> bool {
        self.body_metadata
            .get(&body_handle)
            .map(|metadata| metadata.has_support)
            .unwrap_or(false)
    }

    /// Whether a node is a support (fixed) node.
    pub fn is_support(&self, node_index: u32) -> bool {
        self.support_nodes.contains(&node_index)
    }

    /// Number of tracked bodies.
    pub fn body_count(&self) -> usize {
        self.body_to_nodes.len()
    }

    /// Number of tracked colliders owned by destructible nodes.
    pub fn collider_count(&self) -> usize {
        self.node_to_collider
            .iter()
            .filter(|handle| handle.is_some())
            .count()
    }

    /// Number of dynamic bodies.
    pub fn dynamic_body_count(&self, bodies: &RigidBodySet) -> usize {
        self.body_to_nodes
            .keys()
            .filter(|h| bodies.get(**h).map(|b| b.is_dynamic()).unwrap_or(false))
            .count()
    }

    pub fn awake_dynamic_body_count(&self, bodies: &RigidBodySet) -> usize {
        self.body_to_nodes
            .keys()
            .filter(|h| {
                bodies
                    .get(**h)
                    .map(|b| b.is_dynamic() && !b.is_sleeping())
                    .unwrap_or(false)
            })
            .count()
    }

    pub fn sleeping_dynamic_body_count(&self, bodies: &RigidBodySet) -> usize {
        self.body_to_nodes
            .keys()
            .filter(|h| {
                bodies
                    .get(**h)
                    .map(|b| b.is_dynamic() && b.is_sleeping())
                    .unwrap_or(false)
            })
            .count()
    }

    pub fn support_body_count(&self) -> usize {
        self.body_metadata
            .values()
            .filter(|metadata| metadata.has_support)
            .count()
    }

    pub fn ccd_enabled_body_count(&self, bodies: &RigidBodySet) -> usize {
        self.body_to_nodes
            .keys()
            .filter(|h| bodies.get(**h).map(|b| b.is_ccd_enabled()).unwrap_or(false))
            .count()
    }

    pub fn estimate_split_cost(&self, event: &SplitEvent, bodies: &RigidBodySet) -> SplitCost {
        let existing = self.collect_existing_body_states(event, bodies);
        let child_support: Vec<super::split_migrator::PlannerChildSupport> = event
            .children
            .iter()
            .map(|child| super::split_migrator::PlannerChildSupport {
                is_support: child.nodes.iter().any(|n| self.support_nodes.contains(n)),
            })
            .collect();
        let plan = super::split_migrator::plan_split_migration_with_support(
            &existing,
            &event.children,
            &child_support,
        );
        SplitCost {
            create_bodies: plan.create.len(),
            collider_migrations: event.children.iter().map(|child| child.nodes.len()).sum(),
        }
    }

    pub fn mark_support_contact(
        &mut self,
        body_handle: RigidBodyHandle,
        now_secs: f32,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        damping: SmallBodyDampingOptions,
        sleep_thresholds: SleepThresholdOptions,
        max_colliders_for_debris: usize,
    ) -> bool {
        let collider_count = self.body_node_count(body_handle);
        let Some(metadata) = self.body_metadata.get_mut(&body_handle) else {
            return false;
        };
        if metadata.has_support {
            return false;
        }
        metadata.first_support_contact_at.get_or_insert(now_secs);
        let mut changed = false;
        if damping.mode == OptimizationMode::AfterGroundCollision
            && !metadata.damping_promoted
            && collider_count <= damping.collider_count_threshold
        {
            if Self::apply_small_body_damping(bodies, body_handle, &damping) {
                metadata.damping_promoted = true;
                changed = true;
            }
        }
        if sleep_thresholds.mode == OptimizationMode::AfterGroundCollision
            && !metadata.sleep_threshold_configured
        {
            if Self::apply_sleep_thresholds(bodies, body_handle, &sleep_thresholds) {
                metadata.sleep_threshold_configured = true;
                changed = true;
            }
        }
        if metadata.first_support_contact_at == Some(now_secs)
            && self.debris_collision_mode != DebrisCollisionMode::All
        {
            self.apply_collision_groups(body_handle, bodies, colliders, max_colliders_for_debris);
            changed = true;
        }
        changed
    }

    pub fn cleanup_expired_bodies(
        &mut self,
        now_secs: f32,
        cleanup: DebrisCleanupOptions,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> OptimizationResult {
        let mut result = OptimizationResult::default();
        if cleanup.mode == OptimizationMode::Off {
            return result;
        }

        let mut expired = Vec::new();
        for (&body_handle, metadata) in &self.body_metadata {
            if metadata.has_support {
                continue;
            }
            let Some(body) = bodies.get(body_handle) else {
                continue;
            };
            if !body.is_dynamic() {
                continue;
            }
            if self.body_node_count(body_handle) > cleanup.max_colliders_for_debris {
                continue;
            }

            let anchor = match cleanup.mode {
                OptimizationMode::Off => None,
                OptimizationMode::Always => Some(metadata.created_at),
                OptimizationMode::AfterGroundCollision => metadata.first_support_contact_at,
            };
            let Some(anchor) = anchor else {
                continue;
            };
            if cleanup.mode == OptimizationMode::AfterGroundCollision && !body.is_sleeping() {
                continue;
            }
            if now_secs - anchor >= cleanup.debris_ttl_secs {
                expired.push(body_handle);
            }
        }

        for body_handle in expired {
            let removed_nodes = self.remove_body(
                body_handle,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
            );
            if !removed_nodes.is_empty() {
                result.removed_bodies.push(body_handle);
                result.removed_nodes.extend(removed_nodes);
            }
        }

        result
    }

    pub fn destroy_nodes(
        &mut self,
        nodes: &[u32],
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Vec<u32> {
        let mut removed = Vec::new();
        let mut affected_bodies = HashSet::new();

        for &node_index in nodes {
            let idx = node_index as usize;
            let Some(body_handle) = self.node_to_body[idx] else {
                continue;
            };
            if let Some(collider_handle) = self.node_to_collider[idx] {
                colliders.remove(collider_handle, island_manager, bodies, true);
            }
            self.detach_node(node_index);
            if let Some(body_nodes) = self.body_to_nodes.get_mut(&body_handle) {
                body_nodes.retain(|node| *node != node_index);
            }
            affected_bodies.insert(body_handle);
            removed.push(node_index);
        }

        for body_handle in affected_bodies {
            let should_remove = self
                .body_to_nodes
                .get(&body_handle)
                .map(|nodes| nodes.is_empty())
                .unwrap_or(true);
            if should_remove {
                self.body_to_nodes.remove(&body_handle);
                self.body_metadata.remove(&body_handle);
                if bodies.get(body_handle).is_some() {
                    bodies.remove(
                        body_handle,
                        island_manager,
                        colliders,
                        impulse_joints,
                        multibody_joints,
                        true,
                    );
                }
            } else if let Some(metadata) = self.body_metadata.get_mut(&body_handle) {
                metadata.has_support = self
                    .body_to_nodes
                    .get(&body_handle)
                    .map(|nodes| nodes.iter().any(|node| self.support_nodes.contains(node)))
                    .unwrap_or(false);
            }
        }

        removed
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

    fn update_body_metadata(
        &mut self,
        body_handle: RigidBodyHandle,
        nodes: &[u32],
        created_at: f32,
    ) {
        let has_support = nodes.iter().any(|node| self.support_nodes.contains(node));
        self.body_metadata.insert(
            body_handle,
            BodyMetadata {
                created_at,
                first_support_contact_at: None,
                damping_promoted: false,
                sleep_threshold_configured: false,
                has_support,
            },
        );
    }

    fn promote_small_body_damping(
        &mut self,
        body_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
        damping: &SmallBodyDampingOptions,
    ) {
        if damping.mode != OptimizationMode::Always {
            return;
        }
        if self.body_node_count(body_handle) > damping.collider_count_threshold {
            return;
        }
        if Self::apply_small_body_damping(bodies, body_handle, damping) {
            if let Some(metadata) = self.body_metadata.get_mut(&body_handle) {
                metadata.damping_promoted = true;
            }
        }
    }

    fn configure_sleep_thresholds(
        &mut self,
        body_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
        sleep_thresholds: &SleepThresholdOptions,
    ) {
        if sleep_thresholds.mode != OptimizationMode::Always {
            return;
        }
        if Self::apply_sleep_thresholds(bodies, body_handle, sleep_thresholds) {
            if let Some(metadata) = self.body_metadata.get_mut(&body_handle) {
                metadata.sleep_threshold_configured = true;
            }
        }
    }

    fn apply_small_body_damping(
        bodies: &mut RigidBodySet,
        body_handle: RigidBodyHandle,
        damping: &SmallBodyDampingOptions,
    ) -> bool {
        let Some(body) = bodies.get_mut(body_handle) else {
            return false;
        };
        if !body.is_dynamic() {
            return false;
        }
        let mut changed = false;
        if body.linear_damping() < damping.min_linear_damping {
            body.set_linear_damping(damping.min_linear_damping);
            changed = true;
        }
        if body.angular_damping() < damping.min_angular_damping {
            body.set_angular_damping(damping.min_angular_damping);
            changed = true;
        }
        if changed {
            body.wake_up(true);
        }
        changed
    }

    fn apply_sleep_thresholds(
        bodies: &mut RigidBodySet,
        body_handle: RigidBodyHandle,
        sleep_thresholds: &SleepThresholdOptions,
    ) -> bool {
        let Some(body) = bodies.get_mut(body_handle) else {
            return false;
        };
        if !body.is_dynamic() {
            return false;
        }
        let activation = body.activation_mut();
        let desired_linear = sleep_thresholds.linear_threshold.max(0.0);
        let desired_angular = sleep_thresholds.angular_threshold.max(0.0);
        let changed = (activation.normalized_linear_threshold - desired_linear).abs()
            > Real::EPSILON
            || (activation.angular_threshold - desired_angular).abs() > Real::EPSILON;
        activation.normalized_linear_threshold = desired_linear;
        activation.angular_threshold = desired_angular;
        if changed {
            body.wake_up(true);
        }
        changed
    }

    fn remove_body(
        &mut self,
        body_handle: RigidBodyHandle,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Vec<u32> {
        let nodes = self.body_to_nodes.remove(&body_handle).unwrap_or_default();
        for node in &nodes {
            self.detach_node(*node);
        }
        self.body_metadata.remove(&body_handle);
        if bodies.get(body_handle).is_some() {
            bodies.remove(
                body_handle,
                island_manager,
                colliders,
                impulse_joints,
                multibody_joints,
                true,
            );
        }
        nodes
    }

    fn collect_existing_body_states(
        &self,
        event: &SplitEvent,
        bodies: &RigidBodySet,
    ) -> Vec<super::split_migrator::ExistingBodyState> {
        let mut parent_bodies = HashSet::new();
        for child in &event.children {
            for &node in &child.nodes {
                if let Some(handle) = self.node_body(node) {
                    parent_bodies.insert(handle);
                }
            }
        }

        parent_bodies
            .into_iter()
            .map(|handle| super::split_migrator::ExistingBodyState {
                handle,
                node_indices: self
                    .body_to_nodes
                    .get(&handle)
                    .map(|nodes| nodes.iter().copied().collect())
                    .unwrap_or_default(),
                is_fixed: bodies.get(handle).map(|b| !b.is_dynamic()).unwrap_or(false),
            })
            .collect()
    }

    fn collect_split_planning_data(
        &self,
        event: &SplitEvent,
        bodies: &RigidBodySet,
    ) -> SplitPlanningData {
        let mut parent_bodies = HashSet::new();
        let mut node_sources = HashMap::new();

        for child in &event.children {
            for &node in &child.nodes {
                let idx = node as usize;
                let Some(body_handle) = self.node_to_body.get(idx).copied().flatten() else {
                    continue;
                };
                let Some(body) = bodies.get(body_handle) else {
                    continue;
                };
                let local_offset = self.node_local_offsets[idx];
                let world = body.position().transform_point(&point![
                    local_offset.x,
                    local_offset.y,
                    local_offset.z
                ]);
                parent_bodies.insert(body_handle);
                node_sources.entry(node).or_insert(NodeSourceState {
                    body_handle,
                    collider_handle: self.node_to_collider[idx],
                    world_centroid: Vec3::new(world.x, world.y, world.z),
                });
            }
        }

        let parent_bodies: Vec<RigidBodyHandle> = parent_bodies.into_iter().collect();
        let parent_states = parent_bodies
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

        SplitPlanningData {
            existing: self.collect_existing_body_states(event, bodies),
            parent_bodies,
            parent_states,
            node_sources,
        }
    }

    fn remove_node_collider_from_body(
        &mut self,
        node_index: u32,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
    ) {
        let idx = node_index as usize;
        if let Some(collider_handle) = self.node_to_collider[idx] {
            self.collider_to_node.remove(&collider_handle);
            colliders.remove(collider_handle, island_manager, bodies, true);
        }
    }

    fn remove_stale_collider_mapping(&mut self, collider_handle: Option<ColliderHandle>) {
        if let Some(handle) = collider_handle {
            self.collider_to_node.remove(&handle);
        }
    }

    fn local_offset_for_pose(
        &self,
        source: &NodeSourceState,
        target_pose: &Isometry<Real>,
    ) -> Vec3 {
        let local = target_pose.inverse_transform_point(&point![
            source.world_centroid.x,
            source.world_centroid.y,
            source.world_centroid.z
        ]);
        Vec3::new(local.x, local.y, local.z)
    }

    fn insert_node_collider(
        &self,
        node_index: u32,
        body_handle: RigidBodyHandle,
        local_offset: Vec3,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> ColliderHandle {
        let ni = node_index as usize;
        let size = self.node_sizes[ni];
        let half = vector![size.x * 0.5, size.y * 0.5, size.z * 0.5];
        let collider = ColliderBuilder::cuboid(half.x, half.y, half.z)
            .translation(vector![local_offset.x, local_offset.y, local_offset.z])
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS | ActiveEvents::COLLISION_EVENTS)
            .contact_force_event_threshold(0.0)
            .friction(0.25)
            .restitution(0.0);

        let collider = if self.support_nodes.contains(&node_index) {
            collider.mass(0.0)
        } else {
            collider.mass(self.node_masses[ni].max(0.0))
        };

        colliders.insert_with_parent(collider, body_handle, bodies)
    }

    fn apply_collision_groups(
        &self,
        body_handle: RigidBodyHandle,
        bodies: &RigidBodySet,
        colliders: &mut ColliderSet,
        max_colliders_for_debris: usize,
    ) {
        apply_collision_groups_for_body(
            body_handle,
            bodies,
            colliders,
            self.ground_body_handle,
            self.debris_collision_mode,
            self.debris_collision_active(body_handle),
            max_colliders_for_debris,
        );
    }

    fn debris_collision_active(&self, body_handle: RigidBodyHandle) -> bool {
        self.body_metadata
            .get(&body_handle)
            .and_then(|metadata| metadata.first_support_contact_at)
            .is_some()
    }
}
