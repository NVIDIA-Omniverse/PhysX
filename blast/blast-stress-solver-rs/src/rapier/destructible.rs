use std::collections::{HashMap, VecDeque};

use rapier3d::prelude::*;

use crate::ext_stress_solver::ExtStressSolver;
use crate::types::*;

use super::body_tracker::BodyTracker;
use super::fracture_policy::FracturePolicy;
use super::optimization::{DebrisCleanupOptions, OptimizationResult, SmallBodyDampingOptions};
use super::optimization::SleepThresholdOptions;
use super::resimulation::{BodySnapshots, ResimulationOptions};

/// Configuration for creating a `DestructibleSet`.
pub struct DestructibleConfig {
    pub nodes: Vec<NodeDesc>,
    pub bonds: Vec<BondDesc>,
    pub node_sizes: Vec<Vec3>,
    pub solver_settings: SolverSettings,
    pub gravity: Vec3,
    pub fracture_policy: FracturePolicy,
    pub resimulation: ResimulationOptions,
    pub skip_single_bodies: bool,
    pub sleep_thresholds: SleepThresholdOptions,
    pub small_body_damping: SmallBodyDampingOptions,
    pub debris_cleanup: DebrisCleanupOptions,
}

/// Result of a single step.
#[derive(Clone, Debug, Default)]
pub struct StepResult {
    /// Number of bonds that broke this frame.
    pub fractures: usize,
    /// Number of new rigid bodies created this frame.
    pub new_bodies: usize,
    /// Number of split events that occurred.
    pub split_events: usize,
    /// Whether the solver converged.
    pub converged: bool,
}

/// Main orchestrator for destructible structures with Rapier integration.
///
/// Manages the stress solver and the mapping between solver actors and Rapier bodies.
/// Call `step()` each frame to advance the simulation.
pub struct DestructibleSet {
    solver: ExtStressSolver,
    tracker: BodyTracker,
    policy: FracturePolicy,
    gravity: Vec3,
    initialized: bool,
    resimulation: ResimulationOptions,
    skip_single_bodies: bool,
    sleep_thresholds: SleepThresholdOptions,
    small_body_damping: SmallBodyDampingOptions,
    debris_cleanup: DebrisCleanupOptions,
    bond_table: Vec<BondDesc>,
    node_bonds: Vec<Vec<u32>>,
    removed_bonds: Vec<bool>,
    destroyed_nodes: Vec<bool>,
    pending_split_events: VecDeque<SplitEvent>,
    /// Track whether forces were applied this frame (for idle skip).
    forces_applied: bool,
    /// Number of frames since the last fracture (for idle skip).
    frames_since_fracture: u32,
}

impl DestructibleSet {
    /// Create a new destructible set.
    ///
    /// You must call `initialize()` after creating the Rapier world to set up the initial bodies.
    pub fn new(config: DestructibleConfig) -> Option<Self> {
        let solver = ExtStressSolver::new(&config.nodes, &config.bonds, &config.solver_settings)?;
        let node_count = config.nodes.len();
        let mut node_bonds = vec![Vec::new(); node_count];
        for (bond_index, bond) in config.bonds.iter().enumerate() {
            if let Some(list) = node_bonds.get_mut(bond.node0 as usize) {
                list.push(bond_index as u32);
            }
            if let Some(list) = node_bonds.get_mut(bond.node1 as usize) {
                list.push(bond_index as u32);
            }
        }

        let scenario_nodes: Vec<ScenarioNode> = config
            .nodes
            .iter()
            .map(|n| ScenarioNode {
                centroid: n.centroid,
                mass: n.mass,
                volume: n.volume,
            })
            .collect();

        let tracker = BodyTracker::new(&scenario_nodes, config.node_sizes);

        Some(Self {
            solver,
            tracker,
            policy: config.fracture_policy,
            gravity: config.gravity,
            initialized: false,
            resimulation: config.resimulation,
            skip_single_bodies: config.skip_single_bodies,
            sleep_thresholds: config.sleep_thresholds,
            small_body_damping: config.small_body_damping,
            debris_cleanup: config.debris_cleanup,
            bond_table: config.bonds,
            node_bonds,
            removed_bonds: vec![false; node_count],
            destroyed_nodes: vec![false; node_count],
            pending_split_events: VecDeque::new(),
            forces_applied: false,
            frames_since_fracture: u32::MAX,
        })
    }

    /// Create a destructible set from a scenario description.
    ///
    /// Node sizes are estimated from volume (cube root).
    pub fn from_scenario(
        scenario: &ScenarioDesc,
        settings: SolverSettings,
        gravity: Vec3,
        policy: FracturePolicy,
    ) -> Option<Self> {
        let (nodes, bonds) = scenario.to_solver_descs();

        let node_sizes: Vec<Vec3> = scenario
            .node_sizes
            .iter()
            .copied()
            .chain(
                scenario
                    .nodes
                    .iter()
                    .skip(scenario.node_sizes.len())
                    .map(|n| {
                        let side = n.volume.cbrt().max(0.01);
                        Vec3::new(side, side, side)
                    }),
            )
            .collect();

        Self::new(DestructibleConfig {
            nodes,
            bonds,
            node_sizes,
            solver_settings: settings,
            gravity,
            fracture_policy: policy,
            resimulation: ResimulationOptions::default(),
            skip_single_bodies: false,
            sleep_thresholds: SleepThresholdOptions::default(),
            small_body_damping: SmallBodyDampingOptions::default(),
            debris_cleanup: DebrisCleanupOptions::default(),
        })
    }

    /// Set up initial Rapier bodies from the solver's actor table.
    ///
    /// Must be called once before the first `step()`.
    pub fn initialize(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> Vec<RigidBodyHandle> {
        let actors = self.solver.actors();
        let handles = self.tracker.create_initial_bodies(
            &actors,
            bodies,
            colliders,
            0.0,
            self.small_body_damping,
            self.sleep_thresholds,
        );
        self.initialized = true;
        handles
    }

    /// Run one simulation step:
    /// 1. Apply gravity
    /// 2. Update stress solver
    /// 3. Generate and apply fractures (rate-limited by policy)
    /// 4. Create/destroy Rapier bodies for split events
    /// 5. Apply excess forces to newly separated actors
    pub fn step(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> StepResult {
        self.step_with_time(
            0.0,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
        )
    }

    pub fn step_with_time(
        &mut self,
        now_secs: f32,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> StepResult {
        assert!(self.initialized, "call initialize() before step()");

        let mut result = StepResult::default();
        let mut remaining_new_bodies = self.policy.clamp_new_bodies(usize::MAX);
        let mut remaining_collider_migrations =
            self.policy.clamp_collider_migrations(usize::MAX);

        self.process_pending_split_events(
            now_secs,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
            &mut remaining_new_bodies,
            &mut remaining_collider_migrations,
            &mut result,
        );

        // Idle skip optimization
        if self.policy.idle_skip
            && !self.forces_applied
            && self.frames_since_fracture > 2
            && self.pending_split_events.is_empty()
        {
            return result;
        }

        // 1. Apply gravity
        self.solver.add_gravity(self.gravity);
        self.forces_applied = false;

        // 2. Update solver
        self.solver.update();
        result.converged = self.solver.converged();

        // 3. Check for overstressed bonds
        let overstressed = self.solver.overstressed_bond_count();
        if overstressed == 0 {
            self.frames_since_fracture = self.frames_since_fracture.saturating_add(1);
            return result;
        }

        // Check policy: should we suppress fractures?
        let dynamic_count = self.tracker.dynamic_body_count(bodies);
        if self.policy.should_suppress(dynamic_count) {
            return result;
        }

        // 4. Generate fracture commands
        let mut commands = self.solver.generate_fracture_commands();
        if commands.is_empty() {
            return result;
        }

        // Rate-limit fractures
        let max_fractures = self.policy.max_fractures_per_frame;
        if max_fractures > 0 {
            let mut total = 0usize;
            for cmd in &mut commands {
                let remaining = (max_fractures as usize).saturating_sub(total);
                if remaining == 0 {
                    cmd.bond_fractures.clear();
                } else if cmd.bond_fractures.len() > remaining {
                    cmd.bond_fractures.truncate(remaining);
                }
                total += cmd.bond_fractures.len();
            }
            commands.retain(|c| !c.bond_fractures.is_empty());
        }

        result.fractures = commands.iter().map(|c| c.bond_fractures.len()).sum();

        self.mark_bonds_removed(&commands);

        // 5. Apply fracture commands
        let events = self.solver.apply_fracture_commands(&commands);
        result.split_events += events.len();
        self.pending_split_events.extend(events);
        self.process_pending_split_events(
            now_secs,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
            &mut remaining_new_bodies,
            &mut remaining_collider_migrations,
            &mut result,
        );

        // Optional: kick separated actors with solver-reported excess forces.
        if self.policy.apply_excess_forces {
            self.apply_excess_forces(bodies);
        }

        self.frames_since_fracture = 0;
        result
    }

    /// Apply an external force to a node (e.g., from a projectile impact).
    pub fn add_force(&mut self, node_index: u32, position: Vec3, force: Vec3) {
        self.solver
            .add_force(node_index, position, force, ForceMode::Force);
        self.forces_applied = true;
    }

    /// Apply an external acceleration to a node.
    pub fn add_acceleration(&mut self, node_index: u32, position: Vec3, acceleration: Vec3) {
        self.solver
            .add_force(node_index, position, acceleration, ForceMode::Acceleration);
        self.forces_applied = true;
    }

    /// Get the Rapier body handle for a node.
    pub fn node_body(&self, node_index: u32) -> Option<RigidBodyHandle> {
        self.tracker.node_body(node_index)
    }

    /// Get the Rapier collider handle for a node.
    pub fn node_collider(&self, node_index: u32) -> Option<ColliderHandle> {
        self.tracker.node_collider(node_index)
    }

    /// Get the node index owning a collider.
    pub fn collider_node(&self, collider: ColliderHandle) -> Option<u32> {
        self.tracker.collider_node(collider)
    }

    /// Get the node's local offset relative to its owning Rapier body.
    pub fn node_local_offset(&self, node_index: u32) -> Option<Vec3> {
        self.tracker.node_local_offset(node_index)
    }

    /// Get the node indices attached to a Rapier body.
    pub fn body_nodes(&self, body_handle: RigidBodyHandle) -> Vec<u32> {
        self.tracker.body_nodes(body_handle)
    }

    pub fn body_has_support(&self, body_handle: RigidBodyHandle) -> bool {
        self.tracker.body_has_support(body_handle)
    }

    /// Whether a node is a fixed support.
    pub fn is_support(&self, node_index: u32) -> bool {
        self.tracker.is_support(node_index)
    }

    /// Current actor count in the stress graph.
    pub fn actor_count(&self) -> u32 {
        self.solver.actor_count()
    }

    /// Number of bonds still considered active by the Rust-side fracture tracking.
    pub fn active_bond_count(&self) -> usize {
        self.removed_bonds.iter().filter(|removed| !**removed).count()
    }

    /// Number of tracked Rapier bodies.
    pub fn body_count(&self) -> usize {
        self.tracker.body_count()
    }

    /// Access the underlying solver for advanced use.
    pub fn solver(&self) -> &ExtStressSolver {
        &self.solver
    }

    /// Mutable access to the underlying solver.
    pub fn solver_mut(&mut self) -> &mut ExtStressSolver {
        &mut self.solver
    }

    /// Access the fracture policy.
    pub fn policy(&self) -> &FracturePolicy {
        &self.policy
    }

    /// Mutable access to the fracture policy.
    pub fn set_policy(&mut self, policy: FracturePolicy) {
        self.policy = policy;
    }

    pub fn resimulation_options(&self) -> ResimulationOptions {
        self.resimulation
    }

    pub fn set_resimulation_options(&mut self, options: ResimulationOptions) {
        self.resimulation = options;
    }

    pub fn skip_single_bodies(&self) -> bool {
        self.skip_single_bodies
    }

    pub fn set_skip_single_bodies(&mut self, enabled: bool) {
        self.skip_single_bodies = enabled;
    }

    pub fn sleep_thresholds(&self) -> SleepThresholdOptions {
        self.sleep_thresholds
    }

    pub fn set_sleep_thresholds(&mut self, options: SleepThresholdOptions) {
        self.sleep_thresholds = options;
    }

    pub fn small_body_damping(&self) -> SmallBodyDampingOptions {
        self.small_body_damping
    }

    pub fn set_small_body_damping(&mut self, options: SmallBodyDampingOptions) {
        self.small_body_damping = options;
    }

    pub fn debris_cleanup(&self) -> DebrisCleanupOptions {
        self.debris_cleanup
    }

    pub fn set_debris_cleanup(&mut self, options: DebrisCleanupOptions) {
        self.debris_cleanup = options;
    }

    pub fn capture_resimulation_snapshot(&self, bodies: &RigidBodySet) -> BodySnapshots {
        BodySnapshots::capture(bodies)
    }

    pub fn mark_body_support_contact(
        &mut self,
        body_handle: RigidBodyHandle,
        now_secs: f32,
        bodies: &mut RigidBodySet,
    ) -> bool {
        self.tracker.mark_support_contact(
            body_handle,
            now_secs,
            bodies,
            self.small_body_damping,
            self.sleep_thresholds,
        )
    }

    pub fn process_optimizations(
        &mut self,
        now_secs: f32,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> OptimizationResult {
        let result = self.tracker.cleanup_expired_bodies(
            now_secs,
            self.debris_cleanup,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
        );
        if !result.removed_nodes.is_empty() {
            self.destroy_nodes_in_solver(&result.removed_nodes);
        }
        result
    }

    fn apply_excess_forces(&self, bodies: &mut RigidBodySet) {
        let actors = self.solver.actors();
        for actor in &actors {
            if actor.nodes.is_empty() {
                continue;
            }
            // Find the body for this actor
            let body_handle = match self.tracker.node_body(actor.nodes[0]) {
                Some(h) => h,
                None => continue,
            };
            let body = match bodies.get(body_handle) {
                Some(b) if b.is_dynamic() => b,
                _ => continue,
            };
            let pos = body.translation();
            let com = Vec3::new(pos.x, pos.y, pos.z);

            if let Some((force, torque)) = self.solver.get_excess_forces(actor.actor_index, com) {
                let force_mag = force.magnitude_squared();
                let torque_mag = torque.magnitude_squared();
                if force_mag > 1.0e-6 || torque_mag > 1.0e-6 {
                    if let Some(body_mut) = bodies.get_mut(body_handle) {
                        body_mut.add_force(vector![force.x, force.y, force.z], true);
                        body_mut.add_torque(vector![torque.x, torque.y, torque.z], true);
                    }
                }
            }
        }
    }

    fn process_pending_split_events(
        &mut self,
        now_secs: f32,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
        remaining_new_bodies: &mut usize,
        remaining_collider_migrations: &mut usize,
        result: &mut StepResult,
    ) {
        let pending_len = self.pending_split_events.len();

        for _ in 0..pending_len {
            let Some(event) = self.pending_split_events.pop_front() else {
                break;
            };
            let Some(filtered_event) = self.sanitize_split_event(
                &event,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
            ) else {
                continue;
            };

            let cost = self.tracker.estimate_split_cost(&filtered_event, bodies);
            if cost.create_bodies > *remaining_new_bodies
                || cost.collider_migrations > *remaining_collider_migrations
            {
                self.pending_split_events.push_front(filtered_event);
                break;
            }

            let new_handles = self.tracker.handle_split(
                &filtered_event,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
                now_secs,
                self.small_body_damping,
                self.sleep_thresholds,
            );

            *remaining_new_bodies = (*remaining_new_bodies).saturating_sub(cost.create_bodies);
            *remaining_collider_migrations =
                (*remaining_collider_migrations).saturating_sub(cost.collider_migrations);
            result.new_bodies += new_handles.len();
        }
    }

    fn sanitize_split_event(
        &mut self,
        event: &SplitEvent,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> Option<SplitEvent> {
        let mut destroyed_nodes = Vec::new();
        let mut children = Vec::new();

        for child in &event.children {
            let filtered_nodes: Vec<u32> = child
                .nodes
                .iter()
                .copied()
                .filter(|node| !self.destroyed_nodes[*node as usize])
                .collect();
            if filtered_nodes.is_empty() {
                continue;
            }

            let is_child_support = filtered_nodes.iter().any(|node| self.tracker.is_support(*node));
            let should_destroy = (!is_child_support && self.skip_single_bodies && filtered_nodes.len() <= 1)
                || (!is_child_support
                    && self.policy.min_child_node_count > 1
                    && filtered_nodes.len() < self.policy.min_child_node_count as usize);

            if should_destroy {
                destroyed_nodes.extend(filtered_nodes);
            } else {
                children.push(SplitChild {
                    actor_index: child.actor_index,
                    nodes: filtered_nodes,
                });
            }
        }

        if !destroyed_nodes.is_empty() {
            let removed_nodes = self.tracker.destroy_nodes(
                &destroyed_nodes,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
            );
            self.destroy_nodes_in_solver(&removed_nodes);
        }

        if children.is_empty() {
            None
        } else {
            Some(SplitEvent {
                parent_actor_index: event.parent_actor_index,
                children,
            })
        }
    }

    fn destroy_nodes_in_solver(&mut self, nodes: &[u32]) {
        let mut targets = Vec::new();
        for &node in nodes {
            let idx = node as usize;
            if idx >= self.destroyed_nodes.len() || self.destroyed_nodes[idx] {
                continue;
            }
            self.destroyed_nodes[idx] = true;
            targets.push(node);
        }
        if targets.is_empty() {
            return;
        }

        let actors = self.solver.actors();
        let mut node_to_actor = HashMap::new();
        for actor in &actors {
            for &node in &actor.nodes {
                node_to_actor.insert(node, actor.actor_index);
            }
        }

        let mut commands_by_actor: HashMap<u32, Vec<BondFracture>> = HashMap::new();
        for &node in &targets {
            for &bond_index in &self.node_bonds[node as usize] {
                let bond_index = bond_index as usize;
                if self.removed_bonds.get(bond_index).copied().unwrap_or(true) {
                    continue;
                }
                self.removed_bonds[bond_index] = true;
                let bond = self.bond_table[bond_index];
                let actor_index = node_to_actor
                    .get(&bond.node0)
                    .copied()
                    .or_else(|| node_to_actor.get(&bond.node1).copied());
                let Some(actor_index) = actor_index else {
                    continue;
                };
                commands_by_actor
                    .entry(actor_index)
                    .or_default()
                    .push(BondFracture {
                        userdata: bond_index as u32,
                        node_index0: bond.node0,
                        node_index1: bond.node1,
                        health: 0.0,
                    });
            }
        }

        if commands_by_actor.is_empty() {
            return;
        }

        let commands: Vec<FractureCommand> = commands_by_actor
            .into_iter()
            .map(|(actor_index, bond_fractures)| FractureCommand {
                actor_index,
                bond_fractures,
            })
            .collect();
        let events = self.solver.apply_fracture_commands(&commands);
        self.pending_split_events.extend(events);
    }

    fn mark_bonds_removed(&mut self, commands: &[FractureCommand]) {
        for command in commands {
            for fracture in &command.bond_fractures {
                let bond_index = self.resolve_bond_index(
                    fracture.userdata,
                    fracture.node_index0,
                    fracture.node_index1,
                );
                if let Some(idx) = bond_index {
                    if let Some(removed) = self.removed_bonds.get_mut(idx) {
                        *removed = true;
                    }
                }
            }
        }
    }

    fn resolve_bond_index(&self, userdata: u32, node0: u32, node1: u32) -> Option<usize> {
        let idx = userdata as usize;
        if self
            .bond_table
            .get(idx)
            .is_some_and(|bond| Self::bond_matches(bond, node0, node1))
        {
            return Some(idx);
        }

        self.node_bonds
            .get(node0 as usize)
            .into_iter()
            .flatten()
            .find_map(|bond_index| {
                let idx = *bond_index as usize;
                self.bond_table
                    .get(idx)
                    .filter(|bond| Self::bond_matches(bond, node0, node1))
                    .map(|_| idx)
            })
    }

    fn bond_matches(bond: &BondDesc, node0: u32, node1: u32) -> bool {
        (bond.node0 == node0 && bond.node1 == node1) || (bond.node0 == node1 && bond.node1 == node0)
    }
}
