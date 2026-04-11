use rapier3d::prelude::*;

use crate::ext_stress_solver::ExtStressSolver;
use crate::types::*;

use super::body_tracker::BodyTracker;
use super::fracture_policy::FracturePolicy;
use super::optimization::{DebrisCleanupOptions, OptimizationResult, SmallBodyDampingOptions};
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
    small_body_damping: SmallBodyDampingOptions,
    debris_cleanup: DebrisCleanupOptions,
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
            small_body_damping: config.small_body_damping,
            debris_cleanup: config.debris_cleanup,
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

        // Idle skip optimization
        if self.policy.idle_skip && !self.forces_applied && self.frames_since_fracture > 2 {
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

        // 5. Apply fracture commands
        let events = self.solver.apply_fracture_commands(&commands);
        result.split_events = events.len();

        // 6. Handle splits — create/destroy Rapier bodies
        let mut new_bodies_created = 0usize;
        let max_new = self.policy.clamp_new_bodies(usize::MAX);
        let policy = &self.policy;

        for event in &events {
            if new_bodies_created >= max_new {
                break;
            }

            let remaining = max_new - new_bodies_created;
            let mut budget = remaining;

            let new_handles = self.tracker.handle_split(
                event,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
                now_secs,
                self.small_body_damping,
                |node_count| {
                    if budget == 0 {
                        return false;
                    }
                    if !policy.child_qualifies(node_count) {
                        return false;
                    }
                    budget -= 1;
                    true
                },
            );

            new_bodies_created += new_handles.len();
        }

        // Optional: kick separated actors with solver-reported excess forces.
        if self.policy.apply_excess_forces {
            self.apply_excess_forces(bodies);
        }

        result.new_bodies = new_bodies_created;
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
        self.tracker
            .mark_support_contact(body_handle, now_secs, bodies, self.small_body_damping)
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
        self.tracker.cleanup_expired_bodies(
            now_secs,
            self.debris_cleanup,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
        )
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
}
