use std::collections::{HashMap, HashSet};
use std::ops::{Deref, DerefMut};
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc, Mutex,
};

use rapier3d::prelude::*;

#[cfg(not(target_arch = "wasm32"))]
use std::time::Instant;

#[cfg(target_arch = "wasm32")]
#[derive(Clone, Copy)]
struct Instant;

#[cfg(target_arch = "wasm32")]
impl Instant {
    fn now() -> Self {
        Self
    }

    fn elapsed(&self) -> std::time::Duration {
        std::time::Duration::ZERO
    }
}

use crate::types::Vec3 as SolverVec3;
use crate::{ScenarioDesc, SolverSettings, Vec3};

use super::body_tracker::SplitEditStats;
use super::destructible::{DestructibleSet, SplitCohort, StepResult};
use super::fracture_policy::FracturePolicy;
use super::optimization::OptimizationResult;
use super::resimulation::BodySnapshots;
use super::{
    DebrisCleanupOptions, DebrisCollisionMode, SleepThresholdOptions, SmallBodyDampingOptions,
};

#[derive(Clone, Copy, Debug)]
pub struct ContactImpactOptions {
    pub enabled: bool,
    pub min_total_impulse: f32,
    pub min_external_speed: f32,
    pub min_internal_speed: f32,
    pub cooldown_secs: f32,
    pub force_scale: f32,
    pub max_force_magnitude: f32,
    pub splash_radius: f32,
    pub splash_falloff_exponent: f32,
    pub internal_contact_scale: f32,
}

impl Default for ContactImpactOptions {
    fn default() -> Self {
        Self {
            enabled: true,
            min_total_impulse: 8.0,
            min_external_speed: 0.75,
            min_internal_speed: 0.25,
            cooldown_secs: 0.12,
            force_scale: 30.0,
            max_force_magnitude: f32::INFINITY,
            splash_radius: 2.0,
            splash_falloff_exponent: 2.0,
            internal_contact_scale: 0.5,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct GracePeriodOptions {
    pub sibling_steps: u32,
    pub impact_source_steps: u32,
}

impl Default for GracePeriodOptions {
    fn default() -> Self {
        Self {
            sibling_steps: 1,
            // The production resimulation contract replays projectile contacts
            // against the freshly fractured topology. We keep sibling grace to
            // suppress immediate self-collision churn, but projectile-vs-wall
            // replay contacts must stay enabled by default so the projectile can
            // continue fracturing and pushing through the broken wall without CCD.
            impact_source_steps: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DestructionRuntimeOptions {
    pub contact_impacts: ContactImpactOptions,
    pub grace: GracePeriodOptions,
    pub destructible: DestructibleRuntimeOptions,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DestructibleRuntimeOptions {
    pub sleep_thresholds: Option<SleepThresholdOptions>,
    pub small_body_damping: Option<SmallBodyDampingOptions>,
    pub debris_cleanup: Option<DebrisCleanupOptions>,
    pub debris_collision_mode: Option<DebrisCollisionMode>,
    pub split_child_recentering_enabled: Option<bool>,
    pub split_child_velocity_fit_enabled: Option<bool>,
    pub skip_single_bodies: Option<bool>,
}

/// Borrowed access to the caller-owned Rapier world state.
///
/// `DestructionRuntime` integrates into an existing Rapier application instead
/// of owning the physics pipeline itself. The consumer still calls
/// `PhysicsPipeline::step(...)` and passes these borrowed world sets back to the
/// runtime around that step.
pub struct RapierWorldAccess<'a> {
    pub bodies: &'a mut RigidBodySet,
    pub colliders: &'a mut ColliderSet,
    pub island_manager: &'a mut IslandManager,
    pub broad_phase: &'a mut BroadPhaseBvh,
    pub narrow_phase: &'a mut NarrowPhase,
    pub impulse_joints: &'a mut ImpulseJointSet,
    pub multibody_joints: &'a mut MultibodyJointSet,
    pub ccd_solver: &'a mut CCDSolver,
}

#[derive(Clone, Debug, Default)]
pub struct FrameResult {
    pub rapier_passes: u32,
    pub contact_pairs: usize,
    pub active_contact_pairs: usize,
    pub contact_manifolds: usize,
    pub accepted_impacts: usize,
    pub rejected_below_impulse: usize,
    pub rejected_below_speed: usize,
    pub rejected_cooldown: usize,
    pub support_contacts: usize,
    pub fractures: usize,
    pub new_bodies: usize,
    pub split_events: usize,
    pub split_edits: SplitEditStats,
    pub split_sanitize_ms: f32,
    pub split_estimate_ms: f32,
    pub split_cohorts: Vec<SplitCohort>,
    pub optimization: OptimizationResult,
    pub resim_restore_ms: f32,
    pub resim_snapshot_ms: f32,
    pub sibling_grace_cohorts: usize,
    pub sibling_grace_bodies: usize,
    pub sibling_grace_pairs: usize,
    pub sibling_grace_filtered_pairs: usize,
}

pub enum FrameDirective {
    Resimulate,
    Done(FrameResult),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
struct BodyKey(u32, u32);

impl From<RigidBodyHandle> for BodyKey {
    fn from(handle: RigidBodyHandle) -> Self {
        let (index, generation) = handle.into_raw_parts();
        Self(index, generation)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct BodyPairKey(BodyKey, BodyKey);

fn canonical_body_pair_key(body1: RigidBodyHandle, body2: RigidBodyHandle) -> Option<BodyPairKey> {
    if body1 == body2 {
        return None;
    }
    let key1 = BodyKey::from(body1);
    let key2 = BodyKey::from(body2);
    Some(if key1 <= key2 {
        BodyPairKey(key1, key2)
    } else {
        BodyPairKey(key2, key1)
    })
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
struct ImpactCooldownKey {
    pair: BodyPairKey,
    target_body: BodyKey,
}

#[derive(Clone)]
struct GraceCohort {
    remaining_steps: u32,
    bodies: Vec<RigidBodyHandle>,
}

#[derive(Clone)]
struct GracePairSet {
    remaining_steps: u32,
    pairs: Vec<BodyPairKey>,
}

#[derive(Clone)]
struct RuntimeHooks {
    active_pairs: Arc<HashSet<BodyPairKey>>,
    filtered_pairs: Arc<AtomicUsize>,
}

impl PhysicsHooks for RuntimeHooks {
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        let Some(body1) = context.rigid_body1 else {
            return Some(SolverFlags::default());
        };
        let Some(body2) = context.rigid_body2 else {
            return Some(SolverFlags::default());
        };
        let Some(pair) = canonical_body_pair_key(body1, body2) else {
            return Some(SolverFlags::default());
        };
        if self.active_pairs.contains(&pair) {
            self.filtered_pairs.fetch_add(1, Ordering::Relaxed);
            None
        } else {
            Some(SolverFlags::default())
        }
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        let Some(body1) = context.rigid_body1 else {
            return true;
        };
        let Some(body2) = context.rigid_body2 else {
            return true;
        };
        let Some(pair) = canonical_body_pair_key(body1, body2) else {
            return true;
        };
        if self.active_pairs.contains(&pair) {
            self.filtered_pairs.fetch_add(1, Ordering::Relaxed);
            false
        } else {
            true
        }
    }
}

#[derive(Clone, Copy)]
struct BufferedContactForceEvent {
    dt: Real,
    collider1: ColliderHandle,
    collider2: ColliderHandle,
    total_force_magnitude: Real,
}

#[derive(Default)]
struct BufferedPassEvents {
    collisions: Vec<CollisionEvent>,
    contact_forces: Vec<BufferedContactForceEvent>,
}

pub struct PassAdapter<'a, H: PhysicsHooks + ?Sized, E: EventHandler + ?Sized> {
    runtime: RuntimeHooks,
    user_hooks: &'a H,
    user_events: &'a E,
    buffered_events: Arc<Mutex<BufferedPassEvents>>,
}

impl<H, E> PhysicsHooks for PassAdapter<'_, H, E>
where
    H: PhysicsHooks + ?Sized,
    E: EventHandler + ?Sized,
{
    fn filter_contact_pair(&self, context: &PairFilterContext) -> Option<SolverFlags> {
        self.runtime
            .filter_contact_pair(context)
            .and_then(|_| self.user_hooks.filter_contact_pair(context))
    }

    fn filter_intersection_pair(&self, context: &PairFilterContext) -> bool {
        self.runtime.filter_intersection_pair(context)
            && self.user_hooks.filter_intersection_pair(context)
    }

    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        self.user_hooks.modify_solver_contacts(context);
    }
}

impl<H, E> EventHandler for PassAdapter<'_, H, E>
where
    H: PhysicsHooks + Sync + ?Sized,
    E: EventHandler + ?Sized,
{
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.buffered_events
            .lock()
            .expect("buffered pass events mutex poisoned")
            .collisions
            .push(event);
    }

    fn handle_contact_force_event(
        &self,
        dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        contact_pair: &ContactPair,
        total_force_magnitude: Real,
    ) {
        self.buffered_events
            .lock()
            .expect("buffered pass events mutex poisoned")
            .contact_forces
            .push(BufferedContactForceEvent {
                dt,
                collider1: contact_pair.collider1,
                collider2: contact_pair.collider2,
                total_force_magnitude,
            });
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct GraceStepStats {
    active_cohorts: usize,
    active_bodies: usize,
    active_pairs: usize,
    filtered_pairs: usize,
}

struct GraceState {
    sibling_steps: u32,
    impact_source_steps: u32,
    cohorts: Vec<GraceCohort>,
    pair_sets: Vec<GracePairSet>,
    active_pairs: Arc<HashSet<BodyPairKey>>,
    filtered_pairs: Arc<AtomicUsize>,
}

impl GraceState {
    fn new(options: GracePeriodOptions) -> Self {
        Self {
            sibling_steps: options.sibling_steps,
            impact_source_steps: options.impact_source_steps,
            cohorts: Vec::new(),
            pair_sets: Vec::new(),
            active_pairs: Arc::new(HashSet::new()),
            filtered_pairs: Arc::new(AtomicUsize::new(0)),
        }
    }

    fn set_options(&mut self, options: GracePeriodOptions) {
        self.sibling_steps = options.sibling_steps;
        self.impact_source_steps = options.impact_source_steps;
        self.cohorts.clear();
        self.pair_sets.clear();
        self.rebuild_pairs();
    }

    fn register_split_cohorts(&mut self, split_cohorts: &[SplitCohort]) {
        if self.sibling_steps == 0 {
            return;
        }
        for cohort in split_cohorts {
            let unique_bodies: Vec<_> = cohort
                .target_bodies
                .iter()
                .copied()
                .collect::<HashSet<_>>()
                .into_iter()
                .collect();
            if unique_bodies.len() > 1 {
                self.cohorts.push(GraceCohort {
                    remaining_steps: self.sibling_steps,
                    bodies: unique_bodies,
                });
            }
        }
        self.rebuild_pairs();
    }

    fn register_impact_fracture_pairs(
        &mut self,
        impact_sources: &HashSet<RigidBodyHandle>,
        impacted_bodies: &HashSet<RigidBodyHandle>,
        split_cohorts: &[SplitCohort],
    ) {
        if self.impact_source_steps == 0
            || impact_sources.is_empty()
            || impacted_bodies.is_empty()
            || split_cohorts.is_empty()
        {
            return;
        }

        let matching_targets: Vec<Vec<RigidBodyHandle>> = split_cohorts
            .iter()
            .filter(|cohort| {
                cohort
                    .source_bodies
                    .iter()
                    .any(|body| impacted_bodies.contains(body))
            })
            .map(|cohort| {
                cohort
                    .target_bodies
                    .iter()
                    .chain(cohort.source_bodies.iter())
                    .copied()
                    .collect()
            })
            .collect();

        for &source_body in impact_sources {
            let mut pairs = Vec::new();
            let mut seen = HashSet::new();
            for targets in &matching_targets {
                for &target_body in targets {
                    let Some(pair) = canonical_body_pair_key(source_body, target_body) else {
                        continue;
                    };
                    if seen.insert(pair) {
                        pairs.push(pair);
                    }
                }
            }
            if !pairs.is_empty() {
                self.pair_sets.push(GracePairSet {
                    remaining_steps: self.impact_source_steps,
                    pairs,
                });
            }
        }
        self.rebuild_pairs();
    }

    fn begin_step(&self) -> (RuntimeHooks, GraceStepStats) {
        self.filtered_pairs.store(0, Ordering::Relaxed);
        let unique_bodies = self
            .cohorts
            .iter()
            .flat_map(|cohort| cohort.bodies.iter().copied())
            .collect::<HashSet<_>>()
            .len();
        (
            RuntimeHooks {
                active_pairs: Arc::clone(&self.active_pairs),
                filtered_pairs: Arc::clone(&self.filtered_pairs),
            },
            GraceStepStats {
                active_cohorts: self.cohorts.len(),
                active_bodies: unique_bodies,
                active_pairs: self.active_pairs.len(),
                filtered_pairs: 0,
            },
        )
    }

    fn finish_step(&mut self, mut stats: GraceStepStats) -> GraceStepStats {
        stats.filtered_pairs = self.filtered_pairs.swap(0, Ordering::Relaxed);
        for cohort in &mut self.cohorts {
            cohort.remaining_steps = cohort.remaining_steps.saturating_sub(1);
        }
        for pair_set in &mut self.pair_sets {
            pair_set.remaining_steps = pair_set.remaining_steps.saturating_sub(1);
        }
        self.cohorts.retain(|cohort| cohort.remaining_steps > 0);
        self.pair_sets
            .retain(|pair_set| pair_set.remaining_steps > 0);
        self.rebuild_pairs();
        stats
    }

    fn rebuild_pairs(&mut self) {
        let mut active_pairs = HashSet::new();
        for cohort in &self.cohorts {
            for i in 0..cohort.bodies.len() {
                for j in (i + 1)..cohort.bodies.len() {
                    if let Some(pair) = canonical_body_pair_key(cohort.bodies[i], cohort.bodies[j])
                    {
                        active_pairs.insert(pair);
                    }
                }
            }
        }
        for pair_set in &self.pair_sets {
            for &pair in &pair_set.pairs {
                active_pairs.insert(pair);
            }
        }
        self.active_pairs = Arc::new(active_pairs);
    }
}

#[derive(Clone)]
struct ImpactCandidate {
    cooldown_key: ImpactCooldownKey,
    node_index: u32,
    target_body: RigidBodyHandle,
    total_impulse: f32,
    closing_speed: f32,
    local_point: SolverVec3,
    local_force: Vector<Real>,
    is_internal: bool,
}

#[derive(Clone, Copy)]
struct BodyMotionSnapshot {
    position: Isometry<Real>,
    linvel: Vector<Real>,
    angvel: AngVector<Real>,
}

#[derive(Default)]
struct PassAnalysis {
    support_contacts: HashSet<RigidBodyHandle>,
    impact_candidates: Vec<ImpactCandidate>,
    impact_sources: HashSet<RigidBodyHandle>,
    impacted_bodies: HashSet<RigidBodyHandle>,
    contact_pairs: usize,
    active_contact_pairs: usize,
    contact_manifolds: usize,
}

struct ActiveFrameState {
    now_secs: f32,
    dt: f32,
    remaining_resim_passes: usize,
    snapshot: Option<BodySnapshots>,
    cooldown_snapshot: HashMap<ImpactCooldownKey, f32>,
    pre_step_motion: HashMap<RigidBodyHandle, BodyMotionSnapshot>,
    impact_sources: HashSet<RigidBodyHandle>,
    pending_grace_stats: Option<GraceStepStats>,
    result: FrameResult,
}

pub struct DestructionRuntime {
    destructible: DestructibleSet,
    options: DestructionRuntimeOptions,
    cooldowns: HashMap<ImpactCooldownKey, f32>,
    grace: GraceState,
    frame: Option<ActiveFrameState>,
}

impl DestructionRuntime {
    /// Creates a runtime around an existing low-level [`DestructibleSet`].
    pub fn new(destructible: DestructibleSet, options: DestructionRuntimeOptions) -> Self {
        let mut runtime = Self {
            destructible,
            options,
            cooldowns: HashMap::new(),
            grace: GraceState::new(options.grace),
            frame: None,
        };
        runtime.apply_destructible_options(options.destructible);
        runtime
    }

    /// Builds a high-level runtime directly from a scenario description.
    pub fn from_scenario(
        scenario: &ScenarioDesc,
        settings: SolverSettings,
        gravity: Vec3,
        fracture_policy: FracturePolicy,
        options: DestructionRuntimeOptions,
    ) -> Option<Self> {
        let destructible =
            DestructibleSet::from_scenario(scenario, settings, gravity, fracture_policy)?;
        Some(Self::new(destructible, options))
    }

    pub fn options(&self) -> DestructionRuntimeOptions {
        self.options
    }

    pub fn set_options(&mut self, options: DestructionRuntimeOptions) {
        self.options = options;
        self.grace.set_options(options.grace);
        self.apply_destructible_options(options.destructible);
    }

    pub fn into_inner(self) -> DestructibleSet {
        self.destructible
    }

    pub fn initialize(
        &mut self,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
    ) -> Vec<RigidBodyHandle> {
        self.destructible.initialize(bodies, colliders)
    }

    pub fn set_ground_body_handle(&mut self, handle: Option<RigidBodyHandle>) {
        self.destructible.set_ground_body_handle(handle);
    }

    pub fn refresh_collision_groups(&mut self, bodies: &RigidBodySet, colliders: &mut ColliderSet) {
        self.destructible
            .refresh_collision_groups(bodies, colliders);
    }

    pub fn set_sleep_thresholds(&mut self, options: SleepThresholdOptions) {
        self.destructible.set_sleep_thresholds(options);
    }

    pub fn set_small_body_damping(&mut self, options: SmallBodyDampingOptions) {
        self.destructible.set_small_body_damping(options);
    }

    pub fn set_debris_cleanup(&mut self, options: DebrisCleanupOptions) {
        self.destructible.set_debris_cleanup(options);
    }

    pub fn set_debris_collision_mode(&mut self, mode: DebrisCollisionMode) {
        self.destructible.set_debris_collision_mode(mode);
    }

    pub fn set_resimulation_options(&mut self, options: super::resimulation::ResimulationOptions) {
        self.destructible.set_resimulation_options(options);
    }

    pub fn set_dynamic_body_ccd_enabled(&mut self, enabled: bool) {
        self.destructible.set_dynamic_body_ccd_enabled(enabled);
    }

    pub fn set_split_child_recentering_enabled(&mut self, enabled: bool) {
        self.destructible
            .set_split_child_recentering_enabled(enabled);
    }

    pub fn set_split_child_velocity_fit_enabled(&mut self, enabled: bool) {
        self.destructible
            .set_split_child_velocity_fit_enabled(enabled);
    }

    pub fn active_bond_count(&self) -> usize {
        self.destructible.active_bond_count()
    }

    pub fn body_count(&self) -> usize {
        self.destructible.body_count()
    }

    pub fn body_nodes_slice(&self, body_handle: RigidBodyHandle) -> &[u32] {
        self.destructible.body_nodes_slice(body_handle)
    }

    pub fn body_has_support(&self, body_handle: RigidBodyHandle) -> bool {
        self.destructible.body_has_support(body_handle)
    }

    pub fn fracture_all_bonds_now(
        &mut self,
        now_secs: f32,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> StepResult {
        self.destructible.fracture_all_bonds_now(
            now_secs,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
        )
    }

    pub fn fracture_bond_indices_now(
        &mut self,
        now_secs: f32,
        bond_indices: &[usize],
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        island_manager: &mut IslandManager,
        impulse_joints: &mut ImpulseJointSet,
        multibody_joints: &mut MultibodyJointSet,
    ) -> StepResult {
        self.destructible.fracture_bond_indices_now(
            now_secs,
            bond_indices,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
        )
    }

    pub fn capture_resimulation_snapshot(&self, bodies: &RigidBodySet) -> BodySnapshots {
        self.destructible.capture_resimulation_snapshot(bodies)
    }

    pub fn restore_resimulation_split_children(
        &mut self,
        snapshot: &BodySnapshots,
        bodies: &mut RigidBodySet,
        colliders: &mut ColliderSet,
        split_cohorts: &[SplitCohort],
    ) {
        self.destructible.restore_resimulation_split_children(
            snapshot,
            bodies,
            colliders,
            split_cohorts,
        )
    }

    pub fn set_skip_single_bodies(&mut self, enabled: bool) {
        self.destructible.set_skip_single_bodies(enabled);
    }

    /// Begins an advanced multi-pass frame.
    ///
    /// Most consumers should prefer [`Self::step_frame`], which wraps the full
    /// destruction-aware Rapier loop for a frame.
    pub fn begin_frame(&mut self, now_secs: f32, dt: f32, bodies: &RigidBodySet) {
        let resimulation = self.destructible.resimulation_options();
        let mut result = FrameResult::default();
        let snapshot = if self.destructible.needs_resimulation_snapshot() {
            let snapshot_started_at = Instant::now();
            let captured = self.destructible.capture_resimulation_snapshot(bodies);
            result.resim_snapshot_ms +=
                snapshot_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
            Some(captured)
        } else {
            None
        };
        self.frame = Some(ActiveFrameState {
            now_secs,
            dt,
            remaining_resim_passes: if resimulation.enabled {
                resimulation.max_passes
            } else {
                0
            },
            snapshot,
            cooldown_snapshot: self.cooldowns.clone(),
            pre_step_motion: capture_body_motion(bodies),
            impact_sources: HashSet::new(),
            pending_grace_stats: None,
            result,
        });
    }

    pub fn begin_pass<'a, H, E>(
        &mut self,
        user_hooks: &'a H,
        user_events: &'a E,
    ) -> PassAdapter<'a, H, E>
    where
        H: PhysicsHooks + ?Sized,
        E: EventHandler + ?Sized,
    {
        let (runtime_hooks, stats) = self.grace.begin_step();
        if let Some(frame) = self.frame.as_mut() {
            frame.pending_grace_stats = Some(stats);
        }
        PassAdapter {
            runtime: runtime_hooks,
            user_hooks,
            user_events,
            buffered_events: Arc::new(Mutex::new(BufferedPassEvents::default())),
        }
    }

    /// Completes one speculative Rapier pass started by [`Self::begin_frame`].
    pub fn finish_pass<H, E>(
        &mut self,
        pass: PassAdapter<'_, H, E>,
        world: &mut RapierWorldAccess<'_>,
    ) -> FrameDirective
    where
        H: PhysicsHooks + Sync + ?Sized,
        E: EventHandler + ?Sized,
    {
        let mut frame = self
            .frame
            .take()
            .expect("begin_frame must be called before finish_pass");
        frame.result.rapier_passes += 1;

        self.cooldowns.retain(|_, until| *until > frame.now_secs);
        let buffered_events = pass
            .buffered_events
            .lock()
            .expect("buffered pass events mutex poisoned");
        let analysis =
            self.collect_pass_analysis(frame.dt, &frame.pre_step_motion, &buffered_events, world);
        frame
            .impact_sources
            .extend(analysis.impact_sources.iter().copied());
        frame.result.contact_pairs = analysis.contact_pairs;
        frame.result.active_contact_pairs = analysis.active_contact_pairs;
        frame.result.contact_manifolds = analysis.contact_manifolds;

        self.apply_impact_candidates(frame.now_secs, &analysis, &mut frame.result);

        let solver_result = self.destructible.step_with_time(
            frame.now_secs,
            world.bodies,
            world.colliders,
            world.island_manager,
            world.impulse_joints,
            world.multibody_joints,
        );
        accumulate_step_result(&mut frame.result, solver_result.clone());

        if let Some(pending_stats) = frame.pending_grace_stats.take() {
            let grace_stats = self.grace.finish_step(pending_stats);
            frame.result.sibling_grace_cohorts = frame
                .result
                .sibling_grace_cohorts
                .max(grace_stats.active_cohorts);
            frame.result.sibling_grace_bodies = frame
                .result
                .sibling_grace_bodies
                .max(grace_stats.active_bodies);
            frame.result.sibling_grace_pairs = frame
                .result
                .sibling_grace_pairs
                .max(grace_stats.active_pairs);
            frame.result.sibling_grace_filtered_pairs += grace_stats.filtered_pairs;
        }

        if !solver_result.split_cohorts.is_empty() {
            self.grace
                .register_split_cohorts(&solver_result.split_cohorts);
            self.grace.register_impact_fracture_pairs(
                &analysis.impact_sources,
                &analysis.impacted_bodies,
                &solver_result.split_cohorts,
            );
        }

        let fractured = solver_result.split_events > 0 || solver_result.new_bodies > 0;
        if fractured && frame.remaining_resim_passes > 0 {
            self.cooldowns = frame.cooldown_snapshot.clone();
            *world.narrow_phase = NarrowPhase::new();
            if let Some(snapshot) = frame.snapshot.as_ref() {
                let restore_started_at = Instant::now();
                snapshot.restore(world.bodies);
                self.destructible.restore_resimulation_split_children(
                    snapshot,
                    world.bodies,
                    world.colliders,
                    &solver_result.split_cohorts,
                );
                // Replay teleports many bodies/colliders back to a pre-impact pose.
                // Mark the whole world as modified so Rapier recomputes collider
                // positions and contact state from the restored topology instead
                // of relying on an incremental diff from the speculative pass.
                for _ in world.bodies.iter_mut() {}
                world
                    .bodies
                    .propagate_modified_body_positions_to_colliders(world.colliders);
                for _ in world.colliders.iter_mut() {}
                frame.result.resim_restore_ms +=
                    restore_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
            }
            frame.remaining_resim_passes = frame.remaining_resim_passes.saturating_sub(1);
            if frame.remaining_resim_passes > 0 {
                let snapshot_started_at = Instant::now();
                frame.snapshot = Some(
                    self.destructible
                        .capture_resimulation_snapshot(world.bodies),
                );
                frame.result.resim_snapshot_ms +=
                    snapshot_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
                frame.pre_step_motion = capture_body_motion(world.bodies);
            }
            self.frame = Some(frame);
            return FrameDirective::Resimulate;
        }

        for body_handle in analysis.support_contacts {
            if self.destructible.mark_body_support_contact(
                body_handle,
                frame.now_secs,
                world.bodies,
                world.colliders,
            ) {
                frame.result.support_contacts += 1;
            }
        }

        if frame.result.rapier_passes > 1 {
            self.restore_unblocked_impact_source_motion(
                &frame.impact_sources,
                &frame.pre_step_motion,
                world,
            );
        }

        frame.result.optimization = self.destructible.process_optimizations(
            frame.now_secs,
            world.bodies,
            world.colliders,
            world.island_manager,
            world.impulse_joints,
            world.multibody_joints,
        );

        flush_buffered_events(pass.user_events, &buffered_events, world);
        self.frame = None;
        FrameDirective::Done(frame.result)
    }

    /// Recommended high-level integration for existing Rapier apps.
    ///
    /// The consumer still owns `PhysicsPipeline::step(...)`. This helper wraps
    /// the destruction-specific orchestration around that step: it buffers
    /// speculative events, analyzes Rapier contacts, injects accepted impacts,
    /// applies fractures and split edits, and resimulates the frame when body
    /// topology changes.
    pub fn step_frame<H, E, S>(
        &mut self,
        now_secs: f32,
        dt: f32,
        world: &mut RapierWorldAccess<'_>,
        user_hooks: &H,
        user_events: &E,
        mut step: S,
    ) -> FrameResult
    where
        H: PhysicsHooks + Sync + ?Sized,
        E: EventHandler + ?Sized,
        S: FnMut(&PassAdapter<'_, H, E>, &mut RapierWorldAccess<'_>),
    {
        self.begin_frame(now_secs, dt, world.bodies);
        loop {
            let pass = self.begin_pass(user_hooks, user_events);
            step(&pass, world);
            match self.finish_pass(pass, world) {
                FrameDirective::Resimulate => continue,
                FrameDirective::Done(result) => return result,
            }
        }
    }

    fn collect_pass_analysis(
        &self,
        dt: f32,
        pre_step_motion: &HashMap<RigidBodyHandle, BodyMotionSnapshot>,
        buffered_events: &BufferedPassEvents,
        world: &RapierWorldAccess<'_>,
    ) -> PassAnalysis {
        let mut analysis = PassAnalysis::default();
        let dt = dt.max(1.0e-6);

        for pair in world.narrow_phase.contact_pairs() {
            analysis.contact_pairs += 1;
            analysis.contact_manifolds += pair.manifolds.len();
            if !pair.has_any_active_contact {
                continue;
            }
            analysis.active_contact_pairs += 1;

            let Some(collider1) = world.colliders.get(pair.collider1) else {
                continue;
            };
            let Some(collider2) = world.colliders.get(pair.collider2) else {
                continue;
            };
            let Some(body1) = collider1.parent() else {
                continue;
            };
            let Some(body2) = collider2.parent() else {
                continue;
            };

            let Some(_body_pair_key) = canonical_body_pair_key(body1, body2) else {
                continue;
            };

            let node1 = self.destructible.collider_node(pair.collider1);
            let node2 = self.destructible.collider_node(pair.collider2);
            if node1.is_none() && node2.is_none() {
                continue;
            }

            if !self.options.contact_impacts.enabled
                || self.options.contact_impacts.force_scale <= 0.0
            {
                continue;
            }

            let pair_is_destructible_on_both_sides = node1.is_some() && node2.is_some();

            if let Some(candidate) = self.build_external_candidate(
                pair,
                dt,
                pair.collider1,
                body1,
                body2,
                node1,
                false,
                pair_is_destructible_on_both_sides,
                pre_step_motion,
                world,
            ) {
                analysis.impact_candidates.push(candidate);
                analysis.impact_sources.insert(body2);
                analysis.impacted_bodies.insert(body1);
            }
            if let Some(candidate) = self.build_external_candidate(
                pair,
                dt,
                pair.collider2,
                body2,
                body1,
                node2,
                true,
                pair_is_destructible_on_both_sides,
                pre_step_motion,
                world,
            ) {
                analysis.impact_candidates.push(candidate);
                analysis.impact_sources.insert(body1);
                analysis.impacted_bodies.insert(body2);
            }
        }

        for &event in &buffered_events.collisions {
            let CollisionEvent::Started(collider1_handle, collider2_handle, _) = event else {
                continue;
            };
            let Some(collider1) = world.colliders.get(collider1_handle) else {
                continue;
            };
            let Some(collider2) = world.colliders.get(collider2_handle) else {
                continue;
            };
            let Some(body1) = collider1.parent() else {
                continue;
            };
            let Some(body2) = collider2.parent() else {
                continue;
            };
            if let Some(node_index) = self.destructible.collider_node(collider1_handle) {
                if self.support_contact_for_pair(node_index, body1, collider2_handle, body2, world)
                {
                    analysis.support_contacts.insert(body1);
                }
            }
            if let Some(node_index) = self.destructible.collider_node(collider2_handle) {
                if self.support_contact_for_pair(node_index, body2, collider1_handle, body1, world)
                {
                    analysis.support_contacts.insert(body2);
                }
            }
        }

        analysis
    }

    fn build_external_candidate(
        &self,
        pair: &ContactPair,
        dt: f32,
        target_collider: ColliderHandle,
        target_body: RigidBodyHandle,
        other_body: RigidBodyHandle,
        target_node: Option<u32>,
        target_is_second: bool,
        is_internal: bool,
        pre_step_motion: &HashMap<RigidBodyHandle, BodyMotionSnapshot>,
        world: &RapierWorldAccess<'_>,
    ) -> Option<ImpactCandidate> {
        let target_node = target_node?;
        if self.destructible.is_support(target_node) {
            return None;
        }

        let (world_point, force_world, total_impulse) =
            self.contact_geometry(pair, target_collider, target_is_second, dt, world)?;
        let closing_speed = self.closing_speed(
            world_point,
            force_world,
            target_body,
            other_body,
            pre_step_motion,
            world,
        );
        let Some(target_rb) = world.bodies.get(target_body) else {
            return None;
        };
        let local_point = target_rb.position().inverse_transform_point(&world_point);
        let local_force = target_rb.position().rotation.inverse() * force_world;

        Some(ImpactCandidate {
            cooldown_key: ImpactCooldownKey {
                pair: canonical_body_pair_key(target_body, other_body)?,
                target_body: BodyKey::from(target_body),
            },
            node_index: target_node,
            target_body,
            total_impulse,
            closing_speed,
            local_point: SolverVec3::new(local_point.x, local_point.y, local_point.z),
            local_force,
            is_internal,
        })
    }

    fn contact_geometry(
        &self,
        pair: &ContactPair,
        target_collider: ColliderHandle,
        target_is_second: bool,
        dt: f32,
        world: &RapierWorldAccess<'_>,
    ) -> Option<(Point<Real>, Vector<Real>, f32)> {
        let total_impulse = pair.total_impulse();
        let total_impulse_mag = pair.total_impulse_magnitude();
        if total_impulse_mag <= 1.0e-6 {
            return None;
        }

        let (manifold, contact) = pair.find_deepest_contact()?;
        let target_local_point = if target_is_second {
            contact.local_p2
        } else {
            contact.local_p1
        };
        let collider = world.colliders.get(target_collider)?;
        let world_point = collider.position() * target_local_point;
        let force_world = if target_is_second {
            total_impulse / dt
        } else {
            -total_impulse / dt
        };
        let force_world = if force_world.norm_squared() > 1.0e-12 {
            force_world
        } else {
            let normal = if target_is_second {
                manifold.data.normal
            } else {
                -manifold.data.normal
            };
            normal * (total_impulse_mag / dt)
        };
        Some((world_point, force_world, total_impulse_mag))
    }

    fn closing_speed(
        &self,
        world_point: Point<Real>,
        force_world: Vector<Real>,
        target_body: RigidBodyHandle,
        other_body: RigidBodyHandle,
        pre_step_motion: &HashMap<RigidBodyHandle, BodyMotionSnapshot>,
        world: &RapierWorldAccess<'_>,
    ) -> f32 {
        if let (Some(target_motion), Some(other_motion)) = (
            pre_step_motion.get(&target_body),
            pre_step_motion.get(&other_body),
        ) {
            let target_velocity = motion_velocity_at_point(target_motion, world_point);
            let other_velocity = motion_velocity_at_point(other_motion, world_point);
            return relative_speed_along_force(force_world, other_velocity - target_velocity);
        }

        let Some(target_rb) = world.bodies.get(target_body) else {
            return 0.0;
        };
        let Some(other_rb) = world.bodies.get(other_body) else {
            return 0.0;
        };
        let target_velocity = target_rb.velocity_at_point(&world_point);
        let other_velocity = other_rb.velocity_at_point(&world_point);
        relative_speed_along_force(force_world, other_velocity - target_velocity)
    }

    fn support_contact_for_pair(
        &self,
        node_index: u32,
        body_handle: RigidBodyHandle,
        other_collider: ColliderHandle,
        other_body: RigidBodyHandle,
        world: &RapierWorldAccess<'_>,
    ) -> bool {
        if self.destructible.is_support(node_index) {
            return false;
        }
        if other_body == body_handle {
            return false;
        }
        world
            .bodies
            .get(other_body)
            .map(|body| body.is_fixed())
            .unwrap_or(false)
            || self
                .destructible
                .collider_node(other_collider)
                .map(|other_node| self.destructible.is_support(other_node))
                .unwrap_or(false)
            || self.destructible.body_has_support(other_body)
    }

    fn apply_impact_candidates(
        &mut self,
        now_secs: f32,
        analysis: &PassAnalysis,
        result: &mut FrameResult,
    ) {
        let mut candidates = analysis.impact_candidates.clone();
        candidates.sort_by(|a, b| {
            b.total_impulse
                .partial_cmp(&a.total_impulse)
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        for candidate in candidates {
            let min_speed = if candidate.is_internal {
                self.options.contact_impacts.min_internal_speed
            } else {
                self.options.contact_impacts.min_external_speed
            };
            if candidate.total_impulse < self.options.contact_impacts.min_total_impulse {
                result.rejected_below_impulse += 1;
                continue;
            }
            if candidate.closing_speed < min_speed {
                result.rejected_below_speed += 1;
                continue;
            }
            if let Some(until) = self.cooldowns.get(&candidate.cooldown_key) {
                if *until > now_secs {
                    result.rejected_cooldown += 1;
                    continue;
                }
            }

            let mut local_force = candidate.local_force;
            let mut force_scale = self.options.contact_impacts.force_scale
                * if candidate.is_internal {
                    self.options.contact_impacts.internal_contact_scale
                } else {
                    1.0
                };
            let scaled_mag = local_force.norm() * force_scale;
            if self.options.contact_impacts.max_force_magnitude.is_finite()
                && self.options.contact_impacts.max_force_magnitude > 0.0
                && scaled_mag > self.options.contact_impacts.max_force_magnitude
            {
                force_scale *= self.options.contact_impacts.max_force_magnitude / scaled_mag;
            }
            local_force *= force_scale;
            if local_force.norm_squared() <= 1.0e-12 {
                continue;
            }

            self.apply_localized_force(
                candidate.target_body,
                candidate.node_index,
                candidate.local_point,
                local_force,
            );

            self.cooldowns.insert(
                candidate.cooldown_key,
                now_secs + self.options.contact_impacts.cooldown_secs.max(0.0),
            );
            result.accepted_impacts += 1;
        }
    }

    fn apply_localized_force(
        &mut self,
        body_handle: RigidBodyHandle,
        node_index: u32,
        hit_local: SolverVec3,
        local_force: Vector<Real>,
    ) {
        let splash_radius = self.options.contact_impacts.splash_radius.max(0.0);
        let splash_exp = self
            .options
            .contact_impacts
            .splash_falloff_exponent
            .max(0.01);

        let impacted_nodes: Vec<(u32, SolverVec3, f32)> = self
            .destructible
            .body_nodes_slice(body_handle)
            .iter()
            .copied()
            .filter_map(|other_node| {
                let other_local = self.destructible.node_local_offset(other_node)?;
                let dx = other_local.x - hit_local.x;
                let dy = other_local.y - hit_local.y;
                let dz = other_local.z - hit_local.z;
                let dist = (dx * dx + dy * dy + dz * dz).sqrt();
                let weight = if other_node == node_index {
                    1.0
                } else if splash_radius <= 0.0 || dist > splash_radius {
                    0.0
                } else {
                    (1.0 - dist / splash_radius).max(0.0).powf(splash_exp)
                };
                (weight > 0.0).then_some((other_node, other_local, weight))
            })
            .collect();

        for (other_node, other_local, weight) in impacted_nodes {
            self.destructible.add_force(
                other_node,
                other_local,
                SolverVec3::new(
                    local_force.x * weight,
                    local_force.y * weight,
                    local_force.z * weight,
                ),
            );
        }
    }

    fn apply_destructible_options(&mut self, options: DestructibleRuntimeOptions) {
        if let Some(value) = options.sleep_thresholds {
            self.destructible.set_sleep_thresholds(value);
        }
        if let Some(value) = options.small_body_damping {
            self.destructible.set_small_body_damping(value);
        }
        if let Some(value) = options.debris_cleanup {
            self.destructible.set_debris_cleanup(value);
        }
        if let Some(value) = options.debris_collision_mode {
            self.destructible.set_debris_collision_mode(value);
        }
        if let Some(value) = options.split_child_recentering_enabled {
            self.destructible.set_split_child_recentering_enabled(value);
        }
        if let Some(value) = options.split_child_velocity_fit_enabled {
            self.destructible
                .set_split_child_velocity_fit_enabled(value);
        }
        if let Some(value) = options.skip_single_bodies {
            self.destructible.set_skip_single_bodies(value);
        }
    }

    fn restore_unblocked_impact_source_motion(
        &self,
        impact_sources: &HashSet<RigidBodyHandle>,
        pre_step_motion: &HashMap<RigidBodyHandle, BodyMotionSnapshot>,
        world: &mut RapierWorldAccess<'_>,
    ) {
        for &body_handle in impact_sources {
            if !self.destructible.body_nodes_slice(body_handle).is_empty() {
                continue;
            }
            if source_has_active_destructible_contact(body_handle, world, &self.destructible) {
                continue;
            }
            let Some(motion) = pre_step_motion.get(&body_handle) else {
                continue;
            };
            let Some(body) = world.bodies.get_mut(body_handle) else {
                continue;
            };
            body.set_linvel(motion.linvel, true);
            body.set_angvel(motion.angvel, true);
            body.wake_up(true);
        }
    }
}

impl Deref for DestructionRuntime {
    type Target = DestructibleSet;

    fn deref(&self) -> &Self::Target {
        &self.destructible
    }
}

impl DerefMut for DestructionRuntime {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.destructible
    }
}

fn accumulate_step_result(result: &mut FrameResult, step: StepResult) {
    result.fractures += step.fractures;
    result.new_bodies += step.new_bodies;
    result.split_events += step.split_events;
    result.split_edits.plan_ms += step.split_edits.plan_ms;
    result.split_edits.apply_ms += step.split_edits.apply_ms;
    result.split_edits.child_pose_ms += step.split_edits.child_pose_ms;
    result.split_edits.velocity_fit_ms += step.split_edits.velocity_fit_ms;
    result.split_edits.sleep_init_ms += step.split_edits.sleep_init_ms;
    result.split_edits.body_create_ms += step.split_edits.body_create_ms;
    result.split_edits.collider_move_ms += step.split_edits.collider_move_ms;
    result.split_edits.collider_insert_ms += step.split_edits.collider_insert_ms;
    result.split_edits.body_retire_ms += step.split_edits.body_retire_ms;
    result.split_edits.reused_bodies += step.split_edits.reused_bodies;
    result.split_edits.recycled_bodies += step.split_edits.recycled_bodies;
    result.split_edits.created_bodies += step.split_edits.created_bodies;
    result.split_edits.retired_bodies += step.split_edits.retired_bodies;
    result.split_edits.body_type_flips += step.split_edits.body_type_flips;
    result.split_edits.moved_colliders += step.split_edits.moved_colliders;
    result.split_edits.inserted_colliders += step.split_edits.inserted_colliders;
    result.split_edits.removed_colliders += step.split_edits.removed_colliders;
    result.split_sanitize_ms += step.split_sanitize_ms;
    result.split_estimate_ms += step.split_estimate_ms;
    result.split_cohorts.extend(step.split_cohorts);
}

fn capture_body_motion(set: &RigidBodySet) -> HashMap<RigidBodyHandle, BodyMotionSnapshot> {
    set.iter()
        .map(|(handle, body)| {
            (
                handle,
                BodyMotionSnapshot {
                    position: *body.position(),
                    linvel: *body.linvel(),
                    angvel: *body.angvel(),
                },
            )
        })
        .collect()
}

fn source_has_active_destructible_contact(
    body_handle: RigidBodyHandle,
    world: &RapierWorldAccess<'_>,
    destructible: &DestructibleSet,
) -> bool {
    let Some(body) = world.bodies.get(body_handle) else {
        return false;
    };

    for &collider_handle in body.colliders() {
        for pair in world.narrow_phase.contact_pairs_with(collider_handle) {
            if !pair.has_any_active_contact {
                continue;
            }
            let other_collider = if pair.collider1 == collider_handle {
                pair.collider2
            } else {
                pair.collider1
            };
            if destructible.collider_node(other_collider).is_some() {
                return true;
            }
        }
    }
    false
}

fn motion_velocity_at_point(
    snapshot: &BodyMotionSnapshot,
    world_point: Point<Real>,
) -> Vector<Real> {
    let offset = world_point.coords - snapshot.position.translation.vector;
    snapshot.linvel + snapshot.angvel.cross(&offset)
}

fn relative_speed_along_force(force_world: Vector<Real>, relative_velocity: Vector<Real>) -> f32 {
    if force_world.norm_squared() <= 1.0e-12 {
        return relative_velocity.norm();
    }
    let direction = force_world.normalize();
    let projected = relative_velocity.dot(&direction).abs();
    if projected > 1.0e-5 {
        projected
    } else {
        relative_velocity.norm()
    }
}

fn flush_buffered_events<E: EventHandler + ?Sized>(
    user_events: &E,
    buffered_events: &BufferedPassEvents,
    world: &RapierWorldAccess<'_>,
) {
    for &event in &buffered_events.collisions {
        let contact_pair = match event {
            CollisionEvent::Started(collider1, collider2, _)
            | CollisionEvent::Stopped(collider1, collider2, _) => {
                world.narrow_phase.contact_pair(collider1, collider2)
            }
        };
        user_events.handle_collision_event(world.bodies, world.colliders, event, contact_pair);
    }

    for event in &buffered_events.contact_forces {
        let Some(contact_pair) = world
            .narrow_phase
            .contact_pair(event.collider1, event.collider2)
        else {
            continue;
        };
        user_events.handle_contact_force_event(
            event.dt,
            world.bodies,
            world.colliders,
            contact_pair,
            event.total_force_magnitude,
        );
    }
}
