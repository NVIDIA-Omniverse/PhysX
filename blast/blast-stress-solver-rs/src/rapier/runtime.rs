use std::collections::{HashMap, HashSet};
use std::ops::{Deref, DerefMut};
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc,
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

use super::body_tracker::SplitEditStats;
use super::destructible::{DestructibleSet, SplitCohort, StepResult};
use super::optimization::OptimizationResult;
use super::resimulation::BodySnapshots;

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
            impact_source_steps: 2,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct DestructionRuntimeOptions {
    pub contact_impacts: ContactImpactOptions,
    pub grace: GracePeriodOptions,
}

pub struct RapierWorldAccess<'a> {
    pub bodies: &'a mut RigidBodySet,
    pub colliders: &'a mut ColliderSet,
    pub island_manager: &'a mut IslandManager,
    pub narrow_phase: &'a NarrowPhase,
    pub impulse_joints: &'a mut ImpulseJointSet,
    pub multibody_joints: &'a mut MultibodyJointSet,
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

pub struct CombinedHooks<'a, H: PhysicsHooks + ?Sized> {
    runtime: RuntimeHooks,
    user_hooks: &'a H,
}

impl<H: PhysicsHooks + ?Sized> PhysicsHooks for CombinedHooks<'_, H> {
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
            .map(|cohort| cohort.target_bodies.clone())
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
    active_body_pairs: HashSet<BodyPairKey>,
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
    pre_step_motion: HashMap<RigidBodyHandle, BodyMotionSnapshot>,
    pending_grace_stats: Option<GraceStepStats>,
    result: FrameResult,
}

pub struct DestructionRuntime {
    destructible: DestructibleSet,
    options: DestructionRuntimeOptions,
    active_body_pairs: HashSet<BodyPairKey>,
    cooldowns: HashMap<ImpactCooldownKey, f32>,
    grace: GraceState,
    frame: Option<ActiveFrameState>,
}

impl DestructionRuntime {
    pub fn new(destructible: DestructibleSet, options: DestructionRuntimeOptions) -> Self {
        Self {
            destructible,
            options,
            active_body_pairs: HashSet::new(),
            cooldowns: HashMap::new(),
            grace: GraceState::new(options.grace),
            frame: None,
        }
    }

    pub fn options(&self) -> DestructionRuntimeOptions {
        self.options
    }

    pub fn set_options(&mut self, options: DestructionRuntimeOptions) {
        self.options = options;
        self.grace.set_options(options.grace);
    }

    pub fn into_inner(self) -> DestructibleSet {
        self.destructible
    }

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
            pre_step_motion: capture_body_motion(bodies),
            pending_grace_stats: None,
            result,
        });
    }

    pub fn begin_pass<'a, H>(&mut self, user_hooks: &'a H) -> CombinedHooks<'a, H>
    where
        H: PhysicsHooks + ?Sized,
    {
        let (runtime_hooks, stats) = self.grace.begin_step();
        if let Some(frame) = self.frame.as_mut() {
            frame.pending_grace_stats = Some(stats);
        }
        CombinedHooks {
            runtime: runtime_hooks,
            user_hooks,
        }
    }

    pub fn finish_pass(&mut self, world: &mut RapierWorldAccess<'_>) -> FrameDirective {
        let mut frame = self
            .frame
            .take()
            .expect("begin_frame must be called before finish_pass");
        frame.result.rapier_passes += 1;

        self.cooldowns.retain(|_, until| *until > frame.now_secs);
        let analysis = self.collect_pass_analysis(frame.dt, &frame.pre_step_motion, world);
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

        if !solver_result.split_cohorts.is_empty() {
            self.grace
                .register_split_cohorts(&solver_result.split_cohorts);
            self.grace.register_impact_fracture_pairs(
                &analysis.impact_sources,
                &analysis.impacted_bodies,
                &solver_result.split_cohorts,
            );
        }

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

        let fractured = solver_result.split_events > 0 || solver_result.new_bodies > 0;
        if fractured && frame.remaining_resim_passes > 0 {
            if let Some(snapshot) = frame.snapshot.as_ref() {
                let restore_started_at = Instant::now();
                snapshot.restore(world.bodies);
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

        frame.result.optimization = self.destructible.process_optimizations(
            frame.now_secs,
            world.bodies,
            world.colliders,
            world.island_manager,
            world.impulse_joints,
            world.multibody_joints,
        );

        self.active_body_pairs = analysis.active_body_pairs;
        self.frame = None;
        FrameDirective::Done(frame.result)
    }

    fn collect_pass_analysis(
        &self,
        dt: f32,
        pre_step_motion: &HashMap<RigidBodyHandle, BodyMotionSnapshot>,
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

            let Some(body_pair_key) = canonical_body_pair_key(body1, body2) else {
                continue;
            };
            analysis.active_body_pairs.insert(body_pair_key);

            let node1 = self.destructible.collider_node(pair.collider1);
            let node2 = self.destructible.collider_node(pair.collider2);
            if node1.is_none() && node2.is_none() {
                continue;
            }

            if !self.active_body_pairs.contains(&body_pair_key) {
                if let Some(node_index) = node1 {
                    if self.support_contact_for_pair(
                        node_index,
                        body1,
                        pair.collider2,
                        body2,
                        world,
                    ) {
                        analysis.support_contacts.insert(body1);
                    }
                }
                if let Some(node_index) = node2 {
                    if self.support_contact_for_pair(
                        node_index,
                        body2,
                        pair.collider1,
                        body1,
                        world,
                    ) {
                        analysis.support_contacts.insert(body2);
                    }
                }
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
