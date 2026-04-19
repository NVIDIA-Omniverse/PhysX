mod scene_pack;

use std::{
    any::Any,
    collections::{HashMap, HashSet},
    env,
    fs::OpenOptions,
    io::{BufWriter, Write},
    panic,
    path::PathBuf,
    sync::Arc,
    time::{Duration, Instant, SystemTime, UNIX_EPOCH},
};

use bevy::app::{AppExit, ScheduleRunnerPlugin};
use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::math::primitives::{Cuboid as BevyCuboid, Sphere};
use bevy::prelude::*;
use bevy::transform::TransformPlugin;
use bevy::window::PrimaryWindow;
use blast_stress_solver::rapier::{
    ContactImpactOptions, DebrisCleanupOptions, DebrisCollisionMode, DestructionRuntime,
    DestructionRuntimeOptions, FracturePolicy, FrameDirective, GracePeriodOptions,
    OptimizationMode, RapierWorldAccess, ResimulationOptions, SleepThresholdOptions,
    SmallBodyDampingOptions,
};
use blast_stress_solver::scenarios::{
    build_bridge_scenario, build_tower_scenario, build_wall_scenario, BridgeOptions, TowerOptions,
    WallOptions,
};
use blast_stress_solver::{ScenarioCollider, ScenarioDesc, SolverSettings, Vec3 as SolverVec3};
use rapier3d::prelude::*;
use scene_pack::{load_embedded_scene_pack, EmbeddedSceneKey, LoadedScenePack, SceneMeshAsset};

const GRAVITY: f32 = -9.81;
const CONTACT_FORCE_SCALE: f32 = 30.0;
const SPLASH_RADIUS: f32 = 2.0;
const DEFAULT_HEADLESS_FRAMES: u32 = 180;
const DEFAULT_PROJECTILE_TTL: f32 = 6.0;
const DEFAULT_MAX_PROJECTILE_MASS: f32 = 1_000_000_000.0;

const BASE_COMPRESSION_ELASTIC: f32 = 0.0009;
const BASE_COMPRESSION_FATAL: f32 = 0.0027;
const BASE_TENSION_ELASTIC: f32 = 0.0009;
const BASE_TENSION_FATAL: f32 = 0.0027;
const BASE_SHEAR_ELASTIC: f32 = 0.0012;
const BASE_SHEAR_FATAL: f32 = 0.0036;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum DemoScenarioKind {
    Wall,
    Tower,
    Bridge,
    FracturedWall,
    FracturedTower,
    FracturedBridge,
    BrickBuilding,
}

impl DemoScenarioKind {
    fn slug(self) -> &'static str {
        match self {
            Self::Wall => "wall",
            Self::Tower => "tower",
            Self::Bridge => "bridge",
            Self::FracturedWall => "fractured-wall",
            Self::FracturedTower => "fractured-tower",
            Self::FracturedBridge => "fractured-bridge",
            Self::BrickBuilding => "brick-building",
        }
    }
}

#[derive(Clone, Debug)]
struct DemoConfig {
    title: String,
    scenario: ScenarioDesc,
    node_meshes: Arc<[SceneMeshAsset]>,
    projectile_radius: f32,
    projectile_mass: f32,
    projectile_speed: f32,
    projectile_ttl: f32,
    gravity: f32,
    material_scale: f32,
    skip_single_bodies: bool,
    camera_target: Vec3,
    camera_distance: f32,
    policy: FracturePolicy,
    resimulation: ResimulationOptions,
    sleep_thresholds: SleepThresholdOptions,
    small_body_damping: SmallBodyDampingOptions,
    debris_cleanup: DebrisCleanupOptions,
    debris_collision_mode: DebrisCollisionMode,
}

#[derive(Resource, Clone)]
struct DemoRuntimeToggles {
    rapier_only: bool,
    contact_force_injection_enabled: bool,
    gizmos_enabled: bool,
    projectile_ccd_enabled: bool,
    body_ccd_enabled: bool,
    show_meshes: bool,
    sibling_grace_steps: u32,
    projectile_fracture_grace_steps: u32,
    split_child_recentering_enabled: bool,
    split_child_velocity_fit_enabled: bool,
    projectile_trace_enabled: bool,
    heavy_frame_threshold_ms: f32,
    topology_body_delta_threshold: usize,
    config_summary: String,
}

impl DemoRuntimeToggles {
    fn from_env(scenario: DemoScenarioKind, config: &mut DemoConfig) -> Self {
        let rapier_only = env_flag("BLAST_STRESS_DEMO_RAPIER_ONLY").unwrap_or(false);
        if rapier_only {
            config.resimulation.enabled = false;
            config.resimulation.max_passes = 0;
            config.sleep_thresholds.mode = OptimizationMode::Off;
            config.small_body_damping.mode = OptimizationMode::Off;
            config.debris_cleanup.mode = OptimizationMode::Off;
            config.debris_collision_mode = DebrisCollisionMode::All;
        }
        let show_meshes = mesh_visuals_enabled();
        let resimulation_enabled =
            env_flag("BLAST_STRESS_DEMO_RESIM").unwrap_or(config.resimulation.enabled);
        let max_resimulation_passes = env_usize("BLAST_STRESS_DEMO_MAX_RESIM_PASSES")
            .unwrap_or(config.resimulation.max_passes);
        config.resimulation.enabled = resimulation_enabled;
        config.resimulation.max_passes = max_resimulation_passes;

        if let Some(value) = env_i32("BLAST_STRESS_DEMO_MAX_FRACTURES_PER_FRAME") {
            config.policy.max_fractures_per_frame = value;
        }
        if let Some(value) = env_i32("BLAST_STRESS_DEMO_MAX_NEW_BODIES_PER_FRAME") {
            config.policy.max_new_bodies_per_frame = value;
        }
        if let Some(value) = env_i32("BLAST_STRESS_DEMO_MAX_COLLIDER_MIGRATIONS_PER_FRAME") {
            config.policy.max_collider_migrations_per_frame = value;
        }
        if let Some(value) = env_i32("BLAST_STRESS_DEMO_MAX_DYNAMIC_BODIES") {
            config.policy.max_dynamic_bodies = value;
        }
        if let Some(value) = env_usize("BLAST_STRESS_DEMO_MIN_CHILD_NODE_COUNT") {
            config.policy.min_child_node_count = value as u32;
        }
        if let Some(value) = env_flag("BLAST_STRESS_DEMO_IDLE_SKIP") {
            config.policy.idle_skip = value;
        }
        if let Some(value) = env_flag("BLAST_STRESS_DEMO_APPLY_EXCESS_FORCES") {
            config.policy.apply_excess_forces = value;
        }

        let sleep_thresholds_enabled = env_flag("BLAST_STRESS_DEMO_SLEEP_OPT")
            .unwrap_or(config.sleep_thresholds.mode != OptimizationMode::Off);
        if sleep_thresholds_enabled {
            if config.sleep_thresholds.mode == OptimizationMode::Off {
                config.sleep_thresholds.mode = OptimizationMode::Always;
            }
        } else {
            config.sleep_thresholds.mode = OptimizationMode::Off;
        }
        if let Some(value) = env_f32("BLAST_STRESS_DEMO_SLEEP_LINEAR_THRESHOLD") {
            config.sleep_thresholds.linear_threshold = value;
        }
        if let Some(value) = env_f32("BLAST_STRESS_DEMO_SLEEP_ANGULAR_THRESHOLD") {
            config.sleep_thresholds.angular_threshold = value;
        }

        let small_body_damping_enabled = env_flag("BLAST_STRESS_DEMO_SMALL_BODY_DAMPING")
            .unwrap_or(config.small_body_damping.mode != OptimizationMode::Off);
        if small_body_damping_enabled {
            if config.small_body_damping.mode == OptimizationMode::Off {
                config.small_body_damping.mode = OptimizationMode::Always;
            }
        } else {
            config.small_body_damping.mode = OptimizationMode::Off;
        }
        if let Some(value) = env_usize("BLAST_STRESS_DEMO_SMALL_BODY_COLLIDER_THRESHOLD") {
            config.small_body_damping.collider_count_threshold = value;
        }
        if let Some(value) = env_f32("BLAST_STRESS_DEMO_SMALL_BODY_LINEAR_DAMPING") {
            config.small_body_damping.min_linear_damping = value;
        }
        if let Some(value) = env_f32("BLAST_STRESS_DEMO_SMALL_BODY_ANGULAR_DAMPING") {
            config.small_body_damping.min_angular_damping = value;
        }

        let debris_cleanup_enabled = env_flag("BLAST_STRESS_DEMO_DEBRIS_CLEANUP")
            .unwrap_or(config.debris_cleanup.mode != OptimizationMode::Off);
        if debris_cleanup_enabled {
            if config.debris_cleanup.mode == OptimizationMode::Off {
                config.debris_cleanup.mode = OptimizationMode::Always;
            }
        } else {
            config.debris_cleanup.mode = OptimizationMode::Off;
        }
        if let Some(value) = env_f32("BLAST_STRESS_DEMO_DEBRIS_TTL_SECS") {
            config.debris_cleanup.debris_ttl_secs = value;
        }
        if let Some(value) = env_usize("BLAST_STRESS_DEMO_DEBRIS_MAX_COLLIDERS") {
            config.debris_cleanup.max_colliders_for_debris = value;
        }
        if let Some(value) = env_debris_collision_mode("BLAST_STRESS_DEMO_DEBRIS_COLLISION_MODE") {
            config.debris_collision_mode = value;
        }

        let sibling_grace_steps = env_usize("BLAST_STRESS_DEMO_SIBLING_GRACE_STEPS")
            .map(|value| value as u32)
            .unwrap_or(0);
        let projectile_fracture_grace_steps =
            env_usize("BLAST_STRESS_DEMO_PROJECTILE_FRACTURE_GRACE_STEPS")
                .map(|value| value as u32)
                .unwrap_or(if rapier_only { 0 } else { 1 });
        let split_child_recentering_enabled =
            env_flag("BLAST_STRESS_DEMO_SPLIT_RECENTER_CHILDREN").unwrap_or(true);
        let split_child_velocity_fit_enabled =
            env_flag("BLAST_STRESS_DEMO_SPLIT_VELOCITY_FIT").unwrap_or(true);
        let projectile_trace_enabled =
            env_flag("BLAST_STRESS_DEMO_PROJECTILE_TRACE").unwrap_or(false);
        let heavy_frame_threshold_ms = env_f32("BLAST_STRESS_DEMO_HEAVY_FRAME_MS").unwrap_or(16.0);
        let topology_body_delta_threshold =
            env_usize("BLAST_STRESS_DEMO_TOPOLOGY_BODY_DELTA").unwrap_or(1);
        let headless_shot_script = env::var("BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT")
            .ok()
            .filter(|value| !value.trim().is_empty())
            .unwrap_or_else(|| "none".to_string());

        let config_summary = format!(
            "scenario={} meshes={} gizmos={} rapier_only={} resim={} max_resim={} contact_injection={} projectile_ccd={} body_ccd={} policy[max_fractures={} max_new_bodies={} max_collider_migrations={} max_dynamic_bodies={} min_child_nodes={} idle_skip={} apply_excess_forces={}] sleep[enabled={} mode={} linear={:.3} angular={:.3}] small_body_damping[enabled={} mode={} colliders={} linear={:.2} angular={:.2}] debris[collision_mode={} cleanup_enabled={} cleanup_mode={} ttl={:.2} max_colliders={}] handoff[recenter_children={} velocity_fit={}] debug[sibling_grace_steps={} projectile_fracture_grace_steps={} projectile_trace={} heavy_frame_ms={:.2} topology_body_delta={} headless_shot_script={}]",
            scenario.slug(),
            flag_bit(show_meshes),
            flag_bit(env_flag("BLAST_STRESS_DEMO_GIZMOS").unwrap_or(true)),
            flag_bit(rapier_only),
            flag_bit(resimulation_enabled),
            max_resimulation_passes,
            flag_bit(if rapier_only {
                false
            } else {
                env_flag("BLAST_STRESS_DEMO_CONTACT_FORCE_INJECTION").unwrap_or(true)
            }),
            flag_bit(env_flag("BLAST_STRESS_DEMO_PROJECTILE_CCD").unwrap_or(true)),
            flag_bit(env_flag("BLAST_STRESS_DEMO_BODY_CCD").unwrap_or(false)),
            config.policy.max_fractures_per_frame,
            config.policy.max_new_bodies_per_frame,
            config.policy.max_collider_migrations_per_frame,
            config.policy.max_dynamic_bodies,
            config.policy.min_child_node_count,
            flag_bit(config.policy.idle_skip),
            flag_bit(config.policy.apply_excess_forces),
            flag_bit(sleep_thresholds_enabled),
            optimization_mode_label(config.sleep_thresholds.mode),
            config.sleep_thresholds.linear_threshold,
            config.sleep_thresholds.angular_threshold,
            flag_bit(small_body_damping_enabled),
            optimization_mode_label(config.small_body_damping.mode),
            config.small_body_damping.collider_count_threshold,
            config.small_body_damping.min_linear_damping,
            config.small_body_damping.min_angular_damping,
            debris_collision_mode_label(config.debris_collision_mode),
            flag_bit(debris_cleanup_enabled),
            optimization_mode_label(config.debris_cleanup.mode),
            config.debris_cleanup.debris_ttl_secs,
            config.debris_cleanup.max_colliders_for_debris,
            flag_bit(split_child_recentering_enabled),
            flag_bit(split_child_velocity_fit_enabled),
            sibling_grace_steps,
            projectile_fracture_grace_steps,
            flag_bit(projectile_trace_enabled),
            heavy_frame_threshold_ms,
            topology_body_delta_threshold,
            headless_shot_script,
        );

        let contact_force_injection_enabled = if rapier_only {
            false
        } else {
            env_flag("BLAST_STRESS_DEMO_CONTACT_FORCE_INJECTION").unwrap_or(true)
        };
        let gizmos_enabled = env_flag("BLAST_STRESS_DEMO_GIZMOS").unwrap_or(true);
        let projectile_ccd_enabled = env_flag("BLAST_STRESS_DEMO_PROJECTILE_CCD").unwrap_or(true);
        let body_ccd_enabled = env_flag("BLAST_STRESS_DEMO_BODY_CCD").unwrap_or(false);

        Self {
            rapier_only,
            contact_force_injection_enabled,
            gizmos_enabled,
            projectile_ccd_enabled,
            body_ccd_enabled,
            show_meshes,
            sibling_grace_steps,
            projectile_fracture_grace_steps,
            split_child_recentering_enabled,
            split_child_velocity_fit_enabled,
            projectile_trace_enabled,
            heavy_frame_threshold_ms,
            topology_body_delta_threshold,
            config_summary,
        }
    }

    fn summary(&self) -> String {
        self.config_summary.clone()
    }
}

#[derive(Resource, Clone)]
struct DemoInfo {
    title: String,
    subtitle: String,
    camera_target: Vec3,
    camera_distance: f32,
}

#[derive(Resource, Clone, Copy)]
struct DemoScenario {
    kind: DemoScenarioKind,
}

#[derive(Resource, Clone, Copy)]
struct RunMode {
    headless: bool,
}

#[derive(Resource)]
struct HeadlessRunBudget {
    frames_remaining: u32,
}

#[derive(Resource)]
struct CameraOrbit {
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: Vec3,
}

impl CameraOrbit {
    fn from_info(info: &DemoInfo) -> Self {
        Self {
            yaw: -std::f32::consts::FRAC_PI_2,
            pitch: 0.18,
            distance: info.camera_distance,
            target: info.camera_target,
        }
    }
}

#[derive(Clone, Debug)]
struct HeadlessShot {
    frame: u32,
    label: String,
    origin: Vec3,
    target: Vec3,
    mass: f32,
    speed: f32,
    ttl: f32,
    exit_distance: Option<f32>,
    ballistic_arc: bool,
}

impl HeadlessShot {
    fn launch_direction(&self) -> Vec3 {
        if !self.ballistic_arc {
            return (self.target - self.origin).normalize_or_zero();
        }
        let delta = self.target - self.origin;
        let horizontal = Vec3::new(delta.x, 0.0, delta.z);
        let horizontal_distance = horizontal.length();
        if horizontal_distance <= f32::EPSILON || self.speed <= f32::EPSILON {
            return delta.normalize_or_zero();
        }

        let gravity = GRAVITY.abs();
        let speed_sq = self.speed * self.speed;
        let discriminant = speed_sq * speed_sq
            - gravity
                * (gravity * horizontal_distance * horizontal_distance + 2.0 * delta.y * speed_sq);
        if discriminant <= 0.0 {
            return delta.normalize_or_zero();
        }

        let sqrt_discriminant = discriminant.sqrt();
        for tan_theta in [
            (speed_sq - sqrt_discriminant) / (gravity * horizontal_distance),
            (speed_sq + sqrt_discriminant) / (gravity * horizontal_distance),
        ] {
            if tan_theta.is_finite() {
                let cos_theta = 1.0 / (1.0 + tan_theta * tan_theta).sqrt();
                let sin_theta = tan_theta * cos_theta;
                let horizontal_dir = horizontal / horizontal_distance;
                let launch = horizontal_dir * cos_theta + Vec3::Y * sin_theta;
                if launch.is_finite() {
                    return launch.normalize_or_zero();
                }
            }
        }

        delta.normalize_or_zero()
    }
}

#[derive(Resource, Clone, Debug)]
struct HeadlessShotPlan {
    script_name: String,
    shots: Vec<HeadlessShot>,
    next_index: usize,
    fired: usize,
}

impl HeadlessShotPlan {
    fn total_shots(&self) -> usize {
        self.shots.len()
    }

    fn due_shots(&mut self, frame: u64) -> Vec<HeadlessShot> {
        let mut due = Vec::new();
        while let Some(shot) = self.shots.get(self.next_index) {
            if u64::from(shot.frame) > frame {
                break;
            }
            due.push(shot.clone());
            self.next_index += 1;
            self.fired += 1;
        }
        due
    }
}

#[derive(Resource)]
struct ProjectileFireSettings {
    mass: f32,
    max_mass: f32,
}

impl ProjectileFireSettings {
    fn new(initial_mass: f32) -> Self {
        Self {
            mass: initial_mass.max(1.0),
            max_mass: env_f32("BLAST_STRESS_DEMO_PROJECTILE_MAX_MASS")
                .unwrap_or(DEFAULT_MAX_PROJECTILE_MASS)
                .max(1.0),
        }
    }

    fn increase_mass(&mut self) {
        self.mass = (self.mass * 2.0).min(self.max_mass);
    }

    fn decrease_mass(&mut self) {
        self.mass = (self.mass * 0.5).max(1.0);
    }
}

#[derive(Resource)]
struct PerfLogWriter {
    path: PathBuf,
    writer: BufWriter<std::fs::File>,
}

impl PerfLogWriter {
    fn create() -> std::io::Result<Self> {
        let timestamp_ms = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .unwrap_or_default()
            .as_millis();
        let path = env::temp_dir().join(format!(
            "blast-stress-demo-perf-{}-{}.log",
            std::process::id(),
            timestamp_ms
        ));
        let file = OpenOptions::new().create(true).append(true).open(&path)?;
        Ok(Self {
            path,
            writer: BufWriter::with_capacity(64 * 1024, file),
        })
    }

    fn path(&self) -> &PathBuf {
        &self.path
    }

    fn write_line(&mut self, line: &str) {
        let _ = self.writer.write_all(line.as_bytes());
        let _ = self.writer.write_all(b"\n");
    }

    fn flush(&mut self) {
        let _ = self.writer.flush();
    }
}

#[derive(Component)]
struct ChunkVisual {
    node_index: u32,
    gizmo_shape: ChunkGizmoShape,
    is_support: bool,
}

#[derive(Clone)]
enum ChunkGizmoShape {
    Box { size: Vec3 },
    TriEdges { edges: Arc<[(Vec3, Vec3)]> },
}

#[derive(Component)]
struct ProjectileVisual {
    radius: f32,
}

#[derive(Component)]
struct ChunkTint {
    color: Color,
}

#[derive(Component)]
struct ChunkMaterial {
    handle: Handle<StandardMaterial>,
}

#[derive(Component)]
struct MainCamera;

#[derive(Component)]
struct HudText;

#[derive(Clone, Copy, Debug, Default)]
struct SmoothedMetric {
    last_ms: f32,
    avg_ms: f32,
    max_ms: f32,
}

impl SmoothedMetric {
    fn observe_ms(&mut self, ms: f32) {
        self.last_ms = ms;
        self.avg_ms = if self.avg_ms == 0.0 {
            ms
        } else {
            self.avg_ms * 0.9 + ms * 0.1
        };
        self.max_ms = self.max_ms.max(ms);
    }
}

#[derive(Clone, Copy, Debug, Default)]
struct FrameBreakdown {
    physics_ms: f32,
    rapier_ms: f32,
    collision_events_ms: f32,
    contact_forces_ms: f32,
    solver_ms: f32,
    split_sanitize_ms: f32,
    split_estimate_ms: f32,
    split_plan_ms: f32,
    split_apply_ms: f32,
    split_child_pose_ms: f32,
    split_velocity_fit_ms: f32,
    split_sleep_init_ms: f32,
    split_body_create_ms: f32,
    split_collider_move_ms: f32,
    split_collider_insert_ms: f32,
    split_body_retire_ms: f32,
    resim_restore_ms: f32,
    resim_snapshot_ms: f32,
    optimization_ms: f32,
    projectile_cleanup_ms: f32,
    sync_visuals_ms: f32,
    gizmo_ms: f32,
    hud_ms: f32,
    rapier_passes: u32,
    collision_events: usize,
    contact_events: usize,
    fractures: usize,
    split_events: usize,
    split_cohorts: usize,
    split_cohort_bodies: usize,
    reused_bodies: usize,
    recycled_bodies: usize,
    new_bodies: usize,
    retired_bodies: usize,
    body_type_flips: usize,
    moved_colliders: usize,
    inserted_colliders: usize,
    removed_colliders: usize,
    removed_nodes: usize,
    removed_projectiles: usize,
    world_bodies: usize,
    destructible_bodies: usize,
    support_bodies: usize,
    dynamic_bodies: usize,
    awake_dynamic_bodies: usize,
    sleeping_dynamic_bodies: usize,
    world_colliders: usize,
    destructible_colliders: usize,
    projectile_count: usize,
    ccd_bodies: usize,
    contact_pairs: usize,
    active_contact_pairs: usize,
    contact_manifolds: usize,
    sibling_grace_pairs: usize,
    sibling_grace_filtered_pairs: usize,
    pending_split_events: usize,
    pending_new_bodies: usize,
    pending_collider_migrations: usize,
}

#[derive(Default)]
struct MedianWindow {
    frame_ms: Vec<f32>,
    physics_ms: Vec<f32>,
    rapier_ms: Vec<f32>,
    collision_events_ms: Vec<f32>,
    contact_forces_ms: Vec<f32>,
    solver_ms: Vec<f32>,
    split_sanitize_ms: Vec<f32>,
    split_estimate_ms: Vec<f32>,
    split_plan_ms: Vec<f32>,
    split_apply_ms: Vec<f32>,
    split_child_pose_ms: Vec<f32>,
    split_velocity_fit_ms: Vec<f32>,
    split_sleep_init_ms: Vec<f32>,
    split_body_create_ms: Vec<f32>,
    split_collider_move_ms: Vec<f32>,
    split_collider_insert_ms: Vec<f32>,
    split_body_retire_ms: Vec<f32>,
    resim_restore_ms: Vec<f32>,
    resim_snapshot_ms: Vec<f32>,
    optimization_ms: Vec<f32>,
    projectile_cleanup_ms: Vec<f32>,
    render_prep_ms: Vec<f32>,
    sync_visuals_ms: Vec<f32>,
    gizmo_ms: Vec<f32>,
    hud_ms: Vec<f32>,
    other_cpu_ms: Vec<f32>,
    rapier_passes: Vec<f32>,
    collision_events: Vec<f32>,
    contact_events: Vec<f32>,
    fractures: Vec<f32>,
    split_events: Vec<f32>,
    split_cohorts: Vec<f32>,
    split_cohort_bodies: Vec<f32>,
    reused_bodies: Vec<f32>,
    recycled_bodies: Vec<f32>,
    new_bodies: Vec<f32>,
    retired_bodies: Vec<f32>,
    body_type_flips: Vec<f32>,
    moved_colliders: Vec<f32>,
    inserted_colliders: Vec<f32>,
    removed_colliders: Vec<f32>,
    removed_nodes: Vec<f32>,
    removed_projectiles: Vec<f32>,
    world_bodies: Vec<f32>,
    destructible_bodies: Vec<f32>,
    support_bodies: Vec<f32>,
    dynamic_bodies: Vec<f32>,
    awake_dynamic_bodies: Vec<f32>,
    sleeping_dynamic_bodies: Vec<f32>,
    world_colliders: Vec<f32>,
    destructible_colliders: Vec<f32>,
    projectile_count: Vec<f32>,
    ccd_bodies: Vec<f32>,
    contact_pairs: Vec<f32>,
    active_contact_pairs: Vec<f32>,
    contact_manifolds: Vec<f32>,
    sibling_grace_pairs: Vec<f32>,
    sibling_grace_filtered_pairs: Vec<f32>,
    pending_split_events: Vec<f32>,
    pending_new_bodies: Vec<f32>,
    pending_collider_migrations: Vec<f32>,
}

#[derive(Resource, Default)]
struct DebugProfiler {
    frame_started_at: Option<Instant>,
    log_window_started_at: Option<Instant>,
    frame_index: u64,
    current: FrameBreakdown,
    median_window: MedianWindow,
    pending_event_lines: Vec<String>,
    frame_cpu: SmoothedMetric,
    physics: SmoothedMetric,
    rapier: SmoothedMetric,
    collision_events: SmoothedMetric,
    contact_forces: SmoothedMetric,
    solver: SmoothedMetric,
    split_sanitize: SmoothedMetric,
    split_estimate: SmoothedMetric,
    split_plan: SmoothedMetric,
    split_apply: SmoothedMetric,
    split_child_pose: SmoothedMetric,
    split_velocity_fit: SmoothedMetric,
    split_sleep_init: SmoothedMetric,
    split_body_create: SmoothedMetric,
    split_collider_move: SmoothedMetric,
    split_collider_insert: SmoothedMetric,
    split_body_retire: SmoothedMetric,
    resim_restore: SmoothedMetric,
    resim_snapshot: SmoothedMetric,
    optimization: SmoothedMetric,
    projectile_cleanup: SmoothedMetric,
    sync_visuals: SmoothedMetric,
    gizmo: SmoothedMetric,
    hud: SmoothedMetric,
    render_prep: SmoothedMetric,
    other_cpu: SmoothedMetric,
    avg_rapier_passes: f32,
    last_rapier_passes: u32,
    last_collision_events: usize,
    last_contact_events: usize,
    last_fractures: usize,
    last_split_events: usize,
    last_reused_bodies: usize,
    last_recycled_bodies: usize,
    last_new_bodies: usize,
    last_retired_bodies: usize,
    last_body_type_flips: usize,
    last_moved_colliders: usize,
    last_inserted_colliders: usize,
    last_removed_colliders: usize,
    last_removed_nodes: usize,
    last_removed_projectiles: usize,
    last_world_bodies: usize,
    last_destructible_bodies: usize,
    last_support_bodies: usize,
    last_dynamic_bodies: usize,
    last_awake_dynamic_bodies: usize,
    last_sleeping_dynamic_bodies: usize,
    last_world_colliders: usize,
    last_destructible_colliders: usize,
    last_projectile_count: usize,
    last_ccd_bodies: usize,
    last_contact_pairs: usize,
    last_active_contact_pairs: usize,
    last_contact_manifolds: usize,
    last_pending_split_events: usize,
    last_pending_new_bodies: usize,
    last_pending_collider_migrations: usize,
    peak_frame_ms: f32,
    peak_physics_ms: f32,
    peak_rapier_ms: f32,
    peak_solver_ms: f32,
    peak_split_plan_ms: f32,
    peak_split_apply_ms: f32,
    peak_split_collider_move_ms: f32,
    peak_fracture_frame_ms: f32,
    peak_fracture_physics_ms: f32,
    peak_fracture_rapier_ms: f32,
    peak_fracture_solver_ms: f32,
    peak_fracture_split_plan_ms: f32,
    peak_fracture_split_apply_ms: f32,
    peak_frame_index: u64,
    peak_physics_frame_index: u64,
    peak_rapier_frame_index: u64,
    peak_solver_frame_index: u64,
    peak_split_plan_frame_index: u64,
    peak_split_apply_frame_index: u64,
    peak_split_collider_move_frame_index: u64,
    peak_fracture_frame_index: u64,
    peak_fracture_physics_frame_index: u64,
    peak_fracture_rapier_frame_index: u64,
    peak_fracture_solver_frame_index: u64,
    peak_fracture_split_plan_frame_index: u64,
    peak_fracture_split_apply_frame_index: u64,
    peak_world_bodies: usize,
    peak_world_bodies_frame_index: u64,
    peak_dynamic_bodies: usize,
    peak_dynamic_bodies_frame_index: u64,
    peak_awake_dynamic_bodies: usize,
    peak_awake_dynamic_bodies_frame_index: u64,
    peak_active_contact_pairs: usize,
    peak_active_contact_pairs_frame_index: u64,
    peak_contact_manifolds: usize,
    peak_contact_manifolds_frame_index: u64,
    peak_sibling_grace_filtered_pairs: usize,
    peak_sibling_grace_filtered_pairs_frame_index: u64,
    total_fractures: u64,
    total_split_events: u64,
    total_new_bodies: u64,
    total_moved_colliders: u64,
    total_collision_events: u64,
    total_contact_events: u64,
    first_fracture_frame_index: Option<u64>,
    heavy_frame_threshold_ms: f32,
    topology_body_delta_threshold: usize,
    config_summary: String,
}

impl DebugProfiler {
    fn begin_frame(&mut self) {
        self.frame_started_at = Some(Instant::now());
        self.current = FrameBreakdown::default();
        self.frame_index = self.frame_index.saturating_add(1);
    }

    fn finish_frame(&mut self) -> Option<String> {
        let Some(started_at) = self.frame_started_at.take() else {
            return None;
        };
        let now = Instant::now();
        let log_window_started_at = self.log_window_started_at.get_or_insert(now);

        let frame_ms = started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
        let render_prep_ms =
            self.current.sync_visuals_ms + self.current.gizmo_ms + self.current.hud_ms;
        let known_ms = self.current.physics_ms + render_prep_ms;
        let other_cpu_ms = (frame_ms - known_ms).max(0.0);

        self.frame_cpu.observe_ms(frame_ms);
        self.physics.observe_ms(self.current.physics_ms);
        self.rapier.observe_ms(self.current.rapier_ms);
        self.collision_events
            .observe_ms(self.current.collision_events_ms);
        self.contact_forces
            .observe_ms(self.current.contact_forces_ms);
        self.solver.observe_ms(self.current.solver_ms);
        self.split_sanitize
            .observe_ms(self.current.split_sanitize_ms);
        self.split_estimate
            .observe_ms(self.current.split_estimate_ms);
        self.split_plan.observe_ms(self.current.split_plan_ms);
        self.split_apply.observe_ms(self.current.split_apply_ms);
        self.split_child_pose
            .observe_ms(self.current.split_child_pose_ms);
        self.split_velocity_fit
            .observe_ms(self.current.split_velocity_fit_ms);
        self.split_sleep_init
            .observe_ms(self.current.split_sleep_init_ms);
        self.split_body_create
            .observe_ms(self.current.split_body_create_ms);
        self.split_collider_move
            .observe_ms(self.current.split_collider_move_ms);
        self.split_collider_insert
            .observe_ms(self.current.split_collider_insert_ms);
        self.split_body_retire
            .observe_ms(self.current.split_body_retire_ms);
        self.resim_restore.observe_ms(self.current.resim_restore_ms);
        self.resim_snapshot
            .observe_ms(self.current.resim_snapshot_ms);
        self.optimization.observe_ms(self.current.optimization_ms);
        self.projectile_cleanup
            .observe_ms(self.current.projectile_cleanup_ms);
        self.sync_visuals.observe_ms(self.current.sync_visuals_ms);
        self.gizmo.observe_ms(self.current.gizmo_ms);
        self.hud.observe_ms(self.current.hud_ms);
        self.render_prep.observe_ms(render_prep_ms);
        self.other_cpu.observe_ms(other_cpu_ms);

        let prev_world_bodies = self.last_world_bodies;
        let prev_dynamic_bodies = self.last_dynamic_bodies;
        let prev_awake_dynamic_bodies = self.last_awake_dynamic_bodies;
        let prev_contact_pairs = self.last_contact_pairs;
        let prev_active_contact_pairs = self.last_active_contact_pairs;
        let prev_contact_manifolds = self.last_contact_manifolds;
        let world_bodies_delta = self.current.world_bodies as isize - prev_world_bodies as isize;
        let dynamic_bodies_delta =
            self.current.dynamic_bodies as isize - prev_dynamic_bodies as isize;
        let awake_dynamic_bodies_delta =
            self.current.awake_dynamic_bodies as isize - prev_awake_dynamic_bodies as isize;
        let contact_pairs_delta = self.current.contact_pairs as isize - prev_contact_pairs as isize;
        let active_contact_pairs_delta =
            self.current.active_contact_pairs as isize - prev_active_contact_pairs as isize;
        let contact_manifolds_delta =
            self.current.contact_manifolds as isize - prev_contact_manifolds as isize;

        self.last_rapier_passes = self.current.rapier_passes;
        self.last_collision_events = self.current.collision_events;
        self.last_contact_events = self.current.contact_events;
        self.last_fractures = self.current.fractures;
        self.last_split_events = self.current.split_events;
        self.last_reused_bodies = self.current.reused_bodies;
        self.last_recycled_bodies = self.current.recycled_bodies;
        self.last_new_bodies = self.current.new_bodies;
        self.last_retired_bodies = self.current.retired_bodies;
        self.last_body_type_flips = self.current.body_type_flips;
        self.last_moved_colliders = self.current.moved_colliders;
        self.last_inserted_colliders = self.current.inserted_colliders;
        self.last_removed_colliders = self.current.removed_colliders;
        self.last_removed_nodes = self.current.removed_nodes;
        self.last_removed_projectiles = self.current.removed_projectiles;
        self.last_world_bodies = self.current.world_bodies;
        self.last_destructible_bodies = self.current.destructible_bodies;
        self.last_support_bodies = self.current.support_bodies;
        self.last_dynamic_bodies = self.current.dynamic_bodies;
        self.last_awake_dynamic_bodies = self.current.awake_dynamic_bodies;
        self.last_sleeping_dynamic_bodies = self.current.sleeping_dynamic_bodies;
        self.last_world_colliders = self.current.world_colliders;
        self.last_destructible_colliders = self.current.destructible_colliders;
        self.last_projectile_count = self.current.projectile_count;
        self.last_ccd_bodies = self.current.ccd_bodies;
        self.last_contact_pairs = self.current.contact_pairs;
        self.last_active_contact_pairs = self.current.active_contact_pairs;
        self.last_contact_manifolds = self.current.contact_manifolds;
        self.last_pending_split_events = self.current.pending_split_events;
        self.last_pending_new_bodies = self.current.pending_new_bodies;
        self.last_pending_collider_migrations = self.current.pending_collider_migrations;
        if frame_ms >= self.peak_frame_ms {
            self.peak_frame_ms = frame_ms;
            self.peak_frame_index = self.frame_index;
        }
        if self.current.physics_ms >= self.peak_physics_ms {
            self.peak_physics_ms = self.current.physics_ms;
            self.peak_physics_frame_index = self.frame_index;
        }
        if self.current.rapier_ms >= self.peak_rapier_ms {
            self.peak_rapier_ms = self.current.rapier_ms;
            self.peak_rapier_frame_index = self.frame_index;
        }
        if self.current.solver_ms >= self.peak_solver_ms {
            self.peak_solver_ms = self.current.solver_ms;
            self.peak_solver_frame_index = self.frame_index;
        }
        if self.current.split_plan_ms >= self.peak_split_plan_ms {
            self.peak_split_plan_ms = self.current.split_plan_ms;
            self.peak_split_plan_frame_index = self.frame_index;
        }
        if self.current.split_apply_ms >= self.peak_split_apply_ms {
            self.peak_split_apply_ms = self.current.split_apply_ms;
            self.peak_split_apply_frame_index = self.frame_index;
        }
        if self.current.split_collider_move_ms >= self.peak_split_collider_move_ms {
            self.peak_split_collider_move_ms = self.current.split_collider_move_ms;
            self.peak_split_collider_move_frame_index = self.frame_index;
        }
        if self.current.world_bodies >= self.peak_world_bodies {
            self.peak_world_bodies = self.current.world_bodies;
            self.peak_world_bodies_frame_index = self.frame_index;
        }
        if self.current.dynamic_bodies >= self.peak_dynamic_bodies {
            self.peak_dynamic_bodies = self.current.dynamic_bodies;
            self.peak_dynamic_bodies_frame_index = self.frame_index;
        }
        if self.current.awake_dynamic_bodies >= self.peak_awake_dynamic_bodies {
            self.peak_awake_dynamic_bodies = self.current.awake_dynamic_bodies;
            self.peak_awake_dynamic_bodies_frame_index = self.frame_index;
        }
        if self.current.active_contact_pairs >= self.peak_active_contact_pairs {
            self.peak_active_contact_pairs = self.current.active_contact_pairs;
            self.peak_active_contact_pairs_frame_index = self.frame_index;
        }
        if self.current.contact_manifolds >= self.peak_contact_manifolds {
            self.peak_contact_manifolds = self.current.contact_manifolds;
            self.peak_contact_manifolds_frame_index = self.frame_index;
        }
        if self.current.sibling_grace_filtered_pairs >= self.peak_sibling_grace_filtered_pairs {
            self.peak_sibling_grace_filtered_pairs = self.current.sibling_grace_filtered_pairs;
            self.peak_sibling_grace_filtered_pairs_frame_index = self.frame_index;
        }
        self.total_fractures = self
            .total_fractures
            .saturating_add(self.current.fractures as u64);
        self.total_split_events = self
            .total_split_events
            .saturating_add(self.current.split_events as u64);
        self.total_new_bodies = self
            .total_new_bodies
            .saturating_add(self.current.new_bodies as u64);
        self.total_moved_colliders = self
            .total_moved_colliders
            .saturating_add(self.current.moved_colliders as u64);
        self.total_collision_events = self
            .total_collision_events
            .saturating_add(self.current.collision_events as u64);
        self.total_contact_events = self
            .total_contact_events
            .saturating_add(self.current.contact_events as u64);

        let heavy_frame = frame_ms >= self.heavy_frame_threshold_ms
            || self.current.physics_ms >= self.heavy_frame_threshold_ms
            || self.current.rapier_ms >= self.heavy_frame_threshold_ms;

        if heavy_frame {
            self.pending_event_lines.push(format!(
                "[heavy-frame] frame={} frame_ms={:.3} physics_ms={:.3} rapier_ms={:.3} solver_ms={:.3} split_plan_ms={:.3} split_apply_ms={:.3} split_collider_move_ms={:.3} split_cohorts={} split_cohort_bodies={} sibling_grace_pairs={} sibling_grace_filtered_pairs={} rapier_passes={} fractures={} splits={} reused_bodies={} recycled_bodies={} new_bodies={} retired_bodies={} body_type_flips={} moved_colliders={} world_bodies={} world_bodies_delta={} dynamic_bodies={} dynamic_bodies_delta={} awake_dynamic_bodies={} awake_dynamic_bodies_delta={} contact_pairs={} contact_pairs_delta={} active_contact_pairs={} active_contact_pairs_delta={} contact_manifolds={} contact_manifolds_delta={}",
                self.frame_index,
                frame_ms,
                self.current.physics_ms,
                self.current.rapier_ms,
                self.current.solver_ms,
                self.current.split_plan_ms,
                self.current.split_apply_ms,
                self.current.split_collider_move_ms,
                self.current.split_cohorts,
                self.current.split_cohort_bodies,
                self.current.sibling_grace_pairs,
                self.current.sibling_grace_filtered_pairs,
                self.current.rapier_passes,
                self.current.fractures,
                self.current.split_events,
                self.current.reused_bodies,
                self.current.recycled_bodies,
                self.current.new_bodies,
                self.current.retired_bodies,
                self.current.body_type_flips,
                self.current.moved_colliders,
                self.current.world_bodies,
                world_bodies_delta,
                self.current.dynamic_bodies,
                dynamic_bodies_delta,
                self.current.awake_dynamic_bodies,
                awake_dynamic_bodies_delta,
                self.current.contact_pairs,
                contact_pairs_delta,
                self.current.active_contact_pairs,
                active_contact_pairs_delta,
                self.current.contact_manifolds,
                contact_manifolds_delta,
            ));
        }

        let topology_frame = world_bodies_delta.unsigned_abs()
            >= self.topology_body_delta_threshold
            || dynamic_bodies_delta.unsigned_abs() >= self.topology_body_delta_threshold
            || awake_dynamic_bodies_delta.unsigned_abs() >= self.topology_body_delta_threshold
            || self.current.new_bodies > 0
            || self.current.retired_bodies > 0
            || self.current.recycled_bodies > 0
            || self.current.body_type_flips > 0;

        if topology_frame {
            self.pending_event_lines.push(format!(
                "[topology-frame] frame={} frame_ms={:.3} physics_ms={:.3} rapier_ms={:.3} solver_ms={:.3} split_plan_ms={:.3} split_apply_ms={:.3} split_collider_move_ms={:.3} split_cohorts={} split_cohort_bodies={} sibling_grace_pairs={} sibling_grace_filtered_pairs={} fractures={} splits={} reused_bodies={} recycled_bodies={} new_bodies={} retired_bodies={} body_type_flips={} moved_colliders={} world_bodies={} prev_world_bodies={} world_bodies_delta={} dynamic_bodies={} prev_dynamic_bodies={} dynamic_bodies_delta={} awake_dynamic_bodies={} prev_awake_dynamic_bodies={} awake_dynamic_bodies_delta={} contact_pairs={} prev_contact_pairs={} contact_pairs_delta={} active_contact_pairs={} prev_active_contact_pairs={} active_contact_pairs_delta={} contact_manifolds={} prev_contact_manifolds={} contact_manifolds_delta={}",
                self.frame_index,
                frame_ms,
                self.current.physics_ms,
                self.current.rapier_ms,
                self.current.solver_ms,
                self.current.split_plan_ms,
                self.current.split_apply_ms,
                self.current.split_collider_move_ms,
                self.current.split_cohorts,
                self.current.split_cohort_bodies,
                self.current.sibling_grace_pairs,
                self.current.sibling_grace_filtered_pairs,
                self.current.fractures,
                self.current.split_events,
                self.current.reused_bodies,
                self.current.recycled_bodies,
                self.current.new_bodies,
                self.current.retired_bodies,
                self.current.body_type_flips,
                self.current.moved_colliders,
                self.current.world_bodies,
                prev_world_bodies,
                world_bodies_delta,
                self.current.dynamic_bodies,
                prev_dynamic_bodies,
                dynamic_bodies_delta,
                self.current.awake_dynamic_bodies,
                prev_awake_dynamic_bodies,
                awake_dynamic_bodies_delta,
                self.current.contact_pairs,
                prev_contact_pairs,
                contact_pairs_delta,
                self.current.active_contact_pairs,
                prev_active_contact_pairs,
                active_contact_pairs_delta,
                self.current.contact_manifolds,
                prev_contact_manifolds,
                contact_manifolds_delta,
            ));
        }

        let fracture_frame = self.current.fractures > 0
            || self.current.split_events > 0
            || self.current.new_bodies > 0
            || self.current.reused_bodies > 0
            || self.current.recycled_bodies > 0
            || self.current.body_type_flips > 0
            || self.current.moved_colliders > 0
            || self.current.inserted_colliders > 0
            || self.current.removed_colliders > 0
            || self.current.split_plan_ms > 0.0
            || self.current.split_apply_ms > 0.0
            || self.current.split_collider_move_ms > 0.0;

        if fracture_frame {
            self.first_fracture_frame_index
                .get_or_insert(self.frame_index);
            if frame_ms >= self.peak_fracture_frame_ms {
                self.peak_fracture_frame_ms = frame_ms;
                self.peak_fracture_frame_index = self.frame_index;
            }
            if self.current.physics_ms >= self.peak_fracture_physics_ms {
                self.peak_fracture_physics_ms = self.current.physics_ms;
                self.peak_fracture_physics_frame_index = self.frame_index;
            }
            if self.current.rapier_ms >= self.peak_fracture_rapier_ms {
                self.peak_fracture_rapier_ms = self.current.rapier_ms;
                self.peak_fracture_rapier_frame_index = self.frame_index;
            }
            if self.current.solver_ms >= self.peak_fracture_solver_ms {
                self.peak_fracture_solver_ms = self.current.solver_ms;
                self.peak_fracture_solver_frame_index = self.frame_index;
            }
            if self.current.split_plan_ms >= self.peak_fracture_split_plan_ms {
                self.peak_fracture_split_plan_ms = self.current.split_plan_ms;
                self.peak_fracture_split_plan_frame_index = self.frame_index;
            }
            if self.current.split_apply_ms >= self.peak_fracture_split_apply_ms {
                self.peak_fracture_split_apply_ms = self.current.split_apply_ms;
                self.peak_fracture_split_apply_frame_index = self.frame_index;
            }

            self.pending_event_lines.push(format!(
                "[fracture-frame] frame={} frame_ms={:.3} physics_ms={:.3} rapier_ms={:.3} collision_events_ms={:.3} contact_forces_ms={:.3} solver_ms={:.3} split_sanitize_ms={:.3} split_estimate_ms={:.3} split_plan_ms={:.3} split_apply_ms={:.3} split_child_pose_ms={:.3} split_velocity_fit_ms={:.3} split_sleep_init_ms={:.3} split_body_create_ms={:.3} split_collider_move_ms={:.3} split_collider_insert_ms={:.3} split_body_retire_ms={:.3} split_cohorts={} split_cohort_bodies={} sibling_grace_pairs={} sibling_grace_filtered_pairs={} rapier_passes={} collisions={} contacts={} fractures={} splits={} reused_bodies={} recycled_bodies={} new_bodies={} retired_bodies={} body_type_flips={} moved_colliders={} inserted_colliders={} removed_colliders={} removed_nodes={} world_bodies={} destructible_bodies={} support_bodies={} dynamic_bodies={} awake_dynamic_bodies={} sleeping_dynamic_bodies={} world_colliders={} destructible_colliders={} projectiles={} contact_pairs={} active_contact_pairs={} contact_manifolds={} pending_splits={} pending_new_bodies={} pending_migrations={}",
                self.frame_index,
                frame_ms,
                self.current.physics_ms,
                self.current.rapier_ms,
                self.current.collision_events_ms,
                self.current.contact_forces_ms,
                self.current.solver_ms,
                self.current.split_sanitize_ms,
                self.current.split_estimate_ms,
                self.current.split_plan_ms,
                self.current.split_apply_ms,
                self.current.split_child_pose_ms,
                self.current.split_velocity_fit_ms,
                self.current.split_sleep_init_ms,
                self.current.split_body_create_ms,
                self.current.split_collider_move_ms,
                self.current.split_collider_insert_ms,
                self.current.split_body_retire_ms,
                self.current.split_cohorts,
                self.current.split_cohort_bodies,
                self.current.sibling_grace_pairs,
                self.current.sibling_grace_filtered_pairs,
                self.current.rapier_passes,
                self.current.collision_events,
                self.current.contact_events,
                self.current.fractures,
                self.current.split_events,
                self.current.reused_bodies,
                self.current.recycled_bodies,
                self.current.new_bodies,
                self.current.retired_bodies,
                self.current.body_type_flips,
                self.current.moved_colliders,
                self.current.inserted_colliders,
                self.current.removed_colliders,
                self.current.removed_nodes,
                self.current.world_bodies,
                self.current.destructible_bodies,
                self.current.support_bodies,
                self.current.dynamic_bodies,
                self.current.awake_dynamic_bodies,
                self.current.sleeping_dynamic_bodies,
                self.current.world_colliders,
                self.current.destructible_colliders,
                self.current.projectile_count,
                self.current.contact_pairs,
                self.current.active_contact_pairs,
                self.current.contact_manifolds,
                self.current.pending_split_events,
                self.current.pending_new_bodies,
                self.current.pending_collider_migrations,
            ));
        }

        if self.current.sibling_grace_pairs > 0 || self.current.sibling_grace_filtered_pairs > 0 {
            self.pending_event_lines.push(format!(
                "[post-split-frame] frame={} frame_ms={:.3} physics_ms={:.3} rapier_ms={:.3} split_cohorts={} split_cohort_bodies={} sibling_grace_pairs={} sibling_grace_filtered_pairs={} world_bodies={} dynamic_bodies={} awake_dynamic_bodies={} contact_pairs={} active_contact_pairs={} contact_manifolds={}",
                self.frame_index,
                frame_ms,
                self.current.physics_ms,
                self.current.rapier_ms,
                self.current.split_cohorts,
                self.current.split_cohort_bodies,
                self.current.sibling_grace_pairs,
                self.current.sibling_grace_filtered_pairs,
                self.current.world_bodies,
                self.current.dynamic_bodies,
                self.current.awake_dynamic_bodies,
                self.current.contact_pairs,
                self.current.active_contact_pairs,
                self.current.contact_manifolds,
            ));
        }
        self.avg_rapier_passes = if self.avg_rapier_passes == 0.0 {
            self.current.rapier_passes as f32
        } else {
            self.avg_rapier_passes * 0.9 + self.current.rapier_passes as f32 * 0.1
        };

        self.median_window.frame_ms.push(frame_ms);
        self.median_window.physics_ms.push(self.current.physics_ms);
        self.median_window.rapier_ms.push(self.current.rapier_ms);
        self.median_window
            .collision_events_ms
            .push(self.current.collision_events_ms);
        self.median_window
            .contact_forces_ms
            .push(self.current.contact_forces_ms);
        self.median_window.solver_ms.push(self.current.solver_ms);
        self.median_window
            .split_sanitize_ms
            .push(self.current.split_sanitize_ms);
        self.median_window
            .split_estimate_ms
            .push(self.current.split_estimate_ms);
        self.median_window
            .split_plan_ms
            .push(self.current.split_plan_ms);
        self.median_window
            .split_apply_ms
            .push(self.current.split_apply_ms);
        self.median_window
            .split_child_pose_ms
            .push(self.current.split_child_pose_ms);
        self.median_window
            .split_velocity_fit_ms
            .push(self.current.split_velocity_fit_ms);
        self.median_window
            .split_sleep_init_ms
            .push(self.current.split_sleep_init_ms);
        self.median_window
            .split_body_create_ms
            .push(self.current.split_body_create_ms);
        self.median_window
            .split_collider_move_ms
            .push(self.current.split_collider_move_ms);
        self.median_window
            .split_collider_insert_ms
            .push(self.current.split_collider_insert_ms);
        self.median_window
            .split_body_retire_ms
            .push(self.current.split_body_retire_ms);
        self.median_window
            .resim_restore_ms
            .push(self.current.resim_restore_ms);
        self.median_window
            .resim_snapshot_ms
            .push(self.current.resim_snapshot_ms);
        self.median_window
            .optimization_ms
            .push(self.current.optimization_ms);
        self.median_window
            .projectile_cleanup_ms
            .push(self.current.projectile_cleanup_ms);
        self.median_window.render_prep_ms.push(render_prep_ms);
        self.median_window
            .sync_visuals_ms
            .push(self.current.sync_visuals_ms);
        self.median_window.gizmo_ms.push(self.current.gizmo_ms);
        self.median_window.hud_ms.push(self.current.hud_ms);
        self.median_window.other_cpu_ms.push(other_cpu_ms);
        self.median_window
            .rapier_passes
            .push(self.current.rapier_passes as f32);
        self.median_window
            .collision_events
            .push(self.current.collision_events as f32);
        self.median_window
            .contact_events
            .push(self.current.contact_events as f32);
        self.median_window
            .fractures
            .push(self.current.fractures as f32);
        self.median_window
            .split_events
            .push(self.current.split_events as f32);
        self.median_window
            .split_cohorts
            .push(self.current.split_cohorts as f32);
        self.median_window
            .split_cohort_bodies
            .push(self.current.split_cohort_bodies as f32);
        self.median_window
            .reused_bodies
            .push(self.current.reused_bodies as f32);
        self.median_window
            .recycled_bodies
            .push(self.current.recycled_bodies as f32);
        self.median_window
            .new_bodies
            .push(self.current.new_bodies as f32);
        self.median_window
            .retired_bodies
            .push(self.current.retired_bodies as f32);
        self.median_window
            .body_type_flips
            .push(self.current.body_type_flips as f32);
        self.median_window
            .moved_colliders
            .push(self.current.moved_colliders as f32);
        self.median_window
            .inserted_colliders
            .push(self.current.inserted_colliders as f32);
        self.median_window
            .removed_colliders
            .push(self.current.removed_colliders as f32);
        self.median_window
            .removed_nodes
            .push(self.current.removed_nodes as f32);
        self.median_window
            .removed_projectiles
            .push(self.current.removed_projectiles as f32);
        self.median_window
            .world_bodies
            .push(self.current.world_bodies as f32);
        self.median_window
            .destructible_bodies
            .push(self.current.destructible_bodies as f32);
        self.median_window
            .support_bodies
            .push(self.current.support_bodies as f32);
        self.median_window
            .dynamic_bodies
            .push(self.current.dynamic_bodies as f32);
        self.median_window
            .awake_dynamic_bodies
            .push(self.current.awake_dynamic_bodies as f32);
        self.median_window
            .sleeping_dynamic_bodies
            .push(self.current.sleeping_dynamic_bodies as f32);
        self.median_window
            .world_colliders
            .push(self.current.world_colliders as f32);
        self.median_window
            .destructible_colliders
            .push(self.current.destructible_colliders as f32);
        self.median_window
            .projectile_count
            .push(self.current.projectile_count as f32);
        self.median_window
            .ccd_bodies
            .push(self.current.ccd_bodies as f32);
        self.median_window
            .contact_pairs
            .push(self.current.contact_pairs as f32);
        self.median_window
            .active_contact_pairs
            .push(self.current.active_contact_pairs as f32);
        self.median_window
            .contact_manifolds
            .push(self.current.contact_manifolds as f32);
        self.median_window
            .sibling_grace_pairs
            .push(self.current.sibling_grace_pairs as f32);
        self.median_window
            .sibling_grace_filtered_pairs
            .push(self.current.sibling_grace_filtered_pairs as f32);
        self.median_window
            .pending_split_events
            .push(self.current.pending_split_events as f32);
        self.median_window
            .pending_new_bodies
            .push(self.current.pending_new_bodies as f32);
        self.median_window
            .pending_collider_migrations
            .push(self.current.pending_collider_migrations as f32);

        if now.duration_since(*log_window_started_at) >= Duration::from_secs(1) {
            let line = self.log_median_window();
            self.log_window_started_at = Some(now);
            line
        } else {
            None
        }
    }

    fn fps(&self) -> f32 {
        if self.frame_cpu.avg_ms <= f32::EPSILON {
            0.0
        } else {
            1_000.0 / self.frame_cpu.avg_ms
        }
    }

    fn headless_summary_line(
        &self,
        scenario: DemoScenarioKind,
        shot_plan: Option<&HeadlessShotPlan>,
        projectile_stats: ProjectileRunStats,
    ) -> String {
        let (shot_script, shots_planned, shots_fired) = if let Some(plan) = shot_plan {
            (plan.script_name.as_str(), plan.total_shots(), plan.fired)
        } else {
            ("none", 0, 0)
        };

        format!(
            "[summary] scenario={} total_frames={} shot_script={} shots_planned={} shots_fired={} projectile_crossed_target_plane_count={} projectile_passed_through_count={} projectile_max_progress_ratio={:.3} max_frame_ms={:.3} max_frame_frame={} max_physics_ms={:.3} max_physics_frame={} max_rapier_ms={:.3} max_rapier_frame={} max_solver_ms={:.3} max_solver_frame={} max_split_plan_ms={:.3} max_split_plan_frame={} max_split_apply_ms={:.3} max_split_apply_frame={} max_split_move_ms={:.3} max_split_move_frame={} peak_world_bodies={} peak_world_bodies_frame={} peak_dynamic_bodies={} peak_dynamic_bodies_frame={} peak_awake_dynamic_bodies={} peak_awake_dynamic_bodies_frame={} peak_active_contact_pairs={} peak_active_contact_pairs_frame={} peak_contact_manifolds={} peak_contact_manifolds_frame={} peak_sibling_grace_filtered_pairs={} peak_sibling_grace_filtered_pairs_frame={} total_fractures={} total_splits={} total_new_bodies={} total_moved_colliders={} total_collision_events={} total_contact_events={} first_fracture_frame={} peak_fracture_frame_ms={:.3} peak_fracture_frame={} peak_fracture_physics_ms={:.3} peak_fracture_physics_frame={} peak_fracture_rapier_ms={:.3} peak_fracture_rapier_frame={} peak_fracture_solver_ms={:.3} peak_fracture_solver_frame={} peak_fracture_split_plan_ms={:.3} peak_fracture_split_plan_frame={} peak_fracture_split_apply_ms={:.3} peak_fracture_split_apply_frame={} {}",
            scenario.slug(),
            self.frame_index,
            shot_script,
            shots_planned,
            shots_fired,
            projectile_stats.crossed_target_plane_count,
            projectile_stats.passed_through_count,
            projectile_stats.max_progress_ratio,
            self.peak_frame_ms,
            self.peak_frame_index,
            self.peak_physics_ms,
            self.peak_physics_frame_index,
            self.peak_rapier_ms,
            self.peak_rapier_frame_index,
            self.peak_solver_ms,
            self.peak_solver_frame_index,
            self.peak_split_plan_ms,
            self.peak_split_plan_frame_index,
            self.peak_split_apply_ms,
            self.peak_split_apply_frame_index,
            self.peak_split_collider_move_ms,
            self.peak_split_collider_move_frame_index,
            self.peak_world_bodies,
            self.peak_world_bodies_frame_index,
            self.peak_dynamic_bodies,
            self.peak_dynamic_bodies_frame_index,
            self.peak_awake_dynamic_bodies,
            self.peak_awake_dynamic_bodies_frame_index,
            self.peak_active_contact_pairs,
            self.peak_active_contact_pairs_frame_index,
            self.peak_contact_manifolds,
            self.peak_contact_manifolds_frame_index,
            self.peak_sibling_grace_filtered_pairs,
            self.peak_sibling_grace_filtered_pairs_frame_index,
            self.total_fractures,
            self.total_split_events,
            self.total_new_bodies,
            self.total_moved_colliders,
            self.total_collision_events,
            self.total_contact_events,
            self.first_fracture_frame_index.unwrap_or(0),
            self.peak_fracture_frame_ms,
            self.peak_fracture_frame_index,
            self.peak_fracture_physics_ms,
            self.peak_fracture_physics_frame_index,
            self.peak_fracture_rapier_ms,
            self.peak_fracture_rapier_frame_index,
            self.peak_fracture_solver_ms,
            self.peak_fracture_solver_frame_index,
            self.peak_fracture_split_plan_ms,
            self.peak_fracture_split_plan_frame_index,
            self.peak_fracture_split_apply_ms,
            self.peak_fracture_split_apply_frame_index,
            self.config_summary,
        )
    }

    fn log_median_window(&mut self) -> Option<String> {
        let sample_count = self.median_window.frame_ms.len();
        if sample_count == 0 {
            return None;
        }

        let frame_ms_med = median_ms(&mut self.median_window.frame_ms);
        let physics_ms_med = median_ms(&mut self.median_window.physics_ms);
        let rapier_ms_med = median_ms(&mut self.median_window.rapier_ms);
        let collision_events_ms_med = median_ms(&mut self.median_window.collision_events_ms);
        let contact_forces_ms_med = median_ms(&mut self.median_window.contact_forces_ms);
        let solver_ms_med = median_ms(&mut self.median_window.solver_ms);
        let split_sanitize_ms_med = median_ms(&mut self.median_window.split_sanitize_ms);
        let split_estimate_ms_med = median_ms(&mut self.median_window.split_estimate_ms);
        let split_plan_ms_med = median_ms(&mut self.median_window.split_plan_ms);
        let split_apply_ms_med = median_ms(&mut self.median_window.split_apply_ms);
        let split_child_pose_ms_med = median_ms(&mut self.median_window.split_child_pose_ms);
        let split_velocity_fit_ms_med = median_ms(&mut self.median_window.split_velocity_fit_ms);
        let split_sleep_init_ms_med = median_ms(&mut self.median_window.split_sleep_init_ms);
        let split_body_create_ms_med = median_ms(&mut self.median_window.split_body_create_ms);
        let split_collider_move_ms_med = median_ms(&mut self.median_window.split_collider_move_ms);
        let split_collider_insert_ms_med =
            median_ms(&mut self.median_window.split_collider_insert_ms);
        let split_body_retire_ms_med = median_ms(&mut self.median_window.split_body_retire_ms);
        let resim_restore_ms_med = median_ms(&mut self.median_window.resim_restore_ms);
        let resim_snapshot_ms_med = median_ms(&mut self.median_window.resim_snapshot_ms);
        let optimization_ms_med = median_ms(&mut self.median_window.optimization_ms);
        let projectile_cleanup_ms_med = median_ms(&mut self.median_window.projectile_cleanup_ms);
        let render_prep_ms_med = median_ms(&mut self.median_window.render_prep_ms);
        let sync_visuals_ms_med = median_ms(&mut self.median_window.sync_visuals_ms);
        let gizmo_ms_med = median_ms(&mut self.median_window.gizmo_ms);
        let hud_ms_med = median_ms(&mut self.median_window.hud_ms);
        let other_cpu_ms_med = median_ms(&mut self.median_window.other_cpu_ms);
        let rapier_passes_med = median_ms(&mut self.median_window.rapier_passes);
        let collision_events_med = median_ms(&mut self.median_window.collision_events);
        let contact_events_med = median_ms(&mut self.median_window.contact_events);
        let fractures_med = median_ms(&mut self.median_window.fractures);
        let split_events_med = median_ms(&mut self.median_window.split_events);
        let split_cohorts_med = median_ms(&mut self.median_window.split_cohorts);
        let split_cohort_bodies_med = median_ms(&mut self.median_window.split_cohort_bodies);
        let reused_bodies_med = median_ms(&mut self.median_window.reused_bodies);
        let recycled_bodies_med = median_ms(&mut self.median_window.recycled_bodies);
        let new_bodies_med = median_ms(&mut self.median_window.new_bodies);
        let retired_bodies_med = median_ms(&mut self.median_window.retired_bodies);
        let body_type_flips_med = median_ms(&mut self.median_window.body_type_flips);
        let moved_colliders_med = median_ms(&mut self.median_window.moved_colliders);
        let inserted_colliders_med = median_ms(&mut self.median_window.inserted_colliders);
        let removed_colliders_med = median_ms(&mut self.median_window.removed_colliders);
        let removed_nodes_med = median_ms(&mut self.median_window.removed_nodes);
        let removed_projectiles_med = median_ms(&mut self.median_window.removed_projectiles);
        let world_bodies_med = median_ms(&mut self.median_window.world_bodies);
        let destructible_bodies_med = median_ms(&mut self.median_window.destructible_bodies);
        let support_bodies_med = median_ms(&mut self.median_window.support_bodies);
        let dynamic_bodies_med = median_ms(&mut self.median_window.dynamic_bodies);
        let awake_dynamic_bodies_med = median_ms(&mut self.median_window.awake_dynamic_bodies);
        let sleeping_dynamic_bodies_med =
            median_ms(&mut self.median_window.sleeping_dynamic_bodies);
        let world_colliders_med = median_ms(&mut self.median_window.world_colliders);
        let destructible_colliders_med = median_ms(&mut self.median_window.destructible_colliders);
        let projectile_count_med = median_ms(&mut self.median_window.projectile_count);
        let ccd_bodies_med = median_ms(&mut self.median_window.ccd_bodies);
        let contact_pairs_med = median_ms(&mut self.median_window.contact_pairs);
        let active_contact_pairs_med = median_ms(&mut self.median_window.active_contact_pairs);
        let contact_manifolds_med = median_ms(&mut self.median_window.contact_manifolds);
        let sibling_grace_pairs_med = median_ms(&mut self.median_window.sibling_grace_pairs);
        let sibling_grace_filtered_pairs_med =
            median_ms(&mut self.median_window.sibling_grace_filtered_pairs);
        let pending_split_events_med = median_ms(&mut self.median_window.pending_split_events);
        let pending_new_bodies_med = median_ms(&mut self.median_window.pending_new_bodies);
        let pending_collider_migrations_med =
            median_ms(&mut self.median_window.pending_collider_migrations);
        let fps_med = if frame_ms_med <= f32::EPSILON {
            0.0
        } else {
            1_000.0 / frame_ms_med
        };

        let line = format!(
            "[perf] samples={} fps_med={:.1} frame_ms_med={:.3} physics_ms_med={:.3} rapier_ms_med={:.3} collision_events_ms_med={:.3} contact_forces_ms_med={:.3} solver_ms_med={:.3} split_sanitize_ms_med={:.3} split_estimate_ms_med={:.3} split_plan_ms_med={:.3} split_apply_ms_med={:.3} split_child_pose_ms_med={:.3} split_velocity_fit_ms_med={:.3} split_sleep_init_ms_med={:.3} split_body_create_ms_med={:.3} split_collider_move_ms_med={:.3} split_collider_insert_ms_med={:.3} split_body_retire_ms_med={:.3} resim_restore_ms_med={:.3} snapshot_ms_med={:.3} optimization_ms_med={:.3} projectile_cleanup_ms_med={:.3} render_prep_ms_med={:.3} sync_ms_med={:.3} gizmo_ms_med={:.3} hud_ms_med={:.3} other_cpu_ms_med={:.3} rapier_passes_med={:.1} collisions_med={:.1} contacts_med={:.1} fractures_med={:.1} splits_med={:.1} split_cohorts_med={:.1} split_cohort_bodies_med={:.1} reused_bodies_med={:.1} recycled_bodies_med={:.1} new_bodies_med={:.1} retired_bodies_med={:.1} body_type_flips_med={:.1} moved_colliders_med={:.1} inserted_colliders_med={:.1} removed_colliders_med={:.1} removed_nodes_med={:.1} removed_projectiles_med={:.1} world_bodies_med={:.1} destructible_bodies_med={:.1} support_bodies_med={:.1} dynamic_bodies_med={:.1} awake_dynamic_bodies_med={:.1} sleeping_dynamic_bodies_med={:.1} world_colliders_med={:.1} destructible_colliders_med={:.1} projectiles_med={:.1} ccd_bodies_med={:.1} contact_pairs_med={:.1} active_contact_pairs_med={:.1} contact_manifolds_med={:.1} sibling_grace_pairs_med={:.1} sibling_grace_filtered_pairs_med={:.1} pending_splits_med={:.1} pending_new_bodies_med={:.1} pending_migrations_med={:.1} {}",
            sample_count,
            fps_med,
            frame_ms_med,
            physics_ms_med,
            rapier_ms_med,
            collision_events_ms_med,
            contact_forces_ms_med,
            solver_ms_med,
            split_sanitize_ms_med,
            split_estimate_ms_med,
            split_plan_ms_med,
            split_apply_ms_med,
            split_child_pose_ms_med,
            split_velocity_fit_ms_med,
            split_sleep_init_ms_med,
            split_body_create_ms_med,
            split_collider_move_ms_med,
            split_collider_insert_ms_med,
            split_body_retire_ms_med,
            resim_restore_ms_med,
            resim_snapshot_ms_med,
            optimization_ms_med,
            projectile_cleanup_ms_med,
            render_prep_ms_med,
            sync_visuals_ms_med,
            gizmo_ms_med,
            hud_ms_med,
            other_cpu_ms_med,
            rapier_passes_med,
            collision_events_med,
            contact_events_med,
            fractures_med,
            split_events_med,
            split_cohorts_med,
            split_cohort_bodies_med,
            reused_bodies_med,
            recycled_bodies_med,
            new_bodies_med,
            retired_bodies_med,
            body_type_flips_med,
            moved_colliders_med,
            inserted_colliders_med,
            removed_colliders_med,
            removed_nodes_med,
            removed_projectiles_med,
            world_bodies_med,
            destructible_bodies_med,
            support_bodies_med,
            dynamic_bodies_med,
            awake_dynamic_bodies_med,
            sleeping_dynamic_bodies_med,
            world_colliders_med,
            destructible_colliders_med,
            projectile_count_med,
            ccd_bodies_med,
            contact_pairs_med,
            active_contact_pairs_med,
            contact_manifolds_med,
            sibling_grace_pairs_med,
            sibling_grace_filtered_pairs_med,
            pending_split_events_med,
            pending_new_bodies_med,
            pending_collider_migrations_med,
            self.config_summary,
        );

        self.median_window = MedianWindow::default();
        Some(line)
    }

    fn drain_event_lines(&mut self) -> Vec<String> {
        std::mem::take(&mut self.pending_event_lines)
    }
}

fn median_ms(samples: &mut Vec<f32>) -> f32 {
    if samples.is_empty() {
        return 0.0;
    }
    samples.sort_by(|a, b| a.total_cmp(b));
    let mid = samples.len() / 2;
    if samples.len() % 2 == 0 {
        (samples[mid - 1] + samples[mid]) * 0.5
    } else {
        samples[mid]
    }
}

#[derive(Clone, Copy)]
struct ProjectileState {
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    ttl: f32,
    origin: Vec3,
    direction: Vec3,
    target_distance: f32,
    exit_distance: Option<f32>,
    max_progress: f32,
    crossed_target_plane: bool,
    passed_through: bool,
}

#[derive(Clone, Copy, Debug, Default)]
struct ProjectileRunStats {
    crossed_target_plane_count: usize,
    passed_through_count: usize,
    max_progress_ratio: f32,
}

struct RapierOnlyState {
    node_to_body: Vec<Option<RigidBodyHandle>>,
    node_local_offsets: Vec<SolverVec3>,
    support_body_count: usize,
}

struct DemoPhysicsState {
    config: DemoConfig,
    physics_pipeline: PhysicsPipeline,
    integration_parameters: IntegrationParameters,
    island_manager: IslandManager,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
    ccd_solver: CCDSolver,
    destructible: DestructionRuntime,
    rapier_only: Option<RapierOnlyState>,
    node_to_entity: HashMap<u32, Entity>,
    projectile_entities: HashMap<RigidBodyHandle, Entity>,
    projectile_colliders: HashMap<ColliderHandle, RigidBodyHandle>,
    projectiles: Vec<ProjectileState>,
    projectile_run_stats: ProjectileRunStats,
}

fn main() -> AppExit {
    if should_start_headless() {
        eprintln!(
            "Starting in headless simulation mode for {} frames.",
            headless_frame_budget()
        );
        return run_app(true);
    }

    let default_panic_hook = Arc::new(panic::take_hook());
    let filtered_hook = default_panic_hook.clone();
    panic::set_hook(Box::new(move |info| {
        if is_missing_gpu_panic(info.payload()) {
            return;
        }

        (filtered_hook)(info);
    }));

    match panic::catch_unwind(|| run_app(false)) {
        Ok(exit) => {
            restore_panic_hook(default_panic_hook);
            exit
        }
        Err(payload) if is_missing_gpu_panic(payload.as_ref()) => {
            restore_panic_hook(default_panic_hook);
            eprintln!(
                "No GPU adapter available; restarting in headless simulation mode for {} frames.",
                headless_frame_budget()
            );
            run_app(true)
        }
        Err(payload) => {
            restore_panic_hook(default_panic_hook);
            panic::resume_unwind(payload)
        }
    }
}

fn run_app(headless: bool) -> AppExit {
    let kind = selected_scenario_kind();
    let mut config = build_demo_config(kind);
    let toggles = DemoRuntimeToggles::from_env(kind, &mut config);
    let mut perf_log = PerfLogWriter::create().expect("failed to create perf log file");
    let perf_log_path = perf_log.path().clone();
    let info = DemoInfo {
        title: format!("Blast Stress Solver - {}", config.title),
        subtitle: format!("Scenario: {}", kind.slug()),
        camera_target: config.camera_target,
        camera_distance: config.camera_distance,
    };

    let physics = build_demo_physics(config.clone(), &toggles);
    let headless_shot_plan = if headless {
        build_headless_shot_plan(kind, &config)
    } else {
        None
    };
    let mut profiler = DebugProfiler::default();
    profiler.config_summary = toggles.summary();
    profiler.heavy_frame_threshold_ms = toggles.heavy_frame_threshold_ms;
    profiler.topology_body_delta_threshold = toggles.topology_body_delta_threshold;
    perf_log.write_line(&format!("# {}", profiler.config_summary));
    if let Some(plan) = &headless_shot_plan {
        perf_log.write_line(&format!(
            "[shot-plan] scenario={} script={} shots={}",
            kind.slug(),
            plan.script_name,
            plan.total_shots()
        ));
    }

    let exit = {
        let mut app = App::new();
        app.insert_resource(DemoScenario { kind });
        app.insert_resource(RunMode { headless });
        app.insert_resource(info.clone());
        app.insert_resource(CameraOrbit::from_info(&info));
        app.insert_resource(ProjectileFireSettings::new(config.projectile_mass));
        app.insert_resource(toggles.clone());
        app.insert_resource(profiler);
        app.insert_resource(perf_log);
        if let Some(plan) = headless_shot_plan {
            app.insert_resource(plan);
        }
        app.insert_non_send_resource(physics);

        if headless {
            app.add_plugins((
                MinimalPlugins.set(ScheduleRunnerPlugin::run_loop(Duration::from_secs_f64(
                    1.0 / 60.0,
                ))),
                TransformPlugin,
            ));
            app.insert_resource(HeadlessRunBudget {
                frames_remaining: headless_frame_budget(),
            });
            app.add_systems(
                Update,
                (
                    begin_frame_profile_system,
                    auto_fire_headless_projectiles_system,
                    physics_step_system,
                    finish_frame_profile_system,
                    headless_exit_system,
                )
                    .chain(),
            );
        } else {
            app.insert_resource(ClearColor(Color::srgb(0.07, 0.08, 0.10)));
            app.add_plugins(DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: info.title.clone(),
                    ..default()
                }),
                ..default()
            }));
            app.add_systems(Startup, (setup_scene, setup_visuals));
            app.add_systems(
                Update,
                (
                    begin_frame_profile_system,
                    camera_orbit_system,
                    reset_scene_system,
                    projectile_mass_shortcuts_system,
                    shoot_projectile_system,
                    physics_step_system,
                    sync_visuals_system,
                    gizmo_render_system,
                    hud_system,
                    finish_frame_profile_system,
                )
                    .chain(),
            );
        }

        let exit = app.run();
        if let Some(mut perf_log) = app.world_mut().remove_resource::<PerfLogWriter>() {
            perf_log.flush();
        }
        exit
    };
    println!("{}", perf_log_path.display());
    exit
}

fn selected_scenario_kind() -> DemoScenarioKind {
    match env::var("BLAST_STRESS_DEMO_SCENARIO")
        .ok()
        .map(|value| value.trim().to_ascii_lowercase())
        .as_deref()
    {
        Some("tower") => DemoScenarioKind::Tower,
        Some("bridge") => DemoScenarioKind::Bridge,
        Some("fractured-wall") | Some("fractured_wall") | Some("fwall") => {
            DemoScenarioKind::FracturedWall
        }
        Some("fractured-tower") | Some("fractured_tower") | Some("ftower") => {
            DemoScenarioKind::FracturedTower
        }
        Some("fractured-bridge") | Some("fractured_bridge") | Some("fbridge") => {
            DemoScenarioKind::FracturedBridge
        }
        Some("brick-building") | Some("brick_building") | Some("building") => {
            DemoScenarioKind::BrickBuilding
        }
        _ => DemoScenarioKind::Wall,
    }
}

fn build_wall_demo_config() -> DemoConfig {
    DemoConfig {
        title: "Wall Demolition".to_string(),
        scenario: build_wall_scenario(&WallOptions::default()),
        node_meshes: Arc::from(Vec::<SceneMeshAsset>::new()),
        projectile_radius: 0.35,
        projectile_mass: 1_000.0,
        projectile_speed: 20.0,
        projectile_ttl: DEFAULT_PROJECTILE_TTL,
        gravity: GRAVITY,
        material_scale: 1.0e10,
        skip_single_bodies: false,
        camera_target: Vec3::new(0.0, 1.5, 0.0),
        camera_distance: 14.0,
        policy: FracturePolicy {
            apply_excess_forces: false,
            ..FracturePolicy::default()
        },
        resimulation: ResimulationOptions {
            enabled: true,
            max_passes: 1,
        },
        sleep_thresholds: SleepThresholdOptions::default(),
        small_body_damping: SmallBodyDampingOptions {
            mode: OptimizationMode::Off,
            ..SmallBodyDampingOptions::default()
        },
        debris_cleanup: DebrisCleanupOptions {
            mode: OptimizationMode::Always,
            debris_ttl_secs: 8.0,
            max_colliders_for_debris: 2,
        },
        debris_collision_mode: DebrisCollisionMode::All,
    }
}

fn build_tower_demo_config() -> DemoConfig {
    DemoConfig {
        title: "Tower Collapse".to_string(),
        scenario: build_tower_scenario(&TowerOptions::default()),
        node_meshes: Arc::from(Vec::<SceneMeshAsset>::new()),
        projectile_radius: 0.35,
        projectile_mass: 1_000.0,
        projectile_speed: 22.0,
        projectile_ttl: DEFAULT_PROJECTILE_TTL,
        gravity: GRAVITY,
        material_scale: 1.0e10,
        skip_single_bodies: false,
        camera_target: Vec3::new(0.0, 1.5, 0.0),
        camera_distance: 20.0,
        policy: FracturePolicy {
            apply_excess_forces: false,
            ..FracturePolicy::default()
        },
        resimulation: ResimulationOptions {
            enabled: true,
            max_passes: 1,
        },
        sleep_thresholds: SleepThresholdOptions::default(),
        small_body_damping: SmallBodyDampingOptions {
            mode: OptimizationMode::Always,
            collider_count_threshold: 3,
            min_linear_damping: 2.0,
            min_angular_damping: 2.0,
        },
        debris_cleanup: DebrisCleanupOptions {
            mode: OptimizationMode::Always,
            debris_ttl_secs: 10.0,
            max_colliders_for_debris: 2,
        },
        debris_collision_mode: DebrisCollisionMode::All,
    }
}

fn build_bridge_demo_config() -> DemoConfig {
    DemoConfig {
        title: "Bridge Stress".to_string(),
        scenario: build_bridge_scenario(&BridgeOptions {
            // Keep the bridge footprint and silhouette, but use a much coarser
            // chunk grid so both Rapier and the stress graph stay in a realistic range.
            span_segments: 18,
            width_segments: 6,
            thickness_layers: 1,
            supports_per_side: 3,
            support_width_segments: 1,
            support_depth_segments: 1,
            ..BridgeOptions::default()
        }),
        node_meshes: Arc::from(Vec::<SceneMeshAsset>::new()),
        projectile_radius: 0.4,
        projectile_mass: 1_000.0,
        projectile_speed: 20.0,
        projectile_ttl: DEFAULT_PROJECTILE_TTL,
        gravity: GRAVITY,
        material_scale: 1.0e10,
        skip_single_bodies: false,
        camera_target: Vec3::new(0.0, 2.5, 0.0),
        camera_distance: 24.0,
        policy: FracturePolicy {
            apply_excess_forces: false,
            ..FracturePolicy::default()
        },
        resimulation: ResimulationOptions {
            enabled: true,
            max_passes: 1,
        },
        sleep_thresholds: SleepThresholdOptions {
            mode: OptimizationMode::AfterGroundCollision,
            linear_threshold: 0.2,
            angular_threshold: 0.2,
        },
        small_body_damping: SmallBodyDampingOptions {
            mode: OptimizationMode::AfterGroundCollision,
            collider_count_threshold: 4,
            min_linear_damping: 3.0,
            min_angular_damping: 3.0,
        },
        debris_cleanup: DebrisCleanupOptions {
            mode: OptimizationMode::AfterGroundCollision,
            debris_ttl_secs: 6.0,
            max_colliders_for_debris: 3,
        },
        debris_collision_mode: DebrisCollisionMode::All,
    }
}

fn apply_scene_pack(mut base: DemoConfig, pack: LoadedScenePack) -> DemoConfig {
    base.title = pack.title;
    base.scenario = pack.scenario;
    base.node_meshes = Arc::from(pack.node_meshes);
    base.projectile_radius = pack.projectile_radius;
    base.projectile_mass = pack.projectile_mass;
    base.projectile_speed = pack.projectile_speed;
    base.projectile_ttl = pack.projectile_ttl_secs;
    base.gravity = pack.gravity;
    base.material_scale = pack.material_scale;
    base.skip_single_bodies = pack.skip_single_bodies;
    base.camera_target = pack.camera_target;
    base.camera_distance = pack.camera_distance;
    base.small_body_damping = pack.small_body_damping;
    base.debris_cleanup = pack.debris_cleanup;
    base
}

fn apply_demo_runtime_defaults(mut config: DemoConfig) -> DemoConfig {
    // Keep every Rust demo preset on a single-pass resim and full debris collisions.
    // Runtime env overrides still apply later in `DemoRuntimeToggles::from_env`.
    config.resimulation = ResimulationOptions {
        enabled: true,
        max_passes: 1,
    };
    config.debris_collision_mode = DebrisCollisionMode::All;
    config
}

fn build_demo_config(kind: DemoScenarioKind) -> DemoConfig {
    let config = match kind {
        DemoScenarioKind::Wall => build_wall_demo_config(),
        DemoScenarioKind::Tower => build_tower_demo_config(),
        DemoScenarioKind::Bridge => build_bridge_demo_config(),
        DemoScenarioKind::FracturedWall => apply_scene_pack(
            build_wall_demo_config(),
            load_embedded_scene_pack(EmbeddedSceneKey::FracturedWall)
                .expect("failed to load fractured wall scene pack"),
        ),
        DemoScenarioKind::FracturedTower => apply_scene_pack(
            build_tower_demo_config(),
            load_embedded_scene_pack(EmbeddedSceneKey::FracturedTower)
                .expect("failed to load fractured tower scene pack"),
        ),
        DemoScenarioKind::FracturedBridge => apply_scene_pack(
            build_bridge_demo_config(),
            load_embedded_scene_pack(EmbeddedSceneKey::FracturedBridge)
                .expect("failed to load fractured bridge scene pack"),
        ),
        DemoScenarioKind::BrickBuilding => apply_scene_pack(
            build_tower_demo_config(),
            load_embedded_scene_pack(EmbeddedSceneKey::BrickBuilding)
                .expect("failed to load brick building scene pack"),
        ),
    };
    apply_demo_runtime_defaults(config)
}

fn build_demo_physics(config: DemoConfig, toggles: &DemoRuntimeToggles) -> DemoPhysicsState {
    let settings = scaled_solver_settings(config.material_scale);
    let gravity = SolverVec3::new(0.0, config.gravity, 0.0);
    let mut destructible = blast_stress_solver::rapier::DestructibleSet::from_scenario(
        &config.scenario,
        settings,
        gravity,
        config.policy,
    )
    .expect("failed to create destructible set");
    destructible.set_resimulation_options(config.resimulation);
    destructible.set_sleep_thresholds(config.sleep_thresholds);
    destructible.set_small_body_damping(config.small_body_damping);
    destructible.set_debris_cleanup(config.debris_cleanup);
    destructible.set_debris_collision_mode(config.debris_collision_mode);
    destructible.set_skip_single_bodies(config.skip_single_bodies);
    destructible.set_dynamic_body_ccd_enabled(toggles.body_ccd_enabled);
    destructible.set_split_child_recentering_enabled(toggles.split_child_recentering_enabled);
    destructible.set_split_child_velocity_fit_enabled(toggles.split_child_velocity_fit_enabled);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let rapier_only = if toggles.rapier_only {
        Some(initialize_rapier_only_bodies(
            &config.scenario,
            &mut bodies,
            &mut colliders,
            toggles.body_ccd_enabled,
        ))
    } else {
        let _handles = destructible.initialize(&mut bodies, &mut colliders);
        None
    };

    let ground_body = bodies.insert(RigidBodyBuilder::fixed().translation(vector![0.0, 0.0, 0.0]));
    let ground = ColliderBuilder::cuboid(100.0, 0.025, 100.0)
        .translation(vector![0.0, -0.025, 0.0])
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .friction(0.9)
        .restitution(0.0);
    colliders.insert_with_parent(ground, ground_body, &mut bodies);
    if !toggles.rapier_only {
        destructible.set_ground_body_handle(Some(ground_body));
        destructible.refresh_collision_groups(&bodies, &mut colliders);
    }
    let runtime_options = DestructionRuntimeOptions {
        contact_impacts: ContactImpactOptions {
            enabled: toggles.contact_force_injection_enabled,
            force_scale: CONTACT_FORCE_SCALE,
            splash_radius: SPLASH_RADIUS,
            ..ContactImpactOptions::default()
        },
        grace: GracePeriodOptions {
            sibling_steps: toggles.sibling_grace_steps,
            impact_source_steps: toggles.projectile_fracture_grace_steps,
        },
    };
    let destructible = DestructionRuntime::new(destructible, runtime_options);

    let mut integration_parameters = IntegrationParameters::default();
    integration_parameters.dt = 1.0 / 60.0;

    DemoPhysicsState {
        config,
        physics_pipeline: PhysicsPipeline::new(),
        integration_parameters,
        island_manager: IslandManager::new(),
        broad_phase: BroadPhaseBvh::new(),
        narrow_phase: NarrowPhase::new(),
        bodies,
        colliders,
        impulse_joints: ImpulseJointSet::new(),
        multibody_joints: MultibodyJointSet::new(),
        ccd_solver: CCDSolver::new(),
        destructible,
        rapier_only,
        node_to_entity: HashMap::new(),
        projectile_entities: HashMap::new(),
        projectile_colliders: HashMap::new(),
        projectiles: Vec::new(),
        projectile_run_stats: ProjectileRunStats::default(),
    }
}

#[derive(Clone, Copy, Debug)]
struct Bounds3 {
    min: Vec3,
    max: Vec3,
}

impl Bounds3 {
    fn center(self) -> Vec3 {
        (self.min + self.max) * 0.5
    }

    fn size(self) -> Vec3 {
        self.max - self.min
    }
}

#[derive(Clone, Copy, Debug)]
struct ScenarioBounds {
    all: Bounds3,
    dynamic: Bounds3,
}

fn build_headless_shot_plan(
    kind: DemoScenarioKind,
    config: &DemoConfig,
) -> Option<HeadlessShotPlan> {
    let script_name = env::var("BLAST_STRESS_DEMO_HEADLESS_SHOT_SCRIPT")
        .ok()
        .map(|value| value.trim().to_ascii_lowercase())
        .filter(|value| !value.is_empty())?;
    let bounds = compute_scenario_bounds(&config.scenario);
    let shots = match script_name.as_str() {
        "wall_smoke" => build_wall_smoke_shots(config, bounds),
        "wall_face_heavy" => build_wall_face_heavy_shots(config, bounds),
        "tower_smoke" => build_tower_smoke_shots(config, bounds),
        "bridge_smoke" => build_bridge_smoke_shots(config, bounds),
        "building_smoke" => build_building_smoke_shots(config, bounds),
        "wall_benchmark" => build_wall_benchmark_shots(config, bounds),
        "tower_benchmark" => build_tower_benchmark_shots(config, bounds),
        "bridge_benchmark" => build_bridge_benchmark_shots(config, bounds),
        "building_benchmark" => build_building_benchmark_shots(config, bounds),
        "auto_smoke" => match kind {
            DemoScenarioKind::Wall | DemoScenarioKind::FracturedWall => {
                build_wall_smoke_shots(config, bounds)
            }
            DemoScenarioKind::Tower | DemoScenarioKind::FracturedTower => {
                build_tower_smoke_shots(config, bounds)
            }
            DemoScenarioKind::Bridge | DemoScenarioKind::FracturedBridge => {
                build_bridge_smoke_shots(config, bounds)
            }
            DemoScenarioKind::BrickBuilding => build_building_smoke_shots(config, bounds),
        },
        "auto_benchmark" => match kind {
            DemoScenarioKind::Wall | DemoScenarioKind::FracturedWall => {
                build_wall_benchmark_shots(config, bounds)
            }
            DemoScenarioKind::Tower | DemoScenarioKind::FracturedTower => {
                build_tower_benchmark_shots(config, bounds)
            }
            DemoScenarioKind::Bridge | DemoScenarioKind::FracturedBridge => {
                build_bridge_benchmark_shots(config, bounds)
            }
            DemoScenarioKind::BrickBuilding => build_building_benchmark_shots(config, bounds),
        },
        _ => return None,
    };
    Some(HeadlessShotPlan {
        script_name,
        shots,
        next_index: 0,
        fired: 0,
    })
}

fn compute_scenario_bounds(scenario: &ScenarioDesc) -> ScenarioBounds {
    let mut all_min = Vec3::splat(f32::INFINITY);
    let mut all_max = Vec3::splat(f32::NEG_INFINITY);
    let mut dynamic_min = Vec3::splat(f32::INFINITY);
    let mut dynamic_max = Vec3::splat(f32::NEG_INFINITY);

    for (index, node) in scenario.nodes.iter().enumerate() {
        let center = Vec3::new(node.centroid.x, node.centroid.y, node.centroid.z);
        let size = scenario.node_sizes.get(index).copied().unwrap_or_else(|| {
            let side = node.volume.cbrt().max(0.05);
            SolverVec3::new(side, side, side)
        });
        let half = Vec3::new(size.x, size.y, size.z) * 0.5;
        let node_min = center - half;
        let node_max = center + half;
        all_min = all_min.min(node_min);
        all_max = all_max.max(node_max);
        if node.mass > 0.0 {
            dynamic_min = dynamic_min.min(node_min);
            dynamic_max = dynamic_max.max(node_max);
        }
    }

    ScenarioBounds {
        all: Bounds3 {
            min: all_min,
            max: all_max,
        },
        dynamic: Bounds3 {
            min: dynamic_min,
            max: dynamic_max,
        },
    }
}

fn make_headless_shot(
    frame: u32,
    label: impl Into<String>,
    target: Vec3,
    travel_direction: Vec3,
    distance: f32,
    mass: f32,
    speed: f32,
    ttl: f32,
) -> HeadlessShot {
    let direction = travel_direction.normalize_or_zero();
    let origin = target - direction * distance;
    HeadlessShot {
        frame,
        label: label.into(),
        origin,
        target,
        mass,
        speed,
        ttl,
        exit_distance: None,
        ballistic_arc: true,
    }
}

fn make_headless_shot_through_bounds(
    frame: u32,
    label: impl Into<String>,
    target: Vec3,
    travel_direction: Vec3,
    distance: f32,
    mass: f32,
    speed: f32,
    ttl: f32,
    bounds: Bounds3,
) -> HeadlessShot {
    let mut shot = make_headless_shot(
        frame,
        label,
        target,
        travel_direction,
        distance,
        mass,
        speed,
        ttl,
    );
    let direction = (shot.target - shot.origin).normalize_or_zero();
    shot.exit_distance = ray_box_exit_distance(shot.origin, direction, bounds);
    shot
}

fn nearest_dynamic_node_center(config: &DemoConfig, desired: Vec3) -> Vec3 {
    config
        .scenario
        .nodes
        .iter()
        .filter(|node| node.mass > 0.0)
        .map(|node| Vec3::new(node.centroid.x, node.centroid.y, node.centroid.z))
        .min_by(|a, b| {
            a.distance_squared(desired)
                .total_cmp(&b.distance_squared(desired))
        })
        .unwrap_or(desired)
}

fn support_top_y(config: &DemoConfig, fallback: f32) -> f32 {
    config
        .scenario
        .nodes
        .iter()
        .filter(|node| node.mass == 0.0)
        .map(|node| node.centroid.y)
        .max_by(|a, b| a.total_cmp(b))
        .unwrap_or(fallback)
}

fn build_wall_smoke_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let z_distance = s.z.max(0.5) + 4.0;
    vec![
        make_headless_shot(
            12,
            "wall-low-center",
            Vec3::new(c.x, bounds.dynamic.min.y + s.y * 0.28, c.z),
            Vec3::new(0.0, -0.02, -1.0),
            z_distance,
            config.projectile_mass * 0.85,
            config.projectile_speed,
            config.projectile_ttl,
        ),
        make_headless_shot(
            32,
            "wall-mid-left",
            Vec3::new(c.x - s.x * 0.18, bounds.dynamic.min.y + s.y * 0.55, c.z),
            Vec3::new(0.06, -0.01, -1.0),
            z_distance + 0.5,
            config.projectile_mass * 1.15,
            config.projectile_speed * 1.05,
            config.projectile_ttl,
        ),
        make_headless_shot(
            52,
            "wall-upper-right",
            Vec3::new(c.x + s.x * 0.16, bounds.dynamic.min.y + s.y * 0.78, c.z),
            Vec3::new(-0.08, -0.04, -1.0),
            z_distance + 1.0,
            config.projectile_mass * 1.35,
            config.projectile_speed * 1.1,
            config.projectile_ttl,
        ),
    ]
}

fn build_wall_face_heavy_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let z_distance = s.z.max(0.5) + 4.0;
    let mut shot = make_headless_shot_through_bounds(
        12,
        "wall-face-heavy",
        Vec3::new(c.x, bounds.dynamic.min.y + s.y * 0.52, c.z),
        Vec3::new(0.0, 0.0, -1.0),
        z_distance,
        config.projectile_mass * 128.0,
        config.projectile_speed * 4.0,
        config.projectile_ttl,
        bounds.dynamic,
    );
    shot.ballistic_arc = false;
    vec![shot]
}

fn build_tower_smoke_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let radius = s.x.max(s.z) + 4.0;
    vec![
        make_headless_shot(
            16,
            "tower-lower-diagonal",
            Vec3::new(c.x - s.x * 0.18, bounds.dynamic.min.y + s.y * 0.22, c.z),
            Vec3::new(-1.0, -0.05, -0.25),
            radius,
            config.projectile_mass * 1.0,
            config.projectile_speed,
            config.projectile_ttl,
        ),
        make_headless_shot(
            42,
            "tower-mid-opposite",
            Vec3::new(
                c.x + s.x * 0.12,
                bounds.dynamic.min.y + s.y * 0.45,
                c.z + s.z * 0.08,
            ),
            Vec3::new(0.35, -0.02, 1.0),
            radius + 0.5,
            config.projectile_mass * 1.4,
            config.projectile_speed * 1.05,
            config.projectile_ttl,
        ),
        make_headless_shot(
            70,
            "tower-upper-diagonal",
            Vec3::new(c.x, bounds.dynamic.min.y + s.y * 0.68, c.z - s.z * 0.12),
            Vec3::new(-0.9, -0.08, 0.4),
            radius + 1.0,
            config.projectile_mass * 1.8,
            config.projectile_speed * 1.1,
            config.projectile_ttl,
        ),
    ]
}

fn build_bridge_smoke_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let z_distance = s.z.max(1.0) + 3.5;
    let support_y = support_top_y(config, bounds.all.min.y);
    let midspan_target =
        nearest_dynamic_node_center(config, Vec3::new(c.x, support_y + s.y * 0.22, c.z));
    let left_joint_target = nearest_dynamic_node_center(
        config,
        Vec3::new(
            bounds.dynamic.min.x + s.x * 0.22,
            support_y + s.y * 0.10,
            c.z,
        ),
    );
    let right_joint_target = nearest_dynamic_node_center(
        config,
        Vec3::new(
            bounds.dynamic.max.x - s.x * 0.22,
            support_y + s.y * 0.10,
            c.z - s.z * 0.08,
        ),
    );
    vec![
        make_headless_shot(
            18,
            "bridge-midspan",
            midspan_target,
            Vec3::new(0.0, -0.04, -1.0),
            z_distance,
            config.projectile_mass * 8.0,
            config.projectile_speed * 1.25,
            config.projectile_ttl,
        ),
        make_headless_shot(
            48,
            "bridge-left-joint",
            left_joint_target,
            Vec3::new(0.18, -0.02, -1.0),
            z_distance,
            config.projectile_mass * 12.0,
            config.projectile_speed * 1.35,
            config.projectile_ttl,
        ),
        make_headless_shot(
            82,
            "bridge-right-joint",
            right_joint_target,
            Vec3::new(-0.95, -0.03, 0.32),
            z_distance + 0.75,
            config.projectile_mass * 16.0,
            config.projectile_speed * 1.45,
            config.projectile_ttl,
        ),
    ]
}

fn build_building_smoke_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let size = bounds.dynamic.size();
    let front_z = bounds.dynamic.min.z + 0.25;
    let diagonal_distance = size.x.max(size.z) + 5.0;
    let forward_distance = size.z.max(0.5) + 5.0;
    let door_target = nearest_dynamic_node_center(config, Vec3::new(0.0, 0.9, front_z));
    let upper_window_target = nearest_dynamic_node_center(config, Vec3::new(1.5, 2.8, front_z));
    let parapet_target = nearest_dynamic_node_center(
        config,
        Vec3::new(
            bounds.dynamic.max.x - 0.35,
            bounds.dynamic.max.y - 0.35,
            bounds.dynamic.max.z - 0.35,
        ),
    );

    vec![
        make_headless_shot(
            14,
            "building-door-breach",
            door_target,
            Vec3::new(0.04, -0.01, 1.0),
            forward_distance,
            config.projectile_mass * 1.0,
            config.projectile_speed,
            config.projectile_ttl,
        ),
        make_headless_shot(
            38,
            "building-upper-window",
            upper_window_target,
            Vec3::new(-0.03, -0.05, 1.0),
            forward_distance + 0.75,
            config.projectile_mass * 1.15,
            config.projectile_speed * 1.05,
            config.projectile_ttl,
        ),
        make_headless_shot(
            64,
            "building-parapet-corner",
            parapet_target,
            Vec3::new(0.9, -0.08, 0.95),
            diagonal_distance,
            config.projectile_mass * 1.35,
            config.projectile_speed * 1.1,
            config.projectile_ttl,
        ),
    ]
}

fn build_wall_benchmark_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let mut shots = build_wall_smoke_shots(config, bounds);
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let z_distance = s.z.max(0.5) + 4.5;
    shots.extend([
        make_headless_shot(
            76,
            "wall-benchmark-center",
            Vec3::new(c.x, bounds.dynamic.min.y + s.y * 0.52, c.z),
            Vec3::new(0.0, -0.02, -1.0),
            z_distance,
            config.projectile_mass * 1.75,
            config.projectile_speed * 1.1,
            config.projectile_ttl,
        ),
        make_headless_shot(
            98,
            "wall-benchmark-shear",
            Vec3::new(c.x - s.x * 0.25, bounds.dynamic.min.y + s.y * 0.38, c.z),
            Vec3::new(0.18, 0.0, -1.0),
            z_distance + 0.5,
            config.projectile_mass * 2.0,
            config.projectile_speed * 1.15,
            config.projectile_ttl,
        ),
    ]);
    shots
}

fn build_tower_benchmark_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let mut shots = build_tower_smoke_shots(config, bounds);
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let radius = s.x.max(s.z) + 4.5;
    shots.extend([
        make_headless_shot(
            104,
            "tower-benchmark-base",
            Vec3::new(c.x + s.x * 0.22, bounds.dynamic.min.y + s.y * 0.16, c.z),
            Vec3::new(1.0, -0.03, -0.15),
            radius,
            config.projectile_mass * 2.1,
            config.projectile_speed * 1.08,
            config.projectile_ttl,
        ),
        make_headless_shot(
            132,
            "tower-benchmark-mid",
            Vec3::new(c.x, bounds.dynamic.min.y + s.y * 0.52, c.z - s.z * 0.18),
            Vec3::new(-0.25, -0.05, 1.0),
            radius + 1.0,
            config.projectile_mass * 2.6,
            config.projectile_speed * 1.12,
            config.projectile_ttl,
        ),
    ]);
    shots
}

fn build_bridge_benchmark_shots(config: &DemoConfig, bounds: ScenarioBounds) -> Vec<HeadlessShot> {
    let mut shots = build_bridge_smoke_shots(config, bounds);
    let c = bounds.dynamic.center();
    let s = bounds.dynamic.size();
    let z_distance = s.z.max(1.0) + 3.75;
    let support_y = support_top_y(config, bounds.all.min.y);
    let underside_target =
        nearest_dynamic_node_center(config, Vec3::new(c.x, support_y + s.y * 0.12, c.z));
    let span_joint_target = nearest_dynamic_node_center(
        config,
        Vec3::new(
            bounds.dynamic.min.x + s.x * 0.48,
            support_y + s.y * 0.08,
            c.z + s.z * 0.10,
        ),
    );
    let repeat_joint_target = nearest_dynamic_node_center(
        config,
        Vec3::new(
            bounds.dynamic.max.x - s.x * 0.24,
            support_y + s.y * 0.08,
            c.z - s.z * 0.06,
        ),
    );
    shots.extend([
        make_headless_shot(
            110,
            "bridge-benchmark-under-midspan",
            underside_target,
            Vec3::new(0.0, 0.08, -1.0),
            z_distance,
            config.projectile_mass * 18.0,
            config.projectile_speed * 1.45,
            config.projectile_ttl,
        ),
        make_headless_shot(
            144,
            "bridge-benchmark-span-joint",
            span_joint_target,
            Vec3::new(-0.28, -0.02, -1.0),
            z_distance,
            config.projectile_mass * 24.0,
            config.projectile_speed * 1.5,
            config.projectile_ttl,
        ),
        make_headless_shot(
            170,
            "bridge-benchmark-repeat-joint",
            repeat_joint_target,
            Vec3::new(-0.92, -0.02, 0.28),
            z_distance,
            config.projectile_mass * 28.0,
            config.projectile_speed * 1.55,
            config.projectile_ttl,
        ),
    ]);
    shots
}

fn build_building_benchmark_shots(
    config: &DemoConfig,
    bounds: ScenarioBounds,
) -> Vec<HeadlessShot> {
    let size = bounds.dynamic.size();
    let front_z = bounds.dynamic.min.z + 0.25;
    let right_x = bounds.dynamic.max.x - 0.25;
    let forward_distance = size.z.max(0.5) + 5.25;
    let lateral_distance = size.x.max(0.5) + 5.0;
    let diagonal_distance = size.x.max(size.z) + 5.5;
    let parapet_y = bounds.dynamic.max.y - 0.2;

    vec![
        make_headless_shot(
            12,
            "building-benchmark-door-jamb",
            nearest_dynamic_node_center(config, Vec3::new(-0.75, 1.0, front_z)),
            Vec3::new(0.08, -0.01, 1.0),
            forward_distance,
            config.projectile_mass * 1.05,
            config.projectile_speed,
            config.projectile_ttl,
        ),
        make_headless_shot(
            30,
            "building-benchmark-front-window",
            nearest_dynamic_node_center(config, Vec3::new(1.5, 2.85, front_z)),
            Vec3::new(-0.04, -0.04, 1.0),
            forward_distance + 0.5,
            config.projectile_mass * 1.2,
            config.projectile_speed * 1.02,
            config.projectile_ttl,
        ),
        make_headless_shot(
            48,
            "building-benchmark-side-wall",
            nearest_dynamic_node_center(config, Vec3::new(right_x, 1.8, 0.0)),
            Vec3::new(1.0, -0.03, 0.12),
            lateral_distance,
            config.projectile_mass * 1.2,
            config.projectile_speed * 1.05,
            config.projectile_ttl,
        ),
        make_headless_shot(
            66,
            "building-benchmark-upper-corner",
            nearest_dynamic_node_center(
                config,
                Vec3::new(bounds.dynamic.max.x - 0.3, 3.2, bounds.dynamic.max.z - 0.3),
            ),
            Vec3::new(0.85, -0.06, 0.9),
            diagonal_distance,
            config.projectile_mass * 1.35,
            config.projectile_speed * 1.08,
            config.projectile_ttl,
        ),
        make_headless_shot(
            84,
            "building-benchmark-parapet",
            nearest_dynamic_node_center(config, Vec3::new(0.0, parapet_y, front_z + 0.5)),
            Vec3::new(0.1, -0.15, 1.0),
            forward_distance + 1.0,
            config.projectile_mass * 1.5,
            config.projectile_speed * 1.12,
            config.projectile_ttl,
        ),
    ]
}

fn initialize_rapier_only_bodies(
    scenario: &ScenarioDesc,
    bodies: &mut RigidBodySet,
    colliders: &mut ColliderSet,
    body_ccd_enabled: bool,
) -> RapierOnlyState {
    let mut node_to_body = vec![None; scenario.nodes.len()];
    let mut node_local_offsets = vec![SolverVec3::ZERO; scenario.nodes.len()];
    let mut support_body_count = 0usize;

    for (node_index, node) in scenario.nodes.iter().enumerate() {
        let size = scenario
            .node_sizes
            .get(node_index)
            .copied()
            .unwrap_or_else(|| {
                let side = node.volume.cbrt().max(0.01);
                SolverVec3::new(side, side, side)
            });
        let translation = vector![node.centroid.x, node.centroid.y, node.centroid.z];
        let body = if node.mass == 0.0 {
            support_body_count += 1;
            bodies.insert(RigidBodyBuilder::fixed().translation(translation))
        } else {
            bodies.insert(
                RigidBodyBuilder::dynamic()
                    .translation(translation)
                    .ccd_enabled(body_ccd_enabled),
            )
        };

        let collider = build_rapier_only_node_collider(scenario, node_index, size, node.mass);
        colliders.insert_with_parent(collider, body, bodies);

        node_to_body[node_index] = Some(body);
        node_local_offsets[node_index] = SolverVec3::ZERO;
    }

    RapierOnlyState {
        node_to_body,
        node_local_offsets,
        support_body_count,
    }
}

fn build_rapier_only_node_collider(
    scenario: &ScenarioDesc,
    node_index: usize,
    fallback_size: SolverVec3,
    mass: f32,
) -> ColliderBuilder {
    let fallback = ColliderBuilder::cuboid(
        fallback_size.x * 0.5,
        fallback_size.y * 0.5,
        fallback_size.z * 0.5,
    );
    let base = match scenario
        .collider_shapes
        .get(node_index)
        .and_then(Option::as_ref)
    {
        Some(ScenarioCollider::Cuboid { half_extents }) => {
            ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
        }
        Some(ScenarioCollider::ConvexHull { points }) => {
            let hull_points: Vec<Point<f32>> = points
                .iter()
                .map(|point| point![point.x, point.y, point.z])
                .collect();
            ColliderBuilder::convex_hull(&hull_points).unwrap_or(fallback)
        }
        None => fallback,
    };

    base.active_events(ActiveEvents::CONTACT_FORCE_EVENTS | ActiveEvents::COLLISION_EVENTS)
        .contact_force_event_threshold(0.0)
        .friction(0.25)
        .restitution(0.0)
        .mass(mass.max(0.0))
}

fn demo_node_body(state: &DemoPhysicsState, node_index: u32) -> Option<RigidBodyHandle> {
    if let Some(rapier_only) = &state.rapier_only {
        rapier_only
            .node_to_body
            .get(node_index as usize)
            .copied()
            .flatten()
    } else {
        state.destructible.node_body(node_index)
    }
}

fn demo_node_local_offset(state: &DemoPhysicsState, node_index: u32) -> Option<SolverVec3> {
    if let Some(rapier_only) = &state.rapier_only {
        rapier_only
            .node_local_offsets
            .get(node_index as usize)
            .copied()
    } else {
        state.destructible.node_local_offset(node_index)
    }
}

fn demo_destructible_body_handles(state: &DemoPhysicsState) -> Vec<RigidBodyHandle> {
    let mut handles = std::collections::HashSet::new();
    for node_index in 0..state.config.scenario.nodes.len() {
        if let Some(handle) = demo_node_body(state, node_index as u32) {
            handles.insert(handle);
        }
    }
    handles.into_iter().collect()
}

fn demo_actor_count(state: &DemoPhysicsState) -> usize {
    if state.rapier_only.is_some() {
        demo_destructible_body_handles(state).len()
    } else {
        state.destructible.actor_count() as usize
    }
}

fn demo_destructible_body_count(state: &DemoPhysicsState) -> usize {
    if state.rapier_only.is_some() {
        demo_destructible_body_handles(state).len()
    } else {
        state.destructible.body_count()
    }
}

fn demo_support_body_count(state: &DemoPhysicsState) -> usize {
    if let Some(rapier_only) = &state.rapier_only {
        rapier_only.support_body_count
    } else {
        state.destructible.support_body_count()
    }
}

fn demo_dynamic_body_count(state: &DemoPhysicsState) -> usize {
    demo_destructible_body_handles(state)
        .into_iter()
        .filter(|&handle| {
            state
                .bodies
                .get(handle)
                .is_some_and(|body| body.is_dynamic())
        })
        .count()
}

fn demo_awake_dynamic_body_count(state: &DemoPhysicsState) -> usize {
    demo_destructible_body_handles(state)
        .into_iter()
        .filter(|&handle| {
            state
                .bodies
                .get(handle)
                .is_some_and(|body| body.is_dynamic() && !body.is_sleeping())
        })
        .count()
}

fn demo_sleeping_dynamic_body_count(state: &DemoPhysicsState) -> usize {
    demo_destructible_body_handles(state)
        .into_iter()
        .filter(|&handle| {
            state
                .bodies
                .get(handle)
                .is_some_and(|body| body.is_dynamic() && body.is_sleeping())
        })
        .count()
}

fn demo_destructible_collider_count(state: &DemoPhysicsState) -> usize {
    if state.rapier_only.is_some() {
        demo_destructible_body_handles(state)
            .into_iter()
            .map(|handle| {
                state
                    .bodies
                    .get(handle)
                    .map_or(0usize, |body| body.colliders().len())
            })
            .sum()
    } else {
        state.destructible.collider_count()
    }
}

fn scaled_solver_settings(material_scale: f32) -> SolverSettings {
    SolverSettings {
        max_solver_iterations_per_frame: 24,
        graph_reduction_level: 0,
        compression_elastic_limit: BASE_COMPRESSION_ELASTIC * material_scale,
        compression_fatal_limit: BASE_COMPRESSION_FATAL * material_scale,
        tension_elastic_limit: BASE_TENSION_ELASTIC * material_scale,
        tension_fatal_limit: BASE_TENSION_FATAL * material_scale,
        shear_elastic_limit: BASE_SHEAR_ELASTIC * material_scale,
        shear_fatal_limit: BASE_SHEAR_FATAL * material_scale,
    }
}

fn headless_frame_budget() -> u32 {
    env::var("BLAST_STRESS_DEMO_HEADLESS_FRAMES")
        .ok()
        .and_then(|value| value.parse::<u32>().ok())
        .filter(|&frames| frames > 0)
        .unwrap_or(DEFAULT_HEADLESS_FRAMES)
}

fn should_start_headless() -> bool {
    match env_flag("BLAST_STRESS_DEMO_HEADLESS") {
        Some(value) => value,
        None => is_headless_environment(),
    }
}

fn env_flag(name: &str) -> Option<bool> {
    let value = env::var(name).ok()?;
    let normalized = value.trim().to_ascii_lowercase();
    match normalized.as_str() {
        "1" | "true" | "yes" | "on" => Some(true),
        "0" | "false" | "no" | "off" => Some(false),
        _ => None,
    }
}

fn env_usize(name: &str) -> Option<usize> {
    env::var(name)
        .ok()
        .and_then(|value| value.trim().parse::<usize>().ok())
}

fn env_i32(name: &str) -> Option<i32> {
    env::var(name)
        .ok()
        .and_then(|value| value.trim().parse::<i32>().ok())
}

fn env_f32(name: &str) -> Option<f32> {
    env::var(name)
        .ok()
        .and_then(|value| value.trim().parse::<f32>().ok())
}

fn env_debris_collision_mode(name: &str) -> Option<DebrisCollisionMode> {
    let value = env::var(name).ok()?;
    match value.trim() {
        "all" => Some(DebrisCollisionMode::All),
        "noDebrisPairs" => Some(DebrisCollisionMode::NoDebrisPairs),
        "debrisGroundOnly" => Some(DebrisCollisionMode::DebrisGroundOnly),
        "debrisNone" => Some(DebrisCollisionMode::DebrisNone),
        _ => None,
    }
}

fn flag_bit(value: bool) -> u8 {
    if value {
        1
    } else {
        0
    }
}

fn optimization_mode_label(mode: OptimizationMode) -> &'static str {
    match mode {
        OptimizationMode::Off => "off",
        OptimizationMode::Always => "always",
        OptimizationMode::AfterGroundCollision => "after_ground_collision",
    }
}

fn debris_collision_mode_label(mode: DebrisCollisionMode) -> &'static str {
    mode.as_str()
}

fn mesh_visuals_enabled() -> bool {
    env_flag("BLAST_STRESS_DEMO_SHOW_MESHES").unwrap_or(false)
}

fn is_missing_gpu_panic(payload: &(dyn Any + Send)) -> bool {
    panic_message(payload).is_some_and(|message| message.contains("Unable to find a GPU"))
}

fn panic_message(payload: &(dyn Any + Send)) -> Option<&str> {
    if let Some(message) = payload.downcast_ref::<&'static str>() {
        Some(*message)
    } else if let Some(message) = payload.downcast_ref::<String>() {
        Some(message.as_str())
    } else {
        None
    }
}

fn restore_panic_hook(default_hook: Arc<Box<dyn Fn(&panic::PanicHookInfo<'_>) + Sync + Send>>) {
    panic::set_hook(Box::new(move |info| {
        (default_hook)(info);
    }));
}

#[cfg(target_os = "linux")]
fn is_headless_environment() -> bool {
    env::var_os("DISPLAY").is_none()
        && env::var_os("WAYLAND_DISPLAY").is_none()
        && env::var_os("WAYLAND_SOCKET").is_none()
}

#[cfg(not(target_os = "linux"))]
fn is_headless_environment() -> bool {
    false
}

fn setup_scene(
    mut commands: Commands,
    info: Res<DemoInfo>,
    orbit: Res<CameraOrbit>,
    mut ambient_light: ResMut<GlobalAmbientLight>,
) {
    ambient_light.color = Color::srgb(0.92, 0.94, 1.0);
    ambient_light.brightness = 700.0;

    commands.spawn((Camera3d::default(), camera_transform(&orbit), MainCamera));

    commands.spawn((
        PointLight {
            color: Color::WHITE,
            intensity: 18_000_000.0,
            range: 80.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(8.0, 16.0, 10.0),
    ));

    commands.spawn((
        PointLight {
            color: Color::srgb(0.82, 0.90, 1.0),
            intensity: 8_000_000.0,
            range: 80.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-10.0, 8.0, -8.0),
    ));

    commands.spawn((
        Text::new(format!(
            "{}\nLeft click: Shoot  |  Right drag or Ctrl+Left drag: Orbit  |  Scroll: Zoom  |  R: Reset  |  [-]/[=]: Projectile Mass",
            info.title
        )),
        TextFont {
            font_size: 16.0,
            ..default()
        },
        TextColor(Color::WHITE),
        Node {
            position_type: PositionType::Absolute,
            left: Val::Px(12.0),
            top: Val::Px(12.0),
            ..default()
        },
        HudText,
    ));
}

fn camera_transform(orbit: &CameraOrbit) -> Transform {
    let x = orbit.distance * orbit.yaw.cos() * orbit.pitch.cos();
    let y = orbit.distance * orbit.pitch.sin();
    let z = orbit.distance * orbit.yaw.sin() * orbit.pitch.cos();
    Transform::from_translation(orbit.target + Vec3::new(x, y, z)).looking_at(orbit.target, Vec3::Y)
}

fn setup_visuals(
    mut commands: Commands,
    mode: Res<RunMode>,
    toggles: Res<DemoRuntimeToggles>,
    mut state: NonSendMut<DemoPhysicsState>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
) {
    if mode.headless {
        return;
    }

    let show_meshes = toggles.show_meshes;
    let mut meshes = meshes;
    let mut materials = materials;

    if show_meshes {
        if let (Some(meshes), Some(materials)) = (&mut meshes, &mut materials) {
            commands.spawn((
                Mesh3d(meshes.add(BevyCuboid::new(30.0, 0.05, 30.0))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.18, 0.22, 0.28),
                    perceptual_roughness: 0.85,
                    metallic: 0.08,
                    ..default()
                })),
                Transform::from_xyz(0.0, -0.025, 0.0),
            ));
        }
    }

    spawn_chunk_visuals(
        &mut commands,
        &mut state,
        show_meshes,
        &mut meshes,
        &mut materials,
    );
}

fn spawn_chunk_visuals(
    commands: &mut Commands,
    state: &mut DemoPhysicsState,
    show_meshes: bool,
    meshes: &mut Option<ResMut<Assets<Mesh>>>,
    materials: &mut Option<ResMut<Assets<StandardMaterial>>>,
) {
    for node_index in 0..state.config.scenario.nodes.len() {
        let Some((translation, rotation)) = node_world_transform(&state, node_index as u32) else {
            continue;
        };
        let node = &state.config.scenario.nodes[node_index];
        let size = state
            .config
            .scenario
            .node_sizes
            .get(node_index)
            .copied()
            .unwrap_or_else(|| {
                let side = node.volume.cbrt().max(0.05);
                SolverVec3::new(side, side, side)
            });

        let is_support = node.mass == 0.0;
        let color = demo_node_body(state, node_index as u32)
            .map(|handle| body_color(handle, is_support))
            .unwrap_or_else(|| default_chunk_color(is_support));
        let mut entity = commands.spawn((
            Transform {
                translation,
                rotation,
                ..default()
            },
            ChunkVisual {
                node_index: node_index as u32,
                gizmo_shape: state
                    .config
                    .node_meshes
                    .get(node_index)
                    .map(|mesh| ChunkGizmoShape::TriEdges {
                        edges: Arc::from(build_mesh_gizmo_edges(mesh)),
                    })
                    .unwrap_or_else(|| ChunkGizmoShape::Box {
                        size: Vec3::new(size.x, size.y, size.z),
                    }),
                is_support,
            },
            ChunkTint { color },
        ));

        if show_meshes {
            if let (Some(meshes), Some(materials)) = (meshes.as_mut(), materials.as_mut()) {
                let material_handle = materials.add(StandardMaterial {
                    base_color: color,
                    perceptual_roughness: 0.7,
                    metallic: 0.04,
                    ..default()
                });
                let mesh_handle = if let Some(mesh_asset) = state.config.node_meshes.get(node_index)
                {
                    meshes.add(mesh_asset.to_bevy_mesh())
                } else {
                    meshes.add(BevyCuboid::new(size.x * 0.98, size.y * 0.98, size.z * 0.98))
                };
                entity.insert((
                    Mesh3d(mesh_handle),
                    MeshMaterial3d(material_handle.clone()),
                    ChunkMaterial {
                        handle: material_handle,
                    },
                ));
            }
        }

        state.node_to_entity.insert(node_index as u32, entity.id());
    }
}

fn camera_orbit_system(
    mouse_button: Res<ButtonInput<MouseButton>>,
    keyboard: Res<ButtonInput<KeyCode>>,
    mut mouse_motion: MessageReader<MouseMotion>,
    mut scroll: MessageReader<MouseWheel>,
    mut orbit: ResMut<CameraOrbit>,
    mut camera_q: Query<&mut Transform, With<MainCamera>>,
) {
    let orbit_drag_active = mouse_button.pressed(MouseButton::Right)
        || (keyboard.any_pressed([KeyCode::ControlLeft, KeyCode::ControlRight])
            && mouse_button.pressed(MouseButton::Left));
    if orbit_drag_active {
        for ev in mouse_motion.read() {
            orbit.yaw -= ev.delta.x * 0.005;
            orbit.pitch -= ev.delta.y * 0.005;
            orbit.pitch = orbit.pitch.clamp(-1.4, 1.4);
        }
    } else {
        for _ in mouse_motion.read() {}
    }

    for ev in scroll.read() {
        orbit.distance -= ev.y * 0.5;
        orbit.distance = orbit.distance.clamp(4.0, 60.0);
    }

    if let Ok(mut tf) = camera_q.single_mut() {
        *tf = camera_transform(&orbit);
    }
}

fn reset_scene_system(
    mut commands: Commands,
    keyboard: Res<ButtonInput<KeyCode>>,
    mode: Res<RunMode>,
    toggles: Res<DemoRuntimeToggles>,
    mut state: NonSendMut<DemoPhysicsState>,
    mut profiler: ResMut<DebugProfiler>,
    chunk_entities: Query<Entity, With<ChunkVisual>>,
    projectile_entities: Query<Entity, With<ProjectileVisual>>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
) {
    if mode.headless || !keyboard.just_pressed(KeyCode::KeyR) {
        return;
    }

    let config = state.config.clone();
    for entity in &chunk_entities {
        commands.entity(entity).despawn();
    }
    for entity in &projectile_entities {
        commands.entity(entity).despawn();
    }

    *state = build_demo_physics(config, &toggles);
    profiler.current = FrameBreakdown::default();

    let mut meshes = meshes;
    let mut materials = materials;
    spawn_chunk_visuals(
        &mut commands,
        &mut state,
        toggles.show_meshes,
        &mut meshes,
        &mut materials,
    );
}

fn projectile_mass_shortcuts_system(
    keyboard: Res<ButtonInput<KeyCode>>,
    mut projectile_settings: ResMut<ProjectileFireSettings>,
) {
    if keyboard.just_pressed(KeyCode::Equal) || keyboard.just_pressed(KeyCode::NumpadAdd) {
        projectile_settings.increase_mass();
    }

    if keyboard.just_pressed(KeyCode::Minus) || keyboard.just_pressed(KeyCode::NumpadSubtract) {
        projectile_settings.decrease_mass();
    }
}

fn spawn_projectile(
    commands: &mut Commands,
    state: &mut DemoPhysicsState,
    toggles: &DemoRuntimeToggles,
    origin: Vec3,
    direction: Vec3,
    projectile_mass: f32,
    projectile_speed: f32,
    projectile_ttl: f32,
    visual_assets: Option<(&mut Assets<Mesh>, &mut Assets<StandardMaterial>)>,
) {
    let direction = direction.normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    let projectile_radius = state.config.projectile_radius;
    let body_handle = state.bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(vector![origin.x, origin.y, origin.z])
            .linvel(vector![
                direction.x * projectile_speed,
                direction.y * projectile_speed,
                direction.z * projectile_speed
            ])
            .ccd_enabled(toggles.projectile_ccd_enabled),
    );

    let collider_handle = state.colliders.insert_with_parent(
        ColliderBuilder::ball(projectile_radius)
            .mass(projectile_mass)
            .friction(0.25)
            .restitution(0.0)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS | ActiveEvents::COLLISION_EVENTS)
            .contact_force_event_threshold(0.0),
        body_handle,
        &mut state.bodies,
    );

    if let Some((meshes, materials)) = visual_assets {
        let entity = commands
            .spawn((
                Transform::from_translation(origin),
                ProjectileVisual {
                    radius: projectile_radius,
                },
                Mesh3d(meshes.add(Sphere::new(projectile_radius))),
                MeshMaterial3d(materials.add(StandardMaterial {
                    base_color: Color::srgb(0.96, 0.84, 0.16),
                    emissive: LinearRgba::from(Color::srgb(0.14, 0.09, 0.02)),
                    perceptual_roughness: 0.35,
                    metallic: 0.18,
                    ..default()
                })),
            ))
            .id();
        state.projectile_entities.insert(body_handle, entity);
    }

    state
        .projectile_colliders
        .insert(collider_handle, body_handle);
    state.projectiles.push(ProjectileState {
        body_handle,
        collider_handle,
        ttl: projectile_ttl,
        origin,
        direction,
        target_distance: 0.0,
        exit_distance: None,
        max_progress: 0.0,
        crossed_target_plane: false,
        passed_through: false,
    });
}

fn auto_fire_headless_projectiles_system(
    mut commands: Commands,
    run_mode: Res<RunMode>,
    shot_plan: Option<ResMut<HeadlessShotPlan>>,
    toggles: Res<DemoRuntimeToggles>,
    mut profiler: ResMut<DebugProfiler>,
    mut state: NonSendMut<DemoPhysicsState>,
) {
    if !run_mode.headless {
        return;
    }

    let Some(mut shot_plan) = shot_plan else {
        return;
    };

    let frame_index = profiler.frame_index;
    for shot in shot_plan.due_shots(frame_index) {
        let direction = shot.launch_direction();
        spawn_projectile(
            &mut commands,
            &mut state,
            &toggles,
            shot.origin,
            direction,
            shot.mass,
            shot.speed,
            shot.ttl,
            None,
        );
        if let Some(projectile) = state.projectiles.last_mut() {
            projectile.origin = shot.origin;
            projectile.direction = direction;
            projectile.target_distance = (shot.target - shot.origin).dot(direction);
            projectile.exit_distance = shot.exit_distance;
        }
        profiler.pending_event_lines.push(format!(
            "[shot-fired] frame={} script={} label={} mass={:.3} speed={:.3} ttl={:.3} origin=({:.3},{:.3},{:.3}) target=({:.3},{:.3},{:.3}) direction=({:.3},{:.3},{:.3})",
            frame_index,
            shot_plan.script_name,
            shot.label,
            shot.mass,
            shot.speed,
            shot.ttl,
            shot.origin.x,
            shot.origin.y,
            shot.origin.z,
            shot.target.x,
            shot.target.y,
            shot.target.z,
            direction.x,
            direction.y,
            direction.z,
        ));
    }
}

fn shoot_projectile_system(
    mut commands: Commands,
    mouse_button: Res<ButtonInput<MouseButton>>,
    camera_q: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
    window_q: Query<&Window, With<PrimaryWindow>>,
    projectile_settings: Res<ProjectileFireSettings>,
    toggles: Res<DemoRuntimeToggles>,
    mut state: NonSendMut<DemoPhysicsState>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !mouse_button.just_pressed(MouseButton::Left) {
        return;
    }

    let Ok((camera, cam_tf)) = camera_q.single() else {
        return;
    };
    let Ok(window) = window_q.single() else {
        return;
    };
    let Some(cursor_pos) = window.cursor_position() else {
        return;
    };
    let Ok(ray) = camera.viewport_to_world(cam_tf, cursor_pos) else {
        return;
    };

    let origin = ray.origin;
    let direction = ray.direction.as_vec3().normalize_or_zero();
    if direction == Vec3::ZERO {
        return;
    }

    let projectile_mass = projectile_settings.mass;
    let projectile_speed = state.config.projectile_speed;
    let projectile_ttl = state.config.projectile_ttl;
    spawn_projectile(
        &mut commands,
        &mut state,
        &toggles,
        origin,
        direction,
        projectile_mass,
        projectile_speed,
        projectile_ttl,
        Some((&mut meshes, &mut materials)),
    );
}

fn physics_step_system(
    mut commands: Commands,
    time: Res<Time>,
    toggles: Res<DemoRuntimeToggles>,
    mut profiler: ResMut<DebugProfiler>,
    mut state: NonSendMut<DemoPhysicsState>,
) {
    let physics_started_at = Instant::now();
    let dt = time.delta_secs().clamp(1.0 / 240.0, 1.0 / 30.0);
    let now_secs = time.elapsed_secs();
    let rapier_only = toggles.rapier_only;
    let mut rapier_passes = 0u32;
    let frame_result = if rapier_only {
        loop {
            let rapier_started_at = Instant::now();
            step_rapier_world(&mut state, dt, &());
            profiler.current.rapier_ms +=
                rapier_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
            rapier_passes += 1;
            break None;
        }
    } else {
        {
            let DemoPhysicsState {
                destructible,
                bodies,
                ..
            } = &mut *state;
            destructible.begin_frame(now_secs, dt, bodies);
        }
        loop {
            let unit_hooks = ();
            let hooks = state.destructible.begin_pass(&unit_hooks);
            let rapier_started_at = Instant::now();
            step_rapier_world(&mut state, dt, &hooks);
            profiler.current.rapier_ms +=
                rapier_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
            rapier_passes += 1;

            let runtime_started_at = Instant::now();
            let directive = {
                let DemoPhysicsState {
                    destructible,
                    bodies,
                    colliders,
                    island_manager,
                    narrow_phase,
                    impulse_joints,
                    multibody_joints,
                    ..
                } = &mut *state;
                let mut world_access = RapierWorldAccess {
                    bodies,
                    colliders,
                    island_manager,
                    narrow_phase,
                    impulse_joints,
                    multibody_joints,
                };
                destructible.finish_pass(&mut world_access)
            };
            profiler.current.solver_ms +=
                runtime_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

            match directive {
                FrameDirective::Resimulate => continue,
                FrameDirective::Done(result) => break Some(result),
            }
        }
    };

    let removed_nodes = frame_result
        .as_ref()
        .map(|result| result.optimization.removed_nodes.clone())
        .unwrap_or_default();

    let removed_node_count = removed_nodes.len();
    for node_index in removed_nodes {
        if let Some(entity) = state.node_to_entity.remove(&node_index) {
            commands.entity(entity).despawn();
        }
    }

    update_projectile_progress(&mut state);
    if toggles.projectile_trace_enabled {
        let frame_index = profiler.frame_index;
        log_projectile_trace(&mut profiler, &state, frame_index);
    }

    let cleanup_started_at = Instant::now();
    let removed_projectiles = cleanup_projectiles(&mut state, &mut commands, dt);
    profiler.current.projectile_cleanup_ms +=
        cleanup_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

    let collision_events = frame_result
        .as_ref()
        .map(|result| result.support_contacts)
        .unwrap_or(0);
    let contact_events = frame_result
        .as_ref()
        .map(|result| result.accepted_impacts)
        .unwrap_or(0);

    profiler.current.physics_ms += physics_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
    profiler.current.rapier_passes += rapier_passes;
    profiler.current.collision_events += collision_events;
    profiler.current.contact_events += contact_events;
    if let Some(result) = frame_result {
        profiler.current.split_sanitize_ms += result.split_sanitize_ms;
        profiler.current.split_estimate_ms += result.split_estimate_ms;
        profiler.current.split_plan_ms += result.split_edits.plan_ms;
        profiler.current.split_apply_ms += result.split_edits.apply_ms;
        profiler.current.split_child_pose_ms += result.split_edits.child_pose_ms;
        profiler.current.split_velocity_fit_ms += result.split_edits.velocity_fit_ms;
        profiler.current.split_sleep_init_ms += result.split_edits.sleep_init_ms;
        profiler.current.split_body_create_ms += result.split_edits.body_create_ms;
        profiler.current.split_collider_move_ms += result.split_edits.collider_move_ms;
        profiler.current.split_collider_insert_ms += result.split_edits.collider_insert_ms;
        profiler.current.split_body_retire_ms += result.split_edits.body_retire_ms;
        profiler.current.resim_restore_ms += result.resim_restore_ms;
        profiler.current.resim_snapshot_ms += result.resim_snapshot_ms;
        profiler.current.fractures += result.fractures;
        profiler.current.split_events += result.split_events;
        profiler.current.split_cohorts += result.split_cohorts.len();
        profiler.current.split_cohort_bodies += result
            .split_cohorts
            .iter()
            .map(|cohort| cohort.target_bodies.len())
            .sum::<usize>();
        profiler.current.reused_bodies += result.split_edits.reused_bodies;
        profiler.current.recycled_bodies += result.split_edits.recycled_bodies;
        profiler.current.new_bodies += result.new_bodies;
        profiler.current.retired_bodies += result.split_edits.retired_bodies;
        profiler.current.body_type_flips += result.split_edits.body_type_flips;
        profiler.current.moved_colliders += result.split_edits.moved_colliders;
        profiler.current.inserted_colliders += result.split_edits.inserted_colliders;
        profiler.current.removed_colliders += result.split_edits.removed_colliders;
        profiler.current.sibling_grace_pairs += result.sibling_grace_pairs;
        profiler.current.sibling_grace_filtered_pairs += result.sibling_grace_filtered_pairs;
    }
    profiler.current.removed_nodes += removed_node_count;
    profiler.current.removed_projectiles += removed_projectiles;
    update_scene_counters(&state, &mut profiler.current);
}

fn step_rapier_world<H: PhysicsHooks>(state: &mut DemoPhysicsState, dt: f32, hooks: &H) {
    let DemoPhysicsState {
        physics_pipeline,
        integration_parameters,
        island_manager,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        ccd_solver,
        ..
    } = state;

    integration_parameters.dt = dt;
    let gravity = vector![0.0, GRAVITY, 0.0];
    physics_pipeline.step(
        &gravity,
        integration_parameters,
        island_manager,
        broad_phase,
        narrow_phase,
        bodies,
        colliders,
        impulse_joints,
        multibody_joints,
        ccd_solver,
        hooks,
        &(),
    );
}

fn cleanup_projectiles(state: &mut DemoPhysicsState, commands: &mut Commands, dt: f32) -> usize {
    let mut keep = Vec::with_capacity(state.projectiles.len());
    let mut removed = 0usize;

    for mut projectile in state.projectiles.drain(..) {
        projectile.ttl -= dt;
        let remove_for_ttl = projectile.ttl <= 0.0;
        let body = state.bodies.get(projectile.body_handle);
        let remove_for_body = body.is_none();
        let remove_for_fall = body.map(|rb| rb.translation().y < -20.0).unwrap_or(false);

        if remove_for_ttl || remove_for_body || remove_for_fall {
            if body.is_some() {
                state.bodies.remove(
                    projectile.body_handle,
                    &mut state.island_manager,
                    &mut state.colliders,
                    &mut state.impulse_joints,
                    &mut state.multibody_joints,
                    true,
                );
            }
            state
                .projectile_colliders
                .remove(&projectile.collider_handle);
            if let Some(entity) = state.projectile_entities.remove(&projectile.body_handle) {
                commands.entity(entity).despawn();
            }
            removed += 1;
            continue;
        }

        keep.push(projectile);
    }

    state.projectiles = keep;
    removed
}

fn ray_box_exit_distance(origin: Vec3, direction: Vec3, bounds: Bounds3) -> Option<f32> {
    if direction.length_squared() <= f32::EPSILON {
        return None;
    }
    let origin_components = origin.to_array();
    let dir_components = direction.to_array();
    let min_components = bounds.min.to_array();
    let max_components = bounds.max.to_array();
    let mut t_min = f32::NEG_INFINITY;
    let mut t_max = f32::INFINITY;

    for axis in 0..3 {
        let origin_component = origin_components[axis];
        let dir_component = dir_components[axis];
        let min_component = min_components[axis];
        let max_component = max_components[axis];
        if dir_component.abs() <= 1.0e-6 {
            if origin_component < min_component || origin_component > max_component {
                return None;
            }
            continue;
        }
        let inv_dir = 1.0 / dir_component;
        let mut t1 = (min_component - origin_component) * inv_dir;
        let mut t2 = (max_component - origin_component) * inv_dir;
        if t1 > t2 {
            std::mem::swap(&mut t1, &mut t2);
        }
        t_min = t_min.max(t1);
        t_max = t_max.min(t2);
        if t_min > t_max {
            return None;
        }
    }

    if t_max.is_finite() && t_max >= 0.0 {
        Some(t_max)
    } else {
        None
    }
}

fn update_projectile_progress(state: &mut DemoPhysicsState) {
    for projectile in &mut state.projectiles {
        let Some(body) = state.bodies.get(projectile.body_handle) else {
            continue;
        };
        let pos = body.translation();
        let world_pos = Vec3::new(pos.x, pos.y, pos.z);
        let progress = (world_pos - projectile.origin).dot(projectile.direction);
        projectile.max_progress = projectile.max_progress.max(progress);
        if !projectile.crossed_target_plane && progress >= projectile.target_distance {
            projectile.crossed_target_plane = true;
            state.projectile_run_stats.crossed_target_plane_count += 1;
        }
        if let Some(exit_distance) = projectile.exit_distance {
            if exit_distance > 0.0 {
                state.projectile_run_stats.max_progress_ratio = state
                    .projectile_run_stats
                    .max_progress_ratio
                    .max(projectile.max_progress / exit_distance);
            }
            if !projectile.passed_through && progress >= exit_distance {
                projectile.passed_through = true;
                state.projectile_run_stats.passed_through_count += 1;
            }
        }
    }
}

fn log_projectile_trace(profiler: &mut DebugProfiler, state: &DemoPhysicsState, frame_number: u64) {
    for (index, projectile) in state.projectiles.iter().enumerate() {
        let Some(body) = state.bodies.get(projectile.body_handle) else {
            continue;
        };
        let position = body.translation();
        let linvel = body.linvel();
        let mut contact_pairs = 0usize;
        let mut active_contact_pairs = 0usize;
        for pair in state
            .narrow_phase
            .contact_pairs_with(projectile.collider_handle)
        {
            contact_pairs += 1;
            if pair.has_any_active_contact {
                active_contact_pairs += 1;
            }
        }
        let progress_ratio = projectile
            .exit_distance
            .filter(|distance| *distance > 0.0)
            .map(|distance| projectile.max_progress / distance)
            .unwrap_or(0.0);
        profiler.pending_event_lines.push(format!(
            "[projectile-frame] frame={} projectile_index={} pos=({:.3},{:.3},{:.3}) linvel=({:.3},{:.3},{:.3}) sleeping={} progress={:.3} target_distance={:.3} exit_distance={:.3} progress_ratio={:.3} crossed_target_plane={} passed_through={} contact_pairs={} active_contact_pairs={}",
            frame_number,
            index,
            position.x,
            position.y,
            position.z,
            linvel.x,
            linvel.y,
            linvel.z,
            flag_bit(body.is_sleeping()),
            projectile.max_progress,
            projectile.target_distance,
            projectile.exit_distance.unwrap_or(0.0),
            progress_ratio,
            flag_bit(projectile.crossed_target_plane),
            flag_bit(projectile.passed_through),
            contact_pairs,
            active_contact_pairs,
        ));
    }
}

fn update_scene_counters(state: &DemoPhysicsState, frame: &mut FrameBreakdown) {
    let mut ccd_bodies = 0usize;
    for (_, body) in state.bodies.iter() {
        if body.is_dynamic() && body.is_ccd_enabled() {
            ccd_bodies += 1;
        }
    }

    let mut contact_pairs = 0usize;
    let mut active_contact_pairs = 0usize;
    let mut contact_manifolds = 0usize;
    for pair in state.narrow_phase.contact_pairs() {
        contact_pairs += 1;
        if pair.has_any_active_contact {
            active_contact_pairs += 1;
        }
        contact_manifolds += pair.manifolds.len();
    }

    frame.world_bodies = state.bodies.len();
    frame.destructible_bodies = demo_destructible_body_count(state);
    frame.support_bodies = demo_support_body_count(state);
    frame.dynamic_bodies = demo_dynamic_body_count(state);
    frame.awake_dynamic_bodies = demo_awake_dynamic_body_count(state);
    frame.sleeping_dynamic_bodies = demo_sleeping_dynamic_body_count(state);
    frame.world_colliders = state.colliders.len();
    frame.destructible_colliders = demo_destructible_collider_count(state);
    frame.projectile_count = state.projectiles.len();
    frame.ccd_bodies = ccd_bodies;
    frame.contact_pairs = contact_pairs;
    frame.active_contact_pairs = active_contact_pairs;
    frame.contact_manifolds = contact_manifolds;
    frame.pending_split_events = if state.rapier_only.is_some() {
        0
    } else {
        state.destructible.pending_split_event_count()
    };
    frame.pending_new_bodies = if state.rapier_only.is_some() {
        0
    } else {
        state.destructible.pending_new_body_count(&state.bodies)
    };
    frame.pending_collider_migrations = if state.rapier_only.is_some() {
        0
    } else {
        state
            .destructible
            .pending_collider_migration_count(&state.bodies)
    };
}

fn sync_visuals_system(
    state: NonSend<DemoPhysicsState>,
    mut profiler: ResMut<DebugProfiler>,
    mut chunk_q: Query<
        (
            &ChunkVisual,
            &mut Transform,
            &mut ChunkTint,
            Option<&ChunkMaterial>,
        ),
        Without<ProjectileVisual>,
    >,
    mut projectile_q: Query<&mut Transform, (With<ProjectileVisual>, Without<ChunkVisual>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let started_at = Instant::now();
    for (&node_index, &entity) in &state.node_to_entity {
        let Ok((chunk, mut transform, mut tint, material)) = chunk_q.get_mut(entity) else {
            continue;
        };
        let Some((translation, rotation)) = node_world_transform(&state, node_index) else {
            continue;
        };
        transform.translation = translation;
        transform.rotation = rotation;

        let color = demo_node_body(&state, chunk.node_index)
            .map(|handle| body_color(handle, chunk.is_support))
            .unwrap_or_else(|| default_chunk_color(chunk.is_support));
        if tint.color != color {
            tint.color = color;
            if let Some(material) = material {
                if let Some(standard_material) = materials.get_mut(&material.handle) {
                    standard_material.base_color = color;
                }
            }
        }
    }

    for projectile in &state.projectiles {
        let Some(body) = state.bodies.get(projectile.body_handle) else {
            continue;
        };
        let Some(&entity) = state.projectile_entities.get(&projectile.body_handle) else {
            continue;
        };
        if let Ok(mut transform) = projectile_q.get_mut(entity) {
            let pos = body.translation();
            let rot = body.rotation();
            transform.translation = Vec3::new(pos.x, pos.y, pos.z);
            transform.rotation = Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w);
        }
    }
    profiler.current.sync_visuals_ms += started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
}

fn node_world_transform(state: &DemoPhysicsState, node_index: u32) -> Option<(Vec3, Quat)> {
    let body_handle = demo_node_body(state, node_index)?;
    let body = state.bodies.get(body_handle)?;
    let local = demo_node_local_offset(state, node_index)?;
    let world = body
        .position()
        .transform_point(&point![local.x, local.y, local.z]);
    let rot = body.rotation();
    Some((
        Vec3::new(world.x, world.y, world.z),
        Quat::from_xyzw(rot.i, rot.j, rot.k, rot.w),
    ))
}

fn gizmo_render_system(
    mut gizmos: Gizmos,
    toggles: Res<DemoRuntimeToggles>,
    mut profiler: ResMut<DebugProfiler>,
    chunk_q: Query<(&ChunkVisual, &ChunkTint, &Transform)>,
    projectile_q: Query<(&ProjectileVisual, &Transform)>,
) {
    let started_at = Instant::now();
    if !toggles.gizmos_enabled {
        return;
    }
    draw_floor_gizmo(&mut gizmos);

    for (chunk, tint, transform) in &chunk_q {
        match &chunk.gizmo_shape {
            ChunkGizmoShape::Box { size } => draw_box_gizmo(
                &mut gizmos,
                transform.translation,
                transform.rotation,
                *size,
                tint.color,
            ),
            ChunkGizmoShape::TriEdges { edges } => draw_mesh_gizmo(
                &mut gizmos,
                transform.translation,
                transform.rotation,
                edges,
                tint.color,
            ),
        }
    }

    for (projectile, transform) in &projectile_q {
        let isometry = Isometry3d::from_translation(transform.translation);
        gizmos
            .sphere(isometry, projectile.radius, Color::srgb(1.0, 0.95, 0.32))
            .resolution(18);
    }
    profiler.current.gizmo_ms += started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
}

fn default_chunk_color(is_support: bool) -> Color {
    if is_support {
        Color::srgb(0.38, 0.48, 0.62)
    } else {
        Color::srgb(0.76, 0.58, 0.38)
    }
}

fn body_color(handle: RigidBodyHandle, is_support: bool) -> Color {
    let (index, generation) = handle.into_raw_parts();
    let seed = index
        .wrapping_mul(1_103_515_245)
        .wrapping_add(generation.wrapping_mul(12_345));
    let hue = (seed % 360) as f32;
    let saturation = if is_support { 0.35 } else { 0.78 };
    let lightness = if is_support { 0.58 } else { 0.56 };
    Color::hsl(hue, saturation, lightness)
}

fn draw_floor_gizmo(gizmos: &mut Gizmos) {
    let y = 0.0;
    let half = 12.0;
    let edge_color = Color::srgb(0.50, 0.56, 0.66);
    let grid_color = Color::srgb(0.24, 0.28, 0.34);

    let corners = [
        Vec3::new(-half, y, -half),
        Vec3::new(half, y, -half),
        Vec3::new(half, y, half),
        Vec3::new(-half, y, half),
    ];

    for i in 0..corners.len() {
        gizmos.line(corners[i], corners[(i + 1) % corners.len()], edge_color);
    }

    for step in -12..=12 {
        let offset = step as f32;
        gizmos.line(
            Vec3::new(offset, y, -half),
            Vec3::new(offset, y, half),
            grid_color,
        );
        gizmos.line(
            Vec3::new(-half, y, offset),
            Vec3::new(half, y, offset),
            grid_color,
        );
    }
}

fn draw_box_gizmo(gizmos: &mut Gizmos, center: Vec3, rotation: Quat, size: Vec3, color: Color) {
    let half = size * 0.5;
    let corners = [
        Vec3::new(-half.x, -half.y, -half.z),
        Vec3::new(half.x, -half.y, -half.z),
        Vec3::new(half.x, half.y, -half.z),
        Vec3::new(-half.x, half.y, -half.z),
        Vec3::new(-half.x, -half.y, half.z),
        Vec3::new(half.x, -half.y, half.z),
        Vec3::new(half.x, half.y, half.z),
        Vec3::new(-half.x, half.y, half.z),
    ]
    .map(|corner| center + rotation * corner);

    const EDGES: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];

    for (start, end) in EDGES {
        gizmos.line(corners[start], corners[end], color);
    }
}

fn draw_mesh_gizmo(
    gizmos: &mut Gizmos,
    center: Vec3,
    rotation: Quat,
    edges: &[(Vec3, Vec3)],
    color: Color,
) {
    for &(start, end) in edges {
        gizmos.line(center + rotation * start, center + rotation * end, color);
    }
}

fn build_mesh_gizmo_edges(mesh: &SceneMeshAsset) -> Vec<(Vec3, Vec3)> {
    let mut unique_edges = HashSet::new();
    for triangle in mesh.indices.chunks_exact(3) {
        let triangle_edges = [
            (triangle[0], triangle[1]),
            (triangle[1], triangle[2]),
            (triangle[2], triangle[0]),
        ];
        for (a, b) in triangle_edges {
            unique_edges.insert(if a <= b { (a, b) } else { (b, a) });
        }
    }

    unique_edges
        .into_iter()
        .filter_map(|(a, b)| {
            let start = mesh.positions.get(a as usize)?;
            let end = mesh.positions.get(b as usize)?;
            Some((
                Vec3::new(start[0], start[1], start[2]),
                Vec3::new(end[0], end[1], end[2]),
            ))
        })
        .collect()
}

fn headless_exit_system(
    mut budget: ResMut<HeadlessRunBudget>,
    scenario: Res<DemoScenario>,
    profiler: Res<DebugProfiler>,
    shot_plan: Option<Res<HeadlessShotPlan>>,
    mut perf_log: ResMut<PerfLogWriter>,
    state: Option<NonSend<DemoPhysicsState>>,
    mut app_exit_writer: MessageWriter<AppExit>,
) {
    if budget.frames_remaining > 0 {
        budget.frames_remaining -= 1;
    }

    if budget.frames_remaining == 0 {
        let actors = state.as_ref().map_or(0, |state| demo_actor_count(state));
        let projectile_stats = state
            .as_ref()
            .map_or(ProjectileRunStats::default(), |state| {
                state.projectile_run_stats
            });
        perf_log.write_line(&profiler.headless_summary_line(
            scenario.kind,
            shot_plan.as_deref(),
            projectile_stats,
        ));
        perf_log.flush();
        eprintln!("Headless simulation complete. Actor count: {actors}.");
        app_exit_writer.write(AppExit::Success);
    }
}

fn hud_system(
    info: Res<DemoInfo>,
    projectile_settings: Res<ProjectileFireSettings>,
    toggles: Res<DemoRuntimeToggles>,
    mut profiler: ResMut<DebugProfiler>,
    state: Option<NonSend<DemoPhysicsState>>,
    mut text_q: Query<&mut Text, With<HudText>>,
) {
    let started_at = Instant::now();
    let Some(state) = state else { return };
    if let Ok(mut text) = text_q.single_mut() {
        let actors = demo_actor_count(&state);
        let bodies = demo_destructible_body_count(&state);
        let world_bodies = state.bodies.len();
        let fps = profiler.fps();
        *text = Text::new(format!(
            "{}\n{}\nLeft click: Shoot  |  Right drag or Ctrl+Left drag: Orbit  |  Scroll: Zoom  |  R: Reset  |  [-]/[=]: Projectile Mass\n{}\nProjectile Mass: {}\nActors: {actors}  |  Destructible Bodies: {bodies}  |  World Bodies: {world_bodies}\nScene Last Frame: support_bodies={} dynamic_bodies={} awake={} sleeping={} world_colliders={} destructible_colliders={} projectiles={} ccd_bodies={}\nContacts Last Frame: pairs={} active_pairs={} manifolds={} pending_splits={} pending_new_bodies={} pending_migrations={}\nFPS: {fps:.1}  |  Frame CPU: {:.2} ms avg / {:.2} ms last\nPhysics: {:.2} ms avg / {:.2} ms last  |  Rapier passes: {:.2} avg / {} last\nRapier: {:.2} ms  |  Collision Events: {:.2} ms  |  Contact Forces: {:.2} ms  |  Solver: {:.2} ms\nSplit Edit: sanitize={:.2} estimate={:.2} plan={:.2} apply={:.2} move={:.2} create={:.2} insert={:.2} retire={:.2}\nPeaks: frame={:.2} physics={:.2} rapier={:.2} solver={:.2} split_plan={:.2} split_apply={:.2} split_move={:.2}\nFracture Peaks: frame={:.2} physics={:.2} rapier={:.2} solver={:.2} split_plan={:.2} split_apply={:.2}\nSplit Ops Last Frame: reused={} recycled={} new_bodies={} retired={} flips={} moved_colliders={} inserted_colliders={} removed_colliders={}\nResim Restore: {:.2} ms  |  Snapshot: {:.2} ms  |  Optimization: {:.2} ms  |  Projectile Cleanup: {:.2} ms\nRender Prep CPU: {:.2} ms avg / {:.2} ms last  |  Sync: {:.2} ms  |  Gizmos: {:.2} ms  |  HUD: {:.2} ms  |  Other CPU: {:.2} ms\nEvents Last Frame: collisions={} contacts={} fractures={} splits={} removed_nodes={} removed_projectiles={}",
            info.title,
            info.subtitle,
            toggles.summary(),
            format_projectile_mass(projectile_settings.mass),
            profiler.last_support_bodies,
            profiler.last_dynamic_bodies,
            profiler.last_awake_dynamic_bodies,
            profiler.last_sleeping_dynamic_bodies,
            profiler.last_world_colliders,
            profiler.last_destructible_colliders,
            profiler.last_projectile_count,
            profiler.last_ccd_bodies,
            profiler.last_contact_pairs,
            profiler.last_active_contact_pairs,
            profiler.last_contact_manifolds,
            profiler.last_pending_split_events,
            profiler.last_pending_new_bodies,
            profiler.last_pending_collider_migrations,
            profiler.frame_cpu.avg_ms,
            profiler.frame_cpu.last_ms,
            profiler.physics.avg_ms,
            profiler.physics.last_ms,
            profiler.avg_rapier_passes,
            profiler.last_rapier_passes,
            profiler.rapier.last_ms,
            profiler.collision_events.last_ms,
            profiler.contact_forces.last_ms,
            profiler.solver.last_ms,
            profiler.split_sanitize.last_ms,
            profiler.split_estimate.last_ms,
            profiler.split_plan.last_ms,
            profiler.split_apply.last_ms,
            profiler.split_collider_move.last_ms,
            profiler.split_body_create.last_ms,
            profiler.split_collider_insert.last_ms,
            profiler.split_body_retire.last_ms,
            profiler.peak_frame_ms,
            profiler.peak_physics_ms,
            profiler.peak_rapier_ms,
            profiler.peak_solver_ms,
            profiler.peak_split_plan_ms,
            profiler.peak_split_apply_ms,
            profiler.peak_split_collider_move_ms,
            profiler.peak_fracture_frame_ms,
            profiler.peak_fracture_physics_ms,
            profiler.peak_fracture_rapier_ms,
            profiler.peak_fracture_solver_ms,
            profiler.peak_fracture_split_plan_ms,
            profiler.peak_fracture_split_apply_ms,
            profiler.last_reused_bodies,
            profiler.last_recycled_bodies,
            profiler.last_new_bodies,
            profiler.last_retired_bodies,
            profiler.last_body_type_flips,
            profiler.last_moved_colliders,
            profiler.last_inserted_colliders,
            profiler.last_removed_colliders,
            profiler.resim_restore.last_ms,
            profiler.resim_snapshot.last_ms,
            profiler.optimization.last_ms,
            profiler.projectile_cleanup.last_ms,
            profiler.render_prep.avg_ms,
            profiler.render_prep.last_ms,
            profiler.sync_visuals.last_ms,
            profiler.gizmo.last_ms,
            profiler.hud.last_ms,
            profiler.other_cpu.last_ms,
            profiler.last_collision_events,
            profiler.last_contact_events,
            profiler.last_fractures,
            profiler.last_split_events,
            profiler.last_removed_nodes,
            profiler.last_removed_projectiles,
        ));
    }
    profiler.current.hud_ms += started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
}

fn format_projectile_mass(mass: f32) -> String {
    if mass >= 1000.0 {
        format!("{mass:.0}")
    } else if mass >= 100.0 {
        format!("{mass:.1}")
    } else {
        format!("{mass:.2}")
    }
}

fn begin_frame_profile_system(mut profiler: ResMut<DebugProfiler>) {
    profiler.begin_frame();
}

fn finish_frame_profile_system(
    mut profiler: ResMut<DebugProfiler>,
    mut perf_log: ResMut<PerfLogWriter>,
) {
    let line = profiler.finish_frame();
    for event_line in profiler.drain_event_lines() {
        perf_log.write_line(&event_line);
    }
    if let Some(line) = line {
        perf_log.write_line(&line);
    }
}
