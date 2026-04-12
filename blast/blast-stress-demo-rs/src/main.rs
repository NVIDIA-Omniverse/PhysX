use std::{
    any::Any,
    collections::HashMap,
    env,
    fs::OpenOptions,
    io::{BufWriter, Write},
    panic,
    path::PathBuf,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc,
    },
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
    DebrisCleanupOptions, DebrisCollisionMode, DestructibleSet, FracturePolicy, OptimizationMode,
    ResimulationOptions, SleepThresholdOptions, SmallBodyDampingOptions,
};
use blast_stress_solver::scenarios::{
    build_bridge_scenario, build_tower_scenario, build_wall_scenario, BridgeOptions, TowerOptions,
    WallOptions,
};
use blast_stress_solver::{ScenarioDesc, SolverSettings, Vec3 as SolverVec3};
use rapier3d::prelude::*;

const GRAVITY: f32 = -9.81;
const CONTACT_FORCE_SCALE: f32 = 30.0;
const SPLASH_RADIUS: f32 = 2.0;
const DEFAULT_HEADLESS_FRAMES: u32 = 180;
const DEFAULT_PROJECTILE_TTL: f32 = 6.0;

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
}

impl DemoScenarioKind {
    fn slug(self) -> &'static str {
        match self {
            Self::Wall => "wall",
            Self::Tower => "tower",
            Self::Bridge => "bridge",
        }
    }
}

#[derive(Clone, Debug)]
struct DemoConfig {
    title: &'static str,
    scenario: ScenarioDesc,
    projectile_radius: f32,
    projectile_mass: f32,
    projectile_speed: f32,
    projectile_ttl: f32,
    material_scale: f32,
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
    contact_force_injection_enabled: bool,
    gizmos_enabled: bool,
    projectile_ccd_enabled: bool,
    body_ccd_enabled: bool,
    show_meshes: bool,
    config_summary: String,
}

impl DemoRuntimeToggles {
    fn from_env(scenario: DemoScenarioKind, config: &mut DemoConfig) -> Self {
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

        let config_summary = format!(
            "scenario={} meshes={} gizmos={} resim={} max_resim={} contact_injection={} projectile_ccd={} body_ccd={} policy[max_fractures={} max_new_bodies={} max_collider_migrations={} max_dynamic_bodies={} min_child_nodes={} idle_skip={} apply_excess_forces={}] sleep[enabled={} mode={} linear={:.3} angular={:.3}] small_body_damping[enabled={} mode={} colliders={} linear={:.2} angular={:.2}] debris[collision_mode={} cleanup_enabled={} cleanup_mode={} ttl={:.2} max_colliders={}]",
            scenario.slug(),
            flag_bit(show_meshes),
            flag_bit(env_flag("BLAST_STRESS_DEMO_GIZMOS").unwrap_or(true)),
            flag_bit(resimulation_enabled),
            max_resimulation_passes,
            flag_bit(env_flag("BLAST_STRESS_DEMO_CONTACT_FORCE_INJECTION").unwrap_or(true)),
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
        );

        let contact_force_injection_enabled =
            env_flag("BLAST_STRESS_DEMO_CONTACT_FORCE_INJECTION").unwrap_or(true);
        let gizmos_enabled = env_flag("BLAST_STRESS_DEMO_GIZMOS").unwrap_or(true);
        let projectile_ccd_enabled = env_flag("BLAST_STRESS_DEMO_PROJECTILE_CCD").unwrap_or(true);
        let body_ccd_enabled = env_flag("BLAST_STRESS_DEMO_BODY_CCD").unwrap_or(false);

        Self {
            contact_force_injection_enabled,
            gizmos_enabled,
            projectile_ccd_enabled,
            body_ccd_enabled,
            show_meshes,
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

#[derive(Resource)]
struct ProjectileFireSettings {
    mass: f32,
}

impl ProjectileFireSettings {
    fn new(initial_mass: f32) -> Self {
        Self {
            mass: initial_mass.max(1.0),
        }
    }

    fn increase_mass(&mut self) {
        self.mass = (self.mass * 2.0).min(1_000_000.0);
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
    size: Vec3,
    is_support: bool,
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
    new_bodies: usize,
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
    new_bodies: Vec<f32>,
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
    pending_split_events: Vec<f32>,
    pending_new_bodies: Vec<f32>,
    pending_collider_migrations: Vec<f32>,
}

#[derive(Resource, Default)]
struct DebugProfiler {
    frame_started_at: Option<Instant>,
    log_window_started_at: Option<Instant>,
    current: FrameBreakdown,
    median_window: MedianWindow,
    frame_cpu: SmoothedMetric,
    physics: SmoothedMetric,
    rapier: SmoothedMetric,
    collision_events: SmoothedMetric,
    contact_forces: SmoothedMetric,
    solver: SmoothedMetric,
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
    last_new_bodies: usize,
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
    config_summary: String,
}

impl DebugProfiler {
    fn begin_frame(&mut self) {
        self.frame_started_at = Some(Instant::now());
        self.current = FrameBreakdown::default();
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

        self.last_rapier_passes = self.current.rapier_passes;
        self.last_collision_events = self.current.collision_events;
        self.last_contact_events = self.current.contact_events;
        self.last_fractures = self.current.fractures;
        self.last_split_events = self.current.split_events;
        self.last_new_bodies = self.current.new_bodies;
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
            .new_bodies
            .push(self.current.new_bodies as f32);
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
        let new_bodies_med = median_ms(&mut self.median_window.new_bodies);
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
            "[perf] samples={} fps_med={:.1} frame_ms_med={:.3} physics_ms_med={:.3} rapier_ms_med={:.3} collision_events_ms_med={:.3} contact_forces_ms_med={:.3} solver_ms_med={:.3} resim_restore_ms_med={:.3} snapshot_ms_med={:.3} optimization_ms_med={:.3} projectile_cleanup_ms_med={:.3} render_prep_ms_med={:.3} sync_ms_med={:.3} gizmo_ms_med={:.3} hud_ms_med={:.3} other_cpu_ms_med={:.3} rapier_passes_med={:.1} collisions_med={:.1} contacts_med={:.1} fractures_med={:.1} splits_med={:.1} new_bodies_med={:.1} removed_nodes_med={:.1} removed_projectiles_med={:.1} world_bodies_med={:.1} destructible_bodies_med={:.1} support_bodies_med={:.1} dynamic_bodies_med={:.1} awake_dynamic_bodies_med={:.1} sleeping_dynamic_bodies_med={:.1} world_colliders_med={:.1} destructible_colliders_med={:.1} projectiles_med={:.1} ccd_bodies_med={:.1} contact_pairs_med={:.1} active_contact_pairs_med={:.1} contact_manifolds_med={:.1} pending_splits_med={:.1} pending_new_bodies_med={:.1} pending_migrations_med={:.1} {}",
            sample_count,
            fps_med,
            frame_ms_med,
            physics_ms_med,
            rapier_ms_med,
            collision_events_ms_med,
            contact_forces_ms_med,
            solver_ms_med,
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
            new_bodies_med,
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
            pending_split_events_med,
            pending_new_bodies_med,
            pending_collider_migrations_med,
            self.config_summary,
        );

        self.median_window = MedianWindow::default();
        Some(line)
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
    destructible: DestructibleSet,
    collision_send: Sender<CollisionEvent>,
    collision_recv: Receiver<CollisionEvent>,
    contact_send: Sender<ContactForceEvent>,
    contact_recv: Receiver<ContactForceEvent>,
    node_to_entity: HashMap<u32, Entity>,
    projectile_entities: HashMap<RigidBodyHandle, Entity>,
    projectile_colliders: HashMap<ColliderHandle, RigidBodyHandle>,
    projectiles: Vec<ProjectileState>,
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
    let mut profiler = DebugProfiler::default();
    profiler.config_summary = toggles.summary();
    perf_log.write_line(&format!("# {}", profiler.config_summary));

    let exit = {
        let mut app = App::new();
        app.insert_resource(RunMode { headless });
        app.insert_resource(info.clone());
        app.insert_resource(CameraOrbit::from_info(&info));
        app.insert_resource(ProjectileFireSettings::new(config.projectile_mass));
        app.insert_resource(toggles.clone());
        app.insert_resource(profiler);
        app.insert_resource(perf_log);
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
        _ => DemoScenarioKind::Wall,
    }
}

fn build_demo_config(kind: DemoScenarioKind) -> DemoConfig {
    match kind {
        DemoScenarioKind::Wall => DemoConfig {
            title: "Wall Demolition",
            scenario: build_wall_scenario(&WallOptions::default()),
            projectile_radius: 0.35,
            projectile_mass: 1_000.0,
            projectile_speed: 20.0,
            projectile_ttl: DEFAULT_PROJECTILE_TTL,
            material_scale: 1.0e10,
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
        },
        DemoScenarioKind::Tower => DemoConfig {
            title: "Tower Collapse",
            scenario: build_tower_scenario(&TowerOptions::default()),
            projectile_radius: 0.35,
            projectile_mass: 1_000.0,
            projectile_speed: 22.0,
            projectile_ttl: DEFAULT_PROJECTILE_TTL,
            material_scale: 1.0e10,
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
        },
        DemoScenarioKind::Bridge => DemoConfig {
            title: "Bridge Stress",
            scenario: build_bridge_scenario(&BridgeOptions::default()),
            projectile_radius: 0.4,
            projectile_mass: 1_000.0,
            projectile_speed: 20.0,
            projectile_ttl: DEFAULT_PROJECTILE_TTL,
            material_scale: 1.0e10,
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
            debris_collision_mode: DebrisCollisionMode::NoDebrisPairs,
        },
    }
}

fn build_demo_physics(config: DemoConfig, toggles: &DemoRuntimeToggles) -> DemoPhysicsState {
    let settings = scaled_solver_settings(config.material_scale);
    let gravity = SolverVec3::new(0.0, GRAVITY, 0.0);
    let mut destructible =
        DestructibleSet::from_scenario(&config.scenario, settings, gravity, config.policy)
            .expect("failed to create destructible set");
    destructible.set_resimulation_options(config.resimulation);
    destructible.set_sleep_thresholds(config.sleep_thresholds);
    destructible.set_small_body_damping(config.small_body_damping);
    destructible.set_debris_cleanup(config.debris_cleanup);
    destructible.set_debris_collision_mode(config.debris_collision_mode);
    destructible.set_dynamic_body_ccd_enabled(toggles.body_ccd_enabled);

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let _handles = destructible.initialize(&mut bodies, &mut colliders);

    let ground_body = bodies.insert(RigidBodyBuilder::fixed().translation(vector![0.0, 0.0, 0.0]));
    let ground = ColliderBuilder::cuboid(100.0, 0.025, 100.0)
        .translation(vector![0.0, -0.025, 0.0])
        .active_events(ActiveEvents::COLLISION_EVENTS)
        .friction(0.9)
        .restitution(0.0);
    colliders.insert_with_parent(ground, ground_body, &mut bodies);
    destructible.set_ground_body_handle(Some(ground_body));
    destructible.refresh_collision_groups(&bodies, &mut colliders);

    let (collision_send, collision_recv) = channel();
    let (contact_send, contact_recv) = channel();

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
        collision_send,
        collision_recv,
        contact_send,
        contact_recv,
        node_to_entity: HashMap::new(),
        projectile_entities: HashMap::new(),
        projectile_colliders: HashMap::new(),
        projectiles: Vec::new(),
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
        let color = state
            .destructible
            .node_body(node_index as u32)
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
                size: Vec3::new(size.x, size.y, size.z),
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
                entity.insert((
                    Mesh3d(meshes.add(BevyCuboid::new(
                        size.x * 0.98,
                        size.y * 0.98,
                        size.z * 0.98,
                    ))),
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

    let projectile_speed = state.config.projectile_speed;
    let projectile_radius = state.config.projectile_radius;
    let projectile_mass = projectile_settings.mass;
    let projectile_ttl = state.config.projectile_ttl;
    let DemoPhysicsState {
        bodies,
        colliders,
        projectile_entities,
        projectile_colliders,
        projectiles,
        ..
    } = &mut *state;

    let body_handle = bodies.insert(
        RigidBodyBuilder::dynamic()
            .translation(vector![origin.x, origin.y, origin.z])
            .linvel(vector![
                direction.x * projectile_speed,
                direction.y * projectile_speed,
                direction.z * projectile_speed
            ])
            .ccd_enabled(toggles.projectile_ccd_enabled),
    );

    let collider_handle = colliders.insert_with_parent(
        ColliderBuilder::ball(projectile_radius)
            .mass(projectile_mass)
            .friction(0.25)
            .restitution(0.0)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS | ActiveEvents::COLLISION_EVENTS)
            .contact_force_event_threshold(0.0),
        body_handle,
        bodies,
    );

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

    projectile_entities.insert(body_handle, entity);
    projectile_colliders.insert(collider_handle, body_handle);
    projectiles.push(ProjectileState {
        body_handle,
        collider_handle,
        ttl: projectile_ttl,
    });
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
    let resimulation = state.destructible.resimulation_options();
    let mut remaining_resim_passes = if resimulation.enabled {
        resimulation.max_passes
    } else {
        0
    };
    let mut snapshot = if state.destructible.needs_resimulation_snapshot() {
        let snapshot_started_at = Instant::now();
        let captured = state
            .destructible
            .capture_resimulation_snapshot(&state.bodies);
        profiler.current.resim_snapshot_ms +=
            snapshot_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
        Some(captured)
    } else {
        None
    };
    let mut rapier_passes = 0u32;
    let mut collision_events = 0usize;
    let mut contact_events = 0usize;
    let mut total_fractures = 0usize;
    let mut total_split_events = 0usize;
    let mut total_new_bodies = 0usize;

    loop {
        let rapier_started_at = Instant::now();
        step_rapier_world(&mut state, dt);
        profiler.current.rapier_ms += rapier_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
        rapier_passes += 1;

        let collision_started_at = Instant::now();
        collision_events += drain_collision_events(&mut state, now_secs);
        profiler.current.collision_events_ms +=
            collision_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

        let contact_started_at = Instant::now();
        contact_events += drain_contact_forces(&mut state, toggles.contact_force_injection_enabled);
        profiler.current.contact_forces_ms +=
            contact_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

        let solver_started_at = Instant::now();
        let fracture_result = {
            let DemoPhysicsState {
                destructible,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
                ..
            } = &mut *state;
            destructible.step_with_time(
                now_secs,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
            )
        };
        profiler.current.solver_ms += solver_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
        total_fractures += fracture_result.fractures;
        total_split_events += fracture_result.split_events;
        total_new_bodies += fracture_result.new_bodies;

        let fractured = fracture_result.split_events > 0 || fracture_result.new_bodies > 0;
        if !fractured || remaining_resim_passes == 0 {
            break;
        }

        let Some(current_snapshot) = snapshot.as_ref() else {
            break;
        };

        let restore_started_at = Instant::now();
        current_snapshot.restore(&mut state.bodies);
        profiler.current.resim_restore_ms +=
            restore_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

        remaining_resim_passes = remaining_resim_passes.saturating_sub(1);
        if remaining_resim_passes == 0 {
            snapshot = None;
            continue;
        }

        let snapshot_started_at = Instant::now();
        snapshot = Some(
            state
                .destructible
                .capture_resimulation_snapshot(&state.bodies),
        );
        profiler.current.resim_snapshot_ms +=
            snapshot_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
    }

    let optimization_started_at = Instant::now();
    let removed_nodes = {
        let DemoPhysicsState {
            destructible,
            bodies,
            colliders,
            island_manager,
            impulse_joints,
            multibody_joints,
            ..
        } = &mut *state;
        destructible
            .process_optimizations(
                now_secs,
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
            )
            .removed_nodes
    };
    profiler.current.optimization_ms +=
        optimization_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

    let removed_node_count = removed_nodes.len();
    for node_index in removed_nodes {
        if let Some(entity) = state.node_to_entity.remove(&node_index) {
            commands.entity(entity).despawn();
        }
    }

    let cleanup_started_at = Instant::now();
    let removed_projectiles = cleanup_projectiles(&mut state, &mut commands, dt);
    profiler.current.projectile_cleanup_ms +=
        cleanup_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;

    profiler.current.physics_ms += physics_started_at.elapsed().as_secs_f64() as f32 * 1_000.0;
    profiler.current.rapier_passes += rapier_passes;
    profiler.current.collision_events += collision_events;
    profiler.current.contact_events += contact_events;
    profiler.current.fractures += total_fractures;
    profiler.current.split_events += total_split_events;
    profiler.current.new_bodies += total_new_bodies;
    profiler.current.removed_nodes += removed_node_count;
    profiler.current.removed_projectiles += removed_projectiles;
    update_scene_counters(&state, &mut profiler.current);
}

fn step_rapier_world(state: &mut DemoPhysicsState, dt: f32) {
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
        collision_send,
        contact_send,
        ..
    } = state;

    integration_parameters.dt = dt;

    let gravity = vector![0.0, GRAVITY, 0.0];
    let event_handler = ChannelEventCollector::new(collision_send.clone(), contact_send.clone());

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
        &(),
        &event_handler,
    );
}

fn drain_collision_events(state: &mut DemoPhysicsState, now_secs: f32) -> usize {
    let mut processed = 0usize;
    while let Ok(event) = state.collision_recv.try_recv() {
        let CollisionEvent::Started(collider1, collider2, flags) = event else {
            continue;
        };
        if flags.contains(CollisionEventFlags::SENSOR) {
            continue;
        }
        register_support_contact(state, collider1, collider2, now_secs);
        register_support_contact(state, collider2, collider1, now_secs);
        processed += 1;
    }
    processed
}

fn register_support_contact(
    state: &mut DemoPhysicsState,
    tracked_collider: ColliderHandle,
    other_collider: ColliderHandle,
    now_secs: f32,
) {
    let Some(node_index) = state.destructible.collider_node(tracked_collider) else {
        return;
    };
    if state.destructible.is_support(node_index) {
        return;
    }

    let Some(body_handle) = state.destructible.node_body(node_index) else {
        return;
    };

    let support_contact = {
        let Some(other) = state.colliders.get(other_collider) else {
            return;
        };
        let Some(other_parent) = other.parent() else {
            return;
        };
        if other_parent == body_handle {
            return;
        }

        state
            .bodies
            .get(other_parent)
            .map(|body| body.is_fixed())
            .unwrap_or(false)
            || state
                .destructible
                .collider_node(other_collider)
                .map(|other_node| state.destructible.is_support(other_node))
                .unwrap_or(false)
            || state.destructible.body_has_support(other_parent)
    };

    if support_contact {
        state.destructible.mark_body_support_contact(
            body_handle,
            now_secs,
            &mut state.bodies,
            &mut state.colliders,
        );
    }
}

fn drain_contact_forces(state: &mut DemoPhysicsState, inject_forces: bool) -> usize {
    let mut processed = 0usize;
    while let Ok(event) = state.contact_recv.try_recv() {
        let (projectile_body, node_index) =
            if let Some(&body) = state.projectile_colliders.get(&event.collider1) {
                if let Some(node) = state.destructible.collider_node(event.collider2) {
                    (body, node)
                } else {
                    continue;
                }
            } else if let Some(&body) = state.projectile_colliders.get(&event.collider2) {
                if let Some(node) = state.destructible.collider_node(event.collider1) {
                    (body, node)
                } else {
                    continue;
                }
            } else {
                continue;
            };

        let Some(projectile) = state.bodies.get(projectile_body) else {
            continue;
        };
        let projectile_velocity = projectile.linvel();
        let velocity_vec = Vec3::new(
            projectile_velocity.x,
            projectile_velocity.y,
            projectile_velocity.z,
        );
        let direction = if velocity_vec.length_squared() > 1.0e-6 {
            velocity_vec.normalize()
        } else {
            Vec3::new(
                event.total_force.x,
                event.total_force.y,
                event.total_force.z,
            )
            .normalize_or_zero()
        };
        if direction == Vec3::ZERO {
            continue;
        }

        let force_world = direction * event.total_force_magnitude;
        let Some(body_handle) = state.destructible.node_body(node_index) else {
            continue;
        };
        let Some(body) = state.bodies.get(body_handle) else {
            continue;
        };
        if !inject_forces {
            processed += 1;
            continue;
        }
        let rotation = body.position().rotation;
        let local_force = rotation.inverse() * vector![force_world.x, force_world.y, force_world.z];
        let Some(hit_local) = state.destructible.node_local_offset(node_index) else {
            continue;
        };
        let mut impacted_nodes =
            Vec::with_capacity(state.destructible.body_node_count(body_handle));
        for &other_node in state.destructible.body_nodes_slice(body_handle) {
            let Some(other_local) = state.destructible.node_local_offset(other_node) else {
                continue;
            };
            let dx = other_local.x - hit_local.x;
            let dy = other_local.y - hit_local.y;
            let dz = other_local.z - hit_local.z;
            let dist = (dx * dx + dy * dy + dz * dz).sqrt();
            if dist > SPLASH_RADIUS {
                continue;
            }

            let falloff = if other_node == node_index {
                1.0
            } else {
                let t = (1.0 - dist / SPLASH_RADIUS).max(0.0);
                t * t
            };
            if falloff <= 0.0 {
                continue;
            }

            impacted_nodes.push((other_node, other_local, falloff));
        }

        for (other_node, other_local, falloff) in impacted_nodes {
            state.destructible.add_force(
                other_node,
                other_local,
                SolverVec3::new(
                    local_force.x * CONTACT_FORCE_SCALE * falloff,
                    local_force.y * CONTACT_FORCE_SCALE * falloff,
                    local_force.z * CONTACT_FORCE_SCALE * falloff,
                ),
            );
        }
        processed += 1;
    }
    processed
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
    frame.destructible_bodies = state.destructible.body_count();
    frame.support_bodies = state.destructible.support_body_count();
    frame.dynamic_bodies = state.destructible.dynamic_body_count(&state.bodies);
    frame.awake_dynamic_bodies = state.destructible.awake_dynamic_body_count(&state.bodies);
    frame.sleeping_dynamic_bodies = state
        .destructible
        .sleeping_dynamic_body_count(&state.bodies);
    frame.world_colliders = state.colliders.len();
    frame.destructible_colliders = state.destructible.collider_count();
    frame.projectile_count = state.projectiles.len();
    frame.ccd_bodies = ccd_bodies;
    frame.contact_pairs = contact_pairs;
    frame.active_contact_pairs = active_contact_pairs;
    frame.contact_manifolds = contact_manifolds;
    frame.pending_split_events = state.destructible.pending_split_event_count();
    frame.pending_new_bodies = state.destructible.pending_new_body_count(&state.bodies);
    frame.pending_collider_migrations = state
        .destructible
        .pending_collider_migration_count(&state.bodies);
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

        let color = state
            .destructible
            .node_body(chunk.node_index)
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
    let body_handle = state.destructible.node_body(node_index)?;
    let body = state.bodies.get(body_handle)?;
    let local = state.destructible.node_local_offset(node_index)?;
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
        draw_box_gizmo(
            &mut gizmos,
            transform.translation,
            transform.rotation,
            chunk.size,
            tint.color,
        );
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

fn headless_exit_system(
    mut budget: ResMut<HeadlessRunBudget>,
    state: Option<NonSend<DemoPhysicsState>>,
    mut app_exit_writer: MessageWriter<AppExit>,
) {
    if budget.frames_remaining > 0 {
        budget.frames_remaining -= 1;
    }

    if budget.frames_remaining == 0 {
        let actors = state
            .as_ref()
            .map_or(0, |state| state.destructible.actor_count());
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
        let actors = state.destructible.actor_count();
        let bodies = state.destructible.body_count();
        let world_bodies = state.bodies.len();
        let fps = profiler.fps();
        *text = Text::new(format!(
            "{}\n{}\nLeft click: Shoot  |  Right drag or Ctrl+Left drag: Orbit  |  Scroll: Zoom  |  R: Reset  |  [-]/[=]: Projectile Mass\n{}\nProjectile Mass: {}\nActors: {actors}  |  Destructible Bodies: {bodies}  |  World Bodies: {world_bodies}\nScene Last Frame: support_bodies={} dynamic_bodies={} awake={} sleeping={} world_colliders={} destructible_colliders={} projectiles={} ccd_bodies={}\nContacts Last Frame: pairs={} active_pairs={} manifolds={} pending_splits={} pending_new_bodies={} pending_migrations={}\nFPS: {fps:.1}  |  Frame CPU: {:.2} ms avg / {:.2} ms last\nPhysics: {:.2} ms avg / {:.2} ms last  |  Rapier passes: {:.2} avg / {} last\nRapier: {:.2} ms  |  Collision Events: {:.2} ms  |  Contact Forces: {:.2} ms  |  Solver: {:.2} ms\nResim Restore: {:.2} ms  |  Snapshot: {:.2} ms  |  Optimization: {:.2} ms  |  Projectile Cleanup: {:.2} ms\nRender Prep CPU: {:.2} ms avg / {:.2} ms last  |  Sync: {:.2} ms  |  Gizmos: {:.2} ms  |  HUD: {:.2} ms  |  Other CPU: {:.2} ms\nEvents Last Frame: collisions={} contacts={} fractures={} splits={} new_bodies={} removed_nodes={} removed_projectiles={}",
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
            profiler.last_new_bodies,
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
    if let Some(line) = profiler.finish_frame() {
        perf_log.write_line(&line);
    }
}
