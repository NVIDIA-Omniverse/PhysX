use std::{
    any::Any,
    collections::HashMap,
    env,
    panic,
    sync::{
        mpsc::{channel, Receiver, Sender},
        Arc,
    },
    time::Duration,
};

use bevy::math::primitives::{Cuboid as BevyCuboid, Sphere};
use bevy::app::{AppExit, ScheduleRunnerPlugin};
use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy::transform::TransformPlugin;
use bevy::window::PrimaryWindow;
use blast_stress_solver::rapier::{DestructibleSet, FracturePolicy};
use blast_stress_solver::scenarios::{
    build_bridge_scenario, build_tower_scenario, build_wall_scenario, BridgeOptions,
    TowerOptions, WallOptions,
};
use blast_stress_solver::{ScenarioDesc, SolverSettings, Vec3 as SolverVec3};
use rapier3d::prelude::*;

const GRAVITY: f32 = -9.81;
const CONTACT_FORCE_SCALE: f32 = 30.0;
const SPLASH_RADIUS: f32 = 2.0;
const DEFAULT_HEADLESS_FRAMES: u32 = 180;
const DEFAULT_PROJECTILE_TTL: f32 = 6.0;
const MAX_RESIMULATION_PASSES: usize = 1;

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

#[derive(Clone, Copy)]
struct ProjectileState {
    body_handle: RigidBodyHandle,
    collider_handle: ColliderHandle,
    ttl: f32,
}

#[derive(Clone, Copy)]
struct BodySnapshot {
    handle: RigidBodyHandle,
    position: Isometry<f32>,
    linvel: Vector<f32>,
    angvel: Vector<f32>,
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
    let config = build_demo_config(kind);
    let info = DemoInfo {
        title: format!("Blast Stress Solver - {}", config.title),
        subtitle: format!("Scenario: {}", kind.slug()),
        camera_target: config.camera_target,
        camera_distance: config.camera_distance,
    };

    let physics = build_demo_physics(config.clone());

    let mut app = App::new();
    app.insert_resource(RunMode { headless });
    app.insert_resource(info.clone());
    app.insert_resource(CameraOrbit::from_info(&info));
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
        app.add_systems(Update, (physics_step_system, headless_exit_system).chain());
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
                camera_orbit_system,
                shoot_projectile_system,
                physics_step_system,
                sync_visuals_system,
                gizmo_render_system,
                hud_system,
            )
                .chain(),
        );
    }

    app.run()
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
        },
    }
}

fn build_demo_physics(config: DemoConfig) -> DemoPhysicsState {
    let settings = scaled_solver_settings(config.material_scale);
    let gravity = SolverVec3::new(0.0, GRAVITY, 0.0);
    let policy = FracturePolicy {
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut destructible = DestructibleSet::from_scenario(
        &config.scenario,
        settings,
        gravity,
        policy,
    )
    .expect("failed to create destructible set");

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    let _handles = destructible.initialize(&mut bodies, &mut colliders);

    let ground_body = bodies.insert(RigidBodyBuilder::fixed().translation(vector![0.0, 0.0, 0.0]));
    let ground = ColliderBuilder::cuboid(100.0, 0.025, 100.0)
        .translation(vector![0.0, -0.025, 0.0])
        .friction(0.9)
        .restitution(0.0);
    colliders.insert_with_parent(ground, ground_body, &mut bodies);

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

    commands.spawn((
        Camera3d::default(),
        camera_transform(&orbit),
        MainCamera,
    ));

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
            "{}\nLeft click: Shoot  |  Right drag: Orbit  |  Scroll: Zoom",
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
    mut state: NonSendMut<DemoPhysicsState>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<StandardMaterial>>>,
) {
    if mode.headless {
        return;
    }

    let show_meshes = mesh_visuals_enabled();
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
            if let (Some(meshes), Some(materials)) = (&mut meshes, &mut materials) {
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
    mut mouse_motion: MessageReader<MouseMotion>,
    mut scroll: MessageReader<MouseWheel>,
    mut orbit: ResMut<CameraOrbit>,
    mut camera_q: Query<&mut Transform, With<MainCamera>>,
) {
    if mouse_button.pressed(MouseButton::Right) {
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

fn shoot_projectile_system(
    mut commands: Commands,
    mouse_button: Res<ButtonInput<MouseButton>>,
    camera_q: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
    window_q: Query<&Window, With<PrimaryWindow>>,
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
    let projectile_mass = state.config.projectile_mass;
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
            .ccd_enabled(true),
    );

    let collider_handle = colliders.insert_with_parent(
        ColliderBuilder::ball(projectile_radius)
            .mass(projectile_mass)
            .friction(0.25)
            .restitution(0.0)
            .active_events(ActiveEvents::CONTACT_FORCE_EVENTS)
            .contact_force_event_threshold(0.0),
        body_handle,
        bodies,
    );

    let entity = commands
        .spawn((
            Transform::from_translation(origin),
            ProjectileVisual { radius: projectile_radius },
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
    mut state: NonSendMut<DemoPhysicsState>,
) {
    let dt = time.delta_secs().clamp(1.0 / 240.0, 1.0 / 30.0);
    let pre_step_snapshot = capture_body_snapshots(&state.bodies);
    step_rapier_world(&mut state, dt);
    drain_contact_forces(&mut state);

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
            destructible.step(
                bodies,
                colliders,
                island_manager,
                impulse_joints,
                multibody_joints,
            )
    };

    if fracture_result.split_events > 0 || fracture_result.new_bodies > 0 {
        for _ in 0..MAX_RESIMULATION_PASSES {
            restore_body_snapshots(&mut state.bodies, &pre_step_snapshot);
            step_rapier_world(&mut state, dt);
            drain_contact_forces(&mut state);
        }
    }

    cleanup_projectiles(&mut state, &mut commands, dt);
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
        collision_recv,
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

    while collision_recv.try_recv().is_ok() {}
}

fn capture_body_snapshots(bodies: &RigidBodySet) -> Vec<BodySnapshot> {
    bodies
        .iter()
        .filter_map(|(handle, body)| {
            if body.is_fixed() {
                None
            } else {
                Some(BodySnapshot {
                    handle,
                    position: *body.position(),
                    linvel: *body.linvel(),
                    angvel: *body.angvel(),
                })
            }
        })
        .collect()
}

fn restore_body_snapshots(bodies: &mut RigidBodySet, snapshots: &[BodySnapshot]) {
    for snapshot in snapshots {
        let Some(body) = bodies.get_mut(snapshot.handle) else {
            continue;
        };
        body.set_position(snapshot.position, true);
        body.set_linvel(snapshot.linvel, true);
        body.set_angvel(snapshot.angvel, true);
        body.reset_forces(true);
        body.reset_torques(true);
    }
}

fn drain_contact_forces(state: &mut DemoPhysicsState) {
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
        let velocity_vec = Vec3::new(projectile_velocity.x, projectile_velocity.y, projectile_velocity.z);
        let direction = if velocity_vec.length_squared() > 1.0e-6 {
            velocity_vec.normalize()
        } else {
            Vec3::new(event.total_force.x, event.total_force.y, event.total_force.z)
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
        let rotation = body.position().rotation;
        let local_force = rotation.inverse() * vector![force_world.x, force_world.y, force_world.z];
        let Some(hit_local) = state.destructible.node_local_offset(node_index) else {
            continue;
        };
        let body_nodes = state.destructible.body_nodes(body_handle);

        for other_node in body_nodes {
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
    }
}

fn cleanup_projectiles(
    state: &mut DemoPhysicsState,
    commands: &mut Commands,
    dt: f32,
) {
    let mut keep = Vec::with_capacity(state.projectiles.len());

    for mut projectile in state.projectiles.drain(..) {
        projectile.ttl -= dt;
        let remove_for_ttl = projectile.ttl <= 0.0;
        let body = state.bodies.get(projectile.body_handle);
        let remove_for_body = body.is_none();
        let remove_for_fall = body
            .map(|rb| rb.translation().y < -20.0)
            .unwrap_or(false);

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
            state.projectile_colliders.remove(&projectile.collider_handle);
            if let Some(entity) = state.projectile_entities.remove(&projectile.body_handle) {
                commands.entity(entity).despawn();
            }
            continue;
        }

        keep.push(projectile);
    }

    state.projectiles = keep;
}

fn sync_visuals_system(
    state: NonSend<DemoPhysicsState>,
    mut chunk_q: Query<
        (&ChunkVisual, &mut Transform, &mut ChunkTint, Option<&ChunkMaterial>),
        Without<ProjectileVisual>,
    >,
    mut projectile_q: Query<&mut Transform, (With<ProjectileVisual>, Without<ChunkVisual>)>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
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
        tint.color = color;
        if let Some(material) = material {
            if let Some(standard_material) = materials.get_mut(&material.handle) {
                standard_material.base_color = color;
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
    chunk_q: Query<(&ChunkVisual, &ChunkTint, &Transform)>,
    projectile_q: Query<(&ProjectileVisual, &Transform)>,
) {
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
        let actors = state.as_ref().map_or(0, |state| state.destructible.actor_count());
        eprintln!("Headless simulation complete. Actor count: {actors}.");
        app_exit_writer.write(AppExit::Success);
    }
}

fn hud_system(
    info: Res<DemoInfo>,
    state: Option<NonSend<DemoPhysicsState>>,
    mut text_q: Query<&mut Text, With<HudText>>,
) {
    let Some(state) = state else { return };
    if let Ok(mut text) = text_q.single_mut() {
        let actors = state.destructible.actor_count();
        let bodies = state.destructible.body_count();
        let world_bodies = state.bodies.len();
        *text = Text::new(format!(
            "{}\n{}\nLeft click: Shoot  |  Right drag: Orbit  |  Scroll: Zoom\nActors: {actors}  |  Destructible Bodies: {bodies}  |  World Bodies: {world_bodies}",
            info.title, info.subtitle
        ));
    }
}
