use std::{any::Any, collections::HashMap, env, panic, sync::Arc, time::Duration};

use bevy::app::{AppExit, ScheduleRunnerPlugin};
use bevy::color::LinearRgba;
use bevy::ecs::message::{MessageReader, MessageWriter};
use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::pbr::{Material, MaterialPlugin};
use bevy::prelude::*;
use bevy::reflect::TypePath;
use bevy::render::render_resource::AsBindGroup;
use bevy::shader::ShaderRef;
use bevy::transform::TransformPlugin;
use bevy::window::PrimaryWindow;
use bevy_rapier3d::prelude::*;

use blast_stress_solver::ext_stress_solver::ExtStressSolver;
use blast_stress_solver::types::{
    BondDesc, ForceMode, NodeDesc, SolverSettings, Vec3 as SolverVec3,
};

// ---------------------------------------------------------------------------
// Constants matching the JS wall-demolition demo
// ---------------------------------------------------------------------------

const WALL_COLUMNS: u32 = 12;
const WALL_ROWS: u32 = 6;
const BRICK_W: f32 = 0.5;
const BRICK_H: f32 = 0.5;
const BRICK_D: f32 = 0.32;
const DENSITY: f32 = 138.9;
const GRAVITY: f32 = -9.81;

const COMPRESSION_ELASTIC: f32 = 90_000.0;
const COMPRESSION_FATAL: f32 = 270_000.0;
const TENSION_ELASTIC: f32 = 90_000.0;
const TENSION_FATAL: f32 = 270_000.0;
const SHEAR_ELASTIC: f32 = 120_000.0;
const SHEAR_FATAL: f32 = 360_000.0;

const PROJECTILE_RADIUS: f32 = 0.35;
const PROJECTILE_MASS: f32 = 15_000.0;
const PROJECTILE_SPEED: f32 = 20.0;
const PROJECTILE_TTL: f32 = 6.0;

const CONTACT_FORCE_SCALE: f32 = 30.0;
const DEFAULT_HEADLESS_FRAMES: u32 = 180;
const BODY_SHADER_ASSET_PATH: &str = "shaders/body_material.wgsl";

#[derive(Asset, TypePath, AsBindGroup, Debug, Clone)]
struct BodyMaterial {
    #[uniform(0)]
    color: LinearRgba,
}

impl Material for BodyMaterial {
    fn fragment_shader() -> ShaderRef {
        BODY_SHADER_ASSET_PATH.into()
    }
}

// ---------------------------------------------------------------------------
// Components
// ---------------------------------------------------------------------------

#[derive(Component)]
struct Brick {
    node_index: u32,
}

#[derive(Component)]
struct Projectile {
    ttl: f32,
}

#[derive(Component)]
struct MainCamera;

#[derive(Component)]
struct HudText;

// ---------------------------------------------------------------------------
// Resources
// ---------------------------------------------------------------------------

#[derive(Resource, Clone, Copy)]
struct RunMode {
    headless: bool,
}

#[derive(Resource)]
struct HeadlessRunBudget {
    frames_remaining: u32,
}

#[derive(Resource)]
struct StressSolverState {
    solver: ExtStressSolver,
    nodes: Vec<NodeDesc>,
    _bonds: Vec<BondDesc>,
    node_to_entity: HashMap<u32, Entity>,
}

#[derive(Resource)]
struct CameraOrbit {
    yaw: f32,
    pitch: f32,
    distance: f32,
    target: Vec3,
}

impl Default for CameraOrbit {
    fn default() -> Self {
        Self {
            yaw: 0.0,
            pitch: 0.25,
            distance: 14.0,
            target: Vec3::new(0.0, 1.5, 0.0),
        }
    }
}

// ---------------------------------------------------------------------------
// App
// ---------------------------------------------------------------------------

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
    let mut app = App::new();
    app.insert_resource(RunMode { headless });
    app.insert_resource(CameraOrbit::default());
    app.add_plugins(RapierPhysicsPlugin::<NoUserData>::default());
    app.add_systems(Startup, setup_wall);
    app.add_systems(Update, stress_solver_step_system);

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
        app.add_systems(Startup, setup_physics_world);
        app.add_systems(Update, headless_exit_system);
    } else {
        app.insert_resource(ClearColor(Color::srgb(0.07, 0.08, 0.10)));
        app.add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "Blast Stress Solver - Wall Demolition".into(),
                ..default()
            }),
            ..default()
        }));
        app.add_plugins(MaterialPlugin::<BodyMaterial>::default());
        if debug_render_enabled() {
            app.add_plugins(RapierDebugRenderPlugin::default());
        }
        app.add_systems(Startup, (setup_scene, setup_physics_world));
        app.add_systems(
            Update,
            (
                camera_orbit_system,
                gizmo_render_system,
                shoot_projectile_system,
                projectile_cleanup_system,
                hud_system,
            ),
        );
    }

    app.run()
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

fn debug_render_enabled() -> bool {
    env_flag("BLAST_STRESS_DEMO_DEBUG_RENDER").unwrap_or(false)
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

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

fn setup_scene(mut commands: Commands, mut ambient_light: ResMut<GlobalAmbientLight>) {
    *ambient_light = GlobalAmbientLight {
        color: Color::srgb(0.92, 0.94, 1.0),
        brightness: 650.0,
        ..default()
    };

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-4.5, 4.2, 11.0).looking_at(Vec3::new(0.0, 1.4, 0.0), Vec3::Y),
        MainCamera,
    ));

    commands.spawn((
        PointLight {
            color: Color::WHITE,
            intensity: 18_000_000.0,
            range: 60.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.5, 10.0, 8.0),
    ));

    commands.spawn((
        PointLight {
            color: Color::srgb(0.86, 0.92, 1.0),
            intensity: 6_000_000.0,
            range: 50.0,
            shadows_enabled: false,
            ..default()
        },
        Transform::from_xyz(-8.0, 6.5, -6.0),
    ));

    // HUD text
    commands.spawn((
        Text::new("Blast Stress Solver - Wall Demolition\nLeft click: Shoot  |  Right drag: Orbit  |  Scroll: Zoom"),
        TextFont { font_size: 16.0, ..default() },
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

fn setup_physics_world(
    mut commands: Commands,
    mode: Res<RunMode>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<BodyMaterial>>>,
) {
    commands.spawn((
        Transform::from_xyz(0.0, -0.025, 0.0),
        Collider::cuboid(100.0, 0.025, 100.0),
        RigidBody::Fixed,
        Friction::coefficient(0.9),
    ));

    if mode.headless {
        return;
    }

    if !mesh_visuals_enabled() {
        return;
    }

    let (Some(mut meshes), Some(mut materials)) = (meshes, materials) else {
        return;
    };

    commands.spawn((
        Mesh3d(meshes.add(Cuboid::new(30.0, 0.05, 30.0))),
        MeshMaterial3d(materials.add(BodyMaterial {
            color: LinearRgba::from(Color::srgb(0.18, 0.22, 0.28)),
        })),
        Transform::from_xyz(0.0, -0.025, 0.0),
        ColliderDebugColor(Color::hsla(210.0, 0.20, 0.62, 1.0).into()),
    ));
}

fn setup_wall(
    mut commands: Commands,
    mode: Res<RunMode>,
    meshes: Option<ResMut<Assets<Mesh>>>,
    materials: Option<ResMut<Assets<BodyMaterial>>>,
) {
    let (nodes, bonds) = build_wall_scenario();

    let settings = SolverSettings {
        max_solver_iterations_per_frame: 24,
        compression_elastic_limit: COMPRESSION_ELASTIC,
        compression_fatal_limit: COMPRESSION_FATAL,
        tension_elastic_limit: TENSION_ELASTIC,
        tension_fatal_limit: TENSION_FATAL,
        shear_elastic_limit: SHEAR_ELASTIC,
        shear_fatal_limit: SHEAR_FATAL,
        ..SolverSettings::default()
    };

    let solver =
        ExtStressSolver::new(&nodes, &bonds, &settings).expect("Failed to create stress solver");

    let row_colors = [
        Color::srgb(0.78, 0.62, 0.42),
        Color::srgb(0.74, 0.58, 0.39),
        Color::srgb(0.82, 0.66, 0.45),
        Color::srgb(0.76, 0.60, 0.41),
        Color::srgb(0.80, 0.64, 0.43),
        Color::srgb(0.72, 0.56, 0.37),
    ];

    let (brick_mesh, support_mat, brick_mats) = if mode.headless || !mesh_visuals_enabled() {
        (None, None, Vec::new())
    } else {
        match (meshes, materials) {
            (Some(mut meshes), Some(mut materials)) => {
                let brick_mesh =
                    meshes.add(Cuboid::new(BRICK_W * 0.98, BRICK_H * 0.98, BRICK_D * 0.98));
                let support_mat = materials.add(BodyMaterial {
                    color: LinearRgba::from(Color::srgb(0.20, 0.24, 0.30)),
                });
                let brick_mats = row_colors
                    .iter()
                    .map(|&c| materials.add(BodyMaterial { color: c.into() }))
                    .collect();
                (Some(brick_mesh), Some(support_mat), brick_mats)
            }
            _ => (None, None, Vec::new()),
        }
    };

    let mut node_to_entity = HashMap::new();

    for (i, node) in nodes.iter().enumerate() {
        let row = i as u32 / WALL_COLUMNS;
        let is_support = node.mass == 0.0;
        let pos = Vec3::new(node.centroid.x, node.centroid.y, node.centroid.z);

        let mut entity = if is_support {
            commands.spawn((
                Transform::from_translation(pos),
                RigidBody::Fixed,
                Collider::cuboid(BRICK_W * 0.5, BRICK_H * 0.5, BRICK_D * 0.5),
                Friction::coefficient(0.25),
                Restitution::coefficient(0.0),
                Brick {
                    node_index: i as u32,
                },
            ))
        } else {
            commands.spawn((
                Transform::from_translation(pos),
                RigidBody::Dynamic,
                Collider::cuboid(BRICK_W * 0.5, BRICK_H * 0.5, BRICK_D * 0.5),
                ColliderMassProperties::Mass(node.mass),
                Friction::coefficient(0.25),
                Restitution::coefficient(0.0),
                Brick {
                    node_index: i as u32,
                },
            ))
        };

        if let Some(mesh) = brick_mesh.clone() {
            let material_handle = if is_support {
                support_mat
                    .as_ref()
                    .cloned()
                    .unwrap_or_else(|| brick_mats[row as usize % brick_mats.len()].clone())
            } else {
                brick_mats[row as usize % brick_mats.len()].clone()
            };
            let collider_color = if is_support {
                Color::hsla(210.0, 0.25, 0.86, 1.0)
            } else {
                Color::hsla(28.0, 0.65, 0.18, 1.0)
            };
            entity.insert((
                Mesh3d(mesh),
                MeshMaterial3d(material_handle),
                ColliderDebugColor(collider_color.into()),
            ));
        }

        node_to_entity.insert(i as u32, entity.id());
    }

    commands.insert_resource(StressSolverState {
        solver,
        nodes,
        _bonds: bonds,
        node_to_entity,
    });
}

// ---------------------------------------------------------------------------
// Wall builder
// ---------------------------------------------------------------------------

fn build_wall_scenario() -> (Vec<NodeDesc>, Vec<BondDesc>) {
    let mut nodes = Vec::new();
    let mut bonds = Vec::new();

    let cols = WALL_COLUMNS;
    let rows = WALL_ROWS;
    let volume = BRICK_W * BRICK_H * BRICK_D;

    let idx = |col: u32, row: u32| -> u32 { row * cols + col };

    for row in 0..rows {
        for col in 0..cols {
            let x = col as f32 * BRICK_W + BRICK_W * 0.5 - (cols as f32 * BRICK_W) * 0.5;
            let y = BRICK_H * 0.5 + row as f32 * BRICK_H;
            let mass = if row == 0 { 0.0 } else { DENSITY };
            nodes.push(NodeDesc {
                centroid: SolverVec3::new(x, y, 0.0),
                mass,
                volume,
            });
        }
    }

    // Horizontal bonds
    for row in 0..rows {
        for col in 0..cols - 1 {
            let n0 = idx(col, row);
            let n1 = idx(col + 1, row);
            let c0 = nodes[n0 as usize].centroid;
            let c1 = nodes[n1 as usize].centroid;
            bonds.push(BondDesc {
                centroid: solver_mid(c0, c1),
                normal: SolverVec3::new(1.0, 0.0, 0.0),
                area: BRICK_H * BRICK_D * 0.05,
                node0: n0,
                node1: n1,
            });
        }
    }

    // Vertical bonds
    for row in 0..rows - 1 {
        for col in 0..cols {
            let n0 = idx(col, row);
            let n1 = idx(col, row + 1);
            let c0 = nodes[n0 as usize].centroid;
            let c1 = nodes[n1 as usize].centroid;
            bonds.push(BondDesc {
                centroid: solver_mid(c0, c1),
                normal: SolverVec3::new(0.0, 1.0, 0.0),
                area: BRICK_W * BRICK_D * 0.05,
                node0: n0,
                node1: n1,
            });
        }
    }

    (nodes, bonds)
}

fn solver_mid(a: SolverVec3, b: SolverVec3) -> SolverVec3 {
    SolverVec3::new((a.x + b.x) * 0.5, (a.y + b.y) * 0.5, (a.z + b.z) * 0.5)
}

// ---------------------------------------------------------------------------
// Camera orbit
// ---------------------------------------------------------------------------

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
        // Drain events we don't use
        for _ in mouse_motion.read() {}
    }

    for ev in scroll.read() {
        orbit.distance -= ev.y * 0.5;
        orbit.distance = orbit.distance.clamp(3.0, 50.0);
    }

    if let Ok(mut tf) = camera_q.single_mut() {
        let x = orbit.distance * orbit.yaw.cos() * orbit.pitch.cos();
        let y = orbit.distance * orbit.pitch.sin();
        let z = orbit.distance * orbit.yaw.sin() * orbit.pitch.cos();
        tf.translation = orbit.target + Vec3::new(x, y, z);
        tf.look_at(orbit.target, Vec3::Y);
    }
}

// ---------------------------------------------------------------------------
// Projectile shooting
// ---------------------------------------------------------------------------

fn shoot_projectile_system(
    mut commands: Commands,
    mouse_button: Res<ButtonInput<MouseButton>>,
    camera_q: Query<(&Camera, &GlobalTransform), With<MainCamera>>,
    window_q: Query<&Window, With<PrimaryWindow>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<BodyMaterial>>,
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
    let direction = ray.direction.as_vec3();

    let mut entity = commands.spawn((
        Transform::from_translation(origin),
        RigidBody::Dynamic,
        Collider::ball(PROJECTILE_RADIUS),
        ColliderDebugColor(Color::hsla(55.0, 0.90, 0.76, 1.0).into()),
        ColliderMassProperties::Mass(PROJECTILE_MASS),
        Velocity::linear(direction * PROJECTILE_SPEED),
        Ccd::enabled(),
        Projectile {
            ttl: PROJECTILE_TTL,
        },
    ));

    if mesh_visuals_enabled() {
        entity.insert((
            Mesh3d(meshes.add(Sphere::new(PROJECTILE_RADIUS))),
            MeshMaterial3d(materials.add(BodyMaterial {
                color: LinearRgba::from(Color::srgb(0.96, 0.84, 0.16)),
            })),
        ));
    }
}

// ---------------------------------------------------------------------------
// Projectile cleanup
// ---------------------------------------------------------------------------

fn projectile_cleanup_system(
    mut commands: Commands,
    time: Res<Time>,
    mut query: Query<(Entity, &mut Projectile)>,
) {
    for (entity, mut proj) in &mut query {
        proj.ttl -= time.delta_secs();
        if proj.ttl <= 0.0 {
            commands.entity(entity).despawn();
        }
    }
}

fn gizmo_render_system(
    mut gizmos: Gizmos,
    brick_q: Query<(&Brick, &Transform)>,
    projectile_q: Query<&Transform, With<Projectile>>,
) {
    draw_floor_gizmo(&mut gizmos);

    for (brick, transform) in &brick_q {
        let color = if brick.node_index < WALL_COLUMNS {
            Color::srgb(0.52, 0.72, 0.92)
        } else {
            Color::srgb(0.92, 0.72, 0.38)
        };
        draw_box_gizmo(
            &mut gizmos,
            transform.translation,
            transform.rotation,
            Vec3::new(BRICK_W, BRICK_H, BRICK_D),
            color,
        );
    }

    for transform in &projectile_q {
        let isometry = Isometry3d::from_translation(transform.translation);
        gizmos
            .sphere(isometry, PROJECTILE_RADIUS, Color::srgb(1.0, 0.95, 0.32))
            .resolution(20);
        gizmos.line(
            transform.translation + Vec3::X * PROJECTILE_RADIUS,
            transform.translation - Vec3::X * PROJECTILE_RADIUS,
            Color::WHITE,
        );
        gizmos.line(
            transform.translation + Vec3::Y * PROJECTILE_RADIUS,
            transform.translation - Vec3::Y * PROJECTILE_RADIUS,
            Color::WHITE,
        );
        gizmos.line(
            transform.translation + Vec3::Z * PROJECTILE_RADIUS,
            transform.translation - Vec3::Z * PROJECTILE_RADIUS,
            Color::WHITE,
        );
    }
}

fn draw_floor_gizmo(gizmos: &mut Gizmos) {
    let y = 0.0;
    let half = 8.0;
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

    for step in -8..=8 {
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

// ---------------------------------------------------------------------------
// Stress solver step
// ---------------------------------------------------------------------------

fn stress_solver_step_system(
    mut entity_commands: Commands,
    state: Option<ResMut<StressSolverState>>,
    brick_q: Query<(&Brick, &Transform, &Velocity)>,
) {
    let Some(mut state) = state else { return };

    // Apply gravity to the solver
    state.solver.add_gravity(SolverVec3::new(0.0, GRAVITY, 0.0));

    // Feed brick velocities as forces into the solver (contact force proxy)
    for (brick, _tf, vel) in &brick_q {
        let node = brick.node_index;
        if state.nodes[node as usize].mass == 0.0 {
            continue;
        }
        let v = vel.linvel;
        let speed_sq = v.length_squared();
        if speed_sq > 1.0 {
            let node_pos = state.nodes[node as usize].centroid;
            state.solver.add_force(
                node,
                node_pos,
                SolverVec3::new(v.x, v.y, v.z) * CONTACT_FORCE_SCALE,
                ForceMode::Force,
            );
        }
    }

    // Update solver
    state.solver.update();

    let overstressed = state.solver.overstressed_bond_count();
    if overstressed == 0 {
        return;
    }

    // Generate and apply fractures
    let commands_list = state.solver.generate_fracture_commands();
    if commands_list.is_empty() {
        return;
    }

    let events = state.solver.apply_fracture_commands(&commands_list);

    if !events.is_empty() {
        // Get updated actor table
        let actors = state.solver.actors();

        // Apply excess forces to actors separated from supports
        for actor in &actors {
            let has_support = actor
                .nodes
                .iter()
                .any(|&n| (n as usize) < state.nodes.len() && state.nodes[n as usize].mass == 0.0);
            if has_support {
                continue;
            }

            let mut com = SolverVec3::ZERO;
            let mut count = 0.0f32;
            for &n in &actor.nodes {
                let c = state.nodes[n as usize].centroid;
                com.x += c.x;
                com.y += c.y;
                com.z += c.z;
                count += 1.0;
            }
            if count > 0.0 {
                com = com / count;
            }

            if let Some((force, _torque)) = state.solver.get_excess_forces(actor.actor_index, com) {
                let force_mag = force.x * force.x + force.y * force.y + force.z * force.z;
                if force_mag > 1.0 {
                    for &n in &actor.nodes {
                        if let Some(&entity) = state.node_to_entity.get(&n) {
                            entity_commands.entity(entity).insert(ExternalForce {
                                force: Vec3::new(force.x, force.y, force.z) * CONTACT_FORCE_SCALE,
                                ..default()
                            });
                        }
                    }
                }
            }
        }
    }
}

fn headless_exit_system(
    mut budget: ResMut<HeadlessRunBudget>,
    state: Option<Res<StressSolverState>>,
    mut app_exit_writer: MessageWriter<AppExit>,
) {
    if budget.frames_remaining > 0 {
        budget.frames_remaining -= 1;
    }

    if budget.frames_remaining == 0 {
        let actors = state.as_ref().map_or(0, |state| state.solver.actor_count());
        eprintln!("Headless simulation complete. Actor count: {actors}.");
        app_exit_writer.write(AppExit::Success);
    }
}

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------

fn hud_system(state: Option<Res<StressSolverState>>, mut text_q: Query<&mut Text, With<HudText>>) {
    let Some(state) = state else { return };
    if let Ok(mut text) = text_q.single_mut() {
        let actors = state.solver.actor_count();
        *text = Text::new(format!(
            "Blast Stress Solver - Wall Demolition\n\
             Left click: Shoot  |  Right drag: Orbit  |  Scroll: Zoom\n\
             Actors: {actors}"
        ));
    }
}
