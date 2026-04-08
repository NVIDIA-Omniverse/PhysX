use std::collections::HashMap;

use bevy::input::mouse::{MouseMotion, MouseWheel};
use bevy::prelude::*;
use bevy::ecs::message::MessageReader;
use bevy::window::PrimaryWindow;
use bevy_rapier3d::prelude::*;

use blast_stress_solver::ext_stress_solver::ExtStressSolver;
use blast_stress_solver::types::{
    BondDesc, ForceMode, NodeDesc, SolverSettings,
    Vec3 as SolverVec3,
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

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::srgb(0.1, 0.1, 0.15)))
        .add_plugins((
            DefaultPlugins.set(WindowPlugin {
                primary_window: Some(Window {
                    title: "Blast Stress Solver - Wall Demolition".into(),
                    ..default()
                }),
                ..default()
            }),
            RapierPhysicsPlugin::<NoUserData>::default(),
            RapierDebugRenderPlugin::default(),
        ))
        .insert_resource(CameraOrbit::default())
        .add_systems(Startup, (setup_scene, setup_wall))
        .add_systems(Update, (camera_orbit_system, shoot_projectile_system))
        .add_systems(Update, (projectile_cleanup_system, stress_solver_step_system))
        .add_systems(Update, hud_system)
        .run();
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

fn setup_scene(mut commands: Commands, mut meshes: ResMut<Assets<Mesh>>, mut materials: ResMut<Assets<StandardMaterial>>) {
    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 3.0, 14.0).looking_at(Vec3::new(0.0, 1.5, 0.0), Vec3::Y),
        MainCamera,
    ));

    // Light
    commands.spawn((
        DirectionalLight {
            illuminance: 8000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(5.0, 10.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Ground physics
    commands.spawn((
        Transform::from_xyz(0.0, -0.025, 0.0),
        Collider::cuboid(100.0, 0.025, 100.0),
        RigidBody::Fixed,
        Friction::coefficient(0.9),
    ));

    // Ground visual
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::new(Vec3::Y, Vec2::splat(30.0)))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.15, 0.18, 0.25),
            perceptual_roughness: 0.9,
            ..default()
        })),
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

fn setup_wall(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
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

    let solver = ExtStressSolver::new(&nodes, &bonds, &settings)
        .expect("Failed to create stress solver");

    let row_colors = [
        Color::srgb(0.85, 0.55, 0.35),
        Color::srgb(0.75, 0.50, 0.30),
        Color::srgb(0.80, 0.45, 0.25),
        Color::srgb(0.70, 0.42, 0.28),
        Color::srgb(0.82, 0.52, 0.32),
        Color::srgb(0.78, 0.48, 0.30),
    ];

    let brick_mesh = meshes.add(Cuboid::new(BRICK_W * 0.98, BRICK_H * 0.98, BRICK_D * 0.98));
    let brick_mats: Vec<_> = row_colors
        .iter()
        .map(|&c| materials.add(StandardMaterial {
            base_color: c,
            perceptual_roughness: 0.7,
            ..default()
        }))
        .collect();

    let mut node_to_entity = HashMap::new();

    for (i, node) in nodes.iter().enumerate() {
        let row = i as u32 / WALL_COLUMNS;
        let is_support = node.mass == 0.0;
        let mat = brick_mats[row as usize % brick_mats.len()].clone();

        let pos = Vec3::new(node.centroid.x, node.centroid.y, node.centroid.z);

        let entity = if is_support {
            commands.spawn((
                Mesh3d(brick_mesh.clone()),
                MeshMaterial3d(mat),
                Transform::from_translation(pos),
                RigidBody::Fixed,
                Collider::cuboid(BRICK_W * 0.5, BRICK_H * 0.5, BRICK_D * 0.5),
                Friction::coefficient(0.25),
                Restitution::coefficient(0.0),
                Brick { node_index: i as u32 },
            )).id()
        } else {
            commands.spawn((
                Mesh3d(brick_mesh.clone()),
                MeshMaterial3d(mat),
                Transform::from_translation(pos),
                RigidBody::Dynamic,
                Collider::cuboid(BRICK_W * 0.5, BRICK_H * 0.5, BRICK_D * 0.5),
                ColliderMassProperties::Mass(node.mass),
                Friction::coefficient(0.25),
                Restitution::coefficient(0.0),
                Brick { node_index: i as u32 },
            )).id()
        };
        node_to_entity.insert(i as u32, entity);
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
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    if !mouse_button.just_pressed(MouseButton::Left) {
        return;
    }

    let Ok((camera, cam_tf)) = camera_q.single() else { return };
    let Ok(window) = window_q.single() else { return };
    let Some(cursor_pos) = window.cursor_position() else { return };
    let Ok(ray) = camera.viewport_to_world(cam_tf, cursor_pos) else { return };

    let origin = ray.origin;
    let direction = ray.direction.as_vec3();

    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(PROJECTILE_RADIUS))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgb(0.2, 0.2, 0.9),
            metallic: 0.5,
            ..default()
        })),
        Transform::from_translation(origin),
        RigidBody::Dynamic,
        Collider::ball(PROJECTILE_RADIUS),
        ColliderMassProperties::Mass(PROJECTILE_MASS),
        Velocity::linear(direction * PROJECTILE_SPEED),
        Ccd::enabled(),
        Projectile { ttl: PROJECTILE_TTL },
    ));
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
            let has_support = actor.nodes.iter().any(|&n| {
                (n as usize) < state.nodes.len() && state.nodes[n as usize].mass == 0.0
            });
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

            if let Some((force, _torque)) =
                state.solver.get_excess_forces(actor.actor_index, com)
            {
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

// ---------------------------------------------------------------------------
// HUD
// ---------------------------------------------------------------------------

fn hud_system(
    state: Option<Res<StressSolverState>>,
    mut text_q: Query<&mut Text, With<HudText>>,
) {
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
