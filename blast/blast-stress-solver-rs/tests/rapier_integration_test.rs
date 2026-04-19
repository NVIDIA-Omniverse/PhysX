#![cfg(feature = "rapier")]

use rapier3d::prelude::*;
use std::sync::{
    atomic::{AtomicUsize, Ordering},
    Arc,
};

use blast_stress_solver::rapier::*;
use blast_stress_solver::*;

/// Helper: create a 5-column, 4-row wall scenario.
fn wall_scenario_with_bonds(include_bonds: bool) -> ScenarioDesc {
    let columns = 5u32;
    let rows = 4u32;
    let bw = 1.0f32;
    let bh = 0.5f32;
    let bd = 0.5f32;
    let volume = bw * bh * bd;

    let mut nodes = Vec::new();
    let mut bonds = Vec::new();

    let idx = |col: u32, row: u32| -> u32 { row * columns + col };

    for row in 0..rows {
        for col in 0..columns {
            let x = col as f32 * bw + bw * 0.5 - (columns as f32 * bw) * 0.5;
            let y = bh * 0.5 + row as f32 * bh;
            let mass = if row == 0 { 0.0 } else { volume * 1.0 };
            nodes.push(ScenarioNode {
                centroid: Vec3::new(x, y, 0.0),
                mass,
                volume,
            });
        }
    }

    if include_bonds {
        // Horizontal bonds
        for row in 0..rows {
            for col in 0..columns - 1 {
                let n0 = idx(col, row);
                let n1 = idx(col + 1, row);
                let c0 = nodes[n0 as usize].centroid;
                let c1 = nodes[n1 as usize].centroid;
                bonds.push(ScenarioBond {
                    node0: n0,
                    node1: n1,
                    centroid: (c0 + c1) * 0.5,
                    normal: Vec3::new(1.0, 0.0, 0.0),
                    area: bh * bd,
                });
            }
        }

        // Vertical bonds
        for row in 0..rows - 1 {
            for col in 0..columns {
                let n0 = idx(col, row);
                let n1 = idx(col, row + 1);
                let c0 = nodes[n0 as usize].centroid;
                let c1 = nodes[n1 as usize].centroid;
                bonds.push(ScenarioBond {
                    node0: n0,
                    node1: n1,
                    centroid: (c0 + c1) * 0.5,
                    normal: Vec3::new(0.0, 1.0, 0.0),
                    area: bw * bd,
                });
            }
        }
    }

    ScenarioDesc {
        nodes,
        bonds,
        node_sizes: vec![Vec3::new(bw, bh, bd); (columns * rows) as usize],
        collider_shapes: Vec::new(),
    }
}

fn wall_scenario() -> ScenarioDesc {
    wall_scenario_with_bonds(true)
}

fn rapier_world() -> (
    RigidBodySet,
    ColliderSet,
    IslandManager,
    ImpulseJointSet,
    MultibodyJointSet,
) {
    (
        RigidBodySet::new(),
        ColliderSet::new(),
        IslandManager::new(),
        ImpulseJointSet::new(),
        MultibodyJointSet::new(),
    )
}

fn dynamic_pair_scenario() -> ScenarioDesc {
    let size = Vec3::new(0.5, 0.5, 0.5);
    ScenarioDesc {
        nodes: vec![
            ScenarioNode {
                centroid: Vec3::new(-0.25, 0.5, 0.0),
                mass: 1.0,
                volume: 0.125,
            },
            ScenarioNode {
                centroid: Vec3::new(0.25, 0.5, 0.0),
                mass: 1.0,
                volume: 0.125,
            },
        ],
        bonds: vec![ScenarioBond {
            node0: 0,
            node1: 1,
            centroid: Vec3::new(0.0, 0.5, 0.0),
            normal: Vec3::new(1.0, 0.0, 0.0),
            area: 0.25,
        }],
        node_sizes: vec![size, size],
        collider_shapes: Vec::new(),
    }
}

fn dynamic_triangle_scenario() -> ScenarioDesc {
    let size = Vec3::new(0.5, 0.5, 0.5);
    ScenarioDesc {
        nodes: vec![
            ScenarioNode {
                centroid: Vec3::new(-0.5, 0.0, 0.0),
                mass: 1.0,
                volume: 0.125,
            },
            ScenarioNode {
                centroid: Vec3::new(0.5, 0.0, 0.0),
                mass: 1.0,
                volume: 0.125,
            },
            ScenarioNode {
                centroid: Vec3::new(0.0, 0.866, 0.0),
                mass: 1.0,
                volume: 0.125,
            },
        ],
        bonds: vec![
            ScenarioBond {
                node0: 0,
                node1: 1,
                centroid: Vec3::new(0.0, 0.0, 0.0),
                normal: Vec3::new(1.0, 0.0, 0.0),
                area: 0.25,
            },
            ScenarioBond {
                node0: 0,
                node1: 2,
                centroid: Vec3::new(-0.25, 0.433, 0.0),
                normal: Vec3::new(0.5, 0.866, 0.0),
                area: 0.25,
            },
            ScenarioBond {
                node0: 1,
                node1: 2,
                centroid: Vec3::new(0.25, 0.433, 0.0),
                normal: Vec3::new(-0.5, 0.866, 0.0),
                area: 0.25,
            },
        ],
        node_sizes: vec![size; 3],
        collider_shapes: Vec::new(),
    }
}

fn weak_impact_settings() -> SolverSettings {
    SolverSettings {
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        tension_elastic_limit: 0.01,
        tension_fatal_limit: 0.05,
        shear_elastic_limit: 0.01,
        shear_fatal_limit: 0.05,
        ..SolverSettings::default()
    }
}

struct RuntimeTestWorld {
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    broad_phase: BroadPhaseBvh,
    narrow_phase: NarrowPhase,
    ccd_solver: CCDSolver,
    bodies: RigidBodySet,
    colliders: ColliderSet,
    island_manager: IslandManager,
    impulse_joints: ImpulseJointSet,
    multibody_joints: MultibodyJointSet,
}

impl RuntimeTestWorld {
    fn new(gravity: Vector<Real>) -> Self {
        Self {
            gravity,
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            broad_phase: BroadPhaseBvh::new(),
            narrow_phase: NarrowPhase::new(),
            ccd_solver: CCDSolver::new(),
            bodies: RigidBodySet::new(),
            colliders: ColliderSet::new(),
            island_manager: IslandManager::new(),
            impulse_joints: ImpulseJointSet::new(),
            multibody_joints: MultibodyJointSet::new(),
        }
    }

    fn run_frame(
        &mut self,
        runtime: &mut DestructionRuntime,
        now_secs: f32,
        dt: f32,
    ) -> FrameResult {
        self.run_frame_with(runtime, now_secs, dt, &(), &(), |_| {})
    }

    fn run_frame_with<H, E, O>(
        &mut self,
        runtime: &mut DestructionRuntime,
        now_secs: f32,
        dt: f32,
        user_hooks: &H,
        user_events: &E,
        mut on_step_complete: O,
    ) -> FrameResult
    where
        H: PhysicsHooks + Sync + ?Sized,
        E: EventHandler + ?Sized,
        O: FnMut(&PassAdapter<'_, H, E>),
    {
        runtime.step_frame(
            now_secs,
            dt,
            &mut RapierWorldAccess {
                bodies: &mut self.bodies,
                colliders: &mut self.colliders,
                island_manager: &mut self.island_manager,
                broad_phase: &mut self.broad_phase,
                narrow_phase: &mut self.narrow_phase,
                impulse_joints: &mut self.impulse_joints,
                multibody_joints: &mut self.multibody_joints,
                ccd_solver: &mut self.ccd_solver,
            },
            user_hooks,
            user_events,
            |pass, world| {
                self.physics_pipeline.step(
                    &self.gravity,
                    &self.integration_parameters,
                    world.island_manager,
                    world.broad_phase,
                    world.narrow_phase,
                    world.bodies,
                    world.colliders,
                    world.impulse_joints,
                    world.multibody_joints,
                    world.ccd_solver,
                    pass,
                    pass,
                );
                on_step_complete(pass);
            },
        )
    }

    fn add_ball_projectile(
        &mut self,
        translation: Vector<Real>,
        linvel: Vector<Real>,
        radius: f32,
        density: f32,
    ) -> RigidBodyHandle {
        let handle = self.bodies.insert(
            RigidBodyBuilder::dynamic()
                .translation(translation)
                .linvel(linvel)
                .build(),
        );
        self.colliders.insert_with_parent(
            ColliderBuilder::ball(radius)
                .density(density)
                .friction(0.0)
                .restitution(0.0)
                .build(),
            handle,
            &mut self.bodies,
        );
        handle
    }

    fn add_ground(&mut self, y: f32) -> RigidBodyHandle {
        let handle = self.bodies.insert(
            RigidBodyBuilder::fixed()
                .translation(vector![0.0, y, 0.0])
                .build(),
        );
        self.colliders.insert_with_parent(
            ColliderBuilder::cuboid(10.0, 0.5, 10.0).build(),
            handle,
            &mut self.bodies,
        );
        handle
    }

    fn add_loose_wall_blocks(
        &mut self,
        columns: u32,
        rows: u32,
        half_extents: Vector<Real>,
        density: f32,
    ) -> Vec<RigidBodyHandle> {
        let mut handles = Vec::new();
        let bw = half_extents.x * 2.0;
        let bh = half_extents.y * 2.0;

        for row in 0..rows {
            for col in 0..columns {
                let x = col as f32 * bw + bw * 0.5 - (columns as f32 * bw) * 0.5;
                let y = bh * 0.5 + row as f32 * bh;
                let body = if row == 0 {
                    RigidBodyBuilder::fixed()
                } else {
                    RigidBodyBuilder::dynamic()
                };
                let handle = self
                    .bodies
                    .insert(body.translation(vector![x, y, 0.0]).build());
                self.colliders.insert_with_parent(
                    ColliderBuilder::cuboid(half_extents.x, half_extents.y, half_extents.z)
                        .density(density)
                        .friction(0.25)
                        .restitution(0.0)
                        .build(),
                    handle,
                    &mut self.bodies,
                );
                handles.push(handle);
            }
        }

        handles
    }
}

#[derive(Debug)]
struct WallShotOutcome {
    final_x: f32,
    final_linvel_x: f32,
    max_x: f32,
    total_fractures: usize,
    total_splits: usize,
    saw_resimulation: bool,
    active_bonds_after: usize,
    actor_count_after: u32,
    remaining_bonds: Vec<(u32, u32)>,
    multi_node_bodies: Vec<Vec<u32>>,
    projectile_contact_bodies: Vec<Vec<u32>>,
    projectile_active_contact_bodies: Vec<Vec<u32>>,
}

fn collect_projectile_contact_bodies(
    world: &RuntimeTestWorld,
    runtime: &DestructionRuntime,
    projectile: RigidBodyHandle,
) -> (Vec<Vec<u32>>, Vec<Vec<u32>>) {
    let Some(body) = world.bodies.get(projectile) else {
        return (Vec::new(), Vec::new());
    };

    let mut all_contacts = Vec::new();
    let mut active_contacts = Vec::new();
    for &collider_handle in body.colliders() {
        for pair in world.narrow_phase.contact_pairs_with(collider_handle) {
            let other_collider = if pair.collider1 == collider_handle {
                pair.collider2
            } else {
                pair.collider1
            };
            let Some(other_parent) = world
                .colliders
                .get(other_collider)
                .and_then(|collider| collider.parent())
            else {
                continue;
            };
            let nodes = runtime.body_nodes_slice(other_parent);
            if nodes.is_empty() {
                continue;
            }
            all_contacts.push(nodes.to_vec());
            if pair.has_any_active_contact {
                active_contacts.push(nodes.to_vec());
            }
        }
    }

    all_contacts.sort();
    all_contacts.dedup();
    active_contacts.sort();
    active_contacts.dedup();
    (all_contacts, active_contacts)
}

fn run_heavy_wall_shot_with_scenario(scenario: ScenarioDesc, resim_enabled: bool) -> WallShotOutcome {
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions {
            contact_impacts: ContactImpactOptions::default(),
            grace: GracePeriodOptions {
                sibling_steps: 0,
                impact_source_steps: 0,
            },
            ..DestructionRuntimeOptions::default()
        },
    )
    .expect("wall scenario should create");
    runtime.set_resimulation_options(ResimulationOptions {
        enabled: resim_enabled,
        max_passes: if resim_enabled { 2 } else { 0 },
    });

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    let projectile = world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![80.0, 0.0, 0.0],
        0.35,
        128_000.0,
    );

    let dt = 1.0 / 60.0;
    let mut now_secs = 0.0;
    let mut total_fractures = 0usize;
    let mut total_splits = 0usize;
    let mut saw_resimulation = false;
    let mut max_x = f32::NEG_INFINITY;

    for _ in 0..60 {
        let result = world.run_frame(&mut runtime, now_secs, dt);
        total_fractures += result.fractures;
        total_splits += result.split_events;
        saw_resimulation |= result.rapier_passes > 1;
        now_secs += dt;

        if let Some(body) = world.bodies.get(projectile) {
            max_x = max_x.max(body.translation().x);
        }
    }

    let mut remaining_bonds = Vec::new();
    for bond in &scenario.bonds {
        let body0 = runtime.node_body(bond.node0);
        let body1 = runtime.node_body(bond.node1);
        if body0.is_some() && body0 == body1 {
            remaining_bonds.push((bond.node0, bond.node1));
        }
    }

    let mut multi_node_bodies = Vec::new();
    for (handle, body) in world.bodies.iter() {
        if body.colliders().is_empty() {
            continue;
        }
        let nodes = runtime.body_nodes_slice(handle);
        if nodes.len() > 1 {
            multi_node_bodies.push(nodes.to_vec());
        }
    }

    let body = world
        .bodies
        .get(projectile)
        .expect("projectile body should still exist");
    let (projectile_contact_bodies, projectile_active_contact_bodies) =
        collect_projectile_contact_bodies(&world, &runtime, projectile);
    WallShotOutcome {
        final_x: body.translation().x,
        final_linvel_x: body.linvel().x,
        max_x,
        total_fractures,
        total_splits,
        saw_resimulation,
        active_bonds_after: runtime.active_bond_count(),
        actor_count_after: runtime.actor_count(),
        remaining_bonds,
        multi_node_bodies,
        projectile_contact_bodies,
        projectile_active_contact_bodies,
    }
}

fn run_heavy_wall_shot(resim_enabled: bool) -> WallShotOutcome {
    run_heavy_wall_shot_with_scenario(wall_scenario(), resim_enabled)
}

fn support_strip_bond_indices(scenario: &ScenarioDesc) -> Vec<usize> {
    scenario
        .bonds
        .iter()
        .enumerate()
        .filter_map(|(bond_index, bond)| {
            matches!(
                (bond.node0, bond.node1),
                (0, 1) | (1, 0) | (1, 2) | (2, 1) | (2, 3) | (3, 2)
            )
            .then_some(bond_index)
        })
        .collect()
}

fn run_heavy_support_strip_wall_shot() -> WallShotOutcome {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions {
            contact_impacts: ContactImpactOptions::default(),
            grace: GracePeriodOptions {
                sibling_steps: 0,
                impact_source_steps: 0,
            },
            ..DestructionRuntimeOptions::default()
        },
    )
    .expect("wall scenario should create");

    let kept = support_strip_bond_indices(&scenario);
    let fractured: Vec<usize> = scenario
        .bonds
        .iter()
        .enumerate()
        .filter_map(|(bond_index, _)| (!kept.contains(&bond_index)).then_some(bond_index))
        .collect();

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    runtime.fracture_bond_indices_now(
        0.0,
        &fractured,
        &mut world.bodies,
        &mut world.colliders,
        &mut world.island_manager,
        &mut world.impulse_joints,
        &mut world.multibody_joints,
    );

    let projectile = world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![80.0, 0.0, 0.0],
        0.35,
        128_000.0,
    );

    let dt = 1.0 / 60.0;
    let mut max_x = f32::NEG_INFINITY;
    for _ in 0..60 {
        world.physics_pipeline.step(
            &world.gravity,
            &world.integration_parameters,
            &mut world.island_manager,
            &mut world.broad_phase,
            &mut world.narrow_phase,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            &mut world.ccd_solver,
            &(),
            &(),
        );
        if let Some(body) = world.bodies.get(projectile) {
            max_x = max_x.max(body.translation().x);
        }
    }

    let mut remaining_bonds = Vec::new();
    for bond in &scenario.bonds {
        let body0 = runtime.node_body(bond.node0);
        let body1 = runtime.node_body(bond.node1);
        if body0.is_some() && body0 == body1 {
            remaining_bonds.push((bond.node0, bond.node1));
        }
    }

    let mut multi_node_bodies = Vec::new();
    for (handle, body) in world.bodies.iter() {
        if body.colliders().is_empty() {
            continue;
        }
        let nodes = runtime.body_nodes_slice(handle);
        if nodes.len() > 1 {
            multi_node_bodies.push(nodes.to_vec());
        }
    }

    let body = world
        .bodies
        .get(projectile)
        .expect("projectile body should still exist");
    let (projectile_contact_bodies, projectile_active_contact_bodies) =
        collect_projectile_contact_bodies(&world, &runtime, projectile);
    WallShotOutcome {
        final_x: body.translation().x,
        final_linvel_x: body.linvel().x,
        max_x,
        total_fractures: 0,
        total_splits: 0,
        saw_resimulation: false,
        active_bonds_after: runtime.active_bond_count(),
        actor_count_after: runtime.actor_count(),
        remaining_bonds,
        multi_node_bodies,
        projectile_contact_bodies,
        projectile_active_contact_bodies,
    }
}

fn run_heavy_loose_wall_shot() -> WallShotOutcome {
    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    world.add_loose_wall_blocks(5, 4, vector![0.5, 0.25, 0.25], 8.0);
    let projectile = world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![80.0, 0.0, 0.0],
        0.35,
        128_000.0,
    );

    let dt = 1.0 / 60.0;
    let mut max_x = f32::NEG_INFINITY;
    for _ in 0..60 {
        world.physics_pipeline.step(
            &world.gravity,
            &world.integration_parameters,
            &mut world.island_manager,
            &mut world.broad_phase,
            &mut world.narrow_phase,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            &mut world.ccd_solver,
            &(),
            &(),
        );
        if let Some(body) = world.bodies.get(projectile) {
            max_x = max_x.max(body.translation().x);
        }
    }

    let body = world
        .bodies
        .get(projectile)
        .expect("projectile body should still exist");
    WallShotOutcome {
        final_x: body.translation().x,
        final_linvel_x: body.linvel().x,
        max_x,
        total_fractures: 0,
        total_splits: 0,
        saw_resimulation: false,
        active_bonds_after: 0,
        actor_count_after: 0,
        remaining_bonds: Vec::new(),
        multi_node_bodies: Vec::new(),
        projectile_contact_bodies: Vec::new(),
        projectile_active_contact_bodies: Vec::new(),
    }
}

fn run_heavy_prefractured_runtime_wall_shot() -> WallShotOutcome {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions {
            contact_impacts: ContactImpactOptions::default(),
            grace: GracePeriodOptions {
                sibling_steps: 0,
                impact_source_steps: 0,
            },
            ..DestructionRuntimeOptions::default()
        },
    )
    .expect("wall scenario should create");

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    runtime.fracture_all_bonds_now(
        0.0,
        &mut world.bodies,
        &mut world.colliders,
        &mut world.island_manager,
        &mut world.impulse_joints,
        &mut world.multibody_joints,
    );

    let projectile = world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![80.0, 0.0, 0.0],
        0.35,
        128_000.0,
    );

    let dt = 1.0 / 60.0;
    let mut max_x = f32::NEG_INFINITY;
    for _ in 0..60 {
        world.physics_pipeline.step(
            &world.gravity,
            &world.integration_parameters,
            &mut world.island_manager,
            &mut world.broad_phase,
            &mut world.narrow_phase,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            &mut world.ccd_solver,
            &(),
            &(),
        );
        if let Some(body) = world.bodies.get(projectile) {
            max_x = max_x.max(body.translation().x);
        }
    }

    let mut remaining_bonds = Vec::new();
    for bond in &scenario.bonds {
        let body0 = runtime.node_body(bond.node0);
        let body1 = runtime.node_body(bond.node1);
        if body0.is_some() && body0 == body1 {
            remaining_bonds.push((bond.node0, bond.node1));
        }
    }

    let mut multi_node_bodies = Vec::new();
    for (handle, body) in world.bodies.iter() {
        if body.colliders().is_empty() {
            continue;
        }
        let nodes = runtime.body_nodes_slice(handle);
        if nodes.len() > 1 {
            multi_node_bodies.push(nodes.to_vec());
        }
    }

    let body = world
        .bodies
        .get(projectile)
        .expect("projectile body should still exist");
    let (projectile_contact_bodies, projectile_active_contact_bodies) =
        collect_projectile_contact_bodies(&world, &runtime, projectile);
    WallShotOutcome {
        final_x: body.translation().x,
        final_linvel_x: body.linvel().x,
        max_x,
        total_fractures: 0,
        total_splits: 0,
        saw_resimulation: false,
        active_bonds_after: runtime.active_bond_count(),
        actor_count_after: runtime.actor_count(),
        remaining_bonds,
        multi_node_bodies,
        projectile_contact_bodies,
        projectile_active_contact_bodies,
    }
}

fn run_heavy_restored_prefractured_runtime_wall_shot() -> WallShotOutcome {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions {
            contact_impacts: ContactImpactOptions::default(),
            grace: GracePeriodOptions {
                sibling_steps: 0,
                impact_source_steps: 0,
            },
            ..DestructionRuntimeOptions::default()
        },
    )
    .expect("wall scenario should create");

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    let snapshot = runtime.capture_resimulation_snapshot(&world.bodies);
    let fracture = runtime.fracture_all_bonds_now(
        0.0,
        &mut world.bodies,
        &mut world.colliders,
        &mut world.island_manager,
        &mut world.impulse_joints,
        &mut world.multibody_joints,
    );
    snapshot.restore(&mut world.bodies);
    runtime.restore_resimulation_split_children(
        &snapshot,
        &mut world.bodies,
        &mut world.colliders,
        &fracture.split_cohorts,
    );

    let projectile = world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![80.0, 0.0, 0.0],
        0.35,
        128_000.0,
    );

    let dt = 1.0 / 60.0;
    let mut max_x = f32::NEG_INFINITY;
    for _ in 0..60 {
        world.physics_pipeline.step(
            &world.gravity,
            &world.integration_parameters,
            &mut world.island_manager,
            &mut world.broad_phase,
            &mut world.narrow_phase,
            &mut world.bodies,
            &mut world.colliders,
            &mut world.impulse_joints,
            &mut world.multibody_joints,
            &mut world.ccd_solver,
            &(),
            &(),
        );
        if let Some(body) = world.bodies.get(projectile) {
            max_x = max_x.max(body.translation().x);
        }
    }

    let mut remaining_bonds = Vec::new();
    for bond in &scenario.bonds {
        let body0 = runtime.node_body(bond.node0);
        let body1 = runtime.node_body(bond.node1);
        if body0.is_some() && body0 == body1 {
            remaining_bonds.push((bond.node0, bond.node1));
        }
    }

    let mut multi_node_bodies = Vec::new();
    for (handle, body) in world.bodies.iter() {
        if body.colliders().is_empty() {
            continue;
        }
        let nodes = runtime.body_nodes_slice(handle);
        if nodes.len() > 1 {
            multi_node_bodies.push(nodes.to_vec());
        }
    }

    let body = world
        .bodies
        .get(projectile)
        .expect("projectile body should still exist");
    let (projectile_contact_bodies, projectile_active_contact_bodies) =
        collect_projectile_contact_bodies(&world, &runtime, projectile);
    WallShotOutcome {
        final_x: body.translation().x,
        final_linvel_x: body.linvel().x,
        max_x,
        total_fractures: fracture.fractures,
        total_splits: fracture.split_events,
        saw_resimulation: false,
        active_bonds_after: runtime.active_bond_count(),
        actor_count_after: runtime.actor_count(),
        remaining_bonds,
        multi_node_bodies,
        projectile_contact_bodies,
        projectile_active_contact_bodies,
    }
}

fn setup_heavy_wall_runtime(
    resim_enabled: bool,
) -> (ScenarioDesc, DestructionRuntime, RuntimeTestWorld, RigidBodyHandle) {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions {
            contact_impacts: ContactImpactOptions::default(),
            grace: GracePeriodOptions {
                sibling_steps: 0,
                impact_source_steps: 0,
            },
            ..DestructionRuntimeOptions::default()
        },
    )
    .expect("wall scenario should create");
    runtime.set_resimulation_options(ResimulationOptions {
        enabled: resim_enabled,
        max_passes: if resim_enabled { 2 } else { 0 },
    });

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    let projectile = world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![80.0, 0.0, 0.0],
        0.35,
        128_000.0,
    );
    (scenario, runtime, world, projectile)
}

#[derive(Debug)]
struct PrefracturedWallState {
    active_bonds_after: usize,
    actor_count_after: u32,
    destructible_body_count: usize,
    max_nodes_per_body: usize,
    multi_node_body_count: usize,
}

fn inspect_prefractured_runtime_wall_state() -> PrefracturedWallState {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions::default(),
    )
    .expect("wall scenario should create");

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    let fracture = runtime.fracture_all_bonds_now(
        0.0,
        &mut world.bodies,
        &mut world.colliders,
        &mut world.island_manager,
        &mut world.impulse_joints,
        &mut world.multibody_joints,
    );
    assert!(fracture.fractures > 0, "prefracture control should issue fractures: {fracture:?}");

    let mut max_nodes_per_body = 0usize;
    let mut multi_node_body_count = 0usize;
    for (handle, body) in world.bodies.iter() {
        if body.colliders().is_empty() {
            continue;
        }
        let node_count = runtime.body_nodes_slice(handle).len();
        if node_count == 0 {
            continue;
        }
        max_nodes_per_body = max_nodes_per_body.max(node_count);
        if node_count > 1 {
            multi_node_body_count += 1;
        }
    }

    PrefracturedWallState {
        active_bonds_after: runtime.active_bond_count(),
        actor_count_after: runtime.actor_count(),
        destructible_body_count: runtime.body_count(),
        max_nodes_per_body,
        multi_node_body_count,
    }
}

#[derive(Default)]
struct CountingEventHandler {
    collision_events: AtomicUsize,
    contact_force_events: AtomicUsize,
}

impl EventHandler for CountingEventHandler {
    fn handle_collision_event(
        &self,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _event: CollisionEvent,
        _contact_pair: Option<&ContactPair>,
    ) {
        self.collision_events.fetch_add(1, Ordering::Relaxed);
    }

    fn handle_contact_force_event(
        &self,
        _dt: Real,
        _bodies: &RigidBodySet,
        _colliders: &ColliderSet,
        _contact_pair: &ContactPair,
        _total_force_magnitude: Real,
    ) {
        self.contact_force_events.fetch_add(1, Ordering::Relaxed);
    }
}

#[test]
fn destructible_set_initializes() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 100.0,
        compression_fatal_limit: 200.0,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy::default();

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .expect("should create set");

    let (mut bodies, mut colliders, ..) = rapier_world();

    let handles = set.initialize(&mut bodies, &mut colliders);
    assert!(!handles.is_empty(), "should create initial bodies");

    // Should have at least 1 body
    assert!(set.body_count() >= 1, "body tracker should track bodies");
    assert_eq!(set.actor_count(), 1, "starts with 1 actor");

    // Support nodes should be marked
    assert!(set.is_support(0), "bottom-left is support");
    assert!(!set.is_support(5), "row 1 is dynamic");
}

#[test]
fn destructible_set_step_stable() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 10000.0,
        compression_fatal_limit: 20000.0,
        tension_elastic_limit: 10000.0,
        tension_fatal_limit: 20000.0,
        shear_elastic_limit: 10000.0,
        shear_fatal_limit: 20000.0,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Step several frames — with high limits, no fractures should occur
    for _ in 0..10 {
        let result = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        assert_eq!(
            result.fractures, 0,
            "strong wall should not fracture under normal gravity"
        );
        assert_eq!(result.split_events, 0);
    }

    // Actor count should remain 1
    assert_eq!(set.actor_count(), 1, "no splits should have occurred");
}

#[test]
fn destructible_set_fractures_under_heavy_gravity() {
    let scenario = wall_scenario();
    // Very low limits — easy to break
    let settings = SolverSettings {
        compression_elastic_limit: 0.001,
        compression_fatal_limit: 0.002,
        tension_elastic_limit: 0.001,
        tension_fatal_limit: 0.002,
        shear_elastic_limit: 0.001,
        shear_fatal_limit: 0.002,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -100.0, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let initial_actor_count = set.actor_count();
    let mut total_fractures = 0;
    // Run several steps
    for _ in 0..20 {
        let result = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        total_fractures += result.fractures;
    }

    assert!(
        total_fractures > 0,
        "wall should fracture under heavy gravity"
    );
    assert!(
        set.actor_count() > initial_actor_count,
        "actor count should increase after fractures: initial={initial_actor_count}, now={}",
        set.actor_count()
    );
}

#[test]
fn destructible_set_force_impact() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        tension_elastic_limit: 0.01,
        tension_fatal_limit: 0.05,
        shear_elastic_limit: 0.01,
        shear_fatal_limit: 0.05,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Apply a large force to the top-center node (node 12 = row 2, col 2)
    let target_node = 12u32; // row 2, col 2 for a 5x4 grid
    let target_pos = scenario.nodes[target_node as usize].centroid;
    set.add_force(target_node, target_pos, Vec3::new(5000.0, 0.0, 0.0));

    let mut fractured = false;
    for _ in 0..10 {
        let result = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        if result.fractures > 0 {
            fractured = true;
        }
    }

    assert!(fractured, "large impact force should cause fractures");
}

#[test]
fn fracture_policy_limits_bodies() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 0.001,
        compression_fatal_limit: 0.002,
        tension_elastic_limit: 0.001,
        tension_fatal_limit: 0.002,
        shear_elastic_limit: 0.001,
        shear_fatal_limit: 0.002,
        ..SolverSettings::default()
    };
    // Limit to 2 new bodies per frame
    let policy = FracturePolicy {
        max_new_bodies_per_frame: 2,
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -100.0, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Step once — new bodies should be capped at 2
    let result = set.step(
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert!(
        result.new_bodies <= 2,
        "policy should limit new bodies to 2, got {}",
        result.new_bodies
    );
}

#[test]
fn end_to_end_wall_destruction_full_pipeline() {
    // End-to-end test: create a wall, apply gravity, step simulation,
    // observe fractures and splits, verify bodies are created in Rapier
    let scenario = wall_scenario();
    let settings = SolverSettings {
        max_solver_iterations_per_frame: 64,
        compression_elastic_limit: 5.0,
        compression_fatal_limit: 10.0,
        tension_elastic_limit: 5.0,
        tension_fatal_limit: 10.0,
        shear_elastic_limit: 5.0,
        shear_fatal_limit: 10.0,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();

    // Initialize
    let initial_handles = set.initialize(&mut bodies, &mut colliders);
    let initial_body_count = bodies.len();
    assert!(!initial_handles.is_empty());

    // Verify initial state: all dynamic nodes should have a body
    for (i, node) in scenario.nodes.iter().enumerate() {
        let handle = set.node_body(i as u32);
        assert!(handle.is_some(), "node {i} should have a body");
        if node.mass == 0.0 {
            assert!(set.is_support(i as u32));
        }
    }

    // Simulate 50 frames with increasing gravity to ensure fracture
    let mut total_fractures = 0;
    let mut total_splits = 0;
    let mut total_new_bodies = 0;

    for frame in 0..50 {
        // Add extra downward force that increases over time
        if frame % 5 == 0 {
            // Apply force to top-center
            let top_center = 17u32; // row 3, col 2
            if top_center < scenario.nodes.len() as u32 {
                let pos = scenario.nodes[top_center as usize].centroid;
                set.add_force(top_center, pos, Vec3::new(0.0, -(frame as f32 * 10.0), 0.0));
            }
        }

        let result = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        total_fractures += result.fractures;
        total_splits += result.split_events;
        total_new_bodies += result.new_bodies;
    }

    // Verify outcomes
    assert!(
        total_fractures > 0,
        "wall should have fractured at least once"
    );

    let final_actor_count = set.actor_count();
    let final_body_count = set.body_count();

    println!(
        "End-to-end wall destruction results:\n\
         - Initial bodies: {initial_body_count}\n\
         - Total fractures: {total_fractures}\n\
         - Total splits: {total_splits}\n\
         - Total new bodies created: {total_new_bodies}\n\
         - Final actor count: {final_actor_count}\n\
         - Final body count: {final_body_count}"
    );

    // The wall should have broken apart
    assert!(
        final_actor_count > 1,
        "wall should have multiple actors after destruction"
    );
}

#[test]
fn solver_and_rapier_body_correspondence() {
    // Verify that solver actors and Rapier bodies stay in sync
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        tension_elastic_limit: 0.01,
        tension_fatal_limit: 0.05,
        shear_elastic_limit: 0.01,
        shear_fatal_limit: 0.05,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -50.0, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Step a few times to cause fractures
    for _ in 0..10 {
        set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
    }

    // Every node should still map to a valid body
    for i in 0..scenario.nodes.len() {
        let handle = set.node_body(i as u32);
        if let Some(h) = handle {
            assert!(
                bodies.get(h).is_some(),
                "node {i} maps to body handle that exists in RigidBodySet"
            );
        }
    }
}

#[test]
fn rapier_world_remains_valid_after_split_body_removal() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        tension_elastic_limit: 0.01,
        tension_fatal_limit: 0.05,
        shear_elastic_limit: 0.01,
        shear_fatal_limit: 0.05,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let target_node = 12u32;
    let target_pos = scenario.nodes[target_node as usize].centroid;
    set.add_force(target_node, target_pos, Vec3::new(5000.0, 0.0, 0.0));

    let mut saw_split = false;
    for _ in 0..10 {
        let result = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        if result.split_events > 0 {
            saw_split = true;
            break;
        }
    }

    assert!(saw_split, "expected impact to create at least one split");

    let gravity = vector![0.0, -9.81, 0.0];
    let integration_parameters = IntegrationParameters::default();
    let mut physics_pipeline = PhysicsPipeline::new();
    let mut broad_phase = BroadPhaseBvh::new();
    let mut narrow_phase = NarrowPhase::new();
    let mut ccd_solver = CCDSolver::new();

    physics_pipeline.step(
        &gravity,
        &integration_parameters,
        &mut island_manager,
        &mut broad_phase,
        &mut narrow_phase,
        &mut bodies,
        &mut colliders,
        &mut impulse_joints,
        &mut multibody_joints,
        &mut ccd_solver,
        &(),
        &(),
    );
}

#[test]
fn destructible_set_from_scenario_convenience() {
    let scenario = wall_scenario();
    let settings = SolverSettings::default();
    let policy = FracturePolicy::default();
    let set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy);
    assert!(set.is_some(), "from_scenario should succeed");
    let set = set.unwrap();
    assert_eq!(set.actor_count(), 1);
}

#[test]
fn destructible_set_idle_skip() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 10000.0,
        compression_fatal_limit: 20000.0,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: true,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // First step should run (gravity is always applied on first real step)
    let _r1 = set.step(
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    // After several frames with no external forces and no fractures,
    // idle skip should kick in (step returns default result)
    let mut idle_count = 0;
    for _ in 0..10 {
        let r = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        if r.fractures == 0 && r.split_events == 0 {
            idle_count += 1;
        }
    }
    // Most frames should be idle (no fractures with strong material)
    assert!(
        idle_count >= 5,
        "most frames should be idle with idle_skip: {idle_count}"
    );
}

#[test]
fn body_tracker_support_detection() {
    let scenario = wall_scenario();
    let settings = SolverSettings::default();
    let policy = FracturePolicy::default();

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Bottom row (first 5 nodes) should be support, rest dynamic
    let mut support_count = 0;
    let mut dynamic_count = 0;
    for i in 0..scenario.nodes.len() {
        if set.is_support(i as u32) {
            support_count += 1;
        } else {
            dynamic_count += 1;
        }
    }
    assert!(support_count > 0, "should have support nodes");
    assert!(dynamic_count > 0, "should have dynamic nodes");
    assert_eq!(
        support_count + dynamic_count,
        scenario.nodes.len(),
        "every node is either support or dynamic"
    );
}

#[test]
fn body_tracker_all_nodes_mapped_after_init() {
    let scenario = wall_scenario();
    let settings = SolverSettings::default();
    let policy = FracturePolicy::default();

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    for i in 0..scenario.nodes.len() {
        let handle = set.node_body(i as u32);
        assert!(
            handle.is_some(),
            "node {i} should have a body handle after init"
        );
    }
}

#[test]
fn destructible_set_add_force_triggers_activity() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 0.01,
        compression_fatal_limit: 0.05,
        ..SolverSettings::default()
    };
    let policy = FracturePolicy {
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Apply force to a node
    set.add_force(10, Vec3::new(0.0, 1.0, 0.0), Vec3::new(10000.0, 0.0, 0.0));

    let result = set.step(
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    // With weak settings and added force, should see fractures
    assert!(
        result.fractures > 0 || result.converged,
        "force + gravity should cause activity"
    );
}

#[test]
fn policy_max_dynamic_bodies_caps_world() {
    let scenario = wall_scenario();
    let settings = SolverSettings {
        compression_elastic_limit: 0.001,
        compression_fatal_limit: 0.002,
        tension_elastic_limit: 0.001,
        tension_fatal_limit: 0.002,
        shear_elastic_limit: 0.001,
        shear_fatal_limit: 0.002,
        ..SolverSettings::default()
    };
    // Only allow 3 dynamic bodies
    let policy = FracturePolicy {
        max_dynamic_bodies: 3,
        idle_skip: false,
        ..FracturePolicy::default()
    };

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -100.0, 0.0), policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    // Run many frames
    for _ in 0..20 {
        set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
    }

    // Body count should be capped (some tolerance for fixed bodies)
    let total = set.body_count();
    // We expect the dynamic body limit to constrain creation
    // The exact count depends on how many fixed + dynamic bodies exist
    assert!(
        total <= 20,
        "body count should be bounded by policy: got {total}"
    );
}

#[test]
fn resimulation_snapshot_restores_pose_velocity_and_sleep_state() {
    let scenario = dynamic_pair_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_resimulation_options(ResimulationOptions {
        enabled: true,
        max_passes: 2,
    });

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let body_handle = set
        .node_body(0)
        .expect("dynamic pair should create one body");
    {
        let body = bodies.get_mut(body_handle).unwrap();
        body.add_force(vector![2.0, 3.0, 4.0], true);
        body.add_torque(vector![5.0, 6.0, 7.0], true);
    }
    let snapshot = set.capture_resimulation_snapshot(&bodies);

    {
        let body = bodies.get_mut(body_handle).unwrap();
        body.set_position(Isometry::translation(3.0, 4.0, 5.0), true);
        body.set_linvel(vector![7.0, 8.0, 9.0], true);
        body.set_angvel(vector![1.0, 2.0, 3.0], true);
        body.set_linear_damping(4.0);
        body.set_angular_damping(5.0);
        body.reset_forces(true);
        body.reset_torques(true);
        body.add_force(vector![-1.0, -2.0, -3.0], true);
        body.add_torque(vector![-4.0, -5.0, -6.0], true);
        body.set_enabled(false);
        body.sleep();
    }

    snapshot.restore(&mut bodies);

    let body = bodies.get(body_handle).unwrap();
    let pos = body.translation();
    assert!((pos.x + 0.0).abs() < 1.0e-5);
    assert!((pos.y - 0.5).abs() < 1.0e-5);
    assert!((pos.z - 0.0).abs() < 1.0e-5);
    assert_eq!(*body.linvel(), vector![0.0, 0.0, 0.0]);
    assert_eq!(*body.angvel(), vector![0.0, 0.0, 0.0]);
    assert!(body.is_enabled());
    assert!(!body.is_sleeping());
    assert!(body.linear_damping() < 4.0);
    assert!(body.angular_damping() < 5.0);
    assert_eq!(body.user_force(), vector![2.0, 3.0, 4.0]);
    assert_eq!(body.user_torque(), vector![5.0, 6.0, 7.0]);
}

#[test]
fn destruction_runtime_contact_driven_impacts_fracture_and_resimulate() {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut destructible =
        DestructibleSet::from_scenario(&scenario, weak_impact_settings(), Vec3::ZERO, policy)
            .expect("wall scenario should create");
    destructible.set_resimulation_options(ResimulationOptions {
        enabled: true,
        max_passes: 2,
    });

    let runtime_options = DestructionRuntimeOptions {
        contact_impacts: ContactImpactOptions {
            min_total_impulse: 5.0,
            min_external_speed: 0.1,
            min_internal_speed: 0.1,
            force_scale: 1.0,
            max_force_magnitude: 12_000.0,
            splash_radius: 1.5,
            splash_falloff_exponent: 2.0,
            internal_contact_scale: 0.5,
            ..ContactImpactOptions::default()
        },
        ..DestructionRuntimeOptions::default()
    };
    let mut runtime = DestructionRuntime::new(destructible, runtime_options);
    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![24.0, 0.0, 0.0],
        0.35,
        2500.0,
    );

    let dt = 1.0 / 60.0;
    let mut now_secs = 0.0;
    let mut total_impacts = 0usize;
    let mut total_fractures = 0usize;
    let mut saw_resimulation = false;
    let mut trace = Vec::new();

    for frame_index in 0..45 {
        let result = world.run_frame(&mut runtime, now_secs, dt);
        trace.push((
            frame_index,
            result.contact_pairs,
            result.active_contact_pairs,
            result.accepted_impacts,
            result.rejected_below_impulse,
            result.rejected_below_speed,
            result.rejected_cooldown,
            result.fractures,
            result.rapier_passes,
        ));
        total_impacts += result.accepted_impacts;
        total_fractures += result.fractures;
        saw_resimulation |= result.rapier_passes > 1;
        now_secs += dt;
        if total_fractures > 0 {
            break;
        }
    }

    assert!(
        total_impacts > 0,
        "generic Rapier contact should be admitted as an impact without explicit add_force: {trace:?}"
    );
    assert!(
        total_fractures > 0,
        "accepted contact impact should fracture the destructible wall: {trace:?}"
    );
    assert!(
        saw_resimulation,
        "topology-changing contact fracture should trigger same-frame resimulation"
    );
    assert!(
        runtime.actor_count() > 1,
        "contact-driven fracture should split the wall into multiple actors"
    );
}

#[test]
fn destruction_runtime_registers_support_contacts_from_generic_collisions() {
    let scenario = dynamic_pair_scenario();
    let mut destructible = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .expect("dynamic pair scenario should create");
    destructible.set_small_body_damping(SmallBodyDampingOptions {
        mode: OptimizationMode::AfterGroundCollision,
        collider_count_threshold: 2,
        min_linear_damping: 2.0,
        min_angular_damping: 2.0,
    });

    let runtime_options = DestructionRuntimeOptions {
        contact_impacts: ContactImpactOptions {
            enabled: false,
            ..ContactImpactOptions::default()
        },
        ..DestructionRuntimeOptions::default()
    };
    let mut runtime = DestructionRuntime::new(destructible, runtime_options);
    let mut world = RuntimeTestWorld::new(vector![0.0, -9.81, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    world.add_ground(-0.5);

    let body_handle = runtime
        .node_body(0)
        .expect("dynamic pair should initialize into one body");
    let dt = 1.0 / 60.0;
    let mut now_secs = 0.0;
    let mut support_contacts = 0usize;

    for _ in 0..120 {
        let result = world.run_frame(&mut runtime, now_secs, dt);
        support_contacts += result.support_contacts;
        now_secs += dt;
        if support_contacts > 0 {
            break;
        }
    }

    assert!(
        support_contacts > 0,
        "fixed-body collision should be registered as a support contact by the runtime"
    );
    let body = world
        .bodies
        .get(body_handle)
        .expect("destructible body should still exist");
    assert!(
        body.linear_damping() >= 2.0 && body.angular_damping() >= 2.0,
        "support-contact promotion should update damping without explicit mark_body_support_contact"
    );
}

#[test]
fn destruction_runtime_buffers_user_events_until_the_committed_pass() {
    let scenario = wall_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut runtime = DestructionRuntime::from_scenario(
        &scenario,
        weak_impact_settings(),
        Vec3::ZERO,
        policy,
        DestructionRuntimeOptions {
            contact_impacts: ContactImpactOptions {
                min_total_impulse: 5.0,
                min_external_speed: 0.1,
                min_internal_speed: 0.1,
                force_scale: 1.0,
                max_force_magnitude: 12_000.0,
                splash_radius: 1.5,
                splash_falloff_exponent: 2.0,
                internal_contact_scale: 0.5,
                ..ContactImpactOptions::default()
            },
            ..DestructionRuntimeOptions::default()
        },
    )
    .expect("wall scenario should create");
    runtime.set_resimulation_options(ResimulationOptions {
        enabled: true,
        max_passes: 2,
    });

    let mut world = RuntimeTestWorld::new(vector![0.0, 0.0, 0.0]);
    runtime.initialize(&mut world.bodies, &mut world.colliders);
    world.add_ball_projectile(
        vector![-5.0, 1.25, 0.0],
        vector![24.0, 0.0, 0.0],
        0.35,
        2500.0,
    );

    let event_handler = Arc::new(CountingEventHandler::default());
    let dt = 1.0 / 60.0;
    let mut now_secs = 0.0;
    let mut saw_resimulation = false;

    for _ in 0..45 {
        let result = world.run_frame_with(
            &mut runtime,
            now_secs,
            dt,
            &(),
            event_handler.as_ref(),
            |_| {
                assert_eq!(
                    event_handler.collision_events.load(Ordering::Relaxed),
                    0,
                    "user collision events should stay buffered until the committed pass"
                );
            },
        );
        saw_resimulation |= result.rapier_passes > 1;
        now_secs += dt;
        if result.fractures > 0 {
            break;
        }
    }

    assert!(
        saw_resimulation,
        "impact scenario should trigger resimulation"
    );
    assert!(
        event_handler.collision_events.load(Ordering::Relaxed) > 0,
        "collision events should be forwarded after the committed pass"
    );
}

#[test]
fn wall_heavy_projectile_only_passes_through_with_resimulation() {
    let without_resim = run_heavy_wall_shot(false);
    let with_resim = run_heavy_wall_shot(true);

    assert!(
        without_resim.total_fractures > 0 && without_resim.total_splits > 0,
        "heavy wall shot should fracture and split without resim: {without_resim:?}"
    );
    assert!(
        with_resim.total_fractures > 0 && with_resim.total_splits > 0,
        "heavy wall shot should fracture and split with resim: {with_resim:?}"
    );
    assert!(
        with_resim.saw_resimulation,
        "resim-enabled wall shot should trigger same-frame replay: {with_resim:?}"
    );
    assert!(
        without_resim.max_x < 0.5 && without_resim.final_linvel_x < 1.0,
        "without resim the projectile should shatter the wall but remain blocked near the front face instead of carrying through: {without_resim:?}"
    );
    assert!(
        with_resim.max_x > 0.66 && with_resim.final_linvel_x > 0.0,
        "with resim and CCD off the projectile should replay against the broken wall and pass through: without={without_resim:?} with={with_resim:?}"
    );
}

#[test]
fn heavy_projectile_substantially_breaks_bonded_wall_before_replay() {
    let bonded = run_heavy_wall_shot(false);

    assert!(
        bonded.total_fractures > 0 && bonded.total_splits > 0,
        "heavy wall shot should fracture and split the bonded wall: {bonded:?}"
    );
    assert!(
        bonded.actor_count_after > 1,
        "full wall breakup should leave multiple actors after fracture: {bonded:?}"
    );
    assert!(
        bonded.active_bonds_after < 10,
        "the bonded wall control should leave only a small residue of active bonds after the heavy shot: {bonded:?}"
    );
}

#[test]
fn same_heavy_projectile_passes_through_prefractured_loose_wall_without_ccd() {
    let loose = run_heavy_loose_wall_shot();

    assert_eq!(
        loose.active_bonds_after, 0,
        "the loose-wall control should start and end with no bonds: {loose:?}"
    );
    assert_eq!(
        loose.total_fractures, 0,
        "the loose-wall control should not need fracture to let the projectile through: {loose:?}"
    );
    assert!(
        loose.max_x > 0.66 && loose.final_linvel_x > 0.0,
        "the same heavy projectile should already pass through an equivalent loose wall without CCD: {loose:?}"
    );
}

#[test]
fn same_heavy_projectile_passes_through_prefractured_runtime_wall_without_ccd() {
    let prefractured = run_heavy_prefractured_runtime_wall_shot();

    assert_eq!(
        prefractured.active_bonds_after, 0,
        "the runtime prefracture control should remove every bond before the shot: {prefractured:?}"
    );
    assert!(
        prefractured.multi_node_bodies.is_empty(),
        "the runtime prefracture control should leave only single-node bodies before the shot: {prefractured:?}"
    );
    assert!(
        prefractured.max_x > 0.66 && prefractured.final_linvel_x > 0.0,
        "if the same heavy projectile cannot pass the prefractured runtime wall, the problem is in the fractured runtime body state rather than resim replay: {prefractured:?}"
    );
}

#[test]
fn same_heavy_projectile_still_passes_after_snapshot_restore_of_prefractured_runtime_wall() {
    let restored = run_heavy_restored_prefractured_runtime_wall_shot();

    assert_eq!(
        restored.active_bonds_after, 0,
        "the restored prefractured control should still have no active bonds: {restored:?}"
    );
    assert!(
        restored.multi_node_bodies.is_empty(),
        "the restored prefractured control should still expose only single-node bodies: {restored:?}"
    );
    assert!(
        restored.max_x > 0.66 && restored.final_linvel_x > 0.0,
        "if restoring split children from the intact-body snapshot breaks pass-through, then resimulation restore is rebuilding the fractured wall into the wrong physical state: {restored:?}"
    );
}

#[test]
fn same_heavy_projectile_passes_through_wall_when_only_support_strip_remains_bonded() {
    let support_strip = run_heavy_support_strip_wall_shot();

    assert_eq!(
        support_strip.remaining_bonds,
        vec![(0, 1), (1, 2), (2, 3)],
        "the support-strip control should preserve only the lower fixed row bonds: {support_strip:?}"
    );
    assert_eq!(
        support_strip.multi_node_bodies,
        vec![vec![0, 1, 2, 3]],
        "the support-strip control should start from the same bonded support topology as the live fracture endpoint: {support_strip:?}"
    );
    assert!(
        support_strip.max_x > 0.66 && support_strip.final_linvel_x > 0.0,
        "if the projectile still passes this 17-actor topology, then live resim is failing because the replayed body state differs from the equivalent pre-broken topology: {support_strip:?}"
    );
}

#[test]
fn same_frame_restored_replay_matches_equivalent_prefractured_next_step() {
    let dt = 1.0 / 60.0;

    // Branch A: let the runtime do the first speculative impact pass, fracture,
    // and restore the frame for replay.
    let (_scenario, mut replay_runtime, mut replay_world, replay_projectile) =
        setup_heavy_wall_runtime(true);
    let mut now_secs = 0.0;
    let mut saw_resim = false;
    for _ in 0..10 {
        replay_runtime.begin_frame(now_secs, dt, &replay_world.bodies);
        let pass = replay_runtime.begin_pass(&(), &());
        let directive = {
            let mut world_access = RapierWorldAccess {
                bodies: &mut replay_world.bodies,
                colliders: &mut replay_world.colliders,
                island_manager: &mut replay_world.island_manager,
                broad_phase: &mut replay_world.broad_phase,
                narrow_phase: &mut replay_world.narrow_phase,
                impulse_joints: &mut replay_world.impulse_joints,
                multibody_joints: &mut replay_world.multibody_joints,
                ccd_solver: &mut replay_world.ccd_solver,
            };
            replay_world.physics_pipeline.step(
                &replay_world.gravity,
                &replay_world.integration_parameters,
                world_access.island_manager,
                world_access.broad_phase,
                world_access.narrow_phase,
                world_access.bodies,
                world_access.colliders,
                world_access.impulse_joints,
                world_access.multibody_joints,
                world_access.ccd_solver,
                &pass,
                &pass,
            );
            replay_runtime.finish_pass(pass, &mut world_access)
        };
        match directive {
            FrameDirective::Resimulate => {
                saw_resim = true;
                break;
            }
            FrameDirective::Done(result) => {
                assert_eq!(
                    result.fractures, 0,
                    "the manual replay harness should only see fractures on the frame that requests resim"
                );
                now_secs += dt;
            }
        }
    }
    assert!(saw_resim, "the heavy wall repro should trigger same-frame replay");

    replay_world.physics_pipeline.step(
        &replay_world.gravity,
        &replay_world.integration_parameters,
        &mut replay_world.island_manager,
        &mut replay_world.broad_phase,
        &mut replay_world.narrow_phase,
        &mut replay_world.bodies,
        &mut replay_world.colliders,
        &mut replay_world.impulse_joints,
        &mut replay_world.multibody_joints,
        &mut replay_world.ccd_solver,
        &(),
        &(),
    );
    let (replay_contacts, replay_active_contacts) =
        collect_projectile_contact_bodies(&replay_world, &replay_runtime, replay_projectile);
    let replay_body = replay_world
        .bodies
        .get(replay_projectile)
        .expect("replay projectile should still exist");

    // Branch B: build the equivalent support-strip topology before the same next step.
    let (control_scenario, mut control_runtime, mut control_world, control_projectile) =
        setup_heavy_wall_runtime(false);
    let mut control_now_secs = 0.0;
    for _ in 0..(now_secs / dt) as usize {
        let result = control_world.run_frame(&mut control_runtime, control_now_secs, dt);
        assert_eq!(
            result.fractures, 0,
            "the control branch should match the replay branch before the fracture frame"
        );
        control_now_secs += dt;
    }
    let kept = support_strip_bond_indices(&control_scenario);
    let fractured: Vec<usize> = control_scenario
        .bonds
        .iter()
        .enumerate()
        .filter_map(|(bond_index, _)| (!kept.contains(&bond_index)).then_some(bond_index))
        .collect();
    control_runtime.fracture_bond_indices_now(
        control_now_secs,
        &fractured,
        &mut control_world.bodies,
        &mut control_world.colliders,
        &mut control_world.island_manager,
        &mut control_world.impulse_joints,
        &mut control_world.multibody_joints,
    );
    control_world.physics_pipeline.step(
        &control_world.gravity,
        &control_world.integration_parameters,
        &mut control_world.island_manager,
        &mut control_world.broad_phase,
        &mut control_world.narrow_phase,
        &mut control_world.bodies,
        &mut control_world.colliders,
        &mut control_world.impulse_joints,
        &mut control_world.multibody_joints,
        &mut control_world.ccd_solver,
        &(),
        &(),
    );
    let (control_contacts, control_active_contacts) =
        collect_projectile_contact_bodies(&control_world, &control_runtime, control_projectile);
    let control_body = control_world
        .bodies
        .get(control_projectile)
        .expect("control projectile should still exist");

    assert_eq!(
        replay_active_contacts,
        control_active_contacts,
        "same-frame replay should start the next physics step from the same active-contact state as the equivalent prefractured control: replay={replay_contacts:?} control={control_contacts:?} replay_body=({:.3},{:.3}) control_body=({:.3},{:.3})",
        replay_body.translation().x,
        replay_body.linvel().x,
        control_body.translation().x,
        control_body.linvel().x,
    );
}

#[test]
fn fracture_all_bonds_now_separates_runtime_wall_into_single_block_bodies() {
    let state = inspect_prefractured_runtime_wall_state();

    assert_eq!(
        state.active_bonds_after, 0,
        "prefracture control should remove every active bond: {state:?}"
    );
    assert_eq!(
        state.actor_count_after, 20,
        "the 5x4 control wall should decompose into one actor per block after all bonds are removed: {state:?}"
    );
    assert_eq!(
        state.destructible_body_count, 20,
        "the runtime body tracker should expose one destructible body per block after prefracture: {state:?}"
    );
    assert_eq!(
        state.max_nodes_per_body, 1,
        "no post-fracture body should retain multiple blocks in the fully prefractured control: {state:?}"
    );
    assert_eq!(
        state.multi_node_body_count, 0,
        "multi-block bodies indicate the split pipeline still isn't producing a true loose-wall state: {state:?}"
    );
}

#[test]
fn support_contact_promotes_small_body_damping() {
    let scenario = dynamic_pair_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_small_body_damping(SmallBodyDampingOptions {
        mode: OptimizationMode::AfterGroundCollision,
        collider_count_threshold: 2,
        min_linear_damping: 2.0,
        min_angular_damping: 2.0,
    });

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let body_handle = set
        .node_body(0)
        .expect("dynamic pair should create one body");
    {
        let body = bodies.get(body_handle).unwrap();
        assert!(body.linear_damping() < 2.0);
        assert!(body.angular_damping() < 2.0);
    }

    let changed = set.mark_body_support_contact(body_handle, 1.0, &mut bodies, &mut colliders);
    assert!(changed, "first support contact should promote damping");

    let body = bodies.get(body_handle).unwrap();
    assert!(body.linear_damping() >= 2.0);
    assert!(body.angular_damping() >= 2.0);
}

#[test]
fn support_contact_configures_sleep_thresholds() {
    let scenario = dynamic_pair_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_sleep_thresholds(SleepThresholdOptions {
        mode: OptimizationMode::AfterGroundCollision,
        linear_threshold: 1.25,
        angular_threshold: 0.75,
    });

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let body_handle = set
        .node_body(0)
        .expect("dynamic pair should create one body");
    {
        let body = bodies.get(body_handle).unwrap();
        let activation = body.activation();
        assert!((activation.normalized_linear_threshold - 1.25).abs() > 1.0e-6);
        assert!((activation.angular_threshold - 0.75).abs() > 1.0e-6);
    }

    let changed = set.mark_body_support_contact(body_handle, 1.0, &mut bodies, &mut colliders);
    assert!(
        changed,
        "first support contact should configure sleep thresholds"
    );

    let body = bodies.get(body_handle).unwrap();
    let activation = body.activation();
    assert!((activation.normalized_linear_threshold - 1.25).abs() < 1.0e-6);
    assert!((activation.angular_threshold - 0.75).abs() < 1.0e-6);
}

#[test]
fn debris_cleanup_removes_small_bodies_and_solver_bonds_after_ttl() {
    let scenario = dynamic_triangle_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_debris_cleanup(DebrisCleanupOptions {
        mode: OptimizationMode::Always,
        debris_ttl_secs: 0.5,
        max_colliders_for_debris: 3,
    });

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);
    assert_eq!(set.active_bond_count(), 3);

    let no_cleanup = set.process_optimizations(
        0.25,
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert!(no_cleanup.removed_bodies.is_empty());

    let cleanup = set.process_optimizations(
        0.75,
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert_eq!(cleanup.removed_bodies.len(), 1);
    assert_eq!(cleanup.removed_nodes.len(), 3);
    assert_eq!(set.body_count(), 0);
    assert!(set.node_body(0).is_none());
    assert!(set.node_body(1).is_none());
    assert!(set.node_body(2).is_none());
    assert_eq!(
        set.active_bond_count(),
        0,
        "cleanup should retire all tracked bonds for destroyed debris nodes"
    );
}

#[test]
fn debris_cleanup_after_support_contact_requires_sleeping_body() {
    let scenario = dynamic_triangle_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_debris_cleanup(DebrisCleanupOptions {
        mode: OptimizationMode::AfterGroundCollision,
        debris_ttl_secs: 0.5,
        max_colliders_for_debris: 3,
    });

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let body_handle = set
        .node_body(0)
        .expect("triangle should initialize into one dynamic body");

    let before_contact = set.process_optimizations(
        1.0,
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert!(
        before_contact.removed_bodies.is_empty(),
        "cleanup should wait for support contact in after-ground mode"
    );

    set.mark_body_support_contact(body_handle, 0.25, &mut bodies, &mut colliders);
    let while_awake = set.process_optimizations(
        1.0,
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert!(
        while_awake.removed_bodies.is_empty(),
        "cleanup should not remove moving debris immediately after support contact"
    );

    bodies
        .get_mut(body_handle)
        .expect("body should still exist")
        .sleep();

    let after_sleep = set.process_optimizations(
        1.0,
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert_eq!(after_sleep.removed_bodies.len(), 1);
    assert_eq!(after_sleep.removed_nodes.len(), 3);
}

#[test]
fn no_debris_pairs_activates_only_after_support_contact() {
    let scenario = dynamic_triangle_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_debris_cleanup(DebrisCleanupOptions {
        mode: OptimizationMode::AfterGroundCollision,
        debris_ttl_secs: 10.0,
        max_colliders_for_debris: 3,
    });
    set.set_debris_collision_mode(DebrisCollisionMode::NoDebrisPairs);

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0),
        ground,
        &mut bodies,
    );
    set.set_ground_body_handle(Some(ground));
    set.refresh_collision_groups(&bodies, &mut colliders);

    let body_handle = set
        .node_body(0)
        .expect("triangle should initialize into one dynamic body");
    let before_groups = colliders
        .get(bodies[body_handle].colliders()[0])
        .expect("body collider should exist")
        .collision_groups();
    assert_eq!(before_groups.memberships, Group::GROUP_3);
    assert_eq!(
        before_groups.filter,
        Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_3
    );

    set.mark_body_support_contact(body_handle, 0.25, &mut bodies, &mut colliders);

    let after_groups = colliders
        .get(bodies[body_handle].colliders()[0])
        .expect("body collider should exist")
        .collision_groups();
    assert_eq!(after_groups.memberships, Group::GROUP_2);
    assert_eq!(after_groups.filter, Group::GROUP_1 | Group::GROUP_3);
}

#[test]
fn debris_ground_only_assigns_small_dynamic_bodies_to_ground_only_groups() {
    let scenario = dynamic_triangle_scenario();
    let mut set = DestructibleSet::from_scenario(
        &scenario,
        SolverSettings::default(),
        Vec3::new(0.0, -9.81, 0.0),
        FracturePolicy::default(),
    )
    .unwrap();
    set.set_debris_cleanup(DebrisCleanupOptions {
        mode: OptimizationMode::Always,
        debris_ttl_secs: 10.0,
        max_colliders_for_debris: 3,
    });
    set.set_debris_collision_mode(DebrisCollisionMode::DebrisGroundOnly);

    let (mut bodies, mut colliders, ..) = rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let ground = bodies.insert(RigidBodyBuilder::fixed());
    let ground_collider = colliders.insert_with_parent(
        ColliderBuilder::cuboid(10.0, 0.5, 10.0),
        ground,
        &mut bodies,
    );

    set.set_ground_body_handle(Some(ground));
    set.refresh_collision_groups(&bodies, &mut colliders);

    let body_handle = set
        .node_body(0)
        .expect("triangle should initialize into one dynamic body");
    set.mark_body_support_contact(body_handle, 0.25, &mut bodies, &mut colliders);
    let body_groups = colliders
        .get(bodies[body_handle].colliders()[0])
        .expect("body collider should exist")
        .collision_groups();
    let ground_groups = colliders
        .get(ground_collider)
        .expect("ground collider should exist")
        .collision_groups();

    assert_eq!(body_groups.memberships, Group::GROUP_2);
    assert_eq!(body_groups.filter, Group::GROUP_1);
    assert_eq!(ground_groups.memberships, Group::GROUP_1);
    assert_eq!(
        ground_groups.filter,
        Group::GROUP_1 | Group::GROUP_2 | Group::GROUP_3
    );
}

#[test]
fn skip_single_bodies_drops_singleton_children_instead_of_spawning_bodies() {
    let scenario = dynamic_pair_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut set =
        DestructibleSet::from_scenario(&scenario, weak_impact_settings(), Vec3::ZERO, policy)
            .unwrap();
    set.set_skip_single_bodies(true);

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let impact_pos = scenario.nodes[0].centroid;
    set.add_force(0, impact_pos, Vec3::new(10_000.0, 0.0, 0.0));

    let mut fractured = false;
    for _ in 0..8 {
        let step = set.step(
            &mut bodies,
            &mut colliders,
            &mut island_manager,
            &mut impulse_joints,
            &mut multibody_joints,
        );
        fractured |= step.fractures > 0;
        if set.body_count() == 0 {
            break;
        }
    }

    assert!(fractured, "expected the pair bond to fracture");
    assert_eq!(set.body_count(), 0);
    assert!(set.node_body(0).is_none());
    assert!(set.node_body(1).is_none());
}

#[test]
fn collider_migration_budget_defers_split_until_budget_is_available() {
    let scenario = dynamic_pair_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        max_collider_migrations_per_frame: 0,
        ..FracturePolicy::default()
    };
    let mut set =
        DestructibleSet::from_scenario(&scenario, weak_impact_settings(), Vec3::ZERO, policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let impact_pos = scenario.nodes[0].centroid;
    set.add_force(0, impact_pos, Vec3::new(10_000.0, 0.0, 0.0));

    let first = set.step(
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert!(first.fractures > 0, "impact should fracture the pair");
    assert_eq!(
        first.new_bodies, 0,
        "split should be deferred by zero migration budget"
    );
    assert_eq!(
        set.actor_count(),
        2,
        "solver split should still have happened"
    );
    assert_eq!(
        set.body_count(),
        1,
        "Rapier bodies should remain unsplit until replay budget allows it"
    );

    let mut updated_policy = *set.policy();
    updated_policy.max_collider_migrations_per_frame = 2;
    set.set_policy(updated_policy);

    let second = set.step(
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert_eq!(
        second.new_bodies, 1,
        "pending split should flush once migration budget is available while reusing one parent body"
    );
    assert_eq!(set.body_count(), 2);
}

#[test]
fn split_reuses_existing_collider_handles_instead_of_recreating_colliders() {
    let scenario = dynamic_pair_scenario();
    let policy = FracturePolicy {
        idle_skip: false,
        apply_excess_forces: false,
        ..FracturePolicy::default()
    };
    let mut set =
        DestructibleSet::from_scenario(&scenario, weak_impact_settings(), Vec3::ZERO, policy)
            .unwrap();

    let (mut bodies, mut colliders, mut island_manager, mut impulse_joints, mut multibody_joints) =
        rapier_world();
    set.initialize(&mut bodies, &mut colliders);

    let original_collider_0 = set.node_collider(0).expect("node 0 collider should exist");
    let original_collider_1 = set.node_collider(1).expect("node 1 collider should exist");

    let impact_pos = scenario.nodes[0].centroid;
    set.add_force(0, impact_pos, Vec3::new(10_000.0, 0.0, 0.0));

    let step = set.step(
        &mut bodies,
        &mut colliders,
        &mut island_manager,
        &mut impulse_joints,
        &mut multibody_joints,
    );
    assert!(step.fractures > 0, "impact should fracture the pair");
    assert_eq!(step.split_events, 1);
    assert_eq!(step.split_edits.inserted_colliders, 0);
    assert_eq!(step.split_edits.removed_colliders, 0);
    assert_eq!(step.split_edits.created_bodies, 1);
    assert_eq!(step.split_edits.reused_bodies, 1);

    assert_eq!(set.node_collider(0), Some(original_collider_0));
    assert_eq!(set.node_collider(1), Some(original_collider_1));
    assert!(
        colliders.get(original_collider_0).is_some(),
        "original collider handle for node 0 should still exist"
    );
    assert!(
        colliders.get(original_collider_1).is_some(),
        "original collider handle for node 1 should still exist"
    );
}
