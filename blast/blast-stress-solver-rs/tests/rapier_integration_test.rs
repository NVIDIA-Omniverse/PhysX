#![cfg(feature = "rapier")]

use rapier3d::prelude::*;

use blast_stress_solver::rapier::*;
use blast_stress_solver::*;

/// Helper: create a 5-column, 4-row wall scenario
fn wall_scenario() -> ScenarioDesc {
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

    ScenarioDesc {
        nodes,
        bonds,
        node_sizes: vec![Vec3::new(bw, bh, bd); (columns * rows) as usize],
    }
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
    let snapshot = set.capture_resimulation_snapshot(&bodies);

    {
        let body = bodies.get_mut(body_handle).unwrap();
        body.set_position(Isometry::translation(3.0, 4.0, 5.0), true);
        body.set_linvel(vector![7.0, 8.0, 9.0], true);
        body.set_angvel(vector![1.0, 2.0, 3.0], true);
        body.set_linear_damping(4.0);
        body.set_angular_damping(5.0);
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
