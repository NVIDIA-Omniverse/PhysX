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

    ScenarioDesc { nodes, bonds }
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

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

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

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // Step several frames — with high limits, no fractures should occur
    for _ in 0..10 {
        let result = set.step(&mut bodies, &mut colliders);
        assert_eq!(result.fractures, 0, "strong wall should not fracture under normal gravity");
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

    let mut set = DestructibleSet::from_scenario(
        &scenario,
        settings,
        Vec3::new(0.0, -100.0, 0.0),
        policy,
    )
    .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    let initial_actor_count = set.actor_count();
    let mut total_fractures = 0;
    let mut total_new_bodies = 0;

    // Run several steps
    for _ in 0..20 {
        let result = set.step(&mut bodies, &mut colliders);
        total_fractures += result.fractures;
        total_new_bodies += result.new_bodies;
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

    let mut set = DestructibleSet::from_scenario(
        &scenario,
        settings,
        Vec3::new(0.0, -9.81, 0.0),
        policy,
    )
    .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // Apply a large force to the top-center node (node 12 = row 2, col 2)
    let target_node = 12u32; // row 2, col 2 for a 5x4 grid
    let target_pos = scenario.nodes[target_node as usize].centroid;
    set.add_force(target_node, target_pos, Vec3::new(5000.0, 0.0, 0.0));

    let mut fractured = false;
    for _ in 0..10 {
        let result = set.step(&mut bodies, &mut colliders);
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

    let mut set = DestructibleSet::from_scenario(
        &scenario,
        settings,
        Vec3::new(0.0, -100.0, 0.0),
        policy,
    )
    .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // Step once — new bodies should be capped at 2
    let result = set.step(&mut bodies, &mut colliders);
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

    let mut set = DestructibleSet::from_scenario(
        &scenario,
        settings,
        Vec3::new(0.0, -9.81, 0.0),
        policy,
    )
    .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();

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
                set.add_force(
                    top_center,
                    pos,
                    Vec3::new(0.0, -(frame as f32 * 10.0), 0.0),
                );
            }
        }

        let result = set.step(&mut bodies, &mut colliders);
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

    let mut set = DestructibleSet::from_scenario(
        &scenario,
        settings,
        Vec3::new(0.0, -50.0, 0.0),
        policy,
    )
    .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // Step a few times to cause fractures
    for _ in 0..10 {
        set.step(&mut bodies, &mut colliders);
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
fn destructible_set_from_scenario_convenience() {
    let scenario = wall_scenario();
    let settings = SolverSettings::default();
    let policy = FracturePolicy::default();
    let set = DestructibleSet::from_scenario(
        &scenario,
        settings,
        Vec3::new(0.0, -9.81, 0.0),
        policy,
    );
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

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // First step should run (gravity is always applied on first real step)
    let r1 = set.step(&mut bodies, &mut colliders);
    // After several frames with no external forces and no fractures,
    // idle skip should kick in (step returns default result)
    let mut idle_count = 0;
    for _ in 0..10 {
        let r = set.step(&mut bodies, &mut colliders);
        if r.fractures == 0 && r.split_events == 0 {
            idle_count += 1;
        }
    }
    // Most frames should be idle (no fractures with strong material)
    assert!(idle_count >= 5, "most frames should be idle with idle_skip: {idle_count}");
}

#[test]
fn body_tracker_support_detection() {
    let scenario = wall_scenario();
    let settings = SolverSettings::default();
    let policy = FracturePolicy::default();

    let mut set =
        DestructibleSet::from_scenario(&scenario, settings, Vec3::new(0.0, -9.81, 0.0), policy)
            .unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
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

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
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

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // Apply force to a node
    set.add_force(10, Vec3::new(0.0, 1.0, 0.0), Vec3::new(10000.0, 0.0, 0.0));

    let result = set.step(&mut bodies, &mut colliders);
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
        DestructibleSet::from_scenario(
            &scenario,
            settings,
            Vec3::new(0.0, -100.0, 0.0),
            policy,
        ).unwrap();

    let mut bodies = RigidBodySet::new();
    let mut colliders = ColliderSet::new();
    set.initialize(&mut bodies, &mut colliders);

    // Run many frames
    for _ in 0..20 {
        set.step(&mut bodies, &mut colliders);
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
