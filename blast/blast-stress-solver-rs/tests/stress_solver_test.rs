use blast_stress_solver::*;

/// Helper: create a simple 3-node triangle topology (from the rust_stress_example).
fn triangle_nodes_and_bonds() -> (Vec<NodeDesc>, Vec<BondDesc>) {
    let nodes = vec![
        NodeDesc {
            centroid: Vec3::new(-1.0, 0.0, 0.0),
            mass: 0.0, // support
            volume: 1.0,
        },
        NodeDesc {
            centroid: Vec3::new(1.0, 0.0, 0.0),
            mass: 0.0, // support
            volume: 1.0,
        },
        NodeDesc {
            centroid: Vec3::new(0.0, 1.5, 0.0),
            mass: 15.0,
            volume: 1.0,
        },
    ];

    let bonds = vec![
        BondDesc {
            centroid: Vec3::new(-0.5, 0.75, 0.0),
            normal: Vec3::new(0.55, 0.83, 0.0),
            area: 0.6,
            node0: 0,
            node1: 2,
        },
        BondDesc {
            centroid: Vec3::new(0.5, 0.75, 0.0),
            normal: Vec3::new(-0.55, 0.83, 0.0),
            area: 0.6,
            node0: 1,
            node1: 2,
        },
        BondDesc {
            centroid: Vec3::new(0.0, 0.0, 0.0),
            normal: Vec3::new(1.0, 0.0, 0.0),
            area: 0.9,
            node0: 0,
            node1: 1,
        },
    ];

    (nodes, bonds)
}

#[test]
fn ext_solver_creates_successfully() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings::default();
    let solver = ExtStressSolver::new(&nodes, &bonds, &settings);
    assert!(solver.is_some(), "solver should be created");

    let solver = solver.unwrap();
    // The NvBlast support graph maps nodes through an internal chunk hierarchy.
    // node_count() returns graph node count which may differ from input due to
    // the root chunk. bond_count() returns the internal solver's bond count.
    assert!(solver.node_count() > 0, "should have nodes");
    assert_eq!(solver.actor_count(), 1, "starts with one actor");
}

#[test]
fn ext_solver_gravity_creates_stress() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings {
        compression_elastic_limit: 100.0,
        compression_fatal_limit: 200.0,
        ..SolverSettings::default()
    };
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    // Apply gravity — should produce stress on the bonds holding the top node
    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();

    // Solver should converge
    assert!(solver.converged(), "solver should converge for simple topology");
    assert!(solver.linear_error() < 1.0, "linear error should be small");
}

#[test]
fn ext_solver_strong_gravity_causes_overstress() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    // Set very low limits so gravity easily exceeds them
    let settings = SolverSettings {
        compression_elastic_limit: 0.001,
        compression_fatal_limit: 0.002,
        tension_elastic_limit: 0.001,
        tension_fatal_limit: 0.002,
        shear_elastic_limit: 0.001,
        shear_fatal_limit: 0.002,
        ..SolverSettings::default()
    };
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    solver.add_gravity(Vec3::new(0.0, -100.0, 0.0));
    solver.update();

    let overstressed = solver.overstressed_bond_count();
    assert!(
        overstressed > 0,
        "strong gravity should overstress at least one bond, got {overstressed}"
    );
}

#[test]
fn ext_solver_fracture_and_split() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings {
        compression_elastic_limit: 0.001,
        compression_fatal_limit: 0.002,
        tension_elastic_limit: 0.001,
        tension_fatal_limit: 0.002,
        shear_elastic_limit: 0.001,
        shear_fatal_limit: 0.002,
        ..SolverSettings::default()
    };
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    assert_eq!(solver.actor_count(), 1, "starts with 1 actor");

    // Apply overwhelming gravity
    solver.add_gravity(Vec3::new(0.0, -500.0, 0.0));
    solver.update();

    let overstressed = solver.overstressed_bond_count();
    assert!(overstressed > 0, "bonds should be overstressed");

    // Generate fracture commands
    let commands = solver.generate_fracture_commands();
    assert!(!commands.is_empty(), "should have fracture commands");

    let total_fractures: usize = commands.iter().map(|c| c.bond_fractures.len()).sum();
    assert!(total_fractures > 0, "should have bond fractures");

    // Apply fracture commands
    let events = solver.apply_fracture_commands(&commands);

    // After fracture, should have more actors (the structure split)
    let new_actor_count = solver.actor_count();
    assert!(
        new_actor_count > 1 || !events.is_empty(),
        "structure should have split: actors={new_actor_count}, events={}",
        events.len()
    );

    if !events.is_empty() {
        for event in &events {
            assert!(
                !event.children.is_empty(),
                "split event should have children"
            );
            for child in &event.children {
                assert!(!child.nodes.is_empty(), "child should have nodes");
            }
        }
    }
}

#[test]
fn ext_solver_actors_query() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings::default();
    let solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    let actors = solver.actors();
    assert_eq!(actors.len(), 1, "should have 1 actor initially");

    let actor = &actors[0];
    assert_eq!(actor.nodes.len(), 3, "actor should contain all 3 nodes");

    // All node indices should be present
    let mut sorted_nodes = actor.nodes.clone();
    sorted_nodes.sort();
    assert_eq!(sorted_nodes, vec![0, 1, 2]);
}

#[test]
fn ext_solver_excess_forces() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings::default();
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();

    let actors = solver.actors();
    let actor = &actors[0];

    // Excess forces should be available for the actor
    let result = solver.get_excess_forces(actor.actor_index, Vec3::new(0.0, 0.75, 0.0));
    // May or may not return Some depending on solver state — just ensure it doesn't crash
    if let Some((force, torque)) = result {
        // Force/torque are finite
        assert!(force.x.is_finite() && force.y.is_finite() && force.z.is_finite());
        assert!(torque.x.is_finite() && torque.y.is_finite() && torque.z.is_finite());
    }
}

#[test]
fn stress_processor_low_level() {
    let nodes = vec![
        StressNodeDesc {
            com: Vec3::new(-1.0, 0.0, 0.0),
            mass: 25.0,
            inertia: 2.5,
        },
        StressNodeDesc {
            com: Vec3::new(1.0, 0.0, 0.0),
            mass: 25.0,
            inertia: 2.5,
        },
        StressNodeDesc {
            com: Vec3::new(0.0, 1.5, 0.0),
            mass: 15.0,
            inertia: 1.5,
        },
    ];

    let bonds = vec![
        StressBondDesc {
            centroid: Vec3::new(-0.5, 0.75, 0.0),
            node0: 0,
            node1: 2,
        },
        StressBondDesc {
            centroid: Vec3::new(0.5, 0.75, 0.0),
            node0: 1,
            node1: 2,
        },
        StressBondDesc {
            centroid: Vec3::new(0.0, 0.0, 0.0),
            node0: 0,
            node1: 1,
        },
    ];

    let params = StressDataParams {
        equalize_masses: true,
        center_bonds: true,
    };

    let mut solver = StressProcessor::new(&nodes, &bonds, params).unwrap();
    assert_eq!(solver.node_count(), 3);
    assert_eq!(solver.bond_count(), 3);

    let mut impulses = vec![StressImpulse::default(); 3];
    let mut velocities = vec![StressVelocity::default(); 3];

    // Apply downward force to top node
    velocities[2] = StressVelocity {
        ang: Vec3::ZERO,
        lin: Vec3::new(0.0, -24.0, 0.0),
    };

    let solver_params = StressSolverParams {
        max_iterations: 128,
        tolerance: 1e-5,
        warm_start: true,
    };

    let (iters, error) = solver.solve(&mut impulses, &velocities, solver_params, false).unwrap();
    assert!(iters > 0, "solver should iterate");
    assert!(error.lin < 1.0, "linear error should be small");

    // Check bond stress computation
    let bond0 = solver.bond_desc(0).unwrap();
    let stress = compute_bond_stress(&bond0, &impulses[0], &nodes, 0.6);
    // At least one component should be non-zero since we applied force
    assert!(
        stress.compression > 0.0 || stress.tension > 0.0 || stress.shear > 0.0,
        "stress should be non-zero: {:?}",
        stress
    );
}

#[test]
fn bond_stress_computation() {
    let nodes = vec![
        StressNodeDesc {
            com: Vec3::new(0.0, 0.0, 0.0),
            mass: 10.0,
            inertia: 1.0,
        },
        StressNodeDesc {
            com: Vec3::new(0.0, 1.0, 0.0),
            mass: 10.0,
            inertia: 1.0,
        },
    ];

    let bond = StressBondDesc {
        centroid: Vec3::new(0.0, 0.5, 0.0),
        node0: 0,
        node1: 1,
    };

    // Pure tension: pulling nodes apart along bond normal
    let impulse = StressImpulse {
        ang: Vec3::ZERO,
        lin: Vec3::new(0.0, 100.0, 0.0),
    };

    let stress = compute_bond_stress(&bond, &impulse, &nodes, 1.0);
    assert!(stress.tension > 0.0, "should have tension");
    assert!(
        stress.compression < 0.01,
        "should have minimal compression"
    );

    // Pure compression: pushing nodes together
    let impulse_comp = StressImpulse {
        ang: Vec3::ZERO,
        lin: Vec3::new(0.0, -100.0, 0.0),
    };
    let stress_comp = compute_bond_stress(&bond, &impulse_comp, &nodes, 1.0);
    assert!(stress_comp.compression > 0.0, "should have compression");
    assert!(
        stress_comp.tension < 0.01,
        "should have minimal tension"
    );

    // Pure shear: force perpendicular to bond normal
    let impulse_shear = StressImpulse {
        ang: Vec3::ZERO,
        lin: Vec3::new(100.0, 0.0, 0.0),
    };
    let stress_shear = compute_bond_stress(&bond, &impulse_shear, &nodes, 1.0);
    assert!(stress_shear.shear > 0.0, "should have shear");
}

#[test]
fn stress_limits_failure_detection() {
    let limits = StressLimits {
        compression_elastic_limit: 100.0,
        compression_fatal_limit: 200.0,
        tension_elastic_limit: 80.0,
        tension_fatal_limit: 150.0,
        shear_elastic_limit: 60.0,
        shear_fatal_limit: 120.0,
    };

    // Below all limits
    let safe = BondStressResult {
        compression: 50.0,
        tension: 30.0,
        shear: 20.0,
    };
    assert!(limits.failure_mode(&safe).is_none());

    // Compression failure
    let comp_fail = BondStressResult {
        compression: 250.0,
        tension: 30.0,
        shear: 20.0,
    };
    assert_eq!(
        limits.failure_mode(&comp_fail),
        Some(StressFailure::Compression)
    );

    // Tension failure
    let tens_fail = BondStressResult {
        compression: 50.0,
        tension: 160.0,
        shear: 20.0,
    };
    assert_eq!(
        limits.failure_mode(&tens_fail),
        Some(StressFailure::Tension)
    );

    // Shear failure
    let shear_fail = BondStressResult {
        compression: 50.0,
        tension: 30.0,
        shear: 130.0,
    };
    assert_eq!(
        limits.failure_mode(&shear_fail),
        Some(StressFailure::Shear)
    );

    // Severity mapping
    let severity = limits.severity(&safe);
    assert!(severity.compression < 0.5, "below elastic = < 0.5");
    assert!(severity.tension < 0.5, "below elastic = < 0.5");
    assert!(severity.shear < 0.5, "below elastic = < 0.5");
}

#[test]
fn ext_solver_set_settings() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings::default();
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    // Change settings at runtime
    let new_settings = SolverSettings {
        compression_fatal_limit: 999.0,
        ..settings
    };
    solver.set_settings(&new_settings);

    // Should still work
    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();
    assert!(solver.converged());
}

#[test]
fn ext_solver_reset() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings::default();
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
    solver.update();

    // Reset should clear accumulated forces
    solver.reset();
    // Update without adding forces — should converge with zero stress
    solver.update();
    assert!(solver.converged());
}

#[test]
fn ext_solver_add_force_to_node() {
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings {
        compression_elastic_limit: 0.001,
        compression_fatal_limit: 0.002,
        tension_elastic_limit: 0.001,
        tension_fatal_limit: 0.002,
        shear_elastic_limit: 0.001,
        shear_fatal_limit: 0.002,
        ..SolverSettings::default()
    };
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    // Apply a large force to the top node
    solver.add_force(
        2,
        Vec3::new(0.0, 1.5, 0.0),
        Vec3::new(1000.0, 0.0, 0.0),
        ForceMode::Force,
    );
    solver.update();

    let overstressed = solver.overstressed_bond_count();
    assert!(overstressed > 0, "large force should overstress bonds");
}

#[test]
fn full_lifecycle_gravity_fracture_split() {
    // End-to-end: create solver, apply gravity over multiple frames,
    // detect overstress, fracture, split, verify topology changes
    let (nodes, bonds) = triangle_nodes_and_bonds();
    let settings = SolverSettings {
        max_solver_iterations_per_frame: 64,
        compression_elastic_limit: 10.0,
        compression_fatal_limit: 20.0,
        tension_elastic_limit: 10.0,
        tension_fatal_limit: 20.0,
        shear_elastic_limit: 10.0,
        shear_fatal_limit: 20.0,
        ..SolverSettings::default()
    };
    let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();

    // Simulate multiple frames of increasing gravity
    let mut total_fractures = 0;
    let mut total_splits = 0;

    for frame in 0..20 {
        let g = -9.81 * (1.0 + frame as f32 * 5.0);
        solver.add_gravity(Vec3::new(0.0, g, 0.0));
        solver.update();

        let overstressed = solver.overstressed_bond_count();
        if overstressed > 0 {
            let commands = solver.generate_fracture_commands();
            let fractures: usize = commands.iter().map(|c| c.bond_fractures.len()).sum();
            total_fractures += fractures;

            if !commands.is_empty() {
                let events = solver.apply_fracture_commands(&commands);
                total_splits += events.len();
            }
        }

        // If all bonds are broken, we're done
        if solver.actor_count() as usize >= nodes.len() {
            break;
        }
    }

    assert!(
        total_fractures > 0,
        "should have fractured at least one bond over 20 frames"
    );
    assert!(
        solver.actor_count() > 1,
        "structure should have split into multiple actors, got {}",
        solver.actor_count()
    );
}
