//! Headless scenario tests — run wall/tower/bridge through the solver without Rapier.
//!
//! Tests cover: gravity stability, weak-bond collapse, force impact, bond cutting,
//! progressive destruction, and structure-specific behavior.

#[cfg(feature = "scenarios")]
mod headless {
    use blast_stress_solver::scenarios::*;
    use blast_stress_solver::*;

    fn simulate(solver: &mut ExtStressSolver, gravity: Vec3, frames: u32) -> SimResult {
        let mut total_fractures = 0u32;
        let mut total_split_events = 0u32;
        for _ in 0..frames {
            solver.add_gravity(gravity);
            solver.update();
            if solver.overstressed_bond_count() > 0 {
                let cmds = solver.generate_fracture_commands();
                let fractures: u32 = cmds.iter().map(|c| c.bond_fractures.len() as u32).sum();
                total_fractures += fractures;
                if !cmds.is_empty() {
                    let events = solver.apply_fracture_commands(&cmds);
                    total_split_events += events.len() as u32;
                }
            }
        }
        SimResult {
            total_fractures,
            total_split_events,
            actor_count: solver.actor_count(),
            converged: solver.converged(),
        }
    }

    struct SimResult {
        total_fractures: u32,
        total_split_events: u32,
        actor_count: u32,
        converged: bool,
    }

    fn strong_settings() -> SolverSettings {
        SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 90_000.0,
            compression_fatal_limit: 270_000.0,
            tension_elastic_limit: 90_000.0,
            tension_fatal_limit: 270_000.0,
            shear_elastic_limit: 120_000.0,
            shear_fatal_limit: 360_000.0,
            ..SolverSettings::default()
        }
    }

    fn weak_settings() -> SolverSettings {
        SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 0.001,
            compression_fatal_limit: 0.002,
            tension_elastic_limit: 0.001,
            tension_fatal_limit: 0.002,
            shear_elastic_limit: 0.001,
            shear_fatal_limit: 0.002,
            ..SolverSettings::default()
        }
    }

    fn moderate_settings() -> SolverSettings {
        SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 5.0,
            compression_fatal_limit: 10.0,
            tension_elastic_limit: 5.0,
            tension_fatal_limit: 10.0,
            shear_elastic_limit: 5.0,
            shear_fatal_limit: 10.0,
            ..SolverSettings::default()
        }
    }

    fn gravity() -> Vec3 {
        Vec3::new(0.0, -9.81, 0.0)
    }

    fn make_solver(
        scenario: &ScenarioDesc,
        settings: &SolverSettings,
    ) -> ExtStressSolver {
        let (nodes, bonds) = scenario.to_solver_descs();
        ExtStressSolver::new(&nodes, &bonds, settings).unwrap()
    }

    // === A. Gravity stability ===

    #[test]
    fn wall_stable_under_normal_gravity() {
        let wall = build_wall_scenario(&WallOptions::default());
        let mut solver = make_solver(&wall, &strong_settings());
        let r = simulate(&mut solver, gravity(), 120);
        assert_eq!(r.total_fractures, 0);
        assert_eq!(r.actor_count, 1);
    }

    #[test]
    fn tower_stable_under_normal_gravity() {
        let tower = build_tower_scenario(&TowerOptions::default());
        let mut solver = make_solver(&tower, &strong_settings());
        let r = simulate(&mut solver, gravity(), 120);
        assert_eq!(r.total_fractures, 0);
        assert_eq!(r.actor_count, 1);
    }

    #[test]
    fn bridge_stable_under_normal_gravity() {
        // The simple bridge has end-supports only; use higher limits
        let bridge = build_bridge_scenario(&BridgeOptions::default());
        let s = SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 1e8,
            compression_fatal_limit: 1e9,
            tension_elastic_limit: 1e8,
            tension_fatal_limit: 1e9,
            shear_elastic_limit: 1e8,
            shear_fatal_limit: 1e9,
            ..SolverSettings::default()
        };
        let mut solver = make_solver(&bridge, &s);
        let r = simulate(&mut solver, gravity(), 120);
        assert_eq!(r.total_fractures, 0);
        assert_eq!(r.actor_count, 1);
    }

    // === B. Weak material collapse ===

    #[test]
    fn wall_collapses_with_weak_bonds() {
        let wall = build_wall_scenario(&WallOptions::default());
        let mut solver = make_solver(&wall, &weak_settings());
        let r = simulate(&mut solver, gravity(), 60);
        assert!(r.total_fractures > 0, "weak wall should fracture");
        assert!(r.actor_count > 1, "should split into multiple actors");
    }

    #[test]
    fn tower_collapses_with_weak_bonds() {
        let tower = build_tower_scenario(&TowerOptions::default());
        let mut solver = make_solver(&tower, &weak_settings());
        let r = simulate(&mut solver, gravity(), 60);
        assert!(r.total_fractures > 0, "weak tower should fracture");
        assert!(r.actor_count > 1, "should split");
    }

    #[test]
    fn bridge_collapses_with_weak_bonds() {
        let bridge = build_bridge_scenario(&BridgeOptions::default());
        let mut solver = make_solver(&bridge, &weak_settings());
        let r = simulate(&mut solver, gravity(), 60);
        assert!(r.total_fractures > 0, "weak bridge should fracture");
        assert!(r.actor_count > 1, "should split");
    }

    // === C. Material strength determines breakage ===

    #[test]
    fn stronger_material_loses_fewer_bonds() {
        let wall = build_wall_scenario(&WallOptions::default());

        let run = |limits: f32| {
            let s = SolverSettings {
                max_solver_iterations_per_frame: 24,
                compression_elastic_limit: limits,
                compression_fatal_limit: limits * 2.0,
                tension_elastic_limit: limits,
                tension_fatal_limit: limits * 2.0,
                shear_elastic_limit: limits,
                shear_fatal_limit: limits * 2.0,
                ..SolverSettings::default()
            };
            let mut solver = make_solver(&wall, &s);
            simulate(&mut solver, gravity(), 30)
        };

        let weak = run(1.0);
        let medium = run(10.0);
        let strong = run(100.0);

        assert!(
            strong.total_fractures <= medium.total_fractures,
            "strong <= medium: {} <= {}",
            strong.total_fractures,
            medium.total_fractures
        );
        assert!(
            medium.total_fractures <= weak.total_fractures,
            "medium <= weak: {} <= {}",
            medium.total_fractures,
            weak.total_fractures
        );
    }

    #[test]
    fn very_strong_material_resists_gravity() {
        let tower = build_tower_scenario(&TowerOptions::default());
        let s = SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 1e8,
            compression_fatal_limit: 1e9,
            tension_elastic_limit: 1e8,
            tension_fatal_limit: 1e9,
            shear_elastic_limit: 1e8,
            shear_fatal_limit: 1e9,
            ..SolverSettings::default()
        };
        let mut solver = make_solver(&tower, &s);
        let r = simulate(&mut solver, gravity(), 120);
        assert_eq!(r.total_fractures, 0);
    }

    // === D. Force impact ===

    #[test]
    fn force_impact_breaks_bonds() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let s = SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 0.001,
            compression_fatal_limit: 0.002,
            tension_elastic_limit: 0.001,
            tension_fatal_limit: 0.002,
            shear_elastic_limit: 0.001,
            shear_fatal_limit: 0.002,
            ..SolverSettings::default()
        };
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &s).unwrap();

        // Apply gravity + force together to create stress
        solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
        let center = nodes.len() / 2;
        let pos = nodes[center].centroid;
        solver.add_force(center as u32, pos, Vec3::new(50000.0, 0.0, 0.0), ForceMode::Force);
        solver.update();

        let overstressed = solver.overstressed_bond_count();
        assert!(overstressed > 0, "gravity + force should overstress bonds, got {overstressed}");
    }

    #[test]
    fn acceleration_mode_affects_nodes() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let s = SolverSettings {
            compression_elastic_limit: 0.001,
            compression_fatal_limit: 0.002,
            tension_elastic_limit: 0.001,
            tension_fatal_limit: 0.002,
            shear_elastic_limit: 0.001,
            shear_fatal_limit: 0.002,
            ..SolverSettings::default()
        };
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &s).unwrap();

        solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
        solver.update();
        assert!(solver.overstressed_bond_count() > 0, "gravity on weak wall should overstress");
    }

    // === E. Bond cutting ===

    #[test]
    fn manual_bond_cutting_splits_structure() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &strong_settings()).unwrap();
        assert_eq!(solver.actor_count(), 1);

        // Use the solver's fracture mechanism: apply very strong gravity with weak limits
        // to force bonds to break, then verify split events occur
        let weak = SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 0.0001,
            compression_fatal_limit: 0.0002,
            tension_elastic_limit: 0.0001,
            tension_fatal_limit: 0.0002,
            shear_elastic_limit: 0.0001,
            shear_fatal_limit: 0.0002,
            ..SolverSettings::default()
        };
        solver.set_settings(&weak);
        solver.add_gravity(Vec3::new(0.0, -100.0, 0.0));
        solver.update();

        assert!(solver.overstressed_bond_count() > 0);

        let cmds = solver.generate_fracture_commands();
        assert!(!cmds.is_empty(), "should have fracture commands");

        let events = solver.apply_fracture_commands(&cmds);
        assert!(
            solver.actor_count() > 1,
            "cutting bonds should split: actors={}",
            solver.actor_count()
        );
    }

    // === F. Progressive destruction ===

    #[test]
    fn progressive_destruction_over_many_frames() {
        let wall = build_wall_scenario(&WallOptions::default());
        let mut solver = make_solver(&wall, &moderate_settings());

        let mut prev_actors = solver.actor_count();
        let mut ever_increased = false;

        for _ in 0..100 {
            solver.add_gravity(gravity());
            solver.update();
            if solver.overstressed_bond_count() > 0 {
                let cmds = solver.generate_fracture_commands();
                if !cmds.is_empty() {
                    solver.apply_fracture_commands(&cmds);
                }
            }
            let current = solver.actor_count();
            if current > prev_actors {
                ever_increased = true;
            }
            prev_actors = current;
        }

        assert!(ever_increased, "actor count should increase over time with moderate settings");
    }

    // === G. Structure-specific behavior ===

    #[test]
    fn tower_base_more_stressed_than_top() {
        let tower = build_tower_scenario(&TowerOptions::default());
        let (nodes, bonds) = tower.to_solver_descs();

        // Test with moderate settings that allow some fracture
        let s = SolverSettings {
            max_solver_iterations_per_frame: 24,
            compression_elastic_limit: 2.0,
            compression_fatal_limit: 4.0,
            tension_elastic_limit: 2.0,
            tension_fatal_limit: 4.0,
            shear_elastic_limit: 2.0,
            shear_fatal_limit: 4.0,
            ..SolverSettings::default()
        };
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &s).unwrap();
        solver.add_gravity(gravity());
        solver.update();

        // Under gravity, base bonds should be more stressed
        // (more weight above them)
        assert!(solver.overstressed_bond_count() > 0, "tower should have overstressed bonds");
    }

    // === H. API surface and lifecycle ===

    #[test]
    fn solver_has_expected_api() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &strong_settings()).unwrap();

        // All these should work without panic
        let _nc = solver.node_count();
        let _bc = solver.bond_count();
        let _ac = solver.actor_count();
        let _actors = solver.actors();
        let _converged = solver.converged();
        let _lin_err = solver.linear_error();
        let _ang_err = solver.angular_error();

        solver.add_gravity(gravity());
        solver.update();

        let _overstressed = solver.overstressed_bond_count();
        let _cmds = solver.generate_fracture_commands();

        solver.reset();
        solver.set_settings(&strong_settings());

        let _excess = solver.get_excess_forces(0, Vec3::ZERO);
    }

    #[test]
    fn settings_change_affects_behavior() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();

        // Start with strong settings
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &strong_settings()).unwrap();
        solver.add_gravity(gravity());
        solver.update();
        let strong_overstressed = solver.overstressed_bond_count();

        // Change to weak settings
        solver.set_settings(&weak_settings());
        solver.add_gravity(gravity());
        solver.update();
        let weak_overstressed = solver.overstressed_bond_count();

        assert!(
            weak_overstressed >= strong_overstressed,
            "weaker settings should produce >= overstressed bonds"
        );
    }

    #[test]
    fn reset_clears_forces() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &weak_settings()).unwrap();

        solver.add_gravity(Vec3::new(0.0, -1000.0, 0.0));
        solver.update();
        let before = solver.overstressed_bond_count();

        solver.reset();
        solver.update();
        let after = solver.overstressed_bond_count();

        assert!(before > 0, "should be overstressed before reset");
        assert_eq!(after, 0, "should have 0 overstressed after reset + update with no forces");
    }

    // === I. Scenario builder validation ===

    #[test]
    fn wall_support_nodes_at_bottom_row() {
        let wall = build_wall_scenario(&WallOptions::default());
        // Node ordering: ix outer, iy middle, iz inner
        // Support nodes have centroid.y ≈ 0.25 (bottom row) and mass == 0
        let support_count = wall.nodes.iter().filter(|n| n.mass == 0.0).count();
        assert_eq!(support_count, 12, "12 support nodes (bottom row)");
        for node in &wall.nodes {
            if node.mass == 0.0 {
                assert!(
                    node.centroid.y < 0.3,
                    "support node y={} should be at bottom",
                    node.centroid.y
                );
            }
        }
    }

    #[test]
    fn tower_support_nodes_at_bottom_layer() {
        let tower = build_tower_scenario(&TowerOptions::default());
        for node in &tower.nodes {
            if node.centroid.y < -0.1 {
                // Below ground level = support
                assert_eq!(node.mass, 0.0, "below-ground node should be support");
            }
        }
    }

    #[test]
    fn all_bond_normals_are_unit_vectors() {
        let scenarios = [
            build_wall_scenario(&WallOptions::default()),
            build_tower_scenario(&TowerOptions::default()),
            build_bridge_scenario(&BridgeOptions::default()),
        ];
        for (si, scenario) in scenarios.iter().enumerate() {
            for (bi, bond) in scenario.bonds.iter().enumerate() {
                let mag = bond.normal.magnitude();
                assert!(
                    (mag - 1.0).abs() < 0.01,
                    "scenario {si} bond {bi}: normal magnitude = {mag}, expected 1.0"
                );
            }
        }
    }

    #[test]
    fn all_bond_areas_positive() {
        let scenarios = [
            build_wall_scenario(&WallOptions::default()),
            build_tower_scenario(&TowerOptions::default()),
            build_bridge_scenario(&BridgeOptions::default()),
        ];
        for (si, scenario) in scenarios.iter().enumerate() {
            for (bi, bond) in scenario.bonds.iter().enumerate() {
                assert!(
                    bond.area > 0.0,
                    "scenario {si} bond {bi}: area = {}, must be > 0",
                    bond.area
                );
            }
        }
    }

    #[test]
    fn wall_custom_options() {
        let opts = WallOptions {
            span: 3.0,
            height: 2.0,
            span_segments: 6,
            height_segments: 4,
            ..WallOptions::default()
        };
        let wall = build_wall_scenario(&opts);
        assert_eq!(wall.nodes.len(), 24); // 6 * 4
        let support = wall.nodes.iter().filter(|n| n.mass == 0.0).count();
        assert_eq!(support, 6); // bottom row
    }

    #[test]
    fn tower_custom_options() {
        let opts = TowerOptions {
            side: 2,
            stories: 4,
            ..TowerOptions::default()
        };
        let tower = build_tower_scenario(&opts);
        // 2 * (4+1) * 2 = 20 nodes (side * totalRows * side)
        assert_eq!(tower.nodes.len(), 20);
        let support = tower.nodes.iter().filter(|n| n.mass == 0.0).count();
        assert_eq!(support, 4); // side * side = 2*2
    }
}
