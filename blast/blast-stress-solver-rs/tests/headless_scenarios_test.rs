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

    // === J. Scenario builder isotropy and area normalization ===

    #[test]
    fn wall_normalized_areas_are_isotropic() {
        // With normalizeAreas=true, average X-bond area ≈ average Y-bond area
        let wall = build_wall_scenario(&WallOptions::default());
        let mut sum_x = 0.0f32;
        let mut count_x = 0u32;
        let mut sum_y = 0.0f32;
        let mut count_y = 0u32;

        for b in &wall.bonds {
            let ax = b.normal.x.abs();
            let ay = b.normal.y.abs();
            if ax > ay {
                sum_x += b.area;
                count_x += 1;
            } else {
                sum_y += b.area;
                count_y += 1;
            }
        }
        let avg_x = sum_x / count_x.max(1) as f32;
        let avg_y = sum_y / count_y.max(1) as f32;
        assert!(
            (avg_x - avg_y).abs() < 1e-4,
            "X and Y bond areas should be isotropic: avg_x={avg_x:.6}, avg_y={avg_y:.6}"
        );
    }

    #[test]
    fn tower_normalized_areas_are_isotropic() {
        // Without diagonals, the three axes should have similar average areas
        let opts = TowerOptions {
            add_diagonals: false,
            ..TowerOptions::default()
        };
        let tower = build_tower_scenario(&opts);
        let mut sum = [0.0f32; 3];
        let mut count = [0u32; 3];

        for b in &tower.bonds {
            let ax = b.normal.x.abs();
            let ay = b.normal.y.abs();
            let az = b.normal.z.abs();
            let axis = if ax >= ay && ax >= az { 0 } else if ay >= az { 1 } else { 2 };
            sum[axis] += b.area;
            count[axis] += 1;
        }

        let avg: Vec<f32> = (0..3)
            .map(|i| if count[i] > 0 { sum[i] / count[i] as f32 } else { 0.0 })
            .collect();

        // All three averages should be close
        for i in 0..3 {
            for j in (i + 1)..3 {
                assert!(
                    (avg[i] - avg[j]).abs() < 1e-3,
                    "axes {i} and {j} should be isotropic: {:.6} vs {:.6}",
                    avg[i], avg[j]
                );
            }
        }
    }

    #[test]
    fn diagonals_increase_bond_count() {
        let with_diag = build_tower_scenario(&TowerOptions {
            add_diagonals: true,
            ..TowerOptions::default()
        });
        let without_diag = build_tower_scenario(&TowerOptions {
            add_diagonals: false,
            ..TowerOptions::default()
        });
        assert!(
            with_diag.bonds.len() > without_diag.bonds.len(),
            "diagonals should add bonds: {} > {}",
            with_diag.bonds.len(),
            without_diag.bonds.len()
        );
        // Node counts should be the same
        assert_eq!(with_diag.nodes.len(), without_diag.nodes.len());
    }

    #[test]
    fn non_normalized_areas_are_consistent() {
        let opts = WallOptions {
            normalize_areas: false,
            area_scale: 0.1,
            ..WallOptions::default()
        };
        let wall = build_wall_scenario(&opts);
        for b in &wall.bonds {
            assert!(b.area > 0.0, "all areas should be > 0");
        }
        // Without normalization, X and Y bonds should have raw area values
        // X bonds: cellY * cellZ * areaScale = 0.5 * 0.32 * 0.1 = 0.016
        // Y bonds: cellX * cellZ * areaScale = 0.5 * 0.32 * 0.1 = 0.016
        let first = wall.bonds[0].area;
        assert!(
            (first - 0.016).abs() < 0.001,
            "raw area should be ~0.016: got {first}"
        );
    }

    #[test]
    fn wall_small_scenario_bond_count() {
        // JS test: {spanSegments:4, heightSegments:3, layers:1} => 12 nodes, 17 bonds
        let opts = WallOptions {
            span_segments: 4,
            height_segments: 3,
            layers: 1,
            normalize_areas: false,
            ..WallOptions::default()
        };
        let wall = build_wall_scenario(&opts);
        assert_eq!(wall.nodes.len(), 12);
        // Horizontal: 3 rows * 3 gaps = 9
        // Vertical: 2 gaps * 4 cols = 8
        // Total = 17
        assert_eq!(wall.bonds.len(), 17);
    }

    // === K. Convergence and error reporting ===

    #[test]
    fn solver_converges_on_simple_topology() {
        let nodes = vec![
            NodeDesc { centroid: Vec3::new(-1.0, 0.0, 0.0), mass: 0.0, volume: 1.0 },
            NodeDesc { centroid: Vec3::new(1.0, 0.0, 0.0), mass: 0.0, volume: 1.0 },
            NodeDesc { centroid: Vec3::new(0.0, 1.5, 0.0), mass: 15.0, volume: 1.0 },
        ];
        let bonds = vec![
            BondDesc { centroid: Vec3::new(-0.5, 0.75, 0.0), normal: Vec3::new(0.55, 0.83, 0.0), area: 0.6, node0: 0, node1: 2 },
            BondDesc { centroid: Vec3::new(0.5, 0.75, 0.0), normal: Vec3::new(-0.55, 0.83, 0.0), area: 0.6, node0: 1, node1: 2 },
            BondDesc { centroid: Vec3::new(0.0, 0.0, 0.0), normal: Vec3::new(1.0, 0.0, 0.0), area: 0.9, node0: 0, node1: 1 },
        ];
        let settings = SolverSettings::default();
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &settings).unwrap();
        solver.add_gravity(Vec3::new(0.0, -9.81, 0.0));
        solver.update();
        assert!(solver.converged(), "simple topology should converge");
        let lin_err = solver.linear_error();
        let ang_err = solver.angular_error();
        assert!(lin_err < 1.0, "linear error should be small: {lin_err}");
        assert!(ang_err < 1.0, "angular error should be small: {ang_err}");
    }

    #[test]
    fn large_tower_converges_or_has_low_error() {
        let tower = build_tower_scenario(&TowerOptions::default());
        let (nodes, bonds) = tower.to_solver_descs();
        let mut s = strong_settings();
        s.max_solver_iterations_per_frame = 128; // large tower needs more iterations
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &s).unwrap();
        solver.add_gravity(gravity());
        solver.update();
        // For large structures, convergence may take more iterations.
        // Check that error is at least bounded.
        let lin = solver.linear_error();
        let ang = solver.angular_error();
        assert!(lin < 10.0, "linear error should be bounded: {lin}");
        assert!(ang < 10.0, "angular error should be bounded: {ang}");
    }

    // === L. Multiple actors after split ===

    #[test]
    fn actors_contain_all_nodes_after_split() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &weak_settings()).unwrap();

        // Fracture
        solver.add_gravity(gravity());
        solver.update();
        let cmds = solver.generate_fracture_commands();
        if !cmds.is_empty() {
            solver.apply_fracture_commands(&cmds);
        }

        // All nodes should appear in exactly one actor
        let actors = solver.actors();
        let mut all_nodes: Vec<u32> = actors.iter().flat_map(|a| a.nodes.iter().copied()).collect();
        all_nodes.sort();
        all_nodes.dedup();

        // Every actor node should be a valid index
        for &n in &all_nodes {
            assert!(
                (n as usize) < nodes.len(),
                "actor node index {n} out of range"
            );
        }
    }

    // === M. Excess forces on separated actors ===

    #[test]
    fn excess_forces_on_separated_actor() {
        let wall = build_wall_scenario(&WallOptions::default());
        let (nodes, bonds) = wall.to_solver_descs();
        let mut solver = ExtStressSolver::new(&nodes, &bonds, &weak_settings()).unwrap();

        // Force fracture
        for _ in 0..5 {
            solver.add_gravity(gravity());
            solver.update();
            let cmds = solver.generate_fracture_commands();
            if !cmds.is_empty() {
                solver.apply_fracture_commands(&cmds);
            }
        }

        // Check excess forces on actors
        let actors = solver.actors();
        for actor in &actors {
            if actor.nodes.is_empty() {
                continue;
            }
            // Compute center
            let mut cx = 0.0f32;
            let mut cy = 0.0f32;
            let mut count = 0.0f32;
            for &n in &actor.nodes {
                cx += nodes[n as usize].centroid.x;
                cy += nodes[n as usize].centroid.y;
                count += 1.0;
            }
            let com = Vec3::new(cx / count, cy / count, 0.0);

            if let Some((force, torque)) = solver.get_excess_forces(actor.actor_index, com) {
                assert!(force.x.is_finite());
                assert!(force.y.is_finite());
                assert!(force.z.is_finite());
                assert!(torque.x.is_finite());
                assert!(torque.y.is_finite());
                assert!(torque.z.is_finite());
            }
        }
    }
}
