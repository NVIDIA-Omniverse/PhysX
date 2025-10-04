use rust_stress_example::*;

struct ExampleBond {
    name: &'static str,
    area: f32,
    desc: StressBondDesc,
}

struct FrameInput {
    lin: StressVec3,
    ang: StressVec3,
}

fn main() {
    const TOP_NODE: usize = 2;

    let nodes = [
        StressNodeDesc {
            com: vec3(-1.0, 0.0, 0.0),
            mass: 25.0,
            inertia: 2.5,
        },
        StressNodeDesc {
            com: vec3(1.0, 0.0, 0.0),
            mass: 25.0,
            inertia: 2.5,
        },
        StressNodeDesc {
            com: vec3(0.0, 1.5, 0.0),
            mass: 15.0,
            inertia: 1.5,
        },
    ];

    let bonds = [
        StressBondDesc {
            centroid: vec3(-0.5, 0.75, 0.0),
            node0: 0,
            node1: 2,
        },
        StressBondDesc {
            centroid: vec3(0.5, 0.75, 0.0),
            node0: 1,
            node1: 2,
        },
        StressBondDesc {
            centroid: vec3(0.0, 0.0, 0.0),
            node0: 0,
            node1: 1,
        },
    ];

    const BOND_NAMES: [&str; 3] = ["left diagonal", "right diagonal", "base tie"];
    const BOND_AREAS: [f32; 3] = [0.6, 0.6, 0.9];

    let data_params = StressDataParams {
        equalize_masses: 1,
        center_bonds: 1,
    };

    let mut solver =
        StressProcessor::new(&nodes, &bonds, data_params).expect("failed to build StressProcessor");

    println!(
        "StressProcessor created with {} nodes and {} bonds (SIMD: {})",
        solver.node_count(),
        solver.bond_count(),
        StressProcessor::using_simd()
    );

    let node_data: Vec<_> = (0..solver.node_count())
        .map(|i| solver.node_desc(i).expect("missing node descriptor"))
        .collect();
    println!("\nNode data:");
    for (i, node) in node_data.iter().enumerate() {
        println!(
            "  node {:<2} | mass {:>6.2} | inertia {:>5.2} | com ({:>5.2}, {:>5.2}, {:>5.2})",
            i, node.mass, node.inertia, node.com.x, node.com.y, node.com.z
        );
    }

    let mut bond_entries: Vec<ExampleBond> = (0..solver.bond_count() as usize)
        .map(|i| ExampleBond {
            name: BOND_NAMES[i],
            area: BOND_AREAS[i],
            desc: solver.bond_desc(i as u32).expect("missing bond descriptor"),
        })
        .collect();

    println!("\nInitial bonds:");
    for (bond, desc) in bond_entries.iter().zip(bonds.iter()) {
        println!(
            "  {:<15} connects nodes {} and {} (centroid {:.2}, {:.2}, {:.2}) area {:.2}",
            bond.name,
            desc.node0,
            desc.node1,
            desc.centroid.x,
            desc.centroid.y,
            desc.centroid.z,
            bond.area
        );
    }

    let mut impulses = vec![StressImpulse::default(); bond_entries.len()];
    let mut velocities = vec![StressVelocity::default(); node_data.len()];

    let mut solver_params = StressSolverParams::default();
    solver_params.max_iterations = 128;
    solver_params.tolerance = 1.0e-5;
    solver_params.warm_start = 1;

    let frame_inputs = [
        FrameInput {
            lin: vec3(0.0, -24.0, 0.0),
            ang: vec3(0.0, 0.0, 3.0),
        },
        FrameInput {
            lin: vec3(8.0, -42.0, 0.0),
            ang: vec3(0.0, 0.0, 7.0),
        },
        FrameInput {
            lin: vec3(14.0, -63.0, 0.0),
            ang: vec3(0.0, 0.0, 11.5),
        },
        FrameInput {
            lin: vec3(20.0, -90.0, 0.0),
            ang: vec3(0.0, 0.0, 16.0),
        },
    ];

    let limits = StressLimits {
        compression_elastic_limit: 320.0,
        compression_fatal_limit: 520.0,
        tension_elastic_limit: 180.0,
        tension_fatal_limit: 320.0,
        shear_elastic_limit: 140.0,
        shear_fatal_limit: 260.0,
    };

    for (frame, input) in frame_inputs.iter().enumerate() {
        if bond_entries.is_empty() {
            println!("\nAll bonds were removed; stress network resolved.");
            break;
        }

        velocities.fill(StressVelocity::default());
        velocities[TOP_NODE] = StressVelocity {
            lin: input.lin,
            ang: input.ang,
        };

        let (iterations, error_sq) = solver
            .solve(&mut impulses, &velocities, solver_params, false)
            .expect("solver failed");

        let linear_error = error_sq.lin.sqrt();
        let angular_error = error_sq.ang.sqrt();

        println!(
            "\nFrame {}: iterations {:>3}, linear error {:>8.5}, angular error {:>8.5}",
            frame, iterations, linear_error, angular_error
        );

        let mut overstressed = Vec::new();

        for (index, bond) in bond_entries.iter().enumerate() {
            let stress = compute_bond_stress(&bond.desc, &impulses[index], &node_data, bond.area);
            let severity = limits.severity(&stress);
            println!(
                "  {:<15} | comp {:>7.2} ({:>4.2}) | tens {:>7.2} ({:>4.2}) | shear {:>7.2} ({:>4.2})",
                bond.name,
                stress.compression,
                severity.compression,
                stress.tension,
                severity.tension,
                stress.shear,
                severity.shear
            );

            if let Some(mode) = limits.failure_mode(&stress) {
                println!(
                    "    -> {} limit exceeded (fatal {:.1}); scheduling removal",
                    mode,
                    match mode {
                        StressFailure::Compression => limits.compression_fatal_threshold(),
                        StressFailure::Tension => limits.tension_fatal_threshold(),
                        StressFailure::Shear => limits.shear_fatal_threshold(),
                    }
                );
                overstressed.push((index, mode));
            }
        }

        if !overstressed.is_empty() {
            overstressed.sort_by_key(|(idx, _)| *idx);
            for (idx, mode) in overstressed.into_iter().rev() {
                let removed = bond_entries[idx].name;
                if solver.remove_bond(idx as u32) {
                    println!("  Removing bond '{}' due to {} failure", removed, mode);
                    bond_entries.swap_remove(idx);
                    impulses.swap_remove(idx);
                }
            }
            impulses.resize(bond_entries.len(), StressImpulse::default());
            for i in 0..bond_entries.len() {
                if let Some(desc) = solver.bond_desc(i as u32) {
                    bond_entries[i].desc = desc;
                }
            }
            println!(
                "  Solver now tracks {} remaining bonds",
                solver.bond_count()
            );
        }
    }
}
