#![cfg(feature = "authoring")]

use blast_stress_solver::authoring::{
    build_scenario_from_pieces, create_bonds_from_triangles, BondingMode, BondingOptions,
    ScenarioPiece, TriangleChunk,
};
use blast_stress_solver::{ScenarioNode, Vec3};

fn cuboid(min: Vec3, max: Vec3) -> Vec<Vec3> {
    let p000 = Vec3::new(min.x, min.y, min.z);
    let p001 = Vec3::new(min.x, min.y, max.z);
    let p010 = Vec3::new(min.x, max.y, min.z);
    let p011 = Vec3::new(min.x, max.y, max.z);
    let p100 = Vec3::new(max.x, min.y, min.z);
    let p101 = Vec3::new(max.x, min.y, max.z);
    let p110 = Vec3::new(max.x, max.y, min.z);
    let p111 = Vec3::new(max.x, max.y, max.z);

    vec![
        p000, p101, p001, p000, p100, p101, // bottom
        p010, p011, p111, p010, p111, p110, // top
        p000, p001, p011, p000, p011, p010, // left
        p100, p110, p111, p100, p111, p101, // right
        p000, p010, p110, p000, p110, p100, // front
        p001, p101, p111, p001, p111, p011, // back
    ]
}

fn unit_chunks(gap: f32) -> Vec<TriangleChunk> {
    vec![
        TriangleChunk {
            triangles: cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0)),
            bondable: true,
        },
        TriangleChunk {
            triangles: cuboid(
                Vec3::new(1.0 + gap, 0.0, 0.0),
                Vec3::new(2.0 + gap, 1.0, 1.0),
            ),
            bondable: true,
        },
    ]
}

#[test]
fn exact_mode_bonds_touching_boxes() {
    let bonds = create_bonds_from_triangles(&unit_chunks(0.0), &BondingOptions::default())
        .expect("exact bonding should succeed");

    assert_eq!(bonds.len(), 1, "expected one shared-face bond");
    let bond = &bonds[0];
    assert_eq!(
        (bond.node0.min(bond.node1), bond.node0.max(bond.node1)),
        (0, 1)
    );
    assert!((bond.centroid.x - 1.0).abs() < 1.0e-5);
    assert!((bond.centroid.y - 0.5).abs() < 1.0e-5);
    assert!((bond.centroid.z - 0.5).abs() < 1.0e-5);
    assert!(
        (bond.area - 1.0).abs() < 1.0e-3,
        "unexpected bond area {}",
        bond.area
    );
    assert!((bond.normal.x.abs() - 1.0).abs() < 1.0e-5);
}

#[test]
fn average_mode_bonds_small_gap() {
    let options = BondingOptions {
        mode: BondingMode::Average {
            max_separation: 0.2,
        },
    };
    let bonds = create_bonds_from_triangles(&unit_chunks(0.1), &options)
        .expect("average bonding should succeed");

    assert_eq!(bonds.len(), 1, "expected one bond across the gap");
    assert_eq!(
        (
            bonds[0].node0.min(bonds[0].node1),
            bonds[0].node0.max(bonds[0].node1)
        ),
        (0, 1)
    );
}

#[test]
fn bondable_flag_controls_participation() {
    let mut chunks = unit_chunks(0.0);
    chunks[0].bondable = false;

    let bonds = create_bonds_from_triangles(&chunks, &BondingOptions::default())
        .expect("bond generation should succeed");

    assert!(
        bonds.is_empty(),
        "non-bondable chunks should not create interfaces"
    );
}

#[test]
fn build_scenario_from_pieces_auto_bonds_and_infers_sizes() {
    let pieces = vec![
        ScenarioPiece {
            node: ScenarioNode {
                centroid: Vec3::new(0.5, 0.5, 0.5),
                mass: 1.0,
                volume: 1.0,
            },
            triangles: cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0)),
            bondable: true,
            node_size: None,
            collider_shape: None,
        },
        ScenarioPiece {
            node: ScenarioNode {
                centroid: Vec3::new(1.5, 0.5, 0.5),
                mass: 1.0,
                volume: 1.0,
            },
            triangles: cuboid(Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 1.0, 1.0)),
            bondable: true,
            node_size: None,
            collider_shape: None,
        },
    ];

    let scenario = build_scenario_from_pieces(&pieces, &BondingOptions::default())
        .expect("scenario build should succeed");

    assert_eq!(scenario.nodes.len(), 2);
    assert_eq!(scenario.bonds.len(), 1);
    assert_eq!(scenario.node_sizes.len(), 2);
    assert_eq!(scenario.collider_shapes.len(), 2);
    assert!((scenario.node_sizes[0].x - 1.0).abs() < 1.0e-5);
    assert!((scenario.node_sizes[0].y - 1.0).abs() < 1.0e-5);
    assert!((scenario.node_sizes[0].z - 1.0).abs() < 1.0e-5);
}
