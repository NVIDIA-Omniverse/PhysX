use blast_stress_solver::authoring::{
    build_scenario_from_pieces, BondingMode, BondingOptions, ScenarioPiece,
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
        p000, p101, p001, p000, p100, p101, p010, p011, p111, p010, p111, p110, p000, p001, p011,
        p000, p011, p010, p100, p110, p111, p100, p111, p101, p000, p010, p110, p000, p110, p100,
        p001, p101, p111, p001, p111, p011,
    ]
}

fn main() {
    let left_piece = ScenarioPiece {
        node: ScenarioNode {
            centroid: Vec3::new(0.5, 0.5, 0.5),
            mass: 1.0,
            volume: 1.0,
        },
        triangles: cuboid(Vec3::new(0.0, 0.0, 0.0), Vec3::new(1.0, 1.0, 1.0)),
        bondable: true,
        node_size: None,
        collider_shape: None,
    };

    let right_piece = ScenarioPiece {
        node: ScenarioNode {
            centroid: Vec3::new(1.5, 0.5, 0.5),
            mass: 1.0,
            volume: 1.0,
        },
        triangles: cuboid(Vec3::new(1.0, 0.0, 0.0), Vec3::new(2.0, 1.0, 1.0)),
        bondable: true,
        node_size: None,
        collider_shape: None,
    };

    let scenario = build_scenario_from_pieces(
        &[left_piece, right_piece],
        &BondingOptions {
            mode: BondingMode::Exact,
        },
    )
    .expect("failed to auto-bond pieces");

    println!(
        "nodes={} bonds={} first_bond={:?}",
        scenario.nodes.len(),
        scenario.bonds.len(),
        scenario.bonds.first()
    );
}
