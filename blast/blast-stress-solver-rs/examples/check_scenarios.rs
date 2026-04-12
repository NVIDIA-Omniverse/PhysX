use blast_stress_solver::scenarios::*;

fn main() {
    let wall = build_wall_scenario(&WallOptions::default());
    let support = wall.nodes.iter().filter(|n| n.mass == 0.0).count();
    let dynamic = wall.nodes.iter().filter(|n| n.mass > 0.0).count();
    println!(
        "WALL: nodes={} bonds={} support={} dynamic={}",
        wall.nodes.len(),
        wall.bonds.len(),
        support,
        dynamic
    );
    if !wall.nodes.is_empty() {
        let first = &wall.nodes[0];
        let last = &wall.nodes[wall.nodes.len() - 1];
        println!(
            "  first_centroid=({:.4}, {:.4}, {:.4})",
            first.centroid.x, first.centroid.y, first.centroid.z
        );
        println!(
            "  last_centroid=({:.4}, {:.4}, {:.4})",
            last.centroid.x, last.centroid.y, last.centroid.z
        );
        println!("  first_bond_area={:.6}", wall.bonds[0].area);
        println!(
            "  last_bond_area={:.6}",
            wall.bonds[wall.bonds.len() - 1].area
        );
    }

    let tower = build_tower_scenario(&TowerOptions::default());
    let t_support = tower.nodes.iter().filter(|n| n.mass == 0.0).count();
    let t_dynamic = tower.nodes.iter().filter(|n| n.mass > 0.0).count();
    println!(
        "TOWER: nodes={} bonds={} support={} dynamic={}",
        tower.nodes.len(),
        tower.bonds.len(),
        t_support,
        t_dynamic
    );

    let bridge = build_bridge_scenario(&BridgeOptions::default());
    let b_support = bridge.nodes.iter().filter(|n| n.mass == 0.0).count();
    let b_dynamic = bridge.nodes.iter().filter(|n| n.mass > 0.0).count();
    println!(
        "BRIDGE: nodes={} bonds={} support={} dynamic={}",
        bridge.nodes.len(),
        bridge.bonds.len(),
        b_support,
        b_dynamic
    );
}
