use crate::types::*;

/// Options for building a wall scenario.
#[derive(Clone, Debug)]
pub struct WallOptions {
    pub columns: u32,
    pub rows: u32,
    pub brick_width: f32,
    pub brick_height: f32,
    pub brick_depth: f32,
    pub density: f32,
    /// Whether to offset every other row by half a brick (running bond pattern).
    pub stagger: bool,
}

impl Default for WallOptions {
    fn default() -> Self {
        Self {
            columns: 5,
            rows: 5,
            brick_width: 1.0,
            brick_height: 0.5,
            brick_depth: 0.5,
            density: 1.0,
            stagger: true,
        }
    }
}

/// Build a wall scenario: a grid of bricks with bonds between adjacent bricks.
/// The bottom row has mass == 0 (support nodes).
pub fn build_wall_scenario(opts: &WallOptions) -> ScenarioDesc {
    let mut nodes = Vec::new();
    let mut bonds = Vec::new();

    let bw = opts.brick_width;
    let bh = opts.brick_height;
    let bd = opts.brick_depth;
    let volume = bw * bh * bd;

    // Grid node index lookup
    let idx = |col: u32, row: u32| -> u32 { row * opts.columns + col };

    // Create nodes
    for row in 0..opts.rows {
        let y = bh * 0.5 + row as f32 * bh;
        let x_offset = if opts.stagger && row % 2 == 1 {
            bw * 0.5
        } else {
            0.0
        };

        for col in 0..opts.columns {
            let x = col as f32 * bw + bw * 0.5 + x_offset
                - (opts.columns as f32 * bw) * 0.5;
            let mass = if row == 0 { 0.0 } else { volume * opts.density };
            nodes.push(ScenarioNode {
                centroid: Vec3::new(x, y, 0.0),
                mass,
                volume,
            });
        }
    }

    // Horizontal bonds (same row)
    for row in 0..opts.rows {
        for col in 0..opts.columns - 1 {
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

    // Vertical bonds (between rows)
    for row in 0..opts.rows - 1 {
        for col in 0..opts.columns {
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
