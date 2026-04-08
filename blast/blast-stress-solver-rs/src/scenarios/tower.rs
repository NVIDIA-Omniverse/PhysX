use crate::types::*;

/// Options for building a tower scenario.
#[derive(Clone, Debug)]
pub struct TowerOptions {
    pub width: u32,
    pub depth: u32,
    pub height: u32,
    pub block_size: f32,
    pub density: f32,
}

impl Default for TowerOptions {
    fn default() -> Self {
        Self {
            width: 3,
            depth: 3,
            height: 6,
            block_size: 0.5,
            density: 1.0,
        }
    }
}

/// Build a tower scenario: a 3D grid of blocks stacked vertically.
/// The bottom layer (y=0) blocks are support nodes (mass=0).
pub fn build_tower_scenario(opts: &TowerOptions) -> ScenarioDesc {
    let mut nodes = Vec::new();
    let mut bonds = Vec::new();

    let s = opts.block_size;
    let volume = s * s * s;
    let face_area = s * s;

    let idx = |ix: u32, iy: u32, iz: u32| -> u32 {
        iy * opts.width * opts.depth + iz * opts.width + ix
    };

    // Create nodes
    for iy in 0..opts.height {
        for iz in 0..opts.depth {
            for ix in 0..opts.width {
                let x = (ix as f32 + 0.5) * s - (opts.width as f32 * s) * 0.5;
                let y = (iy as f32 + 0.5) * s;
                let z = (iz as f32 + 0.5) * s - (opts.depth as f32 * s) * 0.5;
                let mass = if iy == 0 { 0.0 } else { volume * opts.density };
                nodes.push(ScenarioNode {
                    centroid: Vec3::new(x, y, z),
                    mass,
                    volume,
                });
            }
        }
    }

    // X-axis bonds
    for iy in 0..opts.height {
        for iz in 0..opts.depth {
            for ix in 0..opts.width - 1 {
                let n0 = idx(ix, iy, iz);
                let n1 = idx(ix + 1, iy, iz);
                let c0 = nodes[n0 as usize].centroid;
                let c1 = nodes[n1 as usize].centroid;
                bonds.push(ScenarioBond {
                    node0: n0,
                    node1: n1,
                    centroid: (c0 + c1) * 0.5,
                    normal: Vec3::new(1.0, 0.0, 0.0),
                    area: face_area,
                });
            }
        }
    }

    // Y-axis bonds (vertical)
    for iy in 0..opts.height - 1 {
        for iz in 0..opts.depth {
            for ix in 0..opts.width {
                let n0 = idx(ix, iy, iz);
                let n1 = idx(ix, iy + 1, iz);
                let c0 = nodes[n0 as usize].centroid;
                let c1 = nodes[n1 as usize].centroid;
                bonds.push(ScenarioBond {
                    node0: n0,
                    node1: n1,
                    centroid: (c0 + c1) * 0.5,
                    normal: Vec3::new(0.0, 1.0, 0.0),
                    area: face_area,
                });
            }
        }
    }

    // Z-axis bonds
    for iy in 0..opts.height {
        for iz in 0..opts.depth - 1 {
            for ix in 0..opts.width {
                let n0 = idx(ix, iy, iz);
                let n1 = idx(ix, iy, iz + 1);
                let c0 = nodes[n0 as usize].centroid;
                let c1 = nodes[n1 as usize].centroid;
                bonds.push(ScenarioBond {
                    node0: n0,
                    node1: n1,
                    centroid: (c0 + c1) * 0.5,
                    normal: Vec3::new(0.0, 0.0, 1.0),
                    area: face_area,
                });
            }
        }
    }

    ScenarioDesc { nodes, bonds }
}
