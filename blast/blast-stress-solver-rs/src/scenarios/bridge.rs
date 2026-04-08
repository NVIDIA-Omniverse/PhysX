use crate::types::*;

/// Options for building a beam bridge scenario.
#[derive(Clone, Debug)]
pub struct BridgeOptions {
    /// Number of segments along the bridge span.
    pub segments: u32,
    pub segment_width: f32,
    pub segment_height: f32,
    pub segment_depth: f32,
    pub density: f32,
}

impl Default for BridgeOptions {
    fn default() -> Self {
        Self {
            segments: 8,
            segment_width: 1.0,
            segment_height: 0.5,
            segment_depth: 1.0,
            density: 1.0,
        }
    }
}

/// Build a beam bridge scenario.
///
/// The bridge is a horizontal beam of `segments` blocks. The first and last blocks
/// are support nodes (mass=0, anchored). All other blocks are dynamic.
/// Bonds connect adjacent segments along the X axis.
pub fn build_bridge_scenario(opts: &BridgeOptions) -> ScenarioDesc {
    let mut nodes = Vec::new();
    let mut bonds = Vec::new();

    let sw = opts.segment_width;
    let sh = opts.segment_height;
    let sd = opts.segment_depth;
    let volume = sw * sh * sd;

    for i in 0..opts.segments {
        let x = (i as f32 + 0.5) * sw - (opts.segments as f32 * sw) * 0.5;
        let y = sh * 0.5;
        let is_support = i == 0 || i == opts.segments - 1;
        let mass = if is_support { 0.0 } else { volume * opts.density };
        nodes.push(ScenarioNode {
            centroid: Vec3::new(x, y, 0.0),
            mass,
            volume,
        });
    }

    for i in 0..opts.segments - 1 {
        let c0 = nodes[i as usize].centroid;
        let c1 = nodes[(i + 1) as usize].centroid;
        bonds.push(ScenarioBond {
            node0: i,
            node1: i + 1,
            centroid: (c0 + c1) * 0.5,
            normal: Vec3::new(1.0, 0.0, 0.0),
            area: sh * sd,
        });
    }

    ScenarioDesc { nodes, bonds }
}
