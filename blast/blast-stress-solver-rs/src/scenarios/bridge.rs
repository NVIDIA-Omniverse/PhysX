use crate::types::*;
use std::collections::{HashMap, HashSet};

const EPS: f32 = 1e-8;

/// Options for building a beam bridge scenario.
/// Matches the JS `BeamBridgeOptions` from `bridgeScenario.ts`.
#[derive(Clone, Debug)]
pub struct BridgeOptions {
    pub span: f64,
    pub deck_width: f64,
    pub deck_thickness: f64,
    pub span_segments: u32,
    pub width_segments: u32,
    pub thickness_layers: u32,
    pub deck_mass: f64,
    pub pier_height: f64,
    pub supports_per_side: u32,
    pub support_width_segments: u32,
    pub support_depth_segments: u32,
    pub footing_thickness: f64,
    pub area_scale: f64,
    pub add_diagonals: bool,
    pub diag_scale: f64,
    pub normalize_areas: bool,
    pub bonds_x: bool,
    pub bonds_y: bool,
    pub bonds_z: bool,
}

impl Default for BridgeOptions {
    fn default() -> Self {
        Self {
            span: 18.0,
            deck_width: 5.0,
            deck_thickness: 0.6,
            span_segments: 30,
            width_segments: 10,
            thickness_layers: 2,
            deck_mass: 60_000.0,
            pier_height: 2.8,
            supports_per_side: 4,
            support_width_segments: 2,
            support_depth_segments: 2,
            footing_thickness: 0.12,
            area_scale: 0.05,
            add_diagonals: true,
            diag_scale: 0.6,
            normalize_areas: true,
            bonds_x: true,
            bonds_y: true,
            bonds_z: true,
        }
    }
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.x - b.x, a.y - b.y, a.z - b.z)
}

fn nrm(p: Vec3) -> Vec3 {
    let l = (p.x * p.x + p.y * p.y + p.z * p.z).sqrt();
    if l <= EPS {
        Vec3::new(0.0, 0.0, 0.0)
    } else {
        Vec3::new(p.x / l, p.y / l, p.z / l)
    }
}

/// Pick the dominant axis of a normal vector (largest absolute component).
/// Matches the JS `pick` function: ax >= ay && ax >= az ? 'x' : (ay >= az ? 'y' : 'z')
fn pick_axis(n: &Vec3) -> usize {
    let ax = n.x.abs();
    let ay = n.y.abs();
    let az = n.z.abs();
    if ax >= ay && ax >= az {
        0
    } else if ay >= az {
        1
    } else {
        2
    }
}

fn clamp(val: i32, lo: i32, hi: i32) -> i32 {
    val.max(lo).min(hi)
}

fn uniq(arr: &[i32]) -> Vec<i32> {
    let mut seen = HashSet::new();
    let mut result = Vec::new();
    for &v in arr {
        if seen.insert(v) {
            result.push(v);
        }
    }
    result
}

/// Build a beam bridge scenario matching the JS `buildBeamBridgeScenario` exactly.
pub fn build_bridge_scenario(opts: &BridgeOptions) -> ScenarioDesc {
    let span = opts.span;
    let deck_width = opts.deck_width;
    let deck_thickness = opts.deck_thickness;
    let raw_seg_x = opts.span_segments;
    let raw_seg_z = opts.width_segments;
    let raw_seg_y = opts.thickness_layers;
    let deck_mass = opts.deck_mass;
    let pier_height = opts.pier_height;
    let supports_per_side = opts.supports_per_side;
    let support_width_segments = opts.support_width_segments;
    let support_depth_segments = opts.support_depth_segments;
    let footing_thickness = opts.footing_thickness;
    let area_scale = opts.area_scale;
    let add_diagonals = opts.add_diagonals;
    let diag_scale = opts.diag_scale;
    let normalize_areas = opts.normalize_areas;
    let bonds_x = opts.bonds_x;
    let bonds_y = opts.bonds_y;
    let bonds_z = opts.bonds_z;

    let seg_x = (raw_seg_x as i32).max(1) as u32;
    let seg_y = (raw_seg_y as i32).max(1) as u32;
    let seg_z = (raw_seg_z as i32).max(1) as u32;

    let cell_x = span / seg_x as f64;
    let cell_y = deck_thickness / seg_y as f64;
    let cell_z = deck_width / seg_z as f64;

    let post_layers = (pier_height / cell_y).ceil().max(1.0) as u32;
    let deck_bottom_y = post_layers as f64 * cell_y;
    let deck_origin_x = -span * 0.5 + 0.5 * cell_x;
    let deck_origin_y = deck_bottom_y + 0.5 * cell_y;
    let deck_origin_z = -deck_width * 0.5 + 0.5 * cell_z;

    // gridDeck[ix][iy][iz] -> node index (-1 means empty)
    let mut grid_deck: Vec<Vec<Vec<i32>>> =
        vec![vec![vec![-1i32; seg_z as usize]; seg_y as usize]; seg_x as usize];

    let mut nodes: Vec<ScenarioNode> = Vec::new();
    let mut bonds: Vec<ScenarioBond> = Vec::new();
    let mut node_sizes: Vec<Vec3> = Vec::new();

    // Build deck nodes
    let deck_cell_volume = (cell_x * cell_y * cell_z) as f32;
    let mut deck_total_volume: f64 = 0.0;
    for ix in 0..seg_x {
        for iy in 0..seg_y {
            for iz in 0..seg_z {
                let px = deck_origin_x + ix as f64 * cell_x;
                let py = deck_origin_y + iy as f64 * cell_y;
                let pz = deck_origin_z + iz as f64 * cell_z;
                let node_idx = nodes.len() as i32;
                nodes.push(ScenarioNode {
                    centroid: Vec3::new(px as f32, py as f32, pz as f32),
                    mass: deck_cell_volume,
                    volume: deck_cell_volume,
                });
                node_sizes.push(Vec3::new(cell_x as f32, cell_y as f32, cell_z as f32));
                grid_deck[ix as usize][iy as usize][iz as usize] = node_idx;
                deck_total_volume += deck_cell_volume as f64;
            }
        }
    }

    // Scale masses so total deck mass matches
    let mass_scale: f64 = if deck_total_volume > 0.0 {
        deck_mass / deck_total_volume
    } else {
        0.0
    };
    if mass_scale != 1.0 {
        for n in nodes.iter_mut() {
            if n.volume > 0.0 {
                n.mass = n.volume * mass_scale as f32;
            }
        }
    }

    // Bond helpers
    let area_x = (cell_y * cell_z * area_scale) as f32;
    let area_y = (cell_x * cell_z * area_scale) as f32;
    let area_z = (cell_x * cell_y * area_scale) as f32;

    // Closure to add a bond between two node indices
    // We use a macro-like approach since closures borrowing nodes and bonds simultaneously is tricky
    let add_bond =
        |bonds: &mut Vec<ScenarioBond>, nodes: &[ScenarioNode], a: i32, b: i32, area: f32| {
            if a < 0 || b < 0 {
                return;
            }
            let na = &nodes[a as usize];
            let nb = &nodes[b as usize];
            let c = Vec3::new(
                (na.centroid.x + nb.centroid.x) * 0.5,
                (na.centroid.y + nb.centroid.y) * 0.5,
                (na.centroid.z + nb.centroid.z) * 0.5,
            );
            let n = nrm(sub(nb.centroid, na.centroid));
            bonds.push(ScenarioBond {
                node0: a as u32,
                node1: b as u32,
                centroid: c,
                normal: n,
                area: area.max(EPS),
            });
        };

    // Deck connectivity
    for ix in 0..seg_x {
        for iy in 0..seg_y {
            for iz in 0..seg_z {
                let cur = grid_deck[ix as usize][iy as usize][iz as usize];
                if cur < 0 {
                    continue;
                }
                if bonds_x && ix + 1 < seg_x {
                    add_bond(
                        &mut bonds,
                        &nodes,
                        cur,
                        grid_deck[(ix + 1) as usize][iy as usize][iz as usize],
                        area_x,
                    );
                }
                if bonds_y && iy + 1 < seg_y {
                    add_bond(
                        &mut bonds,
                        &nodes,
                        cur,
                        grid_deck[ix as usize][(iy + 1) as usize][iz as usize],
                        area_y,
                    );
                }
                if bonds_z && iz + 1 < seg_z {
                    add_bond(
                        &mut bonds,
                        &nodes,
                        cur,
                        grid_deck[ix as usize][iy as usize][(iz + 1) as usize],
                        area_z,
                    );
                }
                if add_diagonals {
                    if bonds_x && bonds_z && ix + 1 < seg_x && iz + 1 < seg_z {
                        add_bond(
                            &mut bonds,
                            &nodes,
                            cur,
                            grid_deck[(ix + 1) as usize][iy as usize][(iz + 1) as usize],
                            0.5 * (area_x + area_z) * diag_scale as f32,
                        );
                    }
                    if bonds_x && bonds_y && ix + 1 < seg_x && iy + 1 < seg_y {
                        add_bond(
                            &mut bonds,
                            &nodes,
                            cur,
                            grid_deck[(ix + 1) as usize][(iy + 1) as usize][iz as usize],
                            0.5 * (area_x + area_y) * diag_scale as f32,
                        );
                    }
                    if bonds_y && bonds_z && iy + 1 < seg_y && iz + 1 < seg_z {
                        add_bond(
                            &mut bonds,
                            &nodes,
                            cur,
                            grid_deck[ix as usize][(iy + 1) as usize][(iz + 1) as usize],
                            0.5 * (area_y + area_z) * diag_scale as f32,
                        );
                    }
                }
            }
        }
    }

    // Build posts under first and last span columns
    let post_x_cols: [u32; 2] = [0, seg_x - 1];
    let post_top_y_layer: u32 = 0;
    let post_top_y = deck_origin_y - 0.5 * cell_y;

    let post_span = (supports_per_side as i32).max(1);
    let mut slots: Vec<i32> = Vec::new();
    for i in 0..post_span {
        let t = if post_span == 1 {
            0.5
        } else {
            i as f64 / (post_span - 1) as f64
        };
        let val = (t * (seg_z as f64 - 1.0)).round() as i32;
        slots.push(clamp(val, 0, seg_z as i32 - 1));
    }

    for &ix_edge in &post_x_cols {
        let ix_cover_raw: Vec<i32> = (0..support_depth_segments as i32)
            .map(|k| {
                if ix_edge == 0 {
                    clamp(ix_edge as i32 + k, 0, seg_x as i32 - 1)
                } else {
                    clamp(ix_edge as i32 - k, 0, seg_x as i32 - 1)
                }
            })
            .collect();
        let ix_cover = uniq(&ix_cover_raw);
        let ix_cover_set: HashSet<i32> = ix_cover.iter().copied().collect();

        for &base_z in &slots {
            let cover_z_raw: Vec<i32> = (0..support_width_segments as i32)
                .map(|k| {
                    clamp(
                        base_z + k - ((support_width_segments as i32 - 1) / 2),
                        0,
                        seg_z as i32 - 1,
                    )
                })
                .collect();
            let cover_z = uniq(&cover_z_raw);
            let cover_z_set: HashSet<i32> = cover_z.iter().copied().collect();
            let mut post_map: HashMap<(i32, i32, i32), i32> = HashMap::new();

            // Create stacks
            for &iz in &cover_z {
                for &ixp in &ix_cover {
                    for py in 0..post_layers as i32 {
                        let y_center = post_top_y - py as f64 * cell_y - 0.5 * cell_y;
                        let node_idx = nodes.len() as i32;
                        let px = deck_origin_x + ixp as f64 * cell_x;
                        let pz = deck_origin_z + iz as f64 * cell_z;
                        let volume = (cell_x * cell_y * cell_z) as f32;
                        nodes.push(ScenarioNode {
                            centroid: Vec3::new(px as f32, y_center as f32, pz as f32),
                            mass: volume * mass_scale as f32,
                            volume,
                        });
                        node_sizes.push(Vec3::new(cell_x as f32, cell_y as f32, cell_z as f32));
                        post_map.insert((ixp, py, iz), node_idx);

                        if py > 0 {
                            if let Some(&prev_idx) = post_map.get(&(ixp, py - 1, iz)) {
                                add_bond(&mut bonds, &nodes, prev_idx, node_idx, area_y);
                            }
                        } else {
                            let deck_index =
                                grid_deck[ixp as usize][post_top_y_layer as usize][iz as usize];
                            add_bond(&mut bonds, &nodes, node_idx, deck_index, area_y);
                        }
                    }

                    // Footing under this column (mass=0 support)
                    let foot_center_y =
                        post_top_y - post_layers as f64 * cell_y - 0.5 * footing_thickness;
                    let f_idx = nodes.len() as i32;
                    let f_px = deck_origin_x + ixp as f64 * cell_x;
                    let f_pz = deck_origin_z + iz as f64 * cell_z;
                    nodes.push(ScenarioNode {
                        centroid: Vec3::new(f_px as f32, foot_center_y as f32, f_pz as f32),
                        mass: 0.0,
                        volume: 0.0,
                    });
                    node_sizes.push(Vec3::new(
                        cell_x as f32,
                        footing_thickness as f32,
                        cell_z as f32,
                    ));
                    if let Some(&lowest_post_idx) = post_map.get(&(ixp, post_layers as i32 - 1, iz))
                    {
                        add_bond(&mut bonds, &nodes, f_idx, lowest_post_idx, area_y);
                    }
                }
            }

            // Lateral bonds within the post cluster
            for &iz in &cover_z {
                for &ixp in &ix_cover {
                    for py in 0..post_layers as i32 {
                        let cur = match post_map.get(&(ixp, py, iz)) {
                            Some(&v) => v,
                            None => continue,
                        };
                        let nx = if ix_edge == 0 { ixp + 1 } else { ixp - 1 };
                        if ix_cover_set.contains(&nx) {
                            if let Some(&nb) = post_map.get(&(nx, py, iz)) {
                                add_bond(&mut bonds, &nodes, cur, nb, area_x);
                            }
                        }
                        let nz = iz + 1;
                        if cover_z_set.contains(&nz) {
                            if let Some(&nbz) = post_map.get(&(ixp, py, nz)) {
                                add_bond(&mut bonds, &nodes, cur, nbz, area_z);
                            }
                        }
                    }
                }
            }
        }
    }

    // Isotropic area normalization — apply uniform scale factor (geometric mean
    // of per-axis scales) to avoid directional bond strength bias.
    if normalize_areas && !bonds.is_empty() {
        let size_x = span as f32;
        let size_y = (deck_thickness + pier_height + footing_thickness) as f32;
        let size_z = deck_width as f32;
        let target = [
            size_y * size_z, // x
            size_x * size_z, // y
            size_x * size_y, // z
        ];
        let mut sum = [0.0f32; 3];
        for b in bonds.iter() {
            let axis = pick_axis(&b.normal);
            sum[axis] += b.area;
        }
        let mut axis_scales: Vec<f32> = Vec::new();
        for k in 0..3 {
            if sum[k] > 0.0 {
                axis_scales.push(target[k] / sum[k]);
            }
        }
        let uniform_scale = if !axis_scales.is_empty() {
            let product: f32 = axis_scales.iter().copied().product();
            product.powf(1.0 / axis_scales.len() as f32)
        } else {
            1.0
        };
        for b in bonds.iter_mut() {
            b.area *= uniform_scale;
        }
    }

    ScenarioDesc {
        nodes,
        bonds,
        node_sizes,
    }
}
