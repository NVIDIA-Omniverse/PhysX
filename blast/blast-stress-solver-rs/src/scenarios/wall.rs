use crate::types::*;

/// Options for building a wall scenario.
/// Matches the JS `WallScenarioOptions` from `wallScenario.ts`.
#[derive(Clone, Debug)]
pub struct WallOptions {
    pub span: f64,
    pub height: f64,
    pub thickness: f64,
    pub span_segments: u32,
    pub height_segments: u32,
    pub layers: u32,
    pub deck_mass: f64,
    pub area_scale: f64,
    pub add_diagonals: bool,
    pub diag_scale: f64,
    pub normalize_areas: bool,
    pub bonds_x: bool,
    pub bonds_y: bool,
    pub bonds_z: bool,
}

impl Default for WallOptions {
    fn default() -> Self {
        Self {
            span: 6.0,
            height: 3.0,
            thickness: 0.32,
            span_segments: 12,
            height_segments: 6,
            layers: 1,
            deck_mass: 10_000.0,
            area_scale: 0.05,
            add_diagonals: false,
            diag_scale: 0.75,
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

fn normalize(v: Vec3) -> Vec3 {
    let len = (v.x * v.x + v.y * v.y + v.z * v.z).sqrt();
    if len == 0.0 {
        Vec3::new(0.0, 0.0, 0.0)
    } else {
        Vec3::new(v.x / len, v.y / len, v.z / len)
    }
}

/// Pick the dominant axis of a normal vector (largest absolute component).
/// Returns 0 for x, 1 for y, 2 for z.
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

/// Build a wall scenario matching the JS `buildWallScenario` exactly.
pub fn build_wall_scenario(opts: &WallOptions) -> ScenarioDesc {
    let span = opts.span;
    let height = opts.height;
    let thickness = opts.thickness;
    let span_segments = opts.span_segments;
    let height_segments = opts.height_segments;
    let layers = opts.layers;
    let deck_mass = opts.deck_mass;
    let area_scale = opts.area_scale;
    let add_diagonals = opts.add_diagonals;
    let diag_scale = opts.diag_scale;
    let normalize_areas = opts.normalize_areas;
    let bonds_x = opts.bonds_x;
    let bonds_y = opts.bonds_y;
    let bonds_z = opts.bonds_z;

    let mut nodes: Vec<ScenarioNode> = Vec::new();
    let mut bonds: Vec<ScenarioBond> = Vec::new();

    let cell_x = span / (span_segments.max(1) as f64);
    let cell_y = height / (height_segments.max(1) as f64);
    let cell_z = thickness / (layers.max(1) as f64);

    let origin_x = -span * 0.5 + 0.5 * cell_x;
    let origin_y = 0.0 + 0.5 * cell_y;
    let origin_z = 0.0;

    let total_nodes = span_segments * height_segments * layers;
    let mass_per_node = deck_mass / (total_nodes.max(1) as f64);
    let volume_per_node = cell_x * cell_y * cell_z;

    // index3D[ix][iy][iz] -> node index
    let mut index_3d: Vec<Vec<Vec<usize>>> = vec![
        vec![vec![0usize; layers as usize]; height_segments as usize];
        span_segments as usize
    ];

    // Node creation: ix outer, iy middle, iz inner (matches JS)
    for ix in 0..span_segments {
        for iy in 0..height_segments {
            for iz in 0..layers {
                let centroid = Vec3::new(
                    (origin_x + ix as f64 * cell_x) as f32,
                    (origin_y + iy as f64 * cell_y) as f32,
                    (origin_z + (iz as f64 - (layers - 1) as f64 * 0.5) * cell_z) as f32,
                );
                let is_support = iy == 0;
                let index = nodes.len();
                nodes.push(ScenarioNode {
                    centroid,
                    mass: if is_support { 0.0 } else { mass_per_node as f32 },
                    volume: if is_support { 0.0 } else { volume_per_node as f32 },
                });
                index_3d[ix as usize][iy as usize][iz as usize] = index;
            }
        }
    }

    let area_x = (cell_y * cell_z * area_scale) as f32;
    let area_y = (cell_x * cell_z * area_scale) as f32;
    let area_z = (cell_x * cell_y * area_scale) as f32;

    let add_bond = |bonds: &mut Vec<ScenarioBond>, a: usize, b: usize, area: f32| {
        let na = &nodes[a];
        let nb = &nodes[b];
        let centroid = Vec3::new(
            (na.centroid.x + nb.centroid.x) * 0.5,
            (na.centroid.y + nb.centroid.y) * 0.5,
            (na.centroid.z + nb.centroid.z) * 0.5,
        );
        let normal = normalize(sub(nb.centroid, na.centroid));
        bonds.push(ScenarioBond {
            node0: a as u32,
            node1: b as u32,
            centroid,
            normal,
            area: area.max(1e-8),
        });
    };

    for ix in 0..span_segments {
        for iy in 0..height_segments {
            for iz in 0..layers {
                let current = index_3d[ix as usize][iy as usize][iz as usize];
                if bonds_x && ix + 1 < span_segments {
                    let neighbor = index_3d[(ix + 1) as usize][iy as usize][iz as usize];
                    add_bond(&mut bonds, current, neighbor, area_x);
                }
                if bonds_y && iy + 1 < height_segments {
                    let neighbor = index_3d[ix as usize][(iy + 1) as usize][iz as usize];
                    add_bond(&mut bonds, current, neighbor, area_y);
                }
                if bonds_z && iz + 1 < layers {
                    let neighbor = index_3d[ix as usize][iy as usize][(iz + 1) as usize];
                    add_bond(&mut bonds, current, neighbor, area_z);
                }
                if add_diagonals {
                    if ix + 1 < span_segments && iy + 1 < height_segments {
                        let neighbor =
                            index_3d[(ix + 1) as usize][(iy + 1) as usize][iz as usize];
                        add_bond(
                            &mut bonds,
                            current,
                            neighbor,
                            0.5 * (area_x + area_y) * diag_scale as f32,
                        );
                    }
                    if ix + 1 < span_segments && iz + 1 < layers {
                        let neighbor =
                            index_3d[(ix + 1) as usize][iy as usize][(iz + 1) as usize];
                        add_bond(
                            &mut bonds,
                            current,
                            neighbor,
                            0.5 * (area_x + area_z) * diag_scale as f32,
                        );
                    }
                    if iy + 1 < height_segments && iz + 1 < layers {
                        let neighbor =
                            index_3d[ix as usize][(iy + 1) as usize][(iz + 1) as usize];
                        add_bond(
                            &mut bonds,
                            current,
                            neighbor,
                            0.5 * (area_y + area_z) * diag_scale as f32,
                        );
                    }
                }
            }
        }
    }

    // Area normalization — isotropic scaling using geometric mean of per-axis scales
    if normalize_areas && !bonds.is_empty() {
        let target = [
            (height * thickness) as f32, // x: height * thickness
            (span * thickness) as f32,   // y: span * thickness
            (span * height) as f32,      // z: span * height
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

    ScenarioDesc { nodes, bonds }
}
