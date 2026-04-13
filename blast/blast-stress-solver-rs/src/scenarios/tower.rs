use crate::types::*;

/// Options for building a tower scenario.
/// Matches the JS `TowerScenarioOptions` from `towerScenario.ts`.
#[derive(Clone, Debug)]
pub struct TowerOptions {
    pub side: u32,
    pub stories: u32,
    pub spacing_x: f64,
    pub spacing_y: f64,
    pub spacing_z: f64,
    pub total_mass: f64,
    pub area_scale: f64,
    pub add_diagonals: bool,
    pub diag_scale: f64,
    pub normalize_areas: bool,
}

impl Default for TowerOptions {
    fn default() -> Self {
        Self {
            side: 4,
            stories: 8,
            spacing_x: 0.5,
            spacing_y: 0.5,
            spacing_z: 0.5,
            total_mass: 5_000.0,
            area_scale: 0.05,
            add_diagonals: true,
            diag_scale: 0.55,
            normalize_areas: true,
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

/// Build a tower scenario matching the JS `buildTowerScenario` exactly.
pub fn build_tower_scenario(opts: &TowerOptions) -> ScenarioDesc {
    let side = opts.side;
    let stories = opts.stories;
    let spacing_x = opts.spacing_x;
    let spacing_y = opts.spacing_y;
    let spacing_z = opts.spacing_z;
    let total_mass = opts.total_mass;
    let area_scale = opts.area_scale;
    let add_diagonals = opts.add_diagonals;
    let diag_scale = opts.diag_scale;
    let normalize_areas = opts.normalize_areas;

    let mut nodes: Vec<ScenarioNode> = Vec::new();
    let mut bonds: Vec<ScenarioBond> = Vec::new();
    let mut node_sizes: Vec<Vec3> = Vec::new();

    let total_rows = stories + 1; // +1 for support row at bottom

    let dynamic_node_count = side * stories * side;
    let node_mass = total_mass / (dynamic_node_count.max(1) as f64);

    // idx(ix, iy, iz) = iz * side * totalRows + iy * side + ix
    let idx =
        |ix: u32, iy: u32, iz: u32| -> usize { (iz * side * total_rows + iy * side + ix) as usize };

    // Node creation: iz outer, iy middle, ix inner (matches JS)
    for iz in 0..side {
        for iy in 0..total_rows {
            for ix in 0..side {
                let is_support = iy == 0;
                let centroid = Vec3::new(
                    ((ix as f64 - (side - 1) as f64 / 2.0) * spacing_x) as f32,
                    ((iy as f64 - 1.0) * spacing_y) as f32,
                    ((iz as f64 - (side - 1) as f64 / 2.0) * spacing_z) as f32,
                );
                let volume = (spacing_x * spacing_y * spacing_z) as f32;
                nodes.push(ScenarioNode {
                    centroid,
                    mass: if is_support { 0.0 } else { node_mass as f32 },
                    volume: if is_support { 0.0 } else { volume },
                });
                node_sizes.push(Vec3::new(
                    spacing_x as f32,
                    spacing_y as f32,
                    spacing_z as f32,
                ));
            }
        }
    }

    let area_xy = (spacing_x * spacing_y * area_scale) as f32;
    let area_yz = (spacing_y * spacing_z * area_scale) as f32;
    let area_xz = (spacing_x * spacing_z * area_scale) as f32;

    // Axis-aligned bond offsets: [dx, dy, dz, normal, area]
    let offsets: [(i32, i32, i32, Vec3, f32); 3] = [
        (1, 0, 0, Vec3::new(1.0, 0.0, 0.0), area_yz),
        (0, 1, 0, Vec3::new(0.0, 1.0, 0.0), area_xz),
        (0, 0, 1, Vec3::new(0.0, 0.0, 1.0), area_xy),
    ];

    for iz in 0..side {
        for iy in 0..total_rows {
            for ix in 0..side {
                let i = idx(ix, iy, iz);
                for &(dx, dy, dz, normal, area) in &offsets {
                    let nx = ix as i32 + dx;
                    let ny = iy as i32 + dy;
                    let nz = iz as i32 + dz;
                    if nx < side as i32 && ny < total_rows as i32 && nz < side as i32 {
                        let j = idx(nx as u32, ny as u32, nz as u32);
                        let c0 = nodes[i].centroid;
                        let c1 = nodes[j].centroid;
                        bonds.push(ScenarioBond {
                            node0: i as u32,
                            node1: j as u32,
                            centroid: Vec3::new(
                                (c0.x + c1.x) / 2.0,
                                (c0.y + c1.y) / 2.0,
                                (c0.z + c1.z) / 2.0,
                            ),
                            normal,
                            area,
                        });
                    }
                }

                if add_diagonals {
                    let diag_area = 0.5 * (area_xz + area_yz) * diag_scale as f32;
                    let diag_offsets: [(i32, i32, i32); 4] =
                        [(1, 1, 0), (1, -1, 0), (0, 1, 1), (0, -1, 1)];
                    for &(ddx, ddy, ddz) in &diag_offsets {
                        let nx = ix as i32 + ddx;
                        let ny = iy as i32 + ddy;
                        let nz = iz as i32 + ddz;
                        if nx >= 0
                            && nx < side as i32
                            && ny >= 0
                            && ny < total_rows as i32
                            && nz >= 0
                            && nz < side as i32
                        {
                            let j = idx(nx as u32, ny as u32, nz as u32);
                            let c0 = nodes[i].centroid;
                            let c1 = nodes[j].centroid;
                            let d = sub(c1, c0);
                            let n = normalize(d);
                            bonds.push(ScenarioBond {
                                node0: i as u32,
                                node1: j as u32,
                                centroid: Vec3::new(
                                    (c0.x + c1.x) / 2.0,
                                    (c0.y + c1.y) / 2.0,
                                    (c0.z + c1.z) / 2.0,
                                ),
                                normal: n,
                                area: diag_area,
                            });
                        }
                    }
                }
            }
        }
    }

    // Area normalization — isotropic scaling using geometric mean of per-axis scales
    if normalize_areas && !bonds.is_empty() {
        let total_height = (stories as f64 * spacing_y) as f32;
        let total_width = (side as f64 * spacing_x) as f32;
        let total_depth = (side as f64 * spacing_z) as f32;
        let target = [
            total_height * total_depth, // x
            total_width * total_depth,  // y
            total_width * total_height, // z
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
        collider_shapes: Vec::new(),
    }
}
