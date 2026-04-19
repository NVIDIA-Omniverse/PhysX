use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::Mesh;
use bevy::render::render_resource::PrimitiveTopology;
use blast_stress_solver::authoring::{
    build_scenario_from_pieces, BondingMode, BondingOptions, ScenarioPiece,
};
use blast_stress_solver::rapier::{
    DebrisCleanupOptions, OptimizationMode, SmallBodyDampingOptions,
};
use blast_stress_solver::{
    ScenarioBond, ScenarioCollider, ScenarioDesc, ScenarioNode, Vec3 as SolverVec3,
};
use serde::Deserialize;

#[derive(Clone, Copy, Debug)]
pub enum EmbeddedSceneKey {
    FracturedWall,
    FracturedTower,
    FracturedBridge,
    BrickBuilding,
}

#[derive(Clone, Debug)]
pub struct SceneMeshAsset {
    pub positions: Vec<[f32; 3]>,
    pub normals: Vec<[f32; 3]>,
    pub indices: Vec<u32>,
}

impl SceneMeshAsset {
    pub fn to_bevy_mesh(&self) -> Mesh {
        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, self.positions.clone());
        mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, self.normals.clone());
        mesh.insert_indices(Indices::U32(self.indices.clone()));
        mesh
    }
}

#[derive(Clone, Debug)]
pub struct LoadedScenePack {
    pub title: String,
    pub camera_target: bevy::prelude::Vec3,
    pub camera_distance: f32,
    pub projectile_radius: f32,
    pub projectile_mass: f32,
    pub projectile_speed: f32,
    pub projectile_ttl_secs: f32,
    pub gravity: f32,
    pub material_scale: f32,
    pub skip_single_bodies: bool,
    pub small_body_damping: SmallBodyDampingOptions,
    pub debris_cleanup: DebrisCleanupOptions,
    pub scenario: ScenarioDesc,
    pub node_meshes: Vec<SceneMeshAsset>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ScenePackJson {
    version: u32,
    title: String,
    defaults: SceneDefaultsJson,
    scenario: ScenarioJson,
    node_meshes: Vec<NodeMeshJson>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct SceneDefaultsJson {
    camera: CameraDefaultsJson,
    projectile: ProjectileDefaultsJson,
    solver: SolverDefaultsJson,
    physics: PhysicsDefaultsJson,
    optimization: OptimizationDefaultsJson,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct CameraDefaultsJson {
    target: Vec3Json,
    distance: f32,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ProjectileDefaultsJson {
    radius: f32,
    mass: f32,
    speed: f32,
    ttl_ms: f32,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct SolverDefaultsJson {
    gravity: f32,
    material_scale: f32,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct PhysicsDefaultsJson {
    #[allow(dead_code)]
    friction: f32,
    #[allow(dead_code)]
    restitution: f32,
    #[allow(dead_code)]
    contact_force_scale: f32,
    #[serde(default)]
    skip_single_bodies: bool,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct OptimizationDefaultsJson {
    small_body_damping_mode: String,
    debris_cleanup_mode: String,
    debris_ttl_ms: f32,
    max_colliders_for_debris: usize,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct ScenarioJson {
    nodes: Vec<ScenarioNodeJson>,
    bonds: Vec<ScenarioBondJson>,
    node_sizes: Vec<Vec3Json>,
    node_colliders: Vec<NodeColliderJson>,
}

#[derive(Deserialize)]
struct ScenarioNodeJson {
    centroid: Vec3Json,
    mass: f32,
    volume: f32,
}

#[derive(Deserialize)]
struct ScenarioBondJson {
    node0: u32,
    node1: u32,
    centroid: Vec3Json,
    normal: Vec3Json,
    area: f32,
}

#[derive(Clone, Copy, Deserialize)]
struct Vec3Json {
    x: f32,
    y: f32,
    z: f32,
}

#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case")]
enum NodeColliderJson {
    Cuboid {
        #[serde(rename = "halfExtents")]
        half_extents: Vec3Json,
    },
    ConvexHull {
        points: Vec<f32>,
    },
}

#[derive(Deserialize)]
struct NodeMeshJson {
    positions: Vec<f32>,
    normals: Vec<f32>,
    indices: Vec<u32>,
}

pub fn load_embedded_scene_pack(key: EmbeddedSceneKey) -> Result<LoadedScenePack, String> {
    let payload = match key {
        EmbeddedSceneKey::FracturedWall => {
            include_str!("../assets/scenes/fractured-wall.json")
        }
        EmbeddedSceneKey::FracturedTower => {
            include_str!("../assets/scenes/fractured-tower.json")
        }
        EmbeddedSceneKey::FracturedBridge => {
            include_str!("../assets/scenes/fractured-bridge.json")
        }
        EmbeddedSceneKey::BrickBuilding => {
            include_str!("../assets/scenes/brick-building.json")
        }
    };

    let pack: ScenePackJson = serde_json::from_str(payload)
        .map_err(|error| format!("invalid scene pack JSON: {error}"))?;
    if pack.version != 1 {
        return Err(format!("unsupported scene pack version {}", pack.version));
    }

    if pack.scenario.nodes.len() != pack.node_meshes.len() {
        return Err(format!(
            "scene pack node/mesh count mismatch: {} nodes vs {} meshes",
            pack.scenario.nodes.len(),
            pack.node_meshes.len()
        ));
    }
    if pack.scenario.nodes.len() != pack.scenario.node_sizes.len() {
        return Err(format!(
            "scene pack node/size count mismatch: {} nodes vs {} sizes",
            pack.scenario.nodes.len(),
            pack.scenario.node_sizes.len()
        ));
    }
    if pack.scenario.nodes.len() != pack.scenario.node_colliders.len() {
        return Err(format!(
            "scene pack node/collider count mismatch: {} nodes vs {} colliders",
            pack.scenario.nodes.len(),
            pack.scenario.node_colliders.len()
        ));
    }

    let node_meshes = pack
        .node_meshes
        .iter()
        .map(parse_mesh_asset)
        .collect::<Result<Vec<_>, _>>()?;

    let scenario = rebuild_scenario_from_public_authoring(&pack, &node_meshes)?;
    if matches!(key, EmbeddedSceneKey::BrickBuilding) {
        // This pack was generated through the triangle-based auto-bonding path,
        // so it should continue to match the public Rust authoring API exactly.
        validate_rebuilt_bonds(&pack.scenario.bonds, &scenario.bonds)?;
    }

    Ok(LoadedScenePack {
        title: pack.title,
        camera_target: bevy::prelude::Vec3::new(
            pack.defaults.camera.target.x,
            pack.defaults.camera.target.y,
            pack.defaults.camera.target.z,
        ),
        camera_distance: pack.defaults.camera.distance,
        projectile_radius: pack.defaults.projectile.radius,
        projectile_mass: pack.defaults.projectile.mass,
        projectile_speed: pack.defaults.projectile.speed,
        projectile_ttl_secs: pack.defaults.projectile.ttl_ms / 1000.0,
        gravity: pack.defaults.solver.gravity,
        material_scale: pack.defaults.solver.material_scale,
        skip_single_bodies: pack.defaults.physics.skip_single_bodies,
        small_body_damping: SmallBodyDampingOptions {
            mode: parse_optimization_mode(&pack.defaults.optimization.small_body_damping_mode)?,
            collider_count_threshold: 3,
            min_linear_damping: 2.0,
            min_angular_damping: 2.0,
        },
        debris_cleanup: DebrisCleanupOptions {
            mode: parse_optimization_mode(&pack.defaults.optimization.debris_cleanup_mode)?,
            debris_ttl_secs: pack.defaults.optimization.debris_ttl_ms / 1000.0,
            max_colliders_for_debris: pack.defaults.optimization.max_colliders_for_debris,
        },
        scenario,
        node_meshes,
    })
}

fn parse_mesh_asset(mesh: &NodeMeshJson) -> Result<SceneMeshAsset, String> {
    let positions = triples(&mesh.positions, "positions")?;
    let normals = triples(&mesh.normals, "normals")?;
    if positions.len() != normals.len() {
        return Err(format!(
            "mesh vertex/normal count mismatch: {} positions vs {} normals",
            positions.len(),
            normals.len()
        ));
    }
    Ok(SceneMeshAsset {
        positions,
        normals,
        indices: mesh.indices.clone(),
    })
}

fn triples(values: &[f32], label: &str) -> Result<Vec<[f32; 3]>, String> {
    if values.len() % 3 != 0 {
        return Err(format!(
            "{label} length must be divisible by 3, got {}",
            values.len()
        ));
    }
    Ok(values
        .chunks_exact(3)
        .map(|chunk| [chunk[0], chunk[1], chunk[2]])
        .collect())
}

fn parse_node_collider(collider: &NodeColliderJson) -> Result<Option<ScenarioCollider>, String> {
    match collider {
        NodeColliderJson::Cuboid { half_extents } => Ok(Some(ScenarioCollider::Cuboid {
            half_extents: (*half_extents).into(),
        })),
        NodeColliderJson::ConvexHull { points } => {
            let hull_points = triples(points, "convex hull points")?
                .into_iter()
                .map(|point| SolverVec3::new(point[0], point[1], point[2]))
                .collect();
            Ok(Some(ScenarioCollider::ConvexHull {
                points: hull_points,
            }))
        }
    }
}

fn parse_optimization_mode(value: &str) -> Result<OptimizationMode, String> {
    match value {
        "off" => Ok(OptimizationMode::Off),
        "always" => Ok(OptimizationMode::Always),
        "afterGroundCollision" => Ok(OptimizationMode::AfterGroundCollision),
        _ => Err(format!("unsupported optimization mode: {value}")),
    }
}

fn rebuild_scenario_from_public_authoring(
    pack: &ScenePackJson,
    node_meshes: &[SceneMeshAsset],
) -> Result<ScenarioDesc, String> {
    let node_sizes: Vec<SolverVec3> = pack
        .scenario
        .node_sizes
        .iter()
        .copied()
        .map(Into::into)
        .collect();
    let collider_shapes = pack
        .scenario
        .node_colliders
        .iter()
        .map(parse_node_collider)
        .collect::<Result<Vec<_>, _>>()?;

    let pieces = pack
        .scenario
        .nodes
        .iter()
        .zip(node_meshes.iter())
        .zip(node_sizes.iter())
        .zip(collider_shapes.iter())
        .map(|(((node, mesh), node_size), collider_shape)| {
            let node_centroid: SolverVec3 = node.centroid.into();
            Ok(ScenarioPiece {
                node: ScenarioNode {
                    centroid: node_centroid,
                    mass: node.mass,
                    volume: node.volume,
                },
                triangles: triangles_from_mesh_asset(mesh, node_centroid)?,
                // Match the current JS auto-bonding path: all pieces participate.
                bondable: true,
                node_size: Some(*node_size),
                collider_shape: collider_shape.clone(),
            })
        })
        .collect::<Result<Vec<_>, String>>()?;

    build_scenario_from_pieces(
        &pieces,
        &BondingOptions {
            mode: BondingMode::Exact,
        },
    )
    .map_err(|error| format!("failed to rebuild bonds via public authoring API: {error}"))
}

fn triangles_from_mesh_asset(
    mesh: &SceneMeshAsset,
    translation: SolverVec3,
) -> Result<Vec<SolverVec3>, String> {
    if mesh.indices.len() % 3 != 0 {
        return Err(format!(
            "mesh index count must be divisible by 3, got {}",
            mesh.indices.len()
        ));
    }

    let mut triangles = Vec::with_capacity(mesh.indices.len());
    for &index in &mesh.indices {
        let position = mesh.positions.get(index as usize).ok_or_else(|| {
            format!(
                "mesh index {} out of range for {} positions",
                index,
                mesh.positions.len()
            )
        })?;
        triangles.push(SolverVec3::new(
            position[0] + translation.x,
            position[1] + translation.y,
            position[2] + translation.z,
        ));
    }
    Ok(triangles)
}

fn validate_rebuilt_bonds(
    expected: &[ScenarioBondJson],
    rebuilt: &[ScenarioBond],
) -> Result<(), String> {
    let mut expected_signatures = expected
        .iter()
        .map(BondSignature::from_json)
        .collect::<Vec<_>>();
    let mut rebuilt_signatures = rebuilt
        .iter()
        .map(BondSignature::from_runtime)
        .collect::<Vec<_>>();
    expected_signatures.sort_unstable();
    rebuilt_signatures.sort_unstable();

    if expected_signatures == rebuilt_signatures {
        return Ok(());
    }

    let expected_only = expected_signatures
        .iter()
        .find(|signature| !rebuilt_signatures.contains(signature))
        .copied();
    let rebuilt_only = rebuilt_signatures
        .iter()
        .find(|signature| !expected_signatures.contains(signature))
        .copied();

    Err(format!(
        "public authoring rebuilt a different bond graph than the embedded pack: expected {} bonds, rebuilt {}; expected_only={expected_only:?}; rebuilt_only={rebuilt_only:?}",
        expected.len(),
        rebuilt.len()
    ))
}

#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct BondSignature {
    node0: u32,
    node1: u32,
    centroid: [i64; 3],
    normal: [i64; 3],
    area: i64,
}

impl BondSignature {
    fn from_json(bond: &ScenarioBondJson) -> Self {
        Self::from_parts(
            bond.node0,
            bond.node1,
            SolverVec3::from(bond.centroid),
            SolverVec3::from(bond.normal),
            bond.area,
        )
    }

    fn from_runtime(bond: &ScenarioBond) -> Self {
        Self::from_parts(
            bond.node0,
            bond.node1,
            bond.centroid,
            bond.normal,
            bond.area,
        )
    }

    fn from_parts(
        node0: u32,
        node1: u32,
        centroid: SolverVec3,
        normal: SolverVec3,
        area: f32,
    ) -> Self {
        if node0 <= node1 {
            Self {
                node0,
                node1,
                centroid: quantize_vec3(centroid),
                normal: quantize_vec3(normal),
                area: quantize_scalar(area),
            }
        } else {
            Self {
                node0: node1,
                node1: node0,
                centroid: quantize_vec3(centroid),
                normal: quantize_vec3(SolverVec3::new(-normal.x, -normal.y, -normal.z)),
                area: quantize_scalar(area),
            }
        }
    }
}

fn quantize_vec3(value: SolverVec3) -> [i64; 3] {
    [
        quantize_scalar(value.x),
        quantize_scalar(value.y),
        quantize_scalar(value.z),
    ]
}

fn quantize_scalar(value: f32) -> i64 {
    (value * 100_000.0).round() as i64
}

impl From<Vec3Json> for SolverVec3 {
    fn from(value: Vec3Json) -> Self {
        SolverVec3::new(value.x, value.y, value.z)
    }
}

#[cfg(test)]
mod tests {
    use super::{load_embedded_scene_pack, EmbeddedSceneKey};

    #[test]
    fn fractured_wall_pack_loads() {
        let pack = load_embedded_scene_pack(EmbeddedSceneKey::FracturedWall)
            .expect("fractured wall pack should load");
        assert_eq!(pack.scenario.nodes.len(), pack.node_meshes.len());
        assert_eq!(
            pack.scenario.nodes.len(),
            pack.scenario.collider_shapes.len()
        );
        assert!(!pack.scenario.bonds.is_empty());
    }

    #[test]
    fn fractured_bridge_pack_contains_buildable_mesh() {
        let pack = load_embedded_scene_pack(EmbeddedSceneKey::FracturedBridge)
            .expect("fractured bridge pack should load");
        let mesh = pack.node_meshes[0].to_bevy_mesh();
        assert!(mesh.count_vertices() > 0);
    }

    #[test]
    fn brick_building_pack_loads() {
        let pack = load_embedded_scene_pack(EmbeddedSceneKey::BrickBuilding)
            .expect("brick building pack should load");
        assert_eq!(pack.scenario.nodes.len(), pack.node_meshes.len());
        assert_eq!(pack.scenario.nodes.len(), pack.scenario.node_sizes.len());
        assert_eq!(
            pack.scenario.nodes.len(),
            pack.scenario.collider_shapes.len()
        );
        assert!(!pack.scenario.bonds.is_empty());
    }
}
