use bevy::asset::RenderAssetUsages;
use bevy::mesh::Indices;
use bevy::prelude::Mesh;
use bevy::render::render_resource::PrimitiveTopology;
use blast_stress_solver::rapier::{
    DebrisCleanupOptions, DebrisCollisionMode, OptimizationMode, SmallBodyDampingOptions,
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
    pub debris_collision_mode: DebrisCollisionMode,
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
    debris_collision_mode: String,
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
        debris_collision_mode: parse_debris_collision_mode(
            &pack.defaults.physics.debris_collision_mode,
        )?,
        scenario: ScenarioDesc {
            nodes: pack
                .scenario
                .nodes
                .iter()
                .map(|node| ScenarioNode {
                    centroid: node.centroid.into(),
                    mass: node.mass,
                    volume: node.volume,
                })
                .collect(),
            bonds: pack
                .scenario
                .bonds
                .iter()
                .map(|bond| ScenarioBond {
                    node0: bond.node0,
                    node1: bond.node1,
                    centroid: bond.centroid.into(),
                    normal: bond.normal.into(),
                    area: bond.area,
                })
                .collect(),
            node_sizes: pack
                .scenario
                .node_sizes
                .iter()
                .copied()
                .map(Into::into)
                .collect(),
            collider_shapes: pack
                .scenario
                .node_colliders
                .iter()
                .map(parse_node_collider)
                .collect::<Result<Vec<_>, _>>()?,
        },
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

fn parse_debris_collision_mode(value: &str) -> Result<DebrisCollisionMode, String> {
    match value {
        "all" => Ok(DebrisCollisionMode::All),
        "noDebrisPairs" => Ok(DebrisCollisionMode::NoDebrisPairs),
        "debrisGroundOnly" => Ok(DebrisCollisionMode::DebrisGroundOnly),
        "debrisNone" => Ok(DebrisCollisionMode::DebrisNone),
        _ => Err(format!("unsupported debris collision mode: {value}")),
    }
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
}
