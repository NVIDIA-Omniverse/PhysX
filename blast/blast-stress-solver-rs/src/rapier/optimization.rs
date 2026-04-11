use rapier3d::prelude::RigidBodyHandle;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OptimizationMode {
    Off,
    Always,
    AfterGroundCollision,
}

impl Default for OptimizationMode {
    fn default() -> Self {
        Self::Off
    }
}

#[derive(Clone, Copy, Debug)]
pub struct SmallBodyDampingOptions {
    pub mode: OptimizationMode,
    pub collider_count_threshold: usize,
    pub min_linear_damping: f32,
    pub min_angular_damping: f32,
}

impl Default for SmallBodyDampingOptions {
    fn default() -> Self {
        Self {
            mode: OptimizationMode::Off,
            collider_count_threshold: 3,
            min_linear_damping: 2.0,
            min_angular_damping: 2.0,
        }
    }
}

#[derive(Clone, Copy, Debug)]
pub struct DebrisCleanupOptions {
    pub mode: OptimizationMode,
    pub debris_ttl_secs: f32,
    pub max_colliders_for_debris: usize,
}

impl Default for DebrisCleanupOptions {
    fn default() -> Self {
        Self {
            mode: OptimizationMode::Off,
            debris_ttl_secs: 10.0,
            max_colliders_for_debris: 2,
        }
    }
}

#[derive(Clone, Debug, Default)]
pub struct OptimizationResult {
    pub damped_bodies: Vec<RigidBodyHandle>,
    pub removed_bodies: Vec<RigidBodyHandle>,
    pub removed_nodes: Vec<u32>,
}
