use std::fmt;
use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign, Neg, Sub, SubAssign};

/// 3D vector used for positions, forces, torques, and impulses.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }

    pub fn cross(self, other: Self) -> Self {
        Self::new(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )
    }

    pub fn magnitude_squared(self) -> f32 {
        self.dot(self)
    }

    pub fn magnitude(self) -> f32 {
        self.magnitude_squared().sqrt()
    }

    pub fn normalize(self) -> Self {
        let mag = self.magnitude();
        if mag > 0.0 {
            self / mag
        } else {
            Self::default()
        }
    }
}

impl Add for Vec3 {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for Vec3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for Vec3 {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for Vec3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Mul<f32> for Vec3 {
    type Output = Self;
    fn mul(self, rhs: f32) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl MulAssign<f32> for Vec3 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl Div<f32> for Vec3 {
    type Output = Self;
    fn div(self, rhs: f32) -> Self {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl DivAssign<f32> for Vec3 {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

impl Neg for Vec3 {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

/// Node descriptor for the extended stress solver.
/// A node with `mass == 0.0` acts as a fixed support.
#[derive(Clone, Copy, Debug, Default)]
pub struct NodeDesc {
    pub centroid: Vec3,
    pub mass: f32,
    pub volume: f32,
}

/// Bond descriptor connecting two nodes.
#[derive(Clone, Copy, Debug, Default)]
pub struct BondDesc {
    pub centroid: Vec3,
    pub normal: Vec3,
    pub area: f32,
    pub node0: u32,
    pub node1: u32,
}

/// Settings for the extended stress solver.
#[derive(Clone, Copy, Debug)]
pub struct SolverSettings {
    pub max_solver_iterations_per_frame: u32,
    pub graph_reduction_level: u32,
    pub compression_elastic_limit: f32,
    pub compression_fatal_limit: f32,
    pub tension_elastic_limit: f32,
    pub tension_fatal_limit: f32,
    pub shear_elastic_limit: f32,
    pub shear_fatal_limit: f32,
}

impl Default for SolverSettings {
    fn default() -> Self {
        Self {
            max_solver_iterations_per_frame: 32,
            graph_reduction_level: 0,
            compression_elastic_limit: 1.0,
            compression_fatal_limit: 2.0,
            tension_elastic_limit: -1.0,
            tension_fatal_limit: -1.0,
            shear_elastic_limit: -1.0,
            shear_fatal_limit: -1.0,
        }
    }
}

/// Decomposed stress result for a bond (Pascals).
#[derive(Clone, Copy, Debug, Default)]
pub struct BondStressResult {
    pub compression: f32,
    pub tension: f32,
    pub shear: f32,
}

/// Mapped stress severity percentages (0..1).
#[derive(Clone, Copy, Debug)]
pub struct StressSeverity {
    pub compression: f32,
    pub tension: f32,
    pub shear: f32,
}

impl StressSeverity {
    pub fn max_component(&self) -> f32 {
        self.compression.max(self.tension).max(self.shear)
    }
}

/// Which failure mode caused a bond to break.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StressFailure {
    Compression,
    Tension,
    Shear,
}

impl fmt::Display for StressFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            StressFailure::Compression => write!(f, "compression"),
            StressFailure::Tension => write!(f, "tension"),
            StressFailure::Shear => write!(f, "shear"),
        }
    }
}

/// Material stress limits for evaluating bond failure.
#[derive(Clone, Copy, Debug)]
pub struct StressLimits {
    pub compression_elastic_limit: f32,
    pub compression_fatal_limit: f32,
    pub tension_elastic_limit: f32,
    pub tension_fatal_limit: f32,
    pub shear_elastic_limit: f32,
    pub shear_fatal_limit: f32,
}

impl Default for StressLimits {
    fn default() -> Self {
        Self {
            compression_elastic_limit: 1.0,
            compression_fatal_limit: 2.0,
            tension_elastic_limit: -1.0,
            tension_fatal_limit: -1.0,
            shear_elastic_limit: -1.0,
            shear_fatal_limit: -1.0,
        }
    }
}

impl StressLimits {
    fn resolve(limit: f32, fallback: f32) -> f32 {
        if limit > 0.0 {
            limit
        } else if fallback > 0.0 {
            fallback
        } else {
            1.0
        }
    }

    pub fn compression_elastic(&self) -> f32 {
        Self::resolve(self.compression_elastic_limit, 1.0)
    }

    pub fn compression_fatal(&self) -> f32 {
        Self::resolve(self.compression_fatal_limit, self.compression_elastic())
    }

    pub fn tension_elastic(&self) -> f32 {
        Self::resolve(self.tension_elastic_limit, self.compression_elastic())
    }

    pub fn tension_fatal(&self) -> f32 {
        Self::resolve(self.tension_fatal_limit, self.compression_fatal())
    }

    pub fn shear_elastic(&self) -> f32 {
        Self::resolve(self.shear_elastic_limit, self.compression_elastic())
    }

    pub fn shear_fatal(&self) -> f32 {
        Self::resolve(self.shear_fatal_limit, self.compression_fatal())
    }

    fn map_stress_value(stress: f32, elastic: f32, fatal: f32) -> f32 {
        if stress <= 0.0 {
            return 0.0;
        }
        let elastic = if elastic > 0.0 { elastic } else { fatal };
        let fatal = if fatal > 0.0 { fatal } else { elastic.max(1.0) };
        if elastic > 0.0 && stress < elastic {
            (stress / elastic * 0.5).clamp(0.0, 0.5)
        } else if fatal > elastic && elastic > 0.0 {
            (0.5 + 0.5 * (stress - elastic) / (fatal - elastic)).clamp(0.5, 1.0)
        } else {
            (stress / fatal).clamp(0.0, 1.0)
        }
    }

    /// Compute severity (0..1) for each stress component.
    pub fn severity(&self, stress: &BondStressResult) -> StressSeverity {
        StressSeverity {
            compression: Self::map_stress_value(
                stress.compression,
                self.compression_elastic(),
                self.compression_fatal(),
            ),
            tension: Self::map_stress_value(
                stress.tension,
                self.tension_elastic(),
                self.tension_fatal(),
            ),
            shear: Self::map_stress_value(stress.shear, self.shear_elastic(), self.shear_fatal()),
        }
    }

    /// Return the failure mode if any stress component exceeds its fatal limit.
    pub fn failure_mode(&self, stress: &BondStressResult) -> Option<StressFailure> {
        if stress.compression > self.compression_fatal() {
            Some(StressFailure::Compression)
        } else if stress.tension > self.tension_fatal() {
            Some(StressFailure::Tension)
        } else if stress.shear > self.shear_fatal() {
            Some(StressFailure::Shear)
        } else {
            None
        }
    }
}

/// Force application mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum ForceMode {
    Force = 0,
    Acceleration = 1,
}

/// A bond fracture within a fracture command.
#[derive(Clone, Copy, Debug, Default)]
pub struct BondFracture {
    pub userdata: u32,
    pub node_index0: u32,
    pub node_index1: u32,
    pub health: f32,
}

/// Fracture commands for a single actor.
#[derive(Clone, Debug, Default)]
pub struct FractureCommand {
    pub actor_index: u32,
    pub bond_fractures: Vec<BondFracture>,
}

/// A child actor produced by a split.
#[derive(Clone, Debug)]
pub struct SplitChild {
    pub actor_index: u32,
    pub nodes: Vec<u32>,
}

/// Result of applying fracture commands — one split event per parent that split.
#[derive(Clone, Debug)]
pub struct SplitEvent {
    pub parent_actor_index: u32,
    pub children: Vec<SplitChild>,
}

/// An actor in the stress graph (a connected component of nodes).
#[derive(Clone, Debug)]
pub struct Actor {
    pub actor_index: u32,
    pub nodes: Vec<u32>,
}

/// A node in a scenario description.
#[derive(Clone, Copy, Debug, Default)]
pub struct ScenarioNode {
    pub centroid: Vec3,
    pub mass: f32,
    pub volume: f32,
}

/// A bond in a scenario description.
#[derive(Clone, Copy, Debug, Default)]
pub struct ScenarioBond {
    pub node0: u32,
    pub node1: u32,
    pub centroid: Vec3,
    pub normal: Vec3,
    pub area: f32,
}

/// Full scenario description (nodes + bonds).
#[derive(Clone, Debug, Default)]
pub struct ScenarioDesc {
    pub nodes: Vec<ScenarioNode>,
    pub bonds: Vec<ScenarioBond>,
    /// Optional exact per-node collider sizes for Rapier integration.
    /// When omitted, Rapier helpers fall back to cube-root volume estimates.
    pub node_sizes: Vec<Vec3>,
}

impl ScenarioDesc {
    /// Convert to NodeDesc/BondDesc slices suitable for the solver.
    pub fn to_solver_descs(&self) -> (Vec<NodeDesc>, Vec<BondDesc>) {
        let nodes = self
            .nodes
            .iter()
            .map(|n| NodeDesc {
                centroid: n.centroid,
                mass: n.mass,
                volume: n.volume,
            })
            .collect();
        let bonds = self
            .bonds
            .iter()
            .map(|b| BondDesc {
                centroid: b.centroid,
                normal: b.normal,
                area: b.area,
                node0: b.node0,
                node1: b.node1,
            })
            .collect();
        (nodes, bonds)
    }
}
