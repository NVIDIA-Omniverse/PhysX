#![allow(clippy::missing_safety_doc)]

use std::ffi::c_int;
use std::fmt;
use std::ops::{Add, AddAssign, Mul, MulAssign, Sub, SubAssign};

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressVec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl StressVec3 {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub fn dot(self, other: Self) -> f32 {
        self.x * other.x + self.y * other.y + self.z * other.z
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

impl Add for StressVec3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}

impl AddAssign for StressVec3 {
    fn add_assign(&mut self, rhs: Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl Sub for StressVec3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self::new(self.x - rhs.x, self.y - rhs.y, self.z - rhs.z)
    }
}

impl SubAssign for StressVec3 {
    fn sub_assign(&mut self, rhs: Self) {
        self.x -= rhs.x;
        self.y -= rhs.y;
        self.z -= rhs.z;
    }
}

impl Mul<f32> for StressVec3 {
    type Output = Self;

    fn mul(self, rhs: f32) -> Self::Output {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs)
    }
}

impl MulAssign<f32> for StressVec3 {
    fn mul_assign(&mut self, rhs: f32) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl std::ops::Div<f32> for StressVec3 {
    type Output = Self;

    fn div(self, rhs: f32) -> Self::Output {
        Self::new(self.x / rhs, self.y / rhs, self.z / rhs)
    }
}

impl std::ops::DivAssign<f32> for StressVec3 {
    fn div_assign(&mut self, rhs: f32) {
        self.x /= rhs;
        self.y /= rhs;
        self.z /= rhs;
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressNodeDesc {
    pub com: StressVec3,
    pub mass: f32,
    pub inertia: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressBondDesc {
    pub centroid: StressVec3,
    pub node0: u32,
    pub node1: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressVelocity {
    pub ang: StressVec3,
    pub lin: StressVec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressImpulse {
    pub ang: StressVec3,
    pub lin: StressVec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressDataParams {
    pub equalize_masses: u8,
    pub center_bonds: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct StressSolverParams {
    pub max_iterations: u32,
    pub tolerance: f32,
    pub warm_start: u8,
    pub _pad: [u8; 3],
}

impl Default for StressSolverParams {
    fn default() -> Self {
        Self {
            max_iterations: 32,
            tolerance: 1.0e-6,
            warm_start: 0,
            _pad: [0; 3],
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressErrorSq {
    pub ang: f32,
    pub lin: f32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct BondStressResult {
    pub compression: f32,
    pub tension: f32,
    pub shear: f32,
}

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

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StressFailure {
    Compression,
    Tension,
    Shear,
}

impl fmt::Display for StressFailure {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let label = match self {
            StressFailure::Compression => "compression",
            StressFailure::Tension => "tension",
            StressFailure::Shear => "shear",
        };
        write!(f, "{}", label)
    }
}

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

    fn compression_elastic(&self) -> f32 {
        Self::resolve(self.compression_elastic_limit, 1.0)
    }

    fn compression_fatal(&self) -> f32 {
        Self::resolve(self.compression_fatal_limit, self.compression_elastic())
    }

    fn tension_elastic(&self) -> f32 {
        Self::resolve(self.tension_elastic_limit, self.compression_elastic())
    }

    fn tension_fatal(&self) -> f32 {
        Self::resolve(self.tension_fatal_limit, self.compression_fatal())
    }

    fn shear_elastic(&self) -> f32 {
        Self::resolve(self.shear_elastic_limit, self.compression_elastic())
    }

    fn shear_fatal(&self) -> f32 {
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

    pub fn compression_elastic_threshold(&self) -> f32 {
        self.compression_elastic()
    }

    pub fn tension_elastic_threshold(&self) -> f32 {
        self.tension_elastic()
    }

    pub fn shear_elastic_threshold(&self) -> f32 {
        self.shear_elastic()
    }

    pub fn compression_fatal_threshold(&self) -> f32 {
        self.compression_fatal()
    }

    pub fn tension_fatal_threshold(&self) -> f32 {
        self.tension_fatal()
    }

    pub fn shear_fatal_threshold(&self) -> f32 {
        self.shear_fatal()
    }
}

#[repr(C)]
pub struct StressProcessorHandle {
    _private: [u8; 0],
}

extern "C" {
    fn stress_processor_create(
        nodes: *const StressNodeDesc,
        node_count: u32,
        bonds: *const StressBondDesc,
        bond_count: u32,
        params: StressDataParams,
    ) -> *mut StressProcessorHandle;
    fn stress_processor_destroy(handle: *mut StressProcessorHandle);
    fn stress_processor_node_count(handle: *const StressProcessorHandle) -> u32;
    fn stress_processor_bond_count(handle: *const StressProcessorHandle) -> u32;
    fn stress_processor_solve(
        handle: *mut StressProcessorHandle,
        impulses: *mut StressImpulse,
        velocities: *const StressVelocity,
        params: StressSolverParams,
        out_error: *mut StressErrorSq,
        resume_solver: u8,
    ) -> c_int;
    fn stress_processor_remove_bond(handle: *mut StressProcessorHandle, bond_index: u32) -> u8;
    fn stress_processor_get_node_desc(
        handle: *const StressProcessorHandle,
        index: u32,
        out_desc: *mut StressNodeDesc,
    ) -> u8;
    fn stress_processor_get_bond_desc(
        handle: *const StressProcessorHandle,
        index: u32,
        out_desc: *mut StressBondDesc,
    ) -> u8;
    fn stress_processor_using_simd() -> u8;
}

pub struct StressProcessor {
    handle: *mut StressProcessorHandle,
}

unsafe impl Send for StressProcessor {}
unsafe impl Sync for StressProcessor {}

impl StressProcessor {
    pub fn new(
        nodes: &[StressNodeDesc],
        bonds: &[StressBondDesc],
        params: StressDataParams,
    ) -> Option<Self> {
        if nodes.is_empty() || bonds.is_empty() {
            return None;
        }

        let handle = unsafe {
            stress_processor_create(
                nodes.as_ptr(),
                nodes.len() as u32,
                bonds.as_ptr(),
                bonds.len() as u32,
                params,
            )
        };

        if handle.is_null() {
            None
        } else {
            Some(Self { handle })
        }
    }

    pub fn node_count(&self) -> u32 {
        unsafe { stress_processor_node_count(self.handle) }
    }

    pub fn bond_count(&self) -> u32 {
        unsafe { stress_processor_bond_count(self.handle) }
    }

    pub fn node_desc(&self, index: u32) -> Option<StressNodeDesc> {
        let mut desc = StressNodeDesc::default();
        let ok = unsafe { stress_processor_get_node_desc(self.handle, index, &mut desc) };
        if ok != 0 {
            Some(desc)
        } else {
            None
        }
    }

    pub fn bond_desc(&self, index: u32) -> Option<StressBondDesc> {
        let mut desc = StressBondDesc::default();
        let ok = unsafe { stress_processor_get_bond_desc(self.handle, index, &mut desc) };
        if ok != 0 {
            Some(desc)
        } else {
            None
        }
    }

    pub fn remove_bond(&mut self, bond_index: u32) -> bool {
        unsafe { stress_processor_remove_bond(self.handle, bond_index) != 0 }
    }

    pub fn solve(
        &mut self,
        impulses: &mut [StressImpulse],
        velocities: &[StressVelocity],
        params: StressSolverParams,
        resume_solver: bool,
    ) -> Result<(i32, StressErrorSq), &'static str> {
        if impulses.len() as u32 != self.bond_count() {
            return Err("impulse array must match bond count");
        }
        if velocities.len() as u32 != self.node_count() {
            return Err("velocity array must match node count");
        }

        let mut error = StressErrorSq::default();
        let iterations = unsafe {
            stress_processor_solve(
                self.handle,
                impulses.as_mut_ptr(),
                velocities.as_ptr(),
                params,
                &mut error,
                resume_solver as u8,
            )
        };

        if iterations < 0 {
            Err("solver returned an error")
        } else {
            Ok((iterations, error))
        }
    }

    pub fn using_simd() -> bool {
        unsafe { stress_processor_using_simd() != 0 }
    }
}

impl Drop for StressProcessor {
    fn drop(&mut self) {
        unsafe { stress_processor_destroy(self.handle) }
    }
}

pub fn vec3(x: f32, y: f32, z: f32) -> StressVec3 {
    StressVec3::new(x, y, z)
}

pub fn compute_bond_stress(
    bond: &StressBondDesc,
    impulse: &StressImpulse,
    nodes: &[StressNodeDesc],
    bond_area: f32,
) -> BondStressResult {
    if bond_area <= 0.0 {
        return BondStressResult::default();
    }

    let node0 = bond.node0 as usize;
    let node1 = bond.node1 as usize;
    if node0 >= nodes.len() || node1 >= nodes.len() {
        return BondStressResult::default();
    }

    let displacement = nodes[node1].com - nodes[node0].com;
    let node_distance = displacement.magnitude().max(1.0e-6);
    let bond_normal = displacement.normalize();

    let linear = impulse.lin;
    let angular = impulse.ang;

    let normal_component_linear = linear.dot(bond_normal);
    let shear_linear_sq =
        (linear.magnitude_squared() - normal_component_linear * normal_component_linear).max(0.0);
    let mut stress_normal = normal_component_linear / bond_area;
    let mut stress_shear = shear_linear_sq.sqrt() / bond_area;

    let normal_component_angular = angular.dot(bond_normal).abs();
    let angular_mag_sq = angular.magnitude_squared();
    let twist = normal_component_angular / bond_area;
    let bend_sq = (angular_mag_sq - normal_component_angular * normal_component_angular).max(0.0);
    let bend = bend_sq.sqrt() / bond_area;

    let twist_contribution = twist * 2.0 / node_distance;
    stress_shear += twist_contribution;

    let bend_contribution = bend * 2.0 / node_distance;
    let sign = if stress_normal >= 0.0 { 1.0 } else { -1.0 };
    stress_normal += bend_contribution * sign;

    BondStressResult {
        compression: if stress_normal < 0.0 {
            -stress_normal
        } else {
            0.0
        },
        tension: if stress_normal > 0.0 {
            stress_normal
        } else {
            0.0
        },
        shear: stress_shear,
    }
}
