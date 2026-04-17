//! Low-level safe wrapper around the C `StressProcessor`.

use crate::ffi;
use crate::types::Vec3;

/// Low-level node descriptor for the StressProcessor.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressNodeDesc {
    pub com: Vec3,
    pub mass: f32,
    pub inertia: f32,
}

/// Low-level bond descriptor for the StressProcessor.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressBondDesc {
    pub centroid: Vec3,
    pub node0: u32,
    pub node1: u32,
}

/// Angular + linear velocity.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressVelocity {
    pub ang: Vec3,
    pub lin: Vec3,
}

/// Angular + linear impulse.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressImpulse {
    pub ang: Vec3,
    pub lin: Vec3,
}

/// Data preparation parameters.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct StressDataParams {
    pub equalize_masses: bool,
    pub center_bonds: bool,
}

/// Solver iteration parameters.
#[derive(Clone, Copy, Debug)]
pub struct StressSolverParams {
    pub max_iterations: u32,
    pub tolerance: f32,
    pub warm_start: bool,
}

impl Default for StressSolverParams {
    fn default() -> Self {
        Self {
            max_iterations: 32,
            tolerance: 1.0e-6,
            warm_start: false,
        }
    }
}

/// Squared error residuals from the solver.
#[derive(Clone, Copy, Debug, Default)]
pub struct StressErrorSq {
    pub ang: f32,
    pub lin: f32,
}

/// Low-level stress solver (conjugate gradient on a bond graph).
///
/// This wraps the C++ `StressProcessor` via FFI. For most use cases,
/// prefer [`ExtStressSolver`](crate::ExtStressSolver) which includes
/// NvBlast actor management and fracture support.
pub struct StressProcessor {
    handle: *mut ffi::StressProcessorHandle,
}

unsafe impl Send for StressProcessor {}
unsafe impl Sync for StressProcessor {}

impl StressProcessor {
    /// Create a new processor from node and bond descriptors.
    pub fn new(
        nodes: &[StressNodeDesc],
        bonds: &[StressBondDesc],
        params: StressDataParams,
    ) -> Option<Self> {
        if nodes.is_empty() || bonds.is_empty() {
            return None;
        }
        let ffi_params = ffi::FfiStressDataParams {
            equalize_masses: params.equalize_masses as u8,
            center_bonds: params.center_bonds as u8,
        };
        let handle = unsafe {
            ffi::stress_processor_create(
                nodes.as_ptr() as *const ffi::FfiStressNodeDesc,
                nodes.len() as u32,
                bonds.as_ptr() as *const ffi::FfiStressBondDesc,
                bonds.len() as u32,
                ffi_params,
            )
        };
        if handle.is_null() {
            None
        } else {
            Some(Self { handle })
        }
    }

    pub fn node_count(&self) -> u32 {
        unsafe { ffi::stress_processor_node_count(self.handle) }
    }

    pub fn bond_count(&self) -> u32 {
        unsafe { ffi::stress_processor_bond_count(self.handle) }
    }

    pub fn node_desc(&self, index: u32) -> Option<StressNodeDesc> {
        let mut desc = ffi::FfiStressNodeDesc::default();
        let ok = unsafe { ffi::stress_processor_get_node_desc(self.handle, index, &mut desc) };
        if ok != 0 {
            Some(StressNodeDesc {
                com: desc.com,
                mass: desc.mass,
                inertia: desc.inertia,
            })
        } else {
            None
        }
    }

    pub fn bond_desc(&self, index: u32) -> Option<StressBondDesc> {
        let mut desc = ffi::FfiStressBondDesc::default();
        let ok = unsafe { ffi::stress_processor_get_bond_desc(self.handle, index, &mut desc) };
        if ok != 0 {
            Some(StressBondDesc {
                centroid: desc.centroid,
                node0: desc.node0,
                node1: desc.node1,
            })
        } else {
            None
        }
    }

    /// Remove a bond from the solver. Returns `true` on success.
    pub fn remove_bond(&mut self, bond_index: u32) -> bool {
        unsafe { ffi::stress_processor_remove_bond(self.handle, bond_index) != 0 }
    }

    /// Solve for impulses given node velocities.
    ///
    /// `impulses` must have length == `bond_count()`, `velocities` must have length == `node_count()`.
    pub fn solve(
        &mut self,
        impulses: &mut [StressImpulse],
        velocities: &[StressVelocity],
        params: StressSolverParams,
        resume: bool,
    ) -> Result<(i32, StressErrorSq), &'static str> {
        if impulses.len() as u32 != self.bond_count() {
            return Err("impulse array must match bond count");
        }
        if velocities.len() as u32 != self.node_count() {
            return Err("velocity array must match node count");
        }
        let ffi_params = ffi::FfiStressSolverParams {
            max_iterations: params.max_iterations,
            tolerance: params.tolerance,
            warm_start: params.warm_start as u8,
            _pad: [0; 3],
        };
        let mut error = ffi::FfiStressErrorSq::default();
        let iterations = unsafe {
            ffi::stress_processor_solve(
                self.handle,
                impulses.as_mut_ptr() as *mut ffi::FfiStressImpulse,
                velocities.as_ptr() as *const ffi::FfiStressVelocity,
                ffi_params,
                &mut error,
                resume as u8,
            )
        };
        if iterations < 0 {
            Err("solver returned an error")
        } else {
            Ok((
                iterations,
                StressErrorSq {
                    ang: error.ang,
                    lin: error.lin,
                },
            ))
        }
    }

    /// Check if the compiled solver uses SIMD.
    pub fn using_simd() -> bool {
        unsafe { ffi::stress_processor_using_simd() != 0 }
    }
}

impl Drop for StressProcessor {
    fn drop(&mut self) {
        unsafe { ffi::stress_processor_destroy(self.handle) }
    }
}
