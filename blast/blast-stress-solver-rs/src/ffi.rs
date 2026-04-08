//! Raw FFI declarations for the C stress solver bridges.
//!
//! These map directly to `stress_bridge.h` and `ext_stress_bridge.h`.
//! All types are `repr(C)` to match the C layout.

#![allow(non_camel_case_types, dead_code)]

use std::ffi::c_int;

use crate::types::Vec3;

// ---- Low-level StressProcessor FFI ----

#[repr(C)]
pub(crate) struct StressProcessorHandle {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiStressNodeDesc {
    pub com: Vec3,
    pub mass: f32,
    pub inertia: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiStressBondDesc {
    pub centroid: Vec3,
    pub node0: u32,
    pub node1: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiStressVelocity {
    pub ang: Vec3,
    pub lin: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiStressImpulse {
    pub ang: Vec3,
    pub lin: Vec3,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiStressDataParams {
    pub equalize_masses: u8,
    pub center_bonds: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct FfiStressSolverParams {
    pub max_iterations: u32,
    pub tolerance: f32,
    pub warm_start: u8,
    pub _pad: [u8; 3],
}

impl Default for FfiStressSolverParams {
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
pub(crate) struct FfiStressErrorSq {
    pub ang: f32,
    pub lin: f32,
}

extern "C" {
    pub(crate) fn stress_processor_create(
        nodes: *const FfiStressNodeDesc,
        node_count: u32,
        bonds: *const FfiStressBondDesc,
        bond_count: u32,
        params: FfiStressDataParams,
    ) -> *mut StressProcessorHandle;

    pub(crate) fn stress_processor_destroy(handle: *mut StressProcessorHandle);

    pub(crate) fn stress_processor_node_count(handle: *const StressProcessorHandle) -> u32;

    pub(crate) fn stress_processor_bond_count(handle: *const StressProcessorHandle) -> u32;

    pub(crate) fn stress_processor_solve(
        handle: *mut StressProcessorHandle,
        impulses: *mut FfiStressImpulse,
        velocities: *const FfiStressVelocity,
        params: FfiStressSolverParams,
        out_error: *mut FfiStressErrorSq,
        resume_solver: u8,
    ) -> c_int;

    pub(crate) fn stress_processor_remove_bond(
        handle: *mut StressProcessorHandle,
        bond_index: u32,
    ) -> u8;

    pub(crate) fn stress_processor_get_node_desc(
        handle: *const StressProcessorHandle,
        index: u32,
        out_desc: *mut FfiStressNodeDesc,
    ) -> u8;

    pub(crate) fn stress_processor_get_bond_desc(
        handle: *const StressProcessorHandle,
        index: u32,
        out_desc: *mut FfiStressBondDesc,
    ) -> u8;

    pub(crate) fn stress_processor_using_simd() -> u8;
}

// ---- High-level ExtStressSolver FFI ----

#[repr(C)]
pub(crate) struct ExtStressSolverHandle {
    _private: [u8; 0],
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiExtStressNodeDesc {
    pub centroid: Vec3,
    pub mass: f32,
    pub volume: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiExtStressBondDesc {
    pub centroid: Vec3,
    pub normal: Vec3,
    pub area: f32,
    pub node0: u32,
    pub node1: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct FfiExtStressSolverSettingsDesc {
    pub max_solver_iterations_per_frame: u32,
    pub graph_reduction_level: u32,
    pub compression_elastic_limit: f32,
    pub compression_fatal_limit: f32,
    pub tension_elastic_limit: f32,
    pub tension_fatal_limit: f32,
    pub shear_elastic_limit: f32,
    pub shear_fatal_limit: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct FfiExtStressBondFracture {
    pub userdata: u32,
    pub node_index0: u32,
    pub node_index1: u32,
    pub health: f32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct FfiExtStressFractureCommands {
    pub actor_index: u32,
    pub bond_fractures: *mut FfiExtStressBondFracture,
    pub bond_fracture_count: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct FfiExtStressActor {
    pub actor_index: u32,
    pub nodes: *const u32,
    pub node_count: u32,
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(crate) struct FfiExtStressSplitEvent {
    pub parent_actor_index: u32,
    pub children: *mut FfiExtStressActor,
    pub child_count: u32,
}

extern "C" {
    pub(crate) fn ext_stress_solver_create(
        nodes: *const FfiExtStressNodeDesc,
        node_count: u32,
        bonds: *const FfiExtStressBondDesc,
        bond_count: u32,
        settings: *const FfiExtStressSolverSettingsDesc,
    ) -> *mut ExtStressSolverHandle;

    pub(crate) fn ext_stress_solver_destroy(handle: *mut ExtStressSolverHandle);

    pub(crate) fn ext_stress_solver_set_settings(
        handle: *mut ExtStressSolverHandle,
        settings: *const FfiExtStressSolverSettingsDesc,
    );

    pub(crate) fn ext_stress_solver_graph_node_count(
        handle: *const ExtStressSolverHandle,
    ) -> u32;

    pub(crate) fn ext_stress_solver_bond_count(handle: *const ExtStressSolverHandle) -> u32;

    pub(crate) fn ext_stress_solver_reset(handle: *mut ExtStressSolverHandle);

    pub(crate) fn ext_stress_solver_add_force(
        handle: *mut ExtStressSolverHandle,
        node_index: u32,
        local_position: *const Vec3,
        local_force: *const Vec3,
        mode: u32,
    );

    pub(crate) fn ext_stress_solver_add_gravity(
        handle: *mut ExtStressSolverHandle,
        local_gravity: *const Vec3,
    );

    pub(crate) fn ext_stress_solver_add_actor_gravity(
        handle: *mut ExtStressSolverHandle,
        actor_index: u32,
        local_gravity: *const Vec3,
    ) -> u8;

    pub(crate) fn ext_stress_solver_update(handle: *mut ExtStressSolverHandle);

    pub(crate) fn ext_stress_solver_overstressed_bond_count(
        handle: *const ExtStressSolverHandle,
    ) -> u32;

    pub(crate) fn ext_stress_solver_generate_fracture_commands_per_actor(
        handle: *const ExtStressSolverHandle,
        command_buffer: *mut FfiExtStressFractureCommands,
        command_capacity: u32,
        bond_buffer: *mut FfiExtStressBondFracture,
        bond_capacity: u32,
        out_command_count: *mut u32,
        out_bond_count: *mut u32,
    ) -> u8;

    pub(crate) fn ext_stress_solver_apply_fracture_commands(
        handle: *mut ExtStressSolverHandle,
        command_buffer: *const FfiExtStressFractureCommands,
        command_count: u32,
        events_buffer: *mut FfiExtStressSplitEvent,
        event_capacity: u32,
        child_buffer: *mut FfiExtStressActor,
        child_capacity: u32,
        out_event_count: *mut u32,
        out_child_count: *mut u32,
        nodes_buffer: *mut u32,
        nodes_capacity: u32,
        out_node_count: *mut u32,
    ) -> u8;

    pub(crate) fn ext_stress_solver_actor_count(handle: *const ExtStressSolverHandle) -> u32;

    pub(crate) fn ext_stress_solver_collect_actors(
        handle: *const ExtStressSolverHandle,
        actor_buffer: *mut FfiExtStressActor,
        actor_capacity: u32,
        nodes_buffer: *mut u32,
        nodes_capacity: u32,
        out_actor_count: *mut u32,
        out_node_count: *mut u32,
    ) -> u8;

    pub(crate) fn ext_stress_solver_get_excess_forces(
        handle: *const ExtStressSolverHandle,
        actor_index: u32,
        center_of_mass: *const Vec3,
        out_force: *mut Vec3,
        out_torque: *mut Vec3,
    ) -> u8;

    pub(crate) fn ext_stress_solver_get_linear_error(
        handle: *const ExtStressSolverHandle,
    ) -> f32;

    pub(crate) fn ext_stress_solver_get_angular_error(
        handle: *const ExtStressSolverHandle,
    ) -> f32;

    pub(crate) fn ext_stress_solver_converged(handle: *const ExtStressSolverHandle) -> u8;
}
