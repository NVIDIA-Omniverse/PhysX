#pragma once

#include <stdint.h>

#include "stress_bridge.h"

#ifdef __cplusplus
extern "C" {
#endif

struct ExtStressSolverHandle;

typedef struct ExtStressNodeDesc {
    StressVec3 centroid;
    float mass;
    float volume;
} ExtStressNodeDesc;

typedef struct ExtStressBondDesc {
    StressVec3 centroid;
    StressVec3 normal;
    float area;
    uint32_t node0;
    uint32_t node1;
} ExtStressBondDesc;

typedef struct ExtStressSolverSettingsDesc {
    uint32_t max_solver_iterations_per_frame;
    uint32_t graph_reduction_level;
    float compression_elastic_limit;
    float compression_fatal_limit;
    float tension_elastic_limit;
    float tension_fatal_limit;
    float shear_elastic_limit;
    float shear_fatal_limit;
} ExtStressSolverSettingsDesc;

typedef struct ExtStressDebugLine {
    StressVec3 p0;
    StressVec3 p1;
    uint32_t color0;
    uint32_t color1;
} ExtStressDebugLine;

ExtStressSolverHandle* ext_stress_solver_create(const ExtStressNodeDesc* nodes,
                                                uint32_t node_count,
                                                const ExtStressBondDesc* bonds,
                                                uint32_t bond_count,
                                                const ExtStressSolverSettingsDesc* settings);

void ext_stress_solver_destroy(ExtStressSolverHandle* handle);

void ext_stress_solver_set_settings(ExtStressSolverHandle* handle,
                                    const ExtStressSolverSettingsDesc* settings);

uint32_t ext_stress_solver_graph_node_count(const ExtStressSolverHandle* handle);

uint32_t ext_stress_solver_bond_count(const ExtStressSolverHandle* handle);

void ext_stress_solver_reset(ExtStressSolverHandle* handle);

void ext_stress_solver_add_force(ExtStressSolverHandle* handle,
                                 uint32_t node_index,
                                 const StressVec3* local_position,
                                 const StressVec3* local_force,
                                 uint32_t mode);

void ext_stress_solver_add_gravity(ExtStressSolverHandle* handle,
                                   const StressVec3* local_gravity);

void ext_stress_solver_update(ExtStressSolverHandle* handle);

uint32_t ext_stress_solver_overstressed_bond_count(const ExtStressSolverHandle* handle);

uint32_t ext_stress_solver_fill_debug_render(const ExtStressSolverHandle* handle,
                                             uint32_t mode,
                                             float scale,
                                             ExtStressDebugLine* out_lines,
                                             uint32_t max_lines);

float ext_stress_solver_get_linear_error(const ExtStressSolverHandle* handle);

float ext_stress_solver_get_angular_error(const ExtStressSolverHandle* handle);

uint8_t ext_stress_solver_converged(const ExtStressSolverHandle* handle);

uint32_t ext_stress_sizeof_ext_node_desc();
uint32_t ext_stress_sizeof_ext_bond_desc();
uint32_t ext_stress_sizeof_ext_settings();
uint32_t ext_stress_sizeof_ext_debug_line();

#ifdef __cplusplus
}
#endif

