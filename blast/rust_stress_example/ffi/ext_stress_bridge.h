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

typedef struct ExtStressBondFracture {
    uint32_t userdata;
    uint32_t nodeIndex0;
    uint32_t nodeIndex1;
    float health;
} ExtStressBondFracture;

typedef struct ExtStressFractureCommands {
    uint32_t actorIndex;
    ExtStressBondFracture* bondFractures;
    uint32_t bondFractureCount;
} ExtStressFractureCommands;

typedef struct ExtStressActor {
    uint32_t actorIndex;
    const uint32_t* nodes;
    uint32_t nodeCount;
} ExtStressActor;

typedef struct ExtStressSplitEvent {
    uint32_t parentActorIndex;
    ExtStressActor* children;
    uint32_t childCount;
} ExtStressSplitEvent;

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

uint8_t ext_stress_solver_generate_fracture_commands(const ExtStressSolverHandle* handle,
                                                     ExtStressFractureCommands* out_commands,
                                                     ExtStressBondFracture* bond_buffer,
                                                     uint32_t max_bonds);

uint32_t ext_stress_solver_actor_count(const ExtStressSolverHandle* handle);

uint8_t ext_stress_solver_collect_actors(const ExtStressSolverHandle* handle,
                                         ExtStressActor* actor_buffer,
                                         uint32_t actor_capacity,
                                         uint32_t* nodes_buffer,
                                         uint32_t nodes_capacity,
                                         uint32_t* out_actor_count,
                                         uint32_t* out_node_count);

uint8_t ext_stress_solver_generate_fracture_commands_per_actor(const ExtStressSolverHandle* handle,
                                                               ExtStressFractureCommands* command_buffer,
                                                               uint32_t command_capacity,
                                                               ExtStressBondFracture* bond_buffer,
                                                               uint32_t bond_capacity,
                                                               uint32_t* out_command_count,
                                                               uint32_t* out_bond_count);

uint8_t ext_stress_solver_apply_fracture_commands(ExtStressSolverHandle* handle,
                                                  const ExtStressFractureCommands* command_buffer,
                                                  uint32_t command_count,
                                                  ExtStressSplitEvent* events_buffer,
                                                  uint32_t event_capacity,
                                                  ExtStressActor* child_buffer,
                                                  uint32_t child_capacity,
                                                  uint32_t* out_event_count,
                                                  uint32_t* out_child_count,
                                                  uint32_t* nodes_buffer,
                                                  uint32_t nodes_capacity,
                                                  uint32_t* out_node_count);

uint8_t ext_stress_solver_get_excess_forces(const ExtStressSolverHandle* handle,
                                            uint32_t actor_index,
                                            const StressVec3* center_of_mass,
                                            StressVec3* out_force,
                                            StressVec3* out_torque);

float ext_stress_solver_get_linear_error(const ExtStressSolverHandle* handle);

float ext_stress_solver_get_angular_error(const ExtStressSolverHandle* handle);

uint8_t ext_stress_solver_converged(const ExtStressSolverHandle* handle);

uint32_t ext_stress_sizeof_ext_node_desc();
uint32_t ext_stress_sizeof_ext_bond_desc();
uint32_t ext_stress_sizeof_ext_settings();
uint32_t ext_stress_sizeof_ext_debug_line();
uint32_t ext_stress_sizeof_ext_bond_fracture();
uint32_t ext_stress_sizeof_ext_fracture_commands();
uint32_t ext_stress_sizeof_actor();
uint32_t ext_stress_sizeof_actor_buffer();
uint32_t ext_stress_sizeof_ext_split_event();

#ifdef __cplusplus
}
#endif

