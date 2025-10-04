#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct StressProcessorHandle;

typedef struct StressVec3 {
    float x;
    float y;
    float z;
} StressVec3;

typedef struct StressNodeDesc {
    StressVec3 com;
    float mass;
    float inertia;
} StressNodeDesc;

typedef struct StressBondDesc {
    StressVec3 centroid;
    uint32_t node0;
    uint32_t node1;
} StressBondDesc;

typedef struct StressVelocity {
    StressVec3 ang;
    StressVec3 lin;
} StressVelocity;

typedef struct StressImpulse {
    StressVec3 ang;
    StressVec3 lin;
} StressImpulse;

typedef struct StressDataParams {
    uint8_t equalize_masses;
    uint8_t center_bonds;
} StressDataParams;

typedef struct StressSolverParams {
    uint32_t max_iterations;
    float tolerance;
    uint8_t warm_start;
    uint8_t _pad[3];
} StressSolverParams;

typedef struct StressErrorSq {
    float ang;
    float lin;
} StressErrorSq;

StressProcessorHandle* stress_processor_create(const StressNodeDesc* nodes,
                                               uint32_t node_count,
                                               const StressBondDesc* bonds,
                                               uint32_t bond_count,
                                               StressDataParams params);

void stress_processor_destroy(StressProcessorHandle* handle);

uint32_t stress_processor_node_count(const StressProcessorHandle* handle);
uint32_t stress_processor_bond_count(const StressProcessorHandle* handle);

int32_t stress_processor_solve(StressProcessorHandle* handle,
                               StressImpulse* impulses,
                               const StressVelocity* velocities,
                               StressSolverParams params,
                               StressErrorSq* out_error,
                               uint8_t resume_solver);

uint8_t stress_processor_remove_bond(StressProcessorHandle* handle, uint32_t bond_index);

uint8_t stress_processor_get_node_desc(const StressProcessorHandle* handle,
                                       uint32_t index,
                                       StressNodeDesc* out_desc);

uint8_t stress_processor_get_bond_desc(const StressProcessorHandle* handle,
                                       uint32_t index,
                                       StressBondDesc* out_desc);

uint8_t stress_processor_using_simd();

#ifdef __cplusplus
}
#endif
