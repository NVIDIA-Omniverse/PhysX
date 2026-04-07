#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct ExtStressBondDesc;

uint32_t authoring_sizeof_ext_bond_desc();

/**
 * Generate bonds from prefractured triangle meshes.
 *
 * @param mesh_count       Number of chunks.
 * @param geometry_offset  Triangle offsets per chunk (mesh_count + 1 entries, last = total triangles).
 * @param triangle_points  Flattened triangle positions (xyz per vertex, 9 floats per triangle).
 * @param triangle_float_count Number of floats in triangle_points.
 * @param chunk_is_support Optional flags marking support chunks (defaults to true if null).
 * @param bond_mode        0 = EXACT, 1 = AVERAGE.
 * @param max_separation   Maximum gap (meters) when bond_mode == AVERAGE.
 * @param out_bonds        Output pointer to ExtStressBondDesc array (allocated via NVBLAST_ALLOC).
 * @return                 Number of generated bonds.
 */
uint32_t authoring_bonds_from_prefractured_triangles(
    uint32_t mesh_count,
    const uint32_t* geometry_offset,
    const float* triangle_points,
    uint32_t triangle_float_count,
    const uint8_t* chunk_is_support,
    uint32_t bond_mode,
    float max_separation,
    ExtStressBondDesc** out_bonds);

void authoring_free(void* ptr);

#ifdef __cplusplus
}
#endif
