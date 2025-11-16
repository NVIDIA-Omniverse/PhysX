#include "authoring_bridge.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "extensions/authoring/NvBlastExtAuthoring.h"
#include "extensions/authoring/NvBlastExtAuthoringBondGenerator.h"
#include "extensions/authoringCommon/NvBlastExtAuthoringTypes.h"
#include "NvBlastGlobals.h"
#include "NvBlastTypes.h"

#include "../../rust_stress_example/ffi/ext_stress_bridge.h"

namespace
{

using namespace Nv::Blast;

struct TriangleSource
{
    const float* ptr{nullptr};

    Triangle makeTriangle(uint32_t triangleIndex) const
    {
        const float* base = ptr + triangleIndex * 9;
        Triangle tri;
        writeVertex(tri.a, base + 0);
        writeVertex(tri.b, base + 3);
        writeVertex(tri.c, base + 6);
        tri.userData = 0;
        tri.materialId = 0;
        tri.smoothingGroup = 0;
        return tri;
    }

    static void writeVertex(Vertex& v, const float* src)
    {
        v.p = NvcVec3{src[0], src[1], src[2]};
        v.n = NvcVec3{0.0f, 0.0f, 0.0f};
        v.uv[0] = NvcVec2{0.0f, 0.0f};
    }
};

inline StressVec3 toStressVec3(const float value[3])
{
    return StressVec3{value[0], value[1], value[2]};
}

}  // namespace

extern "C" uint32_t authoring_sizeof_ext_bond_desc()
{
    return static_cast<uint32_t>(sizeof(ExtStressBondDesc));
}

extern "C" uint32_t authoring_bonds_from_prefractured_triangles(
    uint32_t mesh_count,
    const uint32_t* geometry_offset,
    const float* triangle_points,
    uint32_t triangle_float_count,
    const uint8_t* chunk_is_support,
    uint32_t bond_mode,
    float max_separation,
    ExtStressBondDesc** out_bonds)
{
    if (out_bonds)
    {
        *out_bonds = nullptr;
    }

    if (mesh_count == 0 || geometry_offset == nullptr || triangle_points == nullptr || out_bonds == nullptr)
    {
        return 0;
    }

    const uint32_t triangle_count = geometry_offset[mesh_count];
    const uint64_t expected_float_count = static_cast<uint64_t>(triangle_count) * 9ULL;
    if (triangle_float_count < expected_float_count)
    {
        return 0;
    }

    TriangleSource source{triangle_points};
    std::vector<Triangle> triangles(triangle_count);
    for (uint32_t i = 0; i < triangle_count; ++i)
    {
        triangles[i] = source.makeTriangle(i);
    }

    std::unique_ptr<bool[]> supportFlags(new bool[mesh_count]);
    for (uint32_t i = 0; i < mesh_count; ++i)
    {
        if (chunk_is_support)
        {
            supportFlags[i] = chunk_is_support[i] != 0;
        }
        else
        {
            supportFlags[i] = true;
        }
    }

    BondGenerationConfig cfg{};
    cfg.bondMode = bond_mode == static_cast<uint32_t>(BondGenerationConfig::AVERAGE)
                       ? BondGenerationConfig::AVERAGE
                       : BondGenerationConfig::EXACT;
    cfg.maxSeparation = max_separation;

    NvBlastBondDesc* bondDescs = nullptr;
    BlastBondGenerator* generator = NvBlastExtAuthoringCreateBondGenerator(nullptr);
    if (generator == nullptr)
    {
        return 0;
    }

    const int32_t bondCount = generator->bondsFromPrefractured(
        mesh_count,
        geometry_offset,
        triangles.data(),
        supportFlags.get(),
        bondDescs,
        cfg);

    generator->release();

    if (bondCount <= 0 || bondDescs == nullptr)
    {
        if (bondDescs)
        {
            NVBLAST_FREE(bondDescs);
        }
        return 0;
    }

    ExtStressBondDesc* mapped = reinterpret_cast<ExtStressBondDesc*>(
        NVBLAST_ALLOC(static_cast<size_t>(bondCount) * sizeof(ExtStressBondDesc)));
    if (!mapped)
    {
        NVBLAST_FREE(bondDescs);
        return 0;
    }

    for (int32_t i = 0; i < bondCount; ++i)
    {
        const NvBlastBondDesc& src = bondDescs[i];
        ExtStressBondDesc& dst = mapped[i];

        dst.centroid = toStressVec3(src.bond.centroid);
        dst.normal = toStressVec3(src.bond.normal);
        dst.area = src.bond.area;
        dst.node0 = src.chunkIndices[0];
        dst.node1 = src.chunkIndices[1];
    }

    NVBLAST_FREE(bondDescs);
    *out_bonds = mapped;
    return static_cast<uint32_t>(bondCount);
}

extern "C" void authoring_free(void* ptr)
{
    if (ptr)
    {
        NVBLAST_FREE(ptr);
    }
}


