#include "authoring_bridge.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>
#include <vector>

#include "extensions/authoring/NvBlastExtAuthoring.h"
#include "extensions/authoring/NvBlastExtAuthoringBondGenerator.h"
#include "extensions/authoringCommon/NvBlastExtAuthoringConvexMeshBuilder.h"
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

class SimpleConvexMeshBuilder final : public ConvexMeshBuilder
{
public:
    CollisionHull* buildCollisionGeometry(uint32_t verticesCount, const NvcVec3* vertexData) override
    {
        Bounds bounds;
        if (vertexData != nullptr)
        {
            for (uint32_t i = 0; i < verticesCount; ++i)
            {
                bounds.include(vertexData[i]);
            }
        }

        if (!bounds.initialized)
        {
            bounds.include(NvcVec3{0.0f, 0.0f, 0.0f});
        }

        bounds.ensureExtent(kMinExtent);

        const NvcVec3 corners[8] = {
            {bounds.minimum.x, bounds.minimum.y, bounds.minimum.z},
            {bounds.maximum.x, bounds.minimum.y, bounds.minimum.z},
            {bounds.maximum.x, bounds.maximum.y, bounds.minimum.z},
            {bounds.minimum.x, bounds.maximum.y, bounds.minimum.z},
            {bounds.minimum.x, bounds.minimum.y, bounds.maximum.z},
            {bounds.maximum.x, bounds.minimum.y, bounds.maximum.z},
            {bounds.maximum.x, bounds.maximum.y, bounds.maximum.z},
            {bounds.minimum.x, bounds.maximum.y, bounds.maximum.z}};

        static constexpr uint32_t faceCount = 6;
        static constexpr uint32_t vertsPerFace = 4;
        static constexpr uint32_t faceIndices[faceCount][vertsPerFace] = {
            {0, 1, 2, 3},  // -Z
            {4, 5, 6, 7},  // +Z
            {0, 3, 7, 4},  // -X
            {1, 5, 6, 2},  // +X
            {0, 4, 5, 1},  // -Y
            {3, 2, 6, 7}   // +Y
        };
        static constexpr NvcVec3 normals[faceCount] = {
            {0.0f, 0.0f, -1.0f},
            {0.0f, 0.0f, 1.0f},
            {-1.0f, 0.0f, 0.0f},
            {1.0f, 0.0f, 0.0f},
            {0.0f, -1.0f, 0.0f},
            {0.0f, 1.0f, 0.0f}};
        const NvcVec3 facePoints[faceCount] = {
            corners[0],
            corners[4],
            corners[0],
            corners[1],
            corners[0],
            corners[3]};

        CollisionHull* hull = allocateHull(faceCount, vertsPerFace);
        if (hull == nullptr)
        {
            return nullptr;
        }

        for (uint32_t i = 0; i < 8; ++i)
        {
            hull->points[i] = corners[i];
        }

        for (uint32_t face = 0; face < faceCount; ++face)
        {
            const uint32_t base = face * vertsPerFace;
            for (uint32_t v = 0; v < vertsPerFace; ++v)
            {
                hull->indices[base + v] = faceIndices[face][v];
            }

            HullPolygon& poly = hull->polygonData[face];
            poly.vertexCount = static_cast<uint16_t>(vertsPerFace);
            poly.indexBase = static_cast<uint16_t>(base);
            writePlane(poly.plane, normals[face], facePoints[face]);
        }

        return hull;
    }

    void releaseCollisionHull(CollisionHull* hull) const override
    {
        if (hull == nullptr)
        {
            return;
        }
        NVBLAST_FREE(hull->points);
        NVBLAST_FREE(hull->indices);
        NVBLAST_FREE(hull->polygonData);
        NVBLAST_FREE(hull);
    }

    void release() override
    {
        delete this;
    }

private:
    struct Bounds
    {
        NvcVec3 minimum{0.0f, 0.0f, 0.0f};
        NvcVec3 maximum{0.0f, 0.0f, 0.0f};
        bool initialized{false};

        void include(const NvcVec3& point)
        {
            if (!initialized)
            {
                minimum = maximum = point;
                initialized = true;
                return;
            }

            minimum.x = std::min(minimum.x, point.x);
            minimum.y = std::min(minimum.y, point.y);
            minimum.z = std::min(minimum.z, point.z);
            maximum.x = std::max(maximum.x, point.x);
            maximum.y = std::max(maximum.y, point.y);
            maximum.z = std::max(maximum.z, point.z);
        }

        void ensureExtent(float epsilon)
        {
            if (!initialized)
            {
                return;
            }
            expandAxis(minimum.x, maximum.x, epsilon);
            expandAxis(minimum.y, maximum.y, epsilon);
            expandAxis(minimum.z, maximum.z, epsilon);
        }

    private:
        static void expandAxis(float& minValue, float& maxValue, float epsilon)
        {
            if ((maxValue - minValue) >= epsilon)
            {
                return;
            }
            const float delta = 0.5f * epsilon;
            minValue -= delta;
            maxValue += delta;
        }
    };

    static CollisionHull* allocateHull(uint32_t faceCount, uint32_t vertsPerFace)
    {
        CollisionHull* hull = reinterpret_cast<CollisionHull*>(NVBLAST_ALLOC(sizeof(CollisionHull)));
        if (hull == nullptr)
        {
            return nullptr;
        }

        hull->pointsCount = 8;
        hull->indicesCount = faceCount * vertsPerFace;
        hull->polygonDataCount = faceCount;

        hull->points = reinterpret_cast<NvcVec3*>(NVBLAST_ALLOC(sizeof(NvcVec3) * hull->pointsCount));
        hull->indices = reinterpret_cast<uint32_t*>(NVBLAST_ALLOC(sizeof(uint32_t) * hull->indicesCount));
        hull->polygonData =
            reinterpret_cast<HullPolygon*>(NVBLAST_ALLOC(sizeof(HullPolygon) * hull->polygonDataCount));

        if (hull->points == nullptr || hull->indices == nullptr || hull->polygonData == nullptr)
        {
            releaseAllocatedHull(hull);
            return nullptr;
        }

        return hull;
    }

    static void releaseAllocatedHull(CollisionHull* hull)
    {
        if (hull == nullptr)
        {
            return;
        }
        if (hull->points)
        {
            NVBLAST_FREE(hull->points);
            hull->points = nullptr;
        }
        if (hull->indices)
        {
            NVBLAST_FREE(hull->indices);
            hull->indices = nullptr;
        }
        if (hull->polygonData)
        {
            NVBLAST_FREE(hull->polygonData);
            hull->polygonData = nullptr;
        }
        NVBLAST_FREE(hull);
    }

    static void writePlane(float plane[4], const NvcVec3& normal, const NvcVec3& point)
    {
        plane[0] = normal.x;
        plane[1] = normal.y;
        plane[2] = normal.z;
        plane[3] = -(normal.x * point.x + normal.y * point.y + normal.z * point.z);
    }

    static constexpr float kMinExtent = 1.0e-3f;
};

struct ConvexBuilderDeleter
{
    void operator()(ConvexMeshBuilder* builder) const
    {
        if (builder != nullptr)
        {
            builder->release();
        }
    }
};

using ConvexBuilderPtr = std::unique_ptr<ConvexMeshBuilder, ConvexBuilderDeleter>;

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

    ConvexBuilderPtr convexBuilder;
    if (cfg.bondMode == BondGenerationConfig::AVERAGE)
    {
        convexBuilder.reset(new (std::nothrow) SimpleConvexMeshBuilder());
        if (!convexBuilder)
        {
            return 0;
        }
    }

    NvBlastBondDesc* bondDescs = nullptr;
    BlastBondGenerator* generator = NvBlastExtAuthoringCreateBondGenerator(convexBuilder.get());
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
    convexBuilder.reset();

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


