#include "ext_stress_bridge.h"

#include <algorithm>
#include <cstdio>
#include <new>
#include <vector>

#include "NvBlast.h"
#include "NvBlastGlobals.h"
#include "NvBlastSupportGraph.h"
#include "NvBlastExtStressSolver.h"

namespace
{

using namespace Nv::Blast;

struct ExtStressSolverHandleImpl
{
    ExtStressSolver* solver{nullptr};
    NvBlastAsset* asset{nullptr};
    NvBlastFamily* family{nullptr};
    NvBlastActor* actor{nullptr};

    void* assetMem{nullptr};
    void* assetScratch{nullptr};
    void* familyMem{nullptr};
    void* actorScratch{nullptr};

    std::vector<uint32_t> inputToGraph;
    std::vector<uint32_t> graphNodeIndices;
};

inline StressVec3 toStressVec3(const NvcVec3& value)
{
    StressVec3 result;
    result.x = value.x;
    result.y = value.y;
    result.z = value.z;
    return result;
}

inline NvcVec3 toNvcVec3(const StressVec3& value)
{
    return NvcVec3{value.x, value.y, value.z};
}

inline ExtForceMode::Enum toForceMode(uint32_t mode)
{
    switch (mode)
    {
    case ExtForceMode::ACCELERATION:
        return ExtForceMode::ACCELERATION;
    case ExtForceMode::FORCE:
    default:
        return ExtForceMode::FORCE;
    }
}

inline ExtStressSolverSettings toSettings(const ExtStressSolverSettingsDesc* settingsDesc)
{
    ExtStressSolverSettings settings;
    if (!settingsDesc)
    {
        return settings;
    }

    settings.maxSolverIterationsPerFrame = settingsDesc->max_solver_iterations_per_frame;
    settings.graphReductionLevel = settingsDesc->graph_reduction_level;
    settings.compressionElasticLimit = settingsDesc->compression_elastic_limit;
    settings.compressionFatalLimit = settingsDesc->compression_fatal_limit;
    settings.tensionElasticLimit = settingsDesc->tension_elastic_limit;
    settings.tensionFatalLimit = settingsDesc->tension_fatal_limit;
    settings.shearElasticLimit = settingsDesc->shear_elastic_limit;
    settings.shearFatalLimit = settingsDesc->shear_fatal_limit;
    return settings;
}

void releaseHandle(ExtStressSolverHandleImpl* handle)
{
    if (!handle)
    {
        return;
    }

    if (handle->solver)
    {
        handle->solver->release();
        handle->solver = nullptr;
    }

    if (handle->actorScratch)
    {
        NVBLAST_FREE(handle->actorScratch);
        handle->actorScratch = nullptr;
    }

    if (handle->familyMem)
    {
        NVBLAST_FREE(handle->familyMem);
        handle->familyMem = nullptr;
        handle->family = nullptr;
    }

    if (handle->assetScratch)
    {
        NVBLAST_FREE(handle->assetScratch);
        handle->assetScratch = nullptr;
    }

    if (handle->assetMem)
    {
        NVBLAST_FREE(handle->assetMem);
        handle->assetMem = nullptr;
        handle->asset = nullptr;
    }

    delete handle;
}

void stressLog(int type, const char* msg, const char* file, int line)
{
    NV_UNUSED(type);
    NV_UNUSED(file);
    NV_UNUSED(line);
#ifdef DEBUG
    if (msg)
    {
        std::fprintf(stderr, "[Blast][ExtStress] %s (%s:%d)\n", msg, file ? file : "", line);
    }
#else
    NV_UNUSED(msg);
#endif
}

const NvBlastLog kLogFn = stressLog;

} // namespace

extern "C" ExtStressSolverHandle*
ext_stress_solver_create(const ExtStressNodeDesc* nodes,
                        uint32_t node_count,
                        const ExtStressBondDesc* bonds,
                        uint32_t bond_count,
                        const ExtStressSolverSettingsDesc* settingsDesc)
{
    if (!nodes || node_count == 0U || !bonds || bond_count == 0U)
    {
        return nullptr;
    }

    ExtStressSolverHandleImpl* handle = new (std::nothrow) ExtStressSolverHandleImpl();
    if (!handle)
    {
        return nullptr;
    }

    std::vector<NvBlastChunkDesc> chunkDescs;
    chunkDescs.resize(node_count + 1U);

    NvBlastChunkDesc& rootChunk = chunkDescs[0];
    rootChunk.centroid[0] = 0.0f;
    rootChunk.centroid[1] = 0.0f;
    rootChunk.centroid[2] = 0.0f;
    rootChunk.volume = std::max(1.0f, static_cast<float>(node_count));
    rootChunk.parentChunkDescIndex = UINT32_MAX;
    rootChunk.flags = NvBlastChunkDesc::NoFlags;
    rootChunk.userData = 0U;

    for (uint32_t i = 0; i < node_count; ++i)
    {
        NvBlastChunkDesc& desc = chunkDescs[i + 1];
        desc.centroid[0] = nodes[i].centroid.x;
        desc.centroid[1] = nodes[i].centroid.y;
        desc.centroid[2] = nodes[i].centroid.z;
        desc.volume = nodes[i].volume > 0.0f ? nodes[i].volume : std::max(nodes[i].mass, 1.0f);
        desc.parentChunkDescIndex = 0;
        desc.flags = NvBlastChunkDesc::SupportFlag;
        desc.userData = i;
    }

    std::vector<NvBlastBondDesc> bondDescs;
    bondDescs.resize(bond_count);
    for (uint32_t i = 0; i < bond_count; ++i)
    {
        NvBlastBondDesc& desc = bondDescs[i];
        desc.chunkIndices[0] = bonds[i].node0 + 1U;
        desc.chunkIndices[1] = bonds[i].node1 + 1U;

        desc.bond.centroid[0] = bonds[i].centroid.x;
        desc.bond.centroid[1] = bonds[i].centroid.y;
        desc.bond.centroid[2] = bonds[i].centroid.z;

        desc.bond.normal[0] = bonds[i].normal.x;
        desc.bond.normal[1] = bonds[i].normal.y;
        desc.bond.normal[2] = bonds[i].normal.z;

        desc.bond.area = bonds[i].area > 0.0f ? bonds[i].area : 1.0f;
        desc.bond.userData = i;
    }

    NvBlastAssetDesc assetDesc;
    assetDesc.chunkCount = static_cast<uint32_t>(chunkDescs.size());
    assetDesc.chunkDescs = chunkDescs.data();
    assetDesc.bondCount = static_cast<uint32_t>(bondDescs.size());
    assetDesc.bondDescs = bondDescs.data();

    const size_t scratchSize = NvBlastGetRequiredScratchForCreateAsset(&assetDesc, kLogFn);
    handle->assetScratch = NVBLAST_ALLOC(scratchSize);
    if (!handle->assetScratch)
    {
        releaseHandle(handle);
        return nullptr;
    }

    const size_t assetMemSize = NvBlastGetAssetMemorySize(&assetDesc, kLogFn);
    handle->assetMem = NVBLAST_ALLOC(assetMemSize);
    if (!handle->assetMem)
    {
        releaseHandle(handle);
        return nullptr;
    }

    handle->asset = NvBlastCreateAsset(handle->assetMem, &assetDesc, handle->assetScratch, kLogFn);
    if (!handle->asset)
    {
        releaseHandle(handle);
        return nullptr;
    }

    const size_t familyMemSize = NvBlastAssetGetFamilyMemorySize(handle->asset, kLogFn);
    handle->familyMem = NVBLAST_ALLOC(familyMemSize);
    if (!handle->familyMem)
    {
        releaseHandle(handle);
        return nullptr;
    }

    handle->family = NvBlastAssetCreateFamily(handle->familyMem, handle->asset, kLogFn);
    if (!handle->family)
    {
        releaseHandle(handle);
        return nullptr;
    }

    NvBlastActorDesc actorDesc{};
    actorDesc.uniformInitialBondHealth = 1.0f;
    actorDesc.uniformInitialLowerSupportChunkHealth = 1.0f;

    const size_t actorScratchSize = NvBlastFamilyGetRequiredScratchForCreateFirstActor(handle->family, kLogFn);
    handle->actorScratch = NVBLAST_ALLOC(actorScratchSize);
    if (!handle->actorScratch)
    {
        releaseHandle(handle);
        return nullptr;
    }

    handle->actor = NvBlastFamilyCreateFirstActor(handle->family, &actorDesc, handle->actorScratch, kLogFn);
    if (!handle->actor)
    {
        releaseHandle(handle);
        return nullptr;
    }

    ExtStressSolverSettings settings = toSettings(settingsDesc);
    handle->solver = ExtStressSolver::create(*handle->family, settings);
    if (!handle->solver)
    {
        releaseHandle(handle);
        return nullptr;
    }

    const NvBlastSupportGraph supportGraph = NvBlastAssetGetSupportGraph(handle->asset, kLogFn);
    handle->inputToGraph.assign(node_count, UINT32_MAX);
    handle->graphNodeIndices.resize(supportGraph.nodeCount);
    for (uint32_t graphIndex = 0; graphIndex < supportGraph.nodeCount; ++graphIndex)
    {
        handle->graphNodeIndices[graphIndex] = graphIndex;
        const uint32_t chunkIndex = supportGraph.chunkIndices[graphIndex];
        if (chunkIndex > 0 && chunkIndex <= node_count)
        {
            const uint32_t inputIndex = chunkIndex - 1U;
            handle->inputToGraph[inputIndex] = graphIndex;
            const ExtStressNodeDesc& nodeDesc = nodes[inputIndex];
            handle->solver->setNodeInfo(graphIndex,
                                        nodeDesc.mass,
                                        nodeDesc.volume > 0.0f ? nodeDesc.volume : std::max(nodeDesc.mass, 1.0f),
                                        toNvcVec3(nodeDesc.centroid));
        }
    }

    handle->solver->notifyActorCreated(*handle->actor);

    return reinterpret_cast<ExtStressSolverHandle*>(handle);
}

extern "C" void
ext_stress_solver_destroy(ExtStressSolverHandle* handlePtr)
{
    auto* handle = reinterpret_cast<ExtStressSolverHandleImpl*>(handlePtr);
    releaseHandle(handle);
}

extern "C" void
ext_stress_solver_set_settings(ExtStressSolverHandle* handlePtr, const ExtStressSolverSettingsDesc* settingsDesc)
{
    auto* handle = reinterpret_cast<ExtStressSolverHandleImpl*>(handlePtr);
    if (!handle || !handle->solver)
    {
        return;
    }

    ExtStressSolverSettings settings = toSettings(settingsDesc);
    handle->solver->setSettings(settings);
}

extern "C" uint32_t
ext_stress_solver_graph_node_count(const ExtStressSolverHandle* handlePtr)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    return handle ? static_cast<uint32_t>(handle->graphNodeIndices.size()) : 0U;
}

extern "C" uint32_t
ext_stress_solver_bond_count(const ExtStressSolverHandle* handlePtr)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    return (handle && handle->solver) ? handle->solver->getBondCount() : 0U;
}

extern "C" void
ext_stress_solver_reset(ExtStressSolverHandle* handlePtr)
{
    auto* handle = reinterpret_cast<ExtStressSolverHandleImpl*>(handlePtr);
    if (!handle || !handle->solver)
    {
        return;
    }
    handle->solver->reset();
}

extern "C" void
ext_stress_solver_add_force(ExtStressSolverHandle* handlePtr,
                           uint32_t node_index,
                           const StressVec3* local_position,
                           const StressVec3* local_force,
                           uint32_t mode)
{
    auto* handle = reinterpret_cast<ExtStressSolverHandleImpl*>(handlePtr);
    if (!handle || !handle->solver || node_index >= handle->inputToGraph.size())
    {
        return;
    }

    const uint32_t graphIndex = handle->inputToGraph[node_index];
    if (graphIndex == UINT32_MAX)
    {
        return;
    }

    NV_UNUSED(local_position);
    const NvcVec3 force = local_force ? toNvcVec3(*local_force) : NvcVec3{0.0f, 0.0f, 0.0f};

    handle->solver->addForce(graphIndex, force, toForceMode(mode));
}

extern "C" void
ext_stress_solver_add_gravity(ExtStressSolverHandle* handlePtr, const StressVec3* local_gravity)
{
    auto* handle = reinterpret_cast<ExtStressSolverHandleImpl*>(handlePtr);
    if (!handle || !handle->solver || !handle->actor)
    {
        return;
    }

    const NvcVec3 gravity = local_gravity ? toNvcVec3(*local_gravity) : NvcVec3{0.0f, 0.0f, 0.0f};
    handle->solver->addGravity(*handle->actor, gravity);
}

extern "C" void
ext_stress_solver_update(ExtStressSolverHandle* handlePtr)
{
    auto* handle = reinterpret_cast<ExtStressSolverHandleImpl*>(handlePtr);
    if (!handle || !handle->solver)
    {
        return;
    }
    handle->solver->update();
}

extern "C" uint32_t
ext_stress_solver_overstressed_bond_count(const ExtStressSolverHandle* handlePtr)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    return (handle && handle->solver) ? handle->solver->getOverstressedBondCount() : 0U;
}

extern "C" uint32_t
ext_stress_solver_fill_debug_render(const ExtStressSolverHandle* handlePtr,
                                    uint32_t mode,
                                    float scale,
                                    ExtStressDebugLine* out_lines,
                                    uint32_t max_lines)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    if (!handle || !handle->solver || !out_lines || max_lines == 0U)
    {
        return 0U;
    }

    const ExtStressSolver::DebugBuffer buffer = handle->solver->fillDebugRender(
        handle->graphNodeIndices.data(),
        static_cast<uint32_t>(handle->graphNodeIndices.size()),
        static_cast<ExtStressSolver::DebugRenderMode>(mode),
        scale);

    const uint32_t count = std::min(buffer.lineCount, max_lines);
    for (uint32_t i = 0; i < count; ++i)
    {
        const ExtStressSolver::DebugLine& line = buffer.lines[i];
        out_lines[i].p0 = toStressVec3(line.pos0);
        out_lines[i].p1 = toStressVec3(line.pos1);
        out_lines[i].color0 = line.color0;
        out_lines[i].color1 = line.color1;
    }

    return count;
}

extern "C" float
ext_stress_solver_get_linear_error(const ExtStressSolverHandle* handlePtr)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    return (handle && handle->solver) ? handle->solver->getStressErrorLinear() : 0.0f;
}

extern "C" float
ext_stress_solver_get_angular_error(const ExtStressSolverHandle* handlePtr)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    return (handle && handle->solver) ? handle->solver->getStressErrorAngular() : 0.0f;
}

extern "C" uint8_t
ext_stress_solver_converged(const ExtStressSolverHandle* handlePtr)
{
    const auto* handle = reinterpret_cast<const ExtStressSolverHandleImpl*>(handlePtr);
    return (handle && handle->solver && handle->solver->converged()) ? 1U : 0U;
}

extern "C" uint32_t
ext_stress_sizeof_ext_node_desc()
{
    return static_cast<uint32_t>(sizeof(ExtStressNodeDesc));
}

extern "C" uint32_t
ext_stress_sizeof_ext_bond_desc()
{
    return static_cast<uint32_t>(sizeof(ExtStressBondDesc));
}

extern "C" uint32_t
ext_stress_sizeof_ext_settings()
{
    return static_cast<uint32_t>(sizeof(ExtStressSolverSettingsDesc));
}

extern "C" uint32_t
ext_stress_sizeof_ext_debug_line()
{
    return static_cast<uint32_t>(sizeof(ExtStressDebugLine));
}

