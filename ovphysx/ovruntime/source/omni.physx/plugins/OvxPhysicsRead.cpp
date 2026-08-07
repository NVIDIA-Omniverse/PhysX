// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physx/IOvxPhysicsRead.h>

#include <omni/physics/ovstage/OvstageOutput.h> // buildPathList (no-string list)

#include "OmniPhysX.h"
#include "PhysXTools.h" // getWorldTransform, PhysXType
#include "internal/Internal.h"
#include "internal/InternalPhysXDatabase.h"
#include "internal/InternalActor.h"
#include "internal/InternalScene.h"
#include "internal/InternalDeformable.h"
#include "internal/InternalParticle.h"

#include <cudamanager/PxCudaContextManager.h>
#include <cudamanager/PxCudaContext.h>
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadUsd.h"

#include <omni/physx/IPhysx.h> // PhysXType (ePTActor / ePTLink / ...)

#include <PxPhysicsAPI.h>

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quath.h>
#include <pxr/base/gf/rotation.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

using namespace ::physx;
using namespace omni::physx;          // OmniPhysX, OvxAttr, kOvx* scopes, ePT* enum
using namespace omni::physx::internal;
using namespace omni::physx::usdparser;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;
PXR_NAMESPACE_USING_DIRECTIVE         // GfMatrix4d / GfVec3d / GfQuatd / GfRotation / UsdTimeCode

namespace
{

// ----------------------------------------------------------------------------
// Session storage
// ----------------------------------------------------------------------------

// One typed column group, with the scratch the DLTensor aliases living here so it
// outlives the fetch. tensor_count is always 1 (a fixed group stacks its prims in
// one tensor; an array group is emitted one-per-prim, so one prim → one tensor).
struct GroupStore
{
    std::vector<float>    floats;  // SoA scratch: rows * comp
    std::array<int64_t, 1> shape{ { 0 } }; // [rows]; tuple width is dtype.lanes
    DLTensor              tensor{};
    std::vector<uint32_t> indexMap; // array-slot scatter (instancer active subset); empty = identity
    ovx_primpath_list_t   list = OVX_INVALID_PRIMPATH_LIST;
    uint32_t              primCount = 0; // prims this group covers (fixed: rows; array: 1)
    ovx_token_t           attribute = OVX_INVALID_TOKEN;
    bool                  isArray = false;
    ovstage_attribute_semantic_t semantic = OVSTAGE_SEMANTIC_NONE;
};

struct ReadSession
{
    ovx_path_dictionary_t* dict = nullptr;
    std::vector<GroupStore> groups;
    size_t cursor = 0;
};

struct QueryState
{
    uint32_t type = 0;
    uint32_t scope = 0;
    ovx_path_dictionary_t* dict = nullptr; // shared source dictionary (for ovxQueryDictionary)
    // Backing storage for ovxFetchQueryResult's returned attribute-token array
    // (kept alive until ovxReleaseQuery).
    std::vector<ovx_token_t> resultAttrs;
};

// The source's shared path dictionary. buildPathList sets *outDict from the
// OvstageSource even for a zero-length list, so this gets the dict without
// building one. Returns null for a non-ovstage backend.
ovx_path_dictionary_t* sourceDictionary(IPhysicsSource& source)
{
    ovx_path_dictionary_t* dict = nullptr;
    omni::physics::ovstage::buildPathList(source, nullptr, 0, &dict);
    return dict;
}

std::mutex g_mutex;
uint64_t g_nextQuery = 1;
uint64_t g_nextRead = 1;
std::unordered_map<uint64_t, QueryState> g_queries;
std::unordered_map<uint64_t, ReadSession> g_reads;

// ----------------------------------------------------------------------------
// Common helpers
// ----------------------------------------------------------------------------

struct ActiveContext
{
    AttachedStage* stage = nullptr;
    IPhysicsSource* source = nullptr;
};

bool getActiveContext(const char* who, ActiveContext& out)
{
    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
    out.stage = usdLoad ? usdLoad->getActiveAttachedStage() : nullptr;
    if (!out.stage)
    {
        CARB_LOG_ERROR("%s: no attached simulation — attach an ovstage source first.", who);
        return false;
    }
    out.source = out.stage->getSource();
    if (!out.source)
    {
        CARB_LOG_ERROR("%s: attached stage has no physics source.", who);
        return false;
    }
    return true;
}

ovstage_attribute_semantic_t semanticForName(const std::string& name)
{
    if (name == omni::physx::OvxAttr::kPosition || name == omni::physx::OvxAttr::kPoints ||
        name == omni::physx::OvxAttr::kPositions)
        return OVSTAGE_SEMANTIC_POINT;
    if (name == omni::physx::OvxAttr::kOrientation || name == omni::physx::OvxAttr::kOrientations)
        return OVSTAGE_SEMANTIC_QUATERNION;
    if (name == omni::physx::OvxAttr::kLinearVelocity || name == omni::physx::OvxAttr::kAngularVelocity ||
        name == omni::physx::OvxAttr::kVelocities || name == omni::physx::OvxAttr::kAngularVelocities)
        return OVSTAGE_SEMANTIC_VECTOR;
    return OVSTAGE_SEMANTIC_NONE;
}

ovx_token_t internToken(ovx_path_dictionary_t* dict, const std::string& name)
{
    ovx_token_t tok = OVX_INVALID_TOKEN;
    if (dict)
        ovx_path_dictionary_intern_token(dict, ovx_string_t{ name.c_str(), name.size() }, &tok);
    return tok;
}

// Finish a group: build its prim list, intern its attribute token, and stamp the
// DLTensor. `keys` are the prims the group covers (fixed: one per row; array: the
// single owning prim). Returns false (and adds nothing) if the list cannot be built
// (non-ovstage backend) — keeping the read ovstage-only.
bool finalizeGroup(ReadSession& s,
                   IPhysicsSource& source,
                   GroupStore&& g,
                   const std::string& attrName,
                   const std::vector<ObjectKey>& keys,
                   int64_t rows,
                   int64_t comp)
{
    ovx_path_dictionary_t* dict = nullptr;
    g.list = omni::physics::ovstage::buildPathList(source, keys.data(), keys.size(), &dict);
    if (g.list == OVX_INVALID_PRIMPATH_LIST || !dict)
    {
        CARB_LOG_ERROR("ovxReadAttributes: failed to build ovx_primpath_list_t for '%s' (%zu prims) — "
                       "the active backend is not ovstage (this read is ovstage-only).",
                       attrName.c_str(), keys.size());
        return false;
    }
    s.dict = dict;
    g.primCount = static_cast<uint32_t>(keys.size());
    g.attribute = internToken(dict, attrName);
    g.semantic = semanticForName(attrName);
    g.shape = { rows };

    g.tensor = DLTensor{};
    g.tensor.data = g.floats.data();
    g.tensor.device = DLDevice{ kDLCPU, 0 };
    g.tensor.ndim = 1;
    g.tensor.dtype = DLDataType{ kDLFloat, 32, static_cast<uint16_t>(comp) };
    g.tensor.shape = nullptr;   // patched to point at the stored shape in fetch (stable address)
    g.tensor.strides = nullptr; // contiguous row-major SoA
    g.tensor.byte_offset = 0;

    s.groups.push_back(std::move(g));
    return true;
}

// ----------------------------------------------------------------------------
// Rigid bodies (standalone + point-instancer instances)
// ----------------------------------------------------------------------------

// One standalone (non-instanced) dynamic body's per-prim output.
struct StandaloneBody
{
    ObjectKey key;
    PxVec3 pos;
    PxQuat quat; // xyzw
    PxVec3 lin;
    PxVec3 ang;
};

// One instance's contribution to its point-instancer's arrays.
struct InstanceRow
{
    uint32_t index = 0;
    PxVec3 localPos;  // instancer-local
    PxQuat localQuat; // instancer-local (xyzw)
    PxVec3 lin;       // world linear velocity
    PxVec3 ang;       // world angular velocity
};

struct InstancerAccum
{
    ObjectKey instancerKey;
    std::vector<InstanceRow> rows;
    uint32_t maxIndex = 0;
    bool hasActive = false; // ACTIVE scope: instancer is emitted iff >=1 instance moved last step
};

// Active-actor tracking. When a scene runs with PxSceneFlag::eENABLE_ACTIVE_ACTORS
// (the same set the USD write-back sink consumes), getActiveActors() reports the
// exact bodies the solver moved last step. We map each back to its record index
// via PxActor::userData. `available` is true only when at least one scene has the
// flag, so callers can fall back to isSleeping() otherwise (ADR-0007 oq §2).
struct ActiveActorSet
{
    bool available = false;
    std::unordered_set<size_t> indices; // record indices the solver moved last step
    bool isActive(size_t recIdx) const { return indices.count(recIdx) != 0; }
};

ActiveActorSet collectActiveActors()
{
    ActiveActorSet out;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const size_t nRec = db.getRecords().size();
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTScene || !rec.mInternalPtr)
            continue;
        PxScene* scene = reinterpret_cast<InternalScene*>(rec.mInternalPtr)->getScene();
        if (!scene || !(scene->getFlags() & PxSceneFlag::eENABLE_ACTIVE_ACTORS))
            continue;
        out.available = true;
        PxU32 n = 0;
        PxActor** active = scene->getActiveActors(n);
        for (PxU32 i = 0; i < n; ++i)
        {
            if (!active[i])
                continue;
            const size_t idx = reinterpret_cast<size_t>(active[i]->userData);
            if (idx < nRec)
                out.indices.insert(idx);
        }
    }
    return out;
}

// ----------------------------------------------------------------------------
// DirectGPU pose/velocity readback
//
// Under PxSceneFlag::eENABLE_DIRECT_GPU_API (the ovstage / suppressReadback
// pipeline) PhysX does NOT keep the CPU-side accessors in sync: getGlobalPose()
// / getLinearVelocity() / getAngularVelocity() return the stale spawn value and
// the read would report a frozen pose while the solver runs on the GPU
// (NvBugs 6464833 / OMPE-101753 - the rigid/articulation analogue of the
// deformable freeze). Pull the live state from the direct-GPU API at read time
// instead, mirroring what the tensor GpuRigidBodyView does. With a NULL finish
// event getRigidDynamicData/getArticulationData block until the copy completes,
// and the synchronous memcpy calls need no extra stream sync.
// ----------------------------------------------------------------------------

struct BodyState
{
    PxTransform pose;
    PxVec3 lin;
    PxVec3 ang;
};

inline bool sceneIsDirectGpu(const PxScene* sc)
{
    return sc && (sc->getFlags() & PxSceneFlag::eENABLE_DIRECT_GPU_API);
}

// Rigid dynamics: one PxTransform / PxVec3 per body via getRigidDynamicData.
void buildDirectGpuRigidCache(PxCudaContextManager* ctxMgr,
                              const std::vector<PxRigidDynamic*>& bodies,
                              std::unordered_map<const PxRigidActor*, BodyState>& out)
{
    if (!ctxMgr || !ctxMgr->getCudaContext())
        return;
    std::unordered_map<PxScene*, std::vector<PxRigidDynamic*>> byScene;
    for (PxRigidDynamic* b : bodies)
    {
        PxScene* sc = b->getScene();
        if (sceneIsDirectGpu(sc) && b->getGPUIndex() != PX_INVALID_U32)
            byScene[sc].push_back(b);
    }
    if (byScene.empty())
        return;

    PxScopedCudaLock _lock(*ctxMgr);
    PxCudaContext* cu = ctxMgr->getCudaContext();
    for (auto& [scene, bs] : byScene)
    {
        const uint32_t n = static_cast<uint32_t>(bs.size());

        std::vector<PxRigidDynamicGPUIndex> idx(n);
        for (uint32_t i = 0; i < n; ++i)
            idx[i] = bs[i]->getGPUIndex();

        CUdeviceptr idxDev = 0, poseDev = 0, linDev = 0, angDev = 0;
        if (cu->memAlloc(&idxDev, n * sizeof(PxRigidDynamicGPUIndex)) != 0 ||
            cu->memAlloc(&poseDev, n * sizeof(PxTransform)) != 0 ||
            cu->memAlloc(&linDev, n * sizeof(PxVec3)) != 0 ||
            cu->memAlloc(&angDev, n * sizeof(PxVec3)) != 0)
        {
            if (idxDev) cu->memFree(idxDev);
            if (poseDev) cu->memFree(poseDev);
            if (linDev) cu->memFree(linDev);
            if (angDev) cu->memFree(angDev);
            CARB_LOG_WARN_ONCE("ovxReadAttributes: DirectGPU rigid readback allocation failed.");
            continue;
        }

        cu->memcpyHtoD(idxDev, idx.data(), n * sizeof(PxRigidDynamicGPUIndex));
        PxDirectGPUAPI& gpu = scene->getDirectGPUAPI();
        gpu.getRigidDynamicData(reinterpret_cast<void*>(poseDev),
                                reinterpret_cast<const PxRigidDynamicGPUIndex*>(idxDev),
                                PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, n);
        gpu.getRigidDynamicData(reinterpret_cast<void*>(linDev),
                                reinterpret_cast<const PxRigidDynamicGPUIndex*>(idxDev),
                                PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY, n);
        gpu.getRigidDynamicData(reinterpret_cast<void*>(angDev),
                                reinterpret_cast<const PxRigidDynamicGPUIndex*>(idxDev),
                                PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY, n);

        std::vector<PxTransform> poses(n);
        std::vector<PxVec3> lins(n), angs(n);
        cu->memcpyDtoH(poses.data(), poseDev, n * sizeof(PxTransform));
        cu->memcpyDtoH(lins.data(), linDev, n * sizeof(PxVec3));
        cu->memcpyDtoH(angs.data(), angDev, n * sizeof(PxVec3));

        cu->memFree(idxDev);
        cu->memFree(poseDev);
        cu->memFree(linDev);
        cu->memFree(angDev);

        for (uint32_t i = 0; i < n; ++i)
            out[bs[i]] = BodyState{ poses[i], lins[i], angs[i] };
    }
}

// Articulation links: getArticulationData returns per-articulation blocks of
// maxLinks entries; link `j` of the articulation at gpuIndices position `x` is at
// x*maxLinks + j (j = PxArticulationLink::getLinkIndex()).
void buildDirectGpuLinkCache(PxCudaContextManager* ctxMgr,
                             const std::vector<PxArticulationLink*>& links,
                             std::unordered_map<const PxArticulationLink*, BodyState>& out)
{
    if (!ctxMgr || !ctxMgr->getCudaContext())
        return;
    std::unordered_map<PxScene*, std::vector<PxArticulationReducedCoordinate*>> artisByScene;
    std::unordered_set<PxArticulationReducedCoordinate*> seen;
    for (PxArticulationLink* link : links)
    {
        PxScene* sc = link->getScene();
        if (!sceneIsDirectGpu(sc))
            continue;
        PxArticulationReducedCoordinate* art = &link->getArticulation();
        if (art->getGPUIndex() != PX_INVALID_U32 && seen.insert(art).second)
            artisByScene[sc].push_back(art);
    }
    if (artisByScene.empty())
        return;

    PxScopedCudaLock _lock(*ctxMgr);
    PxCudaContext* cu = ctxMgr->getCudaContext();
    for (auto& [scene, arts] : artisByScene)
    {
        const uint32_t na = static_cast<uint32_t>(arts.size());
        PxDirectGPUAPI& gpu = scene->getDirectGPUAPI();
        const uint32_t maxLinks = gpu.getArticulationGPUAPIMaxCounts().maxLinks;
        if (maxLinks == 0)
            continue;
        const uint32_t blk = na * maxLinks;

        std::unordered_map<PxArticulationReducedCoordinate*, uint32_t> slot;
        std::vector<PxArticulationGPUIndex> idx(na);
        for (uint32_t i = 0; i < na; ++i)
        {
            idx[i] = arts[i]->getGPUIndex();
            slot[arts[i]] = i;
        }

        CUdeviceptr idxDev = 0, poseDev = 0, linDev = 0, angDev = 0;
        if (cu->memAlloc(&idxDev, na * sizeof(PxArticulationGPUIndex)) != 0 ||
            cu->memAlloc(&poseDev, blk * sizeof(PxTransform)) != 0 ||
            cu->memAlloc(&linDev, blk * sizeof(PxVec3)) != 0 ||
            cu->memAlloc(&angDev, blk * sizeof(PxVec3)) != 0)
        {
            if (idxDev) cu->memFree(idxDev);
            if (poseDev) cu->memFree(poseDev);
            if (linDev) cu->memFree(linDev);
            if (angDev) cu->memFree(angDev);
            CARB_LOG_WARN_ONCE("ovxReadAttributes: DirectGPU articulation readback allocation failed.");
            continue;
        }

        cu->memcpyHtoD(idxDev, idx.data(), na * sizeof(PxArticulationGPUIndex));
        gpu.getArticulationData(reinterpret_cast<void*>(poseDev),
                                reinterpret_cast<const PxArticulationGPUIndex*>(idxDev),
                                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, na);
        gpu.getArticulationData(reinterpret_cast<void*>(linDev),
                                reinterpret_cast<const PxArticulationGPUIndex*>(idxDev),
                                PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY, na);
        gpu.getArticulationData(reinterpret_cast<void*>(angDev),
                                reinterpret_cast<const PxArticulationGPUIndex*>(idxDev),
                                PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY, na);

        std::vector<PxTransform> poses(blk);
        std::vector<PxVec3> lins(blk), angs(blk);
        cu->memcpyDtoH(poses.data(), poseDev, blk * sizeof(PxTransform));
        cu->memcpyDtoH(lins.data(), linDev, blk * sizeof(PxVec3));
        cu->memcpyDtoH(angs.data(), angDev, blk * sizeof(PxVec3));

        cu->memFree(idxDev);
        cu->memFree(poseDev);
        cu->memFree(linDev);
        cu->memFree(angDev);

        for (PxArticulationLink* link : links)
        {
            PxArticulationReducedCoordinate* art = &link->getArticulation();
            auto sit = slot.find(art);
            if (sit == slot.end())
                continue; // link belongs to a different scene / articulation batch
            const uint32_t li = link->getLinkIndex();
            if (li >= maxLinks)
                continue;
            const uint32_t off = sit->second * maxLinks + li;
            out[link] = BodyState{ poses[off], lins[off], angs[off] };
        }
    }
}

// Articulation joint DOF state (per-axis position / velocity). getArticulationData
// returns per-articulation blocks of maxDofs reals for eJOINT_POSITION /
// eJOINT_VELOCITY. The DOF index of a joint axis within its articulation block
// follows the reduced-coordinate cache order: links are ordered by low-level index
// (PxArticulationLink::getLinkIndex), each contributing getInboundJointDof() DOFs,
// and within a joint the unlocked axes are laid out in PxArticulationAxis enum
// order. This mirrors the tensor DOF path (BaseSimulationView.cpp). The stored raw
// values are indexed by PxArticulationAxis; locked / uncovered axes stay NaN.
struct JointDofState
{
    float pos[PxArticulationAxis::eCOUNT];
    float vel[PxArticulationAxis::eCOUNT];
};

void buildDirectGpuJointCache(
    PxCudaContextManager* ctxMgr,
    const std::vector<PxArticulationJointReducedCoordinate*>& joints,
    std::unordered_map<const PxArticulationJointReducedCoordinate*, JointDofState>& out)
{
    if (!ctxMgr || !ctxMgr->getCudaContext())
        return;
    // Deduplicate the owning articulations per DirectGPU scene.
    std::unordered_map<PxScene*, std::vector<PxArticulationReducedCoordinate*>> artisByScene;
    std::unordered_set<PxArticulationReducedCoordinate*> seen;
    for (PxArticulationJointReducedCoordinate* j : joints)
    {
        PxArticulationLink& child = j->getChildArticulationLink();
        PxScene* sc = child.getScene();
        if (!sceneIsDirectGpu(sc))
            continue;
        PxArticulationReducedCoordinate* art = &child.getArticulation();
        if (art->getGPUIndex() != PX_INVALID_U32 && seen.insert(art).second)
            artisByScene[sc].push_back(art);
    }
    if (artisByScene.empty())
        return;

    PxScopedCudaLock _lock(*ctxMgr);
    PxCudaContext* cu = ctxMgr->getCudaContext();
    for (auto& [scene, arts] : artisByScene)
    {
        const uint32_t na = static_cast<uint32_t>(arts.size());
        PxDirectGPUAPI& gpu = scene->getDirectGPUAPI();
        const uint32_t maxDofs = gpu.getArticulationGPUAPIMaxCounts().maxDofs;
        if (maxDofs == 0)
            continue;
        const uint32_t blk = na * maxDofs;

        std::unordered_map<PxArticulationReducedCoordinate*, uint32_t> slot;
        std::vector<PxArticulationGPUIndex> idx(na);
        for (uint32_t i = 0; i < na; ++i)
        {
            idx[i] = arts[i]->getGPUIndex();
            slot[arts[i]] = i;
        }

        CUdeviceptr idxDev = 0, posDev = 0, velDev = 0;
        if (cu->memAlloc(&idxDev, na * sizeof(PxArticulationGPUIndex)) != 0 ||
            cu->memAlloc(&posDev, blk * sizeof(PxReal)) != 0 ||
            cu->memAlloc(&velDev, blk * sizeof(PxReal)) != 0)
        {
            if (idxDev) cu->memFree(idxDev);
            if (posDev) cu->memFree(posDev);
            if (velDev) cu->memFree(velDev);
            CARB_LOG_WARN_ONCE("ovxReadAttributes: DirectGPU joint readback allocation failed.");
            continue;
        }

        cu->memcpyHtoD(idxDev, idx.data(), na * sizeof(PxArticulationGPUIndex));
        gpu.getArticulationData(reinterpret_cast<void*>(posDev),
                                reinterpret_cast<const PxArticulationGPUIndex*>(idxDev),
                                PxArticulationGPUAPIReadType::eJOINT_POSITION, na);
        gpu.getArticulationData(reinterpret_cast<void*>(velDev),
                                reinterpret_cast<const PxArticulationGPUIndex*>(idxDev),
                                PxArticulationGPUAPIReadType::eJOINT_VELOCITY, na);

        std::vector<PxReal> pos(blk), vel(blk);
        cu->memcpyDtoH(pos.data(), posDev, blk * sizeof(PxReal));
        cu->memcpyDtoH(vel.data(), velDev, blk * sizeof(PxReal));

        cu->memFree(idxDev);
        cu->memFree(posDev);
        cu->memFree(velDev);

        // Per-articulation DOF-start scan (cache order = low-level link index), then
        // map each joint's unlocked axes onto its DOF sub-slots.
        for (const auto& [art, si] : slot)
        {
            const uint32_t numLinks = art->getNbLinks();
            std::vector<PxArticulationLink*> links(numLinks);
            art->getLinks(links.data(), numLinks);

            std::vector<uint32_t> dofCount(numLinks, 0);
            for (PxArticulationLink* l : links)
            {
                const uint32_t li = l->getLinkIndex();
                const uint32_t nd = l->getInboundJointDof();
                if (li < numLinks && nd != 0xffffffffu)
                    dofCount[li] = nd;
            }
            std::vector<uint32_t> dofStart(numLinks, 0);
            uint32_t running = 0;
            for (uint32_t li = 0; li < numLinks; ++li)
            {
                dofStart[li] = running;
                running += dofCount[li];
            }

            for (PxArticulationJointReducedCoordinate* j : joints)
            {
                PxArticulationLink& child = j->getChildArticulationLink();
                if (&child.getArticulation() != art)
                    continue;
                const uint32_t li = child.getLinkIndex();
                if (li >= numLinks)
                    continue;
                const uint32_t base = si * maxDofs + dofStart[li];

                JointDofState st;
                for (int a = 0; a < PxArticulationAxis::eCOUNT; ++a)
                {
                    st.pos[a] = std::numeric_limits<float>::quiet_NaN();
                    st.vel[a] = std::numeric_limits<float>::quiet_NaN();
                }
                uint32_t sub = 0;
                for (int a = 0; a < PxArticulationAxis::eCOUNT; ++a)
                {
                    const PxArticulationAxis::Enum axis = static_cast<PxArticulationAxis::Enum>(a);
                    if (j->getMotion(axis) == PxArticulationMotion::eLOCKED)
                        continue; // not a degree of freedom
                    const uint32_t gi = base + sub;
                    if (gi < blk)
                    {
                        st.pos[a] = pos[gi];
                        st.vel[a] = vel[gi];
                    }
                    ++sub;
                }
                out[j] = st;
            }
        }
    }
}

void enumerateRigidBodies(AttachedStage& as,
                          uint32_t scope,
                          std::vector<StandaloneBody>& standalone,
                          std::vector<InstancerAccum>& instancers)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    std::unordered_map<uint64_t, size_t> instancerSlot; // instancerKey.handle -> index in instancers
    std::map<uint64_t, GfMatrix4d> instancerWorldInv;   // cached per instancer

    // Prefer the engine's active-actor set when available; else approximate with
    // isSleeping() (a kinematic body reports active only while it is being moved).
    const ActiveActorSet activeSet =
        (scope == omni::physx::kOvxActive) ? collectActiveActors() : ActiveActorSet{};

    const std::vector<InternalDatabase::Record>& records = db.getRecords();

    // Under a DirectGPU scene the CPU-side getGlobalPose()/get*Velocity() are stale;
    // pre-pull the live state from the direct-GPU API into a per-body cache, then
    // read from it below (empty + zero-overhead on the non-DirectGPU path).
    std::vector<PxRigidDynamic*> allDynamics;
    for (const InternalDatabase::Record& rec : records)
    {
        if (rec.mType != omni::physx::ePTActor || !rec.mPtr)
            continue;
        if (PxRigidDynamic* d = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxRigidDynamic>())
            allDynamics.push_back(d);
    }
    std::unordered_map<const PxRigidActor*, BodyState> gpuCache;
    buildDirectGpuRigidCache(OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager(),
                             allDynamics, gpuCache);
    auto stateOf = [&gpuCache](PxRigidDynamic* d) -> BodyState
    {
        auto it = gpuCache.find(d);
        if (it != gpuCache.end())
            return it->second;
        return BodyState{ d->getGlobalPose(), d->getLinearVelocity(), d->getAngularVelocity() };
    };

    for (size_t ri = 0; ri < records.size(); ++ri)
    {
        const InternalDatabase::Record& rec = records[ri];
        if (rec.mType != omni::physx::ePTActor || !rec.mPtr || !rec.mInternalPtr)
            continue;
        PxRigidActor* actor = reinterpret_cast<PxRigidActor*>(rec.mPtr);
        PxRigidDynamic* dyn = actor->is<PxRigidDynamic>();
        if (!dyn)
            continue; // rigid bodies = PxRigidDynamic (dynamic or kinematic)

        // Per-body "moved last step" decision, shared by standalone bodies and
        // point-instancer instances.
        bool bodyActive = true;
        if (scope == omni::physx::kOvxActive)
        {
            bodyActive = activeSet.available
                             ? activeSet.isActive(ri)
                             : !((dyn->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC) || dyn->isSleeping());
        }

        InternalActor* ia = reinterpret_cast<InternalActor*>(rec.mInternalPtr);

        // Standalone bodies honour ACTIVE directly (only emit if they moved).
        if (!ia || ia->mInstanceIndex == kInvalidUint32_t)
        {
            if (scope == omni::physx::kOvxActive && !bodyActive)
                continue;
            const BodyState st = stateOf(dyn);
            if (!st.pose.isValid())
                continue;
            standalone.push_back({ rec.mKey, st.pose.p, st.pose.q, st.lin, st.ang });
            continue;
        }

        const BodyState st = stateOf(dyn);
        const PxTransform xf = st.pose;
        if (!xf.isValid())
            continue;
        const PxVec3 lin = st.lin;
        const PxVec3 ang = st.ang;

        // Instanced: express the pose in instancer-local instance-array space,
        // matching InternalScene::updateRigidBodyTransforms' write-back math.
        const uint64_t ih = ia->mInstanceKey.handle;
        auto invIt = instancerWorldInv.find(ih);
        if (invIt == instancerWorldInv.end())
            invIt = instancerWorldInv
                        .emplace(ih, getWorldTransform(as, ia->mInstanceKey, PXR_NS::UsdTimeCode::Default()).GetInverse())
                        .first;
        const GfMatrix4d trMatrix(GfRotation(GfQuatd(xf.q.w, xf.q.x, xf.q.y, xf.q.z)), GfVec3d(xf.p.x, xf.p.y, xf.p.z));
        const GfMatrix4d writeMatrix = ia->mProtoTransformInverse * trMatrix * invIt->second;
        const GfVec3d t = writeMatrix.ExtractTranslation();
        const GfQuatd q = writeMatrix.ExtractRotation().GetQuat();
        const GfVec3d qi = q.GetImaginary();

        auto slotIt = instancerSlot.find(ih);
        if (slotIt == instancerSlot.end())
        {
            slotIt = instancerSlot.emplace(ih, instancers.size()).first;
            InstancerAccum acc;
            acc.instancerKey = ia->mInstanceKey;
            instancers.push_back(std::move(acc));
        }
        InstancerAccum& acc = instancers[slotIt->second];
        InstanceRow row;
        row.index = ia->mInstanceIndex;
        row.localPos = PxVec3(float(t[0]), float(t[1]), float(t[2]));
        row.localQuat = PxQuat(float(qi[0]), float(qi[1]), float(qi[2]), float(q.GetReal()));
        row.lin = lin;
        row.ang = ang;
        // Accumulate EVERY instance (not just the active ones) so the group can be
        // emitted as the instancer's full instance array — see buildRigidBodyGroups.
        acc.rows.push_back(row);
        acc.maxIndex = std::max(acc.maxIndex, ia->mInstanceIndex);
        acc.hasActive = acc.hasActive || bodyActive;
    }

    // ACTIVE scope: keep an instancer only if at least one of its instances moved
    // last step. The retained ones still carry their full instance set (above), so
    // each is emitted as a dense, contract-compliant full array — no intra-tensor
    // scatter (see the #2 note in buildRigidBodyGroups).
    if (scope == omni::physx::kOvxActive)
        instancers.erase(std::remove_if(instancers.begin(), instancers.end(),
                                        [](const InstancerAccum& a) { return !a.hasActive; }),
                         instancers.end());
}

// Map a requested rigid-body attribute name to its component count and, for the
// point-instancer array form, the instancer-array attribute name. Returns false
// for an unknown name.
bool rigidAttrInfo(const std::string& name, int& comp, std::string& arrayName)
{
    using namespace omni::physx;
    if (name == OvxAttr::kPosition)        { comp = 3; arrayName = OvxAttr::kPositions; return true; }
    if (name == OvxAttr::kOrientation)     { comp = 4; arrayName = OvxAttr::kOrientations; return true; }
    if (name == OvxAttr::kLinearVelocity)  { comp = 3; arrayName = OvxAttr::kVelocities; return true; }
    if (name == OvxAttr::kAngularVelocity) { comp = 3; arrayName = OvxAttr::kAngularVelocities; return true; }
    return false;
}

void fillStandaloneColumn(const std::vector<StandaloneBody>& bodies, const std::string& name, int comp,
                          std::vector<float>& out)
{
    using namespace omni::physx;
    out.resize(bodies.size() * comp);
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        float* d = &out[i * comp];
        const StandaloneBody& b = bodies[i];
        if (name == OvxAttr::kPosition)            { d[0] = b.pos.x; d[1] = b.pos.y; d[2] = b.pos.z; }
        else if (name == OvxAttr::kOrientation)    { d[0] = b.quat.x; d[1] = b.quat.y; d[2] = b.quat.z; d[3] = b.quat.w; }
        else if (name == OvxAttr::kLinearVelocity) { d[0] = b.lin.x; d[1] = b.lin.y; d[2] = b.lin.z; }
        else                                       { d[0] = b.ang.x; d[1] = b.ang.y; d[2] = b.ang.z; }
    }
}

void buildRigidBodyGroups(ReadSession& s,
                          IPhysicsSource& source,
                          AttachedStage& as,
                          uint32_t scope,
                          const std::vector<std::string>& names)
{
    std::vector<StandaloneBody> standalone;
    std::vector<InstancerAccum> instancers;
    enumerateRigidBodies(as, scope, standalone, instancers);

    for (const std::string& name : names)
    {
        int comp = 0;
        std::string arrayName;
        if (!rigidAttrInfo(name, comp, arrayName))
        {
            CARB_LOG_WARN("ovxReadAttributes: '%s' is not a rigid-body output attribute — skipped.", name.c_str());
            continue;
        }

        // Fixed group for standalone bodies.
        if (!standalone.empty())
        {
            GroupStore g;
            g.isArray = false;
            fillStandaloneColumn(standalone, name, comp, g.floats);
            std::vector<ObjectKey> keys;
            keys.reserve(standalone.size());
            for (const StandaloneBody& b : standalone)
                keys.push_back(b.key);
            finalizeGroup(s, source, std::move(g), name, keys, (int64_t)standalone.size(), comp);
        }

        // One array group per instancer (the point-instancer mutation): the
        // attribute is the instancer-array name; positions/orientations are
        // instancer-local, velocities are world.
        //
        // #2: the group is ALWAYS the instancer's FULL instance array, placed
        // by-index (rows = maxIndex + 1), for BOTH scopes. We deliberately do NOT
        // emit a dense active subset with an element-axis index_map: ovstage's
        // index_map indexes the OUTER logical-element/tensor axis (one tensor per
        // logical prim), not rows inside a single instancer tensor, so an
        // intra-tensor scatter could not be forwarded into the ovstage write path
        // verbatim. A full array always forwards verbatim. ACTIVE scope only
        // decides WHICH instancers are emitted (those with >=1 moved instance);
        // an emitted instancer still publishes every instance's current pose.
        for (const InstancerAccum& acc : instancers)
        {
            GroupStore g;
            g.isArray = true;
            const int64_t rows = (int64_t)acc.maxIndex + 1;
            g.floats.assign(size_t(rows) * comp, 0.0f);

            for (const InstanceRow& ir : acc.rows)
            {
                const size_t slot = ir.index; // by-index placement into the full instance array
                if (size_t(slot) * comp + comp > g.floats.size())
                    continue;
                float* d = &g.floats[slot * comp];
                if (name == omni::physx::OvxAttr::kPosition)
                {
                    d[0] = ir.localPos.x; d[1] = ir.localPos.y; d[2] = ir.localPos.z;
                }
                else if (name == omni::physx::OvxAttr::kOrientation)
                {
                    d[0] = ir.localQuat.x; d[1] = ir.localQuat.y; d[2] = ir.localQuat.z; d[3] = ir.localQuat.w;
                }
                else if (name == omni::physx::OvxAttr::kLinearVelocity)
                {
                    d[0] = ir.lin.x; d[1] = ir.lin.y; d[2] = ir.lin.z;
                }
                else
                {
                    d[0] = ir.ang.x; d[1] = ir.ang.y; d[2] = ir.ang.z;
                }
            }
            std::vector<ObjectKey> keys{ acc.instancerKey };
            finalizeGroup(s, source, std::move(g), arrayName, keys, rows, comp);
        }
    }
}

// ----------------------------------------------------------------------------
// Articulation links (link body transforms — a fixed group, never instanced)
// ----------------------------------------------------------------------------

void buildArticulationLinkGroups(ReadSession& s,
                                 IPhysicsSource& source,
                                 uint32_t scope,
                                 const std::vector<std::string>& names)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const ActiveActorSet activeSet =
        (scope == omni::physx::kOvxActive) ? collectActiveActors() : ActiveActorSet{};
    std::vector<StandaloneBody> links;
    const std::vector<InternalDatabase::Record>& records = db.getRecords();

    // DirectGPU: the CPU-side link accessors are stale - pre-pull live link state
    // from the direct-GPU API (empty + zero-overhead on the non-DirectGPU path).
    std::vector<PxArticulationLink*> allLinks;
    for (const InternalDatabase::Record& rec : records)
    {
        if (rec.mType != omni::physx::ePTLink || !rec.mPtr)
            continue;
        if (PxArticulationLink* l = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxArticulationLink>())
            allLinks.push_back(l);
    }
    std::unordered_map<const PxArticulationLink*, BodyState> gpuCache;
    buildDirectGpuLinkCache(OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager(),
                            allLinks, gpuCache);
    auto stateOf = [&gpuCache](PxArticulationLink* l) -> BodyState
    {
        auto it = gpuCache.find(l);
        if (it != gpuCache.end())
            return it->second;
        return BodyState{ l->getGlobalPose(), l->getLinearVelocity(), l->getAngularVelocity() };
    };

    for (size_t ri = 0; ri < records.size(); ++ri)
    {
        const InternalDatabase::Record& rec = records[ri];
        if (rec.mType != omni::physx::ePTLink || !rec.mPtr)
            continue;
        PxArticulationLink* link = reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxArticulationLink>();
        if (!link)
            continue;
        if (scope == omni::physx::kOvxActive)
        {
            if (activeSet.available)
            {
                if (!activeSet.isActive(ri))
                    continue;
            }
            else if (link->getArticulation().isSleeping())
                continue;
        }
        const BodyState st = stateOf(link);
        if (!st.pose.isValid())
            continue;
        links.push_back({ rec.mKey, st.pose.p, st.pose.q, st.lin, st.ang });
    }
    if (links.empty())
        return;

    for (const std::string& name : names)
    {
        int comp = 0;
        std::string arrayName;
        if (!rigidAttrInfo(name, comp, arrayName))
        {
            CARB_LOG_WARN("ovxReadAttributes: '%s' is not an articulation-link output attribute — skipped.",
                          name.c_str());
            continue;
        }
        GroupStore g;
        g.isArray = false;
        fillStandaloneColumn(links, name, comp, g.floats);
        std::vector<ObjectKey> keys;
        keys.reserve(links.size());
        for (const StandaloneBody& b : links)
            keys.push_back(b.key);
        finalizeGroup(s, source, std::move(g), name, keys, (int64_t)links.size(), comp);
    }
}

// ----------------------------------------------------------------------------
// Articulation joint state (per-axis scalar — an array group per joint prim)
// ----------------------------------------------------------------------------

void buildJointStateGroups(ReadSession& s,
                           IPhysicsSource& source,
                           const std::vector<std::string>& names)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    // DirectGPU: the CPU-side joint accessors are stale — PxDirectGPUAPI disables the
    // CPU copy for data the direct API exposes, so getJointPosition/Velocity report
    // frozen values while the solver runs on the GPU. Pre-pull the live joint DOF
    // state (empty + zero-overhead on the non-DirectGPU path), mirroring the link
    // cache in buildArticulationLinkGroups. NvBugs 6481083 / OMPE-102219.
    std::vector<PxArticulationJointReducedCoordinate*> allJoints;
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTLinkJoint || !rec.mPtr || !rec.mInternalPtr)
            continue;
        allJoints.push_back(reinterpret_cast<PxArticulationJointReducedCoordinate*>(rec.mPtr));
    }
    std::unordered_map<const PxArticulationJointReducedCoordinate*, JointDofState> gpuCache;
    buildDirectGpuJointCache(OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager(),
                             allJoints, gpuCache);

    for (const std::string& name : names)
    {
        const bool wantPos = (name == omni::physx::OvxAttr::kJointPosition);
        const bool wantVel = (name == omni::physx::OvxAttr::kJointVelocity);
        if (!wantPos && !wantVel)
        {
            CARB_LOG_WARN("ovxReadAttributes: '%s' is not a joint-state output attribute — skipped.", name.c_str());
            continue;
        }
        for (const InternalDatabase::Record& rec : db.getRecords())
        {
            if (rec.mType != omni::physx::ePTLinkJoint || !rec.mPtr || !rec.mInternalPtr)
                continue;
            InternalJoint* joint = reinterpret_cast<InternalJoint*>(rec.mInternalPtr);
            PxArticulationJointReducedCoordinate* pxJoint =
                reinterpret_cast<PxArticulationJointReducedCoordinate*>(rec.mPtr);

            const JointDofState* gpu = nullptr;
            if (auto it = gpuCache.find(pxJoint); it != gpuCache.end())
                gpu = &it->second;

            // One scalar per unlocked reduced-coordinate DOF axis. Enumerating the
            // actual DOF (getMotion != eLOCKED), rather than only axes flagged by an
            // authored JointStateAPI, makes the runtime joint state readable regardless
            // of whether — or on how many axes — a JointStateAPI is present. This also
            // closes the partial-coverage gap: a joint carrying JointStateAPI on a
            // subset of its DOF now emits every unlocked DOF, not just the flagged ones
            // (NvBugs 6481083 / OMPE-102219). The enum-order + skip-eLOCKED enumeration
            // matches the tensor DOF path (BaseArticulationView.cpp).
            std::vector<float> values;
            for (int a = 0; a < PxArticulationAxis::eCOUNT; ++a)
            {
                const PxArticulationAxis::Enum axis = static_cast<PxArticulationAxis::Enum>(a);
                if (pxJoint->getMotion(axis) == PxArticulationMotion::eLOCKED)
                    continue; // not a degree of freedom

                // Raw PhysX joint value with the body0/body1 order sign convention
                // applied (see InternalJoint::getArticulationJointPosition).
                float raw;
                if (gpu)
                {
                    const float g = wantPos ? gpu->pos[a] : gpu->vel[a];
                    raw = joint->mBody0IsParentLink ? g : -g;
                }
                else
                {
                    raw = wantPos ? joint->getArticulationJointPosition(pxJoint, axis)
                                  : joint->getArticulationJointVelocity(pxJoint, axis);
                }

                // Angular axes report in degrees; linear (prismatic) axes stay in the
                // base length unit. When a JointStateAPI is authored on this axis, honor
                // its per-axis convertToDegrees flag so a partially-flagged joint keeps
                // that axis's exact prior conversion.
                bool toDegrees = (axis == PxArticulationAxis::eTWIST ||
                                  axis == PxArticulationAxis::eSWING1 ||
                                  axis == PxArticulationAxis::eSWING2);
                for (const InternalJoint::InternalJointState& js : joint->mJointStates)
                {
                    if (js.enabled && js.physxAxis == axis)
                    {
                        toDegrees = js.convertToDegrees;
                        break;
                    }
                }
                values.push_back(toDegrees ? radToDeg(raw) : raw);
            }

            if (values.empty())
                continue;

            GroupStore g;
            g.isArray = true;
            g.floats = std::move(values);
            std::vector<ObjectKey> keys{ rec.mKey };
            finalizeGroup(s, source, std::move(g), name, keys, (int64_t)g.floats.size(), 1);
        }
    }
}

// ----------------------------------------------------------------------------
// Vehicle wheels (per-wheel world transform — a fixed group)
// ----------------------------------------------------------------------------

void enumerateVehicleWheels(uint32_t scope, std::vector<StandaloneBody>& wheels)
{
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    const ActiveActorSet activeSet =
        (scope == omni::physx::kOvxActive) ? collectActiveActors() : ActiveActorSet{};
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTVehicle || !rec.mInternalPtr)
            continue;
        InternalVehicle* veh = reinterpret_cast<InternalVehicle*>(rec.mInternalPtr);
        PxRigidDynamic* actor = veh->getRigidDynamicActor();
        if (!actor)
            continue;
        if (scope == omni::physx::kOvxActive)
        {
            // The vehicle's body actor carries its own record index in userData.
            if (activeSet.available)
            {
                if (!activeSet.isActive(reinterpret_cast<size_t>(actor->userData)))
                    continue;
            }
            else if (actor->isSleeping())
                continue;
        }

        const PxTransform actor2World = actor->getGlobalPose();
        const PxTransform body2World = actor2World * actor->getCMassLocalPose();
        for (size_t j = 0; j < veh->mWheelTransformManagementEntries.size(); ++j)
        {
            if (j >= veh->mWheelAttachments.size() || !veh->mWheelAttachments[j])
                continue; // attachment removed
            const InternalVehicle::WheelTransformManagementEntry& e = veh->mWheelTransformManagementEntries[j];
            PxTransform wheelPose;
            if (e.shape)
                wheelPose = actor2World * e.shape->getLocalPose();
            else
                wheelPose = body2World * veh->mPhysXVehicle->getWheelLocalPose(veh->mWheelAttachments[j]->mWheelIndex);
            if (!wheelPose.isValid())
                continue;
            wheels.push_back({ e.wheelRootKey, wheelPose.p, wheelPose.q, PxVec3(0.0f), PxVec3(0.0f) });
        }
    }
}

void buildVehicleWheelGroups(ReadSession& s,
                             IPhysicsSource& source,
                             uint32_t scope,
                             const std::vector<std::string>& names)
{
    std::vector<StandaloneBody> wheels;
    enumerateVehicleWheels(scope, wheels);
    if (wheels.empty())
        return;
    for (const std::string& name : names)
    {
        if (name != omni::physx::OvxAttr::kPosition && name != omni::physx::OvxAttr::kOrientation)
        {
            CARB_LOG_WARN("ovxReadAttributes: '%s' is not a vehicle-wheel output attribute "
                          "(only position / orientation) — skipped.", name.c_str());
            continue;
        }
        const int comp = (name == omni::physx::OvxAttr::kOrientation) ? 4 : 3;
        GroupStore g;
        g.isArray = false;
        fillStandaloneColumn(wheels, name, comp, g.floats);
        std::vector<ObjectKey> keys;
        keys.reserve(wheels.size());
        for (const StandaloneBody& b : wheels)
            keys.push_back(b.key);
        finalizeGroup(s, source, std::move(g), name, keys, (int64_t)wheels.size(), comp);
    }
}

// ----------------------------------------------------------------------------
// Deformable bodies (sim-mesh points / velocities — an array group per mesh prim)
// ----------------------------------------------------------------------------

// Emit a deformable body's sim-mesh point and velocity array groups. Points are
// transformed world-to-mesh-local via mWorldToSimMesh, matching
// InternalScene::updateDeformableTransforms. The sim-mesh host buffer is refreshed
// either by the stepper or by buildDeformableGroups before this function is called.
void emitDeformableBody(ReadSession& s,
                        IPhysicsSource& source,
                        InternalDeformableBody* b,
                        bool sleeping,
                        uint32_t scope,
                        bool wantPoints,
                        bool wantVel)
{
    if (!b || (scope == omni::physx::kOvxActive && sleeping))
        return;
    const uint32_t n = b->mNumSimMeshVertices;
    if (n == 0)
        return;

    if (wantPoints && b->mSimMeshPositionInvMassH)
    {
        GroupStore g;
        g.isArray = true;
        g.floats.resize(size_t(n) * 3);
        for (uint32_t v = 0; v < n; ++v)
        {
            const PxVec4& p = b->mSimMeshPositionInvMassH[v];
            const GfVec3f lp = b->mWorldToSimMesh.Transform(GfVec3f(p.x, p.y, p.z));
            g.floats[v * 3 + 0] = lp[0];
            g.floats[v * 3 + 1] = lp[1];
            g.floats[v * 3 + 2] = lp[2];
        }
        finalizeGroup(s, source, std::move(g), omni::physx::OvxAttr::kPoints, { b->mSimMeshKey }, n, 3);
    }
    if (wantVel && b->mSimMeshVelocityH)
    {
        GroupStore g;
        g.isArray = true;
        g.floats.resize(size_t(n) * 3);
        for (uint32_t v = 0; v < n; ++v)
        {
            const PxVec4& vel = b->mSimMeshVelocityH[v];
            g.floats[v * 3 + 0] = vel.x;
            g.floats[v * 3 + 1] = vel.y;
            g.floats[v * 3 + 2] = vel.z;
        }
        finalizeGroup(s, source, std::move(g), omni::physx::OvxAttr::kVelocities, { b->mSimMeshKey }, n, 3);
    }
}

// Under the ovstage / DirectGPU backend the per-frame USD write-back is off, so
// PhysXStepper::updateDeformables() skips the device-to-host copy of the sim-mesh
// buffers. Pull them here at read time so the read reflects the live simulation
// instead of the frozen bind pose (NvBugs 6464833 / OMPE-101753). Reading
// getSim*BufferD() is valid under eENABLE_DIRECT_GPU_API: the tensor deformable
// body view reads the same device buffers.
void enqueueDeformableSimMeshPull(
    PxCudaContext * cu, InternalDeformableBody * b, CUdeviceptr posD, CUdeviceptr velD, bool wantPoints, bool wantVel)
{
    const uint32_t n = b->mNumSimMeshVertices;
    if (n == 0)
        return;
    if (wantPoints && b->mSimMeshPositionInvMassH && posD)
        cu->memcpyDtoHAsync(b->mSimMeshPositionInvMassH, posD, static_cast<size_t>(n) * sizeof(PxVec4), 0);
    if (wantVel && b->mSimMeshVelocityH && velD)
        cu->memcpyDtoHAsync(b->mSimMeshVelocityH, velD, static_cast<size_t>(n) * sizeof(PxVec4), 0);
}

void buildDeformableGroups(ReadSession& s,
                           IPhysicsSource& source,
                           uint32_t type,
                           uint32_t scope,
                           const std::vector<std::string>& names)
{
    bool wantPoints = false, wantVel = false;
    for (const std::string& nm : names)
    {
        if (nm == omni::physx::OvxAttr::kPoints) wantPoints = true;
        else if (nm == omni::physx::OvxAttr::kVelocities) wantVel = true;
        else CARB_LOG_WARN("ovxReadAttributes: '%s' is not a deformable output attribute "
                           "(only points / velocities) — skipped.", nm.c_str());
    }
    if (!wantPoints && !wantVel)
        return;

    PxCudaContextManager* ctxMgr = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!ctxMgr || !ctxMgr->getCudaContext())
    {
        CARB_LOG_WARN("ovxReadAttributes: deformable read needs a CUDA context — none available.");
        return;
    }
    PxCudaContext* cu = ctxMgr->getCudaContext();
    PxScopedCudaLock _lock(*ctxMgr);

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTScene || !rec.mInternalPtr)
            continue;
        InternalScene* sc = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
        sc->syncDeformableCopyStream(ctxMgr);
        if (type == omni::physx::kOvxDeformableVolume)
        {
            for (InternalVolumeDeformableBody* b : sc->mVolumeDeformableBodies)
            {
                PxDeformableVolume* dv = b->mDeformableVolume;
                const bool sleeping = dv && dv->isSleeping();
                if (dv && (scope != omni::physx::kOvxActive || !sleeping))
                    enqueueDeformableSimMeshPull(
                        cu, b, reinterpret_cast<CUdeviceptr>(dv->getSimPositionInvMassBufferD()),
                        reinterpret_cast<CUdeviceptr>(dv->getSimVelocityBufferD()), wantPoints, wantVel);
            }
            cu->streamSynchronize(0);
            for (InternalVolumeDeformableBody* b : sc->mVolumeDeformableBodies)
                emitDeformableBody(s, source, b, b->mDeformableVolume && b->mDeformableVolume->isSleeping(),
                                   scope, wantPoints, wantVel);
        }
        else
        {
            for (InternalSurfaceDeformableBody* b : sc->mSurfaceDeformableBodies)
            {
                PxDeformableSurface* ds = b->mDeformableSurface;
                const bool sleeping = ds && ds->isSleeping();
                if (ds && (scope != omni::physx::kOvxActive || !sleeping))
                    enqueueDeformableSimMeshPull(cu, b, reinterpret_cast<CUdeviceptr>(ds->getPositionInvMassBufferD()),
                                                 reinterpret_cast<CUdeviceptr>(ds->getVelocityBufferD()), wantPoints,
                                                 wantVel);
            }
            cu->streamSynchronize(0);
            for (InternalSurfaceDeformableBody* b : sc->mSurfaceDeformableBodies)
                emitDeformableBody(s, source, b, b->mDeformableSurface && b->mDeformableSurface->isSleeping(),
                                   scope, wantPoints, wantVel);
        }
    }
}

// ----------------------------------------------------------------------------
// Particle sets (points / velocities — an array group per particle-set prim)
// ----------------------------------------------------------------------------

void buildParticleGroups(ReadSession& s,
                         IPhysicsSource& source,
                         AttachedStage& as,
                         const std::vector<std::string>& names)
{
    bool wantPoints = false, wantVel = false;
    for (const std::string& nm : names)
    {
        if (nm == omni::physx::OvxAttr::kPoints || nm == omni::physx::OvxAttr::kPositions) wantPoints = true;
        else if (nm == omni::physx::OvxAttr::kVelocities) wantVel = true;
        else CARB_LOG_WARN("ovxReadAttributes: '%s' is not a particle output attribute "
                           "(only points / velocities) — skipped.", nm.c_str());
    }
    if (!wantPoints && !wantVel)
        return;

    PxCudaContextManager* ctxMgr = OmniPhysX::getInstance().getPhysXSetup().getCudaContextManager();
    if (!ctxMgr || !ctxMgr->getCudaContext())
    {
        CARB_LOG_WARN("ovxReadAttributes: particle read needs a CUDA context — none available.");
        return;
    }
    PxCudaContext* cu = ctxMgr->getCudaContext();
    PxScopedCudaLock _lock(*ctxMgr);

    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    for (const InternalDatabase::Record& rec : db.getRecords())
    {
        if (rec.mType != omni::physx::ePTParticleSet || !rec.mInternalPtr)
            continue;
        InternalParticleSet* ps = reinterpret_cast<InternalParticleSet*>(rec.mInternalPtr);
        if (!ps->mEnabled || ps->mNumParticles == 0 || !ps->mParticleBuffer)
            continue;
        const uint32_t n = ps->mNumParticles;

        // Under the ovstage backend the per-frame USD write-back is off, so the
        // particle buffers are NOT downloaded by the stepper — pull them here.
        const GfMatrix4f worldToLocal(getWorldTransform(as, ps->mKey, PXR_NS::UsdTimeCode::Default()).GetInverse());

        // Local scratch for the device->host copy. This must NOT be ps->mPositions /
        // ps->mVelocities: those pinned buffers double as the HtoD upload source and
        // are consumed on the first step (mUploadDirtyFlags, InternalParticleSet::
        // uploadParticles). Copying the device buffer back into them before that first
        // upload would overwrite the authored data (incl. invMass in .w) with the
        // still-unseeded device buffer and permanently latch the set to zeros
        // (NvBugs 6481119). While an authored value is still pending upload we return
        // it straight from the host buffer, matching how rigid-body / deformable reads
        // report authored initial state before the first step.
        std::vector<PxVec4> scratch;

        if (wantPoints)
        {
            const PxVec4* src = nullptr;
            if ((ps->mUploadDirtyFlags & ParticleBufferFlags::ePOSITIONS) && ps->mPositions)
            {
                src = ps->mPositions; // authored positions not yet uploaded to the device
            }
            else
            {
                scratch.resize(n);
                cu->memcpyDtoHAsync(scratch.data(), CUdeviceptr(ps->mParticleBuffer->getPositionInvMasses()),
                                    size_t(n) * sizeof(PxVec4), 0);
                cu->streamSynchronize(0);
                src = scratch.data();
            }
            GroupStore g;
            g.isArray = true;
            g.floats.resize(size_t(n) * 3);
            for (uint32_t v = 0; v < n; ++v)
            {
                const PxVec4& p = src[v];
                const GfVec3f lp = worldToLocal.Transform(GfVec3f(p.x, p.y, p.z));
                g.floats[v * 3 + 0] = lp[0];
                g.floats[v * 3 + 1] = lp[1];
                g.floats[v * 3 + 2] = lp[2];
            }
            finalizeGroup(s, source, std::move(g), omni::physx::OvxAttr::kPoints, { ps->mKey }, n, 3);
        }
        if (wantVel)
        {
            const PxVec4* src = nullptr;
            if ((ps->mUploadDirtyFlags & ParticleBufferFlags::eVELOCITIES) && ps->mVelocities)
            {
                src = ps->mVelocities; // authored velocities not yet uploaded to the device
            }
            else
            {
                scratch.resize(n);
                cu->memcpyDtoHAsync(scratch.data(), CUdeviceptr(ps->mParticleBuffer->getVelocities()),
                                    size_t(n) * sizeof(PxVec4), 0);
                cu->streamSynchronize(0);
                src = scratch.data();
            }
            GroupStore g;
            g.isArray = true;
            g.floats.resize(size_t(n) * 3);
            for (uint32_t v = 0; v < n; ++v)
            {
                const PxVec4& vel = src[v];
                g.floats[v * 3 + 0] = vel.x;
                g.floats[v * 3 + 1] = vel.y;
                g.floats[v * 3 + 2] = vel.z;
            }
            finalizeGroup(s, source, std::move(g), omni::physx::OvxAttr::kVelocities, { ps->mKey }, n, 3);
        }
    }
}

// ----------------------------------------------------------------------------
// Per-type dispatch
// ----------------------------------------------------------------------------

void buildGroups(ReadSession& s,
                 IPhysicsSource& source,
                 AttachedStage& as,
                 uint32_t type,
                 uint32_t scope,
                 const std::vector<std::string>& names)
{
    using namespace omni::physx;
    switch (type)
    {
    case kOvxRigidBody:
        buildRigidBodyGroups(s, source, as, scope, names);
        break;
    case kOvxArticulationLink:
        buildArticulationLinkGroups(s, source, scope, names);
        break;
    case kOvxArticulationJoint:
        buildJointStateGroups(s, source, names);
        break;
    case kOvxVehicleWheel:
        buildVehicleWheelGroups(s, source, scope, names);
        break;
    case kOvxDeformableVolume:
    case kOvxDeformableSurface:
        buildDeformableGroups(s, source, type, scope, names);
        break;
    case kOvxParticleSet:
        buildParticleGroups(s, source, as, names);
        break;
    default:
        CARB_LOG_WARN("ovxReadAttributes: object type %u not yet implemented.", type);
        break;
    }
}

// Output attribute names available for a query type (for ovxFetchQueryResult).
std::vector<std::string> attrNamesForType(uint32_t type)
{
    using namespace omni::physx;
    switch (type)
    {
    case kOvxRigidBody:
    case kOvxArticulationLink:
        return { OvxAttr::kPosition, OvxAttr::kOrientation, OvxAttr::kLinearVelocity, OvxAttr::kAngularVelocity };
    case kOvxArticulationJoint:
        return { OvxAttr::kJointPosition, OvxAttr::kJointVelocity };
    case kOvxVehicleWheel:
        return { OvxAttr::kPosition, OvxAttr::kOrientation };
    case kOvxDeformableVolume:
    case kOvxDeformableSurface:
    case kOvxParticleSet:
        return { OvxAttr::kPoints, OvxAttr::kVelocities };
    default:
        return {};
    }
}

// Distinct matched prims for a query type+scope (for ovxFetchQueryResult: count +
// a seed key to reach the dictionary).
std::vector<ObjectKey> collectMatchedKeys(AttachedStage& as, uint32_t type, uint32_t scope)
{
    using namespace omni::physx;
    std::vector<ObjectKey> keys;
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (type == kOvxRigidBody)
    {
        std::vector<StandaloneBody> standalone;
        std::vector<InstancerAccum> instancers;
        enumerateRigidBodies(as, scope, standalone, instancers);
        for (const StandaloneBody& b : standalone)
            keys.push_back(b.key);
        for (const InstancerAccum& a : instancers)
            keys.push_back(a.instancerKey);
    }
    else if (type == kOvxArticulationLink)
    {
        for (const InternalDatabase::Record& rec : db.getRecords())
            if (rec.mType == ePTLink && rec.mPtr &&
                reinterpret_cast<PxRigidActor*>(rec.mPtr)->is<PxArticulationLink>())
                keys.push_back(rec.mKey);
    }
    else if (type == kOvxArticulationJoint)
    {
        for (const InternalDatabase::Record& rec : db.getRecords())
            if (rec.mType == ePTLinkJoint && rec.mPtr)
                keys.push_back(rec.mKey);
    }
    else if (type == kOvxVehicleWheel)
    {
        std::vector<StandaloneBody> wheels;
        enumerateVehicleWheels(scope, wheels);
        for (const StandaloneBody& w : wheels)
            keys.push_back(w.key);
    }
    else if (type == kOvxDeformableVolume || type == kOvxDeformableSurface)
    {
        for (const InternalDatabase::Record& rec : db.getRecords())
        {
            if (rec.mType != ePTScene || !rec.mInternalPtr)
                continue;
            InternalScene* sc = reinterpret_cast<InternalScene*>(rec.mInternalPtr);
            if (type == kOvxDeformableVolume)
                for (InternalVolumeDeformableBody* b : sc->mVolumeDeformableBodies)
                    keys.push_back(b->mSimMeshKey);
            else
                for (InternalSurfaceDeformableBody* b : sc->mSurfaceDeformableBodies)
                    keys.push_back(b->mSimMeshKey);
        }
    }
    else if (type == kOvxParticleSet)
    {
        for (const InternalDatabase::Record& rec : db.getRecords())
            if (rec.mType == ePTParticleSet && rec.mInternalPtr)
                keys.push_back(reinterpret_cast<InternalParticleSet*>(rec.mInternalPtr)->mKey);
    }
    return keys;
}

} // namespace

namespace omni::physx
{

OvxOutputQueryHandle ovxQuery(uint32_t type, uint32_t scope)
{
    ActiveContext ctx;
    if (!getActiveContext("ovxQuery", ctx))
        return 0;

    std::lock_guard<std::mutex> lock(g_mutex);
    const uint64_t h = g_nextQuery++;
    QueryState qs;
    qs.type = type;
    qs.scope = scope;
    qs.dict = sourceDictionary(*ctx.source);
    g_queries[h] = std::move(qs);
    return h;
}

ovx_path_dictionary_t* ovxQueryDictionary(OvxOutputQueryHandle query)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_queries.find(query);
    return it == g_queries.end() ? nullptr : it->second.dict;
}

bool ovxFetchQueryResult(OvxOutputQueryHandle query, ovstage_query_result_t* outResult)
{
    if (!outResult)
        return false;
    *outResult = ovstage_query_result_t{};

    ActiveContext ctx;
    if (!getActiveContext("ovxFetchQueryResult", ctx))
        return false;

    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_queries.find(query);
    if (it == g_queries.end())
    {
        CARB_LOG_ERROR("ovxFetchQueryResult: invalid query handle %llu.", (unsigned long long)query);
        return false;
    }
    QueryState& q = it->second;

    const std::vector<ObjectKey> matched = collectMatchedKeys(*ctx.stage, q.type, q.scope);
    outResult->total_prim_count = matched.size();

    // Intern the type's attribute names into the shared source dictionary (tokens
    // are dict-lifetime, valid until ovxReleaseQuery).
    const std::vector<std::string> names = attrNamesForType(q.type);
    q.resultAttrs.clear();
    if (!names.empty() && q.dict)
        for (const std::string& n : names)
            q.resultAttrs.push_back(internToken(q.dict, n));
    outResult->attributes = q.resultAttrs.empty() ? nullptr : q.resultAttrs.data();
    outResult->attribute_count = q.resultAttrs.size();
    return true;
}

OvxReadHandle ovxReadAttributes(OvxOutputQueryHandle query, const ovx_string_or_token_t* attrs, size_t attrCount)
{
    if (attrCount && !attrs)
    {
        CARB_LOG_ERROR("ovxReadAttributes: attrs must be non-null when attrCount (%zu) > 0.", attrCount);
        return 0;
    }
    ActiveContext ctx;
    if (!getActiveContext("ovxReadAttributes", ctx))
        return 0;

    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_queries.find(query);
    if (it == g_queries.end())
    {
        CARB_LOG_ERROR("ovxReadAttributes: invalid query handle %llu.", (unsigned long long)query);
        return 0;
    }
    const QueryState q = it->second;

    // Resolve attribute names (string-or-token → string). Discovery
    // (ovxFetchQueryResult) hands back interned tokens, so a caller may feed those
    // tokens straight back here; a token is resolved to its canonical string
    // through the query's shared source dictionary. The string form remains a
    // convenience for callers that did not go through discovery.
    std::vector<std::string> names;
    names.reserve(attrCount);
    for (size_t i = 0; i < attrCount; ++i)
    {
        if (attrs[i].token != OVX_INVALID_TOKEN)
        {
            ovx_string_t resolved{ nullptr, 0 };
            if (q.dict &&
                ovx_path_dictionary_token_to_string(q.dict, attrs[i].token, &resolved) == OVX_OK &&
                resolved.ptr && resolved.length)
            {
                names.emplace_back(resolved.ptr, resolved.length);
            }
            else
            {
                CARB_LOG_WARN("ovxReadAttributes: attribute token %llu could not be resolved against the "
                              "query dictionary — skipped.",
                              (unsigned long long)attrs[i].token);
            }
        }
        else if (attrs[i].string.ptr)
        {
            names.emplace_back(attrs[i].string.ptr, attrs[i].string.length);
        }
    }

    ReadSession s;
    buildGroups(s, *ctx.source, *ctx.stage, q.type, q.scope, names);

    const uint64_t h = g_nextRead++;
    g_reads[h] = std::move(s);
    return h;
}

OvxReadStatus ovxFetchReadNext(OvxReadHandle read, ovstage_read_group_t* outGroup)
{
    if (!outGroup)
        return kOvxReadStatusError;
    *outGroup = ovstage_read_group_t{};

    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_reads.find(read);
    if (it == g_reads.end())
    {
        CARB_LOG_ERROR("ovxFetchReadNext: invalid read handle %llu.", (unsigned long long)read);
        return kOvxReadStatusError;
    }
    ReadSession& s = it->second;
    if (s.cursor >= s.groups.size())
        return kOvxReadStatusEndOfIteration; // all groups consumed (not an error)

    GroupStore& g = s.groups[s.cursor];
    g.tensor.shape = g.shape.data(); // stable address inside the stored group

    outGroup->read_group_id = (ovstage_read_group_id_t)(s.cursor + 1);
    outGroup->attribute = g.attribute;
    outGroup->ordinal = 0;
    outGroup->is_delete = false;
    outGroup->is_array = g.isArray;
    outGroup->semantic = g.semantic;
    outGroup->prims = ovstage_prim_group_t{ g.list, 0, g.primCount, nullptr };
    outGroup->data = ovstage_data_t{};
    outGroup->data.tensors = &g.tensor;
    outGroup->data.tensor_count = 1;
    if (!g.indexMap.empty())
    {
        outGroup->data.count = (uint32_t)g.indexMap.size();
        outGroup->data.index_map = g.indexMap.data();
    }
    outGroup->meta = ovstage_attribute_meta_t{};

    ++s.cursor;
    return kOvxReadStatusOk;
}

void ovxReleaseGroup(OvxReadHandle /*read*/, const ovstage_read_group_t* /*group*/)
{
    // Scratch + prim lists are owned by the read session and freed by
    // ovxReleaseRead; per-group release is a no-op for this borrow-then-release
    // session model (the gathered columns are valid until ovxReleaseRead).
}

void ovxReleaseRead(OvxReadHandle read)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    auto it = g_reads.find(read);
    if (it == g_reads.end())
        return;
    ReadSession& s = it->second;
    for (GroupStore& g : s.groups)
        if (g.list != OVX_INVALID_PRIMPATH_LIST && s.dict)
            ovx_path_dictionary_destroy_path_list(s.dict, g.list);
    g_reads.erase(it);
}

void ovxReleaseQuery(OvxOutputQueryHandle query)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    g_queries.erase(query);
}

} // namespace omni::physx
