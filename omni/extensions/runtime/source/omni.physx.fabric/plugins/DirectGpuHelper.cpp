// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DirectGpuHelper.h"
#include "FabricManager.h"
#include "FabricKernels.h"

#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <carb/Framework.h>
#include <carb/container/RHUnorderedMap.h>

#include <omni/physx/IPhysxFoundation.h>

#include <omni/fabric/connectivity/Connectivity.h>
#include <omni/fabric/FabricUSD.h>
#include <usdrt/hierarchy/IFabricHierarchy.h>
#include <carb/settings/ISettings.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/PhysxTokens.h>
#include <pxr/usd/usdPhysics/tokens.h>
#include <physxSchema/tokens.h>

#include "common/foundation/TypeCast.h"

#include <common/utilities/CudaShimWrappers.h>

#include <PxPhysicsAPI.h>

#include <algorithm>
#include <unordered_set>
#include <vector>

using namespace pxr;
using namespace physx;
using namespace carb;
using omni::physx::cudaShimWrappers::getCudaShim;

extern omni::physx::FabricManager* gFabricManager;

extern pxr::GfMatrix4d computeMatrix(const pxr::GfVec3d& translate,
                                     const pxr::GfMatrix3d& rotate,
                                     const pxr::GfVec3d& scale);

extern pxr::GfMatrix4d computeLocalMatrix(omni::fabric::USDHierarchy& usdHierarchy,
                                          omni::fabric::StageReaderWriter& stage,
                                          const omni::fabric::Path& path,
                                          const GfMatrix4d& worldMatrix,
                                          const omni::fabric::Token& worldMatrixToken);

namespace omni
{
namespace physx
{

DirectGpuHelper::DirectGpuHelper()
{
    mRigidBodySchemaToken = omni::fabric::Token::createImmortal(gRigidBodyAPITokenString);

    mWorldMatrixToken = omni::fabric::Token::createImmortal(gWorldMatrixTokenString);

    mLinVelToken = omni::fabric::Token::createImmortal(UsdPhysicsTokens->physicsVelocity.GetText());
    mAngVelToken = omni::fabric::Token::createImmortal(UsdPhysicsTokens->physicsAngularVelocity.GetText());

    mRigidBodyWorldPositionToken = omni::fabric::Token::createImmortal(gRigidBodyWorldPositionTokenString);
    mRigidBodyWorldOrientationToken = omni::fabric::Token::createImmortal(gRigidBodyWorldOrientationTokenString);
    mRigidBodyWorldScaleToken = omni::fabric::Token::createImmortal(gRigidBodyWorldScaleTokenString);

    using omni::fabric::BaseDataType;
    using omni::fabric::AttributeRole;

    mTypeAppliedSchema = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);

    mTypeFloat3 = omni::fabric::Type(BaseDataType::eFloat, 3, 0, AttributeRole::eNone);
    mTypeDouble3 = omni::fabric::Type(BaseDataType::eDouble, 3, 0, AttributeRole::eNone);
    mTypeMatrix4d = omni::fabric::Type(BaseDataType::eDouble, 16, 0, AttributeRole::eMatrix);
    mTypeQuat = omni::fabric::Type(BaseDataType::eFloat, 4, 0, AttributeRole::eQuaternion);

    mPhysxSimulationInterface = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
    mUsdStageId.id = 0;
}

DirectGpuHelper::~DirectGpuHelper()
{
}

void DirectGpuHelper::attach(unsigned long usdStageId)
{
    mAttachTimestamp = mPhysxSimulationInterface->getSimulationTimestamp();
    //printf("~!~!~! Attaching at step %u\n", unsigned(mAttachTimestamp));
    mUsdStageId.id = usdStageId;
    isTopologyConnectivityInitialized = false;
}

void DirectGpuHelper::detach()
{
    releaseBuffers();

    *this = DirectGpuHelper();
}

void DirectGpuHelper::registerRigidBody(const omni::fabric::Path& primPath, const carb::Float3& scale)
{
    //printf("+++ Registering GPU body %s\n", path.GetText());

    // NOTE: we can't get all the indexing data if the body is not in a scene yet, so just remember the path
    mRigidBodies[primPath] = RigidBodyData();

    // Slightly faster to check scale here
    if (scale.x != 1.0f || scale.y != 1.0f || scale.z != 1.0f)
    {
        mInitialScales[primPath] = scale;
    }

    mNeedResync = true;
}

bool DirectGpuHelper::registerScene(PxScene* scene)
{
    if (!scene)
    {
        CARB_LOG_ERROR("DirectGpuHelper: Invalid physics scene!");
        return false;
    }

    // remember the scene and make sure everything is in the same scene
    if (!mScene)
    {
        mScene = scene;
    }
    else if (scene != mScene)
    {
        CARB_LOG_ERROR("DirectGpuHelper: Multiple physics scenes not supported!");
        return false;
    }

    return true;
}

bool DirectGpuHelper::prepareBuffers(omni::fabric::StageReaderWriter& srw, bool fullFabricGpuInterop)
{
    if (!mNeedResync)
    {
        return true;
    }

    releaseBuffers();

    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();

    // track required articulation data
    PxI64 maxArtiIdx = -1;
    PxU32 maxLinks = 0;
    carb::container::RHUnorderedMap<PxArticulationReducedCoordinate*, PxU32> artis;

    std::vector<PxArticulationGPUIndex> artiIndices;
    artiIndices.reserve(1024);
    std::vector<RigidBodyData*> linkEntries;
    linkEntries.reserve(1024);

    // track required rigid dynamic data
    std::vector<PxRigidDynamicGPUIndex> rdIndices;
    rdIndices.reserve(1024);

    // initialize rigid body indexing data
    for (auto it = mRigidBodies.begin(); it != mRigidBodies.end(); ++it)
    {
        const SdfPath path = omni::fabric::toSdfPath(it->first);
        RigidBodyData& rbData = it->second;

        // check if it's an articulation link
        PxArticulationLink* link = reinterpret_cast<PxArticulationLink*>(iPhysX->getPhysXPtr(path, omni::physx::ePTLink));
        if (link)
        {
            PxArticulationReducedCoordinate* arti = &link->getArticulation();

            PxU32 artiOffset = 0xFFFFFFFF;

            auto it = artis.find(arti);

            // we already saw this articulation, retrieve index
            if (it != artis.end())
            {
                artiOffset = it->second;
            }
            else
            {
                if (!registerScene(arti->getScene()))
                {
                    continue;
                }

                // track max links
                PxU32 numLinks = arti->getNbLinks();
                if (numLinks > maxLinks)
                {
                    maxLinks = numLinks;
                }

                artiOffset = PxU32(artiIndices.size());

                PxArticulationGPUIndex artiIdx = arti->getGPUIndex();
                artiIndices.push_back(artiIdx);

                artis[arti] = artiOffset;
            }

            CARB_ASSERT(artiOffset != 0xFFFFFFFF);

            rbData.isArticulationLink = true;
            rbData.link.artiIdx = artiOffset; // this is the offset into the index/data arrays passed to the direct-GPU
                                              // API.
            rbData.link.linkIdx = link->getLinkIndex();
            // NOTE: we can't initialize rbData.link.linkOffset until the sizes of all articulations are known

            // ...that's why we track the link entries here so we can do another pass
            linkEntries.push_back(&rbData);
        }
        else
        {
            // check if it's a rigid dynamic
            PxBase* actor = reinterpret_cast<PxBase*>(iPhysX->getPhysXPtr(path, omni::physx::ePTActor));
            if (actor && actor->is<PxRigidDynamic>())
            {
                PxRigidDynamic* rd = static_cast<PxRigidDynamic*>(actor);

                if (!registerScene(rd->getScene()))
                {
                    continue;
                }

                PxU32 rdIndex = PxU32(rdIndices.size());
                PxRigidDynamicGPUIndex gpuIndex = rd->getGPUIndex();
                rdIndices.push_back(gpuIndex);

                rbData.isArticulationLink = false;
                rbData.rd.idx = rdIndex;
            }
        }
    }

    // now that we have maxLinks, we can initialize the link offsets
    for (RigidBodyData* rb : linkEntries)
    {
        rb->link.linkOffset = rb->link.artiIdx * maxLinks + rb->link.linkIdx;
    }

    mNumRds = PxU32(rdIndices.size());
    mNumArtis = PxU32(artiIndices.size());
    mMaxLinks = maxLinks;
    mLinkBufSize = mNumArtis * mMaxLinks;

    /*
    printf("~*~*~*\n");
    printf("~*~*~* Direct GPU Helper:\n");
    if (mNumRds > 0)
    {
        printf("~*~*~*   %u rigid dynamic%s\n", mNumRds, (mNumRds == 1 ? "" : "s"));
    }
    if (mNumArtis > 0)
    {
        printf("~*~*~*   %u articulation%s, maxLinks=%u\n", mNumArtis, (mNumArtis == 1 ? "" : "s"), mMaxLinks);
    }
    printf("~*~*~*\n");
    */

    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();
    if (!cudaContextManager)
        return false;

    PxScopedCudaLock _lock(*cudaContextManager);

    // allocate articulation buffers
    if (mNumArtis > 0)
    {
        if (!SHIM_CU_ALLOC(&mArtiIndicesDev, artiIndices.size() * sizeof(PxU32)))
        {
            return false;
        }
        if (!CHECK_CUDA_SHIM(getCudaShim()->memcpyHtoD(static_cast<uintptr_t>(mArtiIndicesDev), artiIndices.data(), artiIndices.size() * sizeof(PxU32), nullptr)))
        {
            return false;
        }

        // link transforms
        if (!fullFabricGpuInterop)
        {
            mLinkTransforms.resize(mLinkBufSize);
        }
        if (!SHIM_CU_ALLOC(&mLinkTransformsDev, mLinkBufSize * sizeof(PxTransform)))
        {
            return false;
        }
        CHECK_CUDA_SHIM(getCudaShim()->memsetD8(static_cast<uintptr_t>(mLinkTransformsDev), 0, mLinkBufSize * sizeof(PxTransform), nullptr));

        // link linear velocities
        if (!fullFabricGpuInterop)
        {
            mLinkLinearVelocities.resize(mLinkBufSize);
        }
        if (!SHIM_CU_ALLOC(&mLinkLinearVelocitiesDev, mLinkBufSize * sizeof(PxVec3)))
        {
            return false;
        }
        CHECK_CUDA_SHIM(getCudaShim()->memsetD8(static_cast<uintptr_t>(mLinkLinearVelocitiesDev), 0, mLinkBufSize * sizeof(PxVec3), nullptr));

        // link angular velocities
        if (!fullFabricGpuInterop)
        {
            mLinkAngularVelocities.resize(mLinkBufSize);
        }
        if (!SHIM_CU_ALLOC(&mLinkAngularVelocitiesDev, mLinkBufSize * sizeof(PxVec3)))
        {
            return false;
        }
        CHECK_CUDA_SHIM(getCudaShim()->memsetD8(static_cast<uintptr_t>(mLinkAngularVelocitiesDev), 0, mLinkBufSize * sizeof(PxVec3), nullptr));


        // synchronization events
        if (!SHIM_CU_EVENT_CREATE(&mLinkTransformsStartEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mLinkTransformsCopyEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mLinkLinearVelocitiesStartEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mLinkAngularVelocitiesStartEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mLinkLinearVelocitiesCopyEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mLinkAngularVelocitiesCopyEvent, CU_EVENT_DISABLE_TIMING))
        {
            return false;
        }
    }

    // allocate rigid dynamic buffers
    if (mNumRds > 0)
    {
        // actor indices
        if (!SHIM_CU_ALLOC(&mRdIndicesDev, mNumRds * sizeof(PxRigidDynamicGPUIndex)))
        {
            return false;
        }
        if (!CHECK_CUDA_SHIM(getCudaShim()->memcpyHtoD(static_cast<uintptr_t>(mRdIndicesDev), rdIndices.data(), mNumRds * sizeof(PxRigidDynamicGPUIndex), nullptr)))
        {
            return false;
        }

        // actor data
        if (!fullFabricGpuInterop)
        {
            mRdTransforms.resize(mNumRds);
        }
        if (!SHIM_CU_ALLOC(&mRdTransformsDev, mNumRds * sizeof(PxTransform)))
        {
            return false;
        }
        CHECK_CUDA_SHIM(getCudaShim()->memsetD8(static_cast<uintptr_t>(mRdTransformsDev), 0, mNumRds * sizeof(PxTransform), nullptr));

        if (!fullFabricGpuInterop)
        {
            mRdLinearVelocities.resize(mNumRds);
        }
        if (!SHIM_CU_ALLOC(&mRdLinearVelocitiesDev, mNumRds * sizeof(PxVec3)))
        {
            return false;
        }
        CHECK_CUDA_SHIM(getCudaShim()->memsetD8(static_cast<uintptr_t>(mRdLinearVelocitiesDev), 0, mNumRds * sizeof(PxVec3), nullptr));

        if (!fullFabricGpuInterop)
        {
            mRdAngularVelocities.resize(mNumRds);
        }
        if (!SHIM_CU_ALLOC(&mRdAngularVelocitiesDev, mNumRds * sizeof(PxVec3)))
        {
            return false;
        }
        CHECK_CUDA_SHIM(getCudaShim()->memsetD8(static_cast<uintptr_t>(mRdAngularVelocitiesDev), 0, mNumRds * sizeof(PxVec3), nullptr));

        // synchronization events
        if (!SHIM_CU_EVENT_CREATE(&mRdTransformsStartEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mRdTransformsCopyEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mRdLinearVelocityStartEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mRdLinearVelocityCopyEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mRdAngularVelocityStartEvent, CU_EVENT_DISABLE_TIMING) ||
            !SHIM_CU_EVENT_CREATE(&mRdAngularVelocityCopyEvent, CU_EVENT_DISABLE_TIMING))
        {
            return false;
        }
    }

    if (fullFabricGpuInterop)
    {
        CARB_PROFILE_ZONE(1, "Build Fabric GPU mapping data");

        if (mCopyStream)
        {
            CHECK_CUDA_SHIM(getCudaShim()->streamDestroy(reinterpret_cast<uintptr_t>(mCopyStream), nullptr));
        }
        if (!SHIM_CU_STREAM_CREATE(&mCopyStream, 0))
        {
            return false;
        }

        // prepare initial scales buffer
        // rigid bodies first, then articulations in the same buffer
        if (mNumRds + mLinkBufSize != 0)
        {
            if (!SHIM_CU_ALLOC(&mInitialScalesDev, (mNumRds + mLinkBufSize) * sizeof(carb::Float3)))
            {
                return false;
            }
            vector<carb::Float3> initialScales(mNumRds + mLinkBufSize, {1.0, 1.0, 1.0});
            const size_t artOffset = mRigidBodies.size();
            for (auto& pathScale : mInitialScales)
            {
                auto itRb = mRigidBodies.find(pathScale.first);
                if (itRb == mRigidBodies.end())
                {
                    continue;
                }
                auto& rb = itRb->second;
                if (!itRb->second.isArticulationLink && rb.rd.idx < mNumRds)
                {
                    initialScales[rb.rd.idx] = pathScale.second;
                }
                else if (itRb->second.isArticulationLink && rb.link.linkOffset < mLinkBufSize)
                {
                    initialScales[rb.link.linkOffset + mNumRds] = pathScale.second;
                }
            }
            if (!CHECK_CUDA_SHIM(getCudaShim()->memcpyHtoD(static_cast<uintptr_t>(mInitialScalesDev), initialScales.data(), initialScales.size() * sizeof(carb::Float3), nullptr)))
            {
                return false;
            }
        }

        // Prepare GPU mappings from physics data to Fabric
        if (!prepareMappings(srw))
        {
            return false;
        }
    }

    mNeedResync = false;

    return true;
}

// Note: This must be called when the right Cuda context/device is set!
bool DirectGpuHelper::prepareMappings(omni::fabric::StageReaderWriter& srw)
{
    CARB_PROFILE_ZONE(1, "FabricMapper build");
    omni::fabric::FabricId fabricId = srw.getFabricId();
    if (fabricId == omni::fabric::kInvalidFabricId)
    {
        CARB_LOG_ERROR("Invalid Fabric Id, cannot initialize GPU transform interop");
        return false;
    }

    int deviceId = -1;
    if (!CHECK_CUDA_SHIM(getCudaShim()->ctxGetDevice(&deviceId, nullptr)))
        return false;

    auto iFabricConnectivity = carb::getCachedInterface<omni::fabric::IConnectivity>();
    auto iFabric = carb::getCachedInterface<omni::fabric::IFabric>();

    if (mNumRds > 0 || mLinkBufSize > 0)
    {
        using omni::fabric::AttrNameAndType;
        const omni::fabric::set<AttrNameAndType> requiredAll = { AttrNameAndType(
            mTypeAppliedSchema, mRigidBodySchemaToken) };
        const omni::fabric::set<AttrNameAndType> requiredAny = {
            AttrNameAndType(mTypeMatrix4d, mWorldMatrixToken),
            AttrNameAndType(mTypeFloat3, mLinVelToken),
            AttrNameAndType(mTypeFloat3, mAngVelToken),
            AttrNameAndType(mTypeDouble3, mRigidBodyWorldPositionToken),
            AttrNameAndType(mTypeQuat, mRigidBodyWorldOrientationToken),
            AttrNameAndType(mTypeFloat3, mRigidBodyWorldScaleToken)
        };
        omni::fabric::PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);

        if (mNumRds > 0)
        {
            mFabricMapperRb.bindFabric(srw.getFabricId());
            mFabricMapperRb.bindDevice(deviceId);
            auto getRigidBodyIndex = [&](const RigidBodyData& rb) -> size_t {
                if (rb.isArticulationLink)
                {
                    return mNumRds + 1;
                }
                return rb.rd.idx;
            };

            mFabricMapperRb.build<RigidBodyData, pxr::GfMatrix4d>(mRigidBodies, mNumRds, getRigidBodyIndex, mWorldMatrixToken, primBuckets);
            mFabricMapperRb.build<RigidBodyData, carb::Float3>(mRigidBodies, mNumRds, getRigidBodyIndex, mLinVelToken, primBuckets);
            mFabricMapperRb.build<RigidBodyData, carb::Float3>(mRigidBodies, mNumRds, getRigidBodyIndex, mAngVelToken, primBuckets);
            mFabricMapperRb.build<RigidBodyData, carb::Double3>(mRigidBodies, mNumRds, getRigidBodyIndex, mRigidBodyWorldPositionToken, primBuckets);
            mFabricMapperRb.build<RigidBodyData, carb::Float4>(mRigidBodies, mNumRds, getRigidBodyIndex, mRigidBodyWorldOrientationToken, primBuckets);
            mFabricMapperRb.build<RigidBodyData, carb::Float3>(mRigidBodies, mNumRds, getRigidBodyIndex, mRigidBodyWorldScaleToken, primBuckets);
        }

        if (mLinkBufSize > 0)
        {
            mFabricMapperArticulation.bindFabric(srw.getFabricId());
            mFabricMapperArticulation.bindDevice(deviceId);
            auto getArticulationIndex = [&](const RigidBodyData& rb) -> size_t {
                if (rb.isArticulationLink)
                {
                    return rb.link.linkOffset;
                }
                return mLinkBufSize + 1;
            };

            mFabricMapperArticulation.build<RigidBodyData, pxr::GfMatrix4d>(mRigidBodies, mLinkBufSize, getArticulationIndex, mWorldMatrixToken, primBuckets);
            mFabricMapperArticulation.build<RigidBodyData, carb::Float3>(mRigidBodies, mLinkBufSize, getArticulationIndex, mLinVelToken, primBuckets);
            mFabricMapperArticulation.build<RigidBodyData, carb::Float3>(mRigidBodies, mLinkBufSize, getArticulationIndex, mAngVelToken, primBuckets);
            mFabricMapperArticulation.build<RigidBodyData, carb::Double3>(mRigidBodies, mLinkBufSize, getArticulationIndex, mRigidBodyWorldPositionToken, primBuckets);
            mFabricMapperArticulation.build<RigidBodyData, carb::Float4>(mRigidBodies, mLinkBufSize, getArticulationIndex, mRigidBodyWorldOrientationToken, primBuckets);
            mFabricMapperArticulation.build<RigidBodyData, carb::Float3>(mRigidBodies, mLinkBufSize, getArticulationIndex, mRigidBodyWorldScaleToken, primBuckets);
        }
    }
    
#if KIT_SDK_VERSION < 1090002
    mFabricTopologyVersion = iFabric->getTopologyVersion(fabricId);
#else
    mFabricTopologyVersion = iFabric->getCounter(fabricId, omni::fabric::Counter::eBucketTopology);
#endif
           
    mFabricConnectivityVersion = iFabricConnectivity->getDataRevision(fabricId);
    if(!isTopologyConnectivityInitialized)
        isTopologyConnectivityInitialized = true;

    return true;
}

void DirectGpuHelper::releaseBuffers()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (cudaContextManager)
    {
        PxScopedCudaLock _lock(*cudaContextManager);

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mArtiIndicesDev), nullptr));
        mArtiIndicesDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mLinkTransformsDev), nullptr));
        mLinkTransformsDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mLinkLinearVelocitiesDev), nullptr));
        mLinkLinearVelocitiesDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mLinkAngularVelocitiesDev), nullptr));
        mLinkAngularVelocitiesDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mRdIndicesDev), nullptr));
        mRdIndicesDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mRdTransformsDev), nullptr));
        mRdTransformsDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mRdLinearVelocitiesDev), nullptr));
        mRdLinearVelocitiesDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mRdAngularVelocitiesDev), nullptr));
        mRdAngularVelocitiesDev = 0;

        CHECK_CUDA_SHIM(getCudaShim()->memFree(static_cast<uintptr_t>(mInitialScalesDev), nullptr));
        mInitialScalesDev = 0;

        // Destroy synchronization events (created in prepareBuffers).
        auto destroyEvent = [](CUevent& evt) {
            if (evt)
            {
                omni::physx::cudaShimWrappers::getCudaShim()->eventDestroy(reinterpret_cast<uintptr_t>(evt), nullptr);
                evt = nullptr;
            }
        };
        destroyEvent(mLinkTransformsStartEvent);
        destroyEvent(mLinkTransformsCopyEvent);
        destroyEvent(mLinkLinearVelocitiesStartEvent);
        destroyEvent(mLinkLinearVelocitiesCopyEvent);
        destroyEvent(mLinkAngularVelocitiesStartEvent);
        destroyEvent(mLinkAngularVelocitiesCopyEvent);
        destroyEvent(mRdTransformsStartEvent);
        destroyEvent(mRdTransformsCopyEvent);
        destroyEvent(mRdLinearVelocityStartEvent);
        destroyEvent(mRdLinearVelocityCopyEvent);
        destroyEvent(mRdAngularVelocityStartEvent);
        destroyEvent(mRdAngularVelocityCopyEvent);
    }

    mScene = nullptr;

    mNumRds = 0;
    mNumArtis = 0;
    mMaxLinks = 0;
    mLinkBufSize = 0;

    mLinkTransforms.clear();
    mLinkLinearVelocities.clear();
    mLinkAngularVelocities.clear();
    mRdTransforms.clear();
    mRdLinearVelocities.clear();
    mRdAngularVelocities.clear();

    // NOTE: no need to skip this is the setting is off (it's cheap)
    mFabricMapperRb.clear();
    mFabricMapperArticulation.clear();
    if (mCopyStream)
    {
        CHECK_CUDA_SHIM(getCudaShim()->streamDestroy(reinterpret_cast<uintptr_t>(mCopyStream), nullptr));
        mCopyStream = nullptr;
    }

    if (mCubricAdapter)
    {
        IPhysxFoundation* iFoundation = carb::getCachedInterface<IPhysxFoundation>();
        if (iFoundation)
            iFoundation->cubricReleaseAdapter(mCubricAdapter);
        mCubricAdapter = nullptr;
    }
}

void DirectGpuHelper::clear()
{
    releaseBuffers();
    mRigidBodies.clear();
    mInitialScales.clear();
}

void DirectGpuHelper::update(omni::fabric::StageReaderWriter& srw, bool forceUpdate)
{
    uint64_t timestamp = mPhysxSimulationInterface->getSimulationTimestamp();
    //printf("~!~!~! Update called at step %u\n", unsigned(timestamp));

    // The PhysX direct GPU API needs a warm start.
    // We need at least one call to simulate()/fetchResults() before using it, otherwise it crashes.
    if (timestamp > mAttachTimestamp)
    {
        // get current settings
        carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
        bool updateTransforms = iSettings->getAsBool(kSettingFabricUpdateTransformations);
        bool updateVelocities = iSettings->getAsBool(kSettingFabricUpdateVelocities);
        bool fullFabricGpuInterop = iSettings->getAsBool(kSettingFabricUseGPUInterop);

        uint64_t timestamp = mPhysxSimulationInterface->getSimulationTimestamp();

        if (updateTransforms || updateVelocities)
        {
            // only update rigid bodies if at least one physics step was performed since the last fetch
            if (timestamp > mRbFetchTimestamp || forceUpdate)
            {
                //printf("~!~!~! Updating at step %u\n", unsigned(timestamp));
                fetchRigidBodyData(srw, updateTransforms, updateVelocities, fullFabricGpuInterop);

                PauseChangeTrackingScope changeTrackingPauseScope(mPhysxSimulationInterface);

                updateRigidBodies(srw, updateTransforms, updateVelocities, fullFabricGpuInterop);

                mRbFetchTimestamp = timestamp;
            }
            else
            {
                //printf("~!~!~! SKIPPING UPDATE\n");
            }
        }
    }
}

void DirectGpuHelper::fetchRigidBodyData(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities, bool fullFabricGpuInterop)
{
    if (!prepareBuffers(srw, fullFabricGpuInterop) || !mScene)
    {
        return;
    }

    //
    // launch PhysX kernels to copy state data
    //

    bool fetchingLinkTransforms = false;
    bool fetchingLinkVelocities = false;
    bool fetchingRdTransforms = false;
    bool fetchingRdVelocities = false;

    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();
    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);

    if (mNumArtis > 0 && mArtiIndicesDev)
    {
        if (updateTransforms && mLinkTransformsDev)
        {
            CHECK_CUDA_SHIM(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mLinkTransformsStartEvent), uintptr_t(0), nullptr));
            mScene->getDirectGPUAPI().getArticulationData(
                (void*)mLinkTransformsDev,
                (PxArticulationGPUIndex*)mArtiIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                mLinkTransformsStartEvent, mLinkTransformsCopyEvent);
            fetchingLinkTransforms = true;
        }
        if (updateVelocities && mLinkLinearVelocitiesDev && mLinkAngularVelocitiesDev)
        {
            CHECK_CUDA_SHIM(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mLinkLinearVelocitiesStartEvent), uintptr_t(0), nullptr));
            mScene->getDirectGPUAPI().getArticulationData(
                (void*)mLinkLinearVelocitiesDev, (PxArticulationGPUIndex*)mArtiIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY, mNumArtis, mLinkLinearVelocitiesStartEvent,
                mLinkLinearVelocitiesCopyEvent);

            CHECK_CUDA_SHIM(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mLinkAngularVelocitiesStartEvent), uintptr_t(0), nullptr));
            mScene->getDirectGPUAPI().getArticulationData(
                (void*)mLinkAngularVelocitiesDev, (PxArticulationGPUIndex*)mArtiIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_ANGULAR_VELOCITY, mNumArtis, mLinkLinearVelocitiesStartEvent,
                mLinkLinearVelocitiesCopyEvent);

            fetchingLinkVelocities = true;
        }
    }
    if (mNumRds > 0 && mRdIndicesDev)
    {
        if (mRdTransformsDev)
        {
            CHECK_CUDA_SHIM(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mRdTransformsStartEvent), uintptr_t(0), nullptr));
            mScene->getDirectGPUAPI().getRigidDynamicData(
                (void*)mRdTransformsDev, (PxRigidDynamicGPUIndex*)mRdIndicesDev,
                PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mNumRds, mRdTransformsStartEvent, mRdTransformsCopyEvent);

            fetchingRdTransforms = true;
        }

        if (mRdLinearVelocitiesDev && mRdAngularVelocitiesDev)
        {
            CHECK_CUDA_SHIM(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mRdLinearVelocityStartEvent), uintptr_t(0), nullptr));
            mScene->getDirectGPUAPI().getRigidDynamicData((void*)mRdLinearVelocitiesDev,
                                                          (PxRigidDynamicGPUIndex*)mRdIndicesDev,
                                                          PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY, mNumRds,
                                                          mRdLinearVelocityStartEvent, mRdLinearVelocityCopyEvent);

            CHECK_CUDA_SHIM(getCudaShim()->eventRecord(reinterpret_cast<uintptr_t>(mRdAngularVelocityStartEvent), uintptr_t(0), nullptr));
            mScene->getDirectGPUAPI().getRigidDynamicData((void*)mRdAngularVelocitiesDev,
                                                          (PxRigidDynamicGPUIndex*)mRdIndicesDev,
                                                          PxRigidDynamicGPUAPIReadType::eANGULAR_VELOCITY, mNumRds,
                                                          mRdAngularVelocityStartEvent, mRdAngularVelocityCopyEvent);


            fetchingRdVelocities = true;
        }
    }

    //
    // copy state data to host
    //

    if (fetchingLinkTransforms)
    {
        CHECK_CUDA_SHIM(getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(nullptr), reinterpret_cast<uintptr_t>(mLinkTransformsCopyEvent), 0, nullptr));
        if (!fullFabricGpuInterop)
        {
            CHECK_CUDA_SHIM(getCudaShim()->memcpyDtoH(mLinkTransforms.data(), static_cast<uintptr_t>(mLinkTransformsDev), mLinkBufSize * sizeof(PxTransform), nullptr));
        }
    }
    if (fetchingLinkVelocities)
    {
        CHECK_CUDA_SHIM(getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(nullptr), reinterpret_cast<uintptr_t>(mLinkLinearVelocitiesCopyEvent), 0, nullptr));
        CHECK_CUDA_SHIM(getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(nullptr), reinterpret_cast<uintptr_t>(mLinkAngularVelocitiesCopyEvent), 0, nullptr));
        if (!fullFabricGpuInterop)
        {
            CHECK_CUDA_SHIM(getCudaShim()->memcpyDtoH(mLinkLinearVelocities.data(), static_cast<uintptr_t>(mLinkLinearVelocitiesDev), mLinkBufSize * sizeof(PxVec3), nullptr));
            CHECK_CUDA_SHIM(getCudaShim()->memcpyDtoH(mLinkAngularVelocities.data(), static_cast<uintptr_t>(mLinkAngularVelocitiesDev), mLinkBufSize * sizeof(PxVec3), nullptr));
        }
    }
    if (fetchingRdTransforms)
    {
        CHECK_CUDA_SHIM(getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(nullptr), reinterpret_cast<uintptr_t>(mRdTransformsCopyEvent), 0, nullptr));
        if (!fullFabricGpuInterop)
        {
            CHECK_CUDA_SHIM(getCudaShim()->memcpyDtoH(mRdTransforms.data(), static_cast<uintptr_t>(mRdTransformsDev), mNumRds * sizeof(PxTransform), nullptr));
        }
    }
    if (fetchingRdVelocities)
    {
        CHECK_CUDA_SHIM(getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(nullptr), reinterpret_cast<uintptr_t>(mRdLinearVelocityCopyEvent), 0, nullptr));
        CHECK_CUDA_SHIM(getCudaShim()->streamWaitEvent(reinterpret_cast<uintptr_t>(nullptr), reinterpret_cast<uintptr_t>(mRdAngularVelocityCopyEvent), 0, nullptr));
        if (!fullFabricGpuInterop)
        {
            CHECK_CUDA_SHIM(getCudaShim()->memcpyDtoH(mRdLinearVelocities.data(), static_cast<uintptr_t>(mRdLinearVelocitiesDev), mNumRds * sizeof(PxVec3), nullptr));
            CHECK_CUDA_SHIM(getCudaShim()->memcpyDtoH(mRdAngularVelocities.data(), static_cast<uintptr_t>(mRdAngularVelocitiesDev), mNumRds * sizeof(PxVec3), nullptr));
        }
    }
}


void DirectGpuHelper::applyRigidBodiesToFabric_CPU(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities)
{
    using omni::fabric::AttrNameAndType;

    const omni::fabric::set<AttrNameAndType> requiredAll = { AttrNameAndType(mTypeAppliedSchema, mRigidBodySchemaToken) };
    const omni::fabric::set<AttrNameAndType> requiredAny = { AttrNameAndType(mTypeMatrix4d, mWorldMatrixToken),
                                                             AttrNameAndType(mTypeFloat3, mLinVelToken),
                                                             AttrNameAndType(mTypeFloat3, mAngVelToken),
                                                             AttrNameAndType(mTypeDouble3, mRigidBodyWorldPositionToken),
                                                             AttrNameAndType(mTypeQuat, mRigidBodyWorldOrientationToken),
                                                             AttrNameAndType(mTypeFloat3, mRigidBodyWorldScaleToken) };

    omni::fabric::PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);

    omni::fabric::USDHierarchy usdHierarchy(srw.getFabricId());

    size_t bucketCount = primBuckets.bucketCount();
    //printf("+++ Bucket count %u\n", unsigned(bucketCount));
    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<pxr::GfMatrix4d> worldMatrices = srw.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken);
        gsl::span<carb::Float3> linVelocities = srw.getAttributeArray<carb::Float3>(primBuckets, i, mLinVelToken);
        gsl::span<carb::Float3> angVelocities = srw.getAttributeArray<carb::Float3>(primBuckets, i, mAngVelToken);
        gsl::span<carb::Double3> rbWorldPositions = srw.getAttributeArray<carb::Double3>(primBuckets, i, mRigidBodyWorldPositionToken);
        gsl::span<carb::Float4> rbWorldOrientations = srw.getAttributeArray<carb::Float4>(primBuckets, i, mRigidBodyWorldOrientationToken);
        gsl::span<carb::Float3> rbWorldScales = srw.getAttributeArray<carb::Float3>(primBuckets, i, mRigidBodyWorldScaleToken);

        gsl::span<const omni::fabric::Path> paths = srw.getPathArray(primBuckets, i);
        //printf("+++   Bucket %u: Path count %u\n", unsigned(i), unsigned(paths.size()));

        for (size_t j = 0; j < paths.size(); j++)
        {
            const omni::fabric::Path pathHandle = paths[j];

            // get the indexing data for this rigid body
            auto it = mRigidBodies.find(pathHandle);
            if (it != mRigidBodies.end())
            {
                const RigidBodyData& rb = it->second;
                if (rb.isArticulationLink)
                {
                    if (updateTransforms && mLinkTransforms.size() > rb.link.linkOffset)
                    {
                        const PxTransform& transform = mLinkTransforms[rb.link.linkOffset];

                        carb::Float3 translation;
                        carb::Float4 quaternion;
                        carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
                        gFabricManager->getInitialTransformation(pathHandle, translation, quaternion, scale);
                        worldMatrices[j] = computeMatrix(omni::physx::toVec3d(omni::physx::toDouble3(transform.p)),
                                                            pxr::GfMatrix3d(omni::physx::toQuatd(omni::physx::toDouble4(transform.q))),
                                                            omni::physx::toVec3d(scale));
                        if (j < rbWorldPositions.size())
                        {
                            rbWorldPositions[j] = omni::physx::toDouble3(transform.p);
                        }
                        if (j < rbWorldOrientations.size())
                        {
                            rbWorldOrientations[j] = omni::physx::toFloat4(transform.q);
                        }
                        if (j < rbWorldScales.size())
                        {
                            rbWorldScales[j] = scale;
                        }
                    }
                    if (updateVelocities && linVelocities.size() > 0 && angVelocities.size() > 0 &&
                        mLinkLinearVelocities.size() > rb.link.linkOffset &&
                        mLinkAngularVelocities.size() > rb.link.linkOffset)
                    {
                        const PxVec3& linear = mLinkLinearVelocities[rb.link.linkOffset];
                        const PxVec3& angular = mLinkAngularVelocities[rb.link.linkOffset];
                        linVelocities[j] = { linear.x, linear.y, linear.z };
                        angVelocities[j] = { angular.x, angular.y, angular.z };
                    }
                }
                else
                {
                    if (rb.rd.idx < mRdTransforms.size())
                    {
                        if (updateTransforms)
                        {
                            const PxTransform& rdTransform = mRdTransforms[rb.rd.idx];
                            carb::Float3 translation;
                            carb::Float4 quaternion;
                            carb::Float3 scale{ 1.0f, 1.0f, 1.0f };
                            gFabricManager->getInitialTransformation(pathHandle, translation, quaternion, scale);
                            worldMatrices[j] = computeMatrix(omni::physx::toVec3d(omni::physx::toDouble3(rdTransform.p)),
                                                                pxr::GfMatrix3d(omni::physx::toQuatd(omni::physx::toDouble4(rdTransform.q))),
                                                                omni::physx::toVec3d(scale));
                            if (j < rbWorldPositions.size())
                            {
                                rbWorldPositions[j] = omni::physx::toDouble3(rdTransform.p);
                            }
                            if (j < rbWorldOrientations.size())
                            {
                                rbWorldOrientations[j] = omni::physx::toFloat4(rdTransform.q);
                            }
                            if (j < rbWorldScales.size())
                            {
                                rbWorldScales[j] = scale;
                            }
                        }
                        if (updateVelocities && linVelocities.size() > 0 && angVelocities.size() > 0)
                        {
                            const PxVec3& rdLinVel = mRdLinearVelocities[rb.rd.idx];
                            linVelocities[j] = { rdLinVel.x, rdLinVel.y, rdLinVel.z };

                            const PxVec3& rdAngVel = mRdAngularVelocities[rb.rd.idx];
                            angVelocities[j] = { rdAngVel.x, rdAngVel.y, rdAngVel.z };
                        }
                    }
                }
            }
        }
    }
}


void DirectGpuHelper::applyRigidBodiesToFabric_GPU(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities)
{
    //printf("~!~!~! Updating GPU transforms\n");

    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();
    if (!cudaContextManager)
        return;

    PxScopedCudaLock _lock(*cudaContextManager);
    int deviceId = -1;
    {
        int st = 0;
        if (!getCudaShim()->ctxGetDevice(&deviceId, &st))
        {
            CARB_LOG_ERROR("cuCtxGetDevice failed (CUresult %d)", st);
            return;
        }
    }

    auto iFabric = carb::getCachedInterface<omni::fabric::IFabric>();
    auto iFabricConnectivity = carb::getCachedInterface<omni::fabric::IConnectivity>();

    // NOTE: comment to measure mapping performance in each frame
#if KIT_SDK_VERSION < 1090002
    if (iFabric->getTopologyVersion(srw.getFabricId()) != mFabricTopologyVersion ||
        iFabricConnectivity->getDataRevision(srw.getFabricId()) != mFabricConnectivityVersion ||
        !isTopologyConnectivityInitialized)
    {
        if (!prepareMappings(srw))
        {
            CARB_LOG_ERROR("Could not re-create Fabric mapping");
            return;
        }
    }
#else
    if (iFabric->getCounter(srw.getFabricId(), omni::fabric::Counter::eBucketTopology) != mFabricTopologyVersion ||
        iFabricConnectivity->getDataRevision(srw.getFabricId()) != mFabricConnectivityVersion ||
        !isTopologyConnectivityInitialized)
    {
        if (!prepareMappings(srw))
        {
            CARB_LOG_ERROR("Could not re-create Fabric mapping");
            return;
        }
    }
#endif

    // Get a write pointer to Fabric GPU data to dirty them before the GPU update.
    using omni::fabric::AttrNameAndType;
    const omni::fabric::set<AttrNameAndType> requiredAll = { AttrNameAndType(
        mTypeAppliedSchema, mRigidBodySchemaToken) };
    const omni::fabric::set<AttrNameAndType> requiredAny = {
        AttrNameAndType(mTypeMatrix4d, mWorldMatrixToken),
        AttrNameAndType(mTypeFloat3, mLinVelToken),
        AttrNameAndType(mTypeFloat3, mAngVelToken),
        AttrNameAndType(mTypeDouble3, mRigidBodyWorldPositionToken),
        AttrNameAndType(mTypeQuat, mRigidBodyWorldOrientationToken),
        AttrNameAndType(mTypeFloat3, mRigidBodyWorldScaleToken)
    };
    omni::fabric::PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);
    size_t bucketCount = primBuckets.bucketCount();
    {
        CARB_PROFILE_ZONE(1, "DirectGpuHelper dirty Fabric GPU buffers");
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<const pxr::GfMatrix4d> worldMatrices = srw.getAttributeArrayGpu<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken, deviceId);
            gsl::span<carb::Float3> linVelocities = srw.getAttributeArrayGpu<carb::Float3>(primBuckets, i, mLinVelToken, deviceId);
            gsl::span<carb::Float3> angVelocities = srw.getAttributeArrayGpu<carb::Float3>(primBuckets, i, mAngVelToken, deviceId);
            gsl::span<carb::Double3> rbWorldPositions = srw.getAttributeArrayGpu<carb::Double3>(primBuckets, i, mRigidBodyWorldPositionToken, deviceId);
            gsl::span<carb::Float4> rbWorldOrientations = srw.getAttributeArrayGpu<carb::Float4>(primBuckets, i, mRigidBodyWorldOrientationToken, deviceId);
            gsl::span<carb::Float3> rbWorldScales = srw.getAttributeArrayGpu<carb::Float3>(primBuckets, i, mRigidBodyWorldScaleToken, deviceId);
        }
    }

    // Call GPU kernels to copy transforms
    {
        CARB_PROFILE_ZONE(1, "DirectGpuHelper rigid body data copy to Fabric GPU");

        if (!mFabricMapperRb.empty())
        {
            RigidBodyGpuData rb;
            rb.updateTransforms = updateTransforms;
            rb.updateVelocities = updateVelocities;
            rb.numRigidBodies = mFabricMapperRb.getSize(mWorldMatrixToken);
            rb.rigidBodyTransforms = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mRdTransformsDev) };
            rb.linearVelocities = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mRdLinearVelocitiesDev) };
            rb.angularVelocities = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mRdAngularVelocitiesDev) };
            rb.initialScales = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mInitialScalesDev) };
            rb.worldMatMapping = reinterpret_cast<double**>(mFabricMapperRb.getPtr(mWorldMatrixToken));
            rb.worldPosMapping = reinterpret_cast<double**>(mFabricMapperRb.getPtr(mRigidBodyWorldPositionToken));
            rb.worldOriMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mRigidBodyWorldOrientationToken));
            rb.worldSclMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mRigidBodyWorldScaleToken));
            rb.linVelMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mLinVelToken));
            rb.angVelMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mAngVelToken));
            copyRigidBodyDataToFabricGpu(rb, omni::physx::FabricCudaStreamHandle{ reinterpret_cast<uintptr_t>(mCopyStream) });
        }

        if (!mFabricMapperArticulation.empty())
        {
            const size_t scaleOffset = mFabricMapperRb.getSize(mWorldMatrixToken) * sizeof(carb::Float3);
            RigidBodyGpuData rb;
            rb.updateTransforms = updateTransforms;
            rb.updateVelocities = updateVelocities;
            rb.numRigidBodies = mFabricMapperArticulation.getSize(mWorldMatrixToken);
            rb.rigidBodyTransforms = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mLinkTransformsDev) };
            rb.linearVelocities = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mLinkLinearVelocitiesDev) };
            rb.angularVelocities = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mLinkAngularVelocitiesDev) };
            rb.initialScales = omni::physx::FabricCudaDevicePtrHandle{ static_cast<uint64_t>(mInitialScalesDev + scaleOffset) };
            rb.worldMatMapping = reinterpret_cast<double**>(mFabricMapperArticulation.getPtr(mWorldMatrixToken));
            rb.worldPosMapping = reinterpret_cast<double**>(mFabricMapperArticulation.getPtr(mRigidBodyWorldPositionToken));
            rb.worldOriMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mRigidBodyWorldOrientationToken));
            rb.worldSclMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mRigidBodyWorldScaleToken));
            rb.linVelMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mLinVelToken));
            rb.angVelMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mAngVelToken));
            copyRigidBodyDataToFabricGpu(rb, omni::physx::FabricCudaStreamHandle{ reinterpret_cast<uintptr_t>(mCopyStream) });
        }

        if (!mFabricMapperRb.empty() || !mFabricMapperArticulation.empty())
        {
            CHECK_CUDA_SHIM(getCudaShim()->streamSynchronize(reinterpret_cast<uintptr_t>(mCopyStream), nullptr));
        }
    }
}


void DirectGpuHelper::updateXForms_CPU(omni::fabric::StageReaderWriter& srw)
{
    auto iHierarchyMaker = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
    auto iHierarchy = iHierarchyMaker ? iHierarchyMaker->getFabricHierarchy(srw.getFabricId(), mUsdStageId) : nullptr;

    if (iHierarchy)
    {
        iHierarchy->updateWorldXforms();
    }
}


bool DirectGpuHelper::updateXForms_GPU(omni::fabric::StageReaderWriter& srw)
{
    if (!mCubricAdapter)
    {
        IPhysxFoundation* iFoundation = carb::getCachedInterface<IPhysxFoundation>();
        if (!iFoundation)
            return false;
        mCubricAdapter = iFoundation->cubricCreateAdapter();
        if (!mCubricAdapter)
            return false;
    }

    carb::settings::ISettings* iSettings = carb::getCachedInterface<carb::settings::ISettings>();
    const bool cubricEnableCAS = iSettings->getAsBool(kSettingCubricEnableCAS);

    IPhysxFoundation* iFoundation = carb::getCachedInterface<IPhysxFoundation>();
    if (!iFoundation)
        return false;

    {
        CARB_PROFILE_ZONE(1, "DirectGpuHelper cubric binding");
        if (!iFoundation->cubricBindToStage(mCubricAdapter, srw.getFabricId(), cubricEnableCAS))
            return false;
    }
    {
        CARB_PROFILE_ZONE(1, "DirectGpuHelper cubric compute");
        return iFoundation->cubricCompute(mCubricAdapter);
    }
}


struct ScopedUSDRT
{
    omni::core::ObjectPtr<usdrt::hierarchy::IFabricHierarchy> iHierarchy;
    bool trackingPaused = false;

    ScopedUSDRT(omni::fabric::StageReaderWriter& srw, fabric::UsdStageId usdStageId)
    {
        auto iHierarchyMaker = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
        iHierarchy = iHierarchyMaker ? iHierarchyMaker->getFabricHierarchy(srw.getFabricId(), usdStageId) : nullptr;

        pauseTracking();
    }

    ~ScopedUSDRT()
    {
        resumeTracking();
    }

    void pauseTracking()
    {
        if (iHierarchy && not trackingPaused)
        {
            iHierarchy->trackWorldXformChanges(false);
            iHierarchy->trackLocalXformChanges(false);
            trackingPaused = true;
        }
    }

    void resumeTracking()
    {
        if (iHierarchy && trackingPaused)
        {
            iHierarchy->trackWorldXformChanges(true);
            iHierarchy->trackLocalXformChanges(true);
            trackingPaused = false;
        }
    }
};


void DirectGpuHelper::updateRigidBodies(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities, bool fullFabricGpuInterop)
{
    if (!updateTransforms && !updateVelocities)
    {
        return;
    }

    // full GPU data flow Physics/GPU >>> RB-buffers/GPU >>> Fabric/GPU > cuBric/GPU

    if (fullFabricGpuInterop)
    {
        CARB_PROFILE_ZONE(0, "FabricManager::DirectGpuHelper::updateRigidBodies");

        ScopedUSDRT scopedUSDRT(srw, mUsdStageId);
        
        applyRigidBodiesToFabric_GPU(srw, updateTransforms, updateVelocities);

        if(updateXForms_GPU(srw))
        {
            return;
        }

        CARB_LOG_WARN("Failed to update transform hierarchy on the GPU, falling back to CPU");
    }

    // Testing data flow Physics/GPU >>> RB-buffers/GPU >>> RB-buffers/CPU >>> Fabric/CPU > cuBric/GPU

    if(false)
    {
        applyRigidBodiesToFabric_CPU(srw, updateTransforms, updateVelocities);

        if(updateXForms_GPU(srw))
        {
            return;
        }

        CARB_LOG_WARN("Failed to update transform hierarchy on the GPU, falling back to CPU");
    }

    // full CPY data flow Physics/GPU >>> RB-buffers/GPU >>> RB-buffers/CPU >>> Fabric/CPU > FabricHierarchy/GPU
    {
        // we need to update the transformations
        updateXForms_CPU(srw);
        applyRigidBodiesToFabric_CPU(srw, updateTransforms, updateVelocities);
        updateXForms_CPU(srw);
    }
}


} // physx
} // omni
