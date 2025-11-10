// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DirectGpuHelper.h"
#include "FabricManager.h"
#include "FabricKernels.h"

#include <carb/logging/Log.h>
#include <carb/InterfaceUtils.h>
#include <carb/container/RHUnorderedMap.h>

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

#include "common/utilities/CudaHelpers.h"
#include "common/foundation/TypeCast.h"

#include <PxPhysicsAPI.h>

#include <cuda.h>

#include <algorithm>
#include <unordered_set>
#include <vector>

using namespace pxr;
using namespace physx;
using namespace carb;

extern omni::physx::FabricManager* gFabricManager;

namespace omni
{
namespace physx
{

// private stuff
namespace
{

#define CHECK_CU_DGH(code) cudaHelpers::checkCu(code, __FILE__, __LINE__)

} // end of private stuff

// TODO(mmagdics): this is duplicated from FabricManager.cpp in omni.physx.fabric
// common/utility does not depend on Fabric yet and it seemed better to leave it this way

GfMatrix4d computeMatrix(const pxr::GfVec3d& translate, const pxr::GfMatrix3d& rotate, const pxr::GfVec3d& scale)
{
    // Order is scale*rotate*translate
    return GfMatrix4d(rotate[0][0] * scale[0], rotate[0][1] * scale[0], rotate[0][2] * scale[0], 0,
                      rotate[1][0] * scale[1], rotate[1][1] * scale[1], rotate[1][2] * scale[1], 0,
                      rotate[2][0] * scale[2], rotate[2][1] * scale[2], rotate[2][2] * scale[2], 0, translate[0],
                      translate[1], translate[2], 1);
}

GfMatrix4d computeLocalMatrix(omni::fabric::USDHierarchy& usdHierarchy,
                              omni::fabric::StageReaderWriter& stage,
                              const omni::fabric::Path& path,
                              const GfMatrix4d& worldMatrix,
                              const omni::fabric::TokenC& worldMatrixToken)
{
    omni::fabric::PathC parentPath = usdHierarchy.getParent(path);
    while (parentPath != omni::fabric::kUninitializedPath)
    {
        const pxr::GfMatrix4d* parentWorldMatrix = stage.getAttributeRd<pxr::GfMatrix4d>(parentPath, worldMatrixToken);
        if (parentWorldMatrix)
        {
            return worldMatrix * parentWorldMatrix->GetInverse();
        }
        else
        {
            parentPath = usdHierarchy.getParent(parentPath);
        }
    }
    return worldMatrix;
}

DirectGpuHelper::DirectGpuHelper()
{
    omni::fabric::IToken* iToken = carb::getCachedInterface<omni::fabric::IToken>();

    mRigidBodySchemaToken = iToken->getHandle("PhysicsRigidBodyAPI");

    mWorldMatrixToken = iToken->getHandle(gWorldMatrixTokenString);
    mLocalMatrixToken = iToken->getHandle(gLocalMatrixTokenString);

    mLinVelToken = iToken->getHandle(UsdPhysicsTokens->physicsVelocity.GetText());
    mAngVelToken = iToken->getHandle(UsdPhysicsTokens->physicsAngularVelocity.GetText());

    mRigidBodyWorldPositionToken = iToken->getHandle(gRigidBodyWorldPositionTokenString);
    mRigidBodyWorldOrientationToken = iToken->getHandle(gRigidBodyWorldOrientationTokenString);
    mRigidBodyWorldScaleToken = iToken->getHandle(gRigidBodyWorldScaleTokenString);

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
}

void DirectGpuHelper::detach()
{
    releaseBuffers();
    *this = DirectGpuHelper();
}

void DirectGpuHelper::registerRigidBody(const omni::fabric::PathC& primPath, const carb::Float3& scale)
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
        if (!CHECK_CU_DGH(cuMemAlloc(&mArtiIndicesDev, artiIndices.size() * sizeof(PxU32))))
        {
            return false;
        }
        if (!CHECK_CU_DGH(cuMemcpyHtoD(mArtiIndicesDev, artiIndices.data(), artiIndices.size() * sizeof(PxU32))))
        {
            return false;
        }

        // link transforms
        if (!fullFabricGpuInterop)
        {
            mLinkTransforms.resize(mLinkBufSize);
        }
        if (!CHECK_CU_DGH(cuMemAlloc(&mLinkTransformsDev, mLinkBufSize * sizeof(PxTransform))))
        {
            return false;
        }
        CHECK_CU_DGH(cuMemsetD8(mLinkTransformsDev, 0, mLinkBufSize * sizeof(PxTransform)));

        // link linear velocities
        if (!fullFabricGpuInterop)
        {
            mLinkLinearVelocities.resize(mLinkBufSize);
        }
        if (!CHECK_CU_DGH(cuMemAlloc(&mLinkLinearVelocitiesDev, mLinkBufSize * sizeof(PxVec3))))
        {
            return false;
        }
        CHECK_CU_DGH(cuMemsetD8(mLinkLinearVelocitiesDev, 0, mLinkBufSize * sizeof(PxVec3)));

        // link angular velocities
        if (!fullFabricGpuInterop)
        {
            mLinkAngularVelocities.resize(mLinkBufSize);
        }
        if (!CHECK_CU_DGH(cuMemAlloc(&mLinkAngularVelocitiesDev, mLinkBufSize * sizeof(PxVec3))))
        {
            return false;
        }
        CHECK_CU_DGH(cuMemsetD8(mLinkAngularVelocitiesDev, 0, mLinkBufSize * sizeof(PxVec3)));


        // synchronization events
        CHECK_CU_DGH(cuEventCreate(&mLinkTransformsStartEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mLinkTransformsCopyEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mLinkLinearVelocitiesStartEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mLinkAngularVelocitiesStartEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mLinkLinearVelocitiesCopyEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mLinkAngularVelocitiesCopyEvent, CU_EVENT_DISABLE_TIMING));
    }

    // allocate rigid dynamic buffers
    if (mNumRds > 0)
    {
        // actor indices
        if (!CHECK_CU_DGH(cuMemAlloc(&mRdIndicesDev, mNumRds * sizeof(PxRigidDynamicGPUIndex))))
        {
            return false;
        }
        if (!CHECK_CU_DGH(cuMemcpyHtoD(mRdIndicesDev, rdIndices.data(), mNumRds * sizeof(PxRigidDynamicGPUIndex))))
        {
            return false;
        }

        // actor data
        if (!fullFabricGpuInterop)
        {
            mRdTransforms.resize(mNumRds);
        }
        if (!CHECK_CU_DGH(cuMemAlloc(&mRdTransformsDev, mNumRds * sizeof(PxTransform))))
        {
            return false;
        }
        CHECK_CU_DGH(cuMemsetD8(mRdTransformsDev, 0, mNumRds * sizeof(PxTransform)));

        if (!fullFabricGpuInterop)
        {
            mRdLinearVelocities.resize(mNumRds);
        }
        if (!CHECK_CU_DGH(cuMemAlloc(&mRdLinearVelocitiesDev, mNumRds * sizeof(PxVec3))))
        {
            return false;
        }
        CHECK_CU_DGH(cuMemsetD8(mRdLinearVelocitiesDev, 0, mNumRds * sizeof(PxVec3)));

        if (!fullFabricGpuInterop)
        {
            mRdAngularVelocities.resize(mNumRds);
        }
        if (!CHECK_CU_DGH(cuMemAlloc(&mRdAngularVelocitiesDev, mNumRds * sizeof(PxVec3))))
        {
            return false;
        }
        CHECK_CU_DGH(cuMemsetD8(mRdAngularVelocitiesDev, 0, mNumRds * sizeof(PxVec3)));

        // synchronization event
        CHECK_CU_DGH(cuEventCreate(&mRdTransformsStartEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mRdTransformsCopyEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mRdLinearVelocityStartEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mRdLinearVelocityCopyEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mRdAngularVelocityStartEvent, CU_EVENT_DISABLE_TIMING));
        CHECK_CU_DGH(cuEventCreate(&mRdAngularVelocityCopyEvent, CU_EVENT_DISABLE_TIMING));
    }

    if (fullFabricGpuInterop)
    {
        CARB_PROFILE_ZONE(1, "Build Fabric GPU mapping data");

        if (mCopyStream)
        {
            CHECK_CU_DGH(cuStreamDestroy(mCopyStream));
        }
        if (!CHECK_CU_DGH(cuStreamCreate(&mCopyStream, 0)))
        {
            return false;
        }

        // prepare initial scales buffer
        // rigid bodies first, then articulations in the same buffer
        if (mNumRds + mLinkBufSize != 0)
        {
            if (!CHECK_CU_DGH(cuMemAlloc(&mInitialScalesDev, (mNumRds + mLinkBufSize) * sizeof(carb::Float3))))
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
            if (!CHECK_CU_DGH(cuMemcpyHtoD(mInitialScalesDev, initialScales.data(), initialScales.size() * sizeof(carb::Float3))))
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
    omni::fabric::FabricId fabricId = srw.getFabricId();
    if (fabricId == omni::fabric::kInvalidFabricId)
    {
        CARB_LOG_ERROR("Invalid Fabric Id, cannot initialize GPU transform interop");
        return false;
    }

    int deviceId;
    cudaGetDevice(&deviceId);

    CARB_PROFILE_ZONE(1, "FabricMapper build");
    if (mNumRds > 0)
    {
        mFabricMapperRb.clear();
        mFabricMapperRb.bindFabric(srw.getFabricId());
        mFabricMapperRb.bindDevice(deviceId);
        auto getRigidBodyIndex = [&](const RigidBodyData& rb) -> size_t {
            if (rb.isArticulationLink)
            {
                return mNumRds + 1;
            }
            return rb.rd.idx;
        };
        mFabricMapperRb.build<RigidBodyData, pxr::GfMatrix4d>(
            mRigidBodies, mNumRds, getRigidBodyIndex, mLocalMatrixToken);
        mFabricMapperRb.buildParent<RigidBodyData, pxr::GfMatrix4d>(
            mRigidBodies, mNumRds, getRigidBodyIndex, mWorldMatrixToken);
        mFabricMapperRb.build<RigidBodyData, carb::Float3>(mRigidBodies, mNumRds, getRigidBodyIndex, mLinVelToken);
        mFabricMapperRb.build<RigidBodyData, carb::Float3>(mRigidBodies, mNumRds, getRigidBodyIndex, mAngVelToken);
        mFabricMapperRb.build<RigidBodyData, carb::Double3>(
            mRigidBodies, mNumRds, getRigidBodyIndex, mRigidBodyWorldPositionToken);
        mFabricMapperRb.build<RigidBodyData, carb::Float4>(
            mRigidBodies, mNumRds, getRigidBodyIndex, mRigidBodyWorldOrientationToken);
        mFabricMapperRb.build<RigidBodyData, carb::Float3>(
            mRigidBodies, mNumRds, getRigidBodyIndex, mRigidBodyWorldScaleToken);
    }

    if (mLinkBufSize > 0)
    {
        mFabricMapperArticulation.clear();
        mFabricMapperArticulation.bindFabric(srw.getFabricId());
        mFabricMapperArticulation.bindDevice(deviceId);
        auto getArticulationIndex = [&](const RigidBodyData& rb) -> size_t {
            if (rb.isArticulationLink)
            {
                return rb.link.linkOffset;
            }
            return mLinkBufSize + 1;
        };
        mFabricMapperArticulation.build<RigidBodyData, pxr::GfMatrix4d>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mLocalMatrixToken);
        mFabricMapperArticulation.buildParent<RigidBodyData, pxr::GfMatrix4d>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mWorldMatrixToken);
        mFabricMapperArticulation.build<RigidBodyData, carb::Float3>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mLinVelToken);
        mFabricMapperArticulation.build<RigidBodyData, carb::Float3>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mAngVelToken);
        mFabricMapperArticulation.build<RigidBodyData, carb::Double3>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mRigidBodyWorldPositionToken);
        mFabricMapperArticulation.build<RigidBodyData, carb::Float4>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mRigidBodyWorldOrientationToken);
        mFabricMapperArticulation.build<RigidBodyData, carb::Float3>(
            mRigidBodies, mLinkBufSize, getArticulationIndex, mRigidBodyWorldScaleToken);
    }

    omni::fabric::IFabric* iFabric = carb::getCachedInterface<omni::fabric::IFabric>();
    mFabricTopologyVersion = iFabric->getTopologyVersion(fabricId);

    return true;
}

void DirectGpuHelper::releaseBuffers()
{
    PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();

    if (cudaContextManager)
    {

        PxScopedCudaLock _lock(*cudaContextManager);

        CHECK_CU_DGH(cuMemFree(mArtiIndicesDev));
        mArtiIndicesDev = 0;

        CHECK_CU_DGH(cuMemFree(mLinkTransformsDev));
        mLinkTransformsDev = 0;

        CHECK_CU_DGH(cuMemFree(mLinkLinearVelocitiesDev));
        mLinkLinearVelocitiesDev = 0;

        CHECK_CU_DGH(cuMemFree(mLinkAngularVelocitiesDev));
        mLinkAngularVelocitiesDev = 0;

        CHECK_CU_DGH(cuMemFree(mRdIndicesDev));
        mRdIndicesDev = 0;

        CHECK_CU_DGH(cuMemFree(mRdTransformsDev));
        mRdTransformsDev = 0;

        CHECK_CU_DGH(cuMemFree(mRdLinearVelocitiesDev));
        mRdLinearVelocitiesDev = 0;

        CHECK_CU_DGH(cuMemFree(mRdAngularVelocitiesDev));
        mRdAngularVelocitiesDev = 0;

        CHECK_CU_DGH(cuMemFree(mInitialScalesDev));
        mInitialScalesDev = 0;
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
        CHECK_CU_DGH(cuStreamDestroy(mCopyStream));
        mCopyStream = nullptr;
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
            CHECK_CU_DGH(cuEventRecord(mLinkTransformsStartEvent, nullptr));
            mScene->getDirectGPUAPI().getArticulationData((void*)mLinkTransformsDev,
                                                          (PxArticulationGPUIndex*)mArtiIndicesDev,
                                                          PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE, mNumArtis,
                                                          mLinkTransformsStartEvent, mLinkTransformsCopyEvent);
            fetchingLinkTransforms = true;
        }
        if (updateVelocities && mLinkLinearVelocitiesDev && mLinkAngularVelocitiesDev)
        {
            CHECK_CU_DGH(cuEventRecord(mLinkLinearVelocitiesStartEvent, nullptr));
            mScene->getDirectGPUAPI().getArticulationData(
                (void*)mLinkLinearVelocitiesDev, (PxArticulationGPUIndex*)mArtiIndicesDev,
                PxArticulationGPUAPIReadType::eLINK_LINEAR_VELOCITY, mNumArtis, mLinkLinearVelocitiesStartEvent,
                mLinkLinearVelocitiesCopyEvent);

            CHECK_CU_DGH(cuEventRecord(mLinkAngularVelocitiesStartEvent, nullptr));
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
            CHECK_CU_DGH(cuEventRecord(mRdTransformsStartEvent, nullptr));
            mScene->getDirectGPUAPI().getRigidDynamicData(
                (void*)mRdTransformsDev, (PxRigidDynamicGPUIndex*)mRdIndicesDev,
                PxRigidDynamicGPUAPIReadType::eGLOBAL_POSE, mNumRds, mRdTransformsStartEvent, mRdTransformsCopyEvent);

            fetchingRdTransforms = true;
        }

        if (mRdLinearVelocitiesDev && mRdAngularVelocitiesDev)
        {
            CHECK_CU_DGH(cuEventRecord(mRdLinearVelocityStartEvent, nullptr));
            mScene->getDirectGPUAPI().getRigidDynamicData((void*)mRdLinearVelocitiesDev,
                                                          (PxRigidDynamicGPUIndex*)mRdIndicesDev,
                                                          PxRigidDynamicGPUAPIReadType::eLINEAR_VELOCITY, mNumRds,
                                                          mRdLinearVelocityStartEvent, mRdLinearVelocityCopyEvent);

            CHECK_CU_DGH(cuEventRecord(mRdAngularVelocityStartEvent, nullptr));
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
        CHECK_CU_DGH(cuStreamWaitEvent(nullptr, mLinkTransformsCopyEvent, 0));
        if (!fullFabricGpuInterop)
        {
            CHECK_CU_DGH(cuMemcpyDtoH(mLinkTransforms.data(), mLinkTransformsDev, mLinkBufSize * sizeof(PxTransform)));
        }
    }
    if (fetchingLinkVelocities)
    {
        CHECK_CU_DGH(cuStreamWaitEvent(nullptr, mLinkLinearVelocitiesCopyEvent, 0));
        CHECK_CU_DGH(cuStreamWaitEvent(nullptr, mLinkAngularVelocitiesCopyEvent, 0));
        if (!fullFabricGpuInterop)
        {
            CHECK_CU_DGH(cuMemcpyDtoH(mLinkLinearVelocities.data(), mLinkLinearVelocitiesDev, mLinkBufSize * sizeof(PxVec3)));
            CHECK_CU_DGH(
                cuMemcpyDtoH(mLinkAngularVelocities.data(), mLinkAngularVelocitiesDev, mLinkBufSize * sizeof(PxVec3)));
        }
    }
    if (fetchingRdTransforms)
    {
        CHECK_CU_DGH(cuStreamWaitEvent(nullptr, mRdTransformsCopyEvent, 0));
        if (!fullFabricGpuInterop)
        {
            CHECK_CU_DGH(cuMemcpyDtoH(mRdTransforms.data(), mRdTransformsDev, mNumRds * sizeof(PxTransform)));
        }
    }
    if (fetchingRdVelocities)
    {
        CHECK_CU_DGH(cuStreamWaitEvent(nullptr, mRdLinearVelocityCopyEvent, 0));
        CHECK_CU_DGH(cuStreamWaitEvent(nullptr, mRdAngularVelocityCopyEvent, 0));
        if (!fullFabricGpuInterop)
        {
            CHECK_CU_DGH(cuMemcpyDtoH(mRdLinearVelocities.data(), mRdLinearVelocitiesDev, mNumRds * sizeof(PxVec3)));
            CHECK_CU_DGH(cuMemcpyDtoH(mRdAngularVelocities.data(), mRdAngularVelocitiesDev, mNumRds * sizeof(PxVec3)));
        }
    }
}

void DirectGpuHelper::updateRigidBodies(omni::fabric::StageReaderWriter& srw, bool updateTransforms, bool updateVelocities, bool fullFabricGpuInterop)
{
    if (!updateTransforms && !updateVelocities)
    {
        return;
    }

    if (fullFabricGpuInterop)
    {
        PxCudaContextManager* cudaContextManager = FabricManager::getCudaContextManager();
        if (!cudaContextManager)
            return;

        PxScopedCudaLock _lock(*cudaContextManager);
        int deviceId;
        cudaGetDevice(&deviceId);

        omni::fabric::IFabric* iFabric = carb::getCachedInterface<omni::fabric::IFabric>();
        if (iFabric->getTopologyVersion(srw.getFabricId()) != mFabricTopologyVersion)
        {
            if (!prepareMappings(srw))
            {
                CARB_LOG_ERROR("Could not re-create Fabric mapping");
                return;
            }
        }

        // Get a write pointer to Fabric GPU data to dirty them before the GPU update.
        using omni::fabric::AttrNameAndType_v2;
        const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(
            mTypeAppliedSchema, mRigidBodySchemaToken) };
        const omni::fabric::set<AttrNameAndType_v2> requiredAny = {
            AttrNameAndType_v2(mTypeMatrix4d, mWorldMatrixToken),
            AttrNameAndType_v2(mTypeMatrix4d, mLocalMatrixToken),
            AttrNameAndType_v2(mTypeFloat3, mLinVelToken),
            AttrNameAndType_v2(mTypeFloat3, mAngVelToken),
            AttrNameAndType_v2(mTypeDouble3, mRigidBodyWorldPositionToken),
            AttrNameAndType_v2(mTypeQuat, mRigidBodyWorldOrientationToken),
            AttrNameAndType_v2(mTypeFloat3, mRigidBodyWorldScaleToken)
        };
        omni::fabric::PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);
        size_t bucketCount = primBuckets.bucketCount();
        {
            CARB_PROFILE_ZONE(1, "DirectGpuHelper dirty Fabric GPU buffers");
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const pxr::GfMatrix4d> worldMatrices =
                   srw.getAttributeArrayRdGpu<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken, deviceId);
                gsl::span<pxr::GfMatrix4d> localMatrices =
                   srw.getAttributeArrayGpu<pxr::GfMatrix4d>(primBuckets, i, mLocalMatrixToken, deviceId);
                gsl::span<carb::Float3> linVelocities = srw.getAttributeArrayGpu<carb::Float3>(primBuckets, i, mLinVelToken, deviceId);
                gsl::span<carb::Float3> angVelocities = srw.getAttributeArrayGpu<carb::Float3>(primBuckets, i, mAngVelToken, deviceId);
                gsl::span<carb::Double3> rbWorldPositions =
                    srw.getAttributeArrayGpu<carb::Double3>(primBuckets, i, mRigidBodyWorldPositionToken, deviceId);
                gsl::span<carb::Float4> rbWorldOrientations =
                    srw.getAttributeArrayGpu<carb::Float4>(primBuckets, i, mRigidBodyWorldOrientationToken, deviceId);
                gsl::span<carb::Float3> rbWorldScales =
                    srw.getAttributeArrayGpu<carb::Float3>(primBuckets, i, mRigidBodyWorldScaleToken, deviceId);
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
                rb.numRigidBodies = mFabricMapperRb.getSize(mLocalMatrixToken);
                rb.rigidBodyTransforms = mRdTransformsDev;
                rb.linearVelocities = mRdLinearVelocitiesDev;
                rb.angularVelocities = mRdAngularVelocitiesDev;
                rb.initialScales = mInitialScalesDev;
                rb.localMatMapping = reinterpret_cast<double**>(mFabricMapperRb.getPtr(mLocalMatrixToken));
                rb.parentWorldMatMapping = reinterpret_cast<double**>(mFabricMapperRb.getParentPtr(mWorldMatrixToken));
                rb.worldPosMapping = reinterpret_cast<double**>(mFabricMapperRb.getPtr(mRigidBodyWorldPositionToken));
                rb.worldOriMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mRigidBodyWorldOrientationToken));
                rb.worldSclMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mRigidBodyWorldScaleToken));
                rb.linVelMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mLinVelToken));
                rb.angVelMapping = reinterpret_cast<float**>(mFabricMapperRb.getPtr(mAngVelToken));
                copyRigidBodyDataToFabricGpu(rb, mCopyStream);
            }

            if (!mFabricMapperArticulation.empty())
            {
                const size_t scaleOffset = mFabricMapperRb.getSize(mLocalMatrixToken) * sizeof(carb::Float3);
                RigidBodyGpuData rb;
                rb.updateTransforms = updateTransforms;
                rb.updateVelocities = updateVelocities;
                rb.numRigidBodies = mFabricMapperArticulation.getSize(mLocalMatrixToken);
                rb.rigidBodyTransforms = mLinkTransformsDev;
                rb.linearVelocities = mLinkLinearVelocitiesDev;
                rb.angularVelocities = mLinkAngularVelocitiesDev;
                rb.initialScales = mInitialScalesDev + scaleOffset;
                rb.localMatMapping = reinterpret_cast<double**>(mFabricMapperArticulation.getPtr(mLocalMatrixToken));
                rb.parentWorldMatMapping = reinterpret_cast<double**>(mFabricMapperArticulation.getParentPtr(mWorldMatrixToken));
                rb.worldPosMapping = reinterpret_cast<double**>(mFabricMapperArticulation.getPtr(mRigidBodyWorldPositionToken));
                rb.worldOriMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mRigidBodyWorldOrientationToken));
                rb.worldSclMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mRigidBodyWorldScaleToken));
                rb.linVelMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mLinVelToken));
                rb.angVelMapping = reinterpret_cast<float**>(mFabricMapperArticulation.getPtr(mAngVelToken));
                copyRigidBodyDataToFabricGpu(rb, mCopyStream);
            }

            if (!mFabricMapperRb.empty() || !mFabricMapperArticulation.empty())
            {
                CHECK_CU_DGH(cuStreamSynchronize(mCopyStream));
            }

        }

        // Update fabric hierarchy on GPU only
        auto iFabricHierarchy = omni::core::createType<usdrt::hierarchy::IFabricHierarchy>();
        if (iFabricHierarchy != nullptr)
        {
            auto fabricHierarchy = iFabricHierarchy->getFabricHierarchy(srw.getFabricId(), mUsdStageId);
            bool success = fabricHierarchy->updateWorldXformsGpu(true);
            if (!success)
            {
                CARB_LOG_WARN("Failed to update transform hierarchy on the GPU, falling back to CPU");
                fabricHierarchy->updateWorldXforms();
            }
        }

        return;
    }

    //printf("~!~!~! Updating GPU transforms\n");

    using omni::fabric::AttrNameAndType_v2;

   const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(mTypeAppliedSchema, mRigidBodySchemaToken) };
   const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(mTypeMatrix4d, mWorldMatrixToken),
                                                             AttrNameAndType_v2(mTypeMatrix4d, mLocalMatrixToken),
                                                             AttrNameAndType_v2(mTypeFloat3, mLinVelToken),
                                                             AttrNameAndType_v2(mTypeFloat3, mAngVelToken),
                                                             AttrNameAndType_v2(mTypeDouble3, mRigidBodyWorldPositionToken),
                                                             AttrNameAndType_v2(mTypeQuat, mRigidBodyWorldOrientationToken),
                                                             AttrNameAndType_v2(mTypeFloat3, mRigidBodyWorldScaleToken) };

    omni::fabric::PrimBucketList primBuckets = srw.findPrims(requiredAll, requiredAny);

    omni::fabric::USDHierarchy usdHierarchy(srw.getFabricId());

    size_t bucketCount = primBuckets.bucketCount();
    //printf("+++ Bucket count %u\n", unsigned(bucketCount));
    for (size_t i = 0; i != bucketCount; i++)
    {
        gsl::span<pxr::GfMatrix4d> worldMatrices = srw.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken);
        gsl::span<pxr::GfMatrix4d> localMatrices = srw.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mLocalMatrixToken);
        gsl::span<carb::Float3> linVelocities = srw.getAttributeArray<carb::Float3>(primBuckets, i, mLinVelToken);
        gsl::span<carb::Float3> angVelocities = srw.getAttributeArray<carb::Float3>(primBuckets, i, mAngVelToken);
        gsl::span<carb::Double3> rbWorldPositions =
            srw.getAttributeArray<carb::Double3>(primBuckets, i, mRigidBodyWorldPositionToken);
        gsl::span<carb::Float4> rbWorldOrientations =
            srw.getAttributeArray<carb::Float4>(primBuckets, i, mRigidBodyWorldOrientationToken);
        gsl::span<carb::Float3> rbWorldScales =
            srw.getAttributeArray<carb::Float3>(primBuckets, i, mRigidBodyWorldScaleToken);

        gsl::span<const omni::fabric::Path> paths = srw.getPathArray(primBuckets, i);
        //printf("+++   Bucket %u: Path count %u\n", unsigned(i), unsigned(paths.size()));

        for (size_t j = 0; j < paths.size(); j++)
        {
            omni::fabric::PathC pathHandle = paths[j];

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
                        worldMatrices[j] =
                            computeMatrix(omni::physx::toVec3d(omni::physx::toDouble3(transform.p)),
                                          pxr::GfMatrix3d(omni::physx::toQuatd(omni::physx::toDouble4(transform.q))),
                                          omni::physx::toVec3d(scale));
                        localMatrices[j] =
                            computeLocalMatrix(usdHierarchy, srw, pathHandle, worldMatrices[j], mWorldMatrixToken);
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
                            worldMatrices[j] =
                                computeMatrix(omni::physx::toVec3d(omni::physx::toDouble3(rdTransform.p)),
                                pxr::GfMatrix3d(omni::physx::toQuatd(omni::physx::toDouble4(rdTransform.q))),
                                              omni::physx::toVec3d(scale));
                            localMatrices[j] =
                                computeLocalMatrix(usdHierarchy, srw, pathHandle, worldMatrices[j], mWorldMatrixToken);
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

}
}
