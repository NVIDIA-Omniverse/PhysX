// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include <carb/tasking/ITasking.h>

#include "DirectGpuHelper.h"
#include "VolumeDeformableBodyManager.h"
#include "SurfaceDeformableBodyManager.h"

#include <unordered_map>

#include <omni/fabric/IToken.h>
#include <omni/fabric/IFabric.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/IPhysxJoint.h>
#include <PxPhysicsAPI.h>

#include <usdrt/scenegraph/usd/usd/stage.h>


namespace carb
{
namespace fabric
{
struct IStageReaderWriter;
struct StageReaderWriterId;
} // namespace fabric
} // namespace carb


namespace omni
{
namespace fabric
{
class StageReaderWriter;
}
namespace physx
{
struct IPhysxSimulation;

struct FabricRigidBodyData
{
    carb::Float3 translation;
    carb::Float4 orientation;
    carb::Float3 scale;
};

struct ProtoInstanceData
{
    PXR_NS::SdfPath instancerPath;
    PXR_NS::SdfPath usdProtoPath;
    PXR_NS::GfMatrix4d protoTransfromInverse;
    std::vector<size_t> indices;
};

struct PointIntancerData
{
    PXR_NS::VtArray<PXR_NS::GfVec3f> positions;
    PXR_NS::VtArray<PXR_NS::GfQuath> orientations;
};

using TransformationCache = std::unordered_map<omni::fabric::Path, FabricRigidBodyData>;
using JointStateCache = std::unordered_map<omni::fabric::Path, omni::physx::JointStateData>;
using PathSet = std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
using WheelMap = std::unordered_map<omni::fabric::Path, usdparser::ObjectId>;
using PointInstanceProtoCache = std::unordered_map<omni::fabric::Path, ProtoInstanceData>;
using PointInstancerCache = std::unordered_map<PXR_NS::SdfPath, PointIntancerData, PXR_NS::SdfPath::Hash>;

struct FabricUsdNoticeListener : public PXR_NS::TfWeakBase
{
    FabricUsdNoticeListener() = default;

    void handle(const class PXR_NS::UsdNotice::ObjectsChanged& objectsChanged);
};

struct FabricJointStateToken
{
    omni::fabric::Token token;
    omni::fabric::Token madPositionToken;
    omni::fabric::Token madVelocityToken;
    PXR_NS::TfToken usdToken;
    ::physx::PxArticulationAxis::Enum physxAxis;
};

struct FabricSoftBodyData
{
    uint32_t idx = 0xffffffff; // softbody GPU index
    size_t numVerts = 0;
    size_t numCollVerts = 0;
    PXR_NS::UsdPrim prim;
    PXR_NS::GfMatrix4d initialPrimToParent;
};

class SimulationEventListener final : public carb::events::IEventListener
{
public:
    SimulationEventListener(std::function<void(carb::events::IEvent* e)> eventCallback) :
        mEventCallback(eventCallback)
    {
    }

    std::size_t addRef() override
    {
        return ++mRefCount;
    }

    std::size_t release() override
    {
        if (mRefCount)
        {
            --mRefCount;
        }

        return mRefCount;
    }

    void onEvent(carb::events::IEvent* e) override
    {
        if (mEventCallback)
        {
            mEventCallback(e);
        }
    }

private:
    std::size_t mRefCount{ 0 };
    std::function<void(carb::events::IEvent* e)> mEventCallback{ nullptr };
};

class FabricManager
{
public:
    FabricManager();
    ~FabricManager();
    void release();

    void attach(unsigned long stageId);
    void detach();

    void saveToUsd();

    void update(float currentTime, float elapsedSecs, bool forceUpdate = false);

    void resume();
    void pause();
    void stop();

    void removePrim(const PXR_NS::SdfPath& path);
    void resyncPrim(const PXR_NS::SdfPath& path);

    void toggleKinematics(bool kinematic, const PXR_NS::UsdPrim& usdPrim);

    void setPtrDirty()
    {
        mPtrDirty = true;
    }

    // Marks the volume deformable body manager as dirty so it rebuilds its per-scene sets
    // on the next update(). Called from the PhysX deletion callback when a PxDeformableVolume
    // is destroyed externally so that stale physXPtr values are not dereferenced.
    void setVolumeDeformableBodyDirty()
    {
        mVolumeDeformableBodyManager.setIsDirty();
    }

    // Marks the surface deformable body manager as dirty so it rebuilds its per-scene sets
    // on the next update(). Called from the PhysX deletion callback when a PxDeformableSurface
    // is destroyed externally so that stale physXPtr values are not dereferenced.
    void setSurfaceDeformableBodyDirty()
    {
        mSurfaceDeformableBodyManager.setIsDirty();
    }

    static ::physx::PxCudaContextManager* getCudaContextManager();

    bool getInitialTransformation(const omni::fabric::Path& path,
                                  carb::Float3& translation,
                                  carb::Float4& orientation,
                                  carb::Float3& scale);

    void enableKinematicBodyTransformationUpdate(bool enable)
    {
        mKinematicBodyTransformationUpdateEnabled = enable;
    }
    bool isKinematicBodyTransformationUpdateEnabled()
    {
        return mKinematicBodyTransformationUpdateEnabled;
    }

private:
    void setInitialTransformations();
    void initializeRigidBodyBatched(const std::vector<usdrt::SdfPath>& rbPaths,
                                    omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                    omni::fabric::StageReaderWriterId stageInProgress);
    void initializeWheel(PXR_NS::UsdGeomXformCache& xfCache,
                         const PXR_NS::UsdPrim& prim,
                         omni::fabric::IStageReaderWriter* iStageReaderWriter,
                         omni::fabric::StageReaderWriterId stageInProgress);
    void initializeJointState(const PXR_NS::UsdPrim& prim,
                              omni::fabric::IStageReaderWriter* iStageReaderWriter,
                              omni::fabric::StageReaderWriterId stageInProgress);
    void initializeDeformableBody(PXR_NS::UsdGeomXformCache& xfCache,
                                  const PXR_NS::UsdPrim& prim,
                                  omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                  omni::fabric::StageReaderWriterId stageInProgress);

    void parsePointInstancers(PXR_NS::UsdStageWeakPtr usdStage,
                              usdrt::UsdStageRefPtr usdrtStage,
                              PXR_NS::UsdGeomXformCache& xfCache,
                              omni::fabric::IStageReaderWriter* iStageReaderWriter,
                              omni::fabric::StageReaderWriterId stageInProgress);
    void initializePointInstancer(PXR_NS::UsdStageWeakPtr usdStage,
                                  const PXR_NS::UsdGeomPointInstancer instancer,
                                  PXR_NS::UsdGeomXformCache& xfCache,
                                  omni::fabric::StageReaderWriter& stage);
    bool updatePointInstancer(const omni::fabric::Path primPath,
                              omni::fabric::StageReaderWriter& stage,
                              PXR_NS::UsdGeomXformCache& xfCache);

private:
    void updateBucketsTransformations(const omni::fabric::PrimBucketList& primBuckets,
                                      omni::fabric::StageReaderWriter& stage,
                                      omni::physx::IPhysx* iPhysX,
                                      carb::tasking::ITasking* tasking,
                                      omni::fabric::USDHierarchy& usdHierarchy,
                                      PXR_NS::UsdGeomXformCache& xformCache, bool updateLocalMatrix);

private:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    omni::fabric::UsdStageId mStageId;
    omni::physx::IPhysx* mPhysX;

    SimulationEventListener* mSimulationEventListener{ nullptr };
    carb::events::ISubscriptionPtr mSimEvtSub;
    bool mUpdate;
    bool mResumed;
    TransformationCache mInitialTransformation;
    JointStateCache mInitialJointStates;
    WheelMap mWheelVehicleMap;
    PathSet mResyncPaths;
    PointInstanceProtoCache mPointInstanceProtos;
    PointInstancerCache mInitialPointInstancers;
    bool mUsdResetOnStop;
    bool mPtrDirty;
    bool mKinematicBodyTransformationUpdateEnabled;

    omni::fabric::Token mWorldMatrixToken;
    omni::fabric::Token mLocalMatrixToken;
    omni::fabric::Token mWorldForceToken;
    omni::fabric::Token mWorldTorqueToken;
    omni::fabric::Token mPointsToken;
    omni::fabric::Token mInitPointsToken;
    omni::fabric::Token mPhysXPtrToken;
    omni::fabric::Token mPhysXPtrInstancedToken;

    omni::fabric::Token mLinVelToken;
    omni::fabric::Token mAngVelToken;

    omni::fabric::Token mDynamicBodyToken;
    omni::fabric::Token mNestedBodyToken;

    omni::fabric::Token mRigidBodyWorldPositionToken;
    omni::fabric::Token mRigidBodyWorldOrientationToken;
    omni::fabric::Token mRigidBodyWorldScaleToken;

    omni::fabric::Type mFloat1Type;
    omni::fabric::Type mFloat3Type;
    omni::fabric::Type mDouble3Type;
    omni::fabric::Type mMatrix4dType;
    omni::fabric::Type mQuatType;
    omni::fabric::Type mFloat3ArrayType;
    omni::fabric::Type mPtrType;
    omni::fabric::Type mTagType;
    omni::fabric::Type mPtrInstancedType;

    FabricJointStateToken mTokenJointStates[5];
    FabricUsdNoticeListener* mNoticeListener;
    PXR_NS::TfNotice::Key mNoticeListenerKey;

    omni::physx::SubscriptionId mSubscriptionObjId;

    DirectGpuHelper mDirectGpuHelper;
    VolumeDeformableBodyManager mVolumeDeformableBodyManager;
    SurfaceDeformableBodyManager mSurfaceDeformableBodyManager;
};
} // namespace physx
} // namespace omni
