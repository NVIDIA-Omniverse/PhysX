// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>

#include "DirectGpuHelper.h"
#include "DeformableBodyManagerDeprecated.h"
#include "DeformableSurfaceManagerDeprecated.h"
#include "ParticleManagerDeprecated.h"
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
    pxr::SdfPath instancerPath;
    pxr::SdfPath usdProtoPath;
    pxr::GfMatrix4d protoTransfromInverse;
    std::vector<size_t> indices;
};

struct PointIntancerData
{
    pxr::VtArray<pxr::GfVec3f> positions;
    pxr::VtArray<pxr::GfQuath> orientations;
};

using TransformationCache = std::unordered_map<omni::fabric::Path, FabricRigidBodyData>;
using JointStateCache = std::unordered_map<omni::fabric::Path, omni::physx::JointStateData>;
using PathSet = std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash>;
using WheelMap = std::unordered_map<omni::fabric::Path, usdparser::ObjectId>;
using PointInstanceProtoCache = std::unordered_map<omni::fabric::Path, ProtoInstanceData>;
using PointInstancerCache = std::unordered_map<pxr::SdfPath, PointIntancerData, pxr::SdfPath::Hash>;

struct FabricUsdNoticeListener : public pxr::TfWeakBase
{
    FabricUsdNoticeListener() = default;

    void handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged);
};

struct FabricJointStateToken
{
    omni::fabric::Token token;
    omni::fabric::Token madPositionToken;
    omni::fabric::Token madVelocityToken;
    pxr::TfToken usdToken;
    ::physx::PxArticulationAxis::Enum physxAxis;
};

struct FabricSoftBodyData
{
    uint32_t idx = 0xffffffff; // softbody GPU index
    size_t numVerts = 0;
    size_t numCollVerts = 0;
    pxr::UsdPrim prim;
    pxr::GfMatrix4d initialPrimToParent;
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

    void removePrim(const pxr::SdfPath& path);
    void resyncPrim(const pxr::SdfPath& path);

    void toggleKinematics(bool kinematic, const pxr::UsdPrim& usdPrim);

    void setPtrDirty()
    {
        mPtrDirty = true;
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
    void initializeWheel(pxr::UsdGeomXformCache& xfCache,
                         const pxr::UsdPrim& prim,
                         omni::fabric::IStageReaderWriter* iStageReaderWriter,
                         omni::fabric::StageReaderWriterId stageInProgress);
    void initializeJointState(const pxr::UsdPrim& prim,
                              omni::fabric::IStageReaderWriter* iStageReaderWriter,
                              omni::fabric::StageReaderWriterId stageInProgress);
    void initializeSoftBodyDeprecated(pxr::UsdGeomXformCache& xfCache,
                                      const pxr::UsdPrim& prim,
                                      omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                      omni::fabric::StageReaderWriterId stageInProgress);
    void initializeDeformableSurfaceDeprecated(pxr::UsdGeomXformCache& xfCache,
                                               const pxr::UsdPrim& prim,
                                               omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                               omni::fabric::StageReaderWriterId stageInProgress);
    void initializeParticleClothDeprecated(pxr::UsdGeomXformCache& xfCache,
                                           const pxr::UsdPrim& prim,
                                           omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                           omni::fabric::StageReaderWriterId stageInProgress);
    void initializeDeformableBody(pxr::UsdGeomXformCache& xfCache,
                                  const pxr::UsdPrim& prim,
                                  omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                  omni::fabric::StageReaderWriterId stageInProgress);
    void initializeResiduals(const pxr::UsdPrim& prim,
                             omni::fabric::IStageReaderWriter* iStageReaderWriter,
                             omni::fabric::StageReaderWriterId stageInProgress);

    void parsePointInstancers(pxr::UsdStageWeakPtr usdStage,
                              usdrt::UsdStageRefPtr usdrtStage,
                              pxr::UsdGeomXformCache& xfCache,
                              omni::fabric::IStageReaderWriter* iStageReaderWriter,
                              omni::fabric::StageReaderWriterId stageInProgress);
    void initializePointInstancer(pxr::UsdStageWeakPtr usdStage,
                                  const pxr::UsdGeomPointInstancer instancer,
                                  pxr::UsdGeomXformCache& xfCache,
                                  omni::fabric::StageReaderWriter& stage);
    bool updatePointInstancer(const omni::fabric::Path primPath,
                              omni::fabric::StageReaderWriter& stage,
                              pxr::UsdGeomXformCache& xfCache);

private:
    omni::physx::IPhysxSimulation* mPhysXSimulation;
    omni::fabric::UsdStageId mStageId;
    omni::physx::IPhysx* mPhysX;
    bool mUpdate;
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

    omni::fabric::Token mResidualRmsPosIterToken;
    omni::fabric::Token mResidualMaxPosIterToken;
    omni::fabric::Token mResidualRmsVelIterToken;
    omni::fabric::Token mResidualMaxVelIterToken;

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
    pxr::TfNotice::Key mNoticeListenerKey;

    omni::physx::SubscriptionId mSubscriptionObjId;

    DirectGpuHelper mDirectGpuHelper;
#if !CARB_AARCH64
    DeformableBodyManagerDeprecated mDeformableBodyManagerDeprecated;
    DeformableSurfaceManagerDeprecated mDeformableSurfaceManagerDeprecated;
    ParticleManagerDeprecated mParticleManagerDeprecated;
    VolumeDeformableBodyManager mVolumeDeformableBodyManager;
    SurfaceDeformableBodyManager mSurfaceDeformableBodyManager;
#endif
};
} // namespace physx
} // namespace omni
