// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "FabricManager.h"
#include "FabricKernels.h"

#include <omni/fabric/connectivity/Connectivity.h>
#include <omni/fabric/IFabric.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/SimStageWithHistory.h>
#include <usdrt/scenegraph/usd/usd/stage.h>

#include <carb/settings/ISettings.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>

#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/physx/PhysxTokens.h>

#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>

#include <cuda_runtime_api.h>

#include <PxPhysicsAPI.h>
#include <common/foundation/TypeCast.h>

#include <tbb/concurrent_hash_map.h>

#include "gpu/PxPhysicsGpu.h"
#include "gpu/PxGpu.h"

using namespace pxr;
using namespace carb;
using namespace carb::tasking;
using namespace omni::fabric;

extern carb::settings::ISettings* gSettings;

const omni::fabric::IToken* omni::fabric::Token::iToken = nullptr;

pxr::UsdStageRefPtr gStage = nullptr;
omni::physx::IPhysxPrivate* gPhysXPrivate = nullptr;

extern omni::physx::FabricManager* gFabricManager;

using namespace omni::fabric;
using namespace omni::physx;
using namespace physx;
using ::physx::PxVec3;
using ::physx::PxQuat;
using ::physx::PxTransform;


namespace
{
    GfMatrix4d computeMatrix(const pxr::GfVec3d& translate, const pxr::GfMatrix3d& rotate, const pxr::GfVec3d& scale)
    {
        // Order is scale*rotate*translate
        return GfMatrix4d(rotate[0][0] * scale[0], rotate[0][1] * scale[0], rotate[0][2] * scale[0], 0,
                               rotate[1][0] * scale[1], rotate[1][1] * scale[1], rotate[1][2] * scale[1], 0,
                               rotate[2][0] * scale[2], rotate[2][1] * scale[2], rotate[2][2] * scale[2], 0,
                               translate[0], translate[1], translate[2], 1);
    }

    GfMatrix4d computeLocalMatrix(omni::fabric::USDHierarchy& usdHierarchy, omni::fabric::StageReaderWriter& stage,
                                  const omni::fabric::Path& path, const GfMatrix4d& worldMatrix, 
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
}

namespace omni
{
namespace physx
{
inline float radToDeg(const float a)
{
    return 57.29577951308232286465f * a;
}

void FabricUsdNoticeListener::handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    // This is an old callback, ignore it
    if (!gStage || !gFabricManager || gStage != objectsChanged.GetStage())
    {
        return;
    }

    TRACE_FUNCTION();

    CARB_PROFILE_ZONE(0, "FabricManager::NoticeHandler");

    for (const SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        const SdfPath primPath = gStage->GetPseudoRoot().GetPath() == path ?
            gStage->GetPseudoRoot().GetPath() :
            path.GetPrimPath();

        // If prim is removed, remove it and its descendants from selection.
        const UsdPrim prim = gStage->GetPrimAtPath(primPath);
        if (prim.IsValid() == false || !prim.IsActive()) // remove prim
        {
            gFabricManager->removePrim(primPath);
        }
        else // resync prim
        {
            gFabricManager->resyncPrim(primPath);
        }
    }

    for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
    {
        const SdfPath primPath = gStage->GetPseudoRoot().GetPath() == path ? path : path.GetPrimPath();

        const bool isAttributePath = path.IsPropertyPath();
        if (isAttributePath)
        {
            TfToken const& attrName = path.GetNameToken();
            if (attrName == UsdPhysicsTokens->physicsKinematicEnabled)
            {
                const UsdPrim prim = gStage->GetPrimAtPath(primPath);
                if (prim)
                {
                    bool val = false;
                    prim.GetAttribute(attrName).Get(&val);
                    gFabricManager->toggleKinematics(val, prim);
                }
            }
        }
    }
}

void onPhysXObjectCreatedCallback(const pxr::SdfPath& ,
    usdparser::ObjectId ,
    PhysXType ,
    void* userData)
{
    FabricManager* fm = reinterpret_cast<FabricManager*>(userData);
    fm->setPtrDirty();
}

void onPhysXObjectDeletedCallback(const pxr::SdfPath& ,
    usdparser::ObjectId ,
    PhysXType ,
    void* userData)
{
    FabricManager* fm = reinterpret_cast<FabricManager*>(userData);
    fm->setPtrDirty();
}

void onPhysXObjectDeleteAllCallback(void* userData)
{
    FabricManager* fm = reinterpret_cast<FabricManager*>(userData);
    fm->setPtrDirty();
}

FabricManager::FabricManager()
    : mPhysXSimulation(nullptr), mUpdate(false), mPtrDirty(true), mKinematicBodyTransformationUpdateEnabled(true), mNoticeListener(nullptr)
{
    mStageId.id = 0;
    carb::Framework * framework = carb::getFramework();
    mPhysXSimulation = carb::getCachedInterface<omni::physx::IPhysxSimulation>();
    mPhysX = carb::getCachedInterface<omni::physx::IPhysx>();

    gPhysXPrivate = carb::getCachedInterface<omni::physx::IPhysxPrivate>();

    omni::fabric::Token::iToken = getFramework()->tryAcquireInterface<omni::fabric::IToken>();

    mWorldMatrixToken = omni::fabric::Token::iToken->getHandle(gWorldMatrixTokenString);
    mLocalMatrixToken = omni::fabric::Token::iToken->getHandle(gLocalMatrixTokenString);
    mWorldForceToken = omni::fabric::Token::iToken->getHandle(gWorldForceTokenString);
    mWorldTorqueToken = omni::fabric::Token::iToken->getHandle(gWorldTorqueTokenString);
    mPointsToken = omni::fabric::Token::iToken->getHandle(gPointsTokenString);
    mInitPointsToken = omni::fabric::Token::iToken->getHandle(gInitPointsTokenString);
    mPhysXPtrToken = omni::fabric::Token::iToken->getHandle(gPhysXPtrTokenString);
    mPhysXPtrInstancedToken = omni::fabric::Token::iToken->getHandle(gPhysXPtrInstancedTokenString);

    mDynamicBodyToken = omni::fabric::Token::iToken->getHandle(gDynamicBodyTokenString);

    mLinVelToken = omni::fabric::Token::iToken->getHandle(UsdPhysicsTokens->physicsVelocity.GetText());
    mAngVelToken = omni::fabric::Token::iToken->getHandle(UsdPhysicsTokens->physicsAngularVelocity.GetText());

    mResidualRmsPosIterToken = omni::fabric::Token::iToken->getHandle(PhysxSchemaTokens->physxResidualReportingRmsResidualPositionIteration.GetText());
    mResidualMaxPosIterToken = omni::fabric::Token::iToken->getHandle(PhysxSchemaTokens->physxResidualReportingMaxResidualPositionIteration.GetText());
    mResidualRmsVelIterToken = omni::fabric::Token::iToken->getHandle(PhysxSchemaTokens->physxResidualReportingRmsResidualVelocityIteration.GetText());
    mResidualMaxVelIterToken = omni::fabric::Token::iToken->getHandle(PhysxSchemaTokens->physxResidualReportingMaxResidualVelocityIteration.GetText());

    mRigidBodyWorldPositionToken = omni::fabric::Token::iToken->getHandle(gRigidBodyWorldPositionTokenString);
    mRigidBodyWorldOrientationToken = omni::fabric::Token::iToken->getHandle(gRigidBodyWorldOrientationTokenString);
    mRigidBodyWorldScaleToken = omni::fabric::Token::iToken->getHandle(gRigidBodyWorldScaleTokenString);

    mFloat1Type = omni::fabric::Type(
        omni::fabric::BaseDataType::eFloat, 1, 0, omni::fabric::AttributeRole::eNone);
    mFloat3Type = omni::fabric::Type(
        omni::fabric::BaseDataType::eFloat, 3, 0, omni::fabric::AttributeRole::eNone);
    mDouble3Type = omni::fabric::Type(
        omni::fabric::BaseDataType::eDouble, 3, 0, omni::fabric::AttributeRole::eNone);
    mQuatType = omni::fabric::Type(
        omni::fabric::BaseDataType::eFloat, 4, 0, omni::fabric::AttributeRole::eQuaternion);
    mMatrix4dType = omni::fabric::Type(
        BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);

    mFloat3ArrayType = omni::fabric::Type(
        omni::fabric::BaseDataType::eFloat, 3, 1, omni::fabric::AttributeRole::ePosition);

    mPtrType = omni::fabric::Type(
        omni::fabric::BaseDataType::eUInt64, 1, 0, omni::fabric::AttributeRole::eNone);

    mTagType = omni::fabric::Type(BaseDataType::eTag, 1, 0, AttributeRole::eNone);

    mPtrInstancedType = omni::fabric::Type(omni::fabric::BaseDataType::eUInt64, 1, 1, omni::fabric::AttributeRole::eNone);

    mTokenJointStates[0] =
        FabricJointStateToken{ Token("PhysicsJointStateAPI:angular"), Token("state:angular:physics:position"),
                                  Token("state:angular:physics:velocity"), UsdPhysicsTokens->angular,
                                  ::physx::PxArticulationAxis::eTWIST };
    mTokenJointStates[1] =
        FabricJointStateToken{ Token("PhysicsJointStateAPI:linear"), Token("state:linear:physics:position"),
                                  Token("state:linear:physics:velocity"), UsdPhysicsTokens->linear,
                                  ::physx::PxArticulationAxis::eX };
    mTokenJointStates[2] =
        FabricJointStateToken{ Token("PhysicsJointStateAPI:rotX"), Token("state:rotX:physics:position"),
                                  Token("state:rotX:physics:velocity"), UsdPhysicsTokens->rotX,
                                  ::physx::PxArticulationAxis::eTWIST };
    mTokenJointStates[3] =
        FabricJointStateToken{ Token("PhysicsJointStateAPI:rotY"), Token("state:rotY:physics:position"),
                                  Token("state:rotY:physics:velocity"), UsdPhysicsTokens->rotY,
                                  ::physx::PxArticulationAxis::eSWING1 };
    mTokenJointStates[4] =
        FabricJointStateToken{ Token("PhysicsJointStateAPI:rotZ"), Token("state:rotZ:physics:position"),
                                  Token("state:rotZ:physics:velocity"), UsdPhysicsTokens->rotZ,
                                  ::physx::PxArticulationAxis::eSWING2 };

    omni::physx::IPhysicsObjectChangeCallback callback;
    callback.objectDestructionNotifyFn = onPhysXObjectDeletedCallback;
    callback.allObjectsDestructionNotifyFn = onPhysXObjectDeleteAllCallback;
    callback.objectCreationNotifyFn = onPhysXObjectCreatedCallback;
    callback.userData = this;
    mSubscriptionObjId = mPhysX->subscribeObjectChangeNotifications(callback);

}

FabricManager::~FabricManager()
{
    mPhysX->unsubscribeObjectChangeNotifications(mSubscriptionObjId);
    pause();
    release();

    gPhysXPrivate = nullptr;
}

PxCudaContextManager* FabricManager::getCudaContextManager()
{
    if (gPhysXPrivate)
    {
        return gPhysXPrivate->getCudaContextManager();
    }
    return nullptr;
}

void FabricManager::release()
{
    if (mPhysX)
        mPhysX->disableResetOnStop(false);

    mWheelVehicleMap.clear();
    mPointInstanceProtos.clear();
    mPhysXSimulation = nullptr;
    mPhysX = nullptr;
    mStageId.id = 0;
}

void FabricManager::attach(unsigned long stageId)
{
    mStageId.id = stageId;

    mDirectGpuHelper.attach(stageId);
}

void FabricManager::detach()
{

    mStageId.id = 0;
    mWheelVehicleMap.clear();
    mPointInstanceProtos.clear();

    mDirectGpuHelper.detach();

    mResyncPaths.clear();
}

void updateFabric(float elapsedTime, float currentTime, void* userData)
{
    if (!userData)
        return;

    FabricManager* fcManager = (FabricManager*)userData;

    fcManager->update(currentTime, elapsedTime);
}

void FabricManager::update(float currentTime, float elapsedSecs, bool forceUpdate)
{
    const bool enabled = gSettings->getAsBool(kSettingFabricEnabled);

    CARB_PROFILE_ZONE(0, "FabricManager::update");
    if (mUpdate && enabled)
    {
        IStageReaderWriter* iSip = carb::getCachedInterface<IStageReaderWriter>();
        StageReaderWriter stage = iSip->get(mStageId);

        const bool updateTransformations = gSettings->getAsBool(kSettingFabricUpdateTransformations);
        const bool updateVelocities = gSettings->getAsBool(kSettingFabricUpdateVelocities);
        const bool updateJointStates = gSettings->getAsBool(kSettingFabricUpdateJointStates);
        const bool updatePoints = gSettings->getAsBool(kSettingFabricUpdatePoints);
        const bool updateResiduals = gSettings->getAsBool(kSettingFabricUpdateResiduals);

        ITasking* tasking = carb::getCachedInterface<ITasking>();

        if (!mResyncPaths.empty() && gStage)
        {
            UsdGeomXformCache xformCache;
            for (const SdfPath& resyncPath : mResyncPaths)
            {
                const pxr::UsdPrim prim = gStage->GetPrimAtPath(resyncPath);
                if (!prim)
                    continue;
                if (prim.HasAPI<UsdPhysicsRigidBodyAPI>())
                {                    
                    StageReaderWriterId stageInProgress = iSip->get(mStageId);
                    std::vector<usdrt::SdfPath> paths;
                    paths.push_back(usdrt::SdfPath(prim.GetPrimPath().GetText()));
                    initializeRigidBodyBatched(paths, iSip, stageInProgress);
                }
                else if (prim.HasAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>())
                {
                    initializeWheel(xformCache, prim, iSip, stage.getId());
                }
                else if (prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
                {
                    initializeSoftBodyDeprecated(xformCache, prim, iSip, stage.getId());
                }
                else if (prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
                {
                    initializeDeformableSurfaceDeprecated(xformCache, prim, iSip, stage.getId());
                }
                else if (prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
                {
                    initializeParticleClothDeprecated(xformCache, prim, iSip, stage.getId());
                }
                else if (prim.IsA<UsdGeomPointInstancer>())
                {
                    initializePointInstancer(gStage, UsdGeomPointInstancer{ prim }, xformCache, stage);
                }
                else if (prim.HasAPI(UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformableBodyAPI)))
                {
                    initializeDeformableBody(xformCache, prim, iSip, stage.getId());
                }
            }

            mResyncPaths.clear();
        }

        IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
        CARB_ASSERT(iPhysX);

        // pause change tracking so that we dont get transformations back
        PauseChangeTrackingScope changeTrackingPauseScope(mPhysXSimulation);

        // if readback is suppressed, let the GPU helper handle the update
        if (iPhysX->isReadbackSuppressed())
        {
            mDirectGpuHelper.update(stage, forceUpdate);
#if !CARB_AARCH64            
            mParticleManagerDeprecated.update(stage);
            mDeformableBodyManagerDeprecated.update(stage);
            mDeformableSurfaceManagerDeprecated.update(stage);
            mVolumeDeformableBodyManager.update(stage);
            mSurfaceDeformableBodyManager.update(stage);
#endif
            return;
        }

        const Type typePrimName(BaseDataType::eTag, 1, 0, AttributeRole::ePrimTypeName);
        const Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
        const Type typeAppliedType(BaseDataType::eTag, 1, 0, AttributeRole::eNone);
        const Type typeDouble3(BaseDataType::eDouble, 3, 0, AttributeRole::eNone);
        const Type typeMatrix4d(BaseDataType::eDouble, 16, 0, AttributeRole::eMatrix);
        const Type typeFloat3(BaseDataType::eFloat, 3, 0, AttributeRole::eVector);
        const Type typeQuat(BaseDataType::eFloat, 4, 0, AttributeRole::eQuaternion);
        const Type typeUint64(fabric::BaseDataType::eUInt64);
        const Type typeFloat(fabric::BaseDataType::eFloat);

        const Token tokenVehicleWheelAttachment("PhysxVehicleWheelAttachmentAPI");
        const Token tokenVehicleWheel("PhysxVehicleWheel");
        const Token tokenRigidBody("PhysicsRigidBodyAPI");
        const Token tokenResidualReporting("PhysxResidualReportingAPI");

        omni::fabric::USDHierarchy usdHierarchy(stage.getFabricId());

        // Update rigid bodies
        if (updateTransformations || updateVelocities)
        {
            CARB_PROFILE_ZONE(0, "FabricManager::update - Rigid Bodies");
            UsdGeomXformCache xformCache;
            const size_t minBatchSize = 2000;
            const size_t numThreads = tasking->getDesc().threadCount;

            const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenRigidBody),
                                                                       AttrNameAndType_v2(typeAppliedType, mDynamicBodyToken) };
            const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(typeMatrix4d, mWorldMatrixToken),
                                                                        AttrNameAndType_v2(typeMatrix4d, mLocalMatrixToken),
                                                                        AttrNameAndType_v2(typeFloat3, mLinVelToken),
                                                                        AttrNameAndType_v2(typeFloat3, mAngVelToken),
                                                                        AttrNameAndType_v2(typeUint64, mPhysXPtrToken),
                                                                        AttrNameAndType_v2(typeUint64, mPhysXPtrInstancedToken),
                                                                        AttrNameAndType_v2(typeDouble3, mRigidBodyWorldPositionToken),
                                                                        AttrNameAndType_v2(typeQuat, mRigidBodyWorldOrientationToken),
                                                                        AttrNameAndType_v2(typeFloat3, mRigidBodyWorldScaleToken) };

            PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
            size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<pxr::GfMatrix4d> worldMatrices =
                    stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken);
                gsl::span<pxr::GfMatrix4d> localMatrices =
                   stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mLocalMatrixToken);

                gsl::span<carb::Float3> linVelocities = stage.getAttributeArray<carb::Float3>(primBuckets, i, mLinVelToken);
                gsl::span<carb::Float3> angVelocities = stage.getAttributeArray<carb::Float3>(primBuckets, i, mAngVelToken);

                gsl::span<uint64_t> physxPtrs = stage.getAttributeArray<uint64_t>(primBuckets, i, mPhysXPtrToken);

                gsl::span<carb::Double3> rbWorldPositions =
                    stage.getAttributeArray<carb::Double3>(primBuckets, i, mRigidBodyWorldPositionToken);
                gsl::span<carb::Float4> rbWorldOrientations =
                    stage.getAttributeArray<carb::Float4>(primBuckets, i, mRigidBodyWorldOrientationToken);
                gsl::span<carb::Float3> rbWorldScales = stage.getAttributeArray<carb::Float3>(primBuckets, i, mRigidBodyWorldScaleToken);

                const gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                const size_t numPaths = paths.size();
                const size_t threadBatchSize = numPaths/numThreads;
                const size_t batchSize = threadBatchSize > minBatchSize ? threadBatchSize : minBatchSize;
                const size_t numBatches = (numPaths / batchSize) + 1;

                auto&& computeFunc = [this, iPhysX, numPaths, batchSize, numBatches, paths, &stage, &usdHierarchy,
                                      &physxPtrs, &worldMatrices, &localMatrices, &linVelocities, &angVelocities,
                                      &rbWorldPositions, &rbWorldOrientations, &rbWorldScales, &xformCache](size_t batchIndex)
                {
                    //CARB_PROFILE_ZONE(0, "RigidBodyTransformsUpdateFn");                
                    const size_t startIndex = batchIndex * batchSize;
                    const size_t endIndex = (batchIndex == (numBatches - 1)) ? numPaths : (batchIndex + 1) * batchSize;
                    for (size_t j = startIndex; j < endIndex; j++)
                    {
                        const omni::fabric::Path& path = paths[j];
                        if (updatePointInstancer(path, stage, xformCache))
                        {
                            continue;
                        }

                        const ::physx::PxBase* physxPtr = nullptr;
                        if (mPtrDirty)
                        {
                            const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
                            physxPtr = reinterpret_cast<const ::physx::PxBase*>(iPhysX->getPhysXPtr(usdPath, omni::physx::ePTActor));
                            if (!physxPtr)
                                physxPtr = reinterpret_cast<const ::physx::PxBase*>(iPhysX->getPhysXPtr(usdPath, omni::physx::ePTLink));

                            physxPtrs[j] = reinterpret_cast<uint64_t>(physxPtr);
                        }
                        else
                        {
                            physxPtr = reinterpret_cast<const ::physx::PxBase*>(physxPtrs[j]);
                        }

                        if (!physxPtr)
                        {                         
                            continue;
                        }

                        const ::physx::PxRigidBody* rigidBody = physxPtr->is<::physx::PxRigidBody>();
                        if (!rigidBody)
                        {                         
                            continue;
                        }

                        const ::physx::PxTransform rigidBodyWorldTransformation = rigidBody->getGlobalPose();
                        const ::physx::PxVec3 rigidBodyLinearVelocity = rigidBody->getLinearVelocity();
                        const ::physx::PxVec3 rigidBodyAngularVelocity = rigidBody->getAngularVelocity();

                        pxr::GfVec3d translation(double(rigidBodyWorldTransformation.p.x),
                            double(rigidBodyWorldTransformation.p.y),
                            double(rigidBodyWorldTransformation.p.z));
                        pxr::GfQuatd rotation(rigidBodyWorldTransformation.q.w,
                                { rigidBodyWorldTransformation.q.x, rigidBodyWorldTransformation.q.y,
                                  rigidBodyWorldTransformation.q.z });

                        pxr::GfVec3d scale(1.0);
                        TransformationCache::const_iterator fit = mInitialTransformation.find(PathC(path).path);
                        if (fit != mInitialTransformation.end())
                        {

                            scale = pxr::GfVec3f(
                                double(fit->second.scale.x), double(fit->second.scale.y),
                                                 double(fit->second.scale.z));
                        }

                        worldMatrices[j] = computeMatrix(translation, pxr::GfMatrix3d(rotation), scale);
                        localMatrices[j] = computeLocalMatrix(usdHierarchy, stage, path, worldMatrices[j], mWorldMatrixToken);

                        rbWorldPositions[j] = omni::physx::toDouble3(rigidBodyWorldTransformation.p);
                        rbWorldOrientations[j] = omni::physx::toFloat4(rigidBodyWorldTransformation.q);
                        rbWorldScales[j] = omni::physx::toFloat3(scale);

                        linVelocities[j] = omni::physx::toFloat3(rigidBodyLinearVelocity);
                        angVelocities[j] = omni::physx::toFloat3(rigidBodyAngularVelocity);
                    }
                };
                {                    
                    tasking->parallelFor(size_t(0), numBatches, computeFunc);
                }
            }
        }
        mPtrDirty = false;

        // Update vehicle wheels
        if (updateTransformations)
        {
            CARB_PROFILE_ZONE(0, "FabricManager::update - Vehicle Wheels");
           const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenVehicleWheelAttachment),
                                                                       AttrNameAndType_v2(typeMatrix4d, mWorldMatrixToken),
                                                                       AttrNameAndType_v2(typeMatrix4d, mLocalMatrixToken)
            };
            //AttrNameAndType_v2(typeFloat3, tokenScale) };

            PrimBucketList primBuckets = stage.findPrims(requiredAll);
            size_t bucketCount = primBuckets.bucketCount();

            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<pxr::GfMatrix4d> outWorldMatrices =
                    stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken);
                gsl::span<pxr::GfMatrix4d> outLocalMatrices =
                   stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mLocalMatrixToken);

                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);

                {
                    size_t j = 0;
                    for (const omni::fabric::Path& path : paths)
                    {
                        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
                        const int wheelIndex = iPhysX->getWheelIndex(usdPath);
                        if (wheelIndex >= 0)
                        {
                            WheelMap::const_iterator fit = mWheelVehicleMap.find(PathC(path).path);
                            if (fit != mWheelVehicleMap.end() && fit->second > 0)
                            {
                                omni::physx::usdparser::ObjectId vehicleId = fit->second;
                                carb::Float3 wPos;
                                carb::Float4 wOrient;

                                const bool success = iPhysX->getWheelTransformations(vehicleId, &wheelIndex, 1, true, &wPos, &wOrient);

                                if (success)
                                {
                                    pxr::GfVec3d translation(double(wPos.x), double(wPos.y), double(wPos.z));
                                    pxr::GfQuatd rotation(wOrient.w, wOrient.x, wOrient.y, wOrient.z);
                                    pxr::GfVec3f scale(1.0f);
                                    TransformationCache::const_iterator fit =
                                        mInitialTransformation.find(PathC(path).path);
                                    if (fit != mInitialTransformation.end())
                                    {

                                        scale = pxr::GfVec3f(double(fit->second.scale.x), double(fit->second.scale.y),
                                                             double(fit->second.scale.z));
                                    }

                                    outWorldMatrices[j] = computeMatrix(translation, pxr::GfMatrix3d(rotation), scale);

                                    outLocalMatrices[j] = computeLocalMatrix(usdHierarchy, stage, path, 
                                                                             outWorldMatrices[j], mWorldMatrixToken);
                                }
                                else
                                {
                                    CARB_LOG_ERROR("Wheel transformation data were not found.");
                                }
                            }
                            else
                            {
                                CARB_LOG_ERROR("Wheel vehicle was not found.");
                            }
                        }
                        else
                        {
                            CARB_LOG_ERROR("Wheel index was not found.");
                        }
                        j++;
                    }
                }
            }

        }

        if (updateResiduals)
        {
            CARB_PROFILE_ZONE(0, "FabricManager::update - Residuals");

            const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenResidualReporting) };
            const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(typeFloat, mResidualRmsPosIterToken),
                AttrNameAndType_v2(typeFloat, mResidualMaxPosIterToken),
                AttrNameAndType_v2(typeFloat, mResidualRmsVelIterToken) ,
                AttrNameAndType_v2(typeFloat, mResidualMaxVelIterToken) };

            const PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
            const size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i < bucketCount; i++)
            {
                gsl::span<float> residualRmsPosIter = stage.getAttributeArray<float>(primBuckets, i, mResidualRmsPosIterToken);
                gsl::span<float> residualMaxPosIter = stage.getAttributeArray<float>(primBuckets, i, mResidualMaxPosIterToken);
                gsl::span<float> residualRmsVelIter = stage.getAttributeArray<float>(primBuckets, i, mResidualRmsVelIterToken);
                gsl::span<float> residualMaxVelIter = stage.getAttributeArray<float>(primBuckets, i, mResidualMaxVelIterToken);

                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                size_t j = 0;
                for (const omni::fabric::Path path : paths)
                {
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);

                    PxReal residualRmsPosIterValue = 0.0f;
                    PxReal residualMaxPosIterValue = 0.0f;
                    PxReal residualRmsVelIterValue = 0.0f;
                    PxReal residualMaxVelIterValue = 0.0f;

                    const ::physx::PxArticulationReducedCoordinate* physxPtr = reinterpret_cast<const ::physx::PxArticulationReducedCoordinate*>(iPhysX->getPhysXPtr(usdPath, omni::physx::ePTArticulation));
                    if (physxPtr != NULL)
                    {
                        PxArticulationResidual residual = physxPtr->getSolverResidual();
                        residualRmsPosIterValue = residual.positionIterationResidual.rmsResidual;
                        residualMaxPosIterValue = residual.positionIterationResidual.maxResidual;
                        residualRmsVelIterValue = residual.velocityIterationResidual.rmsResidual;
                        residualMaxVelIterValue = residual.velocityIterationResidual.maxResidual;
                    }
                    else
                    {
                        const ::physx::PxJoint* physxPtr = reinterpret_cast<const ::physx::PxJoint*>(iPhysX->getPhysXPtr(usdPath, omni::physx::ePTJoint));
                        if (physxPtr != NULL)
                        {
                            PxConstraintResidual residual = physxPtr->getConstraint()->getSolverResidual();
                            residualRmsPosIterValue = residual.positionIterationResidual;
                            residualMaxPosIterValue = residual.positionIterationResidual;
                            residualRmsVelIterValue = residual.velocityIterationResidual;
                            residualMaxVelIterValue = residual.velocityIterationResidual;
                        }
                        else
                        {
                            const ::physx::PxScene* physxPtr = reinterpret_cast<const ::physx::PxScene*>(iPhysX->getPhysXPtr(usdPath, omni::physx::ePTScene));
                            if (physxPtr != NULL)
                            {
                                PxSceneResidual residual = physxPtr->getSolverResidual();
                                residualRmsPosIterValue = residual.positionIterationResidual.rmsResidual;
                                residualMaxPosIterValue = residual.positionIterationResidual.maxResidual;
                                residualRmsVelIterValue = residual.velocityIterationResidual.rmsResidual;
                                residualMaxVelIterValue = residual.velocityIterationResidual.maxResidual;
                            }
                            else
                            {
                                j++;
                                continue;
                            }
                        }
                    }

                    residualRmsPosIter[j] = residualRmsPosIterValue;
                    residualMaxPosIter[j] = residualMaxPosIterValue;
                    residualRmsVelIter[j] = residualRmsVelIterValue;
                    residualMaxVelIter[j] = residualMaxVelIterValue;
                    
                    j++;
                }
            }
        }

        if (updateJointStates)
        {
            CARB_PROFILE_ZONE(0, "FabricManager::update - Joint States");
            for(auto& ts : mTokenJointStates)
            {
                const Token tokenJointStateAPI = ts.token;
               const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenJointStateAPI) };
               const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(typeFloat, ts.madPositionToken),
                                                                        AttrNameAndType_v2(typeFloat,  ts.madVelocityToken) };

                PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
                size_t bucketCount = primBuckets.bucketCount();
                for (size_t i = 0; i != bucketCount; i++)
                {
                    gsl::span<float> jointPositions = stage.getAttributeArray<float>(primBuckets, i, ts.madPositionToken);
                    gsl::span<float> jointVelocities = stage.getAttributeArray<float>(primBuckets, i, ts.madVelocityToken);
                    if(jointPositions.empty())
                    {
                        CARB_LOG_ERROR("FabricManager::update - Joint positions are empty");
                        continue;
                    }
                    if(jointVelocities.empty())
                    {
                        CARB_LOG_ERROR("FabricManager::update - Joint velocities are empty");
                        continue;
                    }
                    gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                    size_t fcItemIndex = 0;
                    for (const omni::fabric::Path& path : paths)
                    {
                        pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);
                        ::physx::PxArticulationJointReducedCoordinate* physxPtr = reinterpret_cast<::physx::PxArticulationJointReducedCoordinate*>(iPhysX->getPhysXPtr(usdPath, omni::physx::ePTLinkJoint));

                        if (!physxPtr)
                        {
                            fcItemIndex++;
                            continue;
                        }

                        JointStateCache::const_iterator it = mInitialJointStates.find(PathC(path).path);
                        if (it != mInitialJointStates.end())
                        {
                            const omni::physx::JointStateData& jointStateData = it->second;

                            for(int jointAxis = 0; jointAxis < 6; ++jointAxis)
                            {
                                if(!jointStateData.enabled[jointAxis])
                                    continue;
                                if(jointStateData.fabricTokenC[jointAxis] != omni::fabric::asInt(ts.usdToken).token)
                                    continue;
                                if(jointStateData.physxAxis[jointAxis] != ts.physxAxis)
                                    continue;
                                float jointPos;
                                float jointVel;
                                if(jointStateData.body0IsParentLink)
                                {
                                    // We are consistent with InternalJoint::getArticulationJointPosition behaviour
                                    jointPos = physxPtr->getJointPosition(ts.physxAxis);
                                    jointVel = physxPtr->getJointVelocity(ts.physxAxis);
                                }
                                else
                                {
                                    jointPos = -physxPtr->getJointPosition(ts.physxAxis);
                                    jointVel = -physxPtr->getJointVelocity(ts.physxAxis);
                                }

                                if(jointStateData.convertToDegrees[jointAxis])
                                {
                                    jointPositions[fcItemIndex]  = radToDeg(jointPos);
                                    jointVelocities[fcItemIndex] = radToDeg(jointVel);
                                }
                                else
                                {
                                    jointPositions[fcItemIndex]  = jointPos;
                                    jointVelocities[fcItemIndex] = jointVel;
                                }
                                break;
                            }
                        }
                        fcItemIndex++;
                    }
                }
            }
        }

#if !CARB_AARCH64
        mDeformableBodyManagerDeprecated.update(stage);
        mDeformableSurfaceManagerDeprecated.update(stage);
        mParticleManagerDeprecated.update(stage);
        mVolumeDeformableBodyManager.update(stage);
        mSurfaceDeformableBodyManager.update(stage);
#endif
    }
}

void FabricManager::removePrim(const pxr::SdfPath& path)
{
}

void FabricManager::resyncPrim(const pxr::SdfPath& path)
{
    mResyncPaths.insert(path);
}

void FabricManager::initializeRigidBodyBatched(const std::vector<usdrt::SdfPath>& rbPaths,
    IStageReaderWriter* iStageReaderWriter, StageReaderWriterId stageInProgress)
{
    ITasking* tasking = carb::getCachedInterface<ITasking>();
    IPhysx* iPhysx = carb::getCachedInterface<IPhysx>();
    StageReaderWriter stage(stageInProgress);

    mPtrDirty = true;

    struct RigidBodyInitData
    {
        bool validPath;
        bool usdPath;
        omni::fabric::PathC path;
        bool dynamicBody;
        GfTransform transform;
    };

    const size_t numPaths = rbPaths.size();
    std::vector<RigidBodyInitData> rbInitData(numPaths);
    std::unordered_map<omni::fabric::PathC, GfTransform> rbInitTransforms;

    usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(mStageId.id, stageInProgress);

    // compute the transformations, get the 
    const size_t batchSize = 200;
    const size_t numBatches = (numPaths / batchSize) + 1;

    auto&& computeFunc = [this, usdrtStage,&stage, numPaths, batchSize, numBatches, rbPaths,
                          &rbInitData](size_t batchIndex)
    {
        const size_t startIndex = batchIndex * batchSize;
        const size_t endIndex = (batchIndex == (numBatches - 1)) ? numPaths : (batchIndex + 1) * batchSize;
        for (size_t i = startIndex; i < endIndex; i++)
        {
            const omni::fabric::PathC pathC(rbPaths[i]);
            auto iterProto = mPointInstanceProtos.find(pathC.path);
            const pxr::SdfPath usdPath = (iterProto != mPointInstanceProtos.end()) ? iterProto->second.usdProtoPath : omni::fabric::toSdfPath(pathC);
            const UsdPrim prim = gStage->GetPrimAtPath(usdPath);
            
            const TransformationCache::const_iterator initialtranformFit = mInitialTransformation.find(pathC.path);
            if (initialtranformFit != mInitialTransformation.end())
            {
                // already initialized
                rbInitData[i].validPath = false;
            }
             else
            {
                if (!prim)
                {
                    usdrt::UsdPrim usdrt_prim = usdrtStage->GetPrimAtPath(usdrt::SdfPath(pathC));
                    if (!usdrt_prim)
                    {
                        rbInitData[i].validPath = false;
                        continue;
                    }
                    else
                    {
                        bool isTrackedBody = true;
                        if (!mKinematicBodyTransformationUpdateEnabled)
                        {
                            bool kinematic = false;
                            usdrt_prim.GetAttribute(usdrt::TfToken(UsdPhysicsTokens->physicsKinematicEnabled)).Get(&kinematic);
                            if (kinematic)
                            {
                                isTrackedBody = false;
                                rbInitData[i].dynamicBody = false;
                            }
                        }
                        if (isTrackedBody)
                        {
                            const omni::fabric::Token fabricTransform("omni:fabric:worldMatrix");
                            const pxr::GfMatrix4d* worldPose =
                                stage.getAttributeRd<pxr::GfMatrix4d>(omni::fabric::Path(pathC), fabricTransform);

                            if (worldPose)
                            {
                                const GfTransform tr(*worldPose);
                                rbInitData[i].transform = tr;
                            }
                            rbInitData[i].dynamicBody = true;
                        }
                        rbInitData[i].path = omni::fabric::PathC(rbPaths[i]);                        
                        rbInitData[i].validPath = true;
                        rbInitData[i].usdPath = false;
                    }
                }
                else
                {
                    bool isTrackedBody = true;
                    if (!mKinematicBodyTransformationUpdateEnabled)
                    {
                        bool kinematic = false;
                        prim.GetAttribute(UsdPhysicsTokens->physicsKinematicEnabled).Get(&kinematic);
                        if (kinematic)
                        {
                            isTrackedBody = false;
                            rbInitData[i].dynamicBody = false;
                        }
                    }
                    if (isTrackedBody)
                    {
                        const GfMatrix4d worldPose =
                            UsdGeomXform(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
                        const GfTransform tr(worldPose);

                        rbInitData[i].transform = tr;
                        rbInitData[i].dynamicBody = true;
                    }
                    rbInitData[i].path = omni::fabric::PathC(rbPaths[i]);                    
                    rbInitData[i].validPath = true;
                    rbInitData[i].usdPath = true;
                }
            }
        }
    };
    {
        CARB_PROFILE_ZONE(0, "FabricManager::resume:rigidBodyInitialization:transformationsGet");                
        tasking->parallelFor(size_t(0), numBatches, computeFunc);
    }
       
    {
        CARB_PROFILE_ZONE(0, "FabricManager::resume:rigidBodyInitialization:createFabricAttributes");
        auto initTransformsTask = tasking->addTask(Priority::eHigh, nullptr, [&] {            
            CARB_PROFILE_ZONE(0, "FabricManager::resume:rigidBodyInitialization:initTransforms");
            for (const RigidBodyInitData& initData : rbInitData)
            {
                if (!initData.validPath)
                    continue;

                if (initData.dynamicBody)
                {
                    rbInitTransforms[initData.path] = initData.transform;

                    const GfVec3d& wPos = initData.transform.GetTranslation();
                    Float3 pos = { float(wPos[0]), float(wPos[1]), float(wPos[2]) };
                                        
                    const GfQuatd wRot = initData.transform.GetRotation().GetQuat();
                    Float4 orient = { float(wRot.GetImaginary()[0]), float(wRot.GetImaginary()[1]),
                                float(wRot.GetImaginary()[2]), float(wRot.GetReal()) };

                    const GfVec3d& wScale = initData.transform.GetScale();
                    Float3 scale = { float(wScale[0]), float(wScale[1]), float(wScale[2]) };

                    auto& initTrans = mInitialTransformation[initData.path.path];
                    initTrans.translation = pos;
                    initTrans.orientation = orient;
                    initTrans.scale = scale;

                    if (iPhysx->isReadbackSuppressed())
                    {
                        const GfVec3d& wScale = initData.transform.GetScale();
                        scale = { float(wScale[0]), float(wScale[1]), float(wScale[2]) };
                        mDirectGpuHelper.registerRigidBody(initData.path, scale);
                    }
                }
            }
        });


        static constexpr const size_t kNumPhysicsAttr = 12;
        const std::array<AttrNameAndType, kNumPhysicsAttr> attrNameTypeVec = {
            AttrNameAndType(mMatrix4dType, mWorldMatrixToken),
            AttrNameAndType(mMatrix4dType, mLocalMatrixToken),

            AttrNameAndType(mPtrType, mPhysXPtrToken),

            AttrNameAndType(mFloat3Type, mWorldForceToken),
            AttrNameAndType(mFloat3Type, mWorldTorqueToken),

            AttrNameAndType(mFloat3Type, mLinVelToken),
            AttrNameAndType(mFloat3Type, mAngVelToken),

            // token
            AttrNameAndType(mTagType, mDynamicBodyToken),

            // world trs attributes
            AttrNameAndType(mDouble3Type, mRigidBodyWorldPositionToken),
            AttrNameAndType(mQuatType, mRigidBodyWorldOrientationToken),
            AttrNameAndType(mFloat3Type, mRigidBodyWorldScaleToken),

            // Technically not needed for all, but doing for simplicity
            AttrNameAndType(mPtrInstancedType, mPhysXPtrInstancedToken),
        };


        struct TbbAttrNameAndTypeCompare
        {
            static size_t hash(const omni::fabric::BucketNamesAndTypes& x)
            {
                size_t hash = 0;
                for (const auto& it : x)
                {
                    hash = hash ^ std::hash<omni::fabric::AttrNameAndType>()(it);
                }
                return hash;
            }
            static bool equal(const omni::fabric::BucketNamesAndTypes& x, const omni::fabric::BucketNamesAndTypes& y)
            {
                return x == y;
            }
        };
        using PrimsByBucket = tbb::concurrent_hash_map<omni::fabric::BucketNamesAndTypes,
                                                            std::vector<omni::fabric::PathC>, TbbAttrNameAndTypeCompare>;

        // Figure out the buckets we have
        PrimsByBucket rbsByBucket;
        auto bucketingFunc = [&](size_t index) {
            const RigidBodyInitData& initData = rbInitData[index];
            if (!initData.validPath || !initData.dynamicBody)
            {
                return;
            }

            // Start building the bucket with  the attributes we already have on this prim
            using NamesAndTypes = std::pair<std::vector<omni::fabric::Token>, std::vector<omni::fabric::Type>>;
            omni::fabric::BucketNamesAndTypes bucket;
            NamesAndTypes attrNamesAndTypes = stage.getAttributeNamesAndTypes(initData.path);
            for (size_t i = 0; i < attrNamesAndTypes.first.size(); ++i)
            {
                bucket.insert({ attrNamesAndTypes.second[i], attrNamesAndTypes.first[i] });
            }

            PrimsByBucket::accessor accessor;
            if (rbsByBucket.insert(accessor, std::move(bucket)))
            {
                accessor->second = std::vector<omni::fabric::PathC>();
            }
            accessor->second.push_back(initData.path);
        };
        tasking->parallelFor(size_t(0), rbInitData.size(), bucketingFunc);

        AttrCreateSpec physxAttributes[kNumPhysicsAttr];
        for (size_t i = 0; i < kNumPhysicsAttr; ++i)
        {
            physxAttributes[i].nameAndType = attrNameTypeVec[i];
            physxAttributes[i].value = nullptr;
            physxAttributes[i].arrayElemCount = 0;
        }

        // For each bucket we will add the desired attributes.
        IStageReaderWriterLegacy* srwLegacy = carb::getCachedInterface<omni::fabric::IStageReaderWriterLegacy>();
        for (const auto& bucketIt : rbsByBucket)
        {
            const auto& bucket = bucketIt.first;
            const auto& paths = bucketIt.second;

            CARB_PROFILE_ZONE(0, "CreatePrims (%ld)", paths.size());

            // Get the bucketId of the first path. They will all be equal:
            omni::fabric::BucketIdAndIndex buckedAndIdx = srwLegacy->getBucketIdAndIndex(stage.getId(), paths[0]);

            // Add all the attributes to the bucket in one go.
            srwLegacy->addAttributesToBucket(stage.getId(), buckedAndIdx.bucketId, physxAttributes, kNumPhysicsAttr);
        }

        initTransformsTask.wait();
    }

    {
        CARB_PROFILE_ZONE(0, "FabricManager::resume:rigidBodyInitialization:writeToFabric");

        const Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
        const Type typeAppliedType(BaseDataType::eTag, 1, 0, AttributeRole::eNone);
        const Type typeFloat3(BaseDataType::eFloat, 3, 0, AttributeRole::eVector);
        const Type typeMatrix4d(BaseDataType::eDouble, 16, 0, AttributeRole::eMatrix);

        const Token tokenRigidBody("PhysicsRigidBodyAPI");        

        omni::fabric::USDHierarchy usdHierarchy(stage.getFabricId());

        const omni::fabric::set<AttrNameAndType_v2> requiredAll = {
            AttrNameAndType_v2(typeAppliedSchema, tokenRigidBody), AttrNameAndType_v2(typeAppliedType, mDynamicBodyToken)
        };
        const omni::fabric::set<AttrNameAndType_v2> requiredAny = {
             AttrNameAndType_v2(typeMatrix4d, mWorldMatrixToken), AttrNameAndType_v2(typeMatrix4d, mLocalMatrixToken)
        };

        PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
        size_t bucketCount = primBuckets.bucketCount();
        for (size_t i = 0; i != bucketCount; i++)
        {
            gsl::span<pxr::GfMatrix4d> worldMatrices = stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mWorldMatrixToken);
            gsl::span<pxr::GfMatrix4d> localMatrices = stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, mLocalMatrixToken);

            gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
            size_t j = 0;
            for (const omni::fabric::Path& path : paths)
            {
                pxr::GfMatrix4d wMat(1.0);

                std::unordered_map<omni::fabric::PathC, GfTransform>::const_iterator fit = rbInitTransforms.find(path);
                if (fit != rbInitTransforms.end())
                {
                    const GfTransform& tr = fit->second;
                    wMat = computeMatrix(tr.GetTranslation(), tr.GetRotation(), tr.GetScale());

                    worldMatrices[j] = wMat;
                    // hint: add local transform to rbInitTransforms using ComputeParentToWorldTransform
                    // localMatrices[j] = GfMatrix4d(1.0);
                    localMatrices[j] = computeLocalMatrix(usdHierarchy, stage, path, worldMatrices[j], mWorldMatrixToken);
                }
                j++;
            }            
        }        
    }
}

void FabricManager::toggleKinematics(bool kinematics, const UsdPrim& usdPrim)
{
    if (!mKinematicBodyTransformationUpdateEnabled)
    {
        IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        StageReaderWriterId stageInProgress = iStageReaderWriter->get(mStageId);
        StageReaderWriter stage(stageInProgress);

        const omni::fabric::PathC primPath = omni::fabric::asInt(usdPrim.GetPrimPath());
        if (!kinematics)
        {
            std::vector<usdrt::SdfPath> paths;
            paths.push_back(usdrt::SdfPath(primPath));
            initializeRigidBodyBatched(paths, iStageReaderWriter, stageInProgress);
        }
        else
        {
            stage.destroyTag(primPath, mDynamicBodyToken);
        }
    }
}

void FabricManager::initializeWheel(pxr::UsdGeomXformCache& xfCache, const pxr::UsdPrim& prim, omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress)
{
    const GfMatrix4d worldPose = xfCache.GetLocalToWorldTransform(prim);
    const GfTransform tr(worldPose);

    omni::fabric::PathC primPath = omni::fabric::asInt(prim.GetPrimPath());

    iStageReaderWriter->createAttribute(
        stageInProgress, primPath, mWorldMatrixToken, (omni::fabric::TypeC)mMatrix4dType);
    iStageReaderWriter->createAttribute(
        stageInProgress, primPath, mLocalMatrixToken, (omni::fabric::TypeC)mMatrix4dType);

    // find the vehicle the wheel belongs to
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
    const UsdPrim rootPrim = prim.GetStage()->GetPseudoRoot();
    UsdPrim parent = prim;
    while (parent != rootPrim)
    {
        if (parent.HasAPI<PhysxSchemaPhysxVehicleAPI>())
        {
            const usdparser::ObjectId objectId = iPhysX->getObjectId(parent.GetPrimPath(), ePTVehicle);
            mWheelVehicleMap[primPath.path] = objectId;
            break;
        }
        parent = parent.GetParent();
    }

    // store the initial transformations so that we move things back
    const TransformationCache::const_iterator fit = mInitialTransformation.find(primPath.path);
    if (fit == mInitialTransformation.end())
    {
        const GfVec3d wPos = tr.GetTranslation();
        Float3 pos = { float(wPos[0]), float(wPos[1]), float(wPos[2]) };
        const GfQuatd wRot = tr.GetRotation().GetQuat();
        Float4 orient = { float(wRot.GetImaginary()[0]), float(wRot.GetImaginary()[1]), float(wRot.GetImaginary()[2]),
                          float(wRot.GetReal()) };
        const GfVec3d wScale = tr.GetScale();
        Float3 scale = { float(wScale[0]), float(wScale[1]), float(wScale[2]) };

        auto& initTrans = mInitialTransformation[primPath.path];
        initTrans.translation = pos;
        initTrans.orientation = orient;
        initTrans.scale = scale;
    }
}

void FabricManager::initializeResiduals(const pxr::UsdPrim& prim,
    omni::fabric::IStageReaderWriter* iStageReaderWriter,
    omni::fabric::StageReaderWriterId stageInProgress)
{
    StageReaderWriter stage(stageInProgress);
    const pxr::SdfPath usdPath = prim.GetPrimPath();
    omni::fabric::PathC primPath = omni::fabric::asInt(usdPath);

    std::array<AttrNameAndType, 4> attrNameTypeVec = {
                AttrNameAndType(mFloat1Type, mResidualRmsPosIterToken),
                AttrNameAndType(mFloat1Type, mResidualMaxPosIterToken),
                AttrNameAndType(mFloat1Type, mResidualRmsVelIterToken),
                AttrNameAndType(mFloat1Type, mResidualMaxVelIterToken),
    };

    stage.createAttributes(primPath, attrNameTypeVec);
}

void FabricManager::initializeJointState(const pxr::UsdPrim& prim,
                                            omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                            omni::fabric::StageReaderWriterId stageInProgress)
{
    StageReaderWriter stage(stageInProgress);
    IPhysxJoint* iPhysxJoint = carb::getCachedInterface<IPhysxJoint>();
    IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
    const pxr::SdfPath usdPath = prim.GetPrimPath();
    omni::fabric::PathC primPath = omni::fabric::asInt(usdPath);

    JointStateCache::const_iterator fit = mInitialJointStates.find(primPath.path);
    if (fit != mInitialJointStates.end())
    {
        // already initialized
        return;
    }

    const usdparser::ObjectId objectId = iPhysX->getObjectId(prim.GetPrimPath(), ePTLinkJoint);    

    omni::physx::JointStateData jointStateData;
    iPhysxJoint->getJointStateData(objectId, &jointStateData);
    mInitialJointStates[primPath.path] = jointStateData;
    for (auto& ts : mTokenJointStates)
    {
        const Token tokenJointStateAPI = ts.token;
        for (int jointAxis = 0; jointAxis < 6; ++jointAxis)
        {
            if (!jointStateData.enabled[jointAxis])
                continue;
            // This is needed because otherwise we would be creating both angular and rotX outputs for RevoluteJoint
            // as they both map to physx eTWIST
            if(jointStateData.fabricTokenC[jointAxis] != omni::fabric::asInt(ts.usdToken).token)
                continue;
            if (jointStateData.physxAxis[jointAxis] != ts.physxAxis)
                continue;

            std::array<AttrNameAndType, 2> attrNameTypeVec = {
                AttrNameAndType(mFloat1Type, ts.madPositionToken),
                AttrNameAndType(mFloat1Type, ts.madVelocityToken),
            };

            stage.createAttributes(primPath, attrNameTypeVec);
        }
    }
}

void FabricManager::initializeSoftBodyDeprecated(pxr::UsdGeomXformCache& xfCache, const pxr::UsdPrim& prim,
    omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress)
{
#if !CARB_AARCH64
    mDeformableBodyManagerDeprecated.registerSoftBody(xfCache, mStageId.id, iStageReaderWriter, stageInProgress, prim);
#endif
}

void FabricManager::initializeDeformableSurfaceDeprecated(pxr::UsdGeomXformCache& xfCache, const pxr::UsdPrim& prim,
    omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress)
{
#if !CARB_AARCH64
    mDeformableSurfaceManagerDeprecated.registerDeformableSurface(xfCache, mStageId.id, iStageReaderWriter, stageInProgress, prim);
#endif
}

void FabricManager::initializeParticleClothDeprecated(pxr::UsdGeomXformCache& xfCache, const pxr::UsdPrim& prim,
    omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress)
{
#if !CARB_AARCH64
    mParticleManagerDeprecated.registerParticleCloth(xfCache, mStageId.id, iStageReaderWriter, stageInProgress, prim);
#endif
}

void FabricManager::initializeDeformableBody(pxr::UsdGeomXformCache& xfCache, const pxr::UsdPrim& prim, omni::fabric::IStageReaderWriter* iStageReaderWriter, omni::fabric::StageReaderWriterId stageInProgress)
{
#if !CARB_AARCH64
    mVolumeDeformableBodyManager.registerDeformableBody(xfCache, mStageId.id, iStageReaderWriter, stageInProgress, prim);
    mSurfaceDeformableBodyManager.registerDeformableBody(xfCache, mStageId.id, iStageReaderWriter, stageInProgress, prim);
#endif
   
}

void FabricManager::resume()
{
    CARB_PROFILE_ZONE(0, "FabricManager::resume");
    const bool enabled = gSettings->getAsBool(kSettingFabricEnabled);
    if (!enabled)
        return;

    if (mPhysXSimulation)
    {
        PauseChangeTrackingScope changeTrackingPauseScope(mPhysXSimulation);
        mWheelVehicleMap.clear();
        mPointInstanceProtos.clear();
        mUpdate = true;
        // Disable write of physics transformation, get them through the simulation callback
        physx::ISimulationCallback cb;
        cb.transformationWriteFn = nullptr;
        cb.velocityWriteFn = nullptr;
        cb.transformationUpdateFn = updateFabric;
        cb.userData = this;
        mPhysXSimulation->setSimulationCallback(cb);
        mPhysXSimulation->setSimulationOutputFlags(omni::physx::SimulationOutputType::eTRANSFORMATION,
            omni::physx::SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);
        mPhysXSimulation->addSimulationOutputFlags(omni::physx::SimulationOutputType::eVELOCITY,
            omni::physx::SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);
        mPhysXSimulation->addSimulationOutputFlags(omni::physx::SimulationOutputType::ePOINTS,
            omni::physx::SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);
        mPhysXSimulation->addSimulationOutputFlags(omni::physx::SimulationOutputType::eRESIDUALS,
            omni::physx::SimulationOutputFlag::eSKIP_WRITE, nullptr, 0);

        IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
        ISimStageWithHistory* iSimStageWithHistory = carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
        StageReaderWriterId stageInProgress = iStageReaderWriter->get(mStageId);

        usdrt::UsdStageRefPtr usdrtStage = usdrt::UsdStage::Attach(mStageId, stageInProgress);
        UsdStageWeakPtr usdStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(long(mStageId.id)));
        if (usdStage && usdrtStage)
        {
            gStage = usdStage;
            mNoticeListener = new FabricUsdNoticeListener();
            mNoticeListenerKey =
                pxr::TfNotice::Register(pxr::TfCreateWeakPtr(mNoticeListener), &omni::physx::FabricUsdNoticeListener::handle);

            // If there are prototype prims (i.e. scene graph instancing prototypes) and omnihydra doesn't use scene graph
            // instancing, issue a warning
            if (gStage->GetPrototypes().size() > 0 && !gSettings->getAsBool("/persistent/omnihydra/useSceneGraphInstancing"))
            {
                CARB_LOG_WARN(
                    "Prototype prims (instancing prototypes) are present in the stage but omnihydra scene graph instancing"
                    " is not enabled! Please consider enabling it and reload the stage."
                );
            }

            UsdGeomXformCache xformCache;
            parsePointInstancers(usdStage, usdrtStage, xformCache, iStageReaderWriter, stageInProgress);

            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:rigidBodyInitialization");
                initializeRigidBodyBatched(usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysicsRigidBodyAPI")), iStageReaderWriter, stageInProgress);
            }
            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:wheelAttachmentInitialization");
                for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxVehicleWheelAttachmentAPI")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                    if (prim)
                        initializeWheel(xformCache, prim, iStageReaderWriter, stageInProgress);
                }
            }

            static const std::vector<std::string> jointStateApiSchemas = { 
                                     "PhysicsJointStateAPI:rotX", "PhysicsJointStateAPI:rotY", "PhysicsJointStateAPI:rotZ", "PhysicsJointStateAPI:angular",
                                     "PhysicsJointStateAPI:transX", "PhysicsJointStateAPI:transY", "PhysicsJointStateAPI:transZ", "PhysicsJointStateAPI:linear" };

            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:jointStateInitialization");
                for (const std::string& schemaName : jointStateApiSchemas)
                {
                    for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken(schemaName)))
                    {
                        const omni::fabric::PathC pathC(usdrtPath);
                        const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                        const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                        if (prim)
                            initializeJointState(prim, iStageReaderWriter, stageInProgress);
                    }
                }
            }

            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:residualInitialization");
                for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxResidualReportingAPI")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                    if (prim)
                        initializeResiduals(prim, iStageReaderWriter, stageInProgress);
                }
            }
            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:deformableBodyInitialization");
                for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxDeformableBodyAPI")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                    if (prim)
                        initializeSoftBodyDeprecated(xformCache, prim, iStageReaderWriter, stageInProgress);
                }
            }
            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:deformableSurfaceInitialization");
                for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxDeformableSurfaceAPI")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                    if (prim)
                        initializeDeformableSurfaceDeprecated(xformCache, prim, iStageReaderWriter, stageInProgress);
                }
            }
            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:particleClothInitialization");
                for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("PhysxParticleClothAPI")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                    if (prim)
                        initializeParticleClothDeprecated(xformCache, prim, iStageReaderWriter, stageInProgress);
                }
            }
            {
                CARB_PROFILE_ZONE(0, "FabricManager::resume:deformableBodyInitialization");
                for (auto& usdrtPath : usdrtStage->GetPrimsWithAppliedAPIName(usdrt::TfToken("OmniPhysicsDeformableBodyAPI")))
                {
                    const omni::fabric::PathC pathC(usdrtPath);
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(pathC);
                    const UsdPrim prim = usdStage->GetPrimAtPath(usdPath);
                    if (prim)
                        initializeDeformableBody(xformCache, prim, iStageReaderWriter, stageInProgress);
                }
            }
        }
    }

    mUsdResetOnStop = gSettings->getAsBool(kSettingResetOnStop);
    if (mPhysX)
        mPhysX->disableResetOnStop(true);
}

void FabricManager::pause()
{
    if (mNoticeListener)
    {
        pxr::TfNotice::Revoke(mNoticeListenerKey);
        delete mNoticeListener;
        mNoticeListener = nullptr;
    }

    mDirectGpuHelper.clear();

    const bool enabled = gSettings->getAsBool(kSettingFabricEnabled);
    if (!enabled)
        return;

    if (mPhysXSimulation)
    {
        mWheelVehicleMap.clear();
        mPointInstanceProtos.clear();
        mUpdate = false;

        physx::ISimulationCallback cb;
        cb.transformationWriteFn = nullptr;
        cb.velocityWriteFn = nullptr;
        cb.transformationUpdateFn = nullptr;
        cb.userData = nullptr;
        while(!mPhysXSimulation->checkResults()){}; // wait for last async sim step to complete
        mPhysXSimulation->setSimulationCallback(cb);
        mPhysXSimulation->setSimulationOutputFlags(omni::physx::SimulationOutputType::eTRANSFORMATION,
                                                   0, nullptr, 0);
        mPhysXSimulation->setSimulationOutputFlags(
            omni::physx::SimulationOutputType::eVELOCITY, 0, nullptr, 0);
        mPhysXSimulation->setSimulationOutputFlags(
            omni::physx::SimulationOutputType::ePOINTS, 0, nullptr, 0);
        mPhysXSimulation->setSimulationOutputFlags(
            omni::physx::SimulationOutputType::eRESIDUALS, 0, nullptr, 0);
    }
}


void FabricManager::stop()
{
    pause();  // will also await simulation end

    if (mUsdResetOnStop)
    {
        setInitialTransformations();
    }
    else
    {
        saveToUsd();
    }

    mInitialTransformation.clear();
    mInitialJointStates.clear();

    if (mPhysX)
        mPhysX->disableResetOnStop(false);

    gStage = nullptr;
    mPtrDirty = true;

    mResyncPaths.clear();
}

void FabricManager::setInitialTransformations()
{
    const bool enabled = gSettings->getAsBool(kSettingFabricEnabled);

    CARB_PROFILE_ZONE(0, "FabricManager::resetInitTransformation");
    if (enabled)
    {
        using namespace omni::fabric;
        using namespace omni::physx;
        using namespace physx;
        using ::physx::PxVec3;
        using ::physx::PxQuat;
        using ::physx::PxTransform;

        IStageReaderWriter* iSip = carb::getCachedInterface<IStageReaderWriter>();
        StageReaderWriter stage = iSip->get(mStageId);
        IPhysx* iPhysX = carb::getCachedInterface<IPhysx>();
        CARB_ASSERT(iPhysX);

        const Type typePrimName(BaseDataType::eTag, 1, 0, AttributeRole::ePrimTypeName);
        const Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
        const Type typeAppliedType(BaseDataType::eTag, 1, 0, AttributeRole::eNone);
        const Type typeDouble3(BaseDataType::eDouble, 3, 0, AttributeRole::eNone);
        const Type typeMatrix4d(BaseDataType::eDouble, 16, 0, AttributeRole::eMatrix);
        const Type typeFloat3(BaseDataType::eFloat, 3, 0, AttributeRole::eVector);
        const Type typeQuat(BaseDataType::eFloat, 4, 0, AttributeRole::eQuaternion);
        const Type typeUint64(fabric::BaseDataType::eUInt64);
        const Type typeFloat(fabric::BaseDataType::eFloat);

        const Token tokenVehicleWheelAttachment("PhysxVehicleWheelAttachmentAPI");
        const Token tokenVehicleWheel("PhysxVehicleWheel");
        const Token tokenRigidBody("PhysicsRigidBodyAPI");

        const Token tokenWorldMatrix(gWorldMatrixTokenString);
        const Token tokenLocalMatrix(gLocalMatrixTokenString);

        const Token tokenLinVelocity(UsdPhysicsTokens->physicsVelocity.GetText());
        const Token tokenAngularVelocity(UsdPhysicsTokens->physicsAngularVelocity.GetText());

        PauseChangeTrackingScope changeTrackingPauseScope(mPhysXSimulation);

        omni::fabric::USDHierarchy usdHierarchy(stage.getFabricId());

        // Update point instancers
        {
            for (const auto& instancer : mInitialPointInstancers)
            {
                const omni::fabric::Path instPath = omni::fabric::asInt(instancer.first);
                if (!stage.primExists(instPath))
                {
                    CARB_LOG_WARN("FabricManager::update invalid point instancer: %s.", instPath.getText());
                    continue;
                }
                const size_t count = instancer.second.positions.size();
                if (count != stage.getArrayAttributeSize(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->positions)))
                {
                    stage.setArrayAttributeSize(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->positions), count);
                }
                if (count != stage.getArrayAttributeSize(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->orientations)))
                {
                    stage.setArrayAttributeSize(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->orientations), count);
                }
                gsl::span<pxr::GfVec3f> positions =
                    stage.getArrayAttribute<pxr::GfVec3f>(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->positions));
                gsl::span<pxr::GfQuath> orientations = stage.getArrayAttribute<pxr::GfQuath>(
                    instPath, omni::fabric::asInt(pxr::UsdGeomTokens->orientations));
                if (positions.size() != count || orientations.size() != count)
                {
                    CARB_LOG_WARN("FabricManager::update mismatched array attribute size on point instancer: %s.",
                                   instPath.getText());
                    continue;
                }
                memcpy(positions.data(), instancer.second.positions.data(), sizeof(pxr::GfVec3f) * count);
                memcpy(orientations.data(), instancer.second.orientations.data(), sizeof(pxr::GfQuath) * count);
            }
        }

        // Update vehicles (rigid bodies)
        {
           const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenRigidBody),
                                                                     AttrNameAndType_v2(typeAppliedType, mDynamicBodyToken) };
           const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(typeMatrix4d, tokenWorldMatrix),
                                                                     AttrNameAndType_v2(typeMatrix4d, tokenLocalMatrix),
                                                                     AttrNameAndType_v2(typeFloat3, tokenLinVelocity),
                                                                     AttrNameAndType_v2(typeFloat3, tokenAngularVelocity) };

            PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
            size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<pxr::GfMatrix4d> worldMatrices =
                    stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, tokenWorldMatrix);
                gsl::span<pxr::GfMatrix4d> localMatrices =
                    stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, tokenLocalMatrix);

                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                size_t j = 0;
                for (const omni::fabric::Path& path : paths)
                {
                    TransformationCache::const_iterator fit = mInitialTransformation.find(PathC(path).path);
                    if (fit != mInitialTransformation.end())
                    {
                        pxr::GfVec3d t(double(fit->second.translation.x), double(fit->second.translation.y),
                                       double(fit->second.translation.z));
                        pxr::GfQuatd q(fit->second.orientation.w, { fit->second.orientation.x, fit->second.orientation.y,
                                                                    fit->second.orientation.z });

                        pxr::GfVec3d s(
                            double(fit->second.scale.x), double(fit->second.scale.y), double(fit->second.scale.z));

                        worldMatrices[j] = computeMatrix(t, pxr::GfMatrix3d(q), s);
                        localMatrices[j] =
                            computeLocalMatrix(usdHierarchy, stage, path, worldMatrices[j], mWorldMatrixToken);
                    }
                    j++;
                }
            }
        }

        // Update vehicle wheels
        {
            CARB_PROFILE_ZONE(0, "FabricManager::update - Vehicle Wheels");
           const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenVehicleWheelAttachment),
                                                                       AttrNameAndType_v2(typeDouble3, tokenLocalMatrix),
                                                                       AttrNameAndType_v2(typeDouble3, tokenWorldMatrix)};

            PrimBucketList primBuckets = stage.findPrims(requiredAll);
            size_t bucketCount = primBuckets.bucketCount();

            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<pxr::GfMatrix4d> outWorldMatrices  =
                    stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, tokenWorldMatrix);
                gsl::span<pxr::GfMatrix4d> outLocalMatrices =
                    stage.getAttributeArray<pxr::GfMatrix4d>(primBuckets, i, tokenLocalMatrix);

                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);

                {
                    size_t j = 0;
                    for (const omni::fabric::Path& path : paths)
                    {
                        TransformationCache::const_iterator it = mInitialTransformation.find(PathC(path).path);
                        if (it != mInitialTransformation.end())
                        {
                            pxr::GfVec3d t(double(it->second.translation.x), double(it->second.translation.y),
                                           double(it->second.translation.z));
                            pxr::GfQuatd q(it->second.orientation.w, { it->second.orientation.x, it->second.orientation.y,
                                                                       it->second.orientation.z });

                            pxr::GfVec3d s(
                                double(it->second.scale.x), double(it->second.scale.y),
                                           double(it->second.scale.z));

                            outWorldMatrices[j] = computeMatrix(t, pxr::GfMatrix3d(q), s);
                            outLocalMatrices[j] =
                                computeLocalMatrix(usdHierarchy, stage, path, outWorldMatrices[j], mWorldMatrixToken);
                        }
                        //else
                        //{
                        //    CARB_LOG_ERROR("Wheel transformation was not found in a transformation cache.");
                        //}
                        j++;
                    }
                }
            }

        }

        // Update joint states
        {
            CARB_PROFILE_ZONE(0, "FabricManager::setInitialTransformations - Joint State");
            for(auto& ts : mTokenJointStates)
            {
                const Token tokenJointStateAPI = ts.token;
               const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenJointStateAPI),
                                                                        AttrNameAndType_v2(typeFloat, ts.madPositionToken),
                                                                        AttrNameAndType_v2(typeFloat, ts.madVelocityToken) };

                PrimBucketList primBuckets = stage.findPrims(requiredAll);
                size_t bucketCount = primBuckets.bucketCount();

                for (size_t i = 0; i != bucketCount; i++)
                {
                    gsl::span<float> outPositions =
                        stage.getAttributeArray<float>(primBuckets, i, ts.madPositionToken);
                    gsl::span<float> outVelocities =
                        stage.getAttributeArray<float>(primBuckets, i, ts.madVelocityToken);

                    gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);

                    size_t fcItemIndex = 0;
                    for (const omni::fabric::Path& path : paths)
                    {
                        JointStateCache::const_iterator it = mInitialJointStates.find(PathC(path).path);
                        if (it != mInitialJointStates.end())
                        {
                            const omni::physx::JointStateData& jointStateData = it->second;
                            for(int jointAxis = 0; jointAxis < 6; ++jointAxis)
                            {
                                if(!jointStateData.enabled[jointAxis])
                                    continue;
                                if(jointStateData.fabricTokenC[jointAxis] != omni::fabric::asInt(ts.usdToken).token)
                                    continue;
                                if(jointStateData.physxAxis[jointAxis] != ts.physxAxis)
                                    continue;
                                outPositions[fcItemIndex] = jointStateData.initialPosition[jointAxis];
                                outVelocities[fcItemIndex] = jointStateData.initialVelocity[jointAxis];
                                break;
                            }
                        }
                        fcItemIndex++;
                    }
                }
            }
        }

        mInitialTransformation.clear();
        mInitialJointStates.clear();
        mInitialPointInstancers.clear();

#if !CARB_AARCH64
        mDeformableBodyManagerDeprecated.setInitialTransformation(stage);
        mDeformableSurfaceManagerDeprecated.setInitialTransformation(stage);
        mParticleManagerDeprecated.setInitialTransformation(stage);
        mVolumeDeformableBodyManager.setInitialTransformation(stage);
        mSurfaceDeformableBodyManager.setInitialTransformation(stage);
#endif
    }
}

void FabricManager::saveToUsd()
{
    const bool enabled = gSettings->getAsBool(kSettingFabricEnabled);

    CARB_PROFILE_ZONE(0, "FabricManager::saveToUsd");
    if (enabled && gStage)
    {
        // We need to pause change tracking as omni::fabric will try to sync back
        PauseChangeTrackingScope changeTrackingPauseScope(mPhysXSimulation);
        std::unique_ptr<PXR_NS::SdfChangeBlock> changeBlock;

        using namespace omni::fabric;
        using namespace omni::physx;

        IStageReaderWriter* iSip = carb::getCachedInterface<IStageReaderWriter>();
        StageReaderWriter stage = iSip->get(mStageId);

        const Type typePrimName(BaseDataType::eTag, 1, 0, AttributeRole::ePrimTypeName);
        const Type typeAppliedSchema(BaseDataType::eTag, 1, 0, AttributeRole::eAppliedSchema);
        const Type typeAppliedType(BaseDataType::eTag, 1, 0, AttributeRole::eNone);
        const Type typeDouble3(BaseDataType::eDouble, 3, 0, AttributeRole::eNone);
        const Type typeFloat3(BaseDataType::eFloat, 3, 0, AttributeRole::eVector);
        const Type typeMatrix4d(BaseDataType::eDouble, 16, 0, omni::fabric::AttributeRole::eMatrix);
        const Type typeQuat(BaseDataType::eFloat, 4, 0, AttributeRole::eQuaternion);
        const Type typeUint64(fabric::BaseDataType::eUInt64);
        const Type typeFloat(BaseDataType::eFloat, 1, 0, AttributeRole::eNone);
        const Token tokenRigidBody("PhysicsRigidBodyAPI");

        const Token tokenWorldMatrix("omni:fabric:worldMatrix");

        const Token tokenLinVelocity(UsdPhysicsTokens->physicsVelocity.GetText());
        const Token tokenAngularVelocity(UsdPhysicsTokens->physicsAngularVelocity.GetText());

        UsdGeomXformCache xformCache;

        // Update rigid bodies to USD
        {
           const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenRigidBody),
                                                                     AttrNameAndType_v2(typeAppliedType, mDynamicBodyToken) };
           const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(typeMatrix4d, tokenWorldMatrix),
                                                                     AttrNameAndType_v2(typeFloat3, tokenLinVelocity),
                                                                     AttrNameAndType_v2(typeFloat3, tokenAngularVelocity) };

            PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
            size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i != bucketCount; i++)
            {
                gsl::span<const pxr::GfMatrix4d> worldMatrices  =
                    stage.getAttributeArrayRd<pxr::GfMatrix4d>(primBuckets, i, tokenWorldMatrix);

                gsl::span<const carb::Float3> linVelocities = stage.getAttributeArrayRd<carb::Float3>(primBuckets, i, mLinVelToken);
                gsl::span<const carb::Float3> angVelocities = stage.getAttributeArrayRd<carb::Float3>(primBuckets, i, mAngVelToken);

                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                size_t j = 0;
                for (const omni::fabric::Path& path : paths)
                {
                    const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                    UsdPrim prim = gStage->GetPrimAtPath(primPath);

                    const Float3& linVelocity = linVelocities[j];
                    const Float3& angVelocity = angVelocities[j];

                    const pxr::GfMatrix4d mtxIn = xformCache.GetLocalToWorldTransform(prim);
                    // scale comes from mtxIn, translation and rotation from the world matrix
                    pxr::GfTransform trIn(mtxIn);
                    pxr::GfTransform tr(worldMatrices[j]);
                    trIn.SetRotation(tr.GetRotation());
                    trIn.SetTranslation(tr.GetTranslation());
                    const pxr::GfMatrix4d mtx = trIn.GetMatrix();

                    pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default();
                    const pxr::GfMatrix4d parentToWorldMat = xformCache.GetParentToWorldTransform(prim);
                    const pxr::GfMatrix4d worldToParentMat = parentToWorldMat.GetInverse();

                    omni::usd::UsdUtils::setLocalTransformMatrix(prim, mtx * worldToParentMat, timeCode, false, &changeBlock);

                    TransformationCache::iterator it = mInitialTransformation.find(PathC(path).path);
                    if (it != mInitialTransformation.end())
                    {
                        const pxr::GfVec3d& pos = tr.GetTranslation();
                        const pxr::GfQuatd& ori = tr.GetRotation().GetQuat();
                        const Float3 newInitPos = { float(pos[0]), float(pos[1]), float(pos[2]) };
                        it->second.translation = newInitPos;
                        const Float4 newInitOri = { float(ori.GetImaginary()[0]), float(ori.GetImaginary()[1]), float(ori.GetImaginary()[2]),
                                                    float(ori.GetReal()) };
                        it->second.orientation = newInitOri;
                        const pxr::GfVec3d scl = tr.GetScale();
                        const Float3 scale = { float(scl[0]), float(scl[1]), float(scl[2]) };
                        it->second.scale = scale;
                    }

                    {
                        UsdAttribute velAttr = prim.GetAttribute(UsdPhysicsTokens->physicsVelocity);
                        if (velAttr)
                        {
                            velAttr.Set((const GfVec3f&)linVelocity);
                        }
                    }
                    {
                        UsdAttribute velAttr = prim.GetAttribute(UsdPhysicsTokens->physicsAngularVelocity);
                        if (velAttr)
                        {
                            velAttr.Set((const GfVec3f&)angVelocity);
                        }
                    }

                    j++;
                }
            }
        }

        {
            CARB_PROFILE_ZONE(0, "FabricManager::saveToUsd - Joint State");

            for(auto& ts : mTokenJointStates)
            {
                const Token tokenJointStateAPI = ts.token;
               const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenJointStateAPI),
                                                                        AttrNameAndType_v2(typeFloat, ts.madPositionToken),
                                                                        AttrNameAndType_v2(typeFloat, ts.madVelocityToken) };

                PrimBucketList primBuckets = stage.findPrims(requiredAll);
                size_t bucketCount = primBuckets.bucketCount();

                for (size_t i = 0; i != bucketCount; i++)
                {
                    gsl::span<const float> jointPositions =
                        stage.getAttributeArrayRd<float>(primBuckets, i, ts.madPositionToken);
                    gsl::span<const float> jointVelocities =
                        stage.getAttributeArrayRd<float>(primBuckets, i, ts.madVelocityToken);

                    gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);

                    size_t fcItemIndex = 0;
                    for (const omni::fabric::Path& path : paths)
                    {
                        const pxr::SdfPath primPath = omni::fabric::toSdfPath(path);
                        UsdPrim prim = gStage->GetPrimAtPath(primPath);
                        JointStateCache::iterator it = mInitialJointStates.find(PathC(path).path);
                        if (it != mInitialJointStates.end())
                        {
                            omni::physx::JointStateData& jointStateData = it->second;
                            for(int jointAxis = 0; jointAxis < 6; ++jointAxis)
                            {
                                if(!jointStateData.enabled[jointAxis])
                                    continue;
                                if(jointStateData.fabricTokenC[jointAxis] != omni::fabric::asInt(ts.usdToken).token)
                                    continue;
                                if(jointStateData.physxAxis[jointAxis] != ts.physxAxis)
                                    continue;
                                jointStateData.initialPosition[jointAxis] = jointPositions[fcItemIndex];
                                jointStateData.initialVelocity[jointAxis] = jointVelocities[fcItemIndex];
                                break;
                            }
                        }
                        PhysxSchemaJointStateAPI jointStateAPI = PhysxSchemaJointStateAPI::Get(prim,  ts.usdToken);
                        if(jointStateAPI)
                        {
                            UsdAttribute posAttr = jointStateAPI.GetPositionAttr();
                            if (posAttr)
                            {
                                posAttr.Set(jointPositions[fcItemIndex]);
                            }
                            UsdAttribute velAttr = jointStateAPI.GetVelocityAttr();
                            if (velAttr)
                            {
                                velAttr.Set(jointVelocities[fcItemIndex]);
                            }
                        }
                        fcItemIndex++;
                    }
                }
            }
        }
        {
            const Token tokenResidualReporting("PhysxResidualReportingAPI");
            const omni::fabric::set<AttrNameAndType_v2> requiredAll = { AttrNameAndType_v2(typeAppliedSchema, tokenResidualReporting) };
            const omni::fabric::set<AttrNameAndType_v2> requiredAny = { AttrNameAndType_v2(typeFloat, mResidualRmsPosIterToken),
                AttrNameAndType_v2(typeFloat, mResidualMaxPosIterToken),
                AttrNameAndType_v2(typeFloat, mResidualRmsVelIterToken) ,
                AttrNameAndType_v2(typeFloat, mResidualMaxVelIterToken) };

            const PrimBucketList primBuckets = stage.findPrims(requiredAll, requiredAny);
            const size_t bucketCount = primBuckets.bucketCount();
            for (size_t i = 0; i < bucketCount; i++)
            {
                gsl::span<const float> residualRmsPosIter = stage.getAttributeArrayRd<float>(primBuckets, i, mResidualRmsPosIterToken);
                gsl::span<const float> residualMaxPosIter = stage.getAttributeArrayRd<float>(primBuckets, i, mResidualMaxPosIterToken);
                gsl::span<const float> residualRmsVelIter = stage.getAttributeArrayRd<float>(primBuckets, i, mResidualRmsVelIterToken);
                gsl::span<const float> residualMaxVelIter = stage.getAttributeArrayRd<float>(primBuckets, i, mResidualMaxVelIterToken);

                gsl::span<const omni::fabric::Path> paths = stage.getPathArray(primBuckets, i);
                size_t j = 0;
                for (const omni::fabric::Path path : paths)
                {
                    const pxr::SdfPath usdPath = omni::fabric::toSdfPath(path);                    
                    PhysxSchemaPhysxResidualReportingAPI reportAPI = PhysxSchemaPhysxResidualReportingAPI::Get(gStage, usdPath);
                    if (reportAPI)
                    {
                        reportAPI.CreatePhysxResidualReportingMaxResidualPositionIterationAttr().Set(residualMaxPosIter[j]);
                        reportAPI.CreatePhysxResidualReportingRmsResidualPositionIterationAttr().Set(residualRmsPosIter[j]);

                        reportAPI.CreatePhysxResidualReportingMaxResidualVelocityIterationAttr().Set(residualMaxVelIter[j]);
                        reportAPI.CreatePhysxResidualReportingRmsResidualVelocityIterationAttr().Set(residualRmsVelIter[j]);
                    }
                    j++;
                }
            }
        }

#if !CARB_AARCH64
        mDeformableBodyManagerDeprecated.saveToUsd(stage, gStage);
        mDeformableSurfaceManagerDeprecated.saveToUsd(stage, gStage);
        mParticleManagerDeprecated.saveToUsd(stage, gStage);
        mVolumeDeformableBodyManager.saveToUsd(stage, gStage);
        mSurfaceDeformableBodyManager.saveToUsd(stage, gStage);
#endif
    }
}

bool FabricManager::getInitialTransformation(const omni::fabric::PathC& path,
                                             carb::Float3& translation,
                                             carb::Float4& orientation,
                                             carb::Float3& scale)
{
    if (path != omni::fabric::kUninitializedPath)
    {
        TransformationCache::const_iterator fit = mInitialTransformation.find(path.path);
        if (fit != mInitialTransformation.end())
        {
            translation = fit->second.translation;
            orientation = fit->second.orientation;
            scale = fit->second.scale;
            return true;
        }
    }

    return false;
}


void FabricManager::initializePointInstancer(pxr::UsdStageWeakPtr usdStage,
                                             const pxr::UsdGeomPointInstancer instancer,
                                             pxr::UsdGeomXformCache& xfCache,
                                             omni::fabric::StageReaderWriter& srw)
{
    const pxr::UsdPrim prim = instancer.GetPrim();
    const auto& instancerPath = prim.GetPrimPath();

    pxr::SdfPathVector protos;
    instancer.GetPrototypesRel().GetTargets(&protos);

    const pxr::SdfPath primPath = prim.GetPrimPath();
    omni::fabric::PathC fcPath = omni::fabric::asInt(primPath);
    auto fcProtos =
        srw.getArrayAttributeRd<omni::fabric::PathC>(fcPath, omni::fabric::asInt(pxr::UsdGeomTokens->prototypes));

    if (fcProtos.size() != protos.size())
    {
        CARB_LOG_WARN("FabricManager::initializePointInstancer mismatched prototypes on point instancer: %s.", primPath.GetText());
        return;
    }

    const auto instancerMatrixInv = xfCache.GetLocalToWorldTransform(prim).RemoveScaleShear().GetInverse();
    struct ProtoData
    {
        pxr::GfMatrix4d transformInv;
        bool isRigidBody;
    };
    const size_t protosCount = protos.size();
    std::vector<ProtoData> protosData(protosCount);

    size_t rbCount = 0;
    for (size_t pi = 0; pi < protosCount; ++pi)
    {
        const pxr::UsdPrim protoPrim = usdStage->GetPrimAtPath(protos[pi]);
        auto& protoData = protosData[pi];
        if (!protoPrim.HasAPI<UsdPhysicsRigidBodyAPI>())
        {
            protoData.isRigidBody = false;
            continue;
        }
        ++rbCount;
        protoData.isRigidBody = true;
        auto protoTransform = xfCache.GetLocalToWorldTransform(protoPrim).RemoveScaleShear();
        protoData.transformInv = (protoTransform * instancerMatrixInv).GetInverse();
    }
    if (rbCount == 0)
    {
        return;
    }

    pxr::VtArray<int> protoIndices;
    instancer.GetProtoIndicesAttr().Get(&protoIndices);
    const size_t indicesCount = protoIndices.size();

    pxr::SdfInt64ListOp inactiveIds;
    prim.GetMetadata(UsdGeomTokens->inactiveIds, &inactiveIds);
    const std::vector<int64_t>& inactiveItems = inactiveIds.GetExplicitItems();
    if (indicesCount == inactiveItems.size())
    {
        return;
    }
    std::unordered_set<int64_t> inactiveIdsSet{ inactiveItems.begin(), inactiveItems.end() };

    // Cache initial attributes    
    if (mInitialPointInstancers.count(instancerPath) == 0)
    {
        auto& initialData = mInitialPointInstancers[instancerPath];
        instancer.GetPositionsAttr().Get(&initialData.positions);
        instancer.GetOrientationsAttr().Get(&initialData.orientations);
        if (indicesCount != initialData.positions.size() || indicesCount != initialData.orientations.size())
        {
            mInitialPointInstancers.erase(instancerPath);
            return;
        }
    }

    for (size_t i = 0; i < indicesCount; ++i)
    {
        if (inactiveIdsSet.count((int64_t)i))
        {
            continue;
        }
        const size_t protoIdx = protoIndices[i];
        const auto& protoData = protosData[protoIdx];
        if (!protoData.isRigidBody)
        {
            continue;
        }
        const auto fcProtoPath = fcProtos[protoIdx]; 
        auto iter = mPointInstanceProtos.find(fcProtoPath.path);
        if (iter == mPointInstanceProtos.end())
        {
            iter = mPointInstanceProtos.insert({ fcProtoPath.path, {} }).first;
            iter->second.instancerPath = prim.GetPrimPath();
            iter->second.usdProtoPath = protos[protoIdx];
            iter->second.protoTransfromInverse = protoData.transformInv;
            iter->second.indices.reserve(indicesCount);
        }
        iter->second.indices.push_back(i);
    }
}

void FabricManager::parsePointInstancers(pxr::UsdStageWeakPtr usdStage,
                                         usdrt::UsdStageRefPtr usdrtStage,
                                         pxr::UsdGeomXformCache& xfCache,
                                         omni::fabric::IStageReaderWriter* iStageReaderWriter,
                                         omni::fabric::StageReaderWriterId stageInProgress)
{
    const auto instancerPaths = usdrtStage->GetPrimsWithTypeName(usdrt::TfToken("PointInstancer"));
    StageReaderWriter srw{stageInProgress};
    for (auto& rtPath : instancerPaths)
    {
        const omni::fabric::PathC fcPath{rtPath};
        const pxr::UsdPrim prim = usdStage->GetPrimAtPath(omni::fabric::toSdfPath(fcPath));
        if (prim.IsA<pxr::UsdGeomPointInstancer>())
        {
            initializePointInstancer(usdStage, pxr::UsdGeomPointInstancer{ prim }, xfCache, srw);
        }
    }
}

bool FabricManager::updatePointInstancer(const omni::fabric::Path primPath,
                                         omni::fabric::StageReaderWriter& stage,
                                         pxr::UsdGeomXformCache& xformCache)
{
    auto iterProto = mPointInstanceProtos.find(primPath.asPathC().path);
    if (iterProto == mPointInstanceProtos.end())
    {
        return false;
    }
    const auto& protoData = iterProto->second;
    const uint32_t indicesCount = (uint32_t)protoData.indices.size();
    if (indicesCount == 0)
    {
        return false;
    }
    gsl::span<uint64_t> physxPtrs;
    if (mPtrDirty)
    {
        stage.setArrayAttributeSize(primPath, mPhysXPtrInstancedToken, indicesCount);
        physxPtrs = stage.getArrayAttribute<uint64_t>(primPath, mPhysXPtrInstancedToken);
        if (indicesCount != physxPtrs.size())
        {
            return false;
        }
        const uint32_t instancedCount = gPhysXPrivate->getPhysXPtrInstanced(
            iterProto->second.usdProtoPath, (void**)physxPtrs.data(), indicesCount, omni::physx::ePTActor);
        if (indicesCount != instancedCount)
        {
            return false;
        }
    }
    else
    {
        physxPtrs = stage.getArrayAttribute<uint64_t>(primPath, mPhysXPtrInstancedToken);
        if (indicesCount != physxPtrs.size())
        {
            return false;
        }
    }
    const omni::fabric::Token positionsToken = omni::fabric::asInt(pxr::UsdGeomTokens->positions);
    const omni::fabric::Token orientationsToken = omni::fabric::asInt(pxr::UsdGeomTokens->orientations);
    const size_t maxIdx = protoData.indices.back();
    const omni::fabric::Path instPath = omni::fabric::asInt(protoData.instancerPath);
    gsl::span<GfVec3f> positions =
        stage.getArrayAttribute<GfVec3f>(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->positions));
    gsl::span<GfQuath> orientations =
        stage.getArrayAttribute<GfQuath>(instPath, omni::fabric::asInt(pxr::UsdGeomTokens->orientations));
    if (maxIdx >= positions.size() || maxIdx >= orientations.size())
    {
        return false;
    }
    const pxr::UsdPrim instancerPrim = gStage->GetPrimAtPath(protoData.instancerPath);
    auto instancerMatrixInv = xformCache.GetLocalToWorldTransform(instancerPrim).GetInverse();
    for (size_t i = 0; i < indicesCount; ++i)
    {
        const ::physx::PxBase* basePtr = reinterpret_cast<const ::physx::PxBase*>(physxPtrs[i]);
        if (!basePtr)
        {
            continue;
        }
        const ::physx::PxRigidBody* rigidBody = basePtr->is<::physx::PxRigidBody>();
        if (!rigidBody)
        {
            continue;
        }
        const ::physx::PxTransform rigidBodyWorldTransformation = rigidBody->getGlobalPose();
        const pxr::GfVec3d translation = omni::physx::toVec3d(rigidBodyWorldTransformation.p);
        const pxr::GfQuatd rotation = omni::physx::toQuatd(omni::physx::toDouble4(rigidBodyWorldTransformation.q));
        const auto worldMatrix = computeMatrix(translation, pxr::GfMatrix3d(rotation), pxr::GfVec3d(1.0));
        const auto instMatrix = protoData.protoTransfromInverse * worldMatrix * instancerMatrixInv;
        const size_t instIdx = protoData.indices[i];
        positions[instIdx] = pxr::GfVec3f{ instMatrix.ExtractTranslation() };
        orientations[instIdx] = pxr::GfQuath{ instMatrix.ExtractRotation().GetQuat() };
    }
    return true;
}

}
}
