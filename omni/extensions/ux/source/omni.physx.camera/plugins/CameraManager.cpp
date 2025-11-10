// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <math.h>

#include "CameraManager.h"

#include "CameraController.h"
#include "CameraFollowLook.h"
#include "CameraFollowVelocity.h"
#include "CameraDrone.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <omni/kit/ViewportTypes.h>

#include <omni/physx/IPhysx.h>

#include <common/foundation/Allocator.h>

extern pxr::UsdStageRefPtr gStage;

extern omni::physx::IPhysx* gPhysXInterface;


using namespace pxr;
using namespace carb;
using namespace ::physx;


namespace omni
{
namespace physx
{

CameraManager::CameraManager()
{
    mSimulationStarted = false;
    mSetupInputs = true;

    mUsdChangeListenerCount = 0;

    mHasPhysicsStepEventSubscription = false;

    mActiveCameraPath = SdfPath::EmptyPath();

    // Initialize the Python interface that gets the active camera.
    mScripting = carb::getCachedInterface<carb::scripting::IScripting>();

    if (mScripting)
    {
        mScriptingContext = mScripting->createContext();
        const char* cmd = "from omni.kit.viewport.utility import get_viewport_window_camera_string\n";

        mScripting->executeString(mScriptingContext, cmd, 0, nullptr);

        if (mScripting->getLastExecutionError(mScriptingContext).code != carb::scripting::ExecutionErrorCode::eOk)
        {
            destroyScriptingContext();
        }
    }
}

CameraManager::~CameraManager()
{
    destroyScriptingContext();
    release();
}

void CameraManager::destroyScriptingContext()
{
    if(mScripting)
    {
        mScripting->destroyContext(mScriptingContext);
        carb::getFramework()->releaseInterface<carb::scripting::IScripting>(mScripting);
        mScriptingContext = nullptr;
        mScripting = nullptr;
    }
}

void CameraManager::release()
{
    mPrimHierarchyStorage.clear();

    // Delete Camera Controllers.
    CameraControllerMap::iterator cameraIt = mCameraControllerMap.begin();

    while (cameraIt != mCameraControllerMap.end())
    {
        CameraController::release(*cameraIt->second);
        cameraIt++;

        CARB_ASSERT(mUsdChangeListenerCount > 0);
        mUsdChangeListenerCount--;
    }

    mCameraControllerMap.clear();

    // Unsubscribe from Physics step events.
    if (mHasPhysicsStepEventSubscription)
    {
        gPhysXInterface->unsubscribePhysicsOnStepEvents(mPhysicsStepEventId);
        gPhysXInterface->unsubscribePhysicsSimulationEvents(mPhysicsCompletionEventId);

        mHasPhysicsStepEventSubscription = false;
    }

    CARB_ASSERT(mUsdChangeListenerCount == 0);
    // to ensure the count is adjusted properly when starting to allow creation/deletion
    // while playing
    mUsdChangeListenerCount = 0;  // don't want dangling counts to carry over to next simulation session

    mSimulationStarted = false;
}

bool CameraManager::addFollowLookCamera(const pxr::SdfPath& subjectPath, const pxr::SdfPath& cameraPath)
{
    if (gStage->GetPrimAtPath(cameraPath))
    {
        CARB_LOG_ERROR("PhysX Camera: addFollowLookCamera: camera path \"%s\" is already in use.\n", cameraPath.GetText());
        return false;
    }

    pxr::UsdGeomCamera usdCamera = pxr::UsdGeomCamera::Define(gStage, cameraPath);
    CARB_ASSERT(usdCamera);

    UsdPrim cameraPrim = usdCamera.GetPrim();
    PhysxSchemaPhysxCameraFollowLookAPI followLookCameraApi(cameraPrim);
    PhysxSchemaPhysxCameraFollowAPI followCameraApi(followLookCameraApi);
    PhysxSchemaPhysxCameraAPI cameraApi(followLookCameraApi);
    cameraApi.GetPhysxCameraSubjectRel().ClearTargets(true);
    cameraApi.GetPhysxCameraSubjectRel().AddTarget(subjectPath);

    const float lengthScale = 1.0f / static_cast<float>(UsdGeomGetStageMetersPerUnit(gStage));

    pxr::GfVec3f positionOffset = { 0.0f * lengthScale, 0.0f * lengthScale, 0.0f * lengthScale };
    pxr::GfVec3f cameraPositionTC;
    pxr::GfVec3f lookPositionTC;

    TfToken usdUpAxis = pxr::UsdGeomGetStageUpAxis(gStage);

    if (usdUpAxis == TfToken("Y"))
    {
        cameraPositionTC = { 0.5f, 0.1f, 0.5f };
        lookPositionTC = { 0.2f, 0.5f, 0.2f };
    }
    else
    {
        cameraPositionTC = { 0.5f, 0.5f, 0.1f };
        lookPositionTC = { 0.2f, 0.2f, 0.5f };
    }

    followCameraApi.CreateYawAngleAttr(VtValue(20.0f));
    followCameraApi.CreatePitchAngleAttr(VtValue(15.0f));
    followCameraApi.CreatePitchAngleTimeConstantAttr(VtValue(0.2f));
    followCameraApi.CreateSlowSpeedPitchAngleScaleAttr(VtValue(0.5f));
    followCameraApi.CreateSlowPitchAngleSpeedAttr(VtValue(1000.0f));
    followLookCameraApi.CreateDownHillGroundAngleAttr(VtValue(-45.0f));
    followLookCameraApi.CreateDownHillGroundPitchAttr(VtValue(10.0f));
    followLookCameraApi.CreateUpHillGroundAngleAttr(VtValue(45.0f));
    followLookCameraApi.CreateUpHillGroundPitchAttr(VtValue(-10.0f));
    followCameraApi.CreateVelocityNormalMinSpeedAttr(VtValue(600.0f));
    followLookCameraApi.CreateVelocityBlendTimeConstantAttr(VtValue(0.1f));
    followCameraApi.CreateFollowMinSpeedAttr(VtValue(3.0f * lengthScale));
    followCameraApi.CreateFollowMinDistanceAttr(VtValue(15.0f * lengthScale));
    followCameraApi.CreateFollowMaxSpeedAttr(VtValue(30.0f * lengthScale));
    followCameraApi.CreateFollowMaxDistanceAttr(VtValue(10.0f * lengthScale));
    followLookCameraApi.CreateFollowReverseSpeedAttr(VtValue(15.0f * lengthScale));
    followLookCameraApi.CreateFollowReverseDistanceAttr(VtValue(30.0f * lengthScale));
    followCameraApi.CreateYawRateTimeConstantAttr(VtValue(0.2f));
    followCameraApi.CreateFollowTurnRateGainAttr(VtValue(0.2f));
    followCameraApi.CreateCameraPositionTimeConstantAttr(VtValue(cameraPositionTC));
    followCameraApi.CreatePositionOffsetAttr(VtValue(positionOffset));
    followCameraApi.CreateLookAheadMinSpeedAttr(VtValue(0.0f));
    followCameraApi.CreateLookAheadMinDistanceAttr(VtValue(0.0f));
    followCameraApi.CreateLookAheadMaxSpeedAttr(VtValue(20.0f * lengthScale));
    followCameraApi.CreateLookAheadMaxDistanceAttr(VtValue(5.0f * lengthScale));
    followCameraApi.CreateLookAheadTurnRateGainAttr(VtValue(0.2f));
    followCameraApi.CreateLookPositionHeightAttr(VtValue(0.5f * lengthScale));
    followCameraApi.CreateLookPositionTimeConstantAttr(VtValue(lookPositionTC));

    followLookCameraApi.Apply(cameraPrim);

    return true;
}

bool CameraManager::addFollowVelocityCamera(const pxr::SdfPath& subjectPath, const pxr::SdfPath& cameraPath)
{
    if (gStage->GetPrimAtPath(cameraPath))
    {
        CARB_LOG_ERROR("PhysX Camera: addVelocityCamera: camera path \"%s\" is already in use.\n", cameraPath.GetText());
        return false;
    }

    pxr::UsdGeomCamera usdCamera = pxr::UsdGeomCamera::Define(gStage, cameraPath);
    CARB_ASSERT(usdCamera);

    UsdPrim cameraPrim = usdCamera.GetPrim();
    PhysxSchemaPhysxCameraFollowVelocityAPI followVelocityCameraApi(cameraPrim);
    PhysxSchemaPhysxCameraFollowAPI followCameraApi(followVelocityCameraApi);
    PhysxSchemaPhysxCameraAPI cameraApi(followVelocityCameraApi);
    cameraApi.GetPhysxCameraSubjectRel().ClearTargets(true);
    cameraApi.GetPhysxCameraSubjectRel().AddTarget(subjectPath);

    const float lengthScale = 1.0f / static_cast<float>(UsdGeomGetStageMetersPerUnit(gStage));

    pxr::GfVec3f positionOffset = { 0.0f * lengthScale, 0.0f * lengthScale, 0.0f * lengthScale };
    pxr::GfVec3f cameraPositionTC = { 2.0f, 2.0f, 2.0f };
    pxr::GfVec3f lookPositionTC = { 0.1f, 0.1f, 0.1f };

    followCameraApi.CreateYawAngleAttr(VtValue(20.0f));
    followCameraApi.CreatePitchAngleAttr(VtValue(15.0f));
    followCameraApi.CreatePitchAngleTimeConstantAttr(VtValue(0.2f));
    followCameraApi.CreateSlowSpeedPitchAngleScaleAttr(VtValue(0.5f));
    followCameraApi.CreateSlowPitchAngleSpeedAttr(VtValue(1000.0f));
    followCameraApi.CreateVelocityNormalMinSpeedAttr(VtValue(600.0f));
    followCameraApi.CreateFollowMinSpeedAttr(VtValue(3.0f * lengthScale));
    followCameraApi.CreateFollowMinDistanceAttr(VtValue(15.0f * lengthScale));
    followCameraApi.CreateFollowMaxSpeedAttr(VtValue(30.0f * lengthScale));
    followCameraApi.CreateFollowMaxDistanceAttr(VtValue(10.0f * lengthScale));
    followCameraApi.CreateYawRateTimeConstantAttr(VtValue(0.0f));
    followCameraApi.CreateFollowTurnRateGainAttr(VtValue(0.0f));
    followCameraApi.CreateCameraPositionTimeConstantAttr(VtValue(cameraPositionTC));
    followCameraApi.CreatePositionOffsetAttr(VtValue(positionOffset));
    followCameraApi.CreateLookAheadMinSpeedAttr(VtValue(0.0f));
    followCameraApi.CreateLookAheadMinDistanceAttr(VtValue(0.0f));
    followCameraApi.CreateLookAheadMaxSpeedAttr(VtValue(20.0f * lengthScale));
    followCameraApi.CreateLookAheadMaxDistanceAttr(VtValue(0.0f * lengthScale));
    followCameraApi.CreateLookAheadTurnRateGainAttr(VtValue(0.2f));
    followCameraApi.CreateLookPositionHeightAttr(VtValue(0.0f));
    followCameraApi.CreateLookPositionTimeConstantAttr(VtValue(lookPositionTC));

    followVelocityCameraApi.Apply(cameraPrim);

    return true;
}

bool CameraManager::addDroneCamera(const pxr::SdfPath& subjectPath, const pxr::SdfPath& cameraPath)
{
    if (gStage->GetPrimAtPath(cameraPath))
    {
        CARB_LOG_ERROR("PhysX Camera: addDroneCamera: camera path \"%s\" is already in use.\n", cameraPath.GetText());
        return false;
    }

    pxr::UsdGeomCamera usdCamera = pxr::UsdGeomCamera::Define(gStage, cameraPath);
    CARB_ASSERT(usdCamera);

    UsdPrim cameraPrim = usdCamera.GetPrim();
    PhysxSchemaPhysxCameraDroneAPI droneCameraApi(cameraPrim);
    PhysxSchemaPhysxCameraAPI cameraApi(droneCameraApi);
    cameraApi.GetPhysxCameraSubjectRel().ClearTargets(true);
    cameraApi.GetPhysxCameraSubjectRel().AddTarget(subjectPath);

    const float lengthScale = 1.0f / static_cast<float>(UsdGeomGetStageMetersPerUnit(gStage));

    pxr::GfVec3f positionOffset = { 0.0f * lengthScale, 0.0f * lengthScale, 0.0f * lengthScale };

    droneCameraApi.CreatePositionOffsetAttr(VtValue(positionOffset));

    droneCameraApi.CreateFollowHeightAttr(VtValue(15.0f * lengthScale));
    droneCameraApi.CreateFollowDistanceAttr(VtValue(30.0f * lengthScale));
    droneCameraApi.CreateMaxDistanceAttr(VtValue(100.0f * lengthScale));
    droneCameraApi.CreateMaxSpeedAttr(VtValue(20.0f * lengthScale));
    droneCameraApi.CreateHorizontalVelocityGainAttr(VtValue(1.0f));
    droneCameraApi.CreateVerticalVelocityGainAttr(VtValue(1.0f));
    droneCameraApi.CreateFeedForwardVelocityGainAttr(VtValue(0.1f));
    droneCameraApi.CreateVelocityFilterTimeConstantAttr(VtValue(1.0f));
    droneCameraApi.CreateRotationFilterTimeConstantAttr(VtValue(0.2f));

    droneCameraApi.Apply(cameraPrim);

    return true;
}

CameraController* CameraManager::getCamera(const pxr::UsdPrim& prim)
{
    const pxr::SdfPath& path = prim.GetPath();

    CameraControllerMap::iterator it = mCameraControllerMap.find(path);

    if (it != mCameraControllerMap.end())
    {
        return it->second;
    }

    return nullptr;
}

void CameraManager::cameraDirty(CameraController* cameraController)
{
    if (cameraController)
    {
        cameraController->setDirty(CameraController::DirtyFlag::eUSD);
    }
}

// The camera must be updated immediated after the physics time step in order to remain in sync and animate smoothly.
void onPhysicsStepEventFn(float elapsedTime, void* userData)
{
    CARB_PROFILE_ZONE(0, "CameraManager::onPhysicsStepEvent");
    CameraManager* cameraManager = static_cast<CameraManager*>(userData);

    if (cameraManager)
    {
        cameraManager->stepUpdate(elapsedTime, CameraManager::PhysicsUpdate::eUPDATE_PHYSICS);
    }
}

// The camera USD data must be updated once the simulation has completed to avoid asynchronous data clashes.
void onPhysicsCompletionEventFn(SimulationStatusEvent eventStatus, void* userData)
{
    CARB_PROFILE_ZONE(0, "CameraManager::onPhysicsCompletionEventFn");
    if (eventStatus == SimulationStatusEvent::eSimulationComplete)
    {
        CameraManager* cameraManager = static_cast<CameraManager*>(userData);

        if (cameraManager)
        {
            cameraManager->postUpdate(CameraManager::PhysicsUpdate::eUPDATE_PHYSICS);
        }
    }
}

void CameraManager::onResume()
{
    if (!mSimulationStarted)
    {
        createControllers();
    }
}

void CameraManager::onStop()
{
    release();
}

void CameraManager::createControllers()
{
    if (gStage && gPhysXInterface)
    {
        mPrimHierarchyStorage.init(gStage);

        // Find all of the camera, add a controller for them if one does not already exist.
        pxr::UsdPrimRange range = gStage->Traverse();

        pxr::SdfPathVector cameraPaths;
        cameraPaths.reserve(10);

        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const pxr::UsdPrim& prim = *iter;

            if (!prim)
                continue;

            const pxr::SdfPath& path = prim.GetPath();

            if (prim.HasAPI<PhysxSchemaPhysxCameraAPI>())
            {
                cameraPaths.push_back(path);
            }
        }

        // Create a camera controller for each camera.
        for (const pxr::SdfPath& path : cameraPaths)
        {
            addCameraController(path);
        }

        // Register for a physics step event which is used to update the cameras at the same rate as the subjects to avoid stuttering.
        if (mHasPhysicsStepEventSubscription == false)
        {
            mPhysicsStepEventId = gPhysXInterface->subscribePhysicsOnStepEvents(false, 0, onPhysicsStepEventFn, this);
            mPhysicsCompletionEventId = gPhysXInterface->subscribePhysicsSimulationEvents(onPhysicsCompletionEventFn, this);
            mHasPhysicsStepEventSubscription = true;
        }
    }

    mSimulationStarted = true;
}

void CameraManager::addCameraController(const pxr::SdfPath& cameraPath)
{
    const pxr::UsdPrim& prim = gStage->GetPrimAtPath(cameraPath);
    PhysxSchemaPhysxCameraAPI subjectCamera(prim);

    SdfPathVector paths;
    subjectCamera.GetPhysxCameraSubjectRel().GetTargets(&paths);

    if (paths.size() == 1)
    {
        const pxr::UsdPrim& subjectPrim = gStage->GetPrimAtPath(paths[0]);

        if (!subjectPrim)
        {
            CARB_LOG_ERROR("PhysX Camera: addCameraController: subject path \"%s\" is invalid.\n", paths[0].GetText());
            return;
        }

        CameraController* cameraController = nullptr;

        if (prim.HasAPI<PhysxSchemaPhysxCameraFollowLookAPI>())
        {
            CameraFollowLook* lookFollowCamera = CameraFollowLook::create(prim, subjectPrim);
            if (lookFollowCamera)
            {
                mPrimHierarchyStorage.addPrim(cameraPath);
                mCameraControllerMap[cameraPath] = lookFollowCamera;
                mCameraMap[paths[0]].push_back(cameraPath);
                cameraDirty(lookFollowCamera);
                mUsdChangeListenerCount++;

                cameraController = lookFollowCamera;
            }

        }
        else if (prim.HasAPI<PhysxSchemaPhysxCameraFollowVelocityAPI>())
        {
            CameraFollowVelocity* velocityFollowCamera = CameraFollowVelocity::create(prim, subjectPrim);
            if (velocityFollowCamera)
            {
                mPrimHierarchyStorage.addPrim(cameraPath);
                mCameraControllerMap[cameraPath] = velocityFollowCamera;
                mCameraMap[paths[0]].push_back(cameraPath);
                cameraDirty(velocityFollowCamera);
                mUsdChangeListenerCount++;

                cameraController = velocityFollowCamera;
            }

        }
        else if (prim.HasAPI<PhysxSchemaPhysxCameraDroneAPI>())
        {
            CameraDrone* droneCamera = CameraDrone::create(prim, subjectPrim);
            if (droneCamera)
            {
                mPrimHierarchyStorage.addPrim(cameraPath);
                mCameraControllerMap[cameraPath] = droneCamera;
                mCameraMap[paths[0]].push_back(cameraPath);
                cameraDirty(droneCamera);
                mUsdChangeListenerCount++;

                cameraController = droneCamera;
            }
        }

        if (cameraController)
        {
            cameraController->updateAxes();
            cameraController->preUpdate();
            cameraController->stepUpdate(0.0f);
            cameraController->postUpdate();
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Camera: \"%s\" needs to have exactly one \"subject\" relationship defined.\n",
            prim.GetName().GetText());
    }
}

void CameraManager::removeCameraController(const pxr::SdfPath& cameraPath, bool removeReferences)
{
    CameraControllerMap::iterator cameraIter = mCameraControllerMap.find(cameraPath);

    if (cameraIter != mCameraControllerMap.end())
    {
        if (removeReferences)
        {
            pxr::SdfPath subjectPath = cameraIter->second->getSubjectPath();
            CameraMap::iterator subjectCameraIter = mCameraMap.find(subjectPath);
            subjectCameraIter->second.remove(cameraPath);
        }

        CameraController::release(*cameraIter->second);

        CARB_ASSERT(mUsdChangeListenerCount > 0);
        mUsdChangeListenerCount--;

        mCameraControllerMap.erase(cameraIter);
    }
}

bool CameraManager::addPrimInternal(const pxr::UsdPrim& prim)
{
    if (!prim)
        return true;

    if (prim.HasAPI<PhysxSchemaPhysxCameraAPI>())
    {
        addCameraController(prim.GetPath());
        return true;
    }

    return false;
}

void CameraManager::addPrim(const pxr::UsdPrim& prim)
{
    if (addPrimInternal(prim))
        return;
    else
    {
        // a whole prim tree might get referenced into the stage. Need to check if
        // any child is of interest

        UsdPrimRange range(prim);
        if (!range.empty())
        {
            pxr::UsdPrimRange::const_iterator iter = range.begin();
            ++iter;  // the prim itself is part of the range
            for (; iter != range.end(); ++iter)
            {
                const pxr::UsdPrim& p = *iter;                
                if (addPrimInternal(p))
                {
                    // for now, the prims we are interested in are not expected to be nested
                    iter.PruneChildren();
                }
            }
        }
    }
}

void CameraManager::removePrim(const pxr::SdfPath& primPath)
{
    PrimHierarchyStorage::Iterator iterator(mPrimHierarchyStorage, primPath);
    for (size_t i = iterator.getDescendentsPaths().size(); i--;)
    {
        const pxr::SdfPath& primCPath = iterator.getDescendentsPaths()[i];
        removeCameraController(primCPath);
    }
    mPrimHierarchyStorage.removeIteration(iterator);
}

void CameraManager::setActiveCameraPath(pxr::SdfPath activeCameraPath)
{
    mActiveCameraPath = activeCameraPath;
}

pxr::SdfPath CameraManager::getActiveCameraPath()
{
    if (mScripting == nullptr ||
        mScriptingContext == nullptr)
    {
        return pxr::SdfPath();
    }

    carb::scripting::Object* returnObject = mScripting->createObject();
    CARB_ASSERT(returnObject != nullptr);

    mScripting->executeFunction(mScriptingContext, "get_viewport_window_camera_string", returnObject, 0);

    if (mScripting->getLastExecutionError(mScriptingContext).code != carb::scripting::ExecutionErrorCode::eOk)
    {
        mScripting->destroyObject(returnObject);
        return pxr::SdfPath();
    }

    if (mScripting->isObjectNone(returnObject))
    {
        mScripting->destroyObject(returnObject);
        return pxr::SdfPath();
    }

    mCameraPath = mScripting->getObjectAsString(returnObject);
    mScripting->destroyObject(returnObject);

    return pxr::SdfPath(mCameraPath);
}

// Pre physics
void CameraManager::preUpdate()
{
    if (!gStage)
    {
        gStage = omni::usd::UsdContext::getContext()->getStage();
    }

    // Update the active camera, so track when it changes.
    pxr::SdfPath activeCameraPath = getActiveCameraPath();

    if (activeCameraPath.IsEmpty() == false)
    {
        if (activeCameraPath != mActiveCameraPath)
        {
            setActiveCameraPath(activeCameraPath);

            if (mCameraControllerMap.find(activeCameraPath) != mCameraControllerMap.end())
            {
                mCameraControllerMap[mActiveCameraPath]->initialize();
            }
        }

        // Update the active camera and any cameras that should always be updated.
        CameraControllerMap::iterator cameraIt;

        for (cameraIt = mCameraControllerMap.begin(); cameraIt != mCameraControllerMap.end(); cameraIt++)
        {
            cameraIt->second->preUpdate();
        }
    }
}

// During physics
void CameraManager::stepUpdate(float timeStep, PhysicsUpdate physicsUpdate)
{
    if (mActiveCameraPath.IsEmpty() == false)
    {
        CameraControllerMap::iterator cameraIt;

        for (cameraIt = mCameraControllerMap.begin(); cameraIt != mCameraControllerMap.end(); cameraIt++)
        {
            bool update = (cameraIt->second->getAlwaysUpdate()) || (cameraIt->first == mActiveCameraPath);
            bool isRigidBody = cameraIt->second->isRigidBody();

            if (update &&
                ((physicsUpdate == PhysicsUpdate::eUPDATE_BOTH) ||
                (physicsUpdate == PhysicsUpdate::eUPDATE_PHYSICS && isRigidBody) ||
                    (physicsUpdate == PhysicsUpdate::eUPDATE_ANIMATION && !isRigidBody)))
            {
                cameraIt->second->stepUpdate(timeStep);
            }
        }
    }
}

// Post physics
void CameraManager::postUpdate(PhysicsUpdate physicsUpdate)
{
    if (mActiveCameraPath.IsEmpty() == false)
    {
        CARB_PROFILE_ZONE(0, "PhysXVehicleCameraControllerUpdate");
        CameraControllerMap::iterator cameraIt;

        for (cameraIt = mCameraControllerMap.begin(); cameraIt != mCameraControllerMap.end(); cameraIt++)
        {
            bool update = (cameraIt->second->getAlwaysUpdate()) || (cameraIt->first == mActiveCameraPath);
            bool isRigidBody = cameraIt->second->isRigidBody();

            if (update &&
                ((physicsUpdate == PhysicsUpdate::eUPDATE_BOTH) ||
                 (physicsUpdate == PhysicsUpdate::eUPDATE_PHYSICS && isRigidBody) ||
                 (physicsUpdate == PhysicsUpdate::eUPDATE_ANIMATION && !isRigidBody)))
            {
                cameraIt->second->postUpdate();
            }
        }
    }
}

void CameraManager::onUsdObjectChange(const pxr::SdfPath& path, const pxr::UsdTimeCode& timeCode)
{
    CARB_ASSERT(mUsdChangeListenerCount);  // else it should not be called

    if (path.IsPropertyPath())
    {
        TfToken const& attrName = path.GetNameToken();

        if (!UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrName) &&
            attrName != pxr::UsdPhysicsTokens.Get()->physicsVelocity &&
            attrName != pxr::UsdPhysicsTokens.Get()->physicsAngularVelocity)
        {
            // note: using UsdGeomXformable::IsTransformationAffectedByAttrNamed() is an optimization
            //       to filter out notification for transformation changes early.

            // note: fetching the prim from the stage is not that cheap. Querying the prim
            //       by schema API type (prim.HasAPI()) is really expensive. prim.IsA() is
            //       much cheaper than HasAPI() but not super cheap either. Thus, rather
            //       use the path map we have already.

            CARB_ASSERT(gStage->GetPseudoRoot().GetPath() != path);
            // since the path describes a property (see path.IsPropertyPath() check), there is no
            // need to check for root path and path.GetPrimPath() can be used directly as checking
            // for the root path is not free. Assert to make sure this is as assumed.

            const SdfPath primPath = path.GetPrimPath();

            CameraControllerMap::iterator cameraIter = mCameraControllerMap.find(primPath);

            if (cameraIter != mCameraControllerMap.end())
            {
                cameraDirty(cameraIter->second);
            }
        }
    }
}

} // namespace physx
} // namespace omni
