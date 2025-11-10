// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/physx/IPhysx.h>

#include <common/utilities/PrimHierarchyStorage.h>
#include <carb/scripting/IScripting.h>

#include <map>


namespace omni
{
namespace physx
{

class CameraController;

typedef std::map<pxr::SdfPath, CameraController*> CameraControllerMap;

typedef std::list<pxr::SdfPath> CameraControllerPathList;
typedef std::map<pxr::SdfPath, CameraControllerPathList> CameraMap;


class CameraManager
{
public:
    enum PhysicsUpdate
    {
        eUPDATE_PHYSICS,
        eUPDATE_ANIMATION,
        eUPDATE_BOTH
    };


    CameraManager();
    ~CameraManager();

    bool addFollowLookCamera(const pxr::SdfPath& subjectPath, const pxr::SdfPath& cameraPath);
    bool addFollowVelocityCamera(const pxr::SdfPath& subjectPath, const pxr::SdfPath& cameraPath);
    bool addDroneCamera(const pxr::SdfPath& subjectPath, const pxr::SdfPath& cameraPath);

    void createControllers();
    void addPrim(const pxr::UsdPrim&);
    void removePrim(const pxr::SdfPath&);

    bool hasSimulationStarted()
    {
        return mSimulationStarted;
    }
    bool hasUsdChangeListeners()
    {
        return (mUsdChangeListenerCount > 0);
    }

    void preUpdate();
    void stepUpdate(float timeStep, PhysicsUpdate physicsUpdate);
    void postUpdate(PhysicsUpdate physicsUpdate);

    void onResume();
    void onStop();

    void release();
    void destroyScriptingContext();

    void onUsdObjectChange(const pxr::SdfPath& path, const pxr::UsdTimeCode& timeCode);

    CameraController* getCamera(const pxr::UsdPrim& prim);

    void setActiveCameraPath(pxr::SdfPath activeCameraPath);
    pxr::SdfPath getActiveCameraPath();

private:
    void addCameraController(const pxr::SdfPath& cameraPath);
    void removeCameraController(const pxr::SdfPath& cameraPath, bool removeReferences = true);
    bool addPrimInternal(const pxr::UsdPrim&);

    void cameraDirty(CameraController*);


private:
    bool mSimulationStarted;
    bool mSetupInputs;

    unsigned int mUsdChangeListenerCount;

    pxr::SdfPath mActiveCameraPath;

    SubscriptionId mPhysicsStepEventId;
    SubscriptionId mPhysicsCompletionEventId;
    bool mHasPhysicsStepEventSubscription;

    CameraControllerMap mCameraControllerMap;
    CameraMap mCameraMap;

    // tracking the prims (and their parent prim chain) that are relevant for the
    // vehicle extension update loop. Allows to find out if a relevant prim is
    // a descendant of the prim that got removed.
    PrimHierarchyStorage mPrimHierarchyStorage;

    carb::scripting::IScripting* mScripting;
    carb::scripting::Context* mScriptingContext;
    std::string mCameraPath;
};

} // namespace physx
} // namespace omni
