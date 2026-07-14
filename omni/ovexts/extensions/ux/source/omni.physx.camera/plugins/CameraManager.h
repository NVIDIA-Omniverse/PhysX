// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

typedef std::map<PXR_NS::SdfPath, CameraController*> CameraControllerMap;

typedef std::list<PXR_NS::SdfPath> CameraControllerPathList;
typedef std::map<PXR_NS::SdfPath, CameraControllerPathList> CameraMap;


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

    bool addFollowLookCamera(const PXR_NS::SdfPath& subjectPath, const PXR_NS::SdfPath& cameraPath);
    bool addFollowVelocityCamera(const PXR_NS::SdfPath& subjectPath, const PXR_NS::SdfPath& cameraPath);
    bool addDroneCamera(const PXR_NS::SdfPath& subjectPath, const PXR_NS::SdfPath& cameraPath);

    void createControllers();
    void addPrim(const PXR_NS::UsdPrim&);
    void removePrim(const PXR_NS::SdfPath&);

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

    void onUsdObjectChange(const PXR_NS::SdfPath& path, const PXR_NS::UsdTimeCode& timeCode);

    CameraController* getCamera(const PXR_NS::UsdPrim& prim);

    void setActiveCameraPath(PXR_NS::SdfPath activeCameraPath);
    PXR_NS::SdfPath getActiveCameraPath();

private:
    void addCameraController(const PXR_NS::SdfPath& cameraPath);
    void removeCameraController(const PXR_NS::SdfPath& cameraPath, bool removeReferences = true);
    bool addPrimInternal(const PXR_NS::UsdPrim&);

    void cameraDirty(CameraController*);


private:
    bool mSimulationStarted;
    bool mSetupInputs;

    unsigned int mUsdChangeListenerCount;

    PXR_NS::SdfPath mActiveCameraPath;

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
