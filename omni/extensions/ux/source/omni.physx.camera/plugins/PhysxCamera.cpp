// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include "CameraManager.h"

#include <carb/Framework.h>
#include <carb/input/IInput.h>
#include <carb/settings/ISettings.h>
#include <carb/PluginUtils.h>
#include <carb/scripting/IScripting.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCamera.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <PxPhysicsAPI.h>

#include <common/foundation/Allocator.h>
#include <common/utilities/PhysXErrorCallback.h>


using namespace omni;
using namespace omni::physx;
using namespace pxr;


omni::physx::IPhysx* gPhysXInterface = nullptr;
UsdStageRefPtr gStage = nullptr;
omni::kit::StageUpdatePtr gStageUpdate;
omni::kit::StageUpdateNode* gStageUpdateNodePrePhysics = nullptr;
omni::kit::StageUpdateNode* gStageUpdateNodePostPhysics = nullptr;

omni::physx::CameraManager* gCameraManager = nullptr;

static carb::events::IEventStreamPtr gErrorEventStream;
static ::physx::PxDefaultAllocator gAllocator;
static CarbPhysXErrorCallback gErrorCallback;
static ::physx::PxFoundation* gFoundation = nullptr;

struct RunState
{
    enum Enum
    {
        ePLAYING,
        ePAUSED,
        eSTOPPED
    };
};

static int gRunState = RunState::eSTOPPED;

typedef pxr::TfHashSet<pxr::SdfPath, pxr::SdfPath::Hash> PrimAddMap;

class PrimUpdateMap
{
public:

    PrimUpdateMap()
    {
    }

    void addPrim(const pxr::UsdPrim&);
    void removePrim(const pxr::SdfPath&);

    void clearMap()
    {
        mPrimMap.clear();
    }

    bool isInPrimAddMap(const pxr::UsdPrim&) const;

    PrimAddMap& getMap()
    {
        return mPrimMap;
    }

private:

    PrimAddMap mPrimMap;
};

static PrimUpdateMap gPrimUpdateMap;


const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.camera.plugin", "PhysX Camera", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxCamera)
CARB_PLUGIN_IMPL_DEPS(omni::kit::IStageUpdate,
                      omni::physx::IPhysx, 
                      carb::input::IInput,
                      carb::settings::ISettings,
                      carb::scripting::IScripting)


struct UsdNoticeListener : public pxr::TfWeakBase
{
    UsdNoticeListener() = default;

    void handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged);
};

void processUSDNotifyUpdates()
{
    if (gStage)
    {
        for (PrimAddMap::const_iterator it = gPrimUpdateMap.getMap().begin(); it != gPrimUpdateMap.getMap().end(); it++)
        {
            const pxr::SdfPath& path = (*it);
            const pxr::UsdPrim& prim = gStage->GetPrimAtPath(path);
            if (!prim)
            {
                // a prim can get added and then removed in the same frame. If this prim is removed indirectly
                // through a parent, there is no remove notification for the prim itself and hence this check.
                continue;
            }

            gCameraManager->addPrim(prim);
        }
    }

    gPrimUpdateMap.clearMap();
}

void onAttach(long int stageId, double metersPerUnit, void*)
{
    gStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
}

void onDetach(void*)
{
    if (gCameraManager)
    {
        gCameraManager->release();
    }

    gStage = nullptr;
}

// This method should only be called through the Python interface when the simulation is not running.
// Call it before the update method, below.
void preUpdate()
{
    if (gCameraManager)
    {
        gCameraManager->preUpdate();
    }
}

// This method should only be called through the Python interface when the simulation is not running.
// The physics step and post update methods are called at the same time.
void update(float elapsedSecs)
{
    if (gCameraManager)
    {
        gCameraManager->stepUpdate(elapsedSecs, CameraManager::PhysicsUpdate::eUPDATE_ANIMATION);
        gCameraManager->postUpdate(CameraManager::PhysicsUpdate::eUPDATE_ANIMATION);
    }
}

void onUpdatePrePhysics(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings*, void*)
{
    if (gRunState == RunState::ePLAYING)
    {
        if (gCameraManager)
        {
            gCameraManager->preUpdate();
        }
    }
}

void onUpdatePostPhysics(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings*, void*)
{
    if (gRunState == RunState::ePLAYING)
    {
        if (gCameraManager)
        {
            gCameraManager->stepUpdate(elapsedSecs, CameraManager::PhysicsUpdate::eUPDATE_ANIMATION);
            gCameraManager->postUpdate(CameraManager::PhysicsUpdate::eUPDATE_ANIMATION);
        }

        // Prims can be added during the PhysX update, but are not processed until afterwards. The cameras
        // cache a pointer to a rigid body which must be created and stored in the internal scene before we can
        // retrieve it here.
        processUSDNotifyUpdates();
    }
}

void onResume(float currentTime, void* userData)
{
    if (gCameraManager)
    {
        gCameraManager->onResume();
    }

    gRunState = RunState::ePLAYING;
}

void onPause(void* userData)
{
    gRunState = RunState::ePAUSED;
}

void onStop(void* userData)
{
    gRunState = RunState::eSTOPPED;

    if (gCameraManager)
    {
        gCameraManager->onStop();
    }
}

void UsdNoticeListener::handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    CARB_PROFILE_ZONE(0, "PhysXCameraUsdNoticeListener");
    TRACE_FUNCTION();

    if (!gCameraManager ||
        gCameraManager->hasSimulationStarted() == false)
    {
        return;
    }

    // This is an old callback, ignore it
    if (!gStage || gStage != objectsChanged.GetStage())
    {
        return;
    }

    for (const SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        if (path.IsAbsoluteRootOrPrimPath())
        {
            const SdfPath primPath = gStage->GetPseudoRoot().GetPath() == path ?
                gStage->GetPseudoRoot().GetPath() :
                path.GetPrimPath();

            UsdPrim prim = gStage->GetPrimAtPath(primPath);

            // Determine if the prim was added or removed.
            if (prim.IsValid() == false || !prim.IsActive())
            {
                gPrimUpdateMap.removePrim(primPath);
            }
            else
            {
                gPrimUpdateMap.addPrim(prim);
            }
        }
    }

    // for now this is set to default
    const pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default();

    if (gCameraManager->hasUsdChangeListeners())
    {
        for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
        {
            UsdPrim prim = gStage->GetPrimAtPath(path);

            if (!gPrimUpdateMap.isInPrimAddMap(prim))
            {
                gCameraManager->onUsdObjectChange(path, timeCode);
            }
        }
    }
}

void PrimUpdateMap::addPrim(const pxr::UsdPrim& prim)
{
    // the whole subtree will get parsed later for an added prim, thus only the
    // root prim of an add should get registered
    if (!isInPrimAddMap(prim))
    {
        const SdfPath& path = prim.GetPrimPath();
        mPrimMap.insert(path);
    }
}

void PrimUpdateMap::removePrim(const pxr::SdfPath& primPath)
{
    gCameraManager->removePrim(primPath);

    mPrimMap.erase(primPath);
}

bool PrimUpdateMap::isInPrimAddMap(const pxr::UsdPrim& prim) const
{
    if (mPrimMap.empty() ||
        !prim.IsValid())
    {
        return false;
    }

    UsdPrim parent = prim;
    const UsdPrim root = gStage->GetPseudoRoot();

    while (parent != root)
    {
        PrimAddMap::const_iterator it = mPrimMap.find(parent.GetPrimPath());

        if (it != mPrimMap.end())
        {
            return true;
        }

        parent = parent.GetParent();
    }

    return false;
}

void registerToStageUpdate()
{
    if (gStageUpdate)
    {
        {
            omni::kit::StageUpdateNodeDesc desc = { 0 };
            desc.displayName = "PhysXCameraPrePhysics";
            desc.order = omni::kit::update::eIUsdStageUpdatePhysicsCameraPrePhysics; // this makes sure it runs before omni.physx update
            desc.onAttach = onAttach;
            desc.onDetach = onDetach;
            desc.onUpdate = onUpdatePrePhysics;
            desc.onPause = onPause;
            desc.onStop = onStop;

            gStageUpdateNodePrePhysics = gStageUpdate->createStageUpdateNode(desc);
        }

        {
            omni::kit::StageUpdateNodeDesc desc = { 0 };
            desc.displayName = "PhysXCameraPostPhysics";
            desc.order = omni::kit::update::eIUsdStageUpdatePhysicsCameraPostPhysics; // this makes sure it runs after omni.physx update
            desc.onResume = onResume;
            desc.onUpdate = onUpdatePostPhysics;
            // omni.physx creates the PhysX objects that are needed in the camera controllers.
            // Thus, creation of controllers should happen afterwards.

            gStageUpdateNodePostPhysics = gStageUpdate->createStageUpdateNode(desc);
        }
    }
}

void unregisterFromStageUpdate()
{
    if (gStageUpdate)
    {
        if (gStageUpdateNodePrePhysics)
        {
            gStageUpdate->destroyStageUpdateNode(gStageUpdateNodePrePhysics);
            gStageUpdateNodePrePhysics = nullptr;
        }

        if (gStageUpdateNodePostPhysics)
        {
            gStageUpdate->destroyStageUpdateNode(gStageUpdateNodePostPhysics);
            gStageUpdateNodePostPhysics = nullptr;
        }
    }
}

pxr::TfNotice::Key gUsdNoticeListenerKey;
UsdNoticeListener* gUsdNoticeListener = nullptr;

CARB_EXPORT void carbOnPluginStartup()
{
    carb::Framework* framework = carb::getFramework();
    gPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();

    gErrorEventStream = carb::events::getCachedEventsInterface()->createEventStream();
    gErrorCallback.setEventStream(gErrorEventStream);
    gErrorCallback.setErrorEventType(omni::physx::ePhysxError);
    // create PxFoundation since this extension is using some PhysX methods that can send error
    // messages etc.
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    gStageUpdate = omni::kit::getStageUpdate();
    registerToStageUpdate();

    gUsdNoticeListener = ICE_PLACEMENT_NEW(UsdNoticeListener)();
    gUsdNoticeListenerKey = pxr::TfNotice::Register(pxr::TfCreateWeakPtr(gUsdNoticeListener), &UsdNoticeListener::handle);

    gCameraManager = ICE_PLACEMENT_NEW(CameraManager)();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    ICE_PLACEMENT_DELETE(gCameraManager, CameraManager);

    pxr::TfNotice::Revoke(gUsdNoticeListenerKey);
    ICE_PLACEMENT_DELETE(gUsdNoticeListener, UsdNoticeListener);
    gUsdNoticeListener = nullptr;

    // note: aquired interfaces are not released here as this causes warning messages "...already been released"

    unregisterFromStageUpdate();

    if (gFoundation)
    {
        gFoundation->release();
        gFoundation = nullptr;
    }
    gErrorCallback.invalidateEventStream();
    gErrorEventStream = nullptr;
}

bool addFollowLookCamera(const char* subjectPath, const char* cameraPath)
{
    return gCameraManager->addFollowLookCamera(pxr::SdfPath(subjectPath), pxr::SdfPath(cameraPath));
}

bool addFollowVelocityCamera(const char* subjectPath, const char* cameraPath)
{
    return gCameraManager->addFollowVelocityCamera(pxr::SdfPath(subjectPath), pxr::SdfPath(cameraPath));
}

bool addDroneCamera(const char* subjectPath, const char* cameraPath)
{
    return gCameraManager->addDroneCamera(pxr::SdfPath(subjectPath), pxr::SdfPath(cameraPath));
}

// This method is only called from Python for automated testing.
bool attachStage(long int stageId, bool unregFromStageUpdate)
{
    if (unregFromStageUpdate)
    {
        unregisterFromStageUpdate();
    }

    if (gStage)
    {
        onDetach(nullptr);
        CARB_ASSERT(!gStage);
    }

    if (stageId)
    {
        UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
        
        if (stage)
        {
            gStage = stage;

            // Set the active camera path so the stepUpdate and postUpdate methods can execute during testing.
            gCameraManager->setActiveCameraPath(pxr::SdfPath("TestCamera"));

            onResume(0.0f, nullptr);

            return true;
        }
        else
        {
            CARB_LOG_ERROR("PhysX Camera: attaching to stage failed. Fetching stage for ID %ld failed.\n", stageId);
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Camera: attaching to stage failed. Invalid stage ID %ld.\n", stageId);
    }

    return false;
}

void detachStage(bool regToStageUpdate)
{
    UsdStageRefPtr stage;
    long stageId;

    if (regToStageUpdate)
    {
        stage = gStage;
        stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    }

    onDetach(nullptr);

    if (regToStageUpdate)
    {
        if (stage)
        {
            onAttach(stageId, pxr::UsdGeomGetStageMetersPerUnit(stage), nullptr);
        }

        registerToStageUpdate();
    }
}

void fillInterface(IPhysxCamera& iface)
{
    iface.addFollowLookCamera = addFollowLookCamera;
    iface.addFollowVelocityCamera = addFollowVelocityCamera;
    iface.addDroneCamera = addDroneCamera;

    iface.attachStage = attachStage;
    iface.detachStage = detachStage;
    iface.preUpdate = preUpdate;
    iface.update = update;
    iface.processPendingUSDChanges = processUSDNotifyUpdates;
}
