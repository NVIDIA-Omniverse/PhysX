// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS


#include "CctManager.h"

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCct.h>
#include <carb/input/IInput.h>
#include <carb/input/InputTypes.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>

#include <omni/kit/IAppWindow.h>

using namespace carb;
using namespace omni::physx;
using namespace pxr;

omni::physx::IPhysx* gPhysXInterface = nullptr;
omni::timeline::TimelinePtr gTimeline = nullptr;
carb::dictionary::IDictionary* gDictionary = nullptr;
omni::physx::CctManager* gCctManager = nullptr;
carb::input::IInput* gInput = nullptr;
UsdStageRefPtr gStage = nullptr;
omni::kit::StageUpdatePtr gStageUpdate = nullptr;
omni::kit::StageUpdateNode* gStageUpdateNode = nullptr;

carb::events::IEventStreamPtr gCctEventStream;
carb::events::ISubscriptionPtr gTimelineEvtSub;

using namespace pxr;

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.cct.plugin", "PhysX", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxCct)
CARB_PLUGIN_IMPL_DEPS(omni::timeline::ITimeline,
                      omni::physx::IPhysx,                      
                      omni::kit::IStageUpdate,
                      carb::input::IInput,
                      carb::dictionary::IDictionary)

void onAttach(long int stageId, double metersPerUnit, void*)
{
    gStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
}

void onDetach(void*)
{
    gCctManager->release();
    gStage = nullptr;
}

void update(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings*, void*)
{
    if (gTimeline->isPlaying())
    {
        gCctManager->update(elapsedSecs);

        {
            CARB_PROFILE_ZONE(0, "CctEventStreamPump");
            gCctEventStream->pump();
        }
    }
}

void onResume(float, void*)
{
    gCctManager->onResume();
}

void onPause(void*)
{
    gCctManager->onPause();
}

void onStop(void*)
{
    gCctManager->onStop();
}

void onPrimOrPropertyChange(const SdfPath& primOrPropertyPath, void* userData)
{
    if (!gStage)
        return;

    SdfPath primPath = primOrPropertyPath != SdfPath::AbsoluteRootPath() ?
        primOrPropertyPath.GetPrimPath() :
        primOrPropertyPath;
    UsdPrim usdPrim = gStage->GetPrimAtPath(primPath);
    if (usdPrim && usdPrim.HasAPI<PhysxSchemaPhysxCharacterControllerAPI>())
    {
        gCctManager->controllerDirty(primPath);
    }
}

CARB_EXPORT void carbOnPluginStartup()
{
    carb::Framework* framework = carb::getFramework();
    gPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
    gTimeline = omni::timeline::getTimeline();
    gDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>();
    gInput = carb::getCachedInterface<carb::input::IInput>();
    gCctManager = new CctManager();

    gStageUpdate = omni::kit::getStageUpdate();

    omni::kit::StageUpdateNodeDesc desc = { 0 };
    desc.displayName = "PhysXCct";
    desc.order = omni::kit::update::eIUsdStageUpdatePhysicsCCT;
    desc.onAttach = onAttach;
    desc.onDetach = onDetach;
    desc.onUpdate = update;
    desc.onPrimOrPropertyChange = onPrimOrPropertyChange;
    desc.onResume = onResume;
    desc.onPause = onPause;
    desc.onStop = onStop;
    if (gStageUpdate)
        gStageUpdateNode = gStageUpdate->createStageUpdateNode(desc);

    gTimelineEvtSub = carb::events::createSubscriptionToPopByType(
        gTimeline->getTimelineEventStream(),
        static_cast<carb::events::EventType>(omni::timeline::TimelineEventType::ePlay),
        [](carb::events::IEvent* e) {
            gCctManager->onTimelinePlay();
        },
        0, "PhysX::CCT");

    gCctEventStream = carb::events::getCachedEventsInterface()->createEventStream();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if (gStageUpdate)
        gStageUpdate->destroyStageUpdateNode(gStageUpdateNode);

    delete gCctManager;
    gCctManager = nullptr;
    
    gCctEventStream = nullptr;
    gTimelineEvtSub->unsubscribe();
    gTimelineEvtSub = nullptr;
}

#define CHECK_PARAM_RT(param, rt) if (!param) { CARB_LOG_ERROR("%s : " #param " is None", __FUNCTION__); return rt; }
#define CHECK_PARAM(param) CHECK_PARAM_RT(param, )

void setPosition(const char* path, const carb::Double3& pos)
{
    CHECK_PARAM(path)

    gCctManager->setPosition(pxr::SdfPath(path), pos);
}

void enableGravity(const char* path)
{
    CHECK_PARAM(path)

    gCctManager->enableGravity(pxr::SdfPath(path));
}

void enableCustomGravity(const char* path, const carb::Double3& gravity)
{
    CHECK_PARAM(path)

    gCctManager->enableCustomGravity(pxr::SdfPath(path), gravity);
}

void disableGravity(const char* path)
{
    CHECK_PARAM(path)

    gCctManager->disableGravity(pxr::SdfPath(path));
}

bool hasGravityEnabled(const char* path)
{
    CHECK_PARAM_RT(path, false)

    return gCctManager->hasGravityEnabled(pxr::SdfPath(path));
}

float getControllerHeight(const char* path)
{
    CHECK_PARAM_RT(path, 0.f)

    return gCctManager->getControllerHeight(pxr::SdfPath(path));
}

void setControllerHeight(const char* path, float height)
{
    CHECK_PARAM(path)

    gCctManager->setControllerHeight(pxr::SdfPath(path), height);
}

void enableFirstPerson(const char* path, const char* cameraPath)
{
    CHECK_PARAM(path)
    CHECK_PARAM(cameraPath)

    gCctManager->enableFirstPerson(pxr::SdfPath(path), pxr::SdfPath(cameraPath));
}

void disableFirstPerson(const char* path)
{
    CHECK_PARAM(path)

    gCctManager->disableFirstPerson(pxr::SdfPath(path));
}

void activateCct(const char* path)
{
    CHECK_PARAM(path)

    gCctManager->activateCct(pxr::SdfPath(path));
}

void removeCct(const char* path)
{
    CHECK_PARAM(path)

    gCctManager->removeCct(pxr::SdfPath(path));
}

carb::events::IEventStreamPtr getCctEventStream()
{
    return gCctEventStream;
}

void enableWorldSpaceMove(const char* path, bool enable)
{
    CHECK_PARAM(path)

    gCctManager->enableWorldSpaceMove(pxr::SdfPath(path), enable);
}

void setMove(const char* path, const carb::Float3& displacement)
{
    CHECK_PARAM(path)

    gCctManager->setMove(pxr::SdfPath(path), displacement);
}

void fillInterface(IPhysxCct& iface)
{
    iface.setPosition = setPosition;
    iface.enableGravity = enableGravity;
    iface.enableCustomGravity = enableCustomGravity;
    iface.disableGravity = disableGravity;
    iface.hasGravityEnabled = hasGravityEnabled;
    iface.getControllerHeight = getControllerHeight;
    iface.setControllerHeight = setControllerHeight;
    iface.enableFirstPerson = enableFirstPerson;
    iface.disableFirstPerson = disableFirstPerson;
    iface.activateCct = activateCct;
    iface.removeCct = removeCct;
    iface.getCctEventStream = getCctEventStream;
    iface.enableWorldSpaceMove = enableWorldSpaceMove;
    iface.setMove = setMove;
}
