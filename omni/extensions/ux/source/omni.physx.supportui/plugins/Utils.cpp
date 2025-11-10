// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "Utils.h"

#include <carb/events/EventsUtils.h>
#include <carb/events/IEvents.h>

#include <common/utilities/Utilities.h>
#include <carb/settings/ISettings.h>
#include <private/omni/physx/IPhysxSupportUi.h>

namespace omni
{
namespace physx
{

void resetSettingAtPath(const char* path)
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    std::string defaultPath = std::string(DEFAULT_SETTING_PREFIX) + path;

    carb::dictionary::ItemType itemType = settings->getItemType(defaultPath.c_str());
    switch (itemType)
    {
    case carb::dictionary::ItemType::eBool:
        settings->setBool(path, settings->getAsBool(defaultPath.c_str()));
        break;
    case carb::dictionary::ItemType::eFloat:
        settings->setFloat(path, settings->getAsFloat(defaultPath.c_str()));
        break;
    case carb::dictionary::ItemType::eInt:
        settings->setInt(path, settings->getAsInt(defaultPath.c_str()));
        break;
    case carb::dictionary::ItemType::eString:
        settings->setString(path, settings->getStringBuffer(defaultPath.c_str()));
        break;
    case carb::dictionary::ItemType::eCount:
        CARB_LOG_ERROR("Can't reset setting %s! Path not found.", path);
    default:
        CARB_LOG_ERROR("Can't reset setting %s! Type not supported.", path);
        return;
    }
}

void sendColliderCreatedEvent(carb::events::IEventStreamPtr eventStream,
                              carb::dictionary::IDictionary* dict,
                              const pxr::SdfPath& primPath,
                              bool colliderType,
                              int simplificationType,
                              size_t numRemainingCollTasks,
                              size_t numTotalCollTasks)
{
    if (eventStream)
    {
        // send eColliderCreated event
        carb::events::IEventPtr event = carb::stealObject(eventStream->createEventPtr(
            static_cast<carb::events::EventType>(IPhysxSupportUi::EventType::eColliderCreated),
            carb::events::kGlobalSenderId,
            std::make_pair("colliderType", colliderType),
            std::make_pair("simplificationType", simplificationType),
            std::make_pair("numRemainingCollTasks", (int)numRemainingCollTasks),
            std::make_pair("numTotalCollTasks", (int)numTotalCollTasks)));

        createEventFromSdfPath(event, "primPath", dict, primPath);

        eventStream->push(event.get());
    }
}

void sendRigidBodyApiChangedEvent(carb::events::IEventStreamPtr eventStream,
                              carb::dictionary::IDictionary* dict,
                              const pxr::SdfPath& primPath,
                              bool created)
{
    if (eventStream)
    {
        // send eColliderCreated event
        carb::events::IEventPtr event = carb::stealObject(eventStream->createEventPtr(
            static_cast<carb::events::EventType>(created ? IPhysxSupportUi::EventType::eRigidBodyCreated :
                                                           IPhysxSupportUi::EventType::eRigidBodyRemoved),
            carb::events::kGlobalSenderId));

        createEventFromSdfPath(event, "primPath", dict, primPath);

        eventStream->push(event.get());
    }
}

void sendKinematicsToggleChangedEvent(carb::events::IEventStreamPtr eventStream,
                                      carb::dictionary::IDictionary* dict,
                                      const pxr::SdfPath& primPath)
{
    if (eventStream)
    {
        carb::events::IEventPtr event = carb::stealObject(eventStream->createEventPtr(
            static_cast<carb::events::EventType>(IPhysxSupportUi::EventType::eKinematicsToggled),
            carb::events::kGlobalSenderId));

        createEventFromSdfPath(event, "primPath", dict, primPath);

        eventStream->push(event.get());
    }
}

void sendAutoCollCanceledChangedEvent(carb::events::IEventStreamPtr eventStream,
                                      carb::dictionary::IDictionary* dict,
                                      const pxr::SdfPath& primPath)
{
    if (eventStream)
    {
        carb::events::IEventPtr event = carb::stealObject(eventStream->createEventPtr(
            static_cast<carb::events::EventType>(IPhysxSupportUi::EventType::eAutoCollCanceled),
            carb::events::kGlobalSenderId));

        createEventFromSdfPath(event, "primPath", dict, primPath);

        eventStream->push(event.get());
    }
}


#define CASE_ENUM2STR(enm) case enm: return #enm;

const char* toString(omni::usd::StageState stageState)
{
    switch (stageState)
    {
        CASE_ENUM2STR(omni::usd::StageState::eClosed) ///! USD is closed/unopened.
        CASE_ENUM2STR(omni::usd::StageState::eOpening) ///! USD is opening.
        CASE_ENUM2STR(omni::usd::StageState::eOpened) ///! USD is opened.
        CASE_ENUM2STR(omni::usd::StageState::eClosing) ///! USD is closing.
        default: return "Unknown omni::usd::StageState";
    };
}

const char* toString(omni::usd::StageEventType stageEventType)
{
    switch (stageEventType)
    {
        CASE_ENUM2STR(omni::usd::StageEventType::eSaved) ///! USD file saved.
        CASE_ENUM2STR(omni::usd::StageEventType::eSaveFailed) ///! Failed to save USD.
        CASE_ENUM2STR(omni::usd::StageEventType::eOpening) ///! USD stage is opening.
        CASE_ENUM2STR(omni::usd::StageEventType::eOpened) ///! USD stage is opened successfully.
        CASE_ENUM2STR(omni::usd::StageEventType::eOpenFailed) ///! USD stage failed to open.
        CASE_ENUM2STR(omni::usd::StageEventType::eClosing) ///! USD stage is about to close. This is a good opportunity to shutdown anything depends on USD stage.
        CASE_ENUM2STR(omni::usd::StageEventType::eClosed) ///! USD stage is fully closed.
        CASE_ENUM2STR(omni::usd::StageEventType::eSelectionChanged) ///! USD Prim selection has changed.
        CASE_ENUM2STR(omni::usd::StageEventType::eAssetsLoaded) ///! Current batch of async asset loading has been completed.
        CASE_ENUM2STR(omni::usd::StageEventType::eAssetsLoadAborted) ///! Current batch of async asset loading has been aborted.
        CASE_ENUM2STR(omni::usd::StageEventType::eGizmoTrackingChanged) ///! Started or stopped tracking (hovering) on a gizmo
        CASE_ENUM2STR(omni::usd::StageEventType::eMdlParamLoaded) ///! MDL parameter is loaded for a MDL UsdShadeShader.
        CASE_ENUM2STR(omni::usd::StageEventType::eSettingsLoaded) /// Stage settings have loaded
        CASE_ENUM2STR(omni::usd::StageEventType::eSettingsSaving) /// Stage settings are being saved
        CASE_ENUM2STR(omni::usd::StageEventType::eOmniGraphStartPlay) /// OmniGraph play has started
        CASE_ENUM2STR(omni::usd::StageEventType::eOmniGraphStopPlay) /// OmniGraph play has stopped
        CASE_ENUM2STR(omni::usd::StageEventType::eSimulationStartPlay) /// Simulation play has started
        CASE_ENUM2STR(omni::usd::StageEventType::eSimulationStopPlay) /// Simulation play has stopped
        CASE_ENUM2STR(omni::usd::StageEventType::eAnimationStartPlay) /// Animation playback has started
        CASE_ENUM2STR(omni::usd::StageEventType::eAnimationStopPlay) /// Animation playback has stopped
        CASE_ENUM2STR(omni::usd::StageEventType::eDirtyStateChanged) /// Dirty state of USD stage has changed. Dirty state means if it has unsaved changes or not.
        CASE_ENUM2STR(omni::usd::StageEventType::eAssetsLoading) ///! A new batch of async asset loading has started.
        CASE_ENUM2STR(omni::usd::StageEventType::eActiveLightsCountChanged) ///! Number of active lights in the scene has changed. This signal should be triggered every time a scene has been loaded or number of lights has been changed. A few features, for instance view lighting mode, need to detect when a number of active lights becomes zero / become non-zero.
        CASE_ENUM2STR(omni::usd::StageEventType::eHierarchyChanged) ///! USD stage hierarchy has changed.
        CASE_ENUM2STR(omni::usd::StageEventType::eHydraGeoStreamingStarted) ///! Fabric Scene Delegate sends this when starting to stream rprims.
        CASE_ENUM2STR(omni::usd::StageEventType::eHydraGeoStreamingStopped) ///! Fabric Scene Delegate sends this when stopping to stream rprims.
        CASE_ENUM2STR(omni::usd::StageEventType::eHydraGeoStreamingStoppedNotEnoughMem) ///! Fabric Scene Delegate sends this when geometry streaming stops loading more geometry because of insufficient device memory
        CASE_ENUM2STR(omni::usd::StageEventType::eHydraGeoStreamingStoppedAtLimit) ///! Fabric Scene Delegate sends this when stopping to stream rprims because of the limit set by the user.
        CASE_ENUM2STR(omni::usd::StageEventType::eSaving) ///! Saving is in progress
        default: return "Unknown omni::usd::StageEventType";
    };
}

#undef CASE_ENUM2STR

}
}
