// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/settings/ISettings.h>
#include <omni/renderer/IDebugDraw.h>
#include <carb/settings/ISettings.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/IAppWindow.h>
#include <omni/ui/IGlyphManager.h>
#include <carb/tokens/ITokens.h>
#include <omni/kit/KitUpdateOrder.h>
#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <carb/tasking/ITasking.h>
#include <omni/ext/IExt.h>
#include <omni/ext/ExtensionsUtils.h>
#include <omni/kit/IApp.h>

#include <private/omni/physx/IPhysxPvd.h>
#include <omni/physx/IPhysxSettings.h>

#include "OmniPvdOvdParser.h"
#include "OmniPvdOvdParserConfig.h"
#include "OmniPvdUsdWriter.h"
#include "OmniPvdDrawGizmos.h"
#include "OmniPvdDebugDraw.h"
#include "OmniPvdUsdOverWriter.h"

extern OmniPvdMessages gOmniPvdMessages;

using namespace carb;
using namespace omni;
using namespace pxr;

namespace
{
    carb::settings::ISettings* gSettings = nullptr;
    std::vector<carb::dictionary::SubscriptionId*> gUISettingCallbackSubscriptions;

    void unsubscribeFromSettingsCallbacks()
    {
        for (auto sub : gUISettingCallbackSubscriptions)
        {
            gSettings->unsubscribeToChangeEvents(sub);
        }
        gUISettingCallbackSubscriptions.clear();
    }

    void subscribeToSettingsCallbacks()
    {
        unsubscribeFromSettingsCallbacks();

        auto onGizmoVizModeChange = [](const carb::dictionary::Item* changedItem,
            carb::dictionary::ChangeEventType changeEventType, void* userData) {
                carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
                if (!dict)
                    return;
                const int32_t vizMode = dict->getAsInt(changedItem);
                int32_t type = OmniPVDDebugGizmoType::eNbrEnums; //invalid type
                if (userData == omni::physx::kOmniPvdGizmoContactVizMode)
                {
                    type = OmniPVDDebugGizmoType::eContact;
                }
                else if (userData == omni::physx::kOmniPvdGizmoCenterOfMassVizMode)
                {
                    type = OmniPVDDebugGizmoType::eCenterOfMass;
                }
                else if (userData == omni::physx::kOmniPvdGizmoJointVizMode)
                {
                    type = OmniPVDDebugGizmoType::eJoint;
                }
                else if (userData == omni::physx::kOmniPvdGizmoBoundingBoxVizMode)
                {
                    type = OmniPVDDebugGizmoType::eBoundingBox;
                }
                else if (userData == omni::physx::kOmniPvdGizmoCoordinateSystemVizMode)
                {
                    type = OmniPVDDebugGizmoType::eCoordinateSystem;
                }
                else if (userData == omni::physx::kOmniPvdGizmoVelocityVizMode)
                {
                    type = OmniPVDDebugGizmoType::eVelocity;
                }
                else if (userData == omni::physx::kOmniPvdGizmoTransparencyVizMode)
                {
                    type = OmniPVDDebugGizmoType::eTransparency;
                }


                if (type < OmniPVDDebugGizmoType::eNbrEnums)
                {
                    selectGizmo(type, vizMode);
                }
        };

        auto onGizmoScaleChange = [](const carb::dictionary::Item* changedItem,
            carb::dictionary::ChangeEventType changeEventType, void* userData) {
                carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
                if (!dict)
                    return;

                const float scale = dict->getAsFloat(changedItem);
                if (userData == omni::physx::kOmniPvdGizmoGlobalScale)
                {
                    setGizmoScale(scale);
                }
                else if (userData == omni::physx::kOmniPvdGizmoContactScale)
                {
                    setGizmoScale(OmniPVDDebugGizmoType::eContact, scale);
                }
                else if (userData == omni::physx::kOmniPvdGizmoCenterOfMassScale)
                {
                    setGizmoScale(OmniPVDDebugGizmoType::eCenterOfMass, scale);
                }
                else if (userData == omni::physx::kOmniPvdGizmoJointScale)
                {
                    setGizmoScale(OmniPVDDebugGizmoType::eJoint, scale);
                }
                else if (userData == omni::physx::kOmniPvdGizmoCoordinateSystemScale)
                {
                    setGizmoScale(OmniPVDDebugGizmoType::eCoordinateSystem, scale);
                }
                else if (userData == omni::physx::kOmniPvdGizmoVelocityScale)
                {
                    setGizmoScale(OmniPVDDebugGizmoType::eVelocity, scale);
                }
        };


        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoContactVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoContactVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoCenterOfMassVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoCenterOfMassVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoJointVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoJointVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoBoundingBoxVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoBoundingBoxVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoCoordinateSystemVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoCoordinateSystemVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoVelocityVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoVelocityVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoTransparencyVizMode, onGizmoVizModeChange, (void*)omni::physx::kOmniPvdGizmoTransparencyVizMode));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoGlobalScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoGlobalScale));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoContactScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoContactScale));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoCenterOfMassScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoCenterOfMassScale));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoJointScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoJointScale));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoCoordinateSystemScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoCoordinateSystemScale));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoVelocityScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoVelocityScale));

        gUISettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physx::kOmniPvdGizmoTransparencyScale, onGizmoScaleChange, (void*)omni::physx::kOmniPvdGizmoTransparencyScale));
    }
}

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.pvd.plugin", "PhysX", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysXPvd)
CARB_PLUGIN_IMPL_DEPS(carb::settings::ISettings,
    omni::kit::IStageUpdate,
    omni::timeline::ITimeline,
    carb::dictionary::IDictionary,
    omni::renderer::IDebugDraw,
    carb::tokens::ITokens,
    omni::ui::IGlyphManager,
    carb::tasking::ITasking,
    carb::input::IInput,
    omni::kit::IApp)


CARB_EXPORT void carbOnPluginStartup(
    )
{
    gSettings = carb::getCachedInterface<carb::settings::ISettings>();

    gSettings->setDefaultString(omni::physx::kOmniPvdImportedOvd, "");
    gSettings->setDefaultString(omni::physx::kOmniPvdOvdForBaking, "");
    gSettings->setDefaultString(omni::physx::kOmniPvdLastImportDirectory, "");
    gSettings->setDefaultString(omni::physx::kOmniPvdUsdCacheDirectory, "");
    gSettings->setDefaultString(omni::physx::kOmniPvdPhysXUsdDirectory, "");
    gSettings->setDefaultBool(omni::physx::kOmniPvdInvalidateCache, false);


    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoContactVizMode, OmniPVDDebugVizMode::eNone);
    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoCenterOfMassVizMode, OmniPVDDebugVizMode::eNone);
    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoJointVizMode, OmniPVDDebugVizMode::eNone);
    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoBoundingBoxVizMode, OmniPVDDebugVizMode::eNone);
    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoCoordinateSystemVizMode, OmniPVDDebugVizMode::eNone);
    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoVelocityVizMode, OmniPVDDebugVizMode::eNone);
    gSettings->setDefaultInt(omni::physx::kOmniPvdGizmoTransparencyVizMode, OmniPVDDebugVizMode::eNone);

    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoGlobalScale, 1.0f);
    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoContactScale, 0.3f);
    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoCenterOfMassScale, 1.0f);
    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoJointScale, 0.3f);
    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoCoordinateSystemScale, 1.0f);
    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoVelocityScale, 1.0f);
    gSettings->setDefaultFloat(omni::physx::kOmniPvdGizmoTransparencyScale, 1.0f);

    gSettings->setDefaultBool(omni::physx::kOmniPvdTimelineIsLegacyOvd, false);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelinePlaybackState, OmniPVDTimelinePlaybackState::eStopped);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineFrameMode, OmniPVDTimelineFrameMode::ePostSim);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineFrameDeltaMs, 1000);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineFramesPerSimStep, 1);
    
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineFrameId, 0);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineFrameIdMin, 0);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineFrameIdMax, 0);

    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineSimStepId, 0);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineSimStepIdMin, 0);
    gSettings->setDefaultInt(omni::physx::kOmniPvdTimelineSimStepIdMax, 0);

    subscribeToSettingsCallbacks();

    initGizmoMutex();

    //initialize from settings
    selectGizmo(OmniPVDDebugGizmoType::eContact, gSettings->getAsInt(omni::physx::kOmniPvdGizmoContactVizMode));
    selectGizmo(OmniPVDDebugGizmoType::eCenterOfMass, gSettings->getAsInt(omni::physx::kOmniPvdGizmoCenterOfMassVizMode));
    selectGizmo(OmniPVDDebugGizmoType::eJoint, gSettings->getAsInt(omni::physx::kOmniPvdGizmoJointVizMode));
    selectGizmo(OmniPVDDebugGizmoType::eBoundingBox, gSettings->getAsInt(omni::physx::kOmniPvdGizmoBoundingBoxVizMode));
    selectGizmo(OmniPVDDebugGizmoType::eCoordinateSystem, gSettings->getAsInt(omni::physx::kOmniPvdGizmoCoordinateSystemVizMode));
    selectGizmo(OmniPVDDebugGizmoType::eVelocity, gSettings->getAsInt(omni::physx::kOmniPvdGizmoVelocityVizMode));
    selectGizmo(OmniPVDDebugGizmoType::eTransparency, gSettings->getAsInt(omni::physx::kOmniPvdGizmoTransparencyVizMode));

    setGizmoScale(gSettings->getAsFloat(omni::physx::kOmniPvdGizmoGlobalScale));
    setGizmoScale(OmniPVDDebugGizmoType::eContact, gSettings->getAsFloat(omni::physx::kOmniPvdGizmoContactScale));
    setGizmoScale(OmniPVDDebugGizmoType::eCenterOfMass, gSettings->getAsFloat(omni::physx::kOmniPvdGizmoCenterOfMassScale));
    setGizmoScale(OmniPVDDebugGizmoType::eJoint, gSettings->getAsFloat(omni::physx::kOmniPvdGizmoJointScale));
    setGizmoScale(OmniPVDDebugGizmoType::eCoordinateSystem, gSettings->getAsFloat(omni::physx::kOmniPvdGizmoCoordinateSystemScale));
    setGizmoScale(OmniPVDDebugGizmoType::eVelocity, gSettings->getAsFloat(omni::physx::kOmniPvdGizmoVelocityScale));
    setGizmoScale(OmniPVDDebugGizmoType::eTransparency, gSettings->getAsFloat(omni::physx::kOmniPvdGizmoTransparencyScale));

    //this should probably be done in stage open event, or update notification

    applyGizmos();
}

void clearMessages()
{
    gOmniPvdMessages.clear();
}

CARB_EXPORT void carbOnPluginShutdown(
    )
{
    unsubscribeFromSettingsCallbacks();
    cleanupDebugViz();
    clearMessages();
}

bool ovdToUsd(char *omniPvdFile, char *usdStageDir, int upAxis, int isUSDA)
{
    clearMessages();

    OmniPvdDOMState domState;
    if (!buildPvdDomState(omniPvdFile, domState))
    {
        return false;
    }
    writeUSDFile(usdStageDir, upAxis, isUSDA, domState);
    return true;
}

long int ovdToUsdInMemory(char *omniPvdFile, int upAxis)
{
    clearMessages();

    OmniPvdDOMState domState;
    if (!buildPvdDomState(omniPvdFile, domState))
    {
        return 0;  // Return 0 to indicate failure
    }
    return createUSDFileInMemory(upAxis, domState);
}

bool ovdToUsdOver(char* omniPvdFile)
{
    clearMessages();

    OmniPvdDOMState domState;
    if (!buildPvdDomState(omniPvdFile, domState))
    {
        return false;
    }
    writeUSDFileOver(domState);
    return true;
}

bool ovdToUsdOverWithLayerCreation(char* omniPvdFile, char* inputStageFile  ,char* outputDir, char* outputStageFilename,
 float startTime, float stopTime, bool newLayersAreASCII, bool verifyOverLayer)
{
    clearMessages();

    OmniPvdDOMState domState;
    if (!buildPvdDomState(omniPvdFile, domState))
    {
        return false;
    }

    writeUSDFileOverWithLayerCreation(domState, inputStageFile, outputDir,
      outputStageFilename, startTime, stopTime, newLayersAreASCII, verifyOverLayer);
    return true;
}


bool loadOvd(char* omniPvdFile)
{
    clearMessages();

    OmniPvdDOMState domState;
    if (!buildPvdDomState(omniPvdFile, domState))
    {
        return false;
    }

    return true;
}

const OmniPvdMessages& getMessages()
{
    return gOmniPvdMessages;
}

void fillInterface(omni::physx::IPhysXPvd& iface)
{
    iface.ovdToUsd = ovdToUsd;
    iface.ovdToUsdOver = ovdToUsdOver;
    iface.ovdToUsdOverWithLayerCreation = ovdToUsdOverWithLayerCreation;
    iface.ovdToUsdInMemory = ovdToUsdInMemory;
    iface.loadOvd = loadOvd;
    iface.getMessages = getMessages;
    iface.clearMessages = clearMessages;
}
