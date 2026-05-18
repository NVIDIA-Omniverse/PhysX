// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

#include <cstdio>
#include <private/omni/physx/IPhysxPvd.h>
#include <omni/physx/IPhysxSettings.h>

#include "OmniPvdOvdParser.h"
#include "OmniPvdOvdParserConfig.h"
#include "OmniPvdUsdWriter.h"
#include "OmniPvdDrawGizmos.h"
#include "OmniPvdDebugDraw.h"
#include "OmniPvdUsdOverWriter.h"

extern OmniPvdMessages gOmniPvdMessages;
OmniPVDDebugVizData* gOmniPVDDebugVizData = nullptr;

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


CARB_EXPORT void carbOnPluginStartup()
{
    gOmniPVDDebugVizData = new OmniPVDDebugVizData();

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

CARB_EXPORT void carbOnPluginShutdown()
{
    unsubscribeFromSettingsCallbacks();
    cleanupDebugViz();
    clearMessages();

    delete gOmniPVDDebugVizData;
    gOmniPVDDebugVizData = nullptr;
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

////////////////////////////////////////////////////////////////////////////////
// C++ implementation of recursive visibility update for OVD prims.
// This is much faster than the Python equivalent due to native recursion.
////////////////////////////////////////////////////////////////////////////////
namespace
{
    static pxr::TfToken vizToken("omni:pvdi:viz");
    static pxr::TfToken handleToken("omni:pvdi:handle");
    
    void setVisibleChildrenRecursive(pxr::UsdPrim prim, double timeCode)
    {
        if (!prim)
            return;
        
        pxr::UsdAttribute vizAttr = prim.GetAttribute(vizToken);
        if (vizAttr)
        {
            bool isVisible = false;
            vizAttr.Get(&isVisible, timeCode);
            
            pxr::UsdGeomImageable imageable(prim);
            if (imageable)
            {
                if (isVisible)
                {
                    imageable.MakeVisible();
                    // Only recurse into children if this prim is visible
                    for (const pxr::UsdPrim& child : prim.GetChildren())
                    {
                        setVisibleChildrenRecursive(child, timeCode);
                    }
                }
                else
                {
                    imageable.MakeInvisible();
                    // Don't recurse - children inherit invisibility
                }
            }
        }
        else
        {
            // No viz attribute - just recurse into children
            for (const pxr::UsdPrim& child : prim.GetChildren())
            {
                setVisibleChildrenRecursive(child, timeCode);
            }
        }
    }
    
    ////////////////////////////////////////////////////////////////////////////////
    // Handle-to-prim cache for fast handle lookups
    // This avoids expensive Python stage traversal
    ////////////////////////////////////////////////////////////////////////////////
    struct HandleCacheEntry
    {
        pxr::SdfPath path;
        std::string name;
    };
    
    struct HandleCache
    {
        std::unordered_map<uint64_t, std::vector<HandleCacheEntry>> handleToEntries;
        bool isValid = false;
        
        void clear()
        {
            handleToEntries.clear();
            isValid = false;
        }
    };
    
    static HandleCache gHandleCache;
}

void updateOvdVisibility(double timeCode)
{
    omni::usd::UsdContext* context = omni::usd::UsdContext::getContext();
    if (!context)
        return;
    
    pxr::UsdStageRefPtr stage = context->getStage();
    if (!stage)
        return;
    
    // Use SdfChangeBlock for batched notifications (better performance)
    {
        pxr::SdfChangeBlock changeBlock;
        
        // Update visibility for /Scenes
        pxr::UsdPrim scenesPrim = stage->GetPrimAtPath(pxr::SdfPath("/Scenes"));
        if (scenesPrim)
        {
            setVisibleChildrenRecursive(scenesPrim, timeCode);
        }
        
        // Update visibility for /Shared
        pxr::UsdPrim sharedPrim = stage->GetPrimAtPath(pxr::SdfPath("/Shared"));
        if (sharedPrim)
        {
            setVisibleChildrenRecursive(sharedPrim, timeCode);
        }
    }
}

void buildHandleCache()
{
    gHandleCache.clear();
    
    omni::usd::UsdContext* context = omni::usd::UsdContext::getContext();
    if (!context)
        return;
    
    pxr::UsdStageRefPtr stage = context->getStage();
    if (!stage)
        return;
    
    // Traverse the entire stage once and cache all handle->path mappings
    for (const pxr::UsdPrim& prim : stage->TraverseAll())
    {
        pxr::UsdAttribute handleAttr = prim.GetAttribute(handleToken);
        if (handleAttr)
        {
            uint64_t handle = 0;
            if (handleAttr.Get(&handle))
            {
                HandleCacheEntry entry;
                entry.path = prim.GetPath();
                entry.name = prim.GetName();
                gHandleCache.handleToEntries[handle].push_back(entry);
            }
        }
    }
    
    gHandleCache.isValid = true;
}

void invalidateHandleCache()
{
    gHandleCache.clear();
}

void getHandlePrimNamesBatch(const uint64_t* handles, size_t numHandles, double timeCode,
                             char* outNames, bool* outIsActionable)
{
    if (!handles || !outNames || !outIsActionable || numHandles == 0)
        return;
    
    // Ensure cache is valid
    if (!gHandleCache.isValid)
    {
        buildHandleCache();
    }
    
    omni::usd::UsdContext* context = omni::usd::UsdContext::getContext();
    pxr::UsdStageRefPtr stage = context ? context->getStage() : nullptr;
    
    for (size_t i = 0; i < numHandles; ++i)
    {
        char* nameOut = outNames + (i * 256);
        nameOut[0] = '\0';
        outIsActionable[i] = false;
        
        uint64_t handle = handles[i];
        if (handle == 0)
        {
            snprintf(nameOut, 256, "%s", "NULL");
            continue;
        }
        
        auto it = gHandleCache.handleToEntries.find(handle);
        if (it == gHandleCache.handleToEntries.end() || it->second.empty() || !stage)
        {
            snprintf(nameOut, 256, "%s", "INVALID");
            continue;
        }
        
        // Find the best prim for this handle
        const HandleCacheEntry* bestEntry = nullptr;
        bool bestIsVisible = false;
        double bestLastActiveTime = -1.0;
        bool bestHasViz = false;
        
        for (const HandleCacheEntry& entry : it->second)
        {
            pxr::UsdPrim prim = stage->GetPrimAtPath(entry.path);
            if (!prim)
                continue;
            
            pxr::UsdAttribute vizAttr = prim.GetAttribute(vizToken);
            bool hasViz = vizAttr.IsValid();
            bool isVisible = false;
            double lastActiveTime = -1.0;
            
            if (hasViz)
            {
                vizAttr.Get(&isVisible, timeCode);
                
                if (!isVisible)
                {
                    std::vector<double> timeSamples;
                    vizAttr.GetTimeSamples(&timeSamples);
                    for (auto rit = timeSamples.rbegin(); rit != timeSamples.rend(); ++rit)
                    {
                        if (*rit <= timeCode)
                        {
                            bool wasVisible = false;
                            vizAttr.Get(&wasVisible, *rit);
                            if (wasVisible)
                            {
                                lastActiveTime = *rit;
                                break;
                            }
                        }
                    }
                }
            }
            
            bool useThisEntry = false;
            if (!bestEntry)
            {
                useThisEntry = true;
            }
            else if (hasViz && !bestHasViz)
            {
                useThisEntry = true;
            }
            else if (!hasViz && bestHasViz)
            {
                useThisEntry = false;
            }
            else if (hasViz && bestHasViz)
            {
                if (isVisible && !bestIsVisible)
                    useThisEntry = true;
                else if (!isVisible && bestIsVisible)
                    useThisEntry = false;
                else if (!isVisible && !bestIsVisible && lastActiveTime > bestLastActiveTime)
                    useThisEntry = true;
            }
            
            if (useThisEntry)
            {
                bestEntry = &entry;
                bestIsVisible = isVisible;
                bestLastActiveTime = lastActiveTime;
                bestHasViz = hasViz;
            }
        }
        
        if (bestEntry)
        {
            snprintf(nameOut, 256, "%s", bestEntry->name.c_str());
            outIsActionable[i] = !bestHasViz || bestIsVisible;
        }
        else
        {
            snprintf(nameOut, 256, "%s", "INVALID");
        }
    }
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
    iface.updateOvdVisibility = updateOvdVisibility;
    iface.buildHandleCache = buildHandleCache;
    iface.invalidateHandleCache = invalidateHandleCache;
    iface.getHandlePrimNamesBatch = getHandlePrimNamesBatch;
}
