// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#define CARB_EXPORTS
#include <common/utilities/Utilities.h>

#include <omni/physics/ui/IUsdPhysicsUI.h>
#include <private/omni/physics/ui/IUsdPhysicsUIPrivate.h>
#include <omni/physics/IUsdPhysicsSettings.h>

#include <private/omni/physics/IUsdPhysicsParse.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/events/EventsUtils.h>
#include <carb/tasking/ITasking.h>
#include <carb/input/IInput.h> // VP1
#include <carb/eventdispatcher/IEventDispatcher.h>

#include <carb/settings/ISettings.h>

#include <omni/ext/IExt.h>
#include <omni/ext/ExtensionsUtils.h>

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>

#include <omni/renderer/IDebugDraw.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>
#include <omni/kit/IApp.h>


#include <omni/kit/IViewport.h> // VP1
#include <omni/kit/ViewportTypes.h> // VP1
#include <omni/kit/ViewportWindowUtils.h> // VP1

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include "JointAuthoring.h"
#include "UsdPhysicsUINoticeHandler.h"

using namespace omni::physics::ui;
using namespace pxr;

const static carb::RStringKey kObserverName("omni.usdphysicsui");

struct ExtensionImpl;
ExtensionImpl* gExtensionImpl;
static constexpr char kViewportGizmoMinFadeOutPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/minFadeOut";
static constexpr char kViewportGizmoMaxFadeOutPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/maxFadeOut";

struct ExtensionImpl : public omni::ext::IExt, public pxr::TfWeakBase
{
    // Notice Listener holds pointer to the stage
    UsdPhysicsUINoticeListener mUsdNoticeListener;
    pxr::TfNotice::Key mUsdNoticeListenerKey;

    omni::timeline::TimelinePtr mTimeline = nullptr;
    carb::settings::ISettings* gSettings = nullptr;

    std::array<carb::eventdispatcher::ObserverGuard, 3> mStageEvtSub;
    carb::events::ISubscriptionPtr mTimelineEvtSub;

    omni::kit::StageUpdateNode* mStageUpdateNode = nullptr;

    omni::physics::ui::JointAuthoringManager* mJointAuthoringManager = nullptr;

    omni::kit::IViewport* mViewport = nullptr; // VP1
    carb::input::IInput* mInput = nullptr; // VP1

    uint8_t mProxySelectionGroup = 0;

    std::string mExtensionPath;

    std::vector<carb::dictionary::SubscriptionId*> mUiSettingCallbackSubscriptions;

    void tryRefreshStage()
    {
        omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
        if (!usdContext)
        {
            return;
        }

        mUsdNoticeListener.mStage = usdContext->getStage();
        if (mUsdNoticeListener.mStage)
        {
            stageOpened();
        }
    }

    void stageOpened()
    {
        CARB_PROFILE_ZONE(0, "UsdPhysicsUI stageOpened");

        mUsdNoticeListener.mStage = omni::usd::UsdContext::getContext()->getStage();
        if (mJointAuthoringManager)
        {
            mJointAuthoringManager->parseStage(mUsdNoticeListener.mStage);
        }
    }

    void stageClosed()
    {
        if (mJointAuthoringManager)
            mJointAuthoringManager->stageClosed();
        mUsdNoticeListener.mStage = nullptr;
    }


    void stageUpdate(float, float, const omni::kit::StageUpdateSettings*)
    {
        CARB_PROFILE_ZONE(0, "omni.usdphysics.ui::stageUpdate");
        if (!mUsdNoticeListener.mStage)
            return;

        if (mJointAuthoringManager)
        {
            mJointAuthoringManager->update();
        }
    }

    void onResume(float currentTime)
    {
        if (mJointAuthoringManager)
        {
            mJointAuthoringManager->setJointMeshesVisibilty(false);
        }
    }

    void onPause()
    {

    }

    void onStop()
    {
        if (mJointAuthoringManager)
        {
            const bool showJoints = gSettings->get<bool>(omni::physics::kSettingDisplayJoints);
            mJointAuthoringManager->setJointMeshesVisibilty(showJoints);
        }
    }

    void onStartup(const char* extId) override
    {
        gExtensionImpl = this;
        carb::Framework* framework = carb::getFramework();
        mInput = carb::getCachedInterface<carb::input::IInput>();
        gSettings = carb::getCachedInterface<carb::settings::ISettings>();
        mTimeline = omni::timeline::getTimeline();
        mTimelineEvtSub = carb::events::createSubscriptionToPop(
            mTimeline->getTimelineEventStream(),
            [this](carb::events::IEvent* e) {
                if (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::ePlay)
                {
                    if (mJointAuthoringManager)
                        mJointAuthoringManager->setIsPlaying(true);
                }
                else if (static_cast<omni::timeline::TimelineEventType>(e->type) ==
                         omni::timeline::TimelineEventType::eStop)
                {
                    if (mJointAuthoringManager)
                        mJointAuthoringManager->setIsPlaying(false);
                }
            },
            0, "Usd Physics UI Timeline");

        tryRefreshStage();

        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
        omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
        if (usdContext)
        {
            mStageEvtSub = {
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                 usdContext->stageEventName(omni::usd::StageEventType::eOpened),
                                 [this](const auto&) { stageOpened(); }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                 usdContext->stageEventName(omni::usd::StageEventType::eClosing),
                                 [this](const auto&) { stageClosed(); }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                 usdContext->stageEventName(omni::usd::StageEventType::eSelectionChanged),
                                 [this](const auto&) {
                                     if (!mUsdNoticeListener.mStage)
                                         stageOpened();

                                     if (mJointAuthoringManager)
                                     {
                                         mJointAuthoringManager->selectionChanged();
                                     }
                                 }),
            };
        }

        omni::kit::StageUpdatePtr stageUpdate = omni::kit::getStageUpdate();
        omni::kit::StageUpdateNodeDesc desc = { 0 };
        desc.displayName = "UsdPhysicsUI";
        desc.order = omni::kit::update::eIUsdStageUpdatePhysicsUI;
        desc.onPrimAdd = nullptr;
        desc.onPrimOrPropertyChange = nullptr;
        desc.onPrimRemove = nullptr;
        desc.onUpdate = [](float a, float b, const omni::kit::StageUpdateSettings* c, void* userData) {
            static_cast<ExtensionImpl*>(userData)->stageUpdate(a, b, c);
        };
        desc.onStop = [](void* userData) { static_cast<ExtensionImpl*>(userData)->onStop(); };
        desc.onResume = [](float currentTime, void* userData) {
            static_cast<ExtensionImpl*>(userData)->onResume(currentTime);
        };
        desc.onPause = [](void* userData) { static_cast<ExtensionImpl*>(userData)->onPause(); };
        desc.userData = this;
        if (stageUpdate)
            mStageUpdateNode = stageUpdate->createStageUpdateNode(desc);

        mViewport = framework->tryAcquireInterface<omni::kit::IViewport>();

        gSettings = carb::getCachedInterface<carb::settings::ISettings>();

        const bool showJoints = (gSettings ? gSettings->getAsBool(omni::physics::kSettingDisplayJoints) : true);

        mJointAuthoringManager = new omni::physics::ui::JointAuthoringManager(showJoints);

        subscribeToSettingsCallbacks();

        mUsdNoticeListenerKey =
            pxr::TfNotice::Register(pxr::TfCreateWeakPtr(&mUsdNoticeListener), &UsdPhysicsUINoticeListener::handle);
        mUsdNoticeListener.mJointAuthoringManager = mJointAuthoringManager;

        if (mJointAuthoringManager)
        {
            bool showJoints = gSettings->get<bool>(omni::physics::kSettingDisplayJoints);
            mJointAuthoringManager->setJointMeshesVisibilty(showJoints);
        }

        // create a shared selection group for custom outlines for session layer proxy geometry
        mProxySelectionGroup = usdContext->registerSelectionGroup();
        usdContext->setSelectionGroupOutlineColor(mProxySelectionGroup, { 1.0f, 0.6f, 0.0f, 1.0f });
        usdContext->setSelectionGroupShadeColor(mProxySelectionGroup, { 1.0f, 1.0f, 1.0f, 0.0f });


        omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>();
        mExtensionPath = omni::ext::getExtensionPath(app->getExtensionManager(), extId);
        if (mJointAuthoringManager)
        {
            mJointAuthoringManager->init(mExtensionPath);
            // parse stage on init to refresh when a stage is already opened and the extension is enabled later (or
            // reloaded)
            if (mUsdNoticeListener.mStage)
            {
                mJointAuthoringManager->parseStage(mUsdNoticeListener.mStage);
            }
        }
    }
    void unsubscribeFromSettingsCallbacks(void)
    {
        for (auto sub : mUiSettingCallbackSubscriptions)
        {
            gSettings->unsubscribeToChangeEvents(sub);
        }
        mUiSettingCallbackSubscriptions.clear();
    }
    void subscribeToSettingsCallbacks(void)
    {
        using namespace omni::physics::ui;
        unsubscribeFromSettingsCallbacks();

        auto onDisplayJointsChange = [](const carb::dictionary::Item* changedItem,
                                        carb::dictionary::ChangeEventType changeEventType, void* userData) {
            auto dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            static_cast<JointAuthoringManager*>(userData)->setJointMeshesVisibilty(dict->getAsBool(changedItem));
        };


        auto onDisplayJointsFadeOutChange = [](const carb::dictionary::Item* changedItem,
                                               carb::dictionary::ChangeEventType changeEventType, void* userData) {
            static_cast<JointAuthoringManager*>(userData)->gizmoSettingsDirty();
        };

        auto onDisplayJointBodyTransformCheckTolerance = [](const carb::dictionary::Item* changedItem,
                                                            carb::dictionary::ChangeEventType changeEventType,
                                                            void* userData) {
            auto dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            auto settings = carb::getCachedInterface<carb::settings::ISettings>();

            static float lastVal = settings->getAsFloat(omni::physics::kSettingJointBodyTransformCheckToleranceDefault);

            float tolerance = dict->getAsFloat(changedItem);
            // skip refreshes on the same value
            if (tolerance != lastVal)
            {
                lastVal = tolerance;
                static_cast<JointAuthoringManager*>(userData)->refreshAllGizmos();
            }
        };
        // Defaults
        gSettings->setDefaultBool(omni::physics::kSettingDisplayJoints, true);

        // Subscriptions
        mUiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
            omni::physics::kSettingDisplayJoints, onDisplayJointsChange, mJointAuthoringManager));
        mUiSettingCallbackSubscriptions.push_back(gSettings->subscribeToNodeChangeEvents(
            kViewportGizmoMinFadeOutPath, onDisplayJointsFadeOutChange, mJointAuthoringManager));
        mUiSettingCallbackSubscriptions.push_back(
            gSettings->subscribeToNodeChangeEvents(omni::physics::kSettingJointBodyTransformCheckTolerance,
                                                   onDisplayJointBodyTransformCheckTolerance, mJointAuthoringManager));
    }

    void drawManagers(GfMatrix4d viewMatrix,
                      GfMatrix4d projMatrix,
                      carb::Float4 viewportRect,
                      void (*enablePickingFunction)(bool enable, void* userData),
                      void* userData)
    {
        if (mJointAuthoringManager)
        {
            mJointAuthoringManager->draw(viewMatrix, projMatrix, viewportRect, mViewport != nullptr);
            if (mJointAuthoringManager->hasSomeGizmoBeenDeleted(true))
            {
                if (enablePickingFunction)
                {
                    (*enablePickingFunction)(true, userData);
                }
            }
        }
    }

    void onShutdown() override
    {
        if (mUsdNoticeListener.mStage != nullptr)
            stageClosed();

        mStageEvtSub = {};

        if (mTimelineEvtSub)
        {
            mTimelineEvtSub->unsubscribe();
            mTimelineEvtSub = nullptr;
        }

        unsubscribeFromSettingsCallbacks();
        
        omni::kit::StageUpdatePtr stageUpdate = omni::kit::getStageUpdate();
        if (mStageUpdateNode && stageUpdate)
        {
            stageUpdate->destroyStageUpdateNode(mStageUpdateNode);
            mStageUpdateNode = nullptr;
        }

        if (mJointAuthoringManager)
        {
            delete mJointAuthoringManager;
            mJointAuthoringManager = nullptr;
        }


        mUsdNoticeListener.mStage = nullptr;

        mViewport = nullptr;

        pxr::TfNotice::Revoke(mUsdNoticeListenerKey);
        gExtensionImpl = nullptr;
    }
};

const struct carb::PluginImplDesc kPluginImpl = { "omni.usdphysicsui.plugin", "UsdPhysicsUI", "NVIDIA",
                                                  carb::PluginHotReload::eEnabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, ExtensionImpl, omni::physics::ui::IUsdPhysicsUI, omni::physics::ui::IUsdPhysicsUIPrivate)
CARB_PLUGIN_IMPL_DEPS(carb::settings::ISettings,
                      omni::kit::IApp,
                      omni::kit::IStageUpdate,
                      omni::timeline::ITimeline,
                      carb::tasking::ITasking,
                      carb::input::IInput,
                      carb::dictionary::IDictionary,
                      omni::renderer::IDebugDraw,
                      omni::physics::usdparser::IUsdPhysicsParse)


void fillInterface(IUsdPhysicsUI& iface)
{
    iface.registerCustomJointAuthoring = [](IUsdPhysicsUICustomJointAuthoring& customJointAuthoring, const char* name) {
        if (gExtensionImpl)
        {
            return gExtensionImpl->mJointAuthoringManager->registerCustomJointAuthoring(customJointAuthoring, name);
        }
        return IUsdPhysicsUICustomJointAuthoring::InvalidRegistrationID;
    };

    iface.unregisterCustomJointAuthoring = [](IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID) {
        if (gExtensionImpl)
        {
            return gExtensionImpl->mJointAuthoringManager->unregisterCustomJointAuthoring(registrationID);
        }
        return false;
    };

    iface.registerCustomJointBillboard = [](IUsdPhysicsUICustomJointAuthoring::RegistrationID registrationID,
                                            const char* pngPath) -> uint32_t {
        if (gExtensionImpl)
        {
            return gExtensionImpl->mJointAuthoringManager->registerCustomJointBillboard(registrationID, pngPath);
        }
        return 0;
    };

    iface.setVisibilityFlags = [](VisibilityFlags::Enum visibilityFlags) {
        if (gExtensionImpl)
        {
            gExtensionImpl->mJointAuthoringManager->setVisibilityFlags(visibilityFlags);
        }
    };
    iface.getVisibilityFlags = []() {
        if (gExtensionImpl)
        {
            return gExtensionImpl->mJointAuthoringManager->getVisibilityFlags();
        }
        return VisibilityFlags::eHIDE_ALL;
    };

    iface.blockUsdNoticeHandler = [](bool enable) {
        if(gExtensionImpl)
        {
            gExtensionImpl->mUsdNoticeListener.blockUsdNoticeHandler(enable);
        }
    };

    iface.isUsdNoticeHandlerEnabled = []() {
        if(gExtensionImpl)
        {
            return gExtensionImpl->mUsdNoticeListener.isUsdNoticeHandlerEnabled();
        }
        return false;
    };
}


void fillInterface(IUsdPhysicsUIPrivate& iface)
{
    iface.privateDrawImmediateModeViewportOverlays =
        [](const double* view, const double* proj, float screenPositionX, float screenPositionY,
           float computedContentWidth, float computedContentHeight, float dpiScale,
           void (*enablePickingFunction)(bool enable, void* userData), void* userData) {
            if (gExtensionImpl)
            {
                carb::Float2 windowOrigin = { screenPositionX * dpiScale, screenPositionY * dpiScale };
                carb::Float2 windowSize = { computedContentWidth * dpiScale, computedContentHeight * dpiScale };
                carb::Float4 viewportRect = { windowOrigin.x, windowOrigin.y, windowOrigin.x + windowSize.x,
                                              windowOrigin.y + windowSize.y };

                pxr::GfMatrix4d viewMatrix;
                pxr::GfMatrix4d projMatrix;

                for (int row = 0; row < 4; ++row)
                {
                    for (int col = 0; col < 4; ++col)
                    {
                        viewMatrix[row][col] = view[row * 4 + col];
                        projMatrix[row][col] = proj[row * 4 + col];
                    }
                }
                gExtensionImpl->drawManagers(viewMatrix, projMatrix, viewportRect, enablePickingFunction, userData);
            }
        };
}


void fillInterface(ExtensionImpl& iface)
{
}
