// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS
#include <carb/PluginUtils.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <carb/dictionary/IDictionary.h>
#include <carb/eventdispatcher/IEventDispatcher.h>

#include <private/omni/physx/IPhysxSupportUi.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxSettings.h>

#include <PxPhysicsAPI.h>

#include <omni/ext/IExt.h>

#include <omni/renderer/IDebugDraw.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <omni/timeline/ITimeline.h>
#include <omni/timeline/TimelineTypes.h>

#include <omni/kit/EditorUsd.h>
#include <omni/kit/IApp.h>
#include <omni/kit/KitUpdateOrder.h>

#include <omni/ui/IGlyphManager.h>

#include <pxr/base/tf/ostreamMethods.h>
#include <pxr/base/tf/stringUtils.h>

#include <common/utilities/Utilities.h>

#include "AddToStageHelper.h"
#include "Utils.h"
#include "UsdUtils.h"
#include "RigidBodyManipulatorHelper.h"
#include "PhysXInspector.h"
#include "PhysXInspectorDebugVisualization.h"
#include "PhysXInspectorOverlay.h"
#include "PhysXInspectorWidget.h"

using namespace omni;
using namespace omni::physx;
using namespace pxr;

OMNI_LOG_DECLARE_CHANNEL(kSupportUiLogChannel)
OMNI_LOG_ADD_CHANNEL(kSupportUiLogChannel, "omni.physx.supportui", "PhysX Support UI")

const struct carb::PluginImplDesc kPluginImpl =
{
	"omni.physx.supportui.plugin", "PhysX Support UI", "NVIDIA", carb::PluginHotReload::eDisabled, "dev"
};

const static carb::RStringKey kObserverName("omni.physx.supportui");

class ExtensionImpl;

ExtensionImpl* gSupportUiExt = nullptr;

class SimulationEventListener final : public carb::events::IEventListener
{
public:
    SimulationEventListener(std::function<void(carb::events::IEvent* e)> eventCallback):
        mEventCallback(eventCallback)
    {
    }

    std::size_t addRef() override
    {
        return ++mRefCount;
    }

    std::size_t release() override
    {
        if (mRefCount)
        {
            --mRefCount;
        }

        return mRefCount;
    }

    void onEvent(carb::events::IEvent* e) override
    {
        if (mEventCallback)
        {
            mEventCallback(e);
        }
    }

private:
    std::size_t mRefCount{ 0 };
    std::function<void(carb::events::IEvent* e)> mEventCallback{ nullptr };
};

class ExtensionImpl final : public omni::ext::IExt, public pxr::TfWeakBase
{
public:
    ExtensionImpl()
    {
        gSupportUiExt = this;
        omni::log::addModulesChannels();
    }

    ~ExtensionImpl()
    {
        omni::log::removeModulesChannels();
        gSupportUiExt = nullptr;
    }

    void enableLogging(bool enable)
    {
        auto log = omniGetLogWithoutAcquire();

        if (enable)
        {
            log->setChannelEnabled(kSupportUiLogChannel, true, omni::log::SettingBehavior::eInherit);
        }
        else
        {
            log->setChannelEnabled(kSupportUiLogChannel, false, omni::log::SettingBehavior::eOverride);
        }
    }

    void handleUsdObjChange(const pxr::UsdNotice::ObjectsChanged& objectsChanged)
    {
        CARB_PROFILE_ZONE(0, "PhysXSupportUi handleUsdObjChange");

        if (mAvoidSceneReprocessing || mIgnoreNotificationEvents > 0 || !mStage || mStage != objectsChanged.GetStage())
        {
            return;
        }

        for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
        {
            static TfToken const kinematicsEnabledToken = pxr::UsdPhysicsTokens.Get()->physicsKinematicEnabled;

            const SdfPath primPath = path.GetPrimPath();
            const TfToken& attrToken = path.GetNameToken();

            if (kinematicsEnabledToken == attrToken)
            {
                sendKinematicsToggleChangedEvent(mEventStream, mDictionary, primPath);
            }
        }

        for (const SdfPath& path : objectsChanged.GetResyncedPaths())
        {
            if (path.IsAbsoluteRootOrPrimPath())
            {
                const SdfPath primPath = mStage->GetPseudoRoot().GetPath() == path ? mStage->GetPseudoRoot().GetPath() : path.GetPrimPath();
                const auto& changedFields = objectsChanged.GetChangedFields(primPath);
                bool newAsset = false;

                if (changedFields.size() > 0)
                {
                    // fields change should not trigger collision creation
                    OMNI_LOG_INFO(kSupportUiLogChannel, "\"%s\" - USD changes: %s", primPath.GetText(),
                                  TfStringify(changedFields).c_str());

                    // pretty weak heuristics - this can mean that this prim was newly created
                    newAsset = hasChangedToken(objectsChanged, primPath, pxr::SdfFieldKeys->Specifier);
                               //&& hasChangedToken(objectsChanged, primPath, pxr::SdfFieldKeys->TypeName);

                    if (newAsset)
                    {
                        OMNI_LOG_INFO(kSupportUiLogChannel,
                                      "\"%s\" - assuming new asset created because of changes in specifier.",
                                      primPath.GetText());
                    }
                }

                UsdPrim prim = mStage->GetPrimAtPath(primPath);

                if (prim.IsPseudoRoot())
                {
                    OMNI_LOG_INFO(kSupportUiLogChannel, "PseudoRoot reported as resynced USD path - ignoring!");
                    continue;
                }

                // Determine if the prim was added or removed.
                if (!prim.IsValid() || !prim.IsActive())
                {
                    // removal - in case it's queued for collider creation
                    mAddToStageHelper->removePrim(primPath);
                }
                else
                {
                    if (newAsset)
                    {
                        OMNI_LOG_INFO(kSupportUiLogChannel, "\"%s\" - is being added", primPath.GetText());

                        // TODO FIXME it's usually too early here for this check - remove?
                        if (omni::kit::EditorUsd::isHideInStageWindow(prim))
                        {
                            OMNI_LOG_INFO(kSupportUiLogChannel,
                                          "\"%s\" - invalid or hidden in Stage Window / DebugViz - SKIPPING",
                                          primPath.GetText());
                            continue;
                        }

                        mAddToStageHelper->primAdded(prim);
                    }
                }
            }
        }
    }

    void onStartup(const char* extId) override
    {
        mAddToStageHelper = std::make_unique<AddToStageHelper>();
        mRigidBodyManipulatorHelper = std::make_unique<RigidBodyManipulatorHelper>();

        mDictionary = carb::getCachedInterface<carb::dictionary::IDictionary>(); // Get the carbonite dictionary interface
        CARB_ASSERT(mDictionary != nullptr, "ExtensionImpl::onStartup(): Failed to obtain dictionary interface!");

        mPhysx = carb::getCachedInterface<omni::physx::IPhysx>();
        CARB_ASSERT(mPhysx != nullptr, "ExtensionImpl::onStartup(): Failed to obtain IPhysx interface!");

        mPhysicsStepSubscriptionId = mPhysx->subscribePhysicsOnStepEvents(
            false,
            0,
            [](float elapsedTime, void* userData) {
                ExtensionImpl* extension = static_cast<ExtensionImpl*>(userData);
                if(extension->mRigidBodyManipulatorHelper)
                    extension->mRigidBodyManipulatorHelper->update(elapsedTime);
                    }, this);

        mPhysxPrivate = carb::getCachedInterface<omni::physx::IPhysxPrivate>();
        CARB_ASSERT(mPhysxPrivate != nullptr, "ExtensionImpl::onStartup(): Failed to obtain IPhysxPrivate interface!");

        mSettings = carb::getCachedInterface<carb::settings::ISettings>();
        CARB_ASSERT(mSettings != nullptr, "ExtensionImpl::onStartup(): Failed to obtain ISettings interface!");

        mTimeline = omni::timeline::getTimeline();
        CARB_ASSERT(mTimeline != nullptr, "ExtensionImpl::onStartup(): Failed to obtain ITimeline interface!");

        mTimelineEvtSub = carb::events::createSubscriptionToPop(
            mTimeline->getTimelineEventStream(),
            [this](carb::events::IEvent* e) {
                if (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::ePlay)
                {
                    mRigidBodyManipulatorHelper->setPxScene(SearchForPhysxScene());
                }
                else if (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::eStop)
                {
                    // This event comes sooner than SimulationEvent::eStopped (below)
                    // Resetting PxScene at this point fixes random crashes when Manipulator accesses already freed PxScene
                    mRigidBodyManipulatorHelper->setPxScene(nullptr);
                }
            },
            0, "PhysX Support UI Timeline Event");

        mSimulationEventListener = new SimulationEventListener([this](carb::events::IEvent* e) {
            int eventType = int(e->type);

            OMNI_LOG_INFO(kSupportUiLogChannel, "Received Physx SimulationEvent %d\n", eventType);

            if (eventType == omni::physx::SimulationEvent::eResumed)
            {
                mRigidBodyManipulatorHelper->setPxScene(SearchForPhysxScene());
            }
            else if (eventType == omni::physx::SimulationEvent::eStopped)
            {
                mRigidBodyManipulatorHelper->setPxScene(nullptr);
            }
        });

        bool devMode = mSettings->getAsBool(omni::physx::kSettingPhysicsDevelopmentMode);

        auto setDefaultBool = [this](const char* setting, bool value) {
            mSettings->setDefaultBool((std::string(DEFAULT_SETTING_PREFIX) + setting).c_str(), value);
            mSettings->setDefaultBool(setting, value);
        };

        auto setDefaultInt = [this](const char* setting, int32_t value) {
            mSettings->setDefaultInt((std::string(DEFAULT_SETTING_PREFIX) + setting).c_str(), value);
            mSettings->setDefaultInt(setting, value);
        };

        setDefaultBool(kSettingsActionBarEnabled, kSettingsActionBarEnabledDefaultVal);

        setDefaultBool(kSettingsLoggingEnabled, kSettingsLoggingEnabledDefaultVal);

        setDefaultBool(kSettingsFloatingNotificationsEnabled,
                       kSettingsFloatingNotificationsEnabledDefaultVal);

        setDefaultBool(kSettingsRigidBodySelectionModeEnabled,
                       kSettingsRigidBodySelectionModeEnabledDefaultVal);

        setDefaultBool(kSettingsAutomaticColliderCreationEnabled,
                       kSettingsAutomaticColliderCreationEnabledDefaultVal);

        setDefaultBool(kSettingsAsyncCookingAtStageLoad,
                       kSettingsAsyncCookingAtStageLoadDefaultVal);

        setDefaultBool(kSettingsCustomManipulatorEnabled,
                       kSettingsCustomManipulatorEnabledDefaultVal);

        setDefaultInt(kSettingsManipModeAllowRigidBodyTraversal,
                      kSettingsManipModeAllowRigidBodyTraversalDefaultVal);

        setDefaultInt(kSettingsManipModeAllowRotWhileTranslating,
                      kSettingsManipModeAllowRotWhileTranslatingDefaultVal);

        setDefaultInt(kSettingsManipModeAllowTranOnOtherAxesWhileTranslating,
                      kSettingsManipModeAllowTranOnOtherAxesWhileTranslatingDefaultVal);

        setDefaultInt(kSettingsManipModeAllowTranWhileRotating,
                      kSettingsManipModeAllowTranWhileRotatingDefaultVal);

        setDefaultInt(kSettingsManipModeAllowRotOnOtherAxesWhileRotating,
                      kSettingsManipModeAllowRotOnOtherAxesWhileRotatingDefaultVal);

        setDefaultBool(kSettingsToolbarButtonsWithText,
                       kSettingsToolbarButtonsWithTextDefaultVal);

        setDefaultBool(kSettingsAvoidChangingExistingColliders,
                       kSettingsAvoidChangingExistingCollidersDefaultVal);

        setDefaultInt(kSettingsStaticColliderSimplificationType,
                      static_cast<int>(kSettingsStaticColliderSimplificationTypeDefaultVal));

        setDefaultInt(kSettingsDynamicColliderSimplificationType,
                      static_cast<int>(kSettingsDynamicColliderSimplificationTypeDefaultVal));
            
        enableLogging(mSettings->getAsBool(kSettingsLoggingEnabled));

        carb::dictionary::SubscriptionId* subId = mSettings->subscribeToNodeChangeEvents(
            kSettingsLoggingEnabled,
            [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
                if (eventType == carb::dictionary::ChangeEventType::eChanged)
                {
                    carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
                    if (dict != nullptr)
                    {
                        ExtensionImpl* ext = static_cast<ExtensionImpl*>(userData);
                        if (ext != nullptr)
                        {
                            ext->enableLogging(dict->getAsBool(changedItem));
                        }
                    }
                }
            },
            this);
        mSubscribedSettings.push_back(subId);

        mEventStream = carb::events::getCachedEventsInterface()->createEventStream();
        CARB_ASSERT(mEventStream, "ExtensionImpl: Failed to create event stream!");

        mAddToStageHelper->setEventStream(mEventStream);

        // try to grab currently active stage - it is a must if a user enabled this plugin in the middle of work
        mStage = omni::usd::UsdContext::getContext()->getStage();
        if (mStage != nullptr)
        {
            mAddToStageHelper->setStage(mStage);
            mRigidBodyManipulatorHelper->setStage(mStage);
        }

        mUsdNoticeListenerKey = pxr::TfNotice::Register(pxr::TfCreateWeakPtr(this), &ExtensionImpl::handleUsdObjChange);

        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
        omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
        if (usdContext)
        {
            mStageEvtSub = {
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                    usdContext->stageEventName(omni::usd::StageEventType::eOpened),
                    [this](const auto&) {
                        mStage = omni::usd::UsdContext::getContext()->getStage();
                        mAddToStageHelper->setStage(mStage);
                        mRigidBodyManipulatorHelper->setStage(mStage);
                }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                    usdContext->stageEventName(omni::usd::StageEventType::eClosing),
                    [this](const auto&) {
                        mStage = nullptr;
                        mAddToStageHelper->setStage(mStage);
                        mRigidBodyManipulatorHelper->setStage(mStage);
                        mRigidBodyManipulatorHelper->setPxScene(nullptr);
                    }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                    usdContext->stageEventName(omni::usd::StageEventType::eSaved),
                    [this](const auto&) {
                        // Avoid Scene Reprocessing after "save as" or while first saving a
                        // new scene (when USD transfer happens)
                        mAvoidSceneReprocessing = true;
                    }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                    usdContext->stageEventName(omni::usd::StageEventType::eAssetsLoaded),
                    [this](const auto&) {
                        // This seems to always come after save.
                        // In case it does not it will come again later while working
                        mAvoidSceneReprocessing = false;
                    })
            };
        }

        omni::kit::IApp* app = carb::getCachedInterface<omni::kit::IApp>();
        if (app)
        {
            mUpdateEvtSub =
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder, omni::kit::kGlobalEventUpdate,
                    [this](const carb::eventdispatcher::Event& e) {
                        mIgnoreNotificationEvents++;
                        mAddToStageHelper->update();
                        mIgnoreNotificationEvents--;

                        mEventStream->pump();
                     }
                );
        }

        mSimEvtSub = mPhysx->getSimulationEventStreamV2()->createSubscriptionToPop(
            mSimulationEventListener, 0, "PhysX Support UI Simulation Event");

        mPhysXInspector.onStartup();
        mPhysXInspectorDebugVisualization.onStartup(&mPhysXInspector);
    }

    void onShutdown() override
    {
        CARB_ASSERT(mIgnoreNotificationEvents == 0, "mIgnoreNotificationEvents: Unbalanced calls");
        mIgnoreNotificationEvents = 0;

        mAvoidSceneReprocessing = false;
        
        for (const auto subId : mSubscribedSettings)
        {
            mSettings->unsubscribeToChangeEvents(subId);
        }

        mSubscribedSettings.clear();

        mPhysx->unsubscribePhysicsOnStepEvents(mPhysicsStepSubscriptionId);

        pxr::TfNotice::Revoke(mUsdNoticeListenerKey);

        mSimEvtSub = nullptr; // calls release on mSimulationEventListener (--refCount)

        if (mTimelineEvtSub)
        {
            mTimelineEvtSub->unsubscribe();
            mTimelineEvtSub = nullptr;
        }

        mStageEvtSub = {};
        mUpdateEvtSub.reset();

        mStage = nullptr;
        mDictionary = nullptr;
        mEventStream = nullptr;

        if (mSimulationEventListener != nullptr)
        {
            delete mSimulationEventListener; 
            mSimulationEventListener = nullptr;
        }

        mAddToStageHelper->setStage(mStage);
        mAddToStageHelper->setEventStream(mEventStream);
        mAddToStageHelper.reset();

        mRigidBodyManipulatorHelper->setStage(mStage);
        mRigidBodyManipulatorHelper.reset();

        mPhysXInspector.onShutdown();
        mPhysXInspectorDebugVisualization.onShutdown();

        mSettings = nullptr;
        mPhysxPrivate = nullptr;
        mPhysx = nullptr;
    }

    ::physx::PxScene* SearchForPhysxScene()
    {
        ::physx::PxScene* physxScene = nullptr;

        if (mPhysxPrivate != nullptr)
        {
            physxScene = mPhysxPrivate->getPhysXScene();
        }

        return physxScene;
    }

    bool createColliders(const pxr::SdfPath& primPath, IPhysxSupportUi::ColliderType colliderType)
    {
        if (mStage == nullptr)
        {
            return false;
        }

        UsdPrim prim = mStage->GetPrimAtPath(primPath);

        if (prim)
        {
            mAddToStageHelper->requestColliderCreation(prim, true, colliderType);

            return true;
        }

        return false;
    }

    int getNumOfCollidersToCreate() const
    {
        return static_cast<int>(mAddToStageHelper->numQueuedCollidersToCreate());
    }

    void clearCollidersProcessingQueue()
    {
        mAddToStageHelper->clearPrimQueue();
    }

    bool rigidBodyManipulationMove(const pxr::SdfPath& primPath,
                                   const carb::Float3& deltaTranslation,
                                   bool lockRotation,
                                   bool lockTranslation)
    {
        return mRigidBodyManipulatorHelper->move(primPath, deltaTranslation, lockRotation, lockTranslation);
    }

    bool rigidBodyManipulationRotate(const pxr::SdfPath& primPath,
                                     const carb::Float3& pivotWorldPos,
                                     const carb::Float4& deltaRotation,
                                     bool lockRotation,
                                     bool lockTranslation)
    {
        return mRigidBodyManipulatorHelper->rotate(primPath, pivotWorldPos, deltaRotation, lockRotation, lockTranslation);
    }

    void rigidBodyManipulationBegan(const pxr::SdfPath& primPath)
    {
        mRigidBodyManipulatorHelper->setPxScene(SearchForPhysxScene());
        mRigidBodyManipulatorHelper->onManipulationBegan(primPath);
    }

    void rigidBodyManipulationEnded(const pxr::SdfPath& primPath)
    {
        mRigidBodyManipulatorHelper->onManipulationEnded(primPath);
    }

    std::shared_ptr<PhysXInspectorModel> createPhysXInspectorModel(const pxr::SdfPathVector& selection)
    {
        return mPhysXInspector.createPhysXInspectorModel(selection);
    }

    std::shared_ptr<omni::ui::scene::PhysXInspectorOverlay> createPhysXInspectorOverlay()
    {
        auto newOverlay = omni::ui::scene::PhysXInspectorOverlayImpl::create(&mPhysXInspector);
        mPhysXInspectorDebugVisualization.registerOverlay(newOverlay);
        return newOverlay;
    }

    std::shared_ptr<omni::ui::PhysXInspectorWidget> createPhysXInspectorWidget()
    {
        return omni::ui::PhysXInspectorWidgetImpl::create(&mPhysXInspector);
    }

    carb::events::IEventStreamPtr getEventStream() const
    {
        return mEventStream;
    }
    PhysXInspector& getInspector() { return mPhysXInspector; }

private:
    omni::physx::IPhysx* mPhysx{ nullptr };
    omni::physx::IPhysxPrivate* mPhysxPrivate{ nullptr };
    carb::settings::ISettings* mSettings{ nullptr };
    std::vector<carb::dictionary::SubscriptionId*> mSubscribedSettings;
    UsdStageWeakPtr mStage;
    omni::physx::SubscriptionId mPhysicsStepSubscriptionId;
    omni::timeline::TimelinePtr mTimeline{ nullptr };
    carb::events::ISubscriptionPtr mTimelineEvtSub;
    bool mAvoidSceneReprocessing{ false };
    int mIgnoreNotificationEvents{ 0 };
    pxr::TfNotice::Key mUsdNoticeListenerKey;
    SimulationEventListener* mSimulationEventListener{ nullptr };
    carb::events::ISubscriptionPtr mSimEvtSub;
    std::array<carb::eventdispatcher::ObserverGuard, 4> mStageEvtSub;
    carb::eventdispatcher::ObserverGuard mUpdateEvtSub;
    carb::events::IEventStreamPtr mEventStream;
    carb::dictionary::IDictionary* mDictionary{ nullptr };
    std::unique_ptr<AddToStageHelper> mAddToStageHelper;
    std::unique_ptr<RigidBodyManipulatorHelper> mRigidBodyManipulatorHelper;
    PhysXInspector mPhysXInspector;
    PhysXInspectorDebugVisualization mPhysXInspectorDebugVisualization;
};

CARB_PLUGIN_IMPL(kPluginImpl,
                 ExtensionImpl,
                 omni::physx::IPhysxSupportUi,
                 omni::physx::IPhysxSupportUiRigidBodyManipulator,
                 omni::physx::IPhysxSupportUiPrivate)

CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysx,
                      omni::physx::IPhysxPrivate,
                      omni::physx::usdparser::IPhysxUsdLoad,
                      omni::physx::IPhysxSimulation,
                      omni::physx::IPhysxCooking,
                      omni::physx::IPhysxCookingPrivate,
                      carb::tasking::ITasking,
                      carb::settings::ISettings,
                      omni::kit::IStageUpdate,
                      omni::kit::IApp,
                      carb::dictionary::IDictionary,
                      omni::renderer::IDebugDraw,
                      omni::ui::IGlyphManager)

std::shared_ptr<PhysXInspectorModel> createPhysXInspectorModel(const pxr::SdfPathVector& selection)
{
    if (gSupportUiExt == nullptr)
    {
        return nullptr;
    }
    return gSupportUiExt->createPhysXInspectorModel(selection);
}

std::shared_ptr<omni::ui::scene::PhysXInspectorOverlay> createPhysXInspectorOverlay()
{
    if (gSupportUiExt == nullptr)
    {
        return nullptr;
    }
    return gSupportUiExt->createPhysXInspectorOverlay();
}

std::shared_ptr<omni::ui::PhysXInspectorWidget> createPhysXInspectorWidget()
{
    if (gSupportUiExt == nullptr)
    {
        return nullptr;
    }
    return gSupportUiExt->createPhysXInspectorWidget();
}

void fillInterface(omni::physx::IPhysxSupportUi& iface)
{
    iface.createColliders = [](uint64_t primPath, IPhysxSupportUi::ColliderType colliderType) {
        if (gSupportUiExt != nullptr)
        {
            return gSupportUiExt->createColliders(intToPath(primPath), colliderType);
        }

        return false;
    };

    iface.getNumOfCollidersToCreate = []() {
        if (gSupportUiExt != nullptr)
        {
            return gSupportUiExt->getNumOfCollidersToCreate();
        }

        return 0;
    };

    iface.clearCollidersProcessingQueue = []() {
        if (gSupportUiExt != nullptr)
        {
            return gSupportUiExt->clearCollidersProcessingQueue();
        }
    };

    iface.getEventStream = []() {
        if (gSupportUiExt == nullptr)
        {
            return carb::events::IEventStreamPtr();
        }

        return gSupportUiExt->getEventStream();
    };
}

void fillInterface(omni::physx::IPhysxSupportUiRigidBodyManipulator& iface)
{
    iface.move = [](uint64_t primPath, const carb::Float3& deltaTran, bool lockRot, bool lockTran) {
        if (gSupportUiExt != nullptr)
        {
            return gSupportUiExt->rigidBodyManipulationMove(intToPath(primPath), deltaTran, lockRot, lockTran);
        }

        return false;
    };

    iface.rotate = [](uint64_t primPath, const carb::Float3& pivotWorldPos, const carb::Float4& deltaRot, bool lockRot, bool lockTran) {
        if (gSupportUiExt != nullptr)
        {
            return gSupportUiExt->rigidBodyManipulationRotate(intToPath(primPath), pivotWorldPos, deltaRot, lockRot, lockTran);
        }

        return false;
    };

    iface.manipulationBegan = [](uint64_t primPath) {
        if (gSupportUiExt != nullptr)
        {
            gSupportUiExt->rigidBodyManipulationBegan(intToPath(primPath));
        }
    };

    iface.manipulationEnded = [](uint64_t primPath) {
        if (gSupportUiExt != nullptr)
        {
            gSupportUiExt->rigidBodyManipulationEnded(intToPath(primPath));
        }
    };
}

void fillInterface(omni::physx::IPhysxSupportUiPrivate& iface)
{
    iface.createPhysXInspectorModel = createPhysXInspectorModel;
    iface.createPhysXInspectorOverlay = createPhysXInspectorOverlay;
    iface.createPhysXInspectorWidget = createPhysXInspectorWidget;
    iface.getPhysXInspectorEventStream = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().getEventStream();
        else
            return carb::events::IEventStreamPtr{};
    };
    iface.getPhysXInspectorState = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().getState();
        else
            return PhysXInspectorModel::State::eDisabled;
    };
    iface.stepPhysXInspectorSimulation = [] (float dt) {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().stepIfNecessary(dt);
    };
    iface.resetPhysXInspectorToAuthoringStart = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().resetToAuthoringStart();
    };
    iface.enablePhysXInspectorAuthoringMode = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().enableAuthoringMode();
    };
    iface.enableNoticeHandler = [] (bool enable) {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().enableNoticeHandler(enable);
    };
    iface.refreshAllInspectorModelsStructure = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().refreshAllInspectorModelsStructure();
    };
    iface.refreshAllInspectorModelsValues = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().refreshAllInspectorModelsValues();
    };
    iface.commitPhysXInspectorAuthoringState = [] () {
        if (gSupportUiExt)
            return gSupportUiExt->getInspector().commitAuthoringState();
    };
}

void fillInterface(ExtensionImpl& iface)
{
}
