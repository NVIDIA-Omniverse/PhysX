// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
// clang-format off
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <omni/kit/EditorUsd.h>
#include <PxPhysicsAPI.h>

#include <carb/Framework.h>
#include <carb/cpp/StringView.h>

#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <private/omni/physx/IPhysxUsdLoad.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxSupportUi.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxSupportUiPrivate.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <omni/kit/KitUpdateOrder.h>
#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/usd/Selection.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/OmniPhysXUtilities.h>

#include "LayerUtils.h"
#include "PhysXInspector.h"
#include "PhysXInspectorModel.h"

const static carb::RStringKey kObserverName("omni.physx.supportui:inspector");

namespace
{
struct ScopedReleaseEventIgnore
{
    explicit ScopedReleaseEventIgnore(uint32_t& ignoreCount) : mIgnoreCount(ignoreCount)
    {
        ++mIgnoreCount;
    }

    ~ScopedReleaseEventIgnore()
    {
        --mIgnoreCount;
    }

    uint32_t& mIgnoreCount;
};
} // namespace
// clang-format on

void PhysXInspector::onStartup()
{
    mEnableGravity = omni::ui::SimpleBoolModel::create(false);
    mEnableQuasiStaticMode = omni::ui::SimpleBoolModel::create(true);

    mEnableGravity->addValueChangedFn([this](auto) { enableGravity(mEnableGravity->getValueAsBool()); });
    mEnableQuasiStaticMode->addValueChangedFn(
        [this](auto) { enableQuasiStaticMode(mEnableQuasiStaticMode->getValueAsBool()); });

    auto setDefaultBool = [this](const char* setting, bool value) {
        mISettings->setDefaultBool((std::string(DEFAULT_SETTING_PREFIX) + setting).c_str(), value);
        mISettings->setDefaultBool(setting, value);
    };
    mStream = carb::events::getCachedEventsInterface()->createEventStream();
    mStage = omni::usd::UsdContext::getContext()->getStage();
    mISettings = carb::getCachedInterface<carb::settings::ISettings>();
    setDefaultBool(
        omni::physx::kSettingsPhysicsInspectorEnabled, omni::physx::kSettingsPhysicsInspectorEnabledDefaultVal);

    mPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
    mPhysXPrivateInterface = carb::getCachedInterface<omni::physx::IPhysxPrivate>();

    mPhysicsInspectorEnabledSubID = mISettings->subscribeToNodeChangeEvents(
        omni::physx::kSettingsPhysicsInspectorEnabled,
        [](const carb::dictionary::Item* changedItem, carb::dictionary::ChangeEventType eventType, void* userData) {
            PhysXInspector* self = (PhysXInspector*)userData;
            carb::dictionary::IDictionary* dict = carb::getCachedInterface<carb::dictionary::IDictionary>();
            if (dict->getAsBool(changedItem))
            {
                if (self->mState == PhysXInspectorModel::State::eDisabled)
                {
                    self->enableAuthoringMode();
                }
            }
            else
            {
                if (self->mState == PhysXInspectorModel::State::eAuthoring)
                {
                    self->stopAuthoringSimulation();
                }
                self->disableAuthoringMode();
            }
        },
        this);

    auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
    omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
    if (usdContext)
    {
        mStageEvtSub = {
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eOpened),
                             [this](const auto&) { mStage = omni::usd::UsdContext::getContext()->getStage(); }),
            ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                             usdContext->stageEventName(omni::usd::StageEventType::eClosing),
                             [this](const auto&) {
                                 disableAuthoringMode();
                                 deleteSessionSublayer();
                                 mISettings->setBool(omni::physx::kSettingsPhysicsInspectorEnabled, false);
                                 mStage = nullptr;
                             }),
        };
    }

    mUsdNoticeListenerKey = PXR_NS::TfNotice::Register(PXR_NS::TfCreateWeakPtr(this), &PhysXInspector::handleUsdObjChange);


    omni::kit::StageUpdatePtr stageUpdate = omni::kit::getStageUpdate();
    omni::kit::StageUpdateNodeDesc desc = { 0 };
    desc.onPrimAdd = nullptr;
    desc.onPrimOrPropertyChange = nullptr;
    desc.onPrimRemove = nullptr;
    desc.userData = this;

    desc.displayName = "PhysxInspector Before Update Physics";
    desc.order = omni::kit::update::eIUsdStageUpdatePhysics - 1;
    desc.onResume = [](float currentTime, void* userData) { ((PhysXInspector*)userData)->onSimResume(currentTime); };
    if (stageUpdate)
        mStageUpdateNodeBefore = stageUpdate->createStageUpdateNode(desc);

    desc.displayName = "PhysxInspector After Update Physics";
    desc.order = omni::kit::update::eIUsdStageUpdatePhysics + 1;
    desc.onResume = nullptr;
    desc.onStop = [](void* userData) { ((PhysXInspector*)userData)->onSimStop(); };
    desc.onUpdate = [](float currentTime, float elapsedTime, const omni::kit::StageUpdateSettings* settings,
                       void* userData) { ((PhysXInspector*)userData)->onSimUpdate(); };
    if (stageUpdate)
        mStageUpdateNodeAfter = stageUpdate->createStageUpdateNode(desc);
}

void PhysXInspector::disableAuthoringMode()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::disableAuthoringMode");
    if (mState == PhysXInspectorModel::State::eRunningSimulation)
    {
        mState = PhysXInspectorModel::State::eDisabled;
        return;
    }

    // Clearing session is faster than dropping the entire authoring sublayer
    clearSessionSublayer();
    setState(PhysXInspectorModel::State::eDisabled);
}

void PhysXInspector::enableAuthoringMode()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::enableAuthoringMode");
    applyChangeSelection();
    setState(PhysXInspectorModel::State::eAuthoring);
}

void PhysXInspector::onShutdown()
{
    mEnableGravity = nullptr;
    mEnableQuasiStaticMode = nullptr;

    mISettings->unsubscribeToChangeEvents(mPhysicsInspectorEnabledSubID);
    carb::Framework* framework = carb::getFramework();
    mStageEvtSub = {};
    if (mState == PhysXInspectorModel::State::eAuthoring)
    {
        stopAuthoringSimulation();
        disableAuthoringMode();
    }
    PXR_NS::TfNotice::Revoke(mUsdNoticeListenerKey);
    omni::kit::StageUpdatePtr stageUpdate = omni::kit::getStageUpdate();
    if (mStageUpdateNodeBefore && stageUpdate)
    {
        stageUpdate->destroyStageUpdateNode(mStageUpdateNodeBefore);
        mStageUpdateNodeBefore = nullptr;
    }
    if (mStageUpdateNodeAfter && stageUpdate)
    {
        stageUpdate->destroyStageUpdateNode(mStageUpdateNodeAfter);
        mStageUpdateNodeAfter = nullptr;
    }
    deleteSessionSublayer();
    mInspectorModels.clear();
    mStage = nullptr;
    mStream = nullptr;
}

void PhysXInspector::setState(PhysXInspectorModel::State newState)
{
    const bool stateChanged = mState != newState;
    mState = newState;
    auto event = carb::stealObject(mStream->createEventPtr(
        static_cast<carb::events::EventType>(newState), carb::events::kGlobalSenderId, std::make_pair("a", true)));
    mStream->push(event.get());
    mStream->pump();
    if (stateChanged)
    {
        mCmdUpdateModels = true;
    }
}

PhysXInspectorModel::State PhysXInspector::getState() const
{
    return mState;
}

std::shared_ptr<PhysXInspectorModel> PhysXInspector::createPhysXInspectorModel(const PXR_NS::SdfPathVector& selection)
{
    auto model = std::make_shared<PhysXInspectorModelImpl>(selection, this);
    mInspectorModels.push_back(model);
    onModelCreated(model.get());
    return model;
}

void PhysXInspector::onModelCreated(PhysXInspectorModelImpl* model)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::onModelCreated");
    if (mState != PhysXInspectorModel::State::eDisabled)
    {
        applyChangeSelection();
    }
}

void PhysXInspector::onModelDestroyed(PhysXInspectorModelImpl* model)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::onModelDestroyed");
    if (mState != PhysXInspectorModel::State::eDisabled)
    {
        applyChangeSelection();
    }
}

void PhysXInspector::refreshAllInspectorModelsStructure()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::refreshAllInspectorModelsStructure");
    for (auto it = mInspectorModels.begin(); it != mInspectorModels.end();)
    {
        auto pointer = it->lock();
        if (pointer)
        {
            auto casted = std::static_pointer_cast<PhysXInspectorModelImpl>(pointer);

            casted->refreshModelStructure();
            ++it;
        }
        else
        {
            it = mInspectorModels.erase(it);
        }
    }
}

void PhysXInspector::refreshAllInspectorModelsOverrides()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::refreshAllInspectorModelsOverrides");
    for (auto it = mInspectorModels.begin(); it != mInspectorModels.end();)
    {
        auto pointer = it->lock();
        if (pointer)
        {
            auto casted = std::static_pointer_cast<PhysXInspectorModelImpl>(pointer);

            casted->refreshModelOverrides();
            ++it;
        }
        else
        {
            it = mInspectorModels.erase(it);
        }
    }
}


void PhysXInspector::refreshAllInspectorModelsValues()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::refreshAllInspectorModelsValues");
    for (auto it = mInspectorModels.begin(); it != mInspectorModels.end();)
    {
        auto pointer = it->lock();
        if (pointer)
        {
            auto casted = std::static_pointer_cast<PhysXInspectorModelImpl>(pointer);
            casted->refreshModelValues();
            ++it;
        }
        else
        {
            it = mInspectorModels.erase(it);
        }
    }
}

void PhysXInspector::handlePhysicsObjectsReleased()
{
    if (mIgnorePhysicsObjectsReleased > 0)
    {
        return;
    }

    if (mState == PhysXInspectorModel::State::eAuthoring)
    {
        enableNoticeHandler(false);
        stopAuthoringSimulation();
        disableAuthoringMode();
        mCmdCommitAndDisable = false;
        enableNoticeHandler(true);
        setState(PhysXInspectorModel::State::eRunningSimulation);
    }
}


void PhysXInspector::onSimStop()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::onSimStop");
    if (mISettings->getAsBool(omni::physx::kSettingsPhysicsInspectorEnabled))
    {
        if (mState == PhysXInspectorModel::State::eRunningSimulation)
        {
            // Automatically re-enable authoring when exiting simulation
            // Manually set the state so that applyChangeSelection will create session ojbects
            // Calling setState later so that the "authoring" mode event is sent after updating the models,
            // and this allows to restore proper joint selection state on python side
            mState = PhysXInspectorModel::State::eAuthoring;
            applyChangeSelection();
            setState(PhysXInspectorModel::State::eAuthoring);
        }
    }
    else
    {
        mState = PhysXInspectorModel::State::eDisabled;
    }
}

void PhysXInspector::onSimResume(float currentTime)
{
    if (mISettings->getAsBool(omni::physx::kSettingsPhysicsInspectorEnabled))
    {
        CARB_PROFILE_ZONE(0, "PhysXInspector::onSimResume");
        if (mState == PhysXInspectorModel::State::eAuthoring)
        {
            stopAuthoringSimulation();
            disableAuthoringMode();
        }
        setState(PhysXInspectorModel::State::eRunningSimulation);
    }
    else
    {
        mState = PhysXInspectorModel::State::eRunningSimulation;
    }
}

void PhysXInspector::onSimUpdate()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::onSimUpdate");
    if (mCmdCommitAndDisable)
    {
        mCmdCommitAndDisable = false;
        enableNoticeHandler(false);
        stopAuthoringSimulation();
        disableAuthoringMode();
        enableNoticeHandler(true);
    }

    if (mCmdUpdateModels)
    {
        mCmdUpdateModels = false;
        refreshAllInspectorModelsValues();
    }

    if (mCmdZeroStep)
    {
        stepIfNecessary(0.0f);
    }
}

void PhysXInspector::stopAuthoringSimulation()
{
    ScopedReleaseEventIgnore ignoreReleaseEvents(mIgnorePhysicsObjectsReleased);
    const bool usdResetOnStop = mISettings->getAsBool(omni::physx::kSettingResetOnStop);
    mISettings->setBool(omni::physx::kSettingResetOnStop, true);
    mPhysXInterface->resetSimulation();
    mISettings->setBool(omni::physx::kSettingResetOnStop, usdResetOnStop);
}

void PhysXInspector::resetToAuthoringStart()
{
    if (mState == PhysXInspectorModel::State::eAuthoring)
    {
        enableNoticeHandler(false);
        stopAuthoringSimulation();
        startSimulation();
        refreshAllInspectorModelsOverrides();
        enableNoticeHandler(true);
    }
}

void PhysXInspector::commitAuthoringState()
{
    if (mState == PhysXInspectorModel::State::eAuthoring)
    {
        ScopedReleaseEventIgnore ignoreReleaseEvents(mIgnorePhysicsObjectsReleased);
        // We need to explicitly save authoring state or it will get reset by next simulation start
        const bool usdResetOnStop = mISettings->getAsBool(omni::physx::kSettingResetOnStop);
        mISettings->setBool(omni::physx::kSettingResetOnStop, false);
        mPhysXInterface->resetSimulation();
        mISettings->setBool(omni::physx::kSettingResetOnStop, usdResetOnStop);
        applyChangeSelection();
    }
}

bool PhysXInspector::checkIfShouldStopAuthoring(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& attrName)
{
    static PXR_NS::TfToken visToken("visibility");
    if (attrName == visToken)
    {
        return false;
    }

    // Upper and lower limit for Revolute Joint require a simulation restart if they're changed from default
    if (attrName == PXR_NS::UsdPhysicsTokens.Get()->physicsUpperLimit ||
        attrName == PXR_NS::UsdPhysicsTokens.Get()->physicsLowerLimit)
    {
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);
        if (prim.IsA<PXR_NS::UsdPhysicsRevoluteJoint>())
        {
            auto* joint = static_cast<physx::PxArticulationJointReducedCoordinate*>(
                mPhysXInterface->getPhysXPtr(primPath, omni::physx::ePTLinkJoint));

            if (joint)
            {
                float limitValue = 0;
                prim.GetAttribute(attrName).Get(&limitValue);

                if (isinf(limitValue))
                {
                    return joint->getJointType() == physx::PxArticulationJointType::eREVOLUTE_UNWRAPPED;
                }
                else
                {
                    return joint->getJointType() == physx::PxArticulationJointType::eREVOLUTE;
                }
            }
        }
    }

    // Check if the token is a physics token and if so, we don't restart the simulation
    const auto& allPhysicsTokens = PXR_NS::UsdPhysicsTokens.Get()->allTokens;
    auto found = std::find(allPhysicsTokens.begin(), allPhysicsTokens.end(), attrName);
    bool isFound = found != allPhysicsTokens.end();
    if (!isFound)
    {
        const auto& allPhysXTokens = PXR_NS::PhysxSchemaTokens.Get()->allTokens;
        found = std::find(allPhysXTokens.begin(), allPhysXTokens.end(), attrName);
        isFound = found != allPhysXTokens.end();
    }
    if (!isFound)
    {
        std::string attributeText = attrName.GetText();
        carb::cpp::string_view attributeName = attributeText;
        isFound = attributeName.contains(":physics:") || attributeName.contains(":physxLimit:");
    }

    if (isFound)
    {
        std::string attributeText = attrName.GetText();
        carb::cpp::string_view attributeName = attributeText;
        // Mass properties and others are updated on next omni.physx "update" cycle,
        // so we issue a 0.0f delta time simulation step, that will run that code
        // Other properties like joint state needs to avoid this step because it will
        // copy values from simulation to USD making overwriting them
        if (!attributeName.contains(":angular:") && !attributeName.contains(":prismatic:"))
        {
            mCmdZeroStep = true;
        }
        return false;
    }
    return true;
}

bool PhysXInspector::checkIfShouldUpdateModels(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& attrName)
{
    const std::string& attrString = attrName.GetString();
    if (attrString.find(":targetPosition") != std::string::npos || attrString.find(":position") != std::string::npos ||
        attrString.find(":lowerLimit") != std::string::npos || attrString.find(":upperLimit") != std::string::npos)
    {
        return true;
    }
    return false;
}

void PhysXInspector::enableNoticeHandler(bool enable)
{
    mSilenceUsdChangeEvents = !enable;
}

void PhysXInspector::handleUsdObjChange(const class PXR_NS::UsdNotice::ObjectsChanged& objectsChanged)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::handleUsdObjChange");
    PXR_NAMESPACE_USING_DIRECTIVE;
    if (mSilenceUsdChangeEvents || (mState != PhysXInspectorModel::State::eAuthoring) || !mStage)
    {
        return;
    }

    bool shouldStopAuthoring = false;
    for (const SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        if (path.IsAbsoluteRootOrPrimPath())
        {
            const SdfPath primPath =
                mStage->GetPseudoRoot().GetPath() == path ? mStage->GetPseudoRoot().GetPath() : path.GetPrimPath();
            PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);
            if (prim)
            {
                if (omni::kit::EditorUsd::isHideInStageWindow(prim))
                {
                    return;
                }
                else
                {
                    shouldStopAuthoring = true;
                }
            }
            else
            {
                shouldStopAuthoring = true;
                break; // deleted prim
            }
        }
    }


    for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
    {
        const SdfPath primPath = mStage->GetPseudoRoot().GetPath() == path ? path : path.GetPrimPath();
        const bool isAttributePath = path.IsPropertyPath();
        if (objectsChanged.HasChangedFields(primPath))
        {
            const PXR_NS::TfTokenVector changedFieldTokens = objectsChanged.GetChangedFields(primPath);
            for (const TfToken& f : changedFieldTokens)
            {
                if (checkIfShouldStopAuthoring(primPath, f))
                {
                    shouldStopAuthoring = true;
                    break;
                }
            }
        }

        if (isAttributePath)
        {
            TfToken const& attrName = path.GetNameToken();
            if (checkIfShouldStopAuthoring(primPath, attrName))
            {
                PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(primPath);
                if(prim)
                {
                    if (prim.IsActive() && omni::kit::EditorUsd::isHideInStageWindow(prim))
                    {
                        // Skip this case, we we have been getting spurious scale attribute changes from realsense cache
                        continue;
                    }
                    else if (!prim.IsA<UsdGeomCamera>())
                    {
                        shouldStopAuthoring = true;
                        break;
                    }
                }
                else
                {
                    shouldStopAuthoring = true;
                    break; // deleted prim
                }
            }
            if (checkIfShouldUpdateModels(primPath, attrName))
            {
                mCmdUpdateModels = true;
            }
        }
    }
    if (shouldStopAuthoring)
    {
        // We can't commit and disable from here, because we will get USD threading violation when this notice
        // handler is called by some code that is using an SDFChangeBlock (for example when deleting prims).
        // That's why we just schedule a flag to commit and disable on next sim update frame
        mCmdCommitAndDisable = true;
    }
}

bool PhysXInspector::findParentArticulationRoot(const std::string& usdPath, std::string& outputPath)
{
    // scristiano: TODO improve looking for articulation root by inspecting joints/bodies
    const PXR_NS::SdfPath path(usdPath);
    PXR_NS::UsdPrim pseudoRoot = mStage->GetPseudoRoot();
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;
    do
    {
        if (prim.HasAPI<PXR_NS::UsdPhysicsArticulationRootAPI>())
        {
            const PXR_NS::PhysxSchemaPhysxArticulationAPI physxArticulationAPI =
                PXR_NS::PhysxSchemaPhysxArticulationAPI::Get(mStage, prim.GetPath());

            if (physxArticulationAPI)
            {
                bool enabled = false;
                physxArticulationAPI.GetArticulationEnabledAttr().Get(&enabled);
                if (enabled)
                {
                    outputPath = prim.GetPath().GetText();
                    return true;
                }
            }
            else
            {
                outputPath = prim.GetPath().GetText();
                return true;
            }
        }
        prim = prim.GetParent();
    } while (prim != pseudoRoot);
    return false;
}

void PhysXInspector::createNewSessionScene()
{
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    PXR_NS::SdfJustCreatePrimInLayer(mSessionSubLayer, mSelectedPhysicsScenePath);
    PXR_NS::UsdPhysicsScene scene = PXR_NS::UsdPhysicsScene::Define(mStage, mSelectedPhysicsScenePath);
    PXR_NS::UsdPrim scenePrim = scene.GetPrim();
    PXR_NS::PhysxSchemaPhysxSceneAPI physxSceneAPI = PXR_NS::PhysxSchemaPhysxSceneAPI::Apply(scenePrim);
    PXR_NS::PhysxSchemaPhysxSceneQuasistaticAPI quasistaticAPI =
        PXR_NS::PhysxSchemaPhysxSceneQuasistaticAPI::Apply(scene.GetPrim());

    // cannot use omni::kit::EditorUsd::setHideInStageWindow() because it is scoping
    // session layer inside, overriding our inserted session sublayer
    //
    // omni::kit::EditorUsd::setHideInStageWindow(scenePrim, true);
    // omni::kit::EditorUsd::setNoDelete(scenePrim, true);

    scenePrim.SetMetadata(PXR_NS::TfToken("hide_in_stage_window"), true);
    scenePrim.SetMetadata(PXR_NS::TfToken("no_delete"), true);
}

void PhysXInspector::enableGravity(bool enable)
{
    if (mSelectedPhysicsScenePath.IsEmpty() || mState == PhysXInspectorModel::State::eRunningSimulation)
        return;
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    PXR_NS::SdfJustCreatePrimInLayer(mSessionSubLayer, mSelectedPhysicsScenePath);
    PXR_NS::UsdPhysicsScene scene = PXR_NS::UsdPhysicsScene::Get(mStage, mSelectedPhysicsScenePath);
    if (!scene)
        return;

    if (enable)
    {
        scene.CreateGravityMagnitudeAttr().Set(-INFINITY);
    }
    else
    {
        scene.CreateGravityMagnitudeAttr().Set(0.0f);
    }
}

void PhysXInspector::enableQuasiStaticMode(bool enable)
{
    if (mSelectedPhysicsScenePath.IsEmpty() || mState == PhysXInspectorModel::State::eRunningSimulation)
        return;
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    PXR_NS::SdfJustCreatePrimInLayer(mSessionSubLayer, mSelectedPhysicsScenePath);
    PXR_NS::PhysxSchemaPhysxSceneQuasistaticAPI quasiStaticAPI =
        PXR_NS::PhysxSchemaPhysxSceneQuasistaticAPI::Get(mStage, mSelectedPhysicsScenePath);
    if (!quasiStaticAPI)
        return;
    quasiStaticAPI.CreateEnableQuasistaticAttr().Set(enable);
}


void PhysXInspector::createSessionSublayerIfNeeded()
{
    if (mSessionSubLayer)
    {
        auto layerStack = mStage->GetLayerStack(true);
        if (std::find(layerStack.begin(), layerStack.end(), mSessionSubLayer) == layerStack.end())
        {
            // It has probably been deleted manually by the user, so we must recreate it
            removeSessionSublayers();
        }
    }

    if (!mSessionSubLayer)
    {
        enableNoticeHandler(false);
        const auto usdContext = omni::usd::UsdContext::getContext();
        // scristiano: These calls allow to wait for async rendering to finish. Not sure if there's a better way.
        // enableUsdLocking still allows for spurious crashes with async renderer trying to access data we're changing
        // This is currently being investigated as OM-52029
        usdContext->enableUsdWrites(false);
        usdContext->enableUsdWrites(true);
        mSessionSubLayer = PXR_NS::SdfLayer::CreateAnonymous("/articulationsAuthoring");
        mStage->GetSessionLayer()->InsertSubLayerPath(mSessionSubLayer->GetIdentifier(), 0);
        enableNoticeHandler(true);
    }
}

void PhysXInspector::deleteSessionSublayer()
{
    if (mSessionSubLayer)
    {
        enableNoticeHandler(false);
        const auto usdContext = omni::usd::UsdContext::getContext();
        // scristiano: These calls allow to wait for async rendering to finish. Not sure if there's a better way.
        // enableUsdLocking still allows for spurious crashes with async renderer trying to access data we're changing
        // This is currently being investigated as OM-52029
        usdContext->enableUsdWrites(false);
        usdContext->enableUsdWrites(true);
        removeSessionSublayers();
        enableNoticeHandler(true);
    }
}

void PhysXInspector::removeSessionSublayers()
{
    // We look for the actual index of our sublayer in case user has created additional sublayers above it

    const size_t index =
        omni::usd::LayerUtils::getSublayerPositionInHost(mStage->GetSessionLayer(), mSessionSubLayer->GetIdentifier());
    if (index != omni::usd::kLayerIndexNone)
    {
        mStage->GetSessionLayer()->RemoveSubLayerPath(static_cast<int>(index));
    }
    mSessionSubLayer = nullptr;
}

void PhysXInspector::clearSessionSublayer()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::clearSessionSublayer");
    if (mSessionSubLayer)
    {
        enableNoticeHandler(false);
        // Clearing session sublayer doesn't interfere with async rendering, so we don't need enableUsdWrites
        mSessionSubLayer->Clear();
        enableNoticeHandler(true);
    }
}

void PhysXInspector::updateModelScenes(PXR_NS::SdfPathVector* foundScenesInSelection)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::updateModelScenes");
    for (auto it : mInspectorModels)
    {
        auto ptr = it.lock();
        if (ptr)
        {
            PXR_NS::SdfPathVector foundScenes;
            if (ptr->mSelectionIsArticulation)
            {
                collectAllArticulationScenes(ptr.get(), foundScenes);
            }
            else
            {
                collectAllSelectionScenes(ptr.get(), foundScenes);
            }
            if (foundScenes.size() > 0)
            {
                ptr->mScenePrimPathSdf = foundScenes[0];
            }
            else
            {
                ptr->mScenePrimPathSdf = PXR_NS::SdfPath();
            }
            if (foundScenesInSelection)
            {
                foundScenesInSelection->insert(foundScenesInSelection->end(), foundScenes.begin(), foundScenes.end());
            }
        }
    }
}

void PhysXInspector::setupAuthoringScene(PXR_NS::SdfPathVector& foundScenesInSelection)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::setupAuthoringScene");
    // We look for an assigned scene to simulation owner of currently selected objects
    // In case of a generic selection, that is not considered a single articulation we
    // also look for scenes in the list of selected prims
    // If multiple scenes are found, they are sorted by SdfPath and first one is used
    // If nothing is found yet we just try to look for the first scene that exists in the stage (outside the selection)
    // If a scene is found we clone it to current session sublayer ignoring enableGravity flag
    // If no scene is found anywhere, we setup a brand new scene (createNewSessionScene) obeying the enableGravity flag

    mSelectedPhysicsScenePath = PXR_NS::SdfPath();
    if (foundScenesInSelection.empty())
    {
        PXR_NS::UsdPrimRange range = mStage->Traverse(PXR_NS::UsdTraverseInstanceProxies());
        for (PXR_NS::UsdPrimRange::iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            // Get the current prim
            PXR_NS::UsdPrim existingScene = *iter;
            if (existingScene && existingScene.IsA<PXR_NS::UsdPhysicsScene>())
            {
                if (omni::physx::canSceneBeProcessedByPhysX(existingScene, nullptr))
                {
                    copyExistingSceneToSessionSublayer(existingScene.GetPath());
                    break;
                }
            }
        }
    }
    else
    {
        for (auto scenePath : foundScenesInSelection)
        {
            PXR_NS::UsdPrim existingScene = mStage->GetPrimAtPath(scenePath);
            if (existingScene && existingScene.IsA<PXR_NS::UsdPhysicsScene>())
            {
                if (omni::physx::canSceneBeProcessedByPhysX(existingScene, nullptr))
                {
                    copyExistingSceneToSessionSublayer(scenePath);
                    break;
                }
            }
        }
    }

    if (mSelectedPhysicsScenePath.IsEmpty())
    {
        mSelectedPhysicsScenePath = PXR_NS::SdfPath(kTempPhysicsScenePath);
        createNewSessionScene();
    }
    enableQuasiStaticMode(mEnableQuasiStaticMode->getValueAsBool());
    enableGravity(mEnableGravity->getValueAsBool());
}

void PhysXInspector::copyExistingSceneToSessionSublayer(const PXR_NS::SdfPath& scenePath)
{
    mSelectedPhysicsScenePath = PXR_NS::SdfPath(kTempPhysicsScenePath);
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    PXR_NS::UsdPrim existingScene = mStage->GetPrimAtPath(scenePath);
    omni::usd::UsdUtils::copyPrim(existingScene, kTempPhysicsScenePath, false, true);
    PXR_NS::PhysxSchemaPhysxSceneAPI physxScene = PXR_NS::PhysxSchemaPhysxSceneAPI::Get(mStage, mSelectedPhysicsScenePath);
    if (physxScene)
    {
        // We need to make sure the scene update type is synchronous otherwise inspector will not work properly
        physxScene.CreateUpdateTypeAttr().Set(PXR_NS::TfToken("Synchronous"));
        physxScene.GetPrim().SetMetadata(PXR_NS::TfToken("hide_in_stage_window"), true);
        physxScene.GetPrim().SetMetadata(PXR_NS::TfToken("no_delete"), true);
    }
}

void PhysXInspector::collectAllArticulationScenes(PhysXInspectorModelImpl* model, PXR_NS::SdfPathVector& foundScenes)
{
    for (auto& it : model->mArticulations)
    {
        for (auto& body : it->articulatedBodies)
        {
            findAllSimOwnerForBody(body, foundScenes);
        }
    }
    std::sort(foundScenes.begin(), foundScenes.end());
    auto result = std::unique(foundScenes.begin(), foundScenes.end());
    foundScenes.erase(result, foundScenes.end());
}

void PhysXInspector::collectAllSelectionScenes(PhysXInspectorModelImpl* model, PXR_NS::SdfPathVector& foundScenes)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::collectAllSelectionScenes");
    for (size_t idx = 0; idx < model->mSelection.size(); ++idx)
    {
        PXR_NS::SdfPath selPath = PXR_NS::SdfPath(model->mSelection[idx]);
        PXR_NS::UsdPrim usdPrim = mStage->GetPrimAtPath(selPath);
        PXR_NS::UsdPrimRange range(usdPrim, PXR_NS::UsdTraverseInstanceProxies());
        for (PXR_NS::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const PXR_NS::UsdPrim& prim = *iter;

            if (!prim.IsValid() || prim.IsPseudoRoot() || omni::kit::EditorUsd::isHideInStageWindow(prim))
            {
                continue;
            }
            if (prim.IsA<PXR_NS::UsdPhysicsScene>())
            {
                foundScenes.push_back(prim.GetPath());
            }
            else
            {
                findAllSimOwnerForBody(prim.GetPath(), foundScenes);
            }
        }
    }

    std::sort(foundScenes.begin(), foundScenes.end());
    auto result = std::unique(foundScenes.begin(), foundScenes.end());
    foundScenes.erase(result, foundScenes.end());
}

void PhysXInspector::collectSceneMaterials(::physx::PxScene* scene)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::collectSceneMaterials");
    using namespace physx;
    if (scene == nullptr)
    {
        return;
    }
    const PxU32 nbStaticDynamic = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC);

    for (PxU32 i = 0; i < nbStaticDynamic; i++)
    {
        PxActor* pxActor = nullptr;
        scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC | PxActorTypeFlag::eRIGID_STATIC, &pxActor, 1, i);

        if (!pxActor)
            continue;

        PxRigidBody* rigid = (PxRigidBody*)pxActor;
        PXR_NS::SdfPath rigidPath =
            mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)rigid->userData);
        const PxU32 nbShapes = rigid->getNbShapes();
        for (PxU32 shapeIdx = 0; shapeIdx < nbShapes; ++shapeIdx)
        {
            PxShape* shape = nullptr;
            rigid->getShapes(&shape, 1, shapeIdx);
            const PxU32 nbMaterials = shape->getNbMaterials();
            for (PxU32 materialIdx = 0; materialIdx < nbMaterials; ++materialIdx)
            {
                PxMaterial* material;
                shape->getMaterials(&material, 1, materialIdx);
                PXR_NS::SdfPath materialPath =
                    mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)material->userData);
                // We are inserting shapes directly instead of their ObjectIds or SdfPath because if the
                // shape belongs to a CompoundShape, we can't access CompoundShape struct as it's not part of
                // the public PhysXInterface API.
                mMaterialsToShapes[materialPath].insert(shape);
            }
        }
    }

    PxArticulationReducedCoordinate* articulation = nullptr;
    const PxU32 nbArticulations = scene->getNbArticulations();
    for (PxU32 i = 0; i < nbArticulations; i++)
    {
        scene->getArticulations(&articulation, 1, i);
        collectArticulationMaterials(articulation);
    }
}

void PhysXInspector::collectArticulationMaterials(::physx::PxArticulationReducedCoordinate* articulation)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::collectArticulationMaterials");
    using namespace physx;
    ::physx::PxU32 numLinks = articulation->getNbLinks();
    for (::physx::PxU32 idx = 0; idx < numLinks; ++idx)
    {
        ::physx::PxArticulationLink* rigid = nullptr;
        articulation->getLinks(&rigid, 1, idx);
        const PxU32 nbShapes = rigid->getNbShapes();
        for (PxU32 shapeIdx = 0; shapeIdx < nbShapes; ++shapeIdx)
        {
            PxShape* shape = nullptr;
            rigid->getShapes(&shape, 1, shapeIdx);
            const PxU32 nbMaterials = shape->getNbMaterials();
            for (PxU32 materialIdx = 0; materialIdx < nbMaterials; ++materialIdx)
            {
                PxMaterial* material;
                shape->getMaterials(&material, 1, materialIdx);
                PXR_NS::SdfPath materialPath =
                    mPhysXInterface->getPhysXObjectUsdPath((omni::physx::usdparser::ObjectId)material->userData);
                // We are inserting shapes directly instead of their ObjectIds or SdfPath because if the
                // shape belongs to a CompoundShape, we can't access CompoundShape struct as it's not part of
                // the public PhysXInterface API.
                mMaterialsToShapes[materialPath].insert(shape);
            }
        }
    }
}

void PhysXInspector::reassignArticulationSimOwnerToHiddenScene(PhysXInspectorModelImpl* model,
                                                               const PXR_NS::SdfPath& tempPhysicsScenePath)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::reassignArticulationSimOwnerToHiddenScene");
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    PXR_NS::SdfChangeBlock changeBlock;

    for (auto& it : model->mArticulations)
    {
        for (auto& body : it->articulatedBodies)
        {
            reassignSimOwnerForBody(body, tempPhysicsScenePath);
        }
        for (auto& joint : it->articulatedJoints)
        {
            PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(joint);
            if (!prim)
                continue;
            if (prim.IsA<PXR_NS::UsdPhysicsJoint>())
            {
                PXR_NS::UsdPhysicsJoint physicsJoint(prim);
                insertIntoBodyJointMap(physicsJoint);
            }
        }
    }
}

void PhysXInspector::reassignSelectionSimOwnerToHiddenScene(PhysXInspectorModelImpl* model,
                                                            const PXR_NS::SdfPath& tempPhysicsScenePath)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::reassignSelectionSimOwnerToHiddenScene");
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    // PXR_NS::SdfChangeBlock changeBlock; // Using change block here it will make fail changing drive api
    // damping/stiffness
    PXR_NAMESPACE_USING_DIRECTIVE;
    mJointBodyMap.clear();
    for (size_t idx = 0; idx < model->mSelection.size(); ++idx)
    {
        SdfPath selPath = SdfPath(model->mSelection[idx]);
        UsdPrim usdPrim = mStage->GetPrimAtPath(selPath);
        PXR_NS::UsdPrimRange range(usdPrim, UsdTraverseInstanceProxies());
        for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const UsdPrim& prim = *iter;

            if (!prim.IsValid() || prim.IsPseudoRoot() || omni::kit::EditorUsd::isHideInStageWindow(prim))
            {
                continue;
            }
            reassignSimOwnerForBody(prim.GetPath(), tempPhysicsScenePath);
            if (prim.IsA<UsdPhysicsJoint>())
            {
                PXR_NS::UsdPhysicsJoint physicsJoint(prim);
                insertIntoBodyJointMap(physicsJoint);
            }
        }
    }
}

void PhysXInspector::updateModelJointMapForArticulation(PhysXInspectorModelImpl* model)
{
    for (auto& it : model->mArticulations)
    {
        for (auto& joint : it->articulatedJoints)
        {
            PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(joint);

            if (!prim.IsValid() || prim.IsPseudoRoot() || omni::kit::EditorUsd::isHideInStageWindow(prim))
            {
                continue;
            }
            if (prim.IsA<PXR_NS::UsdPhysicsJoint>())
            {
                PXR_NS::UsdPhysicsJoint physicsJoint(prim);
                insertIntoBodyJointMap(physicsJoint);
            }
        }
    }
}

void PhysXInspector::updateModelJointMapForSelection(PhysXInspectorModelImpl* model)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::updateModelJointMapForSelection");
    PXR_NAMESPACE_USING_DIRECTIVE;
    for (size_t idx = 0; idx < model->mSelection.size(); ++idx)
    {
        SdfPath selPath = SdfPath(model->mSelection[idx]);
        UsdPrim usdPrim = mStage->GetPrimAtPath(selPath);
        PXR_NS::UsdPrimRange range(usdPrim, UsdTraverseInstanceProxies());
        for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const UsdPrim& prim = *iter;

            if (!prim.IsValid() || prim.IsPseudoRoot() || omni::kit::EditorUsd::isHideInStageWindow(prim))
            {
                continue;
            }
            if (prim.IsA<UsdPhysicsJoint>())
            {
                PXR_NS::UsdPhysicsJoint physicsJoint(prim);
                insertIntoBodyJointMap(physicsJoint);
            }
        }
    }
}


void PhysXInspector::insertIntoBodyJointMap(const PXR_NS::UsdPhysicsJoint& physicsJoint)
{
    PXR_NAMESPACE_USING_DIRECTIVE;
    SdfPathVector target;
    physicsJoint.GetBody0Rel().GetTargets(&target);
    if (!target.empty())
    {
        mJointBodyMap[target[0]].insert(physicsJoint.GetPath());
    }
    target.clear();
    physicsJoint.GetBody1Rel().GetTargets(&target);
    if (!target.empty())
    {
        mJointBodyMap[target[0]].insert(physicsJoint.GetPath());
    }
}


void PhysXInspector::createGravitySafetyNet(const std::vector<std::string>& selectedPrims)
{
    PXR_NAMESPACE_USING_DIRECTIVE;
    if (selectedPrims.empty())
        return;
    // first let's compute bounds
    UsdGeomBBoxCache mBBCache(UsdTimeCode::Default(), { UsdGeomTokens->default_ });
    GfRange3d totalExtent;
    bool atLeastOneValidExtent = false;
    for (size_t idx = 0; idx < selectedPrims.size(); ++idx)
    {
        SdfPath selPath = SdfPath(selectedPrims[idx]);
        UsdPrim usdPrim = mStage->GetPrimAtPath(selPath);
        const GfRange3d extent = mBBCache.ComputeWorldBound(usdPrim).ComputeAlignedBox();
        if (extent.IsEmpty())
            continue;
        atLeastOneValidExtent = true;
        totalExtent.ExtendBy(extent);
    }
    if (!atLeastOneValidExtent)
        return;
    const GfVec3d ptMin = totalExtent.GetMin();
    const GfVec3d ptMax = totalExtent.GetMax();

    SdfPath tempPhysicsScenePath(kTempPhysicsScenePath);
    omni::usd::UsdUtils::ScopedLayerEdit scopedSessionLayerEdit(mStage, mSessionSubLayer);
    SdfPath groundPlanePath(mGravitySafeNetPlanePath);
    UsdGeomPlane groundPlane = UsdGeomPlane::Define(mStage, groundPlanePath);
    // cannot use omni::kit::EditorUsd::setHideInStageWindow() because it is scoping
    // session layer inside, overriding our inserted session sublayer

    groundPlane.GetPrim().SetMetadata(TfToken("hide_in_stage_window"), true);
    groundPlane.GetPrim().SetMetadata(TfToken("no_delete"), true);

    groundPlane.CreateAxisAttr().Set(UsdGeomGetStageUpAxis(mStage));
    if (UsdGeomGetStageUpAxis(mStage) == UsdGeomTokens->z)
    {
        groundPlane.AddTranslateOp().Set(GfVec3d(0.0, 0.0, ptMin[2] - (ptMax[2] - ptMin[2])));
    }
    else
    {
        groundPlane.AddTranslateOp().Set(GfVec3d(0.0, ptMin[1] - (ptMax[1] - ptMin[1]), 0.0));
    }
    groundPlane.AddOrientOp().Set(GfQuatf(0.0f));
    groundPlane.AddScaleOp().Set(GfVec3f(1.0f));
    UsdPhysicsCollisionAPI::Apply(groundPlane.GetPrim()).CreateSimulationOwnerRel().AddTarget(tempPhysicsScenePath);
}


void PhysXInspector::reassignSimOwnerForBody(const PXR_NS::SdfPath& body, const PXR_NS::SdfPath& tempPhysicsScenePath)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::reassignSimOwnerForBody");
    const PXR_NS::UsdPhysicsRigidBodyAPI rigidBodyApi = PXR_NS::UsdPhysicsRigidBodyAPI::Get(mStage, body);
    if (rigidBodyApi)
    {
        PXR_NS::UsdRelationship rel = rigidBodyApi.CreateSimulationOwnerRel();
        rel.SetTargets(PXR_NS::SdfPathVector());
        rel.AddTarget(tempPhysicsScenePath, PXR_NS::UsdListPositionFrontOfPrependList);
    }
    const PXR_NS::UsdPhysicsCollisionAPI collisionApi = PXR_NS::UsdPhysicsCollisionAPI::Get(mStage, body);
    if (collisionApi)
    {
        PXR_NS::UsdRelationship rel = collisionApi.CreateSimulationOwnerRel();
        rel.SetTargets(PXR_NS::SdfPathVector());
        rel.AddTarget(tempPhysicsScenePath, PXR_NS::UsdListPositionFrontOfPrependList);
    }
    else
    {
        // We have to find all colliders down
        PXR_NS::UsdPrimRange primRange(mStage->GetPrimAtPath(body));
        for (PXR_NS::UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
        {
            const PXR_NS::UsdPrim& subPrim = *iter;
            const PXR_NS::UsdPhysicsCollisionAPI subCollisionApi =
                PXR_NS::UsdPhysicsCollisionAPI::Get(mStage, subPrim.GetPath());
            if (subCollisionApi)
            {
                PXR_NS::UsdRelationship rel = subCollisionApi.CreateSimulationOwnerRel();
                rel.SetTargets(PXR_NS::SdfPathVector());
                rel.AddTarget(tempPhysicsScenePath, PXR_NS::UsdListPositionFrontOfPrependList);
            }
        }
    }
}

void PhysXInspector::findAllSimOwnerForBody(const PXR_NS::SdfPath& body, PXR_NS::SdfPathVector& foundOwners)
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::findAllSimOwnerForBody");
    const PXR_NS::UsdPhysicsRigidBodyAPI rigidBodyApi = PXR_NS::UsdPhysicsRigidBodyAPI::Get(mStage, body);
    if (rigidBodyApi)
    {
        PXR_NS::UsdRelationship rel = rigidBodyApi.CreateSimulationOwnerRel();
        PXR_NS::SdfPathVector targets;
        if (rel.GetTargets(&targets))
        {
            foundOwners.insert(foundOwners.end(), targets.begin(), targets.end());
        }
    }
    const PXR_NS::UsdPhysicsCollisionAPI collisionApi = PXR_NS::UsdPhysicsCollisionAPI::Get(mStage, body);
    if (collisionApi)
    {
        PXR_NS::UsdRelationship rel = collisionApi.CreateSimulationOwnerRel();
        PXR_NS::SdfPathVector targets;
        if (rel.GetTargets(&targets))
        {
            foundOwners.insert(foundOwners.end(), targets.begin(), targets.end());
        }
    }
    else
    {
        // We have to find all colliders down
        PXR_NS::UsdPrimRange primRange(mStage->GetPrimAtPath(body));
        for (PXR_NS::UsdPrimRange::const_iterator iter = primRange.begin(); iter != primRange.end(); ++iter)
        {
            const PXR_NS::UsdPrim& subPrim = *iter;
            const PXR_NS::UsdPhysicsCollisionAPI subCollisionApi =
                PXR_NS::UsdPhysicsCollisionAPI::Get(mStage, subPrim.GetPath());
            if (subCollisionApi)
            {
                PXR_NS::UsdRelationship rel = subCollisionApi.CreateSimulationOwnerRel();
                PXR_NS::SdfPathVector targets;
                if (rel.GetTargets(&targets))
                {
                    foundOwners.insert(foundOwners.end(), targets.begin(), targets.end());
                }
            }
        }
    }
}


void PhysXInspector::startSimulation()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::startSimulation");
    ScopedReleaseEventIgnore ignoreReleaseEvents(mIgnorePhysicsObjectsReleased);
    mCurrentTime = 0;
    mISettings->setString(omni::physx::kSettingForceParseOnlySingleScene, mSelectedPhysicsScenePath.GetText());
    mPhysXInterface->releasePhysicsObjects(); // sets the empty scene flag so that full parsing is triggered
    mPhysXInterface->startSimulation();
    mPhysXInterface->updateSimulation(0, 0); // this is to force calling finishSetup that adds articulation to scene
    mISettings->destroyItem(omni::physx::kSettingForceParseOnlySingleScene);
}

void PhysXInspector::stepScene(float dt)
{
    mISettings->setString(omni::physx::kSettingForceParseOnlySingleScene, mSelectedPhysicsScenePath.GetText());
    mPhysXInterface->updateSimulation(dt, mCurrentTime);
    mPhysXInterface->updateTransformations(false, true, true, true);
    mISettings->destroyItem(omni::physx::kSettingForceParseOnlySingleScene);
    mCurrentTime += dt;
}

void PhysXInspector::stepIfNecessary(float dt)
{
    if (mState == PhysXInspectorModel::State::eAuthoring)
    {
        CARB_PROFILE_ZONE(0, "PhysXInspector::stepIfNecessary");
        if (mSilenceUsdChangeEvents)
        {
            stepScene(dt);
        }
        else
        {
            enableNoticeHandler(false);
            stepScene(dt);
            enableNoticeHandler(true);
        }
    }
}

void PhysXInspector::applyChangeSelection()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::applyChangeSelection");
    ScopedReleaseEventIgnore ignoreReleaseEvents(mIgnorePhysicsObjectsReleased);
    // We just clear the layer instead of dropping it as it's more efficient, and it will not trigger a full
    // USD resync (affecting also rendering) that happens when you drop the entire sublayer
    clearSessionSublayer();
    if (mState == PhysXInspectorModel::State::eRunningSimulation)
    {
        enableNoticeHandler(false);

        updateModelArticulations();
        updateModelJointMap();
        updateModelMaterials();
        enableNoticeHandler(true);
    }
    else
    {
        createSessionSublayerIfNeeded();
        enableNoticeHandler(false);

        updateModelArticulations();
        PXR_NS::SdfPathVector foundScenesInSelection;
        updateModelScenes(&foundScenesInSelection);
        setupAuthoringScene(foundScenesInSelection);
        updateModelSimulationOwners();
        startSimulation();
        updateModelMaterials();

        enableNoticeHandler(true);
    }
    refreshAllInspectorModelsStructure();
}

void PhysXInspector::updateModelArticulations()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::updateModelArticulations");
    const uint64_t stageId = PXR_NS::UsdUtilsStageCache::Get().GetId(mStage).ToLongInt();

    for (auto it : mInspectorModels)
    {
        auto ptr = it.lock();
        if (!ptr)
            continue;
        ptr->parseArticulations(stageId);
    }
}


void PhysXInspector::updateModelSimulationOwners()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::updateModelSimulationOwners");
    mJointBodyMap.clear();
    for (auto it : mInspectorModels)
    {
        auto ptr = it.lock();
        if (!ptr)
            continue;
        if (ptr->mSelectionIsArticulation)
        {
            reassignArticulationSimOwnerToHiddenScene(ptr.get(), mSelectedPhysicsScenePath);
        }
        else
        {
            reassignSelectionSimOwnerToHiddenScene(ptr.get(), mSelectedPhysicsScenePath);
        }
        ptr->mScenePrimPathSdf = mSelectedPhysicsScenePath;
    }
}

void PhysXInspector::updateModelJointMap()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::updateModelJointMap");
    mJointBodyMap.clear();
    for (auto it : mInspectorModels)
    {
        auto ptr = it.lock();
        if (!ptr)
            continue;
        if (ptr->mSelectionIsArticulation)
        {
            updateModelJointMapForArticulation(ptr.get());
        }
        else
        {
            updateModelJointMapForSelection(ptr.get());
        }
    }
}

void PhysXInspector::updateModelMaterials()
{
    CARB_PROFILE_ZONE(0, "PhysXInspector::updateModelMaterials");
    mMaterialsToShapes.clear();
    for (auto it : mInspectorModels)
    {
        auto ptr = it.lock();
        if (!ptr)
            continue;

        if (ptr->mSelectionIsArticulation)
        {
            if (getArticulationAt(ptr->mSelectedPrimPath) == nullptr)
            {
                ptr->mSelectionIsArticulation = false;
            }
        }

        if (ptr->mSelectionIsArticulation)
        {
            collectArticulationMaterials(getArticulationAt(ptr->mSelectedPrimPath));
        }
        else
        {
            ::physx::PxScene* scene = getSceneAt(ptr->mScenePrimPathSdf.GetText());
            if (scene == nullptr)
            {
                scene = mPhysXPrivateInterface->getPhysXScene();
            }
            collectSceneMaterials(scene);
        }
    }
}

::physx::PxArticulationReducedCoordinate* PhysXInspector::getArticulationAt(const std::string& currentPath)
{
    if (!mStage)
        return nullptr;
    if (!currentPath.empty() && currentPath[0] == '/')
    {
        const PXR_NS::SdfPath path(currentPath);
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
        if (prim)
        {
            omni::physx::usdparser::ObjectId articulationID =
                mPhysXInterface->getObjectId(path, omni::physx::ePTArticulation);
            if (articulationID != omni::physx::usdparser::kInvalidObjectId)
            {
                ::physx::PxArticulationReducedCoordinate* arti = static_cast<::physx::PxArticulationReducedCoordinate*>(
                    mPhysXInterface->getPhysXPtrFast(articulationID));
                return arti;
            }
        }
    }
    return nullptr;
}

::physx::PxScene* PhysXInspector::getSceneAt(const std::string& currentPath)
{
    if (!mStage)
        return nullptr;
    if (!currentPath.empty() && currentPath[0] == '/')
    {
        const PXR_NS::SdfPath path(currentPath);
        PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
        if (prim)
        {
            omni::physx::usdparser::ObjectId sceneID = mPhysXInterface->getObjectId(path, omni::physx::ePTScene);
            if (sceneID != omni::physx::usdparser::kInvalidObjectId)
            {
                ::physx::PxScene* scene = static_cast<::physx::PxScene*>(mPhysXInterface->getPhysXPtrFast(sceneID));
                return scene;
            }
        }
    }
    return nullptr;
}

bool PhysXInspector::USDGetJointLimits(PXR_NS::UsdPrim prim, float& lowLimit, float& highLimit)
{
    if (prim.IsA<PXR_NS::UsdPhysicsRevoluteJoint>())
    {
        PXR_NS::UsdPhysicsRevoluteJoint revoluteJoint(prim);
        revoluteJoint.GetLowerLimitAttr().Get(&lowLimit);
        revoluteJoint.GetUpperLimitAttr().Get(&highLimit);
        return true;
    }
    else if (prim.IsA<PXR_NS::UsdPhysicsPrismaticJoint>())
    {
        PXR_NS::UsdPhysicsPrismaticJoint PxPrismaticJoint(prim);
        PxPrismaticJoint.GetLowerLimitAttr().Get(&lowLimit);
        PxPrismaticJoint.GetUpperLimitAttr().Get(&highLimit);
        return true;
    }
    return false;
}

bool PhysXInspector::USDGetJointStiffnessDamping(PXR_NS::UsdPrim prim, float& stiffness, float& damping)
{
    if (prim.HasAPI<PXR_NS::UsdPhysicsDriveAPI>())
    {
        PXR_NS::UsdPhysicsDriveAPI driveAPI = PXR_NS::UsdPhysicsDriveAPI::Get(prim, PXR_NS::UsdPhysicsTokens->angular);
        driveAPI.GetStiffnessAttr().Get(&stiffness);
        driveAPI.GetDampingAttr().Get(&damping);
        return true;
    }
    else if (prim.IsA<PXR_NS::UsdPhysicsPrismaticJoint>() && prim.HasAPI<PXR_NS::UsdPhysicsDriveAPI>())
    {
        PXR_NS::UsdPhysicsDriveAPI driveAPI = PXR_NS::UsdPhysicsDriveAPI::Get(prim, PXR_NS::UsdPhysicsTokens->linear);
        driveAPI.GetStiffnessAttr().Get(&stiffness);
        driveAPI.GetDampingAttr().Get(&damping);
        return true;
    }
    return false;
}


bool PhysXInspector::USDGetJointLimitsClamped(PXR_NS::UsdPrim prim, float& lowLimit, float& highLimit)
{
    if (USDGetJointLimits(prim, lowLimit, highLimit))
    {
        if (prim.IsA<PXR_NS::UsdPhysicsRevoluteJoint>())
        {
            if (!isfinite(lowLimit))
                lowLimit = -360;
            if (!isfinite(highLimit))
                highLimit = +360;
            return true;
        }
        else if (prim.IsA<PXR_NS::UsdPhysicsPrismaticJoint>())
        {
            if (!isfinite(lowLimit))
                lowLimit = -100;
            if (!isfinite(highLimit))
                highLimit = +100;
            return true;
        }
    }
    return false;
}

const char* PhysXInspector::getPhysXShapeDescription(::physx::PxShape* shape)
{
    // clang-format off
    switch (shape->getGeometry().getType())
    {
        case physx::PxGeometryType::eSPHERE: return "Sphere";
        case physx::PxGeometryType::ePLANE: return "Plane";
        case physx::PxGeometryType::eCAPSULE: return "Capsule";
        case physx::PxGeometryType::eBOX: return "Box";
        case physx::PxGeometryType::eCONVEXMESH: return "Convex Mesh";
        case physx::PxGeometryType::ePARTICLESYSTEM: return "Particle Systme";
        case physx::PxGeometryType::eTETRAHEDRONMESH: return "Tetrahedron Mesh";
        case physx::PxGeometryType::eTRIANGLEMESH: return "Triangle Mesh";
        case physx::PxGeometryType::eHEIGHTFIELD: return "Heightfield";
        case physx::PxGeometryType::eCUSTOM: return "Custom";
        default: return "Unknown";
    }
    // clang-format on
    return "Unknown Shape";
}
