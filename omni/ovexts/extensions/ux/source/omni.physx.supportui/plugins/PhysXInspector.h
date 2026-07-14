// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once
#include <private/omni/physx/IPhysxSupportUiPrivate.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <carb/settings/ISettings.h>
#include <carb/eventdispatcher/IEventDispatcher.h>
#include <PxPhysicsAPI.h>

using JointBodyMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, PXR_NS::SdfPathSet, PXR_NS::SdfPath::Hash>;
using MaterialsShapesMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, std::unordered_set<::physx::PxShape*>, PXR_NS::SdfPath::Hash>;

namespace omni
{
namespace ui
{
class PhysXInspectorWidgetImpl;
namespace scene
{
class PhysXInspectorOverlayImpl;
}
} // namespace ui
} // namespace omni
class PhysXInspectorModelImpl;
class PhysXInspectorDebugVisualization;


class PhysXInspector : public PXR_NS::TfWeakBase
{
public:
    static float constexpr BaseDeltaTimeStep{ 1.0f / 60.0f };
    void onStartup();
    void onShutdown();
    std::shared_ptr<PhysXInspectorModel> createPhysXInspectorModel(const PXR_NS::SdfPathVector& selection);
    carb::events::IEventStreamPtr getEventStream() const
    {
        return mStream;
    }
    PhysXInspectorModel::State getState() const;
    void stepIfNecessary(float dt = BaseDeltaTimeStep);
    void stopAuthoringSimulation();
    void resetToAuthoringStart();
    void commitAuthoringState();
    void enableAuthoringMode();
    void enableNoticeHandler(bool enable);
    void handlePhysicsObjectsReleased();
    void refreshAllInspectorModelsStructure();
    void refreshAllInspectorModelsValues();
    void refreshAllInspectorModelsOverrides();

private:
    friend class PhysXInspectorModelImpl;
    friend class PhysXInspectorDebugVisualization;
    friend class omni::ui::PhysXInspectorWidgetImpl;
    friend class omni::ui::scene::PhysXInspectorOverlayImpl;
    static const char* getPhysXShapeDescription(::physx::PxShape* shape);
    static bool USDGetJointStiffnessDamping(PXR_NS::UsdPrim prim, float& stiffness, float& damping);
    static bool USDGetJointLimits(PXR_NS::UsdPrim prim, float& lowLimit, float& highLimit);
    static bool USDGetJointLimitsClamped(PXR_NS::UsdPrim prim, float& lowLimit, float& highLimit);
    void onModelCreated(PhysXInspectorModelImpl* model);
    void onModelDestroyed(PhysXInspectorModelImpl* model);
    void disableAuthoringMode();
    void handleUsdObjChange(const class PXR_NS::UsdNotice::ObjectsChanged& objectsChanged);
    void setState(PhysXInspectorModel::State newState);

    ::physx::PxArticulationReducedCoordinate* getArticulationAt(const std::string& currentPath);
    ::physx::PxScene* getSceneAt(const std::string& currentPath);
    bool findParentArticulationRoot(const std::string& usdPath, std::string& outputPath);
    void createNewSessionScene();
    void createSessionSublayerIfNeeded();
    void deleteSessionSublayer();
    void removeSessionSublayers();
    void clearSessionSublayer();
    void setupAuthoringScene(PXR_NS::SdfPathVector& foundScenesInSelection);
    void copyExistingSceneToSessionSublayer(const PXR_NS::SdfPath& scenePath);
    void collectAllArticulationScenes(PhysXInspectorModelImpl* model, PXR_NS::SdfPathVector& foundScenes);
    void collectAllSelectionScenes(PhysXInspectorModelImpl* model, PXR_NS::SdfPathVector& foundScenes);
    void collectSceneMaterials(::physx::PxScene* scene);
    void collectArticulationMaterials(::physx::PxArticulationReducedCoordinate* articulation);
    void insertIntoBodyJointMap(const PXR_NS::UsdPhysicsJoint& joint);
    void reassignArticulationSimOwnerToHiddenScene(PhysXInspectorModelImpl* model,
                                                   const PXR_NS::SdfPath& tempPhysicsScenePath);
    void reassignSelectionSimOwnerToHiddenScene(PhysXInspectorModelImpl* model, const PXR_NS::SdfPath& tempPhysicsScenePath);
    void createGravitySafetyNet(const std::vector<std::string>& selectedPrims);
    void reassignSimOwnerForBody(const PXR_NS::SdfPath& body, const PXR_NS::SdfPath& tempPhysicsScenePath);
    void findAllSimOwnerForBody(const PXR_NS::SdfPath& body, PXR_NS::SdfPathVector& foundOwners);
    void startSimulation();
    void stepScene(float dt);
    void applyChangeSelection();
    void onSimStop();
    void onSimUpdate();
    void onSimResume(float currentTime);
    void buildBodyJointMap();
    void updateModelArticulations();
    void updateModelMaterials();
    void updateModelSimulationOwners();
    void updateModelScenes(PXR_NS::SdfPathVector* foundScenesInSelection);
    void updateModelJointMap();
    void updateModelJointMapForArticulation(PhysXInspectorModelImpl* model);
    void updateModelJointMapForSelection(PhysXInspectorModelImpl* model);

    bool checkIfShouldStopAuthoring(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& attrName);
    bool checkIfShouldUpdateModels(const PXR_NS::SdfPath& primPath, const PXR_NS::TfToken& attrName);

    const char* mGravitySafeNetPlanePath = "/PhysicsInspector_SafetyNet";
    const char* kTempPhysicsScenePath = "/PhysicsInspector_AuthoringScene";

    std::shared_ptr<omni::ui::SimpleBoolModel> mEnableGravity;
    std::shared_ptr<omni::ui::SimpleBoolModel> mEnableQuasiStaticMode;

    void enableGravity(bool enable);
    void enableQuasiStaticMode(bool enable);

    PhysXInspectorModel::State mState = PhysXInspectorModel::State::eDisabled;
    float mCurrentTime = 0;
    carb::events::IEventStreamPtr mStream;
    PXR_NS::SdfLayerRefPtr mSessionSubLayer;
    PXR_NS::SdfPath mSelectedPhysicsScenePath;
    carb::dictionary::SubscriptionId* mPhysicsInspectorEnabledSubID = nullptr;
    bool mCmdCommitAndDisable = false;
    bool mCmdUpdateModels = false;
    bool mCmdZeroStep = false;
    bool mSilenceUsdChangeEvents = false;
    uint32_t mIgnorePhysicsObjectsReleased = 0;
    omni::physx::IPhysx* mPhysXInterface = nullptr;
    omni::physx::IPhysxPrivate* mPhysXPrivateInterface = nullptr;
    PXR_NS::UsdStageRefPtr mStage;
    std::array<carb::eventdispatcher::ObserverGuard, 2> mStageEvtSub;
    PXR_NS::TfNotice::Key mUsdNoticeListenerKey;
    omni::kit::StageUpdateNode* mStageUpdateNodeBefore = nullptr;
    omni::kit::StageUpdateNode* mStageUpdateNodeAfter = nullptr;
    carb::settings::ISettings* mISettings;
    JointBodyMap mJointBodyMap;
    MaterialsShapesMap mMaterialsToShapes;
    std::vector<std::weak_ptr<PhysXInspectorModelImpl>> mInspectorModels;
};
