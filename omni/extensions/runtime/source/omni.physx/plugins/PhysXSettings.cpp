// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "PhysXSettings.h"
#include "SceneMultiGPUMode.h"
#include <carb/settings/ISettings.h>
#include <carb/logging/Log.h>
#include <carb/dictionary/DictionaryUtils.h>
#include <private/omni/physx/ui/VisualizerMode.h>
#include <private/omni/physx/ui/ParticleVisualizationModes.h>

using namespace omni::physx;

static PhysXSettings gPhysXSettings = PhysXSettings(
    // bool
    {
        { kSettingUpdateToUsd, true },
        { kSettingUpdateVelocitiesToUsd, true },
        { kSettingOutputVelocitiesLocalSpace, false },
        { kSettingUpdateParticlesToUsd, true },
        { kSettingUpdateResidualsToUsd, true },
        { kSettingAutocreatePhysicsScene, true },
        { kSettingUseLocalMeshCache, true },
        { kSettingUjitsoCollisionCooking, true },
        { kSettingUjitsoRemoteCacheEnabled, false },
        { kSettingResetOnStop, true },
        { kSettingCollisionApproximateCones, false },
        { kSettingCollisionApproximateCylinders, false },
        { kSettingMouseInteractionEnabled, true },
        { kSettingMouseGrab, true },
        { kSettingMouseGrabIgnoreInvisible, true },
        { kSettingMouseGrabWithForce, true },
        { kSettingPVDProfile, true },
        { kSettingPVDMemory, true },
        { kSettingPVDDebug, true },
        { kSettingPVDEnabled, false },
        { kSettingPVDStreamToFile, false },
        { kSettingPhysicsDevelopmentMode, false },
        { kSettingDisplayJoints, true },
        { kSettingMassDistributionManipulator, false },
        { kSettingEnableParticleAuthoring, true },
        { kSettingEnableAttachmentAuthoring, true },
        { kSettingDisplaySimulationOutput, false },
        { kSettingDisplaySimulationDataVisualizer, false },
        { kSettingAutoPopupSimulationOutputWindow, true },
        { kSettingSuppressReadback, false },
        { kSettingExposeProfilerData, true },
        { kSettingExposePrimPathNames, true },
        { kOmniPvdOutputEnabled, false },
        { kOmniPvdIsOVDStage, false },
        { kOmniPvdIsRecording, false },
        { kSettingPhysxDispatcher, false },
        { kSettingDisplayColliderNormals, false },
        { kSettingDisplayParticlesShowDiffuseParticles, false },
        { kSettingDisplayParticlesClothMeshLines, false },
        { kSettingDisplayParticlesShowParticleSetParticles, true },
        { kSettingDisplayParticlesShowFluidSurface, false },
        { kSettingDisplayParticlesShowDeformableParticles, true },
        { kSettingDisplayParticlesShowDeformableMesh, false },
        { kSettingDisplayAttachmentsHideActor0, false },
        { kSettingDisplayAttachmentsHideActor1, false },
        { kSettingLogRobotics, false },
        { kSettingLogSceneMultiGPU, false },
        { kSettingSimulateEmptyScene, false },
        { kSettingDisableSleeping, false },
        { kSettingSynchronousKernelLaunches, false },
        { kSettingShowCollisionGroupsWindow, false },
        { kSettingDisableContactProcessing, false },
        { kSettingDebugVisQueryUsdrtForTraversal, false },
        { kSettingEnableExtendedJointAngles, false },
    },
    // int
    {
        { kSettingOverrideGPU, -1 },
        { kSettingNumThreads, 8 },
        { kSettingMaxNumberOfPhysXErrors, 1000 },
        { kSettingLocalMeshCacheSizeMB, 1024 },
        { kSettingTestRunnerRepeats, 1 },
        { kSettingDisplayColliders, int(ui::VisualizerMode::eNone) },
        { kSettingDisplayMassProperties, int(ui::VisualizerMode::eNone) },
        { kSettingDisplayTendons, int(ui::VisualizerMode::eNone) },
        // DEPRECATED
        { kSettingDisplayDeformableBodies, int(ui::VisualizerMode::eNone) },
        { kSettingDisplayDeformableBodyType, 0 },
        { kSettingDisplayDeformableSurfaces, int(ui::VisualizerMode::eNone) },
        { kSettingDisplayAttachments, int(ui::VisualizerMode::eNone) },
        //~DEPRECATED
        { kSettingDisplayDeformables, int(ui::VisualizerMode::eNone) },
        { kSettingDisplayDeformableMeshType, 0 },
        { kSettingDisplayDeformableAttachments, false },
        { kSettingEnableDeformableBeta, false },
        { kSettingDisplayParticles, int(ui::VisualizerMode::eNone) },
        { kSettingDisplayParticlesParticlePositions, int(ui::ParticlePositionType::eSimPositions) },
        { kSettingDisplayParticlesParticleRadius, int(ui::ParticleRadiusType::eParticleContactOffset) },
        // OM-18510
        // Workaround for Kit event loop issues. If we do not pump the event loop
        // before creating new stages between automated test cases, Kit can
        // get into a bad state where it is still processing events associated
        // with the previous stage even after its contents have gone away.
        { kSettingNumEventPumpsForTestStageSetup, 3 },
        // Same as in KitSDK, there's no getDefault for settings
        { kSettingMinFrameRate, 30 },
        { kSettingAddMenuSelectionLimit, 1000 },
        { kSettingAddMenuSubtreeLimit, 100000 },
        { kSettingUjitsoCookingMaxProcessCount, 16 },
        { kSettingSceneMultiGPUMode, int(SceneMultiGPUMode::eDisabled)},
    },
    // int64
    {
        { kSettingUjitsoCookingDevKey, 0ll },
    },
    // float
    {
        { kSettingMousePush, 1000.f },
        { kSettingMousePickingForce, 1.0f },
        { kSettingVisualizationGap, 0.2f },
        { kSettingJointBodyTransformCheckTolerance, 1e-3f },
        { kSettingDebugVisSimplifyAtDistance, 2500.0f },
    },
    // string
    {
        { kSettingPVDIPAddress, "127.0.0.1" },
        { kSettingPVDOutputDirectory, "" },
        { kSettingTestRunnerFilter, "" },
        { kOmniPvdOvdRecordingDirectory, "" },
        { kSettingDemoAssetsPath, "" },
        { kSettingTestsAssetsPath, "" },
        { kSettingDefaultSimulator, "PhysX" }
    },
    // string array
    {
        { kSettingTestRunnerSelection, { nullptr, 0 } },
        { kSettingTestRunnerStatus, { nullptr, 0 } },
    });

namespace omni
{
namespace physx
{

static constexpr char kDefaultPrefix[] = DEFAULT_SETTING_PREFIX;
static constexpr char kDefaultSettingsPath[] = DEFAULT_SETTING_PREFIX PHYSICS_SETTINGS_PREFIX;
static constexpr char kDefaultPersistentSettingsPath[] =
    DEFAULT_SETTING_PREFIX PERSISTENT_SETTINGS_PREFIX PHYSICS_SETTINGS_PREFIX;
static constexpr char kSettingsPath[] = PHYSICS_SETTINGS_PREFIX;
static constexpr char kPersistentSettingsPath[] = PERSISTENT_SETTINGS_PREFIX PHYSICS_SETTINGS_PREFIX;

PhysXSettings::PhysXSettings(TBoolDefaults boolDefaults,
                             TIntDefaults intDefaults,
                             TInt64Defaults int64Defaults,
                             TFloatDefaults floatDefaults,
                             TStringDefaults stringDefaults,
                             TStringArrayDefaults stringArrayDefaults)
{
    mBoolDefaults = std::move(boolDefaults);
    mIntDefaults = std::move(intDefaults);
    mInt64Defaults = std::move(int64Defaults);
    mFloatDefaults = std::move(floatDefaults);
    mStringDefaults = std::move(stringDefaults);
    mStringArrayDefaults = std::move(stringArrayDefaults);
}

PhysXSettings& PhysXSettings::getInstance()
{
    return gPhysXSettings;
}

void PhysXSettings::setDefaults()
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    if (!settings)
    {
        return;
    }

    for (auto& val : mBoolDefaults)
    {
        settings->setDefaultBool(val.first, val.second);
        settings->setDefaultBool((std::string(kDefaultPrefix) + val.first).c_str(), val.second);
    }

    for (auto& val : mIntDefaults)
    {
        settings->setDefaultInt(val.first, val.second);
        settings->setDefaultInt((std::string(kDefaultPrefix) + val.first).c_str(), val.second);
    }

    for (auto& val : mInt64Defaults)
    {
        settings->setDefaultInt64(val.first, val.second);
        settings->setDefaultInt64((std::string(kDefaultPrefix) + val.first).c_str(), val.second);
    }

    for (auto& val : mFloatDefaults)
    {
        settings->setDefaultFloat(val.first, val.second);
        settings->setDefaultFloat((std::string(kDefaultPrefix) + val.first).c_str(), val.second);
    }

    for (auto& val : mStringDefaults)
    {
        settings->setDefaultString(val.first, val.second);
        settings->setDefaultString((std::string(kDefaultPrefix) + val.first).c_str(), val.second);
    }

    for (auto& val : mStringArrayDefaults)
    {
        settings->setDefaultStringArray(val.first, val.second.first, val.second.second);
        settings->setDefaultStringArray(
            (std::string(kDefaultPrefix) + val.first).c_str(), val.second.first, val.second.second);
    }
}

void PhysXSettings::resetAllSettings()
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();

    resetSettingsSection(kSettingsPath);
    resetSettingsSection(kPersistentSettingsPath);

    // external defaults
    settings->setInt(kSettingMinFrameRate, settings->getAsInt(kSettingMinFrameRateDefault));
}

void PhysXSettings::resetSettingsInPreferences()
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();

    resetSettingAtPath(kSettingAutocreatePhysicsScene);
    resetSettingAtPath(kSettingResetOnStop);
    resetSettingAtPath(kSettingUseActiveCudaContext);
    resetSettingAtPath(kSettingCudaDevice);

    resetSettingAtPath(kSettingNumThreads);

    resetSettingAtPath(kSettingExposeProfilerData);

    resetSettingAtPath(kSettingUseLocalMeshCache);
    resetSettingAtPath(kSettingLocalMeshCacheSizeMB);

    resetSettingAtPath(kSettingUjitsoCookingDevKey);
    resetSettingAtPath(kSettingUjitsoCollisionCooking);
    resetSettingAtPath(kSettingUjitsoRemoteCacheEnabled);
    resetSettingAtPath(kSettingUjitsoCookingMaxProcessCount);

    resetSettingAtPath(kSettingPhysxDispatcher);
    resetSettingAtPath(kSettingDefaultSimulator);

    resetSettingAtPath(kSettingDebugVisSimplifyAtDistance);
    resetSettingAtPath(kSettingDebugVisQueryUsdrtForTraversal);
}


void PhysXSettings::resetSettingsInStage()
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();

    resetSettingAtPath(kSettingUpdateToUsd);
    resetSettingAtPath(kSettingUpdateVelocitiesToUsd);
    resetSettingAtPath(kSettingOutputVelocitiesLocalSpace);
    resetSettingAtPath(kSettingUpdateParticlesToUsd);

    resetSettingAtPath(kSettingMinFrameRate);
    resetSettingAtPath(kSettingJointBodyTransformCheckTolerance);
    resetSettingAtPath(kSettingEnableExtendedJointAngles);

    resetSettingAtPath(kSettingMouseInteractionEnabled);
    resetSettingAtPath(kSettingMouseGrab);
    resetSettingAtPath(kSettingMouseGrabIgnoreInvisible);
    resetSettingAtPath(kSettingMouseGrabWithForce);
    resetSettingAtPath(kSettingMousePush);
    resetSettingAtPath(kSettingMousePickingForce);

    resetSettingAtPath(kSettingDebugVisSimplifyAtDistance);
    resetSettingAtPath(kSettingDebugVisQueryUsdrtForTraversal);

    resetSettingAtPath(kSettingCollisionApproximateCones);
    resetSettingAtPath(kSettingCollisionApproximateCylinders);
}

void PhysXSettings::resetSettingAtPath(const char* path)
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();
    std::string defaultPath = std::string(kDefaultPrefix) + path;

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
        CARB_LOG_ERROR("Can't reset physic settings %s! Path not found.", path);
    default:
        CARB_LOG_ERROR("Can't reset physic settings %s! Type not supported.", path);
        return;
    }
}

void PhysXSettings::resetSettingsSection(const char* path)
{
    auto* settings = carb::getCachedInterface<carb::settings::ISettings>();

    std::string defaultPath = std::string(kDefaultPrefix) + path;

    settings->update(path, settings->getSettingsDictionary(defaultPath.c_str()), nullptr,
                     carb::dictionary::overwriteOriginalWithArrayHandling, nullptr);
}

} // namespace physx
} // namespace omni
