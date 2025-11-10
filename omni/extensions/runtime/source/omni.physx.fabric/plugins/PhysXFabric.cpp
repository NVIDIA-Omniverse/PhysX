// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS


#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <carb/settings/ISettings.h>
#include <omni/fabric/IFabric.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/IPhysxJoint.h>
#include <omni/physx/IPhysxSettings.h>
#include <private/omni/physx/IPhysxPrivate.h>
#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>
#include <omni/kit/IStageUpdate.h>

#include "FabricManager.h"

using namespace carb;
using namespace omni::physx;
using namespace pxr;

omni::kit::StageUpdatePtr gStageUpdate;
omni::kit::StageUpdateNode* gStageUpdateNode = nullptr;
carb::settings::ISettings* gSettings = nullptr;

FabricManager* gFabricManager = nullptr;

using namespace pxr;

class ExtensionImpl;

const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.fabric.plugin", "PhysX", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };
CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxFabric)
CARB_PLUGIN_IMPL_DEPS(carb::settings::ISettings,
                      omni::physx::IPhysxSimulation,
                      omni::physx::IPhysx,
                      omni::physx::IPhysxPrivate,
                      omni::physx::IPhysxJoint,
                      omni::fabric::IStageReaderWriter,
                      omni::fabric::ISimStageWithHistory)

void onAttach(long int stageId, double metersPerUnit, void*)
{
    if (gFabricManager)
    {
        gFabricManager->attach(stageId);
    }    
}

void onDetach(void*)
{
    if (gFabricManager)
    {
        gFabricManager->detach();
    }
}

void externalUpdate(float currentTime, float elapsedSecs)
{
    if (gFabricManager)
    {
        gFabricManager->update(currentTime, elapsedSecs);
    }
}

void forceUpdate(float currentTime, float elapsedSecs)
{
    if (gFabricManager)
    {
        gFabricManager->update(currentTime, elapsedSecs, true);
    }
}

void saveToUsd()
{
    if (gFabricManager)
    {
        gFabricManager->saveToUsd();
    }
}

void enableKinematicBodyTransformationUpdate(bool enable)
{
    if (gFabricManager)
    {
        gFabricManager->enableKinematicBodyTransformationUpdate(enable);
    }
}

bool isKinematicBodyTransformationUpdateEnabled()
{
    if (gFabricManager)
    {
        return gFabricManager->isKinematicBodyTransformationUpdateEnabled();
    }

    return true;
}


bool attachStage(long stageId)
{
    if (gFabricManager)
    {
        gFabricManager->attach(stageId);
        gFabricManager->resume();
    }
    return true;
}

void detachStage()
{
    if (gFabricManager)
    {
        gFabricManager->stop();
        gFabricManager->detach();
    }
}

void onResume(float, void*)
{
    if (gFabricManager)
    {
        gFabricManager->resume();
    }
}

void onPause(void*)
{
    if (gFabricManager)
    {
        gFabricManager->pause();
    }
}

void onStop(void*)
{
    if (gFabricManager)
    {
        gFabricManager->stop();
    }
}

void setupDefaultSettings()
{
    gSettings->setDefaultBool(kSettingFabricEnabled, true);
    gSettings->setDefaultBool(kSettingFabricUpdateTransformations, true);
    gSettings->setDefaultBool(kSettingFabricUpdateVelocities, true);
    gSettings->setDefaultBool(kSettingFabricUpdateJointStates, true);
    gSettings->setDefaultBool(kSettingFabricUpdateResiduals, true);
    gSettings->setDefaultBool(kSettingFabricUpdatePoints, true);
    gSettings->setDefaultBool(kSettingFabricUseGPUInterop, false);
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if (gStageUpdate)
        gStageUpdate->destroyStageUpdateNode(gStageUpdateNode);

    if (gFabricManager)
    {
        delete gFabricManager;
        gFabricManager = nullptr;
    }

    gSettings->setBool(kSettingFabricEnabled, false);
}

CARB_EXPORT void carbOnPluginStartup()
{
    gFabricManager = new FabricManager();

    carb::Framework* framework = carb::getFramework();
    gStageUpdate = omni::kit::getStageUpdate();
    gSettings = carb::getCachedInterface<carb::settings::ISettings>();
    setupDefaultSettings();

    gSettings->setBool(kSettingFabricEnabled, true);

    omni::kit::StageUpdateNodeDesc desc = { 0 };
    desc.displayName = "PhysXFabric";
    desc.order = omni::kit::update::eIUsdStageUpdateFabricPostPhysics;
    desc.onAttach = onAttach;
    desc.onDetach = onDetach;
    desc.onResume = onResume;
    desc.onPause = onPause;
    desc.onStop = onStop;
    if (gStageUpdate)
        gStageUpdateNode = gStageUpdate->createStageUpdateNode(desc);

    // Display simulation output info screen to the user while Fabric is active
    if (gSettings->getAsBool(kSettingAutoPopupSimulationOutputWindow))
    {
        gSettings->setBool(kSettingDisplaySimulationOutput, true);
    }

    // Connectivity needs to be populated when FSD is off
    gSettings->setBool("/app/settings/fabricConnectivityWithoutFSD", true);
}

void fillInterface(IPhysxFabric& iface)
{
    iface.attachStage = attachStage;
    iface.detachStage = detachStage;
    iface.update = externalUpdate;
    iface.saveToUsd = saveToUsd;
    iface.forceUpdate = forceUpdate;
    iface.enableKinematicBodyTransformationUpdate = enableKinematicBodyTransformationUpdate;
    iface.isKinematicBodyTransformationUpdateEnabled = isKinematicBodyTransformationUpdateEnabled;
}
