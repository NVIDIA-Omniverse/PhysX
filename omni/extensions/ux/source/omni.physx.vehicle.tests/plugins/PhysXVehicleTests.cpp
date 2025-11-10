// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include <carb/Framework.h>
#include <carb/PluginUtils.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxVehicle.h>
#include <private/omni/physx/IPhysxVehicleTests.h>


omni::physx::IPhysx* gPhysXInterface = nullptr;


const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.vehicle.tests.plugin", "PhysX Vehicle Tests", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxVehicleTesting)
CARB_PLUGIN_IMPL_DEPS(omni::physx::IPhysxVehicle,
                      omni::physx::IPhysx)


CARB_EXPORT void carbOnPluginStartup()
{
    carb::Framework* framework = carb::getFramework();
    gPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
}

CARB_EXPORT void carbOnPluginShutdown()
{
    // note: aquired interfaces are not released here as this causes warning messages "...already been released"
}

bool doesVehicleExist(const char* path)
{
    const omni::physx::usdparser::ObjectId vehicleId = gPhysXInterface->getObjectId(pxr::SdfPath(path), omni::physx::ePTVehicle);
    return (vehicleId != omni::physx::usdparser::kInvalidObjectId);
}

void fillInterface(omni::physx::IPhysxVehicleTesting& iface)
{
    iface.doesVehicleExist = doesVehicleExist;
}
