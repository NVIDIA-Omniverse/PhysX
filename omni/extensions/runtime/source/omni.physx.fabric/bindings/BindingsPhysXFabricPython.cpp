// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/IPhysxSettings.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.fabric.python")

using namespace omni::physx;

namespace
{

PYBIND11_MODULE(_physxFabric, m)
{
    const char* docString;
    
    using namespace carb;

    m.doc() = "pybind11 carb.physx.fabric bindings";

    m.attr("SETTING_FABRIC_ENABLED") = py::str(kSettingFabricEnabled);
    m.attr("SETTING_FABRIC_UPDATE_TRANSFORMATIONS") = py::str(kSettingFabricUpdateTransformations);
    m.attr("SETTING_FABRIC_UPDATE_VELOCITIES") = py::str(kSettingFabricUpdateVelocities);
    m.attr("SETTING_FABRIC_UPDATE_JOINT_STATES") = py::str(kSettingFabricUpdateJointStates);
    m.attr("SETTING_FABRIC_UPDATE_POINTS") = py::str(kSettingFabricUpdatePoints);
    m.attr("SETTING_FABRIC_USE_GPU_INTEROP") = py::str(kSettingFabricUseGPUInterop);
    m.attr("SETTING_FABRIC_UPDATE_RESIDUALS") = py::str(kSettingFabricUpdateResiduals);

    auto physxFabric = defineInterfaceClass<IPhysxFabric>(m, "PhysXFabric", "acquire_physx_fabric_interface", "release_physx_fabric_interface");
    m.def("release_physx_fabric_interface_scripting", [](IPhysxFabric* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    docString = R"(
        Attach USD stage.

        Args:
            id (int): Stage ID - get with UsdUtils.StageCache.Get().GetId(stage).ToLongInt()
    )";
    physxFabric.def("attach_stage", wrapInterfaceFunction(&IPhysxFabric::attachStage), docString);

    docString = R"(
        Detach USD stage.
    )";
    physxFabric.def("detach_stage", wrapInterfaceFunction(&IPhysxFabric::detachStage), docString);

    docString = R"(
        Update fabric from PhysX data.

        Args:
            currentTime (float): Unused
            elapsedSecs (float): Unused
    )";
    physxFabric.def("update", wrapInterfaceFunction(&IPhysxFabric::update), docString);

    docString = R"(
        Save fabric data into USD.
    )";
    physxFabric.def("save_to_usd", wrapInterfaceFunction(&IPhysxFabric::saveToUsd), docString);

    docString = R"(
        Update fabric from PhysX data even if simulation did not run.
        
        Args:
            currentTime (float): Unused
            elapsedSecs (float): Unused
    )";
    physxFabric.def("force_update", wrapInterfaceFunction(&IPhysxFabric::forceUpdate), docString);
    docString = R"(
        Enable update of Fabric transformations for kinematic bodies

        Note: For USD updates, the update for kinematic body transformations
            cannot happen as this would break the time samples for example. Physics 
            simulation should not be responsible for updating kinematic body transformations
            as physics is not changing the transformation, the user is. However
            in case of Fabric the update can be complex, hence omni.physx.fabric
            offers a way how to update both dynamic and kinematic bodies, though 
            strictly speaking this is not standard.
        
        Note: Changing this while simulation is already initialized will not have any impact
    
        Note: As this is helping to resolve issues in Isaac, its enabled by default
        
        Args:
            enable (bool): Enable update for kinematic body transformation in Fabric
    )";
    physxFabric.def("enable_kinematic_body_transformation_update", wrapInterfaceFunction(&IPhysxFabric::enableKinematicBodyTransformationUpdate), docString);
    docString = R"(
        Checks whether update of Fabric transformations for kinematic bodies is enabled
        
        Returns:
            True if update for kinematic bodies is enabled
    )";
    physxFabric.def("is_kinematic_body_transformation_update_enabled", wrapInterfaceFunction(&IPhysxFabric::isKinematicBodyTransformationUpdateEnabled), docString);
}
} // namespace
