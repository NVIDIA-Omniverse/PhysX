// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <private/omni/physx/IPhysxAssetValidator.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("omni.physx.asset_validator.python")

namespace
{

PYBIND11_MODULE(_physxAssetValidator, m)
{
    const char* docString;

    using namespace carb;
    using namespace omni::physx;

    m.doc() = "pybind11 omni.physx.asset_validator bindings";

    auto assetValidatorIface = defineInterfaceClass<IPhysxAssetValidator>(
        m, "IPhysxAssetValidator", "acquire_physx_asset_validator_interface", "release_physx_asset_validator_interface");

    m.def("release_physx_asset_validator_interface_scripting", [](IPhysxAssetValidator* iface) {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    // joint state checker

    docString = R"(
        Check if Joint States for given articulation root api are coherent with body transforms for the articulation.
        Stage must be already parsed and PhysX objects already created prior to calling this function (use IPhysX::forceLoadPhysicsFromUsd or equivalent)

        Args:
            stage_id: Stage Id containing the USD Path to check
            prim_id: USD Path of prim with an applied ArticulationRootAPI

        Returns:
            bool: True if the joint states are coherent with body transformations of the given ArticulationRootAPI
    )";
    assetValidatorIface.def("joint_state_is_valid", wrapInterfaceFunction(&IPhysxAssetValidator::jointStateIsValid),
                            docString, py::arg("stage_id"), py::arg("prim_id"));

    docString = R"(
        Modifies bodies of the articulations to make them match what's specified by applied Joint States APIs. 
        Stage must be already parsed and PhysX objects already created prior to calling this function (use IPhysX::forceLoadPhysicsFromUsd or equivalent)

        Args:
            stage_id: Stage Id containing the USD Path to check
            prim_id: USD Path of prim with an applied ArticulationRootAPI

    )";
    assetValidatorIface.def("joint_state_apply_fix", wrapInterfaceFunction(&IPhysxAssetValidator::jointStateApplyFix),
                            docString, py::arg("stage_id"), py::arg("prim_id"));

    // backward compatibility checker

    docString = R"(
        Check if the stage is backward compatible with the current version of the asset validator.
    )";
    assetValidatorIface.def("backward_compatibility_check", wrapInterfaceFunction(&IPhysxAssetValidator::backwardCompatibilityCheck),
                            docString, py::arg("stage_id"));

    docString = R"(
        Get the log of the backward compatibility check.
    )";
    assetValidatorIface.def("get_backward_compatibility_log", wrapInterfaceFunction(&IPhysxAssetValidator::getBackwardCompatibilityLog),
                            docString);

    docString = R"(
        Run backward compatibility check on the stage.
    )";
    assetValidatorIface.def("run_backward_compatibility_check", wrapInterfaceFunction(&IPhysxAssetValidator::runBackwardCompatibilityCheck),
                            docString, py::arg("stage_id"));

    docString = R"(
        Check if resulting convex mesh is GPU compatible for given prim.
        Stage must be already parsed and PhysX objects already created prior to calling this function (use IPhysX::forceLoadPhysicsFromUsd or equivalent)

    )";
    assetValidatorIface.def("convex_gpu_compatibility_is_valid", wrapInterfaceFunction(&IPhysxAssetValidator::convexGPUCompatibilityIsValid),
                            docString, py::arg("stage_id"), py::arg("prim_id"));

}
} // namespace
