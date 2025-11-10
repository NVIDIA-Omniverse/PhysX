// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <private/omni/physx/IPhysxVehicleTests.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.vehicle.tests.python")

namespace
{

PYBIND11_MODULE(_physxVehicleTests, m)
{
    const char* docString;

    using namespace carb;
    using namespace omni::physx;

    m.doc() = "pybind11 carb.physx.vehicle.tests bindings";

    py::class_<IPhysxVehicleTesting> physxVehicleTesting = defineInterfaceClass<IPhysxVehicleTesting>(
        m, "IPhysxVehicleTesting", "acquire_physx_vehicle_testing_interface", "release_physx_vehicle_testing_interface");

    docString = R"(
        Check whether a vehicle was created successfully.

        Note:
            Should only be called after the simulation has been started.

        Args:
            path: Vehicle USD path.

        Returns:
            True if the vehicle was created successfully, else False.
    )";
    physxVehicleTesting.def("does_vehicle_exist", wrapInterfaceFunction(&IPhysxVehicleTesting::doesVehicleExist),
                            docString, py::arg("path"));
}

} // namespace
