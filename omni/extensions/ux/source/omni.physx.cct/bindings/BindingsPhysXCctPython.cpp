// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physx/IPhysxCct.h>

#include <carb/BindingsPythonUtils.h>
#include <carb/input/InputTypes.h>

#include <memory>
#include <string>
#include <vector>

CARB_BINDINGS("carb.physx.cct.python")

DISABLE_PYBIND11_DYNAMIC_CAST(carb::events::IEventStream)

namespace
{

PYBIND11_MODULE(_physxCct, m)
{
    const char* docString;
    
    using namespace carb;
    using namespace omni::physx;

    py::enum_<CctEvent::Enum>(m, "CctEvent", R"(
        Cct events used by Cct event stream.
        )")
        .value("COLLISION_DOWN", CctEvent::eCollisionDown,
            R"(Character controller collision down event, during the last cct move a collision below the CCT was found and the status changed; contains the following in a dictionary::

                'cctPath':int2 - Usd path to the CCT decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.                                
                'collision':bool - Reports current collision with CCT.
            )")
        .value("COLLISION_UP", CctEvent::eCollisionUp,
            R"(Character controller collision down event, during the last cct move a collision above the CCT was found and the status changed; contains the following in a dictionary::

                'cctPath':int2 - Usd path to the CCT decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.                                
                'collision':bool - Reports current collision with CCT.
            )")
        .value("COLLISION_SIDES", CctEvent::eCollisionSides,
            R"(Character controller collision down event, during the last cct move a collision on a side of the CCT was found and the status changed; contains the following in a dictionary::

                'cctPath':int2 - Usd path to the CCT decoded into two ints. PhysicsSchemaTools.decodeSdfPath will return SdfPath.                                
                'collision':bool - Reports current collision with CCT.
            )");

    m.doc() = "pybind11 carb.physx.cct bindings";

    auto physxCct = defineInterfaceClass<IPhysxCct>(m, "IPhysxCct", "acquire_physx_cct_interface", "release_physx_cct_interface");

    docString = R"(
        Set the position of the center of the controller's collision. This does not check for collisions.

        Args:
            path: Path of the controller's prim.
            position: New center position of the controller.
    )";
    physxCct.def("set_position", wrapInterfaceFunction(&IPhysxCct::setPosition), docString);

    docString = R"(
        Adds current PhysicsScene's gravity to the controller's move vector.

        Args:
            path: Path of the controller's prim.
    )";
    physxCct.def("enable_gravity", wrapInterfaceFunction(&IPhysxCct::enableGravity), docString);

    docString = R"(
        Adds custom gravity to the controller's move vector.

        Args:
            path: Path of the controller's prim.
            gravity: Custom gravity vector.
    )";
    physxCct.def("enable_custom_gravity", wrapInterfaceFunction(&IPhysxCct::enableCustomGravity), docString);

    docString = R"(
        Disables gravity set through apply_gravity or apply_custom_gravity.

        Args:
            path: Path of the controller's prim.
    )";
    physxCct.def("disable_gravity", wrapInterfaceFunction(&IPhysxCct::disableGravity), docString);

    docString = R"(
        Gets if gravity is being added to the controller's move vector.

        Args:
            path: Path of the controller's prim.
    )";
    physxCct.def("has_gravity_enabled", wrapInterfaceFunction(&IPhysxCct::hasGravityEnabled), docString);

    docString = R"(
        Enable first person camera support for a controller. Hides mouse cursor, sets the controller's capsule as a guide and uses the controller's camera transformation when transforming move vector in local space mode. If you want to use multiple controllers at the same time use setViewportIndex.

        Args:
            path: Path of the controller's prim.
            camera_path: Camera path.
    )";
    physxCct.def("enable_first_person", wrapInterfaceFunction(&IPhysxCct::enableFirstPerson), docString);

    docString = R"(
        Disable first person camera support.

        Args:
            path: Path of the controller's prim.
    )";
    physxCct.def("disable_first_person", wrapInterfaceFunction(&IPhysxCct::disableFirstPerson), docString);

    docString = R"(
        Sets if PhysxCharacterControllerAPI:MoveTarget attribute will be considered as a local or world space vector. Local space move vector is transformed with the controller's capsule transformation (or camera transformation in the case of first person mode).

        Args:
            use: World space is used when true, local space if false.
    )";
    physxCct.def("enable_worldspace_move", wrapInterfaceFunction(&IPhysxCct::enableWorldSpaceMove), docString);

    docString = R"(
        Set the controller's height. This does not check for collisions.

        Args:
            path: Path of the controller's prim.
            val: New controller height.
    )";
    physxCct.def("set_controller_height", wrapInterfaceFunction(&IPhysxCct::setControllerHeight), docString);

    docString = R"(
        Gets controller's height.

        Args:
            path: Path of the controller's prim.

        Returns:
            float: The height of the controller.
    )";
    physxCct.def("get_controller_height", wrapInterfaceFunction(&IPhysxCct::getControllerHeight), docString);

    docString = R"(
        Simulation event stream sending various simulation events defined in SimulationEvent enum.

        Returns:
            Event stream sending the simulation events.
    )";
    physxCct.def("get_cct_event_stream", wrapInterfaceFunction(&IPhysxCct::getCctEventStream), docString);

    docString = R"(
        Removes a character controller's data from the manager.

        Args:
            path: Path of the controller's prim.
    )";
    physxCct.def("remove_cct", wrapInterfaceFunction(&IPhysxCct::removeCct), docString);

    docString = R"(
        Adds character controller to the manager. Use if not calling any of the other methods at least once.

        Args:
            path: Path of the controller's prim.
    )";
    physxCct.def("activate_cct", wrapInterfaceFunction(&IPhysxCct::activateCct), docString);

    docString = R"(
        Move a controller by a given vector each frame (internally sets CharacterControllerAPI:MoveTarget). The vector is transformed from local to world space by either the controller's capsule or camera (if in first person mode) transformation. Use enable_worldspace_move to skip the local->world transform.

        Args:
            path: Path of the controller's prim.
            displacement: Displacement vector.
    )";
    physxCct.def("set_move", wrapInterfaceFunction(&IPhysxCct::setMove), docString);
}
} // namespace
