// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <omni/physx/IPhysxStageUpdateNode.h>

#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.stageupdate.python")

namespace
{

PYBIND11_MODULE(_physxStageUpdateNode, m)
{
    const char* docString;

    using namespace carb;
    using namespace omni::physx;

    m.doc() = "pybind11 carb.physx.stageupdate bindings";

    py::class_<IPhysxStageUpdateNode> physxStageUpdateNode = defineInterfaceClass<IPhysxStageUpdateNode>(
        m, "IPhysxStageUpdateNode", "acquire_physx_stage_update_node_interface", "release_physx_stage_update_node_interface");

    m.def("release_physx_stage_update_node_scripting", [](IPhysxStageUpdateNode* iface)
    {
        carb::getFramework()->releaseInterfaceWithClient("carb.scripting-python.plugin", iface); // OM-60917
    });

    docString = R"(
        Attach the PhysX StageUpdateNode to IStageUpdate
    )";
    physxStageUpdateNode.def("attach_node", wrapInterfaceFunction(&IPhysxStageUpdateNode::attachNode), docString);

    docString = R"(
        Detach the PhysX StageUpdateNode to IStageUpdate
    )";
    physxStageUpdateNode.def("detach_node", wrapInterfaceFunction(&IPhysxStageUpdateNode::detachNode), docString);


    docString = R"(
        Check if StageUpdateNode is attached to IStageUpdate

        Returns:
            bool: True if attached.
    )";
    physxStageUpdateNode.def("is_node_attached", wrapInterfaceFunction(&IPhysxStageUpdateNode::isNodeAttached), docString);

    docString = R"(
    Blocks time line events (play, resume, stop)
    )";
    physxStageUpdateNode.def("block_timeline_events", wrapInterfaceFunction(&IPhysxStageUpdateNode::blockTimeLineEvents), docString);

    docString = R"(
    Check if time line events (play, resume, stop) are blocked

    Returns:
        bool: True if blocked.
    )";
    physxStageUpdateNode.def("timeline_events_blocked", wrapInterfaceFunction(&IPhysxStageUpdateNode::timeLineEventsBlocked), docString);
}

} // namespace
