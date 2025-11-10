// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include <private/omni/physx/IPhysxPvd.h>
#include <carb/BindingsPythonUtils.h>

CARB_BINDINGS("carb.physx.pvd.python")

namespace
{

#define ADD_SETTING(attr_name, path) \
    m.attr(attr_name) = py::str(path);

PYBIND11_MODULE(_physxPvd, m)
{
    const char* docString;
    using namespace carb;
    using namespace omni::physx;

    //Settings
    ADD_SETTING("SETTING_OMNIPVD_IMPORTED_OVD", kOmniPvdImportedOvd);
    ADD_SETTING("SETTING_OMNIPVD_OVD_FOR_BAKING", kOmniPvdOvdForBaking);
    ADD_SETTING("SETTING_OMNIPVD_USD_CACHE_DIRECTORY", kOmniPvdUsdCacheDirectory);
    ADD_SETTING("SETTING_OMNIPVD_PHYSX_USD_DIRECTORY", kOmniPvdPhysXUsdDirectory);
    ADD_SETTING("SETTING_OMNIPVD_LAST_IMPORT_DIRECTORY", kOmniPvdLastImportDirectory);
    ADD_SETTING("SETTING_OMNIPVD_INVALIDATE_CACHE", kOmniPvdInvalidateCache);

    ADD_SETTING("SETTING_OMNIPVD_GIZMO_CONTACT_VIZMODE", kOmniPvdGizmoContactVizMode);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_VIZMODE", kOmniPvdGizmoCenterOfMassVizMode);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_JOINT_VIZMODE", kOmniPvdGizmoJointVizMode);    
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_BOUNDING_BOX_VIZMODE", kOmniPvdGizmoBoundingBoxVizMode);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_VIZMODE", kOmniPvdGizmoCoordinateSystemVizMode);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_VELOCITY_VIZMODE", kOmniPvdGizmoVelocityVizMode);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_TRANSPARENCY_VIZMODE", kOmniPvdGizmoTransparencyVizMode);

    ADD_SETTING("SETTING_OMNIPVD_GIZMO_GLOBAL_SCALE", kOmniPvdGizmoGlobalScale);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_CONTACT_SCALE", kOmniPvdGizmoContactScale);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_CENTER_OF_MASS_SCALE", kOmniPvdGizmoCenterOfMassScale);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_JOINT_SCALE", kOmniPvdGizmoJointScale);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_COORDINATE_SYSTEM_SCALE", kOmniPvdGizmoCoordinateSystemScale);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_VELOCITY_SCALE", kOmniPvdGizmoVelocityScale);
    ADD_SETTING("SETTING_OMNIPVD_GIZMO_TRANSPARENCY_SCALE", kOmniPvdGizmoTransparencyScale);

    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_IS_LEGACY_OVD", kOmniPvdTimelineIsLegacyOvd);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_FRAME_MODE", kOmniPvdTimelineFrameMode);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_PLAYBACK_STATE", kOmniPvdTimelinePlaybackState);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_FRAME_DELTA_MS", kOmniPvdTimelineFrameDeltaMs);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_FRAMES_PER_SIM_STEP", kOmniPvdTimelineFramesPerSimStep);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_FRAME_ID", kOmniPvdTimelineFrameId);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_FRAME_ID_MIN", kOmniPvdTimelineFrameIdMin);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_FRAME_ID_MAX", kOmniPvdTimelineFrameIdMax);

    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID", kOmniPvdTimelineSimStepId);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MIN", kOmniPvdTimelineSimStepIdMin);
    ADD_SETTING("SETTING_OMNIPVD_TIMELINE_SIM_STEP_ID_MAX", kOmniPvdTimelineSimStepIdMax);


    m.doc() = "pybind11 carb.physx.pvd bindings";

    py::class_<IPhysXPvd> physxPvd = defineInterfaceClass<IPhysXPvd>(
        m, "IPhysXPvd", "acquire_physx_pvd_interface", "release_physx_pvd_interface");

    docString = R"(
        Transforms an OmniPvd binary file into a USD Stage in memory.

        Args:
            omniPvdFile : the OmniPvd OVD binary full file path
            upAxis : up axis of the USD stage, 0 = Y-axis, anything else Z-axis

        Returns:
            Returns a long integer representing the USD stage ID on success, 0 on failure
    )";
    physxPvd.def("ovd_to_usd_in_memory", wrapInterfaceFunction(&IPhysXPvd::ovdToUsdInMemory), docString, 
        py::arg("omni_pvd_file"), py::arg("up_axis"));

    docString = R"(
        Transforms an OmniPvd binary file into a USD Stage.

        Args:
            omniPvdFile : the OmniPvd OVD binary full file path
            usdStageDir : the output directory of the output USD Stage
            upAxis : up axis of the USD stage, 0 = Y-axis, anything else Z-axis
            isUSDA : non-zero = USDA format out, 0 = USD format out

        Returns:
            true on success, false on failure
    )";
    physxPvd.def("ovd_to_usd", wrapInterfaceFunction(&IPhysXPvd::ovdToUsd), docString);

    docString = R"(
        Extracts an over layer from an OmniPvd binary file and adds it to the current active Stage.

        Args:
            omniPvdFile : the OmniPvd OVD binary full file path

        Returns:
            true on success, false on failure
    )";
    physxPvd.def("ovd_to_usd_over", wrapInterfaceFunction(&IPhysXPvd::ovdToUsdOver), docString);

    docString = R"(
        Creates a new USD stage with an over layer from an OmniPvd binary file.

        Args:
            omniPvdFile : the OmniPvd OVD binary full file path
            outputDir : the output directory where files will be saved
            stageFilename : the name of the output stage file (will be normalized to end with .usda)
            startTime : the start time of the simulation range to include in the over layer
            stopTime : the end time of the simulation range to include in the over layer
            verifyOverLayer : whether to verify that the over layer contains the Prims also in the simulation.usda layer
        Returns:
            true on success, false on failure
    )";
    physxPvd.def("ovd_to_usd_over_with_layer_creation", wrapInterfaceFunction(&IPhysXPvd::ovdToUsdOverWithLayerCreation), docString);

    docString = R"(
        Loads the input OmniPVD binary file and parses out the messages received. Does not
        covert to a USD stage or save a cached file.

        Args:
            omniPvdFile : the OmniPvd OVD binary full file path

        Returns:
            true on success, false on failure
    )";
    physxPvd.def("load_ovd", wrapInterfaceFunction(&IPhysXPvd::loadOvd), docString);

    docString = R"(
        Retrieve the list of messages in the OVD stream.

        Returns:
            A list of messages contained in the OVD stream in a dictionary format.
            The following keys can be used to reference the corresponding values:
            "message": The message string itself.
            "file": The name of the source file the message was generated in.
            "line": The line number in the source file where the messages was generated.
            "type": The message category type.
            "typeName": The message category type name string if an optional class handle was sent with the message.
            "frameId": The frame ID for the time when the message occurred.
            
            If an error occurs, the list will be empty.
    )";
    physxPvd.def("get_messages", [](IPhysXPvd* omniPVD)
    {
        const OmniPvdMessages& omniPvdMessages = omniPVD->getMessages();

        py::list messages(omniPvdMessages.size());
        size_t i = 0;

        for (const auto pvdMessage : omniPvdMessages)
        {
            py::dict message;
            message["message"] = pvdMessage.message;
            message["file"] = pvdMessage.file;
            message["line"] = pvdMessage.line;
            message["type"] = pvdMessage.type;
            message["typeName"] = pvdMessage.typeName;
            message["frameId"] = pvdMessage.frameId;
            messages[i] = message;
            i++;
        }

        return messages;
    },
    docString);

    docString = R"(
        Reset the list of messages parsed from the OVD stream.
    )";
    physxPvd.def("clear_messages", [](IPhysXPvd* omniPVD)
    {
        omniPVD->clearMessages();
    },
    docString);
}
} // namespace
