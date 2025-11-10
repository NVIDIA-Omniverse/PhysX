// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/kit/SettingsUtils.h>

#include <carb/Defines.h>
#include <carb/Types.h>


// This is the Omni PVD message data structure.
#define OMNI_PVD_MESSAGE_LENGTH 2048

struct OmniPvdMessage
{
    char message[OMNI_PVD_MESSAGE_LENGTH];
    char file[OMNI_PVD_MESSAGE_LENGTH];
    uint32_t line;
    uint32_t type;
    uint32_t handle;
    char typeName[OMNI_PVD_MESSAGE_LENGTH];
    uint64_t frameId;
};

using OmniPvdMessages = std::vector<OmniPvdMessage>;


namespace omni
{
namespace physx
{

static constexpr char kOmniPvdImportedOvd[] = "/physics/omniPvdImportedOvd";
static constexpr char kOmniPvdOvdForBaking[] = "/physics/omniPvdOvdForBaking";

static constexpr char kOmniPvdLastImportDirectory[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdImportDirectory";
static constexpr char kOmniPvdUsdCacheDirectory[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdUsdCacheDirectory";
static constexpr char kOmniPvdPhysXUsdDirectory[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdPhysXUsdDirectory";
static constexpr char kOmniPvdInvalidateCache[] = "/physics/omniPvdInvalidateCache";

static constexpr char kOmniPvdGizmoContactVizMode[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoContactVizMode";
static constexpr char kOmniPvdGizmoCenterOfMassVizMode[] =
    PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoCenterOfMassVizMode";
static constexpr char kOmniPvdGizmoJointVizMode[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoJointVizMode";
static constexpr char kOmniPvdGizmoBoundingBoxVizMode[] =
    PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoBoundingBoxVizMode";
static constexpr char kOmniPvdGizmoCoordinateSystemVizMode[] =
    PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoCoordinateSystemVizMode";
static constexpr char kOmniPvdGizmoVelocityVizMode[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoVelocityVizMode";
static constexpr char kOmniPvdGizmoTransparencyVizMode[] = "/physics/omniPvdGizmoTransparencyVizMode";

static constexpr char kOmniPvdGizmoGlobalScale[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoGlobalScale";
static constexpr char kOmniPvdGizmoContactScale[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoContactScale";
static constexpr char kOmniPvdGizmoCenterOfMassScale[] =
    PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoCenterOfMassScale";
static constexpr char kOmniPvdGizmoJointScale[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoJointScale";
static constexpr char kOmniPvdGizmoCoordinateSystemScale[] =
    PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoCoordinateSystemScale";
static constexpr char kOmniPvdGizmoVelocityScale[] = PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoVelocityScale";
static constexpr char kOmniPvdGizmoTransparencyScale[] =
    PERSISTENT_SETTINGS_PREFIX "/physics/omniPvdGizmoTransparencyScale";

static constexpr char kOmniPvdTimelineIsLegacyOvd[] = "/physics/omniPvdTimelineIsLegacyOvd";
static constexpr char kOmniPvdTimelineFrameMode[] = "/physics/omniPvdTimelineFrameMode";
static constexpr char kOmniPvdTimelinePlaybackState[] = "/physics/omniPvdTimelinePlaybackState";
static constexpr char kOmniPvdTimelineFrameDeltaMs[] = "/physics/omniPvdTimelineFrameDeltaMs";
static constexpr char kOmniPvdTimelineFramesPerSimStep[] = "/physics/omniPvdTimelineFramesPerSimStep";
static constexpr char kOmniPvdTimelineFrameId[] = "/physics/omniPvdTimelineFrameId";
static constexpr char kOmniPvdTimelineFrameIdMin[] = "/physics/omniPvdTimelineFrameIdMin";
static constexpr char kOmniPvdTimelineFrameIdMax[] = "/physics/omniPvdTimelineFrameIdMax";

static constexpr char kOmniPvdTimelineSimStepId[] = "/physics/omniPvdTimelineSimStepId";
static constexpr char kOmniPvdTimelineSimStepIdMin[] = "/physics/omniPvdTimelineSimStepIdMin";
static constexpr char kOmniPvdTimelineSimStepIdMax[] = "/physics/omniPvdTimelineSimStepIdMax";


/// omni.physx PVD interface
struct IPhysXPvd
{
    CARB_PLUGIN_INTERFACE("omni::physx::IPhysXPvd", 0, 1)

    /// Transform an OmniPvd input file to a USD stage.
    ///
    /// \param[in] the OmniPvd OVD binary full file path
    /// \param[in] the output directory of the output USD Stage
    /// \param[in] up axis of the USD stage, 0 = Y-axis, anything else Z-axis
    /// \param[in] non-zero = USDA format out, 0 = USD format out
    /// \param[in] non-zero = write the output to a file
    /// \return Whether the operation was successful
    bool(CARB_ABI* ovdToUsd)(char* omniPvdFile, char* usdStageDir, int upAxis, int isUSDA);

    /// Transform an OmniPvd input file, in combination with the current active Stage into a USD over layer.
    ///
    /// \param[in] the OmniPvd OVD binary full file path
    /// \return Whether the operation was successful
    bool(CARB_ABI* ovdToUsdOver)(char* omniPvdFile);

    /// Transform an OmniPvd input file into a new USD stage with an over layer.
    ///
    /// This function:
    /// 1. Creates a new over layer in the specified output directory
    /// 2. Creates a new stage with the over layer as root
    /// 3. Processes all sublayers from the current stage, converting any anonymous layers to real layers
    /// 4. Applies all the overrides from the OmniPvd DOM state
    /// 5. Saves everything to disk
    ///
    /// \param[in] omniPvdFile The OmniPvd OVD binary full file path
    /// \param[in] inputStageFile The input USD stage file full path
    /// \param[in] outputDir The directory where all output files will be saved
    /// \param[in] outputStageFilename The name of the output stage file (will be normalized to end with .usda)
    /// \param[in] startTime The start time of the simulation range to include in the over layer
    /// \param[in] stopTime The end time of the simulation range to include in the over layer
    /// \param[in] newLayersAreASCII Whether to create ASCII layers
    /// \param[in] verifyOverLayer Whether to verify that the over layer contains the Prims also in the simulation.usda layer
    /// \return Whether the operation was successful
    bool(CARB_ABI* ovdToUsdOverWithLayerCreation)(char* omniPvdFile, char* inputStageFile  ,char* outputDir, char* outputStageFilename,
        float startTime, float stopTime, bool newLayersAreASCII, bool verifyOverLayer);

    /// Transform an OmniPvd input file to a USD stage in memory.
    ///
    /// \param[in] omniPvdFile The OmniPvd OVD binary full file path
    /// \param[in] upAxis Up axis of the USD stage, 0 = Y-axis, anything else Z-axis
    /// \return The USD stage ID on success, 0 on failure
    long(CARB_ABI* ovdToUsdInMemory)(char* omniPvdFile, int upAxis);

    /// Load and parse the OVD file without converting to a USD file.
    /// \param[in] the OmniPvd OVD binary full file path

    /// \return Whether the operation was successful
    bool(CARB_ABI* loadOvd)(char* omniPvdFile);

    /// Return the messages received in the OVD stream.

    /// \return a list of messages received in the OVD stream.
    const OmniPvdMessages&(CARB_ABI* getMessages)();

    /// Clear the list of messages received in the OVD stream.

    /// \return the list of messages received in the OVD stream.
    void(CARB_ABI* clearMessages)();
};

} // namespace physx
} // namespace omni
