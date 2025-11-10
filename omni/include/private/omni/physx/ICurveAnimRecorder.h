// SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/Span.h>
#include <omni/String.h>

#include <omni/physx/IPhysxSettings.h>

namespace omni
{

namespace physx
{

struct ICurveAnimRecorder
{
    CARB_PLUGIN_INTERFACE("omni::physx::ICurveAnimRecorder", 0, 1)

    enum class RecordingReturnCode : int
    {
        RECORDING_SUCCESS,
        NO_CHANGES_DETECTED,
        PLUGIN_NOT_LOADED,
        RECORDING_NOT_RUNNING,
        TARGET_LAYER_ERROR,
        ERROR_WRITING_USD,
    };

    static constexpr char kSessionSubLayerName[] = "ClashStageRecordedData";

    ///  @brief start recording
    void(CARB_ABI* startRecording)();

    /// @brief stop recording, save recording to the layer kSessionSubLayerName.
    /// @param dropCache True to drop recording result. False to keep recording result.
    /// @param copyAlsoUnrecordedAttribs True to copy unrecorded USD attributes to the target layer.
    ICurveAnimRecorder::RecordingReturnCode(CARB_ABI* stopRecording)(bool dropCache, bool copyAlsoUnrecordedAttribs);

    ///  @brief Returns the state of recording. True when the recording is in progress, false if there is no recording
    ///  in progress.
    bool(CARB_ABI* isRecording)();

    /// @brief Set Frame Number to Record
    void(CARB_ABI* setRecordingFrameNum)(unsigned int frameNum);

    /// @brief Set Recording Scope (stage id and prims paths)
    void(CARB_ABI* setRecordingScope)(long int stageId, omni::span<const uint64_t> primPaths);

    /// @brief Clear Recording Scope
    void(CARB_ABI* clearRecordingScope)();

    /// Returns recording layer identifier
    omni::string(CARB_ABI* getRecodingSessionLayerName)();

    /// @brief Resets overridden prim prop deltas in the session layer made by anim extensions.
    bool(CARB_ABI* resetOverriddenPrimDeltas)();
};

DEFINE_PERSISTENT_PHYSX_SETTING(kSettingsLoggingEnabled, "/clashdetectionAnimLoggingEnabled")
static constexpr bool kSettingsLoggingEnabledDefaultVal{ false };

} // namespace physx
} // namespace omni
