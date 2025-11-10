// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "OmniPvdDefines.h"
#include "OmniPvdOvdParserConfig.h"

#include <omni/renderer/IDebugDraw.h>
#include <omni/timeline/ITimeline.h>

#include <carb/eventdispatcher/IEventDispatcher.h>
#include <carb/tasking/TaskingUtils.h>

struct OmniPVDDebugVizData;

struct OmniPVDDebugGizmoSelection
{
    enum Enum
    {
        eBoundingBox,
        eCenterOfMass,
        eVelocity,
        eCoordinateSystem,
        eJoint,
        eContactPoint,
        eContactErrorPoint,
        eContactNormal,
        eContactDistance,
        eRestDistance,
        eTransparency,
        eNbrEnums
    };
};

struct OmniPVDDebugGizmoType
{
    enum Enum
    {
        eBoundingBox,
        eContact,
        eCenterOfMass,
        eVelocity,
        eCoordinateSystem,
        eJoint,
        eTransparency,
        eNbrEnums
    };
};

struct OmniPVDDebugVizMode
{
    enum Enum
    {
        eNone,
        eSelected,
        eAll,
        eNbrEnums
    };
};

struct OmniPVDDebugGizmoRenderModality
{
    enum Enum
    {
        eDraw,
        eErase
    };
};

struct OmniPVDTimelineFrameMode
{
    enum Enum
    {
        ePostSim,
        ePreSim,
        ePreSimAndPostSim,
        ePostSimLegacy,
        eNbrEnums
    };
};

struct OmniPVDTimelinePlaybackState
{
    enum Enum
    {
        eStopped,
        ePlaying,
        ePaused,
        eNbrEnums
    };
};


struct OmniPVDDebugLines
{
    OmniPVDDebugLines();
    void setDebugVizPointer(OmniPVDDebugVizData* debugViz);
    void initLines();
    void initLineInstances();
    void releaseLines();
    void startRenderLines(double timeCode);
    void setVizMode(OmniPVDDebugVizMode::Enum state);
    void setScale(float scale);
    void drawLine(const float* pos0, const float* pos1, const uint32_t color, const float lineWidth);
    void drawLineVarying(const float* pos0,
                         const float* pos1,
                         const uint32_t color0,
                         const uint32_t color1,
                         const float lineWidth0,
                         const float lineWidth1);

    ////////////////////////////////////////////////////////////////////////////////
    // Return the normalized direction in the incoming direction parameter
    // If a normal could not be calculated a null pointer will be returned
    ////////////////////////////////////////////////////////////////////////////////
    float* drawNormal(const float* pos, float* direction, const float normScale, const uint32_t color, const float lineWidth);

    void drawBox(const float* bbox, const uint32_t color, const float lineWidth);

    omni::renderer::SimplexBuffer mLineBuffer;
    omni::renderer::RenderInstanceBuffer mLineInstanceBuffer;
    int mNbrLines;
    float mScale;

    OmniPVDDebugVizMode::Enum mRenderedVizMode;
    OmniPVDDebugVizMode::Enum mVizMode;
    OmniPVDDebugVizData* mOmniPVDDebugVizPtr;
};

struct OmniPVDDebugVizData
{
    OmniPVDDebugVizData();
    ~OmniPVDDebugVizData();
    void reset();
    void initMutex();
    void releaseMutex();
    void initInterfaces();
    void releaseInterfaces();
    void initEventSubs();
    void releaseEventSubs();
    void releaseDebugViz();
    void tickStageDirty();
    void setGizmoVizMode(OmniPVDDebugGizmoSelection::Enum gizmo, OmniPVDDebugVizMode::Enum vizMode);
    void setGizmoScale(OmniPVDDebugGizmoSelection::Enum gizmo, float scale);
    void setGizmoScale(float scale);
    void updateGizmoVizModes();

    bool getClosingState();
    void setClosingState(bool closing);

    bool mIsClosing;

    carb::tasking::MutexWrapper* mNextStateMutex{ nullptr };

    uint64_t mStageDirtyEvent; // stage load, unload, play, stop, prim selection changed or gizmo user drop down
                               // selection changed
    uint64_t mProcessedStageDirtyEvent;
    double mProcessedTimeCode;

    std::array<carb::eventdispatcher::ObserverGuard, 3> mStageEvtSub;
    carb::events::ISubscriptionPtr mTimelineEvtSub;
    omni::renderer::IDebugDraw* mDebugDraw;
    omni::timeline::TimelinePtr mTimeline;

    OmniPVDDebugLines mGizmos[static_cast<int>(OmniPVDDebugGizmoSelection::eNbrEnums)];
    float mGizmoScale;
    bool mHasInterfaces;
};
