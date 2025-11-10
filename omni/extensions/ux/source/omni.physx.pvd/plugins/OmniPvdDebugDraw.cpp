// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <carb/Framework.h>
#include <omni/renderer/IDebugDraw.h>

#include <carb/Framework.h>
#include <carb/PluginUtils.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>
#include <omni/timeline/TimelineTypes.h>

#include <carb/Framework.h>

#include "OmniPvdDebugDraw.h"
#include "OmniPvdDrawGizmos.h"

const static carb::RStringKey kObserverName("omni.physx.pvd");

OmniPVDDebugLines::OmniPVDDebugLines()
{
    mLineBuffer = omni::renderer::IDebugDraw::eInvalidBuffer;
    mLineInstanceBuffer = omni::renderer::IDebugDraw::eInvalidBuffer;
    mVizMode = OmniPVDDebugVizMode::eNone;
    mNbrLines = 0;
    mOmniPVDDebugVizPtr = 0;
    mScale = 1.0f;
}

void OmniPVDDebugLines::setDebugVizPointer(OmniPVDDebugVizData *debugViz)
{
    mOmniPVDDebugVizPtr = debugViz;
}

void OmniPVDDebugLines::initLines()
{
    mNbrLines = 0;
    if (mLineBuffer != ::omni::renderer::IDebugDraw::eInvalidBuffer) return;
    if (!mOmniPVDDebugVizPtr) return;
    if (!mOmniPVDDebugVizPtr->mDebugDraw) return;

    mLineBuffer = mOmniPVDDebugVizPtr->mDebugDraw->allocateLineBuffer(size_t(32768));
}

void OmniPVDDebugLines::initLineInstances()
{
    if (mLineInstanceBuffer != ::omni::renderer::IDebugDraw::eInvalidBuffer) return;
    if (mLineBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer) return;
    if (!mOmniPVDDebugVizPtr) return;
    if (!mOmniPVDDebugVizPtr->mDebugDraw) return;

    if (mLineInstanceBuffer == omni::renderer::IDebugDraw::eInvalidBuffer)
    {
        if (mNbrLines > 0)
        {
            mLineInstanceBuffer = mOmniPVDDebugVizPtr->mDebugDraw->allocateRenderInstanceBuffer(mLineBuffer, 1);
            if (mLineInstanceBuffer != omni::renderer::IDebugDraw::eInvalidBuffer)
            {
                float transform[16] = {};
                transform[0] = 1.f;
                transform[1 + 4] = 1.f;
                transform[2 + 8] = 1.f;
                transform[3 + 12] = 1.f;
                mOmniPVDDebugVizPtr->mDebugDraw->setRenderInstance(mLineInstanceBuffer, 0, transform, 0, 0);
            }
        }
    }
}

void OmniPVDDebugLines::releaseLines()
{
    mNbrLines = 0;
    if (mOmniPVDDebugVizPtr && mOmniPVDDebugVizPtr->mDebugDraw)
    {
        if (mLineInstanceBuffer != ::omni::renderer::IDebugDraw::eInvalidBuffer)
        {
            mOmniPVDDebugVizPtr->mDebugDraw->deallocateRenderInstanceBuffer(mLineInstanceBuffer);
            mLineInstanceBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;

        }
        if (mLineBuffer != ::omni::renderer::IDebugDraw::eInvalidBuffer)
        {
            mOmniPVDDebugVizPtr->mDebugDraw->deallocateLineBuffer(mLineBuffer);
            mLineBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
        }
    }
    else
    {
        mLineInstanceBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
        mLineBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
    }
}

void OmniPVDDebugLines::startRenderLines(double timeCode)
{
    releaseLines();
    initLines();
}

void OmniPVDDebugLines::setVizMode(OmniPVDDebugVizMode::Enum vizMode)
{
    mVizMode = vizMode;
}

void OmniPVDDebugLines::setScale(float scale)
{
    mScale = scale;
}

void OmniPVDDebugLines::drawLine(const float* pos0, const float* pos1, const uint32_t color, const float lineWidth)
{
    if (mLineBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer) return;
    if (!mOmniPVDDebugVizPtr) return;
    if (!mOmniPVDDebugVizPtr->mDebugDraw) return;


    carb::Float3 cpos0;
    carb::Float3 cpos1;
    cpos0.x = pos0[0];
    cpos0.y = pos0[1];
    cpos0.z = pos0[2];

    cpos1.x = pos1[0];
    cpos1.y = pos1[1];
    cpos1.z = pos1[2];

    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, cpos0, color, lineWidth, cpos1, color, lineWidth);
    mNbrLines++;
}

void OmniPVDDebugLines::drawLineVarying(const float* pos0, const float* pos1, const uint32_t color0, const uint32_t color1, const float lineWidth0, const float lineWidth1)
{
    if (mLineBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer) return;
    if (!mOmniPVDDebugVizPtr) return;
    if (!mOmniPVDDebugVizPtr->mDebugDraw) return;


    carb::Float3 cpos0;
    carb::Float3 cpos1;
    cpos0.x = pos0[0];
    cpos0.y = pos0[1];
    cpos0.z = pos0[2];

    cpos1.x = pos1[0];
    cpos1.y = pos1[1];
    cpos1.z = pos1[2];

    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, cpos0, color0, lineWidth0, cpos1, color1, lineWidth1);
    mNbrLines++;

}

float* OmniPVDDebugLines::drawNormal(const float* pos, float* direction, const float normScale, const uint32_t color, const float lineWidth)
{
    if (mLineBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer) return nullptr;
    if (!mOmniPVDDebugVizPtr) return nullptr;
    if (!mOmniPVDDebugVizPtr->mDebugDraw) return nullptr;

    const float normX = direction[0];
    const float normY = direction[1];
    const float normZ = direction[2];

    if ((normX != normX) || (normY != normY) || (normZ != normZ))
        return nullptr;

    const float normXSq = normX * normX;
    const float normYSq = normY * normY;
    const float normZSq = normZ * normZ;

    float normSum = normXSq + normYSq + normZSq;
    if (normSum > 0.0001f)
    {
        normSum = sqrtf(normSum);
        const float normScaleInv = 1.0f / normSum;
        const float normSumScaleInv = normScale * normScaleInv;

        carb::Float3 pos0;
        carb::Float3 pos1;

        pos0.x = pos[0];
        pos0.y = pos[1];
        pos0.z = pos[2];

        pos1.x = pos[0] + normX * normSumScaleInv;
        pos1.y = pos[1] + normY * normSumScaleInv;
        pos1.z = pos[2] + normZ * normSumScaleInv;

        mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
        mNbrLines++;

        direction[0] *= normScaleInv;
        direction[1] *= normScaleInv;
        direction[2] *= normScaleInv;

        return direction;
    }
    else
    {
        return 0;
    }
}

void OmniPVDDebugLines::drawBox(const float* bbox, const uint32_t color, const float lineWidth)
{
    if (mLineBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer) return;
    if (!mOmniPVDDebugVizPtr) return;
    if (!mOmniPVDDebugVizPtr->mDebugDraw) return;

    const float xmin = bbox[0];
    const float ymin = bbox[1];
    const float zmin = bbox[2];
    const float xmax = bbox[3];
    const float ymax = bbox[4];
    const float zmax = bbox[5];

    carb::Float3 pos0;
    carb::Float3 pos1;

    pos0.x = xmin; pos0.y = ymin; pos0.z = zmin;
    pos1.x = xmax; pos1.y = ymin; pos1.z = zmin;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymin; pos0.z = zmax;
    pos1.x = xmax; pos1.y = ymin; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymax; pos0.z = zmin;
    pos1.x = xmax; pos1.y = ymax; pos1.z = zmin;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymax; pos0.z = zmax;
    pos1.x = xmax; pos1.y = ymax; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymin; pos0.z = zmin;
    pos1.x = xmin; pos1.y = ymin; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymax; pos0.z = zmin;
    pos1.x = xmin; pos1.y = ymax; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmax; pos0.y = ymin; pos0.z = zmin;
    pos1.x = xmax; pos1.y = ymin; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmax; pos0.y = ymax; pos0.z = zmin;
    pos1.x = xmax; pos1.y = ymax; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymin; pos0.z = zmin;
    pos1.x = xmin; pos1.y = ymax; pos1.z = zmin;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmin; pos0.y = ymin; pos0.z = zmax;
    pos1.x = xmin; pos1.y = ymax; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmax; pos0.y = ymin; pos0.z = zmin;
    pos1.x = xmax; pos1.y = ymax; pos1.z = zmin;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;

    pos0.x = xmax; pos0.y = ymin; pos0.z = zmax;
    pos1.x = xmax; pos1.y = ymax; pos1.z = zmax;
    mOmniPVDDebugVizPtr->mDebugDraw->setLine(mLineBuffer, mNbrLines, pos0, color, lineWidth, pos1, color, lineWidth);
    mNbrLines++;
}

OmniPVDDebugVizData::OmniPVDDebugVizData()
{
    reset();
}

OmniPVDDebugVizData::~OmniPVDDebugVizData()
{
    releaseMutex();
}

void OmniPVDDebugVizData::reset()
{
    mIsClosing = false;
    mDebugDraw = nullptr;
    mGizmoScale = 1.0;
    mTimeline = nullptr;
    mHasInterfaces = false;
    mTimelineEvtSub = nullptr;

    const int nbrGizmos = OmniPVDDebugGizmoSelection::eNbrEnums;
    for (int i = 0; i < nbrGizmos; i++)
    {
        mGizmos[i].setDebugVizPointer(this);
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Immediately make the selection event dirty, just in case the OmniPvd interface
    // was loaded in between stuff got selected
    ////////////////////////////////////////////////////////////////////////////////
    mStageDirtyEvent = 1;
    mProcessedStageDirtyEvent = 0;
    mProcessedTimeCode = -1000000.0;
}

void OmniPVDDebugVizData::initMutex()
{
    if (!mNextStateMutex)
    {
        mNextStateMutex = new carb::tasking::MutexWrapper();
    }
}

void OmniPVDDebugVizData::releaseMutex()
{
    delete mNextStateMutex;
    mNextStateMutex = nullptr;
}


void OmniPVDDebugVizData::initInterfaces()
{
    if (mHasInterfaces)
    {
        return;
    }
    if (!mDebugDraw)
    {
        mDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();
    }
    if (!mTimeline)
    {
        mTimeline = omni::timeline::getTimeline();
    }
    if ((mDebugDraw == nullptr) || (mTimeline == nullptr))
    {
        return;
    }
    mHasInterfaces = true;
}

void OmniPVDDebugVizData::releaseInterfaces()
{
    ////////////////////////////////////////////////////////////////////////////////
    // Is this really enough to release the interfaces? Doubt (TM)
    ////////////////////////////////////////////////////////////////////////////////
    mDebugDraw = nullptr;
    mTimeline = nullptr;
}

void OmniPVDDebugVizData::initEventSubs()
{
    if (mTimelineEvtSub && mStageEvtSub[0].get() != carb::eventdispatcher::kInvalidObserver)
    {
        return;
    }
    if (!mHasInterfaces)
    {
        initInterfaces();
        if (!mHasInterfaces)
        {
            return;
        }
    }
    if (!mTimelineEvtSub)
    {
        mTimelineEvtSub = carb::events::createSubscriptionToPop(mTimeline->getTimelineEventStream(), [this](carb::events::IEvent* e)
        {
            if ((static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::ePlay)
                || (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::eStop)
                || (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::eCurrentTimeTicked)
                || (static_cast<omni::timeline::TimelineEventType>(e->type) == omni::timeline::TimelineEventType::eCurrentTimeChanged)
                )
            {
                tickStageDirty();
            }
        }, 0, "Timeline Scrub");
    }
    if (mStageEvtSub[0].get() == carb::eventdispatcher::kInvalidObserver)
    {
        auto ed = carb::getCachedInterface<carb::eventdispatcher::IEventDispatcher>();
        omni::usd::UsdContext* usdContext = omni::usd::UsdContext::getContext();
        if (usdContext)
        {
            mStageEvtSub = {
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                 usdContext->stageEventName(omni::usd::StageEventType::eOpened),
                                 [this](const auto&) {
                                     setClosingState(false);
                                     tickStageDirty();
                                 }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                 usdContext->stageEventName(omni::usd::StageEventType::eClosing),
                                 [this](const auto&) { setClosingState(true); }),
                ed->observeEvent(kObserverName, carb::eventdispatcher::kDefaultOrder,
                                 usdContext->stageEventName(omni::usd::StageEventType::eSelectionChanged),
                                 [this](const auto&) {
                                     setClosingState(false);
                                     tickStageDirty();
                                 }),
            };
        }
    }
}

void OmniPVDDebugVizData::releaseEventSubs()
{
    if (mTimelineEvtSub)
    {
        mTimelineEvtSub->unsubscribe();
        mTimelineEvtSub = nullptr;
    }
    mStageEvtSub = {};
}

void OmniPVDDebugVizData::releaseDebugViz()
{
    const int nbrGizmos = OmniPVDDebugGizmoSelection::eNbrEnums;
    for (int i = 0; i < nbrGizmos; i++)
    {
        mGizmos[i].releaseLines();
    }
}

void OmniPVDDebugVizData::tickStageDirty()
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    mStageDirtyEvent++;
}

void OmniPVDDebugVizData::setGizmoVizMode(OmniPVDDebugGizmoSelection::Enum gizmo, OmniPVDDebugVizMode::Enum selection)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    mGizmos[gizmo].setVizMode(selection);
    mStageDirtyEvent++;
}

void OmniPVDDebugVizData::setGizmoScale(OmniPVDDebugGizmoSelection::Enum gizmo, float scale)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    mGizmos[gizmo].setScale(scale);
    mStageDirtyEvent++;
}

void OmniPVDDebugVizData::setGizmoScale(float scale)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    mGizmoScale = scale;
    mStageDirtyEvent++;
}

void OmniPVDDebugVizData::updateGizmoVizModes()
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    const int nbrGizmos = OmniPVDDebugGizmoSelection::eNbrEnums;
    for (int i = 0; i < nbrGizmos; i++)
    {
        mGizmos[i].mRenderedVizMode = mGizmos[i].mVizMode;
    }
}

bool OmniPVDDebugVizData::getClosingState()
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    return mIsClosing;
}

void OmniPVDDebugVizData::setClosingState(bool closing)
{
    std::lock_guard<carb::tasking::MutexWrapper> lock(*mNextStateMutex);
    mIsClosing = closing;
}
