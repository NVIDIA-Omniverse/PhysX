// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Framework.h>
#include <carb/profiler/Profile.h>
#include <carb/settings/ISettings.h>
#include <omni/kit/SettingsUtils.h>

#include "JointBillboardScaleTask.h"

static constexpr char kViewportGizmoMinFadeOutPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/minFadeOut";
static constexpr char kViewportGizmoMaxFadeOutPath[] = PERSISTENT_SETTINGS_PREFIX "/app/viewport/gizmo/maxFadeOut";

using namespace carb;
using namespace pxr;

namespace omni
{
namespace physics
{
namespace ui
{

void performTask(carb::tasking::ITasking* t, void* taskArg);

// This is the C style function callback which is invoked by the carb tasking
// system.
void performTask(carb::tasking::ITasking* t, void* taskArg)
{
    JointBillboardScaleTask* ct = reinterpret_cast<JointBillboardScaleTask*>(taskArg);
    if (ct)
    {
        ct->performTask();
    }
}


JointBillboardScaleTask::JointBillboardScaleTask()
{
    mTasking = carb::getCachedInterface<carb::tasking::ITasking>();
}

JointBillboardScaleTask::~JointBillboardScaleTask()
{
    // Wait until thread is finished
    while (!mFinished)
    {
        std::this_thread::sleep_for(std::chrono::microseconds(10));
    }

    mTasking = nullptr;
}

void JointBillboardScaleTask::initialize(const pxr::GfVec3f& camPos, const JointDataMap& jointDataMap,
    const pxr::GfMatrix4d& viewProjection, const pxr::GfMatrix4d& viewInverse, const carb::Float4& viewPortRect,
    carb::settings::ISettings* settings, pxr::UsdStageWeakPtr stage)
{
    if (!mTasking)
        return;

    mInitialized = true;

    mCameraPos = camPos;

    mMinFadeOut = settings->getAsFloat(kViewportGizmoMinFadeOutPath);
    mMaxFadeOut = settings->getAsFloat(kViewportGizmoMaxFadeOutPath);
    mMetersPerUnit = float(UsdGeomGetStageMetersPerUnit(stage));

    mViewInverse = viewInverse;
    mViewProjection = viewProjection;
    mViewPortRect = viewPortRect;

    mJointData.clear();
    mJointData.reserve(jointDataMap.size());
    for (JointDataMap::const_reference jointDataRef : jointDataMap)
    {
        mJointData.push_back(jointDataRef.second);
    }    
}

void JointBillboardScaleTask::execute(bool async)
{
    if (async)
    {
        mTasking->addTask(carb::tasking::Priority::eMedium, {}, [this] { performTask(); });
    }
    else
    {
        performTask();
        setInitialized(false);
    }
}


void JointBillboardScaleTask::performTask(void)
{    
    mFinished = false;

    CARB_PROFILE_ZONE(0, "JointAuthoringManager::UpdateGizmoFadeOutScaleTask");

    mInstanceMatrices.resize(mJointData.size());

    for (size_t i = 0; i < mJointData.size(); i++)
    {
        JointData& jointData = mJointData[i];
        const GfVec3f currentPos = jointData.currentGizmoMatrix.ExtractTranslation();
        const float distance = (currentPos - mCameraPos).GetLength() * mMetersPerUnit;

        const GfMatrix4d source = GfMatrix4d(jointData.currentGizmoMatrix.RemoveScaleShear());
        const GfMatrix4d modelInverse = source.GetInverse();
        const GfMatrix4d mvp = source * mViewProjection;

        // compute scale from the size of camera right vector projected on screen at the matrix position        
        GfVec4d pointRight = mViewInverse.GetRow(0);
        pointRight = pointRight * modelInverse;
        pointRight[3] = 1.0f;

        GfVec4d startOfSegment = GfVec4d(0.0, 0.0, 0.0, 1.0) * mvp;
        if (fabsf(float(startOfSegment[3])) > FLT_EPSILON) // check for axis aligned with camera direction
            startOfSegment *= 1.f / startOfSegment[3];

        GfVec4d endOfSegment = pointRight * mvp;
        if (fabsf(float(endOfSegment[3])) > FLT_EPSILON) // check for axis aligned with camera direction
            endOfSegment *= 1.f / endOfSegment[3];

        GfVec4d clipSpaceAxis = endOfSegment - startOfSegment;
        clipSpaceAxis[1] /= (mViewPortRect.z - mViewPortRect.x) / (mViewPortRect.w - mViewPortRect.y);
        const float rightLength = float(clipSpaceAxis.GetLength());
        float screenFactor = 0.005f / rightLength;

        if (distance > mMinFadeOut)
        {
            if (distance > mMaxFadeOut)
            {
                jointData.gizmoScale = 0.0f;
            }
            else
            {
                jointData.gizmoScale = (1.0f - (distance - mMinFadeOut) / (mMaxFadeOut - mMinFadeOut)) * screenFactor;
            }
        }
        else
        {
            jointData.gizmoScale = 1.0f * screenFactor;
        }

        GfMatrix4f scaleMatrix;
        scaleMatrix.SetScale(jointData.gizmoScale);
        mInstanceMatrices[i] = scaleMatrix * jointData.currentGizmoMatrix;
    }


    mFinished = true;    
}

}
}
}
