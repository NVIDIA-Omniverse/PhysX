// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>
#include <carb/settings/ISettings.h>

#include <omni/renderer/IDebugDraw.h>

namespace omni
{
namespace physics
{
namespace ui
{

struct JointData
{
    JointData()
        : scale(0.0),
          assetId(0),
          billboardUid(omni::renderer::kInvalidBillboardId),
          billboardId(omni::renderer::kInvalidBillboardId),
          gizmoScale(1.0f)
    {
    }

    PXR_NS::GfVec3d scale;
    uint32_t assetId;
    uint32_t billboardUid;
    uint32_t billboardId;
    float gizmoScale;
    PXR_NS::GfMatrix4f currentGizmoMatrix;
};

using JointDataMap = PXR_NS::TfHashMap<PXR_NS::SdfPath, JointData, PXR_NS::SdfPath::Hash>;

class JointBillboardScaleTask
{
public:
    JointBillboardScaleTask();

    virtual ~JointBillboardScaleTask(void);

    // Start performing the task
    virtual void performTask(void);

    void initialize(const PXR_NS::GfVec3f& camPos,
                    const JointDataMap& jointDataMap,
                    const PXR_NS::GfMatrix4d& viewProjection,
                    const PXR_NS::GfMatrix4d& viewInverse,
                    const carb::Float4& viewPortRect,
                    carb::settings::ISettings* settings,
                    PXR_NS::UsdStageWeakPtr stage);

    bool initialized() const
    {
        return mInitialized;
    }
    void execute(bool async);
    void setInitialized(bool val)
    {
        mInitialized = val;
    }

    const PXR_NS::GfVec3f& getCurrentCameraPos() const
    {
        return mCameraPos;
    }

    bool finished() const
    {
        return mFinished;
    }

    void clear()
    {
        mJointData.clear();
        mInstanceMatrices.clear();
    }

    const std::vector<PXR_NS::GfMatrix4f>& getInstanceMatrices() const
    {
        return mInstanceMatrices;
    }

private:
    std::atomic<bool> mInitialized{ false }; // true if the task has been initialized, but was not scheduled yet
    std::atomic<bool> mFinished{ true }; // Set to true when the task is completed
    carb::tasking::ITasking* mTasking{ nullptr }; // Pointer to the tasking interface

    PXR_NS::GfVec3f mCameraPos;
    float mMinFadeOut;
    float mMaxFadeOut;
    float mMetersPerUnit;

    PXR_NS::GfMatrix4d mViewProjection;
    PXR_NS::GfMatrix4d mViewInverse;
    carb::Float4 mViewPortRect;

    std::vector<JointData> mJointData;
    std::vector<PXR_NS::GfMatrix4f> mInstanceMatrices;
};

} // namespace ui
} // namespace physics
} // namespace omni
