// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <omni/renderer/IDebugDraw.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxVisualization.h>

namespace omni
{
namespace physx
{
namespace ui
{

class DebugVisualizationPhysX
{
public:
    DebugVisualizationPhysX();

    ~DebugVisualizationPhysX();

    void setVisualizationDistance(float distance)
    {
        mVisualizationDistance = distance;
    }

    void enableDebugVisualization(bool enable);

    void simulationComplete();
    void stageUpdate();
    void setCameraMatrix(pxr::GfMatrix4d viewMatrix)
    {
        mViewMatrix = viewMatrix;
    }

private:
    void draw();
    void drawTriangle(const DebugTriangle& triangle);
    void drawLine(const DebugLine& line);
    void drawPoint(const DebugPoint& point);

    void updateVisualizationCullBox();

private:
    float mVisualizationDistance;
    bool mEnableDebugVisualization;
    bool mVisualizationUpdated;
    omni::renderer::IDebugDraw* mIDebugDraw;
    omni::physx::IPhysxVisualization* mIPhysxVisualization;
    omni::physx::IPhysx* mIPhysx;
    SubscriptionId mPhysicsCompletionEventId;
    pxr::GfMatrix4d mViewMatrix;
};

} // namespace ui
} // namespace physx
} // namespace omni
