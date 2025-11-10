// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/profiler/Profile.h>

#include "DebugVisualizationPhysX.h"
#include <common/ui/ImguiDrawingUtils.h>

#include <omni/kit/ViewportTypes.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>


using namespace carb::renderer;
using namespace pxr;
using namespace omni::renderer;
using namespace omni::physics::ui;

extern omni::kit::IViewportWindow* gViewportWindow;

static constexpr float kLineWidth = 1.0f;

namespace omni
{
namespace physx
{
namespace ui
{

void onPhysicsCompletionEventFn(SimulationStatusEvent eventStatus, void* userData)
{
    if (eventStatus == SimulationStatusEvent::eSimulationComplete)
    {
        DebugVisualizationPhysX* debugDraw = static_cast<DebugVisualizationPhysX*>(userData);

        if (debugDraw)
        {
            debugDraw->simulationComplete();
        }
    }
}

DebugVisualizationPhysX::DebugVisualizationPhysX()
    : mVisualizationDistance(1000.0f), mEnableDebugVisualization(false), mVisualizationUpdated(false)
{
    mViewMatrix = pxr::GfMatrix4d(1.0);
    mIDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();
    mIPhysxVisualization = carb::getCachedInterface<omni::physx::IPhysxVisualization>();
    mIPhysx = carb::getCachedInterface<omni::physx::IPhysx>();

    mPhysicsCompletionEventId = mIPhysx->subscribePhysicsSimulationEvents(onPhysicsCompletionEventFn, this);
}

DebugVisualizationPhysX::~DebugVisualizationPhysX()
{
    mIPhysx->unsubscribePhysicsSimulationEvents(mPhysicsCompletionEventId);
}

void DebugVisualizationPhysX::enableDebugVisualization(bool enable)
{
    mEnableDebugVisualization = enable;

    if (enable)
    {
        updateVisualizationCullBox();
    }
}

static carb::Double3 convertToDouble3(const pxr::GfVec3d& v) { return carb::Double3{ v[0], v[1], v[2] }; }

void DebugVisualizationPhysX::updateVisualizationCullBox()
{
    Double3 camPos;
    Double3 fw;
    Double3 right;
    Double3 up;

    if (gViewportWindow)
    {
        // Viewport 1
        const char* currentCamera = gViewportWindow->getActiveCamera();
        gViewportWindow->getCameraPosition(currentCamera, camPos);
        gViewportWindow->getCameraForward(currentCamera, fw);
        gViewportWindow->getCameraRight(currentCamera, right);
        gViewportWindow->getCameraUp(currentCamera, up);
    }
    else
    {
        // Viewport 2
        auto cameraMatrix = mViewMatrix.GetInverse();
        pxr::GfVec3d a1 = cameraMatrix.GetRow3(0);
        pxr::GfVec3d a2 = cameraMatrix.GetRow3(1);
        pxr::GfVec3d a3 = -cameraMatrix.GetRow3(2);
        pxr::GfVec3d trans = cameraMatrix.ExtractTranslation();
        right = convertToDouble3(a1);
        up = convertToDouble3(a2);
        fw = convertToDouble3(a3);
        camPos = convertToDouble3(trans);
    }


    Float3 corners[8];
    corners[0] = { float(camPos.x + (up.x + right.x) * mVisualizationDistance),
                   float(camPos.y + (up.y + right.y) * mVisualizationDistance),
                   float(camPos.z + (up.z + right.z) * mVisualizationDistance) };
    corners[1] = { float(camPos.x + (up.x - right.x) * mVisualizationDistance),
                   float(camPos.y + (up.y - right.y) * mVisualizationDistance),
                   float(camPos.z + (up.z - right.z) * mVisualizationDistance) };
    corners[2] = { float(camPos.x + (fw.x * 2 + up.x + right.x) * mVisualizationDistance),
                   float(camPos.y + (fw.x * 2 + up.y + right.y) * mVisualizationDistance),
                   float(camPos.z + (fw.x * 2 + up.z + right.z) * mVisualizationDistance) };
    corners[3] = { float(camPos.x + (fw.x * 2 + up.x - right.x) * mVisualizationDistance),
                   float(camPos.y + (fw.x * 2 + up.y - right.y) * mVisualizationDistance),
                   float(camPos.z + (fw.x * 2 + up.z - right.z) * mVisualizationDistance) };
    corners[4] = { float(camPos.x + (-up.x + right.x) * mVisualizationDistance),
                   float(camPos.y + (-up.y + right.y) * mVisualizationDistance),
                   float(camPos.z + (-up.z + right.z) * mVisualizationDistance) };
    corners[5] = { float(camPos.x + (-up.x - right.x) * mVisualizationDistance),
                   float(camPos.y + (-up.y - right.y) * mVisualizationDistance),
                   float(camPos.z + (-up.z - right.z) * mVisualizationDistance) };
    corners[6] = { float(camPos.x + (fw.x * 2 - up.x + right.x) * mVisualizationDistance),
                   float(camPos.y + (fw.x * 2 - up.y + right.y) * mVisualizationDistance),
                   float(camPos.z + (fw.x * 2 - up.z + right.z) * mVisualizationDistance) };
    corners[7] = { float(camPos.x + (fw.x * 2 - up.x - right.x) * mVisualizationDistance),
                   float(camPos.y + (fw.x * 2 - up.y - right.y) * mVisualizationDistance),
                   float(camPos.z + (fw.x * 2 - up.z - right.z) * mVisualizationDistance) };

    Float3 min = { FLT_MAX, FLT_MAX, FLT_MAX };
    Float3 max = { -FLT_MAX, -FLT_MAX, -FLT_MAX };

    for (int i = 0; i < 8; i++)
    {
        min.x = fminf(min.x, corners[i].x);
        min.y = fminf(min.y, corners[i].y);
        min.z = fminf(min.z, corners[i].z);

        max.x = fmaxf(max.x, corners[i].x);
        max.y = fmaxf(max.y, corners[i].y);
        max.z = fmaxf(max.z, corners[i].z);
    }

    mIPhysxVisualization->setVisualizationCullingBox(min, max);
}

void DebugVisualizationPhysX::drawTriangle(const DebugTriangle& triangle)
{
    mIDebugDraw->drawLine(triangle.mPos0, convertColor(triangle.mColor0), kLineWidth, triangle.mPos1, convertColor(triangle.mColor1), kLineWidth);
    mIDebugDraw->drawLine(triangle.mPos1, convertColor(triangle.mColor1), kLineWidth, triangle.mPos2, convertColor(triangle.mColor2), kLineWidth);
    mIDebugDraw->drawLine(triangle.mPos2, convertColor(triangle.mColor2), kLineWidth, triangle.mPos0, convertColor(triangle.mColor0), kLineWidth);
}

void DebugVisualizationPhysX::drawLine(const DebugLine& line)
{
    mIDebugDraw->drawLine(line.mPos0, convertColor(line.mColor0), kLineWidth, line.mPos1, convertColor(line.mColor1), kLineWidth);
}

void DebugVisualizationPhysX::drawPoint(const DebugPoint& point)
{
    mIDebugDraw->drawPoint(point.mPos, convertColor(point.mColor), kLineWidth);
}

void DebugVisualizationPhysX::draw()
{
    pxr::UsdStageWeakPtr stage = omni::usd::UsdContext::getContext()->getStage();
    if (!stage)
    {
        return;
    }

    if (mEnableDebugVisualization)
    {
        CARB_PROFILE_ZONE(0, "DebugVisualizationPhysX::draw");

        const uint32_t numTriangles = mIPhysxVisualization->getNbTriangles();
        if (numTriangles)
        {
            const DebugTriangle* triangles = mIPhysxVisualization->getTriangles();
            for (uint32_t i = 0; i < numTriangles; i++)
            {
                const DebugTriangle& triangle = triangles[i];
                drawTriangle(triangle);
            }
        }

        const uint32_t numLines = mIPhysxVisualization->getNbLines();
        if (numLines)
        {
            const DebugLine* lines = mIPhysxVisualization->getLines();
            for (uint32_t i = 0; i < numLines; i++)
            {
                const DebugLine& line = lines[i];
                drawLine(line);
            }
        }

        const uint32_t numPoints = mIPhysxVisualization->getNbPoints();
        if (numPoints)
        {
            const DebugPoint* points = mIPhysxVisualization->getPoints();
            for (uint32_t i = 0; i < numPoints; i++)
            {
                const DebugPoint& point = points[i];
                drawPoint(point);
            }
        }

        updateVisualizationCullBox();
    }
}

void DebugVisualizationPhysX::simulationComplete()
{
    draw();
    mVisualizationUpdated = true;
}
void DebugVisualizationPhysX::stageUpdate()
{
    if (!mVisualizationUpdated)
    {
        draw();
    }
    mVisualizationUpdated = false;
}


}
}
}
