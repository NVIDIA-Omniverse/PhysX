// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on
#include <omni/ui/scene/SceneView.h>
#include "PhysXInspectorOverlay.h"
#include "PhysXInspector.h"
#include "PhysXInspectorModel.h"
#include <imgui.h>
#include <carb/Types.h>

using namespace physx;
OMNIUI_SCENE_NAMESPACE_OPEN_SCOPE

PhysXInspectorOverlayImpl::PhysXInspectorOverlayImpl(PhysXInspector* inspector)
{
    mInspector = inspector;
}

void PhysXInspectorOverlayImpl::_preDrawContent(
    const MouseInput& input, const Matrix44& projection, const Matrix44& view, float width, float height)
{
    ImVec2 windowOrigin = ImGui::GetCursorScreenPos();
    float dpiScale = getSceneView()->getDpiScale();
    ImVec2 windowSize(width * dpiScale, height * dpiScale);
    carb::Float4 viewportRect{ windowOrigin.x, windowOrigin.y, windowOrigin.x + windowSize.x,
                        windowOrigin.y + windowSize.y };

    pxr::GfMatrix4d viewMatrix;
    pxr::GfMatrix4d projMatrix;

    for (int row = 0; row < 4; ++row)
    {
        for (int col = 0; col < 4; ++col)
        {
            viewMatrix[row][col] = (double)view[row][col];
            projMatrix[row][col] = (double)projection[row][col];
        }
    }
    drawViewportOverlays(viewMatrix * projMatrix, viewportRect);
}


inline ImVec2 projectWorldToScreen(const pxr::GfVec4d& worldPos,
                                         const pxr::GfMatrix4d& mat,
                                         const carb::Float4& viewPortRect,
                                         pxr::GfVec4d& viewProjectionSpace)
{
    viewProjectionSpace = worldPos * mat;
    pxr::GfVec4d clipSpace = viewProjectionSpace;
    clipSpace *= 0.5f / clipSpace[3];
    clipSpace += pxr::GfVec4d(double(0.5), double(0.5), double(0.0), double(0.0));
    clipSpace[1] = 1.f - clipSpace[1];
    clipSpace[0] *= viewPortRect.z - viewPortRect.x;
    clipSpace[1] *= viewPortRect.w - viewPortRect.y;
    clipSpace[0] += viewPortRect.x;
    clipSpace[1] += viewPortRect.y;
    return ImVec2(float(clipSpace[0]), float(clipSpace[1]));
}

static inline void drawPolyArcFilled(const float startAngle,
                                     const float arcAngle,
                                     const float radius,
                                     ImVec2 zeroPos,
                                     int numSteps,
                                     uint32_t col,
                                     bool filled = true,
                                     float thickness = 5)
{
    const int maxNumSteps = 50;
    if (numSteps + 1 > maxNumSteps)
        return;
    const float angleStep = static_cast<float>(arcAngle / (numSteps - 1));
    ImVec2 points[maxNumSteps];
    points[0] = zeroPos;
    for (int i = 0; i < numSteps; ++i)
    {
        points[i + 1] = ImVec2(zeroPos.x + cosf(startAngle + angleStep * i) * radius,
                                zeroPos.y + sinf(startAngle + angleStep * i) * radius);
    }
    if (filled)
    {
        ImGui::GetWindowDrawList()->AddConvexPolyFilled(points, numSteps + 1, col);
    }
    else
    {
        ImGui::GetWindowDrawList()->AddPolyline(points, numSteps + 1, col, false, thickness);
    }
}


static inline void drawPolyArcOutline(const float startAngle,
                                      const float arcAngle,
                                      const float radius,
                                      ImVec2 zeroPos,
                                      int numSteps,
                                      uint32_t col,
                                      float thickness)
{
    const int maxNumSteps = 50;
    if (numSteps + 1 > maxNumSteps)
        return;
    const float angleStep = static_cast<float>(arcAngle / (numSteps - 1));
    ImVec2 points[maxNumSteps];
    points[0] = zeroPos;
    for (int i = 0; i < numSteps; ++i)
    {
        points[i + 1] = ImVec2(zeroPos.x + cosf(startAngle + angleStep * i) * radius,
                                zeroPos.y + sinf(startAngle + angleStep * i) * radius);
    }
    ImGui::GetWindowDrawList()->AddPolyline(points + 1, numSteps, col, false, thickness);
}

static inline void drawCircularBillboard(const float radius, ImVec2 zeroPos, uint32_t col)
{
    drawPolyArcFilled(0, static_cast<float>(Py_MATH_PI / 2.0f), radius, zeroPos, 5, col);
    drawPolyArcFilled(
        static_cast<float>(Py_MATH_PI), static_cast<float>(Py_MATH_PI / 2.0f), radius, zeroPos, 5, col);
    ImGui::GetWindowDrawList()->AddCircle(zeroPos, radius, col, 20, 2.0f);
}

static inline void drawWeightBillboard(const float radius, ImVec2 zeroPos, uint32_t col)
{
    ImVec2 vertices[6] = {
        ImVec2(-0.3f, -1.0f), ImVec2(-0.5f, -1.0f), ImVec2(-1.0f, +0.4f),
        ImVec2(+1.0f, +0.4f), ImVec2(+0.5f, -1.0f), ImVec2(+0.3f, -1.0f),
    };
    for (size_t i = 0; i < (sizeof(vertices) / sizeof(ImVec2)); ++i)
    {
        vertices[i].x *= radius;
        vertices[i].y *= radius;
        vertices[i].x += zeroPos.x;
        vertices[i].y += zeroPos.y;
    }
    ImGui::GetWindowDrawList()->AddConvexPolyFilled(vertices, 6, col);
    ImVec2 handleCenter(zeroPos.x, zeroPos.y - radius);
    drawPolyArcOutline(0, static_cast<float>(-Py_MATH_PI), 0.35f * radius, handleCenter, 8, col, 6.0f);
}


void PhysXInspectorOverlayImpl::drawViewportOverlays(const pxr::GfMatrix4d modelViewProjection, carb::Float4 viewportRect)
{
    using namespace physx;
    mModelViewProjection = modelViewProjection;
    mViewportRect = viewportRect;

    for (auto it : mInspector->mInspectorModels)
    {
        auto ptr = it.lock();
        if (ptr)
        {
            if (ptr->getShowMassesAndInertiaModel()->getValueAsBool())
            {
                drawViewportForModel(ptr.get());
            }
        }
    }
}

void PhysXInspectorOverlayImpl::drawViewportForModel(PhysXInspectorModelImpl* model)
{
    if (model->mSelectionIsArticulation)
    {
        PxArticulationReducedCoordinate* articulation = mInspector->getArticulationAt(model->mSelectedPrimPath);
        if (articulation && articulation->getNbLinks() > 0)
        {
            PxU32 numLinks = articulation->getNbLinks();
            mRigidBodiesList.resize(numLinks);
            for (PxU32 idx = 0; idx < numLinks; ++idx)
            {
                PxArticulationLink* link;
                articulation->getLinks(&link, 1, idx);
                mRigidBodiesList[idx] = link;
            }
            drawViewportOverlayMasses();
        }
    }
    else
    {
        PxScene* scene = mInspector->getSceneAt(mInspector->mSelectedPhysicsScenePath.GetText());
        if (scene)
        {
            PxU32 nbActors = scene->getNbActors(PxActorTypeFlag::eRIGID_DYNAMIC);
            mRigidBodiesList.clear();
            mRigidBodiesList.reserve(nbActors);
            for (PxU32 idx = 0; idx < nbActors; idx++)
            {
                PxActor* pxActor = nullptr;
                scene->getActors(PxActorTypeFlag::eRIGID_DYNAMIC, &pxActor, 1, idx);
                if (!model->isIncludedInCurrentSelection(pxActor->userData))
                    continue;

                mRigidBodiesList.push_back((PxRigidBody*)pxActor);
            }
            drawViewportOverlayMasses();
            const PxU32 nbArticulations = scene->getNbArticulations();
            PxArticulationReducedCoordinate* articulation = nullptr;
            for (PxU32 i = 0; i < nbArticulations; i++)
            {
                scene->getArticulations(&articulation, 1, i);
                if (!model->isIncludedInCurrentSelection(articulation->userData))
                    continue;
                PxU32 numLinks = articulation->getNbLinks();
                mRigidBodiesList.resize(numLinks);
                for (PxU32 idx = 0; idx < numLinks; ++idx)
                {
                    PxArticulationLink* link;
                    articulation->getLinks(&link, 1, idx);
                    mRigidBodiesList[idx] = link;
                }
                drawViewportOverlayMasses();
            }
        }
    }
}
bool PhysXInspectorOverlayImpl::isAnyBodyHovered() const
{
    return mHoveredBodies.size() > 0;
}

bool PhysXInspectorOverlayImpl::isBodyHovered(::physx::PxRigidBody* body) const
{
    return std::find(mHoveredBodies.begin(), mHoveredBodies.end(), body) != mHoveredBodies.end();
}

void PhysXInspectorOverlayImpl::drawViewportOverlayMasses()
{
    ImVec2 mousePos = ImGui::GetMousePos();
    const bool isMouseDown = ImGui::IsMouseDown(0) || ImGui::IsMouseDown(1) || ImGui::IsMouseDown(2);

    ::physx::PxU32 numLinks = (::physx::PxU32)mRigidBodiesList.size();

    float minMass = FLT_MAX;
    float maxMass = -FLT_MAX;
    for (::physx::PxU32 idx = 0; idx < numLinks; ++idx)
    {
        ::physx::PxRigidBody* link = mRigidBodiesList[idx];
        const float mass = link->getMass();
        if (mass > maxMass)
            maxMass = mass;
        if (mass < minMass)
            minMass = mass;
    }
    if (fabsf(maxMass - minMass) < FLT_EPSILON)
        maxMass = minMass + 1;
    float overlayRadius = 0.0f;
    ImVec2 overlayProjectedCOM;
    int totalNumberOfHoveredBillboards = 0;
    int totalNumberOfOverlays = 0;
    const int maximumNumberOfOverlays = 3;
    mHoveredBodies.clear();
    for (::physx::PxU32 idx = 0; idx < numLinks; ++idx)
    {
        ::physx::PxRigidBody* link = mRigidBodiesList[idx];
        const float mass = link->getMass();
        ::physx::PxTransform globalCenterOfMass = link->getGlobalPose() * link->getCMassLocalPose();
        ::physx::PxVec3 inertiaTensor = link->getMassSpaceInertiaTensor();
        pxr::GfVec4d viewProjectionPoint;
        ImVec2 projectedCenterOfMass = projectWorldToScreen(
            pxr::GfVec4d{ globalCenterOfMass.p.x, globalCenterOfMass.p.y, globalCenterOfMass.p.z, 1.0 },
            mModelViewProjection, mViewportRect, viewProjectionPoint);
        if (viewProjectionPoint[2] < -1.0f)
            continue; // clipped behind view frustrum
        const float minRadius = 15;
        const float maxRadius = 30;
        float radius = minRadius + (maxRadius - minRadius) * ((mass - minMass) / (maxMass - minMass));
        if (radius < minRadius)
            radius = minRadius;
        else if (radius > maxRadius)
            radius = maxRadius;
        const float distanceSq = (mousePos.x - projectedCenterOfMass.x) * (mousePos.x - projectedCenterOfMass.x) +
                                 (mousePos.y - projectedCenterOfMass.y) * (mousePos.y - projectedCenterOfMass.y);
        if (distanceSq < radius * radius && !isMouseDown)
        {
            // Hovering with mouse on this item, we will draw it later on top of all the others
            if (totalNumberOfOverlays < maximumNumberOfOverlays)
            {
                overlayRadius = radius;
                overlayProjectedCOM = projectedCenterOfMass;
                mHoveredBodies.push_back(link);
                totalNumberOfOverlays++;
            }
            totalNumberOfHoveredBillboards++;
        }
        else
        {
            drawWeightBillboard(radius, projectedCenterOfMass, 0x990000ff);
            drawCircularBillboard(radius / 3.0f, projectedCenterOfMass, 0xffff0000);
        }
    }
    if (totalNumberOfHoveredBillboards > 0)
    {
        drawWeightBillboard(overlayRadius, overlayProjectedCOM, 0xff00ff00);
        drawCircularBillboard(overlayRadius / 3.0f, overlayProjectedCOM, 0xffff0000);
        ImVec4 originalColor = ImGui::GetStyleColorVec4(ImGuiCol_PopupBg);
        originalColor.w *= 0.9f; // make tooltip window a little transparent
        ImGui::PushStyleColor(ImGuiCol_PopupBg, originalColor);
        ImGui::PushStyleColor(
            ImGuiCol_Separator, ImGui::GetStyleColorVec4(ImGuiCol_Text));
        ImGui::BeginTooltip();
        ImGui::InvisibleButton("test", ImVec2(600, 0)); // just make sure tooltip window is at least 600px
        if (totalNumberOfOverlays > 1)
        {
            ImGui::Text("(info for %d of %d links)", totalNumberOfOverlays, totalNumberOfHoveredBillboards);
            ImGui::Dummy(ImVec2(0.0f, 0.0f));
            ImGui::Separator();
            ImGui::Dummy(ImVec2(0.0f, 0.0f));
        }
        for (int i = 0; i < totalNumberOfOverlays; ++i)
        {
            if (i > 0)
            {
                ImGui::Dummy(ImVec2(0.0f, 0.0f));
                ImGui::Separator();
                ImGui::Dummy(ImVec2(0.0f, 0.0f));
            }
            ::physx::PxRigidBody* overlayLink = mHoveredBodies[i];
            ImGui::Text("Link: %s", overlayLink->getName());
            ImGui::Text("Mass: %f", overlayLink->getMass());
            ImGui::Columns(4, "columns", false);
            ImGui::SetColumnWidth(0, 150);
            ImGui::SetColumnWidth(1, 150);
            ImGui::SetColumnWidth(2, 150);
            ImGui::SetColumnWidth(3, 150);
            ImGui::NextColumn();
            ImGui::Text("x");
            ImGui::NextColumn();
            ImGui::Text("y");
            ImGui::NextColumn();
            ImGui::Text("z");
            ImGui::NextColumn();

            ImGui::Text("Inertia");
            ImGui::NextColumn();
            ImGui::Text("%f", overlayLink->getMassSpaceInertiaTensor().x);
            ImGui::NextColumn();
            ImGui::Text("%f", overlayLink->getMassSpaceInertiaTensor().y);
            ImGui::NextColumn();
            ImGui::Text("%f", overlayLink->getMassSpaceInertiaTensor().z);
            ImGui::NextColumn();

            ImGui::Text("Center Of Mass");
            ImGui::NextColumn();
            ImGui::Text("%f", overlayLink->getCMassLocalPose().p.x);
            ImGui::NextColumn();
            ImGui::Text("%f", overlayLink->getCMassLocalPose().p.y);
            ImGui::NextColumn();
            ImGui::Text("%f", overlayLink->getCMassLocalPose().p.z);
            ImGui::NextColumn();

            ImGui::Columns(1, "columns", false);
        }

        ImGui::EndTooltip();
        ImGui::PopStyleColor();
        ImGui::PopStyleColor();
    }
}


OMNIUI_SCENE_NAMESPACE_CLOSE_SCOPE
