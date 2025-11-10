// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on
#include <omni/ui/scene/SceneView.h>
#include <imgui.h>
#include <carb/Types.h>
#include "PhysXUIOmniUISceneOverlay.h"

void internalDrawOmniSceneUIOverlays(pxr::GfMatrix4d viewMatrix,
                                     pxr::GfMatrix4d projMatrix,
                                     carb::Float4 viewportRect,
                                     std::function<void(bool)>& enableViewport2Picking);

OMNIUI_SCENE_NAMESPACE_OPEN_SCOPE

void PhysXUIOmniUISceneOverlayImpl::_preDrawContent(
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
    if (!mEnableViewport2Picking)
    {
        mEnableViewport2Picking = [this](bool enable) {
            // We cache the value to avoid constantly travelling to python side
            if (mViewport2PickEnabled != enable)
            {
                mViewport2PickEnabled = enable;
                if (mEnablePickingFn)
                {
                    mEnablePickingFn(enable);
                }
            }
        };
    }
    // We want to be absolutely sure that we don't block picking if somebody forgets to restore initial state
    mEnableViewport2Picking(true);
    internalDrawOmniSceneUIOverlays(viewMatrix, projMatrix, viewportRect, mEnableViewport2Picking);
    Manipulator::_preDrawContent(input, projection, view, width, height);
}

OMNIUI_SCENE_NAMESPACE_CLOSE_SCOPE
