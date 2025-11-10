// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/IPhysxUIPrivate.h>

OMNIUI_SCENE_NAMESPACE_OPEN_SCOPE

class PhysXUIOmniUISceneOverlayImpl : public PhysXUIOmniUISceneOverlay
{
    OMNIUI_SCENE_OBJECT(PhysXUIOmniUISceneOverlayImpl)
public:
protected:
    virtual void _preDrawContent(
        const MouseInput& input, const Matrix44& projection, const Matrix44& view, float width, float height) override;
    std::function<void(bool)> mEnableViewport2Picking;
    bool mViewport2PickEnabled = false;
};

OMNIUI_SCENE_NAMESPACE_CLOSE_SCOPE
