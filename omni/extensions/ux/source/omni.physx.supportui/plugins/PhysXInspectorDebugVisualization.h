// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/IPhysxSupportUiPrivate.h>
#include <omni/renderer/IDebugDraw.h>
#include <PxPhysicsAPI.h>

namespace omni
{
namespace ui
{
namespace scene
{
class PhysXInspectorOverlayImpl;
}
} // namespace ui
} // namespace omni
class PhysXInspectorModelImpl;
class PhysXInspector;
class PhysXInspectorDebugVisualization
{
    PhysXInspector* mInspector = nullptr;
    omni::renderer::IDebugDraw* mDebugDraw = nullptr;
    omni::kit::StageUpdateNode* mStageUpdateNode = nullptr;
    std::vector<std::weak_ptr<omni::ui::scene::PhysXInspectorOverlayImpl>> mOverlays;
    bool mIsSomeBodyHovered = false;
    void drawViewportForModel(PhysXInspectorModelImpl* model);
    void drawInertiaCubeFor(::physx::PxRigidBody* body);
    void checkIfSomeBodyIsHovered();
    bool isBodyHovered(::physx::PxRigidBody* body) const;

public:
    void onStartup(PhysXInspector* inspector);
    void onShutdown();
    void registerOverlay(std::shared_ptr<omni::ui::scene::PhysXInspectorOverlayImpl> overlay);
    void updateDebugVisualization();
};
