// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/IPhysxSupportUiPrivate.h>
#include <PxPhysicsAPI.h>

class PhysXInspector;
class PhysXInspectorModelImpl;
OMNIUI_SCENE_NAMESPACE_OPEN_SCOPE

class PhysXInspectorOverlayImpl : public PhysXInspectorOverlay
{
    OMNIUI_SCENE_OBJECT(PhysXInspectorOverlayImpl)
public:
    PhysXInspectorOverlayImpl(PhysXInspector* inspector);
    bool isAnyBodyHovered() const;
    bool isBodyHovered(::physx::PxRigidBody* body) const;

protected:
    virtual void _preDrawContent(
        const MouseInput& input, const Matrix44& projection, const Matrix44& view, float width, float height) override;

private:
    void drawViewportOverlays(const pxr::GfMatrix4d modelViewProjection, carb::Float4 viewportRect);
    void drawViewportForModel(PhysXInspectorModelImpl* model);
    void drawRigidMassLabel(::physx::PxRigidBody* rigid);
    void drawViewportOverlayMasses();

    PhysXInspector* mInspector;

    pxr::GfMatrix4d mModelViewProjection;
    carb::Float4 mViewportRect;
    std::vector<::physx::PxRigidBody*> mRigidBodiesList;
    std::vector<::physx::PxRigidBody*> mHoveredBodies;
};

OMNIUI_SCENE_NAMESPACE_CLOSE_SCOPE
