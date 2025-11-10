// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "NoticeHandler.h"

#include "DebugVisualization.h"
#include "DeformableBodyVisualizationManagerDeprecated.h"
#include "DeformableSurfaceVisualizationManagerDeprecated.h"
#include "ParticleAuthoring.h"
#include "ProxyVisualizationManager.h"
#include "ParticlesVisualizationManager.h"
#include "AttachmentsVisualizationManagerDeprecated.h"
#include "AttachmentAuthoringDeprecated.h"
#include "AttachmentAuthoring.h"
#include "SpatialTendonAuthoring.h"
#include "DebugVisualizationFixedTendon.h"

#include <carb/profiler/Profile.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/IPhysxAttachmentPrivate.h>

using namespace pxr;

extern omni::physx::ui::DebugVisualization* gDebugVisualization;
extern UsdStageRefPtr gStage;
bool gBlockNoticeHandle = false;
extern omni::physx::ui::ParticleAuthoring* gParticleAuthoring;
extern omni::physx::ui::ProxyVisualizationManager* gProxyVisualizationManager;
extern omni::physx::ui::ParticlesVisualizationManager* gParticlesVisualizationManager;
extern omni::physx::ui::AttachmentAuthoringDeprecated* gAttachmentAuthoringDeprecated;
extern omni::physx::ui::AttachmentAuthoring* gAttachmentAuthoring;
extern omni::physx::ui::AttachmentsVisualizationManagerDeprecated* gAttachmentsVisualizationManagerDeprecated;
extern omni::physx::ui::DeformableBodyVisualizationManagerDeprecated* gDeformableBodyVisualizationManagerDeprecated;
extern omni::physx::ui::DeformableSurfaceVisualizationManagerDeprecated* gDeformableSurfaceVisualizationManagerDeprecated;
extern omni::physx::ui::SpatialTendonManager* gSpatialTendonManager;
extern omni::physx::IPhysx* gPhysX;
extern omni::physx::IPhysxAttachmentPrivate* gPhysXAttachmentPrivate;
extern omni::physx::ui::FixedTendonVisualizer* gFixedTendonVisualization;


namespace omni
{
namespace physx
{
namespace ui
{


bool isXformAttr(const UsdPrim& prim, const TfToken& propertyName)
{
    if (!UsdGeomXformable::IsTransformationAffectedByAttrNamed(propertyName))
        return false;

    const UsdProperty prop = prim.GetProperty(propertyName);
    if (prop.Is<UsdAttribute>())
    {
        const UsdAttribute& attr = (const UsdAttribute&)(prop);
        return UsdGeomXformOp::IsXformOp(attr);
    }

    return false;
}

bool isXformScaleChgAttr(const UsdPrim& prim, const TfToken& propertyName)
{
    if (!UsdGeomXformable::IsTransformationAffectedByAttrNamed(propertyName))
        return false;

    const UsdProperty prop = prim.GetProperty(propertyName);
    if (prop.Is<UsdAttribute>())
    {
        const UsdAttribute& attr = (const UsdAttribute&)(prop);
        if (UsdGeomXformOp::IsXformOp(attr))
        {
            const UsdGeomXformOp xformOp(attr);
            UsdGeomXformOp::Type opType = xformOp.GetOpType();
            return (opType == UsdGeomXformOp::Type::TypeScale || opType == UsdGeomXformOp::Type::TypeTransform);
        }
    }

    return false;
}

// not the same as handleStandardManager, for various reasons
static void handleFixedTendonsVisualizer(const class pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    CARB_PROFILE_ZONE(0, "PhysXUIUsdNoticeListener::FixedTendonsVisualizer");

    // do not process anything if the visualizer is not active, i.e. being used
    if (!gFixedTendonVisualization || !gFixedTendonVisualization->isActive())
    {
        return;
    }

    FixedTendonVisualizer& tendonVisualization = *gFixedTendonVisualization;

    if (tendonVisualization.isDirty())
    {
        // we're already reparsing next update
        return;
    }

    if (!objectsChanged.GetResyncedPaths().empty())
    {
        tendonVisualization.setDirty();
        return;
    }

    if (tendonVisualization.isEmpty())
    {
        // no structural changes and we have no tendons
        return;
    }

    for (const pxr::SdfPath& infoPath : objectsChanged.GetChangedInfoOnlyPaths())
    {
        if (tendonVisualization.hasTendon(infoPath.GetPrimPath()))
        {
            tendonVisualization.setDirty();
            return;
        }
    }
}

template<typename StandardManager, bool hasAttributeResyncBug> // see OMPE-27632
static void handleStandardManager(StandardManager* manager, const char* zoneName, const pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    CARB_PROFILE_ZONE(0, zoneName);

    if (manager == nullptr || !manager->isActive())
        return;

    for (const SdfPath& path : objectsChanged.GetResyncedPaths())
    {
        const SdfPath primPath =
            gStage->GetPseudoRoot().GetPath() == path ? gStage->GetPseudoRoot().GetPath() : path.GetPrimPath();

        UsdPrim prim = gStage->GetPrimAtPath(primPath);
        if (prim.IsValid() == false || !prim.IsActive()) // remove prim
        {
            manager->handlePrimRemove(primPath);
        }
        else // prim was resynced:
        {
            if (hasAttributeResyncBug)
            {
                manager->handlePrimResync(primPath);
            }
            else
            {
                if (path.IsPropertyPath())
                {
                    const TfToken attrToken = path.GetNameToken();
                    manager->handleAttributeChange(primPath, attrToken, UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrToken));
                }
                else
                {
                    manager->handlePrimResync(primPath);
                }
            }
        }
    }

    // only handle attribute changes if viz manager is tracking relevant prims
    if (!manager->isEmpty())
    {
        for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
        {
            const SdfPath primPath = path.GetPrimPath();
            const TfToken attrToken = path.GetNameToken();
            const bool isXform = UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrToken);
            manager->handleAttributeChange(primPath, attrToken, isXform);
        }
    }
}

void UsdNoticeListener::handle(const pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    // This is an old callback, ignore it
    if (gBlockNoticeHandle || !gStage || gStage != objectsChanged.GetStage())
    {
        return;
    }

    TRACE_FUNCTION();

    // handle updating spatial tendons
    handleStandardManager<SpatialTendonManager, false>(gSpatialTendonManager,
        "PhysXUIUsdNoticeListener::SpatialTendonManager", objectsChanged);

    // handle updating deformable volume debug viz tetmeshes
    handleStandardManager<DeformableBodyVisualizationManagerDeprecated, true>(gDeformableBodyVisualizationManagerDeprecated,
        "PhysXUIUsdNoticeListener::DeformableBodyVisualizationManagerDeprecated", objectsChanged);

    // handle updating deformable surface debug viz triangle meshes
    handleStandardManager<DeformableSurfaceVisualizationManagerDeprecated, true>(gDeformableSurfaceVisualizationManagerDeprecated,
        "PhysXUIUsdNoticeListener::DeformableSurfaceVisualizationManagerDeprecated", objectsChanged);

    handleStandardManager<ParticleAuthoring, false>(
        gParticleAuthoring, "PhysXUIUsdNoticeListener::ParticleAuthoring", objectsChanged);

    handleStandardManager<ProxyVisualizationManager, false>(gProxyVisualizationManager,
        "PhysXUIUsdNoticeListener::ProxyVisualizationManager", objectsChanged);

    // handle updating attachment visualization
    handleStandardManager<AttachmentsVisualizationManagerDeprecated, true>(gAttachmentsVisualizationManagerDeprecated,
        "PhysXUIUsdNoticeListener::AttachmentsVisualizationManagerDeprecated", objectsChanged);

    if (gDebugVisualization && gDebugVisualization->isUsdBasedVisualizationEnabled())
    {
        CARB_PROFILE_ZONE(0, "PhysXUIUsdNoticleListener::DebugVisualization");

        for (const SdfPath& path : objectsChanged.GetResyncedPaths())
        {
            if (path.IsPrimPath())
            {
                gDebugVisualization->onResyncedPrimPath(path);
            }
            else if (path.IsPropertyPath())
            {
                const SdfPath primPath = path.GetParentPath();
                UsdPrim prim = gStage->GetPrimAtPath(primPath);
                const TfToken& attrToken = path.GetNameToken();
                const bool isXform = isXformAttr(prim, attrToken);
                const bool isScaleChg = isXformScaleChgAttr(prim, attrToken);
                gDebugVisualization->onAttributeChange(path, primPath, prim, true, isXform, isScaleChg);
            }
        }

        for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
        {
            const SdfPath primPath = gStage->GetPseudoRoot().GetPath() == path ? path : path.GetPrimPath();
            UsdPrim prim = gStage->GetPrimAtPath(primPath);

            if (!prim.IsValid())
                continue;

            // early exit camera
            if (prim.IsA<pxr::UsdGeomCamera>())
                continue;

            // early exit for joints
            UsdPrim parent = prim;
            bool isJoint = false;
            bool isCamera = false;
            while(!isJoint && !isCamera && parent != gStage->GetPseudoRoot())
            {
                if (parent.IsA<pxr::UsdGeomCamera>())
                    isCamera = true;
                if (parent.IsA<UsdPhysicsJoint>())
                    isJoint = true;
                parent = parent.GetParent();
            }
            if (isJoint || isCamera)
                continue;            

            const bool isAttributePath = path.IsPropertyPath();
            bool clearPrim = false;
            if (objectsChanged.HasChangedFields(primPath))
            {
                const pxr::TfTokenVector changedFieldTokens = objectsChanged.GetChangedFields(primPath);
                for (const TfToken& f : changedFieldTokens)
                {
                    if (!isXformAttr(prim, f))
                    {
                        clearPrim = true;
                    }
                }
            }

            if (isAttributePath)
            {
                TfToken const& attrName = path.GetNameToken();
                if (isXformAttr(prim, attrName))
                {
                    gDebugVisualization->onAttributeChange(
                        path, primPath, prim, false, true, isXformScaleChgAttr(prim, attrName));
                }
                else
                {
                    if (attrName != UsdPhysicsTokens.Get()->physicsAngularVelocity &&
                        attrName != UsdPhysicsTokens.Get()->physicsVelocity &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerSteer &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerSteerLeft &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerSteerRight &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerAccelerator &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerBrake0 &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerBrake1 &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerBrake &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleControllerHandbrake &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleWheelControllerSteerAngle &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleWheelControllerDriveTorque &&
                        attrName != PhysxSchemaTokens.Get()->physxVehicleWheelControllerBrakeTorque)
                    {
                        clearPrim = true;
                    }
                }
            }

            if (clearPrim)
            {
                gDebugVisualization->onAttributeChange(path, primPath, prim, false, false, false);
            }
        }
    }

    handleFixedTendonsVisualizer(objectsChanged);
    handleStandardManager<AttachmentAuthoringDeprecated, false>(gAttachmentAuthoringDeprecated,
        "PhysXUIUsdNoticeListener::AttachmentAuthoringDeprecated", objectsChanged);
    handleStandardManager<AttachmentAuthoring, false>(gAttachmentAuthoring,
        "PhysXUIUsdNoticeListener::AttachmentAuthoring", objectsChanged);
}

} // namespace ui
} // namespace physx
} // namespace omni
