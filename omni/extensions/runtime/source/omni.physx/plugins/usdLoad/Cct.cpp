// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>

#include "LoadTools.h"
#include "LoadUsd.h"

using namespace pxr;
using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{    

CctDesc* getCctShape(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim)
{
    if (usdPrim.IsA<UsdGeomCapsule>())
    {
        float radius = 1.0f;
        float halfHeight = 1.0f;

        Float3 scale;
        Float3 pos;

        // Get xform transform
        pxr::UsdGeomXformable xform(usdPrim);

        {
            const pxr::GfTransform tr(xform.ComputeLocalToWorldTransform(UsdTimeCode()));
            const pxr::GfVec3d sc = tr.GetScale();
            const pxr::GfVec3d trans = tr.GetTranslation();
            halfHeight = float(sc[2]);
            radius = float(sc[1]);
            scale = { float(sc[0]), float(sc[1]), float(sc[2]) };
            pos = { float(trans[0]), float(trans[1]), float(trans[2]) };
        }

        // Get shape parameters
        {
            UsdGeomCapsule shape(usdPrim);
            double radiusAttr;
            shape.GetRadiusAttr().Get(&radiusAttr);
            double heightAttr;
            shape.GetHeightAttr().Get(&heightAttr);
            radius *= (float)radiusAttr;
            halfHeight *= (float)heightAttr * 0.5f;
        }

        CapsuleCctDesc* desc = ICE_PLACEMENT_NEW(CapsuleCctDesc)(radius, halfHeight);
        desc->scale = scale;
        desc->pos = pos;
        return desc;
    }

    CARB_LOG_ERROR("UsdToPhysics: CCT prim must be a capsule geom %s\n", usdPrim.GetTypeName().GetText());

    return nullptr;
}

CctDesc* parseCct(AttachedStage& attachedStage, const pxr::UsdPrim& usdPrim)
{
    PhysxSchemaPhysxCharacterControllerAPI usdCct = (const PhysxSchemaPhysxCharacterControllerAPI&)usdPrim;

    CctDesc* cctDesc = getCctShape(attachedStage.getStage(), usdPrim);

    if (!cctDesc)
        return nullptr;

    UsdAttribute slopeLimitAttr = usdCct.GetSlopeLimitAttr();
    if (slopeLimitAttr)
    {
        slopeLimitAttr.Get(&cctDesc->slopeLimit);
    }
    else
        cctDesc->slopeLimit = 0.0f;

    if (UsdRelationship rel = usdCct.GetSimulationOwnerRel())
    {
        SdfPathVector paths;
        rel.GetTargets(&paths);

        if (paths.size() > 1)
        {
            CARB_LOG_ERROR("parseCct: Relationship \"%s\" can have at most 1 entry.\n", rel.GetPath().GetText());
        }

        if (!paths.empty())
        {
            const SdfPath& path = paths.front();
            const ObjectId id = attachedStage.getObjectDatabase()->findEntry(path, eScene);

			if (id != kInvalidObjectId)
			{
				cctDesc->sceneId = id;
			}
			else
			{
				CARB_LOG_ERROR("parseCct: Failed to find physics simulation owner \"%s\".\n", path.GetText());
			}
        }
    }

    return cctDesc;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
