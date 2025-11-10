// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"


#include "PhysXUSDProperties.h"
#include "propertiesUpdate/PhysXPropertiesUpdate.h"

#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttributeHelpers.h"

using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace omni::physx::usdparser;

namespace omni
{
namespace physx
{

class CCTBehaviorCallback : public PxControllerBehaviorCallback
{
public:
    virtual PxControllerBehaviorFlags getBehaviorFlags(const PxShape& shape, const PxActor& actor)
    {
        return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT | PxControllerBehaviorFlag::eCCT_SLIDE;
    }

    virtual PxControllerBehaviorFlags getBehaviorFlags(const PxController& controller)
    {
        return PxControllerBehaviorFlag::eCCT_CAN_RIDE_ON_OBJECT | PxControllerBehaviorFlag::eCCT_SLIDE;
    }

    virtual PxControllerBehaviorFlags getBehaviorFlags(const PxObstacle& obstacle)
    {
        return PxControllerBehaviorFlags(0);
    }

    virtual ~CCTBehaviorCallback()
    {
    }
} cctBehaviorCallback;

////////////////////////////////////////////////////////////////////////////////////////////////////////
PxCapsuleControllerDesc parsePhysXCharacterControllerDesc(const pxr::UsdStageRefPtr stage, const pxr::UsdPrim& usdPrim, float radius, float height)
{
    double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stage);
    float tolerancesLength = float(1.0f / metersPerUnit);

    PxCapsuleControllerDesc cctDesc;
    cctDesc.setToDefault();
    cctDesc.contactOffset *= tolerancesLength;
    cctDesc.stepOffset = (height+radius*2.0f) * 0.2f;
	cctDesc.radius = radius;
	cctDesc.height = height;
    cctDesc.behaviorCallback = &cctBehaviorCallback;

    TfToken usdUpAxis = pxr::UsdGeomGetStageUpAxis(stage);
    if (usdUpAxis == PhysxSchemaTokens.Get()->X)
        cctDesc.upDirection = PxVec3(1.0f, 0.0f, 0.0f);
    else if (usdUpAxis == PhysxSchemaTokens.Get()->Y)
        cctDesc.upDirection = PxVec3(0.0f, 1.0f, 0.0f);
    else if (usdUpAxis == PhysxSchemaTokens.Get()->Z)
        cctDesc.upDirection = PxVec3(0.0f, 0.0f, 1.0f);

    const PhysxSchemaPhysxCharacterControllerAPI physxCctAPI =
        PhysxSchemaPhysxCharacterControllerAPI::Get(stage, usdPrim.GetPrimPath());
    if (physxCctAPI)
    {
        getAttribute(cctDesc.invisibleWallHeight, physxCctAPI.GetInvisibleWallHeightAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(cctDesc.maxJumpHeight, physxCctAPI.GetMaxJumpHeightAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(cctDesc.contactOffset, physxCctAPI.GetContactOffsetAttr(), -FLT_MAX, FLT_MAX, updateCctContactOffset);
        getAttribute(cctDesc.stepOffset, physxCctAPI.GetStepOffsetAttr(), 0.0f, height+radius*2.0f, updateCctStepOffset);
        getAttribute(cctDesc.scaleCoeff, physxCctAPI.GetScaleCoeffAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(cctDesc.volumeGrowth, physxCctAPI.GetVolumeGrowthAttr(), -FLT_MAX, FLT_MAX, nullptr);

        if (physxCctAPI.GetUpAxisAttr())
        {
            TfToken upAxis;
            physxCctAPI.GetUpAxisAttr().Get(&upAxis);
            if (upAxis == PhysxSchemaTokens.Get()->X)
                cctDesc.upDirection = PxVec3(1.0f, 0.0f, 0.0f);
            else if (upAxis == PhysxSchemaTokens.Get()->Y)
                cctDesc.upDirection = PxVec3(0.0f, 1.0f, 0.0f);
            else if (upAxis == PhysxSchemaTokens.Get()->Z)
                cctDesc.upDirection = PxVec3(0.0f, 0.0f, 1.0f);
        }

        if (physxCctAPI.GetNonWalkableModeAttr())
        {
            TfToken nonWM;
            physxCctAPI.GetNonWalkableModeAttr().Get(&nonWM);
            if (nonWM == PhysxSchemaTokens.Get()->preventClimbing)
                cctDesc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING;
            else if (nonWM == PhysxSchemaTokens.Get()->preventClimbingForceSliding)
                cctDesc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
        }

        if (physxCctAPI.GetClimbingModeAttr())
        {
            TfToken climbMode;
            physxCctAPI.GetClimbingModeAttr().Get(&climbMode);
            if (climbMode == PhysxSchemaTokens.Get()->easy)
                cctDesc.climbingMode = PxCapsuleClimbingMode::eEASY;
            else if (climbMode == PhysxSchemaTokens.Get()->constrained)
                cctDesc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
        }
    }

    return cctDesc;
}

} // namespace physx
} // namespace omni
