// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include "UsdPCH.h"


#include "PhysXUSDProperties.h"
#include "propertiesUpdate/PhysXPropertiesUpdate.h"

#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttributeHelpers.h"

using namespace PXR_NS;
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
PxCapsuleControllerDesc parsePhysXCharacterControllerDesc(usdparser::AttachedStage& attachedStage, const PXR_NS::UsdStageRefPtr stage, const PXR_NS::UsdPrim& usdPrim, float radius, float height)
{
    // Units via the source abstraction (works under USD and ovstage) — no direct
    // UsdGeom read. SourceUnits up-axis is Y/Z only (USD never reports X-up).
    const omni::physics::parse::SourceUnits units = attachedStage.getSourceUnits();
    const float metersPerUnit = units.metersPerUnit;
    float tolerancesLength = float(1.0f / metersPerUnit);

    PxCapsuleControllerDesc cctDesc;
    cctDesc.setToDefault();
    cctDesc.contactOffset *= tolerancesLength;
    cctDesc.stepOffset = (height+radius*2.0f) * 0.2f;
	cctDesc.radius = radius;
	cctDesc.height = height;
    cctDesc.behaviorCallback = &cctBehaviorCallback;

    cctDesc.upDirection = (units.upAxis == omni::physics::parse::UpAxis::eY)
                              ? PxVec3(0.0f, 1.0f, 0.0f)
                              : PxVec3(0.0f, 0.0f, 1.0f);

    const PhysxSchemaPhysxCharacterControllerAPI physxCctAPI =
        PhysxSchemaPhysxCharacterControllerAPI::Get(stage, usdPrim.GetPrimPath());
    if (physxCctAPI)
    {
        getAttribute(attachedStage, cctDesc.invisibleWallHeight, physxCctAPI.GetInvisibleWallHeightAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(attachedStage, cctDesc.maxJumpHeight, physxCctAPI.GetMaxJumpHeightAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(attachedStage, cctDesc.contactOffset, physxCctAPI.GetContactOffsetAttr(), -FLT_MAX, FLT_MAX, updateCctContactOffset);
        getAttribute(attachedStage, cctDesc.stepOffset, physxCctAPI.GetStepOffsetAttr(), 0.0f, height+radius*2.0f, updateCctStepOffset);
        getAttribute(attachedStage, cctDesc.scaleCoeff, physxCctAPI.GetScaleCoeffAttr(), -FLT_MAX, FLT_MAX, nullptr);
        getAttribute(attachedStage, cctDesc.volumeGrowth, physxCctAPI.GetVolumeGrowthAttr(), -FLT_MAX, FLT_MAX, nullptr);

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
