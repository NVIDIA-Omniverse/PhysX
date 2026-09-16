// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-2
 *
 * @implements REQ-PROPS-CCT-001
 * @covers AC-2
 */

#include "PhysXUSDProperties.h"
#include "propertiesUpdate/PhysXPropertiesUpdate.h"

#include "usdLoad/AttributeHelpers.h"
#include "PhysXTools.h"

#include <omni/physics/parse/KnownTokens.h>

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
PxCapsuleControllerDesc parsePhysXCharacterControllerDesc(usdparser::AttachedStage& attachedStage,
                                                          omni::physics::parse::ObjectKey key,
                                                          float radius,
                                                          float height)
{
    // Units via the source abstraction (works under USD and ovstage) — no direct
    // UsdGeom read. SourceUnits up-axis is Y/Z only (USD never reports X-up).
    const omni::physics::parse::SourceUnits units = attachedStage.getSourceUnits();
    const float metersPerUnit = units.metersPerUnit;
    float tolerancesLength = float(1.0f / metersPerUnit);

    PxCapsuleControllerDesc cctDesc;
    cctDesc.setToDefault();
    // PxControllerDesc's raw defaults happen to equal the USD schema's own
    // generatedSchema.usda defaults (0.1 / 0.5) -- captured here, before they're
    // overwritten below, so the getAttribute() reads further down can tell a
    // genuinely authored value apart from ovstage's always-authored fallback
    // (see the ADR-0002 "ambiguous fallback" collapse used below).
    const float rawSchemaContactOffset = cctDesc.contactOffset;
    const float rawSchemaStepOffset = cctDesc.stepOffset;
    cctDesc.contactOffset *= tolerancesLength;
    cctDesc.stepOffset = (height+radius*2.0f) * 0.2f;
	cctDesc.radius = radius;
	cctDesc.height = height;
    cctDesc.behaviorCallback = &cctBehaviorCallback;

    cctDesc.upDirection = (units.upAxis == omni::physics::parse::UpAxis::eY)
                              ? PxVec3(0.0f, 1.0f, 0.0f)
                              : PxVec3(0.0f, 0.0f, 1.0f);

    // Schema presence and every read go through the source: this used to build a
    // PhysxSchemaPhysxCharacterControllerAPI on the backing stage, which is a null
    // deref (not a silent miss) under a USD-free attach.
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    omni::physics::parse::KnownTokens tok;
    if (src)
        tok.intern(*src);
    if (src && src->exists(key) && src->hasSchema(key, tok.physxCharacterControllerAPI))
    {
        getAttribute(attachedStage, cctDesc.invisibleWallHeight, key, tok.physxCharacterControllerInvisibleWallHeight,
                    -FLT_MAX, FLT_MAX, omni::physics::parse::ReadTime::defaultTime(), nullptr);
        getAttribute(attachedStage, cctDesc.maxJumpHeight, key, tok.physxCharacterControllerMaxJumpHeight, -FLT_MAX,
                    FLT_MAX, omni::physics::parse::ReadTime::defaultTime(), nullptr);
        // ovstage reports every recognised attribute as authored (ADR-0020), so an
        // unauthored contactOffset/stepOffset reads back as the raw schema fallback
        // rather than leaving the units/geometry-scaled default above untouched.
        // A read that lands exactly on the raw fallback is indistinguishable from
        // "unauthored" and is treated as such (ADR-0002's ambiguous-fallback
        // collapse), the same trade-off already accepted for physxConvexGeometry:margin.
        float contactOffset{};
        if (getAttribute(attachedStage, contactOffset, key, tok.physxCharacterControllerContactOffset, -FLT_MAX,
                         FLT_MAX, omni::physics::parse::ReadTime::defaultTime(), updateCctContactOffset) &&
            contactOffset != rawSchemaContactOffset)
        {
            cctDesc.contactOffset = contactOffset;
        }
        float stepOffset{};
        if (getAttribute(attachedStage, stepOffset, key, tok.physxCharacterControllerStepOffset, 0.0f,
                         height + radius * 2.0f, omni::physics::parse::ReadTime::defaultTime(), updateCctStepOffset) &&
            stepOffset != rawSchemaStepOffset)
        {
            cctDesc.stepOffset = stepOffset;
        }
        getAttribute(attachedStage, cctDesc.scaleCoeff, key, tok.physxCharacterControllerScaleCoeff, -FLT_MAX,
                    FLT_MAX, omni::physics::parse::ReadTime::defaultTime(), nullptr);
        getAttribute(attachedStage, cctDesc.volumeGrowth, key, tok.physxCharacterControllerVolumeGrowth, -FLT_MAX,
                    FLT_MAX, omni::physics::parse::ReadTime::defaultTime(), nullptr);

        omni::physics::parse::TokenId upAxis;
        if (internal::getValue<omni::physics::parse::TokenId>(
                attachedStage, key, tok.physxCharacterControllerUpAxis, omni::physics::parse::ReadTime::defaultTime(), upAxis))
        {
            // tok.x/y/z are interned from the uppercase "X"/"Y"/"Z" literals (see
            // KnownTokens.cpp) -- the same vocabulary as this attribute's values, despite
            // the lowercase field name.
            if (upAxis == tok.x)
                cctDesc.upDirection = PxVec3(1.0f, 0.0f, 0.0f);
            else if (upAxis == tok.y)
                cctDesc.upDirection = PxVec3(0.0f, 1.0f, 0.0f);
            else if (upAxis == tok.z)
                cctDesc.upDirection = PxVec3(0.0f, 0.0f, 1.0f);
        }

        omni::physics::parse::TokenId nonWM;
        if (internal::getValue<omni::physics::parse::TokenId>(
                attachedStage, key, tok.physxCharacterControllerNonWalkableMode, omni::physics::parse::ReadTime::defaultTime(), nonWM))
        {
            if (nonWM == tok.preventClimbing)
                cctDesc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING;
            else if (nonWM == tok.preventClimbingForceSliding)
                cctDesc.nonWalkableMode = PxControllerNonWalkableMode::ePREVENT_CLIMBING_AND_FORCE_SLIDING;
        }

        omni::physics::parse::TokenId climbMode;
        if (internal::getValue<omni::physics::parse::TokenId>(
                attachedStage, key, tok.physxCharacterControllerClimbingMode, omni::physics::parse::ReadTime::defaultTime(), climbMode))
        {
            if (climbMode == tok.easy)
                cctDesc.climbingMode = PxCapsuleClimbingMode::eEASY;
            else if (climbMode == tok.constrained)
                cctDesc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
        }
    }

    return cctDesc;
}

} // namespace physx
} // namespace omni
