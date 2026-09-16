// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-9
 */

#include "TimeSampledCallbacks.h"

#include "AttachedStage.h"

#include <propertiesUpdate/PhysXPropertiesUpdate.h>

#include <omni/physics/parse/KnownTokens.h>

namespace omni::physx::usdparser::callbacks
{

using namespace ::omni::physx;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;
using omni::physics::parse::TokenId;

// physxSurfaceVelocity:* attribute names have no KnownTokens entry yet
// (out of scope for this batch -- KnownTokens.h is shared with other
// in-flight work). Interned directly here instead of round-tripping through
// the generated PhysxSchemaTokens table.
static TokenId physxSurfaceVelocitySurfaceVelocityEnabledTok(const IPhysicsSource& src)
{
    return src.internToken("physxSurfaceVelocity:surfaceVelocityEnabled");
}
static TokenId physxSurfaceVelocitySurfaceVelocityLocalSpaceTok(const IPhysicsSource& src)
{
    return src.internToken("physxSurfaceVelocity:surfaceVelocityLocalSpace");
}
static TokenId physxSurfaceVelocitySurfaceVelocityTok(const IPhysicsSource& src)
{
    return src.internToken("physxSurfaceVelocity:surfaceVelocity");
}
static TokenId physxSurfaceVelocitySurfaceAngularVelocityTok(const IPhysicsSource& src)
{
    return src.internToken("physxSurfaceVelocity:surfaceAngularVelocity");
}

// Gate matches the legacy `getAttribute(...)` template in `AttributeHelpers.h`:
// register only when the attribute is authored AND might be time-varying.
void maybeCollect(TimeSampledCallbackList& out,
                  const IPhysicsSource& src,
                  ObjectKey key,
                  TokenId attr,
                  OnUpdateObjectFn updateFn)
{
    if (!updateFn)
        return;
    if (src.hasAuthoredAttribute(key, attr) && src.mightBeTimeVarying(key, attr))
        out.push_back({ key, attr, updateFn });
}

// Stricter gate: register only when the attribute has more than one time sample
// (more-than-one-sample implies authored). Matches the explicit
// `GetNumTimeSamples() > 1` sites; `isAttributeTimeSampled` is that predicate.
void maybeCollectIfMultiSample(TimeSampledCallbackList& out,
                               const IPhysicsSource& src,
                               ObjectKey key,
                               TokenId attr,
                               OnUpdateObjectFn updateFn)
{
    if (!updateFn)
        return;
    if (src.isAttributeTimeSampled(key, attr))
        out.push_back({ key, attr, updateFn });
}

// Matches the explicit `if (attr.ValueMightBeTimeVarying()) register(...)`
// pattern (e.g. the rigid body's `physicsVelocity` registration). Skips the
// authored gate.
static void maybeCollectIfTimeVarying(TimeSampledCallbackList& out,
                                      const IPhysicsSource& src,
                                      ObjectKey key,
                                      TokenId attr,
                                      OnUpdateObjectFn updateFn)
{
    if (!updateFn)
        return;
    if (src.mightBeTimeVarying(key, attr))
        out.push_back({ key, attr, updateFn });
}

// ---------------------------------------------------------------------------
// Shape
// ---------------------------------------------------------------------------
//
// Matches `Collision.cpp::fillPhysxShapeDesc`: callbacks for `contactOffset`,
// `restOffset`, `torsionalPatchRadius`, `minTorsionalPatchRadius` on
// `PhysxCollisionAPI`.
void collectShapeTimeSampledCallbacks(const AttachedStage& attachedStage, ObjectKey key, TimeSampledCallbackList& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (!src->hasSchema(key, tok.physxCollisionAPI))
        return;
    // contactOffset / restOffset use the stricter `GetNumTimeSamples > 1` gate;
    // torsionalPatchRadius / minTorsionalPatchRadius use the looser gate.
    maybeCollectIfMultiSample(out, *src, key, tok.physxCollisionContactOffset,           updateShapeContactOffset);
    maybeCollectIfMultiSample(out, *src, key, tok.physxCollisionRestOffset,              updateShapeRestOffset);
    maybeCollect             (out, *src, key, tok.physxCollisionTorsionalPatchRadius,    updateShapeTorsionalPatchRadius);
    maybeCollect             (out, *src, key, tok.physxCollisionMinTorsionalPatchRadius, updateShapeMinTorsionalPatchRadius);
}

// ---------------------------------------------------------------------------
// RigidBody
// ---------------------------------------------------------------------------
//
// Matches `PhysicsBody.cpp::parseRigidBody` (PhysxRigidBodyAPI),
// (PhysxSurfaceVelocityAPI), and the explicit `physicsVelocity` registration.
void collectRigidBodyTimeSampledCallbacks(const AttachedStage& attachedStage, ObjectKey key, TimeSampledCallbackList& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

    if (src->hasSchema(key, tok.physxRigidBodyAPI))
    {
        maybeCollect(out, *src, key, tok.physxRigidBodyLinearDamping,                updateBodyLinearDamping);
        maybeCollect(out, *src, key, tok.physxRigidBodyAngularDamping,               updateBodyAngularDamping);
        maybeCollect(out, *src, key, tok.physxRigidBodyMaxLinearVelocity,            updateBodyMaxLinearVelocity);
        maybeCollect(out, *src, key, tok.physxRigidBodyMaxAngularVelocity,           updateBodyMaxAngularVelocity);
        maybeCollect(out, *src, key, tok.physxRigidBodySleepThreshold,               updateBodySleepThreshold);
        maybeCollect(out, *src, key, tok.physxRigidBodyStabilizationThreshold,       updateBodyStabilizationThreshold);
        maybeCollect(out, *src, key, tok.physxRigidBodyMaxDepenetrationVelocity,     updateBodyMaxDepenetrationVelocity);
        maybeCollect(out, *src, key, tok.physxRigidBodyContactSlopCoefficient,       updateBodyContactSlopCoefficient);
        maybeCollect(out, *src, key, tok.physxRigidBodyMaxContactImpulse,            updateBodyMaxContactImpulse);
        maybeCollect(out, *src, key, tok.physxRigidBodyCfmScale,                     updateBodyCfmScale);
        maybeCollect(out, *src, key, tok.physxRigidBodySolverPositionIterationCount, updateBodySolverPositionIterationCount);
        maybeCollect(out, *src, key, tok.physxRigidBodySolverVelocityIterationCount, updateBodySolverVelocityIterationCount);
        maybeCollect(out, *src, key, tok.physxRigidBodyEnableCCD,                    updateBodyEnableCCD);
        maybeCollect(out, *src, key, tok.physxRigidBodyEnableSpeculativeCCD,         updateBodyEnableSpeculativeCCD);
        maybeCollect(out, *src, key, tok.physxRigidBodyDisableGravity,               updateBodyDisableGravity);
        maybeCollect(out, *src, key, tok.physxRigidBodyRetainAccelerations,          updateBodyRetainAccelerations);
        maybeCollect(out, *src, key, tok.physxRigidBodyEnableGyroscopicForces,       updateBodyGyroscopicForces);
        maybeCollect(out, *src, key, tok.physxRigidBodySolveContact,                 updateBodySolveContacts);
        maybeCollect(out, *src, key, tok.physxRigidBodyLockedPosAxis,                updateBodyLockedPosAxis);
        maybeCollect(out, *src, key, tok.physxRigidBodyLockedRotAxis,                updateBodyLockedRotAxis);
    }

    if (src->hasSchema(key, tok.physxSurfaceVelocityAPI))
    {
        maybeCollect(out, *src, key, physxSurfaceVelocitySurfaceVelocityEnabledTok(*src),    updateBodySurfaceVelocityEnabled);
        maybeCollect(out, *src, key, physxSurfaceVelocitySurfaceVelocityLocalSpaceTok(*src), updateBodySurfaceVelocityLocalSpace);
        maybeCollect(out, *src, key, physxSurfaceVelocitySurfaceVelocityTok(*src),           updateBodySurfaceLinearVelocity);
        maybeCollect(out, *src, key, physxSurfaceVelocitySurfaceAngularVelocityTok(*src),    updateBodySurfaceAngularVelocity);
    }

    // The `physicsVelocity` attribute on the body prim itself is registered
    // without the authored gate in the legacy path.
    if (src->hasSchema(key, tok.physicsRigidBodyAPI))
    {
        maybeCollectIfTimeVarying(out, *src, key, tok.physicsVelocity, updateBodyLinearVelocity);
    }
}

// ---------------------------------------------------------------------------
// Articulation
// ---------------------------------------------------------------------------
//
// Matches `Articulation.cpp::parseArticulation` — four PhysxArticulationAPI
// callbacks.
void collectArticulationTimeSampledCallbacks(const AttachedStage& attachedStage, ObjectKey key, TimeSampledCallbackList& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (!src->hasSchema(key, tok.physxArticulationAPI))
        return;
    maybeCollect(out, *src, key, tok.physxArticulationSleepThreshold,               updateArticulationSleepThreshold);
    maybeCollect(out, *src, key, tok.physxArticulationStabilizationThreshold,       updateArticulationStabilizationThreshold);
    maybeCollect(out, *src, key, tok.physxArticulationSolverPositionIterationCount, updateArticulationSolverPositionIterationCount);
    maybeCollect(out, *src, key, tok.physxArticulationSolverVelocityIterationCount, updateArticulationSolverVelocityIterationCount);
}

// ---------------------------------------------------------------------------
// DeformableBody
// ---------------------------------------------------------------------------
//
// Matches `PhysicsBody.cpp::parseDeformableBody` — two PhysxCollisionAPI
// callbacks (contactOffset / restOffset) gated by `GetNumTimeSamples > 1`.
void collectDeformableBodyTimeSampledCallbacks(const AttachedStage& attachedStage, ObjectKey key, TimeSampledCallbackList& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (!src->hasSchema(key, tok.physxCollisionAPI))
        return;
    maybeCollectIfMultiSample(out, *src, key, tok.physxCollisionContactOffset, updateDeformableContactOffset);
    maybeCollectIfMultiSample(out, *src, key, tok.physxCollisionRestOffset,    updateDeformableRestOffset);
}

// ---------------------------------------------------------------------------
// PhysxForce
// ---------------------------------------------------------------------------
//
// Matches `PhysicsBody.cpp::parsePhysxForce` — five PhysxForceAPI callbacks.
void collectPhysxForceTimeSampledCallbacks(const AttachedStage& attachedStage, ObjectKey key, TimeSampledCallbackList& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (!src->hasSchema(key, tok.physxForceAPI))
        return;
    maybeCollect(out, *src, key, tok.physxForceForceEnabled,      updatePhysxForceEnabled);
    maybeCollect(out, *src, key, tok.physxForceWorldFrameEnabled, updatePhysxForceWorldFrameEnabled);
    maybeCollect(out, *src, key, tok.physxForceForce,             updatePhysxForce);
    maybeCollect(out, *src, key, tok.physxForceTorque,            updatePhysxTorque);
    maybeCollect(out, *src, key, tok.physxForceMode,              updatePhysxForceMode);
}

// ---------------------------------------------------------------------------
// Apply
// ---------------------------------------------------------------------------

void applyTimeSampledCallbacks(AttachedStage& attachedStage,
                               const TimeSampledCallbackList& list)
{
    for (const TimeSampledCallback& cb : list)
    {
        if (!cb.key.valid() || !cb.updateFn)
            continue;
        attachedStage.registerTimeSampledAttribute(cb.key, cb.attr, cb.updateFn);
    }
}

} // namespace omni::physx::usdparser::callbacks
