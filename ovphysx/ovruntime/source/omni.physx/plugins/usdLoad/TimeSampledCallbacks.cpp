// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-9
 */

#include "UsdPCH.h"

#include "TimeSampledCallbacks.h"

#include "AttachedStage.h"

#include <propertiesUpdate/PhysXPropertiesUpdate.h>

// USD + PhysxSchema headers are pulled in transitively by `UsdPCH.h`.
#include <pxr/usd/usdPhysics/tokens.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <physxSchema/physxRigidBodyAPI.h>
#include <physxSchema/physxCollisionAPI.h>
#include <physxSchema/physxSurfaceVelocityAPI.h>
#include <physxSchema/physxArticulationAPI.h>
#include <physxSchema/physxForceAPI.h>
#include <physxSchema/tokens.h>

namespace omni::physx::usdparser::callbacks
{

using namespace ::omni::physx;
using namespace PXR_NS;
using omni::physics::parse::IPhysicsSource;
using omni::physics::parse::ObjectKey;
using omni::physics::parse::TokenId;

// Applied-schema name token for a C++ schema type, interned in the source.
// Mirrors the legacy `PhysxSchemaXxxAPI::Get(...); if (api)` existence check via
// `IPhysicsSource::hasSchema` (single-apply schema == HasAPI<>).
template <typename SchemaT>
static TokenId schemaTok(const IPhysicsSource& src)
{
    static const std::string name = UsdSchemaRegistry::GetSchemaTypeName(TfType::Find<SchemaT>()).GetString();
    return src.internToken(name);
}

// Gate matches the legacy `getAttribute(...)` template in `AttributeHelpers.h`:
// register only when the attribute is authored AND might be time-varying.
void maybeCollect(TimeSampledCallbackList& out,
                  const IPhysicsSource& src,
                  ObjectKey key,
                  const TfToken& attr,
                  OnUpdateObjectFn updateFn)
{
    if (!updateFn)
        return;
    const TokenId t = src.internToken(attr.GetString());
    if (src.hasAuthoredAttribute(key, t) && src.mightBeTimeVarying(key, t))
        out.push_back({ key, attr, updateFn });
}

// Stricter gate: register only when the attribute has more than one time sample
// (more-than-one-sample implies authored). Matches the explicit
// `GetNumTimeSamples() > 1` sites; `isAttributeTimeSampled` is that predicate.
void maybeCollectIfMultiSample(TimeSampledCallbackList& out,
                               const IPhysicsSource& src,
                               ObjectKey key,
                               const TfToken& attr,
                               OnUpdateObjectFn updateFn)
{
    if (!updateFn)
        return;
    const TokenId t = src.internToken(attr.GetString());
    if (src.isAttributeTimeSampled(key, t))
        out.push_back({ key, attr, updateFn });
}

// Matches the explicit `if (attr.ValueMightBeTimeVarying()) register(...)`
// pattern (e.g. the rigid body's `physicsVelocity` registration). Skips the
// authored gate.
static void maybeCollectIfTimeVarying(TimeSampledCallbackList& out,
                                      const IPhysicsSource& src,
                                      ObjectKey key,
                                      const TfToken& attr,
                                      OnUpdateObjectFn updateFn)
{
    if (!updateFn)
        return;
    const TokenId t = src.internToken(attr.GetString());
    if (src.mightBeTimeVarying(key, t))
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
    if (!src || !src->hasSchema(key, schemaTok<PhysxSchemaPhysxCollisionAPI>(*src)))
        return;
    // contactOffset / restOffset use the stricter `GetNumTimeSamples > 1` gate;
    // torsionalPatchRadius / minTorsionalPatchRadius use the looser gate.
    maybeCollectIfMultiSample(out, *src, key, PhysxSchemaTokens->physxCollisionContactOffset,           updateShapeContactOffset);
    maybeCollectIfMultiSample(out, *src, key, PhysxSchemaTokens->physxCollisionRestOffset,              updateShapeRestOffset);
    maybeCollect             (out, *src, key, PhysxSchemaTokens->physxCollisionTorsionalPatchRadius,    updateShapeTorsionalPatchRadius);
    maybeCollect             (out, *src, key, PhysxSchemaTokens->physxCollisionMinTorsionalPatchRadius, updateShapeMinTorsionalPatchRadius);
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

    if (src->hasSchema(key, schemaTok<PhysxSchemaPhysxRigidBodyAPI>(*src)))
    {
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyLinearDamping,                updateBodyLinearDamping);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyAngularDamping,               updateBodyAngularDamping);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyMaxLinearVelocity,            updateBodyMaxLinearVelocity);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyMaxAngularVelocity,           updateBodyMaxAngularVelocity);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodySleepThreshold,               updateBodySleepThreshold);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyStabilizationThreshold,       updateBodyStabilizationThreshold);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyMaxDepenetrationVelocity,     updateBodyMaxDepenetrationVelocity);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyContactSlopCoefficient,       updateBodyContactSlopCoefficient);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyMaxContactImpulse,            updateBodyMaxContactImpulse);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyCfmScale,                     updateBodyCfmScale);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodySolverPositionIterationCount, updateBodySolverPositionIterationCount);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodySolverVelocityIterationCount, updateBodySolverVelocityIterationCount);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyEnableCCD,                    updateBodyEnableCCD);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyEnableSpeculativeCCD,         updateBodyEnableSpeculativeCCD);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyDisableGravity,               updateBodyDisableGravity);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyRetainAccelerations,          updateBodyRetainAccelerations);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyEnableGyroscopicForces,       updateBodyGyroscopicForces);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodySolveContact,                 updateBodySolveContacts);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyLockedPosAxis,                updateBodyLockedPosAxis);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxRigidBodyLockedRotAxis,                updateBodyLockedRotAxis);
    }

    if (src->hasSchema(key, schemaTok<PhysxSchemaPhysxSurfaceVelocityAPI>(*src)))
    {
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityEnabled,    updateBodySurfaceVelocityEnabled);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocityLocalSpace, updateBodySurfaceVelocityLocalSpace);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxSurfaceVelocitySurfaceVelocity,           updateBodySurfaceLinearVelocity);
        maybeCollect(out, *src, key, PhysxSchemaTokens->physxSurfaceVelocitySurfaceAngularVelocity,    updateBodySurfaceAngularVelocity);
    }

    // The `physicsVelocity` attribute on the body prim itself is registered
    // without the authored gate in the legacy path.
    if (src->hasSchema(key, schemaTok<UsdPhysicsRigidBodyAPI>(*src)))
    {
        maybeCollectIfTimeVarying(out, *src, key, UsdPhysicsTokens->physicsVelocity, updateBodyLinearVelocity);
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
    if (!src || !src->hasSchema(key, schemaTok<PhysxSchemaPhysxArticulationAPI>(*src)))
        return;
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxArticulationSleepThreshold,               updateArticulationSleepThreshold);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxArticulationStabilizationThreshold,       updateArticulationStabilizationThreshold);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxArticulationSolverPositionIterationCount, updateArticulationSolverPositionIterationCount);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxArticulationSolverVelocityIterationCount, updateArticulationSolverVelocityIterationCount);
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
    if (!src || !src->hasSchema(key, schemaTok<PhysxSchemaPhysxCollisionAPI>(*src)))
        return;
    maybeCollectIfMultiSample(out, *src, key, PhysxSchemaTokens->physxCollisionContactOffset, updateDeformableContactOffset);
    maybeCollectIfMultiSample(out, *src, key, PhysxSchemaTokens->physxCollisionRestOffset,    updateDeformableRestOffset);
}

// ---------------------------------------------------------------------------
// PhysxForce
// ---------------------------------------------------------------------------
//
// Matches `PhysicsBody.cpp::parsePhysxForce` — five PhysxForceAPI callbacks.
void collectPhysxForceTimeSampledCallbacks(const AttachedStage& attachedStage, ObjectKey key, TimeSampledCallbackList& out)
{
    const IPhysicsSource* src = attachedStage.getSource();
    if (!src || !src->hasSchema(key, schemaTok<PhysxSchemaPhysxForceAPI>(*src)))
        return;
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxForceForceEnabled,      updatePhysxForceEnabled);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxForceWorldFrameEnabled, updatePhysxForceWorldFrameEnabled);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxForceForce,             updatePhysxForce);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxForceTorque,            updatePhysxTorque);
    maybeCollect(out, *src, key, PhysxSchemaTokens->physxForceMode,              updatePhysxForceMode);
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
        const SdfPath primPath = attachedStage.pathFor(cb.key);
        if (!primPath.IsEmpty())
            attachedStage.registerTimeSampledAttribute(primPath.AppendProperty(cb.attr), cb.updateFn);
    }
}

} // namespace omni::physx::usdparser::callbacks
