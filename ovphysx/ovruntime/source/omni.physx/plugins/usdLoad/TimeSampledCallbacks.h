// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * Time-sampled attribute callback collection for consumers migrating
 * off `IUsdPhysicsListener`.
 *
 * Background.  Legacy `parse*` helpers (parseRigidBody, parseCollisionDesc,
 * parseSceneDesc, parseMaterialDesc, parseJoint, parseDeformableBody,
 * parseArticulation, parsePhysxForce, etc.) do TWO things on each tracked
 * attribute: read the value into the descriptor's field, AND register a
 * time-sampled change callback (`AttachedStage::registerTimeSampledAttribute`)
 * if the attribute is time-varying.
 *
 * After descriptor unification, `scanStage` produces the same descriptor
 * fields a parse helper would.  But the callback registration is
 * consumer-side state and is NOT performed by the parse library.
 * Migrations that bypass the legacy `parse*` helpers therefore lose the
 * change-tracking callbacks unless they re-register them explicitly.
 *
 * This header introduces a per-type `collectXxxTimeSampledCallbacks`
 * helper that walks a prim's API attributes, applies the same gate the
 * legacy `getAttribute(...)` template applies (`HasAuthoredValue() &&
 * ValueMightBeTimeVarying()`), and appends matching `(attribute,
 * updateFn)` pairs to a list.  Callers apply the list against an
 * `AttachedStage` via `applyTimeSampledCallbacks`.
 *
 * Why a list and not direct registration?  The list-of-tokens pattern
 * decouples the parse step (which can run with a throwaway / non-
 * registered `AttachedStage`) from the registration step (which always
 * targets the registered stage).
 *
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-9
 */

#pragma once

#include "ChangeParams.h"  // OnUpdateObjectFn

#include <omni/physics/parse/IPhysicsSource.h>  // IPhysicsSource, ObjectKey

#include <pxr/base/tf/token.h>

#include <vector>

namespace omni::physx::usdparser
{
class AttachedStage;
}

namespace omni::physx::usdparser::callbacks
{

// A collected callback: the object key + attribute-name token to register a
// time-sampled change callback for. Resolved against the physics source rather
// than holding a UsdAttribute, so collection no longer needs a UsdPrim.
struct TimeSampledCallback
{
    omni::physics::parse::ObjectKey key;
    PXR_NS::TfToken                 attr;
    OnUpdateObjectFn                updateFn;
};

using TimeSampledCallbackList = std::vector<TimeSampledCallback>;

// Common gate.  Append `(attribute, updateFn)` to `out` when the
// attribute is authored AND its value might be time-varying.  Matches
// the legacy `getAttribute(..., updateFn)` registration condition in
// `AttributeHelpers.h`.
void maybeCollect(TimeSampledCallbackList& out,
                  const omni::physics::parse::IPhysicsSource& src,
                  omni::physics::parse::ObjectKey key,
                  const PXR_NS::TfToken& attr,
                  OnUpdateObjectFn updateFn);

// Stricter gate.  Append only when the attribute is authored AND has
// more than one time sample.  Matches the explicit
// `GetNumTimeSamples() > 1` check used by a handful of legacy sites
// (Collision.cpp::fillPhysxShapeDesc lines 377-379 / 401-404 for shape
// contact/rest offsets; PhysicsBody.cpp lines 927-930 / 949-952 for
// deformable-body contact/rest offsets).  These sites are
// authoritative for those specific attributes; using `maybeCollect`
// instead would change the registration condition and may register
// callbacks in cases the legacy wouldn't (single-sample animations).
void maybeCollectIfMultiSample(TimeSampledCallbackList& out,
                               const omni::physics::parse::IPhysicsSource& src,
                               omni::physics::parse::ObjectKey key,
                               const PXR_NS::TfToken& attr,
                               OnUpdateObjectFn updateFn);

// Per-type collectors.  Each walks the prim's relevant API schemas and
// appends every time-sampleable attribute the legacy parse helper would
// have registered a callback for.  See the matching legacy site
// referenced in the cpp file.
//
// `prim` may be inactive / non-existent — collectors are no-ops in that
// case.  Multiple collectors may be called on the same prim (e.g., a
// prim that's both a body and a shape).
void collectShapeTimeSampledCallbacks(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, TimeSampledCallbackList& out);
void collectRigidBodyTimeSampledCallbacks(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, TimeSampledCallbackList& out);
void collectArticulationTimeSampledCallbacks(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, TimeSampledCallbackList& out);
void collectDeformableBodyTimeSampledCallbacks(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, TimeSampledCallbackList& out);
void collectPhysxForceTimeSampledCallbacks(const AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, TimeSampledCallbackList& out);

// NOTE: Scene, Material, DeformableMaterial, Joint, CollisionGroup,
// DeformableAttachment, ElementCollisionFilter — these types pass
// `nullptr` as the update function in every `getAttribute(...)` call in
// their legacy `parse*` helpers (audited 2026-05-11 across Scene.cpp,
// Material.cpp, Joint.cpp, Articulation.cpp, DeformableAttachment.cpp),
// so they have NO time-sampled callbacks to migrate.  No collector is
// needed for them; the migrated path can skip the apply step entirely.

// Register every collected callback against the `AttachedStage`.  Idempotent
// against the underlying map keyed by attribute path; calling twice for
// the same attribute is a no-op on the second call.
void applyTimeSampledCallbacks(AttachedStage& attachedStage,
                               const TimeSampledCallbackList& list);

} // namespace omni::physx::usdparser::callbacks
