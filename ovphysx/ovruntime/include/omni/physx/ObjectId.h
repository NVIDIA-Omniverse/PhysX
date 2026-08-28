// SPDX-FileCopyrightText: Copyright (c) 2022-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <carb/Defines.h>
#include <carb/Types.h>


namespace omni
{
namespace physx
{
namespace usdparser
{
/// @brief Consumer-side (engine) identity for a registered physics object.
///
/// An ObjectId is minted by the runtime when a parsed descriptor is
/// handed off to the simulation backend (via `IPhysxPhysicsInterface::
/// createObject` / the `ObjectDb::findOrCreateEntry` path). It then
/// names that engine-side entity for the rest of the stage's lifetime
/// — scene IDs, joint endpoint refs, articulation links, tendon parents,
/// material bindings on shapes, etc. all use ObjectId.
///
/// Distinct from `omni::physics::parse::ObjectKey` (re-exported into
/// the parse-lib namespace; see `include/omni/physics/parse/Handles.h`):
/// `ObjectKey` is the *parse-time, source-side* identity carried in the
/// descriptors `scanStage` emits — it names a prim / a procedural node /
/// a mock entity inside the IPhysicsSource. The consumer translates each
/// source-side `ObjectKey` to a runtime `ObjectId` as it registers the
/// parsed objects.
///
/// Lifecycle:
///   1. `scanStage` emits descriptors with `ObjectKey` cross-references
///      (`primPath`, `materialPath`, `rigidBody`, ...).
///   2. Consumer iterates the typed lists; for each entry it allocates
///      the engine object and records `(ObjectKey, ObjectId)` in its
///      `ObjectDb` for the relevant `ObjectCategory`.
///   3. Subsequent consumer-side passes resolve descriptor `ObjectKey`
///      cross-references via `ObjectDb::findEntry` and write the resulting
///      `ObjectId`s into engine-side slots (`sceneIds`, `collisionGroup`,
///      `parentId`, ...).
typedef size_t ObjectId;

/// @brief Invalid sentinel for ObjectId. Used both as a never-assigned
/// initial value and as a "lookup failed" return from `ObjectDb::findEntry`.
const ObjectId kInvalidObjectId = ObjectId(0xFFffFFffFF);

} // namespace usdparser
} // namespace physx
} // namespace omni
