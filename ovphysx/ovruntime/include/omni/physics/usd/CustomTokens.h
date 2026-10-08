// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * `TfToken`-typed view of the process-wide custom-token registry.
 *
 * The registry itself is source-agnostic and lives in the USD-free parse
 * core — `omni/physics/parse/CustomTokens.h` — so the ovstage walker can
 * consult it too. These functions are a thin, behaviour-preserving shim over
 * it, kept because the runtime's extension points and `NativeWalker.cpp` are
 * written against `TfToken`.
 *
 * Consumers (e.g. `IPhysxCustomGeometry` / `IPhysxCustomJoint`
 * extension points + the runtime's built-in token bootstrap) call
 * these functions to register tokens that the walker must recognize
 * during classification:
 *
 *   - **Shape tokens** — applied-API or prim-type tokens that mark a
 *     prim as a custom-geometry shape. The walker's collider branch
 *     promotes such prims to `eCustomShape` (or the typed mesh-merge
 *     path when the token matches `PhysxMeshMergeCollisionAPI`).
 *
 *   - **Joint tokens** — prim-type tokens for typed joint subclasses
 *     beyond the seven `UsdPhysicsJoint` schema variants (e.g.
 *     `PhysxPhysicsGearJoint` / `PhysxPhysicsRackAndPinionJoint`).
 *     The walker classifies these as `eJointCustom`.
 *
 *   - **PointInstancer tokens** — prim-type tokens for custom
 *     PointInstancer-shaped prims (e.g. `PhysxPhysicsJointInstancer`)
 *     whose subtrees the walker must prune (consumer parses them via
 *     a separate `scanStage` call).
 *
 * The registry is global, process-wide, thread-safe. Lifetime is the
 * lifetime of the process; tokens registered at plugin load remain
 * registered until unregistered (or process exit).
 *
 * @implements REQ-PARSE-CORE-005
 * @covers AC-1 AC-2
 */

#pragma once

#include <pxr/base/tf/token.h>

namespace omni::physics::usd
{

// Custom-shape tokens — applied-API names OR prim-type names that
// mark a prim as a custom-geometry shape.
void registerCustomShapeToken(const PXR_NS::TfToken& token);
void unregisterCustomShapeToken(const PXR_NS::TfToken& token);

// Custom-joint tokens — prim-type names for typed joint subclasses.
void registerCustomJointToken(const PXR_NS::TfToken& token);
void unregisterCustomJointToken(const PXR_NS::TfToken& token);

// Custom-PointInstancer tokens — prim-type names whose subtrees the
// walker prunes (consumer-side scan handles them separately).
void registerCustomPhysicsInstancerToken(const PXR_NS::TfToken& token);
void unregisterCustomPhysicsInstancerToken(const PXR_NS::TfToken& token);

} // namespace omni::physics::usd
