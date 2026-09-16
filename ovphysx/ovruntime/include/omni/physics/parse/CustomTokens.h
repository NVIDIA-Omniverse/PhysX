// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * Process-wide custom-token registry — the source-agnostic core.
 *
 * Consumers (the `IPhysxCustomGeometry` / `IPhysxCustomJoint` extension points
 * plus the runtime's built-in token bootstrap) register the prim-type and
 * applied-API names that a walker must recognize during classification:
 *
 *   - **Shape tokens** — applied-API or prim-type names that mark a prim as a
 *     custom-geometry collider. A walker promotes such prims to `eCustomShape`
 *     (or to the typed mesh-merge path for `PhysxMeshMergeCollisionAPI`).
 *
 *   - **Joint tokens** — prim-type names for typed joint subclasses beyond the
 *     `UsdPhysicsJoint` schema variants (e.g. `PhysxPhysicsGearJoint`). A walker
 *     classifies these as `eJointCustom` so the consumer can dispatch them to
 *     the registered custom-joint callback.
 *
 *   - **PointInstancer tokens** — prim-type names for instancer-shaped prims
 *     whose subtrees a walker must prune (the consumer parses their prototypes
 *     through a separate scan).
 *
 * The tokens name *shape and joint types*, not USD constructs, so the registry
 * is source-agnostic and lives in the USD-free parse core. Both walkers consult
 * it: `omni.physics.usd/NativeWalker.cpp` through the `TfToken`-typed shim in
 * `omni/physics/usd/CustomTokens.h`, and `omni.physics.ovstage/OvstageWalker.cpp`
 * directly. Parking it in the USD backend made every registered type silently
 * invisible to an ovstage scan; this is the same relocation ADR-0002's
 * 2026-06-13 amendment applied to `ScannedStage` / `IScanBackend`, for the same
 * reason (`omni.physics.ovstage` is a *peer* of `omni.physics.usd`, not a
 * dependent).
 *
 * The registry is global, process-wide and thread-safe. Registration lasts until
 * unregistered or process exit.
 *
 * @implements REQ-PARSE-CORE-005
 * @covers AC-1 AC-3 AC-4
 */

#pragma once

#include <carb/extras/Hash.h>

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace omni::physics::parse
{

/// @brief Stable content hash of a custom-geometry token name.
///
/// This is the key `CustomPhysxShapeDesc::customGeometryTokenHash` carries and
/// the one `PhysXCustomGeometryManager` registers under, so registration and
/// every walker must agree on it. Defined here, beside the registry that names
/// the token, because both walkers need it and only one of them may name
/// `TfToken` — `private/omni/physx/CustomGeometryHash.h` forwards to this so
/// there is exactly one implementation.
///
/// Uses `carb::extras::fnv128hash` (the hash family the cooking service uses via
/// MeshKey) XOR-reduced to 64-bit, matching `MeshKey::getHashIndex`.
inline size_t customGeometryTokenHash(std::string_view tokenStr)
{
    const auto h = carb::extras::fnv128hash(reinterpret_cast<const uint8_t*>(tokenStr.data()), tokenStr.size());
    return static_cast<size_t>(h.d[0] ^ h.d[1]);
}

/// @brief Which of the three independent token sets a call addresses.
enum class CustomTokenKind
{
    eShape, //!< applied-API OR prim-type names marking a custom-geometry collider
    eJoint, //!< prim-type names for typed joint subclasses
    ePhysicsInstancer //!< prim-type names for instancer-shaped prims whose subtrees are pruned
};

/// @brief Add `token` to `kind`'s set. Idempotent; thread-safe.
void registerCustomToken(CustomTokenKind kind, std::string_view token);

/// @brief Remove `token` from `kind`'s set. A no-op if absent; thread-safe.
void unregisterCustomToken(CustomTokenKind kind, std::string_view token);

/// @brief Membership test against `kind`'s set. Thread-safe.
bool isCustomToken(CustomTokenKind kind, std::string_view token);

/// @brief Snapshot of `kind`'s set, in lexicographic order.
///
/// A walker that *enumerates* rather than traverses (the ovstage walker issues
/// one query per concept) needs the registered names, not just a membership
/// test — see ADR-0002 open question 1, which sizes this at one extra query per
/// registered token. The order is deterministic so the emit order of custom
/// prims does not depend on registration order.
std::vector<std::string> customTokens(CustomTokenKind kind);

} // namespace omni::physics::parse
