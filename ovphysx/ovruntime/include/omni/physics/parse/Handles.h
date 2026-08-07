// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-CORE-002
 * @covers AC-1 AC-2 AC-3 AC-4
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace omni::physics::parse
{

/// @brief Opaque identity of an object exposed by an IPhysicsSource.
///
/// An ObjectKey names a source-side object (a prim, a procedural node,
/// a mock entity) within a single Source. It is *parse-time* identity:
/// every `IPhysicsSource` method that names "an object" takes an
/// `ObjectKey`, and every descriptor field that points back to such an
/// object holds an `ObjectKey` (e.g. `PhysxShapeDesc::primKey`,
/// `PhysxShapeDesc::rigidBody`, `PhysxRigidBodyDesc::sourceCollisions`).
///
/// Lifecycle, paired with `ObjectId`:
///   1. `IPhysicsSource` mints `ObjectKey`s for every source object the
///      walker encounters; descriptors carry them for cross-references.
///   2. Consumer iterates the descriptor lists and, for each entry,
///      allocates an engine object — minting an `ObjectId` that names
///      the runtime entity. The `(ObjectKey, ObjectId)` mapping is
///      recorded in `ObjectDb` under the matching `ObjectCategory`.
///   3. Post-parse passes resolve descriptor `ObjectId` slots
///      (`sceneIds`, `collisionGroup`, `parentId`, ...) via
///      `ObjectDb::findEntry`.
///
/// Distinct from `parse::ObjectId` (re-exported from
/// `omni::physx::usdparser::ObjectId` — see `omni/physx/ObjectId.h`).
/// Two conceptually different namespaces: `ObjectKey` lives inside the
/// IPhysicsSource boundary (a key into the source's vocabulary);
/// `ObjectId` lives inside the simulation engine (an id for a registered
/// runtime object). Confusion risk: descriptor fields named `*Path`/`*Prim`
/// historically held SdfPaths but are now `ObjectKey`s; fields named
/// `*Id`/`sceneId`/`parentId`/`collisionGroup` are `ObjectId`s.
///
/// A zero `handle` is the invalid sentinel.
struct ObjectKey
{
    uint64_t handle = 0;
    bool operator==(ObjectKey o) const { return handle == o.handle; }
    bool operator!=(ObjectKey o) const { return handle != o.handle; }
    /// @brief False when this is the invalid sentinel (`handle == 0`).
    bool valid() const { return handle != 0; }
    struct Hash
    {
        size_t operator()(ObjectKey k) const { return k.handle; }
    };
};

/// @brief Interned token id, minted by `IPhysicsSource::internToken`.
///
/// Tokens identify attribute names, schema names, relationship names, etc.
/// — anything addressed by a string in the source's vocabulary. The id is
/// stable for the lifetime of the Source; a zero `id` is the invalid sentinel.
struct TokenId
{
    uint32_t id = 0;
    bool operator==(TokenId o) const { return id == o.id; }
    bool operator!=(TokenId o) const { return id != o.id; }
    /// @brief False when this is the invalid sentinel (`id == 0`).
    bool valid() const { return id != 0; }
    struct Hash
    {
        size_t operator()(TokenId t) const { return t.id; }
    };
};

/// @brief Element type carried by a BufferHandle's payload.
///
/// Names describe the layout, not the source type. `eVec2/3/4` denote
/// 2/3/4 packed `float` lanes per element (layout-compatible with both
/// `carb::Float2/3/4` and `physx::PxVec2/3/4`), `eQuat` denotes 4 packed
/// floats in (x,y,z,w) order, `eTransform` denotes a 4x4 row-major double
/// matrix per element.
enum class BufferElemType : uint16_t
{
    eFloat,
    eInt32,
    eUInt32,
    eVec2,
    eVec3,
    eVec4,
    eQuat,
    eTransform,
    eInt3, ///< 3 packed int32 lanes per element (layout-compatible with GfVec3i) — mesh topology.
    eInt4, ///< 4 packed int32 lanes per element (layout-compatible with GfVec4i) — tet topology.
    eQuath, ///< 4 packed 16-bit half lanes per element (layout-compatible with GfQuath) — instancer orientations.
    eUInt8, ///< 1 byte per element (layout-compatible with `unsigned char`) — packed binary blobs (e.g. cooking mesh-key markers).
    eUInt16 ///< 2 bytes per element (layout-compatible with `uint16_t`) — per-face material indices for cooking.
};

/// @brief Buffer "fingerprint" minted by the source backend.
///
/// `id` is a per-source monotonic identifier; `resolveBuffer(id, byteCount)`
/// returns the raw pointer. `contentHash` is a 128-bit fnv hash of the
/// buffer's bytes — same hash function the cooking service uses internally
/// (`carb::extras::fnv128hash`, exposed via `MeshKey::computeVerticesHash`).
/// Two buffers with identical content produce identical `contentHash`es
/// regardless of which prim authored them, enabling content-keyed dedup
/// in the cooking-cache layer.
struct BufferHandle
{
    uint64_t id = 0;
    uint64_t contentHash[2] = { 0, 0 };
    uint32_t elemCount = 0;
    BufferElemType type = BufferElemType::eFloat;
    bool valid() const { return id != 0; }
};

// Lightweight ptr+count view returned by ParseContext::getBuffer<T>.
// C++17 has no std::span; this fills the role until we move to C++20.
template <typename T>
struct BufferSpan
{
    const T* data = nullptr;
    size_t   count = 0;

    bool empty() const { return count == 0; }
    const T& operator[](size_t i) const { return data[i]; }
    const T* begin() const { return data; }
    const T* end()   const { return data + count; }
};

} // namespace omni::physics::parse
