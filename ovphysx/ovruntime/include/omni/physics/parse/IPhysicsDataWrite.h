// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-1 AC-2 AC-3 AC-8
 *
 * @implements REQ-WRITE-TRANSFORM-001
 * @covers AC-1 AC-4
 *
 * @implements REQ-WRITE-DATA-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-WRITE-ARRAY-001
 * @covers AC-1
 */

#pragma once

#include "Handles.h"

#include <cstddef>
#include <cstdint>

namespace omni::physics::parse
{

/// @brief Scalar component precision of a DataWriteView buffer.
///
/// This is the only type descriptor the write view needs: the element *shape*
/// (scalar vs vec3 vs quat ...) is taken from the destination attribute, so the
/// view just states the precision of each scalar. Notably USD point instancers
/// store `orientations` as half (`quath`) and most geometry as `float`.
///
/// Element bit-width precision, NOT element kind: `e32Bit` tags both 32-bit
/// float geometry and 32-bit integer connectivity — the element kind and shape
/// come from the destination attribute, so one width tag serves both. A 64-bit
/// width is intentionally NOT offered yet (no 64-bit producer exists; a width
/// the backend would silently downcast is worse than none); add `e64Bit` with
/// the matching 64-bit dispatch (`GfQuatd`/`double`/`GfVec3d`/`int64`) when a
/// 64-bit-source backend exists.
enum class DataType : uint8_t
{
    e16Bit, //!< 16-bit element (half-precision float)
    e32Bit, //!< 32-bit element (float or int32)
};

// ---------------------------------------------------------------------------
/// @brief Columnar, device-aware view over a contiguous batch of write data.
///
/// The write-side echo of how `IPhysicsSource` hands out buffers, shaped to
/// match the tensors batch model (`TensorDesc`) so a future GPU/DLPack column
/// path drops in without an API change. Today it describes a host (CPU) buffer.
/// The element shape comes from the destination attribute, so the view carries
/// only the scalar precision (`type`).
///
///   - `data`   : base pointer to the first element (read by the sink).
///   - `count`  : number of elements; for per-object batches this is the
///                object count and indexes 1:1 with the accompanying keys.
///   - `stride` : bytes between consecutive elements; 0 means tightly packed.
///   - `device` : -1 for host/CPU, >= 0 for a GPU device ordinal (future).
///   - `type`   : element bit-width precision (32-bit by default).
// ---------------------------------------------------------------------------
struct DataWriteView
{
    const void* data = nullptr;
    size_t count = 0;
    size_t stride = 0;
    int device = -1;
    DataType type = DataType::e32Bit;
};

/// @brief Abstract, source-agnostic sink for physics simulation output.
///
/// The write-side twin of `IPhysicsSource`: where the source exposes a scene
/// *into* the engine keyed by `ObjectKey`, `IPhysicsDataWrite` publishes the
/// engine's results *back out* to whatever backs the scene (USD today; a
/// columnar ovstage/Fabric layer in future). The engine speaks only this
/// interface and never names the backing representation.
///
/// Designed vectorized-first and GPU-ready: every output call takes a whole
/// batch of objects, and bulk payloads cross as `DataWriteView` columns that
/// carry a `device` ordinal — so the simulation's results (which may be
/// resident on a PhysX GPU device) can be published without a per-object call
/// and without an interface change when a device-resident path lands. The USD
/// backend consumes host (CPU) columns today.
///
/// Tokens are NOT minted here: there is one token vocabulary, owned by
/// `IPhysicsSource` (`internToken`). Callers intern attribute names through the
/// source and pass the resulting `TokenId` to `writeData` / `writeArray`; the
/// sink resolves it against that same source.
///
/// Three output shapes:
///   - `writeTransforms` — the per-step "poses out" special case: a full TRS
///     pose per object, batched (the "buffer of all transforms for all rigid
///     bodies" an ovstage `publish_outbound` wants).
///   - `writeData` — batched *scatter*: one value per object spread across many
///     objects under a single attribute (e.g. velocities).
///   - `writeArray` — a whole array-valued attribute on *one* object (e.g. the
///     point/orientation arrays of a particle system, deformable, or instancer).
///
/// All writes happen inside a `beginWrite()`/`endWrite()` pair, which gives a
/// USD backend its `SdfChangeBlock` (+ edit-target) batching window and an
/// ovstage backend a publish boundary.
class IPhysicsDataWrite
{
public:
    virtual ~IPhysicsDataWrite() = default;

    /// @name Batch boundary
    /// @{

    /// @brief Open a write batch. A USD backend opens an `SdfChangeBlock`
    ///        (and selects its edit-target layer) here. Calls do not nest.
    virtual void beginWrite() = 0;

    /// @brief Close the write batch opened by `beginWrite()`, flushing any
    ///        accumulated changes to the backend.
    virtual void endWrite() = 0;

    /// @}
    /// @name Per-object setup
    /// @{

    /// @brief Prepare a batch of objects to receive transform writes. Call once
    ///        per object before it is included in a `writeTransforms` batch.
    ///
    /// This is where a backend does whatever one-time work makes the per-frame
    /// transform path cheap and structurally stable. A USD backend normalizes
    /// each object's xform-op stack to scale/orient/translate and caches the
    /// per-object write state (including the residual offset of any ops that
    /// couldn't be normalized, which it folds into subsequent writes); an
    /// ovstage backend reserves the objects' rows in the output columns. The
    /// engine no longer authors the backing representation itself.
    ///
    /// @param keys        `count` object keys to prepare.
    /// @param count       Number of objects.
    /// @param outEligible Optional `count`-sized array; `outEligible[i]` is set
    ///                    true if `keys[i]` is ready to receive
    ///                    `writeTransforms`, false if it could not be set up
    ///                    (the caller should skip it for transform updates).
    ///                    May be null to ignore the result.
    virtual void prepareTransformWrite(const ObjectKey* keys, size_t count, bool* outEligible) = 0;

    /// @brief Drop any per-object state cached by `prepareTransformWrite` for a
    ///        batch of keys. Safe to call for keys that were never prepared.
    virtual void releaseTransformWrite(const ObjectKey* keys, size_t count) = 0;

    /// @}
    /// @name Vectorized output
    /// @{

    /// @brief Publish transforms for a batch of objects in one call.
    /// @param keys         `count` object keys; `keys[i]` gets the i-th TRS.
    ///                     Invalid keys are skipped.
    /// @param count        Number of objects in the batch.
    /// @param positions    WORLD-space translations, vec3 layout (`count` elements).
    /// @param orientations WORLD-space orientations, quaternion `xyzw` layout
    ///                     (`count` elements).
    /// @param scales       WORLD-space scales, vec3 layout (`count` elements);
    ///                     pass a view with `data == nullptr` to leave scale
    ///                     untouched.
    ///
    /// Poses are WORLD-space — the form physics produces natively and the
    /// source-agnostic thing the engine should emit. The backend performs any
    /// world->local conversion itself (a USD backend resolves each object's
    /// parent frame and authors local xform ops; an ovstage backend copies the
    /// columns). The engine therefore needs no knowledge of the backing
    /// scene-graph hierarchy.
    ///
    /// Transforms are passed as decomposed translation/rotation/scale rather
    /// than a packed 4x4 matrix on purpose: it is the non-lossy form, it
    /// matches the engine's native pose and the columnar `[N,3]/[N,4]/[N,3]`
    /// layout an ovstage/tensor backend wants, and it lets a backend author
    /// only the components it tracks. Each column is a `DataWriteView`, so a
    /// device-resident (GPU) column is expressible; a host-only backend may
    /// reject `device >= 0`. Must be called between `beginWrite()`/`endWrite()`.
    virtual void writeTransforms(const ObjectKey* keys,
                                 size_t count,
                                 const DataWriteView& positions,
                                 const DataWriteView& orientations,
                                 const DataWriteView& scales) = 0;

    /// @brief Scatter one value per object across a batch: write element `i` of
    ///        `data` to attribute `attr` on `keys[i]`.
    ///
    /// The batched per-object analogue of writing a single attribute — e.g.
    /// linear/angular velocities across all rigid bodies (one call per channel).
    /// The element shape comes from the destination attribute; the precision
    /// (and, in future, device) from the view. Invalid keys are skipped. Must
    /// be called between `beginWrite()`/`endWrite()`.
    ///
    /// QUATERNION LAYOUT: quaternion source elements are in `PxQuat` order
    /// (`xyzw`, real-last) — the SAME layout `writeTransforms` takes — at the
    /// view precision. The backend reorders to the backing store's layout (the
    /// USD backend to Gf real-first `wxyz`) and converts to the destination
    /// precision. One uniform quaternion convention across the whole interface.
    /// @param keys  `count` object keys.
    /// @param count Number of objects (must equal `data.count`).
    /// @param attr  Interned attribute-name token (minted via `IPhysicsSource`).
    /// @param data  Columnar view holding one value per key.
    virtual void writeData(const ObjectKey* keys, size_t count, TokenId attr, const DataWriteView& data) = 0;

    /// @}
    /// @name Per-object array output
    /// @{

    /// @brief Write a whole array-valued attribute on one object (e.g. point
    ///        positions/orientations for particles, deformables, instancers).
    ///
    /// QUATERNION LAYOUT: as for `writeData`, a quaternion array's source
    /// elements are in `PxQuat` order (`xyzw`, real-last) — the same convention
    /// `writeTransforms` uses — and the backend reorders to the backing store's
    /// layout. One uniform quaternion convention across the whole interface.
    ///
    /// INTEGER CONNECTIVITY: array attributes whose destination shape is an
    /// integer vector (e.g. a `UsdGeomTetMesh`'s `tetVertexIndices` int4 or
    /// `surfaceFaceVertexIndices` int3) are written by passing the contiguous
    /// `int32` elements (`GfVec4i` / `GfVec3i` layout) in `data` with
    /// `type == e32Bit`. The element *shape* (and int-vs-float kind) is taken
    /// from the destination attribute. This is how the cooking write-back
    /// persists generated mesh topology.
    ///
    /// EMPTY (RESET) WRITE: pass a view with `count == 0` (a null `data` is
    /// permitted in this case only) to author an empty array — the "clear the
    /// attribute" reset the cooking write-back uses to drop stale simulation
    /// velocities.
    /// @param key  Target object.
    /// @param attr Interned attribute-name token (minted via `IPhysicsSource`).
    /// @param data Columnar view over the whole array payload.
    virtual void writeArray(ObjectKey key, TokenId attr, const DataWriteView& data) = 0;

    /// @}
};

} // namespace omni::physics::parse
