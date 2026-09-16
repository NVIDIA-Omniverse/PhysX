// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-WRITE-CORE-001
 * @covers AC-1 AC-2 AC-3 AC-8 AC-9 AC-10 AC-11
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
#include "Math.h"

#include <carb/Types.h>

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

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

/// @brief Outcome of `IPhysicsDataWrite::readWheelLocalScale`.
enum class WheelScaleReadResult : uint8_t
{
    eNoDestination, //!< `key` has no backing destination; caller should use its own fallback.
    eScaleMissing,  //!< A destination exists but no scale could be determined; outScale is {1,1,1}.
    eOk,            //!< outScale reflects a real read.
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

    /// @brief Open a coarser, whole-frame write batch spanning multiple
    ///        `beginWrite()`/`endWrite()` pairs, which nest safely inside it
    ///        (each opens/closes its own inner batching scope; notifications
    ///        only flush once the outermost scope closes). The per-frame
    ///        session-layer edit-context/change-block scaffolding a USD
    ///        backend wraps a full simulation-output flush in. No-op on a
    ///        backend without this concept.
    /// @param destinationOverride Opaque, backend-typed, NON-OWNING,
    ///        pointer-sized handle to an alternate write destination for
    ///        this frame (`nullptr` == no override, use the default
    ///        destination). The caller's own reference keeps the pointee
    ///        alive for the duration of this call; a backend that needs the
    ///        handle to outlive the call takes its own reference from it.
    virtual void beginFrameWrite(void* destinationOverride) = 0;

    /// @brief Close the batch opened by `beginFrameWrite()`.
    virtual void endFrameWrite() = 0;

    /// @}
    /// @name Scene setup
    /// @{

    /// @brief Author a placeholder physics-scene prim at `sceneKey`'s
    ///        destination, used when a stage carries no PhysicsScene of its
    ///        own. USD backend: `UsdPhysicsScene::Define` + `PhysxSceneAPI`
    ///        on the SESSION layer (never saved), tagged hidden/no-delete.
    ///        Returns false (no side effect) when there is no backing
    ///        destination to author into (no stage, or a backend without
    ///        this concept) -- the caller still creates the scene in the
    ///        simulation only and simply skips recording that a USD prim
    ///        exists to remove later.
    /// @param sceneKey Well-known object key for the placeholder scene.
    /// @return True if a prim was actually authored.
    virtual bool createDefaultPhysicsScene(ObjectKey sceneKey) = 0;

    /// @brief Remove a placeholder physics-scene prim previously authored by
    ///        `createDefaultPhysicsScene`. A no-op when `sceneKey` names no
    ///        such prim (never authored, already removed, or a backend
    ///        without this concept).
    /// @param sceneKey Object key returned by the matching `createDefaultPhysicsScene` call.
    virtual void removeDefaultPhysicsScene(ObjectKey sceneKey) = 0;

    /// @}
    /// @name Per-object setup
    /// @{

    /// @brief Set the authoring mode `prepareTransformWrite` normalizes an
    ///        object's xform-op stack into: when `true`, prepared prims are
    ///        left in XformCommonAPI-compatible form (translate/rotate/scale
    ///        pivot ops) and written via a common-API-style vector set instead
    ///        of a plain scale/orient/translate op triple. The caller pushes
    ///        the live setting here before preparing any object, so the sink
    ///        need not reach back into engine settings itself. A backend
    ///        without this authoring distinction (e.g. a columnar ovstage
    ///        sink, which has no xform-op concept at all) may treat this as a
    ///        no-op.
    /// @param value New mode.
    virtual void setUpdateToUsdUsingXformCommonAPI(bool value) = 0;

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
    /// @param destinationOverride Same opaque, backend-typed, non-owning,
    ///                    pointer-sized handle convention as
    ///                    `beginFrameWrite`'s parameter of the same name
    ///                    (`nullptr` == no override): a backend that authors
    ///                    new attributes/xform-op-stack state as part of this
    ///                    one-time normalization scopes those edits to this
    ///                    destination instead of its default one, so a
    ///                    transient simulation-output layer is used, not the
    ///                    user's own authoring layer.
    virtual void prepareTransformWrite(const ObjectKey* keys,
                                       size_t count,
                                       bool* outEligible,
                                       void* destinationOverride = nullptr) = 0;

    /// @brief Drop any per-object state cached by `prepareTransformWrite` for a
    ///        batch of keys. Safe to call for keys that were never prepared.
    virtual void releaseTransformWrite(const ObjectKey* keys, size_t count) = 0;

    /// @brief Snapshot `key`'s current xform-op stack, keyed internally by
    ///        `key`, for a later `restoreXformOpReset` -- genuine
    ///        USD-authoring-only functionality (no ovstage equivalent) that
    ///        undoes simulation's per-frame transform writes when a
    ///        simulation run stops. No-op on a backend with no xform-op
    ///        concept or when `key` has no backing destination.
    /// @param key Target object.
    virtual void storeXformOpReset(ObjectKey key) = 0;

    /// @brief Restore the xform-op stack `storeXformOpReset` snapshotted for
    ///        `key`, then drop the stored snapshot (store/restore is a single-
    ///        use pair). No-op when `key` has no stored snapshot.
    /// @param key                              Target object.
    /// @param purgeOrphanedTranslateOrientScale Also remove sanitized
    ///        translate/orient/scale xform-op attributes that the snapshot
    ///        never wrote.
    virtual void restoreXformOpReset(ObjectKey key, bool purgeOrphanedTranslateOrientScale) = 0;

    /// @brief Normalize `key`'s xform-op stack to scale/orient/translate --
    ///        the same one-time normalization `prepareTransformWrite` performs
    ///        -- and read back the resulting local scale synchronously. Unlike
    ///        `prepareTransformWrite`, which readies an object for a later
    ///        `writeTransforms` batch, this is for a caller that needs the
    ///        scale value itself right now (a vehicle wheel/wheel-shape's
    ///        initial scale, folded into the vehicle's own simulation setup).
    ///        A backend with no normalizable xform-op concept may always
    ///        return `eNoDestination`.
    /// @param key      Target object.
    /// @param outScale Set per `WheelScaleReadResult`.
    virtual WheelScaleReadResult readWheelLocalScale(ObjectKey key, carb::Float3& outScale) = 0;

    /// @brief Nearest xformable ancestor of `key`'s prim, skipping non-xformable
    ///        ancestors -- the identity counterpart of the per-frame parent-frame
    ///        walk `writeTransforms` does internally, for a caller (vehicle
    ///        wheel-root parent-xform resolution) that needs the ancestor's
    ///        *identity*, resolved once at attach time, not every frame.
    ///        Returns an invalid key when `key` itself resets its own xform
    ///        stack, has no backing prim, or no xformable ancestor exists. A
    ///        backend with no ancestor-hierarchy concept (e.g. columnar ovstage)
    ///        always returns an invalid key.
    /// @param key Target object whose ancestor chain to walk.
    virtual ObjectKey resolveNearestXformableAncestor(ObjectKey key) = 0;

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

    /// @brief One-shot LOCAL-space pose write for a single object, from a full
    ///        4x4 matrix rather than a decomposed TRS.
    ///
    /// The attach/reset counterpart to `writeTransforms`: for callers handing
    /// over a product of two arbitrary affine transforms (e.g.
    /// `affineInverse(parentWorld) * world`), which can carry shear a TRS
    /// triple cannot represent, and which is already LOCAL (no parent-frame
    /// resolution needed, unlike `writeTransforms`). Used for one-shot/attach-
    /// time writes (e.g. a vehicle wheel's initial local pose), not the
    /// per-step batched output path.
    /// @param key         Target object.
    /// @param localMatrix Row-major 4x4 LOCAL-to-parent matrix (@ref Matrix4d).
    /// @param setScale    Whether to author the matrix's scale component too.
    virtual void writeLocalTransformMatrix(ObjectKey key, const Matrix4d& localMatrix, bool setScale) = 0;

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

    /// @brief If `attr` has no value resolvable at the destination's default/
    ///        rest state but does have an authored value at the source's
    ///        earliest sample, promote that sample to the default/rest state
    ///        (so a subsequent default-state read or write sees it). No-op
    ///        (returns true without side effects) when a default-state value
    ///        already resolves. Idempotent.
    ///
    /// Point-instancer transform arrays are commonly authored only as time
    /// samples (no default value); the per-step write-back reads/writes at
    /// the default/rest state, so this promotion is what makes a purely
    /// time-sampled instancer participate in write-back at all.
    /// @param key  Target object.
    /// @param attr Interned attribute-name token (minted via `IPhysicsSource`).
    /// @return False only when neither a default nor an earliest-sample value
    ///         resolves at all.
    virtual bool promoteEarliestSampleToDefault(ObjectKey key, TokenId attr) = 0;

    /// @}
    /// @name Ad hoc attribute output
    /// @{

    /// @brief Create-or-set a byte-array attribute on one object, keyed by a
    ///        plain attribute name rather than an interned `TokenId` -- these
    ///        are ad hoc cache markers outside `IPhysicsSource`'s standard
    ///        token vocabulary (e.g. a mesh-cook input CRC). USD backend:
    ///        creates the attribute as a UCharArray if absent.
    /// @param key      Target object.
    /// @param attrName Plain attribute name (not interned).
    /// @param data     Byte payload; may be null only when count == 0.
    /// @param count    Number of bytes.
    virtual void writeByteArrayAttribute(ObjectKey key, std::string_view attrName, const uint8_t* data, size_t count) = 0;

    /// @brief Create-or-set a uint32 scalar attribute on one object, keyed by a
    ///        plain attribute name rather than an interned `TokenId` -- an ad hoc
    ///        marker outside `IPhysicsSource`'s standard token vocabulary (e.g. a
    ///        mesh-cook format stamp). USD backend: creates the attribute as a
    ///        UInt if absent, matching `writeByteArrayAttribute`'s create-or-set
    ///        semantics (unlike `writeIntAttribute`, which only sets an existing
    ///        schema-declared attribute and no-ops otherwise).
    /// @param key      Target object.
    /// @param attrName Plain attribute name (not interned).
    /// @param value    Value to author.
    virtual void writeUIntAttribute(ObjectKey key, std::string_view attrName, uint32_t value) = 0;

    /// @brief Remove an authored ad hoc attribute (plain name, not interned) from
    ///        one object, if present. USD backend: `UsdPrim::RemoveProperty`.
    ///        This is a cleanup side effect, not a read, so it is simply a no-op
    ///        on a source with nothing to remove. Distinct from
    ///        `removeAppliedAPI`, which drops a whole applied-schema instance
    ///        rather than a single attribute.
    /// @param key      Target object.
    /// @param attrName Plain attribute name (not interned).
    virtual void removeAttribute(ObjectKey key, std::string_view attrName) = 0;

    /// @brief Publish a VtxTet deformable attachment (barycentric attachment
    ///        of src0 vertices to src1 tets) computed by the auto-attachment
    ///        pipeline. USD backend: authors the OmniPhysics deformable-
    ///        attachment schema on the object's prim.
    virtual void writeVtxTetAttachment(ObjectKey key,
                                       const std::vector<int32_t>& vtxIndicesSrc0,
                                       const std::vector<int32_t>& tetIndicesSrc1,
                                       const std::vector<carb::Float3>& tetCoordsSrc1,
                                       bool enabled) = 0;

    /// @brief Publish a VtxXform deformable attachment (src0 vertices pinned
    ///        to local positions on an xformable) computed by the
    ///        auto-attachment pipeline. Same backend semantics as
    ///        writeVtxTetAttachment.
    virtual void writeVtxXformAttachment(ObjectKey key,
                                         const std::vector<int32_t>& vtxIndicesSrc0,
                                         const std::vector<carb::Float3>& localPositionsSrc1,
                                         bool enabled) = 0;

    /// @brief Publish a per-side element-group collision filter computed by
    ///        the auto-attachment pipeline. Same backend semantics as
    ///        writeVtxTetAttachment.
    virtual void writeElementCollisionFilter(ObjectKey key,
                                             const std::vector<uint32_t>& groupElemCounts0,
                                             const std::vector<uint32_t>& groupElemIndices0,
                                             const std::vector<uint32_t>& groupElemCounts1,
                                             const std::vector<uint32_t>& groupElemIndices1,
                                             bool enabled) = 0;

    /// @brief Remove an applied API schema instance from an object, if a backing
    ///        destination exists. USD backend: `UsdPrim::RemoveAPI`. This is a
    ///        cleanup/downgrade side effect (e.g. dropping a schema PhysX does
    ///        not support outside an articulation), not a read, so it is simply
    ///        a no-op on a source with nothing to author into.
    /// @param key          Target object.
    /// @param apiName      Applied API schema identifier (e.g. "PhysxJointAxisAPI").
    /// @param instanceName Multi-apply instance name (e.g. "angular"); empty for
    ///                     a single-apply schema.
    virtual void removeAppliedAPI(ObjectKey key, std::string_view apiName, std::string_view instanceName) = 0;

    /// @brief Set an existing scalar float attribute by plain name on one
    ///        object, if a backing destination exists. USD backend:
    ///        `UsdAttribute::Set`. Unlike `writeData`, this does not go through
    ///        the interned-token vocabulary -- for ad hoc restore/reset writes
    ///        (e.g. vehicle controller input) outside `IPhysicsSource`'s
    ///        standard tokens. No-op when the object or attribute is absent.
    /// @param key      Target object.
    /// @param attrName Plain attribute name (not interned).
    /// @param value    Value to author.
    virtual void writeFloatAttribute(ObjectKey key, std::string_view attrName, float value) = 0;

    /// @brief Set an existing scalar int attribute by plain name on one
    ///        object. Same semantics as `writeFloatAttribute`.
    virtual void writeIntAttribute(ObjectKey key, std::string_view attrName, int32_t value) = 0;

    /// @brief Create-or-set a bool attribute by plain name on one object, if a
    ///        backing destination exists. USD backend: creates the attribute
    ///        as a Bool if absent (matching `writeUIntAttribute`'s
    ///        create-or-set semantics, unlike `writeFloatAttribute`/
    ///        `writeIntAttribute`, which only set an existing attribute). Used
    ///        for ad hoc cosmetic markers outside `IPhysicsSource`'s standard
    ///        token vocabulary (e.g. a Hydra "skip while rewriting" hint).
    /// @param key      Target object.
    /// @param attrName Plain attribute name (not interned).
    /// @param value    Value to author.
    virtual void writeBoolAttribute(ObjectKey key, std::string_view attrName, bool value) = 0;

    /// @brief Set an object's visibility (USD backend: UsdGeomImageable
    ///        Inherited/Invisible). No-op on a backend with no imageable-
    ///        visibility concept.
    /// @param key     Target object.
    /// @param visible Visible when true, invisible when false.
    virtual void writeVisibility(ObjectKey key, bool visible) = 0;

    /// @brief Set the radius of a point instancer's rendered prototype sphere
    ///        (particle-sampling instancer path, sized from the sampled point
    ///        width). No-op on a backend with no such concept.
    /// @param instancerKey Target point-instancer object.
    /// @param radius       Prototype sphere radius.
    virtual void writeInstancerProtoRadius(ObjectKey instancerKey, float radius) = 0;

    /// @}
    /// @name Relationship output
    /// @{

    /// @brief Check whether a batch of objects can receive `writeTriggerCollisions`
    ///        (the trigger-state destination exists for that key). Call once per
    ///        object before it is tracked for trigger-collision output.
    /// @param keys        `count` object keys to check.
    /// @param count       Number of objects.
    /// @param outEligible `count`-sized array; `outEligible[i]` is set true if
    ///                    `keys[i]` is ready to receive `writeTriggerCollisions`.
    ///                    May be null to ignore the result.
    virtual void prepareTriggerWrite(const ObjectKey* keys, size_t count, bool* outEligible) = 0;

    /// @brief Publish the set of objects a trigger is currently colliding with
    ///        (USD backend: `PhysxTriggerStateAPI`'s `triggeredCollisions`
    ///        relationship). This is a relationship, not a columnar attribute, so
    ///        it does not fit `writeData`/`writeArray`'s DataWriteView shape.
    ///
    ///        `count == 0` authors an empty (not absent) target list -- the
    ///        trigger itself is unaffected; use `releaseTriggerWrite` to drop the
    ///        write-back relationship entirely when the trigger goes away.
    /// @param trigger Object receiving the write.
    /// @param targets `count` object keys currently colliding with `trigger`.
    /// @param count   Number of colliding objects.
    virtual void writeTriggerCollisions(ObjectKey trigger, const ObjectKey* targets, size_t count) = 0;

    /// @brief Drop any authored trigger-collision relationship for a batch of
    ///        keys (removes the opinion, unlike `writeTriggerCollisions(key, ..,
    ///        0)` which authors an empty list). Call when a trigger is released.
    ///        Safe to call for keys that were never prepared.
    virtual void releaseTriggerWrite(const ObjectKey* keys, size_t count) = 0;

    /// @}
    /// @name Particle post-process output (Hydra-only visualization)
    /// @{

    /// @brief Remove any authored opinion for a whole array-valued attribute
    ///        on one object, reverting it to its schema fallback. Unlike
    ///        `writeArray(key, attr, {count=0})` (which authors a real,
    ///        empty-array opinion), this drops the opinion entirely, so a
    ///        later "is this attribute authored" check answers false again --
    ///        the semantics the particle post-process pipeline's simulation-
    ///        point backup/restore round-trip depends on.
    /// @param key  Target object.
    /// @param attr Interned attribute-name token (minted via `IPhysicsSource`).
    virtual void clearArray(ObjectKey key, TokenId attr) = 0;

    /// @brief Toggle the USD `purpose=proxy` visibility hint used to hide an
    ///        object's native render when the particle post-process pipeline
    ///        substitutes an isosurface mesh. No-op on backends without this
    ///        Hydra-only concept.
    /// @param key   Object whose purpose hint to toggle.
    /// @param proxy true authors purpose=proxy; false clears the authored
    ///              opinion (restores the schema default).
    virtual void writeProxyPurpose(ObjectKey key, bool proxy) = 0;

    /// @brief Declare the anisotropyQ1/Q2/Q3 point-array primvars on a particle
    ///        object, consumed by the anisotropy Hydra render delegate. USD
    ///        backend: UsdGeomPrimvarsAPI Float4Array primvars (vertex
    ///        interpolation) on the object's UsdGeomPoints prim. No-op on
    ///        backends without this Hydra-viewport-rendering-only concept
    ///        (or when `key` has no backing destination).
    /// @param key Target particle-set object.
    virtual void defineAnisotropyPrimvars(ObjectKey key) = 0;

    /// @brief Define (`enabled == true`) or remove (`enabled == false`) a
    ///        session-layer-only, non-deletable, outliner-hidden "Isosurface"
    ///        mesh child of `particleSystemKey` -- the particle post-process
    ///        pipeline's scratch render geometry. Session-layer: never saved,
    ///        gone on stage close. No-op if there is no backing stage.
    virtual void setIsosurfaceMeshEnabled(ObjectKey particleSystemKey, bool enabled) = 0;

    /// @brief Publish one frame of isosurface mesh geometry to the session-
    ///        layer mesh `setIsosurfaceMeshEnabled` defined for
    ///        `particleSystemKey`. No-op if that mesh does not currently
    ///        exist (not yet enabled, or no backing stage).
    /// @param particleSystemKey Owning particle system.
    /// @param points/numPoints             Vertex positions.
    /// @param normals/numNormals           Per-vertex normals (`numNormals == numPoints`).
    /// @param faceVertexCounts/numFaces     Per-face vertex counts.
    /// @param faceVertexIndices/numIndices  Flat face vertex indices.
    virtual void writeIsosurfaceMesh(ObjectKey particleSystemKey,
                                     const carb::Float3* points,
                                     size_t numPoints,
                                     const carb::Float3* normals,
                                     size_t numNormals,
                                     const int32_t* faceVertexCounts,
                                     size_t numFaces,
                                     const int32_t* faceVertexIndices,
                                     size_t numIndices) = 0;

    /// @brief Define (`enabled == true`) or remove (`enabled == false`) the
    ///        session-layer-only, non-deletable, outliner-hidden
    ///        "DiffuseParticles" Points child prim of `particleSystemKey`
    ///        that `writeDiffuseParticlePoints` publishes into -- the
    ///        diffuse-particle (foam/spray) render-point sink -- and wire it
    ///        to a sibling FlowEmitterPoint's `pointsPrim` relationship when
    ///        one exists (searched among the particle system's own children
    ///        first, then its parent's). Session-layer: never saved, gone on
    ///        stage close. Mirrors `setIsosurfaceMeshEnabled`. No-op if there
    ///        is no backing stage.
    virtual void setDiffuseParticleRenderingEnabled(ObjectKey particleSystemKey, bool enabled) = 0;

    /// @brief Publish one frame of diffuse-particle (foam/spray) render points
    ///        to the "DiffuseParticles" render-only child prim of
    ///        `particleSystemKey` that `setDiffuseParticleRenderingEnabled`
    ///        defines. No-op if that prim does not currently exist (not yet
    ///        enabled, or no backing stage).
    ///
    ///        `numPoints == 0` is a valid, expected steady-state call (author
    ///        an empty points array when no diffuse particles are currently
    ///        active), not an error.
    /// @param particleSystemKey Owning particle system.
    /// @param points/numPoints  Render-point positions.
    /// @param colors/numColors  Per-point display colors (`numColors` is
    ///                          normally `numPoints`).
    virtual void writeDiffuseParticlePoints(ObjectKey particleSystemKey,
                                            const carb::Float3* points,
                                            size_t numPoints,
                                            const carb::Float3* colors,
                                            size_t numColors) = 0;

    /// @}
};

} // namespace omni::physics::parse
