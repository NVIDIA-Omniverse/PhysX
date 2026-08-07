// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
/**
 * @implements REQ-PARSE-CORE-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-CORE-003
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-SHAPE-001
 * @covers AC-1
 */

#pragma once

#include "Handles.h"
#include "IChangeFeed.h"
#include "Math.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <carb/Types.h>

namespace omni::physics::parse
{

enum class UpAxis
{
    eY,
    eZ
};

struct SourceUnits
{
    float metersPerUnit = 1.0f;
    float kilogramsPerUnit = 1.0f;
    UpAxis upAxis = UpAxis::eZ;
};

// ---------------------------------------------------------------------------
/// @brief Raw mesh geometry, the input the cooking service consumes.
///
/// The four BufferHandles point into source-owned storage; resolve via
/// IPhysicsSource::resolveBuffer or ParseContext::getBuffer<T>.
///
///   - points     : per-vertex position payload, Vec3 (3 packed floats per vertex).
///   - indices    : per-face-vertex index payload, Int32.
///   - faceCounts : vertices-per-face payload, Int32.
///   - holes      : per-face hole markers, Int32.
///
/// `doubleSided` is true when the mesh should render/collide from both
/// face orientations. `leftHanded` is true when the source declares
/// left-handed winding; right-handed is the common default.
// ---------------------------------------------------------------------------

struct MeshGeometry
{
    BufferHandle points;
    BufferHandle indices;
    BufferHandle faceCounts;
    BufferHandle holes;
    /// Per-face physics-material index (UInt16), derived from the mesh's
    /// material-bound `GeomSubset`s — the input the cooking service uses for
    /// multi-material triangle meshes. Invalid/empty when the mesh has no
    /// material subsets (single-material).
    BufferHandle faceMaterials;
    bool doubleSided = false;
    bool leftHanded = false;
};

// ---------------------------------------------------------------------------
// AttrValue — type-erased attribute value returned by IPhysicsSource.
// Covers the scalar and small-vector types parsers need.
// ---------------------------------------------------------------------------

struct AttrValue
{
    enum class Kind : uint8_t
    {
        eNone,
        eBool,
        eInt,
        eFloat,
        eDouble,
        eHalf,   //!< 16-bit half, widened into the float slot but kept distinct from eFloat
        eFloat2,
        eFloat3,
        eFloat4,
        eToken,
        eString,
        eBuffer,
    };

    AttrValue() : kind(Kind::eNone), i(0) {}
    ~AttrValue() = default;
    AttrValue(const AttrValue&) = default;
    AttrValue& operator=(const AttrValue&) = default;
    AttrValue(AttrValue&&) = default;
    AttrValue& operator=(AttrValue&&) = default;

    Kind kind = Kind::eNone;

    union
    {
        bool b;
        int64_t i;
        float f;
        double d;
        carb::Float2 f2;
        carb::Float3 f3;
        carb::Float4 f4;
        TokenId tok;
    };

    std::string str;
    BufferHandle buffer;

    bool valid() const { return kind != Kind::eNone; }

    static AttrValue makeBool(bool v) { AttrValue a; a.kind = Kind::eBool; a.b = v; return a; }
    static AttrValue makeInt(int64_t v) { AttrValue a; a.kind = Kind::eInt; a.i = v; return a; }
    static AttrValue makeFloat(float v) { AttrValue a; a.kind = Kind::eFloat; a.f = v; return a; }
    static AttrValue makeDouble(double v) { AttrValue a; a.kind = Kind::eDouble; a.d = v; return a; }
    // Half value, widened into the float slot (half is a strict subset of float,
    // so this is lossless) but tagged eHalf so consumers can tell it apart.
    static AttrValue makeHalf(float v) { AttrValue a; a.kind = Kind::eHalf; a.f = v; return a; }
    static AttrValue makeFloat2(carb::Float2 v) { AttrValue a; a.kind = Kind::eFloat2; a.f2 = v; return a; }
    static AttrValue makeFloat3(carb::Float3 v) { AttrValue a; a.kind = Kind::eFloat3; a.f3 = v; return a; }
    static AttrValue makeFloat4(carb::Float4 v) { AttrValue a; a.kind = Kind::eFloat4; a.f4 = v; return a; }
    static AttrValue makeToken(TokenId v) { AttrValue a; a.kind = Kind::eToken; a.tok = v; return a; }
    static AttrValue makeString(std::string v) { AttrValue a; a.kind = Kind::eString; a.str = std::move(v); return a; }
    static AttrValue makeBuffer(BufferHandle v) { AttrValue a; a.kind = Kind::eBuffer; a.buffer = v; return a; }
};

// ---------------------------------------------------------------------------
// ReadTime — USD-neutral read time for time-sampled attribute reads.
// ---------------------------------------------------------------------------

/// @brief Time coordinate for a time-aware attribute read.
///
/// Mirrors USD's `UsdTimeCode`: either the default/fallback value
/// (`eDefault`) or a specific time-code / frame (`eAtTime`). The parse-lib
/// stays USD-neutral, so this carries a plain `double` time-code rather than a
/// `UsdTimeCode`. Sources without time sampling (`ProceduralSource`,
/// `MockSource`) ignore it; the USD backend maps it to a `UsdTimeCode`; a
/// future ovstage backend maps `eAtTime` onto a `simulation_time_ns`.
struct ReadTime
{
    enum class Mode : uint8_t
    {
        eDefault,
        eAtTime,
    };

    Mode   mode     = Mode::eDefault;
    double timeCode = 0.0;

    static ReadTime defaultTime() { return {}; }
    static ReadTime at(double t) { return { Mode::eAtTime, t }; }
};

/// @brief Which objects a scoped descendant query visits.
///
/// Expresses *intent*, not a USD predicate: backends map it to their own
/// notion of "present"/"instanced". For non-USD backends (no activation
/// or instancing) the two modes coincide.
enum class DescendantScope
{
    /// Every object in the subtree, with no instance-proxy descent — the
    /// broadest set. USD: `UsdPrimRange::AllPrims` (inactive/abstract
    /// included). Mirrors the native `scanStage` walk.
    eAll,
    /// Only "present" objects (USD: active + defined + loaded +
    /// non-abstract), descending **through instance proxies** so members
    /// of instanced prototypes are visited. USD:
    /// `UsdPrimRange(root, UsdTraverseInstanceProxies())`.
    eActiveInstanced,
    /// Only "present" objects (USD: active + defined + loaded +
    /// non-abstract), **without** instance-proxy descent — the default USD
    /// predicate. USD: `UsdPrimRange(root)`. Used by change-notice subtree
    /// walks that must mirror the editor's default traversal exactly.
    eActive,
};

/// @brief Abstract physics-data source consumed by the parsing library.
///
/// A Source represents a single backing scene description and exposes it
/// as a hierarchy of objects identified by `ObjectKey`s, each carrying
/// attributes, schema applications, and a local-to-world transform.
/// The parsing layer is source-neutral: it speaks only this interface
/// and never sees the underlying scene representation directly.
class IPhysicsSource
{
public:
    virtual ~IPhysicsSource() = default;

    /// @name Identity & display
    /// @{

    /// @brief Human-readable display string for `key` (for logs, errors).
    /// @param key Object key to format. The invalid sentinel maps to an
    ///        empty view.
    /// @return Backend-supplied display string. Backed by source-owned
    ///         storage and valid for the Source's lifetime — callers may
    ///         hold the view across subsequent Source calls.
    virtual std::string_view sourceKeyToString(ObjectKey key) const = 0;

    /// @brief Intern a token string and return its stable id.
    /// @param token String to intern. Equal strings always produce equal
    ///        `TokenId`s on the same Source instance.
    /// @return Stable `TokenId` for `token`. Ids are valid for the
    ///         Source's lifetime; cross-Source comparison is undefined.
    virtual TokenId internToken(std::string_view token) const = 0;

    /// @brief Resolve a previously-interned token id back to its string.
    /// @param id Token id minted by `internToken` on this Source.
    /// @return Backing string view (source-owned). Empty view when `id`
    ///         is the invalid sentinel or was not minted by this Source.
    virtual std::string_view tokenToString(TokenId id) const = 0;

    /// @}

    /// @name Traversal
    /// @{

    /// @brief Synthetic root of the source's object hierarchy.
    /// @return The root `ObjectKey`. Always valid; backends supply a
    ///         synthetic root even when the underlying scene has multiple
    ///         top-level objects. Pass it to `forEachChild` to enumerate
    ///         the scene's top-level objects.
    virtual ObjectKey getRootKey() const = 0;

    /// @brief Invoke `cb` once per direct child of `parent`.
    /// @param parent Object whose direct children to enumerate. Pass
    ///        `getRootKey()` to walk the scene's top-level objects.
    /// @param cb Invoked once per child, in the source's declaration
    ///        order. Not invoked when `parent` has no children or is
    ///        the invalid sentinel.
    virtual void forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const = 0;

    /// @brief Scoped query: invoke `cb` for every object in the subtree rooted
    /// at `root` (root **inclusive**).
    ///
    /// This expresses *intent* ("everything under here"), not a traversal
    /// mechanism: the backend chooses how to satisfy it — USD walks a
    /// `UsdPrimRange`, a future ovstage backend issues a path-prefix
    /// `query_prims` (no range traversal). Consumers that want a subset filter
    /// inside `cb` (e.g. by looking the object up in their own database, or via
    /// `isA`/`hasSchema`).
    /// @param root Subtree root. Visited first, then its descendants.
    /// @param cb Invoked once per object; not invoked for the invalid sentinel.
    /// @note Default implementation is a recursive `forEachChild` walk
    ///       (sufficient for static sources). USD overrides it so the visited
    ///       set matches `UsdPrimRange::AllPrims` (includes inactive/abstract —
    ///       consumers filter), preserving the legacy walk semantics.
    virtual void forEachDescendant(ObjectKey root, std::function<void(ObjectKey)> cb) const
    {
        if (!cb || !root.valid())
            return;
        cb(root);
        forEachChild(root, [this, &cb](ObjectKey child) { forEachDescendant(child, cb); });
    }

    /// @brief Prune-aware scoped query: like `forEachDescendant`, but `visit`
    /// returns `true` to prune — skip the descendants of that object.
    /// (USD: `UsdPrimRange` + `PruneChildren`.)
    /// @param root Subtree root (visited first).
    /// @param visit Invoked per object; return `true` to skip its descendants.
    /// @param scope Which objects to visit; see @ref DescendantScope. The
    ///        default (`eAll`) preserves the native `scanStage` walk.
    /// @note The default implementation recurses via `forEachChild` and is
    ///       `scope`-agnostic — correct for backends without prim activation
    ///       or instancing. USD overrides it to honor `scope`.
    virtual void forEachDescendantPruned(ObjectKey root, std::function<bool(ObjectKey)> visit,
                                         DescendantScope scope = DescendantScope::eAll) const
    {
        if (!visit || !root.valid())
            return;
        if (visit(root))
            return;
        forEachChild(root, [this, &visit, scope](ObjectKey child) { forEachDescendantPruned(child, visit, scope); });
    }

    /// @brief Look up an object by source-native path string.
    /// @param path Source-native path. Format is backend-specific — USD
    ///        sources use SdfPath strings; mock/procedural sources may
    ///        use their own scheme.
    /// @return Matching `ObjectKey`; invalid sentinel when `path` does
    ///         not resolve.
    virtual ObjectKey findByPath(std::string_view path) const = 0;

    /// @brief Canonical (stable, identity-comparable) key for `key`.
    /// @details Most sources mint one handle per object, so this is the
    ///          identity (the default). Some backends (ovstage) can hand back
    ///          DIFFERENT handles for the same object via different lookups
    ///          (e.g. enumerate/get_paths vs intern_path); those override this to
    ///          a single canonical handle so cross-reference matching and graph
    ///          algorithms compare keys reliably. Default: returns `key`.
    virtual ObjectKey canonicalKey(ObjectKey key) const
    {
        return key;
    }

    /// @brief Parent of `key`.
    /// @param key Object whose parent to look up.
    /// @return Parent `ObjectKey`. Returns `getRootKey()` for top-level
    ///         objects, and the invalid sentinel for the root itself or
    ///         unresolvable keys.
    virtual ObjectKey getParent(ObjectKey key) const = 0;

    /// @}

    /// @name Schema queries
    ///
    /// A "schema" in parse-lib is a named, applied tag on an object — a
    /// bundle of attributes/relationships the source declares as applied
    /// to a specific `ObjectKey`. The concept maps directly to USD's
    /// API-schema concept (`UsdAPISchemaBase`); other backends present
    /// the same shape using their own naming.
    ///
    /// Single-apply schemas appear at most once per object and are
    /// queried with `hasSchema`. Multi-apply schemas appear with an
    /// instance suffix `<baseSchema>:<instance>` and are iterated via
    /// `forEachMultiApplyInstance`.
    /// @{

    /// @brief True iff the schema named by `schemaToken` is applied to `key`.
    /// @param key Object to inspect.
    /// @param schemaToken Token for the unqualified schema name (no
    ///        instance suffix). Use `internToken(schemaName)` to obtain.
    /// @return `true` when the schema is applied — single-apply schemas
    ///         match outright, multi-apply schemas match if at least one
    ///         instance is applied. For per-instance enumeration use
    ///         `forEachMultiApplyInstance`.
    virtual bool hasSchema(ObjectKey key, TokenId schemaToken) const = 0;

    /// @brief Polymorphic, subtype-aware "is-a" type check on `key`.
    ///
    /// Unlike `hasSchema` (which matches an *exact* prim-type name or an applied
    /// schema), this matches `key`'s prim type **and all of its subclasses** —
    /// the runtime-gating equivalent of USD's `prim.IsA<T>()` (e.g. an
    /// `isA(key, "PhysicsJoint")` matches `PhysicsRevoluteJoint`/`Prismatic`/…).
    /// `typeToken` is the registered schema type name (obtain it from a C++
    /// schema type via `UsdSchemaRegistry::GetSchemaTypeName` + `internToken`,
    /// so the name round-trips through the backend).
    /// @note Default returns `false` (no type hierarchy) — backends that model
    ///       a type hierarchy (USD) override it.
    virtual bool isA(ObjectKey key, TokenId typeToken) const
    {
        (void)key; (void)typeToken;
        return false;
    }

    /// @brief True iff `key` resolves to a live object in this source.
    ///
    /// Replaces a raw prim-validity guard (`prim != null`) in source-agnostic
    /// gating code.
    /// @note Default returns `false`; backends override.
    virtual bool exists(ObjectKey key) const
    {
        (void)key;
        return false;
    }

    /// @brief True iff `key` is an instancing prototype root (USD:
    /// `UsdPrim::IsPrototype()` — the synthetic `/__Prototype_*` root that
    /// backs instanceable prims).
    ///
    /// Change-notice handlers skip prototype roots: edits propagate via the
    /// instance proxies, not the synthetic prototype prim.
    /// @note Default returns `false` — backends without instancing have no
    ///       prototypes; USD overrides it.
    virtual bool isPrototype(ObjectKey key) const
    {
        (void)key;
        return false;
    }

    /// @brief True iff `key` is an instance proxy (USD: `UsdPrim::IsInstanceProxy()`
    /// — an object inside an instanced prototype, reached through an instanceable
    /// ancestor).
    /// @note Default returns `false` — backends without instancing have none; USD
    ///       overrides it.
    virtual bool isInstanceProxy(ObjectKey key) const
    {
        (void)key;
        return false;
    }

    /// @brief True iff `key` is an instanceable instance (USD: `UsdPrim::IsInstance()`
    /// — a prim with `instanceable=true` that is backed by a prototype).
    /// @note Default returns `false` — backends without instancing have none; USD
    ///       overrides it.
    virtual bool isInstance(ObjectKey key) const
    {
        (void)key;
        return false;
    }

    /// @brief The object's concrete type-name token (USD: `UsdPrim::GetTypeName()`),
    /// interned on this source.
    ///
    /// Unlike `isA` (schema-hierarchy membership), this returns the exact authored
    /// type name — needed for lookups keyed by the raw type string (e.g. the
    /// custom-joint type registry).
    /// @note Default returns the invalid token id; USD overrides it.
    virtual TokenId getTypeName(ObjectKey key) const
    {
        (void)key;
        return TokenId{};
    }

    /// @brief Iterate the instances of a multi-apply schema applied to `key`.
    /// @param key Object to inspect.
    /// @param baseSchema Unqualified schema name (no trailing separator).
    /// @param cb Invoked once per applied instance with the suffix that
    ///        distinguishes one application from another — i.e. for
    ///        `<baseSchema>:<instance>`, the `<instance>` part. Iteration
    ///        order matches the source's applied-schema order; callers
    ///        must not assume a specific order across sources. Not
    ///        invoked when no instances are applied.
    /// @implements REQ-PARSE-CORE-003
    /// @covers AC-1
    virtual void forEachMultiApplyInstance(
        ObjectKey key,
        std::string_view baseSchema,
        std::function<void(std::string_view instance)> cb) const = 0;

    /// @brief Invoke `cb` once per applied API schema name on `key`, in the
    /// source's applied-schema order. Multi-apply instances appear with their
    /// instance suffix (USD: the full `prim.GetAppliedSchemas()` list, e.g.
    /// `PhysxJointAxisAPI:linear`). Used by change-notice handlers that diff the
    /// current applied-schema set against a stored one.
    /// @note Default no-op; backends that model applied schemas (USD) override.
    virtual void forEachAppliedSchema(ObjectKey key, std::function<void(TokenId)> cb) const
    {
        (void)key; (void)cb;
    }

    /// @}

    /// @name Attribute access
    /// @{

    /// @brief Read `attr` on `key` as a dynamically-typed AttrValue.
    /// Resolves schema fallback defaults.
    /// @param key Object to query.
    /// @param attr Token for the attribute name.
    /// @return Resolved value. Returns an invalid `AttrValue`
    ///         (`kind == eNone`) when the attribute is not declared on
    ///         `key`. To distinguish an authored value from a resolved
    ///         schema default use `hasAuthoredAttribute`.
    /// @note Prefer the typed overloads below — they're type-safe at the
    ///       call site and (for backends that override) cheaper than
    ///       building an AttrValue. This AttrValue-returning overload
    ///       stays as the dynamic / introspection path (and as the
    ///       single virtual the typed overloads dispatch through by
    ///       default).
    virtual AttrValue getAttribute(ObjectKey key, TokenId attr) const = 0;

    /// @name Typed attribute reads
    ///
    /// Type-safe overloads of `getAttribute`. Each writes `out` and
    /// returns `true` on success — i.e. the attribute is declared on
    /// `key` AND its `AttrValue::Kind` matches the requested C++ type
    /// (including the documented permissive conversions). Returns
    /// `false` and leaves `out` untouched when the attribute is
    /// absent or its type mismatches.
    ///
    /// Permissive conversions (match existing parse-lib semantics):
    ///   - `float&` / `double&` accept `eFloat`, `eDouble`, or `eHalf`.
    ///   - `int64_t&` accepts either `eInt` or `eBool` (bool → 1/0).
    ///   - `bool&` accepts `eBool` only.
    ///   - The fixed-arity vector / scalar / handle types match exactly.
    ///
    /// Default implementations dispatch through the
    /// AttrValue-returning `getAttribute` above. Backends may override
    /// individual overloads for direct-type fast paths.
    ///
    /// Usage idiom — "read with default" without an explicit AttrValue
    /// intermediate:
    /// @code
    ///     float x = defaultVal;
    ///     src.getAttribute(key, attr, x); // leaves x as defaultVal on miss
    /// @endcode
    /// @{

    virtual bool getAttribute(ObjectKey key, TokenId attr, bool& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, int64_t& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, float& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, double& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, carb::Float2& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, carb::Float3& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, carb::Float4& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, TokenId& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, std::string& out) const;
    virtual bool getAttribute(ObjectKey key, TokenId attr, BufferHandle& out) const;

    /// @}

    /// @brief Time-aware read of `attr` on `key`.
    ///
    /// Same contract as `getAttribute(key, attr)` but evaluated at `time`.
    /// This is the read used by the runtime update path, where time-sampled
    /// attributes (animated drive targets, kinematic transforms, etc.) must be
    /// sampled at the current frame rather than the default.
    ///
    /// The default implementation ignores `time` and forwards to the
    /// time-agnostic `getAttribute` — correct for sources that hold no time
    /// samples (`ProceduralSource`, `MockSource`). The USD backend overrides it
    /// to evaluate at the requested `UsdTimeCode`.
    /// @param key  Object to query.
    /// @param attr Token for the attribute name.
    /// @param time Time coordinate to evaluate at (default ⇒ identical to
    ///        `getAttribute`).
    virtual AttrValue getAttributeAtTime(ObjectKey key, TokenId attr, ReadTime time) const
    {
        (void)time;
        return getAttribute(key, attr);
    }

    /// @brief True iff `attr` on `key` was explicitly authored, as opposed
    /// to resolving via a schema fallback default.
    /// @param key Object to query.
    /// @param attr Token for the attribute name.
    /// @return `true` when the attribute is authored on `key`. Callers
    ///         that gate logic on "user explicitly set this" — e.g.
    ///         `contactOffset`/`restOffset` where the schema default is
    ///         sentinel-valued — must use this rather than
    ///         `getAttribute(...).valid()`, because `getAttribute`
    ///         resolves schema fallbacks too.
    virtual bool hasAuthoredAttribute(ObjectKey key, TokenId attr) const = 0;

    /// @brief True iff `attr` on `key` is animated (carries multiple
    /// samples over time).
    /// @param key Object to query.
    /// @param attr Token for the attribute name.
    /// @return `true` when the attribute carries multiple time samples;
    ///         `false` for default-only or unauthored attrs. Used to
    ///         flag e.g. time-sampled transforms for change tracking.
    virtual bool isAttributeTimeSampled(ObjectKey key, TokenId attr) const = 0;

    /// @brief True iff `attr` on `key` might vary over time.
    /// @param key Object to query.
    /// @param attr Token for the attribute name.
    /// @return Looser counterpart to `isAttributeTimeSampled`: `true` when the
    ///         attribute could resolve to different values at different times
    ///         (any time sample, or affected by value clips), matching USD's
    ///         `UsdAttribute::ValueMightBeTimeVarying()`. Used to gate
    ///         change-tracking registration the way the legacy parse helpers do.
    /// @note Default returns `false`; backends that model time override it.
    virtual bool mightBeTimeVarying(ObjectKey key, TokenId attr) const
    {
        (void)key; (void)attr;
        return false;
    }

    /// @brief Targets of the named relationship on `key`, in declaration order.
    /// @param key Object owning the relationship.
    /// @param rel Token for the relationship name.
    /// @param out Filled with target `ObjectKey`s; cleared first. Empty
    ///        when the relationship has no targets or does not exist
    ///        on `key`.
    virtual void getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const = 0;

    /// @brief True iff the named relationship is defined on `key`.
    /// @param key Object to inspect.
    /// @param rel Token for the relationship name.
    /// @return Whether the relationship exists, independent of whether it has
    ///         any targets — `getRelationshipTargets` reports empty for both an
    ///         absent relationship and a defined-but-empty one, so callers that
    ///         must distinguish the two use this.
    /// @note Default returns `false` (sources that do not model relationship
    ///       existence); backends with relationships (USD) override.
    virtual bool hasRelationship(ObjectKey key, TokenId rel) const
    {
        (void)key; (void)rel;
        return false;
    }

    /// @brief Indices of point-instancer instances that are inactive (not
    /// simulated/expanded), in the order the backend stores them.
    /// @param key Point-instancer object.
    /// @param out Filled with the inactive instance indices; cleared first.
    /// @note Expresses intent ("which instances are off"), not a USD
    ///       mechanism — USD reads the `inactiveIds` list-op metadata; other
    ///       backends map their own activation state. Default: none inactive.
    virtual void getInactiveInstanceIds(ObjectKey key, std::vector<int64_t>& out) const
    {
        (void)key;
        out.clear();
    }

    /// @}

    /// @name Transform queries
    /// @{

    /// @brief Local-to-world transform for `key`.
    /// @param key Object to query.
    /// @param outMatrix Filled with a row-major 4x4 double matrix
    ///        (matches the layout of @ref Matrix4d). Receives identity
    ///        when `key` is the invalid sentinel.
    virtual void getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const = 0;

    /// @brief Local-to-world transform for `key`, evaluated at `time`.
    /// @param key Object to query.
    /// @param time Sample time; `ReadTime::defaultTime()` reads the
    ///        default/static value (as opposed to the time-independent
    ///        overload above, which a backend may pin to a fixed sample).
    /// @param outMatrix Filled with a row-major 4x4 double matrix
    ///        (@ref Matrix4d). Receives identity when `key` is the invalid
    ///        sentinel.
    /// @note The default implementation ignores `time` and delegates to the
    ///       time-independent overload, so static sources need not override.
    virtual void getLocalToWorldTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix) const
    {
        (void)time;
        getLocalToWorldTransform(key, outMatrix);
    }

    /// @brief Local-to-world rotation + per-axis scale, obtained via
    /// affine polar-style decomposition.
    /// @param key Object to query.
    /// @param outRotation Filled with the rotation component as a
    ///        row-major 3x3 double matrix (@ref Matrix3d).
    /// @param outScale Filled with per-axis scale.
    /// @note Sources that cannot perform a proper decomposition (e.g.
    ///       sheared or otherwise non-orthogonal transforms) return
    ///       identity rotation and unit scale.
    virtual void getLocalToWorldRotationAndScale(ObjectKey key,
                                                 Matrix3d& outRotation,
                                                 carb::Float3& outScale) const = 0;

    /// @brief Object-local transform — the transform contributed by `key`'s
    /// own ops, before the parent frame is applied (the runtime equivalent of
    /// `UsdGeomXformable::GetLocalTransformation`).
    /// @param key Object to query.
    /// @param time Sample time; `ReadTime::defaultTime()` reads the
    ///        default/static value.
    /// @param outMatrix Filled with a row-major 4x4 double matrix
    ///        (@ref Matrix4d); identity when `key` carries no local transform
    ///        or is the invalid sentinel.
    /// @param outResetsXformStack Set `true` iff the object resets the
    ///        inherited parent transform (USD `resetXformStack`), else `false`.
    /// @note Default returns identity + `false`; backends that model a local
    ///       transform (USD) override it.
    virtual void getLocalTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix,
                                   bool& outResetsXformStack) const
    {
        (void)key;
        (void)time;
        outMatrix = Matrix4d{};
        outResetsXformStack = false;
    }

    /// @brief True iff `key`'s local-to-world transform might vary over time —
    /// i.e. a transform op on `key` or any of its ancestors carries time samples.
    ///
    /// The world transform is the composition of `key`'s own transform ops and
    /// those of every ancestor, so an animated op anywhere up the chain makes the
    /// world transform time-varying. Used to flag e.g. animated kinematic bodies.
    /// @param key Object whose world transform to inspect.
    /// @note Default returns `false`; backends that model a transform hierarchy
    ///       with time samples (USD) override it by walking the ancestor chain.
    virtual bool mightWorldTransformBeTimeVarying(ObjectKey key) const
    {
        (void)key;
        return false;
    }

    /// @}

    /// @name Bulk buffers (zero-copy)
    /// @{

    /// @brief Resolve a previously-registered buffer handle to its raw bytes.
    /// @param handle Buffer handle to resolve.
    /// @param byteCount Filled with the buffer size in bytes (zero when
    ///        the handle is invalid).
    /// @return Pointer to backing bytes (source-owned); `nullptr` when
    ///         `handle` is the invalid sentinel or was not minted by
    ///         this Source. The pointer remains valid until the Source
    ///         is destroyed or the buffer is explicitly invalidated.
    virtual const void* resolveBuffer(BufferHandle handle, size_t& byteCount) const = 0;

    /// @brief Resolve a mesh-typed object's geometry into buffer handles
    /// + flags: points, indices, face counts, hole indices, plus
    /// `doubleSided` / `leftHanded`.
    /// @param key Mesh-typed object key.
    /// @return A populated `MeshGeometry`. All-invalid handles when
    ///         `key` is not mesh-typed or carries no geometry.
    virtual MeshGeometry getMeshAttributes(ObjectKey key) const = 0;

    /// @brief Read an array-valued attribute as a buffer, evaluated at `time`.
    ///
    /// Distinct from `getAttribute`, which collapses an array to a scalar (its
    /// first element) per the parse-lib convention. This returns the whole
    /// array as a `BufferHandle` (resolve via `resolveBuffer`; element type and
    /// count are on the handle).
    /// @return An invalid handle when the attribute is absent, empty, or not an
    ///         array of a supported element type. The buffer is owned by the
    ///         source until `releaseBuffer` or source destruction.
    /// @note Default returns an invalid handle — correct for sources with no
    ///       array storage (`ProceduralSource`, `MockSource`).
    virtual BufferHandle getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const
    {
        (void)key; (void)attr; (void)time;
        return {};
    }

    /// @brief Release a single buffer previously minted by this source
    /// (`getMeshAttributes` / `getArrayAttribute`). Lets per-call runtime array
    /// reads avoid unbounded accumulation (parse-time mesh buffers are instead
    /// dropped in bulk at run boundaries). No-op by default.
    virtual void releaseBuffer(BufferHandle handle) const { (void)handle; }

    /// @}

    /// @name Source-wide queries
    /// @{

    /// @brief metersPerUnit + up-axis for this Source.
    /// @return The Source-wide `SourceUnits`. These are properties of
    ///         the Source itself, not of any particular physics scene
    ///         — every scene parsed from this Source shares the same
    ///         values.
    virtual SourceUnits getSourceUnits() const = 0;

    /// @}

    /// @name Collections & bindings
    /// @{

    /// @brief Pre-resolve the named collection on `primKey` into its
    /// included member objects.
    /// @param primKey Object owning the collection.
    /// @param collectionName Token for the collection's instance name.
    ///        The Source resolves include/exclude relationships and any
    ///        child-traversal semantics internally — callers receive a
    ///        flat list of resolved members.
    /// @param members Filled with included members; cleared first. Empty
    ///        when the collection does not exist or is empty.
    /// @implements REQ-PARSE-COLGROUP-002
    /// @covers AC-1
    virtual void resolveCollection(ObjectKey primKey,
                                   TokenId collectionName,
                                   std::vector<ObjectKey>& members) const = 0;

    /// @brief Resolve the physics-purpose bound material on `primKey`.
    /// @param primKey Object to look up.
    /// @return Bound material's `ObjectKey`; invalid sentinel when no
    ///         material is bound under the `physics` purpose.
    virtual ObjectKey getMaterialBinding(ObjectKey primKey) const = 0;

    /// @}

    /// @name Change tracking
    /// @{

    /// @brief Create a change feed for this source's live deltas (ADR-0003),
    /// or return `nullptr` for static sources that never change
    /// (`ProceduralSource`, tests) — the runtime dispatcher then no-ops.
    /// @return An owned `IChangeFeed`, or `nullptr` when the source is static.
    virtual std::unique_ptr<IChangeFeed> createChangeFeed() { return nullptr; }

    /// @}
};

// ---------------------------------------------------------------------------
// Default implementations of IPhysicsSource's typed getAttribute overloads.
// Each unpacks an AttrValue produced by the AttrValue-returning virtual.
// Backends may override individual overloads to skip the AttrValue
// construction entirely.
// ---------------------------------------------------------------------------

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, bool& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eBool) { out = v.b; return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, int64_t& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eInt)  { out = v.i; return true; }
    if (v.kind == AttrValue::Kind::eBool) { out = v.b ? 1 : 0; return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, float& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eFloat || v.kind == AttrValue::Kind::eHalf) { out = v.f; return true; }
    if (v.kind == AttrValue::Kind::eDouble) { out = static_cast<float>(v.d); return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, double& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eDouble) { out = v.d; return true; }
    if (v.kind == AttrValue::Kind::eFloat || v.kind == AttrValue::Kind::eHalf) { out = static_cast<double>(v.f); return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, carb::Float2& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eFloat2) { out = v.f2; return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, carb::Float3& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eFloat3) { out = v.f3; return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, carb::Float4& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eFloat4) { out = v.f4; return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, TokenId& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eToken) { out = v.tok; return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, std::string& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eString) { out = std::move(v.str); return true; }
    return false;
}

inline bool IPhysicsSource::getAttribute(ObjectKey key, TokenId attr, BufferHandle& out) const
{
    AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eBuffer) { out = v.buffer; return true; }
    return false;
}

} // namespace omni::physics::parse
