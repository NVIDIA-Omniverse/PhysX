// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

/**
 * @implements REQ-PARSE-CORE-001
 * @implements REQ-PARSE-CORE-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * USD-backed `IPhysicsSource` implementation: mints `ObjectKey` /
 * `TokenId` / `BufferHandle` for stage paths, tokens, mesh arrays;
 * resolves attribute reads / schema checks / collection membership
 * via the PXR USD APIs. Path↔key round-tripping (`pathFor` / `keyFor`)
 * is the foundation for the cross-`ScannedStage` re-key contract
 * described in ADR-0002.
 */

#include "UsdSource.h"

#include "NativeWalker.h"
#include "UsdChangeFeed.h"

#include <pxr/usd/usd/schemaRegistry.h>

#include <unordered_set>

#include <pxr/base/gf/half.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/quath.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/transform.h>
#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/collectionAPI.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdPhysics/metrics.h> // UsdPhysicsGetStageKilogramsPerUnit
#include <pxr/usd/usdGeom/subset.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdPhysics/materialAPI.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#include <pxr/usd/sdf/listOp.h>
#include <pxr/usd/sdf/schema.h>

#include <algorithm>
#include <set>

namespace omni::physics::usd
{
using namespace omni::physics::parse;

UsdSource::UsdSource(PXR_NS::UsdStageWeakPtr stage)
    : mStage(std::move(stage))
{
    // Reserve slot 0 — handle 0 is the invalid sentinel
    mKeyToPath.emplace_back();
    mKeyStrings.emplace_back();
    mIdToToken.emplace_back();
}

UsdSource::~UsdSource() = default;

// ---------------------------------------------------------------------------
// ObjectKey <-> SdfPath intern table
// ---------------------------------------------------------------------------

ObjectKey UsdSource::keyFor(const PXR_NS::SdfPath& path) const
{
    if (path.IsEmpty())
        return {};

    auto [it, inserted] = mPathToKey.try_emplace(path, ObjectKey{});
    if (inserted)
    {
        it->second = ObjectKey{ static_cast<uint64_t>(mKeyToPath.size()) };
        mKeyToPath.push_back(path);
        mKeyStrings.push_back(path.GetString());
    }
    return it->second;
}

PXR_NS::SdfPath UsdSource::pathFor(ObjectKey key) const
{
    if (key.handle == 0 || key.handle >= mKeyToPath.size())
        return {};
    return mKeyToPath[key.handle];
}

std::string_view UsdSource::sourceKeyToString(ObjectKey key) const
{
    if (key.handle == 0 || key.handle >= mKeyStrings.size())
        return {};
    return mKeyStrings[key.handle];
}

// ---------------------------------------------------------------------------
// TokenId <-> TfToken intern table
// ---------------------------------------------------------------------------

TokenId UsdSource::tokenFor(const PXR_NS::TfToken& token) const
{
    if (token.IsEmpty())
        return {};

    auto [it, inserted] = mTokenToId.try_emplace(token, TokenId{});
    if (inserted)
    {
        it->second = TokenId{ static_cast<uint32_t>(mIdToToken.size()) };
        mIdToToken.push_back(token);
    }
    return it->second;
}

PXR_NS::TfToken UsdSource::tfTokenFor(TokenId id) const
{
    if (id.id == 0 || id.id >= mIdToToken.size())
        return {};
    return mIdToToken[id.id];
}

TokenId UsdSource::internToken(std::string_view token) const
{
    return const_cast<UsdSource*>(this)->tokenFor(PXR_NS::TfToken(std::string(token)));
}

std::string_view UsdSource::tokenToString(TokenId id) const
{
    if (id.id == 0 || id.id >= mIdToToken.size())
        return {};
    return mIdToToken[id.id].GetString();
}

// ---------------------------------------------------------------------------
// Traversal
// ---------------------------------------------------------------------------

ObjectKey UsdSource::getRootKey() const
{
    if (!mStage)
        return {};
    return keyFor(mStage->GetPseudoRoot().GetPath());
}

void UsdSource::forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const
{
    if (!mStage)
        return;
    PXR_NS::SdfPath parentKey = pathFor(parent);
    if (parentKey.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(parentKey);
    if (!prim)
        return;
    for (const auto& child : prim.GetChildren())
    {
        cb(keyFor(child.GetPath()));
    }
}

void UsdSource::forEachDescendant(ObjectKey root, std::function<void(ObjectKey)> cb) const
{
    if (!mStage || !cb)
        return;
    const PXR_NS::SdfPath rootPath = pathFor(root);
    if (rootPath.IsEmpty())
        return;
    const PXR_NS::UsdPrim rootPrim = mStage->GetPrimAtPath(rootPath);
    if (!rootPrim)
        return;
    // AllPrims so the visited set matches the legacy UsdPrimRange::AllPrims
    // walks (root inclusive, includes inactive/abstract); consumers filter.
    for (const PXR_NS::UsdPrim& prim : PXR_NS::UsdPrimRange::AllPrims(rootPrim))
    {
        if (prim)
            cb(keyFor(prim.GetPath()));
    }
}

void UsdSource::forEachDescendantPruned(ObjectKey root, std::function<bool(ObjectKey)> visit,
                                        DescendantScope scope) const
{
    if (!mStage || !visit)
        return;
    const PXR_NS::SdfPath rootPath = pathFor(root);
    if (rootPath.IsEmpty())
        return;
    const PXR_NS::UsdPrim rootPrim = mStage->GetPrimAtPath(rootPath);
    if (!rootPrim)
        return;
    // eAll → every prim (mirrors the scanStage walk); eActiveInstanced →
    // the default predicate (active/defined/loaded/non-abstract) plus
    // descent into instance proxies, so members of instanced prototypes
    // are visited; eActive → the default predicate without instance-proxy
    // descent (plain UsdPrimRange(root), the editor's default traversal).
    PXR_NS::UsdPrimRange range =
        scope == DescendantScope::eActiveInstanced ?
            PXR_NS::UsdPrimRange(rootPrim, PXR_NS::UsdTraverseInstanceProxies()) :
            scope == DescendantScope::eActive ? PXR_NS::UsdPrimRange(rootPrim) :
                                                PXR_NS::UsdPrimRange::AllPrims(rootPrim);
    for (PXR_NS::UsdPrimRange::const_iterator it = range.begin(); it != range.end(); ++it)
    {
        if (!*it)
            continue;
        if (visit(keyFor(it->GetPath())))
            it.PruneChildren();
    }
}

ObjectKey UsdSource::findByPath(std::string_view path) const
{
    if (!mStage)
        return {};
    std::string pathStr{path};
    PXR_NS::SdfPath sdfPath{pathStr};
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(sdfPath);
    if (!prim)
        return {};
    return keyFor(sdfPath);
}

// ---------------------------------------------------------------------------
// Schema queries
// ---------------------------------------------------------------------------

bool UsdSource::hasSchema(ObjectKey key, TokenId schemaToken) const
{
    if (!mStage)
        return false;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;

    PXR_NS::TfToken token = tfTokenFor(schemaToken);
    if (token.IsEmpty())
        return false;

    if (prim.GetTypeName() == token)
        return true;

    // Match an applied schema against `token`. Per the IPhysicsSource contract,
    // a multi-apply schema matches its unqualified base name when at least one
    // instance is applied — applied names are "<base>:<instance>", so accept an
    // exact match or a "<token>:<instance>" prefix match.
    const std::string& tokenStr = token.GetString();
    auto schemaMatches = [&token, &tokenStr](const PXR_NS::TfToken& applied) {
        if (applied == token)
            return true;
        const std::string& a = applied.GetString();
        return a.size() > tokenStr.size() + 1 && a[tokenStr.size()] == ':' &&
               a.compare(0, tokenStr.size(), tokenStr) == 0;
    };

    for (const auto& applied : prim.GetAppliedSchemas())
    {
        if (schemaMatches(applied))
            return true;
    }

    // GetAppliedSchemas() filters by schema registry — schemas whose plugin
    // hasn't been loaded won't appear there. Fall back to the raw apiSchemas
    // listOp so the parser sees applied schemas regardless of whether the
    // plugin is loaded (e.g. unit tests, pipelines without omni.physxSchema).
    PXR_NS::SdfTokenListOp listOp;
    if (prim.GetMetadata(PXR_NS::TfToken("apiSchemas"), &listOp))
    {
        auto containsToken = [&schemaMatches](const PXR_NS::TfTokenVector& v) {
            for (const auto& t : v)
                if (schemaMatches(t))
                    return true;
            return false;
        };
        if (containsToken(listOp.GetExplicitItems()) ||
            containsToken(listOp.GetPrependedItems()) ||
            containsToken(listOp.GetAppendedItems()))
            return true;
    }
    return false;
}

bool UsdSource::isA(ObjectKey key, TokenId typeToken) const
{
    if (!mStage)
        return false;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;
    const PXR_NS::TfToken token = tfTokenFor(typeToken);
    if (token.IsEmpty())
        return false;
    // Polymorphic: matches the prim's type and all subclasses (prim.IsA<T>()).
    const PXR_NS::TfType type = PXR_NS::UsdSchemaRegistry::GetTypeFromSchemaTypeName(token);
    return type && prim.IsA(type);
}

bool UsdSource::exists(ObjectKey key) const
{
    if (!mStage)
        return false;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    return static_cast<bool>(mStage->GetPrimAtPath(path));
}

bool UsdSource::isPrototype(ObjectKey key) const
{
    if (!mStage)
        return false;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    return prim && prim.IsPrototype();
}

bool UsdSource::isInstanceProxy(ObjectKey key) const
{
    if (!mStage)
        return false;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    return prim && prim.IsInstanceProxy();
}

bool UsdSource::isInstance(ObjectKey key) const
{
    if (!mStage)
        return false;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    return prim && prim.IsInstance();
}

TokenId UsdSource::getTypeName(ObjectKey key) const
{
    if (!mStage)
        return {};
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return {};
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return {};
    return internToken(prim.GetTypeName().GetString());
}

void UsdSource::forEachAppliedSchema(ObjectKey key, std::function<void(TokenId)> cb) const
{
    if (!mStage || !cb)
        return;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;
    for (const PXR_NS::TfToken& name : prim.GetAppliedSchemas())
        cb(internToken(name.GetString()));
}

// ---------------------------------------------------------------------------
// Attribute access
// ---------------------------------------------------------------------------

namespace
{

// Convert a USD VtValue into a parse-lib AttrValue. Returns an invalid
// AttrValue (`kind == eNone`) when the held type isn't one parse-lib
// surfaces. `tokenInterner` is invoked when the value holds a TfToken
// — needed because TfToken→TokenId interning lives on UsdSource and
// can't be a free function.
template <typename TokenInterner>
AttrValue vtValueToAttrValue(const PXR_NS::VtValue& val, TokenInterner&& tokenInterner)
{
    if (val.IsHolding<bool>())
        return AttrValue::makeBool(val.UncheckedGet<bool>());
    if (val.IsHolding<int>())
        return AttrValue::makeInt(val.UncheckedGet<int>());
    if (val.IsHolding<int64_t>())
        return AttrValue::makeInt(val.UncheckedGet<int64_t>());
    // PhysxSceneAPI scalar fields (min/maxPositionIterationCount, timeStepsPerSecond,
    // GPU capacity ints) are declared `uint` in the schema; VtValue holds them as
    // `unsigned int` / `uint64_t`. Promote into AttrValue::Int so the parse-lib
    // `readClampedIfAuthored<uint32_t>` / `readScalarAlways` template branches read
    // the value (they unwrap via the Int kind). Without this, `getAttribute` for
    // PhysxSceneAPI uint scalars returns an empty AttrValue and the parsed scene
    // desc keeps its setToDefault values.
    if (val.IsHolding<unsigned int>())
        return AttrValue::makeInt(static_cast<int64_t>(val.UncheckedGet<unsigned int>()));
    if (val.IsHolding<uint64_t>())
        return AttrValue::makeInt(static_cast<int64_t>(val.UncheckedGet<uint64_t>()));
    if (val.IsHolding<float>())
        return AttrValue::makeFloat(val.UncheckedGet<float>());
    if (val.IsHolding<double>())
        return AttrValue::makeDouble(val.UncheckedGet<double>());
    if (val.IsHolding<PXR_NS::GfHalf>())
        return AttrValue::makeHalf(val.UncheckedGet<PXR_NS::GfHalf>());
    if (val.IsHolding<PXR_NS::GfVec2f>())
    {
        const auto& v = val.UncheckedGet<PXR_NS::GfVec2f>();
        return AttrValue::makeFloat2(carb::Float2{v[0], v[1]});
    }
    if (val.IsHolding<PXR_NS::GfVec3f>())
    {
        const auto& v = val.UncheckedGet<PXR_NS::GfVec3f>();
        return AttrValue::makeFloat3(carb::Float3{v[0], v[1], v[2]});
    }
    if (val.IsHolding<PXR_NS::GfVec4f>())
    {
        const auto& v = val.UncheckedGet<PXR_NS::GfVec4f>();
        return AttrValue::makeFloat4(carb::Float4{v[0], v[1], v[2], v[3]});
    }
    if (val.IsHolding<PXR_NS::GfQuatf>())
    {
        const auto& q = val.UncheckedGet<PXR_NS::GfQuatf>();
        auto im = q.GetImaginary();
        return AttrValue::makeFloat4(carb::Float4{im[0], im[1], im[2], q.GetReal()});
    }
    if (val.IsHolding<PXR_NS::TfToken>())
        return AttrValue::makeToken(tokenInterner(val.UncheckedGet<PXR_NS::TfToken>()));
    if (val.IsHolding<std::string>())
        return AttrValue::makeString(val.UncheckedGet<std::string>());

    // Tendon-axis APIs (`physxTendon:<inst>:gearing`,
    // `physxTendon:<inst>:forceCoefficient`) declare float[] but only
    // the first element is meaningful. Surface the first element as
    // Float so the parse-lib `getAttribute` consumer sees it as a
    // scalar — no array kind needed.
    if (val.IsHolding<PXR_NS::VtFloatArray>())
    {
        const auto& arr = val.UncheckedGet<PXR_NS::VtFloatArray>();
        if (!arr.empty())
            return AttrValue::makeFloat(arr.cdata()[0]);
    }

    return {};
}

} // namespace

AttrValue UsdSource::getAttribute(ObjectKey key, TokenId attr) const
{
    return getAttributeAtTime(key, attr, ReadTime::defaultTime());
}

std::unique_ptr<IChangeFeed> UsdSource::createChangeFeed()
{
    if (!mStage)
        return nullptr;
    return std::make_unique<UsdChangeFeed>(*this, mStage);
}

AttrValue UsdSource::getAttributeAtTime(ObjectKey key, TokenId attr, ReadTime time) const
{
    if (!mStage)
        return {};
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return {};
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return {};

    PXR_NS::TfToken token = tfTokenFor(attr);
    if (token.IsEmpty())
        return {};

    const PXR_NS::UsdTimeCode timeCode = time.mode == ReadTime::Mode::eAtTime ?
                                             PXR_NS::UsdTimeCode(time.timeCode) :
                                             PXR_NS::UsdTimeCode::Default();

    auto interner = [this](const PXR_NS::TfToken& t) {
        return const_cast<UsdSource*>(this)->tokenFor(t);
    };

    // Primary path: declared USD attribute. Handles every parse-lib
    // attribute read (schema-applied attrs and authored ones alike).
    PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(token);
    if (usdAttr && usdAttr.HasValue())
    {
        PXR_NS::VtValue val;
        if (usdAttr.Get(&val, timeCode))
        {
            AttrValue out = vtValueToAttrValue(val, interner);
            if (out.valid())
                return out;
        }
    }

    // Fallback path: customData / top-level prim metadata.
    //
    // Some per-prim physics knobs (e.g. `physics:localSpaceVelocities`)
    // are authored as USD customData dictionary entries rather than
    // schema-applied attributes (see
    // `omni::physx::utils::set_custom_metadata`). Callers don't need to
    // know which mechanism the source uses — getAttribute checks both.
    // Precedence is attribute first; an attribute and a customData
    // entry of the same name resolve to the attribute. This matches
    // existing usage where the two token namespaces don't overlap.
    PXR_NS::VtValue mval;
    if (prim.GetMetadataByDictKey(PXR_NS::SdfFieldKeys->CustomData, token, &mval) && !mval.IsEmpty())
        return vtValueToAttrValue(mval, interner);
    if (prim.GetMetadata(token, &mval) && !mval.IsEmpty())
        return vtValueToAttrValue(mval, interner);

    return {};
}

// ---------------------------------------------------------------------------
// Time-sampled query / metadata / world transform
// ---------------------------------------------------------------------------

bool UsdSource::isAttributeTimeSampled(ObjectKey key, TokenId attr) const
{
    if (!mStage)
        return false;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;

    PXR_NS::TfToken token = tfTokenFor(attr);
    if (token.IsEmpty())
        return false;

    PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(token);
    return usdAttr && usdAttr.GetNumTimeSamples() > 1;
}

bool UsdSource::mightBeTimeVarying(ObjectKey key, TokenId attr) const
{
    if (!mStage)
        return false;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;

    PXR_NS::TfToken token = tfTokenFor(attr);
    if (token.IsEmpty())
        return false;

    PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(token);
    return usdAttr && usdAttr.ValueMightBeTimeVarying();
}

bool UsdSource::hasAuthoredAttribute(ObjectKey key, TokenId attr) const
{
    if (!mStage)
        return false;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;

    PXR_NS::TfToken token = tfTokenFor(attr);
    if (token.IsEmpty())
        return false;

    PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(token);
    return usdAttr && usdAttr.HasAuthoredValue();
}

bool UsdSource::mightWorldTransformBeTimeVarying(ObjectKey key) const
{
    if (!mStage)
        return false;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;

    // Walk the prim and its ancestors up to (but excluding) the pseudo-root;
    // the world transform is time-varying if any ordered xform op carries time
    // samples. (Mirrors the legacy consumer-side scan; resetXformStack does not
    // stop the walk there, so it does not stop it here either.)
    const PXR_NS::UsdPrim root = mStage->GetPseudoRoot();
    for (PXR_NS::UsdPrim p = prim; p && p != root; p = p.GetParent())
    {
        PXR_NS::UsdGeomXformable xform(p);
        if (!xform)
            continue;
        bool resetsXformStack = false;
        for (const PXR_NS::UsdGeomXformOp& op : xform.GetOrderedXformOps(&resetsXformStack))
        {
            if (op.GetNumTimeSamples() > 1)
                return true;
        }
    }
    return false;
}

void UsdSource::getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const
{
    outMatrix = Matrix4d{}; // identity by default
    if (!mStage)
        return;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;

    if (!mXformCache)
        mXformCache = std::make_unique<PXR_NS::UsdGeomXformCache>(PXR_NS::UsdTimeCode::EarliestTime());
    PXR_NS::GfMatrix4d m = mXformCache->GetLocalToWorldTransform(prim);
    const double* src = m.GetArray(); // pxr stores row-major; matches our convention
    for (int i = 0; i < 16; ++i)
        outMatrix.data[i] = src[i];
}

void UsdSource::getLocalToWorldTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix) const
{
    outMatrix = Matrix4d{}; // identity by default
    if (!mStage)
        return;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;

    const PXR_NS::UsdTimeCode timeCode = time.mode == ReadTime::Mode::eAtTime ?
                                             PXR_NS::UsdTimeCode(time.timeCode) :
                                             PXR_NS::UsdTimeCode::Default();
    // Compute directly via UsdGeomXformable instead of a UsdGeomXformCache. This
    // overload serves runtime/animated reads where the stage may mutate between
    // calls, so a cache risks stale transforms; a fresh-per-call cache buys nothing
    // over a direct ancestor walk anyway. ComputeLocalToWorldTransform honors
    // xformOpOrder and ancestor resetXformStack just like the cache did. The no-arg
    // EarliestTime overload keeps the persistent mXformCache for the static path.
    PXR_NS::UsdGeomXformable xformable(prim);
    if (!xformable)
        return; // non-transformable prim: leave identity
    PXR_NS::GfMatrix4d m = xformable.ComputeLocalToWorldTransform(timeCode);
    const double* src = m.GetArray(); // pxr stores row-major; matches our convention
    for (int i = 0; i < 16; ++i)
        outMatrix.data[i] = src[i];
}

void UsdSource::getLocalTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix,
                                  bool& outResetsXformStack) const
{
    outMatrix = Matrix4d{}; // identity by default
    outResetsXformStack = false;
    if (!mStage)
        return;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;
    PXR_NS::UsdGeomXformable xformable(prim);
    if (!xformable)
        return;

    const PXR_NS::UsdTimeCode timeCode = time.mode == ReadTime::Mode::eAtTime ?
                                             PXR_NS::UsdTimeCode(time.timeCode) :
                                             PXR_NS::UsdTimeCode::Default();
    PXR_NS::GfMatrix4d m(1.0);
    bool reset = false;
    xformable.GetLocalTransformation(&m, &reset, timeCode);
    outResetsXformStack = reset;
    const double* src = m.GetArray(); // pxr stores row-major; matches our convention
    for (int i = 0; i < 16; ++i)
        outMatrix.data[i] = src[i];
}

void UsdSource::getLocalToWorldRotationAndScale(ObjectKey key,
                                                Matrix3d& outRotation,
                                                carb::Float3& outScale) const
{
    // Identity defaults — used on missing prims and pseudo-root.
    outRotation = Matrix3d{};
    outScale = { 1.0f, 1.0f, 1.0f };

    if (!mStage)
        return;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;

    if (!mXformCache)
        mXformCache = std::make_unique<PXR_NS::UsdGeomXformCache>(PXR_NS::UsdTimeCode::EarliestTime());

    // `GfTransform::GetRotation()` / `GetScale()` perform the polar-style
    // decomposition; both consumed below.
    PXR_NS::GfMatrix4d m = mXformCache->GetLocalToWorldTransform(prim);
    PXR_NS::GfTransform tr(m);

    PXR_NS::GfRotation rot = tr.GetRotation();
    PXR_NS::GfMatrix4d rotMat;
    rotMat.SetRotate(rot);
    // rotMat is row-major; we want the upper-left 3x3.
    const double* rmd = rotMat.GetArray();
    outRotation.data[0] = rmd[0];  outRotation.data[1] = rmd[1];  outRotation.data[2] = rmd[2];
    outRotation.data[3] = rmd[4];  outRotation.data[4] = rmd[5];  outRotation.data[5] = rmd[6];
    outRotation.data[6] = rmd[8];  outRotation.data[7] = rmd[9];  outRotation.data[8] = rmd[10];

    PXR_NS::GfVec3d s = tr.GetScale();
    outScale = { static_cast<float>(s[0]), static_cast<float>(s[1]), static_cast<float>(s[2]) };
}

ObjectKey UsdSource::getParent(ObjectKey key) const
{
    if (!mStage)
        return {};
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty() || path.IsAbsoluteRootPath())
        return {};
    PXR_NS::SdfPath parent = path.GetParentPath();
    if (parent.IsEmpty())
        return {};
    return keyFor(parent);
}

// ---------------------------------------------------------------------------
// Relationship targets
// ---------------------------------------------------------------------------

bool UsdSource::hasRelationship(ObjectKey key, TokenId rel) const
{
    if (!mStage)
        return false;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return false;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return false;
    PXR_NS::TfToken token = tfTokenFor(rel);
    if (token.IsEmpty())
        return false;
    return prim.HasRelationship(token);
}

void UsdSource::getInactiveInstanceIds(ObjectKey key, std::vector<int64_t>& out) const
{
    out.clear();
    if (!mStage)
        return;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;
    PXR_NS::SdfInt64ListOp listOp;
    if (prim.GetMetadata(PXR_NS::UsdGeomTokens->inactiveIds, &listOp))
    {
        const std::vector<int64_t>& items = listOp.GetExplicitItems();
        out.assign(items.begin(), items.end());
    }
}

void UsdSource::getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const
{
    if (!mStage)
        return;
    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;

    PXR_NS::TfToken token = tfTokenFor(rel);
    if (token.IsEmpty())
        return;

    PXR_NS::UsdRelationship relationship = prim.GetRelationship(token);
    if (!relationship)
        return;

    PXR_NS::SdfPathVector targets;
    relationship.GetTargets(&targets);
    out.reserve(out.size() + targets.size());
    for (const auto& target : targets)
    {
        out.push_back(keyFor(target));
    }
}

// ---------------------------------------------------------------------------
// Bulk buffers — VtArray-backed, keyed by monotonic id.
// ---------------------------------------------------------------------------

const void* UsdSource::resolveBuffer(BufferHandle handle, size_t& byteCount) const
{
    byteCount = 0;
    if (!handle.valid())
        return nullptr;
    auto it = mBuffers.find(handle.id);
    if (it == mBuffers.end())
        return nullptr;
    byteCount = it->second.byteCount;
    return it->second.ptr;
}

void UsdSource::releaseBuffers() const
{
    mBuffers.clear();
    mNextBufferId = 1;
}

void UsdSource::releaseBuffer(BufferHandle handle) const
{
    if (handle.valid())
        mBuffers.erase(handle.id);
}

// The returned buffer is retained in mBuffers (COW keepalive) until the caller
// releases it: every successful getArrayAttribute must be paired with a
// releaseBuffer(handle) (or releaseBuffers()), or mBuffers grows for the
// source's lifetime. PhysXTools.h::getArrayValue does this for runtime reads.
BufferHandle UsdSource::getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const
{
    if (!mStage)
        return {};
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return {};
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return {};
    const PXR_NS::TfToken token = tfTokenFor(attr);
    if (token.IsEmpty())
        return {};
    const PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(token);
    if (!usdAttr || !usdAttr.HasValue())
        return {};

    const PXR_NS::UsdTimeCode timeCode = time.mode == ReadTime::Mode::eAtTime ?
                                             PXR_NS::UsdTimeCode(time.timeCode) :
                                             PXR_NS::UsdTimeCode::Default();

    PXR_NS::VtValue val;
    if (!usdAttr.Get(&val, timeCode))
        return {};

    // Map the held VtArray element type onto a BufferElemType. Unsupported
    // element types return an invalid handle (caller falls back / returns false).
    if (val.IsHolding<PXR_NS::VtFloatArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtFloatArray>(), BufferElemType::eFloat);
    if (val.IsHolding<PXR_NS::VtVec2fArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtVec2fArray>(), BufferElemType::eVec2);
    if (val.IsHolding<PXR_NS::VtVec3fArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtVec3fArray>(), BufferElemType::eVec3);
    if (val.IsHolding<PXR_NS::VtVec4fArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtVec4fArray>(), BufferElemType::eVec4);
    if (val.IsHolding<PXR_NS::VtIntArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtIntArray>(), BufferElemType::eInt32);
    if (val.IsHolding<PXR_NS::VtUIntArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtUIntArray>(), BufferElemType::eUInt32);
    if (val.IsHolding<PXR_NS::VtVec3iArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtVec3iArray>(), BufferElemType::eInt3);
    if (val.IsHolding<PXR_NS::VtVec4iArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtVec4iArray>(), BufferElemType::eInt4);
    if (val.IsHolding<PXR_NS::VtQuathArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtQuathArray>(), BufferElemType::eQuath);
    if (val.IsHolding<PXR_NS::VtUCharArray>())
        return registerBuffer(val.UncheckedGet<PXR_NS::VtUCharArray>(), BufferElemType::eUInt8);
    return {};
}

MeshGeometry UsdSource::getMeshAttributes(ObjectKey key) const
{
    MeshGeometry out;
    if (!key.valid())
        return out;

    PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty() || !mStage)
        return out;

    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim || !prim.IsA<PXR_NS::UsdGeomMesh>())
        return out;

    PXR_NS::UsdGeomMesh mesh(prim);

    // Geometry buffers — read once at default time, mint a BufferHandle for
    // each non-empty array (`registerBuffer` returns an invalid handle for
    // empty inputs).
    PXR_NS::VtArray<PXR_NS::GfVec3f> points;
    if (auto attr = mesh.GetPointsAttr())
        attr.Get(&points);
    out.points = registerBuffer(points, BufferElemType::eVec3);

    PXR_NS::VtArray<int32_t> indices;
    if (auto attr = mesh.GetFaceVertexIndicesAttr())
        attr.Get(&indices);
    out.indices = registerBuffer(indices, BufferElemType::eInt32);

    PXR_NS::VtArray<int32_t> faceCounts;
    if (auto attr = mesh.GetFaceVertexCountsAttr())
        attr.Get(&faceCounts);
    out.faceCounts = registerBuffer(faceCounts, BufferElemType::eInt32);

    PXR_NS::VtArray<int32_t> holes;
    if (auto attr = mesh.GetHoleIndicesAttr())
        attr.Get(&holes);
    out.holes = registerBuffer(holes, BufferElemType::eInt32);

    // Scalars — doubleSided defaults to false, orientation defaults to
    // rightHanded. USD's `Get()` fills with the schema fallback when
    // unauthored, so the cascade naturally lands on the correct defaults.
    if (auto attr = mesh.GetDoubleSidedAttr())
        attr.Get(&out.doubleSided);

    PXR_NS::TfToken orientation = PXR_NS::UsdGeomTokens->rightHanded;
    if (auto attr = mesh.GetOrientationAttr())
        attr.Get(&orientation);
    out.leftHanded = (orientation == PXR_NS::UsdGeomTokens->leftHanded);

    // Per-face physics-material indices from material-bound `GeomSubset`s.
    // Reproduces the cooking service's TriangulateUSDPrim::fillFaceMaterials so
    // the index scheme is identical to the prim-id cooking path: faces default to
    // `materialCount` (one past the last), and each material-bound face subset
    // (in declaration order) stamps its faces with the next 0-based index. Left
    // invalid when the mesh has no material subsets (single-material).
    {
        const std::vector<PXR_NS::UsdGeomSubset> subsets =
            PXR_NS::UsdGeomSubset::GetGeomSubsets(mesh, PXR_NS::UsdGeomTokens->face);
        if (!subsets.empty())
        {
            // Resolve subset materials through the exact same helper the cooking
            // service uses (getMaterialBindingPath mirrors
            // usdmaterialutils::getMaterialBinding, which
            // TriangulateUSDPrim::fillFaceMaterials calls), guaranteeing the
            // per-face index scheme matches the prim-id cooking path.
            std::set<PXR_NS::SdfPath> physicsSubsets;
            for (const PXR_NS::UsdGeomSubset& subset : subsets)
            {
                const PXR_NS::SdfPath material = getMaterialBindingPath(subset.GetPrim());
                if (!material.IsEmpty())
                {
                    const PXR_NS::UsdPrim materialPrim = mStage->GetPrimAtPath(material);
                    if (materialPrim && materialPrim.HasAPI<PXR_NS::UsdPhysicsMaterialAPI>())
                        physicsSubsets.insert(subset.GetPath());
                }
            }

            if (!physicsSubsets.empty())
            {
                const size_t facesCountSz = faceCounts.size();
                PXR_NS::VtArray<uint16_t> faceMat(facesCountSz);
                std::fill(faceMat.begin(), faceMat.end(), static_cast<uint16_t>(physicsSubsets.size()));
                uint16_t materialIndex = 0;
                for (const PXR_NS::UsdGeomSubset& subset : subsets)
                {
                    if (physicsSubsets.find(subset.GetPath()) == physicsSubsets.end())
                        continue;
                    PXR_NS::VtArray<int> faceIdx;
                    subset.GetIndicesAttr().Get(&faceIdx);
                    for (int face : faceIdx)
                        if (face >= 0 && static_cast<size_t>(face) < facesCountSz)
                            faceMat[face] = materialIndex;
                    ++materialIndex;
                }
                out.faceMaterials = registerBuffer(faceMat, BufferElemType::eUInt16);
            }
        }
    }

    return out;
}

// ---------------------------------------------------------------------------
// Global
// ---------------------------------------------------------------------------

SourceUnits UsdSource::getSourceUnits() const
{
    if (!mStage)
        return {};

    SourceUnits units;
    units.metersPerUnit = static_cast<float>(PXR_NS::UsdGeomGetStageMetersPerUnit(mStage));
    units.kilogramsPerUnit = static_cast<float>(PXR_NS::UsdPhysicsGetStageKilogramsPerUnit(mStage));

    PXR_NS::TfToken upAxis = PXR_NS::UsdGeomGetStageUpAxis(mStage);
    units.upAxis = (upAxis == PXR_NS::UsdGeomTokens->y) ? UpAxis::eY : UpAxis::eZ;

    return units;
}

// ---------------------------------------------------------------------------
// Collection pre-resolution.
//
// @implements REQ-PARSE-COLGROUP-002
// @covers AC-1 AC-2
// ---------------------------------------------------------------------------

void UsdSource::resolveCollection(ObjectKey primKey,
                                  TokenId collectionName,
                                  std::vector<ObjectKey>& members) const
{
    members.clear();

    if (!mStage)
        return;

    PXR_NS::SdfPath path = pathFor(primKey);
    if (path.IsEmpty())
        return;
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;

    PXR_NS::TfToken name = tfTokenFor(collectionName);
    if (name.IsEmpty())
        return;

    PXR_NS::UsdCollectionAPI collectionAPI = PXR_NS::UsdCollectionAPI::Get(prim, name);
    if (!collectionAPI)
        return;

    const PXR_NS::SdfPathSet includedPaths = PXR_NS::UsdCollectionAPI::ComputeIncludedPaths(
        collectionAPI.ComputeMembershipQuery(),
        mStage,
        PXR_NS::UsdTraverseInstanceProxies());

    members.reserve(includedPaths.size());
    for (const PXR_NS::SdfPath& p : includedPaths)
        members.push_back(keyFor(p));
}

// @implements REQ-PARSE-CORE-003
// @covers AC-1
void UsdSource::forEachMultiApplyInstance(
    ObjectKey key,
    std::string_view baseSchema,
    std::function<void(std::string_view)> cb) const
{
    if (!mStage || !cb)
        return;
    const PXR_NS::SdfPath path = pathFor(key);
    if (path.IsEmpty())
        return;
    const PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return;

    // Iterate the prim's applied schemas, including those not yet
    // registered with the schema registry (mirrors hasSchema's fallback
    // path so multi-apply discovery works in unit tests that don't load
    // PhysxSchema's plugInfo). Dedup so an instance appearing in both
    // GetAppliedSchemas() and the raw listOp metadata is reported once.
    std::unordered_set<std::string> seen;
    auto consume = [&](const PXR_NS::TfTokenVector& schemas) {
        for (const PXR_NS::TfToken& tok : schemas)
        {
            const std::string& s = tok.GetString();
            if (s.size() <= baseSchema.size() + 1)
                continue;
            if (std::string_view(s).substr(0, baseSchema.size()) != baseSchema)
                continue;
            if (s[baseSchema.size()] != ':')
                continue;
            if (!seen.insert(s).second)
                continue;
            cb(std::string_view(s).substr(baseSchema.size() + 1));
        }
    };

    consume(prim.GetAppliedSchemas());

    PXR_NS::SdfTokenListOp listOp;
    if (prim.GetMetadata(PXR_NS::TfToken("apiSchemas"), &listOp))
    {
        if (listOp.GetExplicitItems().size())
            consume(listOp.GetExplicitItems());
        if (listOp.GetPrependedItems().size())
            consume(listOp.GetPrependedItems());
        if (listOp.GetAppendedItems().size())
            consume(listOp.GetAppendedItems());
    }
}

ObjectKey UsdSource::getMaterialBinding(ObjectKey primKey) const
{
    // Walks UsdShadeMaterialBindingAPI first; falls back to
    // ComputeBoundMaterials on a single-prim vector.
    static const PXR_NS::TfToken physicsPurpose("physics");

    if (!mStage)
        return {};
    PXR_NS::SdfPath path = pathFor(primKey);
    if (path.IsEmpty())
        return {};
    PXR_NS::UsdPrim prim = mStage->GetPrimAtPath(path);
    if (!prim)
        return {};

    PXR_NS::UsdShadeMaterialBindingAPI bindingAPI(prim);
    PXR_NS::SdfPath materialKey;
    if (bindingAPI)
    {
        PXR_NS::UsdShadeMaterial material = bindingAPI.ComputeBoundMaterial(physicsPurpose);
        if (material)
            materialKey = material.GetPrim().GetPrimPath();
    }
    else
    {
        std::vector<PXR_NS::UsdPrim> prims{ prim };
        std::vector<PXR_NS::UsdShadeMaterial> materials =
            PXR_NS::UsdShadeMaterialBindingAPI::ComputeBoundMaterials(prims, physicsPurpose);
        if (!materials.empty() && materials[0])
            materialKey = materials[0].GetPrim().GetPrimPath();
    }

    if (materialKey.IsEmpty())
        return {};
    return keyFor(materialKey);
}

} // namespace omni::physics::usd
