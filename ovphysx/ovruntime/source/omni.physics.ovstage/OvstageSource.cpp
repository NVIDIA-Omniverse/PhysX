// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "OvstageSource.h"

#include "OvstageChangeFeed.h"

#include <carb/extras/Hash.h>

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/quath.h>
#include <pxr/base/tf/token.h>
#include <pxr/base/vt/array.h>
#include <pxr/base/vt/value.h>
#include <pxr/usd/sdf/listOp.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/sdf/attributeSpec.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/timeCode.h>
#include <pxr/usd/usdUtils/stageCache.h>
#include <pxr/usd/usdGeom/metrics.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdGeom/xformable.h>
#include <pxr/usd/usdGeom/xformCache.h>
#include <pxr/usd/usdPhysics/metrics.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <string_view>
#include <unordered_set>

namespace omni::physics::ovstage
{

namespace
{
class ScopedPathList
{
public:
    explicit ScopedPathList(ovx_path_dictionary_t* dict) : mDict(dict) {}
    ~ScopedPathList()
    {
        if (mDict && mList != OVX_INVALID_PRIMPATH_LIST)
            ovx_path_dictionary_destroy_path_list(mDict, mList);
    }

    ScopedPathList(const ScopedPathList&) = delete;
    ScopedPathList& operator=(const ScopedPathList&) = delete;

    ovx_primpath_list_t* receive() { return &mList; }
    ovx_primpath_list_t get() const { return mList; }

private:
    ovx_path_dictionary_t* mDict = nullptr;
    ovx_primpath_list_t mList = OVX_INVALID_PRIMPATH_LIST;
};

// Test-only fault hook. When > 0, buildChildCache()
// treats its authoritative usd-path read as NOT having completed cleanly — exactly
// the observable effect of a transient/partial ovstage read — so the completeness
// gate can be exercised deterministically. Each build consumes one count. Always 0
// in production; set only by setOvstageAuthoritativeReadFaultForTest().
int& authoritativeReadFaultCounter()
{
    static int counter = 0;
    return counter;
}

bool consumeAuthoritativeReadFaultForTest()
{
    if (authoritativeReadFaultCounter() <= 0)
        return false;
    --authoritativeReadFaultCounter();
    return true;
}

// Test-only call counter for the expensive complete-graph instance-root query.
// The parse-unit tests reset and read it around a synchronous scan. It remains
// zero unless buildPhysicsInstancingCache() expands a prototype mapping.
std::atomic_size_t& instanceRootQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& prototypeRootQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Wait for and release an enqueued async op (mirrors the ovstage test helper).
void waitAndRelease(ovstage_instance_t* inst, ovstage_enqueue_result_t r)
{
    if (r.status != OVSTAGE_OK || r.op_index == OVSTAGE_INVALID_OP_ID)
        return;
    ovstage_wait_op(inst, r.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
    ovstage_release_op(inst, r.op_index);
}

// Total scalar count = product of dims × dtype lanes. Components may live in the
// shape (ovstage-API writes, e.g. the populator: lanes=1) OR in dtype.lanes
// (Fabric-backed attrs can arrive ndim=1, shape=[1], lanes=N). Folding lanes in
// handles both with one count.
int64_t totalElements(const DLTensor& t)
{
    int64_t n = 1;
    for (int i = 0; i < t.ndim; ++i)
        n *= t.shape[i];
    return n * (t.dtype.lanes > 0 ? t.dtype.lanes : 1);
}

// Per-prim component count for a column tensor: total scalar count / prims.
int64_t componentsPerPrim(const DLTensor& t, uint32_t primCount)
{
    const int64_t total = totalElements(t);
    return (primCount > 0) ? total / static_cast<int64_t>(primCount) : total;
}

// Decode row `rowIndex`'s value (one prim's value) from a per-column DLTensor.
// `comps` is the per-prim component count (1=scalar, 3=Float3, ...); row r lives
// at element offset r*comps. Assumes dense contiguous CPU storage.
AttrValue decodeAt(const DLTensor& t, int64_t comps, int64_t rowIndex)
{
    // Reject malformed component widths before forming the base pointer or reading a
    // value: an empty fixed column (comps < 1) has no element 0 to read, and widths
    // above four exceed the scalar/vector AttrValue range (they would otherwise be
    // silently truncated to Float4). Supported widths are 1..4.
    if (comps < 1 || comps > 4)
        return {};

    const DLDataType dt = t.dtype;
    const int64_t elemBytes = dt.bits / 8;
    const auto* base = static_cast<const uint8_t*>(t.data) + t.byte_offset + rowIndex * comps * elemBytes;

    if (dt.code == kDLFloat && dt.bits == 32)
    {
        const auto* f = reinterpret_cast<const float*>(base);
        if (comps <= 1) return AttrValue::makeFloat(f[0]);
        if (comps == 2) return AttrValue::makeFloat2({ f[0], f[1] });
        if (comps == 3) return AttrValue::makeFloat3({ f[0], f[1], f[2] });
        return AttrValue::makeFloat4({ f[0], f[1], f[2], f[3] });
    }
    if (dt.code == kDLFloat && dt.bits == 64)
    {
        const auto* d = reinterpret_cast<const double*>(base);
        if (comps <= 1) return AttrValue::makeDouble(d[0]);
        if (comps == 2) return AttrValue::makeFloat2({ static_cast<float>(d[0]), static_cast<float>(d[1]) });
        if (comps == 3)
            return AttrValue::makeFloat3({ static_cast<float>(d[0]), static_cast<float>(d[1]), static_cast<float>(d[2]) });
        // comps < 4 is fully handled above; only width >= 4 reaches here, so d[0..3] stay in-bounds.
        return AttrValue::makeFloat4({ static_cast<float>(d[0]), static_cast<float>(d[1]),
                                       static_cast<float>(d[2]), static_cast<float>(d[3]) });
    }
    // 8-bit scalar. USD `bool` serializes to a single byte and ovpopulation stores
    // it as DLPack UInt8 (there is no dedicated bool semantic; is_array=false marks
    // it a scalar, not a 1-byte string). It is the only 8-bit scalar in the physics
    // domain, so decode any 8-bit int/uint as a bool — mirroring the kDLBool case so
    // getValue<bool> (fromAttr only accepts eBool) reads runtime edits, not just the
    // populated defaults.
    if ((dt.code == kDLInt || dt.code == kDLUInt || dt.code == kDLBool) && dt.bits == 8)
        return AttrValue::makeBool(reinterpret_cast<const uint8_t*>(base)[0] != 0);
    if (dt.code == kDLInt)
    {
        if (dt.bits == 64) return AttrValue::makeInt(reinterpret_cast<const int64_t*>(base)[0]);
        if (dt.bits == 32) return AttrValue::makeInt(reinterpret_cast<const int32_t*>(base)[0]);
    }
    if (dt.code == kDLUInt)
    {
        if (dt.bits == 64) return AttrValue::makeInt(static_cast<int64_t>(reinterpret_cast<const uint64_t*>(base)[0]));
        if (dt.bits == 32) return AttrValue::makeInt(static_cast<int64_t>(reinterpret_cast<const uint32_t*>(base)[0]));
    }
    if (dt.code == kDLBool)
        return AttrValue::makeBool(reinterpret_cast<const uint8_t*>(base)[0] != 0);

    return {};
}

// Decode element 0's value from a per-column DLTensor (single-prim read path).
AttrValue decodeScalar(const DLTensor& t, uint32_t primCount)
{
    return decodeAt(t, componentsPerPrim(t, primCount), 0);
}

bool canDecodeAttrValue(const DLTensor& t)
{
    const DLDataType dt = t.dtype;
    if (dt.code == kDLFloat && (dt.bits == 32 || dt.bits == 64))
        return true;
    if ((dt.code == kDLInt || dt.code == kDLUInt) && (dt.bits == 8 || dt.bits == 32 || dt.bits == 64))
        return true;
    return dt.code == kDLBool;
}

bool canDecodeRelationshipTargets(const DLTensor& t)
{
    return t.data && t.dtype.code == kDLUInt && t.dtype.bits == 64;
}

bool matrixIsFinite(const double values[16])
{
    for (int i = 0; i < 16; ++i)
    {
        if (!std::isfinite(values[i]))
            return false;
    }
    return true;
}

// Decode row `rowIndex` as a row-major 4x4 matrix (16 doubles). `comps` must be
// >= 16. Returns false on an unsupported dtype / too-few components.
bool decodeMatrixAt(const DLTensor& t, int64_t comps, int64_t rowIndex, double out[16])
{
    if (comps < 16)
        return false;
    const int64_t elemBytes = t.dtype.bits / 8;
    const auto* base = static_cast<const uint8_t*>(t.data) + t.byte_offset + rowIndex * comps * elemBytes;
    if (t.dtype.code == kDLFloat && t.dtype.bits == 64)
    {
        std::memcpy(out, base, 16 * sizeof(double));
        return matrixIsFinite(out);
    }
    if (t.dtype.code == kDLFloat && t.dtype.bits == 32)
    {
        const auto* f = reinterpret_cast<const float*>(base);
        for (int i = 0; i < 16; ++i)
            out[i] = static_cast<double>(f[i]);
        return matrixIsFinite(out);
    }
    return false;
}

Matrix4d multiplyMatrix(const Matrix4d& a, const Matrix4d& b)
{
    Matrix4d out;
    for (int r = 0; r < 4; ++r)
    {
        for (int c = 0; c < 4; ++c)
        {
            double v = 0.0;
            for (int k = 0; k < 4; ++k)
                v += a.data[r * 4 + k] * b.data[k * 4 + c];
            out.data[r * 4 + c] = v;
        }
    }
    return out;
}

Matrix4d makeTranslationMatrix(const carb::Float3& t)
{
    Matrix4d out;
    out.data[12] = static_cast<double>(t.x);
    out.data[13] = static_cast<double>(t.y);
    out.data[14] = static_cast<double>(t.z);
    return out;
}

Matrix4d makeScaleMatrix(const carb::Float3& s)
{
    Matrix4d out;
    out.data[0] = static_cast<double>(s.x);
    out.data[5] = static_cast<double>(s.y);
    out.data[10] = static_cast<double>(s.z);
    return out;
}

Matrix4d makeQuaternionMatrix(const carb::Float4& q)
{
    const double x = static_cast<double>(q.x);
    const double y = static_cast<double>(q.y);
    const double z = static_cast<double>(q.z);
    const double w = static_cast<double>(q.w);
    const double n = x * x + y * y + z * z + w * w;
    if (n <= 1.0e-24)
        return {};
    const double s = 2.0 / n;
    const double xx = x * x * s;
    const double yy = y * y * s;
    const double zz = z * z * s;
    const double xy = x * y * s;
    const double xz = x * z * s;
    const double yz = y * z * s;
    const double wx = w * x * s;
    const double wy = w * y * s;
    const double wz = w * z * s;

    Matrix4d out;
    // Matrix4d uses USD/PhysX parser row-vector convention, so this is the
    // transpose of the usual column-vector quaternion matrix.
    out.data[0] = 1.0 - yy - zz;
    out.data[1] = xy + wz;
    out.data[2] = xz - wy;
    out.data[4] = xy - wz;
    out.data[5] = 1.0 - xx - zz;
    out.data[6] = yz + wx;
    out.data[8] = xz + wy;
    out.data[9] = yz - wx;
    out.data[10] = 1.0 - xx - yy;
    return out;
}

Matrix4d makeAxisRotationMatrix(char axis, double degrees)
{
    constexpr double kPi = 3.141592653589793238462643383279502884;
    const double radians = degrees * kPi / 180.0;
    const double c = std::cos(radians);
    const double s = std::sin(radians);
    Matrix4d out;
    switch (axis)
    {
    case 'X':
        out.data[5] = c;
        out.data[6] = s;
        out.data[9] = -s;
        out.data[10] = c;
        break;
    case 'Y':
        out.data[0] = c;
        out.data[2] = -s;
        out.data[8] = s;
        out.data[10] = c;
        break;
    case 'Z':
        out.data[0] = c;
        out.data[1] = s;
        out.data[4] = -s;
        out.data[5] = c;
        break;
    default:
        break;
    }
    return out;
}

bool attrAsFloat3(const AttrValue& v, carb::Float3& out)
{
    if (v.kind == AttrValue::Kind::eFloat3)
    {
        out = v.f3;
        return true;
    }
    return false;
}

bool attrAsDouble(const AttrValue& v, double& out)
{
    if (v.kind == AttrValue::Kind::eDouble)
    {
        out = v.d;
        return true;
    }
    if (v.kind == AttrValue::Kind::eFloat || v.kind == AttrValue::Kind::eHalf)
    {
        out = static_cast<double>(v.f);
        return true;
    }
    if (v.kind == AttrValue::Kind::eInt)
    {
        out = static_cast<double>(v.i);
        return true;
    }
    return false;
}

bool attrAsQuaternion(const AttrValue& v, carb::Float4& out)
{
    if (v.kind != AttrValue::Kind::eFloat4)
        return false;
    // Ovstage decodes quaternion attributes into the parser/runtime layout:
    // (i, j, k, real).
    out = v.f4;
    return true;
}

void copyGfMatrix(const PXR_NS::GfMatrix4d& src, Matrix4d& dst)
{
    const double* values = src.GetArray();
    for (int i = 0; i < 16; ++i)
        dst.data[i] = values[i];
}

PXR_NS::UsdStageRefPtr usdStageFromId(uint64_t usdStageId)
{
    if (usdStageId == 0)
        return {};
    return PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(static_cast<long int>(usdStageId)));
}

PXR_NS::UsdTimeCode usdTimeCode(ReadTime time)
{
    return time.mode == ReadTime::Mode::eAtTime ? PXR_NS::UsdTimeCode(time.timeCode) :
                                                  PXR_NS::UsdTimeCode::Default();
}

bool usdAttributeHasValueType(const PXR_NS::UsdAttribute& attr, PXR_NS::UsdTimeCode timeCode)
{
    if (!attr)
        return false;

    const PXR_NS::SdfPropertySpecHandleVector stack = attr.GetPropertyStack(timeCode);
    for (const PXR_NS::SdfPropertySpecHandle& propSpec : stack)
    {
        const PXR_NS::SdfAttributeSpecHandle attrSpec =
            PXR_NS::TfDynamic_cast<PXR_NS::SdfAttributeSpecHandle>(propSpec);
        if (attrSpec && attrSpec->GetTypeName())
            return true;
    }

    const PXR_NS::UsdPrim prim = attr.GetPrim();
    if (!prim)
        return false;
    const PXR_NS::SdfAttributeSpecHandle schemaSpec =
        prim.GetPrimDefinition().GetSchemaAttributeSpec(attr.GetName());
    return schemaSpec && schemaSpec->GetTypeName();
}

template <typename TokenInterner>
AttrValue vtValueToAttrValue(const PXR_NS::VtValue& val, TokenInterner&& tokenInterner)
{
    if (val.IsHolding<bool>())
        return AttrValue::makeBool(val.UncheckedGet<bool>());
    if (val.IsHolding<int>())
        return AttrValue::makeInt(val.UncheckedGet<int>());
    if (val.IsHolding<int64_t>())
        return AttrValue::makeInt(val.UncheckedGet<int64_t>());
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
        return AttrValue::makeFloat2(carb::Float2{ v[0], v[1] });
    }
    if (val.IsHolding<PXR_NS::GfVec3f>())
    {
        const auto& v = val.UncheckedGet<PXR_NS::GfVec3f>();
        return AttrValue::makeFloat3(carb::Float3{ v[0], v[1], v[2] });
    }
    if (val.IsHolding<PXR_NS::GfVec4f>())
    {
        const auto& v = val.UncheckedGet<PXR_NS::GfVec4f>();
        return AttrValue::makeFloat4(carb::Float4{ v[0], v[1], v[2], v[3] });
    }
    if (val.IsHolding<PXR_NS::GfQuatf>())
    {
        const auto& q = val.UncheckedGet<PXR_NS::GfQuatf>();
        const auto im = q.GetImaginary();
        return AttrValue::makeFloat4(carb::Float4{ im[0], im[1], im[2], q.GetReal() });
    }
    if (val.IsHolding<PXR_NS::TfToken>())
        return AttrValue::makeToken(tokenInterner(val.UncheckedGet<PXR_NS::TfToken>()));
    if (val.IsHolding<std::string>())
        return AttrValue::makeString(val.UncheckedGet<std::string>());
    if (val.IsHolding<PXR_NS::VtFloatArray>())
    {
        const auto& arr = val.UncheckedGet<PXR_NS::VtFloatArray>();
        if (!arr.empty())
            return AttrValue::makeFloat(arr.cdata()[0]);
    }
    return {};
}


} // namespace

// Test-only: arm `count` subsequent buildChildCache() authoritative reads to be
// treated as transient/partial (not cleanly completed), so the completeness gate
// that guards the negative-leaf fast path can be exercised deterministically.
// Not declared in the public header; test translation
// units forward-declare it. Pass 0 to disarm.
void setOvstageAuthoritativeReadFaultForTest(int count)
{
    authoritativeReadFaultCounter() = count;
}

// Not declared in the public header; test translation units forward-declare
// these accessors. The process-global count assumes sequential test execution,
// matching the existing authoritative-read fault seam above.
void resetOvstageInstanceRootQueryCountForTest()
{
    instanceRootQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageInstanceRootQueryCountForTest()
{
    return instanceRootQueryCounter().load(std::memory_order_relaxed);
}

void resetOvstagePrototypeRootQueryCountForTest()
{
    prototypeRootQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstagePrototypeRootQueryCountForTest()
{
    return prototypeRootQueryCounter().load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------

OvstageSource::OvstageSource(ovstage_instance_t* instance,
                                 ovx_path_dictionary_t* dict,
                                 ovstage_ordinal_t readOrdinal,
                                 uint64_t usdStageId)
    : mInstance(instance), mDict(dict), mReadOrdinal(readOrdinal), mUsdStageId(usdStageId)
{
    mTokenToString.emplace_back(); // index 0 = invalid token
    loadUnits();
}

OvstageSource::~OvstageSource()
{
    clearLoadCache();
}

// --- tokens ----------------------------------------------------------------

TokenId OvstageSource::internToken(std::string_view token) const
{
    return doInternToken(token);
}

std::string_view OvstageSource::tokenToString(TokenId id) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (id.id == 0 || id.id >= mTokenToString.size())
        return {};
    return mTokenToString[id.id];
}

TokenId OvstageSource::doInternToken(std::string_view token) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    std::string key{ token };
    auto it = mStringToToken.find(key);
    if (it != mStringToToken.end())
        return it->second;
    TokenId id{ static_cast<uint32_t>(mTokenToString.size()) };
    mTokenToString.push_back(key);
    mStringToToken[key] = id;
    return id;
}

ovx_token_t OvstageSource::ovxToken(std::string_view s) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    ovx_token_t tok = OVX_INVALID_TOKEN;
    const ovx_string_t str{ s.data(), s.size() };
    ovx_path_dictionary_intern_token(mDict, str, &tok);
    return tok;
}

uint64_t OvstageSource::canonicalHandle(ObjectKey key) const
{
    if (!key.valid())
        return 0;
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const auto cached = mCanonicalHandleCache.find(key.handle);
    if (cached != mCanonicalHandleCache.end())
        return cached->second;

    const std::string p = pathOf(key);
    if (p.empty())
    {
        mCanonicalHandleCache[key.handle] = key.handle;
        return key.handle;
    }
    const ObjectKey c = findByPath(p);
    const uint64_t canonical = c.valid() ? c.handle : key.handle;
    mCanonicalHandleCache[key.handle] = canonical;
    if (canonical)
        mCanonicalHandleCache[canonical] = canonical;
    return canonical;
}

bool OvstageSource::buildPrototypeRootCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mPrototypeRootCacheInitialized)
        return mPrototypeRootCacheValid;
    mPrototypeRootCacheInitialized = true;
    mPrototypeRootCacheValid = false;
    mPrototypeRootPaths.clear();
    if (!mInstance || !mDict)
        return false;
    ovx_path_dictionary_t* const instancingDict = ovstage_get_path_dictionary(mInstance);
    if (!instancingDict)
        return false;

    // The public instancing API describes latest committed topology and has no
    // ordinal parameter. Initial attach/population must therefore remain stable
    // while this source-scoped cache is built; historical topology is not
    // recoverable through this API. This prototype-root query is a single
    // filtered read. Do not eagerly call get_instance_roots() for every root:
    // each call currently rebuilds the complete instancing graph.
    ScopedPathList prototypeList(instancingDict);
    if (ovstage_instancing_get_prototype_roots(mInstance, prototypeList.receive()) != OVSTAGE_OK)
        return false;

    const ovx_primpath_t* prototypeData = nullptr;
    size_t prototypeCount = 0;
    if (ovx_path_dictionary_get_paths(instancingDict, prototypeList.get(), &prototypeData, &prototypeCount) != OVX_OK)
        return false;

    std::unordered_set<std::string> prototypeRootPaths;
    for (size_t i = 0; i < prototypeCount; ++i)
    {
        const std::string prototypePath = pathOf(ObjectKey{ prototypeData[i] });
        if (prototypePath.empty())
            return false;
        prototypeRootPaths.insert(prototypePath);
    }

    mPrototypeRootPaths = std::move(prototypeRootPaths);
    mPrototypeRootCacheValid = true;
    return true;
}

bool OvstageSource::buildPhysicsInstancingCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mPhysicsInstancingCacheInitialized)
        return mPhysicsInstancingCacheValid;
    mPhysicsInstancingCacheInitialized = true;
    mPhysicsInstancingCacheValid = false;
    mPrototypeRootByInstanceRoot.clear();
    mGeometryBackingCache.clear();
    if (!buildPrototypeRootCache())
        return false;
    if (mPrototypeRootPaths.empty())
    {
        mPhysicsInstancingCacheValid = true;
        return true;
    }

    // Temporary consumer-side containment until OVStage provides the batched
    // resolver tracked by OMPE-100947. One get_instance_roots() call currently
    // rebuilds the full P-prototype/I-instance graph. Raw collision membership
    // reduces attach from P such rebuilds to R, where R is the number of unique
    // prototype roots that actually back physics collision shapes.
    bool needsCompleteExpansion = false;
    std::vector<ObjectKey> collisionKeys;
    if (!collectSchemaKeys(internToken("PhysicsCollisionAPI"), collisionKeys))
        needsCompleteExpansion = true;

    std::unordered_set<std::string> relevantPrototypePathSet;
    std::vector<ObjectKey> externalCollisionKeys;
    if (!needsCompleteExpansion)
    {
        for (const ObjectKey key : collisionKeys)
        {
            bool foundPrototype = false;
            std::string ancestor = pathOf(key);
            while (!ancestor.empty())
            {
                if (mPrototypeRootPaths.count(ancestor) != 0)
                {
                    relevantPrototypePathSet.insert(ancestor);
                    foundPrototype = true;
                    break;
                }
                const size_t slash = ancestor.rfind('/');
                if (slash == std::string::npos || slash == 0)
                    break;
                ancestor.resize(slash);
            }
            if (foundPrototype)
                continue;
            externalCollisionKeys.push_back(key);
        }
    }

    if (!externalCollisionKeys.empty())
    {
        if (!mChildCacheBuilt)
            buildChildCache();
        needsCompleteExpansion = !mChildCacheComplete;
        for (const ObjectKey key : externalCollisionKeys)
        {
            uint64_t parent = key.handle;
            std::unordered_map<uint64_t, std::vector<uint64_t>>::const_iterator children =
                mChildCache.find(parent);
            if (children == mChildCache.end())
            {
                parent = canonicalHandle(key);
                children = mChildCache.find(parent);
            }
            if (children != mChildCache.end() && !children->second.empty())
            {
                needsCompleteExpansion = true;
                break;
            }
        }
    }

    // The walker also supports CollisionAPI on a non-geometry prim whose child
    // geometry may come from an instance prototype. The public OVStage API has
    // only a per-instance-root reverse resolver, not a batched reverse query.
    // Preserve the old complete expansion for this ambiguous shape, or when an
    // authoritative read was incomplete, instead of reading private topology
    // columns or issuing reverse queries once per collider ancestor. Leaf
    // colliders, including the QA workload, stay on the physics-scoped path.
    if (needsCompleteExpansion)
        relevantPrototypePathSet = mPrototypeRootPaths;

    std::vector<std::string> relevantPrototypePaths(
        relevantPrototypePathSet.begin(), relevantPrototypePathSet.end());
    std::sort(relevantPrototypePaths.begin(), relevantPrototypePaths.end());

    ovx_path_dictionary_t* const instancingDict = ovstage_get_path_dictionary(mInstance);
    if (!instancingDict)
        return false;

    std::unordered_map<std::string, std::string> prototypeRootByInstanceRoot;
    for (const std::string& prototypePath : relevantPrototypePaths)
    {
        const ObjectKey prototypeRoot = findByPath(prototypePath);
        if (!prototypeRoot.valid())
            return false;

        ScopedPathList instanceList(instancingDict);
        instanceRootQueryCounter().fetch_add(1, std::memory_order_relaxed);
        if (ovstage_instancing_get_instance_roots(mInstance, prototypeRoot.handle, instanceList.receive()) != OVSTAGE_OK)
            return false;

        const ovx_primpath_t* instanceData = nullptr;
        size_t instanceCount = 0;
        if (ovx_path_dictionary_get_paths(instancingDict, instanceList.get(), &instanceData, &instanceCount) != OVX_OK)
            return false;

        for (size_t i = 0; i < instanceCount; ++i)
        {
            const std::string instancePath = pathOf(ObjectKey{ instanceData[i] });
            if (instancePath.empty())
                return false;
            prototypeRootByInstanceRoot[instancePath] = prototypePath;
        }
    }

    mPrototypeRootByInstanceRoot = std::move(prototypeRootByInstanceRoot);
    mPhysicsInstancingCacheValid = true;
    return true;
}

bool OvstageSource::buildInstanceMaterialCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mInstanceMaterialCacheBuilt)
        return mInstanceMaterialCacheValid;

    mInstanceMaterialCacheBuilt = true;
    mInstanceMaterialCacheValid = false;
    mInstanceMaterialByPrim.clear();
    if (!mInstance || !mDict)
        return false;

    const ovx_token_t materialAttr = ovxToken("_instanceMaterialBinding");
    if (materialAttr == OVX_INVALID_TOKEN)
        return false;

    ovstage_predicate_t predicate{};
    predicate.attribute.token = materialAttr;
    predicate.op = OVSTAGE_FILTER_OP_HAS;
    ovstage_filter_t filter{};
    filter.predicates = &predicate;
    filter.count = 1;

    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t queryEnqueue = ovstage_query(mInstance, &filter, nullptr, 0, &query);
    if (queryEnqueue.status != OVSTAGE_OK)
        return false;
    waitAndRelease(mInstance, queryEnqueue);

    ovstage_query_handle_t readQuery = query;
    ovstage_query_result_t queryResult{};
    if (ovstage_fetch_query_result(mInstance, query, OVSTAGE_TIMEOUT_INFINITE, &queryResult) != OVSTAGE_OK)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, query));
        return false;
    }
    if (queryResult.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
        readQuery = queryResult.all_handle;
    ovstage_release_query_result(mInstance, &queryResult);

    // Rendering-populated instance columns are latest-only, like the public
    // instancing topology queries used by buildPrototypeRootCache().
    ovstage_ordinal_range_t range{};
    range.end_ordinal = ~ovstage_ordinal_t(0);
    range.has_start_ordinal = false;

    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t readEnqueue =
        ovstage_read_attributes(mInstance, readQuery, &materialAttr, 1, range, &read);
    if (readEnqueue.status != OVSTAGE_OK)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, query));
        return false;
    }
    waitAndRelease(mInstance, readEnqueue);

    bool malformed = false;
    ovstage_read_group_t group{};
    ovstage_api_status_t fetchStatus = OVSTAGE_OK;
    while ((fetchStatus = ovstage_fetch_read_next(mInstance, read, OVSTAGE_TIMEOUT_INFINITE, &group)) == OVSTAGE_OK)
    {
        const bool usable = !group.is_delete && !group.is_array && group.data.mask == nullptr &&
                            group.data.tensor_count == 1 && group.data.tensors && group.data.tensors[0].data;
        if (!usable)
        {
            malformed = true;
            ovstage_release_group(mInstance, &group);
            continue;
        }

        const DLTensor& tensor = group.data.tensors[0];
        if (tensor.ndim > 0 && !tensor.shape)
        {
            malformed = true;
            ovstage_release_group(mInstance, &group);
            continue;
        }
        const int64_t storedRows = (tensor.ndim > 0 && tensor.shape && tensor.shape[0] > 0) ? tensor.shape[0] : 1;
        const int64_t elements = totalElements(tensor);
        if (tensor.dtype.code != kDLUInt || tensor.dtype.bits != 64 || tensor.dtype.lanes != 1 ||
            storedRows <= 0 || elements != storedRows)
        {
            malformed = true;
            ovstage_release_group(mInstance, &group);
            continue;
        }

        const ovx_primpath_t* primPaths = nullptr;
        size_t primPathCount = 0;
        if (ovx_path_dictionary_get_paths(mDict, group.prims.list, &primPaths, &primPathCount) != OVX_OK ||
            !primPaths)
        {
            malformed = true;
            ovstage_release_group(mInstance, &group);
            continue;
        }

        const uint64_t* values = reinterpret_cast<const uint64_t*>(
            static_cast<const uint8_t*>(tensor.data) + tensor.byte_offset);
        for (uint32_t row = 0; row < group.prims.count; ++row)
        {
            const uint32_t primIndex =
                group.prims.index_map ? group.prims.index_map[row] : (group.prims.offset + row);
            const uint32_t dataRow = group.data.index_map ? group.data.index_map[row] : row;
            if (primIndex >= primPathCount || dataRow >= static_cast<uint32_t>(storedRows))
            {
                malformed = true;
                continue;
            }

            const uint64_t prim = primPaths[primIndex];
            const uint64_t material = values[dataRow];
            if (!prim || !material)
                continue;
            mInstanceMaterialByPrim[prim] = material;
            const uint64_t canonical = canonicalHandle(ObjectKey{ prim });
            if (canonical && canonical != prim)
                mInstanceMaterialByPrim[canonical] = material;
        }
        ovstage_release_group(mInstance, &group);
    }

    waitAndRelease(mInstance, ovstage_release_read(mInstance, read));
    waitAndRelease(mInstance, ovstage_release_query(mInstance, query));
    if (fetchStatus != OVSTAGE_ERROR_END_OF_ITERATION || malformed)
    {
        mInstanceMaterialByPrim.clear();
        return false;
    }

    mInstanceMaterialCacheValid = true;
    return true;
}

ObjectKey OvstageSource::geometryBackingKey(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !buildPhysicsInstancingCache())
        return key;

    const uint64_t canonical = canonicalHandle(key);
    const uint64_t cacheKey = canonical ? canonical : key.handle;
    std::unordered_map<uint64_t, uint64_t>::const_iterator cached = mGeometryBackingCache.find(cacheKey);
    if (cached != mGeometryBackingCache.end())
        return ObjectKey{ cached->second };

    const std::string logicalPath = pathOf(key);
    if (logicalPath.empty())
        return key;

    std::string ancestor = logicalPath;
    ObjectKey backingKey = key;
    bool lookupComplete = true;
    while (!ancestor.empty())
    {
        std::unordered_map<std::string, std::string>::const_iterator root =
            mPrototypeRootByInstanceRoot.find(ancestor);

        // The attach walk only expands prototypes that can back collision
        // shapes. Public type/geometry helpers must still resolve a render-only
        // instance proxy, so after the load walk lazily use OVStage's reverse
        // lookup for map misses. The current provider rebuilds the complete
        // instancing graph per call. Keeping this outside attach avoids paying
        // that cost for every render prototype, and a successful result is
        // cached so each queried proxy pays its ancestor probes only once.
        // OMPE-100947 tracks the batched API.
        if (root == mPrototypeRootByInstanceRoot.end() && !mLoadCacheActive &&
            !mPrototypeRootPaths.empty())
        {
            const ObjectKey instanceRoot = findByPath(ancestor);
            if (!instanceRoot.valid())
            {
                lookupComplete = false;
                break;
            }

            ovx_primpath_t prototypeRoot = OVX_INVALID_PRIMPATH;
            prototypeRootQueryCounter().fetch_add(1, std::memory_order_relaxed);
            const ovstage_api_status_t status =
                ovstage_instancing_get_prototype_root(mInstance, instanceRoot.handle, &prototypeRoot);
            if (status == OVSTAGE_OK)
            {
                const std::string prototypePath = pathOf(ObjectKey{ prototypeRoot });
                if (prototypePath.empty())
                {
                    lookupComplete = false;
                    break;
                }
                root = mPrototypeRootByInstanceRoot.emplace(ancestor, prototypePath).first;
            }
            else if (status != OVSTAGE_ERROR_NOT_FOUND)
            {
                lookupComplete = false;
                break;
            }
        }

        if (root != mPrototypeRootByInstanceRoot.end())
        {
            const std::string backingPath = root->second + logicalPath.substr(ancestor.size());
            const ObjectKey resolved = findByPath(backingPath);
            if (resolved.valid())
                backingKey = resolved;
            else
                lookupComplete = false;
            break;
        }

        const size_t slash = ancestor.rfind('/');
        if (slash == std::string::npos || slash == 0)
            break;
        ancestor.resize(slash);
    }

    // The load walk calls this resolver only for physics-scoped keys, so its
    // identity results are authoritative and must remain cached for later
    // cooking. A post-load reverse-query error is not authoritative and is
    // retried instead of becoming a negative cache entry.
    if (backingKey != key || lookupComplete)
    {
        mGeometryBackingCache[cacheKey] = backingKey.handle;
        if (key.handle != cacheKey)
            mGeometryBackingCache[key.handle] = backingKey.handle;
    }
    return backingKey;
}

bool OvstageSource::isPrototypeBackingKey(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !buildPrototypeRootCache() || mPrototypeRootPaths.empty())
        return false;

    std::string ancestor = pathOf(key);
    while (!ancestor.empty())
    {
        if (mPrototypeRootPaths.count(ancestor) != 0)
            return true;
        const size_t slash = ancestor.rfind('/');
        if (slash == std::string::npos || slash == 0)
            break;
        ancestor.resize(slash);
    }
    return false;
}

std::string OvstageSource::pathOf(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mDict || !key.valid())
        return {};
    const auto cached = mPathStringCache.find(key.handle);
    if (cached != mPathStringCache.end())
        return cached->second;

    ovx_string_t s{};
    if (ovx_path_dictionary_path_to_string(mDict, key.handle, &s) != OVX_OK || !s.ptr)
        return {};
    std::string path(s.ptr, s.length);
    mPathStringCache[key.handle] = path;
    return path;
}

std::string_view OvstageSource::sourceKeyToString(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    // Intern the resolved path string in our own table so the returned view has
    // stable lifetime (the dictionary's string view is also stable, but routing
    // through internToken keeps one ownership story).
    const std::string p = pathOf(key);
    if (p.empty())
        return {};
    return tokenToString(doInternToken(p));
}

// --- traversal -------------------------------------------------------------

ObjectKey OvstageSource::findByPath(std::string_view path) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mDict || path.empty())
        return {};
    const std::string key(path);
    const auto cached = mPathToHandleCache.find(key);
    if (cached != mPathToHandleCache.end())
        return ObjectKey{ cached->second };

    ovx_primpath_t p = OVX_INVALID_PRIMPATH;
    const ovx_string_t str{ key.data(), key.size() };
    if (ovx_path_dictionary_intern_path(mDict, str, &p) != OVX_OK)
        return {};
    mPathToHandleCache.emplace(key, p);
    if (p != OVX_INVALID_PRIMPATH)
        mPathStringCache.emplace(p, key);
    return ObjectKey{ p };
}

ObjectKey OvstageSource::getRootKey() const
{
    return findByPath("/");
}

ObjectKey OvstageSource::getParent(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid())
        return {};
    const auto cached = mParentHandleCache.find(key.handle);
    if (cached != mParentHandleCache.end())
        return ObjectKey{ cached->second };

    const std::string p = pathOf(key);
    if (p.size() <= 1)
    {
        mParentHandleCache[key.handle] = 0;
        return {}; // "/" or empty has no parent
    }
    const size_t slash = p.rfind('/');
    if (slash == std::string::npos)
    {
        mParentHandleCache[key.handle] = 0;
        return {};
    }
    const std::string parent = (slash == 0) ? "/" : p.substr(0, slash);
    const ObjectKey parentKey = findByPath(parent);
    mParentHandleCache[key.handle] = parentKey.handle;
    const uint64_t canonical = canonicalHandle(key);
    if (canonical && canonical != key.handle)
        mParentHandleCache[canonical] = parentKey.handle;
    return parentKey;
}

void OvstageSource::buildChildCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mChildCacheBuilt)
        return;
    mChildCache.clear();
    mDescendantCache.clear();
    mChildCacheBuilt = true;
    mChildCacheComplete = false;

    if (!mInstance || !mDict)
        return;

    const ovx_string_t prefixVal{ "/", 1 };
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() };
    pred.op = OVSTAGE_FILTER_OP_PREFIX;
    pred.values = &prefixVal;
    pred.value_count = 1;
    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
        return;
    waitAndRelease(mInstance, e);

    ovstage_query_handle_t use = q;
    std::vector<ovx_token_t> dataProbes;
    std::vector<ovx_token_t> metadataProbes;
    std::unordered_set<ovx_token_t> probeSet;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        for (size_t i = 0; i < qr.attribute_count; ++i)
        {
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(mDict, qr.attributes[i], &s) != OVX_OK || !s.ptr || s.length == 0)
                continue;
            if (s.ptr[0] == '_')
                continue;
            if (s.length >= 4 && std::strncmp(s.ptr, "usd-", 4) == 0)
                continue;
            if (probeSet.insert(qr.attributes[i]).second)
                dataProbes.push_back(qr.attributes[i]);
        }
        ovstage_release_query_result(mInstance, &qr);
    }

    for (const char* metadataProbe : { conv::kUsdPath, conv::kUsdParent, conv::kUsdChildren })
    {
        ovx_token_t probe = OVX_INVALID_TOKEN;
        ovx_path_dictionary_intern_token(mDict, ovx_string_t{ metadataProbe, std::string_view(metadataProbe).size() }, &probe);
        if (probe != OVX_INVALID_TOKEN && probeSet.insert(probe).second)
            metadataProbes.push_back(probe);
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = mReadOrdinal;
    range.has_start_ordinal = false;

    // Dedup edges: the usd-children/usd-parent column builders and the usd-path
    // derivation below all run and can rediscover the same parent/child edge, so
    // guard against pushing a child twice under one parent.
    std::unordered_set<uint64_t> insertedEdges;
    auto insertChild = [&](uint64_t parent, uint64_t child)
    {
        if (!parent || !child || parent == child)
            return;
        const uint64_t edgeKey = (parent * 0x9E3779B97F4A7C15ull) ^ child;
        if (!insertedEdges.insert(edgeKey).second)
            return;
        mChildCache[parent].push_back(child);
        mParentHandleCache[child] = parent;
    };

    auto tryBuildFromUsdChildren = [&]() -> bool
    {
        ovx_token_t childrenProbe = OVX_INVALID_TOKEN;
        ovx_path_dictionary_intern_token(mDict, ovx_string_t{ conv::kUsdChildren, std::string_view(conv::kUsdChildren).size() },
                                         &childrenProbe);
        if (childrenProbe == OVX_INVALID_TOKEN)
            return false;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &childrenProbe, 1, range, &rh);
        if (re.status != OVSTAGE_OK)
            return false;

        waitAndRelease(mInstance, re);
        bool foundAny = false;
        ovstage_read_group_t g{};
        while (ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
        {
            if (g.is_delete || !g.is_array || g.data.tensor_count == 0 || !g.data.tensors || g.data.mask)
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }

            const ovx_primpath_t* parentPaths = nullptr;
            size_t parentCount = 0;
            if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &parentPaths, &parentCount) != OVX_OK ||
                !parentPaths)
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }

            for (uint32_t row = 0; row < g.prims.count; ++row)
            {
                const uint32_t parentIdx = g.prims.index_map ? g.prims.index_map[row] : (g.prims.offset + row);
                if (parentIdx >= parentCount)
                    continue;
                const uint32_t tensorIndex = g.data.index_map ? g.data.index_map[row] : row;
                if (tensorIndex >= g.data.tensor_count)
                    continue;
                const DLTensor& t = g.data.tensors[tensorIndex];
                if (!canDecodeRelationshipTargets(t))
                    continue;
                const int64_t n = totalElements(t);
                const uint8_t* base = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                const uint64_t* children = reinterpret_cast<const uint64_t*>(base);
                const uint64_t parent = parentPaths[parentIdx];
                for (int64_t i = 0; i < n; ++i)
                {
                    const uint64_t child = children[i];
                    if (child == OVX_INVALID_PRIMPATH)
                        continue;
                    insertChild(parent, child);
                    foundAny = true;
                }
            }
            ovstage_release_group(mInstance, &g);
        }
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        return foundAny;
    };

    auto tryBuildFromUsdParent = [&]() -> bool
    {
        ovx_token_t parentProbe = OVX_INVALID_TOKEN;
        ovx_path_dictionary_intern_token(mDict, ovx_string_t{ conv::kUsdParent, std::string_view(conv::kUsdParent).size() },
                                         &parentProbe);
        if (parentProbe == OVX_INVALID_TOKEN)
            return false;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &parentProbe, 1, range, &rh);
        if (re.status != OVSTAGE_OK)
            return false;

        waitAndRelease(mInstance, re);
        bool foundAny = false;
        ovstage_read_group_t g{};
        while (ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
        {
            if (g.is_delete || g.is_array || g.data.tensor_count == 0 || !g.data.tensors || g.data.mask)
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }

            const DLTensor& t = g.data.tensors[0];
            if (!canDecodeRelationshipTargets(t))
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }

            const ovx_primpath_t* childPaths = nullptr;
            size_t childCount = 0;
            if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &childPaths, &childCount) != OVX_OK ||
                !childPaths)
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }

            const int64_t total = totalElements(t);
            int64_t storedRows = g.prims.count;
            if (g.data.index_map)
            {
                storedRows = 0;
                for (uint32_t i = 0; i < g.prims.count; ++i)
                    storedRows = std::max<int64_t>(storedRows, static_cast<int64_t>(g.data.index_map[i]) + 1);
            }
            if (total < 0 || storedRows <= 0 || (total % storedRows) != 0)
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }
            const int64_t comps = total / storedRows;
            if (comps <= 0)
            {
                ovstage_release_group(mInstance, &g);
                continue;
            }

            const uint8_t* base = static_cast<const uint8_t*>(t.data) + t.byte_offset;
            for (uint32_t row = 0; row < g.prims.count; ++row)
            {
                const uint32_t childIdx = g.prims.index_map ? g.prims.index_map[row] : (g.prims.offset + row);
                if (childIdx >= childCount)
                    continue;
                const uint32_t dataRow = g.data.index_map ? g.data.index_map[row] : row;
                if (dataRow >= static_cast<uint32_t>(storedRows))
                    continue;
                const uint64_t parent = reinterpret_cast<const uint64_t*>(base + dataRow * comps * sizeof(uint64_t))[0];
                const uint64_t child = childPaths[childIdx];
                insertChild(parent, child);
                foundAny = true;
            }
            ovstage_release_group(mInstance, &g);
        }
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        return foundAny;
    };

    // usd-children / usd-parent are only emitted for some prims by some
    // populators (e.g. the internal stage-info prims), so on their own they
    // yield at most a partial edge set. Use them as a supplement, then ALWAYS
    // fall through to the usd-path prefix derivation below: usd-path is
    // populated for every queryable row, so it is the authoritative builder that
    // connects the rest of the hierarchy (e.g. "/" -> "/World" -> descendants).
    tryBuildFromUsdChildren();
    tryBuildFromUsdParent();

    // The usd-path column is populated for every queryable row, so its read is the
    // authoritative whole-stage enumeration. Track whether it terminates cleanly
    // (ends exactly at OVSTAGE_ERROR_END_OF_ITERATION) so a transient/partial read
    // is not mistaken for a complete edge set below.
    ovx_token_t usdPathProbe = OVX_INVALID_TOKEN;
    ovx_path_dictionary_intern_token(
        mDict, ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() }, &usdPathProbe);

    std::unordered_set<ovx_primpath_t> allPaths;
    bool authoritativeReadClean = false;
    auto readPathProbes = [&](const std::vector<ovx_token_t>& probes)
    {
        for (const ovx_token_t probe : probes)
        {
            const bool isAuthoritative = (usdPathProbe != OVX_INVALID_TOKEN && probe == usdPathProbe);
            ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
            const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &probe, 1, range, &rh);
            if (re.status != OVSTAGE_OK)
                continue;

            waitAndRelease(mInstance, re);
            ovstage_read_group_t g{};
            ovstage_api_status_t fetchStatus = OVSTAGE_OK;
            while ((fetchStatus = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
            {
                const ovx_primpath_t* paths = nullptr;
                size_t count = 0;
                if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &paths, &count) == OVX_OK && paths)
                {
                    for (uint32_t i = 0; i < g.prims.count; ++i)
                    {
                        const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                        if (idx < count)
                            allPaths.insert(paths[idx]);
                    }
                }
                ovstage_release_group(mInstance, &g);
            }
            // A clean pass ends exactly at END_OF_ITERATION; any other terminal
            // status means the enumeration was cut short (transient/partial read).
            // The test hook forces the not-clean path for deterministic coverage.
            if (isAuthoritative && fetchStatus == OVSTAGE_ERROR_END_OF_ITERATION &&
                !consumeAuthoritativeReadFaultForTest())
                authoritativeReadClean = true;
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
    };
    readPathProbes(metadataProbes);
    if (allPaths.empty())
        readPathProbes(dataProbes);
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));

    // allPaths is a hash set, so iterating it directly would derive the supplemental
    // ancestor edges (and therefore forEachChild()'s published child order) in a
    // nondeterministic hash order. Resolve the leaf paths and sort them lexicographically
    // so this fallback edge set is stable across runs. Edges already contributed in
    // authoritative usd-children order by tryBuildFromUsdChildren() are deduped by
    // insertChild(), so this sort only orders the edges the fallback itself supplies.
    std::vector<std::string> leafPaths;
    leafPaths.reserve(allPaths.size());
    for (const ovx_primpath_t pathHandle : allPaths)
    {
        std::string leafPath = pathOf(ObjectKey{ pathHandle });
        if (leafPath.empty() || leafPath == "/")
            continue;
        leafPaths.push_back(std::move(leafPath));
    }
    std::sort(leafPaths.begin(), leafPaths.end());

    for (const std::string& leafPath : leafPaths)
    {
        // Walk the FULL ancestor chain by usd-path string, inserting an edge at
        // every level. Intermediate ancestors (e.g. a typeless "/World"
        // container) are not always authored by ovpopulation as their own
        // queryable rows, so they never appear in allPaths directly — but they
        // are interneable by path. Synthesizing the chain here is what connects
        // "/" -> "/World" -> the populated descendants; deriving only each
        // prim's immediate parent would leave the typeless level dangling and
        // break the root-anchored descent (forEachDescendantPruned).
        std::string childPath = leafPath;
        while (childPath != "/" && !childPath.empty())
        {
            const size_t slash = childPath.rfind('/');
            if (slash == std::string::npos)
                break;
            const std::string parentPath = (slash == 0) ? std::string("/") : childPath.substr(0, slash);
            const uint64_t childCanonical = canonicalHandle(findByPath(childPath));
            const uint64_t parentCanonical = canonicalHandle(findByPath(parentPath));
            if (parentCanonical && childCanonical)
                insertChild(parentCanonical, childCanonical);
            childPath = parentPath;
        }
    }

    // Trust the negative-leaf fast path in forEachChild() only when the
    // authoritative usd-path enumeration completed cleanly (terminated at
    // END_OF_ITERATION) and produced rows. A transient or partial read must not
    // become a permanently authoritative negative cache; leaving this false
    // makes forEachChild() fall back to the live
    // prefix query for cache misses, and lets a later build retry.
    mChildCacheComplete = authoritativeReadClean && !allPaths.empty();
}

void OvstageSource::collectDescendantKeys(ObjectKey root, std::vector<ObjectKey>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    out.clear();
    if (!root.valid() || !mInstance || !mDict)
        return;

    if (!mChildCacheBuilt)
        buildChildCache();

    uint64_t rootHandle = root.handle;
    if (mChildCache.find(rootHandle) == mChildCache.end())
        rootHandle = canonicalHandle(root);
    if (!rootHandle)
        return;

    // Only serve from / write to the descendant memo when the child cache built
    // cleanly. A partial/transient build must not memoize (or return) an
    // incomplete descendant set that would then persist across a later retry.
    if (mChildCacheComplete)
    {
        const auto cached = mDescendantCache.find(rootHandle);
        if (cached != mDescendantCache.end())
        {
            out.reserve(cached->second.size());
            for (const uint64_t handle : cached->second)
                out.push_back(ObjectKey{ handle });
            return;
        }
    }

    std::vector<uint64_t> handles;
    std::vector<uint64_t> stack;
    std::unordered_set<uint64_t> seen;
    handles.reserve(32);
    stack.push_back(rootHandle);
    seen.insert(rootHandle);

    while (!stack.empty())
    {
        const uint64_t current = stack.back();
        stack.pop_back();
        handles.push_back(current);

        const auto it = mChildCache.find(current);
        if (it == mChildCache.end())
            continue;

        for (auto childIt = it->second.rbegin(); childIt != it->second.rend(); ++childIt)
        {
            const uint64_t child = *childIt;
            if (child && seen.insert(child).second)
                stack.push_back(child);
        }
    }

    out.reserve(handles.size());
    for (const uint64_t handle : handles)
        out.push_back(ObjectKey{ handle });

    // Memoize only a cleanly-built traversal; see the note above.
    if (mChildCacheComplete)
        mDescendantCache[rootHandle] = std::move(handles);
}

void OvstageSource::forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!parent.valid() || !cb)
        return;
    const std::string parentPath = pathOf(parent);
    if (parentPath.empty())
        return;

    std::unordered_set<uint64_t> emittedChildren;
    auto emitChild = [&](ObjectKey child)
    {
        if (!child.valid())
            return;
        const uint64_t childHandle = canonicalHandle(child);
        if (childHandle && !emittedChildren.insert(childHandle).second)
            return;
        cb(child);
    };

    if (mInstance && mDict)
    {
        if (!mChildCacheBuilt)
            buildChildCache();
        uint64_t parentCanonical = parent.handle;
        if (mChildCache.find(parentCanonical) == mChildCache.end())
            parentCanonical = canonicalHandle(parent);
        std::unordered_map<uint64_t, std::vector<uint64_t>>::const_iterator it = mChildCache.find(parentCanonical);
        if (it != mChildCache.end())
        {
            for (const uint64_t child : it->second)
                emitChild(ObjectKey{ child });
            // A cleanly-built cache is authoritative — its child list is complete,
            // so return without the prefix query. A partial/transient build is not:
            // fall through to merge these cached children with the live query
            // (emitChild dedups) so a child the build missed is still observed
            // (fail-open on a partial/transient build).
            if (mChildCacheComplete)
                return;
        }

        // buildChildCache() derived the COMPLETE edge set from a single
        // whole-stage query. A prim it knows about (present as some edge's child,
        // i.e. in mParentHandleCache) but absent from mChildCache genuinely has no
        // children — it is a leaf. Return empty instead of falling through to the
        // per-call prefix query below: that fallback is a full-stage scan, and
        // firing it once per leaf made whole-stage scans O(prims^2) (the dominant
        // cost of ovstage attach on large scenes).
        //
        // Restrict this negative fast path to the initial load walk
        // (mLoadCacheActive): that window covers the expensive scan, but the
        // hierarchy caches are NOT invalidated by post-attach structural edits
        // (reconcileStructuralChanges only clears the schema cache). Outside the
        // load window a prim that was a leaf at attach time may since have gained a
        // child, so fall through to the live prefix query to observe it.
        // mChildCacheComplete additionally guards against a transient/partial
        // cache build being treated as authoritative.
        // Only prims the bulk build never saw (parentCanonical unknown) fall
        // through even during load.
        if (mLoadCacheActive && mChildCacheComplete && parentCanonical &&
            mParentHandleCache.find(parentCanonical) != mParentHandleCache.end())
        {
            return;
        }

        // The ovstage-wide cache has no children for this prim; fall through to
        // the usd-path prefix query below to enumerate them natively.
    }

    if (!mInstance || !mDict)
        return;

    // Match the subtree by usd-path PREFIX (parentPath + '/'), then keep only direct
    // children (no further '/' in the remainder). usd-path is always populated;
    // usd-parent is not maintained by every populator, so prefix matching is the
    // robust route.
    const std::string prefix = (parentPath == "/") ? std::string("/") : (parentPath + "/");
    const ovx_string_t prefixVal{ prefix.data(), prefix.size() };
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() };
    pred.op = OVSTAGE_FILTER_OP_PREFIX;
    pred.values = &prefixVal;
    pred.value_count = 1;
    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
        return;
    waitAndRelease(mInstance, e);

    // Query results expose a union of discovered attributes. No single data column
    // is guaranteed to be present on every child in a heterogeneous subtree, so
    // materialize through all readable columns and union the path lists.
    ovstage_query_handle_t use = q;
    std::vector<ovx_token_t> probes;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        for (size_t i = 0; i < qr.attribute_count; ++i)
        {
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(mDict, qr.attributes[i], &s) != OVX_OK || !s.ptr || s.length == 0)
                continue;
            if (s.ptr[0] == '_' || (s.length >= 4 && std::strncmp(s.ptr, "usd-", 4) == 0))
                continue;
            probes.push_back(qr.attributes[i]);
        }
        ovstage_release_query_result(mInstance, &qr);
    }
    if (probes.empty())
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = ~ovstage_ordinal_t(0); // latest
    range.has_start_ordinal = false;

    for (const ovx_token_t probe : probes)
    {
        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &probe, 1, range, &rh);
        if (re.status != OVSTAGE_OK)
            continue;

        waitAndRelease(mInstance, re);
        ovstage_read_group_t g{};
        while (ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
        {
            const ovx_primpath_t* paths = nullptr;
            size_t count = 0;
            if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &paths, &count) == OVX_OK && paths)
            {
                for (uint32_t i = 0; i < g.prims.count; ++i)
                {
                    const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                    if (idx >= count)
                        continue;
                    const ObjectKey childKey{ paths[idx] };
                    const std::string childPath = pathOf(childKey);
                    if (childPath.size() > prefix.size() &&
                        childPath.compare(0, prefix.size(), prefix) == 0 &&
                        childPath.find('/', prefix.size()) == std::string::npos)
                    {
                        emitChild(childKey);
                    }
                }
            }
            ovstage_release_group(mInstance, &g);
        }
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    }
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
}

bool OvstageSource::exists(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid())
        return false;

    // During change-feed dispatch the source bucket is seeded from the same
    // ovstage read group that produced the changed keys. Membership there is
    // enough to answer the runtime "is this changed prim still live?" guard
    // without issuing a per-key usd-path query.
    if (mBucketKeys.count(key.handle) != 0)
    {
        return true;
    }
    if (mKnownKeys.count(key.handle) != 0)
    {
        return true;
    }
    if (const uint64_t canonical = canonicalHandle(key))
    {
        if (mBucketKeys.count(canonical) != 0)
        {
            return true;
        }
        if (mKnownKeys.count(canonical) != 0)
        {
            return true;
        }
    }

    const std::string path = pathOf(key);
    if (path.empty())
        return false;


    // A key resolves to a live prim iff a scoped `usd-path IN [path]` query
    // matches it. (findByPath/pathOf only intern/round-trip the path string —
    // they succeed for any well-formed path, present or not — so they can't gate
    // existence.) Used by the consumer to skip absent prims (replaces the USD
    // `prim != null` guard); the default IPhysicsSource::exists returns false.
    if (!mInstance || !mDict)
        return false;

    const ovx_string_t pathVal{ path.data(), path.size() };
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() };
    pred.op = OVSTAGE_FILTER_OP_IN;
    pred.values = &pathVal;
    pred.value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
    {
        return false;
    }
    waitAndRelease(mInstance, e);

    size_t count = 0;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        count = qr.total_prim_count;
        ovstage_release_query_result(mInstance, &qr);
    }
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    return count >= 1;
}

// --- read core -------------------------------------------------------------

bool OvstageSource::withAttributeTensor(ObjectKey key,
                                        std::string_view attrName,
                                        const std::function<void(const DLTensor&, uint32_t)>& fn) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mInstance || !mDict || !key.valid())
        return false;

    // Canonicalize the handle to the intern_path space. Scan-emitted ObjectKeys
    // carry ENUMERATE (get_paths) handles, while writes (ovstage API, runtime
    // change drivers) land on the prim's canonical intern_path handle. A raw
    // enumerate-handle read misses those writes, so a value edit applied after the
    // scan would never be seen (the bucket fast path already canonicalizes for the
    // same reason). Round-tripping through canonicalHandle keeps both in one space.
    const uint64_t canon = canonicalHandle(key);
    ovx_primpath_t p = canon ? canon : key.handle;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(mDict, &p, 1, &list) != OVX_OK)
        return false;

    bool got = false;
    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(mInstance, list, &q) == OVSTAGE_OK && q != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ovx_token_t attrTok = ovxToken(attrName);
        // Read the LATEST value: has_start_ordinal=false means "most recent value
        // with ordinal <= end_ordinal" (end_ordinal at the max), so it always
        // resolves current state. Fabric-only attributes have no ovstage ordinal
        // history, so an exact [start,end] range read misses them but a "latest"
        // read finds them; ovstage-API-written physics attributes work the same
        // way either way. (Mirrors the ovstage population_domains "latest" reads.)
        ovstage_ordinal_range_t range{};
        range.end_ordinal = ~ovstage_ordinal_t(0);
        range.has_start_ordinal = false;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, q, &attrTok, 1, range, &rh);
        if (re.status == OVSTAGE_OK)
        {
            ovstage_wait_op(mInstance, re.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
            ovstage_release_op(mInstance, re.op_index);

            ovstage_read_group_t g{};
            while (ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
            {
                if (!got && !g.is_delete && g.data.tensor_count > 0 && g.data.tensors && g.data.tensors[0].data)
                {
                    fn(g.data.tensors[0], g.prims.count);
                    got = true;
                }
                ovstage_release_group(mInstance, &g);
            }
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    }

    ovx_path_dictionary_destroy_path_list(mDict, list);
    return got;
}

// --- attributes ------------------------------------------------------------

AttrValue OvstageSource::getAttribute(ObjectKey key, TokenId attr) const
{
    return getAttributeAtTime(key, attr, ReadTime::defaultTime());
}

AttrValue OvstageSource::getAttributeAtTime(ObjectKey key, TokenId attr, ReadTime time) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const std::string_view name = tokenToString(attr);
    if (name.empty())
        return {};

    auto fallbackToUsd = [&]() -> AttrValue
    {
        PXR_NS::UsdStageRefPtr stage = usdStageFromId(mUsdStageId);
        if (!stage || !key.valid())
            return {};
        const std::string path = pathOf(key);
        if (path.empty())
            return {};
        PXR_NS::UsdPrim prim = stage->GetPrimAtPath(PXR_NS::SdfPath(path));
        if (!prim)
            return {};

        const PXR_NS::TfToken token{ std::string(name) };
        const PXR_NS::UsdTimeCode timeCode = usdTimeCode(time);
        auto interner = [this](const PXR_NS::TfToken& t) { return doInternToken(t.GetString()); };

        PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(token);
        if (usdAttr && usdAttributeHasValueType(usdAttr, timeCode) && usdAttr.HasValue())
        {
            PXR_NS::VtValue val;
            if (usdAttr.Get(&val, timeCode))
            {
                AttrValue out = vtValueToAttrValue(val, interner);
                if (out.valid())
                    return out;
            }
        }

        PXR_NS::VtValue meta;
        meta = prim.GetCustomDataByKey(token);
        if (!meta.IsEmpty())
            return vtValueToAttrValue(meta, interner);
        if (prim.GetMetadata(token, &meta) && !meta.IsEmpty())
            return vtValueToAttrValue(meta, interner);
        return {};
    };

    if (mLoadCacheActive && key.valid())
    {
        auto lookupLoadValue = [&](uint64_t handle) -> const CachedTensorRow*
        {
            const auto pit = mLoadCacheScalars.find(handle);
            if (pit == mLoadCacheScalars.end())
                return nullptr;
            const auto ait = pit->second.find(attr.id);
            return ait != pit->second.end() ? &ait->second : nullptr;
        };
        auto loadCacheCovers = [&](uint64_t handle) -> bool
        {
            const auto cit = mLoadCacheCoveredAttrs.find(handle);
            return cit != mLoadCacheCoveredAttrs.end() && cit->second.count(attr.id) != 0;
        };
        enum class LoadCacheLookup
        {
            eUnknown,
            eValue,
            eCoveredMiss,
        };
        auto tryHandle = [&](uint64_t handle, AttrValue& out) -> LoadCacheLookup
        {
            if (const CachedTensorRow* row = lookupLoadValue(handle))
            {
                if (row->tensor && row->tensor->data)
                {
                    out = decodeAt(*row->tensor, row->comps, row->row);
                    if (out.valid())
                        return LoadCacheLookup::eValue;
                }
            }
            if (loadCacheCovers(handle))
            {
                return LoadCacheLookup::eCoveredMiss;
            }
            return LoadCacheLookup::eUnknown;
        };

        AttrValue cached;
        const uint64_t raw = key.handle;
        if (raw)
        {
            const LoadCacheLookup result = tryHandle(raw, cached);
            if (result == LoadCacheLookup::eValue)
                return cached;
            if (result == LoadCacheLookup::eCoveredMiss && mUsdStageId == 0)
                return {};
        }

        const uint64_t canonical = canonicalHandle(key);
        if (canonical && canonical != raw)
        {
            const LoadCacheLookup result = tryHandle(canonical, cached);
            if (result == LoadCacheLookup::eValue)
                return cached;
            if (result == LoadCacheLookup::eCoveredMiss && mUsdStageId == 0)
                return {};
        }
    }

    // Bucket fast path: serve from the columnar cache when this prim is in the
    // active bucket. Change-feed buckets keep the live read group and materialize
    // only requested rows; prefetch buckets are already decoded.
    if (mBucketActive && key.valid())
    {
        auto lookupBucketValue = [&](uint64_t handle) -> const AttrValue*
        {
            const auto pit = mBucketScalars.find(handle);
            if (pit == mBucketScalars.end())
                return nullptr;
            const auto ait = pit->second.find(attr.id);
            return ait != pit->second.end() ? &ait->second : nullptr;
        };
        auto materializeBucketValue = [&](uint64_t handle) -> const AttrValue*
        {
            if (!handle || attr != mBucketReadGroupAttr || !mBucketReadGroupTensor)
                return nullptr;
            const auto rit = mBucketRows.find(handle);
            if (rit == mBucketRows.end())
                return nullptr;
            AttrValue v = decodeAt(*mBucketReadGroupTensor, mBucketReadGroupComps, rit->second);
            if (!v.valid())
                return nullptr;
            auto& slot = mBucketScalars[handle][attr.id];
            slot = v;
            return &slot;
        };

        bool inBucket = false;
        const bool attrCovered = mBucketAttributeIds.count(attr.id) != 0;
        const uint64_t raw = key.handle;
        if (raw)
        {
            inBucket = mBucketKeys.count(raw) != 0;
            if (const AttrValue* v = lookupBucketValue(raw))
                return *v;
            if (const AttrValue* v = materializeBucketValue(raw))
                return *v;
        }
        if (!inBucket)
        {
            const uint64_t ch = canonicalHandle(key);
            if (ch && ch != raw)
            {
                inBucket = mBucketKeys.count(ch) != 0;
                if (const AttrValue* v = lookupBucketValue(ch))
                    return *v;
                if (const AttrValue* v = materializeBucketValue(ch))
                    return *v;
            }
        }
        if (inBucket && attrCovered && mBucketScalarsComplete && mUsdStageId == 0)
            return {};
    }

    AttrValue out;
    withAttributeTensor(key, name, [&](const DLTensor& t, uint32_t primCount) { out = decodeScalar(t, primCount); });
    return out.valid() ? out : fallbackToUsd();
}

bool OvstageSource::getAttribute(ObjectKey key, TokenId attr, TokenId& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const AttrValue v = getAttribute(key, attr);
    if (v.kind == AttrValue::Kind::eToken)
    {
        out = v.tok;
        return true;
    }
    // Token attributes are stored as a uint64 token-id column (decoded as eInt).
    // Resolve the id to its string through the dictionary and re-intern into the
    // source token space so the returned TokenId compares against internToken(...).
    if (v.kind == AttrValue::Kind::eInt && mDict)
    {
        ovx_string_t s{};
        if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(v.i), &s) == OVX_OK && s.ptr && s.length)
        {
            out = doInternToken(std::string_view(s.ptr, s.length));
            return true;
        }
    }
    return false;
}

// --- columnar bulk read ----------------------------------------------------

void OvstageSource::clearBucket() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mBucketActive = false;
    mBucketScalarsComplete = true;
    mBucketTransformsComplete = true;
    mBucketKeys.clear();
    mBucketRows.clear();
    mBucketAttributeIds.clear();
    mBucketScalars.clear();
    mBucketWorldTransforms.clear();
    mBucketLocalTransforms.clear();
    mBucketResetXformStack.clear();
    mBucketReadGroupAttr = {};
    mBucketReadGroupTensor = nullptr;
    mBucketReadGroupComps = 0;
}

void OvstageSource::beginLoadCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    for (ovstage_read_group_t& group : mLoadCacheGroups)
        ovstage_release_group(mInstance, &group);
    mLoadCacheActive = true;
    mLoadCacheGroups.clear();
    mLoadCacheCoveredAttrs.clear();
    mLoadCacheScalars.clear();
    mLoadCacheRelationships.clear();
    mLoadCacheWorldTransforms.clear();
    mLoadCacheLocalTransforms.clear();
    mLoadCacheResetXformStack.clear();
    mLoadCacheComposedWorld.clear();
}

void OvstageSource::clearLoadCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    for (ovstage_read_group_t& group : mLoadCacheGroups)
        ovstage_release_group(mInstance, &group);
    mLoadCacheActive = false;
    mLoadCacheGroups.clear();
    mLoadCacheCoveredAttrs.clear();
    mLoadCacheScalars.clear();
    mLoadCacheRelationships.clear();
    mLoadCacheWorldTransforms.clear();
    mLoadCacheLocalTransforms.clear();
    mLoadCacheResetXformStack.clear();
    mLoadCacheComposedWorld.clear();
}

void OvstageSource::seedKnownKeys(const std::vector<ObjectKey>& keys) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (keys.empty())
        return;

    mKnownKeys.reserve(mKnownKeys.size() + keys.size() * 2);
    for (const ObjectKey key : keys)
    {
        if (!key.valid())
            continue;

        mKnownKeys.insert(key.handle);
        if (const uint64_t canonical = canonicalHandle(key))
            mKnownKeys.insert(canonical);
    }
}

void OvstageSource::clearKnownKeys() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mKnownKeys.clear();
}

void OvstageSource::clearSchemaCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mPrototypeRootCacheInitialized = false;
    mPrototypeRootCacheValid = false;
    mPhysicsInstancingCacheInitialized = false;
    mPhysicsInstancingCacheValid = false;
    mPrototypeRootPaths.clear();
    mPrototypeRootByInstanceRoot.clear();
    mGeometryBackingCache.clear();
    mInstanceMaterialCacheBuilt = false;
    mInstanceMaterialCacheValid = false;
    mInstanceMaterialByPrim.clear();
    mSchemaCacheBuilt = false;
    mSchemaCacheComplete = false;
    mSchemaMayExistCache.clear();
    mSchemaMembershipCache.clear();
    mSchemasByPrimCache.clear();
    mMultiApplyMembershipCache.clear();
    mMultiApplyInstancesByPrimCache.clear();
}

bool OvstageSource::buildSchemaCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mSchemaCacheBuilt)
        return mSchemaCacheComplete;

    mSchemaCacheBuilt = true;
    mSchemaCacheComplete = false;
    mSchemaMayExistCache.clear();
    mSchemaMembershipCache.clear();
    mSchemasByPrimCache.clear();
    mMultiApplyMembershipCache.clear();
    mMultiApplyInstancesByPrimCache.clear();

    if (!mInstance || !mDict)
        return false;

    const ovx_string_t pathVal{ "/", 1 };
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() };
    pred.op = OVSTAGE_FILTER_OP_PREFIX;
    pred.values = &pathVal;
    pred.value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t qe = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (qe.status != OVSTAGE_OK)
        return false;
    waitAndRelease(mInstance, qe);

    ovstage_query_handle_t use = q;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        ovstage_release_query_result(mInstance, &qr);
    }

    ovx_token_t schemasAttr = OVX_INVALID_TOKEN;
    const ovx_string_t schemasName{ conv::kUsdSchemas, std::string_view(conv::kUsdSchemas).size() };
    if (ovx_path_dictionary_intern_token(mDict, schemasName, &schemasAttr) != OVX_OK ||
        schemasAttr == OVX_INVALID_TOKEN)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = mReadOrdinal;
    range.has_start_ordinal = false;

    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &schemasAttr, 1, range, &rh);
    if (re.status != OVSTAGE_OK)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }
    waitAndRelease(mInstance, re);

    struct ParsedSchema
    {
        TokenId schemaToken;
        TokenId baseToken;
        std::string instanceName;
    };

    std::unordered_map<uint64_t, ParsedSchema> parsedSchemas;
    std::unordered_map<uint32_t, bool> mayExist;
    std::unordered_map<uint32_t, std::unordered_set<uint64_t>> membership;
    std::unordered_map<uint64_t, std::vector<uint32_t>> schemasByPrim;
    std::unordered_map<uint32_t, std::unordered_set<uint64_t>> multiApplyMembership;
    std::unordered_map<uint64_t, std::unordered_map<uint32_t, std::vector<std::string>>> multiApplyByPrim;

    auto parsedSchema = [&](uint64_t schemaValue) -> const ParsedSchema*
    {
        const auto found = parsedSchemas.find(schemaValue);
        if (found != parsedSchemas.end())
            return &found->second;

        ovx_string_t schemaString{};
        if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(schemaValue), &schemaString) != OVX_OK ||
            !schemaString.ptr || schemaString.length == 0)
            return nullptr;

        const std::string_view name(schemaString.ptr, schemaString.length);
        ParsedSchema parsed;
        parsed.schemaToken = doInternToken(name);
        if (!parsed.schemaToken.valid())
            return nullptr;

        const size_t colon = name.find(':');
        if (colon != std::string_view::npos && colon > 0 && colon + 1 < name.size())
        {
            parsed.baseToken = doInternToken(name.substr(0, colon));
            parsed.instanceName.assign(name.data() + colon + 1, name.size() - colon - 1);
        }

        const auto inserted = parsedSchemas.emplace(schemaValue, std::move(parsed));
        return &inserted.first->second;
    };

    auto appendUniqueToken = [](std::vector<uint32_t>& dst, uint32_t value)
    {
        if (std::find(dst.begin(), dst.end(), value) == dst.end())
            dst.push_back(value);
    };
    auto appendUniqueString = [](std::vector<std::string>& dst, const std::string& value)
    {
        if (std::find(dst.begin(), dst.end(), value) == dst.end())
            dst.push_back(value);
    };
    auto addSchemaForHandle = [&](uint64_t handle, const ParsedSchema& schema)
    {
        if (!handle || !schema.schemaToken.valid())
            return;

        appendUniqueToken(schemasByPrim[handle], schema.schemaToken.id);
        membership[schema.schemaToken.id].insert(handle);
        mayExist[schema.schemaToken.id] = true;

        if (schema.baseToken.valid() && !schema.instanceName.empty())
        {
            multiApplyMembership[schema.baseToken.id].insert(handle);
            appendUniqueString(multiApplyByPrim[handle][schema.baseToken.id], schema.instanceName);
        }
    };
    auto forEachTokenValue = [&](const DLTensor& t, auto&& fn) -> bool
    {
        if (!t.data || !(t.dtype.code == kDLUInt && (t.dtype.bits == 64 || t.dtype.bits == 32)))
            return false;

        const int64_t count = totalElements(t);
        if (count < 0)
            return false;

        const auto* bytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
        for (int64_t i = 0; i < count; ++i)
        {
            const uint64_t value = (t.dtype.bits == 64) ?
                reinterpret_cast<const uint64_t*>(bytes)[i] :
                static_cast<uint64_t>(reinterpret_cast<const uint32_t*>(bytes)[i]);
            if (value != 0)
                fn(value);
        }
        return true;
    };
    auto addSchemaValue = [&](uint64_t raw, uint64_t canonical, uint64_t schemaValue)
    {
        const ParsedSchema* schema = parsedSchema(schemaValue);
        if (!schema)
            return;

        addSchemaForHandle(raw, *schema);
        if (canonical && canonical != raw)
            addSchemaForHandle(canonical, *schema);
    };

    bool unsupported = false;
    ovstage_read_group_t g{};
    ovstage_api_status_t fetchErr;
    while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
    {
        const bool usable = !g.is_delete && g.data.tensor_count > 0 && g.data.tensors && g.data.mask == nullptr;
        if (!usable)
        {
            ovstage_release_group(mInstance, &g);
            continue;
        }

        if (g.prims.count == 0)
        {
            unsupported = true;
            ovstage_release_group(mInstance, &g);
            continue;
        }

        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &paths, &pathCount) != OVX_OK || !paths)
        {
            unsupported = true;
            ovstage_release_group(mInstance, &g);
            continue;
        }

        if (g.is_array)
        {
            for (uint32_t row = 0; row < g.prims.count; ++row)
            {
                const uint32_t idx = g.prims.index_map ? g.prims.index_map[row] : (g.prims.offset + row);
                if (idx >= pathCount)
                    continue;

                const uint32_t tensorIndex = g.data.index_map ? g.data.index_map[row] : row;
                if (tensorIndex >= g.data.tensor_count)
                {
                    unsupported = true;
                    continue;
                }

                const uint64_t raw = paths[idx];
                if (!raw)
                    continue;
                const uint64_t canonical = canonicalHandle(ObjectKey{ raw });
                if (!forEachTokenValue(g.data.tensors[tensorIndex],
                                       [&](uint64_t schemaValue)
                                       {
                                           addSchemaValue(raw, canonical, schemaValue);
                                       }))
                    unsupported = true;
            }
        }
        else
        {
            const DLTensor& t = g.data.tensors[0];
            if (!t.data || !(t.dtype.code == kDLUInt && (t.dtype.bits == 64 || t.dtype.bits == 32)))
            {
                unsupported = true;
                ovstage_release_group(mInstance, &g);
                continue;
            }

            const int64_t total = totalElements(t);
            const int64_t storedRows = (t.ndim > 0 && t.shape[0] > 0) ? t.shape[0] : 1;
            if (total < 0 || storedRows <= 0 || (total % storedRows) != 0)
            {
                unsupported = true;
                ovstage_release_group(mInstance, &g);
                continue;
            }
            const int64_t comps = total / storedRows;
            const auto* bytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
            for (uint32_t row = 0; row < g.prims.count; ++row)
            {
                const uint32_t idx = g.prims.index_map ? g.prims.index_map[row] : (g.prims.offset + row);
                if (idx >= pathCount)
                    continue;

                const uint32_t dataRow = g.data.index_map ? g.data.index_map[row] : row;
                if (dataRow >= static_cast<uint32_t>(storedRows))
                {
                    unsupported = true;
                    continue;
                }

                const uint64_t raw = paths[idx];
                if (!raw)
                    continue;
                const uint64_t canonical = canonicalHandle(ObjectKey{ raw });
                for (int64_t col = 0; col < comps; ++col)
                {
                    const int64_t valueIndex = static_cast<int64_t>(dataRow) * comps + col;
                    const uint64_t schemaValue = (t.dtype.bits == 64) ?
                        reinterpret_cast<const uint64_t*>(bytes)[valueIndex] :
                        static_cast<uint64_t>(reinterpret_cast<const uint32_t*>(bytes)[valueIndex]);
                    if (schemaValue != 0)
                        addSchemaValue(raw, canonical, schemaValue);
                }
            }
        }

        ovstage_release_group(mInstance, &g);
    }

    waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));

    if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION || unsupported)
    {
        mSchemaMayExistCache.clear();
        mSchemaMembershipCache.clear();
        mSchemasByPrimCache.clear();
        mMultiApplyMembershipCache.clear();
        mMultiApplyInstancesByPrimCache.clear();
        return false;
    }

    mSchemaMayExistCache = std::move(mayExist);
    mSchemaMembershipCache = std::move(membership);
    mSchemasByPrimCache = std::move(schemasByPrim);
    mMultiApplyMembershipCache = std::move(multiApplyMembership);
    mMultiApplyInstancesByPrimCache = std::move(multiApplyByPrim);
    mSchemaCacheComplete = true;
    return true;
}

bool OvstageSource::collectSchemaKeys(TokenId schemaToken, std::vector<ObjectKey>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!schemaToken.valid())
        return true;

    const bool schemaCacheComplete = buildSchemaCache();
    if (!schemaCacheComplete && (!mInstance || !mDict))
        return false;

    std::unordered_set<uint64_t> seen;
    seen.reserve(out.size() + 32);
    for (const ObjectKey key : out)
    {
        const uint64_t canonical = canonicalHandle(key);
        seen.insert(canonical ? canonical : key.handle);
    }

    auto appendKey = [&](ObjectKey key)
    {
        if (!key.valid())
            return;
        const uint64_t canonical = canonicalHandle(key);
        const uint64_t stable = canonical ? canonical : key.handle;
        if (stable && seen.insert(stable).second)
            out.push_back(ObjectKey{ stable });
    };

    if (schemaCacheComplete)
    {
        const auto it = mSchemaMembershipCache.find(schemaToken.id);
        if (it != mSchemaMembershipCache.end())
        {
            for (const uint64_t handle : it->second)
                appendKey(ObjectKey{ handle });
        }
        return true;
    }

    const std::unordered_set<uint64_t>* members = schemaMembershipFromQuery(schemaToken);
    if (!members)
        return false;

    for (const uint64_t handle : *members)
        appendKey(ObjectKey{ handle });
    return true;
}

const std::unordered_set<uint64_t>* OvstageSource::schemaMembershipFromQuery(TokenId schemaToken) const
{
    if (!schemaToken.valid() || !mInstance || !mDict)
        return nullptr;

    const auto cached = mSchemaMembershipCache.find(schemaToken.id);
    if (cached != mSchemaMembershipCache.end())
        return &cached->second;

    const std::string_view schemaName = tokenToString(schemaToken);
    if (schemaName.empty())
        return nullptr;

    const ovx_string_t schemaVal{ schemaName.data(), schemaName.size() };
    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovx_string_t{ conv::kUsdSchemas, std::string_view(conv::kUsdSchemas).size() };
    pred.op = OVSTAGE_FILTER_OP_CONTAINS;
    pred.values = &schemaVal;
    pred.value_count = 1;

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
        return nullptr;
    waitAndRelease(mInstance, e);

    ovstage_query_handle_t use = q;
    size_t count = 0;
    bool fetched = false;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        fetched = true;
        count = qr.total_prim_count;
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        ovstage_release_query_result(mInstance, &qr);
    }
    if (!fetched)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return nullptr;
    }

    mSchemaMayExistCache[schemaToken.id] = count >= 1;
    if (count == 0)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        auto inserted = mSchemaMembershipCache.emplace(schemaToken.id, std::unordered_set<uint64_t>{});
        return &inserted.first->second;
    }

    ovx_token_t probe = OVX_INVALID_TOKEN;
    const ovx_string_t probeName{ conv::kUsdSchemas, std::string_view(conv::kUsdSchemas).size() };
    if (ovx_path_dictionary_intern_token(mDict, probeName, &probe) != OVX_OK ||
        probe == OVX_INVALID_TOKEN)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return nullptr;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = mReadOrdinal;
    range.has_start_ordinal = false;

    ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
    const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &probe, 1, range, &rh);
    if (re.status != OVSTAGE_OK)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return nullptr;
    }
    waitAndRelease(mInstance, re);

    std::unordered_set<uint64_t> members;
    members.reserve(count * 2);
    ovstage_read_group_t g{};
    ovstage_api_status_t fetchErr;
    while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
    {
        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &paths, &pathCount) == OVX_OK && paths)
        {
            for (uint32_t i = 0; i < g.prims.count; ++i)
            {
                const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                if (idx >= pathCount)
                    continue;
                const uint64_t raw = paths[idx];
                if (raw == 0)
                    continue;
                members.insert(raw);
                if (const uint64_t canonical = canonicalHandle(ObjectKey{ raw }))
                    members.insert(canonical);
            }
        }
        ovstage_release_group(mInstance, &g);
    }
    waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));

    if (fetchErr != OVSTAGE_ERROR_END_OF_ITERATION || members.empty())
        return nullptr;

    auto inserted = mSchemaMembershipCache.emplace(schemaToken.id, std::move(members));
    return &inserted.first->second;
}


bool OvstageSource::collectMultiApplySchemaKeys(TokenId baseSchemaToken, std::vector<ObjectKey>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!baseSchemaToken.valid())
        return true;
    if (!buildSchemaCache())
        return false;

    const auto it = mMultiApplyMembershipCache.find(baseSchemaToken.id);
    if (it == mMultiApplyMembershipCache.end())
        return true;

    std::unordered_set<uint64_t> seen;
    seen.reserve(out.size() + it->second.size());
    for (const ObjectKey key : out)
    {
        const uint64_t canonical = canonicalHandle(key);
        seen.insert(canonical ? canonical : key.handle);
    }

    for (const uint64_t handle : it->second)
    {
        const uint64_t canonical = canonicalHandle(ObjectKey{ handle });
        const uint64_t stable = canonical ? canonical : handle;
        if (stable && seen.insert(stable).second)
            out.push_back(ObjectKey{ stable });
    }
    return true;
}

void OvstageSource::prefetchBucket(const std::vector<ObjectKey>& keys,
                                   const std::vector<std::string>& attrNames,
                                   bool sealMissing) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    clearBucket();
    if (!mInstance || !mDict || keys.empty() || attrNames.empty())
        return;

    struct CacheHandlePair
    {
        uint64_t raw = 0;
        uint64_t canonical = 0;
    };

    // Record bucket membership up front (by canonical handle): a prim with no
    // readable column is still "in the bucket", so a sealed scalar miss for it
    // resolves to empty.
    std::vector<ovx_primpath_t> paths;
    std::vector<CacheHandlePair> cacheHandles;
    paths.reserve(keys.size());
    cacheHandles.reserve(keys.size());
    for (const ObjectKey k : keys)
    {
        if (!k.valid())
            continue;
        paths.push_back(k.handle);
        mBucketKeys.insert(k.handle);
        const uint64_t ch = canonicalHandle(k);
        if (ch)
            mBucketKeys.insert(ch);
        cacheHandles.push_back({ k.handle, ch });
    }
    if (paths.empty())
        return;
    mBucketActive = true;

    // Map each requested attribute's ovstage token -> parse TokenId so a returned
    // group (tagged by g.attribute) routes to the scalar cache. Transform
    // matrices are cached separately from scalar attrs so transform reads can
    // avoid per-prim ovstage reads while preserving the IPhysicsSource API.
    struct AttrMeta
    {
        TokenId tokenId;
        bool isWorldTransform;
        bool isLocalTransform;
        bool isResetXformStack;
    };
    std::unordered_map<uint64_t, AttrMeta> metaByToken;
    std::vector<ovx_token_t> attrToks;
    std::vector<uint32_t> invalidAttrIds;
    attrToks.reserve(attrNames.size());
    auto loadCacheCovers = [&](uint64_t handle, uint32_t attrId) -> bool
    {
        const auto cit = mLoadCacheCoveredAttrs.find(handle);
        return cit != mLoadCacheCoveredAttrs.end() && cit->second.count(attrId) != 0;
    };
    auto loadCacheCoversAll = [&](uint32_t attrId) -> bool
    {
        if (!mLoadCacheActive)
            return false;
        for (const CacheHandlePair& handle : cacheHandles)
        {
            const bool coveredRaw = handle.raw && loadCacheCovers(handle.raw, attrId);
            const bool coveredCanonical = handle.canonical && loadCacheCovers(handle.canonical, attrId);
            if (!coveredRaw && !coveredCanonical)
                return false;
        }
        return true;
    };
    const TokenId worldTransformAttr = doInternToken(conv::kFabricWorldMatrix);
    const TokenId localTransformAttr = doInternToken(conv::kLocalTransform);
    const TokenId fabricLocalTransformAttr = doInternToken(conv::kFabricLocalMatrix);
    const TokenId resetXformStackAttr = doInternToken(conv::kResetXformStack);
    for (const std::string& name : attrNames)
    {
        const TokenId tokenId = doInternToken(name);
        if (loadCacheCoversAll(tokenId.id))
        {
            continue;
        }
        const ovx_token_t tok = ovxToken(name);
        if (tok == OVX_INVALID_TOKEN)
        {
            if (mLoadCacheActive && sealMissing)
            {
                mBucketAttributeIds.insert(tokenId.id);
                invalidAttrIds.push_back(tokenId.id);
            }
            continue;
        }
        mBucketAttributeIds.insert(tokenId.id);
        attrToks.push_back(tok);
        metaByToken[tok] = AttrMeta{
            tokenId,
            tokenId == worldTransformAttr,
            tokenId == localTransformAttr || tokenId == fabricLocalTransformAttr,
            tokenId == resetXformStackAttr
        };
    }

    auto markLoadCoverage = [&](uint64_t handle, uint32_t attrId)
    {
        if (handle)
            mLoadCacheCoveredAttrs[handle].insert(attrId);
    };
    auto markLoadCoverageForBucket = [&](uint32_t attrId)
    {
        if (!mLoadCacheActive)
            return;
        for (const CacheHandlePair& handle : cacheHandles)
        {
            markLoadCoverage(handle.raw, attrId);
            if (handle.canonical && handle.canonical != handle.raw)
                markLoadCoverage(handle.canonical, attrId);
        }
    };
    for (const uint32_t attrId : invalidAttrIds)
        markLoadCoverageForBucket(attrId);
    if (attrToks.empty())
    {
        if (mLoadCacheActive)
        {
            mBucketScalarsComplete = false;
            mBucketTransformsComplete = false;
        }
        return; // bucket active but nothing to read; sealed misses = empty
    }
    auto storeLoadScalarRef = [&](uint64_t raw, uint64_t canonical, uint32_t attrId, CachedTensorRow row)
    {
        if (!mLoadCacheActive || !raw)
            return;
        mLoadCacheScalars[raw][attrId] = row;
        if (canonical && canonical != raw)
            mLoadCacheScalars[canonical][attrId] = row;
    };
    auto storeLoadRelationshipRef = [&](uint64_t raw, uint64_t canonical, uint32_t attrId, CachedArrayRow row)
    {
        if (!mLoadCacheActive || !raw)
            return;
        mLoadCacheRelationships[raw][attrId] = row;
        if (canonical && canonical != raw)
            mLoadCacheRelationships[canonical][attrId] = row;
    };
    auto storeLoadTransformRef = [&](uint64_t raw,
                                     uint64_t canonical,
                                     const AttrMeta& meta,
                                     CachedTensorRow row)
    {
        if (!mLoadCacheActive || !raw)
            return;
        auto store = [&](uint64_t handle)
        {
            if (meta.isWorldTransform)
                mLoadCacheWorldTransforms[handle] = row;
            else if (meta.isLocalTransform)
                mLoadCacheLocalTransforms[handle] = row;
            else if (meta.isResetXformStack)
                mLoadCacheResetXformStack[handle] = row;
        };
        store(raw);
        if (canonical && canonical != raw)
            store(canonical);
    };

    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(mDict, paths.data(), paths.size(), &list) != OVX_OK)
        return;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(mInstance, list, &q) == OVSTAGE_OK && q != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        // One multi-attribute read over the whole prim set (the columnar bulk read).
        ovstage_ordinal_range_t range{};
        range.end_ordinal = ~ovstage_ordinal_t(0); // latest (see withAttributeTensor)
        range.has_start_ordinal = false;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re =
            ovstage_read_attributes(mInstance, q, attrToks.data(), attrToks.size(), range, &rh);
        if (re.status == OVSTAGE_OK)
        {
            waitAndRelease(mInstance, re);

            std::unordered_set<uint32_t> returnedAttrIds;
            std::unordered_set<uint32_t> cacheableAttrIds;
            std::unordered_set<uint32_t> unsupportedAttrIds;
            ovstage_read_group_t g{};
            while (ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g) == OVSTAGE_OK)
            {
                const auto mit = metaByToken.find(g.attribute);
                if (mit == metaByToken.end())
                {
                    ovstage_release_group(mInstance, &g);
                    continue;
                }
                returnedAttrIds.insert(mit->second.tokenId.id);

                bool groupOwnedByLoadCache = false;
                if (g.is_delete)
                {
                    cacheableAttrIds.insert(mit->second.tokenId.id);
                    ovstage_release_group(mInstance, &g);
                    continue;
                }

                const bool usable = g.data.tensor_count > 0 && g.data.tensors;
                // Fixed-size columns may still carry data.index_map for
                // reorder/dedup. Cache those by resolving each prim row to the
                // indexed tensor row; only masked fixed-size columns are left to
                // the per-prim fallback.
                const bool dense = usable && !g.is_array && g.data.mask == nullptr;
                if (usable && dense)
                {
                    const DLTensor& t = g.data.tensors[0];
                    const bool transformAttr =
                        mit->second.isWorldTransform || mit->second.isLocalTransform || mit->second.isResetXformStack;
                    const bool cacheable =
                        ((mit->second.isWorldTransform || mit->second.isLocalTransform) && t.data) ||
                        (mit->second.isResetXformStack && t.data && canDecodeAttrValue(t)) ||
                        (!transformAttr && t.data && canDecodeAttrValue(t));
                    if (!cacheable)
                    {
                        unsupportedAttrIds.insert(mit->second.tokenId.id);
                        if (transformAttr)
                            mBucketTransformsComplete = false;
                        else
                            mBucketScalarsComplete = false;
                        ovstage_release_group(mInstance, &g);
                        continue;
                    }

                    const ovx_primpath_t* gpaths = nullptr;
                    size_t gcount = 0;
                    if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &gpaths, &gcount) == OVX_OK && gpaths)
                    {
                        cacheableAttrIds.insert(mit->second.tokenId.id);
                        const uint32_t rows = g.prims.count;
                        const int64_t total = totalElements(t);
                        // Per-prim component count = total / (number of tensor rows).
                        // Without a data.index_map the data is identity: the tensor has
                        // exactly one row per logical prim, so rows == prims.count.
                        // With a data.index_map the column is GATHERED from a tensor that
                        // may hold MORE rows than this group references (e.g. a scalar
                        // column materialized over the full populated prim set, of which
                        // one bucket reads only a few). Its true row count is the tensor's
                        // own first-dim extent (shape[0]); deriving it from the index-map
                        // max would under-count rows and over-count comps, mis-striding
                        // decodeAt (e.g. a 1-byte bool column decoded with comps=5) and
                        // corrupting every value.
                        int64_t storedRows = rows;
                        if (g.data.index_map)
                        {
                            if (t.ndim >= 1 && t.shape[0] > 0)
                                storedRows = static_cast<int64_t>(t.shape[0]);
                            else
                            {
                                storedRows = 0;
                                for (uint32_t i = 0; i < rows; ++i)
                                    storedRows = std::max<int64_t>(
                                        storedRows, static_cast<int64_t>(g.data.index_map[i]) + 1);
                            }
                        }
                        if (total < 0 || storedRows <= 0 || (total % storedRows) != 0)
                        {
                            unsupportedAttrIds.insert(mit->second.tokenId.id);
                            if (transformAttr)
                                mBucketTransformsComplete = false;
                            else
                                mBucketScalarsComplete = false;
                            ovstage_release_group(mInstance, &g);
                            continue;
                        }
                        const int64_t comps = total / storedRows;
                        if (((mit->second.isWorldTransform || mit->second.isLocalTransform) && comps < 16) ||
                            (mit->second.isResetXformStack && comps < 1))
                        {
                            unsupportedAttrIds.insert(mit->second.tokenId.id);
                            mBucketTransformsComplete = false;
                            ovstage_release_group(mInstance, &g);
                            continue;
                        }
                        const DLTensor* cacheTensor = &t;
                        if (mLoadCacheActive)
                        {
                            mLoadCacheGroups.push_back(g);
                            groupOwnedByLoadCache = true;
                            cacheTensor = &mLoadCacheGroups.back().data.tensors[0];
                        }
                        for (uint32_t i = 0; i < rows; ++i)
                        {
                            const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                            if (idx >= gcount)
                                continue;
                            uint32_t dataRow = g.data.index_map ? g.data.index_map[i] : i;
                            if (dataRow >= static_cast<uint32_t>(storedRows))
                            {
                                unsupportedAttrIds.insert(mit->second.tokenId.id);
                                if (transformAttr)
                                    mBucketTransformsComplete = false;
                                else
                                    mBucketScalarsComplete = false;
                                continue;
                            }
                            const uint64_t raw = gpaths[idx];
                            if (!raw)
                                continue;
                            const uint64_t canonical = canonicalHandle(ObjectKey{ raw });
                            if (mit->second.isWorldTransform || mit->second.isLocalTransform)
                            {
                                if (comps < 16)
                                {
                                    mBucketTransformsComplete = false;
                                    unsupportedAttrIds.insert(mit->second.tokenId.id);
                                    continue;
                                }
                                if (mLoadCacheActive)
                                    storeLoadTransformRef(raw, canonical, mit->second,
                                                          CachedTensorRow{ cacheTensor, comps, dataRow });
                                else
                                {
                                    Matrix4d matrix;
                                    if (!decodeMatrixAt(t, comps, dataRow, matrix.data))
                                    {
                                        mBucketTransformsComplete = false;
                                        unsupportedAttrIds.insert(mit->second.tokenId.id);
                                        continue;
                                    }
                                    mBucketKeys.insert(raw);
                                    if (mit->second.isWorldTransform)
                                        mBucketWorldTransforms[raw] = matrix;
                                    else
                                        mBucketLocalTransforms[raw] = matrix;
                                    if (canonical && canonical != raw)
                                    {
                                        mBucketKeys.insert(canonical);
                                        if (mit->second.isWorldTransform)
                                            mBucketWorldTransforms[canonical] = matrix;
                                        else
                                            mBucketLocalTransforms[canonical] = matrix;
                                    }
                                }
                                continue;
                            }
                            if (mit->second.isResetXformStack)
                            {
                                if (mLoadCacheActive)
                                    storeLoadTransformRef(raw, canonical, mit->second,
                                                          CachedTensorRow{ cacheTensor, comps, dataRow });
                                else
                                {
                                    const AttrValue v = decodeAt(t, comps, dataRow);
                                    bool reset = false;
                                    if (v.kind == AttrValue::Kind::eBool)
                                        reset = v.b;
                                    else if (v.kind == AttrValue::Kind::eInt)
                                        reset = v.i != 0;
                                    else
                                    {
                                        mBucketTransformsComplete = false;
                                        unsupportedAttrIds.insert(mit->second.tokenId.id);
                                        continue;
                                    }
                                    mBucketKeys.insert(raw);
                                    mBucketResetXformStack[raw] = reset;
                                    if (canonical && canonical != raw)
                                    {
                                        mBucketKeys.insert(canonical);
                                        mBucketResetXformStack[canonical] = reset;
                                    }
                                }
                                continue;
                            }
                            if (mLoadCacheActive)
                                storeLoadScalarRef(raw, canonical, mit->second.tokenId.id,
                                                   CachedTensorRow{ cacheTensor, comps, dataRow });
                            else
                            {
                                AttrValue v = decodeAt(t, comps, dataRow);
                                if (v.valid())
                                {
                                    mBucketKeys.insert(raw);
                                    mBucketScalars[raw][mit->second.tokenId.id] = v;
                                    if (canonical && canonical != raw)
                                    {
                                        mBucketKeys.insert(canonical);
                                        mBucketScalars[canonical][mit->second.tokenId.id] = std::move(v);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        unsupportedAttrIds.insert(mit->second.tokenId.id);
                        if (mit->second.isWorldTransform || mit->second.isLocalTransform || mit->second.isResetXformStack)
                            mBucketTransformsComplete = false;
                        else
                            mBucketScalarsComplete = false;
                    }
                }
                else if (usable && g.is_array && g.data.mask == nullptr)
                {
                    const ovx_primpath_t* gpaths = nullptr;
                    size_t gcount = 0;
                    if (ovx_path_dictionary_get_paths(mDict, g.prims.list, &gpaths, &gcount) != OVX_OK || !gpaths)
                    {
                        unsupportedAttrIds.insert(mit->second.tokenId.id);
                        mBucketScalarsComplete = false;
                        ovstage_release_group(mInstance, &g);
                        continue;
                    }

                    bool cacheable = true;
                    for (uint32_t row = 0; row < g.prims.count; ++row)
                    {
                        uint32_t tensorIndex = g.data.index_map ? g.data.index_map[row] : row;
                        if (tensorIndex >= g.data.tensor_count || !canDecodeRelationshipTargets(g.data.tensors[tensorIndex]))
                        {
                            cacheable = false;
                            break;
                        }
                    }
                    if (!cacheable)
                    {
                        unsupportedAttrIds.insert(mit->second.tokenId.id);
                        mBucketScalarsComplete = false;
                        ovstage_release_group(mInstance, &g);
                        continue;
                    }

                    cacheableAttrIds.insert(mit->second.tokenId.id);
                    if (mLoadCacheActive)
                    {
                        mLoadCacheGroups.push_back(g);
                        groupOwnedByLoadCache = true;
                    }
                    for (uint32_t row = 0; mLoadCacheActive && row < g.prims.count; ++row)
                    {
                        const uint32_t idx = g.prims.index_map ? g.prims.index_map[row] : (g.prims.offset + row);
                        if (idx >= gcount)
                            continue;
                        uint32_t tensorIndex = g.data.index_map ? g.data.index_map[row] : row;
                        if (tensorIndex >= mLoadCacheGroups.back().data.tensor_count)
                            continue;
                        const uint64_t raw = gpaths[idx];
                        if (!raw)
                            continue;
                        const uint64_t canonical = canonicalHandle(ObjectKey{ raw });
                        storeLoadRelationshipRef(raw, canonical, mit->second.tokenId.id,
                                                 CachedArrayRow{ &mLoadCacheGroups.back().data.tensors[tensorIndex] });
                    }
                    if (!mLoadCacheActive)
                        mBucketScalarsComplete = false;
                }
                else if (usable) // a column we can't cache row-wise
                {
                    unsupportedAttrIds.insert(mit->second.tokenId.id);
                    if (mit != metaByToken.end() &&
                        (mit->second.isWorldTransform || mit->second.isLocalTransform || mit->second.isResetXformStack))
                        mBucketTransformsComplete = false;
                    else
                        mBucketScalarsComplete = false;
                }
                else
                {
                    // ovstage can report an attribute group with no tensor data
                    // when the column exists in the dictionary but has no authored
                    // value for this path-list read. Treat that as a sealed empty
                    // result so later getAttribute calls do not fall back to a
                    // per-prim read for every prim in the bucket.
                    cacheableAttrIds.insert(mit->second.tokenId.id);
                }
                if (!groupOwnedByLoadCache)
                    ovstage_release_group(mInstance, &g);
            }
            if (mLoadCacheActive)
            {
                for (const std::pair<const uint64_t, AttrMeta>& attr : metaByToken)
                {
                    const uint32_t attrId = attr.second.tokenId.id;
                    if (unsupportedAttrIds.count(attrId) != 0)
                        continue;
                    if (sealMissing && (returnedAttrIds.count(attrId) == 0 || cacheableAttrIds.count(attrId) != 0))
                        markLoadCoverageForBucket(attrId);
                }
                // Load-cache refs are the authoritative path during scan. Keep
                // the concept bucket from treating an empty local bucket map as a
                // sealed miss before the persistent cache gets a chance to answer.
                mBucketScalarsComplete = false;
                mBucketTransformsComplete = false;
            }
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    }
    ovx_path_dictionary_destroy_path_list(mDict, list);
}

void OvstageSource::seedBucketFromReadGroup(TokenId attr,
                                            const ovstage_read_group_t& group,
                                            const ObjectKey* keys,
                                            size_t keyCount) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    clearBucket();
    if (!mDict || !attr.valid() || group.is_delete || group.prims.count == 0)
        return;

    const TokenId localTransformAttr = doInternToken(conv::kLocalTransform);
    const TokenId fabricLocalTransformAttr = doInternToken(conv::kFabricLocalMatrix);
    const TokenId worldTransformAttr = doInternToken(conv::kFabricWorldMatrix);
    const TokenId resetXformStackAttr = doInternToken(conv::kResetXformStack);
    const bool isWorldTransform = attr == worldTransformAttr;
    const bool isLocalTransform = attr == localTransformAttr || attr == fabricLocalTransformAttr;
    const bool isResetXformStack = attr == resetXformStackAttr;

    const ovx_primpath_t* gpaths = nullptr;
    size_t gcount = 0;
    if (ovx_path_dictionary_get_paths(mDict, group.prims.list, &gpaths, &gcount) != OVX_OK || !gpaths)
        return;

    mBucketRows.reserve(group.prims.count * 2);
    mBucketKeys.reserve(group.prims.count * 2);
    mBucketAttributeIds.insert(attr.id);
    size_t keyOrdinal = 0;
    for (uint32_t i = 0; i < group.prims.count; ++i)
    {
        const uint32_t idx = group.prims.index_map ? group.prims.index_map[i] : (group.prims.offset + i);
        if (idx >= gcount)
            continue;
        const uint64_t raw = gpaths[idx];
        if (!raw)
            continue;
        mBucketKeys.insert(raw);
        mBucketRows[raw] = i;
        if (keys && keyOrdinal < keyCount && keys[keyOrdinal].valid())
        {
            const uint64_t keyHandle = keys[keyOrdinal].handle;
            mBucketKeys.insert(keyHandle);
            mBucketRows[keyHandle] = i;
        }
        ++keyOrdinal;
    }

    if (mBucketKeys.empty())
        return;

    mBucketActive = true;
    const bool usable =
        group.data.tensor_count > 0 && group.data.tensors && group.data.tensors[0].data;
    const bool dense = usable && !group.is_array && group.data.index_map == nullptr && group.data.mask == nullptr;
    if (!dense)
    {
        if (isWorldTransform || isLocalTransform || isResetXformStack)
            mBucketTransformsComplete = false;
        else
            mBucketScalarsComplete = false;
        return;
    }

    const DLTensor& t = group.data.tensors[0];
    mBucketReadGroupAttr = attr;
    mBucketReadGroupTensor = &t;
    mBucketReadGroupComps = componentsPerPrim(t, group.prims.count);
    if (((isWorldTransform || isLocalTransform) && mBucketReadGroupComps < 16) ||
        (isResetXformStack && mBucketReadGroupComps < 1))
    {
        mBucketTransformsComplete = false;
        return;
    }

    if (isWorldTransform)
    {
        for (const std::pair<const uint64_t, uint32_t>& row : mBucketRows)
        {
            Matrix4d matrix;
            if (!decodeMatrixAt(t, mBucketReadGroupComps, row.second, matrix.data))
            {
                mBucketTransformsComplete = false;
                continue;
            }
            mBucketWorldTransforms[row.first] = matrix;
        }
        mBucketReadGroupAttr = {};
        mBucketReadGroupTensor = nullptr;
        mBucketReadGroupComps = 0;
    }
}

bool OvstageSource::hasAuthoredAttribute(ObjectKey key, TokenId attr) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mUsdStageId == 0 || !key.valid() || !attr.valid())
        return false;

    PXR_NS::UsdStageRefPtr stage = PXR_NS::UsdUtilsStageCache::Get().Find(
        PXR_NS::UsdStageCache::Id::FromLongInt(static_cast<long int>(mUsdStageId)));
    if (!stage)
        return false;

    const std::string path = pathOf(key);
    const std::string_view attrName = tokenToString(attr);
    if (path.empty() || attrName.empty())
        return false;

    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(PXR_NS::SdfPath(path));
    if (!prim)
        return false;

    PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(PXR_NS::TfToken(std::string(attrName)));
    return usdAttr && usdAttributeHasValueType(usdAttr, PXR_NS::UsdTimeCode::Default()) && usdAttr.HasAuthoredValue();
}

bool OvstageSource::isAttributeTimeSampled(ObjectKey /*key*/, TokenId /*attr*/) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return false;
}

// --- schemas / type --------------------------------------------------------

bool OvstageSource::hasSchema(ObjectKey key, TokenId schemaToken) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !schemaToken.valid())
        return false;

    const std::string_view schemaName = tokenToString(schemaToken);
    if (schemaName.empty())
        return false;

    auto containsKey = [&](const std::unordered_map<uint32_t, std::unordered_set<uint64_t>>& map,
                           TokenId token) -> bool
    {
        if (!token.valid())
            return false;
        const auto it = map.find(token.id);
        if (it == map.end())
            return false;
        if (it->second.count(key.handle) != 0)
            return true;
        const uint64_t canonical = canonicalHandle(key);
        return canonical && it->second.count(canonical) != 0;
    };

    // Materialize applied-schema membership once. After this point hasSchema is a
    // local token/handle lookup; schema discovery that needs ovstage queries flows
    // through collectSchemaKeys()/collectMultiApplySchemaKeys().
    (void)buildSchemaCache();

    if (containsKey(mSchemaMembershipCache, schemaToken))
        return true;

    // An unqualified multi-apply base (PhysicsLimitAPI, PhysicsDriveAPI, the vehicle
    // brakes/nonlinear-command-response bases, ...) matches when any qualified instance
    // is applied. mMultiApplyMembershipCache records membership for every such base, so
    // consult it generically rather than special-casing individual schema tokens.
    if (containsKey(mMultiApplyMembershipCache, schemaToken))
        return true;

    return false;
}

void OvstageSource::forEachDescendantPruned(ObjectKey root,
                                            std::function<bool(ObjectKey)> visit,
                                            DescendantScope scope) const
{
    if (!visit || !root.valid())
        return;

    if (visit(root))
        return;
    forEachChild(root, [this, &visit, scope](ObjectKey child) { forEachDescendantPruned(child, visit, scope); });
}

bool OvstageSource::isA(ObjectKey key, TokenId typeToken) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !typeToken.valid())
        return false;
    const std::string path = pathOf(key);
    const std::string_view typeName = tokenToString(typeToken);
    if (path.empty() || typeName.empty() || !mInstance)
        return false;

    auto hasPrimType = [&](std::string_view primType) -> bool
    {
        const ovx_string_t pathVal{ path.data(), path.size() };
        const ovx_string_t typeVal{ primType.data(), primType.size() };

        ovstage_predicate_t preds[2]{};
        preds[0].attribute.token = 0;
        preds[0].attribute.string = ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() };
        preds[0].op = OVSTAGE_FILTER_OP_IN;
        preds[0].values = &pathVal;
        preds[0].value_count = 1;
        preds[1].attribute.token = 0;
        preds[1].attribute.string = ovx_string_t{ conv::kUsdPrimType, std::string_view(conv::kUsdPrimType).size() };
        preds[1].op = OVSTAGE_FILTER_OP_IN;
        preds[1].values = &typeVal;
        preds[1].value_count = 1;

        ovstage_filter_t filter{};
        filter.predicates = preds;
        filter.count = 2;

        ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
        const ovstage_enqueue_result_t e = ovstage_query(mInstance, &filter, nullptr, 0, &q);
        if (e.status != OVSTAGE_OK)
            return false;
        waitAndRelease(mInstance, e);

        size_t count = 0;
        ovstage_query_result_t qr{};
        if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
        {
            count = qr.total_prim_count;
            ovstage_release_query_result(mInstance, &qr);
        }
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return count >= 1;
    };

    if (hasPrimType(typeName))
        return true;

    if (typeName == "UsdGeomPointInstancer" && hasPrimType("PointInstancer"))
        return true;

    if (typeName == "PhysicsJoint")
    {
        static const char* const kJointTypes[] = {
            "PhysicsFixedJoint", "PhysicsRevoluteJoint", "PhysicsPrismaticJoint",
            "PhysicsSphericalJoint", "PhysicsDistanceJoint", "PhysicsJoint",
            "PhysxPhysicsGearJoint", "PhysxPhysicsRackAndPinionJoint",
        };
        for (const char* candidate : kJointTypes)
            if (hasPrimType(candidate))
                return true;
    }
    else if (typeName == "Gprim")
    {
        static const char* const kGprimTypes[] = { "Cube", "Sphere", "Capsule", "Cone", "Cylinder", "Plane", "Mesh",
                                                   "TetMesh", "BasisCurves", "PhysxParticleSystem" };
        for (const char* candidate : kGprimTypes)
            if (hasPrimType(candidate))
                return true;
    }
    else if (typeName == "Xformable")
    {
        static const char* const kXformableTypes[] = {
            "Xform", "Cube", "Sphere", "Capsule", "Cone", "Cylinder", "Plane", "Mesh",
            "TetMesh", "PointInstancer", "Points", "BasisCurves", "PhysxParticleSystem",
        };
        for (const char* candidate : kXformableTypes)
            if (hasPrimType(candidate))
                return true;
    }
    else if (typeName == "PointBased")
    {
        static const char* const kPointBasedTypes[] = { "Mesh", "TetMesh", "Points", "BasisCurves" };
        for (const char* candidate : kPointBasedTypes)
            if (hasPrimType(candidate))
                return true;
    }

    static const char* const kPrototypeBackedTypes[] = {
        "Gprim", "Xformable", "PointBased", "Xform",
        "Cube", "Sphere", "Capsule", "Cone", "Cylinder", "Plane", "Mesh",
    };
    bool prototypeBackedType = false;
    for (const char* primType : kPrototypeBackedTypes)
    {
        if (typeName == primType)
        {
            prototypeBackedType = true;
            break;
        }
    }
    if (!prototypeBackedType)
        return false;

    const ObjectKey backingKey = geometryBackingKey(key);
    return backingKey != key && isA(backingKey, typeToken);
}

void OvstageSource::forEachAppliedSchema(ObjectKey key, std::function<void(TokenId)> cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !cb)
        return;

    if (!buildSchemaCache())
        return;

    std::unordered_set<uint32_t> emitted;
    auto emitForHandle = [&](uint64_t handle)
    {
        const auto it = mSchemasByPrimCache.find(handle);
        if (it == mSchemasByPrimCache.end())
            return;
        for (const uint32_t schemaId : it->second)
        {
            if (emitted.insert(schemaId).second)
                cb(TokenId{ schemaId });
        }
    };

    emitForHandle(key.handle);
    if (const uint64_t canonical = canonicalHandle(key))
        emitForHandle(canonical);
}

void OvstageSource::forEachMultiApplyInstance(ObjectKey key,
                                              std::string_view baseSchema,
                                              std::function<void(std::string_view)> cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || baseSchema.empty() || !cb)
        return;

    if (!buildSchemaCache())
        return;

    const TokenId baseToken = doInternToken(baseSchema);
    if (!baseToken.valid())
        return;

    std::unordered_set<std::string> emitted;
    auto emitForHandle = [&](uint64_t handle)
    {
        const auto primIt = mMultiApplyInstancesByPrimCache.find(handle);
        if (primIt == mMultiApplyInstancesByPrimCache.end())
            return;
        const auto baseIt = primIt->second.find(baseToken.id);
        if (baseIt == primIt->second.end())
            return;
        for (const std::string& instance : baseIt->second)
        {
            if (emitted.insert(instance).second)
                cb(instance);
        }
    };

    emitForHandle(key.handle);
    if (const uint64_t canonical = canonicalHandle(key))
        emitForHandle(canonical);
}

// --- relationships ---------------------------------------------------------

bool OvstageSource::hasRelationship(ObjectKey key, TokenId rel) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !rel.valid())
        return false;

    const std::string relName(tokenToString(rel));
    if (relName.empty())
        return false;

    bool found = false;
    withAttributeTensor(key, relName,
                        [&](const DLTensor&, uint32_t)
                        {
                            found = true;
                        });
    return found;
}

void OvstageSource::getRelationshipTargets(ObjectKey key, TokenId rel, std::vector<ObjectKey>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    out.clear();
    if (!key.valid() || !rel.valid())
        return;

    const std::string relName(tokenToString(rel));
    if (relName.empty())
        return;

    if (mLoadCacheActive)
    {
        auto lookupRelationship = [&](uint64_t handle) -> const CachedArrayRow*
        {
            const auto pit = mLoadCacheRelationships.find(handle);
            if (pit == mLoadCacheRelationships.end())
                return nullptr;
            const auto ait = pit->second.find(rel.id);
            return ait != pit->second.end() ? &ait->second : nullptr;
        };
        auto lookupDenseRelationship = [&](uint64_t handle) -> const CachedTensorRow*
        {
            const auto pit = mLoadCacheScalars.find(handle);
            if (pit == mLoadCacheScalars.end())
                return nullptr;
            const auto ait = pit->second.find(rel.id);
            return ait != pit->second.end() ? &ait->second : nullptr;
        };
        auto loadCacheCovers = [&](uint64_t handle) -> bool
        {
            const auto cit = mLoadCacheCoveredAttrs.find(handle);
            return cit != mLoadCacheCoveredAttrs.end() && cit->second.count(rel.id) != 0;
        };
        enum class LoadRelationshipLookup
        {
            eUnknown,
            eValue,
            eCoveredMiss,
        };
        auto appendIds = [&](const uint64_t* ids, int64_t count)
        {
            out.reserve(out.size() + static_cast<size_t>(count));
            for (int64_t i = 0; i < count; ++i)
            {
                if (ids[i] != OVX_INVALID_PRIMPATH)
                    out.push_back(ObjectKey{ ids[i] });
            }
        };
        auto tryHandle = [&](uint64_t handle) -> LoadRelationshipLookup
        {
            if (const CachedArrayRow* row = lookupRelationship(handle))
            {
                const DLTensor* t = row->tensor;
                if (!t || !canDecodeRelationshipTargets(*t))
                    return LoadRelationshipLookup::eUnknown;
                const int64_t n = totalElements(*t);
                const uint8_t* base = static_cast<const uint8_t*>(t->data) + t->byte_offset;
                appendIds(reinterpret_cast<const uint64_t*>(base), n);
                return LoadRelationshipLookup::eValue;
            }
            if (const CachedTensorRow* row = lookupDenseRelationship(handle))
            {
                const DLTensor* t = row->tensor;
                if (!t || !canDecodeRelationshipTargets(*t) || row->comps <= 0)
                    return LoadRelationshipLookup::eUnknown;
                const int64_t elemBytes = t->dtype.bits / 8;
                const uint8_t* base = static_cast<const uint8_t*>(t->data) + t->byte_offset +
                                      static_cast<int64_t>(row->row) * row->comps * elemBytes;
                appendIds(reinterpret_cast<const uint64_t*>(base), row->comps);
                return LoadRelationshipLookup::eValue;
            }
            if (loadCacheCovers(handle))
                return LoadRelationshipLookup::eCoveredMiss;
            return LoadRelationshipLookup::eUnknown;
        };

        // eValue: the load cache holds this prim's targets — done. eCoveredMiss:
        // the cache covers the prim but has no value row for this relationship.
        // Mirror the scalar/attribute read path (see tryHandle for getAttribute):
        // only trust a covered-miss as authoritative-empty when there is NO backing
        // USD stage. With a backing stage attached, fall through to the live
        // data-plane read + USD fallback — a relationship authored after the load
        // cache was built (e.g. a runtime joint body re-target) would otherwise be
        // missed. (For an instance with no backing stage, a covered-miss is the
        // authoritative empty and honours an ovstage rel-clear.)
        const uint64_t raw = key.handle;
        if (raw)
        {
            const LoadRelationshipLookup result = tryHandle(raw);
            if (result == LoadRelationshipLookup::eValue)
                return;
            if (result == LoadRelationshipLookup::eCoveredMiss && mUsdStageId == 0)
                return;
        }
        const uint64_t canonical = canonicalHandle(key);
        if (out.empty() && canonical && canonical != raw)
        {
            const LoadRelationshipLookup result = tryHandle(canonical);
            if (result == LoadRelationshipLookup::eValue)
                return;
            if (result == LoadRelationshipLookup::eCoveredMiss && mUsdStageId == 0)
                return;
        }
    }

    // ovpopulation writes each relationship as a ragged uint64 column of
    // ovx_primpath_t target ids (one row per authoring prim), named by the
    // relationship (ovpopulation_physics.inl pass 3). Read this prim's row.
    withAttributeTensor(key, relName,
                        [&](const DLTensor& t, uint32_t /*primCount*/)
                        {
                            if (!t.data || t.dtype.code != kDLUInt || t.dtype.bits != 64)
                                return;
                            const int64_t n = totalElements(t);
                            const uint8_t* base = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                            const uint64_t* ids = reinterpret_cast<const uint64_t*>(base);
                            out.reserve(static_cast<size_t>(n));
                            for (int64_t i = 0; i < n; ++i)
                                if (ids[i] != OVX_INVALID_PRIMPATH)
                                    out.push_back(ObjectKey{ ids[i] });
                        });
    if (!out.empty() || mUsdStageId == 0)
        return;

    PXR_NS::UsdStageRefPtr stage = usdStageFromId(mUsdStageId);
    const std::string path = pathOf(key);
    if (!stage || path.empty())
        return;

    PXR_NS::UsdPrim prim = stage->GetPrimAtPath(PXR_NS::SdfPath(path));
    if (!prim)
        return;

    PXR_NS::UsdRelationship usdRel = prim.GetRelationship(PXR_NS::TfToken(relName));
    if (!usdRel)
        return;

    PXR_NS::SdfPathVector targets;
    if (!usdRel.GetTargets(&targets))
        return;

    out.reserve(targets.size());
    for (const PXR_NS::SdfPath& target : targets)
    {
        if (target.IsEmpty())
            continue;
        ObjectKey targetKey = findByPath(target.GetPrimPath().GetString());
        if (targetKey.valid())
            out.push_back(targetKey);
    }
}

void OvstageSource::getInactiveInstanceIds(ObjectKey key, std::vector<int64_t>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    out.clear();

    bool readFromOvstage = false;
    withAttributeTensor(key, conv::kInactiveIds,
                        [&](const DLTensor& t, uint32_t /*primCount*/)
                        {
                            const int64_t total = totalElements(t);
                            if (total <= 0 || !t.data)
                                return;

                            const auto* base = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                            out.reserve(static_cast<size_t>(total));
                            if (t.dtype.code == kDLInt && t.dtype.bits == 64)
                            {
                                const auto* ids = reinterpret_cast<const int64_t*>(base);
                                out.assign(ids, ids + total);
                                readFromOvstage = true;
                            }
                            else if (t.dtype.code == kDLInt && t.dtype.bits == 32)
                            {
                                const auto* ids = reinterpret_cast<const int32_t*>(base);
                                for (int64_t i = 0; i < total; ++i)
                                    out.push_back(static_cast<int64_t>(ids[i]));
                                readFromOvstage = true;
                            }
                            else if (t.dtype.code == kDLUInt && t.dtype.bits == 64)
                            {
                                const auto* ids = reinterpret_cast<const uint64_t*>(base);
                                for (int64_t i = 0; i < total; ++i)
                                    out.push_back(static_cast<int64_t>(ids[i]));
                                readFromOvstage = true;
                            }
                            else if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
                            {
                                const auto* ids = reinterpret_cast<const uint32_t*>(base);
                                for (int64_t i = 0; i < total; ++i)
                                    out.push_back(static_cast<int64_t>(ids[i]));
                                readFromOvstage = true;
                            }
                        });
    // ovpopulation authors the inactiveIds column natively (read above); the
    // PointInstancer inactive-id set has no remaining USD fallback.
    (void)readFromOvstage;
}

// --- transforms ------------------------------------------------------------

void OvstageSource::getLocalToWorldTransform(ObjectKey key, Matrix4d& outMatrix) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    outMatrix = Matrix4d{}; // identity default
    if (!key.valid())
        return;

    const TokenId worldTransformAttr = doInternToken(conv::kFabricWorldMatrix);
    bool worldTransformMissCoveredByLoadCache = false;
    bool worldTransformMissCoveredByBucket = false;
    auto loadCacheCoversTransform = [&](uint64_t handle, TokenId attr) -> bool
    {
        const auto cit = mLoadCacheCoveredAttrs.find(handle);
        return cit != mLoadCacheCoveredAttrs.end() && cit->second.count(attr.id) != 0;
    };
    auto applyLoadWorldTransform = [&](uint64_t handle) -> bool
    {
        if (!handle)
            return false;
        const auto it = mLoadCacheWorldTransforms.find(handle);
        if (it == mLoadCacheWorldTransforms.end())
            return false;
        const CachedTensorRow& row = it->second;
        if (!row.tensor || !decodeMatrixAt(*row.tensor, row.comps, row.row, outMatrix.data))
            return false;
        return true;
    };
    auto applyCachedWorldTransform = [&](uint64_t handle) -> bool
    {
        if (!handle || mBucketKeys.count(handle) == 0)
            return false;
        const std::unordered_map<uint64_t, Matrix4d>::const_iterator it = mBucketWorldTransforms.find(handle);
        if (it == mBucketWorldTransforms.end())
            return false;
        outMatrix = it->second;
        return true;
    };

    if (mLoadCacheActive)
    {
        const uint64_t raw = key.handle;
        if (applyLoadWorldTransform(raw))
            return;
        const uint64_t canonical = canonicalHandle(key);
        if (canonical != raw && applyLoadWorldTransform(canonical))
            return;
        worldTransformMissCoveredByLoadCache =
            (raw && loadCacheCoversTransform(raw, worldTransformAttr)) ||
            (canonical && canonical != raw && loadCacheCoversTransform(canonical, worldTransformAttr));
    }

    if (mBucketActive && mBucketTransformsComplete)
    {
        const bool worldTransformCovered = mBucketAttributeIds.count(worldTransformAttr.id) != 0;
        const uint64_t raw = key.handle;
        if (applyCachedWorldTransform(raw))
            return;
        const uint64_t canonical = canonicalHandle(key);
        if (canonical != raw && applyCachedWorldTransform(canonical))
            return;
        worldTransformMissCoveredByBucket =
            worldTransformCovered &&
            (mBucketKeys.count(raw) != 0 || (canonical != raw && mBucketKeys.count(canonical) != 0));
    }

    bool gotWorld = false;
    if (!worldTransformMissCoveredByLoadCache && !worldTransformMissCoveredByBucket)
    {
        withAttributeTensor(key, conv::kFabricWorldMatrix,
                            [&](const DLTensor& t, uint32_t primCount)
                            {
                                gotWorld = decodeMatrixAt(t, componentsPerPrim(t, primCount), 0, outMatrix.data);
                            });
    }
    if (gotWorld)
        return;

    // When the data plane carries no resolved omni:fabric:worldMatrix, compose the
    // world transform natively from the parent chain of local transforms (which
    // ARE read from the data plane — getLocalTransformImpl has no USD fallback).
    // ovstage data-plane locals are a single ordinal snapshot and ignore the time
    // code, so resolve locals at the earliest time to match UsdSource's no-arg
    // overload; a typeless ancestor that authors no local resolves to identity,
    // which is correct for a structural container.
    //
    // During the initial load walk the snapshot is static, so memoize each prim's
    // composed world (see composeWorldFromLocalsLoadCached): sibling and descendant
    // prims share ancestors, and recomposing the full chain per prim was the
    // dominant remaining scan cost (O(prims * depth)). Outside the load walk the
    // plain per-call compose runs, so no memo can go stale under dynamic updates.
    if (mLoadCacheActive)
    {
        composeWorldFromLocalsLoadCached(key, outMatrix);
        return;
    }

    Matrix4d composed;
    const ObjectKey root = getRootKey();
    for (ObjectKey cur = key; cur.valid(); cur = getParent(cur))
    {
        Matrix4d local;
        bool resetsXformStack = false;
        getLocalTransformImpl(cur, ReadTime::defaultTime(), /*usdAtEarliestTime=*/true, local, resetsXformStack);
        composed = multiplyMatrix(composed, local);

        if (resetsXformStack || cur == root)
            break;
    }
    outMatrix = composed;
}

void OvstageSource::composeWorldFromLocalsLoadCached(ObjectKey key, Matrix4d& out) const
{
    // Caller holds mMutex and has established mLoadCacheActive.
    const uint64_t keyHandle = canonicalHandle(key);
    {
        const std::unordered_map<uint64_t, Matrix4d>::const_iterator it =
            mLoadCacheComposedWorld.find(keyHandle);
        if (it != mLoadCacheComposedWorld.end())
        {
            out = it->second;
            return;
        }
    }

    // Walk up until an already-memoized ancestor, an xform-stack reset, or the
    // root, collecting (handle, local) for every prim below the stop point. This
    // mirrors the plain fallback's stop conditions exactly (local of the stop prim
    // is included, then the walk terminates), so the composed product is identical.
    struct ChainNode
    {
        uint64_t handle;
        Matrix4d local;
    };
    std::vector<ChainNode> chain;
    const ObjectKey root = getRootKey();
    Matrix4d baseWorld;   // world transform above the stop point (identity default)
    bool haveBase = false;

    for (ObjectKey cur = key; cur.valid(); cur = getParent(cur))
    {
        const uint64_t curHandle = canonicalHandle(cur);
        const std::unordered_map<uint64_t, Matrix4d>::const_iterator memoIt =
            mLoadCacheComposedWorld.find(curHandle);
        if (memoIt != mLoadCacheComposedWorld.end())
        {
            baseWorld = memoIt->second;   // world(cur) known; chain holds only nodes below it
            haveBase = true;
            break;
        }

        Matrix4d local;
        bool resetsXformStack = false;
        getLocalTransformImpl(cur, ReadTime::defaultTime(), /*usdAtEarliestTime=*/true, local, resetsXformStack);
        chain.push_back({ curHandle, local });

        if (resetsXformStack || cur == root)
            break;   // stop prim's local is the top factor; no parent contribution
    }

    // Unwind from the stop point down to `key`, memoizing each node's world.
    // world(node) = local(node) * world(parent); the top chain entry's parent
    // contribution is baseWorld (identity when we stopped at root/reset).
    Matrix4d world = haveBase ? baseWorld : Matrix4d{};
    for (size_t i = chain.size(); i-- > 0;)
    {
        world = multiplyMatrix(chain[i].local, world);
        mLoadCacheComposedWorld[chain[i].handle] = world;
    }
    out = world;
}

void OvstageSource::getLocalTransform(ObjectKey key, ReadTime time, Matrix4d& outMatrix,
                                      bool& outResetsXformStack) const
{
    getLocalTransformImpl(key, time, /*usdAtEarliestTime=*/false, outMatrix, outResetsXformStack);
}

void OvstageSource::getLocalTransformImpl(ObjectKey key, ReadTime /*time*/, bool /*usdAtEarliestTime*/,
                                          Matrix4d& outMatrix, bool& outResetsXformStack) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    outMatrix = Matrix4d{};
    outResetsXformStack = false;
    if (!key.valid())
        return;

    const TokenId localTransformAttr = doInternToken(conv::kLocalTransform);
    const TokenId fabricLocalTransformAttr = doInternToken(conv::kFabricLocalMatrix);
    const TokenId resetXformStackAttr = doInternToken(conv::kResetXformStack);
    auto loadCacheCoversLocalAttr = [&](uint64_t handle, TokenId attr) -> bool
    {
        const auto cit = mLoadCacheCoveredAttrs.find(handle);
        return cit != mLoadCacheCoveredAttrs.end() && cit->second.count(attr.id) != 0;
    };
    auto applyLoadLocalRefs = [&](uint64_t handle, bool& foundLocal, bool& foundReset)
    {
        if (!handle)
            return;
        const auto localIt = mLoadCacheLocalTransforms.find(handle);
        if (localIt != mLoadCacheLocalTransforms.end())
        {
            const CachedTensorRow& row = localIt->second;
            if (row.tensor && decodeMatrixAt(*row.tensor, row.comps, row.row, outMatrix.data))
            {
                foundLocal = true;
            }
        }
        const auto resetIt = mLoadCacheResetXformStack.find(handle);
        if (resetIt != mLoadCacheResetXformStack.end())
        {
            const CachedTensorRow& row = resetIt->second;
            if (row.tensor)
            {
                const AttrValue v = decodeAt(*row.tensor, row.comps, row.row);
                if (v.kind == AttrValue::Kind::eBool)
                {
                    outResetsXformStack = v.b;
                    foundReset = true;
                }
                else if (v.kind == AttrValue::Kind::eInt)
                {
                    outResetsXformStack = v.i != 0;
                    foundReset = true;
                }
            }
        }
    };

    if (mLoadCacheActive)
    {
        bool foundLocal = false;
        bool foundReset = false;
        bool localCovered = false;
        bool resetCovered = false;
        const uint64_t raw = key.handle;
        if (raw)
        {
            applyLoadLocalRefs(raw, foundLocal, foundReset);
            localCovered = loadCacheCoversLocalAttr(raw, localTransformAttr) ||
                           loadCacheCoversLocalAttr(raw, fabricLocalTransformAttr);
            resetCovered = loadCacheCoversLocalAttr(raw, resetXformStackAttr);
        }
        const uint64_t canonical = canonicalHandle(key);
        if (canonical && canonical != raw)
        {
            applyLoadLocalRefs(canonical, foundLocal, foundReset);
            localCovered = localCovered ||
                           loadCacheCoversLocalAttr(canonical, localTransformAttr) ||
                           loadCacheCoversLocalAttr(canonical, fabricLocalTransformAttr);
            resetCovered = resetCovered || loadCacheCoversLocalAttr(canonical, resetXformStackAttr);
        }
        if (foundLocal && (foundReset || resetCovered))
        {
            return;
        }
        if (mUsdStageId == 0 && (foundLocal || localCovered) && (foundReset || resetCovered))
        {
            return;
        }
    }

    if (mBucketActive && mBucketTransformsComplete)
    {
        const bool localTransformCovered =
            mBucketAttributeIds.count(localTransformAttr.id) != 0 ||
            mBucketAttributeIds.count(fabricLocalTransformAttr.id) != 0;
        const bool resetXformStackCovered = mBucketAttributeIds.count(resetXformStackAttr.id) != 0;
        auto materializeCachedTransform = [&](uint64_t handle) -> bool
        {
            if (!handle || !mBucketReadGroupTensor)
                return false;
            const auto rit = mBucketRows.find(handle);
            if (rit == mBucketRows.end())
                return false;
            if (mBucketReadGroupAttr == localTransformAttr || mBucketReadGroupAttr == fabricLocalTransformAttr)
            {
                Matrix4d matrix;
                if (!decodeMatrixAt(*mBucketReadGroupTensor, mBucketReadGroupComps, rit->second, matrix.data))
                {
                    mBucketTransformsComplete = false;
                    return false;
                }
                mBucketLocalTransforms[handle] = matrix;
                outMatrix = matrix;
                return true;
            }
            if (mBucketReadGroupAttr == resetXformStackAttr)
            {
                const AttrValue v = decodeAt(*mBucketReadGroupTensor, mBucketReadGroupComps, rit->second);
                bool reset = false;
                if (v.kind == AttrValue::Kind::eBool)
                    reset = v.b;
                else if (v.kind == AttrValue::Kind::eInt)
                    reset = v.i != 0;
                else
                {
                    mBucketTransformsComplete = false;
                    return false;
                }
                mBucketResetXformStack[handle] = reset;
                outResetsXformStack = reset;
                return true;
            }
            return false;
        };
        auto applyCachedTransform = [&](uint64_t handle) -> bool
        {
            if (!handle || mBucketKeys.count(handle) == 0)
                return false;
            bool foundLocal = false;
            bool foundReset = false;
            const std::unordered_map<uint64_t, Matrix4d>::const_iterator transformIt =
                mBucketLocalTransforms.find(handle);
            if (transformIt != mBucketLocalTransforms.end())
            {
                outMatrix = transformIt->second;
                foundLocal = true;
            }
            const std::unordered_map<uint64_t, bool>::const_iterator resetIt =
                mBucketResetXformStack.find(handle);
            if (resetIt != mBucketResetXformStack.end())
            {
                outResetsXformStack = resetIt->second;
                foundReset = true;
            }
            if (!foundLocal && !foundReset)
            {
                if (materializeCachedTransform(handle))
                {
                    foundLocal = mBucketLocalTransforms.find(handle) != mBucketLocalTransforms.end();
                    foundReset = mBucketResetXformStack.find(handle) != mBucketResetXformStack.end();
                }
            }
            if (!foundLocal && localTransformCovered && resetXformStackCovered && mUsdStageId == 0)
                return true;
            return foundLocal;
        };

        const uint64_t raw = key.handle;
        if (applyCachedTransform(raw))
            return;
        const uint64_t canonical = canonicalHandle(key);
        if (canonical != raw && applyCachedTransform(canonical))
            return;
    }

    bool gotLocal = false;
    withAttributeTensor(key, conv::kFabricLocalMatrix,
                        [&](const DLTensor& t, uint32_t primCount)
                        {
                            gotLocal = decodeMatrixAt(t, componentsPerPrim(t, primCount), 0, outMatrix.data);
                        });
    if (!gotLocal)
    {
        withAttributeTensor(key, conv::kLocalTransform,
                        [&](const DLTensor& t, uint32_t primCount)
                        {
                            gotLocal = decodeMatrixAt(t, componentsPerPrim(t, primCount), 0, outMatrix.data);
                        });
    }
    if (!gotLocal)
    {
        withAttributeTensor(key, "xformOp:transform",
                            [&](const DLTensor& t, uint32_t primCount)
                            {
                                gotLocal = decodeMatrixAt(t, componentsPerPrim(t, primCount), 0, outMatrix.data);
                            });
    }
    if (!gotLocal)
    {
        auto readXformOp = [&](std::string_view attrName, AttrValue& out) -> bool
        {
            bool found = false;
            withAttributeTensor(key, attrName,
                                [&](const DLTensor& t, uint32_t primCount)
                                {
                                    out = decodeScalar(t, primCount);
                                    found = out.valid();
                                });
            return found;
        };

        Matrix4d composed;
        bool gotOp = false;
        auto appendOp = [&](const Matrix4d& op)
        {
            composed = multiplyMatrix(composed, op);
            gotOp = true;
        };

        AttrValue value;
        carb::Float3 vec3;
        carb::Float4 quat;
        if (readXformOp("xformOp:scale", value) && attrAsFloat3(value, vec3))
        {
            appendOp(makeScaleMatrix(vec3));
        }

        value = {};
        if (readXformOp("xformOp:orient", value) && attrAsQuaternion(value, quat))
        {
            appendOp(makeQuaternionMatrix(quat));
        }

        auto appendScalarRotation = [&](const char* attrName, char axis)
        {
            AttrValue rotValue;
            double degrees = 0.0;
            if (readXformOp(attrName, rotValue) && attrAsDouble(rotValue, degrees))
                appendOp(makeAxisRotationMatrix(axis, degrees));
        };
        auto appendVectorRotation = [&](const char* attrName, const char* order)
        {
            AttrValue rotValue;
            carb::Float3 degrees;
            if (!readXformOp(attrName, rotValue) || !attrAsFloat3(rotValue, degrees))
                return;
            for (const char* c = order; *c; ++c)
            {
                double axisDegrees = 0.0;
                if (*c == 'X')
                    axisDegrees = static_cast<double>(degrees.x);
                else if (*c == 'Y')
                    axisDegrees = static_cast<double>(degrees.y);
                else if (*c == 'Z')
                    axisDegrees = static_cast<double>(degrees.z);
                appendOp(makeAxisRotationMatrix(*c, axisDegrees));
            }
        };

        appendScalarRotation("xformOp:rotateX", 'X');
        appendScalarRotation("xformOp:rotateY", 'Y');
        appendScalarRotation("xformOp:rotateZ", 'Z');
        appendVectorRotation("xformOp:rotateXYZ", "XYZ");
        appendVectorRotation("xformOp:rotateXZY", "XZY");
        appendVectorRotation("xformOp:rotateYXZ", "YXZ");
        appendVectorRotation("xformOp:rotateYZX", "YZX");
        appendVectorRotation("xformOp:rotateZXY", "ZXY");
        appendVectorRotation("xformOp:rotateZYX", "ZYX");

        value = {};
        if (readXformOp("xformOp:translate", value) && attrAsFloat3(value, vec3))
        {
            appendOp(makeTranslationMatrix(vec3));
        }

        if (gotOp)
        {
            outMatrix = composed;
            gotLocal = true;
        }
    }

    AttrValue reset;
    withAttributeTensor(key, conv::kResetXformStack,
                        [&](const DLTensor& t, uint32_t primCount) { reset = decodeScalar(t, primCount); });
    if (reset.kind == AttrValue::Kind::eBool)
        outResetsXformStack = reset.b;
    else if (reset.kind == AttrValue::Kind::eInt)
        outResetsXformStack = reset.i != 0;

    // Local transforms are read entirely from the ovstage data plane
    // (omni:xform / omni:fabric:localMatrix + resetXformStack); no USD fallback.
    // When a prim authors no local transform the default identity is correct
    // (e.g. a typeless container), so an unresolved gotLocal needs no USD read.
    (void)gotLocal;
}

void OvstageSource::getLocalToWorldRotationAndScale(ObjectKey key,
                                                    Matrix3d& outRotation,
                                                    carb::Float3& outScale) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    outRotation = Matrix3d{};
    outScale = { 1.0f, 1.0f, 1.0f };

    Matrix4d wm;
    getLocalToWorldTransform(key, wm);

    // Row-vector convention (v' = v * M): the upper-left 3x3 rows are the
    // transformed basis vectors, so each row's length is the per-axis scale and
    // the normalized row is the pure rotation. (Shear from non-uniform parent
    // scale under rotation is not separated — a later slice, matching the body
    // path which reads translation only.)
    const double rows[3][3] = {
        { wm.data[0], wm.data[1], wm.data[2] },
        { wm.data[4], wm.data[5], wm.data[6] },
        { wm.data[8], wm.data[9], wm.data[10] }
    };
    for (int i = 0; i < 3; ++i)
    {
        const double len =
            std::sqrt(rows[i][0] * rows[i][0] + rows[i][1] * rows[i][1] + rows[i][2] * rows[i][2]);
        const double inv = (len > 1e-12) ? 1.0 / len : 0.0;
        (&outScale.x)[i] = static_cast<float>(len);
        outRotation.data[i * 3 + 0] = rows[i][0] * inv;
        outRotation.data[i * 3 + 1] = rows[i][1] * inv;
        outRotation.data[i * 3 + 2] = rows[i][2] * inv;
    }
}

// --- buffers / mesh --------------------------------------------------------

BufferHandle OvstageSource::registerMeshBuffer(const void* data, size_t byteCount, uint32_t elemCount,
                                               BufferElemType type) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!data || byteCount == 0 || elemCount == 0)
        return {};
    BufferHandle h;
    h.id = mNextBufferId++;
    h.elemCount = elemCount;
    h.type = type;
    const auto fullHash = carb::extras::fnv128hash(static_cast<const uint8_t*>(data), byteCount);
    h.contentHash[0] = fullHash.d[0];
    h.contentHash[1] = fullHash.d[1];
    mBuffers.emplace(h.id, std::vector<uint8_t>(static_cast<const uint8_t*>(data),
                                                static_cast<const uint8_t*>(data) + byteCount));
    return h;
}

BufferHandle OvstageSource::getArrayAttribute(ObjectKey key, TokenId attr, ReadTime time) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    (void)time; // ovstage population currently exposes the latest sealed value for array columns.

    const std::string_view name = tokenToString(attr);
    const auto endsWith = [](std::string_view value, std::string_view suffix)
    {
        return value.size() >= suffix.size() && value.substr(value.size() - suffix.size()) == suffix;
    };

    if (name == "protoIndices" || name == "faceVertexIndices" || name == "faceVertexCounts" ||
        name == "holeIndices")
        return readArrayBuffer(key, name, BufferElemType::eInt32, 1, time);
    if (name == "tetVertexIndices" || name == "omniphysics:restTetVtxIndices" || name == "restTetVtxIndices")
        return readArrayBuffer(key, name, BufferElemType::eInt4, 4, time);
    if (name == "surfaceFaceVertexIndices" || name == "omniphysics:restTriVtxIndices" ||
        name == "restTriVtxIndices")
        return readArrayBuffer(key, name, BufferElemType::eInt3, 3, time);
    if (name == "positions" || name == "points" || name == "velocities" ||
        name == "angularVelocities" || name == "scales" || name == "omniphysics:restShapePoints" ||
        name == "restShapePoints" || endsWith(name, ":omniphysics:points"))
        return readArrayBuffer(key, name, BufferElemType::eVec3, 3, time);
    if (name == "orientations")
        return readArrayBuffer(key, name, BufferElemType::eQuath, 4, time);

    return {};
}

BufferHandle OvstageSource::readArrayAttribute(ObjectKey key, std::string_view attr, BufferElemType type, int comps) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return readArrayBuffer(key, attr, type, comps);
}

BufferHandle OvstageSource::readArrayBuffer(ObjectKey key, std::string_view attr, BufferElemType type, int comps, ReadTime time) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    BufferHandle h;
    withAttributeTensor(key, attr,
        [&](const DLTensor& t, uint32_t /*primCount*/)
        {
            const int64_t total = totalElements(t); // total scalar lanes for the (single) prim
            if (total <= 0 || !t.data || comps <= 0 || (total % comps) != 0)
                return;
            const uint32_t elemCount = static_cast<uint32_t>(total / comps);
            const auto* base = static_cast<const uint8_t*>(t.data) + t.byte_offset;

            if (type == BufferElemType::eInt32 || type == BufferElemType::eInt3 ||
                type == BufferElemType::eInt4)
            {
                // Normalize integer topology payloads to signed 32-bit lanes,
                // preserving the requested scalar/vector element type.
                std::vector<int32_t> tmp(static_cast<size_t>(total));
                if (t.dtype.code == kDLInt && t.dtype.bits == 32)
                    std::memcpy(tmp.data(), base, static_cast<size_t>(total) * sizeof(int32_t));
                else if (t.dtype.code == kDLInt && t.dtype.bits == 64)
                    for (int64_t i = 0; i < total; ++i)
                        tmp[i] = static_cast<int32_t>(reinterpret_cast<const int64_t*>(base)[i]);
                else if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
                    for (int64_t i = 0; i < total; ++i)
                        tmp[i] = static_cast<int32_t>(reinterpret_cast<const uint32_t*>(base)[i]);
                else
                    return;
                h = registerMeshBuffer(tmp.data(), tmp.size() * sizeof(int32_t), elemCount, type);
            }
            else if (type == BufferElemType::eQuath)
            {
                if (t.dtype.code != kDLFloat || t.dtype.bits != 16)
                    return;
                h = registerMeshBuffer(base, static_cast<size_t>(total) * sizeof(uint16_t), elemCount, type);
            }
            else // packed float32 lanes (Vec2/3/4/Float).
            {
                if (t.dtype.code != kDLFloat || t.dtype.bits != 32)
                    return;
                h = registerMeshBuffer(base, static_cast<size_t>(total) * sizeof(float), elemCount, type);
            }
        });
    if (h.valid())
        return h;

    PXR_NS::UsdStageRefPtr stage = usdStageFromId(mUsdStageId);
    if (!stage || !key.valid())
        return {};

    const std::string path = pathOf(key);
    if (path.empty())
        return {};

    const PXR_NS::UsdPrim prim = stage->GetPrimAtPath(PXR_NS::SdfPath(path));
    if (!prim)
        return {};

    const PXR_NS::UsdAttribute usdAttr = prim.GetAttribute(PXR_NS::TfToken(std::string(attr)));
    if (!usdAttr || !usdAttr.HasValue())
        return {};

    const PXR_NS::UsdTimeCode timeCode = usdTimeCode(time);
    auto registerUsdArray = [&](const auto& array) -> BufferHandle
    {
        using ArrayType = std::decay_t<decltype(array)>;
        using ElementType = typename ArrayType::value_type;
        if (array.empty())
            return {};
        return registerMeshBuffer(array.cdata(), array.size() * sizeof(ElementType),
                                  static_cast<uint32_t>(array.size()), type);
    };

    if (type == BufferElemType::eFloat)
    {
        PXR_NS::VtFloatArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eInt32)
    {
        PXR_NS::VtIntArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eVec2)
    {
        PXR_NS::VtVec2fArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eVec3)
    {
        PXR_NS::VtVec3fArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eVec4)
    {
        PXR_NS::VtVec4fArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eInt3)
    {
        PXR_NS::VtVec3iArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eInt4)
    {
        PXR_NS::VtVec4iArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }
    if (type == BufferElemType::eQuath)
    {
        PXR_NS::VtQuathArray array;
        return usdAttr.Get(&array, timeCode) ? registerUsdArray(array) : BufferHandle{};
    }

    return h;
}

const void* OvstageSource::resolveBuffer(BufferHandle handle, size_t& byteCount) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    auto it = mBuffers.find(handle.id);
    if (it == mBuffers.end())
    {
        byteCount = 0;
        return nullptr;
    }
    byteCount = it->second.size();
    return it->second.data();
}

void OvstageSource::releaseBuffer(BufferHandle handle) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mBuffers.erase(handle.id);
}

MeshGeometry OvstageSource::getMeshAttributes(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    MeshGeometry out;
    if (!key.valid())
        return out;
    const ObjectKey backingKey = geometryBackingKey(key);

    // Geometry arrays — points (Vec3 float), face-vertex indices / counts / holes
    // (Int32). Mint a buffer for each non-empty column; the cooking service reads
    // them via resolveBuffer through the same FROM_PRIM_MESH_VIEW path USD uses.
    out.points = readArrayBuffer(backingKey, "points", BufferElemType::eVec3, 3);
    out.indices = readArrayBuffer(backingKey, "faceVertexIndices", BufferElemType::eInt32, 1);
    out.faceCounts = readArrayBuffer(backingKey, "faceVertexCounts", BufferElemType::eInt32, 1);
    out.holes = readArrayBuffer(backingKey, "holeIndices", BufferElemType::eInt32, 1);

    // Scalars — doubleSided defaults false; orientation defaults rightHanded.
    bool dsided = false;
    if (getAttribute(backingKey, internToken("doubleSided"), dsided))
        out.doubleSided = dsided;
    TokenId orient{};
    if (getAttribute(backingKey, internToken("orientation"), orient) && orient.valid())
        out.leftHanded = (orient == internToken("leftHanded"));

    // faceMaterials (per-face physics-material index) is derived from material-bound
    // GeomSubsets on the USD path; left invalid here (single-material) until ovstage
    // surfaces subset bindings. Cooking treats an invalid faceMaterials as single-material.
    return out;
}

// --- source-wide -----------------------------------------------------------

void OvstageSource::loadUnits()
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mInstance || !mDict)
        return;
    // ovstage population authors stage units on the stage-info prim:
    // metersPerUnit / kilogramsPerUnit as doubles, upAxis as a token id ("Y"/"Z").
    ObjectKey info = findByPath(conv::kStageInfoPath);
    if (!info.valid())
        info = findByPath(conv::kLegacyStageInfoPath);
    withAttributeTensor(info, conv::kMetersPerUnit,
                        [&](const DLTensor& t, uint32_t primCount)
                        {
                            const AttrValue v = decodeScalar(t, primCount);
                            if (v.kind == AttrValue::Kind::eDouble)
                                mUnits.metersPerUnit = static_cast<float>(v.d);
                            else if (v.kind == AttrValue::Kind::eFloat)
                                mUnits.metersPerUnit = v.f;
                        });
    withAttributeTensor(info, conv::kKilogramsPerUnit,
                        [&](const DLTensor& t, uint32_t primCount)
                        {
                            const AttrValue v = decodeScalar(t, primCount);
                            if (v.kind == AttrValue::Kind::eDouble)
                                mUnits.kilogramsPerUnit = static_cast<float>(v.d);
                            else if (v.kind == AttrValue::Kind::eFloat)
                                mUnits.kilogramsPerUnit = v.f;
                        });
    withAttributeTensor(info, conv::kUpAxis,
                        [&](const DLTensor& t, uint32_t primCount)
                        {
                            // Stored as a token id (uint64 → AttrValue int); resolve
                            // through the dictionary to the axis name.
                            const AttrValue v = decodeScalar(t, primCount);
                            if (v.kind != AttrValue::Kind::eInt)
                                return;
                            ovx_string_t s{};
                            if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(v.i), &s) == OVX_OK &&
                                s.ptr)
                            {
                                const std::string_view axis(s.ptr, s.length);
                                mUnits.upAxis = (axis == "Y") ? UpAxis::eY : UpAxis::eZ;
                            }
                        });
}

SourceUnits OvstageSource::getSourceUnits() const
{
    return mUnits;
}

void OvstageSource::resolveCollection(ObjectKey primKey,
                                      TokenId collectionName,
                                      std::vector<ObjectKey>& members) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    members.clear();
    if (!primKey.valid() || !collectionName.valid())
        return;
    const std::string name(tokenToString(collectionName));
    if (name.empty())
        return;

    // Collections arrive as ovstage relationship columns. Mirror Fabric
    // collision-group preprocessing: expand includes through active descendants
    // and prune excluded subtrees before parse inverts the member list into the
    // path-keyed CollisionGroupsMap.
    std::vector<ObjectKey> includes, excludes;
    getRelationshipTargets(primKey, internToken("collection:" + name + ":includes"), includes);
    getRelationshipTargets(primKey, internToken("collection:" + name + ":excludes"), excludes);

    std::unordered_set<uint64_t> excluded;
    std::vector<ObjectKey> expanded;
    for (const ObjectKey exc : excludes)
    {
        collectDescendantKeys(exc, expanded);
        for (const ObjectKey member : expanded)
        {
            const uint64_t canonical = canonicalHandle(member);
            if (canonical)
                excluded.insert(canonical);
        }
    }

    std::unordered_set<uint64_t> emitted;
    for (const ObjectKey inc : includes)
    {
        collectDescendantKeys(inc, expanded);
        for (const ObjectKey member : expanded)
        {
            const uint64_t canonical = canonicalHandle(member);
            if (!canonical || excluded.count(canonical) != 0)
                continue;
            if (emitted.insert(canonical).second)
                members.push_back(ObjectKey{ canonical });
        }
    }
}

ObjectKey OvstageSource::getMaterialBinding(ObjectKey primKey) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    // The physics material binding rides on the USD MaterialBindingAPI
    // relationship for the "physics" purpose; ovpopulation resolves and writes it
    // as a target path-id column (same as any relationship). Prefer the
    // physics-purpose binding, fall back to the all-purpose one. (Collection-based
    // bindings are not handled here — direct binding is the physics norm.)
    std::vector<ObjectKey> targets;
    getRelationshipTargets(primKey, internToken("material:binding:physics"), targets);
    if (targets.empty())
        getRelationshipTargets(primKey, internToken("material:binding"), targets);
    if (!targets.empty())
        return targets.front();

    if (!primKey.valid() || !buildInstanceMaterialCache())
        return {};
    std::unordered_map<uint64_t, uint64_t>::const_iterator binding =
        mInstanceMaterialByPrim.find(primKey.handle);
    if (binding == mInstanceMaterialByPrim.end())
    {
        const uint64_t canonical = canonicalHandle(primKey);
        binding = mInstanceMaterialByPrim.find(canonical);
    }
    return binding == mInstanceMaterialByPrim.end() ? ObjectKey{} : ObjectKey{ binding->second };
}

std::unique_ptr<IChangeFeed> OvstageSource::createChangeFeed()
{
    if (!mInstance || !mDict)
        return nullptr;
    return std::make_unique<OvstageChangeFeed>(*this, mInstance, mDict);
}

} // namespace omni::physics::ovstage
