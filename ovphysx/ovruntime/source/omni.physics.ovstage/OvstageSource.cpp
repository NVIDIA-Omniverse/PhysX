// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-6 AC-7 AC-8 AC-9 AC-10 AC-11 AC-12 AC-13 AC-15 AC-16
 *
 * @implements REQ-PARSE-SCAN-001
 * @covers AC-14 AC-16 AC-18
 *
 * @implements REQ-PARSE-MAT-001
 * @covers AC-5 AC-6 AC-7
 *
 * @implements REQ-PARSE-INSTANCER-001
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-COL-005
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-PARSE-SHAPE-005
 * @covers AC-1 AC-2
 *
 * @implements REQ-PARSE-FEED-005
 * @covers AC-2 AC-5 AC-6 AC-7
 *
 * @implements REQ-PUBLICAPI-001
 * @covers AC-40 AC-45
 *
 * @implements REQ-PUBLICAPI-003
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7 AC-8 AC-9
 *
 * @implements REQ-PARSE-MASS-001
 * @covers AC-1
 *
 * @implements REQ-PARSE-FEED-002
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-PARSE-FEED-003
 * @covers AC-6 AC-7 AC-8 AC-9 AC-10 AC-11 AC-12 AC-13 AC-14
 *
 * @implements REQ-LOAD-TOKENS-001
 * @covers AC-5
 */

#include "OvstageSource.h"


#include "OvstageChangeFeed.h"

#include <omni/physics/parse/KnownTokens.h>
#include <ovstage/ovstage_population.h>

#include <common/foundation/MatrixTools.h>

#include <carb/extras/Hash.h>
#include <carb/logging/Log.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <string_view>
#include <unordered_set>

namespace omni::physics::ovstage
{

// Backing storage for OvstageSource::mInternTable -- see that member's doc
// comment (ADR-0021 Alternative A) and the header's "ObjectKey generation
// tagging" block. Defined here (not nested in OvstageSource) so it can be
// forward-declared from the header without exposing its members there.
//
// This table -- generation, and the ObjectKey <-> ovx_primpath_t mapping
// itself -- is scoped to the underlying ovstage_instance_t, not to the C++
// OvstageSource object: production code constructs exactly one OvstageSource
// per attach, but test/probe code (e.g. TestCustomTokenParity.cpp/
// TestOvstageWalker.cpp construct a standalone OvstageSource probe alongside
// scanOvstage()'s own internally-owned instance) legitimately builds multiple
// OvstageSource wrappers over ONE shared, concurrently-live instance/
// dictionary and exchanges ObjectKeys between them. A per-C++-object table
// (mirroring UsdSource::mGeneration/mKeyToPath too literally) broke that:
// two sibling wrappers over the same live instance each restart their own
// local-index numbering from 1, so identical-looking packed keys from each
// wrapper would decode against the WRONG table -- not merely disagree on
// generation, but silently resolve to a different raw handle, exactly the
// aliasing class this ADR exists to close, just moved to same-attach
// siblings instead of cross-attach.
struct OvstageInternTable
{
    std::mutex mutex;
    uint32_t generation = 0;
    // 1-based local index (0 reserved invalid, matching ObjectKey{}'s default).
    std::unordered_map<uint64_t, uint32_t> rawToIndex;
    std::deque<uint64_t> indexToRaw;
};

namespace
{
// gInstanceTables refcounts each live instance pointer via the shared_ptr
// every referencing OvstageSource holds (see acquireOvstageInternTable):
// the first OvstageSource built over a given ovstage_instance_t* mints a
// fresh table with a fresh generation from nextObjectKeyGeneration()
// (Handles.h) -- a counter shared process-wide with UsdSource, not a
// per-backend one, so a key minted by a fresh OvstageSource can't alias one
// minted by a fresh UsdSource either, closing the cross-backend case of this
// same gap; every sibling built over the same still-live instance shares the
// same table -- so ObjectKeys mint/decode identically no matter which
// wrapper touches them. gInstanceTables itself holds only a weak_ptr: once
// the last OvstageSource referencing an instance is destroyed, the table is
// freed and the weak_ptr expires, so if that pointer value is later reused
// for a genuinely different attach (a fresh ovstage_instance_t after
// ovstage_destroy_instance/ovstage_create_instance happens to land at the
// same address), the next OvstageSource built over it mints a brand-new
// table/generation rather than inheriting the old one -- preserving the
// aliasing guard for the actual detach/reattach case this ADR exists for. A
// null instance (the header's documented "degrades to an empty scan" case)
// has no shared identity to key on, so it always mints its own table.
std::mutex gInstanceTablesMutex;
std::unordered_map<const void*, std::weak_ptr<OvstageInternTable>> gInstanceTables;

std::shared_ptr<OvstageInternTable> newInternTable()
{
    auto table = std::make_shared<OvstageInternTable>();
    table->generation = omni::physics::parse::nextObjectKeyGeneration();
    table->indexToRaw.emplace_back(); // reserve local index 0 (always-invalid sentinel)
    return table;
}

std::shared_ptr<OvstageInternTable> acquireOvstageInternTable(ovstage_instance_t* instance)
{
    if (!instance)
        return newInternTable();
    std::lock_guard<std::mutex> lock(gInstanceTablesMutex);
    // Prune every already-expired entry (not just one matching `instance`), so
    // repeated create/attach/detach/destroy churn at distinct addresses does
    // not grow the registry for process lifetime.
    for (auto it = gInstanceTables.begin(); it != gInstanceTables.end();)
    {
        if (it->second.expired())
            it = gInstanceTables.erase(it);
        else
            ++it;
    }
    auto it = gInstanceTables.find(instance);
    if (it != gInstanceTables.end())
    {
        if (std::shared_ptr<OvstageInternTable> existing = it->second.lock())
            return existing;
    }
    std::shared_ptr<OvstageInternTable> table = newInternTable();
    gInstanceTables[instance] = table;
    return table;
}
} // namespace

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

// Test-only count of buildChildCache() runs: lets a test assert that a structural
// drain patched the hierarchy cache instead of dropping and rebuilding it.
std::atomic_size_t& childCacheBuildCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only count of live per-schema membership queries (schemaMembershipFromQuery misses):
// a structural drain that patched the memo must not send the next hasSchema back to ovstage.
std::atomic_size_t& schemaQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only counts of the live round trips the drain fast paths avoid: the usd-path PREFIX child
// enumeration, the usd-path IN existence query, the 2-predicate isA type filter and the
// stage-wide schema presence probe.
std::atomic_size_t& liveChildQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& existsQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& isATypeQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& schemaPresenceQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only count of single-prim reads of the structural columns (usd-schemas / usd-prim-type in
// withAttributeTensor): the prim-type / per-prim schema memos the feed patches must answer a
// spawned prim without one.
std::atomic_size_t& structuralSinglePrimReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only counts of the whole-stage builds a warm source pays once: the usd-prim-type index and
// the attribute-column vocabulary; and of live single-prim relationship reads, which the
// vocabulary gate must spare an unauthored relationship.
std::atomic_size_t& primTypeIndexBuildCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& attributeVocabularyBuildCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& relationshipReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// Test-only counts of the round trips a warm incremental load must not multiply: columnar
// prefetchBucket reads, single-prim live reads (withAttributeTensor), and the whole-stage
// instance-material query the vocabulary gate spares.
std::atomic_size_t& prefetchBucketReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& liveAttributeReadCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

std::atomic_size_t& instanceMaterialQueryCounter()
{
    static std::atomic_size_t counter{ 0 };
    return counter;
}

// This local helper reports completion for strict schema reads; OvstageChangeFeed keeps its lenient copy.
bool waitAndRelease(ovstage_instance_t* inst, ovstage_enqueue_result_t r)
{
    if (r.status != OVSTAGE_OK || r.op_index == OVSTAGE_INVALID_OP_ID)
        return false;

    const ovstage_api_status_t waitStatus =
        ovstage_wait_op(inst, r.op_index, OVSTAGE_TIMEOUT_INFINITE, nullptr);
    if (waitStatus == OVSTAGE_OK || waitStatus == OVSTAGE_ERROR_OP_FAILED)
        (void)ovstage_release_op(inst, r.op_index);
    return waitStatus == OVSTAGE_OK;
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

void resetOvstageChildCacheBuildCountForTest()
{
    childCacheBuildCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageChildCacheBuildCountForTest()
{
    return childCacheBuildCounter().load(std::memory_order_relaxed);
}

void resetOvstageSchemaQueryCountForTest()
{
    schemaQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageSchemaQueryCountForTest()
{
    return schemaQueryCounter().load(std::memory_order_relaxed);
}

void resetOvstageLiveChildQueryCountForTest()
{
    liveChildQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageLiveChildQueryCountForTest()
{
    return liveChildQueryCounter().load(std::memory_order_relaxed);
}

void resetOvstageExistsQueryCountForTest()
{
    existsQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageExistsQueryCountForTest()
{
    return existsQueryCounter().load(std::memory_order_relaxed);
}

void resetOvstageIsATypeQueryCountForTest()
{
    isATypeQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageIsATypeQueryCountForTest()
{
    return isATypeQueryCounter().load(std::memory_order_relaxed);
}

void resetOvstageSchemaPresenceQueryCountForTest()
{
    schemaPresenceQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageSchemaPresenceQueryCountForTest()
{
    return schemaPresenceQueryCounter().load(std::memory_order_relaxed);
}

void resetOvstageStructuralSinglePrimReadCountForTest()
{
    structuralSinglePrimReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageStructuralSinglePrimReadCountForTest()
{
    return structuralSinglePrimReadCounter().load(std::memory_order_relaxed);
}

void resetOvstagePrimTypeIndexBuildCountForTest()
{
    primTypeIndexBuildCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstagePrimTypeIndexBuildCountForTest()
{
    return primTypeIndexBuildCounter().load(std::memory_order_relaxed);
}

void resetOvstageAttributeVocabularyBuildCountForTest()
{
    attributeVocabularyBuildCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageAttributeVocabularyBuildCountForTest()
{
    return attributeVocabularyBuildCounter().load(std::memory_order_relaxed);
}

void resetOvstageRelationshipReadCountForTest()
{
    relationshipReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageRelationshipReadCountForTest()
{
    return relationshipReadCounter().load(std::memory_order_relaxed);
}

void resetOvstagePrefetchBucketReadCountForTest()
{
    prefetchBucketReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstagePrefetchBucketReadCountForTest()
{
    return prefetchBucketReadCounter().load(std::memory_order_relaxed);
}

void resetOvstageLiveAttributeReadCountForTest()
{
    liveAttributeReadCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageLiveAttributeReadCountForTest()
{
    return liveAttributeReadCounter().load(std::memory_order_relaxed);
}

void resetOvstageInstanceMaterialQueryCountForTest()
{
    instanceMaterialQueryCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstageInstanceMaterialQueryCountForTest()
{
    return instanceMaterialQueryCounter().load(std::memory_order_relaxed);
}

// Whole-list fetches out of the path dictionary (compat shim + ReadListMemo misses), process-wide.
void resetOvstagePathListFetchCountForTest()
{
    ovstage_compat::pathListFetchCounter().store(0, std::memory_order_relaxed);
}

size_t getOvstagePathListFetchCountForTest()
{
    return ovstage_compat::pathListFetchCounter().load(std::memory_order_relaxed);
}

ReadListMemo::~ReadListMemo()
{
    for (const auto& entry : mEntries)
        if (entry.second.pinned)
            ovstage_compat::status(mDict, path_dictionary_release_path_list_reference(mDict, entry.first));
}

bool ReadListMemo::paths(ovx_primpath_list_t list, const ovx_primpath_t** outPaths, size_t* outCount)
{
    *outPaths = nullptr;
    *outCount = 0;
    if (!mDict)
        return false;
    const Entry* hit = nullptr;
    if (mLastEntry && list == mLastList)
        hit = mLastEntry;
    else if (const auto it = mEntries.find(list); it != mEntries.end())
        hit = &it->second;
    if (!hit)
    {
        Entry e;
        ovstage_compat::pathListFetchCounter().fetch_add(1, std::memory_order_relaxed);
        size_t count = 0;
        if (ovstage_compat::status(mDict, path_dictionary_get_num_paths_from_path_list(mDict, list, &count)) != OVX_OK)
            return false;
        e.paths.assign(count, OVX_INVALID_PRIMPATH);
        size_t fetched = 0;
        if (count > 0 && ovstage_compat::status(mDict, path_dictionary_get_paths_from_path_list(
                                                           mDict, list, 0, count, e.paths.data(), &fetched)) != OVX_OK)
            return false;
        e.paths.resize(fetched);
        e.pinned = ovstage_compat::status(mDict, path_dictionary_add_path_list_reference(mDict, list)) == OVX_OK;
        hit = &mEntries.emplace(list, std::move(e)).first->second; // node-based: stable address
    }
    mLastList = list;
    mLastEntry = hit;
    *outPaths = hit->paths.empty() ? nullptr : hit->paths.data();
    *outCount = hit->paths.size();
    return *outPaths != nullptr;
}

// Not declared in the public header; test translation units forward-declare
// this accessor. Reports gInstanceTables' current size so a churn test can
// assert it stays bounded rather than growing with every distinct instance
// address ever observed (see acquireOvstageInternTable's pruning above).
size_t getOvstageInternTableRegistrySizeForTest()
{
    std::lock_guard<std::mutex> lock(gInstanceTablesMutex);
    return gInstanceTables.size();
}

// ---------------------------------------------------------------------------

OvstageSource::OvstageSource(ovstage_instance_t* instance,
                                 ovx_path_dictionary_t* dict,
                                 ovstage_ordinal_t readOrdinal,
                                 uint64_t usdStageId)
    : mInstance(instance), mDict(dict), mReadOrdinal(readOrdinal), mLoadCacheMayBeStale(usdStageId),
      mInternTable(acquireOvstageInternTable(instance))
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

const KnownTokens* OvstageSource::knownTokens() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mKnownTokens)
    {
        auto k = std::make_unique<KnownTokens>();
        k->intern(*this); // re-enters doInternToken under the same recursive mutex
        mKnownTokens = std::move(k);
    }
    // Never reset: TokenIds are permanent for this source, so the batch stays valid.
    return mKnownTokens.get();
}

ovx_token_t OvstageSource::ovxToken(std::string_view s) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (auto it = mOvxTokenMemo.find(s); it != mOvxTokenMemo.end())
        return it->second;
    ovx_token_t tok = OVX_INVALID_TOKEN;
    const ovx_string_t str{ s.data(), s.size() };
    // Only a successful intern is permanent (see mOvxTokenMemo); a failed call is retried.
    if (ovx_path_dictionary_intern_token(mDict, str, &tok) == OVX_OK)
        mOvxTokenMemo.emplace(std::string{ s }, tok);
    return tok;
}

ObjectKey OvstageSource::internKey(uint64_t raw) const
{
    if (raw == 0)
        return ObjectKey{};
    // mInternTable is shared by every OvstageSource wrapping the same live
    // ovstage_instance_t (see OvstageInternTable's doc comment), so this locks
    // the table's own mutex, not mMutex -- a sibling wrapper on another thread
    // may be interning concurrently.
    std::lock_guard<std::mutex> lock(mInternTable->mutex);
    auto [it, inserted] = mInternTable->rawToIndex.try_emplace(raw, 0u);
    if (inserted)
    {
        mInternTable->indexToRaw.push_back(raw);
        it->second = static_cast<uint32_t>(mInternTable->indexToRaw.size() - 1);
    }
    return ObjectKey{ (static_cast<uint64_t>(mInternTable->generation) << 32) | it->second };
}

uint64_t OvstageSource::rawHandle(ObjectKey key) const
{
    if (key.handle == 0 || static_cast<uint32_t>(key.handle >> 32) != mInternTable->generation)
        return 0;
    const uint32_t localIndex = static_cast<uint32_t>(key.handle & 0xFFFFFFFFu);
    std::lock_guard<std::mutex> lock(mInternTable->mutex);
    if (localIndex == 0 || localIndex >= mInternTable->indexToRaw.size())
        return 0;
    return mInternTable->indexToRaw[localIndex];
}

uint64_t OvstageSource::canonicalHandleRaw(uint64_t raw) const
{
    if (raw == 0)
        return 0;
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const auto cached = mCanonicalHandleCache.find(raw);
    if (cached != mCanonicalHandleCache.end())
        return cached->second;

    const std::string p = pathOfRaw(raw);
    if (p.empty())
    {
        mCanonicalHandleCache[raw] = raw;
        return raw;
    }
    const uint64_t c = findByPathRaw(p);
    const uint64_t canonical = c ? c : raw;
    mCanonicalHandleCache[raw] = canonical;
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
        const std::string prototypePath = pathOfRaw(prototypeData[i]);
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
    std::vector<uint64_t> collisionKeys;
    if (!collectSchemaKeysRaw(internToken("PhysicsCollisionAPI"), collisionKeys))
        needsCompleteExpansion = true;

    std::unordered_set<std::string> relevantPrototypePathSet;
    std::vector<uint64_t> externalCollisionKeys;
    if (!needsCompleteExpansion)
    {
        for (const uint64_t key : collisionKeys)
        {
            bool foundPrototype = false;
            std::string ancestor = pathOfRaw(key);
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

    // Many external colliders (e.g. all instance proxies) would mean one live child
    // query per key below; past this point the whole-stage child cache is cheaper.
    if (externalCollisionKeys.size() > 64)
        buildChildCache();

    if (!externalCollisionKeys.empty())
    {
        // A scoped per-key lookup, not buildChildCache(): this only needs a
        // leaf/non-leaf check on the (typically few) external collision keys
        // themselves, not the whole stage's edge set. forEachChildScopedRaw
        // reuses the eager cache for free if something else already built it,
        // otherwise issues a scoped query. It returns whether that read was
        // authoritative: silence (no child emitted) only proves a leaf when the
        // read completed a clean pass. A failed / truncated read must fall back
        // to complete expansion, or a non-leaf collider's child shapes would be
        // silently dropped.
        for (const uint64_t key : externalCollisionKeys)
        {
            bool hasChild = false;
            auto noteChild = [&hasChild](uint64_t) { hasChild = true; };
            bool authoritative = forEachChildScopedRaw(key, noteChild);
            if (!hasChild && authoritative)
            {
                const uint64_t canonical = canonicalHandleRaw(key);
                if (canonical && canonical != key)
                    authoritative = forEachChildScopedRaw(canonical, noteChild);
            }
            if (hasChild || !authoritative)
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
        const uint64_t prototypeRoot = findByPathRaw(prototypePath);
        if (!prototypeRoot)
            return false;

        ScopedPathList instanceList(instancingDict);
        instanceRootQueryCounter().fetch_add(1, std::memory_order_relaxed);
        if (ovstage_instancing_get_instance_roots(mInstance, prototypeRoot, instanceList.receive()) != OVSTAGE_OK)
            return false;

        const ovx_primpath_t* instanceData = nullptr;
        size_t instanceCount = 0;
        if (ovx_path_dictionary_get_paths(instancingDict, instanceList.get(), &instanceData, &instanceCount) != OVX_OK)
            return false;

        for (size_t i = 0; i < instanceCount; ++i)
        {
            const std::string instancePath = pathOfRaw(instanceData[i]);
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

    // No prim on the stage authors the column (stage-wide vocabulary): the HAS query below
    // could match nothing, so the cache is exactly empty without the round trip.
    if (stageHasAttributeColumn("_instanceMaterialBinding") == std::optional<bool>(false))
    {
        mInstanceMaterialCacheValid = true;
        return true;
    }

    const ovx_token_t materialAttr = ovxToken("_instanceMaterialBinding");
    if (materialAttr == OVX_INVALID_TOKEN)
        return false;
    instanceMaterialQueryCounter().fetch_add(1, std::memory_order_relaxed);

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
    ReadListMemo listMemo(mDict);
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
        if (!listMemo.paths(group.prims.list, &primPaths, &primPathCount))
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
            const uint64_t canonical = canonicalHandleRaw(prim);
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
    return internKey(geometryBackingKeyRaw(rawHandle(key)));
}

ObjectKey OvstageSource::collisionAttributeBackingKey(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const uint64_t raw = rawHandle(key);
    if (!raw || !buildPhysicsInstancingCache())
        return {};
    if (mPrototypeRootByInstanceRoot.empty())
        return {};

    const std::string logicalPath = pathOfRaw(raw);
    if (logicalPath.empty())
        return {};

    std::string ancestor = logicalPath;
    while (true)
    {
        const size_t slash = ancestor.rfind('/');
        if (slash == std::string::npos || slash == 0)
            return {};
        ancestor.resize(slash);
        const std::unordered_map<std::string, std::string>::const_iterator root =
            mPrototypeRootByInstanceRoot.find(ancestor);
        if (root != mPrototypeRootByInstanceRoot.end())
            return internKey(findByPathRaw(root->second + logicalPath.substr(ancestor.size())));
    }
}

uint64_t OvstageSource::geometryBackingKeyRaw(uint64_t key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key || !buildPhysicsInstancingCache())
        return key;

    const uint64_t canonical = canonicalHandleRaw(key);
    const uint64_t cacheKey = canonical ? canonical : key;
    std::unordered_map<uint64_t, uint64_t>::const_iterator cached = mGeometryBackingCache.find(cacheKey);
    if (cached != mGeometryBackingCache.end())
        return cached->second;

    const std::string logicalPath = pathOfRaw(key);
    if (logicalPath.empty())
        return key;

    auto hasPrimType = [this](uint64_t candidate) -> bool
    {
        TokenId primType{};
        if (!getAttribute(internKey(candidate), doInternToken(conv::kUsdPrimType), primType))
            return false;
        const ovx_string_t untyped = ovstage_population_untyped_type_name();
        return tokenToString(primType) != std::string_view(untyped.ptr, untyped.length);
    };

    std::string ancestor = logicalPath;
    uint64_t backingKey = key;
    bool lookupComplete = true;
    while (!ancestor.empty())
    {
        std::unordered_map<std::string, std::string>::const_iterator root =
            mPrototypeRootByInstanceRoot.find(ancestor);

        // The attach walk only expands prototypes that can back collision
        // shapes. Public type/geometry helpers must still resolve a render-only
        // instance proxy, so after the load lazily use OVStage's reverse
        // lookup for map misses. The current provider rebuilds the complete
        // instancing graph per call, so this must stay out of the attach
        // entirely: not just the scan walk (mLoadCacheActive) but also the
        // consumer's processScannedDescs / cooking pass, which runs on the
        // AttachedStage's own source inside a hierarchy bulk-read window and
        // only asks about physics-scoped keys the map above already covers
        // (NVBug 6532970). A successful post-load result is cached so each
        // queried proxy pays its ancestor probes only once. OMPE-100947 tracks
        // the batched API.
        if (root == mPrototypeRootByInstanceRoot.end() && !mLoadCacheActive && !mHierarchyBulkRead &&
            !mPrototypeRootPaths.empty())
        {
            const uint64_t instanceRoot = findByPathRaw(ancestor);
            if (!instanceRoot)
            {
                lookupComplete = false;
                break;
            }

            ovx_primpath_t prototypeRoot = OVX_INVALID_PRIMPATH;
            prototypeRootQueryCounter().fetch_add(1, std::memory_order_relaxed);
            const ovstage_api_status_t status =
                ovstage_instancing_get_prototype_root(mInstance, instanceRoot, &prototypeRoot);
            if (status == OVSTAGE_OK)
            {
                const std::string prototypePath = pathOfRaw(prototypeRoot);
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
            const uint64_t resolved = findByPathRaw(backingPath);

            // A prim inside an instance can itself be tagged as an instance root.
            // That tag names the prototype root standing in for the whole
            // subtree, so the empty path remainder lands on a typeless container
            // rather than on geometry backing this gprim. Recognize it by the
            // missing or reserved untyped name and keep climbing to the enclosing instance root,
            // whose prototype does carry the corresponding child. A leaf gprim
            // that is genuinely its own instance root keeps its type, so it still
            // resolves here.
            const bool selfTagged = ancestor.size() == logicalPath.size();
            if (resolved && (!selfTagged || hasPrimType(resolved)))
            {
                backingKey = resolved;
                break;
            }
            if (!selfTagged)
            {
                lookupComplete = false;
                break;
            }
            if (!resolved)
            {
                // Not found at all is the same class of transient/incomplete
                // result as the other failure paths above: retry, don't cache.
                lookupComplete = false;
            }
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
        mGeometryBackingCache[cacheKey] = backingKey;
        if (key != cacheKey)
            mGeometryBackingCache[key] = backingKey;
    }
    return backingKey;
}

bool OvstageSource::isPrototypeBackingKey(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const uint64_t raw = rawHandle(key);
    if (!raw || !buildPrototypeRootCache() || mPrototypeRootPaths.empty())
        return false;

    std::string ancestor = pathOfRaw(raw);
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

std::string OvstageSource::pathOfRaw(uint64_t key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mDict || !key)
        return {};
    const auto cached = mPathStringCache.find(key);
    if (cached != mPathStringCache.end())
        return cached->second;

    ovx_string_t s{};
    if (ovx_path_dictionary_path_to_string(mDict, key, &s) != OVX_OK || !s.ptr)
        return {};
    std::string path(s.ptr, s.length);
    mPathStringCache[key] = path;
    return path;
}

std::string_view OvstageSource::sourceKeyToString(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    // Intern the resolved path string in our own table so the returned view has
    // stable lifetime (the dictionary's string view is also stable, but routing
    // through internToken keeps one ownership story).
    const std::string p = pathOfRaw(rawHandle(key));
    if (p.empty())
        return {};
    return tokenToString(doInternToken(p));
}

// --- traversal -------------------------------------------------------------

ObjectKey OvstageSource::findByPath(std::string_view path) const
{
    return internKey(findByPathRaw(path));
}

uint64_t OvstageSource::findByPathRaw(std::string_view path) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mDict || path.empty())
        return 0;
    const std::string key(path);
    const auto cached = mPathToHandleCache.find(key);
    if (cached != mPathToHandleCache.end())
        return cached->second;

    ovx_primpath_t p = OVX_INVALID_PRIMPATH;
    const ovx_string_t str{ key.data(), key.size() };
    if (ovx_path_dictionary_intern_path(mDict, str, &p) != OVX_OK)
        return 0;
    mPathToHandleCache.emplace(key, p);
    if (p != OVX_INVALID_PRIMPATH)
        mPathStringCache.emplace(p, key);
    return p;
}

ObjectKey OvstageSource::getRootKey() const
{
    return findByPath("/");
}

ObjectKey OvstageSource::getParent(ObjectKey key) const
{
    return internKey(getParentRaw(rawHandle(key)));
}

uint64_t OvstageSource::getParentRaw(uint64_t key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key)
        return 0;
    const auto cached = mParentHandleCache.find(key);
    if (cached != mParentHandleCache.end())
        return cached->second;

    const std::string p = pathOfRaw(key);
    if (p.size() <= 1)
    {
        mParentHandleCache[key] = 0;
        return 0; // "/" or empty has no parent
    }
    const size_t slash = p.rfind('/');
    if (slash == std::string::npos)
    {
        mParentHandleCache[key] = 0;
        return 0;
    }
    const std::string parent = (slash == 0) ? "/" : p.substr(0, slash);
    const uint64_t parentKey = findByPathRaw(parent);
    mParentHandleCache[key] = parentKey;
    const uint64_t canonical = canonicalHandleRaw(key);
    if (canonical && canonical != key)
        mParentHandleCache[canonical] = parentKey;
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
    childCacheBuildCounter().fetch_add(1, std::memory_order_relaxed);

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
    // The initial scan is a snapshot at the caller's requested ordinal. After
    // that scan, hierarchy invalidation means a structural edit occurred, so a
    // rebuild must observe the latest topology rather than replaying the attach
    // ordinal. This matches forEachChild()'s post-load live-query semantics.
    range.end_ordinal = mLoadCacheActive ? mReadOrdinal : ~ovstage_ordinal_t(0);
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
        ReadListMemo listMemo(mDict);
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
            if (!listMemo.paths(g.prims.list, &parentPaths, &parentCount))
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
        ReadListMemo listMemo(mDict);
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
            if (!listMemo.paths(g.prims.list, &childPaths, &childCount))
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
            ReadListMemo listMemo(mDict);
            ovstage_read_group_t g{};
            ovstage_api_status_t fetchStatus = OVSTAGE_OK;
            while ((fetchStatus = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
            {
                const ovx_primpath_t* paths = nullptr;
                size_t count = 0;
                if (listMemo.paths(g.prims.list, &paths, &count))
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
        std::string leafPath = pathOfRaw(pathHandle);
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
            const uint64_t childCanonical = canonicalHandleRaw(findByPathRaw(childPath));
            const uint64_t parentCanonical = canonicalHandleRaw(findByPathRaw(parentPath));
            if (parentCanonical && childCanonical)
                insertChild(parentCanonical, childCanonical);
            childPath = parentPath;
        }
    }

    // Impose a stage-derived order on every parent's child list.
    //
    // ovstage does not publish prims in a reproducible order. Its stored order is
    // bucketed by attribute set / prim type, and the BUCKET order varies with
    // process history rather than with the stage: for the same stage, the same
    // blocks of siblings come back permuted depending on what was attached earlier
    // in the process. This was measured on all three routes into this cache — the
    // `usd-children` array column, the `usd-parent` scalar column, and ovstage's
    // dedicated ovstage_get_hierarchy(OVSTAGE_HIERARCHY_CHILDREN) API — which all
    // returned the same set of siblings in different orders across two runs of the
    // same binary. So this is ovstage's own ordering, not an artifact of how the
    // columns are unioned here.
    //
    // Child order is load-bearing well beyond traversal: the walker keys its
    // enumeration buckets off this hierarchy, and Articulation.cpp turns the
    // resulting scan order into PhysX articulation link and DOF indices. Left
    // unordered, an identical asset yields different DOF indices in two processes
    // — silently, with all counts matching. Full analysis: the
    // `TestTensorArticulationView.cpp` entry in
    // `ovphysx/ovruntime/plc/plans/PLAN-ovstage-test-coverage-completion.md` §6.
    //
    // Sorting by path makes the order a pure function of the stage. NOTE: it is
    // NOT USD authoring order, and cannot be — ovstage exposes no authoring-order
    // signal on any of the three routes above. Parity with a USD-sourced scan
    // needs that ordering supplied upstream by ovstage.
    for (std::pair<const uint64_t, std::vector<uint64_t>>& entry : mChildCache)
    {
        std::vector<uint64_t>& children = entry.second;
        if (children.size() < 2)
            continue;
        std::vector<std::pair<std::string, uint64_t>> sorted;
        sorted.reserve(children.size());
        for (const uint64_t child : children)
            sorted.emplace_back(pathOfRaw(child), child);
        std::sort(sorted.begin(), sorted.end(),
                  [](const std::pair<std::string, uint64_t>& a, const std::pair<std::string, uint64_t>& b)
                  {
                      // The handle is only a tiebreak for the degenerate case of two
                      // handles resolving to one path; it keeps the ordering total.
                      if (a.first != b.first)
                          return a.first < b.first;
                      return a.second < b.second;
                  });
        for (size_t i = 0; i < sorted.size(); ++i)
            children[i] = sorted[i].second;
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
    std::vector<uint64_t> raw;
    collectDescendantKeysRaw(rawHandle(root), raw);
    out.clear();
    out.reserve(raw.size());
    for (const uint64_t handle : raw)
        out.push_back(internKey(handle));
}

void OvstageSource::collectDescendantKeysRaw(uint64_t root, std::vector<uint64_t>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    out.clear();
    if (!root || !mInstance || !mDict)
        return;

    if (!mChildCacheBuilt)
        buildChildCache();

    // Canonical key first: applyHierarchyDelta() inserts under canonical handles, while the
    // usd-children/usd-parent build routes may have keyed an alias list by raw handle.
    uint64_t rootHandle = canonicalHandleRaw(root);
    if (rootHandle != root && mChildCache.find(rootHandle) == mChildCache.end() &&
        mChildCache.find(root) != mChildCache.end())
        rootHandle = root;
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
            out = cached->second;
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

    out = handles;

    // Memoize only a cleanly-built traversal; see the note above.
    if (mChildCacheComplete)
        mDescendantCache[rootHandle] = std::move(handles);
}

void OvstageSource::forEachChild(ObjectKey parent, std::function<void(ObjectKey)> cb) const
{
    if (!cb)
        return;
    forEachChildRaw(rawHandle(parent), [this, &cb](uint64_t child) { cb(internKey(child)); });
}

void OvstageSource::forEachChildRaw(uint64_t parent, const std::function<void(uint64_t)>& cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!parent || !cb)
        return;
    const std::string parentPath = pathOfRaw(parent);
    if (parentPath.empty())
        return;

    std::unordered_set<uint64_t> emittedChildren;
    auto emitChild = [&](uint64_t child)
    {
        if (!child)
            return;
        const uint64_t childHandle = canonicalHandleRaw(child);
        if (childHandle && !emittedChildren.insert(childHandle).second)
            return;
        cb(child);
    };

    if (mInstance && mDict)
    {
        if (!mChildCacheBuilt)
            buildChildCache();
        // Canonical key first (see collectDescendantKeysRaw); the raw key only serves an
        // alias list the build keyed by raw handle.
        const uint64_t parentCanonical = canonicalHandleRaw(parent);
        std::unordered_map<uint64_t, std::vector<uint64_t>>::const_iterator it = mChildCache.find(parentCanonical);
        if (it == mChildCache.end() && parentCanonical != parent)
            it = mChildCache.find(parent);
        if (it != mChildCache.end())
        {
            for (const uint64_t child : it->second)
                emitChild(child);
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
        // Only inside a bulk-read window (mLoadCacheActive or beginHierarchyBulkRead()):
        // outside one a caller could observe a structural edit before the change feed
        // invalidates the hierarchy cache, so fall through to the live prefix query.
        // mChildCacheComplete additionally guards against a transient/partial
        // cache build being treated as authoritative.
        // Only prims the bulk build never saw (parentCanonical unknown) fall
        // through even during load.
        if ((mLoadCacheActive || mHierarchyBulkRead) && mChildCacheComplete && parentCanonical &&
            mParentHandleCache.find(parentCanonical) != mParentHandleCache.end())
        {
            return;
        }

        // The ovstage-wide cache has no children for this prim; fall through to
        // the usd-path prefix query below to enumerate them natively.
    }

    if (!mInstance || !mDict)
        return;

    liveQueryDirectChildrenRaw(parent, parentPath, emitChild);
}

bool OvstageSource::forEachChildScopedRaw(uint64_t parent, const std::function<void(uint64_t)>& cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!parent || !cb)
        return false;

    // Some other caller already paid for the whole-stage build (e.g. a broad
    // recursive walk elsewhere in this same attach); reuse it for free instead
    // of issuing a redundant scoped query. That reuse is authoritative only when
    // the eager build completed a clean pass (mChildCacheComplete); a partial
    // build cannot certify a leaf, so report non-authoritative and let the
    // caller fall back.
    if (mChildCacheBuilt)
    {
        forEachChildRaw(parent, cb);
        return mChildCacheComplete;
    }

    const std::string parentPath = pathOfRaw(parent);
    if (parentPath.empty() || !mInstance || !mDict)
        return false;

    std::unordered_set<uint64_t> emittedChildren;
    auto emitChild = [&](uint64_t child)
    {
        if (!child)
            return;
        const uint64_t childHandle = canonicalHandleRaw(child);
        if (childHandle && !emittedChildren.insert(childHandle).second)
            return;
        cb(child);
    };
    return liveQueryDirectChildrenRaw(parent, parentPath, emitChild);
}

bool OvstageSource::liveQueryDirectChildrenRaw(uint64_t parent, const std::string& parentPath,
                                                const std::function<void(uint64_t)>& emit) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    (void)parent;
    if (!mInstance || !mDict)
        return false;
    liveChildQueryCounter().fetch_add(1, std::memory_order_relaxed);

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
        return false;
    waitAndRelease(mInstance, e);

    // usd-path is populated for every queryable row, so reading it is the
    // authoritative enumeration of the matched subtree: a clean pass over it
    // distinguishes a real leaf (zero matched prims) from a subtree whose only
    // columns are the skipped reserved ones. Probe it unconditionally, in
    // addition to whatever data columns the result exposes, and use its
    // clean-termination status as the "was this read authoritative" signal.
    ovx_token_t usdPathProbe = OVX_INVALID_TOKEN;
    ovx_path_dictionary_intern_token(
        mDict, ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() }, &usdPathProbe);

    ovstage_query_handle_t use = q;
    std::vector<ovx_token_t> probes;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        for (size_t i = 0; i < qr.attribute_count; ++i)
        {
            if (usdPathProbe != OVX_INVALID_TOKEN && qr.attributes[i] == usdPathProbe)
                continue; // appended explicitly below as the authoritative probe
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(mDict, qr.attributes[i], &s) != OVX_OK || !s.ptr || s.length == 0)
                continue;
            if (s.ptr[0] == '_' || (s.length >= 4 && std::strncmp(s.ptr, "usd-", 4) == 0))
                continue;
            probes.push_back(qr.attributes[i]);
        }
        ovstage_release_query_result(mInstance, &qr);
    }
    else
    {
        // Could not read the match set at all -- not authoritative.
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }
    if (usdPathProbe != OVX_INVALID_TOKEN)
        probes.push_back(usdPathProbe);
    if (probes.empty())
    {
        // No readable column, not even usd-path -- cannot conclude anything.
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }

    ovstage_ordinal_range_t range{};
    range.end_ordinal = ~ovstage_ordinal_t(0); // latest
    range.has_start_ordinal = false;

    // Collect first, emit in path order. The read groups arrive in ovstage's
    // process-history-dependent order (see the ordering note in buildChildCache),
    // so emitting each child as it is fetched would leave this live fallback with
    // an irreproducible order even though the cached path above is sorted.
    std::vector<std::pair<std::string, uint64_t>> matched;
    std::unordered_set<uint64_t> matchedSeen; // usd-path + data probes can overlap
    bool authoritativeClean = false;
    for (const ovx_token_t probe : probes)
    {
        const bool isAuthoritative = (usdPathProbe != OVX_INVALID_TOKEN && probe == usdPathProbe);
        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &probe, 1, range, &rh);
        if (re.status != OVSTAGE_OK)
            continue;

        waitAndRelease(mInstance, re);
        ReadListMemo listMemo(mDict);
        ovstage_read_group_t g{};
        ovstage_api_status_t fetchStatus = OVSTAGE_OK;
        while ((fetchStatus = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
        {
            const ovx_primpath_t* paths = nullptr;
            size_t count = 0;
            if (listMemo.paths(g.prims.list, &paths, &count))
            {
                for (uint32_t i = 0; i < g.prims.count; ++i)
                {
                    const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                    if (idx >= count)
                        continue;
                    const uint64_t childKey = paths[idx];
                    if (!matchedSeen.insert(childKey).second)
                        continue;
                    std::string childPath = pathOfRaw(childKey);
                    if (childPath.size() > prefix.size() &&
                        childPath.compare(0, prefix.size(), prefix) == 0 &&
                        childPath.find('/', prefix.size()) == std::string::npos)
                    {
                        matched.emplace_back(std::move(childPath), childKey);
                    }
                }
            }
            ovstage_release_group(mInstance, &g);
        }
        // A clean pass over the authoritative usd-path column ends exactly at
        // END_OF_ITERATION; any other terminal status means the enumeration was
        // truncated, so "no children" is not a trustworthy leaf conclusion.
        if (isAuthoritative && fetchStatus == OVSTAGE_ERROR_END_OF_ITERATION)
            authoritativeClean = true;
        waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    }
    std::sort(matched.begin(), matched.end(),
              [](const std::pair<std::string, uint64_t>& a, const std::pair<std::string, uint64_t>& b)
              {
                  if (a.first != b.first)
                      return a.first < b.first;
                  return a.second < b.second;
              });
    for (const std::pair<std::string, uint64_t>& m : matched)
        emit(m.second);
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    return authoritativeClean;
}

bool OvstageSource::exists(ObjectKey key) const
{
    return existsRaw(rawHandle(key));
}

bool OvstageSource::existsRaw(uint64_t key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key)
        return false;

    // During change-feed dispatch the source bucket is seeded from the same
    // ovstage read group that produced the changed keys. Membership there is
    // enough to answer the runtime "is this changed prim still live?" guard
    // without issuing a per-key usd-path query.
    if (mBucketKeys.count(key) != 0)
    {
        return true;
    }
    if (mKnownKeys.count(key) != 0)
    {
        return true;
    }
    if (const uint64_t canonical = canonicalHandleRaw(key))
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

    // Keys the bucket / known set does not cover answer from the per-key memo instead
    // of repeating the round trip. Dropped on the same structural edits that already
    // invalidate mParentHandleCache / mChildCache.
    const auto memoized = mExistsMemo.find(key);
    if (memoized != mExistsMemo.end())
        return memoized->second;

    const std::string path = pathOfRaw(key);
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

    existsQueryCounter().fetch_add(1, std::memory_order_relaxed);
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
    mExistsMemo[key] = (count >= 1);
    return count >= 1;
}

void OvstageSource::existsBatch(const std::vector<ObjectKey>& keys, std::vector<bool>& outExists) const
{
    (void)existsBatchChecked(keys, outExists);
}

bool OvstageSource::existsBatchChecked(const std::vector<ObjectKey>& keys, std::vector<bool>& outExists) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    outExists.assign(keys.size(), false);

    // Same local cache gates as exists(), applied per key first: only keys that
    // miss every cache -- the cold candidates that would otherwise each cost a
    // live round trip -- go into the batched query below. coldPathIndices maps
    // each distinct cold path string back to every `keys` index that carries it,
    // so a caller-side duplicate path is resolved once but marked everywhere.
    std::unordered_map<std::string, std::vector<size_t>> coldPathIndices;
    std::vector<size_t> coldIndices;
    for (size_t i = 0; i < keys.size(); ++i)
    {
        const uint64_t key = rawHandle(keys[i]);
        if (!key)
            continue;
        if (mBucketKeys.count(key) != 0 || mKnownKeys.count(key) != 0)
        {
            outExists[i] = true;
            continue;
        }
        if (const uint64_t canonical = canonicalHandleRaw(key))
        {
            if (mBucketKeys.count(canonical) != 0 || mKnownKeys.count(canonical) != 0)
            {
                outExists[i] = true;
                continue;
            }
        }
        // See existsRaw(): keys already resolved answer from the memo.
        const auto memoized = mExistsMemo.find(key);
        if (memoized != mExistsMemo.end())
        {
            outExists[i] = memoized->second;
            continue;
        }
        const std::string path = pathOfRaw(key);
        if (path.empty())
            continue;
        coldPathIndices[path].push_back(i);
        coldIndices.push_back(i);
    }

    // Record the resolved answers so a caller that batches once primes the per-key memo
    // that the single-key exists() reads.
    auto memoizeResults = [&]()
    {
        for (const size_t i : coldIndices)
            mExistsMemo[rawHandle(keys[i])] = outExists[i];
    };

    if (coldPathIndices.empty())
        return true;
    if (!mInstance || !mDict)
        return false;

    // One `usd-path IN [p1..pN]` query covers every cold path in a single round
    // trip -- the same predicate shape exists() uses for a single key, just with
    // value_count == the distinct cold-path count instead of 1.
    std::vector<ovx_string_t> pathVals;
    pathVals.reserve(coldPathIndices.size());
    for (const std::pair<const std::string, std::vector<size_t>>& kv : coldPathIndices)
        pathVals.push_back(ovx_string_t{ kv.first.data(), kv.first.size() });

    ovstage_predicate_t pred{};
    pred.attribute.token = 0;
    pred.attribute.string = ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() };
    pred.op = OVSTAGE_FILTER_OP_IN;
    pred.values = pathVals.data();
    pred.value_count = pathVals.size();

    ovstage_filter_t filter{};
    filter.predicates = &pred;
    filter.count = 1;

    existsQueryCounter().fetch_add(1, std::memory_order_relaxed);
    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    const ovstage_enqueue_result_t e = ovstage_query(mInstance, &filter, nullptr, 0, &q);
    if (e.status != OVSTAGE_OK)
    {
        return false;
    }
    waitAndRelease(mInstance, e);

    // total_prim_count alone would say HOW MANY of the cold paths matched, not
    // WHICH -- a partial match is the common case for this caller (pattern
    // matching over a mixed literal-path candidate list), so read the matched
    // set's usd-path column back (same technique buildChildCache's live fallback
    // uses to resolve a read group's prims to path handles) and mark exactly the
    // requested indices whose path came back.
    ovstage_query_handle_t use = q;
    size_t totalMatched = 0;
    bool fetched = false;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
    {
        fetched = true;
        if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
            use = qr.all_handle;
        totalMatched = qr.total_prim_count;
        ovstage_release_query_result(mInstance, &qr);
    }

    if (!fetched)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }
    if (totalMatched == 0)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        memoizeResults();
        return true;
    }

    bool resolved = false;
    ovx_token_t usdPathProbe = OVX_INVALID_TOKEN;
    ovx_path_dictionary_intern_token(
        mDict, ovx_string_t{ conv::kUsdPath, std::string_view(conv::kUsdPath).size() }, &usdPathProbe);
    if (usdPathProbe != OVX_INVALID_TOKEN)
    {
        ovstage_ordinal_range_t range{};
        range.end_ordinal = ~ovstage_ordinal_t(0); // latest
        range.has_start_ordinal = false;

        ovstage_read_handle_t rh = OVSTAGE_INVALID_READ_HANDLE;
        const ovstage_enqueue_result_t re = ovstage_read_attributes(mInstance, use, &usdPathProbe, 1, range, &rh);
        if (re.status == OVSTAGE_OK)
        {
            waitAndRelease(mInstance, re);
            ReadListMemo listMemo(mDict);
            ovstage_read_group_t g{};
            ovstage_api_status_t fetchErr;
            while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
            {
                const ovx_primpath_t* paths = nullptr;
                size_t count = 0;
                if (listMemo.paths(g.prims.list, &paths, &count))
                {
                    for (uint32_t i = 0; i < g.prims.count; ++i)
                    {
                        const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                        if (idx >= count)
                            continue;
                        const std::string matchedPath = pathOfRaw(paths[idx]);
                        const auto it = coldPathIndices.find(matchedPath);
                        if (it != coldPathIndices.end())
                        {
                            for (const size_t idxOut : it->second)
                                outExists[idxOut] = true;
                        }
                    }
                }
                ovstage_release_group(mInstance, &g);
            }
            resolved = fetchErr == OVSTAGE_ERROR_END_OF_ITERATION;
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        }
    }
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    if (resolved)
        memoizeResults();
    return resolved;
}

// --- read core -------------------------------------------------------------

bool OvstageSource::withAttributeTensor(uint64_t key,
                                        std::string_view attrName,
                                        const std::function<void(const DLTensor&, uint32_t)>& fn) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mInstance || !mDict || !key)
        return false;
    if (attrName == conv::kUsdSchemas || attrName == conv::kUsdPrimType)
        structuralSinglePrimReadCounter().fetch_add(1, std::memory_order_relaxed);
    liveAttributeReadCounter().fetch_add(1, std::memory_order_relaxed);

    // Canonicalize the handle to the intern_path space. Scan-emitted raw handles
    // carry ENUMERATE (get_paths) handles, while writes (ovstage API, runtime
    // change drivers) land on the prim's canonical intern_path handle. A raw
    // enumerate-handle read misses those writes, so a value edit applied after the
    // scan would never be seen (the bucket fast path already canonicalizes for the
    // same reason). Round-tripping through canonicalHandleRaw keeps both in one space.
    const uint64_t canon = canonicalHandleRaw(key);
    ovx_primpath_t p = canon ? canon : key;
    ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
    if (ovx_path_dictionary_create_path_list(mDict, &p, 1, &list) != OVX_OK)
        return false;

    bool got = false;
    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(mInstance, list, &q) == OVSTAGE_OK && q != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ovx_token_t attrTok = ovxToken(conv::toOvstageColumn(attrName));
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
                // Single-path read: logical row 0 is the requested prim. Guard the row
                // count before indexing the per-logical-element metadata (an empty match
                // can still carry a tensor, so index_map[0] / mask[0] would read past a
                // zero-length map). index_map and mask are mutually exclusive.
                if (!got && !g.is_delete && g.prims.count > 0 && g.data.tensor_count > 0 && g.data.tensors)
                {
                    // mask is a logical-row presence bitset: if row 0's bit is clear the
                    // value is absent for this prim, so leave it to the caller's fallback.
                    const bool present = (g.data.mask == nullptr) || ((g.data.mask[0] & 1u) != 0);
                    // index_map remaps logical row 0 (gather/dedup/reorder); identity otherwise.
                    const uint32_t logical0 = g.data.index_map ? g.data.index_map[0] : 0;
                    if (present && g.is_array)
                    {
                        // Array columns transport one tensor per logical row; select the
                        // MAPPED tensor (a gathered/deduplicated single-path array can map
                        // row 0 to a tensor other than tensors[0]) and pass it through.
                        if (logical0 < g.data.tensor_count && g.data.tensors[logical0].data)
                        {
                            fn(g.data.tensors[logical0], g.prims.count);
                            got = true;
                        }
                    }
                    else if (present)
                    {
                        const DLTensor& t = g.data.tensors[0];
                        if (g.data.index_map == nullptr)
                        {
                            // Identity (or masked-present, since mask/index_map are exclusive):
                            // tensors[0] is this prim's own row, so pass it through and let the
                            // caller derive the component count from prims.count. Byte-identical
                            // to the pre-fix behavior for the common path.
                            if (t.data)
                            {
                                fn(t, g.prims.count);
                                got = true;
                            }
                        }
                        else
                        {
                            // Gathered: tensors[0] is a WIDER stacked tensor holding more rows
                            // than this single-path group references, so index_map[0] is the ROW
                            // and total/prims.count would mis-stride. Narrow to that row (mirrors
                            // prefetchBucket) so the caller's row-0 decode reads the right value.
                            const int64_t storedRows = (t.ndim >= 1 && t.shape && t.shape[0] > 0) ? t.shape[0] : 0;
                            const int64_t total = totalElements(t);
                            if (t.data && storedRows > 0 && total > 0 && (total % storedRows) == 0 &&
                                static_cast<int64_t>(logical0) < storedRows)
                            {
                                const int64_t comps = total / storedRows;
                                int64_t oneShape = comps;
                                DLTensor one = t;
                                one.ndim = 1;
                                one.shape = &oneShape;
                                one.strides = nullptr;
                                one.dtype.lanes = 1;
                                one.byte_offset = t.byte_offset +
                                    static_cast<int64_t>(logical0) * comps * static_cast<int64_t>(t.dtype.bits / 8);
                                fn(one, 1);
                                got = true;
                            }
                        }
                    }
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
    return getAttributeAtTimeRaw(rawHandle(key), attr, ReadTime::defaultTime());
}

AttrValue OvstageSource::getAttributeAtTime(ObjectKey key, TokenId attr, ReadTime time) const
{
    return getAttributeAtTimeRaw(rawHandle(key), attr, time);
}

AttrValue OvstageSource::getAttributeAtTimeRaw(uint64_t key, TokenId attr, ReadTime time) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const std::string_view name = tokenToString(attr);
    if (name.empty())
        return {};

    // ADR-0017: `time` is deliberately unused. The ovstage data plane holds one value per
    // attribute per ordinal, so there is nothing to interpolate between -- the producer has
    // already advanced and sealed the ordinal for the intended time, and this read returns
    // that snapshot for every ReadTime mode.
    (void)time;

    if (mLoadCacheActive && key)
    {
        enum class LoadCacheLookup
        {
            eUnknown,
            eValue,
            eCoveredMiss,
        };
        auto tryHandle = [&](uint64_t handle, AttrValue& out) -> LoadCacheLookup
        {
            const LoadCacheCell* cell = loadCacheCell(handle, attr.id);
            if (!cell)
                return LoadCacheLookup::eUnknown;
            const CachedTensorRow& row = cell->scalar;
            if (row.tensor && row.tensor->data)
            {
                out = decodeAt(*row.tensor, row.comps, row.row);
                if (out.valid())
                    return LoadCacheLookup::eValue;
            }
            return cell->covered ? LoadCacheLookup::eCoveredMiss : LoadCacheLookup::eUnknown;
        };

        AttrValue cached;
        const uint64_t raw = key;
        if (raw)
        {
            const LoadCacheLookup result = tryHandle(raw, cached);
            if (result == LoadCacheLookup::eValue)
                return cached;
            if (result == LoadCacheLookup::eCoveredMiss)
                return {};
        }

        const uint64_t canonical = canonicalHandleRaw(key);
        if (canonical && canonical != raw)
        {
            const LoadCacheLookup result = tryHandle(canonical, cached);
            if (result == LoadCacheLookup::eValue)
                return cached;
            if (result == LoadCacheLookup::eCoveredMiss)
                return {};
        }
    }

    // Bucket fast path: serve from the columnar cache when this prim is in the
    // active bucket. Change-feed buckets keep the live read group and materialize
    // only requested rows; prefetch buckets are already decoded.
    if (mBucketActive && key)
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
        const uint64_t raw = key;
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
            const uint64_t ch = canonicalHandleRaw(key);
            if (ch && ch != raw)
            {
                inBucket = mBucketKeys.count(ch) != 0;
                if (const AttrValue* v = lookupBucketValue(ch))
                    return *v;
                if (const AttrValue* v = materializeBucketValue(ch))
                    return *v;
            }
        }
        if (inBucket && attrCovered && mBucketScalarsComplete)
            return {};
    }

    AttrValue out;
    withAttributeTensor(key, name, [&](const DLTensor& t, uint32_t primCount) { out = decodeScalar(t, primCount); });
    return out;
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
    // clear() on an unordered container costs its bucket count even when empty, and the
    // bucket arrays keep their high-water size; this runs once per read group.
    auto clearIfUsed = [](auto& container)
    {
        if (!container.empty())
            container.clear();
    };
    clearIfUsed(mBucketKeys);
    clearIfUsed(mBucketRows);
    clearIfUsed(mBucketAttributeIds);
    clearIfUsed(mBucketScalars);
    clearIfUsed(mBucketWorldTransforms);
    clearIfUsed(mBucketLocalTransforms);
    clearIfUsed(mBucketResetXformStack);
    mBucketReadGroupAttr = {};
    mBucketReadGroupTensor = nullptr;
    mBucketReadGroupComps = 0;
}

bool OvstageSource::bucketActive() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return mBucketActive;
}

struct OvstageSource::BucketSnapshot
{
    bool scalarsComplete = true;
    bool transformsComplete = true;
    std::unordered_set<uint64_t> keys;
    std::unordered_map<uint64_t, uint32_t> rows;
    std::unordered_set<uint32_t> attributeIds;
    std::unordered_map<uint64_t, std::unordered_map<uint32_t, AttrValue>> scalars;
    std::unordered_map<uint64_t, Matrix4d> worldTransforms;
    std::unordered_map<uint64_t, Matrix4d> localTransforms;
    std::unordered_map<uint64_t, bool> resetXformStack;
    TokenId readGroupAttr;
    const DLTensor* readGroupTensor = nullptr;
    int64_t readGroupComps = 0;
};

void OvstageSource::BucketSnapshotDeleter::operator()(BucketSnapshot* snapshot) const
{
    delete snapshot;
}

OvstageSource::BucketSnapshotPtr OvstageSource::suspendBucket() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mBucketActive)
        return nullptr;
    BucketSnapshotPtr s(new BucketSnapshot);
    s->scalarsComplete = mBucketScalarsComplete;
    s->transformsComplete = mBucketTransformsComplete;
    s->keys = std::move(mBucketKeys);
    s->rows = std::move(mBucketRows);
    s->attributeIds = std::move(mBucketAttributeIds);
    s->scalars = std::move(mBucketScalars);
    s->worldTransforms = std::move(mBucketWorldTransforms);
    s->localTransforms = std::move(mBucketLocalTransforms);
    s->resetXformStack = std::move(mBucketResetXformStack);
    s->readGroupAttr = mBucketReadGroupAttr;
    s->readGroupTensor = mBucketReadGroupTensor;
    s->readGroupComps = mBucketReadGroupComps;
    clearBucket(); // moved-from containers: clear() is the documented way back to a valid state
    return s;
}

void OvstageSource::resumeBucket(BucketSnapshotPtr snapshot) const
{
    if (!snapshot)
        return;
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    clearBucket();
    mBucketActive = true;
    mBucketScalarsComplete = snapshot->scalarsComplete;
    mBucketTransformsComplete = snapshot->transformsComplete;
    mBucketKeys = std::move(snapshot->keys);
    mBucketRows = std::move(snapshot->rows);
    mBucketAttributeIds = std::move(snapshot->attributeIds);
    mBucketScalars = std::move(snapshot->scalars);
    mBucketWorldTransforms = std::move(snapshot->worldTransforms);
    mBucketLocalTransforms = std::move(snapshot->localTransforms);
    mBucketResetXformStack = std::move(snapshot->resetXformStack);
    mBucketReadGroupAttr = snapshot->readGroupAttr;
    mBucketReadGroupTensor = snapshot->readGroupTensor;
    mBucketReadGroupComps = snapshot->readGroupComps;
}

void OvstageSource::beginLoadCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    for (ovstage_read_group_t& group : mLoadCacheGroups)
        ovstage_release_group(mInstance, &group);
    mLoadCacheActive = true;
    mLoadCacheGroups.clear();
    resetLoadCacheTable();
    mLoadCacheComposedWorld.clear();
}

void OvstageSource::beginHierarchyBulkRead() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mHierarchyBulkRead = true;
}

void OvstageSource::endHierarchyBulkRead() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mHierarchyBulkRead = false;
}

void OvstageSource::clearLoadCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    for (ovstage_read_group_t& group : mLoadCacheGroups)
        ovstage_release_group(mInstance, &group);
    mLoadCacheActive = false;
    mLoadCacheGroups.clear();
    resetLoadCacheTable();
    mLoadCacheComposedWorld.clear();
}

void OvstageSource::resetLoadCacheTable() const
{
    // Records are kept (cells cleared, capacity retained) so the next window's prefetches
    // reuse them; only the handle map and the attribute index are emptied.
    if (!mLoadCachePrimIndex.empty())
        mLoadCachePrimIndex.clear();
    for (uint32_t i = 0; i < mLoadCachePrimCount; ++i)
    {
        LoadCachePrim& prim = mLoadCachePrims[i];
        prim.cells.clear();
        prim.world = {};
        prim.local = {};
        prim.reset = {};
        prim.usdPathSerial = 0;
    }
    mLoadCachePrimCount = 0;
    if (mLoadCacheAttrCount != 0)
        std::fill(mLoadCacheAttrIndex.begin(), mLoadCacheAttrIndex.end(), kNoLoadCacheIndex);
    mLoadCacheAttrCount = 0;
}

uint32_t OvstageSource::loadCachePrimIndexFor(uint64_t raw) const
{
    const auto it = mLoadCachePrimIndex.find(raw);
    if (it != mLoadCachePrimIndex.end())
        return it->second;
    // First sight of this handle: it joins its canonical alias' record when one exists
    // (intern_path and get_paths may mint distinct handles for one path), else opens a new one.
    const uint64_t canonical = canonicalHandleRaw(raw);
    const bool hasAlias = canonical && canonical != raw;
    uint32_t index = kNoLoadCacheIndex;
    if (hasAlias)
    {
        const auto cit = mLoadCachePrimIndex.find(canonical);
        if (cit != mLoadCachePrimIndex.end())
            index = cit->second;
    }
    if (index == kNoLoadCacheIndex)
    {
        index = mLoadCachePrimCount++;
        if (index >= mLoadCachePrims.size())
            mLoadCachePrims.emplace_back();
        if (hasAlias)
            mLoadCachePrimIndex.emplace(canonical, index);
    }
    mLoadCachePrimIndex.emplace(raw, index);
    return index;
}

const OvstageSource::LoadCachePrim* OvstageSource::loadCachePrim(uint64_t handle) const
{
    if (!handle)
        return nullptr;
    const auto it = mLoadCachePrimIndex.find(handle);
    return it != mLoadCachePrimIndex.end() ? &mLoadCachePrims[it->second] : nullptr;
}

uint32_t OvstageSource::loadCacheAttrIndexFor(uint32_t attrId) const
{
    if (attrId >= mLoadCacheAttrIndex.size())
        mLoadCacheAttrIndex.resize(std::max<size_t>(size_t(attrId) + 1, mTokenToString.size()), kNoLoadCacheIndex);
    uint32_t& slot = mLoadCacheAttrIndex[attrId];
    if (slot == kNoLoadCacheIndex)
        slot = mLoadCacheAttrCount++;
    return slot;
}

OvstageSource::LoadCacheCell& OvstageSource::loadCacheCellFor(uint32_t primIndex, uint32_t attrIndex) const
{
    std::vector<LoadCacheCell>& cells = mLoadCachePrims[primIndex].cells;
    if (attrIndex >= cells.size())
        cells.resize(std::max<size_t>(size_t(attrIndex) + 1, mLoadCacheAttrCount));
    return cells[attrIndex];
}

const OvstageSource::LoadCacheCell* OvstageSource::loadCacheCell(uint64_t handle, uint32_t attrId) const
{
    const LoadCachePrim* prim = loadCachePrim(handle);
    if (!prim || attrId >= mLoadCacheAttrIndex.size())
        return nullptr;
    const uint32_t attrIndex = mLoadCacheAttrIndex[attrId];
    if (attrIndex == kNoLoadCacheIndex || attrIndex >= prim->cells.size())
        return nullptr;
    return &prim->cells[attrIndex];
}

bool OvstageSource::loadCacheCovers(uint64_t handle, uint32_t attrId) const
{
    const LoadCacheCell* cell = loadCacheCell(handle, attrId);
    return cell && cell->covered;
}

void OvstageSource::seedKnownKeys(const std::vector<ObjectKey>& keys) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (keys.empty())
        return;

    mKnownKeys.reserve(mKnownKeys.size() + keys.size() * 2);
    for (const ObjectKey key : keys)
    {
        const uint64_t raw = rawHandle(key);
        if (!raw)
            continue;

        mKnownKeys.insert(raw);
        if (const uint64_t canonical = canonicalHandleRaw(raw))
            mKnownKeys.insert(canonical);
    }
}

void OvstageSource::clearKnownKeys() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mKnownKeys.clear();
    mExistsMemo.clear();
}

void OvstageSource::clearSchemaCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mExistsMemo.clear();
    resetInstancingCaches();
    mSchemaCacheBuilt = false;
    mSchemaCacheComplete = false;
    mSchemaMayExistCache.clear();
    mSchemaMembershipCache.clear();
    mSchemasByPrimCache.clear();
    mMultiApplyMembershipCache.clear();
    mMultiApplyInstancesByPrimCache.clear();
    mPrimTypeByRaw.clear();
    mPrimTypeIndexBuilt = false;
    mPrimTypeIndexComplete = false;
    mPrimTypeIndex.clear();
    mSchemaVocabularyBuilt = false;
    mSchemaVocabularyComplete = false;
    mBaseInstanceMembership.clear();
    mAttributeVocabularyBuilt = false;
    mAttributeVocabulary.clear();
}

void OvstageSource::invalidateAttributeVocabulary() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mAttributeVocabularyBuilt = false;
    mAttributeVocabulary.clear();
    // The instance-material cache latched empty on this vocabulary's "no such column" answer;
    // a value-only range can author the column, so the latch goes with the vocabulary.
    mInstanceMaterialCacheBuilt = false;
    mInstanceMaterialCacheValid = false;
    mInstanceMaterialByPrim.clear();
}

bool OvstageSource::loadCacheActive() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return mLoadCacheActive;
}

void OvstageSource::resetInstancingCaches() const
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
}

bool OvstageSource::applySchemaDelta(const std::vector<SchemaRowDelta>& rows) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mInstance || !mDict)
        return false;
    // After a complete whole-stage build every schema has an entry, so a name first seen now
    // needs one too; in memo mode only the entries a consumer already asked for are patched.
    const bool complete = mSchemaCacheBuilt && mSchemaCacheComplete;
    for (const SchemaRowDelta& row : rows)
    {
        if (!row.raw)
            return false;
        const uint64_t canonical = canonicalHandleRaw(row.raw);
        std::unordered_set<uint32_t> schemas;
        std::unordered_set<uint32_t> bases;
        std::vector<uint32_t> ordered; // authored order, for the per-prim list
        for (const uint64_t value : row.schemaTokens)
        {
            ovx_string_t s{};
            if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(value), &s) != OVX_OK ||
                !s.ptr || s.length == 0)
                return false;
            const std::string_view name(s.ptr, s.length);
            const TokenId tok = doInternToken(name);
            if (!tok.valid())
                return false;
            if (schemas.insert(tok.id).second)
                ordered.push_back(tok.id);
            const size_t colon = name.find(':');
            if (colon != std::string_view::npos && colon > 0 && colon + 1 < name.size())
            {
                const TokenId base = doInternToken(name.substr(0, colon));
                if (base.valid())
                    bases.insert(base.id);
            }
        }
        // The row is the prim's complete post-write list: replace its per-prim entry (the
        // instance-name map has no incremental reader and is simply forgotten for the prim).
        for (const uint64_t handle : { row.raw, canonical })
        {
            if (!handle)
                continue;
            mMultiApplyInstancesByPrimCache.erase(handle);
            if (ordered.empty())
                mSchemasByPrimCache.erase(handle);
            else
                mSchemasByPrimCache[handle] = ordered;
        }
        for (const uint32_t id : schemas)
            mSchemaMayExistCache[id] = true; // a concrete schema name, applied here
        // Same handle aliasing as buildSchemaCache / schemaMembershipFromQuery: raw and canonical.
        auto patch = [&](std::unordered_map<uint32_t, std::unordered_set<uint64_t>>& cache,
                         const std::unordered_set<uint32_t>& present)
        {
            for (auto& entry : cache)
            {
                std::unordered_set<uint64_t>& members = entry.second;
                if (present.count(entry.first) != 0)
                {
                    members.insert(row.raw);
                    if (canonical)
                        members.insert(canonical);
                }
                else
                {
                    members.erase(row.raw);
                    if (canonical)
                        members.erase(canonical);
                }
            }
            if (!complete)
                return;
            for (const uint32_t id : present)
            {
                std::unordered_set<uint64_t>& members = cache[id];
                members.insert(row.raw);
                if (canonical)
                    members.insert(canonical);
            }
        };
        patch(mSchemaMembershipCache, schemas);
        patch(mMultiApplyMembershipCache, bases);
        // stageHasAnySchema answers unqualified multi-apply bases from the vocabulary's
        // base->instances map; a first "<base>:<inst>" applied at runtime must show up there too.
        if (mSchemaVocabularyBuilt && mSchemaVocabularyComplete)
        {
            for (auto it = mBaseInstanceMembership.begin(); it != mBaseInstanceMembership.end();)
            {
                if (bases.count(it->first) == 0)
                {
                    it->second.erase(row.raw);
                    if (canonical)
                        it->second.erase(canonical);
                }
                if (it->second.empty())
                    it = mBaseInstanceMembership.erase(it);
                else
                    ++it;
            }
            for (const uint32_t id : bases)
            {
                std::unordered_set<uint64_t>& members = mBaseInstanceMembership[id];
                members.insert(row.raw);
                if (canonical)
                    members.insert(canonical);
            }
        }
    }
    return true;
}

void OvstageSource::noteLivePrims(const std::vector<ovx_primpath_t>& raws) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    for (const ovx_primpath_t raw : raws)
    {
        if (!raw)
            continue;
        mExistsMemo[raw] = true;
        if (const uint64_t canonical = canonicalHandleRaw(raw))
            mExistsMemo[canonical] = true;
    }
}

void OvstageSource::clearExistsMemo() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mExistsMemo.clear();
}

void OvstageSource::invalidateHierarchyCache() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mChildCacheBuilt = false;
    mChildCacheComplete = false;
    mChildCache.clear();
    mDescendantCache.clear();
    mParentHandleCache.clear();
    mExistsMemo.clear();
}

bool OvstageSource::hierarchyCacheBuilt() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return mChildCacheBuilt;
}

bool OvstageSource::insertChildEdgeSorted(uint64_t parent, uint64_t child) const
{
    if (!parent || !child || parent == child)
        return false;
    const std::string childPath = pathOfRaw(child);
    if (childPath.empty())
        return false;
    std::vector<uint64_t>& children = mChildCache[parent]; // a leaf parent gains its list here
    // Same order buildChildCache imposes: by path, handle as tiebreak. pathOfRaw is memoised,
    // so the probes are hash lookups. A cached handle with this path is the same prim
    // (get_paths / intern_path may alias one path), i.e. the edge is already present.
    auto pos = std::lower_bound(children.begin(), children.end(), childPath,
                                [this](uint64_t a, const std::string& p) { return pathOfRaw(a) < p; });
    if (pos != children.end() && pathOfRaw(*pos) == childPath)
        return false;
    children.insert(pos, child);
    mParentHandleCache[child] = parent; // keeps the bulk-window leaf fast path valid for the new prim
    return true;
}

bool OvstageSource::applyHierarchyDelta(const std::vector<ovx_primpath_t>& addedRaw) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mChildCacheBuilt)
        return true; // nothing to patch; the next consumer builds at the latest ordinal
    // A partial build must keep its retry-to-complete path: only a rebuild can upgrade it.
    if (!mChildCacheComplete || !mInstance || !mDict)
        return false;

    // Any edge change can stale the memoised descendant sets.
    mDescendantCache.clear();

    std::unordered_set<uint64_t> seen;
    for (const ovx_primpath_t raw : addedRaw)
    {
        const uint64_t canonical = canonicalHandleRaw(raw);
        // Only the rows' own existence answers can be stale (a same-drain tombstone record on a
        // re-created path); the other records this drain made stay.
        mExistsMemo.erase(raw);
        if (canonical)
            mExistsMemo.erase(canonical);
        if (!canonical || !seen.insert(canonical).second)
            continue;
        // Same ancestor-chain rule as buildChildCache: connect every level up to the first
        // edge already cached, so a new row-less container is linked too. A handle that does
        // not resolve would leave a gap in a cache that claims completeness: bail out.
        std::string childPath = pathOfRaw(canonical);
        if (childPath.empty())
            return false;
        while (childPath != "/")
        {
            const size_t slash = childPath.rfind('/');
            if (slash == std::string::npos)
                return false;
            const std::string parentPath = slash == 0 ? std::string("/") : childPath.substr(0, slash);
            const uint64_t childCanonical = canonicalHandleRaw(findByPathRaw(childPath));
            const uint64_t parentCanonical = canonicalHandleRaw(findByPathRaw(parentPath));
            if (!childCanonical || !parentCanonical)
                return false;
            if (!insertChildEdgeSorted(parentCanonical, childCanonical))
                break;
            childPath = parentPath;
        }
    }
    return true;
}

void OvstageSource::purgeHandleFromMemos(uint64_t handle) const
{
    if (!handle)
        return;
    for (auto& entry : mSchemaMembershipCache)
        entry.second.erase(handle);
    for (auto& entry : mMultiApplyMembershipCache)
        entry.second.erase(handle);
    for (auto it = mBaseInstanceMembership.begin(); it != mBaseInstanceMembership.end();)
    {
        it->second.erase(handle);
        it = it->second.empty() ? mBaseInstanceMembership.erase(it) : std::next(it);
    }
    mSchemasByPrimCache.erase(handle);
    mMultiApplyInstancesByPrimCache.erase(handle);
    mPrimTypeByRaw.erase(handle);
    for (auto& entry : mPrimTypeIndex)
        entry.second.erase(handle);
}

bool OvstageSource::applyPrimRemoval(const std::vector<ovx_primpath_t>& removedRaw) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (removedRaw.empty())
        return true;
    // The cached subtree of a tombstoned prim is known only from a complete hierarchy cache;
    // anything less leaves descendants' memberships behind, so the caller drops instead.
    if (!mInstance || !mDict || !mChildCacheBuilt || !mChildCacheComplete)
        return false;

    // Drop `node` from its parent's child list; lists may hold either alias of a prim, and the
    // parent may be keyed by either alias too, so match children by path.
    auto unlinkFromParent = [&](uint64_t node)
    {
        const auto pit = mParentHandleCache.find(node);
        if (pit == mParentHandleCache.end())
            return;
        const std::string nodePath = pathOfRaw(node);
        for (const uint64_t parentKey : { pit->second, canonicalHandleRaw(pit->second) })
        {
            const auto cit = mChildCache.find(parentKey);
            if (cit == mChildCache.end())
                continue;
            std::vector<uint64_t>& children = cit->second;
            children.erase(std::remove_if(children.begin(), children.end(),
                                          [&](uint64_t c) { return c == node || pathOfRaw(c) == nodePath; }),
                           children.end());
        }
    };

    std::vector<uint64_t> stack;
    for (const ovx_primpath_t raw : removedRaw)
    {
        const uint64_t canonical = canonicalHandleRaw(raw);
        if (!raw || !canonical)
            return false;
        unlinkFromParent(raw);
        if (canonical != raw)
            unlinkFromParent(canonical);
        stack.push_back(raw);
        stack.push_back(canonical);
    }

    // The subtree: every cached descendant (under either alias key) is gone with its root.
    std::unordered_set<uint64_t> seen;
    while (!stack.empty())
    {
        const uint64_t node = stack.back();
        stack.pop_back();
        if (!node || !seen.insert(node).second)
            continue;
        const auto cit = mChildCache.find(node);
        if (cit != mChildCache.end())
        {
            for (const uint64_t child : cit->second)
            {
                stack.push_back(child);
                stack.push_back(canonicalHandleRaw(child));
            }
            mChildCache.erase(cit);
        }
        mParentHandleCache.erase(node);
        mKnownKeys.erase(node); // existsRaw trusts mKnownKeys before the memo
        mExistsMemo[node] = false;
        purgeHandleFromMemos(node);
    }
    mDescendantCache.clear();
    return true;
}

bool OvstageSource::applyPrimTypeDelta(const std::vector<PrimTypeRowDelta>& rows) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mInstance || !mDict)
        return false;
    for (const PrimTypeRowDelta& row : rows)
    {
        const uint64_t canonical = canonicalHandleRaw(row.raw);
        if (!row.raw || !canonical)
            return false;
        for (const uint64_t handle : { row.raw, canonical })
        {
            // The type index moves the handle: out of its previous type's set (a sweep when the
            // previous type is not memoised), into the new one.
            if (mPrimTypeIndexBuilt)
            {
                const auto previous = mPrimTypeByRaw.find(handle);
                if (previous != mPrimTypeByRaw.end())
                {
                    const auto set = mPrimTypeIndex.find(previous->second);
                    if (set != mPrimTypeIndex.end())
                        set->second.erase(handle);
                }
                else
                {
                    for (auto& entry : mPrimTypeIndex)
                        entry.second.erase(handle);
                }
                if (row.typeToken != 0)
                    mPrimTypeIndex[row.typeToken].insert(handle);
            }
            if (row.typeToken == 0)
                mPrimTypeByRaw.erase(handle);
            else
                mPrimTypeByRaw[handle] = row.typeToken;
        }
    }
    return true;
}

bool OvstageSource::buildPrimTypeIndex() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (mPrimTypeIndexBuilt)
        return mPrimTypeIndexComplete;
    mPrimTypeIndexBuilt = true;
    mPrimTypeIndexComplete = false;
    mPrimTypeIndex.clear();
    if (!mInstance || !mDict)
        return false;

    primTypeIndexBuildCounter().fetch_add(1, std::memory_order_relaxed);
    std::unordered_map<uint64_t, std::unordered_set<uint64_t>> index;
    std::unordered_map<uint64_t, uint64_t> typeByHandle;
    auto add = [&](uint64_t raw, uint64_t canonical, uint64_t value)
    {
        for (const uint64_t handle : { raw, canonical })
        {
            if (!handle)
                continue;
            index[value].insert(handle);
            typeByHandle[handle] = value;
        }
    };
    if (!forEachStageTokenValue(conv::kUsdPrimType, add))
        return false;

    mPrimTypeIndex = std::move(index);
    // The same rows answer isA()/getTypeName(): no single-prim type read is left to pay.
    for (const auto& entry : typeByHandle)
        mPrimTypeByRaw[entry.first] = entry.second;
    mPrimTypeIndexComplete = true;
    return true;
}

bool OvstageSource::collectPrimTypeKeys(std::string_view typeName, std::vector<ObjectKey>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    out.clear();
    if (typeName.empty())
        return true;
    if (!buildPrimTypeIndex())
        return false;
    // Interning a name no prim carries only mints a token nothing is indexed under.
    const ovx_token_t tok = ovxToken(typeName);
    if (tok == OVX_INVALID_TOKEN)
        return true;
    const auto it = mPrimTypeIndex.find(static_cast<uint64_t>(tok));
    if (it == mPrimTypeIndex.end())
        return true;
    // Both aliases of a prim are indexed; emit each prim once, by its canonical handle.
    std::unordered_set<uint64_t> seen;
    for (const uint64_t handle : it->second)
    {
        const uint64_t canonical = canonicalHandleRaw(handle);
        const uint64_t stable = canonical ? canonical : handle;
        if (seen.insert(stable).second)
            out.push_back(internKey(stable));
    }
    return true;
}

void OvstageSource::setReadOrdinal(ovstage_ordinal_t ord)
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    mReadOrdinal = ord;
}

ovstage_ordinal_t OvstageSource::readOrdinal() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return mReadOrdinal;
}

bool OvstageSource::primTypeTokenRaw(uint64_t raw, TokenId& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!raw || !mDict)
        return false;
    auto resolve = [&](uint64_t value) -> bool
    {
        ovx_string_t s{};
        if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(value), &s) != OVX_OK || !s.ptr ||
            s.length == 0)
            return false;
        out = doInternToken(std::string_view(s.ptr, s.length));
        return out.valid();
    };
    const uint64_t canonical = canonicalHandleRaw(raw);
    auto it = mPrimTypeByRaw.find(canonical ? canonical : raw);
    if (it == mPrimTypeByRaw.end() && canonical != raw)
        it = mPrimTypeByRaw.find(raw);
    if (it != mPrimTypeByRaw.end())
        return resolve(it->second);

    // Through the load cache / bucket / single-prim read; a type read is memoised (any later
    // type write is a structural row the feed patches in).
    const AttrValue v = getAttributeAtTimeRaw(raw, doInternToken(conv::kUsdPrimType), ReadTime::defaultTime());
    if (v.kind == AttrValue::Kind::eInt && v.i != 0 && resolve(static_cast<uint64_t>(v.i)))
    {
        mPrimTypeByRaw[raw] = static_cast<uint64_t>(v.i);
        if (canonical)
            mPrimTypeByRaw[canonical] = static_cast<uint64_t>(v.i);
        return true;
    }
    if (v.kind == AttrValue::Kind::eToken && v.tok.valid())
    {
        out = v.tok;
        return true;
    }
    return false;
}

bool OvstageSource::ensureAttributeVocabulary() const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mAttributeVocabularyBuilt && mInstance && mDict)
    {
        attributeVocabularyBuildCounter().fetch_add(1, std::memory_order_relaxed);
        // Whole-stage query; only the discovered attribute-column vocabulary is read, no rows.
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
        if (waitAndRelease(mInstance, qe) && q != OVSTAGE_INVALID_QUERY_HANDLE)
        {
            ovstage_query_result_t qr{};
            if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
            {
                mAttributeVocabulary.clear();
                mAttributeVocabulary.reserve(qr.attribute_count);
                for (size_t i = 0; i < qr.attribute_count && qr.attributes; ++i)
                {
                    ovx_string_t s{};
                    if (ovx_path_dictionary_token_to_string(mDict, qr.attributes[i], &s) == OVX_OK && s.ptr &&
                        s.length > 0)
                        mAttributeVocabulary.emplace(s.ptr, s.length);
                }
                ovstage_release_query_result(mInstance, &qr);
                mAttributeVocabularyBuilt = true; // only a fetched result is final; a failed query retries
            }
        }
        if (q != OVSTAGE_INVALID_QUERY_HANDLE)
            waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    }
    return mAttributeVocabularyBuilt;
}

std::optional<bool> OvstageSource::stageHasAttributeWithPrefix(std::string_view prefix) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!ensureAttributeVocabulary())
        return std::nullopt;
    for (const std::string& name : mAttributeVocabulary)
        if (name.size() >= prefix.size() && name.compare(0, prefix.size(), prefix.data(), prefix.size()) == 0)
            return true;
    return false;
}

std::optional<bool> OvstageSource::stageHasAttributeColumn(std::string_view name) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!ensureAttributeVocabulary())
        return std::nullopt;
    return mAttributeVocabulary.count(std::string(name)) != 0;
}

std::optional<bool> OvstageSource::stageHasAnySchema(const char* const* schemaNames, size_t nameCount) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!schemaNames || nameCount == 0)
        return false;
    if (!mInstance || !mDict)
        return std::nullopt;

    // Memo first: a membership entry per name (from a query or a structural delta) answers the
    // exact-name half without a round trip; only names nobody memoised go to ovstage.
    std::vector<ovx_string_t> vals;
    std::vector<uint32_t> unmemoisedIds;
    vals.reserve(nameCount);
    bool anyUnqualified = false;
    bool anyMemoisedMember = false;
    for (size_t i = 0; i < nameCount; ++i)
    {
        if (!schemaNames[i])
            continue;
        const std::string_view name(schemaNames[i]);
        vals.push_back(ovx_string_t{ name.data(), name.size() });
        anyUnqualified = anyUnqualified || name.find(':') == std::string_view::npos;
        const TokenId tok = doInternToken(name);
        const auto memo = mSchemaMembershipCache.find(tok.id);
        if (memo == mSchemaMembershipCache.end())
            unmemoisedIds.push_back(tok.id);
        else if (!memo->second.empty())
            anyMemoisedMember = true;
    }
    if (vals.empty())
        return false;
    if (anyMemoisedMember)
        return true;

    if (!unmemoisedIds.empty())
    {
        // One usd-schemas CONTAINS query OR'd over the names, count only; exact per element.
        ovstage_predicate_t pred{};
        pred.attribute.token = 0;
        pred.attribute.string = ovx_string_t{ conv::kUsdSchemas, std::string_view(conv::kUsdSchemas).size() };
        pred.op = OVSTAGE_FILTER_OP_CONTAINS;
        pred.values = vals.data();
        pred.value_count = vals.size();
        ovstage_filter_t filter{};
        filter.predicates = &pred;
        filter.count = 1;

        schemaPresenceQueryCounter().fetch_add(1, std::memory_order_relaxed);
        ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
        const ovstage_enqueue_result_t qe = ovstage_query(mInstance, &filter, nullptr, 0, &q);
        if (!waitAndRelease(mInstance, qe) || q == OVSTAGE_INVALID_QUERY_HANDLE)
        {
            if (q != OVSTAGE_INVALID_QUERY_HANDLE)
                waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
            return std::nullopt;
        }
        bool fetched = false;
        size_t count = 0;
        ovstage_query_result_t qr{};
        if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) == OVSTAGE_OK)
        {
            fetched = true;
            count = qr.total_prim_count;
            ovstage_release_query_result(mInstance, &qr);
        }
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        if (!fetched)
            return std::nullopt;
        if (count >= 1)
            return true; // cannot be attributed to one name: nothing memoised
        // Nobody applies any of them: each unmemoised name's membership is exactly empty. A row
        // later carrying the name inserts into this entry (applySchemaDelta).
        for (const uint32_t id : unmemoisedIds)
        {
            mSchemaMembershipCache.emplace(id, std::unordered_set<uint64_t>{});
            mSchemaMayExistCache.emplace(id, false);
        }
    }
    if (!anyUnqualified)
        return false;

    // Unqualified multi-apply bases: applied iff any "<base>:<instance>" is applied.
    if (!ensureSchemaVocabulary())
        return std::nullopt;
    for (size_t i = 0; i < nameCount; ++i)
    {
        if (!schemaNames[i] || std::string_view(schemaNames[i]).find(':') != std::string_view::npos)
            continue;
        const TokenId base = doInternToken(schemaNames[i]);
        if (base.valid() && mBaseInstanceMembership.count(base.id) != 0)
            return true;
    }
    return false;
}

bool OvstageSource::ensureSchemaVocabulary() const
{
    if (mSchemaVocabularyBuilt)
        return mSchemaVocabularyComplete;
    mSchemaVocabularyBuilt = true;
    mSchemaVocabularyComplete = false;
    mBaseInstanceMembership.clear();
    if (!mInstance || !mDict)
        return false;

    std::unordered_map<uint64_t, TokenId> baseOfValue; // schema token value -> base (invalid: single-apply)
    std::unordered_map<uint32_t, std::unordered_set<uint64_t>> members;
    const bool ok = forEachStageSchemaValue(
        [&](uint64_t raw, uint64_t canonical, uint64_t value)
        {
            auto it = baseOfValue.find(value);
            if (it == baseOfValue.end())
            {
                TokenId base;
                ovx_string_t s{};
                if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(value), &s) == OVX_OK &&
                    s.ptr && s.length > 0)
                {
                    const std::string_view name(s.ptr, s.length);
                    const size_t colon = name.find(':');
                    if (colon != std::string_view::npos && colon > 0 && colon + 1 < name.size())
                        base = doInternToken(name.substr(0, colon));
                }
                it = baseOfValue.emplace(value, base).first;
            }
            if (!it->second.valid())
                return;
            std::unordered_set<uint64_t>& set = members[it->second.id];
            set.insert(raw);
            if (canonical && canonical != raw)
                set.insert(canonical);
        });
    if (!ok)
        return false;
    mBaseInstanceMembership = std::move(members);
    mSchemaVocabularyComplete = true;
    return true;
}

bool OvstageSource::forEachStageSchemaValue(const std::function<void(uint64_t, uint64_t, uint64_t)>& fn) const
{
    return forEachStageTokenValue(conv::kUsdSchemas, fn);
}

bool OvstageSource::forEachStageTokenValue(std::string_view column,
                                           const std::function<void(uint64_t, uint64_t, uint64_t)>& fn) const
{
    if (!mInstance || !mDict || !fn || column.empty())
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
    const bool queryComplete = waitAndRelease(mInstance, qe);
    if (!queryComplete || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        if (q != OVSTAGE_INVALID_QUERY_HANDLE)
            waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }

    ovstage_query_handle_t use = q;
    ovstage_query_result_t qr{};
    if (ovstage_fetch_query_result(mInstance, q, OVSTAGE_TIMEOUT_INFINITE, &qr) != OVSTAGE_OK)
    {
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }
    if (qr.all_handle != OVSTAGE_INVALID_QUERY_HANDLE)
        use = qr.all_handle;
    ovstage_release_query_result(mInstance, &qr);

    ovx_token_t schemasAttr = OVX_INVALID_TOKEN;
    const ovx_string_t schemasName{ column.data(), column.size() };
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
    const bool readComplete = waitAndRelease(mInstance, re);
    if (!readComplete || rh == OVSTAGE_INVALID_READ_HANDLE)
    {
        if (rh != OVSTAGE_INVALID_READ_HANDLE)
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return false;
    }

    auto forEachTokenValue = [&](const DLTensor& t, auto&& visit) -> bool
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
                visit(value);
        }
        return true;
    };

    bool unsupported = false;
    ReadListMemo listMemo(mDict);
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
        if (!listMemo.paths(g.prims.list, &paths, &pathCount))
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
                const uint64_t canonical = canonicalHandleRaw(raw);
                if (!forEachTokenValue(g.data.tensors[tensorIndex],
                                       [&](uint64_t schemaValue) { fn(raw, canonical, schemaValue); }))
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
                const uint64_t canonical = canonicalHandleRaw(raw);
                for (int64_t col = 0; col < comps; ++col)
                {
                    const int64_t valueIndex = static_cast<int64_t>(dataRow) * comps + col;
                    const uint64_t schemaValue = (t.dtype.bits == 64) ?
                        reinterpret_cast<const uint64_t*>(bytes)[valueIndex] :
                        static_cast<uint64_t>(reinterpret_cast<const uint32_t*>(bytes)[valueIndex]);
                    if (schemaValue != 0)
                        fn(raw, canonical, schemaValue);
                }
            }
        }

        ovstage_release_group(mInstance, &g);
    }

    waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
    waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
    return fetchErr == OVSTAGE_ERROR_END_OF_ITERATION && !unsupported;
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
    auto addSchemaValue = [&](uint64_t raw, uint64_t canonical, uint64_t schemaValue)
    {
        const ParsedSchema* schema = parsedSchema(schemaValue);
        if (!schema)
            return;

        addSchemaForHandle(raw, *schema);
        if (canonical && canonical != raw)
            addSchemaForHandle(canonical, *schema);
    };

    if (!forEachStageSchemaValue(addSchemaValue))
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
    std::vector<uint64_t> raw;
    raw.reserve(out.size());
    for (const ObjectKey key : out)
        raw.push_back(rawHandle(key));
    const bool ok = collectSchemaKeysRaw(schemaToken, raw);
    out.clear();
    out.reserve(raw.size());
    for (const uint64_t handle : raw)
        out.push_back(internKey(handle));
    return ok;
}

bool OvstageSource::collectSchemaKeysRaw(TokenId schemaToken, std::vector<uint64_t>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!schemaToken.valid())
        return true;
    if (!mInstance || !mDict)
        return false;

    std::unordered_set<uint64_t> seen;
    seen.reserve(out.size() + 32);
    for (const uint64_t key : out)
    {
        const uint64_t canonical = canonicalHandleRaw(key);
        seen.insert(canonical ? canonical : key);
    }

    auto appendKey = [&](uint64_t key)
    {
        if (!key)
            return;
        const uint64_t canonical = canonicalHandleRaw(key);
        const uint64_t stable = canonical ? canonical : key;
        if (stable && seen.insert(stable).second)
            out.push_back(stable);
    };

    // Targeted, memoized per-schema query; exact for concrete schemas and "Base:instance" names.
    const std::unordered_set<uint64_t>* members = schemaMembershipFromQuery(schemaToken);
    if (!members)
        return false;
    for (const uint64_t handle : *members)
        appendKey(handle);

    // A bare joint-axis multi-apply base ("PhysxLimitAPI") additionally matches
    // every applied instance; nullptr (and a no-op) for anything else.
    if (const std::unordered_set<uint64_t>* baseMembers = multiApplyBaseMembership(schemaToken))
        for (const uint64_t handle : *baseMembers)
            appendKey(handle);
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

    schemaQueryCounter().fetch_add(1, std::memory_order_relaxed);
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
    const bool queryComplete = waitAndRelease(mInstance, e);
    if (!queryComplete || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        if (q != OVSTAGE_INVALID_QUERY_HANDLE)
            waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return nullptr;
    }

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
    const bool readComplete = waitAndRelease(mInstance, re);
    if (!readComplete || rh == OVSTAGE_INVALID_READ_HANDLE)
    {
        if (rh != OVSTAGE_INVALID_READ_HANDLE)
            waitAndRelease(mInstance, ovstage_release_read(mInstance, rh));
        waitAndRelease(mInstance, ovstage_release_query(mInstance, q));
        return nullptr;
    }

    std::unordered_set<uint64_t> members;
    members.reserve(count * 2);
    ReadListMemo listMemo(mDict);
    ovstage_read_group_t g{};
    ovstage_api_status_t fetchErr;
    while ((fetchErr = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
    {
        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (listMemo.paths(g.prims.list, &paths, &pathCount))
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
                if (const uint64_t canonical = canonicalHandleRaw(raw))
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


namespace
{
// Joint-DOF axis instance names shared by every joint multi-apply schema. An instance
// that is not applied yields an empty CONTAINS result, so querying the superset is safe.
const char* const kJointAxisInstances[] = { "transX", "transY", "transZ", "rotX",     "rotY", "rotZ",
                                            "linear", "angular", "distance", "X",       "Y",    "Z" };

bool isJointAxisMultiApplyBase(std::string_view base)
{
    return base == "PhysxLimitAPI" || base == "PhysicsLimitAPI" || base == "PhysicsDriveAPI" ||
           base == "PhysxJointAxisAPI" || base == "PhysicsJointStateAPI" || base == "PhysxMimicJointAPI" ||
           base == "PhysxDrivePerformanceEnvelopeAPI";
}

} // namespace

const std::unordered_set<uint64_t>* OvstageSource::multiApplyBaseMembership(TokenId baseToken) const
{
    if (!baseToken.valid())
        return nullptr;
    const auto cached = mMultiApplyMembershipCache.find(baseToken.id);
    if (cached != mMultiApplyMembershipCache.end())
        return &cached->second;

    const std::string_view base = tokenToString(baseToken);
    if (base.empty() || !isJointAxisMultiApplyBase(base))
        return nullptr; // not a known joint-axis multi-apply base -> caller falls back

    // Union of memoized per-instance queries; a later fully-qualified hasSchema reuses them.
    std::unordered_set<uint64_t> members;
    const std::string baseStr(base);
    for (const char* inst : kJointAxisInstances)
    {
        const TokenId instTok = doInternToken(baseStr + ":" + inst);
        const std::unordered_set<uint64_t>* m = schemaMembershipFromQuery(instTok);
        if (!m)
            return nullptr; // query failed (an absent instance is an empty set): don't cache a partial union
        members.insert(m->begin(), m->end());
    }
    return &mMultiApplyMembershipCache.emplace(baseToken.id, std::move(members)).first->second;
}

bool OvstageSource::collectMultiApplySchemaKeys(TokenId baseSchemaToken, std::vector<ObjectKey>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!baseSchemaToken.valid())
        return true;
    if (!mInstance || !mDict)
        return false;

    const std::unordered_set<uint64_t>* members = multiApplyBaseMembership(baseSchemaToken);
    if (!members)
    {
        // Not a joint-axis base (e.g. tendons, user-arbitrary instances): fall back to the
        // whole-stage schema parse. Cold -- the walker gates this on tendon presence.
        if (!buildSchemaCache())
            return false;
        const auto it = mMultiApplyMembershipCache.find(baseSchemaToken.id);
        members = it != mMultiApplyMembershipCache.end() ? &it->second : nullptr;
    }
    if (!members)
        return true;

    std::unordered_set<uint64_t> seen;
    seen.reserve(out.size() + members->size());
    for (const ObjectKey key : out)
    {
        const uint64_t raw = rawHandle(key);
        const uint64_t canonical = canonicalHandleRaw(raw);
        seen.insert(canonical ? canonical : raw);
    }

    for (const uint64_t handle : *members)
    {
        const uint64_t canonical = canonicalHandleRaw(handle);
        const uint64_t stable = canonical ? canonical : handle;
        if (stable && seen.insert(stable).second)
            out.push_back(internKey(stable));
    }
    return true;
}

std::vector<ObjectKey> OvstageSource::collectAncestors(const std::vector<ObjectKey>& keys) const
{
    std::vector<ObjectKey> out;
    std::unordered_set<uint64_t> seen;
    seen.reserve(keys.size() * 2);
    for (const ObjectKey key : keys)
    {
        if (!key.valid())
            continue;
        // Dedup by canonical raw handle: two ObjectKeys for one prim need not share a packed
        // handle (ADR-0021), and the ancestor walk below inserts canonical handles.
        const uint64_t canonical = canonicalPath(key);
        seen.insert(canonical ? canonical : key.handle);
    }

    const ObjectKey root = getRootKey();
    for (const ObjectKey key : keys)
    {
        ObjectKey cur = getParent(key);
        for (int guard = 0; cur.valid() && guard < 64; ++guard)
        {
            const uint64_t canonical = canonicalPath(cur);
            const uint64_t dedupe = canonical ? canonical : cur.handle;
            if (dedupe && seen.insert(dedupe).second)
                out.push_back(cur);

            if (cur == root)
                break;
            const ObjectKey next = getParent(cur);
            if (next.handle == cur.handle)
                break;
            cur = next;
        }
    }
    return out;
}

void OvstageSource::prefetchRelationshipAncestors(const std::vector<ObjectKey>& keys) const
{
    // Prims and their ancestors: getMaterialBinding tests the prim's own binding first.
    std::vector<ObjectKey> bucketKeys = collectAncestors(keys);
    bucketKeys.insert(bucketKeys.end(), keys.begin(), keys.end());
    if (bucketKeys.empty())
        return;
    prefetchBucket(bucketKeys, relationshipPrefetchAttrs());
}

const std::vector<std::string>& OvstageSource::relationshipPrefetchAttrs()
{
    static const std::vector<std::string> kRelationshipAttrs = {
        "material:binding:physics",
        "material:binding",
        "physics:simulationOwner",
    };
    return kRelationshipAttrs;
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
        uint32_t prim = kNoLoadCacheIndex; // load-cache record; only under a window
    };
    ++mPrefetchSerial;

    // Record bucket membership up front (by canonical handle): a prim with no
    // readable column is still "in the bucket", so a sealed scalar miss for it
    // resolves to empty. Under a window each key also resolves its load-cache
    // record here, once, so no per-cell store below touches the handle map.
    std::vector<ovx_primpath_t> paths;
    std::vector<CacheHandlePair> cacheHandles;
    paths.reserve(keys.size());
    cacheHandles.reserve(keys.size());
    for (const ObjectKey k : keys)
    {
        const uint64_t raw = rawHandle(k);
        if (!raw)
            continue;
        paths.push_back(raw);
        mBucketKeys.insert(raw);
        const uint64_t ch = canonicalHandleRaw(raw);
        if (ch)
            mBucketKeys.insert(ch);
        cacheHandles.push_back({ raw, ch, mLoadCacheActive ? loadCachePrimIndexFor(raw) : kNoLoadCacheIndex });
    }
    if (paths.empty())
        return;
    mBucketActive = true;

    // Map each requested attribute's ovstage token -> parse TokenId so a returned
    // group (tagged by g.attribute) routes to the scalar cache. Transform
    // matrices are cached separately from scalar attrs so transform reads can
    // avoid per-prim ovstage reads while preserving the IPhysicsSource API.
    // Attribute counts are tens, so a vector with a linear token search beats a map.
    struct AttrMeta
    {
        ovx_token_t tok;
        TokenId tokenId;
        uint32_t attrIndex; // load-cache column; kNoLoadCacheIndex outside a window
        bool isWorldTransform;
        bool isLocalTransform;
        bool isResetXformStack;
        bool returned = false; // a group of this column came back
        bool cacheable = false; // ... and was cached (or was a delete / empty group)
        bool unsupported = false; // ... or could not be cached row-wise
    };
    std::vector<AttrMeta> metas;
    std::vector<ovx_token_t> attrToks;
    std::vector<uint32_t> invalidAttrIds;
    metas.reserve(attrNames.size());
    attrToks.reserve(attrNames.size());
    auto findMeta = [&](ovx_token_t tok) -> AttrMeta*
    {
        for (AttrMeta& m : metas)
            if (m.tok == tok)
                return &m;
        return nullptr;
    };
    // The raw handle and its canonical alias share one record, so covering the raw covers both.
    auto loadCacheCoversAll = [&](uint32_t attrId) -> bool
    {
        if (!mLoadCacheActive)
            return false;
        for (const CacheHandlePair& handle : cacheHandles)
            if (!loadCacheCovers(handle.raw, attrId))
                return false;
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
        const ovx_token_t tok = ovxToken(conv::toOvstageColumn(name));
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
        const AttrMeta meta{ tok,
                             tokenId,
                             mLoadCacheActive ? loadCacheAttrIndexFor(tokenId.id) : kNoLoadCacheIndex,
                             tokenId == worldTransformAttr,
                             tokenId == localTransformAttr || tokenId == fabricLocalTransformAttr,
                             tokenId == resetXformStackAttr };
        if (AttrMeta* existing = findMeta(tok))
            *existing = meta; // two names on one column: the later one wins, as the map did
        else
            metas.push_back(meta);
    }

    // Coverage is one bit per (record, column); each bucket key is marked once since the raw
    // handle and its canonical alias share the record.
    auto markLoadCoverageForBucket = [&](uint32_t attrId)
    {
        if (!mLoadCacheActive)
            return;
        const uint32_t attrIndex = loadCacheAttrIndexFor(attrId);
        for (const CacheHandlePair& handle : cacheHandles)
            loadCacheCellFor(handle.prim, attrIndex).covered = true;
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
    // Row references go into the prim's shared record (load-cache window only). A null
    // tensor is never stored: LoadCacheCell reads null as "no row".
    auto storeLoadScalarRef = [&](uint32_t prim, uint32_t attrIndex, CachedTensorRow row)
    {
        loadCacheCellFor(prim, attrIndex).scalar = row;
    };
    auto storeLoadRelationshipRef = [&](uint32_t prim, uint32_t attrIndex, const DLTensor* tensor)
    {
        loadCacheCellFor(prim, attrIndex).relationship = tensor;
    };
    auto storeLoadTransformRef = [&](uint32_t prim, const AttrMeta& meta, CachedTensorRow row)
    {
        LoadCachePrim& rec = mLoadCachePrims[prim];
        if (meta.isWorldTransform)
            rec.world = row;
        else if (meta.isLocalTransform)
            rec.local = row;
        else if (meta.isResetXformStack)
            rec.reset = row;
    };

    // usd-path among the columns (the walker's merged prefetch asks for it): every live prim
    // carries it, so its rows are the bucket's live set and seed the existence memo below --
    // the same `usd-path` evidence existsBatch() reads, without its query. A row stamps its
    // record with this call's serial; the stamp is the per-call "seen" set.
    ovx_token_t usdPathTok = OVX_INVALID_TOKEN;
    if (mLoadCacheActive)
    {
        const TokenId usdPathAttr = doInternToken(conv::kUsdPath);
        for (const AttrMeta& m : metas)
            if (m.tokenId == usdPathAttr)
                usdPathTok = m.tok;
    }
    bool usdPathGroupSeen = false;

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
        prefetchBucketReadCounter().fetch_add(1, std::memory_order_relaxed);
        const ovstage_enqueue_result_t re =
            ovstage_read_attributes(mInstance, q, attrToks.data(), attrToks.size(), range, &rh);
        if (re.status == OVSTAGE_OK)
        {
            const bool readOk = waitAndRelease(mInstance, re);

            ReadListMemo listMemo(mDict);
            ovstage_read_group_t g{};
            ovstage_api_status_t fetchStatus;
            while ((fetchStatus = ovstage_fetch_read_next(mInstance, rh, OVSTAGE_TIMEOUT_INFINITE, &g)) == OVSTAGE_OK)
            {
                if (usdPathTok != OVX_INVALID_TOKEN && g.attribute == usdPathTok && !g.is_delete)
                {
                    usdPathGroupSeen = true;
                    const ovx_primpath_t* rows = nullptr;
                    size_t rowCount = 0;
                    if (listMemo.paths(g.prims.list, &rows, &rowCount))
                        for (uint32_t i = 0; i < g.prims.count; ++i)
                        {
                            const uint32_t idx = g.prims.index_map ? g.prims.index_map[i] : (g.prims.offset + i);
                            if (idx >= rowCount || !rows[idx])
                                continue;
                            mLoadCachePrims[loadCachePrimIndexFor(rows[idx])].usdPathSerial = mPrefetchSerial;
                        }
                }
                AttrMeta* meta = findMeta(g.attribute);
                if (!meta)
                {
                    ovstage_release_group(mInstance, &g);
                    continue;
                }
                meta->returned = true;

                bool groupOwnedByLoadCache = false;
                if (g.is_delete)
                {
                    meta->cacheable = true;
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
                        meta->isWorldTransform || meta->isLocalTransform || meta->isResetXformStack;
                    const bool cacheable =
                        ((meta->isWorldTransform || meta->isLocalTransform) && t.data) ||
                        (meta->isResetXformStack && t.data && canDecodeAttrValue(t)) ||
                        (!transformAttr && t.data && canDecodeAttrValue(t));
                    if (!cacheable)
                    {
                        meta->unsupported = true;
                        if (transformAttr)
                            mBucketTransformsComplete = false;
                        else
                            mBucketScalarsComplete = false;
                        ovstage_release_group(mInstance, &g);
                        continue;
                    }

                    const ovx_primpath_t* gpaths = nullptr;
                    size_t gcount = 0;
                    if (listMemo.paths(g.prims.list, &gpaths, &gcount))
                    {
                        meta->cacheable = true;
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
                            meta->unsupported = true;
                            if (transformAttr)
                                mBucketTransformsComplete = false;
                            else
                                mBucketScalarsComplete = false;
                            ovstage_release_group(mInstance, &g);
                            continue;
                        }
                        const int64_t comps = total / storedRows;
                        if (((meta->isWorldTransform || meta->isLocalTransform) && comps < 16) ||
                            (meta->isResetXformStack && comps < 1))
                        {
                            meta->unsupported = true;
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
                                meta->unsupported = true;
                                if (transformAttr)
                                    mBucketTransformsComplete = false;
                                else
                                    mBucketScalarsComplete = false;
                                continue;
                            }
                            const uint64_t raw = gpaths[idx];
                            if (!raw)
                                continue;
                            // Under a window the record resolves the alias once per prim; the
                            // bucket-only arm still keys its decoded copies by both handles.
                            const uint32_t prim = mLoadCacheActive ? loadCachePrimIndexFor(raw) : kNoLoadCacheIndex;
                            const uint64_t canonical = mLoadCacheActive ? 0 : canonicalHandleRaw(raw);
                            if (meta->isWorldTransform || meta->isLocalTransform)
                            {
                                if (comps < 16)
                                {
                                    mBucketTransformsComplete = false;
                                    meta->unsupported = true;
                                    continue;
                                }
                                if (mLoadCacheActive)
                                    storeLoadTransformRef(prim, *meta, CachedTensorRow{ cacheTensor, comps, dataRow });
                                else
                                {
                                    Matrix4d matrix;
                                    if (!decodeMatrixAt(t, comps, dataRow, matrix.data))
                                    {
                                        mBucketTransformsComplete = false;
                                        meta->unsupported = true;
                                        continue;
                                    }
                                    mBucketKeys.insert(raw);
                                    if (meta->isWorldTransform)
                                        mBucketWorldTransforms[raw] = matrix;
                                    else
                                        mBucketLocalTransforms[raw] = matrix;
                                    if (canonical && canonical != raw)
                                    {
                                        mBucketKeys.insert(canonical);
                                        if (meta->isWorldTransform)
                                            mBucketWorldTransforms[canonical] = matrix;
                                        else
                                            mBucketLocalTransforms[canonical] = matrix;
                                    }
                                }
                                continue;
                            }
                            if (meta->isResetXformStack)
                            {
                                if (mLoadCacheActive)
                                    storeLoadTransformRef(prim, *meta, CachedTensorRow{ cacheTensor, comps, dataRow });
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
                                        meta->unsupported = true;
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
                                storeLoadScalarRef(prim, meta->attrIndex,
                                                   CachedTensorRow{ cacheTensor, comps, dataRow });
                            else
                            {
                                AttrValue v = decodeAt(t, comps, dataRow);
                                if (v.valid())
                                {
                                    mBucketKeys.insert(raw);
                                    mBucketScalars[raw][meta->tokenId.id] = v;
                                    if (canonical && canonical != raw)
                                    {
                                        mBucketKeys.insert(canonical);
                                        mBucketScalars[canonical][meta->tokenId.id] = std::move(v);
                                    }
                                }
                            }
                        }
                    }
                    else
                    {
                        meta->unsupported = true;
                        if (meta->isWorldTransform || meta->isLocalTransform || meta->isResetXformStack)
                            mBucketTransformsComplete = false;
                        else
                            mBucketScalarsComplete = false;
                    }
                }
                else if (usable && g.is_array && g.data.mask == nullptr)
                {
                    const ovx_primpath_t* gpaths = nullptr;
                    size_t gcount = 0;
                    if (!listMemo.paths(g.prims.list, &gpaths, &gcount))
                    {
                        meta->unsupported = true;
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
                        meta->unsupported = true;
                        mBucketScalarsComplete = false;
                        ovstage_release_group(mInstance, &g);
                        continue;
                    }

                    meta->cacheable = true;
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
                        storeLoadRelationshipRef(loadCachePrimIndexFor(raw), meta->attrIndex,
                                                 &mLoadCacheGroups.back().data.tensors[tensorIndex]);
                    }
                    if (!mLoadCacheActive)
                        mBucketScalarsComplete = false;
                }
                else if (usable) // a column we can't cache row-wise
                {
                    meta->unsupported = true;
                    if (meta->isWorldTransform || meta->isLocalTransform || meta->isResetXformStack)
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
                    meta->cacheable = true;
                }
                if (!groupOwnedByLoadCache)
                    ovstage_release_group(mInstance, &g);
            }
            // Misses are sealed (served as authoritative empties) only after a clean,
            // complete read; a failed or truncated read leaves them to the live fallback.
            const bool cleanRead = readOk && fetchStatus == OVSTAGE_ERROR_END_OF_ITERATION;
            if (!cleanRead)
            {
                mBucketScalarsComplete = false;
                mBucketTransformsComplete = false;
            }
            // Existence from the usd-path rows: a bucket key with one is live, one without is
            // not (only after a clean read that returned the column at all).
            if (mLoadCacheActive && cleanRead && usdPathGroupSeen)
            {
                for (const CacheHandlePair& handle : cacheHandles)
                {
                    const bool live = mLoadCachePrims[handle.prim].usdPathSerial == mPrefetchSerial;
                    mExistsMemo[handle.raw] = live;
                    if (handle.canonical)
                        mExistsMemo[handle.canonical] = live;
                }
            }
            if (mLoadCacheActive)
            {
                if (cleanRead)
                    for (const AttrMeta& m : metas)
                    {
                        if (m.unsupported)
                            continue;
                        if (sealMissing && (!m.returned || m.cacheable))
                            markLoadCoverageForBucket(m.tokenId.id);
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
                                            size_t keyCount,
                                            bool append,
                                            const ovx_primpath_t* listPaths,
                                            size_t listCount) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!append)
        clearBucket();
    if (!mDict || !attr.valid() || group.is_delete || group.prims.count == 0)
        return;

    if (!mBucketAttrTokensValid)
    {
        mBucketLocalTransformAttr = doInternToken(conv::kLocalTransform);
        mBucketFabricLocalTransformAttr = doInternToken(conv::kFabricLocalMatrix);
        mBucketWorldTransformAttr = doInternToken(conv::kFabricWorldMatrix);
        mBucketResetXformStackAttr = doInternToken(conv::kResetXformStack);
        mBucketAttrTokensValid = true;
    }
    const bool isWorldTransform = attr == mBucketWorldTransformAttr;
    const bool isLocalTransform = attr == mBucketLocalTransformAttr || attr == mBucketFabricLocalTransformAttr;
    const bool isResetXformStack = attr == mBucketResetXformStackAttr;

    const ovx_primpath_t* gpaths = listPaths;
    size_t gcount = listCount;
    if (!gpaths && (ovx_path_dictionary_get_paths(mDict, group.prims.list, &gpaths, &gcount) != OVX_OK || !gpaths))
        return;

    // Handles this group contributes, with their tensor row. Decoding walks this list
    // rather than mBucketRows so rows an earlier appended group owns are left alone.
    std::vector<std::pair<uint64_t, uint32_t>> groupRows;
    groupRows.reserve(group.prims.count * 2);
    mBucketRows.reserve(mBucketRows.size() + group.prims.count * 2);
    mBucketKeys.reserve(mBucketKeys.size() + group.prims.count * 2);
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
        groupRows.emplace_back(raw, i);
        if (keys && keyOrdinal < keyCount)
        {
            const uint64_t keyHandle = rawHandle(keys[keyOrdinal]);
            if (keyHandle)
            {
                mBucketKeys.insert(keyHandle);
                mBucketRows[keyHandle] = i;
                groupRows.emplace_back(keyHandle, i);
            }
        }
        ++keyOrdinal;
    }

    if (groupRows.empty())
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
    const int64_t comps = componentsPerPrim(t, group.prims.count);
    if (isWorldTransform)
    {
        if (comps < 16)
        {
            mBucketTransformsComplete = false;
            return;
        }
        // Decoded eagerly: the group is released long before the transform flush at
        // group-complete reads these matrices back.
        for (const std::pair<uint64_t, uint32_t>& row : groupRows)
        {
            Matrix4d matrix;
            if (!decodeMatrixAt(t, comps, row.second, matrix.data))
            {
                mBucketTransformsComplete = false;
                continue;
            }
            mBucketWorldTransforms[row.first] = matrix;
        }
        return;
    }

    if (append)
    {
        // Other columns are served straight from the group's tensor, which would dangle
        // once the group is released; report them incomplete so getters read live, and
        // drop any earlier group's tensor so a later key cannot decode against it.
        if (isLocalTransform || isResetXformStack)
            mBucketTransformsComplete = false;
        else
            mBucketScalarsComplete = false;
        mBucketReadGroupAttr = {};
        mBucketReadGroupTensor = nullptr;
        mBucketReadGroupComps = 0;
        return;
    }

    mBucketReadGroupAttr = attr;
    mBucketReadGroupTensor = &t;
    mBucketReadGroupComps = comps;
    if ((isLocalTransform && comps < 16) || (isResetXformStack && comps < 1))
        mBucketTransformsComplete = false;
}

void OvstageSource::seedBucketFromColumn(
    TokenId attr, const uint64_t* rawHandles, const ObjectKey* keys, size_t count, const DLTensor& tensor) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    clearBucket();
    if (!attr.valid() || count == 0 || !tensor.data || !rawHandles || !keys)
        return;

    mBucketAttributeIds.insert(attr.id);
    mBucketRows.reserve(count * 2);
    mBucketKeys.reserve(count * 2);
    for (size_t i = 0; i < count; ++i)
    {
        const uint32_t row = static_cast<uint32_t>(i);
        if (rawHandles[i])
        {
            mBucketKeys.insert(rawHandles[i]);
            mBucketRows[rawHandles[i]] = row;
        }
        if (const uint64_t keyHandle = rawHandle(keys[i]))
        {
            mBucketKeys.insert(keyHandle);
            mBucketRows[keyHandle] = row;
        }
    }
    mBucketActive = true;
    mBucketReadGroupAttr = attr;
    mBucketReadGroupTensor = &tensor;
    mBucketReadGroupComps = componentsPerPrim(tensor, static_cast<uint32_t>(count));
}

// ADR-0020: a value the producer publishes is authoritative, so everything ovstage
// publishes counts as authored and this always answers true. The authored bit exists to
// stop an unresolved schema fallback from overwriting a resolved default; under ovstage
// that resolution -- units, schema defaults, whatever else the schema implies -- happens
// producer-side, so there is nothing left for the consumer to gate on. Answering from the
// backing USD stage, as this used to, made the whole authored surface conditional on a
// stage a USD-free consumer does not have.
bool OvstageSource::hasAuthoredAttribute(ObjectKey /*key*/, TokenId /*attr*/) const
{
    return true;
}

bool OvstageSource::isAttributeTimeSampled(ObjectKey /*key*/, TokenId /*attr*/) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    return false;
}

// --- schemas / type --------------------------------------------------------

bool OvstageSource::hasSchema(ObjectKey keyIn, TokenId schemaToken) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    const uint64_t key = rawHandle(keyIn);
    if (!key || !schemaToken.valid())
        return false;

    const std::string_view schemaName = tokenToString(schemaToken);
    if (schemaName.empty())
        return false;

    auto memberContains = [&](const std::unordered_set<uint64_t>& members) -> bool
    {
        if (members.count(key) != 0)
            return true;
        const uint64_t canonical = canonicalHandleRaw(key);
        return canonical && members.count(canonical) != 0;
    };

    // Primary path: one targeted, memoized `usd-schemas CONTAINS <name>` query per schema;
    // exact for concrete schemas and fully-qualified "Base:instance" names.
    if (const std::unordered_set<uint64_t>* members = schemaMembershipFromQuery(schemaToken))
        if (memberContains(*members))
            return true;

    // Unqualified joint-axis base ("PhysxLimitAPI"): union of per-instance queries; nullptr otherwise.
    if (const std::unordered_set<uint64_t>* baseMembers = multiApplyBaseMembership(schemaToken))
        if (memberContains(*baseMembers))
            return true;

    // Any other unqualified multi-apply base matches when this prim applies an instance of
    // it. Only reached when the exact name is applied nowhere (a name is either a
    // single-apply schema or a multi-apply base, never both).
    if (schemaName.find(':') == std::string_view::npos)
    {
        const auto mayExist = mSchemaMayExistCache.find(schemaToken.id);
        if (mayExist == mSchemaMayExistCache.end() || !mayExist->second)
        {
            if (ensureSchemaVocabulary())
            {
                const auto it = mBaseInstanceMembership.find(schemaToken.id);
                return it != mBaseInstanceMembership.end() && memberContains(it->second);
            }
            // Vocabulary unreadable: one single-prim schema read.
            const std::string prefix = std::string(schemaName) + ":";
            bool found = false;
            readPrimSchemaNames(key,
                                [&](std::string_view name)
                                {
                                    if (name.size() > prefix.size() && name.compare(0, prefix.size(), prefix) == 0)
                                        found = true;
                                });
            return found;
        }
    }

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

TokenId OvstageSource::getTypeName(ObjectKey key) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid())
        return TokenId{};
    TokenId primType{};
    // Read on the geometry backing so an instance proxy reports its prototype's
    // authored type, matching what `isA` resolves for the same key.
    if (!primTypeTokenRaw(rawHandle(key), primType))
        primTypeTokenRaw(rawHandle(geometryBackingKey(key)), primType);
    return primType;
}

bool OvstageSource::isA(ObjectKey key, TokenId typeToken) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !typeToken.valid())
        return false;
    const std::string path = pathOfRaw(rawHandle(key));
    const std::string_view typeName = tokenToString(typeToken);
    if (path.empty() || typeName.empty() || !mInstance)
        return false;

    // The prim's own authored type from the memo or one path-list read (the same column
    // getTypeName reads); every candidate below is then a string compare. The 2-predicate filter
    // query, a stage-side predicate evaluation per candidate, remains only for an unreadable column.
    std::string_view ownType;
    TokenId ownTypeToken{};
    if (primTypeTokenRaw(rawHandle(key), ownTypeToken))
        ownType = tokenToString(ownTypeToken);

    auto hasPrimType = [&](std::string_view primType) -> bool
    {
        if (!ownType.empty())
            return ownType == primType;
        isATypeQueryCounter().fetch_add(1, std::memory_order_relaxed);
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
        // "Points" belongs here: UsdGeomPoints derives Gprim -> PointBased -> Points,
        // exactly as BasisCurves does. It was listed in kXformableTypes and
        // kPointBasedTypes below but not here, so the same prim answered true to two
        // of its bases and false to the third. The gate that paid for it is
        // emitDeformableBody's `isXformable(key) && !isA(key, "Gprim")` root test,
        // which a points cloud wrongly satisfied. This does NOT give a points collider
        // a shape on this backend: the walker's PrimType dispatch has no Points entry
        // at all (PLAN section 6), so that remains an open walker gap.
        static const char* const kGprimTypes[] = { "Cube", "Sphere", "Capsule", "Cone", "Cylinder", "Plane", "Mesh",
                                                   "TetMesh", "Points", "BasisCurves", "PhysxParticleSystem" };
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

// Applied-schema names of a single prim: the per-prim memo when the whole-stage build or a
// structural delta filled it (a present entry is always the complete list), else one single-prim
// read. Only the cold callers (forEachAppliedSchema / forEachMultiApplyInstance) need this.
void OvstageSource::readPrimSchemaNames(uint64_t raw, const std::function<void(std::string_view)>& fn) const
{
    if (!raw || !mDict || !fn)
        return;
    const uint64_t canonical = canonicalHandleRaw(raw);
    auto memo = mSchemasByPrimCache.find(canonical ? canonical : raw);
    if (memo == mSchemasByPrimCache.end() && canonical != raw)
        memo = mSchemasByPrimCache.find(raw);
    if (memo != mSchemasByPrimCache.end())
    {
        for (const uint32_t id : memo->second)
        {
            const std::string_view name = tokenToString(TokenId{ id });
            if (!name.empty())
                fn(name);
        }
        return;
    }
    withAttributeTensor(raw, conv::kUsdSchemas,
                        [&](const DLTensor& t, uint32_t /*primCount*/)
                        {
                            if (!t.data || !(t.dtype.code == kDLUInt && (t.dtype.bits == 64 || t.dtype.bits == 32)))
                                return;
                            const int64_t count = totalElements(t);
                            const auto* bytes = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                            for (int64_t i = 0; i < count; ++i)
                            {
                                const uint64_t value = (t.dtype.bits == 64) ?
                                    reinterpret_cast<const uint64_t*>(bytes)[i] :
                                    static_cast<uint64_t>(reinterpret_cast<const uint32_t*>(bytes)[i]);
                                if (value == 0)
                                    continue;
                                ovx_string_t s{};
                                if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(value), &s) ==
                                        OVX_OK &&
                                    s.ptr && s.length > 0)
                                    fn(std::string_view(s.ptr, s.length));
                            }
                        });
}

void OvstageSource::forEachAppliedSchema(ObjectKey key, std::function<void(TokenId)> cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || !cb)
        return;

    std::unordered_set<uint32_t> emitted;
    readPrimSchemaNames(rawHandle(key),
                        [&](std::string_view name)
                        {
                            const TokenId tok = doInternToken(name);
                            if (tok.valid() && emitted.insert(tok.id).second)
                                cb(tok);
                        });
}

void OvstageSource::forEachMultiApplyInstance(ObjectKey key,
                                              std::string_view baseSchema,
                                              std::function<void(std::string_view)> cb) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!key.valid() || baseSchema.empty() || !cb)
        return;

    // Elements are "Base:instance"; emit the instance suffix of each matching base.
    const std::string prefix = std::string(baseSchema) + ":";
    std::unordered_set<std::string> emitted;
    readPrimSchemaNames(rawHandle(key),
                        [&](std::string_view name)
                        {
                            if (name.size() <= prefix.size() || name.compare(0, prefix.size(), prefix) != 0)
                                return;
                            const std::string instance(name.substr(prefix.size()));
                            if (emitted.insert(instance).second)
                                cb(instance);
                        });
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

    // Same stage-wide column gate as getRelationshipTargets: no column, no round trip.
    if (stageHasAttributeColumn(conv::toOvstageColumn(relName)) == std::optional<bool>(false))
        return false;
    bool found = false;
    relationshipReadCounter().fetch_add(1, std::memory_order_relaxed);
    withAttributeTensor(rawHandle(key), relName,
                        [&](const DLTensor&, uint32_t)
                        {
                            found = true;
                        });
    // No backing-USD fallback (ADR-0017/ADR-0020 campaign): a relationship ovpopulation
    // does not publish reads as absent, matching getRelationshipTargets below
    // (REQ-PARSE-CORE-003 AC-9). This must stay paired with the same removal there --
    // dropping it here alone re-creates the predicate/getter disagreement, and dropping
    // it there alone leaves this answering true for targets the getter no longer returns.
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
                    out.push_back(internKey(ids[i]));
            }
        };
        auto tryHandle = [&](uint64_t handle) -> LoadRelationshipLookup
        {
            const LoadCacheCell* cell = loadCacheCell(handle, rel.id);
            if (!cell)
                return LoadRelationshipLookup::eUnknown;
            if (const DLTensor* t = cell->relationship)
            {
                if (!canDecodeRelationshipTargets(*t))
                    return LoadRelationshipLookup::eUnknown;
                const int64_t n = totalElements(*t);
                const uint8_t* base = static_cast<const uint8_t*>(t->data) + t->byte_offset;
                appendIds(reinterpret_cast<const uint64_t*>(base), n);
                return LoadRelationshipLookup::eValue;
            }
            // A relationship ovpopulation resolved into a dense uint64 column lands in the
            // scalar slot of the same cell.
            if (const DLTensor* t = cell->scalar.tensor)
            {
                const CachedTensorRow& row = cell->scalar;
                if (!canDecodeRelationshipTargets(*t) || row.comps <= 0)
                    return LoadRelationshipLookup::eUnknown;
                const int64_t elemBytes = t->dtype.bits / 8;
                const uint8_t* base = static_cast<const uint8_t*>(t->data) + t->byte_offset +
                                      static_cast<int64_t>(row.row) * row.comps * elemBytes;
                appendIds(reinterpret_cast<const uint64_t*>(base), row.comps);
                return LoadRelationshipLookup::eValue;
            }
            return cell->covered ? LoadRelationshipLookup::eCoveredMiss : LoadRelationshipLookup::eUnknown;
        };

        // eValue: the load cache holds this prim's targets — done. eCoveredMiss:
        // the cache covers the prim but has no value row for this relationship.
        // Mirror the scalar/attribute read path (see tryHandle for getAttribute):
        // only trust a covered-miss as authoritative-empty when there is NO backing
        // USD stage. With a backing stage attached, fall through to the live
        // data-plane read below — a relationship authored after the load cache was
        // built (e.g. a runtime joint body re-target) would otherwise be missed.
        // (For an instance with no backing stage, a covered-miss is the authoritative
        // empty and honours an ovstage rel-clear.)
        const uint64_t raw = rawHandle(key);
        if (raw)
        {
            const LoadRelationshipLookup result = tryHandle(raw);
            if (result == LoadRelationshipLookup::eValue)
                return;
            if (result == LoadRelationshipLookup::eCoveredMiss && mLoadCacheMayBeStale == 0)
                return;
        }
        const uint64_t canonical = canonicalHandleRaw(raw);
        if (out.empty() && canonical && canonical != raw)
        {
            const LoadRelationshipLookup result = tryHandle(canonical);
            if (result == LoadRelationshipLookup::eValue)
                return;
            if (result == LoadRelationshipLookup::eCoveredMiss && mLoadCacheMayBeStale == 0)
                return;
        }
    }

    // No prim on the stage authors this column (stage-wide vocabulary, rebuilt every drain): the
    // live read below could only come back empty, so spare the round trip.
    if (stageHasAttributeColumn(conv::toOvstageColumn(relName)) == std::optional<bool>(false))
        return;

    // ovpopulation writes each relationship as a ragged uint64 column of
    // ovx_primpath_t target ids (one row per authoring prim), named by the
    // relationship (ovpopulation_physics.inl pass 3). Read this prim's row.
    relationshipReadCounter().fetch_add(1, std::memory_order_relaxed);
    withAttributeTensor(rawHandle(key), relName,
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
                                    out.push_back(internKey(ids[i]));
                        });
    // No backing-USD fallback -- paired with the same removal in hasRelationship above
    // (ADR-0017/ADR-0020 campaign). A relationship ovpopulation does not publish yields no
    // targets, rather than being answered from a stage a USD-free consumer does not have.
}

void OvstageSource::getInactiveInstanceIds(ObjectKey key, std::vector<int64_t>& out) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    out.clear();

    // Integer columns arrive in whichever width the producer chose; normalise to int64.
    // `values` is filled, not appended to - the int64 branch below assigns, so clear up front to
    // keep every width on the same contract.
    auto readIntColumn = [&](const char* column, std::vector<int64_t>& values)
    {
        values.clear();
        withAttributeTensor(rawHandle(key), column,
                            [&](const DLTensor& t, uint32_t /*primCount*/)
                            {
                                const int64_t total = totalElements(t);
                                if (total <= 0 || !t.data)
                                    return;

                                const auto* base = static_cast<const uint8_t*>(t.data) + t.byte_offset;
                                values.reserve(static_cast<size_t>(total));
                                if (t.dtype.code == kDLInt && t.dtype.bits == 64)
                                {
                                    const auto* v = reinterpret_cast<const int64_t*>(base);
                                    values.assign(v, v + total);
                                }
                                else if (t.dtype.code == kDLInt && t.dtype.bits == 32)
                                {
                                    const auto* v = reinterpret_cast<const int32_t*>(base);
                                    for (int64_t i = 0; i < total; ++i)
                                        values.push_back(static_cast<int64_t>(v[i]));
                                }
                                else if (t.dtype.code == kDLUInt && t.dtype.bits == 64)
                                {
                                    const auto* v = reinterpret_cast<const uint64_t*>(base);
                                    for (int64_t i = 0; i < total; ++i)
                                        values.push_back(static_cast<int64_t>(v[i]));
                                }
                                else if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
                                {
                                    const auto* v = reinterpret_cast<const uint32_t*>(base);
                                    for (int64_t i = 0; i < total; ++i)
                                        values.push_back(static_cast<int64_t>(v[i]));
                                }
                            });
    };

    // ovpopulation authors the inactiveIds column natively; the PointInstancer
    // inactive-id set has no remaining USD fallback.
    readIntColumn(conv::kInactiveIds, out);
    if (out.empty())
        return;

    // inactiveIds names instances by their `ids` value whenever that column is authored, and
    // positionally only when it is not - the same rule USD applies (see UsdSource). This
    // interface hands back positions, so authored ids have to be translated here.
    std::vector<int64_t> instanceIds;
    readIntColumn(conv::kIds, instanceIds);
    if (instanceIds.empty())
        return;

    std::unordered_map<int64_t, size_t> idToInstance;
    idToInstance.reserve(instanceIds.size());
    for (size_t i = 0; i < instanceIds.size(); i++)
    {
        // duplicate ids are ill formed - keep the first, which is what UsdGeomPointInstancer does
        idToInstance.insert({ instanceIds[i], i });
    }

    std::vector<int64_t> positions;
    positions.reserve(out.size());
    for (int64_t id : out)
    {
        const auto found = idToInstance.find(id);
        if (found == idToInstance.end())
        {
            CARB_LOG_WARN("Physics:PointInstancer: (%s) inactiveIds entry %lld does not match any authored id, ignoring",
                          std::string(sourceKeyToString(key)).c_str(), (long long)id);
            continue;
        }
        positions.push_back(static_cast<int64_t>(found->second));
    }
    out.swap(positions);
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
        return loadCacheCovers(handle, attr.id);
    };
    auto applyLoadWorldTransform = [&](uint64_t handle) -> bool
    {
        const LoadCachePrim* prim = loadCachePrim(handle);
        if (!prim)
            return false;
        const CachedTensorRow& row = prim->world;
        return row.tensor && decodeMatrixAt(*row.tensor, row.comps, row.row, outMatrix.data);
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
        const uint64_t raw = rawHandle(key);
        if (applyLoadWorldTransform(raw))
            return;
        const uint64_t canonical = canonicalHandleRaw(raw);
        if (canonical != raw && applyLoadWorldTransform(canonical))
            return;
        worldTransformMissCoveredByLoadCache =
            (raw && loadCacheCoversTransform(raw, worldTransformAttr)) ||
            (canonical && canonical != raw && loadCacheCoversTransform(canonical, worldTransformAttr));
    }

    if (mBucketActive && mBucketTransformsComplete)
    {
        const bool worldTransformCovered = mBucketAttributeIds.count(worldTransformAttr.id) != 0;
        const uint64_t raw = rawHandle(key);
        if (applyCachedWorldTransform(raw))
            return;
        const uint64_t canonical = canonicalHandleRaw(raw);
        if (canonical != raw && applyCachedWorldTransform(canonical))
            return;
        worldTransformMissCoveredByBucket =
            worldTransformCovered &&
            (mBucketKeys.count(raw) != 0 || (canonical != raw && mBucketKeys.count(canonical) != 0));
    }

    bool gotWorld = false;
    if (!worldTransformMissCoveredByLoadCache && !worldTransformMissCoveredByBucket)
    {
        withAttributeTensor(rawHandle(key), conv::kFabricWorldMatrix,
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
    const uint64_t keyHandle = canonicalHandleRaw(rawHandle(key));
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
        const uint64_t curHandle = canonicalHandleRaw(rawHandle(cur));
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

void OvstageSource::getLocalTransformImpl(ObjectKey keyIn, ReadTime /*time*/, bool /*usdAtEarliestTime*/,
                                          Matrix4d& outMatrix, bool& outResetsXformStack) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    outMatrix = Matrix4d{};
    outResetsXformStack = false;
    const uint64_t key = rawHandle(keyIn);
    if (!key)
        return;

    const TokenId localTransformAttr = doInternToken(conv::kLocalTransform);
    const TokenId fabricLocalTransformAttr = doInternToken(conv::kFabricLocalMatrix);
    const TokenId resetXformStackAttr = doInternToken(conv::kResetXformStack);
    auto loadCacheCoversLocalAttr = [&](uint64_t handle, TokenId attr) -> bool
    {
        return loadCacheCovers(handle, attr.id);
    };
    auto applyLoadLocalRefs = [&](uint64_t handle, bool& foundLocal, bool& foundReset)
    {
        const LoadCachePrim* prim = loadCachePrim(handle);
        if (!prim)
            return;
        const CachedTensorRow& local = prim->local;
        if (local.tensor && decodeMatrixAt(*local.tensor, local.comps, local.row, outMatrix.data))
        {
            foundLocal = true;
        }
        const CachedTensorRow& reset = prim->reset;
        if (reset.tensor)
        {
            const AttrValue v = decodeAt(*reset.tensor, reset.comps, reset.row);
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
    };

    if (mLoadCacheActive)
    {
        bool foundLocal = false;
        bool foundReset = false;
        bool localCovered = false;
        bool resetCovered = false;
        const uint64_t raw = key;
        if (raw)
        {
            applyLoadLocalRefs(raw, foundLocal, foundReset);
            localCovered = loadCacheCoversLocalAttr(raw, localTransformAttr) ||
                           loadCacheCoversLocalAttr(raw, fabricLocalTransformAttr);
            resetCovered = loadCacheCoversLocalAttr(raw, resetXformStackAttr);
        }
        const uint64_t canonical = canonicalHandleRaw(key);
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
        if (mLoadCacheMayBeStale == 0 && (foundLocal || localCovered) && (foundReset || resetCovered))
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
            if (!foundLocal && localTransformCovered && resetXformStackCovered && mLoadCacheMayBeStale == 0)
                return true;
            return foundLocal;
        };

        const uint64_t raw = key;
        if (applyCachedTransform(raw))
            return;
        const uint64_t canonical = canonicalHandleRaw(key);
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

    // omni::physx::decomposeMatrix (common/foundation/MatrixTools.h) reproduces
    // GfTransform::GetRotation()/GetScale() to float precision, including the
    // sign convention on reflections: a mirror scale reports all three scale
    // components negative, matching Gf, rather than the always-non-negative
    // length a per-row normalization would give. (Shear from non-uniform
    // parent scale under rotation is still not separated by either
    // decomposition -- a pre-existing, shared limitation.)
    ::physx::PxTransform pose;
    ::physx::PxVec3 scale;
    omni::physx::decomposeMatrix(pose, scale, wm.data);

    // PxMat33(quat) and the row-major Matrix3d this function returns hold
    // identical elements (MatrixTools.h's "PxMat33(q) vs GfMatrix3f(q)" note):
    // Px's column i is our row i, no transpose needed.
    const ::physx::PxMat33 rot(pose.q);
    outRotation.data[0] = rot.column0.x;
    outRotation.data[1] = rot.column0.y;
    outRotation.data[2] = rot.column0.z;
    outRotation.data[3] = rot.column1.x;
    outRotation.data[4] = rot.column1.y;
    outRotation.data[5] = rot.column1.z;
    outRotation.data[6] = rot.column2.x;
    outRotation.data[7] = rot.column2.y;
    outRotation.data[8] = rot.column2.z;
    outScale = { scale.x, scale.y, scale.z };
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

    // `curveVertexCounts` splits a BasisCurves prim's points into its curves; unmapped, every
    // multi-curve spline path collapses into one curve built from the first CVs only.
    if (name == "protoIndices" || name == "faceVertexIndices" || name == "faceVertexCounts" ||
        name == "holeIndices" || name == "curveVertexCounts")
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

    // Deformable attachment + element-collision-filter topology.
    //
    // These are the arrays that say WHICH vertices/elements an attachment binds.
    // `UsdInterfaceDeformableAttachment::createDeformableAttachment` reads them through
    // `getArrayValue` -> `getArrayAttribute`; an unmapped name returns an invalid handle,
    // `getArrayValue` returns false, and the attachment is then built from an EMPTY index
    // array. That is silent: the attachment object is created, no error is logged, and the
    // deformable simply behaves as if nothing were attached (a pinned cloth free-falls).
    // Caught by the surface-deformable arm of TestTensorDeformableMaterial, whose cloth
    // stretched under USD and fell under OVStage.
    if (name == "omniphysics:vtxIndicesSrc0" || name == "omniphysics:vtxIndicesSrc1" ||
        name == "omniphysics:tetIndicesSrc0" || name == "omniphysics:tetIndicesSrc1" ||
        name == "omniphysics:triIndicesSrc0" || name == "omniphysics:triIndicesSrc1")
        return readArrayBuffer(key, name, BufferElemType::eInt32, 1, time);
    if (name == "omniphysics:localPositionsSrc1" || name == "omniphysics:tetCoordsSrc0" ||
        name == "omniphysics:tetCoordsSrc1" || name == "omniphysics:triCoordsSrc0" ||
        name == "omniphysics:triCoordsSrc1")
        return readArrayBuffer(key, name, BufferElemType::eVec3, 3, time);
    if (name == "omniphysics:groupElemCounts0" || name == "omniphysics:groupElemCounts1" ||
        name == "omniphysics:groupElemIndices0" || name == "omniphysics:groupElemIndices1")
        return readArrayBuffer(key, name, BufferElemType::eUInt32, 1, time);

    // PhysxPhysicsJointInstancer index and local-frame arrays. These carry the
    // `physics:` prefix, so they share no name with the UsdGeomPointInstancer
    // arrays above and need their own entries. `parseJointInstancer` returns early
    // when the proto-index array is unreadable, so the whole instancer — every
    // joint it stamps — silently does not exist rather than being mis-parametrised.
    if (name == "physics:protoIndices" || name == "physics:body0Indices" ||
        name == "physics:body1Indices")
        return readArrayBuffer(key, name, BufferElemType::eInt32, 1, time);
    if (name == "physics:localPos0s" || name == "physics:localPos1s")
        return readArrayBuffer(key, name, BufferElemType::eVec3, 3, time);
    if (name == "physics:localRot0s" || name == "physics:localRot1s")
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
    withAttributeTensor(rawHandle(key), attr,
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
            else if (type == BufferElemType::eUInt32)
            {
                // Unsigned topology payloads (deformable element-collision-filter group
                // counts / indices). Same normalisation as the signed branch, to uint32.
                std::vector<uint32_t> tmp(static_cast<size_t>(total));
                if (t.dtype.code == kDLUInt && t.dtype.bits == 32)
                    std::memcpy(tmp.data(), base, static_cast<size_t>(total) * sizeof(uint32_t));
                else if (t.dtype.code == kDLInt && t.dtype.bits == 32)
                    for (int64_t i = 0; i < total; ++i)
                        tmp[i] = static_cast<uint32_t>(reinterpret_cast<const int32_t*>(base)[i]);
                else if (t.dtype.code == kDLUInt && t.dtype.bits == 64)
                    for (int64_t i = 0; i < total; ++i)
                        tmp[i] = static_cast<uint32_t>(reinterpret_cast<const uint64_t*>(base)[i]);
                else if (t.dtype.code == kDLInt && t.dtype.bits == 64)
                    for (int64_t i = 0; i < total; ++i)
                        tmp[i] = static_cast<uint32_t>(reinterpret_cast<const int64_t*>(base)[i]);
                else
                    return;
                h = registerMeshBuffer(tmp.data(), tmp.size() * sizeof(uint32_t), elemCount, type);
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
    // ADR-0017: `time` is deliberately unused -- see getAttributeAtTime. There is no
    // backing-USD fallback: an array the producer does not publish reads as absent, and the
    // caller's own "no value" path (an empty index array, a skipped overlay) is the answer.
    (void)time;
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

MeshGeometry OvstageSource::getMeshAttributes(ObjectKey key, bool includeFaceMaterials) const
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

    // faceMaterials: the per-face physics-material index, derived from the mesh's
    // material-bound GeomSubset children. The index scheme must match the one the
    // walker builds sourceMaterials with and the one the cooking path expects:
    // unbound faces take `materialCount` (one past the last) and each physics-bound
    // face subset, in child order, stamps its faces with the next 0-based index.
    // Left invalid when the mesh has no physics-material subsets, which cooking
    // treats as single-material.
    // Per-face materials come from material-bound GeomSubset children (a live child walk
    // per mesh). Only multi-material triangle/SDF cooking needs them; callers opt out.
    if (includeFaceMaterials && out.faceCounts.valid() && out.faceCounts.elemCount > 0)
    {
        const TokenId elementTypeAttr = doInternToken("elementType");
        const TokenId faceToken = doInternToken("face");
        const TokenId geomSubsetType = doInternToken("GeomSubset");
        const TokenId physicsMaterialApi = doInternToken("PhysicsMaterialAPI");

        std::vector<ObjectKey> physicsSubsets;
        forEachChild(backingKey,
                     [&](ObjectKey child)
                     {
                         if (!isA(child, geomSubsetType))
                             return;
                         TokenId elementType{};
                         if (!getAttribute(child, elementTypeAttr, elementType) || elementType != faceToken)
                             return;
                         const ObjectKey material = getMaterialBinding(child);
                         if (material.valid() && hasSchema(material, physicsMaterialApi))
                             physicsSubsets.push_back(child);
                     });

        if (!physicsSubsets.empty())
        {
            const size_t faceCount = out.faceCounts.elemCount;
            std::vector<uint16_t> faceMaterials(faceCount, static_cast<uint16_t>(physicsSubsets.size()));
            uint16_t materialIndex = 0;
            for (const ObjectKey subset : physicsSubsets)
            {
                const BufferHandle faceIndices =
                    readArrayBuffer(subset, "indices", BufferElemType::eInt32, 1);
                if (faceIndices.valid())
                {
                    size_t byteCount = 0;
                    if (const auto* faces = static_cast<const int32_t*>(resolveBuffer(faceIndices, byteCount)))
                    {
                        for (size_t i = 0; i < faceIndices.elemCount; ++i)
                            if (faces[i] >= 0 && static_cast<size_t>(faces[i]) < faceCount)
                                faceMaterials[static_cast<size_t>(faces[i])] = materialIndex;
                    }
                    releaseBuffer(faceIndices);
                }
                ++materialIndex;
            }
            out.faceMaterials = registerMeshBuffer(faceMaterials.data(),
                                                   faceMaterials.size() * sizeof(uint16_t),
                                                   static_cast<uint32_t>(faceMaterials.size()),
                                                   BufferElemType::eUInt16);
        }
    }

    return out;
}

// --- source-wide -----------------------------------------------------------

void OvstageSource::loadUnits()
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    if (!mInstance || !mDict)
        return;
    // ovstage 374259+ populates stage metadata onto the root prim `/`, under the
    // reserved `usd-metadata:` column prefix, and only when a populate desc's
    // `stage_metadata_paths` asked for it. Older/controlled instances still author
    // it, unprefixed, on the `/__ovstage_population_stage_info__` (or legacy
    // `/__ovpopulation_stage_info__`) prim. Read the root-prim form first, then fall
    // back to the special-prim form, so both populate paths keep working.
    auto readUnitsFrom = [&](const char* primPath, const std::string& prefix) -> bool
    {
        const ObjectKey info = findByPath(primPath);
        if (!info.valid())
            return false;
        const uint64_t infoRaw = rawHandle(info);
        bool any = false;
        withAttributeTensor(infoRaw, prefix + conv::kMetersPerUnit,
                            [&](const DLTensor& t, uint32_t primCount)
                            {
                                const AttrValue v = decodeScalar(t, primCount);
                                if (v.kind == AttrValue::Kind::eDouble)
                                    mUnits.metersPerUnit = static_cast<float>(v.d);
                                else if (v.kind == AttrValue::Kind::eFloat)
                                    mUnits.metersPerUnit = v.f;
                                else
                                    return;
                                any = true;
                            });
        withAttributeTensor(infoRaw, prefix + conv::kKilogramsPerUnit,
                            [&](const DLTensor& t, uint32_t primCount)
                            {
                                const AttrValue v = decodeScalar(t, primCount);
                                if (v.kind == AttrValue::Kind::eDouble)
                                    mUnits.kilogramsPerUnit = static_cast<float>(v.d);
                                else if (v.kind == AttrValue::Kind::eFloat)
                                    mUnits.kilogramsPerUnit = v.f;
                                else
                                    return;
                                any = true;
                            });
        withAttributeTensor(infoRaw, prefix + conv::kUpAxis,
                            [&](const DLTensor& t, uint32_t primCount)
                            {
                                // Stored as a token id (uint64 → AttrValue int); resolve
                                // through the dictionary to the axis name.
                                const AttrValue v = decodeScalar(t, primCount);
                                if (v.kind != AttrValue::Kind::eInt)
                                    return;
                                ovx_string_t s{};
                                if (ovx_path_dictionary_token_to_string(mDict, static_cast<ovx_token_t>(v.i), &s) ==
                                        OVX_OK &&
                                    s.ptr)
                                {
                                    const std::string_view axis(s.ptr, s.length);
                                    mUnits.upAxis = (axis == "Y") ? UpAxis::eY : UpAxis::eZ;
                                    any = true;
                                }
                            });
        return any;
    };

    if (readUnitsFrom(conv::kRootPrimPath, conv::kMetadataPrefix))
        return;
    if (readUnitsFrom(conv::kStageInfoPath, std::string()))
        return;
    readUnitsFrom(conv::kLegacyStageInfoPath, std::string());
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
            const uint64_t canonical = canonicalHandleRaw(rawHandle(member));
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
            const uint64_t canonical = canonicalHandleRaw(rawHandle(member));
            if (!canonical || excluded.count(canonical) != 0)
                continue;
            if (emitted.insert(canonical).second)
                members.push_back(internKey(canonical));
        }
    }
}

ObjectKey OvstageSource::getMaterialBinding(ObjectKey primKey) const
{
    std::lock_guard<std::recursive_mutex> lock(mMutex);
    // The physics material binding rides on the USD MaterialBindingAPI
    // relationship for the "physics" purpose; ovpopulation resolves and writes it
    // as a target path-id column (same as any relationship). (Collection-based
    // bindings are not handled here — direct binding is the physics norm.)
    auto bindingFor = [this](ObjectKey key, const char* relName, std::vector<ObjectKey>& out)
    {
        out.clear();
        if (!key.valid())
            return false;
        getRelationshipTargets(key, internToken(relName), out);
        return !out.empty();
    };

    // Inherited binding. USD resolves a material binding up the ancestor chain
    // (UsdShadeMaterialBindingAPI::ComputeBoundMaterial, which UsdSource calls),
    // so a binding authored on a parent Xform applies to its descendants — the
    // usual shape for an instanced subtree, where the binding sits on the
    // instance root and the collider is a descendant.
    //
    // Testing the prim first and ancestors afterwards yields
    // weakerThanDescendants precedence, which is the USD default; the
    // strongerThanDescendants inversion is not modelled (nor are collection-based
    // bindings — direct binding is the physics norm).
    auto chainBindingFor = [&](const char* relName, std::vector<ObjectKey>& out) -> ObjectKey
    {
        if (bindingFor(primKey, relName, out))
            return out.front();
        if (!primKey.valid())
            return {};
        // Walk ancestor paths via plain string slicing on the last '/' -- prim
        // names cannot contain '/', so this is exact, not an approximation.
        // Mirrors SdfPath::GetParentPath's walk exactly, including that it stops
        // one level short of the absolute root ("/") -- the pseudo-root prim is
        // never itself tested for a binding.
        std::string path = pathOfRaw(rawHandle(primKey));
        while (!path.empty())
        {
            const size_t slash = path.rfind('/');
            if (slash == std::string::npos)
                break;
            const std::string parent = (slash == 0) ? "/" : path.substr(0, slash);
            if (parent == "/")
                break;
            const ObjectKey parentKey = findByPath(parent);
            if (bindingFor(parentKey, relName, out))
                return out.front();
            path = parent;
        }
        return {};
    };

    std::vector<ObjectKey> targets;

    // No prim on the stage authors a binding column of any purpose (stage-wide vocabulary,
    // rebuilt every drain): the two chain walks below have nothing to find, and only the
    // instance-proxy material can still answer.
    const bool anyBindingColumn = stageHasAttributeWithPrefix("material:binding") != std::optional<bool>(false);

    // Purpose ordering is the OUTER loop, the ancestor chain the inner one —
    // UsdShadeMaterialBindingAPI::ComputeBoundMaterial resolves the requested
    // purpose over the whole chain before falling back to the all-purpose
    // binding, so a "physics"-purpose binding on an ancestor outranks an
    // all-purpose binding authored on the prim itself. Resolving purpose per prim
    // instead inverted that and handed the collider the all-purpose material.
    if (anyBindingColumn)
    {
        const ObjectKey physicsBinding = chainBindingFor("material:binding:physics", targets);
        if (physicsBinding.valid())
            return physicsBinding;

        // The prim's own all-purpose binding, then the instance-proxy material
        // (which stands in for the instanced prim's own binding and carries no
        // purpose), then the all-purpose binding inherited from an ancestor.
        if (bindingFor(primKey, "material:binding", targets))
            return targets.front();
    }

    if (primKey.valid() && buildInstanceMaterialCache())
    {
        const uint64_t primKeyRaw = rawHandle(primKey);
        std::unordered_map<uint64_t, uint64_t>::const_iterator binding =
            mInstanceMaterialByPrim.find(primKeyRaw);
        if (binding == mInstanceMaterialByPrim.end())
        {
            const uint64_t canonical = canonicalHandleRaw(primKeyRaw);
            binding = mInstanceMaterialByPrim.find(canonical);
        }
        if (binding != mInstanceMaterialByPrim.end())
            return internKey(binding->second);
    }

    return anyBindingColumn ? chainBindingFor("material:binding", targets) : ObjectKey{};
}

std::unique_ptr<IChangeFeed> OvstageSource::createChangeFeed()
{
    if (!mInstance || !mDict)
        return nullptr;
    return std::make_unique<OvstageChangeFeed>(*this, mInstance, mDict);
}

} // namespace omni::physics::ovstage
