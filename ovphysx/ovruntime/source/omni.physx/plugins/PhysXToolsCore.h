// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-14
 */

#pragma once

// pxr-free half of PhysXTools.h, split out so USD-free consumers (ADR-0018) can pull in the
// angle-conversion helpers, filter-data/collision-group conversions, filtered-pair-set
// maintenance, error-event forwarding, and the PhysX<->parse-lib Matrix4d conversion without
// dragging in PhysXTools.h's pxr includes (and its AttachedStage-bound getValue/getArrayValue
// ladder, which is genuinely USD-bound). PhysXTools.h includes this header too, so existing
// direct includers of PhysXTools.h are unaffected.

#include <carb/Types.h>
#include <carb/events/IEvents.h>

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/Math.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/ObjectId.h>

#include "utils/Pair.h"

#include <PxPhysicsAPI.h>

#include <common/utilities/CoreUtilities.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni
{
namespace physx
{

inline float degToRad(const float a)
{
    return 0.01745329251994329547f * a;
}

inline float radToDeg(const float a)
{
    return 57.29577951308232286465f * a;
}

inline ::physx::PxVec3 degToRad(const ::physx::PxVec3& a)
{
    return ::physx::PxVec3(0.01745329251994329547f * a);
}

inline ::physx::PxVec3 radToDeg(const ::physx::PxVec3& a)
{
    return ::physx::PxVec3(57.29577951308232286465f * a);
}

// carb-typed forms of the PxVec3 pair above, so a site that has migrated its angular
// velocities off GfVec3f does not have to route back through Gf to scale them.
// Same constants, same per-lane float arithmetic as the PxVec3 overloads.
inline carb::Float3 degToRad(const carb::Float3& a)
{
    return carb::Float3{ 0.01745329251994329547f * a.x, 0.01745329251994329547f * a.y,
                         0.01745329251994329547f * a.z };
}

inline carb::Float3 radToDeg(const carb::Float3& a)
{
    return carb::Float3{ 57.29577951308232286465f * a.x, 57.29577951308232286465f * a.y,
                         57.29577951308232286465f * a.z };
}

namespace internal
{

template <typename T>
inline void removeFilteredObject(T ptr, std::unordered_map<Pair<T>, uint32_t, PairHash>& pairSet)
{
    typedef typename std::unordered_map<Pair<T>, uint32_t, PairHash>::iterator iterator;

    iterator it = pairSet.begin();
    while (it != pairSet.end())
    {
        if (it->first.contains(ptr))
        {
            it = pairSet.erase(it);
        }
        else
        {
            it++;
        }
    }
}

template <typename T>
inline void removeFilteredObject(T ptr, std::unordered_set<Pair<T>, PairHash>& pairSet)
{
    typedef typename std::unordered_set<Pair<T>, PairHash>::iterator iterator;

    iterator it = pairSet.begin();
    while (it != pairSet.end())
    {
        if ((*it).contains(ptr))
        {
            it = pairSet.erase(it);
        }
        else
        {
            it++;
        }
    }
}

template <typename T>
inline void swapFilteredObject(T oldPtr, T newPtr, std::unordered_set<Pair<T>, PairHash>& pairSet)
{
    typedef typename std::unordered_set<Pair<T>, PairHash>::iterator iterator;

    iterator it = pairSet.begin();
    while (it != pairSet.end())
    {
        (*it).swap(oldPtr, newPtr);
        it++;
    }
}

inline uint32_t convertToCollisionGroup(const usdparser::ObjectId collisionGroupId)
{
    const uint32_t collisionGroup =
        (collisionGroupId == usdparser::kInvalidObjectId) ? 0 : uint32_t(size_t(collisionGroupId));
    return collisionGroup;
}

inline void convertCollisionGroupToPxFilterData(const uint32_t collisionGroup, ::physx::PxFilterData& filterData)
{
    filterData.word2 = collisionGroup;
    // in word1 we store pair filtering information
    // in word3 we store contact modify information
}

inline uint32_t convertCollisionGroupFromPxFilterData(const ::physx::PxFilterData& fd)
{
    return fd.word2;
}

inline void convertFilterPairToPxFilterData(const uint32_t filterPair, ::physx::PxFilterData& filterData)
{
    filterData.word1 = filterPair;
    // in word2 we store collision group
    // in word3 we store contact modify information
}

inline uint32_t convertFilterPairFromPxFilterData(const ::physx::PxFilterData& fd)
{
    return fd.word1;
}


template <typename... ValuesT>
void sendErrorEvent(carb::events::IEventStreamPtr eventStream, ErrorEvent type, ValuesT... values)
{
    ::sendErrorEvent(eventStream, static_cast<carb::events::EventType>(type), values...);
}

// parse-lib Matrix4d and PxMat44d hold the same sixteen doubles: the first
// three groups of four are the X/Y/Z basis vectors, [12..14] is the translation.
// GfMatrix4d reads that row-major with a row-vector convention and PxMat44d
// column-major with a column-vector convention, so the bytes copy straight
// across and only the multiplication order flips (Gf A*B == PhysX B*A).
// See common/foundation/MatrixTools.h for the full mapping table.
inline ::physx::PxMat44d toPxMat44d(const omni::physics::parse::Matrix4d& m)
{
    double values[16];
    for (int i = 0; i < 16; ++i)
        values[i] = m.data[i];
    return ::physx::PxMat44d(values);
}

inline omni::physics::parse::Matrix4d toParseMatrix4d(const ::physx::PxMat44d& m)
{
    omni::physics::parse::Matrix4d out;
    const double* values = m.front();
    for (int i = 0; i < 16; ++i)
        out.data[i] = values[i];
    return out;
}

// pxr-free subset of PhysXTools.h's physxtools_detail helpers (ADR-0018): the carb-typed /
// plain-scalar overloads of the fromAttr/fillArray/elemTypeOf/arrayElemData ladders PhysXTools.h's
// TokenId-typed getValue/getArrayValue resolve to for carb-typed T, plus the TokenId-keyed
// isCookedGeometryAttribute. Each is a pxr-free sibling of a same-named, Gf/VtArray-typed
// overload that stays in PhysXTools.h; the ladders are split by parameter type, not moved
// wholesale. (getValue/getArrayValue themselves, and the findCookedArray/cookedBufferHandle pair,
// stay in PhysXTools.h: they take usdparser::AttachedStage / usdparser::CookedArray, and
// usdLoad/AttachedStage.h is itself unconditionally pxr-dependent with no pxr-free sibling of its
// own yet, so including it here would defeat this split.)
namespace physxtools_detail
{

// Unpack an AttrValue into a carb-typed / plain-scalar `out`. Gf-free siblings of the
// GfVec2f/GfVec3f/GfQuatf/TfToken overloads in PhysXTools.h; each accepts exactly the same
// AttrValue kinds as its Gf-typed sibling.
using AttrValue = omni::physics::parse::AttrValue;

inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, bool& out)
{
    if (v.kind == AttrValue::Kind::eBool) { out = v.b; return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, float& out)
{
    // Mirrors USD's implicit float-family coercion (UsdAttribute::Get): float, double, or
    // half values all read as float, else getValue<float> would silently keep the caller's
    // default. eHalf stores its value in the float slot (see AttrValue::makeHalf).
    if (v.kind == AttrValue::Kind::eFloat || v.kind == AttrValue::Kind::eHalf) { out = v.f; return true; }
    if (v.kind == AttrValue::Kind::eDouble) { out = static_cast<float>(v.d); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, double& out)
{
    if (v.kind == AttrValue::Kind::eDouble) { out = v.d; return true; }
    if (v.kind == AttrValue::Kind::eFloat || v.kind == AttrValue::Kind::eHalf) { out = static_cast<double>(v.f); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, int& out)
{
    if (v.kind == AttrValue::Kind::eInt) { out = static_cast<int>(v.i); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, uint32_t& out)
{
    if (v.kind == AttrValue::Kind::eInt) { out = static_cast<uint32_t>(v.i); return true; }
    return false;
}
// carb-typed vector reads (ADR-0001 s8). These are the Gf-free equivalents of the
// GfVec2f/GfVec3f/GfQuatf overloads in PhysXTools.h and accept exactly the same AttrValue
// kinds, so getValue<carb::Float3> and getValue<GfVec3f> agree lane-for-lane -- migrating a
// call site is a retype of the destination and nothing else.
//
// carb::Float4 covers BOTH a float4 attribute and a quaternion one: UsdSource packs
// GfQuatf into Float4 as {x, y, z, w} == {imaginary, real}, which is the carb::Float4
// lane convention, so no reordering happens here (contrast the GfQuatf overload in
// PhysXTools.h, which has to put the real part first for GfQuatf's ctor).
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, carb::Float2& out)
{
    if (v.kind == AttrValue::Kind::eFloat2) { out = v.f2; return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, carb::Float3& out)
{
    if (v.kind == AttrValue::Kind::eFloat3) { out = v.f3; return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, carb::Float4& out)
{
    if (v.kind == AttrValue::Kind::eFloat4) { out = v.f4; return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, std::string& out)
{
    if (v.kind == AttrValue::Kind::eString) { out = v.str; return true; }
    return false;
}

// ---------------------------------------------------------------------------
// Gf-free array reads (ADR-0001 s8) -- std::vector<carb::*> mirrors of PhysXTools.h's VtArray
// fillArray ladder.
//
// Each accepts exactly the same BufferElemType as its VtArray counterpart and
// copies the same bytes, so `getArrayValue<std::vector<carb::Float3>>` and
// `getArrayValue<VtVec3fArray>` return identical data. The layout equivalences
// the memcpys rely on are asserted, not assumed:
//   carb::Float2/3/4 == 2/3/4 packed floats  (== GfVec2f/GfVec3f/GfVec4f)
//   carb::Int3/Int4  == 3/4 packed int32     (== GfVec3i/GfVec4i)
//
// The std::vector<carb::Float4> overload stays in PhysXTools.h, not here: eQuath is served
// through carb::Float4 too (not by a dedicated std::vector<carb::Float4> mirror of its own),
// and that half->float widen touches GfQuath, so despite its carb-typed signature that
// overload is not actually pxr-free.
// ---------------------------------------------------------------------------
static_assert(sizeof(carb::Float2) == 2 * sizeof(float), "carb::Float2 must be 2 packed floats");
static_assert(sizeof(carb::Float3) == 3 * sizeof(float), "carb::Float3 must be 3 packed floats");
static_assert(sizeof(carb::Float4) == 4 * sizeof(float), "carb::Float4 must be 4 packed floats");
static_assert(sizeof(carb::Int3) == 3 * sizeof(int32_t), "carb::Int3 must be 3 packed int32");
static_assert(sizeof(carb::Int4) == 4 * sizeof(int32_t), "carb::Int4 must be 4 packed int32");

template <typename ElemT>
inline bool fillVector(std::vector<ElemT>& out,
                       const void* data,
                       size_t byteCount,
                       const omni::physics::parse::BufferHandle& h,
                       omni::physics::parse::BufferElemType expected)
{
    if (h.type != expected || !data)
        return false;
    // Header/payload agreement, checked before the resize: `out` is sized from the handle's
    // elemCount but the copy is byteCount long, so a payload larger than the header claims
    // writes past the end of `out`. Fail closed rather than truncate -- a disagreement means
    // the handle does not describe this buffer, and a short copy would silently hand the
    // caller half-initialised elements. 64-bit product so the multiply cannot wrap.
    if (uint64_t(byteCount) != uint64_t(h.elemCount) * sizeof(ElemT))
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}

inline bool fillArray(std::vector<float>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eFloat);
}
inline bool fillArray(std::vector<int32_t>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eInt32);
}
inline bool fillArray(std::vector<uint32_t>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eUInt32);
}
inline bool fillArray(std::vector<uint8_t>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eUInt8);
}
inline bool fillArray(std::vector<carb::Float2>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eVec2);
}
inline bool fillArray(std::vector<carb::Float3>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eVec3);
}
inline bool fillArray(std::vector<carb::Int3>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eInt3);
}
inline bool fillArray(std::vector<carb::Int4>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eInt4);
}

// Buffer element type of a std::vector<carb::*> payload -- the Gf-free mirror of
// PhysXTools.h's elemTypeOf(VtArray) ladder. Used by the cooked-geometry write-back to record
// an array in the same layout IPhysicsSource::resolveBuffer would hand out (ADR-0014).
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<float>&)
{
    return omni::physics::parse::BufferElemType::eFloat;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<int32_t>&)
{
    return omni::physics::parse::BufferElemType::eInt32;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<uint32_t>&)
{
    return omni::physics::parse::BufferElemType::eUInt32;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<uint8_t>&)
{
    return omni::physics::parse::BufferElemType::eUInt8;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<carb::Float2>&)
{
    return omni::physics::parse::BufferElemType::eVec2;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<carb::Float3>&)
{
    return omni::physics::parse::BufferElemType::eVec3;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<carb::Float4>&)
{
    return omni::physics::parse::BufferElemType::eVec4;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<carb::Int3>&)
{
    return omni::physics::parse::BufferElemType::eInt3;
}
inline omni::physics::parse::BufferElemType elemTypeOf(const std::vector<carb::Int4>&)
{
    return omni::physics::parse::BufferElemType::eInt4;
}

// Raw element pointer of a std::vector<carb::*> array payload -- the Gf-free mirror of
// PhysXTools.h's arrayElemData(VtArray) overload (VtArray spells it `cdata()`, std::vector
// spells it `data()`); setCookedArrayValue takes either.
template <typename ElemT>
inline const void* arrayElemData(const std::vector<ElemT>& a)
{
    return a.data();
}

// pxr-free sibling of the TfToken-typed isCookedGeometryAttribute in PhysXTools.h, same
// allowlist and suffix rule, for TokenId-keyed callers (ADR-0018) that never construct a
// TfToken.
inline bool isCookedGeometryAttribute(std::string_view attributeName)
{
    static const std::unordered_set<std::string_view> kCookedAttributes = {
        "points",
        "velocities",
        "tetVertexIndices",
        "surfaceFaceVertexIndices",
        "faceVertexCounts",
        "faceVertexIndices",
        "omniphysics:restShapePoints",
        "omniphysics:restTetVtxIndices",
        "omniphysics:restTriVtxIndices",
        "physxDeformableBody:deformableBodyDataCrc",
        "physxVolumeDeformableSim:numTetsPerElement",
        "physxVolumeDeformableSim:simMeshHexCrc",
    };
    if (kCookedAttributes.count(attributeName) != 0)
        return true;

    // Per-instance bind pose: deformablePose:<instance>:omniphysics:points.
    static constexpr std::string_view kBindPoseSuffix(":omniphysics:points");
    return attributeName.size() > kBindPoseSuffix.size() &&
           attributeName.compare(attributeName.size() - kBindPoseSuffix.size(), kBindPoseSuffix.size(),
                                 kBindPoseSuffix) == 0;
}

} // namespace physxtools_detail

} // namespace internal
} // namespace physx
} // namespace omni
