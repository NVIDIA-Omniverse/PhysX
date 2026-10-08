// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONSUMER-001
 * @covers AC-24
 *
 * @implements REQ-COOK-SOURCE-001
 * @covers AC-5
 */

/**
 * @implements REQ-MATH-001
 * @covers AC-7
 */

/**
 * @implements REQ-PARSE-INSTANCER-002
 * @covers AC-1
 */

/**
 * @implements REQ-PARSE-CORE-003
 * @covers AC-14
 */

#pragma once

#include "OmniPhysX.h"
#include "PhysXDefines.h"
#include "internal/InternalScene.h"

#include "ObjectDataQuery.h"
#include "PhysXToolsCore.h"

#include "usdLoad/AttachedStage.h"
#include "usdLoad/PrimUpdate.h"

#include <omni/physics/parse/IPhysicsSource.h>
#include <omni/physics/parse/KnownTokens.h>

#include "utils/Pair.h"

#include <carb/logging/Log.h>
#include <private/omni/physx/PhysxUsd.h>
#include <carb/events/IEvents.h>
#include <omni/physx/IPhysx.h>


#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>

#include <common/utilities/Utilities.h>
#include <common/foundation/CarbPhysXCast.h>
#include <common/foundation/Algorithms.h>
#include <common/foundation/MatrixTools.h>

#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <unordered_set>
#include <vector>

namespace omni
{
namespace physx
{

// pxr-free degToRad/radToDeg overloads (float, PxVec3, carb::Float3) live in PhysXToolsCore.h.

template <class Type>
inline Type* getPtr(PhysXType type, omni::physx::usdparser::ObjectId id)
{
    const internal::InternalPhysXDatabase& internaldb = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (id == omni::physx::usdparser::kInvalidObjectId)
        return nullptr;
    void* ptr = internaldb.getTypedRecord(type, id);
    return reinterpret_cast<Type*>(ptr);
}

template <class Type>
inline Type* getInternalPtr(PhysXType type, omni::physx::usdparser::ObjectId id)
{
    const internal::InternalPhysXDatabase& internaldb = OmniPhysX::getInstance().getInternalPhysXDatabase();
    if (id == omni::physx::usdparser::kInvalidObjectId)
        return nullptr;
    void* ptr = internaldb.getInternalTypedRecord(type, id);
    return reinterpret_cast<Type*>(ptr);
}

namespace internal
{

// removeFilteredObject/swapFilteredObject, the collision-group/filter-pair <-> PxFilterData
// conversions, and sendErrorEvent are pxr-free and live in PhysXToolsCore.h (included above).

// ---------------------------------------------------------------------------
// ObjectKey-keyed overloads. InternalDatabase::Record stores an ObjectKey
// (source-agnostic identity) rather than an SdfPath, so these route through
// AttachedStage::getSource() instead of reaching into USD directly.
//
// Scalar reads (`getValue`, `getFloatBounded`) go through
// `IPhysicsSource::getAttributeAtTime`. Array and relationship reads still
// use the path-based USD helpers -- the source has no general array-read
// path (it surfaces only mesh-geometry buffers) and these have few call sites.
// ---------------------------------------------------------------------------
namespace physxtools_detail
{
// The VtArray fillArray/elemTypeOf/arrayElemData ladders below stay fenced: no pxr-free
// equivalent exists, and every production caller reaches them only by instantiating the
// TokenId+ReadTime getArrayValue/setCookedArrayValue below with a VtArray<T>.

// Unpack an AttrValue into a USD-typed `out`, matching the kind a USD attribute of that
// type resolves to (see UsdSource::vtValueToAttrValue). Mirrors UsdAttribute::Get: each
// returns false and leaves `out` untouched on a kind mismatch or absent attribute.
using AttrValue = omni::physics::parse::AttrValue;

// The carb-typed / plain-scalar fromAttr(bool/float/double/int/uint32_t/carb::Float2/
// carb::Float3/carb::Float4/std::string) overloads are pxr-free and live in
// PhysXToolsCore.h (included above).

// ---------------------------------------------------------------------------
// Gf-free array reads (ADR-0001 s8) -- std::vector<carb::*> mirrors of the VtArray ladder
// above. fillVector, the static_asserts backing the layout equivalences the memcpys rely on,
// and the fillArray(std::vector<float/int32_t/uint32_t/uint8_t/carb::Float2/carb::Float3/
// carb::Int3/carb::Int4>) overloads are pxr-free and live in PhysXToolsCore.h (included above).
//
// The carb::Float4 overload stays here (not PhysXToolsCore.h): it covers both eVec4, a plain
// memcpy via the PhysXToolsCore.h fillVector, and eQuath, a half->float widen.
// ---------------------------------------------------------------------------
inline float halfBitsToFloat(uint16_t bits)
{
    const uint32_t sign = uint32_t(bits & 0x8000u) << 16;
    uint32_t exponent = (bits >> 10) & 0x1fu;
    uint32_t mantissa = bits & 0x3ffu;
    uint32_t out;
    if (exponent == 0)
    {
        if (mantissa == 0)
        {
            out = sign; // signed zero
        }
        else
        {
            // Subnormal half: normalize the mantissa into a normal single-precision float.
            exponent = 127 - 15 + 1;
            while ((mantissa & 0x400u) == 0)
            {
                mantissa <<= 1;
                --exponent;
            }
            mantissa &= 0x3ffu;
            out = sign | (exponent << 23) | (mantissa << 13);
        }
    }
    else if (exponent == 0x1fu)
    {
        out = sign | 0x7f800000u | (mantissa << 13); // inf / nan
    }
    else
    {
        out = sign | ((exponent - 15 + 127) << 23) | (mantissa << 13);
    }
    float result;
    std::memcpy(&result, &out, sizeof(result));
    return result;
}

inline bool fillArray(std::vector<carb::Float4>& out, const void* d, size_t n,
                      const omni::physics::parse::BufferHandle& h)
{
    if (h.type == omni::physics::parse::BufferElemType::eQuath)
    {
        if (!d)
            return false;
        // Same header/payload agreement fillVector enforces, for the 4-half-lane element the
        // widen reads: without it the loop indexes src[elemCount*4-1] on a buffer that may be
        // shorter than the header claims. Checked before the resize; fail closed, don't truncate.
        if (uint64_t(n) != uint64_t(h.elemCount) * 4 * sizeof(uint16_t))
            return false;
        out.resize(h.elemCount);
        const uint16_t* src = static_cast<const uint16_t*>(d);
        for (size_t i = 0; i < h.elemCount; ++i)
        {
            out[i] = carb::Float4{ halfBitsToFloat(src[i * 4 + 0]), halfBitsToFloat(src[i * 4 + 1]),
                                   halfBitsToFloat(src[i * 4 + 2]), halfBitsToFloat(src[i * 4 + 3]) };
        }
        return true;
    }
    return fillVector(out, d, n, h, omni::physics::parse::BufferElemType::eVec4);
}
// fillArray(std::vector<carb::Int3/carb::Int4>&, ...) are pxr-free and live in
// PhysXToolsCore.h (included above).

// Gf-free mirrors, so setCookedArrayValue accepts a std::vector<carb::*> payload. All of
// elemTypeOf(std::vector<float/int32_t/uint32_t/uint8_t/carb::Float2/carb::Float3/carb::Float4/
// carb::Int3/carb::Int4>) are pxr-free and live in PhysXToolsCore.h (included above).

// Raw element pointer of an array payload. VtArray spells it `cdata()` (the
// const form, which does not detach the copy-on-write buffer), std::vector
// spells it `data()`; setCookedArrayValue below takes either. The std::vector<ElemT> overload
// is pxr-free and lives in PhysXToolsCore.h (included above).

// ---------------------------------------------------------------------------
// Cooked-geometry carrier (ADR-0022) — the shared attribute allowlist.
//
// The carrier is only ever populated by the deformable cooking write-back, so
// in principle map membership alone bounds it. The allowlist makes that a
// matter of code rather than of trust: the write-back refuses to record an
// attribute outside this set (loudly), and the read hook refuses to serve one.
// A token that drifts out of step with
// CookingDataAsync.cpp::store{Volume,Surface}DeformableBodyDataToUsd therefore
// shows up as an error at cook time instead of as a silent read miss.
//
// The `physx*` markers are spelled as literals because CookingDataAsync.cpp's
// matching TfTokens are file-static; the `omniphysics:*` ones mirror
// OmniUsdPhysicsDeformableSchemaTokens, whose header is not on this file's
// include path.
// ---------------------------------------------------------------------------
// isCookedGeometryAttribute(std::string_view) is pxr-free and lives in PhysXToolsCore.h.

// Carrier entry for (`key`, `attr`), or null when the carrier must not answer.
//
// Three gates, cheapest first:
//   1. nothing was ever cooked on this attach (the common case, one branch);
//   2. THE LOAD-BEARING ONE — a live write sink exists, so the scene
//      description is authoritative and the carrier must stay inert. `points`
//      and `velocities` are not cook-time-immutable: simulation rewrites them
//      every frame through that same sink and InternalScene reads them back, so
//      an ungated carrier would serve the cook's bind-pose points forever and
//      break save/restore on a USD attach. This gate is also what makes the
//      whole hook a provable no-op on the USD backend;
//   3. the attribute is not one the cooking write-back authors.
inline const usdparser::CookedArray* findCookedArray(const usdparser::AttachedStage& attachedStage,
                                                     omni::physics::parse::ObjectKey key,
                                                     omni::physics::parse::TokenId attr,
                                                     const omni::physics::parse::IPhysicsSource& source)
{
    if (!attachedStage.hasCookedGeometry())
        return nullptr;
    if (attachedStage.getDataWrite() != nullptr)
        return nullptr;
    // An unresolvable path yields the invalid sentinel key, and every such prim
    // would share one carrier slot. Never record or serve under it.
    if (!key.valid())
        return nullptr;
    if (!isCookedGeometryAttribute(source.tokenToString(attr)))
        return nullptr;
    return attachedStage.getCookedArray(key, attr);
}

// Rebuild a BufferHandle over carrier bytes so the fillArray ladder above can
// be reused unchanged — it reads only `type` and `elemCount` off the handle.
// A non-zero `id` only marks the handle valid(); it is never resolved.
inline omni::physics::parse::BufferHandle cookedBufferHandle(const usdparser::CookedArray& cooked)
{
    omni::physics::parse::BufferHandle h;
    h.id = 1;
    h.type = cooked.type;
    h.elemCount = cooked.elemCount;
    return h;
}
} // namespace physxtools_detail

template <typename T>
bool getValue(const usdparser::AttachedStage& attachedStage,
              omni::physics::parse::ObjectKey key,
              omni::physics::parse::TokenId attributeName,
              omni::physics::parse::ReadTime time,
              T& retVal)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    if constexpr (std::is_same_v<T, uint32_t>)
    {
        // Cooked-geometry carrier (ADR-0014).
        if (const usdparser::CookedArray* cooked =
                physxtools_detail::findCookedArray(attachedStage, key, attributeName, *source))
        {
            if (cooked->type == omni::physics::parse::BufferElemType::eUInt32 && cooked->elemCount == 1 &&
                cooked->bytes.size() == sizeof(uint32_t))
            {
                std::memcpy(&retVal, cooked->bytes.data(), sizeof(uint32_t));
                return true;
            }
        }
    }
    if constexpr (std::is_same_v<T, omni::physics::parse::TokenId>)
    {
        // Read token-valued attributes (enum/mode strings) through the typed TokenId
        // getter, not getAttributeAtTime+fromAttr: sources that store tokens as an
        // int-encoded token-id column (ovstage) surface the raw AttrValue as eInt, which
        // has no fromAttr<TokenId> match, so the generic path below would silently drop
        // the value. USD's token attrs resolve identically through this path too.
        return source->getAttribute(key, attributeName, retVal);
    }
    else
    {
        const omni::physics::parse::AttrValue value = source->getAttributeAtTime(key, attributeName, time);
        return physxtools_detail::fromAttr(*source, value, retVal);
    }
}

// Known gap: a *recorded but empty* carrier entry falls through to the source here rather
// than being served as an explicit empty result. No production call site distinguishes the
// two today.
template <typename T>
bool getArrayValue(const usdparser::AttachedStage& attachedStage,
                   omni::physics::parse::ObjectKey key,
                   omni::physics::parse::TokenId attributeName,
                   omni::physics::parse::ReadTime time,
                   T& retVal)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    // Cooked-geometry carrier (ADR-0014/ADR-0022).
    if (const usdparser::CookedArray* cooked =
            physxtools_detail::findCookedArray(attachedStage, key, attributeName, *source))
    {
        if (!cooked->bytes.empty() &&
            physxtools_detail::fillArray(retVal, cooked->bytes.data(), cooked->bytes.size(),
                                         physxtools_detail::cookedBufferHandle(*cooked)))
        {
            return true;
        }
    }
    const omni::physics::parse::BufferHandle handle = source->getArrayAttribute(key, attributeName, time);
    if (!handle.valid())
        return false;
    size_t byteCount = 0;
    const void* data = source->resolveBuffer(handle, byteCount);
    const bool ok = physxtools_detail::fillArray(retVal, data, byteCount, handle);
    // Per-call runtime read: drop the buffer so it doesn't accumulate.
    source->releaseBuffer(handle);
    return ok;
}

// ---------------------------------------------------------------------------
// Cooked-geometry carrier — write side (ADR-0022).
//
// The deformable cooking write-back calls these next to (not instead of) its
// IPhysicsDataWrite calls: record always, publish to the sink when there is
// one. Recording unconditionally keeps the write path's behaviour independent
// of the backend; the READ side is where the no-sink gate lives.
// ---------------------------------------------------------------------------

// Gated by the allowlist (physxtools_detail::isCookedGeometryAttribute, via
// source->tokenToString) and following the unconditional-record-then-publish
// contract (ADR-0014).
inline void setCookedRawValue(usdparser::AttachedStage& attachedStage,
                              omni::physics::parse::ObjectKey key,
                              omni::physics::parse::TokenId attributeName,
                              const void* bytes,
                              size_t byteCount,
                              omni::physics::parse::BufferElemType type,
                              uint32_t elemCount)
{
    omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source || !key.valid())
        return;
    if (!physxtools_detail::isCookedGeometryAttribute(source->tokenToString(attributeName)))
    {
        // Loud on purpose: see the TfToken overload above.
        CARB_LOG_ERROR("setCookedRawValue: %s is not in the cooked-geometry allowlist (PhysXTools.h); "
                       "it will not be recorded and cannot be read back on a sink-less backend.",
                       std::string(source->tokenToString(attributeName)).c_str());
        return;
    }
    attachedStage.setCookedArray(key, attributeName, bytes, byteCount, type, elemCount);
}

template <typename T>
void setCookedArrayValue(usdparser::AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey key,
                         omni::physics::parse::TokenId attributeName,
                         const T& array)
{
    setCookedRawValue(attachedStage, key, attributeName,
                      array.empty() ? nullptr : physxtools_detail::arrayElemData(array),
                      array.size() * sizeof(typename T::value_type), physxtools_detail::elemTypeOf(array),
                      static_cast<uint32_t>(array.size()));
}

inline void setCookedUIntValue(usdparser::AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey key,
                               omni::physics::parse::TokenId attributeName,
                               uint32_t value)
{
    setCookedRawValue(attachedStage, key, attributeName, &value, sizeof(value),
                      omni::physics::parse::BufferElemType::eUInt32, 1);
}

inline void setCookedBlobValue(usdparser::AttachedStage& attachedStage,
                               omni::physics::parse::ObjectKey key,
                               omni::physics::parse::TokenId attributeName,
                               const void* bytes,
                               size_t byteCount)
{
    setCookedRawValue(attachedStage, key, attributeName, bytes, byteCount,
                      omni::physics::parse::BufferElemType::eUInt8, static_cast<uint32_t>(byteCount));
}

inline void clearCookedValue(usdparser::AttachedStage& attachedStage,
                             omni::physics::parse::ObjectKey key,
                             omni::physics::parse::TokenId attributeName)
{
    attachedStage.clearCookedArray(key, attributeName);
}

// toPxMat44d/toParseMatrix4d (the PhysX <-> parse-lib Matrix4d conversion) are pxr-free and
// live in PhysXToolsCore.h (included above).

inline ::physx::PxMat44d getWorldTransform(const usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key,
                                            omni::physics::parse::ReadTime time)
{
    omni::physics::parse::Matrix4d m;
    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
        source->getLocalToWorldTransform(key, time, m);
    return toPxMat44d(m);
}

// Local-to-world transform via the source's cached, time-independent overload
// (pinned to EarliestTime). Use for load-time reads that previously went
// through an EarliestTime UsdGeomXformCache — same result, shared cache.
inline ::physx::PxMat44d getWorldTransform(const usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key)
{
    omni::physics::parse::Matrix4d m;
    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
        source->getLocalToWorldTransform(key, m);
    return toPxMat44d(m);
}

// Object-local transform of `key` at `timeCode` (the transform from the
// object's own ops, before its parent frame), read through the physics source
// (no direct USD prim access). `outResetsXformStack` receives whether the
// object resets the inherited parent transform. Returns identity when the
// source is unavailable or the key does not resolve.
inline ::physx::PxMat44d getLocalTransform(const usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key,
                                            omni::physics::parse::ReadTime time,
                                            bool& outResetsXformStack)
{
    omni::physics::parse::Matrix4d m;
    outResetsXformStack = false;
    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
        source->getLocalTransform(key, time, m, outResetsXformStack);
    return toPxMat44d(m);
}

// A tet mesh is identified by its TET DATA, not only by its concrete prim type.
//
// isA() resolves against the source's prim-type vocabulary, and ovstage reports a
// UsdGeomTetMesh as plain "Mesh" -- its populator has no TetMesh mapping -- so requiring the
// concrete type rejects every volume deformable loaded from a non-USD source. This is the same
// limitation the ovstage walker's `usd-prim-type` query has, and the same remedy applied there
// (OvstageWalker.cpp::hasTetConnectivity): fall back to the presence of readable, non-empty
// tetVertexIndices, which is what the consumers of these gates actually need. The isA() fast
// path stays first, so behaviour is unchanged on every backend.
//
// The fast path checks KnownTokens' tetMeshType TokenId, interned from the literal "TetMesh".
// On the USD backend that resolves through the same UsdSchemaRegistry::
// GetTypeFromSchemaTypeName()+UsdPrim::IsA() path a C++ isAType<UsdGeomTetMesh>() would.
inline bool isTetMeshLike(const usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key)
{
    const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
    if (!src)
        return false;
    const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
    if (src->isA(key, tok.tetMeshType))
        return true;
    // eInt4 payload; carb::Int4 and GfVec4i are the same four packed int32, so this is the
    // same read the VtArray<GfVec4i> form performed.
    std::vector<carb::Int4> tets;
    return getArrayValue(attachedStage, key, tok.tetVertexIndices, omni::physics::parse::ReadTime::defaultTime(),
                         tets) &&
           !tets.empty();
}

// Returns the relationship's targets as ObjectKeys directly.
inline bool getRelationshipValue(const usdparser::AttachedStage& attachedStage,
                                 omni::physics::parse::ObjectKey key,
                                 omni::physics::parse::TokenId relName,
                                 std::vector<omni::physics::parse::ObjectKey>& retVal)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    retVal.clear();
    source->getRelationshipTargets(key, relName, retVal);
    return true;
}

inline bool hasRelationship(const usdparser::AttachedStage& attachedStage,
                            omni::physics::parse::ObjectKey key,
                            omni::physics::parse::TokenId relName)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    return source && source->hasRelationship(key, relName);
}

inline bool hasMultiApplyInstance(const usdparser::AttachedStage& attachedStage,
                                  omni::physics::parse::ObjectKey key,
                                  omni::physics::parse::TokenId schemaName,
                                  omni::physics::parse::TokenId instance)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    const std::string appliedSchema =
        std::string(source->tokenToString(schemaName)) + ":" + std::string(source->tokenToString(instance));
    return source->hasSchema(key, source->internToken(appliedSchema));
}

inline ::physx::PxQuat fixupCapsuleQuat(omni::physx::usdparser::Axis axis)
{
    ::physx::PxQuat fixupQ(::physx::PxIdentity);
    const float hRt2 = sqrt(2.0f) / 2.0f;
    if (axis == usdparser::eZ)
    {
        fixupQ = ::physx::PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
    }
    else if (axis == usdparser::eY)
    {
        fixupQ = ::physx::PxQuat(hRt2, -hRt2, 0.0f, 0.0f);
    }
    return fixupQ;
}

inline ::physx::PxQuat fixupConeAndCylinderQuat(omni::physx::usdparser::Axis axis)
{
    ::physx::PxQuat fixupQ(::physx::PxIdentity);
    if (axis == usdparser::eZ)
        fixupQ = ::physx::PxQuat(::physx::PxPiDivTwo, ::physx::PxVec3(0, -1, 0));
    else if (axis == usdparser::eY)
        fixupQ = ::physx::PxQuat(::physx::PxPiDivTwo, ::physx::PxVec3(0, 0, 1));

    return fixupQ;
}

// pxr-free sibling of the SdfPath overload below: identical body, keyed by
// ObjectKey (ADR-0019) via the getObjectDataOrID ObjectKey overload in
// ObjectDataQuery.h, so ADR-0018 callers can look up the joint's PhysX
// pointer without constructing an SdfPath.
inline bool getJointAndLocalPose(const omni::physx::usdparser::AttachedStage& attachedStage,
                                 omni::physics::parse::ObjectKey jointKey,
                                 const ::physx::PxRigidActor* jointActor,
                                 ::physx::PxBase*& jointOut,
                                 ::physx::PxTransform& localFrame)
{
    ::physx::PxBase* joint =
        reinterpret_cast<::physx::PxBase*>(omni::physx::getObjectDataOrID<omni::physx::ObjectDataQueryType::ePHYSX_PTR>(
            jointKey, ePTJoint, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    if (!joint)
    {
        joint = reinterpret_cast<::physx::PxBase*>(
            omni::physx::getObjectDataOrID<omni::physx::ObjectDataQueryType::ePHYSX_PTR>(
                jointKey, ePTLinkJoint, OmniPhysX::getInstance().getInternalPhysXDatabase(), attachedStage));
    }

    if (joint && joint->getConcreteType() == ::physx::PxJointConcreteType::eD6)
    {
        ::physx::PxRigidActor* actor0;
        ::physx::PxRigidActor* actor1;

        ::physx::PxJoint* jointPtr = (::physx::PxJoint*)joint;
        jointPtr->getActors(actor0, actor1);
        if (jointActor == actor0)
            localFrame = jointPtr->getLocalPose(::physx::PxJointActorIndex::eACTOR0);
        if (jointActor == actor1)
            localFrame = jointPtr->getLocalPose(::physx::PxJointActorIndex::eACTOR1);
    }
    else if (joint && joint->getConcreteType() == ::physx::PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE)
    {
        ::physx::PxArticulationJointReducedCoordinate* jointPtr = (::physx::PxArticulationJointReducedCoordinate*)joint;

        if (jointActor == &jointPtr->getParentArticulationLink())
            localFrame = jointPtr->getParentPose();
        else if (jointActor == &jointPtr->getChildArticulationLink())
        {
            localFrame = jointPtr->getChildPose();
        }
        else
        {
            return false;
        }
    }

    jointOut = joint;
    return true;
}

#define UNKNOWN_FACE_ID 0xffffffff

class FaceIndexResolve
{
public:
    FaceIndexResolve(::physx::PxShape* shape) : mShape(shape)
    {
    }

    uint32_t resolveFaceIndex(uint32_t faceIndex)
    {
        uint32_t retVal = 0;
        const ::physx::PxGeometry& geom = mShape->getGeometry();
        if (geom.getType() == ::physx::PxGeometryType::eTRIANGLEMESH)
        {
            if (faceIndex == UNKNOWN_FACE_ID)
                return UNKNOWN_FACE_ID;

            const ::physx::PxTriangleMeshGeometry& triGeom = static_cast<const ::physx::PxTriangleMeshGeometry&>(geom);
            const ::physx::PxTriangleMesh* mesh = triGeom.triangleMesh;
            if (mesh && faceIndex < mesh->getNbTriangles())
            {
                const uint32_t remappedIndex = mesh->getTrianglesRemap()[faceIndex];
                const uint32_t* remmapedTriangle = getMeshCache()->getTriangleMeshFaceMap(mesh);
                if (remmapedTriangle)
                    retVal = remmapedTriangle[remappedIndex];
            }
            else
            {
                return UNKNOWN_FACE_ID;
            }
        }
        return retVal;
    }

private:
    ::physx::PxShape* mShape;
};

} // namespace internal
} // namespace physx
} // namespace omni
