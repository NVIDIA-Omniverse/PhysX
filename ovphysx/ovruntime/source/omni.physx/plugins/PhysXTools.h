// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include "OmniPhysX.h"
#include "PhysXDefines.h"
#include "internal/InternalScene.h"

#include "ObjectDataQuery.h"

#include "usdLoad/AttachedStage.h"
#include "usdLoad/PrimUpdate.h"

#include <omni/physics/parse/IPhysicsSource.h>

#include "utils/Pair.h"

#include <carb/logging/Log.h>
#include <private/omni/physx/PhysxUsd.h>
#include <carb/events/IEvents.h>
#include <omni/physx/IPhysx.h>


#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>

#include <common/utilities/Utilities.h>
#include <common/foundation/TypeCast.h>
#include <common/foundation/Algorithms.h>

#include <cstring>
#include <set>

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

inline PXR_NS::GfVec3f degToRad(const PXR_NS::GfVec3f& a)
{
    return PXR_NS::GfVec3f(0.01745329251994329547f * a);
}

inline PXR_NS::GfVec3f radToDeg(const PXR_NS::GfVec3f& a)
{
    return PXR_NS::GfVec3f(57.29577951308232286465f * a);
}

inline ::physx::PxVec3 radToDeg(const ::physx::PxVec3& a)
{
    return ::physx::PxVec3(57.29577951308232286465f * a);
}

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

// Forward declaration of the ObjectKey-keyed, source-backed getValue (defined below). The
// SdfPath form resolves the path to an ObjectKey and forwards, so path-keyed reads also
// route through IPhysicsSource rather than reaching into USD directly.
template <typename T>
bool getValue(const usdparser::AttachedStage& attachedStage,
              omni::physics::parse::ObjectKey key,
              const PXR_NS::TfToken& attributeName,
              const PXR_NS::UsdTimeCode& timeCode,
              T& retVal);

template <typename T>
bool getValue(const usdparser::AttachedStage& attachedStage,
              const PXR_NS::SdfPath& path,
              const PXR_NS::TfToken& attributeName,
              const PXR_NS::UsdTimeCode& timeCode,
              T& retVal)
{
    // pathFor(keyFor(path)) == path, so this is an exact identity vs the prior
    // direct GetPrimAtPath(path) read — it works for any prim, parsed or not.
    return getValue<T>(attachedStage, attachedStage.keyFor(path), attributeName, timeCode, retVal);
}

// Forward declaration of the ObjectKey-keyed, source-backed getArrayValue
// (defined below). The SdfPath form resolves the path to an ObjectKey and
// forwards, so path-keyed array reads also route through IPhysicsSource.
template <typename T>
bool getArrayValue(const usdparser::AttachedStage& attachedStage,
                   omni::physics::parse::ObjectKey key,
                   const PXR_NS::TfToken& attributeName,
                   const PXR_NS::UsdTimeCode& timeCode,
                   T& retVal);

template <typename T>
bool getArrayValue(const usdparser::AttachedStage& attachedStage,
                   const PXR_NS::SdfPath& path,
                   const PXR_NS::TfToken& attributeName,
                   const PXR_NS::UsdTimeCode& timeCode,
                   T& retVal)
{
    return getArrayValue<T>(attachedStage, attachedStage.keyFor(path), attributeName, timeCode, retVal);
}

inline bool getRelationshipValue(const usdparser::AttachedStage& attachedStage,
                                 omni::physics::parse::ObjectKey key,
                                 const PXR_NS::TfToken& relName,
                                 PXR_NS::SdfPathVector& retVal);

inline bool getRelationshipValue(const usdparser::AttachedStage& attachedStage,
                                 const PXR_NS::SdfPath& path,
                                 const PXR_NS::TfToken& relName,
                                 PXR_NS::SdfPathVector& retVal)
{
    return getRelationshipValue(attachedStage, attachedStage.keyFor(path), relName, retVal);
}

inline bool getFloatBounded(const usdparser::AttachedStage& attachedStage,
                            const PXR_NS::SdfPath& path,
                            const PXR_NS::TfToken attributeName,
                            const PXR_NS::UsdTimeCode timeCode,
                            float& outFloat,
                            const float lowBound,
                            const float upBound)
{
    float data = 0.0f;
    const bool result = getValue<float>(attachedStage, path, attributeName, timeCode, data);

    if (data > upBound)
    {
        data = upBound;
    }
    else if (data < lowBound)
    {
        data = lowBound;
    }
    outFloat = data;
    return result;
}

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
inline omni::physics::parse::ReadTime toReadTime(const PXR_NS::UsdTimeCode& timeCode)
{
    return timeCode.IsDefault() ? omni::physics::parse::ReadTime::defaultTime() :
                                  omni::physics::parse::ReadTime::at(timeCode.GetValue());
}

// Unpack an AttrValue into a USD-typed `out`, matching the kind a USD attribute of that
// type resolves to (see UsdSource::vtValueToAttrValue). Mirrors UsdAttribute::Get: each
// returns false and leaves `out` untouched on a kind mismatch or absent attribute.
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
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, PXR_NS::GfVec2f& out)
{
    if (v.kind == AttrValue::Kind::eFloat2) { out = PXR_NS::GfVec2f(v.f2.x, v.f2.y); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, PXR_NS::GfVec3f& out)
{
    if (v.kind == AttrValue::Kind::eFloat3) { out = PXR_NS::GfVec3f(v.f3.x, v.f3.y, v.f3.z); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, PXR_NS::GfQuatf& out)
{
    // UsdSource packs GfQuatf as Float4{x, y, z, w} (imaginary then real).
    if (v.kind == AttrValue::Kind::eFloat4) { out = PXR_NS::GfQuatf(v.f4.w, v.f4.x, v.f4.y, v.f4.z); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource& src, const AttrValue& v, PXR_NS::TfToken& out)
{
    if (v.kind == AttrValue::Kind::eToken) { out = PXR_NS::TfToken(std::string(src.tokenToString(v.tok))); return true; }
    return false;
}
inline bool fromAttr(const omni::physics::parse::IPhysicsSource&, const AttrValue& v, std::string& out)
{
    if (v.kind == AttrValue::Kind::eString) { out = v.str; return true; }
    return false;
}

// Copy a resolved array buffer into a USD VtArray `out`. Element type must
// match the buffer's; `out` is resized and filled from the raw bytes.
inline bool fillArray(PXR_NS::VtFloatArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    if (h.type != omni::physics::parse::BufferElemType::eFloat || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtVec2fArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // eVec2 is 2 packed floats per element, layout-compatible with GfVec2f.
    if (h.type != omni::physics::parse::BufferElemType::eVec2 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtVec3fArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // eVec3 is 3 packed floats per element, layout-compatible with GfVec3f.
    if (h.type != omni::physics::parse::BufferElemType::eVec3 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtVec4fArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // eVec4 is 4 packed floats per element, layout-compatible with GfVec4f.
    if (h.type != omni::physics::parse::BufferElemType::eVec4 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtIntArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // Flat int32 array (e.g. UsdGeomMesh faceVertexIndices).
    if (h.type != omni::physics::parse::BufferElemType::eInt32 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtUIntArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // Flat uint32 array (e.g. deformable collision-filter groupElemCounts/Indices).
    if (h.type != omni::physics::parse::BufferElemType::eUInt32 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtVec3iArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // eInt3 is 3 packed int32 per element, layout-compatible with GfVec3i.
    if (h.type != omni::physics::parse::BufferElemType::eInt3 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtVec4iArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // eInt4 is 4 packed int32 per element, layout-compatible with GfVec4i.
    if (h.type != omni::physics::parse::BufferElemType::eInt4 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtUCharArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // Flat byte array (e.g. cooking mesh-key marker blobs stored as UCharArray).
    if (h.type != omni::physics::parse::BufferElemType::eUInt8 || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
inline bool fillArray(PXR_NS::VtQuathArray& out, const void* data, size_t byteCount,
                      const omni::physics::parse::BufferHandle& h)
{
    // eQuath is 4 packed 16-bit half per element, layout-compatible with GfQuath
    // (instancer orientations / joint-instancer local rotations).
    if (h.type != omni::physics::parse::BufferElemType::eQuath || !data)
        return false;
    out.resize(h.elemCount);
    std::memcpy(out.data(), data, byteCount);
    return true;
}
} // namespace physxtools_detail

template <typename T>
bool getValue(const usdparser::AttachedStage& attachedStage,
              omni::physics::parse::ObjectKey key,
              const PXR_NS::TfToken& attributeName,
              const PXR_NS::UsdTimeCode& timeCode,
              T& retVal)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    const omni::physics::parse::TokenId attr = source->internToken(attributeName.GetString());
    if constexpr (std::is_same_v<T, PXR_NS::TfToken>)
    {
        // Read token attributes through the typed TokenId overload so sources that
        // store tokens as an int-encoded token-id column (ovstage) resolve them; the
        // raw AttrValue surfaces such a column as eInt, which fromAttr<TfToken> (eToken
        // only) would reject. USD's token attrs resolve identically through this path.
        omni::physics::parse::TokenId tokVal;
        if (!source->getAttribute(key, attr, tokVal))
            return false;
        retVal = PXR_NS::TfToken(std::string(source->tokenToString(tokVal)));
        return true;
    }
    else
    {
        const omni::physics::parse::AttrValue value =
            source->getAttributeAtTime(key, attr, physxtools_detail::toReadTime(timeCode));
        return physxtools_detail::fromAttr(*source, value, retVal);
    }
}

template <typename T>
bool getArrayValue(const usdparser::AttachedStage& attachedStage,
                   omni::physics::parse::ObjectKey key,
                   const PXR_NS::TfToken& attributeName,
                   const PXR_NS::UsdTimeCode& timeCode,
                   T& retVal)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    const omni::physics::parse::TokenId attr = source->internToken(attributeName.GetString());
    const omni::physics::parse::BufferHandle handle =
        source->getArrayAttribute(key, attr, physxtools_detail::toReadTime(timeCode));
    if (!handle.valid())
        return false;
    size_t byteCount = 0;
    const void* data = source->resolveBuffer(handle, byteCount);
    const bool ok = physxtools_detail::fillArray(retVal, data, byteCount, handle);
    // Per-call runtime read: drop the buffer so it doesn't accumulate.
    source->releaseBuffer(handle);
    return ok;
}

// Local-to-world transform of `key` at `timeCode`, read through the physics
// source (no direct USD prim access). Returns identity when the source is
// unavailable or the key does not resolve.
inline PXR_NS::GfMatrix4d getWorldTransform(const usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key,
                                            const PXR_NS::UsdTimeCode& timeCode)
{
    omni::physics::parse::Matrix4d m;
    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
        source->getLocalToWorldTransform(key, physxtools_detail::toReadTime(timeCode), m);
    // parse-lib Matrix4d is row-major, matching GfMatrix4d's layout.
    return PXR_NS::GfMatrix4d(m.data[0], m.data[1], m.data[2], m.data[3],
                              m.data[4], m.data[5], m.data[6], m.data[7],
                              m.data[8], m.data[9], m.data[10], m.data[11],
                              m.data[12], m.data[13], m.data[14], m.data[15]);
}

// Local-to-world transform via the source's cached, time-independent overload
// (pinned to EarliestTime). Use for load-time reads that previously went
// through an EarliestTime UsdGeomXformCache — same result, shared cache.
inline PXR_NS::GfMatrix4d getWorldTransform(const usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key)
{
    omni::physics::parse::Matrix4d m;
    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
        source->getLocalToWorldTransform(key, m);
    // parse-lib Matrix4d is row-major, matching GfMatrix4d's layout.
    return PXR_NS::GfMatrix4d(m.data[0], m.data[1], m.data[2], m.data[3],
                              m.data[4], m.data[5], m.data[6], m.data[7],
                              m.data[8], m.data[9], m.data[10], m.data[11],
                              m.data[12], m.data[13], m.data[14], m.data[15]);
}

// Object-local transform of `key` at `timeCode` (the transform from the
// object's own ops, before its parent frame), read through the physics source
// (no direct USD prim access). `outResetsXformStack` receives whether the
// object resets the inherited parent transform. Returns identity when the
// source is unavailable or the key does not resolve.
inline PXR_NS::GfMatrix4d getLocalTransform(const usdparser::AttachedStage& attachedStage,
                                            omni::physics::parse::ObjectKey key,
                                            const PXR_NS::UsdTimeCode& timeCode,
                                            bool& outResetsXformStack)
{
    omni::physics::parse::Matrix4d m;
    outResetsXformStack = false;
    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
        source->getLocalTransform(key, physxtools_detail::toReadTime(timeCode), m, outResetsXformStack);
    // parse-lib Matrix4d is row-major, matching GfMatrix4d's layout.
    return PXR_NS::GfMatrix4d(m.data[0], m.data[1], m.data[2], m.data[3],
                              m.data[4], m.data[5], m.data[6], m.data[7],
                              m.data[8], m.data[9], m.data[10], m.data[11],
                              m.data[12], m.data[13], m.data[14], m.data[15]);
}

// Source token for a C++ USD schema type, for isA/hasSchema gating. This is the single
// boundary point that translates a compile-time USD schema type into the backend's token
// vocabulary: it round-trips the schema's registered type name (USD schema *metadata* --
// not a per-object USD data read) through internToken so any IPhysicsSource backend can
// resolve it. Cached per schema type.
template <typename SchemaT>
omni::physics::parse::TokenId schemaTypeToken(const omni::physics::parse::IPhysicsSource& src)
{
    static const std::string name =
        PXR_NS::UsdSchemaRegistry::GetSchemaTypeName(PXR_NS::TfType::Find<SchemaT>()).GetString();
    return src.internToken(name);
}

// Source-routed applied-API check, keyed by ObjectKey (no UsdPrim, no direct-USD
// fallback). Normalises `schemaTypeName` (accepted by GetAPITypeFromSchemaTypeName,
// e.g. OmniUsdPhysicsDeformableSchemaTokens->OmniPhysicsDeformableBodyAPI) to the registered
// applied-schema name and queries IPhysicsSource::hasSchema. Returns false when
// the name does not resolve to an applied API schema.
inline bool hasAppliedSchema(const omni::physics::parse::IPhysicsSource& src,
                             omni::physics::parse::ObjectKey key,
                             const PXR_NS::TfToken& schemaTypeName)
{
    const PXR_NS::TfType type = PXR_NS::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(schemaTypeName);
    const PXR_NS::TfToken applied = PXR_NS::UsdSchemaRegistry::GetAPISchemaTypeName(type);
    if (applied.IsEmpty())
        return false;
    return src.hasSchema(key, src.internToken(applied.GetString()));
}

// Type-parameterised form of hasAppliedSchema for a C++ applied-API schema class.
template <typename SchemaT>
bool hasAppliedSchema(const omni::physics::parse::IPhysicsSource& src, omni::physics::parse::ObjectKey key)
{
    static const std::string applied =
        PXR_NS::UsdSchemaRegistry::GetAPISchemaTypeName(PXR_NS::TfType::Find<SchemaT>()).GetString();
    return !applied.empty() && src.hasSchema(key, src.internToken(applied));
}

// Source-routed IsA check (the schema's registered type name), keyed by ObjectKey.
template <typename SchemaT>
bool isAType(const omni::physics::parse::IPhysicsSource& src, omni::physics::parse::ObjectKey key)
{
    return src.isA(key, schemaTypeToken<SchemaT>(src));
}

inline bool getRelationshipValue(const usdparser::AttachedStage& attachedStage,
                                 omni::physics::parse::ObjectKey key,
                                 const PXR_NS::TfToken& relName,
                                 PXR_NS::SdfPathVector& retVal)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    const omni::physics::parse::TokenId rel = source->internToken(relName.GetString());
    std::vector<omni::physics::parse::ObjectKey> targets;
    source->getRelationshipTargets(key, rel, targets);
    retVal.clear();
    retVal.reserve(targets.size());
    for (const omni::physics::parse::ObjectKey target : targets)
        retVal.push_back(attachedStage.pathFor(target));
    return true;
}

// True iff the named relationship is defined on `key` (independent of whether
// it has targets), read through the source. Use to distinguish an absent
// relationship from a defined-but-empty one (getRelationshipValue reports empty
// for both).
inline bool hasRelationship(const usdparser::AttachedStage& attachedStage,
                            omni::physics::parse::ObjectKey key,
                            const PXR_NS::TfToken& relName)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    return source && source->hasRelationship(key, source->internToken(relName.GetString()));
}

// True iff multi-apply schema `schemaName` is applied to `key` at `instance`,
// read through the source. `schemaName` is the registered applied-schema base
// name (e.g. "PhysxJointAxisAPI"); the source-side equivalent of
// prim.HasAPI(schemaName, instance) for a specific instance.
inline bool hasMultiApplyInstance(const usdparser::AttachedStage& attachedStage,
                                  omni::physics::parse::ObjectKey key,
                                  const PXR_NS::TfToken& schemaName,
                                  const PXR_NS::TfToken& instance)
{
    const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
    if (!source)
        return false;
    const std::string appliedSchema = schemaName.GetString() + ":" + instance.GetString();
    return source->hasSchema(key, source->internToken(appliedSchema));
}

inline bool getFloatBounded(const usdparser::AttachedStage& attachedStage,
                            omni::physics::parse::ObjectKey key,
                            const PXR_NS::TfToken attributeName,
                            const PXR_NS::UsdTimeCode timeCode,
                            float& outFloat,
                            const float lowBound,
                            const float upBound)
{
    float data = 0.0f;
    const bool result = getValue<float>(attachedStage, key, attributeName, timeCode, data);

    if (data > upBound)
    {
        data = upBound;
    }
    else if (data < lowBound)
    {
        data = lowBound;
    }
    outFloat = data;
    return result;
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

inline ::physx::PxQuat fixupCapsuleQuat(const PXR_NS::TfToken& axis)
{
    ::physx::PxQuat fixupQ(::physx::PxIdentity);
    const float hRt2 = sqrt(2.0f) / 2.0f;
    if (axis == PXR_NS::UsdPhysicsTokens.Get()->z)
    {
        fixupQ = ::physx::PxQuat(hRt2, 0.0f, -hRt2, 0.0f);
    }
    else if (axis == PXR_NS::UsdPhysicsTokens.Get()->y)
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

inline bool getJointAndLocalPose(const omni::physx::usdparser::AttachedStage& attachedStage,
                                 const PXR_NS::SdfPath& jointKey,
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
