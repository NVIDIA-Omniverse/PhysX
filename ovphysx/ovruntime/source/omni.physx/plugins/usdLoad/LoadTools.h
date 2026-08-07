// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <common/utilities/PrimHierarchyStorage.h>
#include <omni/physics/parse/Handles.h>

#include <pxr/base/gf/transform.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdGeom/xformCache.h>

#include <functional>
#include <utility>

#if !CARB_PLATFORM_WINDOWS
#    define sprintf_s snprintf
#endif

namespace omni
{
namespace physx
{
namespace usdparser
{

class AttachedStage;

// Compute the shape's local transform relative to the body, with the body's
// world scale baked into the translation (PhysX does not carry a separate body
// scale). Both objects are identified by ObjectKey (no UsdPrim); world
// transforms are read through the physics source (the relative transform is
// composed as collWorld * bodyWorld^-1). Defined in LoadTools.cpp to keep this
// header free of the runtime PhysXTools dependency.
void getCollisionShapeLocalTransform(const AttachedStage& attachedStage,
                                     omni::physics::parse::ObjectKey collisionKey,
                                     omni::physics::parse::ObjectKey bodyKey,
                                     PXR_NS::GfVec3f& localPosOut,
                                     PXR_NS::GfQuatf& localRotOut,
                                     PXR_NS::GfVec3f& localScaleOut);


// PhysxRigidBodyDesc / PhysxMaterialDesc / PhysxArticulationDesc /
// PhysxJointDesc are aliases to the parse-library types (defined in
// PhysxUsd.h via `using`); a forward struct-declaration would conflict.
// The other types are still defined in usdparser:: directly.
struct PhysxDeformableAttachmentDesc;

struct JointDescAndPath
{
    bool operator<(const JointDescAndPath& jd) const
    {
        return index < jd.index ? true : false;
    }

    PXR_NS::SdfPath path;
    PhysxJointDesc* desc;
    bool articulationJoint;
    uint32_t index;
};

struct DeformableAttachmentDescAndPath
{
    PXR_NS::SdfPath path;
    PhysxDeformableAttachmentDesc* desc;
};

struct DeformableCollisionFilterDescAndPath
{
    PXR_NS::SdfPath path;
    PhysxDeformableCollisionFilterDesc* desc;
};

struct BodyDescAndColliders
{
    omni::physx::usdparser::PhysxRigidBodyDesc* desc;
    std::set<PXR_NS::SdfPath> collisions;
};

struct ShapeDescAndMaterials
{
    PXR_NS::SdfPath path;
    omni::physx::usdparser::PhysxShapeDesc* desc;
    PXR_NS::SdfPathVector materials;
};

struct DeformableDescAndMaterials
{
    PXR_NS::SdfPath path;
    omni::physx::usdparser::PhysxDeformableBodyDesc* desc;
    PXR_NS::SdfPath simMeshMaterial;
};

using ObjectIdMap = std::multimap<ObjectCategory, ObjectId>;
using JointVector = std::vector<JointDescAndPath>;
using JointPathIndexMap = std::unordered_map<PXR_NS::SdfPath, size_t, PXR_NS::SdfPath::Hash>;

using ShapePathList = std::vector<PXR_NS::SdfPath>;
using PathSet = std::unordered_set<PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
using BodyMap = std::map<PXR_NS::SdfPath, BodyDescAndColliders>;
using BodyVector = std::vector<std::pair<PXR_NS::SdfPath, BodyDescAndColliders>>;
using JointMap = std::map<PXR_NS::SdfPath, omni::physx::usdparser::PhysxJointDesc*>;
using JointUnorderedMap = std::unordered_map<PXR_NS::SdfPath, omni::physx::usdparser::PhysxJointDesc*, PXR_NS::SdfPath::Hash>;
using ArticulationMap = std::map<PXR_NS::SdfPath, std::vector<omni::physx::usdparser::PhysxArticulationDesc*>>;
using CollisionBlockPair = std::pair<PXR_NS::SdfPath, PXR_NS::SdfPath>;
using CollisionPairVector = std::vector<CollisionBlockPair>;
using CollisionGroupsMap = std::unordered_map<PXR_NS::SdfPath, PXR_NS::SdfPathVector, PXR_NS::SdfPath::Hash>;
using ObjectIdUsdPrimMap = std::map<usdparser::ObjectId, PXR_NS::UsdPrim>;
// Backend-agnostic shape map for the mass path (ADR-0002 M2c-D): the shape's
// source path keyed by ObjectId, so mass works without live USD prims (ovstage).
using ObjectIdPathMap = std::map<usdparser::ObjectId, PXR_NS::SdfPath>;
using MaterialsVector = std::vector<std::pair<PXR_NS::SdfPath, usdparser::PhysxMaterialDesc*>>;
using DeformableMaterialsVector = std::vector<std::pair<PXR_NS::SdfPath, usdparser::PhysxDeformableMaterialDesc*>>;
using ShapeDescsVector = std::vector<ShapeDescAndMaterials>;
using DeformableBodyDescsVector = std::vector<DeformableDescAndMaterials>;

using FixedTendonVector = std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonFixedDesc>>;
// using FixedTendonMap = PXR_NS::TfHashMap<PXR_NS::TfToken, omni::physx::usdparser::PhysxTendonFixedDesc* ,
// PXR_NS::TfToken::HashFunctor>;
using TendonAxisMap =
    PXR_NS::TfHashMap<PXR_NS::SdfPath, std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonAxisDesc>>, PXR_NS::SdfPath::Hash>;
using SpatialTendonVector = std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonSpatialDesc>>;
using TendonAttachmentMap =
    PXR_NS::TfHashMap<PXR_NS::SdfPath, std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonAttachmentDesc>>, PXR_NS::SdfPath::Hash>;

using MimicJointVector = std::vector<omni::physx::usdparser::MimicJointDesc>;

using PathPhysXDescMap = std::unordered_map<PXR_NS::SdfPath, const PhysxObjectDesc*, PXR_NS::SdfPath::Hash>;

using DeformableAttachmentVector = std::vector<DeformableAttachmentDescAndPath>;
using DeformableCollisionFilterVector = std::vector<DeformableCollisionFilterDescAndPath>;
using DeformableAttachmentHistoryMap = std::unordered_multimap<PXR_NS::SdfPath, PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;
using DeformableCollisionFilterHistoryMap = std::unordered_multimap<PXR_NS::SdfPath, PXR_NS::SdfPath, PXR_NS::SdfPath::Hash>;

struct SchemaAPIFlag
{
    enum Enum : uint64_t
    {
        eRigidBodyAPI = (1 << 0),
        eCollisionAPI = (1 << 1),
        eParticleIsosurfaceAPI = (1 << 2),
        eDiffuseParticlesAPI = (1 << 3),
        eParticleSetAPI = (1 << 4),
        eParticleAnisotropyAPI = (1 << 5),
        eParticleSmoothingAPI = (1 << 6),
        ePhysxForceAPI = (1 << 7),
        eFilteredPairsAPI = (1 << 8),
        eMimicJointRotXAPI = (1 << 9),
        eMimicJointRotYAPI = (1 << 10),
        eMimicJointRotZAPI = (1 << 11),
        eContactReportAPI = (1 << 12),
        eDrivePerformanceEnvelopeAngularAPI = (1 << 13),
        eDrivePerformanceEnvelopeLinearAPI = (1 << 14),
        eDrivePerformanceEnvelopeRotXAPI = (1 << 15),
        eDrivePerformanceEnvelopeRotYAPI = (1 << 16),
        eDrivePerformanceEnvelopeRotZAPI = (1 << 17),
        eJointAxisAngularAPI = (1 << 18),
        eJointAxisLinearAPI = (1 << 19),
        eJointAxisRotXAPI = (1 << 20),
        eJointAxisRotYAPI = (1 << 21),
        eJointAxisRotZAPI = (1 << 22),
        eDeformableBodyAPI = (1 << 23),
        eVolumeDeformableSimAPI = (1 << 24),
        eSurfaceDeformableSimAPI = (1 << 25),
        eDeformablePoseAPI = (1 << 26),
        eAutoDeformableBodyAPI = (1 << 27),
        eAutoDeformableHexahedralMeshAPI = (1 << 28),
        eAutoDeformableMeshSimplificationAPI = (1 << 29),
        eNewtonMimicAPI = (1 << 30),
        // eNextItem = (uint64_t(1) << 31),
    };
};

class ObjectDb
{
public:
    using Map = std::unordered_map<PXR_NS::SdfPath, ObjectIdMap, PXR_NS::SdfPath::Hash>;
    using SchemaApiMap = std::unordered_map<PXR_NS::SdfPath, uint64_t, PXR_NS::SdfPath::Hash>;
    using KeyMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                      ObjectIdMap,
                                      omni::physics::parse::ObjectKey::Hash>;
    using KeySchemaApiMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                               uint64_t,
                                               omni::physics::parse::ObjectKey::Hash>;
    using PathKeyMap = std::unordered_map<PXR_NS::SdfPath,
                                          omni::physics::parse::ObjectKey,
                                          PXR_NS::SdfPath::Hash>;
    using KeyResolver = std::function<omni::physics::parse::ObjectKey(const PXR_NS::SdfPath&)>;

    void setKeyResolver(KeyResolver resolver);

    /*
     * Create a new entry at the given path.
     */
    void findOrCreateEntry(const PXR_NS::SdfPath& path, ObjectCategory category, ObjectId newEntryId);
    void findOrCreateEntryWithoutHierarchyStorage(const PXR_NS::SdfPath& path, ObjectCategory category, ObjectId newEntryId);

    /*
     * Return the set of entries at the given path.  If the path has not had entries
     * created, returns nullptr.
     */
    const ObjectIdMap* getEntries(const PXR_NS::SdfPath& path) const;
    ObjectIdMap* getEntries(const PXR_NS::SdfPath& path);
    const ObjectIdMap* getEntries(omni::physics::parse::ObjectKey key) const;
    ObjectIdMap* getEntries(omni::physics::parse::ObjectKey key);

    /*
     * Utility function which returns first entry in the set at the given path if it exists
     */
    ObjectId findEntry(const PXR_NS::SdfPath& path, ObjectCategory category /* = eAllCategories */) const;
    ObjectId findEntry(omni::physics::parse::ObjectKey key, ObjectCategory category) const;

    bool empty() const
    {
        return mPathMap.empty();
    }

    /*
     * Clears all paths at or below the given path, moving all of the entries in the subtree
     * to the remove list.
     */
    bool removeEntries(const PXR_NS::SdfPath& path);

    void removeEntry(const PXR_NS::SdfPath& path, ObjectCategory category, ObjectId entryId);
    void removeEntry(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId entryId);

    void addSchemaAPI(const PXR_NS::SdfPath& path, SchemaAPIFlag::Enum schemaAPI);

    void setSchemaAPI(const PXR_NS::SdfPath& path, uint64_t flags);

    void removeSchemaAPIs(const PXR_NS::SdfPath& path);

    void removeSchemaAPI(const PXR_NS::SdfPath& path, SchemaAPIFlag::Enum schemaAPI);

    uint64_t getSchemaAPIs(const PXR_NS::SdfPath& path) const;
    uint64_t getSchemaAPIs(omni::physics::parse::ObjectKey key) const;

    const PrimHierarchyStorage& getPrimHierarchyStorage() const
    {
        return mPrimHierarchyStorage;
    }

    PrimHierarchyStorage& getPrimHierarchyStorage()
    {
        return mPrimHierarchyStorage;
    }

private:
    omni::physics::parse::ObjectKey resolveKey(const PXR_NS::SdfPath& path);
    void rebuildKeyMaps();

    Map mPathMap;
    SchemaApiMap mSchemaAPIMap;
    PathKeyMap mPathKeyMap;
    KeyMap mKeyMap;
    KeySchemaApiMap mKeySchemaAPIMap;
    KeyResolver mKeyResolver;
    PrimHierarchyStorage mPrimHierarchyStorage;
};

inline void ObjectDb::setKeyResolver(KeyResolver resolver)
{
    mKeyResolver = std::move(resolver);
    rebuildKeyMaps();
}

inline omni::physics::parse::ObjectKey ObjectDb::resolveKey(const PXR_NS::SdfPath& path)
{
    PathKeyMap::const_iterator cached = mPathKeyMap.find(path);
    if (cached != mPathKeyMap.end())
        return cached->second;

    if (!mKeyResolver)
        return {};

    const omni::physics::parse::ObjectKey key = mKeyResolver(path);
    if (key.valid())
        mPathKeyMap[path] = key;
    return key;
}

inline void ObjectDb::rebuildKeyMaps()
{
    mPathKeyMap.clear();
    mKeyMap.clear();
    mKeySchemaAPIMap.clear();

    if (!mKeyResolver)
        return;

    for (Map::const_iterator it = mPathMap.begin(); it != mPathMap.end(); ++it)
    {
        const omni::physics::parse::ObjectKey key = resolveKey(it->first);
        if (key.valid())
            mKeyMap[key] = it->second;
    }

    for (SchemaApiMap::const_iterator it = mSchemaAPIMap.begin(); it != mSchemaAPIMap.end(); ++it)
    {
        const omni::physics::parse::ObjectKey key = resolveKey(it->first);
        if (key.valid())
            mKeySchemaAPIMap[key] = it->second;
    }
}

inline void ObjectDb::addSchemaAPI(const PXR_NS::SdfPath& path, SchemaAPIFlag::Enum schemaAPI)
{
    mSchemaAPIMap[path] |= schemaAPI;
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    if (key.valid())
        mKeySchemaAPIMap[key] |= schemaAPI;
}

inline void ObjectDb::setSchemaAPI(const PXR_NS::SdfPath& path, uint64_t flags)
{
    mSchemaAPIMap[path] = flags;
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    if (key.valid())
        mKeySchemaAPIMap[key] = flags;
}

inline void ObjectDb::removeSchemaAPIs(const PXR_NS::SdfPath& path)
{
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    SchemaApiMap::iterator it = mSchemaAPIMap.find(path);
    if (it != mSchemaAPIMap.end())
        mSchemaAPIMap.erase(it);
    if (key.valid())
    {
        KeySchemaApiMap::iterator kit = mKeySchemaAPIMap.find(key);
        if (kit != mKeySchemaAPIMap.end())
            mKeySchemaAPIMap.erase(kit);
    }
}

inline void ObjectDb::removeSchemaAPI(const PXR_NS::SdfPath& path, SchemaAPIFlag::Enum schemaAPI)
{
    mSchemaAPIMap[path] &= ~schemaAPI;
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    if (key.valid())
        mKeySchemaAPIMap[key] &= ~schemaAPI;
}

inline uint64_t ObjectDb::getSchemaAPIs(const PXR_NS::SdfPath& path) const
{
    SchemaApiMap::const_iterator it = mSchemaAPIMap.find(path);
    if (it != mSchemaAPIMap.end())
        return it->second;

    return 0;
}

inline uint64_t ObjectDb::getSchemaAPIs(omni::physics::parse::ObjectKey key) const
{
    KeySchemaApiMap::const_iterator it = mKeySchemaAPIMap.find(key);
    if (it != mKeySchemaAPIMap.end())
        return it->second;

    return 0;
}

inline void ObjectDb::findOrCreateEntry(const PXR_NS::SdfPath& path, ObjectCategory category, ObjectId newEntryId)
{
    mPrimHierarchyStorage.addPrim(path);
    mPathMap[path].insert(std::make_pair(category, newEntryId));
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    if (key.valid())
        mKeyMap[key].insert(std::make_pair(category, newEntryId));
}

inline void ObjectDb::findOrCreateEntryWithoutHierarchyStorage(const PXR_NS::SdfPath& path,
                                                               ObjectCategory category,
                                                               ObjectId newEntryId)
{
    mPathMap[path].insert(std::make_pair(category, newEntryId));
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    if (key.valid())
        mKeyMap[key].insert(std::make_pair(category, newEntryId));
}


inline const ObjectIdMap* ObjectDb::getEntries(const PXR_NS::SdfPath& path) const
{
    Map::const_iterator it = mPathMap.find(path);
    if (it != mPathMap.end())
        return &it->second;
    else
        return nullptr;
}

inline ObjectIdMap* ObjectDb::getEntries(const PXR_NS::SdfPath& path)
{
    Map::iterator it = mPathMap.find(path);
    if (it != mPathMap.end())
        return &it->second;
    else
        return nullptr;
}

inline const ObjectIdMap* ObjectDb::getEntries(omni::physics::parse::ObjectKey key) const
{
    KeyMap::const_iterator it = mKeyMap.find(key);
    if (it != mKeyMap.end())
        return &it->second;
    else
        return nullptr;
}

inline ObjectIdMap* ObjectDb::getEntries(omni::physics::parse::ObjectKey key)
{
    KeyMap::iterator it = mKeyMap.find(key);
    if (it != mKeyMap.end())
        return &it->second;
    else
        return nullptr;
}

inline ObjectId ObjectDb::findEntry(const PXR_NS::SdfPath& path, ObjectCategory category) const
{
    Map::const_iterator it = mPathMap.find(path);
    if (it != mPathMap.end())
    {
        const ObjectIdMap& map = it->second;
        ObjectIdMap::const_iterator mapit = map.find(category);
        if (mapit != map.end())
            return mapit->second;
    }

    return kInvalidObjectId;
}

inline ObjectId ObjectDb::findEntry(omni::physics::parse::ObjectKey key, ObjectCategory category) const
{
    KeyMap::const_iterator it = mKeyMap.find(key);
    if (it != mKeyMap.end())
    {
        const ObjectIdMap& map = it->second;
        ObjectIdMap::const_iterator mapit = map.find(category);
        if (mapit != map.end())
            return mapit->second;
    }

    return kInvalidObjectId;
}

template <typename T>
void getAttributeArray(PXR_NS::VtArray<T>& array, PXR_NS::UsdAttribute& attribute)
{
    PXR_NS::VtValue arrayDataValue;
    attribute.Get(&arrayDataValue);
    const size_t size = arrayDataValue.GetArraySize();
    array.resize(size);
    if (size)
    {
        const PXR_NS::VtArray<T>& arrayData = arrayDataValue.Get<PXR_NS::VtArray<T>>();
        array.assign(arrayData.begin(), arrayData.end());
    }
}

template <typename T>
bool getAttributeArrayTimedFallback(PXR_NS::VtArray<T>& array,
                                    const PXR_NS::UsdAttribute& attribute,
                                    const PXR_NS::UsdTimeCode& timeCode)
{
    bool retVal = false;
    PXR_NS::VtValue arrayDataValue;
    if (attribute.Get(&arrayDataValue))
    {
        retVal = true;
    }
    else if (attribute.Get(&arrayDataValue, timeCode))
    {
        retVal = true;
    }

    if (retVal)
    {
        const size_t size = arrayDataValue.GetArraySize();
        array.resize(size);
        if (size)
        {
            const PXR_NS::VtArray<T>& arrayData = arrayDataValue.Get<PXR_NS::VtArray<T>>();
            array.assign(arrayData.begin(), arrayData.end());
        }
    }
    return retVal;
}

inline void GfVec3ToFloat3(const PXR_NS::GfVec3f& inVec, carb::Float3& outVec)
{
    outVec.x = inVec[0];
    outVec.y = inVec[1];
    outVec.z = inVec[2];
}

inline void GfVec3ToFloat3(const PXR_NS::GfVec3d& inVec, carb::Float3& outVec)
{
    outVec.x = float(inVec[0]);
    outVec.y = float(inVec[1]);
    outVec.z = float(inVec[2]);
}

inline void GfVec4ToFloat4(const PXR_NS::GfVec4d& inVec, carb::Float4& outVec)
{
    outVec.x = float(inVec[0]);
    outVec.y = float(inVec[1]);
    outVec.z = float(inVec[2]);
    outVec.w = float(inVec[3]);
}

inline void GfQuatToFloat4(const PXR_NS::GfQuatd& inRot, carb::Float4& outVec)
{
    const PXR_NS::GfVec3d im = inRot.GetImaginary();

    outVec.x = float(im[0]);
    outVec.y = float(im[1]);
    outVec.z = float(im[2]);
    outVec.w = float(inRot.GetReal());
}

inline void GfQuatToFloat4(const PXR_NS::GfQuatf& inRot, carb::Float4& outVec)
{
    const PXR_NS::GfVec3f im = inRot.GetImaginary();

    outVec.x = im[0];
    outVec.y = im[1];
    outVec.z = im[2];
    outVec.w = inRot.GetReal();
}

inline void Float4ToGfQuat(const carb::Float4& inVec, PXR_NS::GfQuatd& outRot)
{
    PXR_NS::GfVec3d im;

    im[0] = inVec.x;
    im[1] = inVec.y;
    im[2] = inVec.z;

    outRot.SetReal(inVec.w);
    outRot.SetImaginary(im);
}

inline void Float3ToGfVec3(const carb::Float3& inVec, PXR_NS::GfVec3f& outVec)
{
    outVec[0] = inVec.x;
    outVec[1] = inVec.y;
    outVec[2] = inVec.z;
}

inline void Float3ToGfVec3(const carb::Float3& inVec, PXR_NS::GfVec3d& outVec)
{
    outVec[0] = double(inVec.x);
    outVec[1] = double(inVec.y);
    outVec[2] = double(inVec.z);
}


inline bool isPowerOfTwo(uint32_t val)
{
    if (val == 0u)
        return false;

    return (ceil(log2(val)) == floor(log2(val)));
}

class MemoryAllocator
{
public:
    template <typename T>
    T* allocate(size_t count = 1)
    {
        size_t size = count * sizeof(T);
        T* ret = reinterpret_cast<T*>(malloc(size));
        return ret;
    }

    void deallocate(void* mem)
    {
        if (mem)
        {
            free(mem);
        }
    }
};

template <typename T>
bool SafeGetAttribute(T* out, PXR_NS::UsdAttribute const& attribute)
{
    if (attribute.HasValue())
    {
        attribute.Get(out);
        return true;
    }
    else
        return false;
}

template <>
inline bool SafeGetAttribute<carb::Float3>(carb::Float3* out, PXR_NS::UsdAttribute const& attribute)
{
    if (attribute.HasValue())
    {
        PXR_NS::GfVec3f v;
        attribute.Get(&v);
        out->x = v[0];
        out->y = v[1];
        out->z = v[2];

        return true;
    }
    else
        return false;
}

template <>
inline bool SafeGetAttribute<carb::Float2>(carb::Float2* out, PXR_NS::UsdAttribute const& attribute)
{
    if (attribute.HasValue())
    {
        PXR_NS::GfVec2f v;
        attribute.Get(&v);
        out->x = v[0];
        out->y = v[1];

        return true;
    }
    else
        return false;
}

inline MeshKey loadMeshKey(const PXR_NS::UsdPrim prim, const PXR_NS::TfToken crcToken)
{
    MeshKey meshKey;
    const size_t meshKeySize = sizeof(MeshKey);
    const PXR_NS::UsdAttribute crcAttr = prim.GetAttribute(crcToken);
    if (crcAttr.HasAuthoredValue())
    {
        PXR_NS::VtArray<PXR_NS::uchar> vtMeshKey(meshKeySize);
        crcAttr.Get(&vtMeshKey);

        if (vtMeshKey.size() == meshKeySize)
            std::memcpy(&meshKey, vtMeshKey.data(), meshKeySize);
    }
    return meshKey;
}

inline void storeMeshKey(PXR_NS::UsdPrim prim, const PXR_NS::TfToken crcToken, const MeshKey& meshKey)
{
    PXR_NS::UsdAttribute crcAttr = prim.GetAttribute(crcToken);
    if (!crcAttr.HasAuthoredValue())
    {
        crcAttr = prim.CreateAttribute(crcToken, PXR_NS::SdfValueTypeNames->UCharArray);
    }
    PXR_NS::VtArray<PXR_NS::uchar> vtData(sizeof(MeshKey));
    std::memcpy(vtData.data(), &meshKey, sizeof(MeshKey));
    crcAttr.Set(vtData);
}

// Resolves ref to a string path
std::string GetBody(PXR_NS::UsdRelationship const ref, const PXR_NS::UsdPrim& jointPrim);


bool ExtractTriangulatedFaces(std::vector<uint32_t>& triangles, PXR_NS::UsdGeomMesh const& usdMesh);

#define REPORT_PHYSICS_ERROR(fmt, ...)                                                                                 \
    char errorMsg[4096];                                                                                               \
    sprintf_s(errorMsg, 4096, fmt, ##__VA_ARGS__);                                                                     \
    std::string s(errorMsg);                                                                                           \
    omni::physx::PhysXUsdPhysicsInterface::reportLoadError(omni::physx::usdparser::ErrorCode::eError, s.c_str());

#define REPORT_PHYSICS_MESSAGE(errorCode, fmt, ...)                                                                    \
    char errorMsg[4096];                                                                                               \
    sprintf_s(errorMsg, 4096, fmt, ##__VA_ARGS__);                                                                     \
    std::string s(errorMsg);                                                                                           \
    omni::physx::PhysXUsdPhysicsInterface::reportLoadError(errorCode, s.c_str());

} // namespace usdparser
} // namespace physx
} // namespace omni
