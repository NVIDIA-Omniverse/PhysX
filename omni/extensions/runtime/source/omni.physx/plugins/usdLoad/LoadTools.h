// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>
#include <common/utilities/PrimHierarchyStorage.h>

#if !CARB_PLATFORM_WINDOWS
#    define sprintf_s snprintf
#endif

namespace omni
{
namespace physx
{
namespace usdparser
{

struct PhysxRigidBodyDesc;
struct PhysxJointDesc;
struct PhysxDeformableAttachmentDesc;
struct PhysxArticulationDesc;
struct PhysxMaterialDesc;

struct JointDescAndPath
{
    bool operator<(const JointDescAndPath& jd) const
    {
        return index < jd.index ? true : false;
    }

    pxr::SdfPath path;
    pxr::UsdPrim prim;
    PhysxJointDesc* desc;
    bool articulationJoint;
    uint32_t index;
};

struct DeformableAttachmentDescAndPath
{
    pxr::SdfPath path;
    pxr::UsdPrim prim;
    PhysxDeformableAttachmentDesc* desc;
};

struct DeformableCollisionFilterDescAndPath
{
    pxr::SdfPath path;
    pxr::UsdPrim prim;
    PhysxDeformableCollisionFilterDesc* desc;
};

struct BodyDescAndColliders
{
    omni::physx::usdparser::PhysxRigidBodyDesc* desc;
    omni::physics::schema::SdfPathSet collisions;
};

struct ShapeDescAndMaterials
{
    pxr::SdfPath path;
    omni::physx::usdparser::PhysxShapeDesc* desc;
    pxr::SdfPathVector materials;
};

struct DeformableDescAndMaterials
{
    pxr::SdfPath path;
    omni::physx::usdparser::PhysxDeformableBodyDesc* desc;
    pxr::SdfPath simMeshMaterial;
};

using ObjectIdMap = std::multimap<ObjectCategory, ObjectId>;
using JointVector = std::vector<JointDescAndPath>;
using JointPathIndexMap = std::unordered_map<pxr::SdfPath, size_t, pxr::SdfPath::Hash>;

using ShapePathList = std::vector<pxr::SdfPath>;
using PathSet = std::unordered_set<pxr::SdfPath, pxr::SdfPath::Hash>;
using BodyMap = std::map<pxr::SdfPath, BodyDescAndColliders>;
using BodyVector = std::vector<std::pair<pxr::SdfPath, BodyDescAndColliders>>;
using JointMap = std::map<pxr::SdfPath, omni::physx::usdparser::PhysxJointDesc*>;
using JointUnorderedMap = std::unordered_map<pxr::SdfPath, omni::physx::usdparser::PhysxJointDesc*, pxr::SdfPath::Hash>;
using ArticulationMap = std::map<pxr::SdfPath, std::vector<omni::physx::usdparser::PhysxArticulationDesc*>>;
using CollisionBlockPair = std::pair<pxr::SdfPath, pxr::SdfPath>;
using CollisionPairVector = std::vector<CollisionBlockPair>;
using CollisionGroupsMap = std::unordered_map<pxr::SdfPath, pxr::SdfPathVector, pxr::SdfPath::Hash>;
using ObjectIdUsdPrimMap = std::map<usdparser::ObjectId, pxr::UsdPrim>;
using MaterialsVector = std::vector<std::pair<pxr::SdfPath, usdparser::PhysxMaterialDesc*>>;
using DeformableMaterialsVector = std::vector<std::pair<pxr::SdfPath, usdparser::PhysxDeformableMaterialDesc*>>;
using ShapeDescsVector = std::vector<ShapeDescAndMaterials>;
using DeformableBodyDescsVector = std::vector<DeformableDescAndMaterials>;

using FixedTendonVector = std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonFixedDesc>>;
// using FixedTendonMap = pxr::TfHashMap<pxr::TfToken, omni::physx::usdparser::PhysxTendonFixedDesc* ,
// pxr::TfToken::HashFunctor>;
using TendonAxisMap =
    pxr::TfHashMap<pxr::SdfPath, std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonAxisDesc>>, pxr::SdfPath::Hash>;
using SpatialTendonVector = std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonSpatialDesc>>;
using TendonAttachmentMap =
    pxr::TfHashMap<pxr::SdfPath, std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonAttachmentDesc>>, pxr::SdfPath::Hash>;

using MimicJointVector = std::vector<omni::physx::usdparser::MimicJointDesc>;

using PathPhysXDescMap = std::unordered_map<pxr::SdfPath, const PhysxObjectDesc*, pxr::SdfPath::Hash>;

// DEPRECATED
using PhysxAttachmentVectorDeprecated = std::vector<pxr::SdfPath>;
using AttachmentHistoryMapDeprecated = std::unordered_multimap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;

using DeformableAttachmentVector = std::vector<DeformableAttachmentDescAndPath>;
using DeformableCollisionFilterVector = std::vector<DeformableCollisionFilterDescAndPath>;
using DeformableAttachmentHistoryMap = std::unordered_multimap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;
using DeformableCollisionFilterHistoryMap = std::unordered_multimap<pxr::SdfPath, pxr::SdfPath, pxr::SdfPath::Hash>;

struct SchemaAPIFlag
{
    enum Enum : uint64_t
    {
        eRigidBodyAPI = (1 << 0),
        eCollisionAPI = (1 << 1),
        eDeformableBodyAPIdeprecated = (1 << 2), // DEPRECATED
        eParticleIsosurfaceAPI = (1 << 3),
        eDiffuseParticlesAPI = (1 << 4),
        eParticleSetAPI = (1 << 5),
        eParticleClothAPIdeprecated = (1 << 6), // DEPRECATED
        eParticleAnisotropyAPI = (1 << 7),
        eParticleSmoothingAPI = (1 << 8),
        ePhysxForceAPI = (1 << 9),
        eFilteredPairsAPI = (1 << 10),
        eDeformableSurfaceAPIdeprecated = (1 << 11), // DEPRECATED
        eMimicJointRotXAPI = (1 << 12),
        eMimicJointRotYAPI = (1 << 13),
        eMimicJointRotZAPI = (1 << 14),
        eContactReportAPI = (1 << 15),
        eDrivePerformanceEnvelopeAngularAPI = (1 << 16),
        eDrivePerformanceEnvelopeLinearAPI = (1 << 17),
        eDrivePerformanceEnvelopeRotXAPI = (1 << 18),
        eDrivePerformanceEnvelopeRotYAPI = (1 << 19),
        eDrivePerformanceEnvelopeRotZAPI = (1 << 20),
        eJointAxisAngularAPI = (1 << 21),
        eJointAxisLinearAPI = (1 << 22),
        eJointAxisRotXAPI = (1 << 23),
        eJointAxisRotYAPI = (1 << 24),
        eJointAxisRotZAPI = (1 << 25),
        eDeformableBodyAPI = (1 << 26),
        eVolumeDeformableSimAPI = (1 << 27),
        eSurfaceDeformableSimAPI = (1 << 28),
        eDeformablePoseAPI = (1 << 29),
        eAutoDeformableBodyAPI = (1 << 30),
        eAutoDeformableHexahedralMeshAPI = (uint64_t(1) << 31),
        eAutoDeformableMeshSimplificationAPI = (uint64_t(1) << 32)
    };
};

class ObjectDb
{
public:
    using Map = std::unordered_map<pxr::SdfPath, ObjectIdMap, pxr::SdfPath::Hash>;
    using SchemaApiMap = std::unordered_map<pxr::SdfPath, uint64_t, pxr::SdfPath::Hash>;

    /*
     * Create a new entry at the given path.
     */
    void findOrCreateEntry(const pxr::SdfPath& path, ObjectCategory category, ObjectId newEntryId);
    void findOrCreateEntryWithoutHierarchyStorage(const pxr::SdfPath& path, ObjectCategory category, ObjectId newEntryId);

    /*
     * Return the set of entries at the given path.  If the path has not had entries
     * created, returns nullptr.
     */
    const ObjectIdMap* getEntries(const pxr::SdfPath& path) const;
    ObjectIdMap* getEntries(const pxr::SdfPath& path);

    /*
     * Utility function which returns first entry in the set at the given path if it exists
     */
    ObjectId findEntry(const pxr::SdfPath& path, ObjectCategory category /* = eAllCategories */) const;

    bool empty() const
    {
        return mPathMap.empty();
    }

    /*
     * Clears all paths at or below the given path, moving all of the entries in the subtree
     * to the remove list.
     */
    bool removeEntries(const pxr::SdfPath& path);

    void removeEntry(const pxr::SdfPath& path, ObjectCategory category, ObjectId entryId);

    void addSchemaAPI(const pxr::SdfPath& path, SchemaAPIFlag::Enum schemaAPI);

    void setSchemaAPI(const pxr::SdfPath& path, uint64_t flags);

    void removeSchemaAPIs(const pxr::SdfPath& path);

    void removeSchemaAPI(const pxr::SdfPath& path, SchemaAPIFlag::Enum schemaAPI);

    uint64_t getSchemaAPIs(const pxr::SdfPath& path) const;

    const PrimHierarchyStorage& getPrimHierarchyStorage() const
    {
        return mPrimHierarchyStorage;
    }

    PrimHierarchyStorage& getPrimHierarchyStorage()
    {
        return mPrimHierarchyStorage;
    }

private:
    Map mPathMap;
    SchemaApiMap mSchemaAPIMap;
    PrimHierarchyStorage mPrimHierarchyStorage;
};

inline void ObjectDb::addSchemaAPI(const pxr::SdfPath& path, SchemaAPIFlag::Enum schemaAPI)
{
    mSchemaAPIMap[path] |= schemaAPI;
}

inline void ObjectDb::setSchemaAPI(const pxr::SdfPath& path, uint64_t flags)
{
    mSchemaAPIMap[path] = flags;
}

inline void ObjectDb::removeSchemaAPIs(const pxr::SdfPath& path)
{
    SchemaApiMap::iterator it = mSchemaAPIMap.find(path);
    if (it != mSchemaAPIMap.end())
        mSchemaAPIMap.erase(it);
}

inline void ObjectDb::removeSchemaAPI(const pxr::SdfPath& path, SchemaAPIFlag::Enum schemaAPI)
{
    mSchemaAPIMap[path] &= ~schemaAPI;
}

inline uint64_t ObjectDb::getSchemaAPIs(const pxr::SdfPath& path) const
{
    SchemaApiMap::const_iterator it = mSchemaAPIMap.find(path);
    if (it != mSchemaAPIMap.end())
        return it->second;

    return 0;
}

inline void ObjectDb::findOrCreateEntry(const pxr::SdfPath& path, ObjectCategory category, ObjectId newEntryId)
{
    mPrimHierarchyStorage.addPrim(path);
    mPathMap[path].insert(std::make_pair(category, newEntryId));
}

inline void ObjectDb::findOrCreateEntryWithoutHierarchyStorage(const pxr::SdfPath& path,
                                                               ObjectCategory category,
                                                               ObjectId newEntryId)
{
    mPathMap[path].insert(std::make_pair(category, newEntryId));
}


inline const ObjectIdMap* ObjectDb::getEntries(const pxr::SdfPath& path) const
{
    Map::const_iterator it = mPathMap.find(path);
    if (it != mPathMap.end())
        return &it->second;
    else
        return nullptr;
}

inline ObjectIdMap* ObjectDb::getEntries(const pxr::SdfPath& path)
{
    Map::iterator it = mPathMap.find(path);
    if (it != mPathMap.end())
        return &it->second;
    else
        return nullptr;
}

inline ObjectId ObjectDb::findEntry(const pxr::SdfPath& path, ObjectCategory category) const
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

template <typename T>
void getAttributeArray(pxr::VtArray<T>& array, pxr::UsdAttribute& attribute)
{
    pxr::VtValue arrayDataValue;
    attribute.Get(&arrayDataValue);
    const size_t size = arrayDataValue.GetArraySize();
    array.resize(size);
    if (size)
    {
        const pxr::VtArray<T>& arrayData = arrayDataValue.Get<pxr::VtArray<T>>();
        array.assign(arrayData.begin(), arrayData.end());
    }
}

template <typename T>
bool getAttributeArrayTimedFallback(pxr::VtArray<T>& array,
                                    const pxr::UsdAttribute& attribute,
                                    const pxr::UsdTimeCode& timeCode)
{
    bool retVal = false;
    pxr::VtValue arrayDataValue;
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
            const pxr::VtArray<T>& arrayData = arrayDataValue.Get<pxr::VtArray<T>>();
            array.assign(arrayData.begin(), arrayData.end());
        }
    }
    return retVal;
}

inline void GfVec3ToFloat3(const pxr::GfVec3f& inVec, carb::Float3& outVec)
{
    outVec.x = inVec[0];
    outVec.y = inVec[1];
    outVec.z = inVec[2];
}

inline void GfVec3ToFloat3(const pxr::GfVec3d& inVec, carb::Float3& outVec)
{
    outVec.x = float(inVec[0]);
    outVec.y = float(inVec[1]);
    outVec.z = float(inVec[2]);
}

inline void GfVec4ToFloat4(const pxr::GfVec4d& inVec, carb::Float4& outVec)
{
    outVec.x = float(inVec[0]);
    outVec.y = float(inVec[1]);
    outVec.z = float(inVec[2]);
    outVec.w = float(inVec[3]);
}

inline void GfQuatToFloat4(const pxr::GfQuatd& inRot, carb::Float4& outVec)
{
    const pxr::GfVec3d im = inRot.GetImaginary();

    outVec.x = float(im[0]);
    outVec.y = float(im[1]);
    outVec.z = float(im[2]);
    outVec.w = float(inRot.GetReal());
}

inline void GfQuatToFloat4(const pxr::GfQuatf& inRot, carb::Float4& outVec)
{
    const pxr::GfVec3f im = inRot.GetImaginary();

    outVec.x = im[0];
    outVec.y = im[1];
    outVec.z = im[2];
    outVec.w = inRot.GetReal();
}

inline void Float4ToGfQuat(const carb::Float4& inVec, pxr::GfQuatd& outRot)
{
    pxr::GfVec3d im;

    im[0] = inVec.x;
    im[1] = inVec.y;
    im[2] = inVec.z;

    outRot.SetReal(inVec.w);
    outRot.SetImaginary(im);
}

inline void Float3ToGfVec3(const carb::Float3& inVec, pxr::GfVec3f& outVec)
{
    outVec[0] = inVec.x;
    outVec[1] = inVec.y;
    outVec[2] = inVec.z;
}

inline void Float3ToGfVec3(const carb::Float3& inVec, pxr::GfVec3d& outVec)
{
    outVec[0] = double(inVec.x);
    outVec[1] = double(inVec.y);
    outVec[2] = double(inVec.z);
}

inline void decomposeTransform(const pxr::SdfPath geom,
                               const pxr::UsdStageRefPtr stage,
                               carb::Float3& position,
                               carb::Float4& orientation,
                               carb::Float3& scale)
{
    pxr::GfMatrix4d mat;
    bool resetXformStack = false;
    pxr::UsdGeomXformable xform(stage->GetPrimAtPath(geom));

    pxr::GfVec3d pos(0);
    pxr::GfQuatd rot = pxr::GfQuatd::GetIdentity();
    pxr::GfVec3d sc(1);

    if (xform.GetLocalTransformation(&mat, &resetXformStack))
    {
        const pxr::GfTransform tr(mat);
        pos = tr.GetTranslation();
        rot = tr.GetRotation().GetQuat();
        sc = tr.GetScale();
    }

    position = { float(pos[0]), float(pos[1]), float(pos[2]) };
    orientation = { float(rot.GetImaginary()[0]), float(rot.GetImaginary()[1]), float(rot.GetImaginary()[2]),
                    float(rot.GetReal()) };
    scale = { float(sc[0]), float(sc[1]), float(sc[2]) };
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
bool SafeGetAttribute(T* out, pxr::UsdAttribute const& attribute)
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
inline bool SafeGetAttribute<carb::Float3>(carb::Float3* out, pxr::UsdAttribute const& attribute)
{
    if (attribute.HasValue())
    {
        pxr::GfVec3f v;
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
inline bool SafeGetAttribute<carb::Float2>(carb::Float2* out, pxr::UsdAttribute const& attribute)
{
    if (attribute.HasValue())
    {
        pxr::GfVec2f v;
        attribute.Get(&v);
        out->x = v[0];
        out->y = v[1];

        return true;
    }
    else
        return false;
}

inline MeshKey loadMeshKey(const pxr::UsdPrim prim, const pxr::TfToken crcToken)
{
    MeshKey meshKey;
    const size_t meshKeySize = sizeof(MeshKey);
    const pxr::UsdAttribute crcAttr = prim.GetAttribute(crcToken);
    if (crcAttr.HasAuthoredValue())
    {
        pxr::VtArray<pxr::uchar> vtMeshKey(meshKeySize);
        crcAttr.Get(&vtMeshKey);

        if (vtMeshKey.size() == meshKeySize)
            std::memcpy(&meshKey, vtMeshKey.data(), meshKeySize);
    }
    return meshKey;
}

inline void storeMeshKey(pxr::UsdPrim prim, const pxr::TfToken crcToken, const MeshKey& meshKey)
{
    pxr::UsdAttribute crcAttr = prim.GetAttribute(crcToken);
    if (!crcAttr.HasAuthoredValue())
    {
        crcAttr = prim.CreateAttribute(crcToken, pxr::SdfValueTypeNames->UCharArray);
    }
    pxr::VtArray<pxr::uchar> vtData(sizeof(MeshKey));
    std::memcpy(vtData.data(), &meshKey, sizeof(MeshKey));
    crcAttr.Set(vtData);
}

// Resolves ref to a string path
std::string GetBody(pxr::UsdRelationship const ref, const pxr::UsdPrim& jointPrim);

// Resolves ref to a prim in stage which has to be an xformable. transform is extracted and returned in position and
// orientation
void GetLocalFrame(carb::Float3* position,
                   carb::Float4* orientation,
                   const pxr::UsdStageRefPtr stage,
                   pxr::UsdRelationship const ref,
                   const pxr::UsdPrim& jointPrim);

bool ExtractTriangulatedFaces(std::vector<uint32_t>& triangles, pxr::UsdGeomMesh const& usdMesh);

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
