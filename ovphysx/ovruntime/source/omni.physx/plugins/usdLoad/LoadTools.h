// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PUBLICAPI-001
 * @covers AC-27
 *
 * @implements REQ-LOAD-OBJECTDB-001
 * @covers AC-1 AC-2 AC-3 AC-4
 */
#pragma once

#include <private/omni/physx/PhysxUsd.h>
#include <omni/physics/parse/Handles.h>

#include <common/utilities/PrimHierarchyStorage.h>

#include <functional>
#include <string_view>
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
                                     carb::Float3& localPosOut,
                                     carb::Float4& localRotOut,
                                     carb::Float3& localScaleOut);

// PhysxRigidBodyDesc / PhysxMaterialDesc / PhysxArticulationDesc /
// PhysxJointDesc / PhysxDeformableAttachmentDesc / PhysxDeformableCollisionFilterDesc
// are aliases to the parse-library types (defined in PhysxUsd.h via `using`,
// already fully defined by the `#include <private/omni/physx/PhysxUsd.h>`
// above) -- a forward struct-declaration would conflict with the alias.

struct JointDescAndPath
{
    bool operator<(const JointDescAndPath& jd) const
    {
        return index < jd.index ? true : false;
    }

    omni::physics::parse::ObjectKey path;
    PhysxJointDesc* desc;
    bool articulationJoint;
    uint32_t index;
};

struct DeformableAttachmentDescAndPath
{
    omni::physics::parse::ObjectKey path;
    PhysxDeformableAttachmentDesc* desc;
};

struct DeformableCollisionFilterDescAndPath
{
    omni::physics::parse::ObjectKey path;
    PhysxDeformableCollisionFilterDesc* desc;
};

// Set of ObjectKeys (unordered: ObjectKey has no operator<). Same shape as
// omni.physics.parse/ArticulationGraph.cpp's KeySet.
using KeySet = std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;

struct BodyDescAndColliders
{
    omni::physx::usdparser::PhysxRigidBodyDesc* desc;
    KeySet collisions;
};

struct ShapeDescAndMaterials
{
    omni::physics::parse::ObjectKey path;
    omni::physx::usdparser::PhysxShapeDesc* desc;
    std::vector<omni::physics::parse::ObjectKey> materials;
};

struct DeformableDescAndMaterials
{
    omni::physics::parse::ObjectKey path;
    omni::physx::usdparser::PhysxDeformableBodyDesc* desc;
    omni::physics::parse::ObjectKey simMeshMaterial;
};

using ObjectIdMap = std::multimap<ObjectCategory, ObjectId>;
using JointVector = std::vector<JointDescAndPath>;
using JointPathIndexMap = std::unordered_map<omni::physics::parse::ObjectKey, size_t, omni::physics::parse::ObjectKey::Hash>;

// The `excludePaths` boundary type for loadFromStage()/PhysxUsdPhysicsListener. An exclude
// set is just a KeySet used for membership tests.
using PathSet = KeySet;
using BodyMap = std::unordered_map<omni::physics::parse::ObjectKey, BodyDescAndColliders, omni::physics::parse::ObjectKey::Hash>;
using BodyVector = std::vector<std::pair<omni::physics::parse::ObjectKey, BodyDescAndColliders>>;
using JointMap = std::unordered_map<omni::physics::parse::ObjectKey, omni::physx::usdparser::PhysxJointDesc*, omni::physics::parse::ObjectKey::Hash>;
using JointUnorderedMap =
    std::unordered_map<omni::physics::parse::ObjectKey, omni::physx::usdparser::PhysxJointDesc*, omni::physics::parse::ObjectKey::Hash>;
using ArticulationMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                           std::vector<omni::physx::usdparser::PhysxArticulationDesc*>,
                                           omni::physics::parse::ObjectKey::Hash>;
using CollisionBlockPair = std::pair<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey>;
using CollisionPairVector = std::vector<CollisionBlockPair>;
using CollisionGroupsMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                              std::vector<omni::physics::parse::ObjectKey>,
                                              omni::physics::parse::ObjectKey::Hash>;
// Backend-agnostic shape map for the mass path (ADR-0002 M2c-D): the shape's
// source-agnostic ObjectKey keyed by ObjectId, so mass works without live USD
// prims (ovstage) or an SdfPath round trip. Was SdfPath-valued; retyped to
// ObjectKey since every consumer (Mass.cpp) only ever needed the key.
using ObjectIdPathMap = std::map<usdparser::ObjectId, omni::physics::parse::ObjectKey>;
using MaterialsVector = std::vector<std::pair<omni::physics::parse::ObjectKey, usdparser::PhysxMaterialDesc*>>;
using DeformableMaterialsVector =
    std::vector<std::pair<omni::physics::parse::ObjectKey, usdparser::PhysxDeformableMaterialDesc*>>;
using ShapeDescsVector = std::vector<ShapeDescAndMaterials>;
using DeformableBodyDescsVector = std::vector<DeformableDescAndMaterials>;

using FixedTendonVector = std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonFixedDesc>>;
// using FixedTendonMap = PXR_NS::TfHashMap<PXR_NS::TfToken, omni::physx::usdparser::PhysxTendonFixedDesc* ,
// PXR_NS::TfToken::HashFunctor>;
// Was PXR_NS::TfHashMap<SdfPath, ...>: retyped to std::unordered_map, both to
// key by ObjectKey and to drop the pxr container (TfHashMap is itself a pxr
// type, unlike std::unordered_map which merely used to be keyed by a pxr type).
using TendonAxisMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                         std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonAxisDesc>>,
                                         omni::physics::parse::ObjectKey::Hash>;
using SpatialTendonVector = std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonSpatialDesc>>;
using TendonAttachmentMap =
    std::unordered_map<omni::physics::parse::ObjectKey,
                       std::vector<std::shared_ptr<omni::physx::usdparser::PhysxTendonAttachmentDesc>>,
                       omni::physics::parse::ObjectKey::Hash>;

using MimicJointVector = std::vector<omni::physx::usdparser::MimicJointDesc>;

using PathPhysXDescMap =
    std::unordered_map<omni::physics::parse::ObjectKey, const PhysxObjectDesc*, omni::physics::parse::ObjectKey::Hash>;

using DeformableAttachmentVector = std::vector<DeformableAttachmentDescAndPath>;
using DeformableCollisionFilterVector = std::vector<DeformableCollisionFilterDescAndPath>;
using DeformableAttachmentHistoryMap =
    std::unordered_multimap<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;
using DeformableCollisionFilterHistoryMap =
    std::unordered_multimap<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>;

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

// ObjectDb is keyed entirely by ObjectKey. The one path-keyed index is the plain-string
// mPrimHierarchyStorage, written by findOrCreateEntry(ObjectKey, pathText, ...) and read by
// the tensor wildcard matcher (BaseSimulationView.cpp), PhysX.cpp's clone-target guard and
// PrimUpdate.cpp's subtree walks.
class ObjectDb
{
public:
    using KeyMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                      ObjectIdMap,
                                      omni::physics::parse::ObjectKey::Hash>;
    using KeySchemaApiMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                               uint64_t,
                                               omni::physics::parse::ObjectKey::Hash>;
    // ObjectKey -> parent ObjectKey (the ADR-0019 decision-2 primitive,
    // IPhysicsSource::canonicalKey(getParent(...))). Powers the ObjectKey-typed
    // removeEntries below; unset on a bare-default-constructed ObjectDb (before
    // setParentResolver runs), in which case that overload safely no-ops.
    using ParentResolver = std::function<omni::physics::parse::ObjectKey(omni::physics::parse::ObjectKey)>;
    // ObjectKey -> the path text the object was registered under. Written only by
    // findOrCreateEntry(ObjectKey, pathText, ...) -- the one creation overload that also
    // writes the path-keyed mPrimHierarchyStorage -- so every removal path can evict the
    // hierarchy row that creation added by reverse lookup, with no path argument from the
    // caller. See REQ-LOAD-OBJECTDB-001: keeping the eviction inside ObjectDb is what makes
    // the create/remove symmetry a property of the container rather than of each call site.
    using KeyPathMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                          std::string,
                                          omni::physics::parse::ObjectKey::Hash>;

    void setParentResolver(ParentResolver resolver);

    /*
     * Create a new entry at the given path.
     */
    // ObjectKey-only sibling for a caller with no SdfPath (e.g. an ovstage
    // walk). Creation via a bare key has nothing to feed
    // PrimHierarchyStorage::addPrim, so this only ever touches the Key-side
    // maps -- same shape as the shipped getEntries(ObjectKey)/
    // findEntry(ObjectKey)/removeEntry(ObjectKey) overloads.
    void findOrCreateEntry(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId newEntryId);
    // Unlike the bare-key overload above, this one DOES feed PrimHierarchyStorage::addPrim,
    // giving hierarchy registration and cascade-delete. The registered path is remembered in
    // mKeyPathText so removeEntry/removeEntries can undo the row (REQ-LOAD-OBJECTDB-001).
    void findOrCreateEntry(omni::physics::parse::ObjectKey key, std::string_view pathText, ObjectCategory category, ObjectId newEntryId);
    void findOrCreateEntryWithoutHierarchyStorage(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId newEntryId);

    /*
     * Return the set of entries at the given path.  If the path has not had entries
     * created, returns nullptr.
     */
    const ObjectIdMap* getEntries(omni::physics::parse::ObjectKey key) const;
    ObjectIdMap* getEntries(omni::physics::parse::ObjectKey key);

    /*
     * Utility function which returns first entry in the set at the given path if it exists
     */
    ObjectId findEntry(omni::physics::parse::ObjectKey key, ObjectCategory category) const;

    bool empty() const
    {
        // mKeyMap is the authoritative "what still exists" index: every
        // findOrCreateEntry*/registerObjectId path writes it.
        return mKeyMap.empty();
    }

    /*
     * Clears all paths at or below the given path, moving all of the entries in the subtree
     * to the remove list.
     *
     * Walks mKeyMap directly via isAncestorOrSelf (ADR-0019 decision 2) rather than
     * PrimHierarchyStorage. Requires setParentResolver to have been called; a no-op
     * (returns false) otherwise.
     *
     * Evicts each cleared key's own mPrimHierarchyStorage row too, same reverse lookup as
     * removeEntry below (REQ-LOAD-OBJECTDB-001).
     */
    bool removeEntries(omni::physics::parse::ObjectKey key);

    // Removes one (category, entryId) at `key`, and -- once that was the key's last entry --
    // the path-keyed mPrimHierarchyStorage row creation added for it, found by reverse lookup
    // through mKeyPathText. REQ-LOAD-OBJECTDB-001.
    void removeEntry(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId entryId);

    // Write side of the schema-API bits, mirroring getSchemaAPIs(ObjectKey) below.
    void addSchemaAPI(omni::physics::parse::ObjectKey key, SchemaAPIFlag::Enum schemaAPI)
    {
        mKeySchemaAPIMap[key] |= schemaAPI;
    }

    void setSchemaAPI(omni::physics::parse::ObjectKey key, uint64_t flags)
    {
        mKeySchemaAPIMap[key] = flags;
    }

    void removeSchemaAPIs(omni::physics::parse::ObjectKey key)
    {
        mKeySchemaAPIMap.erase(key);
    }

    void removeSchemaAPI(omni::physics::parse::ObjectKey key, SchemaAPIFlag::Enum schemaAPI)
    {
        KeySchemaApiMap::iterator it = mKeySchemaAPIMap.find(key);
        if (it != mKeySchemaAPIMap.end())
            it->second &= ~schemaAPI;
    }

    uint64_t getSchemaAPIs(omni::physics::parse::ObjectKey key) const;


    /*
     * The source-agnostic index of what was actually created.
     */
    const KeyMap& getKeyMap() const
    {
        return mKeyMap;
    }

    // PrimHierarchyStorage is plain std::string-keyed and pxr-free, so this
    // accessor (and the member it returns) is available in both builds. Only
    // findOrCreateEntry(ObjectKey, pathText, ...) actually feeds it (unconditionally,
    // via pathText); the bare ObjectKey overload below has no path to feed it with.
    // Other consumers (e.g. PhysXReplicator.cpp's subtree enumeration / clone
    // hierarchy merge) use this accessor directly with plain strings under either
    // build.
    const PrimHierarchyStorage& getPrimHierarchyStorage() const
    {
        return mPrimHierarchyStorage;
    }

    PrimHierarchyStorage& getPrimHierarchyStorage()
    {
        return mPrimHierarchyStorage;
    }

private:
    // `ancestor`-rooted ancestry test for removeEntries(ObjectKey): walks
    // mParentResolver rather than converting to SdfPath and doing HasPrefix
    // (ADR-0019 decision 2). Mirrors AttachedStage::isAncestorOrSelf. False
    // when mParentResolver is unset.
    bool isAncestorOrSelf(omni::physics::parse::ObjectKey ancestor, omni::physics::parse::ObjectKey node) const;

    // Drops the mPrimHierarchyStorage row findOrCreateEntry(ObjectKey, pathText, ...)
    // registered for `key`. Call only once the key holds no entries at all.
    void dropHierarchyRow(omni::physics::parse::ObjectKey key);

    // No conditionally-present members: OvruntimeUnitTests includes this header directly, so
    // the layout must be identical in every TU.
    KeyMap mKeyMap;
    KeySchemaApiMap mKeySchemaAPIMap;
    ParentResolver mParentResolver;
    PrimHierarchyStorage mPrimHierarchyStorage;
    KeyPathMap mKeyPathText;
};

inline void ObjectDb::setParentResolver(ParentResolver resolver)
{
    mParentResolver = std::move(resolver);
}

inline bool ObjectDb::isAncestorOrSelf(omni::physics::parse::ObjectKey ancestor, omni::physics::parse::ObjectKey node) const
{
    if (ancestor == node)
        return true;
    if (!mParentResolver)
        return false;
    for (omni::physics::parse::ObjectKey k = mParentResolver(node); k.valid(); k = mParentResolver(k))
    {
        if (k == ancestor)
            return true;
    }
    return false;
}

inline uint64_t ObjectDb::getSchemaAPIs(omni::physics::parse::ObjectKey key) const
{
    KeySchemaApiMap::const_iterator it = mKeySchemaAPIMap.find(key);
    if (it != mKeySchemaAPIMap.end())
        return it->second;

    return 0;
}

inline void ObjectDb::findOrCreateEntry(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId newEntryId)
{
    // No path is available, so there is nothing to feed
    // PrimHierarchyStorage::addPrim -- same as the WithoutHierarchyStorage
    // sibling below (they are identical for a bare-key caller).
    findOrCreateEntryWithoutHierarchyStorage(key, category, newEntryId);
}

inline void ObjectDb::findOrCreateEntry(omni::physics::parse::ObjectKey key, std::string_view pathText,
                                        ObjectCategory category, ObjectId newEntryId)
{
    mPrimHierarchyStorage.addPrim(std::string(pathText));
    mKeyPathText[key].assign(pathText.data(), pathText.size());
    findOrCreateEntryWithoutHierarchyStorage(key, category, newEntryId);
}

inline void ObjectDb::findOrCreateEntryWithoutHierarchyStorage(omni::physics::parse::ObjectKey key,
                                                                ObjectCategory category,
                                                                ObjectId newEntryId)
{
    mKeyMap[key].insert(std::make_pair(category, newEntryId));
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
