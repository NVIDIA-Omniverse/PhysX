// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/** @implements REQ-CACHE-ATTACH-001
 * @covers AC-9
 */
/** @implements REQ-CACHE-NATIVE-001
 * @covers AC-4 AC-1 AC-2 AC-3 AC-5
 * @implements REQ-CACHE-STORE-001
 * @covers AC-7
 */
#pragma once
#include <omni/physics/parse/CacheArchive.h>
#include <PxPhysicsAPI.h>
#include <extensions/PxCollectionExt.h>
#include <common/PxSerializer.h>
#include <extensions/PxGearJoint.h>
#include <extensions/PxRackAndPinionJoint.h>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <new>
#include <functional>
#include <unordered_set>

namespace omni::physx::cache
{
struct ReleaseCollection
{
    void operator()(::physx::PxCollection* p) const
    {
        if (p)
            p->release();
    }
};
using CollectionPtr = std::unique_ptr<::physx::PxCollection, ReleaseCollection>;

// The built-in binary registry does not serialize callback implementations or
// GPU-only object families. Keep the self-contained closure explicit. A
// constraint passes here by type; constraintsOwnedByJoints decides the rest.
inline bool supportsNativeArchive(::physx::PxBase& object)
{
    using namespace ::physx;
    const PxType type = object.getConcreteType();
    switch (type)
    {
    case PxConcreteType::eRIGID_STATIC:
    case PxConcreteType::eRIGID_DYNAMIC:
    case PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
    case PxConcreteType::eARTICULATION_LINK:
    case PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
    case PxConcreteType::eARTICULATION_MIMIC_JOINT:
    case PxConcreteType::eARTICULATION_SPATIAL_TENDON:
    case PxConcreteType::eARTICULATION_ATTACHMENT:
    case PxConcreteType::eARTICULATION_FIXED_TENDON:
    case PxConcreteType::eARTICULATION_TENDON_JOINT:
    case PxConcreteType::ePRUNING_STRUCTURE:
    case PxConcreteType::eHEIGHTFIELD:
    case PxConcreteType::eTRIANGLE_MESH_BVH33:
    case PxConcreteType::eAGGREGATE:
    case PxConcreteType::eMATERIAL:
    case PxConcreteType::eCONVEX_MESH:
    case PxConcreteType::eTRIANGLE_MESH_BVH34:
        break;
    case PxConcreteType::eSHAPE: {
        const PxGeometryType::Enum geometry = static_cast<const PxShape&>(object).getGeometry().getType();
        if (geometry != PxGeometryType::eSPHERE && geometry != PxGeometryType::ePLANE &&
            geometry != PxGeometryType::eCAPSULE && geometry != PxGeometryType::eBOX &&
            geometry != PxGeometryType::eCONVEXMESH && geometry != PxGeometryType::eTRIANGLEMESH &&
            geometry != PxGeometryType::eHEIGHTFIELD && geometry != PxGeometryType::eCONVEXCORE)
            return false;
        break;
    }
    case PxConcreteType::eCONSTRAINT:
        break;
    default:
        if (!object.is<PxFixedJoint>() && !object.is<PxRevoluteJoint>() && !object.is<PxPrismaticJoint>() &&
            !object.is<PxSphericalJoint>() && !object.is<PxDistanceJoint>() && !object.is<PxD6Joint>() &&
            !object.is<PxGearJoint>() && !object.is<PxRackAndPinionJoint>())
            return false;
    }
    return true;
}

// The extension joint object is, by concrete type rather than a name lookup.
inline ::physx::PxJoint* asJoint(::physx::PxBase& object)
{
    const ::physx::PxType type = object.getConcreteType();
    return type >= ::physx::PxJointConcreteType::eSPHERICAL && type < ::physx::PxJointConcreteType::eLast ?
               static_cast<::physx::PxJoint*>(&object) :
               nullptr;
}

// Whether a joint in collection owns every constraint in it. One that no joint
// owns would take its callbacks from the bytes, and a decoded one has no
// connector to ask until its joint resolves it. So only a constraint reached
// through a joint is asked for its owner, which must be that joint: the joints
// then own distinct constraints, and the counts match only when each has one.
inline bool constraintsOwnedByJoints(const ::physx::PxCollection& collection)
{
    using namespace ::physx;
    size_t constraints = 0, owned = 0;
    for (PxU32 i = 0; i < collection.getNbObjects(); ++i)
    {
        PxBase& object = collection.getObject(i);
        if (object.getConcreteType() == PxConcreteType::eCONSTRAINT)
            ++constraints;
        else if (PxJoint* joint = asJoint(object))
        {
            PxConstraint* constraint = joint->getConstraint();
            PxU32 type = 0;
            if (!constraint || !collection.contains(*constraint) || constraint->getExternalReference(type) != joint ||
                type != PxConstraintExtIDs::eJOINT)
                return false;
            ++owned;
        }
    }
    return owned == constraints;
}

// The cooked stream a triangle mesh was created from, or null for a mesh that was
// not created from one.
using CookedTriangleMesh =
    std::function<std::shared_ptr<const omni::physics::parse::cache::Bytes>(const ::physx::PxTriangleMesh&)>;

// A native section holds the cooked stream of each triangle mesh its collection
// uses, then the collection as PhysX serializes it to binary. PhysX binary
// serialization does not carry a whole triangle mesh: Gu::TriangleMesh exports
// neither of the vertex-to-triangle tables that cooking with GPU data adds, so a
// deserialized mesh keeps the addresses those tables had in the process that
// serialized it, and the GPU narrowphase copies from them when it uploads the
// mesh. The collection therefore refers to its triangle meshes as external
// objects, and decoding creates each one from its cooked stream, as ordinary
// loading does.
//
// Layout: a uint64 count n; n pairs of uint64 (serial ID, stream size); the n
// streams; zero padding to a multiple of PX_SERIAL_FILE_ALIGN; the collection.
//
// The caller must hold SDK-wide serialization quiescence: PhysX does not
// support serializing while any scene is simulating. This low-level codec does
// not acquire runtime admission or publish objects into an attachment.
// Fails, without buffering more, once the output exceeds limit (the store's cap),
// and for a triangle mesh without a cooked stream.
inline bool encodeCollection(const ::physx::PxCollection& roots,
                             ::physx::PxSerializationRegistry& registry,
                             const CookedTriangleMesh& cookedTriangleMesh,
                             omni::physics::parse::cache::Bytes& output,
                             bool requireRootActors = false,
                             size_t limit = size_t(512) * 1024 * 1024)
{
    using namespace ::physx;
    CollectionPtr collection(PxCreateCollection());
    CollectionPtr meshes(PxCreateCollection()); // the external triangle meshes
    if (!collection || !meshes)
        return false;
    PxSerialObjectId nextId = 1;
    for (PxU32 i = 0; i < roots.getNbObjects(); ++i)
    {
        PxBase& object = roots.getObject(i);
        const PxSerialObjectId id = roots.getId(object);
        if (id == std::numeric_limits<PxSerialObjectId>::max())
            return false;
        if (id >= nextId)
            nextId = id + 1;
        (object.is<PxTriangleMesh>() ? meshes : collection)->add(object, id);
    }
    // Complete a self-contained closure with the registered serializers, checking
    // each dependency before visiting it. PxSerialization::complete assumes every
    // dependency has a serializer and asserts on unsupported families.
    struct Dependencies : PxProcessPxBaseCallback
    {
        PxCollection& collection;
        PxCollection& meshes;
        std::vector<PxBase*> queue;
        Dependencies(PxCollection& value, PxCollection& triangleMeshes) : collection(value), meshes(triangleMeshes)
        {
        }
        void process(PxBase& value) override
        {
            // A triangle mesh depends on nothing.
            if (value.is<PxTriangleMesh>())
            {
                if (!meshes.contains(value))
                    meshes.add(value);
            }
            else if (!collection.contains(value))
            {
                collection.add(value);
                queue.push_back(&value);
            }
        }
    } dependencies(*collection, *meshes);
    for (PxU32 i = 0; i < collection->getNbObjects(); ++i)
        dependencies.queue.push_back(&collection->getObject(i));
    for (size_t i = 0; i < dependencies.queue.size(); ++i)
    {
        PxBase& object = *dependencies.queue[i];
        // Insertion may share resource dependencies, but must never capture an
        // actor, articulation or joint belonging to the existing scene. Check the
        // actual dependency here, before encoding, rather than decoding a copy.
        if (requireRootActors && !roots.contains(object) &&
            (object.is<PxRigidActor>() || object.is<PxJoint>() || object.is<PxArticulationReducedCoordinate>()))
            return false;
        if (!supportsNativeArchive(object))
            return false;
        const PxSerializer* serializer = registry.getSerializer(object.getConcreteType());
        if (!serializer)
            return false;
        serializer->requiresObjects(object, dependencies);
        // Extension joint serializers restore these references, but their
        // inherited requiresObjects visits only actors and the constraint.
        const PxBase* reference0 = nullptr;
        const PxBase* reference1 = nullptr;
        if (auto* gear = object.is<PxGearJoint>())
            gear->getHinges(reference0, reference1);
        if (auto* rack = object.is<PxRackAndPinionJoint>())
            rack->getJoints(reference0, reference1);
        if (reference0)
            dependencies.process(*const_cast<PxBase*>(reference0));
        if (reference1)
            dependencies.process(*const_cast<PxBase*>(reference1));
    }
    if (!constraintsOwnedByJoints(*collection))
        return false;
    // Every object here passed supportsNativeArchive in the closure walk above.
    // The external meshes take IDs from the same sequence.
    for (PxCollection* objects : { collection.get(), meshes.get() })
        for (PxU32 i = 0; i < objects->getNbObjects(); ++i)
        {
            PxBase& object = objects->getObject(i);
            if (objects->getId(object) == PX_SERIAL_OBJECT_ID_INVALID)
            {
                if (nextId == std::numeric_limits<PxSerialObjectId>::max())
                    return false;
                objects->addId(object, nextId++);
            }
        }
    std::vector<std::shared_ptr<const omni::physics::parse::cache::Bytes>> cooked;
    cooked.reserve(meshes->getNbObjects());
    for (PxU32 i = 0; i < meshes->getNbObjects(); ++i)
    {
        cooked.push_back(cookedTriangleMesh ? cookedTriangleMesh(static_cast<PxTriangleMesh&>(meshes->getObject(i))) :
                                              nullptr);
        if (!cooked.back() || cooked.back()->empty())
            return false;
    }
    // serializeCollectionToBinary performs isSerializable itself.
    struct Output : PxOutputStream
    {
        omni::physics::parse::cache::Bytes bytes;
        size_t limit;
        bool overflow = false;
        uint64_t write(const void* source, uint64_t count) override
        {
            if (overflow || count > limit - bytes.size())
                overflow = true;
            else if (count)
            {
                const auto* data = static_cast<const uint8_t*>(source);
                bytes.insert(bytes.end(), data, data + count);
            }
            return count;
        }
    } stream;
    stream.limit = limit;
    stream.bytes.reserve(std::min<size_t>(64 * 1024 * 1024, size_t(collection->getNbObjects()) * 512));
    const uint64_t count = cooked.size();
    stream.write(&count, sizeof(count));
    for (PxU32 i = 0; i < meshes->getNbObjects(); ++i)
    {
        const uint64_t entry[2] = { meshes->getId(meshes->getObject(i)), cooked[i]->size() };
        stream.write(entry, sizeof(entry));
    }
    for (const auto& bytes : cooked)
        stream.write(bytes->data(), bytes->size());
    // The collection must start at an aligned address: storage is aligned to PX_SERIAL_FILE_ALIGN.
    static const uint8_t padding[PX_SERIAL_FILE_ALIGN] = {};
    stream.write(padding, (PX_SERIAL_FILE_ALIGN - stream.bytes.size() % PX_SERIAL_FILE_ALIGN) % PX_SERIAL_FILE_ALIGN);
    // The streams alone can exceed the limit: then stop before PhysX validates,
    // sorts and serializes the collection.
    if (stream.overflow)
        return false;
    if (!PxSerialization::serializeCollectionToBinary(stream, *collection, registry, meshes.get(), false) ||
        stream.overflow)
        return false;
    output = std::move(stream.bytes);
    return true;
}

struct RuntimeArchive;
class DecodedCollection;
// Decodes the runtime section of an envelope the datastore adapter already
// validated and copies its native section into aligned storage. It creates no
// PhysX object: call DecodedCollection::deserialize under serialization admission.
// With deferredRuntime, the runtime section is copied there instead of decoded,
// so the caller can decode it while the native section deserializes.
bool decodeStoredBundle(const omni::physics::parse::cache::BundleView& view,
                        RuntimeArchive& archive,
                        std::unique_ptr<DecodedCollection>& native,
                        omni::physics::parse::cache::Bytes* deferredRuntime = nullptr);

// Owns native objects AND their writable deserialization storage, and the
// triangle meshes created from the section's cooked streams. Destroy this after
// removing runtime records; do not separately release its native objects.
// A loader transferring ownership must explicitly remove the objects it takes
// from collection(), and retain this owner until those objects are destroyed.
class DecodedCollection
{
public:
    ~DecodedCollection()
    {
        for (const std::function<void()>& retire : mBeforeRelease)
            retire();
        if (mCollection)
        {
            // Pruning structures refer to actors and must retire first. Remove
            // them explicitly, including when one is first in collection order.
            for (::physx::PxU32 i = 0; i < mCollection->getNbObjects();)
            {
                auto* pruning = mCollection->getObject(i).is<::physx::PxPruningStructure>();
                if (pruning)
                {
                    mCollection->remove(*pruning);
                    pruning->release();
                }
                else
                    ++i;
            }
            // A constraint is released by its joint and is not releasable through
            // the collection. PhysX still tracks one that no joint owns, which only
            // a rejected bundle carries, so release it here: bytes PhysX serialized
            // list it among its actors' connectors, where its release finds it.
            if (!constraintsOwnedByJoints(*mCollection))
            {
                std::unordered_set<const ::physx::PxBase*> owned;
                for (::physx::PxU32 i = 0; i < mCollection->getNbObjects(); ++i)
                    if (const ::physx::PxJoint* joint = asJoint(mCollection->getObject(i)))
                        owned.insert(joint->getConstraint());
                for (::physx::PxU32 i = 0; i < mCollection->getNbObjects();)
                {
                    ::physx::PxBase& object = mCollection->getObject(i);
                    if (object.getConcreteType() == ::physx::PxConcreteType::eCONSTRAINT && !owned.count(&object))
                    {
                        mCollection->remove(object);
                        static_cast<::physx::PxConstraint&>(object).release();
                    }
                    else
                        ++i;
                }
            }
            // Deserialized exclusive shapes have an application reference too.
            ::physx::PxCollectionExt::releaseObjects(*mCollection, true);
            mCollection->release();
        }
        std::free(mStorage);
    }
    DecodedCollection(const DecodedCollection&) = delete;
    DecodedCollection& operator=(const DecodedCollection&) = delete;
    // Valid after deserialize succeeded.
    ::physx::PxCollection& collection() const
    {
        return *mCollection;
    }
    // Create the triangle meshes from their cooked streams, then the other native
    // objects in place in the prepared storage, once. The meshes join the
    // collection under their serial IDs, and it releases them with the others.
    bool deserialize(::physx::PxSerializationRegistry& registry)
    {
        using namespace ::physx;
        if (mCollection || !mAligned)
            return false;
        auto* bytes = static_cast<uint8_t*>(mAligned);
        uint64_t count = 0;
        constexpr size_t kEntry = 2 * sizeof(uint64_t);
        if (mSize < sizeof(count))
            return false;
        std::memcpy(&count, bytes, sizeof(count));
        if (count > (mSize - sizeof(count)) / kEntry)
            return false;
        size_t offset = sizeof(count) + size_t(count) * kEntry;
        CollectionPtr meshes(PxCreateCollection());
        if (!meshes)
            return false;
        // Until the collection takes them, a failure releases the meshes created so far.
        struct Created
        {
            PxCollection& meshes;
            bool taken = false;
            ~Created()
            {
                if (!taken)
                    for (PxU32 i = 0; i < meshes.getNbObjects(); ++i)
                        static_cast<PxTriangleMesh&>(meshes.getObject(i)).release();
            }
        } created{ *meshes };
        for (uint64_t i = 0; i < count; ++i)
        {
            uint64_t entry[2]; // serial ID, stream size
            std::memcpy(entry, bytes + sizeof(count) + size_t(i) * kEntry, kEntry);
            if (entry[0] == PX_SERIAL_OBJECT_ID_INVALID || meshes->find(entry[0]) || entry[1] > mSize - offset ||
                entry[1] > std::numeric_limits<PxU32>::max())
                return false;
            PxDefaultMemoryInputData stream(bytes + offset, PxU32(entry[1]));
            PxTriangleMesh* mesh = PxGetPhysics().createTriangleMesh(stream);
            if (!mesh)
                return false;
            meshes->add(*mesh, entry[0]);
            offset += size_t(entry[1]);
        }
        offset += (PX_SERIAL_FILE_ALIGN - offset % PX_SERIAL_FILE_ALIGN) % PX_SERIAL_FILE_ALIGN;
        if (offset >= mSize)
            return false;
        mCollection = PxSerialization::createCollectionFromBinary(bytes + offset, registry, meshes.get());
        if (!mCollection)
            return false;
        // A mesh ID the collection uses too fails the bundle. The destructor then
        // releases the collection, and with its shapes the meshes' last references.
        for (PxU32 i = 0; i < meshes->getNbObjects(); ++i)
            if (mCollection->find(meshes->getId(meshes->getObject(i))))
                return false;
        for (PxU32 i = 0; i < meshes->getNbObjects(); ++i)
        {
            PxBase& mesh = meshes->getObject(i);
            mCollection->add(mesh, meshes->getId(mesh));
        }
        created.taken = true;
        return true;
    }
    const void* storage() const
    {
        return mAligned;
    }
    // Whether a native object was deserialized in place in this allocation. A
    // triangle mesh never is: it is created from its cooked stream.
    bool owns(const void* object) const
    {
        const uintptr_t address = reinterpret_cast<uintptr_t>(object), base = reinterpret_cast<uintptr_t>(mAligned);
        return mAligned && address >= base && address - base < mSize;
    }
    // Runtime metadata arenas follow this same owner lifetime. Records using
    // them mark Allocateable::mOwnsMemory=false; the arena destroys every value
    // after the owner's runtime records retire, including values removed early.
    void retainRuntimeStorage(std::shared_ptr<void> storage)
    {
        mRuntimeStorage.push_back(std::move(storage));
    }
    // Runs before the native objects are released, for a side table keyed by a
    // restored object's address, such as a triangle mesh's face map.
    void beforeRelease(std::function<void()> retire)
    {
        mBeforeRelease.push_back(std::move(retire));
    }

    // Validate the envelope with the expected build/SDK/platform key before
    // entering PhysX. Cache storage must still be trusted: a checksum provides
    // corruption detection, not authentication of arbitrary native payloads.
    static std::unique_ptr<DecodedCollection> decode(const omni::physics::parse::cache::Bytes& encoded,
                                                     const omni::physics::parse::cache::Digest& input,
                                                     const omni::physics::parse::cache::Digest& compatibility,
                                                     ::physx::PxSerializationRegistry& registry)
    {
        omni::physics::parse::cache::Bundle validated;
        if (!omni::physics::parse::cache::decodeBundle(encoded, input, compatibility, validated))
            return nullptr;
        auto result = prepare(validated.native.data(), validated.native.size());
        return result && result->deserialize(registry) ? std::move(result) : nullptr;
    }

private:
    // Only the attachment cache may use the already-validated datastore result.
    // Keep the raw-byte entry point above validating its own envelope.
    friend bool decodeStoredBundle(const omni::physics::parse::cache::BundleView&,
                                   RuntimeArchive&,
                                   std::unique_ptr<DecodedCollection>&,
                                   omni::physics::parse::cache::Bytes*);
    static std::unique_ptr<DecodedCollection> prepare(const uint8_t* bytes, size_t size)
    {
        if (!bytes || !size || size > std::numeric_limits<size_t>::max() - PX_SERIAL_FILE_ALIGN)
            return nullptr;
        std::unique_ptr<DecodedCollection> result(new DecodedCollection);
        result->mStorage = std::malloc(size + PX_SERIAL_FILE_ALIGN - 1);
        // Out of memory is this process's failure, not a defect of the bundle.
        if (!result->mStorage)
            throw std::bad_alloc();
        const uintptr_t address = reinterpret_cast<uintptr_t>(result->mStorage);
        result->mAligned =
            reinterpret_cast<void*>((address + PX_SERIAL_FILE_ALIGN - 1) & ~(uintptr_t(PX_SERIAL_FILE_ALIGN) - 1));
        std::memcpy(result->mAligned, bytes, size);
        result->mSize = size;
        return result;
    }

private:
    DecodedCollection() = default;
    std::vector<std::shared_ptr<void>> mRuntimeStorage;
    std::vector<std::function<void()>> mBeforeRelease;
    void* mStorage = nullptr;
    void* mAligned = nullptr;
    size_t mSize = 0;
    ::physx::PxCollection* mCollection = nullptr;
};
} // namespace omni::physx::cache
