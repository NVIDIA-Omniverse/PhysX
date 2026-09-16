// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-REPLICATE-001
 * @covers AC-2 AC-4 AC-6 AC-11
 *
 * @implements REQ-SIM-OVSTAGE-ATTACH-001
 * @covers AC-2
 */

// BREAKING public-ABI change (ADR-0018): the `uint64_t` carried by `replicate()`'s `path`,
// `HierarchyRenameFn` and `ReplicationAttachFn`'s `excludePaths` is now
// `omni::physics::parse::ObjectKey::handle`, no longer a `reinterpret_cast` of SdfPath's
// private in-memory representation (`sdfPathToInt`). External callers must use
// `IPhysx::resolveObjectKey(path).handle`. See IPhysxReplicator.h.

#include <PxPhysicsAPI.h>
#include "PhysXReplicator.h"
#include "OmniPhysX.h"
#include "PhysXMirror.h"
#include "PhysXTools.h"
#include "ContactReport.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/AttachedStage.h"
#include "usdLoad/LoadStage.h"
#include "usdLoad/CollisionGroup.h"
#include "usdLoad/PhysicsBody.h"
#include "usdInterface/UsdInterface.h"
#include <carb/profiler/Profile.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>
#include <PhysXSimulationCallbacks.h>
#include "internal/InternalMimicJoint.h"
#include <omni/physx/IPhysxSettings.h>

#include "utils/Time.h"
#include <utils/Profile.h>
#include <carb/extras/Timer.h>

#include <regex>


using namespace ::physx;
using namespace carb::tasking;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

using CompoundShapeMap = std::unordered_map<omni::physics::parse::ObjectKey, std::pair<CompoundShape*, size_t>,
                                             omni::physics::parse::ObjectKey::Hash>;

// PxActor/PxAggregate::setEnvironmentID require every non-invalid environment id to be < 1<<24;
// larger ids make the setter fail (returns false) and leave the object at PX_INVALID_U32.
static constexpr uint32_t kMaxPhysxEnvironments = 1u << 24;

#define INTERNAL_PTR_ALIGN 16

namespace omni
{
    namespace physx
    {


        template <typename T>
        class InternalPtrAllocator
        {
        public:
            InternalPtrAllocator()
            {
                mAlignedSize = 0;
                mPtrs = nullptr;
                mAlignedPtrs = nullptr;                
            }

            void preAllocate(size_t numPtrs)
            {
                mAlignedSize = (sizeof(T) + INTERNAL_PTR_ALIGN) & ~(INTERNAL_PTR_ALIGN - 1);
                mPtrs = malloc(mAlignedSize * (numPtrs) + INTERNAL_PTR_ALIGN);
                mAlignedPtrs = (T*)((size_t(mPtrs) + INTERNAL_PTR_ALIGN) & ~(INTERNAL_PTR_ALIGN - 1));                
            }

            const size_t getAlignedSize() const
            {
                return mAlignedSize;
            }

            T* getAlignedMemory()
            {
                return mAlignedPtrs;
            }

            const T* getAlignedMemory() const
            {
                return mAlignedPtrs;
            }

            void* getMemory()
            {
                return mPtrs;
            }

        private:
            void* mPtrs;
            T* mAlignedPtrs;
            size_t mAlignedSize;
        };

        struct MemoryDesc
        {
            MemoryDesc()
                : numActors(0), numLinks(0), numArticulations(0), numArticulationJoints(0), numJoints(0),
                numShapes(0), numMaterials(0),numTendonAxis(0),
                numTendonAttachments(0), numMimicJoints(0)
            {
            }

            size_t numActors;
            size_t numLinks;
            size_t numArticulations;
            size_t numArticulationJoints;
            size_t numJoints;
            size_t numShapes;
            size_t numMaterials;
            size_t numTendonAxis;
            size_t numTendonAttachments;
            size_t numMimicJoints;
        };

        struct ReplicatorMemory
        {
            void initialize(const MemoryDesc& memoryDescIn, uint32_t numReplications)
            {
                memoryDesc = memoryDescIn;
                if (memoryDesc.numActors)
                    actorAllocator.preAllocate(memoryDesc.numActors * numReplications);
                if (memoryDesc.numLinks)
                    linkAllocator.preAllocate(memoryDesc.numLinks * numReplications);
                if (memoryDesc.numArticulations)
                    articulationAllocator.preAllocate(memoryDesc.numArticulations * numReplications);
                if (memoryDesc.numArticulationJoints)
                    articulationJointAllocator.preAllocate(memoryDesc.numArticulationJoints * numReplications);
                if (memoryDesc.numJoints)
                    jointAllocator.preAllocate(memoryDesc.numJoints * numReplications);
                if (memoryDesc.numShapes)
                    shapeAllocator.preAllocate(memoryDesc.numShapes * numReplications);
                if (memoryDesc.numMaterials)
                    materialAllocator.preAllocate(memoryDesc.numMaterials * numReplications);
                if (memoryDesc.numTendonAxis)
                    tendonAxisAllocator.preAllocate(memoryDesc.numTendonAxis * numReplications);
                if (memoryDesc.numTendonAttachments)
                    tendonAttachmentAllocator.preAllocate(memoryDesc.numTendonAttachments * numReplications);
                if (memoryDesc.numMimicJoints)
                    mimicJointAllocator.preAllocate(memoryDesc.numMimicJoints * numReplications);
            }

            InternalPtrAllocator<InternalActor> actorAllocator;
            InternalPtrAllocator<InternalLink> linkAllocator;
            InternalPtrAllocator<InternalArticulation> articulationAllocator;
            InternalPtrAllocator<InternalJoint> articulationJointAllocator;
            InternalPtrAllocator<InternalJoint> jointAllocator;
            InternalPtrAllocator<InternalShape> shapeAllocator;
            InternalPtrAllocator<InternalMaterial> materialAllocator;
            InternalPtrAllocator<InternalTendonAxis> tendonAxisAllocator;
            InternalPtrAllocator<InternalTendonAttachment> tendonAttachmentAllocator;
            InternalPtrAllocator<InternalMimicJoint> mimicJointAllocator;

            MemoryDesc memoryDesc;
        };

        template <typename T>
        class InternalPtrAllocatorIterator
        {
        public:
            InternalPtrAllocatorIterator(const InternalPtrAllocator<T>& alloc, size_t index, size_t numElements)
                : alignedPtr(nullptr), alignedSize(0)
            {
                alignedSize = alloc.getAlignedSize();
                alignedPtr = (T*)(((uint8_t*)alloc.getAlignedMemory()) + (numElements * alignedSize * index));
            }
            
            T* getAlignedMemory()
            {
                T* retVal = alignedPtr;
                alignedPtr = (T*)(((uint8_t*)alignedPtr) + alignedSize);
                return retVal;
            }

        private:
            T* alignedPtr;
            size_t alignedSize;
        };

        struct ReplicatorMemoryView
        {
            ReplicatorMemoryView(const ReplicatorMemory& replicatorMemory, size_t index)
               : actorAllocatorIt(replicatorMemory.actorAllocator, index, replicatorMemory.memoryDesc.numActors),
                linkAllocatorIt(replicatorMemory.linkAllocator, index, replicatorMemory.memoryDesc.numLinks),
                articulationAllocatorIt(replicatorMemory.articulationAllocator, index, replicatorMemory.memoryDesc.numArticulations),
                articulationJointAllocatorIt(replicatorMemory.articulationJointAllocator, index, replicatorMemory.memoryDesc.numArticulationJoints),
                jointAllocatorIt(replicatorMemory.jointAllocator, index, replicatorMemory.memoryDesc.numJoints),
                shapeAllocatorIt(replicatorMemory.shapeAllocator, index, replicatorMemory.memoryDesc.numShapes),
                materialAllocatorIt(replicatorMemory.materialAllocator, index, replicatorMemory.memoryDesc.numMaterials),
                tendonAxisAllocatorIt(replicatorMemory.tendonAxisAllocator, index, replicatorMemory.memoryDesc.numTendonAxis),
                tendonAttachmentAllocatorIt(replicatorMemory.tendonAttachmentAllocator, index, replicatorMemory.memoryDesc.numTendonAttachments),
                mimicJointAllocatorIt(replicatorMemory.mimicJointAllocator, index, replicatorMemory.memoryDesc.numMimicJoints)
            {
            }

            InternalPtrAllocatorIterator<InternalActor> actorAllocatorIt;
            InternalPtrAllocatorIterator<InternalLink> linkAllocatorIt;
            InternalPtrAllocatorIterator<InternalArticulation> articulationAllocatorIt;
            InternalPtrAllocatorIterator<InternalJoint> articulationJointAllocatorIt;
            InternalPtrAllocatorIterator<InternalJoint> jointAllocatorIt;
            InternalPtrAllocatorIterator<InternalShape> shapeAllocatorIt;
            InternalPtrAllocatorIterator<InternalMaterial> materialAllocatorIt;
            InternalPtrAllocatorIterator<InternalTendonAxis> tendonAxisAllocatorIt;
            InternalPtrAllocatorIterator<InternalTendonAttachment> tendonAttachmentAllocatorIt;
            InternalPtrAllocatorIterator<InternalMimicJoint> mimicJointAllocatorIt;
        };

        PhysXReplicator::PhysXReplicator(const IReplicatorCallback& cb)
            : mCallback(cb)
        {

        }

        PhysXReplicator::~PhysXReplicator()
        {
            clear();
        }

        bool PhysXReplicator::attach(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage)
        {
            mExcludePathSet.clear();

            // Create the AttachedStage/Source first so a live Source exists on both paths
            // before replicationAttachFn fires below (a no-op on the ovstage path).
            if (!usdparser::UsdLoad::getUsdLoad()->attachReplicatorCreateSource(stageId, usdPhysicsInt, attachStage))
            {
                return false;
            }

            // Both identities appear in this function on purpose, and they are not interchangeable.
            // The callbacks below report an *attach* (ADR-0016), while attachReplicatorFinish()
            // performs the USD attach's parse and needs a *stage* id -- the ADR-0013 survival
            // clause for the entry points that name the stage object rather than the attach.
            // kActiveAttach is a defensive fallback: the AttachedStage always exists by here.
            auto reportedAttachHandle = [&]() -> AttachHandle
            {
                const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
                return attachedStage ? attachedStage->getAttachHandle() : kActiveAttach;
            };

            // Full parse about to happen, we need to exclude paths defined by the callback.
            if (mCallback.replicationAttachFn)
            {
                uint32_t numPaths = 0;
                uint64_t* paths = nullptr;
                mCallback.replicationAttachFn(reportedAttachHandle(), numPaths, paths, mCallback.userData);
                if (numPaths && paths)
                {
                    for (uint32_t i = 0; i < numPaths; i++)
                    {
                        mExcludePathSet.insert(omni::physics::parse::ObjectKey{ paths[i] });
                    }
                }
            }
            if (!usdparser::UsdLoad::getUsdLoad()->attachReplicatorFinish(stageId, mExcludePathSet, attachStage))
            {
                return false;
            }

            // Full parse is done, we can start to replicate notify the attach end
            if (mCallback.replicationAttachEndFn)
            {
                mCallback.replicationAttachEndFn(reportedAttachHandle(), mCallback.userData);
            }
            return true;
        }

        // Textual equivalent of SdfPath::ReplacePrefix: pathToUpdate is always replicatePath or
        // a namespace descendant of it, so a prefix-with-boundary-check replace is exact. A
        // non-matching pathToUpdate is returned unchanged, as ReplacePrefix would.
        std::string getNewObjectPath(std::string_view pathToUpdate, std::string_view replicatePath,
                                     std::string_view newReplicatePath)
        {
            const bool isPrefix = pathToUpdate.size() >= replicatePath.size() &&
                pathToUpdate.compare(0, replicatePath.size(), replicatePath) == 0 &&
                (pathToUpdate.size() == replicatePath.size() || pathToUpdate[replicatePath.size()] == '/');
            if (!isPrefix)
                return std::string(pathToUpdate);
            return std::string(newReplicatePath) + std::string(pathToUpdate.substr(replicatePath.size()));
        }

        // World pose for clone `cloneIdx` from the caller's flat [N*7] transform array
        // (px,py,pz, qx,qy,qz,qw -- matching the tensor pose format). Returns false when no
        // transform was supplied for this clone, leaving the copy co-located on the source.
        bool cloneTransformAt(const std::vector<float>& transforms, uint32_t cloneIdx, PxTransform& out)
        {
            const size_t base = static_cast<size_t>(cloneIdx) * 7;
            if (base + 7 > transforms.size())
                return false;
            const float* t = &transforms[base];
            out = PxTransform(PxVec3(t[0], t[1], t[2]), PxQuat(t[3], t[4], t[5], t[6]));
            return true;
        }

        // Resolves the PhysXScene an already-created actor/link/cct-typed record landed in at
        // creation (its own physics:simulationOwner -> ObjectId resolution, see
        // UsdInterface.cpp/LoadUsd.cpp), then translates that scene's own SdfPath back to the
        // scene's ObjectId through attachedStage's object registry -- the identity PhysXScenesMap
        // (Setup.h) is actually keyed by. See the "Replicator scene lookup" row in ADR-0016's
        // Remaining residue table (2026-08-16 addendum). Returns kInvalidObjectId when objectId
        // does not name an actor-like record, or that record has no PhysXScene yet.
        ObjectId sceneObjectIdForActorRecord(const internal::InternalPhysXDatabase& db,
                                             const AttachedStage& attachedStage,
                                             ObjectId objectId)
        {
            if (objectId >= db.getRecords().size())
                return kInvalidObjectId;
            const internal::InternalDatabase::Record& record = db.getRecords()[objectId];
            // ePTActor: rigid static/dynamic bodies. ePTLink: articulation links. ePTCct:
            // character controllers. All three are built on InternalActor and carry the
            // PhysXScene they were created into.
            if ((record.mType != ePTActor && record.mType != ePTLink && record.mType != ePTCct) || !record.mInternalPtr)
                return kInvalidObjectId;

            const InternalActor* internalActor = static_cast<const InternalActor*>(record.mInternalPtr);
            if (!internalActor->mPhysXScene)
                return kInvalidObjectId;

            return attachedStage.getObjectDatabase()->findEntry(internalActor->mPhysXScene->getSceneSdfPath(), eScene);
        }

        // Scans subtreePaths for the first object with a resolvable scene (see
        // sceneObjectIdForActorRecord above). Reads back the PhysXScene an already-created object
        // landed in, so it is only ever a fallback for a subtree replicate() has already parsed
        // once (a re-replicate call), or one whose objects carry no explicit
        // physics:simulationOwner at all -- resolveSubtreeSimulationOwner below is the
        // authoritative, pre-parse resolution for an explicitly-owned subtree. Returns
        // kInvalidObjectId when nothing in subtreePaths resolves yet (no objects at all, or only
        // non-actor content such as joints).
        ObjectId findSubtreeSceneId(const internal::InternalPhysXDatabase& db,
                                    const AttachedStage& attachedStage,
                                    const std::vector<omni::physics::parse::ObjectKey>& subtreeKeys)
        {
            for (const omni::physics::parse::ObjectKey& k : subtreeKeys)
            {
                const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(k);
                if (!entries)
                    continue;
                for (const auto& entry : *entries)
                {
                    const ObjectId sceneId = sceneObjectIdForActorRecord(db, attachedStage, entry.second);
                    if (sceneId != kInvalidObjectId)
                        return sceneId;
                }
            }
            return kInvalidObjectId;
        }

        // Reads physics:simulationOwner (getRigidBodySimulationOwner, the same source read
        // LoadStage.cpp's checkArticulatonBodySimulationOwners uses per-articulation/per-joint)
        // off every object in topPath's subtree -- BEFORE anything is parsed or created, since it
        // goes through IPhysicsSource directly rather than the created-object database. Used to
        // resolve the subtree's owning scene up front on a multi-scene setup (see replicate()),
        // so the source is created into the right scene the first time instead of being corrected
        // after the fact.
        // Returns false when two objects in the subtree carry different explicit owners (a mixed-
        // scene subtree, which a single replicate() call cannot serve -- see mirrorHierarchy's
        // single PxScene). Returns true otherwise and fills outOwnerPath with the subtree's single
        // explicit owner, or an empty path when none is authored anywhere in the subtree.
        bool resolveSubtreeSimulationOwner(AttachedStage& attachedStage,
                                           omni::physics::parse::ObjectKey topKey,
                                           omni::physics::parse::ObjectKey& outOwnerKey)
        {
            outOwnerKey = omni::physics::parse::ObjectKey{};
            const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
            if (!source)
                return true;

            omni::physics::parse::KnownTokens tok;
            tok.intern(*source);

            // An empty physics:simulationOwner is not "no opinion" -- it resolves to the default
            // PhysX scene. Comparing against that resolved key (rather than skipping ownerless
            // bodies entirely) is what actually decides whether a subtree targets a single scene:
            // a subtree with one ownerless body and one body explicitly owned by a DIFFERENT scene
            // is just as mixed-scene as two bodies with two different explicit owners, and must be
            // rejected the same way -- not silently narrowed to the explicit owner while the
            // ownerless body is later parsed into the default scene and omitted from replication
            // (mirrorHierarchy enumerates only the one resolved scene).
            PhysXScene* defaultScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(0);
            const omni::physics::parse::ObjectKey defaultSceneKey =
                defaultScene ? defaultScene->getSceneSdfPath() : omni::physics::parse::ObjectKey{};

            bool ownerSeen = false;
            bool consistent = true;
            source->forEachDescendantPruned(
                topKey,
                [&](omni::physics::parse::ObjectKey key) -> bool
                {
                    if (!consistent)
                        return true; // already inconsistent: prune the rest, nothing left to learn
                    // Only a rigid-body- or collision-API-carrying descendant has a scene
                    // membership opinion at all (getRigidBodySimulationOwner's own doc comment:
                    // physics:simulationOwner is declared by UsdPhysicsRigidBodyAPI or
                    // UsdPhysicsCollisionAPI). A plain container (Xform/Scope) descendant reads
                    // back an empty owner too, but for a different reason -- it is not a
                    // scene-owned object at all, not an ownerless one -- and must stay skipped,
                    // not compared against the default scene.
                    if (!source->hasSchema(key, tok.physicsRigidBodyAPI) &&
                        !source->hasSchema(key, tok.physicsCollisionAPI))
                        return false;
                    omni::physics::parse::ObjectKey owner = getRigidBodySimulationOwner(attachedStage, key);
                    if (!owner.valid())
                    {
                        if (!defaultSceneKey.valid())
                            return false; // default scene not resolvable yet -- cannot compare, skip
                        owner = defaultSceneKey;
                    }
                    if (!ownerSeen)
                    {
                        outOwnerKey = owner;
                        ownerSeen = true;
                    }
                    else if (outOwnerKey != owner)
                    {
                        consistent = false;
                    }
                    return false;
                },
                omni::physics::parse::DescendantScope::eActiveInstanced);

            // Preserve the prior outward contract for a subtree that is effectively all-default
            // (no explicit owner anywhere, or every explicit owner IS the default scene): report
            // no override so the caller falls back to its legacy object-database-derived pick,
            // unchanged. defaultSceneKey was only needed internally above, to catch a subtree
            // mixing ownerless bodies with a DIFFERENT explicit owner.
            if (consistent && defaultSceneKey.valid() && outOwnerKey == defaultSceneKey)
                outOwnerKey = omni::physics::parse::ObjectKey{};
            return consistent;
        }

        // Single source of truth for "which InternalDatabase record does this PxBase clone belong
        // to" -- both processObjectFn's per-object dispatch and the numReplications>=128 parallel
        // branch's pre-intern loop (see the "Pre-intern every clone ObjectKey" comment further down)
        // must stay in lock-step on every PxConcreteType case the replicator clones: the
        // pre-intern loop's whole memory-safety argument is that it inserts every key a worker
        // thread will later look up WITHOUT locking, so a case handled by one switch and missed by
        // the other reintroduces a heap-corrupting data race in release builds. Returns SIZE_MAX
        // for a PxConcreteType this replicator does not clone, or a PxConstraint whose external
        // reference is not a PxJoint.
        size_t replicatorIdForPxBase(PxBase& object)
        {
            switch (object.getConcreteType())
            {
            case PxConcreteType::eRIGID_STATIC:
            case PxConcreteType::eRIGID_DYNAMIC:
            case PxConcreteType::eARTICULATION_LINK:
                return (size_t)((PxRigidActor&)object).userData;
            case PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
                return (size_t)((PxArticulationReducedCoordinate&)object).userData;
            case PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
                return (size_t)((PxArticulationJointReducedCoordinate&)object).userData;
            case PxConcreteType::eARTICULATION_TENDON_JOINT:
                return (size_t)((PxArticulationTendonJoint&)object).userData;
            case PxConcreteType::eARTICULATION_ATTACHMENT:
                return (size_t)((PxArticulationAttachment&)object).userData;
            case PxConcreteType::eCONSTRAINT:
            {
                PxConstraint& constraint = (PxConstraint&)object;
                PxU32 typeId;
                PxJoint* joint = reinterpret_cast<PxJoint*>(constraint.getExternalReference(typeId));
                return (joint && typeId == PxConstraintExtIDs::eJOINT) ? (size_t)joint->userData : SIZE_MAX;
            }
            case PxConcreteType::eSHAPE:
                return (size_t)((PxShape&)object).userData;
            case PxConcreteType::eMATERIAL:
                return (size_t)((PxMaterial&)object).userData;
            case PxConcreteType::eARTICULATION_MIMIC_JOINT:
                return (size_t)((PxArticulationMimicJoint&)object).userData;
            default:
                return SIZE_MAX;
            }
        }

        bool PhysXReplicator::replicate(AttachHandle attachHandle,
                                        uint64_t path,
                                        uint32_t numReplications,
                                        bool useEnvIDsIn)
        {
            CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate");
            if (numReplications == 0)
                return false;

            AttachedStage* attachedStage = UsdLoad::getUsdLoad()->resolveAttach(attachHandle);
            if (!attachedStage)
            {
                CARB_LOG_ERROR("Attached Stage not found for replication. AttachHandle: %llu",
                               static_cast<unsigned long long>(attachHandle));
                return false;
            }

            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            ObjectDb* objectDb = attachedStage->getObjectDatabase();
            std::unordered_set<void*> physxPtrs;

            // `path` is an ObjectKey.handle, not an SdfPath-bit encoding -- see the file top.
            const omni::physics::parse::ObjectKey topKey{ path };

            // Source-path validation, and only that, is USD-specific: on a stage-backed attach a
            // typo'd source path must still be rejected up front with the message it always had.
            // A stageless attach has no backing prim for that message to be about -- the
            // post-parse enumeration below is its equivalent gate. Gating on `getSource()`
            // instead would NOT work: an ovstage attach always has a source, stage or not.
            if (attachedStage->getStageId() != 0)
            {
                const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource();
                if (source && !source->exists(topKey))
                {
                    CARB_LOG_ERROR("Prim to replicate not found in USD stage. AttachHandle: %llu",
                                   static_cast<unsigned long long>(attachHandle));
                    return false;
                }
            }

            // The source subtree is enumerated from the object DB's own prim hierarchy, not from
            // USD. Both walks below only ever used USD as a path enumerator -- neither reads an
            // attribute or a schema off the prim; the only thing taken from it is GetPrimPath(),
            // immediately used as an ObjectDb key. PrimHierarchyStorage indexes exactly the paths
            // that had objects created on them (plus their ancestors), on every source, and it is
            // populated for instance-proxy paths too: the USD walker scans with
            // UsdTraverseInstanceProxies, so a collider inside a prototype registers under its
            // instance-proxy path and ObjectDb::findOrCreateEntry links that path into the
            // hierarchy. Measured on a scene-graph-instanced source (cabinet.usda, 4 envs): 84 of
            // 254 ObjectDb paths are instance proxies, 0 ObjectDb or schema-API paths are absent
            // from PrimHierarchyStorage, and the descent below reaches all of them from every
            // subtree root tried. The descent is O(subtree), where prefix-filtering the whole path
            // map would be O(all objects) -- the wrong complexity for the many-environment stages
            // replication exists for.
            auto enumerateSubtree = [&]() -> std::vector<omni::physics::parse::ObjectKey>
            {
                // PrimHierarchyStorage is std::string-keyed; re-intern each descendant back to
                // an ObjectKey at this one boundary.
                const PrimHierarchyStorage::Iterator it(objectDb->getPrimHierarchyStorage(),
                                                        std::string(attachedStage->textFor(topKey)));
                std::vector<omni::physics::parse::ObjectKey> result;
                result.reserve(it.getDescendentsPaths().size());
                for (const std::string& descendant : it.getDescendentsPaths())
                    result.push_back(attachedStage->keyFor(std::string_view(descendant)));
                return result;
            };

            internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();
            PhysXSetup& physxSetupForSceneResolve = OmniPhysX::getInstance().getPhysXSetup();

            // ADR-0016, "Replicator scene lookup" residue (2026-08-16 addendum): PhysXScenesMap is
            // keyed by a scene's runtime ObjectId, never by a stage id or an attach handle, so
            // resolving through either raw integer is meaningless -- on a single-scene stage it
            // "worked" only because getPhysXScene falls back to mDefaultScene on a miss, and on a
            // multi-scene stage that same miss can land on some other stage's scene instead of
            // reliably missing. On a multi-scene setup the owning scene is resolved up front, from
            // the subtree's own authored physics:simulationOwner (resolveSubtreeSimulationOwner
            // above, the same source read LoadStage.cpp's checkArticulatonBodySimulationOwners
            // uses) -- a source read, not an object-database lookup, so it works whether or not the
            // subtree has been parsed yet, and it is what determines the scene the source is
            // CREATED into: unlike the old object-database-derived guess, it does not need a
            // post-parse correction (which could size the pre-parse GPU-flag checks right below
            // from the wrong scene -- see the sourceLacksEnvId comment further down for what that
            // used to cost). A mixed-scene subtree is rejected here, loudly, instead of silently
            // cloning only the part that happens to share the first-resolved scene.
            ObjectId resolvedSceneId = kInvalidObjectId;
            if (physxSetupForSceneResolve.getPhysXScenes().size() > 1)
            {
                omni::physics::parse::ObjectKey subtreeOwner;
                if (!resolveSubtreeSimulationOwner(*attachedStage, topKey, subtreeOwner))
                {
                    CARB_LOG_ERROR(
                        "Cannot replicate '%s': its subtree contains bodies with different simulation owners "
                        "(physics:simulationOwner); a single replicate() call clones into exactly one PhysX "
                        "scene. AttachHandle: %llu",
                        attachedStage->textFor(topKey), static_cast<unsigned long long>(attachHandle));
                    return false;
                }
                if (subtreeOwner.valid())
                {
                    resolvedSceneId = objectDb->findEntry(subtreeOwner, eScene);
                    if (resolvedSceneId == kInvalidObjectId)
                    {
                        CARB_LOG_ERROR(
                            "Cannot replicate '%s': its simulation owner '%s' does not resolve to a registered "
                            "PhysX scene. AttachHandle: %llu",
                            attachedStage->textFor(topKey), attachedStage->textFor(subtreeOwner),
                            static_cast<unsigned long long>(attachHandle));
                        return false;
                    }
                }
            }

            // No explicit owner was authored anywhere in the subtree (or there is only one scene
            // to begin with): fall back to the legacy object-database-derived guess -- non-empty
            // only for a subtree that already carries objects (a re-replicate call, or a source
            // parsed earlier at attach); a brand-new, ownerless subtree lands on the default scene
            // (index 0), same as before this fix.
            if (resolvedSceneId == kInvalidObjectId)
            {
                resolvedSceneId = findSubtreeSceneId(db, *attachedStage, enumerateSubtree());
            }
            PhysXScene* scene = physxSetupForSceneResolve.getPhysXScene(
                resolvedSceneId != kInvalidObjectId ? static_cast<size_t>(resolvedSceneId) : 0);
            if (!scene)
            {
                CARB_LOG_ERROR("PhysX Scene not found for replication. AttachHandle: %llu",
                               static_cast<unsigned long long>(attachHandle));
                return false;
            }

            bool implicitEnvIdsUse = false;
            // kScenePartitionPrimvar has no KnownTokens entry yet (a genuine gap, not one of
            // this round's built siblings), so it is interned locally here, same as
            // PhysXCooking.cpp's inline internToken("Mesh") precedent.
            if (const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource())
            {
                const omni::physics::parse::TokenId scenePartitionAttr = source->internToken(kScenePartitionPrimvar);
                omni::physics::parse::TokenId scenePartitionToken;
                if (getValue<omni::physics::parse::TokenId>(*attachedStage, topKey,
                                                             scenePartitionAttr,
                                                             omni::physics::parse::ReadTime::defaultTime(),
                                                             scenePartitionToken))
                {
                    implicitEnvIdsUse = true;
                    CARB_LOG_INFO("Registering scene partion token: %s",
                                  std::string(source->tokenToString(scenePartitionToken)).c_str());
                    attachedStage->registerEnvIdFromToken(scenePartitionToken);
                }
            }

            PxScene* physXScene = scene->getScene();
            // Not const: the defensive re-check further down (after the source is parsed) may
            // still swap `scene`/`physXScene` out from under this pick, for the no-explicit-owner
            // fallback case. useEnvIDs gates real clone-creation behaviour further down (env id
            // stamping), not just the pre-parse optimization, so it must track whichever scene
            // clones actually land in.
            bool gpuDynamics = physXScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
            bool gpuBroadphase = physXScene->getBroadPhaseType() == PxBroadPhaseType::eGPU;
            bool useEnvIDs = gpuDynamics && gpuBroadphase && (useEnvIDsIn || implicitEnvIdsUse); // PT: TODO: or is gpuBroadphase
                                                                                // enough?

            if (useEnvIDsIn && !gpuDynamics)
            {
                CARB_LOG_WARN("EnvIds requested but gpu dynamic is disabled.");
            }
            if (useEnvIDsIn && !gpuBroadphase)
            {
                CARB_LOG_WARN("EnvIds requested but gpu broadphase is not set.");
            }

            // setup env Ids for initial parsing to avoid scene re-insertion; preserve the
            // attach-time creation-time-ids mode (kSettingReplicatorEnvIdsOnAttach) so bodies
            // parsed after this replicate (e.g. later ovstage updates) keep receiving ids
            const bool priorUseReplicatorEnvIds = attachedStage->isUsingReplicatorEnvIds();
            attachedStage->setUseReplicatorEnvIds(useEnvIDs);

            // parse the source
            bool dataAlreadyParsed = false;
            {
                // Pre-parse the source is normally absent from the hierarchy entirely, so this
                // yields nothing and the parse below runs -- the same decision the USD walk made.
                for (const omni::physics::parse::ObjectKey& traverseKey : enumerateSubtree())
                {
                    const ObjectIdMap* entries = objectDb->getEntries(traverseKey);
                    if (entries && !entries->empty())
                    {
                        dataAlreadyParsed = true;
                        break;
                    }
                }
            }
            if (!dataAlreadyParsed)
            {
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate::source");
                PHYSICS_PROFILE("PhysXReplicator::replicate::source");
                // Block the USD notification to get recursive callback
                UsdLoad::getUsdLoad()->blockUSDUpdate(true);

                const std::vector<std::string> updateRoots{ std::string(attachedStage->textFor(topKey)) };
                loadPhysicsFromPrimitive(*attachedStage, updateRoots);
                // Process batched changes while USD notices remain blocked.
                attachedStage->getPhysXPhysicsInterface()->finishSetup(*attachedStage);

                UsdLoad::getUsdLoad()->blockUSDUpdate(false);
            }
            else
            {
                // Attach or an ovstage update may have parsed the source.
                // Resolve its deferred setup before cloning contact pairs.
                attachedStage->getPhysXPhysicsInterface()->finishSetup(*attachedStage);
            }

            // restore the attach-time env-ids mode after the source parse
            attachedStage->setUseReplicatorEnvIds(priorUseReplicatorEnvIds);

            // If we clone materials, we need to resolve the materials in a second pass
            bool materialResolveEnabled = false;

            std::vector<std::pair<omni::physics::parse::ObjectKey, uint32_t>> schemaAPIFlagsStorage;

            // Source isolation check (env-ids engaged): a source parsed at attach WITHOUT
            // creation-time env ids (kSettingReplicatorEnvIdsOnAttach) carries PX_INVALID_U32,
            // which collides with every environment (PxActor.h) -- including all co-located
            // clones. Env ids can only be assigned OUTSIDE a scene, and remove/re-add of live
            // bodies corrupts eENABLE_DIRECT_GPU_API scenes (PxgBodySimManager has no rigid-body
            // unstaging, so a re-added body's GPU state is never initialized), so this is
            // detect-and-warn only: the fix is to parse with the setting enabled, which assigns
            // ids at creation.
            bool sourceLacksEnvId = false;

            // Re-enumerate: the parse above is what created the source's objects, so the subtree
            // taken before it is stale (empty, on a first replicate).
            const std::vector<omni::physics::parse::ObjectKey> sourceSubtree = enumerateSubtree();
            if (sourceSubtree.empty())
            {
                // Nothing was created under the source, so physxPtrs would come out empty and the
                // serializer would clone nothing while replicate() reported success. That silent
                // empty clone is the failure mode this whole path has to avoid, so say so instead.
                CARB_LOG_ERROR(
                    "Nothing to replicate: no physics object was created under '%s' (AttachHandle: %llu). "
                    "The source subtree carries no physics, or it failed to parse.",
                    attachedStage->textFor(topKey), static_cast<unsigned long long>(attachHandle));
                return false;
            }

            // Defensive re-check, not the primary mechanism: for an explicitly-owned subtree
            // `scene` above is already the scene the source was just created into
            // (resolveSubtreeSimulationOwner resolved it BEFORE the parse), so this is a no-op.
            // It only has real effect for the no-explicit-owner fallback (findSubtreeSceneId's
            // pre-parse pick, empty on a brand-new subtree, so scene 0 above): now that
            // sourceSubtree is guaranteed non-empty, re-derive from the object database and adopt
            // whichever scene the source objects actually landed in, so the clone-insertion below
            // (and useEnvIDs, which gates env id stamping on the clones) tracks the real scene
            // rather than the scene-0 guess. This cannot correct the SOURCE's own creation-time
            // env id -- that was decided when it was parsed, above -- see the sourceLacksEnvId
            // comment below.
            {
                const ObjectId reDerivedSceneId = findSubtreeSceneId(db, *attachedStage, sourceSubtree);
                if (reDerivedSceneId != kInvalidObjectId && reDerivedSceneId != resolvedSceneId)
                {
                    PhysXScene* resolvedScene = physxSetupForSceneResolve.getPhysXScene(static_cast<size_t>(reDerivedSceneId));
                    if (resolvedScene)
                    {
                        scene = resolvedScene;
                        physXScene = scene->getScene();
                        gpuDynamics = physXScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
                        gpuBroadphase = physXScene->getBroadPhaseType() == PxBroadPhaseType::eGPU;
                        useEnvIDs = gpuDynamics && gpuBroadphase && (useEnvIDsIn || implicitEnvIdsUse);
                    }
                }
            }

            {
                PHYSICS_PROFILE("PhysXReplicator::replicate::traverseTypes");
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:traverseTypes");
                for (const omni::physics::parse::ObjectKey& traverseKey : sourceSubtree)
                {
                    const ObjectIdMap* entries = objectDb->getEntries(traverseKey);
                    if (entries)
                    {
                        ObjectIdMap::const_iterator it = entries->begin();
                        while (it != entries->end())
                        {
                            // filter allowed objects
                            if (it->first == eShape || it->first == eBody || it->first == eJoint || it->first == eArticulationLink
                                || it->first == eArticulation || it->first == eTendons || it->first == eMaterial
                                || it->first == eFilteredPair || it->first == eArticulationRootJoint
                                || it->first == eMimicJointRotX || it->first == eMimicJointRotY || it->first == eMimicJointRotZ
                                || it->first == eNewtonMimicJoint)
                            {
                                const ObjectId objectId = it->second;
                                CARB_ASSERT(objectId < db.getRecords().size());
                                const internal::InternalDatabase::Record& record = db.getRecords()[objectId];
                                if (it->first == eArticulation && record.mType == ePTArticulation)
                                {
                                    // for an articulation insert also the aggregate
                                    PxAggregate* aggregate = ((InternalArticulation*)record.mInternalPtr)->mAggregate;
                                    if (aggregate)
                                    {
                                        physxPtrs.insert(aggregate);
                                        if (useEnvIDs && aggregate->getEnvironmentID() == PX_INVALID_U32)
                                            sourceLacksEnvId = true;
                                    }
                                }
                                else if (useEnvIDs && it->first == eBody && record.mType == ePTActor && record.mPtr)
                                {
                                    // Dynamic body: env id lives on the actor, unless it is
                                    // aggregated (then the aggregate carries it). Statics register
                                    // under the same eBody category (including shape-only prims
                                    // promoted to a static) but are intentionally left at
                                    // PX_INVALID_U32 -- they never collide with each other, so a
                                    // stamped static gains nothing (see UsdInterface.cpp). Skip them:
                                    // an unstamped static is by design, not evidence the source
                                    // parsed without env ids, so it must not trip the warning below.
                                    PxActor* actor = (PxActor*)record.mPtr;
                                    if (!actor->is<PxRigidStatic>())
                                    {
                                        PxAggregate* actorAggregate = actor->getAggregate();
                                        const uint32_t envId = actorAggregate ? actorAggregate->getEnvironmentID() :
                                                                                actor->getEnvironmentID();
                                        if (envId == PX_INVALID_U32)
                                            sourceLacksEnvId = true;
                                    }
                                }
                                else if (it->first == eJoint)
                                {
                                    physxPtrs.insert(((PxJoint*)record.mPtr)->getConstraint());
                                }
                                else if (it->first == eMaterial)
                                {
                                    materialResolveEnabled = true;
                                }
                                if (record.mPtr)
                                {
                                    physxPtrs.insert(record.mPtr);
                                }
                            }
                            else
                            {
                                const ObjectId objectId = it->second;
                                CARB_ASSERT(objectId < db.getRecords().size());
                                const internal::InternalDatabase::Record& record = db.getRecords()[objectId];

                                CARB_LOG_ERROR("Replication of this type is not supported: %d, prim path: %s", it->first.getData(), attachedStage->textFor(record.mKey));
                            }
                            it++;
                        }
                    }

                    // Schema-API flags for a rigid body / shape (eRigidBodyAPI, eCollisionAPI, ...).
                    const uint64_t schemaFlags = objectDb->getSchemaAPIs(traverseKey);
                    if (schemaFlags)
                    {
                        schemaAPIFlagsStorage.push_back(std::make_pair(traverseKey, schemaFlags));
                    }
                }
            }

            // Co-located isolation requires the SOURCE to carry an env id too; when it does not
            // (see the sourceLacksEnvId comment above), warn rather than mutate live scene
            // objects -- the copies still receive their ids and explicit-transform placement is
            // unaffected.
            if (useEnvIDs && sourceLacksEnvId)
            {
                CARB_LOG_WARN(
                    "PhysXReplicator: replicating '%s' with env-ids, but the source carries no environment id "
                    "(it parsed before the replicator, without %s). Its copies receive env ids, but the source "
                    "itself collides with every environment, so co-located copies are not isolated from it. "
                    "Enable the setting before attaching, or place copies at explicit non-overlapping transforms.",
                    attachedStage->textFor(topKey), kSettingReplicatorEnvIdsOnAttach);
            }

            {
                PHYSICS_PROFILE("PhysXReplicator::replicate::replication");
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:replication");
                void* mirrorMemory = nullptr;
                uint32_t mirrorMemorySize = 0;
                PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
                PxCollection* sharedCollection = nullptr;
                // Parse-time contact-report registrations (addActorPair) sit in the
                // UNRESOLVED map until finishSetup() runs resolvePairs() -- which has
                // not necessarily happened yet when replication is driven through an
                // eager ovstage attach (no simulate/fetchResults cycle before clone).
                // The clone pass below reads only the RESOLVED map, so without this
                // the clones of a contact-reporting source would silently never
                // register (contacts solved, no reports, zero sensor forces).
                // resolvePairs() is cheap and RETAINS entries whose report target has
                // no object-DB entry yet, so this eager call cannot drop
                // registrations that the finishSetup()-time resolve would still catch.
                scene->getContactReport()->resolvePairs();
                ContactPairsMap& contactReportMap = scene->getContactReport()->getContactPairsMap();
                mirrorHierarchy(*scene->getScene(), mirrorMemory, mirrorMemorySize, *physxSetup.getSerializationRegistry(), sharedCollection, physxPtrs);
                const omni::physics::parse::ObjectKey replicateKey = topKey;

                // Clone placement is ABSOLUTE: each caller-supplied anchor is the final world pose
                // of the exact target subtree root. Relocate each object by
                // anchor * inverse(source-root) while preserving its pose within the subtree.
                // Precompute the exact source subtree root's inverse world pose once.
                const PxTransform sourceRootPoseInv = toTransform(internal::getWorldTransform(
                    *attachedStage, replicateKey, omni::physics::parse::ReadTime::defaultTime())).getInverse();

                size_t alignedSize = (mirrorMemorySize + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1);
                void* memblock = malloc(alignedSize * (numReplications)+PX_SERIAL_FILE_ALIGN);
                void* memblockAligned = (void*)((size_t(memblock) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
                const void* mirrorMemoryAligned = (const void*)((size_t(mirrorMemory) + PX_SERIAL_FILE_ALIGN) & ~(PX_SERIAL_FILE_ALIGN - 1));
                attachedStage->addReplicatorMemoryBlock(memblock);
                attachedStage->addReplicatorMemoryBlock(mirrorMemory);

                struct MaterialInfo
                {
                    size_t oldMaterialId;
                    InternalMaterial* newMaterial;
                    size_t newMaterialId;
                };

                struct ObjectDbEntry
                {
                    omni::physics::parse::ObjectKey key;
                    ObjectCategory category;
                    ObjectId newEntryId;
                };

                struct ProcessReplicationData
                {
                    omni::physics::parse::ObjectKey newReplicateKey;
                    PxCollection* collection;
                    std::vector<std::pair<PxRigidActor*, PxTransform>> rigidTransforms;
                    std::vector<std::pair<PxArticulationReducedCoordinate*, PxTransform>> articulationTransforms;
                    std::vector<InternalActor*> internalActors;
                    std::vector<std::pair<PxActor*, const char*>> actorNames;
                    std::vector<std::pair<PxArticulationReducedCoordinate*, const char*>> articulationNames;
                    std::vector<std::pair<PxJoint*, const char*>> jointNames;
                    std::vector<std::pair<PxShape*, const char*>> shapeNames;
                    std::vector<std::pair<PxRigidActor*, float>> contactReports;
                    std::vector<ObjectDbEntry> objectDbEntries;
                    // first: the clone's hierarchy root, as a plain string (PrimHierarchyStorage's
                    // own key type) -- this field only ever feeds addPrimSubtree/
                    // mergeHierarchyStorage below, never a real SdfPath operation, so it stays
                    // string-typed rather than round-tripping through SdfPath.
                    std::pair<std::string, PrimHierarchyStorage> hierarchyStorage;
                    std::vector<std::pair<InternalMaterial*, ObjectId>> materialPairs;
                    std::vector<std::pair<InternalScene*, InternalMimicJoint*>> mimicJointPairs;
                    uint32_t envIdInt;
                };

                // Environments are cloned in parallel (parallelFor below). Record creation routes
                // a path string -> ObjectKey through attachedStage->keyFor(), which would mutate the
                // shared source intern table (push_back/realloc) on a cache miss; concurrent
                // pathFor()/getWorldTransform() calls read the same table unlocked. This code has
                // no synchronization for that read/write pair by design — instead, every clone
                // path this dispatch will need is pre-interned serially, on this thread, before
                // parallelFor starts (see the pre-intern loop right before the dispatch below), so
                // every keyFor()/pathFor() call made from a worker thread is a guaranteed cache hit
                // — a pure, non-mutating lookup on a table that no longer grows during the parallel
                // window. internMutex below no longer protects that invariant (pre-interning does);
                // it still serializes resolveClone's own keyFor/pathFor pair against sibling
                // environment tasks calling it concurrently, though with pre-interning in place
                // those calls no longer mutate the table either.
                MutexWrapper internMutex;

                // Debug-only self-check for resolveClone below: once the pre-intern loop (further
                // down) has serially inserted every clone ObjectKey a parallel dispatch will touch,
                // preInternedClonePathsPtr points at the recorded set and resolveClone asserts its
                // own keyFor(clonePath) call is a lookup into it, not a fresh insert -- so a
                // PxConcreteType case handled by processObjectFn's dispatch but missed by the
                // pre-intern loop's mirror (see replicatorIdForPxBase) fails loudly on the first
                // parallel clone instead of racing. Stays null for the i==0 / forceSyncReplication
                // paths, which never go through the pre-intern loop and do not need the invariant
                // (they never run concurrently with a sibling).
#if CARB_ASSERT_ENABLED
                std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> preInternedClonePaths;
#endif
                const std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash>* preInternedClonePathsPtr = nullptr;

                auto processReplicationFn = [this, memblockAligned, alignedSize, mirrorMemoryAligned, mirrorMemorySize, path,
                    &attachedStage, &db, &physxSetup, sharedCollection, scene, objectDb, &internMutex, &preInternedClonePathsPtr,
                    materialResolveEnabled, numReplications, useEnvIDs, &contactReportMap, replicateKey, sourceRootPoseInv](
                        uint32_t i, bool recordsReady, size_t recordsStart, size_t recordsOffset,
                        ProcessReplicationData& processReplicationData, ReplicatorMemory& replicatorMemory) -> void
                {
                    const omni::physics::parse::ObjectKey newReplicateKey = processReplicationData.newReplicateKey;

                    // process hierarchy
                    processReplicationData.hierarchyStorage.first = std::string(attachedStage->textFor(newReplicateKey));

                    CompoundShapeMap compoundShapes;
                    std::vector<MaterialInfo> oldMaterialNewMaterialTable;
                    std::vector<std::pair<InternalShape*, size_t>> newShapeShapeIdTable;

                    auto processObjectMemoryDescFn = [](PxBase& object, MemoryDesc& memoryDesc) -> void
                    {
                        const PxType objectType = object.getConcreteType();
                        switch (objectType)
                        {
                        case PxConcreteType::eRIGID_STATIC:
                        case PxConcreteType::eRIGID_DYNAMIC:
                        {
                            memoryDesc.numActors++;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_LINK:
                        {
                            memoryDesc.numLinks++;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
                        {
                            memoryDesc.numArticulations++;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
                        {
                            memoryDesc.numArticulationJoints++;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_TENDON_JOINT:
                        {
                            memoryDesc.numTendonAxis++;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_ATTACHMENT:
                        {
                            memoryDesc.numTendonAttachments++;
                        }
                        break;
                        case PxConcreteType::eCONSTRAINT:
                        {
                            PxConstraint& constraint = (PxConstraint&)object;
                            PxU32 typeId;
                            PxJoint* joint = reinterpret_cast<PxJoint*>(constraint.getExternalReference(typeId));
                            if (joint && (typeId == PxConstraintExtIDs::eJOINT))
                            {
                                memoryDesc.numJoints++;
                            }
                        }
                        break;
                        case PxConcreteType::eSHAPE:
                        {
                            // A.B. this is not ideal for compound shapes, we will be allocating more memory then needed
                            memoryDesc.numShapes++;
                        }
                        break;
                        case PxConcreteType::eMATERIAL:
                        {
                            memoryDesc.numMaterials++;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_MIMIC_JOINT:
                        {
                            memoryDesc.numMimicJoints++;
                        }
                        break;
                        default:
                            break;
                        }
                    };

                    auto processObjectFn = [this, i, numReplications, useEnvIDs, &attachedStage, &compoundShapes, &db, scene, objectDb,
                        materialResolveEnabled, &oldMaterialNewMaterialTable, &newShapeShapeIdTable, &internMutex, preInternedClonePathsPtr,
                                            &contactReportMap, replicateKey, newReplicateKey, sourceRootPoseInv](PxBase& object, size_t& recordsIndex,
                                                      ReplicatorMemoryView& memoryView,
                                                      ProcessReplicationData& processReplicationData) -> void
                    {
                        // preInternedClonePathsPtr's only use below is inside CARB_ASSERT, which
                        // compiles away entirely with assertions disabled -- without this, a Clang
                        // -Wall -Wextra -Werror release build sees an unused lambda capture (the
                        // nested resolveClone lambda no longer references it once CARB_ASSERT is
                        // gone, so this outer capture has nothing left using it).
                        (void)preInternedClonePathsPtr;

                        // Resolve the source path, derive the clone path, and intern its key in a
                        // single critical section. Sibling environment tasks share internMutex, so
                        // doing all the intern-table access for one object under one lock keeps it to
                        // a single lock per processObject rather than locking per textFor/keyFor call.
                        omni::physics::parse::ObjectKey cloneKey;
                        auto resolveClone = [&](omni::physics::parse::ObjectKey sourceKey) -> omni::physics::parse::ObjectKey
                        {
                            std::lock_guard<MutexWrapper> lock(internMutex);
                            const std::string clonePathStr = getNewObjectPath(attachedStage->textViewFor(sourceKey),
                                                                              attachedStage->textViewFor(replicateKey),
                                                                              attachedStage->textViewFor(newReplicateKey));
                            cloneKey = attachedStage->keyFor(std::string_view(clonePathStr));
                            CARB_ASSERT(!preInternedClonePathsPtr || preInternedClonePathsPtr->count(cloneKey) != 0);
                            return cloneKey;
                        };
                        omni::physics::parse::ObjectKey newKey;
                        const PxType objectType = object.getConcreteType();
                        switch (objectType)
                        {
                        case PxConcreteType::eRIGID_STATIC:
                        case PxConcreteType::eRIGID_DYNAMIC:
                        case PxConcreteType::eARTICULATION_LINK:
                        {
                            PxRigidActor& actor = (PxRigidActor&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newKey = resolveClone(record.mKey);

                            // Whether an authored clone prim exists.
                            bool hasClonePrim = false;

                            // Per-target world pose for this clone.  Captured here so we can pass
                            // it to copySurfaceVelocityState below: the actual setGlobalPose call
                            // happens in a later physxTask, so reading actor.getGlobalPose() at
                            // copy time would return the source actor's pose (mirrorHierarchy
                            // preserves it).  Default-init to actor's current (= source) pose so
                            // the missing-prim and articulation-link branches still produce a
                            // sane fallback.
                            PxTransform clonePivotPose = actor.is<PxRigidActor>() ?
                                actor.is<PxRigidActor>()->getGlobalPose() : PxTransform(PxIdentity);

                            {
                                // No backing source reads like "no authored clone prim".
                                if (const omni::physics::parse::IPhysicsSource* cloneSource = attachedStage->getSource())
                                    hasClonePrim = cloneSource->exists(newKey);
                                // The root pose (and hence the setEnvironmentID pass below) must NOT be gated
                                // on an authored clone prim: env-id co-location replicates into PhysX copies
                                // without authoring per-env USD, so hasClonePrim is false there. Always push the
                                // root actor / articulation-root pose — the authored world transform when the
                                // clone prim exists (grid-placed USD/Kit clones), else the deserialized actor's
                                // own (= source) pose that mirrorHierarchy preserved (co-located). Env-ids are
                                // what make co-location safe, so they must be assigned to every replicated root.
                                if (objectType != PxConcreteType::eARTICULATION_LINK)
                                {
                                    if (hasClonePrim)
                                    {
                                        clonePivotPose = toTransform(
                                            internal::getWorldTransform(*attachedStage, cloneKey, omni::physics::parse::ReadTime::defaultTime()));
                                    }
                                    else
                                    {
                                        // No authored clone prim: the anchor is the final world pose
                                        // of the exact target subtree root. Relocate each body by
                                        // anchor * inverse(source-root) so the subtree layout is
                                        // preserved and an off-origin source is not double-counted.
                                        // No transform -> co-locate on the source (deserialized) pose;
                                        // env-ids, when active, keep co-located copies collision-safe.
                                        PxTransform cloneTf;
                                        if (cloneTransformAt(mCloneTransforms, i, cloneTf))
                                            clonePivotPose = cloneTf * sourceRootPoseInv * clonePivotPose;
                                    }
                                    processReplicationData.rigidTransforms.push_back(std::make_pair(&actor, clonePivotPose));
                                }
                                else
                                {
                                    PxArticulationLink& link = (PxArticulationLink&)object;
                                    PxArticulationLink* rootLink = nullptr;
                                    PxArticulationReducedCoordinate& articulation = link.getArticulation();
                                    articulation.getLinks(&rootLink, 1);
                                    if (rootLink == &link)
                                    {
                                        PxTransform tr = clonePivotPose;
                                        if (hasClonePrim)
                                        {
                                            tr = toTransform(
                                                internal::getWorldTransform(*attachedStage, cloneKey, omni::physics::parse::ReadTime::defaultTime()));
                                        }
                                        else
                                        {
                                            // Absolute placement (see the rigid-body branch): the
                                            // anchor is the exact target subtree root's world pose;
                                            // relocate by anchor * inverse(source-root) so an
                                            // off-origin source is not double-counted.
                                            PxTransform cloneTf;
                                            if (cloneTransformAt(mCloneTransforms, i, cloneTf))
                                                tr = cloneTf * sourceRootPoseInv * tr;
                                        }
                                        processReplicationData.articulationTransforms.push_back(std::make_pair(&articulation, tr));
                                    }
                                }
                            }
                            const InternalActor* cloneActor = (InternalActor*)record.mInternalPtr;
                            const PxActor* clonePxActor = (PxActor*)record.mPtr;
                            // check name clone
                            if (clonePxActor->getName())
                            {
                                processReplicationData.actorNames.push_back(std::make_pair(&actor, attachedStage->textFor(newKey)));
                            }

                            // contact report
                            if (!contactReportMap.empty())
                            {
                                ContactPairsMap::const_iterator fit = contactReportMap.find((PxRigidActor*)clonePxActor);
                                if (fit != contactReportMap.end())
                                {
                                    if (fit->second.first == nullptr)
                                    {
                                        processReplicationData.contactReports.push_back(std::make_pair(&actor, fit->second.second));
                                    }
                                    else
                                    {
                                        CARB_LOG_ERROR("Replication does not support pair wise contact filtering, prim: %s", attachedStage->textFor(newKey));
                                    }
                                }
                            }

                            // local space velocities
                            const bool localSpaceVelocities = cloneActor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES;
                            ObjectId outId = kInvalidObjectId;
                            InternalActor* internalActor = nullptr;
                            if (objectType != PxConcreteType::eARTICULATION_LINK)
                            {
                                internalActor = new (memoryView.actorAllocatorIt.getAlignedMemory()) InternalActor(scene,
                                    (actor.is<PxRigidBody>() && !(actor.is<PxRigidBody>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) ? true : false,
                                    nullptr, localSpaceVelocities, cloneKey);
                                internalActor->mOwnsMemory = false;

                                outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTActor, &actor, internalActor, cloneKey) : db.addRecord(ePTActor, &actor, internalActor, cloneKey);
                                processReplicationData.objectDbEntries.push_back({ newKey, eBody, outId });
                            }
                            else
                            {
                                internalActor = new (memoryView.linkAllocatorIt.getAlignedMemory()) InternalLink(scene, nullptr, cloneKey);
                                internalActor->mOwnsMemory = false;

                                outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTLink, &actor, internalActor, cloneKey) : db.addRecord(ePTLink, &actor, internalActor, cloneKey);
                                processReplicationData.objectDbEntries.push_back({ newKey, eArticulationLink, outId });
                            }
                            processReplicationData.internalActors.push_back(internalActor);
                            internalActor->mActor = &actor;
                            internalActor->mScale = cloneActor->mScale;
                            // PhysxSurfaceVelocityAPI stores its derived state on InternalActor
                            // rather than on PhysX SDK objects, so mirrorHierarchy doesn't see it.
                            // Copy it across so cloned conveyor-style bodies actually drag.
                            // clonePivotPose is the per-target world pose captured above; the
                            // actor's own getGlobalPose() at this point still returns the
                            // source's pose (setGlobalPose runs in the later physxTask).
                            internalActor->copySurfaceVelocityState(*cloneActor, actor, clonePivotPose);
                            actor.userData = (void*)outId;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
                        {
                            PxArticulationReducedCoordinate& articulation = (PxArticulationReducedCoordinate&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newKey = resolveClone(record.mKey);
                            const InternalArticulation* cloneArticulation = (InternalArticulation*)record.mInternalPtr;
                            const PxArticulationReducedCoordinate* clonePxArticulation = (PxArticulationReducedCoordinate*)record.mPtr;

                            if (clonePxArticulation->getName())
                            {
                                processReplicationData.articulationNames.push_back(std::make_pair(&articulation, attachedStage->textFor(newKey)));
                            }

                            InternalArticulation* intArt = new (memoryView.articulationAllocatorIt.getAlignedMemory()) (InternalArticulation)(scene);
                            intArt->mOwnsMemory = false;

                            intArt->mEnableSelfCollision = cloneArticulation->mEnableSelfCollision;
                            intArt->mStaticRootBodyKey = cloneArticulation->mStaticRootBodyKey;
                            intArt->mAggregate = articulation.getAggregate();

                            const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTArticulation, &articulation, intArt, cloneKey) : db.addRecord(ePTArticulation, &articulation, intArt, cloneKey);
                            processReplicationData.objectDbEntries.push_back({ newKey, eArticulation, outId });
                            articulation.userData = (void*)(outId);
                        }
                        break;
                        case PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
                        {
                            PxArticulationJointReducedCoordinate& joint = (PxArticulationJointReducedCoordinate&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newKey = resolveClone(record.mKey);
                            const InternalJoint* cloneJoint = (InternalJoint*)record.mInternalPtr;
                            const PxArticulationJointReducedCoordinate* clonePxJoint = (PxArticulationJointReducedCoordinate*)record.mPtr;

                            InternalJoint* intJoint = new (memoryView.articulationJointAllocatorIt.getAlignedMemory()) InternalJoint();
                            intJoint->mOwnsMemory = false;

                            intJoint->copy(*cloneJoint);

                            const ObjectId jointObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTLinkJoint, &joint, intJoint, cloneKey) : db.addRecord(ePTLinkJoint, &joint, intJoint, cloneKey);
                            joint.userData = (void*)jointObjId;
                            processReplicationData.objectDbEntries.push_back({ newKey, eArticulationJoint, jointObjId });
                        }
                        break;
                        case PxConcreteType::eARTICULATION_TENDON_JOINT:
                        {
                            PxArticulationTendonJoint& tendonJoint = (PxArticulationTendonJoint&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newKey = resolveClone(record.mKey);
                            const InternalTendonAxis* cloneIntAxis = (InternalTendonAxis*)record.mInternalPtr;

                            InternalTendonAxis* intTendon = new (memoryView.tendonAxisAllocatorIt.getAlignedMemory()) InternalTendonAxis();
                            intTendon->mOwnsMemory = false;

                            intTendon->instanceName = cloneIntAxis->instanceName;

                            const ObjectId tendonObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTFixedTendonAxis, &tendonJoint, intTendon, cloneKey) : db.addRecord(ePTFixedTendonAxis, &tendonJoint, intTendon, cloneKey);

                            if (tendonJoint.getParent() == nullptr)
                                processReplicationData.objectDbEntries.push_back({ newKey, eTendonFixed, tendonObjId });
                            else
                                processReplicationData.objectDbEntries.push_back({ newKey, eTendonAxis, tendonObjId });
                            tendonJoint.userData = (void*)tendonObjId;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_ATTACHMENT:
                        {
                            PxArticulationAttachment& articulationAttachment = (PxArticulationAttachment&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newKey = resolveClone(record.mKey);
                            const InternalTendonAttachment* cloneIntAttachment = (InternalTendonAttachment*)record.mInternalPtr;

                            InternalTendonAttachment* internalAttachment = new (memoryView.tendonAttachmentAllocatorIt.getAlignedMemory()) InternalTendonAttachment();
                            internalAttachment->mOwnsMemory = false;

                            internalAttachment->globalPos = cloneIntAttachment->globalPos;
                            internalAttachment->initLength = cloneIntAttachment->initLength;
                            internalAttachment->instanceName = cloneIntAttachment->instanceName;

                            const ObjectId tendonObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTTendonAttachment, &articulationAttachment, internalAttachment, cloneKey) :
                                db.addRecord(ePTTendonAttachment, &articulationAttachment, internalAttachment, cloneKey);
                            processReplicationData.objectDbEntries.push_back({ newKey, eTendonAttachment, tendonObjId });
                            articulationAttachment.userData = (void*)tendonObjId;
                        }
                        break;
                        case PxConcreteType::eCONSTRAINT:
                        {
                            PxConstraint& constraint = (PxConstraint&)object;
                            PxU32 typeId;
                            PxJoint* joint = reinterpret_cast<PxJoint*>(constraint.getExternalReference(typeId));
                            if (joint && (typeId == PxConstraintExtIDs::eJOINT))
                            {
                                const size_t id = replicatorIdForPxBase(object);
                                const internal::InternalDatabase::Record& record = db.getRecords()[id];
                                newKey = resolveClone(record.mKey);

                                PxRigidActor* actor0 = nullptr;
                                PxRigidActor* actor1 = nullptr;
                                joint->getActors(actor0, actor1);
                                if (!actor0 || !actor1)
                                {
                                    CARB_LOG_WARN("Cloning joints %s without a body rel may cause issues, since the localPose wont be updated.", attachedStage->textFor(newKey));
                                }

                                const InternalJoint* cloneJoint = (InternalJoint*)record.mInternalPtr;
                                const PxJoint* clonePxJoint = (PxJoint*)record.mPtr;

                                InternalJoint* intJoint = new (memoryView.jointAllocatorIt.getAlignedMemory()) InternalJoint();
                                intJoint->mOwnsMemory = false;

                                if (clonePxJoint->getName())
                                {
                                    processReplicationData.jointNames.push_back(std::make_pair(joint, attachedStage->textFor(newKey)));
                                }

                                intJoint->copy(*cloneJoint);

                                const ObjectId jointObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTJoint, joint, intJoint, cloneKey) : db.addRecord(ePTJoint, joint, intJoint, cloneKey);
                                joint->userData = (void*)jointObjId;
                                processReplicationData.objectDbEntries.push_back({ newKey, eJoint, jointObjId });
                            }
                        }
                        break;
                        case PxConcreteType::eSHAPE:
                        {
                            PxShape& shape = (PxShape&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];

                            newKey = resolveClone(record.mKey);
                            const InternalShape* cloneShape = (InternalShape*)record.mInternalPtr;
                            const PxShape* clonePxShape = nullptr;
                            if (record.mType == ePTShape)
                            {
                                clonePxShape = (PxShape*)record.mPtr;
                            }
                            else if (record.mType == ePTCompoundShape)
                            {
                                PhysXCompoundShape* cShape = (PhysXCompoundShape*)record.mPtr;
                                if (cShape->getShapes().empty())
                                {
                                    CARB_LOG_ERROR("Compound Shape empty, failed to clone.");
                                    break;
                                }
                                clonePxShape = cShape->getShapes()[0];
                            }

                            // check new collision groups
                            const ObjectId newCollisionGroup =
                                getCollisionGroup(*attachedStage, newKey);

                            if (newCollisionGroup != kInvalidObjectId)
                            {
                                const uint32_t cg = convertToCollisionGroup(newCollisionGroup);
                                {
                                    PxFilterData fd = shape.getSimulationFilterData();
                                    convertCollisionGroupToPxFilterData(cg, fd);
                                    shape.setSimulationFilterData(fd);
                                }
                                {
                                    PxFilterData fd = shape.getQueryFilterData();
                                    convertCollisionGroupToPxFilterData(cg, fd);
                                    shape.setQueryFilterData(fd);
                                }
                            }

                            // check name clone
                            if (clonePxShape && clonePxShape->getName())
                            {
                                processReplicationData.shapeNames.push_back(std::make_pair(&shape, attachedStage->textFor(newKey)));
                            }

                            if (record.mType == ePTShape)
                            {
                                InternalShape* internalShape =
                                    new (memoryView.shapeAllocatorIt.getAlignedMemory()) InternalShape(scene, cloneShape->mScale, cloneShape->mMaterialId);
                                internalShape->mOwnsMemory = false;

                                internalShape->mMassInfo = cloneShape->mMassInfo;
                                const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTShape, &shape, internalShape, cloneKey) : db.addRecord(ePTShape, &shape, internalShape, cloneKey);
                                if (cloneShape->mMaterialId != kInvalidObjectId)
                                {
                                    if (!materialResolveEnabled)
                                    {
                                        InternalMaterial* intMat = (InternalMaterial*)db.getInternalTypedRecord(ePTMaterial, cloneShape->mMaterialId);
                                        if (intMat)
                                            processReplicationData.materialPairs.push_back(std::make_pair(intMat, outId));
                                    }
                                    else
                                    {
                                        newShapeShapeIdTable.push_back(std::make_pair(internalShape, outId));
                                    }
                                }
                                shape.userData = (void*)(outId);
                                processReplicationData.objectDbEntries.push_back({ newKey, eShape, outId });
                            }
                            else
                            {
                                CompoundShapeMap::iterator fit = compoundShapes.find(newKey);
                                if (fit == compoundShapes.end())
                                {
                                    InternalShape* internalShape =
                                        new (memoryView.shapeAllocatorIt.getAlignedMemory()) InternalShape(scene, cloneShape->mScale, cloneShape->mMaterialId);
                                    internalShape->mOwnsMemory = false;

                                    internalShape->mMassInfo = cloneShape->mMassInfo;

                                    CompoundShape* newShape = new CompoundShape(scene);
                                    const CompoundShape* oldShape = (const CompoundShape*)record.mPtr;
                                    newShape->mMassInfo = oldShape->mMassInfo;
                                    newShape->mMaterialId = oldShape->mMaterialId;

                                    std::vector<::physx::PxShape*>& cShapes = const_cast<std::vector<::physx::PxShape*>&>(newShape->getShapes());
                                    cShapes.push_back(&shape);

                                    const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTCompoundShape, newShape, internalShape, cloneKey) : db.addRecord(ePTCompoundShape, newShape, internalShape, cloneKey);
                                    if (cloneShape->mMaterialId != kInvalidObjectId)
                                    {
                                        if (!materialResolveEnabled)
                                        {
                                            // Defer the shared-material update to the serial post-pass (materialPairs).
                                            // Calling intMat->addShapeId() directly here races with the other replica
                                            // worker threads (parallelFor) on the shared source material's mShapeIds
                                            // vector, corrupting the heap (OMPE-97081).
                                            InternalMaterial* intMat = (InternalMaterial*)db.getInternalTypedRecord(ePTMaterial, cloneShape->mMaterialId);
                                            if (intMat)
                                                processReplicationData.materialPairs.push_back(std::make_pair(intMat, outId));
                                        }
                                        else
                                        {
                                            newShapeShapeIdTable.push_back(std::make_pair(internalShape, outId));
                                        }
                                    }
                                    shape.userData = (void*)(outId);
                                    processReplicationData.objectDbEntries.push_back({ newKey, eShape, outId });

                                    compoundShapes[newKey] = std::make_pair(newShape, outId);
                                }
                                else
                                {
                                    CompoundShape* newShape = fit->second.first;
                                    std::vector<::physx::PxShape*>& cShapes = const_cast<std::vector<::physx::PxShape*>&>(newShape->getShapes());
                                    cShapes.push_back(&shape);

                                    shape.userData = (void*)(fit->second.second);
                                }
                            }
                        }
                        break;
                        case PxConcreteType::eMATERIAL:
                        {
                            PxMaterial& material = (PxMaterial&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];

                            newKey = resolveClone(record.mKey);
                            const InternalMaterial* cloneMaterial = (InternalMaterial*)record.mInternalPtr;

                            InternalMaterial* internalMat = new (memoryView.materialAllocatorIt.getAlignedMemory()) InternalMaterial(cloneMaterial->mDensity);
                            internalMat->mOwnsMemory = false;

                            const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTMaterial, &material, internalMat, cloneKey) : db.addRecord(ePTMaterial, &material, internalMat, cloneKey);
                            material.userData = (void*)(outId);
                            processReplicationData.objectDbEntries.push_back({ newKey, eMaterial, outId });
                            oldMaterialNewMaterialTable.push_back({ id, internalMat, outId });
                        }
                        break;
                        case PxConcreteType::eARTICULATION_MIMIC_JOINT:
                        {
                            PxArticulationMimicJoint& mimicJoint = (PxArticulationMimicJoint&)object;
                            const size_t id = replicatorIdForPxBase(object);
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newKey = resolveClone(record.mKey);
                            const InternalMimicJoint* cloneIntMimicJoint = (InternalMimicJoint*)record.mInternalPtr;
                            usdparser::ObjectType usdObjectType = cloneIntMimicJoint->getObjectType();

                            InternalMimicJoint* intMimicJoint = new (memoryView.mimicJointAllocatorIt.getAlignedMemory()) InternalMimicJoint(*scene->getInternalScene(), mimicJoint, usdObjectType);
                            intMimicJoint->mOwnsMemory = false;
                            processReplicationData.mimicJointPairs.push_back(std::make_pair(scene->getInternalScene(), intMimicJoint));

                            const ObjectId mimicJointObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTMimicJoint, &mimicJoint, intMimicJoint, cloneKey) : db.addRecord(ePTMimicJoint, &mimicJoint, intMimicJoint, cloneKey);

                            processReplicationData.objectDbEntries.push_back({ newKey, usdObjectType, mimicJointObjId });
                            mimicJoint.userData = (void*)mimicJointObjId;
                        }
                        break;
                        default:
                            break;
                        }

                        if (newKey.valid())
                            processReplicationData.hierarchyStorage.second.addPrimSubtree(
                                std::string(attachedStage->textFor(newKey)), std::string(attachedStage->textFor(newReplicateKey)));
                    };

                    oldMaterialNewMaterialTable.clear();
                    newShapeShapeIdTable.clear();


                    if (i != 0)
                    {
                        ReplicatorMemoryView memoryView(replicatorMemory, i);
                        size_t recordsIndex = recordsStart + i * recordsOffset;
                        const uint32_t nbObjects = processReplicationData.collection->getNbObjects();
                        for (uint32_t o = 0; o < nbObjects; o++)
                        {
                            PxBase& object = processReplicationData.collection->getObject(o);
                            processObjectFn(object, recordsIndex, memoryView, processReplicationData);
                        }
                        CARB_ASSERT(recordsIndex == recordsStart + (i + 1) * recordsOffset);
                    }
                    else
                    {
                        size_t recordsIndex = 0;
                        const uint32_t nbObjects = processReplicationData.collection->getNbObjects();
                        // First process memory
                        MemoryDesc memoryDesc;
                        for (uint32_t o = 0; o < nbObjects; o++)
                        {
                            PxBase& object = processReplicationData.collection->getObject(o);
                            processObjectMemoryDescFn(object, memoryDesc);
                        }
                        replicatorMemory.initialize(memoryDesc, numReplications);
                        ReplicatorMemoryView memoryView(replicatorMemory, i);
                        for (uint32_t o = 0; o < nbObjects; o++)
                        {
                            PxBase& object = processReplicationData.collection->getObject(o);
                            processObjectFn(object, recordsIndex, memoryView, processReplicationData);
                        }
                        CARB_ASSERT(recordsIndex == 0);
                    }                    

                    // resolve material clone
                    for (const std::pair<InternalShape*, size_t>& shapePair : newShapeShapeIdTable)
                    {
                        InternalShape* shape = shapePair.first;
                        const size_t shapeId = shapePair.second;
                        const size_t oldMaterialId = shape->mMaterialId;
                        for (size_t k = 0; k < oldMaterialNewMaterialTable.size(); k++)
                        {
                            if (oldMaterialNewMaterialTable[k].oldMaterialId == oldMaterialId)
                            {
                                InternalMaterial* newIntMaterial = oldMaterialNewMaterialTable[k].newMaterial;
                                newIntMaterial->addShapeId(shapeId);
                                shape->mMaterialId = oldMaterialNewMaterialTable[k].newMaterialId;
                                break;
                            }
                        }
                    }
                };

                std::vector<ProcessReplicationData> replicationOutData(numReplications);
                ReplicatorMemory replicatorMemory;
                ITasking* tasking = carb::getCachedInterface<ITasking>();

                {
                    PHYSICS_PROFILE("PhysXReplicator::replicate::renameFnCb");
                    CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:renameFnCb");

                    for (uint32_t i = 0; i < numReplications; i++)
                    {
                        uint64_t newHierarchyPath = 0;
                        if (mCallback.hierarchyRenameFn)
                        {
                            newHierarchyPath = mCallback.hierarchyRenameFn(path, i, mCallback.userData);
                        }
                        replicationOutData[i].newReplicateKey =
                            newHierarchyPath ? omni::physics::parse::ObjectKey{ newHierarchyPath } : omni::physics::parse::ObjectKey{};

                        // We need i + 1, 0 is used for the first env parsed initially; the base
                        // offsets a second clone() on this attach past the earlier batches' env-ids
                        // so distinct environments never share one (see advanceReplicatorEnvIdBase).
                        // Caller-supplied LOGICAL ids override the positional numbering: the same
                        // caller id maps to the same runtime id (callerId + 1) in every batch, so a
                        // multi-call ClonePlan (one clone() per source row) keeps same-environment
                        // objects on one id — positional numbering would silently stop their contacts.
                        if (i < mCloneEnvIds.size())
                        {
                            // Caller ids are validated < 1<<24 at the ovphysx boundary (runtime id +1).
                            replicationOutData[i].envIdInt = mCloneEnvIds[i] + 1;
                        }
                        else
                        {
                            // Automatic positional numbering. Compute in 64-bit so the running base
                            // cannot wrap uint32, and guard against exhausting the PhysX id space
                            // (rather than passing an id setEnvironmentID would reject) — the clone
                            // then collides with all environments, but the exhaustion is not silent.
                            const uint64_t autoEnvId =
                                uint64_t(i) + 1u + attachedStage->getReplicatorEnvIdBase();
                            if (autoEnvId >= kMaxPhysxEnvironments)
                            {
                                CARB_LOG_WARN(
                                    "Replicator: automatic environment id %llu exceeds the PhysX limit "
                                    "(1<<24) on this attach; the clone will collide with all environments "
                                    "(env-id space exhausted).",
                                    (unsigned long long)autoEnvId);
                                replicationOutData[i].envIdInt = PX_INVALID_U32;
                            }
                            else
                            {
                                replicationOutData[i].envIdInt = uint32_t(autoEnvId);
                            }
                        }

                        // The scene-partition primvar can only exist on authored clone prims; when the
                        // caller supplied explicit ids, those win (prim-less clones never get here).
                        if (implicitEnvIdsUse && mCloneEnvIds.empty() &&
                            replicationOutData[i].newReplicateKey.valid())
                        {
                            {
                                if (const omni::physics::parse::IPhysicsSource* source = attachedStage->getSource())
                                {
                                    const omni::physics::parse::TokenId scenePartitionAttr =
                                        source->internToken(kScenePartitionPrimvar);
                                    omni::physics::parse::TokenId clonePartitionToken;
                                    if (getValue<omni::physics::parse::TokenId>(
                                            *attachedStage, replicationOutData[i].newReplicateKey,
                                            scenePartitionAttr, omni::physics::parse::ReadTime::defaultTime(),
                                            clonePartitionToken))
                                    {
                                        CARB_LOG_INFO("Registering scene partion token: %s",
                                                      std::string(source->tokenToString(clonePartitionToken)).c_str());
                                        const uint32_t envIdInt =
                                            attachedStage->registerEnvIdFromToken(clonePartitionToken);
                                        replicationOutData[i].envIdInt = envIdInt;
                                    }
                                }
                            }
                        }

                        // create the collection synchronously
                        void* nm = nullptr;
                        nm = (PxU8*)memblockAligned + alignedSize * i;
                        memcpy(nm, mirrorMemoryAligned, mirrorMemorySize);

                        PxCollection* collection1 = PxSerialization::createCollectionFromBinary(
                            nm, *physxSetup.getSerializationRegistry(), sharedCollection);

                        replicationOutData[i].collection = collection1;
                    }
                }

                {
                    PHYSICS_PROFILE("PhysXReplicator::replicate::proccessReplication");
                    CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:proccessReplication");
                    const uint32_t minReplications = 128;
                    const size_t preRecordsSize = db.getRecords().size();
                    processReplicationFn(0, false, 0, 0, replicationOutData[0], replicatorMemory);
                    const size_t postRecordsSize = db.getRecords().size();
                    const size_t recordsOffset = postRecordsSize - preRecordsSize;
                    const size_t newRecordsSize = postRecordsSize + recordsOffset * (numReplications - 1);
                    db.getRecords().resize(newRecordsSize);

                    const bool updateUSD = OmniPhysX::getInstance().getCachedSettings().updateToUsd &&
                        !(SimulationCallbacks::getSimulationCallbacks()->checkGlobalSimulationFlags(GlobalSimulationFlag::eTRANSFORMATION | GlobalSimulationFlag::eSKIP_WRITE));
                    const bool updateUSDVelocities = OmniPhysX::getInstance().getCachedSettings().updateVelocitiesToUsd;
                    bool forceSyncReplication = false;
                    if (updateUSD || updateUSDVelocities)
                    {
                        forceSyncReplication = true;
                        if (numReplications > minReplications)
                        {
                            CARB_LOG_WARN("Large amount of replications running inefficiently with USD updates enabled.");
                        }
                    }

                    if (forceSyncReplication || numReplications < minReplications)
                    {
                        for (uint32_t i = 1; i < numReplications; i++)
                        {
                            processReplicationFn(i, true, preRecordsSize, recordsOffset, replicationOutData[i], replicatorMemory);
                        }
                    }
                    else
                    {
                        // Pre-intern every clone ObjectKey this parallelFor dispatch will touch,
                        // serially, before any worker starts, mirroring resolveClone's key
                        // derivation. The concurrent reads inside processObjectFn are then pure,
                        // non-mutating intern-table reads: no thread can observe a keyFor()
                        // reallocation triggered by a sibling, so no lock is needed there.
                        //
                        // The same loop collects each clone key and its parent so their existence
                        // can be resolved in ONE batched source query below.
                        std::vector<omni::physics::parse::ObjectKey> existenceProbe;
                        std::unordered_set<omni::physics::parse::ObjectKey, omni::physics::parse::ObjectKey::Hash> existenceProbeSeen;
                        const omni::physics::parse::IPhysicsSource* preSource = attachedStage->getSource();
                        auto probeKey = [&](omni::physics::parse::ObjectKey k)
                        {
                            if (k.valid() && existenceProbeSeen.insert(k).second)
                                existenceProbe.push_back(k);
                        };
                        for (uint32_t i = 1; i < numReplications; i++)
                        {
                            const omni::physics::parse::ObjectKey newReplicateKeyI = replicationOutData[i].newReplicateKey;
                            PxCollection* collection = replicationOutData[i].collection;
                            const uint32_t nbObjects = collection->getNbObjects();
                            for (uint32_t o = 0; o < nbObjects; o++)
                            {
                                PxBase& object = collection->getObject(o);
                                const size_t id = replicatorIdForPxBase(object);
                                if (id == SIZE_MAX)
                                    continue;

                                const internal::InternalDatabase::Record& record = db.getRecords()[id];
                                const std::string clonePathStr = getNewObjectPath(attachedStage->textViewFor(record.mKey),
                                                                                  attachedStage->textViewFor(replicateKey),
                                                                                  attachedStage->textViewFor(newReplicateKeyI));
                                const omni::physics::parse::ObjectKey clonePathKey = attachedStage->keyFor(std::string_view(clonePathStr));
#if CARB_ASSERT_ENABLED
                                preInternedClonePaths.insert(clonePathKey);
#endif
                                if (preSource)
                                {
                                    // processObjectFn asks exists(cloneKey) per object and
                                    // InternalActor's nested-body walk asks exists(parent(cloneKey)).
                                    probeKey(clonePathKey);
                                    probeKey(preSource->getParent(clonePathKey));
                                }
                            }
                        }
#if CARB_ASSERT_ENABLED
                        preInternedClonePathsPtr = &preInternedClonePaths;
#endif

                        // Resolve every clone-path existence answer the parallel window will
                        // need in a single query. Backends that answer exists() from memory
                        // (USD) are unaffected; the ovstage source otherwise pays one blocking
                        // usd-path round trip per key, held under its own mutex, which turns
                        // the parallel dispatch into a serial convoy (hours for 1023 envs).
                        if (preSource && !existenceProbe.empty())
                        {
                            std::vector<bool> probeExists;
                            preSource->existsBatch(existenceProbe, probeExists);
                        }

                        auto&& computeFunc = [this, processReplicationFn, preRecordsSize, recordsOffset, &replicationOutData, &replicatorMemory](uint32_t batchIndex)
                        {
                            processReplicationFn(batchIndex, true, preRecordsSize, recordsOffset, replicationOutData[batchIndex], replicatorMemory);
                        };
                        tasking->parallelFor(uint32_t(1), numReplications, computeFunc);
                    }
                    CARB_ASSERT(db.getRecords().size() == newRecordsSize);
                }

                auto physxTask = tasking->addTask(Priority::eHigh, nullptr, [&] {
                    CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate::addCollections");
                    
                    const PxU32 size = PxU32(replicationOutData.size());
                    for (PxU32 i=0; i<size; i++)
                    {
                        const ProcessReplicationData& data = replicationOutData[i];

                        for (const std::pair<PxRigidActor*, PxTransform>& rp : data.rigidTransforms)
                        {
                            rp.first->setGlobalPose(rp.second);
                            if (useEnvIDs)
                            {
                                // Needs to be setup here, as its a non-thread safe call
                                // We need i + 1, 0 is used for the first env parsed initially
                                if (!rp.first->setEnvironmentID(data.envIdInt))
                                {
                                    CARB_LOG_WARN("Replicator: setEnvironmentID(%u) failed for a cloned "
                                                  "actor (id must be < 1<<24); it will collide with all "
                                                  "environments.", data.envIdInt);
                                }
                            }
                        }
                        for (const std::pair<PxArticulationReducedCoordinate*, PxTransform>& ap :
                             data.articulationTransforms)
                        {
                            ap.first->setRootGlobalPose(ap.second);
                            if (useEnvIDs)
                            {
                                PxAggregate* agg = ap.first->getAggregate();
                                if (agg)
                                {
                                    // Needs to be setup here, as its a non-thread safe call
                                    // We need i + 1, 0 is used for the first env parsed initially
                                    if (!agg->setEnvironmentID(data.envIdInt))
                                    {
                                        CARB_LOG_WARN("Replicator: setEnvironmentID(%u) failed for a cloned "
                                                      "articulation aggregate (id must be < 1<<24); it will "
                                                      "collide with all environments.", data.envIdInt);
                                    }
                                }
                            }
                        }
                        for (const std::pair<PxArticulationReducedCoordinate*, const char*>& an : data.articulationNames)
                        {
                            an.first->setName(an.second);
                        }
                        for (const std::pair<PxActor*, const char*>& an : data.actorNames)
                        {
                            an.first->setName(an.second);
                        }
                        for (const std::pair<PxJoint*, const char*>& an : data.jointNames)
                        {
                            an.first->setName(an.second);
                        }
                        for (const std::pair<PxShape*, const char*>& an : data.shapeNames)
                        {
                            an.first->setName(an.second);
                        }
                        for (const std::pair<InternalScene*, InternalMimicJoint*>& mmj : data.mimicJointPairs)
                        {
                            mmj.first->addMimicJoint(*mmj.second);
                        }

                        physXScene->addCollection(*data.collection);
                    }
                });

                auto nonPhysxTask = tasking->addTask(Priority::eHigh, nullptr, [&] {
                    CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:proccessReplicationData");
                    for (const ProcessReplicationData& data : replicationOutData)
                    {
                        for (InternalActor* ia : data.internalActors)
                        {
                            scene->getInternalScene()->addActor(*ia);
                        }
                        for (const std::pair<PxRigidActor*, float>& cd : data.contactReports)
                        {
                            contactReportMap.insert(std::make_pair(cd.first, std::make_pair(nullptr, cd.second)));
                        }
                        for (const ObjectDbEntry& dbe : data.objectDbEntries)
                        {
                            objectDb->findOrCreateEntryWithoutHierarchyStorage(dbe.key, dbe.category, dbe.newEntryId);
                        }
                        for (const std::pair<InternalMaterial*, ObjectId>& p : data.materialPairs)
                        {
                            p.first->addShapeId(p.second);
                        }
                        // fixup the schemAPIFlags
                        for (const std::pair<omni::physics::parse::ObjectKey, uint32_t>& p : schemaAPIFlagsStorage)
                        {
                            const std::string newPathStr = getNewObjectPath(attachedStage->textViewFor(p.first),
                                                                            attachedStage->textViewFor(replicateKey),
                                                                            attachedStage->textViewFor(data.newReplicateKey));
                            const omni::physics::parse::ObjectKey newSchemaKey = attachedStage->keyFor(std::string_view(newPathStr));
                            objectDb->setSchemaAPI(newSchemaKey, p.second);
                        }
                        // merge the hierarchy storage
                        objectDb->getPrimHierarchyStorage().mergeHierarchyStorage(
                            data.hierarchyStorage.first, data.hierarchyStorage.second);
                    }
                });

                // main thread work
                if (replicatorMemory.actorAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.actorAllocator.getMemory());
                if (replicatorMemory.linkAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.linkAllocator.getMemory());
                if (replicatorMemory.articulationAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.articulationAllocator.getMemory());
                if (replicatorMemory.articulationJointAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.articulationJointAllocator.getMemory());
                if (replicatorMemory.jointAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.jointAllocator.getMemory());
                if (replicatorMemory.shapeAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.shapeAllocator.getMemory());
                if (replicatorMemory.materialAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.materialAllocator.getMemory());
                if (replicatorMemory.tendonAxisAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.tendonAxisAllocator.getMemory());
                if (replicatorMemory.tendonAttachmentAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.tendonAttachmentAllocator.getMemory());
                if (replicatorMemory.mimicJointAllocator.getMemory())
                    attachedStage->addReplicatorMemoryBlock(replicatorMemory.mimicJointAllocator.getMemory());

                physxTask.wait();
                nonPhysxTask.wait();

            }

            // Advance the stage's explicit-env-id base so a later clone() batch on this attach
            // numbers its copies past these, not from 1 again (only when env-ids actually engaged).
            // With caller-supplied logical ids, raise the base past the highest assigned runtime id
            // instead, so a later positional batch cannot alias an explicitly-placed environment
            // (explicit batches may alias each other on purpose — that is the composition feature).
            if (useEnvIDs)
            {
                if (mCloneEnvIds.empty())
                {
                    attachedStage->advanceReplicatorEnvIdBase(numReplications);
                }
                else
                {
                    uint32_t maxCallerId = 0;
                    for (size_t k = 0; k < mCloneEnvIds.size() && k < numReplications; ++k)
                        maxCallerId = std::max(maxCallerId, mCloneEnvIds[k]);
                    attachedStage->raiseReplicatorEnvIdBase(maxCallerId + 1);
                }
            }
            return true;
        }

        void PhysXReplicator::clear()
        {
        }

        bool registerReplicator(AttachHandle attachHandle, const IReplicatorCallback& callback)
        {
            return OmniPhysX::getInstance().registerReplicator(attachHandle, callback);
        }

        void unregisterReplicator(AttachHandle attachHandle)
        {
            OmniPhysX::getInstance().unregisterReplicator(attachHandle);
        }

        bool replicate(AttachHandle attachHandle, uint64_t path, uint32_t numReplications, bool useEnvIds)
        {
            // No replicator on this attach is a query answer, not a resolution failure -- that is
            // what isReplicatorStage() reports -- so it stays a quiet false. The diagnostic for an
            // unresolvable handle (ADR-0016 Decision 3) is raised one level down, where the attach
            // is actually resolved.
            PhysXReplicator* replicator = OmniPhysX::getInstance().getReplicator(attachHandle);
            if (replicator)
            {
                return replicator->replicate(attachHandle, path, numReplications, useEnvIds);
            }
            return false;
        }


        void isReplicatorStage(AttachHandle attachHandle, bool& replicated)
        {
            replicated = false;

            PhysXReplicator* replicator = OmniPhysX::getInstance().getReplicator(attachHandle);
            if (replicator)
            {
                replicated = true;
            }
        }
    }
}

void fillInterface(omni::physx::IPhysxReplicator& iface)
{
    iface.registerReplicator = omni::physx::registerReplicator;
    iface.unregisterReplicator = omni::physx::unregisterReplicator;
    iface.replicate = omni::physx::replicate;
    iface.isReplicatorStage = omni::physx::isReplicatorStage;
}
