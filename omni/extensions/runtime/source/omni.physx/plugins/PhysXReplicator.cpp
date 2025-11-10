// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

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
#include "usdInterface/UsdInterface.h"
#include <common/foundation/TypeCast.h>
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
using namespace pxr;
using namespace carb::tasking;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

using CompoundShapeMap = std::unordered_map<SdfPath, std::pair<CompoundShape*, size_t>, SdfPath::Hash>;

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
            : mCallback(cb), mFabricReplication(false)
        {

        }

        PhysXReplicator::~PhysXReplicator()
        {
            clear();
        }

        void PhysXReplicator::attach(uint64_t stageId, PhysXUsdPhysicsInterface* usdPhysicsInt, bool attachStage)
        {
            mExcludePathSet.clear();

            // Full parse about to happen, we need to exclude paths defined by the callback
            if (mCallback.replicationAttachFn)
            {
                uint32_t numPaths = 0;
                uint64_t* paths = nullptr;
                mCallback.replicationAttachFn(stageId, numPaths, paths, mCallback.userData);
                if (numPaths && paths)
                {
                    for (uint32_t i = 0; i < numPaths; i++)
                    {
                        const pxr::SdfPath path = intToPath(paths[i]);
                        mExcludePathSet.insert(path);
                    }
                }
            }
            usdparser::UsdLoad::getUsdLoad()->attachReplicator(stageId, &getPhysXUsdPhysicsInterface(), mExcludePathSet, attachStage);

            // Full parse is done, we can start to replicate notify the attach end
            if (mCallback.replicationAttachEndFn)
            {
                mCallback.replicationAttachEndFn(stageId, mCallback.userData);
            }
        }

        SdfPath getNewObjectPath(const SdfPath& pathToUpdate, const SdfPath& replicatePath, const SdfPath& newReplicatePath)
        {
            return pathToUpdate.ReplacePrefix(replicatePath, newReplicatePath);
        }        

        bool PhysXReplicator::replicate(uint64_t stageId,
                                        uint64_t path,
                                        uint32_t numReplications,
                                        bool useEnvIDsIn, bool useFabricReplicate)
        {
            CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate");
            if (numReplications == 0)
                return false;

            PhysXScene* scene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(stageId);
            if (!scene)
            {
                CARB_LOG_ERROR("PhysX Scene not found for replication. StageId: %llu", stageId);
                return false;
            }

            AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(stageId);
            if (!attachedStage)
            {
                CARB_LOG_ERROR("Attached Stage not found for replication. StageId: %llu", stageId);
                return false;
            }
            
            omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
            omni::fabric::StageReaderWriterId stageInProgress = iStageReaderWriter->get(attachedStage->getStageId());                
            omni::fabric::StageReaderWriter stageRw(stageInProgress);

            OmniPhysX& omniPhysX = OmniPhysX::getInstance();
            pxr::UsdStageWeakPtr stage = attachedStage->getStage();
            ObjectDb* objectDb = attachedStage->getObjectDatabase();
            std::unordered_set<void*> physxPtrs;

            const UsdPrim topPrim = stage->GetPrimAtPath(intToPath(path));
            if (!topPrim)
            {
                CARB_LOG_ERROR("Prim to replicate not found in USD stage. StageId: %llu", stageId);
                return false;
            }

            mFabricReplication = useFabricReplicate;

            PxScene* physXScene = scene->getScene();
            const bool gpuDynamics = physXScene->getFlags() & PxSceneFlag::eENABLE_GPU_DYNAMICS;
            const bool gpuBroadphase = physXScene->getBroadPhaseType() == PxBroadPhaseType::eGPU;
            const bool useEnvIDs = gpuDynamics && gpuBroadphase && useEnvIDsIn; // PT: TODO: or is gpuBroadphase
                                                                                // enough?

            if (useEnvIDsIn && !gpuDynamics)
            {
                CARB_LOG_WARN("EnvIds requested but gpu dynamic is disabled.");
            }
            if (useEnvIDsIn && !gpuBroadphase)
            {
                CARB_LOG_WARN("EnvIds requested but gpu broadphase is not set.");
            }

            // setup env Ids for initial parsing to avoid scene re-insertion
            attachedStage->setUseReplicatorEnvIds(useEnvIDs);

            // parse the source
            {
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate::source");
                PHYSICS_PROFILE("PhysXReplicator::replicate::source");
                // Block the USD notification to get recursive callback
                UsdLoad::getUsdLoad()->blockUSDUpdate(true);

                omni::physics::schema::UsdPrimMap primMap;
                primMap[intToPath(path)] = topPrim;
                omni::physics::schema::PrimIteratorMapRange primIteratorMap(primMap);
                loadPhysicsFromPrimitive(*attachedStage, primIteratorMap);
                // proccess batched changes
                attachedStage->getPhysXPhysicsInterface()->finishSetup(*attachedStage);

                UsdLoad::getUsdLoad()->blockUSDUpdate(false);
            }

            // disable envIds usage after first parse
            attachedStage->setUseReplicatorEnvIds(false);

            if (useFabricReplicate && !useEnvIDs)
            {
                // update the collision groups, this is needed right now
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate::fabricUpdateCollisionGroups");
                updateFabricCollisionGroups(*attachedStage);
            }

            internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

            // If we clone materials, we need to resolve the materials in a second pass
            bool materialResolveEnabled = false;

            std::vector<std::pair<SdfPath, uint32_t>> schemaAPIFlagsStorage;

            const pxr::UsdPrimRange range(topPrim, pxr::UsdTraverseInstanceProxies());
            {
                PHYSICS_PROFILE("PhysXReplicator::replicate::traverseTypes");
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:traverseTypes");
                for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
                {
                    const pxr::UsdPrim& prim = *iter;

                    if (!prim)
                        continue;

                    const SdfPath traversePrimPath = prim.GetPrimPath();

                    const ObjectIdMap* entries = objectDb->getEntries(traversePrimPath);
                    if (entries)
                    {
                        ObjectIdMap::const_iterator it = entries->begin();
                        while (it != entries->end())
                        {
                            // filter allowed objects
                            if (it->first == eShape || it->first == eBody || it->first == eJoint || it->first == eArticulationLink
                                || it->first == eArticulation || it->first == eTendons || it->first == eMaterial
                                || it->first == eFilteredPair || it->first == eArticulationRootJoint
                                || it->first == eMimicJointRotX || it->first == eMimicJointRotY || it->first == eMimicJointRotZ)
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

                                CARB_LOG_ERROR("Replication of this type is not supported: %d, prim path: %s", it->first.getData(), record.mPath.GetText());
                            }
                            it++;
                        }
                    }

                    const uint64_t schemaFlags = objectDb->getSchemaAPIs(traversePrimPath);
                    if (schemaFlags)
                    {
                        schemaAPIFlagsStorage.push_back(std::make_pair(traversePrimPath, schemaFlags));
                    }
                }
            }

            {
                PHYSICS_PROFILE("PhysXReplicator::replicate::replication");
                CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:replication");
                void* mirrorMemory = nullptr;
                uint32_t mirrorMemorySize = 0;
                PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
                PxCollection* sharedCollection = nullptr;
                ContactPairsMap& contactReportMap = scene->getContactReport()->getContactPairsMap();
                mirrorHierarchy(*scene->getScene(), mirrorMemory, mirrorMemorySize, *physxSetup.getSerializationRegistry(), sharedCollection, physxPtrs);
                const SdfPath replicatePath = intToPath(path);

                // allocate contiguos memory
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
                    pxr::SdfPath path;
                    ObjectCategory category;
                    ObjectId newEntryId;
                };

                struct ProcessReplicationData
                {
                    pxr::SdfPath newReplicatePath;
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
                    std::pair<pxr::SdfPath, PrimHierarchyStorage> hierarchyStorage;
                    std::vector<std::pair<InternalMaterial*, ObjectId>> materialPairs;
                    std::vector<std::pair<InternalScene*, InternalMimicJoint*>> mimicJointPairs;
                };

                auto processReplicationFn = [this, memblockAligned, alignedSize, mirrorMemoryAligned, mirrorMemorySize, path,
                    stageId, &attachedStage, &db, &physxSetup, sharedCollection, scene, objectDb,
                    materialResolveEnabled, numReplications, &contactReportMap, replicatePath, useFabricReplicate, &stageRw](
                        uint32_t i, bool recordsReady, size_t recordsStart, size_t recordsOffset,
                        ProcessReplicationData& processReplicationData, ReplicatorMemory& replicatorMemory) -> void
                {
                    const SdfPath newReplicatePath = processReplicationData.newReplicatePath; 

                    // process hierarchy
                    processReplicationData.hierarchyStorage.first = newReplicatePath;

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

                    auto processObjectFn = [this, stageId, &attachedStage, &compoundShapes, &db, scene, objectDb,
                        materialResolveEnabled, &oldMaterialNewMaterialTable, &newShapeShapeIdTable,
                                            &contactReportMap, replicatePath, newReplicatePath, useFabricReplicate,
                                            &stageRw](PxBase& object, size_t& recordsIndex,
                                                      ReplicatorMemoryView& memoryView,
                                                      ProcessReplicationData& processReplicationData) -> void
                    {
                        SdfPath newSdfPath = SdfPath();
                        const PxType objectType = object.getConcreteType();
                        switch (objectType)
                        {
                        case PxConcreteType::eRIGID_STATIC:
                        case PxConcreteType::eRIGID_DYNAMIC:
                        case PxConcreteType::eARTICULATION_LINK:
                        {
                            PxRigidActor& actor = (PxRigidActor&)object;
                            const size_t id = (size_t)actor.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);

                            UsdPrim newPrim = UsdPrim();

                            if (!useFabricReplicate)
                            {
                                newPrim = attachedStage->getStage()->GetPrimAtPath(newSdfPath);
                                if (newPrim)
                                {
                                    if (objectType != PxConcreteType::eARTICULATION_LINK)
                                    {
                                        const GfMatrix4d mat = UsdGeomXform(newPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
                                        const GfTransform gftr(mat);
                                        const PxTransform tr = toPhysX(gftr);

                                        processReplicationData.rigidTransforms.push_back(std::make_pair(&actor, tr));
                                    }
                                    else
                                    {
                                        PxArticulationLink& link = (PxArticulationLink&)object;
                                        PxArticulationLink* rootLink = nullptr;
                                        PxArticulationReducedCoordinate& articulation = link.getArticulation();
                                        articulation.getLinks(&rootLink, 1);
                                        if (rootLink == &link)
                                        {
                                            const GfMatrix4d mat = UsdGeomXform(newPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
                                            const GfTransform gftr(mat);
                                            const PxTransform tr = toPhysX(gftr);

                                            processReplicationData.articulationTransforms.push_back(std::make_pair(&articulation, tr));
                                        }
                                    }
                                }
                                else
                                {
                                    CARB_LOG_ERROR("Prim to replicate path not found. StageId: %llu, New prim path: %s, old prim path: %s, replicate path %s, newReplicatePath %s", stageId, newSdfPath.GetText(),
                                        record.mPath.GetText(), replicatePath.GetText(), newReplicatePath.GetText());
                                }

                            }
                            else
                            {
                                if (objectType != PxConcreteType::eARTICULATION_LINK)
                                {
                                    const omni::fabric::Token fabricTransform("omni:fabric:worldMatrix");
                                    const pxr::GfMatrix4d* worldPose = stageRw.getAttributeRd<pxr::GfMatrix4d>(
                                        omni::fabric::asInt(newSdfPath), fabricTransform);
                                    if (worldPose)
                                    {
                                        const GfTransform gftr(*worldPose);
                                        const PxTransform tr = toPhysX(gftr);

                                        processReplicationData.rigidTransforms.push_back(std::make_pair(&actor, tr));
                                    }
                                    else
                                    {
                                        CARB_LOG_ERROR("Expecting omni:fabric:worldMatrix on a prim path: %s", newSdfPath.GetText());
                                    }
                                }
                                else
                                {
                                    PxArticulationLink& link = (PxArticulationLink&)object;
                                    PxArticulationLink* rootLink = nullptr;
                                    PxArticulationReducedCoordinate& articulation = link.getArticulation();
                                    articulation.getLinks(&rootLink, 1);
                                    if (rootLink == &link)
                                    {
                                        const omni::fabric::Token fabricTransform("omni:fabric:worldMatrix");
                                        const pxr::GfMatrix4d* worldPose = stageRw.getAttributeRd<pxr::GfMatrix4d>(
                                            omni::fabric::asInt(newSdfPath), fabricTransform);
                                        if (worldPose)
                                        {
                                            const GfTransform gftr(*worldPose);
                                            const PxTransform tr = toPhysX(gftr);

                                            processReplicationData.articulationTransforms.push_back(std::make_pair(&articulation, tr));
                                        }
                                        else
                                        {
                                            CARB_LOG_ERROR("Expecting omni:fabric:worldMatrix on a prim path: %s", newSdfPath.GetText());
                                        }
                                    }
                                }
                            }

                            const InternalActor* cloneActor = (InternalActor*)record.mInternalPtr;
                            const PxActor* clonePxActor = (PxActor*)record.mPtr;
                            // check name clone
                            if (clonePxActor->getName())
                            {
                                processReplicationData.actorNames.push_back(std::make_pair(&actor, newSdfPath.GetText()));
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
                                        CARB_LOG_ERROR("Replication does not support pair wise contact filtering, prim: %s", newSdfPath.GetText());
                                    }
                                }
                            }

                            // local space velocities
                            const bool localSpaceVelocities = cloneActor->mFlags & InternalActorFlag::eLOCALSPACE_VELOCITIES;
                            ObjectId outId = kInvalidObjectId;
                            InternalActor* internalActor = nullptr;
                            if (objectType != PxConcreteType::eARTICULATION_LINK)
                            {
                                internalActor = new (memoryView.actorAllocatorIt.getAlignedMemory()) InternalActor(scene, newSdfPath, newPrim,
                                    (actor.is<PxRigidBody>() && !(actor.is<PxRigidBody>()->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC)) ? true : false,
                                    nullptr, localSpaceVelocities);
                                internalActor->mOwnsMemory = false;

                                outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTActor, &actor, internalActor, newSdfPath) : db.addRecord(ePTActor, &actor, internalActor, newSdfPath);
                                processReplicationData.objectDbEntries.push_back({ newSdfPath, eBody, outId });
                            }
                            else
                            {
                                internalActor = new (memoryView.linkAllocatorIt.getAlignedMemory()) InternalLink(scene, newSdfPath, newPrim, nullptr);
                                internalActor->mOwnsMemory = false;

                                outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTLink, &actor, internalActor, newSdfPath) : db.addRecord(ePTLink, &actor, internalActor, newSdfPath);
                                processReplicationData.objectDbEntries.push_back({ newSdfPath, eArticulationLink, outId });
                            }
                            processReplicationData.internalActors.push_back(internalActor);
                            internalActor->mActor = &actor;
                            internalActor->mScale = cloneActor->mScale;
                            actor.userData = (void*)outId;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_REDUCED_COORDINATE:
                        {
                            PxArticulationReducedCoordinate& articulation = (PxArticulationReducedCoordinate&)object;
                            const size_t id = (size_t)articulation.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
                            const InternalArticulation* cloneArticulation = (InternalArticulation*)record.mInternalPtr;
                            const PxArticulationReducedCoordinate* clonePxArticulation = (PxArticulationReducedCoordinate*)record.mPtr;

                            if (clonePxArticulation->getName())
                            {
                                processReplicationData.articulationNames.push_back(std::make_pair(&articulation, newSdfPath.GetText()));
                            }

                            InternalArticulation* intArt = new (memoryView.articulationAllocatorIt.getAlignedMemory()) (InternalArticulation)(scene);
                            intArt->mOwnsMemory = false;

                            intArt->mEnableSelfCollision = cloneArticulation->mEnableSelfCollision;
                            intArt->mStaticRootBody = cloneArticulation->mStaticRootBody;
                            intArt->mAggregate = articulation.getAggregate();

                            const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTArticulation, &articulation, intArt, newSdfPath) : db.addRecord(ePTArticulation, &articulation, intArt, newSdfPath);
                            processReplicationData.objectDbEntries.push_back({ newSdfPath, eArticulation, outId });
                            articulation.userData = (void*)(outId);
                        }
                        break;
                        case PxConcreteType::eARTICULATION_JOINT_REDUCED_COORDINATE:
                        {
                            PxArticulationJointReducedCoordinate& joint = (PxArticulationJointReducedCoordinate&)object;
                            const size_t id = (size_t)joint.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
                            const InternalJoint* cloneJoint = (InternalJoint*)record.mInternalPtr;
                            const PxArticulationJointReducedCoordinate* clonePxJoint = (PxArticulationJointReducedCoordinate*)record.mPtr;

                            InternalJoint* intJoint = new (memoryView.articulationJointAllocatorIt.getAlignedMemory()) InternalJoint();
                            intJoint->mOwnsMemory = false;

                            // copy values
                            intJoint->copy(*cloneJoint);

                            const ObjectId jointObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTLinkJoint, &joint, intJoint, newSdfPath) : db.addRecord(ePTLinkJoint, &joint, intJoint, newSdfPath);
                            joint.userData = (void*)jointObjId;
                            processReplicationData.objectDbEntries.push_back({ newSdfPath, eArticulationJoint, jointObjId });
                        }
                        break;
                        case PxConcreteType::eARTICULATION_TENDON_JOINT:
                        {
                            PxArticulationTendonJoint& tendonJoint = (PxArticulationTendonJoint&)object;
                            const size_t id = (size_t)tendonJoint.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
                            const InternalTendonAxis* cloneIntAxis = (InternalTendonAxis*)record.mInternalPtr;

                            InternalTendonAxis* intTendon = new (memoryView.tendonAxisAllocatorIt.getAlignedMemory()) InternalTendonAxis();
                            intTendon->mOwnsMemory = false;

                            // copy int data
                            intTendon->instanceName = cloneIntAxis->instanceName;

                            const ObjectId tendonObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTFixedTendonAxis, &tendonJoint, intTendon, newSdfPath) : db.addRecord(ePTFixedTendonAxis, &tendonJoint, intTendon, newSdfPath);

                            if (tendonJoint.getParent() == nullptr)
                                processReplicationData.objectDbEntries.push_back({ newSdfPath, eTendonFixed, tendonObjId });
                            else
                                processReplicationData.objectDbEntries.push_back({ newSdfPath, eTendonAxis, tendonObjId });
                            tendonJoint.userData = (void*)tendonObjId;
                        }
                        break;
                        case PxConcreteType::eARTICULATION_ATTACHMENT:
                        {
                            PxArticulationAttachment& articulationAttachment = (PxArticulationAttachment&)object;
                            const size_t id = (size_t)articulationAttachment.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
                            const InternalTendonAttachment* cloneIntAttachment = (InternalTendonAttachment*)record.mInternalPtr;

                            InternalTendonAttachment* internalAttachment = new (memoryView.tendonAttachmentAllocatorIt.getAlignedMemory()) InternalTendonAttachment();
                            internalAttachment->mOwnsMemory = false;

                            // copy int data
                            internalAttachment->globalPos = cloneIntAttachment->globalPos;
                            internalAttachment->initLength = cloneIntAttachment->initLength;
                            internalAttachment->instanceName = cloneIntAttachment->instanceName;

                            const ObjectId tendonObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTTendonAttachment, &articulationAttachment, internalAttachment, newSdfPath) :
                                db.addRecord(ePTTendonAttachment, &articulationAttachment, internalAttachment, newSdfPath);
                            processReplicationData.objectDbEntries.push_back({ newSdfPath, eTendonAttachment, tendonObjId });
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
                                const size_t id = (size_t)joint->userData;
                                const internal::InternalDatabase::Record& record = db.getRecords()[id];
                                newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);

                                PxRigidActor* actor0 = nullptr;
                                PxRigidActor* actor1 = nullptr;
                                joint->getActors(actor0, actor1);
                                if (!actor0 || !actor1)
                                {
                                    CARB_LOG_WARN("Cloning joints %s without a body rel may cause issues, since the localPose wont be updated.", newSdfPath.GetText());
                                }

                                const InternalJoint* cloneJoint = (InternalJoint*)record.mInternalPtr;
                                const PxJoint* clonePxJoint = (PxJoint*)record.mPtr;

                                InternalJoint* intJoint = new (memoryView.jointAllocatorIt.getAlignedMemory()) InternalJoint();
                                intJoint->mOwnsMemory = false;

                                if (clonePxJoint->getName())
                                {
                                    processReplicationData.jointNames.push_back(std::make_pair(joint, newSdfPath.GetText()));
                                }

                                // copy values
                                intJoint->copy(*cloneJoint);

                                const ObjectId jointObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTJoint, joint, intJoint, newSdfPath) : db.addRecord(ePTJoint, joint, intJoint, newSdfPath);
                                joint->userData = (void*)jointObjId;
                                processReplicationData.objectDbEntries.push_back({ newSdfPath, eJoint, jointObjId });
                            }
                        }
                        break;
                        case PxConcreteType::eSHAPE:
                        {
                            PxShape& shape = (PxShape&)object;
                            const size_t id = (size_t)shape.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];

                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
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
                            const ObjectId newCollisionGroup = getCollisionGroup(*attachedStage, newSdfPath);

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
                                processReplicationData.shapeNames.push_back(std::make_pair(&shape, newSdfPath.GetText()));
                            }

                            if (record.mType == ePTShape)
                            {
                                InternalShape* internalShape =
                                    new (memoryView.shapeAllocatorIt.getAlignedMemory()) InternalShape(scene, cloneShape->mScale, cloneShape->mMaterialId);
                                internalShape->mOwnsMemory = false;

                                internalShape->mMassInfo = cloneShape->mMassInfo;
                                const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTShape, &shape, internalShape, newSdfPath) : db.addRecord(ePTShape, &shape, internalShape, newSdfPath);
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
                                processReplicationData.objectDbEntries.push_back({ newSdfPath, eShape, outId });
                            }
                            else
                            {
                                CompoundShapeMap::iterator fit = compoundShapes.find(newSdfPath);
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

                                    const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTCompoundShape, newShape, internalShape, newSdfPath) : db.addRecord(ePTCompoundShape, newShape, internalShape, newSdfPath);
                                    if (cloneShape->mMaterialId != kInvalidObjectId)
                                    {
                                        if (!materialResolveEnabled)
                                        {
                                            InternalMaterial* intMat = (InternalMaterial*)db.getInternalTypedRecord(ePTMaterial, cloneShape->mMaterialId);
                                            intMat->addShapeId(outId);
                                        }
                                        else
                                        {
                                            newShapeShapeIdTable.push_back(std::make_pair(internalShape, outId));
                                        }
                                    }
                                    shape.userData = (void*)(outId);
                                    processReplicationData.objectDbEntries.push_back({ newSdfPath, eShape, outId });

                                    compoundShapes[newSdfPath] = std::make_pair(newShape, outId);
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
                            const size_t id = (size_t)material.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];

                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
                            const InternalMaterial* cloneMaterial = (InternalMaterial*)record.mInternalPtr;

                            InternalMaterial* internalMat = new (memoryView.materialAllocatorIt.getAlignedMemory()) InternalMaterial(cloneMaterial->mDensity);
                            internalMat->mOwnsMemory = false;

                            const ObjectId outId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTMaterial, &material, internalMat, newSdfPath) : db.addRecord(ePTMaterial, &material, internalMat, newSdfPath);
                            material.userData = (void*)(outId);
                            processReplicationData.objectDbEntries.push_back({ newSdfPath, eMaterial, outId });
                            oldMaterialNewMaterialTable.push_back({ id, internalMat, outId });
                        }
                        break;
                        case PxConcreteType::eARTICULATION_MIMIC_JOINT:
                        {
                            PxArticulationMimicJoint& mimicJoint = (PxArticulationMimicJoint&)object;
                            const size_t id = (size_t)mimicJoint.userData;
                            const internal::InternalDatabase::Record& record = db.getRecords()[id];
                            newSdfPath = getNewObjectPath(record.mPath, replicatePath, newReplicatePath);
                            const InternalMimicJoint* cloneIntMimicJoint = (InternalMimicJoint*)record.mInternalPtr;
                            usdparser::ObjectType usdObjectType = cloneIntMimicJoint->getObjectType();

                            InternalMimicJoint* intMimicJoint = new (memoryView.mimicJointAllocatorIt.getAlignedMemory()) InternalMimicJoint(*scene->getInternalScene(), mimicJoint, usdObjectType);
                            intMimicJoint->mOwnsMemory = false;
                            processReplicationData.mimicJointPairs.push_back(std::make_pair(scene->getInternalScene(), intMimicJoint));

                            const ObjectId mimicJointObjId = recordsIndex ? db.addRecordAtIndex(recordsIndex++, ePTMimicJoint, &mimicJoint, intMimicJoint, newSdfPath) : db.addRecord(ePTMimicJoint, &mimicJoint, intMimicJoint, newSdfPath);

                            processReplicationData.objectDbEntries.push_back({ newSdfPath, usdObjectType, mimicJointObjId });
                            mimicJoint.userData = (void*)mimicJointObjId;
                        }
                        break;
                        default:
                            break;
                        }

                        if (newSdfPath != SdfPath())
                            processReplicationData.hierarchyStorage.second.addPrimSubtree(newSdfPath, newReplicatePath);
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
                        replicationOutData[i].newReplicatePath = newHierarchyPath ? intToPath(newHierarchyPath) : SdfPath();

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
                        auto&& computeFunc = [this, processReplicationFn, preRecordsSize, recordsOffset, &replicationOutData, &replicatorMemory](uint32_t batchIndex)
                        {
                            processReplicationFn(batchIndex, true, preRecordsSize, recordsOffset, replicationOutData[batchIndex], replicatorMemory);
                        };                        
                        tasking->parallelFor(uint32_t(1), numReplications, computeFunc);
                    }
                    CARB_ASSERT(db.getRecords().size() == newRecordsSize);
                }

                auto physxTask = tasking->addTask(Priority::eHigh, nullptr, [&] {
                    PHYSICS_PROFILE("PhysXReplicator::replicate::addCollections");
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
                                rp.first->setEnvironmentID(i + 1);
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
                                    agg->setEnvironmentID(i + 1);
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
                    PHYSICS_PROFILE("PhysXReplicator::replicate::proccessReplicationData");
                    CARB_PROFILE_ZONE(0, "PhysXReplicator::replicate:proccessReplicationData");
                    for (const ProcessReplicationData& data : replicationOutData)
                    {
                        for (InternalActor* ia : data.internalActors)
                        {
                            scene->getInternalScene()->mActors.push_back(ia);
                        }
                        for (const std::pair<PxRigidActor*, float>& cd : data.contactReports)
                        {
                            contactReportMap.insert(std::make_pair(cd.first, std::make_pair(nullptr, cd.second)));
                        }
                        for (const ObjectDbEntry& dbe : data.objectDbEntries)
                        {
                            objectDb->findOrCreateEntryWithoutHierarchyStorage(dbe.path, dbe.category, dbe.newEntryId);
                        }
                        for (const std::pair<InternalMaterial*, ObjectId>& p : data.materialPairs)
                        {
                            p.first->addShapeId(p.second);
                        }
                        // fixup the schemAPIFlags
                        for (const std::pair<SdfPath, uint32_t>& p : schemaAPIFlagsStorage)
                        {
                            const SdfPath newPath = getNewObjectPath(p.first, replicatePath, data.newReplicatePath);
                            objectDb->setSchemaAPI(newPath, p.second);
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
            return true;
        }

        void PhysXReplicator::clear()
        {
        }

        bool registerReplicator(uint64_t stageId, const IReplicatorCallback& callback)
        {
            return OmniPhysX::getInstance().registerReplicator(stageId, callback);
        }

        void unregisterReplicator(uint64_t stageId)
        {
            OmniPhysX::getInstance().unregisterReplicator(stageId);
        }

        bool replicate(uint64_t stageId, uint64_t path, uint32_t numReplications, bool useEnvIds, bool useFabricReplicate)
        {
            PhysXReplicator* replicator = OmniPhysX::getInstance().getReplicator(stageId);
            if (replicator)
            {
                return replicator->replicate(stageId, path, numReplications, useEnvIds, useFabricReplicate);
            }
            return false;
        }


        void isReplicatorStage(uint64_t stageId, bool& replicated, bool& fabricReplicated)
        {
            replicated = false;
            fabricReplicated = false;

            PhysXReplicator* replicator = OmniPhysX::getInstance().getReplicator(stageId);
            if (replicator)
            {
                replicated = true;
                fabricReplicated = replicator->isFabricReplication();
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

