// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-CONTACTREPORT-001
 * @covers AC-1 AC-2 AC-3
 *
 * @implements REQ-SIM-OBJECTDB-001
 * @covers AC-1
 */

#include <omni/physics/parse/KnownTokens.h>

#include "ContactReport.h"
#include "internal/InternalScene.h"
#include "usdInterface/UsdInterface.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "PhysXTools.h"
#include "PhysXSimulationCallbacks.h"
#include "usdLoad/LoadUsd.h"

#include <private/omni/physx/PhysxUsd.h>

using namespace carb;
using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

namespace omni
{
namespace physx
{
    std::atomic<size_t> ContactReport::s_liveInstanceCount{ 0 };

    ContactReport::ContactReport()
        : mCurrentReportData(nullptr), mCurrentDataVector(nullptr), mReportFlushed(false)
    {
        s_liveInstanceCount.fetch_add(1, std::memory_order_relaxed);
    }

    ContactReport::~ContactReport()
    {
        release();
        s_liveInstanceCount.fetch_sub(1, std::memory_order_relaxed);
    }

    void ContactReport::resolvePairs()
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
        UnresolvedContactPairsMap::iterator itUnresolved = mUnresolvedContactPairsMap.begin();
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        if(!attachedStage)
            return;

        while (itUnresolved != mUnresolvedContactPairsMap.end())
        {
            // Entries that fail to resolve are RETAINED for a later pass, not
            // dropped: resolvePairs runs both at finishSetup and eagerly at
            // replicate time, and an eager pass can see a registration whose
            // report target has no object-DB entry yet (parsed later). Clearing
            // those here would silently and permanently lose the report.
            bool resolved = true;
            if (!itUnresolved->second.first.valid())
            {
                mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(nullptr, itUnresolved->second.second)));
            }
            else
            {
                const ObjectIdMap* entries = attachedStage->getObjectIds(itUnresolved->second.first);
                if (!entries || entries->empty())
                {
                    resolved = false;
                }
                else
                {
                    ObjectIdMap::const_iterator it = entries->begin();
                    while (it != entries->end())
                    {
                        // The object database can hold a kInvalidObjectId entry, and getRecords() is
                        // only CARB_ASSERT-guarded - a no-op in release. Skip ids that cannot index
                        // the record array instead of reading far out of bounds.
                        if (size_t(it->second) >= db.getRecords().size())
                        {
                            it++;
                            continue;
                        }

                        const InternalDatabase::Record& rec = db.getRecords()[size_t(it->second)];
                        if (rec.mType == ePTActor)
                        {
                            InternalActor* actor = (InternalActor*)rec.mInternalPtr;
                            // If we have pair wise filtering we need to filter in the same scene
                            if (actor->mMirrors.empty() || (itUnresolved->first->getScene() == actor->mActor->getScene()))
                            {
                                mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(actor->mActor, itUnresolved->second.second)));
                            }
                            else
                            {
                                for (const MirrorActor& mirror : actor->mMirrors)
                                {
                                    if (mirror.actor->getScene() == itUnresolved->first->getScene())
                                    {
                                        mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(mirror.actor, itUnresolved->second.second)));
                                        break;
                                    }
                                }
                            }
                        }
                        else if (rec.mType == ePTLink)
                        {
                            InternalActor* actor = (InternalActor*)rec.mInternalPtr;
                            // If we have pair wise filtering we need to filter in the same scene
                            if (actor->mMirrors.empty() || (itUnresolved->first->getScene() == actor->mActor->getScene()))
                            {
                                mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(actor->mActor, itUnresolved->second.second)));
                            }
                            else
                            {
                                for (const MirrorActor& mirror : actor->mMirrors)
                                {
                                    if (mirror.actor->getScene() == itUnresolved->first->getScene())
                                    {
                                        mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(mirror.actor, itUnresolved->second.second)));
                                        break;
                                    }
                                }
                            }
                        }
                        else if (rec.mType == eArticulation)
                        {
                            PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)rec.mPtr;
                            for (PxU32 i = 0; i < articulation->getNbLinks(); i++)
                            {
                                PxArticulationLink* link;
                                articulation->getLinks(&link, 1, i);
                                mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(link, itUnresolved->second.second)));
                            }
                        }

                        it++;
                    }
                }
            }
            if (resolved)
                itUnresolved = mUnresolvedContactPairsMap.erase(itUnresolved);
            else
                itUnresolved++;
        }
    }

    bool isCompoundReport(const std::vector<InternalDatabase::Record>& records, const void* shape0UserData, const void* shape1UserData,
        Pair<void*>& compoundPair, omni::physics::parse::ObjectKey& shape0Key, omni::physics::parse::ObjectKey& shape1Key)
    {
        const size_t index0 = (size_t)shape0UserData;
        CARB_ASSERT(index0 < records.size());

        const size_t index1 = (size_t)shape1UserData;
        CARB_ASSERT(index1 < records.size());

        bool shape0Compound = false;
        bool shape1Compound = false;
        void* shape0Ptr = nullptr;
        void* shape1Ptr = nullptr;

        if (index0 && index0 < records.size())
        {
            const InternalDatabase::Record& record = records[index0];
            shape0Key = record.mKey;

            if (record.mType == ePTCompoundShape)
            {
                shape0Compound = true;
            }
            shape0Ptr = record.mInternalPtr;
        }

        if (index1 && index1 < records.size())
        {
            const InternalDatabase::Record& record = records[index1];
            shape1Key = record.mKey;

            if (record.mType == ePTCompoundShape)
            {
                shape1Compound = true;
            }
            shape1Ptr = record.mInternalPtr;
        }

        compoundPair = Pair<void*>(shape0Ptr, shape1Ptr);

        return shape0Compound || shape1Compound;
    }

    // ObjectKey siblings of the former getSdfPath/createEventFromSdfPath helpers
    // (pre-ADR-0019 residue): records[index].mKey is already the identity these
    // call sites need, so no pathFor round trip through SdfPath is required.
    omni::physics::parse::ObjectKey getObjectKey(const std::vector<InternalDatabase::Record>& records, const void* userData, uint32_t& protoIndex)
    {
        omni::physics::parse::ObjectKey key;
        const size_t index = (size_t)userData;
        CARB_ASSERT(index < records.size());

        if (index && index < records.size())
        {
            const InternalDatabase::Record& record = records[index];
            key = record.mKey;
            if (record.mType == ePTActor)
            {
                protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
            }
        }
        return key;
    }

    omni::physics::parse::ObjectKey getObjectKey(const std::vector<InternalDatabase::Record>& records, const void* userData)
    {
        omni::physics::parse::ObjectKey key;
        const size_t index = (size_t)userData;
        CARB_ASSERT(index < records.size());

        if (index && index < records.size())
        {
            key = records[index].mKey;
        }
        return key;
    }

    // ContactEvent.h's actor0/actor1/collider0/collider1/material0/material1 fields are
    // uint64_t, not ObjectKey (unlike IPhysxSceneQuery.h's hit-result fields, which this
    // same round DID retype to ObjectKey) -- their wire format is the legacy
    // asInt(ObjectKey) encoding (key.handle), decoded via keyFromLegacyId()/textFor() by
    // every consumer (production: tensors/base/BaseRigidContactView.cpp; test:
    // TestContactReport.cpp, TestContactAndCleanupCoverage.cpp, and others -- test-side
    // decode fixups deferred to the end-of-session sweep, see AGENTS.md direction).
    // TODO: retype ContactEvent.h itself to ObjectKey directly; the bookkeeping above
    // (map keys, CompoundShapeReportData) is already ObjectKey-native internally.
    uint64_t keyToLegacyPathInt(const AttachedStage* attachedStage, omni::physics::parse::ObjectKey key)
    {
        return attachedStage ? key.handle : 0;
    }

    void createEventFromKey(carb::events::IEventPtr& eventPtr,
                            carb::cpp::string_view itemName,
                            dictionary::IDictionary* dict,
                            const AttachedStage* attachedStage,
                            const std::vector<InternalDatabase::Record>& records,
                            const void* userData)
    {
        omni::physics::parse::ObjectKey key = getObjectKey(records, userData);
        const uint64_t ui64Path = keyToLegacyPathInt(attachedStage, key);

        // Preserve the event payload contract: low 32 bits, then high 32 bits.
        const int32_t splitPath[]{
            static_cast<int32_t>(ui64Path & 0xffffffffu),
            static_cast<int32_t>(ui64Path >> 32),
        };
        carb::dictionary::Item* item =
            dict->createItem(eventPtr->payload, itemName, carb::dictionary::ItemType::eDictionary);
        dict->setArray<int32_t>(item, splitPath);
    }


    ContactEventType::Enum fixupHeaderEvent(ContactEventType::Enum currentType, ContactEventType::Enum newType)
    {        
        // we have both lost and persist, we keep persist
        if (currentType == ContactEventType::eCONTACT_LOST && newType == ContactEventType::eCONTACT_PERSIST)
        {
            return ContactEventType::eCONTACT_PERSIST;
        }
        if (currentType == ContactEventType::eCONTACT_FOUND && newType == ContactEventType::eCONTACT_PERSIST)
        {
            return ContactEventType::eCONTACT_PERSIST;
        }
        if (currentType == ContactEventType::eCONTACT_FOUND && newType == ContactEventType::eCONTACT_LOST)
        {
            return ContactEventType::eCONTACT_PERSIST;
        }
        if (currentType == ContactEventType::eCONTACT_PERSIST && newType == ContactEventType::eCONTACT_LOST)
        {
            return ContactEventType::eCONTACT_PERSIST;
        }
        if (currentType == ContactEventType::eCONTACT_PERSIST && newType == ContactEventType::eCONTACT_FOUND)
        {
            return ContactEventType::eCONTACT_PERSIST;
        }
        if (currentType == ContactEventType::eCONTACT_LOST && newType == ContactEventType::eCONTACT_FOUND)
        {
            return ContactEventType::eCONTACT_PERSIST;
        }
        return newType;
    }

    SimulationEvent fixupHeaderEvent(SimulationEvent currentType, SimulationEvent newType)
    {
        // we have both lost and persist, we keep persist
        if (currentType == SimulationEvent::eContactLost && newType == SimulationEvent::eContactPersists)
        {
            return SimulationEvent::eContactPersists;
        }
        if (currentType == SimulationEvent::eContactFound && newType == SimulationEvent::eContactPersists)
        {
            return SimulationEvent::eContactPersists;
        }
        if (currentType == SimulationEvent::eContactFound && newType == SimulationEvent::eContactLost)
        {
            return SimulationEvent::eContactPersists;
        }
        if (currentType == SimulationEvent::eContactPersists && newType == SimulationEvent::eContactLost)
        {
            return SimulationEvent::eContactPersists;
        }
        if (currentType == SimulationEvent::eContactPersists && newType == SimulationEvent::eContactFound)
        {
            return SimulationEvent::eContactPersists;
        }
        if (currentType == SimulationEvent::eContactLost && newType == SimulationEvent::eContactFound)
        {
            return SimulationEvent::eContactPersists;
        }
        return newType;
    }

    void ContactReport::reportContact(const ::physx::PxActor* actor0, const ::physx::PxShape* shape0,
        const ::physx::PxActor* actor1, const ::physx::PxShape* shape1, SimulationEvent event, uint32_t contactCount,
        const std::vector<PxContactPairFrictionAnchor>& frictionAnchors)
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();
        const AttachHandle attachHandle = attachedStage ? attachedStage->getAttachHandle() : kNoAttach;

        Pair<void*> shapePair(nullptr, nullptr);
        omni::physics::parse::ObjectKey shape0Key;
        omni::physics::parse::ObjectKey shape1Key;
        const bool compoundReport = isCompoundReport(records, shape0->userData, shape1->userData, shapePair, shape0Key, shape1Key);
        uint32_t protoIndex0 = kInvalidUint32_t;
        uint32_t protoIndex1 = kInvalidUint32_t;
        const omni::physics::parse::ObjectKey actor0Key = getObjectKey(records, actor0->userData, protoIndex0);
        const omni::physics::parse::ObjectKey actor1Key = getObjectKey(records, actor1->userData, protoIndex1);

        // Legacy asInt(ObjectKey)-encoded ids for the public ContactEventHeader (see
        // keyToLegacyPathInt's comment); everything above stays ObjectKey-native.
        const uint64_t actor0PathInt = keyToLegacyPathInt(attachedStage, actor0Key);
        const uint64_t actor1PathInt = keyToLegacyPathInt(attachedStage, actor1Key);
        const uint64_t shape0PathInt = keyToLegacyPathInt(attachedStage, shape0Key);
        const uint64_t shape1PathInt = keyToLegacyPathInt(attachedStage, shape1Key);

        const ContactEventType::Enum type = (event == eContactFound) ? ContactEventType::eCONTACT_FOUND :
                                            (event == eContactLost)  ? ContactEventType::eCONTACT_LOST :
                                                                       ContactEventType::eCONTACT_PERSIST;

        if (compoundReport)
        {
            {
                CompoundShapeBufferedData::iterator fit = mCompoundShapeBufferedData.find(shapePair);
                if (fit != mCompoundShapeBufferedData.end())
                {
                    mCurrentReportData = &fit->second;
                    mCurrentReportData->mEvent = fixupHeaderEvent(mCurrentReportData->mEvent, event);
                }
                else
                {
                    mCurrentReportData = &mCompoundShapeBufferedData[shapePair];

                    // Internal bookkeeping only (see CompoundShapeReportData); already
                    // resolved as ObjectKey by isCompoundReport/getObjectKey above, no
                    // AttachedStage needed here.
                    mCurrentReportData->mActor0Path = actor0Key;
                    mCurrentReportData->mActor1Path = actor1Key;

                    mCurrentReportData->mShape0Path = shape0Key;
                    mCurrentReportData->mShape1Path = shape1Key;

                    mCurrentReportData->mEvent = event;
                }
            }

            // store the data for batch report
            {
                Pair<uint64_t> compoundShapePair(shape0Key.handle, shape1Key.handle);
                PairContactDataMap::iterator fit = mPairContactDataMap.find(compoundShapePair);
                if (fit == mPairContactDataMap.end())
                {
                    ContactEventStruct& contactEventStruct = mPairContactDataMap[compoundShapePair];
                    contactEventStruct.mContactHeader = {
                        type,
                        attachHandle,
                        actor0PathInt,
                        actor1PathInt,
                        shape0PathInt,
                        shape1PathInt,
                        0,
                        contactCount,
                        0,
                        0,
                        protoIndex0,
                        protoIndex1
                    };

                    mCurrentDataVector = &contactEventStruct.mContactData;

                    const size_t frictionAnchorsSize = frictionAnchors.size();
                    if (frictionAnchorsSize)
                    {
                        static_assert(sizeof(FrictionAnchor) == sizeof(::physx::PxContactPairFrictionAnchor), "FrictionAnchor structure size check.");
                        contactEventStruct.mFrictionAnchorsData.resize(frictionAnchorsSize);
                        memcpy(contactEventStruct.mFrictionAnchorsData.data(), frictionAnchors.data(), sizeof(FrictionAnchor) * frictionAnchorsSize);
                    }
                }
                else
                {
                    ContactEventStruct& contactEventStruct = fit->second;
                    ContactEventHeader& header = contactEventStruct.mContactHeader;
                    header.numContactData += contactCount;

                    // set just one header type
                    header.type = fixupHeaderEvent(header.type, type);
                    mCurrentDataVector = &contactEventStruct.mContactData;

                    const size_t frictionAnchorsOffset = contactEventStruct.mFrictionAnchorsData.size();
                    const size_t frictionAnchorsSize = frictionAnchors.size();
                    if (frictionAnchorsSize)
                    {
                        static_assert(sizeof(FrictionAnchor) == sizeof(::physx::PxContactPairFrictionAnchor), "FrictionAnchor structure size check.");
                        contactEventStruct.mFrictionAnchorsData.resize(frictionAnchorsOffset + frictionAnchorsSize);
                        memcpy(contactEventStruct.mFrictionAnchorsData.data() + frictionAnchorsOffset, frictionAnchors.data(), sizeof(FrictionAnchor) * frictionAnchorsSize);
                    }
                }
            }
        }
        else
        {
            mCurrentReportData = nullptr;
            mCurrentDataVector = nullptr;

            const size_t frictionAnchorsOffset = mFrictionAnchorsDataVector.size();
            const size_t frictionAnchorsSize = frictionAnchors.size();
            if (frictionAnchorsSize)
            {
                static_assert(sizeof(FrictionAnchor) == sizeof(::physx::PxContactPairFrictionAnchor), "FrictionAnchor structure size check.");
                mFrictionAnchorsDataVector.resize(frictionAnchorsOffset + frictionAnchorsSize);
                memcpy(mFrictionAnchorsDataVector.data() + frictionAnchorsOffset, frictionAnchors.data(), sizeof(FrictionAnchor) * frictionAnchorsSize);
            }
            
            // store the data for batch report
            mContactHeaderVector.push_back(
                {
                    type, attachHandle, actor0PathInt, actor1PathInt, shape0PathInt, shape1PathInt,
                    uint32_t(mContactDataVector.size()), contactCount, uint32_t(frictionAnchorsOffset), uint32_t(frictionAnchorsSize), protoIndex0, protoIndex1
                }
            );
        }
    }

    void ContactReport::reportContactShapeRemoved(const ::physx::PxActor* actor0, const ::physx::PxShape* shape0, const ::physx::PxActor* actor1, const ::physx::PxShape* shape1, const PxContactPairFlags contactFlags)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        ReleasedObjectsMap::const_iterator actor0it = mReleaseActorsMap.find(actor0);
        ReleasedObjectsMap::const_iterator actor1it = mReleaseActorsMap.find(actor1);
        ReleasedObjectsMap::const_iterator shape0it = mReleaseShapesMap.find(shape0);
        ReleasedObjectsMap::const_iterator shape1it = mReleaseShapesMap.find(shape1);
        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

        // we need to send contact lost report for deleted objects
        omni::physics::parse::ObjectKey collider0Key;
        omni::physics::parse::ObjectKey collider1Key;
        omni::physics::parse::ObjectKey actor0Key;
        omni::physics::parse::ObjectKey actor1Key;
        if (actor0it != mReleaseActorsMap.end())
        {
            actor0Key = actor0it->second;
        }
        else
        {
            actor0Key = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_0) ? omni::physics::parse::ObjectKey() : getObjectKey(records, actor0->userData);
        }
        if (actor1it != mReleaseActorsMap.end())
        {
            actor1Key = actor1it->second;
        }
        else
        {
            actor1Key = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_1) ? omni::physics::parse::ObjectKey() : getObjectKey(records, actor1->userData);
        }
        if (shape0it != mReleaseShapesMap.end())
        {
            collider0Key = shape0it->second;
        }
        else
        {
            collider0Key = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_0) ? omni::physics::parse::ObjectKey() : getObjectKey(records, shape0->userData);
        }
        if (shape1it != mReleaseShapesMap.end())
        {
            collider1Key = shape1it->second;
        }
        else
        {
            collider1Key = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_1) ? omni::physics::parse::ObjectKey() : getObjectKey(records, shape1->userData);
        }

        const AttachHandle attachHandle = attachedStage ? attachedStage->getAttachHandle() : kNoAttach;
        const Pair<uint64_t> colliderPair(collider0Key.handle, collider1Key.handle);
        if (mDeletedEventsSet.find(colliderPair) == mDeletedEventsSet.end())
        {
            mDeletedEventsSet.insert(colliderPair);
        }

        if (mPairContactDataMap.find(colliderPair) == mPairContactDataMap.end())
        {
            mPairContactDataMap[colliderPair].mContactHeader = {
                ContactEventType::eCONTACT_LOST,
                attachHandle,
                keyToLegacyPathInt(attachedStage, actor0Key),
                keyToLegacyPathInt(attachedStage, actor1Key),
                keyToLegacyPathInt(attachedStage, collider0Key),
                keyToLegacyPathInt(attachedStage, collider1Key),
                0,
                0,
                0,
                0,
                0xFFFFFFFF,
                0xFFFFFFFF
            };
        }
    }

    void ContactReport::reportContactPoint(const PhysXScene* scene, const ::physx::PxVec3& position, const ::physx::PxVec3& normal, const ::physx::PxVec3& impulse, float separation,
        uint32_t faceIndex0, uint32_t faceIndex1, const PxMaterial* material0, const PxMaterial* material1, bool batch)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
        carb::dictionary::IDictionary* iDictionary = omniPhysX.getIDictionary();
        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        const AttachedStage* attachedStage = &scene->getAttachedStage();

        const omni::physics::parse::ObjectKey material0Key =
            (material0 && material0->userData != nullptr) ? getObjectKey(records, material0->userData) :
            scene->getDefaultMaterialPath();
        const omni::physics::parse::ObjectKey material1Key =
            (material1 && material1->userData != nullptr) ? getObjectKey(records, material1->userData) :
            scene->getDefaultMaterialPath();

        // Legacy asInt(ObjectKey)-encoded ids for the public ContactData (see
        // keyToLegacyPathInt's comment).
        const uint64_t material0PathInt = keyToLegacyPathInt(attachedStage, material0Key);
        const uint64_t material1PathInt = keyToLegacyPathInt(attachedStage, material1Key);

        // check if we just buffer data or send directly
        if (mCurrentReportData && mCurrentDataVector)
        {
            mCurrentReportData->mContactPoints.push_back(
                {
                    scene, position, normal, impulse, separation, faceIndex0, faceIndex1, material0, material1
                });

            mCurrentDataVector->push_back({ (const carb::Float3&)position, (const carb::Float3&)normal,
                                            (const carb::Float3&)impulse, separation, faceIndex0, faceIndex1,
                                            material0PathInt, material1PathInt });

        }
        else
        {
            if (batch)
            {
                mContactDataVector.push_back(
                    {
                        (const carb::Float3&)position, (const carb::Float3&)normal, (const carb::Float3&)impulse,
                        separation, faceIndex0, faceIndex1, material0PathInt, material1PathInt
                    }
                );
            }
        }
    }

    void ContactReport::flushContactReports()
    {
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        mCurrentReportData = nullptr;

        for (CompoundShapeBufferedData::const_reference& reportIterator : mCompoundShapeBufferedData)
        {
            const CompoundShapeReportData& reportData = reportIterator.second;

            for (const ContactPoint& contactPoint : reportData.mContactPoints)
            {
                reportContactPoint(contactPoint.mScene, contactPoint.mPosition, contactPoint.mNormal, contactPoint.mImpulse, contactPoint.mSeparation,
                    contactPoint.mFaceIndex0, contactPoint.mFaceIndex1, contactPoint.mMaterial0, contactPoint.mMaterial1, false);
            }
        }

        mCompoundShapeBufferedData.clear();
        mDeletedEventsSet.clear();

        // gather batched reports
        for (PairContactDataMap::const_reference contactPair : mPairContactDataMap)
        {
            const ContactEventHeader& header = contactPair.second.mContactHeader;
            mContactHeaderVector.push_back(header);
            if (header.numContactData > 0)
            {
                mContactHeaderVector.back().contactDataOffset = uint32_t(mContactDataVector.size());
                const ContactDataVector& contactData = contactPair.second.mContactData;
                CARB_ASSERT(header.numContactData == uint32_t(contactData.size()));
                if (header.numContactData == uint32_t(contactData.size()))
                {
                    for (uint32_t i = 0; i < header.numContactData; i++)
                    {
                        mContactDataVector.push_back(contactData[i]);
                    }
                }
            }
            const FrictionAnchorsDataVector& frictionAnchorsData = contactPair.second.mFrictionAnchorsData;
            const size_t numFrictionAnchorsDataSize = frictionAnchorsData.size();
            if (numFrictionAnchorsDataSize)
            {
                mContactHeaderVector.back().frictionAnchorsDataOffset = uint32_t(mFrictionAnchorsDataVector.size());
                mContactHeaderVector.back().numfrictionAnchorsData = uint32_t(numFrictionAnchorsDataSize);

                mFrictionAnchorsDataVector.insert(mFrictionAnchorsDataVector.end(), frictionAnchorsData.begin(), frictionAnchorsData.end());
            }
        }

        clearReleasedObjectsMap();
    }

    void ContactReport::flushBatchedContactReports()
    {
        if (!mContactHeaderVector.empty() && !mReportFlushed)
        {
            {
                const ContactReportEventSubscriptionRegistry& contactReportRegistry =
                    SimulationCallbacks::getSimulationCallbacks()->getContactReportRegistry();
                ContactReportEventSubscriptionRegistry::EventMap::const_iterator it = contactReportRegistry.map.begin();
                ContactReportEventSubscriptionRegistry::EventMap::const_iterator itEnd = contactReportRegistry.map.end();
                while (it != itEnd)
                {
                    it->second.first(mContactHeaderVector.data(), uint32_t(mContactHeaderVector.size()),
                        mContactDataVector.data(), uint32_t(mContactDataVector.size()), it->second.second);
                    it++;
                }
            }

            {
                const FullContactReportEventSubscriptionRegistry& contactReportRegistry =
                    SimulationCallbacks::getSimulationCallbacks()->getFullContactReportRegistry();
                FullContactReportEventSubscriptionRegistry::EventMap::const_iterator it = contactReportRegistry.map.begin();
                FullContactReportEventSubscriptionRegistry::EventMap::const_iterator itEnd = contactReportRegistry.map.end();
                while (it != itEnd)
                {
                    it->second.first(mContactHeaderVector.data(), uint32_t(mContactHeaderVector.size()),
                        mContactDataVector.data(), uint32_t(mContactDataVector.size()), mFrictionAnchorsDataVector.data(),
                        uint32_t(mFrictionAnchorsDataVector.size()), it->second.second);
                    it++;
                }
            }

            mReportFlushed = true;
        }
    }

    void ContactReport::reportJointBreak(const ::physx::PxJoint* joint)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        const std::vector<InternalDatabase::Record>& records = db.getRecords();
        carb::dictionary::IDictionary* iDictionary = omniPhysX.getIDictionary();
        const AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getActiveAttachedStage();

        carb::events::IEventPtr eventPtr = omniPhysX.createSimulationEventV2(eJointBreak);
        createEventFromKey(eventPtr, "jointKey", iDictionary, attachedStage, records, joint->userData);

        omniPhysX.getSimulationEventStreamV2()->push(eventPtr.get());
    }

    // Contact-report parameters for `key`, read through the parse source rather than
    // off a UsdPrim, so they resolve under any backend (ovstage publishes
    // PhysxContactReportAPI). Returns false when the API is not applied there;
    // `outThreshold` is only overwritten when the resolved value differs from the raw
    // schema fallback (see the ambiguous-fallback-collapse note below).
    static bool readContactReportParams(const AttachedStage& attachedStage,
                                        omni::physics::parse::ObjectKey key,
                                        float& outThreshold,
                                        std::vector<omni::physics::parse::ObjectKey>& outPairs,
                                        bool& outTargetsAuthored)
    {
        outTargetsAuthored = false;
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (!src || !src->exists(key))
            return false;

        const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();

        if (!src->hasSchema(key, tok.physxContactReportAPI))
            return false;

        // The caller's default is units-scaled (tolerances length), which the schema
        // fallback (raw 1.0) cannot express -- so an unauthored threshold must leave it
        // alone. ovstage's hasAuthoredAttribute is unconditionally true (ADR-0020), so a
        // read landing exactly on the raw schema default (1.0) is indistinguishable from
        // unauthored and collapses to it (ADR-0002 invariant 1), same shape as
        // parsePhysXCharacterControllerDesc's contactOffset/stepOffset.
        const float rawSchemaThreshold = 1.0f;
        if (src->hasAuthoredAttribute(key, tok.physxContactReportThreshold))
        {
            float threshold = rawSchemaThreshold;
            if (getValue<float>(attachedStage, key, tok.physxContactReportThreshold,
                                omni::physics::parse::ReadTime::defaultTime(), threshold) &&
                threshold != rawSchemaThreshold)
            {
                outThreshold = threshold;
            }
        }

        // hasRelationship distinguishes "no reportPairs relationship authored at all"
        // (the wildcard/report-all-contacts case) from "authored but empty" --
        // getRelationshipTargets alone reports empty for both (IPhysicsSource.h), so a
        // caller that must tell them apart (see addContactReportPairs below) needs this.
        outTargetsAuthored = src->hasRelationship(key, tok.physxContactReportReportPairs);
        src->getRelationshipTargets(key, tok.physxContactReportReportPairs, outPairs);
        return true;
    }

    // Register `actor` for every reported pair, or once against the invalid key when the
    // relationship names no targets at all (which is what "report all contacts" means
    // here). `targetsAuthored` distinguishes that legitimate no-targets-authored case from
    // a restricted filter whose authored targets all failed to resolve to a live object
    // (`pairs` empty too, but for a different reason): the latter must not silently widen
    // into the all-contacts wildcard, so it registers nothing instead.
    static void addContactReportPairs(PhysXScene* ps,
                                      PxRigidActor* actor,
                                      const std::vector<omni::physics::parse::ObjectKey>& pairs,
                                      float contactThreshold,
                                      bool targetsAuthored)
    {
        if (pairs.empty())
        {
            if (!targetsAuthored)
            {
                ps->getContactReport()->addActorPair(actor, omni::physics::parse::ObjectKey(), contactThreshold);
            }
            return;
        }
        for (const omni::physics::parse::ObjectKey& pair : pairs)
        {
            ps->getContactReport()->addActorPair(actor, pair, contactThreshold);
        }
    }

    void setupContactReportToArticulation(PhysXScene* ps,
                                          const AttachedStage& attachedStage,
                                          omni::physics::parse::ObjectKey key,
                                          PxArticulationReducedCoordinate& art)
    {
        float contactThreshold = 1.0f * OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale().length;
        std::vector<omni::physics::parse::ObjectKey> targets;
        bool targetsAuthored = false;
        if (!readContactReportParams(attachedStage, key, contactThreshold, targets, targetsAuthored))
            return;

        for (PxU32 i = 0; i < art.getNbLinks(); i++)
        {
            PxArticulationLink* link;
            art.getLinks(&link, 1, i);
            addContactReportPairs(ps, link, targets, contactThreshold, targetsAuthored);
        }
    }

    void setupContactReport(PhysXScene* ps, AttachedStage& attachedStage, PxRigidActor& rigidActor, omni::physics::parse::ObjectKey key)
    {
        float contactThreshold = 1.0f * OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale().length;
        std::vector<omni::physics::parse::ObjectKey> targets;
        bool targetsAuthored = false;
        if (!readContactReportParams(attachedStage, key, contactThreshold, targets, targetsAuthored))
            return;

        attachedStage.getPhysXPhysicsInterface()->setDirty(true);
        addContactReportPairs(ps, &rigidActor, targets, contactThreshold, targetsAuthored);
    }
    void changeContactReport(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool removed)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(key);
        if (entries && !entries->empty())
        {
            ObjectIdMap::const_iterator it = entries->begin();
            while (it != entries->end())
            {
                const ObjectId objectId = it->second;
                PhysXType internalType = ePTRemoved;
                InternalDatabase::Record* objectRecord = db.getFullRecord(internalType, objectId);
                if (objectRecord)
                {
                    if (internalType == ePTArticulation)
                    {
                        PxArticulationReducedCoordinate* articulation = (PxArticulationReducedCoordinate*)objectRecord->mPtr;
                        if (articulation && articulation->getScene())
                        {
                            PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(size_t(articulation->getScene()->userData));
                            if (removed)
                            {
                                if (physxScene)
                                {
                                    PxArticulationLink* link = nullptr;
                                    const PxU32 numLinks = articulation->getNbLinks();
                                    for (PxU32 i = 0; i < numLinks; i++)
                                    {
                                        articulation->getLinks(&link, 1, i);
                                        physxScene->getContactReport()->removeActor(link, key);
                                    }
                                }
                            }
                            else
                            {
                                if (physxScene)
                                {
                                    setupContactReportToArticulation(physxScene, attachedStage, key, *articulation);
                                    physxScene->getContactReport()->resolvePairs();
                                }
                            }
                        }
                    }
                    else if (internalType == ePTLink || internalType == ePTActor)
                    {
                        PxRigidActor* rigidActor = (PxRigidActor*)objectRecord->mPtr;
                        if (rigidActor && rigidActor->getScene())
                        {
                            PhysXScene* physxScene = OmniPhysX::getInstance().getPhysXSetup().getPhysXScene(size_t(rigidActor->getScene()->userData));                                
                            if (physxScene)
                            {
                                if (removed)
                                {
                                    physxScene->getContactReport()->removeActor(rigidActor, key);
                                }
                                else
                                {
                                    setupContactReport(physxScene, attachedStage, *rigidActor, key);
                                    physxScene->getContactReport()->resolvePairs();
                                }
                            }
                        }
                    }
                }
                it++;
            }
        }            

        if (removed)
        {
            attachedStage.getObjectDatabase()->removeSchemaAPI(key, SchemaAPIFlag::eContactReportAPI);
        }
        else
        {
            attachedStage.getObjectDatabase()->addSchemaAPI(key, SchemaAPIFlag::eContactReportAPI);
        }


    }
}
}
