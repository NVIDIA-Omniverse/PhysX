// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "ContactReport.h"
#include "internal/InternalScene.h"
#include "usdInterface/UsdInterface.h"
#include "Setup.h"
#include "OmniPhysX.h"
#include "PhysXTools.h"
#include "PhysXSimulationCallbacks.h"
#include "usdLoad/LoadUsd.h"

#include <private/omni/physx/PhysxUsd.h>

#include <common/utilities/Utilities.h>

using namespace pxr;
using namespace carb;
using namespace ::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;

namespace omni
{
namespace physx
{
    ContactReport::ContactReport()
        : mCurrentReportData(nullptr), mCurrentDataVector(nullptr), mReportFlushed(false)
    {
    }

    void ContactReport::resolvePairs()
    {        
        const OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        const internal::InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();
        UnresolvedContactPairsMap::iterator itUnresolved = mUnresolvedContactPairsMap.begin();
        AttachedStage* attachedStage = UsdLoad::getUsdLoad()->getAttachedStage(omniPhysX.getStageId());
        if(!attachedStage)
            return;

        while (itUnresolved != mUnresolvedContactPairsMap.end())
        {
            if (itUnresolved->second.first == SdfPath())
            {
                mContactPairsMap.insert(std::make_pair(itUnresolved->first, std::make_pair(nullptr, itUnresolved->second.second)));
            }
            else
            {
                const ObjectIdMap* entries = attachedStage->getObjectIds(itUnresolved->second.first);
                if (entries && !entries->empty())
                {
                    ObjectIdMap::const_iterator it = entries->begin();
                    while (it != entries->end())
                    {
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
            itUnresolved++;
        }

        mUnresolvedContactPairsMap.clear();
    }

    bool isCompoundReport(const std::vector<InternalDatabase::Record>& records, const void* shape0UserData, const void* shape1UserData,
        Pair<void*>& compoundPair, pxr::SdfPath& shape0Path, pxr::SdfPath& shape1Path)
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
            shape0Path = record.mPath;

            if (record.mType == ePTCompoundShape)
            {
                shape0Compound = true;
            }
            shape0Ptr = record.mInternalPtr;
        }

        if (index1 && index1 < records.size())
        {
            const InternalDatabase::Record& record = records[index1];
            shape1Path = record.mPath;

            if (record.mType == ePTCompoundShape)
            {
                shape1Compound = true;
            }
            shape1Ptr = record.mInternalPtr;
        }

        compoundPair = Pair<void*>(shape0Ptr, shape1Ptr);

        return shape0Compound || shape1Compound;
    }

    pxr::SdfPath getSdfPath(const std::vector<InternalDatabase::Record>& records, const void* userData, uint32_t& protoIndex)
    {
        SdfPath path;
        const size_t index = (size_t)userData;
        CARB_ASSERT(index < records.size());        

        if (index && index < records.size())
        {
            const InternalDatabase::Record& record = records[index];
            path = record.mPath;
            if (record.mType == ePTActor)
            {
                protoIndex = ((InternalActor*)record.mInternalPtr)->mInstanceIndex;
            }            
        }
        return path;
    }

    pxr::SdfPath getSdfPath(const std::vector<InternalDatabase::Record>& records, const void* userData)
    {
        SdfPath path;
        const size_t index = (size_t)userData;
        CARB_ASSERT(index < records.size());

        if (index && index < records.size())
        {
            const InternalDatabase::Record& record = records[index];
            path = record.mPath;
        }
        return path;
    }

    pxr::SdfPath createEventFromSdfPath(
        carb::events::IEventPtr& eventPtr,
        const char* itemName,
        dictionary::IDictionary* dict,
        const std::vector<InternalDatabase::Record>& records,
        const void* userData
    )
    {
        SdfPath path;
        const size_t index = (size_t)userData;
        CARB_ASSERT(index < records.size());

        if (index && index < records.size())
        {
            path = records[index].mPath;
            const uint64_t ui64Path = asInt(path);

            carb::dictionary::Item* item = dict->createItem(eventPtr->payload, itemName, carb::dictionary::ItemType::eDictionary);
            dict->setArray<int32_t>(item, (const int32_t*)&ui64Path, 2);
        }
        else
        {
            const uint64_t ui64Path = 0ul;
            carb::dictionary::Item* item = dict->createItem(eventPtr->payload, itemName, carb::dictionary::ItemType::eDictionary);
            dict->setArray<int32_t>(item, (const int32_t*)&ui64Path, 2);
        }

        return path;
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

        Pair<void*> shapePair(nullptr, nullptr);
        SdfPath shape0Path;
        SdfPath shape1Path;
        const bool compoundReport = isCompoundReport(records, shape0->userData, shape1->userData, shapePair, shape0Path, shape1Path);
        const long stageId = omniPhysX.getStageId();
        uint32_t protoIndex0 = kInvalidUint32_t;
        uint32_t protoIndex1 = kInvalidUint32_t;
        const SdfPath actor0Path = getSdfPath(records, actor0->userData, protoIndex0);
        const SdfPath actor1Path = getSdfPath(records, actor1->userData, protoIndex1);

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

                    mCurrentReportData->mActor0Path = actor0Path;
                    mCurrentReportData->mActor1Path = actor1Path;

                    mCurrentReportData->mShape0Path = shape0Path;
                    mCurrentReportData->mShape1Path = shape1Path;

                    mCurrentReportData->mEvent = event;
                }
            }

            // store the data for batch report
            {
                Pair<uint64_t> compoundShapePair(asInt(shape0Path), asInt(shape1Path));
                PairContactDataMap::iterator fit = mPairContactDataMap.find(compoundShapePair);
                if (fit == mPairContactDataMap.end())
                {
                    ContactEventStruct& contactEventStruct = mPairContactDataMap[compoundShapePair];
                    contactEventStruct.mContactHeader = {
                        type,
                        stageId,
                        asInt(actor0Path),
                        asInt(actor1Path),
                        asInt(shape0Path),
                        asInt(shape1Path),
                        0,
                        contactCount,
                        0,
                        0,
                        protoIndex0,
                        protoIndex1
                    };

                    mCurrentDataVector = &contactEventStruct.mContactData;

                    // copy the friction anchors
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

                    // copy the friction anchors
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

            // copy the friction anchors
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
                    type, stageId, asInt(actor0Path), asInt(actor1Path), asInt(shape0Path), asInt(shape1Path),
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

        // we need to send contact lost report for deleted objects
        SdfPath collider0Path;
        SdfPath collider1Path;
        SdfPath actor0Path;
        SdfPath actor1Path;
        if (actor0it != mReleaseActorsMap.end())
        {
            actor0Path = actor0it->second;
        }
        else
        {
            actor0Path = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_0) ? SdfPath() : getSdfPath(records, actor0->userData);
        }
        if (actor1it != mReleaseActorsMap.end())
        {
            actor1Path = actor1it->second;
        }
        else
        {
            actor1Path = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_1) ? SdfPath() : getSdfPath(records, actor1->userData);
        }        
        if (shape0it != mReleaseShapesMap.end())
        {
            collider0Path = shape0it->second;
        }
        else
        {
            collider0Path = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_0) ? SdfPath() : getSdfPath(records, shape0->userData);
        }
        if (shape1it != mReleaseShapesMap.end())
        {
            collider1Path = shape1it->second;
        }
        else
        {
            collider1Path = (contactFlags & PxContactPairFlag::eREMOVED_SHAPE_1) ? SdfPath() : getSdfPath(records, shape1->userData);
        }

        const long stageId = omniPhysX.getStageId();
        const Pair<uint64_t> colliderPair(asInt(collider0Path), asInt(collider1Path));
        if (mDeletedEventsSet.find(colliderPair) == mDeletedEventsSet.end())
        {
            mDeletedEventsSet.insert(colliderPair);
        }

        if (mPairContactDataMap.find(colliderPair) == mPairContactDataMap.end())
        {
            mPairContactDataMap[colliderPair].mContactHeader = {
                ContactEventType::eCONTACT_LOST,
                stageId,
                asInt(actor0Path),
                asInt(actor1Path),
                asInt(collider0Path),
                asInt(collider1Path),
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

        const SdfPath material0Path =
            (material0 && material0->userData != nullptr) ? getSdfPath(records, material0->userData) :
            scene->getDefaultMaterialPath();
        const SdfPath material1Path =
            (material1 && material1->userData != nullptr) ? getSdfPath(records, material1->userData) :
            scene->getDefaultMaterialPath();

        // check if we just buffer data or send directly
        if (mCurrentReportData && mCurrentDataVector)
        {
            mCurrentReportData->mContactPoints.push_back(
                {
                    scene, position, normal, impulse, separation, faceIndex0, faceIndex1, material0, material1
                });

            mCurrentDataVector->push_back({ (const carb::Float3&)position, (const carb::Float3&)normal,
                                            (const carb::Float3&)impulse, separation, faceIndex0, faceIndex1,
                                            asInt(material0Path), asInt(material1Path) });

        }
        else
        {            
            // store batched data
            if (batch)
            {
                mContactDataVector.push_back(
                    {
                        (const carb::Float3&)position, (const carb::Float3&)normal, (const carb::Float3&)impulse,
                        separation, faceIndex0, faceIndex1, asInt(material0Path), asInt(material1Path)
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

        const long stageId = omniPhysX.getStageId();        

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

        carb::events::IEventPtr eventPtr = omniPhysX.createSimulationEventV2(eJointBreak);
        createEventFromSdfPath(eventPtr, "jointPath", iDictionary, records, joint->userData);

        omniPhysX.getSimulationEventStreamV2()->push(eventPtr.get());
    }

    void setupContactReportToArticulation(PhysXScene* ps, const PhysxSchemaPhysxContactReportAPI& contactReportAPI, PxArticulationReducedCoordinate& art)
    {
        float contactThreshold = 1.0f * OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale().length;
        const UsdAttribute threshold = contactReportAPI.GetThresholdAttr();
        if (threshold && threshold.HasAuthoredValue())
        {
            threshold.Get(&contactThreshold);
        }

        SdfPathVector targets;
        const UsdRelationship rel = contactReportAPI.GetReportPairsRel();
        if (rel)
        {
            rel.GetTargets(&targets);
        }

        for (PxU32 i = 0; i < art.getNbLinks(); i++)
        {
            PxArticulationLink* link;
            art.getLinks(&link, 1, i);

            if (targets.empty())
            {
                ps->getContactReport()->addActorPair(link, SdfPath(), contactThreshold);
            }
            else
            {
                for (size_t i = 0; i < targets.size(); i++)
                {
                    ps->getContactReport()->addActorPair(link, targets[i], contactThreshold);
                }
            }
        }
    }

    void setupContactReport(PhysXScene* ps, AttachedStage& attachedStage, PxRigidActor& rigidActor, const SdfPath& usdPrimPath)
    {
        const PhysxSchemaPhysxContactReportAPI contactReportAPI = PhysxSchemaPhysxContactReportAPI::Get(attachedStage.getStage(), usdPrimPath);
        if (contactReportAPI)
        {
            attachedStage.getPhysXPhysicsInterface()->setDirty(true);

            float contactThreshold = 1.0f * OmniPhysX::getInstance().getPhysXSetup().getPhysics()->getTolerancesScale().length;
            const UsdAttribute threshold = contactReportAPI.GetThresholdAttr();
            if (threshold && threshold.HasAuthoredValue())
            {
                threshold.Get(&contactThreshold);
            }

            const UsdRelationship rel = contactReportAPI.GetReportPairsRel();
            if (rel)
            {
                SdfPathVector targets;
                rel.GetTargets(&targets);

                if (targets.empty())
                {
                    ps->getContactReport()->addActorPair(&rigidActor, SdfPath(), contactThreshold);
                }
                else
                {
                    for (size_t i = 0; i < targets.size(); i++)
                    {
                        ps->getContactReport()->addActorPair(&rigidActor, targets[i], contactThreshold);
                    }
                }
            }
            else
            {
                ps->getContactReport()->addActorPair(&rigidActor, SdfPath(), contactThreshold);
            }
        }
    }
    void changeContactReport(usdparser::AttachedStage& attachedStage, const pxr::SdfPath& path, bool removed)
    {
        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        InternalPhysXDatabase& db = omniPhysX.getInternalPhysXDatabase();

        const ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(path);
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
                            const PhysxSchemaPhysxContactReportAPI contactReportAPI = PhysxSchemaPhysxContactReportAPI::Get(attachedStage.getStage(), path);
                            if (removed)
                            {
                                if (physxScene)
                                {
                                    PxArticulationLink* link = nullptr;
                                    const PxU32 numLinks = articulation->getNbLinks();
                                    for (PxU32 i = 0; i < numLinks; i++)
                                    {
                                        articulation->getLinks(&link, 1, i);
                                        physxScene->getContactReport()->removeActor(link, path);
                                    }
                                }
                            }
                            else
                            {
                                if (physxScene && contactReportAPI)
                                {
                                    setupContactReportToArticulation(physxScene, contactReportAPI, *articulation);
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
                                    physxScene->getContactReport()->removeActor(rigidActor, path);
                                }
                                else
                                {
                                    setupContactReport(physxScene, attachedStage, *rigidActor, path);
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
            attachedStage.getObjectDatabase()->removeSchemaAPI(path, SchemaAPIFlag::eContactReportAPI);
        }
        else
        {
            attachedStage.getObjectDatabase()->addSchemaAPI(path, SchemaAPIFlag::eContactReportAPI);
        }


    }
}
}
