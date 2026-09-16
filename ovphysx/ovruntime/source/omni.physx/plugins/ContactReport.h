// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include "PhysXTools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/ContactEvent.h>
#include <PxPhysicsAPI.h>

#include <atomic>
#include <map>
#include <vector>


namespace omni
{
namespace physx
{
struct ContactPoint
{
    const PhysXScene* mScene;
    ::physx::PxVec3 mPosition;
    ::physx::PxVec3 mNormal;
    ::physx::PxVec3 mImpulse;
    float mSeparation;
    uint32_t mFaceIndex0;
    uint32_t mFaceIndex1;
    const ::physx::PxMaterial* mMaterial0;
    const ::physx::PxMaterial* mMaterial1;
};

struct CompoundShapeReportData
{
    omni::physics::parse::ObjectKey mActor0Path;
    omni::physics::parse::ObjectKey mShape0Path;
    omni::physics::parse::ObjectKey mActor1Path;
    omni::physics::parse::ObjectKey mShape1Path;
    SimulationEvent mEvent;
    std::vector<ContactPoint> mContactPoints;
};

// Internal bookkeeping only, not the wire format (ContactEventHeader is already
// asInt-encoded and backend-agnostic): keyed/valued by ObjectKey rather than
// SdfPath so this state works under any IPhysicsSource.
using UnresolvedContactPairsMap =
    std::unordered_multimap<const ::physx::PxRigidActor*, std::pair<omni::physics::parse::ObjectKey, float>>;
using ContactPairsMap =
    std::unordered_multimap<const ::physx::PxRigidActor*, std::pair<const ::physx::PxRigidActor*, float>>;
using ReleasedObjectsMap = std::unordered_map<const ::physx::PxBase*, omni::physics::parse::ObjectKey>;
using CompoundShapeBufferedData = std::map<Pair<void*>, CompoundShapeReportData>;
using DeletedEventsSet = std::unordered_set<Pair<uint64_t>, PairHash>;
using ContactHeadersVector = std::vector<omni::physx::ContactEventHeader>;
using ContactDataVector = std::vector<ContactData>;
using FrictionAnchorsDataVector = std::vector<FrictionAnchor>;
// A.B. replace with unordered map, map is used now for determinacy, we need to try to replace with PhysX SDK hash map
struct ContactEventStruct
{
    ContactEventHeader mContactHeader;
    ContactDataVector mContactData;
    FrictionAnchorsDataVector mFrictionAnchorsData;
};
using PairContactDataMap = std::map<Pair<uint64_t>, ContactEventStruct>;

class ContactReport
{
public:
    ContactReport();

    ~ContactReport();

    static size_t getLiveInstanceCount()
    {
        return s_liveInstanceCount.load(std::memory_order_relaxed);
    }

    void release()
    {
        mUnresolvedContactPairsMap.clear();
        mContactPairsMap.clear();

        mCompoundShapeBufferedData.clear();
        mCurrentReportData = nullptr;

        mDeletedEventsSet.clear();
    }

    bool empty() const
    {
        return mContactPairsMap.empty();
    }

    void addActorPair(::physx::PxRigidActor* body, omni::physics::parse::ObjectKey key, float forceThreshold)
    {
        mUnresolvedContactPairsMap.insert(std::make_pair(body, std::make_pair(key, forceThreshold)));
    }

    void removeActor(::physx::PxRigidActor* actor, omni::physics::parse::ObjectKey key)
    {
        ContactPairsMap::iterator it = mContactPairsMap.begin();
        while (it != mContactPairsMap.end())
        {
            if (it->first == actor || it->second.first == actor)
            {
                it = mContactPairsMap.erase(it);
            }
            else
            {
                it++;
            }
        }

        // Retained unresolved entries (target not parsed yet) key on the same
        // pointer; drop them too or the next resolvePairs() dereferences a
        // freed actor.
        mUnresolvedContactPairsMap.erase(actor);

        mReleaseActorsMap[actor] = key;
    }

    void removeShape(::physx::PxShape* shape, omni::physics::parse::ObjectKey key)
    {
        mReleaseShapesMap[shape] = key;
    }

    void swapActor(::physx::PxRigidActor* oldActor, ::physx::PxRigidActor* newActor)
    {
        ContactPairsMap::iterator it = mContactPairsMap.begin();
        while (it != mContactPairsMap.end())
        {
            if (it->first == oldActor)
            {
                mContactPairsMap.insert(std::make_pair(newActor, it->second));
                it = mContactPairsMap.erase(it);
            }
            else if (it->second.first == oldActor)
            {
                it->second.first = newActor;
                it++;
            }
            else
            {
                it++;
            }
        }

        // Remap retained unresolved entries to the new source pointer. Collect
        // first: inserting into an unordered_multimap while iterating its
        // equal_range can rehash and invalidate the iterators.
        std::vector<std::pair<omni::physics::parse::ObjectKey, float>> pending;
        UnresolvedContactPairsMap::iterator uit = mUnresolvedContactPairsMap.find(oldActor);
        if (uit != mUnresolvedContactPairsMap.end())
        {
            std::pair<UnresolvedContactPairsMap::iterator, UnresolvedContactPairsMap::iterator> range =
                mUnresolvedContactPairsMap.equal_range(oldActor);
            for (UnresolvedContactPairsMap::iterator i = range.first; i != range.second; ++i)
                pending.push_back(i->second);
            mUnresolvedContactPairsMap.erase(oldActor);
            for (size_t i = 0; i < pending.size(); ++i)
                mUnresolvedContactPairsMap.insert(std::make_pair(newActor, pending[i]));
        }
    }

    ContactPairsMap& getContactPairsMap()
    {
        return mContactPairsMap;
    }

    inline bool checkPair(const ::physx::PxActor* actor0, const ::physx::PxActor* actor1) const
    {
        return checkPairInternal(actor0, actor1) || checkPairInternal(actor1, actor0);
    }

    inline bool checkThreshold(const ::physx::PxActor* actor0, const ::physx::PxActor* actor1, float force)
    {
        return checkThresholdInternal(actor0, force) || checkThresholdInternal(actor1, force);
    }

    void setBodyThreshold(const ::physx::PxRigidActor* actor, float force)
    {
        ContactPairsMap::iterator it = mContactPairsMap.find(actor);
        ContactPairsMap::const_iterator endIt = mContactPairsMap.end();

        if (it != endIt)
        {
            it->second.second = force;
        }
    }

    void reportContact(const ::physx::PxActor* actor0,
                       const ::physx::PxShape* shape0,
                       const ::physx::PxActor* actor1,
                       const ::physx::PxShape* shape1,
                       SimulationEvent event,
                       uint32_t contactCount,
                       const std::vector<::physx::PxContactPairFrictionAnchor>& frictionAnchors);

    void reportContactShapeRemoved(const ::physx::PxActor* actor0,
                                   const ::physx::PxShape* shape0,
                                   const ::physx::PxActor* actor1,
                                   const ::physx::PxShape* shape1,
                                   const ::physx::PxContactPairFlags contactFlags);

    void reportContactPoint(const PhysXScene*,
                            const ::physx::PxVec3& position,
                            const ::physx::PxVec3& normal,
                            const ::physx::PxVec3& impulse,
                            float separation,
                            uint32_t faceIndex0,
                            uint32_t faceIndex1,
                            const ::physx::PxMaterial* material0,
                            const ::physx::PxMaterial* material1,
                            bool batch = true);

    void reportJointBreak(const ::physx::PxJoint* joint);

    void clearReleasedObjectsMap()
    {
        mReleaseActorsMap.clear();
        mReleaseShapesMap.clear();
    }

    void clearBatchedData()
    {
        mContactHeaderVector.clear();
        mContactDataVector.clear();
        mFrictionAnchorsDataVector.clear();
        mPairContactDataMap.clear();
        mReportFlushed = false;
    }

    const ContactHeadersVector& getContactHeaderVector() const
    {
        return mContactHeaderVector;
    }

    const ContactDataVector& getContactDataVector() const
    {
        return mContactDataVector;
    }

    const FrictionAnchorsDataVector& getFrictionAnchorsDataVector() const
    {
        return mFrictionAnchorsDataVector;
    }

    void flushContactReports();

    void flushBatchedContactReports();

    void resolvePairs();

private:
    UnresolvedContactPairsMap mUnresolvedContactPairsMap;
    ContactPairsMap mContactPairsMap;
    ReleasedObjectsMap mReleaseActorsMap;
    ReleasedObjectsMap mReleaseShapesMap;

    CompoundShapeBufferedData mCompoundShapeBufferedData;
    CompoundShapeReportData* mCurrentReportData;
    DeletedEventsSet mDeletedEventsSet;

    ContactHeadersVector mContactHeaderVector;
    ContactDataVector mContactDataVector;
    FrictionAnchorsDataVector mFrictionAnchorsDataVector;
    PairContactDataMap mPairContactDataMap;
    ContactDataVector* mCurrentDataVector;
    bool mReportFlushed;

    static std::atomic<size_t> s_liveInstanceCount;

    inline bool checkPairInternal(const ::physx::PxActor* actor, const ::physx::PxActor* otherActor) const
    {
        const ::physx::PxRigidActor* body = (const ::physx::PxRigidActor*)actor;
        ContactPairsMap::const_iterator it = mContactPairsMap.find(body);
        ContactPairsMap::const_iterator endIt = mContactPairsMap.end();

        while (it != endIt && body == it->first)
        {
            if (it->second.first == nullptr || it->second.first == otherActor)
                return true;
            it++;
        }

        return false;
    }

    inline bool checkThresholdInternal(const ::physx::PxActor* actor, float force) const
    {
        ::physx::PxRigidActor* body = (::physx::PxRigidActor*)actor;
        ContactPairsMap::const_iterator it = mContactPairsMap.find(body);
        ContactPairsMap::const_iterator endIt = mContactPairsMap.end();

        return (it != endIt && force >= it->second.second);
    }
};

// Registers every link of `art` for contact reporting from the PhysxContactReportAPI
// on `key`. The API and its properties are read through the parse source, so this
// works under any backend; a no-op when the API is not applied there.
void setupContactReportToArticulation(PhysXScene* ps,
                                      const usdparser::AttachedStage& attachedStage,
                                      omni::physics::parse::ObjectKey key,
                                      ::physx::PxArticulationReducedCoordinate& art);
void setupContactReport(PhysXScene* ps,
                        usdparser::AttachedStage& attachedStage,
                        ::physx::PxRigidActor& rigidActor,
                        omni::physics::parse::ObjectKey key);
void changeContactReport(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool removed);

} // namespace physx
} // namespace omni
