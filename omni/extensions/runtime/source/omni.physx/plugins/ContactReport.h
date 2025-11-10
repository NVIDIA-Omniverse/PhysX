// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include "PhysXTools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/ContactEvent.h>
#include <PxPhysicsAPI.h>

#include <vector>
#include <map>


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
    pxr::SdfPath mActor0Path;
    pxr::SdfPath mShape0Path;
    pxr::SdfPath mActor1Path;
    pxr::SdfPath mShape1Path;
    SimulationEvent mEvent;
    std::vector<ContactPoint> mContactPoints;
};

using UnresolvedContactPairsMap = std::unordered_multimap<const ::physx::PxRigidActor*, std::pair<pxr::SdfPath, float>>;
using ContactPairsMap =
    std::unordered_multimap<const ::physx::PxRigidActor*, std::pair<const ::physx::PxRigidActor*, float>>;
using ReleasedObjectsMap = std::unordered_map<const ::physx::PxBase*, pxr::SdfPath>;
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

    ~ContactReport()
    {
        release();
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

    void addActorPair(::physx::PxRigidActor* body, const pxr::SdfPath& path, float forceThreshold)
    {
        mUnresolvedContactPairsMap.insert(std::make_pair(body, std::make_pair(path, forceThreshold)));
    }

    void removeActor(::physx::PxRigidActor* actor, const pxr::SdfPath& path)
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

        mReleaseActorsMap[actor] = path;
    }

    void removeShape(::physx::PxShape* shape, const pxr::SdfPath& path)
    {
        mReleaseShapesMap[shape] = path;
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

void setupContactReportToArticulation(PhysXScene* ps,
                                      const pxr::PhysxSchemaPhysxContactReportAPI& contactReportAPI,
                                      ::physx::PxArticulationReducedCoordinate& art);
void setupContactReport(PhysXScene* ps,
                        usdparser::AttachedStage& attachedStage,
                        ::physx::PxRigidActor& rigidActor,
                        const pxr::SdfPath& usdPrimPath);
void changeContactReport(usdparser::AttachedStage& attachedStage, const pxr::SdfPath& path, bool removed);

} // namespace physx
} // namespace omni
