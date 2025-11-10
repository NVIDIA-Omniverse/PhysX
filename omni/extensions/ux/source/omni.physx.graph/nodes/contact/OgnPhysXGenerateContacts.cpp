// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Framework.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

#include <PxPhysicsAPI.h>
#include <PxImmediateMode.h>

#include <common/foundation/TypeCast.h>

// include auto-generated header
#include <OgnPhysXGenerateContactsDatabase.h>

class OgnPhysXGenerateContacts
{
private:

public:
    static bool compute(OgnPhysXGenerateContactsDatabase& db)
    {
        omni::physx::IPhysx* physXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
        if (!physXInterface)
        {
            db.logError("Could not retrieve omni::physx::IPhysx interface");
            return false;
        }

        // The node can potentially process 2 lists of shapes,
        // though for now it's just 2 shapes
        std::vector<NameToken> shape0Tokens;
        shape0Tokens.push_back(db.inputs.shape0());
        std::vector<NameToken> shape1Tokens;
        shape1Tokens.push_back(db.inputs.shape1());

        if (shape0Tokens.size() != shape1Tokens.size())
        {
            db.logError("The sizes of shape0 and shape1 input should be the same.");
            return false;
        }

        ::physx::PxU32 pairCount = ::physx::PxU32(shape0Tokens.size());
        std::vector<const ::physx::PxGeometry*> geom0, geom1;
        std::vector<::physx::PxTransform> pose0, pose1;
        ::physx::PxReal contactDistance = db.inputs.contactDistance();
        ::physx::PxReal meshContactMargin = db.inputs.meshContactMargin();
        ::physx::PxReal toleranceLength = db.inputs.toleranceLength();

        for (::physx::PxU32 i = 0; i < pairCount; ++i)
        {
            const char* shape0Path = db.tokenToString(shape0Tokens[i]);
            const ::physx::PxShape* shape0 = reinterpret_cast<const ::physx::PxShape*>(physXInterface->getPhysXPtr(pxr::SdfPath(shape0Path), omni::physx::ePTShape));
            if (shape0 == nullptr)
            {
                db.logError("Accessing object id for prim at \"%s\" failed", shape0Path);
                return false;
            }

            const char* shape1Path = db.tokenToString(shape1Tokens[i]);
            const ::physx::PxShape* shape1 = reinterpret_cast<const ::physx::PxShape*>(physXInterface->getPhysXPtr(pxr::SdfPath(shape1Path), omni::physx::ePTShape));
            if (shape1 == nullptr)
            {
                db.logError("Accessing object id for prim at \"%s\" failed", shape1Path);
                return false;
            }

            geom0.push_back(&shape0->getGeometry());
            pose0.push_back(shape0->getActor()->getGlobalPose().transform(shape0->getLocalPose()));
            geom1.push_back(&shape1->getGeometry());
            pose1.push_back(shape1->getActor()->getGlobalPose().transform(shape1->getLocalPose()));
        }

        std::vector<std::vector<::physx::PxContactPoint>> contacts(pairCount);
        struct ContactRecorder : ::physx::immediate::PxContactRecorder
        {
            std::vector<std::vector<::physx::PxContactPoint>> &contacts;
            ContactRecorder(std::vector<std::vector<::physx::PxContactPoint>> &_contacts) : contacts(_contacts) {}
            virtual bool recordContacts(const ::physx::PxContactPoint* contactPoints, const ::physx::PxU32 nbContacts, const ::physx::PxU32 index)
            {
                for (::physx::PxU32 i = 0; i < nbContacts; ++i)
                    contacts[index].push_back(contactPoints[i]);

                return true;
            }
        }
        contactRecorder(contacts);

        ::physx::PxCache contactCache;

        ::physx::PxPhysics* pxPhysics = reinterpret_cast<::physx::PxPhysics*>(physXInterface->getPhysXPtr(pxr::SdfPath(), omni::physx::ePTPhysics));
        struct ContactCacheAllocator : ::physx::PxCacheAllocator
        {
            ::physx::PxAllocatorCallback& allocator;
            ::physx::PxU8* buffer = nullptr;
            ContactCacheAllocator(::physx::PxAllocatorCallback& _allocator)
                :
                allocator(_allocator)
            {}
            ~ContactCacheAllocator()
            {
                if (buffer) allocator.deallocate(buffer);
            }
            virtual ::physx::PxU8* allocateCacheData(const ::physx::PxU32 byteSize)
            {
                if (buffer) allocator.deallocate(buffer);
                buffer = (::physx::PxU8*)allocator.allocate(byteSize, "ContactCacheAllocator", __FILE__, __LINE__);
                return buffer;
            }
        }
        contactCacheAllocator(pxPhysics->getFoundation().getAllocatorCallback());

        ::physx::immediate::PxGenerateContacts(geom0.data(), geom1.data(), pose0.data(), pose1.data(), &contactCache, pairCount, contactRecorder, contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);

        CARB_ASSERT(contacts.size() == 1);

        db.outputs.contactCount() = (int)contacts[0].size();
        db.outputs.contactPoints().resize(contacts[0].size());
        db.outputs.contactNormals().resize(contacts[0].size());
        db.outputs.contactDepths().resize(contacts[0].size());
        for (size_t i = 0; i < contacts[0].size(); ++i)
        {
            db.outputs.contactPoints()[i] = omni::physx::toFloat3(contacts[0][i].point);
            db.outputs.contactNormals()[i] = omni::physx::toFloat3(contacts[0][i].normal);
            db.outputs.contactDepths()[i] = contacts[0][i].separation;
        }

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
