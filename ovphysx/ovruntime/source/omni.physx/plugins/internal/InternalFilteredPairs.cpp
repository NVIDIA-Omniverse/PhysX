// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "InternalFilteredPairs.h"
#include "InternalPhysXDatabase.h"
#include <PhysXTools.h>
#include <usdLoad/AttachedStage.h>
#include <usdLoad/FilteredPairs.h>

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physx/IPhysx.h>

#include <PxPhysicsAPI.h>

using namespace carb;
using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;


// collection objects that do have filtered data
void collectFilteredObjects(std::vector<PxBase*>& filteredObjects, const InternalDatabase::Record& objectRecord)
{
    switch (objectRecord.mType)
    {
    case ePTShape:
    {
        filteredObjects.push_back((PxShape*)objectRecord.mPtr);
        break;
    }
    case ePTCompoundShape:
    {
        const PhysXCompoundShape* compoundShape = (PhysXCompoundShape*)objectRecord.mPtr;
        filteredObjects.resize(compoundShape->getShapes().size());
        memcpy(&filteredObjects[0], &compoundShape->getShapes()[0], sizeof(void*) * compoundShape->getShapes().size());
        break;
    }
    case ePTActor:
    case ePTLink:
    {
        const PxRigidActor* actor = (const PxRigidActor*)objectRecord.mPtr;
        PxU32 numShapes = actor->getNbShapes();
        filteredObjects.resize(numShapes);
        actor->getShapes((PxShape**)(&filteredObjects[0]), numShapes);
        break;
    }
    case ePTDeformableVolume:
    {
        const PxDeformableVolume* deformableVolume = (PxDeformableVolume*)objectRecord.mPtr;
        filteredObjects.push_back(const_cast<PxDeformableVolume*>(deformableVolume)->getShape());
        break;
    }
    case ePTDeformableSurface:
    {
        const PxDeformableSurface* deformableSurface = (PxDeformableSurface*)objectRecord.mPtr;
        filteredObjects.push_back(const_cast<PxDeformableSurface*>(deformableSurface)->getShape());
        break;
    }
    case ePTParticleSystem:
    {
        filteredObjects.push_back((PxBase*)objectRecord.mPtr);
        break;
    }
    case ePTArticulation:
    {
        const PxArticulationReducedCoordinate* articulation = (const PxArticulationReducedCoordinate*)objectRecord.mPtr;
        PxU32 numLinks = articulation->getNbLinks();
        PxU32 numShapes = 0;
        for (PxU32 i = 0; i < numLinks; ++i)
        {
            PxArticulationLink* link;
            articulation->getLinks(&link, 1, i);
            numShapes += link->getNbShapes();
        }
        filteredObjects.resize(numShapes);
        numShapes = 0;
        for (PxU32 i = 0; i < numLinks; ++i)
        {
            PxArticulationLink* link;
            articulation->getLinks(&link, 1, i);
            PxU32 numLinkShapes = link->getNbShapes();
            link->getShapes((PxShape**)(&filteredObjects[numShapes]), numLinkShapes, 0);
            numShapes += numLinkShapes;
        }
        break;
    }
    default:
        break;
    }
}

void setFilterPairData(PxBase* basePtr, uint32_t filterPair)
{
    PxShape* shape = basePtr->is<PxShape>();
    if (shape)
    {
        PxFilterData fd = shape->getSimulationFilterData();
        convertFilterPairToPxFilterData(filterPair, fd);
        shape->setSimulationFilterData(fd);
    }
    else if (basePtr->getConcreteType() == PxConcreteType::ePBD_PARTICLESYSTEM)
    {
        PxPBDParticleSystem* ps = (PxPBDParticleSystem*)basePtr;
        PxFilterData fd = ps->getSimulationFilterData();
        convertFilterPairToPxFilterData(filterPair, fd);
        ps->setSimulationFilterData(fd);
    }
}

void resetFiltering(PxBase* basePtr)
{
    PxShape* shape = basePtr->is<PxShape>();
    if (shape)
    {
        if (shape->getActor() && shape->getActor()->getScene())
        {
            shape->getActor()->getScene()->resetFiltering(*shape->getActor());
        }
    }
}

void handleFilteringPair(size_t index0, size_t index1)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    InternalDatabase::Record& objectRecord0 = db.getRecords()[index0];
    InternalDatabase::Record& objectRecord1 = db.getRecords()[index1];

    std::vector<PxBase*> objects0;
    std::vector<PxBase*> objects1;
    collectFilteredObjects(objects0, objectRecord0);
    collectFilteredObjects(objects1, objectRecord1);

    for (size_t i = 0; i < objects0.size(); ++i)
    {
        if (!objects0[i])
            continue;

        const uint32_t hash0 = ::physx::PxComputeHash(uint64_t(objects0[i]));
        setFilterPairData(objects0[i], hash0);
        resetFiltering(objects0[i]);
        for (size_t j = 0; j < objects1.size(); ++j)
        {
            if (!objects1[j])
                continue;

            const uint32_t hash1 = ::physx::PxComputeHash(uint64_t(objects1[j]));
            setFilterPairData(objects1[j], hash1);
            resetFiltering(objects1[j]);
            const Pair<uint32_t> filterPair(hash0, hash1);
            physxSetup.addFilteredPair(filterPair);
        }
    }
}

void removeFilteringPair(size_t index0, size_t index1)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    PhysXSetup& physxSetup = OmniPhysX::getInstance().getPhysXSetup();
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    if (index0 < db.getRecords().size() && index1 < db.getRecords().size())
    {
        InternalDatabase::Record& objectRecord0 = db.getRecords()[index0];
        InternalDatabase::Record& objectRecord1 = db.getRecords()[index1];

        if (objectRecord0.mType != ePTRemoved && objectRecord1.mType != ePTRemoved)
        {

            std::vector<PxBase*> objects0;
            std::vector<PxBase*> objects1;
            collectFilteredObjects(objects0, objectRecord0);
            collectFilteredObjects(objects1, objectRecord1);

            for (size_t i = 0; i < objects0.size(); ++i)
            {
                if (!objects0[i])
                    continue;

                const uint32_t hash0 = ::physx::PxComputeHash(uint64_t(objects0[i]));
                resetFiltering(objects0[i]);
                for (size_t j = 0; j < objects1.size(); ++j)
                {
                    if (!objects1[j])
                        continue;

                    const uint32_t hash1 = ::physx::PxComputeHash(uint64_t(objects1[j]));
                    resetFiltering(objects1[j]);
                    const Pair<uint32_t> filterPair(hash0, hash1);
                    physxSetup.removeFilteredPair(filterPair);
                }
            }
        }
    }
}


void InternalFilteredPairs::createFilteredPairs()
{
    for (const ObjectIdPair& pair : mPairs)
    {
        handleFilteringPair(size_t(pair.first), size_t(pair.second));
    }
}

void InternalFilteredPairs::removeFilteredPairs()
{
    for (const ObjectIdPair& pair : mPairs)
    {
        removeFilteringPair(size_t(pair.first), size_t(pair.second));
    }
}

void omni::physx::internal::changeFilteredPairs(usdparser::AttachedStage& attachedStage, omni::physics::parse::ObjectKey key, bool removed)
{
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

    if (removed)
    {
        ObjectIdMap* entries = attachedStage.getObjectDatabase()->getEntries(key);
        if (entries && !entries->empty())
        {
            auto it = entries->begin();
            while (it != entries->end())
            {
                const InternalDatabase::Record& rec = db.getRecords()[it->second];
                if (rec.mType == ePTFilteredPair)
                {
                    attachedStage.getPhysXPhysicsInterface()->releaseObject(attachedStage, key, it->second);
                    it = entries->erase(it);
                }
                else
                    it++;
            }
        }
        attachedStage.getObjectDatabase()->removeSchemaAPI(key, SchemaAPIFlag::eFilteredPairsAPI);
    }
    else
    {
        const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource();
        omni::physics::parse::KnownTokens tok;
        if (source)
            tok.intern(*source);
        if (source && source->hasSchema(key, tok.physicsFilteredPairsAPI))
        {
            InternalFilteredPairs* intPairs = ICE_NEW(InternalFilteredPairs);

            // Source-routed relationship read (TokenId, no TfToken literal); collectFilteredPairs
            // (usdLoad/FilteredPairs.h) is ObjectKey-typed (ADR-0019), so the targets pass
            // through directly with no pathFor round-trip.
            std::vector<omni::physics::parse::ObjectKey> targetKeys;
            source->getRelationshipTargets(key, tok.physicsFilteredPairs, targetKeys);
            collectFilteredPairs(attachedStage, key, targetKeys, intPairs->mPairs);
            intPairs->createFilteredPairs();
            const ObjectId outId = db.addRecord(ePTFilteredPair, nullptr, intPairs, key);
            attachedStage.getObjectDatabase()->findOrCreateEntry(key, attachedStage.textFor(key), eFilteredPair, outId);
            attachedStage.getObjectDatabase()->addSchemaAPI(key, SchemaAPIFlag::eFilteredPairsAPI);
        }
    }
}
