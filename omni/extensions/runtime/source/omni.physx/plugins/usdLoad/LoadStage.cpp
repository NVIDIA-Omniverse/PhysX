// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on
#include <carb/logging/Log.h>
#include <carb/Framework.h>
#include <private/omni/physx/PhysxUsd.h>
#include <omni/physx/IPhysxSettings.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/PrimUtilities.h>
#include <common/utilities/Utilities.h>
#include <common/utilities/OmniPhysXUtilities.h>
#include <private/omni/physics/schema/DescCache.h>
#include <omni/log/ILog.h>
#include <carb/profiler/Profile.h>
#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>
#include <usdrt/scenegraph/usd/usd/stage.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "Mass.h"
#include "Collision.h"
#include "Particles.h"
#include "Articulation.h"
#include "PointInstancer.h"
#include "JointInstancer.h"
#include "Material.h"
#include "Scene.h"
#include "Joint.h"
#include "PhysicsBody.h"
#include "Cct.h"
#include "CollisionGroup.h"
#include "LoadStage.h"
#include <CookingDataAsync.h>
#include <utils/Profile.h>
#include <PhysXScene.h>
#include <OmniPhysX.h>
#include <VoxelMap.h>
#include <attachment/PhysXAttachmentDeprecated.h>
#include <attachment/PhysXAttachment.h>
#include <particles/PhysXParticleSampling.h>
#include "ReplicatorRange.h"


// physx specific stuff
#include "FixedTendon.h"
#include "SpatialTendon.h"
#include "Vehicle.h"
#include "AttachmentDeprecated.h"
#include "SoftBodyDeprecated.h"
#include "DeformableAttachment.h"
#include "MimicJoint.h"

using namespace pxr;
using namespace carb;
using namespace carb::tasking;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

bool doesBodyExist(AttachedStage& attachedStage, SdfPath& bodyPath)
{
    const ObjectId body = attachedStage.getObjectDatabase()->findEntry(bodyPath, eBody);
    if (body != kInvalidObjectId)
        return true;

    return false;
}

void createJoint(AttachedStage& attachedStage, const SdfPath& primPath, PhysxJointDesc* desc)
{
    if (desc != nullptr)
    {
        ObjectDb* objectDb = attachedStage.getObjectDatabase();
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        ObjectId body0 = objectDb->findEntry(desc->body0, eBody);
        if (body0 == kInvalidObjectId)
        {
            body0 = objectDb->findEntry(desc->body0, eArticulationLink);
        }
        const bool body0Dynamic = body0 == kInvalidObjectId ? false : isRigidBodyDynamic(body0);
        ObjectId body1 = objectDb->findEntry(desc->body1, eBody);
        if (body1 == kInvalidObjectId)
        {
            body1 = objectDb->findEntry(desc->body1, eArticulationLink);
        }
        const bool body1Dynamic = body1 == kInvalidObjectId ? false : isRigidBodyDynamic(body1);

        createJoint(attachedStage, primPath, desc, body0, body0Dynamic, body1, body1Dynamic);
        ICE_FREE(desc);
    }
}

void createBodies(AttachedStage& attachedStage, BodyMap& bodyMap, BodyVector& additonalBodies)
{
    // create rigid bodies
    BodyMap::const_iterator it = bodyMap.begin();
    while (it != bodyMap.end())
    {
        PhysxRigidBodyDesc* bodyDesc = it->second.desc;
        if (bodyDesc)
        {

            // we dont send articulation links, they already have been created
            if (bodyDesc->type == eDynamicBody || bodyDesc->type == eStaticBody)
            {
                const SdfPath& bodyPath = it->first;
                const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyPath, *bodyDesc);
                // If PhysXUsdPhysicsInterface::setForceParseOnlySingleScene is used it may return kInvalidObjectId
                if (id != kInvalidObjectId)
                {
                    attachedStage.getObjectDatabase()->findOrCreateEntry(bodyPath, eBody, id);
                    if (bodyDesc->type == eDynamicBody)
                    {
                        attachedStage.bufferRequestRigidBodyMassUpdate(attachedStage.getStage()->GetPrimAtPath(bodyPath));
                    }
                }

            }
            ICE_FREE(bodyDesc);
        }

        it++;
    }

    bodyMap.clear();

    // Additional bodies
    for (std::pair<SdfPath, BodyDescAndColliders>& ref : additonalBodies)
    {
        PhysxRigidBodyDesc* bodyDesc = ref.second.desc;

        // we should have only static bodies here
        if (bodyDesc->type == eStaticBody)
        {
            const SdfPath& bodyPath = ref.first;
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyPath, *bodyDesc);

            attachedStage.getObjectDatabase()->findOrCreateEntry(bodyPath, eBody, id);
        }
        ICE_FREE(bodyDesc);
    }
    additonalBodies.clear();
}

void createJoints(AttachedStage& attachedStage, const JointVector& joints, bool initialStageLoad)
{
    std::vector<JointVector::const_iterator> jointsToAnalize2ndPass;
    // create joints, exclude gear and rack and pinion, create them in a second pass
    const size_t nbJoints = joints.size();
    for(size_t i=0;i<nbJoints;i++)
    {
        bool skipJoint = joints[i].articulationJoint;  // they get created as part of the articulation
        if ((!initialStageLoad) && (!skipJoint))
        {
            // note: articulationJoint is false if a structural change on an articulation joint prim
            // happened because only that prim might get reparsed and not the whole articulation. If
            // there still is an articulation joint, re-create the internal joint object. This is needed
            // for mimic joints, for example, as they get applied to joints and can trigger structural
            // changes on the prim without affecting the regular joint setup as such. Think of the
            // following example scenario that would fail without this extra logic:
            // - mimic joint API is added => structural change to trigger parsing
            // - InternalJoint object gets deleted (but not the PhysX articulation joint)
            // - joint prim is parsed
            // - InternalJoint object is not re-created because not marked as articulation joint
            //   (plus the PhysX articulation joint is still there anyway)
            // - mimic joint creation fails because there is no InternalJoint object
            // OM-123103 was created to re-visit this

            PhysxJointDesc& jointDesc = *joints[i].desc;

            if ((!jointDesc.excludedFromArticulation) && jointDesc.jointEnabled)
            {
                ObjectDb* objectDb = attachedStage.getObjectDatabase();
                ObjectId link0Id = objectDb->findEntry(jointDesc.body0, eArticulationLink);
                if (link0Id != kInvalidObjectId)
                {
                    ObjectId link1Id = objectDb->findEntry(jointDesc.body1, eArticulationLink);
                    if (link1Id != kInvalidObjectId)
                    {
                        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
                        physInt->recreateArticulationJoint(attachedStage, jointDesc, link0Id, link1Id);
                        ICE_FREE_BASIC(&jointDesc);
                        skipJoint = true;
                    }
                }
            }
        }
        else
        {
            if (skipJoint)
            {
                ICE_FREE_BASIC(joints[i].desc);
            }
        }

        if (!skipJoint)
        {
            if (joints[i].desc->jointFriction != 0.0)
            {
                CARB_LOG_WARN("Joint friction attribute is only applied for joints in articulations. (%s)",
                              joints[i].path.GetText());
            }
            if (joints[i].desc->type != eJointGear && joints[i].desc->type != eJointRackAndPinion)
            {
                createJoint(attachedStage, joints[i].path, joints[i].desc);
            }
            else
            {
                // If we happen to be doing a 2nd pass, we want to make sure we only allocate once
                jointsToAnalize2ndPass.reserve(joints.size());
                jointsToAnalize2ndPass.push_back(joints.begin() + i);
            }
        }
    }
    const size_t nbJoints2ndPass = jointsToAnalize2ndPass.size();
    for (size_t i = 0; i < nbJoints2ndPass; i++)
    {
        // Here we are not articulationJoint and we are either eJointGear or eJointRackAndPinion
        createJoint(attachedStage, jointsToAnalize2ndPass[i]->path, jointsToAnalize2ndPass[i]->desc);
    }
}

void createMimicJoints(AttachedStage& attachedStage, MimicJointVector& mimicJoints)
{
    for (MimicJointDesc& desc : mimicJoints)
    {
        createMimicJoint(attachedStage, desc);
    }
}

void createPhysxAttachmentDeprecated(AttachedStage& attachedStage, const SdfPath& primPath, PhysxAttachmentDesc* desc)
{
    if (desc != nullptr)
    {
        ObjectId id = kInvalidObjectId;

        ObjectDb* objectDb = attachedStage.getObjectDatabase();
        PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();

        for (int i = 0; i < 2; i++)
        {
            {
                ObjectId actor0 = objectDb->findEntry(desc->actor[0].path, eSoftBody);
                if (actor0 != kInvalidObjectId)
                {
                    ObjectType actorType = eUndefined;
                    ObjectId actor1 = kInvalidObjectId;

                    // Check if attachment is attached to world space
                    if (desc->actor[1].path.IsEmpty())
                    {
                        actorType = ePhysXAttachmentTargetWorld;
                    }
                    else
                    {
                        constexpr ObjectType actor1Types[] = { eDynamicBody, eArticulationLink, eSoftBody, eParticleCloth, eFEMCloth, eShape };

                        for (ObjectType actor1Type : actor1Types)
                        {
                            actor1 = objectDb->findEntry(desc->actor[1].path, actor1Type);
                            if (actor1 != kInvalidObjectId)
                            {
                                actorType = actor1Type;
                                break;
                            }
                        }
                    }

                    if (actorType != eUndefined)
                    {
                        desc->actor[0].objId = actor0;
                        desc->actor[0].objType = eSoftBody;
                        desc->actor[1].objId = actor1;
                        desc->actor[1].objType = actorType;

                        id = physInt->createObject(attachedStage, primPath, *desc);
                        break;
                    }
                }
            }

            {
                ObjectId actor0 = objectDb->findEntry(desc->actor[0].path, eParticleCloth);
                if (actor0 != kInvalidObjectId)
                {
                    ObjectType actorType = eUndefined;
                    ObjectId actor1 = kInvalidObjectId;

                    // Check if attachment is attached to world space
                    if (desc->actor[1].path.IsEmpty())
                    {
                        actorType = ePhysXAttachmentTargetWorld;
                    }
                    else
                    {
                        constexpr ObjectType actor1Types[] = { eDynamicBody, eArticulationLink, eShape };

                        for (ObjectType actor1Type : actor1Types)
                        {
                            actor1 = objectDb->findEntry(desc->actor[1].path, actor1Type);
                            if (actor1 != kInvalidObjectId)
                            {
                                actorType = actor1Type;
                                break;
                            }
                        }
                    }

                    if (actorType != eUndefined)
                    {
                        // ParticleCloth-RigidBody attachment
                        desc->actor[0].objId = actor0;
                        desc->actor[0].objType = eParticleCloth;
                        desc->actor[1].objId = actor1;
                        desc->actor[1].objType = actorType;

                        id = physInt->createObject(attachedStage, primPath, *desc);
                        break;
                    }
                }
            }

            {
                ObjectId actor0 = objectDb->findEntry(desc->actor[0].path, eFEMCloth);
                if (actor0 != kInvalidObjectId)
                {
                    ObjectType actorType = eUndefined;
                    ObjectId actor1 = kInvalidObjectId;

                    // Check if attachment is attached to world space
                    if (desc->actor[1].path.IsEmpty())
                    {
                        actorType = ePhysXAttachmentTargetWorld;
                    }
                    else
                    {
                        constexpr ObjectType actor1Types[] = { eDynamicBody, eArticulationLink, eShape };

                        for (ObjectType actor1Type : actor1Types)
                        {
                            actor1 = objectDb->findEntry(desc->actor[1].path, actor1Type);
                            if (actor1 != kInvalidObjectId)
                            {
                                actorType = actor1Type;
                                break;
                            }
                        }
                    }

                    if (actorType != eUndefined)
                    {
                        // DeformableSurface-RigidBody attachment
                        desc->actor[0].objId = actor0;
                        desc->actor[0].objType = eFEMCloth;
                        desc->actor[1].objId = actor1;
                        desc->actor[1].objType = actorType;

                        id = physInt->createObject(attachedStage, primPath, *desc);
                        break;
                    }
                }
            }

            // Swap the actors
            std::swap(desc->actor[0], desc->actor[1]);
        }

        if (id != kInvalidObjectId)
            objectDb->findOrCreateEntry(primPath, ePhysxAttachment, id);

        ICE_FREE(desc);
    }
}


void createPhysxAttachmentsDeprecated(AttachedStage& attachedStage, const PhysxAttachmentVectorDeprecated& attachments)
{
    UsdStageWeakPtr stage = attachedStage.getStage();

    const size_t nbAttachments = attachments.size();
    for (size_t i = 0; i < nbAttachments; i++)
    {
        UsdPrim prim = stage->GetPrimAtPath(attachments[i]);

        if (prim.HasAPI<PhysxSchemaPhysxAutoAttachmentAPI>())
        {
            computeAttachmentPointsDeprecated(prim.GetPath());
        }

        PhysxAttachmentDesc* desc = parsePhysxAttachmentDeprecated(stage, prim);

        createPhysxAttachmentDeprecated(attachedStage, attachments[i], desc);
    }
}

void createDeformableBody(AttachedStage& attachedStage, PhysxDeformableBodyDesc* bodyDesc, SdfPath bodyPath)
{
    if (bodyDesc)
    {
        if (bodyDesc->type == eVolumeDeformableBody ||
            bodyDesc->type == eSurfaceDeformableBody)
        {
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, bodyPath, *bodyDesc);
            // If PhysXUsdPhysicsInterface::setForceParseOnlySingleScene is used it may return kInvalidObjectId
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(bodyPath, bodyDesc->type, id);
            }
        }
        ICE_FREE(bodyDesc);
    }
}

void createDeformableAttachments(AttachedStage& attachedStage, DeformableAttachmentVector& attachments)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
    ObjectId id = kInvalidObjectId;

    UsdStageWeakPtr stage = attachedStage.getStage();

    const size_t nbAttachments = attachments.size();
    for (size_t i = 0; i < nbAttachments; i++)
    {
        if (attachments[i].desc == nullptr)
        {
            omni::physics::usdparser::IUsdPhysicsParse* parser = OmniPhysX::getInstance().getIUsdPhysicsParse();
            omni::physics::schema::AttachmentDesc* desc = parser->parseAttachment(attachedStage.getStageId(), attachments[i].path);
            if (desc == nullptr)
                return;

            attachments[i].desc = parseDeformableAttachment(attachedStage.getStage(), *desc);
            parser->releaseDesc(desc);
        }

        id = physInt->createObject(attachedStage, attachments[i].path, *attachments[i].desc);

        if (id != kInvalidObjectId)
            objectDb->findOrCreateEntry(attachments[i].path, attachments[i].desc->type, id);

        ICE_FREE(attachments[i].desc);
    }
}

void createDeformableCollisionFilters(AttachedStage& attachedStage, DeformableCollisionFilterVector& collisionFilters)
{
    ObjectDb* objectDb = attachedStage.getObjectDatabase();
    PhysXUsdPhysicsInterface* physInt = attachedStage.getPhysXPhysicsInterface();
    ObjectId id = kInvalidObjectId;

    UsdStageWeakPtr stage = attachedStage.getStage();

    const size_t nbCollisionFilters = collisionFilters.size();
    for (size_t i = 0; i < nbCollisionFilters; i++)
    {
        if (collisionFilters[i].desc == nullptr)
        {
            omni::physics::usdparser::IUsdPhysicsParse* parser = OmniPhysX::getInstance().getIUsdPhysicsParse();
            omni::physics::schema::ElementCollisionFilterDesc* desc = parser->parseCollisionFilter(attachedStage.getStageId(), collisionFilters[i].path);
            if (desc == nullptr)
                return;

            collisionFilters[i].desc = parseDeformableCollisionFilter(attachedStage.getStage(), *desc);
            parser->releaseDesc(desc);
        }

        id = physInt->createObject(attachedStage, collisionFilters[i].path, *collisionFilters[i].desc);

        if (id != kInvalidObjectId)
            objectDb->findOrCreateEntry(collisionFilters[i].path, collisionFilters[i].desc->type, id);

        ICE_FREE(collisionFilters[i].desc);
    }
}

void createFilteredPairs(AttachedStage& attachedStage, const CollisionPairVector& pairsVector, const SdfPathVector& filteredPairsPaths)
{
    ObjectDb& objectDb = *attachedStage.getObjectDatabase();
    std::unordered_map<SdfPath, FilteredPairDesc, SdfPath::Hash> blockDescMap;
    for (const SdfPath& path : filteredPairsPaths)
    {
        blockDescMap[path] = FilteredPairDesc();
    }

    for (size_t iPairs = pairsVector.size(); iPairs--;)
    {
        const CollisionBlockPair& pair = pairsVector[iPairs];        

        FilteredPairDesc& blockDesc = blockDescMap[pair.first];
        const ObjectIdMap* entriesFirst = objectDb.getEntries(pair.first);
        if (entriesFirst && !entriesFirst->empty())
        {
            auto itFirst = entriesFirst->begin();
            while (itFirst != entriesFirst->end())
            {
                const ObjectIdMap* entriesSecond = objectDb.getEntries(pair.second);
                if (entriesSecond && !entriesSecond->empty())
                {
                    auto itSecond = entriesSecond->begin();
                    while (itSecond != entriesSecond->end())
                    {
                        const ObjectId firstObject = itFirst->second;
                        const ObjectId secondObject = itSecond->second;
                        blockDesc.pairs.push_back(std::make_pair(firstObject, secondObject));
                        itSecond++;
                    }
                }
                else
                {
                    // traverse and find the childs
                    UsdPrimRange range(attachedStage.getStage()->GetPrimAtPath(pair.second));
                    for (UsdPrimRange::const_iterator iterChilds = range.begin(); iterChilds != range.end();
                        ++iterChilds)
                    {
                        const UsdPrim& childUsd = *iterChilds;
                        if (!childUsd)
                            continue;
                        bool pairFound = false;
                        const ObjectIdMap* entriesSecond = objectDb.getEntries(childUsd.GetPath());
                        if (entriesSecond && !entriesSecond->empty())
                        {
                            auto itSecond = entriesSecond->begin();
                            while (itSecond != entriesSecond->end())
                            {
                                if (itSecond->first == eBody || itSecond->first == eShape)
                                {
                                    pairFound = true;
                                    const ObjectId firstObject = itFirst->second;
                                    const ObjectId secondObject = itSecond->second;
                                    blockDesc.pairs.push_back(std::make_pair(firstObject, secondObject));
                                }
                                itSecond++;
                            }
                        }
                        if (pairFound)
                            iterChilds.PruneChildren();
                    }
                }
                itFirst++;
            }
        }
    }

    for (auto& blockDesc : blockDescMap)
    {
        const ObjectId retId = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, blockDesc.first, blockDesc.second);
        if (retId != kInvalidObjectId)
        {
            objectDb.findOrCreateEntry(blockDesc.first, eFilteredPair, retId);
        }
    }
}

ObjectId createObject(AttachedStage& attachedStage, const SdfPath& primPath, PhysxObjectDesc* desc,
    PhysXUsdPhysicsInterface& physicsInterface, ObjectDb& objectDb)
{
    if (!desc)
        return kInvalidObjectId;

    const ObjectId id = physicsInterface.createObject(attachedStage, primPath, *desc);
    if (id != kInvalidObjectId)
        objectDb.findOrCreateEntry(primPath, desc->type, id);
    return id;
}

ObjectId createObject(AttachedStage& attachedStage, const SdfPath& primPath, PhysxObjectDesc* desc, bool deleteDesc = true)
{
    if (!desc)
        return kInvalidObjectId;

    const ObjectId id = createObject(attachedStage, primPath, desc, *attachedStage.getPhysXPhysicsInterface(), *attachedStage.getObjectDatabase());
    if (deleteDesc)
        ICE_FREE(desc);
    return id;
}

void loadPhysicsFromPrimInternal(AttachedStage& attachedStage,
                                 const UsdPrim& prim,                                 
                                 CollisionPairVector& filteredPairs)
{
    const SdfPath& primPath = prim.GetPrimPath();
    UsdStageWeakPtr stage = attachedStage.getStage();

    // DEPRECATED
    if (prim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
    {
        cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        CARB_ASSERT(cookingDataAsync);

        // make sure simulation mesh is written by the async cooker
        cookingDataAsync->cookDeformableBodyTetMeshDeprecated(const_cast<UsdPrim&>(prim), false);
        auto softBodyDesc = ParseDeformableBodyDeprecated(attachedStage, prim);
        if (softBodyDesc)
        {
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, primPath, *softBodyDesc);
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(primPath, eSoftBody, id);
                attachedStage.getObjectDatabase()->addSchemaAPI(primPath, SchemaAPIFlag::eDeformableBodyAPIdeprecated);

                for (size_t i = 0; i < softBodyDesc->filteredCollisions.size(); i++)
                {
                    filteredPairs.push_back(std::make_pair(primPath, softBodyDesc->filteredCollisions[i]));
                }
            }
            ICE_FREE(softBodyDesc);
        }
    }
    // DEPRECATED
    else if (prim.HasAPI<PhysxSchemaPhysxDeformableSurfaceAPI>())
    {
        auto femClothDesc = ParseDeformableSurfaceDeprecated(attachedStage, prim);
        if (femClothDesc)
        {
            const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, primPath, *femClothDesc);
            if (id != kInvalidObjectId)
            {
                attachedStage.getObjectDatabase()->findOrCreateEntry(primPath, eFEMCloth, id);
                attachedStage.getObjectDatabase()->addSchemaAPI(primPath, SchemaAPIFlag::eDeformableSurfaceAPIdeprecated);

                for (size_t i = 0; i < femClothDesc->filteredCollisions.size(); i++)
                {
                    filteredPairs.push_back(std::make_pair(primPath, femClothDesc->filteredCollisions[i]));
                }
            }
            ICE_FREE(femClothDesc);
        }
    }
    else if (prim.HasAPI<PhysxSchemaPhysxCharacterControllerAPI>())
    {
        CctDesc* desc = parseCct(attachedStage, prim);
        if (desc)
        {
            createObject(attachedStage, primPath, desc);
        }
    }
}

void setupCollisionGroups(AttachedStage& attachedStage, const std::vector<UsdPrim>& collisionGroupsPrims)
{
    const ObjectDb& db = *attachedStage.getObjectDatabase();
    const UsdStageWeakPtr stage = attachedStage.getStage();

    std::vector<UsdPrim>::const_iterator it = collisionGroupsPrims.begin();

    while (it != collisionGroupsPrims.end())
    {
        const UsdPrim& collisionGroupPrim = (*it);
        const SdfPath& collisionGroupPath = collisionGroupPrim.GetPrimPath();
        const ObjectIdMap* map = db.getEntries(collisionGroupPath);
        CollisionGroupDesc desc;
        CARB_ASSERT(map);
        if (map)
        {
            CARB_ASSERT(map->begin()->first == eCollisionGroup);
            const ObjectId groupId = map->begin()->second;
            desc.groupId = groupId;
            UsdPhysicsCollisionGroup cg = UsdPhysicsCollisionGroup(collisionGroupPrim);
            CARB_ASSERT(cg);
            SdfPathVector targets;
            if (cg.GetFilteredGroupsRel())
            {
                cg.GetFilteredGroupsRel().GetTargets(&targets);
            }
            for (size_t iTargets = 0; iTargets < targets.size(); iTargets++)
            {
                const SdfPath& filterPath = targets[iTargets];
                const ObjectIdMap* filterMap = db.getEntries(filterPath);
                if (filterMap && !filterMap->empty())
                {
                    ObjectIdMap::const_iterator itSecond = filterMap->begin();
                    while (itSecond != filterMap->end())
                    {
                        if (itSecond->first == eCollisionGroup)
                            desc.filteredGroups.push_back(itSecond->second);
                        itSecond++;
                    }
                }
            }
            attachedStage.getPhysXPhysicsInterface()->setupCollisionGroup(collisionGroupPath, desc);
        }
        it++;
    }
}

void createParticleSystemsAndObjects(AttachedStage& attachedStage, const std::vector<ParticleSystemDesc*>& particleSysDescs, const std::vector<ParticleDesc*>& particleDescs,
    CollisionPairVector& filteredPairs)
{
    // create particle systems
    for (ParticleSystemDesc* particleSys : particleSysDescs)
    {
        createObject(attachedStage, particleSys->systemPath, particleSys, false);

        for (size_t i = 0; i < particleSys->filteredCollisions.size(); i++)
        {
            filteredPairs.push_back(std::make_pair(particleSys->systemPath, particleSys->filteredCollisions[i]));
        }
        ICE_FREE(particleSys)
    }

    // create particles
    for (ParticleDesc* particleDesc : particleDescs)
    {
        createObject(attachedStage, particleDesc->primPath, particleDesc);
    }
}

omni::physx::usdparser::ObjectId findOrCreatePhysXObject(AttachedStage& attachedStage, const SdfPath& path, PhysxObjectDesc* objectDesc,
    PhysXUsdPhysicsInterface& physicsInterface, ObjectDb& objectDb)
{
    ObjectId objectId = objectDb.findEntry(path, objectDesc->type);
    if (objectId != kInvalidObjectId)
        return objectId;
    else
        return createObject(attachedStage, path, objectDesc, physicsInterface, objectDb);
}

void loadVehicle(AttachedStage& attachedStage, const UsdPrim& prim, VehicleComponentTracker& vehicleComponentTracker,
    UsdGeomXformCache& xfCache)
{
    const SdfPath& primPath = prim.GetPrimPath();

    VehicleDesc vehicleDesc;
    VehicleControllerDesc vehicleControllerDesc;
    VehicleTankControllerDesc vehicleTankControllerDesc;
    ObjectType vehicleControllerType;
    if (parseVehicle(attachedStage.getStage(), prim, vehicleDesc,
        vehicleControllerDesc, vehicleTankControllerDesc, vehicleControllerType,
        vehicleComponentTracker, xfCache))
    {
        PhysXUsdPhysicsInterface& physicsInterface = *attachedStage.getPhysXPhysicsInterface();
        ObjectDb& objectDb = *attachedStage.getObjectDatabase();

        // create components, if they do not exist yet
        if (vehicleDesc.drive)
        {
            if (vehicleDesc.drive->type == ObjectType::eVehicleDriveStandard)
            {
                DriveStandardDesc* driveDesc = static_cast<DriveStandardDesc*>(vehicleDesc.drive);
                driveDesc->engineId = findOrCreatePhysXObject(attachedStage, driveDesc->engine->path, driveDesc->engine,
                    physicsInterface, objectDb);
            }
            else
            {
                CARB_ASSERT(vehicleDesc.drive->type == ObjectType::eVehicleDriveBasic);
                DriveBasicDesc* driveDesc = static_cast<DriveBasicDesc*>(vehicleDesc.drive);
                driveDesc->id = findOrCreatePhysXObject(attachedStage, driveDesc->path, driveDesc,
                    physicsInterface, objectDb);
            }
        }

        for (WheelAttachmentDesc& wheelAttachment : vehicleDesc.wheelAttachments)
        {
            wheelAttachment.id = createObject(attachedStage, wheelAttachment.path, &wheelAttachment, physicsInterface, objectDb);

            WheelDesc* wheelDesc = wheelAttachment.wheel;
            wheelAttachment.wheelId = findOrCreatePhysXObject(attachedStage, wheelDesc->path, wheelDesc,
                physicsInterface, objectDb);

            TireDesc* tireDesc = wheelAttachment.tire;
            wheelAttachment.tireId = findOrCreatePhysXObject(attachedStage,tireDesc->path, tireDesc,
                physicsInterface, objectDb);

            SuspensionDesc* suspDesc = wheelAttachment.suspension;
            wheelAttachment.suspensionId = findOrCreatePhysXObject(attachedStage, suspDesc->path, suspDesc,
                physicsInterface, objectDb);

            if (!wheelAttachment.tire->frictionTablePath.IsEmpty())
                wheelAttachment.tire->frictionTableId = objectDb.findEntry(
                    wheelAttachment.tire->frictionTablePath, eVehicleTireFrictionTable);
            else
                wheelAttachment.tire->frictionTableId = kInvalidObjectId;

            if (!wheelAttachment.collisionGroupPath.IsEmpty())
            {
                wheelAttachment.collisionGroupId = objectDb.findEntry(
                    wheelAttachment.collisionGroupPath, eCollisionGroup);
            }
            else
                wheelAttachment.collisionGroupId = kInvalidObjectId;

            if (wheelAttachment.state & WheelAttachmentDesc::eHAS_SHAPE)
                wheelAttachment.shapeId =
                    objectDb.findEntry(wheelAttachment.shapePath, eShape);
            else
                wheelAttachment.shapeId = kInvalidObjectId;
        }

        for (WheelControllerDesc& wheelController : vehicleDesc.wheelControllers)
        {
            wheelController.id = createObject(attachedStage, wheelController.path, &wheelController, physicsInterface, objectDb);
        }

        vehicleDesc.bodyId = objectDb.findEntry(primPath, eBody);

        ObjectId vehicleId = createObject(attachedStage, primPath, &vehicleDesc, physicsInterface, objectDb);

        if (vehicleId != kInvalidObjectId)
        {
            if (vehicleControllerType != eUndefined)
            {
                if (vehicleControllerType == eVehicleControllerStandard)
                    createObject(attachedStage, primPath, &vehicleControllerDesc, physicsInterface, objectDb);
                else
                {
                    CARB_ASSERT(vehicleControllerType == eVehicleControllerTank);
                    createObject(attachedStage, primPath, &vehicleTankControllerDesc, physicsInterface, objectDb);
                }
            }
        }
        else
        {
            for (WheelControllerDesc& wheelController : vehicleDesc.wheelControllers)
            {
                if (wheelController.id != kInvalidObjectId)
                {
                    physicsInterface.releaseObject(attachedStage, wheelController.path, wheelController.id);
                    wheelController.id = kInvalidObjectId;
                }
            }

            for (WheelAttachmentDesc& wheelAttachment : vehicleDesc.wheelAttachments)
            {
                if (wheelAttachment.id != kInvalidObjectId)
                {
                    physicsInterface.releaseObject(attachedStage, wheelAttachment.path, wheelAttachment.id);
                    wheelAttachment.id = kInvalidObjectId;
                }
            }
        }
    }
}

struct ParsingFlag
{
    enum Enum
    {
        eParseInternal = 1 << 0,
        eParseVehicles = 1 << 1,
        eParseParticles = 1 << 2
    };
};

inline UsdPrim createUsdPhysicsScene(SdfPath& scenePath, UsdStageRefPtr stage)
{
    UsdPhysicsScene scenePrim = UsdPhysicsScene::Define(stage, scenePath);
    PhysxSchemaPhysxSceneAPI::Apply(scenePrim.GetPrim());
    return scenePrim.GetPrim();
}

class PhysxUsdPhysicsListener: public omni::physics::schema::IUsdPhysicsListener
{
public:

    PhysxUsdPhysicsListener(AttachedStage& attachedStage)
        : mSceneFound(false), mNoValidScene(false), mNoPhysXScene(false), mCollectionsPopulated(false), mNumScenes(0), mAttachedStage(attachedStage)
        , mXfCache(UsdTimeCode::EarliestTime())
    {
        mNoPhysXScene = !attachedStage.isPhysXDefaultSimulator();
    }

    void parsePrim(const UsdPrim& prim, omni::physics::schema::ObjectDesc* objectDesc, uint64_t primTypes, const TfTokenVector& appliedApis) override
    {
        // A.B. figure out how to get the TfTokens!
        static const TfToken gDeformableBodyAPIdeprecated = TfToken("PhysxDeformableBodyAPI");
        static const TfToken gDeformableSurfaceAPIdeprecated = TfToken("PhysxDeformableSurfaceAPI");
        static const TfToken gCctAPI = TfToken("PhysxCharacterControllerAPI");

        static const TfToken gDeformableBodyMaterialAPIdeprecated = TfToken("PhysxDeformableBodyMaterialAPI");
        static const TfToken gDeformableSurfaceMaterialAPIdeprecated = TfToken("PhysxDeformableSurfaceMaterialAPI");
        static const TfToken gParticleMaterialAPI = TfToken("PhysxPBDMaterialAPI");

        static const TfToken gParticleClothAPIdeprecated = TfToken("PhysxParticleClothAPI");
        static const TfToken gParticleSetAPI = TfToken("PhysxParticleSetAPI");
        static const TfToken gParticleSamplingAPI = TfToken("PhysxParticleSamplingAPI");

        static const TfToken gVehicleAPI = TfToken("PhysxVehicleAPI");
        static const TfToken gVehicleContextAPI = TfToken("PhysxVehicleContextAPI");

        static const TfToken gCollisionAPIToken("PhysicsCollisionAPI");
        static const TfToken gFilteredPairsAPIToken("PhysicsFilteredPairsAPI");

        static const TfToken gForceAPIToken("PhysxForceAPI");
        static const TfToken gContactReportAPIToken("PhysxContactReportAPI");

        for (const TfToken& api : appliedApis)
        {
            // internal loop check
            if (api == gDeformableBodyAPIdeprecated)
            {
                mParsingFlags |= ParsingFlag::eParseInternal;
            }
            else if (api == gDeformableSurfaceAPIdeprecated)
            {
                mParsingFlags |= ParsingFlag::eParseInternal;
            }
            else if (api == gCctAPI)
            {
                mParsingFlags |= ParsingFlag::eParseInternal;
            }

            // forces
            if (api == gForceAPIToken)
            {
                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::ePhysxForceAPI);
                PhysxForceDesc* desc = parsePhysxForce(mAttachedStage, prim, mXfCache);
                mPhysxForceDescs.push_back(std::make_pair(prim, desc));
            }

            // filtered pairs
            if (api == gFilteredPairsAPIToken)
            {
                mFilteredPairsPaths.push_back(prim.GetPrimPath());
                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eFilteredPairsAPI);
            }

            // DEPRECATED
            if (api == gDeformableBodyMaterialAPIdeprecated)
            {
                FemSoftBodyMaterialDesc* desc = ParseDeformableBodyMaterialDeprecated(prim);
                mFemSoftBodyMatrialsDescs.push_back(std::make_pair(prim, desc));
            }
            else if (api == gDeformableSurfaceMaterialAPIdeprecated)
            {
                FemClothMaterialDesc* desc = ParseDeformableSurfaceMaterialDeprecated(prim);
                mFemClothMatrialsDescs.push_back(std::make_pair(prim, desc));
            }

            if (api == gParticleMaterialAPI)
            {
                PBDMaterialDesc* desc = ParsePBDParticleMaterial(prim.GetStage(), prim);
                mPDBMatrialsDescs.push_back(std::make_pair(prim, desc));
            }

            // particles loop
            if (api == gInfiniteVoxelMapAPI)
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (api == gParticleClothAPIdeprecated)
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (api == gParticleSetAPI)
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }
            else if (api == gParticleSamplingAPI)
            {
                mParsingFlags |= ParsingFlag::eParseParticles;
            }

            // vehicle loop
            if (api == gVehicleAPI)
            {
                mParsingFlags |= ParsingFlag::eParseVehicles;
            }
            if (api == gVehicleContextAPI)
            {
                VehicleContextDesc tmpVehicleContextDesc;
                if (parseVehicleContext(prim, tmpVehicleContextDesc))
                {
                    mVehicleContextDescList.push_back(tmpVehicleContextDesc);
                }
                mParsingFlags |= ParsingFlag::eParseVehicles;
            }

            if (api == gCollisionAPIToken)
            {
                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eCollisionAPI);
            }
            if (api == gContactReportAPIToken)
            {
                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eContactReportAPI);
            }
        }

        // internal check
        if (primTypes & PrimType::eUsdGeomPointInstancer)
        {
            mParsingFlags |= ParsingFlag::eParseParticles;
        }
        else if (prim.IsA<PhysxSchemaPhysxParticleSystem>())
        {
            mParsingFlags |= ParsingFlag::eParseParticles;
        }
        else if (prim.IsA <PhysxSchemaPhysxPhysicsJointInstancer>())
        {
            mParsingFlags |= ParsingFlag::eParseParticles;
        }
        else if (primTypes & PrimType::eUsdPhysicsCollisionGroup)
        {                        
            mCollisionGroupsPrims.push_back(prim);
        }
        else if (prim.IsA<PhysxSchemaPhysxVehicleTireFrictionTable>())
        {
            TireFrictionTableDesc* tireFrictionTableDesc = parseTireFrictionTable(mStage, prim);
            if (tireFrictionTableDesc)
                mTireFrictionTableDescList.push_back(tireFrictionTableDesc);
            mParsingFlags |= ParsingFlag::eParseVehicles;
        }
        else if (prim.IsA<PhysxSchemaPhysxPhysicsAttachment>())
        {
            mPhysxAttachmentVectorDeprecated.push_back(prim.GetPrimPath());
        }

        // DEPRECATED, check attachment history
        {
            AttachmentHistoryMapDeprecated& history = mAttachedStage.getAttachmentHistoryMapDeprecated();

            AttachmentHistoryMapDeprecated::const_iterator it = history.begin();
            AttachmentHistoryMapDeprecated::const_iterator itEnd = history.end();
            while (it != itEnd)
            {
                if (it->first == prim.GetPrimPath())
                {
                    mPhysxAttachmentVectorDeprecated.push_back(it->second);
                }
                it++;
            }
        }

        // check deformable attachment history
        {
            DeformableAttachmentHistoryMap& history = mAttachedStage.getDeformableAttachmentHistoryMap();

            DeformableAttachmentHistoryMap::const_iterator it = history.begin();
            DeformableAttachmentHistoryMap::const_iterator itEnd = history.end();
            while (it != itEnd)
            {
                if (it->first == prim.GetPrimPath())
                {
                    DeformableAttachmentDescAndPath attachmentDescAndPath;
                    attachmentDescAndPath.path = it->second;
                    attachmentDescAndPath.desc = nullptr;
                    attachmentDescAndPath.prim = mAttachedStage.getStage()->GetPrimAtPath(it->second);
                    mDeformableAttachmentVector.push_back(attachmentDescAndPath);
                }
                it++;
            }
        }

        // check deformable collision filter history
        {
            DeformableCollisionFilterHistoryMap& history = mAttachedStage.getDeformableCollisionFilterHistoryMap();

            DeformableCollisionFilterHistoryMap::const_iterator it = history.begin();
            DeformableCollisionFilterHistoryMap::const_iterator itEnd = history.end();
            while (it != itEnd)
            {
                if (it->first == prim.GetPrimPath())
                {
                    DeformableCollisionFilterDescAndPath collisionFilterDescAndPath;
                    collisionFilterDescAndPath.path = it->second;
                    collisionFilterDescAndPath.desc = nullptr;
                    collisionFilterDescAndPath.prim = mAttachedStage.getStage()->GetPrimAtPath(it->second);
                    mDeformableCollisionFilterVector.push_back(collisionFilterDescAndPath);
                }
                it++;
            }
        }

        if (objectDesc)
        {
            switch (objectDesc->type)
            {
            case omni::physics::schema::ObjectType::eRigidBody:
            {
                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eRigidBodyAPI);
            }
            break;
            case omni::physics::schema::ObjectType::eVolumeDeformableBody:
            case omni::physics::schema::ObjectType::eSurfaceDeformableBody:
            {
                mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eDeformableBodyAPI);
                const TfType autoType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableBodyAPI);
                if (prim.HasAPI(autoType))
                {
                    mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eAutoDeformableBodyAPI);
                    const TfType simpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableMeshSimplificationAPI);
                    if (prim.HasAPI(simpType))
                    {
                        mAttachedStage.getObjectDatabase()->addSchemaAPI(prim.GetPrimPath(), SchemaAPIFlag::eAutoDeformableMeshSimplificationAPI);
                    }
                    if (objectDesc->type == omni::physics::schema::ObjectType::eVolumeDeformableBody)
                    {
                        const TfType hexType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableHexahedralMeshAPI);
                        if (prim.HasAPI(hexType))
                        {
                            mAttachedStage.getObjectDatabase()->addSchemaAPI(
                                prim.GetPrimPath(), SchemaAPIFlag::eAutoDeformableHexahedralMeshAPI);
                        }
                    }
                }
            }
            break;
            //tendon axes not supported on fixed and spherical joints for now
            case omni::physics::schema::ObjectType::eJointFixed:
            case omni::physics::schema::ObjectType::eJointSpherical:
            {
                // A.B. multiple applied schema
                if (prim.HasAPI<PhysxSchemaPhysxTendonAxisAPI>())
                {
                    CARB_LOG_WARN("A Tendon Axis API was applied to an unsupported joint type at %s!", prim.GetPath().GetText());
                }
            }
            break;
            case omni::physics::schema::ObjectType::eJointPrismatic:
            case omni::physics::schema::ObjectType::eJointRevolute:
            {
                if (prim.HasAPI<PhysxSchemaPhysxTendonAxisAPI>())
                {
                    const JointDesc* jointDesc = static_cast<const JointDesc*>(objectDesc);
                    parseTendonAxes(mAttachedStage, prim, jointDesc, mTendonAxisMap, mFixedTendons);
                }

                parseMimicJoints(mAttachedStage.getStage(), prim, mMimicJoints);
            }
            break;
            case omni::physics::schema::ObjectType::eJointD6:
            {
                parseMimicJoints(mAttachedStage.getStage(), prim, mMimicJoints);
            }
            break;
            default:
                break;
            }
        }
    }

    void reportObjectDesc(const SdfPath& path, const omni::physics::schema::ObjectDesc* objectDesc) override
    {
        if (mNoValidScene)
            return;

        if (mNoPhysXScene && objectDesc->type != omni::physics::schema::ObjectType::eScene)
            return;

        switch (objectDesc->type)
        {
         // scene is created first
        case omni::physics::schema::ObjectType::eScene:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:scene");
            const UsdPrim scenePrim = mStage->GetPrimAtPath(path);
            if (scenePrim)
            {
                const bool isDefaultSimulator = mAttachedStage.isPhysXDefaultSimulator();
                if (omni::physx::canSceneBeProcessedByPhysX(scenePrim, &isDefaultSimulator))
                {
                    const SceneDesc* inDesc = (const SceneDesc*)objectDesc;
                    PhysxSceneDesc* desc = parseSceneDesc(mStage, *inDesc);
                    const ObjectId sceneId = createObject(mAttachedStage, path, desc);
                    if (sceneId != kInvalidObjectId)
                    {
                        mSceneFound = true;
                        mNoPhysXScene = false;
                        mNumScenes++;
                    }
                    else
                    {
                        if(OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
                        {
                            mNoValidScene = true;
                        }
                    }                    
                }
            }
        }
        break;
        // collision groups are second
        case omni::physics::schema::ObjectType::eCollisionGroup:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:collisionGroup");
            // populate collections first if not done
            if (!mCollectionsPopulated)
            {
                PHYSICS_PROFILE("updateCollisionCollection");
                const size_t minBatchSize = 20;
                const size_t collisionGroupsSize = mCollisionGroupsPrims.size();
                if (collisionGroupsSize < minBatchSize)
                {
                    for (const UsdPrim& prim : mCollisionGroupsPrims)
                    {
                        updateCollisionCollection(prim, mAttachedStage.getCollisionGroupMap());
                    }
                }
                else
                {
                    // run in parallel, limit the number of batches 
                    const size_t numBatches = 24;
                    const size_t batchSize = collisionGroupsSize / numBatches;

                    if (!mAttachedStage.getAdditionalCollisionGroupMaps().empty())
                    {
                        // merge the groups first
                        PHYSICS_PROFILE("updateCollisionCollection:merge");
                        CollisionGroupsMap& cgMap = mAttachedStage.getCollisionGroupMap();
                        for (const CollisionGroupsMap& m : mAttachedStage.getAdditionalCollisionGroupMaps())
                        {
                            cgMap.insert(m.begin(), m.end());
                        }
                        mAttachedStage.getAdditionalCollisionGroupMaps().clear();
                    }
                    std::vector<CollisionGroupsMap>& cgMaps = mAttachedStage.getAdditionalCollisionGroupMaps();
                    cgMaps.resize(numBatches);
                    auto&& computeFunc = [this, numBatches, batchSize, &cgMaps](size_t batchIndex)
                    {
                        const size_t batchEnd = batchIndex == (numBatches - 1) ? mCollisionGroupsPrims.size() : (batchIndex + 1) * batchSize;
                        for (size_t index = batchIndex * batchSize; index < batchEnd; index++)
                        {
                            const UsdPrim& p = mCollisionGroupsPrims[index];
                            updateCollisionCollection(p, cgMaps[batchIndex]);
                        }                        
                    };
                    {
                        PHYSICS_PROFILE("updateCollisionCollection:parallelFor");
                        ITasking* tasking = carb::getCachedInterface<ITasking>();
                        tasking->parallelFor(size_t(0), numBatches, computeFunc);
                    }
                }
                mCollectionsPopulated = true;
            }
            CollisionGroupDesc desc;
            createObject(mAttachedStage, path, &desc, false);
        }
        break;
        // materials are created third
        case omni::physics::schema::ObjectType::eMaterial:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:material");
            const MaterialDesc* inDesc = (const MaterialDesc*)objectDesc;
            PhysxMaterialDesc* desc = parseMaterialDesc(mStage, *inDesc);
            mMaterials.push_back(std::make_pair(path, desc));
        }
        break;
        case omni::physics::schema::ObjectType::eDeformableMaterial:
        case omni::physics::schema::ObjectType::eSurfaceDeformableMaterial:
        case omni::physics::schema::ObjectType::eCurvesDeformableMaterial:
        {
			CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:deformableMaterial");
            const DeformableMaterialDesc* inDesc = (const DeformableMaterialDesc*)objectDesc;
            PhysxDeformableMaterialDesc* desc = parseDeformableMaterialDesc(mStage, *inDesc);
            mDeformableMaterials.push_back(std::make_pair(path, desc));
        }
        break;

        // shapes are created fourth
        case omni::physics::schema::ObjectType::eSphereShape:
        case omni::physics::schema::ObjectType::eCubeShape:
        case omni::physics::schema::ObjectType::eCapsuleShape:
        case omni::physics::schema::ObjectType::eCylinderShape:
        case omni::physics::schema::ObjectType::eConeShape:
        case omni::physics::schema::ObjectType::eMeshShape:
        case omni::physics::schema::ObjectType::eCustomShape:
        case omni::physics::schema::ObjectType::eSpherePointsShape:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:shape");
            const ShapeDesc* inDesc = (const ShapeDesc*)objectDesc;
            SdfPathVector materials;
            PhysxShapeDesc* desc = parseCollisionDesc(mAttachedStage, mXfCache, mPhysXDescCache, path, *inDesc, mFilteredPairs, materials);
            if (desc)
            {
                mShapes.push_back({ path, desc, materials.empty() ? inDesc->materials : materials });
            }
        }
        break;
        // rest is buffered and created after
        case omni::physics::schema::ObjectType::eRigidBody:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:rigidBody");
            const RigidBodyDesc* inDesc = (const RigidBodyDesc*)objectDesc;
            PhysxRigidBodyDesc* desc = parseRigidBody(mAttachedStage, mXfCache, *inDesc, mFilteredPairs);
            if (desc)
            {
                BodyDescAndColliders& bdDesc = mBodyMap[path];
                bdDesc.desc = desc;
                bdDesc.collisions = inDesc->collisions;

                UsdPrim prim = mStage->GetPrimAtPath(path);
                if (prim.HasAPI<PhysxSchemaPhysxTendonAttachmentAPI>())
                {
                    parseTendonAttachments(mAttachedStage, prim, mXfCache, mTendonAttachmentMap, mSpatialTendons);
                }
            }
        }
        break;
        case omni::physics::schema::ObjectType::eArticulation:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:articulation");
            const ArticulationDesc* inDesc = (const ArticulationDesc*)objectDesc;
            if (mNumScenes <= 1 || checkArticulatonBodySimulationOwners(mAttachedStage, mStage, *inDesc))
            {
                parseArticulation(mStage, *inDesc, mFilteredPairs, mArticulationMap);
            }            
        }
        break;
        case omni::physics::schema::ObjectType::eJointSpherical:
        case omni::physics::schema::ObjectType::eJointD6:
        case omni::physics::schema::ObjectType::eJointDistance:
        case omni::physics::schema::ObjectType::eJointFixed:
        case omni::physics::schema::ObjectType::eJointPrismatic:
        case omni::physics::schema::ObjectType::eJointRevolute:
        case omni::physics::schema::ObjectType::eJointCustom:
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:joint");
            const JointDesc* inDesc = (const JointDesc*)objectDesc;
            if (mNumScenes == 1 || checkJointBodySimulationOwners(mAttachedStage, mStage, *inDesc))
            {
                PhysxJointDesc* desc = parseJoint(mStage, *inDesc, mXfCache);
                JointDescAndPath jd;
                jd.path = path;
                jd.desc = desc;
                jd.prim = inDesc->usdPrim;
                jd.articulationJoint = false;
                jd.index = uint32_t(mJointVector.size());
                mJointPathIndexMap[path] = mJointVector.size();
                mJointVector.push_back(jd);
            }
        }
        break;
        case omni::physics::schema::ObjectType::eVolumeDeformableBody:
        case omni::physics::schema::ObjectType::eSurfaceDeformableBody:
        {
			CARB_PROFILE_ZONE(0, "UsdPhysics:reportObjectDesc:deformableBody");
            const DeformableBodyDesc* inDesc = (const DeformableBodyDesc*)objectDesc;
            SdfPath material;
            PhysxDeformableBodyDesc* desc = parseDeformableBody(mAttachedStage, mXfCache, path, *inDesc, mFilteredPairs,
                material);
            if (desc)
            {
                mDeformableBodies.push_back({ path, desc, material });
                if (desc->type == eVolumeDeformableBody)
                {
                    mAttachedStage.getObjectDatabase()->addSchemaAPI(desc->simMeshPath, SchemaAPIFlag::eVolumeDeformableSimAPI);
                }
                else if (desc->type == eSurfaceDeformableBody)
                {
                    mAttachedStage.getObjectDatabase()->addSchemaAPI(desc->simMeshPath, SchemaAPIFlag::eSurfaceDeformableSimAPI);
                }
                mAttachedStage.getObjectDatabase()->addSchemaAPI(desc->simMeshPath, SchemaAPIFlag::eDeformablePoseAPI);
                for (SdfPath skinGeomPath : desc->skinGeomPaths)
                {
                    mAttachedStage.getObjectDatabase()->addSchemaAPI(skinGeomPath, SchemaAPIFlag::eDeformablePoseAPI);
                }
            }
        }
        break;
        case omni::physics::schema::ObjectType::eAttachmentVtxVtx:
        case omni::physics::schema::ObjectType::eAttachmentVtxTri:
        case omni::physics::schema::ObjectType::eAttachmentVtxTet:
        case omni::physics::schema::ObjectType::eAttachmentVtxXform:
        case omni::physics::schema::ObjectType::eAttachmentTetXform:
        {
            const AttachmentDesc* inDesc = (const AttachmentDesc*)objectDesc;
            PhysxDeformableAttachmentDesc* desc = parseDeformableAttachment(mStage, *inDesc);
            DeformableAttachmentDescAndPath attachmentDescAndPath;
            attachmentDescAndPath.path = path;
            attachmentDescAndPath.desc = desc;
            attachmentDescAndPath.prim = inDesc->usdPrim;
            mDeformableAttachmentVector.push_back(attachmentDescAndPath);
        }
        break;
        case omni::physics::schema::ObjectType::eElementCollisionFilter:
        {
            const ElementCollisionFilterDesc* inDesc = (const ElementCollisionFilterDesc*)objectDesc;
            PhysxDeformableCollisionFilterDesc* desc = parseDeformableCollisionFilter(mStage, *inDesc);
            DeformableCollisionFilterDescAndPath collisionFilterDescAndPath;
            collisionFilterDescAndPath.path = path;
            collisionFilterDescAndPath.desc = desc;
            collisionFilterDescAndPath.prim = inDesc->usdPrim;
            mDeformableCollisionFilterVector.push_back(collisionFilterDescAndPath);
        }
        break;
        default:
            break;
        }
    }

    void loadFromRange(PrimIteratorBase& primIterator, bool initialStageLoad)
    {
        mStage = mAttachedStage.getStage();
        mSceneFound = false;
        mNoValidScene = false;
        mParsingFlags = 0;
        mFilteredPairsPaths.clear();

        // run UsdPhysics parser first, eventually this should be the only parse loop
        primIterator.reset();
        IUsdPhysics* usdPhysics = carb::getCachedInterface<IUsdPhysics>();
/*
#if !CARB_AARCH64
        if (initialStageLoad)
        {
            startOmniPVDSampling();
        }
#endif
*/
        usdPhysics->registerPhysicsListener(this);
        {
            CARB_PROFILE_ZONE(0, "UsdPhysics:loadFromRange");
            usdPhysics->loadFromRange(mStage, mXfCache, primIterator);
        }
        usdPhysics->unregisterPhysicsListener(this);

        if (mNoValidScene)
            return;

        // We dont have a PhysX scene, PhysX should not simulate anything early exit
        if (mNoPhysXScene)
            return;

        if(OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) != nullptr)
        {
            // scristiano: do not parse particles, vehicles or internal objects when forcing single scene for inspector (for now)
            mParsingFlags = 0;
        }

        // require always a scene
        if (initialStageLoad && !mSceneFound)
        {
            const bool updateToUsd = OmniPhysX::getInstance().getCachedSettings().updateToUsd;
            if (updateToUsd)
            {
                PhysXUsdPhysicsInterface::reportLoadError(ErrorCode::eInfo,
                    "Physics USD: Physics scene not found. A temporary default PhysicsScene prim was added automatically!");

                ScopedLayerEdit scopedSessionLayerEdit(mStage, mStage->GetSessionLayer());
                SdfPath tempPhysicsScenePath = OmniPhysX::getInstance().getTempPhysicsScenePath();
                SdfJustCreatePrimInLayer(mStage->GetSessionLayer(), tempPhysicsScenePath);
                auto prim = createUsdPhysicsScene(tempPhysicsScenePath, mStage);
                primutils::setHideInStageWindow(prim, true);
                primutils::setNoDelete(prim, true);
                auto primRange = UsdPrimRange::AllPrims(prim);
                PrimIteratorRange primIterRange(primRange);
                usdPhysics->registerPhysicsListener(this);
                usdPhysics->loadFromRange(mStage, mXfCache, primIterRange);
                usdPhysics->unregisterPhysicsListener(this);
                OmniPhysX::getInstance().setHasTempPhysicsScene(true);
            }
            else
            {
                PhysxSceneDesc sceneDesc;
                setToDefault(mStage, sceneDesc);
                SdfPath tempPhysicsScenePath = OmniPhysX::getInstance().getTempPhysicsScenePath();
                const ObjectId sceneId = createObject(mAttachedStage, tempPhysicsScenePath, &sceneDesc, false); // deleteDesc == false 
                if (sceneId != kInvalidObjectId)
                {
                    mSceneFound = true;
                    mNoPhysXScene = false;
                    mNumScenes++;
                }
            }
        }

        // create deformable materials first
        {
            // DEPRECATED
            for (const std::pair<UsdPrim, FemSoftBodyMaterialDesc*>& pair : mFemSoftBodyMatrialsDescs)
            {
                createObject(mAttachedStage, pair.first.GetPath(), pair.second);
            }
            mFemSoftBodyMatrialsDescs.clear();

            // DEPRECATED
            for (const std::pair<UsdPrim, FemClothMaterialDesc*>& pair : mFemClothMatrialsDescs)
            {
                createObject(mAttachedStage, pair.first.GetPath(), pair.second);
            }
            mFemClothMatrialsDescs.clear();

            for (const std::pair<UsdPrim, PBDMaterialDesc*>& pair : mPDBMatrialsDescs)
            {
                createObject(mAttachedStage, pair.first.GetPath(), pair.second);
            }
            mPDBMatrialsDescs.clear();
        }

        // parse all primitives in given range
        primIterator.reset();
        if (mParsingFlags & ParsingFlag::eParseInternal)
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:loadPhysicsFromInternal");
            while (!primIterator.atEnd())
            {
                const UsdPrim& prim = *primIterator.getCurrent();
                if (!prim)
                {
                    primIterator.pruneChildren();
                    primIterator.next();
                    continue;
                }

                // skip the instancer parsing
                if (prim.IsA<UsdGeomPointInstancer>())
                {
                    primIterator.pruneChildren(); // Skip the subtree rooted at this prim
                }

                loadPhysicsFromPrimInternal(mAttachedStage, prim, mFilteredPairs);

                primIterator.next();
            }
        }

        // create materials first
        for (size_t i = 0; i < mMaterials.size(); i++)
        {
            createObject(mAttachedStage, mMaterials[i].first, mMaterials[i].second);
        }
        mMaterials.clear();

        // create shapes next
        for (const ShapeDescAndMaterials& shapeDesc : mShapes)
        {
            finalizeShape(mAttachedStage, shapeDesc.desc, shapeDesc.materials);
            PhysxRigidBodyDesc* bodyDesc = createShape(mAttachedStage, shapeDesc.path, mXfCache, shapeDesc.desc, nullptr);
            // standalone collision -> static body
            if (bodyDesc)
            {
                if (mBodyMap.find(shapeDesc.path) == mBodyMap.end())
                {
                    BodyDescAndColliders& bdCol = mBodyMap[shapeDesc.path];
                    bdCol.desc = bodyDesc;
                }
                else
                {
                    BodyDescAndColliders bdCol;
                    bdCol.desc = bodyDesc;
                    mAdditionalBodyVector.push_back(std::make_pair(shapeDesc.path, bdCol));
                }
            }
        }
        mShapes.clear();

        // finalize bodies
        for (BodyMap::reference ref : mBodyMap)
        {
            finalizeRigidBody(mAttachedStage, ref.second);
        }
        for (std::pair<SdfPath, BodyDescAndColliders>& ref : mAdditionalBodyVector)
        {
            finalizeRigidBody(mAttachedStage, ref.second);
        }

        // create articulation links
        {
            CARB_PROFILE_ZONE(0,"OmniPhysX:createArticulationLinks");
            SdfChangeBlock changeBlock; // add change block to get transforms sanitation changes grouped
            createArticulationLinks(mAttachedStage, mXfCache, mBodyMap, mJointVector, mArticulationMap, mJointPathIndexMap);
        }

        // create tendons (depends on prior articulation creation)
        createFixedTendons(mAttachedStage, mTendonAxisMap, mFixedTendons);
        createSpatialTendons(mAttachedStage, mTendonAttachmentMap, mSpatialTendons);

        // create rigid bodies
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createBodies");
            SdfChangeBlock changeBlock; // add change block to get transforms sanitation changes grouped
            createBodies(mAttachedStage, mBodyMap, mAdditionalBodyVector);
        }

        // deformable materials
        for (size_t i = 0; i < mDeformableMaterials.size(); i++)
        {
            createObject(mAttachedStage, mDeformableMaterials[i].first, mDeformableMaterials[i].second);
        }
        mDeformableMaterials.clear();

        // finalize and create deformable bodies
        for (const DeformableDescAndMaterials& deformableDesc : mDeformableBodies)
        {
            finalizeDeformableBody(mAttachedStage, deformableDesc.desc, deformableDesc.simMeshMaterial);
            createDeformableBody(mAttachedStage, deformableDesc.desc, deformableDesc.path);
        }

        // create forces
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createForces");
            for (const std::pair<UsdPrim, PhysxForceDesc*>& pair : mPhysxForceDescs)
            {
                finalizePhysxForce(mAttachedStage, pair.first, *pair.second, mXfCache);
                createObject(mAttachedStage, pair.first.GetPath(), pair.second);
            }
            mPhysxForceDescs.clear();
        }

        // create joints
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createJoints");
            createJoints(mAttachedStage, mJointVector, initialStageLoad);
        }

        // create mimic joints (has to be done after the joints are created)
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:createMimicJoints");
            createMimicJoints(mAttachedStage, mMimicJoints);
        }

        // parse all instancers and particle objects
        // A.B. TODO merge some traversals
        std::vector<ParticleSystemDesc*> particleSysDescs;
        std::vector<ParticleDesc*> particleDescs;

        primIterator.reset();
        if (mParsingFlags & ParsingFlag::eParseParticles)
        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:particles");
            std::vector<UsdPrim> jointInstancerPrims;

            while (!primIterator.atEnd())
            {
                const UsdPrim& prim = *primIterator.getCurrent();
                if (!prim)
                {
                    primIterator.pruneChildren();
                    primIterator.next();
                    continue;
                }

                if (prim.IsA<UsdGeomXform>())
                {
                    const TfTokenVector& appliedSchemas = prim.GetPrimTypeInfo().GetAppliedAPISchemas();                    

                    static auto isVoxelSchema = [](const TfToken& token) { return token == gInfiniteVoxelMapAPI; };
                    if (std::any_of(appliedSchemas.begin(), appliedSchemas.end(), isVoxelSchema))
                    {
                        // avoid parsing point instancers below
                        primIterator.pruneChildren();
                        InfiniteVoxelMapDesc desc(prim.GetPath());
                        createObject(mAttachedStage, prim.GetPath(), &desc, false);
                    }
                }
                else if (prim.IsA<UsdGeomMesh>() && prim.HasAPI<PhysxSchemaPhysxParticleSamplingAPI>())
                {
                    cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
                    ParticleSamplingDesc* samplingDesc = parseParticleSampling(mAttachedStage.getStageId(), prim.GetPath());
                    // check if it points to a valid particle prim
                    UsdPrim particlePrim = mStage->GetPrimAtPath(samplingDesc->particleSetPath);
                    if (!particlePrim.IsValid() || !particlePrim.HasAPI<PhysxSchemaPhysxParticleSetAPI>() || !(particlePrim.IsA<UsdGeomPoints>() || particlePrim.IsA<UsdGeomPointInstancer>()))
                    {
                        CARB_LOG_WARN("%s: particle sampler does not point to a valid particle prim, needs to be set before stage parsing starts.", prim.GetPath().GetText());
                    }
                    else if (!cookingDataAsync || !samplingDesc)
                    {
                        CARB_LOG_WARN("%s: particle sampling failed.", prim.GetPath().GetText());
                    }
                    else
                    {
                        // AD: OM-89182 - we need to disallow adding new samplers during simulation: it could have been just a path change, and we don't have the setup to clean things up.
                        // we never allowed processing changes in particleAuthoring and this code is just here in case there is no PhysXUI or we need to block at the sim start for async
                        // tasks to finish.
                        if (OmniPhysX::getInstance().getSimulationStepCount() == 0)
                        {
                            omni::physx::particles::createParticleSampler(prim.GetPath(), samplingDesc->particleSetPath);
                            cookingDataAsync->poissonSampleMesh(const_cast<UsdPrim&>(prim), *samplingDesc, false, false);
                        }
                        else
                        {
                            CARB_LOG_WARN("%s: it is not allowed to add new particle samplers once simulation has been started, ignoring.", prim.GetPath().GetText());
                        }
                    }
                    ICE_FREE(samplingDesc);
                }
                // DEPRECATED
                else if (prim.IsA<UsdGeomMesh>() && prim.HasAPI<PhysxSchemaPhysxParticleClothAPI>())
                {
                    // make sure particle cloth cooked data is written by the async cooker
	                cookingdataasync::CookingDataAsync* cookingDataAsync = omni::physx::OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
	                CARB_ASSERT(cookingDataAsync);
	                cookingDataAsync->cookParticleClothDeprecated(const_cast<UsdPrim&>(prim), false);

                    ParticleDesc* desc = nullptr;
                    desc = ParseParticleClothDeprecated(mAttachedStage, prim);
                    if (desc)
                    {
                        particleDescs.push_back(desc);
                    }
                }
                else if (prim.IsA<UsdGeomPointBased>() && prim.HasAPI<PhysxSchemaPhysxParticleSetAPI>())
                {
                    ParticleSetDesc* desc = ParseParticleSet(mAttachedStage, prim);
                    if (desc)
                    {
                        particleDescs.push_back(desc);
                    }
                }
                else if (prim.IsA<UsdGeomPointInstancer>())
                {
                    if (prim.HasAPI<PhysxSchemaPhysxParticleSetAPI>())
                    {
                        ParticleSetDesc* desc = ParseParticleSet(mAttachedStage, prim);
                        if (desc)
                        {
                            particleDescs.push_back(desc);
                        }
                    }
                    else if (!prim.IsInstance() && !prim.IsInstanceProxy())
                    {
                        parseRigidBodyInstancer(mAttachedStage, mXfCache, prim, mFilteredPairs);
                    }
                }
                else if (prim.IsA<PhysxSchemaPhysxPhysicsJointInstancer>())
                {
                    if (!prim.IsInstance() && !prim.IsInstanceProxy())
                    {
                        jointInstancerPrims.push_back(prim);                        
                    }
                }
                else if (prim.IsA<PhysxSchemaPhysxParticleSystem>())
                {
                    ParticleSystemDesc* desc = ParseParticleSystem(mAttachedStage, prim);
                    if (desc)
                        particleSysDescs.push_back(desc);
                }

                primIterator.next();
            }

            for (const UsdPrim& prim : jointInstancerPrims)
            {
                parseJointInstancer(mAttachedStage, mXfCache, prim);
            }

            createParticleSystemsAndObjects(mAttachedStage, particleSysDescs, particleDescs, mFilteredPairs);
        }

        // DEPRECATED, create attachments
        createPhysxAttachmentsDeprecated(mAttachedStage, mPhysxAttachmentVectorDeprecated);

        // create deformable attachments and filters
        createDeformableAttachments(mAttachedStage, mDeformableAttachmentVector);
        createDeformableCollisionFilters(mAttachedStage, mDeformableCollisionFilterVector);

        // create filtered collision groups.
        {
            PHYSICS_PROFILE("SetupCollisionGroups");
            setupCollisionGroups(mAttachedStage, mCollisionGroupsPrims);
        }

        // create collision blocks
        createFilteredPairs(mAttachedStage, mFilteredPairs, mFilteredPairsPaths);

        {
            CARB_PROFILE_ZONE(0, "OmniPhysX:updateMass");
            UsdLoad::getUsdLoad()->updateRigidBodyMass();
        }

        //
        // parse all vehicles
        //
        // note: done after the rigid body masses have been updated such that
        //       vehicle creation can work with the actual body mass.
        //
		if (mParsingFlags & ParsingFlag::eParseVehicles)
		{
	        // all the scenes are expected to be loaded now -> set the vehicle contexts
	        PhysXUsdPhysicsInterface& physicsInterface = *mAttachedStage.getPhysXPhysicsInterface();
	        for (VehicleContextDesc& vehicleContextDesc : mVehicleContextDescList)
	        {
	            physicsInterface.setVehicleContext(mAttachedStage, vehicleContextDesc);
	        }

	        // all the materials are expected to be loaded now -> create the tire friction tables (which reference materials).
	        // The vehicles need the tables to be created already.
	        for (TireFrictionTableDesc*& tireFrictionTableDesc : mTireFrictionTableDescList)
	        {
	            uint32_t materialPathCount = static_cast<uint32_t>(tireFrictionTableDesc->materialPaths.size());
	            tireFrictionTableDesc->materialIds.resize(materialPathCount);

	            for (size_t j = 0; j < materialPathCount; j++)
	            {
	                const SdfPath& materialPath = tireFrictionTableDesc->materialPaths[j];
	                ObjectId materialId = mAttachedStage.getObjectDatabase()->findEntry(materialPath, eMaterial);
	                tireFrictionTableDesc->materialIds[j] = materialId;
	            }

	            createObject(mAttachedStage, tireFrictionTableDesc->path, tireFrictionTableDesc, true);
	        }
	
	        VehicleComponentTracker vehicleComponentTracker;
	        primIterator.reset();
	        while (!primIterator.atEnd())
	        {
	            const UsdPrim& prim = *primIterator.getCurrent();
	            if (!prim)
	            {
	                primIterator.pruneChildren();
	                primIterator.next();
	                continue;
	            }

	            // skip the instancer parsing
	            if (prim.IsA<UsdGeomPointInstancer>())
	            {
	                primIterator.pruneChildren(); // Skip the subtree rooted at this prim
	            }

	            // note: PhysxVehicleContextAPI needs to be set before creating vehicles. However, checking here is not that easy since
                //       the necessary data is not readily available. Thus, the check is done as part of vehicle creation.

	            loadVehicle(mAttachedStage, prim, vehicleComponentTracker, mXfCache);

	            primIterator.next();
	        }
    	}

        for (PathPhysXDescMap::reference ref : mPhysXDescCache)
        {
            PhysxObjectDesc* desc = (PhysxObjectDesc*)ref.second;
            ICE_FREE(desc);
        }
        mPhysXDescCache.clear();
    }

private:
    bool mSceneFound;
    bool mNoValidScene;
    bool mNoPhysXScene;
    bool mCollectionsPopulated;
    uint32_t mNumScenes;

    uint32_t mParsingFlags;

    AttachedStage&  mAttachedStage;
    UsdStageWeakPtr mStage;
    UsdGeomXformCache mXfCache;
    PathPhysXDescMap  mPhysXDescCache;

    // maps holding the descs
    MaterialsVector mMaterials;
    DeformableMaterialsVector mDeformableMaterials;
    ShapeDescsVector mShapes;
    ArticulationMap mArticulationMap;
    BodyMap mBodyMap;
    BodyVector mAdditionalBodyVector;
    JointVector mJointVector;
    JointPathIndexMap mJointPathIndexMap;
    DeformableBodyDescsVector mDeformableBodies;

    SdfPathVector mFilteredPairsPaths;
    CollisionPairVector mFilteredPairs;

    // forces
    std::vector<std::pair<UsdPrim, PhysxForceDesc*>> mPhysxForceDescs;

    // DEPRECATED
    std::vector<std::pair<UsdPrim, FemSoftBodyMaterialDesc*>> mFemSoftBodyMatrialsDescs;
    std::vector<std::pair<UsdPrim, FemClothMaterialDesc*>> mFemClothMatrialsDescs;

    // particle materials
    std::vector<std::pair<UsdPrim, PBDMaterialDesc*>> mPDBMatrialsDescs;

    std::vector<UsdPrim> mCollisionGroupsPrims;

    std::vector<VehicleContextDesc> mVehicleContextDescList;
    std::vector<TireFrictionTableDesc*> mTireFrictionTableDescList;

    // tendon data structures
    SpatialTendonVector mSpatialTendons;
    TendonAttachmentMap mTendonAttachmentMap;
    FixedTendonVector mFixedTendons;
    TendonAxisMap mTendonAxisMap;

    MimicJointVector mMimicJoints;

    // DEPRECATED, PhysX attachments
    PhysxAttachmentVectorDeprecated mPhysxAttachmentVectorDeprecated;
    
    // deformable attachments and collision filters
    DeformableAttachmentVector mDeformableAttachmentVector;
    DeformableCollisionFilterVector mDeformableCollisionFilterVector;
};

void loadFromStage(AttachedStage& attachedStage, const PathSet* excludePaths)
{
    PHYSICS_PROFILE("Physics Load Stage");
    CARB_PROFILE_ZONE(0, "Physics Load Stage");
    UsdPrimRange range = attachedStage.getStage()->Traverse(UsdTraverseInstanceProxies());
    if (excludePaths)
    {
        // Parse only stuff allowed, skip stuff
        ReplicatorPrimIteratorRange primIteratorRange(range, *excludePaths);
        PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
        usdPhysicsListener.loadFromRange(primIteratorRange, true);
    }
    else
    {
        omni::physics::schema::PrimIteratorRange primIteratorRange(range);
        PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
        usdPhysicsListener.loadFromRange(primIteratorRange, true);
    }
}

void loadPhysicsFromPrimitive(AttachedStage& attachedStage, omni::physics::schema::PrimIteratorMapRange& primIteratorUpdateMap)
{
    PHYSICS_PROFILE("Physics Load Prims");
    CARB_PROFILE_ZONE(0, "Physics Load Prims");
    primIteratorUpdateMap.reset();

    if(primIteratorUpdateMap.atEnd() == false)
    {
        PhysxUsdPhysicsListener usdPhysicsListener(attachedStage);
        usdPhysicsListener.loadFromRange(primIteratorUpdateMap, false);
    }
}

} // namespace usdparser
} // namespace physx
} // namespace omni
