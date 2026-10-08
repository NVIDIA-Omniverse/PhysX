// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-PARSE-ART-001
 * @covers AC-4
 *
 * @implements REQ-PARSE-UNIFY-001
 * @covers AC-3
 */

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <OmniPhysX.h>
#include <omni/physx/IPhysxSettings.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "Joint.h"
#include "PhysicsBody.h" // getRigidBodySimulationOwner

#include <omni/physics/parse/KnownTokens.h>
#include <omni/physics/parse/ParseApi.h>
#include <omni/physics/parse/ParseContext.h>

namespace omni
{
namespace physx
{
namespace usdparser
{
// Was SdfPath->SdfPath (std::map, since SdfPath orders); ObjectKey has no
// operator<, so this is now ObjectKey->ObjectKey via unordered_map, same
// treatment as the LoadTools.h aliases it parallels (e.g. JointPathIndexMap).
using PathPathMap = std::unordered_map<omni::physics::parse::ObjectKey,
                                       omni::physics::parse::ObjectKey,
                                       omni::physics::parse::ObjectKey::Hash>;

void checkArticulationSceneID(AttachedStage& attachedStage,
                              omni::physics::parse::ObjectKey bodyKey,
                              PhysxArticulationDesc& articulationDesc,
                              bool& foundSceneID)
{
    // simulationOwner of `bodyKey` via the source (same relationship rbo/collision
    // declare); invalid when unauthored. getRigidBodySimulationOwner is ObjectKey-native
    // (ADR-0019).
    const omni::physics::parse::ObjectKey ownerKey = getRigidBodySimulationOwner(attachedStage, bodyKey);
    if (ownerKey.valid())
    {
        const ObjectId id = attachedStage.getObjectDatabase()->findEntry(ownerKey, eScene);

        if (id != kInvalidObjectId)
        {
            if (articulationDesc.sceneId != kInvalidObjectId && articulationDesc.sceneId != id)
            {
                CARB_LOG_ERROR(
                    "parseArticulation: Inconsistent scene owner \"%s\" for body link \"%s\".\n"
                    "All links in articulation should share the same simulation owner.",
                    attachedStage.textFor(ownerKey), attachedStage.textFor(bodyKey));
            }
            else
            {
                articulationDesc.sceneId = id;
                foundSceneID = true;
            }
        }
        else
        {
            // If we're not forcing parse of a single scene, this is an actual error
            if (OmniPhysX::getInstance().getISettings()->getStringBuffer(kSettingForceParseOnlySingleScene) == nullptr)
            {
                CARB_LOG_ERROR("parseArticulation: Failed to find physics simulation owner \"%s\".", attachedStage.textFor(ownerKey));
            }
        }
    }
}
void createArticulationObject(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, PhysxObjectDesc* desc, ObjectId& articulationId)
{
    PhysxArticulationDesc& articulationDesc = static_cast<PhysxArticulationDesc&>(*desc);
    // Infer articulation scene id from articulatedBodies
    bool foundSceneID = false;
    for(auto& body : articulationDesc.articulatedBodies)
    {
        checkArticulationSceneID(attachedStage, body, articulationDesc, foundSceneID);
    }
    if (!foundSceneID)
    {
        // scristiano: articulatedBodies can contain invalid ObjectKeys when a rigid body is converted to
        // an articulation link (traverseHierarchy pushes an invalid key for static children), so the scene
        // ID must be resolved from the joints instead - e.g. an ArticulationRootAPI xform with a
        // static collider and a rigid body joined by a revolute joint.
        const omni::physics::parse::IPhysicsSource* src = attachedStage.getSource();
        if (src)
        {
            const omni::physics::parse::KnownTokens& tok = attachedStage.getKnownTokens();
            for (auto& jointKey : articulationDesc.articulatedJoints)
            {
                // joint body0/body1 targets via the source (jointKey is already an ObjectKey).
                for (const omni::physics::parse::TokenId bodyRel : { tok.physicsBody0, tok.physicsBody1 })
                {
                    std::vector<omni::physics::parse::ObjectKey> bodyTargets;
                    src->getRelationshipTargets(jointKey, bodyRel, bodyTargets);
                    if (!bodyTargets.empty())
                        checkArticulationSceneID(attachedStage, bodyTargets.front(), articulationDesc, foundSceneID);
                }
            }
        }
    }
    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, primKey, *desc);
    if (id != kInvalidObjectId)
    {
        // findOrCreateEntry(ObjectKey, pathText, ...), not the bare-ObjectKey overload: this
        // also feeds PrimHierarchyStorage, which breaks cascade-delete when an ancestor prim
        // is removed if skipped (mirrors the LoadStage.cpp createBodies fix).
        attachedStage.getObjectDatabase()->findOrCreateEntry(primKey, attachedStage.textFor(primKey), eArticulation, id);
        if (articulationDesc.fixBase && articulationDesc.fixBaseKey.valid())
        {
            // A.B. we created one more for fixed base ePTArticulationFixedBase
            attachedStage.getObjectDatabase()->findOrCreateEntry(
                articulationDesc.fixBaseKey, attachedStage.textFor(articulationDesc.fixBaseKey), eArticulationRootJoint, id + 1);
        }
    }

    articulationId = id;
    ICE_FREE(desc);
}

void createArticulationLink(AttachedStage& attachedStage,
                            omni::physics::parse::ObjectKey linkKey,
                            PhysxArticulationLinkDesc& desc,
                            const ObjectId articulationId,
                            const ObjectId parentId,
                            ObjectId& outId)
{
    desc.articulation = articulationId;
    desc.parent = parentId;

    ObjectDb* objDb = attachedStage.getObjectDatabase();

    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, linkKey, desc);
    outId = id;

    if (id != kInvalidObjectId)
    {
        // findOrCreateEntry(ObjectKey, pathText, ...); see createArticulationObject above.
        objDb->findOrCreateEntry(linkKey, attachedStage.textFor(linkKey), eArticulationLink, id);
        attachedStage.bufferRequestRigidBodyMassUpdate(linkKey);
    }

    desc.shapes.clear();
}

// Retype a parsed dynamic body into an articulation link, in place. Neither the
// stage nor the body path were ever consulted here, so this needs no source access.
void convertBodyToArticulationLink(DynamicPhysxRigidBodyDesc* bodyDesc,
                                   PhysxArticulationLinkDesc& tempArticulationLinkDesc)
{
    ((DynamicPhysxRigidBodyDesc&)tempArticulationLinkDesc) = *bodyDesc;
    tempArticulationLinkDesc.type = eArticulationLink;
    bodyDesc->type = eArticulationLink;
}

void createLinkHierarchy(AttachedStage& attachedStage,
                         omni::physics::parse::ObjectKey parentKey,
                         const JointVector& jointVector,
                         BodyMap& bodyMap,
                         const ObjectId articulationId,
                         const ObjectId parentId,
                         KeySet& articulatedJoints,
                         PathPathMap& linkPaths,
    PhysxArticulationLinkDesc& tempLink)
{
    KeySet childKeys;
    for (size_t jIndex = 0; jIndex < jointVector.size(); jIndex++)
    {
        const JointDescAndPath& jointDescAndPath = jointVector[jIndex];
        const omni::physics::parse::ObjectKey jointLinkKey = jointDescAndPath.path;

        if (articulatedJoints.find(jointLinkKey) != articulatedJoints.end())
            continue;

        const PhysxJointDesc* jointDesc = jointDescAndPath.desc;
        const ArticulationJointType jointType = jointDesc->excludedFromArticulation ? eMaximalJoint : eStandardJoint;

        // maximal joints cannot be part of articulations
        if (jointType == eMaximalJoint)
        {
            continue;
        }

        const char* jointLinkText = attachedStage.textFor(jointLinkKey);

        if (jointDesc->type == eJointDistance)
        {
            CARB_LOG_ERROR("Articulation cannot contain a distance joint! (Please exclude it from articulation) (%s)", jointLinkText);
            continue;
        }
        else if (jointDesc->type == eJointGear)
        {
            CARB_LOG_ERROR("Articulation cannot contain a gear joint! (Please exclude it from articulation) (%s)", jointLinkText);
            continue;
        }
        else if (jointDesc->type == eJointRackAndPinion)
        {
            CARB_LOG_ERROR("Articulation cannot contain a rack and pinion joint! (Please exclude it from articulation) (%s)", jointLinkText);
            continue;
        }
        if(jointDesc->enableCollision)
        {
            CARB_LOG_WARN("Per-joint collision setting is ignored for articulations. (%s)", jointLinkText);
        }


        JointLimits::const_iterator d6limits;
        size_t nLimitsNum = 1;
        if(jointDesc->type == eJointD6)
        {
            const D6PhysxJointDesc* d6joint = static_cast<const D6PhysxJointDesc*>(jointDesc);
            nLimitsNum = d6joint->jointLimits.size();
            d6limits = d6joint->jointLimits.begin();
        }

        for(size_t n = 0; n < nLimitsNum; n++)
        {
            const PhysxJointLimit* limit = nullptr;
            switch(jointDesc->type)
            {
                case eJointRevolute:
                {
                    limit = &static_cast<const RevolutePhysxJointDesc*>(jointDesc)->limit;
                    break;
                }
                case eJointPrismatic:
                {
                    limit = &static_cast<const PrismaticPhysxJointDesc*>(jointDesc)->limit;
                    break;
                }
                case eJointSpherical:
                {
                    limit = &static_cast<const SphericalPhysxJointDesc*>(jointDesc)->limit;
                    break;
                }
                case eJointD6:
                {
                    limit = static_cast<const PhysxJointLimit*>(&(*d6limits).second);
                    d6limits++;
                    break;
                }
            }

            if(limit != nullptr)
            {
                if(limit->stiffness > 0.0f)
                {
                    CARB_LOG_INFO("Stiffness attribute is unsupported for articulation joints and will be ignored (%s).", jointLinkText);
                }
                if(limit->damping > 0.0f)
                {
                    CARB_LOG_INFO("Damping attribute is unsupported for articulation joints and will be ignored (%s).", jointLinkText);
                }
                if(limit->restitution > 0.0f)
                {
                    CARB_LOG_INFO("Restitution attribute is unsupported for articulation joints and will be ignored (%s).", jointLinkText);
                }
                if(limit->bounceThreshold > 0.0f)
                {
                    CARB_LOG_INFO("Bounce threshold attribute is unsupported for articulation joints and will be ignored (%s).", jointLinkText);
                }
            }
        }
        const omni::physics::parse::ObjectKey body0Key = jointDesc->body0;
        const omni::physics::parse::ObjectKey body1Key = jointDesc->body1;
        omni::physics::parse::ObjectKey newBodyKey;
        BodyMap::iterator bodyIterator;
        bool body0Found = false;
        bool body1Found = false;
        if (parentKey == body0Key && (body1Key.valid() || jointType == eMaximalJoint))
        {
            bodyIterator = bodyMap.find(body1Key);
            if (bodyIterator != bodyMap.end() && bodyIterator->second.desc->type != eStaticBody)
            {
                body1Found = true;
                newBodyKey = body1Key;
            }
        }
        else if (parentKey == body1Key && (body0Key.valid() || jointType == eMaximalJoint))
        {
            bodyIterator = bodyMap.find(body0Key);
            if (bodyIterator != bodyMap.end() && bodyIterator->second.desc->type != eStaticBody)
            {
                body0Found = true;
                newBodyKey = body0Key;
            }
        }
        if (body0Found || body1Found)
        {
            // If newBody is not already a child of the parent, create the articulation joint, else skip it
            if (childKeys.find(newBodyKey) == childKeys.end())
            {
                if (jointType == eStandardJoint)
                    childKeys.insert(newBodyKey);
                bool createMaximalJoint = false;
                if (newBodyKey.valid())
                {
                    // check if body was not already created, if it was we have a circular dependency
                    if (linkPaths.find(newBodyKey) == linkPaths.end())
                    {
                        linkPaths[newBodyKey] = jointLinkKey;

                        ObjectId outId = kInvalidObjectId;
                        if (bodyIterator->second.desc->type == eDynamicBody)
                        {
                            convertBodyToArticulationLink((DynamicPhysxRigidBodyDesc*)bodyIterator->second.desc, tempLink);

                            tempLink.articulationJointType = jointType;
                            tempLink.articulationJoint = jointDesc;
                            articulatedJoints.insert(jointLinkKey);
                            createArticulationLink(attachedStage,
                                newBodyKey, tempLink, articulationId, parentId, outId);

                            // do the children
                            if (jointType == eStandardJoint)
                                createLinkHierarchy(attachedStage, newBodyKey, jointVector,
                                    bodyMap, articulationId, outId, articulatedJoints, linkPaths, tempLink);
                        }
                    }
                    else
                    {
                        // detected circular dependency, skip this joint completely
                        createMaximalJoint = false;
                        std::string jointNames;
                        for (PathPathMap::const_reference linkRef : linkPaths)
                        {
                            jointNames += attachedStage.textFor(linkRef.second);
                            jointNames += "\n";
                        }
                        REPORT_PHYSICS_ERROR(
                            "RigidBody (%s) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints:\n%s from articulation, the joint will be now excluded from the articulation.",
                            attachedStage.textFor(newBodyKey), jointNames.c_str());
                        articulatedJoints.insert(jointLinkKey);
                    }
                }
                else
                {
                    createMaximalJoint = true;
                }

                if (createMaximalJoint)
                {
                    // for a maximal joint to world, create directly a joint
                    ObjectId body0 = body0Key.valid() ? attachedStage.getObjectDatabase()->findEntry(body0Key, eArticulationLink) : kInvalidObjectId;
                    ObjectId body1 = body1Key.valid() ? attachedStage.getObjectDatabase()->findEntry(body1Key, eArticulationLink) : kInvalidObjectId;
                    articulatedJoints.insert(jointLinkKey);

                    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createJoint(attachedStage, jointLinkKey, *jointDesc, body0, body1);

                    if (id != kInvalidObjectId)
                        // findOrCreateEntry(ObjectKey, pathText, ...); see createArticulationObject above.
                        attachedStage.getObjectDatabase()->findOrCreateEntry(jointLinkKey, jointLinkText, eJoint, id);
                }
            }
            else
            {
                // Diagnostic prints the full path text now (textFor), not just the prim's leaf
                // name (SdfPath::GetName()) as before -- no leaf-name accessor exists for a
                // bare ObjectKey; the message content is otherwise unchanged.
                CARB_LOG_WARN("PhysX : articulation joint (%s) with body0 (%s) and body1 (%s), was skipped, since body1 is already defined as a child of body0",
                    jointLinkText, attachedStage.textFor(body0Key), attachedStage.textFor(body1Key));
            }
        }
    }
}

void createArticulationLinks(AttachedStage& attachedStage,
                             BodyMap& bodyMap,
                             JointVector& jointVectorIn,
                             const ArticulationMap& articulationMap,
                             const JointPathIndexMap& jointPathIndexMap)
{
    KeySet articulatedJoints; // vector of joints that belong to articulation, those will be removed from the jointVector
    PhysxArticulationLinkDesc* tempArticulationLinkDesc = nullptr;
    if (!articulationMap.empty())
    {
        tempArticulationLinkDesc = ICE_PLACEMENT_NEW(PhysxArticulationLinkDesc)();
    }


    // go over the articulation starting points and construct the articulation hierarchy
    for (ArticulationMap::const_reference& articulationRef : articulationMap)
    {
        for (size_t i = 0; i < articulationRef.second.size(); i++)
        {
            PhysxArticulationDesc* articulationDesc = articulationRef.second[i];
            const PhysxJointDesc* rootJointDesc = nullptr;
            JointVector jointVector;
            jointVector.reserve(articulationDesc->articulatedJoints.size());
            const JointPathIndexMap::const_iterator jointPathEnd = jointPathIndexMap.end();

            for (const omni::physics::parse::ObjectKey& jpKey : articulationDesc->articulatedJoints)
            {
                JointPathIndexMap::const_iterator fit = jointPathIndexMap.find(jpKey);
                if (fit != jointPathEnd)
                {
                    const size_t index = fit->second;
                    jointVector.push_back(jointVectorIn[index]);
                    if (articulationDesc->rootPrim == jointVectorIn[index].path)
                    {
                        rootJointDesc = jointVectorIn[index].desc;
                    }
                }
            }

            // This is required to maintain parsing order and determinism
            std::sort(jointVector.begin(), jointVector.end());

            ObjectId articulationId = kInvalidObjectId;
            ObjectId parentId = kInvalidObjectId;
            ObjectId outId = kInvalidObjectId;
            BodyMap::iterator parentBodyIt = bodyMap.find(articulationDesc->rootPrim);

            // check if we start from an articulated body, parse its parameters
            // if we start from a body the articulation is not fixed
            omni::physics::parse::ObjectKey jointLinkKey;
            if (parentBodyIt != bodyMap.end())
            {
                if (parentBodyIt->second.desc->type == eDynamicBody)
                {
                    convertBodyToArticulationLink((DynamicPhysxRigidBodyDesc*)parentBodyIt->second.desc, *tempArticulationLinkDesc);
                }
                else
                {
                    parentBodyIt = bodyMap.end();
                }
                articulationDesc->fixBase = false;
            }
            else if(rootJointDesc)
            {
                // we start from a articulation joint marked as a root
                // as we traverse we convert the bodies into articulation links
                jointLinkKey = articulationDesc->rootPrim;
                const omni::physics::parse::ObjectKey rootJointBody0 = rootJointDesc->body0;
                const omni::physics::parse::ObjectKey rootJointBody1 = rootJointDesc->body1;
                const char* rootJointPrimText = attachedStage.textFor(rootJointDesc->jointPrimKey);
                omni::physics::parse::ObjectKey rootBodyKey;
                if (!rootJointBody0.valid() || !rootJointBody1.valid())
                {
                    rootBodyKey = rootJointBody0.valid() ? rootJointBody0 : rootJointBody1;
                    parentBodyIt = bodyMap.find(rootBodyKey);
                }
                else
                {
                    bool rootBodySet = false;
                    parentBodyIt = bodyMap.find(rootJointBody0);
                    if (parentBodyIt != bodyMap.end())
                    {
                        if (parentBodyIt->second.desc->type == eStaticBody)
                        {
                            articulationDesc->staticRootBodyPrim = parentBodyIt->first;
                            rootBodyKey = rootJointBody1;
                            parentBodyIt = bodyMap.find(rootBodyKey);
                            rootBodySet = true;
                        }
                        else if (parentBodyIt->second.desc->type == eDynamicBody)
                        {
                            const DynamicPhysxRigidBodyDesc* dynDesc = reinterpret_cast<const DynamicPhysxRigidBodyDesc*>(parentBodyIt->second.desc);
                            if (dynDesc->kinematicBody)
                            {
                                REPORT_PHYSICS_ERROR("Articulations with kinematic bodies are not supported, please exclude joint (%s) from articulation.", rootJointPrimText);
                            }
                        }
                    }
                    if (!rootBodySet)
                    {
                        parentBodyIt = bodyMap.find(rootJointBody1);
                        if (parentBodyIt != bodyMap.end())
                        {
                            if (parentBodyIt->second.desc->type == eStaticBody)
                            {
                                articulationDesc->staticRootBodyPrim = parentBodyIt->first;
                                rootBodyKey = rootJointBody0;
                                parentBodyIt = bodyMap.find(rootBodyKey);
                            }
                            else if (parentBodyIt->second.desc->type == eDynamicBody)
                            {
                                const DynamicPhysxRigidBodyDesc* dynDesc = reinterpret_cast<const DynamicPhysxRigidBodyDesc*>(parentBodyIt->second.desc);
                                if (dynDesc->kinematicBody)
                                {
                                    REPORT_PHYSICS_ERROR(
                                        "Articulations with kinematic bodies are not supported, please exclude joint (%s) from articulation.",
                                        rootJointPrimText);
                                }
                            }
                        }
                    }
                }
                if (!rootBodyKey.valid() || parentBodyIt == bodyMap.end())
                    continue;
                if (parentBodyIt->second.desc && parentBodyIt->second.desc->type == eDynamicBody)
                {
                    convertBodyToArticulationLink((DynamicPhysxRigidBodyDesc*)parentBodyIt->second.desc, *tempArticulationLinkDesc);
                }
                else
                {
                    parentBodyIt = bodyMap.end();
                }
                articulationDesc->fixBase = true;
                articulationDesc->fixBaseKey = jointLinkKey;
                articulatedJoints.insert(jointLinkKey);
            }
            else
            {
                continue;
            }

            if (parentBodyIt == bodyMap.end())
                continue;

            // create the articulation itself, the links with joints will be added by further parsing the hierarchy
            createArticulationObject(attachedStage, articulationRef.first, articulationDesc, articulationId);
            if (articulationId != kInvalidObjectId)
            {
                // create top level articulation root link
                tempArticulationLinkDesc->articulationJointType = eStandardJoint;
                createArticulationLink(attachedStage,
                    parentBodyIt->first, *tempArticulationLinkDesc, articulationId, parentId, outId);

                // do the children and construct the hierarchy
                PathPathMap linkPaths;
                linkPaths[parentBodyIt->first] = parentBodyIt->first;
                createLinkHierarchy(attachedStage, parentBodyIt->first, jointVector, bodyMap, articulationId, outId, articulatedJoints, linkPaths, *tempArticulationLinkDesc);
            }
        }
    }

    if (!articulatedJoints.empty())
    {
        for (size_t i = jointVectorIn.size(); i--;)
        {
            KeySet::const_iterator it = articulatedJoints.find(jointVectorIn[i].path);
            if (it != articulatedJoints.end())
            {
                jointVectorIn[i].articulationJoint = true;
            }
        }
    }

    ICE_FREE(tempArticulationLinkDesc);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
