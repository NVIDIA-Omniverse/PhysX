// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <common/foundation/Allocator.h>
#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <OmniPhysX.h>
#include <omni/physx/IPhysxSettings.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "Joint.h"
#include "Mass.h"
#include "PhysicsBody.h"
#include "AttributeHelpers.h"

using namespace pxr;

namespace omni
{
namespace physx
{
namespace usdparser
{
using PathPathMap = std::map<pxr::SdfPath, pxr::SdfPath>;

bool checkArticulatonBodySimulationOwners(AttachedStage& attachedStage, pxr::UsdStageWeakPtr stage, const omni::physics::schema::ArticulationDesc& inDesc)
{
    bool firstBody = true;
    SdfPath owner = SdfPath();
    for (const SdfPath& path : inDesc.articulatedBodies)
    {
        if (path != SdfPath())
        {
            const SdfPath bodyOwner = getRigidBodySimulationOwner(stage, path);
            if (firstBody)
            {
                owner = bodyOwner;
                firstBody = false;
            }
            else
            {
                if (owner != bodyOwner)
                {
                    CARB_LOG_ERROR("Articulation contains bodies with different simulation owners. Articulation: %s", inDesc.usdPrim.GetPrimPath().GetText());
                    return false;
                }
            }
        }
    }

    if (owner != SdfPath())
    {
            // check if they belong to a scene we do simulate
            const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(owner, eScene);
            if (entry == kInvalidObjectId)
            {                
                return false;
            }
    }
    return true;
}

void setToDefault(UsdStageWeakPtr stage, PhysxArticulationDesc& desc)
{
    double metersPerUnit = pxr::UsdGeomGetStageMetersPerUnit(stage);
    float tolerancesSpeed = float(10.0f / metersPerUnit);

    desc.solverPositionIterationCount = 32;
    desc.solverVelocityIterationCount = 1;
    desc.sleepThreshold = 5e-5f * tolerancesSpeed * tolerancesSpeed;
    desc.stabilizationThreshold = 0.01f * tolerancesSpeed * tolerancesSpeed;
    desc.selfCollision = true;
    desc.staticRootBodyPrim = SdfPath();
    desc.reportResiduals = false;
}

void parseArticulation(const pxr::UsdStageWeakPtr stage, const omni::physics::schema::ArticulationDesc& inDesc,
    CollisionPairVector& filteredPairs, ArticulationMap& articulationMap)
{
    for (size_t i = 0; i < inDesc.rootPrims.size(); i++)
    {
        // physx part
        const PhysxSchemaPhysxArticulationAPI physxArticulationAPI =
            PhysxSchemaPhysxArticulationAPI::Get(stage, inDesc.usdPrim.GetPrimPath());

        if (physxArticulationAPI)
        {
            bool enabled = false;
            physxArticulationAPI.GetArticulationEnabledAttr().Get(&enabled);
            if (!enabled)
            {
                continue;
            }
        }

        PhysxArticulationDesc* desc = ICE_PLACEMENT_NEW(PhysxArticulationDesc)();

        desc->rootPrim = inDesc.rootPrims[i];
        const UsdPrim rootUsdPrim = stage->GetPrimAtPath(desc->rootPrim);
        desc->fixBase = rootUsdPrim.IsA<UsdPhysicsJoint>() ? true : false;
        desc->fixBasePath = rootUsdPrim.GetPrimPath();
        desc->articulatedJoints = inDesc.articulatedJoints;
        desc->articulatedBodies = inDesc.articulatedBodies;

        setToDefault(stage, *desc);

        if (physxArticulationAPI)
        {
            getAttribute(desc->sleepThreshold, physxArticulationAPI.GetSleepThresholdAttr(), 0.0f, FLT_MAX, updateArticulationSleepThreshold);
            getAttribute(desc->stabilizationThreshold, physxArticulationAPI.GetStabilizationThresholdAttr(),
                0.0f, FLT_MAX, updateArticulationStabilizationThreshold);

            getAttribute(desc->solverPositionIterationCount,
                physxArticulationAPI.GetSolverPositionIterationCountAttr(), 1, 255, updateArticulationSolverPositionIterationCount);
            getAttribute(desc->solverVelocityIterationCount,
                physxArticulationAPI.GetSolverVelocityIterationCountAttr(), 0, 255, updateArticulationSolverVelocityIterationCount);

            getBoolAttribute(desc->selfCollision, physxArticulationAPI.GetEnabledSelfCollisionsAttr(), nullptr);
        }

        const PhysxSchemaPhysxResidualReportingAPI physxResidualReportingAPI = PhysxSchemaPhysxResidualReportingAPI::Get(stage, inDesc.usdPrim.GetPrimPath());
        if (physxResidualReportingAPI)
        {
            desc->reportResiduals = true;
        }

        articulationMap[inDesc.usdPrim.GetPrimPath()].push_back(desc);
    }

    for (size_t i = 0; i < inDesc.filteredCollisions.size(); i++)
    {
        filteredPairs.push_back(std::make_pair(inDesc.usdPrim.GetPrimPath(), inDesc.filteredCollisions[i]));
    }
}

void checkArticulationSceneID(AttachedStage& attachedStage,
                              const SdfPath& body,
                              PhysxArticulationDesc& articulationDesc,
                              bool& foundSceneID)
{
    const UsdPhysicsRigidBodyAPI rigidBodyApi = UsdPhysicsRigidBodyAPI::Get(attachedStage.getStage(), body);
    if (UsdRelationship rel = rigidBodyApi.GetSimulationOwnerRel())
    {
        SdfPathVector paths;
        rel.GetTargets(&paths);

        if (!paths.empty())
        {
            const SdfPath& path = paths.front();
            const ObjectId id = attachedStage.getObjectDatabase()->findEntry(path, eScene);

            if (id != kInvalidObjectId)
            {
                if (articulationDesc.sceneId != kInvalidObjectId && articulationDesc.sceneId != id)
                {
                    CARB_LOG_ERROR(
                        "parseArticulation: Inconsistent scene owner \"%s\" for body link \"%s\".\n"
                        "All links in articulation should share the same simulation owner.",
                        path.GetText(), body.GetText());
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
                    CARB_LOG_ERROR("parseArticulation: Failed to find physics simulation owner \"%s\".", path.GetText());
                }
            }
        }
    }
}
void createArticulationObject(AttachedStage& attachedStage, const SdfPath& primPath, PhysxObjectDesc* desc, ObjectId& articulationId)
{
    PhysxArticulationDesc& articulationDesc = static_cast<PhysxArticulationDesc&>(*desc);
    // Infer articulation scene id from articulatedBodies
    OmniPhysX& omniPhysX = OmniPhysX::getInstance();
    UsdStageWeakPtr stage = omniPhysX.getStage();
    bool foundSceneID = false;
    for(auto& body : articulationDesc.articulatedBodies)
    {
        checkArticulationSceneID(attachedStage, body, articulationDesc, foundSceneID);
    }
    if (!foundSceneID)
    {
        // scristiano: There are some edge cases in the parsing where rigid bodies are converted to
        // articulation links (see convertBodyToArticulationLink) and in that case the articulatedBodies
        // can be empty SdfPaths (as pushed by traverseHierarchy with link.childs.push_back(SdfPath()) )
        // so to figure out the right scene ID we must check the actual joints
        // One way to trigger this case is to have an xform with ArticulationRootAPI applied containing
        // a child static collider and a child rigid body connected by a revolute joint
        for (auto& jointPath : articulationDesc.articulatedJoints)
        {
            auto joint = UsdPhysicsJoint::Get(stage, jointPath);
            UsdRelationship rel = joint.GetBody0Rel();
            if (rel)
            {
                SdfPathVector paths;
                rel.GetTargets(&paths);
                if (!paths.empty())
                {
                    const SdfPath& path = paths.front();
                    checkArticulationSceneID(attachedStage, path, articulationDesc, foundSceneID);
                }
            }
            rel = joint.GetBody1Rel();
            if (rel)
            {
                SdfPathVector paths;
                rel.GetTargets(&paths);
                if (!paths.empty())
                {
                    const SdfPath& path = paths.front();
                    checkArticulationSceneID(attachedStage, path, articulationDesc, foundSceneID);
                }
            }
        }
    }
    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, primPath, *desc);
    if (id != kInvalidObjectId)
    {
        attachedStage.getObjectDatabase()->findOrCreateEntry(primPath, eArticulation, id);
        if (articulationDesc.fixBase && articulationDesc.fixBasePath != SdfPath())
        {
            // A.B. we created one more for fixed base ePTArticulationFixedBase
            attachedStage.getObjectDatabase()->findOrCreateEntry(articulationDesc.fixBasePath, eArticulationRootJoint, id + 1);
        }
    }

    articulationId = id;
    ICE_FREE(desc);
}

void createArticulationLink(AttachedStage& attachedStage, const SdfPath& jointPath,
                            const SdfPath& linkPath,
                            PhysxArticulationLinkDesc& desc,
                            const ObjectId articulationId,                           
                            const ObjectId parentId,
                            ObjectId& outId)
{
    desc.articulation = articulationId;
    desc.parent = parentId;
    
    ObjectDb* objDb = attachedStage.getObjectDatabase();

    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createObject(attachedStage, linkPath, desc);
    outId = id;

    if (id != kInvalidObjectId)
    {
        objDb->findOrCreateEntry(linkPath, eArticulationLink, id);
        attachedStage.bufferRequestRigidBodyMassUpdate(attachedStage.getStage()->GetPrimAtPath(linkPath));
    }

    desc.shapes.clear();
}

void convertBodyToArticulationLink(UsdStageWeakPtr stage, pxr::UsdGeomXformCache& xformCache, const SdfPath& bodyPath,
    DynamicPhysxRigidBodyDesc* bodyDesc, PhysxArticulationLinkDesc& tempArticulationLinkDesc)
{    
    ((DynamicPhysxRigidBodyDesc&)tempArticulationLinkDesc) = *bodyDesc;
    tempArticulationLinkDesc.type = eArticulationLink;
    bodyDesc->type = eArticulationLink;
}

void createLinkHierarchy(AttachedStage& attachedStage,
                         pxr::UsdGeomXformCache& xformCache,
                         const SdfPath& parentPath,
                         const JointVector& jointVector,
                         BodyMap& bodyMap,
                         const ObjectId articulationId,
                         const ObjectId parentId,
                         PathSet& articulatedJoints,
                         PathPathMap& linkPaths,
    PhysxArticulationLinkDesc& tempLink)
{
    PathSet childPaths;    
    for (size_t jIndex = 0; jIndex < jointVector.size(); jIndex++)
    {       
        const JointDescAndPath& jointDescAndPath = jointVector[jIndex];
        const SdfPath& jointLinkPath = jointDescAndPath.path;

        if (articulatedJoints.find(jointLinkPath) != articulatedJoints.end())
            continue;
        
        const PhysxJointDesc* jointDesc = jointDescAndPath.desc;
        const UsdPrim& jointPrim = jointDescAndPath.prim;
        const ArticulationJointType jointType = jointDesc->excludedFromArticulation ? eMaximalJoint : eStandardJoint;

        // maximal joints cannot be part of articulations
        if (jointType == eMaximalJoint)
        {
            continue;
        }

        if (jointDesc->type == eJointDistance)
        {
            CARB_LOG_ERROR("Articulation cannot contain a distance joint! (Please exclude it from articulation) (%s)", jointDesc->jointPrimPath.GetText());                
            continue;
        }
        else if (jointDesc->type == eJointGear)
        {
            CARB_LOG_ERROR("Articulation cannot contain a gear joint! (Please exclude it from articulation) (%s)", jointDesc->jointPrimPath.GetText());
            continue;
        }
        else if (jointDesc->type == eJointRackAndPinion)
        {
            CARB_LOG_ERROR("Articulation cannot contain a rack and pinion joint! (Please exclude it from articulation) (%s)", jointDesc->jointPrimPath.GetText());
            continue;
        }
        if(jointDesc->enableCollision)
        {
            CARB_LOG_WARN("Per-joint collision setting is ignored for articulations. (%s)", jointDesc->jointPrimPath.GetText());
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
                    CARB_LOG_WARN("Stiffness attribute is unsupported for articulation joints and will be ignored (%s).", jointDesc->jointPrimPath.GetText());
                }
                if(limit->damping > 0.0f)
                {
                    CARB_LOG_WARN("Damping attribute is unsupported for articulation joints and will be ignored (%s).", jointDesc->jointPrimPath.GetText());
                }
                if(limit->restitution > 0.0f)
                {
                    CARB_LOG_WARN("Restitution attribute is unsupported for articulation joints and will be ignored (%s).", jointDesc->jointPrimPath.GetText());
                }
                if(limit->bounceThreshold > 0.0f)
                {
                    CARB_LOG_WARN("Bounce threshold attribute is unsupported for articulation joints and will be ignored (%s).", jointDesc->jointPrimPath.GetText());
                }
            }
        }
        const SdfPath bodySdfPath0 = jointDesc->body0 == SdfPath() ? SdfPath() : jointDesc->body0;
        const SdfPath bodySdfPath1 = jointDesc->body1 == SdfPath() ? SdfPath() : jointDesc->body1;
        SdfPath newBodySdfPath = SdfPath();
        BodyMap::iterator bodyIterator;
        bool body0Found = false;
        bool body1Found = false;
        if (parentPath == bodySdfPath0 && (bodySdfPath1 != SdfPath() || jointType == eMaximalJoint))
        {
            bodyIterator = bodyMap.find(bodySdfPath1);
            if (bodyIterator != bodyMap.end() && bodyIterator->second.desc->type != eStaticBody)
            {
                body1Found = true;
                newBodySdfPath = bodySdfPath1;
            }
        }
        else if (parentPath == bodySdfPath1 && (bodySdfPath0 != SdfPath() || jointType == eMaximalJoint))
        {
            bodyIterator = bodyMap.find(bodySdfPath0);
            if (bodyIterator != bodyMap.end() && bodyIterator->second.desc->type != eStaticBody)
            {
                body0Found = true;
                newBodySdfPath = bodySdfPath0;
            }
        }
        if (body0Found || body1Found)
        {
            // If newBody is not already a child of the parent, create the articulation joint, else skip it
            if (childPaths.find(newBodySdfPath) == childPaths.end())
            {
                if (jointType == eStandardJoint)
                    childPaths.insert(newBodySdfPath);
                bool createMaximalJoint = false;
                if (newBodySdfPath != SdfPath())
                {
                    // check if body was not already created, if it was we have a circular dependency
                    if (linkPaths.find(newBodySdfPath) == linkPaths.end())
                    {
                        linkPaths[newBodySdfPath] = jointLinkPath;

                        ObjectId outId = kInvalidObjectId;
                        if (bodyIterator->second.desc->type == eDynamicBody)
                        {
                            convertBodyToArticulationLink(attachedStage.getStage(), xformCache, newBodySdfPath,
                                (DynamicPhysxRigidBodyDesc*)bodyIterator->second.desc, tempLink);

                            tempLink.articulationJointType = jointType;
                            tempLink.articulationJoint = jointDesc;
                            articulatedJoints.insert(jointLinkPath);
                            createArticulationLink(attachedStage,
                                jointLinkPath, newBodySdfPath, tempLink, articulationId, parentId, outId);

                            // do the children
                            if (jointType == eStandardJoint)
                                createLinkHierarchy(attachedStage, xformCache, newBodySdfPath, jointVector,
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
                            jointNames += linkRef.second.GetText();
                            jointNames += "\n";
                        }
                        REPORT_PHYSICS_ERROR(
                            "RigidBody (%s) appears to be a part of a closed articulation, which is not supported, please exclude one of the joints:\n%s from articulation, the joint will be now excluded from the articulation.",
                            newBodySdfPath.GetText(), jointNames.c_str());                        
                        articulatedJoints.insert(jointLinkPath);
                    }
                }
                else
                {
                    createMaximalJoint = true;
                }

                if (createMaximalJoint)
                {
                    // for a maximal joint to world, create directly a joint
                    UsdLoad* usdLoad = UsdLoad::getUsdLoad();
                    ObjectId body0 = bodySdfPath0 != SdfPath() ? attachedStage.getObjectDatabase()->findEntry(bodySdfPath0, eArticulationLink) : kInvalidObjectId;
                    ObjectId body1 = bodySdfPath1 != SdfPath() ? attachedStage.getObjectDatabase()->findEntry(bodySdfPath1, eArticulationLink) : kInvalidObjectId;
                    articulatedJoints.insert(jointLinkPath);

                    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createJoint(attachedStage, jointLinkPath, *jointDesc, body0, body1);

                    if (id != kInvalidObjectId)
                        attachedStage.getObjectDatabase()->findOrCreateEntry(jointLinkPath, eJoint, id);
                }
            }
            else
            {
                CARB_LOG_WARN("PhysX : articulation joint (%s) with body0 (%s) and body1 (%s), was skipped, since body1 is already defined as a child of body0", jointLinkPath.GetName().c_str(), bodySdfPath0.GetName().c_str(), bodySdfPath1.GetName().c_str());
            }        
        }
    }
}

void createArticulationLinks(AttachedStage& attachedStage, pxr::UsdGeomXformCache& xformCache,
                             BodyMap& bodyMap,
                             JointVector& jointVectorIn,
                             const ArticulationMap& articulationMap,
                             const JointPathIndexMap& jointPathIndexMap)
{
    PathSet articulatedJoints; // vector of joints that belong to articulation, those will be removed from the jointVector
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

            for (const SdfPath& jp : articulationDesc->articulatedJoints)
            {
                JointPathIndexMap::const_iterator fit = jointPathIndexMap.find(jp);
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
        
            const UsdPrim articulationPrim = attachedStage.getStage()->GetPrimAtPath(articulationRef.first);

            ObjectId articulationId = kInvalidObjectId;
            ObjectId parentId = kInvalidObjectId;
            ObjectId outId = kInvalidObjectId;
            BodyMap::iterator parentBodyIt = bodyMap.find(articulationDesc->rootPrim);

            // check if we start from an articulated body, parse its parameters
            // if we start from a body the articulation is not fixed
            SdfPath jointLinkPath = SdfPath();
            if (parentBodyIt != bodyMap.end())
            {
                if (parentBodyIt->second.desc->type == eDynamicBody)
                {
                    convertBodyToArticulationLink(attachedStage.getStage(), xformCache, articulationDesc->rootPrim, (DynamicPhysxRigidBodyDesc*)parentBodyIt->second.desc, *tempArticulationLinkDesc);
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
                jointLinkPath = articulationDesc->rootPrim;
                const UsdPrim jointPrim = attachedStage.getStage()->GetPrimAtPath(jointLinkPath);
                SdfPath rootBodyPath;
                if (rootJointDesc->body0 == SdfPath() || rootJointDesc->body1 == SdfPath())
                {
                    rootBodyPath = rootJointDesc->body0 != SdfPath() ? rootJointDesc->body0 : rootJointDesc->body1;
                    parentBodyIt = bodyMap.find(rootBodyPath);
                }
                else
                {
                    bool rootBodySet = false;
                    parentBodyIt = bodyMap.find(rootJointDesc->body0);
                    if (parentBodyIt != bodyMap.end())
                    {
                        if (parentBodyIt->second.desc->type == eStaticBody)
                        {
                            articulationDesc->staticRootBodyPrim = parentBodyIt->first;
                            rootBodyPath = rootJointDesc->body1;
                            parentBodyIt = bodyMap.find(rootBodyPath);
                            rootBodySet = true;
                        }
                        else if (parentBodyIt->second.desc->type == eDynamicBody)
                        {
                            const DynamicPhysxRigidBodyDesc* dynDesc = reinterpret_cast<const DynamicPhysxRigidBodyDesc*>(parentBodyIt->second.desc);
                            if (dynDesc->kinematicBody)
                            {
                                REPORT_PHYSICS_ERROR("Articulations with kinematic bodies are not supported, please exclude joint (%s) from articulation.", rootJointDesc->jointPrimPath.GetText());
                            }
                        }
                    }
                    if (!rootBodySet)
                    {
                        parentBodyIt = bodyMap.find(rootJointDesc->body1);
                        if (parentBodyIt != bodyMap.end())
                        {
                            if (parentBodyIt->second.desc->type == eStaticBody)
                            {
                                articulationDesc->staticRootBodyPrim = parentBodyIt->first;
                                rootBodyPath = rootJointDesc->body0;
                                parentBodyIt = bodyMap.find(rootBodyPath);
                            }
                            else if (parentBodyIt->second.desc->type == eDynamicBody)
                            {
                                const DynamicPhysxRigidBodyDesc* dynDesc = reinterpret_cast<const DynamicPhysxRigidBodyDesc*>(parentBodyIt->second.desc);
                                if (dynDesc->kinematicBody)
                                {
                                    REPORT_PHYSICS_ERROR(
                                        "Articulations with kinematic bodies are not supported, please exclude joint (%s) from articulation.",
                                        rootJointDesc->jointPrimPath.GetText());
                                }
                            }
                        }                        
                    }
                }
                if (rootBodyPath == SdfPath() || parentBodyIt == bodyMap.end())
                    continue;
                if (parentBodyIt->second.desc && parentBodyIt->second.desc->type == eDynamicBody)
                {
                    convertBodyToArticulationLink(attachedStage.getStage(), xformCache, rootBodyPath, (DynamicPhysxRigidBodyDesc*)parentBodyIt->second.desc, *tempArticulationLinkDesc);
                }
                else
                {
                    parentBodyIt = bodyMap.end();
                }
                articulationDesc->fixBase = true;
                articulationDesc->fixBasePath = jointLinkPath;
                articulatedJoints.insert(jointLinkPath);
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
                    SdfPath(), parentBodyIt->first, *tempArticulationLinkDesc, articulationId, parentId, outId);

                // do the children and construct the hierarchy
                PathPathMap linkPaths;
                linkPaths[parentBodyIt->first] = parentBodyIt->first;
                createLinkHierarchy(attachedStage, xformCache, parentBodyIt->first, jointVector, bodyMap, articulationId, outId, articulatedJoints, linkPaths, *tempArticulationLinkDesc);             
            }
        }
    }

    if (!articulatedJoints.empty())
    {
        for (size_t i = jointVectorIn.size(); i--;)
        {
            PathSet::const_iterator it = articulatedJoints.find(jointVectorIn[i].path);
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
