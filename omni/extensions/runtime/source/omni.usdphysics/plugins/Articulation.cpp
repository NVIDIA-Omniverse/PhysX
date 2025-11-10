// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <private/omni/physics/schema/IUsdPhysicsListener.h>

#include "Articulation.h"
#include "FilteredPairs.h"

using namespace pxr;

namespace omni
{
namespace physics
{
namespace schema
{
bool checkNestedArticulationRoot(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, const ArticulationMap& articulationMap)
{
    UsdPrim parent = usdPrim.GetParent();
    while (parent && parent != stage->GetPseudoRoot())
    {
        if (articulationMap.find(parent.GetPrimPath()) != articulationMap.end())
            return true;
        parent = parent.GetParent();
    }

    return false;
}

ArticulationDesc* parseArticulation(const pxr::UsdStageWeakPtr stage, const pxr::UsdPrim& usdPrim, const ArticulationMap& articulationMap)
{
    if (checkNestedArticulationRoot(stage, usdPrim, articulationMap))
    {
        CARB_LOG_ERROR("UsdPhysics: Nested articulation roots are not allowed.");
        return nullptr;
    }

    ArticulationDesc* desc = new ArticulationDesc();

    parseFilteredPairs(stage, usdPrim, desc->filteredCollisions);
    return desc;
}

struct ArticulationLink
{ 
    SdfPathVector   childs;
    SdfPath         rootJoint;
    uint32_t        weight;
    uint32_t        index;
    bool            hasFixedJoint;
    SdfPathVector   joints;
};

using ArticulationLinkMap = std::map<pxr::SdfPath, ArticulationLink>;
using BodyJointMap = pxr::TfHashMap<pxr::SdfPath, std::vector<const JointDesc*>, pxr::SdfPath::Hash>;

bool isInLinkMap(const SdfPath& path, const std::vector<ArticulationLinkMap>& linkMaps)
{
    for (size_t i = 0; i < linkMaps.size(); i++)
    {
        ArticulationLinkMap::const_iterator it = linkMaps[i].find(path);
        if (it != linkMaps[i].end())
            return true;
    }

    return false;
}

void traverseHierarchy(const pxr::UsdStageWeakPtr stage, const SdfPath& linkPath, ArticulationLinkMap& articulationLinkMap, const BodyJointMap& bodyJointMap, uint32_t& index, SdfPathVector& linkOrderVector)
{
    // check if we already parsed this link
    ArticulationLinkMap::const_iterator artIt = articulationLinkMap.find(linkPath);
    if (artIt != articulationLinkMap.end())
        return;

    linkOrderVector.push_back(linkPath);

    BodyJointMap::const_iterator bjIt = bodyJointMap.find(linkPath);
    if (bjIt != bodyJointMap.end())
    {
        ArticulationLink& link = articulationLinkMap[linkPath];
        link.weight = 0;
        link.index = index++;
        link.hasFixedJoint = false;
        const std::vector<const JointDesc*>& joints = bjIt->second;
        for (size_t i = 0; i < joints.size(); i++)
        {            
            const JointDesc* desc = joints[i];
            link.joints.push_back(desc->usdPrim.GetPrimPath());
            if (desc->body0 == SdfPath() || (bodyJointMap.find(desc->body0) == bodyJointMap.end()) ||
                desc->body1 == SdfPath() || (bodyJointMap.find(desc->body1) == bodyJointMap.end()))
            {
                if (desc->excludeFromArticulation)
                {
                    link.weight += 1000;
                }
                else
                {
                    link.weight += 100000;
                    link.rootJoint = desc->usdPrim.GetPrimPath();
                    link.hasFixedJoint = true;
                }                
                link.childs.push_back(SdfPath());                                
            }
            else
            {
                if (desc->excludeFromArticulation)
                {
                    link.childs.push_back(desc->body0 == linkPath ? desc->body1 : desc->body0);
                    link.weight += 1000;
                }
                else
                {
                    link.childs.push_back(desc->body0 == linkPath ? desc->body1 : desc->body0);
                    link.weight += 100;
                    traverseHierarchy(stage, link.childs.back(), articulationLinkMap, bodyJointMap, index, linkOrderVector);
                }
            }
        }
    }
}

void traverseChilds(const ArticulationLink& link, const ArticulationLinkMap& map, uint32_t startIndex, uint32_t distance, int32_t* pathMatrix)
{
    const size_t mapSize = map.size();
    const uint32_t currentIndex = link.index;
    pathMatrix[startIndex + currentIndex * mapSize] = distance;

    for (size_t i = 0; i < link.childs.size(); i++)
    {
        ArticulationLinkMap::const_iterator it = map.find(link.childs[i]);
        if (it != map.end())
        {
            const uint32_t childIndex = it->second.index;
            if (pathMatrix[startIndex + childIndex * mapSize] < 0)
            {
                traverseChilds(it->second, map, startIndex, distance + 1, pathMatrix);
            }
        }        
    }
}

pxr::SdfPath getCenterOfGraph(const ArticulationLinkMap& map, const SdfPathVector& linkOrderVector)
{
    const size_t size = map.size();
    int32_t* pathMatrix = new int32_t[size * size];
    for (size_t i = 0; i < size; i ++)
    {
        for (size_t j = 0; j < size; j++)
        {
            pathMatrix [i + j * size] = -1;
        }
    }

    for (ArticulationLinkMap::const_reference& ref : map)
    {
        const uint32_t startIndex = ref.second.index;
        uint32_t distance = 0;
        traverseChilds(ref.second, map, startIndex, distance, pathMatrix);
    }

    int32_t shortestDistance = INT_MAX;
    size_t numChilds = 0;
    SdfPath primpath = SdfPath();
    for (ArticulationLinkMap::const_reference& ref : map)
    {
        const uint32_t startIndex = ref.second.index;
        int32_t longestPath = 0;
        for (size_t i = 0; i < size; i++)
        {
            if (pathMatrix[startIndex + i * size] > longestPath)
            {
                longestPath = pathMatrix[startIndex + i * size];
            }
        }

        // this needs to be deterministic, get the shortest path
        // if there are more paths with same lenght, pick the one with more childs
        // if there are more with same path and same amount of childs, pick the one with lowest hash
        // The lowest hash is not right, this is wrong, it has to be the first link ordered by the traversal
        if (longestPath < shortestDistance)
        {
            shortestDistance = longestPath;
            numChilds = ref.second.childs.size();
            primpath = ref.first;
        }
        else if (longestPath == shortestDistance)
        {
            if (numChilds < ref.second.childs.size())
            {
                numChilds = ref.second.childs.size();
                primpath = ref.first;
            }
            else if (numChilds == ref.second.childs.size())
            {
                for (const SdfPath& orderPath : linkOrderVector)
                {
                    if (orderPath == primpath)
                    {
                        break;
                    }
                    else if (orderPath == ref.first)
                    {
                        primpath = ref.first;
                    }
                }
            }
        }
    }

    delete [] pathMatrix;

    return primpath;
}

void finalizeArticulations(const pxr::UsdStageWeakPtr stage, ArticulationMap& articulationMap, const BodyMap& bodyMap, const JointMap& jointMap)
{
    BodyJointMap bodyJointMap;
    if (!articulationMap.empty())
    {
        // construct the BodyJointMap
        bodyJointMap.reserve(bodyMap.size());
        for (JointMap::const_reference& jointIt : jointMap)
        {
            const JointDesc* desc = jointIt.second;
            if (desc->jointEnabled)
            {
                if (desc->body0 != SdfPath())
                {
                    BodyMap::const_iterator fit = bodyMap.find(desc->body0);
                    if (fit != bodyMap.end() && fit->second->type == ObjectType::eRigidBody)
                    {
                        RigidBodyDesc* rigidBodyDesc = (RigidBodyDesc*)fit->second;
                        if (rigidBodyDesc->rigidBodyEnabled && !rigidBodyDesc->kinematicBody)
                        {
                            bodyJointMap[desc->body0].push_back(desc);
                        }
                    }                    
                }
                if (desc->body1 != SdfPath())
                {
                    BodyMap::const_iterator fit = bodyMap.find(desc->body1);
                    if (fit != bodyMap.end() && fit->second->type == ObjectType::eRigidBody)
                    {
                        RigidBodyDesc* rigidBodyDesc = (RigidBodyDesc*)fit->second;
                        if (rigidBodyDesc->rigidBodyEnabled && !rigidBodyDesc->kinematicBody)
                        {
                            bodyJointMap[desc->body1].push_back(desc);
                        }
                    }
                }
            }
        }
    }

    SdfPathVector articulationLinkOrderVector;

    // first get user defined articulation roots
    // then search for the best root in the articulation hierarchy
    for (ArticulationMap::const_reference& it : articulationMap)
    {
        const SdfPath& articulationPath = it.first;
        SdfPath articulationBaseLinkPath = articulationPath;
        // check if its a floating articulation
        {
            BodyMap::const_iterator bodyIt = bodyMap.find(articulationPath);
            if (bodyIt != bodyMap.end())
            {
                if (bodyIt->second->type != ObjectType::eRigidBody)
                {
                    CARB_LOG_WARN(
                        "ArticulationRootAPI definition on non rigid body is not allowed, articulation root will be ignored. Prim: %s",
                        articulationPath.GetText());
                    continue;
                }
                RigidBodyDesc* rigidBodyDesc = (RigidBodyDesc*)bodyIt->second;
                if (!rigidBodyDesc->rigidBodyEnabled)
                {
                    CARB_LOG_WARN(
                        "ArticulationRootAPI definition on a static rigid body is not allowed, articulation root will be ignored. Prim: %s",
                        articulationPath.GetText());
                    continue;
                }
                if (rigidBodyDesc->kinematicBody)
                {
                    CARB_LOG_WARN(
                        "ArticulationRootAPI definition on a kinematic rigid body is not allowed, articulation root will be ignored. Prim: %s",
                        articulationPath.GetText());
                    continue;
                }
                it.second->rootPrims.push_back(bodyIt->first);
            }
            else
            {
                JointMap::const_iterator jointIt = jointMap.find(articulationPath);
                if (jointIt != jointMap.end())
                {
                    const SdfPath& jointPath = jointIt->first;
                    const JointDesc* jointDesc = jointIt->second;
                    if (jointDesc->body0 == SdfPath() || jointDesc->body1 == SdfPath())
                    {                        
                        it.second->rootPrims.push_back(jointPath);
                        articulationBaseLinkPath = jointDesc->body0 == SdfPath() ? jointDesc->body1 : jointDesc->body0;
                    }
                }
            }
        }

        // search through the hierarchy for the best root        
        const UsdPrim articulationPrim = stage->GetPrimAtPath(articulationBaseLinkPath);
        CARB_ASSERT(articulationPrim);
        if (!articulationPrim)
            continue;        
        UsdPrimRange range(articulationPrim, UsdTraverseInstanceProxies());
        std::vector<ArticulationLinkMap> articulationLinkMaps;
        articulationLinkOrderVector.clear();

        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const pxr::UsdPrim& prim = *iter;
            if (!prim)
                continue;
            const SdfPath primPath = prim.GetPrimPath();            
            if (isInLinkMap(primPath, articulationLinkMaps))
            {
                iter.PruneChildren(); // Skip the subtree rooted at this prim
                continue;
            }

            BodyMap::const_iterator bodyIt = bodyMap.find(primPath);
            if (bodyIt != bodyMap.end() && bodyIt->second->type == ObjectType::eRigidBody)
            {
                articulationLinkMaps.push_back(ArticulationLinkMap());
                uint32_t index = 0;
                traverseHierarchy(stage, primPath, articulationLinkMaps.back(), bodyJointMap, index, articulationLinkOrderVector);
            }
        }

        if (it.second->rootPrims.empty())
        {
            for (size_t i = 0; i < articulationLinkMaps.size(); i++)
            {
                const ArticulationLinkMap& map = articulationLinkMaps[i];
                SdfPath linkPath = SdfPath();
                uint32_t largestWeight = 0;
                bool hasFixedJoint = false;
                for (ArticulationLinkMap::const_reference& linkIt : map)
                {
                    if (linkIt.second.hasFixedJoint)
                    {
                        hasFixedJoint = true;
                    }
                    if (linkIt.second.weight > largestWeight)
                    {
                        linkPath = (linkIt.second.rootJoint != SdfPath()) ? linkIt.second.rootJoint : linkIt.first;
                        largestWeight = linkIt.second.weight;
                    }
                    else if (linkIt.second.weight == largestWeight)
                    {
                        const SdfPath optionalLinkPath = (linkIt.second.rootJoint != SdfPath()) ? linkIt.second.rootJoint : linkIt.first;
                        for (const SdfPath& orderPath : articulationLinkOrderVector)
                        {
                            if (orderPath == linkPath)
                            {
                                break;
                            }
                            else if (orderPath == optionalLinkPath)
                            {
                                linkPath = optionalLinkPath;
                            }
                        }
                    }

                    for (size_t j = linkIt.second.joints.size(); j--;)
                    {
                        it.second->articulatedJoints.insert(linkIt.second.joints[j]);
                    }
                }

                // for floating articulation lets find the body with the shortest paths (center of graph)
                if (!hasFixedJoint)
                {
                    linkPath = getCenterOfGraph(map, articulationLinkOrderVector);
                }

                if (linkPath != SdfPath())
                {
                    it.second->rootPrims.push_back(linkPath);
                }
            }
        }
        else
        {
            for (size_t i = 0; i < articulationLinkMaps.size(); i++)
            {
                const ArticulationLinkMap& map = articulationLinkMaps[i];
                SdfPath linkPath = SdfPath();
                uint32_t largestWeight = 0;
                bool hasFixedOrLoopJoint = false;
                for (ArticulationLinkMap::const_reference& linkIt : map)
                {
                    for (size_t j = linkIt.second.joints.size(); j--;)
                    {
                        it.second->articulatedJoints.insert(linkIt.second.joints[j]);
                    }
                }
            }
        }
        for (size_t i = 0; i < articulationLinkMaps.size(); i++)
        {
            const ArticulationLinkMap& map = articulationLinkMaps[i];
            for (ArticulationLinkMap::const_reference& linkIt : map)
            {
                it.second->articulatedBodies.insert(linkIt.second.childs.begin(), linkIt.second.childs.end());
            }
        }
    }
    

}

} // namespace schema
} // namespace physics
} // namespace omni
