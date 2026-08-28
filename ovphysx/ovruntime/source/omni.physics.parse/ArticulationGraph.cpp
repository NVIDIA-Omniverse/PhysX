// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <omni/physics/parse/ArticulationGraph.h>

#include <carb/logging/Log.h>

#include <algorithm>
#include <climits>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace omni::physics::parse
{
namespace
{
using KeySet = std::unordered_set<ObjectKey, ObjectKey::Hash>;

// A joint reduced to the graph essentials, with all keys canonicalised.
struct GraphJoint
{
    ObjectKey key;    // canonical jointPrimKey
    ObjectKey body0;  // canonical
    ObjectKey body1;  // canonical
    bool      exclude = false;
};

struct ArticulationLink
{
    std::vector<ObjectKey> childs;
    ObjectKey              rootJoint;
    uint32_t               weight = 0;
    uint32_t               index = 0;
    bool                   hasFixedJoint = false;
    std::vector<ObjectKey> joints;
};

using ArticulationLinkMap = std::unordered_map<ObjectKey, ArticulationLink, ObjectKey::Hash>;
using BodyJointMap = std::unordered_map<ObjectKey, std::vector<const GraphJoint*>, ObjectKey::Hash>;

bool isInLinkMap(ObjectKey k, const std::vector<std::pair<ObjectKey, ArticulationLinkMap>>& maps)
{
    for (const auto& m : maps)
        if (m.second.find(k) != m.second.end())
            return true;
    return false;
}

// Is `ancestor` equal to, or a namespace ancestor of, `node`? (Mirrors the native
// SdfPath::GetAncestorsRange check, which includes the path itself.) Walks via the
// source, canonicalising each step.
bool isAncestorOrSelf(IPhysicsSource& source, ObjectKey ancestor, ObjectKey node)
{
    if (ancestor == node)
        return true;
    for (ObjectKey k = source.canonicalKey(source.getParent(node)); k.valid();
         k = source.canonicalKey(source.getParent(k)))
        if (k == ancestor)
            return true;
    return false;
}

void traverseHierarchy(ObjectKey linkKey,
                       ArticulationLinkMap& outMap,
                       const BodyJointMap& bodyJointMap,
                       uint32_t& index,
                       std::vector<ObjectKey>& linkOrder)
{
    if (outMap.find(linkKey) != outMap.end())
        return;
    linkOrder.push_back(linkKey);

    const auto bjIt = bodyJointMap.find(linkKey);
    if (bjIt == bodyJointMap.end())
        return;

    ArticulationLink& link = outMap[linkKey];
    link.weight = 0;
    link.index = index++;
    link.hasFixedJoint = false;

    for (const GraphJoint* j : bjIt->second)
    {
        link.joints.push_back(ObjectKey{}); // placeholder — patched below

        const bool b0Unknown = !j->body0.valid() || bodyJointMap.find(j->body0) == bodyJointMap.end();
        const bool b1Unknown = !j->body1.valid() || bodyJointMap.find(j->body1) == bodyJointMap.end();
        if (b0Unknown || b1Unknown)
        {
            if (j->exclude)
            {
                link.weight += 1000;
            }
            else
            {
                link.weight += 100000;
                link.rootJoint = j->key;
                link.hasFixedJoint = true;
            }
            link.childs.push_back({});
        }
        else
        {
            const ObjectKey other = (j->body0 == linkKey) ? j->body1 : j->body0;
            if (j->exclude)
            {
                link.childs.push_back(other);
                link.weight += 1000;
            }
            else
            {
                link.childs.push_back(other);
                link.weight += 100;
                traverseHierarchy(other, outMap, bodyJointMap, index, linkOrder);
            }
        }
        link.joints.back() = j->key;
    }
}

void traverseChilds(const ArticulationLink& link,
                    const ArticulationLinkMap& map,
                    uint32_t startIndex,
                    uint32_t distance,
                    int32_t* pathMatrix,
                    size_t mapSize)
{
    pathMatrix[startIndex + link.index * mapSize] = int32_t(distance);
    for (const ObjectKey& child : link.childs)
    {
        auto it = map.find(child);
        if (it == map.end())
            continue;
        const uint32_t childIndex = it->second.index;
        if (pathMatrix[startIndex + childIndex * mapSize] < 0)
            traverseChilds(it->second, map, startIndex, distance + 1, pathMatrix, mapSize);
    }
}

ObjectKey isNestedArticulation(IPhysicsSource& source, ObjectKey topKey, const ArticulationLinkMap& map)
{
    if (map.size() <= 1)
        return {};
    for (const auto& entry : map)
        if (!isAncestorOrSelf(source, topKey, entry.first))
            return {};
    return topKey;
}

ObjectKey getCenterOfGraph(const ArticulationLinkMap& map, const std::vector<ObjectKey>& linkOrder)
{
    const size_t size = map.size();
    if (size == 0)
        return {};
    std::vector<int32_t> pathMatrix(size * size, -1);

    for (const ObjectKey& k : linkOrder)
    {
        auto it = map.find(k);
        if (it == map.end())
            continue;
        traverseChilds(it->second, map, it->second.index, 0u, pathMatrix.data(), size);
    }

    int32_t shortestDistance = INT_MAX;
    size_t numChilds = 0;
    ObjectKey chosen;
    for (const ObjectKey& linkKey : linkOrder)
    {
        auto it = map.find(linkKey);
        if (it == map.end())
            continue;
        const ArticulationLink& link = it->second;
        int32_t longestPath = 0;
        for (size_t i = 0; i < size; ++i)
            longestPath = std::max(longestPath, pathMatrix[link.index + i * size]);

        if (longestPath < shortestDistance)
        {
            shortestDistance = longestPath;
            numChilds = link.childs.size();
            chosen = linkKey;
        }
        else if (longestPath == shortestDistance)
        {
            if (numChilds < link.childs.size())
            {
                numChilds = link.childs.size();
                chosen = linkKey;
            }
            else if (numChilds == link.childs.size())
            {
                for (const ObjectKey& ord : linkOrder)
                {
                    if (ord == chosen)
                        break;
                    if (ord == linkKey)
                    {
                        chosen = linkKey;
                        break;
                    }
                }
            }
        }
    }
    return chosen;
}

void emitArticulationDesc(IDescriptorAllocator& allocator,
                          const KeySet& jointKeys,
                          const ArticulationRootInput& root,
                          ObjectKey articulationKey,
                          ObjectKey rootKey,
                          const KeySet& articulatedJoints,
                          const KeySet& articulatedBodies,
                          std::vector<DescPtr<PhysxArticulationDesc>>& out)
{
    DescPtr<PhysxArticulationDesc> desc = allocateDesc<PhysxArticulationDesc>(allocator);
    if (!desc)
        return;
    desc->articulationPrim = articulationKey;
    desc->solverPositionIterationCount = root.fields.solverPositionIterationCount;
    desc->solverVelocityIterationCount = root.fields.solverVelocityIterationCount;
    desc->sleepThreshold = root.fields.sleepThreshold;
    desc->stabilizationThreshold = root.fields.stabilizationThreshold;
    desc->selfCollision = root.fields.selfCollision;
    desc->rootPrim = rootKey;
    desc->fixBaseKey = rootKey;
    // fixBase = the elected root is itself a joint (the floating-base pattern: the
    // articulation is anchored to world via a fixed joint).
    desc->fixBase = rootKey.valid() && jointKeys.count(rootKey) != 0;
    desc->articulatedJoints = articulatedJoints;
    desc->articulatedBodies = articulatedBodies;
    desc->sourceFilteredCollisions = root.sourceFilteredCollisions;
    out.push_back(std::move(desc));
}

} // namespace

void buildArticulations(IPhysicsSource& source,
                        IDescriptorAllocator& allocator,
                        const std::vector<ArticulationRootInput>& roots,
                        const std::vector<DescPtr<PhysxRigidBodyDesc>>& bodies,
                        const std::vector<DescPtr<PhysxJointDesc>>& joints,
                        std::vector<DescPtr<PhysxArticulationDesc>>& outArticulations)
{
    if (roots.empty())
        return;

    const auto C = [&](ObjectKey k) { return source.canonicalKey(k); };

    // Bodies keyed canonically.
    std::unordered_map<ObjectKey, PhysxRigidBodyDesc*, ObjectKey::Hash> bodyByKey;
    for (const auto& b : bodies)
        if (b)
            bodyByKey.emplace(C(b->primKey), b.get());

    // Accepted link = enabled dynamic non-kinematic body (disabled bodies are
    // emitted static, so `type == eDynamicBody` is exactly the enabled filter).
    auto bodyAccept = [&](ObjectKey canonKey) -> bool
    {
        auto it = bodyByKey.find(canonKey);
        if (it == bodyByKey.end() || !it->second || it->second->type != eDynamicBody)
            return false;
        return !static_cast<const DynamicPhysxRigidBodyDesc*>(it->second)->kinematicBody;
    };

    // Canonical joint views + lookups.
    std::vector<GraphJoint> gjoints;
    gjoints.reserve(joints.size());
    KeySet jointKeys;                                                          // all joints (fixBase test)
    std::unordered_map<ObjectKey, std::pair<ObjectKey, ObjectKey>, ObjectKey::Hash> jointBodies; // root-on-joint
    for (const auto& j : joints)
    {
        if (!j)
            continue;
        const ObjectKey jk = C(j->jointPrimKey);
        jointKeys.insert(jk);
        jointBodies.emplace(jk, std::make_pair(C(j->body0), C(j->body1)));
        if (j->jointEnabled)
            gjoints.push_back({ jk, C(j->body0), C(j->body1), j->excludedFromArticulation });
    }

    KeySet rootKeys;
    for (const ArticulationRootInput& r : roots)
        rootKeys.insert(C(r.key));

    BodyJointMap bodyJointMap;
    for (const GraphJoint& gj : gjoints)
    {
        if (bodyAccept(gj.body0))
            bodyJointMap[gj.body0].push_back(&gj);
        if (bodyAccept(gj.body1))
            bodyJointMap[gj.body1].push_back(&gj);
    }

    std::vector<ObjectKey> articulationLinkOrder;

    for (const ArticulationRootInput& r : roots)
    {
        if (!r.fields.articulationEnabled)
            continue;

        const ObjectKey artKey = C(r.key);

        // Nested-root guard.
        bool nested = false;
        for (ObjectKey k = C(source.getParent(r.key)); k.valid(); k = C(source.getParent(k)))
            if (rootKeys.count(k))
            {
                nested = true;
                break;
            }
        if (nested)
        {
            CARB_LOG_ERROR("UsdPhysics: Nested articulation roots are not allowed.");
            continue;
        }

        std::vector<ObjectKey> rootPrims;
        ObjectKey articulationBaseLinkKey = artKey;

        const auto bodyIt = bodyByKey.find(artKey);
        if (bodyIt != bodyByKey.end())
        {
            if (!bodyIt->second || bodyIt->second->type != eDynamicBody)
            {
                CARB_LOG_WARN("ArticulationRootAPI definition on non rigid body is not allowed; ignoring. Prim: %s",
                              std::string(source.sourceKeyToString(r.key)).c_str());
                continue;
            }
            if (static_cast<const DynamicPhysxRigidBodyDesc*>(bodyIt->second)->kinematicBody)
            {
                CARB_LOG_WARN("ArticulationRootAPI on a kinematic rigid body not allowed; ignoring. Prim: %s",
                              std::string(source.sourceKeyToString(r.key)).c_str());
                continue;
            }
            rootPrims.push_back(artKey);
        }
        else
        {
            const auto jIt = jointBodies.find(artKey);
            if (jIt != jointBodies.end())
            {
                const ObjectKey b0 = jIt->second.first;
                const ObjectKey b1 = jIt->second.second;
                if (!b0.valid() || !b1.valid())
                {
                    rootPrims.push_back(artKey);
                    articulationBaseLinkKey = b0.valid() ? b0 : b1;
                }
            }
        }

        // Walk the base-link namespace subtree (root first, mirroring
        // UsdPrimRange(baseLink)), seeding a per-subtree link map at each dynamic
        // body not already mapped.
        std::vector<std::pair<ObjectKey, ArticulationLinkMap>> articulationLinkMaps;
        articulationLinkOrder.clear();
        auto seed = [&](ObjectKey rawKey)
        {
            const ObjectKey k = C(rawKey);
            if (isInLinkMap(k, articulationLinkMaps))
                return;
            const auto it = bodyByKey.find(k);
            if (it != bodyByKey.end() && it->second && it->second->type == eDynamicBody)
            {
                articulationLinkMaps.emplace_back(k, ArticulationLinkMap());
                uint32_t idx = 0;
                traverseHierarchy(k, articulationLinkMaps.back().second, bodyJointMap, idx, articulationLinkOrder);
            }
        };
        seed(articulationBaseLinkKey);
        source.forEachDescendant(articulationBaseLinkKey, [&](ObjectKey k) { seed(k); });

        if (rootPrims.empty())
        {
            // No user-defined root: elect by weight, falling back to
            // isNestedArticulation / getCenterOfGraph for floating articulations.
            for (const auto& subtree : articulationLinkMaps)
            {
                const ArticulationLinkMap& linkMap = subtree.second;
                ObjectKey linkKey;
                uint32_t largestWeight = 0;
                bool hasFixedJoint = false;
                KeySet articulatedJoints;
                KeySet articulatedBodies;
                for (const ObjectKey& k : articulationLinkOrder)
                {
                    auto lit = linkMap.find(k);
                    if (lit == linkMap.end())
                        continue;
                    const ArticulationLink& link = lit->second;
                    if (link.hasFixedJoint)
                        hasFixedJoint = true;
                    if (link.weight > largestWeight)
                    {
                        linkKey = link.rootJoint.valid() ? link.rootJoint : k;
                        largestWeight = link.weight;
                    }
                    else if (link.weight == largestWeight)
                    {
                        const ObjectKey optionalKey = link.rootJoint.valid() ? link.rootJoint : k;
                        for (const ObjectKey& ord : articulationLinkOrder)
                        {
                            if (ord == linkKey)
                                break;
                            if (ord == optionalKey)
                            {
                                linkKey = optionalKey;
                                break;
                            }
                        }
                    }
                    for (size_t jj = link.joints.size(); jj-- > 0;)
                        if (link.joints[jj].valid())
                            articulatedJoints.insert(link.joints[jj]);
                    for (const ObjectKey& c : link.childs)
                        if (c.valid())
                            articulatedBodies.insert(c);
                }

                if (!hasFixedJoint)
                {
                    const ObjectKey nestedKey = isNestedArticulation(source, subtree.first, linkMap);
                    linkKey = nestedKey.valid() ? nestedKey : getCenterOfGraph(linkMap, articulationLinkOrder);
                }

                if (linkKey.valid())
                    emitArticulationDesc(allocator, jointKeys, r, artKey, linkKey, articulatedJoints,
                                         articulatedBodies, outArticulations);
            }
        }
        else
        {
            // User-defined root: aggregate only (no election).
            KeySet articulatedJoints;
            KeySet articulatedBodies;
            for (const auto& subtree : articulationLinkMaps)
                for (const auto& entry : subtree.second)
                {
                    const ArticulationLink& link = entry.second;
                    for (size_t jj = link.joints.size(); jj-- > 0;)
                        if (link.joints[jj].valid())
                            articulatedJoints.insert(link.joints[jj]);
                    for (const ObjectKey& c : link.childs)
                        if (c.valid())
                            articulatedBodies.insert(c);
                }
            for (const ObjectKey& rootKey : rootPrims)
                emitArticulationDesc(allocator, jointKeys, r, artKey, rootKey, articulatedJoints,
                                     articulatedBodies, outArticulations);
        }

        // ArticulationRootAPI applied but nothing aggregable → one default desc
        // anchored on the articulation prim.
        if (rootPrims.empty() && articulationLinkMaps.empty())
        {
            const KeySet empty;
            emitArticulationDesc(allocator, jointKeys, r, artKey, artKey, empty, empty, outArticulations);
        }
    }
}

} // namespace omni::physics::parse
