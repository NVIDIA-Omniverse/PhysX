// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "LoadTools.h"
#include "PhysXTools.h"

#include <private/omni/physics/CollisionShapeTransform.h>

#include <carb/logging/Log.h>

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

void getCollisionShapeLocalTransform(const AttachedStage& attachedStage,
                                     omni::physics::parse::ObjectKey collisionKey,
                                     omni::physics::parse::ObjectKey bodyKey,
                                     PXR_NS::GfVec3f& localPosOut,
                                     PXR_NS::GfQuatf& localRotOut,
                                     PXR_NS::GfVec3f& localScaleOut)
{
    // World transforms via the physics source. The replaced load-time xform
    // cache was pinned to EarliestTime, so use the time-independent
    // (EarliestTime, cached) overload to match it exactly.
    const PXR_NS::GfMatrix4d bodyWorld =
        internal::getWorldTransform(attachedStage, bodyKey);
    PXR_NS::GfMatrix4d shapeToBody(1.0);
    if (collisionKey != bodyKey)
    {
        const PXR_NS::GfMatrix4d collWorld =
            internal::getWorldTransform(attachedStage, collisionKey);
        // Collision prim's transform relative to the body (collWorld * bodyWorld^-1).
        shapeToBody = collWorld * bodyWorld.GetInverse();
    }

    // Matrix sourcing remains specific to this legacy path.
    omni::physics::decomposeCollisionShapeLocalTransform(
        shapeToBody, bodyWorld, localPosOut, localRotOut, localScaleOut);
}

//// ObjectDb methods ////

bool ObjectDb::removeEntries(const PXR_NS::SdfPath& path)
{
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    Map::iterator fit = mPathMap.find(path);
    if (fit != mPathMap.end())
    {
        ObjectIdMap& entries = fit->second;
        entries.clear();

        mPathMap.erase(fit);
    }
    if (key.valid())
    {
        KeyMap::iterator kit = mKeyMap.find(key);
        if (kit != mKeyMap.end())
            mKeyMap.erase(kit);
    }
    return true;
}

void ObjectDb::removeEntry(const PXR_NS::SdfPath& path, ObjectCategory category, ObjectId entryId)
{
    const omni::physics::parse::ObjectKey key = resolveKey(path);
    Map::iterator fit = mPathMap.find(path);
    if (fit != mPathMap.end())
    {
        ObjectIdMap& entries = fit->second;

        std::pair<ObjectIdMap::iterator, ObjectIdMap::iterator> pairIter = entries.equal_range(category);
        ObjectIdMap::iterator it = pairIter.first;
        while (it != pairIter.second)
        {
            if (it->second == entryId)
            {
                entries.erase(it);
                break;
            }

            it++;
        }

        if (entries.size() == 0)
            mPathMap.erase(fit);
    }
    if (key.valid())
        removeEntry(key, category, entryId);
}

void ObjectDb::removeEntry(omni::physics::parse::ObjectKey key, ObjectCategory category, ObjectId entryId)
{
    KeyMap::iterator fit = mKeyMap.find(key);
    if (fit != mKeyMap.end())
    {
        ObjectIdMap& entries = fit->second;

        std::pair<ObjectIdMap::iterator, ObjectIdMap::iterator> pairIter = entries.equal_range(category);
        ObjectIdMap::iterator it = pairIter.first;
        while (it != pairIter.second)
        {
            if (it->second == entryId)
            {
                entries.erase(it);
                break;
            }

            it++;
        }

        if (entries.size() == 0)
            mKeyMap.erase(fit);
    }
}


// Resolves ref to a string path
std::string GetBody(PXR_NS::UsdRelationship const ref, const PXR_NS::UsdPrim& jointPrim)
{
    PXR_NS::SdfPathVector targets;
    ref.GetTargets(&targets);

    if (targets.size() == 0)
    {
        return "";
    }
    if (targets.size() > 1)
    {
        return "";
    }

    // TODO: we could add more error checking like in GetLocalFrame

    return targets.at(0).GetString();
}

bool ExtractTriangulatedFaces(std::vector<uint32_t>& triangles, PXR_NS::UsdGeomMesh const& usdMesh)
{
    // indices and faces converted to triangles
    PXR_NS::VtArray<int> indices;
    usdMesh.GetFaceVertexIndicesAttr().Get(&indices);

    PXR_NS::VtArray<int> faces;
    usdMesh.GetFaceVertexCountsAttr().Get(&faces);

    if (indices.empty() || faces.empty())
        return false;

    triangles.reserve(faces.size() * 3);

    uint32_t indicesOffset = 0;

    uint32_t numIndices = uint32_t(indices.size());
    uint32_t numFaces = uint32_t(faces.size());
    bool valid = true;
    for (uint32_t i = 0; i < numFaces; i++)
    {
        const uint32_t faceCount = faces[i];
        valid &= faceCount >= 3 && indicesOffset + faceCount - 1 < numIndices;
        if (valid)
        {
            const uint32_t v0 = indices[indicesOffset];
            for (uint32_t faceIndex = 0; faceIndex < faceCount - 2; faceIndex++)
            {
                const uint32_t v1 = indices[indicesOffset + faceIndex + 1];
                const uint32_t v2 = indices[indicesOffset + faceIndex + 2];
                triangles.push_back(v0);
                triangles.push_back(v1);
                triangles.push_back(v2);
            }
        }
        indicesOffset += faceCount;
    }
    return valid;
}

} // namespace usdparser
} // namespace physx
} // namespace omni
