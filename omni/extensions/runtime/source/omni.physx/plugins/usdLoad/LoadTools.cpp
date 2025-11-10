// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "LoadTools.h"

#include <carb/logging/Log.h>

using namespace carb;

namespace omni
{
namespace physx
{
namespace usdparser
{

//// ObjectDb methods ////

bool ObjectDb::removeEntries(const pxr::SdfPath& path)
{
    Map::iterator fit = mPathMap.find(path);
    if (fit != mPathMap.end())
    {
        ObjectIdMap& entries = fit->second;
        entries.clear();

        mPathMap.erase(fit);
    }
    return true;
}

void ObjectDb::removeEntry(const pxr::SdfPath& path, ObjectCategory category, ObjectId entryId)
{
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
}


// Resolves ref to a string path
std::string GetBody(pxr::UsdRelationship const ref, const pxr::UsdPrim& jointPrim)
{
    pxr::SdfPathVector targets;
    ref.GetTargets(&targets);

    if (targets.size() == 0)
    {
        // CARB_LOG_WARN("Usd Physics: joint at %s has missing reference at %s\n",
        // jointPrim.GetPath().GetString().c_str(), ref.GetPath().GetString().c_str());
        return "";
    }
    if (targets.size() > 1)
    {
        // CARB_LOG_WARN("Usd Physics: joint at %s has unresolved multiple references for %s, only 1 allowed\n",
        // jointPrim.GetPath().GetString().c_str(), ref.GetPath().GetString().c_str());
        return "";
    }

    // TODO: we could add more error checking like in GetLocalFrame

    return targets.at(0).GetString();
}

// Resolves ref to a prim in stage which has to be an xformable. transform is extracted and returned in position and
// orientation
void GetLocalFrame(carb::Float3* position,
                   carb::Float4* orientation,
                   const pxr::UsdStageRefPtr stage,
                   pxr::UsdRelationship const ref,
                   const pxr::UsdPrim& jointPrim)
{
    pxr::SdfPathVector targets;
    ref.GetTargets(&targets);

    if (targets.size() == 0)
    {
        CARB_LOG_WARN("Usd Physics: joint at %s has missing reference at %s\n", jointPrim.GetPath().GetString().c_str(),
                      ref.GetPath().GetString().c_str());
        return;
    }
    if (targets.size() > 1)
    {
        CARB_LOG_WARN("Usd Physics: joint at %s has unresolved multiple references for %s, only 1 allowed\n",
                      jointPrim.GetPath().GetString().c_str(), ref.GetPath().GetString().c_str());
        return;
    }

    pxr::UsdPrim localFramePrim = stage->GetPrimAtPath(targets.at(0));
    if (!localFramePrim.IsValid())
    {
        CARB_LOG_WARN("Usd Physics: joint at %s has broken reference at %s. Missing prim = %s\n",
                      jointPrim.GetPath().GetString().c_str(), ref.GetPath().GetString().c_str(),
                      targets.at(0).GetString().c_str());
        return;
    }
    if (!localFramePrim.IsA<pxr::UsdGeomXformable>())
    {
        CARB_LOG_WARN("Usd Physics: joint at %s has invalid reference at %s. prim %s is not xformable\n",
                      jointPrim.GetPath().GetString().c_str(), ref.GetPath().GetString().c_str(),
                      targets.at(0).GetString().c_str());
        return;
    }

    // Copied from loadJointBodyAndTransform in PhysicsUsdToEcs
    pxr::UsdGeomXformable xform(localFramePrim);
    pxr::GfMatrix4d mat;
    bool resetXformStack = false;
    pxr::GfTransform transform;
    transform.SetIdentity();
    if (xform.GetLocalTransformation(&mat, &resetXformStack))
    {
        transform.SetMatrix(mat);
    }
    GfVec3ToFloat3(transform.GetTranslation(), *position);
    GfQuatToFloat4(transform.GetRotation().GetQuat(), *orientation);
}

bool ExtractTriangulatedFaces(std::vector<uint32_t>& triangles, pxr::UsdGeomMesh const& usdMesh)
{
    // indices and faces converted to triangles
    pxr::VtArray<int> indices;
    usdMesh.GetFaceVertexIndicesAttr().Get(&indices);

    pxr::VtArray<int> faces;
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
