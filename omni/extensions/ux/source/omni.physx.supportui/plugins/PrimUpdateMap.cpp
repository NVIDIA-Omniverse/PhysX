// SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "PrimUpdateMap.h"

#include <carb/Types.h>
#include <carb/Defines.h>
#include <carb/profiler/Profile.h>
#include <carb/logging/Log.h>

using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{

void PrimUpdateMap::addPrim(const PrimDef& primDef)
{
    const UsdPrim& prim = primDef.mPrim;

    // if a parent is in the map, we don't need to add the prim
    if (!exists(prim))
    {
        // before we add we need to check if some child is not already in the map
        UsdPrimRange range(prim);
        for (UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const UsdPrim& childPrim = *iter;

            if (!childPrim)
                continue;

            TPrimAddMap::iterator fit = mPrimAddMap.find(childPrim.GetPrimPath());

            if (fit != mPrimAddMap.end())
            {
                mPrimAddMap.erase(fit);
            }
        }
        mPrimAddMap[prim.GetPrimPath()] = primDef;
    }
}

void PrimUpdateMap::removePrim(const SdfPath& primPath)
{
    mPrimAddMap.erase(primPath);
}

bool PrimUpdateMap::exists(const UsdPrim& prim) const
{
    if (mPrimAddMap.empty())
        return false;

    UsdPrim parent = prim;
    const UsdPrim root = prim.GetStage()->GetPseudoRoot();

    while (parent != root)
    {
        TPrimAddMap::const_iterator it = mPrimAddMap.find(parent.GetPath());

        if (it != mPrimAddMap.end())
        {
            return true;
        }

        parent = parent.GetParent();
    }

    return false;
}

// remove invalid prims
void PrimUpdateMap::checkMap(const UsdStageWeakPtr stage)
{
    TPrimAddMap::iterator it = mPrimAddMap.begin();
    const TPrimAddMap::iterator itEnd = mPrimAddMap.end();

    while (it != itEnd)
    {
        if (!it->second.mPrim || !stage->GetPrimAtPath(it->first))
        {
            it = mPrimAddMap.erase(it);
        }
        else
            it++;
    }
}

} // namespace physx
} // namespace omni
