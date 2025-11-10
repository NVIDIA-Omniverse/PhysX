// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "FilteredPairs.h"

using namespace pxr;
using namespace carb;
using namespace omni::physics::schema;
using namespace omni::physx::usdparser;


void omni::physx::usdparser::collectFilteredPairs(AttachedStage& attachedStage, const SdfPath& primPath, const SdfPathVector& filterPairPaths, ObjectIdPairVector& pairVector)
{
    ObjectDb& objectDb = *attachedStage.getObjectDatabase();

    const ObjectIdMap* entriesFirst = objectDb.getEntries(primPath);
    if (entriesFirst && !entriesFirst->empty())
    {
        auto itFirst = entriesFirst->begin();
        while (itFirst != entriesFirst->end())
        {
            for (const SdfPath& fPath : filterPairPaths)
            {
                const ObjectIdMap* entriesSecond = objectDb.getEntries(fPath);
                if (entriesSecond && !entriesSecond->empty())
                {
                    auto itSecond = entriesSecond->begin();
                    while (itSecond != entriesSecond->end())
                    {
                        const ObjectId firstObject = itFirst->second;
                        const ObjectId secondObject = itSecond->second;
                        pairVector.push_back(std::make_pair(firstObject, secondObject));
                        itSecond++;
                    }
                }
                else
                {
                    // traverse and find the childs
                    UsdPrimRange range(attachedStage.getStage()->GetPrimAtPath(fPath));
                    for (pxr::UsdPrimRange::const_iterator iterChilds = range.begin(); iterChilds != range.end();
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
                                    pairVector.push_back(std::make_pair(firstObject, secondObject));
                                }
                                itSecond++;
                            }
                        }
                        if (pairFound)
                            iterChilds.PruneChildren();
                    }
                }
            }
            itFirst++;
        }
    }
}
