// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "FilteredPairs.h"

#include <omni/physics/parse/IPhysicsSource.h>

using namespace carb;
using namespace omni::physx::usdparser;


void omni::physx::usdparser::collectFilteredPairs(AttachedStage& attachedStage, omni::physics::parse::ObjectKey primKey, const std::vector<omni::physics::parse::ObjectKey>& filterPairKeys, ObjectIdPairVector& pairVector)
{
    ObjectDb& objectDb = *attachedStage.getObjectDatabase();

    const ObjectIdMap* entriesFirst = objectDb.getEntries(primKey);
    if (entriesFirst && !entriesFirst->empty())
    {
        auto itFirst = entriesFirst->begin();
        while (itFirst != entriesFirst->end())
        {
            for (const omni::physics::parse::ObjectKey fKey : filterPairKeys)
            {
                const ObjectIdMap* entriesSecond = objectDb.getEntries(fKey);
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
                    // traverse and find the childs (scoped query; prune a
                    // subtree once a body/shape is matched, as the legacy
                    // UsdPrimRange walk did with PruneChildren).
                    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
                    {
                        source->forEachDescendantPruned(
                            fKey,
                            [&](omni::physics::parse::ObjectKey childKey) -> bool {
                                bool pairFound = false;
                                // childKey is already attachedStage's own ObjectKey (minted by the
                                // same source as objectDb's key resolver), so look it up directly
                                // instead of round-tripping through pathFor -- no signature change
                                // needed since ObjectDb::getEntries already has an ObjectKey overload.
                                const ObjectIdMap* entriesSecond = objectDb.getEntries(childKey);
                                if (entriesSecond && !entriesSecond->empty())
                                {
                                    for (auto itSecond = entriesSecond->begin(); itSecond != entriesSecond->end();
                                         ++itSecond)
                                    {
                                        if (itSecond->first == eBody || itSecond->first == eShape)
                                        {
                                            pairFound = true;
                                            pairVector.push_back(std::make_pair(itFirst->second, itSecond->second));
                                        }
                                    }
                                }
                                return pairFound;
                            });
                    }
                }
            }
            itFirst++;
        }
    }
}
