// SPDX-FileCopyrightText: Copyright (c) 2019-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "FilteredPairs.h"

#include <omni/physics/parse/IPhysicsSource.h>

using namespace PXR_NS;
using namespace carb;
using namespace omni::physics::schema;
using namespace omni::physx::usdparser;


void omni::physx::usdparser::collectFilteredPairs(AttachedStage& attachedStage, const SdfPath& primKey, const SdfPathVector& filterPairPaths, ObjectIdPairVector& pairVector)
{
    ObjectDb& objectDb = *attachedStage.getObjectDatabase();

    const ObjectIdMap* entriesFirst = objectDb.getEntries(primKey);
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
                    // traverse and find the childs (scoped query; prune a
                    // subtree once a body/shape is matched, as the legacy
                    // UsdPrimRange walk did with PruneChildren).
                    if (const omni::physics::parse::IPhysicsSource* source = attachedStage.getSource())
                    {
                        source->forEachDescendantPruned(
                            attachedStage.keyFor(fPath),
                            [&](omni::physics::parse::ObjectKey childKey) -> bool {
                                bool pairFound = false;
                                const ObjectIdMap* entriesSecond = objectDb.getEntries(attachedStage.pathFor(childKey));
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
