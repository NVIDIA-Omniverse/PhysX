// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQuerySweepPrimAllDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "../../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQuerySweepPrimAll
{
public:

    static bool compute(OgnPhysXSceneQuerySweepPrimAllDatabase& db)
    {
        omni::fabric::PathC path;
        if (db.inputs.prim().size() == 1)
        {
            path = db.inputs.prim()[0];
        }
        else
        {
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
            return true;
        }

        carb::Float3 direction;
        {
            const auto& input = db.inputs.direction();
            if(!input.resolved())
            {
                db.logError("No direction input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                direction = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                direction = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for direction.");
                return false;
            }
        }

        float range;
        {
            const auto& input = db.inputs.sweepRange();
            if(!input.resolved())
            {
                range = PX_MAX_F32;
            }
            else if(auto floatInput = input.get<float>())
            {
                range = *floatInput;
            }
            else if(auto doubleInput = input.get<double>())
            {
                range = (float) *doubleInput;
            }
            else
            {
                db.logError("Invalid data type input for range.");
                return false;
            }
        }
        bool both_sides = db.inputs.bothSides();
        bool sort_by_distance = db.inputs.sortByDistance();

        std::vector<SweepHit> gatherList;
        auto gather = [&gatherList](const SweepHit& hit) -> bool { gatherList.push_back(hit); return true;} ;

        // Sweep reports an error when the range is 0, but as inputs may be generated dynamically (like by offsets) we want to be able to accept this.
        if(range != 0.0f)
        {
            getPhysXSceneQuery()->sweepShapeAll(path.path, direction, range < 0.0f ? PX_MAX_F32 : range, gather, both_sides);
        }

        if(sort_by_distance)
        {
            std::sort(gatherList.begin(), gatherList.end(), sortHitByDistance);
        }

        // Write the outputs.
        int n = 0;
        db.outputs.colliderPrims().resize(gatherList.size());
        db.outputs.bodyPrims().resize(gatherList.size());
        db.outputs.positions().resize(gatherList.size());
        db.outputs.normals().resize(gatherList.size());
        db.outputs.distances().resize(gatherList.size());
        db.outputs.faceIndexes().resize(gatherList.size());
        db.outputs.materialPrims().resize(gatherList.size());

         for (const SweepHit& hit : gatherList)
        {
            db.outputs.colliderPrims()[n] = static_cast<omni::fabric::PathC>(hit.collision);
            db.outputs.bodyPrims()[n] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            db.outputs.positions()[n] = hit.position;
            db.outputs.normals()[n] = hit.normal;
            db.outputs.distances()[n] = hit.distance;
            db.outputs.faceIndexes()[n] = hit.faceIndex;
            db.outputs.materialPrims()[n] = static_cast<omni::fabric::PathC>(hit.material);
            n++;
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()

