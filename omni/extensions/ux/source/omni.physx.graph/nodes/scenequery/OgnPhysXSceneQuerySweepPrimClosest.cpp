// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQuerySweepPrimClosestDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQuerySweepPrimClosest
{
public:

    static bool compute(OgnPhysXSceneQuerySweepPrimClosestDatabase& db)
    {
        // Read all inputs.
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

        bool bHit = false;
        SweepHit hit;
        // SweepPrim reports an error when the range is 0, but as inputs may be generated dynamically (like by offsets) we want to be able to accept this.
        if(range != 0.0f)
        {
            bHit = getPhysXSceneQuery()->sweepShapeClosest(path.path, direction, range < 0.0f ? PX_MAX_F32 : range, hit, both_sides);
        }

        // Write the outputs.
        db.outputs.hit() = bHit;
        if(bHit)
        {
            db.outputs.colliderPrim().resize(1);
            db.outputs.colliderPrim()[0] = static_cast<omni::fabric::PathC>(hit.collision);
            db.outputs.bodyPrim().resize(1);
            db.outputs.bodyPrim()[0] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            db.outputs.position() = hit.position;
            db.outputs.normal() = hit.normal;
            db.outputs.distance() = hit.distance;
            db.outputs.faceIndex() = hit.faceIndex;
            db.outputs.materialPrim().resize(1);
            db.outputs.materialPrim()[0] = static_cast<omni::fabric::PathC>(hit.material);
        }
        else
        {
            db.outputs.colliderPrim().resize(0);
            db.outputs.bodyPrim().resize(0);
            db.outputs.materialPrim().resize(0);
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()

