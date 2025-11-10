// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryRaycastAnyDatabase.h>

#include "../../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQueryRaycastAny
{
public:
    static bool compute(OgnPhysXSceneQueryRaycastAnyDatabase& db)
    {
        // Read all inputs.
        carb::Float3 origin;
        {
            const auto& input = db.inputs.origin();
            if(!input.resolved())
            {
                db.logError("No origin input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                origin = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                origin = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for origin.");
                return false;
            }
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
            const auto& input = db.inputs.raycastRange();
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

        // Raycast reports an error when the range is 0, but as inputs may be generated dynamically (like by offsets) we want to be able to accept this.
        bool bHit = false;
        if(range != 0.0f)
        {
            bHit = getPhysXSceneQuery()->raycastAny(origin, direction, range < 0.0f ? PX_MAX_F32 : range, both_sides);
        }

        // Write the outputs.

        db.outputs.hit() = bHit;
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()

