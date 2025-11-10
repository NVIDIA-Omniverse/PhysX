// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryOverlapSphereAnyDatabase.h>

#include "../../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQueryOverlapSphereAny
{
public:
    static bool compute(OgnPhysXSceneQueryOverlapSphereAnyDatabase& db)
    {
        // Read all inputs.
        carb::Float3 position;
        {
            const auto& input = db.inputs.position();
            if(!input.resolved())
            {
                db.logError("No position input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                position = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                position = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for position.");
                return false;
            }
        }

        float radius;
        {
            const auto& input = db.inputs.radius();
            if(!input.resolved())
            {
                db.logError("No radius input.");
                radius = PX_MAX_F32;
            }
            else if(auto floatInput = input.get<float>())
            {
                radius = *floatInput;
            }
            else if(auto doubleInput = input.get<double>())
            {
                radius = (float) *doubleInput;
            }
            else
            {
                db.logError("Invalid data type input for radius.");
                return false;
            }
        }

        db.outputs.overlap() = getPhysXSceneQuery()->overlapSphere(radius, position, NULL, true);
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()

