// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryOverlapBoxAnyDatabase.h>

#include "../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQueryOverlapBoxAny
{
public:
    static bool compute(OgnPhysXSceneQueryOverlapBoxAnyDatabase& db)
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

        carb::Float3 dimensions;
        {
            const auto& input = db.inputs.dimensions();
            if(!input.resolved())
            {
                db.logError("No dimensions input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                dimensions = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                dimensions = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for dimensions.");
                return false;
            }
        }

        // overlapBox uses half extent.
        dimensions.x *= 0.5f;
        dimensions.y *= 0.5f;
        dimensions.z *= 0.5f;

        GfRotation gf_rotation;
        {
            static const GfVec3d kAxes[] = { GfVec3d::XAxis(), GfVec3d::YAxis(), GfVec3d::ZAxis() };

            const auto& input = db.inputs.rotation();
            if(!input.resolved())
            {
                db.logError("No rotation input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                gf_rotation = GfRotation(kAxes[0], float3Input[0]) *
                              GfRotation(kAxes[1], float3Input[1]) *
                              GfRotation(kAxes[2], float3Input[2]);
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                gf_rotation = GfRotation(kAxes[0], double3Input[0]) *
                              GfRotation(kAxes[1], double3Input[1]) *
                              GfRotation(kAxes[2], double3Input[2]);
            }
            else
            {
                db.logError("Invalid data type input for rotation.");
                return false;
            }
        }
        carb::Float4 rotation = toFloat4(gf_rotation.GetQuaternion());

        db.outputs.overlap() = getPhysXSceneQuery()->overlapBox(dimensions, position, rotation, NULL, true);
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
