// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQuerySweepBoxAllDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "../../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQuerySweepBoxAll
{
public:

    static bool compute(OgnPhysXSceneQuerySweepBoxAllDatabase& db)
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
            getPhysXSceneQuery()->sweepBoxAll(dimensions, origin, rotation, direction, range < 0.0f ? PX_MAX_F32 : range, gather, both_sides);
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

