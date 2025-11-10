// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQuerySweepSphereAllDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "OgnPhysXSceneQuery.inl"

class OgnPhysXSceneQuerySweepSphereAll : public OgnPhysXSceneQuery<OgnPhysXSceneQuerySweepSphereAll, OgnPhysXSceneQuerySweepSphereAllDatabase>
{
public:

    static void initialize(const GraphContextObj& context, const NodeObj& nodeObj)
    {
        SetConnectionCallbacks(context, nodeObj);
        
        // Since converting from paths to name tokens is expensive, we only generate these outputs if actually connected.
        // These outputs are also marked as deprecated and will be hidden, so they are only here for backwards compatibility.
        SetAttributeDeprecated(nodeObj, outputs::bodyPrimPaths.m_token, "Body Prim Paths output is deprecated. Please use the Body Prims output instead.");
        SetAttributeDeprecated(nodeObj, outputs::colliderPrimPaths.m_token, "Collider Prim Paths output is deprecated. Please use the Collider Prims output instead.");
        SetAttributeDeprecated(nodeObj, outputs::materialPaths.m_token, "Material Paths output is deprecated. Please use the Material Prims output instead.");
    }

    static bool compute(OgnPhysXSceneQuerySweepSphereAllDatabase& db)
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

        float radius;
        {
            const auto& input = db.inputs.radius();
            if(!input.resolved())
            {
                db.logError("No radius input supplied to node.");
                return false;
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

        // SweepSphere reports an error when the range is 0, but as inputs may be generated dynamically (like by offsets) we want to be able to accept this.
        if(range != 0.0f)
        {
            getPhysXSceneQuery()->sweepSphereAll(radius, origin, direction, range < 0.0f ? PX_MAX_F32 : range, gather, both_sides);
        }

        auto& state = db.template sharedState<OgnPhysXSceneQuerySweepSphereAll>();

        if(sort_by_distance)
        {
            std::sort(gatherList.begin(), gatherList.end(), sortHitByDistance);
        }

        // Write the outputs.
        int n = 0;
        db.outputs.colliderPrims().resize(gatherList.size());
        db.outputs.bodyPrims().resize(gatherList.size());

        bool bOutputCollidersAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::colliderPrimPaths.m_token);
        bool bOutputBodiesAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::bodyPrimPaths.m_token);
        bool bOutputMaterialsAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::materialPaths.m_token);

        db.outputs.colliderPrimPaths().resize(bOutputCollidersAsTokens ? gatherList.size() : 0);
        db.outputs.bodyPrimPaths().resize(bOutputBodiesAsTokens ? gatherList.size() : 0);
        db.outputs.positions().resize(gatherList.size());
        db.outputs.normals().resize(gatherList.size());
        db.outputs.distances().resize(gatherList.size());
        db.outputs.faceIndexes().resize(gatherList.size());
        db.outputs.materialPaths().resize(bOutputMaterialsAsTokens ? gatherList.size() : 0);
        db.outputs.materialPrims().resize(gatherList.size());

         for (const SweepHit& hit : gatherList)
        {
            db.outputs.colliderPrims()[n] = static_cast<omni::fabric::PathC>(hit.collision);
            db.outputs.bodyPrims()[n] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            if(bOutputCollidersAsTokens) db.outputs.colliderPrimPaths()[n] = asNameToken(hit.collision);
            if(bOutputBodiesAsTokens) db.outputs.bodyPrimPaths()[n] = asNameToken(hit.rigidBody);
            db.outputs.positions()[n] = hit.position;
            db.outputs.normals()[n] = hit.normal;
            db.outputs.distances()[n] = hit.distance;
            db.outputs.faceIndexes()[n] = hit.faceIndex;
            if(bOutputMaterialsAsTokens) db.outputs.materialPaths()[n] = asNameToken(hit.material);
            db.outputs.materialPrims()[n] = static_cast<omni::fabric::PathC>(hit.material);
            n++;
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()

