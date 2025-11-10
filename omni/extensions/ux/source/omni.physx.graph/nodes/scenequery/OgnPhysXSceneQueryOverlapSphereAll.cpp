// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryOverlapSphereAllDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "OgnPhysXSceneQuery.inl"

class OgnPhysXSceneQueryOverlapSphereAll : public OgnPhysXSceneQuery<OgnPhysXSceneQueryOverlapSphereAll, OgnPhysXSceneQueryOverlapSphereAllDatabase>
{
public:

    static void initialize(const GraphContextObj& context, const NodeObj& nodeObj)
    {
        SetConnectionCallbacks(context, nodeObj);
        
        // Since converting from paths to name tokens is expensive, we only generate these outputs if actually connected.
        // These outputs are also marked as deprecated and will be hidden, so they are only here for backwards compatibility.
        SetAttributeDeprecated(nodeObj, outputs::bodyPrimPaths.m_token, "Body Prim Paths output is deprecated. Please use the Body Prims output instead.");
        SetAttributeDeprecated(nodeObj, outputs::colliderPrimPaths.m_token, "Collider Prim Paths output is deprecated. Please use the Collider Prims output instead.");
    }

    static bool compute(OgnPhysXSceneQueryOverlapSphereAllDatabase& db)
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

        std::vector<OverlapHit> gatherList;
        auto gather = [&gatherList](const OverlapHit& hit) -> bool { gatherList.push_back(hit); return true;} ;

        getPhysXSceneQuery()->overlapSphere(radius, position, gather, false);

        auto& state = db.template sharedState<OgnPhysXSceneQueryOverlapSphereAll>();

        // Write the outputs.
        int n = 0;
        db.outputs.colliderPrims().resize(gatherList.size());
        db.outputs.bodyPrims().resize(gatherList.size());
        bool bOutputCollidersAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::colliderPrimPaths.m_token);
        bool bOutputBodiesAsTokens = GetIsDeprecatedAttributeConnected(db, outputs::bodyPrimPaths.m_token);
        db.outputs.colliderPrimPaths().resize(bOutputCollidersAsTokens ? gatherList.size() : 0);
        db.outputs.bodyPrimPaths().resize(bOutputBodiesAsTokens ? gatherList.size() : 0);
        for (const OverlapHit& hit : gatherList)
        {
            db.outputs.colliderPrims()[n] = static_cast<omni::fabric::PathC>(hit.collision);
            db.outputs.bodyPrims()[n] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            if(bOutputCollidersAsTokens) db.outputs.colliderPrimPaths()[n] = asNameToken(hit.collision);
            if(bOutputBodiesAsTokens) db.outputs.bodyPrimPaths()[n] = asNameToken(hit.rigidBody);
            n++;
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
