// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryRaycastClosestDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "OgnPhysXSceneQuery.inl"

class OgnPhysXSceneQueryRaycastClosest : public OgnPhysXSceneQuery<OgnPhysXSceneQueryRaycastClosest, OgnPhysXSceneQueryRaycastClosestDatabase>
{
public:

    static void initialize(const GraphContextObj& context, const NodeObj& nodeObj)
    {
        SetConnectionCallbacks(context, nodeObj);
        
        // Since converting from paths to name tokens is expensive, we only generate these outputs if actually connected.
        // These outputs are also marked as deprecated and will be hidden, so they are only here for backwards compatibility.
        SetAttributeDeprecated(nodeObj, outputs::bodyPrimPath.m_token, "Body Prim Path output is deprecated. Please use the Body Prim output instead.");
        SetAttributeDeprecated(nodeObj, outputs::colliderPrimPath.m_token, "Collider Prim Path output is deprecated. Please use the Collider Prim output instead.");
        SetAttributeDeprecated(nodeObj, outputs::materialPath.m_token, "Material Path output is deprecated. Please use the Material Prim output instead.");
    }

    static bool compute(OgnPhysXSceneQueryRaycastClosestDatabase& db)
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

        bool bHit = false;
        RaycastHit hit;
        // Raycast reports an error when the range is 0, but as inputs may be generated dynamically (like by offsets) we want to be able to accept this.
        if(range != 0.0f)
        {
            bHit = getPhysXSceneQuery()->raycastClosest(origin, direction, range < 0.0f ? PX_MAX_F32 : range, hit, both_sides);
        }
        if(!bHit)
        {
            hit.collision = 0;
            hit.rigidBody = 0;
            hit.position = carb::Float3 {0.0f, 0.0f, 0.0f};
            hit.normal = carb::Float3 {1.0f, 0.0f, 0.0f};
            hit.distance = -1.0f;
            hit.faceIndex = 0;
            hit.material = 0;
        }

        auto& state = db.template sharedState<OgnPhysXSceneQueryRaycastClosest>();

        // Write the outputs.
        db.outputs.hit() = bHit;
        if(bHit)
        {
            db.outputs.colliderPrim().resize(1);
            db.outputs.colliderPrim()[0] = static_cast<omni::fabric::PathC>(hit.collision);
            if(GetIsDeprecatedAttributeConnected(db, outputs::colliderPrimPath.m_token)) db.outputs.colliderPrimPath() = asNameToken(hit.collision);
            db.outputs.bodyPrim().resize(1);
            db.outputs.bodyPrim()[0] = static_cast<omni::fabric::PathC>(hit.rigidBody);
            if(GetIsDeprecatedAttributeConnected(db, outputs::bodyPrimPath.m_token)) db.outputs.bodyPrimPath() = asNameToken(hit.rigidBody);
            db.outputs.position() = hit.position;
            db.outputs.normal() = hit.normal;
            db.outputs.distance() = hit.distance;
            db.outputs.faceIndex() = hit.faceIndex;
            db.outputs.materialPrim().resize(1);
            db.outputs.materialPrim()[0] = static_cast<omni::fabric::PathC>(hit.material);
            if(GetIsDeprecatedAttributeConnected(db, outputs::materialPath.m_token)) db.outputs.materialPath() = asNameToken(hit.material);
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

