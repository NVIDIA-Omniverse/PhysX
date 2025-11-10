// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryOverlapPrimAllDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "../../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

#include "OgnPhysXSceneQuery.inl"

class OgnPhysXSceneQueryOverlapPrimAll : public OgnPhysXSceneQuery<OgnPhysXSceneQueryOverlapPrimAll, OgnPhysXSceneQueryOverlapPrimAllDatabase>
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

    static bool compute(OgnPhysXSceneQueryOverlapPrimAllDatabase& db)
    {
        omni::fabric::PathC path;
        if (db.inputs.prim().size() == 0)
        {
            const auto& primPath = db.tokenToString(db.inputs.primPath());
            if (pxr::SdfPath::IsValidPathString(primPath))
            {
                db.logWarning("Prim Path input is deprecated. Please use Prim input instead.");
                path = omni::fabric::asInt(pxr::SdfPath(primPath));
            }
            else
            {
                db.logError("Invalid Prim Path input.");
                return false;
            }
        }
        else
        {
            path = db.inputs.prim()[0];
        }

        std::vector<OverlapHit> gatherList;
        auto gather = [&gatherList](const OverlapHit& hit) -> bool { gatherList.push_back(hit); return true;} ;

        getPhysXSceneQuery()->overlapShape(path.path, gather, false);

        auto& state = db.template sharedState<OgnPhysXSceneQueryOverlapPrimAll>();

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

