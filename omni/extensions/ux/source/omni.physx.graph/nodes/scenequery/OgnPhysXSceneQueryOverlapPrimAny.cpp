// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXSceneQueryOverlapPrimAnyDatabase.h>

#include <omni/fabric/FabricUSD.h>
#include <pxr/usd/sdf/path.h>

#include "../../plugins/SceneQueryShared.h"

using namespace omni::physx;
using namespace omni::physx::graph;

class OgnPhysXSceneQueryOverlapPrimAny
{
public:
    static bool compute(OgnPhysXSceneQueryOverlapPrimAnyDatabase& db)
    {
        omni::fabric::PathC path;
        if (db.inputs.prim().size() == 0)
        {
            const auto& primPath = db.tokenToString(db.inputs.primPath());
            if (pxr::SdfPath::IsValidPathString(primPath))
            {
                db.logWarning("Prim Path input is deprecated. Please use the Prim input instead.");
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

        db.outputs.overlap() = getPhysXSceneQuery()->overlapShape(path.path, NULL, true);;
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
