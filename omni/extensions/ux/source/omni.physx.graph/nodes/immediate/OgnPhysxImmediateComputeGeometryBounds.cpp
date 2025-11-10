// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"


#include <OgnPhysXImmediateComputeGeometryBoundsDatabase.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/graph/core/BundlePrims.h>
#include "../../plugins/immediate/ImmediateNode.h"
#include "../../plugins/immediate/ImmediateShared.h"
#include "../../plugins/MultiThreader.h"


#include "../../plugins/immediate/ImmediateHashMeshes.h"
#include "../../plugins/immediate/ImmediateFindDuplicates.h"
#include "../../plugins/immediate/ImmediateCookMeshes.h"
#include "../../plugins/immediate/ImmediateCreateGeometries.h"

using namespace ::physx;
using namespace omni::physx::graph;
class OgnPhysXImmediateComputeGeometryBounds
{
    ImmediateHashMeshes phaseHashMeshes;
    ImmediateFindDuplicates phaseFindDuplicates;
    ImmediateCookMeshes phaseCookMeshes;
    ImmediateCreateGeometries phaseCreateGeometries;

public:
    // This method might be overridden to set up initial conditions when a node type is registered, or
    // to replace initialization if the auto-generated version has some problem.
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        if (ImmediateNode::getGlobals().initialize())
        {
            auto& state =
                OgnPhysXImmediateComputeGeometryBoundsDatabase::sSharedState<OgnPhysXImmediateComputeGeometryBounds>(
                    nodeObj);
            state.phaseHashMeshes.init(ImmediateNode::getGlobals());
            state.phaseCookMeshes.init(ImmediateNode::getGlobals());
            state.phaseCreateGeometries.init(ImmediateNode::getGlobals());
        }
        else
        {
            CARB_LOG_ERROR("Cannot initialize OgnPhysXImmediateComputeGeometryBounds Globals");
        }
    }

    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        auto& state =
            OgnPhysXImmediateComputeGeometryBoundsDatabase::sSharedState<OgnPhysXImmediateComputeGeometryBounds>(
                nodeObj);
        state.phaseHashMeshes.release();
        state.phaseFindDuplicates.release();
        state.phaseCookMeshes.release();
        state.phaseCreateGeometries.release();
        ImmediateNode::getGlobals().release();
    }

    static bool compute(OgnPhysXImmediateComputeGeometryBoundsDatabase& db)
    {
        using namespace omni::graph::core::ogn;
        auto& state = db.sharedState<OgnPhysXImmediateComputeGeometryBounds>();
        const GraphContextObj& context = db.abi_context();
        IConstBundle2* inputBundle = db.inputs.primsBundle().abi_bundleInterface();

        // 1. Fill input meshes from attributes
        if (!ImmediateNode::fillMeshInputViewFromBundle(db, inputBundle, state.phaseHashMeshes.inputMeshView))
        {
            return false;
        }
        // 2. Compute Mesh Hashes (or read the one supplied in the prims)
        // 3. Find duplicate meshes and group them together
        // 4. Cook meshes (or get them from the cache)
        // 5. Create Geometries / Setup matrices (true == Compute Bounding Boxes)
        state.phaseHashMeshes.computeInitialMeshData();
        state.phaseFindDuplicates.findSharedMeshes(state.phaseHashMeshes.meshCookedData);
        state.phaseCookMeshes.cookMeshes(state.phaseHashMeshes.inputMeshView, state.phaseFindDuplicates.uniqueMeshIndices,
                                         state.phaseHashMeshes.meshCookedData);
        ImmediateNode::checkCookingWarnings(
            db, state.phaseHashMeshes.inputMeshView, state.phaseHashMeshes.meshCookedData);
        state.phaseCreateGeometries.createGeometries(
            state.phaseHashMeshes.meshCookedData, state.phaseHashMeshes.inputMeshView, true);

        const auto& inputPrims = db.inputs.primsBundle();
        auto& outputPrims = db.outputs.primsBundle();

        // Copy the input bundle to output as is
        outputPrims = inputPrims;

        IBundle2* outputBundle = outputPrims.abi_bundleInterface();
        size_t childrenCount = outputBundle->getChildBundleCount();

        // The following loop iterates all child prim bundles, and will add (or overwrite if existing)
        // bbox[Min|Max]Corner attributes with world space aabb output by PhysX. We also add the computed cooking hash
        // named meshKey, that only hashes input mesh data (excluding cooking params).

        static const NameToken point3Token = omni::fabric::asInt(pxr::TfToken("pointf[3]"));
        static const NameToken uint128Token = omni::fabric::asInt(pxr::TfToken("uint64[]"));
        static const Type point3Type = db.typeFromName(point3Token);
        static const Type uint128Type = db.typeFromName(uint128Token);
        for (size_t i = 0; i < childrenCount; ++i)
        {
            PxBounds3& bounds = state.phaseCreateGeometries.meshWorldBounds[i];
            const MeshKey meshKey = state.phaseHashMeshes.meshCookedData[i].meshHashes.meshKey;

            BundleHandle outputChild = outputBundle->getChildBundle(i);
            ogn::BundleContents<ogn::kOgnOutput, ogn::kCpu> outputChildBundle(context, outputChild);
            auto bboxMinAttr = outputChildBundle.attributeByName(ImmediateNode::kBoundingBoxMin);
            auto bboxMaxAttr = outputChildBundle.attributeByName(ImmediateNode::kBoundingBoxMax);
            auto meshKeyAttr = outputChildBundle.attributeByName(ImmediateNode::kMeshKey);
            if (!bboxMinAttr.isValid() || bboxMinAttr.type() != point3Type)
            {
                // Remove silently fails if not there and handles the case of wrong type
                outputChildBundle.removeAttribute(ImmediateNode::kBoundingBoxMin);
                bboxMinAttr = outputChildBundle.addAttribute(ImmediateNode::kBoundingBoxMin, point3Type);
            }
            if (!bboxMaxAttr.isValid() || bboxMaxAttr.type() != point3Type)
            {
                outputChildBundle.removeAttribute(ImmediateNode::kBoundingBoxMax);
                bboxMaxAttr = outputChildBundle.addAttribute(ImmediateNode::kBoundingBoxMax, point3Type);
            }
            if (!meshKeyAttr.isValid() || meshKeyAttr.type() != uint128Type)
            {
                outputChildBundle.removeAttribute(ImmediateNode::kMeshKey);
                meshKeyAttr = outputChildBundle.addAttribute(ImmediateNode::kMeshKey, uint128Type, 2);
            }
            float* bboxMin = *bboxMinAttr.getCpu<float[3]>();
            float* bboxMax = *bboxMaxAttr.getCpu<float[3]>();
            uint64_t* hash = meshKeyAttr.getCpu<uint64_t[]>()->data();

            // Output AABB corners and meshKey hash
            memcpy(bboxMin, &bounds.minimum, sizeof(bounds.minimum));
            memcpy(bboxMax, &bounds.maximum, sizeof(bounds.maximum));
            memcpy(hash, &meshKey, sizeof(uint64_t) * 2);

            // We want to make sure to remove the bounding box transform in case someone accidentally added such
            // transform by checking the "compute bounding boxes" on the "Read Prims" node that feeds into this one
            // Our "ComputeBoundingBox" node generates the bounds directly in world space, so there's no need for
            // transform
            outputChildBundle.removeAttribute(ImmediateNode::kBoundingBoxTransform);
        }
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
