// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <OgnPhysXImmediateComputeGeometryOverlapsDatabase.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/graph/core/BundlePrims.h>
#include "../../plugins/immediate/ImmediateShared.h"
#include "../../plugins/immediate/ImmediateNode.h"

#include "../../plugins/immediate/ImmediateHashMeshes.h"
#include "../../plugins/immediate/ImmediateFindDuplicates.h"
#include "../../plugins/immediate/ImmediateCookMeshes.h"
#include "../../plugins/immediate/ImmediateCreateGeometries.h"
#include "../../plugins/immediate/ImmediateNarrowPhase.h"
#include "../../plugins/MultiThreader.h"


namespace omni
{
namespace physx
{
namespace graph
{
struct MeshOverlapResult;
struct ImmediateFindOverlaps;

} // namespace graph
} // namespace physx
} // namespace omni

using namespace ::physx;
using namespace omni::physx::graph;

struct omni::physx::graph::MeshOverlapResult
{
    bool doesOverlap = false;
    bool isDuplicate = false;
    uint32_t id0 = 0;
    uint32_t id1 = 0;
};
struct omni::physx::graph::ImmediateFindOverlaps : public ImmediateNarrowPhase
{
    ImmediateShared* shared = nullptr;

    // Outputs
    std::vector<MeshOverlapResult> results;

    const bool enableCoplanar = true;

    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }
    void release()
    {
        results.clear();
    }

    void threadFunction(MeshOverlapResult* PX_RESTRICT result,
                        uint32_t nbPairs,
                        const PxGeomIndexPair* PX_RESTRICT pairs,
                        const MeshGeometryData* PX_RESTRICT geometries,
                        const ImmediateFindOverlaps* self)
    {
        PX_SIMD_GUARD

        uint32_t nbOverlaps = 0;
        for (uint32_t i = 0; i < nbPairs; i++)
        {
            const PxGeomIndexPair& p = pairs[i];
            const uint32_t id0 = p.id0;
            const uint32_t id1 = p.id1;
            if (!geometries[id0].isValid || !geometries[id1].isValid)
                continue; // Invalid geometry, probably failed cooking

            if (sameGeometryInSamePosition(geometries[id0], geometries[id1]))
            {
                // Same mesh in the same pose, probably a modeling error. Don't bother running an
                // overlap query in this case, it's going to be very slow and we already know the
                // results.
                nbOverlaps++;
                result->id0 = id0;
                result->id1 = id1;
                result->doesOverlap = true;
                result->isDuplicate = true;
                result++;
                continue;
            }

            const pxr::GfVec3d trans0 = geometries[id0].worldMatrix.ExtractTranslation();
            const pxr::GfVec3d trans1 = geometries[id1].worldMatrix.ExtractTranslation();

            PxTransform pose0 = geometries[id0].worldTransform;
            PxTransform pose1 = geometries[id1].worldTransform;
            pose0.p = PxVec3(0.0f);
            pose1.p = PxVec3(float(trans1[0] - trans0[0]), float(trans1[1] - trans0[1]), float(trans1[2] - trans0[2]));

            const PxMeshMeshQueryFlags meshMeshFlags =
                self->enableCoplanar ? PxMeshMeshQueryFlag::eDEFAULT : PxMeshMeshQueryFlag::eDISCARD_COPLANAR;


            const PxGeometry* geom0 = &geometries[id0].triangleMeshGeometry;
            if (geometries[id0].convexMeshGeometry.convexMesh)
            {
                geom0 = &geometries[id0].convexMeshGeometry;
            }
            const PxGeometry* geom1 = &geometries[id1].triangleMeshGeometry;
            if (geometries[id1].convexMeshGeometry.convexMesh)
            {
                geom1 = &geometries[id1].convexMeshGeometry;
            }

            if (PxGeometryQuery::overlap(*geom0, pose0, *geom1, pose1, PxGeometryQueryFlag::eDEFAULT))
            {
                nbOverlaps++;
                result->doesOverlap = true;
                result->id0 = id0;
                result->id1 = id1;
            }

            result++;
        }
    }

    void findOverlaps(const std::vector<MeshGeometryData>& meshGeometryData,
                      std::vector<::physx::PxGeomIndexPair>& boxesOverlapPairs)
    {


        const uint32_t nbPairs = (uint32_t)boxesOverlapPairs.size();
        results.clear();
        results.resize(nbPairs);
        if (!shared->singleThreaded && shared->carbTasking)
        {
            MultiThreader mt(shared->numberOfTasks, nbPairs);
            for (uint32_t i = 0; i < mt.mNbTasks; i++)
            {
                const uint32_t start = mt.mStarts[i];
                const uint32_t nbToGo = mt.mLoads[i];
                const MeshGeometryData* geometryData = meshGeometryData.data();
                const PxGeomIndexPair* pairs = boxesOverlapPairs.data() + start;
                MeshOverlapResult* narrowResult = results.data() + start;

                mt.mFutures[i] = shared->carbTasking->addTask(
                    carb::tasking::Priority::eHigh, {}, [narrowResult, nbToGo, pairs, geometryData, this] {
                        threadFunction(narrowResult, nbToGo, pairs, geometryData, this);
                    });
            }
        }
        else
        {
            threadFunction(results.data(), nbPairs, boxesOverlapPairs.data(), meshGeometryData.data(), this);
        }
    }
};

class OgnPhysXImmediateComputeGeometryOverlaps
{
    ImmediateHashMeshes phaseHashMeshes;
    ImmediateFindDuplicates phaseFindDuplicates;
    ImmediateCookMeshes phaseCookMeshes;
    ImmediateCreateGeometries phaseCreateGeometries;
    ImmediateFindOverlaps phaseFindOverlaps;
    std::vector<physx::PxGeomIndexPair> boxesOverlapPairs;

public:
    // This method might be overridden to set up initial conditions when a node type is registered, or
    // to replace initialization if the auto-generated version has some problem.
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        if (ImmediateNode::getGlobals().initialize())
        {
            auto& state =
                OgnPhysXImmediateComputeGeometryOverlapsDatabase::sSharedState<OgnPhysXImmediateComputeGeometryOverlaps>(
                    nodeObj);
            state.phaseHashMeshes.init(ImmediateNode::getGlobals());
            state.phaseCookMeshes.init(ImmediateNode::getGlobals());
            state.phaseCreateGeometries.init(ImmediateNode::getGlobals());
            state.phaseFindOverlaps.init(ImmediateNode::getGlobals());
        }
        else
        {
            CARB_LOG_ERROR("Cannot initialize OgnPhysXImmediateComputeGeometryOverlaps Globals");
        }
    }

    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        auto& state =
            OgnPhysXImmediateComputeGeometryOverlapsDatabase::sSharedState<OgnPhysXImmediateComputeGeometryOverlaps>(
                nodeObj);
        state.phaseHashMeshes.release();
        state.phaseFindDuplicates.release();
        state.phaseCookMeshes.release();
        state.phaseCreateGeometries.release();
        state.phaseFindOverlaps.release();
        state.boxesOverlapPairs.clear();
        ImmediateNode::getGlobals().release();
    }

    static bool compute(OgnPhysXImmediateComputeGeometryOverlapsDatabase& db)
    {
        using namespace omni::graph::core::ogn;
        auto& state = db.sharedState<OgnPhysXImmediateComputeGeometryOverlaps>();
        const GraphContextObj& context = db.abi_context();
        IConstBundle2* inputBundle = db.inputs.primsBundle().abi_bundleInterface();

        auto childrenCount = inputBundle->getChildBundleCount();

        // 1. Fill input meshes from attributes
        // Name to indices map tracks the index of a given path from the input prims lists into the flat mesh array
        std::unordered_map<NameToken, size_t> namesToIndices;
        if (!ImmediateNode::fillMeshInputViewFromBundle(
                db, inputBundle, state.phaseHashMeshes.inputMeshView, &namesToIndices))
        {
            return false;
        }

        // 2. Setup overlap pairs
        if (!ImmediateNode::fillBoxOverlapPairsVector(
                db, db.inputs.overlapsPair0, db.inputs.overlapsPair1, state.boxesOverlapPairs, namesToIndices))
        {
            return false;
        }
        // 3. Compute Mesh Hashes (or read the one supplied in the prims)
        // 4. Find duplicate meshes and group them together
        // 5. Cook meshes (or get them from the cache)
        // 6. Setup matrices / Create Geometries (but false == skip computing Bounding Boxes)
        state.phaseHashMeshes.computeInitialMeshData();
        state.phaseFindDuplicates.findSharedMeshes(state.phaseHashMeshes.meshCookedData);
        state.phaseCookMeshes.cookMeshes(state.phaseHashMeshes.inputMeshView, state.phaseFindDuplicates.uniqueMeshIndices,
                                         state.phaseHashMeshes.meshCookedData);
        ImmediateNode::checkCookingWarnings(
            db, state.phaseHashMeshes.inputMeshView, state.phaseHashMeshes.meshCookedData);
        state.phaseCreateGeometries.createGeometries(
            state.phaseHashMeshes.meshCookedData, state.phaseHashMeshes.inputMeshView, false);

        // 7. Execute the actual intersection computation
        state.phaseFindOverlaps.findOverlaps(state.phaseCreateGeometries.meshGeometryData, state.boxesOverlapPairs);

        // Output results
        bool errorPrinted = false;
        const size_t numOverlaps = state.phaseFindOverlaps.results.size();
        db.outputs.overlaps.resize(numOverlaps);
        for (size_t idx = 0; idx < numOverlaps; ++idx)
        {
            const MeshOverlapResult& result = state.phaseFindOverlaps.results[idx];
            if (result.isDuplicate && !errorPrinted)
            {
                errorPrinted = true;
                NameToken body0 = db.inputs.overlapsPair0()[idx];
                NameToken body1 = db.inputs.overlapsPair1()[idx];
                db.logWarning("Prim '%s' and '%s' are the same mesh in the same position", db.tokenToString(body0),
                              db.tokenToString(body1));
            }
            db.outputs.overlaps()[idx] = result.doesOverlap;
        }

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
