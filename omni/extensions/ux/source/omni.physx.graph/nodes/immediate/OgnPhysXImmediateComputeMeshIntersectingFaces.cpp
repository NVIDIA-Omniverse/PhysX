// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <OgnPhysXImmediateComputeMeshIntersectingFacesDatabase.h>
#include <omni/fabric/FabricUSD.h>

#include <omni/graph/core/BundlePrims.h>
#include "../../plugins/immediate/ImmediateNode.h"
#include "../../plugins/immediate/ImmediateShared.h"

#include "../../plugins/immediate/ImmediateHashMeshes.h"
#include "../../plugins/immediate/ImmediateFindDuplicates.h"
#include "../../plugins/immediate/ImmediateCookMeshes.h"
#include "../../plugins/immediate/ImmediateCreateGeometries.h"
#include "../../plugins/immediate/ImmediateNarrowPhase.h"
#include "../../plugins/MultiThreader.h"
#include "../../plugins/GraphShared.h"

namespace omni
{
namespace physx
{
namespace graph
{
struct MeshIntersectionFacesResult;
struct ImmediateMeshIntersectionFaces;
} // namespace graph
} // namespace physx
} // namespace omni
using namespace ::physx;
using namespace omni::physx::graph;
using namespace omni::graph::core::ogn;

struct omni::physx::graph::MeshIntersectionFacesResult
{
    bool doesOverlap = false; // true if the two meshes actually overlap
    bool isDuplicate = false; // true if the two meshes are the same mesh in same position
    bool areBothTriangleMeshes = false; // true if both meshes are triangle meshes
    std::vector<PxGeomIndexPair> matchingTriangles; // pairs of indexes of triangles matching in mesh0 and mesh1

    struct Pair
    {
        uint32_t face0;
        uint32_t face1;
        bool operator==(const Pair& other) const
        {
            return (face0 == other.face0 && face1 == other.face1);
        }

    };
    struct PairHash
    {
        uint64_t operator()(const Pair& pair) const
        {
            return static_cast<uint64_t>(pair.face0) | (static_cast<uint64_t>(pair.face1) << 32);
        }
    };
    std::unordered_set<Pair, PairHash> pairs; // cache variable used to filter multiple matches for the same pair

    std::vector<uint32_t> intersectingFaces0;
    std::vector<uint32_t> intersectingFaces1;

    // Reset used to set the arrays size to zero without deallocating their memory
    void reset()
    {
        matchingTriangles.clear();
        pairs.clear();
        intersectingFaces0.clear();
        intersectingFaces1.clear();
        isDuplicate = false;
        doesOverlap = false;
        areBothTriangleMeshes = true;
    }
};
struct omni::physx::graph::ImmediateMeshIntersectionFaces : public ImmediateNarrowPhase
{
    ImmediateShared* shared = nullptr;
    const bool enableCoplanar = true; // If true enables finding coplanar intersecting triangles
    const bool stopAtFirstHit = false; // If true will find only the first intersecting triangle

    // Outputs
    std::vector<MeshIntersectionFacesResult> results;

    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }

    void release()
    {
        results.clear();
    }

    // This callback reports the first hit and then returns false immediately to stop testing other triangles
    class FirstHitReportCB : public PxRegularReportCallback<PxGeomIndexPair>
    {
    public:
        FirstHitReportCB() : mNb(0)
        {
        }
        virtual bool processResults(PxU32 nbItems, const PxGeomIndexPair* items)
        {
            mNb += nbItems;
            return false;
        }
        PxU32 mNb;
    };

    // This callback reports all hits and collects the intersecting pair in the supplied array reference
    class AllHitsReportCB : public PxRegularReportCallback<PxGeomIndexPair>
    {
    public:
        AllHitsReportCB(std::vector<PxGeomIndexPair>& results)
            : PxRegularReportCallback<PxGeomIndexPair>(256), mResults(results)
        {
            mResults.clear();
        }
        virtual bool processResults(PxU32 nbItems, const PxGeomIndexPair* items)
        {
            for (PxU32 i = 0; i < nbItems; i++)
                mResults.push_back(items[i]);

            return true;
        }
        std::vector<PxGeomIndexPair>& mResults;
    };

    // This function runs on a thread to process intersections in parallel
    static void threadFunction(MeshIntersectionFacesResult* PX_RESTRICT result,
                               uint32_t nbPairs,
                               const PxGeomIndexPair* PX_RESTRICT pairs,
                               const MeshGeometryData* PX_RESTRICT geometries,
                               const ImmediateMeshIntersectionFaces* self)
    {
        PX_SIMD_GUARD

        for (uint32_t i = 0; i < nbPairs; i++)
        {
            MeshIntersectionFacesResult& taskResult = result[i];
            taskResult.reset();

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
                taskResult.isDuplicate = true;
                taskResult.doesOverlap = true;
                continue;
            }
            // Extract meshes transformations
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
            taskResult.areBothTriangleMeshes =
                geom0->getType() == PxGeometryType::eTRIANGLEMESH && geom1->getType() == PxGeometryType::eTRIANGLEMESH;
            if (taskResult.areBothTriangleMeshes)
            {
                FirstHitReportCB firstHitReportCallback;
                AllHitsReportCB allHitsReportCallback(taskResult.matchingTriangles);
                PxReportCallback<PxGeomIndexPair>* callback = nullptr;
                if (self->stopAtFirstHit)
                {
                    callback = &firstHitReportCallback;
                }
                else
                {
                    callback = &allHitsReportCallback;
                }
                taskResult.doesOverlap = PxMeshQuery::findOverlapTriangleMesh(
                    *callback, geometries[id0].triangleMeshGeometry, pose0, geometries[id1].triangleMeshGeometry, pose1,
                    PxGeometryQueryFlags(0), meshMeshFlags);

                writeIntersectionFacesResult(taskResult, geometries[id0].triangleMeshGeometry.triangleMesh,
                                             geometries[id1].triangleMeshGeometry.triangleMesh,
                                             geometries[id0].meshFacesToTrianglesMapping,
                                             geometries[id1].meshFacesToTrianglesMapping);
            }
        }
    }

    void findIntersectingTriangles(const std::vector<MeshGeometryData>& meshGeometryData,
                                   std::vector<::physx::PxGeomIndexPair>& boxesOverlapPairs)
    {
        // Split the meshes to test into equally between all threads
        const uint32_t nbPairs = (uint32_t)boxesOverlapPairs.size();
        results.resize(nbPairs);
        if (!shared->singleThreaded && shared->carbTasking)
        {
            MultiThreader mt(shared->numberOfTasks, nbPairs);
            for (uint32_t taskIndex = 0; taskIndex < mt.mNbTasks; taskIndex++)
            {
                const uint32_t start = mt.mStarts[taskIndex];
                const uint32_t nbToGo = mt.mLoads[taskIndex];
                const MeshGeometryData* geometryData = meshGeometryData.data();
                const PxGeomIndexPair* pairs = boxesOverlapPairs.data() + start;
                MeshIntersectionFacesResult* taskResult = results.data() + start;

                mt.mFutures[taskIndex] = shared->carbTasking->addTask(
                    carb::tasking::Priority::eHigh, {}, [taskResult, nbToGo, pairs, geometryData, this] {
                        threadFunction(taskResult, nbToGo, pairs, geometryData, this);
                    });
            }
        }
        else
        {
            threadFunction(&results[0], nbPairs, boxesOverlapPairs.data(), meshGeometryData.data(), this);
        }
    }

    static void writeIntersectionFacesResult(MeshIntersectionFacesResult& result,
                                             const PxTriangleMesh* mesh0,
                                             const PxTriangleMesh* mesh1,
                                             const MeshFacesToTrianglesMapping* meshFacesToTrianglesMapping0,
                                             const MeshFacesToTrianglesMapping* meshFacesToTrianglesMapping1)
    {
        const size_t numTriangles = result.matchingTriangles.size();
        for (size_t triIDX = 0; triIDX < numTriangles; ++triIDX)
        {
            const uint32_t triID0 = result.matchingTriangles[triIDX].id0;
            const uint32_t triID1 = result.matchingTriangles[triIDX].id1;
            const uint32_t remappedID0 = mesh0->getTrianglesRemap()[triID0];
            const uint32_t remappedID1 = mesh1->getTrianglesRemap()[triID1];
            const uint32_t faceID0 = meshFacesToTrianglesMapping0->trianglesToFacesMapping[remappedID0];
            const uint32_t faceID1 = meshFacesToTrianglesMapping1->trianglesToFacesMapping[remappedID1];
            if(result.pairs.find({faceID0, faceID1}) == result.pairs.end()) 
            {
                result.pairs.insert({faceID0, faceID1});
            }
        }

        result.intersectingFaces0.reserve(result.pairs.size());
        result.intersectingFaces1.reserve(result.pairs.size());
        for(MeshIntersectionFacesResult::Pair pair : result.pairs)
        {
            result.intersectingFaces0.push_back(pair.face0);
            result.intersectingFaces1.push_back(pair.face1);
        }
    }
};

class OgnPhysXImmediateComputeMeshIntersectingFaces
{
    ImmediateHashMeshes phaseHashMeshes;
    ImmediateFindDuplicates phaseFindDuplicates;
    ImmediateCookMeshes phaseCookMeshes;
    ImmediateCreateGeometries phaseCreateGeometries;
    ImmediateMeshIntersectionFaces phaseFindIntersectingTriangles;
    std::vector<physx::PxGeomIndexPair> boxesOverlapPairs;
    bool errorPrinted = false;
    bool warningPrinted = false;

public:
    // This method might be overridden to set up initial conditions when a node type is registered, or
    // to replace initialization if the auto-generated version has some problem.
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        if (ImmediateNode::getGlobals().initialize())
        {
            auto& state = OgnPhysXImmediateComputeMeshIntersectingFacesDatabase::sSharedState<
                OgnPhysXImmediateComputeMeshIntersectingFaces>(nodeObj);
            state.phaseHashMeshes.init(ImmediateNode::getGlobals());
            state.phaseCookMeshes.init(ImmediateNode::getGlobals());
            state.phaseCreateGeometries.init(ImmediateNode::getGlobals());
            state.phaseFindIntersectingTriangles.init(ImmediateNode::getGlobals());
        }
        else
        {
            CARB_LOG_ERROR("Cannot initialize OgnPhysXImmediateComputeMeshIntersectingFaces Globals");
        }
    }

    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        auto& state =
            OgnPhysXImmediateComputeMeshIntersectingFacesDatabase::sSharedState<OgnPhysXImmediateComputeMeshIntersectingFaces>(
                nodeObj);
        state.phaseHashMeshes.release();
        state.phaseFindDuplicates.release();
        state.phaseCookMeshes.release();
        state.phaseCreateGeometries.release();
        state.phaseFindIntersectingTriangles.release();
        state.boxesOverlapPairs.clear();
        ImmediateNode::getGlobals().release();
    }

    static bool compute(OgnPhysXImmediateComputeMeshIntersectingFacesDatabase& db)
    {
        auto& state = db.sharedState<OgnPhysXImmediateComputeMeshIntersectingFaces>();
        const GraphContextObj& context = db.abi_context();
        IConstBundle2* inputBundle = db.inputs.primsBundle().abi_bundleInterface();
        if (db.inputs.overlapsPair1.size() != db.inputs.overlapsPair0.size())
        {
            db.logError("overlapsPair0 and overlapPair1 arrays must have the same length");
            return false;
        }

        const size_t numOverlaps = db.inputs.overlapsPair0.size();

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
        state.phaseFindIntersectingTriangles.findIntersectingTriangles(
            state.phaseCreateGeometries.meshGeometryData, state.boxesOverlapPairs);

        // Output stuff
        db.outputs.overlaps.resize(numOverlaps);

        state.errorPrinted = false;
        state.warningPrinted = false;

        std::vector<NameToken> newNames(numOverlaps);
        omni::physx::graph::createPrimName(context.iToken->getHandle("prim"), 0, { newNames.data(), newNames.size() });

        IBundle2* facesOutputBundle = db.outputs.faceIndices().abi_bundleInterface();
        facesOutputBundle->clearContents();
        std::vector<omni::graph::core::BundleHandle> childBundles(numOverlaps);

        db.outputs.faceIndices().abi_bundleInterface()->createChildBundles(
            newNames.data(), newNames.size(), childBundles.data());
        const auto& bundleContext = db.outputs.faceIndices().abi_bundleInterface()->getContext();
        auto& overlaps = db.outputs.overlaps();
        static NameToken const kFaces0 = omni::fabric::asInt(pxr::TfToken("faces0", pxr::TfToken::Immortal));
        static NameToken const kFaces1 = omni::fabric::asInt(pxr::TfToken("faces1", pxr::TfToken::Immortal));
        for (size_t idx = 0; idx < numOverlaps; ++idx)
        {
            const MeshIntersectionFacesResult& result = state.phaseFindIntersectingTriangles.results[idx];
            overlaps[idx] = result.doesOverlap;
            if (!result.areBothTriangleMeshes && !state.errorPrinted)
            {
                NameToken body0 = db.inputs.overlapsPair0()[idx];
                NameToken body1 = db.inputs.overlapsPair1()[idx];
                state.errorPrinted = true;
                db.logError(
                    "Prim '%s' and '%s' are not both TriangleMeshes, triangle intersection overlap cannot be computed",
                    db.tokenToString(body0), db.tokenToString(body1));
            }
            else if (result.isDuplicate && !state.warningPrinted)
            {
                NameToken body0 = db.inputs.overlapsPair0()[idx];
                NameToken body1 = db.inputs.overlapsPair1()[idx];
                state.warningPrinted = true;
                db.logWarning("Prim '%s' and '%s' are the same mesh in the same position", db.tokenToString(body0),
                              db.tokenToString(body1));
            }
            BundleHandle childHandle = childBundles[idx];

            ogn::BundleContents<ogn::kOgnOutput, ogn::kCpu> outputChildBundle(bundleContext, childHandle);
            ogn::RuntimeAttribute<ogn::kOgnOutput, ogn::kCpu> sourcePrimPathAttribute = outputChildBundle.addAttribute(
                ImmediateNode::kSourcePrimPathToken, Type(BaseDataType::eToken, 1, 0, AttributeRole::eNone));
            *sourcePrimPathAttribute.getCpu<NameToken>() = newNames[idx];

            state.writeBundleIndicesArray(db, kFaces0, outputChildBundle, result.intersectingFaces0);
            state.writeBundleIndicesArray(db, kFaces1, outputChildBundle, result.intersectingFaces1);
        }

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    void writeBundleIndicesArray(omni::graph::core::ogn::OmniGraphDatabase& db,
                                 NameToken facesAttributeName,
                                 ogn::BundleContents<ogn::kOgnOutput, ogn::kCpu>& outputChildBundle,
                                 const std::vector<uint32_t>& facesIndicesVector)
    {
        ogn::RuntimeAttribute<ogn::kOgnOutput, ogn::kCpu> faceIndicesAttribute =
            outputChildBundle.addAttribute(facesAttributeName, Type(BaseDataType::eInt, 1, 1, AttributeRole::eNone));
        auto facesIndices = faceIndicesAttribute.getCpu<int32_t[]>();
        facesIndices->resize(facesIndicesVector.size());
        uint32_t idx = 0;
        auto faceData = facesIndices->data();
        for (auto face : facesIndicesVector)
        {
            faceData[idx] = face;
            idx++;
        }
    }
};

REGISTER_OGN_NODE()
