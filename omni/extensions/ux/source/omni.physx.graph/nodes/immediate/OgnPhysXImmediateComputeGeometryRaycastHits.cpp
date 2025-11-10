// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <OgnPhysXImmediateComputeGeometryRaycastHitsDatabase.h>
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


struct ImmediateBoundsRaycastBVH;

struct RaycastResult;
struct ImmediateFindRaycastHits;

} // namespace graph
} // namespace physx
} // namespace omni

using namespace ::physx;
using namespace omni::physx::graph;

struct omni::physx::graph::ImmediateBoundsRaycastBVH
{
    std::vector<size_t> raycastHits;
    ::physx::PxBVH* mBVH = nullptr;
    PxGeometryQueryFlags queryFlags = PxGeometryQueryFlag::eDEFAULT;
    std::vector<::physx::PxBounds3> meshWorldBounds;
    std::vector<size_t> meshIndices;

    void buildBVH()
    {
        PxBVHDesc bvhDesc;
        bvhDesc.bounds.count = (PxU32)meshWorldBounds.size();
        bvhDesc.bounds.data = meshWorldBounds.data();
        bvhDesc.bounds.stride = sizeof(PxBounds3);
        bvhDesc.enlargement = 0.0f;
        mBVH = PxCreateBVH(bvhDesc);
    }

    void release()
    {
        if(mBVH)
        {
            mBVH->release();
            mBVH = nullptr;
        }
        raycastHits.clear();
        meshWorldBounds.clear();
        meshIndices.clear();
    }

    void broadPhase(const PxVec3& origin, const PxVec3& direction, const PxReal& range)
    {
        struct LocalRaycastCallback : PxBVH::RaycastCallback
        {
            LocalRaycastCallback() : mCD(nullptr)
            {
            }

            void init(ImmediateBoundsRaycastBVH* CD)
            {
                mCD = CD;
            }

            virtual bool reportHit(PxU32 boundsIndex, PxReal& distance) override
            {
                mCD->raycastHits.push_back(mCD->meshIndices[boundsIndex]);
                return true;
            }

            ImmediateBoundsRaycastBVH* mCD;
        };
        
        buildBVH();
        {
            raycastHits.clear();
            LocalRaycastCallback cb;
            cb.init(this);
            mBVH->raycast(origin, direction, range, cb, PxGeometryQueryFlags(0));
        }
        mBVH->release();
        mBVH = nullptr;
    }
};

constexpr PxU32 k_nMaxHitsPerBody = 8;

struct omni::physx::graph::RaycastResult
{
    size_t index = 0; // Index in the mesh data arrays.
    PxU32 numHits = 0;
    PxGeomRaycastHit hits[k_nMaxHitsPerBody];
};

struct omni::physx::graph::ImmediateFindRaycastHits : public ImmediateNarrowPhase
{
    ImmediateShared* shared = nullptr;
    PxVec3 origin = PxVec3(0.0f, 0.0f, 0.0f);
    PxVec3 direction = PxVec3(0.0f, 0.0f, -1.0f);
    PxReal range = PX_MAX_F32;
    bool bBothSides = false;
    PxGeometryQueryFlags queryFlags = PxGeometryQueryFlag::eDEFAULT;
    std::vector<size_t> meshIndices;

    // Outputs
    std::vector<RaycastResult> raycastHits;
    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }

    void release()
    {
        raycastHits.clear();
        meshIndices.clear();
    }

    void threadFunction(RaycastResult* PX_RESTRICT result,
                        uint32_t nbMeshes,
                        const size_t* meshIndicesData,
                        const MeshGeometryData* PX_RESTRICT geometries,
                        const ImmediateFindRaycastHits* self)
    {
        PX_SIMD_GUARD

        PxHitFlags hitFlags = PxHitFlag::eDEFAULT | PxHitFlag::eMESH_MULTIPLE;
        
        if(bBothSides) hitFlags |= PxHitFlag::eMESH_BOTH_SIDES;

        for (size_t i = 0; i < nbMeshes; i++)
        {
            size_t index = *(meshIndicesData + i); 

            if (!geometries[index].isValid)
                continue; // Invalid geometry, probably failed cooking

            PxTransform pose = geometries[index].worldTransform;

            const PxGeometry* geom = &geometries[index].triangleMeshGeometry;
            if (geometries[index].convexMeshGeometry.convexMesh)
            {
                geom = &geometries[index].convexMeshGeometry;
            }

            result[i].index = index;
            result[i].numHits = PxGeometryQuery::raycast(origin, direction, *geom, pose, range, hitFlags, k_nMaxHitsPerBody, result[i].hits);
        }
    }

    void findRaycastHits(const std::vector<MeshGeometryData>& meshGeometryData)
    {
        const size_t nbMeshes = meshIndices.size();
        raycastHits.clear();
        raycastHits.resize(nbMeshes);
        if (!shared->singleThreaded && shared->carbTasking)
        {
            MultiThreader mt(shared->numberOfTasks, (uint32_t)nbMeshes);
            for (uint32_t i = 0; i < mt.mNbTasks; i++)
            {
                const uint32_t start = mt.mStarts[i];
                const uint32_t nbToGo = mt.mLoads[i];
                const MeshGeometryData* geometryData = meshGeometryData.data();
                const size_t* indicesOffset = meshIndices.data() + start;
                RaycastResult* PX_RESTRICT raycastResults = raycastHits.data() + start;

                mt.mFutures[i] = shared->carbTasking->addTask(
                    carb::tasking::Priority::eHigh, {}, [raycastResults, nbToGo, indicesOffset, geometryData, this] {
                        threadFunction(raycastResults, nbToGo, indicesOffset, geometryData, this);
                    });
            }
        }
        else
        {
            threadFunction(raycastHits.data(), (uint32_t)nbMeshes,  meshIndices.data(), meshGeometryData.data(), this);
        }
    }
};

class OgnPhysXImmediateComputeGeometryRaycastHits
{
    ImmediateBoundsRaycastBVH immediateBVH;
    ImmediateHashMeshes phaseHashMeshes;
    ImmediateFindDuplicates phaseFindDuplicates;
    ImmediateCookMeshes phaseCookMeshes;
    ImmediateCreateGeometries phaseCreateGeometries;
    ImmediateFindRaycastHits phaseFindRaycastHits;

public:
    // This method might be overridden to set up initial conditions when a node type is registered, or
    // to replace initialization if the auto-generated version has some problem.
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        if (ImmediateNode::getGlobals().initialize())
        {
            auto& state =
                OgnPhysXImmediateComputeGeometryRaycastHitsDatabase::sSharedState<OgnPhysXImmediateComputeGeometryRaycastHits>(
                    nodeObj);
            state.phaseHashMeshes.init(ImmediateNode::getGlobals());
            state.phaseCookMeshes.init(ImmediateNode::getGlobals());
            state.phaseCreateGeometries.init(ImmediateNode::getGlobals());
            state.phaseFindRaycastHits.init(ImmediateNode::getGlobals());
        }
        else
        {
            CARB_LOG_ERROR("Cannot initialize OgnPhysXImmediateComputeGeometryRaycastHits Globals");
        }
    }

    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        auto& state =
            OgnPhysXImmediateComputeGeometryRaycastHitsDatabase::sSharedState<OgnPhysXImmediateComputeGeometryRaycastHits>(
                nodeObj);
        state.phaseHashMeshes.release();
        state.phaseFindDuplicates.release();
        state.phaseCookMeshes.release();
        state.phaseCreateGeometries.release();
        state.phaseFindRaycastHits.release();
        state.immediateBVH.release();
        ImmediateNode::getGlobals().release();
    }

    static bool compute(OgnPhysXImmediateComputeGeometryRaycastHitsDatabase& db)
    {
        using namespace omni::graph::core::ogn;
        auto& state = db.sharedState<OgnPhysXImmediateComputeGeometryRaycastHits>();
        const GraphContextObj& context = db.abi_context();
        {
            const auto& input = db.inputs.origin();
            if(!input.resolved())
            {
                db.logError("No origin input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                state.phaseFindRaycastHits.origin = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                state.phaseFindRaycastHits.origin = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for origin.");
                return false;
            }
        }

        {
            const auto& input = db.inputs.direction();
            if(!input.resolved())
            {
                db.logError("No direction input supplied to node.");
                return false;
            }
            else if(auto float3Input = input.get<float[3]>())
            {
                state.phaseFindRaycastHits.direction = { float3Input[0], float3Input[1], float3Input[2]};
            }
            else if(auto double3Input = input.get<double[3]>())
            {
                state.phaseFindRaycastHits.direction = { (float) double3Input[0], (float) double3Input[1], (float) double3Input[2]};
            }
            else
            {
                db.logError("Invalid data type input for direction.");
                return false;
            }
        }

        {
            const auto& input = db.inputs.raycastRange();
            if(!input.resolved())
            {
                state.phaseFindRaycastHits.range = PX_MAX_F32;
            }
            else if(auto floatInput = input.get<float>())
            {
                state.phaseFindRaycastHits.range = *floatInput;
            }
            else if(auto doubleInput = input.get<double>())
            {
                state.phaseFindRaycastHits.range = (float) *doubleInput;
            }
            else
            {
                db.logError("Invalid data type input for range.");
                return false;
            }
        }

        if(db.inputs.prims.size() > 0)
        {
            if (!ImmediateNode::fillMeshInputViewFromTargets(
                    db, db.inputs.prims, state.phaseHashMeshes.inputMeshView))
            {
                db.logError("Failed to fill mesh input view from targets.");
                return false;
            }
        }
        else if(db.inputs.primsBundle().isValid())
        {
            IConstBundle2* inputBundle = db.inputs.primsBundle().abi_bundleInterface();
            size_t childrenCount = inputBundle->getChildBundleCount();
            if (childrenCount < 1)
            {
                return false;
            }

            if (!ImmediateNode::fillMeshInputViewFromBundle(
                    db, inputBundle, state.phaseHashMeshes.inputMeshView))
            {
                db.logError("Failed to fill mesh input view from bundle.");
                return false;
            }
        }
        else
        {
            // No inputs.
            return false;
        }

        state.phaseHashMeshes.computeInitialMeshData();
        state.phaseFindDuplicates.findSharedMeshes(state.phaseHashMeshes.meshCookedData);
        state.phaseCookMeshes.cookMeshes(state.phaseHashMeshes.inputMeshView, state.phaseFindDuplicates.uniqueMeshIndices,
                                         state.phaseHashMeshes.meshCookedData);
        ImmediateNode::checkCookingWarnings(
            db, state.phaseHashMeshes.inputMeshView, state.phaseHashMeshes.meshCookedData);
        state.phaseCreateGeometries.createGeometries(
            state.phaseHashMeshes.meshCookedData, state.phaseHashMeshes.inputMeshView, false);

        // For input prims that have bounds provided, we use these for an initial broadphase filter. If no bounds are provided,
        // we pass the prim directly to the narrowphase.
        state.phaseFindRaycastHits.meshIndices.clear();
        state.immediateBVH.meshWorldBounds.clear();
        state.immediateBVH.meshIndices.clear();

        if(db.inputs.prims.size() > 0)
        {
            const long stageId = context.iContext->getStageId(context);
            UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
            for(size_t n = 0; n < state.phaseHashMeshes.inputMeshView.size(); ++n)
            {
                const TargetPath& target = state.phaseHashMeshes.inputMeshView[n].meshView.primId;
                const omni::fabric::PathC pathC(state.phaseHashMeshes.inputMeshView[n].meshView.primId);
                const UsdPrim targetPrim = stage->GetPrimAtPath(toSdfPath(pathC));
                const UsdGeomMesh primGeom(targetPrim);

                UsdAttribute extentAttrib = primGeom.GetExtentAttr();
                if(extentAttrib.IsValid() && extentAttrib.HasValue())
                {
                    pxr::VtArray<pxr::GfVec3f> extent;
                    extentAttrib.Get(&extent);

                    pxr::GfBBox3d bbox = GfBBox3d(GfRange3d(extent[0], extent[1]), state.phaseCreateGeometries.meshGeometryData[n].worldMatrix);
                    pxr::GfRange3d range = bbox.ComputeAlignedRange();

                    state.immediateBVH.meshWorldBounds.push_back(PxBounds3(omni::physx::toPhysX(range.GetMin()), omni::physx::toPhysX(range.GetMax())));
                    state.immediateBVH.meshIndices.push_back(n);
                }
                else
                {
                    // Pass the index directly to the narrowphase list.
                    state.phaseFindRaycastHits.meshIndices.push_back(n);
                }
            }
        }
        else
        {
            // Sanity check only. If this is not the case, we should not arrive here.
            CARB_ASSERT(db.inputs.primsBundle().isValid());

            IConstBundle2* inputBundle = db.inputs.primsBundle().abi_bundleInterface();
            size_t childrenCount = inputBundle->getChildBundleCount();

            for (size_t n = 0; n < childrenCount; ++n)
            {
                ConstBundleHandle child = inputBundle->getConstChildBundle(n);
                ogn::BundleContents<ogn::kOgnInput, ogn::kCpu> extractedBundle(context, child);
                const auto boxMinAttr = extractedBundle.attributeByName(ImmediateNode::kBoundingBoxMin);
                const auto boxMaxAttr = extractedBundle.attributeByName(ImmediateNode::kBoundingBoxMax);
                if (!boxMinAttr.isValid() || !boxMaxAttr.isValid())
                {
                    // Pass the index directly to the narrowphase list.
                    state.phaseFindRaycastHits.meshIndices.push_back(n);
                    continue;
                }
                PxBounds3 bounds;
                if (ImmediateShared::parseBoundsFromAttributes(bounds, boxMinAttr, boxMaxAttr))
                {
                    // Optional attributes, if box transform is there, we will apply it to the incoming bounds.
                    // This is generated by the "Read Prims" node with "Compute Bounding Box" checked to true.
                    // Our "ComputeBoundingBox" node generates the bounds directly in world space
                    const auto boxTransformAttr = extractedBundle.attributeByName(ImmediateNode::kBoundingBoxTransform);
                    const auto worldMatrixAttr = extractedBundle.attributeByName(ImmediateNode::kWorldMatrixToken);
                    if (boxTransformAttr.isValid() && worldMatrixAttr.isValid())
                    {
                        if (!ImmediateShared::parseAndTransformBoundingBoxFromAttribute(bounds, worldMatrixAttr))
                        {
                            const auto primPathAttribute = extractedBundle.attributeByName(ImmediateNode::kSourcePrimPathToken);
                            Token primPath = *primPathAttribute.getCpu<Token>();
                            db.logError(
                                "Prim '%s' needs both 'bboxTransform' and 'worldMatrix' attributes convertible to float[16] or double[16]",
                                db.tokenToString(primPath));
                            continue;
                        }
                    }
                    state.immediateBVH.meshWorldBounds.push_back(bounds);
                    state.immediateBVH.meshIndices.push_back(n);
                }
                else
                {
                    const auto primPathAttribute = extractedBundle.attributeByName(ImmediateNode::kSourcePrimPathToken);
                    Token primPath = *primPathAttribute.getCpu<Token>();
                    db.logError(
                        "Prim '%s' 'bboxMinCorner' and 'bboxMaxCorner' attributes must be convertible to float[3] or double[3]",
                        db.tokenToString(primPath));
                    continue;
                }
            }
        }

        // If bounds have been provided, do a broadphase check to quickly filter out prims.
        if(state.immediateBVH.meshWorldBounds.size() > 0 )
        {
            state.immediateBVH.broadPhase(state.phaseFindRaycastHits.origin, state.phaseFindRaycastHits.direction, state.phaseFindRaycastHits.range);

            for(size_t n = 0; n < state.immediateBVH.raycastHits.size(); n++)
            {
                state.phaseFindRaycastHits.meshIndices.push_back(state.immediateBVH.raycastHits[n]); 
            }
        }

        state.phaseFindRaycastHits.bBothSides = db.inputs.bothSides();

        // Execute the actual intersection computation.
        state.phaseFindRaycastHits.findRaycastHits(state.phaseCreateGeometries.meshGeometryData);
        PxU32 numHits = 0;
        for(const RaycastResult& raycastHit : state.phaseFindRaycastHits.raycastHits)
        {
            numHits += raycastHit.numHits;
        }
        // Write the outputs.
        db.outputs.primsHit().resize(numHits);
        db.outputs.positions().resize(numHits);
        db.outputs.normals().resize(numHits);
        db.outputs.distances().resize(numHits);
        db.outputs.faceIndexes().resize(numHits);

        size_t n = 0;
        for(const RaycastResult& raycastHit : state.phaseFindRaycastHits.raycastHits)
        {
            for(size_t hit = 0; hit < raycastHit.numHits; hit++)
            {
                db.outputs.primsHit()[n] = state.phaseHashMeshes.inputMeshView[raycastHit.index].meshView.primId;
                db.outputs.positions()[n] = raycastHit.hits[hit].position;
                db.outputs.normals()[n] = raycastHit.hits[hit].normal;
                db.outputs.distances()[n] = raycastHit.hits[hit].distance;
                db.outputs.faceIndexes()[n] = raycastHit.hits[hit].faceIndex;
                n++;
            }
        }

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
