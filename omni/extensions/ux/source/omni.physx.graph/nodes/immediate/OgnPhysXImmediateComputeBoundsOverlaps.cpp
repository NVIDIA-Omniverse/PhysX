// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <OgnPhysXImmediateComputeBoundsOverlapsDatabase.h>
#include <omni/fabric/FabricUSD.h>
#include <omni/graph/core/BundlePrims.h>
#include "../../plugins/immediate/ImmediateNode.h" // mainly for the tokens
#include "../../plugins/immediate/ImmediateShared.h"

#include <PxPhysicsAPI.h>
#include <common/foundation/TypeCast.h>


namespace omni
{
namespace physx
{
namespace graph
{
struct ImmediateBVH;

}
} // namespace physx
} // namespace omni

using namespace ::physx;
using namespace omni::physx::graph;
struct omni::physx::graph::ImmediateBVH
{
    std::vector<::physx::PxBounds3> meshWorldBounds;
    std::vector<::physx::PxGeomIndexPair> boxesOverlapPairs;
    std::vector<uint64_t> mInputMeshPaths;
    ::physx::PxBVH* mBVH = nullptr;
    bool enableFiltering = false;

    void buildBVH()
    {
        boxesOverlapPairs.clear();
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
    }

    void broadPhase()
    {
        struct LocalOverlapCallback : PxBVH::OverlapCallback
        {
            LocalOverlapCallback() : mCD(nullptr), mCurrentIndex(0xffffffff)
            {
            }

            void init(ImmediateBVH* CD)
            {
                mCD = CD;
            }

            virtual bool reportHit(PxU32 boundsIndex) override
            {
                // PT: because this isn't a proper broadphase, these calls will report self-overlaps and duplicate
                // overlaps from previous objects. So we need to do some filtering
                if (boundsIndex > mCurrentIndex)
                {
                    bool addPair = true;
                    if (mCD->enableFiltering)
                    {
                        auto root0 = mCD->mInputMeshPaths[boundsIndex];
                        if (root0 == mRoot1)
                            addPair = false;
                    }
                    if (addPair)
                        mCD->boxesOverlapPairs.push_back(PxGeomIndexPair(mCurrentIndex, boundsIndex));
                }
                return true;
            }

            ImmediateBVH* mCD;
            uint32_t mCurrentIndex;
            uint64_t mRoot1;
        };

        const uint32_t nbToGo = (uint32_t)mInputMeshPaths.size();
        {
            LocalOverlapCallback cb;
            cb.init(this);
            for (uint32_t i = 0; i < nbToGo; i++)
            {
                cb.mCurrentIndex = i;
                if (enableFiltering)
                    cb.mRoot1 = mInputMeshPaths[i];
                mBVH->overlap(PxBoxGeometry(meshWorldBounds[i].getExtents()),
                              PxTransform(meshWorldBounds[i].getCenter()), cb, PxGeometryQueryFlags(0));
            }
        }
    }
};

// This node finds the overlapping pair of bounding boxes by building a BVH in PhysX and querying it
class OgnPhysXImmediateComputeBoundsOverlaps
{
    ImmediateBVH immediateBVH;

public:
    // In the initialize we create a new detection object, sharing the globals with all of our brother nodes
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        if (!ImmediateNode::getGlobals().initialize())
        {
            CARB_LOG_ERROR("Cannot initialize OgnPhysXImmediateComputeBoundsOverlaps Globals");
        }
    }

    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        ImmediateNode::getGlobals().release();
    }

    static bool compute(OgnPhysXImmediateComputeBoundsOverlapsDatabase& db)
    {
        using namespace omni::graph::core::ogn;
        // We're not really holding state, but we keep the detection object there to reuse allocations over invocations
        auto& state = db.sharedState<OgnPhysXImmediateComputeBoundsOverlaps>();
        const GraphContextObj& context = db.abi_context();
        IConstBundle2* inputBundle = db.inputs.primsBundle().abi_bundleInterface();

        // We iterate all children bundles of the current input bundle, checking if they're all valid Prim bundles,
        // containing the attributes of interest ('sourcePrimPath', 'bboxMinCorner', 'bboxMaxCorner')
        // We optionally also check if a ' bboxTransform' exists (as output by the 'Read Prims' node) and optionally
        // apply it to bring the bounding box in world space.
        // Note: Bounding boxes output by the 'Compute Bounding Boxes' Node are output in world space
        // while bounding boxes output by 'Read prims' node are in local space and they specify a separate world
        // transform
        size_t childrenCount = inputBundle->getChildBundleCount();
        state.immediateBVH.meshWorldBounds.resize(childrenCount);
        state.immediateBVH.mInputMeshPaths.resize(childrenCount);
        for (size_t i = 0; i < childrenCount; ++i)
        {
            ConstBundleHandle child = inputBundle->getConstChildBundle(i);
            ogn::BundleContents<ogn::kOgnInput, ogn::kCpu> extractedBundle(context, child);
            const auto pathAttr = extractedBundle.attributeByName(ImmediateNode::kSourcePrimPathToken);
            if (!pathAttr.isValid())
            {
                db.logError(
                    "Invalid non Prim child bundle has been added to the 'prims' input (missing 'sourcePrimPath')");
                state.immediateBVH.release();
                return false;
            }
            auto path = pathAttr.getCpu<Token>();
            if (!path)
            {
                db.logError("Invalid non Prim child bundle has been added to the 'prims' input (empty 'sourcePrimPath')");
                state.immediateBVH.release();
                return false;
            }

            const auto boxMinAttr = extractedBundle.attributeByName(ImmediateNode::kBoundingBoxMin);
            const auto boxMaxAttr = extractedBundle.attributeByName(ImmediateNode::kBoundingBoxMax);
            if (!boxMinAttr.isValid() || !boxMaxAttr.isValid())
            {
                db.logWarning("Prim '%s' is missing 'bboxMinCorner' and 'bboxMaxCorner' attributes",
                              db.tokenToString(path->token));
                state.immediateBVH.release();
                return false;
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
                        db.logWarning(
                            "Prim '%s' needs both 'bboxTransform' and 'worldMatrix' attributes convertible to float[16] or double[16]",
                            db.tokenToString(path->token));
                        state.immediateBVH.release();
                        return false;
                    }
                }
                state.immediateBVH.mInputMeshPaths[i] = path->token;
                state.immediateBVH.meshWorldBounds[i] = bounds;
            }
            else
            {
                db.logWarning(
                    "Prim '%s' 'bboxMinCorner' and 'bboxMaxCorner' attributes must be convertible to float[3] or double[3]",
                    db.tokenToString(path->token));
                state.immediateBVH.release();
                return false;
            }
        }
        // PxBVH  will print warnings if it's invoked with 0 input array
        if (childrenCount > 1)
        {
            state.immediateBVH.buildBVH();
            state.immediateBVH.broadPhase();
        }

        const size_t numOverlaps = state.immediateBVH.boxesOverlapPairs.size();
        db.outputs.overlapsPair0.resize(numOverlaps);
        db.outputs.overlapsPair1.resize(numOverlaps);

        // Finally we can output the found overlap pairs
        for (size_t idx = 0; idx < numOverlaps; ++idx)
        {
            const physx::PxGeomIndexPair& pair = state.immediateBVH.boxesOverlapPairs[idx];
            db.outputs.overlapsPair0()[idx] = state.immediateBVH.mInputMeshPaths[pair.id0];
            db.outputs.overlapsPair1()[idx] = state.immediateBVH.mInputMeshPaths[pair.id1];
        }

        state.immediateBVH.release();
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
