// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

// include auto-generated header
#include <OgnPhysXImmediateGenerateGeometryContactsDatabase.h>

#include <PxImmediateMode.h>

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
struct MeshGenerateContactsResult;
struct ImmediateGenerateContacts;

} // namespace graph
} // namespace physx
} // namespace omni

using namespace ::physx;
using namespace omni::physx::graph;


struct omni::physx::graph::MeshGenerateContactsResult
{
    // Outputs
    bool doesOverlap = false;
    bool isDuplicate = false;
    bool areBothTriangleMeshes = false;
    bool hasPenetration = false;

    // Inputs
    std::vector<::physx::PxContactPoint> contacts;

    void reset()
    {
        contacts.clear();
    }
};

struct omni::physx::graph::ImmediateGenerateContacts : public ImmediateNarrowPhase
{
    ImmediateShared* shared = nullptr;
    float contactDistance = 0;
    float meshContactMargin = 0;
    float toleranceLength = 0;

    // Outputs
    std::vector<MeshGenerateContactsResult> results;

    void init(ImmediateShared& immediateShared)
    {
        shared = &immediateShared;
    }

    void release()
    {
        results.clear();
    }

    static void threadFunction(MeshGenerateContactsResult* PX_RESTRICT results,
                               uint32_t nbPairs,
                               const ::physx::PxGeomIndexPair* PX_RESTRICT pairs,
                               const MeshGeometryData* PX_RESTRICT geometries,
                               const ImmediateGenerateContacts* self)
    {
        PX_SIMD_GUARD
        // It could be worth keeping the contact cache in result structure to re-use it over successive invocations
        std::vector<::physx::PxCache> contactCacheArray;
        contactCacheArray.resize(nbPairs);

        std::vector<const ::physx::PxGeometry*> geom0array, geom1array;
        std::vector<::physx::PxTransform> pose0array, pose1array;
        pose0array.reserve(nbPairs);
        pose1array.reserve(nbPairs);
        geom0array.reserve(nbPairs);
        geom1array.reserve(nbPairs);
        uint32_t actualPairs = 0;
        for (uint32_t i = 0; i < nbPairs; i++)
        {
            results[i].reset();

            const PxGeomIndexPair& p = pairs[i];
            const uint32_t id0 = p.id0;
            const uint32_t id1 = p.id1;
            if (!geometries[id0].isValid || !geometries[id1].isValid)
                continue; // Invalid geometry, probably failed cooking

            actualPairs++;
            const pxr::GfVec3d trans0 = geometries[id0].worldMatrix.ExtractTranslation();
            const pxr::GfVec3d trans1 = geometries[id1].worldMatrix.ExtractTranslation();

            PxTransform pose0 = geometries[id0].worldTransform;
            PxTransform pose1 = geometries[id1].worldTransform;
            pose0.p = PxVec3(0.0f);
            pose1.p = PxVec3(float(trans1[0] - trans0[0]), float(trans1[1] - trans0[1]), float(trans1[2] - trans0[2]));
            pose0array.push_back(pose0);
            pose1array.push_back(pose1);

            if (geometries[id0].convexMeshGeometry.convexMesh)
            {
                geom0array.push_back(&geometries[id0].convexMeshGeometry);
            }
            else
            {
                geom0array.push_back(&geometries[id0].triangleMeshGeometry);
            }

            if (geometries[id1].convexMeshGeometry.convexMesh)
            {
                geom1array.push_back(&geometries[id1].convexMeshGeometry);
            }
            else
            {
                geom1array.push_back(&geometries[id1].triangleMeshGeometry);
            }
        }

        if (actualPairs == 0)
            return;

        ::physx::PxReal contactDistance = self->contactDistance;
        ::physx::PxReal meshContactMargin = self->meshContactMargin;
        ::physx::PxReal toleranceLength = self->toleranceLength;


        struct ContactRecorder : ::physx::immediate::PxContactRecorder
        {
            MeshGenerateContactsResult* results;
            ContactRecorder(MeshGenerateContactsResult* results) : results(results)
            {
            }
            virtual bool recordContacts(const ::physx::PxContactPoint* contactPoints,
                                        const ::physx::PxU32 nbContacts,
                                        const ::physx::PxU32 index)
            {
                for (::physx::PxU32 i = 0; i < nbContacts; ++i)
                    results[index].contacts.push_back(contactPoints[i]);

                return true;
            }
        } contactRecorder(results);


        struct ContactCacheAllocator : ::physx::PxCacheAllocator
        {
            ::physx::PxAllocatorCallback& allocator;
            std::vector<::physx::PxU8*> buffers;

            ContactCacheAllocator(::physx::PxAllocatorCallback& _allocator) : allocator(_allocator)
            {
            }
            ~ContactCacheAllocator()
            {
                for (auto buffer : buffers)
                {
                    allocator.deallocate(buffer);
                }
            }
            virtual ::physx::PxU8* allocateCacheData(const ::physx::PxU32 byteSize)
            {
                auto buffer = (::physx::PxU8*)allocator.allocate(byteSize, "ContactCacheAllocator", __FILE__, __LINE__);
                buffers.push_back(buffer);
                return buffer;
            }
        } contactCacheAllocator(self->shared->pxAllocator);
        ::physx::immediate::PxGenerateContacts(
            geom0array.data(), geom1array.data(), pose0array.data(), pose1array.data(), contactCacheArray.data(),
            actualPairs, contactRecorder, contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);
    }

    void generateContacts(const std::vector<MeshGeometryData>& meshGeometryData,
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
                MeshGenerateContactsResult* narrowResult = results.data() + start;

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

class OgnPhysXImmediateGenerateGeometryContacts
{
    ImmediateHashMeshes phaseHashMeshes;
    ImmediateFindDuplicates phaseFindDuplicates;
    ImmediateCookMeshes phaseCookMeshes;
    ImmediateCreateGeometries phaseCreateGeometries;
    ImmediateGenerateContacts phaseGenerateContacts;
    std::vector<physx::PxGeomIndexPair> boxesOverlapPairs;

public:
    // This method might be overridden to set up initial conditions when a node type is registered, or
    // to replace initialization if the auto-generated version has some problem.
    static void initialize(const GraphContextObj& graphContextObj, const NodeObj& nodeObj)
    {
        if (ImmediateNode::getGlobals().initialize())
        {
            auto& state =
                OgnPhysXImmediateGenerateGeometryContactsDatabase::sSharedState<OgnPhysXImmediateGenerateGeometryContacts>(
                    nodeObj);
            state.phaseHashMeshes.init(ImmediateNode::getGlobals());
            state.phaseCookMeshes.init(ImmediateNode::getGlobals());
            state.phaseCreateGeometries.init(ImmediateNode::getGlobals());
            state.phaseGenerateContacts.init(ImmediateNode::getGlobals());
        }
        else
        {
            CARB_LOG_ERROR("Cannot initialize OgnPhysXImmediateGenerateGeometryContacts Globals");
        }
    }
    // After a node is removed it will get a release call where anything set up in initialize() can be torn down
    static void release(const NodeObj& nodeObj)
    {
        auto& state =
            OgnPhysXImmediateGenerateGeometryContactsDatabase::sSharedState<OgnPhysXImmediateGenerateGeometryContacts>(
                nodeObj);
        state.phaseHashMeshes.release();
        state.phaseFindDuplicates.release();
        state.phaseCookMeshes.release();
        state.phaseCreateGeometries.release();
        state.phaseGenerateContacts.release();
        state.boxesOverlapPairs.clear();
        ImmediateNode::getGlobals().release();
    }

    static bool compute(OgnPhysXImmediateGenerateGeometryContactsDatabase& db)
    {
        auto& state = db.sharedState<OgnPhysXImmediateGenerateGeometryContacts>();
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
        state.phaseGenerateContacts.contactDistance = db.inputs.contactDistance();
        state.phaseGenerateContacts.meshContactMargin = db.inputs.meshContactMargin();
        state.phaseGenerateContacts.toleranceLength = db.inputs.toleranceLength();

        state.phaseGenerateContacts.generateContacts(
            state.phaseCreateGeometries.meshGeometryData, state.boxesOverlapPairs);

        // Output results
        const size_t numOverlaps = state.phaseGenerateContacts.results.size();
        bool warningPrinted = false;
        bool errorPrinted = false;

        std::vector<NameToken> newNames(numOverlaps);
        omni::physx::graph::createPrimName(context.iToken->getHandle("prim"), 0, { newNames.data(), newNames.size() });
        IBundle2* outputContactsBundle = db.outputs.contacts().abi_bundleInterface();
        outputContactsBundle->clearContents();

        for (size_t idx = 0; idx < numOverlaps; ++idx)
        {
            const MeshGenerateContactsResult& result = state.phaseGenerateContacts.results[idx];
            if (result.areBothTriangleMeshes && !errorPrinted)
            {
                errorPrinted = true;
                NameToken body0 = db.inputs.overlapsPair0()[idx];
                NameToken body1 = db.inputs.overlapsPair1()[idx];
                db.logError("Prim '%s' and '%s' are both TriangleMeshes, their penetration cannot be computed",
                            db.tokenToString(body0), db.tokenToString(body1));
            }
            else if (result.isDuplicate && !warningPrinted)
            {
                warningPrinted = true;
                NameToken body0 = db.inputs.overlapsPair0()[idx];
                NameToken body1 = db.inputs.overlapsPair1()[idx];
                db.logWarning("Prim '%s' and '%s' are the same mesh in the same position", db.tokenToString(body0),
                              db.tokenToString(body1));
            }

            const size_t numContacts = result.contacts.size();

            BundleHandle childHandle = outputContactsBundle->createChildBundle(newNames[idx]);
            ogn::BundleContents<ogn::kOgnOutput, ogn::kCpu> outputChildBundle(
                outputContactsBundle->getContext(), childHandle);
            ogn::RuntimeAttribute<ogn::kOgnOutput, ogn::kCpu> sourcePrimPathAttribute = outputChildBundle.addAttribute(
                ImmediateNode::kSourcePrimPathToken, Type(BaseDataType::eToken, 1, 0, AttributeRole::eNone));
            *sourcePrimPathAttribute.getCpu<NameToken>() = newNames[idx];

            static NameToken const kContactPoints = omni::fabric::asInt(pxr::TfToken("points", pxr::TfToken::Immortal));
            static NameToken const kContactNormals = omni::fabric::asInt(pxr::TfToken("normals", pxr::TfToken::Immortal));
            static NameToken const kContactDepths = omni::fabric::asInt(pxr::TfToken("depths", pxr::TfToken::Immortal));

            ogn::RuntimeAttribute<ogn::kOgnOutput, ogn::kCpu> contactPointsAttribute =
                outputChildBundle.addAttribute(kContactPoints, Type(BaseDataType::eFloat, 3, 1, AttributeRole::eNone));

            ogn::RuntimeAttribute<ogn::kOgnOutput, ogn::kCpu> contactNormalsAttribute =
                outputChildBundle.addAttribute(kContactNormals, Type(BaseDataType::eFloat, 3, 1, AttributeRole::eNone));

            ogn::RuntimeAttribute<ogn::kOgnOutput, ogn::kCpu> contactDepthsAttribute =
                outputChildBundle.addAttribute(kContactDepths, Type(BaseDataType::eFloat, 1, 1, AttributeRole::eNone));


            auto contactPoints = contactPointsAttribute.getCpu<float[][3]>();
            auto contactNormals = contactNormalsAttribute.getCpu<float[][3]>();
            auto contactDepths = contactDepthsAttribute.getCpu<float[]>();

            contactPoints->resize(result.contacts.size());
            contactNormals->resize(result.contacts.size());
            contactDepths->resize(result.contacts.size());
            for (size_t contactIdx = 0; contactIdx < numContacts; ++contactIdx)
            {
                float* point = contactPoints->data()[contactIdx];
                float* normal = contactNormals->data()[contactIdx];
                memcpy(point, &result.contacts[contactIdx].point, sizeof(PxVec3));
                memcpy(normal, &result.contacts[contactIdx].normal, sizeof(PxVec3));
                contactDepths->data()[contactIdx] = result.contacts[contactIdx].separation;
            }
        }

        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }
};

REGISTER_OGN_NODE()
