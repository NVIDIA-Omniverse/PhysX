// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxFabric.h>
#include <omni/physx/IPhysxSimulation.h>
#include <private/omni/physx/IPhysxPrivate.h>

#include <omni/fabric/FabricUSD.h>
#include <omni/fabric/SimStageWithHistory.h>

#include "FabricManager.h"
#include "SurfaceDeformableBodyManager.h"
#include "VolumeDeformableBodyManager.h"

using namespace PXR_NS;
using namespace omni::physx;
using namespace ::physx;

namespace omni
{
namespace physx
{
// FabricManager.cpp and FabricKernels.cu are not part of this test binary, only the deformable
// body managers under test are.
PxCudaContextManager* FabricManager::getCudaContextManager()
{
    IPhysxPrivate* iPhysxPrivate = carb::getCachedInterface<IPhysxPrivate>();
    return iPhysxPrivate ? iPhysxPrivate->getCudaContextManager() : nullptr;
}

void convertToVec3fBlockDeformableBody(DeformableBodyGPUData*, int, int, FabricCudaStreamHandle)
{
}
} // namespace physx
} // namespace omni

// An unprepared body set must not be updated. Only a failing prepareBuffers() produces that state,
// so the sets are driven directly instead of through omni.physx.fabric.plugin.
TEST_CASE("Fabric Deformable Body Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=snie][priority=mandatory]")
{
    constexpr uint32_t kNumSimMeshVertices = 4;

    ScopedFabricActivation fabricEnable;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxPrivate* physxPrivate = physicsTests.acquirePhysxPrivateInterface();
    REQUIRE(physxPrivate);

    // The deformable bodies below have no simulation mesh, so PhysX reports an error for the
    // simulation position buffer, which the CPU interop path does not read. The event stream is
    // pumped on stage detach, so the listener stays off for the whole test case.
    ErrorEventListener& errorListener = physicsTests.getErrorEventListener();
    const bool errorListenerEnabled = errorListener.enabled();
    errorListener.setEnabled(false);
    ExecOnDestruction restoreErrorListener([&] { errorListener.setEnabled(errorListenerEnabled); });

    // The crashing path is the CPU interop one.
    carb::settings::ISettings* settings = physicsTests.acquireSettingsInterface();
    const bool gpuInterop = settings->getAsBool(kSettingFabricUseGPUInterop);
    settings->setBool(kSettingFabricUseGPUInterop, false);
    ExecOnDestruction restoreGpuInterop([&] { settings->setBool(kSettingFabricUseGPUInterop, gpuInterop); });

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath defaultPrimPath("/World");
    stage->DefinePrim(defaultPrimPath);
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene::Define(stage, physicsScenePath);
    const SdfPath simMeshPath = defaultPrimPath.AppendChild(TfToken("simMesh"));
    UsdGeomMesh::Define(stage, simMeshPath);

    UsdUtilsStageCache::Get().Insert(stage);
    const long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    physxSim->attachStage(stageId);
    ExecOnDestruction detachStage(
        [&]
        {
            physxSim->detachStage();
            UsdUtilsStageCache::Get().Erase(stage);
        });

    // The stage attach brings up the CUDA context manager.
    PxCudaContextManager* cudaContextManager = physxPrivate->getCudaContextManager();
    REQUIRE(cudaContextManager);

    // Fabric stage the body sets write their points into.
    omni::fabric::UsdStageId fabricStageId;
    fabricStageId.id = uint64_t(stageId);

    omni::fabric::ISimStageWithHistory* iSimStageWithHistory =
        carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
    REQUIRE(iSimStageWithHistory);
    iSimStageWithHistory->getOrCreate(fabricStageId, 1, { 1, 60 }, omni::fabric::GpuComputeType::eCuda);
    ExecOnDestruction releaseSimStage([&] { iSimStageWithHistory->release(fabricStageId); });

    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    REQUIRE(iStageReaderWriter);
    const omni::fabric::StageReaderWriterId srwId = iStageReaderWriter->create(fabricStageId, 0);
    REQUIRE(srwId.id != omni::fabric::kInvalidStageReaderWriterId.id);
    omni::fabric::StageReaderWriter srw(srwId);

    PxScene* pxScene = getPhysxSceneAtPathChecked(physicsScenePath);
    PxPhysics& physics = pxScene->getPhysics();
    const UsdPrim simMeshPrim = stage->GetPrimAtPath(simMeshPath);
    const omni::fabric::Path simMeshPathHandle =
        omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), simMeshPath);
    const omni::fabric::Token pointsToken("points");

    // Stands in for the simulation results copied into fabric.
    std::vector<PxVec4> simMeshPositionInvMass(kNumSimMeshVertices, PxVec4(0.0f));
    for (uint32_t i = 0; i < kNumSimMeshVertices; ++i)
    {
        simMeshPositionInvMass[i] = PxVec4(float(i), 2.0f, 3.0f, 0.0f);
    }

    SUBCASE("Volume deformable body")
    {
        PxDeformableVolume* deformableVolume = physics.createDeformableVolume(*cudaContextManager);
        REQUIRE(deformableVolume);
        ExecOnDestruction releaseDeformableVolume([&] { deformableVolume->release(); });

        VolumeDeformableBody body;
        body.physXPtr = deformableVolume;
        body.hasCollMesh = false;
        body.data.simMeshPrim = simMeshPrim;
        body.data.numSimMeshVertices = kNumSimMeshVertices;
        body.data.worldToSimMesh = GfMatrix4f(1.0f);
        body.data.simMeshPositionInvMassH = simMeshPositionInvMass.data();
        body.data.numSkinMeshVertices = 0;
        body.data.allSkinnedVerticesH = nullptr;
        body.data.allSkinnedVerticesD = nullptr;
        body.data.numCollMeshVertices = 0;
        body.data.collMeshPositionInvMassH = nullptr;

        SUBCASE("Update skips a set that has no interop buffers")
        {
            VolumeDeformableBodySet set;
            set.mVolumeDeformableBodies.push_back(&body);

            // Buffers as a failed prepareBuffers() leaves them.
            REQUIRE(set.mDeformableBodyGPUDataH == nullptr);

            set.updateDeformableBodies(srw);

            CHECK(set.mDeformableBodyGPUDataH == nullptr);
        }

        SUBCASE("Update fills the interop data of a prepared set")
        {
            VolumeDeformableBodySet set;
            set.mVolumeDeformableBodies.push_back(&body);

            set.prepareBuffers(srw);
            REQUIRE(set.mDeformableBodyGPUDataH != nullptr);
            CHECK_EQ(set.mNumFabricAttributes, 1u);

            set.updateDeformableBodies(srw);

            CHECK_EQ(set.mDeformableBodyGPUDataH[0].nbPoints, int(kNumSimMeshVertices));
            CHECK(set.mDeformableBodyGPUDataH[0].dstPoints != nullptr);

            // The simulation positions end up transformed in the fabric points.
            gsl::span<const GfVec3f> points = srw.getArrayAttributeRd<GfVec3f>(simMeshPathHandle, pointsToken);
            REQUIRE_EQ(points.size(), size_t(kNumSimMeshVertices));
            compare(points[1], GfVec3f(1.0f, 2.0f, 3.0f), 1.0e-5f);
        }
    }

    SUBCASE("Surface deformable body")
    {
        PxDeformableSurface* deformableSurface = physics.createDeformableSurface(*cudaContextManager);
        REQUIRE(deformableSurface);
        ExecOnDestruction releaseDeformableSurface([&] { deformableSurface->release(); });

        SurfaceDeformableBody body;
        body.physXPtr = deformableSurface;
        body.data.simMeshPrim = simMeshPrim;
        body.data.numSimMeshVertices = kNumSimMeshVertices;
        body.data.worldToSimMesh = GfMatrix4f(1.0f);
        body.data.simMeshPositionInvMassH = simMeshPositionInvMass.data();
        body.data.numSkinMeshVertices = 0;
        body.data.allSkinnedVerticesH = nullptr;
        body.data.allSkinnedVerticesD = nullptr;

        SUBCASE("Update skips a set that has no interop buffers")
        {
            SurfaceDeformableBodySet set;
            set.mSurfaceDeformableBodies.push_back(&body);

            // Buffers as a failed prepareBuffers() leaves them.
            REQUIRE(set.mDeformableBodyGPUDataH == nullptr);

            set.updateSurfaceDeformableBodies(srw);

            CHECK(set.mDeformableBodyGPUDataH == nullptr);
        }

        SUBCASE("Update fills the interop data of a prepared set")
        {
            SurfaceDeformableBodySet set;
            set.mSurfaceDeformableBodies.push_back(&body);

            set.prepareBuffers(srw);
            REQUIRE(set.mDeformableBodyGPUDataH != nullptr);
            CHECK_EQ(set.mNumFabricAttributes, 1u);

            set.updateSurfaceDeformableBodies(srw);

            CHECK_EQ(set.mDeformableBodyGPUDataH[0].nbPoints, int(kNumSimMeshVertices));
            CHECK(set.mDeformableBodyGPUDataH[0].dstPoints != nullptr);

            // The simulation positions end up transformed in the fabric points.
            gsl::span<const GfVec3f> points = srw.getArrayAttributeRd<GfVec3f>(simMeshPathHandle, pointsToken);
            REQUIRE_EQ(points.size(), size_t(kNumSimMeshVertices));
            compare(points[1], GfVec3f(1.0f, 2.0f, 3.0f), 1.0e-5f);
        }
    }
}


namespace
{
constexpr uint32_t kNumSkinVerticesBefore = 4;
constexpr uint32_t kNumSkinVerticesAfter = 6;

const TfToken gPoseInstanceToken("default");

void setXformOps(const UsdGeomXformable& xformable)
{
    xformable.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(0.0f));
    xformable.AddOrientOp(UsdGeomXformOp::PrecisionFloat).Set(GfQuatf(1.0f, GfVec3f(0.0f)));
    xformable.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(GfVec3f(1.0f));

    VtTokenArray xformOpOrder = { TfToken("xformOp:translate"), TfToken("xformOp:orient"),
                                  TfToken("xformOp:scale") };
    xformable.GetXformOpOrderAttr().Set(xformOpOrder);
}

void setBindPoseAttrs(const VtArray<GfVec3f>& meshPoints, UsdPrim& meshPrim)
{
    const TfToken pointsAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, gPoseInstanceToken);
    const TfToken purposesAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, gPoseInstanceToken);

    meshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformablePoseAPI, gPoseInstanceToken);
    meshPrim.GetAttribute(pointsAttrName).Set(meshPoints);

    VtTokenArray tokens = { TfToken("bindPose") };
    meshPrim.GetAttribute(purposesAttrName).Set(tokens);
}

// Re-authors an existing skin mesh in place when the path is already there.
void authorSkinMesh(const UsdStageRefPtr& stage, const SdfPath& skinMeshPath, const VtArray<GfVec3f>& meshPoints)
{
    UsdGeomMesh skinMesh = UsdGeomMesh::Define(stage, skinMeshPath);
    UsdPrim skinMeshPrim = skinMesh.GetPrim();

    skinMesh.CreatePointsAttr().Set(meshPoints);

    if (meshPoints.size() == kNumSkinVerticesBefore)
    {
        skinMesh.CreateFaceVertexCountsAttr().Set(VtIntArray({ 4 }));
        skinMesh.CreateFaceVertexIndicesAttr().Set(VtIntArray({ 0, 1, 2, 3 }));
    }
    else
    {
        skinMesh.CreateFaceVertexCountsAttr().Set(VtIntArray({ 4, 4 }));
        skinMesh.CreateFaceVertexIndicesAttr().Set(VtIntArray({ 0, 1, 4, 5, 1, 2, 3, 4 }));
    }

    setBindPoseAttrs(meshPoints, skinMeshPrim);

    if (!UsdGeomXformable(skinMesh).GetXformOpOrderAttr().HasAuthoredValue())
    {
        setXformOps(skinMesh);
    }
}

struct SurfaceCase
{
    using PxType = PxDeformableSurface;
    using DataType = InternalSurfaceDeformableBodyData;
    using ManagerType = SurfaceDeformableBodyManager;

    static constexpr PhysXType kPhysXType = ePTDeformableSurface;

    static void getInternalData(IPhysxPrivate* physxPrivate, PxType* body, DataType& data)
    {
        physxPrivate->getInternalSurfaceDeformableBodyData((const usdparser::ObjectId)body->userData, data);
    }

    // In the plane of the simulation mesh square.
    static VtArray<GfVec3f> skinPoints(bool wide)
    {
        if (!wide)
        {
            return VtArray<GfVec3f>({ GfVec3f(0.0f, -25.0f, -25.0f), GfVec3f(0.0f, 25.0f, -25.0f),
                                      GfVec3f(0.0f, 25.0f, 25.0f), GfVec3f(0.0f, -25.0f, 25.0f) });
        }

        return VtArray<GfVec3f>({ GfVec3f(0.0f, -30.0f, -30.0f), GfVec3f(0.0f, 0.0f, -30.0f),
                                  GfVec3f(0.0f, 30.0f, -30.0f), GfVec3f(0.0f, 30.0f, 30.0f),
                                  GfVec3f(0.0f, 0.0f, 30.0f), GfVec3f(0.0f, -30.0f, 30.0f) });
    }

    // One square, and two squares side by side.
    static constexpr uint32_t kNumSimVerticesBefore = 4;
    static constexpr uint32_t kNumSimVerticesAfter = 6;

    static void authorSimMesh(const UsdStageRefPtr& stage, const SdfPath& simMeshPath, bool wide)
    {
        UsdGeomMesh simMesh = UsdGeomMesh::Define(stage, simMeshPath);
        UsdPrim simMeshPrim = simMesh.GetPrim();

        simMeshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
        UsdPhysicsCollisionAPI::Apply(simMeshPrim);

        VtArray<GfVec3f> meshPoints;
        VtArray<GfVec3i> triVtxIndices;
        if (!wide)
        {
            meshPoints = { GfVec3f(0.0f, -50.0f, -50.0f), GfVec3f(0.0f, 50.0f, -50.0f),
                           GfVec3f(0.0f, 50.0f, 50.0f), GfVec3f(0.0f, -50.0f, 50.0f) };
            triVtxIndices = { GfVec3i(0, 1, 2), GfVec3i(0, 2, 3) };
        }
        else
        {
            meshPoints = { GfVec3f(0.0f, -50.0f, -50.0f), GfVec3f(0.0f, 0.0f, -50.0f),
                           GfVec3f(0.0f, 50.0f, -50.0f),  GfVec3f(0.0f, 50.0f, 50.0f),
                           GfVec3f(0.0f, 0.0f, 50.0f),    GfVec3f(0.0f, -50.0f, 50.0f) };
            triVtxIndices = { GfVec3i(0, 1, 4), GfVec3i(0, 4, 5), GfVec3i(1, 2, 3), GfVec3i(1, 3, 4) };
        }

        VtIntArray faceVertexCounts(triVtxIndices.size(), 3);
        VtIntArray faceVertexIndices;
        for (const GfVec3i& tri : triVtxIndices)
        {
            faceVertexIndices.push_back(tri[0]);
            faceVertexIndices.push_back(tri[1]);
            faceVertexIndices.push_back(tri[2]);
        }

        simMesh.CreatePointsAttr().Set(meshPoints);
        simMesh.CreateFaceVertexCountsAttr().Set(faceVertexCounts);
        simMesh.CreateFaceVertexIndicesAttr().Set(faceVertexIndices);
        simMesh.CreateVelocitiesAttr().Set(VtArray<GfVec3f>(meshPoints.size(), GfVec3f(0.0f)));

        setBindPoseAttrs(meshPoints, simMeshPrim);

        simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(meshPoints);
        simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices).Set(triVtxIndices);

        if (!UsdGeomXformable(simMesh).GetXformOpOrderAttr().HasAuthoredValue())
        {
            setXformOps(simMesh);
        }
    }

    static void authorBody(const UsdStageRefPtr& stage,
                           const SdfPath& bodyPath,
                           const SdfPath& skinMeshPath,
                           const SdfPath& simMeshPath)
    {
        UsdGeomXform xform = UsdGeomXform::Define(stage, bodyPath);
        xform.GetPrim().ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        setXformOps(xform);

        authorSkinMesh(stage, skinMeshPath, skinPoints(false));
        authorSimMesh(stage, simMeshPath, false);
    }
};

struct VolumeCase
{
    using PxType = PxDeformableVolume;
    using DataType = InternalVolumeDeformableBodyData;
    using ManagerType = VolumeDeformableBodyManager;

    static constexpr PhysXType kPhysXType = ePTDeformableVolume;

    static void getInternalData(IPhysxPrivate* physxPrivate, PxType* body, DataType& data)
    {
        physxPrivate->getInternalVolumeDeformableBodyData((const usdparser::ObjectId)body->userData, data);
    }

    // Inside the first tetrahedron of the simulation mesh below (x, y, z > 0, x + y + z < 100).
    static VtArray<GfVec3f> skinPoints(bool wide)
    {
        if (!wide)
        {
            return VtArray<GfVec3f>({ GfVec3f(10.0f, 10.0f, 10.0f), GfVec3f(30.0f, 10.0f, 10.0f),
                                      GfVec3f(10.0f, 30.0f, 10.0f), GfVec3f(10.0f, 10.0f, 30.0f) });
        }

        return VtArray<GfVec3f>({ GfVec3f(10.0f, 10.0f, 10.0f), GfVec3f(25.0f, 10.0f, 10.0f),
                                  GfVec3f(40.0f, 10.0f, 10.0f), GfVec3f(10.0f, 25.0f, 10.0f),
                                  GfVec3f(10.0f, 40.0f, 10.0f), GfVec3f(10.0f, 10.0f, 25.0f) });
    }

    // One tetrahedron, and two sharing a face.
    static constexpr uint32_t kNumSimVerticesBefore = 4;
    static constexpr uint32_t kNumSimVerticesAfter = 5;

    static void authorSimMesh(const UsdStageRefPtr& stage, const SdfPath& simMeshPath, bool wide)
    {
        UsdGeomTetMesh simMesh = UsdGeomTetMesh::Define(stage, simMeshPath);
        UsdPrim simMeshPrim = simMesh.GetPrim();

        simMeshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
        UsdPhysicsCollisionAPI::Apply(simMeshPrim);

        VtArray<GfVec3f> meshPoints = { GfVec3f(0.0f, 0.0f, 0.0f), GfVec3f(100.0f, 0.0f, 0.0f),
                                        GfVec3f(0.0f, 100.0f, 0.0f), GfVec3f(0.0f, 0.0f, 100.0f) };
        VtArray<GfVec4i> tetVertexIndices = { GfVec4i(0, 1, 2, 3) };
        if (wide)
        {
            meshPoints.push_back(GfVec3f(100.0f, 100.0f, 100.0f));
            tetVertexIndices.push_back(GfVec4i(1, 2, 3, 4));
        }

        simMesh.CreatePointsAttr().Set(meshPoints);
        simMesh.CreateTetVertexIndicesAttr().Set(tetVertexIndices);
        simMesh.CreateVelocitiesAttr().Set(VtArray<GfVec3f>(meshPoints.size(), GfVec3f(0.0f)));

        setBindPoseAttrs(meshPoints, simMeshPrim);

        simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(meshPoints);
        simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices).Set(tetVertexIndices);

        VtArray<GfVec3i> surfaceFaceVertexIndices;
        UsdGeomTetMesh::ComputeSurfaceFaces(simMesh, &surfaceFaceVertexIndices, UsdTimeCode::Default());
        simMesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices);

        if (!UsdGeomXformable(simMesh).GetXformOpOrderAttr().HasAuthoredValue())
        {
            setXformOps(simMesh);
        }
    }

    static void authorBody(const UsdStageRefPtr& stage,
                           const SdfPath& bodyPath,
                           const SdfPath& skinMeshPath,
                           const SdfPath& simMeshPath)
    {
        UsdGeomXform xform = UsdGeomXform::Define(stage, bodyPath);
        xform.GetPrim().ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        setXformOps(xform);

        authorSkinMesh(stage, skinMeshPath, skinPoints(false));
        authorSimMesh(stage, simMeshPath, false);
    }
};

// Expects the fabric points of both meshes to follow the body that is alive now, after a resync
// destroyed and re-created it at the same prim path with meshes of a different size.
template <typename CaseT>
void checkSnapshotIsRefreshedOnRebuild()
{
    constexpr float kTimeStep = 1.0f / 60.0f;

    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);
    IPhysxPrivate* physxPrivate = physicsTests.acquirePhysxPrivateInterface();
    REQUIRE(physxPrivate);

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    const SdfPath defaultPrimPath("/World");
    stage->SetDefaultPrim(stage->DefinePrim(defaultPrimPath));

    // No gravity: the skinned vertices stay at their bind pose, so the fabric points are exact.
    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.CreateGravityMagnitudeAttr().Set(0.0f);

    const SdfPath bodyPath = defaultPrimPath.AppendChild(TfToken("deformable"));
    const SdfPath skinMeshPath = bodyPath.AppendChild(TfToken("skinMesh"));
    const SdfPath simMeshPath = bodyPath.AppendChild(TfToken("simMesh"));
    CaseT::authorBody(stage, bodyPath, skinMeshPath, simMeshPath);

    UsdUtilsStageCache::Get().Insert(stage);
    const long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    physxSim->attachStage(stageId);
    ExecOnDestruction detachStage(
        [&]
        {
            physxSim->detachStage();
            UsdUtilsStageCache::Get().Erase(stage);
        });

    typename CaseT::PxType* bodyBefore =
        static_cast<typename CaseT::PxType*>(physx->getPhysXPtr(bodyPath, CaseT::kPhysXType));
    REQUIRE(bodyBefore);

    typename CaseT::DataType dataBefore = {};
    CaseT::getInternalData(physxPrivate, bodyBefore, dataBefore);
    REQUIRE(dataBefore.allSkinnedVerticesH != nullptr);
    REQUIRE_EQ(dataBefore.numSkinMeshVertices, kNumSkinVerticesBefore);

    // Fabric stage the manager writes the body's points into.
    omni::fabric::UsdStageId fabricStageId;
    fabricStageId.id = uint64_t(stageId);

    omni::fabric::ISimStageWithHistory* iSimStageWithHistory =
        carb::getCachedInterface<omni::fabric::ISimStageWithHistory>();
    REQUIRE(iSimStageWithHistory);
    iSimStageWithHistory->getOrCreate(fabricStageId, 1, { 1, 60 }, omni::fabric::GpuComputeType::eCuda);
    ExecOnDestruction releaseSimStage([&] { iSimStageWithHistory->release(fabricStageId); });

    omni::fabric::IStageReaderWriter* iStageReaderWriter = carb::getCachedInterface<omni::fabric::IStageReaderWriter>();
    REQUIRE(iStageReaderWriter);
    const omni::fabric::StageReaderWriterId srwId = iStageReaderWriter->create(fabricStageId, 0);
    REQUIRE(srwId.id != omni::fabric::kInvalidStageReaderWriterId.id);
    omni::fabric::StageReaderWriter srw(srwId);

    const omni::fabric::Token pointsToken("points");
    const omni::fabric::Path skinMeshPathHandle =
        omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), skinMeshPath);
    const omni::fabric::Path simMeshPathHandle =
        omni::fabric::convertToPathType<omni::fabric::Path>(srw.getFabricId(), simMeshPath);

    typename CaseT::ManagerType manager;
    UsdGeomXformCache xfCache;
    manager.registerDeformableBody(xfCache, uint64_t(stageId), iStageReaderWriter, srwId, stage->GetPrimAtPath(bodyPath));

    // Baseline: the meshes of the registered body end up in fabric.
    manager.update(srw);

    REQUIRE(srw.attributeExists(simMeshPathHandle, pointsToken));
    REQUIRE_EQ(srw.getArrayAttributeRd<GfVec3f>(simMeshPathHandle, pointsToken).size(),
               size_t(CaseT::kNumSimVerticesBefore));

    REQUIRE(srw.attributeExists(skinMeshPathHandle, pointsToken));
    {
        gsl::span<const GfVec3f> points = srw.getArrayAttributeRd<GfVec3f>(skinMeshPathHandle, pointsToken);
        REQUIRE_EQ(points.size(), size_t(kNumSkinVerticesBefore));

        const VtArray<GfVec3f> expected = CaseT::skinPoints(false);
        for (size_t i = 0; i < points.size(); ++i)
        {
            compare(points[i], expected[i], 1.0e-3f);
        }
    }

    // Resync the body prim with different simulation and skin meshes.
    {
        UsdPrim bodyPrim = stage->GetPrimAtPath(bodyPath);
        bodyPrim.RemoveAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        CaseT::authorSimMesh(stage, simMeshPath, true);
        authorSkinMesh(stage, skinMeshPath, CaseT::skinPoints(true));
        bodyPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
    }

    physxSim->simulate(kTimeStep, 0.0f);
    physxSim->fetchResults();

    typename CaseT::PxType* bodyAfter =
        static_cast<typename CaseT::PxType*>(physx->getPhysXPtr(bodyPath, CaseT::kPhysXType));
    REQUIRE(bodyAfter);

    typename CaseT::DataType dataAfter = {};
    CaseT::getInternalData(physxPrivate, bodyAfter, dataAfter);
    REQUIRE(dataAfter.allSkinnedVerticesH != nullptr);

    // The body really was re-created: it simulates and skins the new meshes now. Its pinned host
    // buffer is not compared - CUDA hands the freed address of the destroyed body out again.
    REQUIRE_EQ(dataAfter.numSimMeshVertices, CaseT::kNumSimVerticesAfter);
    REQUIRE_EQ(dataAfter.numSkinMeshVertices, kNumSkinVerticesAfter);

    // Stands in for FabricManager's object notification callbacks, which are not in this binary.
    manager.setIsDirty();

    // This update has to run on a refreshed snapshot; on a stale one it reads the freed pinned host
    // buffer and dangling spans of the destroyed body.
    manager.update(srw);

    // numSimMeshVertices is a scalar held by value, so unlike anything behind the dangling spans it
    // cannot come out right by the allocator reusing the memory of the destroyed body.
    REQUIRE(srw.attributeExists(simMeshPathHandle, pointsToken));
    CHECK_EQ(srw.getArrayAttributeRd<GfVec3f>(simMeshPathHandle, pointsToken).size(),
             size_t(CaseT::kNumSimVerticesAfter));

    REQUIRE(srw.attributeExists(skinMeshPathHandle, pointsToken));
    {
        gsl::span<const GfVec3f> points = srw.getArrayAttributeRd<GfVec3f>(skinMeshPathHandle, pointsToken);
        REQUIRE_EQ(points.size(), size_t(kNumSkinVerticesAfter));

        const VtArray<GfVec3f> expected = CaseT::skinPoints(true);
        for (size_t i = 0; i < points.size(); ++i)
        {
            compare(points[i], expected[i], 1.0e-3f);
        }
    }

    // A resync also re-registers the body, which is where the positions that reset on stop restores
    // are cached. Those have to follow a rebuild that changed the vertex count as well.
    manager.registerDeformableBody(xfCache, uint64_t(stageId), iStageReaderWriter, srwId,
                                   stage->GetPrimAtPath(bodyPath));
    manager.setInitialTransformation(srw);

    REQUIRE(srw.attributeExists(simMeshPathHandle, pointsToken));
    CHECK_EQ(srw.getArrayAttributeRd<GfVec3f>(simMeshPathHandle, pointsToken).size(),
             size_t(CaseT::kNumSimVerticesAfter));

    REQUIRE(srw.attributeExists(skinMeshPathHandle, pointsToken));
    CHECK_EQ(srw.getArrayAttributeRd<GfVec3f>(skinMeshPathHandle, pointsToken).size(),
             size_t(kNumSkinVerticesAfter));
}
} // namespace

// A deformable body that is destroyed and re-created at the same prim path must keep updating in
// fabric, with the meshes of the body that is alive now. The managers cache a snapshot of the
// internal body data - buffers and spans the destroyed body owned - so a rebuild has to re-read it.
TEST_CASE("Fabric Deformable Body Refresh Tests",
          "[omniphysics]"
          "[component=OmniPhysics][owner=snie][priority=mandatory]")
{
    ScopedFabricActivation fabricEnable;

    // The interop path under test is the CPU one, and it only runs when points are updated.
    carb::settings::ISettings* settings = PhysicsTest::getPhysicsTests()->acquireSettingsInterface();
    const bool gpuInterop = settings->getAsBool(kSettingFabricUseGPUInterop);
    const bool updatePoints = settings->getAsBool(kSettingFabricUpdatePoints);
    settings->setBool(kSettingFabricUseGPUInterop, false);
    settings->setBool(kSettingFabricUpdatePoints, true);
    ExecOnDestruction restoreSettings(
        [&]
        {
            settings->setBool(kSettingFabricUseGPUInterop, gpuInterop);
            settings->setBool(kSettingFabricUpdatePoints, updatePoints);
        });

    SUBCASE("Surface deformable body snapshot refresh")
    {
        checkSnapshotIsRefreshedOnRebuild<SurfaceCase>();
    }

    SUBCASE("Volume deformable body snapshot refresh")
    {
        checkSnapshotIsRefreshedOnRebuild<VolumeCase>();
    }
}
