// SPDX-FileCopyrightText: Copyright (c) 2020-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "../common/TestHelpers.h"

#include "PhysicsTools.h"

#include <PxPhysicsAPI.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxTests.h>
#include <omni/physx/IPhysxSimulation.h>

#include "cudamanager/PxCudaContextManager.h"
#include "cudamanager/PxCudaContext.h"

using namespace pxr;
using namespace omni::physx;
using namespace ::physx;

static const TfToken poseInstanceToken("default");

namespace
{
    void setXformOps(const UsdGeomXformable& xformable, const GfVec3f& translate, const GfQuatf& orient, const GfVec3f& scale)
    {
        xformable.AddTranslateOp(UsdGeomXformOp::PrecisionFloat).Set(translate);
        xformable.AddOrientOp(UsdGeomXformOp::PrecisionFloat).Set(orient);
        xformable.AddScaleOp(UsdGeomXformOp::PrecisionFloat).Set(scale);

        VtTokenArray xformOpOrder = { TfToken("xformOp:translate"), TfToken("xformOp:orient"), TfToken("xformOp:scale") };
        xformable.GetXformOpOrderAttr().Set(xformOpOrder);
    }

    void createXformForDeformable(const UsdStageRefPtr& stage, const SdfPath& xformPath)
    {
        UsdGeomXform xform = UsdGeomXform::Define(stage, xformPath);
        xform.GetPrim().ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        GfVec3f translate(0, 0, 0);
        GfQuatf orient(1, GfVec3f(0, 0, 0));
        GfVec3f scale(1, 1, 1);
        setXformOps(xform, translate, orient, scale);
    }

    void setBindPoseAttrs(const VtArray<GfVec3f>& meshPoints, UsdPrim& simMeshPrim)
    {
        TfToken pointsAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, poseInstanceToken);
        TfToken purposesAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, poseInstanceToken);

        simMeshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformablePoseAPI, poseInstanceToken);

        simMeshPrim.GetAttribute(pointsAttrName).Set(meshPoints);

        VtTokenArray tokens = { TfToken("bindPose") };
        simMeshPrim.GetAttribute(purposesAttrName).Set(tokens);
    }

    void createSurfaceDeformableHierarchical(const UsdStageRefPtr& stage, const SdfPath& xformPath, const SdfPath& skinMeshPath, const SdfPath& simMeshPath)
    {
        // Create Xform
        createXformForDeformable(stage, xformPath);

        // Create skin mesh
        {
            UsdGeomMesh skinMesh = createMeshSquare(stage, skinMeshPath, 50.0f, 50.0f);
            UsdPrim skinMeshPrim = skinMesh.GetPrim();

            VtArray<GfVec3f> meshPoints;
            skinMesh.GetPointsAttr().Get(&meshPoints);

            setBindPoseAttrs(meshPoints, skinMeshPrim);

            VtIntArray faceVertexCounts = { 4 };
            skinMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);
            VtIntArray faceVertexIndices = { 0, 1, 2, 3 };
            skinMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);

            GfVec3f translate(0, 0, 0);
            GfQuatf orient(1, GfVec3f(0, 0, 0));
            GfVec3f scale(1, 1, 1);
            setXformOps(skinMesh, translate, orient, scale);
        }

        // Create sim mesh
        {
            UsdGeomMesh simMesh = createMeshSquare(stage, simMeshPath, 50.0f, 50.0f);
            UsdPrim simMeshPrim = simMesh.GetPrim();

            simMeshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
            UsdPhysicsCollisionAPI::Apply(simMeshPrim);

            VtArray<GfVec3f> meshPoints;
            simMesh.GetPointsAttr().Get(&meshPoints);

            setBindPoseAttrs(meshPoints, simMeshPrim);

            VtIntArray faceVertexCounts = { 3, 3 };
            simMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);

            VtIntArray faceVertexIndices = { 0, 1, 2, 0, 2, 3 };
            simMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);

            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(meshPoints);

            VtArray<GfVec3i> restTriVtxIndices = { GfVec3i(0, 1, 2), GfVec3i(0, 2, 3) };
            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices).Set(restTriVtxIndices);

            VtArray<GfVec3f> velocities(meshPoints.size(), GfVec3f(0.0));
            simMesh.CreateVelocitiesAttr().Set(velocities);
        }
    }

    void createSurfaceDeformableNonHierarchical(const UsdStageRefPtr& stage, const SdfPath& deformablePath)
    {
        UsdGeomMesh mesh = createMeshSquare(stage, deformablePath, 50.0f, 50.0f);
        UsdPrim prim = mesh.GetPrim();

        VtArray<GfVec3f> meshPoints;
        mesh.GetPointsAttr().Get(&meshPoints);

        setBindPoseAttrs(meshPoints, prim);

        VtIntArray faceVertexCounts = { 3, 3 };
        mesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);

        VtIntArray faceVertexIndices = { 0, 1, 2, 0, 2, 3 };
        mesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);

        GfVec3f translate(0, 0, 0);
        GfQuatf orient(1, GfVec3f(0, 0, 0));
        GfVec3f scale(1, 1, 1);
        setXformOps(mesh, translate, orient, scale);

        prim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        prim.ApplyAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI);
        UsdPhysicsCollisionAPI::Apply(prim);

        prim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(meshPoints);

        VtArray<GfVec3i> restTriVtxIndices = { GfVec3i(0, 1, 2), GfVec3i(0, 2, 3) };
        prim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTriVtxIndices).Set(restTriVtxIndices);

        VtArray<GfVec3f> velocities(meshPoints.size(), GfVec3f(0.0));
        mesh.CreateVelocitiesAttr().Set(velocities);
    }

    template <typename T = int>
    struct ResultBuffer
    {
        ~ResultBuffer()
        {
            if (ptr)
            {
                std::free(ptr);
                ptr = nullptr;
            }
            size = 0;
        }

        static void* allocate(size_t numBytes)
        {
            return std::malloc(numBytes);
        }

        T* ptr = nullptr;
        uint32_t size = 0;
    };

    VtArray<GfVec4i> convertToVec4iArray(const VtArray<int>& input)
    {
        VtArray<GfVec4i> output;

        REQUIRE_EQ(input.size() % 4, 0);
        size_t outputSize = input.size() / 4;
        output.reserve(outputSize);
        for (int i = 0; i < outputSize; ++i)
        {
            output.push_back(GfVec4i(input[4 * i], input[4 * i + 1], input[4 * i + 2], input[4 * i + 3]));
        }

        return output;
    }

    VtArray<int> convertToIntArray(const VtArray<GfVec4i>& input)
    {
        VtArray<int> output;

        output.reserve(input.size()*4);

        for (const GfVec4i& elem : input)
        {
            output.push_back(elem[0]);
            output.push_back(elem[1]);
            output.push_back(elem[2]);
            output.push_back(elem[3]);
        }

        return output;
    }

    VtArray<int> ExtractTriangulatedFaces(UsdGeomMesh const& usdMesh)
    {
        // indices and faces converted to triangles
        VtArray<int> indices;
        usdMesh.GetFaceVertexIndicesAttr().Get(&indices);

        VtArray<int> faces;
        usdMesh.GetFaceVertexCountsAttr().Get(&faces);

        VtArray<int> triangles;
        if (indices.empty() || faces.empty())
            return triangles;

        triangles.reserve(1024);

        uint32_t indicesOffset = 0;

        uint32_t numIndices = uint32_t(indices.size());
        uint32_t numFaces = uint32_t(faces.size());
        for (uint32_t i = 0; i < numFaces; i++)
        {
            const uint32_t faceCount = faces[i];
            CARB_ASSERT(faceCount >= 3);
            CARB_ASSERT(indicesOffset < numIndices);
            const uint32_t startIndex = indices[indicesOffset];
            for (uint32_t faceIndex = 0; faceIndex < faceCount - 2; faceIndex++)
            {
                uint32_t index1 = indicesOffset + faceIndex + 1;
                uint32_t index2 = indicesOffset + faceIndex + 2;
                CARB_ASSERT(index1 < numIndices);
                CARB_ASSERT(index2 < numIndices);

                triangles.push_back(startIndex);
                triangles.push_back(indices[index1]);
                triangles.push_back(indices[index2]);
            }
            indicesOffset += faceCount;
        }

        return triangles;
    }

    bool createConformingTetrahedralMesh(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
        const VtArray<GfVec3f>& srcTriMeshPoints, const VtArray<int>& srcTriMeshIndices,
        VtArray<GfVec3f>& dstTetMeshPoints, VtArray<int>& dstTetMeshIndices)
    {
        ResultBuffer<carb::Float3> dstTetPointsR;
        ResultBuffer<uint32_t> dstTetIndicesR;
        bool success = physxCooking.computeConformingTetrahedralMesh(
            dstTetPointsR.ptr, dstTetPointsR.size, dstTetIndicesR.ptr, dstTetIndicesR.size,
            (carb::Float3*)srcTriMeshPoints.data(), uint32_t(srcTriMeshPoints.size()),
            (uint32_t*)srcTriMeshIndices.data(), uint32_t(srcTriMeshIndices.size()),
            ResultBuffer<>::allocate);
        if (success)
        {
            dstTetMeshPoints.assign((GfVec3f*)dstTetPointsR.ptr, (GfVec3f*)dstTetPointsR.ptr + dstTetPointsR.size);
            dstTetMeshIndices.assign((int*)dstTetIndicesR.ptr, (int*)dstTetIndicesR.ptr + dstTetIndicesR.size);
        }
        return success;
    }

    bool createConformingTetrahedralMeshPrim(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
        const SdfPath& srcMeshPath, const SdfPath& destTetMeshPath)
    {
        UsdGeomMesh geomMesh = UsdGeomMesh::Get(stage, srcMeshPath);
        VtArray<GfVec3f> srcPoints;
        geomMesh.GetPointsAttr().Get(&srcPoints);
        VtArray<int> srcIndices = ExtractTriangulatedFaces(geomMesh);

        VtArray<GfVec3f> dstPoints;
        VtArray<int> dstIndices;
        bool result = createConformingTetrahedralMesh(stage, physxCooking, srcPoints, srcIndices, dstPoints, dstIndices);
        VtArray<GfVec4i> dstTetVertexIndices = convertToVec4iArray(dstIndices);
        UsdGeomTetMesh tetMesh = UsdGeomTetMesh::Define(stage, destTetMeshPath);

        tetMesh.CreatePointsAttr().Set(dstPoints);
        tetMesh.CreateTetVertexIndicesAttr().Set(dstTetVertexIndices);

        return result;
    }

    bool createVoxelTetrahedralMesh(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
        const VtArray<GfVec3f>& srcTetMeshPoints, const VtArray<int>& srcTetMeshIndices,
        VtArray<GfVec3f>& dstTetMeshPoints, VtArray<int>& dstTetMeshIndices)
    {
        ResultBuffer<carb::Float3> dstTetPointsR;
        ResultBuffer<uint32_t> dstTetIndicesR;
        ResultBuffer<int32_t> dstEmbeddingR;
        bool success = physxCooking.computeVoxelTetrahedralMesh(
            dstTetPointsR.ptr, dstTetPointsR.size, dstTetIndicesR.ptr, dstTetIndicesR.size,
            dstEmbeddingR.ptr, dstEmbeddingR.size,
            (carb::Float3*)srcTetMeshPoints.data(), uint32_t(srcTetMeshPoints.size()),
            (uint32_t*)srcTetMeshIndices.data(), uint32_t(srcTetMeshIndices.size()),
            { 1.0f, 1.0f, 1.0f }, 0, ResultBuffer<>::allocate);
        if (success)
        {
            dstTetMeshPoints.assign((GfVec3f*)dstTetPointsR.ptr, (GfVec3f*)dstTetPointsR.ptr + dstTetPointsR.size);
            dstTetMeshIndices.assign((int*)dstTetIndicesR.ptr, (int*)dstTetIndicesR.ptr + dstTetIndicesR.size);
        }
        return success;
    }

    bool createVoxelTetrahedralMeshPrim(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
        const SdfPath& srcTetMeshPath, const SdfPath& destTetMeshPath)
    {
        UsdGeomTetMesh srcTetMesh = UsdGeomTetMesh::Get(stage, srcTetMeshPath);
        VtArray<GfVec3f> srcPoints;
        VtArray<GfVec4i> srcTetVertexIndices;

        srcTetMesh.GetPointsAttr().Get(&srcPoints);
        srcTetMesh.GetTetVertexIndicesAttr().Get(&srcTetVertexIndices);

        VtArray<int> srcIndices = convertToIntArray(srcTetVertexIndices);

        VtArray<GfVec3f> dstPoints;
        VtArray<int> dstIndices;
        bool result = createVoxelTetrahedralMesh(stage, physxCooking, srcPoints, srcIndices, dstPoints, dstIndices);
        VtArray<GfVec4i> dstTetIndices = convertToVec4iArray(dstIndices);
        UsdGeomTetMesh tetMesh = UsdGeomTetMesh::Define(stage, destTetMeshPath);
        tetMesh.CreatePointsAttr().Set(dstPoints);
        tetMesh.CreateTetVertexIndicesAttr().Set(dstTetIndices);
        return result;
    }

    bool createVolumeDeformableHierarchicalBase(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking, const SdfPath& xformPath,
        const SdfPath& skinMeshPath, const SdfPath& simMeshPath, VtArray<GfVec3f>& conformingPoints, VtArray<GfVec4i>& collMeshIndices)
    {
        // Create Xform
        createXformForDeformable(stage, xformPath);

        // Create skin mesh
        UsdGeomMesh skinMesh = createMeshBox(stage, skinMeshPath, { 50, 50, 50 });
        {
            UsdPrim skinMeshPrim = skinMesh.GetPrim();

            VtArray<GfVec3f> meshPoints;
            skinMesh.CreatePointsAttr().Get(&meshPoints);

            setBindPoseAttrs(meshPoints, skinMeshPrim);

            VtIntArray faceVertexCounts = { 4, 4, 4, 4, 4, 4 };
            skinMesh.GetFaceVertexCountsAttr().Set(faceVertexCounts);
            VtIntArray faceVertexIndices = { 0, 1, 3, 2, 4, 5, 7, 6, 10, 11, 13, 12, 14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20 };
            skinMesh.GetFaceVertexIndicesAttr().Set(faceVertexIndices);

            GfVec3f translate(0, 0, 0);
            GfQuatf orient(1, GfVec3f(0, 0, 0));
            GfVec3f scale(1, 1, 1);
            setXformOps(skinMesh, translate, orient, scale);
        }

        VtArray<GfVec3f> srcMeshPoints;
        VtArray<int> srcMeshIndices;
        skinMesh.GetPointsAttr().Get(&srcMeshPoints);
        srcMeshIndices = ExtractTriangulatedFaces(skinMesh);

        VtArray<int> conformingIndices;
        if (!createConformingTetrahedralMesh(stage, physxCooking, srcMeshPoints, srcMeshIndices, conformingPoints, conformingIndices))
        {
            return false;
        }
        collMeshIndices = convertToVec4iArray(conformingIndices);

        VtArray<GfVec3f> voxelPoints;
        VtArray<int> voxelIndices;
        if (!createVoxelTetrahedralMesh(stage, physxCooking, conformingPoints, conformingIndices, voxelPoints, voxelIndices))
        {
            return false;
        }
        VtArray<GfVec4i> simMeshIndices = convertToVec4iArray(voxelIndices);

        // Create sim mesh
        {
            UsdGeomTetMesh simMesh = UsdGeomTetMesh::Define(stage, simMeshPath);
            UsdPrim simMeshPrim = simMesh.GetPrim();
            simMeshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);

            simMesh.CreatePointsAttr().Set(voxelPoints);
            simMesh.CreateTetVertexIndicesAttr().Set(simMeshIndices);

            setBindPoseAttrs(voxelPoints, simMeshPrim);

            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(voxelPoints);

            simMeshPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices).Set(simMeshIndices);

            VtArray<GfVec3f> velocities(voxelPoints.size(), GfVec3f(0.0f));
            simMesh.CreateVelocitiesAttr().Set(velocities);
        }

        return true;
    }

    bool createVolumeDeformableHierarchicalHex(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking, const SdfPath& xformPath,
        const SdfPath& skinMeshPath, const SdfPath& simMeshPath, const SdfPath& collMeshPath)
    {
        VtArray<GfVec3f> conformingPoints;
        VtArray<GfVec4i> collMeshIndices;
        if (!createVolumeDeformableHierarchicalBase(stage, physxCooking, xformPath, skinMeshPath, simMeshPath, conformingPoints, collMeshIndices))
            return false;

        // Create collision mesh
        {
            UsdGeomTetMesh collMesh = UsdGeomTetMesh::Define(stage, collMeshPath);
            UsdPrim collMeshPrim = collMesh.GetPrim();
            UsdPhysicsCollisionAPI::Apply(collMeshPrim);

            collMesh.CreatePointsAttr().Set(conformingPoints);
            collMesh.CreateTetVertexIndicesAttr().Set(collMeshIndices);

            setBindPoseAttrs(conformingPoints, collMeshPrim);

            VtArray<GfVec3i> surfaceFaceVertexIndices;
            UsdGeomTetMesh::ComputeSurfaceFaces(collMesh, &surfaceFaceVertexIndices, UsdTimeCode::Default());
            collMesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices);
        }

        return true;
    }

    bool createVolumeDeformableHierarchicalNonHex(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking, const SdfPath& xformPath,
        const SdfPath& skinMeshPath, const SdfPath& simMeshPath)
    {
        VtArray<GfVec3f> conformingPoints;
        VtArray<GfVec4i> collMeshIndices;
        if (!createVolumeDeformableHierarchicalBase(stage, physxCooking, xformPath, skinMeshPath, simMeshPath, conformingPoints, collMeshIndices))
            return false;

        UsdGeomTetMesh simMesh = UsdGeomTetMesh::Get(stage, simMeshPath);
        UsdPhysicsCollisionAPI::Apply(simMesh.GetPrim());

        VtArray<GfVec3i> surfaceFaceVertexIndices;
        UsdGeomTetMesh::ComputeSurfaceFaces(simMesh, &surfaceFaceVertexIndices, UsdTimeCode::Default());
        simMesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices);

        return true;
    }

    bool createVolumeDeformableNonHierarchical(UsdStageRefPtr& stage, const IPhysxCooking& physxCooking,
        const SdfPath& deformablePath)
    {
        UsdGeomTetMesh mesh = UsdGeomTetMesh::Define(stage, deformablePath);
        UsdPrim prim = mesh.GetPrim();

        prim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        prim.ApplyAPI(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI);
        UsdPhysicsCollisionAPI::Apply(prim);

        VtArray<GfVec3f> meshPoints = { GfVec3f(0.0f, 0.0f, 0.0f), GfVec3f(1.0f, 0.0f, 0.0f), GfVec3f(0.0f, 1.0f, 0.0f), GfVec3f(0.0f, 0.0f, 1.0f), GfVec3f(1.0f, 1.0f, 1.0f) };
        VtArray<GfVec4i> tetVertexIndices = { GfVec4i(0, 1, 2, 3),  GfVec4i(1, 2, 3, 4) };

        setBindPoseAttrs(meshPoints, prim);

        mesh.CreatePointsAttr().Set(meshPoints);
        mesh.CreateTetVertexIndicesAttr().Set(tetVertexIndices);

        GfVec3f translate(0, 0, 0);
        GfQuatf orient(1, GfVec3f(0, 0, 0));
        GfVec3f scale(1, 1, 1);
        setXformOps(mesh, translate, orient, scale);

        prim.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(meshPoints);
        prim.GetAttribute(OmniPhysicsDeformableAttrTokens->restTetVtxIndices).Set(tetVertexIndices);

        VtArray<GfVec3f> velocities(meshPoints.size(), GfVec3f(0.0f));
        mesh.CreateVelocitiesAttr().Set(velocities);

        VtArray<GfVec3i> surfaceFaceVertexIndices;
        UsdGeomTetMesh::ComputeSurfaceFaces(mesh, &surfaceFaceVertexIndices, UsdTimeCode::Default());
        mesh.GetSurfaceFaceVertexIndicesAttr().Set(surfaceFaceVertexIndices);

        return true;
    }

}

struct DeformableTestParams
{
    UsdStageRefPtr stage;
    SdfPath defaultPrimPath;
    IPhysxCooking* physxCooking;
    IPhysxSimulation* physxSim;
    SdfPath physicsScenePath;
    float stepSize;

    enum class Type
    {
        eSURFACE,
        eVOLUME
    };

    Type type = Type::eSURFACE;
    bool modifiedRestShape = false;

    SdfPath bodyPath;
    SdfPath simPath;
    SdfPath collPath;
    SdfPath skinPath;
};

void step(DeformableTestParams& tp, uint32_t numSteps)
{
    for (PxU32 u = 0; u < numSteps; u++)
    {
        tp.physxSim->simulate(tp.stepSize, 0.0f);
    }
    tp.physxSim->fetchResults();
}

void checkFalling(DeformableTestParams& tp, PxDeformableBody& body)
{
    PxVec3 startPosition = body.getWorldBounds().getCenter();
    step(tp, 10);
    PxVec3 endPosition = body.getWorldBounds().getCenter();
    CHECK_LT(endPosition.z, startPosition.z);
}

void checkUpdatePosition(DeformableTestParams& tp, PxDeformableBody& body)
{
    UsdGeomPointBased simMesh = UsdGeomPointBased::Get(tp.stage, tp.simPath);
    PxVec3 startPosition = body.getWorldBounds().getCenter();

    VtArray<GfVec3f> positions;
    simMesh.GetPointsAttr().Get(&positions);
    for (unsigned int i = 0; i < positions.size(); ++i)
    {
        positions[i][2] += 1.0f;
    }
    simMesh.GetPointsAttr().Set(positions);

    step(tp, 1);

    PxVec3 endPosition = body.getWorldBounds().getCenter();
    CHECK_GT(endPosition.z, startPosition.z);
}

void checkUpdateVelocity(DeformableTestParams& tp, PxDeformableBody& body)
{
    UsdGeomPointBased simMesh = UsdGeomPointBased::Get(tp.stage, tp.simPath);
    PxVec3 startPosition = body.getWorldBounds().getCenter();

    VtArray<GfVec3f> velocities;
    simMesh.GetVelocitiesAttr().Get(&velocities);
    for (unsigned int i = 0; i < velocities.size(); ++i)
    {
        velocities[i][2] = 100.0f;
    }
    simMesh.GetVelocitiesAttr().Set(velocities);

    //seems like velocity update only has effect on bounds comp after two steps
    step(tp, 2);

    PxVec3 endPosition = body.getWorldBounds().getCenter();
    CHECK_GT(endPosition.z, startPosition.z);
}

void checkRestShape(DeformableTestParams& tp, PxDeformableBody& body)
{
    UsdGeomPointBased simMesh = UsdGeomPointBased::Get(tp.stage, tp.simPath);

    VtArray<GfVec3f> points;
    simMesh.GetPointsAttr().Get(&points);

    if (tp.type == DeformableTestParams::Type::eSURFACE)
    {
        // restTriVtxIndices = { GfVec3i(0, 1, 2), GfVec3i(0, 2, 3) };
        // expect V-shape on points
        GfVec3i tri0(0, 1, 2);
        GfVec3i tri1(0, 2, 3);
        GfVec3f n0 = GfCross(points[tri0[1]] - points[tri0[0]], points[tri0[2]] - points[tri0[0]]);
        GfVec3f n1 = GfCross(points[tri1[1]] - points[tri1[0]], points[tri1[2]] - points[tri1[0]]);
        GfNormalize(&n0);
        GfNormalize(&n1);
        double angleDegree = GfAbs(GfRadiansToDegrees(std::acos(GfClamp(GfDot(n0, n1), -1.0f, 1.0f))));

        TfToken defaultToken;
        simMesh.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->restBendAnglesDefault).Get(&defaultToken);
        if (defaultToken == OmniPhysicsDeformableAttrTokens->flatDefault || !tp.modifiedRestShape)
        {
            CHECK_LT(angleDegree, 1.0);
        }
        else if (defaultToken == OmniPhysicsDeformableAttrTokens->restShapeDefault)
        {
            CHECK_GT(angleDegree, 10.0);
        }
        else
        {
            CHECK(false);
        }
    }
    else if (tp.type == DeformableTestParams::Type::eVOLUME)
    {
        PxVec3 dims = body.getWorldBounds().getDimensions();
        if (tp.modifiedRestShape)
        {
            CHECK_GT(dims.x / dims.y, 1.01f);
        }
        else
        {
            CHECK_LT(dims.x / dims.y, 1.0001f);
        }
    }
}

PxDeformableBody* checkPhysxDeformable(DeformableTestParams& tp)
{
    PxScene* pxScene = getPhysxSceneAtPathChecked(tp.physicsScenePath);

    PxDeformableBody* body = nullptr;
    if (tp.type == DeformableTestParams::Type::eSURFACE)
    {
        // sanity check
        PxU32 nbDeformableSurfaces = pxScene->getNbDeformableSurfaces();
        CHECK_EQ(1u, nbDeformableSurfaces);

        PxDeformableSurface* surfaceDeformable =
            getPhysxBaseDerivedFromPathChecked<PxDeformableSurface>(tp.bodyPath, PhysXType::ePTDeformableSurface);

        body = surfaceDeformable;
    }
    else if (tp.type == DeformableTestParams::Type::eVOLUME)
    {
        // sanity check
        PxU32 nbVolumeDeformables = pxScene->getNbDeformableVolumes();
        CHECK_EQ(1u, nbVolumeDeformables);

        PxDeformableVolume* volumeDeformable =
            getPhysxBaseDerivedFromPathChecked<PxDeformableVolume>(tp.bodyPath, PhysXType::ePTDeformableVolume);

        body = volumeDeformable;
    }
    REQUIRE(body);
    return body;
}

void disableGravity(DeformableTestParams& tp)
{
    UsdPhysicsScene scene(tp.stage->GetPrimAtPath(tp.physicsScenePath));
    REQUIRE(scene);
    scene.GetGravityMagnitudeAttr().Set(0.0f);
}

void applySurfaceMaterialForRestShapeTest(DeformableTestParams& tp)
{
    SdfPath matPath = tp.defaultPrimPath.AppendChild(TfToken("surfaceDeformableMaterial"));
    UsdShadeMaterial mat = UsdShadeMaterial::Define(tp.stage, matPath);
    REQUIRE(mat);
    mat.GetPrim().ApplyAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableMaterialAPI);
    mat.GetPrim().ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);
    mat.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->surfaceThickness).Set(1.0f);
    mat.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->surfaceBendStiffness).Set(10000.0f);
    UsdPrim bodyPrim = tp.stage->GetPrimAtPath(tp.bodyPath);
    UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(bodyPrim);
    UsdRelationship rel = bodyPrim.CreateRelationship(TfToken("material:binding:physics"), false);
    rel.SetTargets({ matPath });
}

void modifySurfaceRestShape(DeformableTestParams& tp)
{
    UsdPrim simMesh = tp.stage->GetPrimAtPath(tp.simPath);
    REQUIRE((simMesh && simMesh.HasAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableSimAPI)));
    VtArray<GfVec3f> restShapePoints;
    simMesh.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Get(&restShapePoints);
    // restTriVtxIndices = { GfVec3i(0, 1, 2), GfVec3i(0, 2, 3) };
    // move vertices off the plane to form V-shape
    restShapePoints[1] += GfVec3f(10.0, 0, 0.0f);
    restShapePoints[3] += GfVec3f(10.0, 0, 0.0f);
    restShapePoints[0] -= GfVec3f(10.0, 0, 0.0f);
    restShapePoints[2] -= GfVec3f(10.0, 0, 0.0f);
    simMesh.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(restShapePoints);

    tp.modifiedRestShape = true;
}

void modifyVolumeRestShape(DeformableTestParams& tp)
{
    // stretch out sim mesh in x direction
    UsdPrim simMesh = tp.stage->GetPrimAtPath(tp.simPath);
    REQUIRE((simMesh && simMesh.HasAPI(OmniPhysicsDeformableAPITokens->VolumeDeformableSimAPI)));
    VtArray<GfVec3f> restShapePoints;
    simMesh.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Get(&restShapePoints);
    for (size_t i = 0; i < restShapePoints.size(); ++i)
    {
        restShapePoints[i][0] *= 1.1f;
    }
    simMesh.GetAttribute(OmniPhysicsDeformableAttrTokens->restShapePoints).Set(restShapePoints);

    tp.modifiedRestShape = true;
}

void testPreAttachSetup(DeformableTestParams& tp)
{
    SUBCASE("Initial Velocity")
    {
        disableGravity(tp);
        UsdGeomPointBased simMesh(tp.stage->GetPrimAtPath(tp.simPath));
        VtArray<GfVec3f> initPoints;
        VtArray<GfVec3f> initVelocities;
        simMesh.GetPointsAttr().Get(&initPoints);
        simMesh.GetVelocitiesAttr().Get(&initVelocities);
        for (size_t i = 0; i < initVelocities.size(); ++i)
        {
            initVelocities[i] = GfVec3f(100.0f, 0.0f, 0.0f);
        }
        simMesh.GetVelocitiesAttr().Set(initVelocities);

        long stageId = UsdUtilsStageCache::Get().GetId(tp.stage).ToLongInt();
        tp.physxSim->attachStage(stageId);

        step(tp, 1);

        VtArray<GfVec3f> points;
        VtArray<GfVec3f> velocities;
        simMesh.GetPointsAttr().Get(&points);
        simMesh.GetVelocitiesAttr().Get(&velocities);

        CHECK_EQ(velocities.size(), points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            float posErr = ((points[i] - initPoints[i]) - initVelocities[i] * tp.stepSize).GetLength();
            float velErr = (velocities[i] - initVelocities[i]).GetLength();
            CHECK_LT(posErr, 0.05f);
            CHECK_LT(velErr, 0.01f);
        }

        tp.physxSim->detachStage();
    }

    SUBCASE("Initial Velocity None")
    {
        disableGravity(tp);
        UsdGeomPointBased simMesh(tp.stage->GetPrimAtPath(tp.simPath));
        VtArray<GfVec3f> initPoints;
        VtArray<GfVec3f> initVelocities;
        simMesh.GetPointsAttr().Get(&initPoints);
        simMesh.GetVelocitiesAttr().Set(initVelocities);

        long stageId = UsdUtilsStageCache::Get().GetId(tp.stage).ToLongInt();
        tp.physxSim->attachStage(stageId);

        step(tp, 1);

        VtArray<GfVec3f> points;
        VtArray<GfVec3f> velocities;
        simMesh.GetPointsAttr().Get(&points);
        simMesh.GetVelocitiesAttr().Get(&velocities);

        CHECK_EQ(velocities.size(), points.size());
        for (size_t i = 0; i < points.size(); ++i)
        {
            float posErr = (points[i] - initPoints[i]).GetLength();
            float velErr = velocities[i].GetLength();
            CHECK_LT(posErr, 0.05f);
            CHECK_LT(velErr, 1e-4f);
        }

        tp.physxSim->detachStage();
    }

    SUBCASE("Modified RestShape")
    {
        disableGravity(tp);

        if (tp.type == DeformableTestParams::Type::eSURFACE)
        {
            applySurfaceMaterialForRestShapeTest(tp);

            modifySurfaceRestShape(tp);

            UsdPrim simMesh = tp.stage->GetPrimAtPath(tp.simPath);
            UsdAttribute defaultAttr = simMesh.GetAttribute(OmniPhysicsDeformableAttrTokens->restBendAnglesDefault);
            SUBCASE("Angle Flat Default")
            {
                defaultAttr.Set(OmniPhysicsDeformableAttrTokens->flatDefault);
            }

            SUBCASE("Angle Rest Shape Default")
            {
                defaultAttr.Set(OmniPhysicsDeformableAttrTokens->restShapeDefault);
            }
        }
        else if (tp.type == DeformableTestParams::Type::eVOLUME)
        {
            modifyVolumeRestShape(tp);
        }

        long stageId = UsdUtilsStageCache::Get().GetId(tp.stage).ToLongInt();
        tp.physxSim->attachStage(stageId);

        PxDeformableBody* body = checkPhysxDeformable(tp);

        step(tp, 10);
        checkRestShape(tp, *body);

        tp.physxSim->detachStage();
    }

    SUBCASE("No Pre Attach Setup")
    {
        long stageId = UsdUtilsStageCache::Get().GetId(tp.stage).ToLongInt();
        tp.physxSim->attachStage(stageId);

        PxDeformableBody* body = checkPhysxDeformable(tp);

        SUBCASE("Default RestShape")
        {
            step(tp, 10);
            checkRestShape(tp, *body);
        }

        SUBCASE("Falling")
        {
            checkFalling(tp, *body);
        }

        SUBCASE("Update Position")
        {
            disableGravity(tp);
            checkUpdatePosition(tp, *body);
        }

        SUBCASE("Update Velocity")
        {
            disableGravity(tp);
            checkUpdateVelocity(tp, *body);
        }

        tp.physxSim->detachStage();
    }


}

void testTypes(DeformableTestParams& tp)
{
    SUBCASE("Surface Deformable")
    {
        tp.type = DeformableTestParams::Type::eSURFACE;

        if (tp.bodyPath != tp.simPath)
        {
            createSurfaceDeformableHierarchical(tp.stage, tp.bodyPath, tp.skinPath, tp.simPath);
        }
        else
        {
            createSurfaceDeformableNonHierarchical(tp.stage, tp.bodyPath);
        }

        testPreAttachSetup(tp);
    }

    SUBCASE("Volume Deformable")
    {
        tp.type = DeformableTestParams::Type::eVOLUME;

        if (tp.bodyPath != tp.simPath)
        {
            SUBCASE("Hex")
            {
                tp.collPath = tp.bodyPath.AppendChild(TfToken("collMesh"));
                REQUIRE(createVolumeDeformableHierarchicalHex(
                    tp.stage, *tp.physxCooking, tp.bodyPath, tp.skinPath, tp.simPath, tp.collPath));

                testPreAttachSetup(tp);
            }

            SUBCASE("Non-Hex")
            {
                REQUIRE(createVolumeDeformableHierarchicalNonHex(
                    tp.stage, *tp.physxCooking, tp.bodyPath, tp.skinPath, tp.simPath));

                testPreAttachSetup(tp);
            }
        }
        else
        {
            REQUIRE(createVolumeDeformableNonHierarchical(tp.stage, *tp.physxCooking, tp.bodyPath));

            testPreAttachSetup(tp);
        }
    }
}

void testDeformableBodies(DeformableTestParams& tp)
{
    SUBCASE("Hierarchical")
    {
        tp.bodyPath = tp.defaultPrimPath.AppendChild(TfToken("Xform"));
        tp.simPath = tp.bodyPath.AppendChild(TfToken("simMesh"));
        tp.collPath = tp.collPath;
        tp.skinPath = tp.bodyPath.AppendChild(TfToken("skinMesh"));

        testTypes(tp);
    }

    SUBCASE("Non Hierarchical")
    {
        tp.bodyPath = tp.defaultPrimPath.AppendChild(TfToken("simMesh"));
        tp.simPath = tp.bodyPath;
        tp.collPath = tp.collPath;

        testTypes(tp);
    }
}

//-----------------------------------------------------------------------------
// Init test
TEST_CASE("Deformable Tests",
    "[omniphysics]"
    "[component=OmniPhysics][owner=snie][priority=mandatory]")
{
    // constants for setup
    const GfVec3f gravityDirection(0.0f, 0.0f, -1.0f); // use z-up
    const float metersPerStageUnit = 0.01f; // work in default centimeters
    const float kilogramsPerStageUnit = 1.0f;
    const float gravityMagnitude = 10.0f / metersPerStageUnit;
    const float density = 1000.f * metersPerStageUnit * metersPerStageUnit * metersPerStageUnit / kilogramsPerStageUnit;

    // setup common to all subcases
    PhysicsTest& physicsTests = *PhysicsTest::getPhysicsTests();
    IPhysx* physx = physicsTests.acquirePhysxInterface();
    REQUIRE(physx);
    IPhysxCooking* physxCooking = physicsTests.acquirePhysxCookingInterface();
    REQUIRE(physxCooking);
    IPhysxSimulation* physxSim = physicsTests.acquirePhysxSimulationInterface();
    REQUIRE(physxSim);

    UsdStageRefPtr stage = UsdStage::CreateInMemory();
    
    UsdGeomSetStageUpAxis(stage, TfToken("Z"));
    const SdfPath defaultPrimPath = SdfPath("/World");
    UsdPrim defaultPrim = stage->DefinePrim(defaultPrimPath);

    UsdUtilsStageCache::Get().Insert(stage);
    long stageId = UsdUtilsStageCache::Get().GetId(stage).ToLongInt();

    const SdfPath physicsScenePath = defaultPrimPath.AppendChild(TfToken("physicsScene"));
    UsdPhysicsScene scene = UsdPhysicsScene::Define(stage, physicsScenePath);
    scene.GetGravityMagnitudeAttr().Set(gravityMagnitude);
    scene.GetGravityDirectionAttr().Set(gravityDirection);

    SUBCASE("USD API Regression Test")
    {
        // GetAttribute(...).Get(...) is not well documented and there might be no guarantee that the output variable is not modified when Get() fails because the attribute is not present
        // Check that Get() performs as expected
        // The same regression test is implemented as Python unit test test_regression_usd_api(self)

        static TfToken tokenBool("regressionTestBool");
        static TfToken tokenBoolBad("regressionTestBoolBad");

        bool checkBool = true;
        defaultPrim.CreateAttribute(tokenBool, SdfValueTypeNames->Bool).Set(checkBool);

        checkBool = false;
        defaultPrim.GetAttribute(tokenBool).Get(&checkBool);
        CHECK_EQ(checkBool, true);

        // Check both bool states (true and false)
        checkBool = false;
        defaultPrim.GetAttribute(tokenBoolBad).Get(&checkBool);
        CHECK_EQ(checkBool, false);

        checkBool = true;
        defaultPrim.GetAttribute(tokenBoolBad).Get(&checkBool);
        CHECK_EQ(checkBool, true);

        static TfToken tokenFloat("regressionTestFloat");
        static TfToken tokenFloatBad("regressionTestFloatBad");

        float value = PX_MAX_F32;
        defaultPrim.CreateAttribute(tokenFloat, SdfValueTypeNames->Float).Set(value);

        value = 0.0f;
        defaultPrim.GetAttribute(tokenFloat).Get(&value);
        CHECK_EQ(value, PX_MAX_F32);

        // Check 0.0f, -PX_MAX_F32 and PX_MAX_F32
        value = 0.0f;
        defaultPrim.GetAttribute(tokenFloatBad).Get(&value);
        CHECK_EQ(value, 0.0f);

        value = -PX_MAX_F32;
        defaultPrim.GetAttribute(tokenFloatBad).Get(&value);
        CHECK_EQ(value, -PX_MAX_F32);

        value = PX_MAX_F32;
        defaultPrim.GetAttribute(tokenFloatBad).Get(&value);
        CHECK_EQ(value, PX_MAX_F32);
    }

    SUBCASE("Conforming and Voxel TetrahedralMeshes")
    {
        const SdfPath boxMeshPath = defaultPrimPath.AppendChild(TfToken("boxMesh"));
        const SdfPath boxTetMeshConformingPath = defaultPrimPath.AppendChild(TfToken("boxTetMeshConforming"));
        const SdfPath boxTetMeshVoxelPath = defaultPrimPath.AppendChild(TfToken("boxTetMeshVoxel"));

        // Create box mesh
        createMeshBox(stage, boxMeshPath, { 50, 50, 50 });

        REQUIRE(createConformingTetrahedralMeshPrim(stage, *physxCooking, boxMeshPath, boxTetMeshConformingPath));
        REQUIRE(createVoxelTetrahedralMeshPrim(stage, *physxCooking, boxTetMeshConformingPath, boxTetMeshVoxelPath));
    }

#if USE_PHYSX_GPU
    SUBCASE("Deformable Body Material")
    {
        const SdfPath physicsMaterialPath = defaultPrimPath.AppendChild(TfToken("PhysicsMaterial"));

        UsdShadeMaterial material = UsdShadeMaterial::Define(stage, physicsMaterialPath);
        UsdPrim materialPrim = material.GetPrim();
        materialPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);
        materialPrim.ApplyAPI(PhysxAdditionAPITokens->DeformableMaterialAPI);

        materialPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->youngsModulus).Set(5000.0f);
        materialPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->poissonsRatio).Set(0.045f);
        materialPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->dynamicFriction).Set(0.025f);
        materialPrim.GetAttribute(PhysxAdditionAttrTokens->elasticityDamping).Set(0.0005f);

        // parse
        physxSim->attachStage(stageId);

        // sanity check
        PxDeformableVolumeMaterial* volumeDeformableMaterial = getPhysxBaseDerivedFromPathChecked<PxDeformableVolumeMaterial>(physicsMaterialPath, PhysXType::ePTDeformableVolumeMaterial);

        // Check user material
        CHECK_EQ(5000.0f, volumeDeformableMaterial->getYoungsModulus());
        CHECK_EQ(0.045f, volumeDeformableMaterial->getPoissons());
        CHECK_EQ(0.025f, volumeDeformableMaterial->getDynamicFriction());
        CHECK_EQ(0.0005f, volumeDeformableMaterial->getElasticityDamping());

        // update params:
        materialPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->youngsModulus).Set(15000.0f);
        materialPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->poissonsRatio).Set(0.145f);
        materialPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->dynamicFriction).Set(0.125f);
        materialPrim.GetAttribute(PhysxAdditionAttrTokens->elasticityDamping).Set(0.0015f);

        // step
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        CHECK_EQ(15000.0f, volumeDeformableMaterial->getYoungsModulus());
        CHECK_EQ(0.145f, volumeDeformableMaterial->getPoissons());
        CHECK_EQ(0.125f, volumeDeformableMaterial->getDynamicFriction());
        CHECK_EQ(0.0015f, volumeDeformableMaterial->getElasticityDamping());

        physxSim->detachStage();
    }

    SUBCASE("Material Precedence")
    {
        //for (bool useBody : { false, true })
        bool useBody = false;
        {
            // Create deformable
            const SdfPath xformPath = defaultPrimPath.AppendChild(TfToken("Xform"));
            const SdfPath skinMeshPath = xformPath.AppendChild(TfToken("skinMesh"));
            const SdfPath simMeshPath = xformPath.AppendChild(TfToken("simMesh"));
            
            if (useBody)
            {
                const SdfPath collMeshPath = xformPath.AppendChild(TfToken("collMesh"));
                REQUIRE(createVolumeDeformableHierarchicalHex(stage, *physxCooking, xformPath, skinMeshPath, simMeshPath, collMeshPath));
            }
            else
            {
                createSurfaceDeformableHierarchical(stage, xformPath, skinMeshPath, simMeshPath);
            }

            UsdPrim xformPrim = stage->GetPrimAtPath(xformPath);

            // Create four deformable materials
            SdfPath deformableMaterialPath[4];
            UsdShadeMaterial shadeMaterial[4];
            for (uint32_t i = 0; i < 4; ++i)
            {
                std::string name(std::string("deformableMaterial_") + std::to_string(i));
                deformableMaterialPath[i] = defaultPrimPath.AppendChild(TfToken(name.c_str()));
                shadeMaterial[i] = UsdShadeMaterial::Define(stage, deformableMaterialPath[i]);
                UsdPrim materialPrim = shadeMaterial[i].GetPrim();
                if (useBody)
                {
                    materialPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);
                }
                else
                {
                    materialPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->SurfaceDeformableMaterialAPI);
                    materialPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);
                }
            }

            // The following SUBCASE structure makes sure we are going through assinging materials
            // at ever increasing precedence.
            uint32_t caseCounter = 0;
            SUBCASE("Default Material") {}
            SUBCASE("Scene Material Binding")
            {
                UsdShadeMaterialBindingAPI sceneBindAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
                sceneBindAPI.Bind(shadeMaterial[0], UsdShadeTokens->fallbackStrength, TfToken("physics"));
                caseCounter++;
                SUBCASE("No Further Bindings") {}
                SUBCASE("Scene Physics Material Binding")
                {
                    UsdRelationship sceneRel =
                        scene.GetPrim().CreateRelationship(TfToken("material:binding:physics"), false);
                    sceneRel.SetTargets(SdfPathVector({ deformableMaterialPath[1] }));
                    caseCounter++;
                    SUBCASE("No Further Bindings") {}
                    SUBCASE("Deformable Material Binding")
                    {
                        UsdShadeMaterialBindingAPI deformableBindAPI = UsdShadeMaterialBindingAPI::Apply(xformPrim);
                        deformableBindAPI.Bind(shadeMaterial[2], UsdShadeTokens->fallbackStrength, TfToken("physics"));
                        caseCounter++;
                        SUBCASE("No Further Bindings") {}
                        SUBCASE("Deformable Physics Material Binding")
                        {
                            UsdRelationship deformableRel = xformPrim.CreateRelationship(TfToken("material:binding:physics"), false);
                            deformableRel.SetTargets({ deformableMaterialPath[3] });
                            caseCounter++;
                        }
                    }
                }
            }

            physxSim->attachStage(stageId);

            // Materials should be there
            PxBase* baseMaterialPtrs[4];
            for (uint32_t i = 0; i < 4; ++i)
            {
                PhysXType materialType = useBody ? ePTDeformableVolumeMaterial : ePTDeformableSurfaceMaterial;
                baseMaterialPtrs[i] = reinterpret_cast<PxBase*>(physx->getPhysXPtr(deformableMaterialPath[i], materialType));
                CHECK(baseMaterialPtrs[i] != nullptr);
                bool isRightType = useBody ? baseMaterialPtrs[i]->is<PxDeformableVolumeMaterial>() != nullptr :
                    baseMaterialPtrs[i]->is<PxDeformableSurfaceMaterial>() != nullptr;
                CHECK(isRightType);
            }

            // Deformable should be there
            PhysXType deformableType = useBody ? ePTDeformableVolume : ePTDeformableSurface;
            PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(xformPath, deformableType));
            CHECK(basePtr != nullptr);
            bool isRightType = useBody ? basePtr->is<PxDeformableVolume>() != nullptr : basePtr->is<PxDeformableSurface>() != nullptr;
            CHECK(isRightType);

            // Check material bindings
            PxDeformableMaterial* deformableMaterial = nullptr;
            if (useBody)
            {
                PxDeformableVolumeMaterial* volumeDeformableMaterial;
                PxShape* shape = basePtr->is<PxDeformableVolume>()->getShape();
                REQUIRE(shape->getNbMaterials() == 1);
                PxU32 numMaterials = shape->getDeformableVolumeMaterials(&volumeDeformableMaterial, 1);
                REQUIRE(numMaterials == 1);
                deformableMaterial = volumeDeformableMaterial;
            }
            else
            {
                PxDeformableSurfaceMaterial* deformableSurfaceMaterial;
                PxShape* shape = basePtr->is<PxDeformableSurface>()->getShape();
                REQUIRE(shape->getNbMaterials() == 1);
                REQUIRE(shape->getDeformableSurfaceMaterials(&deformableSurfaceMaterial, 1) == 1);
                deformableMaterial = deformableSurfaceMaterial;
            }

            if (caseCounter > 0)
            {
                REQUIRE(deformableMaterial == baseMaterialPtrs[caseCounter - 1]);
            }

            physxSim->detachStage();
        }
    }

    SUBCASE("Material Density")
    {
        // Create base physics and deformable body materials
        const SdfPath basePhysicsMaterialPath = defaultPrimPath.AppendChild(TfToken("basePhysicsMaterial"));
        UsdShadeMaterial basePhysicsMaterial = UsdShadeMaterial::Define(stage, basePhysicsMaterialPath);
        basePhysicsMaterial.GetPrim().ApplyAPI(OmniPhysicsDeformableAPITokens->DeformableMaterialAPI);

        SUBCASE("DefaultScene")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(scene.GetPrim());
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));
        }

        // Create deformable body
        const SdfPath xformPath = defaultPrimPath.AppendChild(TfToken("Xform"));
        const SdfPath skinMeshPath = xformPath.AppendChild(TfToken("skinMesh"));
        const SdfPath simMeshPath = xformPath.AppendChild(TfToken("simMesh"));
        const SdfPath collMeshPath = xformPath.AppendChild(TfToken("collMesh"));

        REQUIRE(createVolumeDeformableHierarchicalHex(stage, *physxCooking, xformPath, skinMeshPath, simMeshPath, collMeshPath));

        UsdPrim xformPrim = stage->GetPrimAtPath(xformPath);

        SUBCASE("BoxPrim")
        {
            UsdShadeMaterialBindingAPI bindingAPI = UsdShadeMaterialBindingAPI::Apply(xformPrim);
            bindingAPI.Bind(basePhysicsMaterial, UsdShadeTokens->weakerThanDescendants, TfToken("physics"));
        }

        basePhysicsMaterial.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->density).Set(1000.0f);

        physxSim->attachStage(stageId);

        // material should be there
        PxBase* baseMaterialPtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(basePhysicsMaterialPath, ePTDeformableVolumeMaterial));
        CHECK(baseMaterialPtr != nullptr);
        CHECK(baseMaterialPtr->is<PxDeformableVolumeMaterial>());

        // deformable body should be there
        PxBase* basePtr = reinterpret_cast<PxBase*>(physx->getPhysXPtr(xformPath, ePTDeformableVolume));
        CHECK(basePtr != nullptr);
        CHECK(basePtr->is<PxDeformableVolume>());

        PxDeformableVolume* volumeDeformable = basePtr->is<PxDeformableVolume>();
        PxShape* shape = volumeDeformable->getShape();
        REQUIRE(shape->getNbMaterials() == 1);
        PxDeformableVolumeMaterial* material = nullptr;
        shape->getDeformableVolumeMaterials(&material, 1);
        CHECK(material->userData != nullptr);
        CHECK(material == baseMaterialPtr->is<PxDeformableVolumeMaterial>());

        // Workaround for forcing a sync to get volumeDeformable->getSimPositionInvMassBufferD updated.
        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        PxCudaContextManager* cudaContextManager = volumeDeformable->getCudaContextManager();
        PxScopedCudaLock _lock(*cudaContextManager);

        PxCudaContext* cudaContext = cudaContextManager->getCudaContext();

        // only allocate one PxVec4 to get the mass.
        PxVec4* positionInvGM = PX_PINNED_HOST_ALLOC_T(PxVec4, cudaContextManager, 1);
        cudaContext->memcpyDtoH(positionInvGM, reinterpret_cast<CUdeviceptr>(volumeDeformable->getSimPositionInvMassBufferD()), sizeof(PxVec4));

        float initialMass = 1.0f / positionInvGM[0].w;

        // Change material density
        basePhysicsMaterial.GetPrim().GetAttribute(OmniPhysicsDeformableAttrTokens->density).Set(100.0f);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // Check mass
        cudaContext->memcpyDtoH(positionInvGM, reinterpret_cast<CUdeviceptr>(volumeDeformable->getSimPositionInvMassBufferD()), sizeof(PxVec4));
        float finalMass = 1.0f / positionInvGM[0].w;

        CHECK_LT(finalMass, initialMass);   // Changed density of material that was not assigned to deformable body

        // Change mass instead of density
        xformPrim.GetAttribute(OmniPhysicsDeformableAttrTokens->mass).Set(1000.0f);

        physxSim->simulate(0.017f, 0.0f);
        physxSim->fetchResults();

        // Check mass
        cudaContext->memcpyDtoH(positionInvGM, reinterpret_cast<CUdeviceptr>(volumeDeformable->getSimPositionInvMassBufferD()), sizeof(PxVec4));
        float mass = 1.0f / positionInvGM[0].w;

        CHECK_LT(mass, finalMass);          // Changed mass of material that was not assigned to deformable body

        PX_PINNED_HOST_FREE(cudaContextManager, positionInvGM);

        physxSim->detachStage();
    }

    SUBCASE("Deformable Bodies")
    {
        DeformableTestParams tp;
        tp.stage = stage;
        tp.defaultPrimPath = defaultPrimPath;
        tp.physxCooking = physxCooking;
        tp.physxSim = physxSim;
        tp.physicsScenePath = physicsScenePath;
        tp.stepSize = 0.017f;
        
        testDeformableBodies(tp);
    }

#endif // USE_PHYSX_GPU

    UsdUtilsStageCache::Get().Erase(stage);
    stage = nullptr;
}
