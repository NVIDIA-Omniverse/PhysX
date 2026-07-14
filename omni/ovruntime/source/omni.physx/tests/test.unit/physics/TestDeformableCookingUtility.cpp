// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <common/utilities/Utilities.h>
#include "../common/TestHelpers.h"
#include "TestDeformableCookingUtility.h"

using namespace PXR_NS;
using namespace omni::physx;

namespace deformableutility
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
    TfToken pointsAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, poseInstanceToken);
    TfToken purposesAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, poseInstanceToken);

    simMeshPrim.ApplyAPI(OmniPhysicsDeformableAPITokens->DeformablePoseAPI, poseInstanceToken);

    simMeshPrim.GetAttribute(pointsAttrName).Set(meshPoints);

    VtTokenArray tokens = { TfToken("bindPose") };
    simMeshPrim.GetAttribute(purposesAttrName).Set(tokens);
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

bool createVoxelTetrahedralMesh(UsdStageRefPtr& stage,
                                const IPhysxCooking& physxCooking,
                                const VtArray<GfVec3f>& srcTetMeshPoints,
                                const VtArray<int>& srcTetMeshIndices,
                                VtArray<GfVec3f>& dstTetMeshPoints,
                                VtArray<int>& dstTetMeshIndices)
{
    ResultBuffer<carb::Float3> dstTetPointsR;
    ResultBuffer<uint32_t> dstTetIndicesR;
    ResultBuffer<int32_t> dstEmbeddingR;
    bool success = physxCooking.computeVoxelTetrahedralMesh(
        dstTetPointsR.ptr, dstTetPointsR.size, dstTetIndicesR.ptr, dstTetIndicesR.size, dstEmbeddingR.ptr,
        dstEmbeddingR.size, (carb::Float3*)srcTetMeshPoints.data(), uint32_t(srcTetMeshPoints.size()),
        (uint32_t*)srcTetMeshIndices.data(), uint32_t(srcTetMeshIndices.size()), { 1.0f, 1.0f, 1.0f }, 0,
        ResultBuffer<>::allocate);
    if (success)
    {
        dstTetMeshPoints.assign((GfVec3f*)dstTetPointsR.ptr, (GfVec3f*)dstTetPointsR.ptr + dstTetPointsR.size);
        dstTetMeshIndices.assign((int*)dstTetIndicesR.ptr, (int*)dstTetIndicesR.ptr + dstTetIndicesR.size);
    }
    return success;
}

bool createConformingTetrahedralMesh(UsdStageRefPtr& stage,
                                     const IPhysxCooking& physxCooking,
                                     const VtArray<GfVec3f>& srcTriMeshPoints,
                                     const VtArray<int>& srcTriMeshIndices,
                                     VtArray<GfVec3f>& dstTetMeshPoints,
                                     VtArray<int>& dstTetMeshIndices)
{
    ResultBuffer<carb::Float3> dstTetPointsR;
    ResultBuffer<uint32_t> dstTetIndicesR;
    bool success = physxCooking.computeConformingTetrahedralMesh(
        dstTetPointsR.ptr, dstTetPointsR.size, dstTetIndicesR.ptr, dstTetIndicesR.size,
        (carb::Float3*)srcTriMeshPoints.data(), uint32_t(srcTriMeshPoints.size()), (uint32_t*)srcTriMeshIndices.data(),
        uint32_t(srcTriMeshIndices.size()), ResultBuffer<>::allocate);
    if (success)
    {
        dstTetMeshPoints.assign((GfVec3f*)dstTetPointsR.ptr, (GfVec3f*)dstTetPointsR.ptr + dstTetPointsR.size);
        dstTetMeshIndices.assign((int*)dstTetIndicesR.ptr, (int*)dstTetIndicesR.ptr + dstTetIndicesR.size);
    }
    return success;
}

bool createVolumeDeformableHierarchicalBase(UsdStageRefPtr& stage,
                                            const IPhysxCooking& physxCooking,
                                            const SdfPath& xformPath,
                                            const SdfPath& skinMeshPath,
                                            const SdfPath& simMeshPath,
                                            VtArray<GfVec3f>& conformingPoints,
                                            VtArray<GfVec4i>& collMeshIndices)
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
        VtIntArray faceVertexIndices = { 0,  1,  3, 2, 4,  5,  7,  6,  10, 11, 13, 12,
                                         14, 15, 9, 8, 17, 23, 21, 19, 22, 16, 18, 20 };
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
    if (!createConformingTetrahedralMesh(
            stage, physxCooking, srcMeshPoints, srcMeshIndices, conformingPoints, conformingIndices))
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

bool createVolumeDeformableHierarchicalHex(UsdStageRefPtr& stage,
                                           const IPhysxCooking& physxCooking,
                                           const SdfPath& xformPath,
                                           const SdfPath& skinMeshPath,
                                           const SdfPath& simMeshPath,
                                           const SdfPath& collMeshPath)
{
    VtArray<GfVec3f> conformingPoints;
    VtArray<GfVec4i> collMeshIndices;
    if (!createVolumeDeformableHierarchicalBase(
            stage, physxCooking, xformPath, skinMeshPath, simMeshPath, conformingPoints, collMeshIndices))
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

void createSurfaceDeformableHierarchical(const UsdStageRefPtr& stage,
                                         const SdfPath& xformPath,
                                         const SdfPath& skinMeshPath,
                                         const SdfPath& simMeshPath)
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

UsdAttribute getPosePurposesAttr(UsdPrim posePrim, TfToken instanceName)
{
    TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_purposes, instanceName);
    return posePrim.GetAttribute(attrName);
}

TfToken getPoseNameFromPurpose(const UsdPrim prim, const TfToken posePurposeToken)
{
    TfTokenVector allAPIs = prim.GetAppliedSchemas();

    TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
    TfToken poseTypeName = UsdSchemaRegistry::GetAPISchemaTypeName(poseType);

    for (const auto& api : allAPIs)
    {
        std::pair<TfToken, TfToken> typeNameAndInstance = UsdSchemaRegistry::GetTypeNameAndInstance(api);
        if (typeNameAndInstance.first == poseTypeName)
        {
            VtArray<TfToken> candTokens;
            getPosePurposesAttr(prim, typeNameAndInstance.second).Get(&candTokens);
            for (const TfToken candToken : candTokens)
            {
                if (candToken == posePurposeToken)
                {
                    return typeNameAndInstance.second;
                }
            }
        }
    }

    return TfToken();
}

UsdAttribute getPosePointsAttr(UsdPrim posePrim, const TfType& poseType, TfToken instanceName)
{
    if (instanceName.IsEmpty())
        return UsdAttribute();

    if (!posePrim.HasAPI(poseType, instanceName))
    {
        CARB_LOG_ERROR("Expected UsdPhysicsDeformablePoseAPI instance %s on %s, but not found.", instanceName.GetText(),
                       posePrim.GetPath().GetText());
        return UsdAttribute();
    }

    TfToken attrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
        OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, instanceName);
    return posePrim.GetAttribute(attrName);
}

UsdAttribute getPosePointsOrPointsAttr(UsdPrim posePrim, const TfType& poseType, TfToken instanceName)
{
    if (!instanceName.IsEmpty())
    {
        return getPosePointsAttr(posePrim, poseType, instanceName);
    }
    return UsdGeomPointBased(posePrim).GetPointsAttr();
}

GfVec3d computeQuantizedDir(const GfVec3d& dir)
{
    double eps = 1e-7;
    double eps_inv = 1.0 / eps;

    PXR_NS::GfVec3d dirQuant(
        std::round(dir[0] * eps_inv) * eps, std::round(dir[1] * eps_inv) * eps, std::round(dir[2] * eps_inv) * eps);

    return dirQuant.GetNormalized();
}

GfRotation computeQuantizedRotation(const GfRotation& rotation)
{
    double eps = 1e-7;
    double eps_inv = 1.0 / eps;

    double angleUnit = rotation.GetAngle() / 360.0;
    double angleUnitQuant = std::round(angleUnit * eps_inv) * eps;
    double angleQuant = angleUnitQuant * 360.0;

    return PXR_NS::GfRotation(computeQuantizedDir(rotation.GetAxis()), angleQuant);
}

GfTransform computeQuantizedSkewTransform(double& scaleAbs, const GfTransform& transformSkew)
{
    PXR_NS::GfVec3d scaleNormalized = transformSkew.GetScale();
    scaleAbs = scaleNormalized.Normalize();

    PXR_NS::GfTransform transformSkewQuant(PXR_NS::GfVec3d(0.0), computeQuantizedRotation(transformSkew.GetRotation()),
                                        computeQuantizedDir(scaleNormalized), PXR_NS::GfVec3d(0.0),
                                        computeQuantizedRotation(transformSkew.GetPivotOrientation()));

    return transformSkewQuant;
}

void computeFitBounds(PXR_NS::GfVec3d& translation,
                      double& scale,
                      const PXR_NS::VtArray<PXR_NS::GfVec3f>& points,
                      const PXR_NS::GfTransform& transform)
{
    PXR_NS::GfMatrix4d m = transform.GetMatrix();

    PXR_NS::GfRange3d bounds;
    for (const PXR_NS::GfVec3d& point : points)
    {
        bounds.UnionWith(m.Transform(point));
    }
    PXR_NS::GfVec3d dims = bounds.GetSize();
    double dimMax = std::max(std::max(dims[0], dims[1]), dims[2]);

    translation = bounds.GetMidpoint();
    scale = std::max(1e-7, dimMax);
}

bool computeDeformableCookingTransform(GfMatrix4d* simToCookingTransform,
                                       GfMatrix4d* cookingToWorldTransform,
                                       double* cookingToWorldScale,
                                       const GfMatrix4d& simToWorld,
                                       const VtArray<GfVec3f>& boundsFitPoints)
{
    PXR_NS::GfMatrix4d simToWorldOrtho = simToWorld;
    bool orthonormalized = simToWorldOrtho.Orthonormalize(false);

    if (!orthonormalized)
    {
        return false;
    }

    PXR_NS::GfMatrix4d simToWorldSkew = simToWorld * simToWorldOrtho.GetInverse();

    double scaleAbs;
    PXR_NS::GfTransform skew(simToWorldSkew);
    PXR_NS::GfTransform skewQuant = computeQuantizedSkewTransform(scaleAbs, skew);

    PXR_NS::GfVec3d fbTrans;
    double fbScale;
    computeFitBounds(fbTrans, fbScale, boundsFitPoints, skewQuant);

    PXR_NS::GfRotation pivotOrient = skewQuant.GetPivotOrientation();
    PXR_NS::GfRotation rotation = skewQuant.GetRotation();
    PXR_NS::GfVec3d scale = skewQuant.GetScale();

    if (simToCookingTransform)
    {
        PXR_NS::GfMatrix4d matFbScaleInv;
        matFbScaleInv.SetScale(1.0 / fbScale);
        PXR_NS::GfMatrix4d matFbTransInv;
        matFbTransInv.SetTranslate(-fbTrans);
        PXR_NS::GfMatrix4d matScale;
        matScale.SetScale(scale);
        PXR_NS::GfMatrix4d matOrientInv;
        matOrientInv.SetTransform(pivotOrient.GetInverse(), PXR_NS::GfVec3d(0.0));
        *simToCookingTransform = matOrientInv * matScale * matFbTransInv * matFbScaleInv;
    }

    if (cookingToWorldTransform)
    {
        // this is the corresponding transform from cooking space to world space (ignoring the pre scale factor:
        // scaleAbs*fbScale)
        PXR_NS::GfMatrix4d rigid;
        {
            PXR_NS::GfMatrix4d matRot(pivotOrient * rotation, PXR_NS::GfVec3d(0.0));
            PXR_NS::GfMatrix4d matTrans(PXR_NS::GfRotation(PXR_NS::GfVec3d(1, 0, 0), 0.0), fbTrans * scaleAbs);
            rigid = matTrans * matRot * simToWorldOrtho;
        }
        *cookingToWorldTransform = rigid;
    }

    if (cookingToWorldScale)
    {
        *cookingToWorldScale = scaleAbs * fbScale;
    }
    return true;
}

void setupVolumeDeformableBodyCookingParams(VolumeDeformableBodyCookingParams& params,
                                            VtArray<GfVec3f>& srcPointsInSimData,
                                            const UsdStageRefPtr& stage,
                                            const PhysxCookingComputeRequest& request,
                                            const SdfPath& skinMeshPath)
{
    const SdfPath simMeshPath = intToPath(request.deformablePathInfo.simMeshPrimId);
    const SdfPath collMeshPath = intToPath(request.deformablePathInfo.collMeshPrimId);
    UsdPrim simMeshPrim = stage->GetPrimAtPath(simMeshPath);
    UsdPrim collMeshPrim = stage->GetPrimAtPath(collMeshPath);
    PXR_NS::GfMatrix4d simToWorld =
        PXR_NS::UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
    PXR_NS::GfMatrix4d worldToSim = simToWorld.GetInverse();

    PXR_NS::GfMatrix4d simToColl;
    if (collMeshPrim != simMeshPrim)
    {
        PXR_NS::GfMatrix4d collToWorld =
            PXR_NS::UsdGeomXformable(collMeshPrim).ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
        PXR_NS::GfMatrix4d worldToColl = collToWorld.GetInverse();
        simToColl = simToWorld * worldToColl;
    }
    else
    {
        simToColl.SetIdentity();
    }

    UsdPrim srcMeshPrim = stage->GetPrimAtPath(skinMeshPath);
    PXR_NS::UsdGeomMesh srcMesh(srcMeshPrim);
    TfType poseType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
    PXR_NS::TfToken srcMeshBindPoseToken =
        getPoseNameFromPurpose(srcMesh.GetPrim(), OmniPhysicsDeformableAttrTokens->bindPose);
    UsdAttribute simBindPointsAttr = getPosePointsOrPointsAttr(srcMesh.GetPrim(), poseType, srcMeshBindPoseToken);
    if (simBindPointsAttr)
    {
        simBindPointsAttr.Get(&srcPointsInSimData);
    }

    // Transform skin points to sim mesh space
    PXR_NS::GfMatrix4d srcToWorld = srcMesh.ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
    PXR_NS::GfMatrix4d srcToSim = srcToWorld * worldToSim;
    for (size_t i = 0; i < srcPointsInSimData.size(); ++i)
    {
        PXR_NS::GfVec3f& srcPoint = srcPointsInSimData[i];
        srcPoint = PXR_NS::GfVec3f(srcToSim.Transform(srcPoint));
    }

    uint32_t srcPointCount = uint32_t(srcPointsInSimData.size());
    if (srcPointCount)
    {
        const carb::Float3* srcPoints = reinterpret_cast<const carb::Float3*>(&srcPointsInSimData[0]);
        params.srcPointsInSim = { srcPoints, srcPointCount };
    }

    GfMatrix4d simToCookingTransform;
    computeDeformableCookingTransform(&simToCookingTransform, nullptr, nullptr, simToWorld, srcPointsInSimData);

    static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
    memcpy(params.simToCookingTransform, simToCookingTransform.data(), sizeof(params.simToCookingTransform));

    static_assert(sizeof(params.simToCollTransform) == sizeof(simToColl));
    memcpy(params.simToCollTransform, simToColl.data(), sizeof(params.simToCollTransform));

    params.isAutoMeshSimplificationEnabled = true;
    params.isAutoRemeshingEnabled = true;
    params.hasAutoForceConforming = false;
    params.isAutoHexahedralMeshEnabled = true;
    params.autoRemeshingResolution = 0;
    params.autoTriangleTargetCount = 0;
    params.autoHexahedralResolution = 0;
}

void setupSurfaceDeformableBodyCookingParams(SurfaceDeformableBodyCookingParams& params,
                                             VtArray<GfVec3f>& srcPointsInSimData,
                                             const UsdStageRefPtr& stage,
                                             const PhysxCookingComputeRequest& request,
                                             const SdfPath& skinMeshPath)
{
    const SdfPath simMeshPath = intToPath(request.deformablePathInfo.simMeshPrimId);
    const SdfPath collMeshPath = intToPath(request.deformablePathInfo.collMeshPrimId);
    UsdPrim simMeshPrim = stage->GetPrimAtPath(simMeshPath);
    UsdPrim collMeshPrim = stage->GetPrimAtPath(collMeshPath);
    PXR_NS::GfMatrix4d simToWorld =
        PXR_NS::UsdGeomXformable(simMeshPrim).ComputeLocalToWorldTransform(PXR_NS::UsdTimeCode::Default());
    PXR_NS::GfMatrix4d worldToSim = simToWorld.GetInverse();

    UsdPrim srcMeshPrim = stage->GetPrimAtPath(skinMeshPath);
    PXR_NS::UsdGeomMesh srcMesh(srcMeshPrim);

    TfToken srcMeshBindPoseToken = getPoseNameFromPurpose(srcMesh.GetPrim(), OmniPhysicsDeformableAttrTokens->bindPose);
    TfType dpType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(OmniPhysicsDeformableAPITokens->DeformablePoseAPI);
    bool hasBindPoseAPI = !srcMeshBindPoseToken.IsEmpty() && srcMesh.GetPrim().HasAPI(dpType, srcMeshBindPoseToken);
    if (hasBindPoseAPI)
    {
        TfToken pointsAttrName = UsdSchemaRegistry::MakeMultipleApplyNameInstance(
            OmniPhysicsDeformableAttrTokens->multipleApplyTemplate_points, srcMeshBindPoseToken);
        srcMesh.GetPrim().GetAttribute(pointsAttrName).Get(&srcPointsInSimData);
    }
    else
    {
        srcMesh.GetPointsAttr().Get(&srcPointsInSimData);
    }

    // Transform src points to sim mesh space
    GfMatrix4d srcToWorld = srcMesh.ComputeLocalToWorldTransform(UsdTimeCode::Default());
    GfMatrix4d srcToSim = srcToWorld * worldToSim;
    for (size_t i = 0; i < srcPointsInSimData.size(); ++i)
    {
        GfVec3f& srcPoint = srcPointsInSimData[i];
        srcPoint = GfVec3f(srcToSim.Transform(srcPoint));
    }

    uint32_t srcPointCount = uint32_t(srcPointsInSimData.size());
    if (srcPointCount)
    {
        const carb::Float3* srcPoints = reinterpret_cast<const carb::Float3*>(&srcPointsInSimData[0]);
        params.srcPointsInSim = { srcPoints, srcPointCount };
    }

    GfMatrix4d simToCookingTransform;
    computeDeformableCookingTransform(&simToCookingTransform, nullptr, nullptr, simToWorld, srcPointsInSimData);

    static_assert(sizeof(params.simToCookingTransform) == sizeof(simToCookingTransform));
    memcpy(params.simToCookingTransform, simToCookingTransform.data(), sizeof(params.simToCookingTransform));

    params.isAutoMeshSimplificationEnabled = false;
    params.isAutoRemeshingEnabled = false;
    params.autoRemeshingResolution = 0;
    params.autoTriangleTargetCount = 0;
}
} // namespace deformableutility
