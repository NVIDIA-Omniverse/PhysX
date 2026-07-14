// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include <omni/physx/IPhysxCooking.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h>

static const PXR_NS::TfToken poseInstanceToken("default");

namespace deformableutility
{
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

void setXformOps(const PXR_NS::UsdGeomXformable& xformable,
                 const PXR_NS::GfVec3f& translate,
                 const PXR_NS::GfQuatf& orient,
                 const PXR_NS::GfVec3f& scale);
void createXformForDeformable(const PXR_NS::UsdStageRefPtr& stage, const PXR_NS::SdfPath& xformPath);
void setBindPoseAttrs(const PXR_NS::VtArray<PXR_NS::GfVec3f>& meshPoints, PXR_NS::UsdPrim& simMeshPrim);
PXR_NS::VtArray<int> ExtractTriangulatedFaces(PXR_NS::UsdGeomMesh const& usdMesh);
PXR_NS::VtArray<PXR_NS::GfVec4i> convertToVec4iArray(const PXR_NS::VtArray<int>& input);
bool createVoxelTetrahedralMesh(PXR_NS::UsdStageRefPtr& stage,
                                const omni::physx::IPhysxCooking& physxCooking,
                                const PXR_NS::VtArray<PXR_NS::GfVec3f>& srcTetMeshPoints,
                                const PXR_NS::VtArray<int>& srcTetMeshIndices,
                                PXR_NS::VtArray<PXR_NS::GfVec3f>& dstTetMeshPoints,
                                PXR_NS::VtArray<int>& dstTetMeshIndices);
bool createConformingTetrahedralMesh(PXR_NS::UsdStageRefPtr& stage,
                                     const omni::physx::IPhysxCooking& physxCooking,
                                     const PXR_NS::VtArray<PXR_NS::GfVec3f>& srcTriMeshPoints,
                                     const PXR_NS::VtArray<int>& srcTriMeshIndices,
                                     PXR_NS::VtArray<PXR_NS::GfVec3f>& dstTetMeshPoints,
                                     PXR_NS::VtArray<int>& dstTetMeshIndices);
bool createVolumeDeformableHierarchicalBase(PXR_NS::UsdStageRefPtr& stage,
                                            const omni::physx::IPhysxCooking& physxCooking,
                                            const PXR_NS::SdfPath& xformPath,
                                            const PXR_NS::SdfPath& skinMeshPath,
                                            const PXR_NS::SdfPath& simMeshPath,
                                            PXR_NS::VtArray<PXR_NS::GfVec3f>& conformingPoints,
                                            PXR_NS::VtArray<PXR_NS::GfVec4i>& collMeshIndices);
bool createVolumeDeformableHierarchicalHex(PXR_NS::UsdStageRefPtr& stage,
                                           const omni::physx::IPhysxCooking& physxCooking,
                                           const PXR_NS::SdfPath& xformPath,
                                           const PXR_NS::SdfPath& skinMeshPath,
                                           const PXR_NS::SdfPath& simMeshPath,
                                           const PXR_NS::SdfPath& collMeshPath);
void createSurfaceDeformableHierarchical(const PXR_NS::UsdStageRefPtr& stage,
                                         const PXR_NS::SdfPath& xformPath,
                                         const PXR_NS::SdfPath& skinMeshPath,
                                         const PXR_NS::SdfPath& simMeshPath);
PXR_NS::UsdAttribute getPosePurposesAttr(PXR_NS::UsdPrim posePrim, PXR_NS::TfToken instanceName);
PXR_NS::TfToken getPoseNameFromPurpose(const PXR_NS::UsdPrim prim, const PXR_NS::TfToken posePurposeToken);
PXR_NS::UsdAttribute getPosePointsAttr(PXR_NS::UsdPrim posePrim, const PXR_NS::TfType& poseType, PXR_NS::TfToken instanceName);
PXR_NS::UsdAttribute getPosePointsOrPointsAttr(PXR_NS::UsdPrim posePrim, const PXR_NS::TfType& poseType, PXR_NS::TfToken instanceName);
PXR_NS::GfVec3d computeQuantizedDir(const PXR_NS::GfVec3d& dir);
PXR_NS::GfRotation computeQuantizedRotation(const PXR_NS::GfRotation& rotation);
PXR_NS::GfTransform computeQuantizedSkewTransform(double& scaleAbs, const PXR_NS::GfTransform& transformSkew);
void computeFitBounds(PXR_NS::GfVec3d& translation,
                      double& scale,
                      const PXR_NS::VtArray<PXR_NS::GfVec3f>& points,
                      const PXR_NS::GfTransform& transform);
bool computeDeformableCookingTransform(PXR_NS::GfMatrix4d* simToCookingTransform,
                                       PXR_NS::GfMatrix4d* cookingToWorldTransform,
                                       double* cookingToWorldScale,
                                       const PXR_NS::GfMatrix4d& simToWorld,
                                       const PXR_NS::VtArray<PXR_NS::GfVec3f>& boundsFitPoints);
void setupVolumeDeformableBodyCookingParams(omni::physx::VolumeDeformableBodyCookingParams& params,
                                            PXR_NS::VtArray<PXR_NS::GfVec3f>& srcPointsInSimData,
                                            const PXR_NS::UsdStageRefPtr& stage,
                                            const omni::physx::PhysxCookingComputeRequest& request,
                                            const PXR_NS::SdfPath& skinMeshPath);
void setupSurfaceDeformableBodyCookingParams(omni::physx::SurfaceDeformableBodyCookingParams& params,
                                             PXR_NS::VtArray<PXR_NS::GfVec3f>& srcPointsInSimData,
                                             const PXR_NS::UsdStageRefPtr& stage,
                                             const omni::physx::PhysxCookingComputeRequest& request,
                                             const PXR_NS::SdfPath& skinMeshPath);
} // namespace deformableutility
