// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <omni/Span.h>

#include <omni/physx/MeshKey.h>
#include <omni/physx/PhysxCookingParams.h>
#include <private/omni/physx/IPhysxCookingServicePrivate.h> // PhysxCookingDataVersion_* and PhysxCookingMeshView


namespace omni
{
namespace physx
{
struct MeshKeyComputation
{
    static omni::physx::usdparser::MeshKey computeMeshKey(const PhysxCookingMeshView& meshView)
    {
        omni::physx::usdparser::MeshKey meshKey;
        meshKey.computeVerticesHash(static_cast<uint32_t>(meshView.points.size()), &meshView.points.data()->x);
        meshKey.computeIndicesHash(
            static_cast<uint32_t>(meshView.indices.size()), reinterpret_cast<const uint32_t*>(meshView.indices.data()));
        meshKey.computeFacesHash(
            static_cast<uint32_t>(meshView.faces.size()), reinterpret_cast<const uint32_t*>(meshView.faces.data()));
        meshKey.computeFaceHolesHash(static_cast<uint32_t>(meshView.holeIndices.size()),
                                     reinterpret_cast<const uint32_t*>(meshView.holeIndices.data()));
        meshKey.computeFaceMaterialsHash(
            static_cast<uint32_t>(meshView.faceMaterials.size()), meshView.faceMaterials.data());
        return meshKey;
    }

    static omni::physx::usdparser::MeshKey computeMeshKey(const PhysxCookingDeformableBodyView& deformableBodyView)
    {
        omni::physx::usdparser::MeshKey meshKey;
        meshKey.computeVerticesHash(static_cast<uint32_t>(deformableBodyView.srcPointsInSim.size()), &deformableBodyView.srcPointsInSim.data()->x);
        return meshKey;
    }

    static omni::physx::usdparser::MeshKey computeMeshKey(const PhysxCookingDeformableVolumeMeshView& volumeMeshView)
    {
        omni::physx::usdparser::MeshKey meshKey;
        meshKey.computeVerticesHash(static_cast<uint32_t>(volumeMeshView.simPoints.size()), &volumeMeshView.simPoints.data()->x);
        meshKey.computeVerticesHash(static_cast<uint32_t>(volumeMeshView.simBindPoints.size()), &volumeMeshView.simBindPoints.data()->x);
        meshKey.computeIndicesHash(static_cast<uint32_t>(volumeMeshView.simIndices.size() * 4), reinterpret_cast<const uint32_t*>(volumeMeshView.simIndices.data()));
        if (!volumeMeshView.collBindPointsInSim.empty() && !volumeMeshView.collIndices.empty() && !volumeMeshView.collSurfaceIndices.empty())
        {
            meshKey.computeVerticesHash(static_cast<uint32_t>(volumeMeshView.collBindPointsInSim.size()), &volumeMeshView.collBindPointsInSim.data()->x);
            meshKey.computeIndicesHash(static_cast<uint32_t>(volumeMeshView.collIndices.size() * 4), reinterpret_cast<const uint32_t*>(volumeMeshView.collIndices.data()));
            meshKey.computeIndicesHash(static_cast<uint32_t>(volumeMeshView.collSurfaceIndices.size() * 3), reinterpret_cast<const uint32_t*>(volumeMeshView.collSurfaceIndices.data()));
        }
        return meshKey;
    }
};

struct MeshCRCComputation
{
    using MeshKey = omni::physx::usdparser::MeshKey;
    static MeshKey deriveConvexMeshCRC(MeshKey meshKey, const ConvexMeshCookingParams& params)
    {
        meshKey.setMinThickness(params.minThickness);
        meshKey.setMaxHullVertices(params.maxHullVertices);

        meshKey.setSignScale(&params.signScale.x);
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_ConvexMesh);
        return meshKey;
    }

    static MeshKey deriveConvexDecompositionCRC(MeshKey meshKey, const ConvexDecompositionCookingParams& params)
    {
        meshKey.setMinThickness(params.minThickness);
        meshKey.setMaxHullCount(params.maxConvexHulls);
        meshKey.setMaxHullVertices(params.maxHullVertices);
        meshKey.setVoxelResolution(params.voxelResolution);
        meshKey.setErrorPercentage(params.errorPercentage);
        meshKey.setUseShrinkwrap(params.shrinkWrap);

        meshKey.setSignScale(&params.signScale.x);
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_ConvexDecomposition);
        return meshKey;
    }

    static MeshKey deriveSphereFillCRC(MeshKey meshKey, const SphereFillCookingParams& params)
    {
        meshKey.setMaxSpheres(params.maxSpheres);
        meshKey.setSeedCount(params.seedCount);
        meshKey.setVoxelResolution(params.voxelResolution);
        meshKey.setFillMode(params.fillMode);

        meshKey.setSignScale(&params.signScale.x);
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_SphereFill);
        return meshKey;
    }

    static MeshKey deriveTriangleMeshCRC(MeshKey meshKey,
                                         const TriangleMeshCookingParams& triangleMeshCookingParams,
                                         const SdfMeshCookingParams& sdfMeshCookingParams)
    {
        switch (triangleMeshCookingParams.mode)
        {
        case TriangleMeshMode::eORIGINAL_TRIANGLES:
            meshKey = MeshCRCComputation::deriveTriMeshCRC(meshKey, triangleMeshCookingParams);
            break;
        case TriangleMeshMode::eQUADRIC_SIMPLIFICATION:
            meshKey = MeshCRCComputation::deriveMeshSimplificationCRC(meshKey, triangleMeshCookingParams);
            break;
        }

        if (sdfMeshCookingParams.sdfResolution > 0)
        {
            meshKey = MeshCRCComputation::deriveSdfMeshCRC(meshKey, triangleMeshCookingParams, sdfMeshCookingParams);
            meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_TriangleMesh);
            meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_TriangleMeshSDF);
        }
        else
        {
            meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_TriangleMesh);
        }
        return meshKey;
    }

    static MeshKey deriveParticlePoissonSamplingCRC(MeshKey meshKey,
                                                    const ParticlePoissonSamplingCookingParams& params)
    {
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_ParticlePoissonSampling);
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.shearScale), sizeof(params.shearScale));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.samplingDistance), sizeof(params.samplingDistance));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.maxSamples), sizeof(params.maxSamples));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.sampleVolume), sizeof(params.sampleVolume));
        return meshKey;
    }

    static MeshKey computeDeformableVolumeMeshCRC(const DeformableVolumeMeshCookingParams& params)
    {
        MeshKey meshKey = MeshKey();

        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_DeformableVolumeMesh);
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simPoints.data()),
            sizeof(carb::Float3) * params.simPoints.size());
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simBindPoints.data()),
            sizeof(carb::Float3) * params.simBindPoints.size());
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simIndices.data()),
            sizeof(carb::Int4) * params.simIndices.size());
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.collBindPointsInSim.data()),
            sizeof(carb::Float3) * params.collBindPointsInSim.size());
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.collIndices.data()),
            sizeof(carb::Int4) * params.collIndices.size());
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simToCookingTransform),
            sizeof(params.simToCookingTransform));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.numTetsPerElement),
            sizeof(params.numTetsPerElement));

        return meshKey;
    }

    static MeshKey deriveVolumeDeformableBodyCRC(MeshKey meshKey,
                                                 const VolumeDeformableBodyCookingParams& params)
    {
        // src positions are already factored into the meshKey
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_VolumeDeformableBody);

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simToCookingTransform),
                            sizeof(params.simToCookingTransform));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simToCollTransform),
                            sizeof(params.simToCollTransform));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.isAutoMeshSimplificationEnabled),
                            sizeof(params.isAutoMeshSimplificationEnabled));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.isAutoRemeshingEnabled),
                            sizeof(params.isAutoRemeshingEnabled));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.hasAutoForceConforming),
                            sizeof(params.hasAutoForceConforming));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.isAutoHexahedralMeshEnabled),
                            sizeof(params.isAutoHexahedralMeshEnabled));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.autoRemeshingResolution),
                            sizeof(params.autoRemeshingResolution));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.autoTriangleTargetCount),
                            sizeof(params.autoTriangleTargetCount));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.autoHexahedralResolution),
                            sizeof(params.autoHexahedralResolution));
        return meshKey;
    }

    static MeshKey deriveSurfaceDeformableBodyCRC(MeshKey meshKey,
                                                  const SurfaceDeformableBodyCookingParams& params)
    {
        // src positions are already factored into the meshKey
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_SurfaceDeformableBody);

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.simToCookingTransform),
                            sizeof(params.simToCookingTransform));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.isAutoMeshSimplificationEnabled),
                            sizeof(params.isAutoMeshSimplificationEnabled));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.isAutoRemeshingEnabled),
                            sizeof(params.isAutoRemeshingEnabled));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.autoRemeshingResolution),
                            sizeof(params.autoRemeshingResolution));

        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.autoTriangleTargetCount),
                            sizeof(params.autoTriangleTargetCount));

        return meshKey;
    }

private:
    static MeshKey deriveMeshSimplificationCRC(MeshKey meshKey, const TriangleMeshCookingParams& params)
    {
        meshKey.setSimplificationMetric(params.simplificationMetric);
        meshKey.setWeldTolerance(params.meshWeldTolerance);
        meshKey.setTriangleMeshMode(uint32_t(TriangleMeshMode::eQUADRIC_SIMPLIFICATION));

        return meshKey;
    }

    static MeshKey deriveTriMeshCRC(MeshKey meshKey, const TriangleMeshCookingParams& params)
    {
        meshKey.setWeldTolerance(params.meshWeldTolerance);

        return meshKey;
    }

    static MeshKey deriveSdfMeshCRC(MeshKey meshKey,
                                    const TriangleMeshCookingParams& triangleMeshCookingParams,
                                    const SdfMeshCookingParams& sdfParams)
    {
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&sdfParams.sdfResolution), sizeof(sdfParams.sdfResolution));
        meshKey.setMiscData(
            reinterpret_cast<const uint8_t*>(&sdfParams.sdfSubgridResolution), sizeof(sdfParams.sdfSubgridResolution));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&sdfParams.sdfBitsPerSubgridPixel),
                            sizeof(sdfParams.sdfBitsPerSubgridPixel));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&sdfParams.sdfNarrowBandThickness),
                            sizeof(sdfParams.sdfNarrowBandThickness));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&sdfParams.sdfMargin), sizeof(sdfParams.sdfMargin));
        meshKey.setMiscData(
            reinterpret_cast<const uint8_t*>(&sdfParams.sdfEnableRemeshing), sizeof(sdfParams.sdfEnableRemeshing));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&sdfParams.sdfTriangleCountReductionFactor),
                            sizeof(sdfParams.sdfTriangleCountReductionFactor));

        return meshKey;
    }
};
} // namespace physx
} // namespace omni
