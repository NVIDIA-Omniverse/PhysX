// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

    static MeshKey deriveDeformableBodyTetMeshCRCDeprecated(MeshKey meshKey,
                                                            const DeformableBodyTetMeshCookingParamsDeprecated& params)
    {
        //skin mesh, including params.skinEarliestPoints is expected to be part of meshKey
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_DeformableBodyTetMeshDeprecated);
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.normalizedQuantizedScale),
                            sizeof(params.normalizedQuantizedScale));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simpParams.enabled),
                            sizeof(params.simpParams.enabled));
        if (params.simpParams.enabled)
        {
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simpParams.remeshing),
                                sizeof(params.simpParams.remeshing));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simpParams.remeshingResolution),
                                sizeof(params.simpParams.remeshingResolution));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simpParams.targetTriangleCount),
                                sizeof(params.simpParams.targetTriangleCount));
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simpParams.forceConforming),
                                sizeof(params.simpParams.forceConforming));
        }
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simulationResolution), sizeof(params.simulationResolution));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.kinematicEnabled), sizeof(params.kinematicEnabled));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simulationNumTetsPerVoxel), sizeof(params.simulationNumTetsPerVoxel));
        return meshKey;
    }

    static MeshKey computeSoftBodyMeshCRCDeprecated(const omni::physx::SoftBodyMeshCookingParamsDeprecated& params)
    {
        MeshKey meshKey = MeshKey();

        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_SoftBodyMeshDeprecated);
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.softBodyToWorldScaleNormalized),
                            sizeof(params.softBodyToWorldScaleNormalized));
        meshKey.computeIndicesHash(uint32_t(params.collisionIndices.size()), params.collisionIndices.data());
        meshKey.computeVerticesHash(uint32_t(params.collisionRestPoints.size()),
                                    reinterpret_cast<const float*>(params.collisionRestPoints.data()));
        meshKey.computeIndicesHash(uint32_t(params.simulationIndices.size()), params.simulationIndices.data());
        meshKey.computeVerticesHash(uint32_t(params.simulationRestPoints.size()),
                                    reinterpret_cast<const float*>(params.simulationRestPoints.data()));
        if (params.restPoints.size() > 0)
        {
            meshKey.computeVerticesHash(uint32_t(params.restPoints.size()),
                                        reinterpret_cast<const float*>(params.restPoints.data()));
        }
        if (params.collisionVertexToSimulationTetIndices.size() > 0)
        {
            meshKey.setMiscData(reinterpret_cast<const uint8_t*>(params.collisionVertexToSimulationTetIndices.data()),
                                sizeof(uint32_t) * params.collisionVertexToSimulationTetIndices.size());
        }
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simulationResolution),
                            sizeof(params.simulationResolution));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.simulationNumTetsPerVoxel),
                            sizeof(params.simulationNumTetsPerVoxel));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.kinematicEnabled),
                            sizeof(params.kinematicEnabled));
        return meshKey;
    }

    static MeshKey deriveParticleClothMeshCRCDeprecated(MeshKey meshKey,
                                                        const ParticleClothMeshCookingParamsDeprecated& params)
    {
        //rest positions are already factored into the meshKey
        meshKey.setCookedDataVersion(omni::physx::PhysxCookingDataVersion_ParticleClothDeprecated);
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.needsSprings), sizeof(params.needsSprings));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.isInflatable), sizeof(params.isInflatable));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.enableWelding), sizeof(params.enableWelding));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.springStretchStiffness), sizeof(params.springStretchStiffness));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.springBendStiffness), sizeof(params.springBendStiffness));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.springShearStiffness), sizeof(params.springShearStiffness));
        meshKey.setMiscData(reinterpret_cast<const uint8_t*>(&params.springDamping), sizeof(params.springDamping));
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
