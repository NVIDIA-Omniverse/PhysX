// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"
#include <carb/logging/Log.h>
#include <PxPhysicsAPI.h>
#include <common/foundation/TypeCast.h>

#include <extensions/PxTetMakerExt.h>
#include <extensions/PxTetrahedronMeshExt.h>
#include <cooking/PxTetrahedronMeshDesc.h>
#include <extensions/PxRemeshingExt.h>
#include <extensions/PxSoftBodyExt.h>
#include "MeshSimplifyInternal.h"
using namespace ::physx;

namespace omni
{
namespace physx
{
    
    bool validateTetMesh(const PxArray<PxVec3>& tetMeshVertices, const PxArray<PxU32>& tetMeshIndices)
    {
        PxBoundedData vertices;
        vertices.data = tetMeshVertices.begin();
        vertices.count = uint32_t(tetMeshVertices.size());
        PxBoundedData tetrahedra;
        tetrahedra.data = tetMeshIndices.begin();
        tetrahedra.count = uint32_t(tetMeshIndices.size() / 4);
        PxTetrahedronMeshAnalysisResults meshAnalysis = PxTetMaker::validateTetrahedronMesh(vertices, tetrahedra);
        std::string message = "";
        if (meshAnalysis & PxTetrahedronMeshAnalysisResult::eDEGENERATE_TETRAHEDRON)
            message += "- Mesh contains tetrahedra with zero volume. This is not supported for softbodies. Consider using a skin mesh with triangles that all have approximately equal edge lengths.\n";

        if (meshAnalysis & PxTetrahedronMeshAnalysisResult::eMESH_IS_INVALID)
        {
            //Tetmesher cannot handle the mesh - it will still not crash and generate something but the result is probably not of much use
            message = "\nTetmesh contains features not supported by the softbody cooker:\n" + message;
            CARB_LOG_WARN("%s", message.c_str());
            return false;
        }

        return true;
    }
    bool validateSurfaceMesh(const PxSimpleTriangleMesh& surfaceMesh)
    {
        PxTriangleMeshAnalysisResults meshAnalysis = PxTetMaker::validateTriangleMesh(surfaceMesh);
        std::string message = "";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eZERO_VOLUME)
            message += "- Mesh has zero volume and is not supported by the tetmesher.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eINCONSISTENT_TRIANGLE_ORIENTATION)
            message += "- Mesh has inconsistently oriented triangles and is not supported by the tetmesher.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eCONTAINS_INVALID_POINTS)
            message += "- Mesh has points containing NaN or infinity and is not supported by the tetmesher.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eREQUIRES_32BIT_INDEX_BUFFER)
            message += "- Mesh has a 16bit index buffer but contains more points than the index buffer can address. This is not supported, use a 32bit index buffer instead.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eOPEN_BOUNDARIES)
            message += "- Mesh has open boundaries/holes. The tetmesher will fill the holes.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eSELF_INTERSECTIONS)
            message += "- Mesh has self-intersections. The tetmesher will not match the surface accurately at locations where the mesh intersects with itself.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eCONTAINS_ACUTE_ANGLED_TRIANGLES)
            message += "- Mesh has triangles with small angles (slivers). The vertices of the mesh might not be distributed uniformly across its surface.\n";
        if (meshAnalysis & PxTriangleMeshAnalysisResult::eEDGE_SHARED_BY_MORE_THAN_TWO_TRIANGLES)
            message += "- Mesh has non-manifold edges. The tetmesh will not match the surface accurately at those edges.\n";
        //if (meshAnalysis & physx::PxTriangleMeshAnalysisResult::eCONTAINS_DUPLICATE_POINTS)
        //    message += "- Mesh has duplicate points. The resulting tetmesh will only make use of first unique point that is found. This is usually not problematic.\n";

        if (meshAnalysis & PxTriangleMeshAnalysisResult::eMESH_IS_INVALID)
        {
            //Tetmesher cannot handle the mesh - it will still not crash and generate something but the result is probably not of much use
            message = "\nSurface mesh contains features not supported by the tetmesher:\n" + message;
            CARB_LOG_WARN("%s", message.c_str());
            return false;
        }
        else if (meshAnalysis & PxTriangleMeshAnalysisResult::eMESH_IS_PROBLEMATIC)
        {
            //Tetmesher can handle the mesh but might not generate a non-optimal result
            message = "\nSurface mesh might produce a non-optimal collison tetmesh:\n" + message;
            CARB_LOG_INFO("%s", message.c_str());
        }
        return true;
    }

    void simplifyTriangleMeshInternal(carb::Float3*& dstPoints, uint32_t& dstPointsSize, uint32_t*& dstIndices, uint32_t& dstIndicesSize,
        uint32_t** dstOrigToSimpVertexMap, uint32_t* dstOrigToSimpVertexMapSize, uint32_t** dstSimpToOrigTriMap, uint32_t* dstSimpToOrigTriMapSize,
        const carb::Float3* srcPoints, const uint32_t srcPointsSize, const uint32_t* srcIndices, const uint32_t srcIndicesSize,
        const uint32_t targetTriangleCount, const float maximalEdgeLength, const bool projectSimplifiedVerticesOntoInputMesh, const bool removeDisconnectedPatches,
        void* (*allocateBytes)(size_t))
    {
        PxArray<PxVec3> points((PxVec3*)srcPoints, (PxVec3*)srcPoints + srcPointsSize);
        PxArray<PxU32> indices(srcIndices, srcIndices + srcIndicesSize);

        PxArray<PxVec3> oPoints;
        PxArray<PxU32> oIndices;
        PxArray<PxU32> oSrcToDstVertexMap;
        PxArray<PxU32> oDstToSrcTriMap;
        PxTetMaker::simplifyTriangleMesh(points, indices, targetTriangleCount, maximalEdgeLength, oPoints, oIndices, dstOrigToSimpVertexMap ? &oSrcToDstVertexMap : NULL,
            0.1f, 0.01f, projectSimplifiedVerticesOntoInputMesh, dstSimpToOrigTriMap ? &oDstToSrcTriMap : NULL,  removeDisconnectedPatches);

        dstPointsSize = oPoints.size();
        dstPoints = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * dstPointsSize);
        memcpy(dstPoints, &oPoints[0], sizeof(carb::Float3) * dstPointsSize);

        dstIndicesSize = oIndices.size();
        dstIndices = (uint32_t*)allocateBytes(sizeof(uint32_t) * dstIndicesSize);
        memcpy(dstIndices, &oIndices[0], sizeof(uint32_t) * dstIndicesSize);

        if (dstOrigToSimpVertexMap && dstOrigToSimpVertexMapSize)
        {
            *dstOrigToSimpVertexMapSize = oSrcToDstVertexMap.size();
            *dstOrigToSimpVertexMap = (uint32_t*)allocateBytes(sizeof(uint32_t) * oSrcToDstVertexMap.size());
            memcpy(*dstOrigToSimpVertexMap, &oSrcToDstVertexMap[0], sizeof(uint32_t) * oSrcToDstVertexMap.size());
        }

        if (dstSimpToOrigTriMap && dstSimpToOrigTriMapSize)
        {
            *dstSimpToOrigTriMapSize = oDstToSrcTriMap.size();
            *dstSimpToOrigTriMap = (uint32_t*)allocateBytes(sizeof(uint32_t) * oDstToSrcTriMap.size());
            memcpy(*dstSimpToOrigTriMap, &oDstToSrcTriMap[0], sizeof(uint32_t) * oDstToSrcTriMap.size());
        }
    }

    
    void remeshTriangleMeshInternal(carb::Float3*& dstPoints, uint32_t& dstPointsSize, uint32_t*& dstIndices, uint32_t& dstIndicesSize,
        uint32_t** dstOrigToRemeshedVertexMap, uint32_t* dstOrigToRemeshedVertexMapSize,
        const carb::Float3* srcPoints, const uint32_t srcPointsSize, const uint32_t* srcIndices, const uint32_t srcIndicesSize,
        const uint32_t remeshingVoxelResolution,
        void* (*allocateBytes)(size_t))
    {
        PxArray<PxVec3> points((PxVec3*)srcPoints, (PxVec3*)srcPoints + srcPointsSize);
        PxArray<PxU32> indices(srcIndices, srcIndices + srcIndicesSize);

        PxArray<PxVec3> oPoints;
        PxArray<PxU32> oIndices;
        PxArray<PxU32> oOrigToRemeshedMap;
        PxTetMaker::remeshTriangleMesh(points, indices, remeshingVoxelResolution, oPoints, oIndices, dstOrigToRemeshedVertexMap ? &oOrigToRemeshedMap : NULL);

        dstPointsSize = oPoints.size();
        dstPoints = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * dstPointsSize);
        memcpy(dstPoints, &oPoints[0], sizeof(carb::Float3) * dstPointsSize);

        dstIndicesSize = oIndices.size();
        dstIndices = (uint32_t*)allocateBytes(sizeof(uint32_t) * dstIndicesSize);
        memcpy(dstIndices, &oIndices[0], sizeof(uint32_t) * dstIndicesSize);

        if (dstOrigToRemeshedVertexMap && dstOrigToRemeshedVertexMapSize)
        {
            *dstOrigToRemeshedVertexMapSize = oOrigToRemeshedMap.size();
            *dstOrigToRemeshedVertexMap = (uint32_t*)allocateBytes(sizeof(uint32_t) * oOrigToRemeshedMap.size());
            memcpy(*dstOrigToRemeshedVertexMap, &oOrigToRemeshedMap[0], sizeof(uint32_t) * oOrigToRemeshedMap.size());
        }
    }

    
    bool computeConformingTetrahedralMesh(carb::Float3*& dstTetPoints, uint32_t& dstTetPointsSize, uint32_t*& dstTetIndices, uint32_t& dstTetIndicesSize,
        const carb::Float3* srcTriPoints, const uint32_t srcTriPointsSize, const uint32_t* srcTriIndices, const uint32_t srcTriIndicesSize,
        void* (*allocateBytes)(size_t))
    {
        return computeConformingTetrahedralMeshInternal(dstTetPoints, dstTetPointsSize, dstTetIndices, dstTetIndicesSize,
            srcTriPoints, srcTriPointsSize, srcTriIndices, srcTriIndicesSize, allocateBytes);
    }

    bool computeConformingTetrahedralMeshInternal(carb::Float3*& dstTetPoints, uint32_t& dstTetPointsSize, uint32_t*& dstTetIndices, uint32_t& dstTetIndicesSize,
        const carb::Float3* srcTriPoints, const uint32_t srcTriPointsSize, const uint32_t* srcTriIndices, const uint32_t srcTriIndicesSize,
        void* (*allocateBytes)(size_t))
    {
        PxSimpleTriangleMesh surfaceMesh;
        surfaceMesh.points.count = srcTriPointsSize;
        surfaceMesh.points.data = srcTriPoints;
        surfaceMesh.triangles.count = srcTriIndicesSize / 3u;
        surfaceMesh.triangles.data = srcTriIndices;
        surfaceMesh.flags = PxMeshFlags(0u); // disable both flags

        if (!validateSurfaceMesh(surfaceMesh))
        {
            return false;
        }

        PxArray<PxVec3> physxConformingTetMeshVertices;
        PxArray<PxU32> physxConformingTetMeshIndices;
        constexpr bool validate = true;
        bool conformingSuccess = PxTetMaker::createConformingTetrahedronMesh(surfaceMesh, physxConformingTetMeshVertices, physxConformingTetMeshIndices, validate, 1e-8f);
        if (!conformingSuccess)
        {
            CARB_LOG_ERROR("PxTetMaker::createConformingTetrahedronMesh failed");
            return false;
        }

        if (!validateTetMesh(physxConformingTetMeshVertices, physxConformingTetMeshIndices))
        {
            CARB_LOG_ERROR("PxTetMaker::createConformingTetrahedronMesh failed");
            return false;
        }

        dstTetPointsSize = physxConformingTetMeshVertices.size();
        dstTetPoints = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * dstTetPointsSize);
        memcpy(dstTetPoints, &physxConformingTetMeshVertices[0], sizeof(carb::Float3) * dstTetPointsSize);

        dstTetIndicesSize = physxConformingTetMeshIndices.size();
        dstTetIndices = (uint32_t*)allocateBytes(sizeof(uint32_t) * dstTetIndicesSize);
        memcpy(dstTetIndices, &physxConformingTetMeshIndices[0], sizeof(uint32_t) * dstTetIndicesSize);
        return true;
    }

    

    //Might not match the surface exactly
    bool computeTreeBasedTetrahedralMeshInternal(carb::Float3*& dstPoints, uint32_t& dstPointsSize, uint32_t*& dstIndices, uint32_t& dstIndicesSize,
        const carb::Float3* srcPoints, const uint32_t srcPointsSize, const uint32_t* srcIndices, const uint32_t srcIndicesSize,
        const bool useTreeNodes, void* (*allocateBytes)(size_t))
    {
        PxSimpleTriangleMesh surfaceMesh;
        surfaceMesh.points.count = srcPointsSize;
        surfaceMesh.points.data = srcPoints;
        surfaceMesh.triangles.count = srcIndicesSize / 3u;
        surfaceMesh.triangles.data = srcIndices;
        surfaceMesh.flags = PxMeshFlags(0u); // disable both flags

        if (!validateSurfaceMesh(surfaceMesh))
        {
            return false;
        }

        PxArray<PxVec3> points((PxVec3*)srcPoints, (PxVec3*)srcPoints + srcPointsSize);
        PxArray<PxU32> indices(srcIndices, srcIndices + srcIndicesSize);

        PxArray<PxVec3> oPoints;
        PxArray<PxU32> oIndices;
        PxTetMaker::createTreeBasedTetrahedralMesh(points, indices, useTreeNodes, oPoints, oIndices, 1e-8f);

        if (!validateTetMesh(oPoints, oIndices))
        {
            CARB_LOG_ERROR("PxTetMaker::createTreeBasedTetrahedralMesh failed");
            return false;
        }

        dstPointsSize = oPoints.size();
        dstPoints = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * dstPointsSize);
        memcpy(dstPoints, &oPoints[0], sizeof(carb::Float3) * dstPointsSize);

        dstIndicesSize = oIndices.size();
        dstIndices = (uint32_t*)allocateBytes(sizeof(uint32_t) * dstIndicesSize);
        memcpy(dstIndices, &oIndices[0], sizeof(uint32_t) * dstIndicesSize);

        return true;
    }

    void createPointsToTetrahedraMapInternal(uint32_t* dstPointsToTetIndices, carb::Float3* dstBarycentricCoordinates,
        const carb::Float3* srcPoints, const uint32_t srcPointsSize,
        const carb::Float3* srcTetPoints, const uint32_t srcTetPointsSize, const uint32_t* srcTetIndices, const uint32_t srcTetIndicesSize)
    {
        if (dstPointsToTetIndices == nullptr && dstBarycentricCoordinates == nullptr)
        {
            return;
        }

        bool aliasedInputs = (srcPoints == srcTetPoints && srcPointsSize == srcTetPointsSize);
        PxArray<PxVec3> tetMeshVertices;
        tetMeshVertices.assign(reinterpret_cast<const PxVec3*>(srcTetPoints), reinterpret_cast<const PxVec3*>(srcTetPoints) + srcTetPointsSize);
        PxArray<PxU32> tetMeshIndices;
        tetMeshIndices.assign(srcTetIndices, srcTetIndices + srcTetIndicesSize);
        PxArray<PxVec3>* pointsToEmbed = nullptr;
        PxArray<PxVec3> points;

        if (aliasedInputs)
        {
            pointsToEmbed = &tetMeshVertices;
        }
        else
        {
            points.assign(reinterpret_cast<const PxVec3*>(srcPoints), reinterpret_cast<const PxVec3*>(srcPoints) + srcPointsSize);
            pointsToEmbed = &points;
        }

        PxArray<PxVec4> barycentricCoordinates;
        PxArray<PxU32> tetLinks;
        PxTetrahedronMeshExt::createPointsToTetrahedronMap(tetMeshVertices, tetMeshIndices, *pointsToEmbed, barycentricCoordinates, tetLinks);

        if (dstPointsToTetIndices && tetLinks.size() == srcPointsSize)
        {
            memcpy(dstPointsToTetIndices, &tetLinks[0], sizeof(uint32_t)*srcPointsSize);
        }

        if (dstBarycentricCoordinates && barycentricCoordinates.size() == srcPointsSize)
        {
            memcpy(dstBarycentricCoordinates, &barycentricCoordinates[0], sizeof(carb::Float3)*srcPointsSize);
        }
    }

    
    void transformTetrahedralMeshPoseInternal(carb::Float3* dstTetPoints, const carb::Float3* srcTetPoints, const uint32_t srcTetPointsSize, const carb::Float3* srcKinematicTargetPoints,
        const uint32_t srcKinematicTargetPointsSize, const uint32_t* srcTetIndices, const uint32_t srcTetIndicesSize)
    {
        if (srcKinematicTargetPointsSize > srcTetPointsSize)
        {
            return;
        }

        PxArray<PxVec4> originalVertices(srcTetPointsSize);
        for (uint32_t i = 0; i < originalVertices.size(); ++i)
        {
            originalVertices[i] = PxVec4(toPhysX(srcTetPoints[i]), i < srcKinematicTargetPointsSize ? 0.0f : 1.0f);
        }

        //just initialize first srcKinematicTargetPointsSize points for target.
        PxArray<PxVec4> targetVertices(srcTetPointsSize);
        for (uint32_t i = 0; i < srcKinematicTargetPointsSize; ++i)
        {
            targetVertices[i] = PxVec4(toPhysX(srcKinematicTargetPoints[i]), 0.0f);
        }
        for (uint32_t i = srcKinematicTargetPointsSize; i < srcTetPointsSize; ++i)
        {
            targetVertices[i] = PxVec4(toPhysX(srcTetPoints[i]), 1.0f);
        }

        PxDeformableVolumeExt::relaxDeformableVolumeMesh(originalVertices.begin(), targetVertices.begin(), originalVertices.size(), srcTetIndices, srcTetIndicesSize/4);

        if (dstTetPoints)
        {
            for (uint32_t i = 0; i < srcTetPointsSize; ++i)
            {
                dstTetPoints[i] = fromPhysX(targetVertices[i].getXYZ());
            }
        }
    }
    

    bool computeVoxelTetrahedralMeshInternal(carb::Float3*& dstTetPoints, uint32_t& dstTetPointsSize, uint32_t*& dstTetIndices, uint32_t& dstTetIndicesSize, int32_t*& dstEmbedding, uint32_t& dstEmbeddingSize,
        const carb::Float3* srcTetPoints, const uint32_t srcTetPointsSize, const uint32_t* srcTetIndices, const uint32_t srcTetIndicesSize,
        const uint32_t voxelResolution, const uint32_t numTetsPerVoxel, const uint32_t* anchorNodes, void* (*allocateBytes)(size_t))
    {
        PxArray<PxVec3> physxVoxelTetMeshVertices;
        PxArray<PxU32> physxVoxelTetMeshIndices;
        PxArray<PxI32> vertexToTet;
        vertexToTet.resize(srcTetPointsSize);

        PxTetrahedronMeshDesc meshDesc;
        meshDesc.points.count = srcTetPointsSize;
        meshDesc.points.stride = sizeof(carb::Float3);
        meshDesc.points.data = srcTetPoints;
        meshDesc.tetrahedrons.count = srcTetIndicesSize / 4;
        meshDesc.tetrahedrons.stride = sizeof(uint32_t) * 4;
        meshDesc.tetrahedrons.data = srcTetIndices;
        meshDesc.tetsPerElement = numTetsPerVoxel;

        if (!meshDesc.isValid())
        {
            CARB_LOG_ERROR("PxTetrahedronMeshDesc used as input to PxTetMaker::createVoxelTetrahedronMesh is not valid");
            return false;
        }

        bool voxelSuccess = PxTetMaker::createVoxelTetrahedronMesh(meshDesc,
            voxelResolution, physxVoxelTetMeshVertices, physxVoxelTetMeshIndices, vertexToTet.begin(), anchorNodes, meshDesc.tetsPerElement);

        if (!voxelSuccess)
        {
            CARB_LOG_ERROR("PxTetMaker::createVoxelTetrahedronMesh failed");
            return false;
        }

        if (!validateTetMesh(physxVoxelTetMeshVertices, physxVoxelTetMeshIndices))
        {
            CARB_LOG_ERROR("PxTetMaker::createVoxelTetrahedronMesh failed");
            return false;
        }

        dstEmbeddingSize = vertexToTet.size();
        dstEmbedding = (int32_t*)allocateBytes(sizeof(int32_t) * dstEmbeddingSize);
        memcpy(dstEmbedding, &vertexToTet[0], sizeof(int32_t) * dstEmbeddingSize);

        dstTetPointsSize = physxVoxelTetMeshVertices.size();
        dstTetPoints = (carb::Float3*)allocateBytes(sizeof(carb::Float3) * dstTetPointsSize);
        memcpy(dstTetPoints, &physxVoxelTetMeshVertices[0], sizeof(carb::Float3) * dstTetPointsSize);

        dstTetIndicesSize = physxVoxelTetMeshIndices.size();
        dstTetIndices = (uint32_t*)allocateBytes(sizeof(uint32_t) * dstTetIndicesSize);
        memcpy(dstTetIndices, &physxVoxelTetMeshIndices[0], sizeof(uint32_t) * dstTetIndicesSize);
        return true;
    }

}
}
