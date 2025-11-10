// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/logging/Log.h>

#include <internal/InternalTools.h>
#include "PhysXCooking.h"
#include "MeshCache.h"
#include "CookingDataAsync.h"
#include "CollisionRepresentation.h"
#include "PhysXTools.h"
#include "usdLoad/LoadUsd.h"
#include "usdLoad/Collision.h"
#include <usdLoad/PhysicsBody.h>
#include "OmniPhysX.h"
#include "Setup.h"
#include "ObjectDataQuery.h"
#include <private/omni/physx/PhysXCompoundShape.h>
#include <private/omni/physx/IPhysxCookingPrivate.h>

#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <private/omni/physics/schema/IUsdPhysics.h>

#include <PxPhysicsAPI.h>
#include <extensions/PxTetMakerExt.h>
#include <extensions/PxTetrahedronMeshExt.h>
#include <cooking/PxTetrahedronMeshDesc.h>
#include <extensions/PxRemeshingExt.h>
#include <extensions/PxSoftBodyExt.h>

using namespace ::physx;
using namespace omni::physx;
using namespace omni::physx::usdparser;
using namespace omni::physx::internal;
using namespace pxr;
using namespace carb;
using namespace cookingdataasync;

namespace omni
{
namespace physx
{
    //IPhysxCooking deprecated
 
    bool computeConformingTetrahedralMesh(carb::Float3*& dstTetPoints, uint32_t& dstTetPointsSize, uint32_t*& dstTetIndices, uint32_t& dstTetIndicesSize,
    const carb::Float3* srcTriPoints, const uint32_t srcTriPointsSize, const uint32_t* srcTriIndices, const uint32_t srcTriIndicesSize,
    void* (*allocateBytes)(size_t))
    {
        omni::physx::IPhysxCookingServicePrivate* cookingService = carb::getCachedInterface<omni::physx::IPhysxCookingServicePrivate>();
        omni::physx::PhysxCookingTetrahedralMeshInput meshInput;
        meshInput.srcTriPoints = srcTriPoints;
        meshInput.srcTriPointsSize = srcTriPointsSize;
        meshInput.srcTriIndices = srcTriIndices;
        meshInput.srcTriIndicesSize = srcTriIndicesSize;
        omni::physx::PhysxCookingTetrahedralMeshOutput meshOutput;
        meshOutput.allocateBytes = allocateBytes;

        bool res = cookingService->computeConformingTetrahedralMesh(meshInput, meshOutput);

        dstTetPoints = meshOutput.dstTetPoints;
        dstTetPointsSize = meshOutput.dstTetPointsSize;
        dstTetIndices = meshOutput.dstTetIndices;
        dstTetIndicesSize = meshOutput.dstTetIndicesSize;

        return res;
    }
    bool computeVoxelTetrahedralMesh(carb::Float3*& dstTetPoints, uint32_t& dstTetPointsSize, uint32_t*& dstTetIndices, uint32_t& dstTetIndicesSize, int32_t*& dstEmbedding, uint32_t& dstEmbeddingSize,
        const carb::Float3* srcTetPoints, const uint32_t srcTetPointsSize, const uint32_t* srcTetIndices, const uint32_t srcTetIndicesSize,
        const carb::Float3& srcScale, const int voxelResolution, void* (*allocateBytes)(size_t))
    {
        std::vector<carb::Float3> scaledTetPoints;
        scaledTetPoints.resize(srcTetPointsSize);
        for (uint32_t v = 0; v < (uint32_t)scaledTetPoints.size(); ++v)
        {
            const ::carb::Float3& point = srcTetPoints[v];
            scaledTetPoints[v] = { point.x * srcScale.x, point.y * srcScale.y, point.z * srcScale.z };
        }

        omni::physx::PhysxCookingTetrahedralMeshInput meshInput;
        meshInput.srcTriPoints = &scaledTetPoints[0];
        meshInput.srcTriPointsSize = uint32_t(scaledTetPoints.size());
        meshInput.srcTriIndices = srcTetIndices;
        meshInput.srcTriIndicesSize = srcTetIndicesSize;
        omni::physx::PhysxCookingTetrahedralMeshOutput meshOutput;
        meshOutput.allocateBytes = allocateBytes;

        PhysxCookingTetrahedralVoxelMeshParameters parameters;
        parameters.voxelResolution = voxelResolution;
        parameters.anchorNodes = nullptr;
        omni::physx::IPhysxCookingServicePrivate* cookingService = carb::getCachedInterface<omni::physx::IPhysxCookingServicePrivate>();

        bool success = cookingService->computeVoxelTetrahedralMesh(meshInput, meshOutput, parameters);            
        
        dstTetPoints = meshOutput.dstTetPoints;
        dstTetPointsSize = meshOutput.dstTetPointsSize;
        dstTetIndices = meshOutput.dstTetIndices;
        dstTetIndicesSize = meshOutput.dstTetIndicesSize;
        dstEmbedding = meshOutput.dstEmbedding;
        dstEmbeddingSize = meshOutput.dstEmbeddingSize;

        if (success)
        {
            carb::Float3 scaleInv = { 1.0f / srcScale.x, 1.0f / srcScale.y, 1.0f / srcScale.z };
            for (uint32_t v = 0; v < dstTetPointsSize; ++v)
            {
                carb::Float3& point = dstTetPoints[v];
                point = { point.x * scaleInv.x, point.y * scaleInv.y, point.z * scaleInv.z };
            }
        }
        return success;
    }

    //IPhysxCooking

    void releaseLocalMeshCache()
    {
        CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();

        if (cookingDataAsync != nullptr)
        {
            cookingDataAsync->resetLocalMeshCacheContents();
        }
    }

    /// Add Prim to cooking refresh set
    void addPrimToCookingRefreshSet(const pxr::SdfPath& path)
    {
        CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();

        if (cookingDataAsync != nullptr)
        {
            cookingDataAsync->addPrimRefreshSet(path);
        }
    }

    void releaseRuntimeMeshCache()
    {
        omni::physx::getMeshCache()->release();
    }

    uint32_t getNbConvexMeshData(const pxr::SdfPath& path)
    {
        const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

        const uint64_t stageId = OmniPhysX::getInstance().getStageId();
        const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (!attachedStage)
            return 0;

        uint32_t numMeshes = 0;
        void* physxPtr = (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, ePTShape, db, *attachedStage));
        if (physxPtr)
        {
            PxShape* shape = (PxShape*)physxPtr;
            if (shape->getGeometry().getType() == PxGeometryType::eCONVEXMESH)
                numMeshes++;
        }
        else
        {
            physxPtr = (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, ePTCompoundShape, db, *attachedStage));
            if (physxPtr)
            {
                PhysXCompoundShape* compoundShape = (PhysXCompoundShape*)physxPtr;
                for (size_t i = 0; i < compoundShape->getShapes().size(); i++)
                {
                    PxShape* shape = (PxShape*)(compoundShape->getShapes()[i]);
                    if (shape->getGeometry().getType() == PxGeometryType::eCONVEXMESH)
                        numMeshes++;
                }
            }
        }
        return numMeshes;
    }

    void getConvexMeshData(const pxr::SdfPath& path, uint32_t convexIndex, ConvexMeshData& meshData)
    {
        uint32_t numMeshes = 0;
        const internal::InternalPhysXDatabase& db = OmniPhysX::getInstance().getInternalPhysXDatabase();

        PxConvexMesh* mesh = nullptr;

        const uint64_t stageId = OmniPhysX::getInstance().getStageId();
        const usdparser::AttachedStage* attachedStage = usdparser::UsdLoad::getUsdLoad()->getAttachedStage(stageId);
        if (!attachedStage)
            return;

        void* physxPtr = (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, ePTShape, db, *attachedStage));
        if (physxPtr)
        {
            PxShape* shape = (PxShape*)physxPtr;
            const PxGeometry& geom = shape->getGeometry();
            if (geom.getType() == PxGeometryType::eCONVEXMESH)
            {
                const PxConvexMeshGeometry& convexGeometry = static_cast<const PxConvexMeshGeometry&>(geom);
                mesh = convexGeometry.convexMesh;
            }
        }
        else
        {
            physxPtr = (void*)(getObjectDataOrID<ObjectDataQueryType::ePHYSX_PTR>(path, ePTCompoundShape, db, *attachedStage));
            if (physxPtr)
            {
                PhysXCompoundShape* compoundShape = (PhysXCompoundShape*)physxPtr;
                if (convexIndex < compoundShape->getShapes().size())
                {
                    PxShape* shape = (PxShape*)(compoundShape->getShapes()[convexIndex]);
                    const PxGeometry& geom = shape->getGeometry();
                    if (geom.getType() == PxGeometryType::eCONVEXMESH)
                    {
                        const PxConvexMeshGeometry& convexGeometry = static_cast<const PxConvexMeshGeometry&>(geom);
                        mesh = convexGeometry.convexMesh;
                    }
                }
            }
        }

        if (mesh)
        {
            const ConvexMeshDataMap& meshMap = getMeshCache()->getConvexMeshDataMap();
            ConvexMeshDataMap::const_iterator it = meshMap.find(mesh);
            if (it != meshMap.end())
            {
                const ConvexMeshData& md = it->second;
                meshData.indices = md.indices;
                meshData.numPolygons = md.numPolygons;
                meshData.numVertices = md.numVertices;
                meshData.polygons = md.polygons;
                meshData.vertices = md.vertices;
                meshData.numIndices = md.numIndices;
                return;
            }
        }
    }

    bool createConvexMesh(const pxr::SdfPath& path, uint32_t vertexLimit, ConvexMeshData& meshData)
    {
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        bool retVal = false;
        if (cookingDataAsync && stage)
        {
            UsdPrim prim = stage->GetPrimAtPath(path);
            if (prim && prim.IsA<UsdGeomMesh>())
            {
                ConvexMeshPhysxShapeDesc convexMeshDesc;

                if (usdparser::fillConvexMeshDesc(UsdGeomMesh(prim), convexMeshDesc, ConvexMeshCookingParams()))
                {
                    Float3 scale = { 1.0f, 1.0f, 1.0f };
                    convexMeshDesc.meshScale = scale;
                    convexMeshDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(scale);
                    convexMeshDesc.crc.setMaxHullVertices(vertexLimit);
                    convexMeshDesc.convexCookingParams.maxHullVertices = vertexLimit;

                    PxConvexMesh* convexMesh = cookingDataAsync->getConvexMesh(convexMeshDesc, prim, false);
                    if (convexMesh)
                    {
                        const ConvexMeshDataMap& meshMap = getMeshCache()->getConvexMeshDataMap();
                        ConvexMeshDataMap::const_iterator it = meshMap.find(convexMesh);
                        if (it != meshMap.end())
                        {
                            meshData = it->second;
                        }
                        return true;
                    }
                    else
                    {
                        CARB_LOG_ERROR("omni::physx::createConvexMesh: Failed to cooked convex mesh data.");
                    }
                }
                else
                {
                    CARB_LOG_ERROR("omni::physx::createConvexMesh: Provided mesh is not a valid mesh.");
                }
            }
            else
            {
                CARB_LOG_ERROR("omni::physx::createConvexMesh: Provided prim is invalid or not a UsdGeomMesh.");
            }
        }
        return retVal;
    }

    bool cookDeformableBodyMeshDeprecated(const pxr::SdfPath& deformableBodyPath)
    {
        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        UsdPrim usdPrim = stage->GetPrimAtPath(deformableBodyPath);

        if (!usdPrim.IsValid() || !usdPrim.HasAPI<PhysxSchemaPhysxDeformableBodyAPI>())
        {
            CARB_LOG_ERROR("cookDeformableBodyMesh deformableBodyPath prim requires PhysxSchemaPhysxDeformableBodyAPI");
            return false;
        }

        bool success = false;
        CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        if (cookingDataAsync)
        {
            // make sure simulation mesh is written by the async cooker
            cookingDataAsync->cookDeformableBodyTetMeshDeprecated(const_cast<UsdPrim&>(usdPrim), false);
            SoftBodyDesc* deformableBodyDesc = parseDeformableBodyDeprecated(OmniPhysX::getInstance().getStageId(), deformableBodyPath);
            if (deformableBodyDesc)
            {
                PxDefaultMemoryOutputStream outStream;
                success = cookingDataAsync->cookSoftBodyMeshDeprecated(outStream, *deformableBodyDesc, usdPrim, false);
                releaseDesc(deformableBodyDesc);
            }
        }
        return success;
    }

    bool cookAutoDeformableBody(const pxr::SdfPath& deformableBodyPath)
    {
        //workaround to re-initialize cooking data async when it get's released in some circumstances
        //(memory stage tests), when scene parsing releases physics.
        OmniPhysX::getInstance().getPhysXSetup().getPhysics();
        CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        if (!cookingDataAsync)
        {
            CARB_LOG_ERROR("cookAutoDeformableBody: couldn't access cooking for %s",
                deformableBodyPath.GetText());
            return false;
        }

        UsdStageWeakPtr stage = OmniPhysX::getInstance().getStage();
        UsdPrim usdPrim = stage->GetPrimAtPath(deformableBodyPath);
        bool success = false;
        pxr::TfType dbType = pxr::UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(pxr::OmniPhysicsDeformableAPITokens->DeformableBodyAPI);
        if (!usdPrim.IsValid() || !usdPrim.HasAPI(dbType))
        {
            CARB_LOG_ERROR("cookAutoDeformableBody: prim requires UsdPhysicsDeformableBodyAPI, %s",
                           deformableBodyPath.GetText());
            return success;
        }

        const TfType autoDeformableType = UsdSchemaRegistry::GetAPITypeFromSchemaTypeName(PhysxAdditionAPITokens->AutoDeformableBodyAPI);
        if (!usdPrim.HasAPI(autoDeformableType))
        {
            CARB_LOG_ERROR("cookAutoDeformableBody: prim requires PhysxSchemaPhysxAutoDeformableBodyAPI, %s",
                           deformableBodyPath.GetText());
            return success;
        }

        omni::physx::usdparser::PhysxDeformableBodyDesc* deformableDesc = cookingDataAsync->parseDeformableBody(usdPrim);
        if (deformableDesc)
        {
            if (deformableDesc->type == ObjectType::eVolumeDeformableBody)
            {
                const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc* volumeDesc =
                    static_cast<const omni::physx::usdparser::PhysxVolumeDeformableBodyDesc*>(deformableDesc);

                if (volumeDesc->hasAutoAPI)
                {
                    cookingDataAsync->cookVolumeDeformableBody(*volumeDesc, usdPrim, false);
                    success = true;
                }
                else
                {
                    CARB_LOG_ERROR("cookAutoDeformableBody: expected auto configuration for %s",
                        deformableBodyPath.GetText());
                }
            }
            else if (deformableDesc->type == ObjectType::eSurfaceDeformableBody)
            {
                const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc* surfaceDesc =
                    static_cast<const omni::physx::usdparser::PhysxSurfaceDeformableBodyDesc*>(deformableDesc);

                if (surfaceDesc->hasAutoAPI)
                {
                    cookingDataAsync->cookSurfaceDeformableBody(*surfaceDesc, usdPrim, false);
                    success = true;
                }
                else
                {
                    CARB_LOG_ERROR("cookAutoDeformableBody: expected auto configuration for %s",
                        deformableBodyPath.GetText());
                }
            }
            ICE_FREE(deformableDesc);
        }
        return success;
    }

    PhysxCookingStatistics getCookingStatistics()
    {
        CookingDataAsync* cookingDataAsync = OmniPhysX::getInstance().getPhysXSetup().getCookingDataAsync();
        if (cookingDataAsync)
        {
            return cookingDataAsync->getCookingStatistics();
        }
        return {};
    }

    bool precookMesh(uint64_t stageId, uint64_t path, const CookingParams& cookingParams, IPhysxCookingCallback* cb)
    {
        if (cookingParams.type == CookingParamsType::eUNDEFINED)
        {
            CARB_LOG_ERROR("IPhysxCooking::precookMesh undefined cooking params!");
            return false;
        }

        UsdStageWeakPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt((long)stageId));
        if (!stage)
        {
            CARB_LOG_ERROR("IPhysxCooking::precookMesh stage not found!");
            return false;
        }

        const SdfPath meshPath = intToPath(path);
        UsdPrim meshPrim = stage->GetPrimAtPath(meshPath);
        if (!meshPrim || !meshPrim.IsA<UsdGeomMesh>())
        {
            CARB_LOG_ERROR("IPhysxCooking::precookMesh prim not found or not UsdGeomMesh!");
            return false;
        }

        OmniPhysX& omniPhysX = OmniPhysX::getInstance();
        PhysXSetup& physxSetup = omniPhysX.getPhysXSetup();
        CookingDataAsync* cookingDataAsync = physxSetup.getCookingDataAsync();
        if (!cookingDataAsync)
        {
            CARB_LOG_ERROR("IPhysxCooking::precookMesh cooking not initialized, stage most likely not attached!");
            return false;
        }

        switch (cookingParams.type)
        {
        case CookingParamsType::eCONVEX_MESH:
        {
            ConvexMeshPhysxShapeDesc convexMeshDesc;

            if (usdparser::fillConvexMeshDesc(UsdGeomMesh(meshPrim), convexMeshDesc, (const ConvexMeshCookingParams &)cookingParams))
            {
                cookingDataAsync->getConvexMesh(convexMeshDesc, meshPrim, cb ? true : false, cb);
            }
            else
            {
                CARB_LOG_ERROR("IPhysxCooking::precookMesh UsdGeomMesh invalid!");
                return false;
            }
        }
        break;
        case CookingParamsType::eTRIANGLE_MESH:
        {
            TriangleMeshPhysxShapeDesc triMeshDesc;

            if (usdparser::fillTriangleMeshDesc(UsdGeomMesh(meshPrim), triMeshDesc, (const TriangleMeshCookingParams&)cookingParams))
            {
                cookingDataAsync->getTriangleMesh(triMeshDesc, meshPrim, cb ? true : false, cb);
            }
            else
            {
                CARB_LOG_ERROR("IPhysxCooking::precookMesh UsdGeomMesh invalid!");
                return false;
            }
        }
        break;
        case CookingParamsType::eSDF_TRIANGLE_MESH:
        {
            TriangleMeshPhysxShapeDesc triMeshDesc;

            if (usdparser::fillSdfTriangleMeshDesc(UsdGeomMesh(meshPrim), triMeshDesc, (const SdfMeshCookingParams&)cookingParams))
            {
                cookingDataAsync->getTriangleMesh(triMeshDesc, meshPrim, cb ? true : false, cb);
            }
            else
            {
                CARB_LOG_ERROR("IPhysxCooking::precookMesh UsdGeomMesh invalid!");
                return false;
            }
        }
        break;
        case CookingParamsType::eCONVEX_DECOMPOSITION:
        {
            ConvexMeshDecompositionPhysxShapeDesc triMeshDesc;

            if (usdparser::fillConvexDecompositionDesc(UsdGeomMesh(meshPrim), triMeshDesc, (const ConvexDecompositionCookingParams&)cookingParams))
            {
                cookingDataAsync->getConvexMeshDecomposition(triMeshDesc, meshPrim, cb ? true : false, cb);
            }
            else
            {
                CARB_LOG_ERROR("IPhysxCooking::precookMesh UsdGeomMesh invalid!");
                return false;
            }
        }
        break;
        case CookingParamsType::eSPHERE_FILL:
        {
            SpherePointsPhysxShapeDesc triMeshDesc;

            if (usdparser::fillSphereFillDesc(UsdGeomMesh(meshPrim), triMeshDesc, (const SphereFillCookingParams&)cookingParams))
            {
                cookingDataAsync->getSpherePoints(triMeshDesc, meshPrim, cb ? true : false, cb);
            }
            else
            {
                CARB_LOG_ERROR("IPhysxCooking::precookMesh UsdGeomMesh invalid!");
                return false;
            }
        }
        break;
        default:
        {
            CARB_LOG_ERROR("IPhysxCooking::precookMesh undefined cooking params!");
            return false;
        }
        break;
        }

        return true;
    }
}
}

void fillInterface(omni::physx::IPhysxCooking& iface)
{
    iface.createConvexMesh = createConvexMesh;

    iface.cookDeformableBodyMesh = cookDeformableBodyMeshDeprecated;
    iface.cookAutoDeformableBody = cookAutoDeformableBody;

    iface.releaseLocalMeshCache = releaseLocalMeshCache;

    iface.computeConformingTetrahedralMesh = computeConformingTetrahedralMesh;
    iface.computeVoxelTetrahedralMesh = computeVoxelTetrahedralMesh;

    iface.precookMesh = precookMesh;

    iface.requestConvexCollisionRepresentation = requestConvexCollisionRepresentation;
    iface.cancelCollisionRepresentationTask = cancelCollisionRepresentationTask;
}

void fillInterface(omni::physx::IPhysxCookingPrivate& iface)
{
    iface.getCookingStatistics = getCookingStatistics;
    iface.addPrimToCookingRefreshSet = addPrimToCookingRefreshSet;
    iface.releaseRuntimeMeshCache = releaseRuntimeMeshCache;
}
