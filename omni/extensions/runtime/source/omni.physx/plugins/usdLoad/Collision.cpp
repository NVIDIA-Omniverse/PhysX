// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// This include must come first
// clang-format off
#include "UsdPCH.h"
// clang-format on

#include <carb/Types.h>
#include <carb/logging/Log.h>
#include <carb/settings/ISettings.h>
#include <private/omni/physics/schema/IUsdPhysics.h>
#include <common/foundation/Allocator.h>
#include <common/utilities/UsdMaterialParsing.h>
#include <carb/profiler/Profile.h>

#include <carb/tasking/TaskingTypes.h>
#include <carb/tasking/TaskingUtils.h>


#include <utils/Profile.h>

#include "LoadUsd.h"
#include "LoadTools.h"
#include "Mass.h"
#include "PhysicsBody.h"
#include "Collision.h"
#include "Material.h"
#include "CollisionGroup.h"
#include "AttributeHelpers.h"

#include <propertiesUpdate/PhysXPropertiesUpdate.h>
#include <common/utilities/Utilities.h>
#include <omni/physx/IPhysxCookingService.h>

using namespace pxr;
using namespace carb;
using namespace carb::tasking;
using namespace omni::physics::schema;

namespace omni
{
namespace physx
{
namespace usdparser
{

static TfToken oldConvexPrim("ConvexMesh");
static TfToken obsoleteCustomGeometryAttribute("physxCollisionCustomGeometry");
using MeshKeyMap = std::unordered_map< SdfPath, omni::physx::usdparser::MeshKey,SdfPath::Hash >;


class MeshKeyCache
{
public:

    void setMeshKey(const SdfPath &name,const MeshKey &key)
    {
        mMeshKeys[name] = key;
    }

    bool getMeshKey(const SdfPath &name,MeshKey &key) const
    {
        bool ret = false;

        MeshKeyMap::const_iterator found = mMeshKeys.find(name);
        if ( found != mMeshKeys.end() )
        {
            ret = true;
            key = (*found).second;
        }

        return ret;
    }

    // Reset a single key
    bool clearMeshKey(const SdfPath &name)
    {
        bool ret = false;

        MeshKeyMap::iterator found = mMeshKeys.find(name);
        if (found != mMeshKeys.end())
        {
            ret = true;
            mMeshKeys.erase(found);
        }
        return ret;
    }

    // Reset the entire key cache
    void reset(void)
    {
        mMeshKeys.clear();
    }

private:
    MeshKeyMap  mMeshKeys;
};

MeshKeyCache gMeshKeyCache;


// compute bounding sphere for a given vertex array
// get the furthers points along each axis first and create a sphere based on these two initial points (min/max)
// traverse the vertices and if a vertex does not belong to the sphere, construct a new center and radius including this
// vertex and continue
BoundingSpherePhysxShapeDesc* computeBoundingSphereShape(const VtArray<GfVec3f>& pointsValue)
{
    Float3 sphereCenter;
    float radius;

    const bool success = PhysXUsdPhysicsInterface::createBoundingSphere(
        &pointsValue.front(), pointsValue.size(), sphereCenter, radius);
    if (!success)
    {
        CARB_LOG_ERROR("Failed to create OBB for input point clouds!");
        return nullptr;
    }

    BoundingSpherePhysxShapeDesc* sphereDesc = ICE_PLACEMENT_NEW(BoundingSpherePhysxShapeDesc)();
    sphereDesc->radius = radius;
    sphereDesc->positionOffset = sphereCenter;

    return sphereDesc;
}

// compute OBB around given points
BoundingBoxPhysxShapeDesc* computeBoundingBoxShape(const VtArray<GfVec3f>& pointsValue)
{
    Float3 halfExtent;
    Float3 offsetPos;
    Float4 offsetRot;

    const bool success = PhysXUsdPhysicsInterface::createOBB(
        &pointsValue.front(), pointsValue.size(), halfExtent, offsetPos, offsetRot);
    if (!success)
    {
        CARB_LOG_ERROR("Failed to create OBB for input point clouds!");
        return nullptr;
    }

    BoundingBoxPhysxShapeDesc* boxDesc = ICE_PLACEMENT_NEW(BoundingBoxPhysxShapeDesc)();

    boxDesc->rotationOffset = offsetRot;
    boxDesc->positionOffset = offsetPos;
    boxDesc->halfExtents = halfExtent;

    return boxDesc;
}

// Helper method to initialize the maximum number of convex hull vertices and add it to the MeshKey CRC
void initMaxHullVertices(const UsdAttribute& attr, uint32_t &maxHullVertices)
{
    int _maxHullVertices;
    if (attr.Get(&_maxHullVertices))
    {
        maxHullVertices = uint32_t(_maxHullVertices);
    }    
}

// Helper method to initialize the maximum number of convex hulls and add it to the MeshKey CRC
void initMaxConvexHulls(const UsdAttribute& attr, uint32_t &maxConvexHulls)
{
    int _maxConvexHulls;
    if (attr.Get(&_maxConvexHulls))
    {
        maxConvexHulls = uint32_t(_maxConvexHulls);
    }    
}

// Helper method to initialize the maximum number of spheres and add it to the MeshKey CRC
void initMaxSpheres(const UsdAttribute& attr, uint32_t &maxSpheres)
{
    int _maxSpheres;
    if (attr.Get(&_maxSpheres))
    {
        maxSpheres = uint32_t(_maxSpheres);
    }    
}

void initSeedCount(const UsdAttribute& attr, uint32_t &seedCount)
{
    int _seedCount;
    if (attr.Get(&_seedCount))
    {
        seedCount = uint32_t(_seedCount);
    }    
}

void initFillMode(const UsdAttribute& attr, SphereFillMode::Enum&fillMode)
{
    static TfToken flood("flood");
    static TfToken raycast("raycast");
    static TfToken surface("surface");
    TfToken _fillMode;
    if (attr.Get(&_fillMode))
    {
        if ( _fillMode == flood )
        {
            fillMode = SphereFillMode::eFLOOD;
        }
        else if ( _fillMode == raycast )
        {
            fillMode = SphereFillMode::eRAYCAST;
        }
        else if ( _fillMode == surface )
        {
            fillMode = SphereFillMode::eSURFACE;
        }
    }    
}

// Helper method to initialize the voxel resolution and add it to the MeshKey CRC
void initVoxelResolution(const UsdAttribute& attr, uint32_t &voxelResolution)
{
    int _voxelResolution;
    if (attr.Get(&_voxelResolution))
    {
        voxelResolution = uint32_t(_voxelResolution);
    }    
}

// Helper method to initialize the mesh simplification metric value and add it to the MeshKey CRC
void initUseShrinkwrap(const UsdAttribute& attr, bool &useShrinkWrap)
{
    // Get the simplification metric
    attr.Get(&useShrinkWrap);    
}

// Helper method to initialize the volume error percentage threshold and add it to the MeshKey CRC
void initErrorPercentage(const UsdAttribute& attr, float &errorPercentage)
{
    // Get the error percentage attribute if it exists
    attr.Get(&errorPercentage);    
}

// Helper method to initialize the minimum collision thickness value and add it to the MeshKey CRC
void initMinThickness(const UsdAttribute& attr, float &minThickness)
{
    // Get the min thickness attribute if it exists
    attr.Get(&minThickness);    
}

// Helper method to initialize the mesh simplification metric value and add it to the MeshKey CRC
void initSimplificationMetric(const UsdAttribute& attr, float &simplificationMetric)
{
    // Get the simplification metric
    attr.Get(&simplificationMetric);    
}

// Helper method to initialize the mesh weld tolerance value and add it to the MeshKey CRC
void initWeldToleranceMetric(const UsdAttribute& attr, float &weldTolerance)
{
    // Get the simplification metric
    attr.Get(&weldTolerance);
    if (isnan(weldTolerance))
    {
        weldTolerance = -FLT_MAX;
    }    
}

void fillCookingRequest(omni::physx::PhysxCookingComputeRequest& request, const UsdPrim& prim, UsdTimeCode time)
{
    // If we have the meshKey in cache, we use it, saving omni.physx.cooking from recomputing it
    gMeshKeyCache.getMeshKey(prim.GetPath(), request.meshKey);
    request.primStageId = UsdUtilsStageCache::Get().GetId(prim.GetStage()).ToLongInt();
    request.primId = asInt(prim.GetPath());
    request.primTimeCode = time.GetValue();
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeAsynchronously, false);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kComputeGPUCookingData, true);
    request.options.setFlag(omni::physx::PhysxCookingComputeRequest::Options::kExecuteCookingOnGPU, false);
    request.mode = omni::physx::PhysxCookingComputeRequest::eMODE_COMPUTE_CRC;
}

void setToDefault(UsdStageWeakPtr stage, PhysxShapeDesc& desc)
{
    double metersPerUnit = UsdGeomGetStageMetersPerUnit(stage);
    float tolerancesLength = float(1.0f / metersPerUnit);

    if (desc.type == ePlaneShape)
    {
        desc.contactOffset = 0.02f * tolerancesLength;
    }
    else
    {
        // A.B. for non plane shapes we recompute the contactOffset in the physx integration
        // this is very far from ideal, we need a way to display this information to the user
        desc.contactOffset = -1.0f;
    }

    desc.restOffset = 0.0f;
    desc.torsionalPatchRadius = 0.0f;
    desc.minTorsionalPatchRadius = 0.0f;
    desc.isTrigger = false;
    desc.isTriggerUsdOutput = false;
}

bool fillPhysxShapeDesc(AttachedStage& attachedStage, const ShapeDesc& inDesc, PhysxShapeDesc& outDesc, CollisionPairVector& filteredPairs)
{
    setToDefault(inDesc.usdPrim.GetStage(), outDesc);

    // generic params
    const SdfPath& primPath = inDesc.usdPrim.GetPrimPath();
    outDesc.collisionEnabled = inDesc.collisionEnabled;
    outDesc.collisionGroup = getCollisionGroup(attachedStage, primPath);
    outDesc.rigidBody = inDesc.rigidBody;

    GfVec3ToFloat3(inDesc.localPos, outDesc.localPos);
    GfQuatToFloat4(inDesc.localRot, outDesc.localRot);
    GfVec3ToFloat3(inDesc.localScale, outDesc.localScale);

    for (size_t i = 0; i < inDesc.filteredCollisions.size(); i++)
    {
        filteredPairs.push_back(std::make_pair(primPath, inDesc.filteredCollisions[i]));
    }

    // physx params
    const PhysxSchemaPhysxCollisionAPI physxCollisionAPI =
        PhysxSchemaPhysxCollisionAPI::Get(attachedStage.getStage(), primPath);
    if (physxCollisionAPI)
    {
        float contactOffset = outDesc.contactOffset;
        float restOffset = outDesc.restOffset;
        {
            UsdAttribute attribute = physxCollisionAPI.GetContactOffsetAttr();
            if (attribute && attribute.HasAuthoredValue())
            {
                float attrVal;
                attribute.Get(&attrVal);
                if (isinf((float)attrVal))
                    contactOffset = -1.0f;
                else
                {
                    if (attrVal >= 0.0f && attrVal <= FLT_MAX)
                    {
                        contactOffset = attrVal;
                    }
                }

                // check for time samples
                if (attribute.GetNumTimeSamples() > 1)
                {
                    attachedStage.registerTimeSampledAttribute(attribute.GetPath(), updateShapeContactOffset);
                }
            }
        }

        {
            {
                UsdAttribute attribute = physxCollisionAPI.GetRestOffsetAttr();
                if (attribute && attribute.HasAuthoredValue())
                {
                    float attrVal;
                    attribute.Get(&attrVal);
                    if (isinf((float)attrVal))
                        restOffset = 0.0f;
                    else
                    {
                        if (attrVal >= -FLT_MAX && attrVal <= FLT_MAX)
                        {
                            restOffset = attrVal;
                        }
                    }
                    // check for time samples
                    if (attribute.GetNumTimeSamples() > 1)
                    {
                        attachedStage.registerTimeSampledAttribute(attribute.GetPath(), updateShapeRestOffset);
                    }
                }
            }
        }

        if (contactOffset >= restOffset)
            outDesc.contactOffset = contactOffset;

        if (restOffset < contactOffset)
            outDesc.restOffset = restOffset;

        getAttribute(outDesc.torsionalPatchRadius, physxCollisionAPI.GetTorsionalPatchRadiusAttr(), 0.0f, FLT_MAX, updateShapeTorsionalPatchRadius);
        getAttribute(
            outDesc.minTorsionalPatchRadius, physxCollisionAPI.GetMinTorsionalPatchRadiusAttr(), 0.0f, FLT_MAX, updateShapeMinTorsionalPatchRadius);
    }

    const PhysxSchemaPhysxTriggerAPI physxTriggerAPI = PhysxSchemaPhysxTriggerAPI::Get(attachedStage.getStage(), primPath);
    if (physxTriggerAPI)
    {
        outDesc.isTrigger = true;
        const PhysxSchemaPhysxTriggerStateAPI physxTriggerStateAPI = PhysxSchemaPhysxTriggerStateAPI::Get(attachedStage.getStage(), primPath);
        if (physxTriggerStateAPI)
        {
            outDesc.isTriggerUsdOutput = true;
        }
    }

    for (size_t i = 0; i < inDesc.simulationOwners.size(); i++)
    {
        const ObjectId entry = attachedStage.getObjectDatabase()->findEntry(inDesc.simulationOwners[i], eScene);
        if (entry != kInvalidObjectId)
            outDesc.sceneIds.push_back(entry);
    }

    if (!inDesc.simulationOwners.empty() && outDesc.sceneIds.empty())
    {
        return false;
    }
    return true;
}

void processMeshesToMerge(UsdGeomXformCache& xfCache, const UsdPrim& prim, const SdfPathSet& paths, MergeMeshDesc& meshDesc, UsdTimeCode time)
{
    bool leftHanded = false;
    bool rightHanded = false;

    struct MeshData
    {
        bool valid;

        SdfPath path;

        VtArray<GfVec3f> pointsValue;

        VtArray<int> indicesValue;
        VtArray<int> facesValue;
        VtArray<int> holesValue;

        UsdGeomMesh usdMesh;
        UsdPrim usdPrim;

        bool changeIndicesOrder;

        size_t pointsOffset;
        size_t indicesOffset;
        size_t facesOffset;
        size_t holesOffset;
    };

    std::vector<MeshData> meshDataVector;
    meshDataVector.resize(paths.size());

    size_t pathIndex = 0;
    for (const SdfPath& path : paths)
    {
        meshDataVector[pathIndex].path = path;
        meshDataVector[pathIndex].valid = false;
        meshDataVector[pathIndex].pointsOffset = 0;
        meshDataVector[pathIndex].indicesOffset = 0;
        meshDataVector[pathIndex].facesOffset = 0;
        meshDataVector[pathIndex].holesOffset = 0;
        pathIndex++;
    }

    ITasking* tasking = carb::getCachedInterface<ITasking>();
    const size_t minBatchSize = 10;
    const size_t numThreads = tasking->getDesc().threadCount;
    const size_t numDataToProcess = meshDataVector.size();
    const size_t threadBatchSize = numDataToProcess/numThreads;
    const size_t batchSize = threadBatchSize > minBatchSize ? threadBatchSize : minBatchSize;
    const size_t numBatches = (numDataToProcess / batchSize) + 1;

    UsdStageWeakPtr stage = prim.GetStage();

    auto&& batchGatherFunc = [numDataToProcess, batchSize, numBatches, &meshDataVector, stage, prim, time,
        &leftHanded, &rightHanded, &meshDesc](size_t batchIndex)
    {
        CARB_PROFILE_ZONE(0, "processMeshesToMerge:gatherMeshData");
        const size_t startIndex = batchIndex * batchSize;
        const size_t endIndex = (batchIndex == (numBatches - 1)) ? numDataToProcess : (batchIndex + 1) * batchSize;
        for (size_t dataIndex = startIndex; dataIndex < endIndex; dataIndex++)
        {
            MeshData& meshData = meshDataVector[dataIndex];
            const SdfPath& mergePath = meshData.path;
            const UsdPrim usdPrim = stage->GetPrimAtPath(mergePath);
            if (!usdPrim || !usdPrim.IsA<UsdGeomMesh>())
                continue;

            const UsdGeomMesh usdMesh(usdPrim);

            if (prim != usdPrim && usdPrim.HasAPI<UsdPhysicsCollisionAPI>())
            {
                CARB_LOG_WARN("Prim (%s) belongs to a mesh merge collision and also standalone collision.", usdPrim.GetPrimPath().GetText());
            }

            VtArray<GfVec3f> pointsValue;
            usdMesh.GetPointsAttr().Get(&pointsValue, time);

            VtArray<int> indicesValue;
            VtArray<int> facesValue;
            usdMesh.GetFaceVertexIndicesAttr().Get(&indicesValue, time);
            usdMesh.GetFaceVertexCountsAttr().Get(&facesValue, time);

            VtArray<int> holesValue;
            usdMesh.GetHoleIndicesAttr().Get(&holesValue, time);

            if (pointsValue.size() && indicesValue.size() && facesValue.size())
            {
                TfToken windingOrient = UsdGeomTokens->rightHanded;
                usdMesh.GetOrientationAttr().Get(&windingOrient);

                bool changeIndicesOrder = false;
                if (!leftHanded && !rightHanded)
                {
                    if (windingOrient == UsdGeomTokens->leftHanded)
                    {
                        leftHanded = true;
                    }
                    else
                    {
                        rightHanded = true;
                    }
                }
                else
                {
                    if (windingOrient == UsdGeomTokens->leftHanded && rightHanded)
                    {
                        changeIndicesOrder = true;
                    }
                    else if (windingOrient != UsdGeomTokens->leftHanded && leftHanded)
                    {
                        changeIndicesOrder = true;
                    }
                }
                
                meshData.pointsValue = pointsValue;
                meshData.indicesValue = indicesValue;
                meshData.facesValue = facesValue;
                meshData.holesValue = holesValue;
                meshData.usdMesh = usdMesh;
                meshData.usdPrim = usdPrim;
                meshData.changeIndicesOrder = changeIndicesOrder;
                meshData.valid = true;
            }
        }
    };

    {
        tasking->parallelFor(size_t(0), numBatches, batchGatherFunc);
    }

    size_t pointsTotalSize = 0;
    size_t facesTotalSize = 0;
    size_t indicesTotalSize = 0;
    size_t holesTotalSize = 0;
    {     
        for (MeshData& meshData : meshDataVector)
        {
            if (!meshData.valid)
                continue;
            
            meshData.pointsOffset = pointsTotalSize;
            pointsTotalSize += meshData.pointsValue.size();

            meshData.facesOffset = facesTotalSize;
            facesTotalSize += meshData.facesValue.size();

            meshData.indicesOffset = indicesTotalSize;
            indicesTotalSize += meshData.indicesValue.size();

            meshData.holesOffset = holesTotalSize;
            holesTotalSize += meshData.holesValue.size();
        }
        meshDesc.points.resize(pointsTotalSize);
        meshDesc.faces.resize(facesTotalSize);
        meshDesc.indices.resize(indicesTotalSize);
        meshDesc.holes.resize(holesTotalSize);
    }

    auto&& computeFunc = [numDataToProcess, batchSize, numBatches, meshDataVector, prim, &meshDesc](size_t batchIndex)
    {
        CARB_PROFILE_ZONE(0, "processMeshesToMerge:setMeshData");
        const size_t startIndex = batchIndex * batchSize;
        const size_t endIndex = (batchIndex == (numBatches - 1)) ? numDataToProcess : (batchIndex + 1) * batchSize;
        for (size_t dataIndex = startIndex; dataIndex < endIndex; dataIndex++)
        {
            const MeshData& meshData = meshDataVector[dataIndex];
            if (!meshData.valid)
                continue;

            const GfMatrix4d parentWorld = UsdGeomXform(prim).ComputeLocalToWorldTransform(UsdTimeCode::Default());
            const GfMatrix4d primWorld = UsdGeomXform(meshData.usdPrim).ComputeLocalToWorldTransform(UsdTimeCode::Default());                        
            const GfMatrix4d relMatrix = primWorld * parentWorld.GetInverse();
            
            for (size_t i = 0; i < meshData.pointsValue.size(); i++)
            {
                const GfVec3f newPoint = relMatrix.Transform(meshData.pointsValue[i]);
                meshDesc.points[meshData.pointsOffset + i] = newPoint;
            }
            memcpy(meshDesc.faces.data() + meshData.facesOffset, meshData.facesValue.data(), sizeof(int) * meshData.facesValue.size());


            if (!meshData.changeIndicesOrder)
            {
                for (size_t i = 0; i < meshData.indicesValue.size(); i++)
                {
                    meshDesc.indices[meshData.indicesOffset + i] = meshData.indicesValue[i] + (int)meshData.pointsOffset;
                }
            }
            else
            {
                size_t faceOffset = 0;
                for (size_t j = 0; j < meshData.facesValue.size(); j++)
                {
                    const int faceIndices = meshData.facesValue[j];
                    if (faceIndices > 0)
                    {
                        size_t newIndex = 0;
                        for (size_t i = faceOffset + faceIndices - 1; i >= faceOffset; i--)
                        {
                            meshDesc.indices[meshData.indicesOffset + faceOffset + newIndex] =
                                meshData.indicesValue[faceOffset + i] + (int)meshData.pointsOffset;
                            newIndex++;
                        }
                        faceOffset += faceIndices;
                    }
                }
            }

            if (!meshData.holesValue.empty())
            {
                for (size_t i = 0; i < meshData.holesValue.size(); i++)
                {
                    meshDesc.holes[meshData.holesOffset + i] = meshData.holesValue[i] + (int)meshData.facesOffset;
                }
            }
        }
    };

    {
        tasking->parallelFor(size_t(0), numBatches, computeFunc);
    }

}

void gatherMeshesToMerge(const UsdPrim& prim, SdfPathSet& paths)
{
    PhysxSchemaPhysxMeshMergeCollisionAPI mergeAPI(prim);
    CARB_ASSERT(mergeAPI);

    const UsdCollectionAPI collection = mergeAPI.GetCollisionMeshesCollectionAPI();
    const UsdCollectionMembershipQuery query = collection.ComputeMembershipQuery();
    paths = UsdCollectionAPI::ComputeIncludedPaths(query,
        prim.GetStage(), UsdTraverseInstanceProxies());
}

MergeMeshPhysxShapeDesc* processMeshCollision(AttachedStage& attachedStage,
    const omni::physics::schema::ShapeDesc& shapeDesc, const Float3& scale, bool doubleSided,
    const SdfPath& meshPath, const VtArray<GfVec3f>& pointsValueIn, CollisionPairVector& filteredPairs, bool ignoreOwners,
    omni::physx::PhysxCookingComputeRequest& request, UsdTimeCode time)
{
    IPhysxCookingService* cookingService = carb::getCachedInterface<IPhysxCookingService>();
    MergeMeshPhysxShapeDesc* desc = nullptr;    


    // Get approximation type
    TfToken approximationType = UsdPhysicsTokens.Get()->none;
    const UsdPhysicsMeshCollisionAPI colMeshAPI = UsdPhysicsMeshCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
    if (colMeshAPI)
    {
        colMeshAPI.GetApproximationAttr().Get(&approximationType);
    }
    const bool useConvexHull = (approximationType == UsdPhysicsTokens.Get()->convexHull);
    const bool useBoundingSphere = (approximationType == UsdPhysicsTokens.Get()->boundingSphere);
    const bool useBoundingCube = (approximationType == UsdPhysicsTokens.Get()->boundingCube);
    const bool useConvexDecomposition = (approximationType == PhysxSchemaTokens.Get()->convexDecomposition);
    const bool useSphereFill = (approximationType == PhysxSchemaTokens.Get()->sphereFill);
    TriangleMeshMode::Enum mode = TriangleMeshMode::eORIGINAL_TRIANGLES;
    const bool useMeshSimplification = (approximationType == UsdPhysicsTokens.Get()->meshSimplification);
    const bool useMeshSDF = (approximationType == PhysxSchemaTokens.Get()->sdf);
    const bool useMeshTriangle = (approximationType == UsdPhysicsTokens->none);
            
    if (useMeshSimplification)
    {
        mode = TriangleMeshMode::eQUADRIC_SIMPLIFICATION;
    }

    // A.B. TODO - ideally we don't want to copy the verts and additional data here, we should ask the physics
    // implementation if the mesh representation already exists and if yes, then pass down only the CRC that is enough
    // as an identifier
    if (useConvexHull)
    {
        ConvexMeshPhysxShapeDesc* convexMeshDesc = ICE_PLACEMENT_NEW(ConvexMeshPhysxShapeDesc)();
        convexMeshDesc->meshScale = scale;
        convexMeshDesc->convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexMeshDesc->meshScale);
        convexMeshDesc->meshPath = meshPath;

        const PhysxSchemaPhysxConvexHullCollisionAPI physxColMeshAPI = PhysxSchemaPhysxConvexHullCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
        if (physxColMeshAPI)
        {
            initMinThickness(physxColMeshAPI.GetMinThicknessAttr(), convexMeshDesc->convexCookingParams.minThickness);
            initMaxHullVertices(physxColMeshAPI.GetHullVertexLimitAttr(), convexMeshDesc->convexCookingParams.maxHullVertices);
        }
        desc = convexMeshDesc;
        request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result)
        {
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
                return;
            convexMeshDesc->crc = result.cookedDataCRC;
            convexMeshDesc->meshKey = result.meshKey;
            gMeshKeyCache.setMeshKey(convexMeshDesc->meshPath, convexMeshDesc->meshKey);
        };
        cookingService->requestConvexMeshCookedData(nullptr, request, convexMeshDesc->convexCookingParams);
    }
    else if (useBoundingSphere)
    {
        VtArray<GfVec3f> pointsValue;
        pointsValue.resize(pointsValueIn.size());

        for (size_t i = 0; i < pointsValueIn.size(); i++)
        {
            pointsValue[i][0] = pointsValueIn[i][0] * scale.x;
            pointsValue[i][1] = pointsValueIn[i][1] * scale.y;
            pointsValue[i][2] = pointsValueIn[i][2] * scale.z;
        }        

        desc = computeBoundingSphereShape(pointsValue);
    }
    else if (useBoundingCube)
    {
        VtArray<GfVec3f> pointsValue;
        pointsValue.resize(pointsValueIn.size());

        for (size_t i = 0; i < pointsValueIn.size(); i++)
        {
            pointsValue[i][0] = pointsValueIn[i][0] * scale.x;
            pointsValue[i][1] = pointsValueIn[i][1] * scale.y;
            pointsValue[i][2] = pointsValueIn[i][2] * scale.z;
        }

        desc = computeBoundingBoxShape(pointsValue);
    }
    else
    {
        TriangleMeshPhysxShapeDesc* meshDesc = nullptr;
        ConvexMeshDecompositionPhysxShapeDesc* convexDesc = nullptr;
        SpherePointsPhysxShapeDesc *sphereApproxDesc = nullptr;
        if (useConvexDecomposition)
        {
            convexDesc = ICE_PLACEMENT_NEW(ConvexMeshDecompositionPhysxShapeDesc)();
            meshDesc = static_cast<TriangleMeshPhysxShapeDesc*>(convexDesc);
        }
        else if (useSphereFill)
        {
            sphereApproxDesc = ICE_PLACEMENT_NEW(SpherePointsPhysxShapeDesc)();
            meshDesc = static_cast<TriangleMeshPhysxShapeDesc*>(sphereApproxDesc);
        }
        else
        {
            meshDesc = ICE_PLACEMENT_NEW(TriangleMeshPhysxShapeDesc)();
            meshDesc->triangleMeshCookingParams.mode = mode;
        }
        meshDesc->sdfMeshCookingParams.sdfResolution = 0;
        meshDesc->meshPath = meshPath;
        meshDesc->meshScale = scale;

        bool checkValidity = false;

        meshDesc->doubleSided = doubleSided;
        bool meshCRCComputedSuccessfully = false;
        request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
            if (result.result != omni::physx::PhysxCookingResult::eVALID)
                return;
            meshCRCComputedSuccessfully = true;
            meshDesc->crc = result.cookedDataCRC;
            meshDesc->meshKey = result.meshKey;
            gMeshKeyCache.setMeshKey(meshDesc->meshPath, meshDesc->meshKey);
        };

        if (useConvexDecomposition)
        {
            const PhysxSchemaPhysxConvexDecompositionCollisionAPI physxColMeshAPI = PhysxSchemaPhysxConvexDecompositionCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
            ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionDesc = (ConvexMeshDecompositionPhysxShapeDesc*)meshDesc;
            initMinThickness(physxColMeshAPI.GetMinThicknessAttr(), convexDecompositionDesc->convexDecompositionCookingParams.minThickness);
            initMaxConvexHulls(physxColMeshAPI.GetMaxConvexHullsAttr(), convexDecompositionDesc->convexDecompositionCookingParams.maxConvexHulls);
            initMaxHullVertices(physxColMeshAPI.GetHullVertexLimitAttr(), convexDecompositionDesc->convexDecompositionCookingParams.maxHullVertices);
            initVoxelResolution(physxColMeshAPI.GetVoxelResolutionAttr(), convexDecompositionDesc->convexDecompositionCookingParams.voxelResolution);
            initErrorPercentage(physxColMeshAPI.GetErrorPercentageAttr(), convexDecompositionDesc->convexDecompositionCookingParams.errorPercentage);
            initUseShrinkwrap(physxColMeshAPI.GetShrinkWrapAttr(), convexDecompositionDesc->convexDecompositionCookingParams.shrinkWrap);
            request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result)
            {
                if (result.result != omni::physx::PhysxCookingResult::eVALID)
                    return;
                meshCRCComputedSuccessfully = true;
                convexDecompositionDesc->crc = result.cookedDataCRC;
                convexDecompositionDesc->meshKey = result.meshKey;
                gMeshKeyCache.setMeshKey(convexDecompositionDesc->meshPath, convexDecompositionDesc->meshKey);
            };
            convexDecompositionDesc->convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDecompositionDesc->meshScale);
            cookingService->requestConvexMeshDecompositionCookedData(nullptr, request, convexDecompositionDesc->convexDecompositionCookingParams);
        }
        else if ( useSphereFill )
        {
            // A.B. Sphere fill supports only uniform scaling
            if (!GfIsClose(scale.x, scale.y, 0.001f) ||
                !GfIsClose(scale.x, scale.z, 0.001f) ||
                !GfIsClose(scale.z, scale.y, 0.001f))
            {
                CARB_LOG_WARN("Sphere points approximation does not support non-uniform scale, using max radius results might not be correct: %s", shapeDesc.usdPrim.GetPrimPath().GetText());
            }

            const PhysxSchemaPhysxSphereFillCollisionAPI physxColMeshAPI = PhysxSchemaPhysxSphereFillCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
            initMaxSpheres(physxColMeshAPI.GetMaxSpheresAttr(), sphereApproxDesc->sphereFillCookingParams.maxSpheres);
            initSeedCount(physxColMeshAPI.GetSeedCountAttr(), sphereApproxDesc->sphereFillCookingParams.seedCount);
            initVoxelResolution(physxColMeshAPI.GetVoxelResolutionAttr(), sphereApproxDesc->sphereFillCookingParams.voxelResolution);
            initFillMode(physxColMeshAPI.GetFillModeAttr(), sphereApproxDesc->sphereFillCookingParams.fillMode);
            request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result)
            {
                if (result.result != omni::physx::PhysxCookingResult::eVALID)
                    return;
                meshCRCComputedSuccessfully = true;
                sphereApproxDesc->crc = result.cookedDataCRC;
                sphereApproxDesc->meshKey = result.meshKey;
                gMeshKeyCache.setMeshKey(sphereApproxDesc->meshPath, sphereApproxDesc->meshKey);
            };
            cookingService->requestSphereFillCookedData(nullptr, request, sphereApproxDesc->sphereFillCookingParams);
        }
        else if(useMeshSimplification)
        {
            const PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI physxColMeshAPI = PhysxSchemaPhysxTriangleMeshSimplificationCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
            initSimplificationMetric(physxColMeshAPI.GetSimplificationMetricAttr(), meshDesc->triangleMeshCookingParams.simplificationMetric);
            initWeldToleranceMetric(physxColMeshAPI.GetWeldToleranceAttr(), meshDesc->triangleMeshCookingParams.meshWeldTolerance);
            cookingService->requestTriangleMeshCookedData (nullptr, request, meshDesc->triangleMeshCookingParams); // onCookingFinished already setup earlier
            checkValidity = true;
        }
        else if(useMeshTriangle)
        {
            // Triangle mesh approximation
            const PhysxSchemaPhysxTriangleMeshCollisionAPI physxColMeshAPI = PhysxSchemaPhysxTriangleMeshCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
            initWeldToleranceMetric(physxColMeshAPI.GetWeldToleranceAttr(), meshDesc->triangleMeshCookingParams.meshWeldTolerance);
            cookingService->requestTriangleMeshCookedData (nullptr, request, meshDesc->triangleMeshCookingParams); // onCookingFinished already setup earlier
            checkValidity = true;
        }
        else if(useMeshSDF)
        {
            {
                const PhysxSchemaPhysxTriangleMeshCollisionAPI physxColMeshAPI = PhysxSchemaPhysxTriangleMeshCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
                initWeldToleranceMetric(physxColMeshAPI.GetWeldToleranceAttr(), meshDesc->triangleMeshCookingParams.meshWeldTolerance);
            }
               

            const PhysxSchemaPhysxSDFMeshCollisionAPI physxSDFCollisionAPI = PhysxSchemaPhysxSDFMeshCollisionAPI::Get(attachedStage.getStage(), shapeDesc.usdPrim.GetPrimPath());
            bool isValidSDF = false;

            // if triMeshCollisionAPI, either it is static, or it must have a valid SDF.
            if (physxSDFCollisionAPI)
            {
                int resolution = 0;
                physxSDFCollisionAPI.GetSdfResolutionAttr().Get(&resolution);
                uint32_t sdfResolution = uint32_t(resolution);

                if (sdfResolution > 0) // valid SDF
                {
                    isValidSDF = true;
                    meshDesc->sdfMeshCookingParams.sdfResolution = sdfResolution;

                    int subgridResolution = 0;
                    physxSDFCollisionAPI.GetSdfSubgridResolutionAttr().Get(&subgridResolution);
                    meshDesc->sdfMeshCookingParams.sdfSubgridResolution = subgridResolution;                            

                    int sdfBitsPerSubgridPixel = 16;
                    TfToken bitsPerSubgridPixel;
                    physxSDFCollisionAPI.GetSdfBitsPerSubgridPixelAttr().Get(&bitsPerSubgridPixel);
                    if (bitsPerSubgridPixel == PhysxSchemaTokens.Get()->BitsPerPixel8)
                        sdfBitsPerSubgridPixel = 8;
                    else if (bitsPerSubgridPixel == PhysxSchemaTokens.Get()->BitsPerPixel16)
                        sdfBitsPerSubgridPixel = 16;
                    else if (bitsPerSubgridPixel == PhysxSchemaTokens.Get()->BitsPerPixel32)
                        sdfBitsPerSubgridPixel = 32;
                    meshDesc->sdfMeshCookingParams.sdfBitsPerSubgridPixel = sdfBitsPerSubgridPixel;                            

                    float narrowBandThickness = 0.0f;
                    physxSDFCollisionAPI.GetSdfNarrowBandThicknessAttr().Get(&narrowBandThickness);
                    meshDesc->sdfMeshCookingParams.sdfNarrowBandThickness = narrowBandThickness;                            

                    float sdfMargin;
                    physxSDFCollisionAPI.GetSdfMarginAttr().Get(&sdfMargin);
                    meshDesc->sdfMeshCookingParams.sdfMargin = sdfMargin;

                    bool sdfEnableRemeshing;
                    physxSDFCollisionAPI.GetSdfEnableRemeshingAttr().Get(&sdfEnableRemeshing);
                    meshDesc->sdfMeshCookingParams.sdfEnableRemeshing = sdfEnableRemeshing;

                    float sdfTriangleCountReductionFactor;
                    physxSDFCollisionAPI.GetSdfTriangleCountReductionFactorAttr().Get(&sdfTriangleCountReductionFactor);
                    meshDesc->sdfMeshCookingParams.sdfTriangleCountReductionFactor = sdfTriangleCountReductionFactor;
                }
            }

            if (isValidSDF)
            {
                // onFinished already setup earlier
                cookingService->requestSdfMeshCookedData(
                    nullptr, request, meshDesc->triangleMeshCookingParams, meshDesc->sdfMeshCookingParams);
            }
            else
            {
                // onFinished already setup earlier
                cookingService->requestTriangleMeshCookedData (nullptr, request, meshDesc->triangleMeshCookingParams);
                checkValidity = true;
            }
        }
        else
        {
            CARB_LOG_ERROR("PhysicsUSD: Prim at path %s is using unknown value for physics:approximation attribute.", shapeDesc.usdPrim.GetPrimPath().GetText());
            ICE_FREE(meshDesc);
            return nullptr;
        }

        if (checkValidity)
        {
            // check if this is really a dynamic rigid body - fallback to convex in this case.
            if (shapeDesc.rigidBody != SdfPath())
            {
                bool isKinematic = false;
                bool isEnabled = true;

                UsdPhysicsRigidBodyAPI rigidBodyAPI = UsdPhysicsRigidBodyAPI::Get(attachedStage.getStage(), shapeDesc.rigidBody);
                if (rigidBodyAPI)
                {
                    rigidBodyAPI.GetKinematicEnabledAttr().Get(&isKinematic);
                    rigidBodyAPI.GetRigidBodyEnabledAttr().Get(&isEnabled);
                }
                if (isKinematic == false && isEnabled == true)
                {
                    std::string errorStr =
                        "PhysicsUSD: Parse collision - triangle mesh collision (approximation None/MeshSimplification) cannot be a part of a dynamic body, falling back to convexHull approximation: "
                        + shapeDesc.usdPrim.GetPrimPath().GetString() +
                        ".For dynamic collision please use approximations : convex hull, convex decomposition, box, sphere or SDF approximation.";
                    PhysXUsdPhysicsInterface::reportLoadError(
                        ErrorCode::eError, errorStr.c_str());
                    ICE_FREE(meshDesc);
                    ConvexMeshPhysxShapeDesc* convexMeshDesc = ICE_PLACEMENT_NEW(ConvexMeshPhysxShapeDesc)();
                    convexMeshDesc->meshScale = scale;
                    convexMeshDesc->convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexMeshDesc->meshScale);
                    convexMeshDesc->meshPath = meshPath;
                    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result)
                    {
                        if (result.result != omni::physx::PhysxCookingResult::eVALID)
                            return;
                        meshCRCComputedSuccessfully = true;
                        convexMeshDesc->crc = result.cookedDataCRC;
                        convexMeshDesc->meshKey = result.meshKey;
                        gMeshKeyCache.setMeshKey(convexMeshDesc->meshPath, convexMeshDesc->meshKey);
                    };
                    cookingService->requestConvexMeshCookedData(nullptr, request, convexMeshDesc->convexCookingParams);

                    desc = convexMeshDesc;
                }
            }
        }

        if (!desc)
        {
            desc = meshDesc;
        }
        if(!meshCRCComputedSuccessfully)
        {
            CARB_LOG_ERROR("PhysicsUSD: Prim at path %s failed to compute mesh CRC for cooking", shapeDesc.usdPrim.GetPrimPath().GetText());
            ICE_FREE(desc);
            desc = nullptr;
        }
    }

    if (desc)
    {
        if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *desc, filteredPairs) && !ignoreOwners)
        {
            ICE_FREE(desc);
            return nullptr;
        }
        return desc;
    }

    return nullptr;
}

PhysxShapeDesc* parseCollisionDescInternal(AttachedStage& attachedStage, UsdGeomXformCache& xfCache,
    const omni::physics::schema::ShapeDesc& shapeDesc, CollisionPairVector& filteredPairs, bool ignoreOwners)
{
    UsdStageWeakPtr stage = attachedStage.getStage();
    switch (shapeDesc.type)
    {
        case omni::physics::schema::ObjectType::eSpherePointsShape:
        {
            const SpherePointsShapeDesc &inShape = (const SpherePointsShapeDesc&)shapeDesc;
            SpherePointsPhysxShapeDesc *outShape = ICE_PLACEMENT_NEW(SpherePointsPhysxShapeDesc);
            outShape->spheres.resize(inShape.spherePoints.size());
            for (size_t i=0; i<inShape.spherePoints.size(); i++)
            {
                outShape->spheres[i].position = *(const carb::Float3 *)&inShape.spherePoints[i].center;
                outShape->spheres[i].radius = inShape.spherePoints[i].radius;
            }
            if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *outShape, filteredPairs) && !ignoreOwners)
            {
                ICE_FREE(outShape);
                return nullptr;
            }
            return outShape;
        }
        break;
        case omni::physics::schema::ObjectType::eSphereShape:
        {
            const SphereShapeDesc& inSphere = (const SphereShapeDesc&) shapeDesc;
            SpherePhysxShapeDesc* sphereDesc = ICE_PLACEMENT_NEW(SpherePhysxShapeDesc)(inSphere.radius);
            if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *sphereDesc, filteredPairs) && !ignoreOwners)
            {
                ICE_FREE(sphereDesc);
                return nullptr;
            }
            return sphereDesc;
        }
        break;
        case omni::physics::schema::ObjectType::eCubeShape:
        {
            const CubeShapeDesc& inBox = (const CubeShapeDesc&)shapeDesc;
            BoxPhysxShapeDesc* boxDesc = ICE_PLACEMENT_NEW(BoxPhysxShapeDesc)(inBox.halfExtents[0], inBox.halfExtents[1], inBox.halfExtents[2]);
            if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *boxDesc, filteredPairs) && !ignoreOwners)
            {
                ICE_FREE(boxDesc);
                return nullptr;
            }
            return boxDesc;
        }
        break;
        case omni::physics::schema::ObjectType::eCapsuleShape:
        {
            const CapsuleShapeDesc& inCapsule = (const CapsuleShapeDesc&)shapeDesc;
            CapsulePhysxShapeDesc* capsuleDesc = ICE_PLACEMENT_NEW(CapsulePhysxShapeDesc)(inCapsule.radius, inCapsule.halfHeight, (Axis)inCapsule.axis);
            if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *capsuleDesc, filteredPairs) && !ignoreOwners)
            {
                ICE_FREE(capsuleDesc);
                return nullptr;
            }
            return capsuleDesc;
        }
        break;
        case omni::physics::schema::ObjectType::eCylinderShape:
        {
            const CylinderShapeDesc& inCylinder = (const CylinderShapeDesc&)shapeDesc;
            CylinderPhysxShapeDesc* cylinderDesc = ICE_PLACEMENT_NEW(CylinderPhysxShapeDesc)(inCylinder.radius, inCylinder.halfHeight, (Axis)inCylinder.axis);
            if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *cylinderDesc, filteredPairs) && !ignoreOwners)
            {
                ICE_FREE(cylinderDesc);
                return nullptr;
            }

            if(shapeDesc.usdPrim.GetAttribute(obsoleteCustomGeometryAttribute))
            {
                CARB_LOG_WARN("PhysicsUSD: Prim at path %s is using obsolete 'customGeometry' attribute. To toggle convex mesh approximation for cylinders, use the physics settings option.", shapeDesc.usdPrim.GetPrimPath().GetText());
            }

            const UsdAttribute marginAttr = shapeDesc.usdPrim.GetAttribute(TfToken("physxConvexGeometry:margin"));
            if (marginAttr.HasAuthoredValue())
            {
                float margin = 0; marginAttr.Get(&margin);
                cylinderDesc->margin = fmaxf(0, margin);
            }

            return cylinderDesc;

            
        }
        break;
        case omni::physics::schema::ObjectType::eConeShape:
        {
            const ConeShapeDesc& inCone = (const ConeShapeDesc&)shapeDesc;
            ConePhysxShapeDesc* coneDesc = ICE_PLACEMENT_NEW(ConePhysxShapeDesc)(inCone.radius, inCone.halfHeight, (Axis)inCone.axis);
            if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *coneDesc, filteredPairs) && !ignoreOwners)
            {
                ICE_FREE(coneDesc);
                return nullptr;
            }

            if(shapeDesc.usdPrim.GetAttribute(obsoleteCustomGeometryAttribute))
            {
                CARB_LOG_WARN("PhysicsUSD: Prim at path %s is using obsolete 'customGeometry' attribute. To toggle convex mesh approximation for cones, use the physics settings option.", shapeDesc.usdPrim.GetPrimPath().GetText());
            }

            const UsdAttribute marginAttr = shapeDesc.usdPrim.GetAttribute(TfToken("physxConvexGeometry:margin"));
            if (marginAttr.HasAuthoredValue())
            {
                float margin = 0; marginAttr.Get(&margin);
                coneDesc->margin = fmaxf(0, margin);
            }

            return coneDesc;
        }
        break;
        case omni::physics::schema::ObjectType::eMeshShape:
        {            
            CARB_ASSERT(shapeDesc.sourceGprim.IsA<UsdGeomMesh>());
            const UsdGeomMesh& usdMesh = (const UsdGeomMesh&)(shapeDesc.sourceGprim);
            const SdfPath meshPath = shapeDesc.sourceGprim.GetPrimPath();
         
            UsdTimeCode time = UsdTimeCode::Default();

            omni::physx::PhysxCookingComputeRequest request;
            fillCookingRequest(request, shapeDesc.sourceGprim, time);

            Float3 scale = { 1.0f, 1.0f, 1.0f };

            if (!shapeDesc.masterDesc)
            {
                const GfTransform tr(xfCache.GetLocalToWorldTransform(shapeDesc.sourceGprim));
                const GfVec3d sc = tr.GetScale();
                GfVec3ToFloat3(sc, scale);
            }

            bool doubleSided = false;
            usdMesh.GetDoubleSidedAttr().Get(&doubleSided);

            VtArray<GfVec3f> pointsValue;
            {
                usdMesh.GetPointsAttr().Get(&pointsValue, time);
            }

            return processMeshCollision(attachedStage, shapeDesc, scale, doubleSided,
                meshPath, pointsValue, filteredPairs, ignoreOwners, request, time);
        }
        break;
        case omni::physics::schema::ObjectType::eCustomShape:
        {            
            if (shapeDesc.sourceGprim.IsA<UsdGeomPlane>())
            {
                PlanePhysxShapeDesc* planeDesc = ICE_PLACEMENT_NEW(PlanePhysxShapeDesc)();

                UsdGeomPlane plane(shapeDesc.sourceGprim);
                TfToken axis;
                int planeAxis = 0;
                plane.GetAxisAttr().Get(&axis);
                if (axis == UsdPhysicsTokens.Get()->y)
                {
                    planeAxis = 1;
                }
                else if (axis == UsdPhysicsTokens.Get()->z)
                {
                    planeAxis = 2;
                }
                planeDesc->axis = (Axis)planeAxis;

                if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *planeDesc, filteredPairs) && !ignoreOwners)
                {
                    {
                        ICE_FREE(planeDesc);
                        return nullptr;
                    }
                }
                return planeDesc;
            }
            else if (shapeDesc.sourceGprim.GetTypeName() == oldConvexPrim)
            {
                CARB_LOG_ERROR("Prim type ConvexMesh not supported. Construct a UsdGeomMesh with convex approximation.");
                return nullptr;
            }
            else
            {                
                const CustomShapeDesc& inCustomDesc = (const CustomShapeDesc&)shapeDesc;
                static const TfToken meshMergingCollisionAPI("PhysxMeshMergeCollisionAPI");
                if (inCustomDesc.customGeometryToken == meshMergingCollisionAPI)
                {
                    CARB_PROFILE_ZONE(0, "parseCollisionDescInternal:meshMeshCollision");
                    if (shapeDesc.masterDesc)
                    {
                        CARB_LOG_ERROR("PhysxMeshMergingCollisionAPI is not supported for prims in scene graph instancing. Prim: (%s)", shapeDesc.usdPrim.GetPrimPath().GetText());
                        return nullptr;
                    }

                    // MeshMerging collision
                    const SdfPath meshPath = shapeDesc.usdPrim.GetPrimPath();
                    UsdTimeCode time = UsdTimeCode::Default();

                    omni::physx::PhysxCookingComputeRequest request;
                    fillCookingRequest(request, shapeDesc.usdPrim, time);

                    Float3 scale = { 1.0f, 1.0f, 1.0f };
                    
                    const GfTransform tr(xfCache.GetLocalToWorldTransform(shapeDesc.usdPrim));
                    const GfVec3d sc = tr.GetScale();
                    GfVec3ToFloat3(sc, scale);

                    const bool doubleSided = false;
                    request.dataInputMode = PhysxCookingComputeRequest::DataInputMode::eINPUT_MODE_FROM_PRIM_MESH_VIEW;

                    // gather the child meshes
                    MergeMeshDesc* meshDesc = ICE_PLACEMENT_NEW(MergeMeshDesc);                   
                    SdfPathSet mergePaths;
                    {
                        CARB_PROFILE_ZONE(0, "parseCollisionDescInternal:meshMeshCollision:gatherMeshes");
                        gatherMeshesToMerge(shapeDesc.usdPrim, mergePaths);
                    }
                    {
                        CARB_PROFILE_ZONE(0, "parseCollisionDescInternal:meshMeshCollision:processMeshesToMerge");
                        processMeshesToMerge(xfCache, shapeDesc.usdPrim, mergePaths, *meshDesc, time);
                    }

                    if (meshDesc->points.size() && meshDesc->indices.size() && meshDesc->faces.size())
                    {
                        request.meshKey = MeshKey();
                        request.primMeshView.points = { (carb::Float3*)meshDesc->points.data(), meshDesc->points.size() };
                        request.primMeshView.indices = { (int32_t*)meshDesc->indices.data(), meshDesc->indices.size() };
                        request.primMeshView.faces = { (int32_t*)meshDesc->faces.data(), meshDesc->faces.size() };
                        request.primMeshView.holeIndices = { (int32_t*)meshDesc->holes.data(), meshDesc->holes.size() };

                        MergeMeshPhysxShapeDesc* mergedMeshShapeDesc = processMeshCollision(attachedStage, shapeDesc, scale, doubleSided,
                            meshPath, meshDesc->points, filteredPairs, ignoreOwners, request, time);
                        if (mergedMeshShapeDesc)
                        {
                            mergedMeshShapeDesc->mergedMesh = meshDesc;
                        }
                        else
                        {
                            ICE_FREE(meshDesc);
                        }
                        return mergedMeshShapeDesc;                        
                    }
                    else
                    {
                        ICE_FREE(meshDesc);
                        return nullptr;
                    }
                }
                else
                {
                    CustomPhysxShapeDesc* customDesc = ICE_PLACEMENT_NEW(CustomPhysxShapeDesc)();
                    customDesc->customGeometryToken = inCustomDesc.customGeometryToken;
                    if (!fillPhysxShapeDesc(attachedStage, shapeDesc, *customDesc, filteredPairs) && !ignoreOwners)
                    {
                        ICE_FREE(customDesc);
                        return nullptr;
                    }
                    return customDesc;
                }
            }
        }
        break;
        default:
            break;
    }

    return nullptr;
}

PhysxShapeDesc* parseCollisionDesc(AttachedStage& attachedStage, UsdGeomXformCache& xfCache, PathPhysXDescMap& physxDescCache,
    const SdfPath& path, const omni::physics::schema::ShapeDesc& shapeDesc, CollisionPairVector& filteredPairs,
    SdfPathVector& materials, bool ignoreOwners)
{
    UsdStageWeakPtr stage = attachedStage.getStage();
    if (shapeDesc.masterDesc)
    {
        const PhysxShapeDesc* masterDesc = nullptr;
        PathPhysXDescMap::const_iterator fit = physxDescCache.find(shapeDesc.sourceGprim.GetPrimPath());
        if (fit != physxDescCache.end())
        {
            masterDesc = (const PhysxShapeDesc*)fit->second;
        }
        else
        {
            masterDesc = parseCollisionDescInternal(attachedStage, xfCache, shapeDesc, filteredPairs, ignoreOwners);
            physxDescCache[shapeDesc.sourceGprim.GetPrimPath()] = masterDesc;
        }

        if (!masterDesc)
            return nullptr;

        const UsdPrim instancePrim = stage->GetPrimAtPath(path);
        const GfTransform tf(xfCache.GetLocalToWorldTransform(instancePrim));
        PhysxShapeDesc* newDesc = scaleShapeDesc(*masterDesc, GfVec3f(tf.GetScale()));
        GfVec3f localPos;
        GfVec3f localScale;
        GfQuatf localRot;
        if (shapeDesc.rigidBody != SdfPath())
        {
            getCollisionShapeLocalTransfrom(
                xfCache, instancePrim,
                stage->GetPrimAtPath(shapeDesc.rigidBody),
                localPos, localRot, localScale);
        }
        else
        {
            localPos = GfVec3f(tf.GetTranslation());
            localScale = GfVec3f(tf.GetScale());
            localRot = GfQuatf(tf.GetRotation().GetQuat());
        }
        GfVec3ToFloat3(localPos, newDesc->localPos);
        GfQuatToFloat4(localRot, newDesc->localRot);
        GfVec3ToFloat3(localScale, newDesc->localScale);
        newDesc->collisionGroup = getCollisionGroup(attachedStage, path);
        if (masterDesc->materials.size() < 2)
        {
            SdfPath materialPath = usdmaterialutils::getMaterialBinding(instancePrim);
            if (materialPath != SdfPath())
            {
                newDesc->materials.clear();
                materials.push_back(materialPath);
            }
        }
        return newDesc;
    }
    else
    {
        return parseCollisionDescInternal(attachedStage, xfCache, shapeDesc, filteredPairs, ignoreOwners);
    }
}

bool isCollisionShape(const UsdStageWeakPtr stage, const UsdPrim& prim)
{
    return prim.HasAPI<UsdPhysicsCollisionAPI>();
}

PhysxShapeDesc* scaleShapeDesc(const PhysxShapeDesc& inDesc, const GfVec3f& scale)
{
    PhysxShapeDesc* desc = nullptr;

    switch (inDesc.type)
    {
    case eSphereShape:
    {
        desc = ICE_PLACEMENT_NEW(SpherePhysxShapeDesc)();
        SpherePhysxShapeDesc& sphereDesc = (SpherePhysxShapeDesc&)*desc;
        sphereDesc = (const SpherePhysxShapeDesc&)inDesc;
        const float radiusScale = fmaxf(fmaxf(fabsf(float(scale[1])), fabsf(float(scale[0]))), fabsf(float(scale[2])));
        sphereDesc.radius = sphereDesc.radius * radiusScale;
    }
    break;
    case eBoxShape:
    {
        desc = ICE_PLACEMENT_NEW(BoxPhysxShapeDesc)();
        BoxPhysxShapeDesc& boxDesc = (BoxPhysxShapeDesc&)*desc;
        boxDesc = (const BoxPhysxShapeDesc&)inDesc;

        (GfVec3f&)boxDesc.halfExtents = GfCompMult((const GfVec3f&)boxDesc.halfExtents, scale);
    }
    break;
    case eCapsuleShape:
    {
        desc = ICE_PLACEMENT_NEW(CapsulePhysxShapeDesc)();
        CapsulePhysxShapeDesc& capsuleDesc = (CapsulePhysxShapeDesc&)*desc;
        capsuleDesc = (const CapsulePhysxShapeDesc&)inDesc;

        if (capsuleDesc.axis == Axis::eX)
        {
            capsuleDesc.halfHeight *= scale[0];
            capsuleDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[2]));
        }
        else if (capsuleDesc.axis == Axis::eY)
        {
            capsuleDesc.halfHeight *= scale[1];
            capsuleDesc.radius *= fmaxf(fabsf(scale[0]), fabsf(scale[2]));
        }
        else
        {
            capsuleDesc.halfHeight *= scale[2];
            capsuleDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[0]));
        }
    }
    break;
    case eCylinderShape:
    {
        desc = ICE_PLACEMENT_NEW(CylinderPhysxShapeDesc)();
        CylinderPhysxShapeDesc& cylinderDesc = (CylinderPhysxShapeDesc&)*desc;
        cylinderDesc = (const CylinderPhysxShapeDesc&)inDesc;

        if (cylinderDesc.axis == Axis::eX)
        {
            cylinderDesc.halfHeight *= scale[0];
            cylinderDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[2]));
        }
        else if (cylinderDesc.axis == Axis::eY)
        {
            cylinderDesc.halfHeight *= scale[1];
            cylinderDesc.radius *= fmaxf(fabsf(scale[0]), fabsf(scale[2]));
        }
        else
        {
            cylinderDesc.halfHeight *= scale[2];
            cylinderDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[0]));
        }
    }
    break;
    case eConeShape:
    {
        desc = ICE_PLACEMENT_NEW(ConePhysxShapeDesc)();
        ConePhysxShapeDesc& coneDesc = (ConePhysxShapeDesc&)*desc;
        coneDesc = (const ConePhysxShapeDesc&)inDesc;

        if (coneDesc.axis == Axis::eX)
        {
            coneDesc.halfHeight *= scale[0];
            coneDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[2]));
        }
        else if (coneDesc.axis == Axis::eY)
        {
            coneDesc.halfHeight *= scale[1];
            coneDesc.radius *= fmaxf(fabsf(scale[0]), fabsf(scale[2]));
        }
        else
        {
            coneDesc.halfHeight *= scale[2];
            coneDesc.radius *= fmaxf(fabsf(scale[1]), fabsf(scale[0]));
        }
    }
    break;
    case eConvexMeshShape:
    {
        desc = ICE_PLACEMENT_NEW(ConvexMeshPhysxShapeDesc)();
        ConvexMeshPhysxShapeDesc& convexDesc = (ConvexMeshPhysxShapeDesc&)*desc;
        convexDesc = (const ConvexMeshPhysxShapeDesc&)inDesc;

        (GfVec3f&)convexDesc.meshScale = GfCompMult((const GfVec3f&)convexDesc.meshScale, scale);
        convexDesc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDesc.meshScale);
    }
    break;
    case eConvexMeshDecompositionShape:
    {
        desc = ICE_PLACEMENT_NEW(ConvexMeshDecompositionPhysxShapeDesc)();
        ConvexMeshDecompositionPhysxShapeDesc& convexDecDesc = (ConvexMeshDecompositionPhysxShapeDesc&)*desc;
        convexDecDesc = (const ConvexMeshDecompositionPhysxShapeDesc&)inDesc;

        (GfVec3f&)convexDecDesc.meshScale = GfCompMult((const GfVec3f&)convexDecDesc.meshScale, scale);
        convexDecDesc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(convexDecDesc.meshScale);
    }
    break;
    case eTriangleMeshShape:
    {
        desc = ICE_PLACEMENT_NEW(TriangleMeshPhysxShapeDesc)();
        TriangleMeshPhysxShapeDesc& meshDesc = (TriangleMeshPhysxShapeDesc&)*desc;
        meshDesc = (const TriangleMeshPhysxShapeDesc&)inDesc;

        (GfVec3f&)meshDesc.meshScale = GfCompMult((const GfVec3f&)meshDesc.meshScale, scale);
    }
    break;
    case eBoundingSphereShape:
    {
        desc = ICE_PLACEMENT_NEW(BoundingSpherePhysxShapeDesc)();
        BoundingSpherePhysxShapeDesc& bsDesc = (BoundingSpherePhysxShapeDesc&)*desc;
        bsDesc = (const BoundingSpherePhysxShapeDesc&)inDesc;

        (GfVec3f&)bsDesc.positionOffset = GfCompMult((const GfVec3f&)bsDesc.positionOffset, scale);
        const float radiusScale = fmaxf(fmaxf(fabsf(float(scale[1])), fabsf(float(scale[0]))), fabsf(float(scale[2])));
        bsDesc.radius = bsDesc.radius * radiusScale;
    }
    break;
    case eBoundingBoxShape:
    {
        desc = ICE_PLACEMENT_NEW(BoundingBoxPhysxShapeDesc)();
        BoundingBoxPhysxShapeDesc& bbDesc = (BoundingBoxPhysxShapeDesc&)*desc;
        bbDesc = (const BoundingBoxPhysxShapeDesc&)inDesc;

        (GfVec3f&)bbDesc.positionOffset = GfCompMult((const GfVec3f&)bbDesc.positionOffset, scale);
        (GfVec3f&)bbDesc.halfExtents = GfCompMult((const GfVec3f&)bbDesc.halfExtents, scale);
    }
    break;
    case ePlaneShape:
    {
        desc = ICE_PLACEMENT_NEW(PlanePhysxShapeDesc)();
        PlanePhysxShapeDesc& bbDesc = (PlanePhysxShapeDesc&)*desc;
    }
    break;
    default:
        break;
    }

    if (desc)
    {
        (GfVec3f&)desc->localPos = GfCompMult((const GfVec3f&)desc->localPos, scale);
        (GfVec3f&)desc->localScale = GfCompMult((const GfVec3f&)desc->localScale, scale);
    }

    return desc;
}

void finalizeShape(AttachedStage& attachedStage, PhysxShapeDesc* desc, const SdfPathVector& materials)
{
    for (const SdfPath& materialPath : materials)
    {
        desc->materials.push_back(getMaterial(attachedStage, materialPath));
    }

}

PhysxRigidBodyDesc* createShape(AttachedStage& attachedStage, const SdfPath& path, UsdGeomXformCache& xfCache, PhysxShapeDesc* shapeDesc, const ObjectInstance* objectInstance, ObjectId* instancedShapeId)
{
    const bool hadNoRigidBody = shapeDesc->rigidBody == SdfPath();

    // If we use shape for instanced create, we should not search for existing bodies
    const ObjectId bodyId = instancedShapeId ? kInvalidObjectId : getRigidBody(attachedStage, path, *shapeDesc);
    PhysxRigidBodyDesc* bodyDesc = nullptr;
    if (shapeDesc->rigidBody == SdfPath())
    {
        bodyDesc = createStaticBody();
        bodyDesc->position = shapeDesc->localPos;
        bodyDesc->rotation = shapeDesc->localRot;
        bodyDesc->scale = shapeDesc->localScale;
        bodyDesc->sceneIds = shapeDesc->sceneIds;

        shapeDesc->localPos = { 0.0f, 0.0f , 0.0f };
        shapeDesc->localRot = { 0.0f, 0.0f , 0.0f, 1.0f };
        shapeDesc->localScale = { 1.0f, 1.0f , 1.0f };
    }
    else if (hadNoRigidBody)
    {
        // Need to re-calculate the shape TM relative to the rigid body
        UsdPrim shapePrim = attachedStage.getStage()->GetPrimAtPath(path);
        UsdPrim bodyPrim = attachedStage.getStage()->GetPrimAtPath(shapeDesc->rigidBody);
        GfVec3f localPos;
        GfVec3f localScale;
        GfQuatf localRot;
        getCollisionShapeLocalTransfrom(xfCache, shapePrim, bodyPrim,
            localPos, localRot, localScale);
        GfVec3ToFloat3(localPos, shapeDesc->localPos);
        GfQuatToFloat4(localRot, shapeDesc->localRot);
        GfVec3ToFloat3(localScale, shapeDesc->localScale);
    }

    const ObjectId id = attachedStage.getPhysXPhysicsInterface()->createShape(path, *shapeDesc, bodyId, objectInstance);
    if (id != kInvalidObjectId)
        attachedStage.getObjectDatabase()->findOrCreateEntry(path, eShape, id);

    if (instancedShapeId)
    {
        *instancedShapeId = id;
    }

    if (bodyId != kInvalidObjectId)
    {
        attachedStage.bufferRequestRigidBodyMassUpdate(attachedStage.getStage()->GetPrimAtPath(shapeDesc->rigidBody));
    }

    if (bodyDesc && id != kInvalidObjectId)
    {
        bodyDesc->shapes.push_back(id);
    }

    return bodyDesc;
}

void releaseShapeDesc(PhysxShapeDesc* desc)
{
    ICE_FREE(desc);
}

bool fillConvexMeshDesc(const UsdGeomMesh& mesh, omni::physx::usdparser::ConvexMeshPhysxShapeDesc& desc, const omni::physx::ConvexMeshCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    desc.meshPath = mesh.GetPath();

    Float3 scale = { 1.0f, 1.0f, 1.0f };
    const GfTransform tr(mesh.ComputeLocalToWorldTransform(time));
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.convexCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);

    desc.convexCookingParams = cookingParams;
    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(desc.meshPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = carb::getCachedInterface<IPhysxCookingService>();
    cookingService->requestConvexMeshCookedData(nullptr, request, desc.convexCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillConvexDecompositionDesc(const UsdGeomMesh& mesh, omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc& desc, const omni::physx::ConvexDecompositionCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    Float3 scale = { 1.0f, 1.0f, 1.0f };
    const GfTransform tr(mesh.ComputeLocalToWorldTransform(time));
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.convexDecompositionCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);
    desc.sdfMeshCookingParams.sdfResolution = 0;
    desc.meshPath = mesh.GetPath();

    desc.convexDecompositionCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(desc.meshPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = carb::getCachedInterface<IPhysxCookingService>();
    cookingService->requestConvexMeshDecompositionCookedData(nullptr, request, desc.convexDecompositionCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillSphereFillDesc(const UsdGeomMesh& mesh, omni::physx::usdparser::SpherePointsPhysxShapeDesc& desc, const omni::physx::SphereFillCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    Float3 scale = { 1.0f, 1.0f, 1.0f };
    const GfTransform tr(mesh.ComputeLocalToWorldTransform(time));
    const GfVec3d sc = tr.GetScale();
    GfVec3ToFloat3(sc, scale);
    desc.meshScale = scale;
    desc.sphereFillCookingParams.signScale = omni::physx::usdparser::scaleToSignScale(desc.meshScale);
    desc.sdfMeshCookingParams.sdfResolution = 0;
    desc.meshPath = mesh.GetPath();
    desc.sphereFillCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(desc.meshPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = carb::getCachedInterface<IPhysxCookingService>();
    cookingService->requestSphereFillCookedData(nullptr, request, desc.sphereFillCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillTriangleMeshDesc(const UsdGeomMesh& mesh, omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc, const omni::physx::TriangleMeshCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    desc.meshPath = mesh.GetPath();

    desc.sdfMeshCookingParams.sdfResolution = 0;
    desc.triangleMeshCookingParams = cookingParams;

    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(desc.meshPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = carb::getCachedInterface<IPhysxCookingService>();
    cookingService->requestTriangleMeshCookedData(nullptr, request, desc.triangleMeshCookingParams);
    return meshCRCComputedSuccessfully;
}

bool fillSdfTriangleMeshDesc(const UsdGeomMesh& mesh, omni::physx::usdparser::TriangleMeshPhysxShapeDesc& desc, const omni::physx::SdfMeshCookingParams& cookingParams)
{
    UsdTimeCode time = UsdTimeCode::Default();
    desc.meshPath = mesh.GetPath();

    desc.sdfMeshCookingParams = cookingParams;
    omni::physx::PhysxCookingComputeRequest request;
    fillCookingRequest(request, mesh.GetPrim(), time);
    bool meshCRCComputedSuccessfully = false;
    request.onFinished = [&](const omni::physx::PhysxCookingComputeResult& result) {
        if (result.result != omni::physx::PhysxCookingResult::eVALID)
            return;
        meshCRCComputedSuccessfully = true;
        desc.crc = result.cookedDataCRC;
        desc.meshKey = result.meshKey;
        gMeshKeyCache.setMeshKey(desc.meshPath, desc.meshKey);
    };
    IPhysxCookingService* cookingService = carb::getCachedInterface<IPhysxCookingService>();
    cookingService->requestSdfMeshCookedData(nullptr, request, desc.triangleMeshCookingParams, desc.sdfMeshCookingParams);
    return meshCRCComputedSuccessfully;
}

void notifyStageReset(void)
{
    gMeshKeyCache.reset();
}

void invalidateMeshKeyCache(const SdfPath& path)
{
    gMeshKeyCache.clearMeshKey(path);
}

} // namespace usdparser
} // namespace physx
} // namespace omni
