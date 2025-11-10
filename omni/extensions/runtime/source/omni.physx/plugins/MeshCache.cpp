// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "MeshCache.h"

#include <common/utilities/MemoryMacros.h>

#include "particles/PhysXParticleSampling.h"

#include <private/omni/physx/IPhysxCookingServicePrivate.h>
#include <omni/physx/IPhysxVisualization.h>
#include <carb/profiler/Profile.h>


using namespace ::physx;
using namespace omni::physx;

MeshCache* gMeshCache = nullptr;

MeshCache* omni::physx::getMeshCache()
{
    if (!gMeshCache)
    {
        gMeshCache = new MeshCache();
    }

    return gMeshCache;
}

void omni::physx::releaseMeshCache()
{
    SAFE_RELEASE(gMeshCache);
}

const uint32_t* MeshCache::getRemapTable(const pxr::UsdPrim& usdPrim)
{
    if (usdPrim.IsA<pxr::UsdGeomMesh>())
    {
        const pxr::UsdGeomMesh& usdMesh = (const pxr::UsdGeomMesh&)(usdPrim);
        std::vector<pxr::UsdGeomSubset> subsets = pxr::UsdGeomSubset::GetGeomSubsets(usdMesh, pxr::UsdGeomTokens->face);
        if (subsets.empty())
            return nullptr;
        
        pxr::UsdTimeCode time = pxr::UsdTimeCode::Default();
        pxr::VtArray<int> facesValue;
        // test if the verts are there or if its time sampled
        {
            usdMesh.GetFaceVertexCountsAttr().Get(&facesValue);
            if (!facesValue.size())
            {
                time = pxr::UsdTimeCode::EarliestTime();
                usdMesh.GetFaceVertexCountsAttr().Get(&facesValue, time);
            }
        }
        uint32_t faceCount = (uint32_t)facesValue.size();
        if (faceCount)
        {
            // first compute how many triangles will be needed..
            uint32_t triangleCount = 0;
            for (uint32_t i = 0; i < faceCount; i++)
            {
                uint32_t count = facesValue[i];
                triangleCount += (count - 2);
            }

            uint32_t* triangleFaceMapping = new uint32_t[triangleCount];
            uint32_t* trMapping = triangleFaceMapping;
            for (uint32_t i = 0; i < faceCount; i++)
            {
                const uint32_t faceCount = facesValue[i];
                for (uint32_t faceIndex = 0; faceIndex < (faceCount - 2); faceIndex++)
                {
                    trMapping[0] = i;
                    trMapping++;
                }                
            }
            return triangleFaceMapping;
        }
    }
    return nullptr;
}

void MeshCache::release()
{
    {
        for (auto &i:mConvexMeshMap)
        {
            i.second->release();
        }
        mConvexMeshMap.clear();
    }

    {
        for (auto &i:mTriangleMeshMap)
        {
            i.second->release();
        }
        mTriangleMeshMap.clear();
    }

    {
        for (auto &i:mConvexDecompositionMap)
        {
            for (auto &j:i.second)
            {
                if (j)
                    j->release();
            }
        }
        mConvexDecompositionMap.clear();
    }

    {
        for (auto &i:mConvexMeshDataMap)
        {
            delete[] i.second.polygons;
        }
        mConvexMeshDataMap.clear();
    }

    {
        for (auto &i:mTriMeshRemapTable)
        {
            if (i.second)
                delete [] i.second;
        }
        mTriMeshRemapTable.clear();
    }

    {
        for (auto &i:mSphereFillMap)
        {
            delete i.second;
        }
        mSphereFillMap.clear();
    }
}

PX_COMPILE_TIME_ASSERT(sizeof(PxHullPolygon) == sizeof(ConvexMeshPolygon));

ConvexMeshPolygon* MeshCache::getConvexMeshPolygons(::physx::PxConvexMesh* mesh)
{
    const PxU32 numPoly = mesh->getNbPolygons();
    ConvexMeshPolygon* out = new ConvexMeshPolygon[numPoly];
    for (PxU32 i = 0; i < numPoly; i++)
    {
        PxHullPolygon polygon;
        mesh->getPolygonData(i, polygon);
        memcpy(&out[i].plane[0], &polygon.mPlane[0], sizeof(PxHullPolygon));
    }

    return out;
}

void MeshCache::addConvexMesh(const omni::physx::usdparser::MeshKey &crc, ::physx::PxConvexMesh* mesh)
{
    mesh->acquireReference();
    mConvexMeshMap[crc] = mesh;

    // mesh data
    ConvexMeshPolygon* polygons = getConvexMeshPolygons(mesh);
    mConvexMeshDataMap.insert(
        std::make_pair(mesh, ConvexMeshData(mesh->getNbVertices(), (const carb::Float3*)mesh->getVertices(),
                                               mesh->getIndexBuffer(), mesh->getNbPolygons(), polygons)));
}

void MeshCache::addConvexDecomposition(const omni::physx::usdparser::MeshKey &crc, const ConvexMeshVector& meshes)
{
    mConvexDecompositionMap[crc] = meshes;
    for (size_t i = 0; i < meshes.size(); i++)
    {
        if ( meshes[i] )
        {
            meshes[i]->acquireReference();
            // mesh data
            ConvexMeshPolygon* polygons = getConvexMeshPolygons(meshes[i]);
            mConvexMeshDataMap.insert(std::make_pair(
                meshes[i], ConvexMeshData(meshes[i]->getNbVertices(), (const carb::Float3*)meshes[i]->getVertices(),
                                        meshes[i]->getIndexBuffer(), meshes[i]->getNbPolygons(), polygons)));
        }
    }
}


bool loadTriangleFaceMap(omni::physx::PhysxCookedDataSpan inputTriangulationData, const uint32_t triangleMeshDataVersion, std::vector<uint8_t>& triangleFaceMap)
{
    PxDefaultMemoryInputData idata((PxU8*)inputTriangulationData.data, (PxU32)inputTriangulationData.sizeInBytes);
    uint32_t version = 0;
    uint32_t vertexCount = 0;
    uint32_t triangleCount = 0;
    uint32_t faceCount = 0;
    idata.read(&version, sizeof(version));
    if (version == triangleMeshDataVersion)
    {
        idata.read(&vertexCount, sizeof(vertexCount));
        if (vertexCount)
        {
            uint32_t vsize = sizeof(float) * 3 * vertexCount;
            uint32_t newPos = idata.tell()+vsize;
            idata.seek(newPos);
            if (newPos < idata.getLength())
            {
                idata.read(&triangleCount, sizeof(triangleCount));
                if (triangleCount)
                {
                    vsize = sizeof(uint32_t) * 3 * triangleCount;
                    newPos = idata.tell()+vsize;
                    idata.seek(newPos);
                    if (newPos < idata.getLength())
                    {
                        uint32_t triangleCount;
                        idata.read(&triangleCount, sizeof(triangleCount));
                        if (triangleCount)
                        {
                            vsize = sizeof(uint32_t) * triangleCount;
                            triangleFaceMap.resize(vsize);
                            uint32_t rsize = idata.read(triangleFaceMap.data(), vsize);
                            if (rsize == vsize)
                            {
                               return true;
                            }
                        }
                    }
                }
            }
        }
    }
    return false;
}


bool MeshCache::createRuntimeTriangleMesh(::physx::PxPhysics& physics,
                                          usdparser::MeshKey meshCRC,
                                          bool useOriginalTriangles,
                                          omni::physx::PhysxCookedDataSpan inputCookedData,
                                          gsl::span<const uint32_t> inputTrianglesFaceMapping,
                                          ::physx::PxTriangleMesh** returnedMesh)
{
    CARB_PROFILE_ZONE(0, "MeshCache::createRuntimeTriangleMesh");
    using namespace ::physx;
    if (hasTriangleMesh(meshCRC))
    {
        // If this triangle mesh has already been registered with the in-memory cache, then we can delete it because
        // it got created twice. This is highly unlikely to ever happen but in a multi-threaded situation it is
        // hypothetically possible
        if(returnedMesh)
        {
            *returnedMesh = mTriangleMeshMap.at(meshCRC);
        }
        return true;
    }
    PxTriangleMesh* triangleMesh = nullptr;

    PxDefaultMemoryInputData inData((PxU8*)inputCookedData.data, (PxU32)inputCookedData.sizeInBytes);
    // Create the triangle mesh instance from the cooked data
    // Add the triangle mesh to the in memory mesh cache
    if (useOriginalTriangles)
    {
        if(inputTrianglesFaceMapping.size_bytes() == 0)
            return false; // We can't proceed as we don't have the original USD Prim for the remap table
        triangleMesh = physics.createTriangleMesh(inData);
        if (triangleMesh)
        {
            if(returnedMesh)
            {
                *returnedMesh = triangleMesh;
            }
            addTriangleMesh(meshCRC, triangleMesh, nullptr, &inputTrianglesFaceMapping, true);
            return true;
        }
    }
    else
    {
        triangleMesh = physics.createTriangleMesh(inData);
        if (triangleMesh)
        {
            if(returnedMesh)
            {
                *returnedMesh = triangleMesh;
            }
            addTriangleMesh(meshCRC, triangleMesh, nullptr, nullptr, false);
        }
        return true;
    }
    return false;
}

bool MeshCache::createRuntimeSphereFill(::physx::PxPhysics& physics,
                                usdparser::MeshKey crc,
                                omni::physx::PhysxCookedDataSpan inputCookedData,
                                const omni::physx::usdparser::SpherePointsPhysxShapeDesc ** returnedMesh)
{
    CARB_PROFILE_ZONE(0, "MeshCache::createRuntimeSphereFill");
    using namespace ::physx;

    if(mSphereFillMap.find(crc) == mSphereFillMap.end())
    {
        omni::physx::usdparser::SpherePointsPhysxShapeDesc sphereFill;
        uint32_t sphereCount = 0;
        // Sanity check for corrupted data
        if(inputCookedData.sizeInBytes < sizeof(uint32_t))
            return false;
        // This is not really safe if you're not sure about the alignment of the raw bytes
        const uint32_t* data = (const uint32_t*)inputCookedData.data;
        sphereCount = *data++;
        const omni::physx::usdparser::SpherePhysxPoint* spp = (const omni::physx::usdparser::SpherePhysxPoint*)data;
        // Sanity check for corrupted data
        if(inputCookedData.sizeInBytes != (sizeof(uint32_t) + sphereCount * sizeof(omni::physx::usdparser::SpherePhysxPoint)))
            return false;
        for (uint32_t i = 0; i < sphereCount; i++)
        {
            sphereFill.spheres.push_back(*spp);
            spp++;
        }
        addSphereFill(crc, sphereFill);
    }
    if(returnedMesh)
    {
        *returnedMesh = mSphereFillMap.at(crc);
    }
    return true;
}

bool MeshCache::createRuntimeConvexMesh(::physx::PxPhysics& physics,
                                        usdparser::MeshKey crc,
                                        omni::physx::PhysxCookedDataSpan inputCookedData,
                                        ::physx::PxConvexMesh** returnedMesh)
{
    CARB_PROFILE_ZONE(0, "MeshCache::createRuntimeConvexMesh");
    using namespace ::physx;
    if(hasConvexMesh(crc))
    {
        if(returnedMesh)
        {
            *returnedMesh = mConvexMeshMap.at(crc);
        }
        return true;
    }
    PxDefaultMemoryInputData inData((PxU8*)inputCookedData.data, (PxU32)inputCookedData.sizeInBytes);
    // Here we actually create in instance of a convex hull based on this cooked data
    PxConvexMesh* convexMesh = physics.createConvexMesh(inData);
    if (convexMesh)
    {
        addConvexMesh(crc, convexMesh);
        if (returnedMesh)
        {
            *returnedMesh = convexMesh;
        }
        return true;
    }
    return false;
}

bool MeshCache::createRuntimeConvexDecomposition(::physx::PxPhysics& physics,
                                                 usdparser::MeshKey crc,
                                                 const omni::physx::PhysxCookedDataSpan* inputCookedData,
                                                 uint64_t inputCookedDataNumber,
                                                 std::vector<::physx::PxConvexMesh*>* returnedMeshes)
{
    CARB_PROFILE_ZONE(0, "MeshCache::createRuntimeConvexDecomposition");
    using namespace ::physx;
    if (hasConvexDecomposition(crc))
    {
        // In the rare and unlikely event that a convex decomposition with this CRC has already been
        // added to the local in-memory cache, we don't need to add it a second time.
        if(returnedMeshes)
        {
            *returnedMeshes =  mConvexDecompositionMap.at(crc);
        }
        return true;
    }
    std::vector<PxConvexMesh*> outputMeshes;
    for(uint64_t idx = 0; idx < inputCookedDataNumber; ++idx)
    {
        const omni::physx::PhysxCookedDataSpan span = inputCookedData[idx];
        PxDefaultMemoryInputData inData((PxU8*)span.data, (PxU32)span.sizeInBytes);
        // We should be probably try to handle partial failure
        outputMeshes.push_back(physics.createConvexMesh(inData));
    }
    if (outputMeshes.empty())
    {
        return false;
    }
    addConvexDecomposition(crc, outputMeshes);
    if(returnedMeshes)
    {
        *returnedMeshes = outputMeshes;
    }
    return true;
}

static void fillCollisionMeshFromConvexMesh(PxConvexMesh* pxMesh, omni::physx::CollisionMesh& mesh, carb::Float3 signScale)
{
    uint32_t nbVertices = pxMesh->getNbVertices();
    const PxVec3* vertices = pxMesh->getVertices();
    const PxU8* indexBuffer = pxMesh->getIndexBuffer();
    const uint32_t nbPolygons = pxMesh->getNbPolygons();
    std::vector<uint32_t> indices;
    // Here we iterate the polygons of the convex hull and triangulate
    // the results
    for (uint32_t i = 0; i < nbPolygons; i++)
    {
        PxHullPolygon polygon;
        pxMesh->getPolygonData(i, polygon);
        uint32_t baseIndex = polygon.mIndexBase;
        uint32_t i1 = indexBuffer[baseIndex + 0];
        uint32_t i2 = indexBuffer[baseIndex + 1];
        uint32_t i3 = indexBuffer[baseIndex + 2];
        indices.push_back(i1);
        indices.push_back(i2);
        indices.push_back(i3);
        for (uint32_t i = 3; i < polygon.mNbVerts; i++)
        {
            i2 = i3;
            i3 = indexBuffer[baseIndex + i];
            indices.push_back(i1);
            indices.push_back(i2);
            indices.push_back(i3);
        }
    }
    uint32_t nbTriangles = (uint32_t)indices.size() / 3;

    mesh.vertexCount = nbVertices;
    mesh.vertices = new float[mesh.vertexCount * 3];
    memcpy(mesh.vertices, vertices, sizeof(float) * mesh.vertexCount * 3);
    PxVec3 pxSignScale(signScale.x,signScale.y,signScale.z);
    if (pxSignScale != PxVec3(1,1,1))
    {
        for(uint32_t idx = 0; idx < mesh.vertexCount; ++idx)
        {
            mesh.vertices[idx*3 + 0] *= signScale.x;
            mesh.vertices[idx*3 + 1] *= signScale.y;
            mesh.vertices[idx*3 + 2] *= signScale.z;
        }
    }
    mesh.triangleCount = nbTriangles;
    mesh.indices = new uint32_t[indices.size()];
    memcpy(mesh.indices, &indices[0], sizeof(uint32_t) * indices.size());
    PxBounds3 bounds = pxMesh->getLocalBounds();
    PxVec3 meshCenter = bounds.getCenter();
    memcpy(&mesh.meshCenter.x, &meshCenter.x, sizeof(meshCenter));
}

static void fillCollisionMeshFromTriangleMesh(PxTriangleMesh* pxMesh, omni::physx::CollisionMesh& mesh)
{
    uint32_t nbVertices = pxMesh->getNbVertices();
    const PxVec3* vertices = pxMesh->getVertices();
    const uint32_t nbTriangles = pxMesh->getNbTriangles();

    mesh.vertexCount = nbVertices;
    mesh.vertices = new float[mesh.vertexCount * 3];
    memcpy(mesh.vertices, vertices, sizeof(float) * mesh.vertexCount * 3);
    mesh.triangleCount = nbTriangles;
    mesh.indices = new uint32_t[nbTriangles * 3];
    const bool uint16_indices = pxMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::e16_BIT_INDICES;

    if (uint16_indices)
    {
        const uint16_t* sourceIndices = (const uint16_t*)pxMesh->getTriangles();
        const auto numIndices = nbTriangles * 3;
        for (size_t idx = 0; idx < numIndices; ++idx)
        {
            mesh.indices[idx] = sourceIndices[idx];
        }
    }
    else
    {
        memcpy(mesh.indices, pxMesh->getTriangles(), sizeof(uint32_t) * nbTriangles * 3);
    }
    PxBounds3 bounds = pxMesh->getLocalBounds();
    PxVec3 meshCenter = bounds.getCenter();
    memcpy(&mesh.meshCenter.x, &meshCenter.x, sizeof(meshCenter));
}

omni::physx::CollisionRepresentation* MeshCache::getCollisionRepresentation(usdparser::MeshKey crc,
                                                                            const omni::physx::usdparser::PhysxShapeDesc& desc)
{
    switch (desc.type)
    {
    case omni::physx::usdparser::eConvexMeshShape: {
        const omni::physx::usdparser::ConvexMeshPhysxShapeDesc* convexMeshDesc = (omni::physx::usdparser::ConvexMeshPhysxShapeDesc*)&desc;
        PxConvexMesh* pxMesh = getConvexMesh(crc);
        if (!pxMesh)
            return nullptr;

        carb::Float3 rootCenter = { 0, 0, 0 };
        omni::physx::CollisionRepresentation* cr = new omni::physx::CollisionRepresentation;
        cr->meshCount = 1;
        cr->meshes = new omni::physx::CollisionMesh[cr->meshCount];
        memcpy(&cr->rootCenter, &rootCenter.x, sizeof(cr->rootCenter));
        PxBounds3 bounds = pxMesh->getLocalBounds();
        memcpy(&cr->bmin.x, &bounds.minimum, sizeof(cr->bmin));
        memcpy(&cr->bmax.x, &bounds.maximum, sizeof(cr->bmax));

        fillCollisionMeshFromConvexMesh(pxMesh, cr->meshes[0], convexMeshDesc->convexCookingParams.signScale);

        return cr;
    }
    break;
    case omni::physx::usdparser::eConvexMeshDecompositionShape: {
        omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc* convexDecompositionMeshDesc = (omni::physx::usdparser::ConvexMeshDecompositionPhysxShapeDesc*)&desc;
        auto it = getConvexDecompositionMap().find(crc);
        if (it == getConvexDecompositionMap().end())
            return nullptr;
        const ConvexMeshVector& convexMeshes = it->second;

        carb::Float3 rootCenter = { 0, 0, 0 };
        omni::physx::CollisionRepresentation* cr = new omni::physx::CollisionRepresentation;
        cr->meshCount = (uint32_t)convexMeshes.size();
        cr->meshes = new omni::physx::CollisionMesh[cr->meshCount];
        memcpy(&cr->rootCenter, &rootCenter.x, sizeof(cr->rootCenter));

        PxBounds3 totalBounds = PxBounds3::empty();
        for (size_t i = 0; i < convexMeshes.size(); ++i)
        {
            PxConvexMesh* convexMesh = convexMeshes[i];
            PxBounds3 bounds = convexMesh->getLocalBounds();
            totalBounds.include(bounds);
            fillCollisionMeshFromConvexMesh(convexMesh, cr->meshes[i], convexDecompositionMeshDesc->convexDecompositionCookingParams.signScale);
        }
        memcpy(&cr->bmin.x, &totalBounds.minimum, sizeof(cr->bmin));
        memcpy(&cr->bmax.x, &totalBounds.maximum, sizeof(cr->bmax));
        std::sort(cr->meshes, cr->meshes + cr->meshCount, [](const omni::physx::CollisionMesh& p1, const omni::physx::CollisionMesh& p2)
        {
            float distances[3];
            distances[0] = p1.meshCenter.x - p2.meshCenter.x;
            distances[1] = p1.meshCenter.y - p2.meshCenter.y;
            distances[2] = p1.meshCenter.z - p2.meshCenter.z;

            for(int i = 0; i < 3; ++i)
            {
                if(fabsf(distances[i]) < 1e-5f)
                {
                    continue;
                }            
                return distances[i] < 0.0f;
            }
            return false;
        });
        return cr;
    }
    break;
    case omni::physx::usdparser::eTriangleMeshShape: {
        PxTriangleMesh* pxMesh = getTriangleMesh(crc);
        if (!pxMesh)
            return nullptr;

        carb::Float3 rootCenter = { 0, 0, 0 };
        omni::physx::CollisionRepresentation* cr = new omni::physx::CollisionRepresentation;
        cr->meshCount = 1;
        cr->meshes = new omni::physx::CollisionMesh[cr->meshCount];
        memcpy(&cr->rootCenter, &rootCenter.x, sizeof(cr->rootCenter));
        PxBounds3 bounds = pxMesh->getLocalBounds();
        memcpy(&cr->bmin.x, &bounds.minimum, sizeof(cr->bmin));
        memcpy(&cr->bmax.x, &bounds.maximum, sizeof(cr->bmax));

        fillCollisionMeshFromTriangleMesh(pxMesh, cr->meshes[0]);
        return cr;
    }
    break;
    default:
        break;
    }
    return nullptr;
}
