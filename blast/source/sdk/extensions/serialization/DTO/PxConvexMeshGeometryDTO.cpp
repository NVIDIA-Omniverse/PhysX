// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2022-2024 NVIDIA Corporation. All rights reserved.


#include "PxConvexMeshGeometryDTO.h"
#include "PxMeshScaleDTO.h"
#include "NvBlastAssert.h"
#include "NvBlastExtKJPxInputStream.h"
#include "NvBlastExtKJPxOutputStream.h"
#include "PxConvexMeshDesc.h"
#include "NvBlastExtSerialization.h"
#include "PxVec3.h"
#include <algorithm>
#include <vector>
#include "PxPhysics.h"
#include "NvBlastPxCallbacks.h"
#include "PxDefaultStreams.h"


namespace Nv
{
namespace Blast
{

extern physx::PxPhysics* sExtPxSerializerPhysics;
extern physx::PxCooking* sExtPxSerializerCooking;


bool PxConvexMeshGeometryDTO::serialize(Nv::Blast::Serialization::PxConvexMeshGeometry::Builder builder, const physx::PxConvexMeshGeometry * poco)
{
    NVBLAST_ASSERT(sExtPxSerializerCooking != nullptr);

    PxMeshScaleDTO::serialize(builder.getScale(), &poco->scale);

    //TODO: Use cooking.cookConvexMesh to cook the mesh to a stream - then get that backing buffer and put it into the Data field

    physx::PxConvexMeshDesc desc;
    desc.points.data = poco->convexMesh->getVertices();
    desc.points.count = poco->convexMesh->getNbVertices();
    desc.points.stride = sizeof(physx::PxVec3);

    std::vector<uint32_t> indicesScratch;
    std::vector<physx::PxHullPolygon> hullPolygonsScratch;

    hullPolygonsScratch.resize(poco->convexMesh->getNbPolygons());

    uint32_t indexCount = 0;
    for (uint32_t i = 0; i < hullPolygonsScratch.size(); i++)
    {
        physx::PxHullPolygon polygon;
        poco->convexMesh->getPolygonData(i, polygon);
        if (polygon.mNbVerts)
        {
            indexCount = std::max<uint32_t>(indexCount, polygon.mIndexBase + polygon.mNbVerts);
        }
    }
    indicesScratch.resize(indexCount);

    for (uint32_t i = 0; i < hullPolygonsScratch.size(); i++)
    {
        physx::PxHullPolygon polygon;
        poco->convexMesh->getPolygonData(i, polygon);
        for (uint32_t j = 0; j < polygon.mNbVerts; j++)
        {
            indicesScratch[polygon.mIndexBase + j] = poco->convexMesh->getIndexBuffer()[polygon.mIndexBase + j];
        }

        hullPolygonsScratch[i] = polygon;
    }

    desc.indices.count = indexCount;
    desc.indices.data = indicesScratch.data();
    desc.indices.stride = sizeof(uint32_t);

    desc.polygons.count = poco->convexMesh->getNbPolygons();
    desc.polygons.data = hullPolygonsScratch.data();
    desc.polygons.stride = sizeof(physx::PxHullPolygon);
        
    physx::PxDefaultMemoryOutputStream outStream(NvBlastGetPxAllocatorCallback());
    if (!sExtPxSerializerCooking->cookConvexMesh(desc, outStream))
    {
        return false;
    }

    kj::ArrayPtr<unsigned char> cookedBuffer(outStream.getData(), outStream.getSize());

    builder.setConvexMesh(cookedBuffer);

    return true;
}


physx::PxConvexMeshGeometry* PxConvexMeshGeometryDTO::deserialize(Nv::Blast::Serialization::PxConvexMeshGeometry::Reader reader)
{
    NVBLAST_ASSERT(sExtPxSerializerCooking != nullptr);

    NV_UNUSED(reader);

    return nullptr;
}


bool PxConvexMeshGeometryDTO::deserializeInto(Nv::Blast::Serialization::PxConvexMeshGeometry::Reader reader, physx::PxConvexMeshGeometry * poco)
{
    NVBLAST_ASSERT(sExtPxSerializerPhysics != nullptr);

    PxMeshScaleDTO::deserializeInto(reader.getScale(), &poco->scale);

    Nv::Blast::ExtKJPxInputStream inputStream(reader.getConvexMesh());

    //NOTE: Naive approach, no shared convex hulls
    poco->convexMesh = sExtPxSerializerPhysics->createConvexMesh(inputStream);

    return poco->convexMesh != nullptr;
}

}   // namespace Blast
}   // namespace Nv
