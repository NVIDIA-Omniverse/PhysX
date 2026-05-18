// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include "UsdPCH.h"

#include <common/foundation/Allocator.h>

#include "MeshThickness.h"

#include "foundation/PxPlane.h"
#include "foundation/PxBounds3.h"

namespace meshthickness
{

static void addTri(uint32_t *&indices,uint32_t i1,uint32_t i2,uint32_t i3)
{
    indices[0] = i1;
    indices[1] = i2;
    indices[2] = i3;
    indices+=3;
}

// Check to see if the input mesh is within the provided minimum thickness value.
// Returns false if it was not and no output mesh is generated
// If it returns true, then the outputMesh contains the new extruded triangle mesh
bool checkMeshThickness(const Mesh &inputMesh, Mesh &outputMesh, float minThickness)
{
    bool ret = false;

    if ( minThickness < 0.00001f )
    {
        minThickness = 0.00001f;
    }
    float halfThickness = minThickness*0.5f;

    if ( inputMesh.triangleCount )
    {
        physx::PxBounds3 b = physx::PxBounds3::empty();
        physx::PxPlane plane;
        bool isTiny = true;
        bool isPlanar = true;
        // Iterate through all triangles so long as they are either coplanar or tiny
        for (uint32_t i=0; i<inputMesh.triangleCount && (isTiny || isPlanar); i++)
        {
            // Get the triangle indices
            uint32_t i1 = inputMesh.indices[i*3+0];
            uint32_t i2 = inputMesh.indices[i*3+1];
            uint32_t i3 = inputMesh.indices[i*3+2];
            // Get the three vertices
            const ::physx::PxVec3& p1 = *(const ::physx::PxVec3 *)&inputMesh.vertices[i1*3];
            const ::physx::PxVec3& p2 = *(const ::physx::PxVec3 *)&inputMesh.vertices[i2*3];
            const ::physx::PxVec3& p3 = *(const ::physx::PxVec3 *)&inputMesh.vertices[i3*3];
            // If it's still tiny, include these points into the bounding box and see if
            // the new bounding box is still tiny
            if ( isTiny )
            {
                b.include(p1);
                b.include(p2);
                b.include(p3);
                ::physx::PxVec3 extents = b.getDimensions();
                if ( extents.x > halfThickness || extents.y > halfThickness || extents.z > halfThickness )
                {
                    isTiny = false;
                }
            }
            // If this is the first triangle, compute the plane equation of the first triangle
            // If it's not the first triangle then compute the plane equation and see if it has nearly the same normal vector and the
            // D equation co-efficient is within the minimum thickness. If it is, then it's considered still coplanar
            if ( i == 0 )
            {
                plane = physx::PxPlane(p1,p2,p3);
            }
            else if ( isPlanar ) 
            {
                physx::PxPlane tplane(p1,p2,p3);
                float dot = plane.n.dot(tplane.n);
                if ( dot >= 0.98f )
                {
                    float diff = plane.d - tplane.d;
                    if ( diff >= -halfThickness && diff <= halfThickness )
                    {
                    }
                    else
                    {
                        isPlanar = false;
                    }
                }
                else
                {
                    isPlanar = false;
                }
            }
        }
        if ( isTiny ) // we need to create a bounding box which reflects a hull that at least meets the minimum thickness
        {
            ::physx::PxVec3 box[8];

            physx::PxBounds3 bounds;
            ::physx::PxVec3 thickness(halfThickness,halfThickness,halfThickness);

            bounds.minimum = b.getCenter() - thickness;
            bounds.maximum = b.getCenter() + thickness;

            box[0] = ::physx::PxVec3(bounds.minimum.x, bounds.minimum.y, bounds.minimum.z);
            box[1] = ::physx::PxVec3(bounds.maximum.x, bounds.minimum.y, bounds.minimum.z);
            box[2] = ::physx::PxVec3(bounds.maximum.x, bounds.maximum.y, bounds.minimum.z);
            box[3] = ::physx::PxVec3(bounds.minimum.x, bounds.maximum.y, bounds.minimum.z);
            box[4] = ::physx::PxVec3(bounds.minimum.x, bounds.minimum.y, bounds.maximum.z);
            box[5] = ::physx::PxVec3(bounds.maximum.x, bounds.minimum.y, bounds.maximum.z);
            box[6] = ::physx::PxVec3(bounds.maximum.x, bounds.maximum.y, bounds.maximum.z);
            box[7] = ::physx::PxVec3(bounds.minimum.x, bounds.maximum.y, bounds.maximum.z);

            outputMesh.vertexCount = 8;
            outputMesh.triangleCount = 12;

            float* newVertices = (float *)GetAllocator()->malloc(sizeof(float)*outputMesh.vertexCount * 3);
            uint32_t* newIndices = (uint32_t *)GetAllocator()->malloc(sizeof(uint32_t)*outputMesh.triangleCount * 3);

            memcpy(newVertices, box, sizeof(::physx::PxVec3) * outputMesh.vertexCount);

            outputMesh.vertices = newVertices;
            outputMesh.indices = newIndices;
            uint32_t *indices = newIndices;

            addTri(indices,2,1,0);
            addTri(indices,3,2,0);

            addTri(indices,7,2,3);
            addTri(indices,7,6,2);

            addTri(indices,5,1,2);
            addTri(indices,5,2,6);

            addTri(indices,5,4,1);
            addTri(indices,4,0,1);

            addTri(indices,4,6,7);
            addTri(indices,4,5,6);

            addTri(indices,4,7,0);
            addTri(indices,7,3,0);

            ret = true;
        }
        else if ( isPlanar )
        {
            // extrude the triangles so they are at least the minimum distance apart
            outputMesh.vertexCount   = inputMesh.vertexCount*2;
            outputMesh.triangleCount = inputMesh.triangleCount*2;
            float* newVertices = (float *)GetAllocator()->malloc(sizeof(float)*outputMesh.vertexCount*3);
            uint32_t* newIndices = (uint32_t *)GetAllocator()->malloc(sizeof(uint32_t)*outputMesh.triangleCount*3);
            outputMesh.vertices = newVertices;
            outputMesh.indices = newIndices;
            ::physx::PxVec3 thickness = plane.n*halfThickness;
            for (uint32_t i=0; i<inputMesh.vertexCount; i++)
            {
                const ::physx::PxVec3 &p = *(const ::physx::PxVec3 *)&inputMesh.vertices[i*3];
                ::physx::PxVec3 &p1 = (::physx::PxVec3 &)newVertices[i*3];
                ::physx::PxVec3 &p2 = (::physx::PxVec3 &)newVertices[(i+inputMesh.vertexCount)*3];
                p1 = p-thickness;
                p2 = p+thickness;
            }
            for (uint32_t i=0; i<inputMesh.triangleCount; i++)
            {
                uint32_t i1 = inputMesh.indices[i * 3 + 0];
                uint32_t i2 = inputMesh.indices[i * 3 + 1];
                uint32_t i3 = inputMesh.indices[i * 3 + 2];

                newIndices[i*3+0] = i1;
                newIndices[i*3+1] = i2;
                newIndices[i*3+2] = i3;

                uint32_t j = i+inputMesh.triangleCount;

                newIndices[j * 3 + 0] = i1+inputMesh.vertexCount;
                newIndices[j * 3 + 1] = i2+inputMesh.vertexCount;
                newIndices[j * 3 + 2] = i3+inputMesh.vertexCount;
            }
            ret = true;
        }
    }

    return ret;
}

// Release any memory associated with the output mesh previously generated
void releaseMeshOutput(Mesh &outputMesh)
{
    if ( outputMesh.vertices )
    {
        GetAllocator()->free((void*)outputMesh.vertices,false);
    }
    if ( outputMesh.indices )
    {
        GetAllocator()->free((void*)outputMesh.indices,false);
    }
    outputMesh.vertexCount = 0;
    outputMesh.triangleCount = 0;
    outputMesh.vertices = nullptr;
    outputMesh.indices = nullptr;
}

}
