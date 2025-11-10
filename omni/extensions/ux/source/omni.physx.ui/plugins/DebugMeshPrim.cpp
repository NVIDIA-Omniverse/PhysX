// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"
#include "DebugMeshPrim.h"
#include <omni/usd/UtilsIncludes.h>
#include <omni/usd/UsdUtils.h>
#include <carb/Defines.h>

#include <stdio.h>
#include <string.h>

#if CARB_COMPILER_MSC
#pragma warning(disable:4996)
#endif

extern pxr::UsdStageRefPtr gStage; // Declare external reference to the global stage handle


namespace debugmeshprim
{

static pxr::TfToken getCollisionMeshToken(void) // get the token we use to signify a prim is a debug collision mesh
{
    static pxr::TfToken gCollisionMeshToken("CollisionMesh");
    return gCollisionMeshToken;
}

/**
* Sets a key/value pair on the custom meta data field for this prim.
*
* @param prim : The primitive we wish to modify
* @param key : The key for this metadata expressed as a pxr::TfToken
* @param value : A const char string to assign as the 'data' portion for this metadata
*/
static void setMetaData(pxr::UsdPrim &prim, const pxr::TfToken &key, const char *value)
{
    prim.SetMetadataByDictKey<const char *>(pxr::SdfFieldKeys->CustomData, key, value);
}

static float fm_computePlane(const float* A, const float* B, const float* C, float* n) // returns D
{
    float vx = (B[0] - C[0]);
    float vy = (B[1] - C[1]);
    float vz = (B[2] - C[2]);

    float wx = (A[0] - B[0]);
    float wy = (A[1] - B[1]);
    float wz = (A[2] - B[2]);

    float vw_x = vy * wz - vz * wy;
    float vw_y = vz * wx - vx * wz;
    float vw_z = vx * wy - vy * wx;

    float mag = (float)sqrt((vw_x * vw_x) + (vw_y * vw_y) + (vw_z * vw_z));

    if (mag < 0.000001f)
    {
        mag = 0;
    }
    else
    {
        mag = 1.0f / mag;
    }

    float x = vw_x * mag;
    float y = vw_y * mag;
    float z = vw_z * mag;


    float D = 0.0f - ((x * A[0]) + (y * A[1]) + (z * A[2]));

    n[0] = x;
    n[1] = y;
    n[2] = z;

    return D;
}

static void getColor(pxr::GfVec3f &rgb,uint32_t color)
{
    uint32_t red = (color>>16)&0xFF;
    uint32_t green = (color>>8)&0xFF;
    uint32_t blue = color&0xFF;
    rgb[0] = float(red)/255.0f;
    rgb[1] = float(green)/255.0f;
    rgb[2] = float(blue)/255.0f;
}

    class DebugMeshPrimImpl : public DebugMeshPrim
    {
    public:
        DebugMeshPrimImpl(void)
        {
        }
        virtual ~DebugMeshPrimImpl(void)
        {
        }

        // Create a mesh primitive relative to the provided parent
        // prim.
        // Returns true if it was successful, false if it failed
        virtual bool createDebugMeshPrim(const char *primName, const DebugMesh &mesh,uint32_t meshColor) final
        {
            bool ret = false;

            if ( gStage )
            {
                pxr::SdfPath primPath(primName);
                gStage->RemovePrim(primPath); // remove any previous version of this prim
                pxr::UsdGeomMesh meshPrim = pxr::UsdGeomMesh::Define(gStage,primPath);
                pxr::UsdPrim prim = gStage->GetPrimAtPath(primPath);
                if ( prim )
                {
                    setMetaData(prim,getCollisionMeshToken(),"collision");
                    pxr::GfVec3f color;
                    getColor(color,meshColor);
                    pxr::VtArray<pxr::GfVec3f> colorArray(1,color);
                    meshPrim.CreateDisplayColorAttr().Set(colorArray);

                    pxr::VtArray<int> meshFaceVertexCounts;
                    meshFaceVertexCounts.resize(mesh.mTriangleCount);
                    for (uint32_t i=0; i<mesh.mTriangleCount; i++)
                    {
                        meshFaceVertexCounts[i] = 3;
                    }
                    pxr::VtArray<int> meshIndices;
                    meshIndices.resize(mesh.mTriangleCount*3);
                    for (uint32_t i=0; i<mesh.mTriangleCount*3; i++)
                    {
                        meshIndices[i] = mesh.mIndices[i];
                    }
                    pxr::VtArray<pxr::GfVec3f> meshPoints;
                    meshPoints.resize(mesh.mVertexCount);
                    pxr::GfVec3f bmin;
                    pxr::GfVec3f bmax;
                    for (uint32_t i=0; i<mesh.mVertexCount; i++)
                    {
                        const float *p = &mesh.mVertices[i*3];
                        pxr::GfVec3f point;
                        point[0] = p[0];
                        point[1] = p[1];
                        point[2] = p[2];
                        meshPoints[i] = point;
                        if (i == 0)
                        {
                            bmin = point;
                            bmax = point;
                        }
                        else
                        {
                            if (point[0] < bmin[0]) bmin[0] = point[0];
                            if (point[1] < bmin[1]) bmin[1] = point[1];
                            if (point[2] < bmin[2]) bmin[2] = point[2];

                            if (point[0] > bmax[0]) bmax[0] = point[0];
                            if (point[1] > bmax[1]) bmax[1] = point[1];
                            if (point[2] > bmax[2]) bmax[2] = point[2];
                        }
                    }

                    pxr::VtArray<pxr::GfVec3f> extent;
                    extent.push_back(bmin);
                    extent.push_back(bmax);

                    pxr::VtArray<pxr::GfVec3f> meshNormals;
                    meshNormals.resize(mesh.mTriangleCount);
                    for (uint32_t i = 0; i < mesh.mTriangleCount; i++)
                    {
                        uint32_t i1 = mesh.mIndices[i * 3 + 0];
                        uint32_t i2 = mesh.mIndices[i * 3 + 1];
                        uint32_t i3 = mesh.mIndices[i * 3 + 2];

                        const float *p1 = &mesh.mVertices[i1 * 3];
                        const float *p2 = &mesh.mVertices[i2 * 3];
                        const float *p3 = &mesh.mVertices[i3 * 3];
                        pxr::GfVec3f normal;
                        fm_computePlane(p1, p2, p3, &normal[0]);

                        meshNormals[i] = normal;
                    }

                    meshPrim.CreatePointsAttr().Set(meshPoints);

                    meshPrim.CreateNormalsAttr().Set(meshNormals);
                    meshPrim.SetNormalsInterpolation(pxr::UsdGeomTokens->uniform);

                    meshPrim.CreateFaceVertexCountsAttr().Set(meshFaceVertexCounts);
                    meshPrim.CreateFaceVertexIndicesAttr().Set(meshIndices);
                    meshPrim.CreateExtentAttr().Set(extent);

                    meshPrim.CreateSubdivisionSchemeAttr(pxr::VtValue(pxr::UsdGeomTokens->none));

                    ret = true;
                }
            }

            return ret;
        }

        virtual void release(void) final
        {
            delete this;
        }


    };

DebugMeshPrim *DebugMeshPrim::create(void)
{
    auto ret = new DebugMeshPrimImpl;
    return static_cast< DebugMeshPrim *>(ret);
}



} // end of debugmeshprim namespace
