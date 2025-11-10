// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "TriangulateUsdMeshPrim.h"
//#include <usdLoad/Material.h>
//#include <common/utilities/UsdMaterialParsing.h>

namespace triangulateusd
{

class TriangulateUSDPrimImpl : public TriangulateUSDPrim
{
public:
    TriangulateUSDPrimImpl(const pxr::UsdPrim &usdPrim)
    {
        if (usdPrim.IsA<pxr::UsdGeomMesh>())
        {
            const pxr::UsdGeomMesh& usdMesh = (const pxr::UsdGeomMesh&)(usdPrim);
            pxr::UsdTimeCode time = pxr::UsdTimeCode::Default();
            pxr::VtArray<pxr::GfVec3f> pointsValue;
            // test if the verts are there or if its time sampled
            {
                usdMesh.GetPointsAttr().Get(&pointsValue);
                if (!pointsValue.size())
                {
                    time = pxr::UsdTimeCode::EarliestTime();
                    usdMesh.GetPointsAttr().Get(&pointsValue, time);
                }
            }
            pxr::VtArray<int> indicesValue;
            pxr::VtArray<int> facesValue;
            usdMesh.GetFaceVertexIndicesAttr().Get(&indicesValue, time);
            usdMesh.GetFaceVertexCountsAttr().Get(&facesValue, time);
            if ( pointsValue.size() && indicesValue.size() && facesValue.size() )
            {
                m_isValid = true;

                m_vertexCount = uint32_t(pointsValue.size());
                m_vertices = new float[m_vertexCount*3];
                memcpy(m_vertices,&pointsValue[0],sizeof(float)*3*m_vertexCount);

                m_faceIndicesCount = uint32_t(indicesValue.size());
                m_faceIndices = new uint32_t[m_faceIndicesCount];
                memcpy(m_faceIndices,&indicesValue[0],sizeof(uint32_t)*m_faceIndicesCount);

                m_facesCount = uint32_t(facesValue.size());
                m_faceCounts = new uint32_t[m_facesCount];
                memcpy(m_faceCounts,&facesValue[0],sizeof(uint32_t)*m_facesCount);
            }

            pxr::VtArray<int> holesValue;
            usdMesh.GetHoleIndicesAttr().Get(&holesValue, time);
            if (!holesValue.empty())
            {
                m_holesCount = uint32_t(holesValue.size());

                m_holes = new uint32_t[m_holesCount];
                memcpy(m_holes, &holesValue[0], sizeof(uint32_t) * m_holesCount);
            }

            pxr::TfToken windingOrient = pxr::UsdGeomTokens->rightHanded;
            usdMesh.GetOrientationAttr().Get(&windingOrient);
            if (windingOrient == pxr::UsdGeomTokens->leftHanded)
            {
                m_rightHandedOrientation = false;
            }
        }
    }

    virtual ~TriangulateUSDPrimImpl(void)
    {
        delete []m_vertices;
        delete []m_faceIndices;
        delete []m_faceCounts;
        delete []m_indices;
        if (m_faceMaterials)
            delete []m_faceMaterials;
        if (m_holes)
            delete []m_holes;
    }


    virtual const uint32_t *getFaceIndices(uint32_t &indexCount) const final
    {
        indexCount = m_faceIndicesCount;
        return m_faceIndices;
    }

    virtual const uint32_t *getFaceBuffer(uint32_t &faceCount) const final
    {
        faceCount = m_facesCount;
        return m_faceCounts;
    }

    bool skipFace(uint32_t faceIndex)
    {
        for (uint32_t i = 0; i < m_holesCount; i++)
        {
            if (faceIndex == m_holes[i])
            {
                return true;
            }
        }

        return false;
    }

    // Perform the triangulation of the source data
    // returns the number of triangles produced
    virtual uint32_t triangulate(void) final
    {
        m_triangleCount = 0;
        delete[]m_indices;
        m_indices = nullptr;
        // first compute how many triangles will be needed..
        for (uint32_t i=0; i<m_facesCount; i++)
        {
            if (skipFace(i))
                continue;

            uint32_t count = m_faceCounts[i];
            m_triangleCount+=(count-2);
        }
        m_indices = new uint32_t[m_triangleCount*3];
        m_triangleFaceMapping = new uint32_t[m_triangleCount];
        uint32_t indicesOffset = 0;
        uint32_t *indices = m_indices;
        uint32_t *triangleFaceMapping = m_triangleFaceMapping;
        for (uint32_t i=0; i<m_facesCount; i++)
        {
            if (skipFace(i))
            {
                const uint32_t faceCount = m_faceCounts[i];
                indicesOffset += faceCount;
                continue;
            }

            const uint32_t faceCount = m_faceCounts[i];
            const uint32_t startIndex = m_faceIndices[indicesOffset];
            for (uint32_t faceIndex=0; faceIndex<(faceCount-2); faceIndex++)
            {
                triangleFaceMapping[0] = i;
                indices[0] = startIndex;
                if (m_rightHandedOrientation)
                {
                    indices[1] = m_faceIndices[indicesOffset + faceIndex + 1];
                    indices[2] = m_faceIndices[indicesOffset + faceIndex + 2];
                }
                else
                {
                    indices[1] = m_faceIndices[indicesOffset + faceIndex + 2];
                    indices[2] = m_faceIndices[indicesOffset + faceIndex + 1];
                }
                indices+=3;
                triangleFaceMapping++;
            }
            indicesOffset += faceCount;
        }

        return m_triangleCount;
    }

    virtual float *getVertices(uint32_t &vertexCount) const final
    {
        vertexCount = m_vertexCount;
        return m_vertices;
    }

    virtual uint32_t *getIndices(uint32_t &triangleCount) const final
    {
        triangleCount = m_triangleCount;
        return m_indices;
    }

    virtual uint32_t* getTriangleFaceMap(uint32_t& triangleCount) const final
    {
        triangleCount = m_triangleCount;
        return m_triangleFaceMapping;
    }

    virtual uint16_t* getFaceMaterials(uint32_t& faceCount) const final
    {
        faceCount = m_facesCount;
        return m_faceMaterials;
    }

    virtual void release(void) final
    {
        delete this;
    }

    bool isValid(void) const
    {
        return m_isValid;
    }

    bool        m_isValid{false};
    bool        m_verticesOnly{false};
    bool        m_rightHandedOrientation{true};
    uint32_t    m_vertexCount{0};
    float       *m_vertices{nullptr};

    uint32_t    m_faceIndicesCount{ 0 };
    uint32_t    *m_faceIndices{nullptr};

    uint32_t    m_facesCount{ 0 };
    uint32_t    *m_faceCounts{nullptr};

    uint32_t    m_holesCount{ 0 };
    uint32_t*   m_holes{ nullptr };

    uint32_t    m_triangleCount{0};
    uint32_t    *m_indices{nullptr};

    uint32_t    *m_triangleFaceMapping{ nullptr };
    uint16_t    *m_faceMaterials{ nullptr };
};

TriangulateUSDPrim *TriangulateUSDPrim::create(const pxr::UsdPrim &prim)
{
    auto ret = new TriangulateUSDPrimImpl(prim);
    if ( !ret->isValid() )
    {
        ret->release();
        ret = nullptr;
    }
    return static_cast< TriangulateUSDPrim *>(ret);
}


}
