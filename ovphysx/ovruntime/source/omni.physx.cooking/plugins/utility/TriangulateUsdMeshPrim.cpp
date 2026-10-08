// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <cstring>

#include "TriangulateUsdMeshPrim.h"

namespace triangulateusd
{

class TriangulateUSDPrimImpl : public TriangulateUSDPrim
{
public:
    TriangulateUSDPrimImpl(const omni::physx::PhysxCookingMeshView& meshView)
    {
        m_isValid = (meshView.points.size() && meshView.indices.size() && meshView.faces.size());
        if (!m_isValid)
        {
            return;
        }

        m_vertexCount = uint32_t(meshView.points.size());
        m_vertices = new float[m_vertexCount * 3];
        memcpy(m_vertices, (float*)meshView.points.data(), sizeof(carb::Float3) * m_vertexCount);

        m_faceIndicesCount = uint32_t(meshView.indices.size());
        m_faceIndices = new uint32_t[m_faceIndicesCount];
        memcpy(m_faceIndices, meshView.indices.data(), m_faceIndicesCount * sizeof(uint32_t));

        m_facesCount = uint32_t(meshView.faces.size());
        m_faceCounts = new uint32_t[m_facesCount];
        memcpy(m_faceCounts, meshView.faces.data(), m_facesCount * sizeof(uint32_t));

        const uint32_t faceMaterialsCount = uint32_t(meshView.faceMaterials.size());
        if (faceMaterialsCount)
        {
            m_faceMaterials = new uint16_t[faceMaterialsCount];
            memcpy(m_faceMaterials, meshView.faceMaterials.data(), faceMaterialsCount * sizeof(uint16_t));
        }

        m_holesCount = uint32_t(meshView.holeIndices.size());
        if (m_holesCount)
        {
            m_holes = new uint32_t[m_holesCount];
            memcpy(m_holes, meshView.holeIndices.data(), m_holesCount * sizeof(uint32_t));
        }

        m_rightHandedOrientation = meshView.rightHandedOrientation;
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

            const uint32_t count = m_faceCounts[i];
            if (count > 2)
            {
                m_triangleCount += (count - 2);
            }
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
            if (faceCount > 2)
            {
                const uint32_t startIndex = m_faceIndices[indicesOffset];
                for (uint32_t faceIndex = 0; faceIndex < (faceCount - 2); faceIndex++)
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
                    indices += 3;
                    triangleFaceMapping++;
                }
                indicesOffset += faceCount;
            }
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

TriangulateUSDPrim *TriangulateUSDPrim::create(const omni::physx::PhysxCookingMeshView& meshView)
{
    auto ret = new TriangulateUSDPrimImpl(meshView);
    if ( !ret->isValid() )
    {
        ret->release();
        ret = nullptr;
    }
    return static_cast< TriangulateUSDPrim *>(ret);
}
}
