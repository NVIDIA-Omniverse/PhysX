// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// clang-format off
#include "UsdPCH.h"
// clang-format on

#include "TriangulateUsdMeshPrim.h"
#include <common/utilities/UsdMaterialParsing.h>

namespace triangulateusd
{

class TriangulateUSDPrimImpl : public TriangulateUSDPrim
{
public:
    TriangulateUSDPrimImpl(const pxr::UsdPrim &usdPrim, uint16_t& maxMaterialIndex)
    {
        if (usdPrim.IsA<pxr::UsdGeomMesh>())
        {
            const pxr::UsdGeomMesh usdMesh(usdPrim);
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

            // gather subsets
            m_faceMaterials = new uint16_t[m_facesCount];
            if(!TriangulateUSDPrim::fillFaceMaterials(usdPrim, {m_faceMaterials, m_facesCount}, time, maxMaterialIndex))
            {
                delete []m_faceMaterials;
                m_faceMaterials = nullptr;
            }
        }
    }

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

TriangulateUSDPrim *TriangulateUSDPrim::create(const pxr::UsdPrim &prim, uint16_t& numMaterials)
{
    auto ret = new TriangulateUSDPrimImpl(prim, numMaterials);
    if ( !ret->isValid() )
    {
        ret->release();
        ret = nullptr;
    }
    return static_cast< TriangulateUSDPrim *>(ret);
}

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

bool TriangulateUSDPrim::TriangulateUSDPrim::fillFaceMaterials(const pxr::UsdPrim& usdPrim,
                                                               omni::span<uint16_t> faceMaterials,
                                                               pxr::UsdTimeCode time,
                                                               uint16_t& maxMaterialIndex)
{
    const pxr::UsdGeomMesh usdMesh(usdPrim);
    const std::vector<pxr::UsdGeomSubset> subsets = pxr::UsdGeomSubset::GetGeomSubsets(usdMesh, pxr::UsdGeomTokens->face);
    bool materialsFound = false;
    maxMaterialIndex = 0;
    if (!subsets.empty())
    {
        // gather all materials first
        pxr::TfHashSet<pxr::SdfPath, pxr::SdfPath::Hash> materials;
        for (const pxr::UsdGeomSubset& subset : subsets)
        {
            const pxr::SdfPath material = usdmaterialutils::getMaterialBinding(subset.GetPrim());
            if (material != pxr::SdfPath())
            {
                const pxr::UsdPrim materialPrim = usdPrim.GetStage()->GetPrimAtPath(material);
                if (materialPrim && materialPrim.HasAPI<pxr::UsdPhysicsMaterialAPI>())
                    materials.insert(subset.GetPath());
            }
        }
        uint16_t materialIndex = 0;
        const size_t facesCount = faceMaterials.size();
        bool firstRun = true;
        for (size_t i = 0; i < subsets.size(); i++)
        {
            const pxr::UsdGeomSubset& subset = subsets[i];

            if (materials.find(subset.GetPath()) != materials.end())
            {
                if (firstRun)
                {
                    // By Default we assign default material to all faces in case subsets are not referencing all faces
                    const uint16_t lastMaterialIndex = (uint16_t)materials.size();
                    for (uint32_t fIndex = 0; fIndex < facesCount; fIndex++)
                    {
                        faceMaterials[fIndex] = lastMaterialIndex;
                    }
                    firstRun = false;
                }
                materialsFound = true;

                pxr::VtArray<int> facesValue;
                subset.GetIndicesAttr().Get(&facesValue, time);
                for (int face : facesValue)
                {
                    CARB_ASSERT((uint32_t)face < facesCount);
                    if ((uint32_t)face < facesCount)
                        faceMaterials[face] = materialIndex;
                }
                materialIndex++;
            }
        }

        maxMaterialIndex = 0;
        for (uint32_t fIndex = 0; fIndex < facesCount; fIndex++)
        {
            if (faceMaterials[fIndex] > maxMaterialIndex)
            {
                maxMaterialIndex = faceMaterials[fIndex];
            }
        }
    }
    return materialsFound;
}
}
