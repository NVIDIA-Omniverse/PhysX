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
// Copyright (c) 2016-2024 NVIDIA Corporation. All rights reserved.

#define _CRT_SECURE_NO_WARNINGS

#include "NvBlastExtAuthoringMeshImpl.h"
#include "NvBlastExtAuthoringTypes.h"
#include <NvBlastAssert.h>
#include "NvMath.h"
#include <NvBlastNvSharedHelpers.h>
#include <NvBlastVolumeIntegrals.h>
#include <cmath>
#include <string.h>
#include <vector>
#include <algorithm>

namespace Nv
{
namespace Blast
{

MeshImpl::MeshImpl(const NvcVec3* position, const NvcVec3* normals, const NvcVec2* uv, uint32_t verticesCount,
                   const uint32_t* indices, uint32_t indicesCount)
{

    mVertices.resize(verticesCount);
    for (uint32_t i = 0; i < mVertices.size(); ++i)
    {
        mVertices[i].p = position[i];
    }
    if (normals != 0)
    {
        for (uint32_t i = 0; i < mVertices.size(); ++i)
        {
            mVertices[i].n = normals[i];
        }

    }
    else
    {
        for (uint32_t i = 0; i < mVertices.size(); ++i)
        {
            mVertices[i].n = {0, 0, 0};
        }
    }
    if (uv != 0)
    {
        for (uint32_t i = 0; i < mVertices.size(); ++i)
        {
            mVertices[i].uv[0] = uv[i];
        }
    }
    else
    {
        for (uint32_t i = 0; i < mVertices.size(); ++i)
        {
            mVertices[i].uv[0] = {0, 0};
        }
    }
    mEdges.resize(indicesCount);
    mFacets.resize(indicesCount / 3);


    int32_t facetId = 0;
    for (uint32_t i = 0; i < indicesCount; i += 3)
    {
        mEdges[i].s = indices[i];
        mEdges[i].e = indices[i + 1];

        mEdges[i + 1].s = indices[i + 1];
        mEdges[i + 1].e = indices[i + 2];

        mEdges[i + 2].s = indices[i + 2];
        mEdges[i + 2].e = indices[i];
        mFacets[facetId].firstEdgeNumber = i;
        mFacets[facetId].edgesCount = 3;
        mFacets[facetId].materialId = 0;
        //Unassigned for now
        mFacets[facetId].smoothingGroup = -1;
        facetId++;
    }
    recalculateBoundingBox();
}

MeshImpl::MeshImpl(const Vertex* vertices, const Edge* edges, const Facet* facets, uint32_t posCount, uint32_t edgesCount, uint32_t facetsCount)
{
    mVertices.resize(posCount);
    mEdges.resize(edgesCount);
    mFacets.resize(facetsCount);

    memcpy(mVertices.data(), vertices, sizeof(Vertex) * posCount);
    memcpy(mEdges.data(), edges, sizeof(Edge) * edgesCount);
    memcpy(mFacets.data(), facets, sizeof(Facet) * facetsCount);
    recalculateBoundingBox();   
}

MeshImpl::MeshImpl(const Vertex* vertices, uint32_t count)
{
    mVertices = std::vector<Vertex>(vertices, vertices + count);
    mEdges.resize(count);
    mFacets.resize(count / 3);
    uint32_t vp = 0;
    for (uint32_t i = 0; i < count; i += 3)
    {
        mEdges[i].s = vp;
        mEdges[i].e = vp + 1;

        mEdges[i + 1].s = vp + 1;
        mEdges[i + 1].e = vp + 2;

        mEdges[i + 2].s = vp + 2;
        mEdges[i + 2].e = vp;
        vp += 3;
    }
    for (uint32_t i = 0; i < count / 3; ++i)
    {
        mFacets[i].edgesCount = 3;
        mFacets[i].firstEdgeNumber = i * 3;
    }
    recalculateBoundingBox();
}

MeshImpl::MeshImpl(const Vertex* vertices, uint32_t count, uint32_t* indices, uint32_t indexCount, void* materials, uint32_t materialStride)
{
    mVertices = std::vector<Vertex>(vertices, vertices + count);
    mEdges.resize(indexCount);
    mFacets.resize(indexCount / 3);

    for (uint32_t i = 0; i < indexCount; i += 3)
    {
        mEdges[i].s = indices[i];
        mEdges[i].e = indices[i + 1];

        mEdges[i + 1].s = indices[i + 1];
        mEdges[i + 1].e = indices[i + 2];

        mEdges[i + 2].s = indices[i + 2];
        mEdges[i + 2].e = indices[i];
    }
    for (uint32_t i = 0; i < indexCount / 3; ++i)
    {
        mFacets[i].edgesCount = 3;
        mFacets[i].firstEdgeNumber = i * 3;
        mFacets[i].userData = 0;
        if (materials != nullptr)
        {
            mFacets[i].materialId = *(uint32_t*)((uint8_t*)materials + i * materialStride);
        }
    }
    recalculateBoundingBox();
}

float MeshImpl::getMeshVolumeAndCentroid(NvcVec3& centroid) const
{
    class MeshImplQuery
    {
    public:
        MeshImplQuery(const MeshImpl& mesh) : m_mesh(mesh) {}

        size_t faceCount() const { return (size_t)m_mesh.getFacetCount(); }

        size_t vertexCount(size_t faceIndex) const { return (size_t)m_mesh.getFacet((int32_t)faceIndex)->edgesCount; }

        NvcVec3 vertex(size_t faceIndex, size_t vertexIndex) const
        {
            const Nv::Blast::Facet* facet = m_mesh.getFacet(faceIndex);
            return m_mesh.getVertices()[m_mesh.getEdges()[facet->firstEdgeNumber + vertexIndex].s].p;
        }

        const MeshImpl& m_mesh;
    };

    return calculateMeshVolumeAndCentroid<MeshImplQuery>(centroid, *this);
}

uint32_t MeshImpl::getFacetCount() const
{
    return static_cast<uint32_t>(mFacets.size());
}

Vertex* MeshImpl::getVerticesWritable()
{
    return mVertices.data();
}

Edge* MeshImpl::getEdgesWritable()
{
    return mEdges.data();
}

const Vertex* MeshImpl::getVertices() const
{
    return mVertices.data();
}

const Edge* MeshImpl::getEdges() const
{
    return mEdges.data();
}

uint32_t MeshImpl::getEdgesCount() const
{
    return static_cast<uint32_t>(mEdges.size());
}
uint32_t MeshImpl::getVerticesCount() const
{
    return static_cast<uint32_t>(mVertices.size());
}
Facet* MeshImpl::getFacetsBufferWritable()
{
    return mFacets.data();
}
const Facet* MeshImpl::getFacetsBuffer() const
{
    return mFacets.data();
}
Facet* MeshImpl::getFacetWritable(int32_t facet)
{
    return &mFacets[facet];
}
const Facet* MeshImpl::getFacet(int32_t facet) const
{
    return &mFacets[facet];
}

MeshImpl::~MeshImpl()
{
}

void MeshImpl::release()
{
    delete this;
}

const NvcBounds3& MeshImpl::getBoundingBox() const
{
    return fromNvShared(mBounds);
}

NvcBounds3& MeshImpl::getBoundingBoxWritable()
{
    return fromNvShared(mBounds);
}


void MeshImpl::recalculateBoundingBox()
{
    mBounds.setEmpty();
    for (uint32_t i = 0; i < mVertices.size(); ++i)
    {
        mBounds.include(toNvShared(mVertices[i].p));
    }
    calcPerFacetBounds();
}

const NvcBounds3* MeshImpl::getFacetBound(uint32_t index) const 
{
    if (mPerFacetBounds.empty())
    {
        return nullptr;
    }
    return &fromNvShared(mPerFacetBounds[index]);
}

void MeshImpl::calcPerFacetBounds()
{
    mPerFacetBounds.resize(mFacets.size());

    for (uint32_t i = 0; i < mFacets.size(); ++i)
    {
        auto& fb = mPerFacetBounds[i];
        fb.setEmpty();

        for (uint32_t v = 0; v < mFacets[i].edgesCount; ++v)
        {
            fb.include(toNvShared(mVertices[mEdges[mFacets[i].firstEdgeNumber + v].s].p));
            fb.include(toNvShared(mVertices[mEdges[mFacets[i].firstEdgeNumber + v].e].p));
        }
    }
}

void MeshImpl::setMaterialId(const int32_t* materialId)
{
    if (materialId != nullptr)
    {
        for (uint32_t i = 0; i < mFacets.size(); ++i)
        {
            mFacets[i].materialId = *materialId;
            ++materialId;
        }
    }
}

bool MeshImpl::isValid() const
{
    return mVertices.size() > 0 && mEdges.size() > 0 && mFacets.size() > 0;
}


void MeshImpl::replaceMaterialId(int32_t oldMaterialId, int32_t newMaterialId)
{
    for (uint32_t i = 0; i < mFacets.size(); ++i)
    {
        if (mFacets[i].materialId == oldMaterialId)
        {
            mFacets[i].materialId = newMaterialId;
        }
    }
}

void MeshImpl::setSmoothingGroup(const int32_t* smoothingGroups)
{
    if (smoothingGroups != nullptr)
    {
        for (uint32_t i = 0; i < mFacets.size(); ++i)
        {
            mFacets[i].smoothingGroup = *smoothingGroups;
            ++smoothingGroups;
        }
    }
}



} // namespace Blast
} // namespace Nv
