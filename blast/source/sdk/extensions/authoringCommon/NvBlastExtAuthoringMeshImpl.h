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

#ifndef NVBLASTAUTHORINGMESHIMPL_H
#define NVBLASTAUTHORINGMESHIMPL_H

#include "NvBlastExtAuthoringMesh.h"
#include "NvBounds3.h"
#include <vector>
#include <map>
#include <set>

namespace Nv
{
namespace Blast
{

/**
    Class for internal mesh representation
*/
class MeshImpl : public Mesh
{
public:
        
    /**
        Constructs mesh object from array of triangles.
        \param[in] position         Array of vertex positions
        \param[in] normals          Array of vertex normals
        \param[in] uv               Array of vertex uv coordinates
        \param[in] verticesCount    Vertices count
        \param[in] indices          Array of vertex indices. Indices contain vertex index triplets which form a mesh triangle. 
        \param[in] indicesCount     Indices count (should be equal to numberOfTriangles * 3)
    */
    MeshImpl(const NvcVec3* position, const NvcVec3* normals, const NvcVec2* uv, uint32_t verticesCount, const uint32_t* indices, uint32_t indicesCount);

    /**
        Constructs mesh object from array of facets.
        \param[in] vertices     Array of vertices
        \param[in] edges        Array of edges
        \param[in] facets       Array of facets
        \param[in] posCount     Vertices count
        \param[in] edgesCount   Edges count
        \param[in] facetsCount  Facets count
    */
    MeshImpl(const Vertex* vertices, const Edge* edges, const Facet* facets, uint32_t posCount, uint32_t edgesCount, uint32_t facetsCount);

    MeshImpl(const Vertex* vertices, uint32_t count);

    MeshImpl(const Vertex* vertices, uint32_t count, uint32_t* indices, uint32_t indexCount, void* materials, uint32_t materialStride);

    ~MeshImpl();

    virtual void        release() override;

    /**
        Return true if mesh is valid
    */
    bool                isValid() const override;

    /**
        Return pointer on vertices array
    */
    Vertex*             getVerticesWritable() override;

    /**
        Return pointer on edges array
    */
    Edge*               getEdgesWritable() override;

    /**
        Return pointer on facets array
    */
    Facet*              getFacetsBufferWritable() override;

    /**
    Return pointer on vertices array
    */
    const Vertex*           getVertices() const override;

    /**
    Return pointer on edges array
    */
    const Edge*             getEdges() const override;

    /**
    Return pointer on facets array
    */
    const Facet*            getFacetsBuffer() const override;

    /**
        Return writable pointer on specified facet
    */
    Facet*              getFacetWritable(int32_t facet) override;

    /**
    Return writable pointer on specified facet
    */
    const Facet*        getFacet(int32_t facet) const override;

    /**
        Return edges count
    */
    uint32_t            getEdgesCount() const override;

    /**
        Return vertices count
    */
    uint32_t            getVerticesCount() const override;

    /**
        Return facet count
    */
    uint32_t            getFacetCount() const override;


    /**
        Return reference on mesh bounding box.
    */
    const NvcBounds3&   getBoundingBox() const override;

    /**
        Return writable reference on mesh bounding box.
    */
    NvcBounds3& getBoundingBoxWritable() override;

    /**
        Recalculate bounding box
    */
    void                recalculateBoundingBox() override;

    /**
        Compute mesh volume and centroid.  Assumes mesh has outward normals and no holes.
    */
    float               getMeshVolumeAndCentroid(NvcVec3& centroid) const override;

    /**
    Set per-facet material id.
    */
    void    setMaterialId(const int32_t* materialIds) override;

    /**
    Replaces an material id on faces with a new one
    */
    void    replaceMaterialId(int32_t oldMaterialId, int32_t newMaterialId) override;

    /**
    Set per-facet smoothing group.
    */
    void    setSmoothingGroup(const int32_t* smoothingGroups) override;

    /**
        Calculate per-facet bounding boxes.
    */
    virtual void                        calcPerFacetBounds() override;

    /**
        Get pointer on facet bounding box, if not calculated return nullptr.
    */
    virtual const  NvcBounds3*  getFacetBound(uint32_t index) const override;

private:
    std::vector<Vertex> mVertices;
    std::vector<Edge>   mEdges;
    std::vector<Facet>  mFacets;
    nvidia::NvBounds3 mBounds;
    std::vector<nvidia::NvBounds3> mPerFacetBounds;
};

} // namespace Blast
} // namespace Nv


#endif // ifndef NVBLASTAUTHORINGMESHIMPL_H
