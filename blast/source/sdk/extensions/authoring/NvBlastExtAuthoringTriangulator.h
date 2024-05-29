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


#ifndef NVBLASTEXTAUTHORINGTRIANGULATOR_H
#define NVBLASTEXTAUTHORINGTRIANGULATOR_H


#include <vector>
#include <map>
#include "NvBlastExtAuthoringTypes.h"
#include "NvBlastExtAuthoringMesh.h"
#include "NvBlastExtAuthoringInternalCommon.h"

namespace Nv
{
namespace Blast
{


/**
    Tool for doing all post processing steps of authoring.
*/
class Triangulator
{
public:
    /**
        Triangulates provided mesh and saves result internally. Uses Ear-clipping algorithm.
        \param[in] mesh Mesh for triangulation
    */
    void                            triangulate(const Mesh* mesh);

    /**
        \return Return array of triangles of base mesh.
    */
    std::vector<Triangle>&          getBaseMesh()
    {
        return mBaseMeshUVFittedTriangles;
    }

    std::vector<Triangle>&          getBaseMeshNotFitted()
    {
        return mBaseMeshResultTriangles;
    }


    /**
        \return Return array of TriangleIndexed of base mesh. Each TriangleIndexed contains index of corresponding vertex in internal vertex buffer.
    */
    std::vector<TriangleIndexed>&   getBaseMeshIndexed()
    {
        return mBaseMeshTriangles;
    }
    /**
        \return Return mapping from vertices of input Mesh to internal vertices buffer. Used for island detection.
    */
    std::vector<uint32_t>&          getBaseMapping()
    {
        return mBaseMapping;
    };
    /**
        \return Return mapping from vertices of input Mesh to internal vertices buffer, only positions are accounted. Used for island detection. 
    */
    std::vector<int32_t>&           getPositionedMapping()
    {
        return mPositionMappedVrt;
    };
    /**
        \return Return internal vertex buffer size. Vertices internally are welded with some threshold.
    */
    uint32_t                        getWeldedVerticesCount()
    {
        return static_cast<uint32_t>(mVertices.size());
    }   

    /**
        Removes all information about mesh triangulation.
    */
    void                            reset();

    int32_t&                        getParentChunkId() { return parentChunkId; };

private:

    int32_t                         parentChunkId;

    int32_t                         addVerticeIfNotExist(const Vertex& p);
    void                            addEdgeIfValid(EdgeWithParent& ed);
    
    /* Data used before triangulation to build polygon loops*/

    std::vector<Vertex>                                 mVertices;
    std::vector<EdgeWithParent>                         mBaseMeshEdges;
    std::map<Vertex, int32_t, VrtComp>                  mVertMap;
    std::map<EdgeWithParent, int32_t, EdgeComparator>   mEdgeMap;
    std::vector<uint32_t>                               mBaseMapping;
    std::vector<int32_t>                                mPositionMappedVrt;
    /* ------------------------------------------------------------ */


    /**
        Unite all almost similar vertices, update edges according to this changes
    */
    void                            prepare(const Mesh* mesh);

        
            
    void                            triangulatePolygonWithEarClipping(const std::vector<uint32_t>& inputPolygon, const Vertex* vert, const ProjectionDirections& dir);
    void                            buildPolygonAndTriangulate(std::vector<Edge>& edges, Vertex* vertices, int32_t userData, int32_t materialId, int32_t smoothingGroup);
    void                            computePositionedMapping();
    
    std::vector<TriangleIndexed>                        mBaseMeshTriangles; 
    /**
        Final triangles
    */
    std::vector<Triangle>                               mBaseMeshResultTriangles;
    std::vector<Triangle>                               mBaseMeshUVFittedTriangles;
};

} // namespace Blast
} // namespace Nv


#endif  // ifndef NVBLASTEXTAUTHORINGTRIANGULATOR_H
