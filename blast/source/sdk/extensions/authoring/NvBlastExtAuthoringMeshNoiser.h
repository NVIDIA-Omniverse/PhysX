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


#ifndef NVBLASTEXTAUTHORINGMESHNOISER_H
#define NVBLASTEXTAUTHORINGMESHNOISER_H
#include <vector>
#include <map>
#include "NvBlastExtAuthoringInternalCommon.h"

namespace Nv
{
    namespace Blast
    {
        class SimplexNoise;

        /**
        Structure used on tesselation stage. Maps edge to two neighbor triangles
        */
        struct EdgeToTriangles
        {
            int32_t tr[2];
            int32_t c;
            EdgeToTriangles()
            {
                c = 0;
            }
            /**
            Add triangle to edge. Should not be called more than twice for one edge!!!!.
            */
            void    add(int32_t t)
            {
                tr[c] = t;
                ++c;
            }
            /**
            Replaces mapping from one triangle to another.
            */
            void    replace(int32_t from, int32_t to)
            {
                if (tr[0] == from)
                {
                    tr[0] = to;
                }
                else
                {
                    if (c == 2 && tr[1] == from)
                    {
                        tr[1] = to;
                    }
                }
            }
            /**
            Get triangle which is mapped by this edge and which index is different than provided.
            */
            int32_t getNot(int32_t id)
            {
                if (tr[0] != id)
                {
                    return tr[0];
                }
                if (c == 2 && tr[1] != id)
                {
                    return tr[1];
                }
                return -1;
            }

        };

        /**
            Tool for graphic mesh tesselation and adding noise to internal surface. Each triangle must have initialized 
            Triangle::userInfo field (0 for external surface triangles and != 0 for internal)
        */
        class MeshNoiser
        {
        public:
            MeshNoiser()
            {
                reset();
            }

            void reset();

            /**
            Edge flags
            */
            enum EdgeFlag { INTERNAL_EDGE, EXTERNAL_BORDER_EDGE, INTERNAL_BORDER_EDGE, EXTERNAL_EDGE, NONE };

    
            /**
                Set mesh to tesselate and apply noise
            */
            void setMesh(const std::vector<Triangle>& mesh);

            /**
            Tesselate internal surface.
            \param[in] maxLen - maximal length of edge on internal surface.
            */
            void                            tesselateInternalSurface(float maxLen);

            /**
            Apply noise to internal surface. Must be called only after tesselation!!!
            \param[in] noise - noise generator
            \param[in] falloff - damping of noise around of external surface
            \param[in] relaxIterations - number of smoothing iterations before applying noise
            \param[in] relaxFactor - amount of smooting before applying noise.
            */
            void                            applyNoise(SimplexNoise& noise, float falloff, int32_t relaxIterations, float relaxFactor);

            std::vector<Triangle>           getMesh();

        private:
            nvidia::NvVec3                       mOffset;
            float                               mScale;
            bool                                isTesselated;
            /**
                Mesh data
            */
            std::vector<Vertex>                 mVertices;
            std::vector<TriangleIndexed>        mTriangles;
            std::vector<Edge>                   mEdges;
            std::map<Vertex, int32_t, VrtComp>  mVertMap;
            std::map<Edge, int32_t>             mEdgeMap;


            /**
                Final triangles.
            */
            std::vector<Triangle>           mResultTriangles;


            int32_t                         addVerticeIfNotExist(const Vertex& p);
            int32_t                         addEdge(const Edge& e);
            int32_t                         findEdge(const Edge& e);



            void                            collapseEdge(int32_t id);
            void                            divideEdge(int32_t id);
            void                            updateVertEdgeInfo();
            void                            updateEdgeTriangleInfo();
            void                            relax(int32_t iterations, float factor, std::vector<Vertex>& vertices);
            void                            recalcNoiseDirs();


            std::vector<bool>                           mRestrictionFlag;
            std::vector<EdgeFlag>                       mEdgeFlag;
            std::vector<EdgeToTriangles>                mTrMeshEdToTr;
            std::vector<int32_t>                        mVertexValence;
            std::vector<std::vector<int32_t> >          mVertexToTriangleMap;



            std::vector<float>                          mVerticesDistances;
            std::vector<nvidia::NvVec3>                  mVerticesNormalsSmoothed;
            std::vector<uint32_t>                       mPositionMappedVrt;
            std::vector<std::vector<int32_t> >          mGeometryGraph;

            void                                        prebuildEdgeFlagArray();
            void                                        computePositionedMapping();
            void                                        computeFalloffAndNormals();

            void                                        prebuildTesselatedTriangles();
        };

    } // namespace Blast
} // namespace Nv
#endif // ! NVBLASTEXTAUTHORINGMESHNOISER_H
