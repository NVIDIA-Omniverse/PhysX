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


#ifndef NVBLASTVOLUMEINTEGRALS_H
#define NVBLASTVOLUMEINTEGRALS_H

#include "NvBlastNvSharedHelpers.h"
#include "NvCMath.h"
#include "NvBlastAssert.h"


namespace Nv {
namespace Blast{


/**
Calculate the volume and centroid of a closed mesh with outward-pointing normals.
\param[out] centroid    the calculated centroid of the given mesh
\param[in]  mesh        a class of templated type MeshQuery

MeshQuery must support the following functions:

size_t faceCount()
size_t vertexCount(size_t faceIndex)
NvcVec3 vertex(size_t faceIndex, size_t vertexIndex)

\return the volume of the given mesh
*/
template<class MeshQuery>
NV_INLINE float calculateMeshVolumeAndCentroid(NvcVec3& centroid, const MeshQuery& mesh)
{
    centroid = { 0.0f, 0.0f, 0.0f };

    // First find an approximate centroid for a more accurate calculation
    size_t N = 0;
    NvcVec3 disp = { 0.0f, 0.0f, 0.0f };
    for (size_t i = 0; i < mesh.faceCount(); ++i)
    {
        const size_t faceVertexCount = mesh.vertexCount(i);
        for (size_t j = 0; j < faceVertexCount; ++j)
        {
            disp = disp + mesh.vertex(i, j);
        }
        N += faceVertexCount;
    }

    if (N == 0)
    {
        return 0.0f;
    }

    disp = disp / (float)N;

    float sixV = 0.0f;
    for (size_t i = 0; i < mesh.faceCount(); ++i)
    {
        const size_t faceVertexCount = mesh.vertexCount(i);
        if (faceVertexCount < 3)
        {
            continue;
        }
        const NvcVec3 a = mesh.vertex(i, 0) - disp;
        NvcVec3 b = mesh.vertex(i, 1) - disp;
        for (size_t j = 2; j < faceVertexCount; ++j)
        {
            const NvcVec3 c = mesh.vertex(i, j) - disp;

            const float sixTetV =
                a.x * b.y * c.z - a.x * b.z * c.y - a.y * b.x * c.z +
                a.y * b.z * c.x + a.z * b.x * c.y - a.z * b.y * c.x;

            sixV += sixTetV;

            centroid = centroid + sixTetV*(a + b + c);

            b = c;
        }
    }

    // Extra factor of four to average tet vertices
    centroid = centroid / (4.0f * sixV) + disp;

    return std::abs(sixV) / 6.0f;
}


} // namespace Blast
} // namespace Nv


#endif // NVBLASTVOLUMEINTEGRALS_H
