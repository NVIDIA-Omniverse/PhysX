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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxMat44.h"
#include "assert.h"
#include "utils.cuh"
#include "PxgInterpolation.h"
#include <stdio.h>
#include "PxDeformableSkinning.h"
#include "atomic.cuh"

using namespace physx;

extern "C" __host__ void initCommonKernels2() {}

extern "C" __global__ void interleaveBuffers(const float4* PX_RESTRICT vertices, const float4* PX_RESTRICT normals, PxU32 length, PxVec3* interleavedResultBuffer)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= length)
		return;

	float4 v = vertices[threadIndex];
	float4 n = normals[threadIndex];
	interleavedResultBuffer[2 * threadIndex] = PxVec3(v.x, v.y, v.z);
	interleavedResultBuffer[2 * threadIndex + 1] = PxVec3(n.x, n.y, n.z);
}

//A bit experimental. Can help to get smoother transition between triangles.
__device__ static PxVec3 modifyBarycentrics(PxVec3 bary)
{
    //Use cubic basis function
    bary.x = 3.0f * bary.x * bary.x - 2.0f * bary.x * bary.x * bary.x;
    bary.y = 3.0f * bary.y * bary.y - 2.0f * bary.y * bary.y * bary.y;
    bary.z = 3.0f * bary.z * bary.z - 2.0f * bary.z * bary.z * bary.z;

    float sum = bary.x + bary.y + bary.z;
    bary *= 1.0f / sum;
    return bary;
}

//The paper https://perso.telecom-paristech.fr/boubek/papers/PhongTessellation/PhongTessellation.pdf uses alpha = 0.75 but a slightly lower value 
//seems to be better. 0.5 is too low, so 0.625 (middle of 0.5 and 0.75) was chosen.
template<bool normalsAreNormalized = true>
__device__ static PxVec3 evaluatePointPhongInterpolation(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& uvw_,
    const PxVec3& nA, const PxVec3& nB, const PxVec3& nC, PxReal halfSurfaceThickness, PxReal alpha = 0.625f)
{
    PxVec3 uvw = false ? modifyBarycentrics(uvw_) : uvw_;
    PxVec3 q = uvw.x * a + uvw.y * b + uvw.z * c;

    PxReal scale1 = (q - a).dot(nA);
    if (!normalsAreNormalized)
        scale1 /= nA.magnitudeSquared();
    PxVec3 projA = q - scale1 * nA;

    PxReal scale2 = (q - b).dot(nB);
    if (!normalsAreNormalized)
        scale2 /= nB.magnitudeSquared();
    PxVec3 projB = q - scale2 * nB;

    PxReal scale3 = (q - c).dot(nC);
    if (!normalsAreNormalized)
        scale3 /= nC.magnitudeSquared();
    PxVec3 projC = q - scale3 * nC;

    //uvw = Pow(uvw, 1.5); //Experimental

    PxVec3 qStar = uvw.x * projA + uvw.y * projB + uvw.z * projC;

    PxVec3 dir = qStar - q;
    PxReal offset = dir.normalizeSafe() * alpha;

    //Asymptotic function applied to offset such that the magnitude of offset cannot exceed halfSurfaceThickness
    
    PxReal ratio = 0.0f;
    if (halfSurfaceThickness > 0.0f)
    {
        ratio = offset / halfSurfaceThickness;
        ratio = tanhf(ratio); //Derivative at zero of tanh is one and tanh asymptotically reaches 1 - this is kind of a softMin(val, 1)
    }
    offset = ratio * halfSurfaceThickness;
    
    return q + offset * dir;
}

extern "C" __global__
void normalVectorsAreaWeighted(
    PxTrimeshSkinningGpuData* data)
{
    PxTrimeshSkinningGpuData& d = data[blockIdx.y]; //TODO: Copy into shared memory

    const PxU32 xDim = gridDim.x * blockDim.x;
    const PxU32 loopEnd = d.nbGuideTriangles;
    for (PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x; threadIndex < loopEnd; threadIndex += xDim)
    {
        const PxU32* tri = &d.guideTrianglesD[3 * threadIndex];

        const PxVec3 p0 = d.guideVerticesD.at(tri[0]);
        const PxVec3 p1 = d.guideVerticesD.at(tri[1]);
        const PxVec3 p2 = d.guideVerticesD.at(tri[2]);
        const PxVec3 n = (p1 - p0).cross(p2 - p0);

        AtomicAdd3(d.guideNormalsD.atRef(tri[0]), n);
        AtomicAdd3(d.guideNormalsD.atRef(tri[1]), n);
        AtomicAdd3(d.guideNormalsD.atRef(tri[2]), n);
    }
}

extern "C" __global__
void zeroNormals(
    PxTrimeshSkinningGpuData* data)
{
    PxTrimeshSkinningGpuData& d = data[blockIdx.y]; //TODO: Copy into shared memory

    const PxU32 xDim = gridDim.x * blockDim.x;
    const PxU32 loopEnd = d.guideVerticesD.count;
    for (PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x; threadIndex < loopEnd; threadIndex += xDim)
    {
        d.guideNormalsD.atRef(threadIndex) = PxVec3(0.0f);
    }
}

extern "C" __global__
void normalizeNormals(
    PxTrimeshSkinningGpuData* data)
{
    PxTrimeshSkinningGpuData& d = data[blockIdx.y]; //TODO: Copy into shared memory

    const PxU32 xDim = gridDim.x * blockDim.x;
    const PxU32 loopEnd = d.guideVerticesD.count;
    for (PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x; threadIndex < loopEnd; threadIndex += xDim)
    {
        d.guideNormalsD.atRef(threadIndex).normalizeSafe();
    }
}

extern "C" __global__
void interpolateSkinnedClothVertices(
    PxTrimeshSkinningGpuData* data)
{
    PxTrimeshSkinningGpuData& d = data[blockIdx.y]; //TODO: Copy into shared memory

    const PxU32 xDim = gridDim.x * blockDim.x;
    const PxU32 loopEnd = d.skinnedVerticesD.count;
    for (PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x; threadIndex < loopEnd; threadIndex += xDim)
    {

        PxTriangleMeshEmbeddingInfo info = d.skinningInfoPerVertexD[threadIndex];
        const PxU32* tri = &d.guideTrianglesD[3 * info.guideTriangleId];
        PxReal w = 1.0f - info.uv.x - info.uv.y;

        PxVec3 uvw(info.uv.x, info.uv.y, w);
        PxVec3 uvwProj = uvw.maximum(PxVec3(0.0));
        PxReal sumProj = uvwProj.x + uvwProj.y + uvwProj.z;
        if(sumProj > 0.0f)
        {
            uvwProj *= 1.0f / sumProj;
        }

        PxVec3 nA = d.guideNormalsD.at(tri[0]);
        PxVec3 nB = d.guideNormalsD.at(tri[1]);
        PxVec3 nC = d.guideNormalsD.at(tri[2]);

        PxVec3 normal = uvwProj.x * nA + uvwProj.y * nB + uvwProj.z * nC;
        normal.normalizeSafe();

        PxVec3 pointPhong = evaluatePointPhongInterpolation(
            d.guideVerticesD.at(tri[0]),
            d.guideVerticesD.at(tri[1]),
            d.guideVerticesD.at(tri[2]),
            uvwProj, nA, nB, nC, d.halfSurfaceThickness);

        PxVec3 pointUVW = uvw.x * d.guideVerticesD.at(tri[0]) + uvw.y * d.guideVerticesD.at(tri[1]) + uvw.z * d.guideVerticesD.at(tri[2]);
        PxVec3 pointUVWProj = uvwProj.x * d.guideVerticesD.at(tri[0]) + uvwProj.y * d.guideVerticesD.at(tri[1]) + uvwProj.z * d.guideVerticesD.at(tri[2]);

        //The offset could also be used to modify the alpha factor of the method EvaluatePoint. Or one could introduce an offset to EvaluatePoint that offsets along the same direction as the alpha factor
        PxVec3 offsetPoint = pointPhong + info.offsetAlongInterpolatedNormal * normal + pointUVW - pointUVWProj;

        d.skinnedVerticesD.atRef(threadIndex) = offsetPoint;
    }
}


extern "C" __global__
void interpolateSkinnedSoftBodyVertices(
    PxTetmeshSkinningGpuData* data)
{
    PxTetmeshSkinningGpuData& d = data[blockIdx.y]; //TODO: Copy into shared memory

    const PxU32 xDim = gridDim.x * blockDim.x;
    const PxU32 loopEnd = d.skinnedVerticesD.count;
    for (PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x; threadIndex < loopEnd; threadIndex += xDim)
    {
        //Uses linear barycentric interpolation - plenty of room for improvements

        PxTetrahedronMeshEmbeddingInfo info = d.skinningInfoPerVertexD[threadIndex];
        const PxU32* tet = &d.guideTetrahedraD[4 * info.guideTetrahedronId];
        PxReal s = 1.0f - info.uvw.x - info.uvw.y - info.uvw.z;

        PxVec3 point =
            info.uvw.x * d.guideVerticesD.at(tet[0]) +
            info.uvw.y * d.guideVerticesD.at(tet[1]) +
            info.uvw.z * d.guideVerticesD.at(tet[2]) +
            s * d.guideVerticesD.at(tet[3]);

        d.skinnedVerticesD.atRef(threadIndex) = point;
    }
}
