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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved. 

#ifndef __SDF_COLLISION_CUH__
#define __SDF_COLLISION_CUH__

#include "cudaNpCommon.h"
#include "foundation/PxVec3.h"
#include "geometry/PxMeshScale.h"
#include "GuDistancePointTetrahedron.h"
#include "GuTriangleRefinement.h"
#include "triangleMesh.cuh"
#include "utils.cuh"

namespace physx
{

struct PX_ALIGN_PREFIX(16) SparseSDFTexture
{
	//Dense info - for coarse resolution background grid	
	PxVec3 sdfBoxLower;
	PxReal sdfDx;
	PxVec3 sdfBoxHigher;
	PxReal invSdfDx;
	CUtexObject texture;

	//Sparse info - for high resolution in narrow band around isolevel 0
	CUtexObject textureSubgrids;

	uint3 coarseSize;
	PxU32 subgridSize;

	const PxU32* subgridStartSlots;

	PxReal fineToCoarse; //Inverse of subgridSize

	uint4 sdfDims;
	PxReal subgridsMinSdfValue;
	PxReal subgridsSdfValueRange;

	PX_INLINE __device__ void initialize(const PxgTriangleMesh& mesh)
	{
		texture = mesh.mTexObject;

		const PxReal margin = mesh.subgridSize == 0 ? 0.5f : 0.0f;
		sdfBoxLower = mesh.meshLower + margin * PxVec3(mesh.spacing);
		sdfBoxHigher = mesh.meshLower + PxVec3((mesh.sdfDims.x - margin) * mesh.spacing, (mesh.sdfDims.y - margin) * mesh.spacing, (mesh.sdfDims.z - margin) * mesh.spacing);
		invSdfDx = 1.f / mesh.spacing;
		sdfDx = mesh.spacing;

		textureSubgrids = mesh.mTexObjectSparse;
		subgridStartSlots = mesh.subgridStarts;
		sdfDims = mesh.sdfDims;
		coarseSize.x = mesh.sdfDims.x / PxMax(1u, mesh.subgridSize);
		coarseSize.y = mesh.sdfDims.y / PxMax(1u, mesh.subgridSize);
		coarseSize.z = mesh.sdfDims.z / PxMax(1u, mesh.subgridSize);
		//sdfDx = 1.0f / invSdfDx; // dx;
		subgridSize = mesh.subgridSize;
		fineToCoarse = 1.0f / mesh.subgridSize;

		subgridsMinSdfValue = mesh.subgridsMinSdfValue;
		subgridsSdfValueRange = mesh.subgridsMaxSdfValue - mesh.subgridsMinSdfValue;

		//if (threadIdx.x == 0)
		//	printf("%f, %f\n", mesh.subgridsMinSdfValue, mesh.subgridsMaxSdfValue);
	}

private:
	static PX_INLINE __device__ PxU32 idx(PxU32 x, PxU32 y, PxU32 z, PxU32 width, PxU32 height)
	{
		return z * width * height + y * width + x;
	}
	static PX_INLINE __device__ void applySubgridStart(PxU32 triple, PxVec3& f, const PxU32 subgridSize)
	{
		f.x += (triple & 1023) * (subgridSize + 1);
		triple = triple >> 10;
		f.y += (triple & 1023) * (subgridSize + 1);
		triple = triple >> 10;
		f.z += (triple & 1023) * (subgridSize + 1);
	}

	PX_INLINE __device__ PxReal applySubgridSdfScale(PxReal rawSdfValue) const
	{
		return rawSdfValue * subgridsSdfValueRange + subgridsMinSdfValue;
	}

public:
	PX_INLINE __device__ PxReal Sample(PxVec3 f) const
	{
		if (subgridSize == 0)
		{
			//Dense sdf
			return tex3D<float>(texture, f.x + 0.5f, f.y + 0.5f, f.z + 0.5f);
		}
		else
		{
			const PxU32 xBase = PxClamp(PxU32(f.x * fineToCoarse), 0u, coarseSize.x - 1);
			const PxU32 yBase = PxClamp(PxU32(f.y * fineToCoarse), 0u, coarseSize.y - 1);
			const PxU32 zBase = PxClamp(PxU32(f.z * fineToCoarse), 0u, coarseSize.z - 1);

			const PxU32 i = idx(xBase, yBase, zBase, coarseSize.x, coarseSize.y);
			const PxU32 startSlot = subgridStartSlots[i];
			if (startSlot == 0xFFFFFFFF)
			{
				//Evaluate coarse
				f *= fineToCoarse;
				//+0.5: Align with texel center
				//https://forums.developer.nvidia.com/t/understanding-cuda-texture-2d-linear-interpolation/213924	
				return tex3D<float>(texture, f.x + 0.5f, f.y + 0.5f, f.z + 0.5f);
			}
			else
			{
				f.x = PxClamp(f.x - xBase * subgridSize, 0.0f, PxReal(subgridSize + 1));
				f.y = PxClamp(f.y - yBase * subgridSize, 0.0f, PxReal(subgridSize + 1));
				f.z = PxClamp(f.z - zBase * subgridSize, 0.0f, PxReal(subgridSize + 1));

				//Transform to subgrid storage location in 3D texture
				applySubgridStart(startSlot, f, subgridSize);

				//+0.5: Align with texel center
				//https://forums.developer.nvidia.com/t/understanding-cuda-texture-2d-linear-interpolation/213924	
				PxReal v = tex3D<float>(textureSubgrids, f.x + 0.5f, f.y + 0.5f, f.z + 0.5f);
				
				return applySubgridSdfScale(v);
			}
		}
	}
}
PX_ALIGN_SUFFIX(16);

template<typename T>
PX_INLINE __device__ PxReal PxSdfDistance(const T& texture, const PxVec3& localPos)
{
	// clamp to SDF support
	const PxVec3 clampedGridPt = localPos.maximum(texture.sdfBoxLower).minimum(texture.sdfBoxHigher);

	PxReal diffMag = (localPos - clampedGridPt).magnitude();
	PxVec3 f = (clampedGridPt - texture.sdfBoxLower) * texture.invSdfDx;

	return texture.Sample(f) + diffMag;
}

template<typename T>
PX_INLINE __device__ PxVec3 PxVolumeGrad(const T& texture, const PxVec3& localPos)
{
	PxVec3 grad;

	const PxVec3 clampedGridPt = localPos.maximum(texture.sdfBoxLower).minimum(texture.sdfBoxHigher);

	PxVec3 f = (clampedGridPt - texture.sdfBoxLower) * texture.invSdfDx;

	if (f.x >= 1.0f && f.x <= texture.sdfDims.x - 2.0f &&
		f.y >= 1.0f && f.y <= texture.sdfDims.y - 2.0f &&
		f.z >= 1.0f && f.z <= texture.sdfDims.z - 2.0f)
	{
		grad.x = texture.Sample(PxVec3(f.x + 1.0f, f.y, f.z)) -
			texture.Sample(PxVec3(f.x - 1.0f, f.y, f.z));
		grad.y = texture.Sample(PxVec3(f.x, f.y + 1.0f, f.z)) -
			texture.Sample(PxVec3(f.x, f.y - 1.0f, f.z));
		grad.z = texture.Sample(PxVec3(f.x, f.y, f.z + 1.0f)) -
			texture.Sample(PxVec3(f.x, f.y, f.z - 1.0f));
		return grad;
	}

	grad.x = PxSdfDistance(texture, localPos + PxVec3(texture.sdfDx, 0.f, 0.f)) -
		PxSdfDistance(texture, localPos - PxVec3(texture.sdfDx, 0.f, 0.f));
	grad.y = PxSdfDistance(texture, localPos + PxVec3(0.f, texture.sdfDx, 0.f)) -
		PxSdfDistance(texture, localPos - PxVec3(0.f, texture.sdfDx, 0.f));
	grad.z = PxSdfDistance(texture, localPos + PxVec3(0.f, 0.f, texture.sdfDx)) -
		PxSdfDistance(texture, localPos - PxVec3(0.f, 0.f, texture.sdfDx));

	return grad;
}

template<typename T>
PX_INLINE __device__ PxReal PxSdfSampleWithGrad(const T& texture, const PxVec3& localPos,
	const PxVec3& sdfGradient, PxVec3& dir, PxReal tolerance = PX_MAX_F32)
{
	const PxVec3 clampedGridPt = localPos.maximum(texture.sdfBoxLower).minimum(texture.sdfBoxHigher);

	PxVec3 diff = (localPos - clampedGridPt);
	/*
	const PxReal diffMag = diff.magnitudeSquared();
	if (diffMag > tolerance*tolerance)
		return PX_MAX_F32;*/

	PxVec3 f = (clampedGridPt - texture.sdfBoxLower) * texture.invSdfDx;

	PxReal dist = texture.Sample(f);

	if (dist < tolerance)
	{
		// estimate the contact direction based on the SDF
		// if we are outside the SDF support, add in the distance to the point
		dir = (sdfGradient.getNormalized() * PxAbs(dist) + diff).getNormalized();
		return dist + dir.dot(diff);
	}

	return PX_MAX_F32;
}

static __device__ PxReal doVertexSDFCollision(SparseSDFTexture& texture, const PxVec3& v,
	PxVec3& dir, PxReal tolerance)
{
	PxVec3 sdfGradient = PxVolumeGrad(texture, v);

	if (sdfGradient.normalize() == 0.0f)
	{
		//We ran into a discontinuity e. g. the exact center of a cube
		//Just pick an arbitrary gradient of unit length to move out of the discontinuity
		sdfGradient = PxVec3(0.5448762f, 0.672269f, 0.398825f).getNormalized();
	}

	return PxSdfSampleWithGrad(texture, v, sdfGradient, dir, tolerance);
}


//#define gr 1.6180339887498948482045868343656381177203091798057628621354486227f
#define invGr 0.6180339887498948482045868343656381177203091798057628621354486227f

static __device__ PxReal doSegmentSDFCollision(SparseSDFTexture& texture, const PxVec3& v0, const PxVec3& v1,
	PxVec3& point, PxVec3& dir, PxReal tolerance, PxReal& t)
{
	//Golden section search
	PxReal a = 0.0f;
	PxReal b = 1.0f;
	PxReal c = b - (b - a) * invGr;
	PxReal d = a + (b - a) * invGr;
	for (PxU32 iter = 0; iter < 16; ++iter)
	{
		PxVec3 pC = (1.0f - c) * v0 + c * v1;
		PxReal distC = PxSdfDistance(texture, pC);
		
		PxVec3 pD = (1.0f - d) * v0 + d * v1;
		PxReal distD = PxSdfDistance(texture, pD);

		if (distC < distD)
			b = d;
		else
			a = c;

		c = b - (b - a) * invGr;
		d = a + (b - a) * invGr;
	}

	t = 0.5f * (a + b);
	PxVec3 p = (1.0f - t) * v0 + t * v1;

	PxVec3 sdfGradient = PxVolumeGrad(texture, p);
	if (sdfGradient.normalize() == 0.0f)
	{
		//We ran into a discontinuity e. g. the exact center of a cube
		//Just pick an arbitrary gradient of unit length to move out of the discontinuity
		sdfGradient = PxVec3(0.5448762f, 0.672269f, 0.398825f).getNormalized();
	}

	point = p;
	return PxSdfSampleWithGrad(texture, p, sdfGradient, dir, tolerance);
}

static __device__ PxReal doTetrahedronSDFCollision(SparseSDFTexture& texture, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, const PxVec3& v3,
	PxVec3& point, PxVec3& dir, PxReal tolerance/*, PxVec4& uvw*/)
{
	PxVec3 center = (v0 + v1 + v2 + v3) * 0.25f;
	PxVec3 p = center;

	PxReal dist = PxSdfDistance(texture, p);

	PxReal d0 = PxSdfDistance(texture, v0);
	PxReal d1 = PxSdfDistance(texture, v1);
	PxReal d2 = PxSdfDistance(texture, v2);
	PxReal d3 = PxSdfDistance(texture, v3);

	PxVec4 uvw = PxVec4(0.f);

	if (d0 < d1 && d0 < d2 && d0 < d3 && d0 < dist)
	{
		dist = d0;
		p = v0;
		uvw.x = 1.f;
	}
	else if (d1 < d2 && d1 < d3 && d1 < dist)
	{
		dist = d1;
		p = v1;
		uvw.y = 1.f;
	}
	else if (d2 < d3 && d2 < dist)
	{
		dist = d2;
		p = v2;
		uvw.z = 1.f;
	}
	else if (d3 < dist)
	{
		dist = d3;
		p = v3;
		uvw.w = 1.f;
	}
	else
		uvw = PxVec4(0.25f);

	PxReal difference = PxSqrt(
		PxMax(
			PxMax((v0 - p).magnitudeSquared(), (v1 - p).magnitudeSquared()),
			PxMax((v2 - p).magnitudeSquared(), (v3 - p).magnitudeSquared())));

	const PxReal eps = 1e-4;
	const PxReal toleranceSq = 1e-3f*1e-3f;

	PxVec3 sdfGradient;
	PxReal step = 1.f / (2.f*difference);

	for (PxU32 iter = 0; iter < 16; ++iter)
	{
		sdfGradient = PxVolumeGrad(texture, p);

		// why do we normalize the gradient here?
		if (sdfGradient.normalize() == 0.0f)
		{
			//We ran into a discontinuity e. g. the exact center of a cube
			//Just pick an arbitrary gradient of unit length to move out of the discontinuity
			sdfGradient = PxVec3(0.5448762f, 0.672269f, 0.398825f).getNormalized();
		}

		PxReal dfdu = sdfGradient.dot(v0 - p);
		PxReal dfdv = sdfGradient.dot(v1 - p);
		PxReal dfdw = sdfGradient.dot(v2 - p);
		PxReal dfd4 = sdfGradient.dot(v3 - p);

		PxVec4 newUVW;

		{
			newUVW = uvw;

			newUVW.x -= step * dfdu;
			newUVW.y -= step * dfdv;
			newUVW.z -= step * dfdw;
			newUVW.w -= step * dfd4;

			step = step * 0.8f;


			if (((newUVW.x <= -eps || newUVW.x >= 1.f + eps) || (newUVW.y <= -eps || newUVW.y >= 1.f + eps) ||
				(newUVW.z <= -eps || newUVW.z >= 1.f + eps) || (newUVW.w <= -eps || newUVW.w >= 1.f + eps)))
			{
				PxVec3 cp = Gu::closestPtPointTetrahedron(newUVW.getXYZ(), PxVec3(1.0f, 0.0f, 0.0f), PxVec3(0.0f, 1.0f, 0.0f), PxVec3(0.f, 0.f, 1.f), PxVec3(0.0f));
				newUVW.x = cp.x;
				newUVW.y = cp.y;
				newUVW.z = cp.z;
				newUVW.w = 1.0f - cp.x - cp.y - cp.z; //TODO: Veryfy this logic

				assert(newUVW.w >= -eps && newUVW.w <= 1.f + eps);
			}

			p = v0 * newUVW.x + v1 * newUVW.y + v2 * newUVW.z + v3 * newUVW.w;

			if ((uvw - newUVW).magnitudeSquared() < toleranceSq)
			{
				/*if(iter != 0)
					printf("Iter = %i\n", iter);*/
				break;
			}

			uvw = newUVW;
		}
	}

	sdfGradient = PxVolumeGrad(texture, p);

	point = p;

	dist = PxSdfSampleWithGrad(texture, p, sdfGradient, dir, tolerance);
	return dist;
}

static __device__ PxReal doTriangleSDFCollision(SparseSDFTexture& texture, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2,
	PxVec3& point, PxVec3& dir, PxReal tolerance)
{
	const PxReal third = 1.f / 3.f;
	PxVec3 center = (v0 + v1 + v2) * third;
	PxVec3 p = center;

	PxReal dist = PxSdfDistance(texture, p);

	PxReal d0 = PxSdfDistance(texture, v0);
	PxReal d1 = PxSdfDistance(texture, v1);
	PxReal d2 = PxSdfDistance(texture, v2);

	//PxVec3 nor = (v1 - v0).cross(v2 - v0);

	PxVec3 uvw(0.f);

	// choose starting iterate among centroid and triangle vertices
	if (d0 < d1 && d0 < d2 && d0 < dist)
	{
		p = v0;
		uvw.x = 1.f;
	}
	else if (d1 < d2 && d1 < dist)
	{
		p = v1;
		uvw.y = 1.f;
	}
	else if (d2 < dist)
	{
		p = v2;
		uvw.z = 1.f;
	}
	else
		uvw = PxVec3(third);

	PxReal difference = PxSqrt(PxMax((v0 - p).magnitudeSquared(), PxMax((v1 - p).magnitudeSquared(),
		(v2 - p).magnitudeSquared())));


	const PxReal toleranceSq = 1e-3f*1e-3f;

	PxVec3 sdfGradient;
	PxReal step = 1.f / (2.f*difference);

	for (PxU32 iter = 0; iter < 16; ++iter)
	{
		sdfGradient = PxVolumeGrad(texture, p);

		if (sdfGradient.normalize() == 0.0f)
		{
			//We ran into a discontinuity e. g. the exact center of a cube
			//Just pick an arbitrary gradient of unit length to move out of the discontinuity
			sdfGradient = PxVec3(0.571846586f, 0.705545099f, 0.418566116f);
		}

		PxReal dfdu = sdfGradient.dot(v0 - p);
		PxReal dfdv = sdfGradient.dot(v1 - p);
		PxReal dfdw = sdfGradient.dot(v2 - p);

		PxVec3 newUVW;

		{
			newUVW = uvw;

			newUVW.x -= step * dfdu;
			newUVW.y -= step * dfdv;
			newUVW.z -= step * dfdw;

			step = step * 0.8f;

			newUVW = Gu::closestPtPointBaryTriangle(newUVW);

			p = v0 * newUVW.x + v1 * newUVW.y + v2 * newUVW.z;

			if ((uvw - newUVW).magnitudeSquared() < toleranceSq)
			{
				/*if(iter != 0)
					printf("Iter = %i\n", iter);*/
				break;
			}

			uvw = newUVW;
		}
	}

	sdfGradient = PxVolumeGrad(texture, p);

	point = p;

	dist = PxSdfSampleWithGrad(texture, p, sdfGradient, dir, tolerance);
	return dist;
}

//Returns the new number of elements in the buffer
//T should be of type PxU32 or a multiple of it (uint2, uint2, uint4)
template<typename T, PxU32 NbWarps, const PxU32 TargetCount>
__device__ PxU32 addIndexToBuffer(bool threadEmitsElement, T index, PxU32 nbElementsInBuffer, T* sharedMemoryBuffer, PxU32& offset)
{
	PxU32 total;
	PxU32 exclusiveSum = threadBlockScanExclusive<NbWarps>(threadEmitsElement, total);

	if (threadEmitsElement)
	{
		PxU32 writeOffset = nbElementsInBuffer + exclusiveSum;

		if (writeOffset < TargetCount)
		{
			sharedMemoryBuffer[writeOffset] = index;
		}
		if (writeOffset == (TargetCount - 1))
		{
			//Get the first element
			offset = ((PxU32*)(&index))[0] + 1; //The thread that added the last element to the sharedIndices writes the new offset
		}
	}

	return min(total + nbElementsInBuffer, TargetCount);
}

PX_FORCE_INLINE __device__ void getTriangleVertices(const uint4& triangle, const float4* meshVerts, const PxMeshScale& scale, const PxTransform t,
	PxU32 encodedSubIndex, PxVec3& a, PxVec3& b, PxVec3& c)
{
	a = t.transform(vertex2Shape(PxLoad3(meshVerts[triangle.x]), scale.scale, scale.rotation));
	b = t.transform(vertex2Shape(PxLoad3(meshVerts[triangle.y]), scale.scale, scale.rotation));
	c = t.transform(vertex2Shape(PxLoad3(meshVerts[triangle.z]), scale.scale, scale.rotation));

	if (encodedSubIndex > 0)
		Gu::getSubTriangleEncoded(a, b, c, encodedSubIndex);
}

PX_FORCE_INLINE __device__ void getTriangleVertices(const uint4* meshInd, const float4* meshVerts, const PxMeshScale& scale, const PxTransform t,
	PxU32 triangleIndex, PxU32 encodedSubIndex, PxVec3& a, PxVec3& b, PxVec3& c)
{
	uint4 inds = meshInd[triangleIndex];
	getTriangleVertices(inds, meshVerts, scale, t, encodedSubIndex, a, b, c);
}

PX_FORCE_INLINE __device__ void getTriangleVertices(const PxgTriangleMesh& mesh, const PxMeshScale& scale, const PxTransform t,
	PxU32 triangleIndex, PxU32 encodedSubIndex, PxVec3& a, PxVec3& b, PxVec3& c)
{
	uint4 inds = mesh.indices[triangleIndex];
	a = t.transform(vertex2Shape(PxLoad3(mesh.trimeshVerts[inds.x]), scale.scale, scale.rotation));
	b = t.transform(vertex2Shape(PxLoad3(mesh.trimeshVerts[inds.y]), scale.scale, scale.rotation));
	c = t.transform(vertex2Shape(PxLoad3(mesh.trimeshVerts[inds.z]), scale.scale, scale.rotation));

	if (encodedSubIndex > 0)
		Gu::getSubTriangleEncoded(a, b, c, encodedSubIndex);
}


PX_FORCE_INLINE __device__ PxVec3 getVertex(const float4* meshVerts, const PxMeshScale& scale, const PxTransform t, PxU32 vertexIndex)
{
	return t.transform(vertex2Shape(PxLoad3(meshVerts[vertexIndex]), scale.scale, scale.rotation));
}

PX_FORCE_INLINE __device__ PxVec3 getVertex(const PxgTriangleMesh& mesh, const PxMeshScale& scale, const PxTransform t, PxU32 vertexIndex)
{
	return t.transform(vertex2Shape(PxLoad3(mesh.trimeshVerts[vertexIndex]), scale.scale, scale.rotation));
}

//Input of surface hint is optional
template<PxU32 NbWarps, const PxU32 TargetCount>
__device__ PxU32 findInterestingTets(PxU32 nbTets, const uint4* tetIndices, const float4* tetVerts,
	const PxMeshScale& scale1, const PxReal tolerance, const SparseSDFTexture& sdfMesh1,
	const PxTransform& aToB, PxU32& startIndex, PxU32* sharedIndices, const PxU8* surfaceHint = NULL)
{
	__shared__ PxU32 offset;

	if (threadIdx.x == 0)
		offset = startIndex;

	PxU32 sharedIndicesCount = 0;
	for (PxU32 i = startIndex; (i < nbTets); )
	{
		PxU32 ind = i + threadIdx.x;
		i += blockDim.x;

		bool addToBuffer = false;
		if (ind < nbTets)
		{
			if (!surfaceHint || surfaceHint[ind])
			{
				const uint4 tet = tetIndices[ind];
				const PxVec3 a = aToB.transform(PxLoad3(tetVerts[tet.x]));
				const PxVec3 b = aToB.transform(PxLoad3(tetVerts[tet.y]));
				const PxVec3 c = aToB.transform(PxLoad3(tetVerts[tet.z]));
				const PxVec3 d = aToB.transform(PxLoad3(tetVerts[tet.w]));
				
				PxVec3 centroid = (a + b + c + d) * 0.25f;

				PxReal distInSDFSpace1 = PxSdfDistance(sdfMesh1, shape2Vertex(centroid, scale1.scale, scale1.rotation));
				//PxReal boundingSphereRadius = PxSqrt(PxMax((v0 - centroid).magnitudeSquared(), (v1 - centroid).magnitudeSquared()));

				PxReal boundingSphereRadius = PxSqrt(PxMax(PxMax((a - centroid).magnitudeSquared(), (b - centroid).magnitudeSquared()),
					PxMax((c - centroid).magnitudeSquared(), (d - centroid).magnitudeSquared())));

				boundingSphereRadius /= PxMin(scale1.scale.x, PxMin(scale1.scale.y, scale1.scale.z));
				addToBuffer = distInSDFSpace1 < (boundingSphereRadius + tolerance);				
			}
		}

		__syncthreads();
		sharedIndicesCount = addIndexToBuffer<PxU32, NbWarps, TargetCount>(addToBuffer, ind, sharedIndicesCount, sharedIndices, offset);
	}

	__syncthreads();

	if (startIndex != offset)
		startIndex = offset;
	else
		startIndex = nbTets;

	return sharedIndicesCount;
}


template<PxU32 NbWarps, const PxU32 TargetCount, bool useVertices = false>
__device__ PxU32 findInterestingTrianglesA(PxU32 mesh0NbPrimitives, const uint4* mesh0Ind, const float4* mesh0Verts,
	const PxMeshScale& scale0, const PxMeshScale& scale1, const PxReal tolerance, const SparseSDFTexture& sdfMesh1,
	const PxTransform& aToB, PxU32& startIndex, uint2* sharedIndices, PxU32 nbSubdivisionIndices, const uint2* sharedSubdivisionIndices)
{
	__shared__ PxU32 offset;

	if (threadIdx.x == 0)
		offset = startIndex;

	const PxU32 nbPrimitives = mesh0NbPrimitives; //Can be the number of triangles or the number of vertices depending on the template boolean useVertices
	PxU32 sharedIndicesCount = 0;
	for (PxU32 i = startIndex; (i < nbPrimitives || nbSubdivisionIndices>0) && sharedIndicesCount < TargetCount; )
	{
		PxU32 ind;
		PxU32 subdivisionLevel = 0;
		if (threadIdx.x < 4 * nbSubdivisionIndices)
		{
			uint2 v = sharedSubdivisionIndices[threadIdx.x >> 2];
			ind = v.x;
			subdivisionLevel = v.y;
			subdivisionLevel = Gu::elevateSubdivisionId(subdivisionLevel, threadIdx.x & 3);
		}
		else
		{
			ind = i + threadIdx.x - PxMin(TargetCount, 4 * nbSubdivisionIndices);
		}
		i += blockDim.x - PxMin(TargetCount, 4 * nbSubdivisionIndices);
		nbSubdivisionIndices = 0; //It is guaranteed that all subdivision triangles get consumed in the first run because of the limited size of sharedSubdivisionIndices


		bool addToBuffer = false;
		if (ind < nbPrimitives)
		{
			if (useVertices)
			{
				PxVec3 p = getVertex(mesh0Verts, scale0, aToB, ind);
				PxReal distInSDFSpace1 = PxSdfDistance(sdfMesh1, shape2Vertex(p, scale1.scale, scale1.rotation));
				addToBuffer = distInSDFSpace1 < tolerance;
			}
			else 
			{
				PxVec3 v0, v1, v2;
				getTriangleVertices(mesh0Ind, mesh0Verts, scale0, aToB, ind, subdivisionLevel, v0, v1, v2);

				const PxReal third = 1.f / 3.f;
				PxVec3 centroid = (v0 + v1 + v2) * third;

				PxReal distInSDFSpace1 = PxSdfDistance(sdfMesh1, shape2Vertex(centroid, scale1.scale, scale1.rotation));
				PxReal boundingSphereRadius = PxSqrt(PxMax((v0 - centroid).magnitudeSquared(), PxMax((v1 - centroid).magnitudeSquared(),
					(v2 - centroid).magnitudeSquared())));
				boundingSphereRadius /= PxMin(scale1.scale.x, PxMin(scale1.scale.y, scale1.scale.z));
				addToBuffer = distInSDFSpace1 < (boundingSphereRadius + tolerance);
			}
		}

		__syncthreads();
		sharedIndicesCount = addIndexToBuffer<uint2, NbWarps, TargetCount>(addToBuffer, make_uint2(ind, subdivisionLevel), sharedIndicesCount, sharedIndices, offset);
	}

	__syncthreads();

	if (startIndex != offset)
		startIndex = offset;
	else
		startIndex = nbPrimitives;

	return sharedIndicesCount;
}


template<PxU32 NbWarps, const PxU32 TargetCount>
__device__ PxU32 findInterestingVertices(PxU32 nbVertices, const float4* positions,
	const PxMeshScale& scale1, const PxReal tolerance, const SparseSDFTexture& sdfMesh1,
	const PxTransform& aToB, PxU32& startIndex, PxU32* sharedIndices)
{
	__shared__ PxU32 offset;

	if (threadIdx.x == 0)
		offset = startIndex;

	PxU32 sharedIndicesCount = 0;
	for (PxU32 i = startIndex; (i < nbVertices); )
	{
		PxU32 ind = i + threadIdx.x;
		i += blockDim.x;

		bool addToBuffer = false;
		if (ind < nbVertices)
		{
			PxVec3 v = aToB.transform(PxLoad3(positions[ind]));
			PxReal distInSDFSpace1 = PxSdfDistance(sdfMesh1, shape2Vertex(v, scale1.scale, scale1.rotation));	
			addToBuffer = distInSDFSpace1 < tolerance;			
		}

		__syncthreads();
		sharedIndicesCount = addIndexToBuffer<PxU32, NbWarps, TargetCount>(addToBuffer, ind, sharedIndicesCount, sharedIndices, offset);
	}

	__syncthreads();

	if (startIndex != offset)
		startIndex = offset;
	else
		startIndex = nbVertices;

	return sharedIndicesCount;
}


PX_FORCE_INLINE __device__ PxReal triangleRadiusSquared(const PxVec3& v0, const PxVec3& v1, const PxVec3& v2)
{
	const PxReal third = 1.0f / 3.0f;
	PxVec3 center = third * (v0 + v1 + v2);
	return PxMax(PxMax((v1 - v0).magnitudeSquared(), (v2 - v1).magnitudeSquared()), (v0 - v2).magnitudeSquared());
}

PX_FORCE_INLINE __device__ PxReal sdfRadiusSquared(SparseSDFTexture& sdfTexture)
{
	return (sdfTexture.sdfBoxHigher - sdfTexture.sdfBoxLower).magnitudeSquared();
}

//TargetCount is actually the maximal size of sharedSubdivisionIndices. The naming could be improved...
template<PxU32 NbWarps, const PxU32 TargetCount>
__device__ PxU32 addToRefinementBuffer(bool addToBuffer, PxU32 ind, PxU32 subInd, PxU32 nbElementsInBuffer, uint2* sharedMemoryBuffer, PxU32 maxRefinementLevel)
{
	PxU32 subdivisionLevel, subTriangleIndex;
	Gu::decodeSubdivisionId(subInd, subdivisionLevel, subTriangleIndex);
	addToBuffer = addToBuffer && subdivisionLevel < maxRefinementLevel; //Make sure a upper limit of refinement is never exceeded

	PxU32 total;
	PxU32 exclusiveSum = threadBlockScanExclusive<NbWarps>(addToBuffer, total);

	if (addToBuffer)
	{
		PxU32 writeOffset = nbElementsInBuffer + exclusiveSum;

		if (writeOffset < TargetCount)
		{
			sharedMemoryBuffer[writeOffset] = make_uint2(ind, subInd);
		}
	}

	return min(total + nbElementsInBuffer, TargetCount);
}
}
#endif
