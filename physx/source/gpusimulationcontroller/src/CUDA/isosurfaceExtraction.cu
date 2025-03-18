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

#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "stdio.h"
#include "assert.h"

#include "atomic.cuh"

#include "marchingCubesTables.cuh"

#include "PxgIsosurfaceData.h"

#include "denseGridStandalone.cuh"
#include "sparseGridStandalone.cuh"

#define ENABLE_KERNEL_LAUNCH_ERROR_CHECK 0

extern "C" __host__ void initIsosurfaceExtractionKernels0() {}

__inline__ __device__ void atomicAdd3(PxVec4* p, PxVec3 q)
{
	PxRedAddGlobal(&p->x, q.x);
	PxRedAddGlobal(&p->y, q.y);
	PxRedAddGlobal(&p->z, q.z);
}

__inline__ __device__ PxReal* getDensity(PxIsosurfaceExtractionData& data)
{
	return data.buffer[data.swapState];
}

__inline__ __device__ PxU32* getFirstCellTriId(PxIsosurfaceExtractionData& data)
{
	return reinterpret_cast<PxU32*>(data.buffer[1 - data.swapState]);
}

__inline__ __device__ PxReal dotXYZ(const float4& l, const PxVec3& r)
{
	return l.x*r.x + l.y*r.y + l.z*r.z;
}
PX_FORCE_INLINE __device__ PxReal dotProduct(const float4& l, const float4& r)
{
	return l.x*r.x + l.y*r.y + l.z*r.z + l.w*r.w;
}

__inline__ __device__ PxReal* getDensity(PxSparseIsosurfaceExtractionData& data)
{
	return data.buffer[data.swapState];
}

__inline__ __device__ PxU32* getFirstCellTriId(PxSparseIsosurfaceExtractionData& data)
{
	return reinterpret_cast<PxU32*>(data.buffer[1 - data.swapState]);
}


PX_FORCE_INLINE __device__ float4 scaleW(float4 v, PxReal s)
{
	v.w *= s;
	return v;
}

template<typename T>
__device__ void computeParticleDensityUsingSDF(PxVec4* PX_RESTRICT deviceParticlePos, int numParticles, PxU32* PX_RESTRICT phases, PxU32 validPhaseMask, T& data, PxU32* PX_RESTRICT activeIndices,
	float4* PX_RESTRICT anisotropy1, float4* PX_RESTRICT anisotropy2, float4* PX_RESTRICT anisotropy3, PxReal anisotropyFactor)
{
	PxI32 pNr = blockIdx.x * blockDim.x + threadIdx.x;
	if (pNr >= numParticles)
		return;

	if (activeIndices)
		pNr = activeIndices[pNr];

	if (phases && !(phases[pNr] & validPhaseMask))
		return;

	PxVec3 p = deviceParticlePos[pNr].getXYZ();
	int4 xyz = getCellIndexFromParticleAndTransformToLocalCoordinates(data.mGrid, p);

	if (outOfBounds(data.mGrid, xyz))
		return;

	anisotropyFactor = 1.0f / anisotropyFactor;
	float4 e1 = anisotropy1 ? scaleW(anisotropy1[pNr], anisotropyFactor) : make_float4(1, 0, 0, 1);
	float4 e2 = anisotropy2 ? scaleW(anisotropy2[pNr], anisotropyFactor) : make_float4(0, 1, 0, 1);
	float4 e3 = anisotropy3 ? scaleW(anisotropy3[pNr], anisotropyFactor) : make_float4(0, 0, 1, 1);

	float4 xTransformInv = make_float4(e1.x / e1.w, e1.y / e1.w, e1.z / e1.w, 0.f);
	float4 yTransformInv = make_float4(e2.x / e2.w, e2.y / e2.w, e2.z / e2.w, 0.f);
	float4 zTransformInv = make_float4(e3.x / e3.w, e3.y / e3.w, e3.z / e3.w, 0.f);

	xTransformInv.w = -dotXYZ(xTransformInv, p);
	yTransformInv.w = -dotXYZ(yTransformInv, p);
	zTransformInv.w = -dotXYZ(zTransformInv, p);

	const PxReal dx = data.getSpacing();
	const PxReal invDx = 1.0f / dx;

	const PxReal scale = 1.0f; // powf(e1.w*e2.w*e3.w, 1.0f / 3.0f);

	const PxReal kernelSize = data.kernelSize;

	//lower and upper allow to limit the maximal neighborhood on the grid to have an upper bound in case of very big kernelSizes
	const PxI32 lower = 2; //1;
	const PxI32 upper = lower + 1;

	const PxI32 lowerX = PxMax(xyz.x - lower, (PxI32)PxCeil((p.x - kernelSize) * invDx));
	const PxI32 upperX = PxMin(xyz.x + upper, (PxI32)PxFloor((p.x + kernelSize) * invDx));
	const PxI32 lowerY = PxMax(xyz.y - lower, (PxI32)PxCeil((p.y - kernelSize) * invDx));
	const PxI32 upperY = PxMin(xyz.y + upper, (PxI32)PxFloor((p.y + kernelSize) * invDx));
	const PxI32 lowerZ = PxMax(xyz.z - lower, (PxI32)PxCeil((p.z - kernelSize) * invDx));
	const PxI32 upperZ = PxMin(xyz.z + upper, (PxI32)PxFloor((p.z + kernelSize) * invDx));

	/*if (pNr == 0)
		printf("xyz: %i, %i, %i, %f, %f\n", upperX - lowerX + 1, upperY - lowerY + 1, upperZ - lowerZ + 1, kernelSize, dx);*/

	PxReal* density = getDensity(data);

	for (PxI32 zi = lowerZ; zi <= upperZ; zi++)
	{
		for (PxI32 yi = lowerY; yi <= upperY; yi++)
		{
			for (PxI32 xi = lowerX; xi <= upperX; xi++)
			{
				float4 worldPos = make_float4(xi * dx, yi * dx, zi * dx, 1.0f);

				PxVec3 localPos(
					dotProduct(xTransformInv, worldPos),
					dotProduct(yTransformInv, worldPos),
					dotProduct(zTransformInv, worldPos)
				);
				PxReal distance = scale * localPos.magnitude();

				PxReal v = distance - kernelSize;

				if (v < 0.0f)
				{
					PxI32 cellNr = getCellIndexSafe(data.mGrid, xyz, xi - xyz.x, yi - xyz.y, zi - xyz.z);
					if (cellNr == EMPTY_SUBGRID)
						continue;
					if (v < density[cellNr])
						AtomicMin(&density[cellNr], v);
				}
			}
		}
	}
}

extern "C" __global__ void iso_ComputeParticleDensityUsingSDF(PxVec4* PX_RESTRICT deviceParticlePos, int numParticles, PxU32* PX_RESTRICT phases, PxU32 validPhaseMask, PxIsosurfaceExtractionData data,
	PxU32* PX_RESTRICT activeIndices, float4* PX_RESTRICT anisotropy1, float4* PX_RESTRICT anisotropy2, float4* PX_RESTRICT anisotropy3, PxReal anisotropyFactor)
{
	computeParticleDensityUsingSDF(deviceParticlePos, numParticles, phases, validPhaseMask, data, activeIndices,
		anisotropy1, anisotropy2, anisotropy3, anisotropyFactor);
}

extern "C" __global__ void iso_ComputeParticleDensityUsingSDFSparse(PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxSparseIsosurfaceExtractionData data,
	PxU32* PX_RESTRICT activeIndices, float4* anisotropy1, float4* anisotropy2, float4* anisotropy3, PxReal anisotropyFactor)
{
	computeParticleDensityUsingSDF(deviceParticlePos, numParticles, phases, validPhaseMask, data, activeIndices,
		anisotropy1, anisotropy2, anisotropy3, anisotropyFactor);
}


template<typename T>
__device__ void computeParticleDensity(PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, T& data)
{
	PxI32 pNr = blockIdx.x * blockDim.x + threadIdx.x;
	if (pNr >= numParticles)
		return;

	if (phases && !(phases[pNr] & validPhaseMask))
		return;

	PxVec3 p = deviceParticlePos[pNr].getXYZ();
	int4 xyz = getCellIndexFromParticleAndTransformToLocalCoordinates(data.mGrid, p);

	if (outOfBounds(data.mGrid, xyz))
		return;

	PxReal dx = data.getSpacing();
	PxReal invDx = 1.0f / dx;

	PxReal h = data.kernelSize;
	PxReal h2 = h * h;
	PxReal kernelScale = 315.0f / (64.0f * 3.14159265f * h2 * h2 * h2 * h2 * h) / data.restDensity;

	//lower and upper allow to limit the maximal neighborhood on the grid to have an upper bound in case of very big kernelSizes
	const PxI32 lower = 2; //1;
	const PxI32 upper = lower + 1;

	const PxI32 lowerX = PxMax(xyz.x - lower, (PxI32)PxFloor((p.x - h) * invDx));
	const PxI32 upperX = PxMin(xyz.x + upper, (PxI32)PxCeil((p.x + h) * invDx));
	const PxI32 lowerY = PxMax(xyz.y - lower, (PxI32)PxFloor((p.y - h) * invDx));
	const PxI32 upperY = PxMin(xyz.y + upper, (PxI32)PxCeil((p.y + h) * invDx));
	const PxI32 lowerZ = PxMax(xyz.z - lower, (PxI32)PxFloor((p.z - h) * invDx));
	const PxI32 upperZ = PxMin(xyz.z + upper, (PxI32)PxCeil((p.z + h) * invDx));

	PxReal* density = getDensity(data);
	for (PxI32 xi = lowerX; xi <= upperX; xi++)
	{
		for (PxI32 yi = lowerY; yi <= upperY; yi++)
		{
			for (PxI32 zi = lowerZ; zi <= upperZ; zi++)
			{
				PxVec3 cellPos = PxVec3(xi * dx, yi * dx, zi * dx);
				PxVec3 r = p - cellPos;
				PxReal r2 = r.magnitudeSquared();
				if (r2 < h2)
				{
					PxI32 cellNr = getCellIndexSafe(data.mGrid, xyz, xi - xyz.x, yi - xyz.y, zi - xyz.z); // getCellNr(data, xi, yi, zi);
					if (cellNr == EMPTY_SUBGRID)
						continue;

					PxReal w = (h2 - r2);
					w = kernelScale * w * w * w;

					PxRedAddGlobal(&density[cellNr], w);
				}
			}
		}
	}
}

extern "C" __global__ void iso_ComputeParticleDensity(PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxIsosurfaceExtractionData data)
{
	computeParticleDensity(deviceParticlePos, numParticles, phases, validPhaseMask, data);
}

extern "C" __global__ void iso_ComputeParticleDensitySparse(PxVec4* deviceParticlePos, int numParticles, PxU32* phases, PxU32 validPhaseMask, PxSparseIsosurfaceExtractionData data)
{
	computeParticleDensity(deviceParticlePos, numParticles, phases, validPhaseMask, data);
}

template<typename T>
__device__ void countCellVerts(T& data)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	data.firstCellVert[threadIndex] = 0;

	if (outOfBounds(data.mGrid, xyz))
		return;

	PxReal* density = getDensity(data);
	PxReal d0 = getGridValue(data.mGrid, density, xyz, 0, 0, 0);

	PxReal ds[3];
	ds[0] = getGridValue(data.mGrid, density, xyz, 1, 0, 0);
	ds[1] = getGridValue(data.mGrid, density, xyz, 0, 1, 0);
	ds[2] = getGridValue(data.mGrid, density, xyz, 0, 0, 1);
	int num = 0;
#pragma unroll
	for (int dim = 0; dim < 3; dim++)
	{
		PxReal d = ds[dim];
		if ((d0 <= data.threshold && d >= data.threshold) || (d <= data.threshold && d0 >= data.threshold))
			num++;
	}

	data.firstCellVert[threadIndex] = num;
}

extern "C" __global__ void iso_CountCellVerts(PxIsosurfaceExtractionData data)
{
	countCellVerts(data);
}

extern "C" __global__ void iso_CountCellVertsSparse(PxSparseIsosurfaceExtractionData data)
{
	countCellVerts(data);
}

template<typename T>
__device__ void countCellVertsDC(T& data)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	data.firstCellVert[threadIndex] = 0;

	if (outOfBounds(data.mGrid, xyz))
		return;

	PxReal* density = getDensity(data);

	/*PxU32 aboveThresholdCounter = 0;
	PxU32 belowThresholdCounter = 0;
	PxU32 counter = 0;
	for (int x = 0; x <= 1; ++x) for (int y = 0; y <= 1; ++y) for (int z = 0; z <= 1; ++z)
	{
		PxReal d = getGridValue(data, density, xyz, x, y, z);
		if (d >= data.threshold)
			aboveThresholdCounter++;
		if (d <= data.threshold)
			belowThresholdCounter++;
		if (aboveThresholdCounter > 0 && belowThresholdCounter > 0)
		{
			counter = 1;
			break;
		}
	}*/

	PxReal corners[2][2][2];
	for (int x = 0; x <= 1; ++x) for (int y = 0; y <= 1; ++y) for (int z = 0; z <= 1; ++z)
	{
		corners[x][y][z] = getGridValue(data.mGrid, density, xyz, x, y, z);
	}

	PxU32 counter = 0;
	for (int a = 0; a <= 1; ++a) for (int b = 0; b <= 1; ++b)
	{
		PxReal p = corners[a][b][0];
		PxReal q = corners[a][b][1];

		if ((p <= data.threshold && q >= data.threshold) || (q <= data.threshold && p >= data.threshold))
		{
			++counter;
		}
	}
	for (int a = 0; a <= 1; ++a) for (int b = 0; b <= 1; ++b)
	{
		PxReal p = corners[b][0][a];
		PxReal q = corners[b][1][a];

		if ((p <= data.threshold && q >= data.threshold) || (q <= data.threshold && p >= data.threshold))
		{
			++counter;
		}
	}
	for (int a = 0; a <= 1; ++a) for (int b = 0; b <= 1; ++b)
	{
		PxReal p = corners[0][a][b];
		PxReal q = corners[1][a][b];

		if ((p <= data.threshold && q >= data.threshold) || (q <= data.threshold && p >= data.threshold))
		{
			++counter;
		}
	}

	if (counter > 0)
		counter = 1;

	data.firstCellVert[threadIndex] = counter;
}

extern "C" __global__ void iso_CountCellVertsDC(PxIsosurfaceExtractionData data)
{
	countCellVertsDC(data);
}

extern "C" __global__ void iso_CountCellVertsDCSparse(PxSparseIsosurfaceExtractionData data)
{
	countCellVertsDC(data);
}

template<typename T>
__device__ void createVerts(T& data)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[0] = PxMin(data.maxVerts, data.firstCellVert[threadIndex]);

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);
	if (outOfBounds(data.mGrid, xyz))
		return;

	PxU32 first = data.firstCellVert[threadIndex];

	PxVec3 p = getLocation(data.mGrid, xyz);

	PxReal* density = getDensity(data);
	PxReal d0 = getGridValue(data.mGrid, density, xyz, 0, 0, 0);
	PxReal ds[3];
	ds[0] = getGridValue(data.mGrid, density, xyz, 1, 0, 0);
	ds[1] = getGridValue(data.mGrid, density, xyz, 0, 1, 0);
	ds[2] = getGridValue(data.mGrid, density, xyz, 0, 0, 1);


	PxU32 encoded = first;
#pragma unroll
	for (int dim = 0; dim < 3; dim++)
	{
		PxReal d = ds[dim];
		if ((d0 <= data.threshold && d >= data.threshold) || (d <= data.threshold && d0 >= data.threshold))
		{
			PxReal t = (d != d0) ? PxClamp((data.threshold - d0) / (d - d0), 0.0f, 1.0f) : 0.5f;
			PxU32 id = first++;

			encoded |= 1 << (29 + dim);

			PxVec3 off(0.0f);
			off[dim] = t * data.getSpacing();

			if (id < data.maxVerts)
				data.verts[id] = PxVec4(p + off, 0.0f);
		}
	}
	data.firstCellVert[threadIndex] = encoded;

	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[0] = PxMin(data.maxVerts, first); //This allows to obtain the total number of vertices in the last array element	
}

extern "C" __global__ void iso_CreateVerts(PxIsosurfaceExtractionData data)
{
	createVerts(data);
}

extern "C" __global__ void iso_CreateVertsSparse(PxSparseIsosurfaceExtractionData data)
{
	createVerts(data);
}

template<typename T>
__device__ void createVertsDC(T& data)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[0] = PxMin(data.maxVerts, data.firstCellVert[threadIndex]);

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);
	if (outOfBounds(data.mGrid, xyz))
		return;

	PxU32 id = data.firstCellVert[threadIndex];
	if (!isLastCell(data.mGrid, threadIndex) && id == data.firstCellVert[threadIndex + 1])
		return;

	PxVec3 p = getLocation(data.mGrid, xyz);

	PxReal* density = getDensity(data);


	/*PxReal counter = 0;
	PxVec3 sum(0.0f);

	for (int x = 0; x <= 1; ++x) for (int y = 0; y <= 1; ++y) for (int z = 0; z <= 1; ++z)
	{
		PxReal d = getGridValue(data, density, xyz, x, y, z) - data.threshold;

		PxReal inverseDistance = 1.0f / PxMax(1e-6f, PxAbs(d));
		counter += inverseDistance;
		sum += inverseDistance * PxVec3(x, y, z);
	}
	p = p + sum * (data.getSpacing() / counter);*/

	PxReal corners[2][2][2];
	for (int x = 0; x <= 1; ++x) for (int y = 0; y <= 1; ++y) for (int z = 0; z <= 1; ++z)
	{
		corners[x][y][z] = getGridValue(data.mGrid, density, xyz, x, y, z);
	}

	PxU32 counter = 0;
	PxVec3 sum(0.0f);
	for (int a = 0; a <= 1; ++a) for (int b = 0; b <= 1; ++b)
	{
		PxReal p = corners[a][b][0];
		PxReal q = corners[a][b][1];

		if ((p <= data.threshold && q >= data.threshold) || (q <= data.threshold && p >= data.threshold))
		{
			PxReal t = (q != p) ? PxClamp((data.threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
			sum += PxVec3(a, b, t);
			++counter;
		}
	}
	for (int a = 0; a <= 1; ++a) for (int b = 0; b <= 1; ++b)
	{
		PxReal p = corners[b][0][a];
		PxReal q = corners[b][1][a];

		if ((p <= data.threshold && q >= data.threshold) || (q <= data.threshold && p >= data.threshold))
		{
			PxReal t = (q != p) ? PxClamp((data.threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
			sum += PxVec3(b, t, a);
			++counter;
		}
	}
	for (int a = 0; a <= 1; ++a) for (int b = 0; b <= 1; ++b)
	{
		PxReal p = corners[0][a][b];
		PxReal q = corners[1][a][b];

		if ((p <= data.threshold && q >= data.threshold) || (q <= data.threshold && p >= data.threshold))
		{
			PxReal t = (q != p) ? PxClamp((data.threshold - p) / (q - p), 0.0f, 1.0f) : 0.5f;
			sum += PxVec3(t, a, b);
			++counter;
		}
	}
	p = p + sum * (data.getSpacing() / counter);

	if (counter > 0)
		data.verts[id] = PxVec4(p, 0.0f);

	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[0] = PxMin(data.maxVerts, id + 1); //This allows to obtain the total number of vertices in the last array element	
}

extern "C" __global__ void iso_CreateVertsDC(PxIsosurfaceExtractionData data)
{
	createVertsDC(data);
}

extern "C" __global__ void iso_CreateVertsDCSparse(PxSparseIsosurfaceExtractionData data)
{
	createVertsDC(data);
}

__device__ PxU32 getDecodedId(PxU32 encoded, PxU32 dim)
{
	if (!(encoded & (1 << (29 + dim))))
		return 0xFFFFFFFF;

	PxU32 id = encoded & 0x1FFFFFFF;
	if (dim > 0 && (encoded & (1 << (29 + 0))))
		id++;
	if (dim > 1 && (encoded & (1 << (29 + 1))))
		id++;
	return id;
}

template<typename T>
PX_FORCE_INLINE __device__ bool constructTriangle(T& data, int4& xyz, int i, PxU32 firstIn, PxU32 firstOut, int* buffer = NULL)
{
#pragma unroll
	for (int j = 0; j < 3; ++j)
	{
		int eid = marchingCubesIds[firstIn + i + j];
		int edgeNr = marchingCubesEdgeLocations[eid][3];
		const PxU32 nr = getCellIndex(data.mGrid, xyz, marchingCubesEdgeLocations[eid][0], marchingCubesEdgeLocations[eid][1], marchingCubesEdgeLocations[eid][2], false);
		if (nr == EMPTY_SUBGRID)
			return false;
		PxU32 id = getDecodedId(data.firstCellVert[nr], edgeNr);
		if (id >= data.maxVerts || firstOut + j >= data.maxTriIds)
			return false;
		if (buffer)
			buffer[j] = id;
	}
	return true;
}

template<typename T>
__device__ void countTriIds(T& data)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	PxU32* firstCellTriId = getFirstCellTriId(data);
	firstCellTriId[threadIndex] = 0;
	if (outOfBounds(data.mGrid, xyz))
		return;

	PxReal* density = getDensity(data);
	int code = 0;
#pragma unroll
	for (int i = 0; i < 8; i++)
	{
		if (getGridValue(data.mGrid, density, xyz, marchingCubeCorners[i][0], marchingCubeCorners[i][1], marchingCubeCorners[i][2]) >= data.threshold)
			code |= (1 << i);
	}

	int firstIn = firstMarchingCubesId[code];
	int num = firstMarchingCubesId[code + 1] - firstIn;
	PxI32 counter = 0;
	for (int i = 0; i < num; i += 3)
	{
		if (!constructTriangle(data, xyz, i, firstIn, 0))
			continue;
		counter += 3;
	}

	firstCellTriId[threadIndex] = counter;
}

extern "C" __global__ void iso_CountTriIds(PxIsosurfaceExtractionData data)
{
	countTriIds(data);
}

extern "C" __global__ void iso_CountTriIdsSparse(PxSparseIsosurfaceExtractionData data)
{
	countTriIds(data);
}

template<typename T>
__device__ void countTriIdsDC(T& data)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	PxU32* firstCellTriId = getFirstCellTriId(data);
	firstCellTriId[threadIndex] = 0;

	if (outOfBounds(data.mGrid, xyz))
		return;

	PxReal* density = getDensity(data);
	PxReal d0 = getGridValue(data.mGrid, density, xyz, 0, 0, 0);

	PxReal ds[3];
	ds[0] = getGridValue(data.mGrid, density, xyz, 1, 0, 0);
	ds[1] = getGridValue(data.mGrid, density, xyz, 0, 1, 0);
	ds[2] = getGridValue(data.mGrid, density, xyz, 0, 0, 1);
	int num = 0;
#pragma unroll
	for (int dim = 0; dim < 3; dim++)
	{
		PxReal d = ds[dim];
		if ((d0 <= data.threshold && d >= data.threshold) || (d <= data.threshold && d0 >= data.threshold))
			num++;
	}

	firstCellTriId[threadIndex] = 2 * 3 * num;
}

extern "C" __global__ void iso_CountTriIdsDC(PxIsosurfaceExtractionData data)
{
	countTriIdsDC(data);
}

extern "C" __global__ void iso_CountTriIdsDCSparse(PxSparseIsosurfaceExtractionData data)
{
	countTriIdsDC(data);
}

template<typename T>
__device__ void createTriIds(T& data, bool flipTriangleOrientation)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	const PxU32* firstCellTriId = getFirstCellTriId(data);
	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[1] = PxMin(data.maxTriIds, firstCellTriId[threadIndex]);

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);
	if (outOfBounds(data.mGrid, xyz))
		return;

	PxU32 firstOut = firstCellTriId[threadIndex];
	if (!isLastCell(data.mGrid, threadIndex) && firstOut == firstCellTriId[threadIndex + 1])
		return; //This means no slice reserved for this instance, this is known because of the already completed counting kernel

	const PxReal* density = getDensity(data);
	int code = 0;
#pragma unroll
	for (int i = 0; i < 8; i++)
	{
		if (getGridValue(data.mGrid, density, xyz, marchingCubeCorners[i][0], marchingCubeCorners[i][1], marchingCubeCorners[i][2]) >= data.threshold)
			code |= (1 << i);
	}

	int firstIn = firstMarchingCubesId[code];
	int num = firstMarchingCubesId[code + 1] - firstIn;
	int buffer[3];
	for (int i = 0; i < num; i += 3)
	{
		if (!constructTriangle(data, xyz, i, firstIn, firstOut, buffer))
			continue;
		if (flipTriangleOrientation)
		{
			for (int j = 2; j >= 0; --j)
				data.triIds[firstOut++] = buffer[j];
		}
		else
		{
			for (int j = 0; j < 3; ++j)
				data.triIds[firstOut++] = buffer[j];
		}
	}

	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[1] = PxMin(data.maxTriIds, firstOut); //This allows to obtain the total number of indices in the last array element	
}

extern "C" __global__ void iso_CreateTriIds(PxIsosurfaceExtractionData data, bool flipTriangleOrientation)
{
	createTriIds(data, flipTriangleOrientation);
}

extern "C" __global__ void iso_CreateTriIdsSparse(PxSparseIsosurfaceExtractionData data, bool flipTriangleOrientation)
{
	createTriIds(data, flipTriangleOrientation);
}


__constant__ int offsets[3][3][3] = { { {0,-1,0}, {0,-1,-1},{0,0,-1} },
									{ {0,0,-1}, {-1,0,-1},{-1,0,0} } ,
									{ {-1,0,0}, {-1,-1,0},{0,-1,0} } };

__constant__ int projections[3][2] = { {1, 2}, {2, 0}, {0, 1} };

__device__ float directionSign(int principalDirection, const PxVec3& start, const PxVec3& middle, const PxVec3& end)
{
	PxReal a0 = middle[projections[principalDirection][0]] - start[projections[principalDirection][0]];
	PxReal a1 = middle[projections[principalDirection][1]] - start[projections[principalDirection][1]];

	PxReal b0 = end[projections[principalDirection][0]] - middle[projections[principalDirection][0]];
	PxReal b1 = end[projections[principalDirection][1]] - middle[projections[principalDirection][1]];

	return a0 * b1 - a1 * b0;
}

__device__ int indexOfConcaveCorner(int principalDirection, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& d)
{
	float minimum = 0;
	int result = -1;
	float s = directionSign(principalDirection, a, b, c);
	if (s <= minimum)
	{
		minimum = s;
		result = 1;
	}
	s = directionSign(principalDirection, b, c, d);
	if (s <= minimum)
	{
		minimum = s;
		result = 2;
	}
	s = directionSign(principalDirection, c, d, a);
	if (s <= minimum)
	{
		minimum = s;
		result = 3;
	}
	s = directionSign(principalDirection, d, a, b);
	if (s <= minimum)
	{
		minimum = s;
		result = 0;
	}
	return result;
}

template<typename T>
__device__ void createTriIdsDC(T& data, bool flipTriangleOrientation)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	PxU32* firstCellTriId = getFirstCellTriId(data);
	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[1] = PxMin(data.maxTriIds, firstCellTriId[threadIndex]);

	if (outOfBounds(data.mGrid, xyz))
		return;

	PxReal* density = getDensity(data);
	PxReal d0 = getGridValue(data.mGrid, density, xyz, 0, 0, 0);

	PxU32 firstOut = firstCellTriId[threadIndex];

	PxReal ds[3];
	ds[0] = getGridValue(data.mGrid, density, xyz, 1, 0, 0);
	ds[1] = getGridValue(data.mGrid, density, xyz, 0, 1, 0);
	ds[2] = getGridValue(data.mGrid, density, xyz, 0, 0, 1);
	int buffer[4];
	buffer[0] = data.firstCellVert[threadIndex];
	PxVec3 v0 = data.verts[buffer[0]].getXYZ();
#pragma unroll
	for (int dim = 0; dim < 3; dim++)
	{
		PxReal d = ds[dim];
		bool b1 = d0 <= data.threshold && d >= data.threshold;
		bool b2 = d <= data.threshold && d0 >= data.threshold;
		if (b1 || b2)
		{
			bool flip = flipTriangleOrientation == b1;

			bool skip = false;
#pragma unroll
			for (int i = 0; i < 3; ++i)
			{
				int id = getCellIndexSafe(data.mGrid, xyz, offsets[dim][i][0], offsets[dim][i][1], offsets[dim][i][2], false);
				if (id == EMPTY_SUBGRID)
					skip = true;
				buffer[i + 1] = data.firstCellVert[id];
			}
			if (skip)
				continue;

#pragma unroll
			for (int i = 0; i < 4; ++i)
				if (buffer[i] >= data.maxVerts)
					continue;


			//PxReal d02 = (data.verts[buffer[2]].getXYZ() - v0).magnitudeSquared();
			//PxReal d13 = (data.verts[buffer[3]].getXYZ() - data.verts[buffer[1]].getXYZ()).magnitudeSquared();

			int shift = PxMax(0, indexOfConcaveCorner(dim, v0, data.verts[buffer[1]].getXYZ(), data.verts[buffer[2]].getXYZ(), data.verts[buffer[3]].getXYZ())) % 2;
			//int shift = d02 < d13 ? 0 : 1;//TODO: select better diagonal
			//shift = 1 - shift;

			//Split the quad into two triangles
			for (int i = 0; i < 2; ++i)
			{
				if (firstOut + 3 >= data.maxTriIds)
					break;

				data.triIds[firstOut++] = buffer[shift];
				if (flip)
				{
					for (int j = 2; j >= 1; --j)
						data.triIds[firstOut++] = buffer[(i + j + shift) % 4];
				}
				else
				{
					for (int j = 1; j < 3; ++j)
						data.triIds[firstOut++] = buffer[(i + j + shift) % 4];
				}
			}
		}
	}

	if (isLastCell(data.mGrid, threadIndex))
		data.numVerticesNumIndices[1] = PxMin(data.maxTriIds, firstOut); //This allows to obtain the total number of indices in the last array element	
}

extern "C" __global__ void iso_CreateTriIdsDC(PxIsosurfaceExtractionData data, bool flipTriangleOrientation)
{
	createTriIdsDC(data, flipTriangleOrientation);
}

extern "C" __global__ void iso_CreateTriIdsDCSparse(PxSparseIsosurfaceExtractionData data, bool flipTriangleOrientation)
{
	createTriIdsDC(data, flipTriangleOrientation);
}

//https://stackoverflow.com/questions/3380628/fast-arc-cos-algorithm
PX_FORCE_INLINE __device__ PxReal approxAcos(PxReal x)
{
	return PxMax(0.0f, (-0.69813170079773212f * x * x - 0.87266462599716477f) * x + 1.5707963267948966f);
	//return (-0.69813170079773212f * x * x - 0.87266462599716477f) * x + 1.5707963267948966f;
}

PX_FORCE_INLINE __device__ PxReal angle(const PxVec3 v1, const PxVec3 v2)
{
	return approxAcos(v1.dot(v2) * __frsqrt_rn(v1.magnitudeSquared() * v2.magnitudeSquared()));
}

PX_FORCE_INLINE __device__ PxReal areaTimes2(const PxVec3 v1, const PxVec3 v2)
{
	return v1.cross(v2).magnitude();
}

__device__ void smoothVerts(const PxVec4* vertices, PxVec4* output, const PxU32* triIds, const PxU32 numTriIds)
{
	const PxU32 numTriangles = numTriIds / 3;

	for (PxI32 triNr = blockIdx.x * blockDim.x + threadIdx.x; triNr < numTriangles; triNr += blockDim.x * gridDim.x)
	{
		for (int i = 0; i < 3; i++)
		{
			int id0 = triIds[3 * triNr + i];
			int id1 = triIds[3 * triNr + (i + 1) % 3];
			int id2 = triIds[3 * triNr + (i + 2) % 3];

			PxVec3 p0 = vertices[id0].getXYZ();
			PxVec3 p1 = vertices[id1].getXYZ();
			PxVec3 p2 = vertices[id2].getXYZ();
			PxVec3 p01 = p1 - p0;
			PxVec3 p02 = p2 - p0;

			PxReal a0 = angle(p01, p02);
			//PxReal a0 = ((0.5f * (p1 + p2)) - p0).magnitude();
			//PxReal a0 = PxSqrt(areaTimes2(p01, p02));
			//PxReal a0 = 1.0f;			

			PxRedAddGlobal(&output[id0].w, a0);
			atomicAdd3(&output[id0], a0 * (0.5f * (p1 + p2)));
		}
	}
}

extern "C" __global__ void iso_SmoothVerts(const PxVec4* PX_RESTRICT vertices, PxVec4* PX_RESTRICT output, const PxU32* PX_RESTRICT triIds, const PxU32* PX_RESTRICT numTriIds)
{
	smoothVerts(vertices, output, triIds, *numTriIds);
}

__device__ void averageVerts(PxVec4* PX_RESTRICT vertices, PxVec4* PX_RESTRICT output, const PxU32 length, PxReal blendWeight = 1.0f)
{
	const PxU32 numVertices = length;

	for (PxI32 vNr = blockIdx.x * blockDim.x + threadIdx.x; vNr < numVertices; vNr += blockDim.x * gridDim.x)
	{
		PxVec4 v = vertices[vNr];
		if (v.w > 0)
		{
			PxVec3 x = output[vNr].getXYZ();
			output[vNr] = PxVec4((1.0f - blendWeight) * x + blendWeight * (v.getXYZ() / v.w), 0.0f);
		}
		//Clear the buffer array to use it for followingsmoothing passes
		vertices[vNr] = PxVec4(0.0f);
	}
}

extern "C" __global__ void iso_AverageVerts(PxVec4* PX_RESTRICT vertices, PxVec4* PX_RESTRICT output, const PxU32* PX_RESTRICT length, PxReal blendWeight)
{
	averageVerts(vertices, output, *length, blendWeight);
}

__device__ PxVec3 normalize(PxVec3 v)
{
	const float m = v.magnitude();
	if (m > 0.0f)
		v /= m;
	return v;
}

template<typename T>
__device__ void smoothNormals(T& data)
{
	const PxU32 numTriangles = data.numVerticesNumIndices[1] / 3;

	for (PxI32 triNr = blockIdx.x * blockDim.x + threadIdx.x; triNr < numTriangles; triNr += blockDim.x * gridDim.x)
	{
		int id0 = data.triIds[3 * triNr];
		int id1 = data.triIds[3 * triNr + 1];
		int id2 = data.triIds[3 * triNr + 2];

		PxVec3 n0 = data.normals[id0].getXYZ();
		PxVec3 n1 = data.normals[id1].getXYZ();
		PxVec3 n2 = data.normals[id2].getXYZ();

		PxVec3 n = normalize(n0 + n1 + n2);

		atomicAdd3(&data.smoothingBuffer[id0], n);
		atomicAdd3(&data.smoothingBuffer[id1], n);
		atomicAdd3(&data.smoothingBuffer[id2], n);
	}
}

extern "C" __global__ void iso_SmoothNormals(PxIsosurfaceExtractionData data)
{
	smoothNormals(data);
}

extern "C" __global__ void iso_SmoothNormalsSparse(PxSparseIsosurfaceExtractionData data)
{
	smoothNormals(data);
}

__device__ PxVec4 normalize(const PxVec4& v)
{
	PxReal s = 1.0f / PxMax(1e-6f, PxSqrt(v.x*v.x + v.y*v.y + v.z*v.z));
	return PxVec4(s*v.x, s*v.y, s*v.z, v.w);
}

template<typename T>
__device__ void smoothNormalsNormalize(T& data)
{
	const PxU32 numVertices = data.numVerticesNumIndices[0];

	for (PxI32 vNr = blockIdx.x * blockDim.x + threadIdx.x; vNr < numVertices; vNr += blockDim.x * gridDim.x)
	{
		data.normals[vNr] = normalize(data.smoothingBuffer[vNr]);
		data.smoothingBuffer[vNr] = PxVec4(0, 0, 0, 0);
	}
}

extern "C" __global__ void iso_SmoothNormalsNormalize(PxIsosurfaceExtractionData data)
{
	smoothNormalsNormalize(data);
}

extern "C" __global__ void iso_SmoothNormalsNormalizeSparse(PxSparseIsosurfaceExtractionData data)
{
	smoothNormalsNormalize(data);
}

template<typename T>
__device__ void computeNormals(T& data)
{
	const PxU32 numTriangles = data.numVerticesNumIndices[1] / 3;

	for (PxI32 triNr = blockIdx.x * blockDim.x + threadIdx.x; triNr < numTriangles; triNr += blockDim.x * gridDim.x)
	{
		/*for (int i = 0; i < 3; i++)
		{
			int id0 = data.triIds[3 * triNr + i];
			int id1 = data.triIds[3 * triNr + (i + 1) % 3];
			int id2 = data.triIds[3 * triNr + (i + 2) % 3];

			PxVec3 p0 = data.verts[id0].getXYZ();
			PxVec3 p1 = data.verts[id1].getXYZ();
			PxVec3 p2 = data.verts[id2].getXYZ();
			PxVec3 p01 = p1 - p0;
			PxVec3 p02 = p2 - p0;

			PxReal a0 = angle(p01, p02);
			PxVec3 n = (p1 - p0).cross(p2 - p0);
			PxReal l2 = n.magnitudeSquared();
			if (l2 > 1e-6f)
			{
				atomicAdd3(&data.normals[id0], (a0 * __frsqrt_rn(l2)) * n);
			}
		}*/

		int id0 = data.triIds[3 * triNr];
		int id1 = data.triIds[3 * triNr + 1];
		int id2 = data.triIds[3 * triNr + 2];

		PxVec3 p0 = data.verts[id0].getXYZ();
		PxVec3 p1 = data.verts[id1].getXYZ();
		PxVec3 p2 = data.verts[id2].getXYZ();
		PxVec3 n = (p1 - p0).cross(p2 - p0);

		atomicAdd3(&data.normals[id0], n);
		atomicAdd3(&data.normals[id1], n);
		atomicAdd3(&data.normals[id2], n);
	}
}

extern "C" __global__ void iso_ComputeNormals(PxIsosurfaceExtractionData data)
{
	computeNormals(data);
}

extern "C" __global__ void iso_ComputeNormalsSparse(PxSparseIsosurfaceExtractionData data)
{
	computeNormals(data);
}

template<typename T>
__device__ void normalizeNormals(T& data)
{
	const PxU32 numVertices = data.numVerticesNumIndices[0];
	for (PxI32 vNr = blockIdx.x * blockDim.x + threadIdx.x; vNr < numVertices; vNr += blockDim.x * gridDim.x)
		data.normals[vNr] = normalize(data.normals[vNr]);
}

extern "C" __global__ void iso_NormalizeNormals(PxIsosurfaceExtractionData data)
{
	normalizeNormals(data);
}

extern "C" __global__ void iso_NormalizeNormalsSparse(PxSparseIsosurfaceExtractionData data)
{
	normalizeNormals(data);
}

template<typename T>
__device__ void gridFilterGauss(T& data, PxReal neighborWeight)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	//Gaussian blur - use fact that it is a separable filter
	//Radius is relative to cell size
	PxReal w[3];
	w[0] = neighborWeight; // expf(-1.0f / (2.0f*radius*radius));
	w[1] = 1.0f;
	w[2] = w[0];

	PxReal weightSum = 0;
	PxReal m = 0;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	PxReal* source = data.buffer[data.swapState];
	PxReal* destination = data.buffer[1 - data.swapState];
	for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z)
	{
		PxReal weight = w[x + 1] * w[y + 1] * w[z + 1];
		weightSum += weight;
		m += weight * getGridValueSafe(data.mGrid, source, xyz, x, y, z);
	}

	destination[threadIndex] = m / weightSum;
}

extern "C" __global__ void iso_GridFilterGauss(PxIsosurfaceExtractionData data, PxReal neighborWeight)
{
	gridFilterGauss(data, neighborWeight);
}

extern "C" __global__ void iso_GridFilterGaussSparse(PxSparseIsosurfaceExtractionData data, PxReal neighborWeight)
{
	gridFilterGauss(data, neighborWeight);
}

template<typename T>
__device__ void gridFilterDilateErode(T& data, PxReal sign)
{
	int threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (outOfRange(data.mGrid, threadIndex))
		return;

	int4 xyz = getGridCoordinates(data.mGrid, threadIndex);

	PxReal* source = data.buffer[data.swapState];
	PxReal* destination = data.buffer[1 - data.swapState];
	PxReal m = sign * source[threadIndex];
	for (int x = -1; x <= 1; ++x) for (int y = -1; y <= 1; ++y) for (int z = -1; z <= 1; ++z)
		m = PxMax(m, sign * getGridValueSafe(data.mGrid, source, xyz, x, y, z));

	destination[threadIndex] = sign * m;
}

extern "C" __global__ void iso_GridFilterDilateErode(PxIsosurfaceExtractionData data, PxReal sign)
{
	gridFilterDilateErode(data, sign);
}

extern "C" __global__ void iso_GridFilterDilateErodeSparse(PxSparseIsosurfaceExtractionData data, PxReal sign)
{
	gridFilterDilateErode(data, sign);
}

