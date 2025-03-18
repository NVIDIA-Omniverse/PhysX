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

#include "matrixDecomposition.cuh"
#include "PxParticleGpu.h"
#include "PxgAnisotropyData.h"
#include "sparseGridStandalone.cuh"

#define ENABLE_KERNEL_LAUNCH_ERROR_CHECK 0

extern "C" __host__ void initAnisotropyKernels0() {}

__device__ inline PxVec3 PxLoad3(const float4& v) { float4 tmp = v; return PxVec3(tmp.x, tmp.y, tmp.z); }
__device__ inline PxVec4 PxLoad4(const float4& v) { float4 tmp = v; return PxVec4(tmp.x, tmp.y, tmp.z, tmp.w); }

__device__ inline PxReal cube(PxReal x) { return x * x * x; }
__device__ inline PxReal Wa(PxReal x, PxReal invr)
{
	return 1.f - cube(x*invr);
}
template <typename V, typename T>
__device__ inline V Lerp(const V& start, const V& end, const T& t)
{
	return start + (end - start) * t;
}
template <typename V, typename T>
__device__ inline V Clamp(const V& a, const T s, const T t) {
	return V(PxMin(t, PxMax(s, a[0])),
		PxMin(t, PxMax(s, a[1])),
		PxMin(t, PxMax(s, a[2])));
}

//
//extern "C" __global__ void smoothPositionsLaunch(PxU32* sortedOrder, PxU32* cellEnds, PxU float4* pose, PxU32* phase, float particleContactDistance)
//{
//	const float4* const PX_RESTRICT sortedPose = reinterpret_cast<float4*>(particleSystem.mSortedPositions_InvMass);
//	const PxU32* const PX_RESTRICT sortedPhases = particleSystem.mSortedPhaseArray;
//
//	const PxU32* const PX_RESTRICT cellStart = particleSystem.mCellStart;
//	const PxU32* const PX_RESTRICT cellEnd = particleSystem.mCellEnd;
//
//	const PxReal particleContactDistanceSq = particleContactDistance * particleContactDistance;
//	const PxReal particleContactDistanceInv = 1.0f / particleContactDistance;
//
//	// calculated the sum of weights and weighted avg position for particle neighborhood
//	PxVec3 xs(0.0f); //sum of positions
//	PxReal ws = 0.0f; //sum of weights
//	for (int z = -1; z <= 1; z++)
//	{
//		for (int y = -1; y <= 1; y++)
//		{
//			for (int x = -1; x <= 1; x++)
//			{
//				int3 neighbourPos = make_int3(gridPos.x + x, gridPos.y + y, gridPos.z + z);
//				PxU32 gridHash = calcGridHash(neighbourPos, gridSize);
//				PxU32 startIndex = cellStart[gridHash];
//				PxU32 endIndex = cellEnd[gridHash];
//
//				if (startIndex != EMPTY_CELL)
//				{
//					PxU32 nextPhase = sortedPhases[startIndex];
//					float4 nextPos = fetch(&sortedPose[startIndex]);
//
//					for (PxU32 particleIndex1 = startIndex; particleIndex1 < endIndex /*&& numCollidedParticles < maxNeighborhood*/; particleIndex1++)
//					{
//						const PxU32 phase2 = nextPhase;
//						const float4 pos2 = nextPos;
//
//						if ((particleIndex1 + 1) < endIndex)
//						{
//							nextPhase = sortedPhases[particleIndex1 + 1];
//							nextPos = fetch(&sortedPose[particleIndex1 + 1]);
//						}
//
//						if (phase2 & validPhaseMask)
//						{
//							const PxVec3 xj = PxLoad3(pos2);
//							const PxVec3 xij = xi - xj;
//
//							const PxReal dsq = xij.magnitudeSquared();
//
//							if (0.0f < dsq && dsq < particleContactDistanceSq)
//							{
//								const PxReal w = Wa(sqrtf(dsq), particleContactDistanceInv);
//								ws += w;
//								xs += xj * w;
//							}
//						}
//					}
//				}
//			}
//		}
//	}
//
//	if (ws > 0.f)
//	{
//		PxReal f = 4.0f*Wa(particleContactDistance*0.5f, particleContactDistanceInv);
//		PxReal smooth = PxMin(1.0f, ws / f)*smoothing;
//		xs /= ws;
//		xi = Lerp(xi, xs, smooth);
//	}
//
//	//write smoothed positions back in API order
//	smoothedPositions[origIdx] = make_float4(xi.x, xi.y, xi.z, xi4.w);
//}

// Smooths particle positions in a fluid by moving them closer to the weighted
// average position of their local neighborhood
extern "C" __global__ void smoothPositionsLaunch(PxGpuParticleSystem* particleSystems, const PxU32 id, PxSmoothedPositionData* smoothingDataPerParticleSystem)
{
	PxGpuParticleSystem& particleSystem = particleSystems[id];

	const PxU32 numParticles = particleSystem.mCommonData.mNumParticles;

	PxSmoothedPositionData& data = smoothingDataPerParticleSystem[id];
	const PxReal smoothing = data.mSmoothing;

	//pointers to global memory buffers -- inputs
	const float4* PX_RESTRICT sortedNewPositions = particleSystem.mSortedPositions_InvMass;
	const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
	const PxU32* PX_RESTRICT collisionIndex = particleSystem.mCollisionIndex;
	const PxU32* PX_RESTRICT gridParticleIndices = particleSystem.mSortedToUnsortedMapping;

	//pointers to smoothed position buffers -- outputs
	float4* PX_RESTRICT smoothPosOrig = reinterpret_cast<float4*>(data.mPositions); // particleSystem.mSmoothedOriginPos_InvMass;

	const PxU32 globalThreadIdx = blockIdx.x * blockDim.x + threadIdx.x;

	if (globalThreadIdx >= numParticles)
		return; //skip thread if it's past the end of particle list

	const PxU32 p = globalThreadIdx; //index of particle in sorted order
	const PxU32 origIdx = gridParticleIndices[globalThreadIdx];

	const PxVec4 xi4 = PxLoad4(sortedNewPositions[p]);

	//ignore smoothing for non-fluid particles & write original position
	if (!PxGetFluid(phases[p]))
	{
		PxVec4 x = PxLoad4(sortedNewPositions[p]);
		smoothPosOrig[origIdx] = make_float4(x.x, x.y, x.z, x.w);
		return;
	}

	PxVec3 xi = PxVec3(xi4.x, xi4.y, xi4.z);

	PxU32 contactCount = particleSystem.mParticleSelfCollisionCount[p];

	// calculated the sum of weights and weighted avg position for particle neighborhood
	PxVec3 xs(0.0f); //sum of positions
	PxReal ws = 0.0f; //sum of weights
	for (PxU32 i = 0, offset = p; i < contactCount; ++i, offset += numParticles)
	{
		const PxU32 q = collisionIndex[offset];
		if (PxGetFluid(phases[q])) //ignore non-fluid particles
		{
			const PxVec3 xj = PxLoad3(sortedNewPositions[q]);
			const PxVec3 xij = xi - xj;

			const PxReal dsq = xij.magnitudeSquared();

			if (0.0f < dsq && dsq < particleSystem.mCommonData.mParticleContactDistanceSq)
			{
				const PxReal w = Wa(sqrtf(dsq), particleSystem.mCommonData.mParticleContactDistanceInv);
				ws += w;
				xs += xj * w;
			}
		}
	}

	if (ws > 0.f)
	{
		PxReal f = 4.0f*Wa(particleSystem.mCommonData.mParticleContactDistance*0.5f, particleSystem.mCommonData.mParticleContactDistanceInv);
		PxReal smooth = PxMin(1.0f, ws / f)*smoothing;
		xs /= ws;
		xi = Lerp(xi, xs, smooth);
	}

	//write smoothed positions back in API order
	smoothPosOrig[origIdx] = make_float4(xi.x, xi.y, xi.z, xi4.w);
}

// Calculates Eigen-decomposition of the particle covariance matrix according
// to "Reconstructing Surfaces of Particle-Based Fluids Using Anisotropic Kernels"
extern "C" __global__ void calculateAnisotropyLaunch(PxGpuParticleSystem* particleSystems, const PxU32 id, PxAnisotropyData* anisotropyDataPerParticleSystem)
{
	PxGpuParticleSystem& particleSystem = particleSystems[id];

	const PxU32 numParticles = particleSystem.mCommonData.mNumParticles;

	//pointers to global memory buffers -- inputs
	const float4* PX_RESTRICT sortedNewPositions = particleSystem.mSortedPositions_InvMass;
	const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
	const PxU32* PX_RESTRICT collisionIndex = particleSystem.mCollisionIndex;
	const PxU32* PX_RESTRICT gridParticleIndices = particleSystem.mSortedToUnsortedMapping;

	const PxAnisotropyData& anisotropyData = anisotropyDataPerParticleSystem[id];
	float4* PX_RESTRICT q1 = reinterpret_cast<float4*>(anisotropyData.mAnisotropy_q1);
	float4* PX_RESTRICT q2 = reinterpret_cast<float4*>(anisotropyData.mAnisotropy_q2);
	float4* PX_RESTRICT q3 = reinterpret_cast<float4*>(anisotropyData.mAnisotropy_q3);

	const PxU32 globalThreadIdx = blockIdx.x * blockDim.x + threadIdx.x;

	if (globalThreadIdx >= numParticles)
		return; //skip thread if it's past the end of particle list

	const PxU32 p = globalThreadIdx; //index of particle in sorted order
	const PxU32 origIdx = gridParticleIndices[globalThreadIdx];

	//ignore anisotropy for non-fluid particles
	if (!PxGetFluid(phases[p]))
	{
		float r = anisotropyData.mAnisotropyMin * particleSystem.mCommonData.mParticleContactDistance;
		q1[origIdx] = make_float4(1.0f, 0.0f, 0.0f, r);
		q2[origIdx] = make_float4(0.0f, 1.0f, 0.0f, r);
		q3[origIdx] = make_float4(0.0f, 0.0f, 1.0f, r);
		/*if (globalThreadIdx == 0)
		printf("PxGetFluid\n");*/
		return;
	}

	const PxVec3 xi = PxLoad3(sortedNewPositions[p]);

	PxU32 contactCount = particleSystem.mParticleSelfCollisionCount[p];

	// calculated the sum of weights and weighted avg position for particle neighborhood
	PxVec3 xs(0.f); //sum of positions
	float ws = 0.f; //sum of weights

	PxU32 nextQ;
	PxU32 nextNextQ;
	PxVec4 xj4Next;
	PxU32 nextPhase;

	PxU32 offset = p;

	if (contactCount > 0)
	{
		nextQ = collisionIndex[offset];
		xj4Next = PxLoad4(sortedNewPositions[nextQ]);
		nextPhase = phases[nextQ];

		offset += numParticles;
	}
	if (contactCount > 1)
	{
		nextNextQ = collisionIndex[offset];
		offset += numParticles;
	}

	for (PxU32 i = 0; i < contactCount; ++i, offset += numParticles)
	{
		const PxVec4 xj4 = xj4Next;
		const PxU32 phase2 = nextPhase;

		if ((i + 1) < contactCount)
		{
			xj4Next = PxLoad4(sortedNewPositions[nextNextQ]);
			nextPhase = phases[nextNextQ];

			nextQ = nextNextQ;

			if ((i + 2) < contactCount)
				nextNextQ = collisionIndex[offset];

		}
		if (PxGetFluid(phase2)) //ignore non-fluid particles
		{

			const PxVec3 xj(xj4.x, xj4.y, xj4.z);
			const PxVec3 xij = xi - xj;

			const PxReal dsq = xij.magnitudeSquared();

			if (0.f < dsq && dsq < particleSystem.mCommonData.mParticleContactDistanceSq)
			{
				const PxReal w = Wa(sqrtf(dsq), particleSystem.mCommonData.mParticleContactDistanceInv);
				ws += w;
				xs += xj * w;
			}
		}
	}

	// set to radial and exit early in case of isolated particles
	if (ws == 0.0f)
	{
		float r = anisotropyData.mAnisotropyMin * particleSystem.mCommonData.mParticleContactDistance;
		q1[origIdx] = make_float4(1.0f, 0.0f, 0.0f, r);
		q2[origIdx] = make_float4(0.0f, 1.0f, 0.0f, r);
		q3[origIdx] = make_float4(0.0f, 0.0f, 1.0f, r);
		//if(globalThreadIdx==0)
		//printf("%i, ws\n", contactCount);
		return;
	}

	//compute inverse sum weight and weight the average position
	float invWs = 1.f / ws;
	xs *= invWs;

	PxMat33 covariance(PxVec3(0.0f), PxVec3(0.0f), PxVec3(0.0f));

	offset = p;

	if (contactCount > 0)
	{
		nextQ = collisionIndex[offset];
		xj4Next = PxLoad4(sortedNewPositions[nextQ]);
		nextPhase = phases[nextQ];

		offset += numParticles;
	}
	if (contactCount > 1)
	{
		nextNextQ = collisionIndex[offset];
		offset += numParticles;
	}

	// use weighted average position to calculate the covariance matrix
	for (PxU32 i = 0; i < contactCount; ++i, offset += numParticles)
	{
		const PxVec4 xj4 = xj4Next;
		const PxU32 phase2 = nextPhase;

		if ((i + 1) < contactCount)
		{
			xj4Next = PxLoad4(sortedNewPositions[nextNextQ]);
			nextPhase = phases[nextNextQ];

			nextQ = nextNextQ;

			if ((i + 2) < contactCount)
				nextNextQ = collisionIndex[offset];

		}
		if (PxGetFluid(phase2)) //ignore non-fluid particles
		{
			const PxVec3 xj(xj4.x, xj4.y, xj4.z);
			const PxVec3 xij = xi - xj;

			const PxReal dsq = xij.magnitudeSquared();

			if (0.f < dsq && dsq < particleSystem.mCommonData.mParticleContactDistanceSq)
			{
				const PxReal w = Wa(sqrtf(dsq), particleSystem.mCommonData.mParticleContactDistanceInv);
				const PxVec3 xjs = xj - xs;
				covariance += PxMat33::outer(w*xjs, xjs);
			}
		}
	}

	covariance *= invWs;

	//calculate the eigen decomposition
	PxMat33 r;
	eigenDecomposition(covariance, r);

	//sanitize the eigen values (diagonal of covariance matrix)
	covariance[0][0] = max(covariance[0][0], 0.f);
	covariance[1][1] = max(covariance[1][1], 0.f);
	covariance[2][2] = max(covariance[2][2], 0.f);

	PxVec3 lambda(sqrtf(covariance[0][0]), sqrtf(covariance[1][1]), sqrtf(covariance[2][2]));
	//PxVec3 lambda(covariance[0][0], covariance[1][1], covariance[2][2]);

	const float ks = anisotropyData.mAnisotropy;
	const float kmin = anisotropyData.mAnisotropyMin * particleSystem.mCommonData.mParticleContactDistance;
	const float kmax = anisotropyData.mAnisotropyMax * particleSystem.mCommonData.mParticleContactDistance;

	lambda *= ks;
	lambda = Clamp(lambda, kmin, kmax);

	//write out the anisotropy vectors
	q1[origIdx] = make_float4(r.column0.x, r.column0.y, r.column0.z, lambda.x);
	q2[origIdx] = make_float4(r.column1.x, r.column1.y, r.column1.z, lambda.y);
	q3[origIdx] = make_float4(r.column2.x, r.column2.y, r.column2.z, lambda.z);
}




extern "C" __global__ __launch_bounds__(256, 1)  void anisotropyKernel(const float4* const PX_RESTRICT deviceParticlePos,
	const PxU32* const PX_RESTRICT sortedToOriginalParticleIndex, const PxU32*  const PX_RESTRICT sortedParticleToSubgrid, PxU32 maxNumSubgrids,
	const PxU32*  const PX_RESTRICT subgridNeighbors, const PxU32* const PX_RESTRICT subgridEndIndices, int numParticles, PxU32* phases, PxU32 validPhaseMask,
	float4* q1, float4* q2, float4* q3, PxReal anisotropy, PxReal anisotropyMin, PxReal anisotropyMax, PxReal particleContactDistance)
{
	PxI32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= numParticles)
		return;

	PxI32 pNr = sortedToOriginalParticleIndex[threadIndex];

	PxU32 subgridIndex = sortedParticleToSubgrid[threadIndex];
	if (subgridIndex >= maxNumSubgrids || (phases && !(phases[pNr] & validPhaseMask)))
	{
		float r = anisotropyMin * particleContactDistance;
		q1[pNr] = make_float4(1.0f, 0.0f, 0.0f, r);
		q2[pNr] = make_float4(0.0f, 1.0f, 0.0f, r);
		q3[pNr] = make_float4(0.0f, 0.0f, 1.0f, r);
		return;
	}

	PxVec3 xi = PxLoad3(deviceParticlePos[pNr]);

	const PxReal particleContactDistanceSq = particleContactDistance * particleContactDistance;
	const PxReal particleContactDistanceInv = 1.0f / particleContactDistance;

	// calculated the sum of weights and weighted avg position for particle neighborhood
	PxVec3 xs(0.f); //sum of positions
	float ws = 0.f; //sum of weights
	for (int z = -1; z <= 1; z++)
	{
		for (int y = -1; y <= 1; y++)
		{
			for (int x = -1; x <= 1; x++)
			{
				PxU32 n = subgridNeighborOffset(subgridNeighbors, subgridIndex, x, y, z);
				if (n == EMPTY_SUBGRID)
					continue;

				int start = n == 0 ? 0 : subgridEndIndices[n - 1];
				int end = subgridEndIndices[n];
				for (int i = start; i < end; ++i)
				{
					int j = sortedToOriginalParticleIndex[i];
					if (phases && !(phases[j] & validPhaseMask))
						continue;

					PxVec3 xj = PxLoad3(deviceParticlePos[j]);

					const PxVec3 xij = xi - xj;

					const PxReal dsq = xij.magnitudeSquared();

					if (0.f < dsq && dsq < particleContactDistanceSq)
					{
						const PxReal w = Wa(sqrtf(dsq), particleContactDistanceInv);
						ws += w;
						xs += xj * w;
					}
				}
			}
		}
	}


	// set to radial and exit early in case of isolated particles
	if (ws == 0.0f)
	{
		float r = anisotropyMin * particleContactDistance;
		q1[pNr] = make_float4(1.0f, 0.0f, 0.0f, r);
		q2[pNr] = make_float4(0.0f, 1.0f, 0.0f, r);
		q3[pNr] = make_float4(0.0f, 0.0f, 1.0f, r);
		//if(globalThreadIdx==0)
		//printf("%i, ws\n", contactCount);
		return;
	}

	//compute inverse sum weight and weight the average position
	float invWs = 1.f / ws;
	xs *= invWs;

	PxMat33 covariance(PxVec3(0.0f), PxVec3(0.0f), PxVec3(0.0f));


	for (int z = -1; z <= 1; z++)
	{
		for (int y = -1; y <= 1; y++)
		{
			for (int x = -1; x <= 1; x++)
			{
				PxU32 n = subgridNeighborOffset(subgridNeighbors, subgridIndex, x, y, z);
				if (n == EMPTY_SUBGRID)
					continue;

				int start = n == 0 ? 0 : subgridEndIndices[n - 1];
				int end = subgridEndIndices[n];
				for (int i = start; i < end; ++i)
				{
					int j = sortedToOriginalParticleIndex[i];
					if (phases && !(phases[j] & validPhaseMask))
						continue;

					PxVec3 xj = PxLoad3(deviceParticlePos[j]);
					const PxVec3 xij = xi - xj;

					const PxReal dsq = xij.magnitudeSquared();

					if (0.f < dsq && dsq < particleContactDistanceSq)
					{
						const PxReal w = Wa(sqrtf(dsq), particleContactDistanceInv);
						const PxVec3 xjs = xj - xs;
						covariance += PxMat33::outer(w*xjs, xjs);
					}
				}
			}
		}
	}


	covariance *= invWs;

	//calculate the eigen decomposition
	PxMat33 r;
	eigenDecomposition(covariance, r);

	//sanitize the eigen values (diagonal of covariance matrix)
	covariance[0][0] = max(covariance[0][0], 0.f);
	covariance[1][1] = max(covariance[1][1], 0.f);
	covariance[2][2] = max(covariance[2][2], 0.f);

	PxVec3 lambda(sqrtf(covariance[0][0]), sqrtf(covariance[1][1]), sqrtf(covariance[2][2]));
	//PxVec3 lambda(covariance[0][0], covariance[1][1], covariance[2][2]);

	const float ks = anisotropy;
	const float kmin = anisotropyMin * particleContactDistance;
	const float kmax = anisotropyMax * particleContactDistance;

	lambda *= ks;
	lambda = Clamp(lambda, kmin, kmax);

	//write out the anisotropy vectors
	q1[pNr] = make_float4(r.column0.x, r.column0.y, r.column0.z, lambda.x);
	q2[pNr] = make_float4(r.column1.x, r.column1.y, r.column1.z, lambda.y);
	q3[pNr] = make_float4(r.column2.x, r.column2.y, r.column2.z, lambda.z);
}

extern "C" __global__ void smoothPositionsKernel(float4* deviceParticlePos, PxU32* sortedToOriginalParticleIndex, PxU32* sortedParticleToSubgrid, PxU32 maxNumSubgrids,
	PxU32* subgridNeighbors, PxU32* subgridEndIndices, int numParticles, PxU32* phases, PxU32 validPhaseMask, float4* smoothPos, PxReal smoothing, PxReal particleContactDistance)
{
	PxI32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	if (threadIndex >= numParticles)
		return;

	PxI32 pNr = sortedToOriginalParticleIndex[threadIndex];
	float4 xi4 = deviceParticlePos[pNr];

	PxU32 subgridIndex = sortedParticleToSubgrid[threadIndex];
	if (subgridIndex >= maxNumSubgrids || (phases && !(phases[pNr] & validPhaseMask)))
	{
		smoothPos[pNr] = xi4;
		return;
	}

	PxVec3 xi = PxLoad3(deviceParticlePos[pNr]);

	const PxReal particleContactDistanceSq = particleContactDistance * particleContactDistance;
	const PxReal particleContactDistanceInv = 1.0f / particleContactDistance;

	// calculated the sum of weights and weighted avg position for particle neighborhood
	PxVec3 xs(0.0f); //sum of positions
	PxReal ws = 0.0f; //sum of weights
	for (int z = -1; z <= 1; z++)
	{
		for (int y = -1; y <= 1; y++)
		{
			for (int x = -1; x <= 1; x++)
			{
				PxU32 n = subgridNeighborOffset(subgridNeighbors, subgridIndex, x, y, z);
				if (n == EMPTY_SUBGRID)
					continue;

				int start = n == 0 ? 0 : subgridEndIndices[n - 1];
				int end = subgridEndIndices[n];
				for (int i = start; i < end; ++i)
				{
					int j = sortedToOriginalParticleIndex[i];
					if (phases && !(phases[j] & validPhaseMask))
						continue;

					PxVec3 xj = PxLoad3(deviceParticlePos[j]);

					//Now do the actual calculation
					const PxVec3 xij = xi - xj;

					const PxReal dsq = xij.magnitudeSquared();

					if (0.0f < dsq && dsq < particleContactDistanceSq)
					{
						const PxReal w = Wa(sqrtf(dsq), particleContactDistanceInv);
						ws += w;
						xs += xj * w;
					}
				}
			}
		}
	}

	if (ws > 0.f)
	{
		PxReal f = 4.0f*Wa(particleContactDistance*0.5f, particleContactDistanceInv);
		PxReal smooth = PxMin(1.0f, ws / f)*smoothing;
		xs /= ws;
		xi = Lerp(xi, xs, smooth);
	}

	//write smoothed positions back in API order
	smoothPos[pNr] = make_float4(xi.x, xi.y, xi.z, xi4.w);
}


