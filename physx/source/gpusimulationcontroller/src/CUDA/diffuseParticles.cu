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
#include "foundation/PxBounds3.h"
#include "PxgParticleSystemCore.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "PxgBodySim.h"
#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "shuffle.cuh"
#include "stdio.h"
#include "PxgSolverBody.h"
#include "PxgSolverCoreDesc.h"
#include "PxParticleSystem.h"
#include "assert.h"
#include "copy.cuh"
#include "PxgSimulationCoreDesc.h"
#include "gridCal.cuh"
#include "particleSystem.cuh"
#include "atomic.cuh"
#include "utils.cuh"

using namespace physx;

// simpler kernel for diffuse weighting
__device__ inline PxReal WDiffuse(const PxReal h, const PxReal invR)
{
	return (1.0f - h * invR);
}

extern "C" __host__ void initDiffuseParticlesKernels0() {}

extern "C" __global__ void ps_updateUnsortedDiffuseArrayLaunch(
	const PxgParticleSystem * PX_RESTRICT particleSystems,
	const PxU32 * PX_RESTRICT activeParticleSystems)
{
	const PxU32 particleId = activeParticleSystems[blockIdx.z];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const PxU32 bufferIndex = blockIdx.y;

	if (bufferIndex < particleSystem.mNumDiffuseBuffers)
	{
		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		float4* PX_RESTRICT unsortedPositions = reinterpret_cast<float4*>(particleSystem.mDiffusePosition_LifeTime);
		float4* PX_RESTRICT unsortedVels = reinterpret_cast<float4*>(particleSystem.mDiffuseVelocity);

		PxU32 localSum = 0;

		for (PxU32 i = threadIndexInWarp; i < bufferIndex; i += WARP_SIZE)
		{
			localSum += particleSystem.mDiffuseSimBuffers[i].mNumDiffuseParticles[0];
		}

		PxU32 bufferOffset = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, localSum);

		PxgParticleDiffuseSimBuffer& buffer = particleSystem.mDiffuseSimBuffers[bufferIndex];

		int numDiffuseParticles = buffer.mNumDiffuseParticles[0];

		const float4* particles = buffer.mDiffusePositions_LifeTime;
		const float4* vels = buffer.mDiffuseVelocities;

		const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
		if (globalThreadIndex >= numDiffuseParticles)
			return;

		if (globalThreadIndex == 0)
		{
			buffer.mStartIndex = bufferOffset;
		}

		const PxU32 ind = bufferOffset + globalThreadIndex;
		unsortedPositions[ind] = particles[globalThreadIndex];
		unsortedVels[ind] = vels[globalThreadIndex];
	}
}

extern "C" __global__ void ps_diffuseParticleOneWayCollision(
	PxgParticleSystem * PX_RESTRICT	particleSystems,
	const PxU32* PX_RESTRICT			activeParticleSystems,
	const PxU32 count
)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 id = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[id];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));
	__syncthreads();

	const PxU32 pi = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 numParticles = *shParticleSystem.mNumDiffuseParticles;


	if (pi >= numParticles)
		return;

	float4* PX_RESTRICT newPos = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedPos_LifeTime);

	const PxgParticleContactInfo* PX_RESTRICT contacts = shParticleSystem.mDiffuseOneWayContactInfos;
	const PxU32* PX_RESTRICT contactCounts = shParticleSystem.mDiffuseOneWayContactCount;

	const PxU32 contactCount = PxMin(PxgParticleContactInfo::MaxStaticContactsPerParticle, contactCounts[pi]);

	
	if (contactCount)
	{
		PxVec3 posCorr = PxLoad3(newPos[pi]);
		for (PxU32 c = 0, offset = pi; c < contactCount; ++c, offset += numParticles)
		{
			const PxgParticleContactInfo& contact = contacts[offset];

			const PxVec3 surfaceNormal = PxLoad3(contact.mNormal_PenW);

			const PxVec3 deltaP = -surfaceNormal * contact.mNormal_PenW.w;
			posCorr += deltaP;
		}

		newPos[pi] = make_float4(posCorr.x, posCorr.y, posCorr.z, newPos[pi].w);
	}
}


extern "C" __global__ void ps_diffuseParticleUpdatePBF(
	PxgParticleSystem* PX_RESTRICT				particleSystems,
	const PxU32*								activeParticleSystems,
	const PxVec3								gravity,
	const PxReal								dt)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	__shared__ int offset[3];

	if (threadIdx.x == 0)
	{
		offset[0] = 0; offset[1] = -1; offset[2] = 1;
	}

	const PxU32 id = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[id];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));
	__syncthreads();

	{
		int numDiffuse = *shParticleSystem.mNumDiffuseParticles;			

		const PxU32 pi = threadIdx.x + blockIdx.x * blockDim.x;

		if (pi >= numDiffuse)
			return;

		const PxU32* const PX_RESTRICT cellStarts = shParticleSystem.mCellStart;
		const PxU32* const PX_RESTRICT cellEnds = shParticleSystem.mCellEnd;

		// per-particle data
		const float4* const PX_RESTRICT sortedPose = reinterpret_cast<float4*>(shParticleSystem.mSortedPositions_InvMass);
		const float4* const PX_RESTRICT sortedVel = reinterpret_cast<float4*>(shParticleSystem.mSortedVelocities);

		float4* PX_RESTRICT diffusePositions = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedPos_LifeTime);

		//Overloading this buffer to store the new velocity...
		float4* PX_RESTRICT newVel = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedOriginPos_LifeTime);

		// get elements
		const float4 xi4 = diffusePositions[pi];
		const PxVec3 pos = PxLoad3(xi4);

		// interpolate		
		PxVec3 velAvg(PxZero);
		PxU32 numNeighbors = 0;

		const PxReal cellWidth = shParticleSystem.mCommonData.mGridCellWidth;
		const PxReal contactDistanceSq = shParticleSystem.mCommonData.mParticleContactDistanceSq;
		const PxReal invContactDistance = shParticleSystem.mCommonData.mParticleContactDistanceInv;
		const int3 gridPos = calcGridPos(xi4, cellWidth);
		const uint3 gridSize = make_uint3(shParticleSystem.mCommonData.mGridSizeX, shParticleSystem.mCommonData.mGridSizeY, shParticleSystem.mCommonData.mGridSizeZ);

		// Iterate over cell
		PxReal weightSum = 0.0f;
		PxVec3 velocitySum(0.f);

		const PxU32 maxNeighbors = 16;

		const PxU32 end = (shParticleSystem.mData.mFlags & PxParticleFlag::eFULL_DIFFUSE_ADVECTION) ? 3 : 1;


		for (int z = 0; z < end; ++z)
			for (int y = 0; y < end; ++y)
				for (int x = 0; x < end; ++x)
				{
					const int3 neighbourPos = make_int3(gridPos.x + offset[x], gridPos.y + offset[y], gridPos.z + offset[z]);
					const PxU32 gridHash = calcGridHash(neighbourPos, gridSize);
					const PxU32 startIndex = cellStarts[gridHash];

					if (startIndex != EMPTY_CELL)
					{
						const PxU32 endIndex = cellEnds[gridHash];
						for (PxU32 q = startIndex; q < endIndex; ++q)
						{
							const PxVec3 xj = PxLoad3(sortedPose[q]);
							const PxVec3 xij = pos - xj;

							const PxReal dSq = xij.dot(xij);

							if (dSq < contactDistanceSq)
							{
								const PxVec3 vj = PxLoad3(sortedVel[q]);
								const PxReal w = WDiffuse(sqrtf(dSq), invContactDistance);

								weightSum += w;
								velocitySum += vj * w;

								++numNeighbors;
								if (numNeighbors == maxNeighbors)
									goto weight_sum;
							}
						}
					}
				}

	weight_sum:
		if (weightSum > 0)
			velAvg = velocitySum / weightSum;

		newVel[pi] = make_float4(velAvg.x, velAvg.y, velAvg.z, PxReal(numNeighbors));
	}
}

extern "C" __global__ void ps_diffuseParticleCompact(
	PxgParticleSystem* PX_RESTRICT				particleSystems,
	const PxU32*								activeParticleSystems,
	const PxVec3								gravity,
	const PxReal								dt)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 id = activeParticleSystems[blockIdx.z];

	const PxgParticleSystem& particleSystem = particleSystems[id];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));
    __syncthreads();

	const PxU32 bufferIndex = blockIdx.y;
	if (bufferIndex < shParticleSystem.mNumDiffuseBuffers)
	{

		PxgParticleDiffuseSimBuffer& buffer = shParticleSystem.mDiffuseSimBuffers[bufferIndex];

		int* numDiffuseParticles = buffer.mNumDiffuseParticles;
		int numDiffuse = numDiffuseParticles[0];

		const PxU32 pi = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		if (pi >= numDiffuse)
			return;

		float4* PX_RESTRICT diffusePositionsNew = buffer.mDiffusePositions_LifeTime;
		float4* PX_RESTRICT diffuseVelocitiesNew = buffer.mDiffuseVelocities;

		float4* PX_RESTRICT velAvgs = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedOriginPos_LifeTime);

		float4* PX_RESTRICT diffusePositions = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedPos_LifeTime);
		float4* PX_RESTRICT diffusePositionsOld = reinterpret_cast<float4*>(shParticleSystem.mDiffuseOriginPos_LifeTime);
		
		const PxU32* reverseLookup = shParticleSystem.mDiffuseUnsortedToSortedMapping;
		
		const PxU32 index = pi + buffer.mStartIndex;
		const PxU32 sortedInd = reverseLookup[index];

		// get elements
		const float4 xi4 = diffusePositions[sortedInd];
		const float4 vi4Old = diffusePositionsOld[index];
		const float4 xiva4 = velAvgs[sortedInd];
		const PxVec3 pos = PxLoad3(xi4);
		const PxVec3 oldPos = PxLoad3(vi4Old);
		const PxVec3 velAvg = PxLoad3(xiva4);

		const PxReal lifeDelta = dt;

		PxVec3 vel = (pos - oldPos)*(1.f / dt);

		// integrate diffuse particle
		PxVec3 newVel;
		if (xiva4.w < 4.f)
		{
			// spray (ballistic)
			newVel = vel * (1.0f - buffer.mParams.airDrag * dt);
		}
		else if (xiva4.w < 8.f)
		{
			// foam
			newVel = velAvg;
		}
		else
		{
			// bubble
			newVel = vel - (1.f + buffer.mParams.buoyancy) * gravity * dt + buffer.mParams.bubbleDrag * (velAvg - vel);
		}

		const float maxVel = shParticleSystem.mData.mMaxVelocity;
		if (newVel.magnitudeSquared() > 0)
		{
			newVel = PxMin(newVel.magnitude(), maxVel) * newVel.getNormalized();
		}

		PxVec3 newPosCorr = pos + (newVel - vel) * dt;
		PxVec3 newVelCorr = newVel;

		__syncwarp();

		const PxReal lifeTime = fmaxf(xi4.w - lifeDelta, 0.0f);

		PxU32 res = __ballot_sync(FULL_MASK, lifeTime > 0.f);

		PxU32 offset = 0;

		if (threadIndexInWarp == 0)
			offset = atomicAdd(&numDiffuseParticles[1], __popc(res));

		offset = __shfl_sync(FULL_MASK, offset, 0);



		if (lifeTime > 0.f)
		{
			PxU32 newIndex = offset + warpScanExclusive(res, threadIndexInWarp);
			
			diffusePositionsNew[newIndex] = make_float4(newPosCorr.x, newPosCorr.y, newPosCorr.z, lifeTime);
			diffuseVelocitiesNew[newIndex] = make_float4(newVelCorr.x, newVelCorr.y, newVelCorr.z, 0.0f);
		}
	}
}

extern "C" __global__ void ps_diffuseParticleCreate(
	PxgParticleSystem * PX_RESTRICT			particleSystems,
	const PxU32* const PX_RESTRICT				activeParticleSystems,
	const PxReal* const PX_RESTRICT			randomTable,
	const PxU32									randomTableSize,
	const PxReal								dt)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 id = activeParticleSystems[blockIdx.z];
	const PxgParticleSystem& particleSystem = particleSystems[id];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));
	__syncthreads();

	const PxU32 bufferIndex = blockIdx.y;
	if (bufferIndex < shParticleSystem.mCommonData.mNumParticleBuffers)
	{

		const PxgParticleSimBuffer& buffer = shParticleSystem.mParticleSimBuffers[bufferIndex];

		const PxU32 diffuseParticleBufferIndex = buffer.mDiffuseParticleBufferIndex;

		if (diffuseParticleBufferIndex == 0xffffffff)
			return;

		const PxgParticleSystemData& data = shParticleSystem.mData;

		const PxU32 pi = threadIdx.x + blockIdx.x * blockDim.x;

		const PxU32 numParticles = buffer.mNumActiveParticles;

		if (pi >= numParticles)
			return;


		PxgParticleDiffuseSimBuffer& diffuseBuffer = shParticleSystem.mDiffuseSimBuffers[diffuseParticleBufferIndex];

		if (diffuseBuffer.mMaxNumParticles == 0)
			return;
	
		// get arrays
		const float4* const PX_RESTRICT sortedPose = reinterpret_cast<float4*>(shParticleSystem.mSortedPositions_InvMass);
		const float4* const PX_RESTRICT sortedVel = reinterpret_cast<float4*>(shParticleSystem.mSortedVelocities);
		const PxU32* PX_RESTRICT phases = shParticleSystem.mSortedPhaseArray;
		const float2* const PX_RESTRICT potentials = reinterpret_cast<float2*>(shParticleSystem.mDiffusePotentials);
		
		float4* PX_RESTRICT diffusePositionsNew = diffuseBuffer.mDiffusePositions_LifeTime;
		float4* PX_RESTRICT diffuseVelocitiesNew = diffuseBuffer.mDiffuseVelocities;

		int* numDiffuseParticles = diffuseBuffer.mNumDiffuseParticles;
		
		const PxU32* reverseLookup = shParticleSystem.mUnsortedToSortedMapping;
		const PxU32 offset = particleSystem.mParticleBufferRunsum[bufferIndex];

		const PxU32 sortedInd = reverseLookup[pi + offset];
		// get elements
		const float2 ptnts = potentials[sortedInd];
		const PxReal threshold = diffuseBuffer.mParams.threshold;
		const PxU32 phase = phases[sortedInd];

		if (!PxGetFluid(phase))
			return;

		const float4 vi4 = sortedVel[sortedInd];

		//Kinetic energy + pressure
		const PxReal kineticEnergy = dot3(vi4, vi4) * diffuseBuffer.mParams.kineticEnergyWeight;
		const PxReal divergence = diffuseBuffer.mParams.divergenceWeight * ptnts.x;
		const PxReal pressure = diffuseBuffer.mParams.pressureWeight * ptnts.y;
		PxReal intensity = pressure - divergence + kineticEnergy;

		//if (pi == 0)
		//	printf("numParticles %i diffuseParticleBufferIndex %i numDiffuseParticles[1] %i threshold %f\n", numParticles, diffuseParticleBufferIndex, numDiffuseParticles[1], threshold);

		const PxReal r0 = randomTable[(sortedInd + 0) % randomTableSize];

		if(r0 * intensity > threshold)
		{
			const float4 xi4 = sortedPose[sortedInd];
			

			//for (int i=0; i < 5; ++i)
			{
				// try and allocate new diffuse particles
				const int newIndex = atomicAdd(&numDiffuseParticles[1], 1);

				if (newIndex < diffuseBuffer.mMaxNumParticles)
				{
					
					const PxVec3 xi = PxLoad3(xi4);
					const PxVec3 vi = PxLoad3(vi4);

					const PxReal r1 = randomTable[(sortedInd + 1) % randomTableSize];
					const PxReal r2 = randomTable[(sortedInd + 2) % randomTableSize];
					const PxReal r3 = randomTable[(sortedInd + 3) % randomTableSize];

					const PxReal lifeMin = 1.0f;
					const PxReal lifeMax = diffuseBuffer.mParams.lifetime;
					const PxReal lifeScale = fminf(intensity / threshold, 1.f) * r1;
					const PxReal lifetime = lifeMin + lifeScale * (lifeMax - lifeMin);

					const PxVec3 q = xi - r2 * vi * dt + PxVec3(r1, r2, r3) * data.mRestOffset * 0.25f;

					diffusePositionsNew[newIndex] = make_float4(q.x, q.y, q.z, lifetime);
					diffuseVelocitiesNew[newIndex] = make_float4(vi.x, vi.y, vi.z, 0.0f);
				}
			}
		}
	}
}


extern "C" __global__ void ps_diffuseParticleCopy(
	PxgParticleSystem * PX_RESTRICT	particleSystems,
	const PxU32* const PX_RESTRICT	activeParticleSystems,
	const PxU32 count)
{
	const PxU32 id = activeParticleSystems[blockIdx.z];
	PxgParticleSystem& particleSystem = particleSystems[id];

	const PxU32 numDiffuseBuffers = particleSystem.mNumDiffuseBuffers;

	const PxU32 bufferIndex = blockIdx.y;
	if (bufferIndex < numDiffuseBuffers)
	{
		PxgParticleDiffuseSimBuffer& diffuseBuffer = particleSystem.mDiffuseSimBuffers[bufferIndex];

		int* numDiffuseParticles = diffuseBuffer.mNumDiffuseParticles;
		const PxU32 numDiffuse = PxMin(PxI32(diffuseBuffer.mMaxNumParticles), numDiffuseParticles[1]);
		*diffuseBuffer.mNumActiveDiffuseParticles = numDiffuse; //pinned memory
		numDiffuseParticles[0] = numDiffuse;
		numDiffuseParticles[1] = 0;

	}
}


extern "C" __global__ void ps_diffuseParticleSum(
	PxgParticleSystem * PX_RESTRICT	particleSystems,
	const PxU32* const PX_RESTRICT	activeParticleSystems,
	const PxU32 count)
{
	const PxU32 id = activeParticleSystems[blockIdx.x];
	PxgParticleSystem& particleSystem = particleSystems[id];

	const PxU32 numDiffuseBuffers = particleSystem.mNumDiffuseBuffers;

	PxU32 totalDiffuse = 0;
	for (PxU32 i = threadIdx.x; i < numDiffuseBuffers; i += WARP_SIZE)
	{
		PxgParticleDiffuseSimBuffer& diffuseBuffer = particleSystem.mDiffuseSimBuffers[i];
		totalDiffuse += diffuseBuffer.mNumDiffuseParticles[0];
	}

	totalDiffuse = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, totalDiffuse);


	if(threadIdx.x == 0)
	{
		*particleSystem.mNumDiffuseParticles = totalDiffuse;
	}
}
