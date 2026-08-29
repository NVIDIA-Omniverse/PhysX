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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxBounds3.h"
#include "foundation/PxMathUtils.h"
#include "PxgParticleSystem.h"
#include "PxgParticleSystemCoreKernelIndices.h"
#include "deformableUtils.cuh"
#include "PxgBodySim.h"
#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "shuffle.cuh"
#include "gridCal.cuh"
#include "stdio.h"
#include "PxgSolverBody.h"
#include "PxgSolverCoreDesc.h"
#include "PxNodeIndex.h"
#include "PxParticleSystem.h"
#include "assert.h"
#include "copy.cuh"
#include "PxgSimulationCoreDesc.h"
#include "PxDeformableAttachment.h"

#include "PxgArticulation.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgFEMCloth.h"
#include "PxsPBDMaterialCore.h"

#include "particleSystem.cuh"
#include "deformableUtils.cuh"
#include "atomic.cuh"

#include "matrixDecomposition.cuh"
#include "dataReadWriteHelper.cuh"


#if __CUDA_ARCH__ >= 320
template <typename T> __device__ inline T fetch(T* t) { return __ldg(t); }
#else
template <typename T> __device__ inline T fetch(T* t) { return *t;}
#endif


using namespace physx;

#define kGlobalRelax  0.125f

extern "C" __host__ void initParticleSystemKernels0() {}

__device__ inline PxReal InvSqrt(PxReal x)
{
	return 1.0f / sqrtf(x);
}

template <typename T>
__device__ inline T SafeNormalize(const T& v, const T& fallback)
{
	PxReal l = v.magnitudeSquared();
	if (l > 0.0f)
	{
		return v * InvSqrt(l);
	}
	else
		return fallback;
}

template <typename V, typename T>
__device__ inline V Lerp(const V& start, const V& end, const T& t)
{
	return start + (end - start) * t;
}

template <typename V, typename T>
__device__ inline V Clamp(const V& a, const T s, const T t){
	return V( PxMin(t, PxMax(s, a[0])),
			PxMin(t, PxMax(s, a[1])),
			PxMin(t, PxMax(s, a[2])) );
}

// Fluid solver (Adapted from FLEX)
__device__ inline float cube(PxReal x) { return x * x * x; }
__device__ inline float quart(PxReal x) { return x * x * x * x; }
__device__ inline float oct(PxReal x)
{
	PxReal x2 = x * x;
	PxReal x4 = x2 * x2;
	return x4*x4;
}


extern "C" __global__ void ps_stepParticlesLaunch(
	const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxU32 * PX_RESTRICT activeParticleSystems,
	const PxReal dt,
	const PxReal invTotalDt,
	bool isFirstIteration,
	const bool isTGS,
	const bool externalForcesEveryTgsIterationEnabled,
	const PxVec3 gravity)
{
	const PxgParticleSystem& particleSystem = particleSystems[blockIdx.y];

	const PxU32 numParticles = particleSystem.mCommonData.mNumParticles;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	const float4* PX_RESTRICT srcPositions = isFirstIteration ? particleSystem.mSortedOriginPos_InvMass : particleSystem.mSortedPositions_InvMass;
	float4* PX_RESTRICT positions = particleSystem.mSortedPositions_InvMass;
	float4* const PX_RESTRICT vels = particleSystem.mSortedVelocities;
	float4* PX_RESTRICT deltaP = particleSystem.mSortedDeltaP;
	
	const PxU32* const PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
	const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

	if (globalThreadIndex < numParticles)
	{
		const PxU32 phase = phases[globalThreadIndex];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		const PxReal cflVel = particleSystem.mData.mMaxVelocity;

		float4 tPos = srcPositions[globalThreadIndex];
		if (tPos.w != 0.f)
		{
			PxVec3 vel = PxLoad3(vels[globalThreadIndex]);

			bool velChanged = false;
			if(externalForcesEveryTgsIterationEnabled)
			{
				PX_ASSERT(isTGS);
				const PxReal damping = 1.f - PxMin(1.f, mat.damping * dt);
				vel = (vel + gravity*mat.gravityScale*dt) * damping;
				velChanged = true;
			}

			const PxReal velLimit = cflVel;


			if (vel.magnitudeSquared() > velLimit*velLimit)
			{
				//printf("contactDist = %f, velLimit = %f, vel = (%f, %f, %f)\n", particleSystem.mCommonData.mParticleContactDistance, velLimit, vel.x, vel.y, vel.z);
				vel = vel.getNormalized()*velLimit;
				velChanged = true;
				//vels[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);
			}

			PxVec3 dlta = isFirstIteration ? PxVec3(0.f) : PxLoad3(deltaP[globalThreadIndex]);

			PxVec3 oldPos(tPos.x, tPos.y, tPos.z);

			PxVec3 delta = vel * dt;

			PxVec3 pos = oldPos + delta;

			PxVec3 totalDelta = delta + dlta;

			//if (totalDelta.magnitudeSquared() > (cfl*cfl))
			//{
			//	PxVec3 oldTotalDelta = totalDelta;
			//	totalDelta = totalDelta.getNormalized() * cfl;
			//	
			//	//Clamp pos back into range
			//	pos += (totalDelta - oldTotalDelta);
			//	//Reset back to a reasonably approximated velocity.
			//	vel = totalDelta * invTotalDt;

			//	//vels[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);
			//	velChanged = true;
			//}

			if(velChanged)
				vels[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);

			//printf("***STEP*** pos = (%f, %f, %f), vel = (%f, %f, %f)\n", pos.x, pos.y, pos.z, vel.x, vel.y, vel.z);
			positions[globalThreadIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);
			deltaP[globalThreadIndex] = make_float4(totalDelta.x, totalDelta.y, totalDelta.z, tPos.w);
		
			/*if(globalThreadIndex==0)
			{
				printf("ps_stepParticlesLaunch:  %f, %f, %f, %f, dt: %f\n", pos.x, pos.y, pos.z, tPos.w, dt);
			}*/
		}
	}
}

extern "C" __global__ void ps_updateUnsortedArrayLaunch(const PxgParticleSystem * PX_RESTRICT particleSystems,
	const PxU32 * PX_RESTRICT activeParticleSystems)
{
	const PxU32 particleId = activeParticleSystems[blockIdx.z];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const PxU32 bufferIndex = blockIdx.y;

	if (bufferIndex < particleSystem.mCommonData.mNumParticleBuffers)
	{
		float4* PX_RESTRICT unsortedPositions = reinterpret_cast<float4*>(particleSystem.mUnsortedPositions_InvMass);
		float4* PX_RESTRICT unsortedVels = reinterpret_cast<float4*>(particleSystem.mUnsortedVelocities);
		float4* PX_RESTRICT unsortedRestArray = reinterpret_cast<float4*>(particleSystem.mRestArray);
		PxU32*  PX_RESTRICT unsortedPhases = particleSystem.mUnsortedPhaseArray;
		//PxU32* PX_RESTRICT activeIndices = particleSystem.mActiveArray;

		PxgParticleSimBuffer& buffer = particleSystem.mParticleSimBuffers[bufferIndex];
		const PxU32 offset = particleSystem.mParticleBufferRunsum[bufferIndex];

		const float4* particles = buffer.mPositionInvMasses;
		const float4* velocities = buffer.mVelocities;
		const float4* restPose = buffer.mRestPositions;
		const PxU32* phases = buffer.mPhases;
		

		const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

		const PxU32 flags = buffer.mFlags;

		if (globalThreadIndex < buffer.mNumActiveParticles)
		{
			const PxU32 ind = offset + globalThreadIndex;
			if (flags & PxParticleBufferFlag::eUPDATE_POSITION)
				unsortedPositions[ind] = particles[globalThreadIndex];

			if (flags & PxParticleBufferFlag::eUPDATE_VELOCITY)
				unsortedVels[ind] = velocities[globalThreadIndex];

			if (flags & PxParticleBufferFlag::eUPDATE_PHASE)
				unsortedPhases[ind] = phases[globalThreadIndex];

			if (flags & PxParticleBufferFlag::eUPDATE_RESTPOSITION && restPose)
				unsortedRestArray[ind] = restPose[globalThreadIndex];
	
			////We should be able to bin this array
			//activeIndices[ind] = ind;
		}
	}
}

extern "C" __global__ void ps_updateUserBufferLaunch(const PxgParticleSystem * PX_RESTRICT particleSystems,
	const PxU32 * PX_RESTRICT activeParticleSystems)
{
	const PxU32 particleId = activeParticleSystems[blockIdx.z];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const PxU32 bufferIndex = blockIdx.y;

	if (bufferIndex < particleSystem.mCommonData.mNumParticleBuffers)
	{
		const float4* PX_RESTRICT unsortedPositions = reinterpret_cast<float4*>(particleSystem.mUnsortedPositions_InvMass);
		const float4* PX_RESTRICT unsortedVels = reinterpret_cast<float4*>(particleSystem.mUnsortedVelocities);

		PxgParticleSimBuffer& buffer = particleSystem.mParticleSimBuffers[bufferIndex];
		const PxU32 offset = particleSystem.mParticleBufferRunsum[bufferIndex];

		float4* particles = buffer.mPositionInvMasses;
		float4* velocities = buffer.mVelocities;

		const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

		if (globalThreadIndex < buffer.mNumActiveParticles)
		{
			const PxU32 ind = offset + globalThreadIndex;
			particles[globalThreadIndex] = unsortedPositions[ind];
			velocities[globalThreadIndex] = unsortedVels[ind];
		}

		if (globalThreadIndex == 0)
			buffer.mFlags = 0;
	}
}

//This kernel is called before updateBound
extern "C" __global__ __launch_bounds__(1024, 1) void ps_preIntegrateLaunch(
	const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxU32* PX_RESTRICT activeParticleSystems, 
	const PxVec3 gravity,
	const PxReal dt,
	const bool isTGS, 
	const bool externalForcesEveryTgsIterationEnabled)
{

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	PxU32* PX_RESTRICT contactCounts = particleSystem.mOneWayContactCount;

	PxU32 numParticles = particleSystem.mCommonData.mNumParticles;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	float4* PX_RESTRICT positions = reinterpret_cast<float4*>(particleSystem.mUnsortedPositions_InvMass);
	float4* PX_RESTRICT vels = reinterpret_cast<float4*>(particleSystem.mUnsortedVelocities);
	float4* PX_RESTRICT oriPositions = reinterpret_cast<float4*>(particleSystem.mOriginPos_InvMass);

	float4* PX_RESTRICT accumDeltaP = reinterpret_cast<float4*>(particleSystem.mAccumDeltaP);

	const PxU32* const PX_RESTRICT phases = particleSystem.mUnsortedPhaseArray;
	const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

	
	if (globalThreadIndex < numParticles)
	{
		//const PxU32 activeIndex = activeIndices[globalThreadIndex];
		const PxU32 activeIndex = globalThreadIndex;
		const PxU32 phase = phases[activeIndex];

		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		
		const PxReal damping = 1.f - PxMin(1.f, mat.damping * dt);

		const PxVec3 tGravity = gravity * mat.gravityScale;

		float4 tPos = positions[activeIndex];
		PxVec3 vel(0.f);

		oriPositions[activeIndex] = tPos;

		PxVec3 delta(0.f);
		if (tPos.w != 0.f)
		{
			PxVec3 tVel = PxLoad3(vels[activeIndex]);

			PxVec3 oldPos(tPos.x, tPos.y, tPos.z);

			vel = (tVel + tGravity * dt) * damping;

			PxVec3 pos = oldPos;
						
			//PARTICLE_FORWARD_PROJECTION_STEP_SCALE_PGS = 1.0f
			//PARTICLE_FORWARD_PROJECTION_STEP_SCALE_TGS = 0.5f
			//Project forward only to the mid-point of the unconstrained projection for TGS. Technically, we should not 
			//project forward at all, but the behavior with fluids is not great if we do that. Projecting forward by a subset 
			//of the frame gets us a good indicative neighborhood based on the particles' motions and also allows various contact
			//gen methods to get good results

			const PxReal forwardProjectionStepScale = isTGS ? PARTICLE_FORWARD_PROJECTION_STEP_SCALE_TGS : PARTICLE_FORWARD_PROJECTION_STEP_SCALE_PGS;
			delta = vel * dt * forwardProjectionStepScale;
			//printf("pre integrate before gravity vel %i(%f, %f, %f), dt = %f\n", globalThreadIndex, vel.x, vel.y, vel.z, dt);

			//printf("%i: pre integrate before gravity pos(%f, %f, %f), pos = (%f, %f, %f)\n", globalThreadIndex, tPos.x, tPos.y, tPos.z, pos.x, pos.y, pos.z);
			
			pos += delta;

			positions[activeIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);
		}

		if(!externalForcesEveryTgsIterationEnabled)
		{
			vels[activeIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);
		}
		
		//initialize to zero
		accumDeltaP[globalThreadIndex] = make_float4(0.f, 0.f, 0.f, 0.f);
		contactCounts[globalThreadIndex] = 0;

		float2* PX_RESTRICT oneWayForces = particleSystem.mOneWayForces;
		for (PxU32 i = globalThreadIndex; i < (PxgParticleContactInfo::MaxStaticContactsPerParticle * numParticles); i += numParticles)
		{
			oneWayForces[i] = make_float2(0.0f, 0.0f);
		}
	}
	
}


//This kernel is called before updateBound
extern "C" __global__ __launch_bounds__(1024, 1) void ps_preDiffuseIntegrateLaunch(
	const PxgParticleSystem * PX_RESTRICT particleSystems,
	const PxU32 * PX_RESTRICT activeParticleSystems,
	const PxVec3 gravity, const PxReal dt)
{

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	PxU32 numParticles = *particleSystem.mNumDiffuseParticles;

	float4* PX_RESTRICT positions = reinterpret_cast<float4*>(particleSystem.mDiffusePosition_LifeTime);
	float4* PX_RESTRICT vels = reinterpret_cast<float4*>(particleSystem.mDiffuseVelocity);
	float4* PX_RESTRICT oriPositions = reinterpret_cast<float4*>(particleSystem.mDiffuseOriginPos_LifeTime);
	PxU32* PX_RESTRICT contactCounts = particleSystem.mDiffuseOneWayContactCount;

	if (globalThreadIndex < numParticles)
	{

		float4 tPos = positions[globalThreadIndex];

		oriPositions[globalThreadIndex] = tPos;

		PxVec3 tVel = PxLoad3(vels[globalThreadIndex]);

		PxVec3 oldPos(tPos.x, tPos.y, tPos.z);

		PxVec3 vel = (tVel + gravity * dt);

		const PxVec3 pos = oldPos + vel * dt;

		positions[globalThreadIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);

		vels[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);

		//reset count count. If contactCounts is null, this means solver is pbd
		if(contactCounts)
			contactCounts[globalThreadIndex] = 0;

	}

}

extern "C" __global__ __launch_bounds__(1024, 1) 
void ps_updateBoundFirstPassLaunch(const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxU32 id, const bool isTGS, PxBounds3* PX_RESTRICT bounds)
{
	const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND / WARP_SIZE;

	__shared__ PxReal sMax[3][numWarpsPerBlock];
	__shared__ PxReal sMin[3][numWarpsPerBlock];

	const PxU32 warpIndex = threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	const PxgParticleSystem& particleSystem = particleSystems[id];
	const PxU32 numDiffuseParticles = (particleSystem.mCommonData.mMaxDiffuseParticles > 0) ? *particleSystem.mNumDiffuseParticles : 0;
	const PxU32 numParticles = PxMax(particleSystem.mCommonData.mNumParticles, numDiffuseParticles);
	const PxU32 globalWarpIndex = blockIdx.x * blockDim.y + warpIndex;
	const PxU32 globalThreadIndex = threadIndexInWarp + WARP_SIZE * globalWarpIndex;

	const float4* PX_RESTRICT currentPositions = reinterpret_cast<float4*>(particleSystem.mOriginPos_InvMass);
	const float4* PX_RESTRICT predictedPositions = reinterpret_cast<float4*>(particleSystem.mUnsortedPositions_InvMass);
	const float4* PX_RESTRICT currentDiffusePositions = reinterpret_cast<float4*>(particleSystem.mDiffuseOriginPos_LifeTime);
	const float4* PX_RESTRICT predictedDiffusePositions = reinterpret_cast<float4*>(particleSystem.mDiffusePosition_LifeTime);

	PxVec3 minPos(PX_MAX_F32);
	PxVec3 maxPos(-PX_MAX_F32);

	if (globalThreadIndex < numParticles)
	{
		bool enableCCD = (particleSystem.mData.mFlags & PxParticleFlag::eENABLE_SPECULATIVE_CCD) > 0;
		if (globalThreadIndex < particleSystem.mCommonData.mNumParticles)
		{
			const PxVec3 predictedPos = PxLoad3(predictedPositions[globalThreadIndex]);
			if (enableCCD)
			{
				const PxVec3 currentPos = PxLoad3(currentPositions[globalThreadIndex]);
				PxVec3 cVolumePos;
				PxReal cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos, currentPos, predictedPos, 0.0f, false, isTGS);
				minPos = cVolumePos - PxVec3(cVolumeRadius);
				maxPos = cVolumePos + PxVec3(cVolumeRadius);
			}
			else
			{
				//using predicted pos, otherwise bounds based collision load balacing
				//might be missing cells with particles (and only load empty ones)
				minPos = predictedPos;
				maxPos = predictedPos;
			}
		}

		if (globalThreadIndex < numDiffuseParticles)
		{
			const PxVec3 predictedPos = PxLoad3(predictedDiffusePositions[globalThreadIndex]);
			if (enableCCD)
			{
				const PxVec3 currentPos = PxLoad3(currentDiffusePositions[globalThreadIndex]);
				PxVec3 cVolumePos;
				PxReal cVolumeRadius = getParticleSpeculativeContactVolume(cVolumePos, currentPos, predictedPos, 0.0f, true, isTGS);
				minPos = minPos.minimum(cVolumePos - PxVec3(cVolumeRadius));
				maxPos = maxPos.maximum(cVolumePos + PxVec3(cVolumeRadius));
			}
			else
			{
				//using predicted pos, otherwise bounds based collision load balacing
				//might be missing cells with particles (and only load empty ones)
				minPos = minPos.minimum(predictedPos);
				maxPos = maxPos.maximum(predictedPos);
			}
		}
	}

	PxReal max[3], min[3];
	min[0] = warpReduction<MinOpFloat, PxReal>(FULL_MASK, minPos.x);
	min[1] = warpReduction<MinOpFloat, PxReal>(FULL_MASK, minPos.y);
	min[2] = warpReduction<MinOpFloat, PxReal>(FULL_MASK, minPos.z);

	max[0] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxPos.x);
	max[1] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxPos.y);
	max[2] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxPos.z);

	//result store in the last thread in the warp 31
	if (threadIndexInWarp == 31)
	{
		sMin[0][warpIndex] = min[0];
		sMax[0][warpIndex] = max[0];

		sMin[1][warpIndex] = min[1];
		sMax[1][warpIndex] = max[1];

		sMin[2][warpIndex] = min[2];
		sMax[2][warpIndex] = max[2];
	}

	__syncthreads();

	PxBounds3& bound = bounds[blockIdx.x];

	float* minimun = reinterpret_cast<float*>(&bound.minimum.x);
	float* maximum = reinterpret_cast<float*>(&bound.maximum.x);
	if (warpIndex < 3)
	{
		PxReal newMin = warpReduction<MinOpFloat, PxReal>(FULL_MASK, sMin[warpIndex][threadIndexInWarp]);
		PxReal newMax = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, sMax[warpIndex][threadIndexInWarp]);

		//the final result will be in thread 31
		if (warpIndex < 3 && threadIndexInWarp == 31)
		{
			minimun[warpIndex] = newMin;
			maximum[warpIndex] = newMax;
		}
	}
}

extern "C" __global__ __launch_bounds__(1024, 1)
void ps_updateBoundSecondPassLaunch(
	PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxU32 id,
	const PxBounds3* PX_RESTRICT bounds,
	const PxU32 numBounds,
	PxBounds3* PX_RESTRICT boundArray,	//output, broad phase bound array
	const PxReal* PX_RESTRICT contactDists,
	const PxU32 elemIndex
)
{
	const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::UPDATEBOUND / WARP_SIZE;

	__shared__ PxReal sMax[3][numWarpsPerBlock];
	__shared__ PxReal sMin[3][numWarpsPerBlock];

	__shared__ PxReal sBound[sizeof(PxBounds3) / sizeof(PxReal)];
	PX_COMPILE_TIME_ASSERT(sizeof(PxBounds3) % sizeof(PxReal) == 0);

	PxReal* sMinMax = reinterpret_cast<float*>(&sBound);

	//In this kernel, we set off 32 warps and each warps has 32 thread
	const PxU32 warpIndex = threadIdx.y;
	const PxU32 threadIndexInWarp = threadIdx.x & (WARP_SIZE - 1);

	if (warpIndex == 0 && threadIndexInWarp < 3)
	{
		sMinMax[threadIndexInWarp] = PX_MAX_F32;
		sMinMax[threadIndexInWarp + 3] = -PX_MAX_F32;
	}

	__syncthreads();

	const PxU32 numIterations = (numBounds + PxgParticleSystemKernelBlockDim::UPDATEBOUND - 1) / PxgParticleSystemKernelBlockDim::UPDATEBOUND;

	PxReal min[3], max[3];
	for (PxU32 i = 0; i < numIterations; ++i)
	{
		const PxU32 elemIndex = threadIdx.x + warpIndex * WARP_SIZE + i * PxgParticleSystemKernelBlockDim::UPDATEBOUND;

		PxBounds3 bound;
		bound.setEmpty();
		if (elemIndex < numBounds)
		{
			bound = bounds[elemIndex];
		}

		min[0] = warpReduction<MinOpFloat, PxReal>(FULL_MASK, bound.minimum.x);
		min[1] = warpReduction<MinOpFloat, PxReal>(FULL_MASK, bound.minimum.y);
		min[2] = warpReduction<MinOpFloat, PxReal>(FULL_MASK, bound.minimum.z);

		max[0] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, bound.maximum.x);
		max[1] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, bound.maximum.y);
		max[2] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, bound.maximum.z);

		min[0] = __shfl_sync(FULL_MASK, min[0], 31);
		min[1] = __shfl_sync(FULL_MASK, min[1], 31);
		min[2] = __shfl_sync(FULL_MASK, min[2], 31);

		max[0] = __shfl_sync(FULL_MASK, max[0], 31);
		max[1] = __shfl_sync(FULL_MASK, max[1], 31);
		max[2] = __shfl_sync(FULL_MASK, max[2], 31);

		if (threadIndexInWarp == 31)
		{
			sMin[0][warpIndex] = min[0];
			sMax[0][warpIndex] = max[0];

			sMin[1][warpIndex] = min[1];
			sMax[1][warpIndex] = max[1];

			sMin[2][warpIndex] = min[2];
			sMax[2][warpIndex] = max[2];
		}

		__syncthreads();

		if (warpIndex < 3)
		{
			PxReal newMin = warpReduction<MinOpFloat, PxReal>(FULL_MASK, sMin[warpIndex][threadIndexInWarp]);
			PxReal newMax = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, sMax[warpIndex][threadIndexInWarp]);

			//the final result will be in thread 31
			if (warpIndex < 3 && threadIndexInWarp == 31)
			{
				sMinMax[warpIndex] = PxMin(sMinMax[warpIndex], newMin);
				sMinMax[warpIndex + 3] = PxMax(sMinMax[warpIndex + 3], newMax);
			}
		}

		__syncthreads(); //Required because the next loop iteation will write again into sMin and sMax
	}

	__syncthreads(); //sMinMax is written above and read below

	if (warpIndex == 0 && threadIndexInWarp < 6)
	{
		PxgParticleSystem& particleSystem = particleSystems[id];
		PxgParticleSystemData& data = particleSystem.mData;

		const PxReal contactDist = contactDists[elemIndex];
		float* resBound = reinterpret_cast<float*>(&boundArray[elemIndex].minimum.x);

		//compute min(0-2) max(3-5)
		float value = threadIndexInWarp < 3 ? (sMinMax[threadIndexInWarp] - contactDist) : (sMinMax[threadIndexInWarp] + contactDist);
		resBound[threadIndexInWarp] = value;

		PxU32 mask = 0x3F;
		//compute bound center
		float boundCenter = (__shfl_sync(mask, value, threadIndexInWarp + 3) + value) * 0.5f;
		
		float* pBoundCenter = reinterpret_cast<float*>(&data.mBoundCenter.x);
		if (threadIndexInWarp < 3)
		{
			pBoundCenter[threadIndexInWarp] = boundCenter;
			
			//printf("boundCenter[%i] %f\n", threadIndexInWarp, boundCenter);
		}
	}
}

extern "C" __global__ void ps_calculateHashLaunch(
	const PxgParticleSystem* PX_RESTRICT	particleSystems,
	const PxU32* PX_RESTRICT activeParticleSystems)
{
	//__shared__ PxBounds3 sBound;
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;
	const PxReal cellWidth = shParticleSystem.mCommonData.mGridCellWidth;

	PxU32* hashArray = particleSystem.mGridParticleHash;
	PxU32* particleIndex = particleSystem.mSortedToUnsortedMapping;

	const PxU32 index = threadIdx.x + blockIdx.x * blockDim.x;

	if (index >= numParticles)
		return;
	
	const float4* PX_RESTRICT pos_invmass = reinterpret_cast<float4*>(particleSystem.mUnsortedPositions_InvMass);

	uint3 gridSize = make_uint3(shParticleSystem.mCommonData.mGridSizeX, shParticleSystem.mCommonData.mGridSizeY, shParticleSystem.mCommonData.mGridSizeZ);

	const float4 pos = pos_invmass[index];

	int3 gridPos = calcGridPos(pos, cellWidth);
	
	PxU32 hashValue = calcGridHash(gridPos, gridSize);

	hashArray[index] = hashValue;
	particleIndex[index] = index;
}

extern "C" __global__ void ps_calculateHashForDiffuseParticlesLaunch(
	const PxgParticleSystem * PX_RESTRICT	particleSystems,
	const PxU32 * PX_RESTRICT				activeParticleSystems)
{

	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];
	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxU32 maxParticles = shParticleSystem.mCommonData.mMaxDiffuseParticles;
	const PxU32 numParticles = *shParticleSystem.mNumDiffuseParticles;
	const PxReal cellWidth = shParticleSystem.mCommonData.mGridCellWidth;

	const PxU32 index = threadIdx.x + blockIdx.x * blockDim.x;

	if (index >= maxParticles)
		return;

	PxU32* hashArray = shParticleSystem.mDiffuseGridParticleHash;
	PxU32* particleIndex = shParticleSystem.mDiffuseSortedToUnsortedMapping;

	particleIndex[index] = index;

	if (index >= numParticles)
	{
		hashArray[index] = EMPTY_CELL;
		return;
	}

	const float4* PX_RESTRICT posLife = reinterpret_cast<float4*>(particleSystem.mDiffusePosition_LifeTime);

	const uint3 gridSize = make_uint3(particleSystem.mCommonData.mGridSizeX, particleSystem.mCommonData.mGridSizeY, particleSystem.mCommonData.mGridSizeZ);

	const float4 pos = posLife[index];

	const int3 gridPos = calcGridPos(pos, cellWidth);
	const PxU32 hashValue = calcGridHash(gridPos, gridSize);

	hashArray[index] = hashValue;
}


extern "C" __global__ __launch_bounds__(1024,1) void ps_reorderDataAndFindCellStartLaunch(
	const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxU32 id
)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	__shared__ PxU32 sharedHash[PxgParticleSystemKernelBlockDim::UPDATEGRID + 1];
	
	const PxgParticleSystem& particleSystem = particleSystems[id];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxU32 index = threadIdx.x + blockIdx.x * blockDim.x;
	const bool isDiffuseParticlesThread = blockIdx.z == 1;

	PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;
	const PxU32* PX_RESTRICT gridParticleHash = shParticleSystem.mGridParticleHash;
	const PxU32* PX_RESTRICT gridParticleIndex = shParticleSystem.mSortedToUnsortedMapping;
	PxU32* PX_RESTRICT cellStart = shParticleSystem.mCellStart;
	PxU32* PX_RESTRICT cellEnd = shParticleSystem.mCellEnd;
	PxU32* PX_RESTRICT reverseLookup = shParticleSystem.mUnsortedToSortedMapping;

	if (isDiffuseParticlesThread)
	{
		// Early termination if there are no diffuse particles or is not using PBD
		if (shParticleSystem.mCommonData.mMaxDiffuseParticles == 0)
			return;

		numParticles = *shParticleSystem.mNumDiffuseParticles;
		gridParticleHash = shParticleSystem.mDiffuseGridParticleHash;
		gridParticleIndex = shParticleSystem.mDiffuseSortedToUnsortedMapping;
		cellStart = shParticleSystem.mDiffuseCellStart;
		cellEnd = shParticleSystem.mDiffuseCellEnd;
		reverseLookup = shParticleSystem.mDiffuseUnsortedToSortedMapping;

	}

	PxU32 hash;

	if (index < numParticles)
	{
		hash = gridParticleHash[index];

		// Load hash data into shared memory so that we can look
		// at neighboring particle's hash value without loading
		// two hash values per thread
		sharedHash[threadIdx.x + 1] = hash;

		if (index > 0 && threadIdx.x == 0)
		{
			// first thread in block must load neighbor particle hash
			sharedHash[0] = gridParticleHash[index - 1];
		}
	}

	__syncthreads();

	// common buffers
	const float4* PX_RESTRICT pos = reinterpret_cast<float4*>(shParticleSystem.mUnsortedPositions_InvMass);
	const float4* PX_RESTRICT vel = reinterpret_cast<float4*>(shParticleSystem.mUnsortedVelocities);

	float4* PX_RESTRICT sortedPos = reinterpret_cast<float4*>(shParticleSystem.mSortedPositions_InvMass);
	float4* PX_RESTRICT sortedVel0 = reinterpret_cast<float4*>(shParticleSystem.mSortedVelocities);

	const float4* PX_RESTRICT origin_pos = reinterpret_cast<float4*>(shParticleSystem.mOriginPos_InvMass);
	float4* PX_RESTRICT sortedOriginPos = reinterpret_cast<float4*>(shParticleSystem.mSortedOriginPos_InvMass);

	if (isDiffuseParticlesThread)
	{
		pos = reinterpret_cast<float4*>(shParticleSystem.mDiffusePosition_LifeTime);
		vel = reinterpret_cast<float4*>(shParticleSystem.mDiffuseVelocity);

		sortedPos = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedPos_LifeTime);
		sortedVel0 = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedVel);

		origin_pos = reinterpret_cast<float4*>(shParticleSystem.mDiffuseOriginPos_LifeTime);
		sortedOriginPos = reinterpret_cast<float4*>(shParticleSystem.mDiffuseSortedOriginPos_LifeTime);
	}
	
	// regular particles only
	
	const PxU32* PX_RESTRICT phases = shParticleSystem.mUnsortedPhaseArray;
	
	
	float4* PX_RESTRICT sortedDeltaP = reinterpret_cast<float4*>(shParticleSystem.mSortedDeltaP);
	PxU32* PX_RESTRICT sortedPhases = shParticleSystem.mSortedPhaseArray;

	if (index < numParticles)
	{
		// If this particle has a different cell index to the previous
		// particle then it must be the first particle in the cell,
		// so store the index of this particle in the cell.
		// As it isn't the first particle, it must also be the cell end of
		// the previous particle's cell

		// Now use the sorted index to reorder the pos and vel data
		PxU32 sortedIndex = gridParticleIndex[index];

		//printf("index %i sortedIndex %i\n", index, sortedIndex);

		const float4 p = pos[sortedIndex];
		const float4 v = vel[sortedIndex];

		const PxU32 otherHash = sharedHash[threadIdx.x];

		if (index == 0 || hash != otherHash)
		{
			cellStart[hash] = index;

			if (index > 0)
				cellEnd[otherHash] = index;
		}

		if (index == numParticles - 1)
		{
			cellEnd[hash] = index + 1;
		}

		sortedPos[index] = p;
		sortedVel0[index] = v;
		reverseLookup[sortedIndex] = index;


		const float4 orig = origin_pos[sortedIndex];
		sortedOriginPos[index] = orig;
		
		if (!isDiffuseParticlesThread)
		{

			sortedDeltaP[index] = make_float4(p.x - orig.x, p.y - orig.y, p.z - orig.z, p.w);
			sortedPhases[index] = phases[sortedIndex];
		}

	}
}

struct SelfCollideCache
{
	static const PxU32 MaxCachedContacts = 32;
	PxU32 selfCollisions[MaxCachedContacts][64];
};

// collide a particle against all other particles in a given cell
__device__ 
PxU32 collideCell(
	const PxU32	particleIndex0,
	const PxU32 startIndex,
	const PxU32 endIndex,
	const PxU32 maxNeighborhood,
	const PxReal contactDistanceSq,
	const PxReal filterDistanceSq,
	const PxVec3&	p0,
	const float4* const PX_RESTRICT sortedPose,
	const float4* const PX_RESTRICT restPositions,
	const PxgParticleSystem& sParticleSystem,
	const PxU32 stride, 
	PxU32 currentCount,
	PxU32* PX_RESTRICT collisionIndex,
	const PxU32 phase,
	const PxU32* const PX_RESTRICT sortedPhases,
	const PxVec3& restP,
	const bool selfCollideFilter,
	const PxU32* const PX_RESTRICT particleIndices,
	float2* PX_RESTRICT collisionForces
	, SelfCollideCache& selfCache
	)
{

	PxU32 numCollidedParticles = currentCount;
	
	if (startIndex != EMPTY_CELL)          // cell is not empty
	{
		// iterate over particles in this cell
		//PxU32 endIndex = cellEnd[gridHash];

		PxU32 index = currentCount * stride;

		bool selfCollide = PxGetSelfCollide(phase);

		PxU32 nextPhase =  sortedPhases[startIndex];
		float4 nextPos = fetch(&sortedPose[startIndex]);
		PxU32 nextRestIndex;
		if (selfCollideFilter)
			nextRestIndex = fetch(&particleIndices[startIndex]);

		for (PxU32 particleIndex1 = startIndex; particleIndex1 < endIndex && numCollidedParticles < maxNeighborhood; particleIndex1++)
		{
			const PxU32 phase2 = nextPhase;
			const float4 pos2 = nextPos;
			const PxU32 particleId = nextRestIndex;

			if ((particleIndex1 + 1) < endIndex)
			{
				nextPhase = sortedPhases[particleIndex1 + 1];
				nextPos = fetch(&sortedPose[particleIndex1+1]);
				if (selfCollideFilter)
					nextRestIndex = fetch(&particleIndices[particleIndex1+1]);
			}
			
			if (particleIndex1 != particleIndex0)                // check not colliding with self
			{
				const bool sameGroup = PxGetGroup(phase) == PxGetGroup(phase2);

				if (selfCollide || !sameGroup)
				{
					PxVec3 p1(pos2.x, pos2.y, pos2.z);

					// collide two spheres
					PxVec3 _delta = p0 - p1;

					const PxReal distanceSq = _delta.dot(_delta);

					if (distanceSq <= contactDistanceSq)
					{
						if (selfCollideFilter && sameGroup)
						{
							const PxVec3 restq = PxLoad3(fetch(&restPositions[particleId]));
							const PxVec3 diff = restq - restP;
							if (diff.magnitudeSquared() <= filterDistanceSq)
								continue;
						}

						//collisionIndex[index] = particleIndex1;
						
						if(numCollidedParticles < SelfCollideCache::MaxCachedContacts)
							selfCache.selfCollisions[numCollidedParticles][threadIdx.x] = particleIndex1;	
						else
							collisionIndex[index] = particleIndex1;
						index += stride;
						numCollidedParticles++;
					}
				}
			}
		}
	}

	return numCollidedParticles;
}

//This kernel just for PBD
extern "C" __global__ void ps_selfCollisionLaunch(
	const PxgParticleSystem* const PX_RESTRICT particleSystems, 
	const PxU32 id
)
{
	const PxgParticleSystem& sParticleSystem = particleSystems[id];

	const PxU32 globalThreadIndex = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 numParticles = sParticleSystem.mCommonData.mNumParticles;
	PxU32 totalCollidedParticles = 0;

	float2* PX_RESTRICT collisionForces = sParticleSystem.mDensityCollisionImpulses + globalThreadIndex;
	PxU32* PX_RESTRICT collisionIndices = sParticleSystem.mCollisionIndex + globalThreadIndex;

	__shared__ SelfCollideCache shContacts;

	if (globalThreadIndex < numParticles)
	{
		const PxReal cellWidth = sParticleSystem.mCommonData.mGridCellWidth;
		const PxReal cellWidthSq = cellWidth * cellWidth;

		const float4* const PX_RESTRICT sortedPose = reinterpret_cast<float4*>(sParticleSystem.mSortedPositions_InvMass);
		const PxU32* const PX_RESTRICT sortedPhase = sParticleSystem.mSortedPhaseArray;
		const float4* const PX_RESTRICT restPose = reinterpret_cast<float4*>(sParticleSystem.mRestArray);
		const PxU32* const PX_RESTRICT gridParticleIndex = sParticleSystem.mSortedToUnsortedMapping;
		PxU32* PX_RESTRICT particleSelfCollisionCounts = sParticleSystem.mParticleSelfCollisionCount;

		const PxU32 phase = sortedPhase[globalThreadIndex];

		const float4 pos = sortedPose[globalThreadIndex];
		PxVec3 p0(pos.x, pos.y, pos.z);

		int3 gridPos = calcGridPos(pos, cellWidth);

		const bool selfCollideFilter = PxGetSelfCollideFilter(phase);

		PxVec3 restp;
		if (selfCollideFilter)
			restp = PxLoad3(fetch(&restPose[gridParticleIndex[globalThreadIndex]]));


		//printf("idx %i gridPos(%i, %i, %i)\n", globalThreadIndex, gridPos.x, gridPos.y, gridPos.z);

		const PxU32* const PX_RESTRICT cellStart = sParticleSystem.mCellStart;
		const PxU32* const PX_RESTRICT cellEnd = sParticleSystem.mCellEnd;

		const PxReal filterDistanceSq = (2.01f*2.01f)*sParticleSystem.mData.mSolidRestOffset*sParticleSystem.mData.mSolidRestOffset;
		const PxU32 maxNeighborhood = sParticleSystem.mCommonData.mMaxNeighborhood;

		uint3 gridSize = make_uint3(sParticleSystem.mCommonData.mGridSizeX, sParticleSystem.mCommonData.mGridSizeY, sParticleSystem.mCommonData.mGridSizeZ);
		
		for (int z = -1; z <= 1; z++)
		{
			for (int y = -1; y <= 1; y++)
			{
				for (int x = -1; x <= 1; x++)
				{
					int3 neighbourPos = make_int3(gridPos.x + x, gridPos.y + y, gridPos.z + z);// gridPos + make_int3(x, y, z);

					PxU32 gridHash = calcGridHash(neighbourPos, gridSize);

					// get start of bucket for this cell
					PxU32 startIndex = cellStart[gridHash];

					if (startIndex != EMPTY_CELL)          // cell is not empty
					{
						PxU32 endIndex = cellEnd[gridHash];

						totalCollidedParticles = collideCell(globalThreadIndex, startIndex, endIndex, maxNeighborhood, cellWidthSq,
							filterDistanceSq, p0, sortedPose, restPose, sParticleSystem, numParticles,
							totalCollidedParticles, collisionIndices, phase, sortedPhase, restp, selfCollideFilter,
							gridParticleIndex, collisionForces, shContacts);
					}
				}
			}
		}

		particleSelfCollisionCounts[globalThreadIndex] = totalCollidedParticles;

	}
	__syncwarp();

	for (PxU32 i = 0, offset = 0; i < totalCollidedParticles; ++i, offset += numParticles)
	{
		if(i < SelfCollideCache::MaxCachedContacts)
			collisionIndices[offset] = shContacts.selfCollisions[i][threadIdx.x];
		collisionForces[offset] = make_float2(0.f, 0.f);
	}
}



extern "C" __global__ void ps_contactPrepareLaunch(
	PxgParticleSystem*				particleSystems,
	PxgParticlePrimitiveContact*	sortedContacts,
	PxU32*							numContacts,
	PxgParticleRigidContactBlock*	contactBlocks,
	PxgPrePrepDesc*					preDesc,
	PxgConstraintPrepareDesc*		prepareDesc,
	float2*							appliedForces,
	const PxReal					dt,
	const bool						isTGS,
	float4*							deltaVel,
	PxgSolverSharedDescBase*		sharedDesc
	)
{
	const PxU32 tNumContacts = *numContacts;
	
	PxAlignedTransform* bodyFrames = prepareDesc->body2WorldPool;

	PxU32* solverBodyIndices = preDesc->solverBodyIndices;
	PxgSolverBodyData* solverBodyData = prepareDesc->solverBodyDataPool;
	PxgSolverTxIData* solverDataTxIPool = prepareDesc->solverBodyTxIDataPool;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	PxgBodySim* bodySims = sharedDesc->mBodySimBufferDeviceData;
	
	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		appliedForces[workIndex] = make_float2(0.f, 0.f);

		PxgParticlePrimitiveContact contact = sortedContacts[workIndex];
		PxgParticleRigidContactBlock& block = contactBlocks[workIndex/32];

		PxU32 tIdx = workIndex&31;

		const PxU64 compressedId = contact.particleId;

		const PxU32 particleSystemId = PxGetParticleSystemId(compressedId);

		
		PxgParticleSystem& particleSystem = particleSystems[particleSystemId];
		const float4* const PX_RESTRICT position_invmass = reinterpret_cast<float4*>( particleSystem.mSortedPositions_InvMass);

		const PxU32 particleId = PxGetParticleIndex(compressedId);

		float4 pointInvM = position_invmass[particleId];

		const PxVec3 point(pointInvM.x, pointInvM.y, pointInvM.z);

		const PxVec3 normal(-contact.normal_pen.x, -contact.normal_pen.y, -contact.normal_pen.z);
		const PxReal pen = contact.normal_pen.w;// +normal.dot(delta);

		//nodeIndex
		const PxNodeIndex rigidId = reinterpret_cast<PxNodeIndex&>(contact.rigidId);
		PxU32 idx = 0;
		if (!rigidId.isStaticBody())
		{
			idx = solverBodyIndices[rigidId.index()];
		}

		if (rigidId.isArticulation())
		{
			PxU32 nodeIndexA = rigidId.index();

			PxU32 artiId = bodySims[nodeIndexA].articulationRemapId;

			PxgArticulation& articulation = sharedDesc->articulations[artiId];

			const PxU32 linkID = rigidId.articulationLinkId();
			const PxTransform body2World = articulation.linkBody2Worlds[linkID];

			const PxVec3 bodyFrame0p(body2World.p.x, body2World.p.y, body2World.p.z);

			PxSpatialMatrix& spatialResponse = articulation.spatialResponseMatrixW[linkID];

			//Pick two tangent vectors to the normal
			PxVec3 t0, t1;
			PxComputeBasisVectors(normal, t0, t1);

			PxVec3 ra = point - bodyFrame0p;
			PxVec3 raXn = ra.cross(normal);
			PxVec3 raXF0 = ra.cross(t0);
			PxVec3 raXF1 = ra.cross(t1);

			const Cm::UnAlignedSpatialVector deltaV0 = spatialResponse * Cm::UnAlignedSpatialVector(normal, raXn);
			const Cm::UnAlignedSpatialVector deltaV1 = spatialResponse * Cm::UnAlignedSpatialVector(t0, raXF0);
			const Cm::UnAlignedSpatialVector deltaV2 = spatialResponse * Cm::UnAlignedSpatialVector(t1, raXF1);

			const PxReal resp0 = deltaV0.top.dot(raXn) + deltaV0.bottom.dot(normal);
			const PxReal respF0 = deltaV0.top.dot(raXF0) + deltaV0.bottom.dot(t0);
			const PxReal respF1 = deltaV0.top.dot(raXF1) + deltaV0.bottom.dot(t1);

			block.raXn_resp[tIdx] = make_float4(raXn.x, raXn.y, raXn.z, resp0);
			block.normal_errorW[tIdx] = make_float4(normal.x, normal.y, normal.z, pen);
			block.fricTan0_invMass0[tIdx] = make_float4(t0.x, t0.y, t0.z, 1.f); //invMass0 is classed as 1 so that we record the impulse rather than deltaV!
			block.raXnF0_resp[tIdx] = make_float4(raXF0.x, raXF0.y, raXF0.z, respF0);
			block.raXnF1_resp[tIdx] = make_float4(raXF1.x, raXF1.y, raXF1.z, respF1);
			block.penBiasClampRigid[tIdx] = articulation.links[linkID].initialAngVelXYZ_penBiasClamp.w;
		}
		else
		{

			const float4 linVel_invMass0 = solverBodyData[idx].initialLinVelXYZ_invMassW;
			const PxReal invMass0 = linVel_invMass0.w;

			PxMat33 invSqrtInertia0;
			PxReal inertiaScale = 1.f;

			if (idx != 0 && invMass0 == 0.f)
			{
				invSqrtInertia0 = PxMat33(PxIdentity);
				inertiaScale = 0.f;
			}
			else
			{
				invSqrtInertia0 = solverDataTxIPool[idx].sqrtInvInertia;
			}

			PxAlignedTransform bodyFrame0 = bodyFrames[idx];
			const PxVec3 bodyFrame0p(bodyFrame0.p.x, bodyFrame0.p.y, bodyFrame0.p.z);

			//Pick two tangent vectors to the normal
			PxVec3 t0, t1;
			PxComputeBasisVectors(normal, t0, t1);

			PxVec3 ra = point - bodyFrame0p;
			PxVec3 raXn = ra.cross(normal);
			PxVec3 raXF0 = ra.cross(t0);
			PxVec3 raXF1 = ra.cross(t1);

			const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;
			const float resp0 = (raXnSqrtInertia.dot(raXnSqrtInertia))*inertiaScale + invMass0;

			const PxVec3 raXF0SqrtInertia = invSqrtInertia0 * raXF0;
			const PxVec3 raXF1SqrtInertia = invSqrtInertia0 * raXF1;

			const float respF0 = (raXF0SqrtInertia.dot(raXF0SqrtInertia))*inertiaScale + invMass0;
			const float respF1 = (raXF1SqrtInertia.dot(raXF1SqrtInertia))*inertiaScale + invMass0;

			block.raXn_resp[tIdx] = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, resp0);
			block.normal_errorW[tIdx] = make_float4(normal.x, normal.y, normal.z, pen);
			block.fricTan0_invMass0[tIdx] = make_float4(t0.x, t0.y, t0.z, invMass0);
			block.raXnF0_resp[tIdx] = make_float4(raXF0SqrtInertia.x, raXF0SqrtInertia.y, raXF0SqrtInertia.z, respF0);
			block.raXnF1_resp[tIdx] = make_float4(raXF1SqrtInertia.x, raXF1SqrtInertia.y, raXF1SqrtInertia.z, respF1);
			// Static -> idx 0 (world solver body, penBiasClamp -1e32 = unbounded); dynamic/kinematic -> their clamp.
			block.penBiasClampRigid[tIdx] = solverBodyData[idx].initialAngVelXYZ_penBiasClamp.w;
		}

		if (deltaVel)
		{
			deltaVel[workIndex] = make_float4(0.f);
			deltaVel[workIndex + tNumContacts] = make_float4(0.f);
		}
	}
}

// Particle adhesion as a normal-impulse lower bound (<= 0): within the adhesion radius it allows an
// attractive normal impulse that holds the particle to the collider. Returns 0 when the material has no
// adhesion. Standalone so every particle contact solve can share one adhesion model.
PX_FORCE_INLINE __device__ PxReal particleAdhesionImpulseBound(
	PxReal separation, PxReal adhesion, PxReal adhesionRadiusScale, PxReal restOffset,
	PxReal velMultiplier, PxReal invDt)
{
	if (adhesionRadiusScale <= 0.f || adhesion <= 0.f || separation >= adhesionRadiusScale)
		return 0.f;
	const PxReal adhesionRadius = restOffset * adhesionRadiusScale - restOffset;
	return -PxMax(0.f, adhesion * quart(1.f - separation / adhesionRadius) * PxMax(0.f, separation) * velMultiplier * invDt);
}

// Particle-vs-rigid contact solve. Participant types: PxgRigidPart (rigid), PxgDeformablePart<PxVec3>
// (particle, a single vertex with bc=(1,0,0) inline), PxgDbContactState (normal / tangent0 / initPen,
// filled here with the resolved single-tangent lambdas). Derives the scalar contact terms (mass-split
// velMultiplier/vmF0/1, normalVel, tanVel, normalDelta, tanDelta, fric0/1) from the rigid, then resolves
// the normal projection + friction cone + adhesion. Single function for PGS + TGS -- on PGS readVelocity
// zeros rigid.linDelta / rigid.angDelta. Also serves the bodyless particle-vs-static-mesh (one-way) path,
// which passes a nodeless-static rigid.
//
// This is solveRbDbContact with the deformable being a single-vertex particle, sharing its CN (velocity)
// units, mass-split and friction. PC-only: particle-material adhesion as a normal-impulse lower bound and
// the particle-only friction coefficient (no rigid-material combine).
PX_FORCE_INLINE __device__ void solvePcRbContact(
	PxgRigidPart& rigid,
	PxgDeformablePart<PxVec3>& particle,
	PxgDbContactState& state,
	PxReal frictionCoefficient,
	PxReal restOffset,
	PxReal adhesion,
	PxReal adhesionRadiusScale,
	bool isVelocityIteration,
	PxReal dt,
	float2& appliedForce
)
{
	const PxReal invMass1 = particle.vertexInvMasses.x; // single vertex: only .x is set
	const PxVec3& delta = particle.linDelta;

	const PxVec3& normal = state.normal;
	const PxVec3& fric0 = state.tangent0;
	const PxVec3 fric1 = normal.cross(fric0);
	const PxReal error = state.initPen;

	// Mass-splitting: scale the rigid unit responses by referenceCount (the PC contact count for
	// this rigid, from ps_queryPCContactReferenceCountLaunch), matching solveRbDbContact. This
	// softens each of the rigid's N simultaneous PC contacts by 1/N so the rigid is not over-driven;
	// referenceCount is 1 (a no-op) for static/kinematic rigids.
	const PxReal rigidRefCount = static_cast<PxReal>(rigid.referenceCount);
	const PxReal unitResponse_n = rigidRefCount * rigid.raXnResp + invMass1;
	const PxReal velMultiplier = (unitResponse_n > 0.f) ? (1.0f / unitResponse_n) : 0.f;
	const PxReal unitResponse_f0 = rigidRefCount * rigid.raXnF0Resp + invMass1;
	const PxReal vmF0 = (unitResponse_f0 > 0.f) ? (1.0f / unitResponse_f0) : 0.f;
	const PxReal unitResponse_f1 = rigidRefCount * rigid.raXnF1Resp + invMass1;
	const PxReal vmF1 = (unitResponse_f1 > 0.f) ? (1.0f / unitResponse_f1) : 0.f;

	const PxReal normalVel = rigid.linVel.dot(normal) + rigid.angVel.dot(rigid.raXn);
	const PxReal tanVel0   = rigid.linVel.dot(fric0)  + rigid.angVel.dot(rigid.raXnF0);
	const PxReal tanVel1   = rigid.linVel.dot(fric1)  + rigid.angVel.dot(rigid.raXnF1);
	const PxReal normalDelta = rigid.linDelta.dot(normal) + rigid.angDelta.dot(rigid.raXn);
	const PxReal tanDelta0   = rigid.linDelta.dot(fric0)  + rigid.angDelta.dot(rigid.raXnF0);
	const PxReal tanDelta1   = rigid.linDelta.dot(fric1)  + rigid.angDelta.dot(rigid.raXnF1);

	const PxReal invDt = 1.0f / dt;
	state.raXn = rigid.raXn;

	const PxReal separation = error - normal.dot(delta) + normalDelta;

	// CN: the contact-normal constraint in velocity units, matching solveRbDbContact. penBiasClamp
	// (= -maxDepenetrationVelocity) clamps the depenetration rate; the live-velocity term (normalVel)
	// contributes the position-phase bias only.
	// One-way static: rigid.penBiasClamp is the unbounded sentinel, so the particle's clamp governs.
	const PxReal penBiasClamp = PxMax(rigid.penBiasClamp, particle.penBiasClamp);
	const PxReal liveVelScale = isVelocityIteration ? 0.f : 1.f;
	const PxReal CN = PxMax(penBiasClamp, separation * invDt) + normalVel * liveVelScale;

	// Normal impulse (solveRbDbContact's deltaLambdaN form), with the adhesion pull as an extra lower bound.
	const PxReal clamp = PxMin(-appliedForce.x,
		particleAdhesionImpulseBound(separation, adhesion, adhesionRadiusScale, restOffset, velMultiplier, invDt));
	const PxReal deltaLambdaN = PxMax(clamp, -CN * velMultiplier);
	appliedForce.x = PxMax(0.f, appliedForce.x + deltaLambdaN);
	state.deltaLambdaN = deltaLambdaN;

	// Friction: RB-DB convention (solveRbDbContact) -- per-iteration Coulomb cone clamped by the current
	// normal impulse (no accumulation), tangent along the relative tangential velocity, magnitude from the
	// two per-tangent responses vmF0/vmF1 so the rigid's anisotropic angular response is kept.
	const PxReal CT0 = (tanDelta0 - fric0.dot(delta)) * invDt + tanVel0 * liveVelScale;
	const PxReal CT1 = (tanDelta1 - fric1.dot(delta)) * invDt + tanVel1 * liveVelScale;
	const PxVec3 relTanDelta = CT0 * fric0 + CT1 * fric1;
	const PxReal tanMagSq = relTanDelta.magnitudeSquared();
	if (tanMagSq > 1.0e-14f)
	{
		const PxReal CT = PxSqrt(tanMagSq);
		state.tangent = relTanDelta * (1.0f / CT);

		const PxReal frac0 = state.tangent.dot(fric0);
		const PxReal frac1 = state.tangent.dot(fric1);
		state.raXt = frac0 * rigid.raXnF0 + frac1 * rigid.raXnF1;

		const PxReal deltaLambdaT0 = CT0 * vmF0;
		const PxReal deltaLambdaT1 = CT1 * vmF1;
		state.deltaLambdaT = PxSqrt(deltaLambdaT0 * deltaLambdaT0 + deltaLambdaT1 * deltaLambdaT1);
		state.deltaLambdaT = -PxMin(state.deltaLambdaT, frictionCoefficient * PxAbs(deltaLambdaN));
	}
	else
	{
		state.tangent      = PxVec3(0.f);
		state.raXt         = PxVec3(0.f);
		state.deltaLambdaT = 0.f;
	}
}

// Per-rigid (per-articulation-link) count of PC contacts: solvePcRbContact down-weights the rigid by
// it, and the rigid writeback re-inflates each rigid delta by it. accumulateRigidDeltas then divides
// the summed delta by Sigma(count), and that division cancels the inflation only when this count
// equals the writeback's count -- so both count every contact on a dynamic rigid (the same set).
extern "C" __global__ void ps_queryPCContactReferenceCountLaunch(
	PxgParticleSystem*							particleSystems,
	PxgParticlePrimitiveContact*				sortedContacts,
	PxgParticleRigidContactBlock*				contactBlocks,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	float4*										solverBodyVelPool,
	PxU32*										rigidRefCount,			//output
	PxReal										dt,
	bool										isTGS,
	PxgArticulationCoreDesc*					artiCoreDesc
)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 maxLinksPerArticulation = artiCoreDesc->mMaxLinksPerArticulation;

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= tNumContacts)
			return;

		const PxU32 tIdx = workIndex & 31;

		PxgParticleRigidContactBlock& block = contactBlocks[workIndex / 32];

		const PxU64 tRigidId = sortedContacts[workIndex].rigidId;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		PxgRigidPart rigid;
		rigid.readContactPrep(block, tIdx);

		// Dynamic rigids only (static => id -1). Particle mass is irrelevant: a kinematic (invMass == 0)
		// particle does not move but still pushes the rigid, so it is counted like any other.
		const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, maxLinksPerArticulation);
		if (globalRigidBodyId != -1 && rigid.invMass != 0.0f)
		{
			atomicAdd(&rigidRefCount[globalRigidBodyId], 1);
		}
	}
}

//solve collision between particles and primitives based on the sorted contacts by particleId
//store new velocity to particle
extern "C" __global__ void ps_solvePCOutputParticleDeltaVLaunch(
	PxgParticleSystem*							particleSystems,
	PxgParticlePrimitiveContact*				sortedContacts,
	PxgParticleRigidContactBlock*				contactBlocks,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	float4*										solverBodyVelPool,
	float4*										deltaPos, //output
	float2*										appliedForces,
	PxU32*										rigidRefCount,
	PxReal										dt,
	bool										isVelocityIteration,
	PxgArticulationCoreDesc*					artiCoreDesc
)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 maxLinksPerArticulation = artiCoreDesc->mMaxLinksPerArticulation;
	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= tNumContacts)
			return;

		const PxU32 tIdx = workIndex&31;

		float2 appliedForce = appliedForces[workIndex];

		PxgParticleRigidContactBlock& block = contactBlocks[workIndex / 32];
		PxU64 tParticleId = sortedContacts[workIndex].particleId;
		const PxU32 particleSystemId = PxGetParticleSystemId(tParticleId);
		const PxU32 particleId = PxGetParticleIndex(tParticleId);

		PxgParticleSystem& particleSystem = particleSystems[particleSystemId];
		const float4* PX_RESTRICT sortedDeltaP_invMassW = reinterpret_cast<float4*>( particleSystem.mSortedDeltaP);
		const PxU32* const PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
		const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

		const PxU32 phase = phases[particleId];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		float4 deltaP_invMassW = sortedDeltaP_invMassW[particleId];
		const PxReal invMass1 = deltaP_invMassW.w;

		//If the particle has infinite mass, the particle need not respond to the collision
		if(invMass1 == 0.f)
			continue;

		const PxU64 tRigid = sortedContacts[workIndex].rigidId;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigid);

		PxgRigidPart rigid;
		// Symmetric PC mass-split: scale the rigid response by its PC contact count (referenceCount),
		// exactly as ps_solvePCOutputRigidDeltaVLaunch, so both solve halves compute the identical lambda
		// and the impulse stays equal-and-opposite.
		const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, maxLinksPerArticulation);
		const float rigidInvMass0 = block.fricTan0_invMass0[tIdx].w;
		rigid.readBodyProperties(rigidId, globalRigidBodyId, rigidInvMass0, rigidRefCount, /*material*/ nullptr);
		rigid.readContactPrep(block, tIdx);
		rigid.readVelocity(velocityReader, rigidId, /*isTGS*/ false);

		PxgDeformablePart<PxVec3> particle;
		// PC ignores the particle-side multi-contact inflation that the deformable-particle solves apply;
		// only the rigid-side referenceCount carries the mass-split.
		particle.linDelta = PxVec3(deltaP_invMassW.x, deltaP_invMassW.y, deltaP_invMassW.z);
		particle.vertexInvMasses = PxVec3(invMass1, 0.0f, 0.0f);
		particle.bc = PxVec3(1.0f, 0.0f, 0.0f);
		particle.penBiasClamp = particleSystem.mData.mPenBiasClamp;

		PxgDbContactState state;
		state.readContactPrep(block, tIdx);

		solvePcRbContact(rigid, particle, state,
						 mat.friction, particleSystem.mData.mRestOffset, mat.adhesion, mat.adhesionRadiusScale,
						 /*isVelocityIteration*/ false, dt, appliedForce);

		// Particle delta: equal-and-opposite to the rigid feedback (writeContactDeltas).
		const PxVec3 deltaLinVel = -(state.normal * state.deltaLambdaN + state.tangent * state.deltaLambdaT) * invMass1 * dt;
		PxReal scale = state.deltaLambdaN != 0.f ? 1.f : 0.f;

		deltaPos[workIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, scale);
		appliedForces[workIndex] = appliedForce;
	}
}

//solve collision between particles and primitives based on the sorted contacts by particleId
//store new velocity to particle
extern "C" __global__ void ps_solvePCOutputParticleDeltaVTGSLaunch(
	PxgParticleSystem*							particleSystems,
	PxgParticlePrimitiveContact*				sortedContacts,
	PxgParticleRigidContactBlock*				contactBlocks,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	float4*										solverBodyVelPool,
	float4*										deltaPos, //output
	float2*										appliedForces,
	PxU32*										rigidRefCount,
	PxReal										dt,
	bool										isVelocityIteration,
	PxgArticulationCoreDesc*					artiCoreDesc
)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 maxLinksPerArticulation = artiCoreDesc->mMaxLinksPerArticulation;

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);
	
	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= tNumContacts)
			return;

		const PxU32 tIdx = workIndex & 31;

		float2 appliedForce = appliedForces[workIndex];

		PxgParticleRigidContactBlock& block = contactBlocks[workIndex / 32];
		PxU64 tParticleId = sortedContacts[workIndex].particleId;

		const PxU32 particleSystemId = PxGetParticleSystemId(tParticleId);
		const PxU32 particleId = PxGetParticleIndex(tParticleId);

		PxgParticleSystem& particleSystem = particleSystems[particleSystemId];
		const float4* PX_RESTRICT sortedDeltaP_invMassW = reinterpret_cast<float4*>(particleSystem.mSortedDeltaP);
		const PxU32* const PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
		const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;
		
		const PxU32 phase = phases[particleId];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		float4 deltaP_invMassW = sortedDeltaP_invMassW[particleId];
		const PxReal invMass1 = deltaP_invMassW.w;

		//If the particle has infinite mass, the particle need not respond to the collision
		if (invMass1 == 0.f)
			continue;

		const PxU64 tRigidId = sortedContacts[workIndex].rigidId;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		PxgRigidPart rigid;
		// Symmetric PC mass-split: scale the rigid response by its PC contact count (referenceCount),
		// exactly as ps_solvePCOutputRigidDeltaVTGSLaunch, so both solve halves compute the identical lambda
		// and the impulse stays equal-and-opposite.
		const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, maxLinksPerArticulation);
		const float rigidInvMass0 = block.fricTan0_invMass0[tIdx].w;
		rigid.readBodyProperties(rigidId, globalRigidBodyId, rigidInvMass0, rigidRefCount, /*material*/ nullptr);
		rigid.readContactPrep(block, tIdx);
		rigid.readVelocity(velocityReader, rigidId, /*isTGS*/ true);

		PxgDeformablePart<PxVec3> particle;
		// PC ignores the particle-side multi-contact inflation that the deformable-particle solves apply;
		// only the rigid-side referenceCount carries the mass-split.
		particle.linDelta = PxVec3(deltaP_invMassW.x, deltaP_invMassW.y, deltaP_invMassW.z);
		particle.vertexInvMasses = PxVec3(invMass1, 0.0f, 0.0f);
		particle.bc = PxVec3(1.0f, 0.0f, 0.0f);
		particle.penBiasClamp = particleSystem.mData.mPenBiasClamp;

		PxgDbContactState state;
		state.readContactPrep(block, tIdx);

		solvePcRbContact(rigid, particle, state,
						 mat.friction, particleSystem.mData.mRestOffset, mat.adhesion, mat.adhesionRadiusScale,
						 isVelocityIteration, dt, appliedForce);

		// Particle delta: equal-and-opposite to the rigid feedback (writeContactDeltas).
		PxVec3 deltaLinVel = -(state.normal * state.deltaLambdaN + state.tangent * state.deltaLambdaT) * invMass1 * dt;
		PxReal scale = state.deltaLambdaN != 0.f ? 1.f : 0.f;

		deltaPos[workIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, scale);
		appliedForces[workIndex] = appliedForce;
	}
}

extern "C" __global__ void ps_solveOneWayContactDeltaVLaunch(
	PxgParticleSystem*							particleSystems,
	const PxU32*								activeParticleSystems,
	const PxReal								invDt,
	const PxReal								dt,
	const bool									isVelocityIteration
)
{
	//Launch y as the number of particle systems!
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	PxgParticleSystem& particleSystem = particleSystems[particleId];

	uint2* sParticleSystem = reinterpret_cast<uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;


	if (groupThreadIdx < numParticles)
	{
		const PxgParticleContactInfo* PX_RESTRICT contacts = shParticleSystem.mOneWayContactInfos;
		const PxU32* PX_RESTRICT contactCounts = shParticleSystem.mOneWayContactCount;

		const PxU32* const PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
		const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;
		
		const PxU32 phase = phases[groupThreadIdx];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);
		const PxReal frictionCoefficient = mat.friction;
		const PxReal adhesionRadiusScale = mat.adhesionRadiusScale;
		const PxReal adhesion = mat.adhesion;
		const PxReal restOffset = particleSystem.mData.mRestOffset;

		float2* PX_RESTRICT appliedForces = shParticleSystem.mOneWayForces;
		float4* PX_RESTRICT accumDeltaP = reinterpret_cast<float4*>(shParticleSystem.mAccumDeltaP);
		const float4* const PX_RESTRICT deltaP = reinterpret_cast<float4*>(shParticleSystem.mSortedDeltaP);

		const PxU32 contactCount = PxMin(PxgParticleContactInfo::MaxStaticContactsPerParticle, contactCounts[groupThreadIdx]);

		if (contactCount)
		{
			float4 dp_ = deltaP[groupThreadIdx];
			const PxVec3 dp(dp_.x, dp_.y, dp_.z); // particle position delta (mSortedDeltaP)

			PxReal denom = 0.0f;

			float invMass = dp_.w;
			PxVec3 thisDeltaP(0.f);
			for (PxU32 c = 0, offset = groupThreadIdx; c < contactCount; ++c, offset += numParticles)
			{
				const PxgParticleContactInfo& contact = contacts[offset];
				float2 appliedForce = appliedForces[offset];

				const float4 normalPenW = contact.mNormal_PenW;
				// normal is flipped to be the same as for constraints.
				PxVec3 normal(-normalPenW.x, -normalPenW.y, -normalPenW.z);

				//Pick the friction basis tangent (the adapter derives the second as normal x tangent0).
				PxVec3 tangent0, tangent1;
				PxComputeBasisVectors(normal, tangent0, tangent1);

				// Route the bodyless static-mesh contact through the particle-rigid adapter with a nodeless
				// static rigid. Only the particle delta is produced -- the static collider has no node, so
				// there is no rigid writeback.
				PxgRigidPart rigid;
				rigid.readNodelessStatic();

				PxgDeformablePart<PxVec3> particle;
				particle.linDelta = dp;
				particle.vertexInvMasses = PxVec3(invMass, 0.f, 0.f);
				particle.bc = PxVec3(1.f, 0.f, 0.f);
				particle.penBiasClamp = shParticleSystem.mData.mPenBiasClamp;

				PxgDbContactState state;
				state.normal   = normal;
				state.tangent0 = tangent0;
				state.initPen  = normalPenW.w;

				solvePcRbContact(rigid, particle, state,
								 frictionCoefficient, restOffset, adhesion, adhesionRadiusScale,
								 isVelocityIteration, dt, appliedForce);

				// Particle delta: equal-and-opposite to the resolved feedback, as a position correction.
				PxVec3 deltaLinVel = -(state.normal * state.deltaLambdaN + state.tangent * state.deltaLambdaT) * invMass * dt;
				PxReal scale = state.deltaLambdaN != 0.f ? 1.f : 0.f;

				thisDeltaP += deltaLinVel;

				denom += scale;
				appliedForces[offset] = appliedForce;
			}

			if (denom != 0.0f)
				thisDeltaP *= 1.f / denom;

			float4 delta = accumDeltaP[groupThreadIdx];
			delta.x += thisDeltaP.x; delta.y += thisDeltaP.y; delta.z += thisDeltaP.z;
			accumDeltaP[groupThreadIdx] = delta;
		}
	}
}


//solve collision between particles and primitives based on the sorted contact by rigid id
//store new velocity to rigid body buffer
extern "C" __global__ void ps_solvePCOutputRigidDeltaVLaunch(
	PxgParticleSystem*							particleSystems,
	PxgParticlePrimitiveContact*				sortedContacts,
	PxgParticleRigidContactBlock*				contactBlocks,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	float4*										solverBodyVelPool,
	float4*										deltaVel,				//output
	float2*										appliedForces,
	PxU32*										rigidRefCount,
	PxReal										dt,
	bool										isVelocityIteration,
	PxgArticulationCoreDesc*					artiCoreDesc
)

{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 maxLinksPerArticulation = artiCoreDesc->mMaxLinksPerArticulation;
	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		PxgParticleRigidContactBlock& block = contactBlocks[workIndex / 32];
		const PxU32 tIdx = workIndex & 31;

		const PxU64 tRigidId = sortedContacts[workIndex].rigidId;

		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		//TODO - need to figure out how to make this work for articulation links!
		if (rigidId.isStaticBody())
		{
			continue;
		}

		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[tIdx];
		if (fricTan0_invMass0.w == 0.f)
		{
			deltaVel[workIndex] = make_float4(0.0f);
			deltaVel[workIndex + tNumContacts] = make_float4(0.0f);
			appliedForces[workIndex] = make_float2(0.0f);
			continue;
		}

		const PxU64 tParticleId = sortedContacts[workIndex].particleId;

		const PxU32 particleSystemId = PxGetParticleSystemId(tParticleId);
		const PxU32 particleId = PxGetParticleIndex(tParticleId);

		const PxgParticleSystem& particleSystem = particleSystems[particleSystemId];
		const float4* const PX_RESTRICT sortedDeltaP_invMassW = reinterpret_cast<float4*>(particleSystem.mSortedDeltaP);
		const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
		const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

		const PxU32 phase = phases[particleId];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		const float4 delta4 = sortedDeltaP_invMassW[particleId];
		const PxReal invMass1 = delta4.w;

		float2 appliedForce = appliedForces[workIndex];

		PxgRigidPart rigid;
		// PC friction is particle-material based (passed to solvePcRbContact), so no rigid material.
		const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, maxLinksPerArticulation);
		rigid.readBodyProperties(rigidId, globalRigidBodyId, fricTan0_invMass0.w, rigidRefCount, /*material*/ nullptr);
		rigid.readContactPrep(block, tIdx);
		rigid.readVelocity(velocityReader, rigidId, /*isTGS*/ false);

		PxgDeformablePart<PxVec3> particle;
		// Raw particle invMass on the rigid side (no inflation): the rigid side's mass-split is the
		// rigid referenceCount applied to the unit responses in solvePcRbContact + the integer-count
		// writeback below.
		particle.linDelta = PxLoad3(delta4);
		particle.vertexInvMasses = PxVec3(invMass1, 0.0f, 0.0f);
		particle.bc = PxVec3(1.0f, 0.0f, 0.0f);
		particle.penBiasClamp = particleSystem.mData.mPenBiasClamp;

		PxgDbContactState state;
		state.readContactPrep(block, tIdx);

		solvePcRbContact(rigid, particle, state,
						 mat.friction, particleSystem.mData.mRestOffset, mat.adhesion, mat.adhesionRadiusScale,
						 /*isVelocityIteration*/ false, dt, appliedForce);

		// Rigid feedback through the shared writeContactDeltas (same path as cloth/softbody).
		rigid.writeContactDeltas(deltaVel, rigidId, state, workIndex, workIndex + tNumContacts);
		appliedForces[workIndex] = appliedForce;
	}
}

extern "C" __global__ void ps_solvePCOutputRigidDeltaVTGSLaunch(
	PxgParticleSystem*							particleSystems,
	PxgParticlePrimitiveContact*				sortedContacts,
	PxgParticleRigidContactBlock*				contactBlocks,
	PxU32*										numContacts,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	float4*										solverBodyVelPool,
	float4*										deltaVel,				//output
	float2*										appliedForces,
	PxU32*										rigidRefCount,
	PxReal										dt,
	bool										isVelocityIteration,
	PxgArticulationCoreDesc*					artiCoreDesc
)

{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 maxLinksPerArticulation = artiCoreDesc->mMaxLinksPerArticulation;

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		PxgParticleRigidContactBlock& block = contactBlocks[workIndex / 32];
		const PxU32 tIdx = workIndex & 31;

		const PxU64 tRigidId = sortedContacts[workIndex].rigidId;

		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		//TODO - need to figure out how to make this work for articulation links!
		if (rigidId.isStaticBody())
		{
			continue;
		}

		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[tIdx];
		if (fricTan0_invMass0.w == 0.f)
		{
			deltaVel[workIndex] = make_float4(0.0f);
			deltaVel[workIndex + tNumContacts] = make_float4(0.0f);
			appliedForces[workIndex] = make_float2(0.0f);
			continue;
		}

		const PxU64 tParticleId = sortedContacts[workIndex].particleId;
		const PxU32 particleSystemId = PxGetParticleSystemId(tParticleId);
		const PxU32 particleId = PxGetParticleIndex(tParticleId);

		const PxgParticleSystem& particleSystem = particleSystems[particleSystemId];
		const float4* const PX_RESTRICT sortedDeltaP_invMassW = reinterpret_cast<float4*>(particleSystem.mSortedDeltaP);
		const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
		const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;
		
		const PxU32 phase = phases[particleId];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		const float4 delta4 = sortedDeltaP_invMassW[particleId];
		const PxReal invMass1 = delta4.w;

		float2 appliedForce = appliedForces[workIndex];

		PxgRigidPart rigid;
		// PC friction is particle-material based (passed to solvePcRbContact), so no rigid material.
		const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, maxLinksPerArticulation);
		rigid.readBodyProperties(rigidId, globalRigidBodyId, fricTan0_invMass0.w, rigidRefCount, /*material*/ nullptr);
		rigid.readContactPrep(block, tIdx);
		rigid.readVelocity(velocityReader, rigidId, /*isTGS*/ true);

		PxgDeformablePart<PxVec3> particle;
		// Raw particle invMass on the rigid side (no inflation): the rigid side's mass-split is the
		// rigid referenceCount applied to the unit responses in solvePcRbContact + the integer-count
		// writeback below.
		particle.linDelta = PxLoad3(delta4);
		particle.vertexInvMasses = PxVec3(invMass1, 0.0f, 0.0f);
		particle.bc = PxVec3(1.0f, 0.0f, 0.0f);
		particle.penBiasClamp = particleSystem.mData.mPenBiasClamp;

		PxgDbContactState state;
		state.readContactPrep(block, tIdx);

		solvePcRbContact(rigid, particle, state,
						 mat.friction, particleSystem.mData.mRestOffset, mat.adhesion, mat.adhesionRadiusScale,
						 isVelocityIteration, dt, appliedForce);

		// Rigid feedback through the shared writeContactDeltas (same path as cloth/softbody).
		rigid.writeContactDeltas(deltaVel, rigidId, state, workIndex, workIndex + tNumContacts);
		appliedForces[workIndex] = appliedForce;
	}
}


extern "C" __global__ void ps_findStartEndParticleFirstLaunch(
	PxgParticleSystem*							particleSystems,
	PxgParticlePrimitiveContact*				sortedContacts,
	PxU32*										numContacts,
	PxU32*										blockOffsets,
	PxU32*										offsets
)
{
	__shared__ PxU64 sParticleId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	//numWarpsPerBlock can't be larger than 32
	const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA / WARP_SIZE;

	__shared__ PxU32 sWarpAccumulator[WARP_SIZE];
	__shared__ PxU32 sBlockAccumulator;

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	//gridDim should be 64
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	PxU32 warpIndex = threadIdx.x / (WARP_SIZE);
	const PxU32 idx = threadIdx.x;

	if (threadIdx.x == 0)
	{
		sBlockAccumulator = 0;
	}

	__syncthreads();

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		
		PxU64 particleId = 0xffffffffffffffff;
		if (workIndex < tNumContacts)
		{
			PxgParticlePrimitiveContact& contact = sortedContacts[workIndex];
			particleId = contact.particleId; //include particle system id

			//printf("%i: particleId = %i\n", workIndex, particleId);

			// Load particle id into shared memory so that we can look
			// at neighboring particle id without loading
			// two particle id per thread
			sParticleId[threadIdx.x + 1] = particleId;

			if (workIndex > 0 && threadIdx.x == 0)
			{
				// first thread in block must load neighbor particle 
				PxgParticlePrimitiveContact& contact = sortedContacts[workIndex - 1];
				sParticleId[0] = contact.particleId;
			}
		}

		__syncthreads();

		PxU32 isRangeStart = 0;
		if (workIndex < tNumContacts)
		{
			// If this particle has a different particle index to the previous
			// particle then it must be the first particle in the cell,
			// so store the index of this particle in the cell.
			// As it isn't the first particle, it must also be the cell end of
			// the previous particle's cell
			if (workIndex == 0 || particleId != sParticleId[threadIdx.x])
				isRangeStart = 1;
		}

		const PxU32 threadMask = (1 << threadIndexInWarp) - 1;
		const PxU32 mask = __ballot_sync(FULL_MASK, isRangeStart);
		//Offset is the sum of all the preceding offsets. However, if I am not a range start,
		//then I remove 1 entry to ensure that I get the same results as the preceding element.
		PxI32 offset = __popc(mask & threadMask) - (1 - isRangeStart);

		/*if (workIndex < tNumContacts)
		{
			printf("workIndex %i offset %i isRangeStart %i\n", workIndex, offset, isRangeStart);
		}*/
	
		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			sWarpAccumulator[warpIndex] = __popc(mask);
			/*if(blockIdx.x == 0)
				printf("sWarpAcculator[%i] = %i blockIdx.x %i\n", warpIndex, sWarpAccumulator[warpIndex], blockIdx.x);*/
		}

		const PxU32 prevBlockAccumulator = sBlockAccumulator;

		__syncthreads();

		if (warpIndex == 0)
		{
			PxU32 tOffset = threadIndexInWarp < numWarpsPerBlock ? sWarpAccumulator[threadIndexInWarp] : 0;

			const PxU32 output = warpScan<AddOpPxU32, PxU32>(FULL_MASK, tOffset) - tOffset;

			if (threadIndexInWarp == (WARP_SIZE - 1))
				sBlockAccumulator += (output + tOffset);

			sWarpAccumulator[threadIndexInWarp] = output;
		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			//Now output both offsets...
			offsets[workIndex] = offset + prevBlockAccumulator + sWarpAccumulator[warpIndex];
			
		}
	}

	if (threadIdx.x == 0)
	{
		blockOffsets[blockIdx.x] = sBlockAccumulator;
	}
}

//32 blocks. Each block compute the exclusive ransum for the blockOffset
extern "C" __global__ void ps_findStartEndParticleSecondLaunch(
	PxgParticlePrimitiveContact*	sortedContacts,			//input
	PxU32*							numContacts,			//input
	PxU32*							blockOffsets,			//input
	PxU32*							offsets,				//input
	PxU32*							totalNumPairs,			//output
	PxU32*							rangeStart,				//output
	PxU32*							rangeEnd				//output
)
{
	__shared__ PxU32 sBlockAccum[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sParticleId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];
	__shared__ PxU32 sTotalPairs;

	const PxU32 tNumContacts = *numContacts;
	const PxU32 idx = threadIdx.x;

	PxU32 val = 0;
	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
		val = blockOffsets[idx];

	PxU32 res = warpScan<AddOpPxU32, PxU32>(FULL_MASK, val) - val;

	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
		sBlockAccum[idx] = res;

	if (idx == (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1))
		sTotalPairs = res + val;

	const PxU32 totalBlockRequired = (tNumContacts + (PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;

	__syncthreads();

	PxU32 blockAccum = sBlockAccum[blockIdx.x];
	const PxU32 startBoundaryElement = numIterationPerBlock * blockDim.x;

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU64 particleId;
		if (workIndex < tNumContacts)
		{
			//printf("tNumContacts %i\n", tNumContacts);

			PxgParticlePrimitiveContact& contact = sortedContacts[workIndex];
			particleId = contact.particleId; //include particle system id

			// Load particle id into shared memory so that we can look
			// at neighboring particle id without loading
			// two particle id per thread
			sParticleId[threadIdx.x + 1] = particleId;

			if (workIndex > 0 && threadIdx.x == 0)
			{
				// first thread in block must load neighbor particle 
				PxgParticlePrimitiveContact& contact = sortedContacts[workIndex - 1];
				sParticleId[0] = contact.particleId;
			}
		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			// If this particle has a different particle index to the previous
			// particle then it must be the first particle in the cell,
			// so store the index of this particle in the cell.
			// As it isn't the first particle, it must also be the cell end of
			// the previous particle's cell
			if (workIndex == 0 || particleId != sParticleId[threadIdx.x])
			{
				
				const PxU32 startOffset = offsets[workIndex] + blockAccum;
				rangeStart[startOffset] = workIndex;
				//printf("workIndex %i rangeStart[%i] = %i\n", workIndex, startOffset, workIndex);

				if (workIndex > 0)
				{
					
					PxU32 tBlockAccum = blockAccum;

					//boundary element between block
					if(workIndex % startBoundaryElement == 0)
						tBlockAccum = sBlockAccum[blockIdx.x - 1];

					////boundary element between block
					//if (workIndex % blockDim.x == 0)
					//	preBlockAccum = sBlockAccum[blockIdx.x - 1];

					//using this thread to write to workIndex - 1's end range
					const PxU32 endOffset = offsets[workIndex - 1] + tBlockAccum;
					rangeEnd[endOffset] = workIndex;
					//printf("workIndex %i rangeEnd[%i] = %i\n", workIndex, endOffset, workIndex);
				}
			}

			if (workIndex == tNumContacts - 1)
			{
				const PxU32 endOffset = offsets[workIndex] + blockAccum;
				rangeEnd[endOffset] = workIndex + 1;
				//printf("workIndex %i rangeEnd[%i] = %i\n", workIndex, endOffset, workIndex+1);
			}
		}

		__syncthreads(); //sParticleId (shared memory) is read and written in the same loop - read and write must be separated with syncs
	}

	if (blockIdx.x == 0 && idx == 0)
	{
		*totalNumPairs = sTotalPairs;
		//printf("total pairs in second kernel %i\n", sTotalPairs);
	}
}

extern "C" __global__ void ps_findStartEndFEMFirstLaunch(
	PxgParticleSystem*							particleSystems,
	PxgFemOtherContactInfo*						sortedContactsInfo,
	PxU32*										numContacts,
	PxU32*										blockOffsets,
	PxU32*										offsets
)
{
	__shared__ PxU64 sParticleId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];

	//numWarpsPerBlock can't be larger than 32
	const PxU32 numWarpsPerBlock = PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA / WARP_SIZE;

	__shared__ PxU32 sWarpAccumulator[WARP_SIZE];
	__shared__ PxU32 sBlockAccumulator;

	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	//gridDim should be 64
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 threadIndexInWarp = threadIdx.x&(WARP_SIZE - 1);
	PxU32 warpIndex = threadIdx.x / (WARP_SIZE);
	const PxU32 idx = threadIdx.x;

	if (threadIdx.x == 0)
	{
		sBlockAccumulator = 0;
	}

	__syncthreads();

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;


		PxU64 particleId;
		if (workIndex < tNumContacts)
		{
			PxgFemOtherContactInfo& contactInfo = sortedContactsInfo[workIndex];

			//first pairInd is particle system
			particleId = contactInfo.pairInd0; //include particle system id

			// Load particle id into shared memory so that we can look
			// at neighboring particle id without loading
			// two particle id per thread
			sParticleId[threadIdx.x + 1] = particleId;

			if (workIndex > 0 && threadIdx.x == 0)
			{
				// first thread in block must load neighbor particle 
				PxgFemOtherContactInfo& contactInfo2 = sortedContactsInfo[workIndex - 1];
				sParticleId[0] = contactInfo2.pairInd0;
			}
		}

		__syncthreads();

		PxU32 isRangeStart = 0;
		if (workIndex < tNumContacts)
		{
			// If this particle has a different particle index to the previous
			// particle then it must be the first particle in the cell,
			// so store the index of this particle in the cell.
			// As it isn't the first particle, it must also be the cell end of
			// the previous particle's cell
			if (workIndex == 0 || particleId != sParticleId[threadIdx.x])
				isRangeStart = 1;
		}

		const PxU32 threadMask = (1 << threadIndexInWarp) - 1;
		const PxU32 mask = __ballot_sync(FULL_MASK, isRangeStart);
		//Offset is the sum of all the preceding offsets. However, if I am not a range start,
		//then I remove 1 entry to ensure that I get the same results as the preceding element.
		PxI32 offset = __popc(mask & threadMask) - (1 - isRangeStart);

		/*if (workIndex < tNumContacts)
		{
		printf("workIndex %i offset %i isRangeStart %i\n", workIndex, offset, isRangeStart);
		}*/

		if (threadIndexInWarp == (WARP_SIZE - 1))
		{
			sWarpAccumulator[warpIndex] = __popc(mask);
			/*if(blockIdx.x == 0)
			printf("sWarpAcculator[%i] = %i blockIdx.x %i\n", warpIndex, sWarpAccumulator[warpIndex], blockIdx.x);*/
		}

		const PxU32 prevBlockAccumulator = sBlockAccumulator;

		__syncthreads();

		if (warpIndex == 0)
		{
			PxU32 tOffset = threadIndexInWarp < numWarpsPerBlock ? sWarpAccumulator[threadIndexInWarp] : 0;

			const PxU32 output = warpScan<AddOpPxU32, PxU32>(FULL_MASK, tOffset) - tOffset;

			if (threadIndexInWarp == (WARP_SIZE - 1))
				sBlockAccumulator += (output + tOffset);

			sWarpAccumulator[threadIndexInWarp] = output;
		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			//Now output both offsets...
			offsets[workIndex] = offset + prevBlockAccumulator + sWarpAccumulator[warpIndex];

		}
	}

	if (threadIdx.x == 0)
	{
		blockOffsets[blockIdx.x] = sBlockAccumulator;
	}

}

extern "C" __global__ void ps_findStartEndFEMSecondLaunch(
	PxgFemOtherContactInfo*			sortedContactInfo,		//input
	PxU32*							numContacts,			//input
	PxU32*							blockOffsets,			//input
	PxU32*							offsets,				//input
	PxU32*							totalNumPairs,			//output
	PxU32*							rangeStart,				//output
	PxU32*							rangeEnd				//output
)
{
	__shared__ PxU32 sBlockAccum[PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA];
	__shared__ PxU64 sParticleId[PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + 1];
	__shared__ PxU32 sTotalPairs;

	const PxU32 tNumContacts = *numContacts;
	const PxU32 idx = threadIdx.x;

	PxU32 val = 0;
	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
		val = blockOffsets[idx];

	PxU32 res = warpScan<AddOpPxU32, PxU32>(FULL_MASK, val) - val;

	if (idx < PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA)
		sBlockAccum[idx] = res;

	if (idx == (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1))
		sTotalPairs = res + val;

	const PxU32 totalBlockRequired = (tNumContacts + (PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA;

	const PxU32 numIterationPerBlock = (totalBlockRequired + (PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA - 1)) / PxgParticleSystemKernelGridDim::ACCUMULATE_DELTA;

	__syncthreads();

	PxU32 blockAccum = sBlockAccum[blockIdx.x];
	const PxU32 startBoundaryElement = numIterationPerBlock * blockDim.x;

	for (PxU32 i = 0; i < numIterationPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + numIterationPerBlock * blockIdx.x * blockDim.x;

		PxU64 particleId;
		if (workIndex < tNumContacts)
		{
			//printf("tNumContacts %i\n", tNumContacts);

			PxgFemOtherContactInfo& contact = sortedContactInfo[workIndex];
			//first pairInd is particle system
			particleId = contact.pairInd0; //include particle system id

			// Load particle id into shared memory so that we can look
			// at neighboring particle id without loading
			// two particle id per thread
			sParticleId[threadIdx.x + 1] = particleId;

			if (workIndex > 0 && threadIdx.x == 0)
			{
				// first thread in block must load neighbor particle 
				PxgFemOtherContactInfo& contact = sortedContactInfo[workIndex - 1];
				sParticleId[0] = contact.pairInd0;
			}
		}

		__syncthreads();

		if (workIndex < tNumContacts)
		{
			// If this particle has a different particle index to the previous
			// particle then it must be the first particle in the cell,
			// so store the index of this particle in the cell.
			// As it isn't the first particle, it must also be the cell end of
			// the previous particle's cell
			if (workIndex == 0 || particleId != sParticleId[threadIdx.x])
			{

				const PxU32 startOffset = offsets[workIndex] + blockAccum;
				rangeStart[startOffset] = workIndex;
				//printf("workIndex %i rangeStart[%i] = %i\n", workIndex, startOffset, workIndex);

				if (workIndex > 0)
				{

					PxU32 tBlockAccum = blockAccum;

					//boundary element between block
					if (workIndex % startBoundaryElement == 0)
						tBlockAccum = sBlockAccum[blockIdx.x - 1];

					////boundary element between block
					//if (workIndex % blockDim.x == 0)
					//	preBlockAccum = sBlockAccum[blockIdx.x - 1];

					//using this thread to write to workIndex - 1's end range
					const PxU32 endOffset = offsets[workIndex - 1] + tBlockAccum;
					rangeEnd[endOffset] = workIndex;
					//printf("workIndex %i rangeEnd[%i] = %i\n", workIndex, endOffset, workIndex);
				}
			}

			if (workIndex == tNumContacts - 1)
			{
				const PxU32 endOffset = offsets[workIndex] + blockAccum;
				rangeEnd[endOffset] = workIndex + 1;
				//printf("workIndex %i rangeEnd[%i] = %i\n", workIndex, endOffset, workIndex+1);
			}
		}
	}

	if (blockIdx.x == 0 && idx == 0)
	{
		*totalNumPairs = sTotalPairs;
		//printf("total pairs in second kernel %i\n", sTotalPairs);
	}

}

extern "C" __global__ void ps_accumulateFEMParticleDeltaVLaunch(
	PxgParticleSystem*				particleSystems,		//output
	PxgFemOtherContactInfo*			sortedContactInfo,		//input
	PxU32*							totalNumPairs,			//input
	PxU32*							rangeStart,				//input
	PxU32*							rangeEnd,				//input
	float4*							deltaP					//input

)
{
	const PxU32 numTests = *totalNumPairs;

	const PxU32 nbBlocksRequired = (numTests + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numTests)
			return;

		const PxU32 startIndex = rangeStart[workIndex];
		const PxU32 endIndex = rangeEnd[workIndex];

		float4 accumulatedDeltaP = make_float4(0.f, 0.f, 0.f, 0.f);
		//PxReal fcount = (endIndex - startIndex);
		PxReal fcount = 0.f;

		for (PxU32 j = startIndex; j < endIndex; ++j)
		{
			float4 tDelta = deltaP[j];
			accumulatedDeltaP.x += tDelta.x;
			accumulatedDeltaP.y += tDelta.y;
			accumulatedDeltaP.z += tDelta.z;
			fcount += tDelta.w;
		}


		PxReal recipCount = fcount > 0.f ? 1.f / fcount : 0.f;

		//printf("workIndex %i range[%i, %i] fcount %f\n", workIndex, startIndex, endIndex, fcount);

		accumulatedDeltaP.x *= recipCount;
		accumulatedDeltaP.y *= recipCount;
		accumulatedDeltaP.z *= recipCount;

		PxgFemOtherContactInfo& contactInfo = sortedContactInfo[startIndex];
		PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 tParticleSystemId = PxGetParticleSystemId(pairInd0);
		const PxU32 tParticleId = PxGetParticleIndex(pairInd0);

		PxgParticleSystem& particleSystem = particleSystems[tParticleSystemId];

		//store deltaV changed in mAccumDeltaV. mAccumDeltaV will be initialized to be
		//zero at ps_preIntegrateLaunch kernel
		float4* accumDeltaP = reinterpret_cast<float4*>(particleSystem.mAccumDeltaP);

		accumDeltaP[tParticleId] += accumulatedDeltaP;

#if PX_DB_PARTICLE_MASS_SPLIT
		// Reset the mass-split contact count for the next iteration's pre-count
		// (plain store: one thread per particle here).
		accumDeltaP[tParticleId].w = 0.0f;
#endif
	}

}


extern "C" __global__ void ps_accumulateDeltaVParticleLaunch(
	PxgParticleSystem*				particleSystems,		//output
	PxgParticlePrimitiveContact*	sortedContacts,			//input
	PxU32*							totalNumPairs,			//input
	PxU32*							rangeStart,				//input
	PxU32*							rangeEnd,				//input
	float4*							deltaPos				//input
)
{
	const PxU32 numTests = *totalNumPairs;

	const PxU32 nbBlocksRequired = (numTests + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numTests)
			return;

		const PxU32 startIndex = rangeStart[workIndex];
		const PxU32 endIndex = rangeEnd[workIndex];

		//printf("workIndex %i range[%i, %i], numTests %i\n", workIndex, startIndex, endIndex, numTests);
		
		float4 accumulatedDeltaP = make_float4(0.f, 0.f, 0.f, 0.f);
		//PxReal fcount = (endIndex - startIndex);
		PxReal fcount = 0.f;
		for (PxU32 j = startIndex; j < endIndex; ++j)
		{
			float4 tDelta = deltaPos[j];
			accumulatedDeltaP.x += tDelta.x;
			accumulatedDeltaP.y += tDelta.y;
			accumulatedDeltaP.z += tDelta.z;
			fcount += tDelta.w;
		}

		PxReal recipCount = 1.f / PxMax(1.f, fcount);

		accumulatedDeltaP.x *= recipCount;
		accumulatedDeltaP.y *= recipCount;
		accumulatedDeltaP.z *= recipCount;
		
		PxgParticlePrimitiveContact& contact = sortedContacts[startIndex];
		const PxU64 combinedParticleId = contact.particleId;

		const PxU32 tParticleSystemId = PxGetParticleSystemId(combinedParticleId);
		const PxU32 tParticleId = PxGetParticleIndex(combinedParticleId);

		/*const PxU32 tParticleSystemId = getParticleSystemIdFromCombinedId(combinedParticleId);
		const PxU32 tParticleId = getParticleIdFromCombinedId(combinedParticleId);*/

		PxgParticleSystem& particleSystem = particleSystems[tParticleSystemId];
		
		//sortedVelocity1 should have selfCollision deltaV and the final result will store in sortedVelocity1
		float4* accumDeltaP = reinterpret_cast<float4*>(particleSystem.mAccumDeltaP);
		
		accumDeltaP[tParticleId] += accumulatedDeltaP;
	}

}

extern "C" __global__ void ps_accumulateStaticDensity(
	PxgParticleSystem*				particleSystems,		//output
	const PxU32* activeParticleSystems						//input
)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxgParticleSystemData& data = shParticleSystem.mData;

	const PxReal restDensityBoundary = data.mRestDensityBoundary;
	if (restDensityBoundary > 0.0f)
	{
		const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;
		const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

		if (groupThreadIdx < numParticles)
		{
			const PxgParticleContactInfo* PX_RESTRICT contacts = shParticleSystem.mOneWayContactInfos;
			const PxU32* PX_RESTRICT contactCounts = shParticleSystem.mOneWayContactCount;
			const PxU32 contactCount = PxMin(contactCounts[groupThreadIdx], PxgParticleContactInfo::MaxStaticContactsPerParticle);

			const PxReal spiky1 = data.mSpiky1;
			const PxReal particleContactDistanceInv = shParticleSystem.mCommonData.mParticleContactDistanceInv;
			const PxReal restDistance = data.mFluidRestOffset;

			PxReal rho = 0.f;
			for (PxU32 i = 0, index = groupThreadIdx; i < contactCount; ++i, index += numParticles)
			{
				const PxgParticleContactInfo& contact = contacts[index];

				rho += W(PxMax(0.f, contact.mNormal_PenW.w + restDistance), spiky1, particleContactDistanceInv);
			}
			shParticleSystem.mStaticDensity[groupThreadIdx] = PxMin(rho, restDensityBoundary);
		}
	}
}


extern "C" __global__ void ps_accumulateRigidDensity(
	PxgParticleSystem*				particleSystems,		//output
	PxgParticlePrimitiveContact*	sortedContacts,			//input
	PxU32*							totalNumPairs,			//input
	PxU32*							rangeStart,				//input
	PxU32*							rangeEnd				//input
)
{
	const PxU32 numTests = *totalNumPairs;

	const PxU32 nbBlocksRequired = (numTests + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * PxgParticleSystemKernelBlockDim::ACCUMULATE_DELTA + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numTests)
			return;

		const PxU32 startIndex = rangeStart[workIndex];
		const PxU32 endIndex = rangeEnd[workIndex];

		PxgParticlePrimitiveContact& contact = sortedContacts[startIndex];
		const PxU64 combinedParticleId = contact.particleId;

		const PxU32 tParticleSystemId = PxGetParticleSystemId(combinedParticleId);
		const PxU32 tParticleId = PxGetParticleIndex(combinedParticleId);

		/*const PxU32 tParticleSystemId = getParticleSystemIdFromCombinedId(combinedParticleId);
		const PxU32 tParticleId = getParticleIdFromCombinedId(combinedParticleId);*/

		PxgParticleSystem& particleSystem = particleSystems[tParticleSystemId];
		const PxReal restDensityBoundary = particleSystem.mData.mRestDensityBoundary;

		if (restDensityBoundary > 0.0f)
		{
			const PxReal spiky1 = particleSystem.mData.mSpiky1;
			const PxReal particleContactDistanceInv = particleSystem.mCommonData.mParticleContactDistanceInv;
			const PxReal restDistance = particleSystem.mData.mFluidRestOffset;

			//printf("workIndex %i range[%i, %i], numTests %i\n", workIndex, startIndex, endIndex, numTests);

			//PxReal fcount = (endIndex - startIndex);
			PxReal rho = particleSystem.mStaticDensity[tParticleId];// (endIndex - startIndex) * particleSystem.mData.mSpiky1 * -0.1f;

			for (PxU32 i = startIndex; i < endIndex; ++i)
			{
				PxgParticlePrimitiveContact& contact = sortedContacts[i];

				rho += W(PxMax(0.f, contact.normal_pen.w + restDistance), spiky1, particleContactDistanceInv);
			}

			//sortedVelocity1 should have selfCollision deltaV and the final result will store in sortedVelocity1
			PxReal* staticDensity = particleSystem.mStaticDensity;
			staticDensity[tParticleId] = PxMin(rho, restDensityBoundary);
		}
	}

}



extern "C" __global__ void ps_updateParticleLaunch(
	const PxgParticleSystem* const PX_RESTRICT particleSystems,
	const PxU32* activeParticleSystems,
	const PxReal invDt,
	const bool skipNewPositionAdjustment
)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();
	
	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	if (groupThreadIdx < numParticles)
	{

		float4* PX_RESTRICT accumDeltaP = reinterpret_cast<float4*>(shParticleSystem.mAccumDeltaP);

		float4* PX_RESTRICT sortedVelocity = reinterpret_cast<float4*>(shParticleSystem.mSortedVelocities);
		float4* PX_RESTRICT sortedDeltaP = reinterpret_cast<float4*>(shParticleSystem.mSortedDeltaP);

		const float4 aDeltaP = accumDeltaP[groupThreadIdx];
		const float4 oldVel = sortedVelocity[groupThreadIdx];
		const float4 newVel = oldVel + aDeltaP*invDt;

		const float4 dp = sortedDeltaP[groupThreadIdx];

		float4* PX_RESTRICT sortedNewPos = reinterpret_cast<float4*>(shParticleSystem.mSortedPositions_InvMass);
		const float4 sortedPos = sortedNewPos[groupThreadIdx];
		float4 sortedNewP = sortedPos;
		if (!skipNewPositionAdjustment)
			sortedNewP += aDeltaP;

		accumDeltaP[groupThreadIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
		sortedVelocity[groupThreadIdx] = newVel;
		sortedNewPos[groupThreadIdx] = make_float4(sortedNewP.x, sortedNewP.y, sortedNewP.z, sortedPos.w);
		sortedDeltaP[groupThreadIdx] = dp + aDeltaP;

		/*if(groupThreadIdx==0)
		{
			printf("ps_updateParticleLaunch: %f, %f, %f, %f\n", sortedNewP.x, sortedNewP.y, sortedNewP.z, sortedNewP.w);
		}*/
	}

	/*printf("selfDataV(%f, %f, %f)\n", selfDeltaV.x, selfDeltaV.y, selfDeltaV.z);
	printf("aDeltaV(%f, %f, %f)\n", aDeltaV.x, aDeltaV.y, aDeltaV.z);
	printf("sortedVelocity0(%f, %f, %f)\n", sortedVelocity0[globalThreadIndex].x, sortedVelocity0[globalThreadIndex].y, sortedVelocity0[globalThreadIndex].z);
	printf("sortedVelocity1(%f, %f, %f)\n", sortedVelocity1[globalThreadIndex].x, sortedVelocity1[globalThreadIndex].y, sortedVelocity1[globalThreadIndex].z);*/
}


// clamp a vector to a maximum magnitude
__device__ inline PxVec3 ClampLength(const PxVec3& v, PxReal maxLength)
{
	PxVec3 x = v;
	float lenSq = x.dot(x);
	float lenRcp = rsqrtf(lenSq);

	if (lenSq > sqr(maxLength))
		x *= lenRcp * maxLength;

	return x;
}


extern "C" __global__ __launch_bounds__(PxgParticleSystemKernelBlockDim::UPDATEBOUND, 1) void ps_integrateLaunch(
	PxgParticleSystem*	particleSystems,
	const PxU32*		activeParticleSystems,
	const PxReal dt,
	const PxReal epsTolerance
)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxgParticleSystemData& data = shParticleSystem.mData;

	const PxU32 globalThreadIndex = threadIdx.x + blockIdx.x * blockDim.x;

	if (globalThreadIndex >= shParticleSystem.mCommonData.mNumParticles)
		return;

	float4* PX_RESTRICT position = reinterpret_cast<float4*>(shParticleSystem.mUnsortedPositions_InvMass);
	const float4* PX_RESTRICT newPos = reinterpret_cast<float4*>(shParticleSystem.mSortedPositions_InvMass);
	const float4* PX_RESTRICT origPos = reinterpret_cast<float4*>(shParticleSystem.mSortedOriginPos_InvMass);
	const float4* PX_RESTRICT deltaP = reinterpret_cast<float4*>(shParticleSystem.mSortedDeltaP);
	//float4* original_p = reinterpret_cast<float4*>(shParticleSystem.mOriginPos_InvMass);
	float4* PX_RESTRICT originalVels = reinterpret_cast<float4*>(shParticleSystem.mUnsortedVelocities);
	const float4* const PX_RESTRICT newVels = reinterpret_cast<float4*>(shParticleSystem.mSortedVelocities);

	float4* PX_RESTRICT deltas = reinterpret_cast<float4*>(shParticleSystem.mDelta);

	const PxU32* const PX_RESTRICT gridParticleIndex = shParticleSystem.mSortedToUnsortedMapping;

	const PxU32* const PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
	const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

	PxParticleLockFlags lockFlags = PxParticleLockFlags(data.mLockFlags);

	//KS - should we use sortedPosition here? More coallesced!
	const PxU32 sortedIndex = gridParticleIndex[globalThreadIndex];
	//float4 pos4 = fetch(&position[sortedIndex]);

	const PxVec3 dta = PxLoad3(deltaP[globalThreadIndex]);

	bool locked = dta.magnitudeSquared() <= epsTolerance;
	const float4 origPos4 = origPos[globalThreadIndex];

	float4 pos4 = locked ? origPos4 : newPos[globalThreadIndex];

	if (pos4.w > 0.f)
	{
		const PxU32 phase = phases[globalThreadIndex];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsParticleMaterialData& mat = getParticleMaterial<PxsParticleMaterialData>(particleSystem.mParticleMaterials, mi,
			particleSystem.mParticleMaterialStride);

		const PxReal maxVelocity = locked ? 0.f : data.mMaxVelocity;

		float4 newVel4 = newVels[globalThreadIndex];

		//printf("globalThreadInex = %i RemapIndex = %i, newVels = (%f, %f, %f)\n", globalThreadIndex, sortedIndex, newVel4.x, newVel4.y, newVel4.z);
		PxVec3 newVel(newVel4.x, newVel4.y, newVel4.z);

		const float4 j = deltas[globalThreadIndex];
		const PxReal scale = 1.f / (PxMax(j.w, 1.f));
		newVel.x += j.x * scale; newVel.y += j.y*scale; newVel.z += j.z*scale;

		//if (!data.mUseDEMsolver)
		//{
		//	float4* impulses = reinterpret_cast<float4*>(shParticleSystem.mDelta);

		//	// impulse from velocity solve
		//	const float4 impulse = impulses[globalThreadIndex];

		//	// limit max-acceleration 
		//	//const PxVec3 v0 = oldVel;
		//	const PxVec3 v1 = newVel + PxVec3(impulse.x, impulse.y, impulse.z) / PxMax(impulse.w, 1.0f); 	// impulse averaging is used for the lift + drag forces added in UpdateTriangles()

		//	// clamped acceleration
		//	//const PxVec3 vdelta = ClampLength(v1 - v0, data.mMaxVelocityDelta);

		//	// update user velocities
		//	//newVel = v0 + vdelta;
		//}

		const PxReal magSq = newVel.magnitudeSquared();

		if (magSq > (maxVelocity*maxVelocity))
		{
			PxReal ratio = maxVelocity * rsqrtf(magSq);
			newVel *= ratio;
		}

		if(lockFlags)
		{
			if(lockFlags & PxParticleLockFlag::eLOCK_X)
			{
				pos4.x = origPos4.x;
				newVel.x = 0.0f;
			}
			if(lockFlags & PxParticleLockFlag::eLOCK_Y)
			{
				pos4.y = origPos4.y;
				newVel.y = 0.0f;
			}
			if(lockFlags & PxParticleLockFlag::eLOCK_Z)
			{
				pos4.z = origPos4.z;
				newVel.z = 0.0f;
			}
		}

		position[sortedIndex] = pos4;
		originalVels[sortedIndex] = make_float4(newVel.x, newVel.y, newVel.z, 0.f);

		deltas[globalThreadIndex] = make_float4(0.f, 0.f, 0.f, 0.f);

	}
	//printf("remapIndex %i after solve position(%f, %f, %f)\n", remapIndex, pos4.x, pos4.y, pos4.z);
	//printf("remapIndex %i newVel(%f, %f, %f)\n", remapIndex, newVel.x, newVel.y, newVel.z);
	//printf("newVel2(%f, %f, %f)\n", newVel42.x, newVel42.y, newVel42.z);

	//printf("remap Index %i\n", remapIndex);
}

//This is just for PBD
template <bool shouldComputePotentials>
__device__ void calculateDensityAndPotential(
	PxgParticleSystem* particleSystems, 
	const PxU32* activeParticleSystems)
{

	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];
	const float* sIterData = reinterpret_cast<float*>(&particleSystems[particleId]);
	float* dIterData = reinterpret_cast<float*>(&shParticleSystem);

	blockCopy<float>(dIterData, sIterData, sizeof(PxgParticleSystem));
	__syncthreads();
	
	const PxgParticleSystemData& data = shParticleSystem.mData;

	if ((data.mFlags & PxParticleFlag::eDISABLE_SELF_COLLISION))
		return;

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;

	const bool hasDiffuseParticles = (shParticleSystem.mCommonData.mMaxDiffuseParticles > 0) && shouldComputePotentials;

	const float4* PX_RESTRICT sortedNewPositions = shParticleSystem.mSortedPositions_InvMass;
	const float4* PX_RESTRICT sortedNewVelocities = shParticleSystem.mSortedVelocities;
	const PxU32* PX_RESTRICT collisionIndex = shParticleSystem.mCollisionIndex;
	const PxU32* PX_RESTRICT phases = shParticleSystem.mSortedPhaseArray;
	const PxU16* const PX_RESTRICT phaseToMat = shParticleSystem.mPhaseGroupToMaterialHandle;

	PxReal* PX_RESTRICT densities = shParticleSystem.mDensity;
	const PxReal* PX_RESTRICT staticDensities = shParticleSystem.mStaticDensity;
	float4* PX_RESTRICT normals = (float4*)shParticleSystem.mSurfaceNormal;
	float2* PX_RESTRICT potentials = shParticleSystem.mDiffusePotentials;

	const PxU32 p = blockIdx.x * blockDim.x + threadIdx.x;

	if (p < numParticles)
	{
		const PxU32 phase = phases[p];

		if (!PxGetFluid(phase))
			return;
		
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsPBDMaterialData& mat = getParticleMaterial<PxsPBDMaterialData>(shParticleSystem.mParticleMaterials, mi,
			shParticleSystem.mParticleMaterialStride);

		const PxgPBDParticleMaterialDerived& derivedMaterial = shParticleSystem.mDerivedPBDMaterialData[group];

		const float4 xi4 = sortedNewPositions[p];
		const PxVec3 xi = PxLoad3(xi4);
		const PxVec3 vi = PxLoad3(sortedNewVelocities[p]);

		PxReal rho = 0.0f;
		if (data.mRestDensityBoundary > 0.0)
		{
			rho = staticDensities[p];
		}
		PxVec3 n(0.0f);

		PxReal weight = 0.f;
		PxReal div = 0.f;

		PxU32 contactCount = shParticleSystem.mParticleSelfCollisionCount[p];

		const PxReal cellSizeSq = shParticleSystem.mCommonData.mParticleContactDistanceSq;

		// apply position updates
		PxU32 nextQ;
		PxVec3 xjNext;
		PxVec3 vjNext;

		PxU32 offset = p;

		if (contactCount > 0)
		{
			xjNext = PxLoad3(sortedNewPositions[collisionIndex[offset]]);
			offset += numParticles;
			if (hasDiffuseParticles)
				vjNext = PxLoad3(sortedNewVelocities[collisionIndex[offset]]);
		}
		if (contactCount > 1)
		{
			nextQ = collisionIndex[offset];
			offset += numParticles;
		}

		// apply position updates
		for (PxU32 i = 0; i < contactCount; ++i, offset += numParticles)
		{
			const PxVec3 xj = xjNext;
			const PxVec3 vj = vjNext;
			if ((i + 1) < contactCount)
			{
				xjNext = PxLoad3(sortedNewPositions[nextQ]);

				if (hasDiffuseParticles)
					vjNext = PxLoad3(sortedNewVelocities[nextQ]);

				if ((i + 2) < contactCount)
					nextQ = collisionIndex[offset];

			}
			const PxVec3 xij = xi - xj;

			const PxReal dSq = xij.magnitudeSquared();

			if (dSq < cellSizeSq)
			{
				const PxReal rd = rsqrtf(dSq);
				const PxReal d = 1.f/rd;

				PxReal mass = 1.0f;

				rho += mass * W(d, data.mSpiky1, shParticleSystem.mCommonData.mParticleContactDistanceInv);
				weight += mass;

				if ((derivedMaterial.surfaceTensionDerived > 0.f || hasDiffuseParticles) && d > 0.f)
				{
					const PxVec3 dw = dWdx(d, data.mSpiky2, shParticleSystem.mCommonData.mParticleContactDistanceInv) * xij *rd;
					n += dw;

					if (hasDiffuseParticles)
					{
						const PxVec3 vij = vj - vi;

						// divergence of velocity field
						div += vij.dot(dw);
					}
				}
			}
		}

		PxReal c;

		// density constraint C = rho-rho_zero
		//KS - tiny amount of negative rest density allowed. This helps to stabilize the fluid
		c = PxMax(rho - data.mRestDensity, -data.mRestDensity*0.005f);

		// scale constraint by pre-computed factor
		c *= data.mLambdaScale;

		densities[p] = c;

		// surface normals
		if (derivedMaterial.surfaceTensionDerived > 0.0f)
		{
			n *= derivedMaterial.surfaceTensionDerived;

			normals[p] = make_float4(n.x, n.y, n.z, weight);
		}

		if (hasDiffuseParticles)
		{
			if (xi4.w > 0 && rho > 0)
				div /= (xi4.w * rho);

			potentials[p] = make_float2(div, max(1.0f - 2.0f * rho, 0.0));
		}
	}
}

extern "C" __global__ void ps_calculateDensityAndPotentialLaunch(
	PxgParticleSystem* particleSystems,
	const PxU32* activeParticleSystems,
	bool shouldComputePotentials)
{
	if (shouldComputePotentials)
		calculateDensityAndPotential<true>(particleSystems, activeParticleSystems);
	else
		calculateDensityAndPotential<false>(particleSystems, activeParticleSystems);
}

__device__ inline PxReal WCohesion(const PxReal x, const PxReal kCohesion1, const PxReal kCohesion2, const PxReal kInvRadius)
{
	PxReal w = x * kInvRadius;
	return kCohesion1 * cube(w) + kCohesion2 * sqr(w) - 1.0f;
}

//This function is for PBD only
extern "C" __global__ void ps_solveDensityLaunch(
	PxgParticleSystem* particleSystems,
	const PxU32* activeParticleSystems,
	PxReal coefficient, PxReal dt)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];
	const float* sIterData = reinterpret_cast<float*>(&particleSystems[particleId]);
	float* dIterData = reinterpret_cast<float*>(&shParticleSystem);

	blockCopy<float>(dIterData, sIterData, sizeof(PxgParticleSystem));
	__syncthreads();

	const PxgParticleSystemData& data = shParticleSystem.mData;

	if ((data.mFlags & PxParticleFlag::eDISABLE_SELF_COLLISION))
		return;

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;

	const PxU32 p = blockIdx.x * blockDim.x + threadIdx.x;

	if (p >= numParticles)
		return;

	const float4* PX_RESTRICT sortedNewPositions = shParticleSystem.mSortedPositions_InvMass;
	const float4* PX_RESTRICT sortedDeltaP = shParticleSystem.mSortedDeltaP;
	//const float4* PX_RESTRICT sortedPositions = shParticleSystem.mSortedPosition_InvMass;
	const PxReal* PX_RESTRICT densities = shParticleSystem.mDensity;
	const float4* PX_RESTRICT normals = shParticleSystem.mSurfaceNormal;
	const PxU32* PX_RESTRICT phases = shParticleSystem.mSortedPhaseArray;
	const PxU32* PX_RESTRICT collisionIndex = shParticleSystem.mCollisionIndex;
	const PxU16* const PX_RESTRICT phaseToMat = shParticleSystem.mPhaseGroupToMaterialHandle;
	float2* PX_RESTRICT collisionForces = shParticleSystem.mDensityCollisionImpulses;

	float4* PX_RESTRICT sortedVel = reinterpret_cast<float4*>(shParticleSystem.mSortedVelocities);

	float4* PX_RESTRICT deltas = reinterpret_cast<float4*>(shParticleSystem.mDelta);

	
	{
		const PxU32 phase = phases[p];

		bool isFluid = PxGetFluid(phase);
		
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsPBDMaterialData& mat = getParticleMaterial<PxsPBDMaterialData>(shParticleSystem.mParticleMaterials, mi,
			shParticleSystem.mParticleMaterialStride);

		const PxgPBDParticleMaterialDerived derivedMaterial = shParticleSystem.mDerivedPBDMaterialData[group];
		const float4 xi4 = sortedNewPositions[p];
		//const PxVec3 xiPrev = sortedPositions[p].getXYZ();

		// skip fixed particles
		if (xi4.w == 0.0f)
		{
			// need to clear delta array
			deltas[p] = make_float4(0.0f, 0.0f, 0.0f, 0.0f);
			return;
		}

		const PxVec3 xi(xi4.x, xi4.y, xi4.z);

		const PxVec3 xiDelta = PxLoad3(sortedDeltaP[p]);

		const PxVec3 xiVel = PxLoad3(sortedVel[p]);



		// relying on the compiler removing this load if surface tension disabled
		float4 ni4 = normals[p];
		PxVec3 ni(ni4.x, ni4.y, ni4.z);

		//const PxReal scale = ni4.w*dt;
		const PxReal scale = dt;

		const PxReal density = densities[p];

		PxU32 contactCount = shParticleSystem.mParticleSelfCollisionCount[p];

		PxVec3 delta(0.0f);

		const PxReal cellSizeSq = shParticleSystem.mCommonData.mParticleContactDistanceSq;
		const PxReal cflRadius = shParticleSystem.mCommonData.mParticleContactDistance*mat.cflCoefficient;
		const PxReal solidRestDistance = 2.0f * data.mSolidRestOffset;

		const PxReal spiky2 = data.mSpiky2;
		const PxReal spiky1 = data.mSpiky1;
		const PxReal particleContactDistanceInv = shParticleSystem.mCommonData.mParticleContactDistanceInv;

		const PxReal friction = mat.friction * mat.particleFrictionScale;

		const PxReal cohesion = derivedMaterial.cohesion * scale;
		const PxReal cohesion1 = derivedMaterial.cohesion1;
		const PxReal cohesion2 = derivedMaterial.cohesion2;
		const PxReal invRadius = shParticleSystem.mCommonData.mParticleContactDistanceInv;
		const PxReal viscosity = mat.viscosity * scale * data.mInvRestDensity;// * ni4.w;
		const PxReal adhesion = mat.adhesion * mat.particleAdhesionScale;

		const PxReal cohesionRadius = solidRestDistance * mat.adhesionRadiusScale;

		const PxReal invAura = 1.f/(cohesionRadius - solidRestDistance);

		PxReal weight = 0.f;

		PxU32 nextQ;
		PxU32 nextNextQ;
		PxVec4 xj4Next;
		PxU32 nextPhase;
		PxReal nextDensity;
		PxVec3 nextDeltaQ;

		PxU32 offset = p;

		if (contactCount > 0)
		{
			nextQ = collisionIndex[offset];
			xj4Next = PxLoad4(sortedNewPositions[nextQ]);
			nextPhase = phases[nextQ];
			nextDeltaQ = PxLoad3(sortedDeltaP[nextQ]);

			if (isFluid)
				nextDensity = densities[nextQ];

			offset += numParticles;
		}
		if (contactCount > 1)
		{
			nextNextQ = collisionIndex[offset];
			offset += numParticles;
		}

		// apply position updates
		for (PxU32 i = 0, writeOffset = p; i < contactCount; ++i, offset += numParticles, writeOffset += numParticles)
		{
			const PxVec4 xj4 = xj4Next;
			const PxU32 phase2 = nextPhase;
			PxU32 q = nextQ;
			PxReal density2 = nextDensity;
			PxVec3 xjDelta = nextDeltaQ;

			if ((i + 1) < contactCount)
			{
				xj4Next = PxLoad4(sortedNewPositions[nextNextQ]);
				nextPhase = phases[nextNextQ];
				if (isFluid)
					nextDensity = densities[nextNextQ];

				nextQ = nextNextQ;
				nextDeltaQ = PxLoad3(sortedDeltaP[nextNextQ]);

				if ((i + 2) < contactCount)
					nextNextQ = collisionIndex[offset];

			}

			const PxVec3 xij = (xi - xj4.getXYZ());
			const PxReal dSq = xij.magnitudeSquared();

			if (dSq >= 1e-10f)
			{
				const PxReal d = sqrtf(dSq);
				PxVec3 nij = (1.0f / d) * xij;

				if (PxGetFluid(phase2) && PxGetFluid(phase))
				{
					if (dSq < cellSizeSq && dSq > 0.f)
					{
						// apply delta from both constraints simulataneously 
						PxReal lambda = (density + density2)*0.5f;


						// bilateral density constraint
						PxReal s = lambda * dWdx(d, spiky2, particleContactDistanceInv);

						if (phase2 != phase)
						{
							// no cohesion between fluid particles of different phases (necessary for multiphase effects like Rayleigh Taylor)
							delta -= s * nij;
						}
						else
						{
							//PxReal c = data.mCohesion * WCohesion(d, data.mCohesion1, data.mCohesion2, data.mInvRadius);
							PxReal c = cohesion * WCohesion(d, cohesion1, cohesion2, invRadius);

							PxVec3 relDelta_ij = xiDelta - xjDelta;

							// TOFIX: Assume xi4.w to be 1.0f. If scaling is necessary, a new scaling param should be added rather than using the inverse mass to scale.
							//delta -= xi4.w * (s + c) * nij;
							delta -= nij * (s + c);

							//Viscosity
							PxReal viscosityScale = 1.f - (1.f / (1.f + viscosity * W(d, spiky1, particleContactDistanceInv)));
							delta -= viscosityScale * relDelta_ij;

							//CFL project delta difference to normal direction, TODO maybe add current delta contributions to relDelta_ij
							PxReal relDeltaProj = nij.dot(relDelta_ij);
							if (relDeltaProj < -cflRadius)
							{
								delta -= nij * (relDeltaProj + cflRadius) * 0.5f;
							}
							
							if (mat.surfaceTension > 0.0f)
								delta = delta - (ni - PxLoad3(normals[q]))*scale;
						}

						weight += 1.f;
					}
				}
				else //if (d < solidRestDistance)
				{
					float2 force = collisionForces[writeOffset];
					float2 newForce;

					const PxReal w = xi4.w / (xi4.w + xj4.w);

					PxReal e = (d - solidRestDistance);

					const PxVec3 xij = xjDelta - xiDelta;

					PxReal adhesionClamp = 0.f;

					if (d < cohesionRadius)
					{
						adhesionClamp = PxMax(0.f, adhesion * oct(1.f - e *invAura) * e);
						//printf("AdhesionClamp = %f, coeff = %f\n", adhesionClamp, coeff);
					}

					PxReal mtd = PxClamp(PxMin(e, PxMax(-force.x, adhesionClamp)), -solidRestDistance, solidRestDistance);

					newForce.x = force.x + mtd;
					newForce.y = force.y;

					//printf("newForce.x = %f\n", newForce.x);

					if (newForce.x != 0.f)
					{
						// position delta
						PxReal deltaF = w * mtd;
						delta -= nij * deltaF;

						if (friction > 0.0f)
						{
							// measure relative displacement between particles this time-step
							const PxVec3 j = xiDelta - xjDelta;
							const PxReal jn = j.dot(nij);

							// tangential displacement this time-step
							const PxVec3 jt = j - jn * nij;
							const PxReal jtSq = jt.dot(jt);

							if (jtSq > 0.f)
							{
								const PxReal length = PxSqrt(jtSq);

								PxReal maxForce = PxMax(-newForce.x, 0.f) * friction;
								PxReal clampedForce = min(maxForce, length + newForce.y);
								PxReal deltaForce = clampedForce - newForce.y;
								{
									newForce.y += deltaForce;

									delta -= jt * deltaForce * w / length;
								}
							}
						}

						weight += 1.0f;
					}
					newForce.x = PxMin(newForce.x, 0.f);
					collisionForces[writeOffset] = newForce;
				}
			}
		}

		delta *= coefficient;

		__syncwarp();

		deltas[p] = make_float4(delta.x, delta.y, delta.z, weight);
	}
}

extern "C" __global__ void ps_applyDeltaLaunch(
	PxgParticleSystem* particleSystems,
	const PxU32* activeParticleSystems,
	const PxReal dt)
{
	const PxReal damp = 1.0f;

	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticleSystem = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticleSystem = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticleSystem, sParticleSystem, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxgParticleSystemData& data = shParticleSystem.mData;

	const PxReal relaxation = data.mRelaxationFactor;

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;
	float4* PX_RESTRICT deltas = reinterpret_cast<float4*>(shParticleSystem.mDelta);

	//const PxU32* PX_RESTRICT phases = shParticleSystem.mSortedPhaseArray;

	//PxVec4* sortedNewPositions = shParticleSystem.mSortedNewPosition_Mass;
	//const float4* PX_RESTRICT sortedNewPositions = reinterpret_cast<float4*>(shParticleSystem.mSortedPosition_Mass);
	float4* PX_RESTRICT accumDeltaP = reinterpret_cast<float4*>(shParticleSystem.mAccumDeltaP);
	//float4* PX_RESTRICT solverDeltaP = reinterpret_cast<float4*>(shParticleSystem.mDensitySolveDeltaP);

	const PxU32 p = blockIdx.x * blockDim.x + threadIdx.x;

	if (p < numParticles)
	{
		//const PxU32 phase = phases[p];

		// as a Jacobi solver we need to average deltas in order to
		// guarantee convergence (mass splitting), in order to improve 
		// convergence rate we use an SOR style over-relaxation 
		// values >= 0.5 guarantee convergence, but can occasionally use faster
		const float4 j = deltas[p];
		const float4 accumDltaP = accumDeltaP[p];

		//const float4 solverDelta = solverDeltaP[p];

		//Equal to (1/(relaxation))/dt
		const PxReal scale = 1.0f / (PxMax(j.w * relaxation, damp));

		

		//const float4 oldPos4 = sortedNewPositions[p];
		const PxVec3 deltaPos(j.x, j.y, j.z);
		PxVec3 averagedPos = (deltaPos) * scale;

		//printf("deltaPos = (%f, %f, %f), Scale = %f, relaxation = %f\n", deltaPos.x, deltaPos.y, deltaPos.z, scale, relaxation);

		accumDeltaP[p] = make_float4(accumDltaP.x + averagedPos.x, accumDltaP.y + averagedPos.y, accumDltaP.z + averagedPos.z, 0.0f);
		deltas[p] = make_float4(0.f, 0.f, 0.f, 0.f);
		
		//solverDeltaP[p] = make_float4(solverDelta.x + averagedPos.x, solverDelta.y + averagedPos.y, solverDelta.z + averagedPos.z, 0.0f);
	}
}

extern "C" __global__ void ps_vorticityConfinementLaunch(
	PxgParticleSystem* particleSystems,
	const PxU32* activeParticleSystems)
{

	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticle = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticle = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticle, sParticle, sizeof(PxgParticleSystem));

	__syncthreads();


	const PxgParticleSystemData& data = shParticleSystem.mData;

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;
	const float4* PX_RESTRICT sortedNewPositions = shParticleSystem.mSortedPositions_InvMass;
	const float4* PX_RESTRICT sortedNewVelocities = shParticleSystem.mSortedVelocities;
	const PxU32* PX_RESTRICT sortedPhases = shParticleSystem.mSortedPhaseArray;
	const PxU32* PX_RESTRICT collisionIndex = shParticleSystem.mCollisionIndex;
	const PxU16* const PX_RESTRICT phaseToMat = shParticleSystem.mPhaseGroupToMaterialHandle;

	float4* PX_RESTRICT curl = reinterpret_cast<float4*>(shParticleSystem.mCurl);

	const PxU32 p = blockIdx.x * blockDim.x + threadIdx.x;

	if (p < numParticles)
	{
		const int phase = fetch(&sortedPhases[p]);
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsPBDMaterialData& mat = getParticleMaterial<PxsPBDMaterialData>(shParticleSystem.mParticleMaterials, mi,
			shParticleSystem.mParticleMaterialStride);
		if (PxGetFluid(phase) == false || mat.vorticityConfinement == 0.f)
		{
			curl[p] = make_float4(0.0f);
			return;
		}

		const float4 xi4 = fetch(&sortedNewPositions[p]);
		const float4 vi4 = fetch(&sortedNewVelocities[p]);
		const PxVec3 xi(xi4.x, xi4.y, xi4.z);
		const PxVec3 vi(vi4.x, vi4.y, vi4.z);

		//const int contactCount = contactCounts[p];
		//ContactIter<const int> cIter(contacts, numParticles, p);
		const PxU32 contactCount = shParticleSystem.mParticleSelfCollisionCount[p];

		PxVec3 vorticity(0.0f);

		//const PxReal dist = data.mParticleContactDistanceSq;

		const PxReal dist2 = shParticleSystem.mCommonData.mParticleContactDistanceSq;

		const PxReal spiky2 = data.mSpiky2;
		const PxReal distanceInv = shParticleSystem.mCommonData.mParticleContactDistanceInv;

		PxU32 nextQ;
		PxU32 nextNextQ;
		PxVec4 xj4Next;
		float4 vj4Next;
		PxU32 nextPhase;

		PxU32 offset = p;

		if (contactCount > 0)
		{
			nextQ = collisionIndex[offset];
			xj4Next = PxLoad4(sortedNewPositions[nextQ]);
			vj4Next = sortedNewVelocities[nextQ];
			nextPhase = sortedPhases[nextQ];

			offset += numParticles;
		}
		if (contactCount > 1)
		{
			nextNextQ = collisionIndex[offset];
			offset += numParticles;
		}

		// apply velocity updates
		for (int c = 0; c < contactCount; ++c, offset += numParticles)
		{
			//const int q = cIter.Get();
			//++cIter;
			const PxVec4 cj4 = xj4Next;
			const PxU32 phasej = nextPhase;
			float4 vj4 = vj4Next;

			if ((c + 1) < contactCount)
			{
				xj4Next = PxLoad4(sortedNewPositions[nextNextQ]);
				nextPhase = sortedPhases[nextNextQ];
				vj4Next = sortedNewVelocities[nextQ];

				nextQ = nextNextQ;

				if ((c + 2) < contactCount)
					nextNextQ = collisionIndex[offset];

			}
			if (PxGetFluid(phasej) == false)
				continue;

			const PxVec3 xj(cj4.x, cj4.y, cj4.z);
			const PxVec3 xij = xi - xj;

			const PxReal dSq = xij.magnitudeSquared();

			if (dSq <= dist2 && dSq > 0.0f)
			{
				const PxVec3 vj(vj4.x, vj4.y, vj4.z);
				const PxVec3 vij = vj - vi;

				const PxReal d = sqrtf(dSq);
				const PxVec3 dw = dWdx(d, spiky2, distanceInv) * (xij / d);

				// curl of velocity field
				vorticity += vij.cross(dw);
			}
		}

		curl[p] = make_float4(vorticity.x, vorticity.y, vorticity.z, vorticity.magnitude());
	}
}

//This is just for PBD fluid
extern "C" __global__ void ps_solveVelocityLaunch(
	PxgParticleSystem* particleSystems, 
	const PxU32* activeParticleSystems,
	const PxReal dt)
{
	__shared__ __align__(16) PxU8 particleSystemMemory[sizeof(PxgParticleSystem)];
	PxgParticleSystem& shParticleSystem = *(reinterpret_cast<PxgParticleSystem*>(particleSystemMemory));

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];

	const uint2* sParticle = reinterpret_cast<const uint2*>(&particleSystem);
	uint2* dParticle = reinterpret_cast<uint2*>(&shParticleSystem);

	blockCopy<uint2>(dParticle, sParticle, sizeof(PxgParticleSystem));

	__syncthreads();

	const PxgParticleSystemData& data = shParticleSystem.mData;

	const PxU32 numParticles = shParticleSystem.mCommonData.mNumParticles;
	const float4* PX_RESTRICT sortedNewPositions = reinterpret_cast<float4*>(shParticleSystem.mSortedPositions_InvMass);
	const PxU32* PX_RESTRICT collisionIndex = shParticleSystem.mCollisionIndex;
	const float4* PX_RESTRICT curls = reinterpret_cast<float4*>(shParticleSystem.mCurl);
	const PxU32* PX_RESTRICT phases = shParticleSystem.mSortedPhaseArray;
	const PxU16* const PX_RESTRICT phaseToMat = shParticleSystem.mPhaseGroupToMaterialHandle;

	float4* PX_RESTRICT deltas = reinterpret_cast<float4*>(shParticleSystem.mDelta);

	const PxReal spiky2 = data.mSpiky2;

	const PxU32 p = blockIdx.x * blockDim.x + threadIdx.x;

	if (p < numParticles)
	{
		const PxU32 phase = phases[p];
		const PxU32 group = PxGetGroup(phase);
		const PxU32 mi = phaseToMat[group];
		const PxsPBDMaterialData& mat = getParticleMaterial<PxsPBDMaterialData>(shParticleSystem.mParticleMaterials, mi,
			shParticleSystem.mParticleMaterialStride);
		
		if(mat.vorticityConfinement <= 0.f) //No vorticity, so the whole kernel can be skipped for this particle.
		{
			deltas[p] = make_float4(0.0f);
			return;
		}

		const float4 xi4 = sortedNewPositions[p];
		const PxVec3 xi(xi4.x, xi4.y, xi4.z);

		float4 curl4;
		curl4 = curls[p];

		PxVec3 impulse = PxVec3(0.0f);
		PxVec3 vorticityGrad = PxVec3(0.0f);
		//PxReal divergence = 0.0f;
		PxReal weight = 0.0f;

		const PxReal mParticleContactDistanceSq = shParticleSystem.mCommonData.mParticleContactDistanceSq;

		//const int contactCount = contactCounts[p];
		//ContactIter<const int> cIter(contacts, numParticles, p);

		PxU32 contactCount = shParticleSystem.mParticleSelfCollisionCount[p];

		if (PxGetFluid(phase))
		{
			PxU32 nextQ;
			PxU32 nextNextQ;
			PxVec4 xj4Next;
			PxU32 nextPhase;
			float4 nextCurls;

			PxU32 offset = p;

			if (contactCount > 0)
			{
				nextQ = collisionIndex[offset];
				xj4Next = PxLoad4(sortedNewPositions[nextQ]);
				nextPhase = phases[nextQ];
				nextCurls = curls[nextQ];

				offset += numParticles;
			}
			if (contactCount > 1)
			{
				nextNextQ = collisionIndex[offset];
				offset += numParticles;
			}


			const PxReal particleContactDistanceInv = shParticleSystem.mCommonData.mParticleContactDistanceInv;

			// apply velocity updates
			for (int c = 0; c < contactCount; ++c, offset += numParticles)
			{
				//const int q = cIter.Get();
				//++cIter;
				const PxVec4 xj4 = xj4Next;
				const PxU32 phasej = nextPhase;
				//const PxU32 q = nextQ;
				const float4 curl = nextCurls;

				if ((c + 1) < contactCount)
				{
					xj4Next = PxLoad4(sortedNewPositions[nextNextQ]);
					nextPhase = phases[nextNextQ];
					nextCurls = curls[nextNextQ];

					nextQ = nextNextQ;

					if ((c + 2) < contactCount)
						nextNextQ = collisionIndex[offset];

				}


				const PxVec3 xj(xj4.x, xj4.y, xj4.z);
				const PxVec3 xij = xi - xj;

				const PxReal dSq = xij.magnitudeSquared();

				if (dSq <= mParticleContactDistanceSq && dSq > 0.0f)
				{
					// apply fluid forces between particles
					// ML: We explicitly compare to see if the phases are the same because we do not simulate
					// cohesion or any other fluid-like behavior between particles in different phases
					if (phase == phasej)
					{
						const PxReal d = sqrtf(dSq);
						const PxVec3 n = xij / d;

						const PxVec3 dw = dWdx(d, spiky2, particleContactDistanceInv) * n;
						vorticityGrad += curl.w * dw;
						weight += 1.0f;
					}
				}
			}


			const PxVec3 curl(curl4.x, curl4.y, curl4.z);

			// dtSq here is incorrect (should be dt), but leaving it like this for compatibility
			impulse += dt * data.mInvRestDensity * mat.vorticityConfinement * SafeNormalize(vorticityGrad, PxVec3(0.f)).cross(curl);
		}

		deltas[p] = make_float4(impulse.x, impulse.y, impulse.z, weight);

	}
}

extern "C" __global__ void ps_finalizeParticlesLaunch(const PxgParticleSystem* PX_RESTRICT particleSystems,
	const PxU32 id, const PxReal invTotalDt, PxReal velocityScale)
{
	const PxgParticleSystem& particleSystem = particleSystems[id];

	//const PxgParticleSystemData& data = particleSystem.mData;

	const PxU32 numParticles = particleSystem.mCommonData.mNumParticles;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	float4* const PX_RESTRICT vels = particleSystem.mSortedVelocities;
	float4* PX_RESTRICT deltaP = particleSystem.mSortedDeltaP;

	if (globalThreadIndex < numParticles)
	{
		float4 tVel = vels[globalThreadIndex];
		if (tVel.w != 0.f)
		{
			float4 dlta = deltaP[globalThreadIndex];
			PxVec3 deltaVel(dlta.x*invTotalDt, dlta.y*invTotalDt, dlta.z*invTotalDt);
			PxVec3 vel(tVel.x, tVel.y, tVel.z);

			/*
			* AD 07.2022
			*
			* I reverted the code below from just being
			* newVel = deltaVel;
			* without the if to make picking-attachments work again. 
			* when just assigning deltaVel in all cases, the picking was very jittery.
			*
			* I'm speculating we need the if because having setting the velocity to be
			* from the velocity in case it is higher than the inferred velocity by the position
			* change will actually accelerate the system and introduce energy, thus the jittering.
			* I have no idea if this is correct though, but we have the same code in FEMCloth.
			*
			*/

			if (vel.magnitudeSquared() > deltaVel.magnitudeSquared())
			{
				PxVec3 newVel = vel * velocityScale + (deltaVel*(1.f - velocityScale));

				vels[globalThreadIndex] = make_float4(newVel.x, newVel.y, newVel.z, tVel.w);
			}
		}
	}
}


extern "C" __global__ void ps_updateMaterials(
	const PxgParticleSystem* const PX_RESTRICT		particleSystems,
	const PxU32*	const PX_RESTRICT				activeParticleSystems,
	const PxsPBDMaterialData* const PX_RESTRICT	pbdMaterials,
	const PxReal invTotalDt)
{

	const PxU32 particleId = activeParticleSystems[blockIdx.y];

	const PxgParticleSystem& particleSystem = particleSystems[particleId];
	const PxgParticleSystemData& data = particleSystem.mData;
	const PxU32 numPhaseToMaterials = data.mNumPhaseToMaterials;

	const PxU32 i = threadIdx.x + blockDim.x*blockIdx.x;

	if (i < numPhaseToMaterials)
	{
		
		const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

		const PxU32 mi = phaseToMat[i];
		const PxsPBDMaterialData& mat = pbdMaterials[mi];

		PxReal surfaceTension = mat.surfaceTension;

		const PxReal fluidRestDistance = 2.f * data.mFluidRestOffset;
		const PxReal particleContactDistance = particleSystem.mCommonData.mParticleContactDistance;

		surfaceTension = data.mInvRestDensity * mat.surfaceTension / data.mFluidSurfaceConstraintScale;

		PxReal cohesion = mat.cohesion *particleContactDistance;

		// coefficients for a cubic that has C(0)=-1, C(restDistance)=0, C(1)=0, dCdX(0)=0
		const PxReal rest = fluidRestDistance / particleContactDistance;
		PxReal cohesion1 = -(1.0f + rest) / sqr(rest);
		PxReal cohesion2 = (sqr(rest) + rest + 1.0f) / sqr(rest);

		particleSystem.mDerivedPBDMaterialData[i] = PxgPBDParticleMaterialDerived(surfaceTension, cohesion, cohesion1, cohesion2);
		
	}
}


