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

#include "foundation/PxSimpleTypes.h"
#include "PxgBodySim.h"
#include "PxgSolverBody.h"
#include "PxvDynamics.h"
#include "PxsRigidBody.h"
#include "PxgSolverKernelIndices.h"
#include "stdio.h"

using namespace physx;

__device__ __forceinline__ PxVec3 computeSafeSqrtInertia(const PxVec3& v)
{
	return PxVec3(v.x == 0.f ? 0.f : PxSqrt(v.x), v.y == 0.f ? 0.f : PxSqrt(v.y), v.z == 0.f ? 0.f : PxSqrt(v.z));
}

__device__ __forceinline__ void transformInertiaTensor(const PxVec3& invD, const PxMat33& M, PxAlignedMat33& mIInv)
{
	const float	axx = invD.x*M(0, 0), axy = invD.x*M(1, 0), axz = invD.x*M(2, 0);
	const float	byx = invD.y*M(0, 1), byy = invD.y*M(1, 1), byz = invD.y*M(2, 1);
	const float	czx = invD.z*M(0, 2), czy = invD.z*M(1, 2), czz = invD.z*M(2, 2);

	mIInv(0, 0) = axx*M(0, 0) + byx*M(0, 1) + czx*M(0, 2);
	mIInv(1, 1) = axy*M(1, 0) + byy*M(1, 1) + czy*M(1, 2);
	mIInv(2, 2) = axz*M(2, 0) + byz*M(2, 1) + czz*M(2, 2);

	mIInv(0, 1) = mIInv(1, 0) = axx*M(1, 0) + byx*M(1, 1) + czx*M(1, 2);
	mIInv(0, 2) = mIInv(2, 0) = axx*M(2, 0) + byx*M(2, 1) + czx*M(2, 2);
	mIInv(1, 2) = mIInv(2, 1) = axy*M(2, 0) + byy*M(2, 1) + czy*M(2, 2);
}

__device__ __forceinline__ void transformInertiaTensor(const PxVec3& invD, const PxMat33& M, PxMat33& mIInv)
{
	const float	axx = invD.x*M(0, 0), axy = invD.x*M(1, 0), axz = invD.x*M(2, 0);
	const float	byx = invD.y*M(0, 1), byy = invD.y*M(1, 1), byz = invD.y*M(2, 1);
	const float	czx = invD.z*M(0, 2), czy = invD.z*M(1, 2), czz = invD.z*M(2, 2);

	mIInv(0, 0) = axx*M(0, 0) + byx*M(0, 1) + czx*M(0, 2);
	mIInv(1, 1) = axy*M(1, 0) + byy*M(1, 1) + czy*M(1, 2);
	mIInv(2, 2) = axz*M(2, 0) + byz*M(2, 1) + czz*M(2, 2);

	mIInv(0, 1) = mIInv(1, 0) = axx*M(1, 0) + byx*M(1, 1) + czx*M(1, 2);
	mIInv(0, 2) = mIInv(2, 0) = axx*M(2, 0) + byx*M(2, 1) + czx*M(2, 2);
	mIInv(1, 2) = mIInv(2, 1) = axy*M(2, 0) + byy*M(2, 1) + czy*M(2, 2);
}


PX_FORCE_INLINE __device__ PxVec3 getGravityAcceleration(const PxU32 disableGravity, const PxReal accelScale, const PxVec3& gravity, const PxReal dt)
{
	if(!(disableGravity))
	{
		return gravity * accelScale * dt;
	}

	return PxVec3(0);
}


__device__  __forceinline__ void bodyCoreComputeUnconstrainedVelocity(const float4& maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW, 
	float4& linearVelocityXYZ_inverseMassW, float4& angularVelocityXYZ_maxPenBiasW,
	const PxU32 lockFlags, const PxU32 disableGravity, const PxReal accelScale, const PxVec3& gravity, 
	const float4& linearAccel, const float4& angularAccel, const PxReal dt)
{
	PxVec3 linearVelocity(linearVelocityXYZ_inverseMassW.x, linearVelocityXYZ_inverseMassW.y, linearVelocityXYZ_inverseMassW.z);
	PxVec3 angularVelocity(angularVelocityXYZ_maxPenBiasW.x, angularVelocityXYZ_maxPenBiasW.y, angularVelocityXYZ_maxPenBiasW.z);
	const float4 temp = maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW;



	//Multiply everything that needs multiplied by dt to improve code generation.
	const PxVec3 linearAccelTimesDT = getGravityAcceleration(disableGravity, accelScale, gravity, dt) + PxVec3(linearAccel.x, linearAccel.y, linearAccel.z) * dt;
	const PxVec3 angularAccelTimesDT = PxVec3(angularAccel.x, angularAccel.y, angularAccel.z) * dt;
	//const PxVec3 angularAccelTimesDT = PxVec3(0.f);
	const PxReal linearDampingTimesDT = temp.z*dt;
	const PxReal angularDampingTimesDT = temp.w*dt;
	const PxReal oneMinusLinearDampingTimesDT = 1.0f - linearDampingTimesDT;
	const PxReal oneMinusAngularDampingTimesDT = 1.0f - angularDampingTimesDT;

	//TODO context-global gravity
	linearVelocity += linearAccelTimesDT;
	angularVelocity += angularAccelTimesDT;

	//Apply damping.
	const PxReal linVelMultiplier = physx::intrinsics::fsel(oneMinusLinearDampingTimesDT, oneMinusLinearDampingTimesDT, 0.0f);
	const PxReal angVelMultiplier = physx::intrinsics::fsel(oneMinusAngularDampingTimesDT, oneMinusAngularDampingTimesDT, 0.0f);
	linearVelocity *= linVelMultiplier;
	angularVelocity *= angVelMultiplier;

	// Clamp velocity
	const PxReal angVelSq = angularVelocity.magnitudeSquared();
	if (angVelSq > temp.y)
	{
		angularVelocity *= PxSqrt(temp.y / angVelSq);
	}

	const PxReal linVelSq = linearVelocity.magnitudeSquared();
	if (linVelSq > temp.x)
	{
		linearVelocity *= PxSqrt(temp.x / linVelSq);
	}

	//printf("%i, LV = (%f, %f, %f)\n", threadIdx.x, linearVelocity.x, linearVelocity.y, linearVelocity.z);
	//printf("%i, AV = (%f, %f, %f)\n", threadIdx.x, angularVelocity.x, angularVelocity.y, angularVelocity.z);


	linearVelocityXYZ_inverseMassW.x = lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_X ? 0.f : linearVelocity.x;
	linearVelocityXYZ_inverseMassW.y = lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Y ? 0.f : linearVelocity.y;
	linearVelocityXYZ_inverseMassW.z = lockFlags & PxRigidDynamicLockFlag::eLOCK_LINEAR_Z ? 0.f : linearVelocity.z;
	angularVelocityXYZ_maxPenBiasW.x = lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_X ? 0.f : angularVelocity.x;
	angularVelocityXYZ_maxPenBiasW.y = lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Y ? 0.f : angularVelocity.y;
	angularVelocityXYZ_maxPenBiasW.z = lockFlags & PxRigidDynamicLockFlag::eLOCK_ANGULAR_Z ? 0.f : angularVelocity.z;
}

//Reads a 4 byte element from shared buffer, where data is swizzled to avoid bank conflicts
template<typename RetType, typename ContainedClass>
PX_FORCE_INLINE PX_CUDA_CALLABLE RetType readSwizzledWord(const uint* sharedBuffer, uint element, uint quadwordWithinElem, uint elemInQuadword)
{
	const uint quadword = (element * sizeof(ContainedClass) / sizeof(uint4)) + quadwordWithinElem; //Which quadword within the structure am I reading?

	const uint page = quadword / 32; //Which page is it in?
	const uint subpage = quadword & 31; //Which quadword within the page is it in?

	const uint address = page * 128 + elemInQuadword * 32 + subpage;

	return reinterpret_cast<const RetType*>(sharedBuffer)[address];
}

template<typename ContainedClass>
PX_FORCE_INLINE PX_CUDA_CALLABLE float4 readSwizzledFloat4(const uint* sharedBuffer, uint element, uint quadwordWithinElem)
{
	const uint quadword = (element * sizeof(ContainedClass) / sizeof(uint4)) + quadwordWithinElem; //Which quadword within the structure am I reading?

	const uint page = quadword / 32; //Which page is it in?
	const uint subpage = quadword & 31; //Which quadword within the page is it in?

	const uint address = page * 128 + subpage;

	return make_float4(reinterpret_cast<const float*>(sharedBuffer)[address], reinterpret_cast<const float*>(sharedBuffer)[address + 32], reinterpret_cast<const float*>(sharedBuffer)[address + 64],
		reinterpret_cast<const float*>(sharedBuffer)[address + 96]);
}

//fill in all the dynamic bodies data
static __device__ void preIntegration(const uint32_t offset, const uint32_t nbSolverBodies, const PxReal dt, const PxVec3 gravity,
	PxgSolverBodyData* PX_RESTRICT solverBodyDataPool,
	PxgSolverBodySleepData* PX_RESTRICT solverBodySleepDataPool,
	PxgSolverTxIData* PX_RESTRICT solverTxIDataPool,
	const PxgBodySim* PX_RESTRICT bodySimPool, 
	const PxNodeIndex* PX_RESTRICT islandNodeIndices,
	PxAlignedTransform* PX_RESTRICT gTransforms,
	float4* PX_RESTRICT gOutVelocityPool,
	PxU32* PX_RESTRICT solverBodyIndices,
	bool skipGravityApplication = false)
{
	const uint32_t idx = threadIdx.x + blockIdx.x * blockDim.x;
	const uint32_t a = idx + offset;

	//	const uint32_t warpStartIdx = offset + (idx&(~31));

	const PxU32 BodySimSize = sizeof(PxgBodySim) / sizeof(float4);

	__shared__ uint sharedBufferSpace[PxgKernelBlockDim::PRE_INTEGRATION / 32][16 * 33];

	__shared__ PxU32 sharedIslandNodeIndices[PxgKernelBlockDim::PRE_INTEGRATION / 32][32];

	const PxU32 warpIndex = threadIdx.x / 32;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	if (a < nbSolverBodies)
	{
		PxU32 index = islandNodeIndices[a].index();
		sharedIslandNodeIndices[warpIndex][threadIndexInWarp] = index;
		solverBodyIndices[index] = a;
		//printf("%i: SharedIslandNodeIndices[%i][%i] = %i, %i\n", a, warpIndex, threadIndexInWarp, sharedIslandNodeIndices[warpIndex][threadIndexInWarp], islandNodeIndices[a]);
	}

	__syncwarp();


	PX_COMPILE_TIME_ASSERT((sizeof(uint4) * 4 * 4) >= BodySimSize);

	const PxU32 startReadIndex = a - threadIndexInWarp;

	const PxU32 NbToRead = startReadIndex < nbSolverBodies ? PxMin(32u, nbSolverBodies - startReadIndex) : 0;


	float4 linearVelocityXYZ_inverseMassW;
	float4 angularVelocityXYZ_maxPenBiasW;
	float4 sleepAngVelAccXYZ_accelScaleW;
	float4 inverseInertiaXYZ_contactReportThresholdW;
	float4 maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW;
	float4 linearAccel;
	float4 angularAccel;
	float maxImpulse;
	PxAlignedTransform body2World;
	PxMat33 sqrtInvInertia;
	PxU32 internalFlags;
	PxU16 lockFlags;
	PxU16 disableGravity;
	PxReal offsetSlop;

	for (PxU32 i = 0; i < NbToRead; i += 8)
	{
		const PxU32 TotalUint4ToRead = BodySimSize * PxMin(NbToRead - i, 8u);

		for (PxU32 j = threadIndexInWarp, iter = 0; j < TotalUint4ToRead; j += 32, iter++)
		{
			const PxU32 ind = j / BodySimSize;
			const PxU32 nodeIndex = sharedIslandNodeIndices[warpIndex][ind + i];
			const PxU32 offset = j - (ind*BodySimSize);

			const uint4* src = reinterpret_cast<const uint4*>(&bodySimPool[nodeIndex]);
			const uint4 val = src[offset];
			sharedBufferSpace[warpIndex][iter * 128 + threadIndexInWarp] = val.x;
			sharedBufferSpace[warpIndex][iter * 128 + 32 + threadIndexInWarp] = val.y;
			sharedBufferSpace[warpIndex][iter * 128 + 64 + threadIndexInWarp] = val.z;
			sharedBufferSpace[warpIndex][iter * 128 + 96 + threadIndexInWarp] = val.w;
		}

		__syncwarp();

		if (threadIndexInWarp >= i && threadIndexInWarp < (i + 8) && a < nbSolverBodies)
		{
			const uint* bSims = reinterpret_cast<uint*>(sharedBufferSpace[warpIndex]);

			const PxU32 readIndex = threadIndexInWarp & 7;

			linearVelocityXYZ_inverseMassW = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 0);
			angularVelocityXYZ_maxPenBiasW = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 1);
			maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 2);
			inverseInertiaXYZ_contactReportThresholdW = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 3);
			sleepAngVelAccXYZ_accelScaleW = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 5);
			
			float4 q = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 7);
			float4 p = readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 8);
			body2World.p = make_float4(p.x, p.y, p.z, 0.f);
			body2World.q = PxAlignedQuat(q.x, q.y, q.z, q.w);
			maxImpulse = readSwizzledWord<float, PxgBodySim>(bSims, readIndex, 10, 3);
			internalFlags = readSwizzledWord<PxU32, PxgBodySim>(bSims, readIndex, 11, 1);

			ushort2 tmp = readSwizzledWord<ushort2, PxgBodySim>(bSims, readIndex, 11, 2);

			lockFlags = tmp.x;
			disableGravity = tmp.y;
			offsetSlop = readSwizzledWord<PxReal, PxgBodySim>(bSims, readIndex, 11, 3);

			linearAccel = skipGravityApplication ? make_float4(0.0f) : readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 12);
			angularAccel = skipGravityApplication ? make_float4(0.0f) : readSwizzledFloat4<PxgBodySim>(bSims, readIndex, 13);
		}

		__syncwarp();
	}

	if (a < nbSolverBodies)
	{
		bodyCoreComputeUnconstrainedVelocity(maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW, linearVelocityXYZ_inverseMassW, angularVelocityXYZ_maxPenBiasW,
			lockFlags, disableGravity || skipGravityApplication, sleepAngVelAccXYZ_accelScaleW.w, gravity, linearAccel, angularAccel, dt);

		//initialize solver bodyData
		//const float4 inverseInertiaXYZ_contactReportThresholdW = bodySim.inverseInertiaXYZ_contactReportThresholdW;
		const PxVec3 inverseInertia(inverseInertiaXYZ_contactReportThresholdW.x, inverseInertiaXYZ_contactReportThresholdW.y, inverseInertiaXYZ_contactReportThresholdW.z);
		const PxVec3 safeSqrtInvInertia = computeSafeSqrtInertia(inverseInertia);
		const PxMat33 rotation(reinterpret_cast<PxQuat&>(body2World.q));
		transformInertiaTensor(safeSqrtInvInertia, rotation, sqrtInvInertia);

		gOutVelocityPool[a] = linearVelocityXYZ_inverseMassW;

		//KS - to make this compatible with the rigid body particle system, we store the angular velocity in the gOutVelocityPool
		//in momocity format!
		const PxVec3 sqrtInertiaV(safeSqrtInvInertia.x == 0.f ? 0.f : 1.f / safeSqrtInvInertia.x, safeSqrtInvInertia.y == 0.f ? 0.f : 1.f / safeSqrtInvInertia.y, 
			safeSqrtInvInertia.z == 0.f ? 0.f : 1.f / safeSqrtInvInertia.z);

		PxMat33 sqrtInertia;
		transformInertiaTensor(sqrtInertiaV, rotation, sqrtInertia);

		PxVec3 angVel(angularVelocityXYZ_maxPenBiasW.x, angularVelocityXYZ_maxPenBiasW.y, angularVelocityXYZ_maxPenBiasW.z);

		if (internalFlags & PxsRigidBody::eENABLE_GYROSCOPIC)
		{
			const PxVec3 localInertia(
				inverseInertia.x == 0.f ? 0.f : 1.f / inverseInertia.x,
				inverseInertia.y == 0.f ? 0.f : 1.f / inverseInertia.y,
				inverseInertia.z == 0.f ? 0.f : 1.f / inverseInertia.z);

			const PxVec3 localAngVel = body2World.q.rotateInv(angVel);
			const PxVec3 origMom = localInertia.multiply(localAngVel);
			const PxVec3 torque = -localAngVel.cross(origMom);
			PxVec3 newMom = origMom + torque * dt;
			const PxReal denom = newMom.magnitude();
			PxReal ratio = denom > 0.f ? origMom.magnitude() / denom : 0.f;
			newMom *= ratio;
			PxVec3 newDeltaAngVel = body2World.q.rotate(inverseInertia.multiply(newMom) - localAngVel);

			angVel += newDeltaAngVel;
		}

		angularVelocityXYZ_maxPenBiasW.x = angVel.x; angularVelocityXYZ_maxPenBiasW.y = angVel.y; angularVelocityXYZ_maxPenBiasW.z = angVel.z;

		angVel = sqrtInertia * (angVel);
		
		gOutVelocityPool[a + nbSolverBodies] = make_float4(angVel.x, angVel.y, angVel.z, angularVelocityXYZ_maxPenBiasW.w);

		//KS - TODO - coalesce theses, probably by writing out 2x float4.
		gTransforms[a] = body2World;
	}

	for (PxU32 i = 0; i < NbToRead; i += 16)
	{
		if (threadIndexInWarp >= i && threadIndexInWarp < (i + 16) && a < nbSolverBodies)
		{
			PxgSolverBodyData* bData = reinterpret_cast<PxgSolverBodyData*>(&sharedBufferSpace[warpIndex][0]);

			const PxU32 nodeIndex = sharedIslandNodeIndices[warpIndex][threadIndexInWarp];

			PxgSolverBodyData& data = bData[threadIndexInWarp & 15];
			//PxgSolverBodyData& data = solverBodyDataPool[a];

			//data.sqrtInvInertia = sqrtInvInertia;
			data.body2World = body2World;
			data.initialLinVelXYZ_invMassW = linearVelocityXYZ_inverseMassW;
			data.initialAngVelXYZ_penBiasClamp = angularVelocityXYZ_maxPenBiasW;
			data.offsetSlop = offsetSlop;

			data.reportThreshold = inverseInertiaXYZ_contactReportThresholdW.w;

			data.islandNodeIndex = PxNodeIndex(nodeIndex);
			//data.inverseInertia  = make_float4(bodySim.inverseInertiaXYZ_contactReportThresholdW.x,bodySim.inverseInertiaXYZ_contactReportThresholdW.y, bodySim.inverseInertiaXYZ_contactReportThresholdW.z, 0.f);

			data.maxImpulse = maxImpulse; //KS - can this be read in in a more efficient/better way?
			PxU32 flags = 0;
			if (internalFlags & PxsRigidBody::eSPECULATIVE_CCD)
				flags |= PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD;
			if (internalFlags & PxsRigidBody::eENABLE_GYROSCOPIC)
				flags |= PxRigidBodyFlag::eENABLE_GYROSCOPIC_FORCES;
			data.flags = flags;
		}

		__syncwarp();

		const PxU32 solverBodyDataSize = sizeof(PxgSolverBodyData) / sizeof(uint);

		PxU32 TotalUintToWrite = solverBodyDataSize * PxMin(NbToRead - i, 16u);

		uint* dst = reinterpret_cast<uint*>(&solverBodyDataPool[startReadIndex + i]);
		uint* src = reinterpret_cast<uint*>(&sharedBufferSpace[warpIndex][0]);

		for (PxU32 j = threadIndexInWarp; j < TotalUintToWrite; j += 32)
		{
			dst[j] = src[j];
		}
		__syncwarp();

		//if (threadIndexInWarp >= i && threadIndexInWarp < (i + 16) && a < nbSolverBodies)
		//{
		//	//KS - TODO - is this necessary? We now have PxgBodySim, which stores all this data.
		//	//This entire sleep data thing looks redundant now!
		//	PxgSolverBodySleepData* sData = reinterpret_cast<PxgSolverBodySleepData*>(sharedBufferSpace[warpIndex]);
		//	PxgSolverBodySleepData& sleepData = sData[threadIndexInWarp & 15];
		//	//PxgSolverBodySleepData& sleepData = solverBodySleepDataPool[a];
		//	//initialize solver body sleep data
		//	sleepData.freezeCount = sleepLinVelAccXYZ_freezeCountW.w;
		//	sleepData.sleepLinVelAcc = *reinterpret_cast<PxVec3*>(&(make_float3(sleepLinVelAccXYZ_freezeCountW).x));
		//	sleepData.accelScale = sleepAngVelAccXYZ_accelScaleW.w;
		//	sleepData.sleepAngVelAcc = *reinterpret_cast<PxVec3*>(&(make_float3(sleepAngVelAccXYZ_accelScaleW).x));

		//	sleepData.freezeThreshold = freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.x;
		//	sleepData.wakeCounter = freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.y;
		//	sleepData.sleepThreshold = freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex.z;

		//	sleepData.internalFlags = internalFlags;
		//}

		//__syncwarp();

		//const PxU32 sleepDataSize = sizeof(PxgSolverBodySleepData) / sizeof(uint);

		//TotalUintToWrite = sleepDataSize * PxMin(NbToRead - i, 16u);
		//dst = reinterpret_cast<uint*>(&solverBodySleepDataPool[startReadIndex + i]);

		//for (PxU32 j = threadIndexInWarp; j < TotalUintToWrite; j += 32)
		//{
		//	dst[j] = sharedBufferSpace[warpIndex][j];
		//}

		//__syncwarp();

		if (threadIndexInWarp >= i && threadIndexInWarp < (i + 16) && a < nbSolverBodies)
		{
			PxgSolverTxIData* bData = reinterpret_cast<PxgSolverTxIData*>(&sharedBufferSpace[warpIndex][0]);

			PxgSolverTxIData& data = bData[threadIndexInWarp & 15];
			//PxgSolverBodyData& data = solverBodyDataPool[a];

			data.sqrtInvInertia = sqrtInvInertia;
			data.deltaBody2World = PxTransform(PxIdentity);
		}
		__syncwarp();

		const PxU32 txISize = sizeof(PxgSolverTxIData) / sizeof(uint);

		TotalUintToWrite = txISize * PxMin(NbToRead - i, 16u);
		dst = reinterpret_cast<uint*>(&solverTxIDataPool[startReadIndex + i]);

		for (PxU32 j = threadIndexInWarp; j < TotalUintToWrite; j += 32)
		{
			dst[j] = sharedBufferSpace[warpIndex][j];
		}

		__syncwarp();

	}
}

