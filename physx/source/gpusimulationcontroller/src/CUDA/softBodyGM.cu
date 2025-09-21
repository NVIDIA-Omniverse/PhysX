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
#include "PxgSoftBodyCore.h"
#include "PxgSoftBody.h"
#include "PxgSoftBodyCoreKernelIndices.h"
#include "PxgBodySim.h"

#include "PxgCommonDefines.h"
#include "reduction.cuh"
#include "shuffle.cuh"
#include "atomic.cuh"
#include "gridCal.cuh"
#include "copy.cuh"
#include "softBody.cuh"
#include "deformableUtils.cuh"
#include "attachments.cuh"
#include "particleSystem.cuh"
#include "utils.cuh"

#include "stdio.h"
#include "PxgSolverBody.h"
#include "PxgSolverCoreDesc.h"
#include "PxNodeIndex.h"
#include "assert.h"
#include "GuBV32.h"
#include "sbMidphaseScratch.cuh"


#include "PxgSimulationCoreDesc.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgFEMCloth.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "cudaNpCommon.h"
#include "dataReadWriteHelper.cuh"

using namespace physx;

extern "C" __host__ void initSoftBodyKernels1() {}


//This kernel is called before refitBound
extern "C" __global__ void sb_gm_preIntegrateLaunch(
	PxgSoftBody* PX_RESTRICT softbodies,
	const PxU32* activeId,
	const PxVec3 gravity,
	const PxReal dt,
	const bool isTGS,
	PxReal* speculativeCCDContactOffset,
	const bool externalForcesEveryTgsIterationEnabled)
{
	const PxU32 id = activeId[blockIdx.y];
	PxgSoftBody& softbody = softbodies[id];

	//32 warps in one block
	__shared__ PxReal maxMagnitudeVelSq[32];

	const PxU32 nbVerts = softbody.mNumVertsGM;
	float4* PX_RESTRICT positions = softbody.mSimPosition_InvMass;
	float4* PX_RESTRICT velocities = softbody.mSimVelocity_InvMass;
	bool* PX_RESTRICT vertsDeformed = softbody.mVertsAreDeformed;
	bool* PX_RESTRICT vertsCanNotDeform = softbody.mVertsCanNotDeform;

	float4* PX_RESTRICT posDelta = softbody.mSimDeltaPos;

	const PxU32 threadIdxInWarp = threadIdx.x;
	const PxU32 warpIndex = threadIdx.y;

	const PxU32 globalThreadIndex = threadIdxInWarp + WARP_SIZE * warpIndex + blockDim.x*blockDim.y*blockIdx.x;


	if (globalThreadIndex == 0)
	{
		softbody.mAwake = false;
	}
	
	PxReal maxMagVelSq = 0.f;

	if (softbody.mBodyFlags & PxDeformableBodyFlag::eKINEMATIC)
	{
		const float4* PX_RESTRICT kinematicTargets = softbody.mSimKinematicTarget;

		if (globalThreadIndex < nbVerts && kinematicTargets)
		{
			float4 tPos = positions[globalThreadIndex];
			PxVec3 pos(tPos.x, tPos.y, tPos.z);
			float4 tKinematicPos = kinematicTargets[globalThreadIndex];
			PxVec3 kinematicPos(tKinematicPos.x, tKinematicPos.y, tKinematicPos.z);

			PxVec3 delta = kinematicPos - pos;
			posDelta[globalThreadIndex] = make_float4(delta.x, delta.y, delta.z, tPos.w);

			positions[globalThreadIndex] = tKinematicPos;

			float4 tVel = velocities[globalThreadIndex];
			PxVec3 vel(tVel.x, tVel.y, tVel.z);
			vel += delta / dt;

			velocities[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, 0.f);

			vertsDeformed[globalThreadIndex] = false;
			vertsCanNotDeform[globalThreadIndex] = false;

			maxMagVelSq = vel.magnitudeSquared();
		}
	}
	else
	{
		if (globalThreadIndex < nbVerts)
		{
			float4 tPos = positions[globalThreadIndex];
			PxVec3 vel(0.f);
			PxVec3 oldPos(tPos.x, tPos.y, tPos.z);
			
			bool kinematic = false;
			if (softbody.mVolumeFlags & PxDeformableVolumeFlag::ePARTIALLY_KINEMATIC)
			{
				const float4* PX_RESTRICT kinematicTargets = softbody.mSimKinematicTarget;
				if (kinematicTargets)
				{
					float4 tKinematicPos = kinematicTargets[globalThreadIndex];
					if (tKinematicPos.w == 0) 
					{
						PxVec3 kinematicPos(tKinematicPos.x, tKinematicPos.y, tKinematicPos.z);
						vel = (kinematicPos - oldPos) / dt;
						kinematic = true;
						tPos.w = 0.0f;
					}
				}
			}		

			if (tPos.w != 0.f && !kinematic)
			{
				float4 oldVel = velocities[globalThreadIndex];
				vel = PxVec3(oldVel.x, oldVel.y, oldVel.z);
				if (!externalForcesEveryTgsIterationEnabled)
				{
					if(!(softbody.mActorFlags & PxActorFlag::eDISABLE_GRAVITY))
					{
						vel += gravity * dt;
					}
					const PxReal damping = 1.f - PxMin(dt * softbody.mLinearDamping, 1.f);
					vel *= damping;
				}
			}

			// Max velocity clamping
			const PxReal maxVel = softbody.mMaxLinearVelocity;
			const PxReal velMagSq = vel.magnitudeSquared();

			vel = (maxVel != PX_MAX_REAL && maxVel * maxVel < velMagSq) ? maxVel / PxSqrt(velMagSq) * vel : vel;

			PxVec3 delta(0.f);
			if (!isTGS)
			{
				delta += vel * dt;
			}

			PxVec3 pos = oldPos + delta;
			//The positions are still when gravity is applied every substep such that refit bounds of the bvh and the 
			//following collision detection don't miss potential contacts towards the end of the time step
			positions[globalThreadIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);

			velocities[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, tPos.w);

			posDelta[globalThreadIndex] = make_float4(delta.x, delta.y, delta.z, tPos.w);

			vertsDeformed[globalThreadIndex] = false;
			vertsCanNotDeform[globalThreadIndex] = false;

			maxMagVelSq = vel.magnitudeSquared();

			//printf("velocities[%i](%f, %f, %f)\n", globalThreadIndex, vel.x, vel.y, vel.z);

			/*printf("=========================================================\n");
			printf("pre integrate before gravity pos(%f, %f, %f)\n", tPos.x, tPos.y, tPos.z);
			printf("pre integrate after gravtity pos(%f, %f, %f)\n", pos.x, pos.y, pos.z);
			printf("=========================================================\n");*/
		}
	}

	if (softbody.mBodyFlags & PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD)
	{
		maxMagnitudeVelSq[warpIndex] = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxMagVelSq);

		__syncthreads();

		//max vert vel in a block
		if (warpIndex == 0)
		{
			PxReal maxMagInBlock = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxMagnitudeVelSq[threadIdxInWarp]);

			if (threadIdxInWarp == 0)
			{
				maxMagInBlock = PxSqrt(maxMagInBlock) * dt;
				AtomicMax(&speculativeCCDContactOffset[id], maxMagInBlock);
				//printf("softBodyId %i maxMagInBlock %f\n", id, maxMagInBlock);
			}
		}
	}
}

extern "C" __global__ void sb_gm_stepSoftbodyLaunch(
	const PxgSoftBody* PX_RESTRICT softbodies,
	const PxU32* activeId,
	const PxReal dt,
	const PxVec3 gravity,
	const bool externalForcesEveryTgsIterationEnabled)
{
	const PxU32 id = activeId[blockIdx.y];
	const PxgSoftBody& softbody = softbodies[id];

	const PxU32 nbVerts = softbody.mNumVertsGM;
	float4* PX_RESTRICT velocities = softbody.mSimVelocity_InvMass;
	float4* PX_RESTRICT positions = softbody.mSimPosition_InvMass;
	
	float4* PX_RESTRICT posDelta = softbody.mSimDeltaPos;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x*blockIdx.x;

	if (globalThreadIndex < nbVerts)
	{
		const float4 tPos = positions[globalThreadIndex];
		const float4 pDelta = posDelta[globalThreadIndex];
		const float4 v = velocities[globalThreadIndex];
		PxVec3 vel(0.f);
		const PxVec3 oldPos(tPos.x, tPos.y, tPos.z);
		//if (tPos.w != 0.f) // @@@ VR: this doesn't look right and break the collision of kinematic softbodies
			vel = PxVec3(v.x, v.y, v.z);

		if (externalForcesEveryTgsIterationEnabled) 
		{
			if (tPos.w != 0.f) 
			{
				if(!(softbody.mActorFlags & PxActorFlag::eDISABLE_GRAVITY))
				{
					vel += gravity * dt;
				}
				const PxReal damping = 1.f - PxMin(dt * softbody.mLinearDamping, 1.f);
				vel *= damping;
			}

			// Max velocity clamping
			const PxReal maxVel = softbody.mMaxLinearVelocity;
			const PxReal velMagSq = vel.magnitudeSquared();
			vel = (maxVel != PX_MAX_REAL && maxVel * maxVel < velMagSq) ? maxVel / PxSqrt(velMagSq) * vel : vel;

			velocities[globalThreadIndex] = make_float4(vel.x, vel.y, vel.z, v.w);
		}	
		
		PxVec3 delta = vel * dt;

		PxVec3 pos = oldPos + delta;
		positions[globalThreadIndex] = make_float4(pos.x, pos.y, pos.z, tPos.w);
		posDelta[globalThreadIndex] = make_float4(pDelta.x + delta.x, pDelta.y + delta.y, pDelta.z + delta.z, tPos.w);
	}
}

__device__ void computeGMTetrahedronRotations(const PxVec3& u1, const PxVec3& u2, const PxVec3& u3, 
	const PxMat33& Qinv, PxQuat& q, bool isFirstIteration)
{
	//compute displacement field
	const PxMat33 P = PxMat33(u1, u2, u3);

	PxMat33 F = P * Qinv;
	const PxU32 iters = isFirstIteration ? 100 : 4;
	extractRotation(F, q, iters);  
	//sb_extractRotationAPD(F, q, iters);
}

__device__ void computeSimTetrahedronRotations(const uint4* tetmeshTetIndices, const PxU32 tetrahedronIdx,
	const float4* tetmeshVerts, const PxgMat33Block* restPoses, float4* tetRotations, const PxQuat initialRotation)
{
	const uint4 tetInd = tetmeshTetIndices[tetrahedronIdx];
	const float4 v0 = tetmeshVerts[tetInd.x];
	const float4 v1 = tetmeshVerts[tetInd.y];
	const float4 v2 = tetmeshVerts[tetInd.z];
	const float4 v3 = tetmeshVerts[tetInd.w];

	/*if (sourceInd == testInd)
	{
		printf("vertInd(%i, %i, %i %i)\n", tetInd.x, tetInd.y, tetInd.z, tetInd.w);
		printf("x0(%.10f, %.10f, %.10f, %.10f)\n", v0.x, v0.y, v0.z, v0.w);
		printf("x1(%.10f, %.10f, %.10f, %.10f)\n", v1.x, v1.y, v1.z, v1.w);
		printf("x2(%.10f, %.10f, %.10f, %.10f)\n", v2.x, v2.y, v2.z, v2.w);
		printf("x3(%.10f, %.10f, %.10f, %.10f)\n", v3.x, v3.y, v3.z, v3.w);
	}*/

	//compute displacement field
	const PxVec3 u1 = PxLoad3(v1 - v0);
	const PxVec3 u2 = PxLoad3(v2 - v0);
	const PxVec3 u3 = PxLoad3(v3 - v0);

	// load rest pose
	PxMat33 Qinv;
	loadPxMat33(restPoses[tetrahedronIdx/32], tetrahedronIdx&31, Qinv);

	const PxMat33 P = PxMat33(u1, u2, u3);

			// calculate deformation gradient
	PxMat33 F = P * Qinv;

			// Co-rotational strain
	float4 tetR = tetRotations[tetrahedronIdx];

	PxQuat q(tetR.x, tetR.y, tetR.z, tetR.w);

	// initialize to identity quat if this is the first iteration (zero initialized memory)
	if (q.magnitudeSquared() == 0.0f)
		q = initialRotation;


	//sb_extractRotationAPD(F, q, 100);
	extractRotation(F, q, 100);

	tetRotations[tetrahedronIdx] = make_float4(q.x, q.y, q.z, q.w);
}

__device__ void computeTetrahedronRotations(const uint4* tetmeshTetIndices, const PxU32 tetrahedronIdx,
	const float4* tetmeshVerts, const PxMat33* restPoses, float4* tetRotations, const PxQuat initialRotation)
{
	const uint4 tetInd = tetmeshTetIndices[tetrahedronIdx];
	const float4 v0 = tetmeshVerts[tetInd.x];
	const float4 v1 = tetmeshVerts[tetInd.y];
	const float4 v2 = tetmeshVerts[tetInd.z];
	const float4 v3 = tetmeshVerts[tetInd.w];

	/*if (sourceInd == testInd)
	{
		printf("vertInd(%i, %i, %i %i)\n", tetInd.x, tetInd.y, tetInd.z, tetInd.w);
		printf("x0(%.10f, %.10f, %.10f, %.10f)\n", v0.x, v0.y, v0.z, v0.w);
		printf("x1(%.10f, %.10f, %.10f, %.10f)\n", v1.x, v1.y, v1.z, v1.w);
		printf("x2(%.10f, %.10f, %.10f, %.10f)\n", v2.x, v2.y, v2.z, v2.w);
		printf("x3(%.10f, %.10f, %.10f, %.10f)\n", v3.x, v3.y, v3.z, v3.w);
	}*/

	//compute displacement field
	const PxVec3 u1 = PxLoad3(v1 - v0);
	const PxVec3 u2 = PxLoad3(v2 - v0);
	const PxVec3 u3 = PxLoad3(v3 - v0);

	// load rest pose
	PxMat33 Qinv = restPoses[tetrahedronIdx];
	
	/*if (sourceInd == testInd)
		printf("%i Qinv c0(%f, %f, %f), c1(%f, %f, %f), c2(%f, %f, %f)\n", tetrahedronIdx,
			Qinv.column0.x, Qinv.column0.y, Qinv.column0.z,
			Qinv.column1.x, Qinv.column1.y, Qinv.column1.z,
			Qinv.column2.x, Qinv.column2.y, Qinv.column2.z);*/


	const PxMat33 P = PxMat33(u1, u2, u3);

	/*if (tetrahedronIdx == 0)
		printf("%i P c0(%f, %f, %f), c1(%f, %f, %f), c2(%f, %f, %f)\n", tetrahedronIdx,
			P.column0.x, P.column0.y, P.column0.z,
			P.column1.x, P.column1.y, P.column1.z,
			P.column2.x, P.column2.y, P.column2.z);*/

			// calculate deformation gradient
	PxMat33 F = P * Qinv;

	/*if (sourceInd == testInd)
		printf("%i F c0(%f, %f, %f), c1(%f, %f, %f), c2(%f, %f, %f)\n", tetrahedronIdx,
			F.column0.x, F.column0.y, F.column0.z,
			F.column1.x, F.column1.y, F.column1.z,
			F.column2.x, F.column2.y, F.column2.z);*/

			// Co-rotational strain
	float4 tetR = tetRotations[tetrahedronIdx];

	PxQuat q(tetR.x, tetR.y, tetR.z, tetR.w);


	// initialize to identity quat if this is the first iteration (zero initialized memory)
	if (q.magnitudeSquared() == 0.0f)
		q = initialRotation;

	/*if (sourceInd == testInd)
		printf("before tetRot[%i](%.10f, %.10f, %.10f, %.10f)\n", tetrahedronIdx, q.x, q.y, q.z, q.w);*/

	//sb_extractRotationAPD(F, q, 100);
	extractRotation(F, q, 100);

	tetRotations[tetrahedronIdx] = make_float4(q.x, q.y, q.z, q.w);
}

// updates the rotational factor for each tetrahedron, once per frame
extern "C" __global__ void sb_gm_updateTetrahedraRotationsLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies)
{

	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	const PxU32 tetrahedronIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbTets = shSoftbody.mNumTetsGM;

	if (tetrahedronIdx < nbTets )
	{
		const PxQuat initialRotation = softbody.mInitialRotation;
		const uint4* tetIndicesGM = softbody.mSimTetIndices;
		const float4 * vertsGM = softbody.mSimPosition_InvMass;

		const PxgMat33Block* restPosesGM = softbody.mSimTetraRestPoses;
		//this is quaternion
		float4* tetRotationsGM = softbody.mSimTetraRotations;

		//storeSpatialVector(softbody.mSimTetraMultipliers[tetrahedronIdx/32], Cm::UnAlignedSpatialVector::Zero(), tetrahedronIdx&31);

		computeSimTetrahedronRotations(tetIndicesGM, tetrahedronIdx,
			vertsGM, restPosesGM, tetRotationsGM, initialRotation);

	}
}


// updates the rotational factor for each tetrahedron for collision mesh, once per frame
extern "C" __global__ void sb_updateTetrahedraRotationsLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies)
{

	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	const PxU32 tetrahedronIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbTets = shSoftbody.mNumTets;

	//KS - if we don't need stress tensors, we don't need to compute the 
	//tet rotations.
	if (shSoftbody.mVolumeFlags & PxDeformableVolumeFlag::eCOMPUTE_STRESS_TENSOR ||
		!(shSoftbody.mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION))
	{
		if (tetrahedronIdx < nbTets)
		{
			const PxQuat initialRotation = softbody.mInitialRotation;

			const uint4* tetIndices = softbody.mTetIndices;
			const float4 * verts = softbody.mPosition_InvMass;

			const PxMat33* restPoses = softbody.mTetraRestPoses;
			//this is quaternion
			float4* tetRotations = softbody.mTetraRotations;

			computeTetrahedronRotations(tetIndices, tetrahedronIdx,
				verts, restPoses, tetRotations, initialRotation);

		}
	}
}

static PX_FORCE_INLINE __device__ PxReal evalStiffnessMatrix6x6(PxU32 row, PxU32 col, PxReal invE, PxReal nu)
{
	PxReal result = 0.0f;
	if (row == col)
	{
		result = invE;
		if (row >= 3)
			result *= 1.0f + nu;
	}
	else if (row < 3 && col < 3)
		result = -invE * nu;
	return result;
}

static __device__ inline float multiplyStrainByCompliance(PxReal invE, PxReal nu, int strainIndex, const Cm::UnAlignedSpatialVector& lambda)
{
	float s = lambda.top.x * evalStiffnessMatrix6x6(strainIndex, 0, invE, nu) +
		lambda.top.y *    evalStiffnessMatrix6x6(strainIndex, 1, invE, nu) +
		lambda.top.z *    evalStiffnessMatrix6x6(strainIndex, 2, invE, nu) +
		lambda.bottom.x * evalStiffnessMatrix6x6(strainIndex, 3, invE, nu) +
		lambda.bottom.y * evalStiffnessMatrix6x6(strainIndex, 4, invE, nu) +
		lambda.bottom.z * evalStiffnessMatrix6x6(strainIndex, 5, invE, nu);

	return s;

}

// compute dC/dx = dF/dx:dC/dF, where x: x0, x1, x2, x3, C: constraint, F: deformation gradient, DmInv: inverse of rest
// pos matrix Dm, where F*Ds = Dm.
static PX_FORCE_INLINE __device__ void compute_dCdx(PxVec3& dCdx1, PxVec3& dCdx2, PxVec3& dCdx3, const PxMat33& dCdF,
                                                    const PxMat33& DmInv)
{
	// row vectors of DmInv
	const PxVec3 r0(DmInv.column0[0], DmInv.column1[0], DmInv.column2[0]);
	const PxVec3 r1(DmInv.column0[1], DmInv.column1[1], DmInv.column2[1]);
	const PxVec3 r2(DmInv.column0[2], DmInv.column1[2], DmInv.column2[2]);

	// row vectors of dCdF
	const PxVec3 p0(dCdF.column0[0], dCdF.column1[0], dCdF.column2[0]);
	const PxVec3 p1(dCdF.column0[1], dCdF.column1[1], dCdF.column2[1]);
	const PxVec3 p2(dCdF.column0[2], dCdF.column1[2], dCdF.column2[2]);

	dCdx1 = PxVec3(r0.dot(p0), r0.dot(p1), r0.dot(p2));
	dCdx2 = PxVec3(r1.dot(p0), r1.dot(p1), r1.dot(p2));
	dCdx3 = PxVec3(r2.dot(p0), r2.dot(p1), r2.dot(p2));

	// dCdx0 = PxVec3(-s.dot(p0), -s.dot(p1), -s.dot(p2)) = -dCdx1 - dCdx2 - dCdx3
	// where s = PxVec3(s0, s1, s2), and s_i means sum of entries of column i of DmInv
}

// compute deltaLambda * gradC
static PX_FORCE_INLINE __device__ void compute_deltaLambdaXgradC(PxMat33& deltaLambdaXgradC, PxReal& lambda, PxReal C,
                                                                 const PxMat33& dCdF, const PxMat33& DmInv,
                                                                 const float4& v0, const float4& v1, const float4& v2,
                                                                 const float4& v3, PxReal alphaTilde, PxReal dtInv,
                                                                 PxReal damping, PxReal scaledDamping)
{
	PxVec3 dCdx1, dCdx2, dCdx3;
	compute_dCdx(dCdx1, dCdx2, dCdx3, dCdF, DmInv);

	PxVec3 dCdx0 = -dCdx1 - dCdx2 - dCdx3;
	PxReal deltaLambda;

	if(damping == 0.f) // no damping
	{
		const PxReal denom = dCdx0.magnitudeSquared() * v0.w + dCdx1.magnitudeSquared() * v1.w +
		                     dCdx2.magnitudeSquared() * v2.w + dCdx3.magnitudeSquared() * v3.w + alphaTilde;
		if(PxAbs(denom) < 1.0e-15f)
			return;

		deltaLambda = (-C - alphaTilde * lambda) / denom;
	}
	else
	{
		const PxReal denom =
		    (1.f + damping * dtInv) * (dCdx0.magnitudeSquared() * v0.w + dCdx1.magnitudeSquared() * v1.w +
		                               dCdx2.magnitudeSquared() * v2.w + dCdx3.magnitudeSquared() * v3.w) +
		    alphaTilde;

		if(PxAbs(denom) < 1.0e-15f)
		{
			deltaLambdaXgradC = PxMat33(PxZero);
			return;
		}

		deltaLambda = (-C - alphaTilde * lambda -
		               scaledDamping * (dCdx0.dot(PxLoad3(v0)) + dCdx1.dot(PxLoad3(v1)) + dCdx2.dot(PxLoad3(v2)) +
		                                dCdx3.dot(PxLoad3(v3)))) /
		              denom;
	}

	lambda += deltaLambda;

	// return: deltaLambda * dCdx
	deltaLambdaXgradC.column0 += deltaLambda * dCdx1;
	deltaLambdaXgradC.column1 += deltaLambda * dCdx2;
	deltaLambdaXgradC.column2 += deltaLambda * dCdx3;
}

template <const PxU32 indexA, const PxU32 indexB, const PxU32 strainIndex>
__device__ PxReal computeLambda(const PxMat33& G, const PxMat33& E, const Cm::UnAlignedSpatialVector& totalLambda,
	const PxReal gamma, const PxReal scale, PxReal invE, PxReal nu, const PxReal invDtSq)
{
	return -(G(indexA, indexB) + invDtSq * multiplyStrainByCompliance(invE, nu, strainIndex, totalLambda) + gamma * E(indexA, indexB))*scale;
}

template <const PxU32 indexA, const PxU32 indexB>
__device__ PxReal computeLambda(const PxMat33& G, const PxMat33& E, const PxReal gamma, const PxReal scale, const PxReal invDtSq)
{
	return -(G(indexA, indexB) +  gamma * E(indexA, indexB))*scale;
}



PX_FORCE_INLINE __device__ PxMat33 volumeSolve(const PxMat33& P,
	const float4& v0, const float4& v1, const float4& v2, const float4& v3,
	const PxReal invDtSq,
	const PxMat33& Qinv, bool isTGS,
	const PxReal volumeLambda)
{
	PxReal det = Qinv.getDeterminant();
	PxReal volume = 1.0f / (6.0f* det);

	const PxReal alphaTilde = (volumeLambda == 0.f) ? 0.f : 1.f / (volumeLambda * volume * invDtSq);

	PxMat33 F = P * Qinv;

	const PxVec3 g0 = F.column1.cross(F.column2);
	const PxVec3 g1 = F.column2.cross(F.column0);
	const PxVec3 g2 = F.column0.cross(F.column1);

	PxVec3 grad1 = g0 * Qinv.column0.x + g1 * Qinv.column1.x + g2 * Qinv.column2.x;
	PxVec3 grad2 = g0 * Qinv.column0.y + g1 * Qinv.column1.y + g2 * Qinv.column2.y;
	PxVec3 grad3 = g0 * Qinv.column0.z + g1 * Qinv.column1.z + g2 * Qinv.column2.z;
	PxVec3 grad0 = -grad1 - grad2 - grad3;

	PxReal detF = g2.dot(F.column2);

	PxReal C = (detF - 1.f);

	PxReal w = grad0.magnitudeSquared()*v0.w
		+ grad1.magnitudeSquared()*v1.w
		+ grad2.magnitudeSquared()*v2.w
		+ grad3.magnitudeSquared()*v3.w;

	const PxReal dlambda = -C / (w + alphaTilde);

	return PxMat33(grad1 * dlambda, grad2*dlambda, grad3*dlambda);
}

PX_FORCE_INLINE __device__ PxMat33 tetrahedronsSolveInnerNeoHookean(const PxMat33& P,
	const float4& v0, const float4& v1, const float4& v2, const float4& v3,
	PxgSoftBody& shSoftbody,
	const float invDtSq, const float invDt,
	const PxU32 workIndex,
	const PxMat33& Qinv, bool isTGS,
	const PxReal invE)
{
	PxMat33 F = P * Qinv;


	PxVec3 g1 = F.column0 * 2.0f * Qinv.column0.x +
		F.column1 * 2.0f * Qinv.column1.x +
		F.column2 * 2.0f * Qinv.column2.x;

	PxVec3 g2 = F.column0 * 2.0f * Qinv.column0.y +
		F.column1 * 2.0f * Qinv.column1.y +
		F.column2 * 2.0f * Qinv.column2.y;

	PxVec3 g3 = F.column0 * 2.0f * Qinv.column0.z +
		F.column1 * 2.0f * Qinv.column1.z +
		F.column2 * 2.0f * Qinv.column2.z;

	PxReal C = F.column0.magnitudeSquared() + F.column1.magnitudeSquared() + F.column2.magnitudeSquared() - 3.0f;


	PxVec3 g0 = -g1 - g2 - g3;

	PxReal w = g0.magnitudeSquared() * v0.w +
		g1.magnitudeSquared() * v1.w +
		g2.magnitudeSquared() * v2.w +
		g3.magnitudeSquared() * v3.w;



	// use the first element which for linear isotropic materials is a rough average of compliance (see XPD)
	// invE is 1.0/youngs, see PxgShapeManager.cpp line 609
	const PxReal alpha = invDtSq * invE;
	PxReal dlambda;

	if (!isTGS)
	{
		float2 totalLambda = shSoftbody.mSimTetraMultipliers[workIndex / 32].mbyz[workIndex & 31];

		dlambda = -(C + alpha * totalLambda.x) / (w + alpha);

		totalLambda.x += dlambda;
		shSoftbody.mSimTetraMultipliers[workIndex / 32].mbyz[workIndex & 31] = totalLambda;
	}
	else
	{
		dlambda = -C / (w + alpha);
	}
	
	PxMat33 delta;
	delta.column0 = g1 * dlambda;
	delta.column1 = g2 * dlambda;
	delta.column2 = g3 * dlambda;

	return delta;
}

#define SB_ARAP_EPSILON 1.0e-18f

// ARAP: C = Sqrt(||F - R||_F^2)
PX_FORCE_INLINE __device__ void ARAP_constraint(PxMat33& deltaLambdaXgradC, PxReal& lambda, const float4& v0,
                                                const float4& v1, const float4& v2, const float4& v3, const PxQuat& tetR,
                                                const PxMat33& F, PxReal dtInv, const PxMat33& DmInv, PxReal alphaTilde,
                                                PxReal damping = 0.f, PxReal scaledDamping = 0.f)
{
	const PxMat33 R(tetR);
	PxMat33 tensor = F - R;

	const PxReal ARAP_C = PxSqrt(tensor.column0.dot(tensor.column0) + tensor.column1.dot(tensor.column1) +
	                             tensor.column2.dot(tensor.column2) + SB_ARAP_EPSILON);

	tensor *= (1.f / ARAP_C); // dCdF
	compute_deltaLambdaXgradC(deltaLambdaXgradC, lambda, ARAP_C, tensor, DmInv, v0, v1, v2, v3, alphaTilde, dtInv,
	                          damping, scaledDamping);
}

// trace(S - I): C = tr(S - I), F = RS
PX_FORCE_INLINE __device__ void traceSMinusI_constraint(PxMat33& deltaLambdaXgradC, PxReal& lambda, const float4& v0,
                                                        const float4& v1, const float4& v2, const float4& v3,
                                                        const PxQuat& tetR, const PxMat33& F, PxReal invDt,
                                                        const PxMat33& DmInv, PxReal alphaTilde, PxReal damping = 0.f,
                                                        PxReal scaledDamping = 0.f)
{
	const PxMat33 R(tetR);
	PxMat33 S = R.getTranspose() * F; // to do: convert R^T*F into simpler computation.

	const PxReal C = S.column0.x + S.column1.y + S.column2.z - 3.f;

	// dCdF = R;
	compute_deltaLambdaXgradC(deltaLambdaXgradC, lambda, C, R, DmInv, v0, v1, v2, v3, alphaTilde, invDt, damping,
	                          scaledDamping);
}

// volume preservation: C = (J - 1), where J is det(F).
PX_FORCE_INLINE __device__ void volume_constraint(PxMat33& deltaLambdaXgradC, PxReal& lambda, const float4& v0,
                                                  const float4& v1, const float4& v2, const float4& v3,
                                                  const PxMat33& F, PxReal invDt, const PxMat33& DmInv,
                                                  PxReal alphaTilde, PxReal damping = 0.f, PxReal scaledDamping = 0.f)
{
	const PxReal detF = F.getDeterminant();
	const PxReal C = detF - 1.f;
	PxMat33 dCdF(F.column1.cross(F.column2), F.column2.cross(F.column0), F.column0.cross(F.column1));

	compute_deltaLambdaXgradC(deltaLambdaXgradC, lambda, C, dCdF, DmInv, v0, v1, v2, v3, alphaTilde, invDt, damping,
	                          scaledDamping);
}

PX_FORCE_INLINE __device__ PxMat33 tetrahedronsSolveInner(const PxMat33& P,
	const float4& v0, const float4& v1, const float4& v2, const float4& v3,
	PxgSoftBody& shSoftbody,
	const float invDtSq, const float invDt,
	const PxU32 workIndex,
	const PxMat33& Qinv, const PxMat33& RTranspose, bool isTGS,
	PxsDeformableVolumeMaterialData& material)
{
	//PxQuat& q = reinterpret_cast<PxQuat&>(tetR);

	PxMat33 F = P * Qinv;

	// recover the original tet volume from inverse rest pose
	PxReal det = Qinv.getDeterminant();
	float volume = 1.0f / (6.0f* det);
	float volumeSqrt = sqrtf(volume);


	// Cauchy strain tensor
	PxMat33 G = (F + F.getTranspose())*0.5f;
	G.column0.x -= 1.0f;
	G.column1.y -= 1.0f;
	G.column2.z -= 1.0f;

	// convert from strain energy density to integral over volume (sqrt due to quadratic form of energy)
	G *= volumeSqrt;


	// use the first element which for linear isotropic materials is a rough average of compliance (see XPD)
	const float alpha = invDtSq * (1.0f / material.youngs);

	// Damping
	const float damping = material.elasticityDamping;
	const float scaledDamping = damping * toUniformReal(material.dampingScale);

	/*const float youngs = material.youngs;
	const float poissons = material.poissons;
	const float deformThreshold = material.deformThreshold;
	const float lowLimit = material.deformLowLimitRatio;
	const float highLimit = material.deformHighLimitRatio;
	if (workIndex == 0)
	{
		printf("invDtSq %f alpha %f, alpha2 %f, damping %f, scaledDamping %f youngs %f, poissons %f, deformThreshold %f, lowLimit %f, highLimit %f\n", invDtSq, alpha, alpha2, damping, scaledDamping,
			youngs, poissons, deformThreshold, lowLimit, highLimit);
	}*/
	const PxVec3 uv1 = PxLoad3(v1 - v0);
	const PxVec3 uv2 = PxLoad3(v2 - v0);
	const PxVec3 uv3 = PxLoad3(v3 - v0);


	// Velocity gradient
	const PxMat33 Vg = calculateDeformationGradient(uv1, uv2, uv3, Qinv, RTranspose);
	//PxMat33 Vg = Transpose(R)*(Matrix33(Vec3(x1 - p1), Vec3(x2 - p2), Vec3(x3 - p3))*Qinv);

	// Strain rate
	PxMat33 E = (Vg + Vg.getTranspose()) * 0.5f;
	E *= volumeSqrt;


	// use a conservative maximum scale
	const PxReal sqLength1 = Qinv.column0.magnitudeSquared();
	const PxReal sqLength2 = Qinv.column1.magnitudeSquared();
	const PxReal sqLength3 = Qinv.column2.magnitudeSquared();
	const PxReal sqLengthSum = (Qinv.column0 + Qinv.column1 + Qinv.column2).magnitudeSquared();

	const PxReal common = (1.0f + damping * invDt) * (volumeSqrt * volumeSqrt *(sqLength1 * v1.w + sqLength2 * v2.w + sqLength3 * v3.w + sqLengthSum * v0.w));
	float scale = 1.0f / (common + alpha);

	Cm::UnAlignedSpatialVector lambda;

	if (!isTGS)
	{
		PxReal invE = 1.0f / material.youngs;
		PxReal nu = material.poissons;

		Cm::UnAlignedSpatialVector totalLambda = loadSpatialVector(shSoftbody.mSimTetraMultipliers[workIndex / 32], workIndex & 31);
		lambda.top.x = computeLambda<0, 0, 0>(G, E, totalLambda, scaledDamping, scale, invE, nu, invDtSq);
		lambda.top.y = computeLambda<1, 1, 1>(G, E, totalLambda, scaledDamping, scale, invE, nu, invDtSq);
		lambda.top.z = computeLambda<2, 2, 2>(G, E, totalLambda, scaledDamping, scale, invE, nu, invDtSq);
		lambda.bottom.x = computeLambda<1, 2, 3>(G, E, totalLambda, scaledDamping, scale, invE, nu, invDtSq);
		lambda.bottom.y = computeLambda<0, 2, 4>(G, E, totalLambda, scaledDamping, scale, invE, nu, invDtSq);
		lambda.bottom.z = computeLambda<0, 1, 5>(G, E, totalLambda, scaledDamping, scale, invE, nu, invDtSq);
		totalLambda += lambda;
		storeSpatialVector(shSoftbody.mSimTetraMultipliers[workIndex / 32], totalLambda, workIndex & 31);
	}
	else
	{
		lambda.top.x = computeLambda<0, 0>(G, E, scaledDamping, scale, invDtSq);
		lambda.top.y = computeLambda<1, 1>(G, E, scaledDamping, scale, invDtSq);
		lambda.top.z = computeLambda<2, 2>(G, E, scaledDamping, scale, invDtSq);
		lambda.bottom.x = computeLambda<1, 2>(G, E, scaledDamping, scale, invDtSq);
		lambda.bottom.y = computeLambda<0, 2>(G, E, scaledDamping, scale, invDtSq);
		lambda.bottom.z = computeLambda<0, 1>(G, E, scaledDamping, scale, invDtSq);
	}
		
	

	const PxReal halfSqrtVolume = (0.5f*volumeSqrt);
	// e00 (Voigt 0)
	//PxMat33 grad = (Outer(PxVec3(1.0f, 0.0f, 0.0f), Qinv.column0) + Outer(PxVec3(1.0f, 0.0f, 0.0f), Qinv.column0)) * halfVolume;
	PxMat33 grad = PxMat33::outer(PxVec3(volumeSqrt, 0.0f, 0.0f), Qinv.column0);
	
	//totalLambda[0] = totalLambda[0] + deltaLambda0;
	//deltaLambda = -(G(0, 0) + invDtSq * multiplyStrainByCompliance(shSoftbody.mMaterial, 0, totalLambda) + gamma * E(0, 0))*scale;
	PxMat33 delta = grad * lambda.top.x;
	

	// e11 (Voigt 1)
	//grad = (Outer(PxVec3(0.0f, 1.0f, 0.0f), Qinv.column1) + Outer(PxVec3(0.0f, 1.0f, 0.0f), Qinv.column1)) * halfVolume;
	grad = PxMat33::outer(PxVec3(0.0f, volumeSqrt, 0.0f), Qinv.column1);
	//deltaLambda = -(G(1, 1) + invDtSq * multiplyStrainByCompliance(shSoftbody.mMaterial, 1, totalLambda) + gamma * E(1, 1))*scale;
	
	delta += grad * lambda.top.y;
	//totalLambda[1] = totalLambda[1] + deltaLambda1;

	// e22 (Voigt 2)
	//grad = (Outer(PxVec3(0.0f, 0.0f, 1.0f), Qinv.column2) + Outer(PxVec3(0.0f, 0.0f, 1.0f), Qinv.column2)) * halfVolume;
	grad = PxMat33::outer(PxVec3(0.0f, 0.0f, volumeSqrt), Qinv.column2);
	//deltaLambda = -(G(2, 2) + invDtSq * multiplyStrainByCompliance(shSoftbody.mMaterial, 2, totalLambda) + gamma * E(2, 2))*scale;
	
	delta += grad * lambda.top.z;
	//totalLambda[2] = totalLambda[2] + deltaLambda2;

	if (1)
	{
		// e12 (Voigt 3)
		//grad = (Outer(PxVec3(0.0f, 1.0f, 0.0f), Qinv.column2) + Outer(PxVec3(0.0f, 0.0f, 1.0f), Qinv.column1)) * halfVolume;
		grad = (PxMat33::outer(PxVec3(0.0f, halfSqrtVolume, 0.0f), Qinv.column2) + PxMat33::outer(PxVec3(0.0f, 0.0f, halfSqrtVolume), Qinv.column1));
		//deltaLambda = -(G(1, 2) + invDtSq * multiplyStrainByCompliance(shSoftbody.mMaterial, 3, totalLambda) + gamma * E(1, 2))*scale;

		delta += grad * lambda.bottom.x;
		//totalLambda[3] = totalLambda[3] + deltaLambda3;

		// e02 (Voigt 4)
		//grad = (Outer(PxVec3(1.0f, 0.0f, 0.0f), Qinv.column2) + Outer(PxVec3(0.0f, 0.0f, 1.0f), Qinv.column0)) * halfVolume;
		grad = (PxMat33::outer(PxVec3(halfSqrtVolume, 0.0f, 0.0f), Qinv.column2) + PxMat33::outer(PxVec3(0.0f, 0.0f, halfSqrtVolume), Qinv.column0));
		//deltaLambda = -(G(0, 2) + invDtSq * multiplyStrainByCompliance(shSoftbody.mMaterial, 4, totalLambda) + gamma * E(0, 2))*scale;

		delta += grad * lambda.bottom.y;
		//totalLambda[4] = totalLambda[4] + deltaLambda4;

		// e01 (Voigt 5)
		//grad = (Outer(PxVec3(1.0f, 0.0f, 0.0f), Qinv.column1) + Outer(PxVec3(0.0f, 1.0f, 0.0f), Qinv.column0)) * halfVolume;
		grad = (PxMat33::outer(PxVec3(halfSqrtVolume, 0.0f, 0.0f), Qinv.column1) + PxMat33::outer(PxVec3(0.0f, halfSqrtVolume, 0.0f), Qinv.column0));
		//deltaLambda = -(G(0, 1) + invDtSq * multiplyStrainByCompliance(shSoftbody.mMaterial, 5, totalLambda) + gamma * E(0, 1))*scale;

		delta += grad * lambda.bottom.z;
		//totalLambda[5] = totalLambda[5] + deltaLambda5;
	}


	return delta;

}

#define SB_STIFFNESS_THRESHOLD 1.0e-18f

__device__ void tetrahedronsSolveGM(PxMat33& deltaLambdaXgradC, const PxVec3& u1, const PxVec3& u2, const PxVec3& u3,
                                    const float4& v0, const float4& v1, const float4& v2, const float4& v3,
                                    PxgSoftBody& shSoftbody, const float invDtSq, const float invDt,
                                    const PxU32 workIndex, const PxMat33& Qinv, const PxQuat& tetR, bool isTGS,
                                    PxsDeformableVolumeMaterialData& material)
{
	if (material.youngs < SB_STIFFNESS_THRESHOLD)
	{
		return;
	}

	if (material.materialModel == PxDeformableVolumeMaterialModel::eNEO_HOOKEAN)
	{
		const PxReal invE = 1.0f / material.youngs;
		const PxMat33 R(tetR); // Actually for the neo hookean model this rotation is not required but it also does not
		                       // harm if we use it. It gets computed anyway. 
						 
		assert(R.getDeterminant() > 0.f);
		const PxMat33 RTranspose = R.getTranspose();

		PxMat33 P = RTranspose * PxMat33(u1, u2, u3);

		PxMat33 delta = tetrahedronsSolveInnerNeoHookean(P, v0, v1, v2, v3, shSoftbody, invDtSq, invDt, workIndex, Qinv,
		                                                 isTGS, invE);

		PxVec3 delta0 = -(delta.column0 + delta.column1 + delta.column2) * v0.w;
		P.column0 += (delta.column0 * v1.w - delta0);
		P.column1 += (delta.column1 * v2.w - delta0);
		P.column2 += (delta.column2 * v3.w - delta0);

		if (material.poissons > SB_STIFFNESS_THRESHOLD)
		{
			const PxReal nu = material.poissons;
			const PxReal volumeLambda = (nu < 0.5f - SB_STIFFNESS_THRESHOLD) ? nu / ((1.0f - 2.0f * nu) * material.youngs) : 0.f;
			delta += volumeSolve(P, v0, v1, v2, v3, invDtSq, Qinv, isTGS, volumeLambda);
		}
		deltaLambdaXgradC = R * delta;
	}
	else // PxDeformableVolumeMaterialModel::eCO_ROTATIONAL
	{
		// Lame's parameters
		const PxPair<PxReal, PxReal> lames = lameParameters(material.youngs, material.poissons);

		// lame's second parameters
		const PxReal mu = lames.second;

		const PxReal det = Qinv.getDeterminant();
		const PxReal volume = 1.0f / (6.0f * det);
		PxMat33 F = PxMat33(u1, u2, u3) * Qinv;

		// linear corot without voigt notation
		const PxReal alphaTilde0 = invDtSq / (2.f * mu * volume);
				
		// damping: note that it follows the style of "tetrahedronsSolveInner".
		PxReal damping = material.elasticityDamping; // damping for ARAP term.
		PxReal scaledDamping = damping * toUniformReal(material.dampingScale); // this is maybe not really useful.

		float2 lambdas = isTGS ? make_float2(0.f) : shSoftbody.mSimTetraMultipliers[workIndex / 32].mbyz[workIndex & 31];

		ARAP_constraint(deltaLambdaXgradC, lambdas.x, v0, v1, v2, v3, tetR, F, invDt, Qinv, alphaTilde0, damping,
		                scaledDamping);

		PxVec3 dx0 = -(deltaLambdaXgradC.column0 + deltaLambdaXgradC.column1 + deltaLambdaXgradC.column2) * v0.w;
		PxVec3 dx1 = deltaLambdaXgradC.column0 * v1.w;
		PxVec3 dx2 = deltaLambdaXgradC.column1 * v2.w;
		PxVec3 dx3 = deltaLambdaXgradC.column2 * v3.w;

		PxVec3 x01 = u1 + (dx1 - dx0);
		PxVec3 x02 = u2 + (dx2 - dx0);
		PxVec3 x03 = u3 + (dx3 - dx0);

		F = PxMat33(x01, x02, x03) * Qinv;

		if(material.poissons > SB_STIFFNESS_THRESHOLD)
		{
			PxReal alphaTilde1 = 0.f;
			PxReal ratio = 0.f;

			if (material.poissons < 0.5f - SB_STIFFNESS_THRESHOLD)
			{
				// lame's first parameters
				const PxReal lambda = lames.first;
				alphaTilde1 = invDtSq / (lambda * volume);

				ratio = 2.f * mu / lambda; // alphaTilde1/alphaTilde0 for volume damping.
			}

			damping *= ratio; // damping for volume term.
			scaledDamping *= ratio;

			// linearized volume vs. nonlinear volume.
			// To use linaerized volume, use "traceSMinusI_constraint" instead of "volume_constraint"
			volume_constraint(deltaLambdaXgradC, lambdas.y, v0, v1, v2, v3, F, invDt, Qinv, alphaTilde1, damping,
				scaledDamping);
		}

		if (!isTGS)
			shSoftbody.mSimTetraMultipliers[workIndex / 32].mbyz[workIndex & 31] = lambdas;
	}
}

__device__ __forceinline__ PxVec3 limitLength(const PxVec3& v, const PxReal limit)
{
#if 0
	PxVec3 ret = v;
	PxReal sqLen = ret.magnitudeSquared();
	if (sqLen > (limit*limit))
	{
		ret *= limit/PxSqrt(sqLen);
	}
	return ret;
#else
	return v;
#endif
}

//Most significant bit is 0, the remaining 31 bits are 1
//The most siginificant bit in pull indices is used to store the tets flipped state
#define PULL_IND_MASK 0x7fffffff
//Opposite of PULL_IND_MASK: The most siginificant bit is 1 and the rest is 0
#define FLIPPED_MASK 0x80000000

PX_FORCE_INLINE __device__ void sb_gm_cp_solveTetrahedronsPartition(PxgSoftBody& shSoftbody, const PxU32 workIndex,
	const PxU32 nbTets, const PxReal invDt, bool isTGS, bool isFirstIteration, PxsDeformableVolumeMaterialData* materials)
{
	const PxReal invDtSq = invDt * invDt;

	float4 * PX_RESTRICT curPositions = shSoftbody.mSimPosition_InvMassCP;
	float4* PX_RESTRICT velocities = shSoftbody.mSimVelocity_invMassCP;
	const uint4* PX_RESTRICT writeIndices = shSoftbody.mSimRemapOutputCP;

	float4* PX_RESTRICT unorderedPositions = shSoftbody.mSimPosition_InvMass;
	float4* PX_RESTRICT unorderedVelocities = shSoftbody.mSimVelocity_InvMass;
	const uint4* PX_RESTRICT pullInds = shSoftbody.mSimPullIndices;

	const PxU16 globalMaterialIndex = shSoftbody.mOrderedMaterialIndices[workIndex];
	PxsDeformableVolumeMaterialData& material = materials[globalMaterialIndex];

	float4 p[4];
	float4 v[4];

	{
		float4* pPtr[4];
		float4* vPtr[4];
		
		uint4 pullInd = pullInds[workIndex];
		pullInd.x &= PULL_IND_MASK;

		PxU32* pullIndPtr = &pullInd.x;

#pragma unroll
		for (PxU32 i = 0; i < 4; ++i)
		{
			if (pullIndPtr[i] < PULL_IND_MASK)
			{
				pPtr[i] = &unorderedPositions[pullIndPtr[i]];
				vPtr[i] = &unorderedVelocities[pullIndPtr[i]];
			}
			else
			{
				pPtr[i] = &curPositions[workIndex + i * nbTets];
				vPtr[i] = &velocities[workIndex + i * nbTets];
			}
		}

		__syncwarp();

#pragma unroll
		for (PxU32 i = 0; i < 4; ++i)
			p[i] = *pPtr[i];

#pragma unroll
		for (PxU32 i = 0; i < 4; ++i)
			v[i] = *vPtr[i];
	}


	PxMat33 Qinv;
	loadPxMat33(shSoftbody.mSimTetraRestPoses[workIndex / 32], workIndex & 31, Qinv);

	const float4 q4 = shSoftbody.mSimTetraRotations[workIndex];
	PxQuat q(q4.x, q4.y, q4.z, q4.w);

	const PxVec3 u1 = PxLoad3(p[1] - p[0]);
	const PxVec3 u2 = PxLoad3(p[2] - p[0]);
	const PxVec3 u3 = PxLoad3(p[3] - p[0]);

	{
		if (q.magnitudeSquared() == 0.0f)
			q = shSoftbody.mInitialRotation;

		computeGMTetrahedronRotations(u1, u2, u3, Qinv, q, isFirstIteration);
	
		shSoftbody.mSimTetraRotations[workIndex] = make_float4(q.x, q.y, q.z, q.w);
	}

	PxMat33 deltaLambdaXgradC(PxZero);
	if(!(Qinv == PxMat33(PxZero)))
	{
		tetrahedronsSolveGM(deltaLambdaXgradC, u1, u2, u3, v[0], v[1], v[2], v[3], shSoftbody, invDtSq, invDt,
		                    workIndex, Qinv, q, isTGS, material);
	}

	__syncwarp();

	const uint4 writeIndex4 = writeIndices[workIndex];
	const PxU32* writeIndexPtr = &writeIndex4.x;

#pragma unroll
	for (PxU32 i = 0; i < 4; ++i)
	{
		PxVec3 deltaPos;
		if (i == 0)
			deltaPos = -(deltaLambdaXgradC.column0 + deltaLambdaXgradC.column1 + deltaLambdaXgradC.column2) * p[i].w;
		else
			deltaPos = deltaLambdaXgradC[i-1] * p[i].w;
		const PxU32 writeIndex = writeIndexPtr[i];
		curPositions[writeIndex] = make_float4(p[i].x + deltaPos.x, p[i].y + deltaPos.y, p[i].z + deltaPos.z, p[i].w);
		const PxVec3 deltaV = deltaPos * invDt;
		velocities[writeIndex] = make_float4(v[i].x + deltaV.x, v[i].y + deltaV.y, v[i].z + deltaV.z, v[i].w);
	}
}

PX_FORCE_INLINE __device__ void sb_gm_cp_solveSubTetrahedron(PxgSoftBody& shSoftbody, const PxU32 workIndex,
	float4& p0, float4& p1, float4& p2, float4& p3, float4& v0, float4& v1, float4& v2, float4& v3,
	const PxReal invDtSq, const PxReal invDt, const bool isTGS, const bool isFirstIteration, 
	PxsDeformableVolumeMaterialData& material)
{
	/*if (!isValidTet(PxLoad3(p0), PxLoad3(p1), PxLoad3(p2), PxLoad3(p3)))
		return;*/

	//Solve the hexahedron!
	PxMat33 Qinv;
	loadPxMat33(shSoftbody.mSimTetraRestPoses[workIndex / 32], workIndex & 31, Qinv);

	const float4 q4 = shSoftbody.mSimTetraRotations[workIndex];

	PxQuat q(q4.x, q4.y, q4.z, q4.w);

	const PxVec3 u1 = PxLoad3(p1 - p0);
	const PxVec3 u2 = PxLoad3(p2 - p0);
	const PxVec3 u3 = PxLoad3(p3 - p0);

	{
		if (q.magnitudeSquared() == 0.0f)
			q = shSoftbody.mInitialRotation;

		computeGMTetrahedronRotations(u1, u2, u3, Qinv, q, isFirstIteration);
		
		shSoftbody.mSimTetraRotations[workIndex] = make_float4(q.x, q.y, q.z, q.w);
	}

	if(Qinv == PxMat33(PxZero))
		return;

	PxMat33 deltaLambdaXgradC(PxZero);

	tetrahedronsSolveGM(deltaLambdaXgradC, u1, u2, u3, v0, v1, v2, v3, shSoftbody, invDtSq, invDt, workIndex, Qinv, q,
	                    isTGS, material);

	const PxVec3 sum = deltaLambdaXgradC.column0 + deltaLambdaXgradC.column1 + deltaLambdaXgradC.column2;
	const PxVec3 deltaPos0 = -sum * p0.w;

	assert(deltaPos0.isFinite());
	p0 += make_float4(deltaPos0.x, deltaPos0.y, deltaPos0.z, 0.f);
	v0 += make_float4(deltaPos0.x * invDt, deltaPos0.y * invDt, deltaPos0.z * invDt, 0.f);

	const PxVec3 deltaPos1 = deltaLambdaXgradC.column0 * p1.w;
	assert(deltaPos1.isFinite());
	p1 += make_float4(deltaPos1.x, deltaPos1.y, deltaPos1.z, 0.f);
	v1 += make_float4(deltaPos1.x * invDt, deltaPos1.y * invDt, deltaPos1.z * invDt, 0.f);

	const PxVec3 deltaPos2 = deltaLambdaXgradC.column1 * p2.w;
	assert(deltaPos2.isFinite());
	p2 += make_float4(deltaPos2.x, deltaPos2.y, deltaPos2.z, 0.f);
	v2 += make_float4(deltaPos2.x * invDt, deltaPos2.y * invDt, deltaPos2.z * invDt, 0.f);

	const PxVec3 deltaPos3 = deltaLambdaXgradC.column2 * p3.w;
	assert(deltaPos3.isFinite());
	p3 += make_float4(deltaPos3.x, deltaPos3.y, deltaPos3.z, 0.f);
	v3 += make_float4(deltaPos3.x * invDt, deltaPos3.y * invDt, deltaPos3.z * invDt, 0.f);
}

__constant__ __device__ PxU32 tets6PerVoxel[24] = { 0,1,6,2,  0,1,4,6,  1,4,6,5,  1,2,3,6,  1,3,7,6,  1,5,6,7 };

__constant__ __device__ PxU32 tets5PerVoxel[40] = {
					 0, 6, 3, 5,   0, 1, 5, 3,   6, 7, 3, 5,   4, 5, 6, 0,   2, 3, 0, 6,
					 1, 7, 4, 2,   1, 0, 2, 4,   7, 6, 4, 2,   5, 4, 1, 7,   3, 2, 7, 1 };

PX_FORCE_INLINE __device__ void sb_gm_cp_solveHexahedronsPartition(PxgSoftBody& shSoftbody, const PxU32 workIndex,
	const PxU32 nbTets, const PxReal invDt, bool isTGS, const bool isFirstIteration, PxsDeformableVolumeMaterialData* materials,
	PxU32 numTetsPerElement)
{
	const PxReal invDtSq = invDt * invDt;
	const PxU32 nbElements = nbTets / numTetsPerElement;

	float4 * PX_RESTRICT curPositions = shSoftbody.mSimPosition_InvMassCP;
	float4* PX_RESTRICT velocities = shSoftbody.mSimVelocity_invMassCP;
	const uint4* PX_RESTRICT writeIndices = shSoftbody.mSimRemapOutputCP;

	float4* PX_RESTRICT unorderedPositions = shSoftbody.mSimPosition_InvMass;
	float4* PX_RESTRICT unorderedVelocities = shSoftbody.mSimVelocity_InvMass;
	const uint4* PX_RESTRICT pullInds = shSoftbody.mSimPullIndices;

	bool flipped;	

	const PxU16 globalMaterialIndex = shSoftbody.mOrderedMaterialIndices[workIndex];
	
	PxsDeformableVolumeMaterialData& material = materials[globalMaterialIndex];

	float4 p[8];
	float4 v[8];

	{
		uint4 pullInd[2];
		pullInd[0] = pullInds[workIndex];
		pullInd[1] = pullInds[workIndex + nbElements];
		PxU32* pullIndPtr = &pullInd[0].x;

		flipped = (pullInd[0].x & FLIPPED_MASK) != 0;
		pullInd[0].x &= PULL_IND_MASK;

		float4* pPtr[8];
		float4* vPtr[8];

#pragma unroll
		for (PxU32 i = 0; i < 8; ++i)
		{
			if (pullIndPtr[i] < PULL_IND_MASK)
			{
				pPtr[i] = &unorderedPositions[pullIndPtr[i]];
				vPtr[i] = &unorderedVelocities[pullIndPtr[i]];
			}
			else
			{
				pPtr[i] = &curPositions[workIndex + i * nbElements];
				vPtr[i] = &velocities[workIndex + i * nbElements];
			}
		}

		__syncwarp();

#pragma unroll
		for (PxU32 i = 0; i < 8; ++i)
			p[i] = *pPtr[i];

#pragma unroll
		for (PxU32 i = 0; i < 8; ++i)
			v[i] = *vPtr[i];
	}

	if (1)
	{
		PxU32 offset = 0;
		if (numTetsPerElement == 5 && flipped)
			offset = 20;

		const PxU32* ptr = numTetsPerElement == 5 ? tets5PerVoxel : tets6PerVoxel;

		for (PxU32 i = 0; i < numTetsPerElement; ++i)
		{
			const PxU32 a = ptr[4 * i + 0 + offset];
			const PxU32 b = ptr[4 * i + 1 + offset];
			const PxU32 c = ptr[4 * i + 2 + offset];
			const PxU32 d = ptr[4 * i + 3 + offset];
			sb_gm_cp_solveSubTetrahedron(shSoftbody, workIndex + nbElements * i, p[a], p[b], p[c], p[d], v[a], v[b],
			                             v[c], v[d], invDtSq, invDt, isTGS, isFirstIteration, material);
		}
	}

	__syncwarp();

	uint4 writeIndex[2];
	writeIndex[0] = writeIndices[workIndex];
	writeIndex[1] = writeIndices[workIndex + nbElements];
	PxU32* writeIndexPtr = &writeIndex[0].x;

#pragma unroll
	for (PxU32 i = 0; i < 8; ++i)
	{
		const PxU32 w = writeIndexPtr[i];
		curPositions[w] = p[i];
		velocities[w] = v[i];
	}
}

// following the structure of "sb_gm_cp_solveHexahedronsPartition"
template <bool updatePosVel>
PX_FORCE_INLINE __device__ void sb_gm_cp_solveHexahedronsPartition_parallelGS(PxgSoftBody& shSoftbody, const PxU32 workIndex,
	const PxU32 nbTets, const PxReal invDt, bool isTGS, const bool isFirstIteration, PxsDeformableVolumeMaterialData* materials,
	PxU32 numTetsPerElement)
{
	const PxReal invDtSq = invDt * invDt;
	const PxU32 nbElements = nbTets / numTetsPerElement;

	float4* PX_RESTRICT unorderedPositions = shSoftbody.mSimPosition_InvMass;
	float4* PX_RESTRICT unorderedVelocities = shSoftbody.mSimVelocity_InvMass;
	const uint4* PX_RESTRICT pullInds = shSoftbody.mSimPullIndices;

	const PxU16 globalMaterialIndex = shSoftbody.mOrderedMaterialIndices[workIndex];

	PxsDeformableVolumeMaterialData& material = materials[globalMaterialIndex];

	float4 p[8];
	float4 v[8];

	uint4 pullInd[2];
	pullInd[0] = pullInds[workIndex];
	pullInd[1] = pullInds[workIndex + nbElements];
	PxU32* pullIndPtr = &pullInd[0].x;

	bool flipped = (pullInd[0].x & FLIPPED_MASK) != 0;
	pullInd[0].x &= PULL_IND_MASK;

	float4* PX_RESTRICT pPtr[8];
	float4* PX_RESTRICT vPtr[8];

#pragma unroll
	for(PxU32 i = 0; i < 8; ++i)
	{
		pPtr[i] = &unorderedPositions[pullIndPtr[i]];
		vPtr[i] = &unorderedVelocities[pullIndPtr[i]];
	}

	__syncwarp();

#pragma unroll
	for(PxU32 i = 0; i < 8; ++i)
		p[i] = *pPtr[i];

#pragma unroll
	for(PxU32 i = 0; i < 8; ++i)
		v[i] = *vPtr[i];

	PxU32 offset = 0;
	if(numTetsPerElement == 5 && flipped)
		offset = 20;

	const PxU32* ptr = numTetsPerElement == 5 ? tets5PerVoxel : tets6PerVoxel;

	for(PxU32 i = 0; i < numTetsPerElement; ++i)
	{
		const PxU32 a = ptr[4 * i + 0 + offset];
		const PxU32 b = ptr[4 * i + 1 + offset];
		const PxU32 c = ptr[4 * i + 2 + offset];
		const PxU32 d = ptr[4 * i + 3 + offset];
		sb_gm_cp_solveSubTetrahedron(shSoftbody, workIndex + nbElements * i, p[a], p[b], p[c], p[d], v[a], v[b], v[c],
		                             v[d], invDtSq, invDt, isTGS, isFirstIteration, material);
	}

	__syncwarp();

	if (updatePosVel) // GS-style update - update position and velocity in place
	{

#pragma unroll
		for (PxU32 i = 0; i < 8; ++i)
		{
			const PxU32 index = pullIndPtr[i];
			shSoftbody.mSimDeltaPos[index] += p[i] - unorderedPositions[index];
			unorderedPositions[index] = p[i];
			unorderedVelocities[index] = v[i];
		}
	}
	else // Jacobi-style update - position and velocity will be updated separately via mSimDelta.
	{

#pragma unroll
		for (PxU32 i = 0; i < 8; ++i)
		{
			const PxU32 index = pullIndPtr[i];
			const float4 delta4 = p[i] - unorderedPositions[index];

			const PxVec3 delta = PxLoad3(delta4);
			AtomicAdd(shSoftbody.mSimDelta[index], delta, 1.f); // 4th value (1.f) is unused.
		}
	}
}

//multiple blocks per soft body(NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA), each block has 256 threads
extern "C" __global__ 
__launch_bounds__(PxgSoftBodyKernelBlockDim::SB_SOLVETETRA_LOW, 1)
void sb_gm_cp_solveTetrahedronsPartitionLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies,
	const PxU32 numActiveSoftbodies,
	const PxReal invDt,
	const PxU32 partitionId,
	bool isTGS,
	bool isFirstIteration,
	PxsDeformableVolumeMaterialData* materials)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	blockCopy<float>(reinterpret_cast<float*>(&tSoftbody), reinterpret_cast<float*>(&softbody), sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	if (shSoftbody.mBodyFlags & PxDeformableBodyFlag::eKINEMATIC)
		return;

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbTets = shSoftbody.mNumTetsGM;
	const PxU32 numPartitions = shSoftbody.mNumPartitionsGM;

	if (nbTets > 0 && partitionId < numPartitions)
	{
		const PxU32 startInd = partitionId > 0 ? shSoftbody.mSimAccumulatedPartitionsCP[partitionId - 1] : 0;
		const PxU32 endInd = shSoftbody.mSimAccumulatedPartitionsCP[partitionId];		

		const PxU32 workIndex = startInd + groupThreadIdx;

		if(workIndex < endInd)
		{
			if(shSoftbody.mNumTetsPerElement == 1)
			{
				sb_gm_cp_solveTetrahedronsPartition(shSoftbody, workIndex, nbTets, invDt, isTGS, isFirstIteration,
				                                    materials);
			}
			else
			{

				sb_gm_cp_solveHexahedronsPartition_parallelGS<true>(shSoftbody, workIndex, nbTets, invDt, isTGS,
				                                                    isFirstIteration, materials,
				                                                    shSoftbody.mNumTetsPerElement);
			}
		}
	}
}

//multiple blocks per soft body(NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA), each block has 256 threads
extern "C" __global__
__launch_bounds__(PxgSoftBodyKernelBlockDim::SB_SOLVETETRA_LOW, 1)
void sb_gm_cp_solveTetrahedronsJacobiPartitionLaunch(
	PxgSoftBody * gSoftbodies,
	const PxU32 * activeSoftbodies,
	const PxU32 numActiveSoftbodies,
	const PxReal invDt,
	bool isTGS,
	PxsDeformableVolumeMaterialData* materials)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	blockCopy<float>(reinterpret_cast<float*>(&tSoftbody), reinterpret_cast<float*>(&softbody), sizeof(PxgSoftBody));

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	if (softbody.mNumPartitionsGM <= SB_PARTITION_LIMIT || softbody.mNumTetsPerElement == 1) // no extra Jacobi required.
		return;

	if (shSoftbody.mBodyFlags & PxDeformableBodyFlag::eKINEMATIC)
		return;

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbTets = shSoftbody.mNumTetsGM;

	const PxU32 startInd = shSoftbody.mSimAccumulatedPartitionsCP[SB_PARTITION_LIMIT - 1];
	const PxU32 endInd = shSoftbody.mSimAccumulatedPartitionsCP[SB_PARTITION_LIMIT];

	const PxU32 workIndex = startInd + groupThreadIdx;

	if (workIndex < endInd)
	{
		// Jacobi-style update
		// "isFirstIteration" is set to false, since this kernel always comes after GS steps.
		// If we want to try a purely Jacobi-style update, we need to pass in "isFirstIteration" as a parameter, or
		// always pass "true".
		sb_gm_cp_solveHexahedronsPartition_parallelGS<false>(shSoftbody, workIndex, nbTets, invDt, isTGS, false,
		                                                     materials, shSoftbody.mNumTetsPerElement);
	}
}

//multiple blocks per soft body(NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA), each block has 256 threads
extern "C" __global__ void sb_gm_cp_averageVertsLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies,
	const PxReal invDt)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	if (shSoftbody.mBodyFlags & PxDeformableBodyFlag::eKINEMATIC || shSoftbody.mNumTetsPerElement != 1)
		return;
	
	//float4* accumulatedVels = &vels[offset];
	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 numVerts = shSoftbody.mNumVertsGM;
	const PxU32 numTets = shSoftbody.mNumTetsGM;
	

	if (groupThreadIdx < numVerts &&  numTets > 0)
	{
		const PxU32 numTetsPerElement = shSoftbody.mNumTetsPerElement;
		const PxU32 numElements = numTets / numTetsPerElement;
		const PxU32 numVertsPerElement = numTetsPerElement == 1 ? 4 : 8;

		float4* gridModelPoses = shSoftbody.mSimPosition_InvMass;
		float4* gridModelVels = shSoftbody.mSimVelocity_InvMass;
		float4* deltaPos = shSoftbody.mSimDeltaPos;

		const float4* PX_RESTRICT curPositions = shSoftbody.mSimPosition_InvMassCP;

		const PxU32* PX_RESTRICT accumulatedCopies = shSoftbody.mSimAccumulatedCopiesCP;
		
		const PxU32 offset = numElements * numVertsPerElement;

		const float4* accumulatedPoses = &curPositions[offset];

		const float4 pos_invMass = gridModelPoses[groupThreadIdx];

		if (pos_invMass.w > 0.f)
		{
			const PxU32 startInd = groupThreadIdx == 0 ? 0 : accumulatedCopies[groupThreadIdx - 1];
			const PxU32 endInd = accumulatedCopies[groupThreadIdx];

			float4 diff = accumulatedPoses[startInd] - pos_invMass;

			for (PxU32 j = startInd + 1; j < endInd; ++j)
			{
				float4 newDiff = accumulatedPoses[j] - pos_invMass;
				diff += newDiff;
			}

			const PxU32 numCopies = endInd - startInd;
			PxReal scale = 1.f / numCopies;

			diff.w = 0.f;

			float4 averageDelta = diff * scale;
			float4 averagePos = pos_invMass + averageDelta;
			const float4 averageVel = gridModelVels[groupThreadIdx] + averageDelta * invDt;

			gridModelPoses[groupThreadIdx] = averagePos;
			gridModelVels[groupThreadIdx] = make_float4(averageVel.x, averageVel.y, averageVel.z, pos_invMass.w);
			deltaPos[groupThreadIdx] += averageDelta;
		}
	}
}


//Multiple blocks deal with one soft body. This uses grid model verts to update tet model verts.
extern "C" __global__ void sb_gm_updateTetModelVertsLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies
)
{

	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];

	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 numVerts = shSoftbody.mNumVerts;

	if (groupThreadIdx < numVerts)
	{
		const float4* barycentricCoordinates = shSoftbody.mVertsBarycentricInGridModel;
		const PxU32* remapTable = shSoftbody.mVertsRemapInGridModel;
		const uint4* tetsGM = shSoftbody.mSimTetIndices;
		const float4* positionGM = shSoftbody.mSimPosition_InvMass;

		float4* curPositions = shSoftbody.mPosition_InvMass;
		
		const PxU32 tetrahedronIdx = remapTable[groupThreadIdx];
		const uint4 tetInd = tetsGM[tetrahedronIdx];
		const float4 p0 = positionGM[tetInd.x];
		const float4 p1 = positionGM[tetInd.y];
		const float4 p2 = positionGM[tetInd.z];
		const float4 p3 = positionGM[tetInd.w];

		const float4 barycentric = barycentricCoordinates[groupThreadIdx];

		float4 newPos = p0 * barycentric.x + p1 * barycentric.y + p2 * barycentric.z + p3 * barycentric.w;
		//newPos.w = pos.w;
		//curPosition will get updated if there are collision/interaction with other shape type
		curPositions[groupThreadIdx] = newPos;
		
	}
}

////Multiple blocks deal with one soft body. Compute deltaPos for the grid model verts from tet model
//extern "C" __global__ void sb_gm_updateGMVertsDeltaLaunch(
//	PxgSoftBody* gSoftbodies,
//	const PxU32* activeSoftbodies,
//	const PxU32 oldPosInd
//)
//{
//	__shared__ PxgSoftBody shSoftbody;
//
//	const PxU32 index = blockIdx.x / NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
//
//	const PxU32 softbodyId = activeSoftbodies[index];
//
//	PxgSoftBody& softbody = gSoftbodies[softbodyId];
//
//	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
//	uint2* dSoftbody = reinterpret_cast<uint2*>(&shSoftbody);
//
//	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));
//
//	__syncthreads();
//
//	const float4* barycentricCoordinates = shSoftbody.mVertsBarycentricInGridModel;
//	const PxU32* remapTable = shSoftbody.mVertsRemapInGridModel;
//	const uint4* tetsGM = shSoftbody.mGridModelTetIndices;
//	float4* deltaPosGM = shSoftbody.mGridModelDeltaPos;
//	float4* deltaPosGMin = shSoftbody.mGridModelDeltaPosMin;
//	float4* deltaPosGMax = shSoftbody.mGridModelDeltaPosMax;
//
//	float4* positionGM = shSoftbody.mGridModelPosition_InvMass[1 - oldPosInd];
//	
//	const float4* curPositions = shSoftbody.mPosition_InvMass;
//
//	//this record the previous curPosition
//	float4* tempPostions = shSoftbody.mGridModelTempPosition;
//
//	const PxU32 groupBlockIdx = blockIdx.x % NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
//	const PxU32 groupThreadIdx = threadIdx.x + groupBlockIdx * blockDim.x;
//
//	const PxU32 requiredThreadsInGroup = shSoftbody.mNumVerts * 4;
//
//	const PxU32 totalThreadsInGroup = blockDim.x * NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
//
//	//each 4 threads deal with one vert
//	for (PxU32 i = groupThreadIdx; i < requiredThreadsInGroup; i += totalThreadsInGroup)
//	{
//		const PxU32 workIndex = i / 4;
//		const float4 curPos = curPositions[workIndex];
//		const PxU32 tetrahedronIdx = remapTable[workIndex];
//		const uint4 tetInd = tetsGM[tetrahedronIdx];
//		const int* tetIndArray = reinterpret_cast<const int*>(&tetInd);
//
//		const PxU32 index = i % 4; // 0, 1, 2, 3
//		const PxU32 vertInd = tetIndArray[index];
//	
//		if (curPos.w > 0.f)
//		{
//			const float4 tPos = tempPostions[workIndex];
//			const float4 tDeltaPos = curPos - tPos;
//			const PxVec3 deltaPos(tDeltaPos.x, tDeltaPos.y, tDeltaPos.z);
//
//			const float4 barycentric = barycentricCoordinates[workIndex];
//			const float* baryArray = reinterpret_cast<const float*>(&barycentric);
//
//			const PxVec3 delta = deltaPos * baryArray[index];// * relaxation;
//
//			if (delta.magnitudeSquared() > 0.f)
//			{
//				//printf("Delta = (%f, %f, %f), p.w = %f\n", delta.x, delta.y, delta.z, p.w);
//				float4& dpGM = deltaPosGM[vertInd];
//				AtomicAdd(dpGM, delta, 1.f);
//				AtomicMinf(&deltaPosGMin[vertInd].x, delta.x);
//				AtomicMinf(&deltaPosGMin[vertInd].y, delta.y);
//				AtomicMinf(&deltaPosGMin[vertInd].z, delta.z);
//				AtomicMaxf(&deltaPosGMax[vertInd].x, delta.x);
//				AtomicMaxf(&deltaPosGMax[vertInd].y, delta.y);
//				AtomicMaxf(&deltaPosGMax[vertInd].z, delta.z);
//			}
//			
//		}
//		else
//		{
//			positionGM[vertInd].w = 0.f;
//		}
//	}
//
//}

////Multiple blocks deal with one soft body. Update grid model verts from the deltaPos.
//extern "C" __global__ __launch_bounds__(1024, 1) void sb_gm_updateGMVertsLaunch(
//	PxgSoftBody* gSoftbodies,
//	const PxU32* activeSoftbodies,
//	const PxReal invDt,
//	const PxU32 oldPosInd
//)
//{
//	__shared__ PxgSoftBody shSoftbody;
//
//	const PxU32 index = blockIdx.x / NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
//
//	const PxU32 softbodyId = activeSoftbodies[index];
//
//	PxgSoftBody& softbody = gSoftbodies[softbodyId];
//
//	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
//	uint2* dSoftbody = reinterpret_cast<uint2*>(&shSoftbody);
//
//	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));
//
//	__syncthreads();
//
//	float4* deltaPosGM = shSoftbody.mGridModelDeltaPos;
//	float4* deltaPosGMMin = shSoftbody.mGridModelDeltaPosMin;
//	float4* deltaPosGMMax = shSoftbody.mGridModelDeltaPosMax;
//
//	float4* positionGM = shSoftbody.mGridModelPosition_InvMass[1 - oldPosInd];
//	float4* velGM = shSoftbody.mGridModelVelocity_InvMass;
//
//	const PxU32 groupBlockIdx = blockIdx.x % NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
//	const PxU32 groupThreadIdx = threadIdx.x + groupBlockIdx * blockDim.x;
//
//	const PxU32 requiredThreadsInGroup = shSoftbody.mNumVertsGM;
//
//	const PxU32 totalThreadsInGroup = blockDim.x * NUM_BLOCK_PER_SOFTBODY_SOLVE_TETRA;
//
//	for (PxU32 i = groupThreadIdx; i < requiredThreadsInGroup; i += totalThreadsInGroup)
//	{
//		const float4 p = positionGM[i];
//	
//		if (p.w > 0.f)
//		{
//			const float4 v = velGM[i];
//			const float4 mn = deltaPosGMMin[i];
//			const float4 mx = deltaPosGMMax[i];
//			float4 deltaPos = deltaPosGM[i];
//			deltaPos = deltaPos;// / PxMax(1.f, 0.5f*deltaPos.w);
//
//			//printf("%i: deltaPos = (%f, %f, %f), mn = (%f, %f, %f), mx = (%f, %f, %f)\n", i, deltaPos.x, deltaPos.y, deltaPos.z, mn.x, mn.y, mn.z, mx.x, mx.y, mx.z);
//
//			if (deltaPos.w > 0.f)
//			{
//				PxVec3 diff(mx.x - mn.x, mx.y-mn.y, mx.z-mn.z);
//				const PxReal scale = 2.f;
//
//				deltaPos.x = PxClamp(deltaPos.x, mn.x - scale*diff.x, mx.x + scale*diff.x);
//				deltaPos.y = PxClamp(deltaPos.y, mn.y - scale*diff.y, mx.y + scale*diff.y);
//				deltaPos.z = PxClamp(deltaPos.z, mn.z - scale*diff.z, mx.z + scale*diff.z);
//			}
//
//			deltaPos.w = 0.f;
//			positionGM[i] = p + deltaPos;
//			velGM[i] = v + deltaPos * invDt;
//		}
//
//		deltaPosGM[i] = make_float4(0.f, 0.f, 0.f, 0.f);
//		deltaPosGMMin[i] = make_float4(PX_MAX_F32, PX_MAX_F32, PX_MAX_F32, 0.f);
//		deltaPosGMMax[i] = make_float4(-PX_MAX_F32, -PX_MAX_F32, -PX_MAX_F32, 0.f);
//	}
//}

//Multiple blocks deal with one soft body. This uses grid model verts to duplicate verts for combined partitions.
extern "C" __global__ void sb_gm_zeroTetMultipliers(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies
)
{

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 numTets = softbody.mNumTetsGM;
	
	if(groupThreadIdx < numTets)
	{
		storeSpatialVector(softbody.mSimTetraMultipliers[groupThreadIdx /32], Cm::UnAlignedSpatialVector::Zero(), groupThreadIdx &31);
	}

}


extern "C" __global__ void sb_rigidAttachmentPrepareLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMRigidAttachment*						rigidAttachments,
	PxU32*										activeRigidAttachments,
	PxNodeIndex*								rigidAttachmentIds,
	PxU32										numRigidAttachments,
	PxgFEMRigidAttachmentConstraint*			attachmentConstraints,
	PxgPrePrepDesc*								preDesc,
	PxgConstraintPrepareDesc*					prepareDesc,
	PxgSolverSharedDescBase*					sharedDesc,
	float4*										rigidDeltaVel
)
{
	PxAlignedTransform* bodyFrames = prepareDesc->body2WorldPool;

	PxU32* solverBodyIndices = preDesc->solverBodyIndices;
	PxgSolverBodyData* solverBodyData = prepareDesc->solverBodyDataPool;
	PxgSolverTxIData* solverDataTxIPool = prepareDesc->solverBodyTxIDataPool;

	PxgBodySim* bodySims = sharedDesc->mBodySimBufferDeviceData;
	const PxU32 nbBlocksRequired = (numRigidAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numRigidAttachments)
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		PxU32 attachmentIndex = activeRigidAttachments[workIndex];

		PxgFEMRigidAttachment& attachment = rigidAttachments[attachmentIndex];
		PxgFEMRigidAttachmentConstraint& constraint = attachmentConstraints[index];

		PxU32 elemId = attachment.index1;
		PxU32 softBodyId = PxGetSoftBodyId(elemId);
		PxU32 elemIdx = PxGetSoftBodyElementIndex(elemId);
		bool elemIsVertex = PxGetIsVertexType(attachment.baryOrType1);

		PxgSoftBody& softbody = softbodies[softBodyId];

		const float4* pos_invMassGM = softbody.mSimPosition_InvMass;
		
		float4 gridPos;
		float invMass1;
		if (elemIsVertex)
		{
			gridPos = pos_invMassGM[elemIdx];
			invMass1 = gridPos.w;
		}
		else
		{
			const float4 barycentric = attachment.baryOrType1;
			const uint4* tetsGM = softbody.mSimTetIndices;
			const uint4 tetInd = tetsGM[elemIdx];
			const float4 pos_iMass0 = pos_invMassGM[tetInd.x];
			const float4 pos_iMass1 = pos_invMassGM[tetInd.y];
			const float4 pos_iMass2 = pos_invMassGM[tetInd.z];
			const float4 pos_iMass3 = pos_invMassGM[tetInd.w];
			gridPos = pos_iMass0 * barycentric.x + pos_iMass1 * barycentric.y + pos_iMass2 * barycentric.z + pos_iMass3 * barycentric.w;
			invMass1 = getSoftBodyInvMass(gridPos.w, barycentric);
		}
		
		const PxVec3 point(gridPos.x, gridPos.y, gridPos.z);

		float4 ra4 = attachment.localPose0;

		//nodeIndex
		PxNodeIndex rigidId = reinterpret_cast<PxNodeIndex&>(attachment.index0);
		PxU32 idx = 0;
		if (!rigidId.isStaticBody())
		{
			idx = solverBodyIndices[rigidId.index()];
		}
		
		rigidAttachmentIds[workIndex] = rigidId;

		const PxVec3 normal0(1.f, 0.f, 0.f);
		const PxVec3 normal1(0.f, 1.f, 0.f);
		const PxVec3 normal2(0.f, 0.f, 1.f);

		const float4 low_high_limits = attachment.coneLimitParams.low_high_limits;
		const float4 axis_angle = attachment.coneLimitParams.axis_angle;
		const PxVec3 axis(axis_angle.x, axis_angle.y, axis_angle.z);

		if (rigidId.isArticulation())
		{
	
			PxU32 nodeIndexA = rigidId.index();
			PxU32 artiId = bodySims[nodeIndexA].articulationRemapId;

			PxgArticulation& articulation = sharedDesc->articulations[artiId];

			const PxU32 linkID = rigidId.articulationLinkId();
			const PxTransform body2World = articulation.linkBody2Worlds[linkID];

			const PxVec3 bodyFrame0p(body2World.p.x, body2World.p.y, body2World.p.z);

			PxVec3 ra(ra4.x, ra4.y, ra4.z);
			ra = body2World.rotate(ra);
			PxVec3 error = ra + bodyFrame0p - point;

			const PxVec3 worldAxis = (body2World.rotate(axis)).getNormalized();

			const PxVec3 raXn0 = ra.cross(normal0);
			const PxVec3 raXn1 = ra.cross(normal1);
			const PxVec3 raXn2 = ra.cross(normal2);

			PxSpatialMatrix& spatialResponse = articulation.spatialResponseMatrixW[linkID];
			const Cm::UnAlignedSpatialVector deltaV0 = spatialResponse * Cm::UnAlignedSpatialVector(normal0, raXn0);
			const Cm::UnAlignedSpatialVector deltaV1 = spatialResponse * Cm::UnAlignedSpatialVector(normal1, raXn1);
			const Cm::UnAlignedSpatialVector deltaV2 = spatialResponse * Cm::UnAlignedSpatialVector(normal2, raXn2);

			const PxReal resp0 = deltaV0.top.dot(raXn0) + deltaV0.bottom.dot(normal0) + invMass1;
			const PxReal resp1 = deltaV1.top.dot(raXn1) + deltaV1.bottom.dot(normal1) + invMass1;
			const PxReal resp2 = deltaV2.top.dot(raXn2) + deltaV2.bottom.dot(normal2) + invMass1;

			const float velMultiplier0 = (resp0 > 0.f) ? (1.f / resp0) : 0.f;
			const float velMultiplier1 = (resp1 > 0.f) ? (1.f / resp1) : 0.f;
			const float velMultiplier2 = (resp2 > 0.f) ? (1.f / resp2) : 0.f;

			PxReal biasedErr0 = (error.dot(normal0));
			PxReal biasedErr1 = (error.dot(normal1));
			PxReal biasedErr2 = (error.dot(normal2));

			constraint.raXn0_biasW[offset] = make_float4(raXn0.x, raXn0.y, raXn0.z, biasedErr0);
			constraint.raXn1_biasW[offset] = make_float4(raXn1.x, raXn1.y, raXn1.z, biasedErr1);
			constraint.raXn2_biasW[offset] = make_float4(raXn2.x, raXn2.y, raXn2.z, biasedErr2);
			//articulation don't use invMass0. We set it to 1.0 here so that the impulse scaling for the linear impulse
			//to convert it to a velocity change remains an impulse if it is dealing with an articulation.
			constraint.velMultiplierXYZ_invMassW[offset] = make_float4(velMultiplier0, velMultiplier1, velMultiplier2, 1.f); 
			constraint.elemId[offset] = elemId;
			constraint.rigidId[offset] = rigidId.getInd();
			constraint.baryOrType[offset] = attachment.baryOrType1;
			constraint.low_high_limits[offset] = low_high_limits;
			constraint.axis_angle[offset] = make_float4(worldAxis.x, worldAxis.y, worldAxis.z, axis_angle.w);
		}
		else
		{
			
			//PxMat33 invSqrtInertia0 = solverDataTxIPool[idx].sqrtInvInertia;
			const float4 linVel_invMass0 = solverBodyData[idx].initialLinVelXYZ_invMassW;
			const PxReal invMass0 = linVel_invMass0.w;

			PxMat33 invSqrtInertia0;
			PxReal inertiaScale = 1.f;
			if (invMass0 == 0.f && !rigidId.isStaticBody())
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

			PxVec3 ra(ra4.x, ra4.y, ra4.z);
			ra = bodyFrame0.rotate(ra);
			PxVec3 error = ra + bodyFrame0p - point;

			const PxVec3 worldAxis = (bodyFrame0.rotate(axis)).getNormalized();

			const PxVec3 raXn0 = ra.cross(normal0);
			const PxVec3 raXn1 = ra.cross(normal1);
			const PxVec3 raXn2 = ra.cross(normal2);

			const PxVec3 raXnSqrtInertia0 = invSqrtInertia0 * raXn0;
			const PxVec3 raXnSqrtInertia1 = invSqrtInertia0 * raXn1;
			const PxVec3 raXnSqrtInertia2 = invSqrtInertia0 * raXn2;
			const float resp0 = (raXnSqrtInertia0.dot(raXnSqrtInertia0))*inertiaScale + invMass0 + invMass1;
			const float resp1 = (raXnSqrtInertia1.dot(raXnSqrtInertia1))*inertiaScale + invMass0 + invMass1;
			const float resp2 = (raXnSqrtInertia2.dot(raXnSqrtInertia2))*inertiaScale + invMass0 + invMass1;

			const float velMultiplier0 = (resp0 > 0.f) ? (1.f / resp0) : 0.f;
			const float velMultiplier1 = (resp1 > 0.f) ? (1.f / resp1) : 0.f;
			const float velMultiplier2 = (resp2 > 0.f) ? (1.f / resp2) : 0.f;

			//PxReal biasedErr0 = biasCoefficient * (error.dot(normal0));
			//PxReal biasedErr1 = biasCoefficient * (error.dot(normal1));
			//PxReal biasedErr2 = biasCoefficient * (error.dot(normal2));


			PxReal biasedErr0 = (error.dot(normal0));
			PxReal biasedErr1 = (error.dot(normal1));
			PxReal biasedErr2 = (error.dot(normal2));

			constraint.raXn0_biasW[offset] = make_float4(raXnSqrtInertia0.x, raXnSqrtInertia0.y, raXnSqrtInertia0.z, biasedErr0);
			constraint.raXn1_biasW[offset] = make_float4(raXnSqrtInertia1.x, raXnSqrtInertia1.y, raXnSqrtInertia1.z, biasedErr1);
			constraint.raXn2_biasW[offset] = make_float4(raXnSqrtInertia2.x, raXnSqrtInertia2.y, raXnSqrtInertia2.z, biasedErr2);
			constraint.velMultiplierXYZ_invMassW[offset] = make_float4(velMultiplier0, velMultiplier1, velMultiplier2, invMass0);
			constraint.elemId[offset] = elemId;
			constraint.rigidId[offset] = rigidId.getInd();
			constraint.baryOrType[offset] = attachment.baryOrType1;
			constraint.low_high_limits[offset] = low_high_limits;
			constraint.axis_angle[offset] = make_float4(worldAxis.x, worldAxis.y, worldAxis.z, axis_angle.w);

			if (rigidDeltaVel)
			{
				rigidDeltaVel[workIndex] = make_float4(0.f);
				rigidDeltaVel[workIndex + numRigidAttachments] = make_float4(0.f);
			}
		}
	}
}


extern "C" __global__ void femAttachmentPrepareLaunch(
	PxgFEMFEMAttachment*						femAttachments,
	PxU32*										activeFemAttachments,
	/*PxU64*										femAttachmentIds,*/
	PxU32										numFemAttachments,
	PxgFEMFEMAttachmentConstraint*				attachmentConstraints
)
{


	const PxU32 nbBlocksRequired = (numFemAttachments + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numFemAttachments)
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxU32 attachmentIndex = activeFemAttachments[workIndex];

		PxgFEMFEMAttachment& attachment = femAttachments[attachmentIndex];		
		PxgFEMFEMAttachmentConstraint& constraint = attachmentConstraints[index];

		const PxU64 compressedElemId = attachment.index0;
		//femAttachmentIds[workIndex] = compressedElemId;

		const float4 low_high_angle = attachment.coneLimitParams.low_high_angle;
		const float4 barycentric = attachment.coneLimitParams.barycentric;

		constraint.barycentric0[offset] = attachment.barycentricCoordinates0;
		constraint.barycentric1[offset] = attachment.barycentricCoordinates1;
		constraint.elemId0[offset] = compressedElemId;
		constraint.elemId1[offset] = attachment.index1;
		constraint.constraintOffset[offset] = attachment.constraintOffset;
		constraint.low_high_angles[offset] = make_float4(low_high_angle.x, low_high_angle.y, low_high_angle.z, 0.f);
		constraint.attachmentBarycentric[offset] = barycentric;
		
	}
}

//solve rigid attachment
extern "C" __global__ void sb_solveRigidSoftAttachmentLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMRigidAttachmentConstraint*			attachments,
	const PxU32									numAttachments,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	const PxReal								dt,
	float4*										rigidDeltaVel				//output
)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= numAttachments)
		{
			return;
		}

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];

		// Soft body info
		const PxU32 elemId = constraint.elemId[offset];
		const PxU32 softBodyId = PxGetSoftBodyId(elemId);
		const PxU32 elemIdx = PxGetSoftBodyElementIndex(elemId);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgSoftBody& softbody = softbodies[softBodyId];
		const float4* velGM = softbody.mSimVelocity_InvMass;

		float4 softbodyVel;
		uint4 tetInd;
		float4 tetInvMass;
		float4 barycentric;

		if(elemIsVertex)
		{
			softbodyVel = velGM[elemIdx];
		}
		else
		{
			barycentric = constraint.baryOrType[offset];
			const uint4* tetsGM = softbody.mSimTetIndices;
			tetInd = tetsGM[elemIdx];
			const float4 v0 = velGM[tetInd.x];
			const float4 v1 = velGM[tetInd.y];
			const float4 v2 = velGM[tetInd.z];
			const float4 v3 = velGM[tetInd.w];
			softbodyVel = v0 * barycentric.x + v1 * barycentric.y + v2 * barycentric.z + v3 * barycentric.w;

			tetInvMass.x = v0.w;
			tetInvMass.y = v1.w;
			tetInvMass.z = v2.w;
			tetInvMass.w = v3.w;
		}

		PxVec3 linVel1(softbodyVel.x, softbodyVel.y, softbodyVel.z);

		// Rigid body info
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);
		if(rigidId.isStaticBody() && softbodyVel.w == 0.0f)
		{
			continue;
		}

		PxgVelocityPackPGS vel0;
		velocityReader.readVelocitiesPGS(rigidId, vel0);

		// Compute impulses
		PxVec3 deltaLinVel, deltaAngVel;
		const PxVec3 deltaImpulse =
			calculateAttachmentDeltaImpulsePGS(offset, constraint, vel0, linVel1, 1.f / dt, 0.5f, deltaLinVel, deltaAngVel);

		// Update rigid body
		if(!rigidId.isStaticBody())
		{
			const float4 velMultiplierXYZ_invMassW = constraint.velMultiplierXYZ_invMassW[offset];
			if(velMultiplierXYZ_invMassW.w == 0.0f)
			{
				rigidDeltaVel[workIndex] = make_float4(0.0f);
				rigidDeltaVel[workIndex + numAttachments] = make_float4(0.0f);
			}
			else
			{
				rigidDeltaVel[workIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, 1.0f);
				rigidDeltaVel[workIndex + numAttachments] = make_float4(deltaAngVel.x, deltaAngVel.y, deltaAngVel.z, 0.0f);
			}
		}

		// Update soft body
		if(!deltaImpulse.isZero())
		{
			const PxVec3 deltaPos = -deltaImpulse * dt;

			if(elemIsVertex)
			{
				AtomicAdd(softbody.mSimDelta[elemIdx], deltaPos * softbodyVel.w, 1.f);
			}
			else
			{
				AtomicAdd(softbody.mSimDelta[tetInd.x], deltaPos * tetInvMass.x * barycentric.x, 1.f);
				AtomicAdd(softbody.mSimDelta[tetInd.y], deltaPos * tetInvMass.y * barycentric.y, 1.f);
				AtomicAdd(softbody.mSimDelta[tetInd.z], deltaPos * tetInvMass.z * barycentric.z, 1.f);
				AtomicAdd(softbody.mSimDelta[tetInd.w], deltaPos * tetInvMass.w * barycentric.w, 1.f);
			}
		}
	}
}

//////TGS//////////////


//solve rigid attachment
extern "C" __global__ void sb_solveRigidSoftAttachmentLaunchTGS(
	PxgSoftBody*								softbodies,
	PxgFEMRigidAttachmentConstraint*			attachments,
	const PxU32									numAttachments,
	PxgPrePrepDesc*								prePrepDesc,
	PxgSolverCoreDesc*							solverCoreDesc,
	PxgArticulationCoreDesc*					artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>*	sharedDesc,
	const PxReal								dt,
	const PxReal								biasCoefficient,
	float4*										rigidDeltaVel,				//output
	bool										isVelocityIteration
)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= numAttachments)
		{
			return;
		}

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];

		// Soft body info
		const PxU32 softBodyId = PxGetSoftBodyId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetSoftBodyElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);
		
		PxgSoftBody& softbody = softbodies[softBodyId];
		const float4* deltaGM = softbody.mSimDeltaPos;

		float4 softbodyDelta;
		uint4 tetInd;
		float4 tetInvMass;
		float4 barycentric;

		if(elemIsVertex)
		{
			softbodyDelta = deltaGM[elemIdx];
		}
		else
		{
			barycentric = constraint.baryOrType[offset];
			const uint4* tetsGM = softbody.mSimTetIndices;
			tetInd = tetsGM[elemIdx];
			const float4 d0 = deltaGM[tetInd.x];
			const float4 d1 = deltaGM[tetInd.y];
			const float4 d2 = deltaGM[tetInd.z];
			const float4 d3 = deltaGM[tetInd.w];
			softbodyDelta = d0 * barycentric.x + d1 * barycentric.y + d2 * barycentric.z + d3 * barycentric.w;

			tetInvMass.x = d0.w;
			tetInvMass.y = d1.w;
			tetInvMass.z = d2.w;
			tetInvMass.w = d3.w;
		}
		PxVec3 linDelta1(softbodyDelta.x, softbodyDelta.y, softbodyDelta.z);

		// Rigid body info
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);
		if(rigidId.isStaticBody() && softbodyDelta.w == 0.0f)
		{
			continue;
		}

		PxgVelocityPackTGS vel0;
		velocityReader.readVelocitiesTGS(rigidId, vel0);

		// Compute impulses
		PxVec3 deltaLinVel, deltaAngVel;
		PxVec3 deltaImpulse = calculateAttachmentDeltaImpulseTGS(offset, constraint, vel0, linDelta1, dt, biasCoefficient,
																 isVelocityIteration, deltaLinVel, deltaAngVel);

		// Update rigid body
		if(!rigidId.isStaticBody())
		{
			const float4 velMultiplierXYZ_invMassW = constraint.velMultiplierXYZ_invMassW[offset];
			if(velMultiplierXYZ_invMassW.w == 0.0f)
			{
				rigidDeltaVel[workIndex] = make_float4(0.0f);
				rigidDeltaVel[workIndex + numAttachments] = make_float4(0.0f);
			}
			else
			{
				rigidDeltaVel[workIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, 1.0f);
				rigidDeltaVel[workIndex + numAttachments] = make_float4(deltaAngVel.x, deltaAngVel.y, deltaAngVel.z, 0.0f);
			}
		}

		// Update soft body
		if(!deltaImpulse.isZero())
		{
			const PxVec3 deltaPos = -deltaImpulse * dt;

			if(elemIsVertex)
			{
				AtomicAdd(softbody.mSimDelta[elemIdx], deltaPos * softbodyDelta.w, 1.0f);
			}
			else
			{
				AtomicAdd(softbody.mSimDelta[tetInd.x], deltaPos * barycentric.x * tetInvMass.x, 1.0f);
				AtomicAdd(softbody.mSimDelta[tetInd.y], deltaPos * barycentric.y * tetInvMass.y, 1.0f);
				AtomicAdd(softbody.mSimDelta[tetInd.z], deltaPos * barycentric.z * tetInvMass.z, 1.0f);
				AtomicAdd(softbody.mSimDelta[tetInd.w], deltaPos * barycentric.w * tetInvMass.w, 1.0f);
			}
		}
	}
}

//solve softbody attachment, output soft body delta
extern "C" __global__ void sb_solveOutputSoftBodyAttachmentDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMFEMAttachmentConstraint*				attachments,
	const PxU32									numAttachments
)
{
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= numAttachments)
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMFEMAttachmentConstraint& constraint = attachments[index];

		const PxU32 elemId1 = constraint.elemId1[offset];
		const PxU32 softBodyId1 = PxGetSoftBodyId(elemId1);
		const PxU32 tetrahedronIdx1 = PxGetSoftBodyElementIndex(elemId1);
		PxgSoftBody& softbody1 = softbodies[softBodyId1];
		const float4* simPos1 = softbody1.mSimPosition_InvMass;
		const uint4* simTets1 = softbody1.mSimTetIndices;
		const uint4 tetInd1 = simTets1[tetrahedronIdx1];
		const float4 bary1 = constraint.barycentric1[offset];

		float4 p10 = simPos1[tetInd1.x];
		float4 p11 = simPos1[tetInd1.y];
		float4 p12 = simPos1[tetInd1.z];
		float4 p13 = simPos1[tetInd1.w];
		float4 pos1 = p10*bary1.x + p11*bary1.y + p12*bary1.z + p13*bary1.w;

		const PxU32 elemInd0 = constraint.elemId0[offset];
		const PxU32 softBodyId0 = PxGetSoftBodyId(elemInd0);
		const PxU32 tetrahedronIdx0 = PxGetSoftBodyElementIndex(elemInd0);
		const float4 bary0 = constraint.barycentric0[offset];
		PxgSoftBody& softbody0 = softbodies[softBodyId0];
		const float4* simPos0 = softbody0.mSimPosition_InvMass;
		const uint4* simTets0 = softbody0.mSimTetIndices;
		const uint4 tetInd0 = simTets0[tetrahedronIdx0];

		float4 p00 = simPos0[tetInd0.x];
		float4 p01 = simPos0[tetInd0.y];
		float4 p02 = simPos0[tetInd0.z];
		float4 p03 = simPos0[tetInd0.w];
		float4 pos0 = p00*bary0.x + p01*bary0.y + p02*bary0.z + p03*bary0.w;
		
		//output for the soft body 0 and 1 - just do atomics on this for now...
		float4* PX_RESTRICT accumDeltaV1 = softbody1.mSimDelta;
		float4* PX_RESTRICT accumDeltaV0 = softbody0.mSimDelta;

		const float4 relPos1 = (pos1 - pos0);

		const float4 low_high_angle = constraint.low_high_angles[offset];
		
		PxVec3 error;
		if (isConeLimitedEnabled(low_high_angle.z, low_high_angle.x, low_high_angle.y))
		{
			const float4 attachmentBary = constraint.attachmentBarycentric[offset];
			const float constraintOffset = constraint.constraintOffset[offset];
			const float4 projectP = p00*attachmentBary.x + p01*attachmentBary.y + p02*attachmentBary.z + p03*attachmentBary.w;
			const float4 axis = (projectP - pos0);
			const PxVec3 worldAxis = PxVec3(axis.x, axis.y, axis.z).getNormalized();

			error = computeConeLimitedError(low_high_angle.z, low_high_angle.x, low_high_angle.y, worldAxis, 
						PxVec3(relPos1.x, relPos1.y, relPos1.z) - worldAxis*constraintOffset);
		}
		else
		{
			error = PxVec3(relPos1.x, relPos1.y, relPos1.z);
		}

		float4 b0w = make_float4(bary0.x*p00.w, bary0.y*p01.w, bary0.z*p02.w, bary0.w*p03.w);
		float4 b1w = make_float4(bary1.x*p10.w, bary1.y*p11.w, bary1.z*p12.w, bary1.w*p13.w);

		const PxReal wTot0 = b0w.x*bary0.x + b0w.y*bary0.y + b0w.z*bary0.z + b0w.w*bary0.w;
		const PxReal wTot1 = b1w.x*bary1.x + b1w.y*bary1.y + b1w.z*bary1.z + b1w.w*bary1.w;
		const PxReal wTot = wTot0 + wTot1;

		if (wTot > 0.0f)
		{
			const PxVec3 delta = error * (1.0f/wTot);
			updateTetPositionDelta(accumDeltaV1, tetInd1, -delta, b1w, 1.f);
			updateTetPositionDelta(accumDeltaV0, tetInd0, delta, b0w, 1.f);
		}
	}

}

//solve cloth attachment, output soft body/cloth delta
extern "C" __global__ void sb_solveOutputClothAttachmentDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgFEMCloth*								clothes,
	PxgFEMFEMAttachmentConstraint*				attachments,
	const PxU32									numAttachments
)
{
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numAttachments)
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMFEMAttachmentConstraint& constraint = attachments[index];
		const PxU32 elemId1 = constraint.elemId1[offset];
		const PxU32 softBodyId = PxGetSoftBodyId(elemId1);
		const PxU32 tetrahedronIdx = PxGetSoftBodyElementIndex(elemId1);

		PxgSoftBody& softbody = softbodies[softBodyId];
		const float4* simPos = softbody.mSimPosition_InvMass;
		const uint4* simTets = softbody.mSimTetIndices;
		const uint4 tetInd = simTets[tetrahedronIdx];
		const float4 bary1 = constraint.barycentric1[offset];

		const float4 sp0 = simPos[tetInd.x];
		const float4 sp1 = simPos[tetInd.y];
		const float4 sp2 = simPos[tetInd.z];
		const float4 sp3 = simPos[tetInd.w];
		const float4 pos1 = sp0*bary1.x + sp1*bary1.y + sp2*bary1.z + sp3*bary1.w;

		const PxU32 elemInd0 = constraint.elemId0[offset];
		const PxU32 clothId = PxGetClothId(elemInd0);
		const PxU32 elementId = PxGetClothElementIndex(elemInd0);
		const float4 bary0 = constraint.barycentric0[offset];
		PxgFEMCloth& cloth = clothes[clothId];
		const float4* clothPos = cloth.mPosition_InvMass;
		const uint4 triInd = cloth.mTriangleVertexIndices[elementId];
		
		const float4 cp0 = clothPos[triInd.x];
		const float4 cp1 = clothPos[triInd.y];
		const float4 cp2 = clothPos[triInd.z];
		const float4 pos0 = cp0*bary0.x + cp1*bary0.y + cp2*bary0.z;

		//output for the soft body and cloth- just do atomics on this for now...
		float4* PX_RESTRICT accumDeltaV1 = softbody.mSimDelta;
		float4* PX_RESTRICT accumDeltaV0 = cloth.mDeltaPos;

		const float4 relPos1 = (pos1 - pos0);
		
		const float4 low_high_angle = constraint.low_high_angles[offset];

		PxVec3 error;
		if (isConeLimitedEnabled(low_high_angle.z, low_high_angle.x, low_high_angle.y))
		{
			const float4 attachmentBary = constraint.attachmentBarycentric[offset];
			const float constraintOffset = constraint.constraintOffset[offset];
			const float4 projectP = sp0 * attachmentBary.x + sp1 * attachmentBary.y + sp2 * attachmentBary.z + sp3 * attachmentBary.w;
			const float4 axis = (pos1 - projectP);
			const PxVec3 worldAxis = PxVec3(axis.x, axis.y, axis.z).getNormalized();

			error = computeConeLimitedError(low_high_angle.z, low_high_angle.x, low_high_angle.y, worldAxis,
						PxVec3(relPos1.x, relPos1.y, relPos1.z) - worldAxis*constraintOffset);
		}
		else
		{
			error = PxVec3(relPos1.x, relPos1.y, relPos1.z);
		}

		float4 b0w = make_float4(bary0.x*cp0.w, bary0.y*cp1.w, bary0.z*cp2.w, 0.0f);
		float4 b1w = make_float4(bary1.x*sp0.w, bary1.y*sp1.w, bary1.z*sp2.w, bary1.w*sp3.w);

		const PxReal wTot0 = b0w.x*bary0.x + b0w.y*bary0.y + b0w.z*bary0.z;
		const PxReal wTot1 = b1w.x*bary1.x + b1w.y*bary1.y + b1w.z*bary1.z + b1w.w*bary1.w;
		const PxReal wTot = wTot0 + wTot1;
		if (wTot > 0.0f)
		{
			const PxVec3 delta = error * (1.0f/wTot);
			updateTetPositionDelta(accumDeltaV1, tetInd, -delta, b1w, 1.f);
			updateTriPositionDelta(accumDeltaV0, triInd, delta, b0w, 1.f);
		}
	}
}

static __device__ PxU32 findBufferOffset(PxgParticleSystem& particleSystem, PxU32 uniqueBufferId)
{
	PxU32 bufferIndex = findBufferIndexFromUniqueId(particleSystem, uniqueBufferId);
	return particleSystem.mParticleBufferRunsum[bufferIndex];
}

//solve cloth attachment, output soft body/cloth delta
extern "C" __global__ void sb_solveOutputParticleAttachmentDeltaVLaunch(
	PxgSoftBody*								softbodies,
	PxgParticleSystem*							particleSystems,
	PxgFEMFEMAttachmentConstraint*				attachments,
	const PxU32									numAttachments
)
{

	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;


	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{

		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= numAttachments)
			return;


		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMFEMAttachmentConstraint& constraint = attachments[index];
	
		const PxU32 elemId1 = constraint.elemId1[offset];
		const PxU32 softBodyId = PxGetSoftBodyId(elemId1);
		const PxU32 tetrahedronId = PxGetSoftBodyElementIndex(elemId1);
		PxgSoftBody& softbody = softbodies[softBodyId];
		const float4* simPos = softbody.mSimPosition_InvMass;
		const uint4* simTets = softbody.mSimTetIndices;
		const uint4 tetInd = simTets[tetrahedronId];

		const float4 bary1 = constraint.barycentric1[offset];

		//Output for the soft body - just do atomics on this for now...
		float4* PX_RESTRICT accumDeltaV = softbody.mSimDelta;

		const float4 sp0 = simPos[tetInd.x];
		const float4 sp1 = simPos[tetInd.y];
		const float4 sp2 = simPos[tetInd.z];
		const float4 sp3 = simPos[tetInd.w];
		const float4 pos1 = sp0 * bary1.x + sp1 * bary1.y + sp2 * bary1.z + sp3 * bary1.w;

		const PxReal invMass1 = pos1.w;

		const PxU64 elemId0 = constraint.elemId0[offset];
		const PxU32 particleSystemId = PxGetParticleSystemId(elemId0);		
		PxgParticleSystem& particleSystem = particleSystems[particleSystemId];

		PxU32 userBufferId = PxU32(constraint.barycentric0[offset].x);
		PxU32 o = findBufferOffset(particleSystem, userBufferId);		
		PxU32 particleId = PxGetParticleIndex(elemId0) + o;

		const PxU32* PX_RESTRICT reverseLookup = particleSystem.mUnsortedToSortedMapping;
		particleId = reverseLookup[particleId];
		const float4 pos0 = particleSystem.mSortedPositions_InvMass[particleId];

		const PxReal invMass0 = pos0.w;

		if (invMass1 == 0.f && invMass0 == 0.f)
			continue; //Constrainint an infinte mass particle to an infinite mass soft body. Won't work so skip!

		const float4 diff = (pos1 - pos0);
		const PxVec3 error(diff.x, diff.y, diff.z);

		float4 b1w = make_float4(bary1.x*sp0.w, bary1.y*sp1.w, bary1.z*sp2.w, bary1.w*sp3.w);

		const PxReal wTot0 = pos0.w;
		const PxReal wTot1 = b1w.x*bary1.x + b1w.y*bary1.y + b1w.z*bary1.z + b1w.w*bary1.w;
		const PxReal wTot = wTot0 + wTot1;
		
		if (wTot > 0.0f)
		{
			const PxVec3 delta = error * (1.0f/wTot);
			updateTetPositionDelta(accumDeltaV, tetInd, -delta, b1w, 1.f);
			if (pos0.w > 0.0f)
			{
				AtomicAdd(particleSystem.mDelta[particleId], delta*pos0.w);
			}
		}
	}
}

//Multiple blocks deal with one soft body 
extern "C" __global__ void sb_gm_applyExternalDeltasLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies,
	const PxReal invDt
)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];

	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	float4 * curPositions = shSoftbody.mSimPosition_InvMass;
	float4* vels = shSoftbody.mSimVelocity_InvMass;

	const PxU32 vertIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbVerts = shSoftbody.mNumVertsGM;

	if(vertIdx < nbVerts)
	{
		float4 pos = curPositions[vertIdx];
		float4 vel = vels[vertIdx];

		float4 delta = shSoftbody.mSimDelta[vertIdx];

		const float scale = 1.0f / PxMax(delta.w, 1.f);

		delta.x *= scale; delta.y *= scale, delta.z *= scale;
		delta.w = 0.f;

		const float4 newVel = vel + delta * invDt;

		//tempPos[vertIdx] = pos;
		curPositions[vertIdx] = pos + delta;
		
		vels[vertIdx] = newVel;

		shSoftbody.mSimDeltaPos[vertIdx] += delta;


		//clear delta
		shSoftbody.mSimDelta[vertIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
	}
}

// same as applyExternalDeltas, but with precomputed scale per softbody.
extern "C" __global__ void sb_gm_applyDeformationDeltasLaunch(
	PxgSoftBody * gSoftbodies,
	const PxU32 * activeSoftbodies,
	const PxReal invDt
)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];

	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	float4* PX_RESTRICT curPositions = shSoftbody.mSimPosition_InvMass;
	float4* PX_RESTRICT vels = shSoftbody.mSimVelocity_InvMass;

	const PxU32 groupThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;
	const PxU32 nbJacobiVerts = shSoftbody.mNumJacobiVertices;

	if (groupThreadIdx < nbJacobiVerts)
	{
		const PxU32 vertIdx = shSoftbody.mSimJacobiVertIndices[groupThreadIdx];

		float4 pos = curPositions[vertIdx];
		float4 vel = vels[vertIdx];

		float4 delta = shSoftbody.mSimDelta[vertIdx];
		const float scale = shSoftbody.mJacobiScale;

		delta.x *= scale; delta.y *= scale, delta.z *= scale;
		delta.w = 0.f;

		const float4 newVel = vel + delta * invDt;

		curPositions[vertIdx] = pos + delta;
		vels[vertIdx] = newVel;

		shSoftbody.mSimDeltaPos[vertIdx] += delta;
		shSoftbody.mSimDelta[vertIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
	}
}

//Multiple blocks deal with one soft body 
extern "C" __global__ void sb_calculateStressLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies,
	PxsDeformableVolumeMaterialData* matData
)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];

	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	if (shSoftbody.mVolumeFlags & PxDeformableVolumeFlag::eCOMPUTE_STRESS_TENSOR ||
		!(shSoftbody.mBodyFlags & PxDeformableBodyFlag::eDISABLE_SELF_COLLISION))
	{

		const PxU32 tetrahedronIdx = threadIdx.x + blockIdx.x * blockDim.x;

		const PxU32 nbTets = shSoftbody.mNumTets;

		if (tetrahedronIdx < nbTets)
		{
			const float4 * positions = shSoftbody.mPosition_InvMass;

			const uint4* tetInds = shSoftbody.mTetIndices;

			const PxMat33* restPoses = shSoftbody.mTetraRestPoses;
			float4* tetraRotations = shSoftbody.mTetraRotations;

			const PxQuat initialRotation = shSoftbody.mInitialRotation;

			const uint4 tetIdx = tetInds[tetrahedronIdx];

			const float4 p0 = positions[tetIdx.x];
			const float4 p1 = positions[tetIdx.y];
			const float4 p2 = positions[tetIdx.z];
			const float4 p3 = positions[tetIdx.w];

			//compute displacement field
			const PxVec3 u1 = PxLoad3(p1 - p0);
			const PxVec3 u2 = PxLoad3(p2 - p0);
			const PxVec3 u3 = PxLoad3(p3 - p0);

			// load rest pose
			PxMat33 Qinv = restPoses[tetrahedronIdx];

			const PxMat33 P = PxMat33(u1, u2, u3);

			PxMat33 F = P * Qinv;

			// load rotation factor (updated once per step)
			float4 tetraRot = tetraRotations[tetrahedronIdx];

			PxQuat q(tetraRot.x, tetraRot.y, tetraRot.z, tetraRot.w);

			PxMat33 R(q);

			// remove rotation factor from strain (corotation)
			PxMat33 RTranspose = R.getTranspose();

			// calculate deformation gradient
			F = RTranspose * F;


			const PxU16 globalMaterialIndex = shSoftbody.mMaterialIndices[tetrahedronIdx];
			PxsDeformableVolumeMaterialData& mat = matData[globalMaterialIndex];

			PxReal E = mat.youngs;
			PxReal nu = mat.poissons;
			//conversion from Young's modulus to Lame parameters
			PxReal lambda = E * nu / ((1.0f + nu)*(1.0f - 2.0f*nu));
			PxReal mu = 0.5f*E / (1.0f + nu);
			//reparameterization of Lame parameters to obtain rest stability (from Pixar's Stable Neo-Hookean Flesh Simulation)
			lambda += mu * (5.0f / 6.0f);
			mu = mu * (4.0f / 3.0f);

			PxMat33 I(PxIdentity);
			// compute Cauchy stress
			PxMat33 eps = (F.getTranspose() + F) * 0.5f - I;
			const float trace = eps.column0.x + eps.column1.y + eps.column2.z;
			PxMat33 sigma = eps*2.0*mu + I * lambda*trace;

			const PxReal q1 = eps[0][0];
			const PxReal q2 = eps[1][1];
			const PxReal q3 = eps[2][2];
			const PxReal q12 = q1 - q2;
			const PxReal q23 = q2 - q3;
			const PxReal q31 = q3 - q1;
			const PxReal constant = 0.70710678118654752440084436210485f;//1 / PxSqrt(2.0)
			const PxReal stress = PxSqrt(q12*q12 + q23 * q23 + q31 * q31) *constant;

			shSoftbody.mTetraStresses[tetrahedronIdx] = sigma;
			shSoftbody.mTetraStressCoefficient[tetrahedronIdx] = stress;

		}
	}
}

extern "C" __global__ void sb_plasticDeformLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies,
	PxsDeformableVolumeMaterialData* materials
)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];

	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	if (false/*shSoftbody.mFlags & PxSoftBodyFlag::eENABLE_PLASTIC_DEFORMATION*/)
	{
		const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		const PxU32 nbTets = shSoftbody.mNumTetsGM;
		if (idx < nbTets)
		{
			const float4 * positions = shSoftbody.mSimPosition_InvMass;

			PxU32* orderedTets = shSoftbody.mSimOrderedTetrahedrons;

			const PxU32 tetrahedronIdx = orderedTets[idx];

			const uint4* tetInds = shSoftbody.mSimTetIndices;

			PxgMat33Block* restPoses = shSoftbody.mSimTetraRestPoses;
			PxgMat33Block* origQinv = shSoftbody.mOrigQinv;
			float4* tetraRotations = shSoftbody.mSimTetraRotations;
			bool* vertsAreDeformed = shSoftbody.mVertsAreDeformed;
			bool* vertsCanNotDeform = shSoftbody.mVertsCanNotDeform;

			const uint4 tetIdx = tetInds[tetrahedronIdx];

			const float4 p0 = positions[tetIdx.x];
			const float4 p1 = positions[tetIdx.y];
			const float4 p2 = positions[tetIdx.z];
			const float4 p3 = positions[tetIdx.w];

			//compute displacement field
			const PxVec3 u1 = PxLoad3(p1 - p0);
			const PxVec3 u2 = PxLoad3(p2 - p0);
			const PxVec3 u3 = PxLoad3(p3 - p0);

			// load rest pose
			PxMat33 Qinv;
			loadPxMat33(restPoses[idx / 32], threadIndexInWarp, Qinv);

			PxMat33 Q = Qinv.getInverse();

			

			PxMat33 QinvOrig;
			loadPxMat33(origQinv[idx/32], threadIndexInWarp, QinvOrig);
			PxMat33 QOrig = QinvOrig.getInverse();

			float4 tetraRot = tetraRotations[idx];
			PxQuat q(tetraRot.x, tetraRot.y, tetraRot.z, tetraRot.w);

			PxMat33 P = PxMat33(u1, u2, u3);

			computeGMTetrahedronRotations(u1, u2, u3, Qinv, q, false);

			tetraRotations[idx] = make_float4(q.x, q.y, q.z, q.w);

			PxMat33 R(q);

			// remove rotation factor from strain (corotation)
			PxMat33 RTranspose = R.getTranspose();

			P = RTranspose * P;

			const PxU32 globalMaterialIndex = shSoftbody.mOrderedMaterialIndices[idx];
			PxsDeformableVolumeMaterialData& material = materials[globalMaterialIndex];
			//P = (RTranspose * P);

			PxReal defThresh2 = material.deformThreshold*material.deformThreshold;
			PxReal defLimitLow2 = material.deformLowLimitRatio*material.deformLowLimitRatio;
			PxReal defLimitHigh2 = material.deformHighLimitRatio*material.deformHighLimitRatio;

			bool changed = false;

			bool change1 = false, change2 = false, change3 = false;
			bool noDeform = false;

			{
				PxReal ratio2 = P.column0.magnitudeSquared() / QOrig.column0.magnitudeSquared();
				if (ratio2 >= defLimitLow2 && ratio2 <= defLimitHigh2)
				{
					if ((Q.column0 - P.column0).magnitudeSquared() > defThresh2)
					{
						changed = true;
						change1 = true;
					}
				}
				else
					noDeform = true;
			}

			{
				PxReal ratio2 = P.column1.magnitudeSquared() / QOrig.column1.magnitudeSquared();
				if (ratio2 >= defLimitLow2 && ratio2 <= defLimitHigh2)
				{
					if ((Q.column1 - P.column1).magnitudeSquared() > defThresh2)
					{
						changed = true;
						change1 = true;
					}
				}
				else
					noDeform = true;
			}

			
				
			{
				PxReal ratio2 = P.column2.magnitudeSquared() / QOrig.column2.magnitudeSquared();
				if (ratio2 >= defLimitLow2 && ratio2 <= defLimitHigh2)
				{
					if ((Q.column2 - P.column2).magnitudeSquared() > defThresh2)
					{
						changed = true;
						change1 = true;
					}
				}
				else
					noDeform = true;
			}

			PxReal ratio = P.getDeterminant() / QOrig.getDeterminant();

			if(ratio < material.deformLowLimitRatio || ratio > material.deformHighLimitRatio)
				noDeform = true;

			if (changed && !noDeform)
			{
				{
					vertsAreDeformed[tetIdx.x] = true;
					if (change1)
						vertsAreDeformed[tetIdx.y] = true;
					if (change2)
						vertsAreDeformed[tetIdx.z] = true;
					if (change3)
						vertsAreDeformed[tetIdx.w] = true;
				}
			}

			if (noDeform)
			{
				vertsCanNotDeform[tetIdx.x] = true;
				vertsCanNotDeform[tetIdx.y] = true;
				vertsCanNotDeform[tetIdx.z] = true;
				vertsCanNotDeform[tetIdx.w] = true;
			}

		}
	}
}

extern "C" __global__ void sb_plasticDeformLaunch2(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies
)
{
	__shared__ __align__(16) char tSoftbody[sizeof(PxgSoftBody)];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];

	PxgSoftBody& softbody = gSoftbodies[softbodyId];

	uint2* sSoftbody = reinterpret_cast<uint2*>(&softbody);
	uint2* dSoftbody = reinterpret_cast<uint2*>(&tSoftbody);

	blockCopy<uint2>(dSoftbody, sSoftbody, sizeof(PxgSoftBody));

	__syncthreads();

	PxgSoftBody& shSoftbody = reinterpret_cast<PxgSoftBody&>(*tSoftbody);

	if (false/*shSoftbody.mFlags & PxSoftBodyFlag::eENABLE_PLASTIC_DEFORMATION*/)
	{
		const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;
		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		const PxU32 nbTets = shSoftbody.mNumTetsGM;
		if (idx < nbTets)
		{
			const float4 * positions = shSoftbody.mSimPosition_InvMass;

			PxU32* orderedTets = shSoftbody.mSimOrderedTetrahedrons;

			const PxU32 tetrahedronIdx = orderedTets[idx];

			const uint4* tetInds = shSoftbody.mSimTetIndices;

			PxgMat33Block* restPoses = shSoftbody.mSimTetraRestPoses;
			float4* tetraRotations = shSoftbody.mSimTetraRotations;

			bool* vertsAreDeformed = shSoftbody.mVertsAreDeformed;
			bool* vertsCanNotDeform = shSoftbody.mVertsCanNotDeform;

			const uint4 tetIdx = tetInds[tetrahedronIdx];

			bool deformed0 = vertsAreDeformed[tetIdx.x];
			bool deformed1 = vertsAreDeformed[tetIdx.y];
			bool deformed2 = vertsAreDeformed[tetIdx.z];
			bool deformed3 = vertsAreDeformed[tetIdx.w];

			bool noDeform = vertsCanNotDeform[tetIdx.x] || vertsCanNotDeform[tetIdx.y] || vertsCanNotDeform[tetIdx.z] || vertsCanNotDeform[tetIdx.w];

			if (!noDeform && (deformed0 || deformed1 || deformed2 || deformed3))
			{


				const float4 p0 = positions[tetIdx.x];
				const float4 p1 = positions[tetIdx.y];
				const float4 p2 = positions[tetIdx.z];
				const float4 p3 = positions[tetIdx.w];



				//compute displacement field
				const PxVec3 u1 = PxLoad3(p1 - p0);
				const PxVec3 u2 = PxLoad3(p2 - p0);
				const PxVec3 u3 = PxLoad3(p3 - p0);

				float4 tetraRot = tetraRotations[idx];
				PxQuat q(tetraRot.x, tetraRot.y, tetraRot.z, tetraRot.w);

				PxMat33 P = PxMat33(u1, u2, u3);

				PxMat33 R(q);

				// remove rotation factor from strain (corotation)
				PxMat33 RTranspose = R.getTranspose();

				P = RTranspose * P;

				if(P.getDeterminant() >= 1e-5f)
				{
					storePxMat33(restPoses[idx/32], threadIndexInWarp, P.getInverse());
				}
			}

		}
	}
}

extern "C" __global__ void sb_initPlasticDeformLaunch(
	PxgMat33Block* matrices,
	const PxU32 numTets
)
{
	const PxU32 idx = threadIdx.x + blockIdx.x * blockDim.x;
	if (idx < numTets)
	{
		storePxMat33(matrices[idx/32], idx&31, PxMat33(PxZero));
	}
}

extern "C" __global__ void sb_copyOrApplySoftBodyDataDEPRECATED(
	PxgSoftBody* PX_RESTRICT softbodies,
	const PxU32 dataIndex,
	const PxU32* softBodyIndices,
	const PxU32 softBodyIndicesSize,
	PxU32** data,
	const PxU32* dataSizes,
	const PxU32 threadsPerSoftBody,
	const PxU32 applyDataToSoftBodies)
{
	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;
	const PxU32 sectionIndex = globalThreadIndex / threadsPerSoftBody;

	
	if (sectionIndex < softBodyIndicesSize)
	{
		const PxU32 softBodyIndex = softBodyIndices[sectionIndex];
		PxgSoftBody& softbody = softbodies[softBodyIndex];
		
		PxU32* dest;
		PxU32 * source;
		if (applyDataToSoftBodies)
		{
			dest = (PxU32*)(((void**)&softbody)[dataIndex]);
			source = data[sectionIndex];
		}
		else
		{
			source = (PxU32*)(((void**)&softbody)[dataIndex]);
			dest = data[sectionIndex];
		}

		PxU32 index = globalThreadIndex % threadsPerSoftBody;
		const PxU32 end = (dataSizes[sectionIndex] + 4 - 1) / 4; //We copy 4 bytes at a time
		while (index < end)
		{
			dest[index] = source[index]; //Write to the output buffer	
			index += threadsPerSoftBody;
		}
	}
}

extern "C" __global__ void sb_gm_finalizeVelocitiesLaunch(
	PxgSoftBody* gSoftbodies,
	const PxU32* activeSoftbodies,
	const PxReal invDt,
	const PxReal velocityScale,
	const PxReal dt,
	const bool alwaysRunVelocityAveraging
)
{
	__shared__ bool isAwake[PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION / 32];

	const PxU32 softbodyId = activeSoftbodies[blockIdx.y];
	PxgSoftBody& shSoftbody = gSoftbodies[softbodyId];

	float4* deltaPos = shSoftbody.mSimDeltaPos;
	float4* vels = shSoftbody.mSimVelocity_InvMass;
	float4* positions = shSoftbody.mSimPosition_InvMass;

	const PxU32 vertIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbVerts = shSoftbody.mNumVertsGM;

	bool awake = false;

	if (vertIdx < nbVerts)
	{
		const PxReal settleTolerance = shSoftbody.mSettlingThreshold*dt;
		const PxReal tolerance = shSoftbody.mSleepThreshold*dt;
		const PxReal sleepDamping = 1.f - PxMin(1.f, shSoftbody.mSettlingDamping*dt);

		float4 delta = deltaPos[vertIdx];
		PxVec3 tDelta = PxLoad3(delta);
		PxVec3 deltaVel = tDelta*invDt;
		float4 pos = positions[vertIdx];

		if (pos.w != 0.0f) // skip kinematic and  infinite mass particles
		{
			PxReal velocityScaling = 1.0f;

			const PxReal magSq = tDelta.magnitudeSquared();
			if (magSq < settleTolerance*settleTolerance)
			{
				awake = magSq >= tolerance*tolerance;

				velocityScaling = sleepDamping;
			}
			else
			{
				awake = true;
			}

			if (alwaysRunVelocityAveraging || velocityScaling != 1.0f)
			{
				float4 vel = vels[vertIdx];
				PxVec3 tVel = PxLoad3(vel);

				if (alwaysRunVelocityAveraging && deltaVel.magnitudeSquared() < tVel.magnitudeSquared())
					tVel = tVel * (velocityScale)+deltaVel * (1.f - velocityScale);

				vels[vertIdx] = make_float4(velocityScaling * tVel.x, velocityScaling * tVel.y, velocityScaling * tVel.z, vel.w);
			}
		}
	}

	awake = __any_sync(FULL_MASK, awake);

	if ((threadIdx.x & 31) == 0)
		isAwake[threadIdx.x / 32] = awake;

	__syncthreads();

	if (threadIdx.x < 32)
	{
		awake = isAwake[threadIdx.x];
		awake = __any_sync(FULL_MASK, awake);

		if (awake)
		{
			atomicOr(&shSoftbody.mAwake, 1u);
		}
	}
}

extern "C" __global__ void sb_sleeping(
	PxgSoftBody* PX_RESTRICT softBodies,
	const PxU32 numActiveSoftBodies,
	const PxU32* activeId,
	const PxReal dt,
	const PxReal resetCounter,
	PxReal* wakeCounters,
	PxU32* stateChangedMask)
{
	const PxU32 NumWarps = PxgSoftBodyKernelBlockDim::SB_PREINTEGRATION / 32;
	__shared__ PxU32 shMasks[NumWarps];
	const PxU32 globalThreadIdx = threadIdx.x + blockIdx.x * blockDim.x;

	if (globalThreadIdx >= numActiveSoftBodies)
		return;

	const PxU32 id = activeId[globalThreadIdx];
	PxgSoftBody& softBody = softBodies[id];

	bool reset = softBody.mAwake;

	PxReal counter = wakeCounters[id];
	bool wasActive = counter > 0.f;

	if (reset)
		counter = resetCounter;
	else
		counter = PxMax(0.f, counter - dt);

	bool isActive = counter > 0.f;

	PxU32 mask = __ballot_sync(FULL_MASK, isActive^wasActive);

	/*if (isActive^wasActive)
	{
		if (isActive)
			printf("%i: numActive = %i Waking soft body %i\n", globalThreadIdx, numActiveSoftBodies, id);
		else
			printf("%i: numActive = %i, Soft body %i ready for sleep\n", globalThreadIdx, numActiveSoftBodies, id);
	}*/

	wakeCounters[id] = counter;
	if ((threadIdx.x & 31) == 0)
		shMasks[threadIdx.x / 32] = mask;

	__syncthreads();

	const PxU32 startIdx = (blockIdx.x*blockDim.x) / 32;
	if (threadIdx.x < NumWarps && (startIdx+threadIdx.x) < ((numActiveSoftBodies+31)/32))
		stateChangedMask[startIdx + threadIdx.x] = shMasks[threadIdx.x];


}

