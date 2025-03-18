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

#include "PxgFEMCloth.h"
#include "PxgFEMCore.h"
#include "PxgFEMClothCore.h"
#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxMathUtils.h"
#include "copy.cuh"
#include "assert.h"
#include "atomic.cuh"
#include "stdio.h"
#include "PxgSolverCoreDesc.h"
#include "PxNodeIndex.h"
#include "PxgBodySim.h"
#include "PxgArticulation.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgParticleSystem.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "particleSystem.cuh"
#include "PxgSimulationCoreDesc.h"
#include "dataReadWriteHelper.cuh"
#include "attachments.cuh"
#include "deformableCollision.cuh"
#include "FEMClothUtil.cuh"

#define FEMCLOTH_BIAS_COEFFICIENT 0.7f

using namespace physx;

extern "C" __host__ void initFEMClothKernels2() {}

__device__ void updateTrianglePosDelta(const float4& invMasses, const float4& barycentric, const uint4& vertInds, const PxVec3& deltaPos,
									   float4* outputDeltaPoses, const PxU32 elementId)
{
	if(invMasses.x > 0.f && PxAbs(barycentric.x) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * invMasses.x * barycentric.x;
		AtomicAdd(outputDeltaPoses[vertInds.x], dP, 1.f);
	}

	if(invMasses.y > 0.f && PxAbs(barycentric.y) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * invMasses.y * barycentric.y;
		AtomicAdd(outputDeltaPoses[vertInds.y], dP, 1.f);
	}

	if(invMasses.z > 0.f && PxAbs(barycentric.z) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * invMasses.z * barycentric.z;
		AtomicAdd(outputDeltaPoses[vertInds.z], dP, 1.f);
	}
}

__device__ void updateTrianglePosDelta(const PxVec3& invMasses, const PxVec3& barycentric, const uint4& vertInds, const PxVec3& deltaPos,
									   float4* outputDeltaPoses, const PxU32 elementId)
{
	if(invMasses.x > 0.f && PxAbs(barycentric.x) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * invMasses.x * barycentric.x;
		AtomicAdd(outputDeltaPoses[vertInds.x], dP, 1.f);
	}

	if(invMasses.y > 0.f && PxAbs(barycentric.y) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * invMasses.y * barycentric.y;
		AtomicAdd(outputDeltaPoses[vertInds.y], dP, 1.f);
	}

	if(invMasses.z > 0.f && PxAbs(barycentric.z) > 1e-6f)
	{
		const PxVec3 dP = deltaPos * invMasses.z * barycentric.z;
		AtomicAdd(outputDeltaPoses[vertInds.z], dP, 1.f);
	}
}

__device__ void updateTrianglePosDelta2(const PxVec3& invMasses, const uint4& vertInds, const PxVec3& deltaPos,
                                        float4* outputDeltaPoses)
{
	if(invMasses.x > 0.f)
	{
		const PxVec3 dP = deltaPos;
		AtomicAdd(outputDeltaPoses[vertInds.x], dP, 1.f);
	}

	if(invMasses.y > 0.f)
	{
		const PxVec3 dP = deltaPos;
		AtomicAdd(outputDeltaPoses[vertInds.y], dP, 1.f);
	}

	if(invMasses.z > 0.f)
	{
		const PxVec3 dP = deltaPos;
		AtomicAdd(outputDeltaPoses[vertInds.z], dP, 1.f);
	}
}

__device__ void updateVertexPosDelta(const PxVec3& deltaPos, const PxVec3& angDelta, const PxVec3& offset,
                                     const PxReal invMass, float4& outputDeltaPose)
{
	if(invMass > 0.f)
	{
		const PxVec3 delta = deltaPos + angDelta.cross(offset);
		AtomicAdd(outputDeltaPose, delta, 1.f);
	}
}

__device__ void updateTrianglePosDelta2(const uint4& vertInds, const PxVec3& deltaPos, const PxVec3& angDelta,
                                        const PxVec3& r0, const PxVec3& r1, const PxVec3& r2, float4* outputDeltaPoses,
                                        const PxU32 workIdx)
{
	{
		const PxVec3 delta = deltaPos + angDelta.cross(r0);
		AtomicAdd(outputDeltaPoses[vertInds.x], delta, 1.f);
	}

	{
		const PxVec3 delta = deltaPos + angDelta.cross(r1);
		AtomicAdd(outputDeltaPoses[vertInds.y], delta, 1.f);
	}

	{
		const PxVec3 delta = deltaPos + angDelta.cross(r2);
		AtomicAdd(outputDeltaPoses[vertInds.z], delta, 1.f);
	}
}

__device__ float4 computeTriangleContact(const float4* vels, const uint4& triVertId, const float2& barycentric,
                                         float4& invMass)
{
	const float4 v0 = vels[triVertId.x];
	const float4 v1 = vels[triVertId.y];
	const float4 v2 = vels[triVertId.z];

	invMass = make_float4(v0.w, v1.w, v2.w, 0.f);

	const PxReal u = 1.0 - barycentric.x - barycentric.y;
	const float4 vel = v0 * u + v1 * barycentric.x + v2 * barycentric.y;

	return vel;
}

__device__ float4 computeTriangleContact(const float4* vels, const uint4& triVertId, const float2& barycentric)
{
	const float4 v0 = vels[triVertId.x];
	const float4 v1 = vels[triVertId.y];
	const float4 v2 = vels[triVertId.z];

	const PxReal u = 1.0 - barycentric.x - barycentric.y;
	const float4 vel = v0 * u + v1 * barycentric.x + v2 * barycentric.y;

	return vel;
}

static __device__ float4 computeTriangleContact(const float4* vels, const uint4& triVertId, const float4& barycentric,
                                                float4& invMass)
{
	const float4 v0 = vels[triVertId.x];
	const float4 v1 = vels[triVertId.y];
	const float4 v2 = vels[triVertId.z];

	invMass = make_float4(v0.w, v1.w, v2.w, 0.f);

	const float4 vel = v0 * barycentric.x + v1 * barycentric.y + v2 * barycentric.z;

	return vel;
}

__device__ PxVec3 loadBarycentric(const float2 bary) { return PxVec3(1.f - bary.x - bary.y, bary.x, bary.y); }

__device__ PxVec3 toPxVec3(const float4 v) { return PxVec3(v.x, v.y, v.z); }

__device__ PxVec3 loadBarycentric(const float4 bary) { return PxVec3(bary.x, bary.y, bary.z); }

__device__ PxVec3 computeMassInertiaOffsets(const float4* pos_invMass, const uint4& triVertId,
                                            const PxVec3& barycentric, const PxReal thickness, PxVec3& offset,
                                            PxVec3& ra, PxVec3& rb, PxVec3& rc, PxVec3& invInertia, PxReal& invMass)
{
	const float4 p0_ = pos_invMass[triVertId.x];
	const float4 p1_ = pos_invMass[triVertId.y];
	const float4 p2_ = pos_invMass[triVertId.z];

	const PxVec3 p0 = toPxVec3(p0_);
	const PxVec3 p1 = toPxVec3(p1_);
	const PxVec3 p2 = toPxVec3(p2_);

	const PxVec3 average = (p0 + p1 + p2) * (1.f / 3.f);
	PxVec3 contact = p0 * barycentric.x + p1 * barycentric.y + p2 * barycentric.z;

	PxReal m0 = 1.f / PxMax(1e-18f, p0_.w);
	PxReal m1 = 1.f / PxMax(1e-18f, p1_.w);
	PxReal m2 = 1.f / PxMax(1e-18f, p2_.w);

	offset = contact - average;
	ra = p0 - average;
	rb = p1 - average;
	rc = p2 - average;

	// additional component added to the inertia to represent the cloth's thickness
	// const PxReal thicknessSq = 6.f*thickness * thickness*(m0 + m1 + m2);
	const PxReal thicknessSq = thickness * thickness * (m0 + m1 + m2);

	invInertia.x = 1.f / (((ra.y * ra.y + ra.z * ra.z) * m0 + (rb.y * rb.y + rb.z * rb.z) * m1 +
	                       (rc.y * rc.y + rc.z * rc.z) * m2 + thicknessSq));
	invInertia.y = 1.f / (((ra.x * ra.x + ra.z * ra.z) * m0 + (rb.x * rb.x + rb.z * rb.z) * m1 +
	                       (rc.x * rc.x + rc.z * rc.z) * m2 + thicknessSq));
	invInertia.z = 1.f / (((ra.x * ra.x + ra.y * ra.y) * m0 + (rb.x * rb.x + rb.y * rb.y) * m1 +
	                       (rc.x * rc.x + rc.y * rc.y) * m2 + thicknessSq));

	invMass = 1.f / (m0 + m1 + m2);

	return contact;
}

__device__ bool triangleVertexCollisionConstraint(PxVec3& dx0, PxVec3& dx1, PxVec3& dx2, PxVec3& dx3, const PxVec3& x0, const PxVec3& x1,
												  const PxVec3& x2, const PxVec3& x3, const PxVec4& invMass, PxReal restDistance,
												  PxReal frictionCoefficient, const float4& relD, const PxVec3& bc)
{
	const PxVec3 relDelta(relD.x, relD.y, relD.z);

	// Contact constraint in the normal direction.
	const PxVec3 disp = x0 - (bc.x * x1 + bc.y * x2 + bc.z * x3);
	const PxReal distSq = disp.magnitudeSquared();

	if(distSq < FEMCLOTH_THRESHOLD)
		return false;

	const PxVec3 normal = disp / PxSqrt(distSq);
	PxReal CN = disp.dot(normal) - restDistance;

	// Max depenetration velocity is disabled until it is needed or becomes useful.
	// const PxReal CN = (maxDisplacement == 0.f || initPen > 0.f) ? disp.dot(normal) - restDistance
	//															: disp.dot(normal) - restDistance + PxMax(-initPen - maxDisplacement, 0.f);

	if(CN < 0.0f)
	{
		const PxReal denom = invMass.x + bc.x * bc.x * invMass.y + bc.y * bc.y * invMass.z + bc.z * bc.z * invMass.w;

		if(denom < FEMCLOTH_THRESHOLD)
			return false;

		const PxReal denomInv = 1.f / denom;
		const PxReal deltaLambdaN = -CN * denomInv;

		PxVec3 commonVec = deltaLambdaN * normal;

		// Friction constraint in the tangent direction.
		// The friction constraint is computed after the collision constraint.
		PxVec3 tanDir = (relDelta - relDelta.dot(normal) * normal);
		const PxReal tanMagSq = tanDir.magnitudeSquared();

		if(frictionCoefficient != 0.0f && tanMagSq > FEMCLOTH_THRESHOLD)
		{
			tanDir /= PxSqrt(tanMagSq);
			PxReal CT = relDelta.dot(tanDir);

			PxReal deltaLambdaT = PxMax(0.f, CT * denomInv); // (-) sign is added in the next line
			deltaLambdaT = -PxMin(deltaLambdaT, frictionCoefficient * PxAbs(deltaLambdaN));

			assert(deltaLambdaT <= 0.f);

			// Minor scale-down apllied for improved robustness.
			const PxReal tanScale = 0.99f;
			commonVec += tanScale * deltaLambdaT * tanDir;
		}

		dx0 += invMass.x * commonVec;
		dx1 -= invMass.y * bc.x * commonVec;
		dx2 -= invMass.z * bc.y * commonVec;
		dx3 -= invMass.w * bc.z * commonVec;

		return true;
	}

	return false;
}

template <typename IterativeData>
static __device__ void queryRigidClothContactReferenceCount(
	PxgFEMCloth* femClothes,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	const PxReal dt,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemVsRigidContactInfo& contactInfo = contactInfos[workIndex];
		PxgFemRigidConstraintBlock& constraint = constraints[workIndex / 32];
		const float4 fricTan0_invMass0 = constraint.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = constraint.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: cloth (both triangle and vertex)
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 clothId = PxGetClothId(pairInd1);
		const PxU32 elementId = PxGetClothElementIndex(pairInd1);

		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId < PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxgFEMCloth& cloth = femClothes[clothId];

			// For vertex collision, only x component of the barycentric coordinates is used (1.0).
			const PxVec3 bcVec = (bc.w == 0) ? PxVec3(bc.x, bc.y, bc.z) : PxVec3(1.0f, 0.0f, 0.0f);
			PxU32 globalRigidBodyId;
			bool isActive;

			if(isTGS)
			{
				FEMCollisionTGS<PxVec3> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, NULL, NULL);
				femCollision.readCloth(cloth, elementId, bc, NULL, true);
				isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId, velocityReader, dt,
												   bcVec, true);
				globalRigidBodyId = femCollision.globalRigidBodyId;
			}
			else
			{
				FEMCollisionPGS<PxVec3> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, NULL, NULL);
				femCollision.readCloth(cloth, elementId, bc, NULL, true);
				isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId, velocityReader, dt,
												   bcVec, true);
				globalRigidBodyId = femCollision.globalRigidBodyId;
			}

			if(isActive)
			{
				// Update cloth
				{
					if(bc.w == 0.f) // Cloth triangle
					{
						// Increment the reference count of the three vertices.
						const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
						atomicAdd(&cloth.mDeltaPos[triVertId.x].w, 1.0f);
						atomicAdd(&cloth.mDeltaPos[triVertId.y].w, 1.0f);
						atomicAdd(&cloth.mDeltaPos[triVertId.z].w, 1.0f);
					}
					else // Cloth vertex
					{
						// Increment the reference count of the single vertex.
						atomicAdd(&cloth.mDeltaPos[elementId].w, 1.0f);
					}
				}

				// Update rigidbody
				if(!rigidId.isStaticBody() && fricTan0_invMass0.w != 0.0f)
				{
					// Increment the reference count of the rigid body.
					atomicAdd(&rigidBodyReferenceCounts[globalRigidBodyId], 1);
				}
			}
		}
	}
}

extern "C" __global__
void cloth_queryRigidClothContactReferenceCountLaunch(
	PxgFEMCloth* femClothes,
	PxgFemVsRigidContactInfo * contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc,
	const PxReal dt,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts)
{
	const bool isTGS = false;
	queryRigidClothContactReferenceCount(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
		sharedDesc, dt, appliedForces, rigidBodyReferenceCounts, isTGS);
}

template <typename IterativeData>
static __device__ void solveRigidClothContact(
	PxgFEMCloth* femClothes,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	const PxReal dt,
	float4* rigidDeltaVel,
	float4* appliedForces, // output
	PxU32* rigidBodyReferenceCounts,
	PxsDeformableSurfaceMaterialData* materials,
	bool updateRigid,
	const PxsMaterialData* PX_RESTRICT rigidBodyMaterials,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemVsRigidContactInfo& contactInfo = contactInfos[workIndex];
		PxgFemRigidConstraintBlock& constraint = constraints[workIndex / 32];
		const float4 fricTan0_invMass0 = constraint.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = constraint.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: cloth (both triangle and vertex)
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 clothId = PxGetClothId(pairInd1);
		const PxU32 elementId = PxGetClothElementIndex(pairInd1);

		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId < PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			PxgFEMCloth& cloth = femClothes[clothId];

			// For vertex collision, only x component of the barycentric coordinates is used (1.0).
			const PxVec3 bcVec = (bc.w == 0) ? PxVec3(bc.x, bc.y, bc.z) : PxVec3(1.0f, 0.0f, 0.0f);

			if(isTGS)
			{
				FEMCollisionTGS<PxVec3> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, rigidBodyReferenceCounts,
										   &rigidBodyMaterials[contactInfo.rigidMatInd]);
				femCollision.readCloth(cloth, elementId, bc, materials, false);
				const bool isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId,
															  velocityReader, dt, bcVec, false);

				if(updateRigid) // Update rigid body
				{
					PxVec3 deltaLinVel0, deltaAngVel0;
					appliedForces[workIndex] = femCollision.computeRigidChange(deltaLinVel0, deltaAngVel0, fricTan0_invMass0.w, rigidId);
					femCollision.writeRigidBody(rigidDeltaVel, deltaLinVel0, deltaAngVel0, isActive, workIndex, workIndex + tNumContacts);
				}
				else if(isActive) // Update cloth
				{
					PxVec3 deltaPos;
					appliedForces[workIndex] = femCollision.computeFEMChange(deltaPos, dt);
					femCollision.writeCloth(cloth, elementId, bc, deltaPos);
				}
			}
			else // PGS
			{
				FEMCollisionPGS<PxVec3> femCollision;
				femCollision.readRigidBody(prePrepDesc, rigidId, fricTan0_invMass0.w, numSolverBodies, rigidBodyReferenceCounts,
										   &rigidBodyMaterials[contactInfo.rigidMatInd]);
				femCollision.readCloth(cloth, elementId, bc, materials, false);
				const bool isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId,
															  velocityReader, dt, bcVec, false);

				if(updateRigid) // Update rigid body
				{
					PxVec3 deltaLinVel0, deltaAngVel0;
					appliedForces[workIndex] = femCollision.computeRigidChange(deltaLinVel0, deltaAngVel0, fricTan0_invMass0.w, rigidId);
					femCollision.writeRigidBody(rigidDeltaVel, deltaLinVel0, deltaAngVel0, isActive, workIndex, workIndex + tNumContacts);
				}
				else if(isActive) // Update cloth
				{
					PxVec3 deltaPos;
					appliedForces[workIndex] = femCollision.computeFEMChange(deltaPos, dt);
					femCollision.writeCloth(cloth, elementId, bc, deltaPos);
				}
			}
		}
	}
}

extern "C" __global__ 
void cloth_solveOutputClothDeltaVLaunch(
	PxgFEMCloth* femClothes, 
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts, 
	PxgPrePrepDesc* prePrepDesc, 
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc, 
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, 
	const PxReal dt,
	float4 * rigidDeltaVel,
	float4* appliedForces, // output
	PxU32 * rigidBodyReferenceCounts,
	PxsDeformableSurfaceMaterialData * materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool updateRigid = false; // Update cloth
	const bool isTGS = false;
	solveRigidClothContact(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, dt,
						   rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, materials, updateRigid, rigidBodyMaterials, isTGS);
}

// solve collision between cloth and primitives based on the sorted contact by rigid id
// store new velocity to rigid body buffer
extern "C" __global__ 
void cloth_solveOutputRigidDeltaVLaunch(
	PxgFEMCloth* femClothes, 
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts, 
	PxgPrePrepDesc* prePrepDesc, 
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc, 
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc,
	float4* rigidDeltaVel, // output
	float4* appliedForces, 
	PxU32* rigidBodyReferenceCounts,
	const PxReal dt,
	PxsDeformableSurfaceMaterialData* materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool updateRigid = true;
	const bool isTGS = false;
	solveRigidClothContact(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, dt,
						   rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, materials, updateRigid, rigidBodyMaterials, isTGS);
}

extern "C" __global__
void cloth_queryRigidClothContactReferenceCountTGSLaunch(
	PxgFEMCloth* femClothes,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	const PxReal dt,
	float4* appliedForces,
	PxU32* rigidBodyReferenceCounts)
{
	const bool isTGS = true;
	queryRigidClothContactReferenceCount(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
										 sharedDesc, dt, appliedForces, rigidBodyReferenceCounts, isTGS);
}

extern "C" __global__
void cloth_solveOutputClothDeltaVTGSLaunch(
	PxgFEMCloth* femClothes,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	const PxReal dt,
	float4* rigidDeltaVel,
	float4* appliedForces, //output
	PxU32* rigidBodyReferenceCounts,
	PxsDeformableSurfaceMaterialData* materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool updateRigid = false; // Update cloth
	const bool isTGS = true;
	solveRigidClothContact(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, dt,
						   rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, materials, updateRigid, rigidBodyMaterials, isTGS);
}

// solve collision between cloth and primitives based on the sorted contact by rigid id
// store new velocity to rigid body buffer
extern "C" __global__ void cloth_solveOutputRigidDeltaVTGSLaunch(
	PxgFEMCloth* femClothes,
	PxgFemVsRigidContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	float4* rigidDeltaVel, // output
	float4* appliedForces, //output
	PxU32* rigidBodyReferenceCounts,
	const PxReal dt,
	PxsDeformableSurfaceMaterialData* materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool updateRigid = true;
	const bool isTGS = true;
	solveRigidClothContact(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, dt,
						   rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, materials, updateRigid, rigidBodyMaterials, isTGS);
}



//! 
//! \brief    : solve cloth vs. cloth collision
//! 

extern "C" __global__ __launch_bounds__(256, 4) 
void cloth_solveClothTriClothVertDeltaVLaunch(
    PxgFEMCloth* cloths, 
	PxgFemContactInfo* contactInfos, 
	PxgClothConstraintBlock* constraints, 
	PxU32* numContacts,
	PxReal dt)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];
		PxgClothConstraintBlock& constraint = constraints[workIndex / 32];

		// First actor: cloth0 vertex
		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		const PxU32 clothId0 = PxGetClothId(pairInd0);
		PxgFEMCloth& cloth0 = cloths[clothId0];
		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0); // Vertex id
		const float4 delta0 = cloth0.mAccumulatedDeltaPos[elementId0];
		const float4 x0 = cloth0.mPosition_InvMass[elementId0];
		//const float4 delta0 = x0 - cloth0.mPrevPosition_InvMass[elementId0];

		// Second actor: cloth1 triangle
		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 clothId1 = PxGetClothId(pairInd1);
		PxgFEMCloth& cloth1 = cloths[clothId1];
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);
		const uint4 triVertId1 = cloth1.mTriangleVertexIndices[elementId1];

		const float4 delta1 = cloth1.mAccumulatedDeltaPos[triVertId1.x];
		const float4 delta2 = cloth1.mAccumulatedDeltaPos[triVertId1.y];
		const float4 delta3 = cloth1.mAccumulatedDeltaPos[triVertId1.z];
		const float4 x1 = cloth1.mPosition_InvMass[triVertId1.x];
		const float4 x2 = cloth1.mPosition_InvMass[triVertId1.y];
		const float4 x3 = cloth1.mPosition_InvMass[triVertId1.z];
		//const float4 delta1 = x1 - cloth1.mPrevPosition_InvMass[triVertId1.x];
		//const float4 delta2 = x2 - cloth1.mPrevPosition_InvMass[triVertId1.y];
		//const float4 delta3 = x3 - cloth1.mPrevPosition_InvMass[triVertId1.z];

		//const float4 barycentric_friction_restDist = constraint.barycentric_friction_restDist[threadIndexInWarp];
		const float2 friction_restDist = constraint.friction_restDist[threadIndexInWarp];

		const PxReal frictionCoefficient = friction_restDist.x;
		const PxReal restDistance = friction_restDist.y;

		PxReal w0, w1, w2, w3;
		const PxVec3 xx0 = PxLoad3(x0, w0);
		const PxVec3 xx1 = PxLoad3(x1, w1);
		const PxVec3 xx2 = PxLoad3(x2, w2);
		const PxVec3 xx3 = PxLoad3(x3, w3);

		const PxVec4 invMasses(w0, w1, w2, w3);

#if 1 // Always update the contact point.

		PxReal s, t;
		const PxVec3 closestPt = closestPtPointTriangle(xx0, xx1, xx2, xx3, s, t);

#else // Use the precomputed contact point.

		// To use precomputed contact point, store barycentric coordinates in PxgClothConstraintBlock.
		const PxReal s = barycentric_friction_restDist.x;
		const PxReal t = barycentric_friction_restDist.y;

#endif

		const PxVec3 bc(1.0f - s - t, s, t);

		PxVec3 dx0(0.f), dx1(0.f), dx2(0.f), dx3(0.f);
		const float4 relDelta = delta0 - (bc.x * delta1 + bc.y * delta2 + bc.z * delta3); // cloth0 - cloth1

		if(triangleVertexCollisionConstraint(dx0, dx1, dx2, dx3, xx0, xx1, xx2, xx3, invMasses, restDistance, frictionCoefficient, relDelta,
											 bc))
		{
			if(x0.w != 0.f)
			{
				AtomicAdd(cloth0.mDeltaPos[elementId0], dx0, 1.0f);
			}

			if(x1.w != 0.f && bc.x > 1e-3f)
			{
				AtomicAdd(cloth1.mDeltaPos[triVertId1.x], dx1, 1.0f);
			}

			if(x2.w != 0.f && bc.y > 1e-3f)
			{
				AtomicAdd(cloth1.mDeltaPos[triVertId1.y], dx2, 1.0f);
			}

			if(x3.w != 0.f && bc.z > 1e-3f)
			{
				AtomicAdd(cloth1.mDeltaPos[triVertId1.z], dx3, 1.0f);
			}
		}
	}
}

// multiple blocks deal with one cloth
extern "C" __global__ 
void cloth_applyExternalDeltasLaunch(
	PxgFEMCloth* femClothes, 
	const PxU32* activeClothes,
	const PxReal dt)
{
	__shared__ __align__(16) char tFEMCloth[sizeof(PxgFEMCloth)];

	const PxU32 clothId = activeClothes[blockIdx.y];

	PxgFEMCloth& cloth = femClothes[clothId];

	uint2* sCloth = reinterpret_cast<uint2*>(&cloth);
	uint2* dCloth = reinterpret_cast<uint2*>(&tFEMCloth);

	blockCopy<uint2>(dCloth, sCloth, sizeof(PxgFEMCloth));

	__syncthreads();

	PxgFEMCloth& shCloth = reinterpret_cast<PxgFEMCloth&>(*tFEMCloth);

	float4* curPositions = shCloth.mPosition_InvMass;
	float4* velocities = shCloth.mVelocity_InvMass;

	float4* accumulatedDeltaPos = shCloth.mAccumulatedDeltaPos;

	const PxU32 vertIdx = threadIdx.x + blockIdx.x * blockDim.x;

	const PxU32 nbVerts = shCloth.mNbVerts;

	if(vertIdx < nbVerts)
	{
		float4 delta = shCloth.mDeltaPos[vertIdx];

		if(delta.w != 0.f)
		{
			float4 accumDelta = accumulatedDeltaPos[vertIdx];

			const PxReal invDt = 1.0f / dt;
			float4 pos = curPositions[vertIdx];
			float4 vel = velocities[vertIdx];
			const float scale = 1.0f / delta.w;

			delta.x *= scale;
			delta.y *= scale;
			delta.z *= scale;
			delta.w = 0.f;

			vel += delta * invDt;
			pos += delta;
			accumDelta += delta;

			// Max velocity clamping
			const PxReal maxVel = shCloth.mMaxLinearVelocity;
			if (maxVel != PX_MAX_REAL)
			{
				const float4 prevPos = shCloth.mPrevPosition_InvMass[vertIdx];
				velocityClamping(pos, vel, accumDelta, maxVel, dt, prevPos);
			}

			curPositions[vertIdx] = pos;
			velocities[vertIdx] = vel;
			accumulatedDeltaPos[vertIdx] = accumDelta;

			// Clear delta
			shCloth.mDeltaPos[vertIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
		}
	}
}

extern "C" __global__
void cloth_applyExternalDeltasAndCheckClothCollisionValidityLaunch(
	PxgFEMCloth* femCloths,
	const PxU32* activeClothes,
	const PxReal dt,
	PxU8* updateClothContactPairs)
{
	__shared__ PxU8 hasInvalidContacts; // Shared flag to check invalid contacts

	if (threadIdx.x == 0)
	{
		hasInvalidContacts = 0;
	}

	__syncthreads();

	const PxU32 id = activeClothes[blockIdx.y];
	const PxgFEMCloth& femCloth = femCloths[id];

	float4* PX_RESTRICT curPositions = femCloth.mPosition_InvMass;
	float4* PX_RESTRICT velocities = femCloth.mVelocity_InvMass;
	float4* PX_RESTRICT accumulatedDeltaPos = femCloth.mAccumulatedDeltaPos;

	const float4* PX_RESTRICT const prevAccumulatedDeltaPos = femCloth.mPrevAccumulatedDeltaPos;

	const PxU32 vertIdx = threadIdx.x + blockIdx.x * blockDim.x;
	const PxU32 nbVerts = femCloth.mNbVerts;

	PxU8 invalidContact = 0; // Local flag for each thread
	if (vertIdx < nbVerts)
	{
		float4 delta = femCloth.mDeltaPos[vertIdx];

		// Apply deltas and clamp velocities
		if (delta.w != 0.f)
		{
			float4 accumDelta = accumulatedDeltaPos[vertIdx];

			const PxReal invDt = 1.0f / dt;
			float4 pos = curPositions[vertIdx];
			float4 vel = velocities[vertIdx];
			const float scale = 1.0f / delta.w;

			delta.x *= scale;
			delta.y *= scale;
			delta.z *= scale;
			delta.w = 0.f;

			vel += delta * invDt;
			pos += delta;
			accumDelta += delta;

			// Max velocity clamping
			const PxReal maxVel = femCloth.mMaxLinearVelocity;
			if (maxVel != PX_MAX_REAL)
			{
				const float4 prevPos = femCloth.mPrevPosition_InvMass[vertIdx];
				velocityClamping(pos, vel, accumDelta, maxVel, dt, prevPos);
			}

			curPositions[vertIdx] = pos;
			velocities[vertIdx] = vel;
			accumulatedDeltaPos[vertIdx] = accumDelta;

			// Clear delta
			femCloth.mDeltaPos[vertIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
		}

		// Check the validity of cloth contacts.
		const float4 curDelta = accumulatedDeltaPos[vertIdx];
		const float4 prevDelta = prevAccumulatedDeltaPos[vertIdx];

		if (prevDelta.w == 1.0f) // prevDelta is already set; process accordingly.
		{
			const PxReal deltaSq = PxLoad3(curDelta - prevDelta).magnitudeSquared();

			// Check if (curDelta - prevDelta).magnitude() > femCloth.mOriginalContactOffset.
			if(deltaSq > femCloth.mOriginalContactOffset * femCloth.mOriginalContactOffset) // A vertex has moved beyond the contact offset.
			{
				invalidContact = 1;
			}
		}
		else // prevDelta is unset; this is the first contact query of the simulation.
		{
			invalidContact = 1;
		}
	}

	__syncthreads();

	if (invalidContact == 1)
	{
		hasInvalidContacts = 1; // Set the shared flag to 1.
	}

	__syncthreads();

	if (threadIdx.x == 0 && hasInvalidContacts == 1)
	{
		*updateClothContactPairs = 1; // Contact pairs may be invalid and need updating.
	}
}

extern "C" __global__
void cloth_applyExternalDeltasWithVelocityClampingLaunch(
	PxgFEMCloth* femClothes,
	const PxU32* activeClothes,
	const PxReal dt)
{
	const PxU32 clothId = activeClothes[blockIdx.y];
	PxgFEMCloth& cloth = femClothes[clothId];

	const PxU32 nbVerts = cloth.mNbVerts;

	const PxU32 vertIdx = threadIdx.x + blockIdx.x * blockDim.x;

	if(vertIdx < nbVerts)
	{
		float4* PX_RESTRICT positions = cloth.mPosition_InvMass;
		float4* PX_RESTRICT velocities = cloth.mVelocity_InvMass;
		float4* PX_RESTRICT deltaPos = cloth.mDeltaPos;
		float4* PX_RESTRICT accumulatedDeltaPos = cloth.mAccumulatedDeltaPos;
		const PxReal maxVel = cloth.mMaxLinearVelocity;

		float4 pos = positions[vertIdx];
		float4 vel = velocities[vertIdx];
		float4 delta = deltaPos[vertIdx];
		float4 accumDelta = accumulatedDeltaPos[vertIdx];
		bool update = false;

		if(delta.w != 0.f)
		{
			const float scale = 1.0f / delta.w;
			const PxReal invDt = 1.f / dt;

			delta.x *= scale;
			delta.y *= scale;
			delta.z *= scale;
			delta.w = 0.f;

			vel += delta * invDt;
			pos += delta;
			accumDelta += delta;

			deltaPos[vertIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
			update = true;
		}

		if (maxVel != PX_MAX_REAL)
		{
			const float4 prevPos = cloth.mPrevPosition_InvMass[vertIdx];
			update = update || velocityClamping(pos, vel, accumDelta, maxVel, dt, prevPos);
		}

		if(update)
		{
			positions[vertIdx] = pos;
			velocities[vertIdx] = vel;
			accumulatedDeltaPos[vertIdx] = accumDelta;
		}
	}
}

extern "C" __global__
void cloth_updateClothContactValidityLaunch(
	PxgFEMCloth* PX_RESTRICT femCloths,
	const PxU32* activeIds,
	const PxU8* updateContactPairs,
	PxU32* clothContactCount)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	const PxU32 id = activeIds[blockIdx.y];
	const PxgFEMCloth& femCloth = femCloths[id];
	const PxU32 nbVerts = femCloth.mNbVerts;

	const float4* PX_RESTRICT const accumulatedDeltaPos = femCloth.mAccumulatedDeltaPos;
	float4* PX_RESTRICT prevAccumulatedDeltaPos = femCloth.mPrevAccumulatedDeltaPos;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	if (globalThreadIndex < nbVerts)
	{
		// Update prevDelta to curDelta. The difference (curDelta - prevDelta) indicates how much a vertex has
		// moved after the contact pairs were updated.
		float4 prevDelta = accumulatedDeltaPos[globalThreadIndex];
		prevDelta.w = 1.0f; // Mark prevDelta as set.
		prevAccumulatedDeltaPos[globalThreadIndex] = prevDelta;

		if (globalThreadIndex == 0 && blockIdx.y == 0)
		{
			// Reset the contact count.
			*clothContactCount = 0;
		}
	}
}

//solve soft body vs particle contacts, store positional change to cloth buffer
extern "C" __global__ 
void cloth_solveCPOutputClothDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgParticleSystem* particlesystems,
	PxgFemContactInfo* contactInfos,
	PxgFEMParticleConstraintBlock* constraints,
	PxU32* numContacts,
	float2* appliedForces, // output    
	PxsDeformableSurfaceMaterialData* materials)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];

		PxgFEMParticleConstraintBlock& constraint = constraints[workIndex / 32];

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		// first pairInd0 is particle
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if(invMass0 != 0)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(
			    particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 barycentric = constraint.barycentric[threadIndexInWarp];

			const PxU32 clothId = PxGetClothId(pairInd1);
			PxgFEMCloth& cloth = clothes[clothId];
			const PxU32 elementId = PxGetClothElementIndex(pairInd1);
			const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];

			float4 invMasses1;
			const float4 delta1 = computeTriangleContact(cloth.mAccumulatedDeltaPos, triVertId, barycentric, invMasses1);

			const PxReal dynamicFriction0 = psMat.friction;

			const PxU16 globalMaterialIndex = cloth.mMaterialIndices[elementId];
			const PxReal dynamicFriction1 = materials[globalMaterialIndex].dynamicFriction;
			const PxReal frictionCoefficient = (dynamicFriction0 + dynamicFriction1) * 0.5f;
			const PxVec3 linDelta0(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			const PxVec3 linDelta1(delta1.x, delta1.y, delta1.z);

			const PxVec3 delta = linDelta1 - linDelta0;

			const float4 normal_pen = constraint.normal_pen[threadIndexInWarp];
			const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

			float2 appliedForce = appliedForces[workIndex];

			const float normalDelta = delta.dot(normal);

			PxVec3 tanDir = delta - normal * normalDelta;
			const PxReal fricDelta = tanDir.normalize();

			const PxReal error = normal_pen.w + normalDelta;

			// KS - clamp the maximum force
			// Normal force can only be +ve!
			const float deltaF = PxMax(-appliedForce.x, -error * velMultiplier); // *relaxation;
			appliedForce.x += deltaF;

			const PxReal friction = appliedForce.x * frictionCoefficient;
			PxReal requiredForce = fricDelta * velMultiplier;

			// requiredForce is always positive!
			PxReal deltaFr = PxMin(requiredForce + appliedForce.y, friction) - appliedForce.y;
			appliedForce.y += deltaFr;

			PxVec3 deltaPos = ((normal * deltaF) - tanDir * deltaFr) * FEMCLOTH_BIAS_COEFFICIENT;

			// updateTetraPosDelta(invMasses0, barycentric0, tetrahedronId0, deltaPos, softbody0.mDelta);
			updateTrianglePosDelta(invMasses1, barycentric, triVertId, deltaPos, cloth.mDeltaPos, elementId);

			appliedForces[workIndex] = appliedForce;
		}
	}
}

//solve soft body vs particle contacts, store positional change to particle buffer
extern "C" __global__ 
void cloth_solveCPOutputParticleDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgParticleSystem* particlesystems,
	PxgFemContactInfo* contactInfos, 
	PxgFEMParticleConstraintBlock* constraints,
	PxU32* numContacts,
	float4* deltaP,        // output    
	float2* appliedForces, // output    
	PxsDeformableSurfaceMaterialData* materials
)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemContactInfo& contactInfo = contactInfos[workIndex];

		PxgFEMParticleConstraintBlock& constraint = constraints[workIndex / 32];

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if(invMass0 != 0)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(
			    particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 barycentric = constraint.barycentric[threadIndexInWarp];
			const PxU32 clothId = PxGetClothId(pairInd1);
			PxgFEMCloth& cloth = clothes[clothId];
			const PxU32 elementId = PxGetSoftBodyElementIndex(pairInd1);
			const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];

			float4 invMasses1;
			const float4 delta1 = computeTriangleContact(cloth.mAccumulatedDeltaPos, triVertId, barycentric, invMasses1);

			const PxReal dynamicFriction0 = psMat.friction;
			const PxU16 globalMaterialIndex = cloth.mMaterialIndices[elementId];
			const PxReal dynamicFriction1 = materials[globalMaterialIndex].dynamicFriction;
			const PxReal frictionCoefficient = (dynamicFriction0 + dynamicFriction1) * 0.5f;
			const PxVec3 linDelta0(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			const PxVec3 linDelta1(delta1.x, delta1.y, delta1.z);

			const PxVec3 delta = linDelta1 - linDelta0;

			const float4 normal_pen = constraint.normal_pen[threadIndexInWarp];
			const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

			float2 appliedForce = appliedForces[workIndex];

			const float normalDelta = delta.dot(normal);

			PxVec3 tanDir = delta - normal * normalDelta;
			const PxReal fricDelta = tanDir.normalize();

			const PxReal error = normal_pen.w + normalDelta;

			// KS - clamp the maximum force
			// Normal force can only be +ve!
			const float deltaF = PxMax(-appliedForce.x, -error * velMultiplier);
			appliedForce.x += deltaF;

			const PxReal friction = appliedForce.x * frictionCoefficient;
			PxReal requiredForce = fricDelta * velMultiplier;

			// requiredForce is always positive!
			PxReal deltaFr = PxMin(requiredForce + appliedForce.y, friction) - appliedForce.y;
			appliedForce.y += deltaFr;

			const PxVec3 deltaV = ((-normal * deltaF) + tanDir * deltaFr) * invMass0 * FEMCLOTH_BIAS_COEFFICIENT;
			PxReal w = 0.f;
			if(deltaF != 0.f || deltaFr != 0.f)
				w = 1.f;
			deltaP[workIndex] = make_float4(deltaV.x, deltaV.y, deltaV.z, w);
			appliedForces[workIndex] = appliedForce;
		}
	}
}

// solve rigid attachment, output rigid delta
extern "C" __global__ 
void
cloth_solveOutputAttachmentRigidDeltaVLaunch(
	PxgFEMCloth* clothes, 
	PxgFEMRigidAttachmentConstraint* attachments,
	const PxU32 numAttachments, 
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc, 
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, 
	const PxReal dt,
	float4* rigidDeltaVel // output
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
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		// printf("Hello!\n");

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];

		// nodeIndex
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);		

		//TODO - need to figure out how to make this work for articulation links!
		//Not writing rigidDeltaVel for statics, unlike for contacts
		if (rigidId.isStaticBody())
		{
			continue;
		}

		const float4 velMultiplierXYZ_invMassW = constraint.velMultiplierXYZ_invMassW[offset];
		if (velMultiplierXYZ_invMassW.w == 0.f)
		{
			rigidDeltaVel[workIndex] = make_float4(0.0f);
			rigidDeltaVel[workIndex + numAttachments] = make_float4(0.0f);
			continue;
		}

		const PxU32 clothId = PxGetClothId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];

		const float4* velocities = cloth.mVelocity_InvMass;

		float4 vel;
		if (elemIsVertex)
		{
			vel = velocities[elemIdx];
		}
		else
		{
			const float4 barycentric = constraint.baryOrType[offset];
			const uint4 triVertIdx = cloth.mTriangleVertexIndices[elemIdx];
			const float4 v0 = velocities[triVertIdx.x];
			const float4 v1 = velocities[triVertIdx.y];
			const float4 v2 = velocities[triVertIdx.z];
			vel = v0 * barycentric.x + v1 * barycentric.y + v2 * barycentric.z;
		}

		PxVec3 linVel1(vel.x, vel.y, vel.z);

		PxgVelocityPackPGS vel0;
		velocityReader.readVelocitiesPGS(rigidId, vel0);

		PxVec3 deltaLinVel, deltaAngVel;
		calculateAttachmentDeltaImpulsePGS(constraint.raXn0_biasW[offset], constraint.raXn0_biasW[offset], constraint.raXn0_biasW[offset], velMultiplierXYZ_invMassW,
			constraint.low_high_limits[offset], constraint.axis_angle[offset],
			vel0, linVel1, 1.f / dt, 0.5f, deltaLinVel, deltaAngVel);

		rigidDeltaVel[workIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, 1.f);
		rigidDeltaVel[workIndex + numAttachments] = make_float4(deltaAngVel.x, deltaAngVel.y, deltaAngVel.z, 0.f);
	}
}

//solve rigid attachment, output cloth delta
extern "C" __global__ 
void cloth_solveOutputAttachmentClothDeltaVLaunch(
	PxgFEMCloth* clothes, 
	PxgFEMRigidAttachmentConstraint* attachments,
	const PxU32 numAttachments,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc, 
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, 
	const PxReal dt)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, numSolverBodies);

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{

		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if (workIndex >= numAttachments)
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];
		const PxU32 clothId = PxGetClothId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];

		// Output for the cloth - just do atomics on this for now...
		float4* PX_RESTRICT accumDeltaV = reinterpret_cast<float4*>(cloth.mDeltaPos);

		const float4* velocities = cloth.mVelocity_InvMass;

		float4 vel;
		uint4 triVertIdx;
		float3 triInvMass;
		float4 barycentric;
		if (elemIsVertex)
		{
			vel = velocities[elemIdx];
		}
		else
		{
			barycentric = constraint.baryOrType[offset];
			triVertIdx = cloth.mTriangleVertexIndices[elemIdx];
			const float4 v0 = velocities[triVertIdx.x];
			const float4 v1 = velocities[triVertIdx.y];
			const float4 v2 = velocities[triVertIdx.z];
			vel = v0 * barycentric.x + v1 * barycentric.y + v2 * barycentric.z;
			triInvMass.x = v0.w;
			triInvMass.y = v1.w;
			triInvMass.z = v2.w;
		}

		PxVec3 linVel1(vel.x, vel.y, vel.z);
		const PxReal invMass1 = vel.w;

		// nodeIndex
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);

		if(rigidId.isStaticBody() && invMass1 == 0.f)
			continue; // constraining an infinte mass particle to an infinite mass rigid body. Won't work so skip!

		PxgVelocityPackPGS vel0;
		velocityReader.readVelocitiesPGS(rigidId, vel0);
		
		PxVec3 deltaLinVel, deltaAngVel;
		const PxVec3 deltaImpulse = calculateAttachmentDeltaImpulsePGS(offset, constraint, vel0, linVel1, 1.f / dt, 0.5f, deltaLinVel, deltaAngVel);

		if (!deltaImpulse.isZero())
		{
			//const PxVec3 deltaImpulse = (normal0 * deltaF0 + normal1 * deltaF1 + normal2 * deltaF2);
			const PxVec3 deltaPos = (-deltaImpulse) * dt;

			if (elemIsVertex)
			{
				AtomicAdd(accumDeltaV[elemIdx], deltaPos * vel.w, 1.f);
			}
			else
			{
				AtomicAdd(accumDeltaV[triVertIdx.x], deltaPos * barycentric.x * triInvMass.x, 1.f);
				AtomicAdd(accumDeltaV[triVertIdx.y], deltaPos * barycentric.y * triInvMass.y, 1.f);
				AtomicAdd(accumDeltaV[triVertIdx.z], deltaPos * barycentric.z * triInvMass.z, 1.f);
			}
		}
	}
}

//! 
//! \brief    : solve cloth vs. cloth attachment
//! 

extern "C" __global__ 
void cloth_solveOutputAttachmentClothClothDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgFEMFEMAttachmentConstraint* attachments,
	const PxU32 numAttachments)
{
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= numAttachments)
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;
		const PxgFEMFEMAttachmentConstraint& constraint = attachments[index];

		// cloth0
		const PxU32 elemId0 = constraint.elemId0[offset];
		const PxU32 clothId0 = PxGetClothId(elemId0);
		const PxU32 elementId0 = PxGetClothElementIndex(elemId0);

		PxgFEMCloth& cloth0 = clothes[clothId0];
		const float4 barycentric0 = constraint.barycentric0[offset];
		const float4* clothPos0 = cloth0.mPosition_InvMass;

		const uint4 triInd0 = cloth0.mTriangleVertexIndices[elementId0];
		const float4 p00 = clothPos0[triInd0.x];
		const float4 p01 = clothPos0[triInd0.y];
		const float4 p02 = clothPos0[triInd0.z];

		const float4 pos0 = p00 * barycentric0.x + p01 * barycentric0.y + p02 * barycentric0.z;
		const PxReal invMass0 = pos0.w;

		// cloth1
		const PxU32 elemId1 = constraint.elemId1[offset];
		const PxU32 clothId1 = elemId1 >> 20;
		const PxU32 elementId1 = (0x000fffff) & elemId1;

		PxgFEMCloth& cloth1 = clothes[clothId1];
		const float4 barycentric1 = constraint.barycentric1[offset];
		const float4* clothPos1 = cloth1.mPosition_InvMass;

		const uint4 triInd1 = cloth1.mTriangleVertexIndices[elementId1];
		const float4 p10 = clothPos1[triInd1.x];
		const float4 p11 = clothPos1[triInd1.y];
		const float4 p12 = clothPos1[triInd1.z];

		const float4 pos1 = p10 * barycentric1.x + p11 * barycentric1.y + p12 * barycentric1.z;
		const PxReal invMass1 = pos1.w;

		float4* PX_RESTRICT accumClothDeltaV0 = cloth0.mDeltaPos;
		float4* PX_RESTRICT accumClothDeltaV1 = cloth1.mDeltaPos;

		if(invMass1 == 0.f && invMass0 == 0.f)
			continue;

		const PxReal coefficient = .5f;
		const float4 diff = pos1 - pos0;
		const float4 deltaImpulse = diff * coefficient / (invMass0 + invMass1);

		const PxVec3 deltaImp = PxLoad3(deltaImpulse);
		const PxVec3 deltapos0 = deltaImp;
		const PxVec3 deltapos1 = -deltaImp;

		AtomicAdd(accumClothDeltaV0[triInd0.x], deltapos0 * barycentric0.x * p00.w, 1.f);
		AtomicAdd(accumClothDeltaV0[triInd0.y], deltapos0 * barycentric0.y * p01.w, 1.f);
		AtomicAdd(accumClothDeltaV0[triInd0.z], deltapos0 * barycentric0.z * p02.w, 1.f);

		AtomicAdd(accumClothDeltaV1[triInd1.x], deltapos1 * barycentric1.x * p10.w, 1.f);
		AtomicAdd(accumClothDeltaV1[triInd1.y], deltapos1 * barycentric1.y * p11.w, 1.f);
		AtomicAdd(accumClothDeltaV1[triInd1.z], deltapos1 * barycentric1.z * p12.w, 1.f);
	}
}

// solve rigid attachment, output rigid delta
extern "C" __global__ void cloth_solveOutputAttachmentRigidDeltaVTGSLaunch(
	PxgFEMCloth* clothes, 
	PxgFEMRigidAttachmentConstraint* attachments, 
	const PxU32 numAttachments,
	PxgPrePrepDesc* prePrepDesc, 
	PxgSolverCoreDesc* solverCoreDesc, 
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, 
	const PxReal dt,
	float4* rigidDeltaVel // output
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
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];

		// nodeIndex
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);

		// TODO - need to figure out how to make this work for articulation links!
		// Not writing rigidDeltaVel for statics, unlike for contacts
		if (rigidId.isStaticBody())
		{
			continue;
		}

		const float4 velMultiplierXYZ_invMassW = constraint.velMultiplierXYZ_invMassW[offset];
		if (velMultiplierXYZ_invMassW.w == 0.f)
		{
			rigidDeltaVel[workIndex] = make_float4(0.0f);
			rigidDeltaVel[workIndex + numAttachments] = make_float4(0.0f);
			continue;
		}

		const PxU32 clothId = PxGetClothId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];

		const float4* accumDelta = cloth.mAccumulatedDeltaPos;

		float4 delta;
		if (elemIsVertex)
		{
			delta = accumDelta[elemIdx];
		}
		else
		{
			const float4 barycentric = constraint.baryOrType[offset];
			const uint4 triVertIdx = cloth.mTriangleVertexIndices[elemIdx];
			const float4 d0 = accumDelta[triVertIdx.x];
			const float4 d1 = accumDelta[triVertIdx.y];
			const float4 d2 = accumDelta[triVertIdx.z];
			delta = d0 * barycentric.x + d1 * barycentric.y + d2 * barycentric.z;
		}

		PxVec3 linDelta1(delta.x, delta.y, delta.z);

		PxgVelocityPackTGS vel0;
		velocityReader.readVelocitiesTGS(rigidId, vel0);

		PxVec3 deltaLinVel, deltaAngVel;
		PxVec3 deltaImpulse = calculateAttachmentDeltaImpulseTGS(
		    offset, constraint, vel0, linDelta1, dt, FEMCLOTH_BIAS_COEFFICIENT, false, deltaLinVel, deltaAngVel);

		rigidDeltaVel[workIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, 1.f);
		rigidDeltaVel[workIndex + numAttachments] = make_float4(deltaAngVel.x, deltaAngVel.y, deltaAngVel.z, 0.f);
	}
}

// solve rigid attachment, output cloth delta
extern "C" __global__ void cloth_solveOutputAttachmentClothDeltaVTGSLaunch(
	PxgFEMCloth* clothes, 
	PxgFEMRigidAttachmentConstraint* attachments, 
	const PxU32 numAttachments,
	PxgPrePrepDesc* prePrepDesc, 
	PxgSolverCoreDesc* solverCoreDesc, 
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc, 
	const PxReal dt)
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
			return;

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];
		const PxU32 clothId = PxGetClothId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];

		// Output for the cloth - just do atomics on this for now...
		float4* PX_RESTRICT accumDeltaV = reinterpret_cast<float4*>(cloth.mDeltaPos);

		const float4* accumDelta = cloth.mAccumulatedDeltaPos;

		float4 delta;
		uint4 triVertIdx;
		float3 triInvMass;
		float4 barycentric;
		if (elemIsVertex)
		{
			delta = accumDelta[elemIdx];
		}
		else
		{
			barycentric = constraint.baryOrType[offset];
			triVertIdx = cloth.mTriangleVertexIndices[elemIdx];
			const float4 d0 = accumDelta[triVertIdx.x];
			const float4 d1 = accumDelta[triVertIdx.y];
			const float4 d2 = accumDelta[triVertIdx.z];
			delta = d0 * barycentric.x + d1 * barycentric.y + d2 * barycentric.z;
			triInvMass.x = d0.w;
			triInvMass.y = d1.w;
			triInvMass.z = d2.w;
		}

		PxVec3 linDelta1(delta.x, delta.y, delta.z);

		const PxReal invMass1 = delta.w;

		// nodeIndex
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);

		if(rigidId.isStaticBody() && invMass1 == 0.f)
			continue; // constraining an infinte mass particle to an infinite mass rigid body. Won't work so skip!

		const PxVec3 normal0(1.f, 0.f, 0.f);
		const PxVec3 normal1(0.f, 1.f, 0.f);
		const PxVec3 normal2(0.f, 0.f, 1.f);

		PxgVelocityPackTGS vel0;
		velocityReader.readVelocitiesTGS(rigidId, vel0);

		const PxReal biasCoefficient = 0.7f; // temporary bias number
		PxVec3 deltaLinVel, deltaAngVel;
		PxVec3 deltaImpulse = calculateAttachmentDeltaImpulseTGS(offset, constraint, vel0, linDelta1, dt,
		                                                         biasCoefficient, false, deltaLinVel, deltaAngVel);

		if(!deltaImpulse.isZero())
		{
			//const PxVec3 deltaImpulse = (normal0 * deltaF0 + normal1 * deltaF1 + normal2 * deltaF2);
			const PxVec3 deltaPos = (-deltaImpulse);

			if (elemIsVertex)
			{
				AtomicAdd(accumDeltaV[elemIdx], deltaPos * delta.w, 1.f);
			}
			else
			{
				AtomicAdd(accumDeltaV[triVertIdx.x], deltaPos * barycentric.x * triInvMass.x, 1.f);
				AtomicAdd(accumDeltaV[triVertIdx.y], deltaPos * barycentric.y * triInvMass.y, 1.f);
				AtomicAdd(accumDeltaV[triVertIdx.z], deltaPos * barycentric.z * triInvMass.z, 1.f);
			}
		}
	}
}