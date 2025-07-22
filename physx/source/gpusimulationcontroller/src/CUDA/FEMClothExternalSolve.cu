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

// General constraints applicable to both vertex-triangle and edge-edge collisions.
__device__ bool triangleCollisionConstraint(PxVec3& dx0, PxVec3& dx1, PxVec3& dx2, PxVec3& dx3, const PxVec3& disp, const PxVec3& normal,
											const PxVec3& relDelta, const PxVec4& invMass, PxReal restDistance, PxReal frictionCoefficient,
											const PxVec4& weights)
{
	const PxReal CN = disp.dot(normal) - restDistance;

	const PxReal denom = invMass.x * weights.x * weights.x + invMass.y * weights.y * weights.y + invMass.z * weights.z * weights.z +
						 invMass.w * weights.w * weights.w;
	if(denom < FEMCLOTH_THRESHOLD)
	{
		return false;
	}

	const PxReal invDenom = 1.f / denom;
	const PxReal deltaLambdaN = -CN * invDenom;

	PxVec3 commonVec = deltaLambdaN * normal;

	// Friction constraint in the tangent direction.
	// The friction constraint is computed after the collision constraint.
	PxVec3 tanDir = (relDelta - relDelta.dot(normal) * normal);
	const PxReal tanMagSq = tanDir.magnitudeSquared();

	if(frictionCoefficient != 0.0f && tanMagSq > FEMCLOTH_THRESHOLD)
	{
		const PxReal invTanMag = PxRecipSqrt(tanMagSq);
		tanDir *= invTanMag;
		PxReal CT = relDelta.dot(tanDir);

		PxReal deltaLambdaT = PxMax(0.f, CT * invDenom); // (-) sign is added in the next line
		deltaLambdaT = -PxMin(deltaLambdaT, frictionCoefficient * PxAbs(deltaLambdaN));

		assert(deltaLambdaT <= 0.f);

		commonVec += deltaLambdaT * tanDir;
	}

	dx0 = invMass.x * weights.x * commonVec;
	dx1 = invMass.y * weights.y * commonVec;
	dx2 = invMass.z * weights.z * commonVec;
	dx3 = invMass.w * weights.w * commonVec;

	return true;
}

template <typename IterativeData>
static __device__ void queryRigidClothContactReferenceCount(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	const PxReal dt,
	PxReal* appliedForces,
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

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		const bool wasActive = contactInfo.isInCollision();
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxgFemRigidConstraintBlock& constraint = constraints[workIndex / 32];
		const float4 fricTan0_invMass0 = constraint.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = constraint.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: cloth (both triangle and vertex)
		const PxU32 clothId = PxGetClothId(pairInd1);
		const PxU32 elementId = PxGetClothElementIndex(pairInd1);

		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId < PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			const bool checkOnlyActivity = true; // Check only if the constraint is active, without evaluating forces/impulses.
			bool isActive = false;

			PxgFEMCloth& cloth = femClothes[clothId];

			FEMCollision<PxVec3> femCollision;
			femCollision.isTGS = isTGS;

			const int globalRigidBodyId = femCollision.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies);

			// For vertex collision, only x component of the barycentric coordinates is used (1.0).
			const PxVec3 bcVec = (bc.w == 0) ? PxVec3(bc.x, bc.y, bc.z) : PxVec3(1.0f, 0.0f, 0.0f);
			const PxVec3 deformableInvMasses = femCollision.readCloth(cloth, elementId, bc, NULL, checkOnlyActivity);

			if(wasActive)
			{
				// Once activated, keep the contact pair active to maintain a conservative reference count.
				isActive = true;
			}
			else
			{
				femCollision.readRigidBody(rigidId, globalRigidBodyId, fricTan0_invMass0.w, NULL, NULL);
				isActive = femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId, velocityReader, dt,
												   bcVec, wasActive, checkOnlyActivity);

				if(isActive)
				{
					contactInfo.markInCollision(true);
				}
			}

			if(isActive)
			{
				// Update cloth
				{
					if(bc.w == 0.f) // Cloth triangle
					{
						// Increment the reference count of the three vertices.
						const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
						if(bc.x > 1e-3f && deformableInvMasses.x != 0.0f)
						{
							atomicAdd(&cloth.mDeltaPos[triVertId.x].w, 1.0f);
						}
						if(bc.y > 1e-3f && deformableInvMasses.y != 0.0f)
						{
							atomicAdd(&cloth.mDeltaPos[triVertId.y].w, 1.0f);
						}
						if(bc.z > 1e-3f && deformableInvMasses.z != 0.0f)
						{
							atomicAdd(&cloth.mDeltaPos[triVertId.z].w, 1.0f);
						}
					}
					else // Cloth vertex
					{
						// Increment the reference count of the single vertex.
						if(deformableInvMasses.x != 0.0f)
						{
							atomicAdd(&cloth.mDeltaPos[elementId].w, 1.0f);
						}
					}
				}

				// Update rigidbody
				if(globalRigidBodyId != -1 && fricTan0_invMass0.w != 0.0f)
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
	PxgFemOtherContactInfo * contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc,
	const PxReal dt,
	PxReal* appliedForces,
	PxU32* rigidBodyReferenceCounts)
{
	const bool isTGS = false;
	queryRigidClothContactReferenceCount(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
		sharedDesc, dt, appliedForces, rigidBodyReferenceCounts, isTGS);
}

template <typename IterativeData>
static __device__ void solveRigidClothContact(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeData>* sharedDesc,
	const PxReal dt,
	float4* rigidDeltaVel,
	PxReal* appliedForces, // output
	PxU32* rigidBodyReferenceCounts,
	PxsDeformableSurfaceMaterialData* materials,
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

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		const bool isActive = contactInfo.isInCollision();
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxgFemRigidConstraintBlock& constraint = constraints[workIndex / 32];
		const float4 fricTan0_invMass0 = constraint.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = constraint.barycentric[threadIndexInWarp];

		// First actor: rigid body
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second actor: cloth (both triangle and vertex)
		const PxU32 clothId = PxGetClothId(pairInd1);
		const PxU32 elementId = PxGetClothElementIndex(pairInd1);

		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId < PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			const bool checkOnlyActivity = false;

			FEMCollision<PxVec3> femCollision;
			femCollision.isTGS = isTGS;

			const int globalRigidBodyId = femCollision.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies);
			PxVec3 deltaLinVel0(0.0f), deltaAngVel0(0.0f);
			PxReal count = 0.0f;

			if(isActive)
			{
				PxgFEMCloth& cloth = femClothes[clothId];

				// For vertex collision, only x component of the barycentric coordinates is used (1.0).
				const PxVec3 bcVec = (bc.w == 0) ? PxVec3(bc.x, bc.y, bc.z) : PxVec3(1.0f, 0.0f, 0.0f);

				femCollision.readRigidBody(rigidId, globalRigidBodyId, fricTan0_invMass0.w, rigidBodyReferenceCounts,
										   &rigidBodyMaterials[contactInfo.getRigidMaterialIndex()]);
				femCollision.readCloth(cloth, elementId, bc, materials, checkOnlyActivity);

				femCollision.initialize(fricTan0_invMass0, constraint, appliedForces[workIndex], rigidId, velocityReader, dt, bcVec,
										isActive, checkOnlyActivity);
				
				// Compute cloth delta
				PxVec3 deltaPos;
				appliedForces[workIndex] = femCollision.computeFEMChange(deltaPos, dt);

				// Update cloth
				femCollision.writeCloth(cloth, elementId, bc, deltaPos);

				// Compute rigid body delta
				if(fricTan0_invMass0.w != 0.0f && !rigidId.isStaticBody())
				{
					count = 1.0f;
					femCollision.computeRigidChange(deltaLinVel0, deltaAngVel0, rigidId, fricTan0_invMass0.w);
				}
			}

			// Update rigid body
			femCollision.writeRigidBody(rigidDeltaVel, deltaLinVel0, deltaAngVel0, workIndex, workIndex + tNumContacts, count);
		}
	}
}

// solve collision between cloth and primitives based on the sorted contact by rigid id
// store new velocity to rigid body buffer
extern "C" __global__ 
void cloth_solveRigidClothCollisionLaunch(
	PxgFEMCloth* femClothes, 
	PxgFemOtherContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts, 
	PxgPrePrepDesc* prePrepDesc, 
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc, 
	PxgSolverSharedDesc<IterativeSolveData>* sharedDesc,
	float4* rigidDeltaVel, // output
	PxReal* appliedForces, 
	PxU32* rigidBodyReferenceCounts,
	const PxReal dt,
	PxsDeformableSurfaceMaterialData* materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool isTGS = false;
	solveRigidClothContact(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, dt,
						   rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, materials, rigidBodyMaterials, isTGS);
}

extern "C" __global__
void cloth_queryRigidClothContactReferenceCountTGSLaunch(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	const PxReal dt,
	PxReal* appliedForces,
	PxU32* rigidBodyReferenceCounts)
{
	const bool isTGS = true;
	queryRigidClothContactReferenceCount(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
										 sharedDesc, dt, appliedForces, rigidBodyReferenceCounts, isTGS);
}

// solve collision between cloth and primitives based on the sorted contact by rigid id
// store new velocity to rigid body buffer
extern "C" __global__ void cloth_solveRigidClothCollisionTGSLaunch(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgFemRigidConstraintBlock* constraints,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	PxgSolverSharedDesc<IterativeSolveDataTGS>* sharedDesc,
	float4* rigidDeltaVel, // output
	PxReal* appliedForces, //output
	PxU32* rigidBodyReferenceCounts,
	const PxReal dt,
	PxsDeformableSurfaceMaterialData* materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials)
{
	const bool isTGS = true;
	solveRigidClothContact(femClothes, contactInfos, constraints, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, sharedDesc, dt,
						   rigidDeltaVel, appliedForces, rigidBodyReferenceCounts, materials, rigidBodyMaterials, isTGS);
}



//! 
//! \brief    : solve cloth vs. cloth collision
//! 

PX_FORCE_INLINE __device__ void applyContactDelta(const PxVec3& disp, const PxVec3& normal, const PxVec4& invMasses, const PxVec4& weights,
												  const PxVec4T<PxU32>& vertIndices, PxgFEMCloth* cloth0, PxgFEMCloth* cloth1,
												  PxgFEMCloth* cloth0or1, const PxVec3& relDelta)
{
	PxVec3 dx0(0.0f), dx1(0.0f), dx2(0.0f), dx3(0.0f);

	const PxReal count0 = cloth0->mDeltaPos[vertIndices[0]].w;
	const PxReal count1 = cloth0or1->mDeltaPos[vertIndices[1]].w;
	const PxReal count2 = cloth1->mDeltaPos[vertIndices[2]].w;
	const PxReal count3 = cloth1->mDeltaPos[vertIndices[3]].w;

	const PxVec4 splittedInvMasses(count0 * invMasses[0], count1 * invMasses[1], count2 * invMasses[2], count3 * invMasses[3]);

	const PxReal restDist = cloth0->mRestDistance + cloth1->mRestDistance;
	const PxReal frictionCoefficient = 0.25f * (cloth0->mDynamicFrictions[vertIndices[0]] + cloth0or1->mDynamicFrictions[vertIndices[1]] +
												cloth1->mDynamicFrictions[vertIndices[2]] + cloth1->mDynamicFrictions[vertIndices[3]]);

	triangleCollisionConstraint(dx0, dx1, dx2, dx3, disp, normal, relDelta, splittedInvMasses, restDist, frictionCoefficient, weights);

	if(count0 > 0.0f)
	{
		atomicAdd(&cloth0->mDeltaPos[vertIndices[0]].x, dx0.x);
		atomicAdd(&cloth0->mDeltaPos[vertIndices[0]].y, dx0.y);
		atomicAdd(&cloth0->mDeltaPos[vertIndices[0]].z, dx0.z);
	}
	if(count1 > 0.0f)
	{
		atomicAdd(&cloth0or1->mDeltaPos[vertIndices[1]].x, dx1.x);
		atomicAdd(&cloth0or1->mDeltaPos[vertIndices[1]].y, dx1.y);
		atomicAdd(&cloth0or1->mDeltaPos[vertIndices[1]].z, dx1.z);
	}
	if(count2 > 0.0f)
	{
		atomicAdd(&cloth1->mDeltaPos[vertIndices[2]].x, dx2.x);
		atomicAdd(&cloth1->mDeltaPos[vertIndices[2]].y, dx2.y);
		atomicAdd(&cloth1->mDeltaPos[vertIndices[2]].z, dx2.z);
	}
	if(count3 > 0.0f)
	{
		atomicAdd(&cloth1->mDeltaPos[vertIndices[3]].x, dx3.x);
		atomicAdd(&cloth1->mDeltaPos[vertIndices[3]].y, dx3.y);
		atomicAdd(&cloth1->mDeltaPos[vertIndices[3]].z, dx3.z);
	}
}

extern "C" __global__ __launch_bounds__(256, 4) 
void cloth_solveClothClothDeltaVTLaunch(
    PxgFEMCloth* cloths, 
	PxgFemFemContactInfo* contactInfos, 
	PxU32* numContacts,
	PxReal dt)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + threadIdx.x + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= tNumContacts)
		{
			return;
		}

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		if(!contactInfo.isInCollision())
		{
			continue;
		}

		const PxU32 pairInd0 = static_cast<PxU32>(contactInfo.pairInd0);
		const PxU32 pairInd1 = contactInfo.pairInd1;

		const bool isEdgeEdgePair = contactInfo.isEdgeEdgePair();
		PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
		PxU32 e1_localIndex0 = contactInfo.getAuxInd1();

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		PxVec3 dx0(0.0f), dx1(0.0f), dx2(0.0f), dx3(0.0f);

		const uint4 triVertId1 = cloth1->mTriangleVertexIndices[elementId1];
		const PxVec4T<PxU32> vertIndices(elementId0, triVertId1.x, triVertId1.y, triVertId1.z);

		const float4 x0 = cloth0->mPosition_InvMass[vertIndices[0]];
		const float4 x1 = cloth1->mPosition_InvMass[vertIndices[1]];
		const float4 x2 = cloth1->mPosition_InvMass[vertIndices[2]];
		const float4 x3 = cloth1->mPosition_InvMass[vertIndices[3]];

		const float4 prevX0 = cloth0->mPrevPositionInContactOffset[vertIndices[0]];
		const float4 prevX1 = cloth1->mPrevPositionInContactOffset[vertIndices[1]];
		const float4 prevX2 = cloth1->mPrevPositionInContactOffset[vertIndices[2]];
		const float4 prevX3 = cloth1->mPrevPositionInContactOffset[vertIndices[3]];

		PxVec4 invMasses;
		const PxVec3 xx0 = PxLoad3(x0, invMasses[0]);
		const PxVec3 xx1 = PxLoad3(x1, invMasses[1]);
		const PxVec3 xx2 = PxLoad3(x2, invMasses[2]);
		const PxVec3 xx3 = PxLoad3(x3, invMasses[3]);

		PxReal r, s, t; // Barycentric coordinates
		const PxVec3 closestPt = closestPtPointTriangle(xx0, xx1, xx2, xx3, s, t);
		r = 1.0f - s - t;
		const PxVec4 weights(1.0f, -r, -s, -t); // cloth0 - cloth1

		const PxVec3 disp = weights[0] * xx0 + weights[1] * xx1 + weights[2] * xx2 + weights[3] * xx3;
		const PxReal dispSq = disp.magnitudeSquared();

		PxVec3 normal;
		if(dispSq < FEMCLOTH_THRESHOLD)
		{
			// Vertex is nearly on the triangle, use triangle normal
			const PxVec3 ab = xx2 - xx1;
			const PxVec3 ac = xx3 - xx1;
			normal = ab.cross(ac);

			const PxReal nLenSq = normal.magnitudeSquared();
			if(nLenSq < FEMCLOTH_THRESHOLD)
			{
				// Degenerate triangle. Skip this contact
				continue;
			}

			normal *= (1.0f / PxSqrt(nLenSq));

			// Flip if pointing inward
			PxVec3 toVertex = xx0 - xx1;
			if(normal.dot(toVertex) < 0.0f)
			{
				normal = -normal;
			}
		}
		else
		{
			// Use normalized direction
			const PxReal dist = PxSqrt(dispSq);
			normal = disp / dist;
		}

		const float4 relDelta4 =
			weights[0] * (x0 - prevX0) + weights[1] * (x1 - prevX1) + weights[2] * (x2 - prevX2) + weights[3] * (x3 - prevX3);
		const PxVec3 relDelta(relDelta4.x, relDelta4.y, relDelta4.z);

		applyContactDelta(disp, normal, invMasses, weights, vertIndices, cloth0, cloth1, cloth1, relDelta);
	}
}

extern "C" __global__ __launch_bounds__(256, 4) 
void cloth_solveClothClothDeltaEELaunch(
    PxgFEMCloth* cloths, 
	PxgFemFemContactInfo* contactInfos, 
	PxU32* numContacts,
	PxReal dt)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + threadIdx.x + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= tNumContacts)
		{
			return;
		}

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		if(!contactInfo.isInCollision())
		{
			continue;
		}

		const PxU32 pairInd0 = static_cast<PxU32>(contactInfo.pairInd0);
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
		PxU32 e1_localIndex0 = contactInfo.getAuxInd1();

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		PxVec3 dx0(0.0f), dx1(0.0f), dx2(0.0f), dx3(0.0f);

		PxVec4T<PxU32> vertIndices;
		PxVec4 invMasses;
		PxVec3 xx0, xx1, xx2, xx3;

		// Edge0
		PxU32 e0_localIndex1 = (e0_localIndex0 + 1) % 3;

		const uint4 triVertInd0 = cloth0->mTriangleVertexIndices[elementId0];
		const PxU32* vertexIndices0 = reinterpret_cast<const PxU32*>(&triVertInd0);

		vertIndices[0] = vertexIndices0[e0_localIndex0];
		vertIndices[1] = vertexIndices0[e0_localIndex1];

		// Edge1
		PxU32 e1_localIndex1 = (e1_localIndex0 + 1) % 3;
		const uint4 triVertInd1 = cloth1->mTriangleVertexIndices[elementId1];
		const PxU32* vertexIndices1 = reinterpret_cast<const PxU32*>(&triVertInd1);

		vertIndices[2] = vertexIndices1[e1_localIndex0];
		vertIndices[3] = vertexIndices1[e1_localIndex1];

		const float4 x0 = cloth0->mPosition_InvMass[vertIndices[0]];
		const float4 x1 = cloth0->mPosition_InvMass[vertIndices[1]];
		const float4 x2 = cloth1->mPosition_InvMass[vertIndices[2]];
		const float4 x3 = cloth1->mPosition_InvMass[vertIndices[3]];

		const float4 prevX0 = cloth0->mPrevPositionInContactOffset[vertIndices[0]];
		const float4 prevX1 = cloth0->mPrevPositionInContactOffset[vertIndices[1]];
		const float4 prevX2 = cloth1->mPrevPositionInContactOffset[vertIndices[2]];
		const float4 prevX3 = cloth1->mPrevPositionInContactOffset[vertIndices[3]];

		xx0 = PxLoad3(x0, invMasses[0]);
		xx1 = PxLoad3(x1, invMasses[1]);
		xx2 = PxLoad3(x2, invMasses[2]);
		xx3 = PxLoad3(x3, invMasses[3]);

		// Vertex-edge collisions are handled in vertex-triangle collisions.
		PxReal s, t;
		PxReal distSq;
		const bool isValid = closestPtLineLine(xx0, xx1, xx2, xx3, s, t, distSq);

		PxVec4 weights(1.0f - s, s, -(1.0f - t), -t); // cloth0 - cloth1
		const PxVec3 disp = weights[0] * xx0 + weights[1] * xx1 + weights[2] * xx2 + weights[3] * xx3;
		const PxReal dist = PxSqrt(distSq);

		const PxVec3 normal = disp * (1.0f / dist);

		if(!isValid) // Degenerate cases, re-adjust the weights, so the contact constraint is applied at the mid points.
		{
			weights = PxVec4(0.5f, 0.5f, -0.5f, -0.5f);
		}

		const float4 relDelta4 =
			weights[0] * (x0 - prevX0) + weights[1] * (x1 - prevX1) + weights[2] * (x2 - prevX2) + weights[3] * (x3 - prevX3);
		const PxVec3 relDelta(relDelta4.x, relDelta4.y, relDelta4.z);

		applyContactDelta(disp, normal, invMasses, weights, vertIndices, cloth0, cloth1, cloth0, relDelta);
	}
}

PX_FORCE_INLINE __device__ void countContact(const PxVec4& coeffs, const PxVec3& xx0, const PxVec3& xx1, const PxVec3& xx2,
											 const PxVec3& xx3, PxgFemFemContactInfo& info, PxgFEMCloth* cloth0, PxgFEMCloth* cloth1,
											 PxgFEMCloth* cloth0or1, const PxVec4T<PxU32>& vertIndices)
{
	const PxVec3 disp = coeffs[0] * xx0 + coeffs[1] * xx1 + coeffs[2] * xx2 + coeffs[3] * xx3;
	const PxReal distSq = disp.magnitudeSquared();
	const PxReal restDist = cloth0->mRestDistance + cloth1->mRestDistance;

	// Mark collision state to skip impulse computation for non-colliding contacts.
	if(distSq > restDist * restDist)
	{
		info.markInCollision(false);
		return;
	}

	// Count how many times each vertex is referenced.
	const PxReal coeff0 = PxAbs(coeffs[0]);
	const PxReal coeff1 = PxAbs(coeffs[1]);
	const PxReal coeff2 = PxAbs(coeffs[2]);
	const PxReal coeff3 = PxAbs(coeffs[3]);

	if(coeff0 > 1e-3f)
	{
		atomicAdd(&cloth0->mDeltaPos[vertIndices[0]].w, coeff0);
	}
	if(coeff1 > 1e-3f)
	{
		atomicAdd(&cloth0or1->mDeltaPos[vertIndices[1]].w, coeff1);
	}
	if(coeff2 > 1e-3f)
	{
		atomicAdd(&cloth1->mDeltaPos[vertIndices[2]].w, coeff2);
	}
	if(coeff3 > 1e-3f)
	{
		atomicAdd(&cloth1->mDeltaPos[vertIndices[3]].w, coeff3);
	}

	info.markInCollision(true);
}

extern "C" __global__ __launch_bounds__(256, 4)
void cloth_queryClothClothContactVTCountLaunch(
	PxgFEMCloth* cloths,
	PxgFemFemContactInfo* contactInfos,
	PxU32* numContacts)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + threadIdx.x + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= tNumContacts)
		{
			return;
		}

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		if(!contactInfo.isValidPair())
		{
			contactInfo.markInCollision(false);
			continue;
		}

		const PxU32 pairInd0 = static_cast<PxU32>(contactInfo.pairInd0);
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
		PxU32 e1_localIndex0 = contactInfo.getAuxInd1();

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		const uint4 triVertId1 = cloth1->mTriangleVertexIndices[elementId1];
		PxVec4T<PxU32> vertIndices(elementId0, triVertId1.x, triVertId1.y, triVertId1.z);

		const float4 x0 = cloth0->mPosition_InvMass[vertIndices[0]];
		const float4 x1 = cloth1->mPosition_InvMass[vertIndices[1]];
		const float4 x2 = cloth1->mPosition_InvMass[vertIndices[2]];
		const float4 x3 = cloth1->mPosition_InvMass[vertIndices[3]];

		const PxVec3 xx0 = PxLoad3(x0);
		const PxVec3 xx1 = PxLoad3(x1);
		const PxVec3 xx2 = PxLoad3(x2);
		const PxVec3 xx3 = PxLoad3(x3);

		PxReal r, s, t; // Barycentric coordinates
		const PxVec3 closestPt = closestPtPointTriangle(xx0, xx1, xx2, xx3, s, t);
		r = 1.0f - s - t;
		const PxVec4 weights(1.0f, -r, -s, -t); // cloth0 - cloth1

		countContact(weights, xx0, xx1, xx2, xx3, contactInfo, cloth0, cloth1, cloth1, vertIndices);
	}
}

extern "C" __global__ __launch_bounds__(256, 4)
void cloth_queryClothClothContactEECountLaunch(
	PxgFEMCloth* cloths,
	PxgFemFemContactInfo* contactInfos,
	PxU32* numContacts)
{
	const PxU32 tNumContacts = *numContacts;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + threadIdx.x + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= tNumContacts)
		{
			return;
		}

		PxgFemFemContactInfo& contactInfo = contactInfos[workIndex];
		if(!contactInfo.isValidPair())
		{
			contactInfo.markInCollision(false);
			continue;
		}

		const PxU32 pairInd0 = static_cast<PxU32>(contactInfo.pairInd0);
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
		PxU32 e1_localIndex0 = contactInfo.getAuxInd1();

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		PxVec4T<PxU32> vertIndices;

		// Edge0
		PxU32 e0_localIndex1 = (e0_localIndex0 + 1) % 3;
		const uint4 triVertInd0 = cloth0->mTriangleVertexIndices[elementId0];
		const PxU32* vertexIndices0 = reinterpret_cast<const PxU32*>(&triVertInd0);

		vertIndices[0] = vertexIndices0[e0_localIndex0];
		vertIndices[1] = vertexIndices0[e0_localIndex1];

		// Edge1
		PxU32 e1_localIndex1 = (e1_localIndex0 + 1) % 3;
		const uint4 triVertInd1 = cloth1->mTriangleVertexIndices[elementId1];
		const PxU32* vertexIndices1 = reinterpret_cast<const PxU32*>(&triVertInd1);

		vertIndices[2] = vertexIndices1[e1_localIndex0];
		vertIndices[3] = vertexIndices1[e1_localIndex1];

		const float4 x0 = cloth0->mPosition_InvMass[vertIndices[0]];
		const float4 x1 = cloth0->mPosition_InvMass[vertIndices[1]];
		const float4 x2 = cloth1->mPosition_InvMass[vertIndices[2]];
		const float4 x3 = cloth1->mPosition_InvMass[vertIndices[3]];

		const PxVec3 xx0 = PxLoad3(x0);
		const PxVec3 xx1 = PxLoad3(x1);
		const PxVec3 xx2 = PxLoad3(x2);
		const PxVec3 xx3 = PxLoad3(x3);

		// Vertex-edge collisions are handled in vertex-triangle collisions.
		PxReal s, t;
		PxReal distSq;
		closestPtLineLine(xx0, xx1, xx2, xx3, s, t, distSq);
		const bool isOutside = ((s < DEFORMABLE_BARYCENTRIC_THRESHOLD) || (s > DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD) ||
								(t < DEFORMABLE_BARYCENTRIC_THRESHOLD) || (t > DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD));

		if(isOutside)
		{
			contactInfo.markInCollision(false);
			continue;
		}

		const PxVec4 weights(1.0f - s, s, -(1.0f - t), -t); // cloth0 - cloth1

		countContact(weights, xx0, xx1, xx2, xx3, contactInfo, cloth0, cloth1, cloth0, vertIndices);
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

			curPositions[vertIdx] = pos;
			velocities[vertIdx] = vel;
			accumulatedDeltaPos[vertIdx] = accumDelta;

			// Clear delta
			shCloth.mDeltaPos[vertIdx] = make_float4(0.f, 0.f, 0.f, 0.f);
		}
	}
}

extern "C" __global__
void cloth_updateClothContactValidityLaunch(
	PxgFEMCloth* PX_RESTRICT femCloths,
	const PxU32* activeIds,
	bool adaptiveCollisionPairUpdate,
	bool forceUpdateClothContactPairs,
	const PxU8* updateContactPairs,
	PxU32* VTContactCount,
	PxU32* EEContactCount,
	PxReal dt)
{
	const PxU32 id = activeIds[blockIdx.y];
	const PxgFEMCloth& femCloth = femCloths[id];
	const PxU32 nbVerts = femCloth.mNbVerts;

	const float4* const PX_RESTRICT curPositions = femCloth.mPosition_InvMass;
	float4* PX_RESTRICT prevPositionsInContactOffset = femCloth.mPrevPositionInContactOffset;

	const PxU32 globalThreadIndex = threadIdx.x + blockDim.x * blockIdx.x;

	const bool updateAdaptiveCollisionPair = adaptiveCollisionPairUpdate && (*updateContactPairs == 1);

	if(updateAdaptiveCollisionPair && globalThreadIndex == 0 && blockIdx.y == 0)
	{
		// Reset the contact count.
		*VTContactCount = 0;
		*EEContactCount = 0;
	}

	if(globalThreadIndex < nbVerts)
	{
		float4 curPos = curPositions[globalThreadIndex];
		if(updateAdaptiveCollisionPair)
		{
			curPos.w = 1.0f; // Mark prevPosInContactOffset as set.

			prevPositionsInContactOffset[globalThreadIndex] = curPos; // Update prevPositionInContactOffset to current position. The
																	  // difference indicates how much a vertex has moved after the contact
																	  // pairs were updated.
		}
	}
}

//solve soft body vs particle contacts, store positional change to cloth buffer
extern "C" __global__ 
void cloth_solveCPOutputClothDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgParticleSystem* particlesystems,
	PxgFemOtherContactInfo* contactInfos,
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

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];

		PxgFEMParticleConstraintBlock& constraint = constraints[workIndex / 32];

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		// first pairInd0 is particle
		const PxU32 particleSystemId = PxGetParticleSystemId(contactInfo.pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(contactInfo.pairInd0);

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
	PxgFemOtherContactInfo* contactInfos, 
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

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];

		PxgFEMParticleConstraintBlock& constraint = constraints[workIndex / 32];

		const PxReal velMultiplier = constraint.velMultiplier[threadIndexInWarp];

		const PxU32 particleSystemId = PxGetParticleSystemId(contactInfo.pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(contactInfo.pairInd0);

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

// solve rigid attachment
extern "C" __global__ 
void cloth_solveRigidClothAttachmentLaunch(
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
		{
			return;
		}

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];

		// Cloth info
		const PxU32 clothId = PxGetClothId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];
		const float4* clothVelocities = cloth.mVelocity_InvMass;

		float4 clothVel;
		uint4 triVertIdx;
		float3 triInvMass;
		float4 barycentric;

		if(elemIsVertex)
		{
			clothVel = clothVelocities[elemIdx];
		}
		else
		{
			barycentric = constraint.baryOrType[offset];
			triVertIdx = cloth.mTriangleVertexIndices[elemIdx];
			const float4 v0 = clothVelocities[triVertIdx.x];
			const float4 v1 = clothVelocities[triVertIdx.y];
			const float4 v2 = clothVelocities[triVertIdx.z];
			clothVel = v0 * barycentric.x + v1 * barycentric.y + v2 * barycentric.z;
			
			triInvMass.x = v0.w;
			triInvMass.y = v1.w;
			triInvMass.z = v2.w;
		}

		PxVec3 linVel1(clothVel.x, clothVel.y, clothVel.z);

		// Rigid body info
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);
		if(rigidId.isStaticBody() && clothVel.w == 0.0f)
		{
			continue;
		}

		PxgVelocityPackPGS vel0;
		velocityReader.readVelocitiesPGS(rigidId, vel0);

		// Compute impulses
		const float4 velMultiplierXYZ_invMassW = constraint.velMultiplierXYZ_invMassW[offset];
		PxVec3 deltaLinVel, deltaAngVel;
		const PxVec3 deltaImpulse = calculateAttachmentDeltaImpulsePGS(
			constraint.raXn0_biasW[offset], constraint.raXn0_biasW[offset], constraint.raXn0_biasW[offset], velMultiplierXYZ_invMassW,
			constraint.low_high_limits[offset], constraint.axis_angle[offset], vel0, linVel1, 1.f / dt, 0.5f, deltaLinVel, deltaAngVel);

		// Update rigid body
		if(!rigidId.isStaticBody())
		{
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

		// Update cloth
		if(!deltaImpulse.isZero())
		{
			const PxVec3 deltaPos = -deltaImpulse * dt;

			if(elemIsVertex)
			{
				AtomicAdd(cloth.mDeltaPos[elemIdx], deltaPos * clothVel.w, 1.f);
			}
			else
			{
				AtomicAdd(cloth.mDeltaPos[triVertIdx.x], deltaPos * barycentric.x * triInvMass.x, 1.f);
				AtomicAdd(cloth.mDeltaPos[triVertIdx.y], deltaPos * barycentric.y * triInvMass.y, 1.f);
				AtomicAdd(cloth.mDeltaPos[triVertIdx.z], deltaPos * barycentric.z * triInvMass.z, 1.f);
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

// solve rigid attachment
extern "C" __global__ void cloth_solveRigidClothAttachmentTGSLaunch(
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
		{
			return;
		}

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgFEMRigidAttachmentConstraint& constraint = attachments[index];

		// Cloth info
		const PxU32 clothId = PxGetClothId(constraint.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(constraint.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(constraint.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];
		const float4* clothAccumDeltas = cloth.mAccumulatedDeltaPos;

		float4 clothAccumDelta;
		uint4 triVertIdx;
		float3 triInvMass;
		float4 barycentric;

		if(elemIsVertex)
		{
			clothAccumDelta = clothAccumDeltas[elemIdx];
		}
		else
		{
			barycentric = constraint.baryOrType[offset];
			triVertIdx = cloth.mTriangleVertexIndices[elemIdx];
			const float4 d0 = clothAccumDeltas[triVertIdx.x];
			const float4 d1 = clothAccumDeltas[triVertIdx.y];
			const float4 d2 = clothAccumDeltas[triVertIdx.z];
			clothAccumDelta = d0 * barycentric.x + d1 * barycentric.y + d2 * barycentric.z;

			triInvMass.x = d0.w;
			triInvMass.y = d1.w;
			triInvMass.z = d2.w;
		}
		PxVec3 linDelta1(clothAccumDelta.x, clothAccumDelta.y, clothAccumDelta.z);

		// Rigid body info
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(constraint.rigidId[offset]);
		if(rigidId.isStaticBody() && clothAccumDelta.w == 0.0f)
		{
			continue;
		}

		PxgVelocityPackTGS vel0;
		velocityReader.readVelocitiesTGS(rigidId, vel0);

		// Compute impulses
		PxVec3 deltaLinVel, deltaAngVel;
		PxVec3 deltaImpulse = calculateAttachmentDeltaImpulseTGS(offset, constraint, vel0, linDelta1, dt, FEMCLOTH_BIAS_COEFFICIENT, false,
																 deltaLinVel, deltaAngVel);

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

		// Update cloth
		if(!deltaImpulse.isZero())
		{
			const PxVec3 deltaPos = -deltaImpulse;

			if(elemIsVertex)
			{
				AtomicAdd(cloth.mDeltaPos[elemIdx], deltaPos * clothAccumDelta.w, 1.0f);
			}
			else
			{
				AtomicAdd(cloth.mDeltaPos[triVertIdx.x], deltaPos * barycentric.x * triInvMass.x, 1.0f);
				AtomicAdd(cloth.mDeltaPos[triVertIdx.y], deltaPos * barycentric.y * triInvMass.y, 1.0f);
				AtomicAdd(cloth.mDeltaPos[triVertIdx.z], deltaPos * barycentric.z * triInvMass.z, 1.0f);
			}
		}
	}
}