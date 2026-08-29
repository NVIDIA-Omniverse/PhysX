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

#include "PxgFEMCloth.h"
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
#include "PxgArticulationBlockData.h"
#include "PxgArticulationCoreDesc.h"
#include "PxgParticleSystem.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "particleSystem.cuh"
#include "PxgSimulationCoreDesc.h"
#include "dataReadWriteHelper.cuh"
#include "deformableCollision.cuh"
#include "deformableUtils.cuh"
#include "FEMClothUtil.cuh"

using namespace physx;

// Refcount-bump scheme for cloth-cloth pre-counts.
//   0 (default): magnitude weighting, .w += |bc_i| per touched vertex. The
//                slight over-correction from the magnitude / divide-by-.w
//                combination on CC centroid contacts compensates for residual
//                under-resolution under tight self-collision grip.
//   1: integer count, .w += 1.0 per touched vertex (the universal DB-DB
//      convention used by SS / SC). Cleaner math but visibly degrades
//      self-collision on twisted cloth; provided as a toggle for further
//      research.
#ifndef PX_CLOTH_CLOTH_INTEGER_REFCOUNT
#define PX_CLOTH_CLOTH_INTEGER_REFCOUNT 0
#endif

extern "C" __host__ void initFEMClothKernels2() {}

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

static __device__ void queryRigidClothContactReferenceCount(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	const PxReal dt,
	PxReal* appliedForces,
	PxU32* rigidBodyRefCounts,
	bool isTGS)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		const bool wasActive = contactInfo.isInCollision();
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxgDbRigidContactBlock& block = contactBlocks[workIndex / 32];
		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = block.barycentric[threadIndexInWarp];

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

			PxgRigidPart rigid;
			PxgDeformablePart<PxVec3> db;
			PxgDbContactState state;

			const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, artiCoreDesc->mMaxLinksPerArticulation);

			db.readCloth(cloth, elementId, bc, NULL, checkOnlyActivity);

			if(wasActive)
			{
				// Once activated, keep the contact pair active to maintain a conservative reference count.
				isActive = true;
			}
			else
			{
				rigid.readBodyProperties(rigidId, globalRigidBodyId, fricTan0_invMass0.w, NULL, NULL);
				rigid.readContactPrep(block, threadIndexInWarp);
				db.readContactPrep(block, threadIndexInWarp);
				state.readContactPrep(block, threadIndexInWarp);
				rigid.readVelocity(velocityReader, rigidId, isTGS);
				isActive = solveRbDbContact(rigid, db, state, appliedForces[workIndex], dt, wasActive, checkOnlyActivity);

				if(isActive)
				{
					contactInfo.markInCollision(true);
				}
			}

			if(isActive)
			{
				// Update cloth
				if(bc.w == 0.f) // Cloth triangle
					bumpDbRefCountTri(cloth.mDeltaPos, cloth.mTriangleVertexIndices[elementId], bc.x, bc.y, bc.z);
				else // Cloth vertex
					bumpDbRefCountVtx(cloth.mDeltaPos, elementId);

				// Update rigidbody
				if(globalRigidBodyId != -1 && fricTan0_invMass0.w != 0.0f)
				{
					// Increment the reference count of the rigid body.
					atomicAdd(&rigidBodyRefCounts[globalRigidBodyId], 1);
				}
			}
		}
	}
}

extern "C" __global__
void cloth_queryRigidClothContactReferenceCountLaunch(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo * contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	const PxReal dt,
	PxReal* appliedForces,
	PxU32* rigidBodyRefCounts,
	bool isTGS)
{
	queryRigidClothContactReferenceCount(femClothes, contactInfos, contactBlocks, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc,
		solverBodyVelPool, dt, appliedForces, rigidBodyRefCounts, isTGS);
}

static __device__ void solveRigidClothContact(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	const PxReal dt,
	float4* rigidDeltaVel,
	PxReal* appliedForces, // output
	PxU32* rigidBodyRefCounts,
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

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		PxgFemOtherContactInfo& contactInfo = contactInfos[workIndex];
		const bool isActive = contactInfo.isInCollision();
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 pairInd1 = contactInfo.pairInd1;

		PxgDbRigidContactBlock& block = contactBlocks[workIndex / 32];
		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[threadIndexInWarp];
		const float4 bc = block.barycentric[threadIndexInWarp];

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

			PxgRigidPart rigid;
			PxgDeformablePart<PxVec3> db;
			PxgDbContactState state;

			const int globalRigidBodyId = rigid.getGlobalRigidBodyId(prePrepDesc, rigidId, numSolverBodies, artiCoreDesc->mMaxLinksPerArticulation);

			if(isActive)
			{
				PxgFEMCloth& cloth = femClothes[clothId];

				rigid.readBodyProperties(rigidId, globalRigidBodyId, fricTan0_invMass0.w, rigidBodyRefCounts,
						   &rigidBodyMaterials[contactInfo.getRigidMaterialIndex()]);
				db.readCloth(cloth, elementId, bc, materials, checkOnlyActivity);

				rigid.readContactPrep(block, threadIndexInWarp);
				db.readContactPrep(block, threadIndexInWarp);
				state.readContactPrep(block, threadIndexInWarp);
				rigid.readVelocity(velocityReader, rigidId, isTGS);
				solveRbDbContact(rigid, db, state, appliedForces[workIndex], dt, isActive, checkOnlyActivity);

				// Compute cloth delta
				PxVec3 deltaPos;
				appliedForces[workIndex] = state.computeDeformableDelta(deltaPos, dt);

				// Update cloth: scatter .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
				db.writeCloth(cloth, elementId, bc, /*elemIsVertex*/ bc.w != 0.0f, deltaPos);
			}

			// Update rigid body. Must write every iteration even when !isActive (the
			// accumulate scan reads every slot); writeContactDeltas zeroes the no-op cases.
			rigid.writeContactDeltas(rigidDeltaVel, rigidId, state, workIndex, workIndex + tNumContacts);
		}
	}
}

// solve collision between cloth and primitives based on the sorted contact by rigid id
// store new velocity to rigid body buffer
extern "C" __global__ 
void cloth_solveRigidClothCollisionLaunch(
	PxgFEMCloth* femClothes,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbRigidContactBlock* contactBlocks,
	PxU32* numContacts,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	float4* rigidDeltaVel, // output
	PxReal* appliedForces,
	PxU32* rigidBodyRefCounts,
	const PxReal dt,
	PxsDeformableSurfaceMaterialData* materials,
	const PxsMaterialData * PX_RESTRICT rigidBodyMaterials,
	bool isTGS)
{
	solveRigidClothContact(femClothes, contactInfos, contactBlocks, numContacts, prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, dt,
						   rigidDeltaVel, appliedForces, rigidBodyRefCounts, materials, rigidBodyMaterials, isTGS);
}



//!
//! \brief    : solve cloth vs. cloth collision
//!

// Computes the per-pair VT contact prep (barycentric weights, contact normal, and
// initial penetration) from current positions. Returns false when the pair is too far
// apart or geometrically degenerate. Shared by the count pass (activation + refcount)
// and the solve pass (recompute -- the prep is no longer stored in a contact block).
PX_FORCE_INLINE __device__ bool computeClothClothVTPrep(
	const PxgFEMCloth* cloth0, const PxgFEMCloth* cloth1,
	const float4& x0, const float4& x1, const float4& x2, const float4& x3,
	const float4& prevX0, const float4& prevX1, const float4& prevX2, const float4& prevX3,
	PxVec4& weights, PxVec3& normalFromDisp, PxReal& initPen)
{
	const PxVec3 xx0 = PxLoad3(x0);
	const PxVec3 xx1 = PxLoad3(x1);
	const PxVec3 xx2 = PxLoad3(x2);
	const PxVec3 xx3 = PxLoad3(x3);

	PxReal s, t; // Barycentric coordinates
	closestPtPointTriangle(xx0, xx1, xx2, xx3, s, t);
	const PxReal r = 1.0f - s - t;
	weights = PxVec4(1.0f, -r, -s, -t); // cloth0 - cloth1

	const PxVec3 disp = weights[0] * xx0 + weights[1] * xx1 + weights[2] * xx2 + weights[3] * xx3;
	const PxReal dispSq = disp.magnitudeSquared();
	const PxReal restDist = cloth0->mRestDistance + cloth1->mRestDistance;

	if(dispSq > restDist * restDist)
		return false;

	if(dispSq < FEMCLOTH_NORMAL_FALLBACK_DISTSQ)
	{
		const PxVec3 ab = xx2 - xx1;
		const PxVec3 ac = xx3 - xx1;
		normalFromDisp = ab.cross(ac);

		const PxReal nLenSq = normalFromDisp.magnitudeSquared();
		if(nLenSq < FEMCLOTH_THRESHOLD)
			return false; // Degenerate triangle.

		normalFromDisp *= (1.0f / PxSqrt(nLenSq));

		const PxVec3 toVertex = xx0 - xx1;
		if(normalFromDisp.dot(toVertex) < 0.0f)
			normalFromDisp = -normalFromDisp;
	}
	else
	{
		normalFromDisp = disp * (1.0f / PxSqrt(dispSq));
	}

	const float4 relDelta4 = weights[0] * (x0 - prevX0) + weights[1] * (x1 - prevX1) +
	                         weights[2] * (x2 - prevX2) + weights[3] * (x3 - prevX3);
	const PxVec3 relDelta(relDelta4.x, relDelta4.y, relDelta4.z);
	// CC recomputes disp from current positions every substep, so subtract relDelta to
	// cancel the solver's relLinDelta in the NORMAL direction -> CN = disp.n - restDist.
	// relDelta = x - prevX is live in both PGS and TGS (unlike the soft/cloth shifts,
	// which are a no-op in TGS); relLinDelta's TANGENTIAL part is kept for friction.
	initPen = (disp - relDelta).dot(normalFromDisp) - restDist;
	return true;
}

// Computes the per-pair EE contact prep (edge barycentric weights, contact normal, and
// initial penetration) from current positions. Returns false when the closest points
// fall off the edges, the pair is too far apart, or the configuration is degenerate.
// Shared by the EE count and solve passes.
PX_FORCE_INLINE __device__ bool computeClothClothEEPrep(
	const PxgFEMCloth* cloth0, const PxgFEMCloth* cloth1,
	const float4& x0, const float4& x1, const float4& x2, const float4& x3,
	const float4& prevX0, const float4& prevX1, const float4& prevX2, const float4& prevX3,
	PxVec4& weights, PxVec3& normalFromDisp, PxReal& initPen)
{
	const PxVec3 xx0 = PxLoad3(x0);
	const PxVec3 xx1 = PxLoad3(x1);
	const PxVec3 xx2 = PxLoad3(x2);
	const PxVec3 xx3 = PxLoad3(x3);

	// Vertex-edge collisions are handled in vertex-triangle collisions.
	PxReal s, t;
	PxReal distSq;
	const bool isValid = closestPtLineLine(xx0, xx1, xx2, xx3, s, t, distSq);
	const bool isOutside = ((s < DEFORMABLE_BARYCENTRIC_THRESHOLD) || (s > DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD) ||
	                        (t < DEFORMABLE_BARYCENTRIC_THRESHOLD) || (t > DEFORMABLE_ONE_MINUS_BARYCENTRIC_THRESHOLD));

	if(isOutside)
		return false;

	weights = PxVec4(1.0f - s, s, -(1.0f - t), -t); // cloth0 - cloth1
	const PxVec3 disp = weights[0] * xx0 + weights[1] * xx1 + weights[2] * xx2 + weights[3] * xx3;
	const PxReal dispSq = disp.magnitudeSquared();
	const PxReal restDist = cloth0->mRestDistance + cloth1->mRestDistance;

	if(dispSq > restDist * restDist)
		return false;

	if(distSq < FEMCLOTH_NORMAL_FALLBACK_DISTSQ)
	{
		if(!isValid)
			return false; // Parallel/zero-length AND close: normal undefined.
		normalFromDisp = (xx1 - xx0).cross(xx3 - xx2);
		const PxReal nLenSq = normalFromDisp.magnitudeSquared();
		if(nLenSq < FEMCLOTH_THRESHOLD)
			return false;
		normalFromDisp *= (1.0f / PxSqrt(nLenSq));
	}
	else
	{
		normalFromDisp = disp * (1.0f / PxSqrt(distSq));
	}

	if(!isValid)
	{
		// Degenerate cases: re-adjust the weights so the constraint applies at the edge midpoints.
		weights = PxVec4(0.5f, 0.5f, -0.5f, -0.5f);
	}

	const float4 relDelta4 = weights[0] * (x0 - prevX0) + weights[1] * (x1 - prevX1) +
	                         weights[2] * (x2 - prevX2) + weights[3] * (x3 - prevX3);
	const PxVec3 relDelta(relDelta4.x, relDelta4.y, relDelta4.z);
	// Subtract relDelta to cancel the solver's relLinDelta in the normal direction
	// (CN = disp.n - restDist); see computeClothClothVTPrep for the full rationale.
	initPen = (disp - relDelta).dot(normalFromDisp) - restDist;
	return true;
}

extern "C" __global__ __launch_bounds__(256, 4)
void cloth_solveClothClothDeltaVTLaunch(
    PxgFEMCloth* cloths,
	PxgFemFemContactInfo* contactInfos,
	PxU32* numContacts,
	PxReal dt)
{
	// CC structural specifics vs. the generic DB-DB pattern: magnitude-weighted
	// refcount on mDeltaPos.w, no appliedForces persistence across solve passes,
	// friction averaged live from the cloth material at solve time.

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

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		const uint4 triVertId1 = cloth1->mTriangleVertexIndices[elementId1];
		const PxVec4T<PxU32> vertIndices(elementId0, triVertId1.x, triVertId1.y, triVertId1.z);

		// prevX is the baseline position at the last contact-pair-update
		// (mPrevPositionInContactOffset), refreshed at each cloth-cloth narrowphase.
		const float4 x0 = cloth0->mPosition_InvMass[vertIndices[0]];
		const float4 x1 = cloth1->mPosition_InvMass[vertIndices[1]];
		const float4 x2 = cloth1->mPosition_InvMass[vertIndices[2]];
		const float4 x3 = cloth1->mPosition_InvMass[vertIndices[3]];

		const float4 prevX0 = cloth0->mPrevPositionInContactOffset[vertIndices[0]];
		const float4 prevX1 = cloth1->mPrevPositionInContactOffset[vertIndices[1]];
		const float4 prevX2 = cloth1->mPrevPositionInContactOffset[vertIndices[2]];
		const float4 prevX3 = cloth1->mPrevPositionInContactOffset[vertIndices[3]];

		const PxReal frictionCoefficient = 0.25f * (cloth0->mDynamicFrictions[vertIndices[0]] +
		                                            cloth1->mDynamicFrictions[vertIndices[1]] +
		                                            cloth1->mDynamicFrictions[vertIndices[2]] +
		                                            cloth1->mDynamicFrictions[vertIndices[3]]);

		// Refcounts populated by the count+prep kernel (weighting per
		// PX_CLOTH_CLOTH_INTEGER_REFCOUNT toggle above).
		const PxReal count0 = cloth0->mDeltaPos[vertIndices[0]].w;
		const PxReal count1 = cloth1->mDeltaPos[vertIndices[1]].w;
		const PxReal count2 = cloth1->mDeltaPos[vertIndices[2]].w;
		const PxReal count3 = cloth1->mDeltaPos[vertIndices[3]].w;

		PxgDeformablePart<PxVec3> db0;
		PxgDeformablePart<PxVec3> db1;
		PxgDbContactState state;
		PxVec4 weights;
		PxVec3 normalFromDisp;
		PxReal initPen;
		if(!computeClothClothVTPrep(cloth0, cloth1, x0, x1, x2, x3, prevX0, prevX1, prevX2, prevX3,
		                            weights, normalFromDisp, initPen))
			continue; // pair separated or degenerated since the count pass

		const PxReal clamp = PxMax(cloth0->mPenBiasClamp, cloth1->mPenBiasClamp);
		db0.bc = PxVec3(1.0f, 0.0f, 0.0f); db0.penBiasClamp = clamp; // cloth0 side is a single vertex
		toSolverBc(db1.bc, make_float4(-weights[1], -weights[2], -weights[3], 0.0f)); db1.penBiasClamp = clamp;
		state.normal = PxVec3(-normalFromDisp.x, -normalFromDisp.y, -normalFromDisp.z);
		state.initPen = initPen;

		// Side 0: cloth0 vertex (linDelta is the raw vertex delta -- no barycentric weighting).
		db0.vertexInvMasses = PxVec3(count0 * x0.w, 0.0f, 0.0f);
		db0.linDelta = PxVec3(x0.x - prevX0.x, x0.y - prevX0.y, x0.z - prevX0.z);

		// Side 1: cloth1 triangle (bc-weighted vertex displacement).
		db1.vertexInvMasses = PxVec3(count1 * x1.w, count2 * x2.w, count3 * x3.w);
		const float4 db1Delta4 = db1.bc.x * (x1 - prevX1) + db1.bc.y * (x2 - prevX2) + db1.bc.z * (x3 - prevX3);
		db1.linDelta = PxVec3(db1Delta4.x, db1Delta4.y, db1Delta4.z);

		solveDbDbContact(db0, db1, state, 0.0f, 0.0f, frictionCoefficient, dt);

		if(state.deltaLambdaN != 0.0f || state.deltaLambdaT != 0.0f)
		{
			PxVec3 deltaPos;
			state.computeDeformableDelta(deltaPos, dt);

			// writeCloth scatters .xyz only; the count pass owns .w this iter.
			// Side 0 is a single vertex (elemIsVertex=true; vertIndices[0] the index);
			// side 1 is a triangle (elemIsVertex=false; three verts from elementId1).
			db0.writeCloth(*cloth0, vertIndices[0], make_float4(0.0f), /*elemIsVertex*/ true, deltaPos);
			db1.writeCloth(*cloth1, elementId1, make_float4(db1.bc.x, db1.bc.y, db1.bc.z, 0.0f), /*elemIsVertex*/ false, -deltaPos);
		}
	}
}

extern "C" __global__ __launch_bounds__(256, 4)
void cloth_solveClothClothDeltaEELaunch(
    PxgFEMCloth* cloths,
	PxgFemFemContactInfo* contactInfos,
	PxU32* numContacts,
	PxReal dt)
{
	// EE-specific bookkeeping vs. VT: edge local indices come from
	// contactInfo.getAuxInd{0,1}; vertex indices reconstructed from
	// cloth.mTriangleVertexIndices.

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

		const PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
		const PxU32 e1_localIndex0 = contactInfo.getAuxInd1();

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		PxVec4T<PxU32> vertIndices;

		const PxU32 e0_localIndex1 = (e0_localIndex0 + 1) % 3;
		const uint4 triVertInd0 = cloth0->mTriangleVertexIndices[elementId0];
		const PxU32* vertexIndices0 = reinterpret_cast<const PxU32*>(&triVertInd0);
		vertIndices[0] = vertexIndices0[e0_localIndex0];
		vertIndices[1] = vertexIndices0[e0_localIndex1];

		const PxU32 e1_localIndex1 = (e1_localIndex0 + 1) % 3;
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

		const PxReal frictionCoefficient = 0.25f * (cloth0->mDynamicFrictions[vertIndices[0]] +
		                                            cloth0->mDynamicFrictions[vertIndices[1]] +
		                                            cloth1->mDynamicFrictions[vertIndices[2]] +
		                                            cloth1->mDynamicFrictions[vertIndices[3]]);

		const PxReal count0 = cloth0->mDeltaPos[vertIndices[0]].w;
		const PxReal count1 = cloth0->mDeltaPos[vertIndices[1]].w;
		const PxReal count2 = cloth1->mDeltaPos[vertIndices[2]].w;
		const PxReal count3 = cloth1->mDeltaPos[vertIndices[3]].w;

		PxgDeformablePart<PxVec3> db0;
		PxgDeformablePart<PxVec3> db1;
		PxgDbContactState state;
		PxVec4 weights;
		PxVec3 normalFromDisp;
		PxReal initPen;
		if(!computeClothClothEEPrep(cloth0, cloth1, x0, x1, x2, x3, prevX0, prevX1, prevX2, prevX3,
		                            weights, normalFromDisp, initPen))
			continue; // pair separated or degenerated since the count pass

		const PxReal clamp = PxMax(cloth0->mPenBiasClamp, cloth1->mPenBiasClamp);
		toSolverBc(db0.bc, make_float4(weights[0], weights[1], 0.0f, 0.0f));   db0.penBiasClamp = clamp;
		toSolverBc(db1.bc, make_float4(-weights[2], -weights[3], 0.0f, 0.0f)); db1.penBiasClamp = clamp;
		state.normal = PxVec3(-normalFromDisp.x, -normalFromDisp.y, -normalFromDisp.z);
		state.initPen = initPen;

		// Side 0: cloth0 edge. bc.x / bc.y are the unsigned edge barycentrics (1-s, s).
		db0.vertexInvMasses = PxVec3(count0 * x0.w, count1 * x1.w, 0.0f);
		const float4 db0Delta4 = db0.bc.x * (x0 - prevX0) + db0.bc.y * (x1 - prevX1);
		db0.linDelta = PxVec3(db0Delta4.x, db0Delta4.y, db0Delta4.z);

		// Side 1: cloth1 edge. bc.x / bc.y are the unsigned edge barycentrics (1-t, t);
		// the prep flipped weights[2..3]'s sign so this side reads positive.
		db1.vertexInvMasses = PxVec3(count2 * x2.w, count3 * x3.w, 0.0f);
		const float4 db1Delta4 = db1.bc.x * (x2 - prevX2) + db1.bc.y * (x3 - prevX3);
		db1.linDelta = PxVec3(db1Delta4.x, db1Delta4.y, db1Delta4.z);

		solveDbDbContact(db0, db1, state, 0.0f, 0.0f, frictionCoefficient, dt);

		if(state.deltaLambdaN != 0.0f || state.deltaLambdaT != 0.0f)
		{
			PxVec3 deltaPos;
			state.computeDeformableDelta(deltaPos, dt);

			// vertexInvMasses is pre-scaled by count (line above), so the
			// vertexInvMasses > 0 gate inside writeClothEdge also covers
			// count == 0.
			db0.writeClothEdge(*cloth0, vertIndices[0], vertIndices[1],  deltaPos);
			db1.writeClothEdge(*cloth1, vertIndices[2], vertIndices[3], -deltaPos);
		}
	}
}

extern "C" __global__ __launch_bounds__(256, 4)
void cloth_queryClothClothContactVTCountLaunch(
	PxgFEMCloth* cloths,
	PxgFemFemContactInfo* contactInfos,
	PxU32* numContacts)
{
	// Runs the per-substep activation (markInCollision) + refcount accumulation
	// (mDeltaPos.w += |coeff|), recomputing the contact prep each substep. Lives at the
	// substep cadence because positions change between substeps, so anything baked at
	// the slower narrowphase-pair-discovery cadence would go stale.

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

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

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

		PxVec4 weights;
		PxVec3 normalFromDisp;
		PxReal initPen;
		if(!computeClothClothVTPrep(cloth0, cloth1, x0, x1, x2, x3, prevX0, prevX1, prevX2, prevX3,
		                            weights, normalFromDisp, initPen))
		{
			contactInfo.markInCollision(false);
			continue;
		}

		// VT layout: vert 0 -> cloth0; verts 1..3 -> cloth1 (triangle).
#if PX_CLOTH_CLOTH_INTEGER_REFCOUNT
		bumpDbRefCountVtx(cloth0->mDeltaPos, vertIndices[0]);
		bumpDbRefCountTri(cloth1->mDeltaPos, triVertId1, weights[1], weights[2], weights[3]);
#else
		if(PxAbs(weights[0]) > 1e-3f) atomicAdd(&cloth0->mDeltaPos[vertIndices[0]].w, PxAbs(weights[0]));
		if(PxAbs(weights[1]) > 1e-3f) atomicAdd(&cloth1->mDeltaPos[vertIndices[1]].w, PxAbs(weights[1]));
		if(PxAbs(weights[2]) > 1e-3f) atomicAdd(&cloth1->mDeltaPos[vertIndices[2]].w, PxAbs(weights[2]));
		if(PxAbs(weights[3]) > 1e-3f) atomicAdd(&cloth1->mDeltaPos[vertIndices[3]].w, PxAbs(weights[3]));
#endif
		contactInfo.markInCollision(true);
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

		const PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
		const PxU32 e1_localIndex0 = contactInfo.getAuxInd1();

		const PxU32 clothId0 = PxGetClothId(pairInd0);
		const PxU32 clothId1 = PxGetClothId(pairInd1);

		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		PxgFEMCloth* cloth0 = &cloths[clothId0];
		PxgFEMCloth* cloth1 = &cloths[clothId1];

		PxVec4T<PxU32> vertIndices;

		const PxU32 e0_localIndex1 = (e0_localIndex0 + 1) % 3;
		const uint4 triVertInd0 = cloth0->mTriangleVertexIndices[elementId0];
		const PxU32* vertexIndices0 = reinterpret_cast<const PxU32*>(&triVertInd0);
		vertIndices[0] = vertexIndices0[e0_localIndex0];
		vertIndices[1] = vertexIndices0[e0_localIndex1];

		const PxU32 e1_localIndex1 = (e1_localIndex0 + 1) % 3;
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

		PxVec4 weights;
		PxVec3 normalFromDisp;
		PxReal initPen;
		if(!computeClothClothEEPrep(cloth0, cloth1, x0, x1, x2, x3, prevX0, prevX1, prevX2, prevX3,
		                            weights, normalFromDisp, initPen))
		{
			contactInfo.markInCollision(false);
			continue;
		}

		// EE layout: verts 0..1 -> cloth0 (edge); verts 2..3 -> cloth1 (edge).
#if PX_CLOTH_CLOTH_INTEGER_REFCOUNT
		bumpDbRefCountEdge(cloth0->mDeltaPos, vertIndices[0], vertIndices[1], weights[0], weights[1]);
		bumpDbRefCountEdge(cloth1->mDeltaPos, vertIndices[2], vertIndices[3], weights[2], weights[3]);
#else
		if(PxAbs(weights[0]) > 1e-3f) atomicAdd(&cloth0->mDeltaPos[vertIndices[0]].w, PxAbs(weights[0]));
		if(PxAbs(weights[1]) > 1e-3f) atomicAdd(&cloth0->mDeltaPos[vertIndices[1]].w, PxAbs(weights[1]));
		if(PxAbs(weights[2]) > 1e-3f) atomicAdd(&cloth1->mDeltaPos[vertIndices[2]].w, PxAbs(weights[2]));
		if(PxAbs(weights[3]) > 1e-3f) atomicAdd(&cloth1->mDeltaPos[vertIndices[3]].w, PxAbs(weights[3]));
#endif
		contactInfo.markInCollision(true);
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

// Cloth-particle contact pre-count pass. For each active CP contact, bumps
// cloth.mDeltaPos[v].w by 1 per touched triangle vertex. The CP solve consumes
// .w for XPBD mass-splitting; the finalize divides .xyz by max(.w, 1) and zeros
// both. Activation is decided by solveDbDbContact with checkOnlyActivity=true
// and recorded via markInCollision so contact reports see the same state.
//
// Cloth-vertex contacts (bc.w != 0) are not produced by cloth-particle narrowphase
// today, so the pre-count handles triangle contacts only.
extern "C" __global__ void cloth_queryCPContactReferenceCountLaunch(
	PxgFEMCloth*								clothes,
	PxgParticleSystem*							particlesystems,
	PxgFemOtherContactInfo*						contactInfos,
	PxgDbParticleContactBlock*					contactBlocks,
	PxU32*										numContacts,
	const PxReal								dt
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
		PxgDbParticleContactBlock& block = contactBlocks[workIndex / 32];

		// pairInd0 = particle, pairInd1 = cloth triangle (matches narrowphase).
		const PxU64 pairInd0 = contactInfo.pairInd0;
		const PxU32 particleSystemId = PxGetParticleSystemId(pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(pairInd0);
		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		// Kinematic particles (invMass0 == 0) are inactive end-to-end:
		// cloth_solveCPOutputClothDeltaVLaunch skips the whole solve body.
		if(invMass0 == 0.f)
		{
			contactInfo.markInCollision(false);
			continue;
		}

		const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		const PxU32 clothId = PxGetClothId(pairInd1);
		PxgFEMCloth& cloth = clothes[clothId];
		const PxU32 elementId = PxGetClothElementIndex(pairInd1);

		const float4 clothBcF4 = block.barycentric[threadIndexInWarp];

		// Side convention matches cloth_solveCPOutputClothDeltaVLaunch:
		// side 0 = cloth, side 1 = particle. The prep stores state.normal
		// as -narrowphase to match this db0 -> db1 convention.
		PxgDeformablePart<PxVec3> clothPart;
		clothPart.bc = PxVec3(clothBcF4.x, clothBcF4.y, clothBcF4.z);
		clothPart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
		clothPart.readCloth(cloth, elementId, clothBcF4, NULL, /*countReferenceOnly*/ true);

		PxgDeformablePart<PxVec3> particlePart;
		particlePart.bc = PxVec3(1.0f, 0.0f, 0.0f);
		particlePart.vertexInvMasses = PxVec3(invMass0, 0.0f, 0.0f);
		particlePart.linDelta = PxVec3(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
		particlePart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];

		PxgDbContactState state;
		state.readContactPrepDbDb(block, threadIndexInWarp);

		const bool isActive = solveDbDbContact(clothPart, particlePart, state,
												 /*appliedNormalLambdaRef*/ 0.0f,
												 /*appliedTanLambdaRef*/ 0.0f,
												 /*frictionCoefficient*/ 0.0f,
												 dt,
												 /*checkOnlyActivity*/ true);

		contactInfo.markInCollision(isActive);

		if(isActive)
		{
			// Integer-count refcount bump per touched triangle vertex (gated |bc| > 1e-3).
			const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
			bumpDbRefCountTri(cloth.mDeltaPos, triVertId, clothBcF4.x, clothBcF4.y, clothBcF4.z);

#if PX_DB_PARTICLE_MASS_SPLIT
			// Particle-side mass-split count (dual of the cloth-vertex refCount).
			atomicAdd(&particleSystem.mAccumDeltaP[particleIndex].w, 1.0f);
#endif
		}
	}
}


// Cloth-particle contact solve, cloth side of the split writeback.
// Side 0 = cloth (PxVec3, triangle or vertex), side 1 = particle (PxVec3,
// a single vertex with bc=(1,0,0) filled inline). The prep stores state.normal as -narrowphase,
// which matches solveDbDbContact's db0 -> db1 convention.
//
// Mass-splitting: cloth_queryCPContactReferenceCountLaunch pre-populates
// cloth.mDeltaPos[v].w with the refcount; readCloth multiplies vertexInvMasses
// by .w; scatter via writeCloth does NOT touch .w so it survives to the
// finalize's max(.w, 1) divide.
extern "C" __global__
void cloth_solveCPOutputClothDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgParticleSystem* particlesystems,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbParticleContactBlock* contactBlocks,
	PxU32* numContacts,
	float2* appliedForces, // output
	PxsDeformableSurfaceMaterialData* materials,
	const PxReal dt)
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
		PxgDbParticleContactBlock& block = contactBlocks[workIndex / 32];

		// first pairInd0 is particle
		const PxU32 particleSystemId = PxGetParticleSystemId(contactInfo.pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(contactInfo.pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if(invMass0 != 0.f)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(
			    particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 clothBcF4 = block.barycentric[threadIndexInWarp];

			const PxU32 clothId = PxGetClothId(pairInd1);
			PxgFEMCloth& cloth = clothes[clothId];
			const PxU32 elementId = PxGetClothElementIndex(pairInd1);

			// Cloth side. countReferenceOnly=false consumes the refCount in .w
			// (populated by cloth_queryCPContactReferenceCountLaunch) to
			// mass-split vertexInvMasses, and reads materials for friction.
			PxgDeformablePart<PxVec3> clothPart;
			clothPart.bc = PxVec3(clothBcF4.x, clothBcF4.y, clothBcF4.z);
			clothPart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			clothPart.readCloth(cloth, elementId, clothBcF4, materials, /*countReferenceOnly*/ false);

			// Particle participant (side 1): single vertex, scalar invMass, no pen-bias.
			PxgDeformablePart<PxVec3> particlePart;
			particlePart.bc = PxVec3(1.0f, 0.0f, 0.0f);
#if PX_DB_PARTICLE_MASS_SPLIT
			// Inflate by the contact count so both solve halves compute the same lambda.
			const PxReal particleRefCount = PxMax(particleSystem.mAccumDeltaP[particleIndex].w, 1.0f);
			particlePart.vertexInvMasses = PxVec3(invMass0 * particleRefCount, 0.0f, 0.0f);
#else
			particlePart.vertexInvMasses = PxVec3(invMass0, 0.0f, 0.0f);
#endif
			particlePart.linDelta = PxVec3(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			particlePart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			particlePart.friction = psMat.friction;

			PxgDbContactState state;
			state.readContactPrepDbDb(block, threadIndexInWarp);

			const PxReal frictionCoefficient = (clothPart.friction + particlePart.friction) * 0.5f;

			float2 appliedForce = appliedForces[workIndex];
			solveDbDbContact(clothPart, particlePart, state,
							   appliedForce.x, appliedForce.y, frictionCoefficient, dt);

			if(state.deltaLambdaN != 0.f || state.deltaLambdaT != 0.f)
			{
				PxVec3 deltaPos;
				state.computeDeformableDelta(deltaPos, dt);

				// Cloth side scatter via writeCloth: .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
				clothPart.writeCloth(cloth, elementId, clothBcF4, /*elemIsVertex*/ clothBcF4.w != 0.0f, deltaPos);
			}

			appliedForce.x = state.accumulatedDeltaLambdaN;
			appliedForce.y += state.deltaLambdaT;
			appliedForces[workIndex] = appliedForce;
		}
	}
}

// Cloth-particle contact solve, particle side of the split writeback.
// Mirror of cloth_solveCPOutputClothDeltaVLaunch above; only the scatter
// target differs (deltaP[] vs. cloth.mDeltaPos atomics).
extern "C" __global__
void cloth_solveCPOutputParticleDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgParticleSystem* particlesystems,
	PxgFemOtherContactInfo* contactInfos,
	PxgDbParticleContactBlock* contactBlocks,
	PxU32* numContacts,
	float4* deltaP,        // output
	float2* appliedForces, // output
	PxsDeformableSurfaceMaterialData* materials,
	const PxReal dt
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
		PxgDbParticleContactBlock& block = contactBlocks[workIndex / 32];

		const PxU32 particleSystemId = PxGetParticleSystemId(contactInfo.pairInd0);
		PxgParticleSystem& particleSystem = particlesystems[particleSystemId];
		const PxU32 particleIndex = PxGetParticleIndex(contactInfo.pairInd0);

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[particleIndex];
		const PxReal invMass0 = deltaP_invMass.w;

		if(invMass0 != 0.f)
		{
			const PxU32* PX_RESTRICT phases = particleSystem.mSortedPhaseArray;
			const PxU16* const PX_RESTRICT phaseToMat = particleSystem.mPhaseGroupToMaterialHandle;

			const PxU32 phase = phases[particleIndex];
			const PxU32 group = PxGetGroup(phase);
			const PxU32 mi = phaseToMat[group];
			const PxsParticleMaterialData& psMat = getParticleMaterial<PxsParticleMaterialData>(
			    particleSystem.mParticleMaterials, mi, particleSystem.mParticleMaterialStride);

			const PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
			const float4 clothBcF4 = block.barycentric[threadIndexInWarp];
			const PxU32 clothId = PxGetClothId(pairInd1);
			PxgFEMCloth& cloth = clothes[clothId];
			const PxU32 elementId = PxGetClothElementIndex(pairInd1);

			PxgDeformablePart<PxVec3> clothPart;
			clothPart.bc = PxVec3(clothBcF4.x, clothBcF4.y, clothBcF4.z);
			clothPart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			// Read full (not count-only) so both halves see the same invMass + friction and
			// compute the identical lambda -- the impulse stays equal-and-opposite. Side-effect
			// free here: this kernel scatters only to deltaP[], never to cloth.mDeltaPos.
			clothPart.readCloth(cloth, elementId, clothBcF4, materials, /*countReferenceOnly*/ false);

			PxgDeformablePart<PxVec3> particlePart;
			particlePart.bc = PxVec3(1.0f, 0.0f, 0.0f);
#if PX_DB_PARTICLE_MASS_SPLIT
			// Inflate by the contact count so both solve halves compute the same lambda.
			const PxReal particleRefCount = PxMax(particleSystem.mAccumDeltaP[particleIndex].w, 1.0f);
			particlePart.vertexInvMasses = PxVec3(invMass0 * particleRefCount, 0.0f, 0.0f);
#else
			particlePart.vertexInvMasses = PxVec3(invMass0, 0.0f, 0.0f);
#endif
			particlePart.linDelta = PxVec3(deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);
			particlePart.penBiasClamp = block.maxPenBiasClamp[threadIndexInWarp];
			particlePart.friction = psMat.friction;

			PxgDbContactState state;
			state.readContactPrepDbDb(block, threadIndexInWarp);

			const PxReal frictionCoefficient = (clothPart.friction + particlePart.friction) * 0.5f;

			float2 appliedForce = appliedForces[workIndex];
			solveDbDbContact(clothPart, particlePart, state,
							   appliedForce.x, appliedForce.y, frictionCoefficient, dt);

			// Particle-side scatter into deltaP[] (aggregated into mDeltaP later); .w flags a live contact.
			PxVec3 deltaPos;
			state.computeDeformableDelta(deltaPos, dt);
			const PxVec3 deltaV = -deltaPos * particlePart.vertexInvMasses.x;
			const PxReal w = (state.deltaLambdaN != 0.f || state.deltaLambdaT != 0.f) ? 1.f : 0.f;
			deltaP[workIndex] = make_float4(deltaV.x, deltaV.y, deltaV.z, w);

			appliedForce.x = state.accumulatedDeltaLambdaN;
			appliedForce.y += state.deltaLambdaT;
			appliedForces[workIndex] = appliedForce;
		}
	}
}

// Pre-count pass for rigid-cloth attachments. Bumps cloth.mDeltaPos[v].w with
// the per-vertex refCount; the solve inflates per-vertex invMass by .w and
// writes without touching it. Finalize divides .xyz by .w, cancelling the
// inflation. Shared between PGS and TGS dispatch.
extern "C" __global__ void cloth_queryRigidClothAttachmentReferenceCountLaunch(
	PxgFEMCloth* clothes,
	PxgDbRigidAttachmentBlock* attachmentBlocks,
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

		const PxgDbRigidAttachmentBlock& block = attachmentBlocks[index];

		const PxU32 clothId = PxGetClothId(block.elemId[offset]);
		const PxU32 elemIdx = PxGetClothElementIndex(block.elemId[offset]);
		const bool elemIsVertex = PxGetIsVertexType(block.baryOrType[offset]);

		PxgFEMCloth& cloth = clothes[clothId];

		if(elemIsVertex)
		{
			bumpDbRefCountVtx(cloth.mDeltaPos, elemIdx);
		}
		else
		{
			const float4 barycentric = block.baryOrType[offset];
			bumpDbRefCountTri(cloth.mDeltaPos, cloth.mTriangleVertexIndices[elemIdx], barycentric.x, barycentric.y, barycentric.z);
		}
	}
}

// Serves both PGS and TGS dispatches via the isTGS arg. isVelocityIteration
// is hard-coded false at every call site (no vel-iter branch on the
// cloth-rigid attach path).
extern "C" __global__
void cloth_solveRigidClothAttachmentLaunch(
	PxgFEMCloth* clothes,
	PxgDbRigidAttachmentBlock* attachmentBlocks,
	const PxU32 numAttachments,
	PxgPrePrepDesc* prePrepDesc,
	PxgSolverCoreDesc* solverCoreDesc,
	PxgArticulationCoreDesc* artiCoreDesc,
	float4* solverBodyVelPool,
	const PxReal dt,
	const PxReal biasCoefficient,
	float4* rigidDeltaVel, // output
	bool isVelocityIteration,
	bool isTGS
)
{
	const PxU32 numSolverBodies = solverCoreDesc->numSolverBodies;
	const PxU32 nbBlocksRequired = (numAttachments + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	PxgVelocityReader velocityReader(prePrepDesc, solverCoreDesc, artiCoreDesc, solverBodyVelPool, numSolverBodies);

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;
		if(workIndex >= numAttachments)
		{
			return;
		}

		const PxU32 index = workIndex / 32;
		const PxU32 offset = workIndex & 31;

		const PxgDbRigidAttachmentBlock& block = attachmentBlocks[index];

		// Cloth info
		const PxU32 elemId = block.elemId[offset];
		const PxU32 clothId = PxGetClothId(elemId);
		const PxU32 elemIdx = PxGetClothElementIndex(elemId);
		const float4 baryOrType = block.baryOrType[offset];
		const bool elemIsVertex = PxGetIsVertexType(baryOrType);
		PxgFEMCloth& cloth = clothes[clothId];

		// Deformable side: PGS reads mVelocity_InvMass, TGS reads
		// mAccumulatedDeltaPos. vertexInvMasses come refCount-inflated;
		// attachPointInvMass carries the raw bc^2*invM scalar for the
		// solver's denomBias.
		PxgDeformablePart<PxVec3> db;
		db.readClothAttachment(cloth, elemIdx, baryOrType, elemIsVertex, isTGS);

		// Both-static early exit: rigid static AND every contributing cloth
		// vertex kinematic => attachPointInvMass = 0.
		const PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(block.rigidId[offset]);
		if(rigidId.isStaticBody() && db.attachPointInvMass == 0.0f)
		{
			continue;
		}

		// Rigid side. PGS: readVelocity zeros linDelta/angDelta; TGS: they carry
		// the rigid's accumulated linear/angular delta.
		PxgRigidPart rigid;
		rigid.readAttachmentPrep(block, offset);
		rigid.readVelocity(velocityReader, rigidId, isTGS);

		PxVec3 deltaLinVel, deltaAngVel;
		const PxVec3 deltaImpulse = solveRbDbAttachment(rigid, db, dt, biasCoefficient,
															 isTGS, isVelocityIteration,
															 deltaLinVel, deltaAngVel);

		// Rigid writeback
		rigid.writeAttachmentDeltas(rigidDeltaVel, rigidId, deltaLinVel, deltaAngVel, workIndex, workIndex + numAttachments);

		// Scatter .xyz weighted by the refCount-inflated vertexInvMasses; the pre-count owns .w.
		// elemIsVertex (from PxGetIsVertexType) selects the vertex/triangle scatter; bc unread for a vertex.
		if(!deltaImpulse.isZero())
		{
			const PxVec3 deltaPos = -deltaImpulse * dt;
			db.writeCloth(cloth, elemIdx, baryOrType, elemIsVertex, deltaPos);
		}
	}
}

// Pre-count pass for DB-DB cloth-cloth attachments. Both sides are clothes
// (triangle, 3 verts). Bumps cloth{0,1}.mDeltaPos[v].w for each contributing
// vertex on both sides. cloth_solveOutputAttachmentClothClothDeltaVLaunch then
// reads .w as the per-vertex Jacobi mass-splitting factor (via
// readClothAttachmentDbDb), inflates per-vertex invMass +
// attachPointInvMassSplit, scatters .xyz only, and the next
// cloth_applyExternalDeltasLaunch finalize divides .xyz/.w cancelling the
// inflation.
//
// Refcount scheme: INTEGER count (atomicAdd .w by 1 per touched vertex). This
// matches every other RB-DB / DB-DB attachment pre-count in the codebase.
// Deliberately differs from the CC CONTACT pre-count above, which uses
// magnitude-weighted .w bumps -- that scheme is load-bearing for
// ClothTwistTest's tight self-collision grip per the
// PX_CLOTH_CLOTH_INTEGER_REFCOUNT toggle comment at the top of this file.
// Multi-attach-per-vertex scenarios for cloth-cloth attachments are rare
// enough that integer-count's "average all contributions" semantics is the safer
// default; revisit if a test surfaces a similar grip dependence.
extern "C" __global__ void cloth_queryClothClothAttachmentReferenceCountLaunch(
	PxgFEMCloth*								clothes,
	PxgDbDbAttachmentBlock*						attachmentBlocks,
	const PxU32									numAttachments)
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

		const PxgDbDbAttachmentBlock& block = attachmentBlocks[index];

		// Per-vertex refcount (.w += 1) for both cloth triangles via the shared helper.
		// Kinematic verts now get counted too, but contribute 0 in the solve, so it's a no-op.

		// Side 0
		{
			const PxU32 elemId0 = block.elemId0[offset];
			const PxU32 clothId0 = PxGetClothId(elemId0);
			const PxU32 triIdx0 = PxGetClothElementIndex(elemId0);
			const float4 bary0 = block.barycentric0[offset];
			PxgFEMCloth& cloth0 = clothes[clothId0];
			const uint4 triInd0 = cloth0.mTriangleVertexIndices[triIdx0];

			bumpDbRefCountTri(cloth0.mDeltaPos, triInd0, bary0.x, bary0.y, bary0.z);
		}

		// Side 1
		{
			const PxU32 elemId1 = block.elemId1[offset];
			const PxU32 clothId1 = PxGetClothId(elemId1);
			const PxU32 triIdx1 = PxGetClothElementIndex(elemId1);
			const float4 bary1 = block.barycentric1[offset];
			PxgFEMCloth& cloth1 = clothes[clothId1];
			const uint4 triInd1 = cloth1.mTriangleVertexIndices[triIdx1];

			bumpDbRefCountTri(cloth1.mDeltaPos, triInd1, bary1.x, bary1.y, bary1.z);
		}
	}
}

//!
//! \brief    : solve cloth vs. cloth attachment
//!

extern "C" __global__
void cloth_solveOutputAttachmentClothClothDeltaVLaunch(
	PxgFEMCloth* clothes,
	PxgDbDbAttachmentBlock* attachmentBlocks,
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
		const PxgDbDbAttachmentBlock& block = attachmentBlocks[index];

		const PxU32 elemId0 = block.elemId0[offset];
		const PxU32 clothId0 = PxGetClothId(elemId0);
		const PxU32 elementId0 = PxGetClothElementIndex(elemId0);
		const float4 bary0 = block.barycentric0[offset];
		PxgFEMCloth& cloth0 = clothes[clothId0];

		PxgDeformablePart<PxVec3> db0;
		const PxVec3 pos0 = db0.readClothAttachmentDbDb(cloth0, elementId0, bary0);

		const PxU32 elemId1 = block.elemId1[offset];
		const PxU32 clothId1 = PxGetClothId(elemId1);
		const PxU32 elementId1 = PxGetClothElementIndex(elemId1);
		const float4 bary1 = block.barycentric1[offset];
		PxgFEMCloth& cloth1 = clothes[clothId1];

		PxgDeformablePart<PxVec3> db1;
		const PxVec3 pos1 = db1.readClothAttachmentDbDb(cloth1, elementId1, bary1);

		PxVec3 delta;
		if(solveDbDbAttachment(db0, db1, pos0, pos1, delta))
		{
			const uint4 triInd0 = cloth0.mTriangleVertexIndices[elementId0];
			const uint4 triInd1 = cloth1.mTriangleVertexIndices[elementId1];
			const float4 b0w = make_float4(db0.bc.x*db0.vertexInvMasses.x, db0.bc.y*db0.vertexInvMasses.y,
				db0.bc.z*db0.vertexInvMasses.z, 0.0f);
			const float4 b1w = make_float4(db1.bc.x*db1.vertexInvMasses.x, db1.bc.y*db1.vertexInvMasses.y,
				db1.bc.z*db1.vertexInvMasses.z, 0.0f);
			updatePositionDeltaTri(cloth0.mDeltaPos, triInd0,  delta, b0w);
			updatePositionDeltaTri(cloth1.mDeltaPos, triInd1, -delta, b1w);
		}
	}
}

