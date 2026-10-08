// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DEFORMABLE_AND_PARTICLE_UTILS_CUH__
#define __DEFORMABLE_AND_PARTICLE_UTILS_CUH__

#include "foundation/PxMathUtils.h"
#include "PxDeformableSurface.h" // for PX_MAX_NB_DEFORMABLE_SURFACE_TRI
#include "PxDeformableVolume.h" // for PX_MAX_NB_DEFORMABLE_VOLUME_TET
#include "PxsMaterialCombiner.h"
#include "PxsDeformableVolumeMaterialCore.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "PxgDeformableContactInfo.h"
#include "PxgDeformableConstraints.h"
#include "PxgFEMCloth.h"
#include "PxgSoftBody.h"
#include "PxgParticleSystem.h"
#include "PxgArticulation.h"
#include "PxgBodySim.h"
#include "dataReadWriteHelper.cuh"
#include "PxgSolverCoreDesc.h"
#include "atomic.cuh"

namespace physx
{

// Integer-count refcount bumps: add 1 to .w per touched vertex. The universal DB refcount
// convention for the soft-body, cloth and particle contact and attachment pre-counts, paired
// with the apply-deltas kernels' PxMax(.w, 1) finalize divide.

PX_FORCE_INLINE __device__ void bumpDbRefCountVtx(float4* PX_RESTRICT deltaBuf, PxU32 vertIndex)
{
	atomicAdd(&deltaBuf[vertIndex].w, 1.0f);
}

PX_FORCE_INLINE __device__ void bumpDbRefCountEdge(float4* PX_RESTRICT deltaBuf, PxU32 v0, PxU32 v1, PxReal w0, PxReal w1)
{
	if(PxAbs(w0) > 1e-3f) atomicAdd(&deltaBuf[v0].w, 1.0f);
	if(PxAbs(w1) > 1e-3f) atomicAdd(&deltaBuf[v1].w, 1.0f);
}

PX_FORCE_INLINE __device__ void bumpDbRefCountTri(float4* PX_RESTRICT deltaBuf, const uint4& triVertIndices,
                                               PxReal w0, PxReal w1, PxReal w2)
{
	if(PxAbs(w0) > 1e-3f) atomicAdd(&deltaBuf[triVertIndices.x].w, 1.0f);
	if(PxAbs(w1) > 1e-3f) atomicAdd(&deltaBuf[triVertIndices.y].w, 1.0f);
	if(PxAbs(w2) > 1e-3f) atomicAdd(&deltaBuf[triVertIndices.z].w, 1.0f);
}

PX_FORCE_INLINE __device__ void bumpDbRefCountTet(float4* PX_RESTRICT deltaBuf, const uint4& tetVertIndices,
                                               PxReal w0, PxReal w1, PxReal w2, PxReal w3)
{
	if(PxAbs(w0) > 1e-3f) atomicAdd(&deltaBuf[tetVertIndices.x].w, 1.0f);
	if(PxAbs(w1) > 1e-3f) atomicAdd(&deltaBuf[tetVertIndices.y].w, 1.0f);
	if(PxAbs(w2) > 1e-3f) atomicAdd(&deltaBuf[tetVertIndices.z].w, 1.0f);
	if(PxAbs(w3) > 1e-3f) atomicAdd(&deltaBuf[tetVertIndices.w].w, 1.0f);
}

// Accumulate a per-vertex solve delta (a position delta in position iterations,
// velocity * dt in velocity iterations) into a per-iteration scratch buffer.
// .w is reserved for the mass-split refCount.

static __device__ void updateDeltaVtx(float4* outputDeltas, PxU32 vertIndex,
	const PxVec3& delta, PxReal invMass)
{
	if (invMass > 0.0f)
		AtomicAdd(outputDeltas[vertIndex], delta*invMass);
}

static __device__ void updateDeltaEdge(float4* outputDeltas, PxU32 vertIndex0, PxU32 vertIndex1,
	const PxVec3& delta, PxReal invMassBary0, PxReal invMassBary1)
{
	if (invMassBary0 > 0.0f)
		AtomicAdd(outputDeltas[vertIndex0], delta*invMassBary0);
	if (invMassBary1 > 0.0f)
		AtomicAdd(outputDeltas[vertIndex1], delta*invMassBary1);
}

static __device__ void updateDeltaTri(float4* outputDeltas, const uint4& triVertIndices,
	const PxVec3& delta, const float4& invMassBary)
{
	if (invMassBary.x > 0.0f)
		AtomicAdd(outputDeltas[triVertIndices.x], delta*invMassBary.x);
	if (invMassBary.y > 0.0f)
		AtomicAdd(outputDeltas[triVertIndices.y], delta*invMassBary.y);
	if (invMassBary.z > 0.0f)
		AtomicAdd(outputDeltas[triVertIndices.z], delta*invMassBary.z);
}

static __device__ void updateDeltaTet(float4* outputDeltas, const uint4& tetVertIndices,
	const PxVec3& delta, const float4& invMassBary)
{
	if (invMassBary.x > 0.0f)
		AtomicAdd(outputDeltas[tetVertIndices.x], delta*invMassBary.x);
	if (invMassBary.y > 0.0f)
		AtomicAdd(outputDeltas[tetVertIndices.y], delta*invMassBary.y);
	if (invMassBary.z > 0.0f)
		AtomicAdd(outputDeltas[tetVertIndices.z], delta*invMassBary.z);
	if (invMassBary.w > 0.0f)
		AtomicAdd(outputDeltas[tetVertIndices.w], delta*invMassBary.w);
}

// Convert the prep-time barycentric float4 to the solver-side Vec form.
// SB (PxVec4): all four components pass through (bc.{x,y,z,w} are the four
// tet baries). Cloth (PxVec3): bc.w is the triangle-vs-vertex marker
// (0 = triangle/edge, 1 = vertex marker; the PxVec3 overload below
// dispatches on bc.w != 0, so any nonzero counts as vertex). Triangle keeps
// the bary; vertex collapses to (1, 0, 0), which the solver pairs with a
// matching vertexInvMasses = (perVertexInvMass, 0, 0) so the bc^2*invMass dot
// product reduces to the single vertex's invMass.
PX_FORCE_INLINE __device__ static void toSolverBc(PxVec4& out, const float4& bc)
{
	out = PxVec4(bc.x, bc.y, bc.z, bc.w);
}
PX_FORCE_INLINE __device__ static void toSolverBc(PxVec3& out, const float4& bc)
{
	out = (bc.w == 0.0f) ? PxVec3(bc.x, bc.y, bc.z) : PxVec3(1.0f, 0.0f, 0.0f);
}

// Pair-level state for a deformable contact: normal / tangent, rigid
// angular-jacobian rows raXn / raXt (= ra x normal, ra x tangent), and the
// accumulated + per-iter Lagrange multipliers.
// Operated on by solveRbDbContact and solveDbDbContact. Both also cover particle
// contacts, where a particle is represented as a single-vertex PxgDeformablePart.
// Pair-level contact geometry and prep inputs, read-only during the solve.
// Populated by readContactPrep (RB-DB / particle-rigid) or readContactPrepDbDb
// (DB-DB), or filled directly by the cloth-cloth callers.
struct PxgDbContactPair
{
	PxVec3 normal = PxVec3(0.0f);

	// tangent0 is the prep-time first friction basis (RB-DB only; the second basis
	// is normal.cross(tangent0) at solve time, and DB-DB derives its own tangent
	// from the tangential motion). initPen is the bc-weighted penetration error
	// baked at prep. The solves add both parts' accumulated motion to it to obtain
	// the current separation. maxPenBiasClamp is the pair-max depenetration-velocity
	// clamp (= -maxDepenetrationVelocity), baked at prep.
	PxVec3 tangent0;
	PxReal initPen = 0.0f;
	PxReal maxPenBiasClamp = 0.0f;

	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrep(const Block& block, PxU32 lane)
	{
		const float4 normal_errorW = block.normal_errorW[lane];
		normal = PxVec3(normal_errorW.x, normal_errorW.y, normal_errorW.z);
		initPen = normal_errorW.w;

		// fricTan0_invMass0.xyz packs the pair tangent. Its .w holds the rigid invMass and is
		// read separately by PxgRigidPart::readContactPrep.
		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[lane];
		tangent0 = PxVec3(fricTan0_invMass0.x, fricTan0_invMass0.y, fricTan0_invMass0.z);

		maxPenBiasClamp = block.maxPenBiasClamp[lane];
	}

	// Overload for the particle-rigid block: it carries only the rigid side's clamp
	// (penBiasClampRigid), so the caller passes the particle's clamp and the pair max is taken here.
	PX_FORCE_INLINE __device__ void readContactPrep(const PxgParticleRigidContactBlock& block,
													PxReal particlePenBiasClamp, PxU32 lane)
	{
		const float4 normal_errorW = block.normal_errorW[lane];
		normal = PxVec3(normal_errorW.x, normal_errorW.y, normal_errorW.z);
		initPen = normal_errorW.w;

		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[lane];
		tangent0 = PxVec3(fricTan0_invMass0.x, fricTan0_invMass0.y, fricTan0_invMass0.z);

		maxPenBiasClamp = PxMax(block.penBiasClampRigid[lane], particlePenBiasClamp);
	}

	// Two-sided (DB-DB) variant. PxgDbDbContactBlock packs the prep-time
	// penetration into normal_pen[lane].w and has no prep-time tangent0.
	// solveDbDbContact derives the friction tangent at solve time from the
	// tangential relative motion.
	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrepDbDb(const Block& block, PxU32 lane)
	{
		const float4 normal_pen = block.normal_pen[lane];
		normal = PxVec3(normal_pen.x, normal_pen.y, normal_pen.z);
		initPen = normal_pen.w;
		maxPenBiasClamp = block.maxPenBiasClamp[lane];
	}
};

// The solve's output: the normal / tangential multipliers, the solve-resolved
// friction direction (tangent) and its rigid angular-jacobian row (raXt), and the
// running normal-impulse accumulator. A fresh instance is default-zeroed, so an
// inactive or friction-free path leaves the untouched outputs at zero. The
// solves only seed the accumulator and write the rest on the active path.
struct PxgDbSolveOutput
{
	PxVec3 tangent = PxVec3(0.0f);
	PxVec3 raXt = PxVec3(0.0f);
	PxReal deltaLambdaN = 0.0f;
	PxReal deltaLambdaT = 0.0f;
	PxReal accumulatedDeltaLambdaN = 0.0f;

	// Convert the resolved normal/friction multipliers into the solve delta to
	// scatter via writeSoftBody / writeCloth / writeParticle: a position delta in
	// position iterations, velocity * dt in velocity iterations (the apply kernel
	// decides what it becomes). The rigid-side counterpart conversion lives in
	// PxgRigidPart::writeContactDeltas. Returns the accumulated normal multiplier
	// the caller pipes back into appliedForces. The normal comes from the pair the
	// solve read.
	PX_FORCE_INLINE __device__ PxReal computeDelta(const PxgDbContactPair& contactPair, PxVec3& delta, PxReal dt) const
	{
		delta = -(deltaLambdaN * contactPair.normal + deltaLambdaT * tangent) * dt;
		return accumulatedDeltaLambdaN;
	}
};

// The rigid side of a deformable contact/attachment (or particle contact)
// constraint. Carries the rigid body properties, the prep-time block reads, and
// the live rigid state.
// Operated on by solveRbDbContact, solveRbDbAttachment.
struct PxgRigidPart
{
	// Body properties (populated by readBodyProperties)
	// referenceCount is how many contacts share this rigid, looked up from the shared
	// refcount array by getGlobalRigidBodyId, so each articulation link counts on its own.
	// The contact writeback scales by it and the finalize divides it back out. Attachments
	// keep their own count in attachRefCount below.
	PxU32 referenceCount = 1;
	PxReal friction = 0.0f;
	PxI32 fricCombineMode;

	// Contact-prep cache (populated by readContactPrep)
	PxVec3 raXn;
	PxVec3 raXnF0;
	PxVec3 raXnF1;
	PxReal raXnResp = 0.0f;
	PxReal raXnF0Resp = 0.0f;
	PxReal raXnF1Resp = 0.0f;
	PxReal invMass = 0.0f;

	// Attachment-prep cache (populated by readAttachmentPrep)
	// Per-axis (X/Y/Z): three angular-jacobian rows ra x e_axis (where ra is
	// the lever from the rigid CoM to the attach point), three position
	// errors, three solver multipliers, plus rigid invMass and attachRefCount, the
	// attachment counterpart of referenceCount. It is baked into the prep block rather
	// than looked up, and counts attachments rather than contacts, so the two never mix.
	// The contact kernels do not read these fields.
	PxVec3 raXn0;
	PxVec3 raXn1;
	PxVec3 raXn2;
	PxVec3 positionErrorXYZ;
	PxVec3 velMultiplierXYZ;
	PxReal attachInvMass = 0.0f;
	PxU32  attachRefCount = 1;

	// Live rigid-body state (populated by readVelocity)
	// linVel / angVel: the current velocity.
	//
	// linDelta / angDelta: the accumulated position delta, zero on PGS where
	// the rigid does not move during the solve. Unlike PxgDeformablePart::linDelta
	// they never carry velocity * dt. In velocity iterations the rigid velocity
	// enters through linVel / angVel and the solves zero their delta terms
	// where only velocity may project.
	PxVec3 linVel;
	PxVec3 angVel;
	PxVec3 linDelta;
	PxVec3 angDelta;

	PX_FORCE_INLINE __device__ int getGlobalRigidBodyId(const PxgPrePrepDesc* const prePrepDesc, const PxNodeIndex& rigidId,
														PxU32 numSolverBodies, PxU32 maxLinksPerArticulation)
	{
		// Following PxgVelocityReader style to read rigid body indices.
		if(rigidId.isStaticBody())
		{
			return -1;
		}

		const PxU32 solverBodyIdx = prePrepDesc->solverBodyIndices[rigidId.index()];

		// Articulation buckets are keyed per-LINK (not per-articulation),
		// so that the reference count matches the per-link grouping used by
		// accumulateRigidDeltas. Otherwise, when multiple links of the same
		// articulation contact the same deformable, rigidRefCount double-counts
		// against the per-link averaging and the chain receives 2x the correct
		// per-iter impulse.
		// Layout: [0..numSolverBodies)        = rigid-body slots
		//         [numSolverBodies..end)      = numArticulations * maxLinksPerArticulation slots
		return rigidId.isArticulation()
				 ? static_cast<int>(numSolverBodies + solverBodyIdx * maxLinksPerArticulation + rigidId.articulationLinkId())
				 : static_cast<int>(solverBodyIdx);
	}

	PX_FORCE_INLINE __device__ void readBodyProperties(const PxNodeIndex& rigidId, int globalRigidBodyId, PxReal rigidInvMass,
										 const PxU32* const rigidBodyRefCounts, const PxsMaterialData* rigidMaterial)
	{
		referenceCount = 1;

		// Query the reference count for the rigid body.
		if(rigidBodyRefCounts && globalRigidBodyId != -1 && rigidInvMass != 0.0f)
		{
			referenceCount = rigidBodyRefCounts[globalRigidBodyId];
		}

		if(rigidMaterial != NULL)
		{
			friction = rigidMaterial->dynamicFriction;
			fricCombineMode = rigidMaterial->fricCombineMode;
		}
		else
		{
			friction = 0.0f;
			fricCombineMode = PxCombineMode::eMAX;
		}
	}

	// Stand-in for readBodyProperties + readContactPrep + readVelocity on the particle-vs-static-mesh
	// (one-way) contact, which has no rigid node. writeContactDeltas is never called for it.
	PX_FORCE_INLINE __device__ void readNodelessStatic()
	{
		referenceCount = 1;
		// No rigid material on a static mesh: mirror readBodyProperties' no-material branch.
		friction = 0.0f;
		fricCombineMode = PxCombineMode::eMAX;
		raXn = raXnF0 = raXnF1 = PxVec3(0.0f);
		raXnResp = raXnF0Resp = raXnF1Resp = 0.0f;
		invMass = 0.0f;              // static -> infinite mass (the writeContactDeltas invMass==0 no-op sentinel)
		linVel = angVel = linDelta = angDelta = PxVec3(0.0f);
	}

	PX_FORCE_INLINE __device__ void writeDeltas(float4* rigidDeltaVel, const PxVec3& deltaLinVel, const PxVec3& deltaAngVel,
												PxU32 linVelIndex, PxU32 angVelIndex, PxReal count)
	{
		rigidDeltaVel[linVelIndex] = make_float4(deltaLinVel.x, deltaLinVel.y, deltaLinVel.z, count);
		rigidDeltaVel[angVelIndex] = make_float4(deltaAngVel.x, deltaAngVel.y, deltaAngVel.z, 0.f);
	}

	// Attachment rigid writeback: skip static bodies, zero kinematic
	// (attachInvMass == 0), else write attachRefCount-inflated deltas (count 1
	// so accumulateRigidDeltas' sum-then-average recovers the total impulse).
	PX_FORCE_INLINE __device__ void writeAttachmentDeltas(float4* rigidDeltaVel, const PxNodeIndex& rigidId, const PxVec3& deltaLinVel,
														  const PxVec3& deltaAngVel, PxU32 linVelIndex, PxU32 angVelIndex)
	{
		if(rigidId.isStaticBody())
			return;

		if(attachInvMass == 0.0f)
		{
			writeDeltas(rigidDeltaVel, PxVec3(0.0f), PxVec3(0.0f), linVelIndex, angVelIndex, 0.0f);
		}
		else
		{
			const PxReal refN = static_cast<PxReal>(attachRefCount);
			writeDeltas(rigidDeltaVel, deltaLinVel * refN, deltaAngVel * refN, linVelIndex, angVelIndex, 1.0f);
		}
	}

	// Contact rigid writeback. Always writes, unlike writeAttachmentDeltas: the
	// accumulate scan reads every slot each iteration, so a dynamic rigid's slot
	// must be redefined every time. invMass is populated only on the active+dynamic
	// path (readContactPrep), so invMass == 0 collapses static / kinematic / inactive
	// into a zeroed count-0 no-op. Otherwise convert the resolved multipliers to a
	// rigid velocity delta and write with count 1.
	//
	// refCount scales lin AND ang symmetrically: the finalize divides both by
	// max(1, .w) and the attachment writeback inflates both, so the contact path
	// must match. Per-link bucketing of articulation referenceCount
	// (getGlobalRigidBodyId) keeps refCount equal to the per-link write count,
	// matching the per-link finalize divisor.
	PX_FORCE_INLINE __device__ void writeContactDeltas(float4* rigidDeltaVel, const PxNodeIndex& rigidId,
													   const PxgDbContactPair& contactPair, const PxgDbSolveOutput& solveOut,
													   PxU32 linVelIndex, PxU32 angVelIndex)
	{
		PxVec3 deltaLinVel(0.0f), deltaAngVel(0.0f);
		PxReal count = 0.0f;
		if(invMass != 0.0f && !rigidId.isStaticBody())
		{
			count = 1.0f;
			const PxReal refCount = static_cast<PxReal>(referenceCount);
			// raXn is the rigid's own normal angular-jacobian row (this->raXn). raXt is
			// the solve-blended friction row (solveOut.raXt).
			deltaAngVel = (raXn * solveOut.deltaLambdaN + solveOut.raXt * solveOut.deltaLambdaT) * refCount;
			deltaLinVel = (contactPair.normal * solveOut.deltaLambdaN + solveOut.tangent * solveOut.deltaLambdaT) * invMass * refCount;
		}
		writeDeltas(rigidDeltaVel, deltaLinVel, deltaAngVel, linVelIndex, angVelIndex, count);
	}

	// Pulls the rigid-side prep cache out of the warp-packed constraint block: the
	// angular-jacobian rows ra x {n, t0, t1}, their unit responses, and invMass.
	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrep(const Block& block, PxU32 lane)
	{
		const float4 raXn_resp = block.raXn_resp[lane];
		raXn = PxVec3(raXn_resp.x, raXn_resp.y, raXn_resp.z);
		raXnResp = raXn_resp.w;

		const float4 raXnF0_resp = block.raXnF0_resp[lane];
		raXnF0 = PxVec3(raXnF0_resp.x, raXnF0_resp.y, raXnF0_resp.z);
		raXnF0Resp = raXnF0_resp.w;

		const float4 raXnF1_resp = block.raXnF1_resp[lane];
		raXnF1 = PxVec3(raXnF1_resp.x, raXnF1_resp.y, raXnF1_resp.z);
		raXnF1Resp = raXnF1_resp.w;

		invMass = block.fricTan0_invMass0[lane].w;
	}

	// Pulls the attachment-side rigid prep cache: per-axis angular-jacobian
	// rows ra x e_axis, per-axis position errors, per-axis solver multipliers,
	// rigid invMass, and the attachment-bucket refCount.
	template <typename Block>
	PX_FORCE_INLINE __device__ void readAttachmentPrep(const Block& block, PxU32 lane)
	{
		const float4 raXn0_biasW = block.raXn0_biasW[lane];
		const float4 raXn1_biasW = block.raXn1_biasW[lane];
		const float4 raXn2_biasW = block.raXn2_biasW[lane];
		raXn0 = PxVec3(raXn0_biasW.x, raXn0_biasW.y, raXn0_biasW.z);
		raXn1 = PxVec3(raXn1_biasW.x, raXn1_biasW.y, raXn1_biasW.z);
		raXn2 = PxVec3(raXn2_biasW.x, raXn2_biasW.y, raXn2_biasW.z);
		positionErrorXYZ = PxVec3(raXn0_biasW.w, raXn1_biasW.w, raXn2_biasW.w);

		const float4 velMultXYZ_invMassW = block.velMultiplierXYZ_invMassW[lane];
		velMultiplierXYZ = PxVec3(velMultXYZ_invMassW.x, velMultXYZ_invMassW.y, velMultXYZ_invMassW.z);
		attachInvMass = velMultXYZ_invMassW.w;

		attachRefCount = block.rigidBodyRefCount[lane];
	}

	// Reads live rigid velocity and, on TGS, the accumulated position delta
	// (readVelocitiesPGS zeros linDelta / angDelta itself).
	PX_FORCE_INLINE __device__ void readVelocity(PxgVelocityReader& reader, PxNodeIndex rigidId, bool isTGS)
	{
		if (isTGS)
			reader.readVelocitiesTGS(rigidId, linVel, angVel, linDelta, angDelta);
		else
			reader.readVelocitiesPGS(rigidId, linVel, angVel, linDelta, angDelta);
	}

};

// Particle linDelta for a db-particle contact: the accumulated position delta
// (mSortedDeltaP), or in velocity mode the sorted velocity (mSortedVelocities)
// scaled by dt so the solve's invDt recovers the point velocity. invMass stays
// in mSortedDeltaP.w and is read separately by the caller.
PX_FORCE_INLINE __device__ PxVec3 readParticleLinDelta(const PxgParticleSystem& particleSystem, PxU32 particleIndex,
													   const float4& sortedDeltaP, PxReal dt, bool isVelocityIteration)
{
	if(isVelocityIteration)
	{
		const float4 vel = particleSystem.mSortedVelocities[particleIndex];
		return PxVec3(vel.x, vel.y, vel.z) * dt;
	}
	return PxVec3(sortedDeltaP.x, sortedDeltaP.y, sortedDeltaP.z);
}

// The deformable/particle side of a contact or attachment constraint. Carries
// the per-vertex invMasses, the bc-weighted deltas, and per-type read/write
// helpers: cloth (PxVec3, triangle or single vertex), soft body (PxVec4, tet),
// particle (PxVec3, a single vertex with bc = (1,0,0), contacts only). An
// instantiation only ever calls the helpers of its own body type.
// Operated on by solveRbDbContact, solveDbDbContact, solveRbDbAttachment,
// solveDbDbAttachment.
template <typename Vec>
struct PxgDeformablePart
{
	PxReal friction = 0.0f;

	// Particle-side adhesion, read only by the withAdhesion instantiations, which are the
	// particle-rigid contacts. It is 0 for deformables. adhesionRadius = restOffset *
	// (adhesionRadiusScale - 1), the characteristic falloff distance of the t^4 adhesion
	// kernel (see particleAdhesionImpulse).
	PxReal adhesion = 0.0f;
	PxReal adhesionRadius = 0.0f;

	// The accumulated position delta in position iterations, or velocity * dt in velocity iterations, so
	// the solve's invDt recovers the point velocity.
	PxVec3 linDelta;

	// The accumulated position delta in both iteration modes. In position iterations it equals
	// linDelta. The solves read it for computing the separation. A manual-fill caller supplies the delta
	// matching its initPen baseline (cloth-cloth VT/EE uses the motion since the last narrowphase
	// refresh).
	PxVec3 posDelta = PxVec3(0.0f);
	Vec vertexInvMasses; // refCount-inflated. TODO: rename to vertexInvMassesSplit

	// Contact-prep cache (populated by readContactPrep)
	// `bc` is the solver-side bary (cloth-vertex collapses to (1,0,0) via
	// toSolverBc); used in solveRbDbContact for the bc^2*invMass denominator.
	Vec bc;

	// Attachment-prep cache, populated by readSoftBodyAttachment / readClothAttachment for
	// RB-DB attachments, or readSoftBodyAttachmentDbDb / readClothAttachmentDbDb for DB-DB
	// attachments.
	// attachPointInvMass:      raw bc^2*invMass (no refCount inflation). Used by
	//                          solveRbDbAttachment in the denomBias formula
	//                          (refN * raw - split), and by the RB-DB kernels'
	//                          isStaticBody kinematic-skip gate. Set by RB-DB
	//                          readers only. DB-DB readers leave it at 0.
	// attachPointInvMassSplit: refCount-inflated bc^2*invMass (= bc.multiply(bc)
	//                          .dot(vertexInvMasses) once vertexInvMasses are
	//                          refCount-inflated). Used by solveDbRigidAttachment
	//                          NT for denomBias's other term, and by
	//                          solveDbDbAttachment as the per-side wTot input.
	//                          Set by both RB-DB and DB-DB readers.
	PxReal attachPointInvMass = 0.0f;
	PxReal attachPointInvMassSplit = 0.0f;

	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrep(const Block& block, PxU32 lane)
	{
		toSolverBc(bc, block.barycentric[lane]);
	}

	// Side-tagged variants for two-sided blocks (PxgDbDbContactBlock), where each
	// side has its own barycentric slot. (The pair maxPenBiasClamp is read once
	// into PxgDbContactPair by readContactPrepDbDb.)
	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrepDbSide0(const Block& block, PxU32 lane)
	{
		toSolverBc(bc, block.barycentric0[lane]);
	}

	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrepDbSide1(const Block& block, PxU32 lane)
	{
		toSolverBc(bc, block.barycentric1[lane]);
	}

	// linDelta is the accumulated position delta, or in velocity mode the velocity
	// (mVelocity_InvMass) scaled by dt so the solve's invDt recovers the point velocity.
	PX_FORCE_INLINE __device__ PxVec3 readCloth(const PxgFEMCloth& cloth, PxU32 elementId, const float4& bc,
												const PxsDeformableSurfaceMaterialData* const materials, bool countReferenceOnly,
												PxReal dt, bool isVelocityIteration)
	{
		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			vertexInvMasses = PxVec3(0.0f);
			return vertexInvMasses;
		}

		const float4* const PX_RESTRICT posDeltasOrVelocities = isVelocityIteration ? cloth.mVelocity_InvMass : cloth.mAccumulatedDeltaPos;
		float4 posDeltaOrVelocity;

		if(bc.w == 0) // Cloth triangle
		{
			const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
			const float4 pdv0 = posDeltasOrVelocities[triVertId.x];
			const float4 pdv1 = posDeltasOrVelocities[triVertId.y];
			const float4 pdv2 = posDeltasOrVelocities[triVertId.z];

			posDeltaOrVelocity = pdv0 * bc.x + pdv1 * bc.y + pdv2 * bc.z;
			vertexInvMasses = PxVec3(pdv0.w, pdv1.w, pdv2.w);

			if(isVelocityIteration)
			{
				// Accumulated position delta, provided separately from the velocity to supply the
				// geometric separation for velocity iterations.
				const float4 accumulatedDelta = cloth.mAccumulatedDeltaPos[triVertId.x] * bc.x +
								  cloth.mAccumulatedDeltaPos[triVertId.y] * bc.y +
								  cloth.mAccumulatedDeltaPos[triVertId.z] * bc.z;
				posDelta = PxVec3(accumulatedDelta.x, accumulatedDelta.y, accumulatedDelta.z);
			}

			if(!countReferenceOnly)
			{
				const PxU16 globalMaterialIndex = cloth.mMaterialIndices[elementId];
				friction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

				// Query the reference count for the cloth.
				PxVec3 vertexRefCount;
				vertexRefCount.x = cloth.mDeltaPos[triVertId.x].w;
				vertexRefCount.y = cloth.mDeltaPos[triVertId.y].w;
				vertexRefCount.z = cloth.mDeltaPos[triVertId.z].w;

				// Mass-splitting
				vertexInvMasses = vertexInvMasses.multiply(vertexRefCount);
			}
		}
		else // Cloth vertex
		{
			posDeltaOrVelocity = posDeltasOrVelocities[elementId];
			vertexInvMasses = PxVec3(posDeltaOrVelocity.w, 0.0f, 0.0f);

			if(isVelocityIteration)
			{
				// Accumulated position delta, provided separately from the velocity to supply the
				// geometric separation for velocity iterations.
				const float4 accumulatedDelta = cloth.mAccumulatedDeltaPos[elementId];
				posDelta = PxVec3(accumulatedDelta.x, accumulatedDelta.y, accumulatedDelta.z);
			}

			if(!countReferenceOnly)
			{
				friction = materials ? cloth.mDynamicFrictions[elementId] : 0.0f;

				// Query the reference count for the cloth.
				const PxReal vertexRefCount = cloth.mDeltaPos[elementId].w;

				// Mass-splitting
				vertexInvMasses.x *= vertexRefCount;
			}
		}

		linDelta = PxVec3(posDeltaOrVelocity.x, posDeltaOrVelocity.y, posDeltaOrVelocity.z);
		if(isVelocityIteration)
			linDelta *= dt;
		else
			posDelta = linDelta; // in position iterations the accumulated delta is linDelta

		return vertexInvMasses;
	}

	// linDelta is the accumulated position delta, or in velocity mode the velocity
	// (mSimVelocity_InvMass) scaled by dt so the solve's invDt recovers the point velocity.
	PX_FORCE_INLINE __device__ PxVec4 readSoftBody(const PxgSoftBody& softbody, PxU32 tetId, const float4& bc,
												 const PxsDeformableVolumeMaterialData* const materials, bool checkOnlyActivity,
												 PxReal dt, bool isVelocityIteration)
	{
		if(tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			vertexInvMasses = PxVec4(0.0f);
			return vertexInvMasses;
		}

		const float4* const PX_RESTRICT posDeltasOrVelocities = isVelocityIteration ? softbody.mSimVelocity_InvMass : softbody.mSimDeltaPos;

		const uint4 tetVertId = softbody.mSimTetIndices[tetId];
		const float4 pdv0 = posDeltasOrVelocities[tetVertId.x];
		const float4 pdv1 = posDeltasOrVelocities[tetVertId.y];
		const float4 pdv2 = posDeltasOrVelocities[tetVertId.z];
		const float4 pdv3 = posDeltasOrVelocities[tetVertId.w];

		const float4 posDeltaOrVelocity = pdv0 * bc.x + pdv1 * bc.y + pdv2 * bc.z + pdv3 * bc.w;
		linDelta = PxVec3(posDeltaOrVelocity.x, posDeltaOrVelocity.y, posDeltaOrVelocity.z);

		if(isVelocityIteration)
		{
			linDelta *= dt;
			// Accumulated position delta, provided separately from the velocity to supply the
			// geometric separation for velocity iterations.
			const float4* const PX_RESTRICT simDeltaPos = softbody.mSimDeltaPos;
			const float4 accumulatedDelta = simDeltaPos[tetVertId.x] * bc.x + simDeltaPos[tetVertId.y] * bc.y +
							  simDeltaPos[tetVertId.z] * bc.z + simDeltaPos[tetVertId.w] * bc.w;
			posDelta = PxVec3(accumulatedDelta.x, accumulatedDelta.y, accumulatedDelta.z);
		}
		else
		{
			posDelta = linDelta; // in position iterations the accumulated delta is linDelta
		}

		vertexInvMasses = PxVec4(pdv0.w, pdv1.w, pdv2.w, pdv3.w);

		if(!checkOnlyActivity)
		{
			const PxU16 globalMaterialIndex = softbody.mMaterialIndices[tetId];
			friction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

			// Query the reference count for soft body.
			PxVec4 vertexRefCount;
			vertexRefCount.x = softbody.mSimDelta[tetVertId.x].w;
			vertexRefCount.y = softbody.mSimDelta[tetVertId.y].w;
			vertexRefCount.z = softbody.mSimDelta[tetVertId.z].w;
			vertexRefCount.w = softbody.mSimDelta[tetVertId.w].w;

			// Mass-splitting
			vertexInvMasses = vertexInvMasses.multiply(vertexRefCount);
		}

		return vertexInvMasses;
	}

	// Particle read for the contact solves, the single-vertex analog of readCloth /
	// readSoftBody: bc = (1,0,0), vertexInvMasses from sortedDeltaP.w (inflated by
	// the mass-split contact count when inflateByRefCount is set, so both halves of
	// a split solve compute the same lambda), friction and adhesion from the
	// material (NULL skips them, as in the query kernels), linDelta and posDelta
	// from the particle buffers. linDelta holds velocity * dt in velocity
	// iterations and the accumulated position delta otherwise. posDelta always
	// holds the accumulated position delta.
	PX_FORCE_INLINE __device__ void readParticle(const PxgParticleSystem& particleSystem, PxU32 particleIndex,
												 const float4& sortedDeltaP, const PxsParticleMaterialData* mat,
												 bool inflateByRefCount, PxReal dt, bool isVelocityIteration)
	{
		bc = PxVec3(1.0f, 0.0f, 0.0f);

		// Deformable-particle mass-splitting: the pre-count leaves the particle's contact count in
		// .w, and inflating by it keeps a grain touching several elements from over-applying.
		const PxReal refCount = inflateByRefCount ? PxMax(particleSystem.mAccumDeltaP[particleIndex].w, 1.0f) : 1.0f;
		vertexInvMasses = PxVec3(sortedDeltaP.w * refCount, 0.0f, 0.0f);

		if(mat)
		{
			friction = mat->friction;
			adhesion = mat->adhesion;
			adhesionRadius = particleSystem.mData.mRestOffset * (mat->adhesionRadiusScale - 1.0f);
		}

		linDelta = readParticleLinDelta(particleSystem, particleIndex, sortedDeltaP, dt, isVelocityIteration);
		posDelta = PxVec3(sortedDeltaP.x, sortedDeltaP.y, sortedDeltaP.z);
	}

	// Attachment-side reads. Differ from readSoftBody / readCloth in:
	//  - Source buffer is PGS velocity (mSimVelocity_InvMass / mVelocity_InvMass)
	//    or TGS accumulated delta-pos (mSimDeltaPos / mAccumulatedDeltaPos).
	//  - `bary` and `elemIsVertex` MUST be derived from the same source.
	//    Callers compute `elemIsVertex = PxGetIsVertexType(bary)`, which
	//    tests bary == (0,0,0,0), the all-zeros sentinel that marks a vertex
	//    attachment, and pass both. `bary` is then either consulted as the
	//    element-side barycentric (element branch) or unread (vertex branch).
	//    Passing inconsistent values silently produces zero-effect reads
	//    (`elemIsVertex=false` + zeros bary) or drops the bary
	//    (`elemIsVertex=true` + non-zero bary).
	//  - vertexInvMasses are refCount-inflated from .w (pre-counted), so the
	//    scatter pairs with the finalize-time /.w divide.
	//  - attachPointInvMass      = raw bc^2*invM (D in denomBias formula).
	//    attachPointInvMassSplit = refCount-inflated bc^2*invM (D' in denomBias).

	PX_FORCE_INLINE __device__ void readSoftBodyAttachment(const PxgSoftBody& softbody, PxU32 elemIdx,
														   const float4& bary, bool elemIsVertex, bool isTGS,
														   PxReal dt, bool isVelocityIteration)
	{
		// Source: PGS always the velocity buffer; TGS the accumulated delta, except
		// in velocity iterations where it reads the velocity buffer (scaled by dt below).
		const float4* const PX_RESTRICT posDeltasOrVelocities = (isTGS && !isVelocityIteration) ? softbody.mSimDeltaPos : softbody.mSimVelocity_InvMass;
		const float4* const PX_RESTRICT simDelta = softbody.mSimDelta;

		if(elemIsVertex)
		{
			const float4 posDeltaOrVelocity = posDeltasOrVelocities[elemIdx];
			const float refCount_v = simDelta[elemIdx].w;
			linDelta = PxVec3(posDeltaOrVelocity.x, posDeltaOrVelocity.y, posDeltaOrVelocity.z);
			vertexInvMasses = PxVec4(posDeltaOrVelocity.w * refCount_v, 0.0f, 0.0f, 0.0f);
			attachPointInvMass      = posDeltaOrVelocity.w;
			attachPointInvMassSplit = posDeltaOrVelocity.w * refCount_v;
			bc = PxVec4(1.0f, 0.0f, 0.0f, 0.0f);
		}
		else
		{
			const uint4 tetVertId = softbody.mSimTetIndices[elemIdx];
			const float4 pdv0 = posDeltasOrVelocities[tetVertId.x];
			const float4 pdv1 = posDeltasOrVelocities[tetVertId.y];
			const float4 pdv2 = posDeltasOrVelocities[tetVertId.z];
			const float4 pdv3 = posDeltasOrVelocities[tetVertId.w];
			const float4 posDeltaOrVelocity = pdv0 * bary.x + pdv1 * bary.y + pdv2 * bary.z + pdv3 * bary.w;
			linDelta = PxVec3(posDeltaOrVelocity.x, posDeltaOrVelocity.y, posDeltaOrVelocity.z);

			const PxReal r0 = simDelta[tetVertId.x].w;
			const PxReal r1 = simDelta[tetVertId.y].w;
			const PxReal r2 = simDelta[tetVertId.z].w;
			const PxReal r3 = simDelta[tetVertId.w].w;

			vertexInvMasses = PxVec4(pdv0.w * r0, pdv1.w * r1, pdv2.w * r2, pdv3.w * r3);

			const PxReal bx2 = bary.x * bary.x;
			const PxReal by2 = bary.y * bary.y;
			const PxReal bz2 = bary.z * bary.z;
			const PxReal bw2 = bary.w * bary.w;
			attachPointInvMass      = bx2 * pdv0.w      + by2 * pdv1.w      + bz2 * pdv2.w      + bw2 * pdv3.w;
			attachPointInvMassSplit = bx2 * pdv0.w * r0 + by2 * pdv1.w * r1 + bz2 * pdv2.w * r2 + bw2 * pdv3.w * r3;
			bc = PxVec4(bary.x, bary.y, bary.z, bary.w);
		}

		if(isTGS && isVelocityIteration)
			linDelta *= dt;
	}

	PX_FORCE_INLINE __device__ void readClothAttachment(const PxgFEMCloth& cloth, PxU32 elemIdx,
														const float4& bary, bool elemIsVertex, bool isTGS,
														PxReal dt, bool isVelocityIteration)
	{
		const float4* const PX_RESTRICT posDeltasOrVelocities = (isTGS && !isVelocityIteration) ? cloth.mAccumulatedDeltaPos : cloth.mVelocity_InvMass;
		const float4* const PX_RESTRICT delta = cloth.mDeltaPos;

		if(elemIsVertex)
		{
			const float4 posDeltaOrVelocity = posDeltasOrVelocities[elemIdx];
			const float refCount_v = delta[elemIdx].w;
			linDelta = PxVec3(posDeltaOrVelocity.x, posDeltaOrVelocity.y, posDeltaOrVelocity.z);
			vertexInvMasses = PxVec3(posDeltaOrVelocity.w * refCount_v, 0.0f, 0.0f);
			attachPointInvMass      = posDeltaOrVelocity.w;
			attachPointInvMassSplit = posDeltaOrVelocity.w * refCount_v;
			bc = PxVec3(1.0f, 0.0f, 0.0f);
		}
		else
		{
			const uint4 triVertId = cloth.mTriangleVertexIndices[elemIdx];
			const float4 pdv0 = posDeltasOrVelocities[triVertId.x];
			const float4 pdv1 = posDeltasOrVelocities[triVertId.y];
			const float4 pdv2 = posDeltasOrVelocities[triVertId.z];
			const float4 posDeltaOrVelocity = pdv0 * bary.x + pdv1 * bary.y + pdv2 * bary.z;
			linDelta = PxVec3(posDeltaOrVelocity.x, posDeltaOrVelocity.y, posDeltaOrVelocity.z);

			const PxReal r0 = delta[triVertId.x].w;
			const PxReal r1 = delta[triVertId.y].w;
			const PxReal r2 = delta[triVertId.z].w;

			vertexInvMasses = PxVec3(pdv0.w * r0, pdv1.w * r1, pdv2.w * r2);

			const PxReal bx2 = bary.x * bary.x;
			const PxReal by2 = bary.y * bary.y;
			const PxReal bz2 = bary.z * bary.z;
			attachPointInvMass      = bx2 * pdv0.w      + by2 * pdv1.w      + bz2 * pdv2.w;
			attachPointInvMassSplit = bx2 * pdv0.w * r0 + by2 * pdv1.w * r1 + bz2 * pdv2.w * r2;
			bc = PxVec3(bary.x, bary.y, bary.z);
		}

		if(isTGS && isVelocityIteration)
			linDelta *= dt;
	}

	// DB-DB attachment read.
	//
	// Position iteration reads the live position buffer, so the caller computes
	// error = pos1 - pos0 directly. Velocity iteration reads the velocity buffer and
	// scales by dt, so error = (vel1 - vel0) * dt projects the relative attachment-point
	// velocity (the velocity-only finalize then does vel += delta * invDt).
	//
	// vertexInvMasses + attachPointInvMassSplit are refCount-inflated: the `.w`
	// reference count is pre-counted into mSimDelta[v].w / mDeltaPos[v].w by the DB-DB
	// attachment reference-count kernels (the same in position and velocity iterations),
	// and read from that buffer's .w (invMass is identical in the position and velocity buffers).
	//
	// The later delta scatter writes only .xyz, leaving .w for the finalize step
	// (which applies the delta and zeroes .w).
	//
	// attachPointInvMass (raw) is left at 0, because DB-DB attachments do not need it.
	PX_FORCE_INLINE __device__ PxVec3 readSoftBodyAttachmentDbDb(const PxgSoftBody& softbody, PxU32 tetId, const float4& bary,
																 PxReal dt, bool isVelocityIteration)
	{
		const float4* const PX_RESTRICT positionsOrVelocities = isVelocityIteration ? softbody.mSimVelocity_InvMass : softbody.mSimPosition_InvMass;
		const float4* const PX_RESTRICT simDelta = softbody.mSimDelta;
		const uint4 tetVertId = softbody.mSimTetIndices[tetId];
		const float4 pv0 = positionsOrVelocities[tetVertId.x];
		const float4 pv1 = positionsOrVelocities[tetVertId.y];
		const float4 pv2 = positionsOrVelocities[tetVertId.z];
		const float4 pv3 = positionsOrVelocities[tetVertId.w];
		const float4 positionOrVelocity = pv0 * bary.x + pv1 * bary.y + pv2 * bary.z + pv3 * bary.w;

		const PxReal r0 = simDelta[tetVertId.x].w;
		const PxReal r1 = simDelta[tetVertId.y].w;
		const PxReal r2 = simDelta[tetVertId.z].w;
		const PxReal r3 = simDelta[tetVertId.w].w;

		vertexInvMasses = PxVec4(pv0.w * r0, pv1.w * r1, pv2.w * r2, pv3.w * r3);
		bc = PxVec4(bary.x, bary.y, bary.z, bary.w);

		const PxReal bx2 = bary.x * bary.x;
		const PxReal by2 = bary.y * bary.y;
		const PxReal bz2 = bary.z * bary.z;
		const PxReal bw2 = bary.w * bary.w;
		attachPointInvMassSplit = bx2 * pv0.w * r0 + by2 * pv1.w * r1 + bz2 * pv2.w * r2 + bw2 * pv3.w * r3;

		PxVec3 result(positionOrVelocity.x, positionOrVelocity.y, positionOrVelocity.z);
		if(isVelocityIteration)
			result *= dt;
		return result;
	}

	PX_FORCE_INLINE __device__ PxVec3 readClothAttachmentDbDb(const PxgFEMCloth& cloth, PxU32 elementId, const float4& bary,
															 PxReal dt, bool isVelocityIteration)
	{
		const float4* const PX_RESTRICT positionsOrVelocities = isVelocityIteration ? cloth.mVelocity_InvMass : cloth.mPosition_InvMass;
		const float4* const PX_RESTRICT delta = cloth.mDeltaPos;
		const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
		const float4 pv0 = positionsOrVelocities[triVertId.x];
		const float4 pv1 = positionsOrVelocities[triVertId.y];
		const float4 pv2 = positionsOrVelocities[triVertId.z];
		const float4 positionOrVelocity = pv0 * bary.x + pv1 * bary.y + pv2 * bary.z;

		const PxReal r0 = delta[triVertId.x].w;
		const PxReal r1 = delta[triVertId.y].w;
		const PxReal r2 = delta[triVertId.z].w;

		vertexInvMasses = PxVec3(pv0.w * r0, pv1.w * r1, pv2.w * r2);
		bc = PxVec3(bary.x, bary.y, bary.z);

		const PxReal bx2 = bary.x * bary.x;
		const PxReal by2 = bary.y * bary.y;
		const PxReal bz2 = bary.z * bary.z;
		attachPointInvMassSplit = bx2 * pv0.w * r0 + by2 * pv1.w * r1 + bz2 * pv2.w * r2;

		PxVec3 result(positionOrVelocity.x, positionOrVelocity.y, positionOrVelocity.z);
		if(isVelocityIteration)
			result *= dt;
		return result;
	}

	// Writebacks, shared by the contact and attachment solves.

	// Cloth-side writeback: scatter the solve delta into the cloth's per-iteration
	// scratch (mDeltaPos), weighted per vertex by bary * vertexInvMasses.
	// elemIsVertex selects the scatter: a single vertex (elementId is the vertex
	// index, bc unread) or a triangle (elementId indexes mTriangleVertexIndices,
	// bc.xyz are the barycentric weights).
	PX_FORCE_INLINE __device__ void writeCloth(PxgFEMCloth& cloth, PxU32 elementId, const float4& bc, bool elemIsVertex, PxVec3 delta)
	{
		if(elemIsVertex)
			updateDeltaVtx(cloth.mDeltaPos, elementId, delta, vertexInvMasses.x);
		else
			updateDeltaTri(cloth.mDeltaPos, cloth.mTriangleVertexIndices[elementId], delta,
				make_float4(vertexInvMasses.x * bc.x, vertexInvMasses.y * bc.y, vertexInvMasses.z * bc.z, 0.0f));
	}

	// Edge variant for the CC EE solver, where each side is a cloth edge.
	PX_FORCE_INLINE __device__ void writeClothEdge(PxgFEMCloth& cloth, PxU32 vertIdx0, PxU32 vertIdx1, PxVec3 delta)
	{
		updateDeltaEdge(cloth.mDeltaPos, vertIdx0, vertIdx1, delta,
			vertexInvMasses.x * bc.x, vertexInvMasses.y * bc.y);
	}

	// Softbody-side writeback: scatter the solve delta into the softbody's
	// per-iteration scratch (mSimDelta), weighted per vertex by bary *
	// vertexInvMasses. elemIsVertex selects the scatter: a single vertex
	// (elementId is the vertex index, bc unread) or a tet (elementId indexes
	// mSimTetIndices, bc.xyzw are the barycentric weights).
	PX_FORCE_INLINE __device__ void writeSoftBody(const PxgSoftBody& softbody, PxU32 elementId, const float4& bc, bool elemIsVertex, PxVec3 delta)
	{
		if(elemIsVertex)
			updateDeltaVtx(softbody.mSimDelta, elementId, delta, vertexInvMasses.x);
		else
			updateDeltaTet(softbody.mSimDelta, softbody.mSimTetIndices[elementId], delta,
				make_float4(vertexInvMasses.x * bc.x, vertexInvMasses.y * bc.y, vertexInvMasses.z * bc.z, vertexInvMasses.w * bc.w));
	}

	// Particle-side contact writeback, the single-vertex analog of writeSoftBody /
	// writeCloth: scale the solve delta by the particle invMass and store it in the
	// per-contact slot, with .w flagging a live contact. An accumulation kernel
	// (ps_accumulateFEMParticleDeltaVLaunch for SP/CP, ps_accumulateDeltaVParticleLaunch
	// for PC) sums the slots per particle into mAccumDeltaP.
	PX_FORCE_INLINE __device__ void writeParticle(float4* deltas, PxU32 slot, const PxVec3& delta,
												  const PxgDbSolveOutput& solveOut) const
	{
		const PxVec3 scaled = delta * vertexInvMasses.x;
		const PxReal applied = (solveOut.deltaLambdaN != 0.0f || solveOut.deltaLambdaT != 0.0f) ? 1.0f : 0.0f;
		deltas[slot] = make_float4(scaled.x, scaled.y, scaled.z, applied);
	}

};

// Particle adhesion as an attractive normal impulse (<= 0): within the adhesion radius it holds the
// particle to the collider. Returns 0 in velocity iterations or when the material has no adhesion. Used
// by solveRbDbContact's withAdhesion instantiation (particle-rigid and particle-static). Deformable pairs
// compile it out (withAdhesion == false).
PX_FORCE_INLINE __device__ PxReal particleAdhesionImpulse(
	PxReal separation, PxReal adhesion, PxReal adhesionRadius,
	PxReal velMultiplier, PxReal invDt, bool isVelocityIteration)
{
	// adhesionRadius = restOffset * (adhesionRadiusScale - 1) is the characteristic falloff distance of the
	// t^4 adhesion kernel, NOT a hard cutoff. The contact-generation range bounds separation (and PxMax(0,
	// separation) zeroes the pull once penetrating). Gate only on dimensionally-clean conditions: no velocity
	// iters, positive adhesion (force), positive radius (distance; rejects adhesionRadiusScale <= 1, which
	// would divide by <= 0).
	if (isVelocityIteration || adhesion <= 0.f || adhesionRadius <= 0.f)
		return 0.f;
	const PxReal t = 1.f - separation / adhesionRadius;
	return -PxMax(0.f, adhesion * (t * t * t * t) * PxMax(0.f, separation) * velMultiplier * invDt);
}

/**
\brief Solves one deformable/particle-vs-rigid contact for the normal and tangential (friction) multipliers.

\tparam			Vec						Deformable participant layout: PxVec3 for a cloth vertex or particle, PxVec4 for
										a soft-body tetrahedron.
\tparam			withAdhesion			Enables the particle-material adhesion lower bound; set only for particle-rigid
										contacts.

\param[in]		rigid					Rigid participant. linDelta/angDelta hold its accumulated position delta
										(zero on PGS). In velocity iterations the projection and friction terms
										zero their rigid delta contribution and see the velocity (linVel/angVel)
										only.
\param[in]		db						Deformable or particle participant, carrying two deltas the solve reads.
										linDelta is the position delta, or velocity * dt in velocity iterations.
										posDelta is always the accumulated position delta, read for computing
										the separation.
\param[in]		contactPair					Pair contact geometry and prep inputs. Reads normal/tangent0/initPen/
										maxPenBiasClamp (filled at prep).
\param[out]		solveOut						Solve output. Writes tangent, raXt, deltaLambdaN/T and
										accumulatedDeltaLambdaN.
\param[in]		appliedForceRef			Accumulated normal impulse from the previous iteration: the repulsion
										(or compression) contribution, since adhesion is applied per iteration and not
										accumulated. One accumulator spans position and velocity iterations, so a
										velocity iteration can remove repulsion the position iterations applied.
\param[in]		dt						Substep time.
\param[in]		wasActive				Activity from a prior iteration. Particle-rigid contacts pass true, because they
										have no activation mechanism and adhesion has to act on separated pairs.
\param[in]		checkOnlyActivity		If true, compute and return the activity flag only, without writing any other
										outputs.
\param[in]		isVelocityIteration		Velocity- vs position-iteration mode. In velocity iterations CN is the
										predicted end-of-step separation over dt and the projection removes
										velocity only by the amount of predicted penetration.

\return In checkOnlyActivity mode, whether the contact is active, which is wasActive or CN < 0. Otherwise whether an
impulse was applied.

The particle-rigid path reuses this as a single-vertex contact: bc = (1,0,0) collapses the bc^2*invMass denominator
to the particle invMass, and friction comes from db.friction (the rigid material is null, so combineScalars with eMAX
returns the particle coefficient).
*/
template <typename Vec, bool withAdhesion = false>
PX_FORCE_INLINE __device__ bool solveRbDbContact(PxgRigidPart& rigid, PxgDeformablePart<Vec>& db,
												 const PxgDbContactPair& contactPair, PxgDbSolveOutput& solveOut,
												 PxReal appliedForceRef, PxReal appliedTanLambdaRef, PxReal dt,
												 bool wasActive, bool checkOnlyActivity, bool isVelocityIteration)
{
	// Seed the accumulators. The other outputs stay at their default-zeroed value unless the
	// active path below writes them. Both the normal and the tangent reference span position and
	// velocity iterations, as in the rigid solver: a velocity iteration can remove repulsion the
	// position iterations applied, and the tangent reference carries so the friction shares one
	// Coulomb budget across the step rather than applying the full cone in each velocity iteration.
	solveOut.accumulatedDeltaLambdaN = appliedForceRef;
	const PxReal appliedTanRef = appliedTanLambdaRef;

	const PxReal threshold = 1.0e-14f;
	const PxReal invDt = 1.0f / dt;

	// The current separation of the pair, from the prep-time penetration plus both
	// sides' accumulated motion since prep. The CN computation and the adhesion impulse
	// derive from it. The rigid solver computes the same quantity in
	// solveContactBlockTGS.
	const PxReal separation = contactPair.initPen + rigid.linDelta.dot(contactPair.normal)
							+ rigid.angDelta.dot(rigid.raXn) - db.posDelta.dot(contactPair.normal);

	// In velocity iterations the rigid's accumulated motion enters only through the
	// separation above. The projection and friction terms see zeroed rigid deltas and
	// read the rigid velocity from linVel / angVel.
	const PxVec3 rigidLinDelta = isVelocityIteration ? PxVec3(0.0f) : rigid.linDelta;
	const PxVec3 rigidAngDelta = isVelocityIteration ? PxVec3(0.0f) : rigid.angDelta;
	const PxVec3 relLinDelta = rigidLinDelta - db.linDelta;
	const PxReal normalVel = rigid.linVel.dot(contactPair.normal) + rigid.angVel.dot(rigid.raXn);

	PxReal CN;
	if(isVelocityIteration)
	{
		// CN * dt is the predicted separation at the end of the step: the current
		// separation plus the motion at the current velocities (relLinDelta carries
		// -velocity * dt here). The solve drives the prediction toward zero, so
		// velocity projects only by the amount of predicted penetration. A present
		// penetration counts as zero separation and does not project into velocity.
		//
		// This is deliberately one look-ahead step more conservative than the rigid-body
		// solver, which gates on the predicted penetration (pen + dt * vrel < 0) and leaves a
		// contact that stays separated untouched. Separation is frozen across velocity
		// iterations here (positions do not integrate), so a separated pair that keeps
		// approaching converges to the gap-closing velocity and holds there. This forward-looking
		// projection is inherited from the PBD particle solver and kept for its robustness.
		// NvBug 6465733 tracks reconciling the two conventions.
		CN = normalVel + (PxMax(0.0f, separation) + relLinDelta.dot(contactPair.normal)) * invDt;
	}
	else
	{
		// The separation expressed as a velocity over this step. For a penetration
		// this is the depenetration rate, clamped to the pair's maximum
		// depenetration velocity.
		CN = normalVel + PxMax(contactPair.maxPenBiasClamp, separation * invDt);
	}

	// Particle-rigid contacts (the adhesion path) pass wasActive = true, so a separating
	// velocity (CN >= 0) does not deactivate them and the adhesion pull below still applies.
	// Adhesion is only instantiated for that path, so the assert holds by construction.
	PX_ASSERT(!withAdhesion || wasActive);
	const bool isActive = wasActive || CN < 0.0f;
	if(checkOnlyActivity || !isActive)
	{
		return isActive;
	}

	// Deformable body term in the denominator of the impulse calculation. Also, refer to delta lambda in the XPBD paper.
	const PxReal deformableInvMass_massSplitting = db.bc.multiply(db.bc).dot(db.vertexInvMasses);

	const PxReal rigidRefCount = static_cast<PxReal>(rigid.referenceCount);
	const PxReal unitResponse = rigidRefCount * rigid.raXnResp + deformableInvMass_massSplitting;
	const PxReal invDenom = (unitResponse > 0.0f) ? (1.0f / unitResponse) : 0.0f;

	// Normal impulse, with the -accumulated no-pull floor: one iteration removes at most the
	// repulsion the whole step applied, so the accumulated repulsion stays nonnegative (no
	// explicit clamp needed). The accumulator holds the repulsion, not the net, so the friction
	// cone and any contact report see the contact compression.
	if(withAdhesion)
	{
		// Adhesion adds a per-iteration attraction (adhesionImpulse <= 0): the applied normal
		// impulse is the repulsion plus the attraction, but only the repulsion accumulates. So
		// the compression stays nonzero at a resting adhesive contact, where the net is zero but
		// repulsion and attraction are both nonzero.
		const PxReal adhesionImpulse =
			particleAdhesionImpulse(separation, db.adhesion, db.adhesionRadius, invDenom, invDt, isVelocityIteration);
		const PxReal repulsionIncrement = PxMax(-CN * invDenom - adhesionImpulse, -solveOut.accumulatedDeltaLambdaN);
		solveOut.deltaLambdaN = repulsionIncrement + adhesionImpulse;
		solveOut.accumulatedDeltaLambdaN += repulsionIncrement;
	}
	else
	{
		// No adhesion: the whole normal impulse is repulsion (as in solveDbDbContact).
		solveOut.deltaLambdaN = PxMax(-CN * invDenom, -solveOut.accumulatedDeltaLambdaN);
		solveOut.accumulatedDeltaLambdaN += solveOut.deltaLambdaN;
	}

	// raXnF0 / raXnF1 are the rigid's angular-jacobian rows (ra x tangent) matching fric0 / fric1.
	const PxVec3 fric0 = contactPair.tangent0;
	const PxVec3 fric1 = contactPair.normal.cross(fric0);

	const PxReal tanVel0 = rigid.linVel.dot(fric0) + rigid.angVel.dot(rigid.raXnF0);
	const PxReal tanVel1 = rigid.linVel.dot(fric1) + rigid.angVel.dot(rigid.raXnF1);

	const PxReal CT0 = (fric0.dot(relLinDelta) + rigidAngDelta.dot(rigid.raXnF0)) * invDt + tanVel0;
	const PxReal CT1 = (fric1.dot(relLinDelta) + rigidAngDelta.dot(rigid.raXnF1)) * invDt + tanVel1;
	const PxVec3 relTanDelta = CT0 * fric0 + CT1 * fric1;
	const PxReal tanMagSq = relTanDelta.magnitudeSquared();

	if(tanMagSq > threshold)
	{
		const PxReal CT = PxSqrt(tanMagSq);
		const PxReal invTanMag = 1.0f / CT;
		solveOut.tangent = relTanDelta * invTanMag;

		const PxReal frac0 = solveOut.tangent.dot(fric0);
		const PxReal frac1 = solveOut.tangent.dot(fric1);
		solveOut.raXt = frac0 * rigid.raXnF0 + frac1 * rigid.raXnF1;

		const PxReal unitResponseT0 = rigidRefCount * rigid.raXnF0Resp + deformableInvMass_massSplitting;
		const PxReal invTanDenom0 = (unitResponseT0 > 0.0f) ? (1.0f / unitResponseT0) : 0.0f;

		const PxReal unitResponseT1 = rigidRefCount * rigid.raXnF1Resp + deformableInvMass_massSplitting;
		const PxReal invTanDenom1 = (unitResponseT1 > 0.0f) ? (1.0f / unitResponseT1) : 0.0f;

		PxReal deltaLambdaT0 = CT0 * invTanDenom0;
		PxReal deltaLambdaT1 = CT1 * invTanDenom1;

		const PxReal combinedFriction = combineScalars(rigid.friction, db.friction, rigid.fricCombineMode);

		// The clamp bounds the accumulated tangent to the Coulomb cone, not the per-iteration
		// increment: deltaLambdaT stays <= 0 to oppose slip along solveOut.tangent while the cone
		// holds, and turns positive to release impulse when the accumulated tangent exceeds a cone
		// that has shrunk with the repulsion. The cone is the accumulated repulsion, so an adhesive
		// contact keeps its grip from the compression rather than the near-zero net impulse at a
		// resting adhesive contact.
		const PxReal requiredLambdaTNeg = -PxSqrt(deltaLambdaT0 * deltaLambdaT0 + deltaLambdaT1 * deltaLambdaT1);
		const PxReal frictionLimit = combinedFriction * solveOut.accumulatedDeltaLambdaN; // >= 0 (accumulated normal >= 0)
		const PxReal targetAccumLambdaT = PxMax(appliedTanRef + requiredLambdaTNeg, -frictionLimit);
		solveOut.deltaLambdaT = targetAccumLambdaT - appliedTanRef;
	}

	return true;
}

/**
\brief Solves one deformable-vs-deformable/particle contact for the normal and tangential (friction) multipliers.

\tparam			Vec0					Layout of db0: PxVec3 for a cloth vertex, PxVec4 for a soft-body tetrahedron.
\tparam			Vec1					Layout of db1: PxVec3 for a cloth vertex or particle, PxVec4 for a soft-body
										tetrahedron.

\param[in]		db0						First participant, a soft body or cloth. Carries two deltas the solve reads.
										linDelta is the position delta, or velocity * dt in velocity iterations.
										posDelta is always the accumulated position delta, read for computing
										the separation.
\param[in]		db1						Second participant, a soft body, cloth, or particle. Its linDelta / posDelta
										play the same roles as db0's.
\param[in]		contactPair					Pair contact geometry and prep inputs. Reads normal / initPen /
										maxPenBiasClamp (filled at prep).
\param[out]		solveOut						Solve output. Writes tangent, deltaLambdaN/T and accumulatedDeltaLambdaN.
\param[in]		appliedNormalLambdaRef	Accumulated normal impulse from the previous iteration. One accumulator
										spans position and velocity iterations. It caps the friction in velocity
										iterations.
\param[in]		appliedTanLambdaRef		Accumulated tangential impulse from the previous iteration. One accumulator
										spans position and velocity iterations, so the whole step shares one Coulomb
										budget and friction is two-sided, releasing impulse when the cone shrinks.
\param[in]		frictionCoefficient		Combined friction coefficient for the pair.
\param[in]		dt						Substep time.
\param[in]		wasActive				Activity from a prior iteration. Currently every caller passes false, so
										activity reduces to CN < 0.
\param[in]		checkOnlyActivity		If true, compute CN and return activity without writing any other outputs.
\param[in]		isVelocityIteration		Velocity- vs position-iteration mode. In velocity iterations CN is the
										predicted end-of-step separation over dt and the projection removes
										velocity only by the amount of predicted penetration.

\return In checkOnlyActivity mode, whether the contact is active, which is wasActive or CN < 0. Otherwise whether an
impulse was applied.

A particle enters as db1, a single-vertex participant (bc = (1,0,0), so its bc^2*invMass term collapses to the
particle invMass).

Sign conventions: contactPair.normal points db0 -> db1 (stored at prep as -narrowphase); solveOut.deltaLambdaN > 0 for an
unresolved penetration. solveOut.deltaLambdaT is <= 0 while the Coulomb cone holds, and > 0 when it releases accumulated
tangential impulse that a shrunken cone no longer supports. computeDelta returns delta = -(lambdaN * normal +
lambdaT * tangent) * dt, and the caller applies +delta to db0 and -delta to db1.
*/
template <typename Vec0, typename Vec1>
PX_FORCE_INLINE __device__ bool solveDbDbContact(PxgDeformablePart<Vec0>& db0, PxgDeformablePart<Vec1>& db1,
												   const PxgDbContactPair& contactPair, PxgDbSolveOutput& solveOut,
												   PxReal appliedNormalLambdaRef, PxReal appliedTanLambdaRef,
												   PxReal frictionCoefficient,
												   PxReal dt,
												   bool wasActive,
												   bool checkOnlyActivity,
												   bool isVelocityIteration)
{
	// Seed the accumulators. The other outputs stay at their default-zeroed value
	// unless the active path below writes them. Both the normal and the tangent
	// reference span position and velocity iterations, as in the rigid solver: the
	// tangent reference carries across the whole step so the friction shares one
	// Coulomb budget rather than applying the full cone in each velocity iteration, and
	// releases tangential impulse when the cone shrinks below the accumulated tangent.
	// Friction is two-sided.
	solveOut.accumulatedDeltaLambdaN = appliedNormalLambdaRef;
	const PxReal appliedTanRef = appliedTanLambdaRef;
	const PxReal invDt = 1.0f / dt;

	const PxVec3 relLinDelta = db1.linDelta - db0.linDelta;

	// The current separation of the pair, from the prep-time penetration plus both
	// sides' accumulated motion since prep.
	const PxReal separation = contactPair.initPen + (db1.posDelta - db0.posDelta).dot(contactPair.normal);

	PxReal CN;
	if(isVelocityIteration)
	{
		// CN * dt is the predicted separation at the end of the step: the current
		// separation plus the motion at the current velocities (relLinDelta carries
		// velocity * dt here). The solve drives the prediction toward zero, so
		// velocity projects only by the amount of predicted penetration. A present
		// penetration counts as zero separation and does not project into velocity.
		CN = (PxMax(0.0f, separation) + relLinDelta.dot(contactPair.normal)) * invDt;
	}
	else
	{
		// The separation expressed as a velocity over this step. For a penetration
		// this is the depenetration rate, clamped to the pair's maximum
		// depenetration velocity.
		CN = PxMax(contactPair.maxPenBiasClamp, separation * invDt);
	}

	// Currently every caller passes wasActive = false, so activity is CN < 0 and a pair that depenetrates stops
	// contributing. That keeps the pre-count in step with the solves, since both then count the same pairs.
	// The parameter stays for a caller that has to remain active while separated, which is what adhesion needs
	// and what solveRbDbContact uses it for. It cannot be sourced from the isInCollision flag, because that bit
	// is shared with the mass-splitting reference count at auxInd bit 29, and persisting it would double-count
	// deformable pairs.
	const bool isActive = wasActive || CN < 0.0f;
	if(checkOnlyActivity || !isActive)
	{
		return isActive;
	}

	// XPBD unit response: bc^2*invMass per side (after mass-splitting), both
	// sides contributing additively.
	const PxReal db0Resp = db0.bc.multiply(db0.bc).dot(db0.vertexInvMasses);
	const PxReal db1Resp = db1.bc.multiply(db1.bc).dot(db1.vertexInvMasses);
	const PxReal unitResponse = db0Resp + db1Resp;
	const PxReal invDenom = (unitResponse > 1e-16f) ? (1.0f / unitResponse) : 0.0f;

	solveOut.deltaLambdaN = PxMax(-CN * invDenom, -solveOut.accumulatedDeltaLambdaN);
	solveOut.accumulatedDeltaLambdaN += solveOut.deltaLambdaN;

	// Friction: solve-time tangent from the tangential component of
	// relLinDelta. (Differs from solveRbDbContact, which uses a prep-time
	// tangent0 with rigid ra x tangent jacobian rows. contactPair.tangent0 is
	// unused here.)
	const PxReal normalProj = relLinDelta.dot(contactPair.normal);
	PxVec3 tanDir = relLinDelta - contactPair.normal * normalProj;
	const PxReal tanDelta = tanDir.normalize();
	solveOut.tangent = tanDir;

	// The clamp bounds the accumulated tangent to the cone, not the per-iteration
	// increment: deltaLambdaT stays <= 0 to oppose drift along solveOut.tangent while the
	// cone holds, and turns positive to release impulse when the accumulated tangent
	// exceeds a cone that has shrunk with the normal impulse.
	const PxReal requiredLambdaTNeg = -tanDelta * invDt * invDenom;
	const PxReal frictionLimit = solveOut.accumulatedDeltaLambdaN * frictionCoefficient; // >= 0 (lambdaN >= 0).
	const PxReal targetAccumLambdaT = PxMax(appliedTanRef + requiredLambdaTNeg, -frictionLimit);
	solveOut.deltaLambdaT = targetAccumLambdaT - appliedTanRef;

	return true;
}

// Solves a single rigid-vs-deformable attachment for the per-axis impulse.
// Returns deltaImpulse for the caller's deformable-side writeback.
// outDeltaLinVel and outDeltaAngVel are for the rigid-side writeback, where the
// caller does the refN inflation and the 2-slot scatter.
//
// Attachment vs. contact math:
//  - Per-axis (X/Y/Z) constraint with three independent scalar lambdas, not
//    a normal and tangent decomposition.
//  - biasCoefficient softening retained, and not clamped to maxPenBias.
//  - No friction term.
//
// PGS / TGS unified. rigid.linDelta / rigid.angDelta are zero on PGS, and in
// velocity iterations the solve zeros its rigid delta terms, so the position
// terms collapse to a pure velocity projection (the caller also passes
// biasCoefficient = 0). The isTGS branch separates the units: PGS works in
// velocity, TGS in position deltas scaled by invDt at the end.
template <typename Vec>
PX_FORCE_INLINE __device__ PxVec3 solveRbDbAttachment(
	const PxgRigidPart& rigid,
	const PxgDeformablePart<Vec>& db,
	PxReal dt, PxReal biasCoefficient,
	bool isTGS, bool isVelocityIteration,
	PxVec3& outDeltaLinVel, PxVec3& outDeltaAngVel)
{
	const PxReal invDt = 1.0f / dt;

	// Per-axis rigid velocity at the attachment point: vel.linVel.{x,y,z} +
	// vel.angVel dot raXn{0,1,2}.
	const PxReal velOfRigidAtAttachmentPointX = rigid.linVel.x + rigid.angVel.dot(rigid.raXn0);
	const PxReal velOfRigidAtAttachmentPointY = rigid.linVel.y + rigid.angVel.dot(rigid.raXn1);
	const PxReal velOfRigidAtAttachmentPointZ = rigid.linVel.z + rigid.angVel.dot(rigid.raXn2);

	// Mass-splitting denominator adjustment. Prep baked
	// velMult = 1 / (R + D) with D = attachPointInvMass (raw bc^2*invM). We
	// want denom' = N*R + D_split with D_split = attachPointInvMassSplit
	// (refCount-inflated bc^2*invM, populated at read time). Rearranging gives
	// denom' = N/velMult - (N*D - D_split), which is what the
	// (refN / velMult - denomBias) below computes.
	const PxReal refN = static_cast<PxReal>(rigid.attachRefCount);
	const PxReal denomBias = refN * db.attachPointInvMass - db.attachPointInvMassSplit;
	const PxReal adjVelMult0 = 1.0f / (refN / rigid.velMultiplierXYZ.x - denomBias);
	const PxReal adjVelMult1 = 1.0f / (refN / rigid.velMultiplierXYZ.y - denomBias);
	const PxReal adjVelMult2 = 1.0f / (refN / rigid.velMultiplierXYZ.z - denomBias);

	PxReal deltaF0, deltaF1, deltaF2;
	if(isTGS)
	{
		// db.linDelta is the bc-weighted position delta in position iterations and the
		// deformable velocity * dt in velocity iterations. The rigid deltas carry position
		// motion only and are zeroed in velocity iterations so the attachment projects
		// the relative velocity.
		const PxVec3 rigidLinDelta = isVelocityIteration ? PxVec3(0.0f) : rigid.linDelta;
		const PxVec3 rigidAngDelta = isVelocityIteration ? PxVec3(0.0f) : rigid.angDelta;
		const PxVec3 linDelta = db.linDelta - rigidLinDelta;
		const PxVec3 tgsErrVel(
			(linDelta.x - rigidAngDelta.dot(rigid.raXn0) - rigid.positionErrorXYZ.x * biasCoefficient) * invDt,
			(linDelta.y - rigidAngDelta.dot(rigid.raXn1) - rigid.positionErrorXYZ.y * biasCoefficient) * invDt,
			(linDelta.z - rigidAngDelta.dot(rigid.raXn2) - rigid.positionErrorXYZ.z * biasCoefficient) * invDt);
		deltaF0 = (tgsErrVel.x - velOfRigidAtAttachmentPointX) * adjVelMult0;
		deltaF1 = (tgsErrVel.y - velOfRigidAtAttachmentPointY) * adjVelMult1;
		deltaF2 = (tgsErrVel.z - velOfRigidAtAttachmentPointZ) * adjVelMult2;
	}
	else
	{
		// PGS: db.linDelta is bc-weighted softbody / cloth velocity.
		// biasCoefficient is the Box2D-style Baumgarte factor
		// (Erin Catto, "Sequential Impulses", GDC 2006).
		const PxVec3 velError(
			db.linDelta.x - rigid.positionErrorXYZ.x * biasCoefficient * invDt,
			db.linDelta.y - rigid.positionErrorXYZ.y * biasCoefficient * invDt,
			db.linDelta.z - rigid.positionErrorXYZ.z * biasCoefficient * invDt);
		deltaF0 = (velError.x - velOfRigidAtAttachmentPointX) * adjVelMult0;
		deltaF1 = (velError.y - velOfRigidAtAttachmentPointY) * adjVelMult1;
		deltaF2 = (velError.z - velOfRigidAtAttachmentPointZ) * adjVelMult2;
	}

	const PxVec3 deltaImpulse(deltaF0, deltaF1, deltaF2);
	outDeltaLinVel = deltaImpulse * rigid.attachInvMass;
	outDeltaAngVel = rigid.raXn0 * deltaF0 + rigid.raXn1 * deltaF1 + rigid.raXn2 * deltaF2;

	return deltaImpulse;
}

// Solves a single deformable-vs-deformable attachment for the position
// correction. Math: error = pos1 - pos0, wTot = sum of refCount-inflated
// bc^2*invM per side (attachPointInvMassSplit from the DbDb readers),
// outDelta = error / wTot. It is pair-symmetric, so callers apply +outDelta to side
// 0 and -outDelta to side 1 via an .xyz-only scatter. The finalize-time /.w cancels
// the inflation. Returns false iff both sides are kinematic (wTot == 0).
// Mode-agnostic: in velocity iterations the readers pass pos0/pos1 as the
// bc-weighted velocity * dt, so error = (vel1 - vel0) * dt and the velocity-only
// finalize projects the relative attachment-point velocity. No separate branch.
template <typename Vec0, typename Vec1>
PX_FORCE_INLINE __device__ bool solveDbDbAttachment(
	const PxgDeformablePart<Vec0>& db0,
	const PxgDeformablePart<Vec1>& db1,
	const PxVec3& pos0, const PxVec3& pos1,
	PxVec3& outDelta)
{
	const PxVec3 error = pos1 - pos0;
	const PxReal wTot = db0.attachPointInvMassSplit + db1.attachPointInvMassSplit;
	if(wTot > 0.0f)
	{
		outDelta = error * (1.0f / wTot);
		return true;
	}
	outDelta = PxVec3(0.0f);
	return false;
}

} // namespace physx

#endif // __DEFORMABLE_AND_PARTICLE_UTILS_CUH__
