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

#ifndef __DEFORMABLE_UTILS_CUH__
#define __DEFORMABLE_UTILS_CUH__

// Toggle: deformable-particle XPBD mass-splitting (per-particle contact-count inflation, the
// dual of the deformable-vertex refCount).
//   1 = on : CP/SP pre-count bumps mAccumDeltaP[.].w; both solve halves inflate the particle
//            invMass by it; the particle finalize divides it out + zeros .w.
//   0 = off: per-particle averaging only (raw particle invMass).
// On is required for correct load transmission under PGS/TGS; it is also the root of an open
// TGS-ext over-displacement deficit, so it stays a toggle (flip to 0 to A/B).
#ifndef PX_DB_PARTICLE_MASS_SPLIT
#define PX_DB_PARTICLE_MASS_SPLIT 1
#endif

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
// convention for the soft-body / cloth / particle contact + attachment pre-counts; pairs
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

// Accumulate per-vertex position-delta into a deformable's
// mDeltaPos / mSimDelta buffer. Does NOT touch .w: callers populate .w in a
// dedicated pre-count pass that runs before the solve, and the finalize then
// divides .xyz by max(.w, 1) and zeros both.

static __device__ void updatePositionDeltaVtx(float4* outputDeltaPositions, PxU32 vertIndex,
	const PxVec3& deltaPosition, PxReal invMass)
{
	if (invMass > 0.0f)
		AtomicAdd(outputDeltaPositions[vertIndex], deltaPosition*invMass);
}

static __device__ void updatePositionDeltaEdge(float4* outputDeltaPositions, PxU32 vertIndex0, PxU32 vertIndex1,
	const PxVec3& deltaPosition, PxReal invMassBary0, PxReal invMassBary1)
{
	if (invMassBary0 > 0.0f)
		AtomicAdd(outputDeltaPositions[vertIndex0], deltaPosition*invMassBary0);
	if (invMassBary1 > 0.0f)
		AtomicAdd(outputDeltaPositions[vertIndex1], deltaPosition*invMassBary1);
}

static __device__ void updatePositionDeltaTri(float4* outputDeltaPositions, const uint4& triVertIndices,
	const PxVec3& deltaPosition, const float4& invMassBary)
{
	if (invMassBary.x > 0.0f)
		AtomicAdd(outputDeltaPositions[triVertIndices.x], deltaPosition*invMassBary.x);
	if (invMassBary.y > 0.0f)
		AtomicAdd(outputDeltaPositions[triVertIndices.y], deltaPosition*invMassBary.y);
	if (invMassBary.z > 0.0f)
		AtomicAdd(outputDeltaPositions[triVertIndices.z], deltaPosition*invMassBary.z);
}

static __device__ void updatePositionDeltaTet(float4* outputDeltaPositions, const uint4& tetVertIndices,
	const PxVec3& deltaPosition, const float4& invMassBary)
{
	if (invMassBary.x > 0.0f)
		AtomicAdd(outputDeltaPositions[tetVertIndices.x], deltaPosition*invMassBary.x);
	if (invMassBary.y > 0.0f)
		AtomicAdd(outputDeltaPositions[tetVertIndices.y], deltaPosition*invMassBary.y);
	if (invMassBary.z > 0.0f)
		AtomicAdd(outputDeltaPositions[tetVertIndices.z], deltaPosition*invMassBary.z);
	if (invMassBary.w > 0.0f)
		AtomicAdd(outputDeltaPositions[tetVertIndices.w], deltaPosition*invMassBary.w);
}

//This code is based on Matthias Muller's paper: A robust method to extract the rotational part of deformations
//Basically, this is another way to extract a rotational matrix from deformation gradient instead of using polar
//decomposition 
__device__ inline void extractRotation(const PxMat33 &A, PxQuat& q, int maxIter)
{
	const PxReal eps = 1.0e-6f;
	for (int iter = 0; iter < maxIter; iter++)
	{
		PxMat33 R(q);
		PxVec3 omega = R.column0.cross(A.column0) + R.column1.cross(A.column1) + R.column2.cross(A.column2);
		// (Cross(R.cols[0], A.cols[0]) + Cross(R.cols[1], A.cols[1]) + Cross(R.cols[2], A.cols[2]));

		//omega *= 1.0f / (fabsf(Dot(R.cols[0], A.cols[0]) + Dot(R.cols[1], A.cols[1]) + Dot(R.cols[2], A.cols[2])) + 1.0e-6f);
		omega *= 1.0f / (PxAbs(R.column0.dot(A.column0) + R.column1.dot(A.column1) + R.column2.dot(A.column2)) + eps);

		const float w = omega.normalize();

		PxQuat tempQ;

		if (w < eps) // near-zero omega produces non-unit vector after normalize()
			tempQ = PxQuat(PxIdentity);
		else
			tempQ = PxQuat(w, omega);

		q = tempQ * q;
		q = q.getNormalized();

		// early-exit after one update (instead of before) since we've already done the expensive computations to find w
		if (w < eps)
			break;
	}
}

__device__ inline void sb_extractRotationAPD(const PxMat33 &F, PxQuat& q, int maxIter)
{
	const PxReal eps = 1.0e-6;
	const PxReal threshold = 1 - eps;
	//Use properties of Rodriguez's formula to detect degenerate case of exact 180 deg rotation by checking if the matrix' trace is close to -1
	//Rodrigues formula for rotation matrices: trace(R) = 1 + 2*cos(theta)
	//Double3 scaling = new Double3(Math.Max(eps, F.column0.Length), Math.Max(eps, F.column1.Length), Math.Max(eps, F.column2.Length));
	//bool overwriteGradient = F.column0.x / scaling.x + F.column1.y / scaling.y + F.column2.z / scaling.z < -0.99;   
	//double wPrev = 0;
	for (int i = 0; i < maxIter; ++i)
	{
		PxMat33 B = PxMat33(q.getConjugate()) * F;
		PxVec3 gradient = PxVec3(B.column2.y - B.column1.z, B.column0.z - B.column2.x, B.column1.x - B.column0.y);
		/*if (overwriteGradient)
		{
			gradient = new Double3(-2, 0, 0); //Gradient for 90 Degree rotation around x axis, any non-zero gradient should work
			overwriteGradient = false;
		}*/
		if (i == 0 && gradient.magnitudeSquared() < 1e-16)
		{
			//If loop got stuck already in first iteration (e. g. rotation around 180 deg around an arbitrary axis), distort gradient
			gradient = PxVec3(-2, 0, 0); //Gradient for 90 Degree rotation around x axis, any non-zero gradient should work
		}
		PxReal h00 = B.column1.y + B.column2.z;
		PxReal h11 = B.column0.x + B.column2.z;
		PxReal h22 = B.column0.x + B.column1.y;
		PxReal h01 = -0.5f * (B.column1.x + B.column0.y);
		PxReal h02 = -0.5f * (B.column2.x + B.column0.z);
		PxReal h12 = -0.5f * (B.column2.y + B.column1.z);
		PxReal detH = -h02 * h02 * h11 + 2.0f * h01 * h02 * h12 - h00 * h12 * h12 - h01 * h01 * h22 + h00 * h11 * h22;
		PxVec3 omega;
		PxReal factor = -0.25f / detH;
		omega.x = factor * ((h11 * h22 - h12 * h12) * gradient.x + (h02 * h12 - h01 * h22) * gradient.y + (h01 * h12 - h02 * h11) * gradient.z);
		omega.y = factor * ((h02 * h12 - h01 * h22) * gradient.x + (h00 * h22 - h02 * h02) * gradient.y + (h01 * h02 - h00 * h12) * gradient.z);
		omega.z = factor * ((h01 * h12 - h02 * h11) * gradient.x + (h01 * h02 - h00 * h12) * gradient.y + (h00 * h11 - h01 * h01) * gradient.z);
		if (fabs(detH) < 1e-9f)
			omega = -gradient;
		if (omega.dot(gradient) > 0.0f)
			omega = gradient * -0.125f;
		PxReal l_omega2 = omega.magnitudeSquared();
		PxReal w = (1.0 - l_omega2) / (1.0f + l_omega2);
		PxVec3 vec = omega * (2.0f / (1.0f + l_omega2));
		q = q * PxQuat(vec.x, vec.y, vec.z, w);
		if (w > threshold /*&& wPrev>= w*/)
			break;
		//wPrev = w;
	}
}

PX_FORCE_INLINE __device__ PxVec3 projectVectorOntoPlane(PxVec3 v, PxVec3 planeNormal)
{
	return v - (planeNormal.dot(v) / planeNormal.magnitudeSquared()) * planeNormal;
}

// Function to compute Lame's parameters (lambda and mu)
PX_FORCE_INLINE __device__ PxPair<PxReal, PxReal> lameParameters(PxReal Young, PxReal Poisson)
{
	const PxReal lambda = Young * Poisson / ((1.0f + Poisson) * (1.0f - 2.0f * Poisson));
	const PxReal mu = Young / (2.0f * (1.0f + Poisson));

	return PxPair<PxReal, PxReal>(lambda, mu);
}

struct MagnitudeClampVelocityAveraging
{
	PX_FORCE_INLINE __device__ void operator()(PxVec3& tVel, const PxVec3& deltaVel) const
	{
		const PxReal deltaVelMagSqr = deltaVel.magnitudeSquared();
		const PxReal velMagSqr = tVel.magnitudeSquared();
		if (deltaVelMagSqr < velMagSqr)
			tVel = tVel * PxSqrt(deltaVelMagSqr / velMagSqr);
	}
};

struct LinearBlendVelocityAveraging
{
	PxReal scale;

	PX_FORCE_INLINE __device__ LinearBlendVelocityAveraging(PxU32 nbPosIters)
		: scale(PxMin(0.9f, 2.0f * PxSqrt(1.0f / PxReal(nbPosIters))))
	{}

	PX_FORCE_INLINE __device__ void operator()(PxVec3& tVel, const PxVec3& deltaVel) const
	{
		if (deltaVel.magnitudeSquared() < tVel.magnitudeSquared())
			tVel = tVel * scale + deltaVel * (1.f - scale);
	}
};

// Per-vertex velocity finalization shared by softbody and cloth. Attenuates
// `vel` (in-place) toward the position-derived velocity via `averageOp`, and
// applies settle-damping below the settling threshold. Returns whether the
// vertex is awake. Kinematic / infinite-mass vertices (pos.w == 0) return false.
template <typename AverageOp>
PX_FORCE_INLINE __device__ bool finalizeVertexVelocity(
	const float4& pos,
	float4& vel,                                // in/out
	const float4& posDelta,
	const PxReal settlingThreshold,
	const PxReal sleepThreshold,
	const PxReal settlingDamping,
	const PxReal dt,
	const PxReal invDt,
	const bool alwaysRunVelocityAveraging,
	const AverageOp& averageOp)
{
	if (pos.w == 0.0f)
		return false;

	const PxReal settleTolerance = settlingThreshold * dt;
	const PxReal tolerance = sleepThreshold * dt;
	const PxReal sleepDamping = 1.f - PxMin(1.f, settlingDamping * dt);

	const PxVec3 tDelta = PxLoad3(posDelta);
	const PxReal magSq = tDelta.magnitudeSquared();

	bool awake;
	PxReal velocityScaling = 1.0f;
	if (magSq < settleTolerance * settleTolerance)
	{
		awake = magSq >= tolerance * tolerance;
		velocityScaling = sleepDamping;
	}
	else
	{
		awake = true;
	}

	if (alwaysRunVelocityAveraging || velocityScaling != 1.0f)
	{
		PxVec3 tVel = PxLoad3(vel);
		if (alwaysRunVelocityAveraging)
			averageOp(tVel, tDelta * invDt);
		vel = make_float4(velocityScaling * tVel.x, velocityScaling * tVel.y, velocityScaling * tVel.z, vel.w);
	}
	return awake;
}

// Barycentric interpolation of a per-vertex float4 buffer over a tet (4 verts) or
// triangle (3 verts) -- projects per-vertex deltas/positions to a contact's bary point.
PX_FORCE_INLINE __device__ static float4 barycentricProjectTet(const uint4 vertIdx, const float4* buffer, const float4 barycentric)
{
	const float4 a = buffer[vertIdx.x];
	const float4 b = buffer[vertIdx.y];
	const float4 c = buffer[vertIdx.z];
	const float4 d = buffer[vertIdx.w];
	return a * barycentric.x + b * barycentric.y + c * barycentric.z + d * barycentric.w;
}

PX_FORCE_INLINE __device__ static float4 barycentricProjectTri(const uint4 vertIdx, const float4* buffer, const float4 barycentric)
{
	const float4 a = buffer[vertIdx.x];
	const float4 b = buffer[vertIdx.y];
	const float4 c = buffer[vertIdx.z];
	return a * barycentric.x + b * barycentric.y + c * barycentric.z;
}

PX_FORCE_INLINE __device__ void prepareDbRigidAttachment(
	PxgDbRigidAttachmentBlock& block,
	const PxVec3& point, PxReal deformableInvMass,
	const PxVec3& rigidLocalPose, PxNodeIndex rigidId, PxU32 elemId,
	const float4& baryOrType, PxU32 rigidBodyRefCount,
	const PxgConstraintPrepareDesc* prepareDesc, const PxU32* solverBodyIndices,
	const PxgSolverSharedDescBase* sharedDesc,
	float4* rigidDeltaVel, PxU32 workIndex, PxU32 numRigidAttachments)
{
	const PxU32 offset = threadIdx.x & 31;
	const PxgBodySim* bodySims = sharedDesc->mBodySimBufferDeviceData;
	const PxgSolverBodyData* solverBodyData = prepareDesc->solverBodyDataPool;
	const PxgSolverTxIData* solverDataTxIPool = prepareDesc->solverBodyTxIDataPool;
	const PxAlignedTransform* bodyFrames = prepareDesc->body2WorldPool;

	PxU32 idx = 0;
	if (!rigidId.isStaticBody())
		idx = solverBodyIndices[rigidId.index()];

	const PxVec3 normal0(1.f, 0.f, 0.f);
	const PxVec3 normal1(0.f, 1.f, 0.f);
	const PxVec3 normal2(0.f, 0.f, 1.f);

	if (rigidId.isArticulation())
	{
		PxU32 nodeIndexA = rigidId.index();
		PxU32 artiId = bodySims[nodeIndexA].articulationRemapId;
		PxgArticulation& articulation = sharedDesc->articulations[artiId];
		const PxU32 linkID = rigidId.articulationLinkId();
		const PxTransform body2World = articulation.linkBody2Worlds[linkID];
		const PxVec3 bodyFrame0p(body2World.p.x, body2World.p.y, body2World.p.z);

		PxVec3 ra = rigidLocalPose;
		ra = body2World.rotate(ra);
		PxVec3 error = ra + bodyFrame0p - point;

		const PxVec3 raXn0 = ra.cross(normal0);
		const PxVec3 raXn1 = ra.cross(normal1);
		const PxVec3 raXn2 = ra.cross(normal2);

		PxSpatialMatrix& spatialResponse = articulation.spatialResponseMatrixW[linkID];
		const Cm::UnAlignedSpatialVector deltaV0 = spatialResponse * Cm::UnAlignedSpatialVector(normal0, raXn0);
		const Cm::UnAlignedSpatialVector deltaV1 = spatialResponse * Cm::UnAlignedSpatialVector(normal1, raXn1);
		const Cm::UnAlignedSpatialVector deltaV2 = spatialResponse * Cm::UnAlignedSpatialVector(normal2, raXn2);

		const PxReal resp0 = deltaV0.top.dot(raXn0) + deltaV0.bottom.dot(normal0) + deformableInvMass;
		const PxReal resp1 = deltaV1.top.dot(raXn1) + deltaV1.bottom.dot(normal1) + deformableInvMass;
		const PxReal resp2 = deltaV2.top.dot(raXn2) + deltaV2.bottom.dot(normal2) + deformableInvMass;

		const float velMultiplier0 = (resp0 > 0.f) ? (1.f / resp0) : 0.f;
		const float velMultiplier1 = (resp1 > 0.f) ? (1.f / resp1) : 0.f;
		const float velMultiplier2 = (resp2 > 0.f) ? (1.f / resp2) : 0.f;

		const PxReal biasedErr0 = error.dot(normal0);
		const PxReal biasedErr1 = error.dot(normal1);
		const PxReal biasedErr2 = error.dot(normal2);

		block.raXn0_biasW[offset] = make_float4(raXn0.x, raXn0.y, raXn0.z, biasedErr0);
		block.raXn1_biasW[offset] = make_float4(raXn1.x, raXn1.y, raXn1.z, biasedErr1);
		block.raXn2_biasW[offset] = make_float4(raXn2.x, raXn2.y, raXn2.z, biasedErr2);
		// Articulations don't use invMass0; set it to 1 so the linear impulse stays an impulse.
		block.velMultiplierXYZ_invMassW[offset] = make_float4(velMultiplier0, velMultiplier1, velMultiplier2, 1.f);
		block.elemId[offset] = elemId;
		block.rigidId[offset] = rigidId.getInd();
		block.baryOrType[offset] = baryOrType;
		block.rigidBodyRefCount[offset] = rigidBodyRefCount;
	}
	else
	{
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

		PxVec3 ra = rigidLocalPose;
		ra = bodyFrame0.rotate(ra);
		PxVec3 error = ra + bodyFrame0p - point;

		const PxVec3 raXn0 = ra.cross(normal0);
		const PxVec3 raXn1 = ra.cross(normal1);
		const PxVec3 raXn2 = ra.cross(normal2);

		const PxVec3 raXnSqrtInertia0 = invSqrtInertia0 * raXn0;
		const PxVec3 raXnSqrtInertia1 = invSqrtInertia0 * raXn1;
		const PxVec3 raXnSqrtInertia2 = invSqrtInertia0 * raXn2;
		const float resp0 = (raXnSqrtInertia0.dot(raXnSqrtInertia0))*inertiaScale + invMass0 + deformableInvMass;
		const float resp1 = (raXnSqrtInertia1.dot(raXnSqrtInertia1))*inertiaScale + invMass0 + deformableInvMass;
		const float resp2 = (raXnSqrtInertia2.dot(raXnSqrtInertia2))*inertiaScale + invMass0 + deformableInvMass;

		const float velMultiplier0 = (resp0 > 0.f) ? (1.f / resp0) : 0.f;
		const float velMultiplier1 = (resp1 > 0.f) ? (1.f / resp1) : 0.f;
		const float velMultiplier2 = (resp2 > 0.f) ? (1.f / resp2) : 0.f;

		const PxReal biasedErr0 = error.dot(normal0);
		const PxReal biasedErr1 = error.dot(normal1);
		const PxReal biasedErr2 = error.dot(normal2);

		block.raXn0_biasW[offset] = make_float4(raXnSqrtInertia0.x, raXnSqrtInertia0.y, raXnSqrtInertia0.z, biasedErr0);
		block.raXn1_biasW[offset] = make_float4(raXnSqrtInertia1.x, raXnSqrtInertia1.y, raXnSqrtInertia1.z, biasedErr1);
		block.raXn2_biasW[offset] = make_float4(raXnSqrtInertia2.x, raXnSqrtInertia2.y, raXnSqrtInertia2.z, biasedErr2);
		block.velMultiplierXYZ_invMassW[offset] = make_float4(velMultiplier0, velMultiplier1, velMultiplier2, invMass0);
		block.elemId[offset] = elemId;
		block.rigidId[offset] = rigidId.getInd();
		block.baryOrType[offset] = baryOrType;
		block.rigidBodyRefCount[offset] = rigidBodyRefCount;

		if (rigidDeltaVel)
		{
			rigidDeltaVel[workIndex] = make_float4(0.f);
			rigidDeltaVel[workIndex + numRigidAttachments] = make_float4(0.f);
		}
	}
}

PX_FORCE_INLINE __device__ void prepareDbRigidContact(PxgDbRigidContactBlock& block, const PxVec3& normal,
												   PxgSolverSharedDescBase* sharedDesc, const PxVec3& p, PxReal pen, const PxVec3& delta,
												   const PxNodeIndex& rigidId, const float4& barycentric,
												   PxgConstraintPrepareDesc* prepareDesc, PxU32* solverBodyIndices, PxReal penBiasClampFEM,
												   PxReal invDt, bool isTGS)
{
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	PxAlignedTransform* bodyFrames = prepareDesc->body2WorldPool;

	PxgBodySim* bodySims = sharedDesc->mBodySimBufferDeviceData;

	PxgSolverBodyData* solverBodyData = prepareDesc->solverBodyDataPool;
	PxgSolverTxIData* solverDataTxIPool = prepareDesc->solverBodyTxIDataPool;

	// Select two tangent vectors to the normal.
	// Note that the friction behavior may vary depending on the chosen tangent vectors.

	PxVec3 t0, t1;
	PxComputeBasisVectors(normal, t0, t1);

	PxReal penBiasClampRigid;
	float4 raXn_resp;
	float4 raXnF0_resp;
	float4 raXnF1_resp;
	PxReal invMass0;

	if(rigidId.isArticulation())
	{
		PxU32 nodeIndexA = rigidId.index();
		PxU32 artiId = bodySims[nodeIndexA].articulationRemapId;

		PxgArticulation& articulation = sharedDesc->articulations[artiId];

		const PxU32 linkID = rigidId.articulationLinkId();
		const PxTransform body2World = articulation.linkBody2Worlds[linkID];
		penBiasClampRigid = articulation.links[linkID].initialAngVelXYZ_penBiasClamp.w;

		const PxVec3 bodyFrame0p(body2World.p.x, body2World.p.y, body2World.p.z);

		PxVec3 ra = p - bodyFrame0p;
		PxVec3 raXn = ra.cross(normal);
		PxVec3 raXF0 = ra.cross(t0);
		PxVec3 raXF1 = ra.cross(t1);

		PxSpatialMatrix& spatialResponse = articulation.spatialResponseMatrixW[linkID];

		const Cm::UnAlignedSpatialVector deltaV0 = spatialResponse * Cm::UnAlignedSpatialVector(normal, raXn);
		const PxReal resp0 = deltaV0.top.dot(raXn) + deltaV0.bottom.dot(normal);

		const Cm::UnAlignedSpatialVector deltaFV0 = spatialResponse * Cm::UnAlignedSpatialVector(t0, raXF0);
		const Cm::UnAlignedSpatialVector deltaFV1 = spatialResponse * Cm::UnAlignedSpatialVector(t1, raXF1);

		const PxReal respF0 = deltaFV0.top.dot(raXF0) + deltaFV0.bottom.dot(t0);
		const PxReal respF1 = deltaFV1.top.dot(raXF1) + deltaFV1.bottom.dot(t1);

		raXn_resp = make_float4(raXn.x, raXn.y, raXn.z, resp0);
		raXnF0_resp = make_float4(raXF0.x, raXF0.y, raXF0.z, respF0);
		raXnF1_resp = make_float4(raXF1.x, raXF1.y, raXF1.z, respF1);

		// Articulations don't use invMass0. We set it to 1 so we get the linear impulse rather than velocity change.
		invMass0 = 1.f;
	}
	else
	{
		PxU32 idx = 0;
		if(!rigidId.isStaticBody())
		{
			idx = solverBodyIndices[rigidId.index()];
		}

		PxMat33 invSqrtInertia0 = solverDataTxIPool[idx].sqrtInvInertia;
		const float4 linVel_invMass0 = solverBodyData[idx].initialLinVelXYZ_invMassW;
		penBiasClampRigid = solverBodyData[idx].initialAngVelXYZ_penBiasClamp.w;
		invMass0 = linVel_invMass0.w;

		// both static and kinematic object have invMass = 0.f
		const bool isKinematic = (invMass0 == 0.f) && (!rigidId.isStaticBody());

		PxAlignedTransform bodyFrame0 = bodyFrames[idx];
		const PxVec3 bodyFrame0p(bodyFrame0.p.x, bodyFrame0.p.y, bodyFrame0.p.z);

		PxVec3 ra = p - bodyFrame0p;
		PxVec3 raXn = ra.cross(normal);
		PxVec3 raXF0 = ra.cross(t0);
		PxVec3 raXF1 = ra.cross(t1);

		const PxVec3 raXnSqrtInertia = invSqrtInertia0 * raXn;
		const float resp0 = (raXnSqrtInertia.dot(raXnSqrtInertia)) + invMass0;

		const PxVec3 raXF0SqrtInertia = invSqrtInertia0 * raXF0;
		const PxVec3 raXF1SqrtInertia = invSqrtInertia0 * raXF1;

		const float respF0 = (raXF0SqrtInertia.dot(raXF0SqrtInertia)) + invMass0;
		const float respF1 = (raXF1SqrtInertia.dot(raXF1SqrtInertia)) + invMass0;

		if(isKinematic)
		{
			raXn_resp = make_float4(raXn.x, raXn.y, raXn.z, resp0);
			raXnF0_resp = make_float4(raXF0.x, raXF0.y, raXF0.z, respF0);
			raXnF1_resp = make_float4(raXF1.x, raXF1.y, raXF1.z, respF1);
		}
		else
		{
			raXn_resp = make_float4(raXnSqrtInertia.x, raXnSqrtInertia.y, raXnSqrtInertia.z, resp0);
			raXnF0_resp = make_float4(raXF0SqrtInertia.x, raXF0SqrtInertia.y, raXF0SqrtInertia.z, respF0);
			raXnF1_resp = make_float4(raXF1SqrtInertia.x, raXF1SqrtInertia.y, raXF1SqrtInertia.z, respF1);
		}
	}

	// The narrowphase reports pen at the predicted position; re-reference it to the start-of-step
	// configuration, the same frame the solver's live relLinDelta is measured from, so the two agree.
	// No-op in TGS (the deformable delta buffer is 0 at prep). The rigid side enters the solve via its
	// live velocity (normalVel) + TGS linDelta, not here.
	PxReal error = pen + delta.dot(normal);

	// KS - TODO - split these into 5 separate vectors to promote coalesced memory accesses!
	block.normal_errorW[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, error);
	block.raXn_resp[threadIndexInWarp] = raXn_resp;
	block.raXnF0_resp[threadIndexInWarp] = raXnF0_resp;
	block.raXnF1_resp[threadIndexInWarp] = raXnF1_resp;
	block.fricTan0_invMass0[threadIndexInWarp] = make_float4(t0.x, t0.y, t0.z, invMass0);
	block.maxPenBiasClamp[threadIndexInWarp] = PxMax(penBiasClampRigid, penBiasClampFEM);
	block.barycentric[threadIndexInWarp] = barycentric;
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
// Operated on by solveRbDbContact (RB-DB), solveDbDbContact (DB-DB), and
// solveRbPcContact (particle-rigid, in particlesystem.cu).
struct PxgDbContactState
{
	// --- Resolved at solve time ---
	PxVec3 normal = PxVec3(0.0f);
	PxVec3 tangent = PxVec3(0.0f);
	PxVec3 raXn = PxVec3(0.0f);
	PxVec3 raXt = PxVec3(0.0f);
	PxReal deltaLambdaN = 0.0f;
	PxReal deltaLambdaT = 0.0f;
	PxReal accumulatedDeltaLambdaN = 0.0f;

	// --- Pair-level contact-prep cache (populated by readContactPrep) ---
	// tangent0 is the prep-time first friction basis; the second is derived
	// at solve time via normal.cross(tangent0). initPen is the bc-weighted
	// initial penetration error baked at prep.
	PxVec3 tangent0;
	PxReal initPen = 0.0f;

	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrep(const Block& block, PxU32 lane)
	{
		const float4 normal_errorW = block.normal_errorW[lane];
		normal = PxVec3(normal_errorW.x, normal_errorW.y, normal_errorW.z);
		initPen = normal_errorW.w;

		// fricTan0_invMass0.xyz packs the pair tangent; .w (rigid invMass) is
		// read separately by PxgRigidPart::readContactPrep.
		const float4 fricTan0_invMass0 = block.fricTan0_invMass0[lane];
		tangent0 = PxVec3(fricTan0_invMass0.x, fricTan0_invMass0.y, fricTan0_invMass0.z);
	}

	// Two-sided (DB-DB) variant. PxgDbDbContactBlock packs initPen into
	// normal_pen[lane].w and has no prep-time tangent0 -- DB-DB derives the
	// friction tangent at solve time from the tangential relative position
	// delta inside solveDbDbContact.
	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrepDbDb(const Block& block, PxU32 lane)
	{
		const float4 normal_pen = block.normal_pen[lane];
		normal = PxVec3(normal_pen.x, normal_pen.y, normal_pen.z);
		initPen = normal_pen.w;
	}

	// Convert the resolved normal/friction multipliers into a deformable position
	// delta to scatter via writeSoftBody / writeCloth. Returns the accumulated
	// normal multiplier the caller pipes back into appliedForces.
	PX_FORCE_INLINE __device__ PxReal computeDeformableDelta(PxVec3& deltaPos, PxReal dt) const
	{
		deltaPos = -(deltaLambdaN * normal + deltaLambdaT * tangent) * dt;
		return accumulatedDeltaLambdaN;
	}
};

// The rigid side of a deformable contact/attachment constraint. Carries the
// rigid body properties, the prep-time block reads, and the live rigid state.
// Operated on by solveRbDbContact (RB-DB contact), solveRbDbAttachment
// (RB-DB attach), and solveRbPcContact (particle-rigid contact, in
// particlesystem.cu).
struct PxgRigidPart
{
	// --- Body properties (populated by readBodyProperties) ---
	PxU32 referenceCount = 1;
	PxReal friction = 0.0f;
	PxI32 fricCombineMode;

	// --- Contact-prep cache (populated by readContactPrep) ---
	PxVec3 raXn;
	PxVec3 raXnF0;
	PxVec3 raXnF1;
	PxReal raXnResp = 0.0f;
	PxReal raXnF0Resp = 0.0f;
	PxReal raXnF1Resp = 0.0f;
	PxReal invMass = 0.0f;
	PxReal penBiasClamp = 0.0f;

	// --- Attachment-prep cache (populated by readAttachmentPrep) ---
	// Per-axis (X/Y/Z): three angular-jacobian rows ra x e_axis (where ra is
	// the lever from the rigid CoM to the attach point), three position
	// errors, three solver multipliers, plus rigid invMass and the
	// attachment-bucket refCount. Unused by the contact kernels (DCE
	// eliminates the registers).
	PxVec3 raXn0;
	PxVec3 raXn1;
	PxVec3 raXn2;
	PxVec3 positionErrorXYZ;
	PxVec3 velMultiplierXYZ;
	PxReal attachInvMass = 0.0f;
	PxU32  attachRefCount = 1;

	// --- Live rigid-body state (populated by readVelocity) ---
	// linDelta / angDelta are TGS-only; readVelocity zeros them on PGS so the
	// solver math runs without an isTGS branch around the delta terms.
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
		penBiasClamp = -PX_MAX_F32;  // unbounded: a particle-static-mesh contact has no maxDepenetrationVelocity
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
	// path (readContactPrep), so invMass == 0 folds in static / kinematic / inactive
	// into a zeroed count-0 no-op. Otherwise convert the resolved multipliers to a
	// rigid velocity delta and write with count 1.
	//
	// refCount scales lin AND ang symmetrically: the finalize divides both by
	// max(1, .w) and the attachment writeback inflates both, so the contact path
	// must match. Per-link bucketing of articulation referenceCount
	// (getGlobalRigidBodyId) keeps refCount equal to the per-link write count,
	// matching the per-link finalize divisor.
	PX_FORCE_INLINE __device__ void writeContactDeltas(float4* rigidDeltaVel, const PxNodeIndex& rigidId,
													   const PxgDbContactState& state, PxU32 linVelIndex, PxU32 angVelIndex)
	{
		PxVec3 deltaLinVel(0.0f), deltaAngVel(0.0f);
		PxReal count = 0.0f;
		if(invMass != 0.0f && !rigidId.isStaticBody())
		{
			count = 1.0f;
			const PxReal refCount = static_cast<PxReal>(referenceCount);
			deltaAngVel = (state.raXn * state.deltaLambdaN + state.raXt * state.deltaLambdaT) * refCount;
			deltaLinVel = (state.normal * state.deltaLambdaN + state.tangent * state.deltaLambdaT) * invMass * refCount;
		}
		writeDeltas(rigidDeltaVel, deltaLinVel, deltaAngVel, linVelIndex, angVelIndex, count);
	}

	// Pulls the rigid-side prep cache out of the warp-packed constraint block:
	// the angular-jacobian rows ra x {n, t0, t1}, their unit responses,
	// invMass, and penBiasClamp.
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
		penBiasClamp = block.maxPenBiasClamp[lane];
	}

	// Overload for the particle-rigid block, which has no penBiasClamp slot: the rigid side gets the
	// -1e32 no-op sentinel; the particle side carries its own clamp, set inline by the solver.
	PX_FORCE_INLINE __device__ void readContactPrep(const PxgParticleRigidContactBlock& block, PxU32 lane)
	{
		const float4 raXn_resp_lane = block.raXn_resp[lane];
		raXn = PxVec3(raXn_resp_lane.x, raXn_resp_lane.y, raXn_resp_lane.z);
		raXnResp = raXn_resp_lane.w;

		const float4 raXnF0_resp_lane = block.raXnF0_resp[lane];
		raXnF0 = PxVec3(raXnF0_resp_lane.x, raXnF0_resp_lane.y, raXnF0_resp_lane.z);
		raXnF0Resp = raXnF0_resp_lane.w;

		const float4 raXnF1_resp_lane = block.raXnF1_resp[lane];
		raXnF1 = PxVec3(raXnF1_resp_lane.x, raXnF1_resp_lane.y, raXnF1_resp_lane.z);
		raXnF1Resp = raXnF1_resp_lane.w;

		invMass = block.fricTan0_invMass0[lane].w;
		penBiasClamp = block.penBiasClampRigid[lane];
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

	// Reads live rigid velocity (and, on TGS, the position delta) into the
	// participant; readVelocitiesPGS zeroes linDelta / angDelta itself.
	PX_FORCE_INLINE __device__ void readVelocity(PxgVelocityReader& reader, PxNodeIndex rigidId, bool isTGS)
	{
		if (isTGS)
			reader.readVelocitiesTGS(rigidId, linVel, angVel, linDelta, angDelta);
		else
			reader.readVelocitiesPGS(rigidId, linVel, angVel, linDelta, angDelta);
	}

};

// The deformable side of a deformable contact/attachment constraint. Carries
// per-vertex invMasses (after mass-splitting), bc-weighted linear delta, and
// the read/write helpers for cloth (PxVec3, triangle or single vertex) and
// soft body (PxVec4, tet). Each instantiation uses one of readCloth /
// readSoftBody; the other is ill-typed for that Vec and never called.
template <typename Vec>
struct PxgDeformablePart
{
	PxReal friction = 0.0f;
	PxVec3 linDelta;
	Vec vertexInvMasses; // After mass-splitting.

	// --- Contact-prep cache (populated by readContactPrep) ---
	// `bc` is the solver-side bary (cloth-vertex collapses to (1,0,0) via
	// toSolverBc); used in solveRbDbContact for the bc^2*invMass denominator.
	Vec bc;
	PxReal penBiasClamp = 0.0f;

	// --- Attachment-prep cache (populated by readSoftBodyAttachment / readClothAttachment
	// for RB-DB attach, or readSoftBodyAttachmentDbDb / readClothAttachmentDbDb for DB-DB
	// attach) ---
	// attachPointInvMass:      raw bc^2*invMass (no refCount inflation). Used by
	//                          solveRbDbAttachment in the denomBias formula
	//                          (refN * raw - split), and by the RB-DB kernels'
	//                          isStaticBody kinematic-skip gate. Set by RB-DB
	//                          readers only; DB-DB readers leave it at 0.
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
		penBiasClamp = block.maxPenBiasClamp[lane];
	}

	// Side-tagged variants for two-sided blocks (PxgDbDbContactBlock), where
	// each side has its own barycentric slot. The pair max penBiasClamp is
	// shared, so both side readers load the same maxPenBiasClamp slot.
	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrepDbSide0(const Block& block, PxU32 lane)
	{
		toSolverBc(bc, block.barycentric0[lane]);
		penBiasClamp = block.maxPenBiasClamp[lane];
	}

	template <typename Block>
	PX_FORCE_INLINE __device__ void readContactPrepDbSide1(const Block& block, PxU32 lane)
	{
		toSolverBc(bc, block.barycentric1[lane]);
		penBiasClamp = block.maxPenBiasClamp[lane];
	}

	PX_FORCE_INLINE __device__ PxVec3 readCloth(const PxgFEMCloth& cloth, PxU32 elementId, const float4& bc,
												const PxsDeformableSurfaceMaterialData* const materials, bool countReferenceOnly)
	{
		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			vertexInvMasses = PxVec3(0.0f);
			return vertexInvMasses;
		}

		const float4* const PX_RESTRICT clothPosDeltas = cloth.mAccumulatedDeltaPos;
		float4 clothDelta;

		if(bc.w == 0) // Cloth triangle
		{
			const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
			const float4 pd0 = clothPosDeltas[triVertId.x];
			const float4 pd1 = clothPosDeltas[triVertId.y];
			const float4 pd2 = clothPosDeltas[triVertId.z];

			clothDelta = pd0 * bc.x + pd1 * bc.y + pd2 * bc.z;
			vertexInvMasses = PxVec3(pd0.w, pd1.w, pd2.w);

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
			clothDelta = clothPosDeltas[elementId];
			vertexInvMasses = PxVec3(clothDelta.w, 0.0f, 0.0f);

			if(!countReferenceOnly)
			{
				friction = materials ? cloth.mDynamicFrictions[elementId] : 0.0f;

				// Query the reference count for the cloth.
				const PxReal vertexRefCount = cloth.mDeltaPos[elementId].w;

				// Mass-splitting
				vertexInvMasses.x *= vertexRefCount;
			}
		}

		linDelta = PxVec3(clothDelta.x, clothDelta.y, clothDelta.z);

		return vertexInvMasses;
	}

	// elemIsVertex selects the scatter: a single vertex (elementId is the vertex
	// index; bc unread) or a triangle (elementId indexes mTriangleVertexIndices;
	// bc.xyz are the barycentric weights). Contact derives elemIsVertex from bc.w,
	// attachment from PxGetIsVertexType(baryOrType).
	PX_FORCE_INLINE __device__ void writeCloth(PxgFEMCloth& cloth, PxU32 elementId, const float4& bc, bool elemIsVertex, PxVec3 deltaPos)
	{
		if(elemIsVertex)
			updatePositionDeltaVtx(cloth.mDeltaPos, elementId, deltaPos, vertexInvMasses.x);
		else
			updatePositionDeltaTri(cloth.mDeltaPos, cloth.mTriangleVertexIndices[elementId], deltaPos,
				make_float4(vertexInvMasses.x * bc.x, vertexInvMasses.y * bc.y, vertexInvMasses.z * bc.z, 0.0f));
	}

	// Edge variant for the CC EE solver, where each side is a cloth edge.
	PX_FORCE_INLINE __device__ void writeClothEdge(PxgFEMCloth& cloth, PxU32 vertIdx0, PxU32 vertIdx1, PxVec3 deltaPos)
	{
		updatePositionDeltaEdge(cloth.mDeltaPos, vertIdx0, vertIdx1, deltaPos,
			vertexInvMasses.x * bc.x, vertexInvMasses.y * bc.y);
	}

	PX_FORCE_INLINE __device__ PxVec4 readSoftBody(const PxgSoftBody& softbody, PxU32 tetId, const float4& bc,
												 const PxsDeformableVolumeMaterialData* const materials, bool checkOnlyActivity)
	{
		if(tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			vertexInvMasses = PxVec4(0.0f);
			return vertexInvMasses;
		}

		const float4* const PX_RESTRICT posDeltas = softbody.mSimDeltaPos;

		const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
		const float4 pd0 = posDeltas[tetrahedronId.x];
		const float4 pd1 = posDeltas[tetrahedronId.y];
		const float4 pd2 = posDeltas[tetrahedronId.z];
		const float4 pd3 = posDeltas[tetrahedronId.w];

		const float4 softBodyDelta = pd0 * bc.x + pd1 * bc.y + pd2 * bc.z + pd3 * bc.w;
		linDelta = PxVec3(softBodyDelta.x, softBodyDelta.y, softBodyDelta.z);
		vertexInvMasses = PxVec4(pd0.w, pd1.w, pd2.w, pd3.w);

		if(!checkOnlyActivity)
		{
			const PxU16 globalMaterialIndex = softbody.mMaterialIndices[tetId];
			friction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

			// Query the reference count for soft body.
			PxVec4 vertexRefCount;
			vertexRefCount.x = softbody.mSimDelta[tetrahedronId.x].w;
			vertexRefCount.y = softbody.mSimDelta[tetrahedronId.y].w;
			vertexRefCount.z = softbody.mSimDelta[tetrahedronId.z].w;
			vertexRefCount.w = softbody.mSimDelta[tetrahedronId.w].w;

			// Mass-splitting
			vertexInvMasses = vertexInvMasses.multiply(vertexRefCount);
		}

		return vertexInvMasses;
	}

	// elemIsVertex selects the scatter: a single vertex (elementId is the vertex
	// index; bc unread) or a tet (elementId indexes mSimTetIndices; bc.xyzw are the
	// barycentric weights). Contacts are always tet; attachment derives the flag
	// from PxGetIsVertexType(baryOrType).
	PX_FORCE_INLINE __device__ void writeSoftBody(const PxgSoftBody& softbody, PxU32 elementId, const float4& bc, bool elemIsVertex, PxVec3 deltaPos)
	{
		if(elemIsVertex)
			updatePositionDeltaVtx(softbody.mSimDelta, elementId, deltaPos, vertexInvMasses.x);
		else
			updatePositionDeltaTet(softbody.mSimDelta, softbody.mSimTetIndices[elementId], deltaPos,
				make_float4(vertexInvMasses.x * bc.x, vertexInvMasses.y * bc.y, vertexInvMasses.z * bc.z, vertexInvMasses.w * bc.w));
	}

	// Attachment-side reads. Differ from readSoftBody / readCloth in:
	//  - Source buffer is PGS velocity (mSimVelocity_InvMass / mVelocity_InvMass)
	//    or TGS accumulated delta-pos (mSimDeltaPos / mAccumulatedDeltaPos).
	//  - `bary` and `elemIsVertex` MUST be derived from the same source.
	//    Callers compute `elemIsVertex = PxGetIsVertexType(bary)` -- which
	//    tests bary == (0,0,0,0), the all-zeros sentinel that marks a vertex
	//    attachment -- and pass both. `bary` is then either consulted as the
	//    element-side barycentric (element branch) or unread (vertex branch).
	//    Passing inconsistent values silently produces zero-effect reads
	//    (`elemIsVertex=false` + zeros bary) or drops the bary
	//    (`elemIsVertex=true` + non-zero bary).
	//  - vertexInvMasses are refCount-inflated from .w (pre-counted), so the
	//    scatter pairs with the finalize-time /.w divide.
	//  - attachPointInvMass      = raw bc^2*invM (D in denomBias formula).
	//    attachPointInvMassSplit = refCount-inflated bc^2*invM (D' in denomBias).

	PX_FORCE_INLINE __device__ void readSoftBodyAttachment(const PxgSoftBody& softbody, PxU32 elemIdx,
														   const float4& bary, bool elemIsVertex, bool isTGS)
	{
		const float4* const PX_RESTRICT source = isTGS ? softbody.mSimDeltaPos : softbody.mSimVelocity_InvMass;
		const float4* const PX_RESTRICT simDelta = softbody.mSimDelta;

		if(elemIsVertex)
		{
			const float4 s = source[elemIdx];
			const float refCount_v = simDelta[elemIdx].w;
			linDelta = PxVec3(s.x, s.y, s.z);
			vertexInvMasses = PxVec4(s.w * refCount_v, 0.0f, 0.0f, 0.0f);
			attachPointInvMass      = s.w;
			attachPointInvMassSplit = s.w * refCount_v;
			bc = PxVec4(1.0f, 0.0f, 0.0f, 0.0f);
		}
		else
		{
			const uint4 tetInd = softbody.mSimTetIndices[elemIdx];
			const float4 s0 = source[tetInd.x];
			const float4 s1 = source[tetInd.y];
			const float4 s2 = source[tetInd.z];
			const float4 s3 = source[tetInd.w];
			const float4 weighted = s0 * bary.x + s1 * bary.y + s2 * bary.z + s3 * bary.w;
			linDelta = PxVec3(weighted.x, weighted.y, weighted.z);

			const PxReal r0 = simDelta[tetInd.x].w;
			const PxReal r1 = simDelta[tetInd.y].w;
			const PxReal r2 = simDelta[tetInd.z].w;
			const PxReal r3 = simDelta[tetInd.w].w;

			vertexInvMasses = PxVec4(s0.w * r0, s1.w * r1, s2.w * r2, s3.w * r3);

			const PxReal bx2 = bary.x * bary.x;
			const PxReal by2 = bary.y * bary.y;
			const PxReal bz2 = bary.z * bary.z;
			const PxReal bw2 = bary.w * bary.w;
			attachPointInvMass      = bx2 * s0.w      + by2 * s1.w      + bz2 * s2.w      + bw2 * s3.w;
			attachPointInvMassSplit = bx2 * s0.w * r0 + by2 * s1.w * r1 + bz2 * s2.w * r2 + bw2 * s3.w * r3;
			bc = PxVec4(bary.x, bary.y, bary.z, bary.w);
		}
	}

	PX_FORCE_INLINE __device__ void readClothAttachment(const PxgFEMCloth& cloth, PxU32 elemIdx,
														const float4& bary, bool elemIsVertex, bool isTGS)
	{
		const float4* const PX_RESTRICT source = isTGS ? cloth.mAccumulatedDeltaPos : cloth.mVelocity_InvMass;
		const float4* const PX_RESTRICT deltaPos = cloth.mDeltaPos;

		if(elemIsVertex)
		{
			const float4 s = source[elemIdx];
			const float refCount_v = deltaPos[elemIdx].w;
			linDelta = PxVec3(s.x, s.y, s.z);
			vertexInvMasses = PxVec3(s.w * refCount_v, 0.0f, 0.0f);
			attachPointInvMass      = s.w;
			attachPointInvMassSplit = s.w * refCount_v;
			bc = PxVec3(1.0f, 0.0f, 0.0f);
		}
		else
		{
			const uint4 triInd = cloth.mTriangleVertexIndices[elemIdx];
			const float4 s0 = source[triInd.x];
			const float4 s1 = source[triInd.y];
			const float4 s2 = source[triInd.z];
			const float4 weighted = s0 * bary.x + s1 * bary.y + s2 * bary.z;
			linDelta = PxVec3(weighted.x, weighted.y, weighted.z);

			const PxReal r0 = deltaPos[triInd.x].w;
			const PxReal r1 = deltaPos[triInd.y].w;
			const PxReal r2 = deltaPos[triInd.z].w;

			vertexInvMasses = PxVec3(s0.w * r0, s1.w * r1, s2.w * r2);

			const PxReal bx2 = bary.x * bary.x;
			const PxReal by2 = bary.y * bary.y;
			const PxReal bz2 = bary.z * bary.z;
			attachPointInvMass      = bx2 * s0.w      + by2 * s1.w      + bz2 * s2.w;
			attachPointInvMassSplit = bx2 * s0.w * r0 + by2 * s1.w * r1 + bz2 * s2.w * r2;
			bc = PxVec3(bary.x, bary.y, bary.z);
		}
	}

	// DB-DB attachment read: source is the live position buffer, not a delta.
	// Returns the bc-weighted position so the caller computes error = pos1 -
	// pos0 directly. vertexInvMasses + attachPointInvMassSplit are refCount-
	// inflated (refCount pre-counted by sb_querySoftBodyAttachment /
	// sb_queryClothAttachment / cloth_queryClothClothAttachment ReferenceCount
	// Launch onto mSimDelta[v].w / mDeltaPos[v].w). Scatter writes .xyz only
	// and pairs with the finalize /.w. attachPointInvMass (raw) left at 0 --
	// DB-DB has no denomBias formula so the raw scalar has no consumer here.
	PX_FORCE_INLINE __device__ PxVec3 readSoftBodyAttachmentDbDb(const PxgSoftBody& softbody, PxU32 tetId, const float4& bary)
	{
		const float4* const PX_RESTRICT simPos = softbody.mSimPosition_InvMass;
		const float4* const PX_RESTRICT simDelta = softbody.mSimDelta;
		const uint4 tetInd = softbody.mSimTetIndices[tetId];
		const float4 p0 = simPos[tetInd.x];
		const float4 p1 = simPos[tetInd.y];
		const float4 p2 = simPos[tetInd.z];
		const float4 p3 = simPos[tetInd.w];
		const float4 weighted = p0 * bary.x + p1 * bary.y + p2 * bary.z + p3 * bary.w;

		const PxReal r0 = simDelta[tetInd.x].w;
		const PxReal r1 = simDelta[tetInd.y].w;
		const PxReal r2 = simDelta[tetInd.z].w;
		const PxReal r3 = simDelta[tetInd.w].w;

		vertexInvMasses = PxVec4(p0.w * r0, p1.w * r1, p2.w * r2, p3.w * r3);
		bc = PxVec4(bary.x, bary.y, bary.z, bary.w);

		const PxReal bx2 = bary.x * bary.x;
		const PxReal by2 = bary.y * bary.y;
		const PxReal bz2 = bary.z * bary.z;
		const PxReal bw2 = bary.w * bary.w;
		attachPointInvMassSplit = bx2 * p0.w * r0 + by2 * p1.w * r1 + bz2 * p2.w * r2 + bw2 * p3.w * r3;

		return PxVec3(weighted.x, weighted.y, weighted.z);
	}

	PX_FORCE_INLINE __device__ PxVec3 readClothAttachmentDbDb(const PxgFEMCloth& cloth, PxU32 elementId, const float4& bary)
	{
		const float4* const PX_RESTRICT clothPos = cloth.mPosition_InvMass;
		const float4* const PX_RESTRICT deltaPos = cloth.mDeltaPos;
		const uint4 triInd = cloth.mTriangleVertexIndices[elementId];
		const float4 p0 = clothPos[triInd.x];
		const float4 p1 = clothPos[triInd.y];
		const float4 p2 = clothPos[triInd.z];
		const float4 weighted = p0 * bary.x + p1 * bary.y + p2 * bary.z;

		const PxReal r0 = deltaPos[triInd.x].w;
		const PxReal r1 = deltaPos[triInd.y].w;
		const PxReal r2 = deltaPos[triInd.z].w;

		vertexInvMasses = PxVec3(p0.w * r0, p1.w * r1, p2.w * r2);
		bc = PxVec3(bary.x, bary.y, bary.z);

		const PxReal bx2 = bary.x * bary.x;
		const PxReal by2 = bary.y * bary.y;
		const PxReal bz2 = bary.z * bary.z;
		attachPointInvMassSplit = bx2 * p0.w * r0 + by2 * p1.w * r1 + bz2 * p2.w * r2;

		return PxVec3(weighted.x, weighted.y, weighted.z);
	}
};

// Solves a single deformable-vs-rigid contact for the normal + tangent
// (friction) multipliers. PGS and TGS share the body: readVelocity zeros
// rigid.linDelta / rigid.angDelta on PGS so the position-delta terms drop
// out without an isTGS branch. Fills state.tangent, raXn, raXt, deltaLambda*,
// accumulatedDeltaLambdaN; returns activity (checkOnlyActivity) or success.
template <typename Vec>
PX_FORCE_INLINE __device__ bool solveRbDbContact(PxgRigidPart& rigid, PxgDeformablePart<Vec>& db, PxgDbContactState& state,
												 PxReal appliedForceRef, PxReal dt,
												 bool wasActive, bool checkOnlyActivity)
{
	state.accumulatedDeltaLambdaN = appliedForceRef;

	const PxReal threshold = 1.0e-14f;
	const PxReal invDt = 1.0f / dt;

	state.raXn = rigid.raXn;

	const PxVec3 relLinDelta = rigid.linDelta - db.linDelta;
	const PxReal normalVel = rigid.linVel.dot(state.normal) + rigid.angVel.dot(state.raXn);
	const PxReal error = (state.initPen + relLinDelta.dot(state.normal) + rigid.angDelta.dot(state.raXn)) * invDt;

	// Both Parts carry the same pair-max clamp (baked at prep into
	// maxPenBiasClamp), so either side is equivalent here.
	const PxReal errorBiased = PxMax(rigid.penBiasClamp, error);
	const PxReal CN = errorBiased + normalVel;

	const bool isActive = wasActive || CN < 0.0f;
	state.deltaLambdaN = 0.0f;

	if(checkOnlyActivity)
	{
		return isActive;
	}

	if(!isActive)
	{
		return false;
	}

	// Deformable body term in the denominator of the impulse calculation. Also, refer to delta lambda in the XPBD paper.
	const PxReal deformableInvMass_massSplitting = db.bc.multiply(db.bc).dot(db.vertexInvMasses);

	const PxReal rigidRefCount = static_cast<PxReal>(rigid.referenceCount);
	const PxReal unitResponse = rigidRefCount * rigid.raXnResp + deformableInvMass_massSplitting;
	const PxReal invDenom = (unitResponse > 0.0f) ? (1.0f / unitResponse) : 0.0f;

	state.deltaLambdaN = PxMax(-CN * invDenom, -state.accumulatedDeltaLambdaN);
	state.accumulatedDeltaLambdaN += state.deltaLambdaN;

	// Friction: tangent0 is pair-level (prep-cached); tangent1 = normal x tangent0.
	// raXnF0 / raXnF1 are the corresponding angular-jacobian rows (ra x tangent).
	const PxVec3 fric0 = state.tangent0;
	const PxVec3 fric1 = state.normal.cross(fric0);

	const PxReal tanVel0 = rigid.linVel.dot(fric0) + rigid.angVel.dot(rigid.raXnF0);
	const PxReal tanVel1 = rigid.linVel.dot(fric1) + rigid.angVel.dot(rigid.raXnF1);

	const PxReal CT0 = (fric0.dot(relLinDelta) + rigid.angDelta.dot(rigid.raXnF0)) * invDt + tanVel0;
	const PxReal CT1 = (fric1.dot(relLinDelta) + rigid.angDelta.dot(rigid.raXnF1)) * invDt + tanVel1;
	const PxVec3 relTanDelta = CT0 * fric0 + CT1 * fric1;
	const PxReal tanMagSq = relTanDelta.magnitudeSquared();

	if(tanMagSq > threshold)
	{
		const PxReal CT = PxSqrt(tanMagSq);
		const PxReal invTanMag = 1.0f / CT;
		state.tangent = relTanDelta * invTanMag;

		const PxReal frac0 = state.tangent.dot(fric0);
		const PxReal frac1 = state.tangent.dot(fric1);
		state.raXt = frac0 * rigid.raXnF0 + frac1 * rigid.raXnF1;

		// Using two precomputed orthonormal tangent directions.
		const PxReal unitResponseT0 = rigidRefCount * rigid.raXnF0Resp + deformableInvMass_massSplitting;
		const PxReal invTanDenom0 = (unitResponseT0 > 0.0f) ? (1.0f / unitResponseT0) : 0.0f;

		const PxReal unitResponseT1 = rigidRefCount * rigid.raXnF1Resp + deformableInvMass_massSplitting;
		const PxReal invTanDenom1 = (unitResponseT1 > 0.0f) ? (1.0f / unitResponseT1) : 0.0f;

		PxReal deltaLambdaT0 = CT0 * invTanDenom0;
		PxReal deltaLambdaT1 = CT1 * invTanDenom1;

		const PxReal combinedFriction = combineScalars(rigid.friction, db.friction, rigid.fricCombineMode);

		state.deltaLambdaT = PxSqrt(deltaLambdaT0 * deltaLambdaT0 + deltaLambdaT1 * deltaLambdaT1);
		state.deltaLambdaT = -PxMin(state.deltaLambdaT, combinedFriction * PxAbs(state.deltaLambdaN));

		assert(state.deltaLambdaT <= 0.0f);
	}

	return true;
}

// Solves a single deformable-vs-deformable contact for the normal + tangent
// (friction) multipliers. Pair-level normal + initPen come from
// state.readContactPrepDbDb; per-side bc / penBiasClamp come from
// readContactPrepDbSide{0,1}; vertexInvMasses / linDelta / friction from
// readSoftBody / readCloth.
//
// Sign conventions:
// - state.normal points db0 -> db1 (stored at prep as -narrowphase).
// - state.deltaLambdaN > 0 for an unresolved penetration. computeDeformableDelta
//   returns deltaPos = -(lambdaN * normal + lambdaT * tangent) * dt; the caller
//   applies +deltaPos to db0 and -deltaPos to db1.
// - state.tangent is derived at solve time from the tangential component of
//   relLinDelta. state.deltaLambdaT <= 0.
//
// Asymmetric Vec0 / Vec1: each participant carries its own Vec independently;
// the body only uses bc.multiply(bc).dot(vertexInvMasses) per side.
//
// checkOnlyActivity: when true, returns immediately after computing CN
// without touching deltaLambda*. Returns true iff CN < 0.
template <typename Vec0, typename Vec1>
PX_FORCE_INLINE __device__ bool solveDbDbContact(PxgDeformablePart<Vec0>& db0, PxgDeformablePart<Vec1>& db1,
												   PxgDbContactState& state,
												   PxReal appliedNormalLambdaRef, PxReal appliedTanLambdaRef,
												   PxReal frictionCoefficient,
												   PxReal dt,
												   bool checkOnlyActivity = false)
{
	state.accumulatedDeltaLambdaN = appliedNormalLambdaRef;
	state.deltaLambdaN = 0.0f;
	state.deltaLambdaT = 0.0f;
	const PxReal invDt = 1.0f / dt;

	const PxVec3 relLinDelta = db1.linDelta - db0.linDelta;
	const PxReal error = (state.initPen + relLinDelta.dot(state.normal)) * invDt;

	// Both Parts carry the same pair-max clamp (baked at prep), so either
	// side is equivalent.
	const PxReal errorBiased = PxMax(db0.penBiasClamp, error);
	const PxReal CN = errorBiased; // Position-domain: no live-velocity term.

	const bool isActive = CN < 0.0f;
	if(checkOnlyActivity)
		return isActive;

	if(!isActive)
		return false;

	// XPBD unit response: bc^2*invMass per side (after mass-splitting), both
	// sides contributing additively.
	const PxReal db0Resp = db0.bc.multiply(db0.bc).dot(db0.vertexInvMasses);
	const PxReal db1Resp = db1.bc.multiply(db1.bc).dot(db1.vertexInvMasses);
	const PxReal unitResponse = db0Resp + db1Resp;
	const PxReal invDenom = (unitResponse > 1e-16f) ? (1.0f / unitResponse) : 0.0f;

	state.deltaLambdaN = PxMax(-CN * invDenom, -state.accumulatedDeltaLambdaN);
	state.accumulatedDeltaLambdaN += state.deltaLambdaN;

	// Friction: solve-time tangent from the tangential component of
	// relLinDelta. (Differs from solveRbDbContact, which uses a prep-time
	// tangent0 + rigid ra x tangent jacobian rows; state.tangent0 is
	// unused here.)
	const PxReal normalProj = relLinDelta.dot(state.normal);
	PxVec3 tanDir = relLinDelta - state.normal * normalProj;
	const PxReal tanDelta = tanDir.normalize();
	state.tangent = tanDir;

	// Coulomb friction in lambda units. The minus sign yields deltaLambdaT <= 0
	// -- the impulse opposes drift along state.tangent.
	const PxReal requiredLambdaTNeg = -tanDelta * invDt * invDenom;
	const PxReal frictionLimit = state.accumulatedDeltaLambdaN * frictionCoefficient; // >= 0 (lambdaN >= 0).
	const PxReal targetAccumLambdaT = PxMax(appliedTanLambdaRef + requiredLambdaTNeg, -frictionLimit);
	state.deltaLambdaT = targetAccumLambdaT - appliedTanLambdaRef;

	assert(state.deltaLambdaT <= 0.0f);
	return true;
}

// Solves a single rigid-vs-deformable attachment for the per-axis impulse.
// Returns deltaImpulse for the caller's deformable-side writeback;
// outDeltaLinVel / outDeltaAngVel are for the rigid-side writeback (the
// caller does the refN inflation + 2-slot scatter).
//
// Attachment vs. contact math:
//  - Per-axis (X/Y/Z) constraint with three independent scalar lambdas; not
//    a normal+tangent decomposition.
//  - biasCoefficient softening retained; not clamped to maxPenBias.
//  - No friction term.
//  - Rigid-side mass-splitting via rigid.attachRefCount (attachment-bucketed,
//    distinct from contact's per-link-bucketed rigid.referenceCount).
//
// PGS / TGS unified: PGS readVelocity zeros rigid.linDelta / rigid.angDelta so
// the TGS position-domain terms collapse. The remaining isTGS branch handles
// the math-shape divergence (PGS works in velocity units; TGS in position-
// delta units, scaled by invDt at the end).
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
		// TGS: db.linDelta is bc-weighted accumulated position delta;
		// rigid.linDelta / rigid.angDelta are the rigid's accumulated deltas.
		const PxVec3 linDelta = db.linDelta - rigid.linDelta;
		const PxVec3 tgsErrVel(
			(linDelta.x - rigid.angDelta.dot(rigid.raXn0) - rigid.positionErrorXYZ.x * biasCoefficient) * invDt,
			(linDelta.y - rigid.angDelta.dot(rigid.raXn1) - rigid.positionErrorXYZ.y * biasCoefficient) * invDt,
			(linDelta.z - rigid.angDelta.dot(rigid.raXn2) - rigid.positionErrorXYZ.z * biasCoefficient) * invDt);
		const PxReal velIterScale = isVelocityIteration ? 0.0f : 1.0f;
		deltaF0 = (tgsErrVel.x - velOfRigidAtAttachmentPointX * velIterScale) * adjVelMult0;
		deltaF1 = (tgsErrVel.y - velOfRigidAtAttachmentPointY * velIterScale) * adjVelMult1;
		deltaF2 = (tgsErrVel.z - velOfRigidAtAttachmentPointZ * velIterScale) * adjVelMult2;
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
// outDelta = error / wTot. Pair-symmetric -- callers apply +outDelta to side
// 0 and -outDelta to side 1 via .xyz-only scatter; finalize-time /.w cancels
// the inflation. Returns false iff both sides are kinematic (wTot == 0).
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

#endif // __DEFORMABLE_UTILS_CUH__
