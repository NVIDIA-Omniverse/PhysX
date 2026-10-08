// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DEFORMABLE_UTILS_CUH__
#define __DEFORMABLE_UTILS_CUH__

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

} // namespace physx

#endif // __DEFORMABLE_UTILS_CUH__
