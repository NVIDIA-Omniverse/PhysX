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

#ifndef __DEFORMABLE_UTILS_CUH__
#define __DEFORMABLE_UTILS_CUH__

#include "foundation/PxMathUtils.h"
#include "PxsMaterialCombiner.h"
#include "PxgFEMCore.h"
#include "PxgFEMCloth.h"
#include "PxgSoftBody.h"
#include "PxgArticulation.h"
#include "PxgBodySim.h"
#include "dataReadWriteHelper.cuh"

using namespace physx;

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
		const PxQuat tempQ = PxQuat(w, omega);
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

PX_FORCE_INLINE __device__ void prepareFEMContacts(PxgFemRigidConstraintBlock& constraint, const PxVec3& normal,
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

	PxReal maxPenBias = fmaxf(penBiasClampRigid, penBiasClampFEM);
	PxReal error = pen + delta.dot(normal);

	// KS - TODO - split these into 5 separate vectors to promote coalesced memory accesses!
	constraint.normal_errorW[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, error);
	constraint.raXn_resp[threadIndexInWarp] = raXn_resp;
	constraint.raXnF0_resp[threadIndexInWarp] = raXnF0_resp;
	constraint.raXnF1_resp[threadIndexInWarp] = raXnF1_resp;
	constraint.fricTan0_invMass0[threadIndexInWarp] = make_float4(t0.x, t0.y, t0.z, invMass0);
	constraint.maxPenBias[threadIndexInWarp] = maxPenBias;
	constraint.barycentric[threadIndexInWarp] = barycentric;
}

// Vec: PxVec3 for triangles and PxVec4 for tetrahedra.
template <typename Vec>
struct FEMCollision
{
	bool isTGS = true;

	// Rigid body
	PxU32 rigidBodyReferenceCount = 1;
	PxReal rigidBodyFriction = 0.0f;
	PxI32 frictionCombineMode;

	// Deformable body
	PxReal deformableFriction = 0.0f;
	PxVec3 deformableLinDelta;
	Vec deformableVertexInvMasses; // After mass-splitting.

	// Constraint
	PxVec3 normal = PxVec3(0.0f);
	PxVec3 tangent = PxVec3(0.0f);
	PxVec3 raXn = PxVec3(0.0f); 
	PxVec3 raXt = PxVec3(0.0f);
	PxReal deltaLambdaN = 0.0f;
	PxReal deltaLambdaT = 0.0f;
	PxReal accumulatedDeltaLambdaN = 0.0f;

	PX_FORCE_INLINE __device__ bool initialize(const float4& fricTan0_invMass0, PxgFemRigidConstraintBlock& constraint,
											   PxReal appliedForceRef, PxNodeIndex rigidId, PxgVelocityReader& velocityReader, PxReal dt,
											   const Vec& bc, bool wasActive, bool checkOnlyActivity)
	{
		// PxgFemRigidConstraintBlock is packed with arrays with size 32.
		assert(blockDim.x % 32 == 0 && blockDim.y == 1 && blockDim.z == 1);

		// PBD way of appying constraints
		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		accumulatedDeltaLambdaN = appliedForceRef;

		const PxReal threshold = 1.0e-14f;
		const PxReal invDt = 1.0f / dt;

		const float4 raXn_resp = constraint.raXn_resp[threadIndexInWarp];
		const float4 normal_biasW = constraint.normal_errorW[threadIndexInWarp];
		const float4 raXnF0_resp = constraint.raXnF0_resp[threadIndexInWarp];
		const float4 raXnF1_resp = constraint.raXnF1_resp[threadIndexInWarp];
		const PxReal maxPenBias = constraint.maxPenBias[threadIndexInWarp];

		normal = PxVec3(normal_biasW.x, normal_biasW.y, normal_biasW.z);
		const PxVec3 fric0 = PxVec3(fricTan0_invMass0.x, fricTan0_invMass0.y, fricTan0_invMass0.z);
		const PxVec3 fric1 = normal.cross(fric0);

		const float initPen = normal_biasW.w;

		raXn = PxVec3(raXn_resp.x, raXn_resp.y, raXn_resp.z);

		PxReal CN;
		PxReal normalVel;
		PxVec3 relLinDelta;
		PxVec3 angDelta(0.0f);
		PxVec3 linVel;
		PxVec3 angVel;

		if(isTGS)
		{
			PxgVelocityPackTGS rigidStateVec;
			velocityReader.readVelocitiesTGS(rigidId, rigidStateVec);

			linVel = rigidStateVec.linVel;
			angVel = rigidStateVec.angVel;

			normalVel = linVel.dot(normal) + angVel.dot(raXn);
			relLinDelta = rigidStateVec.linDelta - deformableLinDelta;
			angDelta = rigidStateVec.angDelta;

			const PxReal error = (initPen + relLinDelta.dot(normal) + angDelta.dot(raXn)) * invDt;

			// maxPenBias is negative.
			const PxReal errorBiased = PxMax(maxPenBias, error);

			CN = errorBiased + normalVel;
		}
		else
		{
			PxgVelocityPackPGS rigidStateVec;
			velocityReader.readVelocitiesPGS(rigidId, rigidStateVec);

			linVel = rigidStateVec.linVel;
			angVel = rigidStateVec.angVel;

			normalVel = linVel.dot(normal) + angVel.dot(raXn);
			relLinDelta = -deformableLinDelta;

			const PxReal error = (initPen + relLinDelta.dot(normal)) * invDt;

			// maxPenBias is negative.
			const PxReal errorBiased = PxMax(maxPenBias, error);

			CN = errorBiased + normalVel;
		}

		const bool isActive = wasActive || CN < 0.0f;
		deltaLambdaN = 0.0f;

		if(checkOnlyActivity)
		{
			return isActive;
		}

		if(!isActive)
		{
			return false;
		}

		// Deformable body term in the denominator of the impulse calculation. Also, refer to delta lambda in the XPBD paper.
		const PxReal deformableInvMass_massSplitting = bc.multiply(bc).dot(deformableVertexInvMasses);

		const PxReal rigidRefCount = static_cast<PxReal>(rigidBodyReferenceCount);
		const PxReal unitResponse = rigidRefCount * raXn_resp.w + deformableInvMass_massSplitting;
		const PxReal invDenom = (unitResponse > 0.0f) ? (1.0f / unitResponse) : 0.0f;

		deltaLambdaN = PxMax(-CN * invDenom, -accumulatedDeltaLambdaN);
		accumulatedDeltaLambdaN += deltaLambdaN;

		// Friction constraint in the tangent direction.
		const PxVec3 raXnF0 = PxVec3(raXnF0_resp.x, raXnF0_resp.y, raXnF0_resp.z);
		const PxVec3 raXnF1 = PxVec3(raXnF1_resp.x, raXnF1_resp.y, raXnF1_resp.z);

		const float tanVel0 = linVel.dot(fric0) + angVel.dot(raXnF0);
		const float tanVel1 = linVel.dot(fric1) + angVel.dot(raXnF1);

		const PxReal CT0 = (fric0.dot(relLinDelta) + angDelta.dot(raXnF0)) * invDt + tanVel0;
		const PxReal CT1 = (fric1.dot(relLinDelta) + angDelta.dot(raXnF1)) * invDt + tanVel1;
		const PxVec3 relTanDelta = CT0 * fric0 + CT1 * fric1;
		const PxReal tanMagSq = relTanDelta.magnitudeSquared();

		if(tanMagSq > threshold)
		{
			const PxReal CT = PxSqrt(tanMagSq);
			const PxReal invTanMag = 1.0f / CT;
			tangent = relTanDelta * invTanMag;

			const PxReal frac0 = tangent.dot(fric0);
			const PxReal frac1 = tangent.dot(fric1);
			raXt = frac0 * raXnF0 + frac1 * raXnF1;

			// Using two precomputed orthonormal tangent directions.
			const PxReal unitResponseT0 = rigidRefCount * raXnF0_resp.w + deformableInvMass_massSplitting;
			const PxReal invTanDenom0 = (unitResponseT0 > 0.0f) ? (1.0f / unitResponseT0) : 0.0f;

			const PxReal unitResponseT1 = rigidRefCount * raXnF1_resp.w + deformableInvMass_massSplitting;
			const PxReal invTanDenom1 = (unitResponseT1 > 0.0f) ? (1.0f / unitResponseT1) : 0.0f;

			PxReal deltaLambdaT0 = CT0 * invTanDenom0;
			PxReal deltaLambdaT1 = CT1 * invTanDenom1;

			deltaLambdaT = PxSqrt(deltaLambdaT0 * deltaLambdaT0 + deltaLambdaT1 * deltaLambdaT1);
			deltaLambdaT = -PxMin(deltaLambdaT, getCombinedFriction() * PxAbs(deltaLambdaN));

			assert(deltaLambdaT <= 0.0f);
		}

		return true;
	}

	PX_FORCE_INLINE __device__ PxReal computeRigidChange(PxVec3& deltaLinVel0, PxVec3& deltaAngVel0, const PxNodeIndex& rigidId,
														 PxReal rigidInvMass)
	{
		const PxReal rigidRefCount = static_cast<PxReal>(rigidBodyReferenceCount);

		deltaAngVel0 = rigidId.isArticulation() ? raXn * deltaLambdaN + raXt * deltaLambdaT
												: (raXn * deltaLambdaN + raXt * deltaLambdaT) * rigidRefCount;

		deltaLinVel0 = (normal * deltaLambdaN + tangent * deltaLambdaT) * rigidInvMass * rigidRefCount;

		return accumulatedDeltaLambdaN;
	}

	PX_FORCE_INLINE __device__ PxReal computeFEMChange(PxVec3& deltaPos, PxReal dt)
	{
		deltaPos = -(deltaLambdaN * normal + deltaLambdaT * tangent) * dt;
		return accumulatedDeltaLambdaN;
	}

	PX_FORCE_INLINE __device__ PxReal getCombinedFriction()
	{
		return PxsCombinePxReal(rigidBodyFriction, deformableFriction, frictionCombineMode);
	}

	PX_FORCE_INLINE __device__ int getGlobalRigidBodyId(const PxgPrePrepDesc* const prePrepDesc, const PxNodeIndex& rigidId,
														PxU32 numSolverBodies)
	{
		// Following PxgVelocityReader style to read rigid body indices.
		if(rigidId.isStaticBody())
		{
			return -1;
		}

		const PxU32 solverBodyIdx = prePrepDesc->solverBodyIndices[rigidId.index()];

		// Placing articulation indices at the end of rigid body indices to distinguish between rigid body reference counts and
		// articulation reference counts.
		return rigidId.isArticulation() ? static_cast<int>(numSolverBodies + solverBodyIdx) : static_cast<int>(solverBodyIdx);
	}

	PX_FORCE_INLINE __device__ void readRigidBody(const PxNodeIndex& rigidId, int globalRigidBodyId, PxReal rigidInvMass,
												  const PxU32* const rigidBodyReferenceCounts, const PxsMaterialData* rigidMaterial)
	{
		rigidBodyReferenceCount = 1;

		// Query the reference count for the rigid body.
		if(rigidBodyReferenceCounts && globalRigidBodyId != -1 && rigidInvMass != 0.0f)
		{
			rigidBodyReferenceCount = rigidBodyReferenceCounts[globalRigidBodyId];
		}

		if(rigidMaterial != NULL)
		{
			rigidBodyFriction = rigidMaterial->dynamicFriction;
			frictionCombineMode = rigidMaterial->fricCombineMode;
		}
		else
		{
			rigidBodyFriction = 0.0f;
			frictionCombineMode = PxCombineMode::eMAX;
		}
	}

	PX_FORCE_INLINE __device__ void writeRigidBody(float4* rigidDeltaVel, const PxVec3& deltaLinVel0, const PxVec3& deltaAngVel0,
												   PxU32 workIndex0, PxU32 workIndex1, PxReal count)
	{
		rigidDeltaVel[workIndex0] = make_float4(deltaLinVel0.x, deltaLinVel0.y, deltaLinVel0.z, count);
		rigidDeltaVel[workIndex1] = make_float4(deltaAngVel0.x, deltaAngVel0.y, deltaAngVel0.z, 0.f);
	}

	PX_FORCE_INLINE __device__ PxVec3 readCloth(const PxgFEMCloth& cloth, PxU32 elementId, const float4& bc,
												const PxsDeformableSurfaceMaterialData* const materials, bool countReferenceOnly)
	{
		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			deformableVertexInvMasses = PxVec3(0.0f);
			return deformableVertexInvMasses;
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
			deformableVertexInvMasses = PxVec3(pd0.w, pd1.w, pd2.w);

			if(!countReferenceOnly)
			{
				const PxU16 globalMaterialIndex = cloth.mMaterialIndices[elementId];
				deformableFriction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

				// Query the reference count for the cloth.
				PxVec3 deformableVertexReferenceCount;
				deformableVertexReferenceCount.x = cloth.mDeltaPos[triVertId.x].w;
				deformableVertexReferenceCount.y = cloth.mDeltaPos[triVertId.y].w;
				deformableVertexReferenceCount.z = cloth.mDeltaPos[triVertId.z].w;

				// Mass-splitting
				deformableVertexInvMasses = deformableVertexInvMasses.multiply(deformableVertexReferenceCount);
			}
		}
		else // Cloth vertex
		{
			clothDelta = clothPosDeltas[elementId];
			deformableVertexInvMasses = PxVec3(clothDelta.w, 0.0f, 0.0f);

			if(!countReferenceOnly)
			{
				deformableFriction = materials ? cloth.mDynamicFrictions[elementId] : 0.0f;

				// Query the reference count for the cloth.
				const PxReal deformableVertexReferenceCount = cloth.mDeltaPos[elementId].w;

				// Mass-splitting
				deformableVertexInvMasses.x *= deformableVertexReferenceCount;
			}
		}

		deformableLinDelta = PxVec3(clothDelta.x, clothDelta.y, clothDelta.z);

		return deformableVertexInvMasses;
	}

	PX_FORCE_INLINE __device__ void writeCloth(PxgFEMCloth& cloth, PxU32 elementId, const float4& bc, PxVec3 deltaPos)
	{
		if(bc.w == 0.f) // Cloth triangle
		{
			const float* bcPtr = reinterpret_cast<const float*>(&bc.x);
			const uint4 triVertInds = cloth.mTriangleVertexIndices[elementId];
			const PxU32* triVertices = reinterpret_cast<const PxU32*>(&triVertInds.x);

#pragma unroll
			for(PxU32 it = 0; it < 3; ++it)
			{
				if(deformableVertexInvMasses[it] > 0.0f)
				{
					const PxVec3 dP = deltaPos * (deformableVertexInvMasses[it] * bcPtr[it]);
					atomicAdd(&cloth.mDeltaPos[triVertices[it]].x, dP.x);
					atomicAdd(&cloth.mDeltaPos[triVertices[it]].y, dP.y);
					atomicAdd(&cloth.mDeltaPos[triVertices[it]].z, dP.z);
				}
			}
		}
		else // Cloth vertex
		{
			if(deformableVertexInvMasses.x > 0.0f)
			{
				const PxVec3 dP = deltaPos * deformableVertexInvMasses.x;
				atomicAdd(&cloth.mDeltaPos[elementId].x, dP.x);
				atomicAdd(&cloth.mDeltaPos[elementId].y, dP.y);
				atomicAdd(&cloth.mDeltaPos[elementId].z, dP.z);
			}
		}
	}

	PX_FORCE_INLINE __device__ PxVec4 readSoftBody(const PxgSoftBody& softbody, PxU32 tetId, const float4& bc,
												 const PxsDeformableVolumeMaterialData* const materials, bool checkOnlyActivity)
	{
		if(tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			deformableVertexInvMasses = PxVec4(0.0f);
			return deformableVertexInvMasses;
		}

		const float4* const PX_RESTRICT posDeltas = softbody.mSimDeltaPos;

		const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
		const float4 pd0 = posDeltas[tetrahedronId.x];
		const float4 pd1 = posDeltas[tetrahedronId.y];
		const float4 pd2 = posDeltas[tetrahedronId.z];
		const float4 pd3 = posDeltas[tetrahedronId.w];
		
		const float4 softBodyDelta = pd0 * bc.x + pd1 * bc.y + pd2 * bc.z + pd3 * bc.w;
		deformableLinDelta = PxVec3(softBodyDelta.x, softBodyDelta.y, softBodyDelta.z);
		deformableVertexInvMasses = PxVec4(pd0.w, pd1.w, pd2.w, pd3.w);

		if(!checkOnlyActivity)
		{
			const PxU16 globalMaterialIndex = softbody.mMaterialIndices[tetId];
			deformableFriction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

			// Query the reference count for soft body.
			PxVec4 deformableVertexReferenceCount;
			deformableVertexReferenceCount.x = softbody.mSimDelta[tetrahedronId.x].w;
			deformableVertexReferenceCount.y = softbody.mSimDelta[tetrahedronId.y].w;
			deformableVertexReferenceCount.z = softbody.mSimDelta[tetrahedronId.z].w;
			deformableVertexReferenceCount.w = softbody.mSimDelta[tetrahedronId.w].w;

			// Mass-splitting
			deformableVertexInvMasses = deformableVertexInvMasses.multiply(deformableVertexReferenceCount);
		}

		return deformableVertexInvMasses;
	}

	PX_FORCE_INLINE __device__ void writeSoftBody(const PxgSoftBody& softbody, PxU32 tetId, const float4& bc, PxVec3 deltaPos)
	{
		const float* bcPtr = reinterpret_cast<const float*>(&bc.x);
		const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
		const PxU32* tetVertices = reinterpret_cast<const PxU32*>(&tetrahedronId.x);

#pragma unroll
		for(PxU32 it = 0; it < 4; ++it)
		{
			if(deformableVertexInvMasses[it] > 0.0f)
			{
				const PxVec3 dP = deltaPos * (deformableVertexInvMasses[it] * bcPtr[it]);
				atomicAdd(&softbody.mSimDelta[tetVertices[it]].x, dP.x);
				atomicAdd(&softbody.mSimDelta[tetVertices[it]].y, dP.y);
				atomicAdd(&softbody.mSimDelta[tetVertices[it]].z, dP.z);
			}
		}
	}
};

#endif // __DEFORMABLE_UTILS_CUH__
