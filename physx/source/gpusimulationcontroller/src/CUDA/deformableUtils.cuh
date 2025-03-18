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

	// fmaxf because biasClamps are negative
	PxReal maxPenBias = fmaxf(penBiasClampRigid, penBiasClampFEM);
	PxReal error;
	if(isTGS)
	{
		error = pen + delta.dot(normal);
	}
	else
	{
		PxReal scaledBias = pen >= 0.0f ? pen : fmaxf(maxPenBias / invDt, pen);
		error = scaledBias + delta.dot(normal);
	}

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
	// Rigid body
	PxU32 globalRigidBodyId;
	PxReal rigidBodyReferenceCount = -1.0f;
	PxReal rigidBodyFriction;
	PxI32 frictionCombineMode;

	// Deformable body
	PxReal deformableFriction;
	PxVec3 deformableLinDelta;
	Vec deformableVertexInvMasses = Vec(-1.0f); // With mass-splitting.

	// Purely virtual functions
	PX_FORCE_INLINE __device__ bool initialize(const float4& fricTan0_invMass0, PxgFemRigidConstraintBlock& constraint,
											   float4& appliedForceRef, PxNodeIndex rigidId, PxgVelocityReader& velocityReader, PxReal dt,
											   const Vec& bc, bool countReferenceOnly);
	PX_FORCE_INLINE __device__ float4 computeRigidChange(PxVec3& deltaLinVel0, PxVec3& deltaAngVel0, PxReal rigidInvMass,
														 const PxNodeIndex& rigidId);
	PX_FORCE_INLINE __device__ float4 computeFEMChange(PxVec3& deltaPos, PxReal dt);

	PX_FORCE_INLINE __device__ PxReal getCombinedFriction()
	{
		return PxsCombinePxReal(rigidBodyFriction, deformableFriction, frictionCombineMode);
	}

	// Shared functions
	PX_FORCE_INLINE __device__ void readRigidBody(const PxgPrePrepDesc* const prePrepDesc, const PxNodeIndex& rigidId, PxReal rigidInvMass,
												  PxU32 numSolverBodies, const PxU32* const rigidBodyReferenceCounts,
												  const PxsMaterialData* rigidMaterial)
	{
		// Following PxgVelocityReader style to read rigid body indices.
		const PxU32 solverBodyIdx = rigidId.isStaticBody() ? 0 : prePrepDesc->solverBodyIndices[rigidId.index()];

		globalRigidBodyId = 0;
		rigidBodyReferenceCount = 1.0f;

		if(solverBodyIdx)
		{
			// Placing articulation indices at the end of rigid body indices to distinguish between rigid body reference counts and
			// articulation reference counts.
			globalRigidBodyId = rigidId.isArticulation() ? numSolverBodies + solverBodyIdx : solverBodyIdx;
		}

		// Query the reference count for the rigid body.
		if(rigidBodyReferenceCounts && !rigidId.isStaticBody() && rigidInvMass != 0.f)
		{
			rigidBodyReferenceCount = static_cast<PxReal>(rigidBodyReferenceCounts[globalRigidBodyId]);
		}

		rigidBodyReferenceCount = PxMax(rigidBodyReferenceCount, 1.0f);

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
												   bool isActive, PxU32 workIndex0, PxU32 workIndex1)
	{
		const PxU32 count = isActive ? 1.0f : 0.0f;
		rigidDeltaVel[workIndex0] = make_float4(deltaLinVel0.x, deltaLinVel0.y, deltaLinVel0.z, count);
		rigidDeltaVel[workIndex1] = make_float4(deltaAngVel0.x, deltaAngVel0.y, deltaAngVel0.z, 0.f);
	}

	PX_FORCE_INLINE __device__ void readCloth(const PxgFEMCloth& cloth, PxU32 elementId, const float4& bc,
											  const PxsDeformableSurfaceMaterialData* const materials, bool countReferenceOnly)
	{
		// Note: PX_MAX_NB_DEFORMABLE_SURFACE_TRI == PX_MAX_NB_DEFORMABLE_SURFACE_VTX
		if(elementId == PX_MAX_NB_DEFORMABLE_SURFACE_TRI)
		{
			deformableVertexInvMasses = PxVec3(0.0f);
			return;
		}

		float4* clothPosDeltas = cloth.mAccumulatedDeltaPos;

		float4 clothDelta;
		PxVec3 deformableVertexReferenceCount(1.0f);

		if(bc.w == 0) // Cloth triangle
		{
			const PxU16 globalMaterialIndex = cloth.mMaterialIndices[elementId];
			deformableFriction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

			const uint4 triVertId = cloth.mTriangleVertexIndices[elementId];
			const float4 pd0 = clothPosDeltas[triVertId.x];
			const float4 pd1 = clothPosDeltas[triVertId.y];
			const float4 pd2 = clothPosDeltas[triVertId.z];
			clothDelta = pd0 * bc.x + pd1 * bc.y + pd2 * bc.z;

			deformableVertexInvMasses = PxVec3(pd0.w, pd1.w, pd2.w);

			if(!countReferenceOnly)
			{
				// Query the reference count for the cloth.
				deformableVertexReferenceCount.x = cloth.mDeltaPos[triVertId.x].w;
				deformableVertexReferenceCount.y = cloth.mDeltaPos[triVertId.y].w;
				deformableVertexReferenceCount.z = cloth.mDeltaPos[triVertId.z].w;

				// Just in case the reference count is zero.
				deformableVertexReferenceCount.x = PxMax(deformableVertexReferenceCount.x, 1.0f);
				deformableVertexReferenceCount.y = PxMax(deformableVertexReferenceCount.y, 1.0f);
				deformableVertexReferenceCount.z = PxMax(deformableVertexReferenceCount.z, 1.0f);

				// Mass-splitting
				deformableVertexInvMasses = deformableVertexInvMasses.multiply(deformableVertexReferenceCount);
			}
		}
		else // Cloth vertex
		{
			deformableFriction = materials ? cloth.mDynamicFrictions[elementId] : 0.0f;
			clothDelta = clothPosDeltas[elementId];
			deformableVertexInvMasses = PxVec3(clothDelta.w, 0.0f, 0.0f);

			if(!countReferenceOnly)
			{
				// Query the reference count for the cloth.
				deformableVertexReferenceCount.x = cloth.mDeltaPos[elementId].w;

				// Just in case the reference count is zero.
				deformableVertexReferenceCount.x = PxMax(deformableVertexReferenceCount.x, 1.0f);

				// Mass-splitting
				deformableVertexInvMasses.x *= deformableVertexReferenceCount.x;
			}
		}

		deformableLinDelta = PxVec3(clothDelta.x, clothDelta.y, clothDelta.z);
	}

	PX_FORCE_INLINE __device__ void writeCloth(PxgFEMCloth& cloth, PxU32 elementId, const float4& bc, PxVec3 deltaPos)
	{
		if(bc.w == 0.f) // Cloth triangle
		{
			const uint4 triVertInds = cloth.mTriangleVertexIndices[elementId];
			const float* bcPtr = reinterpret_cast<const float*>(&bc.x);
			const PxU32* triVertices = reinterpret_cast<const PxU32*>(&triVertInds.x);

#pragma unroll
			for(PxU32 it = 0; it < 3; ++it)
			{
				if(deformableVertexInvMasses[it] > 0.0f && PxAbs(bcPtr[it]) > 1e-6f)
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
			const PxVec3 dP = deltaPos * deformableVertexInvMasses.x;

			atomicAdd(&cloth.mDeltaPos[elementId].x, dP.x);
			atomicAdd(&cloth.mDeltaPos[elementId].y, dP.y);
			atomicAdd(&cloth.mDeltaPos[elementId].z, dP.z);
		}
	}

	PX_FORCE_INLINE __device__ void readSoftBody(const PxgSoftBody& softbody, PxU32 tetId, const float4& bc,
												 const PxsDeformableVolumeMaterialData* const materials, bool countReferenceOnly)
	{
		if(tetId == PX_MAX_NB_DEFORMABLE_VOLUME_TET)
		{
			deformableVertexInvMasses = PxVec4(0.0f);
			return;
		}

		float4* posDeltas = softbody.mSimDeltaPos;

		const PxU16 globalMaterialIndex = softbody.mMaterialIndices[tetId];
		deformableFriction = materials ? materials[globalMaterialIndex].dynamicFriction : 0.0f;

		const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
		const float4 pd0 = posDeltas[tetrahedronId.x];
		const float4 pd1 = posDeltas[tetrahedronId.y];
		const float4 pd2 = posDeltas[tetrahedronId.z];
		const float4 pd3 = posDeltas[tetrahedronId.w];

		float4 softBodyDelta = pd0 * bc.x + pd1 * bc.y + pd2 * bc.z + pd3 * bc.w;
		deformableLinDelta = PxVec3(softBodyDelta.x, softBodyDelta.y, softBodyDelta.z);

		const PxVec4 invMasses(pd0.w, pd1.w, pd2.w, pd3.w);

		deformableVertexInvMasses = PxVec4(pd0.w, pd1.w, pd2.w, pd3.w);

		if(!countReferenceOnly)
		{
			PxVec4 deformableVertexReferenceCount;

			// Query the reference count for soft body.
			deformableVertexReferenceCount.x = PxMax(softbody.mSimDelta[tetrahedronId.x].w, 1.0f);
			deformableVertexReferenceCount.y = PxMax(softbody.mSimDelta[tetrahedronId.y].w, 1.0f);
			deformableVertexReferenceCount.z = PxMax(softbody.mSimDelta[tetrahedronId.z].w, 1.0f);
			deformableVertexReferenceCount.w = PxMax(softbody.mSimDelta[tetrahedronId.w].w, 1.0f);

			// Mass-splitting
			deformableVertexInvMasses = deformableVertexInvMasses.multiply(deformableVertexReferenceCount);
		}
	}

	PX_FORCE_INLINE __device__ void writeSoftBody(const PxgSoftBody& softbody, PxU32 tetId, const float4& bc, PxVec3 deltaPos)
	{
		const float* bcPtr = reinterpret_cast<const float*>(&bc.x);
		const uint4 tetrahedronId = softbody.mSimTetIndices[tetId];
		const PxU32* tetVertices = reinterpret_cast<const PxU32*>(&tetrahedronId.x);

#pragma unroll
		for(PxU32 it = 0; it < 4; ++it)
		{
			if(deformableVertexInvMasses[it] > 0.0f && PxAbs(bcPtr[it]) > 1e-6f)
			{
				const PxVec3 dP = deltaPos * (deformableVertexInvMasses[it] * bcPtr[it]);
				atomicAdd(&softbody.mSimDelta[tetVertices[it]].x, dP.x);
				atomicAdd(&softbody.mSimDelta[tetVertices[it]].y, dP.y);
				atomicAdd(&softbody.mSimDelta[tetVertices[it]].z, dP.z);
			}
		}
	}
};

template <typename Vec>
struct FEMCollisionTGS : public FEMCollision<Vec>
{
	PxReal deltaLambdaT;
	PxReal deltaLambdaN;
	PxVec3 tanDir;
	float4 appliedForce;
	PxVec3 normal;
	PxVec3 raXn, raXt;

	// Returns whether this collision pair generates collision impulses.
	PX_FORCE_INLINE __device__ bool initialize(const float4& fricTan0_invMass0, PxgFemRigidConstraintBlock& constraint,
											   float4& appliedForceRef, PxNodeIndex rigidId, PxgVelocityReader& velocityReader, PxReal dt,
											   const Vec& bc, bool countReferenceOnly)
	{
		// Read rigid body and cloth/softbody before calling initialize.
		assert(FEMCollision<Vec>::rigidBodyReferenceCount >= 0 && FEMCollision<Vec>::deformableVertexInvMasses.x >= 0);

		// PxgFemRigidConstraintBlock is packed with arrays with size 32.
		assert(blockDim.x % 32 == 0 && blockDim.y == 1 && blockDim.z == 1);

		// PBD way of appying constraints
		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		appliedForce = appliedForceRef;

		PxgVelocityPackTGS rigidStateVec;
		velocityReader.readVelocitiesTGS(rigidId, rigidStateVec);

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

		const float normalVel = rigidStateVec.linVel.dot(normal) + rigidStateVec.angVel.dot(raXn);

		const PxVec3 relLinDelta = rigidStateVec.linDelta - FEMCollision<Vec>::deformableLinDelta;

		const PxReal dtInv = 1.0f / dt;

		// Collision constraint in normal direction
		const PxReal errorDtInv = (initPen + relLinDelta.dot(normal) + rigidStateVec.angDelta.dot(raXn)) * dtInv;

		// fmaxf because maxPenBias is negative
		const PxReal errorBiased = errorDtInv < 0.0f ? fmaxf(maxPenBias, errorDtInv) : errorDtInv;
		const PxReal CN = errorBiased + normalVel;

		const PxReal threshold = 1.0e-14f;
		const bool isActive = PxAbs(CN) > threshold;

		if(countReferenceOnly)
		{
			// Exit early by checking if there is any impulse.
			return isActive;
		}

		if(!isActive)
		{
			deltaLambdaT = 0.0f;
			raXt = PxVec3(0.0f);
			tanDir = PxVec3(0.0f);
			return false;
		}

		// Deformable body term in the denominator of the impulse calculation. Also, refer to delta lambda in the XPBD paper.
		const PxReal deformableInvMass_massSplitting = bc.multiply(bc).dot(FEMCollision<Vec>::deformableVertexInvMasses);

		const PxReal unitResponse = FEMCollision<Vec>::rigidBodyReferenceCount * raXn_resp.w + deformableInvMass_massSplitting;

		const PxReal denomInv = (unitResponse > 0.0f) ? (1.0f / unitResponse) : 0.0f;

		deltaLambdaN = PxMax(-CN * denomInv, -appliedForce.x);

		appliedForce.x += deltaLambdaN;

		// Friction constraint in the tangent direction.
		const PxVec3 raXnF0 = PxVec3(raXnF0_resp.x, raXnF0_resp.y, raXnF0_resp.z);
		const PxVec3 raXnF1 = PxVec3(raXnF1_resp.x, raXnF1_resp.y, raXnF1_resp.z);

		const float tanVel0 = rigidStateVec.linVel.dot(fric0) + rigidStateVec.angVel.dot(raXnF0);
		const float tanVel1 = rigidStateVec.linVel.dot(fric1) + rigidStateVec.angVel.dot(raXnF1);

		const PxReal CT0 = (fric0.dot(relLinDelta) + rigidStateVec.angDelta.dot(raXnF0)) * dtInv + tanVel0;
		const PxReal CT1 = (fric1.dot(relLinDelta) + rigidStateVec.angDelta.dot(raXnF1)) * dtInv + tanVel1;
		const PxVec3 relTanDelta = CT0 * fric0 + CT1 * fric1;
		const PxReal tanMagSq = relTanDelta.magnitudeSquared();

		if(tanMagSq > threshold)
		{
			const PxReal CT = PxSqrt(tanMagSq);
			const PxReal tanMagInv = 1.f / CT;
			tanDir = relTanDelta * tanMagInv;

			const PxReal frac0 = tanDir.dot(fric0);
			const PxReal frac1 = tanDir.dot(fric1);
			raXt = frac0 * raXnF0 + frac1 * raXnF1;

			// Using two precomputed orthonormal tangent directions.
			const PxReal unitResponseT0 = FEMCollision<Vec>::rigidBodyReferenceCount * raXnF0_resp.w + deformableInvMass_massSplitting;
			const PxReal tanDenomInv0 = (unitResponseT0 > 0.0f) ? (1.0f / unitResponseT0) : 0.0f;

			const PxReal unitResponseT1 = FEMCollision<Vec>::rigidBodyReferenceCount * raXnF1_resp.w + deformableInvMass_massSplitting;
			const PxReal tanDenomInv1 = (unitResponseT1 > 0.0f) ? (1.0f / unitResponseT1) : 0.0f;

			PxReal deltaLambdaT0 = CT0 * tanDenomInv0;
			PxReal deltaLambdaT1 = CT1 * tanDenomInv1;

			deltaLambdaT = PxSqrt(deltaLambdaT0 * deltaLambdaT0 + deltaLambdaT1 * deltaLambdaT1);
			deltaLambdaT = -PxMin(deltaLambdaT, FEMCollision<Vec>::getCombinedFriction() * PxAbs(deltaLambdaN));

			assert(deltaLambdaT <= 0.f);
		}
		else
		{
			deltaLambdaT = 0.0f;
			raXt = PxVec3(0.0f);
			tanDir = PxVec3(0.0f);
		}

		return isActive;
	}

	PX_FORCE_INLINE __device__ float4 computeRigidChange(PxVec3& deltaLinVel0, PxVec3& deltaAngVel0, PxReal rigidInvMass,
														 const PxNodeIndex& rigidId)
	{
		if(deltaLambdaN != 0.0f && rigidInvMass != 0.0f)
		{
			// Due to momocity, raXn and raXt are multiplied by sqrt(referenceCount) for rigid bodies.
			deltaAngVel0 = rigidId.isArticulation()
							   ? raXn * deltaLambdaN + raXt * deltaLambdaT
							   : (raXn * deltaLambdaN + raXt * deltaLambdaT) * PxSqrt(FEMCollision<Vec>::rigidBodyReferenceCount);

			deltaLinVel0 = (normal * deltaLambdaN + tanDir * deltaLambdaT) * rigidInvMass * FEMCollision<Vec>::rigidBodyReferenceCount;
		}
		else
		{
			deltaAngVel0 = PxVec3(0.0f);
			deltaLinVel0 = PxVec3(0.0f);
		}

		return appliedForce;
	}

	PX_FORCE_INLINE __device__ float4 computeFEMChange(PxVec3& deltaPos, PxReal dt)
	{
		if(deltaLambdaN != 0.f)
			deltaPos = -(deltaLambdaN * normal + deltaLambdaT * tanDir) * dt;
		else
			deltaPos = PxVec3(0.0f);

		return appliedForce;
	}
};

template <typename Vec>
struct FEMCollisionPGS : public FEMCollision<Vec>
{
	float4 appliedForce;
	PxReal requiredF0Force;
	PxReal requiredF1Force;
	PxReal totalForce0;
	PxReal totalForce1;
	PxReal ratio;
	PxReal deltaF;
	PxVec3 fric0, fric1;
	PxVec3 normal;
	PxVec3 raXn, raXnF0, raXnF1;

	// Note that friction is differently formulated from TGS, currently. See FEMCollisionTGS.
	PX_FORCE_INLINE __device__ bool initialize(const float4& fricTan0_invMass0, PxgFemRigidConstraintBlock& constraint,
											   float4& appliedForceRef, PxNodeIndex rigidId, PxgVelocityReader& velocityReader, PxReal dt,
											   const Vec& bc, bool countReferenceOnly)
	{
		// Read rigid body and cloth/softbody before calling initialize.
		assert(FEMCollision<Vec>::rigidBodyReferenceCount >= 0 && FEMCollision<Vec>::deformableVertexInvMasses.x >= 0);

		const PxU32 threadIndexInWarp = threadIdx.x & 31;

		appliedForce = appliedForceRef;

		PxgVelocityPackPGS vel0;
		velocityReader.readVelocitiesPGS(rigidId, vel0);

		const float4 raXn_resp = constraint.raXn_resp[threadIndexInWarp];
		const float4 normal_errorW = constraint.normal_errorW[threadIndexInWarp];

		const float4 raXnF0_resp = constraint.raXnF0_resp[threadIndexInWarp];
		const float4 raXnF1_resp = constraint.raXnF1_resp[threadIndexInWarp];

		normal = PxVec3(normal_errorW.x, normal_errorW.y, normal_errorW.z);

		fric0 = PxVec3(fricTan0_invMass0.x, fricTan0_invMass0.y, fricTan0_invMass0.z);
		fric1 = normal.cross(fric0);

		const float error = normal_errorW.w - FEMCollision<Vec>::deformableLinDelta.dot(normal);
		raXn = PxVec3(raXn_resp.x, raXn_resp.y, raXn_resp.z);

		const PxReal tanDelta0 = -FEMCollision<Vec>::deformableLinDelta.dot(fric0);
		const PxReal tanDelta1 = -FEMCollision<Vec>::deformableLinDelta.dot(fric1);

		raXnF0 = PxVec3(raXnF0_resp.x, raXnF0_resp.y, raXnF0_resp.z);

		raXnF1 = PxVec3(raXnF1_resp.x, raXnF1_resp.y, raXnF1_resp.z);

		// Compute the normal velocity of the constraint.
		const float normalVel = vel0.linVel.dot(normal) + vel0.angVel.dot(raXn);
		const float tanVel0 = vel0.linVel.dot(fric0) + vel0.angVel.dot(raXnF0);
		const float tanVel1 = vel0.linVel.dot(fric1) + vel0.angVel.dot(raXnF1);

		const PxReal dtInv = 1.0f / dt;

		const PxReal deformableInvMass_massSplitting = bc.multiply(bc).dot(FEMCollision<Vec>::deformableVertexInvMasses);

		const PxReal unitResponse = FEMCollision<Vec>::rigidBodyReferenceCount * raXn_resp.w + deformableInvMass_massSplitting;
		const PxReal denomInv = (unitResponse > 0.0f) ? (1.0f / unitResponse) : 0.0f;

		// KS - clamp the maximum force
		// Normal force can only be +ve!
		deltaF = PxMax(-appliedForce.x, (-error * dtInv - normalVel) * denomInv);

		const PxReal threshold = 1.0e-14f;
		const bool isActive = PxAbs(deltaF) > threshold;
		if(countReferenceOnly)
		{
			// Exit early by checking if there is any impulse.
			return isActive;
		}

		if(!isActive)
		{
			requiredF0Force = 0.0f;
			requiredF1Force = 0.0f;
			ratio = 0.0f;
			return false;
		}

		appliedForce.x += deltaF;

		// Track maximum normal force
		appliedForce.y = PxMax(appliedForce.x, appliedForce.y);

		// const PxReal friction = (takeMaxForFriction ? appliedForce.y : appliedForce.x) * frictionCoefficient;
		const PxReal friction = appliedForce.y * FEMCollision<Vec>::getCombinedFriction();

		const PxReal unitResponseT0 = FEMCollision<Vec>::rigidBodyReferenceCount * raXnF0_resp.w + deformableInvMass_massSplitting;
		const PxReal tanDenomInv0 = (unitResponseT0 > 0.0f) ? (1.0f / unitResponseT0) : 0.0f;

		const PxReal unitResponseT1 = FEMCollision<Vec>::rigidBodyReferenceCount * raXnF1_resp.w + deformableInvMass_massSplitting;
		const PxReal tanDenomInv1 = (unitResponseT1 > 0.0f) ? (1.0f / unitResponseT1) : 0.0f;

		requiredF0Force = (-tanDelta0 * dtInv - tanVel0) * tanDenomInv0;
		requiredF1Force = (-tanDelta1 * dtInv - tanVel1) * tanDenomInv1;

		totalForce0 = requiredF0Force + appliedForce.z;
		totalForce1 = requiredF1Force + appliedForce.w;

		const PxReal totalForce = PxSqrt(totalForce0 * totalForce0 + totalForce1 * totalForce1);
		ratio = PxMin(1.f, totalForce >= 1e-8f ? friction / totalForce : 0.f);

		return isActive;
	}

	PX_FORCE_INLINE __device__ float4 computeRigidChange(PxVec3& deltaLinVel0, PxVec3& deltaAngVel0, PxReal rigidInvMass,
														 const PxNodeIndex& rigidId)
	{
		// RequiredForce is always positive!
		PxReal deltaFr0 = (requiredF0Force * ratio) - appliedForce.z;
		PxReal deltaFr1 = (requiredF1Force * ratio) - appliedForce.w;
		appliedForce.z += deltaFr0;
		appliedForce.w += deltaFr1;

		if((deltaF != 0.f || deltaFr0 != 0.f || deltaFr1 != 0.f) && rigidInvMass > 0.0f) // if rigid is kinematic or static, deltaVel must
																						 // remain zero
		{
			// Due to momocity, raXn and raXt are multiplied by sqrt(referenceCount) for rigid bodies.
			deltaAngVel0 = rigidId.isArticulation() ? raXn * deltaF + raXnF0 * deltaFr0 + raXnF1 * deltaFr1
													: (raXn * deltaF + raXnF0 * deltaFr0 + raXnF1 * deltaFr1) *
														  PxSqrt(FEMCollision<Vec>::rigidBodyReferenceCount);

			deltaLinVel0 =
				((normal * deltaF) + fric0 * deltaFr0 + fric1 * deltaFr1) * rigidInvMass * FEMCollision<Vec>::rigidBodyReferenceCount;
		}
		else
		{
			deltaAngVel0 = PxVec3(0.0f);
			deltaLinVel0 = PxVec3(0.0f);
		}

		return appliedForce;
	}

	PX_FORCE_INLINE __device__ float4 computeFEMChange(PxVec3& deltaPos, PxReal dt)
	{
		PxReal deltaFr0 = (totalForce0 * ratio) - appliedForce.z;
		PxReal deltaFr1 = (totalForce1 * ratio) - appliedForce.w;

		if(deltaF != 0.f || deltaFr0 != 0.f || deltaFr1 != 0.f)
		{
			const PxVec3 deltaNorVel = -(normal * deltaF);
			const PxVec3 deltaFricVel = -(fric0 * deltaFr0 + fric1 * deltaFr1);

			deltaPos = (deltaNorVel + deltaFricVel) * dt;
		}
		else
			deltaPos = PxVec3(0.0f);

		appliedForce.z += deltaFr0;
		appliedForce.w += deltaFr1;

		return appliedForce;
	}
};

#endif // __DEFORMABLE_UTILS_CUH__
