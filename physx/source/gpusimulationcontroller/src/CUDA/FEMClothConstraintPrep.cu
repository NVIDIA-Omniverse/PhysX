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
#include "stdio.h"
#include "PxgSolverCoreDesc.h"
#include "PxNodeIndex.h"
#include "PxgBodySim.h"
#include "PxgArticulation.h"
#include "PxgParticleSystem.h"
#include "PxgNpKernelIndices.h"
#include "PxgSimulationCoreDesc.h"
#include "PxsDeformableSurfaceMaterialCore.h"
#include "utils.cuh"
#include "deformableUtils.cuh"

using namespace physx;

extern "C" __host__ void initFEMClothKernels0() {}

static __device__ inline float4 computeBarycentricPos(const uint4 triangleIdx, const float4* PX_RESTRICT position_invmass,
													  const float4 barycentric)
{
	const float4 a = position_invmass[triangleIdx.x];
	const float4 b = position_invmass[triangleIdx.y];
	const float4 c = position_invmass[triangleIdx.z];

	const float4 posInvMass = a * barycentric.x + b * barycentric.y + c * barycentric.z;

	return posInvMass;
}

//! 
//! \brief    : prep cloth vs. rigid body collision
//! 

extern "C" __global__ void cloth_rigidContactPrepareLaunch(
	PxgFEMCloth*					femClothes,
	float4*							contacts_restW,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemContactInfo*				contactInfos,
	PxU32*							numContacts,
	PxgFemRigidConstraintBlock*		primitiveConstraints,
	PxgPrePrepDesc*					preDesc,
	PxgConstraintPrepareDesc*		prepareDesc,
	float4* 						rigidLambdaNs, 
	float4* 						clothLambdaNs,
	const PxReal					invDt,
	PxgSolverSharedDescBase*		sharedDesc,
	bool isTGS
)
{
	const PxU32 tNumContacts = *numContacts;

	PxU32* solverBodyIndices = preDesc->solverBodyIndices;
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;
	const PxU32 idx = threadIdx.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex >= tNumContacts)
			return;

		rigidLambdaNs[workIndex] = make_float4(0.f, 0.f, 0.f, 0.f);
		clothLambdaNs[workIndex] = make_float4(0.f, 0.f, 0.f, 0.f);

		PxgFemContactInfo contactInfo = contactInfos[workIndex];
		PxgFemRigidConstraintBlock& constraint = primitiveConstraints[workIndex / 32];

		PxU64 pairInd0 = contactInfo.pairInd0;

		// First one is rigid body
		const PxU64 tRigidId = pairInd0;
		const PxNodeIndex& rigidId = reinterpret_cast<const PxNodeIndex&>(tRigidId);

		// Second one is cloth
		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);

		PxgFEMCloth& cloth = femClothes[PxGetClothId(pairInd1)];
		const PxU32 elementId = PxGetClothElementIndex(pairInd1);

		if(elementId == 0xfffff)
			continue;

		const float4* PX_RESTRICT accumDelta_invMass = cloth.mAccumulatedDeltaPos;

		const float4 contact_restW = contacts_restW[workIndex];
		const float4 normal_pen = normalPens[workIndex];

		const PxVec3 p(contact_restW.x, contact_restW.y, contact_restW.z);

		float4 barycentric = barycentrics[workIndex];

		float4 deltaP;
		if(barycentric.w == 0.f)
		{
			const uint4 vertexIndices = cloth.mTriangleVertexIndices[elementId];
			deltaP = computeBarycentricPos(vertexIndices, accumDelta_invMass, barycentric);
		}
		else
		{
			deltaP = accumDelta_invMass[elementId];
		}

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);
		const PxReal pen = normal_pen.w - contact_restW.w;

		const PxVec3 delta(deltaP.x, deltaP.y, deltaP.z);

		prepareFEMContacts(constraint, normal, sharedDesc, p, pen, delta, rigidId, barycentric, prepareDesc, solverBodyIndices, cloth.mPenBiasClamp, invDt, isTGS);
	}
}


//!
//! \brief    : prep cloth vs. cloth collision
//!

extern "C" __global__ 
void cloth_clothContactPrepareLaunch(
	PxgFEMCloth*						clothes,
	float4*								contacts,
	float4*								normalRestDistSq,
	float4*								barycentrics0,
	float4*								barycentrics1,
	PxgFemContactInfo*					contactInfos,
	PxU32*								numContacts,
	PxU32								maxContacts,
	PxgClothConstraintBlock*			constraints,
	PxsDeformableSurfaceMaterialData*	clothMaterials,
	const PxU8*							updateContactPairs
)
{
	// Early exit if contact pairs are not updated.
	if (*updateContactPairs == 0)
		return;

	const PxU32 tNumContacts = PxMin(*numContacts, maxContacts);
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex == 0) // Clamp the cloth contact count.
			*numContacts = tNumContacts;

		if(workIndex >= tNumContacts)
			return;

		PxgFemContactInfo contactInfo = contactInfos[workIndex];
		PxgClothConstraintBlock& constraint = constraints[workIndex / 32];

		// First actor: cloth vertex (currently)
		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		PxgFEMCloth& cloth0 = clothes[PxGetClothId(pairInd0)];
		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);
		const PxReal clothFriction0 = cloth0.mDynamicFrictions[elementId0];

		// Second actor: cloth triangle
		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		PxgFEMCloth& cloth1 = clothes[PxGetClothId(pairInd1)];
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);
		const PxReal clothFriction1 = clothMaterials[cloth1.mMaterialIndices[elementId1]].dynamicFriction;
		//const float4 barycentric1 = barycentrics1[workIndex];

		const float4 normal_restDistSq = normalRestDistSq[workIndex];
		const PxReal distSqInRestPos = normal_restDistSq.w;


		// Ensure collision constraints do not interfere with the cloth's internal energy constraints during self-collision by shrinking
		// rest distances when necessary.
		const PxReal restDistance = (distSqInRestPos == PX_MAX_REAL || pairInd0 != pairInd1)
										? cloth0.mRestDistance + cloth1.mRestDistance
										: PxMin(cloth0.mRestDistance + cloth1.mRestDistance, PxSqrt(distSqInRestPos));

		// Friction: the average of the two cloth friction values is currently used.
		constraint.friction_restDist[threadIndexInWarp] = make_float2(0.5f * (clothFriction0 + clothFriction1), restDistance);
	}
}

static __device__ float4 computeTriangleContact(const float4* vels, const uint4& triVertId,
	const float4& barycentric)
{
	const float4 v0 = vels[triVertId.x];
	const float4 v1 = vels[triVertId.y];
	const float4 v2 = vels[triVertId.z];

	const float4 vel = v0 * barycentric.x + v1 * barycentric.y + v2 * barycentric.z;

	return vel;
}

extern "C" __global__ void cloth_particleContactPrepareLaunch(
	PxgFEMCloth*					clothes,
	PxgParticleSystem*				particlesystems,
	float4*							contacts,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemContactInfo*				contactInfos,
	PxU32*							numContacts,
	PxgFEMParticleConstraintBlock*	spConstraints, //soft body particle constraint
	float2*							softBodyAppliedForces,
	float2*							particleAppliedForces
)
{
	const PxU32 tNumContacts = *numContacts;

	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;
	const PxU32 threadIndexInWarp = threadIdx.x & 31;

	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= tNumContacts)
			return;

		//initialize appliedForces to be zero
		softBodyAppliedForces[workIndex] = make_float2(0.f, 0.f);
		particleAppliedForces[workIndex] = make_float2(0.f, 0.f);

		PxgFemContactInfo contactInfo = contactInfos[workIndex];
		PxgFEMParticleConstraintBlock& constraint = spConstraints[workIndex / 32];

		PxU64 pairInd0 = contactInfo.pairInd0;

		//pairInd0 is a particle system
		const PxU32 tParticleSystemId = PxGetParticleSystemId(pairInd0);
		const PxU32 tParticleIndex = PxGetParticleIndex(pairInd0);

		//second one will be cloth
		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		PxgFEMCloth& cloth = clothes[PxGetClothId(pairInd1)];
		const PxU32 triangleInd = PxGetClothElementIndex(pairInd1);

		/*printf("workIndex %i particleSystemId %i particleIndex %i\n", workIndex, tParticleSystemId, tParticleIndex);
		printf("workIndex %i softbodyId %i tetInd %i\n", workIndex, pairInd1.getSoftBodyId(), tetInd);*/

		const uint4 triVertInd = cloth.mTriangleVertexIndices[triangleInd];

		/*	printf("workIndex %i tetrahedronId(%i, %i, %i, %i)\n", workIndex, tetrahedronIdx.x, tetrahedronIdx.y,
		tetrahedronIdx.z, tetrahedronIdx.w);*/

		//get out the contact point
		const float4 contact = contacts[workIndex];
		const float4 normal_pen = normalPens[workIndex];

		/*printf("workIndex %i normal_pen(%f, %f, %f, %f)\n", workIndex, normal_pen.x, normal_pen.y,
		normal_pen.z, normal_pen.w);*/

		const PxVec3 p(contact.x, contact.y, contact.z);

		/*float4 barycentric;
		float invMass1 = computeInvMass(tetrahedronIdx, position_invmass, p, barycentric);*/
		float4 barycentric = barycentrics[workIndex];
		//float invMass1 = computeClothInvMass(triVertInd, position_invmass, barycentric);
		
		const float4 delta1 = computeTriangleContact(cloth.mAccumulatedDeltaPos, triVertInd, barycentric);
		float invMass1 = delta1.w;		

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

		PxgParticleSystem& particleSystem = particlesystems[tParticleSystemId];
		//const float4 position_invMass = particleSystem.mSortedPosition_InvMass[tParticleIndex];
		//const PxReal invMass0 = position_invMass.w;

		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[tParticleIndex];

		const PxReal invMass0 = deltaP_invMass.w;

		PxVec3 delta(delta1.x - deltaP_invMass.x, delta1.y - deltaP_invMass.y, delta1.z - deltaP_invMass.z);

		const PxReal pen = normal_pen.w + normal.dot(delta) - cloth.mRestDistance;

		//printf("pen = %f, normal_pen.w = %f, normal.dot(delta) = %f, delta1 = (%f, %f, %f), deltaP = (%f, %f, %f)\n", 
		//	pen, normal_pen.w, normal.dot(delta), delta1.x, delta1.y, delta1.z, deltaP_invMass.x, deltaP_invMass.y, deltaP_invMass.z);


		const float unitResponse = invMass0 + invMass1;
		//KS - perhaps we don't need the > 0.f check here?
		const float velMultiplier = (unitResponse > 0.f) ? (1.f / unitResponse) : 0.f;

		//PxReal biasedErr = PxMin(-0.5f * pen * invDt, 5.f)*velMultiplier;
		//PxReal biasedErr = (-0.5f * pen * invDt)*velMultiplier;
		//printf("biasedErr %f, pen %f, invDt %f, velMultiplier %f\n", biasedErr, pen, invDt, velMultiplier);
		constraint.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);
		constraint.barycentric[threadIndexInWarp] = barycentric;
		constraint.velMultiplier[threadIndexInWarp] = velMultiplier;
	}
}


extern "C" __global__ void cloth_rigidAttachmentPrepareLaunch(
	PxgFEMCloth*								clothes,
	PxgFEMRigidAttachment*						rigidAttachments,
	PxU32*										activeRigidAttachments,
	PxNodeIndex*								rigidAttachmentIds,
	PxU32										numRigidAttachments,
	PxgFEMRigidAttachmentConstraint*			attachmentConstraints,
	const PxgPrePrepDesc*						preDesc,
	const PxgConstraintPrepareDesc*				prepareDesc,
	const PxgSolverSharedDescBase*				sharedDesc,
	float4*										rigidDeltaVel
)
{

	const PxAlignedTransform* bodyFrames = prepareDesc->body2WorldPool;

	const PxU32* solverBodyIndices = preDesc->solverBodyIndices;
	const PxgSolverBodyData* solverBodyData = prepareDesc->solverBodyDataPool;
	const PxgSolverTxIData* solverDataTxIPool = prepareDesc->solverBodyTxIDataPool;


	const PxgBodySim* bodySims = sharedDesc->mBodySimBufferDeviceData;

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

		const PxU32 attachmentId = activeRigidAttachments[workIndex];

		const PxgFEMRigidAttachment& attachment = rigidAttachments[attachmentId];
		PxgFEMRigidAttachmentConstraint& constraint = attachmentConstraints[index];

		const PxU32 elemId = attachment.index1;
		const PxU32 clothId = PxGetClothId(elemId);
		const PxU32 elemIdx = PxGetClothElementIndex(elemId);
		const bool elemIsVertex = PxGetIsVertexType(attachment.baryOrType1);

		PxgFEMCloth& cloth = clothes[clothId];

		const float4* pos_invMass = cloth.mPosition_InvMass;
		const float4 low_high_limits = attachment.coneLimitParams.low_high_limits;
		const float4 axis_angle = attachment.coneLimitParams.axis_angle;

		float4 attachmentPose;
		if (elemIsVertex)
		{
			attachmentPose = pos_invMass[elemIdx];
		}
		else
		{
			const float4 barycentric = attachment.baryOrType1;
			const uint4 triVertInd = cloth.mTriangleVertexIndices[elemIdx];
			const float4 pos_iMass0 = pos_invMass[triVertInd.x];
			const float4 pos_iMass1 = pos_invMass[triVertInd.y];
			const float4 pos_iMass2 = pos_invMass[triVertInd.z];
			attachmentPose = pos_iMass0 * barycentric.x + pos_iMass1 * barycentric.y + pos_iMass2 * barycentric.z;
		}

		float invMass1 = attachmentPose.w;
		const PxVec3 point(attachmentPose.x, attachmentPose.y, attachmentPose.z);
		const PxVec3 axis(axis_angle.x, axis_angle.y, axis_angle.z);

		float4 ra4 = attachment.localPose0;

		//nodeIndex
		PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(attachment.index0);
		PxU32 idx = 0;
		if (!rigidId.isStaticBody())
		{
			idx = solverBodyIndices[rigidId.index()];
		}

		rigidAttachmentIds[workIndex] = rigidId;

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

			const PxVec3 worldAxis = (body2World.rotate(axis)).getNormalized();

			PxVec3 ra(ra4.x, ra4.y, ra4.z);
			ra = body2World.rotate(ra);
			PxVec3 error = ra + bodyFrame0p - point;

			const PxVec3 raXn0 = ra.cross(normal0);
			const PxVec3 raXn1 = ra.cross(normal1);
			const PxVec3 raXn2 = ra.cross(normal2);

			PxSpatialMatrix& spatialResponse = articulation.spatialResponseMatrixW[linkID];
			const Cm::UnAlignedSpatialVector deltaV0 = spatialResponse * Cm::UnAlignedSpatialVector(normal0, raXn0);
			const Cm::UnAlignedSpatialVector deltaV1 = spatialResponse * Cm::UnAlignedSpatialVector(normal1, raXn1);
			const Cm::UnAlignedSpatialVector deltaV2 = spatialResponse * Cm::UnAlignedSpatialVector(normal2, raXn2);

			const PxReal resp0 = deltaV0.top.dot(raXn0) + deltaV0.bottom.dot(normal0) + invMass1;
			const PxReal resp1 = deltaV0.top.dot(raXn1) + deltaV0.bottom.dot(normal1) + invMass1;
			const PxReal resp2 = deltaV0.top.dot(raXn2) + deltaV0.bottom.dot(normal2) + invMass1;

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

