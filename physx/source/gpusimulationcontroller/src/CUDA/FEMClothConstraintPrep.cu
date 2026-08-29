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
#include "deformableCollision.cuh"

using namespace physx;

extern "C" __host__ void initFEMClothKernels0() {}

//! 
//! \brief    : prep cloth vs. rigid body collision
//! 

extern "C" __global__ void cloth_rigidContactPrepareLaunch(
	PxgFEMCloth*					femClothes,
	float4*							contacts_restW,
	float4*							normalPens,
	float4*							barycentrics,
	const PxgFemOtherContactInfo*	contactInfos,
	PxU32*							numContacts,
	PxgDbRigidContactBlock*			contactBlocks,
	PxgPrePrepDesc*					preDesc,
	PxgConstraintPrepareDesc*		prepareDesc,
	PxReal* 						rigidLambdaNs, 
	const PxReal					invDt,
	PxgSolverSharedDescBase*		sharedDesc,
	bool							isTGS
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

		rigidLambdaNs[workIndex] = 0.0f;

		PxgFemOtherContactInfo contactInfo = contactInfos[workIndex];
		PxgDbRigidContactBlock& block = contactBlocks[workIndex / 32];

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
			deltaP = barycentricProjectTri(vertexIndices, accumDelta_invMass, barycentric);
		}
		else
		{
			deltaP = accumDelta_invMass[elementId];
		}

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);
		const PxReal pen = normal_pen.w - contact_restW.w;

		const PxVec3 delta(deltaP.x, deltaP.y, deltaP.z);

		prepareDbRigidContact(block, normal, sharedDesc, p, pen, delta, rigidId, barycentric, prepareDesc, solverBodyIndices, cloth.mPenBiasClamp, invDt, isTGS);
	}
}


//!
//! \brief    : prep cloth vs. cloth collision.
//!

extern "C" __global__ 
void cloth_clothContactPrepareLaunch(
	PxgFEMCloth*						clothes,
	PxgFemFemContactInfo*				contactInfos,
	PxU32*								numContacts,
	PxU32								maxContacts,
	PxsDeformableSurfaceMaterialData*	clothMaterials,
	const PxU8*							updateContactPairs
)
{
	// Early exit if contact pairs are not updated.
	if(*updateContactPairs == 0)
		return;

	const PxU32 tNumContacts = PxMin(*numContacts, maxContacts);
	const PxU32 nbBlocksRequired = (tNumContacts + blockDim.x - 1) / blockDim.x;
	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;

	for(PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if(workIndex == 0) // Clamp the cloth contact count.
		{
			*numContacts = tNumContacts;
		}

		if(workIndex >= tNumContacts)
		{
			return;
		}

		PxgFemFemContactInfo contactInfo = contactInfos[workIndex];
		if(contactInfo.isValidPair()) // For different cloths, contactInfo is already set to valid.
		{
			continue;
		}

		const PxU32 pairInd0 = PxU32(contactInfo.pairInd0);
		PxgFEMCloth& cloth0 = clothes[PxGetClothId(pairInd0)];
		const PxU32 elementId0 = PxGetClothElementIndex(pairInd0);

		PxU32 pairInd1 = PxU32(contactInfo.pairInd1);
		PxgFEMCloth& cloth1 = clothes[PxGetClothId(pairInd1)];
		const PxU32 elementId1 = PxGetClothElementIndex(pairInd1);

		if(contactInfo.isEdgeEdgePair()) // Edge-edge collision
		{
			// Edge0
			PxU32 e0_localIndex0 = contactInfo.getAuxInd0();
			PxU32 e0_localIndex1 = (e0_localIndex0 + 1) % 3;

			const uint4 triVertInd0 = cloth0.mTriangleVertexIndices[elementId0];
			const PxU32* vertexIndices0 = reinterpret_cast<const PxU32*>(&triVertInd0);
			const PxU32 e0_v0 = vertexIndices0[e0_localIndex0];
			const PxU32 e0_v1 = vertexIndices0[e0_localIndex1];

			// Edge1
			PxU32 e1_localIndex0 = contactInfo.getAuxInd1();
			PxU32 e1_localIndex1 = (e1_localIndex0 + 1) % 3;

			const uint4 triVertInd1 = cloth1.mTriangleVertexIndices[elementId1];
			const PxU32* vertexIndices1 = reinterpret_cast<const PxU32*>(&triVertInd1);
			const PxU32 e1_v0 = vertexIndices1[e1_localIndex0];
			const PxU32 e1_v1 = vertexIndices1[e1_localIndex1];

			// Compute the exact rest distance for filtering.
			// Mark pairs as valid if their rest distance exceeds the filter threshold.
			const PxVec3 r0 = PxLoad3(cloth0.mRestPosition[e0_v0]);
			const PxVec3 r1 = PxLoad3(cloth0.mRestPosition[e0_v1]);
			const PxVec3 r2 = PxLoad3(cloth1.mRestPosition[e1_v0]);
			const PxVec3 r3 = PxLoad3(cloth1.mRestPosition[e1_v1]);

			// Linear blending coefficients for edge0 and edge1, respectively.
			PxReal s, t;
			PxReal restDistSq;

			// Computationally more expensive than closestPtLineLine.
			closestPtEdgeEdge(r0, r1, r2, r3, s, t, restDistSq);

			// Apply exact (non-approximated) rest distance filtering.
			if(restDistSq > cloth0.mSelfCollisionFilterDistance * cloth0.mSelfCollisionFilterDistance)
			{
				contactInfo.markValid();
				contactInfos[workIndex] = contactInfo;
			}
		}
		else // Vertex-triangle collision
		{
			const uint4 triVertId1 = cloth1.mTriangleVertexIndices[elementId1];
			PxVec4T<PxU32> vertIndices(elementId0, triVertId1.x, triVertId1.y, triVertId1.z);

			// Compute the exact rest distance for filtering.
			// Mark pairs as valid if their rest distance exceeds the filter threshold.
			const PxVec3 r0 = PxLoad3(cloth0.mRestPosition[vertIndices[0]]);
			const PxVec3 r1 = PxLoad3(cloth1.mRestPosition[vertIndices[1]]);
			const PxVec3 r2 = PxLoad3(cloth1.mRestPosition[vertIndices[2]]);
			const PxVec3 r3 = PxLoad3(cloth1.mRestPosition[vertIndices[3]]);

			const PxVec3 r12 = r2 - r1;
			const PxVec3 r13 = r3 - r1;

			// Apply exact (non-approximated) rest distance filtering.
			const PxVec3 closest = Gu::closestPtPointTriangle2(r0, r1, r2, r3, r12, r13);

			const PxReal restDistSq = (r0 - closest).magnitudeSquared();
			if(restDistSq > cloth0.mSelfCollisionFilterDistance * cloth0.mSelfCollisionFilterDistance)
			{
				contactInfo.markValid();
				contactInfos[workIndex] = contactInfo;
			}
		}
	}
}

extern "C" __global__ void cloth_particleContactPrepareLaunch(
	PxgFEMCloth*					clothes,
	PxgParticleSystem*				particlesystems,
	float4*							contacts,
	float4*							normalPens,
	float4*							barycentrics,
	PxgFemOtherContactInfo*			contactInfos,
	PxU32*							numContacts,
	PxgDbParticleContactBlock*		contactBlocks,
	float2*							clothAppliedForces,
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
		clothAppliedForces[workIndex] = make_float2(0.f, 0.f);
		particleAppliedForces[workIndex] = make_float2(0.f, 0.f);

		PxgFemOtherContactInfo contactInfo = contactInfos[workIndex];
		PxgDbParticleContactBlock& block = contactBlocks[workIndex / 32];

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

		const float4 normal_pen = normalPens[workIndex];

		float4 barycentric = barycentrics[workIndex];

		// The narrowphase reports pen at the predicted position; re-reference it to the start-of-step
		// configuration, the same frame the solver's live relLinDelta is measured from, so the two agree.
		// Cloth delta is 0 at prep in TGS, but the particle side integrates in both modes, so it's live
		// in TGS too.
		const float4 delta1 = barycentricProjectTri(triVertInd, cloth.mAccumulatedDeltaPos, barycentric);

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);

		PxgParticleSystem& particleSystem = particlesystems[tParticleSystemId];
		const float4 deltaP_invMass = particleSystem.mSortedDeltaP[tParticleIndex];

		PxVec3 delta(delta1.x - deltaP_invMass.x, delta1.y - deltaP_invMass.y, delta1.z - deltaP_invMass.z);

		const PxReal pen = normal_pen.w + normal.dot(delta) - cloth.mRestDistance;

		block.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);
		block.barycentric[threadIndexInWarp] = barycentric;
		block.maxPenBiasClamp[threadIndexInWarp] = PxMax(cloth.mPenBiasClamp, particleSystem.mData.mPenBiasClamp);
	}
}


extern "C" __global__ void cloth_rigidAttachmentPrepareLaunch(
	PxgFEMCloth*								clothes,
	PxgFEMRigidAttachment*						rigidAttachments,
	PxU32*										activeRigidAttachments,
	PxNodeIndex*								rigidAttachmentIds,
	PxU32										numRigidAttachments,
	PxgDbRigidAttachmentBlock*					attachmentBlocks,
	const PxgPrePrepDesc*						preDesc,
	const PxgConstraintPrepareDesc*				prepareDesc,
	const PxgSolverSharedDescBase*				sharedDesc,
	float4*										rigidDeltaVel
)
{


	const PxU32 nbBlocksRequired = (numRigidAttachments + blockDim.x - 1) / blockDim.x;

	const PxU32 nbIterationsPerBlock = (nbBlocksRequired + gridDim.x - 1) / gridDim.x;

	const PxU32 idx = threadIdx.x;


	for (PxU32 i = 0; i < nbIterationsPerBlock; ++i)
	{
		const PxU32 workIndex = i * blockDim.x + idx + nbIterationsPerBlock * blockIdx.x * blockDim.x;

		if (workIndex >= numRigidAttachments)
			return;

		const PxU32 index = workIndex / 32;

		const PxU32 attachmentId = activeRigidAttachments[workIndex];

		const PxgFEMRigidAttachment& attachment = rigidAttachments[attachmentId];
		PxgDbRigidAttachmentBlock& block = attachmentBlocks[index];

		const PxU32 elemId = attachment.index1;
		const PxU32 clothId = PxGetClothId(elemId);
		const PxU32 elemIdx = PxGetClothElementIndex(elemId);
		const bool elemIsVertex = PxGetIsVertexType(attachment.baryOrType1);

		PxgFEMCloth& cloth = clothes[clothId];

		const float4* pos_invMass = cloth.mPosition_InvMass;

		float4 attachmentPose;
		float invMass1;
		if (elemIsVertex)
		{
			attachmentPose = pos_invMass[elemIdx];
			invMass1 = attachmentPose.w;
		}
		else
		{
			const float4 barycentric = attachment.baryOrType1;
			const uint4 triVertInd = cloth.mTriangleVertexIndices[elemIdx];
			const float4 pos_iMass0 = pos_invMass[triVertInd.x];
			const float4 pos_iMass1 = pos_invMass[triVertInd.y];
			const float4 pos_iMass2 = pos_invMass[triVertInd.z];
			attachmentPose = pos_iMass0 * barycentric.x + pos_iMass1 * barycentric.y + pos_iMass2 * barycentric.z;
			// Squared-bary invMass for the constraint Jacobian-Mass-Jacobian denominator.
			invMass1 = barycentric.x * barycentric.x * pos_iMass0.w
			         + barycentric.y * barycentric.y * pos_iMass1.w
			         + barycentric.z * barycentric.z * pos_iMass2.w;
		}
		const PxVec3 point(attachmentPose.x, attachmentPose.y, attachmentPose.z);

		PxNodeIndex rigidId = reinterpret_cast<const PxNodeIndex&>(attachment.index0);
		rigidAttachmentIds[workIndex] = rigidId;

		prepareDbRigidAttachment(block, point, invMass1, attachment.localPose0, rigidId,
		                         elemId, attachment.baryOrType1, attachment.rigidBodyRefCount,
		                         prepareDesc, preDesc->solverBodyIndices, sharedDesc,
		                         rigidDeltaVel, workIndex, numRigidAttachments);
	}
}

