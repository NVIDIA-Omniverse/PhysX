// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	const PxgDeformablePairInfo*	contactInfos,
	PxU32*							numContacts,
	PxgRigidDbContactBlock*			contactBlocks,
	PxgPrePrepDesc*					preDesc,
	PxgConstraintPrepareDesc*		prepareDesc,
	float2*							rigidLambdaNs,
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

		rigidLambdaNs[workIndex] = make_float2(0.f, 0.f);

		PxgDeformablePairInfo contactInfo = contactInfos[workIndex];
		PxgRigidDbContactBlock& block = contactBlocks[workIndex / 32];

		// First one is rigid body
		const PxNodeIndex rigidId = PxgGetRigidNode(contactInfo.ref0);

		if(!PxgIsValidPairInfo(contactInfo))
			continue;

		// Second one is cloth.
		PxgFEMCloth& cloth = femClothes[PxgGetDeformableId(contactInfo.ref1)];
		const PxU32 elementId = PxgGetDeformableFeatureId(contactInfo.ref1);
		const bool clothIsVertex = PxgGetDeformableFeatureType(contactInfo.ref1) == PxgDeformableRef::eVERTEX;

		const float4* PX_RESTRICT accumDelta_invMass = cloth.mAccumulatedDeltaPos;

		const float4 contact_restW = contacts_restW[workIndex];
		const float4 normal_pen = normalPens[workIndex];

		const PxVec3 p(contact_restW.x, contact_restW.y, contact_restW.z);

		float4 deltaP;
		if(!clothIsVertex)
		{
			const float4 barycentric = barycentrics[workIndex];
			const uint4 vertexIndices = cloth.mTriangleVertexIndices_ownedEdges[elementId];
			deltaP = barycentricProjectTri(vertexIndices, accumDelta_invMass, barycentric);
		}
		else
		{
			deltaP = accumDelta_invMass[elementId];
		}

		const PxVec3 normal(-normal_pen.x, -normal_pen.y, -normal_pen.z);
		const PxReal pen = normal_pen.w - contact_restW.w;

		const PxVec3 delta(deltaP.x, deltaP.y, deltaP.z);

		prepareRigidDbContact(block, normal, sharedDesc, p, pen, delta, rigidId,
							  prepareDesc, solverBodyIndices, cloth.mPenBiasClamp, invDt, isTGS);
	}
}


//!
//! \brief    : prep cloth vs. cloth collision.
//!

extern "C" __global__ 
void cloth_clothContactPrepareLaunch(
	PxgFEMCloth*						clothes,
	PxgDeformablePairInfo*				contactInfos,
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

		PxgDeformablePairInfo contactInfo = contactInfos[workIndex];

		const PxU32 clothId0 = PxgGetDeformableId(contactInfo.ref0);
		const PxU32 clothId1 = PxgGetDeformableId(contactInfo.ref1);

		// Only self-collision needs the exact test. A contact between two cloths stands as generated,
		// the midphase having applied the filter distance that self-collision only approximates.
		if(clothId0 != clothId1)
		{
			continue;
		}

		PxgFEMCloth& cloth0 = clothes[clothId0];
		PxgFEMCloth& cloth1 = clothes[clothId1];

		// prepClothContactBlocks runs this kernel once for the vertex-triangle stream and once for the
		// edge-edge stream. The ref0 feature type is sufficient to identify which of the two it is.
		if(PxgGetDeformableFeatureType(contactInfo.ref0) == PxgDeformableRef::eTRIANGLE_EDGE)
		{
			// Edge0
			PxU32 e0_localIndex0 = PxgGetDeformableEdgeLocalIndex(contactInfo.ref0);
			PxU32 e0_localIndex1 = (e0_localIndex0 + 1) % 3;

			const uint4 triVertInd0 = cloth0.mTriangleVertexIndices_ownedEdges[PxgGetDeformableEdgeTriangle(contactInfo.ref0)];
			const PxU32* vertexIndices0 = reinterpret_cast<const PxU32*>(&triVertInd0);
			const PxU32 e0_v0 = vertexIndices0[e0_localIndex0];
			const PxU32 e0_v1 = vertexIndices0[e0_localIndex1];

			// Edge1
			PxU32 e1_localIndex0 = PxgGetDeformableEdgeLocalIndex(contactInfo.ref1);
			PxU32 e1_localIndex1 = (e1_localIndex0 + 1) % 3;

			const uint4 triVertInd1 = cloth1.mTriangleVertexIndices_ownedEdges[PxgGetDeformableEdgeTriangle(contactInfo.ref1)];
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
			if(restDistSq <= cloth0.mSelfCollisionFilterDistance * cloth0.mSelfCollisionFilterDistance)
			{
				PxgInvalidatePairInfo(contactInfos[workIndex]);
			}
		}
		else // Vertex-triangle collision
		{
			const uint4 triVertId1 = cloth1.mTriangleVertexIndices_ownedEdges[PxgGetDeformableFeatureId(contactInfo.ref1)];
			PxVec4T<PxU32> vertIndices(PxgGetDeformableFeatureId(contactInfo.ref0), triVertId1.x, triVertId1.y, triVertId1.z);

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
			if(restDistSq <= cloth0.mSelfCollisionFilterDistance * cloth0.mSelfCollisionFilterDistance)
			{
				PxgInvalidatePairInfo(contactInfos[workIndex]);
			}
		}
	}
}

extern "C" __global__ void cloth_particleContactPrepareLaunch(
	PxgFEMCloth*				clothes,
	PxgParticleSystem*			particlesystems,
	float4*						contacts,
	float4*						normalPens,
	float4*						barycentrics,
	PxgDeformablePairInfo*		contactInfos,
	PxU32*						numContacts,
	PxgParticleDbContactBlock*	contactBlocks,
	float2*						clothAppliedForces,
	float2*						particleAppliedForces
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

		PxgDeformablePairInfo contactInfo = contactInfos[workIndex];
		PxgParticleDbContactBlock& block = contactBlocks[workIndex / 32];

		if(!PxgIsValidPairInfo(contactInfo))
			continue;

		PxU64 particleRef = contactInfo.ref0;

		const PxU32 tParticleSystemId = PxgGetParticleSystemId(particleRef);
		const PxU32 tParticleIndex = PxgGetParticleIndex(particleRef);

		//second one will be cloth
		PxgFEMCloth& cloth = clothes[PxgGetDeformableId(contactInfo.ref1)];
		const PxU32 triangleInd = PxgGetDeformableFeatureId(contactInfo.ref1);

		const uint4 triVertInd = cloth.mTriangleVertexIndices_ownedEdges[triangleInd];

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

		// Computation of rest distance as the SS/SC contact prep functions do. The narrowphase writes a raw distance.
		const PxReal restDistance = particleSystem.mData.mRestOffset + cloth.mRestOffset;
		const PxReal pen = normal_pen.w + normal.dot(delta) - restDistance;

		block.normal_pen[threadIndexInWarp] = make_float4(normal.x, normal.y, normal.z, pen);
		block.barycentric[threadIndexInWarp] = barycentric;
		block.maxPenBiasClamp[threadIndexInWarp] = PxMax(cloth.mPenBiasClamp, particleSystem.mData.mPenBiasClamp);
	}
}


extern "C" __global__ void cloth_rigidAttachmentPrepareLaunch(
	PxgFEMCloth*						clothes,
	PxgDeformableAttachment*			rigidAttachments,
	PxU32*								activeRigidAttachments,
	const PxU32*						rigidAttachmentRigidRefCounts,
	PxNodeIndex*						rigidAttachmentIds,
	PxU32								numRigidAttachments,
	PxgRigidDbAttachmentBlock*			attachmentBlocks,
	const PxgPrePrepDesc*				preDesc,
	const PxgConstraintPrepareDesc*		prepareDesc,
	const PxgSolverSharedDescBase*		sharedDesc,
	float4*								rigidDeltaVel
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

		const PxgDeformableAttachment& attachment = rigidAttachments[attachmentId];
		PxgRigidDbAttachmentBlock& block = attachmentBlocks[index];

		const PxU64 clothRef = attachment.pair.ref1;
		const PxU32 clothId = PxgGetDeformableId(clothRef);
		const PxU32 elemIdx = PxgGetDeformableFeatureId(clothRef);
		const bool elemIsVertex = PxgGetDeformableFeatureType(clothRef) == PxgDeformableRef::eVERTEX;

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
			const float4 barycentric = attachment.barycentric1;
			const uint4 triVertInd = cloth.mTriangleVertexIndices_ownedEdges[elemIdx];
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

		PxNodeIndex rigidId = PxgGetRigidNode(attachment.pair.ref0);
		rigidAttachmentIds[workIndex] = rigidId;

		prepareRigidDbAttachment(block, point, invMass1, PxLoad3(attachment.barycentricOrPosition0), rigidId,
		                         clothRef, attachment.barycentric1, rigidAttachmentRigidRefCounts[workIndex],
		                         prepareDesc, preDesc->solverBodyIndices, sharedDesc,
		                         rigidDeltaVel, workIndex, numRigidAttachments);
	}
}

