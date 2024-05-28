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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "CmConeLimitHelper.h"
#include "DySolverConstraint1D.h"
#include "DyFeatherstoneArticulation.h"
#include "PxsRigidBody.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "DyDynamics.h"
#include "DyArticulationPImpl.h"
#include "foundation/PxProfiler.h"
#include "extensions/PxContactJoint.h"
#include "DyFeatherstoneArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "DyConstraint.h"
#include "DyConstraintPrep.h"
#include "DySolverContext.h"

namespace physx
{

namespace Dy
{
	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV);

	void FeatherstoneArticulation::computeLinkAccelerationInv(ArticulationData& data, ScratchData& scratchData)
	{
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		PxReal* jointAccelerations = scratchData.jointAccelerations;

		motionAccelerations[0] = Cm::SpatialVectorF::Zero();

		for (PxU32 linkID = 1; linkID < data.getLinkCount(); ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			Cm::SpatialVectorF pMotionAcceleration = translateSpatialVector(-data.getRw(linkID), motionAccelerations[link.parent]);

			Cm::SpatialVectorF motionAcceleration(PxVec3(0.f), PxVec3(0.f));

			if (jointAccelerations)
			{
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				const PxU32 jointOffset = jointDatum.jointOffset;
				const PxReal* jAcceleration = &jointAccelerations[jointOffset];
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					motionAcceleration.top += data.mWorldMotionMatrix[jointOffset + ind].top * jAcceleration[ind];
					motionAcceleration.bottom += data.mWorldMotionMatrix[jointOffset + ind].bottom * jAcceleration[ind];
				}
			}

			//calculate the acceleration due to external forces, applied external acceleration, and Coriolis force
			//used later to calculate the zero acceleration force
			motionAccelerations[linkID] = pMotionAcceleration + coriolisVectors[linkID] + motionAcceleration;
		}
	}

	//generalized force
	void FeatherstoneArticulation::computeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData)
	{
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		PxReal* jointForces = scratchData.jointForces;

		for (PxU32 linkID = (linkCount - 1); linkID > 0; --linkID)
		{
			ArticulationLink& link = data.getLink(linkID);
			
			//joint force

			spatialZAForces[link.parent] += translateSpatialVector(data.getRw(linkID), spatialZAForces[linkID]);

			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			//compute generalized force
			const PxU32 jointOffset = jointDatum.jointOffset;
			PxReal* force = &jointForces[jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				force[ind] = data.mWorldMotionMatrix[jointOffset + ind].innerProduct(spatialZAForces[linkID]);
			}
		}
	}

	void FeatherstoneArticulation::computeZAForceInv(ArticulationData& data, ScratchData& scratchData)
	{
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		
		Cm::SpatialVectorF* biasForce = scratchData.spatialZAVectors;

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			PxsBodyCore& core = *link.bodyCore;

			const PxVec3& ii = core.inverseInertia;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;
			const PxVec3 inertiaTensor = PxVec3(ii.x == 0.f ? 0.f : (1.f / ii.x), ii.y == 0.f ? 0.f : (1.f / ii.y), ii.z == 0.f ? 0.f : (1.f / ii.z));


			Cm::SpatialVectorF Ia;
			Ia.bottom = core.body2World.rotate(core.body2World.rotateInv(motionAccelerations[linkID].top).multiply(inertiaTensor));
			Ia.top = motionAccelerations[linkID].bottom * m;

			biasForce[linkID] +=Ia;
		}
	}

	void FeatherstoneArticulation::initCompositeSpatialInertia(ArticulationData& data, Dy::SpatialMatrix* compositeSpatialInertia)
	{
		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			SpatialMatrix& spatialInertia = compositeSpatialInertia[linkID];

			ArticulationLink& link = data.getLink(linkID);

			PxsBodyCore& core = *link.bodyCore;

			const PxVec3& ii = core.inverseInertia;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

			//construct mass matric
			spatialInertia.topLeft = PxMat33(PxZero);
			spatialInertia.topRight = PxMat33::createDiagonal(PxVec3(m));

			//construct inertia matrix
			PxMat33 rot(data.getLink(linkID).bodyCore->body2World.q);
			PxMat33& I = spatialInertia.bottomLeft;
			const PxVec3 inertiaTensor = PxVec3(ii.x == 0.f ? 0.f : (1.f / ii.x), ii.y == 0.f ? 0.f : (1.f / ii.y), ii.z == 0.f ? 0.f : (1.f / ii.z));
			Cm::transformInertiaTensor(inertiaTensor, rot, I);
		}
	}

	void FeatherstoneArticulation::computeCompositeSpatialInertiaAndZAForceInv(ArticulationData& data, ScratchData& scratchData)
	{
		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Dy::SpatialMatrix* compositeSpatialInertia = scratchData.compositeSpatialInertias;
		Cm::SpatialVectorF* zaForce = scratchData.spatialZAVectors;

		initCompositeSpatialInertia(data, compositeSpatialInertia);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& link = links[linkID];

			Dy::SpatialMatrix cSpatialInertia = compositeSpatialInertia[linkID];
			translateInertia(FeatherstoneArticulation::constructSkewSymmetricMatrix(data.getRw(linkID)), cSpatialInertia);
			
			//compute parent's composite spatial inertia
			compositeSpatialInertia[link.parent] += cSpatialInertia;

			//compute zero acceleration force. This is the force that would be required to support the
			//motion of all the bodies in children set if root node acceleration happened to be zero
			zaForce[link.parent] += translateSpatialVector(data.getRw(linkID), zaForce[linkID]);
		}
	}

	void FeatherstoneArticulation::computeRelativeGeneralizedForceInv(ArticulationData& data, ScratchData& scratchData)
	{
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Dy::SpatialMatrix* compositeSpatialInertia = scratchData.compositeSpatialInertias;
		Cm::SpatialVectorF* zaForce = scratchData.spatialZAVectors;
		PxReal* jointForces = scratchData.jointForces;

		Dy::SpatialMatrix invInertia = compositeSpatialInertia[0].invertInertia();
		//obtained by solving the equation: F = I * a + Z for the root where F = 0
		motionAccelerations[0] = -(invInertia * zaForce[0]);

		const PxU32 linkCount = data.getLinkCount();
		ArticulationLink* links = data.getLinks();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];

			//transform the acceleration into the child link frame
			motionAccelerations[linkID] = translateSpatialVector(-data.getRw(linkID), motionAccelerations[link.parent]);

			// equation: F = I * a + Z
			zaForce[linkID] = compositeSpatialInertia[linkID] * motionAccelerations[linkID] + zaForce[linkID];
			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			//compute generalized force
			const PxU32 jointOffset = jointDatum.jointOffset;
			PxReal* jForce = &jointForces[jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				jForce[ind] = data.mWorldMotionMatrix[jointOffset+ind].innerProduct(zaForce[linkID]);
			}
		}
	}

	void FeatherstoneArticulation::inverseDynamic(ArticulationData& data, const PxVec3& gravity,
		ScratchData& scratchData, bool computeCoriolis)
	{
		//pass 1
		computeLinkVelocities(data, scratchData);

		if(computeCoriolis)
			computeC(data, scratchData);
		else
			PxMemZero(scratchData.coriolisVectors, sizeof(Cm::SpatialVectorF)*data.getLinkCount());

		computeZ(data, gravity, scratchData);

		computeLinkAccelerationInv(data, scratchData);

		computeZAForceInv(data, scratchData);

		//pass 2
		computeGeneralizedForceInv(data, scratchData);
	}

	void FeatherstoneArticulation::inverseDynamicFloatingBase(ArticulationData& data, const PxVec3& gravity,
		ScratchData& scratchData, bool computeCoriolis)
	{
		//pass 1
		computeLinkVelocities(data, scratchData);

		if(computeCoriolis)
			computeC(data, scratchData);
		else
			PxMemZero(scratchData.coriolisVectors, sizeof(Cm::SpatialVectorF)*data.getLinkCount());

		computeZ(data, gravity, scratchData);
		//no gravity, no external accelerations because we have turned those in force in
		//computeZ
		computeLinkAccelerationInv(data, scratchData);

		computeZAForceInv(data, scratchData);

		//pass 2
		computeCompositeSpatialInertiaAndZAForceInv(data, scratchData);

		//pass 3
		computeRelativeGeneralizedForceInv(data, scratchData);
	}


	bool FeatherstoneArticulation::applyCacheToDest(ArticulationData& data, PxArticulationCache& cache,
		PxReal* jVelocities, PxReal* jPositions, PxReal* jointForces, PxReal* jointTargetPositions, PxReal* jointTargetVelocities,
		const PxArticulationCacheFlags flag, bool& shouldWake)
	{
		bool needsScheduling = !mGPUDirtyFlags;

		bool localShouldWake = false;
		if (flag & PxArticulationCacheFlag::eVELOCITY)
		{
			const PxU32 dofCount = data.getDofs();
			for (PxU32 i = 0; i < dofCount; ++i)
			{
				const PxReal jv = cache.jointVelocity[i];
				localShouldWake = localShouldWake || jv != 0.f;
				jVelocities[i] = jv;
			}

			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_VELOCITIES;
		}

		if (flag & PxArticulationCacheFlag::eROOT_TRANSFORM)
		{
			ArticulationLink& rLink = mArticulationData.getLink(0);
			// PT:: tag: scalar transform*transform
			rLink.bodyCore->body2World = cache.rootLinkData->transform * rLink.bodyCore->getBody2Actor();
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM;
		}

		if(flag & PxArticulationCacheFlag::eROOT_VELOCITIES)
		{
			ArticulationLink& rLink = mArticulationData.getLink(0);
			rLink.bodyCore->linearVelocity = cache.rootLinkData->worldLinVel;
			rLink.bodyCore->angularVelocity = cache.rootLinkData->worldAngVel;
			localShouldWake = localShouldWake || (!cache.rootLinkData->worldLinVel.isZero()) || (!cache.rootLinkData->worldAngVel.isZero());
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES;
		}
		
		if (flag & PxArticulationCacheFlag::ePOSITION)
		{
			copyJointData(data, jPositions, cache.jointPosition);
			//When we update the joint positions, we also have to update the link state, so need to make links
			//dirty!
			mGPUDirtyFlags |= (ArticulationDirtyFlag::eDIRTY_POSITIONS); 
		}

		if (flag & PxArticulationCacheFlag::eFORCE)
		{
			const PxU32 dofCount = data.getDofs();
			for (PxU32 i = 0; i < dofCount; ++i)
			{
				const PxReal jf = cache.jointForce[i];
				localShouldWake = localShouldWake || jf != 0.f;
				jointForces[i] = jf;
			}
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_FORCES;
		}

		if (flag & PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS)
		{
			const PxU32 dofCount = data.getDofs();
			for (PxU32 i = 0; i < dofCount; ++i)
			{
				const PxReal jt = cache.jointTargetPositions[i];
				localShouldWake = localShouldWake || jt != jPositions[i];
				jointTargetPositions[i] = jt;
			}
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_POS;
		}

		if (flag & PxArticulationCacheFlag::eJOINT_TARGET_VELOCITIES)
		{
			const PxU32 dofCount = data.getDofs();
			for (PxU32 i = 0; i < dofCount; ++i)
			{
				const PxReal jv = cache.jointTargetVelocities[i];
				localShouldWake = localShouldWake || jv != jVelocities[i];
				jointTargetVelocities[i] = jv;
			}
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_JOINT_TARGET_VEL;
		}

		// the updateKinematic functions rely on updated joint frames.
		if (mJcalcDirty)
		{
			jcalc(data);
		}
		mJcalcDirty = false;

		if (flag & (PxArticulationCacheFlag::ePOSITION | PxArticulationCacheFlag::eROOT_TRANSFORM))
		{
			//update link's position based on the joint position
			teleportLinks(data);
		}

		if (flag & (PxArticulationCacheFlag::eVELOCITY | PxArticulationCacheFlag::ePOSITION | PxArticulationCacheFlag::eROOT_VELOCITIES | PxArticulationCacheFlag::eROOT_TRANSFORM))
		{
			computeLinkVelocities(data);
		}
		shouldWake = localShouldWake;
		return needsScheduling;
	}

	void FeatherstoneArticulation::packJointData(const PxReal* maximum, PxReal* reduced)
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
	
			ArticulationLink& linkDatum = mArticulationData.getLink(linkID);
			ArticulationJointCore* joint = linkDatum.inboundJoint;
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			const PxReal* maxJointData = &maximum[(linkID - 1) * DY_MAX_DOF];
			PxReal* reducedJointData = &reduced[jointDatum.jointOffset];

			PxU32 count = 0;
			for (PxU32 j = 0; j < DY_MAX_DOF; ++j)
			{
				PxArticulationMotions motion = PxArticulationMotions(joint->motion[j]);
				if (motion != PxArticulationMotion::eLOCKED)
				{
					reducedJointData[count] = maxJointData[j];
					count++;
				}
			}

			PX_ASSERT(count == jointDatum.dof);
		}

	}

	void FeatherstoneArticulation::unpackJointData(const PxReal* reduced, PxReal* maximum)
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
			ArticulationLink& linkDatum = mArticulationData.getLink(linkID);
			ArticulationJointCore* joint = linkDatum.inboundJoint;
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			PxReal* maxJointData = &maximum[(linkID - 1) * DY_MAX_DOF];
			const PxReal* reducedJointData = &reduced[jointDatum.jointOffset];

			PxU32 count = 0;
			for (PxU32 j = 0; j < DY_MAX_DOF; ++j)
			{
				PxArticulationMotions motion = PxArticulationMotions(joint->motion[j]);
				if (motion != PxArticulationMotion::eLOCKED)
				{
					maxJointData[j] = reducedJointData[count];
					count++;
				}
				else
				{
					maxJointData[j] = 0.f;
				}
			}

			PX_ASSERT(count == jointDatum.dof);
		}
	}

	void FeatherstoneArticulation::initializeCommonData()
	{
		if (mJcalcDirty)
		{
			jcalc(mArticulationData);
			mJcalcDirty = false;
		}

		{
			//constants
			const ArticulationLink* links = mArticulationData.getLinks();
			const PxU32 linkCount = mArticulationData.getLinkCount();
			const ArticulationJointCoreData* jointCoreDatas = mArticulationData.getJointData();
			const Cm::UnAlignedSpatialVector* motionMatrices = mArticulationData.getMotionMatrix();

			//outputs
			PxTransform* accumulatedPoses = mArticulationData.getAccumulatedPoses();
			PxVec3* rws = mArticulationData.getRw();
			Cm::UnAlignedSpatialVector* motionMatricesW = mArticulationData.getWorldMotionMatrix();

			computeRelativeTransformC2P(
				links, linkCount, jointCoreDatas, motionMatrices,
				accumulatedPoses, rws, motionMatricesW);
		}

		computeRelativeTransformC2B(mArticulationData);

		computeSpatialInertia(mArticulationData);

		mArticulationData.setDataDirty(false);
	}

	void FeatherstoneArticulation::getGeneralizedGravityForce(const PxVec3& gravity, PxArticulationCache& cache)
	{

		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Articulation::getGeneralisedGravityForce() commonInit need to be called first to initialize data!");
			return;
		}

#if FEATHERSTONE_DEBUG
		PxReal* jointForce = reinterpret_cast<PxReal*>(PX_ALLOC(sizeof(PxReal) * mArticulationData.getDofs(), "jointForce"));
		{
			
			const PxU32 linkCount = mArticulationData.getLinkCount();

			PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

			ScratchData scratchData;
			PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

			scratchData.jointVelocities = NULL;
			scratchData.jointAccelerations = NULL;
			scratchData.jointForces = jointForce;

			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
			if (fixBase)
				inverseDynamic(mArticulationData, gravity, scratchData, false);
			else
				inverseDynamicFloatingBase(mArticulationData, gravity, scratchData, false);

			allocator->free(tempMemory);
		}
#endif

		const PxVec3 tGravity = -gravity;
		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);
		const PxU32 linkCount = mArticulationData.getLinkCount();

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		if (fixBase)
		{
			Cm::SpatialVectorF* spatialZAForces = reinterpret_cast<Cm::SpatialVectorF*>(allocator->alloc(sizeof(Cm::SpatialVectorF) * linkCount));

			for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
			{
				ArticulationLink& link = mArticulationData.getLink(linkID);

				PxsBodyCore& core = *link.bodyCore;

				const PxReal m = 1.0f / core.inverseMass;

				const PxVec3 linkGravity = tGravity;

				spatialZAForces[linkID].top = m*linkGravity;
				spatialZAForces[linkID].bottom = PxVec3(0.f);
			}

			ScratchData scratchData;
			scratchData.spatialZAVectors = spatialZAForces;
			scratchData.jointForces = cache.jointForce;

			computeGeneralizedForceInv(mArticulationData, scratchData);

			//release spatialZA vectors
			allocator->free(spatialZAForces);
		}
		else
		{
			ScratchData scratchData;
			PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

			scratchData.jointVelocities = NULL;
			scratchData.jointAccelerations = NULL;
			scratchData.jointForces = cache.jointForce;
			scratchData.externalAccels = NULL;

			inverseDynamicFloatingBase(mArticulationData, tGravity, scratchData, false);

			allocator->free(tempMemory);
		}

#if FEATHERSTONE_DEBUG
		//compare joint force
		const PxU32 totalDofs = mArticulationData.getDofs();
		for (PxU32 i = 0; i < totalDofs; ++i)
		{
			const PxReal dif = jointForce[i] - cache.jointForce[i];
			PX_ASSERT(PxAbs(dif) < 5e-3f);
		}

		PX_FREE(jointForce);
#endif

	}

	//gravity, acceleration and external force(external acceleration) are zero
	void  FeatherstoneArticulation::getCoriolisAndCentrifugalForce(PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Articulation::getCoriolisAndCentrifugalForce() commonInit need to be called first to initialize data!");
			return;
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		scratchData.jointVelocities = cache.jointVelocity;
		scratchData.jointAccelerations = NULL;
		scratchData.jointForces = cache.jointForce;
		scratchData.externalAccels = NULL;
		
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		if (fixBase)
			inverseDynamic(mArticulationData, PxVec3(0.f), scratchData, true);
		else
			inverseDynamicFloatingBase(mArticulationData, PxVec3(0.f), scratchData, true);

		allocator->free(tempMemory);
	}

	//gravity, joint acceleration and joint velocity are zero
	void  FeatherstoneArticulation::getGeneralizedExternalForce(PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Articulation::getCoriolisAndCentrifugalForce() commonInit need to be called first to initialize data!");
			return;
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		scratchData.jointVelocities = NULL;
		scratchData.jointAccelerations = NULL;
		scratchData.jointForces = cache.jointForce;
		
		Cm::SpatialVector* accels = reinterpret_cast<Cm::SpatialVector*>(allocator->alloc(sizeof(Cm::SpatialVector) * linkCount));

		//turn external forces to external accels
		for (PxU32 i = 0; i < linkCount; ++i)
		{
			ArticulationLink& link = mArticulationData.getLink(i);
			PxsBodyCore& core = *link.bodyCore;
			
			const PxSpatialForce& force = cache.externalForces[i];
			Cm::SpatialVector& accel = accels[i];

			accel.linear = force.force * core.inverseMass;

			PxMat33 inverseInertiaWorldSpace;
			Cm::transformInertiaTensor(core.inverseInertia, PxMat33(core.body2World.q), inverseInertiaWorldSpace);

			accel.angular = inverseInertiaWorldSpace * force.torque;
		}
		
		scratchData.externalAccels = accels;

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		if (fixBase)
			inverseDynamic(mArticulationData, PxVec3(0.f), scratchData, false);
		else
			inverseDynamicFloatingBase(mArticulationData, PxVec3(0.f), scratchData, false);

		allocator->free(tempMemory);
		allocator->free(accels);
	}

	//provided joint acceleration, calculate joint force
	void FeatherstoneArticulation::getJointForce(PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "ArticulationHelper::getJointForce() commonInit need to be called first to initialize data!");
			return;
		}

		//const PxU32 size = sizeof(PxReal) * mArticulationData.getDofs();
		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);
		//PxReal* jointVelocities = reinterpret_cast<PxReal*>(allocator->alloc(size));

		ScratchData scratchData;
		scratchData.jointVelocities = NULL;//jont velocity will be zero
		scratchData.jointAccelerations = cache.jointAcceleration; //input
		scratchData.jointForces = cache.jointForce; //output
		scratchData.externalAccels = NULL;

		PxU8* tempMemory = allocateScratchSpatialData(allocator, mArticulationData.getLinkCount(), scratchData);

		//make sure joint velocity be zero
		//PxMemZero(jointVelocities, sizeof(PxReal) * mArticulationData.getDofs());
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		if (fixBase)
			inverseDynamic(mArticulationData, PxVec3(0.f), scratchData, false);
		else
			inverseDynamicFloatingBase(mArticulationData, PxVec3(0.f), scratchData, false);

		//allocator->free(jointVelocities);
		allocator->free(tempMemory);
	}

	void FeatherstoneArticulation::jcalcLoopJointSubspace(ArticulationJointCore* joint,
		ArticulationJointCoreData& jointDatum, SpatialSubspaceMatrix& T,
		const Cm::UnAlignedSpatialVector* jointAxis)
	{
		PX_UNUSED(jointDatum);
		const PxVec3 childOffset = -joint->childPose.p;
		const PxVec3 zero(0.f);

		//if the column is free, we put zero for it, this is for computing K(coefficient matrix)
		T.setNumColumns(6);

		//transpose(Tc)*S = 0
		//transpose(Ta)*S = 1
		switch (joint->jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		{
			PX_ASSERT(jointDatum.dof == 1);

			const PxVec3 rx = (joint->childPose.rotate(PxVec3(1.f, 0.f, 0.f))).getNormalized();
			const PxVec3 ry = (joint->childPose.rotate(PxVec3(0.f, 1.f, 0.f))).getNormalized();
			const PxVec3 rz = (joint->childPose.rotate(PxVec3(0.f, 0.f, 1.f))).getNormalized();

			//joint->activeForceSubspace.setNumColumns(1);

			if (jointAxis[0][3] == 1.f)
			{
				//x is the free translation axis
				T.setColumn(0, rx, zero);
				T.setColumn(1, ry, zero);
				T.setColumn(2, rz, zero);
				T.setColumn(3, zero, zero);
				T.setColumn(4, zero, ry);
				T.setColumn(5, zero, rz);

				//joint->activeForceSubspace.setColumn(0, PxVec3(0.f), rx);
			}
			else if (jointAxis[0][4] == 1.f)
			{
				//y is the free translation axis
				T.setColumn(0, rx, zero);
				T.setColumn(1, ry, zero);
				T.setColumn(2, rz, zero);
				T.setColumn(3, zero, rx);
				T.setColumn(4, zero, zero);
				T.setColumn(5, zero, rz);

				//joint->activeForceSubspace.setColumn(0, PxVec3(0.f), ry);
			}
			else if (jointAxis[0][5] == 1.f)
			{
				//z is the free translation axis
				T.setColumn(0, rx, zero);
				T.setColumn(1, ry, zero);
				T.setColumn(2, rx, zero);
				T.setColumn(3, zero, rx);
				T.setColumn(4, zero, ry);
				T.setColumn(5, zero, zero);

				//joint->activeForceSubspace.setColumn(0, PxVec3(0.f), rz);
			}

			break;
		}
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			//joint->activeForceSubspace.setNumColumns(1);

			const PxVec3 rx = (joint->childPose.rotate(PxVec3(1.f, 0.f, 0.f))).getNormalized();
			const PxVec3 ry = (joint->childPose.rotate(PxVec3(0.f, 1.f, 0.f))).getNormalized();
			const PxVec3 rz = (joint->childPose.rotate(PxVec3(0.f, 0.f, 1.f))).getNormalized();

			const PxVec3 rxXd = rx.cross(childOffset);
			const PxVec3 ryXd = ry.cross(childOffset);
			const PxVec3 rzXd = rz.cross(childOffset);

			if (jointAxis[0][0] == 1.f)
			{
				//x is the free rotation axis

				T.setColumn(0, zero, zero);
				T.setColumn(1, ry, zero);
				T.setColumn(2, rz, zero);

				//joint->activeForceSubspace.setColumn(0, rx, PxVec3(0.f));

			}
			else if (jointAxis[0][1] == 1.f)
			{
				//y is the free rotation axis
				T.setColumn(0, rx, zero);
				T.setColumn(1, zero, zero);
				T.setColumn(2, rz, zero);

				//joint->activeForceSubspace.setColumn(0, ry, PxVec3(0.f));
			}
			else if (jointAxis[0][2] == 1.f)
			{
				//z is the rotation axis
				T.setColumn(0, rx, zero);
				T.setColumn(1, ry, zero);
				T.setColumn(2, zero, zero);

				//joint->activeForceSubspace.setColumn(0, rz, PxVec3(0.f));
			}

			T.setColumn(3, rxXd, rx);
			T.setColumn(4, ryXd, ry);
			T.setColumn(5, rzXd, rz);

			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
			//joint->activeForceSubspace.setNumColumns(3);

			const PxVec3 rx = (joint->childPose.rotate(PxVec3(1.f, 0.f, 0.f))).getNormalized();
			const PxVec3 ry = (joint->childPose.rotate(PxVec3(0.f, 1.f, 0.f))).getNormalized();
			const PxVec3 rz = (joint->childPose.rotate(PxVec3(0.f, 0.f, 1.f))).getNormalized();

			const PxVec3 rxXd = rx.cross(childOffset);
			const PxVec3 ryXd = ry.cross(childOffset);
			const PxVec3 rzXd = rz.cross(childOffset);

			T.setColumn(0, zero, zero);
			T.setColumn(1, zero, zero);
			T.setColumn(2, zero, zero);

			T.setColumn(3, rxXd, rx);
			T.setColumn(4, ryXd, ry);
			T.setColumn(5, rzXd, rz);

			//need to implement constraint force subspace matrix and active force subspace matrix

			break;
		}
		case PxArticulationJointType::eFIX:
		{
			//joint->activeForceSubspace.setNumColumns(0);
			//T.setNumColumns(6);

			/*	const PxVec3 rx = (joint->childPose.rotate(PxVec3(1.f, 0.f, 0.f))).getNormalized();
			const PxVec3 ry = (joint->childPose.rotate(PxVec3(0.f, 1.f, 0.f))).getNormalized();
			const PxVec3 rz = (joint->childPose.rotate(PxVec3(0.f, 0.f, 1.f))).getNormalized();

			T.setColumn(0, rx, PxVec3(0.f));
			T.setColumn(1, ry, PxVec3(0.f));
			T.setColumn(2, rz, PxVec3(0.f));
			T.setColumn(3, PxVec3(0.f), rx);
			T.setColumn(4, PxVec3(0.f), ry);
			T.setColumn(5, PxVec3(0.f), rz);
			*/

			T.setColumn(0, PxVec3(1.f, 0.f, 0.f), zero);
			T.setColumn(1, PxVec3(0.f, 1.f, 0.f), zero);
			T.setColumn(2, PxVec3(0.f, 0.f, 1.f), zero);
			T.setColumn(3, zero, PxVec3(1.f, 0.f, 0.f));
			T.setColumn(4, zero, PxVec3(0.f, 1.f, 0.f));
			T.setColumn(5, zero, PxVec3(0.f, 0.f, 1.f));

			PX_ASSERT(jointDatum.dof == 0);
			break;
		}
		default:
			break;

		}
	}

	//This method supports just one loopJoint
	void FeatherstoneArticulation::getKMatrix(ArticulationJointCore* loopJoint, const PxU32 parentIndex, const PxU32 childIndex, PxArticulationCache& cache)
	{
		PX_UNUSED(loopJoint);
		PX_UNUSED(parentIndex);
		PX_UNUSED(childIndex);
		PX_UNUSED(cache);

		////initialize all tree links motion subspace matrix
		//jcalc(mArticulationData);

		////linkID is the parent link, ground is the child link so child link is the fix base
		//ArticulationLinkData& pLinkDatum = mArticulationData.getLinkData(parentIndex);

		//ArticulationLink& cLink = mArticulationData.getLink(childIndex);
		//ArticulationLinkData& cLinkDatum = mArticulationData.getLinkData(childIndex);
		//
		//ArticulationJointCoreData loopJointDatum;
		//loopJointDatum.computeJointDof(loopJoint);

		////this is constraintForceSubspace in child body space(T)
		//SpatialSubspaceMatrix T;

		////loop joint constraint subspace matrix(T)
		//jcalcLoopJointSubspace(loopJoint, loopJointDatum, T);

		//const PxU32 linkCount = mArticulationData.getLinkCount();
		////set Jacobian matrix to be zero
		//PxMemZero(cache.jacobian, sizeof(PxKinematicJacobian) * linkCount);

		////transform T to world space
		//PxTransform& body2World = cLink.bodyCore->body2World;

		//for (PxU32 ind = 0; ind < T.getNumColumns(); ++ind)
		//{
		//	Cm::SpatialVectorF& column = T[ind];
		//	T.setColumn(ind, body2World.rotate(column.top), body2World.rotate(column.bottom));
		//}

		//const Cm::SpatialVectorF& pAccel = pLinkDatum.motionAcceleration;
		//const Cm::SpatialVectorF& cAccel = cLinkDatum.motionAcceleration;

		//const Cm::SpatialVectorF& pVel = pLinkDatum.motionVelocity;
		//const Cm::SpatialVectorF& cVel = cLinkDatum.motionVelocity;

		//Cm::SpatialVectorF k = (pAccel - cAccel) + pVel.cross(cVel);
		//k = T.transposeMultiply(k);
		//k = -k;

		//PxU32 i = childIndex;
		//PxU32 j = parentIndex;

		//PxU32* index = NULL;

		//while (i != j)
		//{
		//	if (i > j)
		//		index = &i;
		//	else
		//		index = &j;

		//	const PxU32 linkIndex = *index;

		//	PxKinematicJacobian* K = cache.jacobian + linkIndex;

		//	ArticulationLink& link = mArticulationData.getLink(linkIndex);

		//	ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkIndex);

		//	SpatialSubspaceMatrix& S = jointDatum.motionMatrix;

		//	PxTransform& tBody2World = link.bodyCore->body2World;

		//	Cm::SpatialVectorF res;
		//	for (PxU32 ind = 0; ind < S.getNumColumns(); ++ind)
		//	{
		//		Cm::SpatialVectorF& sCol = S[ind];

		//		//transform spatial axis into world space
		//		sCol.top = tBody2World.rotate(sCol.top);
		//		sCol.bottom = tBody2World.rotate(sCol.bottom);

		//		res = T.transposeMultiply(sCol);
		//		res = -res;

		//		PxReal* kSubMatrix = K->j[ind];

		//		kSubMatrix[0] = res.top.x; kSubMatrix[1] = res.top.y; kSubMatrix[2] = res.top.z;
		//		kSubMatrix[3] = res.bottom.x; kSubMatrix[4] = res.bottom.y; kSubMatrix[5] = res.bottom.z;
		//	}

		//	//overwrite either i or j to its parent index
		//	*index = link.parent;
		//}
	}


	void FeatherstoneArticulation::getCoefficientMatrix(const PxReal dt, const PxU32 linkID, const PxContactJoint* contactJoints, const PxU32 nbContacts, PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "ArticulationHelper::getCoefficientMatrix() commonInit need to be called first to initialize data!");
			return;
		}

		computeArticulatedSpatialInertia(mArticulationData);

		ArticulationLink* links = mArticulationData.getLinks();
	
		const PxU32 linkCount = mArticulationData.getLinkCount();

		PxReal* coefficientMatrix = cache.coefficientMatrix;

		const PxU32 elementCount = mArticulationData.getDofs();
		
		//zero coefficient matrix
		PxMemZero(coefficientMatrix, sizeof(PxReal) * elementCount * nbContacts);

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
	
		for (PxU32 a = 0; a < nbContacts; ++a)
		{
			PxJacobianRow row;
			contactJoints[a].computeJacobians(&row);

			//impulse lin is contact normal, and ang is raxn. R is body2World, R(t) is world2Body
			//| R(t),	0	|
			//| R(t)*r, R(t)|
			//r is the vector from center of mass to contact point
			//p(impluse) =	|n|
			//				|0|

			//transform p(impluse) from work space to the local space of link
			ArticulationLink& link = links[linkID];
			PxTransform& body2World = link.bodyCore->body2World;

			PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);
			ScratchData scratchData;
			PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

			Cm::SpatialVectorF* Z = scratchData.spatialZAVectors;

			//make sure all links' spatial zero acceleration impulse are zero
			PxMemZero(Z, sizeof(Cm::SpatialVectorF) * linkCount);

			const Cm::SpatialVectorF impl(body2World.rotateInv(row.linear0), body2World.rotateInv(row.angular0));

			getZ(linkID, mArticulationData, Z, impl);

			const PxU32 totalDofs = mArticulationData.getDofs();

			const PxU32 size = sizeof(PxReal) * totalDofs;

			PxU8* tData = reinterpret_cast<PxU8*>(allocator->alloc(size * 2));

			PxReal* jointVelocities = reinterpret_cast<PxReal*>(tData);
			PxReal* jointAccelerations = reinterpret_cast<PxReal*>(tData + size);
			//zero joint Velocites
			PxMemZero(jointVelocities, size);

			getDeltaVWithDeltaJV(fixBase, linkID, mArticulationData, Z, jointVelocities);

			const PxReal invDt = 1.f / dt;
			//calculate joint acceleration due to velocity change
			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				jointAccelerations[i] = jointVelocities[i] * invDt;
			}

			//compute individual link's spatial inertia tensor. This is very important
			computeSpatialInertia(mArticulationData);

			PxReal* coeCol = &coefficientMatrix[elementCount * a];

			//this means the joint force calculated by the inverse dynamic
			//will be just influenced by joint acceleration change
			scratchData.jointVelocities = NULL;
			scratchData.externalAccels = NULL;

			//Input
			scratchData.jointAccelerations = jointAccelerations;

			//a column of the coefficient matrix is the joint force
			scratchData.jointForces = coeCol;

			if (fixBase)
			{
				inverseDynamic(mArticulationData, PxVec3(0.f), scratchData, false);
			}
			else
			{
				inverseDynamicFloatingBase(mArticulationData, PxVec3(0.f), scratchData, false);
			}

			allocator->free(tData);
			allocator->free(tempMemory);
		}
	}

	void FeatherstoneArticulation::getImpulseResponseSlowInv(Dy::ArticulationLink* links,
		const ArticulationData& data,
		PxU32 linkID0_,
		const Cm::SpatialVector& impulse0,
		Cm::SpatialVector& deltaV0,
		PxU32 linkID1_,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV1,
		PxReal* jointVelocities,
		Cm::SpatialVectorF* Z)
	{
		PX_UNUSED(jointVelocities);

		const PxU32 numLinks = data.getLinkCount();

		PX_ALLOCA(_stack, PxU32, numLinks);
		PxU32* stack = _stack;

		PxU32 i0, i1, ic;

		PxU32 linkID0 = linkID0_;
		PxU32 linkID1 = linkID1_;


		for (i0 = linkID0, i1 = linkID1; i0 != i1;)	// find common path
		{
			if (i0<i1)
				i1 = links[i1].parent;
			else
				i0 = links[i0].parent;
		}

		PxU32 common = i0;

		Cm::SpatialVectorF Z0(-impulse0.linear, -impulse0.angular);
		Cm::SpatialVectorF Z1(-impulse1.linear, -impulse1.angular);

		Z[linkID0] = Z0;
		Z[linkID1] = Z1;

		//for (i0 = linkID0; i0 != common; i0 = links[i0].parent)
		for (i0 = 0; linkID0 != common; linkID0 = links[linkID0].parent)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(linkID0).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(linkID0).dof;
			Z0 = propagateImpulseW(
					data.getRw(linkID0), 
					Z0, 
					NULL, &data.getWorldIsInvD(jointOffset), &data.getWorldMotionMatrix(jointOffset), dofCount,
					NULL);
			Z[links[linkID0].parent] = Z0;
			stack[i0++] = linkID0;
		}

		for (i1 = i0; linkID1 != common; linkID1 = links[linkID1].parent)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(linkID1).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(linkID1).dof;
			Z1 = propagateImpulseW(
					data.getRw(linkID1), 
					Z1, 
					NULL, &data.getWorldIsInvD(jointOffset), &data.getWorldMotionMatrix(jointOffset), dofCount,
					NULL);
			Z[links[linkID1].parent] = Z1;
			stack[i1++] = linkID1;
		}

		//KS - we can replace the following section of code with the impulse response matrix - until next comment!

		Cm::SpatialVectorF ZZ = Z0 + Z1;
		Z[common] = ZZ;
		for (ic = i1; common; common = links[common].parent)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(common).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(common).dof;
			Z[links[common].parent] = propagateImpulseW(
				data.getRw(common), 
				Z[common], 
				NULL, &data.getWorldIsInvD(jointOffset), &data.getMotionMatrix(jointOffset), dofCount,
				NULL);
			stack[ic++] = common;
		}

		if(data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE)
			Z[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));

		//SpatialMatrix inverseArticulatedInertia = data.getLinkData(0).spatialArticulatedInertia.getInverse();
		const SpatialMatrix& inverseArticulatedInertia = data.getBaseInvSpatialArticulatedInertiaW();
		Cm::SpatialVectorF v = inverseArticulatedInertia * (-Z[0]);

		for (PxU32 index = ic; (index--) > i1;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = mArticulationData.getJointData(id).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(id).dof;
			v = FeatherstoneArticulation::propagateVelocityW(
				data.getRw(id), v, 
				data.mWorldSpatialArticulatedInertia[id], Z[id],
				NULL, data.mInvStIs[id], &data.getWorldMotionMatrix(jointOffset), dofCount,
				jointVelocities);
		}

		//Replace everything to here with the impulse response matrix multiply

		Cm::SpatialVectorF dv1 = v;
		for (PxU32 index = i1; (index--) > i0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = mArticulationData.getJointData(id).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(id).dof;
			dv1 = FeatherstoneArticulation::propagateVelocityW(
				data.getRw(id), v, 
				data.mWorldSpatialArticulatedInertia[id], Z[id], 
				NULL, data.mInvStIs[id], &data.getWorldMotionMatrix(jointOffset), dofCount,
				jointVelocities);
		}

		Cm::SpatialVectorF dv0 = v;
		for (PxU32 index = i0; (index--) > 0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = mArticulationData.getJointData(id).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(id).dof;
			dv0 = FeatherstoneArticulation::propagateVelocityW(
				data.getRw(id), v, 
				data.mWorldSpatialArticulatedInertia[id], Z[id], 
				NULL, data.mInvStIs[id], &data.getWorldMotionMatrix(jointOffset), dofCount,
				jointVelocities);
		}

		deltaV0.linear = dv0.bottom;
		deltaV0.angular = dv0.top;

		deltaV1.linear = dv1.bottom;
		deltaV1.angular = dv1.top;
	}

	void FeatherstoneArticulation::getImpulseSelfResponseInv(const bool fixBase, 
		PxU32 linkID0,
		PxU32 linkID1,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse0,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV0,
		Cm::SpatialVector& deltaV1,
		PxReal* jointVelocities)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		//transform p(impluse) from work space to the local space of link
		ArticulationLink& link = links[linkID1];
		//ArticulationLinkData& linkDatum = mArticulationData.getLinkData(linkID1);

		if (link.parent == linkID0)
		{
			PX_ASSERT(linkID0 == link.parent);
			PX_ASSERT(linkID0 < linkID1);
			
			//impulse is in world space
			const Cm::SpatialVector& imp1 = impulse1;
			const Cm::SpatialVector& imp0 = impulse0;


			Cm::SpatialVectorF pImpulse(imp0.linear, imp0.angular);

			PX_ASSERT(linkID0 == link.parent);

			const PxU32 jointOffset = mArticulationData.getJointData(linkID1).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(linkID1).dof;
			
			//initialize child link spatial zero acceleration impulse
			Cm::SpatialVectorF Z1(-imp1.linear, -imp1.angular);
			//this calculate parent link spatial zero acceleration impulse
			Cm::SpatialVectorF Z0 = propagateImpulseW(
				mArticulationData.getRw(linkID1), 
				Z1,
				NULL, &mArticulationData.mISInvStIS[jointOffset], &mArticulationData.mWorldMotionMatrix[jointOffset], dofCount,
				NULL);

			//in parent space
			const Cm::SpatialVectorF impulseDif = pImpulse - Z0;

			Cm::SpatialVectorF delV0(PxVec3(0.f), PxVec3(0.f));
			Cm::SpatialVectorF delV1(PxVec3(0.f), PxVec3(0.f));

			//calculate velocity change start from the parent link to the root
			delV0 = FeatherstoneArticulation::getImpulseResponseWithJ(linkID0, fixBase, mArticulationData, Z, impulseDif, jointVelocities);

			//calculate velocity change for child link
			delV1 = FeatherstoneArticulation::propagateVelocityW(
				mArticulationData.getRw(linkID1), delV0, 
				mArticulationData.mWorldSpatialArticulatedInertia[linkID1], Z1, 
				NULL, mArticulationData.mInvStIs[linkID1], &mArticulationData.mWorldMotionMatrix[jointOffset], dofCount,
				jointVelocities);

			//translate delV0 and delV1 into world space again
			deltaV0.linear = delV0.bottom;
			deltaV0.angular = delV0.top;
			deltaV1.linear = delV1.bottom;
			deltaV1.angular = delV1.top;
		}
		else
		{
			getImpulseResponseSlowInv(links, mArticulationData, linkID0, impulse0, deltaV0, linkID1,impulse1, deltaV1, jointVelocities, Z);
		}
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getImpulseResponseInv(
		const bool fixBase, const PxU32 linkID,
		Cm::SpatialVectorF* Z, 
		const Cm::SpatialVector& impulse,
		PxReal* jointVelocities)
	{
		//impulse lin is contact normal, and ang is raxn. R is body2World, R(t) is world2Body
		//| R(t),	0	|
		//| R(t)*r, R(t)|
		//r is the vector from center of mass to contact point
		//p(impluse) =	|n|
		//				|0|

		ArticulationLink* links = mArticulationData.getLinks();
		//ArticulationLinkData* linkData = mArticulationData.getLinkData();
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();
		const PxU32 linkCount = mArticulationData.getLinkCount();

		//make sure all links' spatial zero acceleration impulse are zero
		PxMemZero(Z, sizeof(Cm::SpatialVectorF) * linkCount);

		Z[linkID] = Cm::SpatialVectorF(-impulse.linear, -impulse.angular);

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			ArticulationLink& tLink = links[i];
			const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(i).dof;
			//ArticulationLinkData& tLinkDatum = linkData[i];
			Z[tLink.parent] = propagateImpulseW(
				mArticulationData.getRw(i),
				Z[i], 
				NULL, &mArticulationData.mISInvStIS[jointOffset], &mArticulationData.mWorldMotionMatrix[jointOffset], dofCount,
				NULL);
		}

		//set velocity change of the root link to be zero
		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		if (!fixBase)
			deltaV = mArticulationData.mBaseInvSpatialArticulatedInertiaW * (-Z[0]);

		const PxU32 startIndex = links[linkID].mPathToRootStartIndex;
		const PxU32 numElems = links[linkID].mPathToRootCount;

		const PxU32* pathToRoot = &mArticulationData.mPathToRootElements[startIndex];

		for(PxU32 i = 0; i < numElems; ++i)
		{
			const PxU32 index = pathToRoot[i];
			PX_ASSERT(links[index].parent < index);

			ArticulationJointCoreData& tJointDatum = jointData[index];

			PxReal* jVelocity = &jointVelocities[tJointDatum.jointOffset];
			deltaV = propagateVelocityW(
				mArticulationData.getRw(index), deltaV, 
				mArticulationData.mWorldSpatialArticulatedInertia[index], Z[index], 
				NULL, mArticulationData.mInvStIs[index], &mArticulationData.mWorldMotionMatrix[tJointDatum.jointOffset], tJointDatum.dof,
				jVelocity);
		}

		return deltaV;

	}


	void FeatherstoneArticulation::getCoefficientMatrixWithLoopJoints(ArticulationLoopConstraint* lConstraints, const PxU32 nbConstraints, PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "ArticulationHelper::getCoefficientMatrix() commonInit need to be called first to initialize data!");
			return;
		}

		computeArticulatedSpatialInertia(mArticulationData);

		const PxU32 linkCount = mArticulationData.getLinkCount();
		 
		PxReal* coefficientMatrix = cache.coefficientMatrix;

		const PxU32 elementCount = mArticulationData.getDofs();

		//zero coefficient matrix
		PxMemZero(coefficientMatrix, sizeof(PxReal) * elementCount * nbConstraints);

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);
		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		Cm::SpatialVectorF* Z = scratchData.spatialZAVectors;
		const PxU32 totalDofs = mArticulationData.getDofs();

		const PxU32 size = sizeof(PxReal) * totalDofs;

		PxU8* tData = reinterpret_cast<PxU8*>(allocator->alloc(size * 2));

		const PxReal invDt = 1.f / mArticulationData.getDt();
		PxReal* jointVelocities = reinterpret_cast<PxReal*>(tData);
		PxReal* jointAccelerations = reinterpret_cast<PxReal*>(tData + size);

		for (PxU32 a = 0; a < nbConstraints; ++a)
		{
			ArticulationLoopConstraint& lConstraint = lConstraints[a];
			Constraint* aConstraint = lConstraint.constraint;

			Px1DConstraint rows[MAX_CONSTRAINT_ROWS];
			setupConstraintRows(rows, MAX_CONSTRAINT_ROWS);
			
			const PxTransform idt(PxIdentity);
			const PxTransform& body2World0 = aConstraint->body0 ? aConstraint->bodyCore0->body2World : idt;
			const PxTransform& body2World1 = aConstraint->body1 ? aConstraint->bodyCore1->body2World : idt;

			PxVec3p unused_body0WorldOffset(0.0f);
			PxVec3p unused_ra, unused_rb;
			PxConstraintInvMassScale unused_invMassScales;
			//TAG:solverprepcall
			PxU32 constraintCount = (*aConstraint->solverPrep)(rows,
				unused_body0WorldOffset,
				MAX_CONSTRAINT_ROWS,
				unused_invMassScales,
				aConstraint->constantBlock,
				body2World0, body2World1, !!(aConstraint->flags & PxConstraintFlag::eENABLE_EXTENDED_LIMITS), unused_ra, unused_rb);

			const PxU32 linkIndex0 = lConstraint.linkIndex0;
			const PxU32 linkIndex1 = lConstraint.linkIndex1;

			//zero joint Velocites
			PxMemZero(jointVelocities, size);

			for (PxU32 j = 0; j < constraintCount; ++j)
			{
				Px1DConstraint& row = rows[j];

				if (linkIndex0 != 0x80000000 && linkIndex1 != 0x80000000)
				{
					const bool flip = linkIndex0 > linkIndex1;

					Cm::SpatialVector impulse0(row.linear0, row.angular0);
					Cm::SpatialVector impulse1(row.linear1, row.angular1);

					Cm::SpatialVector deltaV0, deltaV1;

					if (flip)
					{
						getImpulseSelfResponseInv(fixBase, linkIndex1, linkIndex0, Z, impulse1, impulse0,
							deltaV1, deltaV0, jointVelocities);
					}
					else
					{
						getImpulseSelfResponseInv(fixBase, linkIndex0, linkIndex1, Z, impulse0, impulse1,
							deltaV0, deltaV1, jointVelocities);
					}
				}
				else
				{
					if (linkIndex0 == 0x80000000)
					{
						Cm::SpatialVector impulse1(row.linear1, row.angular1);
						getImpulseResponseInv(fixBase, linkIndex1, Z, impulse1, jointVelocities);
					}
					else
					{
						Cm::SpatialVector impulse0(row.linear0, row.angular0);
						getImpulseResponseInv(fixBase, linkIndex0, Z, impulse0, jointVelocities);
					}
				}
			}

			//calculate joint acceleration due to velocity change
			for (PxU32 i = 0; i < totalDofs; ++i)
			{
				jointAccelerations[i] = jointVelocities[i] * invDt;
			}

			//reset spatial inertia
			computeSpatialInertia(mArticulationData);

			PxReal* coeCol = &coefficientMatrix[elementCount * a];

			//this means the joint force calculated by the inverse dynamic
			//will be just influenced by joint acceleration change
			scratchData.jointVelocities = NULL;
			scratchData.externalAccels = NULL;

			//Input
			scratchData.jointAccelerations = jointAccelerations;

			//a column of the coefficient matrix is the joint force
			scratchData.jointForces = coeCol;

			if (fixBase)
			{
				inverseDynamic(mArticulationData, PxVec3(0.f), scratchData, false);
			}
			else
			{
				inverseDynamicFloatingBase(mArticulationData, PxVec3(0.f), scratchData, false);
			}
		
			allocator->free(tData);
			allocator->free(tempMemory);
		}
	}

	void FeatherstoneArticulation::constraintPrep(ArticulationLoopConstraint* lConstraints, 
		const PxU32 nbJoints, PxSolverConstraintPrepDesc& prepDesc,
		PxSolverBody& sBody, PxSolverBodyData& sBodyData, PxSolverConstraintDesc* descs,
		PxConstraintAllocator& allocator)
	{
		const PxReal dt = mArticulationData.getDt();
		const PxReal invDt = 1.f / dt;
		//constraint prep
		for (PxU32 a = 0; a < nbJoints; ++a)
		{
			ArticulationLoopConstraint& lConstraint = lConstraints[a];
			Constraint* aConstraint = lConstraint.constraint;

			PxSolverConstraintDesc& desc = descs[a];
			prepDesc.desc = &desc;
			prepDesc.linBreakForce = aConstraint->linBreakForce;
			prepDesc.angBreakForce = aConstraint->angBreakForce;
			prepDesc.writeback = &mContext->getConstraintWriteBackPool()[aConstraint->index];
			setupConstraintFlags(prepDesc, aConstraint->flags);
			prepDesc.minResponseThreshold = aConstraint->minResponseThreshold;

			Px1DConstraint rows[MAX_CONSTRAINT_ROWS];
			setupConstraintRows(rows, MAX_CONSTRAINT_ROWS);

			prepDesc.invMassScales.linear0 = prepDesc.invMassScales.linear1 = prepDesc.invMassScales.angular0 = prepDesc.invMassScales.angular1 = 1.0f;
			prepDesc.body0WorldOffset = PxVec3(0.0f);

			const PxTransform idt(PxIdentity);
			const PxTransform& body2World0 = aConstraint->body0 ? aConstraint->bodyCore0->body2World : idt;
			const PxTransform& body2World1 = aConstraint->body1 ? aConstraint->bodyCore1->body2World : idt;

			PxVec3p unused_ra, unused_rb;
			PxConstraintInvMassScale unused_invMassScales;

			//TAG:solverprepcall
			prepDesc.numRows = (*aConstraint->solverPrep)(rows,
				prepDesc.body0WorldOffset,
				MAX_CONSTRAINT_ROWS,
				unused_invMassScales,
				aConstraint->constantBlock,
				body2World0, body2World1, !!(aConstraint->flags & PxConstraintFlag::eENABLE_EXTENDED_LIMITS),
				unused_ra, unused_rb);
	
			prepDesc.bodyFrame0 = body2World0;
			prepDesc.bodyFrame1 = body2World1;
			prepDesc.rows = rows;

			const PxU32 linkIndex0 = lConstraint.linkIndex0;
			const PxU32 linkIndex1 = lConstraint.linkIndex1;
		
			if (linkIndex0 != 0x80000000 && linkIndex1 != 0x80000000)
			{
				desc.articulationA = this;
				desc.articulationB = this;
				desc.linkIndexA = PxTo8(linkIndex0);
				desc.linkIndexB = PxTo8(linkIndex1);

				desc.bodyA = reinterpret_cast<PxSolverBody*>(this);
				desc.bodyB = reinterpret_cast<PxSolverBody*>(this);

				prepDesc.bodyState0 = PxSolverConstraintPrepDescBase::eARTICULATION;
				prepDesc.bodyState1 = PxSolverConstraintPrepDescBase::eARTICULATION;
			
			}
			else if (linkIndex0 == 0x80000000)
			{
				desc.articulationA = NULL;
				desc.articulationB = this;

				desc.linkIndexA = PxSolverConstraintDesc::RIGID_BODY;
				desc.linkIndexB = PxTo8(linkIndex1);

				desc.bodyA = &sBody;
				desc.bodyB = reinterpret_cast<PxSolverBody*>(this);

				prepDesc.bodyState0 = PxSolverConstraintPrepDescBase::eSTATIC_BODY;
				prepDesc.bodyState1 = PxSolverConstraintPrepDescBase::eARTICULATION;
			}
			else if (linkIndex1 == 0x80000000)
			{
				desc.articulationA = this;
				desc.articulationB = NULL;

				desc.linkIndexA = PxTo8(linkIndex0);
				desc.linkIndexB = PxSolverConstraintDesc::RIGID_BODY;

				desc.bodyA = reinterpret_cast<PxSolverBody*>(this);
				desc.bodyB = &sBody;

				prepDesc.bodyState0 = PxSolverConstraintPrepDescBase::eARTICULATION;
				prepDesc.bodyState1 = PxSolverConstraintPrepDescBase::eSTATIC_BODY;

			}

			prepDesc.body0 = desc.bodyA;
			prepDesc.body1 = desc.bodyB;
			prepDesc.data0 = &sBodyData;
			prepDesc.data1 = &sBodyData;
		
			ConstraintHelper::setupSolverConstraint(prepDesc, allocator, dt, invDt);
		}

	}

	class BlockBasedAllocator
	{
		struct AllocationPage
		{
			static const PxU32 PageSize = 32 * 1024;
			PxU8 mPage[PageSize];

			PxU32 currentIndex;

			AllocationPage() : currentIndex(0) {}

			PxU8* allocate(const PxU32 size)
			{
				PxU32 alignedSize = (size + 15)&(~15);
				if ((currentIndex + alignedSize) < PageSize)
				{
					PxU8* ret = &mPage[currentIndex];
					currentIndex += alignedSize;
					return ret;
				}
				return NULL;
			}
		};

		AllocationPage* currentPage;

		physx::PxArray<AllocationPage*> mAllocatedBlocks;
		PxU32 mCurrentIndex;

	public:
		BlockBasedAllocator() : currentPage(NULL), mCurrentIndex(0)
		{
		}

		virtual PxU8* allocate(const PxU32 byteSize)
		{
			if (currentPage)
			{
				PxU8* data = currentPage->allocate(byteSize);
				if (data)
					return data;
			}

			if (mCurrentIndex < mAllocatedBlocks.size())
			{
				currentPage = mAllocatedBlocks[mCurrentIndex++];
				currentPage->currentIndex = 0;
				return currentPage->allocate(byteSize);
			}
			currentPage = PX_PLACEMENT_NEW(PX_ALLOC(sizeof(AllocationPage), "AllocationPage"), AllocationPage)();
			mAllocatedBlocks.pushBack(currentPage);
			mCurrentIndex = mAllocatedBlocks.size();

			return currentPage->allocate(byteSize);
		}

		void release() { for (PxU32 a = 0; a < mAllocatedBlocks.size(); ++a) PX_FREE(mAllocatedBlocks[a]); mAllocatedBlocks.clear(); currentPage = NULL; mCurrentIndex = 0; }

		void reset() { currentPage = NULL; mCurrentIndex = 0; }

		virtual ~BlockBasedAllocator()
		{
			release();
		}
	};

	class ArticulationBlockAllocator : public PxConstraintAllocator
	{
		BlockBasedAllocator mConstraintAllocator;
		BlockBasedAllocator mFrictionAllocator[2];

		PxU32 currIdx;

	public:

		ArticulationBlockAllocator() : currIdx(0)
		{
		}

		virtual ~ArticulationBlockAllocator() {}

		virtual PxU8* reserveConstraintData(const PxU32 size)
		{
			return reinterpret_cast<PxU8*>(mConstraintAllocator.allocate(size));
		}

		virtual PxU8* reserveFrictionData(const PxU32 byteSize)
		{
			return reinterpret_cast<PxU8*>(mFrictionAllocator[currIdx].allocate(byteSize));
		}

		void release() { currIdx = 1 - currIdx; mConstraintAllocator.release(); mFrictionAllocator[currIdx].release(); }

		PX_NOCOPY(ArticulationBlockAllocator)

	};

	// forward declarations
	void solveExt1D(const PxSolverConstraintDesc& desc, bool isPositionIteration);
	void writeBack1D(const PxSolverConstraintDesc& desc);
	void conclude1D(const PxSolverConstraintDesc& desc);
	void clearExt1D(const PxSolverConstraintDesc& desc);

	bool FeatherstoneArticulation::getLambda(ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints, 
		PxArticulationCache& cache, PxArticulationCache& initialState,
		const PxReal* jointTorque, const PxVec3& gravity, const PxU32 maxIter, const PxReal invLengthScale)
	{
		const PxReal dt = mArticulationData.getDt();
		const PxReal invDt = 1.f / dt;
		const PxU32 totalDofs = mArticulationData.getDofs();
		
		const PxU32 linkCount = mArticulationData.getLinkCount();
	
		ArticulationBlockAllocator bAlloc;

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		Cm::SpatialVectorF* deltaV = reinterpret_cast<Cm::SpatialVectorF*>(allocator->alloc(sizeof(Cm::SpatialVectorF) * linkCount, true));

		PxReal* previousLambdas =reinterpret_cast<PxReal*>(allocator->alloc(sizeof(PxReal)*nbJoints * 2, true));
		PxReal* lambdas = cache.lambda;

		//this is the joint force changed caused by contact force based on impulse strength is 1
		PxReal* J = cache.coefficientMatrix; 

		PxSolverBody staticSolverBody;
		PxMemZero(&staticSolverBody, sizeof(PxSolverBody));
		PxSolverBodyData staticSolverBodyData;
		PxMemZero(&staticSolverBodyData, sizeof(PxSolverBodyData));
		staticSolverBodyData.maxContactImpulse = PX_MAX_F32;
		staticSolverBodyData.penBiasClamp = -PX_MAX_F32;
		staticSolverBodyData.body2World = PxTransform(PxIdentity);

		PxSolverConstraintDesc* desc = reinterpret_cast<PxSolverConstraintDesc*>(allocator->alloc(sizeof(PxSolverConstraintDesc) * nbJoints, true));
		ArticulationSolverDesc artiDesc;
		
		PxSolverConstraintDesc* constraintDescs = reinterpret_cast<PxSolverConstraintDesc*>(allocator->alloc(sizeof(PxSolverConstraintDesc) * mArticulationData.getLinkCount()-1, true));
		
		//run forward dynamic to calculate the lamba

		artiDesc.articulation = this;
		PxU32 acCount = 0;
		computeUnconstrainedVelocities(artiDesc, dt, acCount, gravity, invLengthScale);

		ScratchData scratchData;
		scratchData.motionVelocities = mArticulationData.getMotionVelocities();
		scratchData.motionAccelerations = mArticulationData.getMotionAccelerations();
		scratchData.coriolisVectors = mArticulationData.getCorioliseVectors();
		scratchData.spatialZAVectors = mArticulationData.getSpatialZAVectors();
		scratchData.jointAccelerations = mArticulationData.getJointAccelerations();
		scratchData.jointVelocities = mArticulationData.getJointVelocities();
		scratchData.jointPositions = mArticulationData.getJointPositions();
		scratchData.jointForces = mArticulationData.getJointForces();
		scratchData.externalAccels = mArticulationData.getExternalAccelerations();

		//prepare constraint data
		PxSolverConstraintPrepDesc prepDesc;
		constraintPrep(lConstraints, nbJoints, prepDesc, staticSolverBody,
			staticSolverBodyData, desc, bAlloc);

		for (PxU32 i = 0; i < nbJoints; ++i)
		{
			previousLambdas[i] = PX_MAX_F32;
		}

		bool found = true;

		for (PxU32 iter = 0; iter < maxIter; ++iter)
		{
			found = true;
			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				clearExt1D(desc[i]);
			}

			//solve
			for (PxU32 itr = 0; itr < 4; itr++)
			{
				for (PxU32 i = 0; i < nbJoints; ++i)
				{
					solveExt1D(desc[i], true); //TODO: Should error accumulation be suppressed?
				}
			}
			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				conclude1D(desc[i]);
			}

			PxcFsFlushVelocity(*this, deltaV);

			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				solveExt1D(desc[i], true); //TODO: Should error accumulation be suppressed?
				writeBack1D(desc[i]);
			}

			PxReal eps = 1e-5f;
			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				Dy::Constraint* constraint = lConstraints->constraint;

				Dy::ConstraintWriteback& solverOutput = mContext->getConstraintWriteBackPool()[constraint->index];
				PxVec3 linearForce = solverOutput.linearImpulse * invDt;

				//linear force is normalize so lambda is the magnitude of linear force
				lambdas[i] = linearForce.magnitude() * dt;

				const PxReal dif = PxAbs(previousLambdas[i] - lambdas[i]);
				if (dif > eps)
					found = false;
		
				previousLambdas[i] = lambdas[i];
			}

			if (found)
				break;

			//joint force
			PxReal* jf3 = cache.jointForce;

			//zero the joint force buffer
			PxMemZero(jf3, sizeof(PxReal)*totalDofs);

			for (PxU32 colInd = 0; colInd < nbJoints; ++colInd)
			{
				PxReal* col = &J[colInd * totalDofs];

				for (PxU32 j = 0; j < totalDofs; ++j)
				{
					jf3[j] += col[j] * lambdas[colInd];
				}
			}

			//jointTorque is M(q)*qddot + C(q,qdot)t - g(q)
			//jointTorque - J*lambda.
			for (PxU32 j = 0; j < totalDofs; ++j)
			{
				jf3[j] = jointTorque[j] - jf3[j];
			}

			bool shouldWake = false;
			//reset all joint velocities/
			applyCache(initialState, PxArticulationCacheFlag::eALL, shouldWake);

			//copy constraint torque to internal data
			applyCache(cache, PxArticulationCacheFlag::eFORCE, shouldWake);

			mArticulationData.init();
			
			computeLinkVelocities(mArticulationData, scratchData);
			computeZ(mArticulationData, gravity, scratchData);
			computeArticulatedSpatialZ(mArticulationData, scratchData);

			{
			//Constant terms.
			const bool doIC = true;
			const PxArticulationFlags articulationFlags = mArticulationData.getArticulationFlags();
			const ArticulationLink* links = mArticulationData.getLinks();
			const ArticulationJointCoreData* jointDatas = mArticulationData.getJointData();
			const Cm::SpatialVectorF* linkSpatialZAExtForces = scratchData.spatialZAVectors;
			const Cm::SpatialVectorF* linkCoriolisForces = scratchData.coriolisVectors;
			const PxVec3* linkRws = mArticulationData.getRw(); 
			const Cm::UnAlignedSpatialVector* jointDofMotionMatrices = mArticulationData.getWorldMotionMatrix();
			const SpatialMatrix& baseInvSpatialArticulatedInertiaW = mArticulationData.getBaseInvSpatialArticulatedInertiaW();

			//Cached constant terms.
			const InvStIs* linkInvStIs = mArticulationData.getInvStIS();
			const Cm::SpatialVectorF* jointDofIsWs = mArticulationData.getIsW();
			const PxReal* jointDofQstZics = mArticulationData.getQstZIc();	

			//Output
			Cm::SpatialVectorF* linkMotionVelocities = scratchData.motionVelocities;
			Cm::SpatialVectorF* linkMotionAccelerations = scratchData.motionAccelerations;
			PxReal* jointAccelerations = scratchData.jointAccelerations;
			PxReal* jointVelocities = scratchData.jointVelocities;
			PxReal* jointNewVelocities = mArticulationData.getJointNewVelocities();

			computeLinkAcceleration(
					doIC, dt, 
					articulationFlags,
					links, linkCount, jointDatas,
					linkSpatialZAExtForces, linkCoriolisForces, linkRws,
					jointDofMotionMatrices, baseInvSpatialArticulatedInertiaW,
					linkInvStIs, jointDofIsWs, jointDofQstZics,
					linkMotionAccelerations, linkMotionVelocities, 
					jointAccelerations, jointVelocities, jointNewVelocities);			
			}

			//zero zero acceleration vector in the articulation data so that we can use this buffer to accumulated
			//impulse for the contacts/constraints in the PGS/TGS solvers
			PxMemZero(mArticulationData.getSpatialZAVectors(), sizeof(Cm::SpatialVectorF) * linkCount);		
		}
		
		allocator->free(constraintDescs);
		allocator->free(previousLambdas);
		allocator->free(deltaV);
		allocator->free(desc);
		bAlloc.release();
		
		bool shouldWake = false;
		//roll back to the current stage
		applyCache(initialState, PxArticulationCacheFlag::eALL, shouldWake);

		return found;

	}

	//i is the current link ID, we need to compute the row/column related to the joint i with all the other joints
	PxU32 computeHi(ArticulationData& data, const PxU32 linkID, PxReal* massMatrix, Cm::SpatialVectorF* f)
	{
		ArticulationLink* links = data.getLinks();

		ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

		const PxU32 totalDofs = data.getDofs();

		//Hii
		for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
		{
			const PxU32 row = (jointDatum.jointOffset + ind)* totalDofs;
			const Cm::SpatialVectorF& tf = f[ind];
			for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
			{
				const PxU32 col = jointDatum.jointOffset + ind2;
				const Cm::UnAlignedSpatialVector& sa = data.getWorldMotionMatrix(jointDatum.jointOffset + ind2);
				massMatrix[row + col] = sa.innerProduct(tf);
			}
		}

		PxU32 j = linkID;

		ArticulationLink* jLink = &links[j];
		while (jLink->parent != 0)
		{
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				//f[ind] = data.getChildToParent(j) * f[ind];
				f[ind] = FeatherstoneArticulation::translateSpatialVector(data.getRw(j), f[ind]);
			}

			//assign j to the parent link
			j = jLink->parent;
			jLink = &links[j];

			//Hij
			ArticulationJointCoreData& pJointDatum = data.getJointData(j);

			for (PxU32 ind = 0; ind < pJointDatum.dof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& sa = data.getWorldMotionMatrix(pJointDatum.jointOffset + ind);
				const PxU32 col = pJointDatum.jointOffset + ind;

				for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
				{
					const PxU32 row = (jointDatum.jointOffset + ind2)* totalDofs;

					Cm::SpatialVectorF& fcol = f[ind2];

					massMatrix[row + col] = sa.innerProduct(fcol);
				}
			}

			//Hji = transpose(Hij)
			{
				for (PxU32 ind = 0; ind < pJointDatum.dof; ++ind)
				{
					const PxU32 pRow = (pJointDatum.jointOffset + ind)* totalDofs;
					const PxU32 col = pJointDatum.jointOffset + ind;

					for (PxU32 ind2 = 0; ind2 < jointDatum.dof; ++ind2)
					{
						const PxU32 pCol = jointDatum.jointOffset + ind2;
						const PxU32 row = (jointDatum.jointOffset + ind2) * totalDofs;

						massMatrix[pRow + pCol] = massMatrix[row + col];
					}
				}
			}

		}
		return j;
	}

	void FeatherstoneArticulation::calculateHFixBase(PxArticulationCache& cache)
	{
		const PxU32 elementCount = mArticulationData.getDofs();

		PxReal* massMatrix = cache.massMatrix;

		PxMemZero(massMatrix, sizeof(PxReal) * elementCount * elementCount);

		const PxU32 linkCount = mArticulationData.getLinkCount();

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ArticulationLink* links = mArticulationData.getLinks();

		const PxU32 startIndex = PxU32(linkCount - 1);

		Dy::SpatialMatrix* compositeSpatialInertia = reinterpret_cast<Dy::SpatialMatrix*>(allocator->alloc(sizeof(Dy::SpatialMatrix) * linkCount));

		//initialize composite spatial inertial
		initCompositeSpatialInertia(mArticulationData, compositeSpatialInertia);

		Cm::SpatialVectorF F[6];
		for (PxU32 i = startIndex; i > 0; --i)
		{
			ArticulationLink& link = links[i];

			Dy::SpatialMatrix cSpatialInertia = compositeSpatialInertia[i];
			//transform current link's spatial inertia to parent's space
			PxVec3 rw = link.bodyCore->body2World.p - links[link.parent].bodyCore->body2World.p;
			FeatherstoneArticulation::translateInertia(FeatherstoneArticulation::constructSkewSymmetricMatrix(rw), cSpatialInertia);

			//compute parent's composite spatial inertia
			compositeSpatialInertia[link.parent] += cSpatialInertia;

			Dy::SpatialMatrix& tSpatialInertia = compositeSpatialInertia[i];

			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(i);

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::UnAlignedSpatialVector& sa = mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind];
				Cm::UnAlignedSpatialVector tmp = tSpatialInertia* sa;
				F[ind].top = tmp.top;
				F[ind].bottom = tmp.bottom;
			}

			//Hii, Hij, Hji
			computeHi(mArticulationData, i, massMatrix, F);
		}

		allocator->free(compositeSpatialInertia);
	}


	void FeatherstoneArticulation::calculateHFloatingBase(PxArticulationCache& cache)
	{
		const PxU32 elementCount = mArticulationData.getDofs();

		PxReal* massMatrix = cache.massMatrix;

		PxMemZero(massMatrix, sizeof(PxReal) * elementCount * elementCount);

		const PxU32 linkCount = mArticulationData.getLinkCount();

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ArticulationLink* links = mArticulationData.getLinks();
		//ArticulationLinkData* linkData = mArticulationData.getLinkData();

		const PxU32 startIndex = PxU32(linkCount - 1);

		Dy::SpatialMatrix* compositeSpatialInertia = reinterpret_cast<Dy::SpatialMatrix*>(allocator->alloc(sizeof(Dy::SpatialMatrix) * linkCount));
		Cm::SpatialVectorF* F = reinterpret_cast<Cm::SpatialVectorF*>(allocator->alloc(sizeof(Cm::SpatialVectorF) * elementCount));
		
		//initialize composite spatial inertial
		initCompositeSpatialInertia(mArticulationData, compositeSpatialInertia);

		for (PxU32 i = startIndex; i > 0; --i)
		{
			ArticulationLink& link = links[i];

			Dy::SpatialMatrix cSpatialInertia = compositeSpatialInertia[i];
			//transform current link's spatial inertia to parent's space
			PxVec3 rw = link.bodyCore->body2World.p - links[link.parent].bodyCore->body2World.p;
			FeatherstoneArticulation::translateInertia(FeatherstoneArticulation::constructSkewSymmetricMatrix(rw), cSpatialInertia);

			//compute parent's composite spatial inertia
			compositeSpatialInertia[link.parent] += cSpatialInertia;

			Dy::SpatialMatrix& tSpatialInertia = compositeSpatialInertia[i];

			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(i);

			Cm::SpatialVectorF* f = &F[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				Cm::UnAlignedSpatialVector& sa = mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind];
				Cm::UnAlignedSpatialVector tmp = tSpatialInertia* sa;
				f[ind].top = tmp.top;
				f[ind].bottom = tmp.bottom;
			}

			//Hii, Hij, Hji 
			const PxU32 j = computeHi(mArticulationData, i, massMatrix, f);

			//transform F to the base link space
			//ArticulationLinkData& fDatum = linkData[j];
			
			PxVec3 brw = links[j].bodyCore->body2World.p - links[0].bodyCore->body2World.p;
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				f[ind] = translateSpatialVector(brw, f[ind]);
			}
		}

		//Ib = base link composite inertia tensor
		//compute transpose(F) * inv(Ib) *F
		Dy::SpatialMatrix invI0 = compositeSpatialInertia[0].invertInertia();

		//H - transpose(F) * inv(Ib) * F;
		for (PxU32 row = 0; row < elementCount; ++row)
		{
			const Cm::SpatialVectorF& f = F[row];
			for (PxU32 col = 0; col < elementCount; ++col)
			{
				const Cm::SpatialVectorF invIf = invI0 * F[col];
				const PxReal v = f.innerProduct(invIf);
				const PxU32 index = row * elementCount + col;
				massMatrix[index] = massMatrix[index] - v;
			}
		}

		allocator->free(compositeSpatialInertia);
		allocator->free(F);
	
	}

	//calculate a single column of H, jointForce is equal to a single column of H
	void FeatherstoneArticulation::calculateMassMatrixColInv(ScratchData& scratchData)
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();

		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		
		//Input
		PxReal* jointAccelerations = scratchData.jointAccelerations;

		//set base link motion acceleration to be zero because H should
		//be just affected by joint position/link position
		motionAccelerations[0] = Cm::SpatialVectorF::Zero();
		spatialZAForces[0] = Cm::SpatialVectorF::Zero();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = mArticulationData.getLink(linkID);
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			//parent motion accelerations into child space
			Cm::SpatialVectorF accel = translateSpatialVector(-mArticulationData.getRw(linkID), motionAccelerations[link.parent]);

			const PxReal* jAcceleration = &jointAccelerations[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				accel.top += mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind].top * jAcceleration[ind];
				accel.bottom += mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset + ind].bottom * jAcceleration[ind];
			}

			motionAccelerations[linkID] = accel;

			spatialZAForces[linkID] = mArticulationData.mWorldSpatialArticulatedInertia[linkID] * accel;
		}

		computeGeneralizedForceInv(mArticulationData, scratchData);

	}

	void FeatherstoneArticulation::getGeneralizedMassMatrixCRB(PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "ArticulationHelper::getGeneralizedMassMatrix() commonInit need to be called first to initialize data!");
			return;
		}

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		if (fixBase)
		{
			calculateHFixBase(cache);
		}
		else
		{
			calculateHFloatingBase(cache);
		}

	}

	void FeatherstoneArticulation::getGeneralizedMassMatrix( PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "ArticulationHelper::getGeneralizedMassMatrix() commonInit need to be called first to initialize data!");
			return;
		}


		//calculate each column for mass matrix
		PxReal* massMatrix = cache.massMatrix;
		
		const PxU32 linkCount = mArticulationData.getLinkCount();

		const PxU32 elementCount = mArticulationData.getDofs();

		const PxU32 size = sizeof(PxReal) * elementCount;
		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		PxReal* jointAccelerations = reinterpret_cast<PxReal*>(allocator->alloc(size));

		scratchData.jointAccelerations = jointAccelerations;
		scratchData.jointVelocities = NULL;
		scratchData.externalAccels = NULL;

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		
		//initialize jointAcceleration to be zero
		PxMemZero(jointAccelerations, size);

		for (PxU32 colInd = 0; colInd < elementCount; ++colInd)
		{
			PxReal* col = &massMatrix[colInd * elementCount];

			scratchData.jointForces = col;

			//set joint acceleration 1 in the col + 1 and zero elsewhere
			jointAccelerations[colInd] = 1;

			if (fixBase)
			{
				//jointAcceleration is Q, HQ = ID(model, qdot, Q).
				calculateMassMatrixColInv(scratchData);
			}
			else
			{
				inverseDynamicFloatingBase(mArticulationData, PxVec3(0.f), scratchData, false);
			}

			//reset joint acceleration to be zero
			jointAccelerations[colInd] = 0;
		}

		allocator->free(jointAccelerations);
		allocator->free(tempMemory);
	}



} //namespace Dy

}
