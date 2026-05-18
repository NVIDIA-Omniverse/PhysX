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

#include "DyFeatherstoneArticulation.h"
#include "CmPinnableArray.h"
#include "DyDynamics.h"
#include "DyConstraintPrep.h"

namespace physx
{

namespace Dy
{
	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV);

	void FeatherstoneArticulation::computeLinkAccelerationInv(const ArticulationData& data, ScratchData& scratchData)
	{
		Cm::SpatialVectorF* PX_RESTRICT motionAccelerations = scratchData.motionAccelerations;
		
		Cm::SpatialVectorF* PX_RESTRICT coriolisVectors = scratchData.coriolisVectors;

		const PxReal* PX_RESTRICT jointAccelerations = scratchData.jointAccelerations;

		const PxVec3* PX_RESTRICT rw = data.getRw();

		motionAccelerations[0] = Cm::SpatialVectorF::Zero();

		for (PxU32 linkID = 1; linkID < data.getLinkCount(); ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);

			Cm::SpatialVectorF pMotionAcceleration = translateSpatialVector(-rw[linkID], motionAccelerations[link.parent]);

			Cm::SpatialVectorF motionAcceleration(PxVec3(0.f), PxVec3(0.f));

			if (jointAccelerations)
			{
				const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();

				const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				const PxU32 jointOffset = jointDatum.jointOffset;
				const PxReal* jAcceleration = &jointAccelerations[jointOffset];
				for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
				{
					motionAcceleration.top += worldMotionMatrix[jointOffset + ind].top * jAcceleration[ind];
					motionAcceleration.bottom += worldMotionMatrix[jointOffset + ind].bottom * jAcceleration[ind];
				}
			}

			//calculate the acceleration due to external forces, applied external acceleration, and Coriolis force
			//used later to calculate the zero acceleration force
			motionAccelerations[linkID] = pMotionAcceleration + coriolisVectors[linkID] + motionAcceleration;
		}
	}

	//generalized force
	void FeatherstoneArticulation::computeGeneralizedForceInv(const ArticulationData& data, ScratchData& scratchData)
	{
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* PX_RESTRICT spatialZAForces = scratchData.spatialZAVectors;
		PxReal* PX_RESTRICT jointForces = scratchData.jointForces;
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = (linkCount - 1); linkID > 0; --linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);
			
			//joint force

			spatialZAForces[link.parent] += translateSpatialVector(rw[linkID], spatialZAForces[linkID]);

			const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			//compute generalized force
			const PxU32 jointOffset = jointDatum.jointOffset;
			PxReal* force = &jointForces[jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				force[ind] = worldMotionMatrix[jointOffset + ind].innerProduct(spatialZAForces[linkID]);
			}
		}
	}

	void FeatherstoneArticulation::computeZAForceInv(const ArticulationData& data, const Cm::SpatialVectorF* PX_RESTRICT motionAccelerations, Cm::SpatialVectorF* PX_RESTRICT biasForce)
	{
		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);

			const PxsBodyCore& core = *link.bodyCore;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;
			const PxVec3 inertiaTensor = Cm::safeRecip<PxVec3>(core.inverseInertia);

			Cm::SpatialVectorF Ia;
			Ia.bottom = core.body2World.rotate(core.body2World.rotateInv(motionAccelerations[linkID].top).multiply(inertiaTensor));
			Ia.top = motionAccelerations[linkID].bottom * m;

			biasForce[linkID] += Ia;
		}
	}

	void FeatherstoneArticulation::initCompositeSpatialInertia(const ArticulationData& data, Dy::SpatialMatrix* compositeSpatialInertia)
	{
		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			SpatialMatrix& spatialInertia = compositeSpatialInertia[linkID];

			const ArticulationLink& link = data.getLink(linkID);

			PxsBodyCore& core = *link.bodyCore;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

			//construct mass matric
			spatialInertia.topLeft = PxMat33(PxZero);
			spatialInertia.topRight = PxMat33::createDiagonal(PxVec3(m));

			//construct inertia matrix
			const PxMat33 rot(core.body2World.q);
			PxMat33& I = spatialInertia.bottomLeft;
			const PxVec3 inertiaTensor = Cm::safeRecip<PxVec3>(core.inverseInertia);
			Cm::transformInertiaTensor(inertiaTensor, rot, I);
		}
	}

	void FeatherstoneArticulation::computeCompositeSpatialInertiaAndZAForceInv(const ArticulationData& data, ScratchData& scratchData)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();
		const PxU32 startIndex = PxU32(linkCount - 1);

		Dy::SpatialMatrix* PX_RESTRICT compositeSpatialInertia = scratchData.compositeSpatialInertias;
		Cm::SpatialVectorF* PX_RESTRICT zaForce = scratchData.spatialZAVectors;
		const PxVec3* PX_RESTRICT rw = data.getRw();

		initCompositeSpatialInertia(data, compositeSpatialInertia);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			const ArticulationLink& link = links[linkID];

			Dy::SpatialMatrix cSpatialInertia = compositeSpatialInertia[linkID];
			translateInertia(FeatherstoneArticulation::constructSkewSymmetricMatrix(rw[linkID]), cSpatialInertia);
			
			//compute parent's composite spatial inertia
			compositeSpatialInertia[link.parent] += cSpatialInertia;

			//compute zero acceleration force. This is the force that would be required to support the
			//motion of all the bodies in children set if root node acceleration happened to be zero
			zaForce[link.parent] += translateSpatialVector(rw[linkID], zaForce[linkID]);
		}
	}

	void FeatherstoneArticulation::computeRelativeGeneralizedForceInv(const ArticulationData& data, ScratchData& scratchData, const bool coriolisFloatingBase)
	{
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;
		Dy::SpatialMatrix* compositeSpatialInertia = scratchData.compositeSpatialInertias;
		Cm::SpatialVectorF* zaForce = scratchData.spatialZAVectors;
		PxReal* jointForces = scratchData.jointForces;

		if (!coriolisFloatingBase)
		{
			Dy::SpatialMatrix invInertia = compositeSpatialInertia[0].invertInertia();
			//obtained by solving the equation: F = I * a + Z for the root where F = 0
			motionAccelerations[0] = -(invInertia * zaForce[0]);
		}

		const PxU32 linkCount = data.getLinkCount();
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];

			//transform the acceleration into the child link frame
			motionAccelerations[linkID] = translateSpatialVector(-rw[linkID], motionAccelerations[link.parent]);

			// equation: F = I * a + Z
			zaForce[linkID] = compositeSpatialInertia[linkID] * motionAccelerations[linkID] + zaForce[linkID];
			const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			//compute generalized force
			const PxU32 jointOffset = jointDatum.jointOffset;
			PxReal* jForce = &jointForces[jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				jForce[ind] = worldMotionMatrix[jointOffset+ind].innerProduct(zaForce[linkID]);
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

		computeZAForceInv(data, scratchData.motionAccelerations, scratchData.spatialZAVectors);

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

		computeZAForceInv(data, scratchData.motionAccelerations, scratchData.spatialZAVectors);

		//pass 2
		computeCompositeSpatialInertiaAndZAForceInv(data, scratchData);

		//pass 3
		computeRelativeGeneralizedForceInv(data, scratchData, computeCoriolis);
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
			const ArticulationLink& rLink = mArticulationData.getLink(0);
			// PT:: tag: scalar transform*transform
			rLink.bodyCore->body2World = cache.rootLinkData->transform * rLink.bodyCore->getBody2Actor();
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_ROOT_TRANSFORM;
		}

		if(flag & PxArticulationCacheFlag::eROOT_VELOCITIES)
		{
			const ArticulationLink& rLink = mArticulationData.getLink(0);
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

		if (flag & PxArticulationCacheFlag::eLINK_FORCE)
		{
			const PxU32 linkCount = data.getLinkCount();
			for (PxU32 i = 0; i < linkCount; ++i)
			{
				const PxVec3 linkForce = cache.linkForce[i];
				localShouldWake = localShouldWake || linkForce != PxVec3(0.0f);
				const ArticulationLink& link = mArticulationData.getLink(i);

				PxsBodyCore& core = *link.bodyCore;
				data.mExternalAcceleration[i].linear = linkForce * core.inverseMass;
			}
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_EXT_ACCEL;
		}

		if (flag & PxArticulationCacheFlag::eLINK_TORQUE)
		{
			const PxU32 linkCount = data.getLinkCount();
			for (PxU32 i = 0; i < linkCount; ++i)
			{
				const ArticulationLink& link = mArticulationData.getLink(i);
				PxsBodyCore& core = *link.bodyCore;

				const PxQuat& q = core.body2World.q;
				const PxVec3& linkTorque = cache.linkTorque[i];
				const PxVec3 localLinkTorque = q.rotateInv(linkTorque);
				localShouldWake = localShouldWake || localLinkTorque != PxVec3(0.0f);

				// turn localLinkTorque into acceleration in local frame
				const PxVec3 localAccel = core.inverseInertia.multiply(localLinkTorque);
				// turn localAccel into world space
				const PxVec3 worldAccel = q.rotate(localAccel);

				data.mExternalAcceleration[i].angular = worldAccel;
			}
			mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_EXT_ACCEL;
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
			const ArticulationLink& linkDatum = mArticulationData.getLink(linkID);
			const ArticulationJointCore* joint = linkDatum.inboundJoint;
			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

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

			PX_ASSERT(count == jointDatum.nbDof);
		}

	}

	void FeatherstoneArticulation::unpackJointData(const PxReal* reduced, PxReal* maximum)
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
			const ArticulationLink& linkDatum = mArticulationData.getLink(linkID);
			const ArticulationJointCore* joint = linkDatum.inboundJoint;
			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

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

			PX_ASSERT(count == jointDatum.nbDof);
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
		const PxU32 rootDof = fixBase ? 0 : 6; //add the DoF of the root in the floating base case

		//note that with the new API, for both fixed-base and floating-base, we consider no acceleration for all joints and all links
		//this is consistent with the assumption behind the equation of motion
		Cm::SpatialVectorF* spatialZAForces = reinterpret_cast<Cm::SpatialVectorF*>(allocator->alloc(sizeof(Cm::SpatialVectorF) * linkCount));

		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = mArticulationData.getLink(linkID);

			PxsBodyCore& core = *link.bodyCore;

			const PxReal m = 1.0f / core.inverseMass;

			const PxVec3 linkGravity = tGravity;

			spatialZAForces[linkID].top = m * linkGravity;
			spatialZAForces[linkID].bottom = PxVec3(0.0f);
		}

		ScratchData scratchData;
		scratchData.spatialZAVectors = spatialZAForces;

		scratchData.jointForces = &cache.gravityCompensationForce[rootDof];

		computeGeneralizedForceInv(mArticulationData, scratchData);

		//add the gravity compensation for the root, force first
		if (rootDof == 6)
		{
			Cm::SpatialVectorF* zaForce = scratchData.spatialZAVectors;
			cache.gravityCompensationForce[0] = zaForce[0].top.x;
			cache.gravityCompensationForce[1] = zaForce[0].top.y;
			cache.gravityCompensationForce[2] = zaForce[0].top.z;
			cache.gravityCompensationForce[3] = zaForce[0].bottom.x;
			cache.gravityCompensationForce[4] = zaForce[0].bottom.y;
			cache.gravityCompensationForce[5] = zaForce[0].bottom.z;
		}

		//release spatialZA vectors
		allocator->free(spatialZAForces);

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
	void FeatherstoneArticulation::getCoriolisAndCentrifugalForce(PxArticulationCache& cache)
	{
		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Articulation::getCoriolisAndCentrifugalForce() commonInit need to be called first to initialize data!");
			return;
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		const PxU32 rootDof = fixBase ? 0 : 6;

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		scratchData.jointVelocities = cache.jointVelocity;
		scratchData.jointAccelerations = NULL;
		scratchData.externalAccels = NULL;
		scratchData.jointForces = &cache.coriolisForce[rootDof];
		
		if (fixBase)
			inverseDynamic(mArticulationData, PxVec3(0.0f), scratchData, true);
		else
			inverseDynamicFloatingBase(mArticulationData, PxVec3(0.0f), scratchData, true);

		if(!fixBase)
		{
			//add the Coriolis compensation for the root, force first
			Cm::SpatialVectorF* zaForce = scratchData.spatialZAVectors;
			cache.coriolisForce[0] = zaForce[0].top.x;
			cache.coriolisForce[1] = zaForce[0].top.y;
			cache.coriolisForce[2] = zaForce[0].top.z;
			cache.coriolisForce[3] = zaForce[0].bottom.x;
			cache.coriolisForce[4] = zaForce[0].bottom.y;
			cache.coriolisForce[5] = zaForce[0].bottom.z;
		}

		allocator->free(tempMemory);
	}

	//gravity, joint acceleration and joint velocity are zero
	void FeatherstoneArticulation::getGeneralizedExternalForce(PxArticulationCache& cache)
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
			const ArticulationLink& link = mArticulationData.getLink(i);
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

	void FeatherstoneArticulation::getImpulseResponseSlowInv(const Dy::ArticulationLink* links,
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

		const PxVec3* PX_RESTRICT rw = data.getRw();

		//for (i0 = linkID0; i0 != common; i0 = links[i0].parent)
		for (i0 = 0; linkID0 != common; linkID0 = links[linkID0].parent)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(linkID0).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(linkID0).nbDof;
			Z0 = propagateImpulseW(
					rw[linkID0],
					Z0, 
					NULL, &data.getWorldIsInvD(jointOffset), &data.getWorldMotionMatrix(jointOffset), dofCount,
					NULL);
			Z[links[linkID0].parent] = Z0;
			stack[i0++] = linkID0;
		}

		for (i1 = i0; linkID1 != common; linkID1 = links[linkID1].parent)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(linkID1).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(linkID1).nbDof;
			Z1 = propagateImpulseW(
					rw[linkID1],
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
			const PxU8 dofCount = mArticulationData.getJointData(common).nbDof;
			Z[links[common].parent] = propagateImpulseW(
				rw[common],
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

		const SpatialMatrix* PX_RESTRICT worldSpatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();
		const InvStIs* PX_RESTRICT invStIS = data.getInvStIS();

		for (PxU32 index = ic; (index--) > i1;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = mArticulationData.getJointData(id).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(id).nbDof;
			v = FeatherstoneArticulation::propagateVelocityW(
				rw[id], v, 
				worldSpatialArticulatedInertia[id], Z[id],
				NULL, invStIS[id], &data.getWorldMotionMatrix(jointOffset), dofCount,
				jointVelocities);
		}

		//Replace everything to here with the impulse response matrix multiply

		Cm::SpatialVectorF dv1 = v;
		for (PxU32 index = i1; (index--) > i0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = mArticulationData.getJointData(id).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(id).nbDof;
			dv1 = FeatherstoneArticulation::propagateVelocityW(
				rw[id], v, 
				worldSpatialArticulatedInertia[id], Z[id], 
				NULL, invStIS[id], &data.getWorldMotionMatrix(jointOffset), dofCount,
				jointVelocities);
		}

		Cm::SpatialVectorF dv0 = v;
		for (PxU32 index = i0; (index--) > 0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = mArticulationData.getJointData(id).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(id).nbDof;
			dv0 = FeatherstoneArticulation::propagateVelocityW(
				rw[id], v, 
				worldSpatialArticulatedInertia[id], Z[id], 
				NULL, invStIS[id], &data.getWorldMotionMatrix(jointOffset), dofCount,
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

		//transform p(impulse) from world space to the local space of link
		ArticulationLink& link = links[linkID1];

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
			const PxU8 dofCount = mArticulationData.getJointData(linkID1).nbDof;

			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
			const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = mArticulationData.getISInvStIS();
			const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

			//initialize child link spatial zero acceleration impulse
			Cm::SpatialVectorF Z1(-imp1.linear, -imp1.angular);
			//this calculate parent link spatial zero acceleration impulse
			Cm::SpatialVectorF Z0 = propagateImpulseW(
				rw[linkID1],
				Z1,
				NULL, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount,
				NULL);

			//in parent space
			const Cm::SpatialVectorF impulseDif = pImpulse - Z0;

			Cm::SpatialVectorF delV0(PxVec3(0.f), PxVec3(0.f));
			Cm::SpatialVectorF delV1(PxVec3(0.f), PxVec3(0.f));

			//calculate velocity change start from the parent link to the root
			delV0 = FeatherstoneArticulation::getImpulseResponseWithJ(linkID0, fixBase, mArticulationData, Z, impulseDif, jointVelocities);

			const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();

			//calculate velocity change for child link
			delV1 = FeatherstoneArticulation::propagateVelocityW(
				rw[linkID1], delV0, 
				mArticulationData.getWorldSpatialArticulatedInertia()[linkID1], Z1, 
				NULL, invStIS[linkID1], &worldMotionMatrix[jointOffset], dofCount,
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
		//p(impulse) =	|n|
		//				|0|

		ArticulationLink* links = mArticulationData.getLinks();
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();
		const PxU32 linkCount = mArticulationData.getLinkCount();

		//make sure all links' spatial zero acceleration impulse are zero
		PxMemZero(Z, sizeof(Cm::SpatialVectorF) * linkCount);

		Z[linkID] = Cm::SpatialVectorF(-impulse.linear, -impulse.angular);

		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
		const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = mArticulationData.getISInvStIS();
		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			ArticulationLink& tLink = links[i];
			const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
			const PxU8 dofCount = mArticulationData.getJointData(i).nbDof;
			Z[tLink.parent] = propagateImpulseW(
				rw[i],
				Z[i], 
				NULL, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount,
				NULL);
		}

		//set velocity change of the root link to be zero
		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		if (!fixBase)
			deltaV = mArticulationData.mBaseInvSpatialArticulatedInertiaW * (-Z[0]);

		const PxU32 startIndex = links[linkID].mPathToRootStartIndex;
		const PxU32 numElems = links[linkID].mPathToRootCount;

		const PxU32* pathToRoot = &mArticulationData.mPathToRootElements[startIndex];

		const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();

		for(PxU32 i = 0; i < numElems; ++i)
		{
			const PxU32 index = pathToRoot[i];
			PX_ASSERT(links[index].parent < index);

			ArticulationJointCoreData& tJointDatum = jointData[index];

			PxReal* jVelocity = &jointVelocities[tJointDatum.jointOffset];
			deltaV = propagateVelocityW(
				rw[index], deltaV, 
				mArticulationData.getWorldSpatialArticulatedInertia()[index], Z[index], 
				NULL, invStIS[index], &worldMotionMatrix[tJointDatum.jointOffset], tJointDatum.nbDof,
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

	void FeatherstoneArticulation::constraintPrep(Cm::PinnableArray<Dy::ConstraintWriteback>& writeBack, ArticulationLoopConstraint* lConstraints, 
		const PxU32 nbJoints, PxSolverConstraintPrepDesc& prepDesc,
		PxSolverBody& sBody, PxSolverBodyData& sBodyData, PxSolverConstraintDesc* descs,
		PxConstraintAllocator& allocator, const PxReal biasCoefficient)
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
			prepDesc.writeback = &writeBack[aConstraint->index];
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
		
			ConstraintHelper::setupSolverConstraint(prepDesc, allocator, dt, invDt, biasCoefficient);
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
	void solveExt1D(const PxSolverConstraintDesc& desc);
	void writeBack1D(const PxSolverConstraintDesc& desc);
	void conclude1D(const PxSolverConstraintDesc& desc);
	void clearExt1D(const PxSolverConstraintDesc& desc);

	bool FeatherstoneArticulation::getLambda_Deprecated(Cm::PinnableArray<Dy::ConstraintWriteback>& writeBack, ArticulationLoopConstraint* lConstraints, const PxU32 nbJoints, 
		PxArticulationCache& cache, PxArticulationCache& initialState,
		const PxReal* jointTorque, const PxVec3& gravity, const PxU32 maxIter, const PxReal invLengthScale)
	{
		const PxReal dt = mArticulationData.getDt();
		const PxReal invDt = 1.f / dt;
		const PxReal biasCoefficient = 1.0f;
		const PxU32 totalDofs = mArticulationData.getDofs();
		
		const PxU32 linkCount = mArticulationData.getLinkCount();
	
		ArticulationBlockAllocator bAlloc;

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		Cm::SpatialVectorF* deltaV = reinterpret_cast<Cm::SpatialVectorF*>(allocator->alloc(sizeof(Cm::SpatialVectorF) * linkCount, true));

		PxReal* previousLambdas = reinterpret_cast<PxReal*>(allocator->alloc(sizeof(PxReal)*nbJoints * 2, true));
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
		
		PxSolverConstraintDesc* constraintDescs = reinterpret_cast<PxSolverConstraintDesc*>(allocator->alloc(sizeof(PxSolverConstraintDesc) * mArticulationData.getLinkCount()-1, true));
		
		//run forward dynamic to calculate the lamba

		computeUnconstrainedVelocities(this, dt, gravity, invLengthScale, false, false);

		ScratchData scratchData;
		scratchData.motionVelocities = mArticulationData.getMotionVelocities();
		scratchData.motionAccelerations = mArticulationData.getMotionAccelerations();
		scratchData.coriolisVectors = mArticulationData.getCoriolisVectors();
		scratchData.spatialZAVectors = mArticulationData.getSpatialZAVectors();
		scratchData.jointAccelerations = mArticulationData.getJointAccelerations();
		scratchData.jointVelocities = mArticulationData.getJointVelocities();
		scratchData.jointForces = mArticulationData.getJointForces();
		scratchData.externalAccels = mArticulationData.getExternalAccelerations();

		//prepare constraint data
		PxSolverConstraintPrepDesc prepDesc;
		constraintPrep(writeBack, lConstraints, nbJoints, prepDesc, staticSolverBody,
			staticSolverBodyData, desc, bAlloc, biasCoefficient);

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
					solveExt1D(desc[i]);
				}
			}
			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				conclude1D(desc[i]);
			}

			PxcFsFlushVelocity(*this, deltaV);

			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				solveExt1D(desc[i]);
				writeBack1D(desc[i]);
			}

			PxReal eps = 1e-5f;
			for (PxU32 i = 0; i < nbJoints; ++i)
			{
				Dy::Constraint* constraint = lConstraints->constraint;

				const Dy::ConstraintWriteback& solverOutput = writeBack[constraint->index];
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
	PxU32 computeHi(ArticulationData& data, const PxU32 linkID, PxReal* massMatrix, Cm::SpatialVectorF* f, const PxU32 rootDof = 0)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();

		const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

		const PxU32 totalDofs = data.getDofs();
		const PxU32 matSize = totalDofs + rootDof;

		//Hii
		for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
		{
			const PxU32 row = jointDatum.jointOffset + ind + rootDof;
			const Cm::SpatialVectorF& tf = f[ind];
			for (PxU32 ind2 = 0; ind2 < jointDatum.nbDof; ++ind2)
			{
				const PxU32 col = jointDatum.jointOffset + ind2 + rootDof;
				const Cm::UnAlignedSpatialVector& sa = data.getWorldMotionMatrix(jointDatum.jointOffset + ind2);
				massMatrix[row * matSize + col] = sa.innerProduct(tf);
			}
		}

		PxU32 j = linkID;

		const PxVec3* PX_RESTRICT rw = data.getRw();

		const ArticulationLink* jLink = &links[j];
		while (jLink->parent != 0)
		{
			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				//f[ind] = data.getChildToParent(j) * f[ind];
				f[ind] = FeatherstoneArticulation::translateSpatialVector(rw[j], f[ind]);
			}

			//assign j to the parent link
			j = jLink->parent;
			jLink = &links[j];

			//Hij and Hji
			ArticulationJointCoreData& pJointDatum = data.getJointData(j);

			for (PxU32 ind = 0; ind < pJointDatum.nbDof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& sa = data.getWorldMotionMatrix(pJointDatum.jointOffset + ind);
				const PxU32 col = pJointDatum.jointOffset + ind + rootDof;

				for (PxU32 ind2 = 0; ind2 < jointDatum.nbDof; ++ind2)
				{
					const PxU32 row = jointDatum.jointOffset + ind2 + rootDof;

					Cm::SpatialVectorF& fcol = f[ind2];

					PxReal m = sa.innerProduct(fcol);
					massMatrix[row * matSize + col] = m;
					massMatrix[row + col * matSize] = m;
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

		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();

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

			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(i);

			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& sa = worldMotionMatrix[jointDatum.jointOffset + ind];
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

		PxMemZero(massMatrix, sizeof(PxReal) * (elementCount + 6) * (elementCount + 6));

		const PxU32 linkCount = mArticulationData.getLinkCount();
		const PxU32 dofCount = mArticulationData.getDofs();
		const PxU32 matSize = dofCount + 6; // Add the DoF of the root

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ArticulationLink* links = mArticulationData.getLinks();

		const PxU32 startIndex = PxU32(linkCount - 1);

		Dy::SpatialMatrix* compositeSpatialInertia = reinterpret_cast<Dy::SpatialMatrix*>(allocator->alloc(sizeof(Dy::SpatialMatrix) * linkCount));
		Cm::SpatialVectorF* F = reinterpret_cast<Cm::SpatialVectorF*>(allocator->alloc(sizeof(Cm::SpatialVectorF) * elementCount));

		// initialize composite spatial inertial
		initCompositeSpatialInertia(mArticulationData, compositeSpatialInertia);

		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();

		for(PxU32 i = startIndex; i > 0; --i)
		{
			ArticulationLink& link = links[i];

			Dy::SpatialMatrix cSpatialInertia = compositeSpatialInertia[i];
			// transform current link's spatial inertia to parent's space
			PxVec3 rw = link.bodyCore->body2World.p - links[link.parent].bodyCore->body2World.p;
			FeatherstoneArticulation::translateInertia(FeatherstoneArticulation::constructSkewSymmetricMatrix(rw), cSpatialInertia);

			// compute parent's composite spatial inertia
			compositeSpatialInertia[link.parent] += cSpatialInertia;

			Dy::SpatialMatrix& tSpatialInertia = compositeSpatialInertia[i];

			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(i);

			Cm::SpatialVectorF* f = &F[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& sa = worldMotionMatrix[jointDatum.jointOffset + ind];
				Cm::UnAlignedSpatialVector tmp = tSpatialInertia * sa;
				f[ind].top = tmp.top;
				f[ind].bottom = tmp.bottom;
			}

			// Hii, Hij, Hji
			const PxU32 j = computeHi(mArticulationData, i, massMatrix, f, 6);

			// transform F to the base link space

			PxVec3 brw = links[j].bodyCore->body2World.p - links[0].bodyCore->body2World.p;
			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				f[ind] = translateSpatialVector(brw, f[ind]);
			}

			// add the resulting force on the root
			for(PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				const PxU32 col = jointDatum.jointOffset + ind + 6;
				const Cm::SpatialVectorF& tf = f[ind];
				for(PxU32 row = 0; row < 6; ++row)
				{
					massMatrix[col * matSize + row] = tf[row];
					massMatrix[col + row * matSize] = tf[row];
				}
			}
		}

		// adding the spatial articulated inertia of the root
		// note that the spatial articulated inertia assumes that the root angular acceleration comes first,
		// while the mass matrix assumes that the root linear acceleration comes first
		// we have therefore to invert the angular and linear component of the spatial articulated inertia
		// this also ensures that the mass matrix is symmetric
		const PxReal* rootSpatialInertia = reinterpret_cast<PxReal*>(&compositeSpatialInertia[0]);
		for(PxU32 row = 0; row < 6; ++row)
		{
			const PxU32 rowSpatialInertia = (row < 3) ? row : row - 3; // Convert to the index of a 3 x 3 matrix
			// Only process elements above the diagonal as the matrix is symmetric
			for(PxU32 col = row; col < 6; ++col)
			{
				// This offset is due to how the spatial matrix is indexed and how the linear amd angular components
				// of the acceleration should be inverted, the index is as follows
				//	0	3	6	9	12	15								9	12	15	0	3	6
				//	1	4	7	10	13	16								10	23	16	1	4	7
				//	2	5	8	11	14	17	inversion linear/angular	11	24	17	2	5	8
				//	18	21	24	0	1	2	------------------------>	0	1	2	18	21	24
				//	19	22	25	3	4	5								3	4	5	19	22	25
				//	20	23	26	6	7	8								6	7	8	20	23	26
				const PxU32 offset = (row > 2) ? 18 : (col < 3) * 9;
				const PxU32 colSpatialInertia = (col < 3) ? col : col - 3; // Convert to the index of a 3 x 3 matrix
				const PxU32 index = offset + colSpatialInertia * 3 + rowSpatialInertia;
				massMatrix[row * matSize + col] = rootSpatialInertia[index];
				massMatrix[col * matSize + row] = rootSpatialInertia[index];
			}
		}

		allocator->free(compositeSpatialInertia);
		allocator->free(F);
	}

	//calculate a single column of the mass matrix, assuming the presence of a joint acceleration and the absence of:
	//Coriolis effects, external forces, motionVelocities, root acceleration
	//Note that it assumes that the spatial articulated inertia are already calculated
	void FeatherstoneArticulation::calculateMassMatrixColInv(ScratchData& scratchData)
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();

		Cm::SpatialVectorF* PX_RESTRICT motionAccelerations = scratchData.motionAccelerations;
		Cm::SpatialVectorF* PX_RESTRICT spatialZAForces = scratchData.spatialZAVectors;

		// Input
		const PxReal* PX_RESTRICT jointAccelerations = scratchData.jointAccelerations;
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		// set base link motion acceleration to be zero because H should
		// be just affected by joint position/link position
		motionAccelerations[0] = Cm::SpatialVectorF::Zero();
		spatialZAForces[0] = Cm::SpatialVectorF::Zero();

		// First pass:
		//- Propagates the motion acceleration outward
		//- Calculate the first term of the joint force (I * A)
		for(PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = mArticulationData.getLink(linkID);
			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			// parent motion accelerations into child space
			Cm::SpatialVectorF accel = translateSpatialVector(-rw[linkID], motionAccelerations[link.parent]);
			const PxReal* jAcceleration = &jointAccelerations[jointDatum.jointOffset];

			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				accel.top += worldMotionMatrix[jointDatum.jointOffset + ind].top * jAcceleration[ind];
				accel.bottom += worldMotionMatrix[jointDatum.jointOffset + ind].bottom * jAcceleration[ind];
			}

			motionAccelerations[linkID] = accel;

			spatialZAForces[linkID] = mArticulationData.getWorldSpatialArticulatedInertia()[linkID] * accel;
		}

		// Second pass:
		//- Propagates forces inward (F_{i-1} = I_{i-1} * a_{i-1} + X * Z_i)
		//- Calculate joint force (tau_i = S_i * F_i)
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

	PxVec3 FeatherstoneArticulation::getArticulationCOM(const bool rootFrame)
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();
		PxVec3 articulationCOM = PxVec3(0.0f);
		PxReal totalMass = 0.0f; // needed because mInvSumMass is not available on GPU
		for(PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			const PxVec3 childPose = mArticulationData.mLinks[linkID].bodyCore->body2World.p;
			// This is because mArticulationData.mMasses[linkID] is not always available on GPU
			PxReal mass = 1.0f / mArticulationData.mLinks[linkID].bodyCore->inverseMass;
			totalMass += mass;

			articulationCOM += childPose * mass;
		}
		articulationCOM /= totalMass;

		if(rootFrame)
		{
			articulationCOM = mArticulationData.mLinks[0].bodyCore->body2World.getInverse().transform(articulationCOM);
		}

		return articulationCOM;
	}

	void FeatherstoneArticulation::getCentroidalMomentumMatrix(PxArticulationCache& cache)
	{
		if(mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "ArticulationHelper::getCentroidalMomentumMatrix() commonInit need to be called first to initialize data!");
			return;
		}

		PxReal* centroidalMomentumMatrix = cache.centroidalMomentumMatrix;
		PxReal* centroidalMomentumBias = cache.centroidalMomentumBias;
		const PxU32 dofCount = mArticulationData.getDofs();

		// adding mass matrix terms corresponding to the root DoFs
		for(PxU32 col = 0; col < dofCount + 6; ++col)
		{
			for(PxU32 row = 0; row < 6; ++row)
			{
				centroidalMomentumMatrix[col + row * (dofCount + 6)] = cache.massMatrix[col + row * (dofCount + 6)];
			}
		}

		// calculating the COM momentum from the spatial articulated inertia of the root in the mass matrix
		const PxReal totalMass = cache.massMatrix[2 * (dofCount + 6) + 2];
		const PxVec3 pg =
			PxVec3(cache.massMatrix[2 * (dofCount + 6) + 4], cache.massMatrix[5], cache.massMatrix[(dofCount + 6) + 3]) / totalMass;
		PxMat33 mat = FeatherstoneArticulation::constructSkewSymmetricMatrix(pg);

		// adding remaining contributions
		for(PxU32 col = 0; col < dofCount + 6; ++col)
		{
			const PxVec3 m = PxVec3(cache.massMatrix[0 * (dofCount + 6) + col], cache.massMatrix[1 * (dofCount + 6) + col],
									cache.massMatrix[2 * (dofCount + 6) + col]);
			const PxVec3 tmp = mat * m;

			for(PxU32 row = 3; row < 6; ++row)
			{
				centroidalMomentumMatrix[col + row * (dofCount + 6)] += tmp[row - 3];
			}
		}

		// adding coriolis force terms corresponding to the root DoFs
		for(PxU32 row = 0; row < 6; ++row)
		{
			centroidalMomentumBias[row] = cache.coriolisForce[row];
		}

		// adding remaining contributions
		const PxVec3 m = PxVec3(cache.coriolisForce[0], cache.coriolisForce[1], cache.coriolisForce[2]);
		const PxVec3 tmp = mat * m;
		for(PxU32 row = 3; row < 6; ++row)
		{
			centroidalMomentumBias[row] += tmp[row - 3];
		}
	}
} //namespace Dy

}
