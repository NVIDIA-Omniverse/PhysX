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
#include "DyDynamics.h"
#include "DyConstraintPrep.h"
#include "PxsContactManager.h"
#include "DyContactPrep.h"
#include "DySolverContext.h"
#include "DyTGSContactPrep.h"

#ifndef FEATURESTONE_DEBUG
#define FEATURESTONE_DEBUG 0
#endif

namespace physx
{
namespace Dy
{
	// PT: using C-style arrays instead of PxArrays saves 320 bytes per articulation

	ArticulationData::~ArticulationData()
	{
		PX_FREE(mISInvStIS);
		PX_FREE(mWorldMotionMatrix);
		PX_FREE(mMotionMatrix);
		PX_FREE(qstZIntIc);
		PX_FREE(qstZIc);
		PX_FREE(mIsW);
		PX_FREE(mDeferredQstZ);
		PX_FREE(mPosIterJointVelocities);
		PX_FREE(mJointTargetVelocities);
		PX_FREE(mJointTargetPositions);
		PX_FREE(mJointForce);
		PX_FREE(mJointPosition);
		PX_FREE(mJointNewVelocity);
		PX_FREE(mJointVelocity);
		PX_FREE(mJointAcceleration);

		PX_FREE(mRelativeQuat);
		PX_FREE(mRw);
		PX_FREE(mJointTransmittedForce);
		PX_FREE(mPosIterMotionVelocities);
		PX_FREE(mDeltaQ);
		PX_FREE(mAccumulatedPoses);
		PX_FREE(mInvStIs);
		PX_FREE(mMasses);
		PX_FREE(mWorldIsolatedSpatialArticulatedInertia);
		PX_FREE(mWorldSpatialArticulatedInertia);
		PX_FREE(mResponseMatrixW);
		PX_FREE(mPreTransform);
		PX_FREE(mDeltaMotionVector);
		PX_FREE(mStaticContactConstraintStartIndex);
		PX_FREE(mNbStaticContactConstraints);
		PX_FREE(mStatic1DConstraintStartIndex);
		PX_FREE(mNbStatic1DConstraints);
		PX_FREE(mZAInternalForces);
		PX_FREE(mZAForces);
		PX_FREE(mCoriolisVectors);
		PX_FREE(mMotionAccelerationsInternal);
		PX_FREE(mLinkIncomingJointForces);
		PX_FREE(mMotionAccelerations);
		PX_FREE(mSolverLinkSpatialImpulses);
		PX_FREE(mSolverLinkSpatialDeltaVels);
		PX_FREE(mMotionVelocities);

		PX_FREE(mJointData);
		PX_FREE(mPathToRootElements);
	}

	void ArticulationData::resizeLinkData(const PxU32 linkCount)
	{
		const PxU32 oldSize = mLinkCapacity;
		mLinkCapacity = linkCount;
		// PT: these could be allocated in a single linear array to further reduce the size of the class.
		mMotionVelocities = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mMotionVelocities");
		mSolverLinkSpatialDeltaVels = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mSolverLinkSpatialDeltaVels");
		mSolverLinkSpatialImpulses = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mSolverLinkSpatialImpulses");
		mMotionAccelerations = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mMotionAccelerations");
		mLinkIncomingJointForces = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mLinkIncomingJointForces");
		mMotionAccelerationsInternal = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mMotionAccelerationsInternal");
		mCoriolisVectors = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mCoriolisVectors");
		mZAForces = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mZAForces");
		mZAInternalForces = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mZAInternalForces");
		mNbStatic1DConstraints = PX_ALLOCATE(PxU32, linkCount, "mNbStatic1DConstraints");
		mStatic1DConstraintStartIndex = PX_ALLOCATE(PxU32, linkCount, "mStatic1DConstraintStartIndex");
		mNbStaticContactConstraints = PX_ALLOCATE(PxU32, linkCount, "mNbStaticContactConstraints");
		mStaticContactConstraintStartIndex = PX_ALLOCATE(PxU32, linkCount, "mStaticContactConstraintStartIndex");
		mDeltaMotionVector = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mDeltaMotionVector");
		mPreTransform = PX_ALLOCATE(PxTransform, linkCount, "mPreTransform");
		mResponseMatrixW = PX_ALLOCATE(TestImpulseResponse, linkCount, "mResponseMatrixW");
		mWorldSpatialArticulatedInertia = PX_ALLOCATE(SpatialMatrix, linkCount, "mWorldSpatialArticulatedInertia");
		mWorldIsolatedSpatialArticulatedInertia = PX_ALLOCATE(PxMat33, linkCount, "mWorldIsolatedSpatialArticulatedInertia");
		mMasses = PX_ALLOCATE(PxReal, linkCount, "mMasses");
		mInvStIs = PX_ALLOCATE(InvStIs, linkCount, "mInvStIs");
		mAccumulatedPoses = PX_ALLOCATE(PxTransform, linkCount, "mAccumulatedPoses");
		mDeltaQ = PX_ALLOCATE(PxQuat, linkCount, "mDeltaQ");
		mPosIterMotionVelocities = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mPosIterMotionVelocities");
		mJointTransmittedForce = PX_ALLOCATE(Cm::SpatialVectorF, linkCount, "mJointTransmittedForce");
		mRw = PX_ALLOCATE(PxVec3, linkCount, "mRw");
		mRelativeQuat = PX_ALLOCATE(PxQuat, linkCount, "mRelativeQuat");

		if (oldSize < linkCount)
		{
			ArticulationJointCoreData* oldJoints = mJointData;

			mJointData = PX_ALLOCATE(ArticulationJointCoreData, linkCount, "ArticulationJointCoreData");

			PxMemCopy(mJointData, oldJoints, sizeof(ArticulationJointCoreData)*oldSize);

			PX_FREE(oldJoints);

			const PxU32 newElems = (linkCount - oldSize);

			PxMemZero(mJointData + oldSize, sizeof(ArticulationJointCoreData) * newElems);

			for (PxU32 linkID = oldSize; linkID < linkCount; ++linkID)
			{
				PX_PLACEMENT_NEW(mJointData + linkID, ArticulationJointCoreData)();
			}
		}
	}

	void ArticulationData::resizeJointData(const PxU32 dofs)
	{
		mDofCapacity = dofs;
		// PT: these could be allocated in a single linear array to further reduce the size of the class.
		mJointAcceleration = PX_ALLOCATE(PxReal, dofs, "mJointAcceleration");
		mJointVelocity = PX_ALLOCATE(PxReal, dofs, "mJointVelocity");
		mJointNewVelocity = PX_ALLOCATE(PxReal, (dofs + 3), "mJointVelocity");	// PT: TODO: why +3 ?
		mJointPosition = PX_ALLOCATE(PxReal, dofs, "mJointPosition");
		mJointForce = PX_ALLOCATE(PxReal, dofs, "mJointForce");
		mJointTargetPositions = PX_ALLOCATE(PxReal, dofs, "mJointTargetPositions");
		mJointTargetVelocities = PX_ALLOCATE(PxReal, dofs, "mJointTargetVelocities");
		mPosIterJointVelocities = PX_ALLOCATE(PxReal, dofs, "mPosIterJointVelocities");
		mDeferredQstZ = PX_ALLOCATE(PxReal, dofs, "mDeferredQstZ");
		mIsW = PX_ALLOCATE(Cm::SpatialVectorF, dofs, "mIsW");
		qstZIc = PX_ALLOCATE(PxReal, dofs, "qstZIc");
		qstZIntIc = PX_ALLOCATE(PxReal, dofs, "qstZIntIc");
		mMotionMatrix = PX_ALLOCATE(Cm::UnAlignedSpatialVector, dofs, "mMotionMatrix");
		mWorldMotionMatrix = PX_ALLOCATE(Cm::UnAlignedSpatialVector, dofs, "mWorldMotionMatrix");
		mISInvStIS = PX_ALLOCATE(Cm::SpatialVectorF, dofs, "mISInvStIS");

		const PxU32 size = sizeof(PxReal) * dofs;
		PxMemZero(mJointAcceleration, size);
		PxMemZero(mJointVelocity, size);
		PxMemZero(mJointPosition, size);
		PxMemZero(mJointForce, size);
		PxMemZero(mJointTargetPositions, size);
		PxMemZero(mJointTargetVelocities, size);
	}

	PxU32 ArticulationData::countDofs() const
	{
		const PxU32 linkCount = mLinkCount;
		PxU32 totalDofs = 0;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = mLinks[linkID];
			const ArticulationJointCoreData& jointDatum = mJointData[linkID];
			const PxU8 dof = jointDatum.countJointDofs(link.inboundJoint);
			totalDofs += dof;
		}
		
		return totalDofs;
	}

	void ArticulationData::configureDofs()
	{
		const PxU32 linkCount = mLinkCount;
		PxU32 totalDof = 0;

		for(PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationJointCoreData& jointDatum = mJointData[linkID];
			const PxU8 dof = jointDatum.configureJointDofs(mLinks[linkID].inboundJoint);
			jointDatum.jointOffset = totalDof;
			totalDof += dof;
		}
	}

	void ArticulationData::setupDofs()
	{
		const PxU32 totalDofs = countDofs();

		const PxU32 existedTotalDofs = mDofs;

		if(totalDofs != existedTotalDofs)
		{
			resizeJointData(totalDofs + 1);
			mDofs = totalDofs;
		}
		configureDofs();
	}

	/////

	FeatherstoneArticulation::FeatherstoneArticulation() : mJcalcDirty(true)
	{
		mGPUDirtyFlags = 0;
	}

	FeatherstoneArticulation::~FeatherstoneArticulation()
	{
	}

	void FeatherstoneArticulation::setupLinks(PxU32 nbLinks, Dy::ArticulationLink* links)
	{
		if (nbLinks != mArticulationData.mLinkCount)
			mArticulationData.resizeLinkData(nbLinks);

		mArticulationData.mLinks				= links;
		mArticulationData.mLinkCount			= nbLinks;
		mArticulationData.mFlags				= mSolverDescCore ? &mSolverDescCore->flags : mSolverDescFlags;	// PT: PX-1399
		mArticulationData.mExternalAcceleration	= mAcceleration.begin();
		mArticulationData.mArticulation			= this;

		//allocate memory for articulation data
		mArticulationData.setupDofs();
	}

	void FeatherstoneArticulation::initPathToRoot()
	{
		Dy::ArticulationLink* links = mArticulationData.getLinks();
		
		const PxU32 linkCount = mArticulationData.getLinkCount();

		links[0].mPathToRootCount = 0;
		links[0].mPathToRootStartIndex = 0;

		PxU32 totalPathToRootCount = 1; //add on root

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			Dy::ArticulationLink& link = links[linkID];

			PxU32 parent = link.parent;
			PxU32 pathToRootCount = 1; // add myself to the path
			while (parent != 0) // don't add the root
			{
				parent = links[parent].parent;
				pathToRootCount++;
			}

			link.mPathToRootStartIndex = totalPathToRootCount;
			link.mPathToRootCount = PxU16(pathToRootCount);
			totalPathToRootCount += pathToRootCount;
		}

		if (mArticulationData.mNumPathToRootElements < totalPathToRootCount)
		{
			PX_FREE(mArticulationData.mPathToRootElements);
			mArticulationData.mPathToRootElements = PX_ALLOCATE(PxU32, totalPathToRootCount, "PxU32");
			mArticulationData.mNumPathToRootElements = totalPathToRootCount;
		}
		PxU32* pathToRootElements = mArticulationData.mPathToRootElements;

		pathToRootElements[0] = 0; //add on root index

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const Dy::ArticulationLink& link = links[linkID];
			PxU32* pathToRoot = &pathToRootElements[link.mPathToRootStartIndex];
			PxU32 numElements = link.mPathToRootCount;

			pathToRoot[--numElements] = linkID;
			PxU32 parent = links[linkID].parent;
			while (parent != 0)
			{
				pathToRoot[--numElements] = parent;
				parent = links[parent].parent;
			}
		}
	}

	void FeatherstoneArticulation::assignTendons(const PxU32 nbTendons, Dy::ArticulationSpatialTendon** tendons)
	{
		mArticulationData.mSpatialTendons = tendons;
		mArticulationData.mNumSpatialTendons = nbTendons;
	}

	void FeatherstoneArticulation::assignTendons(const PxU32 nbTendons, Dy::ArticulationFixedTendon** tendons)
	{
		mArticulationData.mFixedTendons = tendons;
		mArticulationData.mNumFixedTendons = nbTendons;
	}

	void FeatherstoneArticulation::assignMimicJoints(const PxU32 nbMimicJoints, Dy::ArticulationMimicJointCore** mimicJoints)
	{
		mArticulationData.mMimicJoints = mimicJoints;
		mArticulationData.mNbMimicJoints = nbMimicJoints;
	}

	PxU32 FeatherstoneArticulation::getDofs() const
	{
		return mArticulationData.getDofs();
	}

	PxU32 FeatherstoneArticulation::getDof(const PxU32 linkID)
	{
		const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
		return jointDatum.nbDof;
	}

	bool FeatherstoneArticulation::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool& shouldWake)
	{
		return applyCacheToDest(mArticulationData, cache, mArticulationData.getJointVelocities(), mArticulationData.getJointPositions(),
			mArticulationData.getJointForces(), mArticulationData.getJointTargetPositions(), mArticulationData.getJointTargetVelocities(), flag, shouldWake);
	}

	void FeatherstoneArticulation::copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, const bool isGpuSimEnabled)
	{
		if (flag & PxArticulationCacheFlag::eVELOCITY)
			copyJointData(mArticulationData, cache.jointVelocity, mArticulationData.getJointVelocities());

		if (flag & PxArticulationCacheFlag::eACCELERATION)
			copyJointData(mArticulationData, cache.jointAcceleration, mArticulationData.getJointAccelerations());

		if (flag & PxArticulationCacheFlag::ePOSITION)
			copyJointData(mArticulationData, cache.jointPosition, mArticulationData.getJointPositions());

		if (flag & PxArticulationCacheFlag::eFORCE)
			copyJointData(mArticulationData, cache.jointForce, mArticulationData.getJointForces());

		if (flag & PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS)
			copyJointData(mArticulationData, cache.jointTargetPositions, mArticulationData.getJointTargetPositions());

		if (flag & PxArticulationCacheFlag::eJOINT_TARGET_VELOCITIES)
			copyJointData(mArticulationData, cache.jointTargetVelocities, mArticulationData.getJointTargetVelocities());

		if (flag & PxArticulationCacheFlag::eLINK_VELOCITY)
		{
			const Cm::SpatialVectorF* vels = mArticulationData.getMotionVelocities();
			const PxU32 numLinks = mArticulationData.getLinkCount();
			for (PxU32 i = 0; i < numLinks; ++i)
			{
				const Cm::SpatialVectorF& vel = vels[i];
				cache.linkVelocity[i].linear = vel.bottom;
				cache.linkVelocity[i].angular = vel.top;
			}
		}

		const float dt = mArticulationData.getDt();

		if (flag & PxArticulationCacheFlag::eLINK_ACCELERATION)
		{
			const PxU32 linkCount = mArticulationData.getLinkCount();

			if(dt == 0.0f)
			{
				PxMemZero(cache.linkAcceleration, sizeof(PxSpatialVelocity)*linkCount);
			}
			else if(isGpuSimEnabled)
			{
				const Cm::SpatialVectorF* PX_RESTRICT linkMotionAccelerationsW = mArticulationData.getMotionAccelerations();

				//Iterate over all links and compute the acceleration for each link.
				for (PxU32 i = 0; i < linkCount; ++i)
				{
					cache.linkAcceleration[i].linear = linkMotionAccelerationsW[i].bottom;
					cache.linkAcceleration[i].angular = linkMotionAccelerationsW[i].top;
				}
			}
			else
			{
				const PxReal invDt = 1.0f / dt;

				const Cm::SpatialVectorF* PX_RESTRICT linkMotionAccelerationsW = mArticulationData.getMotionAccelerations();
				const Cm::SpatialVectorF* PX_RESTRICT linkSpatialDeltaVelsW = mArticulationData.getSolverLinkSpatialDeltaVels();

				//Iterate over all links and compute the acceleration for each link.
				for (PxU32 i = 0; i < linkCount; ++i)
				{
					cache.linkAcceleration[i].linear = linkMotionAccelerationsW[i].bottom + linkSpatialDeltaVelsW[i].bottom*invDt;
					cache.linkAcceleration[i].angular = linkMotionAccelerationsW[i].top + linkSpatialDeltaVelsW[i].top*invDt;
				}				
			}
		}

		if(flag & PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE)
		{
			const PxU32 linkCount = mArticulationData.getLinkCount();
			if (dt == 0.0f)
			{
				PxMemZero(cache.linkIncomingJointForce, sizeof(PxSpatialForce)*linkCount);
			}
			else if(isGpuSimEnabled)
			{
				//Calculation already completed on gpu.
				const Cm::SpatialVectorF* incomingJointForces = mArticulationData.getLinkIncomingJointForces();

				//Root links have no incoming joint.
				cache.linkIncomingJointForce[0].force = PxVec3(PxZero);
				cache.linkIncomingJointForce[0].torque = PxVec3(PxZero);
			
				//Iterate over all links and get the incoming joint force for each link.
				for (PxU32 i = 1; i < linkCount; ++i)
				{
					cache.linkIncomingJointForce[i].force = incomingJointForces[i].top;
					cache.linkIncomingJointForce[i].torque = incomingJointForces[i].bottom;
				}
			}
			else
			{
				const PxReal invDt = 1.0f / dt;

				//Get everything we need.
				const Cm::SpatialVectorF* PX_RESTRICT linkZAForcesExtW = mArticulationData.getSpatialZAVectors();
				const Cm::SpatialVectorF* PX_RESTRICT linkZAForcesIntW = mArticulationData.getSpatialZAInternalVectors();
				const Cm::SpatialVectorF* PX_RESTRICT linkMotionAccelerationsW = mArticulationData.getMotionAccelerations();
				const SpatialMatrix* PX_RESTRICT linkSpatialInertiasW = mArticulationData.getWorldSpatialArticulatedInertia();
				const Cm::SpatialVectorF* PX_RESTRICT linkSpatialDeltaVelsW = mArticulationData.getSolverLinkSpatialDeltaVels();
				const Cm::SpatialVectorF* PX_RESTRICT linkSpatialImpulsesW = mArticulationData.getSolverLinkSpatialImpulses();

				//Root links have no incoming joint.
				cache.linkIncomingJointForce[0].force = PxVec3(PxZero);
				cache.linkIncomingJointForce[0].torque = PxVec3(PxZero);

				//Iterate over all links and compute the incoming joint force for each link.
				for (PxU32 i = 1; i < linkCount; ++i)
				{
					const ArticulationLink& link = mArticulationData.getLink(i);
					const ArticulationJointCore* joint = link.inboundJoint;
					const PxTransform Gc = link.bodyCore->body2World;
					const PxTransform Lc = joint->childPose;				
					const PxTransform GcLc = Gc*Lc;
					const PxVec3 dW = Gc.rotate(Lc.p);

					//Compute the force measured at the link.
					const Cm::SpatialVectorF incomingJointForceAtLinkW =
						linkSpatialInertiasW[i]*(linkMotionAccelerationsW[i] + linkSpatialDeltaVelsW[i]*invDt) + 
							(linkZAForcesExtW[i] + linkZAForcesIntW[i] + linkSpatialImpulsesW[i]*invDt);

					//Compute the equivalent force measured at the joint.
					const Cm::SpatialVectorF incomingJointForceAtJointW =
						 FeatherstoneArticulation::translateSpatialVector(-dW, incomingJointForceAtLinkW);
		
					//Transform the force to the child joint frame.
					cache.linkIncomingJointForce[i].force = GcLc.rotateInv(incomingJointForceAtJointW.top);
					cache.linkIncomingJointForce[i].torque = GcLc.rotateInv(incomingJointForceAtJointW.bottom);
				}
			}						
		}

		if (flag & PxArticulationCacheFlag::eROOT_TRANSFORM)
		{
			const ArticulationLink& rLink = mArticulationData.getLink(0);
			const PxsBodyCore& rBodyCore = *rLink.bodyCore;
			const PxTransform& body2World = rBodyCore.body2World;
			// PT:: tag: scalar transform*transform
			cache.rootLinkData->transform = body2World * rBodyCore.getBody2Actor().getInverse();
		}

		if (flag & PxArticulationCacheFlag::eROOT_VELOCITIES)
		{
			const Cm::SpatialVectorF& vel = mArticulationData.getMotionVelocity(0);
			cache.rootLinkData->worldLinVel = vel.bottom;
			cache.rootLinkData->worldAngVel = vel.top;

			const Cm::SpatialVectorF& accel = mArticulationData.getMotionAcceleration(0);
			cache.rootLinkData->worldLinAccel = accel.bottom;
			cache.rootLinkData->worldAngAccel = accel.top;
		}
	}

	PxU32 FeatherstoneArticulation::getCacheDataSize(PxU32 totalDofs, PxU32 linkCount)
	{
		const PxU32 totalSize =
			sizeof(PxSpatialForce) * linkCount						// external force
			+ sizeof(PxReal) * (6 + totalDofs) * (linkCount * 6)	// Free floating base dofs = 6 + totalDofs, and each link (incl. base) velocity has 6 elements
																	// == offset to end of dense jacobian (assuming free floating base)
			+ sizeof(PxReal) * (totalDofs + 6) * (totalDofs + 6)	// mass matrix
			+ sizeof(PxReal) * (totalDofs + 6) 						// Coriolis and centrifugal compensation force
			+ sizeof(PxReal) * (totalDofs + 6)						// gravity compensation force
			+ sizeof(PxReal) * 6 * (totalDofs + 6)					// centroidal momentum matrix
			+ sizeof(PxReal) * 6									// centroidal momentum bias
			+ sizeof(PxReal) * totalDofs * 6						// jointVelocity (PxArticulationCacheFlag::eVELOCITY)
																	// jointAcceleration (PxArticulationCacheFlag::eACCELERATION)
																	// jointPosition (PxArticulationCacheFlag::ePOSITION)
																	// joint force (PxArticulationCacheFlag::eFORCE)
																	// joint target positions (PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS)
																	// joint target velocities (PxArticulationCacheFlag::eJOINT_TARGET_VELOCITIES)
			+ sizeof(PxSpatialVelocity) * linkCount * 2				// link velocity, (PxArticulationCacheFlag::eLINK_VELOCITY)
																	// link acceleration (PxArticulationCacheFlag::eLINK_ACCELERATION)
			+ sizeof(PxSpatialForce) * linkCount					// link incoming joint forces (PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE)
			+ sizeof(PxVec3) * linkCount * 2						// link force (PxArticulationCacheFlag::eLINK_FORCE)
																	// link torque (PxArticulationCacheFlag::eLINK_TORQUE)
			+ sizeof(PxArticulationRootLinkData);					// root link data (PxArticulationCacheFlag::eROOT_TRANSFORM, PxArticulationCacheFlag::eROOT_VELOCITIES)
	
		return totalSize;
	}

	// AD: attention - some of the types here have 16B alignment requirements.
	// But the size of PxArticulationCache is not necessarily a multiple of 16B.
	PX_COMPILE_TIME_ASSERT(sizeof(Cm::SpatialVector)==sizeof(PxSpatialForce));
	PxArticulationCache* FeatherstoneArticulation::createCache(PxU32 totalDofs, PxU32 linkCount)
	{
		const PxU32 pxArticulationCacheSize16BAligned = (sizeof(PxArticulationCache) + 15) & ~15;

		const PxU32 totalSize = getCacheDataSize(totalDofs, linkCount) + pxArticulationCacheSize16BAligned;

		PxU8* tCache = reinterpret_cast<PxU8*>(PX_ALLOC(totalSize, "Articulation cache"));
		PX_ASSERT(((size_t)tCache & 15) == 0);

		PxMemZero(tCache, totalSize);

		PxArticulationCache* cache = reinterpret_cast<PxArticulationCache*>(tCache);

		PxU32 offset = pxArticulationCacheSize16BAligned;

		// the following code assumes that the size of PxSpatialForce and PxSpatialVelocity are multiples of 16B
		PX_COMPILE_TIME_ASSERT((sizeof(PxSpatialForce) & 15) == 0);
		PX_COMPILE_TIME_ASSERT((sizeof(PxSpatialVelocity) & 15) == 0);

		// PT: filled in FeatherstoneArticulation::getGeneralizedExternalForce, size = mArticulationData.getLinkCount()
		// 16B aligned
		PX_ASSERT((offset & 15) == 0);
		cache->externalForces = reinterpret_cast<PxSpatialForce*>(tCache + offset);
		offset += sizeof(PxSpatialForce) * linkCount;

		// PT: PxArticulationCacheFlag::eLINK_VELOCITY
		// 16B aligned
		PX_ASSERT((offset & 15) == 0);
		cache->linkVelocity = reinterpret_cast<PxSpatialVelocity*>(tCache + offset);
		offset += sizeof(PxSpatialVelocity) * linkCount;

		// PT: PxArticulationCacheFlag::eLINK_ACCELERATION
		// 16B aligned
		PX_ASSERT((offset & 15) == 0);
		cache->linkAcceleration = reinterpret_cast<PxSpatialVelocity*>(tCache + offset);
		offset += sizeof(PxSpatialVelocity) * linkCount;

		// PT: PxArticulationCacheFlag::eLINK_INCOMING_JOINT_FORCE
		// 16B aligned
		PX_ASSERT((offset & 15) == 0);
		cache->linkIncomingJointForce = reinterpret_cast<PxSpatialForce*>(tCache + offset);
		offset += sizeof(PxSpatialForce) * linkCount;

		// PxArticulationCacheFlag::eLINK_FORCE
		PX_ASSERT((offset & 15) == 0);
		cache->linkForce = reinterpret_cast<PxVec3*>(tCache + offset);
        offset += sizeof(PxVec3) * linkCount;

		// PxArticulationCacheFlag::eLINK_TORQUE
		cache->linkTorque = reinterpret_cast<PxVec3*>(tCache + offset);
		offset += sizeof(PxVec3) * linkCount;

		cache->denseJacobian = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * (6 + totalDofs) * (linkCount * 6);				//size of dense jacobian assuming free floating base link.

		cache->massMatrix = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * (totalDofs + 6) * (totalDofs + 6);

		cache->coriolisForce = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * (totalDofs + 6);

		cache->gravityCompensationForce = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * (totalDofs + 6);

		cache->centroidalMomentumMatrix = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * 6 * (totalDofs + 6);

		cache->centroidalMomentumBias = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * 6;

		// PT: PxArticulationCacheFlag::eVELOCITY
		cache->jointVelocity = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * totalDofs;

		// PT: PxArticulationCacheFlag::eACCELERATION
		cache->jointAcceleration = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * totalDofs;

		// PT: PxArticulationCacheFlag::ePOSITION
		cache->jointPosition = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * totalDofs;

		// PT: PxArticulationCacheFlag::eFORCE
		cache->jointForce = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * totalDofs;

		// PxArticulationCacheFlag::eJOINT_TARGET_POSITIONS
		cache->jointTargetPositions = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * totalDofs;

		// PxArticulationCacheFlag::eJOINT_TARGET_VELOCITIES
		cache->jointTargetVelocities = reinterpret_cast<PxReal*>(tCache + offset);
		offset += sizeof(PxReal) * totalDofs;

		// PT: PxArticulationCacheFlag::eROOT_TRANSFORM, PxArticulationCacheFlag::eROOT_VELOCITIES
		cache->rootLinkData = reinterpret_cast<PxArticulationRootLinkData*>(tCache + offset);
		PX_ASSERT((offset + sizeof(PxArticulationRootLinkData)) == totalSize);

		cache->coefficientMatrix = NULL;
		cache->lambda = NULL;

		PxU32 scratchMemorySize =
			sizeof(Cm::SpatialVectorF) * linkCount * 5	//motionVelocity, motionAccelerations, coriolisVectors, spatialZAVectors, externalAccels;
			+ sizeof(Dy::SpatialMatrix) * linkCount		//compositeSpatialInertias;
			+ sizeof(PxVec3) * linkCount * 2			//linkForce, linkTorque
			+ sizeof(PxReal) * totalDofs * 7;			//jointVelocity, jointAcceleration, jointForces, jointPositions, jointFrictionForces, jointTargetPositions, jointTargetVelocities

		scratchMemorySize = (scratchMemorySize+15)&~15;

		void* scratchMemory = PX_ALLOC(scratchMemorySize, "Cache scratch memory");
		cache->scratchMemory = scratchMemory;

		PxcScratchAllocator* sa = PX_NEW(PxcScratchAllocator);
		sa->setBlock(scratchMemory, scratchMemorySize);
		cache->scratchAllocator = sa;

		return cache;
	}

	static PX_FORCE_INLINE Mat33V loadPxMat33(const PxMat33& m)
	{
		return Mat33V(Vec3V_From_Vec4V(V4LoadU(&m.column0.x)), 
			Vec3V_From_Vec4V(V4LoadU(&m.column1.x)), V3LoadU(&m.column2.x));
	}

	static PX_FORCE_INLINE void storePxMat33(const Mat33V& src, PxMat33& dst)
	{
		V3StoreU(src.col0, dst.column0);
		V3StoreU(src.col1, dst.column1);
		V3StoreU(src.col2, dst.column2);
	}

	void FeatherstoneArticulation::translateInertia(const PxMat33& sTod, SpatialMatrix& inertia)
	{
#if 1
		Mat33V sTodV = loadPxMat33(sTod);
		Mat33V dTos = M33Trnsps(sTodV);

		const Mat33V tL = loadPxMat33(inertia.topLeft);
		const Mat33V tR = loadPxMat33(inertia.topRight);
		const Mat33V bL = loadPxMat33(inertia.bottomLeft);

		const Mat33V bl = M33Add(M33MulM33(sTodV, tL), bL);
		const Mat33V br = M33Add(M33MulM33(sTodV, tR), M33Trnsps(tL));
		const Mat33V bottomLeft = M33Add(bl, M33MulM33(br, dTos));

		storePxMat33(M33Add(tL, M33MulM33(tR, dTos)), inertia.topLeft);
		storePxMat33(M33Scale(M33Add(bottomLeft, M33Trnsps(bottomLeft)), FHalf()), inertia.bottomLeft);
#else
		const PxMat33 dTos = sTod.getTranspose();

		PxMat33 bl = sTod * inertia.topLeft + inertia.bottomLeft;
		PxMat33 br = sTod * inertia.topRight + inertia.getBottomRight();

		inertia.topLeft = inertia.topLeft + inertia.topRight * dTos;
		inertia.bottomLeft = bl + br * dTos;

		//aligned inertia - make it symmetrical! OPTIONAL!!!!
		inertia.bottomLeft = (inertia.bottomLeft + inertia.bottomLeft.getTranspose()) * 0.5f;
#endif
	}

	void FeatherstoneArticulation::getImpulseResponse(
		PxU32 linkID,
		const Cm::SpatialVector& impulse,
		Cm::SpatialVector& deltaVV) const
	{
		//impulse lin is contact normal, and ang is raxn. R is body2World, R(t) is world2Body
		//| R(t),	0	|
		//| R(t)*r, R(t)|
		//r is the vector from center of mass to contact point
		//p(impulse) =	|n|
		//				|0|

		Cm::SpatialVectorF deltaV = mArticulationData.getImpulseResponseMatrixWorld()[linkID].getLinkDeltaVImpulseResponse(reinterpret_cast<const Cm::SpatialVectorF&>(impulse));

		deltaVV.linear = deltaV.bottom;
		deltaVV.angular = deltaV.top;		
	}

	void FeatherstoneArticulation::getImpulseResponse(
		PxU32 linkID,
		const Cm::SpatialVectorV& impulse,
		Cm::SpatialVectorV& deltaVV) const
	{
#if 0
		const PxTransform& body2World = mArticulationData.getPreTransform(linkID);

		QuatV rot = QuatVLoadU(&body2World.q.x);

		Cm::SpatialVectorV impl(QuatRotateInv(rot, impulse.linear), QuatRotateInv(rot, impulse.angular));

		//transform p(impulse) from world space to the local space of linkId

		//Cm::SpatialVectorF impl(impulse.linear, impulse.angular);
		Cm::SpatialVectorV deltaV = mArticulationData.getImpulseResponseMatrix()[linkID].getResponse(impl);
		deltaVV.linear = QuatRotate(rot, deltaV.angular);
		deltaVV.angular = QuatRotate(rot, deltaV.linear);
#else
		const Cm::SpatialVectorV deltaV = mArticulationData.getImpulseResponseMatrixWorld()[linkID].getLinkDeltaVImpulseResponse(impulse);
		deltaVV.linear = deltaV.angular;
		deltaVV.angular = deltaV.linear;
#endif
	}

	//This will return world space SpatialVectorV
	Cm::SpatialVectorV FeatherstoneArticulation::getLinkVelocity(const PxU32 linkID) const
	{
		//This is in the world space
		const Cm::SpatialVectorF& motionVelocity = mArticulationData.getMotionVelocity(linkID);

		Cm::SpatialVectorV velocity;
		velocity.linear = V3LoadA(motionVelocity.bottom);
		velocity.angular = V3LoadA(motionVelocity.top);

		return velocity;
	}

	Cm::SpatialVector FeatherstoneArticulation::getLinkScalarVelocity(const PxU32 linkID) const
	{
		//This is in the world space
		const Cm::SpatialVectorF& motionVelocity = mArticulationData.getMotionVelocity(linkID);

		return Cm::SpatialVector(motionVelocity.bottom, motionVelocity.top);
	}

	Cm::SpatialVectorV FeatherstoneArticulation::getLinkMotionVector(const PxU32 linkID) const
	{
		const Cm::SpatialVectorF& motionVector = mArticulationData.getDeltaMotionVector(linkID);

		Cm::SpatialVectorV velocity;
		velocity.linear = V3LoadU(motionVector.bottom);
		velocity.angular = V3LoadU(motionVector.top);

		return velocity;
	}

	//this is called by island gen to determine whether the articulation should be awake or sleep
	Cm::SpatialVector FeatherstoneArticulation::getMotionVelocity(const PxU32 linkID) const
	{
		//This is in the world space
		const Cm::SpatialVectorF& motionVelocity = mArticulationData.getPosIterMotionVelocities()[linkID];
		return Cm::SpatialVector(motionVelocity.bottom, motionVelocity.top);
	}

	Cm::SpatialVector FeatherstoneArticulation::getMotionAcceleration(const PxU32 linkID, const bool isGpuSimEnabled) const
	{
		Cm::SpatialVector a = Cm::SpatialVector::zero();
		const float dt = mArticulationData.getDt();
		if(dt > 0.0f)
		{
			if(isGpuSimEnabled)
			{	
				const Cm::SpatialVectorF& linkAccel = mArticulationData.mMotionAccelerations[linkID];
				a = Cm::SpatialVector(linkAccel.bottom, linkAccel.top);
			}		
			else
			{
				const PxReal invDt = 1.0f / dt;
				const Cm::SpatialVectorF linkAccel = 
					mArticulationData.mMotionAccelerations[linkID] + mArticulationData.mSolverLinkSpatialDeltaVels[linkID] * invDt;
				a = Cm::SpatialVector(linkAccel.bottom, linkAccel.top);
			}
		}
		return a;
	}

	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV)
	{
		PX_ASSERT(deltaV);

		ArticulationData& data = articulation.mArticulationData;
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		Cm::SpatialVectorF* PX_RESTRICT motionVelocities = data.getMotionVelocities();
		//Cm::SpatialVectorF* deferredZ = data.getSpatialZAVectors();
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();

		//PxTransform* poses = data.getAccumulatedPoses();
		//const PxTransform* poses = data.getPreTransform();

		//This will be zero at the beginning of the frame
		PxReal* PX_RESTRICT jointNewVelocities = data.getJointNewVelocities();

		if (fixBase)
		{
			deltaV[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			//ArticulationLink& link = links[0];

			deltaV[0] = data.getBaseInvSpatialArticulatedInertiaW() * -data.getRootDeferredZ();

			motionVelocities[0] += deltaV[0];
			data.incrementSolverSpatialDeltaVel(0, deltaV[0]);

			PX_ASSERT(motionVelocities[0].isFinite());
		}

		const PxVec3* PX_RESTRICT rw = data.getRw();

		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 i = 1; i < linkCount; i++)
		{
			const ArticulationLink& tLink = links[i];
			const ArticulationJointCoreData& tJointDatum = jointData[i];

			const Cm::SpatialVectorF dV = propagateAccelerationW(
				rw[i], deltaV[tLink.parent], 
				data.getInvStIs(i), &data.getWorldMotionMatrix(tJointDatum.jointOffset), 
				&data.getIsW(tJointDatum.jointOffset), &data.getDeferredQstZ()[tJointDatum.jointOffset], tJointDatum.nbDof,
				&jointNewVelocities[tJointDatum.jointOffset]);

			deltaV[i] = dV;
			motionVelocities[i] += dV;
			data.incrementSolverSpatialDeltaVel(i, dV);

			PX_ASSERT(motionVelocities[i].isFinite());
		}

		//PxMemZero(deferredZ, sizeof(Cm::SpatialVectorF)*linkCount);
		PxMemZero(data.getDeferredQstZ(), sizeof(PxReal) * data.getDofs());

		data.getRootDeferredZ() = Cm::SpatialVectorF::Zero();
	}

	// Apply substep external forces and joint actuation, propagate from leaves to root
	// Gravity and external force/acceleration fullstep application happens in FeatherstoneArticulation::computeLinkStates
	// Joint force fullstep application happens in FeatherstoneArticulation::computePropagateSpatialInertia_ZA_ZIc
	void FeatherstoneArticulation::applyTgsSubstepForces(FeatherstoneArticulation* articulation, PxReal stepDt, Cm::SpatialVectorF* scratchExtForcesArticulatedYW)
	{
		ArticulationData& data = articulation->mArticulationData;

		// inputs
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT jointDofMotionMatrixW = data.getWorldMotionMatrix();
		const Cm::SpatialVectorF* PX_RESTRICT jointDofISInvStISW  = data.getISInvStIS();
		const PxVec3* PX_RESTRICT childToParentRW = data.getRw();
		const ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		const PxReal* PX_RESTRICT jointForces = data.getJointForces();
		const Cm::SpatialVector* PX_RESTRICT linkExternalAcceleration = data.getExternalAccelerations(); // we have converted to spatial forces here already, see FeatherstoneArticulation::computeLinkStates

		// outputs
		PxReal* jointDofQStY = data.getDeferredQstZ();
		Cm::SpatialVectorF& rootDeferredZ = data.getRootDeferredZ();
		data.mJointDirty = true;

		// Scratch only temporarily needed until this function returns, passed in from outside to avoid allocating here.
		// We set it to zero because it might be in a dirty state.
		PX_ASSERT(scratchExtForcesArticulatedYW);
		Cm::SpatialVectorF* extForcesArticulatedYW = scratchExtForcesArticulatedYW;
		for(PxU32 linkID = 0; linkID < linkCount; linkID++)
		{
			extForcesArticulatedYW[linkID] = Cm::SpatialVectorF::Zero();
		}

		// Iterate from leaves to root, accumulating the substep forces in articulatedY up the tree
		// This is like FeatherstoneArticulation::propagateImpulseW but with impulses on all links
		const PxI32 startIndex = linkCount - 1;
		for (PxI32 linkID = startIndex; linkID > 0; --linkID)
		{
			const ArticulationLink& link = links[linkID];
			const Cm::SpatialVector& extForceStoredAsMotionVec = linkExternalAcceleration[linkID];
			const Cm::SpatialVectorF isolatedYW(extForceStoredAsMotionVec.linear, extForceStoredAsMotionVec.angular); // order defined in FeatherstoneArticulation::computeLinkStates
			const Cm::SpatialVectorF articulatedYW = isolatedYW * stepDt + extForcesArticulatedYW[linkID]; // could avoid adding this for leaf links
			const ArticulationJointCoreData& jointDatum = jointData[linkID];
			const PxU32 jointOffset = jointDatum.jointOffset;
			const PxU8 dofCount = jointDatum.nbDof;

			Cm::SpatialVectorF parentYW = articulatedYW;
			for(PxU8 jointDof = 0; jointDof < dofCount; jointDof++)
			{
				const PxU32 jointIdx = jointOffset + jointDof;
				const Cm::UnAlignedSpatialVector& sa = jointDofMotionMatrixW[jointIdx];
				const Cm::SpatialVectorF& ISInvStISW = jointDofISInvStISW[jointIdx];
				const PxReal jointForce = jointForces[jointIdx];

				const PxReal stZY = jointForce * stepDt - sa.innerProduct(articulatedYW);
				PX_ASSERT(PxIsFinite(stZY));

				parentYW += ISInvStISW * stZY;
				jointDofQStY[jointIdx] += stZY; // store as deferred impulse in joint space
			}

			const PxU32 parentLinkID = link.parent;
			if(parentLinkID == 0 && fixBase)
				continue; // no need to propagate to a fixed base

			parentYW = FeatherstoneArticulation::translateSpatialVector(childToParentRW[linkID], parentYW);
			extForcesArticulatedYW[parentLinkID] += parentYW; // add rather than assign because other children may have already contributed
		}

		// process the root if it's not fixed
		if(!fixBase)
		{
			const Cm::SpatialVector& extForceStoredAsMotionVec = linkExternalAcceleration[0];
			const Cm::SpatialVectorF isolatedYW(extForceStoredAsMotionVec.linear, extForceStoredAsMotionVec.angular); // order defined in FeatherstoneArticulation::computeLinkStates
			const Cm::SpatialVectorF articulatedYW = isolatedYW * stepDt + extForcesArticulatedYW[0];
			rootDeferredZ += articulatedYW;
		}
	}

	void FeatherstoneArticulation::recordDeltaMotion(FeatherstoneArticulation* articulation, const PxReal dt, Cm::SpatialVectorF* deltaV)
	{
		PX_ASSERT(deltaV);

		ArticulationData& data = articulation->mArticulationData;
		const PxU32 linkCount = data.getLinkCount();

		const PxU32 flags = data.getArticulationFlags();

		if (data.mJointDirty)
		{
			PxcFsFlushVelocity(*articulation, deltaV);
		}

		Cm::SpatialVectorF* PX_RESTRICT deltaMotion = data.getDeltaMotionVector();
		Cm::SpatialVectorF* PX_RESTRICT posMotionVelocities = data.getPosIterMotionVelocities();
		Cm::SpatialVectorF* PX_RESTRICT motionVelocities = data.getMotionVelocities();

		PxReal* PX_RESTRICT jointPosition = data.getJointPositions();
		PxReal* PX_RESTRICT jointNewVelocities = data.getJointNewVelocities();
		
		const PxTransform* PX_RESTRICT preTransform = data.getPreTransform();
		PxTransform* PX_RESTRICT accumulatedPoses = data.getAccumulatedPoses();
		PxQuat* PX_RESTRICT deltaQ = data.getDeltaQ();

		//data.mAccumulatedDt += dt;
		data.setDt(dt);

		const bool fixBase = flags & PxArticulationFlag::eFIX_BASE;

		if (!fixBase)
		{
			const Cm::SpatialVectorF& motionVelocity = motionVelocities[0];
			PX_ASSERT(motionVelocity.top.isFinite());
			PX_ASSERT(motionVelocity.bottom.isFinite());

			const PxTransform preTrans = accumulatedPoses[0];

			const PxVec3 lin = motionVelocity.bottom;
			const PxVec3 ang = motionVelocity.top;

			const PxVec3 newP = preTrans.p + lin * dt;

			const PxTransform newPose = PxTransform(newP, PxExp(ang*dt) * preTrans.q);

			//PxVec3 lin, ang;
			/*calculateNewVelocity(newPose, preTransform[0],
				1.f, lin, ang);		*/	

			accumulatedPoses[0] = newPose;

			PxQuat dq = newPose.q * preTransform[0].q.getConjugate();

			if (dq.w < 0.f)
				dq = -dq;

			deltaQ[0] = dq;

			Cm::SpatialVectorF delta = motionVelocity * dt;

			deltaMotion[0] += delta;
			posMotionVelocities[0] += delta;
		}

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			
			const PxTransform newPose = articulation->propagateTransform(linkID, data.getLinks(), jointDatum, data.getMotionVelocities(),
				dt, accumulatedPoses[data.getLink(linkID).parent], accumulatedPoses[linkID], 
				jointNewVelocities, jointPosition, &data.getMotionMatrix(jointDatum.jointOffset));

			//data.mDeltaQ[linkID] = preTransform[linkID].q.getConjugate() * newPose.q;
			PxQuat dq = newPose.q * preTransform[linkID].q.getConjugate();

			if(dq.w < 0.f)
				dq = -dq;

			deltaQ[linkID] = dq;

			/*PxVec3 lin, ang;
			calculateNewVelocity(newPose, preTransform[linkID],
				1.f, lin, ang);*/

			PxVec3 lin = (newPose.p - preTransform[linkID].p);

			Cm::SpatialVectorF delta = motionVelocities[linkID] * dt;
			
			//deltaMotion[linkID].top = ang;// motionVeloties[linkID].top * dt;
			deltaMotion[linkID].top += delta.top;
			deltaMotion[linkID].bottom = lin;// motionVeloties[linkID].top * dt;
			posMotionVelocities[linkID] += delta;

			//Record the new current pose
			accumulatedPoses[linkID] = newPose;
		}
	}

	Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocity(const PxU32 linkID, PxReal* jointDofSpeeds) const
	{
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
	
		const ArticulationLink* PX_RESTRICT links = mArticulationData.getLinks();

		Cm::SpatialVectorF deltaV(PxVec3(0.f), PxVec3(0.f));

		if (!fixBase)
		{
			//deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
			//DeferredZ now in world space!
			deltaV = mArticulationData.mBaseInvSpatialArticulatedInertiaW * -mArticulationData.getRootDeferredZ();
		}

		const PxU32 startIndex = links[linkID].mPathToRootStartIndex;
		const PxU32 elementCount = links[linkID].mPathToRootCount;

		//Take care of the case where elementCount is 0. 
		const PxU32 elementCountMinusOne = (0 == elementCount) ? 0 : elementCount -1;

		const PxU32* pathToRootElements = &mArticulationData.mPathToRootElements[startIndex];
	
		const PxReal* PX_RESTRICT deferredQstZ = mArticulationData.getDeferredQstZ();
		const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();
		const Cm::SpatialVectorF* PX_RESTRICT isW = mArticulationData.getIsW();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		//We don't need to compute the deltaJointDofSpeed for these links and joints.
		for (PxU32 i = 0; i < elementCountMinusOne; ++i)
		{
			const PxU32 index = pathToRootElements[i];
			PX_ASSERT(links[index].parent < index);

			const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(index).nbDof;

			deltaV = propagateAccelerationW(
				rw[index], deltaV,
				invStIS[index], &worldMotionMatrix[jointOffset], &isW[jointOffset],
				&deferredQstZ[jointOffset], dofCount, 
				NULL);
		}
	
		//The last link might need to compute the deltaJointDofSpeed.
		PxReal deltaJointDofSpeeds[3] = {0, 0, 0};
		PxReal* optionalDeltaJointDofSpeeds = jointDofSpeeds ? deltaJointDofSpeeds : NULL;
		for (PxU32 i = elementCountMinusOne; i < elementCount; ++i)
		{
			const PxU32 index = pathToRootElements[i];
			PX_ASSERT(links[index].parent < index);

			const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(index).nbDof;

			deltaV = propagateAccelerationW(
				rw[index], deltaV,
				invStIS[index], &worldMotionMatrix[jointOffset], &isW[jointOffset],
				&deferredQstZ[jointOffset], dofCount, 
				optionalDeltaJointDofSpeeds);
		}

		if(jointDofSpeeds)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(linkID).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(linkID).nbDof;
			const PxReal* jointNewVelocities = mArticulationData.getJointNewVelocities();

			for(PxU32 i = 0; i < dofCount; i++)
			{
				jointDofSpeeds[i] = jointNewVelocities[jointOffset + i] + deltaJointDofSpeeds[i];	
			}
		}
		const Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV;			
		return Cm::SpatialVectorV(Cm::SpatialVector(vel.bottom, vel.top));
	}

	void FeatherstoneArticulation::pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1) const
	{
		{
			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

			const ArticulationLink* PX_RESTRICT links = mArticulationData.getLinks();

			Cm::SpatialVectorF deltaV(PxVec3(0.f), PxVec3(0.f));

			if (!fixBase)
			{
				//deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
				deltaV = mArticulationData.mBaseInvSpatialArticulatedInertiaW * (-mArticulationData.mRootDeferredZ);
			}

			const PxU32* PX_RESTRICT pathToRootElements = mArticulationData.mPathToRootElements;
			const PxReal* PX_RESTRICT deferredQstZ = mArticulationData.getDeferredQstZ();
			const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();
			const Cm::SpatialVectorF* PX_RESTRICT isW = mArticulationData.getIsW();
			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
			const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

			const Dy::ArticulationLink& link0 = links[linkID];
			const Dy::ArticulationLink& link1 = links[linkID1];

			const PxU32* pathToRoot0 = &pathToRootElements[link0.mPathToRootStartIndex];
			const PxU32* pathToRoot1 = &pathToRootElements[link1.mPathToRootStartIndex];

			const PxU32 numElems0 = link0.mPathToRootCount;
			const PxU32 numElems1 = link1.mPathToRootCount;

			PxU32 offset = 0;
			while (pathToRoot0[offset] == pathToRoot1[offset])
			{
				const PxU32 index = pathToRoot0[offset++];
				PX_ASSERT(links[index].parent < index);
				if (offset >= numElems0 || offset >= numElems1)
					break;

				const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(index).nbDof;

				deltaV = propagateAccelerationW(
					rw[index], deltaV, 
					invStIS[index], &worldMotionMatrix[jointOffset], 
					&isW[jointOffset], &deferredQstZ[jointOffset], dofCount,
					NULL);
			}

			Cm::SpatialVectorF deltaV1 = deltaV;

			for (PxU32 idx = offset; idx < numElems0; ++idx)
			{
				const PxU32 index = pathToRoot0[idx];
				PX_ASSERT(links[index].parent < index);

				const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(index).nbDof;

				deltaV = propagateAccelerationW(
					rw[index], deltaV,
					invStIS[index],
					&worldMotionMatrix[jointOffset], &isW[jointOffset],
					&deferredQstZ[jointOffset], dofCount, 
					NULL);
			}

			for (PxU32 idx = offset; idx < numElems1; ++idx)
			{
				const PxU32 index = pathToRoot1[idx];
				PX_ASSERT(links[index].parent < index);

				const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(index).nbDof;

				deltaV1 = propagateAccelerationW(
					rw[index], deltaV1, 
					invStIS[index], &worldMotionMatrix[jointOffset], 
					&isW[jointOffset], &deferredQstZ[jointOffset], dofCount, 
					NULL);
			}

			const Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV;

			v0 = Cm::SpatialVectorV(Cm::SpatialVector(vel.bottom, vel.top));

			const Cm::SpatialVectorF vel1 = mArticulationData.getMotionVelocity(linkID1) + deltaV1;

			v1 = Cm::SpatialVectorV(Cm::SpatialVector(vel1.bottom, vel1.top));
		}
	}

	/*Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocity(PxU32 linkID)
	{

	Cm::SpatialVectorF& vel = mArticulationData.getMotionVelocity(linkID);

	return Cm::SpatialVector(vel.bottom, vel.top);
	}*/

	//This is used in the solveExt1D, solveExtContact
	void FeatherstoneArticulation::pxcFsApplyImpulse(PxU32 linkID, aos::Vec3V linkImpulseLinear, aos::Vec3V linkImpulseAngular, const PxReal* jointImpulse)
	{
		const ArticulationLink* links = mArticulationData.mLinks;
		ArticulationData& data = mArticulationData;

		data.mJointDirty = true;

		//impulse is in world space
		Cm::SpatialVector impulse;
		V4StoreA(Vec4V_From_Vec3V(linkImpulseAngular), &impulse.angular.x);
		V4StoreA(Vec4V_From_Vec3V(linkImpulseLinear), &impulse.linear.x);
		Cm::SpatialVectorF Z0(-impulse.linear, -impulse.angular);

		PxReal* PX_RESTRICT deferredQstZ = data.getDeferredQstZ();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = data.getISInvStIS();
		Cm::SpatialVectorF* PX_RESTRICT solverLinkSpatialImpulses = data.getSolverLinkSpatialImpulses();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			const PxU32 jointOffset = data.getJointData(i).jointOffset;
			const PxU8 dofCount = data.getJointData(i).nbDof;

			solverLinkSpatialImpulses[i] += Z0;

			//Only apply the joint impulse to the inbound joint of linkID.
			const PxReal* jointImpulseToApply = (linkID == i) ? jointImpulse : NULL;
			
			Z0 = propagateImpulseW(
					rw[i],
					Z0, 
					jointImpulseToApply, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount, 
					&deferredQstZ[jointOffset]);
		}

		data.mRootDeferredZ += Z0;
	}

	void FeatherstoneArticulation::pxcFsApplyImpulses(
		PxU32 linkID1, const aos::Vec3V& linear1, const aos::Vec3V& angular1, const PxReal* jointImpulse1,
		PxU32 linkID2, const aos::Vec3V& linear2, const aos::Vec3V& angular2, const PxReal* jointImpulse2)
	{
		if (0)
		{
			pxcFsApplyImpulse(linkID1, linear1, angular1, jointImpulse1);
			pxcFsApplyImpulse(linkID2, linear2, angular2, jointImpulse2);
		}
		else
		{
			ArticulationData& data = mArticulationData;
			data.mJointDirty = true;
			ArticulationLink* links = mArticulationData.mLinks;
			//impulse is in world space
			Cm::SpatialVector impulse0;
			V3StoreU(angular1, impulse0.angular);
			V3StoreU(linear1, impulse0.linear);

			Cm::SpatialVector impulse1;
			V3StoreU(angular2, impulse1.angular);
			V3StoreU(linear2, impulse1.linear);

			Cm::SpatialVectorF Z1(-impulse0.linear, -impulse0.angular);
			Cm::SpatialVectorF Z2(-impulse1.linear, -impulse1.angular);

			ArticulationLink& link1 = links[linkID1];
			ArticulationLink& link2 = links[linkID2];
			
			const PxU32* pathToRoot1 = &data.mPathToRootElements[link1.mPathToRootStartIndex];
			const PxU32* pathToRoot2 = &data.mPathToRootElements[link2.mPathToRootStartIndex];

			const PxU32 numElems1 = link1.mPathToRootCount;
			const PxU32 numElems2 = link2.mPathToRootCount;

			//find the common link, work from one to that common, then the other to that common, then go from there upwards...
			PxU32 offset = 0;
			PxU32 commonLink = 0;
			while (pathToRoot1[offset] == pathToRoot2[offset])
			{
				commonLink = pathToRoot1[offset++];
				PX_ASSERT(links[commonLink].parent < commonLink);
				if (offset >= numElems1 || offset >= numElems2)
					break;
			}

			PxReal* PX_RESTRICT deferredQstZ = data.getDeferredQstZ();
			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
			const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = data.getISInvStIS();
			Cm::SpatialVectorF* PX_RESTRICT solverLinkSpatialImpulses = data.getSolverLinkSpatialImpulses();
			const PxVec3* PX_RESTRICT rw = data.getRw();

			//The common link will either be linkID2, or its ancestors.
			//The common link cannot be an index before either linkID2 or linkID
			for (PxU32 i = linkID2; i != commonLink; i = links[i].parent)
			{
				const PxU32 jointOffset = data.getJointData(i).jointOffset;
				const PxU8 dofCount = data.getJointData(i).nbDof;

				//Note: linkID2 might be the common link. If this is the case, we will only apply 
				//jointImpulse2 when we propagate from the common link to the root.
				//Watch out for that when we propagate from the common link.
				const PxReal* jointImpulseToApply = (linkID2 == i) ? jointImpulse2 : NULL;

				solverLinkSpatialImpulses[i] += Z2;

				Z2 = propagateImpulseW(
						rw[i],
						Z2,
						jointImpulseToApply, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount, 
						&deferredQstZ[jointOffset]);
			}

			for (PxU32 i = linkID1; i != commonLink; i = links[i].parent)
			{
				const PxU32 jointOffset = data.getJointData(i).jointOffset;
				const PxU8 dofCount = data.getJointData(i).nbDof;

				//Note: linkID1 might be the common link. If this is the case, we will only apply 
				//jointImpulse1 when we propagate from the common link to the root.
				//Watch out for that when we propagate from the common link.
				const PxReal* jointImpulseToApply = (linkID1 == i) ? jointImpulse1 : NULL;

				solverLinkSpatialImpulses[i] += Z1;

				Z1 = propagateImpulseW(
						rw[i],
						Z1, 
						jointImpulseToApply, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset],dofCount,
						&deferredQstZ[jointOffset]);
			}

			//If linkID1 (or linkID2) is the common link then we will not yet have applied 
			//jointImpulse1 (or jointImpulse2) to the inbound joint of the link.  
			//Work out how much joint impulse to apply to the inbound joint of the common link.
			PxReal jointImpulseToApplyAtCommonLink[3] = {0, 0, 0};
			if(((linkID1 == commonLink) && jointImpulse1) || ((linkID2 == commonLink) && jointImpulse2))
			{
				const PxU32 linkIndices[2] = {linkID1, linkID2};
				const PxReal* jointImpulses[2]= {jointImpulse1, jointImpulse2};
				const PxU32 dofCountAtCommonLink = data.getJointData(commonLink).nbDof;
				for(PxU32 k = 0; k < 2; k++)
				{
					const PxU32 linkId = linkIndices[k];
					const PxReal* jointImpulse = jointImpulses[k];
					if((linkId == commonLink) && jointImpulse)
					{		
						for(PxU32 i = 0; i < dofCountAtCommonLink; i++)
						{
							jointImpulseToApplyAtCommonLink[i] += jointImpulse[i];
						}
					}
				}
			}

			Cm::SpatialVectorF ZCommon = Z1 + Z2;

			for (PxU32 i = commonLink; i; i = links[i].parent)
			{
				const PxU32 jointOffset = data.getJointData(i).jointOffset;
				const PxU8 dofCount = data.getJointData(i).nbDof;

				//Only apply a joint impulse to the inbound joint of commonLink.
				//The joint impulse to apply to the inbound joint will only be non-zero if the common link 
				//is either linkID1 or linkID2.
				const PxReal* jointImpulseToApply = (commonLink == i) ? jointImpulseToApplyAtCommonLink : NULL;

				solverLinkSpatialImpulses[i] += ZCommon;

				ZCommon = propagateImpulseW(
						rw[i],
						ZCommon, 
						jointImpulseToApply, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount,
						&deferredQstZ[jointOffset]);
			}

			data.mRootDeferredZ += ZCommon;
		}
	}

	//This version uses in updateBodies
	PxQuat computeSphericalJointPositions(const PxQuat& relativeQuat,
		const PxQuat& newRot, const PxQuat& pBody2WorldRot,
		PxReal* jPositions, const Cm::UnAlignedSpatialVector* motionMatrix,
		const PxU32 dofs);

	PxQuat computeSphericalJointPositions(const PxQuat& relativeQuat,
		const PxQuat& newRot, const PxQuat& pBody2WorldRot);

	PxTransform FeatherstoneArticulation::propagateTransform(const PxU32 linkID, const ArticulationLink* links,
		const ArticulationJointCoreData& jointDatum, const Cm::SpatialVectorF* motionVelocities, const PxReal dt, const PxTransform& pBody2World, 
		const PxTransform& currentTransform, PxReal* jointVelocities, PxReal* jointPositions,
		const Cm::UnAlignedSpatialVector* motionMatrix)
	{
		const ArticulationLink& link = links[linkID];

		const PxQuat* PX_RESTRICT relativeQuats = mArticulationData.getRelativeQuat();

		const PxQuat& relativeQuat = relativeQuats[linkID];
		
		ArticulationJointCore* joint = link.inboundJoint;
		
		PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
		PxReal* jPosition = &jointPositions[jointDatum.jointOffset];
		
		PxQuat newParentToChild;
		PxQuat newWorldQ;
		PxVec3 r;
		
		const PxVec3 childOffset = -joint->childPose.p;
		const PxVec3 parentOffset = joint->parentPose.p;
		
		switch (joint->jointType)
		{
		case PxArticulationJointType::ePRISMATIC:
		{
			const PxReal tJointPosition = jPosition[0] + (jVelocity[0]) * dt;

			jPosition[0] = tJointPosition;

			newParentToChild = relativeQuat;
			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;
		
			r = e + d + motionMatrix[0].bottom * tJointPosition;
			break;
		}
		case PxArticulationJointType::eREVOLUTE:
		case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
		{
			const PxReal tJointPosition = jPosition[0] + (jVelocity[0]) * dt;

			jPosition[0] = tJointPosition;
		
			const PxVec3& u = motionMatrix[0].top;
		
			PxQuat jointRotation = PxQuat(-tJointPosition, u);
			if (jointRotation.w < 0)	//shortest angle.
				jointRotation = -jointRotation;
		
			newParentToChild = (jointRotation * relativeQuat).getNormalized();
		
			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;
			r = e + d;
		
			PX_ASSERT(r.isFinite());
		
			break;
		}
		case PxArticulationJointType::eSPHERICAL:
		{
			Cm::SpatialVectorF worldVel = motionVelocities[linkID];

			const PxTransform oldTransform = currentTransform;

			PxVec3 worldAngVel = worldVel.top;
			//PxVec3 worldAngVel = motionVelocities[linkID].top;

			PxReal dist = worldAngVel.normalize() * dt;

			if (dist > 1e-6f)
				newWorldQ = PxQuat(dist, worldAngVel) * oldTransform.q;
			else
				newWorldQ = oldTransform.q;

			//newWorldQ = Ps::exp(worldAngVel*dt) * oldTransform.q;

			//PxVec3 axis;

			newParentToChild = computeSphericalJointPositions(relativeQuats[linkID], newWorldQ,
				pBody2World.q);

			PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();

			if(jointRotation.w < 0.0f)
				jointRotation = -jointRotation;

			/*PxVec3 axis = jointRotation.getImaginaryPart();
			for (PxU32 i = 0; i < jointDatum.dof; ++i)
			{
				PxVec3 sa = mArticulationData.getMotionMatrix(jointDatum.jointOffset + i).top;
				PxReal angle = -compAng(sa.dot(axis), jointRotation.w);
				jPosition[i] = angle;
			}*/

			PxVec3 axis; PxReal angle;
			jointRotation.toRadiansAndUnitAxis(angle, axis);
			axis *= angle;
			for (PxU32 i = 0; i < jointDatum.nbDof; ++i)
			{
				PxVec3 sa = mArticulationData.getMotionMatrix(jointDatum.jointOffset + i).top;
				PxReal ang = -sa.dot(axis);
				jPosition[i] = ang;
			}
			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;
			r = e + d;

			PX_ASSERT(r.isFinite());
		
			break;
		}
		case PxArticulationJointType::eFIX:
		{
			//this is fix joint so joint don't have velocity
			newParentToChild = relativeQuat;
		
			const PxVec3 e = newParentToChild.rotate(parentOffset);
			const PxVec3 d = childOffset;
		
			r = e + d;
			break;
		}
		default:
			break;
		}
		
		PxTransform cBody2World;
		cBody2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
		cBody2World.p = pBody2World.p + cBody2World.q.rotate(r);
		
		PX_ASSERT(cBody2World.isSane());
		
		return cBody2World;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocityW(
		const PxVec3& parentToChild, const Cm::SpatialVectorF& parentLinkDeltaV,
		const Dy::SpatialMatrix& spatialInertia, const Cm::SpatialVectorF& Z,
		const PxReal* jointDofImpulses, const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrixW, const PxU32 dofCount, 
		PxReal* jointVelocity)
	{
		const Cm::SpatialVectorF pDeltaV = translateSpatialVector(-parentToChild, parentLinkDeltaV); //parent velocity change

		// -s_i^T * [I_i^A * translated(vParent) + Z]
		const Cm::SpatialVectorF temp = spatialInertia * pDeltaV + Z;
		PxReal tJointDelta[6];
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrixW[ind];
		    const PxReal jointDofImpulse = jointDofImpulses ? jointDofImpulses[ind] : 0.0f;
			tJointDelta[ind] = jointDofImpulse - sa.innerProduct(temp);
		}

		//qdot = [1/ s_i^T * i_i^A * s_i] * { -[s_i^T * I_i^A * translated(vParent) + Z]}
		//deltaV = qdot * s^T_i
		Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			PxReal jDelta = 0.f;
			for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			{
				jDelta += invStIs.invStIs[ind2][ind] * tJointDelta[ind2];
			}

			if(jointVelocity)
				jointVelocity[ind] += jDelta;
			jointSpatialDeltaV.top += motionMatrixW[ind].top * jDelta;
			jointSpatialDeltaV.bottom += motionMatrixW[ind].bottom * jDelta;
		}

		//deltaV = translated(parentLinkDeltaV) + qdot * s^T_i
		return pDeltaV + jointSpatialDeltaV;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getDeltaVWithDeltaJV(const bool fixBase, const PxU32 linkID, 
		const ArticulationData& data, Cm::SpatialVectorF* Z,
		PxReal* jointVelocities)
	{
		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF::Zero();
		if (!fixBase)
		{
			//velocity change
			//SpatialMatrix inverseArticulatedInertia = hLinkDatum.spatialArticulatedInertia.getInverse();
			const SpatialMatrix& inverseArticulatedInertia = data.mBaseInvSpatialArticulatedInertiaW;
			deltaV = inverseArticulatedInertia * (-Z[0]);
		}

		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const ArticulationLink& link = links[linkID];
		const PxU32* PX_RESTRICT pathToRoot = &data.mPathToRootElements[link.mPathToRootStartIndex];
		const PxU32 numElems = link.mPathToRootCount;

		const SpatialMatrix* PX_RESTRICT worldSpatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();
		const InvStIs* PX_RESTRICT invStIS = data.getInvStIS();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 i = 0; i < numElems; ++i)
		{
			const PxU32 index = pathToRoot[i];
			PX_ASSERT(links[index].parent < index);
			ArticulationJointCoreData& tJointDatum = data.getJointData(index);
			PxReal* jVelocity = &jointVelocities[tJointDatum.jointOffset];
			deltaV = FeatherstoneArticulation::propagateVelocityW(
				rw[index], deltaV,
				worldSpatialArticulatedInertia[index], Z[index], 
				NULL, invStIS[index], &worldMotionMatrix[tJointDatum.jointOffset], tJointDatum.nbDof,
				jVelocity);
		}

		return deltaV;
	}

	void FeatherstoneArticulation::getZ(const PxU32 linkID,
		const ArticulationData& data, Cm::SpatialVectorF* Z, 
		const Cm::SpatialVectorF& impulse)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = data.getISInvStIS();
		const PxVec3* PX_RESTRICT rw = data.getRw();

		//impulse need to be in linkID space!!!
		Z[linkID] = -impulse;

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			const ArticulationLink& tLink = links[i];
			const PxU32 jointOffset = data.getJointData(i).jointOffset;
			const PxU8 dofCount = data.getJointData(i).nbDof;
			Z[tLink.parent] = propagateImpulseW(
				rw[i],
				Z[i], 
				NULL, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount,
				NULL);
		}
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getImpulseResponseW(
		const PxU32 linkID,
		const ArticulationData& data,
		const Cm::SpatialVectorF& impulse)
	{
		return data.getImpulseResponseMatrixWorld()[linkID].getLinkDeltaVImpulseResponse(impulse);
	}

	//This method use in impulse self response. The input impulse is in the link space
	Cm::SpatialVectorF FeatherstoneArticulation::getImpulseResponseWithJ(
		const PxU32 linkID,
		const bool fixBase,
		const ArticulationData& data,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVectorF& impulse,
		PxReal* jointVelocites)
	{
		getZ(linkID, data, Z, impulse);

		return getDeltaVWithDeltaJV(fixBase, linkID, data, Z, jointVelocites);
	}

	void FeatherstoneArticulation::saveVelocity(FeatherstoneArticulation* articulation, Cm::SpatialVectorF* deltaV)
	{
		ArticulationData& data = articulation->mArticulationData;

		//update all links' motion velocity, joint delta velocity if there are contacts/constraints
		if (data.mJointDirty)
		{
			PxcFsFlushVelocity(*articulation, deltaV);
		}

		const PxU32 linkCount = data.getLinkCount();
		//copy motion velocites
		const Cm::SpatialVectorF* vels = data.getMotionVelocities();
		Cm::SpatialVectorF* posVels = data.getPosIterMotionVelocities();
		PxMemCopy(posVels, vels, sizeof(Cm::SpatialVectorF) * linkCount);
	
		//copy joint velocities
		const PxU32 dofs = data.getDofs();

		PxReal* jPosVels = data.getPosIterJointVelocities();

		const PxReal* jNewVels = data.getJointNewVelocities();

		PxMemCopy(jPosVels, jNewVels, sizeof(PxReal) * dofs);

		articulation->concludeInternalConstraints(false);
	}

	void FeatherstoneArticulation::saveVelocityTGS(FeatherstoneArticulation* articulation, PxReal invDtF32)
	{
		ArticulationData& data = articulation->mArticulationData;

		//KS - we should not need to flush velocity because we will have already stepped the articulation with TGS

		const PxU32 linkCount = data.getLinkCount();
		Cm::SpatialVectorF* posVels = data.getPosIterMotionVelocities();
		for (PxU32 i = 0; i < linkCount; ++i)
		{
			posVels[i] = posVels[i] * invDtF32;
		}
	}

	void FeatherstoneArticulation::getImpulseResponseSlow(const Dy::ArticulationLink* links, ArticulationData& data,
		PxU32 linkID0_, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0,
		PxU32 linkID1_, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1)
	{
		const PxU32 linkCount = data.getLinkCount();

		PX_ALLOCA(_stack, PxU32, linkCount);
		PxU32* stack = _stack;

		PxU32 i0, i1;//, ic;

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

		PxReal qstZ[192];

		PxMemZero(qstZ, data.getDofs() * sizeof(PxReal));

		//Z[linkID0] = Z0;
		//Z[linkID1] = Z1;

		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (i0 = 0; linkID0 != common; linkID0 = links[linkID0].parent)
		{
			const PxU32 jointOffset = data.getJointData(linkID0).jointOffset;
			const PxU8 dofCount = data.getJointData(linkID0).nbDof;
			Z0 = propagateImpulseW(
				rw[linkID0],
				Z0, 
				NULL, &data.getWorldIsInvD(jointOffset), &data.getWorldMotionMatrix(jointOffset), dofCount, 
				&qstZ[jointOffset]);
			stack[i0++] = linkID0;
		}

		for (i1 = i0; linkID1 != common; linkID1 = links[linkID1].parent)
		{
			const PxU32 jointOffset = data.getJointData(linkID1).jointOffset;
			const PxU8 dofCount = data.getJointData(linkID1).nbDof;
			Z1 = propagateImpulseW(
				rw[linkID1],
				Z1, 
				NULL, &data.getWorldIsInvD(jointOffset), &data.getWorldMotionMatrix(jointOffset), dofCount,
				&qstZ[jointOffset]);
			stack[i1++] = linkID1;
		}

		Cm::SpatialVectorF ZZ = Z0 + Z1;
		
		Cm::SpatialVectorF v = data.getImpulseResponseMatrixWorld()[common].getLinkDeltaVImpulseResponse(-ZZ); 
	
		Cm::SpatialVectorF dv1 = v;

		const InvStIs* PX_RESTRICT invStIS = data.getInvStIS();
		const Cm::SpatialVectorF* PX_RESTRICT isW = data.getIsW();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();

		for (PxU32 index = i1; (index--) > i0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = data.getJointData(id).jointOffset;
			const PxU32 dofCount = data.getJointData(id).nbDof;
			dv1 = propagateAccelerationW(
				rw[id], dv1, 
				invStIS[id], &worldMotionMatrix[jointOffset], &isW[jointOffset], &qstZ[jointOffset], dofCount, 
				NULL);
		}

		Cm::SpatialVectorF dv0 = v;
		for (PxU32 index = i0; (index--) > 0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = data.getJointData(id).jointOffset;
			const PxU32 dofCount = data.getJointData(id).nbDof;
			dv0 = propagateAccelerationW(
				rw[id], dv0, invStIS[id],
				&worldMotionMatrix[jointOffset], &isW[jointOffset], &qstZ[jointOffset], dofCount, 
				NULL);
		}

		deltaV0.linear = dv0.bottom;
		deltaV0.angular = dv0.top;

		deltaV1.linear = dv1.bottom;
		deltaV1.angular = dv1.top;
	}

	void FeatherstoneArticulation::getImpulseSelfResponse(const ArticulationLink* links, ArticulationData& data,
		PxU32 linkID0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0,
		PxU32 linkID1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1)
	{
		const ArticulationLink& link = links[linkID1];

		if (link.parent == linkID0)
		{
			PX_ASSERT(linkID0 == link.parent);
			PX_ASSERT(linkID0 < linkID1);

			//impulse is in world space
			const Cm::SpatialVectorF imp1(impulse1.linear, impulse1.angular);
			const Cm::SpatialVectorF imp0(impulse0.linear, impulse0.angular);

			Cm::SpatialVectorF Z1W(-imp1.top, -imp1.bottom);
			
			const PxU32 jointOffset1 = data.getJointData(linkID1).jointOffset;
			const PxU8 dofCount1 = data.getJointData(linkID1).nbDof;

			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
			const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = data.getISInvStIS();
			const PxVec3* PX_RESTRICT rw = data.getRw();

			PxReal qstZ[3] = { 0.f, 0.f, 0.f };
			const Cm::SpatialVectorF Z0W = propagateImpulseW(
				rw[linkID1],
				Z1W, 
				NULL, &iSInvStIS[jointOffset1], &worldMotionMatrix[jointOffset1], dofCount1, 
				qstZ);
			
			//in parent space
			const Cm::SpatialVectorF impulseDifW = imp0 - Z0W;
			
			//calculate velocity change start from the parent link to the root
			const Cm::SpatialVectorF delV0W = FeatherstoneArticulation::getImpulseResponseW(linkID0, data, impulseDifW);
			
			const InvStIs* PX_RESTRICT invStIS = data.getInvStIS();
			const Cm::SpatialVectorF* PX_RESTRICT isW = data.getIsW();

			const Cm::SpatialVectorF delV1W = propagateAccelerationW(
				rw[linkID1], delV0W, 
				invStIS[linkID1], &worldMotionMatrix[jointOffset1], &isW[jointOffset1], qstZ, dofCount1, 
				NULL);
			
			deltaV0.linear = delV0W.bottom;
			deltaV0.angular = delV0W.top;
			deltaV1.linear = delV1W.bottom;
			deltaV1.angular = delV1W.top;
		}
		else
		{
			getImpulseResponseSlow(	links, data,
									linkID0, impulse0, deltaV0,
									linkID1, impulse1, deltaV1);
		}
	}

	struct ArticulationStaticConstraintSortPredicate
	{
		bool operator()(const PxSolverConstraintDesc& left, const PxSolverConstraintDesc& right) const
		{
			PxU32 linkIndexA = left.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? left.linkIndexA : left.linkIndexB;
			PxU32 linkIndexB = right.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? right.linkIndexA : right.linkIndexB;

			return linkIndexA < linkIndexB;
		}
	};

	bool createFinalizeSolverContactsStep(PxTGSSolverContactDesc& contactDesc,
		PxsContactManagerOutput& output,
		ThreadContext& threadContext,
		const PxReal invDtF32,
		const PxReal invTotalDt,
		const PxReal totalDt,
		const PxReal stepDt,
		const PxReal bounceThresholdF32,
		const PxReal frictionOffsetThreshold,
		const PxReal correlationDistance,
		const PxReal biasCoefficient,
		PxConstraintAllocator& constraintAllocator);

	void FeatherstoneArticulation::prepareStaticConstraintsTGS(const PxReal stepDt, const PxReal totalDt, const PxReal invStepDt, const PxReal invTotalDt, 
		PxsContactManagerOutputIterator& outputs, Dy::ThreadContext& threadContext, PxReal correlationDist, PxReal bounceThreshold, PxReal frictionOffsetThreshold,
		PxTGSSolverBodyData* solverBodyData, PxTGSSolverBodyTxInertia* txInertia, PxsConstraintBlockManager& blockManager,
		Dy::ConstraintWriteback* constraintWritebackPool, const PxReal rigidContactBiasCoefficient, const PxReal jointBiasCoefficient, const PxReal lengthScale)
	{
		BlockAllocator blockAllocator(blockManager, threadContext.mConstraintBlockStream, threadContext.mFrictionPatchStreamPair, threadContext.mConstraintSize);

		const PxTransform id(PxIdentity);

		PxSort<PxSolverConstraintDesc, ArticulationStaticConstraintSortPredicate>(mStatic1DConstraints.begin(), mStatic1DConstraints.size(), ArticulationStaticConstraintSortPredicate());
		PxSort<PxSolverConstraintDesc, ArticulationStaticConstraintSortPredicate>(mStaticContactConstraints.begin(), mStaticContactConstraints.size(), ArticulationStaticConstraintSortPredicate());

		for (PxU32 i = 0; i < mStatic1DConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStatic1DConstraints[i];

			PxU32 linkIndex = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? desc.linkIndexA : desc.linkIndexB;
			
			PX_ASSERT(desc.constraintType == DY_SC_TYPE_RB_1D);
			const Constraint* constraint = reinterpret_cast<const Constraint*>(desc.constraint);

			SolverConstraintShaderPrepDesc shaderPrepDesc;
			PxTGSSolverConstraintPrepDesc prepDesc;

			const PxConstraintSolverPrep solverPrep = constraint->solverPrep;
			const void* constantBlock = constraint->constantBlock;
			const PxU32 constantBlockByteSize = constraint->constantBlockSize;
			const PxTransform& pose0 = (constraint->body0 ? constraint->body0->getPose() : id);
			const PxTransform& pose1 = (constraint->body1 ? constraint->body1->getPose() : id);
			const PxTGSSolverBodyVel* sbody0 = desc.tgsBodyA;
			const PxTGSSolverBodyVel* sbody1 = desc.tgsBodyB;
			PxTGSSolverBodyData* sbodyData0 = &solverBodyData[desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? 0 : desc.bodyADataIndex];
			PxTGSSolverBodyData* sbodyData1 = &solverBodyData[desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? 0 : desc.bodyBDataIndex];
			PxTGSSolverBodyTxInertia& txI0 = txInertia[desc.bodyADataIndex];
			PxTGSSolverBodyTxInertia& txI1 = txInertia[desc.bodyBDataIndex];

			shaderPrepDesc.constantBlock = constantBlock;
			shaderPrepDesc.constantBlockByteSize = constantBlockByteSize;
			shaderPrepDesc.constraint = constraint;
			shaderPrepDesc.solverPrep = solverPrep;

			prepDesc.desc = &desc;
			prepDesc.bodyFrame0 = pose0;
			prepDesc.bodyFrame1 = pose1;
			prepDesc.body0 = sbody0;
			prepDesc.body1 = sbody1;
			prepDesc.body0TxI = &txI0;
			prepDesc.body1TxI = &txI1;
			prepDesc.bodyData0 = sbodyData0;
			prepDesc.bodyData1 = sbodyData1;
			prepDesc.linBreakForce = constraint->linBreakForce;
			prepDesc.angBreakForce = constraint->angBreakForce;
			prepDesc.writeback = &constraintWritebackPool[constraint->index];
			setupConstraintFlags(prepDesc, constraint->flags);
			prepDesc.minResponseThreshold = constraint->minResponseThreshold;

			prepDesc.bodyState0 = desc.linkIndexA == PxSolverConstraintDesc::RIGID_BODY ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eARTICULATION;
			prepDesc.bodyState1 = desc.linkIndexB == PxSolverConstraintDesc::RIGID_BODY ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eARTICULATION;

			SetupSolverConstraintStep(shaderPrepDesc, prepDesc, blockAllocator, stepDt, totalDt, invStepDt, invTotalDt, lengthScale, jointBiasCoefficient);

			if (desc.constraint)
			{
				if (mArticulationData.mNbStatic1DConstraints[linkIndex] == 0)
					mArticulationData.mStatic1DConstraintStartIndex[linkIndex] = i;
				mArticulationData.mNbStatic1DConstraints[linkIndex]++;
			}
			else
			{
				//Shuffle down 
				mStatic1DConstraints.remove(i);
				i--;
			}
		}

		for (PxU32 i = 0; i < mStaticContactConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStaticContactConstraints[i];

			PxU32 linkIndex = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? desc.linkIndexA : desc.linkIndexB;

			PX_ASSERT(desc.constraintType == DY_SC_TYPE_RB_CONTACT);
			
			PxTGSSolverContactDesc blockDesc;
			PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);
			PxcNpWorkUnit& unit = cm->getWorkUnit();
			PxsContactManagerOutput* cmOutput = &outputs.getContactManagerOutput(unit.mNpIndex);

			PxTGSSolverBodyVel& b0 = *desc.tgsBodyA;
			PxTGSSolverBodyVel& b1 = *desc.tgsBodyB;

			PxTGSSolverBodyData& data0 = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyADataIndex];
			PxTGSSolverBodyData& data1 = desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyBDataIndex];

			PxTGSSolverBodyTxInertia& txI0 = txInertia[desc.bodyADataIndex];
			PxTGSSolverBodyTxInertia& txI1 = txInertia[desc.bodyBDataIndex];

			blockDesc.bodyFrame0 = unit.mRigidCore0->body2World;
			blockDesc.bodyFrame1 = unit.mRigidCore1->body2World;
			blockDesc.shapeInteraction = cm->getShapeInteraction();
			blockDesc.contactForces = cmOutput->contactForces;
			blockDesc.desc = &desc;
			blockDesc.body0 = &b0;
			blockDesc.body1 = &b1;
			blockDesc.body0TxI = &txI0;
			blockDesc.body1TxI = &txI1;
			blockDesc.bodyData0 = &data0;
			blockDesc.bodyData1 = &data1;
			blockDesc.hasForceThresholds = !!(unit.mFlags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);
			blockDesc.disableStrongFriction = !!(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION);
			blockDesc.bodyState0 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? PxSolverContactDesc::eARTICULATION : PxSolverContactDesc::eDYNAMIC_BODY;
			blockDesc.bodyState1 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? PxSolverContactDesc::eARTICULATION : (unit.mFlags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR) ? PxSolverContactDesc::eKINEMATIC_BODY :
				((unit.mFlags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1) ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eSTATIC_BODY);
			//blockDesc.flags = unit.flags;

			const PxReal maxImpulse0 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? static_cast<const PxsBodyCore*>(unit.mRigidCore0)->maxContactImpulse : data0.maxContactImpulse;
			const PxReal maxImpulse1 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? static_cast<const PxsBodyCore*>(unit.mRigidCore1)->maxContactImpulse : data1.maxContactImpulse;

			unit.setInvMassScaleFromDominance(blockDesc.invMassScales);

			blockDesc.restDistance = unit.mRestDistance;
			blockDesc.frictionPtr = unit.mFrictionDataPtr;
			blockDesc.frictionCount = unit.mFrictionPatchCount;
			blockDesc.maxCCDSeparation = PX_MAX_F32;
			blockDesc.maxImpulse = PxMin(maxImpulse0, maxImpulse1);
			blockDesc.torsionalPatchRadius = unit.mTorsionalPatchRadius;
			blockDesc.minTorsionalPatchRadius = unit.mMinTorsionalPatchRadius;
			blockDesc.offsetSlop = unit.mOffsetSlop;

			createFinalizeSolverContactsStep(blockDesc, *cmOutput, threadContext,
				invStepDt, invTotalDt, totalDt, stepDt, bounceThreshold, frictionOffsetThreshold, 
				correlationDist, rigidContactBiasCoefficient, blockAllocator);

			getContactManagerConstraintDesc(*cmOutput, *cm, desc);

			updateFrictionAnchorCountAndPosition(desc, *cmOutput, blockDesc);

			unit.mFrictionDataPtr = blockDesc.frictionPtr;
			unit.mFrictionPatchCount = blockDesc.frictionCount;
			//KS - Don't track this for now!
			//axisConstraintCount += blockDesc.axisConstraintCount;

			if (desc.constraint)
			{
				if (mArticulationData.mNbStaticContactConstraints[linkIndex] == 0)
					mArticulationData.mStaticContactConstraintStartIndex[linkIndex] = i;
				mArticulationData.mNbStaticContactConstraints[linkIndex]++;
			}
			else
			{
				//Shuffle down 
				mStaticContactConstraints.remove(i);
				i--;
			}
		}
	}

	void FeatherstoneArticulation::prepareStaticConstraints(const PxReal dt, const PxReal invDt, PxsContactManagerOutputIterator& outputs,
		Dy::ThreadContext& threadContext, PxReal correlationDist, PxReal bounceThreshold, PxReal frictionOffsetThreshold,
		PxReal ccdMaxSeparation, const PxReal rigidContactBiasCoefficient, const PxReal jointBiasCoefficient,
		PxSolverBodyData* solverBodyData, PxsConstraintBlockManager& blockManager,
		Dy::ConstraintWriteback* constraintWritebackPool)
	{
		BlockAllocator blockAllocator(blockManager, threadContext.mConstraintBlockStream, threadContext.mFrictionPatchStreamPair, threadContext.mConstraintSize);

		const PxTransform id(PxIdentity);

		Cm::SpatialVectorF* Z = threadContext.mZVector.begin();

		PxSort<PxSolverConstraintDesc, ArticulationStaticConstraintSortPredicate>(mStatic1DConstraints.begin(), mStatic1DConstraints.size(), ArticulationStaticConstraintSortPredicate());
		PxSort<PxSolverConstraintDesc, ArticulationStaticConstraintSortPredicate>(mStaticContactConstraints.begin(), mStaticContactConstraints.size(), ArticulationStaticConstraintSortPredicate());

		for (PxU32 i = 0; i < mStatic1DConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStatic1DConstraints[i];

			PxU32 linkIndex = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? desc.linkIndexA : desc.linkIndexB;

			PX_ASSERT(desc.constraintType == DY_SC_TYPE_RB_1D);
			const Constraint* constraint = reinterpret_cast<const Constraint*>(desc.constraint);

			SolverConstraintShaderPrepDesc shaderPrepDesc;
			PxSolverConstraintPrepDesc prepDesc;

			const PxConstraintSolverPrep solverPrep = constraint->solverPrep;
			const void* constantBlock = constraint->constantBlock;
			const PxU32 constantBlockByteSize = constraint->constantBlockSize;
			const PxTransform& pose0 = (constraint->body0 ? constraint->body0->getPose() : id);
			const PxTransform& pose1 = (constraint->body1 ? constraint->body1->getPose() : id);
			const PxSolverBody* sbody0 = desc.bodyA;
			const PxSolverBody* sbody1 = desc.bodyB;
			PxSolverBodyData* sbodyData0 = &solverBodyData[desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? 0 : desc.bodyADataIndex];
			PxSolverBodyData* sbodyData1 = &solverBodyData[desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? 0 : desc.bodyBDataIndex];

			shaderPrepDesc.constantBlock = constantBlock;
			shaderPrepDesc.constantBlockByteSize = constantBlockByteSize;
			shaderPrepDesc.constraint = constraint;
			shaderPrepDesc.solverPrep = solverPrep;

			prepDesc.desc = &desc;
			prepDesc.bodyFrame0 = pose0;
			prepDesc.bodyFrame1 = pose1;
			prepDesc.data0 = sbodyData0;
			prepDesc.data1 = sbodyData1;
			prepDesc.body0 = sbody0;
			prepDesc.body1 = sbody1;
			prepDesc.linBreakForce = constraint->linBreakForce;
			prepDesc.angBreakForce = constraint->angBreakForce;
			prepDesc.writeback = &constraintWritebackPool[constraint->index];
			setupConstraintFlags(prepDesc, constraint->flags);
			prepDesc.minResponseThreshold = constraint->minResponseThreshold;

			SetupSolverConstraint(shaderPrepDesc, prepDesc, blockAllocator, dt, invDt, jointBiasCoefficient);

			if (desc.constraint)
			{
				if (mArticulationData.mNbStatic1DConstraints[linkIndex] == 0)
					mArticulationData.mStatic1DConstraintStartIndex[linkIndex] = i;
				mArticulationData.mNbStatic1DConstraints[linkIndex]++;
			}
			else
			{
				//Shuffle down 
				mStatic1DConstraints.remove(i);
				i--;
			}
		}

		for (PxU32 i = 0; i < mStaticContactConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStaticContactConstraints[i];

			PxU32 linkIndex = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? desc.linkIndexA : desc.linkIndexB;

			PX_ASSERT(desc.constraintType == DY_SC_TYPE_RB_CONTACT);
			
			PxSolverContactDesc blockDesc;
			PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);
			PxcNpWorkUnit& unit = cm->getWorkUnit();
			PxsContactManagerOutput* cmOutput = &outputs.getContactManagerOutput(unit.mNpIndex);

			PxSolverBodyData& data0 = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyADataIndex];
			PxSolverBodyData& data1 = desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyBDataIndex];

			blockDesc.data0 = &data0;
			blockDesc.data1 = &data1;

			PxU8 flags = unit.mRigidCore0->mFlags;
			if (unit.mRigidCore1)
				flags |= PxU8(unit.mRigidCore1->mFlags);

			blockDesc.bodyFrame0 = unit.mRigidCore0->body2World;
			blockDesc.bodyFrame1 = unit.mRigidCore1 ? unit.mRigidCore1->body2World : id;
			blockDesc.shapeInteraction = cm->getShapeInteraction();
			blockDesc.contactForces = cmOutput->contactForces;
			blockDesc.desc = &desc;
			blockDesc.body0 = desc.bodyA;
			blockDesc.body1 = desc.bodyB;
			blockDesc.hasForceThresholds = !!(unit.mFlags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);
			blockDesc.disableStrongFriction = !!(unit.mFlags & PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION);
			blockDesc.bodyState0 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? PxSolverContactDesc::eARTICULATION : PxSolverContactDesc::eDYNAMIC_BODY;
			blockDesc.bodyState1 = (unit.mFlags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? PxSolverContactDesc::eARTICULATION : (unit.mFlags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR) ? PxSolverContactDesc::eKINEMATIC_BODY :
				((unit.mFlags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1) ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eSTATIC_BODY);
			//blockDesc.flags = unit.flags;

			unit.setInvMassScaleFromDominance(blockDesc.invMassScales);

			blockDesc.restDistance = unit.mRestDistance;
			blockDesc.frictionPtr = unit.mFrictionDataPtr;
			blockDesc.frictionCount = unit.mFrictionPatchCount;
			blockDesc.maxCCDSeparation = (flags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD) ? ccdMaxSeparation : PX_MAX_F32;
			blockDesc.offsetSlop = unit.mOffsetSlop;

			createFinalizeSolverContacts(blockDesc, *cmOutput, threadContext, invDt, dt, bounceThreshold,
				frictionOffsetThreshold, correlationDist, rigidContactBiasCoefficient, blockAllocator, Z);

			getContactManagerConstraintDesc(*cmOutput, *cm, desc);

			updateFrictionAnchorCountAndPosition(desc, *cmOutput, blockDesc);

			unit.mFrictionDataPtr = blockDesc.frictionPtr;
			unit.mFrictionPatchCount = blockDesc.frictionCount;
			//KS - Don't track this for now!
			//axisConstraintCount += blockDesc.axisConstraintCount;

			if (desc.constraint)
			{
				if (mArticulationData.mNbStaticContactConstraints[linkIndex] == 0)
					mArticulationData.mStaticContactConstraintStartIndex[linkIndex] = i;
				mArticulationData.mNbStaticContactConstraints[linkIndex]++;
			}
			else
			{
				mStaticContactConstraints.remove(i);
				i--;
			}
		}
	}

	static PX_FORCE_INLINE void setupDeltaV(ArticulationInternalConstraintBase& c, const Cm::SpatialVector& deltaVA, const Cm::SpatialVector& deltaVB)
	{
		c.deltaVA.top = deltaVA.angular;
		c.deltaVA.bottom = deltaVA.linear;
		c.deltaVB.top = deltaVB.angular;
		c.deltaVB.bottom = deltaVB.linear;
	}

	static void setupComplexLimit(const ArticulationLink* links, ArticulationData& data, const PxU32 linkID, 
		const PxReal angle, const PxReal lowLimit, const PxReal highLimit, const PxVec3& axis, const PxReal cfm, ArticulationInternalConstraint& complexConstraint,
		ArticulationInternalLimit& limit)
	{
		Cm::SpatialVector deltaVA, deltaVB;
		FeatherstoneArticulation::getImpulseSelfResponse(links, data,
			links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
			linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

		const PxReal r0 = deltaVA.angular.dot(axis);
		const PxReal r1 = deltaVB.angular.dot(axis);

		const PxReal unitResponse = r0 - r1;

		const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / (cfm + unitResponse) : 0.0f;

		complexConstraint.row0 = Cm::UnAlignedSpatialVector(PxVec3(0), axis);
		complexConstraint.row1 = Cm::UnAlignedSpatialVector(PxVec3(0), axis);
		setupDeltaV(complexConstraint, deltaVA, deltaVB);
		complexConstraint.recipResponse = recipResponse;
		complexConstraint.response = unitResponse;
		complexConstraint.isLinearConstraint = true;
		limit.errorLow = angle - lowLimit;
		limit.errorHigh = highLimit - angle;
		limit.lowImpulse = 0.f;
		limit.highImpulse = 0.f;
	}

	void FeatherstoneArticulation::setupInternalConstraintsRecursive(
		const ArticulationLink* links,
		ArticulationData& data,
		const PxReal stepDt,
		const PxReal dt,
		const bool isTGSSolver, 
		const PxU32 linkID,
		const PxReal maxForceScale
		)
	{
		const ArticulationLink& link = links[linkID];

		const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

		const ArticulationLink& pLink = links[link.parent];

		const ArticulationJointCore& j = *link.inboundJoint;

		//const bool jointDrive = (j.driveType != PxArticulationJointDriveType::eNONE);

		const bool hasDeprecatedFriction = j.frictionCoefficient > 0.f;

		const PxReal deprecatedFCoefficient = j.frictionCoefficient * stepDt;
		
		const PxReal transmissionForce = data.getTransmittedForce(linkID).magnitude() * deprecatedFCoefficient;

		// PT:: tag: scalar transform*transform
		const PxTransform cA2w = pLink.bodyCore->body2World.transform(j.parentPose);
		const PxTransform cB2w = link.bodyCore->body2World.transform(j.childPose);

		const PxU32 parent = link.parent;

		const PxReal cfm = PxMax(link.cfm, pLink.cfm);

		//Linear, then angular...

		PxVec3 driveError(0.f);
		PxVec3 angles(0.f);
		PxVec3 row[3];
		if (j.jointType == PxArticulationJointType::eSPHERICAL && jointDatum.nbDof > 1)
		{
			//It's a spherical joint. We can't directly work on joint positions with spherical joints, so we instead need to compute the quaternion
			//and from that compute the joint error projected onto the DOFs. This will yield a rotation that is singularity-free, where the joint
			//angles match the target joint angles provided the angles are within +/- Pi around each axis. Spherical joints do not support
			//quaternion double cover cases/wide angles.

			PxVec3 driveAxis(0.f);

			bool hasAngularDrives = false;

			PxU32 tmpDofId = 0;

			const PxReal* PX_RESTRICT jointTargetPositions = data.getJointTargetPositions();

			for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].driveType != PxArticulationDriveType::eNONE);

					if (hasDrive)
					{
						// The code further below relies on x,y,z of a PxVec3 to map to twist, swing1, swing2
						PX_COMPILE_TIME_ASSERT(PxArticulationAxis::eTWIST == 0);
						PX_COMPILE_TIME_ASSERT(PxArticulationAxis::eSWING1 == 1);
						PX_COMPILE_TIME_ASSERT(PxArticulationAxis::eSWING2 == 2);
						PX_ASSERT(i < 3);

						driveAxis[i] = jointTargetPositions[jointDatum.jointOffset + tmpDofId];

						hasAngularDrives = true;
					}

					tmpDofId++;
				}
			}
			
			{
				{
					//Spherical joint drive calculation using 3x child-space Euler angles
					if (hasAngularDrives)
					{
						PxQuat qB2qA = cA2w.q.getConjugate() * cB2w.q;

						PxReal angle = driveAxis.normalize();

						if (angle < 1e-12f)
						{
							driveAxis = PxVec3(1.f, 0.f, 0.f);
							angle = 0.f;
						}

						PxQuat targetQ = PxQuat(angle, driveAxis);
						// note: the targets are defined relative to the parent link joint frame, so this transform is targetQ2qA.

						if (targetQ.dot(qB2qA) < 0.f)
							targetQ = -targetQ;

						driveError = -2.f * (targetQ.getConjugate() * qB2qA).getImaginaryPart();
					}

					const PxReal* PX_RESTRICT jointPositions = data.getJointPositions();
					const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
					for (PxU32 i = 0, tmpDof = 0; i < PxArticulationAxis::eX; ++i)
					{
						if (j.motion[i] != PxArticulationMotion::eLOCKED)
						{
							angles[i] = jointPositions[jointDatum.jointOffset + tmpDof];
							row[i] = worldMotionMatrix[jointDatum.jointOffset + tmpDof].top;
							tmpDof++;
						}
					}
				}
			}
		}
		else
		{
			const PxReal* PX_RESTRICT jointPositions = data.getJointPositions();
			const PxReal* PX_RESTRICT jointTargetPositions = data.getJointTargetPositions();
			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
			for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					const float jPos = jointPositions[jointDatum.jointOffset];
					driveError[i] = jointTargetPositions[jointDatum.jointOffset] - jPos;
					angles[i] = jPos;
					row[i] = worldMotionMatrix[jointDatum.jointOffset].top;

					break;
				}
			}
		}

		PxU32 dofId = 0;

		for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
		{
			if (j.motion[i] != PxArticulationMotion::eLOCKED)
			{
				//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
				const PxVec3 axis = row[i];

				Cm::SpatialVector deltaVA, deltaVB;
				FeatherstoneArticulation::getImpulseSelfResponse(links, data,
					parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
					linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

				const PxReal r0 = deltaVA.angular.dot(axis);
				const PxReal r1 = deltaVB.angular.dot(axis);

				const PxReal unitResponse = r0 - r1;

				const PxReal recipResponse = unitResponse <= 0.f ? 0.f : 1.0f / (unitResponse+cfm);

				const PxU32 count = data.mInternalConstraints.size();
				data.mInternalConstraints.forceSize_Unsafe(count + 1);
				ArticulationInternalConstraint* constraints = &data.mInternalConstraints[count];

				constraints->recipResponse = recipResponse;
				constraints->response = unitResponse;
				constraints->row0 = Cm::SpatialVectorF(PxVec3(0), axis);
				constraints->row1 = Cm::SpatialVectorF(PxVec3(0), axis);
				setupDeltaV(*constraints, deltaVA, deltaVB);
				constraints->isLinearConstraint = false;
				constraints->maxJointVelocity = j.maxJointVelocity[i];

				constraints->accumulatedFrictionImpulse = 0.0f;
				constraints->deprecatedFrictionMaxForce = hasDeprecatedFriction ? transmissionForce : 0.f;

				constraints->dynamicFrictionEffort = j.frictionParams[i].dynamicFrictionEffort;
				constraints->staticFrictionEffort = j.frictionParams[i].staticFrictionEffort;
				constraints->viscousFrictionCoefficient = j.frictionParams[i].viscousFrictionCoefficient;

				const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].driveType != PxArticulationDriveType::eNONE);
				constraints->driveImpulse = 0.0f;
				constraints->driveMaxImpulse = j.drives[i].maxForce * maxForceScale;
				constraints->envelope = j.drives[i].envelope;
				constraints->externalJointForce = data.getJointForces()[jointDatum.jointOffset + dofId];
				
				if(hasDrive)
				{
					constraints->setImplicitDriveDesc(
						computeImplicitDriveParams(
							j.drives[i].driveType, j.drives[i].stiffness, j.drives[i].damping,
							isTGSSolver ? stepDt : dt, dt,
							unitResponse, recipResponse, driveError[i], data.getJointTargetVelocities()[jointDatum.jointOffset + dofId],
							isTGSSolver));
				}
				else
				{
					constraints->setImplicitDriveDesc(ArticulationImplicitDriveDesc(PxZero));
				}

				if (j.motion[i] == PxArticulationMotion::eLIMITED)
				{
					const PxU32 limitCount = data.mInternalLimits.size();
					data.mInternalLimits.forceSize_Unsafe(limitCount + 1);
					ArticulationInternalLimit* limits = &data.mInternalLimits[limitCount];

					const PxReal jPos = angles[i];
					limits->errorHigh = j.limits[i].high - jPos;
					limits->errorLow = jPos - j.limits[i].low;
					limits->lowImpulse = 0.f;
					limits->highImpulse = 0.f;
				}
				dofId++;
			}
		}

		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();
		for (PxU32 i = PxArticulationAxis::eX; i < PxArticulationAxis::eCOUNT; ++i)
		{
			if (j.motion[i] != PxArticulationMotion::eLOCKED)
			{
				//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
				const PxVec3 axis = worldMotionMatrix[jointDatum.jointOffset + dofId].bottom;
				const PxVec3 ang0 = (cA2w.p - pLink.bodyCore->body2World.p).cross(axis);
				const PxVec3 ang1 = (cB2w.p - link.bodyCore->body2World.p).cross(axis);

				Cm::SpatialVector deltaVA, deltaVB;
				FeatherstoneArticulation::getImpulseSelfResponse(links, data,
					links[linkID].parent, Cm::SpatialVector(axis, ang0), deltaVA,
					linkID, Cm::SpatialVector(-axis, -ang1), deltaVB);

				const PxReal r0 = deltaVA.linear.dot(axis) + deltaVA.angular.dot(ang0);
				const PxReal r1 = deltaVB.linear.dot(axis) + deltaVB.angular.dot(ang1);

				const PxReal unitResponse = r0 - r1;

				//const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / (unitResponse + cfm) : 0.0f;
				const PxReal recipResponse = 1.0f / (unitResponse + cfm);

				const PxU32 count = data.mInternalConstraints.size();
				data.mInternalConstraints.forceSize_Unsafe(count + 1);
				ArticulationInternalConstraint* constraints = &data.mInternalConstraints[count];

				constraints->response = unitResponse;
				constraints->recipResponse = recipResponse;
				constraints->row0 = Cm::SpatialVectorF(axis, ang0);
				constraints->row1 = Cm::SpatialVectorF(axis, ang1);
				setupDeltaV(*constraints, deltaVA, deltaVB);
				constraints->isLinearConstraint = true;
				constraints->maxJointVelocity = j.maxJointVelocity[i];

				constraints->accumulatedFrictionImpulse = 0.f;
				constraints->deprecatedFrictionMaxForce = hasDeprecatedFriction ? transmissionForce : 0.f;
				constraints->dynamicFrictionEffort = j.frictionParams[i].dynamicFrictionEffort;
				constraints->staticFrictionEffort = j.frictionParams[i].staticFrictionEffort;
				constraints->viscousFrictionCoefficient = j.frictionParams[i].viscousFrictionCoefficient;
												  
				const PxReal* jointPositions = data.getJointPositions();

				const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && (j.drives[i].envelope.maxEffort > 0.0f || j.drives[i].maxForce > 0.0f) && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));
				constraints->driveImpulse = 0.0f;
				constraints->envelope = j.drives[i].envelope;
				constraints->driveMaxImpulse = j.drives[i].maxForce * maxForceScale;
				constraints->externalJointForce = data.getJointForces()[jointDatum.jointOffset + dofId];
				if(hasDrive)
				{
					const PxU32 jOffset = jointDatum.jointOffset + dofId;
					constraints->setImplicitDriveDesc(
						computeImplicitDriveParams(
							j.drives[i].driveType, j.drives[i].stiffness, j.drives[i].damping,
							isTGSSolver ? stepDt : dt, dt,
							unitResponse, recipResponse,
							data.getJointTargetPositions()[jOffset] - jointPositions[jOffset],
							data.getJointTargetVelocities()[jOffset],
							isTGSSolver));
				}
				else
				{
					constraints->setImplicitDriveDesc(ArticulationImplicitDriveDesc(PxZero));
				}

				if (j.motion[i] == PxArticulationMotion::eLIMITED)
				{
					const PxU32 limitCount = data.mInternalLimits.size();
					data.mInternalLimits.forceSize_Unsafe(limitCount + 1);
					ArticulationInternalLimit* limits = &data.mInternalLimits[limitCount];
					const PxReal jPos = jointPositions[jointDatum.jointOffset + dofId];
					limits->errorHigh = j.limits[i].high - jPos;
					limits->errorLow = jPos - j.limits[i].low;
					limits->lowImpulse = 0.f;
					limits->highImpulse = 0.f;
				}
				dofId++;
			}
		}

		if (jointDatum.dofLimitMask)
		{
			for (PxU32 dof = 0; dof < jointDatum.nbDof; ++dof)
			{
				PxU32 i = j.dofIds[dof];
				if (j.motion[i] == PxArticulationMotion::eLOCKED)
				{
					const PxU32 count = data.mInternalConstraints.size();
					data.mInternalConstraints.forceSize_Unsafe(count + 1);
					ArticulationInternalConstraint* constraints = &data.mInternalConstraints[count];

					const PxU32 limitCount = data.mInternalLimits.size();
					data.mInternalLimits.forceSize_Unsafe(limitCount + 1);
					ArticulationInternalLimit* limits = &data.mInternalLimits[limitCount];

					const PxVec3 axis = row[i];

					PxReal angle = angles[i];

					//A locked axis is equivalent to a limit of 0
					PxReal low = 0.f;
					PxReal high = 0.f;

					setupComplexLimit(links, data, linkID, angle,
						low, high, axis, cfm, *constraints, *limits++);
				}
			}
		}

		const PxU32 numChildren = link.mNumChildren;
		const PxU32 offset = link.mChildrenStartIndex;
		for (PxU32 i = 0; i < numChildren; ++i)
		{
			const PxU32 child = offset + i;
			setupInternalConstraintsRecursive(links, data, stepDt, dt, isTGSSolver, child, maxForceScale);
		}
	}

	void FeatherstoneArticulation::setupInternalSpatialTendonConstraintsRecursive(
		const ArticulationLink* links,
		ArticulationAttachment* attachments,
		const PxU32 attachmentCount,
		const PxVec3& pAttachPoint,
		const bool fixBase,
		ArticulationData& data,
		const PxReal stepDt,
		const bool isTGSSolver,
		const PxU32 attachmentID,
		const PxReal stiffness,
		const PxReal damping,
		const PxReal limitStiffness,
		const PxReal accumLength,
		const PxU32 startLink,
		const PxVec3& startAxis,
		const PxVec3& startRaXn)
	{
		ArticulationAttachment& attachment = attachments[attachmentID];
		
		const ArticulationLink& cLink = links[attachment.linkInd];

		const PxTransform cBody2World = cLink.bodyCore->body2World;

		const PxVec3 rb = cBody2World.q.rotate(attachment.relativeOffset);

		const PxVec3 cAttachPoint = cBody2World.p + rb;

		const PxVec3 dif = pAttachPoint - cAttachPoint;
		const PxReal distanceSq = dif.magnitudeSquared();
		const PxReal distance = PxSqrt(distanceSq);

		const PxReal u = distance * attachment.coefficient + accumLength;		
	
		const PxU32 childCount = attachment.childCount;
		if (childCount)
		{		
			for (ArticulationBitField children = attachment.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);

				setupInternalSpatialTendonConstraintsRecursive(links, attachments, attachmentCount, cAttachPoint, fixBase, data, stepDt,
					isTGSSolver, child, stiffness, damping, limitStiffness, u, startLink,
					startAxis, startRaXn);
			}
		}
		else
		{
			const PxVec3 axis = distance > 0.001f ? dif / distance : PxVec3(0.f);

			const PxVec3 rbXn = rb.cross(axis);

			Cm::SpatialVector deltaVA, deltaVB;
			FeatherstoneArticulation::getImpulseSelfResponse(links, data,
				startLink, Cm::SpatialVector(startAxis, startRaXn), deltaVA,
				attachment.linkInd, Cm::SpatialVector(-axis, -rbXn), deltaVB);

			const PxReal r0 = deltaVA.linear.dot(startAxis) + deltaVA.angular.dot(startRaXn);
			const PxReal r1 = deltaVB.linear.dot(axis) + deltaVB.angular.dot(rbXn);

			const PxReal unitResponse = (r0 - r1);

			//const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / (unitResponse + cfm) : 0.0f;
			//const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / (unitResponse) : 0.0f;
			const PxReal recipResponse = 1.0f / (unitResponse + cLink.cfm);

			const PxU32 count = data.mInternalSpatialTendonConstraints.size();
			data.mInternalSpatialTendonConstraints.forceSize_Unsafe(count + 1);

			ArticulationInternalTendonConstraint* constraint = &data.mInternalSpatialTendonConstraints[count];
			attachment.mConstraintInd = PxU16(count);

			constraint->row0 = Cm::SpatialVectorF(startAxis, startRaXn);
			constraint->row1 = Cm::SpatialVectorF(axis, rbXn);
			constraint->linkID0 = startLink;
			constraint->linkID1 = attachment.linkInd;
			constraint->recipResponse = recipResponse;
			constraint->appliedForce = 0.f;
			constraint->limitAppliedForce = 0.f;

			constraint->setTendonImplicitSpringParams(
				computeTendonSpringParams
					(stepDt, isTGSSolver,
					 unitResponse,
					stiffness, damping,
					limitStiffness));

			constraint->accumulatedLength = u;// + u*0.2f;
			constraint->restDistance = attachment.restLength;
			constraint->lowLimit = attachment.lowLimit;
			constraint->highLimit = attachment.highLimit;
		}
	}

	void FeatherstoneArticulation::updateSpatialTendonConstraintsRecursive(
		const ArticulationAttachment* attachments, const PxU32 attachmentID, 
		const PxReal accumLength, const PxVec3& pAttachPoint, 
		ArticulationData& data)
	{
		const ArticulationAttachment& attachment = attachments[attachmentID];

		//const PxReal restDist = attachment.restDistance;

		const PxTransform& cBody2World = data.getAccumulatedPoses()[attachment.linkInd];

		const PxVec3 rb = cBody2World.q.rotate(attachment.relativeOffset);

		const PxVec3 cAttachPoint = cBody2World.p + rb;

		const PxVec3 dif = pAttachPoint - cAttachPoint;
		const PxReal distanceSq = dif.magnitudeSquared();
		const PxReal distance = PxSqrt(distanceSq);

		const PxReal u = distance * attachment.coefficient + accumLength;

		const PxU32 childCount = attachment.childCount;
		if (childCount)
		{
			for (ArticulationBitField children = attachment.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);

				updateSpatialTendonConstraintsRecursive(attachments, child, u, cAttachPoint, data);
			}
		}
		else
		{
			PxU32 index = attachment.mConstraintInd;
			ArticulationInternalTendonConstraint& constraint = data.mInternalSpatialTendonConstraints[index];
			
			constraint.accumulatedLength = u;
		}
	}

	void FeatherstoneArticulation::setupInternalFixedTendonConstraintsRecursive(
		const ArticulationLink* links,
		ArticulationTendonJoint* tendonJoints,
		const bool fixBase,
		ArticulationData& data,
		const PxReal stepDt,
		const bool isTGSSolver,
		const PxU32 tendonJointID,
		const PxReal stiffness,
		const PxReal damping,
		const PxReal limitStiffness,
		const PxU32 startLink,
		const PxVec3& startAxis,
		const PxVec3& startRaXn)
	{
		ArticulationTendonJoint& tendonJoint = tendonJoints[tendonJointID];

		const ArticulationLink& cLink = links[tendonJoint.linkInd];

		const PxTransform& cBody2World = cLink.bodyCore->body2World;

		const PxReal cfm = PxMax(cLink.cfm, links[startLink].cfm);

		ArticulationJointCoreData& jointDatum = data.getJointData(tendonJoint.linkInd);
		ArticulationJointCore& joint = *cLink.inboundJoint;

		PxU16 tendonJointAxis = tendonJoint.axis;

		PX_ASSERT(joint.motion[tendonJointAxis] != PxArticulationMotion::eLOCKED);

		//Cm::SpatialVector cImpulse;
		PxU32 dofIndex = joint.invDofIds[tendonJointAxis];

		tendonJoint.startJointOffset = PxTo16(jointDatum.jointOffset + dofIndex);

		///*PxReal jointPose = jointPositions[jointDatum.jointOffset + dofIndex] * tendonJoint.coefficient;

		//jointPose += accumulatedJointPose;*/

		{
			const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();

			PxVec3 axis, rbXn;
			if (tendonJointAxis < PxArticulationAxis::eX)
			{
				const PxVec3 tAxis = worldMotionMatrix[jointDatum.jointOffset + dofIndex].top;
				axis = PxVec3(0.f);
				rbXn = tAxis;
			}
			else
			{
				// PT:: tag: scalar transform*transform
				const PxTransform cB2w = cBody2World.transform(joint.childPose);
				const PxVec3 tAxis = worldMotionMatrix[jointDatum.jointOffset + dofIndex].bottom;
				axis = tAxis;
				rbXn = (cB2w.p - cBody2World.p).cross(axis);
			}
		
			Cm::SpatialVector deltaVA, deltaVB;
			FeatherstoneArticulation::getImpulseSelfResponse(links, data,
				startLink, Cm::SpatialVector(startAxis, startRaXn), deltaVA,
				tendonJoint.linkInd, Cm::SpatialVector(-axis, -rbXn), deltaVB);

			/*const PxU32 pLinkInd = cLink.parent;
			printf("(%i, %i) deltaV1(%f, %f, %f, %f, %f, %f)\n",
				pLinkInd, tendonJoint.linkInd, deltaV1.linear.x, deltaV1.linear.y, deltaV1.linear.z, deltaV1.angular.x,
				deltaV1.angular.y, deltaV1.angular.z);*/

			const PxReal r0 = deltaVA.linear.dot(startAxis) + deltaVA.angular.dot(startRaXn);
			const PxReal r1 = deltaVB.linear.dot(axis) + deltaVB.angular.dot(rbXn);

			const PxReal unitResponse = r0 - r1;

			const PxReal recipResponse = 1.0f / (unitResponse + cfm);

			const PxU32 count = data.mInternalFixedTendonConstraints.size();
			data.mInternalFixedTendonConstraints.forceSize_Unsafe(count + 1);

			ArticulationInternalTendonConstraint* constraint = &data.mInternalFixedTendonConstraints[count];
			tendonJoint.mConstraintInd = PxU16(count);

			constraint->row0 = Cm::UnAlignedSpatialVector(startAxis, startRaXn);
			constraint->row1 = Cm::UnAlignedSpatialVector(axis, rbXn);
			constraint->deltaVA = r0; //We only need to record the change in velocity projected onto the dof for this!
			constraint->deltaVB = Cm::UnAlignedSpatialVector(deltaVB.angular, deltaVB.linear);
			constraint->linkID0 = startLink;
			constraint->linkID1 = tendonJoint.linkInd;
			constraint->recipResponse = recipResponse;
			constraint->appliedForce = 0.0f;
			constraint->limitAppliedForce = 0.0f;

			constraint->setTendonImplicitSpringParams(
				computeTendonSpringParams
					(stepDt, isTGSSolver, unitResponse, 
					 stiffness, damping, 
					 limitStiffness));
		}

		const PxU32 childCount = tendonJoint.childCount;

		if (childCount)
		{
			for (ArticulationBitField children = tendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);

				setupInternalFixedTendonConstraintsRecursive(links, tendonJoints, fixBase, data, stepDt,
					isTGSSolver, child, stiffness, damping, limitStiffness, startLink, startAxis, startRaXn);
			}
		}
	}

	void FeatherstoneArticulation::setupInternalConstraints(
		ArticulationData& data,
		PxReal stepDt,
		PxReal dt,
		PxReal /*invDt*/,
		bool isTGSSolver)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		data.mInternalConstraints.forceSize_Unsafe(0);
		data.mInternalConstraints.reserve(data.getDofs());

		data.mInternalLimits.forceSize_Unsafe(0);
		data.mInternalLimits.reserve(data.getDofs());

		const PxReal maxForceScale = data.getArticulationFlags() & PxArticulationFlag::eDRIVE_LIMITS_ARE_FORCES ? dt : 1.f;

		const PxU32 numChildren = links[0].mNumChildren;
		const PxU32 offset = links[0].mChildrenStartIndex;
		for (PxU32 i = 0; i < numChildren; ++i)
		{
			const PxU32 child = offset + i;

			setupInternalConstraintsRecursive(links, data, stepDt, dt, isTGSSolver, child, maxForceScale);
		}

		PxU32 totalNumAttachments = 0;
		for (PxU32 i = 0; i < data.mNumSpatialTendons; ++i)
		{
			Dy::ArticulationSpatialTendon* tendon = data.mSpatialTendons[i];
			totalNumAttachments += tendon->getNumAttachments();
		}

		data.mInternalSpatialTendonConstraints.forceSize_Unsafe(0);
		data.mInternalSpatialTendonConstraints.reserve(totalNumAttachments);

		for (PxU32 i = 0; i < data.mNumSpatialTendons; ++i)
		{
			Dy::ArticulationSpatialTendon* tendon = data.mSpatialTendons[i];

			Dy::ArticulationAttachment* attachments = tendon->getAttachments();

			ArticulationAttachment& pAttachment = attachments[0];

			//const PxU32 childCount = pAttachment.childCount;
			
			//PxReal scale = 1.f/PxReal(childCount);

			const PxReal coefficient = pAttachment.coefficient;

			const PxU32 startLink = pAttachment.linkInd;
			const ArticulationLink& pLink = links[startLink];
			const PxTransform pBody2World = pLink.bodyCore->body2World;
			const PxVec3 ra = pBody2World.q.rotate(pAttachment.relativeOffset);
			const PxVec3 pAttachPoint = pBody2World.p + ra;
			
			for (ArticulationAttachmentBitField children = pAttachment.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);

				ArticulationAttachment& attachment = attachments[child];
				const ArticulationLink& cLink = links[attachment.linkInd];
				const PxTransform cBody2World = cLink.bodyCore->body2World;
				const PxVec3 rb = cBody2World.q.rotate(attachment.relativeOffset);
				const PxVec3 cAttachPoint = cBody2World.p + rb;

				const PxVec3 axis = (pAttachPoint - cAttachPoint).getNormalized();
				const PxVec3 raXn = ra.cross(axis);

				setupInternalSpatialTendonConstraintsRecursive(links, attachments, tendon->getNumAttachments(), pAttachPoint, fixBase, data, stepDt, isTGSSolver,
					child, tendon->mStiffness, tendon->mDamping, tendon->mLimitStiffness, tendon->mOffset*coefficient, startLink,
					axis, raXn);
			}
		}

		PxU32 totalNumTendonJoints = 0;
		for (PxU32 i = 0; i < data.mNumFixedTendons; ++i)
		{
			Dy::ArticulationFixedTendon* tendon = data.mFixedTendons[i];
			totalNumTendonJoints += tendon->getNumJoints();
		}

		data.mInternalFixedTendonConstraints.forceSize_Unsafe(0);
		data.mInternalFixedTendonConstraints.reserve(totalNumTendonJoints);

		for (PxU32 i = 0; i < data.mNumFixedTendons; ++i)
		{
			ArticulationFixedTendon* tendon = data.mFixedTendons[i];

			ArticulationTendonJoint* tendonJoints = tendon->getTendonJoints();

			ArticulationTendonJoint& pTendonJoint = tendonJoints[0];

			const PxU32 startLinkInd = pTendonJoint.linkInd;
			const ArticulationLink& pLink = links[startLinkInd];
			const PxTransform& pBody2World = pLink.bodyCore->body2World;
			
			for (ArticulationAttachmentBitField children = pTendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);

				ArticulationTendonJoint& cTendonJoint = tendonJoints[child];

				const ArticulationLink& cLink = links[cTendonJoint.linkInd];

				ArticulationJointCore* joint = cLink.inboundJoint;

				PX_ASSERT(joint != NULL);

				ArticulationJointCoreData* jointDatum = &data.getJointData(cTendonJoint.linkInd);

				PxU16 tendonJointAxis = cTendonJoint.axis;

				PX_ASSERT(joint->motion[tendonJointAxis] != PxArticulationMotion::eLOCKED);

				PxVec3 startAxis, raXn;
				PxU32 dofIndex = joint->invDofIds[tendonJointAxis];

				const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();

				if (tendonJointAxis < PxArticulationAxis::eX)
				{
					const PxVec3 axis = worldMotionMatrix[jointDatum->jointOffset + dofIndex].top;
					startAxis = PxVec3(0.f);
					raXn = axis;
				}
				else
				{
					// PT:: tag: scalar transform*transform
					const PxTransform cA2w = pBody2World.transform(joint->parentPose);
					const PxVec3 axis = worldMotionMatrix[jointDatum->jointOffset + dofIndex].bottom;
					const PxVec3 ang0 = (cA2w.p - pBody2World.p).cross(axis);
					startAxis = axis;
					raXn = ang0;
				}

				setupInternalFixedTendonConstraintsRecursive(links, tendonJoints, fixBase, data, stepDt, isTGSSolver,
					child, tendon->mStiffness, tendon->mDamping, tendon->mLimitStiffness, startLinkInd, startAxis, raXn);
			}
		}

		//Now set up the internal mimic joint constraints.
		setupInternalMimicJointConstraints(data);
	}

	void FeatherstoneArticulation::setupSolverConstraints(ArticulationData& data)
	{
		setupInternalConstraints(data, data.getDt(), data.getDt(), 1.f / data.getDt(), false);
	}

	void FeatherstoneArticulation::setupSolverConstraintsTGS(FeatherstoneArticulation* articulation, PxReal dt, PxReal invDt, PxReal totalDt)
	{
		articulation->setupInternalConstraints(articulation->mArticulationData, dt, totalDt, invDt, true);
	}

	void FeatherstoneArticulation::teleportLinks(ArticulationData& data)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
	
		ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();

		const PxReal* PX_RESTRICT jointPositions = data.getJointPositions();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getMotionMatrix();

		const PxU32 linkCount = data.getLinkCount();
		const PxQuat* PX_RESTRICT relativeQuats = data.getRelativeQuat();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			
			const ArticulationJointCoreData& jointDatum = jointData[linkID];

			const ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			const ArticulationJointCore* joint = link.inboundJoint;

			const PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;

			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			const PxQuat relativeQuat = relativeQuats[linkID];

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				newParentToChild = relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = motionMatrix[jointDatum.jointOffset].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
			{
				const PxVec3& u = motionMatrix[jointDatum.jointOffset].top;

				PxQuat jointRotation = PxQuat(-jPosition[0], u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;

				newParentToChild = (jointRotation * relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());

				break;
			}
			case PxArticulationJointType::eSPHERICAL:
			{
				PxQuat jointRotation(PxIdentity);

				{
					PxVec3 axis(0.f);
					for (PxU32 d = 0; d < jointDatum.nbDof; ++d)
					{
						axis += motionMatrix[jointDatum.jointOffset + d].top * -jPosition[d];
					}
					PxReal angle = axis.normalize();
					jointRotation = angle < 1e-10f ? PxQuat(PxIdentity) : PxQuat(angle, axis);
					if(jointRotation.w < 0.f)
						jointRotation = -jointRotation;
				}

				newParentToChild = (jointRotation * relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;
				break;
			}
			case PxArticulationJointType::eFIX:
			{
				//this is fix joint so joint don't have velocity
				newParentToChild = relativeQuat;

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				r = e + d;
				break;
			}
			default:
				break;
			}

			PxTransform& body2World = link.bodyCore->body2World;
			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

			PX_ASSERT(body2World.isSane());
		}
	}

	void FeatherstoneArticulation::computeLinkVelocities(ArticulationData& data)
	{
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* PX_RESTRICT motionVelocities = data.getMotionVelocities();
		const PxReal* PX_RESTRICT jointVelocities = data.getJointVelocities();

		// sync root motion vel:
		const PxsBodyCore& rootBodyCore = *links[0].bodyCore;
		motionVelocities[0].top = rootBodyCore.angularVelocity;
		motionVelocities[0].bottom = rootBodyCore.linearVelocity;

		const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getMotionMatrix();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			
			PxsBodyCore& bodyCore = *link.bodyCore;

			const PxTransform& body2World = bodyCore.body2World;

			const ArticulationLink& plink = links[link.parent];
			const PxsBodyCore& pbodyCore = *plink.bodyCore;

			const Cm::SpatialVectorF parentVel(pbodyCore.angularVelocity, pbodyCore.linearVelocity);

			const PxTransform& pBody2World = pbodyCore.body2World;

			const PxVec3 rw = body2World.p - pBody2World.p;

			Cm::SpatialVectorF vel = FeatherstoneArticulation::translateSpatialVector(-rw, parentVel);

			if (jointVelocities)
			{
				const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

				Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
				for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
				{
					deltaV += motionMatrix[jointDatum.jointOffset + ind] * jVelocity[ind];
				}

				vel.top += body2World.q.rotate(deltaV.top);
				vel.bottom += body2World.q.rotate(deltaV.bottom);
			}

			bodyCore.linearVelocity = vel.bottom;
			bodyCore.angularVelocity = vel.top;
			motionVelocities[linkID] = vel;
		}
	}

	// AD: needed because we define the templated function in a CPP.
	template void FeatherstoneArticulation::jcalc<false>(ArticulationData& data);
	template void FeatherstoneArticulation::jcalc<true>(ArticulationData& data);

	template<bool immediateMode>
	void FeatherstoneArticulation::jcalc(ArticulationData& data)
	{	
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		ArticulationJointCoreData* PX_RESTRICT jointData = data.getJointData();
		PxQuat* PX_RESTRICT relativeQuats = data.getRelativeQuat();
		Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getMotionMatrix();

		const PxU32 linkCount = data.getLinkCount();

		PxU32 totalDof = 0;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = jointData[linkID];

			PX_CHECK_AND_RETURN(joint->jointType != PxArticulationJointType::eUNDEFINED, "FeatherstoneArticulation::jcalc application need to define valid joint type and motion");

			// AD: only used in immediate mode because we don't have the llArticulation there to write directly.
			if (immediateMode)
			{
				// compute joint dof
				const PxU32 dof = jointDatum.configureJointDofs(joint);
				PxReal* PX_RESTRICT jointTargetPositions = data.getJointTargetPositions();
				PxReal* PX_RESTRICT jointTargetVelocities = data.getJointTargetVelocities();

				for(PxU32 dofId = 0; dofId < dof; ++dofId)
				{
					PxU32 id = totalDof + dofId;
					PxU32 llDofId = joint->dofIds[dofId];
					jointTargetPositions[id] = joint->targetP[llDofId];
					jointTargetVelocities[id] = joint->targetV[llDofId];
				}

				jointDatum.jointOffset = totalDof;
				totalDof += dof;
			}

			// PT: we now compute the joint axes on-the-fly (as we do in the GPU version) instead of storing them
			Cm::UnAlignedSpatialVector jointAxis[DY_MAX_DOF];
			{
				PxU32 nbDof = 0;
				for (PxU8 i = 0; i < DY_MAX_DOF; ++i)
				{
					// PT: make sure the data is initialized to a valid vector
					jointAxis[i] = Cm::UnAlignedSpatialVector::Zero();
					jointAxis[i].top.x = 1.0f;

					if (joint->motion[i] != PxArticulationMotion::eLOCKED)
					{
						Cm::UnAlignedSpatialVector axis = Cm::UnAlignedSpatialVector::Zero();
						//axis is in the local space of joint
						axis[i] = 1.f;

						jointAxis[nbDof++] = axis;
					}
				}
			}

			joint->setJointFrame(&motionMatrix[jointDatum.jointOffset], jointAxis,
								relativeQuats[linkID],
								jointDatum.nbDof);
		}

		if(immediateMode)
		{
			if(totalDof != data.getDofs())
			{
				data.resizeJointData(totalDof);
			}
			data.setDofs(totalDof);
		} 
	}

	//compute link's spatial inertia tensor
	void FeatherstoneArticulation::computeSpatialInertia(ArticulationData& data)
	{
		SpatialMatrix* PX_RESTRICT worldSpatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();
		PxMat33* PX_RESTRICT worldIsolatedSpatialArticulatedInertia = data.getWorldIsolatedSpatialArticulatedInertia();
		PxReal* PX_RESTRICT masses = data.getMasses();
		const PxU32 nbLinks = data.getLinkCount();
		for (PxU32 linkID = 0; linkID < nbLinks; ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);

			const PxsBodyCore& core = *link.bodyCore;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

			SpatialMatrix& worldArticulatedInertia = worldSpatialArticulatedInertia[linkID];

			//construct inertia matrix
			const PxVec3 inertiaTensor = Cm::safeRecip<PxVec3>(core.inverseInertia);

			const PxMat33 rot(core.body2World.q);

			worldArticulatedInertia.topLeft = PxMat33(PxZero);
			worldArticulatedInertia.topRight = PxMat33::createDiagonal(PxVec3(m));
			Cm::transformInertiaTensor(inertiaTensor, rot, worldArticulatedInertia.bottomLeft);

			worldIsolatedSpatialArticulatedInertia[linkID] = worldArticulatedInertia.bottomLeft;
			masses[linkID] = m;
		}
	}

	void FeatherstoneArticulation::computeZ(const ArticulationData& data, 
		const PxVec3& gravity, ScratchData& scratchData)
	{
		const Cm::SpatialVectorF* PX_RESTRICT motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* PX_RESTRICT spatialZAForces = scratchData.spatialZAVectors;
		const Cm::SpatialVector* PX_RESTRICT externalAccels = scratchData.externalAccels;	

		const SpatialMatrix* PX_RESTRICT worldSpatialArticulatedInertia = data.getWorldSpatialArticulatedInertia();

		const PxU32 nbLinks = data.getLinkCount();
		for (PxU32 linkID = 0; linkID < nbLinks; ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);

			const PxsBodyCore& core = *link.bodyCore;
			//const PxTransform& body2World = core.body2World;

			const PxMat33& I = worldSpatialArticulatedInertia[linkID].bottomLeft;

			//construct spatial zero acceleration
			Cm::SpatialVectorF& z = spatialZAForces[linkID];

			//Cm::SpatialVectorF v;
			//v.top = motionVelocities[linkID].top;
			//v.bottom = motionVelocities[linkID].bottom;

			//KS - limit the magnitude of the angular velocity that contributes to the geometric term. This is a
			//v^2 term and can become unstable if it is too large!

			Cm::SpatialVectorF v = motionVelocities[linkID];

			PxVec3 vA = v.top;

			PxVec3 gravLinAccel(0.f);
			if(!core.disableGravity)
				gravLinAccel = -gravity;

			PX_ASSERT(core.inverseMass != 0.f);

			const PxReal m = 1.0f / core.inverseMass;

			Cm::SpatialVectorF zTmp;

			zTmp.top = (gravLinAccel * m);
			zTmp.bottom = vA.cross(I * vA);

			PX_ASSERT(zTmp.top.isFinite());
			PX_ASSERT(zTmp.bottom.isFinite());

			if (externalAccels)
			{
				const Cm::SpatialVector& externalAccel = externalAccels[linkID];

				const PxVec3 exLinAccel = -externalAccel.linear;
				const PxVec3 exAngAccel = -externalAccel.angular;

				zTmp.top += (exLinAccel * m);
				zTmp.bottom += I * exAngAccel;
			}

			z = zTmp;
		}
	}

	//compute coriolis and centrifugal term
	void FeatherstoneArticulation::computeC(const ArticulationData& data, ScratchData& scratchData)
	{
		const PxReal* jointVelocities = scratchData.jointVelocities;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		const PxU32 linkCount = data.getLinkCount();

		coriolisVectors[0] = Cm::SpatialVectorF::Zero();

		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);
			const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

			const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
			Cm::SpatialVectorF& coriolis = coriolisVectors[linkID];
			
			//const PxTransform& body2World = link.bodyCore->body2World;
			//transform parent link's angular velocity into current link's body space
			const PxVec3 pAngular = scratchData.motionVelocities[link.parent].top;
			
			PxVec3 torque = pAngular.cross(pAngular.cross(rw[linkID]));

			//PX_ASSERT(parentAngular.magnitude() < 100.f);

			PxVec3 force(0.f);
			if (jointDatum.nbDof > 0)
			{
				const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = data.getWorldMotionMatrix();

				Cm::SpatialVectorF relVel(PxVec3(0.f), PxVec3(0.f));
				for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
				{
					//Clamp joint velocity used in coriolis terms to reduce chances of unstable feed-back loops
					const PxReal jV = jVelocity[ind];
					relVel.top += worldMotionMatrix[jointDatum.jointOffset + ind].top * jV;
					relVel.bottom += worldMotionMatrix[jointDatum.jointOffset + ind].bottom * jV;
				}
				const PxVec3 aVec = relVel.top;
				force = pAngular.cross(aVec);

				//compute linear part
				const PxVec3 lVel = relVel.bottom;

				const PxVec3 temp1 = 2.f * pAngular.cross(lVel);
				const PxVec3 temp2 = aVec.cross(lVel);
				torque += temp1 + temp2;
			}

			PX_ASSERT(force.isFinite());
			PX_ASSERT(torque.isFinite());
	
			coriolis = Cm::SpatialVectorF(force, torque);//.rotateInv(body2World);
		}
	}

	void FeatherstoneArticulation::computeRelativeTransformC2P(
		const ArticulationLink* links, const PxU32 linkCount, const ArticulationJointCoreData* jointCoreDatas,
		const Cm::UnAlignedSpatialVector* jointDofMotionMatrices,
		PxTransform* linkAccumulatedPoses, PxVec3* linkRws, Cm::UnAlignedSpatialVector* jointDofMotionMatricesW)
	{
		linkAccumulatedPoses[0] = links[0].bodyCore->body2World;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;
			const PxU32 jointOffset = jointCoreDatas[linkID].jointOffset;
			const PxU32 dofCount = jointCoreDatas[linkID].nbDof;

			const PxTransform& body2World = bodyCore.body2World;

			const ArticulationLink& pLink = links[link.parent];
			const PxsBodyCore& pBodyCore = *pLink.bodyCore;
			const PxTransform& pBody2World = pBodyCore.body2World;

			//const PxTransform tC2P = pBody2World.transformInv(body2World).getNormalized();
			
			linkRws[linkID] = body2World.p - pBody2World.p;
			
			const Cm::UnAlignedSpatialVector* motionMatrix = &jointDofMotionMatrices[jointOffset];
			Cm::UnAlignedSpatialVector* worldMotionMatrix = &jointDofMotionMatricesW[jointOffset];

			for (PxU32 i = 0; i < dofCount; ++i)
			{
				const Cm::UnAlignedSpatialVector worldRow = motionMatrix[i].rotate(body2World);
				worldMotionMatrix[i] = worldRow;
			}

			linkAccumulatedPoses[linkID] = body2World;

#if FEATURESTONE_DEBUG
			{
				//debug
				PxMat33 pToC = c2p.getTranspose();
				//parentToChild -rR
				PxMat33 T2 = skewMatrixPR * pToC;

				PX_ASSERT(SpatialMatrix::isTranspose(linkDatum.childToParent.T, T2));
			}
#endif
		}
	}

	void FeatherstoneArticulation::getDenseJacobian(PxArticulationCache& cache, PxU32 & nRows, PxU32 & nCols)
	{
		//make sure motionMatrix has been set
		//jcalc(mArticulationData);
		initializeCommonData();

		const PxU32 linkCount = mArticulationData.getLinkCount();
		ArticulationLink* links = mArticulationData.getLinks();

#if 0	//Enable this if you want to compare results of this function with results of computeLinkVelocities().

		if (mArticulationData.getDataDirty())
		{
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, PX_FL, "Articulation::getDenseJacobian(): commonInit need to be called first to initialize data!");
			return;
		}

		PxcScratchAllocator* allocator = reinterpret_cast<PxcScratchAllocator*>(cache.scratchAllocator);

		ScratchData scratchData;
		PxU8* tempMemory = allocateScratchSpatialData(allocator, linkCount, scratchData);

		scratchData.jointVelocities = mArticulationData.getJointVelocities();


		computeLinkVelocities(mArticulationData, scratchData);
		const ArticulationLink& baseLink = links[0];
		PxsBodyCore& core0 = *baseLink.bodyCore;
#endif

		const PxU32 totalDofs = getDofs();
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//matrix dims is 
		nCols = (fixBase ? 0 : 6) + totalDofs;
		nRows = (fixBase ? 0 : 6) + (linkCount - 1) * 6;

//		auto jacobian = [&](PxU32 row, PxU32 col) -> PxReal &  { return cache.denseJacobian[nCols * row + col]; } ;
		#define jacobian(row, col)	cache.denseJacobian[nCols * (row) + (col)]
		
		PxU32 destRow = 0;
		PxU32 destCol = 0;

		if (!fixBase)
		{
			jacobian(0, 0) = 1.0f;
			jacobian(0, 1) = 0.0f;
			jacobian(0, 2) = 0.0f;
			jacobian(0, 3) = 0.0f;
			jacobian(0, 4) = 0.0f;
			jacobian(0, 5) = 0.0f;

			jacobian(1, 0) = 0.0f;
			jacobian(1, 1) = 1.0f;
			jacobian(1, 2) = 0.0f;
			jacobian(1, 3) = 0.0f;
			jacobian(1, 4) = 0.0f;
			jacobian(1, 5) = 0.0f;

			jacobian(2, 0) = 0.0f;
			jacobian(2, 1) = 0.0f;
			jacobian(2, 2) = 1.0f;
			jacobian(2, 3) = 0.0f;
			jacobian(2, 4) = 0.0f;
			jacobian(2, 5) = 0.0f;

			jacobian(3, 0) = 0.0f;
			jacobian(3, 1) = 0.0f;
			jacobian(3, 2) = 0.0f;
			jacobian(3, 3) = 1.0f;
			jacobian(3, 4) = 0.0f;
			jacobian(3, 5) = 0.0f;

			jacobian(4, 0) = 0.0f;
			jacobian(4, 1) = 0.0f;
			jacobian(4, 2) = 0.0f;
			jacobian(4, 3) = 0.0f;
			jacobian(4, 4) = 1.0f;
			jacobian(4, 5) = 0.0f;

			jacobian(5, 0) = 0.0f;
			jacobian(5, 1) = 0.0f;
			jacobian(5, 2) = 0.0f;
			jacobian(5, 3) = 0.0f;
			jacobian(5, 4) = 0.0f;
			jacobian(5, 5) = 1.0f;

			destRow += 6;
			destCol += 6;
		}

		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)//each iteration of this writes 6 rows in the matrix
		{
			const ArticulationLink& link = links[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			const PxTransform& body2World = bodyCore.body2World;

			const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
			const PxU32 parentLinkID = link.parent;

			if (parentLinkID || !fixBase)
			{
				// VR: mArticulationData.getJointData(0) isn't initialized. parentLinkID can be 0 if fixBase is false.
				//const ArticulationJointCoreData& parentJointDatum = mArticulationData.getJointData(parentLinkID);
				//const PxU32 parentsFirstDestCol = parentJointDatum.jointOffset + (fixBase ? 0 : 6);
				//const PxU32 parentsLastDestCol = parentsFirstDestCol + parentJointDatum.dof;
				const PxU32 parentsLastDestCol = parentLinkID ?
													mArticulationData.getJointData(parentLinkID).jointOffset + (fixBase ? 0 : 6) + mArticulationData.getJointData(parentLinkID).nbDof :
													6;

				// VR: With parentLinkID == 0 this expression has two unsigned integer overflows, but the result is still correct.
				const PxU32 parentsDestRow = (fixBase ? 0 : 6) + (parentLinkID - 1) * 6;

				for (PxU32 col = 0; col < parentsLastDestCol; col++)
				{
					//copy downward the 6 cols from parent
					const PxVec3 parentAng(
						jacobian(parentsDestRow + 3, col),
						jacobian(parentsDestRow + 4, col),
						jacobian(parentsDestRow + 5, col)
						);

					const PxVec3 parentAngxRw = parentAng.cross(rw[linkID]);

					jacobian(destRow + 0, col) = jacobian(parentsDestRow + 0, col) + parentAngxRw.x;
					jacobian(destRow + 1, col) = jacobian(parentsDestRow + 1, col) + parentAngxRw.y;
					jacobian(destRow + 2, col) = jacobian(parentsDestRow + 2, col) + parentAngxRw.z;

					jacobian(destRow + 3, col) = parentAng.x;
					jacobian(destRow + 4, col) = parentAng.y;
					jacobian(destRow + 5, col) = parentAng.z;
				}

				for (PxU32 col = parentsLastDestCol; col < destCol; col++)
				{
					//fill with zeros.
					jacobian(destRow + 0, col) = 0.0f;
					jacobian(destRow + 1, col) = 0.0f;
					jacobian(destRow + 2, col) = 0.0f;

					jacobian(destRow + 3, col) = 0.0f;
					jacobian(destRow + 4, col) = 0.0f;
					jacobian(destRow + 5, col) = 0.0f;
				}
			}

			//diagonal block:
			const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = mArticulationData.getMotionMatrix();
			for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& v = motionMatrix[jointDatum.jointOffset + ind];

				const PxVec3 ang = body2World.rotate(v.top);
				const PxVec3 lin = body2World.rotate(v.bottom);

				jacobian(destRow + 0, destCol) = lin.x;
				jacobian(destRow + 1, destCol) = lin.y;
				jacobian(destRow + 2, destCol) = lin.z;

				jacobian(destRow + 3, destCol) = ang.x;
				jacobian(destRow + 4, destCol) = ang.y;
				jacobian(destRow + 5, destCol) = ang.z;

				destCol++;
			}

			//above diagonal block: always zero
			for (PxU32 col = destCol; col < nCols; col++)
			{
				jacobian(destRow + 0, col) = 0.0f;
				jacobian(destRow + 1, col) = 0.0f;
				jacobian(destRow + 2, col) = 0.0f;

				jacobian(destRow + 3, col) = 0.0f;
				jacobian(destRow + 4, col) = 0.0f;
				jacobian(destRow + 5, col) = 0.0f;
			}

			destRow += 6;
		}
		#undef jacobian

#if 0  //Enable this if you want to compare results of this function with results of computeLinkVelocities().

		PxReal * jointVels = mArticulationData.getJointVelocities();//size is totalDofs

		PxReal * jointSpaceVelsVector = new PxReal[nCols];
		PxReal * worldSpaceVelsVector = new PxReal[nRows];

		PxU32 offset = 0;

		//stack input:

		if (!fixBase)
		{
			jointSpaceVelsVector[0] = core0.linearVelocity[0];
			jointSpaceVelsVector[1] = core0.linearVelocity[1];
			jointSpaceVelsVector[2] = core0.linearVelocity[2];

			jointSpaceVelsVector[3] = core0.angularVelocity[0];
			jointSpaceVelsVector[4] = core0.angularVelocity[1];
			jointSpaceVelsVector[5] = core0.angularVelocity[2];

			offset = 6;
		}

		for (PxU32 i = 0; i < totalDofs; i++)
			jointSpaceVelsVector[i + offset] = jointVels[i];

		//multiply:

		for (PxU32 row = 0; row < nRows; row++)
		{
			worldSpaceVelsVector[row] = 0.0f;
			for (PxU32 col = 0; col < nCols; col++)
			{
				worldSpaceVelsVector[row] += jacobian(row, col)*jointSpaceVelsVector[col];
			}
		}

		//worldSpaceVelsVector should now contain the same result as scratchData.motionVelocities (except for swapped linear/angular vec3 order).

		delete[] jointSpaceVelsVector;
		delete[] worldSpaceVelsVector;

		allocator->free(tempMemory);
#endif
	}

	void FeatherstoneArticulation::computeLinkStates(
		const PxF32 dt, const PxReal invLengthScale, const PxVec3& gravity,
		const bool fixBase,
		const PxU32 linkCount,
		const PxTransform* linkAccumulatedPosesW, Cm::SpatialVector* linkExternalAccelsW, const PxVec3* linkRsW, const Cm::UnAlignedSpatialVector* jointDofMotionMatricesW,
		const Dy::ArticulationJointCoreData* jointCoreData, bool externalForcesEveryTgsIterationEnabled,
		Dy::ArticulationLink* links,
		Cm::SpatialVectorF* linkMotionAccelerationsW, Cm::SpatialVectorF* linkMotionVelocitiesW,
		Cm::SpatialVectorF* linkZAExtForcesW, Cm::SpatialVectorF* linkZAIntForcesW, Cm::SpatialVectorF* linkCoriolisVectorsW,
		PxMat33* linkIsolatedSpatialArticulatedInertiasW, PxF32* linkMasses, Dy::SpatialMatrix* linkSpatialArticulatedInertiasW,
		PxReal* jointDofVelocities,
		Cm::SpatialVectorF& rootPreMotionVelocityW, PxVec3& comW, PxF32& invSumMass)
	{
		const PxReal invDt = dt == 0.0f ? 0.0f : 1.0f / dt;

		//Initialise motion velocity, motion acceleration and coriolis vector of root link.
		Cm::SpatialVectorF rootLinkVel;
		{
			const Dy::ArticulationLink& baseLink = links[0];
			const PxsBodyCore& core0 = *baseLink.bodyCore;
			rootLinkVel = fixBase ? Cm::SpatialVectorF::Zero() : Cm::SpatialVectorF(core0.angularVelocity, core0.linearVelocity);
			linkMotionVelocitiesW[0] = rootLinkVel;
			linkMotionAccelerationsW[0] = fixBase ? Cm::SpatialVectorF::Zero() : linkMotionAccelerationsW[0];
			linkCoriolisVectorsW[0] = Cm::SpatialVectorF::Zero();
			rootPreMotionVelocityW = rootLinkVel;
		}

		//Is it really necessary? It is already resolved as an internal constraint.
		PxReal ratio = 1.f;
		if (jointDofVelocities)
		{
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				const ArticulationLink& link = links[linkID];
				const ArticulationJointCoreData& jointDatum = jointCoreData[linkID];
				const PxReal* jVelocity = &jointDofVelocities[jointDatum.jointOffset];
				for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
				{
					const PxReal maxJVelocity = link.inboundJoint->maxJointVelocity[ind];
					const PxReal jVel = jVelocity[ind];
					ratio = (jVel != 0.0f) ? PxMin(ratio, maxJVelocity / PxAbs(jVel)) : ratio;
				}
			}
		}

		PxReal sumMass = 0.f;
		PxVec3 COM(0.f);
		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			//Set the cfm value from bodyCore
			link.cfm = (fixBase && linkID == 0) ? 0.f : bodyCore.cfmScale * invLengthScale;

			//Read the inertia and mass from bodyCore
			const PxVec3 inertiaTensor = Cm::safeRecip<PxVec3>(bodyCore.inverseInertia);

			const PxReal invMass = bodyCore.inverseMass;
			const PxReal m = invMass == 0.f ? 0.f : 1.0f / invMass;

			//Compute the inertia matrix Iw = R * I * Rtranspose
			//Compute the articulated inertia.
			PxMat33 Iw; //R * I * Rtranspose
			SpatialMatrix worldArticulatedInertia;
			{
				const PxMat33 rot(linkAccumulatedPosesW[linkID].q);
				Cm::transformInertiaTensor(inertiaTensor, rot, Iw);
				worldArticulatedInertia.topLeft = PxMat33(PxZero);
				worldArticulatedInertia.topRight = PxMat33::createDiagonal(PxVec3(m));
				worldArticulatedInertia.bottomLeft = Iw;
			}

			//Set the articulated inertia, inertia and mass of the link.
			linkSpatialArticulatedInertiasW[linkID] = worldArticulatedInertia;
			linkIsolatedSpatialArticulatedInertiasW[linkID] = Iw;
			linkMasses[linkID] = m;

			//Accumulate the centre of mass.
			sumMass += m;
			COM += linkAccumulatedPosesW[linkID].p * m;

			Cm::SpatialVectorF vel;
			if (linkID != 0)
			{
				//Propagate spatial velocity of link parent to link's spatial velocity.
				const Cm::SpatialVectorF pVel = linkMotionVelocitiesW[link.parent];
				vel = FeatherstoneArticulation::translateSpatialVector(-linkRsW[linkID], pVel);

				//Propagate joint dof velocities to the link's spatial velocity vector.
				//Accumulate spatial forces that the joint applies to the link.
				if (jointDofVelocities)
				{
					//The coriolis vector depends on the type of joint and the joint motion matrix.
					//However, some terms in the coriolis vector are common to all joint types. 
					//Write down the term that is independent of the joint.
					Cm::SpatialVectorF coriolisVector(PxVec3(PxZero), pVel.top.cross(pVel.top.cross(linkRsW[linkID])));
					const ArticulationJointCoreData& jointDatum = jointCoreData[linkID];
					if (jointDatum.nbDof)
					{
						//Compute the effect of the joint velocities on the link.
						PxReal* jVelocity = &jointDofVelocities[jointDatum.jointOffset];
						Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
						for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
						{
							const PxReal jVel = jVelocity[ind] * ratio;
							deltaV += jointDofMotionMatricesW[jointDatum.jointOffset + ind] * jVel;
							jVelocity[ind] = jVel;
						}

						//Add the effect of the joint velocities to the link.
						vel.top += deltaV.top;
						vel.bottom += deltaV.bottom;

						//Compute the coriolis vector.
						//mu(i) is the velocity arising from the joint motion: mu(i) = motionMatrix*qdot
						//where qdot is the joint velocity.
						//mu(i) is already computed and cached in deltaV.
						//For the revolute joint we have: 
						//deltaV = {mu(i), mu(i) X d(i)}
						//coriolis vector += {omega(i-1) X mu(i), 2*omega(i-1) X (mu(i) X d(i)) + mu(i) X (mu(i) X d(i))}
						//For the prismatic joint we have:
						//deltaV = {0, mu(i)}
						//coriolis vector += {0, 2*omega(i-1) X mu(i)}
						coriolisVector += Cm::SpatialVectorF(pVel.top.cross(deltaV.top), 2.0f*pVel.top.cross(deltaV.bottom) + deltaV.top.cross(deltaV.bottom));
					}
					//TODOGY - if jointVelocities is null we do not appear to set coriolisVectors[linkId] but we do set coriolisVectors[0]
					linkCoriolisVectorsW[linkID] = coriolisVector;
				}

				//PX_ASSERT(vel.top.isFinite() && PxAbs(vel.top.x) < 10000.f && PxAbs(vel.top.y) < 10000.f && PxAbs(vel.top.z) < 10000.f);
				//PX_ASSERT(vel.bottom.isFinite() && PxAbs(vel.bottom.x) < 10000.f && PxAbs(vel.bottom.y) < 10000.f && PxAbs(vel.bottom.z) < 10000.f);
				linkMotionVelocitiesW[linkID] = vel;
			}
			else
			{
				vel = rootLinkVel;
				//Note: we have already set motionVelocities[0] and coriolisVectors[0] so no need to set them again.
			}

			//Account for force arising from external accelerations
			//Account for damping force arising from the velocity and from the velocity that will accumulate from the acceleration terms.
			//Account for scaling force that will bring velocity back to the maximum allowed velocity if velocity exceed the maximum allowed.
			//Example for the linear term:
			//acceleration force = m*(g + extLinAccel)
			//linVelNew = linVel + (g + extLinAccel)*dt
			//damping force = -m*linVelNew*linDamp = -m*linVel*linDamp - m*(g + extLinAccel)*linDamp*dt
			//scaling force = -m*linVel*linScale/dt with linScale = (1.0f - maxLinVel/linVel)
			//Put it all together
			// m*(g + extLinAccel)*(1 - linDamp*dt) - m*linVel*(linDamp + linScale/dt)
			//Bit more reordering:
			//m*[g + extLinAccel)*(1 - linDamp*dt) - linVel*(linDamp + linScale/dt)]
			//Zero acceleration means we need to work against change:
			//-m*[g + extLinAccel)*(1 - linDamp*dt) - linVel*(linDamp + linScale/dt)]
			Cm::SpatialVectorF zExtForces;
			Cm::SpatialVectorF zDamping;
			{
				const PxVec3 g = bodyCore.disableGravity ? PxVec3(PxZero) : gravity;
				const PxVec3 extLinAccel = linkExternalAccelsW ? linkExternalAccelsW[linkID].linear : PxVec3(PxZero);
				const PxF32 lindamp = bodyCore.linearDamping > 0.f ?  PxMin(bodyCore.linearDamping, invDt) : 0.0f;
				const float m2 = vel.bottom.magnitudeSquared();
				const PxF32 linscale = (m2 > bodyCore.maxLinearVelocitySq) ? (1.0f - (PxSqrt(bodyCore.maxLinearVelocitySq)/PxSqrt(m2))) : 0.0f;
				zExtForces.top = (m * (lindamp * dt - 1.0f)) * (g + extLinAccel);
				zDamping.top = (m * (lindamp + linscale * invDt)) * vel.bottom;
			}
			{
				const PxVec3 extAngAccel = linkExternalAccelsW ? linkExternalAccelsW[linkID].angular : PxVec3(PxZero);
				const PxF32 angdamp = bodyCore.angularDamping > 0.f ? PxMin(bodyCore.angularDamping, invDt) : 0.0f;
				const float m2 = vel.top.magnitudeSquared();
				const PxF32 angscale = (m2 > bodyCore.maxAngularVelocitySq) ? (1.0f - (PxSqrt(bodyCore.maxAngularVelocitySq)/PxSqrt(m2))) : 0.0f;
				zExtForces.bottom = Iw * (extAngAccel * (angdamp * dt - 1.0f));
				zDamping.bottom = Iw * (vel.top * (angdamp + angscale * invDt));
			}

			if(externalForcesEveryTgsIterationEnabled)
			{
				// We precompute the isolated z vector for gravity and ext forces here so we don't need to load inertias in the per-substep application.
				// The ugly thing is that we start storing spatial forces into the spatial motion vector linkExternalAccelsW.
				// Reusing linkExternalAccelsW is ok because at the end of the sim step they are anyways cleared or refreshed from velMod in Sc::ArticulationSim::clearAcceleration
				// and inverse dynamics does not use it.

				linkZAExtForcesW[linkID] = zDamping; // only apply damping once for the full step
				linkExternalAccelsW[linkID].linear = zExtForces.top; // must use the same order (linear<=>top) again when picking this up in the substep application
				linkExternalAccelsW[linkID].angular = zExtForces.bottom;
			}
			else
			{
				linkZAExtForcesW[linkID] = zExtForces + zDamping;
			}

			//Account for forces arising from internal accelerations.
			//Note: Mirtich thesis introduces a single spatial zero acceleration force that contains an external [mass*gravity] term and the internal [omega X  (Iw *omega)] term. 
			//In Mirtich it looks like this:
			//	[-m_i * g					]
			//	[omega_i * (I_i * omega_i)	]
			//We split the spatial zero acceleration force into external (above) and internal (below).
			//The sum of the two (external and internal) corresponds to Z_i in the Mirtich formulation.
			// In this function, we're only computing the isolated zero acceleration Z_i but already store
			// it in the Z_i^A (articulated) fields as they will be accumulated later.
			const Cm::SpatialVectorF zInternal(PxVec3(0.f), vel.top.cross(Iw * vel.top));
			linkZAIntForcesW[linkID] = zInternal;
		}

		const PxReal invMass = 1.f / sumMass;
		comW = COM * invMass;
		invSumMass = invMass;
	}

	//compute all links velocities
	void FeatherstoneArticulation::computeLinkVelocities(ArticulationData& data, ScratchData& scratchData)
	{
		Cm::SpatialVectorF* PX_RESTRICT coriolisVectors = scratchData.coriolisVectors;
		const ArticulationLink* PX_RESTRICT links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		//motion velocities has to be in world space to avoid numerical errors caused by space 
		Cm::SpatialVectorF* PX_RESTRICT motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* PX_RESTRICT motionAccelerations = scratchData.motionAccelerations;

		PxReal* PX_RESTRICT jointVelocities = scratchData.jointVelocities;

		const ArticulationLink& baseLink = links[0];

		PxsBodyCore& core0 = *baseLink.bodyCore;

		if (fixBase)
		{
			motionVelocities[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
			motionAccelerations[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			motionVelocities[0] = Cm::SpatialVectorF(core0.angularVelocity, core0.linearVelocity);

			//PX_ASSERT(core0.angularVelocity.isFinite() && PxAbs(core0.angularVelocity.x) < 10000.f && PxAbs(core0.angularVelocity.y) < 10000.f && PxAbs(core0.angularVelocity.z) < 10000.f);
			//PX_ASSERT(core0.linearVelocity.isFinite() && PxAbs(core0.linearVelocity.x) < 10000.f && PxAbs(core0.linearVelocity.y) < 10000.f && PxAbs(core0.linearVelocity.z) < 10000.f);
		}

		coriolisVectors[0] = Cm::SpatialVectorF::Zero();

		data.mRootPreMotionVelocity = motionVelocities[0];

		//Is it really necessary? It is already resolved as an internal constraint.
		//const PxU32 dofCount = data.mDofs;
		PxReal ratio = 1.f;
		if (jointVelocities)
		{
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				const ArticulationLink& link = links[linkID];
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
				for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
				{
					const PxReal maxJVelocity = link.inboundJoint->maxJointVelocity[ind];
					PxReal absJvel = PxAbs(jVelocity[ind]);
					ratio = ratio * absJvel > maxJVelocity ? (maxJVelocity / absJvel) : ratio;
				}
			}
		}

		const PxVec3* PX_RESTRICT rw = data.getRw();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			const Cm::SpatialVectorF pVel = motionVelocities[link.parent];
			Cm::SpatialVectorF vel = FeatherstoneArticulation::translateSpatialVector(-rw[linkID], pVel);
			const PxTransform& body2World = bodyCore.body2World;

			if (jointVelocities)
			{
				PxVec3 torque = pVel.top.cross(pVel.top.cross(rw[linkID]));
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
				
				PxVec3 force(0.f);
				if (jointDatum.nbDof)
				{
					const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = data.getMotionMatrix();

					Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
					for (PxU32 ind = 0; ind < jointDatum.nbDof; ++ind)
					{
						const PxReal jVel = jVelocity[ind] * ratio;
						//deltaV += data.mWorldMotionMatrix[jointDatum.jointOffset + ind] * jVel;
						deltaV += motionMatrix[jointDatum.jointOffset+ind].rotate(body2World) * jVel;
						jVelocity[ind] = jVel;
					}

					vel.top += deltaV.top;
					vel.bottom += deltaV.bottom;

					const PxVec3 aVec = deltaV.top;
					force = pVel.top.cross(aVec);

					//compute linear part
					const PxVec3 lVel = deltaV.bottom;

					const PxVec3 temp1 = 2.f * pVel.top.cross(lVel);
					const PxVec3 temp2 = aVec.cross(lVel);
					torque += temp1 + temp2;

				}
				coriolisVectors[linkID] = Cm::SpatialVectorF(force, torque);
			}

			//PX_ASSERT(vel.top.isFinite() && PxAbs(vel.top.x) < 10000.f && PxAbs(vel.top.y) < 10000.f && PxAbs(vel.top.z) < 10000.f);
			//PX_ASSERT(vel.bottom.isFinite() && PxAbs(vel.bottom.x) < 10000.f && PxAbs(vel.bottom.y) < 10000.f && PxAbs(vel.bottom.z) < 10000.f);

			motionVelocities[linkID] = vel;
		}
	}

	void solveExtContact(const PxSolverConstraintDesc& desc, Vec3V& linVel0, Vec3V& linVel1, Vec3V& angVel0, Vec3V& angVel1,
		Vec3V& linImpulse0, Vec3V& linImpulse1, Vec3V& angImpulse0, Vec3V& angImpulse1, bool doFriction);

	void solveExt1D(const PxSolverConstraintDesc& desc, Vec3V& linVel0, Vec3V& linVel1, Vec3V& angVel0, Vec3V& angVel1,
		Vec3V& li0, Vec3V& li1, Vec3V& ai0, Vec3V& ai1);

	void solveExt1D(const PxSolverConstraintDesc& desc, Vec3V& linVel0, Vec3V& linVel1, Vec3V& angVel0, Vec3V& angVel1,
		const Vec3V& linMotion0, const Vec3V& linMotion1, const Vec3V& angMotion0, const Vec3V& angMotion1,
		const QuatV& rotA, const QuatV& rotB, const PxReal elapsedTimeF32, Vec3V& linImpulse0, Vec3V& linImpulse1, Vec3V& angImpulse0,
		Vec3V& angImpulse1);

	void solveExtContactStep(const PxSolverConstraintDesc& desc, Vec3V& linVel0, Vec3V& linVel1, Vec3V& angVel0, Vec3V& angVel1,
		Vec3V& linDelta0, Vec3V& linDelta1, Vec3V& angDelta0, Vec3V& angDelta1, Vec3V& linImpulse0, Vec3V& linImpulse1, Vec3V& angImpulse0, Vec3V& angImpulse1,
		bool doFriction, const PxReal minPenetration, const PxReal elapsedTimeF32);

	void solveStaticConstraint(const PxSolverConstraintDesc& desc, Cm::SpatialVectorF& linkV,
		Cm::SpatialVectorF& impulse, Cm::SpatialVectorF& deltaV, const Cm::SpatialVectorF& motion,
		const PxQuat& rot, bool isTGS, PxReal elapsedTime, const PxReal minPenetration)
	{
		Vec3V linVel = V3LoadA(linkV.bottom);
		Vec3V angVel = V3LoadA(linkV.top);

		Vec3V linVel0, linVel1, angVel0, angVel1;
		Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

		if (isTGS)
		{
			PxQuat idt(PxIdentity);
			Vec3V linMotion0, angMotion0, linMotion1, angMotion1;
			QuatV rotA, rotB;
			if (desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY)
			{
				linVel0 = linVel;
				angVel0 = angVel;
				linMotion0 = V3LoadA(motion.bottom);
				angMotion0 = V3LoadA(motion.top);
				rotA = QuatVLoadU(&rot.x);
				rotB = QuatVLoadU(&idt.x);
				linVel1 = angVel1 = linMotion1 = angMotion1 = V3Zero();
			}
			else
			{
				linVel1 = linVel;
				angVel1 = angVel;
				linMotion1 = V3LoadA(motion.bottom);
				angMotion1 = V3LoadA(motion.top);
				rotB = QuatVLoadU(&rot.x);
				rotA = QuatVLoadU(&idt.x);
				linVel0 = angVel0 = linMotion0 = angMotion0 = V3Zero();
			}

			if (*desc.constraint == DY_SC_TYPE_EXT_CONTACT)
			{
				Dy::solveExtContactStep(desc, 
					linVel0, linVel1, angVel0, angVel1, 
					linMotion0, linMotion1, angMotion0, angMotion1,
					li0, li1, ai0, ai1, 
					true, minPenetration, elapsedTime);
			}
			else
			{
				Dy::solveExt1D(desc, 
					linVel0, linVel1, angVel0, angVel1, 
					linMotion0, linMotion1, angMotion0, angMotion1, 
					rotA, rotB, elapsedTime,
					li0, li1, ai0, ai1);
			}
		}
		else
		{
			if (desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY)
			{
				linVel0 = linVel;
				angVel0 = angVel;
				linVel1 = angVel1 = V3Zero();
			}
			else
			{
				linVel1 = linVel;
				angVel1 = angVel;
				linVel0 = angVel0 = V3Zero();
			}

			if (*desc.constraint == DY_SC_TYPE_EXT_CONTACT)
			{
				Dy::solveExtContact(desc, linVel0, linVel1, angVel0, angVel1, li0, li1, ai0, ai1, true);
			}
			else
			{
				Dy::solveExt1D(desc, linVel0, linVel1, angVel0, angVel1, li0, li1, ai0, ai1);
			}
		}

		Cm::SpatialVectorF newVel, newImp;

		if (desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY)
		{
			V4StoreA(Vec4V_From_Vec3V(linVel0), &newVel.bottom.x);
			V4StoreA(Vec4V_From_Vec3V(angVel0), &newVel.top.x);

			V4StoreA(Vec4V_From_Vec3V(li0), &newImp.top.x);
			V4StoreA(Vec4V_From_Vec3V(ai0), &newImp.bottom.x);
		}
		else
		{
			V4StoreA(Vec4V_From_Vec3V(linVel1), &newVel.bottom.x);
			V4StoreA(Vec4V_From_Vec3V(angVel1), &newVel.top.x);

			V4StoreA(Vec4V_From_Vec3V(li1), &newImp.top.x);
			V4StoreA(Vec4V_From_Vec3V(ai1), &newImp.bottom.x);
		}

		deltaV.top += (newVel.top - linkV.top);
		deltaV.bottom += (newVel.bottom - linkV.bottom);
		linkV.top = newVel.top;
		linkV.bottom = newVel.bottom;
		impulse -= newImp;
	}

	// Forward declarations
	void writeBackContact(const PxSolverConstraintDesc& desc, SolverContext& cache,
			PxSolverBodyData& bd0, PxSolverBodyData& bd1);
	void writeBackContact(const PxSolverConstraintDesc& desc, SolverContext* cache);
	void writeBack1D(const PxSolverConstraintDesc& desc);
	void writeBack1DStep(const PxSolverConstraintDesc& desc);

	void FeatherstoneArticulation::writebackInternalConstraints(bool isTGS)
	{
		SolverContext context;
		PxSolverBodyData data;

		for (PxU32 i = 0; i < mStatic1DConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStatic1DConstraints[i];

			PX_ASSERT(*desc.constraint == DY_SC_TYPE_EXT_1D);

			if (isTGS)
			{
				writeBack1DStep(desc);
			}
			else
			{
				writeBack1D(desc);
			}
		}

		for (PxU32 i = 0; i < mStaticContactConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStaticContactConstraints[i];

			PX_ASSERT(*desc.constraint == DY_SC_TYPE_EXT_CONTACT);
			if (isTGS)
			{
				writeBackContact(static_cast<PxSolverConstraintDesc&>(desc), NULL);
			}
			else
			{
				writeBackContact(desc, context, data, data);
			}
		}
	}

	// Forward declarations
	void concludeContact(const PxSolverConstraintDesc& desc);
	void concludeContactStep(const PxSolverConstraintDesc& desc);
	void conclude1D(const PxSolverConstraintDesc& desc);
	void conclude1DStep(const PxSolverConstraintDesc& desc);

	void FeatherstoneArticulation::concludeInternalConstraints(bool isTGS)
	{
		for (PxU32 i = 0; i < mStatic1DConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStatic1DConstraints[i];

			PX_ASSERT(*desc.constraint == DY_SC_TYPE_EXT_1D);
			if (isTGS)
			{
				conclude1DStep(desc);
			}
			else
			{
				conclude1D(desc);
			}
		}

		for (PxU32 i = 0; i < mStaticContactConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStaticContactConstraints[i];
			PX_ASSERT(*desc.constraint == DY_SC_TYPE_EXT_CONTACT);

			if (isTGS)
			{
				concludeContactStep(desc);
			}
			else
			{
				concludeContact(desc);
			}
		}
	}

	struct InternalConstraintSolverData
	{
		const PxReal dt;
		const PxReal stepDt;
		const PxReal invStepDt;
		const PxReal elapsedTime;
		const PxReal erp;
		const bool isTGS;
		const bool isVelIter;
		const bool isExternalForceEveryStep;
		const bool doStaticContactAnd1dConstraint;
		const bool doFrictionDrivePosLimit;
		const ArticulationConstraintProcessingConfig::VelLimit::Enum doVelLimit;
 
		InternalConstraintSolverData(const PxReal dt_, const PxReal stepDt_, const PxReal invStepDt_, const PxReal elapsedTime_, const PxReal erp_,
									const bool isTGS_, const bool isVelIter_,
									const bool isExternalForcesEveryTgsIterationEnabled_,
									const bool doStaticContactAnd1dConstraint_,
									const bool doFrictionDrivePosLimit_,
									const ArticulationConstraintProcessingConfig::VelLimit::Enum doVelLimit_
									)
			: dt(dt_)
			, stepDt(stepDt_)
			, invStepDt(invStepDt_)
			, elapsedTime(elapsedTime_)
			, erp(erp_)
			, isTGS(isTGS_)
			, isVelIter(isVelIter_)
			, isExternalForceEveryStep(isExternalForcesEveryTgsIterationEnabled_)
			, doStaticContactAnd1dConstraint(doStaticContactAnd1dConstraint_)
			, doFrictionDrivePosLimit(doFrictionDrivePosLimit_)
			, doVelLimit(doVelLimit_)
		{
		}

		PX_NOCOPY(InternalConstraintSolverData)
	};

	static void accumulateLinkImpulsesAndLinkVelocities(
		const PxReal deltaF, const ArticulationInternalConstraint& constraint, 
		Cm::UnAlignedSpatialVector& i0, Cm::SpatialVectorF& i1, 
		Cm::SpatialVectorF& parentV, Cm::SpatialVectorF& childV, 
		Cm::SpatialVectorF& dv1)
	{
		i0 += constraint.row0 * deltaF;
		i1.top -= constraint.row1.top * deltaF;
		i1.bottom -= constraint.row1.bottom * deltaF;

		const Cm::UnAlignedSpatialVector deltaVP = constraint.deltaVA * (-deltaF);
		const Cm::UnAlignedSpatialVector deltaVC = constraint.deltaVB * (-deltaF);

		parentV += Cm::SpatialVectorF(deltaVP.top, deltaVP.bottom);
		childV += Cm::SpatialVectorF(deltaVC.top, deltaVC.bottom);

		dv1.top += deltaVC.top;
		dv1.bottom += deltaVC.bottom;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::solveInternalJointConstraintRecursive
	(const InternalConstraintSolverData& PX_RESTRICT data, PxU32 linkID,
	 const Cm::SpatialVectorF& PX_RESTRICT parentDeltaV, PxU32& dofId, PxU32& limitId)
	{
		//PxU32 linkID = stack[stackSize];
		const ArticulationLink* links = mArticulationData.mLinks;
		const ArticulationLink& link = links[linkID];
	
		const PxU32 startDofId = dofId;

		const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

		PxReal* deferredQstZ = mArticulationData.getDeferredQstZ();

		const Cm::SpatialVectorF* PX_RESTRICT motionVelocities = mArticulationData.getMotionVelocities();

		//We know the absolute parentDeltaV from the call to this function so no need to modify it. 
		Cm::SpatialVectorF parentV = parentDeltaV + motionVelocities[link.parent];

		const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();
		const Cm::SpatialVectorF* PX_RESTRICT isW = mArticulationData.getIsW();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		Cm::SpatialVectorF parentVelContrib = propagateAccelerationW(
			rw[linkID], parentDeltaV, 
			invStIS[linkID], &worldMotionMatrix[jointDatum.jointOffset], 
			&isW[jointDatum.jointOffset], &deferredQstZ[jointDatum.jointOffset], jointDatum.nbDof,
			NULL);

		Cm::SpatialVectorF childV = motionVelocities[linkID] + parentVelContrib;

		Cm::UnAlignedSpatialVector i0(PxVec3(0.f), PxVec3(0.f));
		Cm::SpatialVectorF i1(PxVec3(0.f), PxVec3(0.f));

		Cm::SpatialVectorF dv1 = parentVelContrib;

		//Process internal constraints (parent/child limits/locks/drives)
		if(data.doFrictionDrivePosLimit || (ArticulationConstraintProcessingConfig::VelLimit::eBEFORE_STATIC_CONSTRAINTS == data.doVelLimit))
		{
			for (PxU32 dof = 0; dof < jointDatum.nbDof; ++dof)
			{
				if (mArticulationData.mInternalConstraints.size() <= (startDofId + dof))
					continue;

				ArticulationInternalConstraint& constraint = mArticulationData.mInternalConstraints[startDofId + dof];

				// This jointV is just used to compute velocity-dependent forces (friction, limits, ...). It is not stored to the articulation joint velocities.
				PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);

				PxReal frictionDeltaF = 0.0f;
				PxReal driveDeltaF = 0.0f;
				PxReal posLimitDeltaF = 0.0f;
				if(data.doFrictionDrivePosLimit)
				{
					const PxReal jointPDelta = constraint.row1.innerProduct(mArticulationData.mDeltaMotionVector[linkID]) - constraint.row0.innerProduct(mArticulationData.mDeltaMotionVector[link.parent]);

					const bool newFrictionModel = constraint.staticFrictionEffort !=  0.0f || constraint.viscousFrictionCoefficient !=  0.0f;
					const bool isPerStep = (data.isTGS && data.isExternalForceEveryStep && !(data.isVelIter));
					const PxReal effectiveTimestep = isPerStep ? data.stepDt : data.dt;

					// deprecated friction model
					if (!newFrictionModel)
					{
						// Friction force is accumulated through all position iterations only for PGS
						const PxReal appliedFriction = data.isTGS ? 0.0f : constraint.accumulatedFrictionImpulse;
						const PxReal frictionForce = PxClamp(-jointV * constraint.recipResponse + appliedFriction,
															-constraint.deprecatedFrictionMaxForce, constraint.deprecatedFrictionMaxForce);
						constraint.accumulatedFrictionImpulse = frictionForce; // This is not used for TGS

						frictionDeltaF = frictionForce - appliedFriction;

						jointV += frictionDeltaF * constraint.response;
					}

					if (constraint.envelope.maxEffort > 0.0f)
					{
						const PxReal appliedDriveImpulse = isPerStep ? 0.0f : constraint.driveImpulse;
						const PxReal maxImpulse = constraint.envelope.maxEffort * effectiveTimestep;
						const PxReal speedImpulseGradient = constraint.envelope.speedEffortGradient / effectiveTimestep;
						const PxReal velocityDependentResistance = constraint.envelope.velocityDependentResistance * effectiveTimestep;
						const PxReal externalJointImpulse = constraint.externalJointForce * effectiveTimestep;

						const PxReal unclampedImpulse = (data.isTGS && data.isVelIter)
							? appliedDriveImpulse
							: computeDriveImpulse(appliedDriveImpulse, jointV, jointPDelta, data.elapsedTime, constraint.getImplicitDriveDesc());

						const PxReal clampedImpulse = clampDriveImpulse(
							jointV,
							appliedDriveImpulse + externalJointImpulse,
							unclampedImpulse + externalJointImpulse,
							constraint.response,
							constraint.envelope.maxActuatorVelocity,
							maxImpulse,
							speedImpulseGradient,
							velocityDependentResistance
						) - externalJointImpulse;

						driveDeltaF = clampedImpulse - appliedDriveImpulse;
						constraint.driveImpulse += driveDeltaF;
					}
					else
					{
						const PxReal unclampedImpulse = (data.isTGS && data.isVelIter) ? constraint.driveImpulse : 
							computeDriveImpulse(constraint.driveImpulse, jointV, jointPDelta, data.elapsedTime, constraint.getImplicitDriveDesc());
						const PxReal clampedImpulse = PxClamp(unclampedImpulse, -constraint.driveMaxImpulse, constraint.driveMaxImpulse);
						driveDeltaF = (clampedImpulse - constraint.driveImpulse);
						constraint.driveImpulse = clampedImpulse;
					}
					jointV += driveDeltaF * constraint.response;
		
					if (newFrictionModel)
					{
						const PxReal staticFrictionImpulse = constraint.staticFrictionEffort * effectiveTimestep;
						const PxReal dynamicFrictionImpulse = constraint.dynamicFrictionEffort * effectiveTimestep;
						const PxReal viscousFrictionCoefficient = constraint.viscousFrictionCoefficient * effectiveTimestep;
						const PxReal accumulatedFrictionImpulse = isPerStep ? 0.0f : constraint.accumulatedFrictionImpulse;

						const PxReal totalFrictionImpulse = computeFrictionImpulse(
							accumulatedFrictionImpulse - jointV * constraint.recipResponse,
							staticFrictionImpulse,
							dynamicFrictionImpulse,
							viscousFrictionCoefficient,
							jointV);

						frictionDeltaF = totalFrictionImpulse - accumulatedFrictionImpulse;
						constraint.accumulatedFrictionImpulse += frictionDeltaF; // to keep track of accumulated impulse for velIter in TGS with isExternalForceEveryStep
						jointV += frictionDeltaF * constraint.response;
					}

					//Where we will be next frame - we use this to compute error bias terms to correct limits and drives...

					//printf("LinkID %i driveDeltaV = %f, jointV = %f\n", linkID, driveDeltaF, jointV);

					if (jointDatum.dofLimitMask & (1 << dof))
					{
						ArticulationInternalLimit& limit = mArticulationData.mInternalLimits[limitId++];
						posLimitDeltaF = computeLimitImpulse(
							data.stepDt, data.invStepDt, data.isVelIter,
							constraint.response, constraint.recipResponse, data.erp,
							limit.errorLow, limit.errorHigh,
							jointPDelta, 
							limit.lowImpulse, limit.highImpulse, jointV);
					}
				} //doFrictionDrivePosLimit

				PxReal velLimitDeltaF = 0.0f;
				const PxReal maxJointVel = constraint.maxJointVelocity;
				if ((ArticulationConstraintProcessingConfig::VelLimit::eBEFORE_STATIC_CONSTRAINTS == data.doVelLimit) && (PxAbs(jointV) > maxJointVel))
				{
					PxReal newJointV = PxClamp(jointV, -maxJointVel, maxJointVel);
					velLimitDeltaF = (newJointV - jointV) * constraint.recipResponse;
					jointV = newJointV;
				}

				const PxReal deltaF = frictionDeltaF + driveDeltaF + posLimitDeltaF + velLimitDeltaF;

				if (deltaF != 0.f)
				{
					accumulateLinkImpulsesAndLinkVelocities(deltaF, constraint, i0, i1, parentV, childV, dv1);
				}
			}
		}

		//Cache the impulse arising from internal constraints.
		//We'll subtract this from the total impulse applied later in this function.
		Cm::SpatialVectorF i1Internal = i1;

		//Resolve static constraints and record the change in i1.
		Cm::SpatialVectorF i1FromStaticContactAnd1dConstraints(PxVec3(0.0f, 0.0f, 0.0f), PxVec3(0.0f, 0.0f, 0.0f));
		const PxU32 nbStatic1DConstraints = mArticulationData.mNbStatic1DConstraints[linkID];
		const PxU32 nbStaticContactConstraints = mArticulationData.mNbStaticContactConstraints[linkID];
		const bool processStaticContactAnd1dConstraints = data.doStaticContactAnd1dConstraint && ((nbStatic1DConstraints > 0) || (nbStaticContactConstraints > 0));
		if(processStaticContactAnd1dConstraints)
		{
			const Cm::SpatialVectorF i1BeforeStaticContactAnd1dConstraints = i1;

			const Cm::SpatialVectorF& deltaMotion = mArticulationData.getDeltaMotionVector(linkID);
			const PxQuat& deltaQ = getDeltaQ(linkID);

			PxU32 start1DIdx = mArticulationData.mStatic1DConstraintStartIndex[linkID];
			for (PxU32 i = 0; i < nbStatic1DConstraints; ++i)
			{
				PxSolverConstraintDesc& desc = mStatic1DConstraints[start1DIdx++];
				solveStaticConstraint(
					desc, childV, i1, dv1, deltaMotion, deltaQ, data.isTGS, data.elapsedTime, data.isVelIter ? 0.f : -PX_MAX_F32);
			}

			PxU32 startContactIdx = mArticulationData.mStaticContactConstraintStartIndex[linkID];
			for (PxU32 i = 0; i < nbStaticContactConstraints; ++i)
			{
				PxSolverConstraintDesc& desc = mStaticContactConstraints[startContactIdx++];
				solveStaticConstraint(
					desc, childV, i1, dv1, deltaMotion, deltaQ, data.isTGS, data.elapsedTime, data.isVelIter ? 0.f : -PX_MAX_F32);
			}

			i1FromStaticContactAnd1dConstraints = i1 - i1BeforeStaticContactAnd1dConstraints;
		}

		//Process vel limit of each dof on their own.
		if(ArticulationConstraintProcessingConfig::VelLimit::eAFTER_STATIC_CONSTRAINTS == data.doVelLimit)
		{
			const Cm::SpatialVectorF i1BeforeVelLimit = i1;

			//Work out deltaVParent that is a consequence of applying static contact/constraint impulses to the child link.
			Cm::SpatialVectorF deltaVParent(PxVec3(0.0f, 0.0f, 0.0f), PxVec3(0.0f, 0.0f, 0.0f));
			if(processStaticContactAnd1dConstraints)
			{
				//We know the impulse applied by static contact/constraint to the child link.
				//We also know the current velocity of the child link after static contact/constraint.
				//jointV is no longer up to date because we have not accounted for static contact/constraint impulses
				//by propagating them to the parent link.
				//We need to know the velocity of the parent link so that we may recompute the dof speeds and apply 
				//the dof speed limits.
				//To compute the parent link velocity we need to propagate the static contact/constraint impulse applied to the 
				//child inwards to the parent and then compute the deltaVel arising at the parent. We can add that deltaV to the 
				//last known parent link velocity computed immediately before static contact/constraint.
				const PxVec3& r = rw[linkID];
				const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = mArticulationData.getISInvStIS();
				const Cm::SpatialVectorF* jointDofISInvStISW = &iSInvStIS[jointDatum.jointOffset];
				const Cm::UnAlignedSpatialVector* jointDofMotionMatrixW = &worldMotionMatrix[jointDatum.jointOffset];
				const PxU8 nbDofs = jointDatum.nbDof;
				const Cm::SpatialVectorF propagatedImpulseAtParentW = propagateImpulseW(
					r,
					i1FromStaticContactAnd1dConstraints, NULL, 
					jointDofISInvStISW, jointDofMotionMatrixW, nbDofs, NULL);
				deltaVParent = -getImpulseResponseW(link.parent, mArticulationData, propagatedImpulseAtParentW);
			}

			for (PxU32 dof = 0; dof < jointDatum.nbDof; ++dof)
			{
				if (mArticulationData.mInternalConstraints.size() <= (startDofId + dof))
					continue;

				const ArticulationInternalConstraint& constraint = mArticulationData.mInternalConstraints[startDofId + dof];

				//Note that we need to account for the deltaVParent arising from the static contact/constraint impulse applied
				//to the child.
				PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV + deltaVParent);

				PxReal velLimitDeltaF = 0.0f;
				const PxReal maxJointVel =constraint.maxJointVelocity;
				if (PxAbs(jointV) > maxJointVel)
				{
					PxReal newJointV = PxClamp(jointV, -maxJointVel, maxJointVel);
					velLimitDeltaF = (newJointV - jointV) * constraint.recipResponse;
					jointV = newJointV;
				}

				if (velLimitDeltaF != 0.f)
				{
					accumulateLinkImpulsesAndLinkVelocities(velLimitDeltaF, constraint, i0, i1, parentV, childV, dv1);
				}
			}

			i1Internal += (i1 - i1BeforeVelLimit);
		}

		dofId = startDofId + jointDatum.nbDof;

		PxU32 numChildren = link.mNumChildren;
		PxU32 offset = link.mChildrenStartIndex;

		const TestImpulseResponse* responseMatrix = mArticulationData.getImpulseResponseMatrixWorld();

		for(PxU32 i = 0; i < numChildren; ++i)
		{
			const PxU32 child = offset+i;

			Cm::SpatialVectorF childImp = solveInternalJointConstraintRecursive(data, child, dv1, dofId, limitId);
			i1 += childImp;

			if ((numChildren-i) > 1)
			{
				//Propagate the childImp to my dv1 so that the next constraint gets to see an updated velocity state based
				//on the propagation of the child velocities
				Cm::SpatialVectorF deltaV = responseMatrix[linkID].getLinkDeltaVImpulseResponse(-childImp);
				dv1 += deltaV;
				childV += deltaV;	// PT: is that necessary?
			}
		} 
			
		Cm::SpatialVectorF propagatedImpulseAtParentW;
		{
			//const inputs
			const PxVec3& r = rw[linkID];
			const Cm::SpatialVectorF* PX_RESTRICT iSInvStIS = mArticulationData.getISInvStIS();
			const Cm::SpatialVectorF* jointDofISInvStISW = &iSInvStIS[jointDatum.jointOffset];
			const Cm::UnAlignedSpatialVector* jointDofMotionMatrixW = &worldMotionMatrix[jointDatum.jointOffset];
			const PxU8 nbDofs = jointDatum.nbDof;

			//output
			PxReal* jointDofDeferredQstZ = &deferredQstZ[jointDatum.jointOffset];
			propagatedImpulseAtParentW = propagateImpulseW(
				r,
				i1, 
				NULL, jointDofISInvStISW, jointDofMotionMatrixW, nbDofs,
				jointDofDeferredQstZ);
		}

		Cm::SpatialVectorF* PX_RESTRICT solverLinkSpatialImpulses = mArticulationData.getSolverLinkSpatialImpulses();

		//Accumulate the propagated impulse at the link.
		//Don't forget to subtract the impulse arising from internal constraints.
		//This can be used to compute the link's incoming joint force.
		solverLinkSpatialImpulses[linkID] += (i1 - i1Internal);

		return Cm::SpatialVectorF(i0.top, i0.bottom) + propagatedImpulseAtParentW;
	}

	void FeatherstoneArticulation::solveInternalJointConstraints(const PxReal dt, const PxReal stepDt, const PxReal invStepDt, 
																 bool isVelIter, bool isTGS,
																 bool doFrictionDrivePosLimit, 
																 ArticulationConstraintProcessingConfig::VelLimit::Enum doVelLimit, 
																 bool doStaticContactAnd1dConstraint,
																 const PxReal elapsedTime, const PxReal biasCoefficient,
																 bool isExternalForcesEveryTgsIterationEnabled)
	{
		//const PxU32 count = mArticulationData.getLinkCount();

		const PxReal erp = biasCoefficient;

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		PxU32* static1DConstraintCounts = mArticulationData.mNbStatic1DConstraints;
		PxU32* static1DConstraintStarts = mArticulationData.mStatic1DConstraintStartIndex;

		PxU32* staticContactConstraintCounts = mArticulationData.mNbStaticContactConstraints;
		PxU32* staticContactConstraintStarts = mArticulationData.mStaticContactConstraintStartIndex;

		ArticulationLink* links = mArticulationData.getLinks();
		const Cm::SpatialVectorF* baseVelocities = mArticulationData.getMotionVelocities();
		//PxTransform* transforms = mArticulationData.mPreTransform.begin();		

		//Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

		const PxReal minPenetration = isVelIter ? 0.f : -PX_MAX_F32;

		Cm::SpatialVectorF rootLinkV;
		{
			Cm::SpatialVectorF rootLinkDeltaV(PxVec3(0.f), PxVec3(0.f));

			if (!fixBase)
			{
				//Cm::SpatialVectorF temp =  mArticulationData.getBaseInvSpatialArticulatedInertia() * (-deferredZ[0]);
				//const PxTransform& body2World0 = transforms[0];
				//rootLinkDeltaV = temp.rotate(body2World0);
				//temp is now in world space!
				rootLinkDeltaV = mArticulationData.getBaseInvSpatialArticulatedInertiaW() * -mArticulationData.mRootDeferredZ;
			}

			rootLinkV = rootLinkDeltaV + baseVelocities[0];

			Cm::SpatialVectorF im0 = Cm::SpatialVectorF::Zero();
			{
				const PxU32 nbStatic1DConstraints = static1DConstraintCounts[0];

				if (doStaticContactAnd1dConstraint && nbStatic1DConstraints)
				{
					const Cm::SpatialVectorF& deltaMotion = mArticulationData.getDeltaMotionVector(0);
					const PxQuat& deltaQ = getDeltaQ(0);

					PxU32 startIdx = static1DConstraintStarts[0];
					for (PxU32 i = 0; i < nbStatic1DConstraints; ++i)
					{
						PxSolverConstraintDesc& desc = mStatic1DConstraints[startIdx++];

						solveStaticConstraint(desc, rootLinkV, im0, rootLinkDeltaV, deltaMotion, deltaQ, isTGS, elapsedTime, minPenetration);
					}

					//Impulses and deferredZ are now in world space, not link space!
					/*im0.top = transforms[0].rotateInv(im0.top);
					im0.bottom = transforms[0].rotateInv(im0.bottom);*/
				}

				const PxU32 nbStaticContactConstraints = staticContactConstraintCounts[0];

				if (doStaticContactAnd1dConstraint && nbStaticContactConstraints)
				{
					const Cm::SpatialVectorF& deltaMotion = mArticulationData.getDeltaMotionVector(0);
					const PxQuat& deltaQ = getDeltaQ(0);

					PxU32 startIdx = staticContactConstraintStarts[0];
					for (PxU32 i = 0; i < nbStaticContactConstraints; ++i)
					{
						PxSolverConstraintDesc& desc = mStaticContactConstraints[startIdx++];

						solveStaticConstraint(desc, rootLinkV, im0, rootLinkDeltaV, deltaMotion, deltaQ, isTGS, elapsedTime, minPenetration);
					}

					//Impulses and deferredZ are now in world space, not link space!
					/*im0.top = transforms[0].rotateInv(im0.top);
					im0.bottom = transforms[0].rotateInv(im0.bottom);*/
				}
			}	
			
			//Store the constant that will be used by every dof and limit encountered.
			const InternalConstraintSolverData data(dt, stepDt, invStepDt, elapsedTime, erp, isTGS, isVelIter, isExternalForcesEveryTgsIterationEnabled,
													doStaticContactAnd1dConstraint, doFrictionDrivePosLimit, doVelLimit);

			//Increment dofId and limitId as each is encountered to make sure we stay in sync with the ordering of 
			//ArticulationData::mInternalConstraints and ArticulationData::mInternalLimits.
			PxU32 dofId = 0;
			PxU32 limitId = 0;

			const PxU32 numChildren = links[0].mNumChildren;
			const PxU32 offset = links[0].mChildrenStartIndex;

			for (PxU32 i = 0; i < numChildren; ++i)
			{
				const PxU32 child = offset + i;

				Cm::SpatialVectorF imp = solveInternalJointConstraintRecursive(data, child, rootLinkDeltaV, dofId, limitId);

				im0 += imp;

				//There's an impulse, we have to work out how it impacts our velocity (only if required (we have more children to traverse))!
				if (!fixBase && (numChildren - 1) != 0)
				{
					//Impulses and deltaVs are all now in world space
					rootLinkDeltaV += mArticulationData.getBaseInvSpatialArticulatedInertiaW() * (-imp);
				}
			}

			mArticulationData.mRootDeferredZ += im0;
			mArticulationData.mJointDirty = true;
		}
	}

	void FeatherstoneArticulation::solveInternalSpatialTendonConstraints(bool isTGS)
	{
		if (mArticulationData.mInternalSpatialTendonConstraints.size() == 0)
			return;

		if (isTGS)
		{
			//Update the error terms in the tendons recursively...
			const PxU32 nbTendons = mArticulationData.mNumSpatialTendons;

			for (PxU32 i = 0; i < nbTendons; ++i)
			{
				Dy::ArticulationSpatialTendon* tendon = mArticulationData.mSpatialTendons[i];

				Dy::ArticulationAttachment& attachment = tendon->getAttachment(0);

				//const PxU32 childCount = attachment.childCount;

				//PxReal scale = 1.f / PxReal(childCount);

				PxReal coefficient = attachment.coefficient;

				const PxU32 startLink = tendon->getAttachment(0).linkInd;
				const PxTransform pBody2World = mArticulationData.getAccumulatedPoses()[startLink];
				const PxVec3 pAttachPoint = pBody2World.transform(tendon->getAttachment(0).relativeOffset);

				Dy::ArticulationAttachment* attachments = tendon->getAttachments();
				for (ArticulationAttachmentBitField children = attachments[0].children; children != 0; children &= (children - 1))
				{
					//index of child of link h on path to link linkID
					const PxU32 child = PxLowestSetBit(children);

					updateSpatialTendonConstraintsRecursive(attachments, child, tendon->mOffset*coefficient, pAttachPoint, mArticulationData);
				}
			}
		}

		for (PxU32 i = 0; i < mArticulationData.mInternalSpatialTendonConstraints.size(); ++i)
		{
			ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalSpatialTendonConstraints[i];

			const PxU32 parentID = constraint.linkID0;
			const PxU32 linkID = constraint.linkID1;

			//Cm::SpatialVectorF childDeltaP = mArticulationData.mDeltaMotionVector[linkID];
			//Cm::SpatialVectorF parentDeltaP = mArticulationData.mDeltaMotionVector[parentID];

			//PxReal deltaP = constraint.row1.innerProduct(childDeltaP) - constraint.row0.innerProduct(parentDeltaP);// + deltaErr;

			Cm::SpatialVectorV childVel = pxcFsGetVelocity(linkID);
			Cm::SpatialVectorV parentVel = pxcFsGetVelocity(parentID);

			Cm::UnAlignedSpatialVector childV;
			V3StoreU(childVel.angular, childV.top);
			V3StoreU(childVel.linear, childV.bottom);

			Cm::UnAlignedSpatialVector parentV;
			V3StoreU(parentVel.angular, parentV.top);
			V3StoreU(parentVel.linear, parentV.bottom);

			PxReal error = constraint.restDistance - constraint.accumulatedLength;// + deltaP;

			PxReal limitError = 0.f;
			if (constraint.accumulatedLength > constraint.highLimit)
				limitError = constraint.highLimit - constraint.accumulatedLength;
			if (constraint.accumulatedLength < constraint.lowLimit)
				limitError = constraint.lowLimit - constraint.accumulatedLength;

			PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);

			PX_ASSERT(PxIsFinite(jointV));

			PxReal unclampedForce = 0.0f;
			PxReal unclampedLimitForce = 0.0f;
			PxReal deltaF = 0.0f;
			computeTendonImpulse(
				1.0f, 
				constraint.getTendonImplicitSpringParams(),
				error, jointV, limitError, 
				constraint.appliedForce, constraint.limitAppliedForce,
				unclampedForce, unclampedLimitForce, deltaF);
				
			constraint.appliedForce = unclampedForce;
			constraint.limitAppliedForce = unclampedLimitForce;

			//Accumulate error even if it is zero because the increment of the counter affects the RMS value  
			//Ignore tendons for now
			//(isVelIter ? mInternalErrorAccumulatorVelIter : mInternalErrorAccumulatorPosIter).accumulateErrorLocal(deltaF, constraint.recipResponse);

			if (deltaF != 0.f)
			{
				Cm::UnAlignedSpatialVector i0 = constraint.row0 * -deltaF;
				Cm::UnAlignedSpatialVector i1 = constraint.row1 * deltaF;
				pxcFsApplyImpulses(
					parentID, V3LoadU(i0.top), V3LoadU(i0.bottom), NULL,
					linkID, V3LoadU(i1.top), V3LoadU(i1.bottom), NULL);
			}
		}
	}

	PxVec3 FeatherstoneArticulation::calculateFixedTendonVelocityAndPositionRecursive(
		const ArticulationTendonJoint* tendonJoints, const PxU32 tendonJointID,
		const Cm::SpatialVectorF& parentV, const Cm::SpatialVectorF& parentDeltaV)
	{
		const ArticulationTendonJoint& tendonJoint = tendonJoints[tendonJointID];

		const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(tendonJoint.linkInd);

		const PxReal* PX_RESTRICT deferredQstZ = mArticulationData.getDeferredQstZ();
		const InvStIs* PX_RESTRICT invStIS = mArticulationData.getInvStIS();
		const Cm::SpatialVectorF* PX_RESTRICT isW = mArticulationData.getIsW();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
		const PxVec3* PX_RESTRICT rw = mArticulationData.getRw();

		Cm::SpatialVectorF deltaV = propagateAccelerationW(
			rw[tendonJoint.linkInd], parentDeltaV, 
			invStIS[tendonJoint.linkInd],
			&worldMotionMatrix[jointDatum.jointOffset], 
			&isW[jointDatum.jointOffset], &deferredQstZ[jointDatum.jointOffset], jointDatum.nbDof,
			NULL);

		const Cm::SpatialVectorF* PX_RESTRICT motionVelocities = mArticulationData.getMotionVelocities();
		Cm::SpatialVectorF childV = motionVelocities[tendonJoint.linkInd] + deltaV;

		const PxU32 index = tendonJoint.mConstraintInd;
		ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalFixedTendonConstraints[index];

		const PxU32 childCount = tendonJoint.childCount;

		PxVec3 jointVError;

		const PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);
		const PxReal jointP = mArticulationData.getJointPositions()[tendonJoint.startJointOffset];

		jointVError.x = jointV * tendonJoint.coefficient;

		jointVError.y = jointP * tendonJoint.coefficient;

		//printf("%i: jointPose = %f, jointV = %f, coefficient = %f\n", tendonJoint.linkInd, jointP, jointV, tendonJoint.coefficient);

		jointVError.z = 1.f;

		if (childCount)
		{
			for (ArticulationBitField children = tendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);
				jointVError += calculateFixedTendonVelocityAndPositionRecursive(tendonJoints, child, childV, deltaV);
			}
		}

		return jointVError;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::solveFixedTendonConstraintsRecursive
	(const ArticulationTendonJoint* tendonJoints,const PxU32 tendonJointID,
	 const PxReal lengthError, const PxReal limitError, const PxReal jointV,
	 PxReal& rootImp)
	{
		const ArticulationTendonJoint& tendonJoint = tendonJoints[tendonJointID];

		const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(tendonJoint.linkInd);
			   
		PxU32 index = tendonJoint.mConstraintInd;
		ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalFixedTendonConstraints[index];

		const PxU32 childCount = tendonJoint.childCount;

		// the constraint bias coefficients need to flip signs together with the tendon joint's coefficient
		// in order for the constraint force to point into the correct direction:
		PxReal unclampedForce = 0.0f;
		PxReal unclampedForceLimit = 0.0f;
		PxReal deltaF = 0.0f;
		computeTendonImpulse(
			tendonJoint.recipCoefficient,
			constraint.getTendonImplicitSpringParams(),
			lengthError, jointV, limitError,
			constraint.appliedForce, constraint.limitAppliedForce,
			unclampedForce, unclampedForceLimit, deltaF);

		//Ignore tendons for now
		//(isVelIter ? mInternalErrorAccumulatorVelIter : mInternalErrorAccumulatorPosIter).accumulateErrorLocal(deltaF, constraint.recipResponse);

		constraint.appliedForce = unclampedForce;
		constraint.limitAppliedForce = unclampedForceLimit;

		rootImp += deltaF;
	
		Cm::SpatialVectorF impulse(constraint.row1.top * -deltaF, constraint.row1.bottom * -deltaF);
		const Cm::SpatialVectorF YInt = impulse;
		if (childCount)
		{
			for (ArticulationBitField children = tendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = PxLowestSetBit(children);

				Cm::SpatialVectorF propagatedImpulse = solveFixedTendonConstraintsRecursive(tendonJoints, child, lengthError, limitError, jointV, rootImp);

				impulse.top += propagatedImpulse.top;
				impulse.bottom += propagatedImpulse.bottom;
			}
		}

		Cm::SpatialVectorF* PX_RESTRICT solverLinkSpatialImpulses = mArticulationData.getSolverLinkSpatialImpulses();
		solverLinkSpatialImpulses[tendonJoint.linkInd] += impulse - YInt;

		PxReal* deferredQstZ = mArticulationData.getDeferredQstZ();
		const Cm::UnAlignedSpatialVector* worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
		const Cm::SpatialVectorF* iSInvStIS = mArticulationData.getISInvStIS();
		const PxVec3* rw = mArticulationData.getRw();

		return propagateImpulseW(
			rw[tendonJoint.linkInd], 
			impulse, 
			NULL, &iSInvStIS[jointDatum.jointOffset], &worldMotionMatrix[jointDatum.jointOffset], jointDatum.nbDof, 
			&deferredQstZ[jointDatum.jointOffset]);
	}

	void FeatherstoneArticulation::solveInternalFixedTendonConstraints(bool isTGS)
	{
		PX_UNUSED(isTGS);
		if (mArticulationData.mInternalFixedTendonConstraints.size() == 0)
			return;

		{
			//Update the error terms in the tendons recursively...
			const PxU32 nbTendons = mArticulationData.mNumFixedTendons;

			ArticulationLink* links = mArticulationData.getLinks();
			const Cm::SpatialVectorF* motionVelocities = mArticulationData.getMotionVelocities();

			for (PxU32 i = 0; i < nbTendons; ++i)
			{
				Dy::ArticulationFixedTendon* tendon = mArticulationData.mFixedTendons[i];

				ArticulationTendonJoint* tendonJoints = tendon->getTendonJoints();

				Dy::ArticulationTendonJoint& pTendonJoint = tendonJoints[0];

				//const PxU32 childCount = pTendonJoint.childCount;

				const PxU32 startLink = pTendonJoint.linkInd;

				Cm::SpatialVectorV parentVel = pxcFsGetVelocity(startLink);
				Cm::SpatialVectorF parentV;
				V3StoreU(parentVel.angular, parentV.top);
				V3StoreU(parentVel.linear, parentV.bottom);

				Cm::SpatialVectorF Z(PxVec3(0.f), PxVec3(0.f));

				Cm::SpatialVectorF parentDeltaV = parentV - motionVelocities[startLink];

				PxReal lengthError = 0.0f;
				PxReal limitError = 0.0f;
				PxReal rootVel = 0.0f;
				{
					//sum.x  = sum_i{c_i * jointSpeed_i}
					//sum.y  = sum_i{c_i * jointPos_i}
					//sum.x = nbArticulationJointsInTendon
					PxVec3 sum(0.f);						   
					for (ArticulationAttachmentBitField children = pTendonJoint.children; children != 0; children &= (children - 1))
					{
						//index of child of link h on path to link linkID
						const PxU32 child = PxLowestSetBit(children);
						sum += calculateFixedTendonVelocityAndPositionRecursive(tendonJoints, child, parentV, parentDeltaV);
					}

					const PxReal recipScale = sum.z == 0.f ? 0.f : 1.f / sum.z;
					const PxReal length = sum.y + tendon->mOffset;
					lengthError = (length - tendon->mRestLength)*recipScale;		
					rootVel = sum.x*recipScale;
					limitError = (length < tendon->mLowLimit) ? ((length - tendon->mLowLimit)*recipScale) : ((length > tendon->mHighLimit) ? (length - tendon->mHighLimit)*recipScale : 0.0f);
				}
				

				for (ArticulationAttachmentBitField children = pTendonJoint.children; children != 0; children &= (children - 1))
				{
					//index of child of link h on path to link linkID
					const PxU32 child = PxLowestSetBit(children);
					const ArticulationTendonJoint& tendonJoint = tendonJoints[child];

					const ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalFixedTendonConstraints[tendonJoint.mConstraintInd];

					//KS - TODO - hook up offsets
					PxReal rootImp = 0.0f;
					Cm::SpatialVectorF propagatedImpulse = solveFixedTendonConstraintsRecursive(tendonJoints, child, lengthError, limitError, rootVel, rootImp);

					propagatedImpulse.top += constraint.row0.top * rootImp;
					propagatedImpulse.bottom += constraint.row0.bottom * rootImp;

					Z += propagatedImpulse;
				}
				
				PxReal* deferredQstZ = mArticulationData.getDeferredQstZ();
				const Cm::UnAlignedSpatialVector* worldMotionMatrix = mArticulationData.getWorldMotionMatrix();
				const Cm::SpatialVectorF* iSInvStIS = mArticulationData.getISInvStIS();
				const PxVec3* rw = mArticulationData.getRw();

				for (PxU32 linkID = pTendonJoint.linkInd; linkID; linkID = links[linkID].parent)
				{
					const PxU32 jointOffset = mArticulationData.getJointData(linkID).jointOffset;
					const PxU8 dofCount = mArticulationData.getJointData(linkID).nbDof;

					Z = propagateImpulseW(
							rw[linkID],
							Z, 
							NULL, &iSInvStIS[jointOffset], &worldMotionMatrix[jointOffset], dofCount, 
							&deferredQstZ[jointOffset]);
				}

				mArticulationData.mRootDeferredZ += Z;
				mArticulationData.mJointDirty = true;
			}
		}
	}

	void FeatherstoneArticulation::solveInternalConstraints
	(const PxReal dt, const PxReal stepDt, const PxReal invStepDt, 
	 bool velocityIteration, bool isTGS,
	 const ArticulationConstraintProcessingConfigCPU& articulationConstraintProcessingConfig,
	 const PxReal elapsedTime, const PxReal biasCoefficient,
	 bool isExternalForcesEveryTgsIterationEnabled)
	{
		if(articulationConstraintProcessingConfig.mDoSpatialTendonsFixedTendonsMimicJoints)
		{
			solveInternalSpatialTendonConstraints(isTGS);
			solveInternalFixedTendonConstraints(isTGS);
			solveInternalMimicJointConstraints(stepDt, invStepDt, velocityIteration, isTGS, biasCoefficient);
		}
		solveInternalJointConstraints(
				dt, stepDt, invStepDt, 
				velocityIteration, isTGS, 
				articulationConstraintProcessingConfig.mDoFrictionDrivePosLimit, 
				articulationConstraintProcessingConfig.mDoVelLimit, 
				articulationConstraintProcessingConfig.mDoStaticContactsAnd1dConstraints,
				elapsedTime, 
				biasCoefficient, 
				isExternalForcesEveryTgsIterationEnabled);
	}

	bool FeatherstoneArticulation::storeStaticConstraint(const PxSolverConstraintDesc& desc)
	{
		if (DY_STATIC_CONTACTS_IN_INTERNAL_SOLVER)
		{
			if (desc.constraintType == DY_SC_TYPE_RB_CONTACT)
				mStaticContactConstraints.pushBack(desc);
			else
				mStatic1DConstraints.pushBack(desc);
		}
		return DY_STATIC_CONTACTS_IN_INTERNAL_SOLVER;
	}

	void FeatherstoneArticulation::setRootLinearVelocity(const PxVec3& velocity)
	{
		const ArticulationLink& rLink = mArticulationData.getLink(0);
		rLink.bodyCore->linearVelocity = velocity;
		mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES;
		computeLinkVelocities(mArticulationData);
	}

	void FeatherstoneArticulation::setRootAngularVelocity(const PxVec3& velocity)
	{
		const ArticulationLink& rLink = mArticulationData.getLink(0);
		rLink.bodyCore->angularVelocity = velocity;
		mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES;
		computeLinkVelocities(mArticulationData);
	}

	//This method is for user update the root link transform so we need to
	//fix up other link's position. In this case, we should assume all joint
	//velocity/pose is to be zero
	void FeatherstoneArticulation::teleportRootLink()
	{
		//make sure motionMatrix has been set
		//jcalc(mArticulationData);

		const PxU32 linkCount = mArticulationData.getLinkCount();
		ArticulationLink* PX_RESTRICT links = mArticulationData.getLinks();
		PxReal* PX_RESTRICT jointPositions = mArticulationData.getJointPositions();
		Cm::SpatialVectorF* PX_RESTRICT motionVelocities = mArticulationData.getMotionVelocities();
		const Cm::UnAlignedSpatialVector* PX_RESTRICT motionMatrix = mArticulationData.getMotionMatrix();
		const PxQuat* PX_RESTRICT relativeQuats = mArticulationData.getRelativeQuat();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			const PxTransform oldTransform = link.bodyCore->body2World;

			const ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;

			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			const PxQuat relativeQuat = relativeQuats[linkID];

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				newParentToChild = relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = motionMatrix[jointDatum.jointOffset].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
			{
				const PxVec3& u = motionMatrix[jointDatum.jointOffset].top;

				PxQuat jointRotation = PxQuat(-jPosition[0], u);
				if (jointRotation.w < 0)	//shortest angle.
					jointRotation = -jointRotation;

				newParentToChild = (jointRotation * relativeQuat).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				/*PxVec3 worldAngVel = oldTransform.rotate(link.motionVelocity.top);

				newWorldQ = PxExp(worldAngVel*dt) * oldTransform.q;

				PxQuat newParentToChild2 = (newWorldQ.getConjugate() * joint->relativeQuat * pBody2World.q).getNormalized();

				const PxVec3 e2 = newParentToChild2.rotate(parentOffset);
				const PxVec3 d2 = childOffset;
				r = e2 + d2;*/

				PX_ASSERT(r.isFinite());

				break;
			}
			case PxArticulationJointType::eSPHERICAL:
			{

				//PxVec3 angVel(joint->jointVelocity[0], joint->jointVelocity[1], joint->jointVelocity[2]);
				//PxVec3 worldAngVel = pLink.bodyCore->angularVelocity + oldTransform.rotate(angVel);

				PxVec3 worldAngVel = motionVelocities[linkID].top;

				/*const PxReal eps = 0.001f;
				const PxVec3 dif = worldAngVel - worldAngVel2;
				PX_ASSERT(PxAbs(dif.x) < eps && PxAbs(dif.y) < eps && PxAbs(dif.z) < eps);*/

				newWorldQ = PxExp(worldAngVel) * oldTransform.q;

				newParentToChild = (newWorldQ.getConjugate() * relativeQuat * pBody2World.q).getNormalized();

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());
				break;
			}
			case PxArticulationJointType::eFIX:
			{
				//this is fix joint so joint don't have velocity
				newParentToChild = relativeQuat;

				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				r = e + d;
				break;
			}
			default:
				break;
			}

			PxTransform& body2World = link.bodyCore->body2World;
			body2World.q = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();
			body2World.p = pBody2World.p + body2World.q.rotate(r);

			PX_ASSERT(body2World.isSane());
		}
	}

	PxU8* FeatherstoneArticulation::allocateScratchSpatialData(PxcScratchAllocator* allocator,
		const PxU32 linkCount, ScratchData& scratchData, bool fallBackToHeap)
	{
		const PxU32 size = sizeof(Cm::SpatialVectorF) * linkCount;
		const PxU32 totalSize = size * 4 + sizeof(Dy::SpatialMatrix) * linkCount;

		PxU8* tempMemory = reinterpret_cast<PxU8*>(allocator->alloc(totalSize, fallBackToHeap));

		scratchData.motionVelocities = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory);
		PxU32 offset = size;
		scratchData.motionAccelerations = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.coriolisVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.spatialZAVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.compositeSpatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(tempMemory + offset);

		return tempMemory;
	}

/*	void FeatherstoneArticulation::allocateScratchSpatialData(DyScratchAllocator& allocator,
		const PxU32 linkCount, ScratchData& scratchData)
	{
		const PxU32 size = sizeof(Cm::SpatialVectorF) * linkCount;
		const PxU32 totalSize = size * 5 + sizeof(Dy::SpatialMatrix) * linkCount;

		PxU8* tempMemory = allocator.alloc<PxU8>(totalSize);

		scratchData.motionVelocities = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory);
		PxU32 offset = size;
		scratchData.motionAccelerations = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.coriolisVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.spatialZAVectors = reinterpret_cast<Cm::SpatialVectorF*>(tempMemory + offset);
		offset += size;
		scratchData.externalAccels = reinterpret_cast<Cm::SpatialVector*>(tempMemory + offset);
		offset += size;
		scratchData.compositeSpatialInertias = reinterpret_cast<Dy::SpatialMatrix*>(tempMemory + offset);
		
	}*/

}//namespace Dy
}
