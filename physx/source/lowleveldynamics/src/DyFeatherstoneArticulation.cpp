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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "foundation/PxMathUtils.h"
#include "CmConeLimitHelper.h"
#include "DySolverConstraint1D.h"
#include "DyFeatherstoneArticulation.h"
#include "PxsRigidBody.h"
#include "PxcConstraintBlockStream.h"
#include "DyArticulationContactPrep.h"
#include "DyDynamics.h"
#include "DyArticulationPImpl.h"
#include "DyFeatherstoneArticulationLink.h"
#include "DyFeatherstoneArticulationJointData.h"
#include "DySolverConstraint1DStep.h"
#include "DyTGSDynamics.h"
#include "DyConstraintPrep.h"
#include "common/PxProfileZone.h"
#include "PxsContactManager.h"
#include "DyContactPrep.h"
#include "DySolverContext.h"
#include "DyTGSContactPrep.h"

#ifndef FEATURESTONE_DEBUG
#define FEATURESTONE_DEBUG 0
#endif


// we encode articulation link handles in the lower bits of the pointer, so the
// articulation has to be aligned, which in an aligned pool means we need to size it
// appropriately

namespace physx
{

namespace Dy
{

	extern PxcCreateFinalizeSolverContactMethod createFinalizeMethods[3];

	void SolverCoreRegisterArticulationFns();

	void SolverCoreRegisterArticulationFnsCoulomb();

	ArticulationPImpl::ComputeUnconstrainedVelocitiesFn ArticulationPImpl::sComputeUnconstrainedVelocities = NULL;
	ArticulationPImpl::UpdateBodiesFn ArticulationPImpl::sUpdateBodies = NULL;
	ArticulationPImpl::UpdateBodiesFn ArticulationPImpl::sUpdateBodiesTGS = NULL;
	ArticulationPImpl::SaveVelocityFn ArticulationPImpl::sSaveVelocity = NULL;
	ArticulationPImpl::SaveVelocityTGSFn ArticulationPImpl::sSaveVelocityTGS = NULL;

	ArticulationPImpl::UpdateDeltaMotionFn ArticulationPImpl::sUpdateDeltaMotion = NULL;
	ArticulationPImpl::DeltaMotionToMotionVelFn ArticulationPImpl::sDeltaMotionToMotionVel = NULL;
	ArticulationPImpl::ComputeUnconstrainedVelocitiesTGSFn ArticulationPImpl::sComputeUnconstrainedVelocitiesTGS = NULL;

	ArticulationPImpl::SetupInternalConstraintsTGSFn ArticulationPImpl::sSetupInternalConstraintsTGS = NULL;

	void PxvRegisterArticulationsReducedCoordinate()
	{
		ArticulationPImpl::sComputeUnconstrainedVelocities = &FeatherstoneArticulation::computeUnconstrainedVelocities;
		ArticulationPImpl::sUpdateBodies = &FeatherstoneArticulation::updateBodies;
		ArticulationPImpl::sUpdateBodiesTGS = &FeatherstoneArticulation::updateBodiesTGS;
		ArticulationPImpl::sSaveVelocity = &FeatherstoneArticulation::saveVelocity;
		ArticulationPImpl::sSaveVelocityTGS = &FeatherstoneArticulation::saveVelocityTGS;

		ArticulationPImpl::sUpdateDeltaMotion = &FeatherstoneArticulation::recordDeltaMotion;
		ArticulationPImpl::sDeltaMotionToMotionVel = &FeatherstoneArticulation::deltaMotionToMotionVelocity;
		ArticulationPImpl::sComputeUnconstrainedVelocitiesTGS = &FeatherstoneArticulation::computeUnconstrainedVelocitiesTGS;
		ArticulationPImpl::sSetupInternalConstraintsTGS = &FeatherstoneArticulation::setupSolverConstraintsTGS;

		SolverCoreRegisterArticulationFns();
		SolverCoreRegisterArticulationFnsCoulomb();
	}

	ArticulationData::~ArticulationData()
	{
		PX_FREE(mLinksData);
		PX_FREE(mJointData);
		PX_FREE(mJointTranData);
		PX_FREE(mPathToRootElements);
	}

	void ArticulationData::resizeLinkData(const PxU32 linkCount)
	{
		const PxU32 oldSize = mMotionVelocities.size();
		mMotionVelocities.reserve(linkCount);
		mMotionVelocities.forceSize_Unsafe(linkCount);
		
		mSolverSpatialForces.reserve(linkCount);
		mSolverSpatialForces.forceSize_Unsafe(linkCount);

		mMotionAccelerations.reserve(linkCount);
		mMotionAccelerations.forceSize_Unsafe(linkCount);

		mMotionAccelerationsInternal.reserve(linkCount);
		mMotionAccelerationsInternal.forceSize_Unsafe(linkCount);

		mCorioliseVectors.reserve(linkCount);
		mCorioliseVectors.forceSize_Unsafe(linkCount);

		mZAForces.reserve(linkCount);
		mZAForces.forceSize_Unsafe(linkCount);

		mZAInternalForces.reserve(linkCount);
		mZAInternalForces.forceSize_Unsafe(linkCount);

		mNbStatic1DConstraints.reserve(linkCount);
		mNbStatic1DConstraints.forceSize_Unsafe(linkCount);

		mStatic1DConstraintStartIndex.reserve(linkCount);
		mStatic1DConstraintStartIndex.forceSize_Unsafe(linkCount);

		mNbStaticContactConstraints.reserve(linkCount);
		mNbStaticContactConstraints.forceSize_Unsafe(linkCount);

		mStaticContactConstraintStartIndex.reserve(linkCount);
		mStaticContactConstraintStartIndex.forceSize_Unsafe(linkCount);

		mDeltaMotionVector.reserve(linkCount);
		mDeltaMotionVector.forceSize_Unsafe(linkCount);

		mPreTransform.reserve(linkCount);
		mPreTransform.forceSize_Unsafe(linkCount);

		mResponseMatrixW.reserve(linkCount);
		mResponseMatrixW.forceSize_Unsafe(linkCount);

		mWorldSpatialArticulatedInertia.reserve(linkCount);
		mWorldSpatialArticulatedInertia.forceSize_Unsafe(linkCount);

		mWorldIsolatedSpatialArticulatedInertia.reserve(linkCount);
		mWorldIsolatedSpatialArticulatedInertia.forceSize_Unsafe(linkCount);

		mMasses.reserve(linkCount);
		mMasses.forceSize_Unsafe(linkCount);

		mInvStIs.reserve(linkCount);
		mInvStIs.forceSize_Unsafe(linkCount);

		/*mMotionMatrix.resize(linkCount);

		mWorldMotionMatrix.reserve(linkCount);
		mWorldMotionMatrix.forceSize_Unsafe(linkCount);*/

		mAccumulatedPoses.reserve(linkCount);
		mAccumulatedPoses.forceSize_Unsafe(linkCount);

		mDeltaQ.reserve(linkCount);
		mDeltaQ.forceSize_Unsafe(linkCount);

		mPosIterMotionVelocities.reserve(linkCount);
		mPosIterMotionVelocities.forceSize_Unsafe(linkCount);

		mJointTransmittedForce.reserve(linkCount);
		mJointTransmittedForce.forceSize_Unsafe(linkCount);

		mRw.reserve(linkCount);
		mRw.forceSize_Unsafe(linkCount);

		//This stores how much an impulse on a given link influences the root link.
		//We combine this with the back-propagation of joint forces to compute the 
		//change in velocity of a given impulse!
		mRootResponseMatrix.reserve(linkCount);
		mRootResponseMatrix.forceSize_Unsafe(linkCount);

		mRelativeQuat.resize(linkCount);

		if (oldSize < linkCount)
		{

			ArticulationLinkData* oldLinks = mLinksData;
			ArticulationJointCoreData* oldJoints = mJointData;
			ArticulationJointTargetData* oldJointTran = mJointTranData;

			mLinksData = PX_ALLOCATE(ArticulationLinkData, linkCount, "ArticulationLinkData");
			mJointData = PX_ALLOCATE(ArticulationJointCoreData, linkCount, "ArticulationJointCoreData");
			mJointTranData = PX_ALLOCATE(ArticulationJointTargetData, linkCount, "ArticulationJointTargetData");

			PxMemCopy(mLinksData, oldLinks, sizeof(ArticulationLinkData)*oldSize);
			PxMemCopy(mJointData, oldJoints, sizeof(ArticulationJointCoreData)*oldSize);
			PxMemCopy(mJointTranData, oldJointTran, sizeof(ArticulationJointTargetData)*oldSize);

			PX_FREE(oldLinks);
			PX_FREE(oldJoints);
			PX_FREE(oldJointTran);

			const PxU32 newElems = (linkCount - oldSize);

			PxMemZero(mLinksData + oldSize, sizeof(ArticulationLinkData) * newElems);
			PxMemZero(mJointData + oldSize, sizeof(ArticulationJointCoreData) * newElems);

			for (PxU32 linkID = oldSize; linkID < linkCount; ++linkID)
			{
				PX_PLACEMENT_NEW(mLinksData + linkID, ArticulationLinkData)();
				PX_PLACEMENT_NEW(mJointData + linkID, ArticulationJointCoreData)();
				PX_PLACEMENT_NEW(mJointTranData + linkID, ArticulationJointTargetData)();
			}
		}
	}

	void ArticulationData::resizeJointData(const PxU32 dofs)
	{
		mJointAcceleration.reserve(dofs);
		mJointAcceleration.forceSize_Unsafe(dofs);

		mJointInternalAcceleration.reserve(dofs);
		mJointInternalAcceleration.forceSize_Unsafe(dofs);

		mJointVelocity.reserve(dofs);
		mJointVelocity.forceSize_Unsafe(dofs);

		mJointNewVelocity.reserve(dofs+3);
		mJointNewVelocity.forceSize_Unsafe(dofs+3);

		mJointPosition.reserve(dofs);
		mJointPosition.forceSize_Unsafe(dofs);

		mJointForce.reserve(dofs);
		mJointForce.forceSize_Unsafe(dofs);

		mMotionMatrix.resize(dofs);

		mJointSpaceJacobians.resize(dofs*mLinkCount);
		mJointSpaceDeltaVMatrix.resize(((dofs + 3) / 4)*mLinkCount);
		mJointSpaceResponseMatrix.resize(dofs); //This is only an entry per-dof

		mPropagationAccelerator.resize(dofs);

		mWorldMotionMatrix.reserve(dofs);
		mWorldMotionMatrix.forceSize_Unsafe(dofs);

		mJointAxis.reserve(dofs);
		mJointAxis.forceSize_Unsafe(dofs);

		mIsW.reserve(dofs);
		mIsW.forceSize_Unsafe(dofs);

		mDeferredQstZ.reserve(dofs);
		mDeferredQstZ.forceSize_Unsafe(dofs);

		mJointConstraintForces.resizeUninitialized(dofs);

		qstZIc.reserve(dofs);
		qstZIc.forceSize_Unsafe(dofs);

		qstZIntIc.reserve(dofs);
		qstZIntIc.forceSize_Unsafe(dofs);

		mIsInvDW.reserve(dofs);
		mIsInvDW.forceSize_Unsafe(dofs);

		mPosIterJointVelocities.reserve(dofs);
		mPosIterJointVelocities.forceSize_Unsafe(dofs);

		PxMemZero(mJointAcceleration.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mJointVelocity.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mJointPosition.begin(), sizeof(PxReal) * dofs);
		PxMemZero(mJointForce.begin(), sizeof(PxReal) * dofs);
	}

	ArticulationLinkData& ArticulationData::getLinkData(PxU32 index) const
	{
		PX_ASSERT(index < mLinkCount);
		return mLinksData[index];
	}

	void ArticulationJointCore::setJointFrame(ArticulationJointCoreData& jointDatum, Cm::UnAlignedSpatialVector* motionMatrix, 
		const Cm::UnAlignedSpatialVector* jointAxis, bool forceUpdate, PxQuat& relativeQuat)
	{
		if (jointDirtyFlag & ArticulationJointCoreDirtyFlag::eFRAME || forceUpdate)
		{
			relativeQuat = (childPose.q * (parentPose.q.getConjugate())).getNormalized();

			jointDatum.computeMotionMatrix(this, motionMatrix, jointAxis);

			jointDirtyFlag &= ~ArticulationJointCoreDirtyFlag::eFRAME;
		}
	}

	FeatherstoneArticulation::FeatherstoneArticulation(void* userData)
		: mUserData(userData), mContext(NULL), mUpdateSolverData(true),
		mMaxDepth(0)
	{
		mGPUDirtyFlags = 0;
	}

	FeatherstoneArticulation::~FeatherstoneArticulation()
	{
	}

	void FeatherstoneArticulation::copyJointData(ArticulationData& data, PxReal* toJointData, const PxReal* fromJointData)
	{
		const PxU32 dofCount = data.getDofs();

		PxMemCopy(toJointData, fromJointData, sizeof(PxReal)*dofCount);
	}

	PxU32 FeatherstoneArticulation::computeDofs()
	{
		const PxU32 linkCount = mArticulationData.getLinkCount();
		PxU32 totalDofs = 0;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = mArticulationData.getLink(linkID);
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
			const PxU8 dof = jointDatum.computeJointDofs(link.inboundJoint);
			totalDofs += dof;
		}
		
		return totalDofs;
	}

	bool FeatherstoneArticulation::resize(const PxU32 linkCount)
	{
		if (mUpdateSolverData)
		{
			if (linkCount != mSolverDesc.linkCount)
			{
				mSolverDesc.acceleration = mAcceleration.begin();
				mSolverDesc.articulation = this;
			}

			mUpdateSolverData = false;

			if (linkCount != mSolverDesc.linkCount)
			{

				mArticulationData.resizeLinkData(linkCount);

			}
			return true;
			
		}
		return false;
	}

	void FeatherstoneArticulation::getDataSizes(PxU32 /*linkCount*/, PxU32& solverDataSize, PxU32& totalSize, PxU32& scratchSize)
	{		
		solverDataSize = 0;
		totalSize = 0;
		scratchSize = 0;
	}

	void FeatherstoneArticulation::setupLinks(PxU32 nbLinks, Dy::ArticulationLink* links)
	{
		//if this is needed, we need to re-allocated the link data
		resize(nbLinks);

		mSolverDesc.links = links;
		mSolverDesc.linkCount = PxTo8(nbLinks);

		mArticulationData.mLinks				= links;
		mArticulationData.mLinkCount			= PxTo8(nbLinks);
		mArticulationData.mFlags				= mSolverDesc.core ? &mSolverDesc.core->flags : mSolverDesc.flags;	// PT: PX-1399
		mArticulationData.mExternalAcceleration	= mSolverDesc.acceleration;
		mArticulationData.mArticulation			= this;

		
		//allocate memory for articulation data
		PxU32 totalDofs = computeDofs();

		const PxU32 existedTotalDofs = mArticulationData.getDofs();

		if (totalDofs != existedTotalDofs)
		{
			mArticulationData.resizeJointData(totalDofs + 1);
			mArticulationData.setDofs(totalDofs);
		}
	}

	void FeatherstoneArticulation::allocatePathToRootElements(const PxU32 totalPathToRootElements)
	{
		if (mArticulationData.mNumPathToRootElements < totalPathToRootElements)
		{
			mArticulationData.mPathToRootElements = PX_ALLOCATE(PxU32, totalPathToRootElements, "PxU32");
			mArticulationData.mNumPathToRootElements = totalPathToRootElements;
		}
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

		allocatePathToRootElements(totalPathToRootCount);

		PxU32* pathToRootElements = mArticulationData.getPathToRootElements();

		pathToRootElements[0] = 0; //add on root index

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			Dy::ArticulationLink& link = links[linkID];
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

	void FeatherstoneArticulation::assignSensors(const PxU32 nbSensors, Dy::ArticulationSensor** sensors, PxSpatialForce* sensorForces)
	{
		mArticulationData.mSensors = sensors;
		mArticulationData.mNbSensors = nbSensors;
		mArticulationData.mSensorForces = sensorForces;
	}

	PxU32 FeatherstoneArticulation::getDofs()
	{
		return mArticulationData.getDofs();
	
	}

	PxU32 FeatherstoneArticulation::getDof(const PxU32 linkID)
	{
		const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
		return jointDatum.dof;
	}


	bool FeatherstoneArticulation::applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag, bool& shouldWake)
	{
		return applyCacheToDest(mArticulationData, cache, mArticulationData.getJointVelocities(), mArticulationData.getJointAccelerations(),
			mArticulationData.getJointPositions(), mArticulationData.getJointForces(), flag, shouldWake);
	}

	void FeatherstoneArticulation::copyInternalStateToCache(PxArticulationCache& cache,
		const PxArticulationCacheFlags flag)
	{
		if (flag & PxArticulationCacheFlag::eVELOCITY)
		{
			copyJointData(mArticulationData, cache.jointVelocity, mArticulationData.getJointVelocities());
		}

		if (flag & PxArticulationCacheFlag::eACCELERATION)
		{
			copyJointData(mArticulationData, cache.jointAcceleration, mArticulationData.getJointAccelerations());
		}

		if (flag & PxArticulationCacheFlag::ePOSITION)
		{
			copyJointData(mArticulationData, cache.jointPosition, mArticulationData.getJointPositions());
		}

		if (flag & PxArticulationCacheFlag::eFORCE)
		{
			copyJointData(mArticulationData, cache.jointForce, mArticulationData.getJointForces());
		}

		if (flag & PxArticulationCacheFlag::eJOINT_SOLVER_FORCES)
		{
			copyJointData(mArticulationData, cache.jointSolverForces, mArticulationData.getJointConstraintForces());
		}

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

		if (flag & PxArticulationCacheFlag::eLINK_ACCELERATION)
		{
			const PxU32 numLinks = mArticulationData.getLinkCount();

			if (mArticulationData.getDt() > 0.f)
				recomputeAccelerations(mArticulationData.getDt());

			const Cm::SpatialVectorF* accels = mArticulationData.getMotionAccelerations();

			for (PxU32 i = 0; i < numLinks; ++i)
			{
				const Cm::SpatialVectorF& accel = accels[i];
				cache.linkAcceleration[i].linear = accel.bottom;
				cache.linkAcceleration[i].angular = accel.top;
			}			
		}

		if (flag & PxArticulationCacheFlag::eROOT_TRANSFORM)
		{
			const ArticulationLink& rLink = mArticulationData.getLink(0);
			const PxsBodyCore& rBodyCore = *rLink.bodyCore;
			const PxTransform& body2World = rBodyCore.body2World;
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

		if (flag & PxArticulationCacheFlag::eSENSOR_FORCES)
		{
			const PxU32 nbSensors = mArticulationData.mNbSensors;
			PxMemCopy(cache.sensorForces, mArticulationData.mSensorForces, sizeof(PxSpatialForce)*nbSensors);
		}
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

	void FeatherstoneArticulation::transformInertia(const SpatialTransform& sTod, SpatialMatrix& spatialInertia)
	{
#if 1
		const SpatialTransform dTos = sTod.getTranspose();

		Mat33V tL = loadPxMat33(spatialInertia.topLeft);
		Mat33V tR = loadPxMat33(spatialInertia.topRight);
		Mat33V bL = loadPxMat33(spatialInertia.bottomLeft);

		Mat33V R = loadPxMat33(sTod.R);
		Mat33V T = loadPxMat33(sTod.T);

		Mat33V tl = M33MulM33(R, tL);
		Mat33V tr = M33MulM33(R, tR);
		Mat33V bl = M33Add(M33MulM33(T, tL), M33MulM33(R, bL));
		Mat33V br = M33Add(M33MulM33(T, tR), M33MulM33(R, M33Trnsps(tL)));

		Mat33V dR = loadPxMat33(dTos.R);
		Mat33V dT = loadPxMat33(dTos.T);

		tL = M33Add(M33MulM33(tl, dR), M33MulM33(tr, dT));
		tR = M33MulM33(tr, dR);
		bL = M33Add(M33MulM33(bl, dR), M33MulM33(br, dT));

		bL = M33Scale(M33Add(bL, M33Trnsps(bL)), FHalf());

		storePxMat33(tL, spatialInertia.topLeft);
		storePxMat33(tR, spatialInertia.topRight);
		storePxMat33(bL, spatialInertia.bottomLeft);
#else
		const SpatialTransform dTos = sTod.getTranspose();

		PxMat33 tl = sTod.R * spatialInertia.topLeft;
		PxMat33 tr = sTod.R * spatialInertia.topRight;
		PxMat33 bl = sTod.T * spatialInertia.topLeft + sTod.R * spatialInertia.bottomLeft;
		PxMat33 br = sTod.T * spatialInertia.topRight + sTod.R * spatialInertia.getBottomRight();

		spatialInertia.topLeft = tl * dTos.R + tr * dTos.T;
		spatialInertia.topRight = tr * dTos.R;
		spatialInertia.bottomLeft = bl * dTos.R + br * dTos.T;

		//aligned inertia
		spatialInertia.bottomLeft = (spatialInertia.bottomLeft + spatialInertia.bottomLeft.getTranspose()) * 0.5f;
#endif
	}

	PxMat33 FeatherstoneArticulation::translateInertia(const PxMat33& inertia, const PxReal mass, const PxVec3& t)
	{
		PxMat33 s(PxVec3(0, t.z, -t.y),
			PxVec3(-t.z, 0, t.x),
			PxVec3(t.y, -t.x, 0));

		PxMat33 translatedIT = s.getTranspose() * s * mass + inertia;
		return translatedIT;
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
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse,
		Cm::SpatialVector& deltaVV) const
	{
		PX_UNUSED(Z);
		PX_ASSERT(impulse.pad0 == 0.f && impulse.pad1 == 0.f);

		//impulse lin is contact normal, and ang is raxn. R is body2World, R(t) is world2Body
		//| R(t),	0	|
		//| R(t)*r, R(t)|
		//r is the vector from center of mass to contact point
		//p(impluse) =	|n|
		//				|0|

#if 0
		const PxTransform& body2World = mArticulationData.getPreTransform(linkID);

		//transform p(impulse) from world space to the local space of linkId
		const Cm::SpatialVectorF impl(body2World.rotateInv(impulse.linear), body2World.rotateInv(impulse.angular));

		getZ(linkID, mArticulationData, Z, impl);

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		
		const Cm::SpatialVectorF deltaV = getDeltaV(fixBase, linkID, mArticulationData, Z);
		PX_ASSERT(deltaV.pad0 == 0.f && deltaV.pad1 == 0.f);
		
		////Cm::SpatialVectorF resp = mArticulationData.getImpulseResponseMatrix()[linkID].getResponse(Cm::SpatialVectorF(impulse.linear, impulse.angular));
		//Cm::SpatialVectorF resp = mArticulationData.getImpulseResponseMatrix()[linkID].getResponse(impl);

		//Cm::SpatialVectorF test = resp - deltaV;

		//PX_ASSERT(test.magnitude() < 1e-5f);

		//this is in world space
		deltaVV.linear = body2World.rotate(deltaV.bottom);
		deltaVV.angular = body2World.rotate(deltaV.top);

#else
		Cm::SpatialVectorF deltaV = mArticulationData.getImpulseResponseMatrixWorld()[linkID].getResponse(reinterpret_cast<const Cm::SpatialVectorF&>(impulse));

		deltaVV.linear = deltaV.bottom;
		deltaVV.angular = deltaV.top;		
#endif

		
	}

	void FeatherstoneArticulation::getImpulseResponse(
		PxU32 linkID,
		Cm::SpatialVectorV* /*Z*/,
		const Cm::SpatialVectorV& impulse,
		Cm::SpatialVectorV& deltaVV) const
	{
#if 0
		const PxTransform& body2World = mArticulationData.getPreTransform(linkID);

		QuatV rot = QuatVLoadU(&body2World.q.x);

		Cm::SpatialVectorV impl(QuatRotateInv(rot, impulse.linear), QuatRotateInv(rot, impulse.angular));

		//transform p(impluse) from world space to the local space of linkId

		//Cm::SpatialVectorF impl(impulse.linear, impulse.angular);
		Cm::SpatialVectorV deltaV = mArticulationData.getImpulseResponseMatrix()[linkID].getResponse(impl);
		deltaVV.linear = QuatRotate(rot, deltaV.angular);
		deltaVV.angular = QuatRotate(rot, deltaV.linear);
#else
		Cm::SpatialVectorV deltaV = mArticulationData.getImpulseResponseMatrixWorld()[linkID].getResponse(impulse);
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

	Cm::SpatialVector FeatherstoneArticulation::getMotionAcceleration(const PxU32 linkID) const
	{
		const PxReal dt = mArticulationData.getDt();
		if(0.0f == dt)
			return Cm::SpatialVector(PxVec3(0.f), PxVec3(0.f));
		return recomputeAcceleration(linkID, dt);
	}


	void FeatherstoneArticulation::fillIndexType(const PxU32 linkId, PxU8& indexType)
	{
		ArticulationLink& link = mArticulationData.getLink(linkId);

		//turn the kinematic link to static for the solver
		if (link.bodyCore->kinematicLink)
		{
			indexType = PxsIndexedInteraction::eWORLD;
		}
		else
		{
			indexType = PxsIndexedInteraction::eARTICULATION;
		}
	}

	PxReal FeatherstoneArticulation::getLinkMaxPenBias(const PxU32 linkID) const
	{
		return mArticulationData.getLinkData(linkID).maxPenBias;
	}

	PxReal FeatherstoneArticulation::getCfm(const PxU32 linkID) const
	{
		return mArticulationData.getLink(linkID).cfm;
	}

	void PxcFsFlushVelocity(FeatherstoneArticulation& articulation, Cm::SpatialVectorF* deltaV, bool computeForces)
	{
		PX_ASSERT(deltaV);

		ArticulationData& data = articulation.mArticulationData;
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();
		//Cm::SpatialVectorF* deferredZ = data.getSpatialZAVectors();
		ArticulationLink* links = data.getLinks();
		ArticulationJointCoreData* jointData = data.getJointData();

		//PxTransform* poses = data.getAccumulatedPoses();
		//const PxTransform* poses = data.getPreTransform();

		//This will be zero at the begining of the frame
		PxReal* jointNewVelocities = data.getJointNewVelocities();

		data.getSolverSpatialForce(0) -= data.getRootDeferredZ();

		if (fixBase)
		{
			deltaV[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{
			//ArticulationLink& link = links[0];

			deltaV[0] = data.getBaseInvSpatialArticulatedInertiaW() * -data.getRootDeferredZ();

			motionVelocities[0] += deltaV[0];

			PX_ASSERT(motionVelocities[0].isFinite());
		}

		const PxU32 linkCount = data.getLinkCount();

		for (PxU32 i = 1; i < linkCount; i++)
		{
			const ArticulationLink& tLink = links[i];
			const ArticulationJointCoreData& tJointDatum = jointData[i];

			const Cm::SpatialVectorF dV = FeatherstoneArticulation::propagateAccelerationW(data.getRw(i), data.getInvStIs(i),
				&data.getWorldMotionMatrix(tJointDatum.jointOffset), &jointNewVelocities[tJointDatum.jointOffset], deltaV[tLink.parent], tJointDatum.dof,
				&data.getIsW(tJointDatum.jointOffset), &data.getDeferredQstZ()[tJointDatum.jointOffset]);

			deltaV[i] = dV;
			motionVelocities[i] += dV;

			//Cm::SpatialVectorF& v = motionVelocities[i];
			//printf("linkID %i motionV(%f, %f, %f, %f, %f, %f)\n", i, v.top.x, v.top.y, v.top.z, v.bottom.x, v.bottom.y, v.bottom.z);

			/*if(computeForces)
				data.getSolverSpatialForce(i) += data.getWorldSpatialArticulatedInertia(i) * dV;*/
			if (computeForces)
				data.getSolverSpatialForce(i) += dV;

			PX_ASSERT(motionVelocities[i].isFinite());

		}

		//PxMemZero(deferredZ, sizeof(Cm::SpatialVectorF)*linkCount);
		PxMemZero(data.getDeferredQstZ(), sizeof(PxReal) * data.getDofs());

		data.getRootDeferredZ() = Cm::SpatialVectorF::Zero();
	}

	void FeatherstoneArticulation::recordDeltaMotion(const ArticulationSolverDesc& desc, 
		const PxReal dt, Cm::SpatialVectorF* deltaV, const PxReal /*totalInvDt*/)
	{
		PX_ASSERT(deltaV);

		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		const PxU32 linkCount = data.getLinkCount();

		const PxU32 flags = data.getArticulationFlags();

		if (data.mJointDirty)
		{
			bool doForces = (flags & PxArticulationFlag::eCOMPUTE_JOINT_FORCES) || data.getSensorCount();
			PxcFsFlushVelocity(*articulation, deltaV, doForces);
		}

		Cm::SpatialVectorF* deltaMotion = data.getDeltaMotionVector();
		Cm::SpatialVectorF* posMotionVelocities = data.getPosIterMotionVelocities();
		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();

		PxReal* jointPosition = data.getJointPositions();
		PxReal* jointNewVelocities = data.getJointNewVelocities();
		
		//data.mAccumulatedDt += dt;
		data.setDt(dt);

		const bool fixBase = flags & PxArticulationFlag::eFIX_BASE;

		if (!fixBase)
		{
			Cm::SpatialVectorF& motionVelocity = motionVelocities[0];
			PX_ASSERT(motionVelocity.top.isFinite());
			PX_ASSERT(motionVelocity.bottom.isFinite());

			const PxTransform preTrans = data.mAccumulatedPoses[0];

			const PxVec3 lin = motionVelocity.bottom;
			const PxVec3 ang = motionVelocity.top;

			const PxVec3 newP = preTrans.p + lin * dt;

			const PxTransform newPose = PxTransform(newP, PxExp(ang*dt) * preTrans.q);

			//PxVec3 lin, ang;
			/*calculateNewVelocity(newPose, data.mPreTransform[0],
				1.f, lin, ang);		*/	

			data.mAccumulatedPoses[0] = newPose;

			PxQuat dq = newPose.q * data.mPreTransform[0].q.getConjugate();

			if (dq.w < 0.f)
				dq = -dq;

			data.mDeltaQ[0] = dq;

			Cm::SpatialVectorF delta = motionVelocity * dt;

			deltaMotion[0] += delta;
			posMotionVelocities[0] += delta;
		}

		for (PxU32 linkID = 1; linkID < linkCount; linkID++)
		{
			ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
			
			const PxTransform newPose = articulation->propagateTransform(linkID, data.getLinks(), jointDatum, data.getMotionVelocities(),
				dt, data.mAccumulatedPoses[data.getLink(linkID).parent], data.mAccumulatedPoses[linkID], 
				jointNewVelocities, jointPosition, &data.getMotionMatrix(jointDatum.jointOffset), &data.getWorldMotionMatrix(jointDatum.jointOffset));

			//data.mDeltaQ[linkID] = data.mPreTransform[linkID].q.getConjugate() * newPose.q;
			PxQuat dq = newPose.q * data.mPreTransform[linkID].q.getConjugate();

			if(dq.w < 0.f)
				dq = -dq;

			data.mDeltaQ[linkID] = dq;

			

			/*PxVec3 lin, ang;
			calculateNewVelocity(newPose, data.mPreTransform[linkID],
				1.f, lin, ang);*/

			PxVec3 lin = (newPose.p - data.mPreTransform[linkID].p);

			Cm::SpatialVectorF delta = motionVelocities[linkID] * dt;
			
			//deltaMotion[linkID].top = ang;// motionVeloties[linkID].top * dt;
			deltaMotion[linkID].top += delta.top;
			deltaMotion[linkID].bottom = lin;// motionVeloties[linkID].top * dt;
			posMotionVelocities[linkID] += delta;


			//Record the new current pose
			data.mAccumulatedPoses[linkID] = newPose;
		}
	}

	void FeatherstoneArticulation::deltaMotionToMotionVelocity(const ArticulationSolverDesc& desc, PxReal invDt)
	{
		FeatherstoneArticulation* articulation = static_cast<FeatherstoneArticulation*>(desc.articulation);
		ArticulationData& data = articulation->mArticulationData;
		const PxU32 linkCount = data.getLinkCount();
		const Cm::SpatialVectorF* deltaMotion = data.getDeltaMotionVector();

		for (PxU32 linkID = 0; linkID<linkCount; linkID++)
		{
			Cm::SpatialVectorF& v = data.getMotionVelocity(linkID);

			Cm::SpatialVectorF delta = deltaMotion[linkID] * invDt;

			v = delta;

			desc.motionVelocity[linkID] = reinterpret_cast<Cm::SpatialVectorV&>(delta);
		}
	}

	//This is used in the solveExt1D, solveExtContact
	Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocity(PxU32 linkID)
	{
		//Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
	
		ArticulationLink* links = mArticulationData.getLinks();

		Cm::SpatialVectorF deltaV(PxVec3(0.f), PxVec3(0.f));

		if (!fixBase)
		{
			//deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
			//DeferredZ now in world space!
			deltaV = mArticulationData.mBaseInvSpatialArticulatedInertiaW * -mArticulationData.getRootDeferredZ();
		}


		const PxU32 startIndex = links[linkID].mPathToRootStartIndex;
		const PxU32 elementCount = links[linkID].mPathToRootCount;

		const PxU32* pathToRootElements = &mArticulationData.mPathToRootElements[startIndex];

		for (PxU32 i = 0; i < elementCount; ++i)
		{
			const PxU32 index = pathToRootElements[i];
			PX_ASSERT(links[index].parent < index);

			const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(index).dof;

			deltaV = propagateAccelerationW(mArticulationData.getRw(index), mArticulationData.mInvStIs[index],
				&mArticulationData.mWorldMotionMatrix[jointOffset], deltaV, dofCount, &mArticulationData.mIsW[jointOffset],
				&mArticulationData.mDeferredQstZ[jointOffset]);
		}

		Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV;

		return Cm::SpatialVector(vel.bottom, vel.top);
	}

	void FeatherstoneArticulation::pxcFsGetVelocities(PxU32 linkID, PxU32 linkID1, Cm::SpatialVectorV& v0, Cm::SpatialVectorV& v1)
	{
		{
			const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

			ArticulationLink* links = mArticulationData.getLinks();

			Cm::SpatialVectorF deltaV(PxVec3(0.f), PxVec3(0.f));

			if (!fixBase)
			{
				//deltaV = mArticulationData.mBaseInvSpatialArticulatedInertia * (-deferredZ[0]);
				deltaV = mArticulationData.mBaseInvSpatialArticulatedInertiaW * (-mArticulationData.mRootDeferredZ);
			}


			const PxU32* pathToRootElements = mArticulationData.mPathToRootElements;

			Dy::ArticulationLink& link0 = links[linkID];
			Dy::ArticulationLink& link1 = links[linkID1];

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
				const PxU32 dofCount = mArticulationData.getJointData(index).dof;

				deltaV = propagateAccelerationW(mArticulationData.getRw(index), mArticulationData.mInvStIs[index],
					&mArticulationData.mWorldMotionMatrix[jointOffset], deltaV, dofCount, &mArticulationData.mIsW[jointOffset], &mArticulationData.mDeferredQstZ[jointOffset]);
			}

			Cm::SpatialVectorF deltaV1 = deltaV;

			for (PxU32 idx = offset; idx < numElems0; ++idx)
			{
				const PxU32 index = pathToRoot0[idx];
				PX_ASSERT(links[index].parent < index);

				const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(index).dof;

				deltaV = propagateAccelerationW(mArticulationData.getRw(index), mArticulationData.mInvStIs[index],
					&mArticulationData.mWorldMotionMatrix[jointOffset], deltaV, dofCount, &mArticulationData.mIsW[jointOffset], &mArticulationData.mDeferredQstZ[jointOffset]);
				
			}

			for (PxU32 idx = offset; idx < numElems1; ++idx)
			{
				const PxU32 index = pathToRoot1[idx];
				PX_ASSERT(links[index].parent < index);

				const PxU32 jointOffset = mArticulationData.getJointData(index).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(index).dof;

				deltaV1 = propagateAccelerationW(mArticulationData.getRw(index), mArticulationData.mInvStIs[index],
					&mArticulationData.mWorldMotionMatrix[jointOffset], deltaV1, dofCount, &mArticulationData.mIsW[jointOffset], &mArticulationData.mDeferredQstZ[jointOffset]);

			}



			Cm::SpatialVectorF vel = mArticulationData.getMotionVelocity(linkID) + deltaV;

			v0 = Cm::SpatialVector(vel.bottom, vel.top);

			Cm::SpatialVectorF vel1 = mArticulationData.getMotionVelocity(linkID1) + deltaV1;

			v1 = Cm::SpatialVector(vel1.bottom, vel1.top);
		}
	}

	/*Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocity(PxU32 linkID)
	{

	Cm::SpatialVectorF& vel = mArticulationData.getMotionVelocity(linkID);

	return Cm::SpatialVector(vel.bottom, vel.top);
	}*/

	Cm::SpatialVectorV FeatherstoneArticulation::pxcFsGetVelocityTGS(PxU32 linkID)
	{
		return getLinkVelocity(linkID);
	}

	//This is used in the solveExt1D, solveExtContact
	void FeatherstoneArticulation::pxcFsApplyImpulse(PxU32 linkID, 
		aos::Vec3V linear, aos::Vec3V angular,
		Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*deltaV*/)
	{
		const ArticulationSolverDesc* desc = &mSolverDesc;

		ArticulationLink* links = static_cast<ArticulationLink*>(desc->links);

		//initialize all zero acceration impulse to be zero
		ArticulationData& data = mArticulationData;

		data.mJointDirty = true;

		//impulse is in world space
		Cm::SpatialVector impulse;
		V4StoreA(Vec4V_From_Vec3V(angular), &impulse.angular.x);
		V4StoreA(Vec4V_From_Vec3V(linear), &impulse.linear.x);
		Cm::SpatialVectorF Z0(-impulse.linear, -impulse.angular);



		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(i).dof;
			
			Z0 = propagateImpulseW(&data.mIsInvDW[jointOffset], mArticulationData.getRw(i), &data.mWorldMotionMatrix[jointOffset], Z0, dofCount, &mArticulationData.mDeferredQstZ[jointOffset]);
		}

		data.mRootDeferredZ += Z0;
	}

	void FeatherstoneArticulation::pxcFsApplyImpulses(Cm::SpatialVectorF* Z)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		//initialize all zero acceration impulse to be zero
		ArticulationData& data = mArticulationData;

		const PxU32 linkCount = mArticulationData.getLinkCount();

		const PxU32 startIndex = PxU32(linkCount - 1);

		data.mJointDirty = true;

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& tLink = links[linkID];
			const PxU32 jointOffset = mArticulationData.getJointData(linkID).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(linkID).dof;

			Cm::SpatialVectorF ZA = Z[linkID];

			Z[tLink.parent] += propagateImpulseW(&data.mIsInvDW[jointOffset], mArticulationData.getRw(linkID), &data.mWorldMotionMatrix[jointOffset], ZA,
				dofCount, &mArticulationData.mDeferredQstZ[jointOffset]);

		}
		data.mRootDeferredZ += Z[0];
	}

	void FeatherstoneArticulation::pxcFsApplyImpulses(PxU32 linkID, const aos::Vec3V& linear,
		const aos::Vec3V& angular, PxU32 linkID2, const aos::Vec3V& linear2,
		const aos::Vec3V& angular2, Cm::SpatialVectorF* /*Z*/, Cm::SpatialVectorF* /*deltaV*/)
	{
		if (0)
		{
			pxcFsApplyImpulse(linkID, linear, angular, NULL, NULL);
			pxcFsApplyImpulse(linkID2, linear2, angular2, NULL, NULL);
		}
		else
		{
			const ArticulationSolverDesc* desc = &mSolverDesc;
			ArticulationData& data = mArticulationData;
			data.mJointDirty = true;
			ArticulationLink* links = static_cast<ArticulationLink*>(desc->links);

			//impulse is in world space
			Cm::SpatialVector impulse0;
			V3StoreU(angular, impulse0.angular);
			V3StoreU(linear, impulse0.linear);

			Cm::SpatialVector impulse1;
			V3StoreU(angular2, impulse1.angular);
			V3StoreU(linear2, impulse1.linear);

			Cm::SpatialVectorF Z1(-impulse0.linear, -impulse0.angular);
			Cm::SpatialVectorF Z2(-impulse1.linear, -impulse1.angular);

			ArticulationLink& link0 = links[linkID];
			ArticulationLink& link1 = links[linkID2];
			
			const PxU32* pathToRoot0 = &mArticulationData.mPathToRootElements[link0.mPathToRootStartIndex];
			const PxU32* pathToRoot1 = &mArticulationData.mPathToRootElements[link1.mPathToRootStartIndex];

			const PxU32 numElems0 = link0.mPathToRootCount;
			const PxU32 numElems1 = link1.mPathToRootCount;

			//find the common link, work from one to that common, then the other to that common, then go from there upwards...
			PxU32 offset = 0;
			PxU32 commonLink = 0;
			while (pathToRoot0[offset] == pathToRoot1[offset])
			{
				commonLink = pathToRoot0[offset++];
				PX_ASSERT(links[commonLink].parent < commonLink);
				if (offset >= numElems0 || offset >= numElems1)
					break;
			}

			//The common link will either be linkID2, or its ancestors.
			//The common link cannot be an index before either linkID2 or linkID
			for (PxU32 i = linkID2; i != commonLink; i = links[i].parent)
			{
				const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(i).dof;
				Z2 = propagateImpulseW(&data.mIsInvDW[jointOffset], mArticulationData.getRw(i), &data.mWorldMotionMatrix[jointOffset], Z2, dofCount, &data.mDeferredQstZ[jointOffset]);
			}

			for (PxU32 i = linkID; i != commonLink; i = links[i].parent)
			{
				const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(i).dof;
				Z1 = propagateImpulseW(&data.mIsInvDW[jointOffset], mArticulationData.getRw(i), &data.mWorldMotionMatrix[jointOffset], Z1, dofCount,
					&data.mDeferredQstZ[jointOffset]);
			}

			Cm::SpatialVectorF ZCommon = Z1 + Z2;

			for (PxU32 i = commonLink; i; i = links[i].parent)
			{
				const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
				const PxU32 dofCount = mArticulationData.getJointData(i).dof;
				ZCommon = propagateImpulseW(&data.mIsInvDW[jointOffset], mArticulationData.getRw(i), &data.mWorldMotionMatrix[jointOffset], ZCommon, dofCount,
					&data.mDeferredQstZ[jointOffset]);
			}

			data.mRootDeferredZ += ZCommon;
		}
	}

	//Z is the link space(drag force)
	void FeatherstoneArticulation::applyImpulses(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
	{
		ArticulationLink* links = mArticulationData.getLinks();

		//initialize all zero acceration impulse to be zero
		ArticulationData& data = mArticulationData;
		

		const PxU32 linkCount = mArticulationData.getLinkCount();

		const PxU32 startIndex = PxU32(linkCount - 1);

		for (PxU32 linkID = startIndex; linkID > 0; --linkID)
		{
			ArticulationLink& tLink = links[linkID];
			const PxU32 jointOffset = mArticulationData.getJointData(linkID).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(linkID).dof;
			
			Z[tLink.parent] += propagateImpulseW(&data.mIsInvDW[jointOffset], mArticulationData.getRw(linkID), &data.mWorldMotionMatrix[jointOffset], Z[linkID], dofCount);
		}

		getDeltaV(Z, deltaV);
	}

	void FeatherstoneArticulation::getDeltaV(Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV)
	{
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		Cm::SpatialVectorF* motionVelocities = mArticulationData.getMotionVelocities();
		ArticulationLink* links = mArticulationData.getLinks();
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		//This will be zero at the begining of the frame
		PxReal* jointDeltaVelocities = mArticulationData.getJointNewVelocities();

		if (fixBase)
		{
			deltaV[0] = Cm::SpatialVectorF(PxVec3(0.f), PxVec3(0.f));
		}
		else
		{

			deltaV[0] = mArticulationData.mBaseInvSpatialArticulatedInertiaW * (-Z[0]);
			motionVelocities[0] += deltaV[0];

			PX_ASSERT(motionVelocities[0].isFinite());
		}

		const PxU32 linkCount = mArticulationData.getLinkCount();
		
		for (PxU32 i = 1; i < linkCount; i++)
		{
			ArticulationLink& tLink = links[i];
			ArticulationJointCoreData& tJointDatum = jointData[i];
			const PxU32 jointOffset = mArticulationData.getJointData(i).jointOffset;
			const PxU32 dofCount = mArticulationData.getJointData(i).dof;
			Cm::SpatialVectorF dV = propagateVelocityW(mArticulationData.getRw(i), mArticulationData.mWorldSpatialArticulatedInertia[i],
				mArticulationData.mInvStIs[i], &mArticulationData.mWorldMotionMatrix[jointOffset], Z[i], &jointDeltaVelocities[tJointDatum.jointOffset], deltaV[tLink.parent], dofCount);

			deltaV[i] = dV;

			motionVelocities[i] += dV;

			PX_ASSERT(motionVelocities[i].isFinite());
		}
	}

	//This version uses in updateBodies
	PxQuat computeSphericalJointPositions(const PxQuat& relativeQuat,
		const PxQuat& newRot, const PxQuat& pBody2WorldRot,
		PxReal* jPositions, const Cm::UnAlignedSpatialVector* motionMatrix,
		const PxU32 dofs);

	PxQuat computeSphericalJointPositions(const PxQuat& relativeQuat,
		const PxQuat& newRot, const PxQuat& pBody2WorldRot);

	PxTransform FeatherstoneArticulation::propagateTransform(const PxU32 linkID, ArticulationLink* links,
		ArticulationJointCoreData& jointDatum, Cm::SpatialVectorF* motionVelocities, const PxReal dt, const PxTransform& pBody2World, 
		const PxTransform& currentTransform, PxReal* jointVelocities, PxReal* jointPositions,
		const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::UnAlignedSpatialVector* worldMotionMatrix)
	{
		ArticulationLink& link = links[linkID];

		const PxQuat relativeQuat = mArticulationData.mRelativeQuat[linkID];
		
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
			PxReal tJointPosition = jPosition[0] + (jVelocity[0]) * dt;

			const PxU32 dofId = link.inboundJoint->dofIds[0];

			if (link.inboundJoint->motion[dofId] == PxArticulationMotion::eLIMITED)
			{
				if (tJointPosition < (link.inboundJoint->limits[dofId].low))
					tJointPosition = link.inboundJoint->limits[dofId].low;
				if (tJointPosition >(link.inboundJoint->limits[dofId].high))
					tJointPosition = link.inboundJoint->limits[dofId].high;
			}

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
			PxReal tJointPosition = jPosition[0] + (jVelocity[0]) * dt;

			/*PxU8 dofId = link.inboundJoint->dofIds[0];
			if (link.inboundJoint->motion[dofId] == PxArticulationMotion::eLIMITED)
			{
				if (tJointPosition < (link.inboundJoint->limits[dofId].low))
					tJointPosition = link.inboundJoint->limits[dofId].low;
				if (tJointPosition >(link.inboundJoint->limits[dofId].high))
					tJointPosition = link.inboundJoint->limits[dofId].high;
			}*/

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
			if (1)//jointDatum.dof < 3)
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

				newParentToChild = computeSphericalJointPositions(mArticulationData.mRelativeQuat[linkID], newWorldQ,
					pBody2World.q);

				PxQuat jointRotation = newParentToChild * relativeQuat.getConjugate();

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
				for (PxU32 i = 0; i < jointDatum.dof; ++i)
				{
					PxVec3 sa = mArticulationData.getMotionMatrix(jointDatum.jointOffset + i).top;
					PxReal ang = -sa.dot(axis);
					jPosition[i] = ang;
				}
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;
				r = e + d;

				PX_ASSERT(r.isFinite());
			}
			else
			{
				PxVec3 worldAngVel = motionVelocities[linkID].top;

				newWorldQ = PxExp(worldAngVel*dt) * currentTransform.q;

				newParentToChild = computeSphericalJointPositions(mArticulationData.mRelativeQuat[linkID], newWorldQ,
					pBody2World.q, jPosition, motionMatrix, jointDatum.dof);

				/*PxQuat newQ = (pBody2World.q * newParentToChild.getConjugate()).getNormalized();

				const PxQuat cB2w = newQ * joint->childPose.q;

				const PxMat33 cB2w_m(cB2w);

				const PxVec3* axis = &cB2w_m.column0;

				PxU32 dofIdx = 0;*/
				PxVec3 relAngVel = worldAngVel - motionVelocities[link.parent].top;

				for (PxU32 i = 0; i < jointDatum.dof; ++i)
				{
					jVelocity[i] = worldMotionMatrix[i].top.dot(relAngVel);
				}
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

	const PxTransform& FeatherstoneArticulation::getCurrentTransform(PxU32 linkID) const
	{
		return mArticulationData.mAccumulatedPoses[linkID];
	}

	const PxQuat& FeatherstoneArticulation::getDeltaQ(PxU32 linkID) const
	{
		return mArticulationData.mDeltaQ[linkID];
	}

	////Z is the spatial acceleration impulse of links[linkID]
	//Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocity(const Dy::SpatialTransform& c2p, const Dy::SpatialMatrix& spatialInertia, 
	//	const InvStIs& invStIs, const SpatialSubspaceMatrix& motionMatrix, const Cm::SpatialVectorF& Z, PxReal* jointVelocity, const Cm::SpatialVectorF& hDeltaV)
	//{
	//	const PxU32 dofCount = motionMatrix.getNumColumns();
	//	Cm::SpatialVectorF pDeltaV = c2p.transposeTransform(hDeltaV); //parent velocity change

	//	Cm::SpatialVectorF temp = spatialInertia * pDeltaV + Z;

	//	PxReal tJointDelta[6];
	//	for (PxU32 ind = 0; ind < dofCount; ++ind)
	//	{
	//		const Cm::SpatialVectorF& sa = motionMatrix[ind];
	//		tJointDelta[ind] = -sa.innerProduct(temp);
	//	}

	//	Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

	//	for (PxU32 ind = 0; ind < dofCount; ++ind)
	//	{
	//		PxReal jDelta = 0.f;
	//		for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
	//		{
	//			jDelta += invStIs.invStIs[ind2][ind] * tJointDelta[ind2];
	//		}

	//		jointVelocity[ind] += jDelta;

	//		const Cm::SpatialVectorF& sa = motionMatrix[ind];
	//		jointSpatialDeltaV += sa * jDelta;
	//	}

	//	return pDeltaV + jointSpatialDeltaV;
	//}

	////This method calculate the velocity change due to collision/constraint impulse
	//Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocityTestImpulse(const Dy::SpatialTransform& c2p, const Dy::SpatialMatrix& spatialInertia, 
	//	const InvStIs& invStIs, const SpatialSubspaceMatrix& motionMatrix, const Cm::SpatialVectorF& Z,
	//	const Cm::SpatialVectorF& hDeltaV)
	//{
	//	const PxU32 dofCount = motionMatrix.getNumColumns();
	//	Cm::SpatialVectorF pDeltaV = c2p.transposeTransform(hDeltaV); //parent velocity change

	//	Cm::SpatialVectorF temp = spatialInertia * pDeltaV + Z;

	//	PxReal tJointDelta[6];
	//	for (PxU32 ind = 0; ind < dofCount; ++ind)
	//	{
	//		const Cm::SpatialVectorF& sa = motionMatrix[ind];
	//		tJointDelta[ind] = -sa.innerProduct(temp);
	//	}

	//	Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

	//	for (PxU32 ind = 0; ind < dofCount; ++ind)
	//	{
	//		PxReal jDelta = 0.f;
	//		for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
	//		{
	//			jDelta += invStIs.invStIs[ind2][ind] * tJointDelta[ind2];
	//		}

	//		const Cm::SpatialVectorF& sa = motionMatrix[ind];
	//		jointSpatialDeltaV += sa * jDelta;
	//	}

	//	return pDeltaV + jointSpatialDeltaV;
	//}

	PX_CUDA_CALLABLE PX_FORCE_INLINE Cm::SpatialVectorF translateImpulse(const Cm::SpatialVectorF& s, const PxVec3& offset)
	{
		return Cm::SpatialVectorF(s.top, offset.cross(s.top) + s.bottom);
	}

	//This method calculate the velocity change due to collision/constraint impulse, record joint velocity and acceleration
	Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocityW(const PxVec3& c2p, const Dy::SpatialMatrix& spatialInertia,
		const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z,
		PxReal* jointVelocity, const Cm::SpatialVectorF& hDeltaV, const PxU32 dofCount)
	{
		Cm::SpatialVectorF pDeltaV = translateImpulse(hDeltaV, -c2p); //parent velocity change

		Cm::SpatialVectorF temp = spatialInertia * pDeltaV + Z;

		PxReal tJointDelta[6];
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];
			tJointDelta[ind] = -sa.innerProduct(temp);
		}

		Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			PxReal jDelta = 0.f;
			for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			{
				jDelta += invStIs.invStIs[ind2][ind] * tJointDelta[ind2];
			}

			jointVelocity[ind] += jDelta;

			const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];
			jointSpatialDeltaV.top += sa.top * jDelta;
			jointSpatialDeltaV.bottom += sa.bottom * jDelta;
		}

		return pDeltaV + jointSpatialDeltaV;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::propagateAccelerationW(const PxVec3& c2p,
		const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix,
		PxReal* jointVelocity, const Cm::SpatialVectorF& pAcceleration, const PxU32 dofCount, const Cm::SpatialVectorF* IsW, PxReal* qstZIc)
	{
		Cm::SpatialVectorF motionAcceleration = translateImpulse(pAcceleration, -c2p); //parent velocity change

		PxReal tJAccel[3];
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			//stI * pAcceleration
			const PxReal temp = IsW[ind].innerProduct(motionAcceleration);

			tJAccel[ind] = (qstZIc[ind] - temp);
		}

		//calculate jointAcceleration
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			PxReal jVel = 0.f;
			//for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			{
				jVel += invStIs.invStIs[ind2][ind] * tJAccel[ind2];
			}
			//PX_ASSERT(PxAbs(jointAcceleration[ind]) < 5000);

			motionAcceleration.top += motionMatrix[ind].top * jVel;
			motionAcceleration.bottom += motionMatrix[ind].bottom * jVel;

			jointVelocity[ind] += jVel;
		}

		return motionAcceleration;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::propagateAccelerationW(const PxVec3& c2p,
		const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix,
		const Cm::SpatialVectorF& pAcceleration, const PxU32 dofCount, const Cm::SpatialVectorF* IsW, PxReal* qstZ)
	{
		Cm::SpatialVectorF motionAcceleration = translateImpulse(pAcceleration, -c2p); //parent velocity change

		PxReal tJAccel[3];
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			//stI * pAcceleration
			const PxReal temp = IsW[ind].innerProduct(motionAcceleration);

			tJAccel[ind] = (qstZ[ind] - temp);
		}

		//calculate jointAcceleration
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			PxReal jVel = 0.f;
			for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			{
				jVel += invStIs.invStIs[ind2][ind] * tJAccel[ind2];
			}
			//PX_ASSERT(PxAbs(jointAcceleration[ind]) < 5000);

			motionAcceleration.top += motionMatrix[ind].top * jVel;
			motionAcceleration.bottom += motionMatrix[ind].bottom * jVel;
		}

		return motionAcceleration;
	}


	Cm::SpatialVectorF FeatherstoneArticulation::propagateAccelerationW(const PxVec3& c2p,
		const InvStIs& invStIs, const Cm::UnAlignedSpatialVector* motionMatrix,
		PxReal* jointVelocity, const Cm::SpatialVectorF& pAcceleration, Cm::SpatialVectorF& Z, const PxU32 dofCount, const Cm::SpatialVectorF* IsW)
	{
		Cm::SpatialVectorF motionAcceleration = translateImpulse(pAcceleration, -c2p); //parent velocity change

		PxReal tJAccel[3];
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			//stI * pAcceleration
			const PxReal temp = IsW[ind].innerProduct(motionAcceleration);
			PxReal qstZ = -motionMatrix[ind].innerProduct(Z);
			tJAccel[ind] = (qstZ - temp);
		}

		//calculate jointAcceleration
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			PxReal jVel = 0.f;
			for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			{
				jVel += invStIs.invStIs[ind2][ind] * tJAccel[ind2];
			}
			//PX_ASSERT(PxAbs(jointAcceleration[ind]) < 5000);

			motionAcceleration.top += motionMatrix[ind].top * jVel;
			motionAcceleration.bottom += motionMatrix[ind].bottom * jVel;

			jointVelocity[ind] += jVel;
		}

		return motionAcceleration;
	}


	//This method calculate the velocity change due to collision/constraint impulse
	Cm::SpatialVectorF FeatherstoneArticulation::propagateVelocityTestImpulseW(const PxVec3& c2p, const Dy::SpatialMatrix& spatialInertia, const InvStIs& invStIs,
		const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z, const Cm::SpatialVectorF& hDeltaV, const PxU32 dofCount)
	{
		Cm::SpatialVectorF pDeltaV = translateImpulse(hDeltaV, -c2p); //parent velocity change

		//Convert parent velocity change into an impulse
		Cm::SpatialVectorF temp = spatialInertia * pDeltaV + Z;

		PxReal tJointDelta[3];
		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];
			tJointDelta[ind] = -sa.innerProduct(temp);
		}

		Cm::SpatialVectorF jointSpatialDeltaV(PxVec3(0.f), PxVec3(0.f));

		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			PxReal jDelta = 0.f;
			for (PxU32 ind2 = 0; ind2 < dofCount; ++ind2)
			{
				jDelta += invStIs.invStIs[ind2][ind] * tJointDelta[ind2];
			}

			const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];
			jointSpatialDeltaV.top += sa.top * jDelta;
			jointSpatialDeltaV.bottom += sa.bottom * jDelta;
		}

		return pDeltaV + jointSpatialDeltaV;
	}


	//Cm::SpatialVectorF FeatherstoneArticulation::propagateImpulse(const IsInvD& isInvD, 
	//	const SpatialTransform& childToParent, const SpatialSubspaceMatrix& motionMatrix, const Cm::SpatialVectorF& Z)
	//{
	//	const PxU32 dofCount = motionMatrix.getNumColumns();
	//	Cm::SpatialVectorF temp(PxVec3(0.f), PxVec3(0.f));

	//	for (PxU32 ind = 0; ind < dofCount; ++ind)
	//	{
	//		const Cm::SpatialVectorF& sa = motionMatrix[ind];
	//		const PxReal stZ = sa.innerProduct(Z);
	//		temp += isInvD.isInvD[ind] * stZ;
	//	}

	//	//parent space's spatial zero acceleration impulse
	//	return  childToParent * (Z - temp);
	//}

	Cm::SpatialVectorF FeatherstoneArticulation::propagateImpulseW(const Cm::SpatialVectorF* isInvD, const PxVec3& childToParent,
		const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z, const PxU32 dofCount)
	{
		Cm::SpatialVectorF temp(PxVec3(0.f), PxVec3(0.f));

		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];
			const PxReal stZ = sa.innerProduct(Z);
			temp += isInvD[ind] * stZ;
		}

		//parent space's spatial zero acceleration impulse

		return FeatherstoneArticulation::translateSpatialVector(childToParent, (Z-temp));
		/*Cm::SpatialVectorF temp2 = (Z - temp);
		temp2.bottom += childToParent.cross(temp2.top);
		return  temp2;*/
	}

	Cm::SpatialVectorF FeatherstoneArticulation::propagateImpulseW(const Cm::SpatialVectorF* isInvD, const PxVec3& childToParent,
		const Cm::UnAlignedSpatialVector* motionMatrix, const Cm::SpatialVectorF& Z, const PxU32 dofCount, PxReal* qsztZ)
	{
		Cm::SpatialVectorF temp = Z;

		for (PxU32 ind = 0; ind < dofCount; ++ind)
		{
			const Cm::UnAlignedSpatialVector& sa = motionMatrix[ind];
			const PxReal stZ = -sa.innerProduct(Z);
			PX_ASSERT(PxIsFinite(stZ));
			qsztZ[ind] += stZ;
			PX_ASSERT(PxIsFinite(qsztZ[ind]));
			temp += isInvD[ind] * stZ;
		}

		//parent space's spatial zero acceleration impulse
		return FeatherstoneArticulation::translateSpatialVector(childToParent, temp);
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

		ArticulationLink* links = data.getLinks();
		const ArticulationLink& link = links[linkID];
		const PxU32* pathToRoot = &data.mPathToRootElements[link.mPathToRootStartIndex];
		const PxU32 numElems = link.mPathToRootCount;

		for (PxU32 i = 0; i < numElems; ++i)
		{
			const PxU32 index = pathToRoot[i];
			PX_ASSERT(links[index].parent < index);
			ArticulationJointCoreData& tJointDatum = data.getJointData(index);
			PxReal* jVelocity = &jointVelocities[tJointDatum.jointOffset];
			deltaV = FeatherstoneArticulation::propagateVelocityW(data.getRw(index), data.mWorldSpatialArticulatedInertia[index],
				data.mInvStIs[index], &data.mWorldMotionMatrix[tJointDatum.jointOffset], Z[index], jVelocity, deltaV, tJointDatum.dof);
		}

		return deltaV;
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getDeltaV(const bool fixBase, const PxU32 linkID,
		const ArticulationData& data, Cm::SpatialVectorF* Z)
	{
		Cm::SpatialVectorF deltaV = Cm::SpatialVectorF::Zero();
		if (!fixBase)
		{
			//velocity change
			//SpatialMatrix inverseArticulatedInertia = hLinkDatum.spatialArticulatedInertia.getInverse();
			const SpatialMatrix& inverseArticulatedInertia = data.mBaseInvSpatialArticulatedInertiaW;
			deltaV = inverseArticulatedInertia * (-Z[0]);
		}

		ArticulationLink* links = data.getLinks();
		ArticulationLink& link = links[linkID];
		const PxU32* pathToRoot = &data.mPathToRootElements[link.mPathToRootStartIndex];
		const PxU32 numElems = link.mPathToRootCount;
		
		for (PxU32 i = 0; i < numElems; ++i)
		{
			const PxU32 index = pathToRoot[i];
			PX_ASSERT(links[index].parent < index);

			const PxU32 jointOffset = data.getJointData(index).jointOffset;
			const PxU32 dofCount = data.getJointData(index).dof;
			
			PxReal jDeltaV[3];
			deltaV = propagateAccelerationW(data.getRw(index), data.mInvStIs[index],
				&data.mWorldMotionMatrix[jointOffset], jDeltaV, deltaV, Z[index], dofCount, &data.mIsW[jointOffset]);
		}

		return deltaV;
	}

	void  FeatherstoneArticulation::getZ(const PxU32 linkID,
		const ArticulationData& data, Cm::SpatialVectorF* Z, 
		const Cm::SpatialVectorF& impulse)
	{
		ArticulationLink* links = data.getLinks();

		//impulse need to be in linkID space!!!
		Z[linkID] = -impulse;

		for (PxU32 i = linkID; i; i = links[i].parent)
		{
			ArticulationLink& tLink = links[i];
			const PxU32 jointOffset = data.getJointData(i).jointOffset;
			const PxU32 dofCount = data.getJointData(i).dof;
			Z[tLink.parent] = FeatherstoneArticulation::propagateImpulseW(&data.mIsInvDW[jointOffset], data.getRw(i),
				&data.mWorldMotionMatrix[jointOffset], Z[i], dofCount);
		}
	}

	Cm::SpatialVectorF FeatherstoneArticulation::getImpulseResponseW(
		const PxU32 linkID,
		const ArticulationData& data,
		const Cm::SpatialVectorF& impulse)
	{
		return data.getImpulseResponseMatrixWorld()[linkID].getResponse(impulse);
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

	void FeatherstoneArticulation::saveVelocity(const ArticulationSolverDesc& d, Cm::SpatialVectorF* deltaV)
	{
		FeatherstoneArticulation* arti = static_cast<FeatherstoneArticulation*>(d.articulation);
		ArticulationData& data = arti->mArticulationData;

		//update all links' motion velocity, joint delta velocity if there are contacts/constraints
		if (data.mJointDirty)
		{
			bool doForces = (data.getArticulationFlags() & PxArticulationFlag::eCOMPUTE_JOINT_FORCES) || data.getSensorCount();
			PxcFsFlushVelocity(*arti, deltaV, doForces);
		}

		const PxU32 linkCount = data.getLinkCount();
		//copy motion velocites
		Cm::SpatialVectorF* vels = data.getMotionVelocities();
		Cm::SpatialVectorF* posVels = data.getPosIterMotionVelocities();
		PxMemCopy(posVels, vels, sizeof(Cm::SpatialVectorF) * linkCount);
	
		//copy joint velocities
		const PxU32 dofs = data.getDofs();

		PxReal* jPosVels = data.getPosIterJointVelocities();

		const PxReal* jNewVels = data.getJointNewVelocities();

		PxMemCopy(jPosVels, jNewVels, sizeof(PxReal) * dofs);

		static_cast<FeatherstoneArticulation*>(d.articulation)->concludeInternalConstraints(false);

	/*	for (PxU32 i = 0; i < dofs; ++i)
		{
			PX_ASSERT(PxAbs(jPosDeltaVels[i]) < 30.f);
		}*/
	}

	void FeatherstoneArticulation::saveVelocityTGS(const ArticulationSolverDesc& d, PxReal invDtF32)
	{
		FeatherstoneArticulation* arti = static_cast<FeatherstoneArticulation*>(d.articulation);
		ArticulationData& data = arti->mArticulationData;

		//KS - we should not need to flush velocity because we will have already stepped the articulation with TGS

		const PxU32 linkCount = data.getLinkCount();
		Cm::SpatialVectorF* posVels = data.getPosIterMotionVelocities();
		for (PxU32 i = 0; i < linkCount; ++i)
		{
			posVels[i] = posVels[i] * invDtF32;
		}
	}

	void FeatherstoneArticulation::getImpulseSelfResponse(
		PxU32 linkID0,
		PxU32 linkID1,
		Cm::SpatialVectorF* Z,
		const Cm::SpatialVector& impulse0,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV0,
		Cm::SpatialVector& deltaV1) const
	{
		FeatherstoneArticulation::getImpulseSelfResponse(mArticulationData.getLinks(),
			Z, const_cast<Dy::ArticulationData&>(mArticulationData), linkID0, reinterpret_cast<const Cm::SpatialVectorV&>(impulse0), 
			reinterpret_cast<Cm::SpatialVectorV&>(deltaV0), linkID1, reinterpret_cast<const Cm::SpatialVectorV&>(impulse1), 
			reinterpret_cast<Cm::SpatialVectorV&>(deltaV1));
	}

	void FeatherstoneArticulation::getImpulseResponseSlow(Dy::ArticulationLink* links,
		ArticulationData& data,
		PxU32 linkID0_,
		const Cm::SpatialVector& impulse0,
		Cm::SpatialVector& deltaV0,
		PxU32 linkID1_,
		const Cm::SpatialVector& impulse1,
		Cm::SpatialVector& deltaV1,
		Cm::SpatialVectorF* /*Z*/)
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

		for (i0 = 0; linkID0 != common; linkID0 = links[linkID0].parent)
		{
			const PxU32 jointOffset = data.getJointData(linkID0).jointOffset;
			const PxU32 dofCount = data.getJointData(linkID0).dof;
			Z0 = FeatherstoneArticulation::propagateImpulseW(&data.getWorldIsInvD(jointOffset), data.getRw(linkID0),
				&data.getWorldMotionMatrix(jointOffset), Z0, dofCount, &qstZ[jointOffset]);
			stack[i0++] = linkID0;
		}

		for (i1 = i0; linkID1 != common; linkID1 = links[linkID1].parent)
		{
			const PxU32 jointOffset = data.getJointData(linkID1).jointOffset;
			const PxU32 dofCount = data.getJointData(linkID1).dof;
			Z1 = FeatherstoneArticulation::propagateImpulseW(&data.getWorldIsInvD(jointOffset), data.getRw(linkID1),
				&data.getWorldMotionMatrix(jointOffset), Z1, dofCount, &qstZ[jointOffset]);
			stack[i1++] = linkID1;
		}

		Cm::SpatialVectorF ZZ = Z0 + Z1;
		
		Cm::SpatialVectorF v = data.getImpulseResponseMatrixWorld()[common].getResponse(-ZZ); 
	
		Cm::SpatialVectorF dv1 = v;
		for (PxU32 index = i1; (index--) > i0;)
		{
			//Dy::ArticulationLinkData& tLinkDatum = data.getLinkData(stack[index]);
			const PxU32 id = stack[index];
			const PxU32 jointOffset = data.getJointData(id).jointOffset;
			const PxU32 dofCount = data.getJointData(id).dof;
			dv1 = propagateAccelerationW(data.getRw(id), data.mInvStIs[id],
				&data.mWorldMotionMatrix[jointOffset], dv1, dofCount, &data.mIsW[jointOffset], &qstZ[jointOffset]);
		}

		Cm::SpatialVectorF dv0= v;
		for (PxU32 index = i0; (index--) > 0;)
		{
			const PxU32 id = stack[index];
			const PxU32 jointOffset = data.getJointData(id).jointOffset;
			const PxU32 dofCount = data.getJointData(id).dof;
			dv0 = propagateAccelerationW(data.getRw(id), data.mInvStIs[id],
				&data.mWorldMotionMatrix[jointOffset], dv0, dofCount, &data.mIsW[jointOffset], &qstZ[jointOffset]);
		}

		deltaV0.linear = dv0.bottom;
		deltaV0.angular = dv0.top;

		deltaV1.linear = dv1.bottom;
		deltaV1.angular = dv1.top;
	}

	void FeatherstoneArticulation::getImpulseSelfResponse(ArticulationLink* links,
		Cm::SpatialVectorF* Z,
		ArticulationData& data,
		PxU32 linkID0,
		const Cm::SpatialVectorV& impulse0,
		Cm::SpatialVectorV& deltaV0,
		PxU32 linkID1,
		const Cm::SpatialVectorV& impulse1,
		Cm::SpatialVectorV& deltaV1)
	{
		
		ArticulationLink& link = links[linkID1];

		if (link.parent == linkID0)
		{
			PX_ASSERT(linkID0 == link.parent);
			PX_ASSERT(linkID0 < linkID1);

			//impulse is in world space
			Cm::SpatialVectorF imp1;
			V4StoreA(Vec4V_From_Vec3V(impulse1.angular), &imp1.bottom.x);
			V4StoreA(Vec4V_From_Vec3V(impulse1.linear), &imp1.top.x);

			Cm::SpatialVectorF imp0;
			V4StoreA(Vec4V_From_Vec3V(impulse0.angular), &imp0.bottom.x);
			V4StoreA(Vec4V_From_Vec3V(impulse0.linear), &imp0.top.x);

			Cm::SpatialVectorF Z1W(-imp1.top, -imp1.bottom);
			
			const PxU32 jointOffset1 = data.getJointData(linkID1).jointOffset;
			const PxU32 dofCount1 = data.getJointData(linkID1).dof;

			PxReal qstZ[3] = { 0.f, 0.f, 0.f };
			Cm::SpatialVectorF Z0W = FeatherstoneArticulation::propagateImpulseW(&data.mIsInvDW[jointOffset1], data.getRw(linkID1),
				&data.mWorldMotionMatrix[jointOffset1], Z1W, dofCount1, qstZ);
			
			//in parent space
			const Cm::SpatialVectorF impulseDifW = imp0 - Z0W;
			
			//calculate velocity change start from the parent link to the root
			const Cm::SpatialVectorF delV0W = FeatherstoneArticulation::getImpulseResponseW(linkID0, data, impulseDifW);
			
			//calculate velocity change for child link
			/*const Cm::SpatialVectorF delV1W = FeatherstoneArticulation::propagateVelocityTestImpulseW(data.getLinkData(linkID1).rw,
				data.mWorldSpatialArticulatedInertia[linkID1], data.mInvStIs[linkID1], &data.mWorldMotionMatrix[jointOffset1], Z1W, delV0W, dofCount1);*/

			const Cm::SpatialVectorF delV1W = propagateAccelerationW(data.getRw(linkID1), data.mInvStIs[linkID1],
				&data.mWorldMotionMatrix[jointOffset1], delV0W, dofCount1, &data.mIsW[jointOffset1], qstZ);
			
			deltaV0.linear = Vec3V_From_Vec4V(V4LoadA(&delV0W.bottom.x));
			deltaV0.angular = Vec3V_From_Vec4V(V4LoadA(&delV0W.top.x));
			deltaV1.linear = Vec3V_From_Vec4V(V4LoadA(&delV1W.bottom.x));
			deltaV1.angular = Vec3V_From_Vec4V(V4LoadA(&delV1W.top.x));
			
		}
		else
		{
			getImpulseResponseSlow(links, data, linkID0, reinterpret_cast<const Cm::SpatialVector&>(impulse0), 
				reinterpret_cast<Cm::SpatialVector&>(deltaV0), linkID1, 
				reinterpret_cast<const Cm::SpatialVector&>(impulse1), reinterpret_cast<Cm::SpatialVector&>(deltaV1),
				Z);
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
		Dy::ConstraintWriteback* constraintWritebackPool, const PxReal biasCoefficient, const PxReal lengthScale)
	{
		BlockAllocator blockAllocator(blockManager, threadContext.mConstraintBlockStream, threadContext.mFrictionPatchStreamPair, threadContext.mConstraintSize);

		const PxTransform id(PxIdentity);

		PxSort<PxSolverConstraintDesc, ArticulationStaticConstraintSortPredicate>(mStatic1DConstraints.begin(), mStatic1DConstraints.size(), ArticulationStaticConstraintSortPredicate());
		PxSort<PxSolverConstraintDesc, ArticulationStaticConstraintSortPredicate>(mStaticContactConstraints.begin(), mStaticContactConstraints.size(), ArticulationStaticConstraintSortPredicate());

		for (PxU32 i = 0; i < mStatic1DConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStatic1DConstraints[i];

			PxU32 linkIndex = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? desc.linkIndexA : desc.linkIndexB;
			
			PX_ASSERT(desc.constraintLengthOver16 == DY_SC_TYPE_RB_1D);
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

			prepDesc.desc = static_cast<PxSolverConstraintDesc*>(&desc);
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

			SetupSolverConstraintStep(shaderPrepDesc, prepDesc, blockAllocator, stepDt, totalDt, invStepDt, invTotalDt, lengthScale, biasCoefficient);

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

			PX_ASSERT(desc.constraintLengthOver16 == DY_SC_TYPE_RB_CONTACT);
			
			PxTGSSolverContactDesc blockDesc;
			PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);
			PxcNpWorkUnit& unit = cm->getWorkUnit();
			PxsContactManagerOutput* cmOutput = &outputs.getContactManager(unit.mNpIndex);

			PxTGSSolverBodyVel& b0 = *desc.tgsBodyA;
			PxTGSSolverBodyVel& b1 = *desc.tgsBodyB;

			PxTGSSolverBodyData& data0 = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyADataIndex];
			PxTGSSolverBodyData& data1 = desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyBDataIndex];

			PxTGSSolverBodyTxInertia& txI0 = txInertia[desc.bodyADataIndex];
			PxTGSSolverBodyTxInertia& txI1 = txInertia[desc.bodyBDataIndex];

			blockDesc.bodyFrame0 = unit.rigidCore0->body2World;
			blockDesc.bodyFrame1 = unit.rigidCore1->body2World;
			blockDesc.shapeInteraction = cm->getShapeInteraction();
			blockDesc.contactForces = cmOutput->contactForces;
			blockDesc.desc = static_cast<PxSolverConstraintDesc*>(&desc);
			blockDesc.body0 = &b0;
			blockDesc.body1 = &b1;
			blockDesc.body0TxI = &txI0;
			blockDesc.body1TxI = &txI1;
			blockDesc.bodyData0 = &data0;
			blockDesc.bodyData1 = &data1;
			blockDesc.hasForceThresholds = !!(unit.flags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);
			blockDesc.disableStrongFriction = !!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION);
			blockDesc.bodyState0 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? PxSolverContactDesc::eARTICULATION : PxSolverContactDesc::eDYNAMIC_BODY;
			blockDesc.bodyState1 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? PxSolverContactDesc::eARTICULATION : (unit.flags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR) ? PxSolverContactDesc::eKINEMATIC_BODY :
				((unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1) ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eSTATIC_BODY);
			//blockDesc.flags = unit.flags;

			PxReal maxImpulse0 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? static_cast<const PxsBodyCore*>(unit.rigidCore0)->maxContactImpulse : data0.maxContactImpulse;
			PxReal maxImpulse1 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? static_cast<const PxsBodyCore*>(unit.rigidCore1)->maxContactImpulse : data1.maxContactImpulse;

			PxReal dominance0 = unit.dominance0 ? 1.f : 0.f;
			PxReal dominance1 = unit.dominance1 ? 1.f : 0.f;

			blockDesc.invMassScales.linear0 = blockDesc.invMassScales.angular0 = dominance0;
			blockDesc.invMassScales.linear1 = blockDesc.invMassScales.angular1 = dominance1;
			blockDesc.restDistance = unit.restDistance;
			blockDesc.frictionPtr = unit.frictionDataPtr;
			blockDesc.frictionCount = unit.frictionPatchCount;
			blockDesc.maxCCDSeparation = PX_MAX_F32;
			blockDesc.maxImpulse = PxMin(maxImpulse0, maxImpulse1);
			blockDesc.torsionalPatchRadius = unit.mTorsionalPatchRadius;
			blockDesc.minTorsionalPatchRadius = unit.mMinTorsionalPatchRadius;
			blockDesc.offsetSlop = unit.mOffsetSlop;

			createFinalizeSolverContactsStep(blockDesc, *cmOutput, threadContext,
				invStepDt, invTotalDt, totalDt, stepDt, bounceThreshold, frictionOffsetThreshold, 
				correlationDist, biasCoefficient, blockAllocator);

			getContactManagerConstraintDesc(*cmOutput, *cm, desc);

			unit.frictionDataPtr = blockDesc.frictionPtr;
			unit.frictionPatchCount = blockDesc.frictionCount;
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
		PxReal ccdMaxSeparation, PxSolverBodyData* solverBodyData, PxsConstraintBlockManager& blockManager,
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

			PX_ASSERT(desc.constraintLengthOver16 == DY_SC_TYPE_RB_1D);
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

			SetupSolverConstraint(shaderPrepDesc, prepDesc, blockAllocator, dt, invDt, Z);

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

			PX_ASSERT(desc.constraintLengthOver16 == DY_SC_TYPE_RB_CONTACT);
			
			PxSolverContactDesc blockDesc;
			PxsContactManager* cm = reinterpret_cast<PxsContactManager*>(desc.constraint);
			PxcNpWorkUnit& unit = cm->getWorkUnit();
			PxsContactManagerOutput* cmOutput = &outputs.getContactManager(unit.mNpIndex);

			PxSolverBodyData& data0 = desc.linkIndexA != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyADataIndex];
			PxSolverBodyData& data1 = desc.linkIndexB != PxSolverConstraintDesc::RIGID_BODY ? solverBodyData[0] : solverBodyData[desc.bodyBDataIndex];

			blockDesc.data0 = &data0;
			blockDesc.data1 = &data1;

			PxU8 flags = unit.rigidCore0->mFlags;
			if (unit.rigidCore1)
				flags |= PxU8(unit.rigidCore1->mFlags);

			blockDesc.bodyFrame0 = unit.rigidCore0->body2World;
			blockDesc.bodyFrame1 = unit.rigidCore1 ? unit.rigidCore1->body2World : id;
			blockDesc.shapeInteraction = cm->getShapeInteraction();
			blockDesc.contactForces = cmOutput->contactForces;
			blockDesc.desc = &desc;
			blockDesc.body0 = desc.bodyA;
			blockDesc.body1 = desc.bodyB;
			blockDesc.hasForceThresholds = !!(unit.flags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);
			blockDesc.disableStrongFriction = !!(unit.flags & PxcNpWorkUnitFlag::eDISABLE_STRONG_FRICTION);
			blockDesc.bodyState0 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY0) ? PxSolverContactDesc::eARTICULATION : PxSolverContactDesc::eDYNAMIC_BODY;
			blockDesc.bodyState1 = (unit.flags & PxcNpWorkUnitFlag::eARTICULATION_BODY1) ? PxSolverContactDesc::eARTICULATION : (unit.flags & PxcNpWorkUnitFlag::eHAS_KINEMATIC_ACTOR) ? PxSolverContactDesc::eKINEMATIC_BODY :
				((unit.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1) ? PxSolverContactDesc::eDYNAMIC_BODY : PxSolverContactDesc::eSTATIC_BODY);
			//blockDesc.flags = unit.flags;

			PxReal dominance0 = unit.dominance0 ? 1.f : 0.f;
			PxReal dominance1 = unit.dominance1 ? 1.f : 0.f;

			blockDesc.invMassScales.linear0 = blockDesc.invMassScales.angular0 = dominance0;
			blockDesc.invMassScales.linear1 = blockDesc.invMassScales.angular1 = dominance1;
			blockDesc.restDistance = unit.restDistance;
			blockDesc.frictionPtr = unit.frictionDataPtr;
			blockDesc.frictionCount = unit.frictionPatchCount;
			blockDesc.maxCCDSeparation = (flags & PxRigidBodyFlag::eENABLE_SPECULATIVE_CCD) ? ccdMaxSeparation : PX_MAX_F32;
			blockDesc.offsetSlop = unit.mOffsetSlop;

			createFinalizeSolverContacts(blockDesc, *cmOutput, threadContext, invDt, dt, bounceThreshold, frictionOffsetThreshold,
				correlationDist, blockAllocator, Z);

			getContactManagerConstraintDesc(*cmOutput, *cm, desc);

			unit.frictionDataPtr = blockDesc.frictionPtr;
			unit.frictionPatchCount = blockDesc.frictionCount;
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


	static void setupDrive(ArticulationInternalConstraint* constraints,
		bool hasDrive, PxArticulationDriveType::Enum driveType, PxReal stiffness, PxReal damping, const PxReal dt,
		const PxReal unitResponse, const PxReal recipResponse, const PxReal error, const PxReal targetVelocity, const bool isTGS,
		const PxReal maxForce)
	{
		if (hasDrive)
		{
			PxReal x = 0.f;


			if (driveType == PxArticulationDriveType::eTARGET)
			{
				stiffness = 1e+25f;
				damping = 0.f;
				driveType = PxArticulationDriveType::eFORCE;
			}
			else if (driveType == PxArticulationDriveType::eVELOCITY)
			{
				damping = 1e+25f;
				stiffness = 0.f;
				driveType = PxArticulationDriveType::eFORCE;
			}

			const PxReal a = dt * (dt*stiffness + damping);
			const PxReal b = dt * (damping * targetVelocity);// + stiffness * (targetPos - jointPos));
			const PxReal aDamp = dt * dt * (damping + stiffness);// + stiffness * (targetPos - jointPos));

			PxReal driveBiasCoefficient = 0.f;

			switch (driveType)
			{
			case PxArticulationDriveType::eFORCE:
			{

				x = unitResponse > 0.f ? 1.0f / (1.0f + a*unitResponse) : 0.f;
				PxReal xDamp = unitResponse > 0.f ? 1.0f / (1.0f + aDamp*unitResponse) : 0.f;
				constraints->driveTargetVel = x * b;
				constraints->driveVelMultiplier = -x * a;
				driveBiasCoefficient = stiffness * x * dt;
				constraints->driveBiasCoefficient = driveBiasCoefficient + xDamp * damping * dt;
				break;
			}

			case PxArticulationDriveType::eACCELERATION:
			{
				x = 1.0f / (1.0f + a);
				PxReal xDamp = 1.0f / (1.0f + aDamp);
				constraints->driveTargetVel = x * b*recipResponse;
				constraints->driveVelMultiplier = -x * a*recipResponse;
				driveBiasCoefficient = stiffness * x * recipResponse * dt;
				constraints->driveBiasCoefficient = driveBiasCoefficient + xDamp * damping * dt * recipResponse;
				break;
			}
			//KS - this is required to avoid a warning on Linux
			case PxArticulationDriveType::eTARGET:
			case PxArticulationDriveType::eVELOCITY:
			case PxArticulationDriveType::eNONE:
				break;

			}

			const PxReal im = 1.0f - x;
			constraints->driveInitialBias = error * driveBiasCoefficient;
			constraints->driveImpulseMultiplier = isTGS ? 1.f : im;
			constraints->maxDriveForce = maxForce;
			constraints->driveForce = 0.f;
		}
		else
		{
			constraints->driveTargetVel = 0.f;
			constraints->driveInitialBias = 0.f;
			constraints->driveBiasCoefficient = 0.f;
			constraints->driveVelMultiplier = 0.f;
			constraints->driveImpulseMultiplier = 0.f;
			constraints->maxDriveForce = 0.f;
			constraints->driveForce = 0.f;
		}
	}

	void setupComplexLimit(ArticulationLink* links, Cm::SpatialVectorF* Z, ArticulationData& data, const PxU32 linkID, 
		const PxReal angle, const PxReal lowLimit, const PxReal highLimit, const PxVec3& axis, const PxReal cfm, ArticulationInternalConstraint& complexConstraint,
		ArticulationInternalLimit& limit)
	{
		Cm::SpatialVectorV deltaVA, deltaVB;
		FeatherstoneArticulation::getImpulseSelfResponse(links, Z, data,
			links[linkID].parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
			linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

		const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
		const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

		const PxReal r0 = deltaV0.angular.dot(axis);
		const PxReal r1 = deltaV1.angular.dot(axis);

		const PxReal unitResponse = r0 - r1;

		const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / (cfm + unitResponse) : 0.0f;

		complexConstraint.row0 = Cm::UnAlignedSpatialVector(PxVec3(0), axis);
		complexConstraint.row1 = Cm::UnAlignedSpatialVector(PxVec3(0), axis);
		complexConstraint.deltaVA.top = unsimdRef(deltaVA).angular;
		complexConstraint.deltaVA.bottom = unsimdRef(deltaVA).linear;
		complexConstraint.deltaVB.top = unsimdRef(deltaVB).angular;
		complexConstraint.deltaVB.bottom = unsimdRef(deltaVB).linear;
		complexConstraint.recipResponse = recipResponse;
		complexConstraint.response = unitResponse;
		complexConstraint.isLinearConstraint = true;
		limit.errorLow = angle - lowLimit;
		limit.errorHigh = highLimit - angle;
		limit.lowImpulse = 0.f;
		limit.highImpulse = 0.f;
	}


	void FeatherstoneArticulation::setupInternalConstraintsRecursive(
		ArticulationLink* links,
		const PxU32 linkCount,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
		const PxReal stepDt,
		const PxReal dt,
		const PxReal invDt,
		const PxReal erp,
		const bool isTGSSolver, 
		const PxU32 linkID,
		const PxReal maxForceScale)
	{
		const ArticulationLink& link = links[linkID];

		ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
		ArticulationJointTargetData& jointTranDatum = data.getJointTranData(linkID);
		const PxReal* jPosition = &data.getJointPositions()[jointDatum.jointOffset];
		PX_UNUSED(jPosition);

		const ArticulationLink& pLink = links[link.parent];

		const ArticulationJointCore& j = *link.inboundJoint;

		//const bool jointDrive = (j.driveType != PxArticulationJointDriveType::eNONE);

		bool hasFriction = j.frictionCoefficient > 0.f;

		const PxReal fCoefficient = j.frictionCoefficient * stepDt;
		

		const PxU32 limitedRows = jointDatum.limitMask;

		PxU8 driveRows = 0;

		for (PxU32 i = 0; i < PxArticulationAxis::eCOUNT; ++i)
		{
			if (j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f))
				driveRows++;
		}

		const PxU8 frictionRows = hasFriction ? jointDatum.dof : PxU8(0);

		const PxU8 constraintCount = PxU8(driveRows + frictionRows + limitedRows);
		if (!constraintCount)
		{
			//Skip these constraints...
			//constraints += jointDatum.dof;
			jointDatum.dofInternalConstraintMask = 0;
		}
		else
		{
			const PxReal transmissionForce = data.getTransmittedForce(linkID).magnitude() * fCoefficient;

			const PxTransform cA2w = pLink.bodyCore->body2World.transform(j.parentPose);
			const PxTransform cB2w = link.bodyCore->body2World.transform(j.childPose);

			const PxU32 parent = link.parent;

			const PxReal cfm = PxMax(link.cfm, pLink.cfm);


			//Linear, then angular...

			PxVec3 driveError(0.f);
			PxVec3 angles(0.f);
			PxVec3 row[3];
			if (j.jointType == PxArticulationJointType::eSPHERICAL && jointDatum.dof > 1)
			{
				//It's a spherical joint. We can't directly work on joint positions with spherical joints, so we instead need to compute the quaternion
				//and from that compute the joint error projected onto the DOFs. This will yield a rotation that is singularity-free, where the joint 
				//angles match the target joint angles provided provided the angles are within +/- Pi around each axis. Spherical joints do not support
				//quaternion double cover cases/wide angles.

				PxVec3 driveAxis(0.f);

				bool hasAngularDrives = false;

				PxU32 tmpDofId = 0;

				for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
				{
					if (j.motion[i] != PxArticulationMotion::eLOCKED)
					{
						const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].driveType != PxArticulationDriveType::eNONE);

						if (hasDrive)
						{
							const PxVec3 axis = data.mMotionMatrix[jointDatum.jointOffset + tmpDofId].top;
							PxReal target = jointTranDatum.targetJointPosition[tmpDofId];

							driveAxis += axis * target;
							hasAngularDrives = true;

						}

						tmpDofId++;
					}
				}

				
				{

					PxQuat qB2qA = cA2w.q.getConjugate() * cB2w.q;

					{
						//Spherical joint drive calculation using 3x child-space Euler angles
						if (hasAngularDrives)
						{
							PxReal angle = driveAxis.normalize();

							if (angle < 1e-12f)
							{
								driveAxis = PxVec3(1.f, 0.f, 0.f);
								angle = 0.f;
							}

							PxQuat targetQ = PxQuat(angle, driveAxis);

							if (targetQ.dot(qB2qA) < 0.f)
								targetQ = -targetQ;

							driveError = -2.f * (targetQ.getConjugate() * qB2qA).getImaginaryPart();
						}
						
						for (PxU32 i = 0, tmpDof = 0; i < PxArticulationAxis::eX; ++i)
						{
							if (j.motion[i] != PxArticulationMotion::eLOCKED)
							{
								angles[i] = data.mJointPosition[j.jointOffset + tmpDof];
								row[i] = data.mWorldMotionMatrix[jointDatum.jointOffset + tmpDof].top;
								tmpDof++;
							}
						}
					}
				}
			}
			else
			{
				for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
				{
					if (j.motion[i] != PxArticulationMotion::eLOCKED)
					{
						driveError[i] = jointTranDatum.targetJointPosition[0] - data.mJointPosition[j.jointOffset];
						angles[i] = data.mJointPosition[j.jointOffset];
						row[i] = data.mWorldMotionMatrix[jointDatum.jointOffset].top;
					}
				}
			}

			PxU32 dofId = 0;

			PxU8 dofMask = 0;
			for (PxU32 i = 0; i < PxArticulationAxis::eX; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].driveType != PxArticulationDriveType::eNONE);

					
					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{
						dofMask |= (1 << dofId);
						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = row[i];

	
						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, Z, data,
							parent, Cm::SpatialVector(PxVec3(0), axis), deltaVA,
							linkID, Cm::SpatialVector(PxVec3(0), -axis), deltaVB);

				
						const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
						const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

						const PxReal r0 = deltaV0.angular.dot(axis);
						const PxReal r1 = deltaV1.angular.dot(axis);

						const PxReal unitResponse = r0 - r1;

						const PxReal recipResponse = unitResponse <= 0.f ? 0.f : 1.0f / (unitResponse+cfm);

						const PxU32 count = data.mInternalConstraints.size();
						data.mInternalConstraints.forceSize_Unsafe(count + 1);
						ArticulationInternalConstraint* constraints = &data.mInternalConstraints[count];

						

						constraints->recipResponse = recipResponse;
						constraints->response = unitResponse;
						constraints->row0 = Cm::SpatialVectorF(PxVec3(0), axis);
						constraints->row1 = Cm::SpatialVectorF(PxVec3(0), axis);
						constraints->deltaVA.top = unsimdRef(deltaVA).angular;
						constraints->deltaVA.bottom = unsimdRef(deltaVA).linear;
						constraints->deltaVB.top = unsimdRef(deltaVB).angular;
						constraints->deltaVB.bottom = unsimdRef(deltaVB).linear;
						constraints->isLinearConstraint = false;

						constraints->frictionForce = 0.f;
						constraints->maxFrictionForce = hasFriction ? transmissionForce : 0.f;
						constraints->frictionForceCoefficient = isTGSSolver ? 0.f : 1.f;

						setupDrive(constraints, hasDrive, j.drives[i].driveType,
							j.drives[i].stiffness, j.drives[i].damping, stepDt, unitResponse, recipResponse, driveError[i], jointTranDatum.targetJointVelocity[dofId], isTGSSolver, 
							j.drives[i].maxForce * maxForceScale);

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

					}

					dofId++;
				}
			}

			for (PxU32 i = PxArticulationAxis::eX; i < PxArticulationAxis::eCOUNT; ++i)
			{
				if (j.motion[i] != PxArticulationMotion::eLOCKED)
				{
					const bool hasDrive = (j.motion[i] != PxArticulationMotion::eLOCKED && j.drives[i].maxForce > 0.f && (j.drives[i].stiffness > 0.f || j.drives[i].damping > 0.f));

					if (j.motion[i] == PxArticulationMotion::eLIMITED || hasDrive || frictionRows)
					{

						dofMask |= (1 << dofId);
						//Impulse response vector and axes are common for all constraints on this axis besides locked axis!!!
						const PxVec3 axis = data.mWorldMotionMatrix[jointDatum.jointOffset + dofId].bottom;
						const PxVec3 ang0 = (cA2w.p - pLink.bodyCore->body2World.p).cross(axis);
						const PxVec3 ang1 = (cB2w.p - link.bodyCore->body2World.p).cross(axis);

						Cm::SpatialVectorV deltaVA, deltaVB;
						FeatherstoneArticulation::getImpulseSelfResponse(links, Z, data,
							links[linkID].parent, Cm::SpatialVector(axis, ang0), deltaVA,
							linkID, Cm::SpatialVector(-axis, -ang1), deltaVB);

						//Now add in friction rows, then limit rows, then drives...

						const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
						const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

						const PxReal r0 = deltaV0.linear.dot(axis) + deltaV0.angular.dot(ang0);
						const PxReal r1 = deltaV1.linear.dot(axis) + deltaV1.angular.dot(ang1);

						const PxReal unitResponse = r0 - r1;

						//const PxReal recipResponse = unitResponse > DY_ARTICULATION_MIN_RESPONSE ? 1.0f / (unitResponse+cfm) : 0.0f;
						const PxReal recipResponse = 1.0f / (unitResponse + cfm);

						const PxU32 count = data.mInternalConstraints.size();
						data.mInternalConstraints.forceSize_Unsafe(count + 1);
						ArticulationInternalConstraint* constraints = &data.mInternalConstraints[count];

						constraints->response = unitResponse;
						constraints->recipResponse = recipResponse;
						constraints->row0 = Cm::SpatialVectorF(axis, ang0);
						constraints->row1 = Cm::SpatialVectorF(axis, ang1);
						constraints->deltaVA.top = unsimdRef(deltaVA).angular;
						constraints->deltaVA.bottom = unsimdRef(deltaVA).linear;
						constraints->deltaVB.top = unsimdRef(deltaVB).angular;
						constraints->deltaVB.bottom = unsimdRef(deltaVB).linear;
						constraints->isLinearConstraint = true;

						constraints->frictionForce = 0.f;
						constraints->maxFrictionForce = hasFriction ? transmissionForce : 0.f;

						constraints->frictionForceCoefficient = isTGSSolver ? 0.f : 1.f;

						setupDrive(constraints, hasDrive, j.drives[i].driveType,
							j.drives[i].stiffness, j.drives[i].damping, stepDt, unitResponse, recipResponse,
							jointTranDatum.targetJointPosition[dofId] - data.mJointPosition[j.jointOffset + dofId],
							jointTranDatum.targetJointVelocity[dofId], isTGSSolver, j.drives[i].maxForce * maxForceScale);

						if (j.motion[i] == PxArticulationMotion::eLIMITED)
						{
							const PxU32 limitCount = data.mInternalLimits.size();
							data.mInternalLimits.forceSize_Unsafe(limitCount + 1);
							ArticulationInternalLimit* limits = &data.mInternalLimits[limitCount];
							const PxReal jPos = data.mJointPosition[j.jointOffset + dofId];
							limits->errorHigh = j.limits[i].high - jPos;
							limits->errorLow = jPos - j.limits[i].low;
							limits->lowImpulse = 0.f;
							limits->highImpulse = 0.f;
						}

						
					}
					dofId++;
				}
			}

			if (jointDatum.limitMask)
			{
				for (PxU32 dof = 0; dof < jointDatum.dof; ++dof)
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

						setupComplexLimit(links, Z, data, linkID, angle,
							low, high, axis, cfm, *constraints, *limits++);
					}
				}
			}
			jointDatum.dofInternalConstraintMask = dofMask;

		}		

		const PxU32 numChildren = link.mNumChildren;
		const PxU32 offset = link.mChildrenStartIndex;
		for (PxU32 i = 0; i < numChildren; ++i)
		{
			const PxU32 child = offset + i;
			setupInternalConstraintsRecursive(links, linkCount, fixBase, data, Z, stepDt, dt, invDt, erp, isTGSSolver, child, maxForceScale);
		}

	}

	void FeatherstoneArticulation::setupInternalSpatialTendonConstraintsRecursive(
		ArticulationLink* links,
		ArticulationAttachment* attachments,
		const PxU32 attachmentCount,
		const PxVec3& pAttachPoint,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
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
		
		ArticulationLink& cLink = links[attachment.linkInd];

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
				const PxU32 child = ArticulationLowestSetBit(children);

				setupInternalSpatialTendonConstraintsRecursive(links, attachments, attachmentCount, cAttachPoint, fixBase, data, Z, stepDt,
					isTGSSolver, child, stiffness, damping, limitStiffness, u, startLink,
					startAxis, startRaXn);
			}
		}
		else
		{
			const PxVec3 axis = distance > 0.001f ? dif / distance : PxVec3(0.f);

			const PxVec3 rbXn = rb.cross(axis);

			Cm::SpatialVectorV deltaVA, deltaVB;
			FeatherstoneArticulation::getImpulseSelfResponse(links, Z, data,
				startLink, Cm::SpatialVector(startAxis, startRaXn), deltaVA,
				attachment.linkInd, Cm::SpatialVector(-axis, -rbXn), deltaVB);

			const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
			const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

			const PxReal r0 = deltaV0.linear.dot(startAxis) + deltaV0.angular.dot(startRaXn);
			const PxReal r1 = deltaV1.linear.dot(axis) + deltaV1.angular.dot(rbXn);

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

			const PxReal a = stepDt * (stepDt*stiffness + damping);
			const PxReal a2 = stepDt * (stepDt*limitStiffness + damping);

			PxReal x = unitResponse > 0.f ? 1.0f / (1.0f + a * unitResponse) : 0.f;
			PxReal x2 = unitResponse > 0.f ? 1.0f / (1.0f + a2 * unitResponse) : 0.f;


			constraint->velMultiplier = -x * a;// * unitResponse;
			//constraint->velMultiplier = -x * damping*stepDt;

			constraint->impulseMultiplier = isTGSSolver ? 1.f : 1.f - x;
			constraint->biasCoefficient = (-stiffness * x * stepDt);//*unitResponse;
			constraint->appliedForce = 0.f;

			constraint->accumulatedLength = u;// + u*0.2f;
			constraint->restDistance = attachment.restLength;
			constraint->lowLimit = attachment.lowLimit;
			constraint->highLimit = attachment.highLimit;

			constraint->limitBiasCoefficient = (-limitStiffness * x2 * stepDt);//*unitResponse;
			constraint->limitImpulseMultiplier = isTGSSolver ? 1.f : 1.f - x2;
			constraint->limitAppliedForce = 0.f;
		}
	}


	void FeatherstoneArticulation::updateSpatialTendonConstraintsRecursive(ArticulationAttachment* attachments, ArticulationData& data, const PxU32 attachmentID, PxReal accumLength,
		const PxVec3& pAttachPoint)
	{
		ArticulationAttachment& attachment = attachments[attachmentID];

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
				const PxU32 child = ArticulationLowestSetBit(children);

				updateSpatialTendonConstraintsRecursive(attachments, data, child, u, cAttachPoint);
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
		ArticulationLink* links,
		ArticulationTendonJoint* tendonJoints,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
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

		ArticulationLink& cLink = links[tendonJoint.linkInd];

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
			PxVec3 axis, rbXn;
			if (tendonJointAxis < PxArticulationAxis::eX)
			{
				const PxVec3 tAxis = data.mWorldMotionMatrix[jointDatum.jointOffset + dofIndex].top;
				axis = PxVec3(0.f);
				rbXn = tAxis;
			}
			else
			{
				const PxTransform cB2w = cBody2World.transform(joint.childPose);
				const PxVec3 tAxis = data.mWorldMotionMatrix[jointDatum.jointOffset + dofIndex].bottom;
				axis = tAxis;
				rbXn = (cB2w.p - cBody2World.p).cross(axis);
			}
		

			Cm::SpatialVectorV deltaVA, deltaVB;
			FeatherstoneArticulation::getImpulseSelfResponse(links, Z, data,
				startLink, Cm::SpatialVector(startAxis, startRaXn), deltaVA,
				tendonJoint.linkInd, Cm::SpatialVector(-axis, -rbXn), deltaVB);

			

			const Cm::SpatialVector& deltaV0 = unsimdRef(deltaVA);
			const Cm::SpatialVector& deltaV1 = unsimdRef(deltaVB);

			/*const PxU32 pLinkInd = cLink.parent;
			printf("(%i, %i) deltaV1(%f, %f, %f, %f, %f, %f)\n",
				pLinkInd, tendonJoint.linkInd, deltaV1.linear.x, deltaV1.linear.y, deltaV1.linear.z, deltaV1.angular.x,
				deltaV1.angular.y, deltaV1.angular.z);*/

			const PxReal r0 = deltaV0.linear.dot(startAxis) + deltaV0.angular.dot(startRaXn);
			const PxReal r1 = deltaV1.linear.dot(axis) + deltaV1.angular.dot(rbXn);


			const PxReal unitResponse = r0 - r1;

			const PxReal recipResponse = 1.0f / (unitResponse + cfm);

			const PxU32 count = data.mInternalFixedTendonConstraints.size();
			data.mInternalFixedTendonConstraints.forceSize_Unsafe(count + 1);

			ArticulationInternalTendonConstraint* constraint = &data.mInternalFixedTendonConstraints[count];
			tendonJoint.mConstraintInd = PxU16(count);


			constraint->row0 = Cm::UnAlignedSpatialVector(startAxis, startRaXn);
			constraint->row1 = Cm::UnAlignedSpatialVector(axis, rbXn);
			constraint->deltaVA = r0; //We only need to record the change in velocity projected onto the dof for this!
			constraint->deltaVB = Cm::UnAlignedSpatialVector(deltaV1.angular, deltaV1.linear);
			constraint->linkID0 = startLink;
			constraint->linkID1 = tendonJoint.linkInd;
			constraint->recipResponse = recipResponse;

			const PxReal a = stepDt * (stepDt*stiffness + damping);

			const PxReal a2 = stepDt * (stepDt*limitStiffness + damping);

			PxReal x = unitResponse > 0.f ? 1.0f / (1.0f + a * unitResponse) : 0.f;

			PxReal x2 = unitResponse > 0.f ? 1.0f / (1.0f + a2* unitResponse) : 0.f;


			constraint->velMultiplier = -x * a;// * unitResponse;
			
			constraint->impulseMultiplier = isTGSSolver ? 1.f : 1.f - x;
			constraint->biasCoefficient = (-stiffness * x * stepDt);//*unitResponse;
			constraint->appliedForce = 0.f;
			//constraint->accumulatedLength = jointPose;

			constraint->limitImpulseMultiplier = isTGSSolver ? 1.f : 1.f - x2;
			constraint->limitBiasCoefficient = (-limitStiffness * x2 * stepDt);//*unitResponse;
			constraint->limitAppliedForce = 0.f;
			
			/*printf("(%i, %i) r0 %f, r1 %f cmf %f unitResponse %f recipResponse %f a %f x %f\n",
				pLinkInd, tendonJoint.linkInd, r0, r1, cLink.cfm, unitResponse, recipResponse, a, x);*/
		}

		const PxU32 childCount = tendonJoint.childCount;

		if (childCount)
		{

			for (ArticulationBitField children = tendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = ArticulationLowestSetBit(children);

				setupInternalFixedTendonConstraintsRecursive(links, tendonJoints, fixBase, data, Z, stepDt,
					isTGSSolver, child, stiffness, damping, limitStiffness, startLink, startAxis, startRaXn);
			}
		}
	}


	void FeatherstoneArticulation::setupInternalConstraints(
		ArticulationLink* links,
		const PxU32 linkCount,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
		PxReal stepDt,
		PxReal dt,
		PxReal invDt,
		PxReal erp,
		bool isTGSSolver)
	{
		PX_UNUSED(linkCount);

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

			setupInternalConstraintsRecursive(links, linkCount, fixBase, data, Z, stepDt, dt, invDt, erp, isTGSSolver, child, maxForceScale);

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
			ArticulationLink& pLink = links[startLink];
			const PxTransform pBody2World = pLink.bodyCore->body2World;
			const PxVec3 ra = pBody2World.q.rotate(pAttachment.relativeOffset);
			const PxVec3 pAttachPoint = pBody2World.p + ra;
			
			
			for (ArticulationAttachmentBitField children = pAttachment.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = ArticulationLowestSetBit(children);

				ArticulationAttachment& attachment = attachments[child];
				ArticulationLink& cLink = links[attachment.linkInd];
				const PxTransform cBody2World = cLink.bodyCore->body2World;
				const PxVec3 rb = cBody2World.q.rotate(attachment.relativeOffset);
				const PxVec3 cAttachPoint = cBody2World.p + rb;

				const PxVec3 axis = (pAttachPoint - cAttachPoint).getNormalized();
				const PxVec3 raXn = ra.cross(axis);

				setupInternalSpatialTendonConstraintsRecursive(links, attachments, tendon->getNumAttachments(), pAttachPoint, fixBase, data, Z, stepDt, isTGSSolver,
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
			ArticulationLink& pLink = links[startLinkInd];
			const PxTransform& pBody2World = pLink.bodyCore->body2World;
			

			for (ArticulationAttachmentBitField children = pTendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = ArticulationLowestSetBit(children);

				ArticulationTendonJoint& cTendonJoint = tendonJoints[child];

				ArticulationLink& cLink = links[cTendonJoint.linkInd];

				ArticulationJointCore* joint = cLink.inboundJoint;

				PX_ASSERT(joint != NULL);

				ArticulationJointCoreData* jointDatum = &data.getJointData(cTendonJoint.linkInd);

				PxU16 tendonJointAxis = cTendonJoint.axis;

				PX_ASSERT(joint->motion[tendonJointAxis] != PxArticulationMotion::eLOCKED);

				PxVec3 startAxis, raXn;
				PxU32 dofIndex = joint->invDofIds[tendonJointAxis];

				if (tendonJointAxis < PxArticulationAxis::eX)
				{
					const PxVec3 axis = data.mWorldMotionMatrix[jointDatum->jointOffset + dofIndex].top;
					startAxis = PxVec3(0.f);
					raXn = axis;
				}
				else
				{
					const PxTransform cA2w = pBody2World.transform(joint->parentPose);
					const PxVec3 axis = data.mWorldMotionMatrix[jointDatum->jointOffset + dofIndex].bottom;
					const PxVec3 ang0 = (cA2w.p - pBody2World.p).cross(axis);
					startAxis = axis;
					raXn = ang0;
				}


				setupInternalFixedTendonConstraintsRecursive(links, tendonJoints, fixBase, data, Z, stepDt, isTGSSolver,
					child, tendon->mStiffness, tendon->mDamping, tendon->mLimitStiffness, startLinkInd, startAxis, raXn);
			}
		}

	}


	PxU32 FeatherstoneArticulation::setupSolverConstraints(
		ArticulationLink* links,
		const PxU32 linkCount,
		const bool fixBase,
		ArticulationData& data,
		Cm::SpatialVectorF* Z,
		PxU32& acCount)
	{
		acCount = 0;

		setupInternalConstraints(links, linkCount, fixBase, data, Z, data.getDt(), data.getDt(), 1.f / data.getDt(), 1.f, false);

		return 0;
	}


	PxU32 FeatherstoneArticulation::setupSolverConstraintsTGS(const ArticulationSolverDesc& articDesc,
		PxReal dt,
		PxReal invDt,
		PxReal totalDt,
		const PxReal biasCoefficient,
		PxU32& acCount,
		Cm::SpatialVectorF* Z)
	{
		PX_UNUSED(dt);
		PX_UNUSED(totalDt);
		acCount = 0;

		FeatherstoneArticulation* thisArtic = static_cast<FeatherstoneArticulation*>(articDesc.articulation);

		ArticulationLink* links = thisArtic->mArticulationData.getLinks();
		const PxU32 linkCount = thisArtic->mArticulationData.getLinkCount();
		const bool fixBase = thisArtic->mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		const PxReal erp = PxMin(0.7f, biasCoefficient);

		thisArtic->setupInternalConstraints(links, linkCount, fixBase, thisArtic->mArticulationData, Z, dt, totalDt, invDt, erp, true);

		return 0;
	}

	void FeatherstoneArticulation::teleportLinks(ArticulationData& data)
	{
	
		ArticulationLink* links = mArticulationData.getLinks();
	
		ArticulationJointCoreData* jointData = mArticulationData.getJointData();

		const PxReal* jointPositions = data.getJointPositions();

		const PxU32 linkCount = mArticulationData.getLinkCount();

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

			const PxQuat relativeQuat = mArticulationData.mRelativeQuat[linkID];

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				newParentToChild = relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = data.mMotionMatrix[jointDatum.jointOffset].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
			{
				const PxVec3& u = data.mMotionMatrix[jointDatum.jointOffset].top;

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
					for (PxU32 d = 0; d < jointDatum.dof; ++d)
					{
						axis += data.mMotionMatrix[jointDatum.jointOffset + d].top * -jPosition[d];
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

		ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		Cm::SpatialVectorF* motionVelocities = data.getMotionVelocities();
		const PxReal* jointVelocities = data.mJointVelocity.begin();

		// sync root motion vel:
		const PxsBodyCore& rootBodyCore = *links[0].bodyCore;
		motionVelocities[0].top = rootBodyCore.angularVelocity;
		motionVelocities[0].bottom = rootBodyCore.linearVelocity;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			
			PxsBodyCore& bodyCore = *link.bodyCore;

			PxTransform body2World = bodyCore.body2World;

			ArticulationLink& plink = links[link.parent];
			const PxsBodyCore& pbodyCore = *plink.bodyCore;

			Cm::SpatialVectorF parentVel(pbodyCore.angularVelocity, pbodyCore.linearVelocity);

			PxTransform pBody2World = pbodyCore.body2World;

			const PxVec3 rw = body2World.p - pBody2World.p;

			Cm::SpatialVectorF vel = FeatherstoneArticulation::translateSpatialVector(-rw,parentVel);

			if (jointVelocities)
			{
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];

				Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					deltaV += data.mMotionMatrix[jointDatum.jointOffset + ind] * jVelocity[ind];
				}

				vel.top += body2World.q.rotate(deltaV.top);
				vel.bottom += body2World.q.rotate(deltaV.bottom);
			}

			bodyCore.linearVelocity = vel.bottom;
			bodyCore.angularVelocity = vel.top;
			motionVelocities[linkID] = vel;
		}
	}

	void FeatherstoneArticulation::jcalc(ArticulationData& data, bool forceUpdate)
	{	
		const ArticulationLink* links = data.getLinks();
		ArticulationJointCoreData* jointData = data.getJointData();
		ArticulationJointTargetData* jointTranData = data.getJointTranData();
		const PxU32 linkCount = data.getLinkCount();

		PxU32 totalDof = 0;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = jointData[linkID];
			ArticulationJointTargetData& jointTranDatum = jointTranData[linkID];
				
			PX_CHECK_AND_RETURN(joint->jointType != PxArticulationJointType::eUNDEFINED, "FeatherstoneArticulation::jcalc application need to define valid joint type and motion");
			
			//compute joint dof
			jointDatum.computeJointDof(joint, forceUpdate, data.mJointAxis.begin() + totalDof);
			joint->setJointFrame(jointDatum, &data.mMotionMatrix[totalDof], &data.mJointAxis[totalDof], forceUpdate, mArticulationData.mRelativeQuat[linkID]);
			const PxU8 dof = jointDatum.dof;

			jointTranDatum.setJointVelocityDrive(joint, dof);
			jointTranDatum.setJointPoseDrive(joint, dof);
			jointTranDatum.setArmature(joint, dof);

			jointDatum.jointOffset = totalDof;
			joint->jointOffset = totalDof;
			totalDof += dof;
		}

		if (totalDof != mArticulationData.getDofs())
		{
			mArticulationData.resizeJointData(totalDof);
		}
		mArticulationData.setDofs(totalDof);
	}

	//compute link's spatial inertia tensor
	void  FeatherstoneArticulation::computeSpatialInertia(ArticulationData& data)
	{
		for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);
			//ArticulationLinkData& linkDatum = data.getLinkData(linkID);

			const PxsBodyCore& core = *link.bodyCore;

			const PxVec3& ii = core.inverseInertia;

			const PxReal m = core.inverseMass == 0.f ? 0.f : 1.0f / core.inverseMass;

			SpatialMatrix& worldArticulatedInertia = data.mWorldSpatialArticulatedInertia[linkID];

			//construct inertia matrix
			const PxVec3 inertiaTensor = PxVec3(ii.x == 0.f ? 0.f : (1.f / ii.x), ii.y == 0.f ? 0.f : (1.f / ii.y), ii.z == 0.f ? 0.f : (1.f / ii.z));

			PxMat33 rot(data.getLink(linkID).bodyCore->body2World.q);

			worldArticulatedInertia.topLeft = PxMat33(PxZero);
			worldArticulatedInertia.topRight = PxMat33::createDiagonal(PxVec3(m));
			Cm::transformInertiaTensor(inertiaTensor, rot, worldArticulatedInertia.bottomLeft);

			data.mWorldIsolatedSpatialArticulatedInertia[linkID] = worldArticulatedInertia.bottomLeft;
			data.mMasses[linkID] = m;
		}
	}

	void FeatherstoneArticulation::computeZ(const ArticulationData& data, 
		const PxVec3& gravity, ScratchData& scratchData)
	{
		const Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		const Cm::SpatialVector* externalAccels = scratchData.externalAccels;	

		for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			const PxsBodyCore& core = *link.bodyCore;
			//const PxTransform& body2World = core.body2World;

			const PxMat33& I = data.mWorldSpatialArticulatedInertia[linkID].bottomLeft;

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

	void FeatherstoneArticulation::computeZD(const ArticulationData& data,
		const PxVec3& gravity, ScratchData& scratchData)
	{
		const Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* spatialZAForces = scratchData.spatialZAVectors;
		const Cm::SpatialVector* externalAccels = scratchData.externalAccels;

		const PxReal dt = data.getDt();
		const PxReal invDt = dt < 1e-6f ? PX_MAX_F32 : 1.f / data.mDt;

		for (PxU32 linkID = 0; linkID < data.getLinkCount(); ++linkID)
		{
			ArticulationLink& link = data.getLink(linkID);

			const PxsBodyCore& core = *link.bodyCore;
			//const PxTransform& body2World = core.body2World;

			const PxMat33& I = data.mWorldSpatialArticulatedInertia[linkID].bottomLeft;

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
			if (!core.disableGravity)
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

			if (core.linearDamping > 0.f || core.angularDamping > 0.f)
			{
				const PxReal linDamp = PxMin(core.linearDamping, invDt);
				const PxReal angDamp = PxMin(core.angularDamping, invDt);


				zTmp.top += (v.bottom * linDamp*m) - zTmp.top * linDamp*dt;
				zTmp.bottom += I * (v.top* angDamp) - zTmp.bottom * angDamp*dt;
			}

			const PxReal maxAng = core.maxAngularVelocitySq;
			const PxReal maxLin = core.maxLinearVelocitySq;

			const PxReal angMag = v.top.magnitudeSquared();
			const PxReal linMag = v.bottom.magnitudeSquared();

			if (angMag > maxAng || linMag > maxLin)
			{
				if (angMag > maxAng)
				{
					const PxReal scale = 1.f - PxSqrt(maxAng) / PxSqrt(angMag);
					const PxVec3 tmpaccelerationAng = (I * v.top)*scale;
					zTmp.bottom += tmpaccelerationAng*invDt;
				}

				if (linMag > maxLin)
				{
					const PxReal scale = 1.f - (PxSqrt(maxLin) / PxSqrt(linMag));
					const PxVec3 tmpaccelerationLin = (v.bottom*m*scale);
					PX_UNUSED(tmpaccelerationLin);
					zTmp.top += tmpaccelerationLin*invDt;
				}
			}

			z = zTmp;
		}
	}

	//compute coriolis and centrifugal term
	void FeatherstoneArticulation::computeC(ArticulationData& data, ScratchData& scratchData)
	{
		const PxReal* jointVelocities = scratchData.jointVelocities;
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;

		const PxU32 linkCount = data.getLinkCount();

		coriolisVectors[0] = Cm::SpatialVectorF::Zero();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = data.getLink(linkID);
			const ArticulationJointCoreData& jointDatum = data.getJointData(linkID);

			const PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
			Cm::SpatialVectorF& coriolis = coriolisVectors[linkID];

			
			//const PxTransform& body2World = link.bodyCore->body2World;
			//transform parent link's angular velocity into current link's body space
			const PxVec3 pAngular = scratchData.motionVelocities[link.parent].top;
			
			PxVec3 torque = pAngular.cross(pAngular.cross(data.getRw(linkID)));

			//PX_ASSERT(parentAngular.magnitude() < 100.f);

			PxVec3 force(0.f);
			if (jointDatum.dof > 0)
			{
				Cm::SpatialVectorF relVel(PxVec3(0.f), PxVec3(0.f));
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					//Clamp joint velocity used in coriolis terms to reduce chances of unstable feed-back loops
					const PxReal jV = jVelocity[ind];
					relVel.top += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].top * jV;
					relVel.bottom += data.mWorldMotionMatrix[jointDatum.jointOffset + ind].bottom * jV;
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

	

	void FeatherstoneArticulation::computeRelativeTransformC2P(ArticulationData& data)
	{
		const ArticulationLink* links = data.getLinks();
		const PxU32 linkCount = data.getLinkCount();

		PxTransform* accumulatedPose = data.getAccumulatedPoses();

		accumulatedPose[0] = links[0].bodyCore->body2World;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;
			const PxU32 jointOffset = data.getJointData(linkID).jointOffset;
			const PxU32 dofCount = data.getJointData(linkID).dof;

			const PxTransform& body2World = bodyCore.body2World;

			const ArticulationLink& pLink = links[link.parent];
			const PxsBodyCore& pBodyCore = *pLink.bodyCore;
			const PxTransform& pBody2World = pBodyCore.body2World;

			//const PxTransform tC2P = pBody2World.transformInv(body2World).getNormalized();
			
			data.mRw[linkID] =body2World.p - pBody2World.p;
			
			const Cm::UnAlignedSpatialVector* motionMatrix = &data.mMotionMatrix[jointOffset];
			Cm::UnAlignedSpatialVector* worldMotionMatrix = &data.mWorldMotionMatrix[jointOffset];

			for (PxU32 i = 0; i < dofCount; ++i)
			{
				const Cm::UnAlignedSpatialVector worldRow = motionMatrix[i].rotate(body2World);
				
				worldMotionMatrix[i] = worldRow;
			}

			accumulatedPose[linkID] = body2World;

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

	void FeatherstoneArticulation::computeRelativeTransformC2B(ArticulationData& data)
	{
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		const PxU32 linkCount = data.getLinkCount();

		const ArticulationLink& bLink = links[0];
		const PxTransform& bBody2World = bLink.bodyCore->body2World;

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			const PxTransform& body2World = bodyCore.body2World;

			const PxVec3 rw = body2World.p - bBody2World.p;//body space of link i

			//rotation matrix cToP's inverse is rotation matrix pToC 
			linkDatum.childToBase = rw;
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
			PxGetFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, "Articulation::getDenseJacobian(): commonInit need to be called first to initialize data!");
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

		ArticulationLinkData* linkData = mArticulationData.getLinkData();

		const PxU32 totalDofs = getDofs();
		const PxU32 jointCount = mArticulationData.getLinkCount() - 1;
		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;
		//matrix dims is 
		nCols = (fixBase ? 0 : 6) + totalDofs;
		nRows = (fixBase ? 0 : 6) + jointCount * 6;

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

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)//each iteration of this writes 6 rows in the matrix
		{
			const ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			linkDatum.maxPenBias = bodyCore.maxPenBias;

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
													mArticulationData.getJointData(parentLinkID).jointOffset + (fixBase ? 0 : 6) + mArticulationData.getJointData(parentLinkID).dof :
													6;

				// VR: With parentLinkID == 0 this experssion has two unsigned integer overflows, but the result is still correct.
				const PxU32 parentsDestRow = (fixBase ? 0 : 6) + (parentLinkID - 1) * 6;

				for (PxU32 col = 0; col < parentsLastDestCol; col++)
				{
					//copy downward the 6 cols from parent
					const PxVec3 parentAng(
						jacobian(parentsDestRow + 3, col),
						jacobian(parentsDestRow + 4, col),
						jacobian(parentsDestRow + 5, col)
						);

					const PxVec3 parentAngxRw = parentAng.cross(mArticulationData.getRw(linkID));

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
			for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
			{
				const Cm::UnAlignedSpatialVector& v = mArticulationData.mMotionMatrix[jointDatum.jointOffset + ind];

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
		const PxTransform* accumulatedPoses, const Cm::SpatialVector* externalAccels,  const PxVec3* rws, const Cm::UnAlignedSpatialVector* worldMotionMatrices, 
		const Dy::ArticulationJointCoreData* jointCoreData,
		Dy::ArticulationLinkData *linkData, Dy::ArticulationLink* links, Cm::SpatialVectorF* motionAccelerations, 
		Cm::SpatialVectorF* motionVelocities, 
		Cm::SpatialVectorF* spatialZAForces, Cm::SpatialVectorF* spatialZAInternal, Cm::SpatialVectorF* coriolisVectors, 
		PxMat33* worldIsolatedSpatialArticulatedInertias, PxF32* linkMasses, Dy::SpatialMatrix* worldSpatialArticulatedInertias, 
		const PxU32 jointDofCount,
		PxReal* jointVelocities,
		Cm::SpatialVectorF& rootPreMotionVelocity, PxVec3& com, PxF32& invSumMass)
	{
		PX_UNUSED(jointDofCount);

		const PxReal invDt = dt < 1e-6f ? PX_MAX_F32 : 1.f / dt;

		//Initialise motion velocity, motion acceleration and coriolis vector of root link.
		Cm::SpatialVectorF rootLinkVel;
		{
			const Dy::ArticulationLink& baseLink = links[0];
			const PxsBodyCore& core0 = *baseLink.bodyCore;
			rootLinkVel = fixBase ? Cm::SpatialVectorF::Zero() : Cm::SpatialVectorF(core0.angularVelocity, core0.linearVelocity);
			motionVelocities[0] = rootLinkVel;
			motionAccelerations[0] = fixBase ? Cm::SpatialVectorF::Zero() : motionAccelerations[0];
			coriolisVectors[0] = Cm::SpatialVectorF::Zero();
			rootPreMotionVelocity = rootLinkVel;
		}

		PxReal ratio = 1.f;
		if (jointVelocities)
		{

			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				const ArticulationLink& link = links[linkID];
				const ArticulationJointCoreData& jointDatum = jointCoreData[linkID];
				PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
				const PxReal maxJVelocity = link.inboundJoint->maxJointVelocity;
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					PxReal jVel = jVelocity[ind];
					ratio = (jVel != 0.0f) ? PxMin(ratio, maxJVelocity / PxAbs(jVel)) : ratio;
				}
			}
		}

		PxReal sumMass = 0.f;
		PxVec3 COM(0.f);
		for (PxU32 linkID = 0; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			//Set the maxPemBias and cfm values from bodyCore
			linkDatum.maxPenBias = bodyCore.maxPenBias;
			link.cfm = (fixBase && linkID == 0) ? 0.f : bodyCore.cfmScale * invLengthScale;

			//Read the inertia and mass from bodyCore
			const PxVec3& ii = bodyCore.inverseInertia;
			const PxVec3 inertiaTensor = PxVec3(ii.x == 0.f ? 0.f : (1.f / ii.x), ii.y == 0.f ? 0.f : (1.f / ii.y), ii.z == 0.f ? 0.f : (1.f / ii.z));
			const PxReal invMass = bodyCore.inverseMass;
			const PxReal m = invMass == 0.f ? 0.f : 1.0f / invMass;

			//Compute the inertia matrix Iw = R * I * Rtranspose
			//Compute the articulated inertia.
			PxMat33 Iw; //R * I * Rtranspose
			SpatialMatrix worldArticulatedInertia;
			{
				PxMat33 rot(accumulatedPoses[linkID].q);
				Cm::transformInertiaTensor(inertiaTensor, rot, Iw);
			worldArticulatedInertia.topLeft = PxMat33(PxZero);
			worldArticulatedInertia.topRight = PxMat33::createDiagonal(PxVec3(m));
				worldArticulatedInertia.bottomLeft = Iw;
			}

			//Set the articulated inertia, inertia and mass of the link.
			worldSpatialArticulatedInertias[linkID] = worldArticulatedInertia;
			worldIsolatedSpatialArticulatedInertias[linkID] = Iw;
			linkMasses[linkID] = m;

			//Accumulate the centre of mass.
			sumMass += m;
			COM += accumulatedPoses[linkID].p * m;

			Cm::SpatialVectorF vel;
			if (linkID != 0)
			{
				//Propagate spatial vector of link parent to link's spatial vector.
				const Cm::SpatialVectorF pVel = motionVelocities[link.parent];
				vel = FeatherstoneArticulation::translateSpatialVector(-rws[linkID], pVel);

				//Propagate joint dof velocities to the link's spatial velocity vector.
				//Accumulate spatial forces that the joint applies to the link.
				if (jointVelocities)
				{
					//The coriolis vector depends on the type of joint and the joint motion matrix.
					//However, some terms in the coriolis vector are common to all joint types. 
					//Write down the term that is independent of the joint.
					Cm::SpatialVectorF coriolisVector(PxVec3(PxZero), pVel.top.cross(pVel.top.cross(rws[linkID])));
					const ArticulationJointCoreData& jointDatum = jointCoreData[linkID];
					if (jointDatum.dof)
					{
						//Compute the effect of the joint velocities on the link.
						PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
						Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
						for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
						{
							PxReal jVel = jVelocity[ind] * ratio;
							deltaV += worldMotionMatrices[jointDatum.jointOffset + ind] * jVel;
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
					coriolisVectors[linkID] = coriolisVector;
				}

				//PX_ASSERT(vel.top.isFinite() && PxAbs(vel.top.x) < 10000.f && PxAbs(vel.top.y) < 10000.f && PxAbs(vel.top.z) < 10000.f);
				//PX_ASSERT(vel.bottom.isFinite() && PxAbs(vel.bottom.x) < 10000.f && PxAbs(vel.bottom.y) < 10000.f && PxAbs(vel.bottom.z) < 10000.f);
				motionVelocities[linkID] = vel;
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
			Cm::SpatialVectorF zTmp;
			{
				const PxVec3 g = bodyCore.disableGravity ? PxVec3(PxZero) : gravity;
				const PxVec3 exLinAccel = externalAccels ? externalAccels[linkID].linear : PxVec3(PxZero);
				const PxF32 lindamp = bodyCore.linearDamping > 0.f ?  PxMin(bodyCore.linearDamping, invDt) : 0.0f;
				const PxF32 linscale =  (vel.bottom.magnitudeSquared() > bodyCore.maxLinearVelocitySq) ?  (1.0f - (PxSqrt(bodyCore.maxLinearVelocitySq)/PxSqrt(vel.bottom.magnitudeSquared()))): 0.0f;
				zTmp.top = -(m*((g + exLinAccel)*(1.0f - lindamp*dt)  - vel.bottom*(lindamp + linscale*invDt))); 
			}
			{
				const PxVec3 exAngAccel = externalAccels ? externalAccels[linkID].angular : PxVec3(PxZero);
				const PxF32 angdamp = bodyCore.angularDamping > 0.f ? PxMin(bodyCore.angularDamping, invDt) : 0.0f;
				const PxF32 angscale = (vel.top.magnitudeSquared() > bodyCore.maxAngularVelocitySq) ? (1.0f - (PxSqrt(bodyCore.maxAngularVelocitySq)/PxSqrt(vel.top.magnitudeSquared()))) : 0.0f;
				zTmp.bottom = -(Iw*(exAngAccel*(1.0f - angdamp*dt) - vel.top*(angdamp + angscale*invDt)));
				}
			spatialZAForces[linkID] = zTmp;

			//Account for forces arising from internal accelerations.
			//Note: Mirtich thesis introduces a single spatial zero acceleration force that contains an external [mass*gravity] term and the internal [omega X  (Iw *omega)] term. 
			//We split the spatial zero acceleration force into external (above) and internal (below).
			const Cm::SpatialVectorF zInternal(PxVec3(0.f), vel.top.cross(Iw*vel.top));
			spatialZAInternal[linkID] = zInternal;
		}

		PxReal invMass = 1.f / sumMass;
		com = COM * invMass;
		invSumMass = invMass;
	}


	//compute all links velocities
	void FeatherstoneArticulation::computeLinkVelocities(ArticulationData& data,
		ScratchData& scratchData)
	{
		Cm::SpatialVectorF* coriolisVectors = scratchData.coriolisVectors;
		ArticulationLink* links = data.getLinks();
		ArticulationLinkData* linkData = data.getLinkData();
		const PxU32 linkCount = data.getLinkCount();
		const bool fixBase = data.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		//motion velocities has to be in world space to avoid numerical errors caused by space 
		Cm::SpatialVectorF* motionVelocities = scratchData.motionVelocities;
		Cm::SpatialVectorF* motionAccelerations = scratchData.motionAccelerations;

		PxReal* jointVelocities = scratchData.jointVelocities;

		const ArticulationLink& baseLink = links[0];
		ArticulationLinkData& baseLinkDatum = linkData[0];

		PxsBodyCore& core0 = *baseLink.bodyCore;

		baseLinkDatum.maxPenBias = core0.maxPenBias;

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

		//const PxU32 dofCount = data.mDofs;
		PxReal ratio = 1.f;
		if (jointVelocities)
		{
			
			for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
			{
				const ArticulationLink& link = links[linkID];
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
				const PxReal maxJVelocity = link.inboundJoint->maxJointVelocity;
				for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
				{
					PxReal absJvel = PxAbs(jVelocity[ind]);
					ratio = ratio * absJvel > maxJVelocity ? (maxJVelocity / absJvel) : ratio;
				}
			}
		}

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			const ArticulationLink& link = links[linkID];
			ArticulationLinkData& linkDatum = linkData[linkID];
			const PxsBodyCore& bodyCore = *link.bodyCore;

			linkDatum.maxPenBias = bodyCore.maxPenBias;
			const Cm::SpatialVectorF pVel = motionVelocities[link.parent];
			Cm::SpatialVectorF vel = FeatherstoneArticulation::translateSpatialVector(-mArticulationData.getRw(linkID), pVel);
			const PxTransform& body2World = bodyCore.body2World;

			if (jointVelocities)
			{
				PxVec3 torque = pVel.top.cross(pVel.top.cross(data.getRw(linkID)));
				ArticulationJointCoreData& jointDatum = data.getJointData(linkID);
				PxReal* jVelocity = &jointVelocities[jointDatum.jointOffset];
				
				PxVec3 force(0.f);
				if (jointDatum.dof)
				{

					Cm::UnAlignedSpatialVector deltaV = Cm::UnAlignedSpatialVector::Zero();
					for (PxU32 ind = 0; ind < jointDatum.dof; ++ind)
					{
						PxReal jVel = jVelocity[ind] * ratio;
						//deltaV += data.mWorldMotionMatrix[jointDatum.jointOffset + ind] * jVel;
						deltaV += data.mMotionMatrix[jointDatum.jointOffset+ind].rotate(body2World) * jVel;
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
		const PxQuat& rot, bool isTGS, PxReal elapsedTime,	const PxReal minPenetration)
	{
		PX_UNUSED(isTGS);
		PX_UNUSED(elapsedTime);
		PX_UNUSED(minPenetration);
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
				Dy::solveExtContactStep(desc, linVel0, linVel1, angVel0, angVel1, linMotion0, linMotion1, angMotion0, angMotion1,
					li0, li1, ai0, ai1, true, minPenetration, elapsedTime);
			}
			else
			{
				Dy::solveExt1D(desc, linVel0, linVel1, angVel0, angVel1, linMotion0, linMotion1, angMotion0, angMotion1, 
					rotA, rotB, elapsedTime, li0, li1, ai0, ai1);
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

	void writeBackContact(const PxSolverConstraintDesc& desc, SolverContext& cache,
			PxSolverBodyData& bd0, PxSolverBodyData& bd1);

	void writeBack1D(const PxSolverConstraintDesc& desc, SolverContext&, 
		PxSolverBodyData&, PxSolverBodyData&);

	void writeBackContact(const PxSolverConstraintDesc& desc, SolverContext* cache);
	void writeBack1D(const PxSolverConstraintDesc& desc);

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
				writeBack1D(static_cast<PxSolverConstraintDesc&>(desc));
			}
			else
			{
				writeBack1D(desc, context, data, data);
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

	void concludeContact(const PxSolverConstraintDesc& desc, SolverContext& cache);

	void conclude1D(const PxSolverConstraintDesc& desc, SolverContext& cache);

	void concludeContact(const PxSolverConstraintDesc& desc);

	void conclude1DStep(const PxSolverConstraintDesc& desc);


	void FeatherstoneArticulation::concludeInternalConstraints(bool isTGS)
	{
		SolverContext context;

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
				conclude1D(desc, context);
			}
		}

		for (PxU32 i = 0; i < mStaticContactConstraints.size(); ++i)
		{
			PxSolverConstraintDesc& desc = mStaticContactConstraints[i];
			PX_ASSERT(*desc.constraint == DY_SC_TYPE_EXT_CONTACT);

			if (isTGS)
			{
				concludeContact(desc);
			}
			else
			{
				concludeContact(desc, context);
			}
		}
	}

	//Takes jointV, returns deltaF
	static PxReal solveLimit(ArticulationInternalLimit& limit, PxReal& jointV, const PxReal jointPDelta,
		const PxReal response, const PxReal recipResponse, const InternalConstraintSolverData& data)
	{
		PxReal futureDeltaJointP = jointPDelta + jointV * data.dt;

		bool limited = false;

		const PxReal tolerance = 0.f;

		PxReal deltaF = 0.f;
		if ((limit.errorLow + jointPDelta) < tolerance || (limit.errorLow + futureDeltaJointP) < tolerance)
		{
			PxReal newJointV = jointV;
			limited = true;
			if ((limit.errorLow + jointPDelta) < tolerance)
			{
				if (!data.velocityIteration)
					newJointV = -(limit.errorLow + jointPDelta) * data.invDt*data.erp;
			}
			else
				newJointV = -(limit.errorLow + jointPDelta) * data.invDt;

			PxReal deltaV = newJointV - jointV;
			const PxReal lowImpulse = limit.lowImpulse;
			deltaF = PxMax(lowImpulse + deltaV * recipResponse, 0.f) - lowImpulse;
			
			limit.lowImpulse = lowImpulse + deltaF;
		}
		else if ((limit.errorHigh - jointPDelta) < tolerance || (limit.errorHigh - futureDeltaJointP) < tolerance)
		{
			PxReal newJointV = jointV;
			limited = true;
			if ((limit.errorHigh - jointPDelta) < tolerance)
			{
				if (!data.velocityIteration)
					newJointV = (limit.errorHigh - jointPDelta) * data.invDt*data.erp;
			}
			else
				newJointV = (limit.errorHigh - jointPDelta) * data.invDt;

			PxReal deltaV = newJointV - jointV;
			const PxReal highImpulse = limit.highImpulse;
			deltaF = PxMin(highImpulse + deltaV * recipResponse, 0.f) - highImpulse;
			limit.highImpulse = highImpulse + deltaF;
		}


		if (!limited)
		{
			const PxReal forceLimit = -jointV*recipResponse;
			if (jointV > 0.f)
			{
				deltaF = PxMax(forceLimit, -limit.lowImpulse);
				limit.lowImpulse += deltaF;
				
			}
			else
			{
				deltaF = PxMin(forceLimit, -limit.highImpulse);
				limit.highImpulse += deltaF;
			}
		}

		jointV += deltaF * response;
		return deltaF;
	}


	Cm::SpatialVectorF FeatherstoneArticulation::solveInternalJointConstraintRecursive(InternalConstraintSolverData& data, const PxU32 linkID, const Cm::SpatialVectorF& parentDeltaV)
	{
		//PxU32 linkID = stack[stackSize];
		const ArticulationLink* links = mArticulationData.mLinks;
		const ArticulationLink& link = links[linkID];
		//const ArticulationLink& plink = links[link.parent];
		ArticulationLinkData& linkDatum = mArticulationData.getLinkData(linkID);

		//PxTransform* transforms = mArticulationData.mPreTransform.begin();

		PX_UNUSED(linkDatum);

		const ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);
	//	const ArticulationJointTargetData& jointTranDatum = mArticulationData.getJointTranData(linkID);

		Cm::SpatialVectorF i1(PxVec3(0.f), PxVec3(0.f));

		//We know the absolute parentDeltaV from the call to this function so no need to modify it. 
		Cm::SpatialVectorF parentV = parentDeltaV + mArticulationData.mMotionVelocities[link.parent];

		Cm::SpatialVectorF parentVelContrib = propagateAccelerationW(mArticulationData.getRw(linkID), mArticulationData.mInvStIs[linkID],
			&mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset], parentDeltaV, jointDatum.dof,
			&mArticulationData.mIsW[jointDatum.jointOffset], &mArticulationData.mDeferredQstZ[jointDatum.jointOffset]);

		Cm::SpatialVectorF childV = mArticulationData.mMotionVelocities[linkID] + parentVelContrib;

		Cm::UnAlignedSpatialVector i0(PxVec3(0.f), PxVec3(0.f));

		Cm::SpatialVectorF dv1 = parentVelContrib;

		const PxReal maxJointVel = link.inboundJoint->maxJointVelocity;

		//If we have any internal constraints to process (parent/child limits/locks/drives)
		if (jointDatum.dofInternalConstraintMask)
		{
			for (PxU32 dof = 0; dof < jointDatum.dof; ++dof)
			{
				PxReal deltaF = 0.f;
				PxReal clampedForce = 0.f;

				PxU32 internalConstraint = jointDatum.dofInternalConstraintMask & (1 << dof);

				if (internalConstraint)
				{
					ArticulationInternalConstraint& constraint = mArticulationData.mInternalConstraints[data.dofId++];
					const PxReal jointPDelta = constraint.row1.innerProduct(mArticulationData.mDeltaMotionVector[linkID]) - constraint.row0.innerProduct(mArticulationData.mDeltaMotionVector[link.parent]);

					//PxReal driveError = constraint.driveError - jointPDelta;

					PxReal errorDelta = (constraint.driveTargetVel*constraint.response*data.elapsedTime) - jointPDelta;

					PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);

					const PxReal appliedFriction = constraint.frictionForce*constraint.frictionForceCoefficient;

					PxReal frictionForce = PxClamp(-jointV *constraint.recipResponse + appliedFriction,
						-constraint.maxFrictionForce, constraint.maxFrictionForce);

					PxReal frictionDeltaF = frictionForce - appliedFriction;

					constraint.frictionForce += frictionDeltaF;

					jointV += frictionDeltaF * constraint.response;



					PxReal unclampedForce = constraint.driveImpulseMultiplier * constraint.driveForce +
						jointV * constraint.driveVelMultiplier + constraint.driveTargetVel + constraint.driveInitialBias + errorDelta * constraint.driveBiasCoefficient;

					clampedForce = PxClamp(unclampedForce, -constraint.maxDriveForce, constraint.maxDriveForce);
					PxReal driveDeltaF = (clampedForce - constraint.driveForce);

					//Where we will be next frame - we use this to compute error bias terms to correct limits and drives...

					jointV += driveDeltaF * constraint.response;

					driveDeltaF += frictionDeltaF;

					//printf("LinkID %i driveDeltaV = %f, jointV = %f\n", linkID, driveDeltaF, jointV);

					if (jointDatum.limitMask & (1 << dof))
					{
						ArticulationInternalLimit& limit = mArticulationData.mInternalLimits[data.limitId++];
						deltaF = solveLimit(limit, jointV, jointPDelta, constraint.response, constraint.recipResponse, data);
					}

					if (PxAbs(jointV) > maxJointVel)
					{
						PxReal newJointV = PxClamp(jointV, -maxJointVel, maxJointVel);
						deltaF += (newJointV - jointV) * constraint.recipResponse*data.erp;
						jointV = newJointV;
					}

					deltaF += driveDeltaF;

					if (deltaF != 0.f)
					{
						//impulse = true;
						constraint.driveForce = clampedForce;

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
				}
			}
		}



		

		const Cm::SpatialVectorF& deltaMotion = mArticulationData.getDeltaMotionVector(linkID);
		const PxQuat& deltaQ = getDeltaQ(linkID);

		const PxU32 nbStatic1DConstraints = mArticulationData.mNbStatic1DConstraints[linkID];
		PxU32 start1DIdx = mArticulationData.mStatic1DConstraintStartIndex[linkID];
		for (PxU32 i = 0; i < nbStatic1DConstraints; ++i)
		{
			PxSolverConstraintDesc& desc = mStatic1DConstraints[start1DIdx++];
			solveStaticConstraint(desc, childV, i1, dv1, deltaMotion, deltaQ, data.isTGS, data.elapsedTime, data.velocityIteration ? 0.f : -PX_MAX_F32);
		}

		const PxU32 nbStaticContactConstraints = mArticulationData.mNbStaticContactConstraints[linkID];
		PxU32 startContactIdx = mArticulationData.mStaticContactConstraintStartIndex[linkID];
		for (PxU32 i = 0; i < nbStaticContactConstraints; ++i)
		{
			PxSolverConstraintDesc& desc = mStaticContactConstraints[startContactIdx++];
			solveStaticConstraint(desc, childV, i1, dv1, deltaMotion, deltaQ, data.isTGS, data.elapsedTime, data.velocityIteration ? 0.f : -PX_MAX_F32);
		}

		PxU32 numChildren = link.mNumChildren;
		PxU32 offset = link.mChildrenStartIndex;

		for(PxU32 i = 0; i < numChildren; ++i)
		{
			const PxU32 child = offset+i;

			Cm::SpatialVectorF childImp = solveInternalJointConstraintRecursive(data, child, dv1);
			i1 += childImp;

			if ((numChildren-i) > 1)
			{
				//Propagate the childImp to my dv1 so that the next constraint gets to see an updated velocity state based
				//on the propagation of the child velocities
				Cm::SpatialVectorF deltaV = mArticulationData.mResponseMatrixW[linkID].getResponse(-childImp);
				dv1 += deltaV;
				childV += deltaV;

			}
		} 

			
		Cm::SpatialVectorF propagatedImpulse = propagateImpulseW(&mArticulationData.mIsInvDW[jointDatum.jointOffset], mArticulationData.getRw(linkID), &mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset], i1, jointDatum.dof, &mArticulationData.mDeferredQstZ[jointDatum.jointOffset]);
		return Cm::SpatialVectorF(i0.top, i0.bottom) + propagatedImpulse;
	}


	void FeatherstoneArticulation::solveInternalJointConstraints(const PxReal dt, const PxReal invDt,
		Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV, bool velocityIteration, bool isTGS,
		const PxReal elapsedTime, const PxReal biasCoefficient)
	{
		//const PxU32 count = mArticulationData.getLinkCount();

		if (mArticulationData.mInternalConstraints.size() == 0 
			&& mStatic1DConstraints.size() == 0 && mStaticContactConstraints.size() == 0)
			return;

		//const PxReal erp = isTGS ? 0.5f*biasCoefficient : biasCoefficient;
		const PxReal erp = biasCoefficient;

		const bool fixBase = mArticulationData.getArticulationFlags() & PxArticulationFlag::eFIX_BASE;

		PxU32* static1DConstraintCounts = mArticulationData.mNbStatic1DConstraints.begin();
		PxU32* static1DConstraintStarts = mArticulationData.mStatic1DConstraintStartIndex.begin();

		PxU32* staticContactConstraintCounts = mArticulationData.mNbStaticContactConstraints.begin();
		PxU32* staticContactConstraintStarts = mArticulationData.mStaticContactConstraintStartIndex.begin();
		
		ArticulationLink* links = mArticulationData.getLinks();
		Cm::SpatialVectorF* baseVelocities = mArticulationData.getMotionVelocities();
		//PxTransform* transforms = mArticulationData.mPreTransform.begin();		

		//Cm::SpatialVectorF* deferredZ = mArticulationData.getSpatialZAVectors();

		const PxReal minPenetration = velocityIteration ? 0.f : -PX_MAX_F32;

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

				if (nbStatic1DConstraints)
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

				if (nbStaticContactConstraints)
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
			
			InternalConstraintSolverData data(dt, invDt, elapsedTime,
				erp, impulses, DeltaV,
				velocityIteration, isTGS);

			data.articId = mArticulationIndex;

			const PxU32 numChildren = links[0].mNumChildren;
			const PxU32 offset = links[0].mChildrenStartIndex;

			for (PxU32 i = 0; i < numChildren; ++i)
			{
				const PxU32 child = offset + i;

				Cm::SpatialVectorF imp = solveInternalJointConstraintRecursive(data, child, rootLinkDeltaV);

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
					const PxU32 child = ArticulationLowestSetBit(children);

					updateSpatialTendonConstraintsRecursive(attachments, mArticulationData, child, tendon->mOffset*coefficient, pAttachPoint);
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

			PxReal error2 = 0.f;
			if (constraint.accumulatedLength > constraint.highLimit)
				error2 = constraint.highLimit - constraint.accumulatedLength;
			if (constraint.accumulatedLength < constraint.lowLimit)
				error2 = constraint.lowLimit - constraint.accumulatedLength;

			PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);

			PX_ASSERT(PxIsFinite(jointV));

			PxReal unclampedForce = (jointV * constraint.velMultiplier + error * constraint.biasCoefficient) /** constraint.recipResponse*/
				+ constraint.appliedForce * constraint.impulseMultiplier;

			PxReal unclampedForce2 = (error2 * constraint.limitBiasCoefficient) + constraint.limitAppliedForce * constraint.limitImpulseMultiplier;

			const PxReal deltaF = (unclampedForce - constraint.appliedForce) + (unclampedForce2 - constraint.limitAppliedForce);

			constraint.appliedForce = unclampedForce;
			constraint.limitAppliedForce = unclampedForce2;

			if (deltaF != 0.f)
			{
				Cm::UnAlignedSpatialVector i0 = constraint.row0 * -deltaF;
				Cm::UnAlignedSpatialVector i1 = constraint.row1 * deltaF;
				pxcFsApplyImpulses(parentID, V3LoadU(i0.top), V3LoadU(i0.bottom),
					linkID, V3LoadU(i1.top), V3LoadU(i1.bottom), NULL, NULL);
			}
		}
	}

	PxVec3 FeatherstoneArticulation::calculateFixedTendonVelocityAndPositionRecursive(FixedTendonSolveData& solveData,
		const Cm::SpatialVectorF& parentV, const Cm::SpatialVectorF& parentDeltaV, const PxU32 tendonJointID)
	{
		ArticulationTendonJoint& tendonJoint = solveData.tendonJoints[tendonJointID];

		ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(tendonJoint.linkInd);

		Cm::SpatialVectorF deltaV = propagateAccelerationW(mArticulationData.getRw(tendonJoint.linkInd), mArticulationData.mInvStIs[tendonJoint.linkInd],
			&mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset], parentDeltaV, jointDatum.dof,
			&mArticulationData.mIsW[jointDatum.jointOffset], &mArticulationData.mDeferredQstZ[jointDatum.jointOffset]);

		Cm::SpatialVectorF childV = mArticulationData.mMotionVelocities[tendonJoint.linkInd] + deltaV;


		PxU32 index = tendonJoint.mConstraintInd;
		ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalFixedTendonConstraints[index];

		const PxU32 childCount = tendonJoint.childCount;

		PxVec3 jointVError;

		const PxReal jointV = constraint.row1.innerProduct(childV) - constraint.row0.innerProduct(parentV);
		const PxReal jointP = mArticulationData.mJointPosition[tendonJoint.startJointOffset];

		jointVError.x = jointV * tendonJoint.coefficient;

		jointVError.y = jointP * tendonJoint.coefficient;

		//printf("%i: jointPose = %f, jointV = %f, coefficient = %f\n", tendonJoint.linkInd, jointP, jointV, tendonJoint.coefficient);

		jointVError.z = 1.f;

		if (childCount)
		{
			for (ArticulationBitField children = tendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = ArticulationLowestSetBit(children);
				jointVError += calculateFixedTendonVelocityAndPositionRecursive(solveData, childV, deltaV, child);
			}
		}

		return jointVError;
	}


	Cm::SpatialVectorF FeatherstoneArticulation::solveFixedTendonConstraintsRecursive(FixedTendonSolveData& solveData, 
		const PxU32 tendonJointID)
	{
		ArticulationTendonJoint& tendonJoint = solveData.tendonJoints[tendonJointID];

		ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(tendonJoint.linkInd);
			   
		PxU32 index = tendonJoint.mConstraintInd;
		ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalFixedTendonConstraints[index];

		const PxU32 childCount = tendonJoint.childCount;

		PxReal jointV = solveData.rootVel;

		// calculate current accumulated tendon length from parent accumulated length

		const PxReal lengthError = solveData.error;
		PxReal limitError = solveData.limitError;

		// the constraint bias coefficients need to flip signs together with the tendon joint's coefficient
		// in order for the constraint force to point into the correct direction:
		const PxReal coefficientSign = tendonJoint.recipCoefficient;// PxSign(tendonJoint.coefficient);
		const PxReal biasCoefficient = constraint.biasCoefficient;
		const PxReal limitBiasCoefficient = constraint.limitBiasCoefficient;
		
		PxReal unclampedForce = ((jointV * constraint.velMultiplier + lengthError * biasCoefficient)*coefficientSign)
			+ constraint.appliedForce * constraint.impulseMultiplier;

		PxReal unclampedForce2 = (limitError * limitBiasCoefficient * coefficientSign)
			+ constraint.limitAppliedForce * constraint.limitImpulseMultiplier;

		const PxReal deltaF = ((unclampedForce - constraint.appliedForce) + (unclampedForce2 - constraint.limitAppliedForce));

		constraint.appliedForce = unclampedForce;
		constraint.limitAppliedForce = unclampedForce2;

		solveData.rootImp += deltaF;

		
		Cm::SpatialVectorF impulse(constraint.row1.top * -deltaF, constraint.row1.bottom * -deltaF);
		
		if (childCount)
		{
			for (ArticulationBitField children = tendonJoint.children; children != 0; children &= (children - 1))
			{
				//index of child of link h on path to link linkID
				const PxU32 child = ArticulationLowestSetBit(children);

				Cm::SpatialVectorF propagatedImpulse = solveFixedTendonConstraintsRecursive(solveData, child);


				impulse.top += propagatedImpulse.top;
				impulse.bottom += propagatedImpulse.bottom;
			}
		}

		return propagateImpulseW(&mArticulationData.mIsInvDW[jointDatum.jointOffset], mArticulationData.mRw[tendonJoint.linkInd], 
			&mArticulationData.mWorldMotionMatrix[jointDatum.jointOffset], impulse, jointDatum.dof, &mArticulationData.mDeferredQstZ[jointDatum.jointOffset]);
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

				Cm::SpatialVectorF parentDeltaV = parentV - mArticulationData.mMotionVelocities[startLink];


				PxVec3 velError(0.f);
							   
				for (ArticulationAttachmentBitField children = pTendonJoint.children; children != 0; children &= (children - 1))
				{
					//index of child of link h on path to link linkID
					const PxU32 child = ArticulationLowestSetBit(children);

					FixedTendonSolveData solveData;
					solveData.links = links;
					solveData.erp = 1.f;
					solveData.rootImp = 0.f;
					solveData.error = tendon->mError;
					solveData.tendonJoints = tendonJoints;


					velError += calculateFixedTendonVelocityAndPositionRecursive(solveData, parentV, parentDeltaV, child);
				}
				
				const PxReal recipScale = velError.z == 0.f ? 0.f : 1.f / velError.z;

				for (ArticulationAttachmentBitField children = pTendonJoint.children; children != 0; children &= (children - 1))
				{
					//index of child of link h on path to link linkID
					const PxU32 child = ArticulationLowestSetBit(children);
					ArticulationTendonJoint& tendonJoint = tendonJoints[child];

					ArticulationInternalTendonConstraint& constraint = mArticulationData.mInternalFixedTendonConstraints[tendonJoint.mConstraintInd];

					const PxReal length = (velError.y + tendon->mOffset);

					FixedTendonSolveData solveData;
					solveData.links = links;
					solveData.erp = 1.f;
					solveData.rootImp = 0.f;
					solveData.error = (length - tendon->mRestLength) * recipScale;
					solveData.rootVel = velError.x*recipScale;

					PxReal limitError = 0.f;
					if (length < tendon->mLowLimit)
						limitError = length - tendon->mLowLimit;
					else if (length > tendon->mHighLimit)
						limitError = length - tendon->mHighLimit;
					solveData.limitError = limitError * recipScale;
					solveData.tendonJoints = tendonJoints;

					//KS - TODO - hook up offsets
					Cm::SpatialVectorF propagatedImpulse = solveFixedTendonConstraintsRecursive(solveData, child);

					propagatedImpulse.top += constraint.row0.top * solveData.rootImp;
					propagatedImpulse.bottom += constraint.row0.bottom * solveData.rootImp;

					Z += propagatedImpulse;
				}
				
				for (PxU32 linkID = pTendonJoint.linkInd; linkID; linkID = links[linkID].parent)
				{
					const PxU32 jointOffset = mArticulationData.getJointData(linkID).jointOffset;
					const PxU32 dofCount = mArticulationData.getJointData(linkID).dof;

					Z = propagateImpulseW(&mArticulationData.mIsInvDW[jointOffset], mArticulationData.getRw(linkID),
						&mArticulationData.mWorldMotionMatrix[jointOffset], Z, dofCount, &mArticulationData.mDeferredQstZ[jointOffset]);
				}

				mArticulationData.mRootDeferredZ += Z;
				mArticulationData.mJointDirty = true;

			}
		}

	}


	void FeatherstoneArticulation::solveInternalConstraints(const PxReal dt, const PxReal invDt,
		Cm::SpatialVectorF* impulses, Cm::SpatialVectorF* DeltaV, bool velocityIteration, bool isTGS,
		const PxReal elapsedTime, const PxReal biasCoefficient)
	{
		solveInternalSpatialTendonConstraints(isTGS);
		solveInternalFixedTendonConstraints(isTGS);
		solveInternalJointConstraints(dt, invDt, impulses, DeltaV, velocityIteration, isTGS, elapsedTime, biasCoefficient);
	}

	bool FeatherstoneArticulation::storeStaticConstraint(const PxSolverConstraintDesc& desc)
	{
		if (DY_STATIC_CONTACTS_IN_INTERNAL_SOLVER)
		{
			if (desc.constraintLengthOver16 == DY_SC_TYPE_RB_CONTACT)
				mStaticContactConstraints.pushBack(desc);
			else
				mStatic1DConstraints.pushBack(desc);
		}
		return DY_STATIC_CONTACTS_IN_INTERNAL_SOLVER;
	}

	void FeatherstoneArticulation::setRootLinearVelocity(const PxVec3& velocity)
	{
		ArticulationLink& rLink = mArticulationData.getLink(0);
		rLink.bodyCore->linearVelocity = velocity;
		mGPUDirtyFlags |= ArticulationDirtyFlag::eDIRTY_ROOT_VELOCITIES;
		computeLinkVelocities(mArticulationData);
	}

	void FeatherstoneArticulation::setRootAngularVelocity(const PxVec3& velocity)
	{
		ArticulationLink& rLink = mArticulationData.getLink(0);
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
		ArticulationLink* links = mArticulationData.getLinks();
		PxReal* jointPositions = mArticulationData.getJointPositions();
		Cm::SpatialVectorF* motionVelocities = mArticulationData.getMotionVelocities();

		for (PxU32 linkID = 1; linkID < linkCount; ++linkID)
		{
			ArticulationLink& link = links[linkID];
			const PxTransform oldTransform = link.bodyCore->body2World;

			ArticulationLink& pLink = links[link.parent];
			const PxTransform pBody2World = pLink.bodyCore->body2World;

			ArticulationJointCore* joint = link.inboundJoint;
			ArticulationJointCoreData& jointDatum = mArticulationData.getJointData(linkID);

			PxReal* jPosition = &jointPositions[jointDatum.jointOffset];

			PxQuat newParentToChild;
			PxQuat newWorldQ;
			PxVec3 r;

			const PxVec3 childOffset = -joint->childPose.p;
			const PxVec3 parentOffset = joint->parentPose.p;

			const PxQuat relativeQuat = mArticulationData.mRelativeQuat[linkID];

			switch (joint->jointType)
			{
			case PxArticulationJointType::ePRISMATIC:
			{
				newParentToChild = relativeQuat;
				const PxVec3 e = newParentToChild.rotate(parentOffset);
				const PxVec3 d = childOffset;

				const PxVec3& u = mArticulationData.mMotionMatrix[jointDatum.jointOffset].bottom;

				r = e + d + u * jPosition[0];
				break;
			}
			case PxArticulationJointType::eREVOLUTE:
			case PxArticulationJointType::eREVOLUTE_UNWRAPPED:
			{
				const PxVec3& u = mArticulationData.mMotionMatrix[jointDatum.jointOffset].top;

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
