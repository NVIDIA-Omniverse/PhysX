// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "foundation/PxMathUtils.h"
#include "extensions/PxRigidBodyExt.h"
#include "characterkinematic/PxController.h"
#include "PxPhysics.h"
#include "PxScene.h"
#include "PxRigidDynamic.h"
#include "PxShape.h"

#include "CctController.h"
#include "CctBoxController.h"
#include "CctCharacterControllerManager.h"
#include "foundation/PxUtilities.h"

using namespace physx;
using namespace Cct;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Controller::Controller(const PxControllerDesc& desc, PxScene* s) :
	mCctModule				(desc.registerDeletionListener),
	mScene					(s),
	mPreviousSceneTimestamp	(0xffffffff),		
	mGlobalTime				(0.0),
	mPreviousGlobalTime		(0.0),
	mProxyDensity			(0.0f),
	mProxyScaleCoeff		(0.0f),
	mCollisionFlags			(0),
	mCachedStandingOnMoving	(false),
	mManager				(NULL)
{
	mType								= PxControllerShapeType::eFORCE_DWORD;

	mUserParams.mNonWalkableMode		= desc.nonWalkableMode;
	mUserParams.mSlopeLimit				= desc.slopeLimit;
	mUserParams.mContactOffset			= desc.contactOffset;
	mUserParams.mStepOffset				= desc.stepOffset;
	mUserParams.mInvisibleWallHeight	= desc.invisibleWallHeight;
	mUserParams.mMaxJumpHeight			= desc.maxJumpHeight;
	mUserParams.mHandleSlope			= desc.slopeLimit!=0.0f;

	mReportCallback						= desc.reportCallback;
	mBehaviorCallback					= desc.behaviorCallback;
	mUserData							= desc.userData;

	mKineActor							= NULL;
	mPosition							= desc.position;
	mProxyDensity						= desc.density;
	mProxyScaleCoeff					= desc.scaleCoeff;

	mCctModule.mVolumeGrowth			= desc.volumeGrowth;	

	mRegisterDeletionListener			= desc.registerDeletionListener;

	mDeltaXP							= PxVec3(0);
	mOverlapRecover						= PxVec3(0);	

	mUserParams.mUpDirection = PxVec3(0.0f);
	setUpDirectionInternal(desc.upDirection);
}

Controller::~Controller()
{
	if(mScene)
	{
		if(mKineActor)
			mKineActor->release();
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::onRelease(const PxBase& observed)
{
	mCctModule.onRelease(observed);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::onOriginShift(const PxVec3& shift)
{
	sub(mPosition, shift);
	
	if(mManager && mManager->mLockingEnabled)
		mWriteLock.lock();

	mCctModule.onOriginShift(shift);	

	if(mManager && mManager->mLockingEnabled)
		mWriteLock.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::setUpDirectionInternal(const PxVec3& up)
{
	PX_CHECK_MSG(up.isNormalized(), "CCT: up direction must be normalized");

	if(mUserParams.mUpDirection==up)
		return;

	const PxQuat q = PxShortestRotation(PxVec3(1.0f, 0.0f, 0.0f), up);

	mUserParams.mQuatFromUp		= q;
	mUserParams.mUpDirection	= up;

	// Update kinematic actor
	/*if(mKineActor)
	{
		PxTransform pose = mKineActor->getGlobalPose();
		pose.q = q;
		mKineActor->setGlobalPose(pose);	
	}*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::releaseInternal()
{
	mManager->releaseController(*getPxController());
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::getInternalState(PxControllerState& state) const
{
	if(mManager->mLockingEnabled)
		mWriteLock.lock();

	state.deltaXP				= mDeltaXP;
	state.touchedShape			= const_cast<PxShape*>(mCctModule.mTouchedShape.get());
	state.touchedActor			= const_cast<PxRigidActor*>(mCctModule.mTouchedActor.get());
	state.touchedObstacleHandle	= mCctModule.mTouchedObstacleHandle;
	state.standOnAnotherCCT		= (mCctModule.mFlags & STF_TOUCH_OTHER_CCT)!=0;
	state.standOnObstacle		= (mCctModule.mFlags & STF_TOUCH_OBSTACLE)!=0;
	state.isMovingUp			= (mCctModule.mFlags & STF_IS_MOVING_UP)!=0;
	state.collisionFlags		= mCollisionFlags;

	if(mManager->mLockingEnabled)
		mWriteLock.unlock();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Controller::getInternalStats(PxControllerStats& stats) const
{
	stats.nbFullUpdates		= mCctModule.mNbFullUpdates;
	stats.nbPartialUpdates	= mCctModule.mNbPartialUpdates;
	stats.nbIterations		= mCctModule.mNbIterations;
	stats.nbTessellation	= mCctModule.mNbTessellation;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::setPos(const PxExtendedVec3& pos)
{
	mPosition = pos;

	// Update kinematic actor
	if(mKineActor)
	{
		PxTransform targetPose = mKineActor->getGlobalPose();
		targetPose.p = toVec3(mPosition);  // LOSS OF ACCURACY
		targetPose.q = mUserParams.mQuatFromUp;
		mKineActor->setKinematicTarget(targetPose);	
	}
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Controller::createProxyActor(PxPhysics& sdk, const PxGeometry& geometry, const PxMaterial& material, PxClientID clientID)
{
	// PT: we don't disable raycasting or CD because:
	// - raycasting is needed for visibility queries (the SDK otherwise doesn't know about the CCTS)
	// - collision is needed because the only reason we create actors there is to handle collisions with dynamic shapes
	// So it's actually wrong to disable any of those.

	PxTransform globalPose;
	globalPose.p = toVec3(mPosition);	// LOSS OF ACCURACY
	globalPose.q = mUserParams.mQuatFromUp;

	mKineActor = sdk.createRigidDynamic(globalPose);
	if(!mKineActor)
		return false;

	PxShape* shape = sdk.createShape(geometry, material, true);
	mKineActor->attachShape(*shape);
	shape->release();
	mKineActor->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);

	PxRigidBodyExt::updateMassAndInertia(*mKineActor, mProxyDensity);

	if(clientID!=PX_DEFAULT_CLIENT)
		mKineActor->setOwnerClient(clientID);

	mScene->addActor(*mKineActor);
	return true;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PxShape* Controller::getKineShape() const
{
	// PT: TODO: cache this and avoid the virtual call
	PxShape* shape = NULL;
	PxU32 nb = mKineActor->getShapes(&shape, 1);
	PX_ASSERT(nb==1);
	PX_UNUSED(nb);
	return shape;
}
