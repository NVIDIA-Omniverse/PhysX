// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CCT_CONTROLLER
#define CCT_CONTROLLER

/* Exclude from documentation */
/** \cond */

#include "CctCharacterController.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxMutex.h"

namespace physx
{

class PxPhysics;
class PxScene;
class PxRigidDynamic;
class PxGeometry;
class PxMaterial;

namespace Cct
{
	class CharacterControllerManager;

	class Controller : public PxUserAllocated
	{
		PX_NOCOPY(Controller)
	public:
													Controller(const PxControllerDesc& desc, PxScene* scene);
		virtual										~Controller();

					void							releaseInternal();
					void							getInternalState(PxControllerState& state)	const;
					void							getInternalStats(PxControllerStats& stats)	const;					

		virtual		PxF32							getHalfHeightInternal()				const	= 0;
		virtual		bool							getWorldBox(PxExtendedBounds3& box)	const	= 0;
		virtual		PxController*					getPxController()							= 0;

					void							onOriginShift(const PxVec3& shift);

					void							onRelease(const PxBase& observed);

					void							setCctManager(CharacterControllerManager* cm)
													{
														mManager = cm;
														mCctModule.setCctManager(cm);
													}

	PX_FORCE_INLINE	CharacterControllerManager*		getCctManager()				{ return mManager;		}
	PX_FORCE_INLINE	PxU64							getContextId()		const	{ return PxU64(mScene);	}

					PxControllerShapeType::Enum		mType;
		// User params
					CCTParams						mUserParams;
					PxUserControllerHitReport*		mReportCallback;
					PxControllerBehaviorCallback*	mBehaviorCallback;
					void*							mUserData;
		// Internal data
					SweepTest						mCctModule;			// Internal CCT object. Optim test for Ubi.
					PxRigidDynamic*					mKineActor;			// Associated kinematic actor
					PxExtendedVec3					mPosition;			// Current position
					PxVec3							mDeltaXP;
					PxVec3							mOverlapRecover;
					PxScene*						mScene;				// Handy scene owner
					PxU32							mPreviousSceneTimestamp;					
					PxF64							mGlobalTime;
					PxF64							mPreviousGlobalTime;
					PxF32							mProxyDensity;		// Density for proxy actor
					PxF32							mProxyScaleCoeff;	// Scale coeff for proxy actor
					PxControllerCollisionFlags		mCollisionFlags;	// Last known collision flags (PxControllerCollisionFlag)
					bool							mCachedStandingOnMoving;
					bool							mRegisterDeletionListener;
		mutable		PxMutex							mWriteLock;			// Lock used for guarding touched pointers and cache data from overwriting 
																			// during onRelease call.
	protected:
		// Internal methods
					void							setUpDirectionInternal(const PxVec3& up);
					PxShape*						getKineShape()	const;
					bool							createProxyActor(PxPhysics& sdk, const PxGeometry& geometry, const PxMaterial& material, PxClientID clientID);
					bool							setPos(const PxExtendedVec3& pos);
					void							findTouchedObject(const PxControllerFilters& filters, const PxObstacleContext* obstacleContext, const PxVec3& upDirection);
					bool							rideOnTouchedObject(SweptVolume& volume, const PxVec3& upDirection, PxVec3& disp, const PxObstacleContext* obstacleContext);
					PxControllerCollisionFlags		move(SweptVolume& volume, const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles, bool constrainedClimbingMode);
					bool							filterTouchedShape(const PxControllerFilters& filters);

	PX_FORCE_INLINE	float							computeTimeCoeff()
													{
														const float elapsedTime = float(mGlobalTime - mPreviousGlobalTime);
														mPreviousGlobalTime = mGlobalTime;
														return 1.0f / elapsedTime;
													}

					CharacterControllerManager*		mManager;			// Owner manager
	};

} // namespace Cct

}

/** \endcond */
#endif
