// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CCT_CHARACTER_CONTROLLER_MANAGER
#define CCT_CHARACTER_CONTROLLER_MANAGER

//Exclude file from docs
/** \cond */

#include "geometry/PxMeshQuery.h"
#include "common/PxRenderBuffer.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "characterkinematic/PxControllerManager.h"
#include "characterkinematic/PxControllerObstacles.h"
#include "PxDeletionListener.h"
#include "CctUtils.h"
#include "foundation/PxMutex.h"
#include "foundation/PxArray.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
namespace Cct
{
	class Controller;
	class ObstacleContext;

	struct ObservedRefCounter
	{
		ObservedRefCounter(): refCount(0)
		{
		}

		PxU32 refCount;
	};

	typedef PxHashMap<const PxBase*, ObservedRefCounter>	ObservedRefCountMap;

	//Implements the PxControllerManager interface, this class used to be called ControllerManager
	class CharacterControllerManager : public PxControllerManager, public PxUserAllocated, public PxDeletionListener
	{
														PX_NOCOPY(CharacterControllerManager)
	public:
														CharacterControllerManager(PxScene& scene, bool lockingEnabled = false);
	private:
		virtual											~CharacterControllerManager();
	public:

		// PxControllerManager
		virtual			void							release()	PX_OVERRIDE	PX_FINAL;
		virtual			PxScene&						getScene() const	PX_OVERRIDE	PX_FINAL;
		virtual			PxU32							getNbControllers()	const	PX_OVERRIDE	PX_FINAL;
		virtual			PxController*					getController(PxU32 index)	PX_OVERRIDE	PX_FINAL;
        virtual			PxController*					createController(const PxControllerDesc& desc)	PX_OVERRIDE	PX_FINAL;
       
		virtual			void							purgeControllers()	PX_OVERRIDE	PX_FINAL;
		virtual			PxRenderBuffer&					getRenderBuffer()	PX_OVERRIDE	PX_FINAL;
		virtual			void							setDebugRenderingFlags(PxControllerDebugRenderFlags flags)	PX_OVERRIDE	PX_FINAL;
		virtual			PxU32							getNbObstacleContexts() const	PX_OVERRIDE	PX_FINAL;
		virtual			PxObstacleContext*				getObstacleContext(PxU32 index)	PX_OVERRIDE	PX_FINAL;
		virtual			PxObstacleContext*				createObstacleContext()	PX_OVERRIDE	PX_FINAL;
		virtual			void							computeInteractions(PxF32 elapsedTime, PxControllerFilterCallback* cctFilterCb)	PX_OVERRIDE	PX_FINAL;
		virtual			void							setTessellation(bool flag, float maxEdgeLength)	PX_OVERRIDE	PX_FINAL;
		virtual			void							setOverlapRecoveryModule(bool flag)	PX_OVERRIDE	PX_FINAL;
		virtual			void							setPreciseSweeps(bool flag)	PX_OVERRIDE	PX_FINAL;
		virtual			void							setPreventVerticalSlidingAgainstCeiling(bool flag)	PX_OVERRIDE	PX_FINAL;
		virtual			void							shiftOrigin(const PxVec3& shift)	PX_OVERRIDE	PX_FINAL;
		//~PxControllerManager

		// PxDeletionListener
		virtual		void								onRelease(const PxBase* observed, void* userData, PxDeletionEventFlag::Enum deletionEvent)	PX_OVERRIDE	PX_FINAL;
		//~PxDeletionListener
						void							registerObservedObject(const PxBase* obj);
						void							unregisterObservedObject(const PxBase* obj);

		// ObstacleContextNotifications
						void							onObstacleRemoved(PxObstacleHandle index) const;
						void							onObstacleUpdated(PxObstacleHandle index, const PxObstacleContext* ) const;
						void							onObstacleAdded(PxObstacleHandle index, const PxObstacleContext*) const;

						void							releaseController(PxController& controller);
						Controller**					getControllers();
						void							releaseObstacleContext(ObstacleContext& oc);
						void							resetObstaclesBuffers();

						PxScene&						mScene;

						PxRenderBuffer*					mRenderBuffer;
						PxControllerDebugRenderFlags	mDebugRenderingFlags;
		// Shared buffers for obstacles
						PxArray<const void*>			mBoxUserData;
						PxArray<PxExtendedBox>			mBoxes;

						PxArray<const void*>			mCapsuleUserData;
						PxArray<PxExtendedCapsule>		mCapsules;

						PxArray<Controller*>			mControllers;
						PxHashSet<PxShape*>				mCCTShapes;

						PxArray<ObstacleContext*>		mObstacleContexts;

						float							mMaxEdgeLength;
						bool							mTessellation;

						bool							mOverlapRecovery;
						bool							mPreciseSweeps;
						bool							mPreventVerticalSlidingAgainstCeiling;

						bool							mLockingEnabled;						
	private:
						ObservedRefCountMap				mObservedRefCountMap;
						mutable	PxMutex					mWriteLock;			// Lock used for guarding pointers in observedrefcountmap
	};

} // namespace Cct

}

/** \endcond */
#endif //CCT_CHARACTER_CONTROLLER_MANAGER
