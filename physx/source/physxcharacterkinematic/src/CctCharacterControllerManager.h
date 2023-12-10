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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

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
	public:
														CharacterControllerManager(PxScene& scene, bool lockingEnabled = false);
	private:
		virtual											~CharacterControllerManager();
	public:

		// PxControllerManager
		virtual			void							release()	PX_OVERRIDE;
		virtual			PxScene&						getScene() const	PX_OVERRIDE;
		virtual			PxU32							getNbControllers()	const	PX_OVERRIDE;
		virtual			PxController*					getController(PxU32 index)	PX_OVERRIDE;
        virtual			PxController*					createController(const PxControllerDesc& desc)	PX_OVERRIDE;
       
		virtual			void							purgeControllers()	PX_OVERRIDE;
		virtual			PxRenderBuffer&					getRenderBuffer()	PX_OVERRIDE;
		virtual			void							setDebugRenderingFlags(PxControllerDebugRenderFlags flags)	PX_OVERRIDE;
		virtual			PxU32							getNbObstacleContexts() const	PX_OVERRIDE;
		virtual			PxObstacleContext*				getObstacleContext(PxU32 index)	PX_OVERRIDE;
		virtual			PxObstacleContext*				createObstacleContext()	PX_OVERRIDE;
		virtual			void							computeInteractions(PxF32 elapsedTime, PxControllerFilterCallback* cctFilterCb)	PX_OVERRIDE;
		virtual			void							setTessellation(bool flag, float maxEdgeLength)	PX_OVERRIDE;
		virtual			void							setOverlapRecoveryModule(bool flag)	PX_OVERRIDE;
		virtual			void							setPreciseSweeps(bool flag)	PX_OVERRIDE;
		virtual			void							setPreventVerticalSlidingAgainstCeiling(bool flag)	PX_OVERRIDE;
		virtual			void							shiftOrigin(const PxVec3& shift)	PX_OVERRIDE;
		//~PxControllerManager

		// PxDeletionListener
		virtual		void								onRelease(const PxBase* observed, void* userData, PxDeletionEventFlag::Enum deletionEvent)	PX_OVERRIDE;
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

	protected:
		CharacterControllerManager &operator=(const CharacterControllerManager &);
		CharacterControllerManager(const CharacterControllerManager& );

	private:
						ObservedRefCountMap				mObservedRefCountMap;
						mutable	PxMutex					mWriteLock;			// Lock used for guarding pointers in observedrefcountmap
	};

} // namespace Cct

}

/** \endcond */
#endif //CCT_CHARACTER_CONTROLLER_MANAGER
