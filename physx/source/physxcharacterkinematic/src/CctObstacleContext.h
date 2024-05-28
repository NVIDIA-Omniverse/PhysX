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

#ifndef CCT_OBSTACLE_CONTEXT
#define CCT_OBSTACLE_CONTEXT

/* Exclude from documentation */
/** \cond */

#include "characterkinematic/PxControllerObstacles.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxArray.h"

namespace physx
{
	struct PxGeomRaycastHit;

namespace Cct
{
	class CharacterControllerManager;

    typedef PxU32  Handle;
	class HandleManager : public PxUserAllocated
	{
		public:
									HandleManager();
									~HandleManager();

						Handle		Add(void* object);
						void		Remove(Handle handle);
						void*		GetObject(Handle handle)	const;	// Returns object according to handle.
						bool		UpdateObject(Handle handle, void* object);

		PX_FORCE_INLINE	PxU32		GetMaxNbObjects()			const	{ return mMaxNbObjects;		}	//!< Returns max number of objects
		PX_FORCE_INLINE	PxU32		GetNbObjects()				const	{ return mCurrentNbObjects;	}	//!< Returns current number of objects
		PX_FORCE_INLINE	void**		GetObjects()				const	{ return mObjects;			}	//!< Gets the complete list of objects
		PX_FORCE_INLINE	void*		PickObject(Handle handle)	const	{ return mObjects[mOutToIn[PxU16(handle)]]; }

		private:
		// Physical list
						void**		mObjects;			//!< Physical list, with no holes but unsorted.
						PxU32		mCurrentNbObjects;	//!< Current number of objects in the physical list.
						PxU32		mMaxNbObjects;		//!< Maximum possible number of objects in the physical list.

		// Cross-references
						PxU16*		mOutToIn;			//!< Maps virtual indices (handles) to real ones.
						PxU16*		mInToOut;			//!< Maps real indices to virtual ones (handles).
						PxU16*      mStamps;

		// Recycled locations
						PxU32		mNbFreeIndices;		//!< Current number of free indices

		// Internal methods
						bool		SetupLists(void** objects=NULL, PxU16* oti=NULL, PxU16* ito=NULL, PxU16* stamps=NULL);
	};

	class ObstacleContext : public PxObstacleContext, public PxUserAllocated
	{
												PX_NOCOPY(ObstacleContext)
		public:
												ObstacleContext(CharacterControllerManager& );
		virtual									~ObstacleContext();

		// PxObstacleContext
		virtual	void							release()	PX_OVERRIDE	PX_FINAL;
		virtual PxControllerManager&			getControllerManager() const	PX_OVERRIDE	PX_FINAL;
		virtual	PxObstacleHandle				addObstacle(const PxObstacle& obstacle)	PX_OVERRIDE	PX_FINAL;
		virtual	bool							removeObstacle(PxObstacleHandle handle)	PX_OVERRIDE	PX_FINAL;
		virtual	bool							updateObstacle(PxObstacleHandle handle, const PxObstacle& obstacle)	PX_OVERRIDE	PX_FINAL;
		virtual	PxU32							getNbObstacles()		const	PX_OVERRIDE	PX_FINAL;
		virtual	const PxObstacle*				getObstacle(PxU32 i)	const	PX_OVERRIDE	PX_FINAL;
		virtual	const PxObstacle*				getObstacleByHandle(PxObstacleHandle handle)	const	PX_OVERRIDE	PX_FINAL;
		//~PxObstacleContext

				const PxObstacle*				raycastSingle(PxGeomRaycastHit& hit, const PxVec3& origin, const PxVec3& unitDir, PxReal distance, PxObstacleHandle& obstacleHandle)	const;
				const PxObstacle*				raycastSingle(PxGeomRaycastHit& hit, const PxObstacleHandle& obstacleHandle, const PxVec3& origin, const PxVec3& unitDir, PxReal distance)	const; // raycast just one obstacle handle

				void							onOriginShift(const PxVec3& shift);

				struct InternalBoxObstacle
				{
					InternalBoxObstacle(PxObstacleHandle handle, const PxBoxObstacle& data) : mHandle(handle), mData(data)	{}

					PxObstacleHandle	mHandle;
					PxBoxObstacle		mData;
				};
				PxArray<InternalBoxObstacle>	mBoxObstacles;

				struct InternalCapsuleObstacle
				{
					InternalCapsuleObstacle(PxObstacleHandle handle, const PxCapsuleObstacle& data) : mHandle(handle), mData(data)	{}

					PxObstacleHandle	mHandle;
					PxCapsuleObstacle	mData;
				};
				PxArray<InternalCapsuleObstacle>	mCapsuleObstacles;

	private:
				HandleManager					mHandleManager;
				CharacterControllerManager&		mCCTManager;
	};


} // namespace Cct

}

/** \endcond */
#endif
