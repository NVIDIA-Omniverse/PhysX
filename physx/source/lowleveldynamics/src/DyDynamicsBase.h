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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_DYNAMICS_BASE_H
#define DY_DYNAMICS_BASE_H

#include "DyContext.h"
#include "DyThreadContext.h"
#include "PxvNphaseImplementationContext.h"
#include "PxsIslandManagerTypes.h"
#include "solver/PxSolverDefs.h"

namespace physx
{
namespace Cm
{
	class FlushPool;
}

namespace IG
{
	class SimpleIslandManager;
}

namespace Dy
{

// PT: base class containing code and data shared between PGS and TGS. Ideally this would just be named "DynamicsContext" and the PGS
// context would have been renamed "DynamicsPGSContext" (to match DynamicsTGSContext) but let's limit the gratuitous changes for now.
class DynamicsContextBase : public Context
{
	PX_NOCOPY(DynamicsContextBase)
public:
			DynamicsContextBase(PxcNpMemBlockPool* memBlockPool,
								Cm::FlushPool& taskPool,
								PxvSimStats& simStats,
								PxVirtualAllocatorCallback* allocatorCallback,
								PxsMaterialManager* materialManager,
								IG::SimpleIslandManager& islandManager,
								PxU64 contextID,
								PxReal maxBiasCoefficient,
								PxReal lengthScale,
								bool enableStabilization,
								bool useEnhancedDeterminism,
								bool isResidualReportingEnabled
								);

	virtual	~DynamicsContextBase();

	/**
	\brief Allocates and returns a thread context object.
	\return A thread context.
	*/
	PX_FORCE_INLINE ThreadContext*		getThreadContext()	{ return mThreadContextPool.get();	}

	/**
	\brief Returns a thread context to the thread context pool.
	\param[in] context The thread context to return to the thread context pool.
	*/
					void				putThreadContext(ThreadContext* context)	{ mThreadContextPool.put(context);	}

	PX_FORCE_INLINE ThresholdStream&	getThresholdStream()			{ return *mThresholdStream; }
	PX_FORCE_INLINE PxvSimStats&		getSimStats()					{ return mSimStats;			}
	PX_FORCE_INLINE Cm::FlushPool&		getTaskPool()					{ return mTaskPool;			}
	PX_FORCE_INLINE	PxU32				getKinematicCount()		const	{ return mKinematicCount;	}

	PxcThreadCoherentCache<ThreadContext, PxcNpMemBlockPool> mThreadContextPool;	// A thread context pool

	PxsMaterialManager*				mMaterialManager;
	Cm::FlushPool&					mTaskPool;
	PxsContactManagerOutputIterator	mOutputIterator;

	PxArray<PxConstraintBatchHeader>	mContactConstraintBatchHeaders;	// An array of contact constraint batch headers
	PxArray<Cm::SpatialVector>			mMotionVelocityArray;			// Array of motion velocities for all bodies in the scene.
	PxArray<PxsBodyCore*>				mBodyCoreArray;					// Array of body core pointers for all bodies in the scene.
	PxArray<PxsRigidBody*>				mRigidBodyArray;				// Array of rigid body pointers for all bodies in the scene.
	PxArray<FeatherstoneArticulation*>	mArticulationArray;				// Array of articulation pointers for all articulations in the scene.

	ThresholdStream*					mExceededForceThresholdStream[2]; // this store previous and current exceeded force thresholdStream	
	PxArray<PxU32>						mExceededForceThresholdStreamMask;
	PxArray<PxU32>						mSolverBodyRemapTable;	// Remaps from the "active island" index to the index within a solver island
	PxArray<PxU32>						mNodeIndexArray;		// island node index
	PxArray<PxsIndexedContactManager>	mContactList;

	PxU32	mKinematicCount;		// The total number of kinematic bodies in the scene
	PxI32	mThresholdStreamOut;	// Atomic counter for the number of threshold stream elements.
	PxU32	mCurrentIndex;			// this is the index point to the current exceeded force threshold stream

protected:
	void	resetThreadContexts();
	PxU32	reserveSharedSolverConstraintsArrays(const IG::IslandSim& islandSim, PxU32 maxArticulationLinks);
};

}
}

#endif
