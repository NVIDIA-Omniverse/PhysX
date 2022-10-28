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

#ifndef DY_CONTEXT_H
#define DY_CONTEXT_H

#include "PxSceneDesc.h"
#include "DyThresholdTable.h"
#include "PxcNpThreadContext.h"
#include "PxsSimulationController.h"
#include "DyConstraintWriteBack.h"
#include "foundation/PxAllocator.h"

#define DY_MAX_VELOCITY_COUNT 4

namespace physx
{

class PxsIslandManager;
class PxcNpMemBlockPool;

namespace Cm
{
	class EventProfiler;
	class FlushPool;
}

namespace IG
{
	class SimpleIslandManager;
	class IslandSim;
}

template<typename T, typename P> class PxcThreadCoherentCache;
class PxcScratchAllocator;
struct PxvSimStats;
class PxTaskManager;
class PxsContactManagerOutputIterator;
struct PxsContactManagerOutput;
class PxsContactManager;
struct PxsContactManagerOutputCounts;

class PxvNphaseImplementationContext;


namespace Dy
{


class Context
{
	PX_NOCOPY(Context)
public:

	// PT: TODO: consider making all of these public at this point
	PX_FORCE_INLINE PxReal				getMaxBiasCoefficient()			const	{ return mMaxBiasCoefficient;	}
	PX_FORCE_INLINE void				setMaxBiasCoefficient(PxReal coeff)		{ mMaxBiasCoefficient = coeff;	}

	/**
	\brief Returns the bounce threshold
	\return The bounce threshold.
	*/
	PX_FORCE_INLINE PxReal				getBounceThreshold()			const	{ return mBounceThreshold;	}
	/**
	\brief Returns the friction offset threshold
	\return The friction offset threshold.
	*/
	PX_FORCE_INLINE PxReal				getFrictionOffsetThreshold()	const	{ return mFrictionOffsetThreshold;	}
	/**
	\brief Returns the correlation distance
	\return The correlation distance.
	*/
	PX_FORCE_INLINE PxReal				getCorrelationDistance()		const	{ return mCorrelationDistance;	}

	/**
	\brief Returns the CCD separation threshold
	\return The CCD separation threshold.
	*/
	PX_FORCE_INLINE PxReal				getCCDSeparationThreshold()		const	{ return mCCDSeparationThreshold; }

	/**
	\brief Returns the length scale
	\return The length scale.
	*/
	PX_FORCE_INLINE PxReal				getLengthScale()				const	{ return mLengthScale; }

	/**
	\brief Sets the bounce threshold
	\param[in] f The bounce threshold
	*/
	PX_FORCE_INLINE void				setBounceThreshold(PxReal f)			{ mBounceThreshold = f;		}
	/**
	\brief Sets the correlation distance
	\param[in] f The correlation distance
	*/
	PX_FORCE_INLINE void				setCorrelationDistance(PxReal f)			{ mCorrelationDistance = f;		}
	/**
	\brief Sets the friction offset threshold
	\param[in] offset The friction offset threshold
	*/
	PX_FORCE_INLINE void				setFrictionOffsetThreshold(PxReal offset)		{ mFrictionOffsetThreshold = offset;				}

	/**
	\brief Sets the friction offset threshold
	\param[in] offset The friction offset threshold
	*/
	PX_FORCE_INLINE void				setCCDSeparationThreshold(PxReal offset)		{ mCCDSeparationThreshold = offset; }

	/**
	\brief Returns the solver batch size
	\return The solver batch size.
	*/
	PX_FORCE_INLINE PxU32				getSolverBatchSize()				const	{ return mSolverBatchSize;	}
	/**
	\brief Sets the solver batch size
	\param[in] f The solver batch size
	*/
 	PX_FORCE_INLINE void				setSolverBatchSize(PxU32 f)				{ mSolverBatchSize = f;		}

	/**
	\brief Returns the solver batch size
	\return The solver batch size.
	*/
	PX_FORCE_INLINE PxU32				getSolverArticBatchSize()				const { return mSolverArticBatchSize; }
	/**
	\brief Sets the solver batch size
	\param[in] f The solver batch size
	*/
	PX_FORCE_INLINE void				setSolverArticBatchSize(PxU32 f) { mSolverArticBatchSize = f; }

	/**
	\brief Returns the maximum solver constraint size
	\return The maximum solver constraint size in this island in bytes.
	*/
	PX_FORCE_INLINE PxU32				getMaxSolverConstraintSize()	const	{ return mMaxSolverConstraintSize; }

	/**
	\brief Returns the friction model being used.
	\return The friction model being used.
	*/
	PX_FORCE_INLINE PxFrictionType::Enum getFrictionType() const				{ return mFrictionType; }

	/**
	\brief Returns the threshold stream
	\return The threshold stream
	*/
	PX_FORCE_INLINE ThresholdStream&	getThresholdStream()					{ return *mThresholdStream; }

	PX_FORCE_INLINE ThresholdStream&	getForceChangedThresholdStream()		{ return *mForceChangedThresholdStream; }

	/**
	\brief Returns the threshold table
	\return The threshold table
	*/
	PX_FORCE_INLINE ThresholdTable&		getThresholdTable()						{ return mThresholdTable; }

	/**
	\brief Sets the friction model to be used.
	\param[in] f The friction model to be used.
	*/
	PX_FORCE_INLINE void				setFrictionType(PxFrictionType::Enum f) 	{ mFrictionType = f; }

	/**
	\brief Destroys this dynamics context
	*/
	virtual void						destroy() = 0;

	PX_FORCE_INLINE PxcDataStreamPool&				getContactStreamPool()						{ return mContactStreamPool;	}

	PX_FORCE_INLINE PxcDataStreamPool&				getPatchStreamPool()						{ return mPatchStreamPool;	}

	PX_FORCE_INLINE PxcDataStreamPool&				getForceStreamPool()						{ return mForceStreamPool;	}

	PX_FORCE_INLINE PxPinnedArray<Dy::ConstraintWriteback>&		getConstraintWriteBackPool()			{ return mConstraintWriteBackPool;  }

	/**
	\brief Returns the current frame's timestep
	\return The current frame's timestep.
	*/
	PX_FORCE_INLINE PxReal					getDt()							const	{ return mDt;				}
	/**
	\brief Returns 1/(current frame's timestep)
	\return 1/(current frame's timestep).
	*/
	PX_FORCE_INLINE PxReal					getInvDt()						const	{ return mInvDt;			}

	PX_FORCE_INLINE PxVec3					getGravity()					const	{ return mGravity;			}

	/**
	\brief The entry point for the constraint solver. 
	\param[in]	dt	The simulation time-step	
	\param[in]	continuation The continuation task for the solver
	\param[in] processLostTouchTask The task that processes lost touches	

	This method is called after the island generation has completed. Its main responsibilities are:
	(1) Reserving the solver body pools
	(2) Initializing the static and kinematic solver bodies, which are shared resources between islands.
	(3) Construct the solver task chains for each island

	Each island is solved as an independent solver task chain. In addition, large islands may be solved using multiple parallel tasks.
	Island solving is asynchronous. Once all islands have been solved, the continuation task will be called.

	*/
	virtual void						update(IG::SimpleIslandManager& simpleIslandManager, PxBaseTask* continuation, PxBaseTask* processLostTouchTask,
		PxvNphaseImplementationContext* nPhaseContext, const PxU32 maxPatchesPerCM, const PxU32 maxArticulationLinks, const PxReal dt, const PxVec3& gravity, PxBitMapPinned& changedHandleMap) = 0;

	virtual void						processLostPatches(IG::SimpleIslandManager& simpleIslandManager, PxsContactManager** lostPatchManagers, PxU32 nbLostPatchManagers, PxsContactManagerOutputCounts* outCounts) = 0;
	virtual void						processFoundPatches(IG::SimpleIslandManager& simpleIslandManager, PxsContactManager** foundPatchManagers, PxU32 nbFoundPatchManagers, PxsContactManagerOutputCounts* outCounts) = 0;


	/**
	\brief This method copy gpu solver body data to cpu body core
	*/
	virtual void						updateBodyCore(PxBaseTask* continuation) = 0;

	/**
	\brief Called after update's task chain has completed. This collects the results of the solver together
	*/
	virtual void						mergeResults() = 0;

	virtual void						setSimulationController(PxsSimulationController* simulationController) = 0;

	virtual void						getDataStreamBase(void*& contactStreamBase, void*& patchStreamBase, void*& forceAndIndiceStreamBase) = 0;

	virtual PxSolverType::Enum			getSolverType()	const	= 0;

	void createThresholdStream(PxVirtualAllocatorCallback& callback)			{ PX_ASSERT(!mThresholdStream);	mThresholdStream = PX_NEW(ThresholdStream)(callback);	}

	void createForceChangeThresholdStream(PxVirtualAllocatorCallback& callback) { PX_ASSERT(!mForceChangedThresholdStream); mForceChangedThresholdStream = PX_NEW(ThresholdStream)(callback);	}

	//Forces any cached body state to be updated!
	void setStateDirty(bool dirty) { mBodyStateDirty = dirty; }

	bool isStateDirty() { return mBodyStateDirty;}

	void setSuppressReadback(bool suppressReadback) { mSuppressReadback = suppressReadback; }

	bool getSuppressReadback() { return mSuppressReadback; }

	void setDt(const PxReal dt) { mDt = dt; }

protected:

	Context(IG::SimpleIslandManager* islandManager, PxVirtualAllocatorCallback* allocatorCallback,
		PxvSimStats& simStats, bool enableStabilization, bool useEnhancedDeterminism,
		const PxReal maxBiasCoefficient, const PxReal lengthScale) :
		mThresholdStream			(NULL),
		mForceChangedThresholdStream(NULL),		
		mIslandManager				(islandManager),
		mDt							(1.0f), 
		mInvDt						(1.0f),
		mMaxBiasCoefficient			(maxBiasCoefficient),
		mEnableStabilization		(enableStabilization),
		mUseEnhancedDeterminism		(useEnhancedDeterminism),
		mBounceThreshold			(-2.0f),
		mLengthScale				(lengthScale),
		mSolverBatchSize			(32),
		mConstraintWriteBackPool	(PxVirtualAllocator(allocatorCallback)),
		mSimStats					(simStats),
		mBodyStateDirty				(false),
		mSuppressReadback			(false)
		{
		}

	virtual ~Context() 
	{ 
		PX_DELETE(mThresholdStream);
		PX_DELETE(mForceChangedThresholdStream);
	}

	ThresholdStream*						mThresholdStream;
	ThresholdStream*						mForceChangedThresholdStream;
	ThresholdTable							mThresholdTable;

	IG::SimpleIslandManager*				mIslandManager;
	PxsSimulationController*				mSimulationController;
	/**
	\brief Time-step.
	*/
	PxReal						mDt;
	/**
	\brief 1/time-step.
	*/
	PxReal						mInvDt;

	PxReal						mMaxBiasCoefficient;

	const bool					mEnableStabilization;

	const bool					mUseEnhancedDeterminism;

	PxVec3						mGravity;
	/**
	\brief max solver constraint size
	*/
	PxU32						mMaxSolverConstraintSize;

	/**
	\brief Threshold controlling the relative velocity at which the solver transitions between restitution and bias for solving normal contact constraint.
	*/
	PxReal						mBounceThreshold;
	/**
	\brief Threshold controlling whether friction anchors are constructed or not. If the separation is above mFrictionOffsetThreshold, the contact will not be considered to become a friction anchor
	*/
	PxReal						mFrictionOffsetThreshold;

	/**
	\brief Threshold controlling whether distant contacts are processed using bias, restitution or a combination of the two. This only has effect on pairs involving bodies that have enabled speculative CCD simulation mode.
	*/
	PxReal						mCCDSeparationThreshold;

	/**
	\brief Threshold for controlling friction correlation
	*/
	PxReal						mCorrelationDistance;


	/**
	\brief The length scale from PxTolerancesScale::length.
	*/
	PxReal						mLengthScale;

	/**
	\brief The minimum size of an island to generate a solver task chain.
	*/
	PxU32						mSolverBatchSize;

	/**
	\brief The minimum number of articulations required to generate a solver task chain.
	*/
	PxU32						mSolverArticBatchSize;


	/**
	\brief The current friction model being used
	*/
	PxFrictionType::Enum		mFrictionType;

	/**
	\brief Structure to encapsulate contact stream allocations. Used by GPU solver to reference pre-allocated pinned host memory
	*/
	PxcDataStreamPool		mContactStreamPool;

	/**
	\brief	Struct to encapsulate the contact patch stream allocations. Used by GPU solver to reference pre-allocated pinned host memory
	*/

	PxcDataStreamPool		mPatchStreamPool;

	/**
	\brief Structure to encapsulate force stream allocations. Used by GPU solver to reference pre-allocated pinned host memory for force reports.
	*/
	PxcDataStreamPool		mForceStreamPool;
	
	/**
	\brief Structure to encapsulate constraint write back allocations. Used by GPU/CPU solver to reference pre-allocated pinned host memory for breakable joint reports.
	*/
	PxPinnedArray<Dy::ConstraintWriteback>	mConstraintWriteBackPool;

	PxvSimStats& mSimStats;

	bool mBodyStateDirty;
	bool mSuppressReadback;
};

Context* createDynamicsContext(	PxcNpMemBlockPool* memBlockPool,
								PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
								PxvSimStats& simStats, PxTaskManager* taskManager, PxVirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
								IG::SimpleIslandManager* islandManager, PxU64 contextID,
								const bool enableStabilization, const bool useEnhancedDeterminism, const PxReal maxBiasCoefficient,
								const bool frictionEveryIteration, const PxReal lengthScale
								);

Context* createTGSDynamicsContext(PxcNpMemBlockPool* memBlockPool,
	PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
	PxvSimStats& simStats, PxTaskManager* taskManager, PxVirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
	IG::SimpleIslandManager* islandManager, PxU64 contextID,
	const bool enableStabilization, const bool useEnhancedDeterminism, const PxReal lengthScale
);


}

}

#endif

