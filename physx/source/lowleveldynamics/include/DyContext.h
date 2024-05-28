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

#ifndef DY_CONTEXT_H
#define DY_CONTEXT_H

#include "PxSceneDesc.h"
#include "DyThresholdTable.h"
#include "PxcNpThreadContext.h"
#include "PxsSimulationController.h"
#include "DyConstraintWriteBack.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxUserAllocated.h"
#include "PxsRigidBody.h"
#include "DyResidualAccumulator.h"

namespace physx
{
class PxcNpMemBlockPool;

namespace Cm
{
	class FlushPool;
}

namespace IG
{
	class SimpleIslandManager;
}

class PxcScratchAllocator;
struct PxvSimStats;
class PxTaskManager;
class PxsContactManager;
struct PxsContactManagerOutputCounts;

class PxvNphaseImplementationContext;

namespace Dy
{

class Context : public PxUserAllocated
{
	PX_NOCOPY(Context)
public:

	// PT: TODO: consider making all of these public at this point
	// PT: please avoid useless comments like "returns Blah" for a function called "getBlah".

	PX_FORCE_INLINE PxReal					getMaxBiasCoefficient()				const	{ return mMaxBiasCoefficient;	}
	PX_FORCE_INLINE void					setMaxBiasCoefficient(PxReal coeff)			{ mMaxBiasCoefficient = coeff;	}

	PX_FORCE_INLINE PxReal					getCorrelationDistance()			const	{ return mCorrelationDistance;	}
	PX_FORCE_INLINE void					setCorrelationDistance(PxReal f)			{ mCorrelationDistance = f;		}

	PX_FORCE_INLINE PxReal					getBounceThreshold()				const	{ return mBounceThreshold;		}
	PX_FORCE_INLINE void					setBounceThreshold(PxReal f)				{ mBounceThreshold = f;			}

	PX_FORCE_INLINE PxReal					getFrictionOffsetThreshold()		const	{ return mFrictionOffsetThreshold;		}
	PX_FORCE_INLINE void					setFrictionOffsetThreshold(PxReal offset)	{ mFrictionOffsetThreshold = offset;	}

	PX_FORCE_INLINE PxReal					getCCDSeparationThreshold()			const	{ return mCCDSeparationThreshold;	}
	PX_FORCE_INLINE void					setCCDSeparationThreshold(PxReal offset)	{ mCCDSeparationThreshold = offset;	}

	PX_FORCE_INLINE PxU32					getSolverBatchSize()				const	{ return mSolverBatchSize;	}
 	PX_FORCE_INLINE void					setSolverBatchSize(PxU32 f)					{ mSolverBatchSize = f;		}

	PX_FORCE_INLINE PxU32					getSolverArticBatchSize()			const	{ return mSolverArticBatchSize; }
	PX_FORCE_INLINE void					setSolverArticBatchSize(PxU32 f)			{ mSolverArticBatchSize = f;	}

	PX_FORCE_INLINE PxFrictionType::Enum	getFrictionType()					const	{ return mFrictionType;	}
	PX_FORCE_INLINE void					setFrictionType(PxFrictionType::Enum f) 	{ mFrictionType = f;	}

	PX_FORCE_INLINE PxReal					getDt()								const	{ return mDt;		}
	PX_FORCE_INLINE void					setDt(const PxReal dt)						{ mDt = dt;			}
	// PT: TODO: we have a setDt function but it doesn't set the inverse dt, what's the story here?
	PX_FORCE_INLINE PxReal					getInvDt()							const	{ return mInvDt;	}

	//Forces any cached body state to be updated!
	PX_FORCE_INLINE	void					setStateDirty(bool dirty)					{ mBodyStateDirty = dirty;	}
	PX_FORCE_INLINE bool					isStateDirty()						const	{ return mBodyStateDirty;	}

	// Returns the maximum solver constraint size in this island in bytes.
	PX_FORCE_INLINE PxU32					getMaxSolverConstraintSize()		const	{ return mMaxSolverConstraintSize; }

	PX_FORCE_INLINE PxReal					getLengthScale()					const	{ return mLengthScale;	}
	PX_FORCE_INLINE const PxVec3&			getGravity()						const	{ return mGravity;		}
	PX_FORCE_INLINE	PxU64					getContextId()						const	{ return mContextID;	}

	PX_FORCE_INLINE ThresholdStream&		getThresholdStream()						{ return *mThresholdStream;				}
	PX_FORCE_INLINE ThresholdStream&		getForceChangedThresholdStream()			{ return *mForceChangedThresholdStream;	}
	PX_FORCE_INLINE ThresholdTable&			getThresholdTable()							{ return mThresholdTable;				}

	void createThresholdStream(PxVirtualAllocatorCallback& callback)			{ PX_ASSERT(!mThresholdStream);	mThresholdStream = PX_NEW(ThresholdStream)(callback);	}
	void createForceChangeThresholdStream(PxVirtualAllocatorCallback& callback) { PX_ASSERT(!mForceChangedThresholdStream); mForceChangedThresholdStream = PX_NEW(ThresholdStream)(callback);	}

	PX_FORCE_INLINE PxcDataStreamPool&							getContactStreamPool()			{ return mContactStreamPool;		}
	PX_FORCE_INLINE PxcDataStreamPool&							getPatchStreamPool()			{ return mPatchStreamPool;			}
	PX_FORCE_INLINE PxcDataStreamPool&							getForceStreamPool()			{ return mForceStreamPool;			}
	PX_FORCE_INLINE PxPinnedArray<Dy::ConstraintWriteback>&		getConstraintWriteBackPool()	{ return mConstraintWriteBackPool;	}
	PX_FORCE_INLINE PxcDataStreamPool&							getFrictionPatchStreamPool()	{ return mFrictionPatchStreamPool;	}

	PX_FORCE_INLINE PxPinnedArray<PxReal>&						getConstraintPositionIterResidualPoolGpu() { return mConstraintPositionIterResidualPoolGpu; }

	//Reports the sum of squared errors of the delta Force corrections. Geometric error was not possible because a compliant contact might have penetration (=geometric error) but can still be solved perfectly
	PX_FORCE_INLINE PxReal										getContactError() const			{ return (mContactErrorVelIter ? mContactErrorVelIter->mErrorSumOfSquares : 0.0f) + (mArticulationContactErrorVelIter.size() ? mArticulationContactErrorVelIter[0].mErrorSumOfSquares : 0.0f); }
	PX_FORCE_INLINE PxU32										getContactErrorCounter() const	{ return (mContactErrorVelIter ? mContactErrorVelIter->mCounter : 0u) + (mArticulationContactErrorVelIter.size() ? mArticulationContactErrorVelIter[0].mCounter : 0u); }
	PX_FORCE_INLINE PxReal										getMaxContactError() const { return PxMax(mContactErrorVelIter ? mContactErrorVelIter->mMaxError : 0.0f, mArticulationContactErrorVelIter.size() ? mArticulationContactErrorVelIter[0].mMaxError : 0.0f); }
	
	PX_FORCE_INLINE PxReal										getContactErrorPosIter() const { return (mContactErrorPosIter ? mContactErrorPosIter->mErrorSumOfSquares : 0.0f) + (mArticulationContactErrorPosIter.size() ? mArticulationContactErrorPosIter[0].mErrorSumOfSquares : 0.0f); }
	PX_FORCE_INLINE PxU32										getContactErrorCounterPosIter() const { return (mContactErrorPosIter ? mContactErrorPosIter->mCounter : 0u) + (mArticulationContactErrorPosIter.size() ? mArticulationContactErrorPosIter[0].mCounter : 0u); }
	PX_FORCE_INLINE PxReal										getMaxContactErrorPosIter() const { return PxMax(mContactErrorPosIter ? mContactErrorPosIter->mMaxError : 0.0f, mArticulationContactErrorPosIter.size() ? mArticulationContactErrorPosIter[0].mMaxError : 0.0f); }


	PX_FORCE_INLINE bool										isResidualReportingEnabled() const { return mIsResidualReportingEnabled; }
	
	/**
	\brief Destroys this dynamics context
	*/
	virtual void						destroy() = 0;

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
										PxvNphaseImplementationContext* nPhaseContext, PxU32 maxPatchesPerCM, PxU32 maxArticulationLinks, PxReal dt, const PxVec3& gravity, PxBitMapPinned& changedHandleMap) = 0;

	virtual void						processLostPatches(IG::SimpleIslandManager& /*simpleIslandManager*/, PxsContactManager** /*lostPatchManagers*/, PxU32 /*nbLostPatchManagers*/, PxsContactManagerOutputCounts* /*outCounts*/)	{}
	virtual void						processFoundPatches(IG::SimpleIslandManager& /*simpleIslandManager*/, PxsContactManager** /*foundPatchManagers*/, PxU32 /*nbFoundPatchManagers*/, PxsContactManagerOutputCounts* /*outCounts*/) {}

	/**
	\brief This method copy gpu solver body data to cpu body core
	*/
	virtual void						updateBodyCore(PxBaseTask* /*continuation*/)	{}

	/**
	\brief Called after update's task chain has completed. This collects the results of the solver together.
	This method combines the results of several islands, e.g. constructing scene-level simulation statistics and merging together threshold streams for contact notification.
	*/
	virtual void						mergeResults() = 0;

	virtual void						setSimulationController(PxsSimulationController* simulationController) = 0;

	virtual void						getDataStreamBase(void*& /*contactStreamBase*/, void*& /*patchStreamBase*/, void*& /*forceAndIndicesStreamBase*/)	{}

	virtual PxSolverType::Enum			getSolverType()	const	= 0;

	virtual PxsExternalAccelerationProvider& getExternalRigidAccelerations() { return mRigidExternalAccelerations; }

protected:

	Context(IG::SimpleIslandManager* islandManager, PxVirtualAllocatorCallback* allocatorCallback,
			PxvSimStats& simStats, bool enableStabilization, bool useEnhancedDeterminism,
			PxReal maxBiasCoefficient, PxReal lengthScale, PxU64 contextID, bool isResidualReportingEnabled) :
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
		mConstraintPositionIterResidualPoolGpu(PxVirtualAllocator(allocatorCallback)),
		mIsResidualReportingEnabled(isResidualReportingEnabled),
		mContactErrorPosIter		(NULL),
		mContactErrorVelIter		(NULL),
		mArticulationContactErrorVelIter(PxVirtualAllocator(allocatorCallback)),
		mArticulationContactErrorPosIter(PxVirtualAllocator(allocatorCallback)),
		mSimStats					(simStats),
		mContextID					(contextID),
		mBodyStateDirty(false),
		mTotalContactError			()
		{
		}

	virtual ~Context() 
	{ 
		PX_DELETE(mThresholdStream);
		PX_DELETE(mForceChangedThresholdStream);
	}

	ThresholdStream*			mThresholdStream;
	ThresholdStream*			mForceChangedThresholdStream;
	ThresholdTable				mThresholdTable;

	IG::SimpleIslandManager*	mIslandManager;
	PxsSimulationController*	mSimulationController;
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
	\brief	Struct to encapsulate the friction patch stream allocations. Used by GPU solver to reference pre-allocated pinned host memory
	*/
	PxcDataStreamPool		mFrictionPatchStreamPool;
	
	/**
	\brief Structure to encapsulate constraint write back allocations. Used by GPU/CPU solver to reference pre-allocated pinned host memory for breakable joint reports.
	*/
	PxPinnedArray<Dy::ConstraintWriteback>	mConstraintWriteBackPool;

	/**
	\brief Buffer that contains only the joint residuals for the gpu solver, cpu solver residuals take a different code path
	*/
	PxPinnedArray<PxReal>	mConstraintPositionIterResidualPoolGpu;

	/**
	\brief Indicates if solver residuals should get computed and reported
	*/
	bool mIsResidualReportingEnabled;

	/**
	\brief Pointer to contact error data during the last position iteration (can point to memory owned by the GPU solver context that is copied to the host asynchronously)
	*/
	Dy::ErrorAccumulator* mContactErrorPosIter;
	
	/**
	\brief Pointer to contact error data during the last velocity iteration (can point to memory owned by the GPU solver context that is copied to the host asynchronously)
	*/
	Dy::ErrorAccumulator* mContactErrorVelIter;

	/**
	\brief Contains the articulation contact error during the last velocity iteration. Has size 1 if articulations are present in the scene. Pinned host memory for fast device to host copies.
	*/
	PxPinnedArray<Dy::ErrorAccumulator> mArticulationContactErrorVelIter;

	/**
	\brief Contains the articulation contact error during the last position iteration. Has size 1 if articulations are present in the scene. Pinned host memory for fast device to host copies.
	*/
	PxPinnedArray<Dy::ErrorAccumulator> mArticulationContactErrorPosIter;


	PxvSimStats& mSimStats;

	const PxU64	mContextID;

	PxsExternalAccelerationProvider mRigidExternalAccelerations;

	bool mBodyStateDirty;

	Dy::ErrorAccumulatorEx mTotalContactError; 
};

Context* createDynamicsContext(	PxcNpMemBlockPool* memBlockPool, PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
								PxvSimStats& simStats, PxTaskManager* taskManager, PxVirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
								IG::SimpleIslandManager* islandManager, PxU64 contextID, bool enableStabilization, bool useEnhancedDeterminism,
								PxReal maxBiasCoefficient, bool frictionEveryIteration, PxReal lengthScale, bool isResidualReportingEnabled);

Context* createTGSDynamicsContext(	PxcNpMemBlockPool* memBlockPool, PxcScratchAllocator& scratchAllocator, Cm::FlushPool& taskPool,
									PxvSimStats& simStats, PxTaskManager* taskManager, PxVirtualAllocatorCallback* allocatorCallback, PxsMaterialManager* materialManager,
									IG::SimpleIslandManager* islandManager, PxU64 contextID, bool enableStabilization, bool useEnhancedDeterminism, PxReal lengthScale, 
									bool externalForcesEveryTgsIterationEnabled, bool isResidualReportingEnabled);
}

}

#endif

