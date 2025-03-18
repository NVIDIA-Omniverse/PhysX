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

#include "PxgTGSDynamicsContext.h"
#include "PxgKernelWrangler.h"
#include "PxgArticulationCore.h"
#include "PxgTGSCudaSolverCore.h"

namespace physx
{
	PxgTGSDynamicsContext::PxgTGSDynamicsContext(Cm::FlushPool& flushPool, PxsKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
		const PxGpuDynamicsMemoryConfig& config, IG::SimpleIslandManager& islandManager, PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions,
		bool enableStabilization, bool useEnhancedDeterminism,
		PxReal maxBiasCoefficient,
		PxvSimStats& simStats, PxgHeapMemoryAllocatorManager* heapMemoryManager,
		bool externalForcesEveryTgsIterationEnabled, PxReal lengthScale, bool enableDirectGPUAPI, PxU64 contextID, bool isResidualReportingEnabled)
		:
		PxgGpuContext(flushPool, islandManager, maxNumPartitions, maxNumStaticPartitions, enableStabilization, useEnhancedDeterminism,
			maxBiasCoefficient, simStats, heapMemoryManager, lengthScale, enableDirectGPUAPI, contextID, isResidualReportingEnabled, true)
	{
		mWorldSolverBody.linearVelocity = PxVec3(0);
		mWorldSolverBody.angularVelocity = PxVec3(0);
		mWorldSolverBodyData.invMass = 0;
		mWorldSolverBodyData.reportThreshold = PX_MAX_REAL;
		mWorldSolverBodyData.maxImpulse = PX_MAX_REAL;
		mWorldSolverBodyData.penBiasClamp = -PX_MAX_REAL;
		mWorldSolverBodyData.initialAngVel = mWorldSolverBodyData.initialLinVel = PxVec3(0.f);
		mWorldSolverBodyData.body2World = PxAlignedTransform(PxIdentity);
		mWorldSolverBodyData.islandNodeIndex = PxNodeIndex(PX_INVALID_NODE);
		mWorldSolverBodyData.offsetSlop = 0.f;

		mWorldTxIData.sqrtInvInertia = PxMat33(PxZero);
		mWorldTxIData.deltaBody2World = PxTransform(PxIdentity);

		{
			mGpuArticulationCore = PX_NEW(PxgArticulationCore)(static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), cudaContextManager, heapMemoryManager);

			mGpuSolverCore = PX_NEW(PxgTGSCudaSolverCore)(static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), cudaContextManager, this, heapMemoryManager, config);

			mGpuArticulationCore->setGpuContext(this);
		}

		mGpuSolverCore->acquireContext();

		mGpuSolverCore->createStreams();

		createThresholdStream(*heapMemoryManager->mMappedMemoryAllocators);
		createForceChangeThresholdStream(*heapMemoryManager->mMappedMemoryAllocators);

		mPinnedMemoryAllocator = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.tempBufferCapacity);

		mCurrentContactStream = 0;
		mContactStreamAllocators[0] = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.maxRigidContactCount * sizeof(PxContact));
		mContactStreamAllocators[1] = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.maxRigidContactCount * sizeof(PxContact));

		mPatchStreamAllocators[0] = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.maxRigidPatchCount * sizeof(PxContactPatch));
		mPatchStreamAllocators[1] = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.maxRigidPatchCount * sizeof(PxContactPatch));
	
		mForceStreamAllocator = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.maxRigidContactCount * sizeof(PxReal) * 2);

		mFrictionPatchStreamAllocator = PX_NEW(PxgPinnedHostLinearMemoryAllocator)(cudaContextManager, config.maxRigidPatchCount * sizeof(PxFrictionPatch));

		mContactStreamPool.mDataStream = mContactStreamAllocators[mCurrentContactStream]->mStart;
		mContactStreamPool.mDataStreamSize = (PxU32)mContactStreamAllocators[mCurrentContactStream]->mTotalSize;
		mContactStreamPool.mSharedDataIndex = 0;
		mContactStreamPool.mSharedDataIndexGPU = 0;

		mPatchStreamPool.mDataStream = mPatchStreamAllocators[mCurrentContactStream]->mStart;
		mPatchStreamPool.mDataStreamSize = (PxU32)mPatchStreamAllocators[mCurrentContactStream]->mTotalSize;
		mPatchStreamPool.mSharedDataIndex = 0;
		mPatchStreamPool.mSharedDataIndexGPU = 0;

		mForceStreamPool.mDataStream = mForceStreamAllocator->mStart;
		mForceStreamPool.mDataStreamSize = (PxU32)mForceStreamAllocator->mTotalSize;
		mForceStreamPool.mSharedDataIndex = 0;
		mForceStreamPool.mSharedDataIndexGPU = 0;

		mFrictionPatchStreamPool.mDataStream = mFrictionPatchStreamAllocator->mStart;
		mFrictionPatchStreamPool.mDataStreamSize = PxTo32(mFrictionPatchStreamAllocator->mTotalSize);
		mFrictionPatchStreamPool.mSharedDataIndex = 0;
		mFrictionPatchStreamPool.mSharedDataIndexGPU = 0;

		//Arbitrarily-large number to reserve to minimize allocation churn.
		mConstraintsPerPartition.reserve(1024);

		mArtiConstraintsPerPartition.reserve(1024);

		mGpuSolverCore->releaseContext();
	    mIsExternalForcesEveryTgsIterationEnabled = externalForcesEveryTgsIterationEnabled;
	}

	void PxgTGSDynamicsContext::destroy()
	{
		this->~PxgTGSDynamicsContext();
		PX_FREE_THIS;
	}
}
