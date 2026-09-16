// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxgDynamicsContext.h"
#include "PxgKernelWrangler.h"
#include "PxgArticulationCore.h"
#include "PxgCudaSolverCore.h"

namespace physx
{
	PxgDynamicsContext::PxgDynamicsContext(Cm::FlushPool& flushPool, PxsKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
		const PxGpuDynamicsMemoryConfig& config, IG::SimpleIslandManager& islandManager, PxU32 maxNumPartitions, PxU32 maxNumStaticPartitions,
		PxReal maxBiasCoefficient, PxvSimStats& simStats, PxgAllocatorDesc& allocDesc,
		PxReal lengthScale, PxU64 contextID, PxSceneFlags sceneFlags)
	:
	PxgGpuContext(flushPool, islandManager, maxNumPartitions, maxNumStaticPartitions, maxBiasCoefficient, simStats, allocDesc, lengthScale, contextID, false, sceneFlags)
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
		mWorldSolverBodyData.flags = 0;

		mWorldTxIData.sqrtInvInertia = PxMat33(PxZero);
		mWorldTxIData.deltaBody2World = PxTransform(PxIdentity);

		{
			mGpuArticulationCore = PX_NEW(PxgArticulationCore)(static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), cudaContextManager, allocDesc);

			mGpuSolverCore = PX_NEW(PxgCudaSolverCore)(static_cast<PxgCudaKernelWranglerManager*>(gpuKernelWrangler), cudaContextManager, this, allocDesc, config, sceneFlags & PxSceneFlag::eENABLE_FRICTION_EVERY_ITERATION);

			mGpuArticulationCore->setGpuContext(this);
		}

		mGpuSolverCore->acquireContext();

		mGpuSolverCore->createStreams();

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
	}

	void PxgDynamicsContext::destroy()
	{
		this->~PxgDynamicsContext();
		PX_FREE_THIS;
	}
}
