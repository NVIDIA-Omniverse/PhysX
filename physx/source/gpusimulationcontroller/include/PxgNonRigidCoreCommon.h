// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_NONRIGID_CORE_COMMON_H
#define PXG_NONRIGID_CORE_COMMON_H

#include "common/PxPhysXCommonConfig.h"
#include "PxNodeIndex.h"
#include "CmPinnableArray.h"
#include "CmPinnableObject.h"
#include "PxgEssentialCore.h"
#include "PxgCudaBuffer.h"
#include "PxgCudaPagedLinearAllocator.h"

namespace physx
{
	struct PxgRadixSortBlockDesc;

	class PxgNonRigidCore : public PxgEssentialCore
	{
	public:
		PxgNonRigidCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgAllocatorDesc& allocDesc, PxgSimulationController* simController,
			PxgGpuContext* context, const PxU32 maxContacts, const PxU32 collisionStackSize, PxU32 statType);

		virtual ~PxgNonRigidCore();

		void updateGPURadixSortBlockDesc(CUstream stream, CUdeviceptr inputKeyd, CUdeviceptr inputRankd,
			CUdeviceptr outputKeyd, CUdeviceptr outputRankd, CUdeviceptr radixCountd,
			CUdeviceptr numKeysd, PxgRadixSortBlockDesc* rsDescs,
			CUdeviceptr radixSortDescBuf0, CUdeviceptr radixSortDescBuf1);

		PX_FORCE_INLINE PxgTypedCudaBuffer<PxNodeIndex>& getContactByRigid() { return mContactByRigidBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxNodeIndex>& getContactSortedByRigid() { return mContactSortedByRigidBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getTempContactByRigid() { return mTempContactByRigidBitBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getContactRemapSortedByRigid() { return mContactRemapSortedByRigidBuf; }

		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU64>& getContactSortedByParticle() { return mContactSortedByParticleBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getTempContactByParticle() { return mTempContactByParticleBitBuf; }
		PX_FORCE_INLINE PxgTypedCudaBuffer<PxU32>& getContactRemapSortedByParticle() { return mContactRemapSortedByParticleBuf; }
		
		PxgCudaPagedLinearAllocator		mIntermStackAlloc;
		PxgTypedCudaBuffer<PxU32>		mStackSizeNeededOnDevice;

		Cm::PinnableObject<PxU32>		mStackSizeNeededPinned;

		PxU32							mMaxContacts;
		PxU32							mCollisionStackSizeBytes;
		
		//for sorting contacts
		Cm::PinnableArray<PxgRadixSortBlockDesc>	mRSDesc;
		PxgCudaBufferN<2>							mRadixSortDescBuf; //radix sort with rank
		PxgCudaBuffer								mRadixCountTotalBuf;
		PxU32										mRadixCountSize;

		//for radix sort
		PxgTypedCudaBuffer<PxNodeIndex>	mContactByRigidBuf;			//rigidId is nodeIndex, which is 64 bit
		PxgTypedCudaBuffer<PxNodeIndex> mContactSortedByRigidBuf;	//rigidId is nodeIndex, which is 64 bit
		PxgTypedCudaBuffer<PxU32>		mTempContactByRigidBitBuf; //low/high 32 bit
		PxgTypedCudaBuffer<PxU32>		mContactRemapSortedByRigidBuf; //rank index

		PxgTypedCudaBuffer<PxU64>		mContactSortedByParticleBuf;	//PxU64 particle system id and particle index
		PxgTypedCudaBuffer<PxU32>		mTempContactByParticleBitBuf; //low/high 32 bit
		PxgTypedCudaBuffer<PxU32>		mContactRemapSortedByParticleBuf; //rank index

	
		PxgTypedCudaBuffer<PxU32>		mTempContactBuf;
		PxgTypedCudaBuffer<PxU32>		mTempContactRemapBuf;
		PxgTypedCudaBuffer<PxU32>		mTempContactBuf2;
		PxgTypedCudaBuffer<PxU32>		mTempContactRemapBuf2;

#if PX_ENABLE_SIM_STATS
		PxU32							mCollisionStackSizeBytesStats;
#else
		PX_CATCH_UNDEFINED_ENABLE_SIM_STATS
#endif
	};
}

#endif