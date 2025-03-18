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

#ifndef PXG_NONRIGID_CORE_COMMON_H
#define PXG_NONRIGID_CORE_COMMON_H

#include "PxgCudaBuffer.h"
#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "PxgCudaPagedLinearAllocator.h"
#include "PxgRadixSortDesc.h"
#include "foundation/PxPinnedArray.h"
#include "PxgEssentialCore.h"
#include "PxNodeIndex.h"

namespace physx
{

	class PxgCudaKernelWranglerManager;
	class PxCudaContextManager;
	class PxgHeapMemoryAllocatorManager;

	struct PxGpuDynamicsMemoryConfig;

	class PxgSimulationController;
	class PxgCudaBroadPhaseSap;
	class PxgGpuNarrowphaseCore;
	class PxgGpuContext;

	class PxgNonRigidCore : public PxgEssentialCore
	{
	public:
		PxgNonRigidCore(PxgCudaKernelWranglerManager* gpuKernelWrangler, PxCudaContextManager* cudaContextManager,
			PxgHeapMemoryAllocatorManager* heapMemoryManager, PxgSimulationController* simController,
			PxgGpuContext* context, const PxU32 maxContacts, const PxU32 collisionStackSize, PxsHeapStats::Enum statType);

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
		
		PxgCudaPagedLinearAllocator<PxgHeapMemoryAllocator> mIntermStackAlloc;
		PxgTypedCudaBuffer<PxU32>		mStackSizeNeededOnDevice;
		PxU32*							mStackSizeNeededPinned;

		PxU32							mMaxContacts;
		PxU32							mCollisionStackSizeBytes;
		
		//for sorting contacts
		PxPinnedArray<PxgRadixSortBlockDesc>		mRSDesc;
		PxgCudaBufferN<2>				mRadixSortDescBuf; //radix sort with rank
		PxgCudaBuffer					mRadixCountTotalBuf;
		PxU32							mRadixCountSize;

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