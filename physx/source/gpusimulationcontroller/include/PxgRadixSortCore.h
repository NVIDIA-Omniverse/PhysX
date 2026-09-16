// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_RADIX_SORT_CORE_H
#define PXG_RADIX_SORT_CORE_H

#include "PxgCudaBuffer.h"
#include "PxgEssentialCore.h"
#include "PxgRadixSortDesc.h"
#include "CmPinnableArray.h"

namespace physx
{

	class PxgRadixSortCore
	{
	public:
		Cm::PinnableArray<PxgRadixSortBlockDesc>	mRSDesc;
		PxgCudaBufferN<2>							mRadixSortDescBuf; //radix sort with rank
		PxgCudaBuffer								mRadixCountTotalBuf;
		PxU32										mRadixCountSize;
		PxgEssentialCore*							mEssentialCore;

		PxgRadixSortCore(PxgEssentialCore* core);

		void allocate(PxU32 nbRequired = 1);

		static void updateGPURadixSortDesc(PxCudaContext* cudaContext, const CUstream& stream, CUdeviceptr inputKeyd, CUdeviceptr inputRankd,
			CUdeviceptr outputKeyd, CUdeviceptr outputRankd, CUdeviceptr radixCountd, PxgRadixSortDesc* rsDescs,
			CUdeviceptr radixSortDescBuf0, CUdeviceptr radixSortDescBuf1, const PxU32 count);

		static void sort(PxgCudaKernelWranglerManager* gpuKernelWranglerManager, PxCudaContext*cudaContext, const CUstream& stream,
			const PxU32 numOfKeys, PxgCudaBuffer* radixSortDescBuf, const PxU32 numBits, PxgRadixSortDesc* rsDescs);
		static void sort(PxgCudaKernelWranglerManager* gpuKernelWranglerManager, PxCudaContext*cudaContext, const CUstream& stream,
			PxgCudaBuffer* radixSortDescBuf, const PxU32 numBits);


		static PX_FORCE_INLINE PxI32 getNbBits(PxI32 x)
		{
			PxI32 n = 0;
			while (x >= 2)
			{
				++n;
				x /= 2;
			}

			return n;
		}

		void sort(CUdeviceptr inputKeyd, CUdeviceptr inputRankd, CUdeviceptr outputKeyd, CUdeviceptr outputRankd, const PxU32 numOfKeys, const PxU32 numBits, const CUstream& stream, PxU32 id = 0)
		{
			PxgRadixSortDesc* rsDescs = &mRSDesc[id * 2];

			updateGPURadixSortDesc(mEssentialCore->mCudaContext, stream, inputKeyd, inputRankd, outputKeyd, outputRankd, mRadixCountTotalBuf.getDevicePtr(), rsDescs,
				mRadixSortDescBuf[0].getDevicePtr(), mRadixSortDescBuf[1].getDevicePtr(), numOfKeys);

			sort(mEssentialCore->mGpuKernelWranglerManager, mEssentialCore->mCudaContext, stream, numOfKeys, mRadixSortDescBuf.begin(), numBits, rsDescs);
		}

		void sort(CUdeviceptr inputKeyd, CUdeviceptr inputRankd, CUdeviceptr outputKeyd, CUdeviceptr outputRankd, const PxU32 numOfKeys, const PxU32 numBits, PxU32 id = 0)
		{
			sort(inputKeyd, inputRankd, outputKeyd, outputRankd, numOfKeys, numBits, mEssentialCore->mStream, id);
		}
	};

}

#endif
