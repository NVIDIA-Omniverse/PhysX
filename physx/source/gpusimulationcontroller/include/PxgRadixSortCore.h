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

#ifndef PXG_RADIX_SORT_CORE_H
#define PXG_RADIX_SORT_CORE_H

#include "PxgCudaBuffer.h"
#include "PxgEssentialCore.h"
#include "PxgKernelWrangler.h"
#include "PxgRadixSortDesc.h"

#include "cudamanager/PxCudaTypes.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxPinnedArray.h"

namespace physx
{
	class PxCudaContext;
	class PxgCudaKernelWranglerManager;

	class PxgRadixSortCore
	{
	public:
		PxPinnedArray<PxgRadixSortBlockDesc>	mRSDesc;
		PxgCudaBufferN<2>						mRadixSortDescBuf; //radix sort with rank
		PxgCudaBuffer							mRadixCountTotalBuf;
		PxU32									mRadixCountSize;
		PxgEssentialCore*						mEssentialCore;

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
