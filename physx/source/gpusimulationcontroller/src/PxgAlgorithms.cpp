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

#include "PxgAlgorithms.h"
#include "PxgAlgorithmsData.h"

#include "PxParticleGpu.h"

#include "PxgKernelWrangler.h"
#include "PxgKernelIndices.h"
#include "PxgCudaMemoryAllocator.h"

#include "cudamanager/PxCudaContext.h"
#include "cudamanager/PxCudaContextManager.h"

#include "PxgCudaMemoryAllocator.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"

namespace physx
{

	void scanPerBlockLaunch(PxgKernelLauncher& launcher, const PxU32* data, PxU32* result, PxU32* partialSums, const PxU32 length, const PxU32 numBlocks,
		const PxU32 numThreadsPerBlock, CUstream stream, const PxU32 exclusiveScan, PxU32* totalSum)
	{
		launcher.launchKernel(PxgKernelIds::scanPerBlockKernel, numBlocks, numThreadsPerBlock, numThreadsPerBlock * sizeof(int) / 32, stream,
			data, result, partialSums, length, exclusiveScan, totalSum);
	}

	void addBlockSumsLaunch(PxgKernelLauncher& launcher, const PxU32* partialSums, PxU32* data, const PxU32 length,
		const PxU32 numBlocks, const PxU32 numThreadsPerBlock, CUstream stream, PxU32* totalSum)
	{
		launcher.launchKernel(PxgKernelIds::addBlockSumsKernel, numBlocks, numThreadsPerBlock, 0, stream,
			partialSums, data, length, totalSum);
	}


	void scanPerBlockLaunch(PxgKernelLauncher& launcher, const PxInt4x4* data, PxInt4x4* result, PxInt4x4* partialSums, const PxU32 length, const PxU32 numBlocks,
		const PxU32 numThreadsPerBlock, CUstream stream, const PxU32 exclusiveScan, PxInt4x4* totalSum)
	{
		launcher.launchKernel(PxgKernelIds::scanPerBlockKernel4x4, numBlocks, numThreadsPerBlock, numThreadsPerBlock * sizeof(PxInt4) / 32, stream,
			data, result, partialSums, length, exclusiveScan, totalSum);
	}

	void addBlockSumsLaunch(PxgKernelLauncher& launcher, const PxInt4x4* partialSums, PxInt4x4* data, const PxU32 length,
		const PxU32 numBlocks, const PxU32 numThreadsPerBlock, CUstream stream, PxInt4x4* totalSum)
	{
		launcher.launchKernel(PxgKernelIds::addBlockSumsKernel4x4, numBlocks, numThreadsPerBlock, 0, stream,
			partialSums, data, length, totalSum);
	}

	void radixFourBitCountPerBlockLaunch(PxgKernelLauncher& launcher, const PxU32* data, PxU16* offsets, const int passIndex, PxInt4x4* partialSums, const PxU32 length, const PxU32 numBlocks,
		const PxU32 numThreadsPerBlock, CUstream stream, PxInt4x4* totalSum)
	{
		launcher.launchKernel(PxgKernelIds::radixFourBitCountPerBlockKernel, numBlocks, numThreadsPerBlock, numThreadsPerBlock * sizeof(PxInt4x4) / 32, stream,
			data, offsets, passIndex, partialSums, length, totalSum);
	}

	void radixFourBitReorderLaunch(PxgKernelLauncher& launcher, const PxU32* data, const PxU16* offsets, PxU32* reordered, PxU32 passIndex, PxInt4x4* partialSums, const PxU32 length, PxInt4x4* cumulativeSum,
		const PxU32 numBlocks, const PxU32 numThreadsPerBlock, CUstream stream, PxU32* dependentData, PxU32* dependentDataReordered)
	{
		launcher.launchKernel(PxgKernelIds::radixFourBitReorderKernel, numBlocks, numThreadsPerBlock, 0, stream,
			data, offsets, reordered, passIndex, partialSums, length, cumulativeSum, dependentData, dependentDataReordered);
	}

	void reorderLaunch(PxgKernelLauncher& launcher, const float4* data, float4* reordered, const PxU32 length, const PxU32* reorderedToOriginalMap, CUstream stream)
	{
		const PxU32 numThreadsPerBlock = 512;
		const PxU32 numBlocks = (length + numThreadsPerBlock - 1) / numThreadsPerBlock;
		launcher.launchKernel(PxgKernelIds::reorderKernel, numBlocks, numThreadsPerBlock, 0, stream,
			data, reordered, length, reorderedToOriginalMap);
	}
		

	PX_FORCE_INLINE PxU32 toNextHigherEvenNumber(PxU32 n)
	{
		return n + n % 2;
	}

	static PxU32 computeTmpScanBufferSize(const PxU32 blockSize, const PxU32 n)
	{
		if (n > blockSize)
		{
			const PxU32 numThreads = blockSize;
			const PxU32 numBlocks = (n + numThreads - 1) / numThreads;

			return numBlocks + computeTmpScanBufferSize(blockSize, numBlocks);
		}
		else
		{
			return 1;
		}
	}

	template<typename T>
	void computeBlockSum(PxgKernelLauncher& launcher, const T* blockSum, T* blockSumScan, T* blockSumBlockSum, T* result,
		const PxU32 blockSize, const PxU32 n, const PxU32 numBlocks, const CUstream& stream, T* totalSum = NULL)
	{
		PxU32 numThreads2 = blockSize;
		PxU32 numBlocks2 = (numBlocks + numThreads2 - 1) / numThreads2;
		T* tmp = NULL;
		scanPerBlockLaunch(launcher, blockSum, blockSumScan, blockSumBlockSum, numBlocks, numBlocks2, numThreads2, stream, 1, tmp);

		if (numBlocks2 > 1)
		{
			computeBlockSum<T>(launcher,
				blockSumBlockSum,
				blockSumScan + numBlocks,
				blockSumBlockSum + numBlocks2,
				blockSumScan,
				blockSize,
				numBlocks,
				numBlocks2,
				stream);
		}

		numThreads2 = blockSize;
		numBlocks2 = (n + numThreads2 - 1) / numThreads2;
		addBlockSumsLaunch(launcher, blockSumScan, result, n, numBlocks2, numThreads2, stream, totalSum);
	}

	PX_FORCE_INLINE PxU32 getBits(PxU32 value, PxU32 numBits)
	{
		return value & ((1 << numBits) - 1);
	}

	template<typename T>
	void PxGpuRadixSort<T>::sort(T* inAndOutBuf, PxU32 numBitsToSort, const CUstream& stream, PxU32* outReorderTrackingBuffer, PxU32 numElementsToSort)
	{
		if (!mValueReorderBuffer && outReorderTrackingBuffer)
		{
			mValueReorderBuffer = PX_DEVICE_MEMORY_ALLOC(PxU32, *mKernelLauncher->getCudaContextManager(), mNumElements);
			if (!mValueReorderBuffer)
			{
				// the allocation above can fail, and if we just continue we're getting a mess with the pointers because of the swap below.		
				PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL, "PxGpuRadixSort: failed to allocate reorder buffer, aborting sort!\n");
				return;
			}
		}

		PxU32 numActiveElements = numElementsToSort != 0xFFFFFFFF ? PxU32(numElementsToSort) : mNumElements;
		if (numActiveElements == 0)
			return;

		PxInt4x4* blockSumsBuf = mTempBlockSumsGpuPtr;
		PxInt4x4* blockSumScanBuf = mTempBlockSumScanGpuPtr;

		PxU32 numPasses = toNextHigherEvenNumber((numBitsToSort + 3) / 4);
		PX_ASSERT(numPasses % 2 == 0);

		PxU32 numBlocks = (numActiveElements + mNumThreadsPerBlock - 1) / mNumThreadsPerBlock;
		for (PxU32 i = 0; i < numPasses; ++i)
		{
			radixFourBitCountPerBlockLaunch(*mKernelLauncher, inAndOutBuf, mOffsetBuffer, i, blockSumsBuf, numActiveElements, numBlocks, mNumThreadsPerBlock, stream, mTotalSum);

			computeBlockSum<PxInt4x4>(*mKernelLauncher, blockSumsBuf, blockSumScanBuf, blockSumsBuf + numBlocks, NULL, mNumThreadsPerBlock, numActiveElements, numBlocks, stream, mTotalSum);

			radixFourBitReorderLaunch(*mKernelLauncher, inAndOutBuf, mOffsetBuffer, mReorderBuffer, i, blockSumScanBuf, numActiveElements, mTotalSum, numBlocks, mNumThreadsPerBlock, stream, outReorderTrackingBuffer, mValueReorderBuffer);

			PxSwap(inAndOutBuf, mReorderBuffer);
			if (outReorderTrackingBuffer)
				PxSwap(outReorderTrackingBuffer, mValueReorderBuffer);
		}
	}

	template<typename T>
	bool PxGpuRadixSort<T>::initialize(PxgKernelLauncher* kernelLauncher, PxU32 numElements, PxU32 numThreadsPerBlock)
	{
		if (mTempBlockSumsGpuPtr)
			return false;

		PX_ASSERT(numThreadsPerBlock <= 1024);
		PX_ASSERT(numThreadsPerBlock >= 32);

		mKernelLauncher = kernelLauncher;

		mNumThreadsPerBlock = numThreadsPerBlock;
		mNumElements = numElements;

		mTempBufferSize = computeTmpScanBufferSize(numThreadsPerBlock, numElements);
		mTempBlockSumsGpuPtr = PX_DEVICE_MEMORY_ALLOC(PxInt4x4, *kernelLauncher->getCudaContextManager(), mTempBufferSize);
		mTempBlockSumScanGpuPtr = PX_DEVICE_MEMORY_ALLOC(PxInt4x4, *kernelLauncher->getCudaContextManager(), mTempBufferSize);
		mTotalSum = PX_DEVICE_MEMORY_ALLOC(PxInt4x4, *kernelLauncher->getCudaContextManager(), 1);
			
		mReorderBuffer = PX_DEVICE_MEMORY_ALLOC(T, *kernelLauncher->getCudaContextManager(), numElements);
		mOffsetBuffer = PX_DEVICE_MEMORY_ALLOC(PxU16, *kernelLauncher->getCudaContextManager(), numElements);

		return true;
	}
	template<typename T>
	PxGpuRadixSort<T>::PxGpuRadixSort(PxgKernelLauncher* kernelLauncher, PxU32 numElements, PxU32 numThreadsPerBlock) : mValueReorderBuffer(NULL)
	{
		initialize(kernelLauncher, numElements, numThreadsPerBlock);
	}

	template<typename T>
	bool PxGpuRadixSort<T>::release()
	{
		if (!mTempBlockSumsGpuPtr)
			return false;

		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mTempBlockSumsGpuPtr);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mTempBlockSumScanGpuPtr);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mTotalSum);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mReorderBuffer);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mOffsetBuffer);

		if (mValueReorderBuffer)
			PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mValueReorderBuffer);

		return true;
	}

	void PxGpuScan::scan(PxU32* inAndOutBuf, PxU32 exclusiveScan, const CUstream& stream, PxU32 numElementsToScan)
	{
		PxU32 numActiveElements = numElementsToScan != 0xFFFFFFFF ? PxU32(numElementsToScan) : mNumElements;
		if (numActiveElements == 0)
			return;

		PxU32* blockSumsBuf = mTempBlockSumsGpuPtr;
		PxU32* blockSumScanBuf = mTempBlockSumScanGpuPtr;

		PxU32 numBlocks = (numActiveElements + mNumThreadsPerBlock - 1) / mNumThreadsPerBlock;
		scanPerBlockLaunch(*mKernelLauncher, inAndOutBuf, inAndOutBuf, blockSumsBuf, numActiveElements, numBlocks, mNumThreadsPerBlock, stream, exclusiveScan, mTotalSum);

		if (numBlocks > 1)
			computeBlockSum(*mKernelLauncher, blockSumsBuf, blockSumScanBuf, blockSumsBuf + numBlocks, inAndOutBuf, mNumThreadsPerBlock, numActiveElements, numBlocks, stream, mTotalSum);
	}

	void PxGpuScan::sumOnly(PxU32* inBuf, const CUstream& stream, PxU32 numElementsToScan)
	{
		PxU32 numActiveElements = numElementsToScan != 0xFFFFFFFF ? PxU32(numElementsToScan) : mNumElements;
		if (numActiveElements == 0)
			return;

		PxU32* blockSumsBuf = mTempBlockSumsGpuPtr;
		PxU32* blockSumScanBuf = mTempBlockSumScanGpuPtr;

		PxU32 numBlocks = (numActiveElements + mNumThreadsPerBlock - 1) / mNumThreadsPerBlock;
		PxU32 exclusiveScan = 1;
		scanPerBlockLaunch(*mKernelLauncher, inBuf, NULL, blockSumsBuf, numActiveElements, numBlocks, mNumThreadsPerBlock, stream, exclusiveScan, mTotalSum);

		if (numBlocks > 1)
			computeBlockSum<PxU32>(*mKernelLauncher, blockSumsBuf, blockSumScanBuf, blockSumsBuf + numBlocks, NULL, mNumThreadsPerBlock, numActiveElements, numBlocks, stream, mTotalSum);
	}

	bool PxGpuScan::initialize(PxgKernelLauncher* kernelLauncher, PxU32 numElements, PxU32 numThreadsPerBlock)
	{
		if (mTempBlockSumsGpuPtr)
			return false;

		PX_ASSERT(numThreadsPerBlock <= 1024);
		PX_ASSERT(numThreadsPerBlock >= 32);

		mKernelLauncher = kernelLauncher;

		mNumThreadsPerBlock = numThreadsPerBlock;
		mNumElements = numElements;

		mTempBufferSize = computeTmpScanBufferSize(numThreadsPerBlock, numElements);
		mTempBlockSumsGpuPtr = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), mTempBufferSize);
		mTempBlockSumScanGpuPtr = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), mTempBufferSize);
		mTotalSum = PX_DEVICE_MEMORY_ALLOC(PxU32, *kernelLauncher->getCudaContextManager(), 1);

		return true;
	}

	PxGpuScan::PxGpuScan(PxgKernelLauncher* cudaContextManager, PxU32 numElements, PxU32 numThreadsPerBlock)
		: mTempBlockSumsGpuPtr(NULL), mTempBlockSumScanGpuPtr(NULL), mTotalSum(NULL)
	{
		initialize(cudaContextManager, numElements, numThreadsPerBlock);
	}

	bool PxGpuScan::release()
	{
		if (!mTempBlockSumsGpuPtr)
			return false;

		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mTempBlockSumsGpuPtr);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mTempBlockSumScanGpuPtr);
		PX_DEVICE_MEMORY_FREE(*mKernelLauncher->getCudaContextManager(), mTotalSum);

		return true;
	}

	template class PxGpuRadixSort<PxU32>;	
}
