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

#ifndef PXG_ALGORITHMS_H
#define PXG_ALGORITHMS_H

#include "foundation/PxSimpleTypes.h"
#include "PxgAlgorithmsData.h"
#include "PxgKernelLauncher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	class PxgKernelLauncher;

	/**
	\brief Performs a sort operation on the GPU
	*/
	template<typename T>
	class PxGpuRadixSort
	{
	protected:
		PxgKernelLauncher* mKernelLauncher;

		PxU32 mTempBufferSize;

		PxInt4x4* mTempBlockSumsGpuPtr;
		PxInt4x4* mTempBlockSumScanGpuPtr;
		PxInt4x4* mTotalSum;

		PxU32 mNumThreadsPerBlock = 0;
		PxU32 mNumElements = 0;

		T* mReorderBuffer;
		PxU16* mOffsetBuffer;

		//Optional, reorder buffer when sorting key-value pairs, uses lazy initialization
		PxU32* mValueReorderBuffer;

	public:
		/**
		\brief Empty constructor which allows creating uninitialized objects
		*/
		PxGpuRadixSort() : mTempBlockSumsGpuPtr(NULL), mTempBlockSumScanGpuPtr(NULL), mTotalSum(NULL), mValueReorderBuffer(NULL) {}

		/**
		\brief Constructor that initializes and allocates all internal data

		\param[in]	cudaContextManager		The cuda context manager
		\param[in]	numElements				The maximum number of elements that can be processed by this gpu sort instance
		\param[in]	numThreadsPerBlock		The number of threads applied per block when scheduling the gpu work
		*/
		PxGpuRadixSort(PxgKernelLauncher* cudaContextManager, PxU32 numElements, PxU32 numThreadsPerBlock = 512);

		/**
		\brief Initializes and allocates all internal data

		\param[in]	cudaContextManager		The cuda context manager
		\param[in]	numElements				The maximum number of elements that can be processed by this gpu sort instance
		\param[in]	numThreadsPerBlock		The number of threads applied per block when scheduling the gpu work
		*/
		virtual bool initialize(PxgKernelLauncher* cudaContextManager, PxU32 numElements, PxU32 numThreadsPerBlock = 512);

		/**
		\brief Sorts the integer array in place

		\param[in,out]	inAndOutBuffer				Gpu array with the integer data which gets sorted
		\param[in]		numBitsToSort				The number of bits to sort. For 32bit integers where it is known that only 24 bits are used at most, it is sufficient to sort 24 bits only.
		\param[in]		stream						Gpu stream on which the calculation is scheduled. To be sure that the sort finished, a synchronize call must be executed on that stream.
		\param[in]		outReorderTrackingBuffer	Optional: Gpu tracking buffer that contains the original location in the unsorted array for every element after the sorting completed.
		\param[in]		numElementsToSort			Optional: The number of elements that should get sorted. By default all elements are processed. The maximal number of elements is specified in the constructor.
		*/
		virtual void sort(T* inAndOutBuffer, PxU32 numBitsToSort, const CUstream& stream, PxU32* outReorderTrackingBuffer = NULL, PxU32 numElementsToSort = 0xFFFFFFFF);

		/**
		\brief Releases all internal data
		*/
		virtual bool release();

		virtual ~PxGpuRadixSort() { }
	};

	/**
	\brief Performs a scan operation (exclusive or inclusive cumulative sum) on the GPU
	*/
	class PxGpuScan
	{
	private:
		PxU32 mTempBufferSize;
		PxU32* mTempBlockSumsGpuPtr;
		PxU32* mTempBlockSumScanGpuPtr;
		PxU32* mTotalSum;
		PxU32 mNumThreadsPerBlock = 0;
		PxU32 mNumElements = 0;

		PxgKernelLauncher* mKernelLauncher;

		void scan(PxU32* inAndOutBuf, PxU32 exclusiveScan, const CUstream& stream, PxU32 numElementsToScan);
		void sumOnly(PxU32* inBuf, const CUstream& stream, PxU32 numElementsToScan);

	public:
		/**
		\brief Empty constructor which allows creating uninitialized objects 
		*/
		PxGpuScan() : mTempBlockSumsGpuPtr(NULL), mTempBlockSumScanGpuPtr(NULL), mTotalSum(NULL) {}

		/**
		\brief Constructor that initializes and allocates all internal data

		\param[in]	cudaContextManager		The cuda context manager
		\param[in]	numElements				The maximum number of elements that can be processed by this gpu scan instance
		\param[in]	numThreadsPerBlock		The number of threads applied per block when scheduling the gpu work
		*/
		PxGpuScan(PxgKernelLauncher* cudaContextManager, PxU32 numElements, PxU32 numThreadsPerBlock = 512);

		/**
		\brief Initializes and allocates all internal data

		\param[in]	cudaContextManager		The cuda context manager
		\param[in]	numElements				The maximum number of elements that can be processed by this gpu scan instance
		\param[in]	numThreadsPerBlock		The number of threads applied per block when scheduling the gpu work
		*/
		bool initialize(PxgKernelLauncher* cudaContextManager, PxU32 numElements, PxU32 numThreadsPerBlock = 512);

		/**
		\brief Allows to access to total sum of all elements that took part in the scan operation

		\return A gpu pointer to the total sum. Only contains valid data after a scan operation finished.
		*/
		PX_FORCE_INLINE PxU32* getSumPointer()
		{
			return mTotalSum;
		}

		/**
		\brief Performs an exclusive scan in place on the given array

		\param[in,out]	inAndOutBuf			Gpu array with the integer data which gets transformed into its exclusive cumulative sum
		\param[in]		stream				Gpu stream on which the calculation is scheduled. To be sure that the scan finished, a synchronize call must be executed on that stream.
		\param[in]		numElementsToScan	Optional: The number of elements that should get scanned. By default all elements are processed. The maximal number of elements is specified in the constructor.
		*/
		PX_FORCE_INLINE void exclusiveScan(PxU32* inAndOutBuf, const CUstream& stream, PxU32 numElementsToScan = 0xFFFFFFFF)
		{
			const PxU32 exclusiveScan = 1;
			scan(inAndOutBuf, exclusiveScan, stream, numElementsToScan);
		}

		/**
		\brief Performs an inclusive scan in place on the given array

		\param[in,out]	inAndOutBuf		Gpu array with the integer data which gets transformed into its inclusive cumulative sum
		\param[in]	stream				Gpu stream on which the calculation is scheduled. To be sure that the scan finished, a synchronize call must be executed on that stream.
		\param[in]	numElementsToScan	The number of elements that should get scanned. By default all elements are processed. The maximal number of elements is specified in the constructor.
		*/
		PX_FORCE_INLINE void inclusiveScan(PxU32* inAndOutBuf, const CUstream& stream, PxU32 numElementsToScan = 0xFFFFFFFF)
		{
			const PxU32 exclusiveScan = 0;
			scan(inAndOutBuf, exclusiveScan, stream, numElementsToScan);
		}

		/**
		\brief Releases all internal data
		*/
		bool release();

		~PxGpuScan() { }
	};	

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
