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

#ifndef PXG_AGGREGATE_H
#define PXG_AGGREGATE_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "PxgCudaBuffer.h"
#include "foundation/PxUserAllocated.h"

namespace physx
{
	class PxgHeapMemoryAllocatorManager;
	class PxgSapBox1D;

	// PT: TODO:
	// - fix/unify class member names (m prefix)
	// - figure out why we don't initialize all members in ctor
	// - check that all of these are still needed
	struct PxgAggregate
	{
	public:
		PxgAggregate() : updateBoundIndices(NULL)
		{
			initValues();
		}

		PX_FORCE_INLINE void initValues()
		{
			prevComparisons = 0;
			prevSize = 0;
			mIndex = 0xFFFFFFFF; 
			size = 0;
			filterHint = 0;
			mEnvID = PX_INVALID_U32;
			isNew = true;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE bool isValid() { return (size > 0) && (mIndex != 0xFFFFFFFF); }

		// resetting back to constructor values.
		PX_FORCE_INLINE void reset()
		{
			PX_FREE(updateBoundIndices);

			initValues();
		}

		PxU32* updateBoundIndices; // list of bound indices for this aggregate, used for CPU->GPU transfer.

		// pointers to device mem containing a list of shape bound indices belong to this aggregate, double-buffered.
		// Will contain the previous and current list, we flip these pointers each time the aggregate is processed and sorted.
		PxU32* boundIndices[2]; 
		
		//A list of the shape projections. There are 2x sorted lists (the prev sorted list, and the current). We flip these pointers
		//each time the pair is processed and sorted...
		PxU32* sortedProjections[2];
		PxU32* sortedHandles[2];
		PxgSapBox1D* sapBox1D[2]; //Prev/curr sapBox...
		PxU32* startMasks[2]; //Prev/curr start masks. This is a bitmask so we require nbProjections/32 PxU32s.
		PxU32* comparisons[2];
		PxU32  prevComparisons;
		PxU32  prevSize; //Can be different from size. Copied to every frame...
		PxU32  mIndex; //aggregate bound index
		PxU32  size;
		PxU32  filterHint; //PxAggregateFilterHint
		PxU32	mEnvID;	// PT: environment ID
		bool isNew;
	};


	struct PxgAggregatePair
	{
	public:
		PxU32 actorHandle0;
		PxU32 actorHandle1;
		bool isNew;
		bool isDead;
		PxU16 pad;
	};

	class PxgAggregateBuffer : public PxUserAllocated
	{
	public:
		PxgAggregateBuffer(PxgHeapMemoryAllocatorManager* heapMemoryManager);

		PxgCudaBuffer		updateBoundIndices;
		PxgCudaBufferN<2>	boundIndices;
		PxgCudaBufferN<2>	sortedProjections;
		PxgCudaBufferN<2>	sortedHandles;
		PxgCudaBufferN<2>	sapBox1D;
		PxgCudaBufferN<2>	startMasks;
		PxgCudaBufferN<2>	comparisons;
	};

}

#endif