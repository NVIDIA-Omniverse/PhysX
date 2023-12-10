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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef DY_SOLVER_CONTROL_H
#define DY_SOLVER_CONTROL_H

#include "DySolverCore.h"
#include "DySolverConstraintDesc.h"

namespace physx
{
namespace Dy
{

class BatchIterator
{
	PX_NOCOPY(BatchIterator)
public:
	PxConstraintBatchHeader* constraintBatchHeaders;
	PxU32 mSize;
	PxU32 mCurrentIndex;

	BatchIterator(PxConstraintBatchHeader* _constraintBatchHeaders, PxU32 size) : constraintBatchHeaders(_constraintBatchHeaders),
		mSize(size), mCurrentIndex(0)
	{
	}

	PX_FORCE_INLINE const PxConstraintBatchHeader& GetCurrentHeader(const PxU32 constraintIndex)
	{
		PxU32 currentIndex = mCurrentIndex;
		while((constraintIndex - constraintBatchHeaders[currentIndex].startIndex) >= constraintBatchHeaders[currentIndex].stride)
			currentIndex = (currentIndex + 1)%mSize;
		PxPrefetchLine(&constraintBatchHeaders[currentIndex], 128);
		mCurrentIndex = currentIndex;
		return constraintBatchHeaders[currentIndex];
	}
};

inline void SolveBlockParallel(	PxSolverConstraintDesc* PX_RESTRICT constraintList, const PxI32 batchCount, const PxI32 index,  
								 const PxI32 headerCount, SolverContext& cache, BatchIterator& iterator,
								 SolveBlockMethod solveTable[],
								 const PxI32 iteration)
{
	const PxI32 indA = index - (iteration * headerCount);

	const PxConstraintBatchHeader* PX_RESTRICT headers = iterator.constraintBatchHeaders;

	const PxI32 endIndex = indA + batchCount;
	for(PxI32 i = indA; i < endIndex; ++i)
	{
		PX_ASSERT(i < PxI32(iterator.mSize));
		const PxConstraintBatchHeader& header = headers[i];

		const PxI32 numToGrab = header.stride;
		PxSolverConstraintDesc* PX_RESTRICT block = &constraintList[header.startIndex];

		// PT: TODO: revisit this one
		PxPrefetch(block[0].constraint, 384);

		for(PxI32 b = 0; b < numToGrab; ++b)
		{
			PxPrefetchLine(block[b].bodyA);
			PxPrefetchLine(block[b].bodyB);
		}

		//OK. We have a number of constraints to run...
		solveTable[header.constraintType](block, PxU32(numToGrab), cache);
	}
}

// PT: TODO: these "solver core" classes are mostly stateless, at this point they could just be function pointers like the solve methods.
class SolverCoreGeneral : public SolverCore
{
public:
	bool mFrictionEveryIteration;
	SolverCoreGeneral(bool fricEveryIteration) : mFrictionEveryIteration(fricEveryIteration)	{}

	// SolverCore
	virtual void solveVParallelAndWriteBack
		(SolverIslandParams& params, Cm::SpatialVectorF* Z, Cm::SpatialVectorF* deltaV) const	PX_OVERRIDE	PX_FINAL;

	virtual void solveV_Blocks
		(SolverIslandParams& params) const	PX_OVERRIDE	PX_FINAL;
	//~SolverCore
};

// PT: TODO: we use "extern" instead of functions for TGS. Unify.
SolveBlockMethod* getSolveBlockTable();
SolveBlockMethod* getSolverConcludeBlockTable();
SolveWriteBackBlockMethod* getSolveWritebackBlockTable();

}
}

#endif
