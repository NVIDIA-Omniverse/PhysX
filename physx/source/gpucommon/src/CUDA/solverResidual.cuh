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


#ifndef __CU_SOLVER_ERROR_CUH__
#define __CU_SOLVER_ERROR_CUH__

#include "DyResidualAccumulator.h"
#include "atomic.cuh"
#include "reduction.cuh"

struct PxgErrorAccumulator
{
	PxReal sumOfSquares;
	PxU32 counter;
	PxReal maxError;

	PX_FORCE_INLINE __device__ PxgErrorAccumulator() : sumOfSquares(0.0f), counter(0), maxError(0.0f)
	{ }

	// Provides a calculateResidual function using fast GPU math instructions
	static PX_FORCE_INLINE __device__ PxReal calculateResidual(PxReal deltaF, PxReal velocityMultiplier)
	{
		return velocityMultiplier == 0.0f ? 0.0f : __fdividef(deltaF, velocityMultiplier);
	}

	PX_FORCE_INLINE __device__ void accumulateErrorLocal(PxReal deltaF, PxReal velocityMultiplier)
	{
		PxReal e = PxgErrorAccumulator::calculateResidual(deltaF, velocityMultiplier);
		sumOfSquares += e * e;
		++counter;
		maxError = PxMax(maxError, PxAbs(e));
	}

	PX_FORCE_INLINE __device__ void accumulateErrorLocal(PxReal deltaF0, PxReal deltaF1,
		PxReal velocityMultiplier0, PxReal velocityMultiplier1)
	{
		accumulateErrorLocal(deltaF0, velocityMultiplier0);
		accumulateErrorLocal(deltaF1, velocityMultiplier1);
	}

	/*PX_FORCE_INLINE __device__ void accumulateErrorGlobal(Dy::ErrorAccumulator& globalAccumulator)
	{
		atomicAdd(&globalAccumulator.mErrorSumOfSquares, sumOfSquares);
		atomicAdd(&globalAccumulator.mCounter, counter);
		if (maxError > globalAccumulator.mMaxError)
			AtomicMax(&globalAccumulator.mMaxError, maxError);
	}*/

	PX_FORCE_INLINE __device__ void accumulateErrorGlobalNoAtomics(Dy::ErrorAccumulator& globalAccumulator)
	{
		globalAccumulator.mErrorSumOfSquares += sumOfSquares;
		globalAccumulator.mCounter += counter;
		if (maxError > globalAccumulator.mMaxError)
			globalAccumulator.mMaxError = maxError;
	}

	PX_FORCE_INLINE __device__ void accumulateErrorGlobalFullWarp(Dy::ErrorAccumulator& globalAccumulator, PxU32 threadIndexInWarp)
	{
		PxReal s = warpReduction<AddOpPxReal, PxReal>(FULL_MASK, sumOfSquares);
		PxU32 count = warpReduction<AddOpPxU32, PxU32>(FULL_MASK, counter);
		PxReal maxErr = warpReduction<MaxOpFloat, PxReal>(FULL_MASK, maxError);
		if (threadIndexInWarp == 0)
		{
			atomicAdd(&globalAccumulator.mErrorSumOfSquares, s);
			atomicAdd(&globalAccumulator.mCounter, count);
			if (maxErr > globalAccumulator.mMaxError)
				AtomicMax(&globalAccumulator.mMaxError, maxErr);
		}
	}
};

#endif
