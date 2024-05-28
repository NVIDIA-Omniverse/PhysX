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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef CM_ERROR_ACCUMULATOR_H
#define CM_ERROR_ACCUMULATOR_H

#include "foundation/PxVecMath.h"
#include "foundation/PxArray.h"

namespace physx
{
namespace Dy
{
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal calculateResidual(PxReal deltaF, PxReal velocityMultiplier)
	{
		return velocityMultiplier == 0.0f ? 0.0f : deltaF / velocityMultiplier;
	}

	//Vectorized variant
	PX_FORCE_INLINE aos::FloatV calculateResidual(const aos::FloatV& deltaF, const aos::FloatV& velocityMultiplier)
	{
		aos::BoolV isZero = aos::FIsEq(velocityMultiplier, aos::FZero());
		return aos::FSel(isZero, aos::FZero(), aos::FDivFast(deltaF, velocityMultiplier));
	}
	
	PX_FORCE_INLINE aos::Vec4V calculateResidualV4(const aos::Vec4V& deltaF, const aos::Vec4V& velocityMultiplier)
	{
		aos::BoolV isZero = aos::V4IsEq(velocityMultiplier, aos::V4Zero());
		return aos::V4Sel(isZero, aos::V4Zero(), aos::V4DivFast(deltaF, velocityMultiplier));
	}

	struct ErrorAccumulator
	{
		PxReal mErrorSumOfSquares;
		PxI32 mCounter;
		PxReal mMaxError;
		
#if !__CUDACC__
		PX_FORCE_INLINE ErrorAccumulator() : mErrorSumOfSquares(0.0f), mCounter(0), mMaxError(0.0f)
		{ }
#endif

		PX_FORCE_INLINE void combine(ErrorAccumulator& other)
		{
			mErrorSumOfSquares += other.mErrorSumOfSquares;
			mCounter += other.mCounter;
			mMaxError = PxMax(mMaxError, other.mMaxError);
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void accumulateErrorLocal(PxReal residual)
		{
			mErrorSumOfSquares += residual * residual;
			++mCounter;
			mMaxError = PxMax(mMaxError, PxAbs(residual));
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void accumulateErrorLocal(PxReal deltaF, PxReal velocityMultiplier)
		{
			PxReal e = calculateResidual(deltaF, velocityMultiplier);
			accumulateErrorLocal(e);
		}

		//For friction constraints
		PX_FORCE_INLINE void accumulateErrorLocal(PxReal deltaF0, PxReal deltaF1,
			PxReal velocityMultiplier0, PxReal velocityMultiplier1)
		{
			accumulateErrorLocal(deltaF0, velocityMultiplier0);
			accumulateErrorLocal(deltaF1, velocityMultiplier1);
		}	

		PX_FORCE_INLINE void accumulateErrorLocal(const aos::FloatV& deltaF, const aos::FloatV& velocityMultiplier)
		{
			PxReal e;
			aos::FStore(calculateResidual(deltaF, velocityMultiplier), &e);

			mErrorSumOfSquares += e * e;
			++mCounter;
			mMaxError = PxMax(mMaxError, PxAbs(e));
		}

		PX_FORCE_INLINE void accumulateErrorLocal(const aos::FloatV& deltaF0, const aos::FloatV& deltaF1,
			const aos::FloatV& velocityMultiplier0, const aos::FloatV& velocityMultiplier1)
		{
			accumulateErrorLocal(deltaF0, velocityMultiplier0);
			accumulateErrorLocal(deltaF1, velocityMultiplier1);
		}

		//Vectorized variants
		PX_FORCE_INLINE void accumulateErrorLocalV4(const aos::Vec4V& deltaF, const aos::Vec4V& velocityMultiplier)
		{
			aos::BoolV isZero = aos::V4IsEq(velocityMultiplier, aos::V4Zero());
			aos::Vec4V div = aos::V4Sel(isZero,	aos::V4Zero(), aos::V4DivFast(deltaF, velocityMultiplier));
			aos::FloatV dot = aos::V4Dot(div, div);

			PxReal tmp;
			aos::FStore(dot, &tmp);
		
			mErrorSumOfSquares += tmp;
			PxU32 maskNonZero = ~aos::BGetBitMask(isZero);
			mCounter += (maskNonZero & 1) + ((maskNonZero & 2) >> 1) + ((maskNonZero & 4) >> 2) + ((maskNonZero & 8) >> 3);

			aos::FloatV maxVal = aos::V4ExtractMax(aos::V4Abs(div));
			aos::FStore(maxVal, &tmp);
			mMaxError = PxMax(mMaxError, tmp);
		}		

		//For friction constraints
		PX_FORCE_INLINE void accumulateErrorLocalV4(const aos::Vec4V& deltaF0, const aos::Vec4V& deltaF1,
			const aos::Vec4V& velocityMultiplier0, const aos::Vec4V& velocityMultiplier1)
		{
			accumulateErrorLocalV4(deltaF0, velocityMultiplier0);
			accumulateErrorLocalV4(deltaF1, velocityMultiplier1);
		}

		PX_FORCE_INLINE PX_CUDA_CALLABLE void reset()
		{
			mErrorSumOfSquares = 0.0f;
			mCounter = 0;
			mMaxError = 0.0f;
		}

		PX_FORCE_INLINE void accumulateErrorGlobal(Dy::ErrorAccumulator& globalAccumulator)
		{
			globalAccumulator.mErrorSumOfSquares += mErrorSumOfSquares;
			globalAccumulator.mCounter += mCounter;
			if (mMaxError > globalAccumulator.mMaxError)
			{
				globalAccumulator.mMaxError = mMaxError;
			}
		}
	};

	struct ErrorAccumulatorEx
	{
		ErrorAccumulator mPositionIterationErrorAccumulator;
		ErrorAccumulator mVelocityIterationErrorAccumulator;

		PX_FORCE_INLINE void reset()
		{
			mPositionIterationErrorAccumulator.reset();
			mVelocityIterationErrorAccumulator.reset();
		}

		PX_FORCE_INLINE void combine(ErrorAccumulatorEx& other)
		{
			mPositionIterationErrorAccumulator.combine(other.mPositionIterationErrorAccumulator);
			mVelocityIterationErrorAccumulator.combine(other.mVelocityIterationErrorAccumulator);
		}
	};

} // namespace Cm

}

#endif
