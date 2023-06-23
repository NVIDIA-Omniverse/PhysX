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

#ifndef DY_TGS_CONTACT_PREP_H
#define DY_TGS_CONTACT_PREP_H

#include "foundation/PxPreprocessor.h"
#include "DySolverConstraintDesc.h"
#include "PxSceneDesc.h"
#include "DySolverContact4.h"

namespace physx
{
	struct PxsContactManagerOutput;
	struct PxSolverConstraintDesc;

	namespace Dy
	{
		class ThreadContext;
		struct CorrelationBuffer;

		bool createFinalizeSolverContactsStep(PxTGSSolverContactDesc& contactDesc,
			PxsContactManagerOutput& output,
			ThreadContext& threadContext,
			const PxReal invDtF32,
			const PxReal invTotalDtF32,
			const PxReal totalDtF32,
			const PxReal stepDt,
			const PxReal bounceThresholdF32,
			const PxReal frictionOffsetThreshold,
			const PxReal correlationDistance,
			const PxReal biasCoefficient,
			PxConstraintAllocator& constraintAllocator);

		bool createFinalizeSolverContactsStep(
			PxTGSSolverContactDesc& contactDesc,
			CorrelationBuffer& c,
			const PxReal invDtF32,
			const PxReal invTotalDtF32,
			const PxReal totalDtF32,
			const PxReal dtF32,
			const PxReal bounceThresholdF32,
			const PxReal frictionOffsetThreshold,
			const PxReal correlationDistance,
			const PxReal biasCoefficient,
			PxConstraintAllocator& constraintAllocator);

		SolverConstraintPrepState::Enum setupSolverConstraintStep4
		(PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
			const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
			PxConstraintAllocator& allocator, PxU32 maxRows,
			const PxReal lengthScale, const PxReal biasCoefficient);

		PxU32 SetupSolverConstraintStep(SolverConstraintShaderPrepDesc& shaderDesc,
			PxTGSSolverConstraintPrepDesc& prepDesc,
			PxConstraintAllocator& allocator,
			const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt,
			const PxReal lengthScale, const PxReal biasCoefficient);

		PxU32 setupSolverConstraintStep(
			const PxTGSSolverConstraintPrepDesc& prepDesc,
			PxConstraintAllocator& allocator,
			const PxReal dt, const PxReal totalDt, const PxReal invdt, const PxReal invTotalDt,
			const PxReal lengthScale, const PxReal biasCoefficient);

		SolverConstraintPrepState::Enum setupSolverConstraintStep4
		(SolverConstraintShaderPrepDesc* PX_RESTRICT constraintShaderDescs,
			PxTGSSolverConstraintPrepDesc* PX_RESTRICT constraintDescs,
			const PxReal dt, const PxReal totalDt, const PxReal recipdt, const PxReal recipTotalDt, PxU32& totalRows,
			PxConstraintAllocator& allocator, const PxReal lengthScale, const PxReal biasCoefficient);

		SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
			PxsContactManagerOutput** cmOutputs,
			ThreadContext& threadContext,
			PxTGSSolverContactDesc* blockDescs,
			const PxReal invDtF32,
			const PxReal totalDtF32,
			const PxReal invTotalDtF32,
			const PxReal dt,
			const PxReal bounceThresholdF32,
			const PxReal frictionOffsetThreshold,
			const PxReal correlationDistance,
			const PxReal biasCoefficient,
			PxConstraintAllocator& constraintAllocator);

		SolverConstraintPrepState::Enum createFinalizeSolverContacts4Step(
			Dy::CorrelationBuffer& c,
			PxTGSSolverContactDesc* blockDescs,
			const PxReal invDtF32,
			const PxReal totalDt,
			const PxReal invTotalDtF32,
			const PxReal dt,
			const PxReal bounceThresholdF32,
			const PxReal frictionOffsetThreshold,
			const PxReal correlationDistance,
			const PxReal biasCoefficient,
			PxConstraintAllocator& constraintAllocator);
	}
}

#endif
