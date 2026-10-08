// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
