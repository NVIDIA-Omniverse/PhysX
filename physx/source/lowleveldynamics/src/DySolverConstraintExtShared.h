// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONSTRAINT_EXT_SHARED_H
#define DY_SOLVER_CONSTRAINT_EXT_SHARED_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxVecMath.h"
#include "DyArticulationContactPrep.h"
#include "DySolverConstraintDesc.h"
#include "DySolverConstraint1D.h"
#include "DySolverContact.h"
#include "PxcNpWorkUnit.h"
#include "PxsMaterialManager.h"

namespace physx
{
	namespace Dy
	{
		FloatV setupExtSolverContact(const SolverExtBody& b0, const SolverExtBody& b1,
			const FloatV& d0, const FloatV& d1, const FloatV& angD0, const FloatV& angD1, const Vec3V& bodyFrame0p, const Vec3V& bodyFrame1p,
			const Vec3VArg normal, const FloatVArg invDt, const FloatVArg invDtWithBiasCoefficient, const FloatVArg dt, 
			const FloatVArg restDistance, const FloatVArg maxPenBias, const FloatVArg restitution,
			const FloatVArg bounceThreshold, const PxContactPoint& contact, SolverContactPointExt& solverContact, const FloatVArg ccdMaxSeparation,
			Cm::SpatialVectorF* Z, const Cm::SpatialVectorV& v0, const Cm::SpatialVectorV& v1, const FloatV& cfm, const Vec3VArg solverOffsetSlop,
			const FloatVArg norVel0, const FloatVArg norVel1, const FloatVArg damping, const BoolVArg accelerationSpring);
	} //namespace Dy
}

#endif
