// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_PGS_H
#define DY_PGS_H

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"

namespace physx
{
	struct PxSolverConstraintDesc;

	namespace Dy
	{
		struct SolverContext;

		// PT: using defines like we did in Gu (GU_OVERLAP_FUNC_PARAMS, etc). Additionally this gives a
		// convenient way to find the PGS solver methods, which are scattered in different files and use
		// the same function names as other functions (with a different signature).

		#define DY_PGS_SOLVE_METHOD_PARAMS	const PxSolverConstraintDesc* desc, PxU32 constraintCount, SolverContext& cache

		typedef void (*SolveBlockMethod)			(DY_PGS_SOLVE_METHOD_PARAMS);
		typedef void (*SolveWriteBackBlockMethod)	(DY_PGS_SOLVE_METHOD_PARAMS);
		typedef void (*WriteBackBlockMethod)		(DY_PGS_SOLVE_METHOD_PARAMS);

		extern SolveBlockMethod				gVTableSolveBlock[];
		extern SolveWriteBackBlockMethod	gVTableSolveWriteBackBlock[];
		extern SolveBlockMethod				gVTableSolveConcludeBlock[];
	}
}

#endif
