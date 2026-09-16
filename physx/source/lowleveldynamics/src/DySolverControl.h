// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_CONTROL_H
#define DY_SOLVER_CONTROL_H

#include "DySolverCore.h"

namespace physx
{
namespace Dy
{

/*
solves dual problem exactly by GS-iterating until convergence stops
only uses regular velocity vector for storing results, and backs up initial state, which is restored.
the solution forces are saved in a vector.

state should not be stored, this function is safe to call from multiple threads.
*/
void solveVParallelAndWriteBack(SolverIslandParams& params, Cm::SpatialVectorF* deltaV, PxReal articulationBiasCoefficient,
	bool solveFrictionEveryIteration, bool solveArticulationContactLast);

void solveV_Blocks(SolverIslandParams& params, PxReal articulationBiasCoefficient, bool solveFrictionEveryIteration, bool solveArticulationContactLast);

}
}

#endif
