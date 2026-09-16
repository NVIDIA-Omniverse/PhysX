// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_TGS_H
#define DY_TGS_H

#include "foundation/PxPreprocessor.h"

namespace physx
{
	struct PxConstraintBatchHeader;
	struct PxSolverConstraintDesc;
	struct PxTGSSolverBodyTxInertia;
	struct PxTGSSolverBodyData;
	struct PxTGSSolverBodyVel;

	namespace Dy
	{
		// PT: using defines like we did in Gu (GU_OVERLAP_FUNC_PARAMS, etc). Additionally this gives a
		// convenient way to find the TGS solver methods, which are scattered in different files and use
		// the same function names as other functions (with a different signature).

		#define DY_TGS_SOLVE_METHOD_PARAMS		const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc, const PxTGSSolverBodyTxInertia* const txInertias, PxReal minPenetration, PxReal elapsedTime
		#define DY_TGS_CONCLUDE_METHOD_PARAMS	const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc, const PxTGSSolverBodyTxInertia* const txInertias, PxReal elapsedTime
		#define DY_TGS_WRITEBACK_METHOD_PARAMS	const PxConstraintBatchHeader& hdr, const PxSolverConstraintDesc* desc

		typedef void (*TGSSolveBlockMethod)		(DY_TGS_SOLVE_METHOD_PARAMS);
		typedef void (*TGSSolveConcludeMethod)	(DY_TGS_CONCLUDE_METHOD_PARAMS);
		typedef void (*TGSWriteBackMethod)		(DY_TGS_WRITEBACK_METHOD_PARAMS);
		
		extern TGSSolveBlockMethod		g_SolveTGSMethods[];
		extern TGSSolveConcludeMethod	g_SolveConcludeTGSMethods[];
		extern TGSWriteBackMethod		g_WritebackTGSMethods[];

		// PT: also used by immediate mode
		void copyToSolverBodyDataStep(const PxVec3& linearVelocity, const PxVec3& angularVelocity, PxReal invMass, const PxVec3& invInertia, const PxTransform& globalPose,
			PxReal maxDepenetrationVelocity, PxReal maxContactImpulse, PxU32 nodeIndex, PxReal reportThreshold,
			PxReal maxAngVelSq, PxU32 lockFlags, bool isKinematic,
			PxTGSSolverBodyVel& solverVel, PxTGSSolverBodyTxInertia& solverBodyTxInertia, PxTGSSolverBodyData& solverBodyData,
			PxReal dt, bool gyroscopicForces);

		// PT: also used by immediate mode
		void integrateCoreStep(PxTGSSolverBodyVel& vel, PxTGSSolverBodyTxInertia& txInertia, PxF32 dt);
	}
}

#endif
