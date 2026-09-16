// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SOLVER_BODY_H
#define DY_SOLVER_BODY_H

#include "foundation/PxVec3.h"
#include "foundation/PxTransform.h"
#include "foundation/PxMat33.h"
#include "CmSpatialVector.h"
#include "solver/PxSolverDefs.h"

namespace physx
{

class PxsRigidBody;
struct PxsBodyCore;

namespace Dy
{

// PT: TODO: make sure this is still needed / replace with V4sqrt
PX_FORCE_INLINE PxVec3 computeSafeSqrtInertia(const PxVec3& v)
{
	return PxVec3(PxSqrt(v.x), PxSqrt(v.y), PxSqrt(v.z));
}

void copyToSolverBodyData(const PxVec3& linearVelocity, const PxVec3& angularVelocity, PxReal invMass, const PxVec3& invInertia, const PxTransform& globalPose,
	PxReal maxDepenetrationVelocity, PxReal maxContactImpulse, PxU32 nodeIndex, PxReal reportThreshold, PxSolverBodyData& solverBodyData, PxU32 lockFlags,
	PxReal dt, bool gyroscopicForces);

// PT: TODO: using PxsBodyCore in the interface makes us write less data to the stack for passing arguments, and we can take advantage of the class layout
// (we know what is aligned or not, we know if it is safe to V4Load vectors, etc). Note that this is what we previously had, which is why PxsBodyCore was still
// forward-referenced above.
//void copyToSolverBodyData(PxSolverBodyData& solverBodyData, const PxsBodyCore& core, const PxU32 nodeIndex);

}

}

#endif
