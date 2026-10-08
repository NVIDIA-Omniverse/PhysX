// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_DEFORMABLE_BODY_CORE_H
#define DY_DEFORMABLE_BODY_CORE_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxTransform.h"
#include "foundation/PxArray.h"

namespace physx
{
namespace Dy
{

struct DeformableBodyCore
{
public:
	PxReal					linearDamping;
	PxReal					settlingThreshold;
	PxReal					sleepThreshold;
	PxReal					settlingDamping;
	PxReal					selfCollisionFilterDistance;
	PxReal					selfCollisionStressTolerance;

	PxReal					maxLinearVelocity;
	PxReal					maxPenetrationBias;

	PxU16					solverIterationCounts; //vel iters are in low word and pos iters in high word.
	PxArray<PxU16>			materialHandles;
	PxReal					wakeCounter;

	PxDeformableBodyFlags	bodyFlags;
	PxActorFlags			actorFlags;
	bool					dirty;

	DeformableBodyCore()
		: linearDamping(0.05f)
		, settlingThreshold(0.1f)
		, sleepThreshold(0.05f)
		, settlingDamping(10.f)
		, selfCollisionFilterDistance(0.1f)
		, selfCollisionStressTolerance(0.9f)
		, maxLinearVelocity(PX_MAX_REAL) // see Sc::BodyCore::BodyCore
		, maxPenetrationBias(-1e32f) // see PxsBodyCore::init
		, solverIterationCounts(0)
		, wakeCounter(0)
		, bodyFlags(0)
		, actorFlags(0)
		, dirty(false)
	{
	}
};

} // namespace Dy
} // namespace physx

#endif

