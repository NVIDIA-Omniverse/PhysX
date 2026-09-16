// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_CAPSULE_BOX_H
#define GU_SWEEP_CAPSULE_BOX_H

#include "foundation/PxVec3.h"
#include "PxQueryReport.h"

namespace physx
{
namespace Gu
{
	class Capsule;

	bool sweepCapsuleBox(const Capsule& capsule, const PxTransform& boxWorldPose, const PxVec3& boxDim, const PxVec3& dir, PxReal length, PxVec3& hit, PxReal& min_dist, PxVec3& normal, PxHitFlags hitFlags);

} // namespace Gu

}

#endif
