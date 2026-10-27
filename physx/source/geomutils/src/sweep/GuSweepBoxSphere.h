// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_BOX_SPHERE_H
#define GU_SWEEP_BOX_SPHERE_H

#include "foundation/PxVec3.h"
#include "PxQueryReport.h"

namespace physx
{
namespace Gu
{
	class Box;

	bool sweepBoxSphere(const Box& box, PxReal sphereRadius, const PxVec3& spherePos, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& normal, PxHitFlags hitFlags);

} // namespace Gu

}

#endif
