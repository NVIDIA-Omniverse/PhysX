// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_CAPSULE_CAPSULE_H
#define GU_SWEEP_CAPSULE_CAPSULE_H

#include "foundation/PxVec3.h"

namespace physx
{
namespace Gu
{
	class Capsule;

	bool sweepCapsuleCapsule(const Capsule& capsule0, const Capsule& capsule1, const PxVec3& dir, PxReal length, PxReal& min_dist, PxVec3& ip, PxVec3& normal, PxU32 inHitFlags, PxU16& outHitFlags);

} // namespace Gu

}

#endif
