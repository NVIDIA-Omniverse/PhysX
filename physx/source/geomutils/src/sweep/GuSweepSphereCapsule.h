// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_SPHERE_CAPSULE_H
#define GU_SWEEP_SPHERE_CAPSULE_H

#include "foundation/PxVec3.h"
#include "PxQueryReport.h"

namespace physx
{
namespace Gu
{
	class Sphere;
	class Capsule;

	bool sweepSphereCapsule(const Sphere& sphere, const Capsule& lss, const PxVec3& dir, PxReal length, PxReal& d, PxVec3& ip, PxVec3& nrm, PxHitFlags hitFlags);

} // namespace Gu

}

#endif
