// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_SPHERE_SPHERE_H
#define GU_SWEEP_SPHERE_SPHERE_H

#include "foundation/PxVec3.h"

namespace physx
{
namespace Gu
{
	bool sweepSphereSphere(const PxVec3& center0, PxReal radius0, const PxVec3& center1, PxReal radius1, const PxVec3& motion, PxReal& d, PxVec3& nrm);

} // namespace Gu

}

#endif
