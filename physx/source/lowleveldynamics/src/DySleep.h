// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_SLEEP_H
#define DY_SLEEP_H

#include "PxvDynamics.h"
#include "PxsRigidBody.h"
#include "DySleepingConfigulation.h"

namespace physx
{
namespace Dy
{
	void sleepCheck(PxsRigidBody* originalBody, PxReal dt, bool enableStabilization, const Cm::SpatialVector& motionVelocity, PxIntBool hasStaticTouch);
}
}

#endif
