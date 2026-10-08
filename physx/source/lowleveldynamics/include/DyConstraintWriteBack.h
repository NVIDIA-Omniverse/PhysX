// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_CONSTRAINT_WRITE_BACK_H
#define DY_CONSTRAINT_WRITE_BACK_H

#include "foundation/PxVec3.h"
#include "PxPhysXConfig.h"
#include "PxvDynamics.h"

namespace physx
{
	namespace Dy
	{
		PX_ALIGN_PREFIX(16)
		struct ConstraintWriteback
		{
			void initialize()
			{
				linearImpulse = PxVec3(0);
				angularImpulse = PxVec3(0);
				broken = false;
			}

			PxVec3	linearImpulse;
			PxU32	broken;
			PxVec3	angularImpulse;
			PxU32	pad;
		}
		PX_ALIGN_SUFFIX(16);

	}
}

#endif
