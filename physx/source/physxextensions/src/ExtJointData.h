// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_JOINT_DATA_H
#define EXT_JOINT_DATA_H

#include "extensions/PxJointLimit.h"

namespace physx
{
namespace Ext
{
	struct JointData
	{
				PxConstraintInvMassScale	invMassScale;
				PxTransform32				c2b[2];
	protected:
		        ~JointData()	{}
	};

} // namespace Ext
}

#endif
