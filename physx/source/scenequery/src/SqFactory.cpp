// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "SqFactory.h"
#include "SqCompoundPruner.h"

using namespace physx;
using namespace Sq;

CompoundPruner* physx::Sq::createCompoundPruner(PxU64 contextID)
{
	return PX_NEW(BVHCompoundPruner)(contextID);
}

