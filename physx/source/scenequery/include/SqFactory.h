// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_FACTORY_H
#define SQ_FACTORY_H

#include "foundation/PxSimpleTypes.h"
#include "GuFactory.h"
#include "SqTypedef.h"

namespace physx
{
namespace Sq
{
	class CompoundPruner;

	CompoundPruner*	createCompoundPruner(PxU64 contextID);
}
}

#endif
