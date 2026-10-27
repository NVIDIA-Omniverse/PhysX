// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SQ_TYPEDEF_H
#define SQ_TYPEDEF_H

#include "foundation/PxSimpleTypes.h"
#include "GuPrunerTypedef.h"

namespace physx
{
namespace Sq
{
	typedef PxU32 PrunerCompoundId;
	static const PrunerCompoundId INVALID_COMPOUND_ID = 0xffffffff;

	typedef PxU32	PrunerData;
	#define SQ_INVALID_PRUNER_DATA	0xffffffff
}
}

#endif
