// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CACHED_FUNCS_H
#define GU_CACHED_FUNCS_H

#include "GuRaycastTests.h"
#include "GuSweepTests.h"
#include "GuOverlapTests.h"

namespace physx
{
namespace Gu
{
	struct CachedFuncs
	{
		CachedFuncs() :
			mCachedRaycastFuncs	(Gu::getRaycastFuncTable()),
			mCachedSweepFuncs	(Gu::getSweepFuncTable()),
			mCachedOverlapFuncs	(Gu::getOverlapFuncTable())
		{
		}

		const Gu::GeomRaycastTable&	mCachedRaycastFuncs;
		const Gu::GeomSweepFuncs&	mCachedSweepFuncs;
		const Gu::GeomOverlapTable*	mCachedOverlapFuncs;

		PX_NOCOPY(CachedFuncs)
	};
}
}

#endif
