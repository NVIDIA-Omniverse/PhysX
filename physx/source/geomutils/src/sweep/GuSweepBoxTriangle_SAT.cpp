// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuSweepBoxTriangle_SAT.h"

using namespace physx;
using namespace Gu;

// PT: SAT-based version, in box space
int Gu::triBoxSweepTestBoxSpace(const PxTriangle& tri, const PxVec3& extents, const PxVec3& dir, const PxVec3& oneOverDir, float tmax, float& toi, bool doBackfaceCulling)
{
	return triBoxSweepTestBoxSpace_inlined(tri, extents, dir, oneOverDir, tmax, toi, PxU32(doBackfaceCulling));
}
