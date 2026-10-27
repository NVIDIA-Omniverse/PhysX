// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_GJK_TEST_H
#define GU_GJK_TEST_H

#include "common/PxPhysXCommonConfig.h"
#include "GuGJKUtil.h"

namespace physx
{
namespace Gu
{
	struct GjkConvex;

	PX_PHYSX_COMMON_API GjkStatus testGjk(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialSearchDir, const aos::FloatVArg contactDist, aos::Vec3V& closestA, aos::Vec3V& closestB,
		aos::Vec3V& normal, aos::FloatV& dist);
	
	PX_PHYSX_COMMON_API	bool testGjkRaycast(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialSearchDir, const aos::FloatVArg initialLambda, const aos::Vec3VArg s, const aos::Vec3VArg r, 
		aos::FloatV& lambda, aos::Vec3V& normal, aos::Vec3V& closestA, PxReal inflation);

	PX_PHYSX_COMMON_API GjkStatus testGjkPenetration(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialSearchDir, const aos::FloatVArg contactDist,
		PxU8* aIndices, PxU8* bIndices, PxU8& size, GjkOutput& output);

	PX_PHYSX_COMMON_API GjkStatus testEpaPenetration(const GjkConvex& a, const GjkConvex& b, const PxU8* aIndices, const PxU8* bIndices, PxU8 size, GjkOutput& output);
}
}

#endif
