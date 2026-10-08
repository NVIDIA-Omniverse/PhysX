// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuGJK.h"
#include "GuGJKRaycast.h"
#include "GuGJKPenetration.h"
#include "GuGJKTest.h"

namespace physx
{
namespace Gu
{
using namespace aos;

GjkStatus testGjk(const GjkConvex& a, const GjkConvex& b, const Vec3VArg initialSearchDir, const FloatVArg contactDist, Vec3V& closestA, Vec3V& closestB, Vec3V& normal, FloatV& dist)
{
	return gjk<GjkConvex, GjkConvex>(a, b, initialSearchDir, contactDist, closestA, closestB, normal, dist);
}

bool testGjkRaycast(const GjkConvex& a, const GjkConvex& b, const Vec3VArg initialSearchDir, const aos::FloatVArg initialLambda, const aos::Vec3VArg s, const aos::Vec3VArg r, aos::FloatV& lambda, 
		aos::Vec3V& normal, aos::Vec3V& closestA, PxReal inflation)
{
	return gjkRaycast(a, b, initialSearchDir, initialLambda, s, r, lambda, normal, closestA, inflation);
}

GjkStatus testGjkPenetration(const GjkConvex& a, const GjkConvex& b, const Vec3VArg initialSearchDir, const FloatVArg contactDist,
	PxU8* aIndices, PxU8* bIndices, PxU8& size, GjkOutput& output)
{
	return gjkPenetration<GjkConvex, GjkConvex>(a, b, initialSearchDir, contactDist, true,
		aIndices, bIndices, size, output);
}

GjkStatus testEpaPenetration(const GjkConvex& a, const GjkConvex& b, const PxU8* aIndices, const PxU8* bIndices, PxU8 size, GjkOutput& output)
{
	return epaPenetration(a, b, aIndices, bIndices, size, true, aos::FLoad(1.f), output);
}

}
}

