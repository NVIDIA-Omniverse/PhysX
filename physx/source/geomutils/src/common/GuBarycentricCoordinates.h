// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BARYCENTRIC_COORDINATES_H
#define GU_BARYCENTRIC_COORDINATES_H

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxVecMath.h"

namespace physx
{
namespace Gu
{
	//calculate the barycentric coorinates for a point in a segment
	void barycentricCoordinates(const aos::Vec3VArg p, 
		const aos::Vec3VArg a, 
		const aos::Vec3VArg b, 
		aos::FloatV& v);

	//calculate the barycentric coorinates for a point in a triangle
	void barycentricCoordinates(const aos::Vec3VArg p, 
		const aos::Vec3VArg a, 
		const aos::Vec3VArg b, 
		const aos::Vec3VArg c, 
		aos::FloatV& v, 
		aos::FloatV& w);

	void barycentricCoordinates(const aos::Vec3VArg v0, 
		const aos::Vec3VArg v1, 
		const aos::Vec3VArg v2,
		aos::FloatV& v, 
		aos::FloatV& w);

	PX_INLINE aos::BoolV isValidTriangleBarycentricCoord(const aos::FloatVArg v, const aos::FloatVArg w)
	{
		using namespace aos;
		const FloatV zero = FNeg(FEps());
		const FloatV one = FAdd(FOne(), FEps());

		const BoolV con0 = BAnd(FIsGrtrOrEq(v, zero), FIsGrtrOrEq(one, v));
		const BoolV con1 = BAnd(FIsGrtrOrEq(w, zero), FIsGrtrOrEq(one, w));
		const BoolV con2 = FIsGrtr(one, FAdd(v, w));
		return BAnd(con0, BAnd(con1, con2));
	}

	PX_INLINE aos::BoolV isValidTriangleBarycentricCoord2(const aos::Vec4VArg vwvw)
	{
		using namespace aos;
		const Vec4V eps = V4Splat(FEps());
		const Vec4V zero = V4Neg(eps);
		const Vec4V one = V4Add(V4One(), eps);

		const Vec4V v0v1v0v1 = V4PermXZXZ(vwvw);
		const Vec4V w0w1w0w1 = V4PermYWYW(vwvw);

		const BoolV con0 = BAnd(V4IsGrtrOrEq(v0v1v0v1, zero), V4IsGrtrOrEq(one, v0v1v0v1));
		const BoolV con1 = BAnd(V4IsGrtrOrEq(w0w1w0w1, zero), V4IsGrtrOrEq(one, w0w1w0w1));
		const BoolV con2 = V4IsGrtr(one, V4Add(v0v1v0v1, w0w1w0w1));
		return BAnd(con0, BAnd(con1, con2));
	}

} // namespace Gu

}

#endif
