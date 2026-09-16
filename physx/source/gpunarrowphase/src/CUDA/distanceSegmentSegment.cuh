// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DIS_SEG_SEG_CUH__
#define __DIS_SEG_SEG_CUH__

namespace physx
{

__device__ __forceinline__ static
void distanceSegmentSegmentSquared(
	const PxVec3& p1, const PxVec3& d1,
	const PxVec3& p2, const PxVec3& d2,
	PxReal& s, PxReal& t)
{
	//const FloatV zero = FZero();
	//const FloatV one = FOne();
	//const FloatV eps = FEps();

	const PxReal eps = 1e-6f;
	const PxVec3 r = p1 - p2;
	const PxReal a = d1.dot(d1);
	const PxReal e = d2.dot(d2);
	const PxReal b = d1.dot(d2);
	const PxReal c = d1.dot(r);
	const PxReal aRecip = a > eps ? (1.f / a) : 0.f;
	const PxReal eRecip = e > eps ? (1.f / e) : 0.f;
	const PxReal f = d2.dot(r);

	/*
		s = (b*f - c*e)/(a*e - b*b);
		t = (a*f - b*c)/(a*e - b*b);

		s = (b*t - c)/a;
		t = (b*s + f)/e;
	*/

	//if segments not parallel, the general non-degenerated case, compute closest point on two segments and clamp to segment1
	const PxReal denom = a * e - b * b;
	const PxReal temp = b * f - c * e;
	const PxReal s0 = PxClamp(temp / denom, 0.f, 1.f);

	//if segment is parallel, demon < eps
	//const bool con2 = eps > denom;
	const PxReal sTmp = eps > denom ? 0.5f : s0;
	//const BoolV con2 = FIsGrtr(eps, denom);//FIsEq(denom, zero);
	//const FloatV sTmp = FSel(con2, FHalf(), s0);

	//compute point on segment2 closest to segment1
	const PxReal tTmp = (b * sTmp + f) * eRecip;
	//const FloatV tTmp = FMul(FScaleAdd(b, sTmp, f), eRecip);

	//if t is in [zero, one], done. otherwise clamp t
	const PxReal t2 = PxClamp(tTmp, 0.f, 1.f);
	//const FloatV t2 = FClamp(tTmp, zero, one);

	//recompute s for the new value
	const PxReal comp = (b * t2 - c) * aRecip;
	const PxReal s2 = PxClamp(comp, 0.f, 1.f);
	//const FloatV comp = FMul(FSub(FMul(b, t2), c), aRecip);
	//const FloatV s2 = FClamp(comp, zero, one);

	s = s2;
	t = t2;
}

} // namespace physx

#endif