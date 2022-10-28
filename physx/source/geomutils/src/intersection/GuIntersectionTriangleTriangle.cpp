// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuIntersectionTriangleTriangle.h"
#include "foundation/PxPlane.h"

namespace physx
{
namespace Gu
{

	//Based on the paper A Fast Triangle-Triangle Intersection Test by T. Moeller
	//http://web.stanford.edu/class/cs277/resources/papers/Moller1997b.pdf
	struct Interval
	{
		PxReal min;
		PxReal max;
		PxVec3 minPoint;
		PxVec3 maxPoint;

		PX_FORCE_INLINE Interval() : min(FLT_MAX), max(-FLT_MAX), minPoint(PxVec3(NAN)), maxPoint(PxVec3(NAN)) { }

		PX_FORCE_INLINE static bool overlapOrTouch(const Interval& a, const Interval& b)
		{
			return !(a.min > b.max || b.min > a.max);
		}

		PX_FORCE_INLINE static Interval intersection(const Interval& a, const Interval& b)
		{
			Interval result;
			if (!overlapOrTouch(a, b))
				return result;

			if (a.min > b.min)
			{
				result.min = a.min;
				result.minPoint = a.minPoint;
			}
			else
			{
				result.min = b.min;
				result.minPoint = b.minPoint;
			}

			if (a.max < b.max)
			{
				result.max = a.max;
				result.maxPoint = a.maxPoint;
			}
			else
			{
				result.max = b.max;
				result.maxPoint = b.maxPoint;
			}
			return result;
		}

		PX_FORCE_INLINE void include(PxReal d, const PxVec3& p)
		{
			if (d < min) { min = d; minPoint = p; }
			if (d > max) { max = d; maxPoint = p; }
		}
	};

	static Interval computeInterval(PxReal distanceA, PxReal distanceB, PxReal distanceC, const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& dir)
	{
		Interval i;

		const bool bA = distanceA > 0;
		const bool bB = distanceB > 0;
		const bool bC = distanceC > 0;
		distanceA = PxAbs(distanceA);
		distanceB = PxAbs(distanceB);
		distanceC = PxAbs(distanceC);

		if (bA != bB)
		{
			const PxVec3 p = (distanceA / (distanceA + distanceB)) * b + (distanceB / (distanceA + distanceB)) * a;
			i.include(dir.dot(p), p);
		}
		if (bA != bC)
		{
			const PxVec3 p = (distanceA / (distanceA + distanceC)) * c + (distanceC / (distanceA + distanceC)) * a;
			i.include(dir.dot(p), p);
		}
		if (bB != bC)
		{
			const PxVec3 p = (distanceB / (distanceB + distanceC)) * c + (distanceC / (distanceB + distanceC)) * b;
			i.include(dir.dot(p), p);
		}

		return i;
	}

	bool trianglesIntersect(const PxVec3& a1, const PxVec3& b1, const PxVec3& c1, const PxVec3& a2, const PxVec3& b2, const PxVec3& c2, Segment* intersection)
	{
		const PxPlane p1(a1, b1, c1);
		const PxReal p1ToA = p1.distance(a2);
		const PxReal p1ToB = p1.distance(b2);
		const PxReal p1ToC = p1.distance(c2);

		if ((p1ToA > 0) == (p1ToB > 0) && (p1ToA > 0) == (p1ToC > 0))
			return false;

		const PxPlane p2(a2, b2, c2);
		const PxReal p2ToA = p2.distance(a1);
		const PxReal p2ToB = p2.distance(b1);
		const PxReal p2ToC = p2.distance(c1);

		if ((p2ToA > 0) == (p2ToB > 0) && (p2ToA > 0) == (p2ToC > 0))
			return false;

		PxVec3 intersectionDirection = p1.n.cross(p2.n);
		const PxReal l2 = intersectionDirection.magnitudeSquared();
		if (l2 < 1e-12f)
			return false; //Coplanar triangles

		intersectionDirection *= 1.0f / PxSqrt(l2);

		const Interval i1 = computeInterval(p2ToA, p2ToB, p2ToC, a1, b1, c1, intersectionDirection);
		const Interval i2 = computeInterval(p1ToA, p1ToB, p1ToC, a2, b2, c2, intersectionDirection);

		if (Interval::overlapOrTouch(i1, i2))
		{
			if (intersection)
			{
				const Interval i = Interval::intersection(i1, i2);
				intersection->p0 = i.minPoint;
				intersection->p1 = i.maxPoint;
			}
			return true;
		}
		return false;
	}
}
}
