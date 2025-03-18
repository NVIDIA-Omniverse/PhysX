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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXG_INTERPOLATION_H
#define PXG_INTERPOLATION_H


#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxVec4.h"
#include "foundation/PxMat44.h"
#include "foundation/PxMathUtils.h"

#if !PX_DOXYGEN
namespace physx
{
#endif
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxQuat rotateAroundAxis(const PxQuat& q, PxVec3 rotationAxis, PxReal angle)
	{
		PxReal mag2 = rotationAxis.magnitudeSquared();
		if (mag2 < 1e-8f || angle < 1e-3f)
			return q;
		rotationAxis *= 1.0f / PxSqrt(mag2);
		const PxQuat rot(angle, rotationAxis);
		return rot * q;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal getT(PxReal t, PxReal alpha, const PxVec3& p0, const PxVec3& p1)
	{
		PxVec3 d = p1 - p0;
		PxReal a = d.magnitudeSquared();
		PxReal b = PxPow(a, alpha * 0.5f);
		return (b + t);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal lerp(PxReal p0, PxReal p1, PxReal t)
	{
		return (1.0f - t) * p0 + t * p1;
	}

	//https://en.wikipedia.org/wiki/Centripetal_CatmullRom_spline
	inline PX_CUDA_CALLABLE PxVec3 evaluateSpline(const PxVec3& p0, const PxVec3& p1, const PxVec3& p2,
		const PxVec3& p3, PxReal t, PxReal alpha = 0.5f)
	{
		PxReal t0 = 0.0f;
		PxReal t1 = getT(t0, alpha, p0, p1);
		PxReal t2 = getT(t1, alpha, p1, p2);
		PxReal t3 = getT(t2, alpha, p2, p3);
		t = lerp(t1, t2, t);
		PxVec3 A1 = ((t1 - t)  * p0 + (t - t0)  * p1) * (1.0f / PxMax(0.0001f, t1 - t0));
		PxVec3 A2 = ((t2 - t) * p1 + (t - t1) * p2)* (1.0f / PxMax(0.0001f, t2 - t1));
		PxVec3 A3 = ((t3 - t) * p2 + (t - t2) * p3)* (1.0f / PxMax(0.0001f, t3 - t2));
		PxVec3 B1 = ((t2 - t) * A1 + (t - t0) * A2)* (1.0f / PxMax(0.0001f, t2 - t0));
		PxVec3 B2 = ((t3 - t) * A2 + (t - t1) * A3)* (1.0f / PxMax(0.0001f, t3 - t1));
		PxVec3 C = ((t2 - t) * B1 + (t - t1) * B2)* (1.0f / PxMax(0.0001f, t2 - t1));
		return C;
	}

	inline PX_CUDA_CALLABLE PxVec3 evaluateSpline(const PxVec3 controlPoints[4], PxU32 numControlPoints, PxReal t, PxReal alpha = 0.5f)
	{
		if (numControlPoints == 1)
			return controlPoints[0];
		return evaluateSpline(controlPoints[0], controlPoints[1], controlPoints[2], controlPoints[3], t, alpha);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxVec3 PxLoad3(const PxVec4& v)
	{
		PxVec4 tmp = v;
		return PxVec3(tmp.x, tmp.y, tmp.z);
	}

	template<typename V4>
	inline PX_CUDA_CALLABLE PxVec3 load3(const V4& val, const PxMat44& transf)
	{
		return transf.transform(PxLoad3(val));
	}

	//Handles boundary case and forwards to standard evaluateSpline
	template<typename V4>
	inline PX_CUDA_CALLABLE PxVec3 evaluateSpline(const V4* points, PxU32 nbPoints, PxU32 index, PxReal t, const PxMat44& transf, PxReal alpha = 0.5f)
	{
		if (nbPoints == 0)
			return PxVec3(0.0f);
		if (nbPoints < 2)
			return load3(points[0], transf);
		if (index > nbPoints - 2)
			index = nbPoints - 2;
		PxVec3 p0;
		if (index == 0)
			p0 = load3(points[0], transf) - (load3(points[1], transf) - load3(points[0], transf));
		else
			p0 = load3(points[index - 1], transf);
		PxVec3 p3;
		if (index == nbPoints - 2)
			p3 = load3(points[nbPoints - 1], transf) + (load3(points[nbPoints - 1], transf) - load3(points[nbPoints - 2], transf));
		else
			p3 = load3(points[index + 2], transf);
		return evaluateSpline(p0, load3(points[index], transf), load3(points[index + 1], transf), p3, t, alpha);
	}

	template<typename V4>
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 catmullRomFetchLastPoint(const V4* points, PxU32 nbPoints, PxU32 index, PxVec3& controlPoint, const PxMat44& transf)
	{
		if (nbPoints == 0)
		{
			controlPoint = PxVec3(0.0f);
			return 1;
		}
		if (nbPoints < 2)
		{
			controlPoint = load3(points[0], transf);
			return 1;
		}
		if (index >= nbPoints - 2)
		{
			PxVec3 p = load3(points[nbPoints - 1], transf);
			controlPoint = p + (p - load3(points[nbPoints - 2], transf));
		}
		else
			controlPoint = load3(points[index + 2], transf);
		return 4;
	}

	inline PX_CUDA_CALLABLE void insertNextControlPoint(PxVec3 controlPoints[4], const PxVec3& nextControlPoint)
	{
		controlPoints[0] = controlPoints[1];
		controlPoints[1] = controlPoints[2];
		controlPoints[2] = controlPoints[3];
		controlPoints[3] = nextControlPoint;
	}

	template<typename V4>
	inline PX_CUDA_CALLABLE PxU32 catmullRomFetchControlPoints(const V4* points, PxU32 nbPoints, PxU32 index, PxVec3 controlPoints[4], const PxMat44& transf)
	{
		if (nbPoints == 0)
		{
			controlPoints[0] = PxVec3(0.0f);
			return 1;
		}
		if (nbPoints < 2)
		{
			controlPoints[0] = load3(points[0], transf);
			return 1;
		}
		if (index > nbPoints - 2)
			index = nbPoints - 2;
		if (index <= 0)
		{
			PxVec3 p = load3(points[0], transf);
			controlPoints[0] = p - (load3(points[1], transf) - p);
		}
		else
			controlPoints[0] = load3(points[index - 1], transf);
		if (index >= nbPoints - 2)
		{
			PxVec3 p = load3(points[nbPoints - 1], transf);
			controlPoints[3] = p + (p - load3(points[nbPoints - 2], transf));
		}
		else
			controlPoints[3] = load3(points[index + 2], transf);
		controlPoints[1] = load3(points[index], transf);
		controlPoints[2] = load3(points[index + 1], transf);
		return 4;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal encodeStrandLocation(PxReal uniform, PxU32 segmentIndexInStrand, bool markAsLast)
	{
		PX_ASSERT(segmentIndexInStrand > 0);		
		uniform = PxClamp(uniform, 0.0f, 0.999f); //Avoid the uniform to be smaller 0 or exactly 1 because then fractional and integer part cannot be separated reliably
		return (markAsLast ? -1.0f : 1.0f) * (uniform + segmentIndexInStrand);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxReal decodeStrandLocation(PxReal strandValue, PxU32& segmentIndexInStrand, bool& isLast)
	{
		isLast = strandValue < 0.0f;
		strandValue = PxAbs(strandValue);
		segmentIndexInStrand = PxU32(strandValue);
		return strandValue - segmentIndexInStrand;
	}



#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
