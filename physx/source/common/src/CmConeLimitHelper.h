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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef CM_CONE_LIMIT_HELPER_H
#define CM_CONE_LIMIT_HELPER_H

// This class contains methods for supporting the tan-quarter swing limit - that
// is the, ellipse defined by tanQ(theta)^2/tanQ(thetaMax)^2 + tanQ(phi)^2/tanQ(phiMax)^2 = 1
// 
// Angles are passed as an PxVec3 swing vector with x = 0 and y and z the swing angles
// around the y and z axes

#include "foundation/PxMathUtils.h"

namespace physx
{
namespace Cm
{
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal tanAdd(PxReal tan1, PxReal tan2)
	{
		PX_ASSERT(PxAbs(1.0f-tan1*tan2)>1e-6f);
		return (tan1+tan2)/(1.0f-tan1*tan2);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float computeAxisAndError(const PxVec3& r, const PxVec3& d, const PxVec3& twistAxis, PxVec3& axis)
	{
		// the point on the cone defined by the tanQ swing vector r		
		// this code is equal to quatFromTanQVector(r).rotate(PxVec3(1.0f, 0.0f, 0.0f);
		const PxVec3 p(1.0f, 0.0f, 0.0f);
		const PxReal r2 = r.dot(r), a = 1.0f - r2, b = 1.0f/(1.0f+r2), b2 = b*b;
		const PxReal v1 = 2.0f * a * b2;
		const PxVec3 v2(a, 2.0f * r.z, -2.0f * r.y);	// a*p + 2*r.cross(p);
		const PxVec3 coneLine = v1 * v2 - p;			// already normalized

		// the derivative of coneLine in the direction d	
		const PxReal rd = r.dot(d);
		const PxReal dv1 = -4.0f * rd * (3.0f - r2)*b2*b;
		const PxVec3 dv2(-2.0f * rd, 2.0f * d.z, -2.0f * d.y);
	
		const PxVec3 coneNormal = v1 * dv2 + dv1 * v2;

		axis = coneLine.cross(coneNormal)/coneNormal.magnitude();
		return coneLine.cross(axis).dot(twistAxis);
	}

	// this is here because it's used in both LL and Extensions. However, it
	// should STAY IN THE SDK CODE BASE because it's SDK-specific

	class ConeLimitHelper
	{
	public:
		PX_CUDA_CALLABLE	ConeLimitHelper(PxReal tanQSwingY, PxReal tanQSwingZ, PxReal tanQPadding)
			: mTanQYMax(tanQSwingY), mTanQZMax(tanQSwingZ), mTanQPadding(tanQPadding) {}

		// whether the point is inside the (inwardly) padded cone - if it is, there's no limit
		// constraint

		PX_CUDA_CALLABLE PX_FORCE_INLINE bool contains(const PxVec3& tanQSwing)	const
		{
			const PxReal tanQSwingYPadded = tanAdd(PxAbs(tanQSwing.y),mTanQPadding);
			const PxReal tanQSwingZPadded = tanAdd(PxAbs(tanQSwing.z),mTanQPadding);
			return PxSqr(tanQSwingYPadded/mTanQYMax)+PxSqr(tanQSwingZPadded/mTanQZMax) <= 1;
		}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 clamp(const PxVec3& tanQSwing, PxVec3& normal)	const
		{
			const PxVec3 p = PxEllipseClamp(tanQSwing, PxVec3(0.0f, mTanQYMax, mTanQZMax));
			normal = PxVec3(0.0f, p.y/PxSqr(mTanQYMax), p.z/PxSqr(mTanQZMax));
#ifdef PX_PARANOIA_ELLIPSE_CHECK
			PxReal err = PxAbs(PxSqr(p.y/mTanQYMax) + PxSqr(p.z/mTanQZMax) - 1);
			PX_ASSERT(err<1e-3);
#endif
			return p;
		}

		// input is a swing quat, such that swing.x = twist.y = twist.z = 0, q = swing * twist
		// The routine is agnostic to the sign of q.w (i.e. we don't need the minimal-rotation swing)

		// output is an axis such that positive rotation increases the angle outward from the
		// limit (i.e. the image of the x axis), the error is the sine of the angular difference,
		// positive if the twist axis is inside the cone 

		PX_CUDA_CALLABLE bool getLimit(const PxQuat& swing, PxVec3& axis, PxReal& error)	const
		{
			PX_ASSERT(swing.w>0.0f);
			const PxVec3 twistAxis = swing.getBasisVector0();
			const PxVec3 tanQSwing = PxVec3(0.0f, PxTanHalf(swing.z,swing.w), -PxTanHalf(swing.y,swing.w));
			if(contains(tanQSwing))
				return false;

			PxVec3 normal, clamped = clamp(tanQSwing, normal);

			// rotation vector and ellipse normal
			const PxVec3 r(0.0f, -clamped.z, clamped.y), d(0.0f, -normal.z, normal.y);

			error = computeAxisAndError(r, d, twistAxis, axis);

			PX_ASSERT(PxAbs(axis.magnitude()-1)<1e-5f);

#ifdef PX_PARANOIA_ELLIPSE_CHECK
			bool inside = PxSqr(tanQSwing.y/mTanQYMax) + PxSqr(tanQSwing.z/mTanQZMax) <= 1;
			PX_ASSERT(inside && error>-1e-4f || !inside && error<1e-4f);
#endif
			return true;
		}

	private:

		PxReal mTanQYMax, mTanQZMax, mTanQPadding;
	};
	
	class ConeLimitHelperTanLess
	{
	public:
		PX_CUDA_CALLABLE	ConeLimitHelperTanLess(PxReal swingY, PxReal swingZ)
			: mYMax(swingY), mZMax(swingZ)	{}

		PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 clamp(const PxVec3& swing, PxVec3& normal)	const
		{
			// finds the closest point on the ellipse to a given point
			const PxVec3 p = PxEllipseClamp(swing, PxVec3(0.0f, mYMax, mZMax));
			// normal to the point on ellipse
			normal = PxVec3(0.0f, p.y/PxSqr(mYMax), p.z/PxSqr(mZMax));
#ifdef PX_PARANOIA_ELLIPSE_CHECK
			PxReal err = PxAbs(PxSqr(p.y/mYMax) + PxSqr(p.z/mZMax) - 1);
			PX_ASSERT(err<1e-3);
#endif
			return p;
		}

		// input is a swing quat, such that swing.x = twist.y = twist.z = 0, q = swing * twist
		// The routine is agnostic to the sign of q.w (i.e. we don't need the minimal-rotation swing)

		// output is an axis such that positive rotation increases the angle outward from the
		// limit (i.e. the image of the x axis), the error is the sine of the angular difference,
		// positive if the twist axis is inside the cone 

		PX_CUDA_CALLABLE void getLimit(const PxQuat& swing, PxVec3& axis, PxReal& error)	const
		{
			PX_ASSERT(swing.w>0.0f);
			const PxVec3 twistAxis = swing.getBasisVector0();			
			// get the angles from the swing quaternion
			const PxVec3 swingAngle(0.0f, 4.0f * PxAtan2(swing.y, 1.0f + swing.w), 4.0f * PxAtan2(swing.z, 1.0f + swing.w));

			PxVec3 normal, clamped = clamp(swingAngle, normal);

			// rotation vector and ellipse normal
			const PxVec3 r(0.0f, PxTan(clamped.y/4.0f), PxTan(clamped.z/4.0f)), d(0.0f, normal.y, normal.z);

			error = computeAxisAndError(r, d, twistAxis, axis);

			PX_ASSERT(PxAbs(axis.magnitude()-1.0f)<1e-5f);
		}

	private:
		PxReal mYMax, mZMax;
	};

} // namespace Cm

}

#endif
