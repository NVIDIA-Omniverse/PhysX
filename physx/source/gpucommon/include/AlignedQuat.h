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

#ifndef PX_ALIGNED_QUAT_H
#define PX_ALIGNED_QUAT_H


#include "vector_types.h"
#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "cutil_math.h"
#include "foundation/PxQuat.h"
#include "foundation/PxAssert.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxAlignedMat33;

/**
\brief This is a quaternion class. For more information on quaternion mathematics
consult a mathematics source on complex numbers.

*/
PX_ALIGN_PREFIX(16)
class PxAlignedQuat
{
public:
	/**
	\brief Default constructor, does not do any initialization.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat()	{	}


	//! identity constructor
	PX_CUDA_CALLABLE PX_INLINE PxAlignedQuat(PxIDENTITY r)
		: q(make_float4(0.0f, 0.0f, 0.0f, 1.0f))
	{
		PX_UNUSED(r);
	}

	/**
	\brief Constructor from a scalar: sets the real part w to the scalar value, and the imaginary parts (x,y,z) to zero
	*/
	explicit PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat(PxReal r)
		: q(make_float4(0.0f, 0.0f, 0.0f, r))
	{
	}


	/**
	\brief Constructor.  Take note of the order of the elements!
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat(PxReal nx, PxReal ny, PxReal nz, PxReal nw) : q(make_float4(nx, ny, nz, nw)) {}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat(const PxQuat& q0) : q(make_float4(q0.x, q0.y, q0.z, q0.w)) {}

	/**
	\brief Creates from angle-axis representation.

	Axis must be normalized!

	Angle is in radians!

	<b>Unit:</b> Radians
	*/
	PX_CUDA_CALLABLE PX_INLINE PxAlignedQuat(PxReal angleRadians, const PxVec3& unitAxis)
	{
		PX_ASSERT(PxAbs(1.0f-unitAxis.magnitude())<1e-3f);
		const PxReal a = angleRadians * 0.5f;
		const PxReal s = PxSin(a);
		q.w = PxCos(a);
		q.x = unitAxis.x * s;
		q.y = unitAxis.y * s;
		q.z = unitAxis.z * s;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat(const float4 v) : q(v) {}

	/**
	\brief Copy ctor.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat(const PxAlignedQuat& v): q(v.q) {}

	/**
	\brief Creates from orientation matrix.

	\param[in] m Rotation matrix to extract quaternion from.
	*/
	PX_CUDA_CALLABLE PX_INLINE explicit PxAlignedQuat(const PxAlignedMat33& m); /* defined in PxAlignedMat33.h */

	/**
	\brief returns true if all elements are finite (not NAN or INF, etc.)
	*/
	PX_CUDA_CALLABLE bool isFinite() const
	{
		return PxIsFinite(q.x) 
			&& PxIsFinite(q.y) 
			&& PxIsFinite(q.z)
			&& PxIsFinite(q.w);
	}


	/**
	\brief returns true if finite and magnitude is close to unit
	*/

	PX_CUDA_CALLABLE bool isUnit() const
	{
		const PxReal unitTolerance = 1e-4f;
		return isFinite() && PxAbs(magnitude()-1)<unitTolerance;
	}


	/**
	\brief returns true if finite and magnitude is reasonably close to unit to allow for some accumulation of error vs isValid
	*/

	PX_CUDA_CALLABLE bool isSane() const
	{
	      const PxReal unitTolerance = 1e-2f;
	      return isFinite() && PxAbs(magnitude()-1)<unitTolerance;
	}

	/**
	\brief returns true if the two quaternions are exactly equal
	*/
	PX_CUDA_CALLABLE PX_INLINE bool operator==(const PxAlignedQuat&q2) const	{ return q.x == q2.q.x && q.y == q2.q.y && q.z == q2.q.z && q.w == q2.q.w; }


	/**
	\brief This is the squared 4D vector length, should be 1 for unit quaternions.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal magnitudeSquared() const
	{
		return ::dot(q,q);
	}

	/**
	\brief returns the scalar product of this and other.
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal dot(const PxAlignedQuat& v) const
	{
		return ::dot(q,v.q);
	}

	PX_CUDA_CALLABLE PX_INLINE PxAlignedQuat getNormalized() const
	{
		const PxReal s = 1.0f/magnitude();
		return PxAlignedQuat(q.x*s, q.y*s, q.z*s, q.w*s);
	}


	PX_CUDA_CALLABLE PX_INLINE float magnitude() const
	{
		return PxSqrt(magnitudeSquared());
	}

	//modifiers:
	/**
	\brief maps to the closest unit quaternion.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxReal normalize()											// convert this PxAlignedQuat to a unit quaternion
	{
		const PxReal mag = magnitude();
		if (mag != 0.0f)
		{
			const PxReal imag = 1.0f / mag;

			q.x *= imag;
			q.y *= imag;
			q.z *= imag;
			q.w *= imag;
		}
		return mag;
	}

	/*
	\brief returns the conjugate.

	\note for unit quaternions, this is the inverse.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxAlignedQuat getConjugate() const
	{
		return PxAlignedQuat(-q.x,-q.y,-q.z,q.w);
	}

	/*
	\brief returns imaginary part.
	*/
	PX_CUDA_CALLABLE PX_INLINE PxVec3 getImaginaryPart() const
	{
		return PxVec3(q.x,q.y,q.z);
	}

	/** brief computes rotation of x-axis */
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getBasisVector0()	const
	{	
		const PxF32 x2 = q.x*2.0f;
		const PxF32 w2 = q.w*2.0f;
		return PxVec3(	(q.w * w2) - 1.0f + q.x*x2,
						(q.z * w2)        + q.y*x2,
						(-q.y * w2)       + q.z*x2);
	}
	
	/** brief computes rotation of y-axis */
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getBasisVector1()	const 
	{	
		const PxF32 y2 = q.y*2.0f;
		const PxF32 w2 = q.w*2.0f;
		return PxVec3(	(-q.z * w2)       + q.x*y2,
						(q.w * w2) - 1.0f + q.y*y2,
						(q.x * w2)        + q.z*y2);
	}


	/** brief computes rotation of z-axis */
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 getBasisVector2() const	
	{	
		const PxF32 z2 = q.z*2.0f;
		const PxF32 w2 = q.w*2.0f;
		return PxVec3(	(q.y * w2)        + q.x*z2,
						(-q.x * w2)       + q.y*z2,
						(q.w * w2) - 1.0f + q.z*z2);
	}

	/**
	rotates passed vec by this (assumed unitary)
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3 rotate(const PxVec3& v) const
	{
		const PxF32 vx = 2.0f*v.x;
		const PxF32 vy = 2.0f*v.y;
		const PxF32 vz = 2.0f*v.z;
		const PxF32 w2 = q.w*q.w-0.5f;
		const PxF32 dot2 = (q.x*vx + q.y*vy +q.z*vz);
		return PxVec3
		(
			(vx*w2 + (q.y * vz - q.z * vy)*q.w + q.x*dot2), 
			(vy*w2 + (q.z * vx - q.x * vz)*q.w + q.y*dot2), 
			(vz*w2 + (q.x * vy - q.y * vx)*q.w + q.z*dot2)
		);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const float4 rotate(const float4& v) const
	{
		const PxF32 vx = 2.0f*v.x;
		const PxF32 vy = 2.0f*v.y;
		const PxF32 vz = 2.0f*v.z;
		const PxF32 w2 = q.w*q.w-0.5f;
		const PxF32 dot2 = (q.x*vx + q.y*vy +q.z*vz);
		return make_float4
		(
			(vx*w2 + (q.y * vz - q.z * vy)*q.w + q.x*dot2), 
			(vy*w2 + (q.z * vx - q.x * vz)*q.w + q.y*dot2), 
			(vz*w2 + (q.x * vy - q.y * vx)*q.w + q.z*dot2),
			0.f
		);
	}

	/**
	inverse rotates passed vec by this (assumed unitary)
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE const PxVec3 rotateInv(const PxVec3& v) const
	{
		const PxF32 vx = 2.0f*v.x;
		const PxF32 vy = 2.0f*v.y;
		const PxF32 vz = 2.0f*v.z;
		const PxF32 w2 = q.w*q.w-0.5f;
		const PxF32 dot2 = (q.x*vx + q.y*vy +q.z*vz);
		return PxVec3
		(
			(vx*w2 - (q.y * vz - q.z * vy)*q.w + q.x*dot2), 
			(vy*w2 - (q.z * vx - q.x * vz)*q.w + q.y*dot2), 
			(vz*w2 - (q.x * vy - q.y * vx)*q.w + q.z*dot2)
		);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE const float4 rotateInv(const float4& v) const
	{
		const PxF32 vx = 2.0f*v.x;
		const PxF32 vy = 2.0f*v.y;
		const PxF32 vz = 2.0f*v.z;
		const PxF32 w2 = q.w*q.w-0.5f;
		const PxF32 dot2 = (q.x*vx + q.y*vy +q.z*vz);
		return make_float4
		(
			(vx*w2 - (q.y * vz - q.z * vy)*q.w + q.x*dot2), 
			(vy*w2 - (q.z * vx - q.x * vz)*q.w + q.y*dot2), 
			(vz*w2 - (q.x * vy - q.y * vx)*q.w + q.z*dot2),
			0.f
		);
	}

	/**
	\brief Assignment operator
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE	PxAlignedQuat&	operator=(const PxAlignedQuat& p)			{ q = p.q;	return *this;		}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat& operator*= (const PxAlignedQuat& q2)
	{
		const PxReal tx = q.w*q2.q.x + q2.q.w*q.x + q.y*q2.q.z - q2.q.y*q.z;
		const PxReal ty = q.w*q2.q.y + q2.q.w*q.y + q.z*q2.q.x - q2.q.z*q.x;
		const PxReal tz = q.w*q2.q.z + q2.q.w*q.z + q.x*q2.q.y - q2.q.x*q.y;

		q.w = q.w*q2.q.w - q2.q.x*q.x - q.y*q2.q.y - q2.q.z*q.z;
		q.x = tx;
		q.y = ty;
		q.z = tz;

		return *this;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat& operator+= (const PxAlignedQuat& q2)
	{
		q += q2.q;
		return *this;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat& operator-= (const PxAlignedQuat& q2)
	{
		q -= q2.q;
		return *this;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat& operator*= (const PxReal s)
	{
		q *= s;
		return *this;
	}

	/** quaternion multiplication */
	PX_CUDA_CALLABLE PX_INLINE PxAlignedQuat operator*(const PxAlignedQuat& q2) const
	{
		return PxAlignedQuat(q.w*q2.q.x + q2.q.w*q.x + q.y*q2.q.z - q2.q.y*q.z,
					  q.w*q2.q.y + q2.q.w*q.y + q.z*q2.q.x - q2.q.z*q.x,
					  q.w*q2.q.z + q2.q.w*q.z + q.x*q2.q.y - q2.q.x*q.y,
					  q.w*q2.q.w - q.x*q2.q.x - q.y*q2.q.y - q.z*q2.q.z);
	}

	/** quaternion addition */
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat operator+(const PxAlignedQuat& q2) const
	{
		return PxAlignedQuat(q + q2.q);
	}

	/** quaternion subtraction */
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat operator-() const
	{
		return PxAlignedQuat(-q.x,-q.y,-q.z,-q.w);
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat operator-(const PxAlignedQuat& q2) const
	{
		return PxAlignedQuat(q - q2.q);
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedQuat operator*(PxReal r) const
	{
		return PxAlignedQuat(q*r);
	}

	// TODO avoroshilov: check if it's OK
	PX_CUDA_CALLABLE PX_FORCE_INLINE operator PxQuat() const
	{
		return PxQuat(q.x, q.y, q.z, q.w);
	}

	/** the quaternion elements */
	float4 q;
} 
PX_ALIGN_SUFFIX(16);

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif // PX_FOUNDATION_PX_QUAT_H
