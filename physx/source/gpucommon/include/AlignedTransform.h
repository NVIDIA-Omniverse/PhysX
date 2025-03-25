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

#ifndef PX_ALIGNED_TRANSFORM_H
#define PX_ALIGNED_TRANSFORM_H

#include "AlignedQuat.h"
#include "foundation/PxPlane.h"
#include "foundation/PxTransform.h"


namespace physx
{

class PxAlignedTransform
{
public:
	PxAlignedQuat q;
	float4 p;

//#define PxAlignedTransform_DEFAULT_CONSTRUCT_NAN

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform() 
#ifdef PXTRANSFORM_DEFAULT_CONSTRUCT_IDENTITY
		: q(0, 0, 0, 1), p(0, 0, 0)
#elif defined(PXTRANSFORM_DEFAULT_CONSTRUCT_NAN)
#define invalid PxSqrt(-1.0f)
		: q(invalid, invalid, invalid, invalid), p(invalid, invalid, invalid)
#undef invalid
#endif
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxAlignedTransform(const float4& position): q(PxIdentity), p(position)
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxAlignedTransform(PxIDENTITY r)
		: q(PxIdentity), p(make_float4(0.f))
	{
		PX_UNUSED(r);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE explicit PxAlignedTransform(const PxAlignedQuat& orientation): q(orientation), p(make_float4(0.f))
	{
		PX_ASSERT(orientation.isSane());
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform(PxReal x, PxReal y, PxReal z)
		: q(PxIdentity), p(make_float4(x, y, z, 0.f))
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform(PxReal x, PxReal y, PxReal z, const PxAlignedQuat& aQ)
		: q(aQ), p(make_float4(x, y, z, 0.f))
	{
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform(const float4& p0, const PxAlignedQuat& q0): q(q0), p(p0) 
	{
		PX_ASSERT(q0.isSane());
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform(const PxTransform& x)
	{
		PX_ASSERT(x.isSane());
		q = PxAlignedQuat( x.q );
		p = make_float4(x.p.x, x.p.y, x.p.z, 0.f);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxTransform getTransform() const
	{
		return PxTransform(PxVec3(p.x, p.y, p.z), PxQuat(q.q.x, q.q.y, q.q.z, q.q.w)); 
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform operator*(const PxAlignedTransform& x) const
	{
		PX_ASSERT(x.isSane());
		return transform(x);
	}

	//! Equals matrix multiplication
	PX_CUDA_CALLABLE PX_INLINE PxAlignedTransform& operator*=(PxAlignedTransform &other)
	{
		*this = *this * other;
		return *this;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool operator==(const PxAlignedTransform &other) const
	{
		return (p.x == other.p.x) && (p.y == other.p.y) && (p.z == other.p.z) && (q == other.q);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE bool operator!=(const PxAlignedTransform &other) const
	{
		return !(*this == other);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform getInverse() const
	{
		PX_ASSERT(isFinite());
		return PxAlignedTransform(q.rotateInv(make_float4(-p.x, -p.y, -p.z, -p.w)),q.getConjugate());
	}


	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transform(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotate(input) + PxVec3(p.x, p.y, p.z);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 transformInv(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotateInv(input-PxVec3(p.x, p.y, p.z));
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float4 transform(const float4& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotate(input) + p;
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float4 transformInv(const float4& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotateInv(input-p);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 rotate(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotate(input);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec3 rotateInv(const PxVec3& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotateInv(input);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float4 rotate(const float4& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotate(input);
	}

	PX_CUDA_CALLABLE PX_FORCE_INLINE float4 rotateInv(const float4& input) const
	{
		PX_ASSERT(isFinite());
		return q.rotateInv(input);
	}

	//! Transform transform to parent (returns compound transform: first src, then *this)
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform transform(const PxAlignedTransform& src) const
	{
		PX_ASSERT(src.isSane());
		PX_ASSERT(isSane());
		// src = [srct, srcr] -> [r*srct + t, r*srcr]
		return PxAlignedTransform(q.rotate(src.p) + p, q*src.q);
	}

	/**
	\brief returns true if finite and q is a unit quaternion
	*/

	PX_CUDA_CALLABLE bool isValid() const
	{
		return PxIsFinite(p.x) && PxIsFinite(p.y) && PxIsFinite(p.z) && q.isFinite() && q.isUnit();
	}

	/**
	\brief returns true if finite and quat magnitude is reasonably close to unit to allow for some accumulation of error vs isValid
	*/

	PX_CUDA_CALLABLE bool isSane() const
	{
	      return isFinite() && q.isSane();
	}


	/**
	\brief returns true if all elems are finite (not NAN or INF, etc.)
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE bool isFinite() const { return PxIsFinite(p.x) && PxIsFinite(p.y) && PxIsFinite(p.z) && q.isFinite(); }

	//! Transform transform from parent (returns compound transform: first src, then this->inverse)
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform transformInv(const PxAlignedTransform& src) const
	{
		PX_ASSERT(src.isSane());
		PX_ASSERT(isFinite());
		// src = [srct, srcr] -> [r^-1*(srct-t), r^-1*srcr]
		PxAlignedQuat qinv = q.getConjugate();
		return PxAlignedTransform(qinv.rotate(src.p - p), qinv*src.q);
	}

	/**
	\brief transform plane
	*/

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane transform(const PxPlane& plane) const
	{
		PxVec3 transformedNormal = rotate(plane.n);
		return PxPlane(transformedNormal, plane.d - PxVec3(p.x, p.y, p.z).dot(transformedNormal));
	}

	/**
	\brief inverse-transform plane
	*/

	PX_CUDA_CALLABLE PX_FORCE_INLINE PxPlane inverseTransform(const PxPlane& plane) const
	{
		PxVec3 transformedNormal = rotateInv(plane.n);
		return PxPlane(transformedNormal, plane.d + PxVec3(p.x, p.y, p.z).dot(plane.n));
	}


	/**
	\brief return a normalized transform (i.e. one in which the quaternion has unit magnitude)
	*/
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxAlignedTransform getNormalized() const
	{
		return PxAlignedTransform(p, q.getNormalized());
	}

};
}

#endif
