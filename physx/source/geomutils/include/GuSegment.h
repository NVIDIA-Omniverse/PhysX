// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SEGMENT_H
#define GU_SEGMENT_H

#include "foundation/PxVec3.h"

namespace physx
{
namespace Gu
{

	/**
	\brief Represents a line segment.

	Line segment geometry
	In some cases this structure will be used to represent the infinite line that passes point0 and point1.
	*/
	class Segment
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE Segment()
		{
		}

		/**
		\brief Constructor
		*/
		PX_INLINE Segment(const PxVec3& _p0, const PxVec3& _p1) : p0(_p0), p1(_p1)
		{
		}

		/**
		\brief Copy constructor
		*/
		PX_INLINE Segment(const Segment& seg) : p0(seg.p0), p1(seg.p1)
		{
		}

		/**
		\brief Destructor
		*/
		PX_INLINE ~Segment()
		{
		}

		//! Assignment operator
		PX_INLINE Segment& operator=(const Segment& other)
		{
			p0 = other.p0; 
			p1 = other.p1; 
			return *this;
		}

		//! Equality operator
		PX_INLINE bool operator==(const Segment& other) const
		{
			return (p0==other.p0 && p1==other.p1);
		}

		//! Inequality operator
		PX_INLINE bool operator!=(const Segment& other) const
		{
			return (p0!=other.p0 || p1!=other.p1);
		}

		PX_INLINE const PxVec3& getOrigin() const
		{
			return p0;
		}

		//! Return the vector from point0 to point1
		PX_INLINE PxVec3 computeDirection() const
		{
			return p1 - p0;
		}

		//! Return the vector from point0 to point1
		PX_INLINE void computeDirection(PxVec3& dir) const
		{
			dir = p1 - p0;
		}

		//! Return the center of the segment segment
		PX_INLINE PxVec3 computeCenter() const
		{
			return (p0 + p1)*0.5f;
		}

		PX_INLINE PxF32 computeLength() const
		{
			return (p1-p0).magnitude();
		}

		PX_INLINE PxF32 computeSquareLength() const
		{
			return (p1-p0).magnitudeSquared();
		}

		// PT: TODO: remove this one
		//! Return the square of the length of vector from point0 to point1
		PX_INLINE PxReal lengthSquared() const
		{
			return ((p1 - p0).magnitudeSquared());
		}

		// PT: TODO: remove this one
		//! Return the length of vector from point0 to point1
		PX_INLINE PxReal length() const
		{
			return ((p1 - p0).magnitude());
		}

		/*		PX_INLINE void setOriginDirection(const PxVec3& origin, const PxVec3& direction)
		{
		p0 = p1 = origin;
		p1 += direction;
		}*/

		/**
		\brief Computes a point on the segment

		\param[out] pt point on segment
		\param[in] t point's parameter [t=0 => pt = mP0, t=1 => pt = mP1]
		*/
		PX_INLINE void computePoint(PxVec3& pt, PxF32 t) const
		{
			pt = p0 + t * (p1 - p0);
		}

		// PT: TODO: remove this one
		//! Return the point at parameter t along the line: point0 + t*(point1-point0)
		PX_INLINE PxVec3 getPointAt(PxReal t) const
		{
			return (p1 - p0)*t + p0;
		}

		PxVec3	p0;		//!< Start of segment
		PxVec3	p1;		//!< End of segment
	};
	PX_COMPILE_TIME_ASSERT(sizeof(Gu::Segment) == 24);
}

}

#endif
