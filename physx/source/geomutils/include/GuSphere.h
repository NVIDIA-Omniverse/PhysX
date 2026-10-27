// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SPHERE_H
#define GU_SPHERE_H

#include "foundation/PxVec3.h"

namespace physx
{

/**
\brief Represents a sphere defined by its center point and radius.
*/
namespace Gu
{
	class Sphere
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE Sphere()
		{
		}

		/**
		\brief Constructor
		*/
		PX_INLINE Sphere(const PxVec3& _center, PxF32 _radius) : center(_center), radius(_radius)
		{
		}
		/**
		\brief Copy constructor
		*/
		PX_INLINE Sphere(const Sphere& sphere) : center(sphere.center), radius(sphere.radius)
		{
		}
		/**
		\brief Destructor
		*/
		PX_INLINE ~Sphere()
		{
		}

		PX_INLINE	void	set(const PxVec3& _center, float _radius)		{ center = _center; radius = _radius;	}

		/**
		\brief Checks the sphere is valid.

		\return		true if the sphere is valid
		*/
		PX_INLINE bool isValid() const
		{
			// Consistency condition for spheres: Radius >= 0.0f
			return radius >= 0.0f;
		}

		/**
		\brief Tests if a point is contained within the sphere.

		\param[in] p the point to test
		\return	true if inside the sphere
		*/
		PX_INLINE bool contains(const PxVec3& p) const
		{
			return (center-p).magnitudeSquared() <= radius*radius;
		}

		PxVec3	center;		//!< Sphere's center
		PxF32	radius;		//!< Sphere's radius
	};
}

}

#endif
