// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CAPSULE_H
#define GU_CAPSULE_H


#include "GuSegment.h"

namespace physx
{
namespace Gu
{

/**
\brief Represents a capsule.
*/
	class Capsule : public Segment
	{
	public:
		/**
		\brief Constructor
		*/
		PX_INLINE Capsule()
		{
		}

		/**
		\brief Constructor

		\param seg Line segment to create capsule from.
		\param _radius Radius of the capsule.
		*/
		PX_INLINE Capsule(const Segment& seg, PxF32 _radius) : Segment(seg), radius(_radius)
		{
		}

		/**
		\brief Constructor

		\param _p0 First segment point
		\param _p1 Second segment point
		\param _radius Radius of the capsule.
		*/
		PX_INLINE Capsule(const PxVec3& _p0, const PxVec3& _p1, PxF32 _radius) : Segment(_p0, _p1), radius(_radius)
		{           
		}

		/**
		\brief Destructor
		*/
		PX_INLINE ~Capsule()
		{
		}

		PxF32	radius;
	};
}

}

#endif
