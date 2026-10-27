// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CENTER_EXTENTS_H
#define GU_CENTER_EXTENTS_H


#include "foundation/PxUserAllocated.h"
#include "foundation/PxBounds3.h"

namespace physx
{
namespace Gu
{
	class CenterExtents : public physx::PxUserAllocated
	{
		public:
		PX_FORCE_INLINE				CenterExtents()						{}
		PX_FORCE_INLINE				CenterExtents(const PxBounds3& b)	{ mCenter = b.getCenter();	mExtents = b.getExtents();	}
		PX_FORCE_INLINE				~CenterExtents()					{}

		PX_FORCE_INLINE	void		getMin(PxVec3& min)		const		{ min = mCenter - mExtents;					}
		PX_FORCE_INLINE	void		getMax(PxVec3& max)		const		{ max = mCenter + mExtents;					}

		PX_FORCE_INLINE	float		getMin(PxU32 axis)		const		{ return mCenter[axis] - mExtents[axis];	}
		PX_FORCE_INLINE	float		getMax(PxU32 axis)		const		{ return mCenter[axis] + mExtents[axis];	}

		PX_FORCE_INLINE	PxVec3		getMin()				const		{ return mCenter - mExtents;				}
		PX_FORCE_INLINE	PxVec3		getMax()				const		{ return mCenter + mExtents;				}

		PX_FORCE_INLINE	void		setMinMax(const PxVec3& min, const PxVec3& max)
									{
										mCenter = (max + min)*0.5f;
										mExtents = (max - min)*0.5f;
									}

		PX_FORCE_INLINE	PxU32		isInside(const CenterExtents& box)	const
									{
										if(box.getMin(0)>getMin(0))	return 0;
										if(box.getMin(1)>getMin(1))	return 0;
										if(box.getMin(2)>getMin(2))	return 0;
										if(box.getMax(0)<getMax(0))	return 0;
										if(box.getMax(1)<getMax(1))	return 0;
										if(box.getMax(2)<getMax(2))	return 0;
										return 1;
									}

		PX_FORCE_INLINE	void		setEmpty()
									{
										mExtents = PxVec3(-PX_MAX_BOUNDS_EXTENTS);
									}

		PX_FORCE_INLINE	bool		isEmpty()	const
									{
										PX_ASSERT(isValid());
										return mExtents.x<0.0f;
									}

		PX_FORCE_INLINE	bool		isFinite()	const
									{
										return mCenter.isFinite() && mExtents.isFinite();
									}

		PX_FORCE_INLINE	bool		isValid()	const
									{
										const PxVec3& c = mCenter;
										const PxVec3& e = mExtents;
										return (c.isFinite() && e.isFinite() && (((e.x >= 0.0f) && (e.y >= 0.0f) && (e.z >= 0.0f)) ||
																				((e.x == -PX_MAX_BOUNDS_EXTENTS) &&
																				(e.y == -PX_MAX_BOUNDS_EXTENTS) &&
																				(e.z == -PX_MAX_BOUNDS_EXTENTS))));
									}

		PX_FORCE_INLINE	PxBounds3	transformFast(const PxMat33& matrix)	const
									{
										PX_ASSERT(isValid());
										return PxBounds3::basisExtent(matrix * mCenter, matrix, mExtents);
									}

						PxVec3		mCenter;
						PxVec3		mExtents;
	};

	//! A padded version of CenterExtents, to safely load its data using SIMD
	class CenterExtentsPadded : public CenterExtents
	{
	public:
		PX_FORCE_INLINE CenterExtentsPadded()	{}
		PX_FORCE_INLINE ~CenterExtentsPadded()	{}
		PxU32	padding;
	};
	PX_COMPILE_TIME_ASSERT(sizeof(CenterExtentsPadded) == 7*4);

}

}

#endif
