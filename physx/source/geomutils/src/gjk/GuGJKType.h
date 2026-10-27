// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_GJKTYPE_H
#define GU_GJKTYPE_H

#include "GuVecConvex.h"
#include "foundation/PxVecTransform.h"

namespace physx
{
namespace Gu
{
	class ConvexHullV;
	class ConvexHullNoScaleV;
	class BoxV;

	template <typename Convex> struct ConvexGeom		{ typedef Convex Type;				};
	template <> struct ConvexGeom<ConvexHullV>			{ typedef ConvexHullV Type;			};
	template <> struct ConvexGeom<ConvexHullNoScaleV>	{ typedef ConvexHullNoScaleV Type;	};
	template <> struct ConvexGeom<BoxV>					{ typedef BoxV Type;				};

	struct GjkConvex
	{
										GjkConvex(const ConvexV& convex) : mConvex(convex)	{}
		virtual							~GjkConvex()										{}

		PX_FORCE_INLINE aos::FloatV		getMinMargin()		const { return mConvex.getMinMargin();		} 
		PX_FORCE_INLINE aos::BoolV		isMarginEqRadius()	const { return mConvex.isMarginEqRadius();	}
		PX_FORCE_INLINE bool			getMarginIsRadius()	const { return mConvex.getMarginIsRadius();	}
		PX_FORCE_INLINE	aos::FloatV		getMargin()			const { return mConvex.getMargin();			}

		template <typename Convex>
		PX_FORCE_INLINE const Convex&	getConvex()			const { return static_cast<const Convex&>(mConvex); }

		virtual aos::Vec3V	supportPoint(const PxI32 index)					const	{ return doVirtualSupportPoint(index);	}
		virtual aos::Vec3V	support(const aos::Vec3VArg v)					const	{ return doVirtualSupport(v);			}
		virtual aos::Vec3V	support(const aos::Vec3VArg dir, PxI32& index)	const	{ return doVirtualSupport(dir, index);	}
		virtual aos::FloatV	getSweepMargin()								const	{ return doVirtualGetSweepMargin();		}
		virtual aos::Vec3V	getCenter()										const = 0;

	protected:
		const ConvexV& mConvex;

	private:
		// PT: following functions call the v-table. I think this curious pattern is needed for the de-virtualization
		// approach used in the GJK code, combined with the GuGJKTest file that is only used externally.
		aos::Vec3V doVirtualSupportPoint(const PxI32 index)					const	{ return supportPoint(index);	}
		aos::Vec3V doVirtualSupport(const aos::Vec3VArg v)					const	{ return support(v);			}
		aos::Vec3V doVirtualSupport(const aos::Vec3VArg dir, PxI32& index)	const	{ return support(dir, index);	}
		aos::FloatV doVirtualGetSweepMargin()								const	{ return getSweepMargin();		}

		//PX_NOCOPY(GjkConvex)
		GjkConvex& operator = (const GjkConvex&);
	};

	template <typename Convex>
	struct LocalConvex : public GjkConvex
	{
		LocalConvex(const Convex& convex) : GjkConvex(convex){}

		// PT: I think we omit the virtual on purpose here, for the de-virtualization approach, i.e. the code calls these
		// functions directly (bypassing the v-table). In fact I think we don't need virtuals at all here, it's probably
		// only for external cases and/or cases where we want to reduce code size.
		// The mix of virtual/force-inline/inline below is confusing/unclear.

		// GjkConvex
		PX_FORCE_INLINE	aos::Vec3V	supportPoint(const PxI32 index)					const	{ return getConvex<Convex>().supportPoint(index);		}
						aos::Vec3V	support(const aos::Vec3VArg v)					const	{ return getConvex<Convex>().supportLocal(v);			}
						aos::Vec3V	support(const aos::Vec3VArg dir, PxI32& index)	const	{ return getConvex<Convex>().supportLocal(dir, index);	}
		PX_INLINE		aos::FloatV	getSweepMargin()								const	{ return getConvex<Convex>().getSweepMargin();			}
						aos::Vec3V	getCenter()										const	{ return getConvex<Convex>().getCenter();				}
		//~GjkConvex

		//ML: we can't force inline function, otherwise win modern will throw compiler error
		PX_INLINE LocalConvex<typename ConvexGeom<Convex>::Type > getGjkConvex() const
		{
			return LocalConvex<typename ConvexGeom<Convex>::Type >(static_cast<const typename ConvexGeom<Convex>::Type&>(GjkConvex::mConvex));
		}

		typedef LocalConvex<typename ConvexGeom<Convex>::Type > ConvexGeomType;

		typedef Convex	Type;

	private:
		//PX_NOCOPY(LocalConvex<Convex>)
		LocalConvex<Convex>& operator = (const LocalConvex<Convex>&);
	};

	template <typename Convex>
	struct RelativeConvex : public GjkConvex
	{
		RelativeConvex(const Convex& convex, const aos::PxMatTransformV& aToB) : GjkConvex(convex), mAToB(aToB), mAToBTransposed(aToB)
		{
			aos::V3Transpose(mAToBTransposed.rot.col0, mAToBTransposed.rot.col1, mAToBTransposed.rot.col2);
		}

		// GjkConvex
		PX_FORCE_INLINE	aos::Vec3V	supportPoint(const PxI32 index)					const	{ return mAToB.transform(getConvex<Convex>().supportPoint(index));					}
						aos::Vec3V	support(const aos::Vec3VArg v)					const	{ return getConvex<Convex>().supportRelative(v, mAToB, mAToBTransposed);			}
						aos::Vec3V	support(const aos::Vec3VArg dir, PxI32& index)	const	{ return getConvex<Convex>().supportRelative(dir, mAToB, mAToBTransposed, index);	}
		PX_INLINE		aos::FloatV	getSweepMargin()								const	{ return getConvex<Convex>().getSweepMargin();										}
						aos::Vec3V	getCenter()										const	{ return mAToB.transform(getConvex<Convex>().getCenter());							}
		//~GjkConvex

		PX_FORCE_INLINE const aos::PxMatTransformV&	getRelativeTransform()			const	{ return mAToB;	}

		//ML: we can't force inline function, otherwise win modern will throw compiler error
		PX_INLINE RelativeConvex<typename ConvexGeom<Convex>::Type > getGjkConvex() const
		{
			return RelativeConvex<typename ConvexGeom<Convex>::Type >(static_cast<const typename ConvexGeom<Convex>::Type&>(GjkConvex::mConvex), mAToB);
		}

		typedef RelativeConvex<typename ConvexGeom<Convex>::Type > ConvexGeomType;

		typedef Convex	Type;

	private:
		//PX_NOCOPY(RelativeConvex<Convex>)
		RelativeConvex<Convex>& operator = (const RelativeConvex<Convex>&);

		const aos::PxMatTransformV& mAToB;
		aos::PxMatTransformV mAToBTransposed;	// PT: precomputed mAToB transpose (because 'rotate' is faster than 'rotateInv')
	};

}
}

#endif
