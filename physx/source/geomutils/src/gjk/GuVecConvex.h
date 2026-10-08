// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_VEC_CONVEX_H
#define GU_VEC_CONVEX_H

#include "foundation/PxVecMath.h"

#define PX_SUPPORT_INLINE PX_FORCE_INLINE
#define PX_SUPPORT_FORCE_INLINE PX_FORCE_INLINE

namespace physx
{
namespace Gu
{

	struct ConvexType
	{
		enum Type
		{
			eCONVEXHULL,
			eCONVEXHULLNOSCALE,
			eSPHERE,
			eBOX,
			eCAPSULE,
			eTRIANGLE,
			eTETRAHEDRON,
			eCUSTOM
		};
	};
	
	class ConvexV
	{
	public:

		PX_FORCE_INLINE ConvexV(const ConvexType::Type type_) : type(type_), bMarginIsRadius(false)
		{
			margin = 0.f;
			minMargin = 0.f;
			sweepMargin = 0.f;
			center = aos::V3Zero();
		}

		PX_FORCE_INLINE ConvexV(const ConvexType::Type type_, const aos::Vec3VArg center_) : type(type_), bMarginIsRadius(false)
		{
			using namespace aos;
			center = center_;
			margin = 0.f;
			minMargin = 0.f;
			sweepMargin = 0.f;
		}

		//everytime when someone transform the object, they need to up
		PX_FORCE_INLINE void setCenter(const aos::Vec3VArg _center)
		{
			center = _center;
		}

		PX_FORCE_INLINE void setMargin(const aos::FloatVArg margin_)
		{
			aos::FStore(margin_, &margin);
		}

		PX_FORCE_INLINE void setMargin(const PxReal margin_)
		{
			margin = margin_;
		}


		PX_FORCE_INLINE void setMinMargin(const aos::FloatVArg minMargin_)
		{
			aos::FStore(minMargin_, & minMargin);
		}

		PX_FORCE_INLINE void setSweepMargin(const aos::FloatVArg sweepMargin_)
		{
			aos::FStore(sweepMargin_, &sweepMargin);
		}

		PX_FORCE_INLINE aos::Vec3V getCenter()const 
		{
			return center;
		}

		PX_FORCE_INLINE aos::FloatV getMargin() const
		{
			return aos::FLoad(margin);
		}

		PX_FORCE_INLINE aos::FloatV getMinMargin() const
		{
			return aos::FLoad(minMargin);
		}

		PX_FORCE_INLINE aos::FloatV getSweepMargin() const
		{
			return aos::FLoad(sweepMargin);
		}

		PX_FORCE_INLINE ConvexType::Type getType() const
		{
			return type;
		}

		PX_FORCE_INLINE aos::BoolV isMarginEqRadius()const
		{
			return aos::BLoad(bMarginIsRadius);
		}

		PX_FORCE_INLINE bool getMarginIsRadius() const
		{
			return bMarginIsRadius;
		}

		PX_FORCE_INLINE PxReal getMarginF() const
		{
			return margin;
		}


	protected:
		~ConvexV(){}
		aos::Vec3V center;
		PxReal margin;				//margin is the amount by which we shrunk the shape for a convex or box. If the shape are sphere/capsule, margin is the radius
		PxReal minMargin;			//minMargin is some percentage of marginBase, which is used to determine the termination condition for gjk
		PxReal sweepMargin;			//sweepMargin minMargin is some percentage of marginBase, which is used to determine the termination condition for gjkRaycast
		ConvexType::Type	type;
		bool bMarginIsRadius;
	};

	PX_FORCE_INLINE aos::FloatV getContactEps(const aos::FloatV& _marginA, const aos::FloatV& _marginB)
	{
		using namespace aos;

		const FloatV ratio = FLoad(0.25f);
		const FloatV minMargin = FMin(_marginA, _marginB);
		
		return FMul(minMargin, ratio);
	}

	PX_FORCE_INLINE aos::FloatV getSweepContactEps(const aos::FloatV& _marginA, const aos::FloatV& _marginB)
	{
		using namespace aos;

		const FloatV ratio = FLoad(100.f);
		const FloatV minMargin = FAdd(_marginA, _marginB);
		
		return FMul(minMargin, ratio);
	}
}

}

#endif
