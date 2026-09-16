// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_CONVEX_SUPPORT_TABLE_H
#define GU_CONVEX_SUPPORT_TABLE_H

#include "common/PxPhysXCommonConfig.h"
#include "GuVecConvex.h"
#include "foundation/PxVecTransform.h"

namespace physx
{
namespace Gu  
{

	class TriangleV; 
	class CapsuleV;
	class BoxV;
	class ConvexHullV;
	class ConvexHullNoScaleV;

#if PX_VC 
    #pragma warning(push)   
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif
	class SupportLocal
	{
	public:		
		aos::Vec3V shapeSpaceCenterOfMass;
		const aos::PxTransformV& transform;
		const aos::Mat33V& vertex2Shape;
		const aos::Mat33V& shape2Vertex;
		const bool isIdentityScale;	

		SupportLocal(const aos::PxTransformV& _transform, const aos::Mat33V& _vertex2Shape, const aos::Mat33V& _shape2Vertex, const bool _isIdentityScale = true): transform(_transform), 
			vertex2Shape(_vertex2Shape), shape2Vertex(_shape2Vertex), isIdentityScale(_isIdentityScale)
		{
		}

		PX_FORCE_INLINE void setShapeSpaceCenterofMass(const aos::Vec3VArg _shapeSpaceCenterOfMass)
		{
			shapeSpaceCenterOfMass = _shapeSpaceCenterOfMass;
		}
        virtual ~SupportLocal() {}
		virtual aos::Vec3V doSupport(const aos::Vec3VArg dir) const = 0;
		virtual void doSupport(const aos::Vec3VArg dir, aos::FloatV& min, aos::FloatV& max) const = 0;
		virtual void populateVerts(const PxU8* inds, PxU32 numInds, const PxVec3* originalVerts, aos::Vec3V* verts)const = 0;
	
	protected:
		SupportLocal& operator=(const SupportLocal&);
	};
#if PX_VC 
     #pragma warning(pop) 
#endif

	template <typename Convex>
	class SupportLocalImpl : public SupportLocal
	{
		
	public:
		const Convex& conv;
		SupportLocalImpl(const Convex& _conv, const aos::PxTransformV& _transform, const aos::Mat33V& _vertex2Shape, const aos::Mat33V& _shape2Vertex, const bool _isIdentityScale = true) : 
		SupportLocal(_transform, _vertex2Shape, _shape2Vertex, _isIdentityScale), conv(_conv)
		{
		}

		aos::Vec3V doSupport(const aos::Vec3VArg dir) const
		{
			//return conv.supportVertsLocal(dir);
			return conv.supportLocal(dir);
		}

		void doSupport(const aos::Vec3VArg dir, aos::FloatV& min, aos::FloatV& max) const
		{
			return conv.supportLocal(dir, min, max);
		}

		void populateVerts(const PxU8* inds, PxU32 numInds, const PxVec3* originalVerts, aos::Vec3V* verts) const 
		{
			conv.populateVerts(inds, numInds, originalVerts, verts);
		}

	protected:
		SupportLocalImpl& operator=(const SupportLocalImpl&);

	};
}

}

#endif
