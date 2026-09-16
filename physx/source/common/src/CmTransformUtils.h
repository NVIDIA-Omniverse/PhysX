// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef CM_TRANSFORM_UTILS_H
#define CM_TRANSFORM_UTILS_H

#include "foundation/PxVecMath.h"

namespace
{

using namespace physx::aos;

// V3PrepareCross would help here, but it's not on all platforms yet...

PX_FORCE_INLINE void transformFast(const FloatVArg wa, const Vec3VArg va, const Vec3VArg pa, 
							   const FloatVArg wb, const Vec3VArg vb, const Vec3VArg pb, 
							   FloatV& wo, Vec3V& vo, Vec3V& po)
{
	wo = FSub(FMul(wa, wb), V3Dot(va, vb));
	vo = V3ScaleAdd(va, wb, V3ScaleAdd(vb, wa, V3Cross(va, vb)));

	const Vec3V t1 = V3Scale(pb, FScaleAdd(wa, wa, FLoad(-0.5f)));
	const Vec3V t2 = V3ScaleAdd(V3Cross(va, pb), wa, t1);
	const Vec3V t3 = V3ScaleAdd(va, V3Dot(va, pb), t2);

	po = V3ScaleAdd(t3, FLoad(2.f), pa);
}

PX_FORCE_INLINE void transformInvFast(const FloatVArg wa, const Vec3VArg va, const Vec3VArg pa, 
								      const FloatVArg wb, const Vec3VArg vb, const Vec3VArg pb, 
								      FloatV& wo, Vec3V& vo, Vec3V& po)
{
	wo = FScaleAdd(wa, wb, V3Dot(va, vb));
	vo = V3NegScaleSub(va, wb, V3ScaleAdd(vb, wa, V3Cross(vb, va)));

	const Vec3V pt = V3Sub(pb, pa);
	const Vec3V t1 = V3Scale(pt, FScaleAdd(wa, wa, FLoad(-0.5f)));
	const Vec3V t2 = V3ScaleAdd(V3Cross(pt, va), wa, t1);
	const Vec3V t3 = V3ScaleAdd(va, V3Dot(va, pt), t2);
	po = V3Add(t3,t3);
}

}

namespace physx
{
namespace Cm
{

// PT: actor2World * shape2Actor
PX_FORCE_INLINE void getStaticGlobalPoseAligned(const PxTransform& actor2World, const PxTransform& shape2Actor, PxTransform& outTransform)
{
	using namespace aos;

	PX_ASSERT((size_t(&actor2World)&15) == 0);
	PX_ASSERT((size_t(&shape2Actor)&15) == 0);
	PX_ASSERT((size_t(&outTransform)&15) == 0);

	const Vec3V actor2WorldPos = V3LoadA(actor2World.p);
	const QuatV actor2WorldRot = QuatVLoadA(&actor2World.q.x);

	const Vec3V shape2ActorPos = V3LoadA(shape2Actor.p);
	const QuatV shape2ActorRot = QuatVLoadA(&shape2Actor.q.x);
	
	Vec3V v,p;
	FloatV w; 

	transformFast(V4GetW(actor2WorldRot), Vec3V_From_Vec4V(actor2WorldRot), actor2WorldPos,
				  V4GetW(shape2ActorRot), Vec3V_From_Vec4V(shape2ActorRot), shape2ActorPos,
				  w, v, p);

	V3StoreA(p, outTransform.p);
	V4StoreA(V4SetW(v,w), &outTransform.q.x);
}

// PT: body2World * body2Actor.getInverse() * shape2Actor
PX_FORCE_INLINE void getDynamicGlobalPoseAligned(const PxTransform& body2World, const PxTransform& shape2Actor, const PxTransform& body2Actor, PxTransform& outTransform)
{
	PX_ASSERT((size_t(&body2World)&15) == 0);
	PX_ASSERT((size_t(&shape2Actor)&15) == 0);
	PX_ASSERT((size_t(&body2Actor)&15) == 0);
	PX_ASSERT((size_t(&outTransform)&15) == 0);

	using namespace aos;

	const Vec3V shape2ActorPos = V3LoadA(shape2Actor.p);
	const QuatV shape2ActorRot = QuatVLoadA(&shape2Actor.q.x);

	const Vec3V body2ActorPos = V3LoadA(body2Actor.p);
	const QuatV body2ActorRot = QuatVLoadA(&body2Actor.q.x);

	const Vec3V body2WorldPos = V3LoadA(body2World.p);
	const QuatV body2WorldRot = QuatVLoadA(&body2World.q.x);

	Vec3V v1, p1, v2, p2;
	FloatV w1, w2; 

	transformInvFast(V4GetW(body2ActorRot), Vec3V_From_Vec4V(body2ActorRot), body2ActorPos,
					 V4GetW(shape2ActorRot), Vec3V_From_Vec4V(shape2ActorRot), shape2ActorPos,
					 w1, v1, p1);

	transformFast(V4GetW(body2WorldRot), Vec3V_From_Vec4V(body2WorldRot), body2WorldPos,
			      w1, v1, p1,
			      w2, v2, p2);

	V3StoreA(p2, outTransform.p);
	V4StoreA(V4SetW(v2, w2), &outTransform.q.x);
}

}
}

#endif
