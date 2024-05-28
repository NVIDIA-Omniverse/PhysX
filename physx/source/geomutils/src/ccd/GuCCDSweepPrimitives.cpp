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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "GuVecCapsule.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecTriangle.h"
#include "GuGJKRaycast.h"
#include "GuCCDSweepConvexMesh.h"
#include "GuGJKType.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxCustomGeometry.h"

//#define USE_VIRTUAL_GJK

namespace physx
{
namespace Gu
{

using namespace aos;

template<typename Geom> PX_FORCE_INLINE  PxReal getRadius(const PxGeometry&) 
{	
	return 0;	
}

template<> PX_FORCE_INLINE PxReal getRadius<CapsuleV>(const PxGeometry& g) 
{ 
	PX_ASSERT(g.getType() == PxGeometryType::eCAPSULE || g.getType() == PxGeometryType::eSPHERE);
	PX_COMPILE_TIME_ASSERT(PX_OFFSET_OF(PxSphereGeometry, radius) == PX_OFFSET_OF(PxCapsuleGeometry, radius));
	return static_cast<const PxSphereGeometry&>(g).radius;
}

#ifdef USE_VIRTUAL_GJK
static bool virtualGjkRaycastPenetration(const GjkConvex& a, const GjkConvex& b, const aos::Vec3VArg initialDir, const aos::FloatVArg initialLambda, const aos::Vec3VArg s, const aos::Vec3VArg r, aos::FloatV& lambda, 
		aos::Vec3V& normal, aos::Vec3V& closestA, const PxReal _inflation, const bool initialOverlap)
{
	return gjkRaycastPenetration<GjkConvex, GjkConvex >(a, b, initialDir, initialLambda, s, r, lambda, normal, closestA, _inflation, initialOverlap);
}
#endif

template<class ConvexA, class ConvexB>
static PX_FORCE_INLINE PxReal CCDSweep(	ConvexA& a, ConvexB& b, const PxTransform32& transform0, const PxTransform32& transform1, const PxTransform32& lastTm0, const PxTransform32& lastTm1,
										const aos::FloatV& toiEstimate, PxVec3& worldPoint, PxVec3& worldNormal, PxReal inflation = 0.0f)
{
	PX_UNUSED(toiEstimate); //KS - TODO - can we use this again?
	using namespace aos;

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&lastTm0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&lastTm1.p.x);

	const PxTransformV tr0(p0, q0);
	const PxTransformV tr1(p1, q1);

	const PxMatTransformV aToB(tr1.transformInv(tr0));
	
	const Vec3V trans0p = V3LoadA(transform0.p);
	const Vec3V trans1p = V3LoadA(transform1.p);
	const Vec3V trA = V3Sub(trans0p, p0);
	const Vec3V trB = V3Sub(trans1p, p1);
	const Vec3V relTr = tr1.rotateInv(V3Sub(trB, trA));

	FloatV lambda;
	Vec3V closestA, normal;
	const FloatV initialLambda = FZero();
	const RelativeConvex<ConvexA> convexA(a, aToB);
	const LocalConvex<ConvexB> convexB(b);
#ifdef USE_VIRTUAL_GJK
	if(virtualGjkRaycastPenetration(convexA, convexB, aToB.p, initialLambda, V3Zero(), relTr, lambda, normal, closestA, inflation, true))
#else
	if(gjkRaycastPenetration<RelativeConvex<ConvexA>, LocalConvex<ConvexB> >(convexA, convexB, aToB.p, initialLambda, V3Zero(), relTr, lambda, normal, closestA, inflation, true))
#endif
	{
		//Adjust closestA because it will be on the surface of convex a in its initial position (s). If the TOI > 0, we need to move 
		//the point along the sweep direction to get the world-space hit position.
		PxF32 res;
		FStore(lambda, &res);
		closestA = V3ScaleAdd(trA, FMax(lambda, FZero()), tr1.transform(closestA));
		normal = tr1.rotate(normal);

		V3StoreU(normal, worldNormal);
		V3StoreU(closestA, worldPoint);
		return res;
	}
	return PX_MAX_REAL;
}

//
// lookup table for geometry-vs-geometry sweeps
//

PxReal UnimplementedSweep (GU_SWEEP_METHOD_ARGS_UNUSED)
{
	return PX_MAX_REAL;	//no impact
}

template<typename Geom0, typename Geom1>	
static PxReal SweepGeomGeom(GU_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(outCCDFaceIndex);
	PX_UNUSED(fastMovingThreshold);

	const PxGeometry& g0 = *shape0.mGeometry;
	const PxGeometry& g1 = *shape1.mGeometry;

	typename ConvexGeom<Geom0>::Type geom0(g0);
	typename ConvexGeom<Geom1>::Type geom1(g1);

	return CCDSweep(geom0, geom1, transform0, transform1, lastTm0, lastTm1, FLoad(toiEstimate), worldPoint, worldNormal, restDistance+getRadius<Geom0>(g0)+getRadius<Geom1>(g1) );
}

static PxReal SweepAnyShapeCustom(GU_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(fastMovingThreshold);
	PX_UNUSED(toiEstimate);
	PX_UNUSED(restDistance);

	const PxGeometry& g0 = *shape0.mGeometry;
	const PxGeometry& g1 = *shape1.mGeometry;

	PX_ASSERT(g1.getType() == PxGeometryType::eCUSTOM);

	const PxVec3 trA = transform0.p - lastTm0.p;
	const PxVec3 trB = transform1.p - lastTm1.p;

	const PxVec3 relTr = trA - trB;
	PxVec3 unitDir = relTr;
	const PxReal length = unitDir.normalize();

	PxGeomSweepHit sweepHit;

	if (!static_cast<const PxCustomGeometry&>(g1).callbacks->sweep(unitDir, length, g1, lastTm1, g0, lastTm0, sweepHit, PxHitFlag::eDEFAULT, 0.0f, NULL))
		return PX_MAX_REAL;

	worldNormal = sweepHit.normal;
	worldPoint = sweepHit.position;
	outCCDFaceIndex = sweepHit.faceIndex;

	return sweepHit.distance / length;
}

typedef PxReal (*SweepMethod) (GU_SWEEP_METHOD_ARGS);

PxReal SweepAnyShapeHeightfield(GU_SWEEP_METHOD_ARGS);
PxReal SweepAnyShapeMesh(GU_SWEEP_METHOD_ARGS);

SweepMethod g_SweepMethodTable[][PxGeometryType::eGEOMETRY_COUNT] = 
{
	//PxGeometryType::eSPHERE
	{
		SweepGeomGeom<CapsuleV, CapsuleV>,			//PxGeometryType::eSPHERE
		UnimplementedSweep,							//PxGeometryType::ePLANE
		SweepGeomGeom<CapsuleV, CapsuleV>,			//PxGeometryType::eCAPSULE
		SweepGeomGeom<CapsuleV, BoxV>,				//PxGeometryType::eBOX
		SweepGeomGeom<CapsuleV, ConvexHullV>,		//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		SweepAnyShapeMesh,							//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,					//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePLANE
	{
		0,											//PxGeometryType::eSPHERE
		UnimplementedSweep,							//PxGeometryType::ePLANE
		UnimplementedSweep,							//PxGeometryType::eCAPSULE
		UnimplementedSweep,							//PxGeometryType::eBOX
		UnimplementedSweep,							//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		UnimplementedSweep,							//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		UnimplementedSweep,							//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCAPSULE
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		SweepGeomGeom<CapsuleV, CapsuleV>,			//PxGeometryType::eCAPSULE
		SweepGeomGeom<CapsuleV, BoxV>,				//PxGeometryType::eBOX
		SweepGeomGeom<CapsuleV, ConvexHullV>,		//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		SweepAnyShapeMesh,							//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,					//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eBOX
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		SweepGeomGeom<BoxV, BoxV>,					//PxGeometryType::eBOX
		SweepGeomGeom<BoxV, ConvexHullV>,			//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		SweepAnyShapeMesh,							//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,					//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCONVEXMESH
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		SweepGeomGeom<ConvexHullV, ConvexHullV>,	//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		SweepAnyShapeMesh,							//PxGeometryType::eTRIANGLEMESH
		SweepAnyShapeHeightfield,					//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::ePARTICLESYSTEM
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		UnimplementedSweep,							//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		UnimplementedSweep,							//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		UnimplementedSweep,							//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTETRAHEDRONMESH
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::ePARTICLESYSTEM
		UnimplementedSweep,							//PxGeometryType::eTETRAHEDRONMESH
		UnimplementedSweep,							//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		UnimplementedSweep,							//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eTRIANGLEMESH
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::ePARTICLESYSTEM
		0,											//PxGeometryType::eTETRAHEDRONMESH
		UnimplementedSweep,							//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHEIGHTFIELD
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::ePARTICLESYSTEM
		0,											//PxGeometryType::eTETRAHEDRONMESH
		0,											//PxGeometryType::eTRIANGLEMESH
		UnimplementedSweep,							//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eHAIRSYSTEM
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::ePARTICLESYSTEM
		0,											//PxGeometryType::eTETRAHEDRONMESH
		0,											//PxGeometryType::eTRIANGLEMESH
		0,											//PxGeometryType::eHEIGHTFIELD
		UnimplementedSweep,							//PxGeometryType::eHAIRSYSTEM
		UnimplementedSweep,							//PxGeometryType::eCUSTOM
	},

	//PxGeometryType::eCUSTOM
	{
		0,											//PxGeometryType::eSPHERE
		0,											//PxGeometryType::ePLANE
		0,											//PxGeometryType::eCAPSULE
		0,											//PxGeometryType::eBOX
		0,											//PxGeometryType::eCONVEXMESH
		0,											//PxGeometryType::ePARTICLESYSTEM
		0,											//PxGeometryType::eTETRAHEDRONMESH
		0,											//PxGeometryType::eTRIANGLEMESH
		0,											//PxGeometryType::eHEIGHTFIELD
		0,											//PxGeometryType::eHAIRSYSTEM
		SweepAnyShapeCustom,						//PxGeometryType::eCUSTOM
	},
};
PX_COMPILE_TIME_ASSERT(sizeof(g_SweepMethodTable) / sizeof(g_SweepMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

PxReal SweepShapeShape(GU_SWEEP_METHOD_ARGS)
{
	const PxGeometryType::Enum type0 = shape0.mGeometry->getType();
	const PxGeometryType::Enum type1 = shape1.mGeometry->getType();

	return g_SweepMethodTable[type0][type1](shape0, shape1, transform0, transform1, lastTm0, lastTm1,
		restDistance, worldNormal, worldPoint, toiEstimate, outCCDFaceIndex, fastMovingThreshold);
}

//
// lookup table for sweeps agains triangles
//

PxReal UnimplementedTriangleSweep(GU_TRIANGLE_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(shape1);
	PX_UNUSED(transform0);
	PX_UNUSED(transform1);
	PX_UNUSED(lastTm0);
	PX_UNUSED(lastTm1);
	PX_UNUSED(restDistance);
	PX_UNUSED(worldNormal);
	PX_UNUSED(worldPoint);
	PX_UNUSED(meshScaling);
	PX_UNUSED(triangle);
	PX_UNUSED(toiEstimate);

	return 1e10f;	//no impact
}

template<typename Geom>	
PxReal SweepGeomTriangles(GU_TRIANGLE_SWEEP_METHOD_ARGS)
{
	PX_UNUSED(meshScaling);
	PX_UNUSED(shape1);

	const PxGeometry& g = shape0;
	//Geom geom(g);
	typename ConvexGeom<Geom>::Type geom(g);

	return CCDSweep<TriangleV, Geom>(triangle, geom, transform1, transform0, lastTm1, lastTm0, FLoad(toiEstimate), worldPoint, worldNormal, restDistance+getRadius<Geom>(g) );
}

typedef PxReal (*TriangleSweepMethod) (GU_TRIANGLE_SWEEP_METHOD_ARGS);
TriangleSweepMethod g_TriangleSweepMethodTable[] = 
{
	SweepGeomTriangles<CapsuleV>,		//PxGeometryType::eSPHERE
	UnimplementedTriangleSweep,			//PxGeometryType::ePLANE
	SweepGeomTriangles<CapsuleV>,		//PxGeometryType::eCAPSULE
	SweepGeomTriangles<BoxV>,			//PxGeometryType::eBOX
	SweepGeomTriangles<ConvexHullV>,	//PxGeometryType::eCONVEXMESH
	UnimplementedTriangleSweep,			//PxGeometryType::ePARTICLESYSTEM
	UnimplementedTriangleSweep,			//PxGeometryType::eTETRAHEDRONMESH
	UnimplementedTriangleSweep,			//PxGeometryType::eTRIANGLEMESH
	UnimplementedTriangleSweep,			//PxGeometryType::eHEIGHTFIELD
	UnimplementedTriangleSweep,			//PxGeometryType::eHAIRSYSTEM
	UnimplementedTriangleSweep,			//PxGeometryType::eCUSTOM
};
PX_COMPILE_TIME_ASSERT(sizeof(g_TriangleSweepMethodTable) / sizeof(g_TriangleSweepMethodTable[0]) == PxGeometryType::eGEOMETRY_COUNT);

PxReal SweepShapeTriangle(GU_TRIANGLE_SWEEP_METHOD_ARGS)
{
	const PxGeometryType::Enum type0 = shape0.getType();
	const TriangleSweepMethod method = g_TriangleSweepMethodTable[type0];
	return method(shape0, shape1, transform0, transform1, lastTm0, lastTm1, restDistance, worldNormal, worldPoint, meshScaling, triangle, toiEstimate);
}

}
}

