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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geomutils/PxContactBuffer.h"
#include "GuGJKPenetration.h"
#include "GuEPA.h"
#include "GuVecCapsule.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactGen.h"
#include "GuPCMShapeConvex.h"

using namespace physx;
using namespace Gu;
using namespace aos;

static void addToContactBuffer(PxContactBuffer& contactBuffer, const aos::Vec3VArg worldNormal, const aos::Vec3VArg worldPoint, const aos::FloatVArg penDep)
{
	PxContactPoint& contact = contactBuffer.contacts[contactBuffer.count++];
	V4StoreA(Vec4V_From_Vec3V(worldNormal), reinterpret_cast<PxF32*>(&contact.normal.x));
	V4StoreA(Vec4V_From_Vec3V(worldPoint), reinterpret_cast<PxF32*>(&contact.point.x));
	FStore(penDep, &contact.separation);

	PX_ASSERT(contact.point.isFinite());
	PX_ASSERT(contact.normal.isFinite());
	PX_ASSERT(PxIsFinite(contact.separation));

	contact.internalFaceIndex1 = PXC_CONTACT_NO_FACE_INDEX;
}

static bool fullContactsGenerationSphereConvex(const Gu::CapsuleV& capsule, const Gu::ConvexHullV& convexHull, const aos::PxTransformV& transf0,const aos::PxTransformV& transf1,
								Gu::PersistentContact* manifoldContacts, PxContactBuffer& contactBuffer, const bool idtScale, Gu::PersistentContactManifold& manifold, 
								aos::Vec3VArg normal, const aos::FloatVArg contactDist, bool doOverlapTest, PxRenderOutput* renderOutput)
{
	PX_UNUSED(renderOutput);

	Gu::PolygonalData polyData;
	getPCMConvexData(convexHull,idtScale, polyData);

	PxU8 buff[sizeof(SupportLocalImpl<ConvexHullV>)];
	SupportLocal* map = (idtScale ? static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullNoScaleV>)(static_cast<const ConvexHullNoScaleV&>(convexHull), transf1, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScale)) : 
	static_cast<SupportLocal*>(PX_PLACEMENT_NEW(buff, SupportLocalImpl<ConvexHullV>)(convexHull, transf1, convexHull.vertex2Shape, convexHull.shape2Vertex, idtScale)));

	PxU32 numContacts = 0;
	if(generateSphereFullContactManifold(capsule, polyData, map, manifoldContacts, numContacts, contactDist, normal, doOverlapTest))
	{
		if(numContacts > 0)
		{
			Gu::PersistentContact& p = manifold.getContactPoint(0);

			p.mLocalPointA = manifoldContacts[0].mLocalPointA;
			p.mLocalPointB = manifoldContacts[0].mLocalPointB;
			p.mLocalNormalPen = manifoldContacts[0].mLocalNormalPen;
			manifold.mNumContacts =1;

			//transform normal to world space
			const Vec3V worldNormal = transf1.rotate(normal);
			const Vec3V worldP = V3NegScaleSub(worldNormal, capsule.radius, transf0.p);
			const FloatV penDep = FSub(V4GetW(manifoldContacts[0].mLocalNormalPen), capsule.radius);

#if	PCM_LOW_LEVEL_DEBUG
			manifold.drawManifold(*renderOutput, transf0, transf1, capsule.radius);
#endif

			addToContactBuffer(contactBuffer, worldNormal, worldP, penDep);

			return true;
		}
	}

	return false;
}

bool Gu::pcmContactSphereConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());
	
	const PxConvexMeshGeometry& shapeConvex = checkedCast<PxConvexMeshGeometry>(shape1);
	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);

	Gu::PersistentContactManifold& manifold = cache.getManifold();

	const Vec3V zeroV = V3Zero();

	const ConvexHullData* hullData = _getHullData(shapeConvex);
	PxPrefetchLine(hullData);
	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const FloatV sphereRadius = FLoad(shapeSphere.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);
	
	//Transfer A into the local space of B
	const PxTransformV transf0 = loadTransformA(transform0);
	const PxTransformV transf1 = loadTransformA(transform1);
	const PxTransformV curRTrans(transf1.transformInv(transf0));
	const PxMatTransformV aToB(curRTrans);
	
	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV convexMargin = Gu::CalculatePCMConvexMargin(hullData, vScale, toleranceLength);

	const PxU32 initialContacts = manifold.mNumContacts;
	const FloatV minMargin = FMin(convexMargin, sphereRadius);
	const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.05f));
	
	const FloatV refreshDistance = FAdd(sphereRadius, contactDist);
	manifold.refreshContactPoints(aToB, projectBreakingThreshold, refreshDistance);
	//ML: after refreshContactPoints, we might lose some contacts
	const bool bLostContacts = (manifold.mNumContacts != initialContacts);

	if(bLostContacts || manifold.invalidate_SphereCapsule(curRTrans, minMargin))
	{
		GjkStatus status = manifold.mNumContacts > 0 ? GJK_UNDEFINED : GJK_NON_INTERSECT;

		manifold.setRelativeTransform(curRTrans);
		
		const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);  
	
		const bool idtScale = shapeConvex.scale.isIdentity();
		//use the original shape
		const ConvexHullV convexHull(hullData, V3LoadU(hullData->mCenterOfMass), vScale, vQuat, idtScale);
		//transform capsule into the local space of convexHull
		const CapsuleV capsule(aToB.p, sphereRadius);

		GjkOutput output;
		LocalConvex<CapsuleV> convexA(capsule);
		const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), convexHull.getCenter());
		if(idtScale)
		{
			const LocalConvex<ConvexHullNoScaleV> convexB(*PX_CONVEX_TO_NOSCALECONVEX(&convexHull));
			status = gjkPenetration<LocalConvex<CapsuleV>, LocalConvex<ConvexHullNoScaleV> >(convexA, convexB, initialSearchDir, contactDist, true,
				manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);
		}
		else
		{
			const LocalConvex<ConvexHullV> convexB(convexHull);
			status = gjkPenetration<LocalConvex<CapsuleV>, LocalConvex<ConvexHullV> >(convexA, convexB, initialSearchDir, contactDist, true,
				manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);
		}

		if(status == GJK_NON_INTERSECT)
		{
			return false;
		}
		else if(status == GJK_CONTACT)
		{
			Gu::PersistentContact& p = manifold.getContactPoint(0);
			p.mLocalPointA = zeroV;//sphere center
			p.mLocalPointB = output.closestB;
			p.mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
			manifold.mNumContacts =1;

#if	PCM_LOW_LEVEL_DEBUG
			manifold.drawManifold(*renderOutput, transf0, transf1, capsule.radius);
#endif
			
			//transform normal to world space
			const Vec3V worldNormal = transf1.rotate(output.normal);
			const Vec3V worldP = V3NegScaleSub(worldNormal, sphereRadius, transf0.p);
			const FloatV penDep = FSub(output.penDep, sphereRadius);
			addToContactBuffer(contactBuffer, worldNormal, worldP, penDep);
			return true;
		}
		else if(status == GJK_DEGENERATE)
		{
			Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
			
			return fullContactsGenerationSphereConvex(capsule, convexHull, transf0, transf1, manifoldContacts, contactBuffer, idtScale, 
				manifold, output.normal, contactDist, true, renderOutput);
		}
		else if (status == EPA_CONTACT)
		{
			if (idtScale)
			{
				const LocalConvex<ConvexHullNoScaleV> convexB(*PX_CONVEX_TO_NOSCALECONVEX(&convexHull));
				status= Gu::epaPenetration(convexA, convexB, manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints,
						true, FLoad(toleranceLength), output);
			}
			else
			{
				const LocalConvex<ConvexHullV> convexB(convexHull);			
				status= Gu::epaPenetration(convexA, convexB, manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints,
						true, FLoad(toleranceLength), output);
			}

			if (status == EPA_CONTACT)
			{
				Gu::PersistentContact& p = manifold.getContactPoint(0);
				p.mLocalPointA = zeroV;//sphere center
				p.mLocalPointB = output.closestB;
				p.mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
				manifold.mNumContacts = 1;

#if	PCM_LOW_LEVEL_DEBUG
				manifold.drawManifold(*renderOutput, transf0, transf1, capsule.radius);
#endif

				//transform normal to world space
				const Vec3V worldNormal = transf1.rotate(output.normal);
				const Vec3V worldP = V3NegScaleSub(worldNormal, sphereRadius, transf0.p);
				const FloatV penDep = FSub(output.penDep, sphereRadius);

				addToContactBuffer(contactBuffer, worldNormal, worldP, penDep);
				return true;
			}
			else
			{
				Gu::PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
				return fullContactsGenerationSphereConvex(capsule, convexHull, transf0, transf1, manifoldContacts, contactBuffer, idtScale,
					manifold, output.normal, contactDist, true, renderOutput);
			}
		}
	}
	else if(manifold.mNumContacts > 0)
	{
		//ML:: the manifold originally has contacts
		Gu::PersistentContact& p = manifold.getContactPoint(0);
		const Vec3V worldNormal = transf1.rotate(Vec3V_From_Vec4V(p.mLocalNormalPen));
		const Vec3V worldP = V3NegScaleSub(worldNormal, sphereRadius, transf0.p);
		const FloatV penDep = FSub(V4GetW(p.mLocalNormalPen), sphereRadius);

#if	PCM_LOW_LEVEL_DEBUG
		manifold.drawManifold(*renderOutput, transf0, transf1, sphereRadius);
#endif
	
		addToContactBuffer(contactBuffer, worldNormal, worldP, penDep);
		return true;
	}

	return false;
}

