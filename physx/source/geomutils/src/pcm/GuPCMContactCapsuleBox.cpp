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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "geomutils/PxContactBuffer.h"
#include "GuVecBox.h"
#include "GuVecCapsule.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactGen.h"
#include "GuPCMShapeConvex.h"
#include "GuGJKPenetration.h"
#include "GuEPA.h"

using namespace physx;
using namespace Gu;
using namespace aos;

static bool fullContactsGenerationCapsuleBox(const CapsuleV& capsule, const BoxV& box, const PxVec3 halfExtents,  const PxMatTransformV& aToB, const PxTransformV& transf0, const PxTransformV& transf1,
								PersistentContact* manifoldContacts, PxU32& numContacts, PxContactBuffer& contactBuffer, PersistentContactManifold& manifold, Vec3V& normal, const Vec3VArg closest,
								const PxReal boxMargin, const FloatVArg contactDist, const bool doOverlapTest, const PxReal toleranceScale, PxRenderOutput* renderOutput)
{

	PolygonalData polyData;
	PCMPolygonalBox polyBox(halfExtents);
	polyBox.getPolygonalData(&polyData);

	const Mat33V identity = M33Identity();
	SupportLocalImpl<BoxV> map(box, transf1, identity, identity);

	PxU32 origContacts = numContacts;
	if (generateCapsuleBoxFullContactManifold(capsule, polyData, &map, aToB, manifoldContacts, numContacts, contactDist, normal, closest, boxMargin, doOverlapTest, toleranceScale, renderOutput))
	{
		//EPA has contacts and we have new contacts, we discard the EPA contacts
		if(origContacts != 0 && numContacts != origContacts)
		{
			numContacts--;
			manifoldContacts++;
		}

		manifold.addBatchManifoldContacts2(manifoldContacts, numContacts);
		
		normal = transf1.rotate(normal);
		
		manifold.addManifoldContactsToContactBuffer(contactBuffer, normal, normal, transf0, capsule.radius, contactDist);
	
		return true;	
	}

	return false;
}

bool Gu::pcmContactCapsuleBox(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxCapsuleGeometry& shapeCapsule = checkedCast<PxCapsuleGeometry>(shape0);
	const PxBoxGeometry& shapeBox = checkedCast<PxBoxGeometry>(shape1);

	PersistentContactManifold& manifold = cache.getManifold();
	PxPrefetchLine(&manifold, 256);

	PX_ASSERT(transform1.q.isSane());
	PX_ASSERT(transform0.q.isSane());  

	const Vec3V boxExtents = V3LoadU(shapeBox.halfExtents);

	const FloatV contactDist = FLoad(params.mContactDistance);

	const PxTransformV transf0 = loadTransformA(transform0);
	const PxTransformV transf1 = loadTransformA(transform1);

	const PxTransformV curRTrans = transf1.transformInv(transf0);
	const PxMatTransformV aToB_(curRTrans);

	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const FloatV capsuleHalfHeight = FLoad(shapeCapsule.halfHeight);

	const PxU32 initialContacts = manifold.mNumContacts;

	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV boxMargin = Gu::CalculatePCMBoxMargin(boxExtents, toleranceLength);
	
	const FloatV minMargin = FMin(boxMargin, capsuleRadius);

	const FloatV projectBreakingThreshold = FMul(minMargin, FLoad(0.8f));
	
	const FloatV refreshDist = FAdd(contactDist, capsuleRadius);
	//refreshContactPoints remove invalid contacts from the manifold and update the number correspondingly
	manifold.refreshContactPoints(aToB_, projectBreakingThreshold, refreshDist);

	const bool bLostContacts = (manifold.mNumContacts != initialContacts);

	if(bLostContacts || manifold.invalidate_SphereCapsule(curRTrans, minMargin))	
	{
		GjkStatus status = manifold.mNumContacts > 0 ? GJK_UNDEFINED : GJK_NON_INTERSECT;

		manifold.setRelativeTransform(curRTrans);
		const PxMatTransformV aToB(curRTrans);
		
		BoxV box(transf1.p, boxExtents);
		//transform capsule into the local space of box
		CapsuleV capsule(aToB.p, aToB.rotate(V3Scale(V3UnitX(), capsuleHalfHeight)), capsuleRadius);
		const LocalConvex<CapsuleV> convexA(capsule);
		const LocalConvex<BoxV> convexB(box);
		GjkOutput output;

		const Vec3V initialSearchDir = V3Sub(capsule.getCenter(), box.getCenter());
		status = gjkPenetration<LocalConvex<CapsuleV>, LocalConvex<BoxV> >(convexA, convexB, initialSearchDir, contactDist, true, 
			manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, output);

		PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
		PxU32 numContacts = 0;
		bool doOverlapTest = false;
		if(status == GJK_NON_INTERSECT)
		{
			return false;
		}
		else if(status == GJK_DEGENERATE)
		{
			return fullContactsGenerationCapsuleBox(capsule, box, shapeBox.halfExtents,  aToB, transf0, transf1, manifoldContacts, numContacts, contactBuffer,
				manifold, output.normal, output.closestB, box.getMarginF(), contactDist, true, params.mToleranceLength, renderOutput);
		}
		else 
		{
			if(status == GJK_CONTACT)
			{
				const Vec3V localPointA = aToB.transformInv(output.closestA);//curRTrans.transformInv(closestA);
				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
				//Add contact to contact stream
				manifoldContacts[numContacts].mLocalPointA = localPointA;
				manifoldContacts[numContacts].mLocalPointB = output.closestB;
				manifoldContacts[numContacts++].mLocalNormalPen = localNormalPen;
			}
			else
			{
				PX_ASSERT(status == EPA_CONTACT);
	
				status= epaPenetration(convexA, convexB,  manifold.mAIndice, manifold.mBIndice, manifold.mNumWarmStartPoints, 
					 true, FLoad(toleranceLength), output);
				if(status == EPA_CONTACT)
				{
					const Vec3V localPointA = aToB.transformInv(output.closestA);//curRTrans.transformInv(closestA);
					const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
					//Add contact to contact stream
					manifoldContacts[numContacts].mLocalPointA = localPointA;
					manifoldContacts[numContacts].mLocalPointB = output.closestB;
					manifoldContacts[numContacts++].mLocalNormalPen = localNormalPen;

				}
				else
				{
					doOverlapTest = true;
				}

			}
			

			if(initialContacts == 0 || bLostContacts || doOverlapTest)
			{
				return fullContactsGenerationCapsuleBox(capsule, box, shapeBox.halfExtents,  aToB, transf0, transf1, manifoldContacts, numContacts, contactBuffer, manifold, output.normal, 
					output.closestB, box.getMarginF(), contactDist, doOverlapTest, params.mToleranceLength, renderOutput);
			}
			else
			{
				
				//The contacts is either come from GJK or EPA
				const FloatV replaceBreakingThreshold = FMul(minMargin, FLoad(0.1f));
				const Vec4V localNormalPen = V4SetW(Vec4V_From_Vec3V(output.normal), output.penDep);
				manifold.addManifoldPoint2(curRTrans.transformInv(output.closestA), output.closestB, localNormalPen, replaceBreakingThreshold);
				
				const Vec3V normal = transf1.rotate(output.normal);
				manifold.addManifoldContactsToContactBuffer(contactBuffer, normal, normal, transf0, capsuleRadius, contactDist);
			
				return true;
			}
		}	
	}
	else if(manifold.getNumContacts() > 0)
	{
		const Vec3V worldNormal = manifold.getWorldNormal(transf1);
		manifold.addManifoldContactsToContactBuffer(contactBuffer, worldNormal, worldNormal, transf0, capsuleRadius, contactDist);
		return true;
	}

	return false;
}
