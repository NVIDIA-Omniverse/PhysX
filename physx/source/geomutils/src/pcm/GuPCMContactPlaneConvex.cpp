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

#include "geomutils/PxContactBuffer.h"
#include "GuVecConvexHull.h"
#include "GuContactMethodImpl.h"
#include "GuPersistentContactManifold.h"

using namespace physx;

bool Gu::pcmContactPlaneConvex(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(shape0);
	PX_UNUSED(renderOutput);

	using namespace aos;

	PersistentContactManifold& manifold = cache.getManifold();
	PxPrefetchLine(&manifold, 256);

	// Get actual shape data
	const PxConvexMeshGeometry& shapeConvex = checkedCast<PxConvexMeshGeometry>(shape1);

	const PxTransformV transf0 = loadTransformA(transform1);//convex transform
	const PxTransformV transf1 = loadTransformA(transform0);//plane transform
	//convex to plane
	const PxTransformV curTransf(transf1.transformInv(transf0));
	
	const Vec3V vScale = V3LoadU_SafeReadW(shapeConvex.scale.scale);	// PT: safe because 'rotation' follows 'scale' in PxMeshScale
	const ConvexHullData* hullData = _getHullData(shapeConvex);

	const PxReal toleranceLength = params.mToleranceLength;
	const FloatV convexMargin = CalculatePCMConvexMargin(hullData, vScale, toleranceLength);
	
	//in world space
	const Vec3V planeNormal = V3Normalize(QuatGetBasisVector0(transf1.q));
	const Vec3V negPlaneNormal = V3Neg(planeNormal);
	
	const FloatV contactDist = FLoad(params.mContactDistance);

	//const FloatV replaceBreakingThreshold = FMul(convexMargin, FLoad(0.001f));
	const FloatV projectBreakingThreshold = FMul(convexMargin, FLoad(0.2f));
	const PxU32 initialContacts = manifold.mNumContacts;
	
	manifold.refreshContactPoints(curTransf, projectBreakingThreshold, contactDist);

	const PxU32 newContacts = manifold.mNumContacts;
	const bool bLostContacts = (newContacts != initialContacts);//((initialContacts == 0) || (newContacts != initialContacts));

	if(bLostContacts || manifold.invalidate_PrimitivesPlane(curTransf, convexMargin, FLoad(0.2f)))
	{
		const PxMatTransformV aToB(curTransf);
		const QuatV vQuat = QuatVLoadU(&shapeConvex.scale.rotation.x);

		const Mat33V vertex2Shape = ConstructVertex2ShapeMatrix(vScale, vQuat);
		
		//ML:localNormal is the local space of plane normal, however, because shape1 is box and shape0 is plane, we need to use the reverse of contact normal(which will be the plane normal) to make the refreshContactPoints
		//work out the correct pentration for points
		const Vec3V localNormal = V3UnitX();

		manifold.mNumContacts = 0;
		manifold.setRelativeTransform(curTransf);
		const PxVec3* PX_RESTRICT verts = hullData->getHullVertices();

		const PxU32 nbPolygons = hullData->mNbPolygons;

		const Vec3V n = V3Normalize(M33MulV3(vertex2Shape, aToB.rotateInv(localNormal)));
		const Vec3V nnormal = V3Neg(n);
		const FloatV zero = FZero();

		PersistentContact* manifoldContacts = PX_CP_TO_PCP(contactBuffer.contacts);
		PxU32 numContacts = 0;

		const PxMatTransformV aToBVertexSpace(aToB.p, M33MulM33(aToB.rot, vertex2Shape));

		FloatV minProj = FMax();
		PxU32 closestFaceIndex = 0;
		PxU32 polyIndex2 = 0xFFFFFFFF;

		for (PxU32 i = 0; i < nbPolygons; ++i)
		{
			const HullPolygonData& polyData = hullData->mPolygons[i];
			const Vec3V planeN = V3LoadU_SafeReadW(polyData.mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
			const FloatV proj = V3Dot(n, planeN);
			if (FAllGrtr(minProj, proj))
			{
				minProj = proj;
				closestFaceIndex = PxI32(i);
			}
		}

		const PxU32 numEdges = hullData->mNbEdges;
		const PxU8* const edgeToFace = hullData->getFacesByEdges8();

		//Loop through edges
		PxU32 closestEdge = 0xffffffff;
		//We subtract a small bias to increase the chances of selecting an edge below
		minProj = FSub(minProj, FLoad(5e-4f));
		FloatV maxDpSq = FMul(minProj, minProj);

		for (PxU32 i = 0; i < numEdges; ++i)//, inc = VecI32V_Add(inc, vOne))
		{
			const PxU32 index = i * 2;
			const PxU8 f0 = edgeToFace[index];
			const PxU8 f1 = edgeToFace[index + 1];

			const Vec3V planeNormal0 = V3LoadU_SafeReadW(hullData->mPolygons[f0].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
			const Vec3V planeNormal1 = V3LoadU_SafeReadW(hullData->mPolygons[f1].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class

			// unnormalized edge normal
			const Vec3V edgeNormal = V3Add(planeNormal0, planeNormal1);//polys[f0].mPlane.n + polys[f1].mPlane.n;
			const FloatV enMagSq = V3Dot(edgeNormal, edgeNormal);//edgeNormal.magnitudeSquared();
			//Test normal of current edge - squared test is valid if dp and maxDp both >= 0
			const FloatV dp = V3Dot(edgeNormal, nnormal);//edgeNormal.dot(normal);
			const FloatV sqDp = FMul(dp, dp);

			const BoolV con0 = FIsGrtrOrEq(dp, zero);
			const BoolV con1 = FIsGrtr(sqDp, FMul(maxDpSq, enMagSq));
			const BoolV con = BAnd(con0, con1);
			if (BAllEqTTTT(con))
			{
				maxDpSq = FDiv(sqDp, enMagSq);
				closestEdge = i;
			}
		}

		if (closestEdge != 0xffffffff)
		{
			const PxU8* FBE = edgeToFace;

			const PxU32 index = closestEdge * 2;
			const PxU32 f0 = FBE[index];
			const PxU32 f1 = FBE[index + 1];

			const Vec3V planeNormal0 = V3LoadU_SafeReadW(hullData->mPolygons[f0].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class
			const Vec3V planeNormal1 = V3LoadU_SafeReadW(hullData->mPolygons[f1].mPlane.n);	// PT: safe because 'd' follows 'n' in the plane class

			const FloatV dp0 = V3Dot(planeNormal0, nnormal);
			const FloatV dp1 = V3Dot(planeNormal1, nnormal);
			if (FAllGrtr(dp0, dp1))
			{
				closestFaceIndex = PxI32(f0);
				polyIndex2 = PxI32(f1);
			}
			else
			{
				closestFaceIndex = PxI32(f1);
				polyIndex2 = PxI32(f0);
			}
		}

		for (PxU32 index = closestFaceIndex; index != 0xFFFFFFFF; index = polyIndex2, polyIndex2 = 0xFFFFFFFF)
		{
			const HullPolygonData& face = hullData->mPolygons[closestFaceIndex];
			const PxU32 nbFaceVerts = face.mNbVerts;

			const PxU8* vertInds = hullData->getVertexData8() + face.mVRef8;

			for (PxU32 i = 0; i < nbFaceVerts; ++i)
			{
				const Vec3V pInVertexSpace = V3LoadU(verts[vertInds[i]]);

				//transform p into plane space
				const Vec3V pInPlaneSpace = aToBVertexSpace.transform(pInVertexSpace);//V3Add(aToB.p, M33MulV3(temp1, pInVertexSpace));

				const FloatV signDist = V3GetX(pInPlaneSpace);

				if (FAllGrtr(contactDist, signDist))
				{
					//transform p into shape space
					const Vec3V pInShapeSpace = M33MulV3(vertex2Shape, pInVertexSpace);
					//add to manifold

					manifoldContacts[numContacts].mLocalPointA = pInShapeSpace;
					manifoldContacts[numContacts].mLocalPointB = V3NegScaleSub(localNormal, signDist, pInPlaneSpace);
					manifoldContacts[numContacts++].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(localNormal), signDist);

					if (numContacts == 64)
					{
						manifold.reduceBatchContactsCluster(manifoldContacts, numContacts);
						numContacts = GU_MANIFOLD_CACHE_SIZE;

						for (PxU32 c = 0; c < GU_MANIFOLD_CACHE_SIZE; ++c)
							manifoldContacts[c] = manifold.mContactPoints[c];
					}
				}
			}
		}

		//reduce contacts
		//manifold.addBatchManifoldContactsCluster(manifoldContacts, numContacts);

		manifold.addBatchManifoldContacts(manifoldContacts, numContacts, toleranceLength);
	}
	
	manifold.addManifoldContactsToContactBuffer(contactBuffer, negPlaneNormal, transf1, contactDist);
#if	PCM_LOW_LEVEL_DEBUG
	manifold.drawManifold(*renderOutput, transf0, transf1);
#endif
	return manifold.getNumContacts() > 0;
}

