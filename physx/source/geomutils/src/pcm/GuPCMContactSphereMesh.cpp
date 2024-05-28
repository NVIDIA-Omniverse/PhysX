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

#include "foundation/PxSort.h"
#include "GuContactMethodImpl.h"
#include "GuPCMContactConvexCommon.h"
#include "GuPCMContactMeshCallback.h"
#include "GuFeatureCode.h"
#include "GuBox.h"

using namespace physx;
using namespace Gu;
using namespace aos;

namespace
{
struct PCMSphereVsMeshContactGenerationCallback : PCMMeshContactGenerationCallback< PCMSphereVsMeshContactGenerationCallback >
{
	PCMSphereVsMeshContactGeneration	mGeneration;
	
	PCMSphereVsMeshContactGenerationCallback(
		const Vec3VArg sphereCenter,
		const FloatVArg sphereRadius,
		const FloatVArg contactDist,
		const FloatVArg replaceBreakingThreshold,
		const PxTransformV& sphereTransform,
		const PxTransformV& meshTransform,
		MultiplePersistentContactManifold& multiManifold,
		PxContactBuffer& contactBuffer,
		const PxU8* extraTriData,
		const Cm::FastVertex2ShapeScaling& meshScaling,
		bool idtMeshScale,
		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE>* deferredContacts,
		PxRenderOutput* renderOutput = NULL
	) :
		PCMMeshContactGenerationCallback<PCMSphereVsMeshContactGenerationCallback>(meshScaling, extraTriData, idtMeshScale), 
		mGeneration(sphereCenter, sphereRadius, contactDist, replaceBreakingThreshold, sphereTransform, meshTransform, multiManifold, contactBuffer, deferredContacts, renderOutput)
	{
	}

	PX_FORCE_INLINE bool doTest(const PxVec3&, const PxVec3&, const PxVec3&) { return true; }

	template<PxU32 CacheSize>
	void processTriangleCache(TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMSphereVsMeshContactGeneration>(cache);
	}
};
}

bool Gu::pcmContactSphereMesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);
	const PxTriangleMeshGeometry& shapeMesh = checkedCast<PxTriangleMeshGeometry>(shape1);

	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV sphereRadius = FLoad(shapeSphere.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);
	
	const PxTransformV sphereTransform(p0, q0);//sphere transform
	const PxTransformV meshTransform(p1, q1);//triangleMesh  
	const PxTransformV curTransform = meshTransform.transformInv(sphereTransform);
	
	// We must be in local space to use the cache
	if(multiManifold.invalidate(curTransform, sphereRadius, FLoad(0.02f)))
	{
		const FloatV replaceBreakingThreshold = FMul(sphereRadius, FLoad(0.001f));
		const PxVec3 sphereCenterShape1Space = transform1.transformInv(transform0.p);
		PxReal inflatedRadius = shapeSphere.radius + params.mContactDistance;

		const Vec3V sphereCenter = V3LoadU(sphereCenterShape1Space);

		const TriangleMesh* meshData = _getMeshData(shapeMesh);

		Cm::FastVertex2ShapeScaling meshScaling;	// PT: TODO: get rid of default ctor :(
		const bool idtMeshScale = shapeMesh.scale.isIdentity();
		if(!idtMeshScale)
			meshScaling.init(shapeMesh.scale);

		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE> delayedContacts;

		const PxU8* PX_RESTRICT extraData = meshData->getExtraTrigData();
		// mesh scale is not baked into cached verts
		PCMSphereVsMeshContactGenerationCallback callback(
			sphereCenter,
			sphereRadius,
			contactDist,
			replaceBreakingThreshold,
			sphereTransform,
			meshTransform,
			multiManifold,
			contactBuffer,
			extraData,
			meshScaling,
			idtMeshScale,
			&delayedContacts,
			renderOutput);

		PxVec3 obbCenter = sphereCenterShape1Space;
		PxVec3 obbExtents = PxVec3(inflatedRadius);
		PxMat33 obbRot(PxIdentity);
		if(!idtMeshScale)
			meshScaling.transformQueryBounds(obbCenter, obbExtents, obbRot);
		const Box obb(obbCenter, obbExtents, obbRot);

		Midphase::intersectOBB(meshData, obb, callback, true);

		callback.flushCache();

		callback.mGeneration.generateLastContacts();
		callback.mGeneration.processContacts(GU_SPHERE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PxMatTransformV aToB(curTransform);
		const FloatV projectBreakingThreshold = FMul(sphereRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(sphereRadius, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);
	}
	
	//multiManifold.drawManifold(*gRenderOutPut, sphereTransform, meshTransform);
	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, sphereTransform, meshTransform, sphereRadius);
}

static FloatV pcmDistancePointTriangleSquared(	const Vec3VArg p, 
												const Vec3VArg a, 
												const Vec3VArg b, 
												const Vec3VArg c,
												Vec3V& closestP,
												FeatureCode& fc)
{
	const FloatV zero = FZero();
	const Vec3V ab = V3Sub(b, a);
	const Vec3V ac = V3Sub(c, a);
	const Vec3V bc = V3Sub(c, b);
	const Vec3V ap = V3Sub(p, a);
	const Vec3V bp = V3Sub(p, b);
	const Vec3V cp = V3Sub(p, c);

	const FloatV d1 = V3Dot(ab, ap); //  snom
	const FloatV d2 = V3Dot(ac, ap); //  tnom
	const FloatV d3 = V3Dot(ab, bp); // -sdenom
	const FloatV d4 = V3Dot(ac, bp); //  unom = d4 - d3
	const FloatV d5 = V3Dot(ab, cp); //  udenom = d5 - d6
	const FloatV d6 = V3Dot(ac, cp); // -tdenom
	const FloatV unom = FSub(d4, d3);
	const FloatV udenom = FSub(d5, d6);

	const Vec3V n = V3Cross(ab, ac);
	const VecCrossV crossA = V3PrepareCross(ap);
	const VecCrossV crossB = V3PrepareCross(bp);
	const VecCrossV crossC = V3PrepareCross(cp);
	const Vec3V bCrossC = V3Cross(crossB, crossC);
	const Vec3V cCrossA = V3Cross(crossC, crossA);
	const Vec3V aCrossB = V3Cross(crossA, crossB);

	//const FloatV va = V3Dot(n, bCrossC);//edge region of BC, signed area rbc, u = S(rbc)/S(abc) for a
	//const FloatV vb = V3Dot(n, cCrossA);//edge region of AC, signed area rac, v = S(rca)/S(abc) for b
	//const FloatV vc = V3Dot(n, aCrossB);//edge region of AB, signed area rab, w = S(rab)/S(abc) for c
	
	//check if p in vertex region outside a
	const BoolV con00 = FIsGrtr(zero, d1); // snom <= 0
	const BoolV con01 = FIsGrtr(zero, d2); // tnom <= 0
	const BoolV con0 = BAnd(con00, con01); // vertex region a

	if(BAllEqTTTT(con0))
	{
		//Vertex 0
		fc = FC_VERTEX0;
		closestP = a;
		return V3Dot(ap, ap);
	}

	//check if p in vertex region outside b
	const BoolV con10 = FIsGrtrOrEq(d3, zero);
	const BoolV con11 = FIsGrtrOrEq(d3, d4);
	const BoolV con1 = BAnd(con10, con11); // vertex region b
	if(BAllEqTTTT(con1))
	{
		//Vertex 1
		fc = FC_VERTEX1;
		closestP = b;
		return V3Dot(bp, bp);
	}

	//check if p in vertex region outside c
	const BoolV con20 = FIsGrtrOrEq(d6, zero);
	const BoolV con21 = FIsGrtrOrEq(d6, d5); 
	const BoolV con2 = BAnd(con20, con21); // vertex region c
	if(BAllEqTTTT(con2))
	{
		//Vertex 2
		fc = FC_VERTEX2;
		closestP = c;
		return V3Dot(cp, cp);
	}

	//check if p in edge region of AB
	//const FloatV vc = FSub(FMul(d1, d4), FMul(d3, d2));
	const FloatV vc = V3Dot(n, aCrossB);//edge region of AB, signed area rab, w = S(rab)/S(abc) for c
	const BoolV con30 = FIsGrtr(zero, vc);
	const BoolV con31 = FIsGrtrOrEq(d1, zero);
	const BoolV con32 = FIsGrtr(zero, d3);
	const BoolV con3 = BAnd(con30, BAnd(con31, con32));
	if(BAllEqTTTT(con3))
	{
		// Edge 01
		fc = FC_EDGE01;
		const FloatV sScale = FDiv(d1, FSub(d1, d3));
		const Vec3V closest3 = V3ScaleAdd(ab, sScale, a);//V3Add(a, V3Scale(ab, sScale));
		const Vec3V vv = V3Sub(p, closest3);
		closestP = closest3;
		return V3Dot(vv, vv);
	}

	//check if p in edge region of BC
	//const FloatV va = FSub(FMul(d3, d6),FMul(d5, d4));
	const FloatV va = V3Dot(n, bCrossC);//edge region of BC, signed area rbc, u = S(rbc)/S(abc) for a
	
	const BoolV con40 = FIsGrtr(zero, va);
	const BoolV con41 = FIsGrtrOrEq(d4, d3);
	const BoolV con42 = FIsGrtrOrEq(d5, d6);
	const BoolV con4 = BAnd(con40, BAnd(con41, con42)); 
	if(BAllEqTTTT(con4))
	{
		// Edge 12
		fc = FC_EDGE12;
		const FloatV uScale = FDiv(unom, FAdd(unom, udenom));
		const Vec3V closest4 = V3ScaleAdd(bc, uScale, b);//V3Add(b, V3Scale(bc, uScale));
		const Vec3V vv = V3Sub(p, closest4);
		closestP = closest4;
		return V3Dot(vv, vv);
	}

	//check if p in edge region of AC
	//const FloatV vb = FSub(FMul(d5, d2), FMul(d1, d6));
	const FloatV vb = V3Dot(n, cCrossA);//edge region of AC, signed area rac, v = S(rca)/S(abc) for b
	const BoolV con50 = FIsGrtr(zero, vb);
	const BoolV con51 = FIsGrtrOrEq(d2, zero);
	const BoolV con52 = FIsGrtr(zero, d6);
	const BoolV con5 = BAnd(con50, BAnd(con51, con52));
	if(BAllEqTTTT(con5))
	{
		//Edge 20
		fc = FC_EDGE20;
		const FloatV tScale = FDiv(d2, FSub(d2, d6));
		const Vec3V closest5 = V3ScaleAdd(ac, tScale, a);//V3Add(a, V3Scale(ac, tScale));
		const Vec3V vv = V3Sub(p, closest5);
		closestP = closest5;
		return V3Dot(vv, vv);
	}

	fc = FC_FACE;

	//P must project inside face region. Compute Q using Barycentric coordinates
	const FloatV nn = V3Dot(n, n);
	const FloatV t = FDiv(V3Dot(n, V3Sub(a, p)), nn); 
	const Vec3V vv = V3Scale(n, t);
	closestP = V3Add(p, vv);
	return V3Dot(vv, vv);	
}

bool Gu::PCMSphereVsMeshContactGeneration::processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds)
{
	const FloatV zero = FZero();

	const Vec3V v0 = V3LoadU(verts[0]);
	const Vec3V v1 = V3LoadU(verts[1]);
	const Vec3V v2 = V3LoadU(verts[2]);

	const Vec3V v10 = V3Sub(v1, v0);
	const Vec3V v20 = V3Sub(v2, v0);

	const Vec3V n = V3Normalize(V3Cross(v10, v20));//(p1 - p0).cross(p2 - p0).getNormalized();
	const FloatV d = V3Dot(v0, n);//d = -p0.dot(n);

	const FloatV dist0 = FSub(V3Dot(mSphereCenter, n), d);//p.dot(n) + d;
	
	// Backface culling
	if(FAllGrtr(zero, dist0))
		return false;

	Vec3V closestP;
	//mSphereCenter will be in the local space of the triangle mesh
	FeatureCode fc;
	FloatV sqDist = pcmDistancePointTriangleSquared(mSphereCenter, v0, v1, v2, closestP, fc);

	 //sphere overlap with triangles
	if (FAllGrtr(mSqInflatedSphereRadius, sqDist))
	{
		//sphere center is on the triangle surface, we take triangle normal as the patchNormal. Otherwise, we need to calculate the patchNormal
		if(fc==FC_FACE)
		{
			const Vec3V patchNormal = n;

			const FloatV dist = FSqrt(sqDist);

			mEdgeCache.addData(CachedEdge(vertInds[0], vertInds[1]));
			mEdgeCache.addData(CachedEdge(vertInds[1], vertInds[2]));
			mEdgeCache.addData(CachedEdge(vertInds[2], vertInds[0]));

			mVertexCache.addData(CachedVertex(vertInds[0]));
			mVertexCache.addData(CachedVertex(vertInds[1]));
			mVertexCache.addData(CachedVertex(vertInds[2]));

			addToPatch(closestP, patchNormal, dist, triangleIndex);
		}
		else
		{
			const Vec3V patchNormal = V3Normalize(V3Sub(mSphereCenter, closestP));

			//ML : defer the contacts generation
			const PxU32 nb = sizeof(PCMDeferredPolyData) / sizeof(PxU32);
			PxU32 newSize = nb + mDeferredContacts->size();
			if(mDeferredContacts->capacity() < newSize)
				mDeferredContacts->reserve(2*(newSize+1));
			PCMDeferredPolyData* PX_RESTRICT data = reinterpret_cast<PCMDeferredPolyData*>(mDeferredContacts->end());
			mDeferredContacts->forceSize_Unsafe(newSize);

			SortedTriangle sortedTriangle;
			sortedTriangle.mSquareDist = sqDist;
			sortedTriangle.mIndex = mSortedTriangle.size();
			mSortedTriangle.pushBack(sortedTriangle);

			data->mTriangleIndex = triangleIndex;
			data->mFeatureIndex = fc;
			data->triFlags32 = PxU32(triFlags);
			data->mInds[0] = vertInds[0];
			data->mInds[1] = vertInds[1];
			data->mInds[2] = vertInds[2];
			V3StoreU(closestP, data->mVerts[0]);
			V3StoreU(patchNormal, data->mVerts[1]);
			V3StoreU(V3Splat(sqDist), data->mVerts[2]);
		}
	}
	return true;
}

void Gu::PCMSphereVsMeshContactGeneration::addToPatch(const Vec3VArg contactP, const Vec3VArg patchNormal, const FloatV dist, PxU32 triangleIndex)
{
	PX_ASSERT(mNumContactPatch < PCM_MAX_CONTACTPATCH_SIZE);

	const Vec3V sphereCenter = V3Zero(); // in sphere local space

	bool foundPatch = false;
	if (mNumContactPatch > 0)
	{
		if (FAllGrtr(V3Dot(mContactPatch[mNumContactPatch - 1].mPatchNormal, patchNormal), mAcceptanceEpsilon))
		{
			PCMContactPatch& patch = mContactPatch[mNumContactPatch - 1];

			PX_ASSERT((patch.mEndIndex - patch.mStartIndex) == 1);

			if (FAllGrtr(patch.mPatchMaxPen, dist))
			{
				//overwrite the old contact
				mManifoldContacts[patch.mStartIndex].mLocalPointA = sphereCenter;//in sphere's space
				mManifoldContacts[patch.mStartIndex].mLocalPointB = contactP;
				mManifoldContacts[patch.mStartIndex].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(patchNormal), dist);
				mManifoldContacts[patch.mStartIndex].mFaceIndex = triangleIndex;
				patch.mPatchMaxPen = dist;
			}

			foundPatch = true;
		}
	}
	if (!foundPatch)
	{
		mManifoldContacts[mNumContacts].mLocalPointA = sphereCenter;//in sphere's space
		mManifoldContacts[mNumContacts].mLocalPointB = contactP;
		mManifoldContacts[mNumContacts].mLocalNormalPen = V4SetW(Vec4V_From_Vec3V(patchNormal), dist);
		mManifoldContacts[mNumContacts++].mFaceIndex = triangleIndex;

		mContactPatch[mNumContactPatch].mStartIndex = mNumContacts - 1;
		mContactPatch[mNumContactPatch].mEndIndex = mNumContacts;
		mContactPatch[mNumContactPatch].mPatchMaxPen = dist;
		mContactPatch[mNumContactPatch++].mPatchNormal = patchNormal;
	}

	PX_ASSERT(mNumContactPatch < PCM_MAX_CONTACTPATCH_SIZE);

	if (mNumContacts >= 16)
	{
		PX_ASSERT(mNumContacts <= 64);
		processContacts(GU_SPHERE_MANIFOLD_CACHE_SIZE);
	}
}

void Gu::PCMSphereVsMeshContactGeneration::generateLastContacts()
{
	// Process delayed contacts
	PxU32 nbSortedTriangle = mSortedTriangle.size();

	if (nbSortedTriangle)
	{
		PxSort(mSortedTriangle.begin(), mSortedTriangle.size(), PxLess<SortedTriangle>());

		const PCMDeferredPolyData* PX_RESTRICT cd = reinterpret_cast<const PCMDeferredPolyData*>(mDeferredContacts->begin());
		
		for (PxU32 i = 0; i < nbSortedTriangle; ++i)
		{
			const PCMDeferredPolyData& currentContact = cd[mSortedTriangle[i].mIndex];
			const PxU32 ref0 = currentContact.mInds[0];
			const PxU32 ref1 = currentContact.mInds[1];
			const PxU32 ref2 = currentContact.mInds[2];

			//if addData sucessful, which means mEdgeCache doesn't have the edge
		    const bool noEdge01 = mEdgeCache.addData(CachedEdge(ref0, ref1));
			const bool noEdge12 = mEdgeCache.addData(CachedEdge(ref1, ref2));
			const bool noEdge20 = mEdgeCache.addData(CachedEdge(ref2, ref0));
		
			const bool noVertex0 = mVertexCache.addData(CachedVertex(ref0));
			const bool noVertex1 = mVertexCache.addData(CachedVertex(ref1));
			const bool noVertex2 = mVertexCache.addData(CachedVertex(ref2));

			bool needsProcessing = false;
			{
				switch(currentContact.mFeatureIndex)
				{
					case FC_VERTEX0:
						needsProcessing = noVertex0;
						break;

					case FC_VERTEX1:
						needsProcessing = noVertex1;
						break;

					case FC_VERTEX2:
						needsProcessing = noVertex2;
						break;

					case FC_EDGE01:
						needsProcessing = noEdge01;
						break;

					case FC_EDGE12:
						needsProcessing = noEdge12;
						break;

					case FC_EDGE20:
						needsProcessing = noEdge20;
						break;

					case FC_FACE:
					case FC_UNDEFINED:
						PX_ASSERT(0);	// PT: should not be possible
						break;
				};
			}

			if (needsProcessing)
			{
				//we store the contact, patch normal and sq distance in the vertex memory in PCMDeferredPolyData
				const Vec3V contactP = V3LoadU(currentContact.mVerts[0]);
				const Vec3V patchNormal = V3LoadU(currentContact.mVerts[1]);
				const FloatV sqDist = FLoad(currentContact.mVerts[2].x);
				const FloatV dist = FSqrt(sqDist);
				addToPatch(contactP, patchNormal, dist, currentContact.mTriangleIndex);
			}
		}
	}
}


