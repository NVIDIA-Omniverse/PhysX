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

#include "GuVecTriangle.h"
#include "GuPCMContactConvexCommon.h"
#include "GuConvexEdgeFlags.h"
#include "GuBarycentricCoordinates.h"

using namespace physx;
using namespace Gu;
using namespace aos;

// This function adds the newly created manifold contacts to a new patch or existing patches 
void PCMConvexVsMeshContactGeneration::addContactsToPatch(const Vec3VArg patchNormal, PxU32 previousNumContacts)
{
	const Vec3V patchNormalInTriangle = mMeshToConvex.rotateInv(patchNormal);
	
	const PxU32 newContacts = mNumContacts - previousNumContacts;

	if(newContacts > GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE)
	{
		//if the current created manifold contacts are more than GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE(4) points, we will reduce the total numContacts 
		//to GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE. However, after we add these points into a patch, the patch contacts will be variable. Then we will 
		//do contact reduction for that patch in the processContacts. After the contact reduction, there will be no more than GU_SINGLE_MANIFOLD_CACHE_SIZE(6) 
		//contacts inside a signlePersistentContactManifold
		SinglePersistentContactManifold::reduceContacts(&mManifoldContacts[previousNumContacts], newContacts);
		mNumContacts = previousNumContacts + GU_SINGLE_MANIFOLD_SINGLE_POLYGONE_CACHE_SIZE;  
	}   

	//get rid of duplicate manifold contacts for the newly created contacts
	for(PxU32 i = previousNumContacts; i<mNumContacts; ++i)
	{
		for(PxU32 j=i+1; j<mNumContacts; ++j)
		{
			Vec3V dif = V3Sub(mManifoldContacts[j].mLocalPointB, mManifoldContacts[i].mLocalPointB);
			FloatV d = V3Dot(dif, dif);
			if(FAllGrtr(mSqReplaceBreakingThreshold, d))
			{
				mManifoldContacts[j] = mManifoldContacts[mNumContacts-1];
				mNumContacts--;
				j--;
			}
		}
	}

	//calculate the maxPen and transform the patch normal and localPointB into mesh's local space
	FloatV maxPen = FMax();
	for(PxU32 i = previousNumContacts; i<mNumContacts; ++i)
	{
		const FloatV pen = V4GetW(mManifoldContacts[i].mLocalNormalPen);
		mManifoldContacts[i].mLocalNormalPen = V4SetW(patchNormalInTriangle, pen);
		mManifoldContacts[i].mLocalPointB = mMeshToConvex.transformInv(mManifoldContacts[i].mLocalPointB);
		maxPen = FMin(maxPen, pen);
	}

	//Based on the patch normal and add the newly avaiable manifold points to the corresponding patch
	addManifoldPointToPatch(patchNormalInTriangle, maxPen, previousNumContacts);
	
	PX_ASSERT(mNumContactPatch <PCM_MAX_CONTACTPATCH_SIZE);
	if(mNumContacts >= GU_MESH_CONTACT_REDUCTION_THRESHOLD)
	{
		PX_ASSERT(mNumContacts <= PxContactBuffer::MAX_CONTACTS);
		processContacts(GU_SINGLE_MANIFOLD_CACHE_SIZE);
	}
}

void PCMConvexVsMeshContactGeneration::generateLastContacts()
{
	// Process delayed contacts
	PxU32 nbEntries = mDeferredContacts->size();

	if(nbEntries)
	{
		nbEntries /= sizeof(PCMDeferredPolyData)/sizeof(PxU32);

		const PCMDeferredPolyData* PX_RESTRICT cd = reinterpret_cast<const PCMDeferredPolyData*>(mDeferredContacts->begin());
		for(PxU32 i=0;i<nbEntries;i++)
		{
			const PCMDeferredPolyData& currentContact = cd[i];  

			const PxU32 ref0 = currentContact.mInds[0];
			const PxU32 ref1 = currentContact.mInds[1];  
			const PxU32 ref2 = currentContact.mInds[2];

			const PxU8 triFlags = PxU8(currentContact.triFlags32);

			const bool needsProcessing =	(((triFlags & ETD_CONVEX_EDGE_01) != 0 || mEdgeCache.get(CachedEdge(ref0, ref1)) == NULL)) && 
											(((triFlags & ETD_CONVEX_EDGE_12) != 0 || mEdgeCache.get(CachedEdge(ref1, ref2)) == NULL)) && 
											(((triFlags & ETD_CONVEX_EDGE_20) != 0 || mEdgeCache.get(CachedEdge(ref2, ref0)) == NULL));
			
			if(needsProcessing)
			{
				const TriangleV localTriangle(currentContact.mVerts);
				Vec3V patchNormal;
				const PxU32 previousNumContacts = mNumContacts;
				//the localTriangle is in the convex space
				//Generate contacts - we didn't generate contacts with any neighbours
				generatePolyDataContactManifold(localTriangle, currentContact.mFeatureIndex, currentContact.mTriangleIndex, triFlags, mManifoldContacts, mNumContacts, mContactDist, patchNormal);

				FloatV v, w;
				const FloatV upperBound = FLoad(0.97f);
				const FloatV lowerBound = FSub(FOne(), upperBound);
				PxU32 currentContacts = mNumContacts;
				for(PxU32 j=currentContacts; j>previousNumContacts; --j)
				{
					PxU32 ind = j-1;
					//calculate the barycentric coordinate of the contacts in localTriangle, p = a + v(b-a) + w(c-a)., p=ua+vb+wc
					barycentricCoordinates(mManifoldContacts[ind].mLocalPointB, localTriangle.verts[0], localTriangle.verts[1], localTriangle.verts[2], v, w);
					//const FloatV u = FSub(one, FAdd(v, w)); 

					bool keepContact = true;
					if(FAllGrtr(v, upperBound))//v > upperBound
					{
						//vertex1
						keepContact = !mVertexCache.contains(CachedVertex(ref1));
					}
					else if(FAllGrtr(w, upperBound))// w > upperBound
					{
						//vertex2
						keepContact = !mVertexCache.contains(CachedVertex(ref2));
					}
					else if(FAllGrtrOrEq(lowerBound, FAdd(v, w))) // u(1-(v+w)) > upperBound
					{
						//vertex0
						keepContact = !mVertexCache.contains(CachedVertex(ref0));
					}
					
					if(!keepContact)
					{
						//ML: if feature code is any of the vertex in this triangle and we have generated contacts with any other triangles which contains this vertex, we should drop it
						currentContacts--;

						for(PxU32 k = ind; k < currentContacts; ++k)
						{
							mManifoldContacts[k] = mManifoldContacts[k+1];
						}
					}	
				}

				mNumContacts = currentContacts;

				if(currentContacts > previousNumContacts)
				{
					addContactsToPatch(patchNormal, previousNumContacts);
				}
			}
		}
	}
}

bool PCMConvexVsMeshContactGeneration::processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds)
{
	const Mat33V identity = M33Identity();
	const FloatV zero = FZero();

	const Vec3V v0 = V3LoadU(verts[0]);
	const Vec3V v1 = V3LoadU(verts[1]);
	const Vec3V v2 = V3LoadU(verts[2]);

	const Vec3V v10 = V3Sub(v1, v0);
	const Vec3V v20 = V3Sub(v2, v0);

	const Vec3V n = V3Normalize(V3Cross(v10, v20));//(p1 - p0).cross(p2 - p0).getNormalized();
	const FloatV d = V3Dot(v0, n);//d = -p0.dot(n);

	const FloatV dist = FSub(V3Dot(mHullCenterMesh, n), d);//p.dot(n) + d;
	
	// Backface culling
	if(FAllGrtr(zero, dist))
		return false;

	//tranform verts into the box local space
	const Vec3V locV0 = mMeshToConvex.transform(v0);
	const Vec3V locV1 = mMeshToConvex.transform(v1);
	const Vec3V locV2 = mMeshToConvex.transform(v2);

	const TriangleV localTriangle(locV0, locV1, locV2);

	{
		SupportLocalImpl<TriangleV> localTriMap(localTriangle, mConvexTransform, identity, identity, true);

		const PxU32 previousNumContacts = mNumContacts;
		Vec3V patchNormal;

		generateTriangleFullContactManifold(localTriangle, triangleIndex, vertInds, triFlags, mPolyData, &localTriMap, mPolyMap, mManifoldContacts, mNumContacts, mContactDist, patchNormal);
		
		if(mNumContacts > previousNumContacts)
		{
#if PCM_LOW_LEVEL_DEBUG
			PersistentContactManifold::drawTriangle(*mRenderOutput, mMeshTransform.transform(v0), mMeshTransform.transform(v1), mMeshTransform.transform(v2), 0x00ff00);
#endif
			const bool inActiveEdge0 = (triFlags & ETD_CONVEX_EDGE_01) == 0;
			const bool inActiveEdge1 = (triFlags & ETD_CONVEX_EDGE_12) == 0;
			const bool inActiveEdge2 = (triFlags & ETD_CONVEX_EDGE_20) == 0;

			if(inActiveEdge0)
				mEdgeCache.addData(CachedEdge(vertInds[0], vertInds[1]));
			if(inActiveEdge1)
				mEdgeCache.addData(CachedEdge(vertInds[1], vertInds[2]));
			if(inActiveEdge2)
				mEdgeCache.addData(CachedEdge(vertInds[2], vertInds[0]));

			mVertexCache.addData(CachedVertex(vertInds[0]));
			mVertexCache.addData(CachedVertex(vertInds[1]));
			mVertexCache.addData(CachedVertex(vertInds[2]));

			addContactsToPatch(patchNormal, previousNumContacts);
		}
	}
	return true;
}

bool PCMConvexVsMeshContactGeneration::processTriangle(const PolygonalData& polyData, const SupportLocal* polyMap, const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const FloatVArg inflation, bool isDoubleSided, 
													   const PxTransformV& convexTransform, const PxMatTransformV& meshToConvex, MeshPersistentContact* manifoldContacts, PxU32& numContacts)
{
	const Mat33V identity = M33Identity();
	const FloatV zero = FZero();

	const Vec3V v0 = V3LoadU(verts[0]);
	const Vec3V v1 = V3LoadU(verts[1]);
	const Vec3V v2 = V3LoadU(verts[2]);

	//tranform verts into the box local space
	const Vec3V locV0 = meshToConvex.transform(v0);
	const Vec3V locV1 = meshToConvex.transform(v1);
	const Vec3V locV2 = meshToConvex.transform(v2);

	const Vec3V v10 = V3Sub(locV1, locV0);
	const Vec3V v20 = V3Sub(locV2, locV0);

	const Vec3V n = V3Normalize(V3Cross(v10, v20));//(p1 - p0).cross(p2 - p0).getNormalized();
	const FloatV d = V3Dot(locV0, n);//d = -p0.dot(n);

	const FloatV dist = FSub(V3Dot(polyMap->shapeSpaceCenterOfMass, n), d);//p.dot(n) + d;
	
	// Backface culling
	const bool culled = !isDoubleSided && (FAllGrtr(zero, dist));
	if(culled)
		return false;

	const TriangleV localTriangle(locV0, locV1, locV2);

	SupportLocalImpl<TriangleV> localTriMap(localTriangle, convexTransform, identity, identity, true);

	Vec3V patchNormal;

	generateTriangleFullContactManifold(localTriangle, triangleIndex, triFlags, polyData, &localTriMap, polyMap, manifoldContacts, numContacts, inflation, patchNormal);
	
	return true;
}

