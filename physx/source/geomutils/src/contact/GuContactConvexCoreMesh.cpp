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

#include "GuContactPolygonPolygon.h"
#include "GuContactMethodImpl.h"
#include "GuMidphaseInterface.h"
#include "GuHeightFieldUtil.h"
#include "GuEntityReport.h"
#include "GuBounds.h"
#include "GuConvexGeometry.h"
#include "GuConvexSupport.h"
#include "GuContactReduction.h"
#include <GuTriangleMesh.h>

using namespace physx;
using namespace Gu;
using namespace Cm;
using namespace aos;
using namespace intrinsics;

namespace
{
	struct TriangleMeshTriangles
	{
		const TriangleMesh* data;
		const PxMeshScale& scale;

		TriangleMeshTriangles(const TriangleMesh* _data, const PxMeshScale& _scale)
			:
			data(_data), scale(_scale)
		{}

		void getVertexIndices(PxU32 triIndex, PxU32& i0, PxU32& i1, PxU32& i2) const
		{
			const void* tris = data->getTriangles();
			const bool ints16bit = data->has16BitIndices();
			getVertexRefs(triIndex, i0, i1, i2, tris, ints16bit);
		}

		PxVec3 getVertex(PxU32 vertIndex) const
		{
			const PxVec3* verts = data->getVertices();
			return scale.transform(verts[vertIndex]);
		}

		bool hasAdjacency() const
		{
			return data->getAdjacencies() != NULL;
		}

		PxU32 getAdjacentTriIndex(PxU32 triIndex, PxU32 edgIndex) const
		{
			const PxU32* adjucent = data->getAdjacencies();
			return adjucent[triIndex * 3 + edgIndex];
		}
	};

	struct HeightFieldTriangles
	{
		const HeightFieldUtil& hfUtil;

		HeightFieldTriangles(const HeightFieldUtil& _hfUtil)
			:
			hfUtil(_hfUtil)
		{}

		void getVertexIndices(PxU32 triIndex, PxU32& i0, PxU32& i1, PxU32& i2) const
		{
			hfUtil.mHeightField->getTriangleVertexIndices(triIndex, i0, i1, i2);
		}

		PxVec3 getVertex(PxU32 vertIndex) const
		{
			PxVec3 v = hfUtil.mHeightField->getVertex(vertIndex);
			PxVec3 s(hfUtil.mHfGeom->rowScale, hfUtil.mHfGeom->heightScale, hfUtil.mHfGeom->columnScale);
			return PxVec3(v.x * s.x, v.y * s.y, v.z * s.z);
		}

		bool hasAdjacency() const
		{
			return true;
		}

		PxU32 getAdjacentTriIndex(PxU32 triIndex, PxU32 edgIndex) const
		{
			PxU32 adjucent[3];
			hfUtil.mHeightField->getTriangleAdjacencyIndices(triIndex, 0, 0, 0, adjucent[0], adjucent[1], adjucent[2]);
			return adjucent[edgIndex];
		}
	};

	PxVec3 computeBarycentric(const PxVec3& a, const PxVec3& b, const PxVec3& c, const PxVec3& p)
	{
		PxVec4 bary;
		PxComputeBarycentric(a, b, c, p, bary);
		//PxReal u = bary.x, v = bary.y, w = bary.z;
		//PX_ASSERT((a * u + b * v + c * w - p).magnitude() < 1e-3f); // VR: find out why this asserts sometimes
		return bary.getXYZ();
	}

	template <typename TriangleSource>
	bool validateContact(const PxVec3& normal, const PxVec3& pointB, PxU32 triIndex, const TriangleSource& tris)
	{
		const PxReal eps = 1e-5f;

		PxU32 i0, i1, i2;
		tris.getVertexIndices(triIndex, i0, i1, i2);

		const PxVec3 v0 = tris.getVertex(i0),
					 v1 = tris.getVertex(i1),
					 v2 = tris.getVertex(i2);

		const PxVec3 tn = (v1 - v0).cross(v2 - v0).getNormalized();
		// close enough to a face contact
		if (tn.dot(normal) > 0.99f)
			// better to accept
			return true;

		const PxVec3 bc = computeBarycentric(v0, v1, v2, pointB);

		// face contact
		if (bc.x > eps && bc.x < 1.0f - eps &&
			bc.y > eps && bc.y < 1.0f - eps &&
			bc.z > eps && bc.z < 1.0f - eps)
			// always accept
			return true;

		// vertex contact
		if (bc.x > 1.0f - eps ||
			bc.y > 1.0f - eps ||
			bc.z > 1.0f - eps)
		{
			PxU32 vrtIndex = 0xffffffff;
			if (tris.hasAdjacency())
			{
				if (bc.x > 1.0f - eps)
					vrtIndex = 0;
				else if (bc.y > 1.0f - eps)
					vrtIndex = 1;
				else if (bc.z > 1.0f - eps)
					vrtIndex = 2;
			}

			if (vrtIndex != 0xffffffff)
			{
				PxU32 ai[] = { i0, i1, i2 };
				PxU32 ai0 = ai[vrtIndex];
				PxU32 adjIndex = tris.getAdjacentTriIndex(triIndex, (vrtIndex + 2) % 3);
				while (adjIndex != triIndex && adjIndex != 0xffffffff)
				{
					// walking through the adjucent triangles surrounding the vertex and checking
					// if any other end of the edges sharing the vertex projects onto the contact
					// normal higher than the vertex itself.it'd meand that the contact normal is
					// out of the vertex's voronoi region.
					PxU32 bi[3]; tris.getVertexIndices(adjIndex, bi[0], bi[1], bi[2]);
					for (PxU32 i = 0; i < 3; ++i)
					{
						PxU32 bi0 = bi[i], bi1 = bi[(i + 1) % 3], bi2 = bi[(i + 2) % 3];
						if (bi1 == ai0)
						{
							const PxVec3 bv0 = tris.getVertex(bi0),
										 bv1 = tris.getVertex(bi1),
										 bv2 = tris.getVertex(bi2);
							const PxReal bd10 = normal.dot((bv0 - bv1).getNormalized()),
										 bd12 = normal.dot((bv2 - bv1).getNormalized());

							if (bd10 > eps || bd12 > eps)
								// the vertex is hidden by one of the adjacent
								// edges we can't collide with this vertex
								return false;

							// next triangle to check
							adjIndex = tris.getAdjacentTriIndex(adjIndex, i);
							break;
						}
					}
				}
			}

			return true;
		}

		// edge contact
		PxU32 edgIndex = 0xffffffff;
		if (tris.hasAdjacency())
		{
			if (bc.x < eps)
				edgIndex = 1;
			else if (bc.y < eps)
				edgIndex = 2;
			else if (bc.z < eps)
				edgIndex = 0;
		}

		if (edgIndex != 0xffffffff)
		{
			PxU32 ai[] = { i0, i1, i2 };
			PxU32 ai0 = ai[edgIndex], ai1 = ai[(edgIndex + 1) % 3];
			PxU32 adjIndex = tris.getAdjacentTriIndex(triIndex, edgIndex);
			if (adjIndex != 0xffffffff)
			{
				// testing if the adjacent triangle's vertex opposite to this edge
				// projects onto the contact normal higher than the edge itself. it'd
				// mean that the normal is out of the edge's voronoi region.
				PxU32 bi[3]; tris.getVertexIndices(adjIndex, bi[0], bi[1], bi[2]);
				for (PxU32 i = 0; i < 3; ++i)
				{
					PxU32 bi0 = bi[i], bi1 = bi[(i + 1) % 3], bi2 = bi[(i + 2) % 3];
					if (bi0 == ai1 && bi1 == ai0)
					{
						const PxVec3 bv1 = tris.getVertex(bi1),
									 bv2 = tris.getVertex(bi2);
						const PxReal bd12 = normal.dot((bv2 - bv1).getNormalized());

						if (bd12 > eps)
							// the edge is hidden by the adjacent triangle
							// we can't collide with this edge
							return false;
					}
				}
			}
		}

		return true;
	}
}

bool Gu::contactConvexCoreTrimesh(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	struct Callback : MeshHitCallback<PxGeomRaycastHit>
	{
		const Gu::ConvexShape& mConvex;
		const PxMeshScale& mScale;
		const TriangleMesh* mData;
		const PxReal mContactDist;
		const PxReal mTriMargin;
		const PxTransform& mTransform;
		Gu::Contact& mContact;

		PxRenderOutput* mRenderOutput;

		Callback(const Gu::ConvexShape& convex, const PxMeshScale& scale, const TriangleMesh* data, PxReal contactDist,
			PxReal triMargin, const PxTransform& transform, Gu::Contact& contact, PxRenderOutput* renderOutput)
			:
			MeshHitCallback<PxGeomRaycastHit>(CallbackMode::eMULTIPLE),
			mConvex(convex), mScale(scale), mData(data), mContactDist(contactDist),
			mTriMargin(triMargin), mTransform(transform), mContact(contact), mRenderOutput(renderOutput)
		{}

		virtual PxAgain processHit(const PxGeomRaycastHit& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2,
			PxReal&, const PxU32*)
		{
			const PxVec3 verts[] = { v0, v1, v2 };

			Gu::ConvexShape tri;
			tri.coreType = Gu::ConvexCore::Type::ePOINTS;
			tri.pose = PxTransform(PxIdentity);
			Gu::ConvexCore::PointsCore& core = *reinterpret_cast<Gu::ConvexCore::PointsCore*>(tri.coreData);
			core.points = verts;
			core.numPoints = 3;
			core.stride = sizeof(PxVec3);
			core.S = mScale.scale;
			core.R = mScale.rotation;
			tri.margin = mTriMargin;

			const PxVec3 triNormal = (v1 - v0).cross(v2 - v0).getNormalized();

			TriangleMeshTriangles triSource(mData, mScale);
			PxVec3 normal, points[Gu::MAX_CONVEX_CONTACTS];
			PxReal dists[Gu::MAX_CONVEX_CONTACTS];
			if (PxU32 count = Gu::generateContacts(mConvex, tri, mContactDist, triNormal, normal, points, dists))
			{
				const PxVec3 worldNormal = mTransform.rotate(normal);
				for (PxU32 i = 0; i < count; ++i)
				{
					PxVec3 pointB = points[i] - normal * dists[i];
					if (validateContact(normal, pointB, hit.faceIndex, triSource))
					{
						const PxVec3 worldPoint = mTransform.transform(points[i]);
						mContact.addPoint(worldPoint, worldNormal, dists[i]);
					}
				}
			}

			return true;
		}
	};

	const PxConvexCoreGeometry& shapeConvex = checkedCast<PxConvexCoreGeometry>(shape0);
	const PxTriangleMeshGeometry& shapeMesh = checkedCast<PxTriangleMeshGeometry>(shape1);
	const TriangleMesh* meshData = _getMeshData(shapeMesh);

	const PxTransform transform0in1 = transform1.transformInv(transform0);
	const PxBounds3 bounds = Gu::computeBounds(shapeConvex, PxTransform(PxIdentity));

	Box queryBox;
	queryBox.extents = bounds.getExtents() + PxVec3(params.mContactDistance);
	queryBox.center = transform0in1.transform(bounds.getCenter());
	queryBox.rot = PxMat33(transform0in1.q);

	PxReal triMargin = queryBox.extents.minElement() * 0.0001f;

	const FastVertex2ShapeScaling meshScaling(shapeMesh.scale);
	meshScaling.transformQueryBounds(queryBox.center, queryBox.extents, queryBox.rot);

	Gu::Contact contact;
	Gu::ConvexShape convex; Gu::makeConvexShape(shapeConvex, transform0in1, convex);
	Callback callback(convex, shapeMesh.scale, meshData, params.mContactDistance, triMargin, transform1, contact, renderOutput);

	Midphase::intersectOBB(meshData, queryBox, callback, false);

	for (PxU32 i = 0; i < contact.numPatches(); ++i)
		for (PxU32 j = 0; j < contact.numPatchPoints(i); ++j)
			contactBuffer.contact(contact.patchPoint(i, j).p, contact.patchNormal(i), contact.patchPoint(i, j).d);

	return contactBuffer.count > 0;
}

bool Gu::contactConvexCoreHeightfield(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(cache);
	PX_UNUSED(renderOutput);

	struct Callback : Gu::OverlapReport
	{
		const Gu::ConvexShape& mConvex;
		const HeightFieldUtil& mHfUtil;
		const PxReal mContactDist;
		const PxTransform& mTransform;
		Gu::Contact& mContact;

		Callback(const Gu::ConvexShape& convex, const HeightFieldUtil& hfUtil, const PxReal contactDist,
			const PxTransform& transform, Gu::Contact& contact)
			:
			mConvex(convex), mHfUtil(hfUtil), mContactDist(contactDist), mTransform(transform), mContact(contact)
		{}

		virtual	bool reportTouchedTris(PxU32 numTris, const PxU32* triInds)
		{
			HeightFieldTriangles triSource(mHfUtil);

			for (PxU32 t = 0; t < numTris; ++t)
			{
				PxU32 triIndex = triInds[t];

				PxU32 vertInds[3];
				triSource.getVertexIndices(triIndex, vertInds[0], vertInds[1], vertInds[2]);
				PxVec3 verts[] = { triSource.getVertex(vertInds[0]),
								   triSource.getVertex(vertInds[1]),
								   triSource.getVertex(vertInds[2]) };

				Gu::ConvexShape tri;
				tri.coreType = Gu::ConvexCore::Type::ePOINTS;
				tri.pose = PxTransform(PxIdentity);
				Gu::ConvexCore::PointsCore& core = *reinterpret_cast<Gu::ConvexCore::PointsCore*>(tri.coreData);
				core.points = verts;
				core.numPoints = 3;
				core.stride = sizeof(PxVec3);
				core.S = PxVec3(1);
				core.R = PxQuat(PxIdentity);
				tri.margin = 0.0f;

				PxVec3 normal, points[Gu::MAX_CONVEX_CONTACTS];
				PxReal dists[Gu::MAX_CONVEX_CONTACTS];
				if (PxU32 count = Gu::generateContacts(mConvex, tri, mContactDist, normal, points, dists))
				{
					const PxVec3 worldNormal = mTransform.rotate(normal);
					for (PxU32 i = 0; i < count; ++i)
					{
						// VR: disabled for now - find out why it skips the tris it shouldn't
						//PxVec3 pointB = points[i] - normal * (dists[i] * 0.5f);
						//if (validateContact(normal, pointB, triIndex, triSource))
						{
							const PxVec3 worldPoint = mTransform.transform(points[i]);
							mContact.addPoint(worldPoint, worldNormal, dists[i]);
						}
					}
				}
			}

			return true;
		}
	};

	const PxConvexCoreGeometry& shapeConvex = checkedCast<PxConvexCoreGeometry>(shape0);
	const PxHeightFieldGeometry& shapeHeightfield = checkedCast<PxHeightFieldGeometry>(shape1);

	const HeightFieldUtil hfUtil(shapeHeightfield);

	const PxTransform transform0in1 = transform1.transformInv(transform0);
	PxBounds3 bounds = Gu::computeBounds(shapeConvex, PxTransform(PxIdentity));
	bounds.fattenFast(params.mContactDistance);

	Gu::Contact contact;
	Gu::ConvexShape convex; Gu::makeConvexShape(shapeConvex, transform0in1, convex);
	Callback callback(convex, hfUtil, params.mContactDistance, transform1, contact);

	hfUtil.overlapAABBTriangles0to1(transform0in1, bounds, callback);

	for (PxU32 i = 0; i < contact.numPatches(); ++i)
		for (PxU32 j = 0; j < contact.numPatchPoints(i); ++j)
			contactBuffer.contact(contact.patchPoint(i, j).p, contact.patchNormal(i), contact.patchPoint(i, j).d);

	return contactBuffer.count > 0;
}