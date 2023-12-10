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

#include "extensions/PxCustomGeometryExt.h"

#include <geometry/PxGeometryHelpers.h>
#include <geometry/PxGeometryQuery.h>
#include <geometry/PxMeshQuery.h>
#include <geometry/PxTriangle.h>
#include <geometry/PxTriangleMesh.h>
#include <geometry/PxTriangleMeshGeometry.h>
#include <geomutils/PxContactBuffer.h>
#include <common/PxRenderOutput.h>
#include <extensions/PxGjkQueryExt.h>
#include <extensions/PxMassProperties.h>
#include <PxImmediateMode.h>

#include "omnipvd/ExtOmniPvdSetData.h"

using namespace physx;
#if PX_SUPPORT_OMNI_PVD
using namespace Ext;
#endif

static const PxU32 gCollisionShapeColor = PxU32(PxDebugColor::eARGB_MAGENTA);

///////////////////////////////////////////////////////////////////////////////

static const PxU32 MAX_TRIANGLES = 512;
static const PxU32 MAX_TRIANGLE_CONTACTS = 6;
const PxReal FACE_CONTACT_THRESHOLD = 0.99999f;

struct TrimeshContactFilter
{
	PxU32 triCount;
	PxU32 triIndices[MAX_TRIANGLES];
	PxU32 triAdjacencies[MAX_TRIANGLES][3];
	PxU32 triContactCounts[MAX_TRIANGLES][2];
	PxContactPoint triContacts[MAX_TRIANGLES][MAX_TRIANGLE_CONTACTS];

	TrimeshContactFilter() : triCount(0) {}

	void addTriangleContacts(const PxContactPoint* points, PxU32 count, const PxU32 triIndex, const PxVec3 triVerts[3], const PxU32 triAdjacency[3])
	{
		if (triCount == MAX_TRIANGLES)
			return;

		triIndices[triCount] = triIndex;
		PxU32& faceContactCount = triContactCounts[triCount][0];
		PxU32& edgeContactCount = triContactCounts[triCount][1];
		faceContactCount = edgeContactCount = 0;

		for (PxU32 i = 0; i < 3; ++i)
			triAdjacencies[triCount][i] = triAdjacency[i];

		PxVec3 triNormal = (triVerts[1] - triVerts[0]).cross(triVerts[2] - triVerts[0]).getNormalized();

		for (PxU32 i = 0; i < count; ++i)
		{
			const PxContactPoint& point = points[i];
			bool faceContact = fabsf(point.normal.dot(triNormal)) > FACE_CONTACT_THRESHOLD;

			if (faceContactCount + edgeContactCount < MAX_TRIANGLE_CONTACTS)
			{
				if (faceContact) triContacts[triCount][faceContactCount++] = point;
				else triContacts[triCount][MAX_TRIANGLE_CONTACTS - 1 - edgeContactCount++] = point;
			}
		}

		++triCount;
	}

	void writeToBuffer(PxContactBuffer& buffer)
	{
		for (PxU32 i = 0; i < triCount; ++i)
		{
			if (triContactCounts[i][1] > 0)
			{
				for (PxU32 j = 0; j < triCount; ++j)
				{
					if (triIndices[j] == triAdjacencies[i][0] || triIndices[j] == triAdjacencies[i][1] || triIndices[j] == triAdjacencies[i][2])
					{
						if (triContactCounts[j][0] > 0)
						{
							triContactCounts[i][1] = 0;
							break;
						}
					}
				}
			}

			for (PxU32 j = 0; j < triContactCounts[i][0]; ++j)
				buffer.contact(triContacts[i][j]);

			for (PxU32 j = 0; j < triContactCounts[i][1]; ++j)
				buffer.contact(triContacts[i][MAX_TRIANGLE_CONTACTS - 1 - j]);
		}
	}

	PX_NOCOPY(TrimeshContactFilter)
};

///////////////////////////////////////////////////////////////////////////////

struct TriangleSupport : PxGjkQuery::Support
{
	PxVec3 v0, v1, v2;
	PxReal margin;

	TriangleSupport(const PxVec3& _v0, const PxVec3& _v1, const PxVec3& _v2, PxReal _margin)
		:
		v0(_v0), v1(_v1), v2(_v2), margin(_margin)
	{}

	virtual PxReal getMargin() const
	{
		return margin;
	}
	virtual PxVec3 supportLocal(const PxVec3& dir) const
	{
		float d0 = dir.dot(v0), d1 = dir.dot(v1), d2 = dir.dot(v2);
		return (d0 > d1 && d0 > d2) ? v0 : (d1 > d2) ? v1 : v2;
	}
};

///////////////////////////////////////////////////////////////////////////////

static void drawArc(const PxVec3& center, const PxVec3& radius, const PxVec3& axis, PxReal angle, PxReal error, PxRenderOutput& out)
{
	int sides = int(ceilf(angle / (2 * acosf(1.0f - error))));
	float step = angle / sides;
	out << PxRenderOutput::LINESTRIP;
	for (int i = 0; i <= sides; ++i)
		out << center + PxQuat(step * i, axis).rotate(radius);
}
static void drawCircle(const PxVec3& center, const PxVec3& radius, const PxVec3& axis, PxReal error, PxRenderOutput& out)
{
	drawArc(center, radius, axis, PxTwoPi, error, out);
}
static void drawQuarterCircle(const PxVec3& center, const PxVec3& radius, const PxVec3& axis, PxReal error, PxRenderOutput& out)
{
	drawArc(center, radius, axis, PxPiDivTwo, error, out);
}
static void drawLine(const PxVec3& s, const PxVec3& e, PxRenderOutput& out)
{
	out << PxRenderOutput::LINES << s << e;
}

///////////////////////////////////////////////////////////////////////////////

PxBounds3 PxCustomGeometryExt::BaseConvexCallbacks::getLocalBounds(const PxGeometry&) const
{
	const PxVec3 min(supportLocal(PxVec3(-1, 0, 0)).x, supportLocal(PxVec3(0, -1, 0)).y, supportLocal(PxVec3(0, 0, -1)).z);
	const PxVec3 max(supportLocal(PxVec3(1, 0, 0)).x, supportLocal(PxVec3(0, 1, 0)).y, supportLocal(PxVec3(0, 0, 1)).z);
	PxBounds3 bounds(min, max);
	bounds.fattenSafe(getMargin());
	return bounds;
}

bool PxCustomGeometryExt::BaseConvexCallbacks::generateContacts(const PxGeometry& geom0, const PxGeometry& geom1,
	const PxTransform& pose0, const PxTransform& pose1, const PxReal contactDistance, const PxReal meshContactMargin,
	const PxReal toleranceLength, PxContactBuffer& contactBuffer) const
{
	struct ContactRecorder : immediate::PxContactRecorder
	{
		PxContactBuffer* contactBuffer;
		ContactRecorder(PxContactBuffer& _contactBuffer) : contactBuffer(&_contactBuffer) {}
		virtual bool recordContacts(const PxContactPoint* contactPoints, PxU32 nbContacts, PxU32 /*index*/)
		{
			for (PxU32 i = 0; i < nbContacts; ++i)
				contactBuffer->contact(contactPoints[i]);
			return true;
		}
	}
	contactRecorder(contactBuffer);

	PxCache contactCache;

	struct ContactCacheAllocator : PxCacheAllocator
	{
		PxU8 buffer[1024];
		ContactCacheAllocator() { PxMemSet(buffer, 0, sizeof(buffer)); }
		virtual PxU8* allocateCacheData(const PxU32 /*byteSize*/) { return reinterpret_cast<PxU8*>(size_t(buffer + 0xf) & ~0xf); }
	}
	contactCacheAllocator;

	const PxTransform identityPose(PxIdentity);

	switch (geom1.getType())
	{
	case PxGeometryType::eSPHERE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eCONVEXMESH:
	{
		PxGjkQueryExt::ConvexGeomSupport geomSupport(geom1);
		if (PxGjkQueryExt::generateContacts(*this, geomSupport, pose0, pose1, contactDistance, toleranceLength, contactBuffer))
		{
			PxGeometryHolder substituteGeom; PxTransform preTransform;
			if (useSubstituteGeometry(substituteGeom, preTransform, contactBuffer.contacts[contactBuffer.count - 1], pose0))
			{
				contactBuffer.count--;
				const PxGeometry* pGeom0 = &substituteGeom.any();
				const PxGeometry* pGeom1 = &geom1;
				// PT:: tag: scalar transform*transform
				PxTransform pose = pose0.transform(preTransform);
				immediate::PxGenerateContacts(&pGeom0, &pGeom1, &pose, &pose1, &contactCache, 1, contactRecorder,
					contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);
			}
		}
		break;
	}
	case PxGeometryType::ePLANE:
	{
		const PxPlane plane = PxPlane(1.0f, 0.0f, 0.0f, 0.0f).transform(pose1);
		const PxPlane localPlane = plane.inverseTransform(pose0);
		const PxVec3 point = supportLocal(-localPlane.n) - localPlane.n * margin;
		const float dist = localPlane.distance(point);
		if (dist < contactDistance)
		{
			const PxVec3 n = localPlane.n;
			const PxVec3 p = point - n * dist * 0.5f;
			PxContactPoint contact;
			contact.point = pose0.transform(p);
			contact.normal = pose0.rotate(n);
			contact.separation = dist;
			contactBuffer.contact(contact);
			PxGeometryHolder substituteGeom; PxTransform preTransform;
			if (useSubstituteGeometry(substituteGeom, preTransform, contactBuffer.contacts[contactBuffer.count - 1], pose0))
			{
				contactBuffer.count--;
				const PxGeometry* pGeom0 = &substituteGeom.any();
				const PxGeometry* pGeom1 = &geom1;
				// PT:: tag: scalar transform*transform
				PxTransform pose = pose0.transform(preTransform);
				immediate::PxGenerateContacts(&pGeom0, &pGeom1, &pose, &pose1, &contactCache, 1, contactRecorder,
					contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);
			}
		}
		break;
	}
	case PxGeometryType::eTRIANGLEMESH:
	case PxGeometryType::eHEIGHTFIELD:
	{
		// As a triangle has no thickness, we need to choose some collision margin - the distance
		// considered a touching contact. I set it as 1% of the custom shape minimum dimension.
		PxReal meshMargin = getLocalBounds(geom0).getDimensions().minElement() * 0.01f;
		TrimeshContactFilter contactFilter;
		bool hasAdjacency = (geom1.getType() != PxGeometryType::eTRIANGLEMESH || (static_cast<const PxTriangleMeshGeometry&>(geom1).triangleMesh->getTriangleMeshFlags() & PxTriangleMeshFlag::eADJACENCY_INFO));
		PxBoxGeometry boxGeom(getLocalBounds(geom0).getExtents() + PxVec3(contactDistance + meshMargin));
		PxU32 triangles[MAX_TRIANGLES];
		bool overflow = false;
		PxU32 triangleCount = (geom1.getType() == PxGeometryType::eTRIANGLEMESH) ?
			PxMeshQuery::findOverlapTriangleMesh(boxGeom, pose0, static_cast<const PxTriangleMeshGeometry&>(geom1), pose1, triangles, MAX_TRIANGLES, 0, overflow) :
			PxMeshQuery::findOverlapHeightField(boxGeom, pose0, static_cast<const PxHeightFieldGeometry&>(geom1), pose1, triangles, MAX_TRIANGLES, 0, overflow);
		if (overflow)
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, PX_FL, "PxCustomGeometryExt::BaseConvexCallbacks::generateContacts() Too many triangles.\n");
		for (PxU32 i = 0; i < triangleCount; ++i)
		{
			PxTriangle tri; PxU32 adjacent[3];
			if (geom1.getType() == PxGeometryType::eTRIANGLEMESH)
				PxMeshQuery::getTriangle(static_cast<const PxTriangleMeshGeometry&>(geom1), pose1, triangles[i], tri, NULL, hasAdjacency ? adjacent : NULL);
			else
				PxMeshQuery::getTriangle(static_cast<const PxHeightFieldGeometry&>(geom1), pose1, triangles[i], tri, NULL, adjacent);
			TriangleSupport triSupport(tri.verts[0], tri.verts[1], tri.verts[2], meshMargin);
			if (PxGjkQueryExt::generateContacts(*this, triSupport, pose0, identityPose, contactDistance, toleranceLength, contactBuffer))
			{
				contactBuffer.contacts[contactBuffer.count - 1].internalFaceIndex1 = triangles[i];
				PxGeometryHolder substituteGeom; PxTransform preTransform;
				if (useSubstituteGeometry(substituteGeom, preTransform, contactBuffer.contacts[contactBuffer.count - 1], pose0))
				{
					contactBuffer.count--;
					const PxGeometry& geom = substituteGeom.any();
					// PT:: tag: scalar transform*transform
					PxTransform pose = pose0.transform(preTransform);
					PxGeometryQuery::generateTriangleContacts(geom, pose, tri.verts, triangles[i], contactDistance, meshMargin, toleranceLength, contactBuffer);
				}
			}
			if (hasAdjacency)
			{
				contactFilter.addTriangleContacts(contactBuffer.contacts, contactBuffer.count, triangles[i], tri.verts, adjacent);
				contactBuffer.count = 0;
			}
		}
		if (hasAdjacency)
			contactFilter.writeToBuffer(contactBuffer);
		break;
	}
	case PxGeometryType::eCUSTOM:
	{
		const PxCustomGeometry& customGeom1 = static_cast<const PxCustomGeometry&>(geom1);
		if (customGeom1.getCustomType() == CylinderCallbacks::TYPE() ||
			customGeom1.getCustomType() == ConeCallbacks::TYPE()) // It's a CustomConvex
		{
			BaseConvexCallbacks* custom1 = static_cast<BaseConvexCallbacks*>(customGeom1.callbacks);
			if (PxGjkQueryExt::generateContacts(*this, *custom1, pose0, pose1, contactDistance, toleranceLength, contactBuffer))
			{
				PxGeometryHolder substituteGeom; PxTransform preTransform;
				if (useSubstituteGeometry(substituteGeom, preTransform, contactBuffer.contacts[contactBuffer.count - 1], pose0))
				{
					contactBuffer.count--;

					PxU32 oldCount = contactBuffer.count;

					const PxGeometry* pGeom0 = &substituteGeom.any();
					const PxGeometry* pGeom1 = &geom1;
					// PT:: tag: scalar transform*transform
					PxTransform pose = pose0.transform(preTransform);
					immediate::PxGenerateContacts(&pGeom1, &pGeom0, &pose1, &pose, &contactCache, 1, contactRecorder,
						contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);

					for (int i = oldCount; i < int(contactBuffer.count); ++i)
						contactBuffer.contacts[i].normal = -contactBuffer.contacts[i].normal;
				}
			}
		}
		else
		{
			const PxGeometry* pGeom0 = &geom0;
			const PxGeometry* pGeom1 = &geom1;
			immediate::PxGenerateContacts(&pGeom1, &pGeom0, &pose1, &pose0, &contactCache, 1, contactRecorder,
				contactDistance, meshContactMargin, toleranceLength, contactCacheAllocator);

			for (int i = 0; i < int(contactBuffer.count); ++i)
				contactBuffer.contacts[i].normal = -contactBuffer.contacts[i].normal;
		}
		break;
	}
	default:
		break;
	}

	return contactBuffer.count > 0;
}

PxU32 PxCustomGeometryExt::BaseConvexCallbacks::raycast(const PxVec3& origin, const PxVec3& unitDir, const PxGeometry& geom, const PxTransform& pose,
	PxReal maxDist, PxHitFlags /*hitFlags*/, PxU32 /*maxHits*/, PxGeomRaycastHit* rayHits, PxU32 /*stride*/, PxRaycastThreadContext*) const
{
	// When FLT_MAX is used as maxDist, it works bad with GJK algorithm.
	// Here I compute the maximum needed distance (wiseDist) as the diagonal
	// of the bounding box of both the geometry and the ray origin.
	PxBounds3 bounds = PxBounds3::transformFast(pose, getLocalBounds(geom));
	bounds.include(origin);
	PxReal wiseDist = PxMin(maxDist, bounds.getDimensions().magnitude());
	PxReal t;
	PxVec3 n, p;
	if (PxGjkQuery::raycast(*this, pose, origin, unitDir, wiseDist, t, n, p))
	{
		PxGeomRaycastHit& hit = *rayHits;
		hit.distance = t;
		hit.position = p;
		hit.normal = n;
		return 1;
	}

	return 0;
}

bool PxCustomGeometryExt::BaseConvexCallbacks::overlap(const PxGeometry& /*geom0*/, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1, PxOverlapThreadContext*) const
{
	switch (geom1.getType())
	{
	case PxGeometryType::eSPHERE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eCONVEXMESH:
	{
		PxGjkQueryExt::ConvexGeomSupport geomSupport(geom1);
		if (PxGjkQuery::overlap(*this, geomSupport, pose0, pose1))
			return true;
		break;
	}
	default:
		break;
	}

	return false;
}

bool PxCustomGeometryExt::BaseConvexCallbacks::sweep(const PxVec3& unitDir, const PxReal maxDist,
	const PxGeometry& geom0, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1,
	PxGeomSweepHit& sweepHit, PxHitFlags /*hitFlags*/, const PxReal inflation, PxSweepThreadContext*) const
{
	PxBounds3 bounds0 = PxBounds3::transformFast(pose0, getLocalBounds(geom0));

	switch (geom1.getType())
	{
	case PxGeometryType::eSPHERE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eCONVEXMESH:
	{
		// See comment in BaseConvexCallbacks::raycast
		PxBounds3 bounds; PxGeometryQuery::computeGeomBounds(bounds, geom1, pose1, 0, inflation);
		bounds.include(bounds0);
		PxReal wiseDist = PxMin(maxDist, bounds.getDimensions().magnitude());
		PxGjkQueryExt::ConvexGeomSupport geomSupport(geom1, inflation);
		PxReal t;
		PxVec3 n, p;
		if (PxGjkQuery::sweep(*this, geomSupport, pose0, pose1, unitDir, wiseDist, t, n, p))
		{
			sweepHit.distance = t;
			sweepHit.position = p;
			sweepHit.normal = n;
			sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL;
			return true;
		}
		break;
	}
	// sweep against triangle meshes and height fields for CCD support
	case PxGeometryType::eTRIANGLEMESH:
	case PxGeometryType::eHEIGHTFIELD:
	{
		PxReal radius = getLocalBounds(geom0).getExtents().magnitude();
		PxGeometryHolder sweepGeom = PxSphereGeometry(radius + inflation);
		PxTransform sweepGeomPose = pose0;
		if (maxDist > FLT_EPSILON)
		{
			sweepGeom = PxCapsuleGeometry(radius + inflation, maxDist * 0.5f);
			sweepGeomPose = PxTransform(pose0.p - unitDir * maxDist * 0.5f, PxShortestRotation(PxVec3(1, 0, 0), unitDir));
		}
		PxU32 triangles[MAX_TRIANGLES];
		bool overflow = false;
		PxU32 triangleCount = (geom1.getType() == PxGeometryType::eTRIANGLEMESH) ?
			PxMeshQuery::findOverlapTriangleMesh(sweepGeom.any(), sweepGeomPose, static_cast<const PxTriangleMeshGeometry&>(geom1), pose1, triangles, MAX_TRIANGLES, 0, overflow) :
			PxMeshQuery::findOverlapHeightField(sweepGeom.any(), sweepGeomPose, static_cast<const PxHeightFieldGeometry&>(geom1), pose1, triangles, MAX_TRIANGLES, 0, overflow);
		if(overflow)
			PxGetFoundation().error(PxErrorCode::eDEBUG_INFO, PX_FL, "PxCustomGeometryExt::BaseConvexCallbacks::sweep() Too many triangles.\n");
		sweepHit.distance = PX_MAX_F32;
		const PxTransform identityPose(PxIdentity);
		for (PxU32 i = 0; i < triangleCount; ++i)
		{
			PxTriangle tri;
			if (geom1.getType() == PxGeometryType::eTRIANGLEMESH)
				PxMeshQuery::getTriangle(static_cast<const PxTriangleMeshGeometry&>(geom1), pose1, triangles[i], tri);
			else
				PxMeshQuery::getTriangle(static_cast<const PxHeightFieldGeometry&>(geom1), pose1, triangles[i], tri);
			TriangleSupport triSupport(tri.verts[0], tri.verts[1], tri.verts[2], inflation);
			PxBounds3 bounds = bounds0;
			for (PxU32 j = 0; j < 3; ++j)
				bounds.include(tri.verts[j]);
			bounds.fattenFast(inflation);
			// See comment in BaseConvexCallbacks::raycast
			PxReal wiseDist = PxMin(maxDist, bounds.getDimensions().magnitude());
			PxReal t;
			PxVec3 n, p;
			if (PxGjkQuery::sweep(*this, triSupport, pose0, identityPose, unitDir, wiseDist, t, n, p))
			{
				if (sweepHit.distance > t)
				{
					sweepHit.faceIndex = triangles[i];
					sweepHit.distance = t;
					sweepHit.position = p;
					sweepHit.normal = n;
					sweepHit.flags = PxHitFlag::ePOSITION | PxHitFlag::eNORMAL | PxHitFlag::eFACE_INDEX;
				}
			}
		}
		if (sweepHit.distance < PX_MAX_F32)
			return true;
		break;
	}
	default:
		break;
	}

	return false;
}

bool PxCustomGeometryExt::BaseConvexCallbacks::usePersistentContactManifold(const PxGeometry& /*geometry*/, PxReal& breakingThreshold) const
{
	// Even if we don't use persistent manifold, we still need to set proper breakingThreshold
	// because the other geometry still can force the PCM usage. FLT_EPSILON ensures that
	// the contacts will be discarded and recomputed every frame if actor moves.
	breakingThreshold = FLT_EPSILON;
	return false;
}

void PxCustomGeometryExt::BaseConvexCallbacks::setMargin(float m)
{
	margin = m;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtBaseConvexCallbacks, margin, *this, margin);
#endif
}

///////////////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(PxCustomGeometryExt::CylinderCallbacks)

PxCustomGeometryExt::CylinderCallbacks::CylinderCallbacks(float _height, float _radius, int _axis, float _margin)
	:
	BaseConvexCallbacks(_margin), height(_height), radius(_radius), axis(_axis)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, *this);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtBaseConvexCallbacks, margin, *static_cast<BaseConvexCallbacks*>(this), margin);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, height, *this, height);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, radius, *this, radius);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, axis, *this, axis);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void PxCustomGeometryExt::CylinderCallbacks::setHeight(float h)
{
	height = h;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, height, *this, height);
#endif
}

void PxCustomGeometryExt::CylinderCallbacks::setRadius(float r)
{
	radius = r;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, radius, *this, radius);
#endif
}

void PxCustomGeometryExt::CylinderCallbacks::setAxis(int a)
{
	axis = a;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtCylinderCallbacks, axis, *this, axis);
#endif
}

void PxCustomGeometryExt::CylinderCallbacks::visualize(const PxGeometry&, PxRenderOutput& out, const PxTransform& transform, const PxBounds3&) const
{
	const float ERR = 0.001f;

	out << gCollisionShapeColor;
	out << transform;

	int axis1 = (axis + 1) % 3;
	int axis2 = (axis + 2) % 3;

	PxVec3 zr(PxZero), rd(PxZero), ax(PxZero), ax1(PxZero), ax2(PxZero), r0(PxZero), r1(PxZero);
	ax[axis] = ax1[axis1] = ax2[axis2] = 1.0f;
	r0[axis1] = r1[axis2] = radius;

	rd[axis1] = radius;
	rd[axis] = -(height * 0.5f + margin);
	drawCircle(zr, rd, ax, ERR, out);
	rd[axis] = (height * 0.5f + margin);
	drawCircle(zr, rd, ax, ERR, out);
	rd[axis1] = radius + margin;
	rd[axis] = -(height * 0.5f);
	drawCircle(zr, rd, ax, ERR, out);
	rd[axis] = (height * 0.5f);
	drawCircle(zr, rd, ax, ERR, out);

	drawLine(-ax * height * 0.5f + ax1 * (radius + margin), ax * height * 0.5f + ax1 * (radius + margin), out);
	drawLine(-ax * height * 0.5f - ax1 * (radius + margin), ax * height * 0.5f - ax1 * (radius + margin), out);
	drawLine(-ax * height * 0.5f + ax2 * (radius + margin), ax * height * 0.5f + ax2 * (radius + margin), out);
	drawLine(-ax * height * 0.5f - ax2 * (radius + margin), ax * height * 0.5f - ax2 * (radius + margin), out);

	drawQuarterCircle(-ax * height * 0.5f + ax1 * radius, -ax * margin, -ax2, ERR, out);
	drawQuarterCircle(-ax * height * 0.5f - ax1 * radius, -ax * margin, ax2, ERR, out);
	drawQuarterCircle(-ax * height * 0.5f + ax2 * radius, -ax * margin, ax1, ERR, out);
	drawQuarterCircle(-ax * height * 0.5f - ax2 * radius, -ax * margin, -ax1, ERR, out);

	drawQuarterCircle(ax * height * 0.5f + ax1 * radius, ax * margin, ax2, ERR, out);
	drawQuarterCircle(ax * height * 0.5f - ax1 * radius, ax * margin, -ax2, ERR, out);
	drawQuarterCircle(ax * height * 0.5f + ax2 * radius, ax * margin, -ax1, ERR, out);
	drawQuarterCircle(ax * height * 0.5f - ax2 * radius, ax * margin, ax1, ERR, out);
}

PxVec3 PxCustomGeometryExt::CylinderCallbacks::supportLocal(const PxVec3& dir) const
{
	float halfHeight = height * 0.5f;
	PxVec3 d = dir.getNormalized();

	switch (axis)
	{
	case 0: // X
	{
		if (PxSign2(d.x) != 0 && PxSign2(d.y) == 0 && PxSign2(d.z) == 0) return PxVec3(PxSign2(d.x) * halfHeight, 0, 0);
		return PxVec3(PxSign2(d.x) * halfHeight, 0, 0) + PxVec3(0, d.y, d.z).getNormalized() * radius;
	}
	case 1: // Y
	{
		if (PxSign2(d.x) == 0 && PxSign2(d.y) != 0 && PxSign2(d.z) == 0) return PxVec3(0, PxSign2(d.y) * halfHeight, 0);
		return PxVec3(0, PxSign2(d.y) * halfHeight, 0) + PxVec3(d.x, 0, d.z).getNormalized() * radius;
	}
	case 2: // Z
	{
		if (PxSign2(d.x) == 0 && PxSign2(d.y) == 0 && PxSign2(d.z) != 0) return PxVec3(0, 0, PxSign2(d.z) * halfHeight);
		return PxVec3(0, 0, PxSign2(d.z) * halfHeight) + PxVec3(d.x, d.y, 0).getNormalized() * radius;
	}
	}

	return PxVec3(0);
}

void PxCustomGeometryExt::CylinderCallbacks::computeMassProperties(const PxGeometry& /*geometry*/, PxMassProperties& massProperties) const
{
	if (margin == 0)
	{
		PxMassProperties& mass = massProperties;
		float R = radius, H = height;

		mass.mass = PxPi * R * R * H;
		mass.inertiaTensor = PxMat33(PxZero);
		mass.centerOfMass = PxVec3(PxZero);

		float I0 = mass.mass * R * R / 2.0f;
		float I1 = mass.mass * (3 * R * R + H * H) / 12.0f;

		mass.inertiaTensor[axis][axis] = I0;
		mass.inertiaTensor[(axis + 1) % 3][(axis + 1) % 3] = mass.inertiaTensor[(axis + 2) % 3][(axis + 2) % 3] = I1;
	}
	else
	{
		const int SLICE_COUNT = 32;
		PxMassProperties sliceMasses[SLICE_COUNT];
		PxTransform slicePoses[SLICE_COUNT];
		float sliceHeight = height / SLICE_COUNT;

		for (int i = 0; i < SLICE_COUNT; ++i)
		{
			float t = -height * 0.5f + i * sliceHeight + sliceHeight * 0.5f;
			float R = getRadiusAtHeight(t), H = sliceHeight;

			PxMassProperties mass;
			mass.mass = PxPi * R * R * H;
			mass.inertiaTensor = PxMat33(PxZero);
			mass.centerOfMass = PxVec3(PxZero);

			float I0 = mass.mass * R * R / 2.0f;
			float I1 = mass.mass * (3 * R * R + H * H) / 12.0f;

			mass.inertiaTensor[axis][axis] = I0;
			mass.inertiaTensor[(axis + 1) % 3][(axis + 1) % 3] = mass.inertiaTensor[(axis + 2) % 3][(axis + 2) % 3] = I1;
			mass.centerOfMass[axis] = t;

			sliceMasses[i] = mass;
			slicePoses[i] = PxTransform(PxIdentity);
		}

		massProperties = PxMassProperties::sum(sliceMasses, slicePoses, SLICE_COUNT);
	}
}

bool PxCustomGeometryExt::CylinderCallbacks::useSubstituteGeometry(PxGeometryHolder& geom, PxTransform& preTransform, const PxContactPoint& p, const PxTransform& pose0) const
{
	// here I check if we contact with the cylender bases or the lateral surface
	// where more than 1 contact point can be generated.
	PxVec3 locN = pose0.rotateInv(p.normal);
	float nAng = acosf(PxClamp(-locN[axis], -1.0f, 1.0f));
	float epsAng = PxPi / 36.0f; // 5 degrees
	if (nAng < epsAng || nAng > PxPi - epsAng)
	{
		// if we contact with the bases
		// make the substitute geometry a box and rotate it so one of
		// the corners matches the contact point
		PxVec3 halfSize;
		halfSize[axis] = height * 0.5f + margin;
		halfSize[(axis + 1) % 3] = halfSize[(axis + 2) % 3] = radius / sqrtf(2.0f);
		geom = PxBoxGeometry(halfSize);
		PxVec3 axisDir(PxZero); axisDir[axis] = 1.0f;
		PxVec3 locP = pose0.transformInv(p.point);
		float s1 = locP[(axis + 1) % 3], s2 = locP[(axis + 2) % 3];
		float ang = ((s1 * s1) + (s2 * s2) > 1e-3f) ? atan2f(s2, s1) : 0;
		preTransform = PxTransform(PxQuat(ang + PxPi * 0.25f, axisDir));
		return true;
	}
	else if (nAng > PxPiDivTwo - epsAng && nAng < PxPiDivTwo + epsAng)
	{
		// if it's the lateral surface
		// make the substitute geometry a capsule and rotate it so it aligns
		// with the cylinder surface
		geom = PxCapsuleGeometry(radius + margin, height * 0.5f);
		switch (axis)
		{
		case 0: preTransform = PxTransform(PxIdentity); break;
		case 1: preTransform = PxTransform(PxQuat(PxPi * 0.5f, PxVec3(0, 0, 1))); break;
		case 2: preTransform = PxTransform(PxQuat(PxPi * 0.5f, PxVec3(0, 1, 0))); break;
		}
		return true;
	}
	return false;
}

float PxCustomGeometryExt::CylinderCallbacks::getRadiusAtHeight(float h) const
{
	if (h >= -height * 0.5f && h <= height * 0.5f)
		return radius + margin;

	if (h < -height * 0.5f)
	{
		float a = -h - height * 0.5f;
		return radius + sqrtf(margin * margin - a * a);
	}

	if (h > height * 0.5f)
	{
		float a = h - height * 0.5f;
		return radius + sqrtf(margin * margin - a * a);
	}

	PX_ASSERT(0);
	return 0;
}

///////////////////////////////////////////////////////////////////////////////

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(PxCustomGeometryExt::ConeCallbacks)

PxCustomGeometryExt::ConeCallbacks::ConeCallbacks(float _height, float _radius, int _axis, float _margin)
	:
	BaseConvexCallbacks(_margin), height(_height), radius(_radius), axis(_axis)
{
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, *this);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtBaseConvexCallbacks, margin, *static_cast<BaseConvexCallbacks*>(this), margin);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, height, *this, height);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, radius, *this, radius);
	OMNI_PVD_SET_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, axis, *this, axis);
	OMNI_PVD_WRITE_SCOPE_END
#endif
}

void PxCustomGeometryExt::ConeCallbacks::setHeight(float h)
{
	height = h;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, height, *this, height);
#endif
}

void PxCustomGeometryExt::ConeCallbacks::setRadius(float r)
{
	radius = r;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, radius, *this, radius);
#endif
}

void PxCustomGeometryExt::ConeCallbacks::setAxis(int a)
{
	axis = a;
#if PX_SUPPORT_OMNI_PVD
	OMNI_PVD_SET(OMNI_PVD_CONTEXT_HANDLE, PxCustomGeometryExtConeCallbacks, axis, *this, axis);
#endif
}

void PxCustomGeometryExt::ConeCallbacks::visualize(const PxGeometry&, PxRenderOutput& out, const PxTransform& transform, const PxBounds3&) const
{
	const float ERR = 0.001f;

	out << gCollisionShapeColor;
	out << transform;

	int axis1 = (axis + 1) % 3;
	int axis2 = (axis + 2) % 3;

	PxVec3 zr(PxZero), rd(PxZero), ax(PxZero), ax1(PxZero), ax2(PxZero), r0(PxZero), r1(PxZero);
	ax[axis] = ax1[axis1] = ax2[axis2] = 1.0f;
	r0[axis1] = r1[axis2] = radius;

	float ang = atan2f(radius, height);
	float aSin = sinf(ang);

	rd[axis1] = radius;
	rd[axis] = -(height * 0.5f + margin);
	drawCircle(zr, rd, ax, ERR, out);
	rd[axis] = -(height * 0.5f) + margin * aSin;
	rd[axis1] = getRadiusAtHeight(rd[axis]);
	drawCircle(zr, rd, ax, ERR, out);
	rd[axis] = height * 0.5f + margin * aSin;
	rd[axis1] = getRadiusAtHeight(rd[axis]);
	drawCircle(zr, rd, ax, ERR, out);

	float h0 = -height * 0.5f + margin * aSin, h1 = height * 0.5f + margin * aSin;
	float s0 = getRadiusAtHeight(h0), s1 = getRadiusAtHeight(h1);
	drawLine(ax * h0 + ax1 * s0, ax * h1 + ax1 * s1, out);
	drawLine(ax * h0 - ax1 * s0, ax * h1 - ax1 * s1, out);
	drawLine(ax * h0 + ax2 * s0, ax * h1 + ax2 * s1, out);
	drawLine(ax * h0 - ax2 * s0, ax * h1 - ax2 * s1, out);

	drawArc(-ax * height * 0.5f + ax1 * radius, -ax * margin, -ax2, PxPiDivTwo + ang, ERR, out);
	drawArc(-ax * height * 0.5f - ax1 * radius, -ax * margin, ax2, PxPiDivTwo + ang, ERR, out);
	drawArc(-ax * height * 0.5f + ax2 * radius, -ax * margin, ax1, PxPiDivTwo + ang, ERR, out);
	drawArc(-ax * height * 0.5f - ax2 * radius, -ax * margin, -ax1, PxPiDivTwo + ang, ERR, out);

	drawArc(ax * height * 0.5f, ax * margin, ax2, PxPiDivTwo - ang, ERR, out);
	drawArc(ax * height * 0.5f, ax * margin, -ax2, PxPiDivTwo - ang, ERR, out);
	drawArc(ax * height * 0.5f, ax * margin, -ax1, PxPiDivTwo - ang, ERR, out);
	drawArc(ax * height * 0.5f, ax * margin, ax1, PxPiDivTwo - ang, ERR, out);
}

PxVec3 PxCustomGeometryExt::ConeCallbacks::supportLocal(const PxVec3& dir) const
{
	float halfHeight = height * 0.5f;
	float cosAlph = radius / sqrtf(height * height + radius * radius);
	PxVec3 d = dir.getNormalized();

	switch (axis)
	{
	case 0: // X
	{
		if (d.x > cosAlph || (PxSign2(d.x) != 0 && PxSign2(d.y) == 0 && PxSign2(d.z) == 0)) return PxVec3(PxSign2(d.x) * halfHeight, 0, 0);
		return PxVec3(-halfHeight, 0, 0) + PxVec3(0, d.y, d.z).getNormalized() * radius;
	}
	case 1: // Y
	{
		if (d.y > cosAlph || (PxSign2(d.y) != 0 && PxSign2(d.x) == 0 && PxSign2(d.z) == 0)) return PxVec3(0, PxSign2(d.y) * halfHeight, 0);
		return PxVec3(0, -halfHeight, 0) + PxVec3(d.x, 0, d.z).getNormalized() * radius;
	}
	case 2: // Z
	{
		if (d.z > cosAlph || (PxSign2(d.z) != 0 && PxSign2(d.x) == 0 && PxSign2(d.y) == 0)) return PxVec3(0, 0, PxSign2(d.z) * halfHeight);
		return PxVec3(0, 0, -halfHeight) + PxVec3(d.x, d.y, 0).getNormalized() * radius;
	}
	}

	return PxVec3(0);
}

void PxCustomGeometryExt::ConeCallbacks::computeMassProperties(const PxGeometry& /*geometry*/, PxMassProperties& massProperties) const
{
	if (margin == 0)
	{
		PxMassProperties& mass = massProperties;

		float H = height, R = radius;

		mass.mass = PxPi * R * R * H / 3.0f;
		mass.inertiaTensor = PxMat33(PxZero);
		mass.centerOfMass = PxVec3(PxZero);

		float I0 = mass.mass * R * R * 3.0f / 10.0f;
		float I1 = mass.mass * (R * R * 3.0f / 20.0f + H * H * 3.0f / 80.0f);

		mass.inertiaTensor[axis][axis] = I0;
		mass.inertiaTensor[(axis + 1) % 3][(axis + 1) % 3] = mass.inertiaTensor[(axis + 2) % 3][(axis + 2) % 3] = I1;

		mass.centerOfMass[axis] = -H / 4.0f;

		mass.inertiaTensor = PxMassProperties::translateInertia(mass.inertiaTensor, mass.mass, mass.centerOfMass);
	}
	else
	{
		const int SLICE_COUNT = 32;
		PxMassProperties sliceMasses[SLICE_COUNT];
		PxTransform slicePoses[SLICE_COUNT];
		float sliceHeight = height / SLICE_COUNT;

		for (int i = 0; i < SLICE_COUNT; ++i)
		{
			float t = -height * 0.5f + i * sliceHeight + sliceHeight * 0.5f;
			float R = getRadiusAtHeight(t), H = sliceHeight;

			PxMassProperties mass;
			mass.mass = PxPi * R * R * H;
			mass.inertiaTensor = PxMat33(PxZero);
			mass.centerOfMass = PxVec3(PxZero);

			float I0 = mass.mass * R * R / 2.0f;
			float I1 = mass.mass * (3 * R * R + H * H) / 12.0f;

			mass.inertiaTensor[axis][axis] = I0;
			mass.inertiaTensor[(axis + 1) % 3][(axis + 1) % 3] = mass.inertiaTensor[(axis + 2) % 3][(axis + 2) % 3] = I1;
			mass.centerOfMass[axis] = t;

			sliceMasses[i] = mass;
			slicePoses[i] = PxTransform(PxIdentity);
		}

		massProperties = PxMassProperties::sum(sliceMasses, slicePoses, SLICE_COUNT);

		massProperties.inertiaTensor = PxMassProperties::translateInertia(massProperties.inertiaTensor, massProperties.mass, massProperties.centerOfMass);
	}
}

bool PxCustomGeometryExt::ConeCallbacks::useSubstituteGeometry(PxGeometryHolder& geom, PxTransform& preTransform, const PxContactPoint& p, const PxTransform& pose0) const
{
	// here I check if we contact with the cone base or the lateral surface
	// where more than 1 contact point can be generated.
	PxVec3 locN = pose0.rotateInv(p.normal);
	float nAng = acosf(PxClamp(-locN[axis], -1.0f, 1.0f));
	float epsAng = PxPi / 36.0f; // 5 degrees
	float coneAng = atan2f(radius, height);
	if (nAng > PxPi - epsAng)
	{
		// if we contact with the base
		// make the substitute geometry a box and rotate it so one of
		// the corners matches the contact point
		PxVec3 halfSize;
		halfSize[axis] = height * 0.5f + margin;
		halfSize[(axis + 1) % 3] = halfSize[(axis + 2) % 3] = radius / sqrtf(2.0f);
		geom = PxBoxGeometry(halfSize);
		PxVec3 axisDir(PxZero); axisDir[axis] = 1.0f;
		PxVec3 locP = pose0.transformInv(p.point);
		float s1 = locP[(axis + 1) % 3], s2 = locP[(axis + 2) % 3];
		float ang = ((s1 * s1) + (s2 * s2) > 1e-3f) ? atan2f(s2, s1) : 0;
		preTransform = PxTransform(PxQuat(ang + PxPi * 0.25f, axisDir));
		return true;
	}
	else if (nAng > PxPiDivTwo - coneAng - epsAng && nAng < PxPiDivTwo - coneAng + epsAng)
	{
		// if it's the lateral surface
		// make the substitute geometry a capsule and rotate it so it aligns
		// with the cone surface
		geom = PxCapsuleGeometry(radius * 0.25f + margin, sqrtf(height * height + radius * radius) * 0.5f);
		switch (axis)
		{
		case 0:
		{
			PxVec3 capC = (PxVec3(height * 0.5f, 0, 0) + PxVec3(-height * 0.5f, radius, 0)) * 0.5f - PxVec3(radius, height, 0).getNormalized() * radius * 0.25f;
			preTransform = PxTransform(capC, PxQuat(-coneAng, PxVec3(0, 0, 1)));
			break;
		}
		case 1:
		{
			PxVec3 capC = (PxVec3(0, height * 0.5f, 0) + PxVec3(0, -height * 0.5f, radius)) * 0.5f - PxVec3(0, radius, height).getNormalized() * radius * 0.25f;
			preTransform = PxTransform(capC, PxQuat(-coneAng, PxVec3(1, 0, 0)) * PxQuat(PxPiDivTwo, PxVec3(0, 0, 1)));
			break;
		}
		case 2:
		{
			PxVec3 capC = (PxVec3(0, 0, height * 0.5f) + PxVec3(radius, 0, -height * 0.5f)) * 0.5f - PxVec3(height, 0, radius).getNormalized() * radius * 0.25f;
			preTransform = PxTransform(capC, PxQuat(-coneAng, PxVec3(0, 1, 0)) * PxQuat(PxPiDivTwo, PxVec3(0, 1, 0)));
			break;
		}
		}
		PxVec3 axisDir(PxZero); axisDir[axis] = 1.0f;
		float n1 = -locN[(axis + 1) % 3], n2 = -locN[(axis + 2) % 3];
		float ang = atan2f(n2, n1);
		// PT:: tag: scalar transform*transform
		preTransform = PxTransform(PxQuat(ang, axisDir)) * preTransform;
		return true;
	}
	return false;
}

float PxCustomGeometryExt::ConeCallbacks::getRadiusAtHeight(float h) const
{
	float angle = atan2f(radius, height);
	float aSin = sinf(angle);
	float aCos = cosf(angle);

	if (h >= -height * 0.5f + margin * aSin && h <= height * 0.5f + margin * aSin)
		return radius * (height * 0.5f - h) / height + margin / aCos;

	if (h < -height * 0.5f + margin * aSin)
	{
		float a = -h - height * 0.5f;
		return radius + sqrtf(margin * margin - a * a);
	}

	if (h > height * 0.5f + margin * aSin)
	{
		float a = h - height * 0.5f;
		return sqrtf(margin * margin - a * a);
	}

	PX_ASSERT(0);
	return 0;
}
