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

#include "collision/PxCollisionDefs.h"
#include "PxImmediateMode.h"
#include "geometry/PxGjkQuery.h"
#include "extensions/PxGjkQueryExt.h"
#include "geometry/PxCustomGeometry.h"
#include "CustomConvex.h"
#include "geomutils/PxContactBuffer.h"

using namespace physx;

CustomConvex::CustomConvex(float _margin)
	:
	margin(_margin)
{}

PxBounds3 CustomConvex::getLocalBounds(const PxGeometry&) const
{
	const PxVec3 min(supportLocal(PxVec3(-1, 0, 0)).x, supportLocal(PxVec3(0, -1, 0)).y, supportLocal(PxVec3(0, 0, -1)).z);
	const PxVec3 max(supportLocal(PxVec3(1, 0, 0)).x, supportLocal(PxVec3(0, 1, 0)).y, supportLocal(PxVec3(0, 0, 1)).z);
	PxBounds3 bounds(min, max);
	bounds.fattenSafe(getMargin());
	return bounds;
}
bool CustomConvex::generateContacts(const PxGeometry& geom0, const PxGeometry& geom1,
	const PxTransform& pose0, const PxTransform& pose1, const PxReal contactDistance, const PxReal meshContactMargin,
	const PxReal toleranceLength, PxContactBuffer& contactBuffer) const
{
	switch (int(geom1.getType()))
	{
	case PxGeometryType::eSPHERE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eCONVEXMESH:
	{
		PxGjkQueryExt::ConvexGeomSupport geomSupport(geom1);
		if (PxGjkQueryExt::generateContacts(*this, geomSupport, pose0, pose1, contactDistance, toleranceLength, contactBuffer))
			return true;
		break;
	}
	case PxGeometryType::ePLANE:
	{
		const PxPlane plane = PxPlane(1.0f, 0.0f, 0.0f, 0.0f).transform(pose1);
		const PxPlane localPlane = plane.inverseTransform(pose0);
		const PxVec3 point = supportLocal(-localPlane.n);
		const float dist = localPlane.distance(point);
		const float radius = getMargin();
		if (dist < radius)
		{
			const PxVec3 n = localPlane.n;
			const PxVec3 p = point + n * (radius - dist) * 0.5f;
			PxContactPoint contact;
			contact.point = pose0.transform(p);
			contact.normal = pose0.rotate(n);
			contact.separation = dist - radius;
			contactBuffer.contact(contact);
			return true;
		}
		break;
	}
	case PxGeometryType::eCUSTOM:
	{
		const PxCustomGeometry& customGeom1 = static_cast<const PxCustomGeometry&>(geom1);
		if (customGeom1.getCustomType() == CustomCylinder::TYPE() ||
			customGeom1.getCustomType() == CustomCone::TYPE()) // It's a CustomConvex
		{
			CustomConvex* custom1 = static_cast<CustomConvex*>(customGeom1.callbacks);
			if (PxGjkQueryExt::generateContacts(*this, *custom1, pose0, pose1, contactDistance, toleranceLength, contactBuffer))
				return true;
		}
		else
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
				ContactCacheAllocator() { memset(buffer, 0, sizeof(buffer)); }
				virtual PxU8* allocateCacheData(const PxU32 /*byteSize*/) { return reinterpret_cast<PxU8*>(size_t(buffer + 0xf) & ~0xf); }
			}
			contactCacheAllocator;

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

	return false;
}
PxU32 CustomConvex::raycast(const PxVec3& origin, const PxVec3& unitDir, const PxGeometry& /*geom*/, const PxTransform& pose,
	PxReal maxDist, PxHitFlags /*hitFlags*/, PxU32 /*maxHits*/, PxGeomRaycastHit* rayHits, PxU32 /*stride*/, PxRaycastThreadContext*) const
{
	PxReal t;
	PxVec3 n, p;
	if (PxGjkQuery::raycast(*this, pose, origin, unitDir, maxDist, t, n, p))
	{
		PxGeomRaycastHit& hit = *rayHits;
		hit.distance = t;
		hit.position = p;
		hit.normal = n;
		return 1;
	}

	return 0;
}
bool CustomConvex::overlap(const PxGeometry& /*geom0*/, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1, PxOverlapThreadContext*) const
{
	switch (int(geom1.getType()))
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
bool CustomConvex::sweep(const PxVec3& unitDir, const PxReal maxDist,
	const PxGeometry& /*geom0*/, const PxTransform& pose0, const PxGeometry& geom1, const PxTransform& pose1,
	PxGeomSweepHit& sweepHit, PxHitFlags /*hitFlags*/, const PxReal inflation, PxSweepThreadContext*) const
{
	switch (int(geom1.getType()))
	{
	case PxGeometryType::eSPHERE:
	case PxGeometryType::eCAPSULE:
	case PxGeometryType::eBOX:
	case PxGeometryType::eCONVEXMESH:
	{
		PxGjkQueryExt::ConvexGeomSupport geomSupport(geom1, inflation);
		PxReal t;
		PxVec3 n, p;
		if (PxGjkQuery::sweep(*this, geomSupport, pose0, pose1, unitDir, maxDist, t, n, p))
		{
			PxGeomSweepHit& hit = sweepHit;
			hit.distance = t;
			hit.position = p;
			hit.normal = n;
			return true;
		}
		break;
	}
	default:
		break;
	}

	return false;
}

PxReal CustomConvex::getMargin() const
{
	return margin;
}

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(CustomCylinder)

//PxU32 CustomCylinder::getCustomType() const
//{
//	return 1;
//}
void CustomCylinder::computeMassProperties(const physx::PxGeometry& /*geometry*/, physx::PxMassProperties& massProperties) const
{
	float H = height + 2.0f * margin, R = radius + margin;
	massProperties.mass = PxPi * R * R * H;
	massProperties.inertiaTensor = PxMat33(PxZero);
	massProperties.centerOfMass = PxVec3(PxZero);
	massProperties.inertiaTensor[0][0] = massProperties.mass * R * R / 2.0f;
	massProperties.inertiaTensor[1][1] = massProperties.inertiaTensor[2][2] = massProperties.mass * (3 * R * R + H * H) / 12.0f;
}
PxVec3 CustomCylinder::supportLocal(const PxVec3& dir) const
{
	float halfHeight = height * 0.5f;
	PxVec3 d = dir.getNormalized();

	if (PxSign2(d.x) != 0 && PxSign2(d.y) == 0 && PxSign2(d.z) == 0)
		return PxVec3(PxSign2(d.x) * halfHeight, 0, 0);

	return PxVec3(PxSign2(d.x) * halfHeight, 0, 0) + PxVec3(0, d.y, d.z).getNormalized() * radius;
}

IMPLEMENT_CUSTOM_GEOMETRY_TYPE(CustomCone)
void CustomCone::computeMassProperties(const physx::PxGeometry&, physx::PxMassProperties& massProperties) const
{
	float H = height + 2.0f * margin, R = radius + margin;
	massProperties.mass = PxPi * R * R * H / 3.0f;
	massProperties.inertiaTensor = PxMat33(PxZero);
	massProperties.centerOfMass = PxVec3(PxZero);
	massProperties.inertiaTensor[0][0] = massProperties.mass * R * R * 3.0f / 10.0f;
	massProperties.inertiaTensor[1][1] = massProperties.inertiaTensor[2][2] = massProperties.mass * (R * R * 3.0f / 20.0f + H * H * 3.0f / 80.0f);
	massProperties.centerOfMass[0] = -height / 4.0f;
}
PxVec3 CustomCone::supportLocal(const PxVec3& dir) const
{
	float halfHeight = height * 0.5f;
	float cosAlph = radius / sqrtf(height * height + radius * radius);

	PxVec3 d = dir.getNormalized();

	if (d.x > cosAlph || (PxSign2(d.x) != 0 && PxSign2(d.y) == 0 && PxSign2(d.z) == 0))
		return PxVec3(PxSign2(d.x) * halfHeight, 0, 0);

	return PxVec3(-halfHeight, 0, 0) + PxVec3(0, d.y, d.z).getNormalized() * radius;
}
