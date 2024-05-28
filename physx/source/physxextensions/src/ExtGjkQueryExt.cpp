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

#include "extensions/PxGjkQueryExt.h"
#include "geometry/PxSphereGeometry.h"
#include "geometry/PxBoxGeometry.h"
#include "geometry/PxPlaneGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "foundation/PxAllocator.h"
#include "geomutils/PxContactBuffer.h"

using namespace physx;

///////////////////////////////////////////////////////////////////////////////

PxGjkQueryExt::SphereSupport::SphereSupport() : radius(0.0f)
{
}

PxGjkQueryExt::SphereSupport::SphereSupport(PxReal _radius) : radius(_radius)
{
}

PxGjkQueryExt::SphereSupport::SphereSupport(const PxSphereGeometry& geom) : radius(geom.radius)
{
}

PxReal PxGjkQueryExt::SphereSupport::getMargin() const
{
	return radius;
}

PxVec3 PxGjkQueryExt::SphereSupport::supportLocal(const PxVec3& /*dir*/) const
{
	return PxVec3(0.0f);
}

///////////////////////////////////////////////////////////////////////////////

PxGjkQueryExt::CapsuleSupport::CapsuleSupport() : radius(0.0f), halfHeight(0.0f)
{
}

PxGjkQueryExt::CapsuleSupport::CapsuleSupport(PxReal _radius, PxReal _halfHeight) : radius(_radius), halfHeight(_halfHeight)
{
}

PxGjkQueryExt::CapsuleSupport::CapsuleSupport(const PxCapsuleGeometry& geom) : radius(geom.radius)
{
}

PxReal PxGjkQueryExt::CapsuleSupport::getMargin() const
{
	return radius;
}

PxVec3 PxGjkQueryExt::CapsuleSupport::supportLocal(const PxVec3& dir) const
{
	return PxVec3(PxSign2(dir.x) * halfHeight, 0.0f, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////

PxGjkQueryExt::BoxSupport::BoxSupport() : halfExtents(0.0f), margin(0.0f)
{
}

PxGjkQueryExt::BoxSupport::BoxSupport(const PxVec3& _halfExtents, PxReal _margin) : halfExtents(_halfExtents), margin(_margin)
{
}

PxGjkQueryExt::BoxSupport::BoxSupport(const PxBoxGeometry& box, PxReal _margin) : halfExtents(box.halfExtents), margin(_margin)
{
}

PxReal PxGjkQueryExt::BoxSupport::getMargin() const
{
	return margin;
}

PxVec3 PxGjkQueryExt::BoxSupport::supportLocal(const PxVec3& dir) const
{
	const PxVec3 d = dir.getNormalized();
	return PxVec3(PxSign2(d.x) * halfExtents.x, PxSign2(d.y) * halfExtents.y, PxSign2(d.z) * halfExtents.z);
}

///////////////////////////////////////////////////////////////////////////////

PxGjkQueryExt::ConvexMeshSupport::ConvexMeshSupport() :
	convexMesh		(NULL),
	scale			(0.0f),
	scaleRotation	(0.0f),
	margin			(0.0f)
{
}

PxGjkQueryExt::ConvexMeshSupport::ConvexMeshSupport(const PxConvexMesh& _convexMesh, const PxVec3& _scale, const PxQuat& _scaleRotation, PxReal _margin) :
	convexMesh		(&_convexMesh),
	scale			(_scale),
	scaleRotation	(_scaleRotation),
	margin			(_margin)
{
}

PxGjkQueryExt::ConvexMeshSupport::ConvexMeshSupport(const PxConvexMeshGeometry& _convexMesh, PxReal _margin) :
	convexMesh		(_convexMesh.convexMesh),
	scale			(_convexMesh.scale.scale),
	scaleRotation	(_convexMesh.scale.rotation),
	margin			(_margin)
{
}

PxReal PxGjkQueryExt::ConvexMeshSupport::getMargin() const
{
	return margin * scale.minElement();
}

PxVec3 PxGjkQueryExt::ConvexMeshSupport::supportLocal(const PxVec3& dir) const
{
	if (convexMesh == NULL)
		return PxVec3(0.0f);

	PxVec3 d = scaleRotation.rotateInv(scaleRotation.rotate(dir).multiply(scale));
	const PxVec3* verts = convexMesh->getVertices();
	int count = int(convexMesh->getNbVertices());
	float maxDot = -FLT_MAX;
	int index = -1;
	for (int i = 0; i < count; ++i)
	{
		float dot = verts[i].dot(d);
		if (dot > maxDot)
		{
			maxDot = dot;
			index = i;
		}
	}

	if (index == -1)
		return PxVec3(0);

	return scaleRotation.rotateInv(scaleRotation.rotate(verts[index]).multiply(scale));
}

///////////////////////////////////////////////////////////////////////////////

PxGjkQueryExt::ConvexGeomSupport::ConvexGeomSupport() : mType(PxGeometryType::eINVALID)
{
}

PxGjkQueryExt::ConvexGeomSupport::ConvexGeomSupport(const PxGeometry& geom, PxReal margin)
{
	mType = PxGeometryType::eINVALID;
	switch (geom.getType())
	{
	case PxGeometryType::eSPHERE:
	{
		mType = PxGeometryType::eSPHERE;
		const PxSphereGeometry& sphere = static_cast<const PxSphereGeometry&>(geom);
		PX_PLACEMENT_NEW(&mSupport, SphereSupport(sphere.radius + margin));
		break;
	}
	case PxGeometryType::eCAPSULE:
	{
		mType = PxGeometryType::eCAPSULE;
		const PxCapsuleGeometry& capsule = static_cast<const PxCapsuleGeometry&>(geom);
		PX_PLACEMENT_NEW(&mSupport, CapsuleSupport(capsule.radius + margin, capsule.halfHeight));
		break;
	}
	case PxGeometryType::eBOX:
	{
		mType = PxGeometryType::eBOX;
		PX_PLACEMENT_NEW(&mSupport, BoxSupport(static_cast<const PxBoxGeometry&>(geom), margin));
		break;
	}
	case PxGeometryType::eCONVEXMESH:
	{
		mType = PxGeometryType::eCONVEXMESH;
		PX_PLACEMENT_NEW(&mSupport, ConvexMeshSupport(static_cast<const PxConvexMeshGeometry&>(geom), margin));
		break;
	}
	default:
		break;
	}
}

PxGjkQueryExt::ConvexGeomSupport::~ConvexGeomSupport()
{
	if (isValid())
		reinterpret_cast<Support&>(mSupport).~Support();
}

bool PxGjkQueryExt::ConvexGeomSupport::isValid() const
{
	return mType != PxGeometryType::eINVALID;
}

PxReal PxGjkQueryExt::ConvexGeomSupport::getMargin() const
{
	return isValid() ? reinterpret_cast<const Support&>(mSupport).getMargin() : 0.0f;
}

PxVec3 PxGjkQueryExt::ConvexGeomSupport::supportLocal(const PxVec3& dir) const
{
	return isValid() ? reinterpret_cast<const Support&>(mSupport).supportLocal(dir) : PxVec3(0.0f);
}

///////////////////////////////////////////////////////////////////////////////

bool PxGjkQueryExt::generateContacts(const PxGjkQuery::Support& a, const PxGjkQuery::Support& b, const PxTransform& poseA, const PxTransform& poseB, PxReal contactDistance, PxReal toleranceLength, PxContactBuffer& contactBuffer)
{
	PxVec3 pointA, pointB, separatingAxis; PxReal separation;
	if (!PxGjkQuery::proximityInfo(a, b, poseA, poseB, contactDistance, toleranceLength, pointA, pointB, separatingAxis, separation))
		return false;

	PxContactPoint contact;
	contact.point = (pointA + pointB) * 0.5f; // VR: should I make it just pointB?
	contact.normal = separatingAxis;
	contact.separation = separation;
	contactBuffer.contact(contact);

	return true;
}
