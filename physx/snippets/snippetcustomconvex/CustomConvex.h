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

#ifndef CUSTOM_CONVEX_H
#define CUSTOM_CONVEX_H

#include "PxPhysicsAPI.h"

struct CustomConvex : physx::PxCustomGeometry::Callbacks, physx::PxGjkQuery::Support
{
	float margin;

	CustomConvex(float margin);

	// override PxCustomGeometry::Callbacks

	virtual physx::PxBounds3 getLocalBounds(const physx::PxGeometry&) const;
	virtual bool generateContacts(const physx::PxGeometry& geom0, const physx::PxGeometry& geom1, const physx::PxTransform& pose0, const physx::PxTransform& pose1,
		const physx::PxReal contactDistance, const physx::PxReal meshContactMargin, const physx::PxReal toleranceLength,
		physx::PxContactBuffer& contactBuffer) const;
	virtual physx::PxU32 raycast(const physx::PxVec3& origin, const physx::PxVec3& unitDir, const physx::PxGeometry& geom, const physx::PxTransform& pose,
		physx::PxReal maxDist, physx::PxHitFlags hitFlags, physx::PxU32 maxHits, physx::PxGeomRaycastHit* rayHits, physx::PxU32 stride, physx::PxRaycastThreadContext*) const;
	virtual bool overlap(const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1, physx::PxOverlapThreadContext*) const;
	virtual bool sweep(const physx::PxVec3& unitDir, const physx::PxReal maxDist,
		const physx::PxGeometry& geom0, const physx::PxTransform& pose0, const physx::PxGeometry& geom1, const physx::PxTransform& pose1,
		physx::PxGeomSweepHit& sweepHit, physx::PxHitFlags hitFlags, const physx::PxReal inflation, physx::PxSweepThreadContext*) const;
	virtual void visualize(const physx::PxGeometry&, physx::PxRenderOutput&, const physx::PxTransform&, const physx::PxBounds3&) const {}
	virtual bool usePersistentContactManifold(const physx::PxGeometry&, physx::PxReal&) const { return true; }

	// override PxGjkQuery::Support

	virtual physx::PxReal getMargin() const;
};
struct CustomCylinder : CustomConvex
{
	float height, radius;

	CustomCylinder(float _height, float _radius, float _margin) : CustomConvex(_margin), height(_height), radius(_radius) {}

	// override PxCustomGeometry::Callbacks

	DECLARE_CUSTOM_GEOMETRY_TYPE

	virtual void computeMassProperties(const physx::PxGeometry& geometry, physx::PxMassProperties& massProperties) const;

	// override PxGjkQuery::Support

	virtual physx::PxVec3 supportLocal(const physx::PxVec3& dir) const;
};
struct CustomCone : CustomConvex
{
	float height, radius;

	CustomCone(float _height, float _radius, float _margin) : CustomConvex(_margin), height(_height), radius(_radius) {}

	// override PxCustomGeometry::Callbacks

	DECLARE_CUSTOM_GEOMETRY_TYPE

	virtual void computeMassProperties(const physx::PxGeometry& geometry, physx::PxMassProperties& massProperties) const;

	// override PxGjkQuery::Support

	virtual physx::PxVec3 supportLocal(const physx::PxVec3& dir) const;
};

#endif
