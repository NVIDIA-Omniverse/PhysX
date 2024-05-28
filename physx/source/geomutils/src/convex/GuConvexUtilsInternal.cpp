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

#include "foundation/PxBounds3.h"
#include "geometry/PxConvexMeshGeometry.h"
#include "GuConvexUtilsInternal.h"
#include "GuBoxConversion.h"
#include "GuConvexMesh.h"
#include "CmScaling.h"
#include "CmMatrix34.h"

using namespace physx;
using namespace Gu;
using namespace Cm;

void Gu::computeHullOBB(Box& hullOBB, const PxBounds3& hullAABB, float offset, 
						const PxMat34& convexPose, 
						const PxMat34& meshPose, const FastVertex2ShapeScaling& meshScaling, bool idtScaleMesh)
{
	// transform bounds = mesh space
	const PxMat34 m0to1 = meshPose.transformTranspose(convexPose);

	hullOBB.extents = hullAABB.getExtents() + PxVec3(offset);
	hullOBB.center = m0to1.transform(hullAABB.getCenter());
	hullOBB.rot = m0to1.m;
	
	if(!idtScaleMesh)
		meshScaling.transformQueryBounds(hullOBB.center, hullOBB.extents, hullOBB.rot);
}

void Gu::computeVertexSpaceOBB(Box& dst, const Box& src, const PxTransform& meshPose, const PxMeshScale& meshScale)
{
	// AP scaffold failure in x64 debug in GuConvexUtilsInternal.cpp
	//PX_ASSERT("Performance warning - this path shouldn't execute for identity mesh scale." && !meshScale.isIdentity());

	dst = transform(meshScale.getInverse() * Matrix34FromTransform(meshPose.getInverse()), src);
}

void Gu::computeOBBAroundConvex(Box& obb, const PxConvexMeshGeometry& convexGeom, const PxConvexMesh* cm, const PxTransform& convexPose)
{
	const CenterExtents& aabb = static_cast<const Gu::ConvexMesh*>(cm)->getLocalBoundsFast();

	if(convexGeom.scale.isIdentity())
	{
		const PxMat33Padded m(convexPose.q);
		obb = Gu::Box(m.transform(aabb.mCenter) + convexPose.p, aabb.mExtents, m);
	}
	else
	{
		obb = transform(Matrix34FromTransform(convexPose) * toMat33(convexGeom.scale), Box(aabb.mCenter, aabb.mExtents, PxMat33(PxIdentity)));
	}
}
