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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef __CU_UPDATE_CACHE_AND_BOUND_CUH__
#define __CU_UPDATE_CACHE_AND_BOUND_CUH__

#include "foundation/PxTransform.h"
#include "PxsTransformCache.h"
#include "PxgConvexConvexShape.h"
#include "geometry/PxGeometry.h"

namespace physx
{

static __device__ PxTransform getAbsPose(const PxTransform& body2World, const PxTransform& shape2Actor, const PxTransform& body2Actor)
{
	PxTransform t0 = body2Actor.transformInv(shape2Actor);
	return body2World.transform(t0);
}

static __device__ void setTransformCache(PxsCachedTransform* cacheArray, const PxTransform& transform, const PxU32 flags, const PxU32 index)
{
	cacheArray[index].transform = transform;
	cacheArray[index].flags = flags;
}

 static __device__ PxVec3 basisExtent(const PxMat33& basis, const PxVec3& extent)
{
	// extended basis vectors
	const PxVec3 c0 = basis.column0 * extent.x;
	const PxVec3 c1 = basis.column1 * extent.y;
	const PxVec3 c2 = basis.column2 * extent.z;

	PxVec3 w;
	// find combination of base vectors that produces max. distance for each component = sum of abs()
	w.x = PxAbs(c0.x) + PxAbs(c1.x) + PxAbs(c2.x);
	w.y = PxAbs(c0.y) + PxAbs(c1.y) + PxAbs(c2.y);
	w.z = PxAbs(c0.z) + PxAbs(c1.z) + PxAbs(c2.z);

	return w;
}


static __device__ void updateBounds(const PxgShapeSim& shapeSim, const PxgShape* convexShapes, PxBounds3* boundsArray, const PxTransform& pose, const PxU32 index)
{

	const PxBounds3& localBound = shapeSim.mLocalBounds;
	PxBounds3& updatedBound = boundsArray[index];
	switch (shapeSim.mShapeType)
	{
	case PxGeometryType::eSPHERE:
	{
		updatedBound.minimum = pose.p + localBound.minimum;
		updatedBound.maximum = pose.p + localBound.maximum;
	}
	break;

	case PxGeometryType::eCAPSULE:
	{
		const PxF32 radius = localBound.maximum.y;
		const PxF32 halfHeight = localBound.maximum.x - radius;
		const PxVec3 d = pose.q.getBasisVector0();
		PxVec3 extents;
		for (PxU32 ax = 0; ax < 3; ax++)
			extents[ax] = PxAbs(d[ax]) * halfHeight + radius;
		updatedBound.minimum = pose.p - extents;
		updatedBound.maximum = pose.p + extents;
	}
	break;

	case PxGeometryType::eBOX:
	{
		const PxVec3 halfExtents = localBound.maximum;
		const PxVec3 extents = basisExtent(PxMat33(pose.q), halfExtents);
		updatedBound.minimum = pose.p - extents;
		updatedBound.maximum = pose.p + extents;
	}
	break;

	case PxGeometryType::eCONVEXMESH:
	{
		const PxU32 hullIndex = shapeSim.mHullDataIndex;

		if (hullIndex != 0xFFffFFff)
		{
			const PxgShape& shape = convexShapes[hullIndex];
			PxMat33 rot(pose.q);
			if (!shape.scale.isIdentity())
				rot = rot * shape.scale.toMat33();

			const PxU8* convexPtr = (PxU8*)shape.hullOrMeshPtr;
			const uint4 tmp = *(((uint4*)convexPtr) + 1);
			const float4* pVertices = reinterpret_cast<const float4*>(convexPtr + sizeof(uint4) + sizeof(float4) + sizeof(float4));

			//const PxU32 polyData0_NbEdgesNbHullVerticesNbPolygons = tmp.x;

			const PxU32 nbHullVertices = u16High(u32Low(tmp.x));//getNbHullVertices(polyData0_NbEdgesNbHullVerticesNbPolygons);

			//PxU32 nb = shape.hullData->mNbHullVertices;
			//const PxVec3* v = shape.hullData->getHullVertices();
			PxVec3 minV = PxVec3(PX_MAX_F32);
			PxVec3 maxV = PxVec3(-PX_MAX_F32);

			for (PxU32 i = 0; i < nbHullVertices; ++i)
			{
				const float4 vf = pVertices[i];
				const PxVec3 v = PxVec3(vf.x, vf.y, vf.z);
				const PxVec3 vertexV = rot.transform(v);
				minV = minV.minimum(vertexV);
				maxV = maxV.maximum(vertexV);
			}

			//const Vec4V posV = Vec4V_From_Vec3V(V3LoadU(&pose.p.x));
			maxV += pose.p;
			minV += pose.p;

			updatedBound.minimum = minV;
			updatedBound.maximum = maxV;
		}
		else
		{
			//ML: this is for GPU incompatible type, which is hull vertices >64 and each hull polygon has vertices > 31
			updatedBound = PxBounds3::transformFast(pose, localBound);
		}

	}
	break;
	default:
	{
		//This updates any dynamic meshes or HFs that may be attached to simulation shapes
		updatedBound = PxBounds3::transformFast(pose, localBound);
	}
	break;

	}
}

__device__ static inline void updateCacheAndBound(const PxTransform& absPos, const PxgShapeSim& shapeSim, PxU32 index,
	PxsCachedTransform* cacheArray, PxBounds3* boundsArray, const PxgShape* shapes, bool isBP)
{
	//TODO: port the transform flags
	setTransformCache(cacheArray, absPos, 0, index);
	if (isBP)
		updateBounds(shapeSim, shapes, boundsArray, absPos, index);
	
}

}

#endif
