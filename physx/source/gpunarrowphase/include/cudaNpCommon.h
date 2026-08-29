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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef CUDA_NP_COMMON_H
#define CUDA_NP_COMMON_H

#include "foundation/PxVec3.h"
#include <stdio.h>
#include "convexFormat.h"
#include "cutil_math.h"
#include "PxgCommonDefines.h"
#include "assert.h"

#define CONVEX_PLANE_WARPS_PER_BLOCK 4
#define GJK_EPA_WARPS_PER_BLOCK 1
#define NUM_TMP_CONTACTS_PER_PAIR 32


//The smallest epsilon we will permit (scaled by PxTolerancesScale.length)
#define	PCM_WITNESS_POINT_LOWER_EPS		1e-2f
//The largest epsilon we will permit (scaled by PxTolerancesScale.length)
#define	PCM_WITNESS_POINT_UPPER_EPS		5e-2f

namespace physx
{
struct PxgPersistentContactManifold;
struct PxgPersistentContactMultiManifold;
struct PxgContactManagerInput;
struct PxsContactManagerOutput;
class PxsContactManager;
struct PxsTorsionalFrictionData;

namespace Sc
{
	class ShapeInteraction;
}


struct PxgPairManagementData
{
	PxgContactManagerInput*			mContactManagerInputData;
	PxsContactManagerOutput*		mContactManagerOutputData;
	PxsContactManager**				mCpuContactManagerMapping;
	Sc::ShapeInteraction**			mShapeInteractions;
	PxReal*							mRestDistances;
	PxsTorsionalFrictionData*		mTorsionalData;
	PxU32*							mTempAccumulator;
	PxU32*							mBlockSharedAccumulator;
	PxU32*							mRemoveIndices;
	PxU32							mNbPairs;
	PxU32							mNbToRemove;

	void*							mPersistentContactManagers;// either PxgPersistentContactManifold or PxgPersistentMultiManagementData

	PxgPairManagementData() : mContactManagerInputData(NULL), mContactManagerOutputData(NULL), mCpuContactManagerMapping(NULL), mShapeInteractions(NULL), 
		mRestDistances(NULL), mTorsionalData(NULL), mTempAccumulator(NULL), mBlockSharedAccumulator(NULL), mNbPairs(0)
	{
	}
};


struct PX_ALIGN_PREFIX(16) PxgPatchAndContactCounters
{
	enum OverflowError
	{
		NO_OVERFLOW = 0,
		CONTACT_BUFFER_OVERFLOW = 1 << 0,
		FORCE_BUFFER_OVERFLOW = 1 << 1,
		PATCH_BUFFER_OVERFLOW = 1 << 2
	};

	PxU32 patchesBytes;
	PxU32 contactsBytes;
	PxU32 forceAndIndiceBytes;
	PxU32 overflowError;

	__device__
	void setOverflowError(const OverflowError& err)
	{
		PX_UNUSED(err);
#if __CUDA_ARCH__
		atomicOr(&overflowError, PxU32(err));
#endif
	}

	__host__ __device__
	PxU32 getOverflowError() const
	{
		return overflowError;
	}
} PX_ALIGN_SUFFIX(16);

__host__ __device__ inline
ConvexHullCooked::Valency u32ToValency(PxU32 countOffset)
{
	ConvexHullCooked::Valency v;
	v.mCount = u32High(countOffset);
	v.mOffset = u32Low(countOffset);
	
	return v;
}

__host__ __device__ inline
PxU32 valencyToPxU32(const ConvexHullCooked::Valency& v)
{
	return ::merge(v.mCount, v.mOffset); 
}

__host__ __device__ inline
PxU8 getNbAdjVerts(PxU32 val)
{
	return u16Low(u32Low(val));
}

__host__ __device__ inline
PxU16 getNbEdges(PxU32 val)
{
	return u32High(val);
}

__host__ __device__ inline
PxU8 getNbPolygons(PxU32 val)
{
	return u16Low(u32Low(val));
}

__host__ __device__ inline
PxU16 getVRef8(PxU32 val)
{
	return u32High(val);
}

__host__ __device__ inline
PxU8 getNbVerts(PxU32 val)
{
	return u16High(u32Low(val));
}

__host__ __device__ inline
PxU8 getMinIndex(PxU32 val)
{
	return u16Low(u32Low(val));
}

__host__ __device__ inline
PxU32 merge(PxU16 hi, PxU8 lohi, PxU8 lolo)
{
	return ::merge(hi, ::merge(lohi, lolo));
}

__host__ __device__ inline bool isValidTet(const PxVec3& worldV0, const PxVec3& worldV1, const PxVec3& worldV2, const PxVec3& worldV3)
{
	return (worldV1 - worldV0).dot((worldV2 - worldV0).cross(worldV3 - worldV0)) > 1e-9f;
}

} // namespace physx

//device only functions
#if PX_CUDA_COMPILER

#include "reduction.cuh"

namespace physx
{

class MeshScaling
{
public:
	__device__ MeshScaling(const PxVec3& scale_, const PxQuat& rotation_)
	{
		scale = scale_;
		rotation = rotation_;
		idtMeshScale = (scale.x == 1.0f && scale.y == 1.0f && scale.z == 1.0f);
		flipNormal = ((scale.x * scale.y * scale.z) < 0.0f);
	}

	__device__ inline PxVec3 vertex2Shape(const PxVec3& v)	const
	{
		PxVec3 temp = rotation.rotate(v);

		temp.x *= scale.x;
		temp.y *= scale.y;
		temp.z *= scale.z;

		return rotation.rotateInv(temp);
	}

	__device__ inline PxVec3 shape2Vertex(const PxVec3& v)	const
	{
		PxVec3 temp = rotation.rotate(v);

		temp.x /= scale.x;
		temp.y /= scale.y;
		temp.z /= scale.z;

		return rotation.rotateInv(temp);
	}

	//Transforms a normal vector from vertex to shape space. This keeps the normal vector perpendicular to surfaces that get scaled with the same transform.
	//Does not preserve length. Is applicable to other covariant vectors too.
	__device__ inline PxVec3 vertex2ShapeNormalVector(const PxVec3& normal)	const
	{
		return shape2Vertex(normal);
	}

	//Transforms a normal vector from shape to vertex space. This keeps the normal vector perpendicular to surfaces that get scaled with the same transform.
	//Does not preserve length. Is applicable to other covariant vectors too.
	__device__ inline PxVec3 shape2VertexNormalVector(const PxVec3& normal)	const
	{
		return vertex2Shape(normal);
	}

	__device__ inline void getShapeSpaceVert(PxVec3& triV0, PxVec3& triV1, PxVec3& triV2,
		PxVec3& v0, PxVec3& v1, PxVec3& v2)	const
	{
		if (idtMeshScale)
		{
			v0 = triV0;
			v1 = triV1;
			v2 = triV2;
		}
		else
		{
			// Triangle scaling, triangle verts in shape space
			triV0 = vertex2Shape(triV0);
			triV1 = vertex2Shape(triV1);
			triV2 = vertex2Shape(triV2);
			v0 = triV0;
			v1 = flipNormal ? triV2 : triV1;
			v2 = flipNormal ? triV1 : triV2;
		}
	}

	PxVec3 scale;
	PxQuat rotation;
	bool idtMeshScale;
	bool flipNormal;
};

//Applies a potentially non-uniform scaling to the point v. The scaling can be expressed in a rotated coordinate frame defined by the quaternion called rotation.
__device__ inline
PxVec3 vertex2Shape(const PxVec3& v, const PxVec3& scale, const PxQuat& rotation)
{
	PxVec3 temp = rotation.rotate(v);

	temp.x *= scale.x; 
	temp.y *= scale.y;
	temp.z *= scale.z; 
	
	return rotation.rotateInv(temp);
}

//Removes a potentially non-uniform scaling from the point v. The scaling can be expressed in a rotated coordinate frame defined by the quaternion called rotation.
__device__ inline
PxVec3 shape2Vertex(const PxVec3& v, const PxVec3& scale, const PxQuat& rotation)
{
	PxVec3 temp = rotation.rotate(v);

	temp.x /= scale.x; 
	temp.y /= scale.y;
	temp.z /= scale.z; 
	
	return rotation.rotateInv(temp);
}

//Transforms a normal vector from vertex to shape space. This keeps the normal vector perpendicular to surfaces that get scaled with the same transform.
//Does not preserve length. Is applicable to other covariant vectors too.
__device__ inline
PxVec3 vertex2ShapeNormalVector(const PxVec3& normal, const PxVec3& scale, const PxQuat& rotation)
{
	return shape2Vertex(normal, scale, rotation);
}

//Transforms a normal vector from shape to vertex space. This keeps the normal vector perpendicular to surfaces that get scaled with the same transform.
//Does not preserve length. Is applicable to other covariant vectors too.
__device__ inline
PxVec3 shape2VertexNormalVector(const PxVec3& normal, const PxVec3& scale, const PxQuat& rotation)
{
	return vertex2Shape(normal, scale, rotation);
}

__device__ inline static
void prepareVertices(const PxTransform& transf,
	PxVec3 scale, PxQuat rot,
	PxU32 nbHullVertices,
	const float4* pVertices,
	PxVec3* s_vertices
)
{
	assert(transf.isFinite());
	for (PxU32 i = threadIdx.x; i < nbHullVertices; i += WARP_SIZE)
	{
		float4 f = pVertices[i];
		s_vertices[i] = transf.transform(vertex2Shape(PxVec3(f.x, f.y, f.z), scale, rot));
		assert(s_vertices[i].isFinite());
	}
}

__device__ inline static
void prepareVertices(PxVec3 scale, PxQuat rot,
	PxU32 nbHullVertices,
	const float4* pVertices,
	PxVec3* s_vertices
)
{
	for (PxU32 i = threadIdx.x; i < nbHullVertices; i += WARP_SIZE)
	{
		float4 f = pVertices[i];
		s_vertices[i] = vertex2Shape(PxVec3(f.x, f.y, f.z), scale, rot);
	}
}

#define MESH_MANIFOLD_EPSILON   0.05f
#define BOX_MARGIN_RATIO		0.25f

__device__ inline static
PxReal calculatePCMConvexMargin(const float4& extents_, const PxVec3& scale, const PxReal toleranceLength)
{
	const PxVec3 extents = PxVec3(extents_.x * scale.x, 
								 extents_.y * scale.y,
								 extents_.z * scale.z);

	const PxReal min = fminf(fminf(extents.x, extents.y), extents.z);
	const PxReal toleranceMargin = toleranceLength * MESH_MANIFOLD_EPSILON;
	//ML: 25% of the minimum extents of the internal AABB as this convex hull's margin
	return fminf((min * BOX_MARGIN_RATIO), toleranceMargin);
}

__device__ inline static
PxReal calculatePCMConvexMargin(const PxVec3& extents, const PxReal toleranceLength)
{
	const PxReal min = fminf(fminf(extents.x, extents.y), extents.z);
	const PxReal toleranceMargin = toleranceLength * MESH_MANIFOLD_EPSILON;
	//ML: 25% of the minimum extents of the internal AABB as this convex hull's margin
	return fminf((min * BOX_MARGIN_RATIO), toleranceMargin);
}

__device__ inline static PxReal maxTransformPositionDelta(const PxVec3& curP, const PxVec3& preP)
{
	const PxVec3 deltaP = curP - preP;
	return PxMax(PxAbs(deltaP.x), PxMax(PxAbs(deltaP.y), PxAbs(deltaP.z)));
}


__constant__ __device__ PxF32 local_invalidateThresholdsConvex[5] = {	0.5f, 0.125f, 0.25f, 0.375f, 0.375f	};
__constant__ __device__ PxF32 local_invalidateQuatThresholdsConvex[5] = {	0.9998f, 0.9999f, 0.9999f, 0.9999f, 0.9999f	};

__constant__ __device__ PxF32 local_invalidateThresholdsSphere[3] = { 0.5f, 0.1f, 0.75f };
__constant__ __device__ PxF32 local_invalidateQuatThresholdsSphere[3] = { 0.9995f, 0.9999f, 0.9997f };

__constant__ __device__ PxF32 local_invalidateQuatThresholdsConvexPlane[5] = { 0.99996f, 0.99996f, 0.99996f, 0.99996f, 0.99996f };
	
__device__ inline static
PxU32 invalidate_BoxConvex(const PxVec3& curRelPos, const PxQuat& curQuatA, const PxQuat& curQuatB, 
	const PxVec3& preRelPos, const PxQuat& preRelQuatA, const PxQuat& preRelQuatB,
	const PxReal minMargin, const PxReal radiusA, const PxReal radiusB, 
	const PxU8 manifold_NumContacts, PxF32* local_invalidateThresholds, PxF32* local_invalidateQuatThresholds)
{
	//This is the translational threshold used in invalidate_BoxConvexHull. 0.5 is 50% of the object margin. we use different threshold between
	//0 and 4 points. This threashold is a scale that is multiplied by the objects' margins.
	const PxReal ratio = local_invalidateThresholds[manifold_NumContacts];
	
	const PxReal thresholdP = minMargin * ratio;
	const PxReal deltaP = maxTransformPositionDelta(curRelPos, preRelPos);

	//This is the rotational threshold used in invalidate_BoxConvexHull. 0.9998 is a threshold for quat difference
	//between previous and current frame
	const PxReal thresholdQ = local_invalidateQuatThresholds[manifold_NumContacts];
	const PxReal deltaQA = curQuatA.dot(preRelQuatA);
	const PxReal deltaQB = curQuatB.dot(preRelQuatB);

	bool generateContacts = (deltaP > thresholdP) || (thresholdQ > deltaQA) || (thresholdQ > deltaQB);
	if (!generateContacts)
	{
		const PxReal aRadian = deltaQA < 1.0f ? acosf(deltaQA) : 0.f;
		const PxReal bRadian = deltaQB < 1.0f ? acosf(deltaQB) : 0.f;

		const PxReal travelDistA = aRadian * radiusA;
		const PxReal travelDistB = bRadian * radiusB;

		generateContacts = (travelDistA > thresholdP) || (travelDistB > thresholdP);
	}
	return generateContacts;
}

} // namespace physx

#endif
#endif
