// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef __DEFORMABLE_ELEMENT_FILTER_CUH__
#define __DEFORMABLE_ELEMENT_FILTER_CUH__

#include "foundation/PxSimpleTypes.h"
#include "PxFiltering.h"
#include "PxgSimulationCoreDesc.h"

namespace physx
{

static __device__ bool find(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs, const PxgDeformablePairInfo& pair)
{
	PxU32 l = 0, r = nbPairs;

	while (l < r)
	{
		PxU32 mid = (l + r) / 2;
		PxI32 cmp = PxgComparePairInfo(pair, pairs[mid]);

		if (cmp == 0)
			return true;
		if (cmp < 0)
			r = mid;
		else
			l = mid + 1;
	}
	return false;
}

// One helper per filter array. The helper name gives the order of the two references, which differs
// per array.

// Volume against volume:
// ref0 is a vertex of the probing volume's collision mesh.
// ref1 is a tetrahedron of the other volume.
static __device__ PX_FORCE_INLINE bool findVolumeVolumeFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxU32 volumeId0, const PxU32 volumeVertexId0, const PxU32 volumeId1, const PxU32 volumeTetId1)
{
	PxgDeformablePairInfo pair = { PxgEncodeDeformableVertex(volumeId0, volumeVertexId0),
								   PxgEncodeDeformableTetrahedron(volumeId1, volumeTetId1) };
	return find(pairs, nbPairs, pair);
}

// Rigid against volume:
// ref0 is the rigid's filter reference, see PxgEncodeRigidFilterRef.
// ref1 is a tetrahedron of the volume.
static __device__ PX_FORCE_INLINE bool findRigidVolumeFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxNodeIndex& rigidId, const PxU32 rigidCacheRef, const PxU32 volumeId, const PxU32 volumeTetId)
{
	PxgDeformablePairInfo pair = { PxgEncodeRigidFilterRef(rigidId, rigidCacheRef), PxgEncodeDeformableTetrahedron(volumeId, volumeTetId) };
	return find(pairs, nbPairs, pair);
}

// Volume against surface, for the paths that resolve a single vertex:
// ref0 is a tetrahedron of the volume.
// ref1 is a vertex of the surface.
// Vertices and triangles are resolved from the same pair list based on the feature type in the reference.
static __device__ PX_FORCE_INLINE bool findVolumeSurfaceVertexFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxU32 volumeId, const PxU32 volumeTetId, const PxU32 surfaceId, const PxU32 surfaceVertexId)
{
	PxgDeformablePairInfo pair = { PxgEncodeDeformableTetrahedron(volumeId, volumeTetId),
								   PxgEncodeDeformableVertex(surfaceId, surfaceVertexId) };
	return find(pairs, nbPairs, pair);
}

// Volume against surface, for the paths that resolve a whole triangle:
// ref0 is a tetrahedron of the volume.
// ref1 is a triangle of the surface.
// Vertices and triangles are resolved from the same pair list based on the feature type in the reference.
static __device__ PX_FORCE_INLINE bool findVolumeSurfaceTriangleFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxU32 volumeId, const PxU32 volumeTetId, const PxU32 surfaceId, const PxU32 surfaceTriId)
{
	PxgDeformablePairInfo pair = { PxgEncodeDeformableTetrahedron(volumeId, volumeTetId),
								   PxgEncodeDeformableTriangle(surfaceId, surfaceTriId) };
	return find(pairs, nbPairs, pair);
}

// Rigid against surface, for the paths that resolve a single vertex:
// ref0 is the rigid's filter reference, see PxgEncodeRigidFilterRef.
// ref1 is a vertex of the surface.
// Vertices and triangles are resolved from the same pair list based on the feature type in the reference.
static __device__ PX_FORCE_INLINE bool findRigidSurfaceVertexFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxNodeIndex& rigidId, const PxU32 rigidCacheRef, const PxU32 surfaceId, const PxU32 surfaceVertexId)
{
	PxgDeformablePairInfo pair = { PxgEncodeRigidFilterRef(rigidId, rigidCacheRef), PxgEncodeDeformableVertex(surfaceId, surfaceVertexId) };
	return find(pairs, nbPairs, pair);
}

// Rigid against surface, for the paths that resolve a whole triangle:
// ref0 is the rigid's filter reference, see PxgEncodeRigidFilterRef.
// ref1 is a triangle of the surface.
// Vertices and triangles are resolved from the same pair list based on the feature type in the reference.
static __device__ PX_FORCE_INLINE bool findRigidSurfaceTriangleFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxNodeIndex& rigidId, const PxU32 rigidCacheRef, const PxU32 surfaceId, const PxU32 surfaceTriId)
{
	PxgDeformablePairInfo pair = { PxgEncodeRigidFilterRef(rigidId, rigidCacheRef), PxgEncodeDeformableTriangle(surfaceId, surfaceTriId) };
	return find(pairs, nbPairs, pair);
}

// Surface against surface, for the paths that resolve a vertex against a triangle:
// ref0 is a vertex of the probing surface.
// ref1 is a triangle of the other surface.
// Vertex-face and edge-edge are resolved from the same pair list based on the feature type in the reference.
static __device__ PX_FORCE_INLINE bool findSurfaceSurfaceVertexTriangleFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxU32 surfaceId0, const PxU32 surfaceVertexId0, const PxU32 surfaceId1, const PxU32 surfaceTriId1)
{
	PxgDeformablePairInfo pair = { PxgEncodeDeformableVertex(surfaceId0, surfaceVertexId0),
								   PxgEncodeDeformableTriangle(surfaceId1, surfaceTriId1) };
	return find(pairs, nbPairs, pair);
}

// Surface against surface, for the paths that resolve two triangle edges:
// ref0 is a triangle edge of the probing surface.
// ref1 is a triangle edge of the other surface.
// Vertex-face and edge-edge are resolved from the same pair list based on the feature type in the reference.
static __device__ PX_FORCE_INLINE bool findSurfaceSurfaceEdgeEdgeFilter(const PxgDeformablePairInfo* pairs, const PxU32 nbPairs,
	const PxU32 surfaceId0, const PxU32 surfaceTriId0, const PxU32 localEdge0,
	const PxU32 surfaceId1, const PxU32 surfaceTriId1, const PxU32 localEdge1)
{
	PxgDeformablePairInfo pair = { PxgEncodeDeformableTriangleEdge(surfaceId0, surfaceTriId0, localEdge0),
								   PxgEncodeDeformableTriangleEdge(surfaceId1, surfaceTriId1, localEdge1) };
	return find(pairs, nbPairs, pair);
}

} // namespace physx

#endif // __DEFORMABLE_ELEMENT_FILTER_CUH__