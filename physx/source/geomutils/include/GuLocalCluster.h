// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_LOCAL_CLUSTER_H
#define GU_LOCAL_CLUSTER_H

#include "common/PxPhysXCommonConfig.h"
#include "foundation/PxTransform.h"
#include "foundation/PxInlineArray.h"
#include "geometry/PxTriangle.h"

namespace physx
{
	class PxTriangleMeshGeometry;
	class PxMeshScale;

namespace Gu
{
	// PT: a local cluster is a triangle and its immediate neighbor.
	// For a regular triangle with at most 3 neighbors the structure will not allocate extra memory.
	struct LocalCluster
	{
		PxInlineArray<PxTriangle, 4>	mTris;		// Space for a source triangle and its 3 direct neighbors.
		PxInlineArray<PxVec3, 4>    	mNormals;	// Precomputed triangle normals.
		float							mExtent;	// Precomputed size of the bounds around all triangles.

		PX_PHYSX_COMMON_API	void reset();
		PX_PHYSX_COMMON_API	void init(const float* src, PxU32 nbFloats);
		PX_PHYSX_COMMON_API	void addTriangle(PxU32 triangleIndex, const PxMeshScale& scale, const PxTransform* meshPose, const PxVec3* verts, const void* triangles, PxU32 has16BitIndices);
	};

	class EdgeList;

	PX_PHYSX_COMMON_API	void createLocalCluster(LocalCluster& mesh, const PxTriangleMeshGeometry& meshGeom, const PxTransform* meshPose, PxU32 triIndex, const EdgeList* edgeList = NULL);
	PX_PHYSX_COMMON_API	float depenetrateLocalClusters(const LocalCluster& mesh0, const LocalCluster& mesh1, const PxVec3& inputDir, bool iterative);

} // namespace Gu
}

#endif
