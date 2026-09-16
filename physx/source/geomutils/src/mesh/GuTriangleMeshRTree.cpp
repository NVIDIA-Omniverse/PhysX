// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuTriangleMesh.h"
#include "GuTriangleMeshRTree.h"

using namespace physx;

namespace physx
{

Gu::RTreeTriangleMesh::RTreeTriangleMesh(MeshFactory* factory, TriangleMeshData& d) : TriangleMesh(factory, d)
{
	PX_ASSERT(d.mType==PxMeshMidPhase::eBVH33);

	RTreeTriangleData& rtreeData = static_cast<RTreeTriangleData&>(d);
	mRTree = rtreeData.mRTree;
	rtreeData.mRTree.mPages = NULL;
}

Gu::TriangleMesh* Gu::RTreeTriangleMesh::createObject(PxU8*& address, PxDeserializationContext& context)
{
	RTreeTriangleMesh* obj = PX_PLACEMENT_NEW(address, RTreeTriangleMesh(PxBaseFlag::eIS_RELEASABLE));
	address += sizeof(RTreeTriangleMesh);	
	obj->importExtraData(context);
	return obj;
}

void Gu::RTreeTriangleMesh::exportExtraData(PxSerializationContext& stream)
{
	mRTree.exportExtraData(stream);
	TriangleMesh::exportExtraData(stream);
}

void Gu::RTreeTriangleMesh::importExtraData(PxDeserializationContext& context)
{
	mRTree.importExtraData(context);
	TriangleMesh::importExtraData(context);
}

PxVec3 * Gu::RTreeTriangleMesh::getVerticesForModification()
{
	return const_cast<PxVec3*>(getVertices());
}

template<typename IndexType>
struct RefitCallback : Gu::RTree::CallbackRefit
{
	const PxVec3* newPositions;
	const IndexType* indices;

	RefitCallback(const PxVec3* aNewPositions, const IndexType* aIndices) : newPositions(aNewPositions), indices(aIndices) {}
	PX_FORCE_INLINE ~RefitCallback() {}

	virtual void recomputeBounds(PxU32 index, aos::Vec3V& aMn, aos::Vec3V& aMx) PX_OVERRIDE
	{
		using namespace aos;

		// Each leaf box has a set of triangles
		Gu::LeafTriangles currentLeaf; currentLeaf.Data = index;
		PxU32 nbTris = currentLeaf.GetNbTriangles();
		PxU32 baseTri = currentLeaf.GetTriangleIndex();
		PX_ASSERT(nbTris > 0);
		const IndexType* vInds = indices + 3 * baseTri;
		Vec3V vPos = V3LoadU(newPositions[vInds[0]]);
		Vec3V mn = vPos, mx = vPos;
		//PxBounds3 result(newPositions[vInds[0]], newPositions[vInds[0]]);
		vPos = V3LoadU(newPositions[vInds[1]]);
		mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
		vPos = V3LoadU(newPositions[vInds[2]]);
		mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
		for (PxU32 i = 1; i < nbTris; i++)
		{
			const IndexType* vInds1 = indices + 3 * (baseTri + i);
			vPos = V3LoadU(newPositions[vInds1[0]]);
			mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
			vPos = V3LoadU(newPositions[vInds1[1]]);
			mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
			vPos = V3LoadU(newPositions[vInds1[2]]);
			mn = V3Min(mn, vPos); mx = V3Max(mx, vPos);
		}

		aMn = mn;
		aMx = mx;
	}
};

PxBounds3 Gu::RTreeTriangleMesh::refitBVH()
{
	PxBounds3 meshBounds;
	if (has16BitIndices())
	{
		RefitCallback<PxU16> cb(mVertices, static_cast<const PxU16*>(mTriangles));
		mRTree.refitAllStaticTree(cb, &meshBounds);
	}
	else
	{
		RefitCallback<PxU32> cb(mVertices, static_cast<const PxU32*>(mTriangles));
		mRTree.refitAllStaticTree(cb, &meshBounds);
	}

	// reset edge flags and remember we did that using a mesh flag (optimization)
	if ((mRTree.mFlags & RTree::IS_EDGE_SET) == 0)
	{
		mRTree.mFlags |= RTree::IS_EDGE_SET;
		setAllEdgesActive();
	}

	mAABB = meshBounds;
	return meshBounds;
}

} // namespace physx
