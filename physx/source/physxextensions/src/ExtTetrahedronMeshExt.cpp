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

#include "extensions/PxTetrahedronMeshExt.h"
#include "foundation/PxMathUtils.h"
#include "foundation/PxHashMap.h"
#include "GuTetrahedronMesh.h"
#include "GuBox.h"
#include "GuBV4.h"
#include "GuBV4_Common.h"
#include "GuDistancePointTetrahedron.h"

namespace physx
{
	struct TetrahedronFinderCallback
	{
		PxVec3 mQueryPoint;
		PxI32 mTetId;
		PxVec4 mBary;
		
		const PxVec3* mVertices;
		const PxU32* mTets;

		PxReal mTolerance = 1e-6f;

		TetrahedronFinderCallback(const PxVec3& queryPoint, const PxVec3* vertices, const PxU32* tets, const PxReal tolerance = 1e-6f) :
			mQueryPoint(queryPoint), mTetId(-1), mVertices(vertices), mTets(tets), mTolerance(tolerance) {}


		PX_FORCE_INLINE	bool testPrimitive(const PxU32 primitiveStartId, const PxU32 numPrimitives)
		{
			for (PxU32 i = 0; i < numPrimitives; ++i) 
			{
				const PxU32* tet = &mTets[4 * (primitiveStartId + i)];
				computeBarycentric(mVertices[tet[0]], mVertices[tet[1]], mVertices[tet[2]], mVertices[tet[3]], mQueryPoint, mBary);

				if (mBary.x >= -mTolerance && mBary.x <= 1 + mTolerance && mBary.y >= -mTolerance && mBary.y <= 1 + mTolerance &&
					mBary.z >= -mTolerance && mBary.z <= 1 + mTolerance && mBary.w >= -mTolerance && mBary.w <= 1 + mTolerance)
				{
					mTetId = PxI32(primitiveStartId + i);
					return true;
				}
			}
			return false;
		}

		PX_FORCE_INLINE	bool testBox(const float boxMinX, const float boxMinY, const float boxMinZ, const float boxMaxX, const float boxMaxY, const float boxMaxZ)
		{
			return mQueryPoint.x >= boxMinX && mQueryPoint.y >= boxMinY && mQueryPoint.z >= boxMinZ &&
				mQueryPoint.x <= boxMaxX && mQueryPoint.y <= boxMaxY && mQueryPoint.z <= boxMaxZ;
		}
	};

	struct ClosestTetrahedronFinderCallback
	{
		PxVec3 mQueryPoint;
		PxI32 mTetId;
		PxVec4 mBary;
		PxReal mDist = 1.84467e+19f; // sqrtf(FLT_MAX)

		const PxVec3* mVertices;
		const PxU32* mTets;
		PxReal mTolerance = 1e-6f;

		ClosestTetrahedronFinderCallback(const PxVec3& queryPoint, const PxVec3* vertices, const PxU32* tets) :
			mQueryPoint(queryPoint), mTetId(-1), mVertices(vertices), mTets(tets) {}

		PX_FORCE_INLINE	bool testPrimitive(const PxU32 primitiveStartId, const PxU32 numPrimitives)
		{
			for (PxU32 i = 0; i < numPrimitives; ++i)
			{
				PxVec4 bary;
				const PxU32* tet = &mTets[4 * (primitiveStartId + i)];
				computeBarycentric(mVertices[tet[0]], mVertices[tet[1]], mVertices[tet[2]], mVertices[tet[3]], mQueryPoint, bary);

				if (bary.x >= -mTolerance && bary.x <= 1 + mTolerance && bary.y >= -mTolerance && bary.y <= 1 + mTolerance &&
					bary.z >= -mTolerance && bary.z <= 1 + mTolerance && bary.w >= -mTolerance && bary.w <= 1 + mTolerance)
				{
					mTetId = PxI32(primitiveStartId + i);
					mBary = bary;
					mDist = 0;
					return true;
				}

				PxVec3 closest = Gu::closestPtPointTetrahedron(mQueryPoint, mVertices[tet[0]], mVertices[tet[1]], mVertices[tet[2]], mVertices[tet[3]]);
				PxReal distSq = (closest - mQueryPoint).magnitudeSquared();
				if (distSq < mDist * mDist)
				{
					mTetId = PxI32(primitiveStartId + i);
					mBary = bary;
					mDist = PxSqrt(distSq);
				}
			}
			return false;
		}

		PX_FORCE_INLINE	bool testBox(const float boxMinX, const float boxMinY, const float boxMinZ, const float boxMaxX, const float boxMaxY, const float boxMaxZ)
		{
			if (mQueryPoint.x >= boxMinX && mQueryPoint.y >= boxMinY && mQueryPoint.z >= boxMinZ &&
				mQueryPoint.x <= boxMaxX && mQueryPoint.y <= boxMaxY && mQueryPoint.z <= boxMaxZ)
				return true;
			PxVec3 closest = mQueryPoint;
			closest.x = PxClamp(closest.x, boxMinX, boxMaxX);
			closest.y = PxClamp(closest.y, boxMinY, boxMaxY);
			closest.z = PxClamp(closest.z, boxMinZ, boxMaxZ);
			PxReal distSq = (closest - mQueryPoint).magnitudeSquared();
			return distSq < mDist * mDist;
		}
	};

	
	template<typename T, PxU32 i>
	int process(PxU32* stack, PxU32& stackSize, const Gu::BVDataSwizzledNQ* node, T& callback)
	{
		if (callback.testBox(node->mMinX[i], node->mMinY[i], node->mMinZ[i], node->mMaxX[i], node->mMaxY[i], node->mMaxZ[i]))
		{
			if (node->isLeaf(i))
			{
				PxU32 primitiveIndex = node->getPrimitive(i);
				const PxU32 numPrimitives = Gu::getNbPrimitives(primitiveIndex);
				if(callback.testPrimitive(primitiveIndex, numPrimitives)) //Returns true if the query should be terminated immediately
					return 1;
			}
			else
				stack[stackSize++] = node->getChildData(i);
		}
		return 0;
	}

	template<typename T>
	void traverseBVH(const Gu::BV4Tree& tree, T& callback)
	{
		const Gu::BVDataPackedNQ* root = static_cast<const Gu::BVDataPackedNQ*>(tree.mNodes);

		PxU32 stack[GU_BV4_STACK_SIZE];
		PxU32 stackSize = 0;
		stack[stackSize++] = tree.mInitData;

		while (stackSize > 0)
		{
			const PxU32 childData = stack[--stackSize];
			const Gu::BVDataSwizzledNQ* node = reinterpret_cast<const Gu::BVDataSwizzledNQ*>(root + Gu::getChildOffset(childData));
			
			const PxU32 nodeType = Gu::getChildType(childData);

			if (nodeType > 1)
				if (process<T, 3>(stack, stackSize, node, callback)) return;
			if (nodeType > 0)
				if (process<T, 2>(stack, stackSize, node, callback)) return;
			if (process<T, 1>(stack, stackSize, node, callback)) return;
			if (process<T, 0>(stack, stackSize, node, callback)) return;
		}
	}

	PxI32 PxTetrahedronMeshExt::findTetrahedronContainingPoint(const PxTetrahedronMesh* mesh, const PxVec3& point, PxVec4& bary, PxReal tolerance)
	{
		TetrahedronFinderCallback callback(point, mesh->getVertices(), static_cast<const PxU32*>(mesh->getTetrahedrons()), tolerance);
		traverseBVH(static_cast<const Gu::BVTetrahedronMesh*>(mesh)->getBV4Tree(), callback);
		bary = callback.mBary;
		return callback.mTetId;
	}

	PxI32 PxTetrahedronMeshExt::findTetrahedronClosestToPoint(const PxTetrahedronMesh* mesh, const PxVec3& point, PxVec4& bary)
	{
		ClosestTetrahedronFinderCallback callback(point, mesh->getVertices(), static_cast<const PxU32*>(mesh->getTetrahedrons()));
		const Gu::BV4Tree& tree = static_cast<const Gu::BVTetrahedronMesh*>(mesh)->getBV4Tree();
		if (tree.mNbNodes) traverseBVH(tree, callback);
		else callback.testPrimitive(0, mesh->getNbTetrahedrons());
		bary = callback.mBary;
		return callback.mTetId;
	}

	struct SortedTriangle
	{
	public:
		PxU32 A;
		PxU32 B;
		PxU32 C;
		PxI32 TetIndex;
		bool Flipped;

		PX_FORCE_INLINE SortedTriangle(PxU32 a, PxU32 b, PxU32 c, PxI32 tetIndex = -1)
		{
			A = a; B = b; C = c; Flipped = false; TetIndex = tetIndex;
			if (A > B) { PxSwap(A, B); Flipped = !Flipped; }
			if (B > C) { PxSwap(B, C); Flipped = !Flipped; }
			if (A > B) { PxSwap(A, B); Flipped = !Flipped; }
		}
	};

	struct TriangleHash
	{
		PX_FORCE_INLINE std::size_t operator()(const SortedTriangle& k) const
		{
			return k.A ^ k.B ^ k.C;
		}

		PX_FORCE_INLINE bool equal(const SortedTriangle& first, const SortedTriangle& second) const
		{
			return first.A == second.A && first.B == second.B && first.C == second.C;
		}
	};

	static const PxU32 tetFaces[4][3] = { {0, 2, 1},  {0, 1, 3},  {0, 3, 2}, {1, 2, 3} };
	

	void PxTetrahedronMeshExt::extractTetMeshSurface(const void* tetrahedra, PxU32 numTetrahedra, bool sixteenBitIndices, PxArray<PxU32>& surfaceTriangles, PxArray<PxU32>* surfaceTriangleToTet, bool flipTriangleOrientation)
	{
		PxHashMap<SortedTriangle, PxU32, TriangleHash> tris;

		const PxU32* tets32 = reinterpret_cast<const PxU32*>(tetrahedra);
		const PxU16* tets16 = reinterpret_cast<const PxU16*>(tetrahedra);

		PxU32 l = 4 * numTetrahedra;
		for (PxU32 i = 0; i < l; i += 4)
		{
			for (PxU32 j = 0; j < 4; ++j)
			{
				SortedTriangle tri(sixteenBitIndices ? tets16[i + tetFaces[j][0]] : tets32[i + tetFaces[j][0]], 
					sixteenBitIndices ? tets16[i + tetFaces[j][1]] : tets32[i + tetFaces[j][1]],
					sixteenBitIndices ? tets16[i + tetFaces[j][2]] : tets32[i + tetFaces[j][2]], i);
				if (const PxPair<const SortedTriangle, PxU32>* ptr = tris.find(tri))
					tris[tri] = ptr->second + 1;
				else
					tris.insert(tri, 1);
			}
		}

		surfaceTriangles.clear();
		if (surfaceTriangleToTet)
			surfaceTriangleToTet->clear();
		for (PxHashMap<SortedTriangle, PxU32, TriangleHash>::Iterator iter = tris.getIterator(); !iter.done(); ++iter)
		{
			if (iter->second == 1) {
				surfaceTriangles.pushBack(iter->first.A);
				if (iter->first.Flipped != flipTriangleOrientation)
				{
					surfaceTriangles.pushBack(iter->first.C);
					surfaceTriangles.pushBack(iter->first.B);
				}
				else
				{
					surfaceTriangles.pushBack(iter->first.B);
					surfaceTriangles.pushBack(iter->first.C);
				}
				if (surfaceTriangleToTet)
					surfaceTriangleToTet->pushBack(iter->first.TetIndex);
			}
		}
	}

	void PxTetrahedronMeshExt::extractTetMeshSurface(const PxTetrahedronMesh* mesh, PxArray<PxU32>& surfaceTriangles, PxArray<PxU32>* surfaceTriangleToTet, bool flipTriangleOrientation)
	{
		extractTetMeshSurface(mesh->getTetrahedrons(), mesh->getNbTetrahedrons(), mesh->getTetrahedronMeshFlags() & PxTetrahedronMeshFlag::e16_BIT_INDICES, surfaceTriangles, surfaceTriangleToTet, flipTriangleOrientation);
	}
}
