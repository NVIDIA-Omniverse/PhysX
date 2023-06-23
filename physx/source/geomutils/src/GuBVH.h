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

#ifndef GU_BVH_H
#define GU_BVH_H

/** \addtogroup geomutils
@{
*/

#include "geometry/PxBVH.h"

#include "CmRefCountable.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxUserAllocated.h"
#include "GuAABBTreeBounds.h"
#include "GuAABBTree.h"

namespace physx
{
	struct PxBVHInternalData;

namespace Gu
{
	class MeshFactory;
	struct BVHNode;
	class ShapeData;

	class BVHData : public BVHPartialRefitData
	{
		public:
						BVHData()		{}

						BVHData(BVHData& other)
						{
							mNbIndices	= other.mNbIndices;
							mNbNodes	= other.mNbNodes;
							mIndices	= other.mIndices;
							mNodes		= other.mNodes;

							mBounds.moveFrom(other.mBounds);
							other.mIndices = NULL;
							other.mNodes = NULL;
						}

						~BVHData()
						{
							if(mBounds.ownsMemory())
							{
								mBounds.release();
								PX_FREE(mIndices);
								PX_FREE(mNodes);	// PT: TODO: fix this, unify with AABBTree version
							}
							mNbNodes = 0;
							mNbIndices = 0;
						}

		PX_PHYSX_COMMON_API	bool	build(PxU32 nbBounds, const void* boundsData, PxU32 boundsStride, float enlargement, PxU32 numPrimsPerLeaf, BVHBuildStrategy bs);
		PX_PHYSX_COMMON_API	bool	save(PxOutputStream& stream, bool endian) const;

		AABBTreeBounds	mBounds;
	};

	/**
	\brief Represents a BVH.
	*/
	class BVH : public PxBVH, public PxUserAllocated, public Cm::RefCountable
	{
		public:
		// PT: TODO: revisit these PX_PHYSX_COMMON_API calls. At the end of the day the issue is that things like PxUserAllocated aren't exported.
		PX_PHYSX_COMMON_API						BVH(MeshFactory* factory);
		PX_PHYSX_COMMON_API						BVH(MeshFactory* factory, BVHData& data);
		PX_PHYSX_COMMON_API						BVH(const PxBVHInternalData& data);
		virtual									~BVH();

		PX_PHYSX_COMMON_API	bool				init(PxU32 nbPrims, AABBTreeBounds* bounds, const void* boundsData, PxU32 stride, BVHBuildStrategy bs, PxU32 nbPrimsPerLeaf, float enlargement);
							bool				load(PxInputStream& desc);
							void				release();

		// PxBVH
		virtual				bool				raycast(const PxVec3& origin, const PxVec3& unitDir, float distance, RaycastCallback& cb, PxGeometryQueryFlags flags)							const	PX_OVERRIDE;
		virtual				bool				overlap(const PxGeometry& geom, const PxTransform& pose, OverlapCallback& cb, PxGeometryQueryFlags flags)										const	PX_OVERRIDE;
		virtual				bool				sweep(const PxGeometry& geom, const PxTransform& pose, const PxVec3& unitDir, float distance, RaycastCallback& cb, PxGeometryQueryFlags flags)	const	PX_OVERRIDE;
		virtual				bool				cull(PxU32 nbPlanes, const PxPlane* planes, OverlapCallback& cb, PxGeometryQueryFlags flags)													const	PX_OVERRIDE;

		virtual				PxU32				getNbBounds()	const PX_OVERRIDE	{ return mData.mNbIndices;			}
		virtual				const PxBounds3*	getBounds()		const PX_OVERRIDE	{ return mData.mBounds.getBounds();	}

		virtual				void				refit()														PX_OVERRIDE;
		virtual				bool				updateBounds(PxU32 boundsIndex, const PxBounds3& newBounds)	PX_OVERRIDE;
		virtual				void				partialRefit()												PX_OVERRIDE;

		virtual				bool				traverse(TraversalCallback& cb)	const	PX_OVERRIDE;
		//~PxBVH

		// Cm::RefCountable
		virtual				void				onRefCountZero()	PX_OVERRIDE;
		//~Cm::RefCountable

		PX_FORCE_INLINE		const BVHNode*		getNodes()		const	{ return mData.mNodes;		}
		PX_FORCE_INLINE		const PxU32*		getIndices()	const	{ return mData.mIndices;	}
		PX_FORCE_INLINE		const BVHData&		getData()		const	{ return mData;				}

							bool				getInternalData(PxBVHInternalData&, bool)	const;
							bool				updateBoundsInternal(PxU32 localIndex, const PxBounds3& bounds);
		// PT: alternative implementations directly working on shape data
							bool				overlap(const ShapeData& shapeData, OverlapCallback& cb, PxGeometryQueryFlags flags)										const;
							bool				sweep(const ShapeData& shapeData, const PxVec3& unitDir, float distance, RaycastCallback& cb, PxGeometryQueryFlags flags)	const;
		private:
							MeshFactory*		mMeshFactory;
							BVHData				mData;
	};
}

}

/** @} */
#endif
