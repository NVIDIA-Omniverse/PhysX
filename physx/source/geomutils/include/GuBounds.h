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

#ifndef GU_BOUNDS_H
#define GU_BOUNDS_H

#include "foundation/PxBounds3.h"
#include "foundation/PxFlags.h"
#include "foundation/PxVecMath.h"
#include "geometry/PxGeometry.h"
#include "geometry/PxCapsuleGeometry.h"
#include <stddef.h>
#include "GuBox.h"
#include "GuCenterExtents.h"
#include "GuSphere.h"
#include "GuCapsule.h"

// PT: the PX_MAX_BOUNDS_EXTENTS value is too large and produces INF floats when the box values are squared in
// some collision routines. Thus, for the SQ subsystem we use this alternative (smaller) value to mark empty bounds.
// See PX-954 for details.
#define GU_EMPTY_BOUNDS_EXTENTS	PxSqrt(0.25f * 1e33f)

namespace physx
{
	class PxMeshScale;

namespace Gu
{
	PX_FORCE_INLINE void computeCapsuleBounds(PxBounds3& bounds, const PxCapsuleGeometry& capsuleGeom, const PxTransform& pose, float contactOffset=0.0f, float inflation=1.0f)
	{
		const PxVec3 d = pose.q.getBasisVector0();
		PxVec3 extents;
		for(PxU32 ax = 0; ax<3; ax++)
			extents[ax] = (PxAbs(d[ax]) * capsuleGeom.halfHeight + capsuleGeom.radius + contactOffset)*inflation;
		bounds.minimum = pose.p - extents;
		bounds.maximum = pose.p + extents;
	}

	//'contactOffset' and 'inflation' should not be used at the same time, i.e. either contactOffset==0.0f, or inflation==1.0f
	PX_PHYSX_COMMON_API void computeBounds(PxBounds3& bounds, const PxGeometry& geometry, const PxTransform& transform, float contactOffset, float inflation);	//AABB in world space.

	PX_FORCE_INLINE PxBounds3 computeBounds(const PxGeometry& geometry, const PxTransform& pose)
	{
		PxBounds3 bounds;
		computeBounds(bounds, geometry, pose, 0.0f, 1.0f);
		return bounds;
	}

	void computeGlobalBox(PxBounds3& bounds, PxU32 nbPrims, const PxBounds3* PX_RESTRICT boxes, const PxU32* PX_RESTRICT primitives);

	PX_PHYSX_COMMON_API void computeBoundsAroundVertices(PxBounds3& bounds, PxU32 nbVerts, const PxVec3* PX_RESTRICT verts);
	PX_PHYSX_COMMON_API void computeTightBounds(PxBounds3& bounds, PxU32 nbVerts, const PxVec3* PX_RESTRICT verts, const PxTransform& pose, const PxMeshScale& scale, float contactOffset, float inflation);
	PX_PHYSX_COMMON_API void computeLocalBoundsAndGeomEpsilon(const PxVec3* vertices, PxU32 nbVerties, PxBounds3& localBounds, PxReal& geomEpsilon);

	#define StoreBounds(bounds, minV, maxV)	\
		V4StoreU(minV, &bounds.minimum.x);	\
		PX_ALIGN(16, PxVec4) max4;			\
		V4StoreA(maxV, &max4.x);			\
		bounds.maximum = PxVec3(max4.x, max4.y, max4.z);

	// PT: TODO: - refactor with "inflateBounds" in GuBounds.cpp if possible
	template<const bool useSIMD>
	PX_FORCE_INLINE void inflateBounds(PxBounds3& dst, const PxBounds3& src, float enlargement)
	{
		const float coeff = 0.5f * enlargement;
		if(useSIMD)
		{
			using namespace physx::aos;

			Vec4V minV = V4LoadU(&src.minimum.x);
			Vec4V maxV = V4LoadU(&src.maximum.x);
			const Vec4V eV = V4Scale(V4Sub(maxV, minV), FLoad(coeff));

			minV = V4Sub(minV, eV);
			maxV = V4Add(maxV, eV);

			StoreBounds(dst, minV, maxV);
		}
		else
		{
			// PT: this clumsy but necessary second codepath is used to read the last bound of the array
			// (making sure we don't V4LoadU invalid memory). Implementation must stay in sync with the
			// main codepath above. No, this is not very nice.
			const PxVec3& minV = src.minimum;
			const PxVec3& maxV = src.maximum;
			const PxVec3 eV = (maxV - minV) * coeff;
			dst.minimum = minV - eV;
			dst.maximum = maxV + eV;
		}
	}

	class ShapeData
	{
	public:

		PX_PHYSX_COMMON_API						ShapeData(const PxGeometry& g, const PxTransform& t, PxReal inflation);	

		// PT: used by overlaps (box, capsule, convex)
		PX_FORCE_INLINE const PxVec3&			getPrunerBoxGeomExtentsInflated()	const	{ return mPrunerBoxGeomExtents; }

		// PT: used by overlaps (box, capsule, convex)
		PX_FORCE_INLINE const PxVec3&			getPrunerWorldPos()					const	{ return mGuBox.center;			}

		PX_FORCE_INLINE const PxBounds3&		getPrunerInflatedWorldAABB()		const	{ return mPrunerInflatedAABB;	}

		// PT: used by overlaps (box, capsule, convex)
		PX_FORCE_INLINE const PxMat33&			getPrunerWorldRot33()				const	{ return mGuBox.rot;			}

		// PT: this one only used by overlaps so far (for sphere shape, pruner level)
		PX_FORCE_INLINE const Gu::Sphere&		getGuSphere() const
		{
			PX_ASSERT(mType == PxGeometryType::eSPHERE);
			return reinterpret_cast<const Gu::Sphere&>(mGuSphere);
		}

		// PT: this one only used by sweeps so far (for box shape, NP level)
		PX_FORCE_INLINE const Gu::Box&			getGuBox() const
		{
			PX_ASSERT(mType == PxGeometryType::eBOX);
			return mGuBox;
		}

		// PT: this one used by sweeps (NP level) and overlaps (pruner level) - for capsule shape
		PX_FORCE_INLINE const Gu::Capsule&		getGuCapsule() const
		{
			PX_ASSERT(mType == PxGeometryType::eCAPSULE);
			return reinterpret_cast<const Gu::Capsule&>(mGuCapsule);
		}

		PX_FORCE_INLINE float					getCapsuleHalfHeight() const
		{
			PX_ASSERT(mType == PxGeometryType::eCAPSULE);
			return mGuBox.extents.x;
		}

		PX_FORCE_INLINE	PxU32					isOBB()		const { return PxU32(mIsOBB);				}
		PX_FORCE_INLINE	PxGeometryType::Enum	getType()	const { return PxGeometryType::Enum(mType);	}

		PX_NOCOPY(ShapeData)
	private:

		// PT: box: pre-inflated box extents
		//     capsule: pre-inflated extents of box-around-capsule
		//     convex: pre-inflated extents of box-around-convex
		//     sphere: not used
		PxVec3				mPrunerBoxGeomExtents;	// used for pruners. This volume encloses but can differ from the original shape

		// PT:
		//
		// box center = unchanged copy of initial shape's position, except for convex (position of box around convex)
		// SIMD code will load it as a V4 (safe because member is not last of Gu structure)
		//
		// box rot = precomputed PxMat33 version of initial shape's rotation, except for convex (rotation of box around convex)
		// SIMD code will load it as V4s (safe because member is not last of Gu structure)
		//
		// box extents = non-inflated initial box extents for box shape, half-height for capsule, otherwise not used
		Gu::Box				mGuBox;

		PxBounds3			mPrunerInflatedAABB;	// precomputed AABB for the pruner shape
		PxU16				mIsOBB;					// true for OBB, false for AABB. Also used as padding for mPrunerInflatedAABB, don't move.
		PxU16				mType;					// shape's type

		// these union Gu shapes are only precomputed for narrow phase (not pruners), can be different from mPrunerVolume
		// so need separate storage
		union
		{
			PxU8 mGuCapsule[sizeof(Gu::Capsule)];	// 28
			PxU8 mGuSphere[sizeof(Gu::Sphere)];		// 16
		};
	};

// PT: please make sure it fits in "one" cache line
PX_COMPILE_TIME_ASSERT(sizeof(ShapeData)==128);

}  // namespace Gu

}
#endif
