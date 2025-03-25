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


#ifndef __CONVEXNPCOMMON_H__
#define __CONVEXNPCOMMON_H__

#define NUM_TMP_CONTACTS_PER_PAIR       32

#define CVX_TRI_MAX_CONTACTS            5

#define MAX_MESH_MESH_PATCHES           128

// Patches correlation - same normal threshold
// 8 degrees
//#define PATCH_ACCEPTANCE_EPS          0.990268f
// 5 degrees
#define PATCH_ACCEPTANCE_EPS            0.996194698f
// 3 degrees
//#define PATCH_ACCEPTANCE_EPS          0.99862953f

#define CONVEX_TRIMESH_CACHED           0xFFffFFff

// AD: used to specify sizes of shared memory structs
// for collisions involving convexes. Make sure you
// know what you are doing if you change this.
#define CONVEX_MAX_VERTICES_POLYGONS    64

#include "foundation/PxPreprocessor.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"
#include "foundation/PxQuat.h"
#include "foundation/PxTransform.h"

#include "geometry/PxMeshScale.h"

#include "GuBV32.h"

#include "PxgCommonDefines.h"

#include <vector_types.h>

namespace physx
{
	struct ConvexMeshPair
	{
		physx::PxTransform aToB;		// pB = aToB.transform(pA + normal * separation)										28
		int startIndex;			// start index in the ConvexTriContacts and ConvexTriPairs arrays								32
		int count;				// the number of triangles in the ConvexTriContacts and ConvexTriPairs							36
		int cmIndex;			// index of the original CM, to index into the output buffer and pass on to contact reporting	40
		int roundedStartIndex;	// start index in the sorted triangle index array. For each pair, the number of indices			44 
								// assigned to is the number of triangles padded to a multiple of 4 and duplicated to have
								// space for the temp buffer for the radix sort. The start index is aligned to a mutliple of 4.
		int pad;				// pad																							48
		uint2 materialIndices;	// material index for shape0 and shape1															56
	};

	struct ConvexTriNormalAndIndex
	{
		static const PxU32 DeferredContactMask = 0x80000000;
		static const PxU32 NbContactsShift = 27;
		physx::PxVec3 normal;		// normal
		int index;					// isDeferredContact is in bit 31, nbContacts is in bits 27-30, rest is cpu mesh index for the triangle

		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getTriangleIndex(PxU32 index)
		{
			return (index & ((1<<NbContactsShift)-1));
		}

		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 getNbContacts(PxU32 index)
		{
			return (index & (~DeferredContactMask)) >> NbContactsShift;
		}

		static PX_CUDA_CALLABLE PX_FORCE_INLINE PxU32 isDeferred(const PxU32 index)
		{
			return index & DeferredContactMask;
		}
	};

	struct ConvexTriContacts
	{
		PxU32 index;
	};

	struct ConvexTriContact
	{
		float4 contact_sepW;
	};

	//each convex triangle pairs will generate the intermediate data in case we need to
	//do post process of this pair
	struct ConvexTriIntermediateData
	{
		//Enums to indicate the barycentric weighting of the contacts
		enum VoronoiState: PxU32
		{
			eV0 = 1u << 29,
			eV1 = 2u << 29,
			eV2 = 3u << 29,
			eE01 = 4u << 29,
			eE02 = 5u << 29,
			eE12 = 6u << 29,
			eMASK = 0xe0000000
		};

		physx::PxU32 gpuTriIndex;				//gpu triangle index

		static PX_CUDA_CALLABLE PX_FORCE_INLINE bool isEdgeContact(const PxU32 mask)
		{
			return mask & 0x80000000;
		}
	};

	typedef ConvexMeshPair SphereMeshPair;
	typedef ConvexTriContacts SphereTriContacts;
	typedef ConvexTriNormalAndIndex SphereTriNormalAndIndex;
	typedef ConvexTriIntermediateData SphereTriIntermediateData;
}


#if PX_CUDA_COMPILER

#include "PxgCommonDefines.h"

namespace physx
{

	PX_ALIGN_PREFIX(16)
	struct MidphaseScratch
	{
		const float4 * PX_RESTRICT trimeshVerts;
		const uint4 * PX_RESTRICT trimeshTriIndices;
		PxTransform shapeToMeshNoScale;
		PxTransform meshToWorld;
		PxMeshScale trimeshScale;
		PxVec3 inflatedExtents;
		PxVec3 center;
		PxU32 shape_materialIndex;
		PxU32 trimeshShape_materialIndex;
		
		//bv32 tree
		const Gu::BV32DataPacked* bv32PackedNodes;

		//stack for traversal
		int sBv32Nodes[320]; //10 depth of the bv32 tree
	}PX_ALIGN_SUFFIX(16);
	PX_COMPILE_TIME_ASSERT(sizeof(MidphaseScratch) <= WARP_SIZE * 16 * sizeof(PxU32));
}

#include "schlockShared.h"

//0 is primitive, 1 is tetrahedron world space
struct TetCollideScratch
{
	physx::PxVec3 vA[CONVEX_MAX_VERTICES_POLYGONS];
	physx::PxVec3 vB[4];
	schlock::GjkCachedData cachedData;
	schlock::GjkOutput gjkOutput;

	physx::PxQuat rot0;
	physx::PxVec3 scale0;

	physx::PxReal contactDistance;
	physx::PxReal inSphereRadius0;
	physx::PxReal inSphereRadius1;

	physx::PxVec3 searchDir;

	physx::PxU8 nbVertices0;
	physx::PxU8 nbVertices1;

	physx::PxU32 tetIndex0;
	physx::PxU32 tetIndex1;

	__device__ void Clear()
	{
		if (threadIdx.x == 0)
			cachedData.size = 0;
		__syncwarp();
	}
};

#include "epa.cuh"
#include "gjk.cuh"

__device__ static schlock::GjkResult::Enum tetPrimitivesCollide2(
		squawk::EpaScratch& ss_epa_scratch,
		TetCollideScratch& ss_scratch,
		const physx::PxU32 globalWarpIndex)
{
	typedef schlock::GjkResult GjkResult;

	const physx::PxReal convergenceRatio = 1 - 0.000225f;

	assert(ss_scratch.nbVertices0 <= 64);
	assert(ss_scratch.nbVertices1 <= 64);

	physx::PxVec3 initialDir(1, 0, 0);

	physx::PxVec3 aToB_p = ss_scratch.searchDir; // ss_scratch.aToB.p;

	if (aToB_p.magnitudeSquared() > 0)
		initialDir = aToB_p; // GJK's initial start dir is usually from the warm cache => lazy normalize inside GJK

	GjkResult::Enum gjkResult = squawk::gjk(
		ss_scratch.vA, ss_scratch.nbVertices0,
		ss_scratch.vB, ss_scratch.nbVertices1,
		initialDir,
		ss_scratch.contactDistance,
		convergenceRatio,
		ss_scratch.gjkOutput, ss_scratch.cachedData);

	__syncwarp();

	if (gjkResult == GjkResult::eDISTANT)
	{
		return GjkResult::eDISTANT;
	}

	bool separated = (gjkResult == GjkResult::eCLOSE);
	bool anomaly = separated && ss_scratch.gjkOutput.closestPointDir.dot(ss_scratch.gjkOutput.direction) < 0.999f;

	GjkResult::Enum result = GjkResult::eCLOSE;

	if (!separated || anomaly)
	{
		GjkResult::Enum epaResult = squawk::epa(ss_epa_scratch,
			ss_scratch.vA, ss_scratch.nbVertices0,
			ss_scratch.vB, ss_scratch.nbVertices1,
			&(ss_scratch.cachedData),
			convergenceRatio,
			0.5f * (ss_scratch.inSphereRadius0 + ss_scratch.inSphereRadius1),
			ss_scratch.gjkOutput);

		//separated = epaResult == GjkResult::eCLOSE;

		result = epaResult;

		__syncwarp();

		if (ss_scratch.gjkOutput.degenerate)
		{
			//we need to re-run gjk epa with other configurations
			ss_scratch.cachedData.size = 0;

			//we need to re-run gjk epa with other configurations
			initialDir = physx::PxVec3(0.f, 1.f, 0.f);

			GjkResult::Enum gjkResult = squawk::gjk(
				ss_scratch.vA, ss_scratch.nbVertices0,
				ss_scratch.vB, ss_scratch.nbVertices1,
				initialDir,
				ss_scratch.contactDistance,
				convergenceRatio,
				ss_scratch.gjkOutput, ss_scratch.cachedData);

			__syncwarp();

			GjkResult::Enum epaResult = squawk::epa(ss_epa_scratch,
				ss_scratch.vA, ss_scratch.nbVertices0,
				ss_scratch.vB, ss_scratch.nbVertices1,
				&(ss_scratch.cachedData),
				convergenceRatio,
				0.5f * (ss_scratch.inSphereRadius0 + ss_scratch.inSphereRadius1),
				ss_scratch.gjkOutput);

			//separated = epaResult == GjkResult::eCLOSE;

			result = epaResult;
		}
		__syncwarp();
	}
	return result;
}



#endif

#endif
