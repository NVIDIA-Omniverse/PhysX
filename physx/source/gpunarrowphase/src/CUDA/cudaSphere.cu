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


#include "foundation/PxVec3.h"
#include "foundation/PxMat34.h"
#include "foundation/PxTransform.h"
#include "foundation/PxAssert.h"
#include <stdio.h>

#include "cuda.h"
#include "cuda_runtime.h"
#include "convexFormat.h"
#include "cudaNpCommon.h"

#include "PxsTransformCache.h"
#include "PxsContactManagerState.h"
#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgPersistentContactManifold.h"

#include "PxgNpKernelIndices.h"

#include "PxsMaterialCore.h"

#include "PxContact.h"
#include "geometry/PxGeometry.h"

#include "SparseRemove.cuh"

#include "sphereCollision.cuh"
#include "contactPatchUtils.cuh"

using namespace physx;

extern "C" __host__ void initNarrowphaseKernels9() {}

extern "C" __global__ void sphereNphase_Kernel(
	PxU32 numTests,
	const PxReal toleranceLength,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	PxgShape* PX_RESTRICT shapes,
	const PxsCachedTransform* PX_RESTRICT transformCache,
	const PxReal* contactDistance,
	const PxsMaterialData* PX_RESTRICT materials,
	PxU8* PX_RESTRICT contactStream,
	PxU8* PX_RESTRICT patchStream,
	PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU32* PX_RESTRICT touchChangeFlags,
	PxU32* PX_RESTRICT patchChangeFlags,
	PxU8* PX_RESTRICT startContactPatches,
	PxU8* PX_RESTRICT startContactPoints,
	PxU8* PX_RESTRICT startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit)
{
	PxU32 globalThreadIndex = blockIdx.x * blockDim.x + threadIdx.x;

	if (globalThreadIndex >= numTests)
		return;

	PxgContactManagerInput contactInput = cmInputs[globalThreadIndex];

	PxU32 transformCacheRef0 = contactInput.transformCacheRef0;
	PxU32 transformCacheRef1 = contactInput.transformCacheRef1;
	
	PxsCachedTransform transformCache0 = transformCache[transformCacheRef0];
	PxsCachedTransform transformCache1 = transformCache[transformCacheRef1];

	const PxReal cDistance = contactDistance[transformCacheRef0] + contactDistance[transformCacheRef1];

	PxgShape& shape0 = shapes[contactInput.shapeRef0];
	PxgShape& shape1 = shapes[contactInput.shapeRef1];

	PxGeometryType::Enum type0 = PxGeometryType::Enum(shape0.type);
	PxGeometryType::Enum type1 = PxGeometryType::Enum(shape1.type);

	PxVec3 scale0 = shape0.scale.scale;
	PxVec3 scale1 = shape1.scale.scale;

	const bool flip = (type1<type0);

	//shape0 should be always sphere
	if (flip)
	{
		PxSwap(type0, type1);
		PxSwap(scale0, scale1);
		PxSwap(transformCache0, transformCache1);
		//printf("type0 %i type1 %i\n", type0, type1);
	}


	PxTransform& transform0 = transformCache0.transform;
	PxTransform& transform1 = transformCache1.transform;

	const PxReal sphereRadius = scale0.x;

	float4 pointPen[2];
	PxVec3 normal;
	PxU32 nbContacts = 0;

	if (type0 == PxGeometryType::eSPHERE)
	{

		switch (type1)
		{
		case PxGeometryType::eSPHERE:
		{
			//printf("sphere/sphere \n");
			const PxReal sphereRadius1 = scale1.x;
			nbContacts = spheresphere(transform0, transform1, sphereRadius, sphereRadius1, cDistance, pointPen[0], normal);
			break;
		}
		case PxGeometryType::ePLANE:
		{
			//printf("sphere/plane \n");
			nbContacts = sphereplane(transform0, transform1, sphereRadius, cDistance, pointPen[0], normal);
			break;
		}
		case PxGeometryType::eCAPSULE:
		{
			//printf("sphere/capsule \n");
			const PxReal capsuleRadius = scale1.y;
			const PxReal halfHeight = scale1.x;
			nbContacts = spherecapsule(transform0, transform1, sphereRadius, capsuleRadius, halfHeight, cDistance, pointPen[0], normal);
			break;
		}
		case PxGeometryType::eBOX:
		{
			//printf("sphere/box \n");
			const PxVec3 boxHalfExtents = scale1;
			nbContacts = spherebox(transform0, transform1, sphereRadius, boxHalfExtents, cDistance, pointPen[0], normal);
			break;
		}
		default:
			break;
		};
	}
	else
	{
		assert(type1 == PxGeometryType::eCAPSULE);

		const PxReal capsuleRadius1 = scale1.y;
		const PxReal capsuleHalfHeight1 = scale1.x;

		switch (type0)
		{
		case PxGeometryType::ePLANE:
		{
			//printf("sphere/plane \n");
			nbContacts = planeCapsule(transform0, transform1, capsuleRadius1, capsuleHalfHeight1, cDistance, pointPen, normal);
			break;
		}
		case PxGeometryType::eCAPSULE:
		{
			////printf("sphere/capsule \n");
			const PxReal capsuleRadius0 = scale0.y;
			const PxReal halfHeight0 = scale0.x;
			nbContacts = capsuleCapsule(transform0, transform1, capsuleRadius0, halfHeight0,
				capsuleRadius1, capsuleHalfHeight1, cDistance, pointPen, normal);
			break;
		}
		default:
			break;
		};


	}

	if (nbContacts && flip)
		normal = -normal;

	PxU32 contactByteOffset = setContactPointAndForcePointers(cmOutputs, patchAndContactCounters,
		startContactPoints, startContactForces, contactBytesLimit, forceBytesLimit, globalThreadIndex, nbContacts);

	if (contactByteOffset != 0xFFFFFFFF)
	{
		float4* baseContactStream = (float4*)(contactStream + contactByteOffset);
		for (PxU32 i = 0; i < nbContacts; ++i)
		{
			float4& outPointPen = baseContactStream[i];
			outPointPen = pointPen[i];
		}
	}

	PxU32 patchIndex = registerContactPatch(cmOutputs, patchAndContactCounters, touchChangeFlags, patchChangeFlags, startContactPatches, patchBytesLimit, globalThreadIndex, nbContacts);

	insertIntoPatchStream(materials, patchStream, shape0, shape1, patchIndex, normal, nbContacts);
}



