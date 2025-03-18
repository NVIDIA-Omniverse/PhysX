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

#include "foundation/PxSimpleTypes.h"

#include "cuda.h"
#include "cudaNpCommon.h"

#include "PxsTransformCache.h"
#include "PxsContactManagerState.h"
#include "PxgContactManager.h"
#include "PxgConvexConvexShape.h"
#include "PxgPersistentContactManifold.h"
#include "PxgSoftBodyCore.h"
#include "PxgSoftBody.h"
#include "PxgFEMCloth.h"

#include "PxsMaterialCore.h"

#include "PxContact.h"
#include "geometry/PxConvexCoreGeometry.h"

#include "contactPatchUtils.cuh"
#include "triangleMesh.cuh"

#include "GuConvexSupport.h"
#include "GuRefGjkEpa.h"

#include "bv32Traversal.cuh"
#include "nputils.cuh"
#include "shuffle.cuh"
#include "utils.cuh"
#include "deformableElementFilter.cuh"

using namespace physx;
using namespace Gu;

extern "C" __host__ void initNarrowphaseKernels24() {}

struct TestInput
{
	PxU32 cacheRef0, cacheRef1;
	PxgShape shape0, shape1;
	PxTransform transform0, transform1;
	PxReal contactDist, restDist;
	bool swapShapes;

	__device__ TestInput(
		PxU32 testIndex,
		const PxgContactManagerInput* PX_RESTRICT cmInputs,
		const PxgShape* PX_RESTRICT shapes,
		const PxsCachedTransform* PX_RESTRICT transformCache,
		const PxReal* contactDistance,
		const PxReal* restDistance)
	{
		const PxgContactManagerInput& cmInput = cmInputs[testIndex];
		shape0 = shapes[cmInput.shapeRef0];
		shape1 = shapes[cmInput.shapeRef1];
		const PxU32 srcCacheRef0 = cacheRef0 = cmInput.transformCacheRef0;
		const PxsCachedTransform& transformCache0 = transformCache[srcCacheRef0];
		const PxU32 srcCacheRef1 = cacheRef1 = cmInput.transformCacheRef1;
		const PxsCachedTransform& transformCache1 = transformCache[srcCacheRef1];
		transform0 = transformCache0.transform;
		assert(transform0.isSane());
		transform1 = transformCache1.transform;
		assert(transform1.isSane());
		contactDist = contactDistance[srcCacheRef0] + contactDistance[srcCacheRef1];
		restDist = restDistance ? restDistance[testIndex] : 0;
		swapShapes = false;
	}

	__device__ void checkTypes(PxGeometryType::Enum type0, PxGeometryType::Enum type1)
	{
		if (type0 == shape1.type && type1 == shape0.type)
		{
			PxSwap(shape0, shape1);
			PxSwap(cacheRef0, cacheRef1);
			PxSwap(transform0, transform1);
			swapShapes = true;
		}
		assert(type0 == shape0.type && type1 == shape1.type);
	}
};

namespace physx
{
namespace Gu
{
	__device__ bool makeConvexShape(const PxgShape& shape, const PxTransform& pose, ConvexShape& convex)
	{
		convex.coreType = Gu::ConvexCore::Type::Enum(-1);

		switch (shape.type)
		{
			case PxGeometryType::eCONVEXCORE:
			{
				convex.coreType = Gu::ConvexCore::Type::Enum(shape.hullOrMeshPtr);
				memcpy(convex.coreData, &shape.scale.scale.x, PxConvexCoreGeometry::MAX_CORE_SIZE);
				convex.margin = shape.scale.rotation.w;
				convex.pose = pose;
				return true;
			}
			case PxGeometryType::eSPHERE:
			{
				convex.coreType = Gu::ConvexCore::Type::ePOINT;
				convex.margin = shape.scale.scale.x;
				convex.pose = pose;
				return true;
			}
			case PxGeometryType::eCAPSULE:
			{
				convex.coreType = Gu::ConvexCore::Type::eSEGMENT;
				Gu::ConvexCore::SegmentCore& core = *reinterpret_cast<Gu::ConvexCore::SegmentCore*>(convex.coreData);
				core.length = shape.scale.scale.x * 2.0f;
				convex.margin = shape.scale.scale.y;
				convex.pose = pose;
				return true;
			}
			case PxGeometryType::eBOX:
			{
				convex.coreType = Gu::ConvexCore::Type::eBOX;
				Gu::ConvexCore::BoxCore& core = *reinterpret_cast<Gu::ConvexCore::BoxCore*>(convex.coreData);
				core.extents = shape.scale.scale * 2.0f;
				convex.margin = 0;
				convex.pose = pose;
				return true;
			}
			case PxGeometryType::eCONVEXMESH:
			{
				convex.coreType = Gu::ConvexCore::Type::ePOINTS;
				Gu::ConvexCore::PointsCore core;
				const float4* ptr = reinterpret_cast<const float4*>(shape.hullOrMeshPtr);
				core.points =  ptr + 3;
				core.numPoints = (reinterpret_cast<const int4*>(ptr + 1)->x >> 8) & 0xff;
				core.stride = PxU8(sizeof(float4));
				core.S = shape.scale.scale;
				core.R = shape.scale.rotation;
				convex.margin = 0;
				convex.pose = pose;
				memcpy(convex.coreData, &core, sizeof(core));
				return true;
			}
		}
		return false;
	}
}
}

struct TestOutput
{
	__device__ static void writeOutput(
		PxU32 testIndex, const PxContact* contacts, PxU32 nbContacts,
		const PxgShape& shape0, const PxgShape& shape1, const PxVec3& normal,
		const PxgContactManagerInput* PX_RESTRICT cmInputs,
		PxsContactManagerOutput* PX_RESTRICT cmOutputs,
		const PxsMaterialData* PX_RESTRICT materials,
		PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
		PxU8* PX_RESTRICT contactStream,
		PxU8* PX_RESTRICT patchStream,
		PxU32* PX_RESTRICT touchChangeFlags,
		PxU32* PX_RESTRICT patchChangeFlags,
		PxU8* PX_RESTRICT startContactPatches,
		PxU8* PX_RESTRICT startContactPoints,
		PxU8* PX_RESTRICT startContactForces,
		PxU32 patchBytesLimit,
		PxU32 contactBytesLimit,
		PxU32 forceBytesLimit)
	{
		PxU32 contactByteOffset = 0xFFFFFFFF;
		PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

		if (nbContacts)
		{
			contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * nbContacts);
			forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * nbContacts);

			if ((contactByteOffset + sizeof(PxContact)) > contactBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
				contactByteOffset = 0xFFFFFFFF; //overflow
			}

			if ((forceAndIndiceByteOffset + sizeof(PxU32)) > forceBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
				forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
			}
		}

		PxsContactManagerOutput& output = cmOutputs[testIndex];

		if (contactByteOffset != 0xFFFFFFFF)
		{
			float4* baseContactStream = (float4*)(contactStream + contactByteOffset);

			for (PxU32 i = 0; i < nbContacts; ++i)
			{
				const PxContact& p = contacts[i];
				baseContactStream[i] = make_float4(p.contact.x, p.contact.y, p.contact.z, p.separation);
			}

			output.contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
			output.contactPoints = startContactPoints + contactByteOffset;
		}
		else
		{
			output.contactForces = NULL;
			output.contactPoints = NULL;
		}

		PxU32 allflags = reinterpret_cast<PxU32*>(&(output.allflagsStart))[0];

		PxU8 oldStatusFlags = u16Low(u32High(allflags));
		PxU8 statusFlags = oldStatusFlags;

		statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);

		if (nbContacts)
			statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
		else
			statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

		PxU8 prevPatches = u16High(u32Low(allflags)); //Get out the current number of patches to store as the previous frame's number of patches
		bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
		bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;

		PxU8 numPatches = (nbContacts > 0) ? 1 : 0;

		bool currentlyHasTouch = nbContacts > 0;

		const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
		touchChangeFlags[testIndex] = change;
		patchChangeFlags[testIndex] = (prevPatches != numPatches);

		reinterpret_cast<PxU32*>(&(output.allflagsStart))[0] = merge(merge(prevPatches, statusFlags), merge(numPatches, 0));
		cmOutputs[testIndex].nbContacts = nbContacts;

		PxU32 patchIndex = 0xFFFFFFFF;
		if (nbContacts)
		{
			patchIndex = atomicAdd(&(patchAndContactCounters->patchesBytes), sizeof(PxContactPatch));

			if ((patchIndex + sizeof(PxContactPatch)) > patchBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
				patchIndex = 0xFFFFFFFF; //overflow

				statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
				statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

				reinterpret_cast<PxU32*>(&(output.allflagsStart))[0] = merge(merge(prevPatches, statusFlags), 0);
				cmOutputs[testIndex].nbContacts = 0;

				touchChangeFlags[testIndex] = previouslyHadTouch;
				patchChangeFlags[testIndex] = prevPatches != 0;
			}
			else
			{
				output.contactPatches = startContactPatches + patchIndex;
			}
		}

		insertIntoPatchStream(materials, patchStream, shape0, shape1, patchIndex, normal, nbContacts);
	}

	__device__ static void writeOutput(
		PxU32 testIndex, const Gu::Contact& contact,
		const PxgShape& shape0, const PxgShape& shape1,
		const PxgContactManagerInput* PX_RESTRICT cmInputs,
		PxsContactManagerOutput* PX_RESTRICT cmOutputs,
		const PxsMaterialData* PX_RESTRICT materials,
		PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
		PxU8* PX_RESTRICT contactStream,
		PxU8* PX_RESTRICT patchStream,
		PxU32* PX_RESTRICT touchChangeFlags,
		PxU32* PX_RESTRICT patchChangeFlags,
		PxU8* PX_RESTRICT startContactPatches,
		PxU8* PX_RESTRICT startContactPoints,
		PxU8* PX_RESTRICT startContactForces,
		PxU32 patchBytesLimit,
		PxU32 contactBytesLimit,
		PxU32 forceBytesLimit)
	{
		PxU32 contactByteOffset = 0xFFFFFFFF;
		PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

		PxU8 numPatches = PxU8(contact.numPatches());
		PxU16 numPoints = PxU16(contact.numPoints());

		if (numPoints)
		{
			contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * numPoints);
			forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * numPoints);

			if ((contactByteOffset + sizeof(PxContact)) > contactBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
				contactByteOffset = 0xFFFFFFFF; //overflow
			}

			if ((forceAndIndiceByteOffset + sizeof(PxU32)) > forceBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
				forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
			}
		}

		PxsContactManagerOutput& output = cmOutputs[testIndex];

		if (contactByteOffset != 0xFFFFFFFF)
		{
			float4* baseContactStream = (float4*)(contactStream + contactByteOffset);

			for (PxU32 i = 0; i < contact.numPatches(); ++i)
			{
				for (PxU32 j = 0; j < contact.numPatchPoints(i); ++j)
				{
					const Gu::Contact::Point& p = contact.patchPoint(i, j);
					*(baseContactStream++) = make_float4(p.p.x, p.p.y, p.p.z, p.d);
				}
			}

			output.contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
			output.contactPoints = startContactPoints + contactByteOffset;
		}
		else
		{
			output.contactForces = NULL;
			output.contactPoints = NULL;
		}

		PxU32 allflags = reinterpret_cast<PxU32*>(&(output.allflagsStart))[0];

		PxU8 oldStatusFlags = u16Low(u32High(allflags));
		PxU8 statusFlags = oldStatusFlags;

		statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);

		if (numPoints)
			statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
		else
			statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

		PxU8 prevPatches = u16High(u32Low(allflags)); //Get out the current number of patches to store as the previous frame's number of patches
		bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
		bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;

		bool currentlyHasTouch = numPatches > 0;

		const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
		touchChangeFlags[testIndex] = change;
		patchChangeFlags[testIndex] = (prevPatches != numPatches);

		reinterpret_cast<PxU32*>(&(output.allflagsStart))[0] = merge(merge(prevPatches, statusFlags), merge(numPatches, 0));
		output.nbContacts = numPoints;

		PxU32 patchIndex = 0xFFFFFFFF;
		if (numPatches)
		{
			patchIndex = atomicAdd(&(patchAndContactCounters->patchesBytes), numPatches * sizeof(PxContactPatch));

			if ((patchIndex + numPatches * sizeof(PxContactPatch)) > patchBytesLimit)
			{
				patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
				patchIndex = 0xFFFFFFFF; //overflow

				statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
				statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

				reinterpret_cast<PxU32*>(&(output.allflagsStart))[0] = merge(merge(prevPatches, statusFlags), 0);
				output.nbContacts = 0;

				touchChangeFlags[testIndex] = previouslyHadTouch;
				patchChangeFlags[testIndex] = prevPatches != 0;
			}
			else
			{
				output.contactPatches = startContactPatches + patchIndex;
			}
		}

		if (patchIndex != 0xFFFFFFFF)
		{
			PxContactPatch* patches = ((PxContactPatch*)(patchStream + patchIndex));

			PxReal restitution, dynamicFriction, staticFriction, damping;
			PxU32 materialFlags;

			PxU16 materialIndex0 = shape0.materialIndex;
			PxU16 materialIndex1 = shape1.materialIndex;

			combineMaterials(materials, materialIndex0, materialIndex1, materialFlags,
				staticFriction, dynamicFriction, restitution, damping);

			PxU32 startContactIndex = 0;

			for (PxU32 i = 0; i < numPatches; ++i)
			{
				PxContactPatch& patch = patches[i];
				patch.normal = contact.patchNormal(i);
				patch.nbContacts = contact.numPatchPoints(i);
				patch.startContactIndex = startContactIndex;
				patch.staticFriction = staticFriction;
				patch.dynamicFriction = dynamicFriction;
				patch.damping = damping;
				patch.restitution = restitution;
				patch.materialIndex0 = materialIndex0;
				patch.materialIndex1 = materialIndex1;
				assert(materialFlags <= PX_MAX_U8);
				patch.materialFlags = PxU8(materialFlags);
				patch.internalFlags = 0;
				patch.mMassModification.linear0 = 1.0f;
				patch.mMassModification.linear1 = 1.0f;
				patch.mMassModification.angular0 = 1.0f;
				patch.mMassModification.angular1 = 1.0f;
				startContactIndex += contact.numPatchPoints(i);
			}
		}
	}
};

extern "C" __global__ __launch_bounds__(1024)
void convexCorePlaneNphase_Kernel(
	PxU32 numTests,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	const PxgShape* PX_RESTRICT shapes,
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
	const PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	const PxU32 testIndex = threadIndex / 1; // 1 tread per test

	if (testIndex >= numTests)
		return;

	TestInput in(testIndex, cmInputs, shapes, transformCache, contactDistance, NULL);
	in.checkTypes(PxGeometryType::ePLANE, PxGeometryType::eCONVEXCORE);

	PxPlane plane0(in.transform0.p, in.transform0.q.getBasisVector0());
	Gu::ConvexShape convex1; Gu::makeConvexShape(in.shape1, in.transform1, convex1);
	assert(convex1.isValid());

	PxVec3 normal;
	PxVec3 points[Gu::MAX_CONVEX_CONTACTS];
	PxReal dists[Gu::MAX_CONVEX_CONTACTS];
	PxU32 numContacts = Gu::generateContacts(plane0, convex1, in.contactDist, normal, points, dists);
	PxContact contacts[Gu::MAX_CONVEX_CONTACTS];
	for (PxU32 i = 0; i < numContacts; ++i)
		contacts[i] = PxContact({ points[i], dists[i] });

	TestOutput::writeOutput(testIndex, contacts, numContacts, in.shape0, in.shape1, -normal,
		cmInputs, cmOutputs, materials, patchAndContactCounters, contactStream, patchStream,
		touchChangeFlags, patchChangeFlags, startContactPatches, startContactPoints,
		startContactForces, patchBytesLimit, contactBytesLimit, forceBytesLimit);
}

extern "C" __global__ __launch_bounds__(512)
void convexCoreConvexNphase_Kernel(
	PxU32 numTests,
	const PxgContactManagerInput* PX_RESTRICT cmInputs,
	PxsContactManagerOutput* PX_RESTRICT cmOutputs,
	const PxgShape* PX_RESTRICT shapes,
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
	const PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	const PxU32 testIndex = threadIndex / 1; // 1 tread per test

	if (testIndex >= numTests)
		return;

	TestInput in(testIndex, cmInputs, shapes, transformCache, contactDistance, NULL);

	const PxVec3 shift = (in.transform0.p + in.transform1.p) * 0.5f;
	const PxTransform pose0(in.transform0.p - shift, in.transform0.q);
	const PxTransform pose1(in.transform1.p - shift, in.transform1.q);

	Gu::ConvexShape convex0, convex1;
	Gu::makeConvexShape(in.shape0, pose0, convex0);
	Gu::makeConvexShape(in.shape1, pose1, convex1);
	assert(convex0.isValid() && convex1.isValid());

	PxVec3 normal;
	PxVec3 points[Gu::MAX_CONVEX_CONTACTS];
	PxReal dists[Gu::MAX_CONVEX_CONTACTS];
	PxU32 numContacts = Gu::generateContacts(convex0, convex1, in.contactDist, normal, points, dists);
	PxContact contacts[Gu::MAX_CONVEX_CONTACTS];
	for (PxU32 i = 0; i < numContacts; ++i)
		contacts[i] = PxContact({ points[i] + shift, dists[i] });

	TestOutput::writeOutput(testIndex, contacts, numContacts, in.shape0, in.shape1, normal,
		cmInputs, cmOutputs, materials, patchAndContactCounters, contactStream, patchStream,
		touchChangeFlags, patchChangeFlags, startContactPatches, startContactPoints,
		startContactForces, patchBytesLimit, contactBytesLimit, forceBytesLimit);
}

template <typename _Bvh, int BLOCK_WARPS>
struct TestBvh
{
	_Bvh bvh;
	static const PxU32 NODE_STACK_MAX = 320;
	PxU32 nodeStack[NODE_STACK_MAX];

	__device__ TestBvh(const _Bvh& _bvh)
		:
		bvh(_bvh)
	{}

	template <typename ConvexTest>
	__device__ void queryBV32(const PxBounds3& bounds, const ConvexTest& test)
	{
		struct _Callback
		{
			const _Bvh& bvh;
			const PxBounds3& bounds;
			const PxReal margin;
			const ConvexTest& test;

			PX_FORCE_INLINE __device__ _Callback(const _Bvh& _bvh, const PxBounds3& _bounds, const ConvexTest& _test)
				:
				bvh(_bvh), bounds(_bounds), margin(_bounds.getDimensions().minElement() * 0.0001f), test(_test)
			{}

			PX_FORCE_INLINE __device__ bool intersectBoxFullWarp(bool hasBox, const PxVec3& min, const PxVec3& max) const
			{
				if (hasBox)
					return bounds.intersects(PxBounds3(min, max));

				return false;
			}

			PX_FORCE_INLINE __device__ void intersectPrimitiveFullWarp(PxU32 primIndex, PxU32 /*idxInWarp*/) const
			{
				Gu::ConvexShape prim;
				float4 verts[_Bvh::PRIM_VERT_COUNT];
				if (primIndex != 0xffffffff)
					bvh.createConvex(primIndex, margin, verts, prim);
				test(prim, primIndex);
			}
		}
		callback(bvh, bvh.scaleBounds(bounds), test);
		bv32TreeTraversal<_Callback, BLOCK_WARPS>(bvh.getNodes(), (int*)nodeStack, callback);
	}
};

struct TrimeshBvh
{
	const float4* trimeshVerts;
	const uint4* trimeshTriIndices;
	const Gu::BV32DataPacked* bv32PackedNodes;
	PxMeshScale scale;

	__device__ TrimeshBvh(const PxgShape& shape)
	{
		const PxU8* trimeshGeomPtr = reinterpret_cast<const PxU8*>(shape.hullOrMeshPtr);
		readTriangleMesh(trimeshGeomPtr, bv32PackedNodes, trimeshVerts, trimeshTriIndices);
		scale = shape.scale;
	}

	__device__ TrimeshBvh(const PxgFEMCloth& femCloth)
	{
		PxU8* trimeshGeomPtr = reinterpret_cast<PxU8*>(femCloth.mTriMeshData);
		trimeshGeomPtr += sizeof(uint4); // skip nbVerts_nbTets_maxDepth_nbBv32TreeNodes
		trimeshVerts = femCloth.mPosition_InvMass;
		trimeshTriIndices = femCloth.mTriangleVertexIndices;
		bv32PackedNodes = reinterpret_cast<const Gu::BV32DataPacked*>(trimeshGeomPtr);
	}

	__device__ PxBounds3 scaleBounds(const PxBounds3& bounds) const
	{
		return PxBounds3::transformFast(scale.getInverse().toMat33(), bounds);
	}

	__device__ const Gu::BV32DataPacked* getNodes() const
	{
		return bv32PackedNodes;
	}

	static const PxU32 PRIM_VERT_COUNT = 3;

	__device__ void createConvex(PxU32 index, PxReal margin, float4* verts, Gu::ConvexShape& prim) const
	{
		uint4 inds = trimeshTriIndices[index];

		const PxMat33 xform = scale.toMat33();
		const PxVec3 v0 = xform.transform(PxLoad3(trimeshVerts[inds.x]));
		const PxVec3 v1 = xform.transform(PxLoad3(trimeshVerts[inds.y]));
		const PxVec3 v2 = xform.transform(PxLoad3(trimeshVerts[inds.z]));

		verts[0] = make_float4(v0.x, v0.y, v0.z, 1);
		verts[1] = make_float4(v1.x, v1.y, v1.z, 1);
		verts[2] = make_float4(v2.x, v2.y, v2.z, 1);

		prim.coreType = Gu::ConvexCore::Type::ePOINTS;
		prim.pose = PxTransform(PxIdentity);
		Gu::ConvexCore::PointsCore core;
		core.points = verts;
		core.numPoints = 3;
		core.stride = sizeof(float4);
		core.S = PxVec3(1);
		core.R = PxQuat(PxIdentity);
		memcpy(prim.coreData, &core, sizeof(core));
		prim.margin = margin;
	}

	__device__ PxVec3 getTriNormal(PxU32 index) const
	{
		uint4 inds = trimeshTriIndices[index];
		const PxMat33 xform = scale.toMat33();
		const PxVec3 v0 = xform.transform(PxLoad3(trimeshVerts[inds.x]));
		const PxVec3 v1 = xform.transform(PxLoad3(trimeshVerts[inds.y]));
		const PxVec3 v2 = xform.transform(PxLoad3(trimeshVerts[inds.z]));
		return (v1 - v0).cross(v2 - v0).getNormalized();
	}

	__device__ uint4 getTriIndices(PxU32 index) const
	{
		return trimeshTriIndices[index];
	}

	__device__ float4 getTriBarycentric(PxU32 index, const PxVec3& p) const
	{
		uint4 inds = trimeshTriIndices[index];
		const PxMat33 xform = scale.toMat33();
		const PxVec3 v0 = xform.transform(PxLoad3(trimeshVerts[inds.x]));
		const PxVec3 v1 = xform.transform(PxLoad3(trimeshVerts[inds.y]));
		const PxVec3 v2 = xform.transform(PxLoad3(trimeshVerts[inds.z]));
		PxVec4 bary; PxComputeBarycentric(v0, v1, v2, p, bary);
		return make_float4(bary.x, bary.y, bary.z, bary.w);
	}
};

namespace physx
{
	namespace Gu
	{
		struct Contact32 : Contact
		{
			struct NewPoint
			{
				Point point;
				PxVec3 normal;

				__device__ static NewPoint make(const PxVec3& p, PxReal d, const PxVec3& n)
					{ NewPoint np; np.point = Point::make(p, d); np.normal = n; return np; }
			};

			NewPoint newPoints[WARP_SIZE * MAX_CONVEX_CONTACTS];
			Point tmpBuffer[8];

			__device__ void addPoints(const PxVec3& normal, PxVec3 points[Gu::MAX_CONVEX_CONTACTS],
				PxReal dists[Gu::MAX_CONVEX_CONTACTS], PxU32 numPoints, const PxTransform& transform)
			{
				const PxU32 thread = threadIdx.x;

				PxU32 totalPoints = copyPoints(normal, points, dists, numPoints, transform);

				__syncwarp();

				const NewPoint dummy = NewPoint::make(PxVec3(0), FLT_MAX, PxVec3(0));
				const PxU32 numFullWarps = (totalPoints + WARP_SIZE - 1) / WARP_SIZE;
				for (PxU32 i = thread; i / WARP_SIZE < numFullWarps; i += WARP_SIZE)
				{
					PxU32 allMask = __ballot_sync(FULL_MASK, i < totalPoints);
					while (allMask)
					{
						// read up to 32 points from the newPoints buffer
						NewPoint p = allMask & (1 << thread) ? newPoints[i] : dummy;

						// find the deepest point
						PxReal minDist = p.point.d;
						PxU32 minDistThread = minIndex(minDist, allMask, minDist);
						PxVec3 minDistNormal = shuffle(FULL_MASK, p.normal, minDistThread);

						// select points with the same normal as the deepest one
						bool sameNormal = p.normal.dot(minDistNormal) > SAME_NORMAL;
						PxU32 sameNormalMask = allMask & __ballot_sync(FULL_MASK, sameNormal);

						// selest up to 4 best points forming the largest quad
						PxU32 deepest;
						PxU32 bestPointsMask = reducePoints(p.point.p, p.point.d, minDistNormal, sameNormalMask, deepest);
						PxU32 count = __popc(bestPointsMask);
						PxU32 offset = __popc(bestPointsMask & ((1 << thread) - 1));

						// write the result into the tmp buffer
						if (bestPointsMask & (1 << thread))
							tmpBuffer[offset] = p.point;

						// remove processed points
						allMask &= ~sameNormalMask;

						// find/create a patch with the fitting normal
						PxU32 patchIndex = findCreatePatch(minDistNormal);
						if (patchIndex == 0xffffffff)
							// we're told to drop the current points
							continue;

						// add existing patch points into the tmp buffer
						Patch& patch = mPatches[patchIndex];
						if (thread < patch.numPoints)
							tmpBuffer[count + thread] = mPoints[patchIndex * MAX_PATCH_POINTS + thread];

						count += patch.numPoints;

						__syncwarp();

						bool patchPoint = thread < count;
						PxU32 patchPointMask = __ballot_sync(FULL_MASK, patchPoint);

						// read all collected points from the tmp buffer
						Point pp = patchPoint ? tmpBuffer[thread] : Point();
						// select up to 4 best points forming the largest quad
						bestPointsMask = reducePoints(pp.p, pp.d, minDistNormal, patchPointMask, deepest);

						// write the best points back into the patch
						count = __popc(bestPointsMask);
						offset = __popc(bestPointsMask & ((1 << thread) - 1));
						if (bestPointsMask & (1 << thread))
							mPoints[patchIndex * MAX_PATCH_POINTS + offset] = pp;

						__syncwarp();

						if (thread == 0)
						{
							// move the deepest point to the beginnig
							offset = __popc(bestPointsMask & ((1 << deepest) - 1));
							PxSwap(mPoints[patchIndex * MAX_PATCH_POINTS], mPoints[patchIndex * MAX_PATCH_POINTS + offset]);

							// adjust point count
							mNumPoints -= patch.numPoints;
							patch.numPoints = count;
							mNumPoints += patch.numPoints;
						}
					}
				}
			}

			__device__ PxU32 copyPoints(const PxVec3& normal, PxVec3 points[Gu::MAX_CONVEX_CONTACTS],
				PxReal dists[Gu::MAX_CONVEX_CONTACTS], PxU32 numPoints, const PxTransform& transform)
			{
				const PxVec3 worldNormal = transform.rotate(normal);

				const PxU32 prefixSum = warpScan<AddOpPxU32, PxU32>(FULL_MASK, numPoints);

				const PxU32 newPointOffset = prefixSum - numPoints;
				for (PxU32 i = 0; i < numPoints; ++i)
					newPoints[newPointOffset + i] = NewPoint::make(transform.transform(points[i]), dists[i], worldNormal);

				PxU32 totalPoints = __shfl_sync(FULL_MASK, prefixSum, 31);

				return totalPoints;
			}

			__device__ PxU32 reducePoints(const PxVec3& p, PxReal d, const PxVec3& n, PxU32 mask, PxU32& deepest)
			{
				// p0 - the deepest point
				PxReal minDist = d;
				PxU32 pi0 = deepest = minIndex(minDist, mask, minDist);
				PxVec3 p0 = shuffle(FULL_MASK, p, pi0);

				// p1 - the farthest from p0
				PxReal maxDist = (p0 - p).magnitude();
				PxU32 pi1 = maxIndex(maxDist, mask, maxDist);
				PxVec3 p1 = shuffle(FULL_MASK, p, pi1);

				// p2 and p3 - 2 farthest (from + and - sides)
				// from the plane formed by n and p1 - p0
				PxReal sideDist = n.cross(p1 - p0).dot(p - p0);
				PxReal maxSideDist, minSideDist;
				PxU32 pi2 = minIndex(sideDist, mask, minSideDist);
				PxU32 pi3 = maxIndex(sideDist, mask, maxSideDist);

				// combine the indices into a mask. note that
				// it will eliminate duplicates, so it can be less
				// than 4 points. but at least 1
				return (1 << pi0) | (1 << pi1) | (1 << pi2) | (1 << pi3);
			}

			__device__ PxU32 findCreatePatch(const PxVec3& normal)
			{
				assert(MAX_PATCHES < WARP_SIZE);

				const PxU32 thread = threadIdx.x;

				const PxVec3 patchNormal = thread < mNumPatches ? mPatches[thread].normal : PxVec3(0);

				PxReal maxDot = normal.dot(patchNormal);
				PxU32 maxDotThread = maxIndex(maxDot, FULL_MASK, maxDot);

				if (maxDot > SAME_NORMAL)
					// fitting patch found
					return maxDotThread;

				if (mNumPatches < MAX_PATCHES)
				{
					// there's still free patches
					// let's take one
					if (thread == 0)
					{
						Patch& patch = mPatches[mNumPatches++];
						patch.numPoints = 0;
						patch.normal = normal;
					}

					__syncwarp();

					return mNumPatches - 1;
				}

				// there's no free patches
				// let's check if we can drop any.
				// we're dot'ing every patch's normal
				// with every other patche's normal
				// thus for every patch we find the other
				// closest patch
				PxReal patchMaxDot = -FLT_MAX;
				PxU32 closestPatchThread = 0xffffffff;
				for (PxU32 i = thread + 1; i < MAX_PATCHES; ++i)
				{
					const PxVec3 otherNormal = mPatches[i].normal;
					PxReal dot = patchNormal.dot(otherNormal);
					if (patchMaxDot < dot)
					{
						patchMaxDot = dot;
						closestPatchThread = i;
					}
				}

				__syncwarp();

				// here's the winner
				PxU32 patchMaxDotThread = maxIndex(patchMaxDot, FULL_MASK, patchMaxDot);

				if (patchMaxDot > maxDot)
				{
					// and this patch pair is closer
					// than our new patch to any other patch
					// so we drop one of these 2. the one with
					// bigger dist (less important)
					closestPatchThread = __shfl_sync(FULL_MASK, closestPatchThread, patchMaxDotThread);
					PxReal dist0 = mPoints[patchMaxDotThread * MAX_PATCH_POINTS].d;
					PxReal dist1 = mPoints[closestPatchThread * MAX_PATCH_POINTS].d;
					PxU32 patchIndex = dist0 > dist1 ? patchMaxDotThread : closestPatchThread;

					if (thread == 0)
					{
						Patch& patch = mPatches[patchIndex];
						mNumPoints -= patch.numPoints;
						patch.numPoints = 0;
						patch.normal = normal;
					}

					__syncwarp();

					return patchIndex;
				}

				// no. all existing patches are more
				// important than the new one. so we
				// drop the new one
				return 0xffffffff;
			}
		};
	}
}

extern "C" __global__ __launch_bounds__(128)
void convexCoreTrimeshNphase_Kernel32(
	PxU32 numTests,
	const PxgContactManagerInput * PX_RESTRICT cmInputs,
	PxsContactManagerOutput * PX_RESTRICT cmOutputs,
	const PxgShape * PX_RESTRICT shapes,
	const PxsCachedTransform * PX_RESTRICT transformCache,
	const PxReal * PX_RESTRICT contactDistance,
	const PxsMaterialData * PX_RESTRICT materials,
	PxU8 * PX_RESTRICT contactStream,
	PxU8 * PX_RESTRICT patchStream,
	PxgPatchAndContactCounters * PX_RESTRICT patchAndContactCounters,
	PxU32 * PX_RESTRICT touchChangeFlags,
	PxU32 * PX_RESTRICT patchChangeFlags,
	PxU8 * PX_RESTRICT startContactPatches,
	PxU8 * PX_RESTRICT startContactPoints,
	PxU8 * PX_RESTRICT startContactForces,
	PxU32 patchBytesLimit,
	PxU32 contactBytesLimit,
	PxU32 forceBytesLimit)
{
	assert(blockDim.x == WARP_SIZE);
	static const PxU32 BLOCK_WARP_COUNT = 4;
	assert(blockDim.y == BLOCK_WARP_COUNT);
	const PxU32 warpThreadIndex = threadIdx.x;
	const PxU32 blockWarpIndex = threadIdx.y;

	const PxU32 testIndex = threadIdx.y + blockIdx.x * blockDim.y; // A warp (32 treads) per test

	using TestTrimesh = TestBvh<TrimeshBvh, BLOCK_WARP_COUNT>;

	if (testIndex >= numTests)
		return;

	struct _Shared
	{
		TestInput in;
		Gu::ConvexShape convex;
		PxTransform transform0in1;
		PxBounds3 convexBounds;
		TestTrimesh trimesh;
		Gu::Contact32 contact;
		PxU32 contactLock;

		__device__ static _Shared& cast(PxU8* ptr)
			{ return *reinterpret_cast<_Shared*>(ptr); }
	};

	__shared__ PxU8 blockShared[BLOCK_WARP_COUNT][sizeof(_Shared)];
	_Shared& _sh = _Shared::cast(blockShared[blockWarpIndex]);

	TestInput& in = _sh.in;
	PxTransform& transform0in1 = _sh.transform0in1;
	Gu::ConvexShape& convex = _sh.convex;
	PxBounds3& convexBounds = _sh.convexBounds;
	TestTrimesh& trimesh = _sh.trimesh;
	Gu::Contact32& contact = _sh.contact;

	if (warpThreadIndex == 0)
	{
		in = TestInput(testIndex, cmInputs, shapes, transformCache, contactDistance, NULL);
		in.checkTypes(PxGeometryType::eCONVEXCORE, PxGeometryType::eTRIANGLEMESH);
		transform0in1 = in.transform1.transformInv(in.transform0);
		Gu::makeConvexShape(in.shape0, transform0in1, convex);
		convexBounds = convex.computeBounds();
		convexBounds.fattenFast(in.contactDist);
		trimesh = TestTrimesh(TrimeshBvh(in.shape1));
		contact = Gu::Contact32();
		_sh.contactLock = 0;
	}

	__syncwarp();

	struct Callback
	{
		_Shared& sh;
		__device__ Callback(_Shared& _sh) : sh(_sh) {}
		__device__ void operator()(const Gu::ConvexShape& tri, PxU32 triIndex) const
		{
			PxVec3 normal;
			PxVec3 points[Gu::MAX_CONVEX_CONTACTS];
			PxReal dists[Gu::MAX_CONVEX_CONTACTS];
			PxU32 numPoints = 0;

			if (triIndex != 0xffffffff)
			{
				const PxVec3 triNormal = sh.trimesh.bvh.getTriNormal(triIndex);
				numPoints = Gu::generateContacts(sh.convex, tri, sh.in.contactDist, triNormal, normal, points, dists);
			}
#if 1
			// this version uses full warp of 32 threads to generate contact patches
			sh.contact.addPoints(normal, points, dists, numPoints, sh.in.transform1);
#else
			// in this version every thread waits on a spinlock for its turn to add
			// its contact points to one of contact patches. the version is slow and
			// will be removed after the version above proves reliable.
			if (numPoints)
			{
				const PxVec3 worldNormal = sh.in.transform1.rotate(normal);
				for (PxU32 i = 0; i < numPoints; ++i)
				{
					const PxVec3 worldPoint = sh.in.transform1.transform(points[i]);
					while (atomicCAS(&sh.contactLock, 0, 1) != 0);
					sh.contact.addPoint(worldPoint, worldNormal, dists[i]);
					atomicExch(&sh.contactLock, 0);
				}
			}
#endif
		}
	}
	callback(_sh);
	trimesh.queryBV32(convexBounds, callback);

	__syncwarp();

	if (warpThreadIndex == 0)
	{
		TestOutput::writeOutput(testIndex, contact, in.shape0, in.shape1,
			cmInputs, cmOutputs, materials, patchAndContactCounters, contactStream, patchStream,
			touchChangeFlags, patchChangeFlags, startContactPatches, startContactPoints,
			startContactForces, patchBytesLimit, contactBytesLimit, forceBytesLimit);
	}
}

struct TetmeshBvh
{
	const float4* tetmeshVerts;
	const uint4* tetmeshTetIndices;
	const PxU8* tetmeshSurfaceHint;
	const Gu::BV32DataPacked* bv32PackedNodes;

	__device__ TetmeshBvh(const PxgShape& shape, const PxgSoftBody* softbodies)
	{
		const PxU32 softbodyId = shape.particleOrSoftbodyId;
		const PxgSoftBody& softbody = softbodies[softbodyId];
		const PxU8* tetmeshGeomPtr = reinterpret_cast<PxU8*>(softbody.mTetMeshData);
		tetmeshGeomPtr += sizeof(uint4);
		tetmeshVerts = softbody.mPosition_InvMass;
		tetmeshTetIndices = softbody.mTetIndices;
		tetmeshSurfaceHint = softbody.mTetMeshSurfaceHint;
		bv32PackedNodes = reinterpret_cast<const Gu::BV32DataPacked*>(tetmeshGeomPtr);
	}

	__device__ PxBounds3 scaleBounds(const PxBounds3& bounds) const
	{
		return bounds;
	}

	__device__ const Gu::BV32DataPacked* getNodes() const
	{
		return bv32PackedNodes;
	}

	static const PxU32 PRIM_VERT_COUNT = 4;

	__device__ void createConvex(PxU32 index, PxReal margin, float4* verts, Gu::ConvexShape& prim) const
	{
		uint4 inds = tetmeshTetIndices[index];

		verts[0] = tetmeshVerts[inds.x];
		verts[1] = tetmeshVerts[inds.y];
		verts[2] = tetmeshVerts[inds.z];
		verts[3] = tetmeshVerts[inds.w];

		prim.coreType = Gu::ConvexCore::Type::ePOINTS;
		prim.pose = PxTransform(PxIdentity);
		Gu::ConvexCore::PointsCore core;
		core.points = verts;
		core.numPoints = 4;
		core.stride = sizeof(float4);
		core.S = PxVec3(1);
		core.R = PxQuat(PxIdentity);
		memcpy(prim.coreData, &core, sizeof(core));
		prim.margin = margin;
	}
};

extern "C" __global__ void convexCoreTetmeshNphase_Kernel32(
	PxU32 numTests,
	const PxgContactManagerInput * PX_RESTRICT cmInputs,
	const PxgShape * PX_RESTRICT shapes,
	const PxgSoftBody * PX_RESTRICT softbodies,
	const PxNodeIndex * PX_RESTRICT shapeToRigidRemapTable,
	const PxsCachedTransform * PX_RESTRICT transformCache,
	const PxReal * PX_RESTRICT contactDistance,
	const PxReal * PX_RESTRICT restDistance,
	PxgFEMContactWriter writer
)
{
	assert(blockDim.x == WARP_SIZE);
	static const PxU32 BLOCK_WARP_COUNT = 4;
	assert(blockDim.y == BLOCK_WARP_COUNT);
	const PxU32 warpThreadIndex = threadIdx.x;
	const PxU32 blockWarpIndex = threadIdx.y;

	const PxU32 testIndex = threadIdx.y + blockIdx.x * blockDim.y; // A warp (32 treads) per test

	using TestTetmesh = TestBvh<TetmeshBvh, BLOCK_WARP_COUNT>;

	if (testIndex >= numTests)
		return;

	struct _Shared
	{
		TestInput in;
		Gu::ConvexShape convex;
		PxTransform transform0in1;
		PxBounds3 convexBounds;
		TestTetmesh tetmesh;
		PxNodeIndex rigidId;
		PxU32 softbodyId;
		PxgFEMContactWriter writer;

		__device__ static _Shared& cast(PxU8* ptr)
			{ return *reinterpret_cast<_Shared*>(ptr); }
	};

	__shared__ PxU8 blockShared[BLOCK_WARP_COUNT][sizeof(_Shared)];
	_Shared& _sh = _Shared::cast(blockShared[blockWarpIndex]);

	TestInput& in = _sh.in;
	PxTransform& transform0in1 = _sh.transform0in1;
	Gu::ConvexShape& convex = _sh.convex;
	PxBounds3& convexBounds = _sh.convexBounds;
	TestTetmesh& tetmesh = _sh.tetmesh;

	if (warpThreadIndex == 0)
	{
		in = TestInput(testIndex, cmInputs, shapes, transformCache, contactDistance, restDistance);
		in.checkTypes(PxGeometryType::eCONVEXCORE, PxGeometryType::eTETRAHEDRONMESH);
		transform0in1 = in.transform1.transformInv(in.transform0);
		Gu::makeConvexShape(in.shape0, transform0in1, convex);
		convexBounds = convex.computeBounds();
		convexBounds.fattenFast(in.contactDist);
		tetmesh = TestTetmesh(TetmeshBvh(in.shape1, softbodies));
		_sh.rigidId = shapeToRigidRemapTable[_sh.in.cacheRef0];
		_sh.softbodyId = _sh.in.shape1.particleOrSoftbodyId;
		_sh.writer = writer;
	}

	__syncwarp();

	struct Callback
	{
		_Shared& sh;
		__device__ Callback(_Shared& _sh) : sh(_sh) {}
		__device__ void operator()(const Gu::ConvexShape& tet, PxU32 tetIndex) const
		{
			PxVec3 normal;
			PxVec3 points[Gu::MAX_CONVEX_CONTACTS];
			PxReal dists[Gu::MAX_CONVEX_CONTACTS];
			PxU32 numPoints = 0;

			if (tetIndex != 0xffffffff)
				numPoints = Gu::generateContacts(sh.convex, tet, sh.in.contactDist, normal, points, dists);

			if (numPoints)
			{
				const PxU64 pairInd0 = sh.rigidId.getInd();
				const PxU32 pairInd1 = PxEncodeSoftBodyIndex(sh.softbodyId, tetIndex);
				const PxU32 index = atomicAdd(sh.writer.totalContactCount, numPoints);
				for (PxU32 i = 0; i < numPoints; ++i)
				{
					const PxVec3 point = points[i] + normal * dists[i] * 0.5f;
					if (dists[i] < 0) dists[i] *= 0.4f; // VR @@@ add fake ERP. it overshoots otherwise
					const float4 posRest = make_float4(point.x, point.y, point.z, sh.in.restDist);
					const float4 normDist = make_float4(-normal.x, -normal.y, -normal.z, dists[i]);
					sh.writer.writeRigidVsDeformableContactNoBarycentric(index + i, posRest, normDist, pairInd0,
						pairInd1, pairInd0, sh.in.shape0.materialIndex);
				}
			}
		}
	}
	callback(_sh);
	tetmesh.queryBV32(convexBounds, callback);
}

extern "C" __global__ __launch_bounds__(512)
void convexCoreClothmeshNphase_Kernel32(
	const PxU8 * PX_RESTRICT stackPtr,
	const PxU32 * PX_RESTRICT midphasePairsNum,
	const PxgContactManagerInput * PX_RESTRICT cmInputs,
	const PxgShape * PX_RESTRICT shapes,
	const PxgFEMCloth * PX_RESTRICT femClothes,
	const PxNodeIndex * PX_RESTRICT shapeToRigidRemapTable,
	const PxsCachedTransform * PX_RESTRICT transformCache,
	const PxReal * PX_RESTRICT contactDistance,
	const PxReal * PX_RESTRICT restDistance,
	const PxgRigidFilterPair * PX_RESTRICT filterPairs,
	const PxU32 nbFilterPairs,
	const PxU32 stackSizeBytes,
	PxU32 * PX_RESTRICT stackSizeNeededOnDevice,
	PxgFEMContactWriter writer
)
{
	const PxU32 threadIndex = blockIdx.x * blockDim.x + threadIdx.x;
	const PxU32 numThreads = gridDim.x * blockDim.x;
	const PxU32 stackSize = stackSizeBytes / sizeof(uint4);
	const PxU32 numMidphasePairsFound = *midphasePairsNum;
	const PxU32 numPairs = PxMin(numMidphasePairsFound, stackSize);
	const uint4* pairs = reinterpret_cast<const uint4*>(stackPtr);

	for (PxU32 pairIndex = threadIndex; pairIndex < numPairs; pairIndex += numThreads)
	{
		const uint4 testPair = pairs[pairIndex];
		const PxU32 cmIndex = testPair.x;
		const PxU32 triIndex = testPair.y;

		TestInput in(cmIndex, cmInputs, shapes, transformCache, contactDistance, restDistance);
		in.checkTypes(PxGeometryType::eCONVEXCORE, PxGeometryType::eTRIANGLEMESH);

		const PxNodeIndex rigidId = shapeToRigidRemapTable[in.cacheRef0];
		const PxU32 clothId = in.shape1.particleOrSoftbodyId;
		const TrimeshBvh bvh(femClothes[clothId]);

		const PxU64 rigidInd = rigidId.getInd();
		const uint4 vertInds = bvh.getTriIndices(triIndex);
		if (find(filterPairs, nbFilterPairs, rigidInd, PxEncodeClothIndex(clothId, vertInds.x)) &&
			find(filterPairs, nbFilterPairs, rigidInd, PxEncodeClothIndex(clothId, vertInds.y)) &&
			find(filterPairs, nbFilterPairs, rigidInd, PxEncodeClothIndex(clothId, vertInds.z)))
			continue;

		const PxTransform transform0in1 = in.transform1.transformInv(in.transform0);
		Gu::ConvexShape convex0; Gu::makeConvexShape(in.shape0, transform0in1, convex0);
		PxBounds3 bounds0 = convex0.computeBounds();
		bounds0.fattenFast(in.contactDist);

		// let's give triangle some margin proportional
		// to the convex shape's size - for stability
		const PxReal triMarginK = 0.001f;

		const PxReal margin = bounds0.getDimensions().maxElement() * triMarginK;
		float4 verts[TrimeshBvh::PRIM_VERT_COUNT];
		Gu::ConvexShape convex1; bvh.createConvex(triIndex, margin, verts, convex1);

		assert(convex0.isValid() && convex1.isValid());

		PxVec3 normal;
		PxVec3 points[Gu::MAX_CONVEX_CONTACTS];
		PxReal dists[Gu::MAX_CONVEX_CONTACTS];
		PxU32 numPoints = Gu::generateContacts(convex0, convex1, in.contactDist, normal, points, dists);

		if (numPoints)
		{
			const PxU64 pairInd0 = rigidInd;
			const PxU32 pairInd1 = PxEncodeClothIndex(clothId, triIndex);
			const PxU32 index = atomicAdd(writer.totalContactCount, numPoints);
			for (PxU32 i = 0; i < numPoints; ++i)
			{
				const PxVec3 point = points[i] + normal * dists[i] * 0.5f;
				const float4 posRest = make_float4(point.x, point.y, point.z, in.restDist);
				const float4 normDist = make_float4(-normal.x, -normal.y, -normal.z, dists[i]);
				const float4 barycentric = bvh.getTriBarycentric(triIndex, point);
				writer.writeRigidVsDeformableContact(index + i, posRest, normDist, barycentric, pairInd0,
					pairInd1, in.shape0.materialIndex, rigidId);
			}
		}
	}

	if (threadIndex == 0)
		atomicMax(stackSizeNeededOnDevice, numMidphasePairsFound * sizeof(uint4));
}
