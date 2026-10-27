// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geometry/PxTriangleMesh.h"
#include "geomutils/PxContactBuffer.h"
#include "foundation/PxVecMath.h"
#include "foundation/PxVecTransform.h"
#include "GuVecTriangle.h"
#include "GuContactMethodImpl.h"
#include "GuHeightField.h"
#include "GuPCMContactConvexCommon.h"
#include "GuSegment.h"
#include "GuInternal.h"
#include "GuPCMContactMeshCallback.h"

using namespace physx;
using namespace Gu;
using namespace aos;

struct PCMCapsuleVsHeightfieldContactGenerationCallback : PCMHeightfieldContactGenerationCallback<PCMCapsuleVsHeightfieldContactGenerationCallback>
{
	PCMCapsuleVsHeightfieldContactGenerationCallback& operator=(const PCMCapsuleVsHeightfieldContactGenerationCallback&);

	PCMCapsuleVsMeshContactGeneration	mGeneration;

	PCMCapsuleVsHeightfieldContactGenerationCallback(
		const CapsuleV& capsule,
		const FloatVArg contactDistance,
		const FloatVArg replaceBreakingThreshold,
		const PxTransformV& capsuleTransform, 
		const PxTransformV& heightfieldTransform,
		const PxTransform& heightfieldTransform1,
		MultiplePersistentContactManifold& multiManifold,
		PxContactBuffer& contactBuffer,
		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE>* deferredContacts,
		HeightFieldUtil& hfUtil 
	) :
		PCMHeightfieldContactGenerationCallback<PCMCapsuleVsHeightfieldContactGenerationCallback>(hfUtil, heightfieldTransform1),
		mGeneration(capsule, contactDistance, replaceBreakingThreshold, capsuleTransform, heightfieldTransform, multiManifold, 
			contactBuffer, deferredContacts)
	{
	}

	template<PxU32 CacheSize>
	void processTriangleCache(TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMCapsuleVsMeshContactGeneration>(cache);
	}
};

bool Gu::pcmContactCapsuleHeightField(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxCapsuleGeometry& shapeCapsule = checkedCast<PxCapsuleGeometry>(shape0);
	const PxHeightFieldGeometry& shapeHeight = checkedCast<PxHeightFieldGeometry>(shape1);

	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const FloatV capsuleRadius = FLoad(shapeCapsule.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);

	const PxTransformV capsuleTransform = loadTransformA(transform0);//capsule transform
	const PxTransformV heightfieldTransform = loadTransformA(transform1);//height feild

	const PxTransformV curTransform = heightfieldTransform.transformInv(capsuleTransform);
	
	const FloatV replaceBreakingThreshold = FMul(capsuleRadius, FLoad(0.001f));

	if(multiManifold.invalidate(curTransform, capsuleRadius, FLoad(0.02f)))
	{
		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform); 

		HeightFieldUtil hfUtil(shapeHeight);

		const PxVec3 tmp = getCapsuleHalfHeightVector(transform0, shapeCapsule);

		const PxReal inflatedRadius = shapeCapsule.radius + params.mContactDistance;

		const PxVec3 capsuleCenterInMesh = transform1.transformInv(transform0.p);
		const PxVec3 capsuleDirInMesh = transform1.rotateInv(tmp);
		const CapsuleV capsule(V3LoadU(capsuleCenterInMesh), V3LoadU(capsuleDirInMesh), capsuleRadius);

		PCMCapsuleVsHeightfieldContactGenerationCallback callback(
			capsule,
			contactDist,
			replaceBreakingThreshold,
			capsuleTransform,
			heightfieldTransform,
			transform1,
			multiManifold,
			contactBuffer,
			NULL,
			hfUtil
		);

		// PT: TODO: improve these bounds - see computeCapsuleBounds
		hfUtil.overlapAABBTriangles(transform0, transform1, getLocalCapsuleBounds(inflatedRadius, shapeCapsule.halfHeight), callback);

		callback.mGeneration.processContacts(GU_CAPSULE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PxMatTransformV aToB(curTransform);
		// We must be in local space to use the cache
		const FloatV projectBreakingThreshold = FMul(capsuleRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(capsuleRadius, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);
	}

	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, capsuleTransform, heightfieldTransform, capsuleRadius);
}
