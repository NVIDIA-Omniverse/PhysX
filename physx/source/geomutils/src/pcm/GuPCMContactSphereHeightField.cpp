// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "geometry/PxTriangleMesh.h"
#include "geomutils/PxContactBuffer.h"
#include "GuVecBox.h"
#include "GuVecConvexHull.h"
#include "GuVecConvexHullNoScale.h"
#include "GuVecTriangle.h"
#include "GuContactMethodImpl.h"
#include "GuHeightField.h"
#include "GuPCMContactConvexCommon.h"
#include "GuPCMContactMeshCallback.h"

using namespace physx;
using namespace Gu;
using namespace aos;

namespace
{
struct PCMSphereVsHeightfieldContactGenerationCallback : PCMHeightfieldContactGenerationCallback<PCMSphereVsHeightfieldContactGenerationCallback>
{
	PCMSphereVsMeshContactGeneration	mGeneration;

	PCMSphereVsHeightfieldContactGenerationCallback(
		const Vec3VArg sphereCenter,
		const FloatVArg sphereRadius,
		const FloatVArg contactDistance,
		const FloatVArg replaceBreakingThreshold,
		const PxTransformV& sphereTransform, 
		const PxTransformV& heightfieldTransform,
		const PxTransform& heightfieldTransform1,
		MultiplePersistentContactManifold& multiManifold,
		PxContactBuffer& contactBuffer,
		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE>* deferredContacts,
		HeightFieldUtil& hfUtil 
	) :
		PCMHeightfieldContactGenerationCallback<PCMSphereVsHeightfieldContactGenerationCallback>(hfUtil, heightfieldTransform1),
		mGeneration(sphereCenter, sphereRadius, contactDistance, replaceBreakingThreshold, sphereTransform,
			heightfieldTransform, multiManifold, contactBuffer, deferredContacts)
	{
	}

	template<PxU32 CacheSize>
	void processTriangleCache(TriangleCache<CacheSize>& cache)
	{
		mGeneration.processTriangleCache<CacheSize, PCMSphereVsMeshContactGeneration>(cache);
	}
};
}

bool Gu::pcmContactSphereHeightField(GU_CONTACT_METHOD_ARGS)
{
	PX_UNUSED(renderOutput);

	const PxSphereGeometry& shapeSphere = checkedCast<PxSphereGeometry>(shape0);
	const PxHeightFieldGeometry& shapeHeight = checkedCast<PxHeightFieldGeometry>(shape1);

	MultiplePersistentContactManifold& multiManifold = cache.getMultipleManifold();

	const QuatV q0 = QuatVLoadA(&transform0.q.x);
	const Vec3V p0 = V3LoadA(&transform0.p.x);

	const QuatV q1 = QuatVLoadA(&transform1.q.x);
	const Vec3V p1 = V3LoadA(&transform1.p.x);

	const FloatV sphereRadius = FLoad(shapeSphere.radius);
	const FloatV contactDist = FLoad(params.mContactDistance);
	
	const PxTransformV sphereTransform(p0, q0);//sphere transform
	const PxTransformV heightfieldTransform(p1, q1);//height feild
	const PxTransformV curTransform = heightfieldTransform.transformInv(sphereTransform);
	
	// We must be in local space to use the cache

	if(multiManifold.invalidate(curTransform, sphereRadius, FLoad(0.02f)))
	{
		multiManifold.mNumManifolds = 0;
		multiManifold.setRelativeTransform(curTransform);

		const FloatV replaceBreakingThreshold = FMul(sphereRadius, FLoad(0.001f));
		HeightFieldUtil hfUtil(shapeHeight);

		PxBounds3 localBounds;
		const PxVec3 localSphereCenter = getLocalSphereData(localBounds, transform0, transform1, shapeSphere.radius + params.mContactDistance);

		const Vec3V sphereCenter = V3LoadU(localSphereCenter);

		PxInlineArray<PxU32, LOCAL_PCM_CONTACTS_SIZE> delayedContacts;

		PCMSphereVsHeightfieldContactGenerationCallback blockCallback(
			sphereCenter,
			sphereRadius,
			contactDist,
			replaceBreakingThreshold,
			sphereTransform,
			heightfieldTransform,
			transform1,
			multiManifold,
			contactBuffer,
			&delayedContacts,
			hfUtil);

		hfUtil.overlapAABBTriangles(localBounds, blockCallback);

		blockCallback.mGeneration.generateLastContacts();
		blockCallback.mGeneration.processContacts(GU_SPHERE_MANIFOLD_CACHE_SIZE, false);
	}
	else
	{
		const PxMatTransformV aToB(curTransform);
		const FloatV projectBreakingThreshold = FMul(sphereRadius, FLoad(0.05f));
		const FloatV refereshDistance = FAdd(sphereRadius, contactDist);
		multiManifold.refreshManifold(aToB, projectBreakingThreshold, refereshDistance);
	}

	return multiManifold.addManifoldContactsToContactBuffer(contactBuffer, sphereTransform, heightfieldTransform, sphereRadius);
}
