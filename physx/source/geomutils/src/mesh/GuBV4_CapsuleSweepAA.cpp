// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "GuBV4.h"
#include "GuSweepSphereTriangle.h"
using namespace physx;
using namespace Gu;

#include "foundation/PxVecMath.h"
using namespace aos;

#include "GuBV4_Common.h"
#include "GuInternal.h"

#define SWEEP_AABB_IMPL

	// PT: TODO: refactor structure (TA34704)
namespace
{
	struct RayParams
	{
		BV4_ALIGN16(PxVec3p	mCenterOrMinCoeff_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mExtentsOrMaxCoeff_PaddedAligned);
	#ifndef GU_BV4_USE_SLABS
		BV4_ALIGN16(PxVec3p	mData2_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mFDir_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mData_PaddedAligned);
		BV4_ALIGN16(PxVec3p	mLocalDir_PaddedAligned);
	#endif
		BV4_ALIGN16(PxVec3p	mOrigin_Padded);		// PT: TODO: this one could be switched to PaddedAligned & V4LoadA (TA34704)
	};
}
#include "GuBV4_BoxSweep_Params.h"

namespace
{
	struct CapsuleSweepParams : BoxSweepParams
	{
		Capsule	mLocalCapsule;
		PxVec3	mCapsuleCenter;
		PxVec3	mExtrusionDir;
		float	mBestAlignmentValue;
		float	mBestDistance;
		float	mMaxDist;

//		PX_FORCE_INLINE float	getReportDistance()	const	{ return mStabbedFace.mDistance; }
		PX_FORCE_INLINE float	getReportDistance()	const	{ return mBestDistance; }
	};
}

#include "GuBV4_CapsuleSweep_Internal.h"
#include "GuBV4_AABBAABBSweepTest.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs.h"
#endif
#include "GuBV4_ProcessStreamOrdered_SegmentAABB_Inflated.h"
#include "GuBV4_ProcessStreamNoOrder_SegmentAABB_Inflated.h"
#ifdef GU_BV4_USE_SLABS
	#include "GuBV4_Slabs_KajiyaNoOrder.h"
	#include "GuBV4_Slabs_KajiyaOrdered.h"
#endif

#define GU_BV4_PROCESS_STREAM_RAY_NO_ORDER
#define GU_BV4_PROCESS_STREAM_RAY_ORDERED
#include "GuBV4_Internal.h"

PxIntBool BV4_CapsuleSweepSingleAA(const Capsule& capsule, const PxVec3& dir, float maxDist, const BV4Tree& tree, SweepHit* PX_RESTRICT hit, PxU32 flags)
{
	const SourceMesh* PX_RESTRICT mesh = static_cast<const SourceMesh*>(tree.mMeshInterface);

	CapsuleSweepParams Params;
	setupCapsuleParams(&Params, capsule, dir, maxDist, &tree, mesh, flags);

	if(tree.mNodes)
	{
		if(Params.mEarlyExit)
			processStreamRayNoOrder<1, LeafFunction_CapsuleSweepAny>(tree, &Params);
		else
			processStreamRayOrdered<1, LeafFunction_CapsuleSweepClosest>(tree, &Params);
	}
	else
		doBruteForceTests<LeafFunction_CapsuleSweepAny, LeafFunction_CapsuleSweepClosest>(mesh->getNbPrimitives(), &Params);

	return computeImpactDataT<ImpactFunctionCapsule>(capsule, dir, hit, &Params, NULL, (flags & QUERY_MODIFIER_DOUBLE_SIDED)!=0, (flags & QUERY_MODIFIER_MESH_BOTH_SIDES)!=0);
}

