// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_SWEEP_MESH_H
#define GU_SWEEP_MESH_H

#include "GuMidphaseInterface.h"
#include "GuVecConvexHull.h"

namespace physx
{

namespace Gu
{
	// PT: intermediate class containing shared bits of code & members
	struct SweepShapeMeshHitCallback : MeshHitCallback<PxGeomRaycastHit>
	{
							SweepShapeMeshHitCallback(CallbackMode::Enum mode, const PxHitFlags& hitFlags, bool flipNormal, float distCoef);

		const PxHitFlags	mHitFlags;
		bool				mStatus;			// Default is false, set to true if a valid hit is found. Stays true once true.
		bool				mInitialOverlap;	// Default is false, set to true if an initial overlap hit is found. Reset for each hit.
		bool				mFlipNormal;		// If negative scale is used we need to flip normal
		PxReal				mDistCoeff;			// dist coeff from unscaled to scaled distance

		void operator=(const SweepShapeMeshHitCallback&) {}
	};

	struct SweepCapsuleMeshHitCallback : SweepShapeMeshHitCallback
	{
		PxGeomSweepHit&		mSweepHit;
		const PxMat34&		mVertexToWorldSkew;		
		const PxReal		mTrueSweepDistance;		// max sweep distance that can be used
		PxReal				mBestAlignmentValue;	// best alignment value for triangle normal
		PxReal				mBestDist;				// best distance, not the same as sweepHit.distance, can be shorter by epsilon
		const Capsule&		mCapsule;
		const PxVec3&		mUnitDir;
		const bool			mMeshDoubleSided;	// PT: true if PxMeshGeometryFlag::eDOUBLE_SIDED
		const bool			mIsSphere;

		SweepCapsuleMeshHitCallback(PxGeomSweepHit& sweepHit, const PxMat34& worldMatrix, PxReal distance, bool meshDoubleSided,
									const Capsule& capsule, const PxVec3& unitDir, const PxHitFlags& hitFlags, bool flipNormal, float distCoef);

		virtual PxAgain processHit(const PxGeomRaycastHit& aHit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal& shrunkMaxT, const PxU32*) PX_OVERRIDE;

		// PT: TODO: unify these operators
		void operator=(const SweepCapsuleMeshHitCallback&) {}

		bool finalizeHit(	PxGeomSweepHit& sweepHit, const Capsule& lss, const PxTriangleMeshGeometry& triMeshGeom,
							const PxTransform& pose, bool isDoubleSided) const;
	};

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

	struct SweepBoxMeshHitCallback : SweepShapeMeshHitCallback
	{
		const PxMat34Padded&	mMeshToBox;
		PxReal					mDist, mDist0;
		physx::aos::FloatV		mDistV;
		const Box&				mBox;
		const PxVec3&			mLocalDir;
		const PxVec3&			mWorldUnitDir;
		PxReal					mInflation;
		PxTriangle				mHitTriangle;
		physx::aos::Vec3V		mMinClosestA;
		physx::aos::Vec3V		mMinNormal;
		physx::aos::Vec3V		mLocalMotionV;
		PxU32					mMinTriangleIndex;
		PxVec3					mOneOverDir;
		const bool				mBothTriangleSidesCollide;	// PT: true if PxMeshGeometryFlag::eDOUBLE_SIDED || PxHitFlag::eMESH_BOTH_SIDES

		SweepBoxMeshHitCallback(CallbackMode::Enum mode_, const PxMat34Padded& meshToBox, PxReal distance, bool bothTriangleSidesCollide, 
								const Box& box, const PxVec3& localMotion, const PxVec3& localDir, const PxVec3& unitDir,
								const PxHitFlags& hitFlags, PxReal inflation, bool flipNormal, float distCoef);

		virtual ~SweepBoxMeshHitCallback() {}

		virtual PxAgain processHit(const PxGeomRaycastHit& meshHit, const PxVec3& lp0, const PxVec3& lp1, const PxVec3& lp2, PxReal& shrinkMaxT, const PxU32*) PX_OVERRIDE;

		bool	finalizeHit(	PxGeomSweepHit& sweepHit, const PxTriangleMeshGeometry& triMeshGeom, const PxTransform& pose,
								const PxTransform& boxTransform, const PxVec3& localDir,
								bool meshBothSides, bool isDoubleSided)	const;

	private:
		SweepBoxMeshHitCallback& operator=(const SweepBoxMeshHitCallback&);
	};

	struct SweepConvexMeshHitCallback : SweepShapeMeshHitCallback
	{
		PxTriangle							mHitTriangle;
		ConvexHullV							mConvexHull;
		physx::aos::PxMatTransformV			mMeshToConvex;
		physx::aos::PxTransformV			mConvexPoseV;
		const Cm::FastVertex2ShapeScaling&	mMeshScale;
		PxGeomSweepHit						mSweepHit; // stores either the closest or any hit depending on value of mAnyHit
		physx::aos::FloatV					mInitialDistance;
		physx::aos::Vec3V					mConvexSpaceDir; // convexPose.rotateInv(-unit*distance)
		PxVec3								mUnitDir;
		PxVec3								mMeshSpaceUnitDir;
		PxReal								mInflation;
		const bool							mAnyHit;
		const bool							mBothTriangleSidesCollide;	// PT: true if PxMeshGeometryFlag::eDOUBLE_SIDED || PxHitFlag::eMESH_BOTH_SIDES

		SweepConvexMeshHitCallback(	const ConvexHullData& hull, const PxMeshScale& convexScale, const Cm::FastVertex2ShapeScaling& meshScale,
									const PxTransform& convexPose, const PxTransform& meshPose,
									const PxVec3& unitDir, PxReal distance, PxHitFlags hitFlags, bool bothTriangleSidesCollide, PxReal inflation,
									bool anyHit, float distCoef);

		virtual ~SweepConvexMeshHitCallback()	{}

		virtual PxAgain processHit(const PxGeomRaycastHit& hit, const PxVec3& av0, const PxVec3& av1, const PxVec3& av2, PxReal& shrunkMaxT, const PxU32*) PX_OVERRIDE;

		bool	finalizeHit(PxGeomSweepHit& sweepHit, const PxTriangleMeshGeometry& meshGeom, const PxTransform& pose,
							const PxConvexMeshGeometry& convexGeom, const PxTransform& convexPose,
							const PxVec3& unitDir, PxReal inflation,
							bool isMtd, bool meshBothSides, bool isDoubleSided, bool bothTriangleSidesCollide);

	private:
		SweepConvexMeshHitCallback& operator=(const SweepConvexMeshHitCallback&);
	};

#if PX_VC 
     #pragma warning(pop) 
#endif

}
}

#endif
