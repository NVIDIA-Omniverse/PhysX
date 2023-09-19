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

#ifndef DY_HAIR_SYSTEM_CORE_H
#define DY_HAIR_SYSTEM_CORE_H

#include "PxAttachment.h"
#include "PxHairSystemFlag.h"
#include "PxNodeIndex.h"
#include "foundation/PxArray.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec2.h"
#include "foundation/PxVec4.h"

namespace physx
{

namespace Dy
{
// These parameters are needed on GPU for simulation and are grouped in a struct
// to reduce the number of assignments in update user data.
struct HairSystemSimParameters
{
	PX_CUDA_CALLABLE PX_FORCE_INLINE PxReal getCellSize() const { return mSegmentLength + 2.0f * mSegmentRadius; }
	PX_CUDA_CALLABLE PX_FORCE_INLINE int getGridSize() const { return mGridSize[0] * mGridSize[1] * mGridSize[2]; }
	PxHairSystemFlags::InternalType mFlags;
	PxReal mSegmentLength;
	PxReal mSegmentRadius;
	PxReal mInterHairRepulsion;       // strength of the repulsion field
	PxReal mInterHairVelocityDamping; // friction based on interpolated vel field
	PxReal mFrictionCoeff;            // coulomb friction coefficient for collisions (internal and external)
	PxReal mMaxDepenetrationVelocity; // max velocity delta coming out of collision responses
	PxReal mAeroDrag;                 // the aerodynamic drag coefficient
	PxReal mAeroLift;                 // the aerodynamic lift coefficient
	PxReal mBendingCompliance;
	PxReal mTwistingCompliance;
	int mGridSize[3];                 // number of cells in x,y,z directions
	PxVec2 mShapeCompliance;          // compliance for shape matching
	PxReal mSelfCollisionContactDist; // contact distance for self collisions expressed as a multiple of the segment
	                                  // radius
	PxReal mSelfCollisionRelaxation;
	PxReal mLraRelaxation;
	PxReal mShapeMatchingCompliance;
	PxReal mShapeMatchingBeta; // balance between rigid rotation and linear stretching
	PxU16 mShapeMatchingNumVertsPerGroup;
	PxU16 mShapeMatchingNumVertsOverlap;

	PxU32 mRestPositionTransformNumVertsPerStrand; // how many vertices of each strand to use for computing the rest
	                                               // position targets for global shape preservation

	HairSystemSimParameters()
	: mFlags(0)
	, mSegmentLength(0.1f)
	, mSegmentRadius(0.02f)
	, mInterHairRepulsion(0.0f)
	, mInterHairVelocityDamping(0.03f)
	, mFrictionCoeff(0.0f)
	, mMaxDepenetrationVelocity(PX_MAX_F32)
	, mAeroDrag(0.0f)
	, mAeroLift(0.0f)
	, mBendingCompliance(-1.0f)
	, mTwistingCompliance(-1.0f)
	, mShapeCompliance(-1.0f)
	, mSelfCollisionContactDist(1.5f)
	, mSelfCollisionRelaxation(0.7f)
	, mLraRelaxation(1.0f)
	, mShapeMatchingCompliance(-1.0f)
	, mShapeMatchingBeta(0.0f)
	, mShapeMatchingNumVertsPerGroup(10)
	, mShapeMatchingNumVertsOverlap(5)
	, mRestPositionTransformNumVertsPerStrand(2)
	{
		// grid size must be powers of two
		mGridSize[0] = 32;
		mGridSize[1] = 64;
		mGridSize[0] = 32;
	}
};

PX_ALIGN_PREFIX(16)
struct SoftbodyHairAttachment
{
	PxVec4 tetBarycentric; // must be aligned, is being loaded as float4
	PxU32 tetId;
	PxU32 softbodyNodeIdx;
	PxU32 hairVtxIdx;

	PxReal constraintOffset;
	PxVec4 low_high_angle;
	PxVec4 attachmentBarycentric;
}
PX_ALIGN_SUFFIX(16);

PX_COMPILE_TIME_ASSERT(sizeof(SoftbodyHairAttachment) % 16 == 0);

struct HairSystemCore
{
  public:
	PxU32 mDirtyFlags;
	PxHairSystemDataFlags mReadRequests;

	PxU32 mNumVertices;
	PxU32 mNumStrands;

	// Parameters
	HairSystemSimParameters mParams;
	PxU16 mSolverIterationCounts;
	PxVec4 mWind;

	// Topology data
	const PxU32* mStrandPastEndIndices;
	const PxReal* mBendingRestAngles;

	PxArray<PxU16> mMaterialhandles;

	// Buffers for simulation (device or pinned host mirrors)
	PxVec4* mPositionInvMass;
	PxVec4* mVelocity;

	// pointers to PxgHairSystemCore buffers that are exposed to the user
	PxU32* mStrandPastEndIndicesGpuSim;
	PxVec4* mPositionInvMassGpuSim;
	PxReal* mTwistingRestPositionsGpuSim;

	// rest positions
	PxVec4* mRestPositionsD; // Gpu buffer

	// Attachments to rigids
	PxParticleRigidAttachment* mRigidAttachments; // Gpu buffer
	PxU32 mNumRigidAttachments;

	// Attachments to softbodies
	SoftbodyHairAttachment* mSoftbodyAttachments;
	PxU32 mNumSoftbodyAttachments;

	// LOD data
	PxU32 mLodLevel;     // the selected level, zero by default meaning full detail
	PxU32 mLodNumLevels; // number of levels excluding level zero
	const PxReal* mLodProportionOfStrands;
	const PxReal* mLodProportionOfVertices;

	// sleeping
	PxReal				mSleepThreshold;
	PxReal				mWakeCounter;

	void setMaterial(PxU16 materialhandle) { mMaterialhandles.pushBack(materialhandle); }

	void clearMaterials() { mMaterialhandles.clear(); }

	HairSystemCore()
	: mDirtyFlags(PX_MAX_U32)
	, mNumVertices(0)
	, mNumStrands(0)
	, mSolverIterationCounts(8)
	, mWind(0.0f)
	, mStrandPastEndIndices(NULL)
	, mBendingRestAngles(NULL)
	, mPositionInvMass(NULL)
	, mVelocity(NULL)
	, mStrandPastEndIndicesGpuSim(NULL)
	, mPositionInvMassGpuSim(NULL)
	, mTwistingRestPositionsGpuSim(NULL)
	, mRestPositionsD(NULL)
	, mRigidAttachments(NULL)
	, mNumRigidAttachments(0)
	, mSoftbodyAttachments(NULL)
	, mNumSoftbodyAttachments(0)
	, mLodLevel(0)
	, mLodNumLevels(0)
	, mLodProportionOfStrands(NULL)
	, mLodProportionOfVertices(NULL)
	, mSleepThreshold(0.1f)
	, mWakeCounter(1.0f)
	{
	}
};
} // namespace Dy
} // namespace physx

#endif

