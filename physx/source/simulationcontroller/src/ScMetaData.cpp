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

#include "foundation/PxIO.h"
#include "ScActorCore.h"
#include "ScActorSim.h"
#include "ScBodyCore.h"
#include "ScStaticCore.h"
#include "ScConstraintCore.h"
#include "ScShapeCore.h"
#include "ScArticulationCore.h"
#include "ScArticulationJointCore.h"
#include "ScArticulationSensor.h"
#include "ScArticulationTendonCore.h"
#include "ScArticulationAttachmentCore.h"
#include "ScArticulationTendonJointCore.h"

using namespace physx;
using namespace Cm;
using namespace Sc;

///////////////////////////////////////////////////////////////////////////////

template <typename T> class PxMetaDataArray : public physx::PxArray<T>
{
public:
    static PX_FORCE_INLINE physx::PxU32 getDataOffset()		{ return PX_OFFSET_OF(PxMetaDataArray<T>, mData); }
    static PX_FORCE_INLINE physx::PxU32 getDataSize()		{ return PX_SIZE_OF(PxMetaDataArray<T>, mData); }
    static PX_FORCE_INLINE physx::PxU32 getSizeOffset()		{ return PX_OFFSET_OF(PxMetaDataArray<T>, mSize); }
    static PX_FORCE_INLINE physx::PxU32 getSizeSize()		{ return PX_SIZE_OF(PxMetaDataArray<T>, mSize); }
    static PX_FORCE_INLINE physx::PxU32 getCapacityOffset()	{ return PX_OFFSET_OF(PxMetaDataArray<T>, mCapacity); }
    static PX_FORCE_INLINE physx::PxU32 getCapacitySize()	{ return PX_SIZE_OF(PxMetaDataArray<T>, mCapacity); }
};

void Sc::ActorCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxActorFlags,     PxU8)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxDominanceGroup, PxU8)
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxClientID,       PxU8)

	PX_DEF_BIN_METADATA_CLASS(stream,	Sc::ActorCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, ActorSim,			mSim,						PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxU32,			    mAggregateIDOwnerClient,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxActorFlags,		mActorFlags,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxU8,				mActorType,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	Sc::ActorCore, PxU8,				mDominanceGroup,			0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PxsRigidCore(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxsRigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxTransform,		body2World,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxRigidBodyFlags,	mFlags,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxsBodyCore, PxU16,				solverIterationCounts,	0)
}

namespace
{
	class ShadowPxsBodyCore : public PxsBodyCore
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream)
		{
			PX_DEF_BIN_METADATA_CLASS(stream,		ShadowPxsBodyCore)
			PX_DEF_BIN_METADATA_BASE_CLASS(stream,	ShadowPxsBodyCore, PxsRigidCore)

			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxTransform,	body2Actor,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		ccdAdvanceCoefficient,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxVec3,		linearVelocity,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxPenBias,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxVec3,		angularVelocity,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		contactReportThreshold,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxAngularVelocitySq,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxLinearVelocitySq,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		linearDamping,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		angularDamping,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxVec3,		inverseInertia,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		inverseMass,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		maxContactImpulse,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		sleepThreshold,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		freezeThreshold,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		wakeCounter,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		solverWakeCounter,		0)
			PX_DEF_BIN_METADATA_ITEM(stream, 		ShadowPxsBodyCore, PxU32,		numCountedInteractions,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxReal,		offsetSlop,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU8,		isFastMoving,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU8,		disableGravity,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU8,		lockFlags,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,		ShadowPxsBodyCore, PxU8,		fixedBaseLink,			0)
		}
	};
}

static void getBinaryMetaData_PxsBodyCore(PxOutputStream& stream)
{
	getBinaryMetaData_PxsRigidCore(stream);

/*	PX_DEF_BIN_METADATA_CLASS(stream,		PxsBodyCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	PxsBodyCore, PxsRigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxTransform,	body2Actor,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		ccdAdvanceCoefficient,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxVec3,		linearVelocity,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxPenBias,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxVec3,		angularVelocity,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		contactReportThreshold,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxAngularVelocitySq,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxLinearVelocitySq,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		linearDamping,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		angularDamping,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxVec3,		inverseInertia,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		inverseMass,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		maxContactImpulse,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		sleepThreshold,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		freezeThreshold,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		wakeCounter,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		PxsBodyCore, PxReal,		solverWakeCounter,		0)
	PX_DEF_BIN_METADATA_ITEM(stream, 		PxsBodyCore, PxU32,			numCountedInteractions,	0)*/

	ShadowPxsBodyCore::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxsBodyCore, ShadowPxsBodyCore)
}

/*
We need to fix the header deps by moving the API out of PhysXCore and into its own dir where other code can get to it.
[25.08.2010 18:34:57] Dilip Sequeira: In the meantime, I think it's Ok to include PxSDK.h, but you're right, we need to be very careful about include deps in that direction.
[25.08.2010 18:38:15] Dilip Sequeira: On the memory thing... PxsBodyCore has 28 bytes of padding at the end, for no reason. In addition, it has two words of padding after the velocity fields, to facilitate SIMD loads. But in fact, Vec3FromVec4 is fast enough such that unless you were using it in an inner loop (which we never are with PxsBodyCore) that padding isn't worth it.
[25.08.2010 18:38:58] Dilip Sequeira: So, we should drop the end-padding, and move the damping values to replace the velocity padding. This probably requires a bit of fixup in the places where we do SIMD writes to the velocity.
[25.08.2010 18:39:18] Dilip Sequeira: Then we're down to 92 bytes of data, and 4 bytes of padding I think.
[25.08.2010 18:50:41] Dilip Sequeira: The reason we don't want to put the sleep data there explicitly is that it isn't LL data so I'd rather not have it in an LL interface struct.
[25.08.2010 19:04:53] Gordon Yeoman nvidia: simd loads are faster when they are 16-byte aligned.  I think the padding might be to ensure the second vector is also 16-byte aligned.  We could drop the second 4-byte pad but dropping the 1st 4-byte pad will likely  have performance implications.
[25.08.2010 19:06:22] Dilip Sequeira: We should still align the vec3s, as now - but we shouldn't use padding to do it, since there are a boatload of scalar data fields floating around in that struct too.
*/
void Sc::BodyCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_PxsBodyCore(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxRigidBodyFlags, PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,		Sc::BodyCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Sc::BodyCore, Sc::RigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		Sc::BodyCore, PxsBodyCore,	mCore,					0)
}

///////////////////////////////////////////////////////////////////////////////

void Sc::ConstraintCore::getBinaryMetaData(PxOutputStream& stream)
{   
	PX_DEF_BIN_METADATA_TYPEDEF(stream,		PxConstraintFlags, PxU16)

	PX_DEF_BIN_METADATA_CLASS(stream,		ConstraintCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxConstraintFlags,		mFlags,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxU8,					mIsDirty,				0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream,	ConstraintCore, PxU8,					mPadding,				PxMetaDataFlag::ePADDING)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxVec3,					mAppliedForce,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxVec3,					mAppliedTorque,			0)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxConstraintConnector,	mConnector,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxConstraintSolverPrep,	mSolverPrep,			PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxConstraintVisualize,	mVisualize,				PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxU32,					mDataSize,				0)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxReal,					mLinearBreakForce,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxReal,					mAngularBreakForce,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, PxReal,					mMinResponseThreshold,	0)		
	PX_DEF_BIN_METADATA_ITEM(stream,		ConstraintCore, ConstraintSim,			mSim,					PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

void Sc::RigidCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Sc::RigidCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Sc::RigidCore, Sc::ActorCore)
}

///////////////////////////////////////////////////////////////////////////////

void Sc::StaticCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,		Sc::StaticCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream,	Sc::StaticCore, Sc::RigidCore)

	PX_DEF_BIN_METADATA_ITEM(stream,		Sc::StaticCore, PxsRigidCore,		mCore,		0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_PxFilterData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	PxFilterData)

	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word0,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word1,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word2,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	PxFilterData, PxU32,		word3,		0)
}

namespace
{
	class ShadowPxsShapeCore : public PxsShapeCore
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream)
		{
			PX_DEF_BIN_METADATA_CLASS(stream,	ShadowPxsShapeCore)

			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore,	PxTransform,		mTransform,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore,	GeometryUnion,		mGeometry,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore,	PxReal,				mContactOffset,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore,	PxShapeFlags,		mShapeFlags,		0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore,	PxShapeCoreFlags,	mShapeCoreFlags,	0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore,	PxU16,				mMaterialIndex,		PxMetaDataFlag::eHANDLE)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore, PxReal,				mRestOffset,				0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore, PxReal,				mTorsionalRadius,			0)
			PX_DEF_BIN_METADATA_ITEM(stream,	ShadowPxsShapeCore, PxReal,				mMinTorsionalPatchRadius,	0)
		}
	};
}

static void getBinaryMetaData_PxsShapeCore(PxOutputStream& stream)
{
	ShadowPxsShapeCore::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,	PxsShapeCore, ShadowPxsShapeCore)
}

void Sc::ShapeCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_PxFilterData(stream);
	getBinaryMetaData_PxsShapeCore(stream);

	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxShapeFlags, PxU8)
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxShapeCoreFlags, PxU8)

	PX_DEF_BIN_METADATA_CLASS(stream,	ShapeCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxFilterData,	mSimulationFilterData,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, PxsShapeCore,	mCore,					0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ShapeCore, ShapeSim,		mExclusiveSim,			PxMetaDataFlag::ePTR)

	PX_DEF_BIN_METADATA_ITEM(stream,		ShapeCore, char,		mName,					PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_EXTRA_NAME(stream,	ShapeCore,				mName,					0)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_ArticulationCore(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationCore)

	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxArticulationFlags, PxU8)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxU16, solverIterationCounts, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxArticulationFlags, flags, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, sleepThreshold, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, freezeThreshold, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, wakeCounter, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, gpuRemapIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, maxLinearVelocity, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationCore, PxReal, maxAngularVelocity, 0)
}

void Sc::ArticulationCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_ArticulationCore(stream);

	PX_DEF_BIN_METADATA_CLASS(stream,	ArticulationCore)

	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationCore, ArticulationSim,		mSim,					PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationCore, Dy::ArticulationCore,	mCore,					0)
}

void Sc::ArticulationSensorCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, ArticulationSensorCore)

	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationSensorCore, ArticulationSensorSim, mSim, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationSensorCore, PxTransform, mRelativePose, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationSensorCore, PxU16, mFlags, 0)
}

void Sc::ArticulationAttachmentCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, ArticulationAttachmentCore)

	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxVec3, mRelativeOffset, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, ArticulationAttachmentCore, mParent, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxReal, mLowLimit, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxReal, mHighLimit, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxReal, mRestLength, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxReal, mCoefficient, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxU32, mLLLinkIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, PxU32, mAttachmentIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationAttachmentCore, ArticulationSpatialTendonSim, mTendonSim, PxMetaDataFlag::ePTR)
}

void Sc::ArticulationTendonCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, ArticulationTendonCore)

	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonCore, PxReal, mStiffness, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonCore, PxReal, mDamping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonCore, PxReal, mOffset, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonCore, PxReal, mLimitStiffness, 0)
}

void Sc::ArticulationSpatialTendonCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, ArticulationSpatialTendonCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, ArticulationSpatialTendonCore, ArticulationTendonCore)

	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationSpatialTendonCore, ArticulationSpatialTendonSim, mSim, PxMetaDataFlag::ePTR)
}

void Sc::ArticulationFixedTendonCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, ArticulationFixedTendonCore)
	PX_DEF_BIN_METADATA_BASE_CLASS(stream, ArticulationFixedTendonCore, ArticulationTendonCore)

	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationFixedTendonCore, PxReal, mLowLimit, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationFixedTendonCore, PxReal, mHighLimit, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationFixedTendonCore, PxReal, mRestLength, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationFixedTendonCore, ArticulationFixedTendonSim, mSim, PxMetaDataFlag::ePTR)
}

void Sc::ArticulationTendonJointCore::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxArticulationAxis, PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream, ArticulationTendonJointCore)

	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, PxArticulationAxis, axis, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, PxReal, coefficient, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, PxReal, recipCoefficient, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, PxU32, mLLLinkIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, ArticulationTendonJointCore, mParent, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, PxU32, mLLTendonJointIndex, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, ArticulationTendonJointCore, ArticulationFixedTendonSim, mTendonSim, PxMetaDataFlag::ePTR)
}

///////////////////////////////////////////////////////////////////////////////

static void getBinaryMetaData_ArticulationLimit(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream, PxArticulationLimit)
	PX_DEF_BIN_METADATA_ITEM(stream, PxArticulationLimit, PxReal, low, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxArticulationLimit, PxReal, high, 0)
}

static void getBinaryMetaData_ArticulationDrive(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxArticulationDriveType::Enum, PxU32)

	PX_DEF_BIN_METADATA_CLASS(stream, PxArticulationDrive)
	PX_DEF_BIN_METADATA_ITEM(stream, PxArticulationDrive, PxReal, stiffness, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxArticulationDrive, PxReal, damping, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxArticulationDrive, PxReal, maxForce, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, PxArticulationDrive, PxArticulationDriveType::Enum, driveType, 0)
}

static void getBinaryMetaData_ArticulationJointCore(PxOutputStream& stream)
{
	getBinaryMetaData_ArticulationLimit(stream);
	getBinaryMetaData_ArticulationDrive(stream);

	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationJointCore)	

	PX_DEF_BIN_METADATA_TYPEDEF(stream, ArticulationJointCoreDirtyFlags, PxU8)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxTransform, parentPose, 0)
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxTransform, childPose, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxArticulationLimit, limits, 0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxArticulationDrive, drives, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxReal, targetP, 0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxReal, targetV, 0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxReal, armature, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxReal, jointPos, 0)
	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxReal, jointVel, 0)
	
	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, frictionCoefficient, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxU8, dofIds, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxU8, motion, 0)

	PX_DEF_BIN_METADATA_ITEMS_AUTO(stream, Dy::ArticulationJointCore, PxU8, invDofIds, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxReal, maxJointVelocity, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, ArticulationJointCoreDirtyFlags, jointDirtyFlag, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxU32, jointOffset, 0)

	PX_DEF_BIN_METADATA_ITEM(stream, Dy::ArticulationJointCore, PxU8, jointType, 0)
}

//
//static void getBinaryMetaData_ArticulationJointCore(PxOutputStream& stream)
//{	
//	getBinaryMetaData_ArticulationJointCoreBase(stream);
//	PX_DEF_BIN_METADATA_CLASS(stream, Dy::ArticulationJointCore)
//	PX_DEF_BIN_METADATA_BASE_CLASS(stream, Dy::ArticulationJointCore, Dy::ArticulationJointCoreBase)
//}

void Sc::ArticulationJointCore::getBinaryMetaData(PxOutputStream& stream)
{
	getBinaryMetaData_ArticulationJointCore(stream);
	PX_DEF_BIN_METADATA_CLASS(stream,	ArticulationJointCore)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, ArticulationJointSim,		mSim,		PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, Dy::ArticulationJointCore,	mCore,		0)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, Dy::ArticulationCore,		mArticulation, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, PxArticulationJointReducedCoordinate, mRootType, PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	ArticulationJointCore, PxU32, mLLLinkIndex,			0)
}

///////////////////////////////////////////////////////////////////////////////

/*
#define PX_DEF_BIN_METADATA_ARRAY(stream, Class, type, array)	\
{ PxMetaDataEntry tmp = {"void", #array".mData", PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getDataOffset(), PxMetaDataArray<type>::getDataSize(), 1, 0, PxMetaDataFlag::ePTR, 0};							PX_STORE_METADATA(stream, tmp); } \
{ PxMetaDataEntry tmp = {"PxU32", #array".mSize", PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getSizeOffset(), PxMetaDataArray<type>::getSizeSize(), 1, 0, 0, 0};											PX_STORE_METADATA(stream, tmp);	} \
{ PxMetaDataEntry tmp = {"PxU32", #array".mCapacity", PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getCapacityOffset(), PxMetaDataArray<type>::getCapacitySize(), 1, 0, PxMetaDataFlag::eCOUNT_MASK_MSB, 0};	PX_STORE_METADATA(stream, tmp);	} \
{ PxMetaDataEntry tmp = {#type, 0, PxU32(PX_OFFSET_OF(Class, array)) + PxMetaDataArray<type>::getSizeOffset(), PxMetaDataArray<type>::getSizeSize(), 0, 0, PxMetaDataFlag::eEXTRA_DATA, 0};									PX_STORE_METADATA(stream, tmp); }
*/

///////////////////////////////////////////////////////////////////////////////

void MaterialIndicesStruct::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_CLASS(stream,	MaterialIndicesStruct)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialIndicesStruct, PxU16,	indices,	PxMetaDataFlag::ePTR)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialIndicesStruct, PxU16,	numIndices,	0)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialIndicesStruct, PxU16,	pad,		PxMetaDataFlag::ePADDING)
	PX_DEF_BIN_METADATA_ITEM(stream,	MaterialIndicesStruct, PxU32,	gpuRemapId,	0)

	//------ Extra-data ------
	// indices
	PX_DEF_BIN_METADATA_EXTRA_ITEMS(stream, MaterialIndicesStruct, PxU16, indices, numIndices, PxMetaDataFlag::eHANDLE, PX_SERIAL_ALIGN)
}

///////////////////////////////////////////////////////////////////////////////

void GeometryUnion::getBinaryMetaData(PxOutputStream& stream)
{
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxGeometryType::Enum, PxU32)

	// The various PxGeometry classes are all public, so I can't really put the meta-data function in there. And then
	// I can't access their protected members. So we use the same trick as for the ShapeContainer
	class ShadowConvexMeshGeometry : public PxConvexMeshGeometryLL
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_TYPEDEF(stream_, PxConvexMeshGeometryFlags, PxU8)

			PX_DEF_BIN_METADATA_CLASS(stream_,	ShadowConvexMeshGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowConvexMeshGeometry, PxGeometryType::Enum,			mType,				0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowConvexMeshGeometry, float,						mTypePadding,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowConvexMeshGeometry, PxMeshScale,					scale,				0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowConvexMeshGeometry, PxConvexMesh,					convexMesh,			PxMetaDataFlag::ePTR)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowConvexMeshGeometry, PxConvexMeshGeometryFlags,	meshFlags,			0)
			PX_DEF_BIN_METADATA_ITEMS(stream_,	ShadowConvexMeshGeometry, PxU8,							paddingFromFlags,	PxMetaDataFlag::ePADDING, 3)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowConvexMeshGeometry, bool,							gpuCompatible,		0)
		}
	};
	ShadowConvexMeshGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxConvexMeshGeometryLL, ShadowConvexMeshGeometry)

	/////////////////

	class ShadowTriangleMeshGeometry : public PxTriangleMeshGeometryLL
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_TYPEDEF(stream_, PxMeshGeometryFlags, PxU8)

			PX_DEF_BIN_METADATA_CLASS(stream_,	ShadowTriangleMeshGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowTriangleMeshGeometry, PxGeometryType::Enum,		mType,				0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowTriangleMeshGeometry, float,						mTypePadding,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowTriangleMeshGeometry, PxMeshScale,				scale,				0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowTriangleMeshGeometry, PxMeshGeometryFlags,		meshFlags,			0)
			PX_DEF_BIN_METADATA_ITEMS(stream_,	ShadowTriangleMeshGeometry,	PxU8,						paddingFromFlags,	PxMetaDataFlag::ePADDING, 3)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowTriangleMeshGeometry, PxTriangleMesh,				triangleMesh,		PxMetaDataFlag::ePTR)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowTriangleMeshGeometry, MaterialIndicesStruct,		materialsLL,		0)
		}
	};
	ShadowTriangleMeshGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,PxTriangleMeshGeometryLL, ShadowTriangleMeshGeometry)

	/////////////////

	class ShadowHeightFieldGeometry : public PxHeightFieldGeometryLL
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,		ShadowHeightFieldGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, PxGeometryType::Enum,	mType,				0)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, float,					mTypePadding,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, PxHeightField,		    heightField,		PxMetaDataFlag::ePTR)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, PxReal,					heightScale,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, PxReal,					rowScale,			0)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, PxReal,					columnScale,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, PxMeshGeometryFlags,		heightFieldFlags,	0)
			PX_DEF_BIN_METADATA_ITEMS_AUTO(stream_,	ShadowHeightFieldGeometry, PxU8,					paddingFromFlags,	PxMetaDataFlag::ePADDING)
			PX_DEF_BIN_METADATA_ITEM(stream_,		ShadowHeightFieldGeometry, MaterialIndicesStruct,	materialsLL,		0)
		}
	};
	ShadowHeightFieldGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,PxHeightFieldGeometryLL, ShadowHeightFieldGeometry)

	/////////////////

	class ShadowPlaneGeometry : public PxPlaneGeometry
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,	ShadowPlaneGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowPlaneGeometry, PxGeometryType::Enum,	mType,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowPlaneGeometry, float,					mTypePadding,	0)
		}
	};
	ShadowPlaneGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream,PxPlaneGeometry, ShadowPlaneGeometry)

	/////////////////

	class ShadowSphereGeometry : public PxSphereGeometry
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,	ShadowSphereGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowSphereGeometry, PxGeometryType::Enum,		mType,		0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowSphereGeometry, float,					mTypePadding,	0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowSphereGeometry, PxReal,				    radius,		0)
		}
	};
	ShadowSphereGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxSphereGeometry, ShadowSphereGeometry)

	/////////////////

	class ShadowCapsuleGeometry : public PxCapsuleGeometry
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,	ShadowCapsuleGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowCapsuleGeometry, PxGeometryType::Enum,	mType,			0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowCapsuleGeometry, float,					mTypePadding,	0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowCapsuleGeometry, PxReal,					radius,			0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowCapsuleGeometry, PxReal,					halfHeight,		0)
		}
	};
	ShadowCapsuleGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxCapsuleGeometry, ShadowCapsuleGeometry)

	/////////////////

	class ShadowBoxGeometry : public PxBoxGeometry
	{
	public:
		static void getBinaryMetaData(PxOutputStream& stream_)
		{
			PX_DEF_BIN_METADATA_CLASS(stream_,	ShadowBoxGeometry)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowBoxGeometry, PxGeometryType::Enum,	mType,			0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowBoxGeometry, float,					mTypePadding,	0)
			PX_DEF_BIN_METADATA_ITEM(stream_,	ShadowBoxGeometry, PxVec3,					halfExtents,	0)
		}
	};
	ShadowBoxGeometry::getBinaryMetaData(stream);
	PX_DEF_BIN_METADATA_TYPEDEF(stream, PxBoxGeometry, ShadowBoxGeometry)

	/*
	- geom union offset & size
	- control type offset & size
	- type-to-class mapping
	*/

	PX_DEF_BIN_METADATA_CLASS(stream, GeometryUnion)

	PX_DEF_BIN_METADATA_UNION(stream,		GeometryUnion, mGeometry)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream,	GeometryUnion, PxSphereGeometry,		PxGeometryType::eSPHERE)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream, 	GeometryUnion, PxPlaneGeometry,			PxGeometryType::ePLANE)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream, 	GeometryUnion, PxCapsuleGeometry,		PxGeometryType::eCAPSULE)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream, 	GeometryUnion, PxBoxGeometry,			PxGeometryType::eBOX)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream, 	GeometryUnion, PxConvexMeshGeometryLL,	PxGeometryType::eCONVEXMESH)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream, 	GeometryUnion, PxTriangleMeshGeometryLL,PxGeometryType::eTRIANGLEMESH)
	PX_DEF_BIN_METADATA_UNION_TYPE(stream, 	GeometryUnion, PxHeightFieldGeometryLL,	PxGeometryType::eHEIGHTFIELD)
}
