// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef DY_CONTACT_PREP_H
#define DY_CONTACT_PREP_H

#include "CmSpatialVector.h"
#include "DySolverConstraintDesc.h"

namespace physx
{
struct PxcNpWorkUnit;
class PxsConstraintBlockManager;
class PxcConstraintBlockStream;
struct PxsContactManagerOutput;
class FrictionPatchStreamPair;
struct PxSolverBody;
struct PxSolverBodyData;
struct PxSolverConstraintDesc;
class PxsContactManager;

namespace Dy
{
	class ThreadContext;
	struct CorrelationBuffer;

#define CREATE_FINALIZE_SOLVER_CONTACT_METHOD_ARGS	\
	PxSolverContactDesc& contactDesc,				\
	PxsContactManagerOutput& output,				\
	ThreadContext& threadContext,					\
	const PxReal invDtF32,							\
	const PxReal dtF32,								\
	PxReal bounceThresholdF32,						\
	PxReal frictionOffsetThreshold,					\
	PxReal correlationDistance,						\
	PxConstraintAllocator& constraintAllocator,		\
	Cm::SpatialVectorF* Z

#define CREATE_FINALIZE_SOLVER_CONTACT_METHOD_ARGS_4	\
	PxsContactManagerOutput** outputs,					\
	ThreadContext& threadContext,						\
	PxSolverContactDesc* blockDescs,					\
	const PxReal invDtF32,								\
	const PxReal dtF32,									\
	PxReal bounceThresholdF32,							\
	PxReal	frictionThresholdF32,						\
	PxReal	correlationDistanceF32,						\
	PxConstraintAllocator& constraintAllocator				
	
/*!
Method prototype for create finalize solver contact
*/

typedef	bool (*PxcCreateFinalizeSolverContactMethod)(CREATE_FINALIZE_SOLVER_CONTACT_METHOD_ARGS);

typedef	SolverConstraintPrepState::Enum (*PxcCreateFinalizeSolverContactMethod4)(CREATE_FINALIZE_SOLVER_CONTACT_METHOD_ARGS_4);

bool createFinalizeSolverContacts(	PxSolverContactDesc& contactDesc,
									PxsContactManagerOutput& output,
									ThreadContext& threadContext,
									const PxReal invDtF32,
									const PxReal dtF32,
									PxReal bounceThresholdF32,
									PxReal frictionOffsetThreshold,
									PxReal correlationDistance,
									const PxReal biasCoefficient,
									PxConstraintAllocator& constraintAllocator);

bool createFinalizeSolverContacts(	PxSolverContactDesc& contactDesc,
									CorrelationBuffer& c,
									const PxReal invDtF32,
									const PxReal dtF32,
									PxReal bounceThresholdF32,
									PxReal frictionOffsetThreshold,
									PxReal correlationDistance,
									const PxReal biasCoefficient,
									PxConstraintAllocator& constraintAllocator);

SolverConstraintPrepState::Enum createFinalizeSolverContacts4(	PxsContactManagerOutput** outputs,
																 ThreadContext& threadContext,
																 PxSolverContactDesc* blockDescs,
																 const PxReal invDtF32,
																 const PxReal dtF32,
																 PxReal bounceThresholdF32,
																 PxReal frictionOffsetThreshold,
																 PxReal correlationDistance,
																 const PxReal biasCoefficient,
																 PxConstraintAllocator& constraintAllocator);

SolverConstraintPrepState::Enum createFinalizeSolverContacts4(	Dy::CorrelationBuffer& c,
																PxSolverContactDesc* blockDescs,
																const PxReal invDtF32,
																const PxReal dtF32,
																PxReal bounceThresholdF32,
																PxReal	frictionOffsetThreshold,
																PxReal correlationDistance,
																const PxReal biasCoefficient,
																PxConstraintAllocator& constraintAllocator);

PxU32 getContactManagerConstraintDesc(const PxsContactManagerOutput& cmOutput, const PxsContactManager& cm, PxSolverConstraintDesc& desc);

template <typename SolverContactDesc>
void updateFrictionAnchorCountAndPosition(PxSolverConstraintDesc& desc, PxsContactManagerOutput& output, SolverContactDesc& blockDesc);

template <typename SolverFriction>
void writeBackContactFriction(const SolverFriction* PX_RESTRICT frictions, PxU32 numFrictionConstr, PxU32 frictionStride, PxVec3* PX_RESTRICT vFrictionWriteback);

class BlockAllocator : public PxConstraintAllocator
{
	PxsConstraintBlockManager& mConstraintBlockManager;
	PxcConstraintBlockStream& mConstraintBlockStream;
	FrictionPatchStreamPair& mFrictionPatchStreamPair;
	PxU32& mTotalConstraintByteSize;
public:

	BlockAllocator(PxsConstraintBlockManager& constraintBlockManager, PxcConstraintBlockStream& constraintBlockStream, FrictionPatchStreamPair& frictionPatchStreamPair,
		PxU32& totalConstraintByteSize) :
		mConstraintBlockManager(constraintBlockManager), mConstraintBlockStream(constraintBlockStream), mFrictionPatchStreamPair(frictionPatchStreamPair),
		mTotalConstraintByteSize(totalConstraintByteSize)
	{
	}

	virtual PxU8* reserveConstraintData(const PxU32 size) PX_OVERRIDE;

	virtual PxU8* reserveFrictionData(const PxU32 size) PX_OVERRIDE;

	virtual PxU8* findInputPatches(PxU8* frictionCookie)
	{
		return frictionCookie;
	}

	PX_NOCOPY(BlockAllocator)
};

}

}

#endif
