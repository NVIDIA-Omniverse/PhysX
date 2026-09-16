// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ExtFixedJoint.h"
#include "ExtConstraintHelper.h"

#include "omnipvd/ExtOmniPvdSetData.h"

using namespace physx;
using namespace Ext;

FixedJoint::FixedJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1) :
	FixedJointT(PxJointConcreteType::eFIXED, actor0, localFrame0, actor1, localFrame1, "FixedJointData")
{
//	FixedJointData* data = static_cast<FixedJointData*>(mData);
}

static void FixedJointVisualize(PxConstraintVisualizer& viz, const void* constantBlock, const PxTransform& body0Transform, const PxTransform& body1Transform, PxU32 flags)
{
	if(flags & PxConstraintVisualizationFlag::eLOCAL_FRAMES)
	{
		const FixedJointData& data = *reinterpret_cast<const FixedJointData*>(constantBlock);

		PxTransform32 cA2w, cB2w;
		joint::computeJointFrames(cA2w, cB2w, data, body0Transform, body1Transform);
		viz.visualizeJointFrames(cA2w, cB2w);
	}
}

//TAG:solverprepshader
static PxU32 FixedJointSolverPrep(Px1DConstraint* constraints,
	PxVec3p& body0WorldOffset,
	PxU32 /*maxConstraints*/,
	PxConstraintInvMassScale& invMassScale,
	const void* constantBlock,
	const PxTransform& bA2w,
	const PxTransform& bB2w,
	bool /*useExtendedLimits*/,
	PxVec3p& cA2wOut, PxVec3p& cB2wOut)
{
	const FixedJointData& data = *reinterpret_cast<const FixedJointData*>(constantBlock);

	PxTransform32 cA2w, cB2w;
	joint::ConstraintHelper ch(constraints, invMassScale, cA2w, cB2w, body0WorldOffset, data, bA2w, bB2w);

	joint::applyNeighborhoodOperator(cA2w, cB2w);

	PxVec3 ra, rb;
	ch.prepareLockedAxes(cA2w.q, cB2w.q, cA2w.transformInv(cB2w.p), 7, 7, ra, rb);
	cA2wOut = ra + bA2w.p;
	cB2wOut = rb + bB2w.p;

	return ch.getCount();
}

///////////////////////////////////////////////////////////////////////////////

static PxConstraintShaderTable gFixedJointShaders = { FixedJointSolverPrep, FixedJointVisualize, PxConstraintFlag::Enum(0) };

PxConstraintSolverPrep FixedJoint::getPrep()	const	{ return gFixedJointShaders.solverPrep;  }

PxFixedJoint* physx::PxFixedJointCreate(PxPhysics& physics, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1)
{
	PX_CHECK_AND_RETURN_NULL(localFrame0.isSane(), "PxFixedJointCreate: local frame 0 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL(localFrame1.isSane(), "PxFixedJointCreate: local frame 1 is not a valid transform"); 
	PX_CHECK_AND_RETURN_NULL((actor0 && actor0->is<PxRigidBody>()) || (actor1 && actor1->is<PxRigidBody>()), "PxFixedJointCreate: at least one actor must be dynamic");
	PX_CHECK_AND_RETURN_NULL(actor0 != actor1, "PxFixedJointCreate: actors must be different");

	return createJointT<FixedJoint, FixedJointData>(physics, actor0, localFrame0, actor1, localFrame1, gFixedJointShaders);
}

// PX_SERIALIZATION
void FixedJoint::resolveReferences(PxDeserializationContext& context)
{
	mPxConstraint = resolveConstraintPtr(context, mPxConstraint, this, gFixedJointShaders);
}
//~PX_SERIALIZATION

#if PX_SUPPORT_OMNI_PVD

template<> void physx::Ext::omniPvdInitJoint<FixedJoint>(OmniPvdWriter* pvdWriter, const OmniPvdPxExtensionsRegistrationData* pvdRegData, FixedJoint& joint); // declared before the wrapper so the wrapper binds the explicit form below instead of implicitly instantiating it
template<>
void physx::Ext::omniPvdInitJoint<FixedJoint>(FixedJoint& joint)
{
	OMNI_PVD_WRITE_SCOPE_BEGIN(pvdWriter, pvdRegData)
	omniPvdInitJoint(pvdWriter, pvdRegData, joint);
	OMNI_PVD_WRITE_SCOPE_END
}

template<>
void physx::Ext::omniPvdInitJoint<FixedJoint>(OmniPvdWriter* pvdWriter, const OmniPvdPxExtensionsRegistrationData* pvdRegData, FixedJoint& joint)
{
	PxFixedJoint& j = static_cast<PxFixedJoint&>(joint);
	OMNI_PVD_CREATE_EXPLICIT(pvdWriter, pvdRegData, OMNI_PVD_CONTEXT_HANDLE, PxFixedJoint, j);
	omniPvdSetBaseJointParams(pvdWriter, pvdRegData, static_cast<PxJoint&>(joint), PxJointConcreteType::eFIXED);
}

#endif
