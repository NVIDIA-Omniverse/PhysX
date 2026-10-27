// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_FIXED_JOINT_H
#define EXT_FIXED_JOINT_H

#include "extensions/PxFixedJoint.h"

#include "ExtJoint.h"
#include "CmUtils.h"

namespace physx
{
struct PxFixedJointGeneratedValues;
namespace Ext
{
	struct FixedJointData : public JointData
	{
	};

	typedef JointT<PxFixedJoint, FixedJointData, PxFixedJointGeneratedValues> FixedJointT;

	class FixedJoint : public FixedJointT
	{
	public:
// PX_SERIALIZATION
										FixedJoint(PxBaseFlags baseFlags) : FixedJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	FixedJoint*				createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<FixedJoint>(address, context);	}
//~PX_SERIALIZATION
										FixedJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxFixedJoint
		//~PxFixedJoint
	
		// PxConstraintConnector
		virtual PxConstraintSolverPrep	getPrep()	const	PX_OVERRIDE;
		//~PxConstraintConnector
	};
} // namespace Ext

} // namespace physx

#endif
