// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_PRISMATIC_JOINT_H
#define EXT_PRISMATIC_JOINT_H

#include "common/PxTolerancesScale.h"
#include "extensions/PxPrismaticJoint.h"

#include "ExtJoint.h"
#include "CmUtils.h"

namespace physx
{
struct PxPrismaticJointGeneratedValues;
namespace Ext
{
	struct PrismaticJointData : public JointData
	{
		PxJointLinearLimitPair	limit;

		PxPrismaticJointFlags	jointFlags;

	private:
		PrismaticJointData(const PxJointLinearLimitPair& pair) : limit(pair)	{}
	};

    typedef JointT<PxPrismaticJoint, PrismaticJointData, PxPrismaticJointGeneratedValues> PrismaticJointT;
   
	class PrismaticJoint : public PrismaticJointT
	{
	public:
// PX_SERIALIZATION
										PrismaticJoint(PxBaseFlags baseFlags) : PrismaticJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	PrismaticJoint*			createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<PrismaticJoint>(address, context);	}
//~PX_SERIALIZATION
										PrismaticJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxPrismaticJoint
		virtual	PxReal					getPosition()	const	PX_OVERRIDE	{	return getRelativeTransform().p.x;		}
		virtual	PxReal					getVelocity()	const 	PX_OVERRIDE	{	return getRelativeLinearVelocity().x;	}
		virtual	void					setLimit(const PxJointLinearLimitPair& limit)	PX_OVERRIDE;
		virtual	PxJointLinearLimitPair	getLimit()	const	PX_OVERRIDE;
		virtual	void					setPrismaticJointFlags(PxPrismaticJointFlags flags)	PX_OVERRIDE;
		virtual	void					setPrismaticJointFlag(PxPrismaticJointFlag::Enum flag, bool value)	PX_OVERRIDE;
		virtual	PxPrismaticJointFlags	getPrismaticJointFlags()	const	PX_OVERRIDE;
		//~PxPrismaticJoint

		// PxConstraintConnector
		virtual PxConstraintSolverPrep	getPrep()	const	PX_OVERRIDE;
#if PX_SUPPORT_OMNI_PVD
		virtual void updateOmniPvdProperties() const PX_OVERRIDE;
#endif
		//~PxConstraintConnector
	};
} // namespace Ext

} // namespace physx

#endif
