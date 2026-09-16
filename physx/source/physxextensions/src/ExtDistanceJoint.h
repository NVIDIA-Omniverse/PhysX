// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_DISTANCE_JOINT_H
#define EXT_DISTANCE_JOINT_H

#include "common/PxTolerancesScale.h"
#include "extensions/PxDistanceJoint.h"

#include "ExtJoint.h"
#include "foundation/PxUserAllocated.h"
#include "CmUtils.h"

namespace physx
{
struct PxDistanceJointGeneratedValues;
namespace Ext
{
	struct DistanceJointData : public JointData
	{
		PxReal					minDistance;
		PxReal					maxDistance;
		PxReal					tolerance;
		PxReal					stiffness;
		PxReal					damping;

		PxDistanceJointFlags	jointFlags;
	};

    typedef JointT<PxDistanceJoint, DistanceJointData, PxDistanceJointGeneratedValues> DistanceJointT;
	class DistanceJoint : public DistanceJointT
	{
	public:
		// PX_SERIALIZATION
										DistanceJoint(PxBaseFlags baseFlags) : DistanceJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	DistanceJoint*			createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<DistanceJoint>(address, context);	}
		//~PX_SERIALIZATION
										DistanceJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxDistanceJoint
		virtual	PxReal					getDistance()	const	PX_OVERRIDE;
		virtual	void					setMinDistance(PxReal distance)	PX_OVERRIDE;
		virtual	PxReal					getMinDistance()	const	PX_OVERRIDE;
		virtual	void					setMaxDistance(PxReal distance)	PX_OVERRIDE;
		virtual	PxReal					getMaxDistance()	const	PX_OVERRIDE;
		virtual	void					setTolerance(PxReal tolerance)	PX_OVERRIDE;
		virtual	PxReal					getTolerance()	const	PX_OVERRIDE;
		virtual	void					setStiffness(PxReal spring)	PX_OVERRIDE;
		virtual	PxReal					getStiffness()	const	PX_OVERRIDE;
		virtual	void					setDamping(PxReal damping)	PX_OVERRIDE;
		virtual	PxReal					getDamping()	const	PX_OVERRIDE;
		virtual	void					setDistanceJointFlags(PxDistanceJointFlags flags)	PX_OVERRIDE;
		virtual	void					setDistanceJointFlag(PxDistanceJointFlag::Enum flag, bool value)	PX_OVERRIDE;
		virtual	PxDistanceJointFlags	getDistanceJointFlags()	const	PX_OVERRIDE;
		//~PxDistanceJoint

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
