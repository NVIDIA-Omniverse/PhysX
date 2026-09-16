// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_SPHERICAL_JOINT_H
#define EXT_SPHERICAL_JOINT_H

#include "extensions/PxSphericalJoint.h"

#include "ExtJoint.h"
#include "CmUtils.h"

namespace physx
{
struct PxSphericalJointGeneratedValues;
namespace Ext
{
	struct SphericalJointData: public JointData
	{
		PxJointLimitCone		limit;

		PxSphericalJointFlags	jointFlags;
	private:
		SphericalJointData(const PxJointLimitCone& cone) : limit(cone)	{}
	};
    
    typedef JointT<PxSphericalJoint, SphericalJointData, PxSphericalJointGeneratedValues> SphericalJointT;
   
	class SphericalJoint : public SphericalJointT
	{
	public:
// PX_SERIALIZATION
										SphericalJoint(PxBaseFlags baseFlags) : SphericalJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	SphericalJoint*			createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<SphericalJoint>(address, context);	}
//~PX_SERIALIZATION
										SphericalJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxSphericalJoint
		virtual	void					setLimitCone(const PxJointLimitCone &limit)	PX_OVERRIDE;
		virtual	PxJointLimitCone		getLimitCone() const	PX_OVERRIDE;
		virtual	void					setSphericalJointFlags(PxSphericalJointFlags flags)	PX_OVERRIDE;
		virtual	void					setSphericalJointFlag(PxSphericalJointFlag::Enum flag, bool value)	PX_OVERRIDE;
		virtual	PxSphericalJointFlags	getSphericalJointFlags() const	PX_OVERRIDE;
		virtual PxReal					getSwingYAngle() const	PX_OVERRIDE;
		virtual PxReal					getSwingZAngle() const	PX_OVERRIDE;
		//~PxSphericalJoint

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
