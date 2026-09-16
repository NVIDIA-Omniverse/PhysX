// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_GEAR_JOINT_H
#define EXT_GEAR_JOINT_H

#include "extensions/PxGearJoint.h"
#include "ExtJoint.h"
#include "CmUtils.h"

namespace physx
{
struct PxGearJointGeneratedValues;
namespace Ext
{
	struct GearJointData : public JointData
	{
		const PxBase*	hingeJoint0; //either PxJoint or PxArticulationJointReducedCoordinate
		const PxBase*	hingeJoint1; //either PxJoint or PxArticulationJointReducedCoordinate
		float			gearRatio;
		float			error;
	};

	typedef JointT<PxGearJoint, GearJointData, PxGearJointGeneratedValues> GearJointT;

	class GearJoint : public GearJointT
	{
	public:
// PX_SERIALIZATION
										GearJoint(PxBaseFlags baseFlags) : GearJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	GearJoint*				createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<GearJoint>(address, context);	}
//~PX_SERIALIZATION
										GearJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxGearJoint
		virtual	bool					setHinges(const PxBase* hinge0, const PxBase* hinge1)	PX_OVERRIDE;
		virtual	void					getHinges(const PxBase*& hinge0, const PxBase*& hinge1)	const	PX_OVERRIDE;
		virtual	void					setGearRatio(float ratio)	PX_OVERRIDE;
		virtual	float					getGearRatio()	const	PX_OVERRIDE;
		//~PxGearJoint

		// PxConstraintConnector
		virtual	void*					prepareData()	PX_OVERRIDE
										{
											updateError();
											return mData;
										}
		virtual PxConstraintSolverPrep	getPrep()	const	PX_OVERRIDE;
		//~PxConstraintConnector
	private:
				float					mVirtualAngle0;
				float					mVirtualAngle1;
				float					mPersistentAngle0;
				float					mPersistentAngle1;
				bool					mInitDone;

				void					updateError();
				void					resetError();
	};
} // namespace Ext

}

#endif
