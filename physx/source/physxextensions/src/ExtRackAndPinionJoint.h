// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef EXT_RACK_AND_PINION_JOINT_H
#define EXT_RACK_AND_PINION_JOINT_H

#include "extensions/PxRackAndPinionJoint.h"
#include "ExtJoint.h"
#include "CmUtils.h"

namespace physx
{
struct PxRackAndPinionJointGeneratedValues;
namespace Ext
{
	struct RackAndPinionJointData : public JointData
	{
		const PxBase*	hingeJoint;
		const PxBase*	prismaticJoint;
		float			ratio;
		float			px;
		float			vangle;
	};

	typedef JointT<PxRackAndPinionJoint, RackAndPinionJointData, PxRackAndPinionJointGeneratedValues> RackAndPinionJointT;

	class RackAndPinionJoint : public RackAndPinionJointT
	{
	public:
// PX_SERIALIZATION
										RackAndPinionJoint(PxBaseFlags baseFlags) : RackAndPinionJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	RackAndPinionJoint*		createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<RackAndPinionJoint>(address, context);	}
//~PX_SERIALIZATION
										RackAndPinionJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxRackAndPinionJoint
		virtual	bool					setJoints(const PxBase* hinge, const PxBase* prismatic)	PX_OVERRIDE;
		virtual	void					getJoints(const PxBase*& hinge, const PxBase*& prismatic)	const	PX_OVERRIDE;
		virtual	void					setRatio(float ratio)	PX_OVERRIDE;
		virtual	float					getRatio()	const	PX_OVERRIDE;
		virtual	bool					setData(PxU32 nbRackTeeth, PxU32 nbPinionTeeth, float rackLength)	PX_OVERRIDE;
		//~PxRackAndPinionJoint

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
				float					mPersistentAngle0;
				bool					mInitDone;

				void					updateError();
				void					resetError();
	};
} // namespace Ext

}

#endif
