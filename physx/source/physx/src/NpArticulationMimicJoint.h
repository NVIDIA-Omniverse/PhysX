// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "PxArticulationReducedCoordinate.h"
#include "PxArticulationMimicJoint.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxMemory.h"
#include "ScArticulationMimicJointCore.h"
#include "NpBase.h"

#ifndef NP_ARTICULATION_MIMIC_JOINT_H
#define NP_ARTICULATION_MIMIC_JOINT_H

namespace physx
{

typedef PxU32	ArticulationMimicJointHandle;

class NpArticulationMimicJoint : public PxArticulationMimicJoint, public NpBase
{
public:

// PX_SERIALIZATION
											NpArticulationMimicJoint(PxBaseFlags baseFlags)
													: PxArticulationMimicJoint(baseFlags), NpBase(PxEmpty), mCore(PxEmpty) {}
				void						preExportDataReset() {}
	virtual		void						exportExtraData(PxSerializationContext& ) {}
				void						importExtraData(PxDeserializationContext& ) {}
				void						resolveReferences(PxDeserializationContext& );
	virtual		void						requiresObjects(PxProcessPxBaseCallback&) {}
	virtual		bool						isSubordinate()  const	 { return true; } 
	static		NpArticulationMimicJoint*	createObject(PxU8*& address, PxDeserializationContext& context);
//~PX_SERIALIZATION

	NpArticulationMimicJoint(
		const PxArticulationJointReducedCoordinate& jointA, PxArticulationAxis::Enum axisA, 
		const PxArticulationJointReducedCoordinate& jointB, PxArticulationAxis::Enum axisB, 
		PxReal gearRatio, PxReal offset, 
		PxReal naturalFrequency, PxReal dampingRatio);
	virtual ~NpArticulationMimicJoint() {}

	// PxBase
	virtual void release() PX_OVERRIDE PX_FINAL;
	//~PxBase

	// PxArticulationMimicJoint
	virtual PxArticulationReducedCoordinate& getArticulation() const PX_OVERRIDE PX_FINAL;
	virtual PxReal getGearRatio() const PX_OVERRIDE PX_FINAL;
	virtual void setGearRatio(PxReal gearRatio) PX_OVERRIDE PX_FINAL;
	virtual PxReal getOffset() const PX_OVERRIDE PX_FINAL;
	virtual void setOffset(PxReal offset) PX_OVERRIDE PX_FINAL;
	virtual PxReal getNaturalFrequency() const PX_OVERRIDE PX_FINAL;
	virtual void setNaturalFrequency(PxReal naturalFrequency) PX_OVERRIDE PX_FINAL;
	virtual PxReal getDampingRatio() const PX_OVERRIDE PX_FINAL;
	virtual void setDampingRatio(PxReal dampingRatio)  PX_OVERRIDE PX_FINAL;
	virtual PxArticulationJointReducedCoordinate& getJointA() const PX_OVERRIDE PX_FINAL;
	virtual PxArticulationJointReducedCoordinate& getJointB() const PX_OVERRIDE PX_FINAL;
	virtual PxArticulationAxis::Enum getAxisA() const PX_OVERRIDE PX_FINAL;
	virtual PxArticulationAxis::Enum getAxisB() const PX_OVERRIDE PX_FINAL;

	//~PxArticulationMimicJoint

	PX_FORCE_INLINE	void setHandle(ArticulationMimicJointHandle handle) { mHandle = handle; }

	PX_FORCE_INLINE Sc::ArticulationMimicJointCore& getMimicJointCore() {return mCore;}
	PxArticulationLink* getLinkA() {return mLinkA;}
	PxArticulationLink* getLinkB() {return mLinkB;}

private:

	PxArticulationLink* mLinkA;
	PxArticulationLink* mLinkB;

	Sc::ArticulationMimicJointCore mCore;

	//mHandle is the index into the array of mimic joints owned by NpArticulationReducedCoordinate.
	//Releasing a mimic joint will lead to the last mimic joint filling the array entry of the released
	//mimic joint. The handle of the last mimic joint will be updated accordingly.
	ArticulationMimicJointHandle   mHandle;
};
}

#endif //NP_ARTICULATION_MIMIC_JOINT_H
