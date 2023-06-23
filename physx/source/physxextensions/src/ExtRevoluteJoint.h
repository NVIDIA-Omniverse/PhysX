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

#ifndef EXT_REVOLUTE_JOINT_H
#define EXT_REVOLUTE_JOINT_H

#include "extensions/PxRevoluteJoint.h"

#include "ExtJoint.h"
#include "foundation/PxIntrinsics.h"
#include "CmUtils.h"

namespace physx
{
struct PxRevoluteJointGeneratedValues;

namespace Ext
{
	struct RevoluteJointData : public JointData
	{
		PxReal					driveVelocity;
		PxReal					driveForceLimit;
		PxReal					driveGearRatio;

		PxJointAngularLimitPair	limit;
							
		PxRevoluteJointFlags	jointFlags;
//	private:	// PT: must be public for a benchmark
		RevoluteJointData(const PxJointAngularLimitPair& pair) : limit(pair)	{}
	};

    typedef JointT<PxRevoluteJoint, RevoluteJointData, PxRevoluteJointGeneratedValues> RevoluteJointT;
    
	class RevoluteJoint : public RevoluteJointT
	{
	public:
// PX_SERIALIZATION
										RevoluteJoint(PxBaseFlags baseFlags) : RevoluteJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	RevoluteJoint*			createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<RevoluteJoint>(address, context);	}
		static	void					getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
										RevoluteJoint(const PxTolerancesScale& /*scale*/, PxRigidActor* actor0, const PxTransform& localFrame0,  PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxRevoluteJoint
		virtual	PxReal					getAngle() const	PX_OVERRIDE;
		virtual	PxReal					getVelocity() const	PX_OVERRIDE;
		virtual	void					setLimit(const PxJointAngularLimitPair& limit)	PX_OVERRIDE;
		virtual	PxJointAngularLimitPair	getLimit()	const	PX_OVERRIDE;
		virtual	void					setDriveVelocity(PxReal velocity, bool autowake = true)	PX_OVERRIDE;
		virtual	PxReal					getDriveVelocity() const	PX_OVERRIDE;
		virtual	void					setDriveForceLimit(PxReal forceLimit)	PX_OVERRIDE;
		virtual	PxReal					getDriveForceLimit() const	PX_OVERRIDE;
		virtual	void					setDriveGearRatio(PxReal gearRatio)	PX_OVERRIDE;
		virtual	PxReal					getDriveGearRatio() const	PX_OVERRIDE;
		virtual	void					setRevoluteJointFlags(PxRevoluteJointFlags flags)	PX_OVERRIDE;
		virtual	void					setRevoluteJointFlag(PxRevoluteJointFlag::Enum flag, bool value)	PX_OVERRIDE;
		virtual	PxRevoluteJointFlags	getRevoluteJointFlags()	const	PX_OVERRIDE;
		//~PxRevoluteJoint
	
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
