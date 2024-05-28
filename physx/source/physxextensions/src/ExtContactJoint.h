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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef EXT_CONTACT_JOINT_H
#define EXT_CONTACT_JOINT_H

#include "common/PxTolerancesScale.h"
#include "extensions/PxContactJoint.h"

#include "ExtJoint.h"
#include "foundation/PxUserAllocated.h"
#include "CmUtils.h"

namespace physx
{
struct PxContactJointGeneratedValues;
namespace Ext
{
	struct ContactJointData : public JointData
	{
		PxVec3	contact;
		PxVec3	normal;
		PxReal	penetration;
		PxReal  restitution;
		PxReal	bounceThreshold;
	};

	typedef JointT<PxContactJoint, ContactJointData, PxContactJointGeneratedValues> ContactJointT;
	class ContactJoint : public ContactJointT
	{
	public:
		// PX_SERIALIZATION
										ContactJoint(PxBaseFlags baseFlags) : ContactJointT(baseFlags) {}
				void					resolveReferences(PxDeserializationContext& context);
		static	ContactJoint*			createObject(PxU8*& address, PxDeserializationContext& context)	{ return createJointObject<ContactJoint>(address, context);	}
		static	void					getBinaryMetaData(PxOutputStream& stream);
		//~PX_SERIALIZATION
										ContactJoint(const PxTolerancesScale& scale, PxRigidActor* actor0, const PxTransform& localFrame0, PxRigidActor* actor1, const PxTransform& localFrame1);
		// PxContactJoint
		virtual	PxVec3					getContact()	const	PX_OVERRIDE;
		virtual	void					setContact(const PxVec3& contact)	PX_OVERRIDE;
		virtual	PxVec3					getContactNormal()	const	PX_OVERRIDE;
		virtual	void					setContactNormal(const PxVec3& normal)	PX_OVERRIDE;
		virtual	PxReal					getPenetration()	const	PX_OVERRIDE;
		virtual	void					setPenetration(const PxReal penetration)	PX_OVERRIDE;
		virtual	PxReal					getRestitution()	const	PX_OVERRIDE;
		virtual	void					setRestitution(const PxReal resititution)	PX_OVERRIDE;
		virtual PxReal					getBounceThreshold()	const	PX_OVERRIDE;
		virtual void					setBounceThreshold(const PxReal bounceThreshold)	PX_OVERRIDE;
		virtual void					computeJacobians(PxJacobianRow* jacobian)	const	PX_OVERRIDE;
		virtual PxU32					getNbJacobianRows()	const	PX_OVERRIDE;
		//~PxContactJoint

		// PxConstraintConnector
		virtual PxConstraintSolverPrep	getPrep()	const	PX_OVERRIDE;
		//~PxConstraintConnector
	};

} // namespace Ext

} // namespace physx
#endif
