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
		static	void					getBinaryMetaData(PxOutputStream& stream);
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
