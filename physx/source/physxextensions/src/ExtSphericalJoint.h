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
		static	void					getBinaryMetaData(PxOutputStream& stream);
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
