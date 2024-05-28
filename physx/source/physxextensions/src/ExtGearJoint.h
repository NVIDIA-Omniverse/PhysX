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
		static	void					getBinaryMetaData(PxOutputStream& stream);
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
