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
		static	void					getBinaryMetaData(PxOutputStream& stream);
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
