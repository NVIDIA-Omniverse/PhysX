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

#ifndef NP_CONSTRAINT_H
#define NP_CONSTRAINT_H

#include "foundation/PxUserAllocated.h"
#include "PxConstraint.h"
#include "NpBase.h"
#include "../../../simulationcontroller/include/ScConstraintCore.h"
#include "NpActor.h"

namespace physx
{
class NpScene;

class NpConstraint : public PxConstraint, public NpBase
{
public:
// PX_SERIALIZATION
												NpConstraint(PxBaseFlags baseFlags) : PxConstraint(baseFlags), NpBase(PxEmpty), mCore(PxEmpty) {}
	static			NpConstraint*				createObject(PxU8*& address, PxDeserializationContext& context);
	static			void						getBinaryMetaData(PxOutputStream& stream);
					void						preExportDataReset() {}
					void						exportExtraData(PxSerializationContext&) {}
					void						importExtraData(PxDeserializationContext&) {}
					void						resolveReferences(PxDeserializationContext& context);
	virtual			void						requiresObjects(PxProcessPxBaseCallback&) {}
	virtual		    bool						isSubordinate() const { return true; }  
//~PX_SERIALIZATION
												NpConstraint(PxRigidActor* actor0, PxRigidActor* actor1, PxConstraintConnector& connector, const PxConstraintShaderTable& shaders, PxU32 dataSize);
	virtual										~NpConstraint();
	// PxConstraint
	virtual			void						release()	PX_OVERRIDE;
	virtual			PxScene*					getScene()	const	PX_OVERRIDE;
	virtual			void						getActors(PxRigidActor*& actor0, PxRigidActor*& actor1)	const	PX_OVERRIDE;
	virtual			void						setActors(PxRigidActor* actor0, PxRigidActor* actor1)	PX_OVERRIDE;
	virtual			void						markDirty()	PX_OVERRIDE;
	virtual			PxConstraintFlags			getFlags()	const	PX_OVERRIDE;
	virtual			void						setFlags(PxConstraintFlags flags)	PX_OVERRIDE;
	virtual			void						setFlag(PxConstraintFlag::Enum flag, bool value)	PX_OVERRIDE;
	virtual			void						getForce(PxVec3& linear, PxVec3& angular)	const	PX_OVERRIDE;
	virtual			bool						isValid()	const	PX_OVERRIDE;
	virtual			void						setBreakForce(PxReal linear, PxReal angular)	PX_OVERRIDE;
	virtual			void						getBreakForce(PxReal& linear, PxReal& angular)	const	PX_OVERRIDE;
	virtual			void						setMinResponseThreshold(PxReal threshold)	PX_OVERRIDE;
	virtual			PxReal						getMinResponseThreshold()	const	PX_OVERRIDE;
	virtual			void*						getExternalReference(PxU32& typeID)	PX_OVERRIDE;
	virtual			void						setConstraintFunctions(PxConstraintConnector& n, const PxConstraintShaderTable& t)	PX_OVERRIDE;
	//~PxConstraint

					void						updateConstants(PxsSimulationController& simController);
					void						comShift(PxRigidActor*);
					void						actorDeleted(PxRigidActor*);

					NpScene*					getSceneFromActors() const;

	PX_FORCE_INLINE	Sc::ConstraintCore&			getCore()			{ return mCore; }
	PX_FORCE_INLINE	const Sc::ConstraintCore&	getCore() const		{ return mCore; }
	static PX_FORCE_INLINE size_t				getCoreOffset()		{ return PX_OFFSET_OF_RT(NpConstraint, mCore); }

	PX_FORCE_INLINE	bool						isDirty() const		{ return mCore.isDirty(); }
	PX_FORCE_INLINE	void						markClean()			{ mCore.clearDirty(); }
private:
					PxRigidActor*				mActor0;
					PxRigidActor*				mActor1;
					Sc::ConstraintCore			mCore;

					void						addConnectors(PxRigidActor* actor0, PxRigidActor* actor1);
					void						removeConnectors(const char* errorMsg0, const char* errorMsg1);

	PX_INLINE		void						scSetFlags(PxConstraintFlags f)
												{
													PX_ASSERT(!isAPIWriteForbidden());
													mCore.setFlags(f);
													markDirty();
													UPDATE_PVD_PROPERTY
												}
};

}

#endif
