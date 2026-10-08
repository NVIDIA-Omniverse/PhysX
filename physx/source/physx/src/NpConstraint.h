// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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
	virtual			void						release()	PX_OVERRIDE PX_FINAL;
	virtual			PxScene*					getScene()	const	PX_OVERRIDE PX_FINAL;
	virtual			void						getActors(PxRigidActor*& actor0, PxRigidActor*& actor1)	const	PX_OVERRIDE PX_FINAL;
	virtual			void						setActors(PxRigidActor* actor0, PxRigidActor* actor1)	PX_OVERRIDE PX_FINAL;
	virtual			void						markDirty()	PX_OVERRIDE PX_FINAL;
	virtual			PxConstraintFlags			getFlags()	const	PX_OVERRIDE PX_FINAL;
	virtual			void						setFlags(PxConstraintFlags flags)	PX_OVERRIDE PX_FINAL;
	virtual			void						setFlag(PxConstraintFlag::Enum flag, bool value)	PX_OVERRIDE PX_FINAL;
	virtual			void						getForce(PxVec3& linear, PxVec3& angular)	const	PX_OVERRIDE PX_FINAL;
	virtual			bool						isValid()	const	PX_OVERRIDE PX_FINAL;
	virtual			void						setBreakForce(PxReal linear, PxReal angular)	PX_OVERRIDE PX_FINAL;
	virtual			void						getBreakForce(PxReal& linear, PxReal& angular)	const	PX_OVERRIDE PX_FINAL;
	virtual			void						setMinResponseThreshold(PxReal threshold)	PX_OVERRIDE PX_FINAL;
	virtual			PxReal						getMinResponseThreshold()	const	PX_OVERRIDE PX_FINAL;
	virtual			void*						getExternalReference(PxU32& typeID)	PX_OVERRIDE PX_FINAL;
	virtual			void						setConstraintFunctions(PxConstraintConnector& n, const PxConstraintShaderTable& t)	PX_OVERRIDE PX_FINAL;
	virtual			PxConstraintGPUIndex		getGPUIndex() const PX_OVERRIDE PX_FINAL;
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
