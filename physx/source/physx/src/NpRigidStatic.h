// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_RIGID_STATIC_H
#define NP_RIGID_STATIC_H

#include "PxRigidStatic.h"
#include "NpRigidActorTemplate.h"
#include "ScStaticCore.h"

namespace physx
{
typedef NpRigidActorTemplate<PxRigidStatic> NpRigidStaticT;

class NpRigidStatic : public NpRigidStaticT
{
public:
// PX_SERIALIZATION
											NpRigidStatic(PxBaseFlags baseFlags) : NpRigidStaticT(baseFlags), mCore(PxEmpty) {}
					void					preExportDataReset() { NpRigidStaticT::preExportDataReset(); }
	virtual			void					requiresObjects(PxProcessPxBaseCallback& c) PX_OVERRIDE;
	static			NpRigidStatic*			createObject(PxU8*& address, PxDeserializationContext& context);
//~PX_SERIALIZATION

											NpRigidStatic(const PxTransform& pose);
	virtual									~NpRigidStatic();

	// PxActor
	virtual			void					release()	PX_OVERRIDE PX_FINAL;
	virtual			PxActorType::Enum		getType() const PX_OVERRIDE PX_FINAL	{ return PxActorType::eRIGID_STATIC; }
	//~PxActor

	// PxRigidActor
	virtual			void 					setGlobalPose(const PxTransform& pose, bool wake)	PX_OVERRIDE PX_FINAL;
	virtual			PxTransform				getGlobalPose() const	PX_OVERRIDE PX_FINAL;
	//~PxRigidActor

	// PT: I think these come from NpRigidActorTemplate
	// PT: TODO: drop them eventually, they all re-route to NpActor now
	virtual			void					switchToNoSim()	PX_OVERRIDE PX_FINAL;
	virtual			void					switchFromNoSim()	PX_OVERRIDE PX_FINAL;

#if PX_CHECKED
					bool					checkConstraintValidity() const;
#endif

	PX_FORCE_INLINE	const Sc::StaticCore&	getCore()				const	{ return mCore;	}
	PX_FORCE_INLINE	Sc::StaticCore&			getCore()						{ return mCore;	}

	static PX_FORCE_INLINE size_t			getCoreOffset()					{ return PX_OFFSET_OF_RT(NpRigidStatic, mCore);			}
	static PX_FORCE_INLINE size_t			getNpShapeManagerOffset()		{ return PX_OFFSET_OF_RT(NpRigidStatic, mShapeManager);	}

#if PX_ENABLE_DEBUG_VISUALIZATION
					void					visualize(PxRenderOutput& out, NpScene& scene, float scale)	const;
#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

private:
					Sc::StaticCore			mCore;
};

}

#endif
