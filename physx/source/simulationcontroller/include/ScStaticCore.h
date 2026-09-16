// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_STATIC_CORE_H
#define SC_STATIC_CORE_H

#include "ScRigidCore.h"
#include "PxvDynamics.h"

namespace physx
{
namespace Sc
{
	class StaticSim;

	class StaticCore : public RigidCore
	{
	public:
											StaticCore(const PxTransform& actor2World): RigidCore(PxActorType::eRIGID_STATIC)	
											{
												mCore.body2World = actor2World;
												mCore.mFlags = PxRigidBodyFlags();
											}
							
		PX_FORCE_INLINE	const PxTransform&	getActor2World() const	{ return mCore.body2World;	}
						void				setActor2World(const PxTransform& actor2World);

		PX_FORCE_INLINE	PxsRigidCore&		getCore()				{ return mCore;								}
		static PX_FORCE_INLINE size_t		getCoreOffset()			{ return PX_OFFSET_OF_RT(StaticCore, mCore);}

											StaticCore(const PxEMPTY) :	RigidCore(PxEmpty), mCore(PxEmpty) {}

						StaticSim*			getSim() const;

		PX_FORCE_INLINE	void				onOriginShift(const PxVec3& shift)	{ mCore.body2World.p -= shift; }
	
	private:
						PxsRigidCore		mCore;
	};

} // namespace Sc

}

#endif
