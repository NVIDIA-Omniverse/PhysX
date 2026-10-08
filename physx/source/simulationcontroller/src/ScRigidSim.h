// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_RIGID_SIM_H
#define SC_RIGID_SIM_H

#include "ScActorSim.h"
#include "ScRigidCore.h"

namespace physx
{
namespace Sc
{
	class Scene;

	class RigidSim : public ActorSim
	{
	public:
									RigidSim(Scene&, RigidCore&);
		virtual						~RigidSim();

		PX_FORCE_INLINE	RigidCore&	getRigidCore()	const	{ return static_cast<RigidCore&>(mCore);	}

						void		notifyShapesOfTransformChange();

		virtual			PxActor*	getPxActor() const PX_OVERRIDE { return getRigidCore().getPxActor(); }
	};

} // namespace Sc

}

#endif
