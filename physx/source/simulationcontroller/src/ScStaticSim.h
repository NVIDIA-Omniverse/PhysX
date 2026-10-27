// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_STATIC_SIM_H
#define SC_STATIC_SIM_H

#include "ScRigidSim.h"
#include "ScStaticCore.h"

namespace physx
{
namespace Sc
{
	class StaticSim : public RigidSim
	{
	public:
									StaticSim(Scene& scene, StaticCore& core) : RigidSim(scene, core)	{}
									~StaticSim()														{ getStaticCore().setSim(NULL);	}

		PX_FORCE_INLINE	StaticCore&	getStaticCore()		const											{ return static_cast<StaticCore&>(getRigidCore());	}
	};

} // namespace Sc

}

#endif
