// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_SHAPE_SIM_H
#define SC_SHAPE_SIM_H

#include "ScShapeSimBase.h"

namespace physx
{

/** Simulation object corresponding to a shape core object. This object is created when
    a ShapeCore object is added to the simulation, and destroyed when it is removed
*/

namespace Sc
{
	class ActorSim;
	class ShapeCore;

	class ShapeSim : public ShapeSimBase
	{
		PX_NOCOPY(ShapeSim)
		public:
				ShapeSim(ActorSim&, ShapeCore& core);
				~ShapeSim();
	};

} // namespace Sc

}

#endif
