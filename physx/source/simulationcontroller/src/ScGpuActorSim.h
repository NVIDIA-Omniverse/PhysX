// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_GPU_ACTOR_SIM_H
#define SC_GPU_ACTOR_SIM_H

#include "ScActorSim.h"
#include "ScShapeSimBase.h"

namespace physx
{
namespace Sc
{
	class GPUActorSim : public ActorSim
	{
		public:
		ShapeSimBase	mShapeSim;

		GPUActorSim(Scene& scene, ActorCore& core, const ShapeCore* shapeCore);
		virtual	~GPUActorSim();

		const ShapeSimBase&	getShapeSim() const	 { return mShapeSim; }
		ShapeSimBase&		getShapeSim()		 { return mShapeSim; }

		void	addToAABBMgr(Bp::FilterType::Enum type);
		void	destroyLowLevelVolume();
	};
}
}

#endif
