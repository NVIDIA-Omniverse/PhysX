// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_DEBUG_VIZ_H
#define NP_DEBUG_VIZ_H

#include "common/PxPhysXCommonConfig.h"

#if PX_ENABLE_DEBUG_VISUALIZATION

namespace physx
{
	class PxRenderOutput;
	class NpScene;
	class PxRigidActor;

	namespace Sc
	{
		class BodyCore;
	}

	void visualizeRigidBody(PxRenderOutput& out, NpScene& scene, const PxRigidActor& actor, const Sc::BodyCore& mCore, float scale);
}

#else
	PX_CATCH_UNDEFINED_ENABLE_DEBUG_VISUALIZATION
#endif

#endif

