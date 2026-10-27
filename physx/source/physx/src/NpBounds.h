// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_BOUNDS_H
#define NP_BOUNDS_H

namespace physx
{
	class PxBounds3;
	class NpShape;
	class NpActor;

namespace Sq
{
	typedef void(*ComputeBoundsFunc)	(PxBounds3& bounds, const NpShape& scShape, const NpActor& npActor);

	extern const ComputeBoundsFunc gComputeBoundsTable[2];

//	#define SQ_PRUNER_EPSILON	0.01f
	#define SQ_PRUNER_EPSILON	0.005f
	#define SQ_PRUNER_INFLATION	(1.0f + SQ_PRUNER_EPSILON)	// pruner test shape inflation (not narrow phase shape)
}
}

#endif
