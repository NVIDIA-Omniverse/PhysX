// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SC_TRIGGER_PAIRS_H
#define SC_TRIGGER_PAIRS_H

#include "foundation/PxArray.h"
#include "PxFiltering.h"
#include "PxClient.h"
#include "PxSimulationEventCallback.h"

namespace physx
{
class PxShape;

namespace Sc
{
	struct TriggerPairFlag
	{
		enum Enum
		{
			eTEST_FOR_REMOVED_SHAPES = PxTriggerPairFlag::eNEXT_FREE	// for cases where the pair got deleted because one of the shape volumes got removed from broadphase.
																		// This covers scenarios like volume re-insertion into broadphase as well since the shape might get removed
																		// after such an operation. The scenarios to consider are:
																		//
																		// - shape gets removed (this includes raising PxActorFlag::eDISABLE_SIMULATION)
																		// - shape switches to eSCENE_QUERY_SHAPE only
																		// - shape switches to eSIMULATION_SHAPE
																		// - resetFiltering()
																		// - actor gets removed from an aggregate
		};
	};

	PX_COMPILE_TIME_ASSERT((1 << (8*sizeof(PxTriggerPairFlags::InternalType))) > TriggerPairFlag::eTEST_FOR_REMOVED_SHAPES);

	struct TriggerPairExtraData
	{
		PX_INLINE TriggerPairExtraData() : 
			shape0ID(0xffffffff),
			shape1ID(0xffffffff),
			client0ID(0xff),
			client1ID(0xff)
		{
		}

		PX_INLINE TriggerPairExtraData(PxU32 s0ID, PxU32 s1ID,
										PxClientID cl0ID, PxClientID cl1ID) : 
			shape0ID(s0ID),
			shape1ID(s1ID),
			client0ID(cl0ID),
			client1ID(cl1ID)
		{
		}

		PxU32						shape0ID;
		PxU32						shape1ID;
		PxClientID					client0ID;
		PxClientID					client1ID;
	};

	typedef	PxArray<TriggerPairExtraData>	TriggerBufferExtraData;
	typedef	PxArray<PxTriggerPair>		TriggerBufferAPI;

} // namespace Sc

}

#endif
