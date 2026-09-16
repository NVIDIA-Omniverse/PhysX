// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PVD_META_DATA_BINDING_DATA_H
#define PVD_META_DATA_BINDING_DATA_H

#if PX_SUPPORT_PVD
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxHashSet.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxArray.h"


namespace physx
{
namespace Vd
{

typedef PxHashSet<const PxRigidActor*> OwnerActorsValueType;
typedef PxHashMap<const PxShape*, OwnerActorsValueType*> OwnerActorsMap;

struct PvdMetaDataBindingData : public PxUserAllocated
{
	PxArray<PxU8> mTempU8Array;
	PxArray<PxActor*> mActors;
	PxArray<PxArticulationReducedCoordinate*> mArticulations;
	PxArray<PxArticulationLink*> mArticulationLinks;
	PxHashSet<PxActor*> mSleepingActors;
	OwnerActorsMap mOwnerActorsMap;

	PvdMetaDataBindingData()
	: mTempU8Array("TempU8Array")
	, mActors("PxActor")
	, mArticulations("Articulations")
	, mArticulationLinks("ArticulationLinks")
	, mSleepingActors("SleepingActors")
	{
	}

	template <typename TDataType>
	TDataType* allocateTemp(PxU32 numItems)
	{
		mTempU8Array.resize(numItems * sizeof(TDataType));
		if(numItems)
			return reinterpret_cast<TDataType*>(mTempU8Array.begin());
		else
			return NULL;
	}

	DataRef<const PxU8> tempToRef()
	{
		return DataRef<const PxU8>(mTempU8Array.begin(), mTempU8Array.size());
	}
};
}
}
#endif // PX_SUPPORT_PVD
#endif
