// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

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
