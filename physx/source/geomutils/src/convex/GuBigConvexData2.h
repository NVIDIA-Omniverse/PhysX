// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef GU_BIG_CONVEX_DATA2_H
#define GU_BIG_CONVEX_DATA2_H

#include "foundation/PxVec3.h"
#include "common/PxPhysXCommonConfig.h"
#include "GuBigConvexData.h"

namespace physx
{
	class PxSerializationContext;
	class PxDeserializationContext;

	class PX_PHYSX_COMMON_API BigConvexData : public PxUserAllocated
	{
		public:
// PX_SERIALIZATION
											BigConvexData(const PxEMPTY)	{}
//~PX_SERIALIZATION
											BigConvexData();
											~BigConvexData();
		// Support vertex map
					bool					Load(PxInputStream& stream);

					PxU32					ComputeOffset(const PxVec3& dir)		const;
					PxU32					ComputeNearestOffset(const PxVec3& dir)	const;

		// Data access
		PX_INLINE	PxU32					GetSubdiv()								const	{ return mData.mSubdiv;											}
		PX_INLINE	PxU32					GetNbSamples()							const	{ return mData.mNbSamples;										}
		//~Support vertex map

		// Valencies
		// Data access
		PX_INLINE	PxU32					GetNbVerts()							const	{ return mData.mNbVerts;										}
		PX_INLINE	const Gu::Valency*		GetValencies()							const	{ return mData.mValencies;										}
		PX_INLINE	PxU16					GetValency(PxU32 i)						const	{ return mData.mValencies[i].mCount;							}
		PX_INLINE	PxU16					GetOffset(PxU32 i)						const	{ return mData.mValencies[i].mOffset;							}
		PX_INLINE	const PxU8*				GetAdjacentVerts()						const	{ return mData.mAdjacentVerts;									}

		PX_INLINE	PxU16					GetNbNeighbors(PxU32 i)					const	{ return mData.mValencies[i].mCount;							}
		PX_INLINE	const PxU8*				GetNeighbors(PxU32 i)					const	{ return &mData.mAdjacentVerts[mData.mValencies[i].mOffset];	}

// PX_SERIALIZATION
					void					exportExtraData(PxSerializationContext& stream);
					void					importExtraData(PxDeserializationContext& context);
//~PX_SERIALIZATION
					Gu::BigConvexRawData	mData;
		protected:
					void*					mVBuffer;
		// Internal methods
					void					CreateOffsets();
					bool					VLoad(PxInputStream& stream);
		//~Valencies
		friend class BigConvexDataBuilder;
	};

}

#endif

