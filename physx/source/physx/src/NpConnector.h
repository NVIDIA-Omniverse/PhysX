// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_CONNECTOR_H
#define NP_CONNECTOR_H

#include "common/PxSerialFramework.h"
#include "foundation/PxInlineArray.h"
#include "foundation/PxUtilities.h"
#include "CmUtils.h"

namespace physx
{

struct NpConnectorType
{
	enum Enum
	{
		eConstraint,
		eAggregate,
		eObserver,
		eBvh,
		eAttachment,
		eElementFilter,
		eInvalid
	};
};

class NpConnector
{
public:
	NpConnector() : NpConnector(NpConnectorType::eInvalid, NULL) {}
	NpConnector(NpConnectorType::Enum type, PxBase* object) : mType(PxTo8(type)), mObject(object) 
	{
#if PX_CHECKED
		const PxU32 numPaddings = sizeof(mPadding) / sizeof(mPadding[0]);
		for(PxU32 i = 0; i < numPaddings; ++i)
		{
			mPadding[i] = PX_PADDING_8;
		}
#endif
	}
	// PX_SERIALIZATION
	NpConnector(const NpConnector& c)
	{
		//special copy constructor that initializes padding bytes for meta data verification (PX_CHECKED only)		
		PxMarkSerializedMemory(this, sizeof(NpConnector));
		mType = c.mType;
#if PX_CHECKED
		const PxU32 numPaddings = sizeof(mPadding) / sizeof(mPadding[0]);
		for(PxU32 i = 0; i < numPaddings; ++i)
		{
			mPadding[i] = c.mPadding[i];
		}
#endif
		mObject = c.mObject;
	}
	//~PX_SERIALIZATION

	PxU8			mType;			// Revisit whether the type is really necessary or whether the serializable type is enough.
									// Since joints might gonna inherit from observers to register for constraint release events, the type
									// is necessary because a joint has its own serializable type and could not be detected as observer anymore.
	PxU8			mPadding[3];	// PT: padding from prev byte
	PxBase*			mObject;		// So far the serialization framework only supports ptr resolve for PxBase objects.
									// However, so far the observers all are PxBase, hence this choice of type.
};

class NpConnectorIterator
{
public:
	PX_FORCE_INLINE NpConnectorIterator(NpConnector* c, PxU32 size, NpConnectorType::Enum type) : mConnectors(c), mSize(size), mIndex(0), mType(type) {}

	PX_FORCE_INLINE PxBase* getNext()
	{
		PxBase* s = NULL;
		while(mIndex < mSize)
		{
			NpConnector& c = mConnectors[mIndex];
			mIndex++;
			if (c.mType == mType)
				return c.mObject;
		}
		return s;
	}

private:
	NpConnector*			mConnectors;
	PxU32					mSize;
	PxU32					mIndex;
	NpConnectorType::Enum	mType;
};

class NpConnectorArray: public PxInlineArray<NpConnector, 4> 
{
public:
// PX_SERIALIZATION
	NpConnectorArray(const PxEMPTY) : PxInlineArray<NpConnector, 4> (PxEmpty) {}
//~PX_SERIALIZATION
	NpConnectorArray() : PxInlineArray<NpConnector, 4>("connectorArray") 
	{
		//special default constructor that initializes padding bytes for meta data verification (PX_CHECKED only)
		PxMarkSerializedMemory(this->mData, 4*sizeof(NpConnector));
	}
};

}

#endif
