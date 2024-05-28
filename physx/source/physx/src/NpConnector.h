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
		eInvalid
	};
};

class NpConnector
{
public:
	NpConnector() : mType(NpConnectorType::eInvalid), mObject(NULL) {}
	NpConnector(NpConnectorType::Enum type, PxBase* object) : mType(PxTo8(type)), mObject(object) {}
// PX_SERIALIZATION
	NpConnector(const NpConnector& c)
	{
		//special copy constructor that initializes padding bytes for meta data verification (PX_CHECKED only)		
		PxMarkSerializedMemory(this, sizeof(NpConnector));
		mType = c.mType;
		mObject = c.mObject;
	}

	static	void	getBinaryMetaData(PxOutputStream& stream);
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
	static	void	getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
	NpConnectorArray() : PxInlineArray<NpConnector, 4>("connectorArray") 
	{
		//special default constructor that initializes padding bytes for meta data verification (PX_CHECKED only)
		PxMarkSerializedMemory(this->mData, 4*sizeof(NpConnector));
	}
};

}

#endif
