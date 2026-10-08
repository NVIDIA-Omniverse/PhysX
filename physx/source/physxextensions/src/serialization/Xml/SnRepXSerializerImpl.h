// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_REPX_SERIALIZER_IMPL_H
#define SN_REPX_SERIALIZER_IMPL_H

#include "foundation/PxUserAllocated.h"
#include "SnXmlVisitorWriter.h"
#include "SnXmlVisitorReader.h"

namespace physx { 
	using namespace Sn;

	/**
	 *	The repx serializer impl takes the raw, untyped repx extension interface
	 *	and implements the simpler functions plus does the reinterpret-casts required 
	 *	for any object to implement the serializer safely.
	 */
	template<typename TLiveType>
	struct RepXSerializerImpl : public PxRepXSerializer, PxUserAllocated
	{
	protected:
		RepXSerializerImpl( const RepXSerializerImpl& inOther );
		RepXSerializerImpl& operator=( const RepXSerializerImpl& inOther );

	public:
		PxAllocatorCallback& mAllocator;

		RepXSerializerImpl( PxAllocatorCallback& inAllocator )
			: mAllocator( inAllocator )
		{
		}
				
		virtual const char* getTypeName() PX_OVERRIDE { return PxTypeInfo<TLiveType>::name(); }
		
		virtual void objectToFile( const PxRepXObject& inLiveObject, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& inArgs ) PX_OVERRIDE
		{
			const TLiveType* theObj = reinterpret_cast<const TLiveType*>( inLiveObject.serializable );
			objectToFileImpl( theObj, inCollection, inWriter, inTempBuffer, inArgs );
		}

		virtual PxRepXObject fileToObject( XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection ) PX_OVERRIDE
		{
			TLiveType* theObj( allocateObject( inArgs ) );
			if ( theObj )
				if(fileToObjectImpl( theObj, inReader, inAllocator, inArgs, inCollection ))
					return PxCreateRepXObject(theObj);
			return PxRepXObject();
		}
		
		virtual void objectToFileImpl( const TLiveType* inObj, PxCollection* inCollection, XmlWriter& inWriter, MemoryBuffer& inTempBuffer, PxRepXInstantiationArgs& /*inArgs*/)
		{
			writeAllProperties( inObj, inWriter, inTempBuffer, *inCollection );
		}

		virtual bool fileToObjectImpl( TLiveType* inObj, XmlReader& inReader, XmlMemoryAllocator& inAllocator, PxRepXInstantiationArgs& inArgs, PxCollection* inCollection )
		{
			return readAllProperties( inArgs, inReader, inObj, inAllocator, *inCollection );
		}

		virtual TLiveType* allocateObject( PxRepXInstantiationArgs& inArgs ) = 0;
	};
}

#endif
