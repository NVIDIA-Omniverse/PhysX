// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef SN_XML_SERIALIZER_H
#define SN_XML_SERIALIZER_H

#include "PxExtensionMetaDataObjects.h"
#include "SnXmlVisitorWriter.h"

namespace physx {

namespace Sn {
	
	void writeHeightFieldSample( PxOutputStream& inStream, const PxHeightFieldSample& inSample )
	{
		PxU32 retval = 0;
		PxU8* writePtr( reinterpret_cast< PxU8*>( &retval ) );
		const PxU8* inPtr( reinterpret_cast<const PxU8*>( &inSample ) );
		if ( isBigEndian() )
		{
			//Height field samples are a
			//16 bit integer followed by two bytes.
			//right now, data is 2 1 3 4
			//We need a 32 bit integer that 
			//when read in by a LE system is 4 3 2 1.
			//Thus, we need a BE integer that looks like:
			//4 3 2 1

			writePtr[0] = inPtr[3];
			writePtr[1] = inPtr[2];
			writePtr[2] = inPtr[0];
			writePtr[3] = inPtr[1];
		}
		else
		{
			writePtr[0] = inPtr[0];
			writePtr[1] = inPtr[1];
			writePtr[2] = inPtr[2];
			writePtr[3] = inPtr[3];
		}
		inStream << retval;
	}
	
	

	template<typename TDataType, typename TWriteOperator>
	inline void writeStridedBufferProperty( XmlWriter& writer, MemoryBuffer& tempBuffer, const char* inPropName, const void* inData, PxU32 inStride, PxU32 inCount, PxU32 inItemsPerLine, TWriteOperator inOperator )
	{
		PX_ASSERT( inStride == 0 || inStride == sizeof( TDataType ) );
		PX_UNUSED( inStride );
		writeBuffer( writer, tempBuffer
					, inItemsPerLine, reinterpret_cast<const TDataType*>( inData )
					, inCount, inPropName, inOperator );
	}

	template<typename TDataType, typename TWriteOperator>
	inline void writeStridedBufferProperty( XmlWriter& writer, MemoryBuffer& tempBuffer, const char* inPropName, const PxStridedData& inData, PxU32 inCount, PxU32 inItemsPerLine, TWriteOperator inOperator )
	{
		writeStridedBufferProperty<TDataType>( writer, tempBuffer, inPropName, inData.data, inData.stride, inCount, inItemsPerLine, inOperator );
	}
	
	template<typename TDataType, typename TWriteOperator>
	inline void writeStridedBufferProperty( XmlWriter& writer, MemoryBuffer& tempBuffer, const char* inPropName, const PxTypedBoundedData<TDataType>& inData, PxU32 inCount, PxU32 inItemsPerLine, TWriteOperator inOperator )
	{
		writeStridedBufferProperty<TDataType>( writer, tempBuffer, inPropName, inData.data, inData.stride, inCount, inItemsPerLine, inOperator );
	}
	
	template<typename TDataType, typename TWriteOperator>
	inline void writeStridedBufferProperty( XmlWriter& writer, MemoryBuffer& tempBuffer, const char* inPropName, const PxBoundedData& inData, PxU32 inItemsPerLine, TWriteOperator inWriteOperator )
	{
		writeStridedBufferProperty<TDataType>( writer, tempBuffer, inPropName, inData, inData.count, inItemsPerLine, inWriteOperator );
	}

	template<typename TDataType, typename TWriteOperator>
	inline void writeStridedBufferProperty( XmlWriter& writer, MemoryBuffer& tempBuffer, const char* inPropName, PxStrideIterator<const TDataType>& inData, PxU32 inCount, PxU32 inItemsPerLine, TWriteOperator inWriteOperator )
	{
		writeStrideBuffer<TDataType>(writer, tempBuffer
					, inItemsPerLine, inData, PtrAccess<TDataType>
					, inCount, inPropName, inData.stride(), inWriteOperator );
	}

	template<typename TDataType>
	inline void writeStridedFlagsProperty( XmlWriter& writer, MemoryBuffer& tempBuffer, const char* inPropName, PxStrideIterator<const TDataType>& inData, PxU32 inCount, PxU32 inItemsPerLine, const PxU32ToName* inTable )
	{
		writeStrideFlags<TDataType>(writer, tempBuffer
					, inItemsPerLine, inData, PtrAccess<TDataType>
					, inCount, inPropName, inTable );
	}
	
}
}
#endif
