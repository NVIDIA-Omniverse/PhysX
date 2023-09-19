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
// Copyright (c) 2008-2023 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#include "common/PxSerializer.h"
#include "foundation/PxPhysicsVersion.h"
#include "foundation/PxUtilities.h"
#include "foundation/PxSort.h"
#include "extensions/PxBinaryConverter.h"
#include "SnSerializationContext.h"
#include "serialization/SnSerialUtils.h"
#include "serialization/SnSerializationRegistry.h"

using namespace physx;
using namespace Cm;
using namespace Sn;

//------------------------------------------------------------------------------------
//// Binary Serialized PxCollection, format documentation
//------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------
//// overview:
//// header information
//// manifest table
//// import references
//// export references
//// internal references
//// object data
//// extra data
//------------------------------------------------------------------------------------
//
//
//------------------------------------------------------------------------------------
//// header information:
//// header tag plus various version and platform information
//------------------------------------------------------------------------------------
// header SEBD
// PX_PHYSICS_VERSION
// PX_BINARY_SERIAL_VERSION
// platform tag
// markedPadding (on for PX_CHECKED)
// nbObjectsInCollection
//
//
//------------------------------------------------------------------------------------
//// manifest table:
//// one entry per collected object
//// offsets relative to object data buffer
//------------------------------------------------------------------------------------
// alignment
// PxU32 size
// (PxU32 offset, PxType type)*size
// PxU32 endOffset
//
//
//------------------------------------------------------------------------------------
//// import references:
//// one entry per required reference to external collection
//------------------------------------------------------------------------------------
// alignment 
// PxU32 size
// (PxSerialObjectId id, PxType type)*size
//
//
//------------------------------------------------------------------------------------
//// export references:
//// one entry per object in the collection with id
//// object indices point into the manifest table (objects in the same collection)
//------------------------------------------------------------------------------------
// alignment 
// PxU32 size
// (PxSerialObjectId id, SerialObjectIndex objIndex)*size
//
//
//------------------------------------------------------------------------------------
//// internal references:
//// one entry per reference, kind pair
//// object indices point either into the manifest table or into the import references
//// depending on whether the entry references the same collection or the external one
//// one section for pointer type references and one for index type references.
//------------------------------------------------------------------------------------
// alignment 
// PxU32 sizePtrs;
// (size_t reference, PxU32 kind, SerialObjectIndex objIndex)*sizePtrs
// PxU32 sizeHandle16; 
// (PxU16 reference, PxU32 kind, SerialObjectIndex objIndex)*sizeHandle16
//
//
//------------------------------------------------------------------------------------
//// object data:
//// serialized PxBase derived class instances
//// each object size depends on specific class
//// offsets are stored in manifest table
//------------------------------------------------------------------------------------
// alignment
// (PxConcreteType type, -----)
// alignment
// (PxConcreteType type, --------)
// alignment
// (PxConcreteType type, --)
// .
// .
// 
//
// -----------------------------------------------------------------------------------
//// extra data:
//// extra data memory block
//// serialized and deserialized by PxBase implementations 
////----------------------------------------------------------------------------------
// extra data
//
//------------------------------------------------------------------------------------

namespace
{
	class OutputStreamWriter
	{
	public:

		PX_INLINE OutputStreamWriter(PxOutputStream& stream) 
		:	mStream(stream)
		,	mCount(0)
		{}

		PX_INLINE	PxU32	write(const void* src, PxU32 offset)		
		{		
			PxU32 count = mStream.write(src, offset);
			mCount += count;
			return count;
		}

		PX_INLINE	PxU32	getStoredSize()
		{
			return mCount;
		}

	private:

		OutputStreamWriter& operator=(const OutputStreamWriter&);
		PxOutputStream& mStream;
		PxU32 mCount;
	};

	class LegacySerialStream : public PxSerializationContext
	{
	public:
		LegacySerialStream(OutputStreamWriter& writer, 
						   const PxCollection& collection,
						   bool exportNames) : mWriter(writer), mCollection(collection), mExportNames(exportNames) {}
		void		writeData(const void* buffer, PxU32 size)	{ mWriter.write(buffer, size);		}
		PxU32		getTotalStoredSize()						{ return mWriter.getStoredSize();	}
		void		alignData(PxU32 alignment) 
		{ 
			if(!alignment)
				return;

			PxI32 bytesToPad = PxI32(getPadding(getTotalStoredSize(), alignment));
			static const PxI32 BUFSIZE = 64;
			char buf[BUFSIZE];
			PxMemSet(buf, 0, bytesToPad < BUFSIZE ? PxU32(bytesToPad) : PxU32(BUFSIZE));
			while(bytesToPad > 0)
			{
				writeData(buf, bytesToPad < BUFSIZE ? PxU32(bytesToPad) : PxU32(BUFSIZE));
				bytesToPad -= BUFSIZE;
			}
			PX_ASSERT(!getPadding(getTotalStoredSize(), alignment));
		}

		virtual void	registerReference(PxBase&, PxU32, size_t)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_OPERATION, PX_FL, 
					"Cannot register references during exportData, exportExtraData.");
		}

		virtual const PxCollection& getCollection() const
		{
			return mCollection;
		}
		virtual void writeName(const char* name)
		{
			PxU32 len = name && mExportNames ? PxU32(strlen(name)) + 1 : 0;
			writeData(&len, sizeof(len));
			if(len) writeData(name, len);
		}

	private:
		LegacySerialStream& operator=(const LegacySerialStream&);
		OutputStreamWriter& mWriter;
		const PxCollection& mCollection;
		bool mExportNames;
	};

	void writeHeader(PxSerializationContext& stream, bool hasDeserializedAssets)
	{
		PX_UNUSED(hasDeserializedAssets);

		//serialized binary data.
		const PxU32 header = PX_MAKE_FOURCC('S','E','B','D');
		stream.writeData(&header, sizeof(PxU32));

		PxU32 version = PX_PHYSICS_VERSION;
		stream.writeData(&version, sizeof(PxU32));

		stream.writeData(PX_BINARY_SERIAL_VERSION, SN_BINARY_VERSION_GUID_NUM_CHARS);

		PxU32 platformTag = getBinaryPlatformTag();
		stream.writeData(&platformTag, sizeof(PxU32));

	    PxU32 markedPadding = 0;
#if PX_CHECKED
		if(!hasDeserializedAssets) 
			markedPadding = 1;
#endif
		stream.writeData(&markedPadding, sizeof(PxU32));
	}

	template<typename InternalReferenceType>
	struct InternalReferencePredicate
	{
		PX_FORCE_INLINE bool operator()(InternalReferenceType& a, InternalReferenceType& b) const { return a.objIndex < b.objIndex; }
	};

}

bool PxSerialization::serializeCollectionToBinary(PxOutputStream& outputStream, PxCollection& pxCollection, PxSerializationRegistry& sr, const PxCollection* pxExternalRefs, bool exportNames)
{
	if(!PxSerialization::isSerializable(pxCollection, sr, pxExternalRefs))
		return false; 
		
	Collection& collection = static_cast<Collection&>(pxCollection);
	const Collection* externalRefs = static_cast<const Collection*>(pxExternalRefs);
	
	//temporary memory stream which allows fixing up data up stream
	
	SerializationRegistry& sn = static_cast<SerializationRegistry&>(sr);
	
	// sort collection by "order" value (this will be the order in which they get serialized)
	sortCollection(collection, sn, false);

	//initialized the context with the sorted collection. 
	SerializationContext context(collection, externalRefs);
	
	// gather reference information
    bool hasDeserializedAssets = false;
	{
		const PxU32 nb = collection.internalGetNbObjects();
		for(PxU32 i=0;i<nb;i++)
		{
			PxBase* s = collection.internalGetObject(i);
			PX_ASSERT(s && s->getConcreteType());
#if PX_CHECKED
			//can't guarantee marked padding for deserialized instances
			if(!(s->getBaseFlags() & PxBaseFlag::eOWNS_MEMORY))
			   hasDeserializedAssets = true;
#endif
			const PxSerializer* serializer = sn.getSerializer(s->getConcreteType());
			PX_ASSERT(serializer);
			serializer->registerReferences(*s, context);
		}
	}

	// now start the actual serialization into the output stream
	OutputStreamWriter writer(outputStream);
	LegacySerialStream stream(writer, collection, exportNames);
	
	writeHeader(stream, hasDeserializedAssets);

	// write size of collection
	stream.alignData(PX_SERIAL_ALIGN);
	PxU32 nbObjectsInCollection = collection.internalGetNbObjects();
	stream.writeData(&nbObjectsInCollection, sizeof(PxU32));

	// write the manifest table (PxU32 offset, PxConcreteType type)
	{
		PxArray<ManifestEntry> manifestTable(collection.internalGetNbObjects());
		PxU32 headerOffset = 0;
		for(PxU32 i=0;i<collection.internalGetNbObjects();i++)
		{
			PxBase* s = collection.internalGetObject(i);
			PX_ASSERT(s && s->getConcreteType());
			PxType concreteType = s->getConcreteType();
			const PxSerializer* serializer = sn.getSerializer(concreteType);
			PX_ASSERT(serializer);
			manifestTable[i] = ManifestEntry(headerOffset, concreteType);
			PxU32 classSize = PxU32(serializer->getClassSize());
			headerOffset += getPadding(classSize, PX_SERIAL_ALIGN) + classSize;
		}
		stream.alignData(PX_SERIAL_ALIGN);
		const PxU32 nb = manifestTable.size();
		stream.writeData(&nb, sizeof(PxU32));
		stream.writeData(manifestTable.begin(), manifestTable.size()*sizeof(ManifestEntry));
			
		//store offset for end of object buffer (PxU32 offset)
		stream.writeData(&headerOffset, sizeof(PxU32));
	}

	// write import references
	{
		const PxArray<ImportReference>& importReferences = context.getImportReferences();
		stream.alignData(PX_SERIAL_ALIGN);
		const PxU32 nb = importReferences.size();
		stream.writeData(&nb, sizeof(PxU32));
		stream.writeData(importReferences.begin(), importReferences.size()*sizeof(ImportReference));
	}

	// write export references
	{
		PxU32 nbIds = collection.getNbIds();
		PxArray<ExportReference> exportReferences(nbIds);
		//we can't get quickly from id to object index in collection. 
		//if we only need this here, its not worth to build a hash
		nbIds = 0;
		for (PxU32 i=0;i<collection.getNbObjects();i++)
		{
			PxBase& obj = collection.getObject(i);
			PxSerialObjectId id = collection.getId(obj);
			if (id != PX_SERIAL_OBJECT_ID_INVALID)
			{
				SerialObjectIndex objIndex(i, false); //i corresponds to manifest entry
				exportReferences[nbIds++] = ExportReference(id, objIndex);
			}
		}
		stream.alignData(PX_SERIAL_ALIGN);
		stream.writeData(&nbIds, sizeof(PxU32));
		stream.writeData(exportReferences.begin(), exportReferences.size()*sizeof(ExportReference));
	}

	// write internal references
	{
		InternalPtrRefMap& internalPtrReferencesMap = context.getInternalPtrReferencesMap();
		PxArray<InternalReferencePtr> internalReferencesPtr(internalPtrReferencesMap.size());
		PxU32 nbInternalPtrReferences = 0;

		InternalHandle16RefMap& internalHandle16ReferencesMap = context.getInternalHandle16ReferencesMap();
		PxArray<InternalReferenceHandle16> internalReferencesHandle16(internalHandle16ReferencesMap.size());
		PxU32 nbInternalHandle16References = 0;

		{
			for(InternalPtrRefMap::Iterator iter = internalPtrReferencesMap.getIterator(); !iter.done(); ++iter)
				internalReferencesPtr[nbInternalPtrReferences++] = InternalReferencePtr(iter->first, iter->second);

			for(InternalHandle16RefMap::Iterator iter = internalHandle16ReferencesMap.getIterator(); !iter.done(); ++iter)
				internalReferencesHandle16[nbInternalHandle16References++] = InternalReferenceHandle16(PxTo16(iter->first), iter->second);

			//sort InternalReferences according to SerialObjectIndex for determinism
			PxSort<InternalReferencePtr, InternalReferencePredicate<InternalReferencePtr> >(internalReferencesPtr.begin(), internalReferencesPtr.size(), InternalReferencePredicate<InternalReferencePtr>());
			PxSort<InternalReferenceHandle16, InternalReferencePredicate<InternalReferenceHandle16> >(internalReferencesHandle16.begin(), internalReferencesHandle16.size(), InternalReferencePredicate<InternalReferenceHandle16>());
		}

		stream.alignData(PX_SERIAL_ALIGN);
		
		stream.writeData(&nbInternalPtrReferences, sizeof(PxU32));
		stream.writeData(internalReferencesPtr.begin(), internalReferencesPtr.size()*sizeof(InternalReferencePtr));

		stream.writeData(&nbInternalHandle16References, sizeof(PxU32));
		stream.writeData(internalReferencesHandle16.begin(), internalReferencesHandle16.size()*sizeof(InternalReferenceHandle16));
	}

	// write object data
	{
		stream.alignData(PX_SERIAL_ALIGN);
		const PxU32 nb = collection.internalGetNbObjects();
		for(PxU32 i=0;i<nb;i++)
		{
			PxBase* s = collection.internalGetObject(i);
			PX_ASSERT(s && s->getConcreteType());
			const PxSerializer* serializer = sn.getSerializer(s->getConcreteType());
			PX_ASSERT(serializer);
			stream.alignData(PX_SERIAL_ALIGN);
			serializer->exportData(*s, stream);
		}
	}

	// write extra data
	{
		const PxU32 nb = collection.internalGetNbObjects();
		for(PxU32 i=0;i<nb;i++)
		{
			PxBase* s = collection.internalGetObject(i);
			PX_ASSERT(s && s->getConcreteType());

			const PxSerializer* serializer = sn.getSerializer(s->getConcreteType());
			PX_ASSERT(serializer);

			stream.alignData(PX_SERIAL_ALIGN);
			serializer->exportExtraData(*s, stream);
		}
	}

	return true;
}

bool PxSerialization::serializeCollectionToBinaryDeterministic(PxOutputStream& outputStream, PxCollection& pxCollection, PxSerializationRegistry& sr, const PxCollection* pxExternalRefs, bool exportNames)
{
	PxDefaultMemoryOutputStream tmpOutputStream;
	if (!serializeCollectionToBinary(tmpOutputStream, pxCollection, sr, pxExternalRefs, exportNames))
		return false;

	PxDefaultMemoryOutputStream metaDataOutput;
	dumpBinaryMetaData(metaDataOutput, sr);

	PxBinaryConverter* converter = createBinaryConverter();
	PxDefaultMemoryInputData srcMetaData(metaDataOutput.getData(), metaDataOutput.getSize());
	PxDefaultMemoryInputData dstMetaData(metaDataOutput.getData(), metaDataOutput.getSize());
	
	if (!converter->setMetaData(srcMetaData, dstMetaData))
		return false;
	
	PxDefaultMemoryInputData srcBinaryData(tmpOutputStream.getData(), tmpOutputStream.getSize());
	bool ret = converter->convert(srcBinaryData, srcBinaryData.getLength(), outputStream);
	converter->release();

	return ret;
}

