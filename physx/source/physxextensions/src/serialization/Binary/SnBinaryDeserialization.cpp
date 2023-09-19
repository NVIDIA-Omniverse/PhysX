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
#include "foundation/PxHash.h"
#include "foundation/PxHashMap.h"
#include "foundation/PxString.h"
#include "extensions/PxSerialization.h"
#include "PxPhysics.h"
#include "PxPhysicsSerialization.h"

#include "SnFile.h"
#include "SnSerializationContext.h"
#include "SnConvX_Align.h"
#include "serialization/SnSerializationRegistry.h"
#include "serialization/SnSerialUtils.h"
#include "CmCollection.h"

using namespace physx;
using namespace Sn;

namespace
{
	PX_INLINE PxU8* alignPtr(PxU8* ptr, PxU32 alignment = PX_SERIAL_ALIGN)
	{
		if(!alignment)
			return ptr;

		const PxU32 padding = getPadding(size_t(ptr), alignment);
		PX_ASSERT(!getPadding(size_t(ptr + padding), alignment));
		return ptr + padding;
	}

	PX_FORCE_INLINE PxU32 read32(PxU8*& address)
	{
		const PxU32 value = *reinterpret_cast<PxU32*>(address);
		address += sizeof(PxU32);
		return value;
	}

	bool readHeader(PxU8*& address)
	{
		const PxU32 header = read32(address);
		PX_UNUSED(header);
		
		const PxU32 version = read32(address);
		PX_UNUSED(version);

		char binaryVersionGuid[SN_BINARY_VERSION_GUID_NUM_CHARS + 1];
		PxMemCopy(binaryVersionGuid, address, SN_BINARY_VERSION_GUID_NUM_CHARS);
		binaryVersionGuid[SN_BINARY_VERSION_GUID_NUM_CHARS] = 0;
		address += SN_BINARY_VERSION_GUID_NUM_CHARS;
		PX_UNUSED(binaryVersionGuid);

		const PxU32 platformTag = read32(address);
		PX_UNUSED(platformTag);

		const PxU32 markedPadding = read32(address);
		PX_UNUSED(markedPadding);

		if (header != PX_MAKE_FOURCC('S','E','B','D'))
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, 
				"Buffer contains data with wrong header indicating invalid binary data.");
			return false;
		}

		if (!checkCompatibility(binaryVersionGuid))
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, 
				"Buffer contains binary data version 0x%s and is incompatible with this PhysX sdk (0x%s).\n", 
				binaryVersionGuid, getBinaryVersionGuid());
			return false;
		}

		if (platformTag != getBinaryPlatformTag())
		{
			PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, 
				"Buffer contains data with platform mismatch:\nExpected: %s \nActual: %s\n",
				getBinaryPlatformName(getBinaryPlatformTag()),
				getBinaryPlatformName(platformTag));
			return false;
		}

		return true;
	}

	bool checkImportReferences(const ImportReference* importReferences, PxU32 nbImportReferences, const Cm::Collection* externalRefs)
	{
		if (!externalRefs)
		{
			if (nbImportReferences > 0)
			{
				PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxSerialization::createCollectionFromBinary: External references needed but no externalRefs collection specified.");
				return false;			
			}
		}
		else
		{
			for (PxU32 i=0; i<nbImportReferences;i++)
			{
				PxSerialObjectId id = importReferences[i].id;
				PxType type = importReferences[i].type;

				PxBase* referencedObject = externalRefs->find(id);
				if (!referencedObject)
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxSerialization::createCollectionFromBinary: External reference %" PX_PRIu64 " expected in externalRefs collection but not found.", id);
					return false;
				}
				if (referencedObject->getConcreteType() != type)
				{
					PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxSerialization::createCollectionFromBinary: External reference %d type mismatch. Expected %d but found %d in externalRefs collection.", type, referencedObject->getConcreteType());
					return false;
				}
			}
		}
		return true;
	}
	
}

PxCollection* PxSerialization::createCollectionFromBinary(void* memBlock, PxSerializationRegistry& sr, const PxCollection* pxExternalRefs)
{
	if(size_t(memBlock) & (PX_SERIAL_FILE_ALIGN-1))
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "Buffer must be 128-bytes aligned.");
		return NULL;
	}

	PxU8* address = reinterpret_cast<PxU8*>(memBlock);
	const Cm::Collection* externalRefs = static_cast<const Cm::Collection*>(pxExternalRefs);
			
	if (!readHeader(address))
	{
		return NULL;
	}

	ManifestEntry* manifestTable;
	PxU32 nbManifestEntries;
	PxU32 nbObjectsInCollection;
	PxU32 objectDataEndOffset;

	// read number of objects in collection
	address = alignPtr(address);
	nbObjectsInCollection = read32(address);

	// read manifest (PxU32 offset, PxConcreteType type)
	{
		address = alignPtr(address);
		nbManifestEntries = read32(address);
		PX_ASSERT(*reinterpret_cast<PxU32*>(address) == 0); //first offset is always 0
		manifestTable = (nbManifestEntries > 0) ? reinterpret_cast<ManifestEntry*>(address) : NULL;
		address += nbManifestEntries*sizeof(ManifestEntry);
		objectDataEndOffset = read32(address);
	}

	ImportReference* importReferences;
	PxU32 nbImportReferences;
	// read import references
	{
		address = alignPtr(address);
		nbImportReferences = read32(address);
		importReferences = (nbImportReferences > 0) ? reinterpret_cast<ImportReference*>(address) : NULL;
		address += nbImportReferences*sizeof(ImportReference);
	}

	if (!checkImportReferences(importReferences, nbImportReferences, externalRefs))
	{
		return NULL;
	}

	ExportReference* exportReferences;
	PxU32 nbExportReferences;
	// read export references
	{
		address = alignPtr(address);
		nbExportReferences = read32(address);
		exportReferences = (nbExportReferences > 0) ? reinterpret_cast<ExportReference*>(address) : NULL;
		address += nbExportReferences*sizeof(ExportReference);
	}

	// read internal references arrays
	PxU32 nbInternalPtrReferences = 0;
	PxU32 nbInternalHandle16References = 0;
	InternalReferencePtr* internalPtrReferences = NULL;
	InternalReferenceHandle16* internalHandle16References = NULL;
	{
		address = alignPtr(address);

		nbInternalPtrReferences = read32(address);
		internalPtrReferences = (nbInternalPtrReferences > 0) ? reinterpret_cast<InternalReferencePtr*>(address) : NULL;
		address += nbInternalPtrReferences*sizeof(InternalReferencePtr);

		nbInternalHandle16References = read32(address);
		internalHandle16References = (nbInternalHandle16References > 0) ? reinterpret_cast<InternalReferenceHandle16*>(address) : NULL;
		address += nbInternalHandle16References*sizeof(InternalReferenceHandle16);
	}

	// create internal references map
	InternalPtrRefMap internalPtrReferencesMap(nbInternalPtrReferences*2);
	{
		//create hash (we should load the hashes directly from memory)
		for (PxU32 i = 0; i < nbInternalPtrReferences; i++)
		{
			const InternalReferencePtr& ref = internalPtrReferences[i];
			internalPtrReferencesMap.insertUnique(ref.reference, SerialObjectIndex(ref.objIndex));
		}
	}
	InternalHandle16RefMap internalHandle16ReferencesMap(nbInternalHandle16References*2);
	{
		for (PxU32 i=0;i<nbInternalHandle16References;i++)
		{
			const InternalReferenceHandle16& ref = internalHandle16References[i];
			internalHandle16ReferencesMap.insertUnique(ref.reference, SerialObjectIndex(ref.objIndex));
		}
	}

	SerializationRegistry& sn = static_cast<SerializationRegistry&>(sr);
	Cm::Collection* collection = static_cast<Cm::Collection*>(PxCreateCollection());
	PX_ASSERT(collection);
	collection->mObjects.reserve(nbObjectsInCollection*2);
	if(nbExportReferences > 0)
	    collection->mIds.reserve(nbExportReferences*2);

	PxU8* addressObjectData = alignPtr(address);
	PxU8* addressExtraData = alignPtr(addressObjectData + objectDataEndOffset);

	DeserializationContext context(manifestTable, importReferences, addressObjectData, internalPtrReferencesMap, internalHandle16ReferencesMap, externalRefs, addressExtraData);
	
	// iterate over memory containing PxBase objects, create the instances, resolve the addresses, import the external data, add to collection.
	{
		PxU32 nbObjects = nbObjectsInCollection;

		while(nbObjects--)
		{
			address = alignPtr(address);
			context.alignExtraData();

			// read PxBase header with type and get corresponding serializer.
			PxBase* header = reinterpret_cast<PxBase*>(address);
			const PxType classType = header->getConcreteType();
			const PxSerializer* serializer = sn.getSerializer(classType);
			PX_ASSERT(serializer);

			PxBase* instance = serializer->createObject(address, context);
			if (!instance)
			{
				PxGetFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, PX_FL, 
					"Cannot create class instance for concrete type %d.", classType);
				collection->release();
				return NULL;
			}

			collection->internalAdd(instance);
		}
	}

	PX_ASSERT(nbObjectsInCollection == collection->internalGetNbObjects());
	
	// update new collection with export references
	{
		bool manifestTableAccessError = false;
		PX_ASSERT(addressObjectData != NULL);
		for (PxU32 i=0;i<nbExportReferences;i++)
		{
			bool isExternal;
			PxU32 manifestIndex = exportReferences[i].objIndex.getIndex(isExternal);
			PX_ASSERT(!isExternal);

			if (manifestIndex < nbManifestEntries)
			{
				PxBase* obj = reinterpret_cast<PxBase*>(addressObjectData + manifestTable[manifestIndex].offset);
				collection->mIds.insertUnique(exportReferences[i].id, obj);
				collection->mObjects[obj] = exportReferences[i].id;
			}
			else
			{
				manifestTableAccessError = true;
			}
		}

		if (manifestTableAccessError)
		{
			PxGetFoundation().error(physx::PxErrorCode::eINTERNAL_ERROR, PX_FL, "Manifest table access error");
			collection->release();
			return NULL;
		}
	}

	PxAddCollectionToPhysics(*collection);
	return collection;
}
