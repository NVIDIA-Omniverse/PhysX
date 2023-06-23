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

#ifndef SN_SERIALIZATION_CONTEXT_H
#define SN_SERIALIZATION_CONTEXT_H

#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "foundation/PxHash.h"
#include "common/PxSerialFramework.h"
#include "extensions/PxDefaultStreams.h"

#include "foundation/PxUserAllocated.h"
#include "CmCollection.h"
#include "CmUtils.h"
#include "SnConvX_Align.h"

namespace physx
{
	namespace Sn
	{
		struct ManifestEntry
		{
			PX_FORCE_INLINE	ManifestEntry(PxU32 _offset, PxType _type)
			{
				PxMarkSerializedMemory(this, sizeof(ManifestEntry));
				offset = _offset;
				type = _type;
			}			
			PX_FORCE_INLINE	ManifestEntry() { PxMarkSerializedMemory(this, sizeof(ManifestEntry)); }
			PX_FORCE_INLINE void operator =(const ManifestEntry& m)
			{
				PxMemCopy(this, &m, sizeof(ManifestEntry));				
			}
	
			PxU32 offset;
			PxType type;
		};

		struct ImportReference
		{
			PX_FORCE_INLINE	ImportReference(PxSerialObjectId _id, PxType _type)
			{ 
				PxMarkSerializedMemory(this, sizeof(ImportReference));
				id = _id;
				type = _type;
			}
			PX_FORCE_INLINE	ImportReference() { PxMarkSerializedMemory(this, sizeof(ImportReference)); }
			PX_FORCE_INLINE void operator =(const ImportReference& m)
			{
				PxMemCopy(this, &m, sizeof(ImportReference));				
			}
			PxSerialObjectId id;
			PxType type;
		};

#define SERIAL_OBJECT_INDEX_TYPE_BIT (1u<<31)
		struct SerialObjectIndex
		{
			PX_FORCE_INLINE	SerialObjectIndex(PxU32 index, bool external) { setIndex(index, external); }
			PX_FORCE_INLINE	SerialObjectIndex(const SerialObjectIndex& objIndex) : mObjIndex(objIndex.mObjIndex) {}
			PX_FORCE_INLINE	SerialObjectIndex() : mObjIndex(PX_INVALID_U32) {}

			PX_FORCE_INLINE void setIndex(PxU32 index, bool external)
			{
				PX_ASSERT((index & SERIAL_OBJECT_INDEX_TYPE_BIT) == 0); 
				mObjIndex = index | (external ? SERIAL_OBJECT_INDEX_TYPE_BIT : 0);
			}

			PX_FORCE_INLINE PxU32 getIndex(bool& isExternal)
			{
				PX_ASSERT(mObjIndex != PX_INVALID_U32);
				isExternal = (mObjIndex & SERIAL_OBJECT_INDEX_TYPE_BIT) > 0;
				return mObjIndex & ~SERIAL_OBJECT_INDEX_TYPE_BIT;
			}

			PX_FORCE_INLINE bool operator < (const SerialObjectIndex& so) const
			{
				return mObjIndex < so.mObjIndex;
			}

		private:
			PxU32 mObjIndex;
		};

		struct ExportReference
		{
			PX_FORCE_INLINE	ExportReference(PxSerialObjectId _id, SerialObjectIndex _objIndex)
			{
				PxMarkSerializedMemory(this, sizeof(ExportReference));
				id = _id;
				objIndex = _objIndex;
			}
			PX_FORCE_INLINE	ExportReference() { PxMarkSerializedMemory(this, sizeof(ExportReference)); }
			PX_FORCE_INLINE void operator =(const ExportReference& m)
			{
				PxMemCopy(this, &m, sizeof(ExportReference));				
			}
			PxSerialObjectId id;
			SerialObjectIndex objIndex;
		};

		struct InternalReferencePtr
		{
			PX_FORCE_INLINE	InternalReferencePtr() {}
			
			PX_FORCE_INLINE	InternalReferencePtr(size_t _reference, SerialObjectIndex _objIndex) :
				reference(_reference),
				objIndex(_objIndex)
#if PX_P64_FAMILY
				,pad(PX_PADDING_32)
#endif
			{
			}

			size_t reference;
			SerialObjectIndex objIndex;
#if PX_P64_FAMILY
			PxU32 pad;
#endif
		};

		struct InternalReferenceHandle16
		{
			PX_FORCE_INLINE	InternalReferenceHandle16() {}

			PX_FORCE_INLINE	InternalReferenceHandle16(PxU16 _reference, SerialObjectIndex _objIndex) :
				reference(_reference),
				pad(PX_PADDING_16),
				objIndex(_objIndex)
			{
			}

			PxU16 reference;
			PxU16 pad;
			SerialObjectIndex objIndex;
		};

		typedef Cm::CollectionHashMap<size_t, SerialObjectIndex> InternalPtrRefMap;
		typedef Cm::CollectionHashMap<PxU16, SerialObjectIndex> InternalHandle16RefMap;

		class DeserializationContext : public PxDeserializationContext, public PxUserAllocated
		{
			PX_NOCOPY(DeserializationContext)
		
		public:
			DeserializationContext(const ManifestEntry* manifestTable, 
								   const ImportReference* importReferences,
								   PxU8* objectDataAddress, 
								   const InternalPtrRefMap& internalPtrReferencesMap, 
								   const InternalHandle16RefMap& internalHandle16ReferencesMap, 
								   const Cm::Collection* externalRefs,
								   PxU8* extraData)
			: mManifestTable(manifestTable)
			, mImportReferences(importReferences)
			, mObjectDataAddress(objectDataAddress)
			, mInternalPtrReferencesMap(internalPtrReferencesMap)
			, mInternalHandle16ReferencesMap(internalHandle16ReferencesMap)
			, mExternalRefs(externalRefs)
			{
				mExtraDataAddress = extraData;
			}

			virtual	PxBase*	resolveReference(PxU32 kind, size_t reference) const;

		private:
			//various pointers to deserialized data
			const ManifestEntry* mManifestTable;
			const ImportReference* mImportReferences;
			PxU8* mObjectDataAddress;

			//internal references maps for resolving references.
			const InternalPtrRefMap& mInternalPtrReferencesMap;
			const InternalHandle16RefMap& mInternalHandle16ReferencesMap;

			//external collection for resolving import references.
			const Cm::Collection* mExternalRefs;
			//const PxU32 mPhysXVersion;
		};

		class SerializationContext : public PxSerializationContext, public PxUserAllocated
		{
			PX_NOCOPY(SerializationContext)
		public:
			SerializationContext(const Cm::Collection& collection, const Cm::Collection* externalRefs) 
			: mCollection(collection)
			, mExternalRefs(externalRefs) 
			{
				// fill object to collection index map (same ordering as manifest)
				for (PxU32 i=0;i<mCollection.internalGetNbObjects();i++)
				{
					mObjToCollectionIndexMap[mCollection.internalGetObject(i)] = i;
				}
			}

			virtual		void		writeData(const void* buffer, PxU32 size)		{	mMemStream.write(buffer, size);	}
			virtual		PxU32		getTotalStoredSize()							{	return mMemStream.getSize(); }
			virtual		void		alignData(PxU32 alignment = PX_SERIAL_ALIGN)		
			{	
				if(!alignment)
					return;

				PxI32 bytesToPad = PxI32(getPadding(mMemStream.getSize(), alignment));
				static const PxI32 BUFSIZE = 64;
				char buf[BUFSIZE];
				PxMemSet(buf, 0, bytesToPad < BUFSIZE ? PxU32(bytesToPad) : PxU32(BUFSIZE));
				while(bytesToPad > 0)
				{
					mMemStream.write(buf, bytesToPad < BUFSIZE ? PxU32(bytesToPad) : PxU32(BUFSIZE));
					bytesToPad -= BUFSIZE;
				}
				PX_ASSERT(!getPadding(getTotalStoredSize(), alignment));
			}

			virtual void writeName(const char*)
			{
				PxGetFoundation().error(physx::PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__, 
					"Cannot export names during exportData.");
			}

			const PxCollection& getCollection() const	{	return mCollection;		}

			virtual void registerReference(PxBase& serializable, PxU32 kind, size_t reference);

			const PxArray<ImportReference>& getImportReferences() { return mImportReferences; }
			InternalPtrRefMap& getInternalPtrReferencesMap() { return mInternalPtrReferencesMap; }
			InternalHandle16RefMap& getInternalHandle16ReferencesMap() { return mInternalHandle16ReferencesMap; }

			PxU32		getSize()	const	{	return mMemStream.getSize(); }
			PxU8*		getData()	const	{	return mMemStream.getData(); }



		private:
			//import reference map for unique registration of import references and corresponding buffer.
			PxHashMap<PxSerialObjectId, PxU32> mImportReferencesMap;
			PxArray<ImportReference> mImportReferences;
			
			//maps for unique registration of internal references
			InternalPtrRefMap mInternalPtrReferencesMap;
			InternalHandle16RefMap mInternalHandle16ReferencesMap;

			//map for quick lookup of manifest index. 
			PxHashMap<const PxBase*, PxU32> mObjToCollectionIndexMap;

			//collection and externalRefs collection for assigning references.
			const Cm::Collection& mCollection;
			const Cm::Collection* mExternalRefs;

			PxDefaultMemoryOutputStream mMemStream;

		};

	} // namespace Sn
}

#endif
