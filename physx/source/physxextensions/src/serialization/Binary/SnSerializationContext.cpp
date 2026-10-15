// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "common/PxBase.h"
#include "SnSerializationContext.h"

using namespace physx;
using namespace Sn;

PxBase* DeserializationContext::resolveReference(PxU32 kind, size_t reference) const
{
	//A reference that is not in the map has no object behind it: the collection was serialized with a
	//dangling reference (see e.g. NpShape::requiresObjects, which skips a material it cannot resolve, so
	//nothing registers it), or the buffer has been damaged. Both lookups below used to guard the miss with an
	//assert only and then dereference the NULL entry in a release build. Report it and return NULL, like the
	//unknown-kind branch already does - the callers have to handle NULL either way.
	//The assert is deliberately gone: this is a statement about the input, not about a PhysX invariant, so it
	//must behave the same in every configuration instead of aborting a debug build on malformed data.
	// ### DEFENSIVE (OMPE-109162)
	SerialObjectIndex objIndex;
	if (kind == PX_SERIAL_REF_KIND_PXBASE)
	{
		const InternalPtrRefMap::Entry* entry0 = mInternalPtrReferencesMap.find(reference);
		if (!entry0)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
				"PxDeserializationContext::resolveReference: unknown object reference in the serialized data.");
			return NULL;
		}
		objIndex = entry0->second;
	}
	else if (kind == PX_SERIAL_REF_KIND_MATERIAL_IDX)
	{
		const InternalHandle16RefMap::Entry* entry0 = mInternalHandle16ReferencesMap.find(PxU16(reference));
		if (!entry0)
		{
			PxGetFoundation().error(PxErrorCode::eINTERNAL_ERROR, PX_FL,
				"PxDeserializationContext::resolveReference: unknown material reference %u in the serialized data.", PxU32(PxU16(reference)));
			return NULL;
		}
		objIndex = entry0->second;
	}
	else
	{
		return NULL;
	}

	bool isExternal;
	PxU32 index = objIndex.getIndex(isExternal);
	PxBase* base = NULL;
	if (isExternal)
	{
		const ImportReference& entry = mImportReferences[index];
		base = mExternalRefs->find(entry.id);	
	}
	else
	{
		const ManifestEntry& entry = mManifestTable[index];
		base = reinterpret_cast<PxBase*>(mObjectDataAddress + entry.offset);
	}
	PX_ASSERT(base);
	return base;
}
	
void SerializationContext::registerReference(PxBase& serializable, PxU32 kind, size_t reference)
{
#if PX_CHECKED
	if ((kind & PX_SERIAL_REF_KIND_PTR_TYPE_BIT) == 0 && reference > 0xffff)
	{
		PxGetFoundation().error(PxErrorCode::eINVALID_PARAMETER, PX_FL, "PxSerializationContext::registerReference: only 16 bit handles supported.");
		return;
	}
#endif

	bool isExternal = mExternalRefs && mExternalRefs->contains(serializable);
	PxU32 index;
	if (isExternal)
	{
		PxSerialObjectId id = mExternalRefs->getId(serializable);
		PX_ASSERT(id != PX_SERIAL_OBJECT_ID_INVALID);
		if (const PxHashMap<PxSerialObjectId, PxU32>::Entry* entry = mImportReferencesMap.find(id))
		{
			index = entry->second;
		}
		else
		{
			index = mImportReferences.size();
			mImportReferencesMap.insert(id, index);
			mImportReferences.pushBack(ImportReference(id, serializable.getConcreteType()));
		}
	}
	else
	{
		PX_ASSERT(mCollection.contains(serializable));
		index = mObjToCollectionIndexMap[&serializable];
	}

	if (kind & PX_SERIAL_REF_KIND_PXBASE)
	{
		mInternalPtrReferencesMap[reference] = SerialObjectIndex(index, isExternal);
	}
	else if (kind & PX_SERIAL_REF_KIND_MATERIAL_IDX)
	{
		mInternalHandle16ReferencesMap[PxU16(reference)] = SerialObjectIndex(index, isExternal);
	}
}
