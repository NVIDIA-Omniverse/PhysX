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

#include "vehicle/PxVehicleTireFriction.h"

#include "foundation/PxMemory.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundation.h"
#include "foundation/PxAllocator.h"
#include "foundation/PxIO.h"
#include "foundation/PxBitMap.h"
#include "common/PxCollection.h"

#include "PxMaterial.h"

namespace physx
{

PX_FORCE_INLINE PxU32 computeByteSize(const PxU32 maxNbTireTypes, const PxU32 maxNbSurfaceTypes)
{
	PxU32 byteSize = ((sizeof(PxU32)*(maxNbTireTypes*maxNbSurfaceTypes) + 15) & ~15);
	byteSize += ((sizeof(PxMaterial*)*maxNbSurfaceTypes + 15) & ~15);
	byteSize += ((sizeof(PxVehicleDrivableSurfaceType)*maxNbSurfaceTypes + 15) & ~15);
	byteSize += ((sizeof(PxSerialObjectId)*maxNbSurfaceTypes + 15) & ~15);
	byteSize += ((sizeof(PxVehicleDrivableSurfaceToTireFrictionPairs) + 15) & ~ 15);
	return byteSize;
}

PxVehicleDrivableSurfaceToTireFrictionPairs* PxVehicleDrivableSurfaceToTireFrictionPairs::allocate
(const PxU32 maxNbTireTypes, const PxU32 maxNbSurfaceTypes)
{
	PX_CHECK_AND_RETURN_VAL(maxNbSurfaceTypes <= eMAX_NB_SURFACE_TYPES, "maxNbSurfaceTypes must be less than eMAX_NB_SURFACE_TYPES", NULL);

	PxU32 byteSize = computeByteSize(maxNbTireTypes, maxNbSurfaceTypes);
	PxU8* ptr = static_cast<PxU8*>(PX_ALLOC(byteSize, "PxVehicleDrivableSurfaceToTireFrictionPairs"));
	PxMemSet(ptr, 0, byteSize);
	PxVehicleDrivableSurfaceToTireFrictionPairs* pairs = reinterpret_cast<PxVehicleDrivableSurfaceToTireFrictionPairs*>(ptr);

	pairs->mPairs = NULL;
	pairs->mDrivableSurfaceMaterials = NULL;
	pairs->mDrivableSurfaceTypes = NULL;
	pairs->mMaterialSerialIds = NULL;
	pairs->mNbTireTypes = 0;
	pairs->mMaxNbTireTypes = maxNbTireTypes;
	pairs->mNbSurfaceTypes = 0;
	pairs->mMaxNbSurfaceTypes = maxNbSurfaceTypes;

	return pairs;
}

void PxVehicleDrivableSurfaceToTireFrictionPairs::setup
(const PxU32 numTireTypes, const PxU32 numSurfaceTypes, const PxMaterial** drivableSurfaceMaterials, const PxVehicleDrivableSurfaceType* drivableSurfaceTypes)
{
	PX_CHECK_AND_RETURN(numTireTypes <= mMaxNbTireTypes, "numTireTypes must be less than mMaxNumSurfaceTypes");
	PX_CHECK_AND_RETURN(numSurfaceTypes <= mMaxNbSurfaceTypes, "numSurfaceTypes must be less than mMaxNumSurfaceTypes");

	PxU8* ptr = reinterpret_cast<PxU8*>(this);

	const PxU32 maxNbTireTypes = mMaxNbTireTypes;
	const PxU32 maxNbSurfaceTypes = mMaxNbSurfaceTypes;
	PxU32 byteSize = computeByteSize(mMaxNbTireTypes, mMaxNbSurfaceTypes);
	PxMemSet(ptr, 0, byteSize);
	mMaxNbTireTypes = maxNbTireTypes;
	mMaxNbSurfaceTypes = maxNbSurfaceTypes;

	PxVehicleDrivableSurfaceToTireFrictionPairs* pairs = reinterpret_cast<PxVehicleDrivableSurfaceToTireFrictionPairs*>(ptr);
	ptr += ((sizeof(PxVehicleDrivableSurfaceToTireFrictionPairs) + 15) & ~ 15);

	mPairs = reinterpret_cast<PxReal*>(ptr);
	ptr += ((sizeof(PxU32)*(numTireTypes*numSurfaceTypes) + 15) & ~15);
	mDrivableSurfaceMaterials = reinterpret_cast<const PxMaterial**>(ptr);
	ptr += ((sizeof(PxMaterial*)*numSurfaceTypes + 15) & ~15);
	mDrivableSurfaceTypes = reinterpret_cast<PxVehicleDrivableSurfaceType*>(ptr);
	ptr += ((sizeof(PxVehicleDrivableSurfaceType)*numSurfaceTypes +15) & ~15);
	mMaterialSerialIds = reinterpret_cast<PxSerialObjectId*>(ptr);
	ptr += ((sizeof(PxSerialObjectId)*numSurfaceTypes + 15) & ~15);

	for(PxU32 i=0;i<numSurfaceTypes;i++)
	{
		mDrivableSurfaceTypes[i] = drivableSurfaceTypes[i];
		mDrivableSurfaceMaterials[i] = drivableSurfaceMaterials[i];
		mMaterialSerialIds[i] = 0;
	}
	for(PxU32 i=0;i<numTireTypes*numSurfaceTypes;i++)
	{
		mPairs[i]=1.0f;
	}

	pairs->mNbTireTypes=numTireTypes;
	pairs->mNbSurfaceTypes=numSurfaceTypes;
}

void PxVehicleDrivableSurfaceToTireFrictionPairs::release()
{
	PX_FREE_THIS;
}

void PxVehicleDrivableSurfaceToTireFrictionPairs::setTypePairFriction(const PxU32 surfaceType, const PxU32 tireType, const PxReal value)
{
	PX_CHECK_AND_RETURN(tireType<mNbTireTypes, "Invalid tireType");
	PX_CHECK_AND_RETURN(surfaceType<mNbSurfaceTypes, "Invalid surfaceType");

	*(mPairs + mNbTireTypes*surfaceType + tireType) = value;
}

PxReal PxVehicleDrivableSurfaceToTireFrictionPairs::getTypePairFriction(const PxU32 surfaceType, const PxU32 tireType) const 
{
	PX_CHECK_AND_RETURN_VAL(tireType<mNbTireTypes, "Invalid tireType", 0.0f);
	PX_CHECK_AND_RETURN_VAL(surfaceType<mNbSurfaceTypes, "Invalid surfaceType", 0.0f);

	return *(mPairs + mNbTireTypes*surfaceType + tireType);
}

////////////////////////////////////////////////////////////////////////////
//Hash table of PxMaterial pointers used to associate each PxMaterial pointer
//with a unique PxDrivableSurfaceType.  PxDrivableSurfaceType is just an integer
//representing an id but introducing this type allows different PxMaterial pointers
//to be associated with the same surface type.  The friction of a specific tire
//touching a specific PxMaterial is found from a 2D table using the integers for
//the tire type (stored in the tire) and drivable surface type (from the hash table).
//It would be great to use PsHashSet for the hash table of PxMaterials but
//PsHashSet will never, ever work on spu so this will need to do instead.
//Perf isn't really critical so this will do in the meantime.
//It is probably wasteful to compute the hash table each update
//but this is really not an expensive operation so keeping the api as 
//simple as possible wins out at the cost of a relatively very small number of wasted cycles.
////////////////////////////////////////////////////////////////////////////

class VehicleSurfaceTypeHashTable
{
public:

	VehicleSurfaceTypeHashTable(const PxVehicleDrivableSurfaceToTireFrictionPairs& pairs)
		: mNbEntries(pairs.mNbSurfaceTypes),
		mMaterials(pairs.mDrivableSurfaceMaterials),
		mDrivableSurfaceTypes(pairs.mDrivableSurfaceTypes)
	{
		for (PxU32 i = 0; i < eHASH_SIZE; i++)
		{
			mHeadIds[i] = PX_MAX_U32;
		}
		for (PxU32 i = 0; i < eMAX_NB_KEYS; i++)
		{
			mNextIds[i] = PX_MAX_U32;
		}

		if (mNbEntries > 0)
		{
			//Compute the number of bits to right-shift that gives the maximum number of unique hashes.
			//Keep searching until we find either a set of completely unique hashes or a peak count of unique hashes.
			PxU32 prevShift = 0;
			PxU32 shift = 2;
			PxU32 prevNumUniqueHashes = 0;
			PxU32 currNumUniqueHashes = 0;
			while (((currNumUniqueHashes = computeNumUniqueHashes(shift)) > prevNumUniqueHashes) && currNumUniqueHashes != mNbEntries)
			{
				prevNumUniqueHashes = currNumUniqueHashes;
				prevShift = shift;
				shift = (shift << 1);
			}
			if (currNumUniqueHashes != mNbEntries)
			{
				//Stopped searching because we have gone past the peak number of unqiue hashes.
				mShift = prevShift;
			}
			else
			{
				//Stopped searching because we found a unique hash for each key.
				mShift = shift;
			}

			//Compute the hash values with the optimum shift.
			for (PxU32 i = 0; i < mNbEntries; i++)
			{
				const PxMaterial* const material = mMaterials[i];
				const PxU32 hash = computeHash(material, mShift);
				if (PX_MAX_U32 == mHeadIds[hash])
				{
					mNextIds[i] = PX_MAX_U32;
					mHeadIds[hash] = i;
				}
				else
				{
					mNextIds[i] = mHeadIds[hash];
					mHeadIds[hash] = i;
				}
			}
		}
	}
	~VehicleSurfaceTypeHashTable()
	{
	}

	PX_FORCE_INLINE PxU32 get(const PxMaterial* const key) const
	{
		PX_ASSERT(key);
		const PxU32 hash = computeHash(key, mShift);
		PxU32 id = mHeadIds[hash];
		while (PX_MAX_U32 != id)
		{
			const PxMaterial* const mat = mMaterials[id];
			if (key == mat)
			{
				return mDrivableSurfaceTypes[id].mType;
			}
			id = mNextIds[id];
		}

		return 0;
	}

private:

	PxU32 mNbEntries;
	const PxMaterial* const* mMaterials;
	const PxVehicleDrivableSurfaceType* mDrivableSurfaceTypes;

	static PX_FORCE_INLINE PxU32 computeHash(const PxMaterial* const key, const PxU32 shift)
	{
		const uintptr_t ptr = ((uintptr_t(key)) >> shift);
		const uintptr_t hash = (ptr & (eHASH_SIZE - 1));
		return PxU32(hash);
	}

	PxU32 computeNumUniqueHashes(const PxU32 shift) const
	{
		PxU32 words[eHASH_SIZE >> 5];
		PxU8* bitmapBuffer[sizeof(PxBitMap)];
		PxBitMap* bitmap = reinterpret_cast<PxBitMap*>(bitmapBuffer);
		bitmap->setWords(words, eHASH_SIZE >> 5);

		PxU32 numUniqueHashes = 0;
		PxMemZero(words, sizeof(PxU32)*(eHASH_SIZE >> 5));
		for (PxU32 i = 0; i < mNbEntries; i++)
		{
			const PxMaterial* const material = mMaterials[i];
			const PxU32 hash = computeHash(material, shift);
			if (!bitmap->test(hash))
			{
				bitmap->set(hash);
				numUniqueHashes++;
			}
		}
		return numUniqueHashes;
	}

	enum
	{
		eHASH_SIZE = PxVehicleDrivableSurfaceToTireFrictionPairs::eMAX_NB_SURFACE_TYPES
	};
	PxU32 mHeadIds[eHASH_SIZE];
	enum
	{
		eMAX_NB_KEYS = PxVehicleDrivableSurfaceToTireFrictionPairs::eMAX_NB_SURFACE_TYPES
	};
	PxU32 mNextIds[eMAX_NB_KEYS];

	PxU32 mShift;
};

PxU32 PxVehicleDrivableSurfaceToTireFrictionPairs::getSurfaceType(const PxMaterial& surfaceMaterial) const
{
	const VehicleSurfaceTypeHashTable surfaceTypeHashTable(*this);
	const PxU32 surfaceType = surfaceTypeHashTable.get(&surfaceMaterial);
	return surfaceType;
}

PxReal PxVehicleDrivableSurfaceToTireFrictionPairs::getTypePairFriction(const PxMaterial& surfaceMaterial, const PxU32 tireType) const
{
	const PxU32 surfaceType = getSurfaceType(surfaceMaterial);
	return getTypePairFriction(surfaceType, tireType);
}

#if PX_CHECKED
bool isLegalCollectionWithNullArray(PxCollection* collection, const PxSerialObjectId* materialIds, const PxMaterial** materials, const PxU32 nbMaterials)
{
	if (!materialIds)
	{
		for (PxU32 i = 0; i < nbMaterials; i++)
		{
			const PxMaterial* material = materials[i];
			if (0 == collection->getId(*material))
			{
				return false;
			}
		}
	}
	return true;
}

bool isLegalCollection(PxCollection* collection, const PxSerialObjectId* materialIds, const PxMaterial** materials, const PxU32 nbMaterials)
{
	if(materialIds)
	{
		for (PxU32 i = 0; i < nbMaterials; i++)
		{
			const PxMaterial* material = materials[i];
			if (0 == collection->getId(*material))
			{
				//Material has not yet been assigned an id.
				//Make sure materialId is free.
				if(collection->find(materialIds[i]))
				{
					return false;
				}
			}
		}
	}
	return true;
}
#endif

void PxVehicleDrivableSurfaceToTireFrictionPairs::serializeToBinary
(const PxVehicleDrivableSurfaceToTireFrictionPairs& frictionTable,
 const PxSerialObjectId* materialIds, const PxU32 nbMaterialIds,
 PxCollection* collection, PxOutputStream& stream)
{
	PX_CHECK_AND_RETURN(!materialIds || nbMaterialIds >= frictionTable.mNbSurfaceTypes, "PxVehicleDrivableSurfaceToTireFrictionPairs::serializeToBinary - insufficient nbMaterialIds");
	PX_CHECK_AND_RETURN(isLegalCollectionWithNullArray(collection, materialIds, frictionTable.mDrivableSurfaceMaterials, frictionTable.mNbSurfaceTypes), "PxVehicleDrivableSurfaceToTireFrictionPairs::serializeToBinary - material ids not configured");
	PX_CHECK_AND_RETURN(isLegalCollection(collection, materialIds, frictionTable.mDrivableSurfaceMaterials, frictionTable.mNbSurfaceTypes), "PxVehicleDrivableSurfaceToTireFrictionPairs::serializeToBinary - material id already in use");
	PX_UNUSED(nbMaterialIds);

	for (PxU32 i = 0; i < frictionTable.mNbSurfaceTypes; i++)
	{
		const PxMaterial* material = frictionTable.mDrivableSurfaceMaterials[i];	
		const PxSerialObjectId id = collection->getId(*material);
		if(0 == id)
		{
			collection->add(*const_cast<PxMaterial*>(material), materialIds[i]);
			const_cast<PxVehicleDrivableSurfaceToTireFrictionPairs&>(frictionTable).mMaterialSerialIds[i] = materialIds[i];
		}
		else
		{
			const_cast<PxVehicleDrivableSurfaceToTireFrictionPairs&>(frictionTable).mMaterialSerialIds[i] = collection->getId(*material);
		}
	}
			
	stream.write(&frictionTable, computeByteSize(frictionTable.mMaxNbTireTypes, frictionTable.mMaxNbSurfaceTypes));

	for (PxU32 i = 0; i < frictionTable.mNbSurfaceTypes; i++)
	{
		const_cast<PxVehicleDrivableSurfaceToTireFrictionPairs&>(frictionTable).mMaterialSerialIds[i] = 0;
	}
}

PxVehicleDrivableSurfaceToTireFrictionPairs* PxVehicleDrivableSurfaceToTireFrictionPairs::deserializeFromBinary(const PxCollection& collection, void* buffer)
{
	PxVehicleDrivableSurfaceToTireFrictionPairs* fricTable = reinterpret_cast<PxVehicleDrivableSurfaceToTireFrictionPairs*>(buffer);

	//Patch up pointers.
	{
		PxU8* ptr = reinterpret_cast<PxU8*>(fricTable);
		ptr += ((sizeof(PxVehicleDrivableSurfaceToTireFrictionPairs) + 15) & ~15);
		fricTable->mPairs = reinterpret_cast<PxReal*>(ptr);
		ptr += ((sizeof(PxU32) * (fricTable->mNbTireTypes * fricTable->mNbSurfaceTypes) + 15) & ~15);
		fricTable->mDrivableSurfaceMaterials = reinterpret_cast<const PxMaterial**>(ptr);
		ptr += ((sizeof(PxMaterial*) * fricTable->mNbSurfaceTypes + 15) & ~15);
		fricTable->mDrivableSurfaceTypes = reinterpret_cast<PxVehicleDrivableSurfaceType*>(ptr);
		ptr += ((sizeof(PxVehicleDrivableSurfaceType) * fricTable->mNbSurfaceTypes + 15) & ~15);
		fricTable->mMaterialSerialIds = reinterpret_cast<PxSerialObjectId*>(ptr);
		ptr += ((sizeof(PxSerialObjectId) * fricTable->mNbSurfaceTypes + 15) & ~15);
	}

	//Set the material pointers in order.
	for (PxU32 i = 0; i < fricTable->mNbSurfaceTypes; i++)
	{
		const PxSerialObjectId id = fricTable->mMaterialSerialIds[i];
		const PxBase* base = collection.find(id);
		const PxMaterial* material = base->is<PxMaterial>();
		fricTable->mDrivableSurfaceMaterials[i] = material;
	}

	return fricTable;
}

}//physx

