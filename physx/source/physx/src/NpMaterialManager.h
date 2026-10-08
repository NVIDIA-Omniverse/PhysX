// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


#ifndef NP_MATERIALMANAGER
#define NP_MATERIALMANAGER

#include "foundation/PxMemory.h"
#include "CmIDPool.h"

namespace physx
{
	template<class Material>
	class NpMaterialManager 
	{
	public:
		NpMaterialManager()
		{
			const PxU32 matCount = 128;
			mMaterials = reinterpret_cast<Material**>(PX_ALLOC(sizeof(Material*) * matCount, "NpMaterialManager::initialise"));
			mMaxMaterials = matCount;
			PxMemZero(mMaterials, sizeof(Material*)*mMaxMaterials);
		}

		~NpMaterialManager() {}

		void releaseMaterials()
		{
			for(PxU32 i=0; i<mMaxMaterials; ++i)
			{
				if(mMaterials[i])
				{
					const PxU32 handle(mMaterials[i]->mMaterial.mMaterialIndex);
					mHandleManager.freeID(handle);
					mMaterials[i]->release();
					mMaterials[i] = NULL;
				}
			}
			PX_FREE(mMaterials);
		}

		bool setMaterial(Material& mat)
		{
			const PxU32 poolID = mHandleManager.getNewID();
			if (poolID >= MATERIAL_INVALID_HANDLE)
				return false;

			const PxU16 materialIndex = PxTo16(poolID);

			if(materialIndex >= mMaxMaterials)
				resize();

			mMaterials[materialIndex] = &mat;
			mat.mMaterial.mMaterialIndex = materialIndex;
			return true;
		}

		void updateMaterial(Material& mat)
		{
			mMaterials[mat.mMaterial.mMaterialIndex] = &mat;
		}

		PX_FORCE_INLINE PxU32 getNumMaterials()	const
		{
			return mHandleManager.getNumUsedID();
		}

		void removeMaterial(Material& mat)
		{
			const PxU16 handle = mat.mMaterial.mMaterialIndex;
			if(handle != MATERIAL_INVALID_HANDLE)
			{
				mMaterials[handle] = NULL;
				mHandleManager.freeID(PxU32(handle));
			}
		}

		PX_FORCE_INLINE Material* getMaterial(const PxU32 index)	const
		{
			PX_ASSERT(index <  mMaxMaterials);
			return mMaterials[index];
		}

		PX_FORCE_INLINE PxU32 getMaxSize()	const 
		{
			return mMaxMaterials;
		}

		PX_FORCE_INLINE Material** getMaterials() const
		{
			return mMaterials;
		}

	private:
		void resize()
		{
			const PxU32 numMaterials = mMaxMaterials;
			mMaxMaterials = PxMin(mMaxMaterials*2, PxU32(MATERIAL_INVALID_HANDLE));

			Material** mat = reinterpret_cast<Material**>(PX_ALLOC(sizeof(Material*)*mMaxMaterials, "NpMaterialManager::resize"));
			PxMemZero(mat, sizeof(Material*)*mMaxMaterials);
			for(PxU32 i=0; i<numMaterials; ++i)
				mat[i] = mMaterials[i];

			PX_FREE(mMaterials);

			mMaterials = mat;
		}

		Cm::IDPool		mHandleManager;
		Material**		mMaterials;
		PxU32			mMaxMaterials;
	};

	template<class Material>
	class NpMaterialManagerIterator
	{
	public:
		NpMaterialManagerIterator(const NpMaterialManager<Material>& manager) : mManager(manager), mIndex(0)
		{
		}

		bool getNextMaterial(Material*& np)
		{
			const PxU32 maxSize = mManager.getMaxSize();
			PxU32 index = mIndex;
			while(index < maxSize && mManager.getMaterial(index)==NULL)
				index++;
			np = NULL;
			if(index < maxSize)
				np = mManager.getMaterial(index++);
			mIndex = index;
			return np!=NULL;
		}

	private:
		NpMaterialManagerIterator& operator=(const NpMaterialManagerIterator&);
		const NpMaterialManager<Material>&	mManager;
		PxU32								mIndex;
	};
}

#endif
