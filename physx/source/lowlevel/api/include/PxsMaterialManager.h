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

#ifndef PXS_MATERIAL_MANAGER_H
#define PXS_MATERIAL_MANAGER_H

#include "PxsMaterialCore.h"
#include "PxsFEMSoftBodyMaterialCore.h"
#include "PxsFEMClothMaterialCore.h"
#include "PxsPBDMaterialCore.h"
#include "PxsFLIPMaterialCore.h"
#include "PxsMPMMaterialCore.h"
#include "foundation/PxAlignedMalloc.h"

namespace physx
{
	struct PxsMaterialInfo
	{
		PxU16 mMaterialIndex0;
		PxU16 mMaterialIndex1;
	};

	template<class MaterialCore>
	class PxsMaterialManagerT 
	{
	public:
		PxsMaterialManagerT()
		{
			const PxU32 matCount = 128;
			materials = reinterpret_cast<MaterialCore*>(physx::PxAlignedAllocator<16>().allocate(sizeof(MaterialCore)*matCount,  PX_FL));
			maxMaterials = matCount;
			for(PxU32 i=0; i<matCount; ++i)
			{
				materials[i].mMaterialIndex = MATERIAL_INVALID_HANDLE;
			}
		}

		~PxsMaterialManagerT()
		{
			physx::PxAlignedAllocator<16>().deallocate(materials);
		}

		void setMaterial(MaterialCore* mat)
		{
			const PxU16 materialIndex = mat->mMaterialIndex;
			resize(PxU32(materialIndex) + 1);
			materials[materialIndex] = *mat;
		}

		void updateMaterial(MaterialCore* mat)
		{
			materials[mat->mMaterialIndex] =*mat;
		}

		void removeMaterial(MaterialCore* mat)
		{
			mat->mMaterialIndex = MATERIAL_INVALID_HANDLE;
		}

		PX_FORCE_INLINE MaterialCore* getMaterial(const PxU32 index)const
		{
			PX_ASSERT(index <  maxMaterials);
			return &materials[index];
		}

		PxU32 getMaxSize()const 
		{
			return maxMaterials;
		}

		void resize(PxU32 minValueForMax)
		{			
			if(maxMaterials>=minValueForMax)
				return;

			const PxU32 numMaterials = maxMaterials;
			
			maxMaterials = (minValueForMax+31)&~31;
			MaterialCore* mat = reinterpret_cast<MaterialCore*>(physx::PxAlignedAllocator<16>().allocate(sizeof(MaterialCore)*maxMaterials,  PX_FL));
			for(PxU32 i=0; i<numMaterials; ++i)
				mat[i] = materials[i];

			for(PxU32 i = numMaterials; i < maxMaterials; ++i)
				mat[i].mMaterialIndex = MATERIAL_INVALID_HANDLE;

			physx::PxAlignedAllocator<16>().deallocate(materials);

			materials = mat;
		}

		MaterialCore* materials;//make sure materials's start address is 16 bytes align
		PxU32 maxMaterials;
		PxU32 mPad;
#if !PX_P64_FAMILY
		PxU32 mPad2;
#endif
	};

	//This class is used for forward declaration
	class PxsMaterialManager : public PxsMaterialManagerT<PxsMaterialCore>
	{
	};

	class PxsFEMMaterialManager : public PxsMaterialManagerT<PxsFEMSoftBodyMaterialCore>
	{
	};

	class PxsFEMClothMaterialManager : public PxsMaterialManagerT<PxsFEMClothMaterialCore>
	{
	};

	class PxsPBDMaterialManager : public PxsMaterialManagerT<PxsPBDMaterialCore>
	{
	};

	class PxsFLIPMaterialManager : public PxsMaterialManagerT<PxsFLIPMaterialCore>
	{
	};

	class PxsMPMMaterialManager : public PxsMaterialManagerT<PxsMPMMaterialCore>
	{
	};

	template<class MaterialCore>
	class PxsMaterialManagerIterator
	{
	
	public:
		PxsMaterialManagerIterator(PxsMaterialManagerT<MaterialCore>& manager) : mManager(manager), mIndex(0)
		{
		}

		bool getNextMaterial(MaterialCore*& materialCore)
		{
			const PxU32 maxSize = mManager.getMaxSize();
			PxU32 index = mIndex;
			while(index < maxSize && mManager.getMaterial(index)->mMaterialIndex == MATERIAL_INVALID_HANDLE)
				index++;
			materialCore = NULL;
			if(index < maxSize)
				materialCore = mManager.getMaterial(index++);
			mIndex = index;
			return materialCore!=NULL;
		}

	private:
		PxsMaterialManagerIterator& operator=(const PxsMaterialManagerIterator&);
		PxsMaterialManagerT<MaterialCore>&	mManager;
		PxU32								mIndex;
	};

}

#endif
