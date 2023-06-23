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

#ifndef NP_FEM_CLOTH_MATERIAL_H
#define NP_FEM_CLOTH_MATERIAL_H

#include "common/PxSerialFramework.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "CmRefCountable.h"
#include "PxsFEMClothMaterialCore.h"

namespace physx
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	// Compared to other objects, materials are special since they belong to the SDK and not to scenes
	// (similar to meshes). That's why the NpFEMClothMaterial does have direct access to the core material instead
	// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
	// the materials will be buffered.

	class NpFEMClothMaterial : public PxFEMClothMaterial, public PxUserAllocated
	{
	public:
		// PX_SERIALIZATION            
										NpFEMClothMaterial(PxBaseFlags baseFlags) : PxFEMClothMaterial(baseFlags), mMaterial(PxEmpty) {}
		virtual		void				resolveReferences(PxDeserializationContext& context);
		static		NpFEMClothMaterial*	createObject(PxU8*& address, PxDeserializationContext& context);
		static		void				getBinaryMetaData(PxOutputStream& stream);

					void				preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
					void				exportExtraData(PxSerializationContext&) {}
					void				importExtraData(PxDeserializationContext&) {}
		virtual		void				requiresObjects(PxProcessPxBaseCallback&) {}
		//~PX_SERIALIZATION
										NpFEMClothMaterial(const PxsFEMClothMaterialCore& desc);
		virtual							~NpFEMClothMaterial();

		// PxBase
		virtual		void				release()	PX_OVERRIDE;
		//~PxBase

		// PxRefCounted
		virtual		void				acquireReference()	PX_OVERRIDE;
		virtual		PxU32				getReferenceCount() const	PX_OVERRIDE;
		virtual		void				onRefCountZero()	PX_OVERRIDE;
		//~PxRefCounted

		// PxFEMMaterial
		virtual		void				setYoungsModulus(PxReal young)	PX_OVERRIDE;
		virtual		PxReal				getYoungsModulus() const	PX_OVERRIDE;
		virtual		void				setPoissons(PxReal poisson)	PX_OVERRIDE;
		virtual		PxReal				getPoissons() const	PX_OVERRIDE;
		virtual		void				setDynamicFriction(PxReal threshold)	PX_OVERRIDE;
		virtual		PxReal				getDynamicFriction() const	PX_OVERRIDE;
		//~PxFEMMaterial

		// PxFEMClothMaterial
		virtual		void				setThickness(PxReal thickness)	PX_OVERRIDE;
		virtual		PxReal				getThickness() const	PX_OVERRIDE;
		//~PxFEMClothMaterial

		PX_FORCE_INLINE static void		getMaterialIndices(PxFEMClothMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

	private:
		PX_INLINE	void				updateMaterial();

		// PX_SERIALIZATION
	public:
		//~PX_SERIALIZATION
			PxsFEMClothMaterialCore		mMaterial;
	};

	PX_FORCE_INLINE void NpFEMClothMaterial::getMaterialIndices(PxFEMClothMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
	{
		for (PxU32 i = 0; i < materialCount; i++)
			materialIndices[i] = static_cast<NpFEMClothMaterial*>(materials[i])->mMaterial.mMaterialIndex;
	}
#endif
}

#endif
