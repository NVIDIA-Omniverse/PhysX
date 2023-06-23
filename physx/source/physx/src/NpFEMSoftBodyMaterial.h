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

#ifndef NP_FEM_MATERIAL_H
#define NP_FEM_MATERIAL_H

#include "common/PxSerialFramework.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "CmRefCountable.h"
#include "PxsFEMSoftBodyMaterialCore.h"

namespace physx
{
	// Compared to other objects, materials are special since they belong to the SDK and not to scenes
	// (similar to meshes). That's why the NpFEMMaterial does have direct access to the core material instead
	// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
	// the materials will be buffered.

	class NpFEMSoftBodyMaterial : public PxFEMSoftBodyMaterial, public PxUserAllocated
	{
	public:
		// PX_SERIALIZATION            
										NpFEMSoftBodyMaterial(PxBaseFlags baseFlags) : PxFEMSoftBodyMaterial(baseFlags), mMaterial(PxEmpty) {}
		virtual		void				resolveReferences(PxDeserializationContext& context);
		static	NpFEMSoftBodyMaterial*	createObject(PxU8*& address, PxDeserializationContext& context);
		static		void				getBinaryMetaData(PxOutputStream& stream);

		void							preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
		void							exportExtraData(PxSerializationContext&) {}
		void							importExtraData(PxDeserializationContext&) {}
		virtual		void				requiresObjects(PxProcessPxBaseCallback&) {}
		//~PX_SERIALIZATION
										NpFEMSoftBodyMaterial(const PxsFEMSoftBodyMaterialCore& desc);
		virtual							~NpFEMSoftBodyMaterial();

		// PxBase
		virtual		void				onRefCountZero();
		//~PxBase

		virtual		void				release();

		// PxRefCounted
		virtual		void				acquireReference();
		virtual		PxU32				getReferenceCount() const;
		//~PxRefCounted

		// PxFEMMaterial
		virtual		void				setYoungsModulus(PxReal young)	PX_OVERRIDE;
		virtual		PxReal				getYoungsModulus() const	PX_OVERRIDE;
		virtual		void				setPoissons(PxReal poisson)	PX_OVERRIDE;
		virtual		PxReal				getPoissons() const	PX_OVERRIDE;
		virtual		void				setDynamicFriction(PxReal threshold)	PX_OVERRIDE;
		virtual		PxReal				getDynamicFriction() const	PX_OVERRIDE;
		//~PxFEMMaterial

		// PxFEMSoftBodyMaterial
		virtual		void				setDamping(PxReal damping)	PX_OVERRIDE;
		virtual		PxReal				getDamping() const	PX_OVERRIDE;
		virtual		void				setDampingScale(PxReal scale);
		virtual		PxReal				getDampingScale() const;
		virtual		void				setMaterialModel(PxFEMSoftBodyMaterialModel::Enum model);
		virtual		PxFEMSoftBodyMaterialModel::Enum getMaterialModel() const;
		virtual		void				setDeformThreshold(PxReal threshold);
		virtual		PxReal				getDeformThreshold() const;
		virtual		void				setDeformLowLimitRatio(PxReal threshold);
		virtual		PxReal				getDeformLowLimitRatio() const;
		virtual		void				setDeformHighLimitRatio(PxReal threshold);
		virtual		PxReal				getDeformHighLimitRatio() const;
		//~PxFEMSoftBodyMaterial

		PX_FORCE_INLINE static void		getMaterialIndices(PxFEMSoftBodyMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

	private:
		PX_INLINE	void				updateMaterial();

		// PX_SERIALIZATION
	public:
		//~PX_SERIALIZATION
			PxsFEMSoftBodyMaterialCore	mMaterial;
	};

	PX_FORCE_INLINE void NpFEMSoftBodyMaterial::getMaterialIndices(PxFEMSoftBodyMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
	{
		for (PxU32 i = 0; i < materialCount; i++)
			materialIndices[i] = static_cast<NpFEMSoftBodyMaterial*>(materials[i])->mMaterial.mMaterialIndex;
	}
}

#endif
