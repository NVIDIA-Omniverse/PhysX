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

#ifndef NP_MPM_MATERIAL_H
#define NP_MPM_MATERIAL_H

#include "common/PxSerialFramework.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "CmRefCountable.h"
#include "PxsMPMMaterialCore.h"
#include "PxMPMMaterial.h"

namespace physx
{
#if PX_ENABLE_FEATURES_UNDER_CONSTRUCTION
	// Compared to other objects, materials are special since they belong to the SDK and not to scenes
	// (similar to meshes). That's why the NpMPMMaterial does have direct access to the core material instead
	// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
	// the materials will be buffered.

	class NpMPMMaterial : public PxMPMMaterial, public PxUserAllocated
	{
	public:
		// PX_SERIALIZATION            
										NpMPMMaterial(PxBaseFlags baseFlags) : PxMPMMaterial(baseFlags), mMaterial(PxEmpty) {}
		virtual		void				resolveReferences(PxDeserializationContext& context);
		static		NpMPMMaterial*		createObject(PxU8*& address, PxDeserializationContext& context);
		static		void				getBinaryMetaData(PxOutputStream& stream);

		void							preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
		void							exportExtraData(PxSerializationContext&) {}
		void							importExtraData(PxDeserializationContext&) {}
		virtual		void				requiresObjects(PxProcessPxBaseCallback&) {}
		//~PX_SERIALIZATION
										NpMPMMaterial(const PxsMPMMaterialCore& desc);
		virtual							~NpMPMMaterial();

		// PxBase
		virtual		void				release()	PX_OVERRIDE;
		//~PxBase

		// PxRefCounted
		virtual		void				acquireReference()	PX_OVERRIDE;
		virtual		PxU32				getReferenceCount() const	PX_OVERRIDE;
		virtual		void				onRefCountZero()	PX_OVERRIDE;
		//~PxRefCounted
		
		// PxParticleMaterial
		virtual		void				setFriction(PxReal friction)	PX_OVERRIDE;
		virtual		PxReal				getFriction() const	PX_OVERRIDE;
		virtual		void				setDamping(PxReal damping)	PX_OVERRIDE;
		virtual		PxReal				getDamping() const	PX_OVERRIDE;
		virtual		void				setAdhesion(PxReal adhesion)	PX_OVERRIDE;
		virtual		PxReal				getAdhesion() const	PX_OVERRIDE;
		virtual		void				setGravityScale(PxReal scale)	PX_OVERRIDE;
		virtual		PxReal				getGravityScale() const	PX_OVERRIDE;
		virtual		void				setAdhesionRadiusScale(PxReal scale)	PX_OVERRIDE;
		virtual		PxReal				getAdhesionRadiusScale() const	PX_OVERRIDE;
		//~PxParticleMaterial

		// PxMPMMaterial
		virtual		void				setStretchAndShearDamping(PxReal stretchAndShearDamping) PX_OVERRIDE;
		virtual		PxReal				getStretchAndShearDamping() const PX_OVERRIDE;
		virtual		void				setRotationalDamping(PxReal rotationalDamping) PX_OVERRIDE;
		virtual		PxReal				getRotationalDamping() const PX_OVERRIDE;
		virtual		void				setDensity(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getDensity() const	PX_OVERRIDE;
		virtual		void				setMaterialModel(PxMPMMaterialModel::Enum)	PX_OVERRIDE;
		virtual		PxMPMMaterialModel::Enum getMaterialModel() const	PX_OVERRIDE;
		virtual		void				setCuttingFlags(PxMPMCuttingFlags cuttingFlags)	PX_OVERRIDE;
		virtual		PxMPMCuttingFlags getCuttingFlags() const	PX_OVERRIDE;
		virtual		void				setSandFrictionAngle(PxReal sandFrictionAngle)	PX_OVERRIDE;
		virtual		PxReal				getSandFrictionAngle() const	PX_OVERRIDE;
		virtual		void				setYieldStress(PxReal yieldStress)	PX_OVERRIDE;
		virtual		PxReal				getYieldStress() const	PX_OVERRIDE;


		virtual		void				setIsPlastic(bool)	PX_OVERRIDE;
		virtual		bool				getIsPlastic() const	PX_OVERRIDE;
		virtual		void				setYoungsModulus(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getYoungsModulus() const	PX_OVERRIDE;
		virtual		void				setPoissons(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getPoissons() const	PX_OVERRIDE;
		virtual		void				setHardening(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getHardening() const	PX_OVERRIDE;
		virtual		void				setCriticalCompression(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getCriticalCompression() const	PX_OVERRIDE;
		virtual		void				setCriticalStretch(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getCriticalStretch() const	PX_OVERRIDE;
		virtual		void				setTensileDamageSensitivity(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getTensileDamageSensitivity() const	PX_OVERRIDE;
		virtual		void				setCompressiveDamageSensitivity(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getCompressiveDamageSensitivity() const	PX_OVERRIDE;
		virtual		void				setAttractiveForceResidual(PxReal)	PX_OVERRIDE;
		virtual		PxReal				getAttractiveForceResidual() const	PX_OVERRIDE;
		//~PxMPMMaterial

		PX_FORCE_INLINE static void		getMaterialIndices(NpMPMMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

	private:
		PX_INLINE	void				updateMaterial();

		// PX_SERIALIZATION
	public:
		//~PX_SERIALIZATION
					PxsMPMMaterialCore	mMaterial;
	};

	PX_FORCE_INLINE void NpMPMMaterial::getMaterialIndices(NpMPMMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
	{
		for (PxU32 i = 0; i < materialCount; i++)
			materialIndices[i] = static_cast<NpMPMMaterial*>(materials[i])->mMaterial.mMaterialIndex;
	}
#endif
}

#endif
