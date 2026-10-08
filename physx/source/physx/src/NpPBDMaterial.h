// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_PBD_MATERIAL_H
#define NP_PBD_MATERIAL_H

#include "common/PxSerialFramework.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "CmRefCountable.h"
#include "PxsPBDMaterialCore.h"
#include "PxPBDMaterial.h"

namespace physx
{
	// Compared to other objects, materials are special since they belong to the SDK and not to scenes
	// (similar to meshes). That's why the NpPBDMaterial does have direct access to the core material instead
	// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
	// the materials will be buffered.

	class NpPBDMaterial : public PxPBDMaterial, public PxUserAllocated
	{
	public:
		// PX_SERIALIZATION            
										NpPBDMaterial(PxBaseFlags baseFlags) : PxPBDMaterial(baseFlags), mMaterial(PxEmpty) {}
		virtual		void				resolveReferences(PxDeserializationContext& context);
		static		NpPBDMaterial*		createObject(PxU8*& address, PxDeserializationContext& context);

		void							preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
		void							exportExtraData(PxSerializationContext&) {}
		void							importExtraData(PxDeserializationContext&) {}
		virtual		void				requiresObjects(PxProcessPxBaseCallback&) {}
		//~PX_SERIALIZATION
										NpPBDMaterial(const PxsPBDMaterialCore& desc);
		virtual							~NpPBDMaterial();

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

		// PxPBDMaterial
		virtual		void				setViscosity(PxReal viscosity)	PX_OVERRIDE;
		virtual		PxReal				getViscosity() const	PX_OVERRIDE;
		virtual		void				setVorticityConfinement(PxReal vorticityConfinement)	PX_OVERRIDE;
		virtual		PxReal				getVorticityConfinement() const	PX_OVERRIDE;
		virtual		void				setSurfaceTension(PxReal surfaceTension)	PX_OVERRIDE;
		virtual		PxReal				getSurfaceTension() const	PX_OVERRIDE;
		virtual		void				setCohesion(PxReal cohesion)	PX_OVERRIDE;
		virtual		PxReal				getCohesion() const	PX_OVERRIDE;
		virtual		void				setLift(PxReal lift)	PX_OVERRIDE;
		virtual		PxReal				getLift() const	PX_OVERRIDE;
		virtual		void				setDrag(PxReal drag)	PX_OVERRIDE;
		virtual		PxReal				getDrag() const	PX_OVERRIDE;
		virtual		void				setCFLCoefficient(PxReal coefficient)	PX_OVERRIDE;
		virtual		PxReal				getCFLCoefficient() const	PX_OVERRIDE;
		virtual		void				setParticleFrictionScale(PxReal scale)	PX_OVERRIDE;
		virtual		PxReal				getParticleFrictionScale() const	PX_OVERRIDE;
		virtual		void				setParticleAdhesionScale(PxReal adhesion)	PX_OVERRIDE;
		virtual		PxReal				getParticleAdhesionScale() const	PX_OVERRIDE;
		//~PxPBDMaterial

		PX_FORCE_INLINE static void		getMaterialIndices(NpPBDMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

	private:
		PX_INLINE	void				updateMaterial();

		// PX_SERIALIZATION
	public:
		//~PX_SERIALIZATION
					PxsPBDMaterialCore	mMaterial;
	};

	PX_FORCE_INLINE void NpPBDMaterial::getMaterialIndices(NpPBDMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
	{
		for (PxU32 i = 0; i < materialCount; i++)
			materialIndices[i] = static_cast<NpPBDMaterial*>(materials[i])->mMaterial.mMaterialIndex;
	}
}

#endif
