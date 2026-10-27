// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_DEFORMABLE_SURFACE_MATERIAL_H
#define NP_DEFORMABLE_SURFACE_MATERIAL_H

#include "common/PxSerialFramework.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "CmRefCountable.h"
#include "PxsDeformableSurfaceMaterialCore.h"

namespace physx
{

// Compared to other objects, materials are special since they belong to the SDK and not to scenes
// (similar to meshes). That's why the NpDeformableSurfaceMaterial does have direct access to the core material instead
// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
// the materials will be buffered.

class NpDeformableSurfaceMaterial : public PxDeformableSurfaceMaterial, public PxUserAllocated
{
public:
	// PX_SERIALIZATION
												NpDeformableSurfaceMaterial(PxBaseFlags baseFlags) : PxDeformableSurfaceMaterial(baseFlags), mMaterial(PxEmpty) {}
	virtual		void							resolveReferences(PxDeserializationContext& context);
	static		NpDeformableSurfaceMaterial*	createObject(PxU8*& address, PxDeserializationContext& context);

				void							preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
				void							exportExtraData(PxSerializationContext&) {}
				void							importExtraData(PxDeserializationContext&) {}
	virtual		void							requiresObjects(PxProcessPxBaseCallback&) {}
	//~PX_SERIALIZATION
												NpDeformableSurfaceMaterial(const PxsDeformableSurfaceMaterialCore& desc);
	virtual										~NpDeformableSurfaceMaterial();

	// PxBase
	virtual		void							release()	PX_OVERRIDE;
	//~PxBase

	// PxRefCounted
	virtual		void							acquireReference()	PX_OVERRIDE;
	virtual		PxU32							getReferenceCount() const	PX_OVERRIDE;
	virtual		void							onRefCountZero()	PX_OVERRIDE;
	//~PxRefCounted

	// PxDeformableMaterial
	virtual		void							setYoungsModulus(PxReal young)	PX_OVERRIDE;
	virtual		PxReal							getYoungsModulus() const	PX_OVERRIDE;
	virtual		void							setPoissons(PxReal poisson)	PX_OVERRIDE;
	virtual		PxReal							getPoissons() const	PX_OVERRIDE;
	virtual		void							setDynamicFriction(PxReal threshold)	PX_OVERRIDE;
	virtual		PxReal							getDynamicFriction() const	PX_OVERRIDE;
	//~PxDeformableMaterial

	// PxDeformableSurfaceMaterial
	virtual		void							setThickness(PxReal thickness)	PX_OVERRIDE;
	virtual		PxReal							getThickness() const	PX_OVERRIDE;
	virtual		void							setBendingStiffness(PxReal bendingStiffness)	PX_OVERRIDE;
	virtual		PxReal							getBendingStiffness() const	PX_OVERRIDE;
	//~PxDeformableSurfaceMaterial

	virtual		void							setElasticityDamping(PxReal damping)	PX_OVERRIDE;
	virtual		PxReal							getElasticityDamping() const	PX_OVERRIDE;
	virtual		void							setBendingDamping(PxReal bendingDamping)	PX_OVERRIDE;
	virtual		PxReal							getBendingDamping() const	PX_OVERRIDE;

	PX_FORCE_INLINE static void					getMaterialIndices(PxDeformableSurfaceMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

private:
	PX_INLINE	void							updateMaterial();

	// PX_SERIALIZATION
public:
	//~PX_SERIALIZATION
		PxsDeformableSurfaceMaterialCore		mMaterial;
};

PX_FORCE_INLINE void NpDeformableSurfaceMaterial::getMaterialIndices(PxDeformableSurfaceMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
{
	for (PxU32 i = 0; i < materialCount; i++)
		materialIndices[i] = static_cast<NpDeformableSurfaceMaterial*>(materials[i])->mMaterial.mMaterialIndex;
}

} // namespace physx

#endif // NP_DEFORMABLE_SURFACE_MATERIAL_H
