// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef NP_MATERIAL_H
#define NP_MATERIAL_H

#include "common/PxSerialFramework.h"
#include "foundation/PxUserAllocated.h"
#include "foundation/PxUtilities.h"
#include "CmRefCountable.h"
#include "PxsMaterialCore.h"

namespace physx
{
// Compared to other objects, materials are special since they belong to the SDK and not to scenes
// (similar to meshes). That's why the NpMaterial does have direct access to the core material instead
// of having a buffered interface for it. Scenes will have copies of the SDK material table and there
// the materials will be buffered.

class NpMaterial : public PxMaterial, public PxUserAllocated
{
public:
// PX_SERIALIZATION            
									NpMaterial(PxBaseFlags baseFlags) : PxMaterial(baseFlags), mMaterial(PxEmpty) {}
	virtual		void				resolveReferences(PxDeserializationContext& context);
	static		NpMaterial*			createObject(PxU8*& address, PxDeserializationContext& context);

				void				preExportDataReset() { Cm::RefCountable_preExportDataReset(*this); }
				void				exportExtraData(PxSerializationContext&) {}
				void				importExtraData(PxDeserializationContext&) {}
	virtual		void				requiresObjects(PxProcessPxBaseCallback&){}
//~PX_SERIALIZATION
									NpMaterial(const PxsMaterialCore& desc);
	virtual							~NpMaterial();

	// PxBase
	virtual		void				release()	PX_OVERRIDE;
	//~PxBase

	// PxRefCounted
	virtual		void				acquireReference()	PX_OVERRIDE;
	virtual		PxU32				getReferenceCount() const	PX_OVERRIDE;
	virtual		void				onRefCountZero()	PX_OVERRIDE;
	//~PxRefCounted

	// PxMaterial
	virtual		void				setDynamicFriction(PxReal)	PX_OVERRIDE;
	virtual		PxReal				getDynamicFriction() const	PX_OVERRIDE;
	virtual		void				setStaticFriction(PxReal)	PX_OVERRIDE;
	virtual		PxReal				getStaticFriction() const	PX_OVERRIDE;
	virtual		void				setRestitution(PxReal)	PX_OVERRIDE;
	virtual		PxReal				getRestitution() const	PX_OVERRIDE; 
	virtual		void				setDamping(PxReal)	PX_OVERRIDE;
	virtual		PxReal				getDamping() const	PX_OVERRIDE;
	virtual		void				setFlag(PxMaterialFlag::Enum flag, bool value)	PX_OVERRIDE;
	virtual		void				setFlags(PxMaterialFlags inFlags)	PX_OVERRIDE;
	virtual		PxMaterialFlags		getFlags() const	PX_OVERRIDE;
	virtual		void				setFrictionCombineMode(PxCombineMode::Enum)	PX_OVERRIDE;
	virtual		PxCombineMode::Enum	getFrictionCombineMode() const	PX_OVERRIDE;
	virtual		void				setRestitutionCombineMode(PxCombineMode::Enum)	PX_OVERRIDE;
	virtual		PxCombineMode::Enum	getRestitutionCombineMode() const	PX_OVERRIDE;
	virtual		void				setDampingCombineMode(PxCombineMode::Enum combMode) PX_OVERRIDE;
	virtual		PxCombineMode::Enum	getDampingCombineMode() const PX_OVERRIDE;

	//~PxMaterial

	PX_FORCE_INLINE static void		getMaterialIndices(PxMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount);

private:
	PX_INLINE	void				updateMaterial();

// PX_SERIALIZATION
public:
//~PX_SERIALIZATION
				PxsMaterialCore		mMaterial;
};

PX_FORCE_INLINE void NpMaterial::getMaterialIndices(PxMaterial*const* materials, PxU16* materialIndices, PxU32 materialCount)
{
	for(PxU32 i=0; i < materialCount; i++)
		materialIndices[i] = static_cast<NpMaterial*>(materials[i])->mMaterial.mMaterialIndex;
}
}

#endif
