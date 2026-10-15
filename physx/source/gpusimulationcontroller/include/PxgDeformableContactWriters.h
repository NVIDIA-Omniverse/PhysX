// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXG_DEFORMABLE_CONTACT_WRITERS_H
#define PXG_DEFORMABLE_CONTACT_WRITERS_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPreprocessor.h"
#include "PxNodeIndex.h"
#include "PxgSimulationCoreDesc.h"
#include <vector_types.h>

namespace physx
{

// Deformable-deformable contact writer, for the volume-volume and volume-surface streams - data and device
// methods only, initialization in PxgSoftBodyCore.h. ref1 is always the soft body tetrahedron and ref0 the
// feature that met it, which sb_fem_contact_remap_to_simLaunch relies on to search for the one and look up
// the other.
struct PxgDbDbContactWriter
{
	float4* outPoint;
	float4* outNormalPen;
	float4* outBarycentric0;
	float4*	outBarycentric1;
	PxgDeformablePairInfo* outContactInfo;
	PxU32* totalContactCount;
	PxU32 maxContacts;

	// Everything a contact carries except its barycentrics, which is what the methods below add.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeBaseContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 ref0, PxU64 ref1)
	{
		if (index >= maxContacts)
			return false;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outContactInfo[index].ref0 = ref0;
		outContactInfo[index].ref1 = ref1;

		return true;
	}

	// sb_fem_contact_remap_to_simLaunch writes both barycentrics. It replaces vertexRef0 with the simulation
	// mesh tetrahedron containing that vertex, and tetRef1 with the one containing the contact point.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeVolumeVolumeContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 vertexRef0, PxU64 tetRef1)
	{
		return writeBaseContact(index, contact, normalPen, vertexRef0, tetRef1);
	}

	// The surface is not remapped, so its barycentric is final. The volume's is written by
	// sb_fem_contact_remap_to_simLaunch, which also replaces tetRef with a simulation mesh tetrahedron.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeSurfaceVolumeContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 surfaceRef, const float4& barycentric, PxU64 tetRef)
	{
		// A vertex has no barycentric and belongs in the overload below, which writes none.
		PX_ASSERT(PxgGetDeformableFeatureType(surfaceRef) != PxgDeformableRef::eVERTEX);

		if (!writeBaseContact(index, contact, normalPen, surfaceRef, tetRef))
			return false;

		outBarycentric0[index] = barycentric;

		return true;
	}

	// Vertex overload. No barycentric coordinates are needed for a vertex and the slot is left untouched.
	// The reader methods load it based on the feature type.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeSurfaceVolumeContact(PxU32 index, const float4& contact,
		const float4& normalPen, PxU64 vertexRef, PxU64 tetRef)
	{
		PX_ASSERT(PxgGetDeformableFeatureType(vertexRef) == PxgDeformableRef::eVERTEX);

		return writeBaseContact(index, contact, normalPen, vertexRef, tetRef);
	}
};

// Rigid-deformable contact writer - data and device methods only, initialization in PxgFEMCore.h
// Note: the particle streams have their own writers, PxgParticleDbContactWriter below and
// PxgRigidParticleContactWriter in PxgParticleSystem.h.
struct PxgRigidDbContactWriter
{
	float4* outPoint;
	float4* outNormalPen;
	float4* outBarycentric;
	PxgDeformablePairInfo* outContactInfo;
	// PxU16 matches PxMaterialTableIndex, although PxgShape carries the index as a PxU32.
	PxU16* outRigidMaterialIndex;
	PxU32* totalContactCount;

	//Buffers for sorting
	PxU64* contactByRigid; //value
	PxU32* tempContactByRigid; //the lower 32 bit o value
	PxU32* contactIndexSortedByRigid; //rank

	PxU32 maxNumContacts;

	// The sort key is ref0, which is the rigid participant. Deformable-deformable contacts are not sorted
	// and use PxgDbDbContactWriter instead. Kept separate from writeBaseContact because
	// writeVolumeContactFullWarp calls it on its own, writing the rest of the record across four lanes.
	PX_FORCE_INLINE PX_CUDA_CALLABLE void writeSortKey(PxU32 index, PxU64 ref0)
	{
		contactByRigid[index] = ref0;
		tempContactByRigid[index] = PxgGetContactSortKey(ref0);
		contactIndexSortedByRigid[index] = index;
	}

	// Everything a contact carries except its barycentric and its place in the sort order, which is what
	// the methods below add.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeBaseContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 rigidRef, PxU32 rigidMaterialIndex, PxU64 deformableRef)
	{
		if (index >= maxNumContacts)
			return false;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outContactInfo[index].ref0 = rigidRef;
		outContactInfo[index].ref1 = deformableRef;
		outRigidMaterialIndex[index] = PxU16(rigidMaterialIndex);

		return true;
	}

	// Soft body contacts are generated on the collision mesh, so deformableRef refers to a collision mesh
	// tetrahedron and no barycentric is written. sb_other_contact_remap_to_simLaunch replaces the reference
	// and computes the barycentric, both from the contact point written here.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeVolumeContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 rigidRef, PxU32 rigidMaterialIndex, PxU64 deformableRef)
	{
		if (!writeBaseContact(index, contact, normalPen, rigidRef, rigidMaterialIndex, deformableRef))
			return false;

		writeSortKey(index, rigidRef);

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeSurfaceContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 rigidRef, PxU32 rigidMaterialIndex, PxU64 deformableRef, const float4& barycentric)
	{
		// A vertex has no barycentric and belongs in the overload below, which writes none.
		PX_ASSERT(PxgGetDeformableFeatureType(deformableRef) != PxgDeformableRef::eVERTEX);

		if (!writeBaseContact(index, contact, normalPen, rigidRef, rigidMaterialIndex, deformableRef))
			return false;

		writeSortKey(index, rigidRef);
		outBarycentric[index] = barycentric;

		return true;
	}

	// Vertex overload, as above: nothing is written to the barycentric slot for a vertex contact.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeSurfaceContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 rigidRef, PxU32 rigidMaterialIndex, PxU64 vertexRef)
	{
		PX_ASSERT(PxgGetDeformableFeatureType(vertexRef) == PxgDeformableRef::eVERTEX);

		if (!writeBaseContact(index, contact, normalPen, rigidRef, rigidMaterialIndex, vertexRef))
			return false;

		writeSortKey(index, rigidRef);

		return true;
	}


};

// Particle-deformable contact writer - data and device methods only, initialization in PxgFEMCore.h.
// The stream has no rigid material and is not gathered into a sorted contact array, so neither member
// is present here.
struct PxgParticleDbContactWriter
{
	float4* outPoint;
	float4* outNormalPen;
	float4* outBarycentric;
	PxgDeformablePairInfo* outContactInfo;
	PxU32* totalContactCount;

	//Buffers for sorting
	PxU64* contactByParticle; //value
	PxU32* tempContactByParticle; //the lower 32 bit o value
	PxU32* contactIndexSortedByParticle; //rank

	PxU32 maxNumContacts;

	// The sort key is ref0, which is the particle participant.
	PX_FORCE_INLINE PX_CUDA_CALLABLE void writeSortKey(PxU32 index, PxU64 ref0)
	{
		contactByParticle[index] = ref0;
		tempContactByParticle[index] = PxgGetContactSortKey(ref0);
		contactIndexSortedByParticle[index] = index;
	}

	// Everything a contact carries except its barycentric and its place in the sort order, which is what
	// the methods below add.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeBaseContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 ref0, PxU64 ref1)
	{
		if (index >= maxNumContacts)
			return false;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outContactInfo[index].ref0 = ref0;
		outContactInfo[index].ref1 = ref1;

		return true;
	}

	// Soft body contacts are generated on the collision mesh, so ref1 refers to a collision mesh tetrahedron
	// and no barycentric is written. sb_other_contact_remap_to_simLaunch replaces the reference and computes
	// the barycentric, both from the contact point written here.
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeVolumeContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 ref0, PxU64 ref1)
	{
		if (!writeBaseContact(index, contact, normalPen, ref0, ref1))
			return false;

		writeSortKey(index, ref0);

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeSurfaceContact(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 ref0, PxU64 ref1, const float4& barycentric)
	{
		// No vertex overload here: the cloth side of a particle contact is always a triangle. This
		// records that, and is what fires first if a vertex producer is ever added.
		PX_ASSERT(PxgGetDeformableFeatureType(ref1) != PxgDeformableRef::eVERTEX);

		if (!writeBaseContact(index, contact, normalPen, ref0, ref1))
			return false;

		writeSortKey(index, ref0);
		outBarycentric[index] = barycentric;

		return true;
	}
};

} // namespace physx

#endif // PXG_DEFORMABLE_CONTACT_WRITERS_H
