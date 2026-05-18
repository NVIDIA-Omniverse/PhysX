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
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PXG_DEFORMABLE_CONTACT_INFO_H
#define PXG_DEFORMABLE_CONTACT_INFO_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxPreprocessor.h"
#include "PxNodeIndex.h"
#include <vector_types.h>

namespace physx
{

//rigid vs soft body : pairInd0 is rigid , pairInd1 is soft body
//particle vs soft body : pairInd0 is particle, pairInd1 is soft body
//soft body vs soft body : pairInd0 is soft body, pairInd1 is soft body
//soft body vs fem cloth : pairInd0 is soft body, pairInd1 is fem cloth
//fem cloth vs fem cloth : pairInd0 is fem cloth, pairInd1 is fem cloth
struct PX_ALIGN_PREFIX(16) PxgFemFemContactInfo
{
	PxU64 pairInd0;
	PxU32 pairInd1;
	PxU32 auxInd; // Packed 32-bit auxiliary data

	// Bit layout of auxInd (32 bits total):
	//
	//  | 31     | 30       | 29         | 28 - 14          | 13 - 0         |
	//  | Valid  | PairType | InCollision| AuxInd1 (15 bits)| AuxInd0 (14b)  |
	//
	// - Bits [0-13]   : AuxInd0. First auxiliary index (14 bits, max value 16,383)
	// - Bits [14-28]  : AuxInd1. Second auxiliary index (15 bits, max value 32,767)
	// - Bit  [29]     : isInCollision flag (0 = not in contact, 1 = in contact)
	// - Bit  [30]     : Pair type flag (0 = Vertex-Triangle, 1 = Edge-Edge)
	// - Bit  [31]     : Validity flag (0 = Invalid, 1 = Valid)

	// Bit layout masks and shifts for auxInd
	static constexpr PxU32 AUX_IND0_BITS = 14;
	static constexpr PxU32 AUX_IND1_BITS = 15;

	static constexpr PxU32 AUX_IND0_MASK = (1u << AUX_IND0_BITS) - 1;					 // 0x00003FFF
	static constexpr PxU32 AUX_IND1_MASK = ((1u << AUX_IND1_BITS) - 1) << AUX_IND0_BITS; // 0x1FFFC000
	static constexpr PxU32 AUX_IND1_SHIFT = AUX_IND0_BITS;

	static constexpr PxU32 CONTACT_FLAG_SHIFT = 29;
	static constexpr PxU32 PAIR_TYPE_FLAG_SHIFT = 30;
	static constexpr PxU32 VALIDITY_FLAG_SHIFT = 31;

	static constexpr PxU32 CONTACT_FLAG_MASK = 1u << CONTACT_FLAG_SHIFT;
	static constexpr PxU32 PAIR_TYPE_FLAG_MASK = 1u << PAIR_TYPE_FLAG_SHIFT;
	static constexpr PxU32 VALIDITY_FLAG_MASK = 1u << VALIDITY_FLAG_SHIFT;

	// Set auxiliary indices
	PX_FORCE_INLINE PX_CUDA_CALLABLE void setAuxInd0(PxU32 id) { auxInd = (auxInd & ~AUX_IND0_MASK) | (id & AUX_IND0_MASK); }

	PX_FORCE_INLINE PX_CUDA_CALLABLE void setAuxInd1(PxU32 id)
	{
		auxInd = (auxInd & ~AUX_IND1_MASK) | ((id & ((1u << AUX_IND1_BITS) - 1)) << AUX_IND1_SHIFT);
	}

	// Get auxiliary indices
	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 getAuxInd0() const { return auxInd & AUX_IND0_MASK; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 getAuxInd1() const { return (auxInd >> AUX_IND1_SHIFT) & ((1u << AUX_IND1_BITS) - 1); }

	// Mark validity
	PX_FORCE_INLINE PX_CUDA_CALLABLE void markInvalid() { auxInd &= ~VALIDITY_FLAG_MASK; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE void markValid() { auxInd |= VALIDITY_FLAG_MASK; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE void markValidity(bool isValid)
	{
		auxInd = (auxInd & ~VALIDITY_FLAG_MASK) | (static_cast<PxU32>(isValid) << VALIDITY_FLAG_SHIFT);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isValidPair() const { return (auxInd & VALIDITY_FLAG_MASK) != 0; }

	// Mark pair type
	PX_FORCE_INLINE PX_CUDA_CALLABLE void markVertexTrianglePair() { auxInd &= ~PAIR_TYPE_FLAG_MASK; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE void markEdgeEdgePair() { auxInd |= PAIR_TYPE_FLAG_MASK; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isVertexTrianglePair() const { return (auxInd & PAIR_TYPE_FLAG_MASK) == 0; }

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isEdgeEdgePair() const { return (auxInd & PAIR_TYPE_FLAG_MASK) != 0; }

	// Mark collision status (bit 29)
	PX_FORCE_INLINE PX_CUDA_CALLABLE void markInCollision(bool inContact)
	{
		auxInd = (auxInd & ~CONTACT_FLAG_MASK) | (static_cast<PxU32>(inContact) << CONTACT_FLAG_SHIFT);
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isInCollision() const { return (auxInd & CONTACT_FLAG_MASK) != 0; }

} PX_ALIGN_SUFFIX(16);

// Soft body contact writer - data and device methods only, initialization in PxgSoftBodyCore.h
// Note: PxgSoftBodySoftBodyConstraintBlock is in PxgDeformableConstraints.h
struct PxgSoftBodyContactWriter
{
	float4* outPoint;
	float4* outNormalPen;
	float4* outBarycentric0;
	float4*	outBarycentric1;
	PxgFemFemContactInfo* outContactInfo;
	PxU32* totalContactCount;
	PxU32 maxContacts;

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric0, const float4& barycentric1,
		PxU32 pairId0, PxU32 pairId1)
	{
		if (index >= maxContacts)
			return false;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;

		outBarycentric0[index] = barycentric0;
		outBarycentric1[index] = barycentric1;

		outContactInfo[index].pairInd0 = pairId0;
		outContactInfo[index].pairInd1 = pairId1;

		return true;
	}
};

struct PX_ALIGN_PREFIX(16) PxgFemOtherContactInfo
{
	PxU64 pairInd0; // Rigid/Particle
	PxU32 pairInd1; // Fem id
	PxU32 rigidMatInd_isInCollision;

	// rigidMatInd_isInCollision encodes both rigid material index and collision state
	// Bit layout of rigidMatInd_isInCollision (32 bits total):
	//
	//  | 31          | 30 - 0              |
	//  |-------------|---------------------|
	//  | isCollision | rigidMaterialIndex  |
	//
	// - Bits [0-30]   : rigidMaterialIndex (up to 2^31 materials)
	// - Bit [31]      : isInCollision flag (0 = not in contact, 1 = in contact)

	static constexpr PxU32 COLLISION_FLAG_BIT = 31;
	static constexpr PxU32 COLLISION_FLAG_MASK = 1u << COLLISION_FLAG_BIT;
	static constexpr PxU32 RIGID_MAT_INDEX_MASK = ~COLLISION_FLAG_MASK;

	PX_FORCE_INLINE PX_CUDA_CALLABLE void setRigidMaterialIndex(PxU32 matIndex)
	{
		rigidMatInd_isInCollision = (rigidMatInd_isInCollision & COLLISION_FLAG_MASK) | (matIndex & RIGID_MAT_INDEX_MASK);
	}
	PX_FORCE_INLINE PX_CUDA_CALLABLE void markInCollision(bool inContact)
	{
		if(inContact)
			rigidMatInd_isInCollision |= COLLISION_FLAG_MASK;
		else
			rigidMatInd_isInCollision &= ~COLLISION_FLAG_MASK;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE PxU32 getRigidMaterialIndex() const { return rigidMatInd_isInCollision & RIGID_MAT_INDEX_MASK; }
	PX_FORCE_INLINE PX_CUDA_CALLABLE bool isInCollision() const { return (rigidMatInd_isInCollision & COLLISION_FLAG_MASK) != 0; }
}
PX_ALIGN_SUFFIX(16);

// FEM contact writer struct - data and device methods only, initialization in PxgFEMCore.h
// Note: PxgParticleContactWriter is in PxgParticleSystem.h
struct PxgFEMContactWriter
{
	float4* outPoint;
	float4* outNormalPen;
	float4* outBarycentric;
	PxgFemOtherContactInfo* outContactInfo;
	PxU32* totalContactCount;

	//Buffers for sorting. X either stands for Rigid or Particle
	PxU64* contactByX; //value
	PxU32* tempContactByX; //the lower 32 bit o value
	PxU32* contactIndexSortedByX; //rank
	PxU32* contactSortedByX;

	PxU32 maxNumContacts;

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContactCore(PxU32 index, const float4& contact, const float4& normalPen, PxU64 rigidId)
	{
		if (index >= maxNumContacts)
			return false;

		contactByX[index] = rigidId;
		tempContactByX[index] = PxU32(rigidId & 0xffffffff);
		contactIndexSortedByX[index] = index;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContactNoBarycentric(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 pairInd0, PxU32 pairInd1, PxU64 rigidId)
	{
		if (index >= maxNumContacts)
			return false;

		contactByX[index] = rigidId;
		tempContactByX[index] = PxU32(rigidId & 0xffffffff);
		contactIndexSortedByX[index] = index;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outContactInfo[index].pairInd0 = pairInd0;
		outContactInfo[index].pairInd1 = pairInd1;

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeRigidVsDeformableContactNoBarycentric(PxU32 index, const float4& contact, const float4& normalPen,
		PxU64 pairInd0, PxU32 pairInd1, PxU64 rigidId, PxU32 rigidBodyMaterialId)
	{
		if (index >= maxNumContacts)
			return false;

		contactByX[index] = rigidId;
		tempContactByX[index] = PxU32(rigidId & 0xffffffff);
		contactIndexSortedByX[index] = index;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outContactInfo[index].pairInd0 = pairInd0;
		outContactInfo[index].pairInd1 = pairInd1;
		outContactInfo[index].setRigidMaterialIndex(rigidBodyMaterialId);
		outContactInfo[index].markInCollision(false);

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
		PxU64 pairInd0, PxU32 pairInd1, PxU64 id)
	{
		if (index >= maxNumContacts)
			return false;

		writeContactNoBarycentric(index, contact, normalPen, pairInd0, pairInd1, id);
		outBarycentric[index] = barycentric;

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
		PxU64 pairInd0, PxU32 pairInd1, PxNodeIndex rigidId)
	{
		return writeContact(index, contact, normalPen, barycentric, pairInd0, pairInd1, rigidId.getInd());
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeRigidVsDeformableContact(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
		PxU64 pairInd0, PxU32 pairInd1, PxU32 rigidMaterialIndex, PxNodeIndex rigidId)
	{
		bool result = writeContactCore(index, contact, normalPen, rigidId.getInd());
		if (result)
		{
			outBarycentric[index] = barycentric;
			PxgFemOtherContactInfo* femRigidInfo = reinterpret_cast<PxgFemOtherContactInfo*>(outContactInfo);
			femRigidInfo[index].pairInd0 = pairInd0;
			femRigidInfo[index].pairInd1 = pairInd1;
			femRigidInfo[index].setRigidMaterialIndex(rigidMaterialIndex);
			femRigidInfo[index].markInCollision(false);
		}
		return result;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeRigidVsDeformableContact32(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
		PxU64 pairInd0, PxU32 pairInd1, PxU32 rigidMaterialIndex, PxU32 contactSortedByRigid_)
	{
		if (index >= maxNumContacts)
			return false;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outBarycentric[index] = barycentric;

		PxgFemOtherContactInfo* femRigidInfo = reinterpret_cast<PxgFemOtherContactInfo*>(outContactInfo);
		femRigidInfo[index].pairInd0 = pairInd0;
		femRigidInfo[index].pairInd1 = pairInd1;
		femRigidInfo[index].setRigidMaterialIndex(rigidMaterialIndex);
		femRigidInfo[index].markInCollision(false);

		contactSortedByX[index] = contactSortedByRigid_;
		contactIndexSortedByX[index] = index;

		return true;
	}

	PX_FORCE_INLINE PX_CUDA_CALLABLE bool writeContact32(PxU32 index, const float4& contact, const float4& normalPen, const float4& barycentric,
		PxU64 pairInd0, PxU32 pairInd1, PxU32 contactSortedByRigid_)
	{
		if (index >= maxNumContacts)
			return false;

		outPoint[index] = contact;
		outNormalPen[index] = normalPen;
		outBarycentric[index] = barycentric;
		outContactInfo[index].pairInd0 = pairInd0;
		outContactInfo[index].pairInd1 = pairInd1;

		contactSortedByX[index] = contactSortedByRigid_;
		contactIndexSortedByX[index] = index;

		return true;
	}
};

} // namespace physx

#endif // PXG_DEFORMABLE_CONTACT_INFO_H
