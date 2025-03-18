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
// Copyright (c) 2008-2025 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef __CONTACT_PATCH_UTILS_CUH__
#define __CONTACT_PATCH_UTILS_CUH__

#include "materialCombiner.cuh"
#include "PxsContactManagerState.h"
#include "PxgContactManager.h"

PX_FORCE_INLINE __device__ PxU32 setContactPointAndForcePointers(PxsContactManagerOutput* PX_RESTRICT cmOutputs, PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU8* PX_RESTRICT startContactPoints, PxU8* PX_RESTRICT startContactForces, PxU32 contactBytesLimit, PxU32 forceBytesLimit, PxU32 workIndex, PxU32 nbContacts)
{
	PxU32 contactByteOffset = 0xFFFFFFFF;
	PxU32 forceAndIndiceByteOffset = 0xFFFFFFFF;

	if (nbContacts)
	{
		PxsContactManagerOutput* output = &cmOutputs[workIndex];

		contactByteOffset = atomicAdd(&(patchAndContactCounters->contactsBytes), sizeof(PxContact) * nbContacts);
		forceAndIndiceByteOffset = atomicAdd(&(patchAndContactCounters->forceAndIndiceBytes), sizeof(PxU32) * nbContacts);

		if ((contactByteOffset + sizeof(PxContact)) > contactBytesLimit)
		{
			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::CONTACT_BUFFER_OVERFLOW);
			contactByteOffset = 0xFFFFFFFF; //overflow
		}

		if ((forceAndIndiceByteOffset + sizeof(PxU32)) > forceBytesLimit)
		{
			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::FORCE_BUFFER_OVERFLOW);
			forceAndIndiceByteOffset = 0xFFFFFFFF; //overflow
		}
	
		if (contactByteOffset != 0xFFFFFFFF)
		{
			output->contactForces = reinterpret_cast<PxReal*>(startContactForces + forceAndIndiceByteOffset);
			output->contactPoints = startContactPoints + contactByteOffset;
		}
		else
		{
			output->contactForces = NULL;
			output->contactPoints = NULL;
		}
	}

	return contactByteOffset;
}

PX_FORCE_INLINE __device__ PxU32 registerContactPatch(PxsContactManagerOutput* PX_RESTRICT cmOutputs, PxgPatchAndContactCounters* PX_RESTRICT patchAndContactCounters,
	PxU32* PX_RESTRICT touchChangeFlags, PxU32* PX_RESTRICT patchChangeFlags, PxU8* PX_RESTRICT startContactPatches, PxU32 patchBytesLimit, PxU32 workIndex, PxU32 nbContacts)
{
	PxU32 allflags = reinterpret_cast<PxU32*>(&((cmOutputs + workIndex)->allflagsStart))[0];

	PxU8 oldStatusFlags = u16Low(u32High(allflags));
	PxU8 statusFlags = oldStatusFlags;

	statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);

	if (nbContacts)
		statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else
		statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	PxU8 prevPatches = u16High(u32Low(allflags)); //Get out the current number of patches to store as the previous frame's number of patches
	bool previouslyHadTouch = oldStatusFlags & PxsContactManagerStatusFlag::eHAS_TOUCH;
	bool prevTouchKnown = oldStatusFlags & PxsContactManagerStatusFlag::eTOUCH_KNOWN;

	PxU8 numPatches = (nbContacts > 0) ? 1 : 0;

	bool currentlyHasTouch = nbContacts > 0;

	const bool change = (previouslyHadTouch ^ currentlyHasTouch) || (!prevTouchKnown);
	touchChangeFlags[workIndex] = change;
	patchChangeFlags[workIndex] = (prevPatches != numPatches);

	reinterpret_cast<PxU32*>(&((cmOutputs + workIndex)->allflagsStart))[0] = merge(merge(prevPatches, statusFlags),
		merge(numPatches, 0));

	cmOutputs[workIndex].nbContacts = nbContacts;

	//fill in patch information
	PxU32 patchIndex = 0xFFFFFFFF;
	if (nbContacts)
	{
		patchIndex = atomicAdd(&(patchAndContactCounters->patchesBytes), sizeof(PxContactPatch));

		if ((patchIndex + sizeof(PxContactPatch)) > patchBytesLimit)
		{
			//patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
			//patchIndex = 0xFFFFFFFF; //overflow

			patchAndContactCounters->setOverflowError(PxgPatchAndContactCounters::PATCH_BUFFER_OVERFLOW);
			patchIndex = 0xFFFFFFFF; //overflow

			statusFlags &= (~PxsContactManagerStatusFlag::eTOUCH_KNOWN);
			statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

			reinterpret_cast<PxU32*>(&((cmOutputs + workIndex)->allflagsStart))[0] = merge(merge(prevPatches, statusFlags), 0);
			cmOutputs[workIndex].nbContacts = 0;

			touchChangeFlags[workIndex] = previouslyHadTouch;
			patchChangeFlags[workIndex] = prevPatches != 0;
		}
		else
		{
			(cmOutputs + workIndex)->contactPatches = startContactPatches + patchIndex;
		}
	}
	return patchIndex;
}

PX_FORCE_INLINE __device__ void insertIntoPatchStream(const PxsMaterialData* PX_RESTRICT materials, PxU8* PX_RESTRICT patchStream, 
	const PxgShape& shape0, const PxgShape& shape1, PxU32 patchIndex, const PxVec3& normal, PxU32 nbContacts)
{
	//write to patch to stream
	if (patchIndex != 0xFFFFFFFF)
	{
		//fill in material
		PxReal restitution, dynamicFriction, staticFriction, damping;
		PxU32 materialFlags;

		PxU16 materialIndex0 = shape0.materialIndex;
		PxU16 materialIndex1 = shape1.materialIndex;

		combineMaterials(materials, materialIndex0,
			materialIndex1,
			materialFlags,
			staticFriction,
			dynamicFriction,
			restitution,
			damping
		);

		PxContactPatch* patch = ((PxContactPatch*)(patchStream + patchIndex));
		patch->normal = normal;
		patch->nbContacts = nbContacts;
		patch->startContactIndex = 0;
		//KS - we could probably compress this further into the header but the complexity might not be worth it
		patch->staticFriction = staticFriction;
		patch->dynamicFriction = dynamicFriction;
		patch->damping = damping;
		patch->restitution = restitution;
		patch->materialIndex0 = materialIndex0;
		patch->materialIndex1 = materialIndex1;
		assert(materialFlags <= PX_MAX_U8);
		patch->materialFlags = PxU8(materialFlags);
		patch->internalFlags = 0;
		patch->mMassModification.linear0 = 1.0f;
		patch->mMassModification.linear1 = 1.0f;
		patch->mMassModification.angular0 = 1.0f;
		patch->mMassModification.angular1 = 1.0f;
	}
}

#endif
