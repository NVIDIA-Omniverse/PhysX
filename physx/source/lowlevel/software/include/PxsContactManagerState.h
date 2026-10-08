// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#ifndef PXS_CONTACT_MANAGER_STATE_H
#define PXS_CONTACT_MANAGER_STATE_H

#include "foundation/PxSimpleTypes.h"

namespace physx
{
	/**
	There is an implicit 1:1 mapping between PxgContactManagerInput and PxsContactManagerOutput. The structures are split because PxgNpContactManagerInput contains constant
	data that is produced by the CPU code and PxgNpContactManagerOutput contains per-frame contact information produced by the NP.

	There is also a 1:1 mapping between the PxgNpContactManager and PxsContactManager. This mapping is handled within the PxgNPhaseCore.

	The previous contact states are implicitly cached in PxsContactManager and will be propagated to the solver. Friction correlation is also done implicitly using cached 
	information in PxsContactManager.
	The NP will produce a list of pairs that found/lost patches for the solver along with updating the PxgNpContactManagerOutput for all pairs.
	*/
	struct PxsContactManagerStatusFlag
	{
		enum Enum
		{
			eHAS_NO_TOUCH				= (1 << 0),
			eHAS_TOUCH					= (1 << 1),
			//eHAS_SOLVER_CONSTRAINTS		= (1 << 2),
			eREQUEST_CONSTRAINTS		= (1 << 3),
			eHAS_CCD_RETOUCH			= (1 << 4),	// Marks pairs that are touching at a CCD pass and were touching at discrete collision or at a previous CCD pass already
													// but we can not tell whether they lost contact in a pass before. We send them as pure eNOTIFY_TOUCH_CCD events to the 
													// contact report callback if requested.
			eDIRTY_MANAGER				= (1 << 5),
			eTOUCH_KNOWN				= eHAS_NO_TOUCH | eHAS_TOUCH,	// The touch status is known (if narrowphase never ran for a pair then no flag will be set)
			eSTATIC_OR_KINEMATIC		= (1 << 6)
		};
	};
		
	struct PX_ALIGN_PREFIX(16) PxsContactManagerOutput
	{
		PxU8* contactPatches;				//Start index/ptr for contact patches
		PxU8* contactPoints;				//Start index/ptr for contact points
		PxReal* contactForces;				//Start index/ptr for contact forces
		PxU8* frictionPatches;				//Contact patches friction information
		PxU8 allflagsStart;					//padding for compatibility with existing code
		PxU8 nbPatches;						//Num patches
		PxU8 statusFlag;					//Status flag (has touch etc.)
		PxU8 prevPatches;					//Previous number of patches
		PxU16 nbContacts;					//Num contacts
		PxU16 flags;						//Not really part of outputs, but we have 4 bytes of padding, so why not?
		PxU8 pad[8];

		PX_FORCE_INLINE PxU32* getInternalFaceIndice()	const
		{
			return contactForces ? reinterpret_cast<PxU32*>(contactForces + nbContacts) : NULL;
		}
	} 
	PX_ALIGN_SUFFIX(16);
	PX_COMPILE_TIME_ASSERT((sizeof(PxsContactManagerOutput) & 0xf) == 0);

	struct PX_ALIGN_PREFIX(4) PxsContactManagerOutputCounts
	{
		PxU8 nbPatches;						//Num patches
		PxU8 prevPatches;					//Previous number of patches
		PxU8 statusFlag;					//Status flag;
		PxU8 unused;						//Unused
	} PX_ALIGN_SUFFIX(4);

	struct PX_ALIGN_PREFIX(8) PxsTorsionalFrictionData
	{
		PxReal mTorsionalPatchRadius;
		PxReal mMinTorsionalRadius;

		PxsTorsionalFrictionData() {}
		PxsTorsionalFrictionData(const PxReal patchRadius, const PxReal minPatchRadius) :
			mTorsionalPatchRadius(patchRadius), mMinTorsionalRadius(minPatchRadius) {}
	} PX_ALIGN_SUFFIX(8);
}

#endif
