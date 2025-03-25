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

#ifndef PX_ARTICULATION_FLAG_H
#define PX_ARTICULATION_FLAG_H

#include "PxPhysXConfig.h"
#include "foundation/PxFlags.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief These flags determine what data is read or written to the internal articulation data via cache.

	\see PxArticulationCache PxArticulationReducedCoordinate::copyInternalStateToCache PxArticulationReducedCoordinate::applyCache
	*/
	class PxArticulationCacheFlag
	{
	public:
		enum Enum
		{
			eVELOCITY = (1 << 0),						//!< The joint velocities, see PxArticulationCache::jointVelocity.
			eACCELERATION = (1 << 1),					//!< The joint accelerations, see PxArticulationCache::jointAcceleration.
			ePOSITION = (1 << 2),						//!< The joint positions, see PxArticulationCache::jointPosition.
			eFORCE = (1 << 3),							//!< The joint forces, see PxArticulationCache::jointForce.
			eLINK_VELOCITY = (1 << 4),					//!< The link velocities, see PxArticulationCache::linkVelocity. Link velocities cannot be set except for the root link velocity via  PxArticulationCache::rootLinkData.
			eLINK_ACCELERATION = (1 << 5),				//!< The link accelerations, see PxArticulationCache::linkAcceleration.
			eROOT_TRANSFORM = (1 << 6),					//!< The root link transform, see PxArticulationCache::rootLinkData.
			eROOT_VELOCITIES = (1 << 7),				//!< The root link velocities (read/write) and accelerations (read), see PxArticulationCache::rootLinkData.
			eLINK_INCOMING_JOINT_FORCE = (1 << 10),		//!< The link incoming joint forces, see PxArticulationCache::linkIncomingJointForce.
			eJOINT_TARGET_POSITIONS = (1 << 11),		//!< The joint target positions, see PxArticulationCache::jointTargetPositions.
			eJOINT_TARGET_VELOCITIES = (1 << 12),		//!< The joint target velocities, see PxArticulationCache::jointTargetVelocities.
			eLINK_FORCE = (1 << 13),					//!< The link forces, see PxArticulationCache::linkForce.
			eLINK_TORQUE = (1 << 14),					//!< The link torques, see PxArticulationCache::linkTorque.
			eALL = (eVELOCITY | eACCELERATION | ePOSITION | eLINK_VELOCITY | eLINK_ACCELERATION | eROOT_TRANSFORM | eROOT_VELOCITIES)
		};
	};

	typedef PxFlags<PxArticulationCacheFlag::Enum, PxU32> PxArticulationCacheFlags;
	PX_FLAGS_OPERATORS(PxArticulationCacheFlag::Enum, PxU32)

#if !PX_DOXYGEN
}
#endif

#endif
