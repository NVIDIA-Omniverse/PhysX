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
// Copyright (c) 2008-2022 NVIDIA Corporation. All rights reserved.
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

	Some flags are only relevant to read and write data using the direct GPU API of PxScene.

	@see PxArticulationCache, PxScene::copyArticulationData, PxScene::applyArticulationData
	*/
	class PxArticulationCacheFlag
	{
	public:
		enum Enum
		{
			eVELOCITY = (1 << 0),			//!< The joint velocities, see PxArticulationCache::jointVelocity.
			eACCELERATION = (1 << 1),		//!< The joint accelerations, see PxArticulationCache::jointAcceleration.
			ePOSITION = (1 << 2),			//!< The joint positions, see PxArticulationCache::jointPosition.
			eFORCE = (1 << 3),				//!< The joint forces, see PxArticulationCache::jointForce.
			eLINK_VELOCITY = (1 << 4),		//!< The link velocities, see PxArticulationCache::linkVelocity.
			eLINK_ACCELERATION = (1 << 5),	//!< The link accelerations, see PxArticulationCache::linkAcceleration.
			eLINK_TRANSFORM = (1 << 6),		//!< (GPU API only) The link transforms including root link, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eROOT_TRANSFORM = (1 << 7),		//!< Root link transform, see PxArticulationCache::rootLinkData.
			eROOT_VELOCITIES = (1 << 8),	//!< Root link velocities (read/write) and accelerations (read), see PxArticulationCache::rootLinkData.
			eLINKFORCE = (1 << 9),			//!< (GPU API only) Forces to apply to links, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eLINKTORQUE = (1 << 10),		//!< (GPU API only) Torques to apply to links, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eJOINT_TARGET_VELOCITY = (1 << 11),	 //!< (GPU API only) The velocity targets for the joint drives, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eJOINT_TARGET_POSITION = (1 << 12),	 //!< (GPU API only) The position targets for the joint drives, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eSENSOR_FORCES = (1<<13),		//!< The spatial sensor forces, see PxArticulationCache::sensorForces.
			eJOINT_SOLVER_FORCES = (1 << 14),  //!< Solver constraint joint forces, see PxArticulationCache::jointSolverForces.
			eFIXED_TENDON = (1 << 15),		//!< (GPU API only) Fixed tendon data, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eFIXED_TENDON_JOINT = (1 << 16),  //!< (GPU API only) Fixed tendon joint data, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eSPATIAL_TENDON = (1 << 17),	//!< (GPU API only) Spatial tendon data, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eSPATIAL_TENDON_ATTACHMENT = (1 << 18),  //!< (GPU API only) Spatial tendon attachment data, see PxScene::copyArticulationData and PxScene::applyArticulationData.
			eALL = (eVELOCITY | eACCELERATION | ePOSITION | eLINK_VELOCITY | eLINK_ACCELERATION | eLINK_TRANSFORM | eROOT_TRANSFORM | eROOT_VELOCITIES)
		};
	};

	typedef PxFlags<PxArticulationCacheFlag::Enum, PxU32> PxArticulationCacheFlags;
	PX_FLAGS_OPERATORS(PxArticulationCacheFlag::Enum, PxU32)

#if !PX_DOXYGEN
}
#endif

#endif
