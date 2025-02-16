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

#ifndef PX_DEFORMABLE_VOLUME_FLAGS_H
#define PX_DEFORMABLE_VOLUME_FLAGS_H

#include "PxPhysXConfig.h"
#include "foundation/PxFlags.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Flags to enable or disable special modes of a PxDeformableVolume instance
*/
struct PxDeformableVolumeFlag
{
	enum Enum
	{
		eCOMPUTE_STRESS_TENSOR = 1 << 0,				//!< Enables computation of a Cauchy stress tensor for every tetrahedron in the simulation mesh. The tensors can be accessed through the deformable volume direct API
		ePARTIALLY_KINEMATIC = 1 << 1,					//!< Enables partially kinematic motion of the collision and simulation mesh.
		eDISPLAY_SIM_MESH PX_DEPRECATED = 1 << 2,		//!< Deprecated
		eDISABLE_SELF_COLLISION PX_DEPRECATED = 1 << 3,	//!< Deprecated, use PxDeformableBodyFlag::eDISABLE_SELF_COLLISION instead
		eENABLE_CCD PX_DEPRECATED = 1 << 4,				//!< Deprecated, use PxDeformableBodyFlag::eENABLE_SPECULATIVE_CCD
		eKINEMATIC PX_DEPRECATED = 1 << 5				//!< Deprecated, use PxDeformableBodyFlag::eKINEMATIC instead
	};
};

typedef PxFlags<PxDeformableVolumeFlag::Enum, PxU16> PxDeformableVolumeFlags;

/**
\brief Identifies the buffers of a PxDeformableVolume instance.

\see PxDeformableVolume::markDirty()
*/
struct PxDeformableVolumeDataFlag
{
	enum Enum
	{
		eNONE = 0,

		ePOSITION_INVMASS = 1 << 0,             //!< The collision mesh's positions
		eSIM_POSITION_INVMASS = 1 << 1,         //!< The simulation mesh's positions and inverse masses
		eSIM_VELOCITY = 1 << 2,                 //!< The simulation mesh's velocities
		eREST_POSITION_INVMASS = 1 << 3,        //!< The collision mesh's rest positions

		eALL = ePOSITION_INVMASS | eSIM_POSITION_INVMASS | eSIM_VELOCITY | eREST_POSITION_INVMASS
	};
};

typedef PxFlags<PxDeformableVolumeDataFlag::Enum, PxU32> PxDeformableVolumeDataFlags;

#if !PX_DOXYGEN
}
#endif

#endif // PX_DEFORMABLE_VOLUME_FLAGS_H
