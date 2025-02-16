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

#ifndef PX_SOFT_BODY_FLAG_H
#define PX_SOFT_BODY_FLAG_H

#include "PxDeformableVolumeFlag.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Deprecated
\see PxDeformableVolumeDataFlag
*/
typedef PX_DEPRECATED PxDeformableVolumeDataFlag PxSoftBodyDataFlag;

/**
\brief Deprecated
\see PxDeformableVolumeDataFlags
*/
typedef PX_DEPRECATED PxDeformableVolumeDataFlags PxSoftBodyDataFlags;

/**
\brief These flags determine what data is read or written when using PxScene::copySoftBodyData()
or PxScene::applySoftBodyData.

\see PxScene::copySoftBodyData, PxScene::applySoftBodyData

\deprecated There is no direct replacement. The data is exposed in the PxSoftBody interface, accessible directly from GPU.
There is no replacement for eTET_REST_POSES, as the data is constant and can be derived from the input collision mesh.
*/
PX_DEPRECATED class PxSoftBodyGpuDataFlag
{
public:
	enum Enum
	{
		eTET_INDICES = 0,			//!< The collision mesh tetrahedron indices (quadruples of int32)
		eTET_REST_POSES = 1,		//!< The collision mesh tetrahedron rest poses (float 3x3 matrices)
		eTET_ROTATIONS = 2,			//!< The collision mesh tetrahedron orientations (quaternions, quadruples of float)
		eTET_POSITION_INV_MASS = 3,	//!< The collision mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
		eSIM_TET_INDICES = 4,		//!< The simulation mesh tetrahedron indices (quadruples of int32)
		eSIM_TET_ROTATIONS = 5,		//!< The simulation mesh tetrahedron orientations (quaternions, quadruples of float)
		eSIM_VELOCITY_INV_MASS = 6,	//!< The simulation mesh vertex velocities and their inverted mass in the 4th component (quadruples of float)
		eSIM_POSITION_INV_MASS = 7  //!< The simulation mesh vertex positions and their inverted mass in the 4th component (quadruples of float)
	};
};

#if !PX_DOXYGEN
}
#endif

#endif
