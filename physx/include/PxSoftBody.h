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
// Copyright (c) 2008-2024 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_SOFT_BODY_H
#define PX_SOFT_BODY_H

#include "PxDeformableVolume.h"
#include "PxDeformableVolumeFlag.h"
#include "PxFEMParameter.h"
#include "PxSoftBodyFlag.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4435)
#endif


	/**
	\brief Deprecated
	\see PX_MAX_NB_DEFORMABLE_VOLUME_TET
	*/
	#define PX_MAX_NB_SOFTBODY_TET PX_MAX_NB_DEFORMABLE_VOLUME_TET

	/**
	\brief Deprecated
	\see PX_MAX_NB_DEFORMABLE_VOLUME
	*/
	#define PX_MAX_NB_SOFTBODY PX_MAX_NB_DEFORMABLE_VOLUME

	/**
	\brief Deprecated
	\see PxDeformableVolumeFlag
	*/
	typedef PX_DEPRECATED PxDeformableVolumeFlag PxSoftBodyFlag;

	/**
	\brief Deprecated
	\see PxDeformableVolumeFlags
	*/
	typedef PX_DEPRECATED PxDeformableVolumeFlags PxSoftBodyFlags;

	/**
	\brief Deprecated
	\see PxDeformableVolume
	*/
	typedef PX_DEPRECATED PxDeformableVolume PxSoftBody;

	/**
	\brief Deprecated
	\see PxConfigureDeformableVolumeKinematicTarget
	*/
	PX_DEPRECATED PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4 PxConfigureSoftBodyKinematicTarget(const PxVec4& target, bool isActive)
	{
		return PxConfigureDeformableVolumeKinematicTarget(target, isActive);
	}

	/**
	\brief Deprecated
	\see PxConfigureDeformableVolumeKinematicTarget
	*/
	PX_DEPRECATED PX_CUDA_CALLABLE PX_FORCE_INLINE PxVec4 PxConfigureSoftBodyKinematicTarget(const PxVec3& target, bool isActive)
	{
		return PxConfigureDeformableVolumeKinematicTarget(PxVec4(target, 0.0f), isActive);
	}

#if PX_VC
#pragma warning(pop)
#endif


#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
