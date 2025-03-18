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

#ifndef PXG_SIMULATION_CORE_KERNEL_INDICES_H
#define PXG_SIMULATION_CORE_KERNEL_INDICES_H

namespace physx
{

struct PxgSimulationCoreKernelBlockDim
{
	enum
	{
		UPDATE_BODY_EXTERNAL_VELOCITIES				=	256,
		UPDATE_BODIES_AND_SHAPES					=	256,
		UPDATE_LINKS_AND_JOINTCORES					=	256,
		UPDATE_JOINTS								=	256,
		UPDATE_TRANSFORMCACHE_AND_BOUNDARRAY		=	256,
		MERGE_TRANSFORMCACHE_AND_BOUNDARRAY_CHANGES =	256,
		UPDATE_AABBMGR_HANDLES						=	256,
		COMPUTE_FROZEN_UNFROZEN_HISTOGRAM			=	256,
		OUTPUT_FROZEN_UNFROZEN_HISTOGRAM			=	256,
		CREATE_FROZEN_UNFROZEN_ARRAY				=	256,
		NEW_ARTICULATION							=	256,
		RIGID_DYNAMIC_GET_GLOBAL_POSE				=   256,
		RIGID_DYNAMIC_GET_LINVEL					=   256,
		RIGID_DYNAMIC_GET_ANGVEL					= 	256,
		RIGID_DYNAMIC_GET_LINACCEL					=   256,
		RIGID_DYNAMIC_GET_ANGACCEL					= 	256,
		RIGID_DYNAMIC_SET_GLOBAL_POSE				= 	256,
		RIGID_DYNAMIC_SET_LINVEL					= 	256,
		RIGID_DYNAMIC_SET_ANGVEL 					= 	256,
		RIGID_DYNAMIC_SET_FORCE						=	256,
		RIGID_DYNAMIC_SET_TORQUE					=	256,
		D6_JOINT_GET_FORCE							=	256,
		D6_JOINT_GET_TORQUE							=	256
	};
};

struct PxgSimulationCoreKernelGridDim
{
	enum
	{
		UPDATE_BODY_EXTERNAL_VELOCITIES				=   64,
		UPDATE_BODIES_AND_SHAPES					=	64,
		NEW_ARTICULATION							=	64,
		UPDATE_SOFTBODIES							=	64,
		UPDATE_PARTICLESYSTEMS						=	64,
		UPDATE_LINKS_AND_JOINTCORES					=	64,
		UPDATE_JOINTS								=	64,
		UPDATE_TRANSFORMCACHE_AND_BOUNDARRAY		=	256,
		UPDATE_AABBMGR_HANDLES						=	64,
		COMPUTE_FROZEN_UNFROZEN_HISTOGRAM			=	32,	//this has to be 32 because we are doing warp scan
		OUTPUT_FROZEN_UNFROZEN_HISTOGRAM			=	32, //this has to be 32
		CREATE_FROZEN_UNFROZEN_ARRAY				=	64
	};
};

}  

#endif
