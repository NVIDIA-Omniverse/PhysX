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

#ifndef PXG_ARTICULATION_CORE_KERNEL_INDICES_H
#define PXG_ARTICULATION_CORE_KERNEL_INDICES_H

namespace physx
{

	struct PxgArticulationCoreKernelBlockDim
	{
		enum
		{
			COMPUTE_UNCONSTRAINED_VELOCITES = 64,
			UPDATE_BODIES = 128,
			SOLVE_INTERNAL_CONSTRAINTS = 32,
			COMPUTE_UNCONSTRAINED_SPATIAL_INERTIA = 32,
			COMPUTE_UNCONSTRAINED_SPATIAL_INERTIA_PARTIAL = 64,
			COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT = 512,
			APPLY_ARTI_STATE = 512,
			ARTI_GET_DOF_STATES = 512,
			ARTI_GET_TRANSFORM_STATES = 512,
			ARTI_GET_VELOCITY_STATES = 480,
			ARTI_GET_SPATIAL_FORCE_STATES = 512,
			ARTI_SET_DOF_STATES = 512,
			ARTI_SET_ROOT_GLOBAL_POSE_STATE = 512,
			ARTI_SET_ROOT_VELOCITY_STATE = 480,
			ARTI_SET_LINK_FORCE_STATE = 480,
			ARTI_SET_LINK_TORQUE_STATE = 512,
			ARTI_SET_TENDON_STATE = 512,
			ARTI_SET_SPATIAL_TENDON_ATTACHMENT_STATE = 512,
			ARTI_SET_FIXED_TENDON_JOINT_STATE = 512
		};
	};

	struct PxgArticulationCoreKernelGridDim
	{
		enum
		{
			COMPUTE_UNCONSTRAINED_VELOCITES = 64,
			UPDATE_BODIES = 64,
			UPDATE_KINEMATIC = 1024 // AD: see inline comment in PxgArticulationCore
		};
	};

}

#endif
