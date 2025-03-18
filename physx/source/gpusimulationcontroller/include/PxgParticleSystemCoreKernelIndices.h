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

#ifndef PXG_PARTICLE_SYSTEM_CORE_KERNEL_INDICES_H
#define PXG_PARTICLE_SYSTEM_CORE_KERNEL_INDICES_H

namespace physx
{

	struct PxgParticleSystemKernelBlockDim
	{
		enum
		{
			UPDATEBOUND = 1024, //can't change this. updateBound kernel is relied on numOfWarpPerBlock = 32
			UPDATEGRID = 1024,
			BOUNDCELLUPDATE = 512,
			PS_COLLISION = 256, //128,
			PS_MESH_COLLISION = 512,
			PS_HEIGHTFIELD_COLLISION = 64,
			ACCUMULATE_DELTA = 512,
			PS_SOLVE = 256,
			PS_CELL_RECOMPUTE = 256,
			PS_INFLATABLE = 256,
			PS_SOLVE_SHAPE = 64,
			SCAN = 512
		};
	};

	struct PxgParticleSystemKernelGridDim
	{
		enum
		{
			BOUNDCELLUPDATE = 32,
			PS_COLLISION = 1024,
			PS_MESH_COLLISION = 16384,
			PS_HEIGHTFIELD_COLLISION = 4096,
			ACCUMULATE_DELTA = 32,
			PS_CELL_RECOMPUTE = 32,
		};
	};
}

#endif
