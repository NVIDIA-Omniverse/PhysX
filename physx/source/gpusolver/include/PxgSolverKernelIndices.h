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

#ifndef PXG_SOLVER_KERNEL_INDICES_H
#define PXG_SOLVER_KERNEL_INDICES_H

namespace physx
{

#define PXG_USE_SHARED_MEMORY_PRE_PREP 0

struct PxgKernelBlockDim
{
	enum
	{
		//Constraint partition
		CONSTRAINT_PRE_PARTITION						= 1024,
		CONSTRAINT_PARTITION							= 1024,

		PRE_INTEGRATION									= 128,

		//Constraint pre-preparation
		CONSTRAINT_PREPREP_BLOCK						= 128,

		//Constraint preparation
		CONSTRAINT_PREPARE_BLOCK_PARALLEL				= 64,

		ARTI_CONSTRAINT_PREPARE							= 64,
		
		//Multi-block solver code
		ZERO_BODIES										= 256,
		SOLVE_BLOCK_PARTITION							= 64,
		CONCLUDE_BLOCKS									= 256,
		WRITEBACK_BLOCKS								= 256,
		WRITE_BACK_BODIES								= 256,
		COMPUTE_BODIES_AVERAGE_VELOCITY					= 256,

		//threshold stream
		INITIALIZE_INPUT_AND_RANKS						= 256,
		RADIXSORT										= 256,
		REORGANIZE_THRESHOLDSTREAM						= 256,
		COMPUTE_ACCUMULATED_THRESHOLDSTREAM				= 256,
		OUTPUT_ACCUMULATED_THRESHOLDSTREAM				= 256,
		WRITEOUT_ACCUMULATEDFORCEPEROBJECT				= 256,
		COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE	= 256,
		OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE	= 256,
		SET_THRESHOLDELEMENT_MASK						= 256,
		COMPUTE_THRESHOLDELEMENT_MASK_INDICES			= 256,
		OUTPUT_THRESHOLDELEMENT_MASK_INDICES			= 256,
		CREATE_FORCECHANGE_THRESHOLDELEMENTS			= 256,

		//Integration
		INTEGRATE_CORE_PARALLEL							= 128,
		CLEAR_FRICTION_PATCH_COUNTS						= 256,
		DMA_CHANGED_ELEMS								= 512,
		COMPUTE_STATIC_CONTACT_CONSTRAINT_COUNT			= 512
	};
};

struct PxgKernelGridDim
{
	enum
	{
		//Constraint partition
		CONSTRAINT_PRE_PARTITION						= 1,
		CONSTRAINT_PARTITION							= 1,

		PRE_INTEGRATION									= 64,
		//Constraint preparation
		CONSTRAINT_PREPREP_BLOCK						= 128,

		CONSTRAINT_PREPARE_BLOCK_PARALLEL				= 256,	
		
		//Multi-block solver code
		ZERO_BODIES								= 64,
		SOLVE_BLOCK_PARTITION					= 64,
		CONCLUDE_BLOCKS							= 64,
		WRITEBACK_BLOCKS						= 64,
		WRITE_BACK_BODIES						= 64,
		COMPUTE_BODIES_AVERAGE_VELOCITY			= 128,

		//threshold stream
		INITIALIZE_INPUT_AND_RANKS						= 64,
		RADIXSORT										= 32, //this must be 32 to match the BLOCK_SIZE for the radix sort kernel
		REORGANIZE_THRESHOLDSTREAM						= 64,
		COMPUTE_ACCUMULATED_THRESHOLDSTREAM				= 32,//this must be 32 to match the BLOCK_SIZE for the compute kernel
		OUTPUT_ACCUMULATED_THRESHOLDSTREAM				= 32,//this must be 32 to match the BLOCK_SIZE for the output kernel
		WRITEOUT_ACCUMULATEDFORCEPEROBJECT				= 64,
		COMPUTE_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE	= 32,
		OUTPUT_EXCEEDEDFORCE_THRESHOLDELEMENT_INDICE	= 32,
		SET_THRESHOLDELEMENT_MASK						= 64,
		COMPUTE_THRESHOLDELEMENT_MASK_INDICES			= 32,
		OUTPUT_THRESHOLDELEMENT_MASK_INDICES			= 32,
		CREATE_FORCECHANGE_THRESHOLDELEMENTS			= 64,
		//Integration
		INTEGRATE_CORE_PARALLEL							= 64,
		CLEAR_FRICTION_PATCH_COUNTS						= 64,
		DMA_CHANGED_ELEMS								= 64,
	};
};

}  

#endif
