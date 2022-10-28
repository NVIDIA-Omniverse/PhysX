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


#ifndef PX_ARTICULATION_TENDON_DATA_H
#define PX_ARTICULATION_TENDON_DATA_H

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxVec3.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief PxGpuSpatialTendonData

This is the data structure used by the direct API for spatial tendon data updates
*/
PX_ALIGN_PREFIX(16)
class PxGpuSpatialTendonData
{
public:
	PxReal		stiffness;
	PxReal		damping;
	PxReal		limitStiffness;
	PxReal		offset;
}
PX_ALIGN_SUFFIX(16);

/**
\brief PxGpuFixedTendonData

This is the data stucture use by the direct API for fixed tendon data update
*/
PX_ALIGN_PREFIX(16)
class PxGpuFixedTendonData : public PxGpuSpatialTendonData
{
public:
	PxReal		lowLimit;
	PxReal		highLimit;
	PxReal		restLength;
	PxReal		padding;
}
PX_ALIGN_SUFFIX(16);


PX_ALIGN_PREFIX(16)
class PxGpuTendonJointCoefficientData
{
public:
	PxReal		coefficient;
	PxReal		recipCoefficient;
	PxU32		axis;
	PxU32		pad;
}
PX_ALIGN_SUFFIX(16);

PX_ALIGN_PREFIX(16)
class PxGpuTendonAttachmentData
{
public:
	PxVec3	relativeOffset;		//relative offset to the link			
	PxReal	restLength;			//equilibrium length for spring			

	PxReal	coefficient;		//user defined scale value				
	PxReal	lowLimit;			//user defined low limit				
	PxReal	highLimit;			//user defined high limit				
	PxReal	padding;											
}
PX_ALIGN_SUFFIX(16);

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
