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

#ifndef PXG_BODYSIM_H
#define PXG_BODYSIM_H

#include "AlignedTransform.h"

namespace physx
{

struct PxgBodySim
{
	float4		linearVelocityXYZ_inverseMassW;													//16	16
	float4		angularVelocityXYZ_maxPenBiasW;													//32	16

	float4		maxLinearVelocitySqX_maxAngularVelocitySqY_linearDampingZ_angularDampingW;		//48	16
	float4		inverseInertiaXYZ_contactReportThresholdW;										//64	16

	float4		sleepLinVelAccXYZ_freezeCountW;													//80	16
	float4		sleepAngVelAccXYZ_accelScaleW;													//96	16
	float4		freezeThresholdX_wakeCounterY_sleepThresholdZ_bodySimIndex;						//112	16

	PxAlignedTransform body2World;																//144	32		
	PxAlignedTransform body2Actor_maxImpulseW;													//176	32

	PxU32		articulationRemapId;															//180	4
	PxU32		internalFlags;																	//184	4
	PxU16		lockFlags;																		//186	2
	PxU16		disableGravity;																	//188	2
	PxReal		offsetSlop;																		//192	4	

	float4		externalLinearAcceleration;														//208	16
	float4		externalAngularAcceleration;													//224	16
};

struct PxgBodySimVelocities
{
	float4	linearVelocity;
	float4	angularVelocity;
};

struct PxgBodySimVelocityUpdate
{
	float4 linearVelocityXYZ_bodySimIndexW;
	float4 angularVelocityXYZ_maxPenBiasW;
	float4 externalLinearAccelerationXYZ;
	float4 externalAngularAccelerationXYZ;
};

}//physx

#endif