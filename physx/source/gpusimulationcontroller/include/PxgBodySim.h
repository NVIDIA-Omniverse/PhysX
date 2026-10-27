// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

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