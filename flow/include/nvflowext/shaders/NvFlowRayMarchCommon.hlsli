/*
 * Copyright (c) 2014-2020, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#ifndef NV_FLOW_RAY_MARCH_COMMON_HLSLI
#define NV_FLOW_RAY_MARCH_COMMON_HLSLI

#include "NvFlowRayMarchParams.h"

#include "NvFlowShader.hlsli"

// ray origin is implied zero
bool NvFlowIntersectBox(float3 rayDir, float3 rayDirInv, float3 boxMin, float3 boxMax, out float tnear, out float tfar)
{
	// compute intersection of ray with all six bbox planes
	float3 tbot = boxMin * rayDirInv;
	float3 ttop = boxMax * rayDirInv;

	// re-order intersections to find smallest and largest on each axis
	float3 tmin = min(ttop, tbot);
	float3 tmax = max(ttop, tbot);

	// find the largest tmin and the smallest tmax
	tnear = max(max(tmin.x, tmin.y), max(tmin.x, tmin.z));
	tfar = min(min(tmax.x, tmax.y), min(tmax.x, tmax.z));

	return tfar > tnear;
}

int3 NvFlowRayMarchComputeFinalLocation(float3 rayDir, int4 location, int4 locationMin, int4 locationMax)
{
	return int3(
		rayDir.x > 0.f ? max(location.x, locationMax.x) : min(location.x, locationMin.x - 1),
		rayDir.y > 0.f ? max(location.y, locationMax.y) : min(location.y, locationMin.y - 1),
		rayDir.z > 0.f ? max(location.z, locationMax.z) : min(location.z, locationMin.z - 1)
		);
}

void NvFlowRayMarchAdvanceRay(
	float3 blockSizeWorld,
	float3 rayDir,
	float3 rayDirInv,
	float3 rayOrigin,
	inout int4 location,
	inout float hitT
)
{
	float hitTx = (float(location.x + (rayDir.x > 0.f ? +1 : 0)) * blockSizeWorld.x - rayOrigin.x) * rayDirInv.x;
	float hitTy = (float(location.y + (rayDir.y > 0.f ? +1 : 0)) * blockSizeWorld.y - rayOrigin.y) * rayDirInv.y;
	float hitTz = (float(location.z + (rayDir.z > 0.f ? +1 : 0)) * blockSizeWorld.z - rayOrigin.z) * rayDirInv.z;

	if (rayDir.x != 0.f && (hitTx <= hitTy || rayDir.y == 0.f) && (hitTx <= hitTz || rayDir.z == 0.f))
	{
		hitT = hitTx;
		location.x += rayDir.x > 0.f ? +1 : -1;
	}
	else if (rayDir.y != 0.f && (hitTy <= hitTx || rayDir.x == 0.f) && (hitTy <= hitTz || rayDir.z == 0.f))
	{
		hitT = hitTy;
		location.y += rayDir.y > 0.f ? +1 : -1;
	}
	else
	{
		hitT = hitTz;
		location.z += rayDir.z > 0.f ? +1 : -1;
	}
}

#endif