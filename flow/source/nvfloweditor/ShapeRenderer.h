/*
 * Copyright (c) 2014-2017, NVIDIA CORPORATION.  All rights reserved.
 *
 * NVIDIA CORPORATION and its licensors retain all intellectual property
 * and proprietary rights in and to this software, related documentation
 * and any modifications thereto.  Any use, reproduction, disclosure or
 * distribution of this software and related documentation without an express
 * license agreement from NVIDIA CORPORATION is strictly prohibited.
 */

#pragma once

struct NvFlowShapeRenderer;

struct NvFlowShapeRendererParams
{
	NvFlowUint numSpheres;
	NvFlowFloat4* spherePositionRadius;
};

struct NvFlowShapeRendererInterface
{
	NV_FLOW_REFLECT_INTERFACE();

	NvFlowShapeRenderer*(NV_FLOW_ABI* create)(NvFlowContextInterface* contextInterface,	NvFlowContext* context);

	void(NV_FLOW_ABI* destroy)(NvFlowContext* context, NvFlowShapeRenderer* renderer);

	void(NV_FLOW_ABI* render)(
		NvFlowContext* context, 
		NvFlowShapeRenderer* renderer, const NvFlowShapeRendererParams* params,
		const NvFlowFloat4x4* view,
		const NvFlowFloat4x4* projection,
		NvFlowUint textureWidth,
		NvFlowUint textureHeight, 
		NvFlowTextureTransient* depthOut, 
		NvFlowTextureTransient* colorOut
	);
};

#define NV_FLOW_REFLECT_TYPE NvFlowShapeRendererInterface
NV_FLOW_REFLECT_BEGIN()
NV_FLOW_REFLECT_FUNCTION_POINTER(create, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroy, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(render, 0, 0)
NV_FLOW_REFLECT_END(0)
NV_FLOW_REFLECT_INTERFACE_IMPL()
#undef NV_FLOW_REFLECT_TYPE

NvFlowShapeRendererInterface* NvFlowGetShapeRendererInterface();