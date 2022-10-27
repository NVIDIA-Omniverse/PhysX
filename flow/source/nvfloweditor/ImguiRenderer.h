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

struct NvFlowImguiRenderer;

struct NvFlowImguiRendererInterface
{
	NV_FLOW_REFLECT_INTERFACE();

	NvFlowImguiRenderer*(NV_FLOW_ABI* create)(
		NvFlowContextInterface* contextInterface, 
		NvFlowContext* context, 
		unsigned char* pixels,
		int texWidth,
		int texHeight
	);

	void(NV_FLOW_ABI* destroy)(NvFlowContext* context, NvFlowImguiRenderer* renderer);

	void(NV_FLOW_ABI* render)(NvFlowContext* context, NvFlowImguiRenderer* renderer, ImDrawData* drawData, NvFlowUint width, NvFlowUint height, NvFlowTextureTransient* colorIn, NvFlowTextureTransient* colorOut);
};

#define NV_FLOW_REFLECT_TYPE NvFlowImguiRendererInterface
NV_FLOW_REFLECT_BEGIN()
NV_FLOW_REFLECT_FUNCTION_POINTER(create, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroy, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(render, 0, 0)
NV_FLOW_REFLECT_END(0)
NV_FLOW_REFLECT_INTERFACE_IMPL()
#undef NV_FLOW_REFLECT_TYPE

NvFlowImguiRendererInterface* NvFlowGetImguiRendererInterface();