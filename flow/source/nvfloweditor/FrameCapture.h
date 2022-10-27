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

#include "NvFlowContext.h"

struct NvFlowFrameCapture;

struct NvFlowFrameCaptureInterface
{
	NV_FLOW_REFLECT_INTERFACE();

	NvFlowFrameCapture*(NV_FLOW_ABI* create)(NvFlowContextInterface* contextInterface, NvFlowContext* context);

	void(NV_FLOW_ABI* destroy)(NvFlowContext* context, NvFlowFrameCapture* frameCapture);

	void(NV_FLOW_ABI* capture)(NvFlowContext* context, NvFlowFrameCapture* frameCapture, NvFlowUint width, NvFlowUint height, NvFlowTextureTransient* texture);

	void(NV_FLOW_ABI* update)(NvFlowContext* context, NvFlowFrameCapture* frameCapture);
};

#define NV_FLOW_REFLECT_TYPE NvFlowFrameCaptureInterface
NV_FLOW_REFLECT_BEGIN()
NV_FLOW_REFLECT_FUNCTION_POINTER(create, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroy, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(capture, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(update, 0, 0)
NV_FLOW_REFLECT_END(0)
NV_FLOW_REFLECT_INTERFACE_IMPL()
#undef NV_FLOW_REFLECT_TYPE

NvFlowFrameCaptureInterface* NvFlowGetFrameCaptureInterface();