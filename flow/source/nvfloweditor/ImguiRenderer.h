// SPDX-FileCopyrightText: Copyright (c) 2014-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
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

#pragma once

struct NvFlowImguiRenderer;
struct NvFlowImguiTexture;

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

    NvFlowImguiTexture*(NV_FLOW_ABI* createTexture)(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        unsigned char* pixels,
        int texWidth,
        int texHeight
    );

    void(NV_FLOW_ABI* updateTexture)(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        NvFlowImguiTexture* texture,
        unsigned char* pixels,
        int texWidth,
        int texHeight
    );

    void(NV_FLOW_ABI* destroyTexture)(
        NvFlowContext* context,
        NvFlowImguiRenderer* renderer,
        NvFlowImguiTexture* texture
    );
};

#define NV_FLOW_REFLECT_TYPE NvFlowImguiRendererInterface
NV_FLOW_REFLECT_BEGIN()
NV_FLOW_REFLECT_FUNCTION_POINTER(create, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroy, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(render, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(createTexture, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(updateTexture, 0, 0)
NV_FLOW_REFLECT_FUNCTION_POINTER(destroyTexture, 0, 0)
NV_FLOW_REFLECT_END(0)
NV_FLOW_REFLECT_INTERFACE_IMPL()
#undef NV_FLOW_REFLECT_TYPE

NvFlowImguiRendererInterface* NvFlowGetImguiRendererInterface();
