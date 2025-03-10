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

#define GLFW_DLL

#if defined(_WIN32)
#define GLFW_EXPOSE_NATIVE_WIN32
#else
#define GLFW_EXPOSE_NATIVE_X11
#endif
#include <GLFW/glfw3.h>
#include <GLFW/glfw3native.h>

#define NV_FLOW_SWAPCHAIN_DESC 1

#include "NvFlowLoader.h"

#define GLFW_PTR(X) decltype(&X) p_##X = nullptr

#define GLFW_PTR_LOAD(X) ptr->p_##X = (decltype(&X))GlfwLoader_loadFunction(ptr, #X)

struct GlfwLoader
{
    void* module = nullptr;

    GLFW_PTR(glfwInit);
    GLFW_PTR(glfwWindowHint);
    GLFW_PTR(glfwCreateWindow);
    GLFW_PTR(glfwGetPrimaryMonitor);
    GLFW_PTR(glfwGetVideoMode);
    GLFW_PTR(glfwSetWindowUserPointer);
    GLFW_PTR(glfwSetWindowPos);
    GLFW_PTR(glfwSetWindowSizeCallback);
    GLFW_PTR(glfwSetKeyCallback);
    GLFW_PTR(glfwSetCharCallback);
    GLFW_PTR(glfwSetMouseButtonCallback);
    GLFW_PTR(glfwSetCursorPosCallback);
    GLFW_PTR(glfwSetScrollCallback);
#if defined(_WIN32)
    GLFW_PTR(glfwGetWin32Window);
#else
    GLFW_PTR(glfwGetX11Display);
    GLFW_PTR(glfwGetX11Window);
#endif
    GLFW_PTR(glfwDestroyWindow);
    GLFW_PTR(glfwTerminate);
    GLFW_PTR(glfwPollEvents);
    GLFW_PTR(glfwWindowShouldClose);
    GLFW_PTR(glfwGetWindowUserPointer);
    GLFW_PTR(glfwSetWindowMonitor);
    GLFW_PTR(glfwGetMouseButton);
};

inline void* GlfwLoader_loadFunction(GlfwLoader* ptr, const char* name)
{
    return NvFlowGetProcAddress(ptr->module, name);
}

inline void GlfwLoader_init(GlfwLoader* ptr)
{
#if defined(__aarch64__)
    ptr->module = NvFlowLoadLibrary("glfw3.dll", "libglfw_aarch64.so.3.3");
#else
    ptr->module = NvFlowLoadLibrary("glfw3.dll", "libglfw.so.3");
#endif

    GLFW_PTR_LOAD(glfwInit);
    GLFW_PTR_LOAD(glfwWindowHint);
    GLFW_PTR_LOAD(glfwCreateWindow);
    GLFW_PTR_LOAD(glfwGetPrimaryMonitor);
    GLFW_PTR_LOAD(glfwGetVideoMode);
    GLFW_PTR_LOAD(glfwSetWindowUserPointer);
    GLFW_PTR_LOAD(glfwSetWindowPos);
    GLFW_PTR_LOAD(glfwSetWindowSizeCallback);
    GLFW_PTR_LOAD(glfwSetKeyCallback);
    GLFW_PTR_LOAD(glfwSetCharCallback);
    GLFW_PTR_LOAD(glfwSetMouseButtonCallback);
    GLFW_PTR_LOAD(glfwSetCursorPosCallback);
    GLFW_PTR_LOAD(glfwSetScrollCallback);
#if defined(_WIN32)
    GLFW_PTR_LOAD(glfwGetWin32Window);
#else
    GLFW_PTR_LOAD(glfwGetX11Display);
    GLFW_PTR_LOAD(glfwGetX11Window);
#endif
    GLFW_PTR_LOAD(glfwDestroyWindow);
    GLFW_PTR_LOAD(glfwTerminate);
    GLFW_PTR_LOAD(glfwPollEvents);
    GLFW_PTR_LOAD(glfwWindowShouldClose);
    GLFW_PTR_LOAD(glfwGetWindowUserPointer);
    GLFW_PTR_LOAD(glfwSetWindowMonitor);
    GLFW_PTR_LOAD(glfwGetMouseButton);
}

inline void GlfwLoader_destroy(GlfwLoader* ptr)
{
    NvFlowFreeLibrary(ptr->module);
}
