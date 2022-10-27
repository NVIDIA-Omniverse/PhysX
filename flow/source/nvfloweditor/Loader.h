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
