// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#include "UsdPCH.h"

#ifndef __linux__
#define DISABLE_ONNX 1
#endif

#ifndef DISABLE_ONNX

#include "MLIRPolicy.h"

#include <inttypes.h>
#include <stdlib.h>

#if defined(__linux__)
#include <dlfcn.h>
#elif defined(_WIN32)
#include <windows.h>
#else
#endif

namespace {
    std::string resolveSharedObject(std::string path)
    {
#if defined(__x86_64__) || defined(_M_X64)
        path += "-x86_64";  // PX_X64
#elif defined(__i386__) || defined(_M_IX86) || defined (__EMSCRIPTEN__)
        path += "-i386";  // PX_X86
#elif defined(__arm64__) || defined(__aarch64__) || defined(_M_ARM64)
        path += "-aarch64";  // PX_A64
#else
        assert(false && "Architecture not supported by MLIRPolicy");
#endif
#if defined(__linux__)
        path += "-linux.so";
#elif defined(_WIN64)
        path += "-windows.dll";
#else
        assert(false && "OS not supported by MLIRPolicy");
#endif
        return path;
    }
}

MLIRPolicy::MLIRPolicy(const std::string& basePath, const Shape inputShape): shape(inputShape)
{
    const std::string libPath = resolveSharedObject(basePath);
#if defined(__linux__)
    policySoHandle = dlopen(libPath.c_str(), RTLD_LAZY);
#elif defined(_WIN64)
    policySoHandle = LoadLibraryA(libPath.c_str());
#else
#endif
    if (policySoHandle == nullptr)
    {
        std::cerr << "Error: policy at " << libPath << " could not be loaded.\n";
        dlLoadSuccessful = false;
        return;
    }

    loadFunctions();
    initTensors();
    loaded = true;
}

MLIRPolicy::~MLIRPolicy()
{
    loaded = false;
    if (inputTensorList != nullptr)
        tensorListDestroy(inputTensorList);
    inputTensorList = nullptr;

    if (outputTensorList != nullptr)
        tensorListDestroy(outputTensorList);
    outputTensorList = nullptr;

    if (policySoHandle != nullptr)
#if defined(__linux__)
        dlclose(policySoHandle);
#elif defined(_WIN64)
        FreeLibrary(policySoHandle);
#else
#endif
    policySoHandle = nullptr;
}


void MLIRPolicy::initTensors()
{
    // initialize tensors
    inputArray = new float[shape.dim0*shape.dim1];
    int64_t shapeArr[] = {shape.dim0, shape.dim1};
    OMTensor* x1 = tensorCreateWithOwnership(inputArray, shapeArr, 2, ONNX_TYPE_FLOAT, true);
    if (x1 == nullptr)
    {
        std::cerr << "Tensor creation failed.\n";
        return;
    }
    inputTensorList = tensorListCreate(&x1, 1);
    if (inputTensorList == nullptr)
        std::cerr << "TensorList creation failed.\n";
}

void MLIRPolicy::loadFunctions()
{
    tensorListCreate = loadSymbol<tensorListCreate_t>("omTensorListCreate");
    tensorListDestroy = loadSymbol<tensorListDestroy_t>("omTensorListDestroy");
    tensorListGetOmtByIndex = loadSymbol<tensorListGetOmtByIndex_t>("omTensorListGetOmtByIndex");

    tensorCreateWithOwnership = loadSymbol<tensorCreateWithOwnership_t>("omTensorCreateWithOwnership");
    tensorGetDataPtr = loadSymbol<tensorGetDataPtr_t>("omTensorGetDataPtr");

    run_main_graph = loadSymbol<run_main_graph_t>("run_main_graph");
}

int MLIRPolicy::forward()
{
    if (!loaded)
    {
        std::cerr << "Errors during policy load. Skipping inference." << "\n";
        return 1;
    }

    // Destroy last result if present
    if (outputTensorList != nullptr)
        tensorListDestroy(outputTensorList);

    // Call the compiled onnx model function.
    outputTensorList = run_main_graph(inputTensorList);

    if (outputTensorList == nullptr) {
        // May inspect errno to get info about the error.
        std::cerr << "Errors during inference." << "\n";
        return 1;
    }

    return 0;
}

template <typename F>
F MLIRPolicy::loadSymbol(const char* name)
{
#if defined(__linux__)
    void* sym = dlsym(policySoHandle, name);
#elif defined(_WIN64)
    FARPROC sym = GetProcAddress(policySoHandle, name);
#else
#endif
    if (sym != nullptr)
        return (F) sym;
    dlLoadSuccessful = false;
    std::cerr << "Error loading symbol " << name << " in policy dynamic library " << policySoHandle << ".\n";
    return nullptr;
}

float* MLIRPolicy::inputData() { return inputArray; }

int MLIRPolicy::inputShape() { return shape.dim1; }

const float* MLIRPolicy::outputData()
{
    if (outputTensorList == nullptr)
        return nullptr;

    return static_cast<float*>(tensorGetDataPtr(tensorListGetOmtByIndex(outputTensorList, 0)));
}

int MLIRPolicy::outputShape() { return 0; }

#endif
