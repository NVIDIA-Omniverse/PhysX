// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: LicenseRef-NvidiaProprietary
//

#ifndef MLIR_POLICY_H
#define MLIR_POLICY_H

#include <inttypes.h>

#ifndef __linux__
#define DISABLE_ONNX 1
#endif

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

#ifndef DISABLE_ONNX
#include <OnnxMlirRuntime.h>
#endif

// TODO:
// - support batching
// - don't allocate new memory

struct Shape { int64_t dim0, dim1; };

class MLIRPolicy
{
public:
    MLIRPolicy(const MLIRPolicy& other) = delete;
    MLIRPolicy(MLIRPolicy&& other) = delete;
    MLIRPolicy& operator=(const MLIRPolicy& other) = delete;
    MLIRPolicy& operator=(MLIRPolicy&& other) = delete;

    operator bool(){ return loaded; }

private:
    bool loaded = false;

    // tensor shape & storage
    Shape shape{0, 0};
    float* inputArray = nullptr;  // single, linear input tensor, owned by inputTensorList


#ifdef DISABLE_ONNX
public:
    MLIRPolicy(const std::string& path, Shape inputShape) {
        CARB_LOG_ERROR("This build does not support MLIR policies (DISABLE_ONNX).");
    }
    ~MLIRPolicy() {}

    /**
     * Pointer to input data
     *
     * @return pointer to beginning of input data
     */
    float* inputData() { return nullptr; }

    /**
     * Input shape
     *
     * @return Shape
     */

    int inputShape() { return 0; }

    const float* outputData() { return nullptr; }
    int outputShape() {return 0; }

    int forward() { return 0; }

#else
public:
    // Instantiate MLIR policy from shared object at `path`
    MLIRPolicy(const std::string& path, Shape inputShape);
    ~MLIRPolicy();

    /**
     * Pointer to input data
     *
     * @return pointer to beginning of input data
     */
    float* inputData();

    /**
     * Input shape
     *
     * @return Shape
     */

    int inputShape();

    const float* outputData();
    int outputShape();

    int forward();



    void clear();


    using tensorListCreate_t = OMTensorList* (*)(OMTensor**, int64_t);
    using tensorListDestroy_t = void (*)(OMTensorList *list);
    using tensorListGetOmtByIndex_t = OMTensor* (*)(const OMTensorList *list, int64_t index);

    using tensorCreateWithOwnership_t = OMTensor* (*)(void*, const int64_t*, int64_t, OM_DATA_TYPE, int64_t);
    using tensorGetDataPtr_t = void* (*)(const OMTensor *tensor);

    using run_main_graph_t = OMTensorList* (*)(OMTensorList *);

#if defined(__linux__)
    using SOType = void*;
#elif defined(_WIN32) || defined(_WIN64) || defined(WIN32) || defined(WIN64)
    using SOType = HMODULE;
#endif

    tensorListCreate_t tensorListCreate;
    tensorListDestroy_t tensorListDestroy;
    tensorListGetOmtByIndex_t tensorListGetOmtByIndex;

    tensorCreateWithOwnership_t tensorCreateWithOwnership;
    tensorGetDataPtr_t tensorGetDataPtr;

    run_main_graph_t run_main_graph;

    const char (*inputSignature)(const char *);

    // load a symbol and emit error message if this fails
    template <typename F>
    F loadSymbol(const char* name);

    // load functions from dll
    void loadFunctions();

    // allocate memory and create tensors
    void initTensors();

    bool dlLoadSuccessful = true;
    SOType policySoHandle; // policy.so

    OMTensorList* inputTensorList = nullptr;
    OMTensorList* outputTensorList = nullptr;

#endif // !DISABLE_ONNX
};

#endif  // MLIR_POLICY_H
