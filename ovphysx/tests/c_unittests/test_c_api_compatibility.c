// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

/*
 * C API Compatibility Test
 * 
 * This file MUST compile with a pure C compiler (gcc -std=c11).
 * It validates that the SDK headers are truly C-compatible.
 * 
 * If this test fails to compile, it means:
 * - C++ keywords (nullptr, delete, etc.) leaked into C code paths
 * - Typedefs are missing for struct types
 * - C++ headers (<cstdlib>) were used instead of C headers (<stdlib.h>)
 */

#include <ovphysx/ovphysx.h>
#include "AsyncEventManager/AsyncEventManager.h"
#include <stdio.h>
#include <stdlib.h>

/* Test 1: Struct typedefs allow usage without "struct" keyword */
static void test_typedefs(void)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
    (void)args;

    /* Log level constants and types should compile in C */
    ovphysx_log_level_t level = OVPHYSX_LOG_INFO;
    (void)level;

    /* This would fail in C without typedef:
     * error: unknown type name 'ovphysx_create_args'; use 'struct' keyword
     */
}

/* Test 2: NULL (not nullptr) is used in C code paths */
static void test_null_usage(void)
{
    ovphysx_result_t result;
    result.status = OVPHYSX_API_SUCCESS;
    (void)result;
}

/* Test 3: Handles and NULL usage */
static void test_macros_with_null(void)
{
    ovphysx_handle_t handle = 0;
    (void)handle;

    /* If the macro used nullptr, this file would fail to compile in C mode */
}

/* Test 4: Enqueue result types */
static void test_enqueue_result(void)
{
    ovphysx_enqueue_result_t result;
    result.status = OVPHYSX_API_SUCCESS;
    result.op_index = 0;

    (void)result;
}

/* Test 5: Status codes and handles */
static void test_status_and_handles(void)
{
    ovphysx_api_status_t status = OVPHYSX_API_SUCCESS;
    ovphysx_handle_t handle = 0;
    ovphysx_op_index_t op_index = 0;
    
    (void)status;
    (void)handle;
    (void)op_index;
}

/* Test 6: Binding and buffer types */
static void test_binding_types(void)
{
    ovphysx_attribute_binding_handle_t binding_handle = 0;
    ovphysx_write_map_handle_t write_map_handle = 0;
    ovphysx_read_map_handle_t read_map_handle = 0;
    
    (void)binding_handle;
    (void)write_map_handle;
    (void)read_map_handle;
}

/* Test 7: TensorBinding API types */
static void test_tensor_binding_types(void)
{
    /* ovphysx_tensor_binding_desc_t - input descriptor for creating bindings */
    ovphysx_tensor_binding_desc_t desc;
    desc.pattern.ptr = "/World/robot*";
    desc.pattern.length = 13;
    desc.prim_paths = NULL;
    desc.prim_paths_count = 0;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    
    /* ovphysx_tensor_spec_t - output spec from get_tensor_binding_spec */
    ovphysx_tensor_spec_t spec;
    spec.dtype.code = kDLFloat;
    spec.dtype.bits = 32;
    spec.dtype.lanes = 1;
    spec.ndim = 2;
    spec.shape[0] = 10;
    spec.shape[1] = 7;
    spec.shape[2] = 0;
    spec.shape[3] = 0;
    
    /* ovphysx_tensor_binding_handle_t - handle for created bindings */
    ovphysx_tensor_binding_handle_t tensor_binding = 0;
    
    (void)desc;
    (void)spec;
    (void)tensor_binding;
}

/* Main test entry point */
int main(void)
{
    printf("Running C API compatibility tests...\n");
    
    test_typedefs();
    test_null_usage();
    test_macros_with_null();
    test_enqueue_result();
    test_status_and_handles();
    test_binding_types();
    test_tensor_binding_types();
    
    printf("SUCCESS: All C API compatibility tests passed!\n");
    printf("  - Headers compile with C compiler (not C++)\n");
    printf("  - NULL used instead of nullptr\n");
    printf("  - Typedefs allow clean syntax\n");
    printf("  - free() used in C mode (not delete[])\n");
    
    return 0;
}
