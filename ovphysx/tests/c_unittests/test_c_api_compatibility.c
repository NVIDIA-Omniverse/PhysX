// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// PARTIALLY DEPRECATED (tensor-binding-deprecation): Test 7 (TensorBinding API types) retires with the binding. The other C-API ABI checks stay.

/*
 * @implements REQ-CAPI-OVD-NAMES-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-OMNIPVD-LATE-001
 * @covers AC-1
 * @implements REQ-CAPI-BINDING-DEVICE-001
 * @covers AC-1
 */

/**
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-1 AC-2
 */

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
#include <ovphysx/ovphysx_config.h>
#include <ovphysx/ovphysx_geometry_query.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

_Static_assert(sizeof(ovphysx_instance_handle_t) == sizeof(uint64_t), "instance handle ABI size");
_Static_assert(_Alignof(ovphysx_instance_handle_t) == _Alignof(uint64_t), "instance handle ABI alignment");
_Static_assert(OVPHYSX_INVALID_HANDLE == 0, "instance invalid handle ABI value");

#define ASSERT_TASK_API(function, ...) \
    _Static_assert(_Generic(&(function), ovphysx_enqueue_result_t (*)(__VA_ARGS__): 1, default: 0), \
                   #function " returns a task receipt")
ASSERT_TASK_API(ovphysx_query, ovphysx_instance_handle_t, ovphysx_query_target_t, ovphysx_object_scope_t,
                ovphysx_query_handle_t*, ovstage_query_result_t*);
ASSERT_TASK_API(ovphysx_query_selection, ovphysx_instance_handle_t, ovphysx_query_target_t,
                const ovphysx_query_selection_t*, ovphysx_query_handle_t*, ovstage_query_result_t*);
ASSERT_TASK_API(ovphysx_query_contacts, ovphysx_instance_handle_t, const ovphysx_contact_sensor_selection_t*, uint32_t,
                uint32_t, ovphysx_query_handle_t*, ovstage_query_result_t*);
ASSERT_TASK_API(ovphysx_release_query, ovphysx_instance_handle_t, ovphysx_query_handle_t);
ASSERT_TASK_API(ovphysx_read, ovphysx_instance_handle_t, ovphysx_query_handle_t, const ovx_string_or_token_t*, size_t,
                ovphysx_read_handle_t*, ovphysx_read_result_t*);
ASSERT_TASK_API(ovphysx_fetch_read_next, ovphysx_instance_handle_t, ovphysx_read_handle_t, const ovstage_read_group_t**);
ASSERT_TASK_API(ovphysx_release_group, ovphysx_instance_handle_t, ovphysx_read_handle_t, ovstage_read_group_id_t);
ASSERT_TASK_API(ovphysx_release_read, ovphysx_instance_handle_t, ovphysx_read_handle_t);
ASSERT_TASK_API(ovphysx_write, ovphysx_instance_handle_t, ovphysx_query_handle_t, const ovx_string_or_token_t*,
                size_t, ovphysx_write_handle_t*, ovphysx_write_result_t*);
ASSERT_TASK_API(ovphysx_fetch_write_next, ovphysx_instance_handle_t, ovphysx_write_handle_t,
                ovphysx_fetched_write_group_t*);
ASSERT_TASK_API(ovphysx_commit_group, ovphysx_instance_handle_t, ovphysx_write_handle_t, const ovstage_map_group_t*,
                ovstage_cuda_sync_t);
ASSERT_TASK_API(ovphysx_release_write, ovphysx_instance_handle_t, ovphysx_write_handle_t);
ASSERT_TASK_API(
    ovphysx_update_articulations_kinematic, ovphysx_instance_handle_t, ovphysx_query_handle_t, uint32_t);
ASSERT_TASK_API(ovphysx_evaluate_sdf_shapes, ovphysx_instance_handle_t, ovphysx_query_handle_t, const DLTensor*,
                DLTensor*, ovstage_cuda_sync_t);
#undef ASSERT_TASK_API

_Static_assert(OVPHYSX_LOG_DEFAULT == 0, "DEFAULT ABI value");
_Static_assert(OVPHYSX_LOG_VERBOSE == 1, "VERBOSE ABI value");
_Static_assert(OVPHYSX_LOG_INFO == 2, "INFO ABI value");
_Static_assert(OVPHYSX_LOG_WARNING == 3, "WARNING ABI value");
_Static_assert(OVPHYSX_LOG_ERROR == 4, "ERROR ABI value");
_Static_assert(OVPHYSX_LOG_NONE == 5, "NONE ABI value");
_Static_assert(sizeof(ovphysx_timeout_t) == sizeof(uint64_t), "timeout ABI size");
_Static_assert(_Alignof(ovphysx_timeout_t) == _Alignof(uint64_t), "timeout ABI alignment");
_Static_assert(OVPHYSX_TIMEOUT_POLL == 0, "poll timeout value");
_Static_assert(OVPHYSX_TIMEOUT_INFINITE == UINT64_MAX, "infinite timeout value");
_Static_assert(OVPHYSX_API_INVALID_STATE == 9, "invalid-state ABI value");
_Static_assert(OVPHYSX_CONFIG_BOOL_COUNT == 5, "retired OVD bool slots remain reserved");
_Static_assert(OVPHYSX_CONFIG_OVSTAGE_READ_POOL_MAX_MB == 4, "ovstage read-pool int32 config ABI value");
_Static_assert(OVPHYSX_CONFIG_PROFILER_BACKEND == 5, "profiler-backend int32 config ABI value");
_Static_assert(OVPHYSX_CONFIG_PROFILER_OUTPUT_PATH == 4, "profiler-output-path string config ABI value");
_Static_assert(OVPHYSX_PROFILER_BACKEND_NONE == 0, "profiler backend NONE ABI value");
_Static_assert(OVPHYSX_PROFILER_BACKEND_NVTX == 1, "profiler backend NVTX ABI value");
_Static_assert(OVPHYSX_PROFILER_BACKEND_TRACE == 2, "profiler backend TRACE ABI value");
_Static_assert(OVPHYSX_OMNIPVD_TRANSPORT_FILE == 0, "OmniPVD FILE transport ABI value");
_Static_assert(OVPHYSX_OMNIPVD_TRANSPORT_TCP == 1, "OmniPVD TCP transport ABI value");
_Static_assert(
    _Generic(
        &ovphysx_ovd_start_recording,
        ovphysx_result_t (*)(
            ovphysx_instance_handle_t,
            const ovphysx_omnipvd_destination_t*): 1,
        default: 0),
    "start_recording C ABI shape");
_Static_assert(
    _Generic(
        &ovphysx_ovd_stop_recording,
        ovphysx_result_t (*)(ovphysx_instance_handle_t): 1,
        default: 0),
    "stop_recording C ABI shape");
_Static_assert(
    _Generic(
        &ovphysx_ovd_is_recording,
        ovphysx_result_t (*)(ovphysx_instance_handle_t, bool*): 1,
        default: 0),
    "is_recording C ABI shape");
_Static_assert(
    _Generic(
        &ovphysx_wait_task,
        ovphysx_result_t (*)(
            ovphysx_instance_handle_t,
            ovphysx_enqueue_result_t,
            ovphysx_timeout_t,
            ovphysx_wait_task_result_t*): 1,
        default: 0),
    "wait_task C ABI shape");
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
_Static_assert(
    _Generic(
        &ovphysx_get_tensor_binding_native_device,
        ovphysx_result_t (*)(ovphysx_instance_handle_t, ovphysx_tensor_binding_handle_t, DLDevice*): 1,
        default: 0),
    "get_tensor_binding_native_device C ABI shape");
#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
_Static_assert(
    _Generic(
        &ovphysx_set_log_callback,
        ovphysx_result_t (*)(
            ovphysx_log_level_t,
            const ovphysx_string_t*,
            ovphysx_log_callback_t,
            void*): 1,
        default: 0),
    "set_log_callback C ABI shape");
_Static_assert(
    _Generic(
        &ovphysx_flush_log,
        ovphysx_result_t (*)(ovphysx_timeout_t): 1,
        default: 0),
    "flush_log C ABI shape");

_Static_assert(
    _Generic(&ovphysx_wait_all,
        ovphysx_result_t (*)(ovphysx_instance_handle_t, ovphysx_timeout_t, ovphysx_wait_task_result_t*): 1,
        default: 0),
    "wait_all C ABI shape");
_Static_assert(
    _Generic(&ovphysx_get_last_task_error,
        ovphysx_string_t (*)(ovphysx_instance_handle_t, ovphysx_enqueue_result_t): 1,
        default: 0),
    "get_last_task_error C ABI shape");
_Static_assert(
    _Generic(&ovphysx_destroy_wait_task_result,
        void (*)(ovphysx_wait_task_result_t*): 1,
        default: 0),
    "destroy_wait_task_result C ABI shape");

static void test_log_callback(
    ovphysx_log_level_t severity,
    ovphysx_string_t message,
    ovphysx_string_t channel,
    double timestamp,
    void* user_data)
{
    (void)severity;
    (void)message;
    (void)channel;
    (void)timestamp;
    (void)user_data;
}

static void test_logging_api_shape(void)
{
    ovphysx_log_callback_t callback = test_log_callback;
    ovphysx_string_t filter = OVPHYSX_LITERAL("omni.physx=warning");
    (void)callback;
    (void)filter;
}
static void test_omnipvd_destination_type(void)
{
    ovphysx_omnipvd_destination_t destination = { 0 };
    destination.transport = OVPHYSX_OMNIPVD_TRANSPORT_FILE;
    destination.file_path = OVPHYSX_LITERAL("capture.ovd");
    destination.tcp_address = OVPHYSX_LITERAL("");
    destination.tcp_port = 0;
    destination.tcp_timeout_ms = 0;
    (void)destination;
}

/* Deprecated exports must remain callable from C as well as C++. */
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4996)
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
_Static_assert(sizeof(ovphysx_handle_t) == sizeof(ovphysx_instance_handle_t),
               "legacy instance handle C ABI size");
_Static_assert(_Generic(&ovphysx_start_recording,
                       ovphysx_result_t (*)(ovphysx_handle_t, const ovphysx_omnipvd_destination_t*): 1,
                       default: 0),
               "legacy start_recording C ABI shape");
_Static_assert(_Generic(&ovphysx_stop_recording,
                       ovphysx_result_t (*)(ovphysx_handle_t): 1,
                       default: 0),
               "legacy stop_recording C ABI shape");
_Static_assert(_Generic(&ovphysx_is_recording,
                       ovphysx_result_t (*)(ovphysx_handle_t, bool*): 1,
                       default: 0),
               "legacy is_recording C ABI shape");
#if defined(_MSC_VER)
#pragma warning(pop)
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

_Static_assert(_Generic(&ovphysx_set_global_config,
                       ovphysx_result_t (*)(ovphysx_global_config_key_t, ovphysx_config_value_t): 1,
                       default: 0),
               "set_global_config C ABI shape");

static void test_omnipvd_recording_capability_config(void)
{
    ovphysx_config_value_t value = ovphysx_config_value_bool(true);
    ovphysx_global_config_key_t key = OVPHYSX_GLOBAL_CONFIG_OVD_RECORDING_CAPABLE;
    (void)value;
    (void)key;
}

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
    ovphysx_instance_handle_t handle = 0;
    (void)handle;

    /* If the macro used nullptr, this file would fail to compile in C mode */
}

/* Test 4: Enqueue result types */
static void test_enqueue_result(void)
{
    ovphysx_enqueue_result_t result;
    result.status = OVPHYSX_API_SUCCESS;
    result.op_index = 0;
    result.instance = 0;

    (void)result;
}

/* Test 5: Status codes and handles */
static void test_status_and_handles(void)
{
    ovphysx_api_status_t status = OVPHYSX_API_SUCCESS;
    ovphysx_instance_handle_t handle = 0;
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
    test_logging_api_shape();
    test_omnipvd_destination_type();
    test_omnipvd_recording_capability_config();
    
    printf("SUCCESS: All C API compatibility tests passed!\n");
    printf("  - Headers compile with C compiler (not C++)\n");
    printf("  - NULL used instead of nullptr\n");
    printf("  - Typedefs allow clean syntax\n");
    printf("  - free() used in C mode (not delete[])\n");
    
    return 0;
}
