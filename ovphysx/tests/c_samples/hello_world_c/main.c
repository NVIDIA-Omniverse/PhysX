// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// NOTE: This file is included verbatim in documentation.
// When editing, keep tutorial line ranges in sync.

// Compile-time check: fail compilation if C++ compiler is used (for internal testing)
#ifdef __cplusplus
#error "This file should be compiled as C, not C++"
#endif

#include "ovphysx/ovphysx.h"
#include <stdio.h>

int main() {
  // Create PhysX instance with default args
  ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
  ovphysx_handle_t handle = 0;
  
  ovphysx_result_t result = ovphysx_create_instance(&create_args, &handle);
  if (result.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to create PhysX instance\n");
    if (result.error.ptr) {
      ovphysx_destroy_error(result.error);
    }
    return 1;
  }

  // Load USD scene
  ovphysx_string_t path_str = ovphysx_cstr(OVPHYSX_TEST_DATA "/simple_physics_scene.usda");
  ovphysx_string_t prefix_str = {NULL, 0};
  ovphysx_usd_handle_t usd_handle = 0;
  
  ovphysx_enqueue_result_t add_result = ovphysx_add_usd(handle, path_str, prefix_str, &usd_handle);
  if (add_result.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to load USD\n");
    if (add_result.error.ptr) {
      ovphysx_destroy_error(add_result.error);
    }
    ovphysx_destroy_instance(handle);
    return 1;
  }

  // Step the simulation
  ovphysx_enqueue_result_t step_result = ovphysx_step(handle, 0.016f, 0.0f);
  if (step_result.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to step simulation\n");
    if (step_result.error.ptr) {
      ovphysx_destroy_error(step_result.error);
    }
    ovphysx_destroy_instance(handle);
    return 1;
  }

  // Wait for step to complete
  ovphysx_op_wait_result_t step_wait_result = {0};
  ovphysx_result_t step_wait_status = ovphysx_wait_op(handle, step_result.op_index, UINT64_MAX, &step_wait_result);
  if (step_wait_status.status != OVPHYSX_API_SUCCESS || step_wait_result.num_errors > 0) {
    fprintf(stderr, "Simulation step failed\n");
    if (step_wait_result.errors) {
      ovphysx_destroy_errors(step_wait_result.errors, step_wait_result.num_errors);
    }
    if (step_wait_status.error.ptr) {
      ovphysx_destroy_error(step_wait_status.error);
    }
    ovphysx_destroy_instance(handle);
    return 1;
  }
  
  printf("Simulation step completed successfully\n");

  // Clean up
  ovphysx_destroy_instance(handle);

  return 0;
}
