// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// NOTE: This file is included verbatim in documentation.
// When editing, keep tutorial line ranges in sync.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_types.h>
#include <stdio.h>

#ifdef _WIN32
#include <windows.h>
#define sleep_ms(ms) Sleep(ms)
#else
#include <unistd.h>
#define sleep_ms(ms) usleep((ms) * 1000)
#endif

// Compile-time check: fail compilation if C++ compiler is used
#ifdef __cplusplus
#error "This file must be compiled as C, not C++"
#endif

static int wait_op_success(ovphysx_handle_t handle, ovphysx_enqueue_result_t res, uint64_t timeout_ns) {
  if (res.status != OVPHYSX_API_SUCCESS) {
    if (res.error.length > 0) {
      ovphysx_destroy_error(res.error);
    }
    return 0;
  }
  if (res.error.length > 0) {
    ovphysx_destroy_error(res.error);
  }
  ovphysx_op_wait_result_t wait_result = {0};
  ovphysx_result_t wait_res = ovphysx_wait_op(handle, res.op_index, timeout_ns, &wait_result);
  if (wait_result.errors && wait_result.num_errors > 0) {
    ovphysx_destroy_errors(wait_result.errors, wait_result.num_errors);
  }
  int success = wait_res.status == OVPHYSX_API_SUCCESS;
  if (wait_res.error.length > 0) {
    ovphysx_destroy_error(wait_res.error);
  }
  return success;
}

int main() {
  printf("=== ovphysx Clone Example (C API) ===\n\n");

  // Initialize create args with defaults
  ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;

  // Create PhysX instance
  printf("Creating PhysX instance...\n");
  ovphysx_handle_t handle = 0;
  ovphysx_result_t create_res = ovphysx_create_instance(&create_args, &handle);
  if (create_res.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to create PhysX instance\n");
    if (create_res.error.length > 0) {
      ovphysx_destroy_error(create_res.error);
    }
    return 1;
  }
  if (create_res.error.length > 0) {
    ovphysx_destroy_error(create_res.error);
  }
  printf("  [OK] PhysX instance created\n\n");

  // Load USD scene with physics content
  printf("Loading USD scene...\n");
  ovphysx_usd_handle_t usd_handle = 0;
  ovphysx_enqueue_result_t load_res = ovphysx_add_usd(handle, ovphysx_cstr(OVPHYSX_TEST_DATA "/basic_simulation.usda"), ovphysx_cstr(""), &usd_handle);
  if (!wait_op_success(handle, load_res, 10ULL * 1000 * 1000 * 1000)) {
    fprintf(stderr, "USD loading failed or timed out\n");
    ovphysx_destroy_instance(handle);
    return 1;
  }
  printf("  [OK] USD scene loaded\n\n");

  // Clone the environment (source: env0, targets: env1, env2, env3)
  printf("Cloning /World/envs/env0 to create env1, env2, env3...\n");
  const char* clone_targets[] = {
    "/World/envs/env1",
    "/World/envs/env2",
    "/World/envs/env3"
  };
  enum { NUM_TARGETS = 3 };

  ovphysx_string_t target_strings[NUM_TARGETS];
  for (uint32_t i = 0; i < NUM_TARGETS; ++i) {
    target_strings[i] = ovphysx_cstr(clone_targets[i]);
  }

  ovphysx_enqueue_result_t clone_res = ovphysx_clone(
      handle,
      ovphysx_cstr("/World/envs/env0"),
      target_strings,
      NUM_TARGETS);
  if (!wait_op_success(handle, clone_res, 10ULL * 1000 * 1000 * 1000)) {
    fprintf(stderr, "Clone operation failed or timed out\n");
    ovphysx_destroy_instance(handle);
    return 1;
  }
  printf("  [OK] Created 3 clones successfully\n\n");

  // Run a few simulation steps to verify clones work correctly
  printf("Running simulation with clones (10 steps)...\n");
  for (int i = 0; i < 10; i++) {
    ovphysx_enqueue_result_t step_res = ovphysx_step(handle, 1.0f/60.0f, (float)i / 60.0f);
    if (!wait_op_success(handle, step_res, 10ULL * 1000 * 1000 * 1000)) {
      fprintf(stderr, "Failed to run simulation step %d\n", i);
      ovphysx_destroy_instance(handle);
      return 1;
    }
  }
  printf("  [OK] All 10 simulation steps completed successfully\n\n");

  // Clean up
  printf("Cleaning up...\n");
  ovphysx_result_t destroy_res = ovphysx_destroy_instance(handle);
  if (destroy_res.error.length > 0) {
    ovphysx_destroy_error(destroy_res.error);
  }
  printf("  [OK] PhysX instance destroyed\n\n");

  printf("=== Clone Example Completed Successfully ===\n");
  return 0;
}
