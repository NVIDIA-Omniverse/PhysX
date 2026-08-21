// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// NOTE: This file is included verbatim in documentation via literalinclude.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_types.h>
#include "ovstage_sample.h"
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
    return 0;
  }
  ovphysx_op_wait_result_t wait_result = {0};
  ovphysx_result_t wait_res = ovphysx_wait_op(handle, res.op_index, timeout_ns, &wait_result);
  int success = (wait_res.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0);
  ovphysx_destroy_wait_result(&wait_result);
  return success;
}

static int run(void)
{
  printf("=== ovphysx Clone Example (C API) ===\n\n");

  ovphysx_result_t init_res = ovphysx_initialize();
  if (init_res.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to initialize ovphysx\n");
    return 1;
  }

  // Initialize create args with defaults
  ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;

  // Create PhysX instance
  printf("Creating PhysX instance...\n");
  ovphysx_handle_t handle = 0;
  ovphysx_result_t create_res = ovphysx_create_instance(&create_args, &handle);
  if (create_res.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to create PhysX instance\n");
    ovphysx_shutdown();
    return 1;
  }
  printf("  [OK] PhysX instance created\n\n");

  // Populate ovstage from USD and attach it to ovphysx
  printf("Loading USD scene...\n");
  ovphysx_sample_stage_attachment_t stage_attachment = {0};
  if (!ovphysx_sample_attach_usd_with_ovstage(
          handle, OVPHYSX_TEST_DATA "/basic_simulation.usda", &stage_attachment)) {
    fprintf(stderr, "ovstage attach/update failed\n");
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
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
      NUM_TARGETS,
      NULL,   /* parent_transforms: co-locate on the source */
      NULL);  /* env_ids: automatic per-call numbering */
  if (!wait_op_success(handle, clone_res, 10ULL * 1000 * 1000 * 1000)) {
    fprintf(stderr, "Clone operation failed or timed out\n");
    ovphysx_sample_destroy_stage(handle, &stage_attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
    return 1;
  }
  printf("  [OK] Created 3 clones successfully\n\n");

  // Run a few simulation steps to verify clones work correctly
  printf("Running simulation with clones (10 steps)...\n");
  for (int i = 0; i < 10; i++) {
    ovphysx_enqueue_result_t step_res = ovphysx_step(handle, 1.0f/60.0f);
    if (!wait_op_success(handle, step_res, 10ULL * 1000 * 1000 * 1000)) {
      fprintf(stderr, "Failed to run simulation step %d\n", i);
      ovphysx_sample_destroy_stage(handle, &stage_attachment);
      ovphysx_destroy_instance(handle);
      ovphysx_shutdown();
      return 1;
    }
  }
  printf("  [OK] All 10 simulation steps completed successfully\n\n");

  printf("=== Clone Example Completed Successfully ===\n");

  ovphysx_sample_destroy_stage(handle, &stage_attachment);
  ovphysx_result_t destroy_res = ovphysx_destroy_instance(handle);
  ovphysx_shutdown();
  printf("Cleanup complete\n");

  return 0;
}

int main(void) {
  int rc = run();
  return rc;
}
