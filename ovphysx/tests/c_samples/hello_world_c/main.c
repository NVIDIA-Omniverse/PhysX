// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// NOTE: This file is included verbatim in documentation via literalinclude.
// Tutorial marker comments below define the included range.

// Compile-time check: fail compilation if C++ compiler is used (for internal testing)
#ifdef __cplusplus
#error "This file should be compiled as C, not C++"
#endif

// [tutorial-start]
#include "ovphysx/ovphysx.h"
#include "ovstage_sample.h"
#include <stdio.h>

static int run(void)
{
  // Create PhysX instance with default args
  ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
  ovphysx_handle_t handle = 0;
  
  ovphysx_result_t result = ovphysx_create_instance(&create_args, &handle);
  if (result.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to create PhysX instance\n");
  ovphysx_shutdown();
    return 1;
  }

  // Populate ovstage from USD and attach it to ovphysx
  ovphysx_sample_stage_attachment_t stage_attachment = {0};
  if (!ovphysx_sample_attach_usd_with_ovstage(
          handle, OVPHYSX_TEST_DATA "/simple_physics_scene.usda", &stage_attachment)) {
    fprintf(stderr, "Failed to attach ovstage scene\n");
    ovphysx_destroy_instance(handle);
  ovphysx_shutdown();
    return 1;
  }

  // Step the simulation
  ovphysx_enqueue_result_t step_result = ovphysx_step(handle, 0.016f);
  if (step_result.status != OVPHYSX_API_SUCCESS) {
    fprintf(stderr, "Failed to step simulation\n");
    ovphysx_sample_destroy_stage(handle, &stage_attachment);
    ovphysx_destroy_instance(handle);
  ovphysx_shutdown();
    return 1;
  }

  // Wait for step to complete
  ovphysx_op_wait_result_t step_wait_result = {0};
  ovphysx_result_t step_wait_status = ovphysx_wait_op(handle, step_result.op_index, UINT64_MAX, &step_wait_result);
  int step_ok = (step_wait_status.status == OVPHYSX_API_SUCCESS && step_wait_result.num_errors == 0);
  ovphysx_destroy_wait_result(&step_wait_result);
  if (!step_ok) {
    fprintf(stderr, "Simulation step failed\n");
    ovphysx_sample_destroy_stage(handle, &stage_attachment);
    ovphysx_destroy_instance(handle);
  ovphysx_shutdown();
    return 1;
  }

  printf("Simulation step completed successfully\n");

  ovphysx_sample_destroy_stage(handle, &stage_attachment);
  ovphysx_destroy_instance(handle);
  ovphysx_shutdown();
  printf("Cleanup complete\n");

  return 0;
}

int main(void) {
    ovphysx_result_t init_r = ovphysx_initialize();
    if (init_r.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "ovphysx_initialize() failed\n");
        return 1;
    }
  int rc = run();
  return rc;
}
// [tutorial-end]
