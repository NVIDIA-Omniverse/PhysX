// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// NOTE: This file is included verbatim in documentation via literalinclude.
// Tutorial marker comments below define the included range.

// [tutorial-start]
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_types.h"
#include "ovstage_sample.h"

// PhysX SDK headers (shipped with the ovphysx SDK under include/physx/)
#include "PxRigidDynamic.h"
#include "foundation/PxTransform.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>

static bool check_result(ovphysx_result_t result, const char* context) {
    if (result.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "ERROR in %s: ", context);
        if (err.ptr && err.length > 0) {
            fprintf(stderr, "%.*s\n", (int)err.length, err.ptr);
        } else {
            fprintf(stderr, "status=%d\n", (int)result.status);
        }
        return false;
    }
    return true;
}

static bool wait_op(ovphysx_handle_t handle, ovphysx_op_index_t op_index, const char* context) {
    ovphysx_op_wait_result_t wait_result = {};
    ovphysx_result_t result = ovphysx_wait_op(handle, op_index, 10ULL * 1000 * 1000 * 1000, &wait_result);

    bool ok = (result.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0);
    if (!ok) {
        fprintf(stderr, "ERROR in %s: %s\n", context,
                wait_result.num_errors > 0 ? "async operation failed" : "wait failed");
    }
    ovphysx_destroy_wait_result(&wait_result);
    return ok;
}

static ovphysx_sample_stage_attachment_t g_stage_attachment{};

static int destroy_instance_and_shutdown(ovphysx_handle_t handle) {
    ovphysx_sample_destroy_stage(handle, &g_stage_attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
    return 1;
}

static int run(void)
{
    printf("=== ovphysx PhysX Interop (C++ API) ===\n");

    ovphysx_result_t result = ovphysx_initialize();
    if (!check_result(result, "initialize"))
        return 1;

    // 1. Create instance
    ovphysx_handle_t handle = 0;
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    result = ovphysx_create_instance(&args, &handle);
    if (!check_result(result, "create_instance")) {
        ovphysx_shutdown();
        return 1;
    }

    printf("Instance created.\n");

    // 2. Populate ovstage from USD and attach it
    if (!ovphysx_sample_attach_usd_with_ovstage(
            handle, OVPHYSX_TEST_DATA "/simple_physics_scene.usda", &g_stage_attachment)) {
        return destroy_instance_and_shutdown(handle);
    }

    printf("USD scene loaded.\n");

    // 3. Step once to initialize physics
    ovphysx_enqueue_result_t step_result = ovphysx_step(handle, 1.0f / 60.0f);
    if (step_result.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to enqueue step\n");
        return destroy_instance_and_shutdown(handle);
    }
    if (!wait_op(handle, step_result.op_index, "initial step")) {
        return destroy_instance_and_shutdown(handle);
    }

    printf("Initial simulation step completed.\n");

    // 4. Get PhysX pointer for the kinematic cube
    //    OVPHYSX_PHYSX_TYPE_ACTOR returns either PxRigidDynamic* or PxRigidStatic*,
    //    so cast to PxRigidActor* first, then validate the concrete type.
    void* actor_ptr = nullptr;
    result = ovphysx_get_physx_ptr(
        handle, OVPHYSX_LITERAL("/World/KinematicCube"), OVPHYSX_PHYSX_TYPE_ACTOR, &actor_ptr);
    if (!check_result(result, "get_physx_ptr")) {
        return destroy_instance_and_shutdown(handle);
    }

    physx::PxRigidActor* rigid_actor = static_cast<physx::PxRigidActor*>(actor_ptr);
    physx::PxRigidDynamic* actor = rigid_actor->is<physx::PxRigidDynamic>();
    if (!actor) {
        fprintf(stderr, "ERROR: /World/KinematicCube is not a PxRigidDynamic\n");
        return destroy_instance_and_shutdown(handle);
    }
    printf("Got PxRigidDynamic* for /World/KinematicCube\n");

    // 5. Set kinematic target to move the cube from (0,2,0) to (3,2,0)
    physx::PxTransform target(physx::PxVec3(3.0f, 2.0f, 0.0f));
    actor->setKinematicTarget(target);
    printf("Set kinematic target to (3, 2, 0)\n");

    // 6. Step again so PhysX moves the kinematic body to the target
    step_result = ovphysx_step(handle, 1.0f / 60.0f);
    if (step_result.status != OVPHYSX_API_SUCCESS) {
        fprintf(stderr, "Failed to enqueue step\n");
        return destroy_instance_and_shutdown(handle);
    }
    if (!wait_op(handle, step_result.op_index, "target step")) {
        return destroy_instance_and_shutdown(handle);
    }

    // 7. Read back pose via tensor binding to verify the body moved
    ovphysx_tensor_binding_handle_t pose_binding = 0;
    ovphysx_tensor_binding_desc_t pose_desc = {};
    pose_desc.pattern = OVPHYSX_LITERAL("/World/KinematicCube");
    pose_desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    result = ovphysx_create_tensor_binding(handle, &pose_desc, &pose_binding);
    if (!check_result(result, "create_tensor_binding")) {
        return destroy_instance_and_shutdown(handle);
    }

    // Allocate a [1, 7] tensor: [x, y, z, qx, qy, qz, qw]
    float pose_data[7] = {};
    int64_t shape[2] = {1, 7};
    DLTensor pose_tensor = {};
    pose_tensor.data = pose_data;
    pose_tensor.ndim = 2;
    pose_tensor.shape = shape;
    pose_tensor.strides = nullptr;
    pose_tensor.byte_offset = 0;
    pose_tensor.dtype = {kDLFloat, 32, 1};
    pose_tensor.device = {kDLCPU, 0};

    result = ovphysx_read_tensor_binding(handle, pose_binding, &pose_tensor);
    if (!check_result(result, "read_tensor_binding")) {
        ovphysx_destroy_tensor_binding(handle, pose_binding);
        return destroy_instance_and_shutdown(handle);
    }

    printf("Pose after setKinematicTarget: pos=(%.3f, %.3f, %.3f) quat=(%.3f, %.3f, %.3f, %.3f)\n",
           pose_data[0], pose_data[1], pose_data[2],
           pose_data[3], pose_data[4], pose_data[5], pose_data[6]);

    // Verify the body moved to the target position
    const float tolerance = 0.1f;
    bool moved = (pose_data[0] > 3.0f - tolerance) && (pose_data[0] < 3.0f + tolerance) &&
                 (pose_data[1] > 2.0f - tolerance) && (pose_data[1] < 2.0f + tolerance);

    if (moved) {
        printf("SUCCESS: Kinematic body moved to target position via direct PhysX API call.\n");
    } else {
        fprintf(stderr, "FAILED: Expected position near (3, 2, 0), got (%.3f, %.3f, %.3f)\n",
                pose_data[0], pose_data[1], pose_data[2]);
        ovphysx_destroy_tensor_binding(handle, pose_binding);
        return destroy_instance_and_shutdown(handle);
    }

    // 8. Cleanup
    ovphysx_destroy_tensor_binding(handle, pose_binding);
    ovphysx_sample_destroy_stage(handle, &g_stage_attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
    printf("Cleanup complete\n");

    return 0;
}

int main(void) {
    int rc = run();
    return rc;
}
// [tutorial-end]
