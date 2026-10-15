// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// NOTE: This file is included in the documentation via literalinclude.
// The tutorial marker comments below define the included range.

// [tutorial-start]
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/ovphysx_types.h"
#include "ovstage_sample.h"
#include <ovx/path_dictionary/path_dictionary.h>  // path_dictionary_* prim/token/string resolution

// PhysX SDK headers, shipped with the ovphysx SDK under include/physx/.
#include "PxRigidDynamic.h"
#include "foundation/PxTransform.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

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

static bool wait_task(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task, const char* context) {
    ovphysx_wait_task_result_t wait_result = {};
    ovphysx_result_t result = ovphysx_wait_task(handle, task, 10ULL * 1000 * 1000 * 1000, &wait_result);

    bool ok = (result.status == OVPHYSX_API_SUCCESS && wait_result.num_errors == 0);
    if (!ok) {
        fprintf(stderr, "ERROR in %s: %s\n", context,
                wait_result.num_errors > 0 ? "async operation failed" : "wait failed");
    }
    ovphysx_destroy_wait_task_result(&wait_result);
    return ok;
}

static ovphysx_sample_stage_attachment_t g_stage_attachment{};

static int destroy_instance_and_shutdown(ovphysx_instance_handle_t handle) {
    ovphysx_sample_destroy_stage(handle, &g_stage_attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
    return 1;
}

// Return the index of the read group's row whose prim path equals `target_path`, or -1 if absent.
// A fixed group stacks every prim into tensors[0] with row i owned by prims.list entry i, so this
// index also selects that prim's tensor row. The shared dictionary only resolves component tokens,
// so a full path is rebuilt by joining each prim's component strings with '/'. All handles and
// strings it returns are dictionary-owned and must not be freed. The group's own borrow keeps
// prims.list valid here, so no extra reference is taken.
static int find_prim_row(path_dictionary_instance_t* dict, const ovstage_read_group_t* rg,
                         const char* target_path) {
    const uint32_t n = rg->prims.count;
    std::vector<ovx_primpath_t> paths(n);
    size_t got = 0;
    if (path_dictionary_get_paths_from_path_list(
            dict, (ovx_primpath_list_t)rg->prims.list, rg->prims.offset, n, paths.data(), &got)
                .status != OVX_API_SUCCESS ||
        got != n)
        return -1;

    for (uint32_t i = 0; i < n; ++i) {
        ovx_token_t token_buffer[64];
        ovx_token_t* tokens_per_path[1] = { nullptr };
        size_t num_tokens[1] = { 0 };
        size_t processed = 0;
        if (path_dictionary_get_tokens_from_paths(dict, &paths[i], 1, token_buffer, 64,
                                                  tokens_per_path, num_tokens, &processed)
                    .status != OVX_API_SUCCESS ||
            processed != 1)
            continue;

        std::vector<ovx_string_t> comps(num_tokens[0]);
        if (path_dictionary_get_strings_from_tokens(dict, tokens_per_path[0], num_tokens[0], comps.data())
                .status != OVX_API_SUCCESS)
            continue;

        std::string full;
        for (size_t c = 0; c < num_tokens[0]; ++c) {
            full.push_back('/');
            full.append(comps[c].ptr, comps[c].length);
        }
        if (full == target_path)
            return (int)i;
    }
    return -1;
}

// Borrowed PxRigidActor address from the physxPointer column, or NULL.
static void* read_rigid_actor_pointer(ovphysx_instance_handle_t handle, const char* path) {
    ovphysx_query_handle_t query = 0;
    ovphysx_enqueue_result_t opened =
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, OVPHYSX_SCOPE_ALL, &query, nullptr);
    if (opened.status != OVPHYSX_API_SUCCESS || !wait_task(handle, opened, "query rigid actor"))
        return NULL;

    ovstage_instance_t* stage = g_stage_attachment.stage;
    path_dictionary_instance_t* dict = stage ? stage->vtable->get_path_dictionary(stage->context) : nullptr;
    if (!dict) {
        ovphysx_enqueue_result_t released_query = ovphysx_release_query(handle, query);
        wait_task(handle, released_query, "release rigid-actor query");
        return NULL;
    }

    ovx_string_or_token_t name = {};
    name.string.ptr = OVPHYSX_ATTR_PHYSX_POINTER;
    name.string.length = std::strlen(OVPHYSX_ATTR_PHYSX_POINTER);
    ovphysx_read_handle_t read = 0;
    ovphysx_enqueue_result_t started = ovphysx_read(handle, query, &name, 1, &read, nullptr);
    if (started.status != OVPHYSX_API_SUCCESS) {
        ovphysx_enqueue_result_t released_query = ovphysx_release_query(handle, query);
        wait_task(handle, released_query, "release rigid-actor query");
        return NULL;
    }
    if (!wait_task(handle, started, "read physxPointer")) {
        ovphysx_enqueue_result_t released_read = ovphysx_release_read(handle, read);
        wait_task(handle, released_read, "release physxPointer read");
        ovphysx_enqueue_result_t released_query = ovphysx_release_query(handle, query);
        wait_task(handle, released_query, "release rigid-actor query");
        return NULL;
    }

    void* found = NULL;
    for (;;) {
        const ovstage_read_group_t* group = nullptr;
        ovphysx_enqueue_result_t fetched = ovphysx_fetch_read_next(handle, read, &group);
        if (fetched.status != OVPHYSX_API_SUCCESS || !wait_task(handle, fetched, "fetch physxPointer") || !group)
            break;
        const DLTensor* tensor = group->data.tensors;
        const int row = find_prim_row(dict, group, path);
        if (row >= 0 && tensor && tensor->data && tensor->dtype.code == kDLUInt && tensor->dtype.bits == 64)
            found = reinterpret_cast<void*>(static_cast<uint64_t*>(tensor->data)[row]);
        ovphysx_enqueue_result_t released = ovphysx_release_group(handle, read, group->read_group_id);
        wait_task(handle, released, "release physxPointer group");
        if (found)
            break;
    }
    ovphysx_enqueue_result_t released_read = ovphysx_release_read(handle, read);
    wait_task(handle, released_read, "release physxPointer read");
    ovphysx_enqueue_result_t released_query = ovphysx_release_query(handle, query);
    wait_task(handle, released_query, "release rigid-actor query");
    return found;
}

static int run(void)
{
    printf("=== ovphysx PhysX Interop (C++ API) ===\n");

    ovphysx_result_t result = ovphysx_initialize();
    if (!check_result(result, "initialize"))
        return 1;

    // 1. Create instance
    ovphysx_instance_handle_t handle = 0;
    result = ovphysx_create_instance(&handle);
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
    if (!wait_task(handle, step_result, "initial step")) {
        return destroy_instance_and_shutdown(handle);
    }

    printf("Initial simulation step completed.\n");

    // 4. Read the borrowed PhysX address for the kinematic cube.
    //    A rigid-actor row is PxRigidDynamic or PxRigidStatic, so cast to
    //    PxRigidActor first, then validate the concrete type.
    void* actor_ptr = read_rigid_actor_pointer(handle, "/World/KinematicCube");
    if (!actor_ptr) {
        fprintf(stderr, "ERROR: physxPointer for /World/KinematicCube was absent\n");
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
    if (!wait_task(handle, step_result, "target step")) {
        return destroy_instance_and_shutdown(handle);
    }

    // 7. Read the pose back through ovphysx, which is the point of the interop round-trip. The
    //    actor was driven with the raw PhysX pointer. Reading rigid-body positions with the session
    //    read API (ovphysx_query + ovphysx_read) confirms the ovphysx runtime observed the move.
    //    The kinematic cube is the only body driven to (3, 2, 0). The dynamic ones fall away.
    ovphysx_query_handle_t read_query = 0;
    result = ovphysx_sample_complete_task(handle, ovphysx_query(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ALL, &read_query, nullptr));
    if (!check_result(result, "query") || read_query == 0) {
        return destroy_instance_and_shutdown(handle);
    }
    const ovx_string_or_token_t pos_attr = { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } };
    ovphysx_read_handle_t read_session = 0;
    result = ovphysx_sample_complete_task(handle, ovphysx_read(handle, read_query, &pos_attr, 1, &read_session, nullptr));
    if (!check_result(result, "read")) {
        return destroy_instance_and_shutdown(handle);
    }

    // The attached Stage's own dictionary interned the group prim lists. It resolves each row's prim
    // path so that /World/KinematicCube specifically is verified, not merely some body that drifted
    // near the target. Matching on x/y alone would false-pass on a future body at the same x/y or
    // a target-z regression.
    ovstage_instance_t* stage = g_stage_attachment.stage;
    path_dictionary_instance_t* dict = stage ? stage->vtable->get_path_dictionary(stage->context) : nullptr;
    if (!dict) {
        fprintf(stderr, "The attached Stage published no path dictionary\n");
        return destroy_instance_and_shutdown(handle);
    }

    const float tolerance = 0.1f;
    bool found_cube = false;
    float obs_x = 0.0f, obs_y = 0.0f, obs_z = 0.0f;
    for (;;) {
        const ovstage_read_group_t* rg = nullptr;
        const ovphysx_result_t r = ovphysx_sample_complete_task(handle, ovphysx_fetch_read_next(handle, read_session, &rg));
        if (r.status == OVPHYSX_API_SUCCESS && !rg)
            break;  // exhausted. The only non-failure exit
        if (r.status != OVPHYSX_API_SUCCESS) {
            fprintf(stderr, "ovphysx_fetch_read_next(RIGID_BODY) failed (status %d)\n", (int)r.status);
            return destroy_instance_and_shutdown(handle);
        }
        // A fixed rigid-body position group stacks every body into tensors[0] ([N,3]) with row i ==
        // prim i, so resolve prims.list and pick the /World/KinematicCube row exactly.
        if (!found_cube && !rg->is_delete && rg->data.tensor_count == 1 && rg->data.tensors &&
            rg->prims.list != 0) {
            const DLTensor& t = rg->data.tensors[0];
            if (t.data && t.device.device_type == kDLCPU && t.dtype.lanes == 3) {
                const int cube_row = find_prim_row(dict, rg, "/World/KinematicCube");
                if (cube_row >= 0 && cube_row < t.shape[0]) {
                    const float* p = static_cast<const float*>(t.data);
                    obs_x = p[cube_row * 3 + 0];
                    obs_y = p[cube_row * 3 + 1];
                    obs_z = p[cube_row * 3 + 2];
                    found_cube = true;
                }
            }
        }
        if (ovphysx_sample_complete_task(handle, ovphysx_release_group(handle, read_session, rg->read_group_id)).status != OVPHYSX_API_SUCCESS)
        return destroy_instance_and_shutdown(handle);
    }
    if (ovphysx_sample_complete_task(handle, ovphysx_release_read(handle, read_session)).status != OVPHYSX_API_SUCCESS)
        return destroy_instance_and_shutdown(handle);
    if (ovphysx_sample_complete_task(handle, ovphysx_release_query(handle, read_query)).status != OVPHYSX_API_SUCCESS)
        return destroy_instance_and_shutdown(handle);

    if (!found_cube) {
        fprintf(stderr, "FAILED: /World/KinematicCube not present in the ovphysx_read read-back.\n");
        return destroy_instance_and_shutdown(handle);
    }
    // Check the full kinematic target driven through the raw PhysX pointer, (3, 2, 0) on every axis.
    if (!(obs_x > 3.0f - tolerance && obs_x < 3.0f + tolerance &&
          obs_y > 2.0f - tolerance && obs_y < 2.0f + tolerance &&
          obs_z > 0.0f - tolerance && obs_z < 0.0f + tolerance)) {
        fprintf(stderr, "FAILED: /World/KinematicCube read back at (%.3f, %.3f, %.3f), expected (3, 2, 0).\n",
                obs_x, obs_y, obs_z);
        return destroy_instance_and_shutdown(handle);
    }
    printf("SUCCESS: ovphysx_read observed /World/KinematicCube at (%.3f, %.3f, %.3f) -- "
           "the runtime saw the move made through the raw PhysX pointer.\n", obs_x, obs_y, obs_z);

    // 8. Cleanup
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
