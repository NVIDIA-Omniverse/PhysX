/*
 * SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

/* [example-start] */
#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_population.h>
#include <ovphysx/ovphysx_attributes.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_population.h>

#include <stdio.h>
#include <stdlib.h>

/* A ground plane and a 1 m box whose center starts 2 m above it. */
static const char SCENE[] =
    "#usda 1.0\n"
    "(\n"
    "    metersPerUnit = 1\n"
    "    kilogramsPerUnit = 1\n"
    "    upAxis = \"Z\"\n"
    ")\n"
    "def Xform \"World\"\n"
    "{\n"
    "    def PhysicsScene \"PhysicsScene\" {}\n"
    "    def Plane \"Ground\" (prepend apiSchemas = [\"PhysicsCollisionAPI\"])\n"
    "    {\n"
    "        uniform token axis = \"Z\"\n"
    "    }\n"
    "    def Cube \"Box\" (\n"
    "        prepend apiSchemas = [\"PhysicsRigidBodyAPI\",\n"
    "                              \"PhysicsCollisionAPI\"]\n"
    "    )\n"
    "    {\n"
    "        double size = 1\n"
    "        double3 xformOp:translate = (0, 0, 2)\n"
    "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
    "    }\n"
    "}\n";

static ovphysx_instance_handle_t physx;

/* Stop the program with a message when something fails. */
static void need(int ok, const char* what)
{
    if (!ok) {
        fprintf(stderr, "falling_box: %s failed\n", what);
        exit(1);
    }
}

/* Return 1 when a call that finishes before it returns succeeded. */
static int ok(ovphysx_result_t result)
{
    return result.status == OVPHYSX_API_SUCCESS;
}

/* Wait for an ovphysx task. Print its error and return 0 when it failed. */
static int done(ovphysx_enqueue_result_t task)
{
    ovphysx_wait_task_result_t waited = {0};
    ovphysx_result_t result = {task.status};
    if (task.status == OVPHYSX_API_SUCCESS)
        result = ovphysx_wait_task(physx, task, OVPHYSX_TIMEOUT_INFINITE, &waited);
    for (size_t i = 0; i < waited.num_errors; ++i) {
        ovphysx_string_t e =
            ovphysx_get_last_task_error(physx, waited.error_tasks[i]);
        fprintf(stderr, "ovphysx: %.*s\n", (int)e.length, e.ptr ? e.ptr : "");
    }
    if (result.status != OVPHYSX_API_SUCCESS && waited.num_errors == 0) {
        ovphysx_string_t e = ovphysx_get_last_error();
        fprintf(stderr, "ovphysx: %.*s\n", (int)e.length, e.ptr ? e.ptr : "");
    }
    ovphysx_destroy_wait_task_result(&waited);
    return result.status == OVPHYSX_API_SUCCESS;
}

/* Read the position of every rigid body and print the box's z. */
static void print_box_height(double t)
{
    const ovx_token_or_string_t position = {
        OVX_INVALID_TOKEN,
        {OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1}};
    const ovstage_cuda_sync_t no_cuda = {0, 0}; /* CPU read: no CUDA ordering */
    ovphysx_query_handle_t query = 0;
    ovphysx_read_handle_t read = 0;
    const ovstage_read_group_t* group = NULL;

    need(done(ovphysx_query(physx, OVPHYSX_QUERY_TARGET_RIGID_BODY,
                            OVPHYSX_SCOPE_ALL, &query, NULL)),
         "query");
    need(done(ovphysx_read(physx, query, &position, 1, no_cuda, &read, NULL)),
         "read");
    need(done(ovphysx_fetch_read_next(physx, read, &group)), "fetch");

    /* One row per rigid body, three float32 values per row: x, y, z. */
    const DLTensor* rows = &group->columns[0].data.tensors[0];
    need(rows->device.device_type == kDLCPU, "finding the positions in CPU memory");
    const float* xyz = (const float*)((const char*)rows->data + rows->byte_offset);
    printf("t=%.2f s  box z = %.3f m\n", t, xyz[2]);

    /* Releasing the read also frees its data. */
    need(done(ovphysx_release_read(physx, read)), "releasing the read");
    need(done(ovphysx_release_query(physx, query)), "releasing the query");
}

int main(void)
{
    /* ovstage must know the PhysX USD schemas before it reads a scene. They come
     * from the physx-usd-schemas Python package: PHYSX_USD_SCHEMAS_ROOT names the
     * folder that holds its physx_usd_schemas folder. */
    const char* root = getenv("PHYSX_USD_SCHEMAS_ROOT");
    need(root != NULL, "reading PHYSX_USD_SCHEMAS_ROOT");
    const char* modules[2] = {"PhysxSchema", "OmniUsdPhysicsDeformableSchema"};
    char dirs[2][4096];
    ovx_string_t schemas[2];
    for (int i = 0; i < 2; ++i) {
        int n = snprintf(dirs[i], sizeof dirs[i],
                         "%s/physx_usd_schemas/%s/resources", root, modules[i]);
        need(n > 0 && n < (int)sizeof dirs[i], "building a schema path");
        schemas[i] = (ovx_string_t){dirs[i], (size_t)n};
    }

    need(ok(ovphysx_initialize()), "ovphysx_initialize");
    need(ovstage_population_register_usd_schemas(schemas, 2) == OVSTAGE_OK,
         "registering the schemas");

    /* Load the scene into a stage, then seal the edits at ordinal 1. */
    ovstage_instance_desc_t desc = {.name = "falling_box"};
    ovstage_instance_t* stage = NULL;
    need(ovstage_create_instance(&desc, &stage) == OVSTAGE_OK,
         "creating the stage");

    const ovx_string_t usda = {SCENE, sizeof SCENE - 1};
    const ovstage_population_desc_t population_descs[] = {
        *ovphysx_get_physics_population_desc(),
        {.domains = OVSTAGE_POPULATION_DOMAIN_RENDERING},
    };
    ovstage_population_enqueue_result_t load =
        ovstage_population_open_usd_from_string_with_desc(stage, usda, 1,
                                                OVSTAGE_POPULATION_DEFAULT_TIME,
                                                population_descs, 2);
    ovstage_population_op_wait_result_t loaded = {0};
    need(load.status == OVSTAGE_OK &&
             ovstage_population_wait_op(stage, load.op_index,
                                        OVSTAGE_TIMEOUT_INFINITE,
                                        &loaded) == OVSTAGE_OK,
         "loading the scene");

    ovstage_write_floor_desc_t floor = {.ordinal = 1, .scope = OVSTAGE_SCOPE_ALL};
    ovstage_enqueue_result_t seal = ovstage_advance_write_floor(stage, &floor);
    ovstage_op_wait_result_t sealed = {0};
    need(seal.status == OVSTAGE_OK &&
             ovstage_wait_op(stage, seal.op_index, OVSTAGE_TIMEOUT_INFINITE,
                             &sealed) == OVSTAGE_OK &&
             sealed.error_op_id_count == 0,
         "sealing ordinal 1");
    need(ovstage_release_op(stage, seal.op_index) == OVSTAGE_OK,
         "releasing the seal");

    /* Create the simulation from the sealed stage. */
    need(ok(ovphysx_create_instance(&physx)), "creating the simulation");
    need(done(ovphysx_attach_ovstage(physx, stage, 1)), "attaching the stage");

    print_box_height(0.0);
    for (int step = 1; step <= 60; ++step) {
        need(ok(ovphysx_step_sync(physx, 1.0f / 60.0f)), "step");
        if (step % 10 == 0)
            print_box_height(step / 60.0);
    }

    need(done(ovphysx_detach_ovstage(physx)), "detaching the stage");
    need(ok(ovphysx_destroy_instance(physx)), "destroying the simulation");
    need(ovstage_destroy_instance(stage) == OVSTAGE_OK, "destroying the stage");
    need(ok(ovphysx_shutdown()), "ovphysx_shutdown");
    return 0;
}
/* [example-end] */
