// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
// Incremental runtime spawn through ovstage population references.
//
// A simulator front end that spawns objects while the simulation is running has
// to add prims to an already-attached stage. The loop for that is:
//
//   ovstage_population_add_usd_reference_from_string  (author the new prim)
//   ovstage_population_apply_usd_changes(ordinal)     (into the ovstage)
//   ovstage_advance_write_floor(ordinal)              (seal it)
//   ovphysx_update_from_ovstage([prev+1, ordinal])    (drain it into physics)
//   ovphysx_step_sync(dt)                             (simulate)
//
// This sample runs that loop and reads the rigid-body positions back to confirm
// every spawned body is live and simulating.
//
// KNOWN LIMITATION (the reason this sample has a --repro mode)
//
// Once 64 population USD references are live on the attached stage, the next
// ovphysx_step_sync() never returns. Every add / apply / seal / drain before it
// reports success. The calling thread parks in a timed futex wait inside
// ovphysx_step_sync and the process idles at roughly 10% of one core, so it
// presents as a freeze rather than an error.
//
// The threshold counts references, not rigid bodies and not ordinals:
//
//   --count 256 --pack 16                 17 refs, 256 bodies      completes
//   --count 128 --pack 4                  33 refs, 128 bodies      completes
//   --count 128 --pack 2                  64 refs, 126 bodies      wedges
//   --count 128 --pack 1                  64 refs,  63 bodies      wedges
//   --count 128 --pack 1 --ordinal-stride 10   64 refs, ordinal 631 wedges
//   --count 63 --pack 1 --steps-between 0 --steps-after 5           wedges on
//                                         the first step after the adds
//
// (Reference counts include the seed scene's own population source. Verified on
// ovphysx 0.5.9, Linux x86_64, CPU mode and GPU mode alike.)
//
// WORKAROUND, which is what this sample does by default
//
// Author sibling bodies into ONE referenced layer instead of one layer each.
// --pack N puts N bodies per reference, so N spawned bodies cost one reference
// instead of N. Packing 16 to a reference moves the ceiling from 63 bodies to
// roughly 1000, which is enough for scenes that spawn steadily over a long run.
// It does not help a front end that must spawn exactly one object at a time and
// cannot batch, which is why the underlying limit still needs fixing.
//
// Run with --repro to reproduce the freeze (one reference per body). A SIGALRM
// watchdog turns the freeze into a diagnostic message and exit code 2 on POSIX
// so the process terminates on its own; pass --step-timeout 0 to disable it and
// leave the process wedged for a debugger.

#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_config.h"
#include "ovstage_sample.h"

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(_WIN32)
#include <windows.h>
#else
#include <signal.h>
#include <time.h>
#include <unistd.h>
#endif

#ifndef OVPHYSX_TEST_DATA
#define OVPHYSX_TEST_DATA "."
#endif

typedef struct
{
    int         count;          // rigid bodies to add in total
    int         pack;           // bodies authored per population reference
    int         steps_between;  // step_sync calls after each reference
    int         steps_after;    // step_sync calls once every reference is in
    int         ordinal_stride; // ordinals consumed per reference
    int         step_timeout_s; // watchdog: give up on a step that never returns
    int         cpu_only;
    const char* usd;
    const char* parent;
} options_t;

// ---- watchdog -------------------------------------------------------------
//
// Pre-formatted in the main flow so the POSIX handler only has to write() it.

static char   g_wedge_msg[320];
static size_t g_wedge_len = 0;

#if defined(_WIN32)

// No SIGALRM on Windows. --repro relies on an external timeout there; the
// default (workaround) path never needs the watchdog.
static void watchdog_init(void)
{
}
static void watchdog_arm(int seconds)
{
    (void)seconds;
}
static void watchdog_disarm(void)
{
}
static double monotonic_ms(void)
{
    LARGE_INTEGER freq, now;
    QueryPerformanceFrequency(&freq);
    QueryPerformanceCounter(&now);
    return (double)now.QuadPart * 1000.0 / (double)freq.QuadPart;
}

#else

static void on_alarm(int sig)
{
    (void)sig;
    if (g_wedge_len > 0)
    {
        ssize_t ignored = write(2, g_wedge_msg, g_wedge_len);
        (void)ignored;
    }
    _exit(2);
}
static void watchdog_init(void)
{
    signal(SIGALRM, on_alarm);
}
static void watchdog_arm(int seconds)
{
    alarm((unsigned)seconds);
}
static void watchdog_disarm(void)
{
    alarm(0);
}
static double monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec * 1e3 + (double)now.tv_nsec / 1e6;
}

#endif

// ---- helpers --------------------------------------------------------------

// `max` is the caller's buffer size, so each view stays bounded by its source.
static ovx_string_t str_view(const char* s, size_t max)
{
    ovx_string_t v;
    v.ptr    = s;
    v.length = strnlen(s, max);
    return v;
}

static ovx_string_or_token_t name_str(const char* s)
{
    ovx_string_or_token_t n;
    n.token         = 0;
    n.string.ptr    = s;
    n.string.length = strnlen(s, 256);
    return n;
}

static void body_pose(int i, double* x, double* y, double* z)
{
    *x = (double)(i % 8) * 1.5 - 6.0;
    *y = 3.0 + (double)(i / 8) * 1.5;
    *z = (double)((i / 8) % 8) * 1.5 - 6.0;
}

#define BODY_COLLIDER_USDA                                                    \
    "        def Cube \"Collider\" (prepend apiSchemas = "                    \
    "[\"PhysicsCollisionAPI\"])\n"                                            \
    "        {\n"                                                             \
    "            double size = 1.0\n"                                         \
    "            double3 xformOp:scale = (0.5, 0.5, 0.5)\n"                   \
    "            uniform token[] xformOpOrder = [\"xformOp:scale\"]\n"         \
    "        }\n"

// One body as its own referenced layer, targeted at an absolute prim path.
static void author_single(char* out, size_t cap, int index)
{
    double x, y, z;
    body_pose(index, &x, &y, &z);
    snprintf(out, cap,
             "#usda 1.0\n"
             "(defaultPrim = \"Body\")\n"
             "def Xform \"Body\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", "
             "\"PhysicsMassAPI\"])\n"
             "{\n"
             "    double3 xformOp:translate = (%.9g, %.9g, %.9g)\n"
             "    quatf xformOp:orient = (1, 0, 0, 0)\n"
             "    uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:orient\"]\n"
             "    float physics:mass = 1.0\n" BODY_COLLIDER_USDA "}\n",
             x, y, z);
}

// The workaround: several sibling bodies in ONE referenced layer, targeted at
// their shared parent prim, so N bodies cost a single population reference.
static void author_packed(char* out, size_t cap, int first, int last)
{
    size_t n = 0;
    n += (size_t)snprintf(out + n, cap - n,
                          "#usda 1.0\n(defaultPrim = \"Bodies\")\ndef Scope \"Bodies\"\n{\n");
    for (int i = first; i < last && n < cap; ++i)
    {
        double x, y, z;
        body_pose(i, &x, &y, &z);
        n += (size_t)snprintf(
            out + n, cap - n,
            "    def Xform \"box_%d\" (prepend apiSchemas = [\"PhysicsRigidBodyAPI\", "
            "\"PhysicsMassAPI\"])\n"
            "    {\n"
            "        double3 xformOp:translate = (%.9g, %.9g, %.9g)\n"
            "        quatf xformOp:orient = (1, 0, 0, 0)\n"
            "        uniform token[] xformOpOrder = [\"xformOp:translate\", "
            "\"xformOp:orient\"]\n"
            "        float physics:mass = 1.0\n" BODY_COLLIDER_USDA "    }\n",
            i, x, y, z);
    }
    snprintf(out + n, cap - n, "}\n");
}

static int wait_pop(ovstage_instance_t* stage, uint64_t op, const char* what)
{
    ovstage_population_op_wait_result_t wr;
    memset(&wr, 0, sizeof(wr));
    const ovstage_api_status_t st = ovstage_population_wait_op(stage, op, OVSTAGE_TIMEOUT_INFINITE, &wr);
    if (st != OVSTAGE_OK)
    {
        const ovx_string_t err = ovstage_population_get_last_error();
        fprintf(stderr, "ERROR: %s failed: %d %.*s\n", what, (int)st, (int)err.length,
                err.ptr ? err.ptr : "");
        return 0;
    }
    return 1;
}

static int wait_stage(ovstage_instance_t* stage, uint64_t op, const char* what)
{
    ovstage_op_wait_result_t wr;
    memset(&wr, 0, sizeof(wr));
    const ovstage_api_status_t st = ovstage_wait_op(stage, op, OVSTAGE_TIMEOUT_INFINITE, &wr);
    if (st != OVSTAGE_OK)
    {
        fprintf(stderr, "ERROR: %s failed: %d\n", what, (int)st);
        return 0;
    }
    return 1;
}

// Count simulated rigid bodies and summarize their height, so the sample proves
// the spawned bodies reached physics instead of only reporting API success.
typedef struct
{
    uint32_t count;
    float    min_y;
    float    max_y;
} height_stats_t;

static int read_rigid_body_heights(ovphysx_handle_t handle, height_stats_t* stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->min_y = FLT_MAX;
    stats->max_y = -FLT_MAX;

    ovphysx_query_handle_t query = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query).status
            != OVPHYSX_API_SUCCESS
        || query == 0)
    {
        return 0;
    }

    const ovx_string_or_token_t name = name_str(OVPHYSX_ATTR_POSITION);
    ovphysx_read_handle_t       read = 0;
    if (ovphysx_read(handle, query, &name, 1, &read).status != OVPHYSX_API_SUCCESS || read == 0)
    {
        ovphysx_release_query(handle, query);
        return 0;
    }

    int valid = 1;
    for (;;)
    {
        const ovstage_read_group_t* g = NULL;
        const ovphysx_result_t      r = ovphysx_fetch_read_next(handle, read, &g);
        if (r.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        if (r.status != OVPHYSX_API_SUCCESS || !g)
        {
            valid = 0;
            break;
        }

        if (!g->is_delete && !g->is_array && g->data.tensor_count == 1 && g->data.tensors)
        {
            const DLTensor* tensor = &g->data.tensors[0];
            if (!tensor->data || tensor->device.device_type != kDLCPU || tensor->dtype.code != kDLFloat
                || tensor->dtype.bits != 32 || tensor->dtype.lanes < 3 || tensor->ndim != 1
                || !tensor->shape)
            {
                valid = 0;
            }
            else
            {
                const unsigned char* base   = (const unsigned char*)tensor->data + tensor->byte_offset;
                const int64_t        stride = tensor->strides ? tensor->strides[0] : 1;
                const size_t         element_bytes = sizeof(float) * tensor->dtype.lanes;
                for (int64_t i = 0; i < tensor->shape[0]; ++i)
                {
                    const float* element = (const float*)(base + (size_t)(i * stride) * element_bytes);
                    const float  y       = element[1];
                    stats->min_y         = y < stats->min_y ? y : stats->min_y;
                    stats->max_y         = y > stats->max_y ? y : stats->max_y;
                    ++stats->count;
                }
            }
        }
        ovphysx_release_group(handle, read, g->read_group_id);
        if (!valid)
            break;
    }

    ovphysx_release_read(handle, read);
    ovphysx_release_query(handle, query);
    return valid;
}

static int step_n(ovphysx_handle_t handle, int steps, int timeout_s)
{
    for (int s = 0; s < steps; ++s)
    {
        watchdog_arm(timeout_s);
        const ovphysx_result_t st = ovphysx_step_sync(handle, 1.0f / 60.0f);
        watchdog_disarm();
        if (st.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            fprintf(stderr, "ERROR: ovphysx_step_sync failed: %.*s\n", (int)err.length,
                    err.ptr ? err.ptr : "");
            return 0;
        }
    }
    return 1;
}

static int run(const options_t* opt)
{
    if (opt->cpu_only)
    {
        // Keeps the sample CUDA-free. GPU dynamics is authored per scene in USD;
        // the reference limit behaves identically either way.
        ovphysx_set_cpu_mode(true);
    }

    // UJITSO collision cooking needs Carbonite plugins that are not part of the
    // standalone runtime, so disable it for a kitless process.
    static const char      kUjitsoKey[]   = "/physics/cooking/ujitsoCollisionCooking";
    static const char      kUjitsoValue[] = "false";
    ovphysx_config_entry_t config_entries[1];
    ovphysx_string_t       ujitso_key;
    ovphysx_string_t       ujitso_value;
    ujitso_key.ptr      = kUjitsoKey;
    ujitso_key.length   = sizeof(kUjitsoKey) - 1;
    ujitso_value.ptr    = kUjitsoValue;
    ujitso_value.length = sizeof(kUjitsoValue) - 1;
    config_entries[0]   = ovphysx_config_entry_carbonite(ujitso_key, ujitso_value);

    ovphysx_create_args create_args   = OVPHYSX_CREATE_ARGS_DEFAULT;
    create_args.config_entries        = config_entries;
    create_args.config_entry_count    = 1;

    ovphysx_handle_t handle = 0;
    if (ovphysx_create_instance(&create_args, &handle).status != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t err = ovphysx_get_last_error();
        fprintf(stderr, "ERROR: ovphysx_create_instance failed: %.*s\n", (int)err.length,
                err.ptr ? err.ptr : "");
        return 1;
    }

    ovphysx_sample_stage_attachment_t att = { 0 };
    if (!ovphysx_sample_attach_usd_with_ovstage(handle, opt->usd, &att))
    {
        fprintf(stderr, "ERROR: failed to attach seed stage '%s'\n", opt->usd);
        ovphysx_destroy_instance(handle);
        return 1;
    }

    height_stats_t seed_stats;
    if (!read_rigid_body_heights(handle, &seed_stats))
    {
        fprintf(stderr, "ERROR: failed to read seed rigid-body positions\n");
        ovphysx_sample_destroy_stage(handle, &att);
        ovphysx_destroy_instance(handle);
        return 1;
    }

    printf("seed scene: %s\n", opt->usd);
    printf("  %u rigid bodies, 1 population reference\n", seed_stats.count);
    printf("adding %d bodies, %d per reference (%d references), %d step(s) between\n\n",
           opt->count, opt->pack, (opt->count + opt->pack - 1) / opt->pack, opt->steps_between);

    ovstage_ordinal_t ordinal      = att.ordinal;
    ovstage_ordinal_t last_drained = att.ordinal;
    int               refs         = 0;
    int               ok           = 1;

    static char usda[256 * 1024];

    for (int first = 0; first < opt->count && ok; first += opt->pack)
    {
        int last = first + opt->pack;
        if (last > opt->count)
            last = opt->count;

        char target[256];
        if (last - first == 1)
        {
            author_single(usda, sizeof(usda), first);
            snprintf(target, sizeof(target), "%s/box_%d", opt->parent, first);
        }
        else
        {
            author_packed(usda, sizeof(usda), first, last);
            snprintf(target, sizeof(target), "%s", opt->parent);
        }

        const double t_add = monotonic_ms();
        ovstage_population_usd_reference_handle_t ref = 0;
        const ovstage_population_enqueue_result_t add =
            ovstage_population_add_usd_reference_from_string(att.stage, str_view(usda, sizeof(usda)),
                                                            str_view(target, sizeof(target)), &ref);
        if (add.status != OVSTAGE_OK)
        {
            fprintf(stderr, "ERROR: add_usd_reference enqueue rejected: %d\n", (int)add.status);
            ok = 0;
            break;
        }
        if (!wait_pop(att.stage, add.op_index, "add_usd_reference"))
        {
            ok = 0;
            break;
        }
        const double add_ms = monotonic_ms() - t_add;
        ++refs;

        // --ordinal-stride > 1 makes the ordinal counter outrun the reference
        // counter, which is what separates "too many ordinals" from "too many
        // references" when narrowing the limitation described at the top.
        ordinal += (ovstage_ordinal_t)opt->ordinal_stride;

        const double                             t_apply = monotonic_ms();
        const ovstage_population_enqueue_result_t apply =
            ovstage_population_apply_usd_changes(att.stage, ordinal);
        if (apply.status != OVSTAGE_OK)
        {
            fprintf(stderr, "ERROR: apply_usd_changes enqueue rejected: %d\n", (int)apply.status);
            ok = 0;
            break;
        }
        if (!wait_pop(att.stage, apply.op_index, "apply_usd_changes"))
        {
            ok = 0;
            break;
        }
        const double apply_ms = monotonic_ms() - t_apply;

        const double               t_drain = monotonic_ms();
        ovstage_write_floor_desc_t wf;
        memset(&wf, 0, sizeof(wf));
        wf.ordinal                          = ordinal;
        wf.scope                            = OVSTAGE_SCOPE_ALL;
        const ovstage_enqueue_result_t seal = ovstage_advance_write_floor(att.stage, &wf);
        if (seal.status != OVSTAGE_OK)
        {
            fprintf(stderr, "ERROR: advance_write_floor enqueue rejected: %d\n", (int)seal.status);
            ok = 0;
            break;
        }
        if (!wait_stage(att.stage, seal.op_index, "advance_write_floor"))
        {
            ok = 0;
            break;
        }

        ovstage_ordinal_range_t range;
        memset(&range, 0, sizeof(range));
        range.has_start_ordinal = true;
        range.start_ordinal     = last_drained + 1;
        range.end_ordinal       = ordinal;
        if (ovphysx_update_from_ovstage(handle, range).status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t err = ovphysx_get_last_error();
            fprintf(stderr, "ERROR: ovphysx_update_from_ovstage [%llu,%llu] failed: %.*s\n",
                    (unsigned long long)range.start_ordinal, (unsigned long long)range.end_ordinal,
                    (int)err.length, err.ptr ? err.ptr : "");
            ok = 0;
            break;
        }
        last_drained          = ordinal;
        const double drain_ms = monotonic_ms() - t_drain;

        g_wedge_len = (size_t)snprintf(
            g_wedge_msg, sizeof(g_wedge_msg),
            "\nWEDGED: ovphysx_step_sync() did not return within %d s\n"
            "  after population reference %d (%d including the seed scene)\n"
            "  bodies %d..%d, ordinal %llu, %d spawned bodies live\n"
            "  Every add / apply / seal / drain above reported success.\n",
            opt->step_timeout_s, refs, refs + 1, first, last - 1, (unsigned long long)ordinal, last);

        const double t_step = monotonic_ms();
        if (!step_n(handle, opt->steps_between, opt->step_timeout_s))
        {
            ok = 0;
            break;
        }

        printf("reference %3d (%3d with seed)  bodies %3d..%-3d  add %7.1f ms  apply %6.1f ms  "
               "drain %7.1f ms  step %6.1f ms\n",
               refs, refs + 1, first, last - 1, add_ms, apply_ms, drain_ms, monotonic_ms() - t_step);
        fflush(stdout);
    }

    // With --steps-between 0 nothing steps during the add loop, which shows the
    // freeze is a property of the accumulated references rather than of
    // interleaving topology changes with stepping.
    if (ok && opt->steps_after > 0)
    {
        g_wedge_len = (size_t)snprintf(
            g_wedge_msg, sizeof(g_wedge_msg),
            "\nWEDGED: ovphysx_step_sync() did not return within %d s\n"
            "  on the first of %d steps after the add loop, with %d population\n"
            "  references live (%d including the seed scene)\n",
            opt->step_timeout_s, opt->steps_after, refs, refs + 1);
        const double t = monotonic_ms();
        ok             = step_n(handle, opt->steps_after, opt->step_timeout_s);
        if (ok)
            printf("\n%d steps after the add loop: %.1f ms\n", opt->steps_after, monotonic_ms() - t);
    }

    if (ok)
    {
        height_stats_t final_stats;
        if (!read_rigid_body_heights(handle, &final_stats))
        {
            fprintf(stderr, "ERROR: failed to read rigid-body positions\n");
            ok = 0;
        }
        else
        {
            const uint32_t expected = seed_stats.count + (uint32_t)opt->count;
            printf("%u rigid bodies simulating over %d population references (%d with seed)\n",
                   final_stats.count, refs, refs + 1);
            printf("  height range: %.3f .. %.3f\n", final_stats.min_y, final_stats.max_y);
            if (final_stats.count != expected)
            {
                fprintf(stderr, "ERROR: expected %u rigid bodies, found %u\n", expected,
                        final_stats.count);
                ok = 0;
            }
            else
            {
                printf("\nAll %d spawned bodies reached physics. No reference limit hit.\n",
                       opt->count);
            }
        }
    }

    ovphysx_sample_destroy_stage(handle, &att);
    ovphysx_destroy_instance(handle);
    return ok ? 0 : 1;
}

static void usage(const char* argv0)
{
    printf("Usage: %s [options]\n"
           "  --repro            one reference per body until step_sync wedges\n"
           "                     (shorthand for --count 128 --pack 1)\n"
           "  --count N          rigid bodies to add (default 64)\n"
           "  --pack N           bodies per population reference (default 16)\n"
           "  --steps-between N  step_sync calls after each reference (default 1)\n"
           "  --steps-after N    step_sync calls after the add loop (default 30)\n"
           "  --ordinal-stride N ordinals consumed per reference (default 1)\n"
           "  --step-timeout N   seconds before declaring a step wedged, 0 to\n"
           "                     disable the watchdog (default 30, POSIX only)\n"
           "  --gpu              leave GPU dynamics available to the scene\n"
           "  --usd PATH         seed scene\n"
           "  --parent PATH      parent prim for spawned bodies\n",
           argv0);
}

int main(int argc, char** argv)
{
    options_t opt;
    opt.count          = 64;
    opt.pack           = 16;
    opt.steps_between  = 1;
    opt.steps_after    = 30;
    opt.ordinal_stride = 1;
    opt.step_timeout_s = 30;
    opt.cpu_only       = 1;
    opt.usd            = OVPHYSX_TEST_DATA "/simple_physics_scene.usda";
    opt.parent         = "/World/Spawned";

    for (int i = 1; i < argc; ++i)
    {
        const char* a = argv[i];
        if (strcmp(a, "--repro") == 0)
        {
            opt.count = 128;
            opt.pack  = 1;
        }
        else if (strcmp(a, "--count") == 0 && i + 1 < argc)
            opt.count = atoi(argv[++i]);
        else if (strcmp(a, "--pack") == 0 && i + 1 < argc)
            opt.pack = atoi(argv[++i]);
        else if (strcmp(a, "--steps-between") == 0 && i + 1 < argc)
            opt.steps_between = atoi(argv[++i]);
        else if (strcmp(a, "--steps-after") == 0 && i + 1 < argc)
            opt.steps_after = atoi(argv[++i]);
        else if (strcmp(a, "--ordinal-stride") == 0 && i + 1 < argc)
            opt.ordinal_stride = atoi(argv[++i]);
        else if (strcmp(a, "--step-timeout") == 0 && i + 1 < argc)
            opt.step_timeout_s = atoi(argv[++i]);
        else if (strcmp(a, "--gpu") == 0)
            opt.cpu_only = 0;
        else if (strcmp(a, "--usd") == 0 && i + 1 < argc)
            opt.usd = argv[++i];
        else if (strcmp(a, "--parent") == 0 && i + 1 < argc)
            opt.parent = argv[++i];
        else
        {
            usage(argv[0]);
            return (strcmp(a, "-h") == 0 || strcmp(a, "--help") == 0) ? 0 : 1;
        }
    }
    if (opt.count < 1 || opt.pack < 1 || opt.ordinal_stride < 1 || opt.steps_between < 0
        || opt.steps_after < 0 || opt.step_timeout_s < 0)
    {
        usage(argv[0]);
        return 1;
    }

    watchdog_init();

    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "ERROR: ovphysx_initialize failed\n");
        return 1;
    }
    const int rc = run(&opt);
    ovphysx_shutdown();
    return rc;
}
