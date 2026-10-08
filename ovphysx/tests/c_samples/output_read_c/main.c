// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-BUILD-UNIBUILD-001
 * @covers AC-7
 */

// NOTE: This file is included in the documentation via literalinclude.
// The tutorial marker comments below define the included range.
//
// Closed-loop ovstage sample (ADR-0007). Control is driven into ovphysx through
// the attached ovstage Stage. After a step, a representative set of simulated
// object types is queried, including the whole-articulation type, and each
// returned column is written straight back into ovstage. This scene has no
// articulation, so that query is empty.
//
// The read is ovstage-native. ovphysx_fetch_read_next hands back ovstage's own
// ovstage_read_group_t, which carries the interned prims.list handle and the
// borrowed ovstage column tensors. The write-back reuses both verbatim, with no
// path rebuild and no tensor repack. The group struct and prim list stay valid
// until ovphysx_release_group, and the numeric storage until ovphysx_release_read.
//
// The destination attribute name is also an interned token. The sample renames
// the output to "sim:<name>" and resolves and interns that token once per
// attribute (sim_token_cache_t) rather than once per group per frame. The
// dictionary used for that is ovstage's own shared source dictionary, fetched
// via ovphysx_query_shared_dictionary. There is no separate ovphysx-owned
// dictionary.
//
// Physics must not re-ingest its own output (see the ovstage Integration guide).
// Control edits flow through ovphysx_update_from_ovstage, and physics output is
// written at ordinals that call never covers. The sample interleaves two ordinal
// lanes for that: an even control lane that is drained and an odd output lane
// that never is.

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_types.h>

#include "ovstage_sample.h"                       // pulls in ovstage.h (write API + DLTensor)
#include <ovx/path_dictionary/path_dictionary.h>  // token -> string resolution (vtable + wrappers)

#include <float.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// The sample exercises the plain C API, so a C++ compiler is rejected.
#ifdef __cplusplus
#error "This file must be compiled as C, not C++"
#endif

// [tutorial-start]
// A representative output-attribute set (see the ovstage Integration guide).
// ovphysx_read skips names a type does not emit, and a type with no objects
// yields no groups, so iterating this table is safe on any scene.
typedef struct
{
    ovphysx_sim_object_type_t type;
    const char* attrs[4];
    size_t attr_count;
} output_spec_t;

static const output_spec_t OUTPUT_SPECS[] = {
    { OVPHYSX_OBJECT_RIGID_BODY,
      { OVPHYSX_ATTR_POSITION, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_ATTR_ANGULAR_VELOCITY }, 4 },
    { OVPHYSX_OBJECT_ARTICULATION_LINK,
      { OVPHYSX_ATTR_POSITION, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_ATTR_LINEAR_VELOCITY, OVPHYSX_ATTR_ANGULAR_VELOCITY }, 4 },
    { OVPHYSX_OBJECT_ARTICULATION,
      { OVPHYSX_ATTR_ROOT_POSITION, OVPHYSX_ATTR_ROOT_ORIENTATION, OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY,
        OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY },
      4 },
    { OVPHYSX_OBJECT_ARTICULATION_JOINT, { OVPHYSX_ATTR_JOINT_POSITION, OVPHYSX_ATTR_JOINT_VELOCITY }, 2 },
    { OVPHYSX_OBJECT_VEHICLE_WHEEL, { OVPHYSX_ATTR_POSITION, OVPHYSX_ATTR_ORIENTATION }, 2 },
    { OVPHYSX_OBJECT_DEFORMABLE_VOLUME, { OVPHYSX_ATTR_POINTS, OVPHYSX_ATTR_VELOCITIES }, 2 },
    { OVPHYSX_OBJECT_DEFORMABLE_SURFACE, { OVPHYSX_ATTR_POINTS, OVPHYSX_ATTR_VELOCITIES }, 2 },
    { OVPHYSX_OBJECT_PARTICLE_SET, { OVPHYSX_ATTR_POINTS, OVPHYSX_ATTR_VELOCITIES }, 2 },
};
enum { NUM_OUTPUT_SPECS = (int)(sizeof(OUTPUT_SPECS) / sizeof(OUTPUT_SPECS[0])) };

// Block on an ovstage enqueue op (write or floor advance) and release it.
// Returns 1 on success, 0 on failure. The outcome is returned rather than only
// printed, so a run whose writes all failed cannot exit 0.
static int ovstage_sample_wait(ovstage_instance_t* inst, ovstage_enqueue_result_t r)
{
    if (r.status != OVSTAGE_OK)
    {
        fprintf(stderr, "ovstage enqueue failed: %s\n", ovstage_get_error_string(inst, r.status));
        return 0;
    }
    int ok = 1;
    if (r.op_index != OVSTAGE_INVALID_OP_ID)
    {
        ovstage_op_wait_result_t wait_result = { 0 };
        const ovstage_api_status_t status = ovstage_wait_op(inst, r.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait_result);
        if (status != OVSTAGE_OK)
        {
            ok = 0;
            fprintf(stderr, "ovstage operation failed: %s\n", ovstage_get_error_string(inst, status));
            for (size_t i = 0; i < wait_result.error_op_id_count; ++i)
            {
                const ovx_string_t error = ovstage_get_last_op_error(inst, wait_result.error_op_ids[i]);
                fprintf(stderr, "  %.*s\n", (int)error.length, error.ptr ? error.ptr : "");
            }
        }
        ovstage_release_op(inst, r.op_index);
    }
    return ok;
}

static int advance_floor(ovstage_instance_t* inst, ovstage_ordinal_t ordinal)
{
    ovstage_write_floor_desc_t desc;
    memset(&desc, 0, sizeof(desc));
    desc.ordinal = ordinal;
    desc.scope = OVSTAGE_SCOPE_ALL;
    return ovstage_sample_wait(inst, ovstage_advance_write_floor(inst, &desc));
}

static ovx_string_or_token_t name_str(const char* s)
{
    ovx_string_or_token_t n;
    n.token = 0;
    n.string.ptr = s;
    /* Callers pass short attribute and type identifiers, so 256 bounds the scan. */
    n.string.length = strnlen(s, 256);
    return n;
}

// An attribute name given as an already-interned token. A non-zero token selects
// the token form and the string side is ignored. Reusing an interned token skips
// the string round-trip ovstage would otherwise do to re-intern a name.
static ovx_string_or_token_t name_tok(ovx_token_t tok)
{
    ovx_string_or_token_t n;
    n.token = tok;
    n.string.ptr = NULL;
    n.string.length = 0;
    return n;
}

// Caches the interned "sim:<name>" token for each emitted attribute token, so the
// token -> string -> "sim:" concat -> re-intern work happens once per attribute
// name for the whole run, not once per group per frame. The map keys on the
// emitted group->attribute token, which is stable because every group is
// interned in the same shared source dictionary.
enum { SIM_TOKEN_CACHE_CAP = 16 };
typedef struct
{
    ovx_token_t emitted[SIM_TOKEN_CACHE_CAP]; // group->attribute as read back
    ovx_token_t sim[SIM_TOKEN_CACHE_CAP];     // interned "sim:<name>" (0 == unresolved)
    int count;
} sim_token_cache_t;

// Return the interned "sim:<name>" token for an emitted attribute token, interning
// it on first sight. Returns 0 if the name cannot be resolved or interned, and
// the caller then falls back to a literal string name.
static ovx_token_t sim_token_for(
    sim_token_cache_t* cache, path_dictionary_instance_t* dict, ovx_token_t emitted)
{
    for (int i = 0; i < cache->count; ++i)
        if (cache->emitted[i] == emitted)
            return cache->sim[i];

    ovx_token_t sim = 0;
    ovx_string_t name = { NULL, 0 };
    if (dict && path_dictionary_get_strings_from_tokens(dict, &emitted, 1, &name).status == OVX_API_SUCCESS
        && name.ptr)
    {
        char buf[160];
        memcpy(buf, "sim:", 4);
        size_t n = name.length < 150 ? name.length : 150;
        memcpy(buf + 4, name.ptr, n);
        buf[4 + n] = '\0';
        ovx_string_t s = { buf, strnlen(buf, sizeof buf) };
        // Interning is idempotent. Equal strings yield equal tokens, so this both
        // mints the token and lets the cache short-circuit every later sighting.
        ovx_token_t minted = 0;
        if (path_dictionary_create_tokens_from_strings(dict, &s, 1, &minted).status == OVX_API_SUCCESS)
            sim = minted;
    }

    if (cache->count < SIM_TOKEN_CACHE_CAP)
    {
        cache->emitted[cache->count] = emitted;
        cache->sim[cache->count] = sim;
        ++cache->count;
    }
    return sim;
}

// app -> physics: author physics:velocity on every prim covered by `g`, reusing
// the group's interned prim_list directly with no path rebuild. Only the new
// velocity values are built. The prim set is reused verbatim.
static int author_velocity(
    ovstage_instance_t* inst, const ovstage_read_group_t* g, ovstage_ordinal_t ordinal, const float vel[3])
{
    const uint32_t count = g->prims.count;
    if (count == 0 || g->prims.list == 0)
        return 1; // nothing to author is not a failure

    float* values = (float*)malloc((size_t)count * 3 * sizeof(float));
    for (uint32_t i = 0; i < count; ++i)
    {
        values[i * 3 + 0] = vel[0];
        values[i * 3 + 1] = vel[1];
        values[i * 3 + 2] = vel[2];
    }

    int64_t shape = (int64_t)count;
    DLTensor t;
    memset(&t, 0, sizeof(t));
    t.data = values;
    t.device.device_type = kDLCPU;
    t.ndim = 1;
    t.shape = &shape;
    t.dtype.code = kDLFloat;
    t.dtype.bits = 32;
    t.dtype.lanes = 3;

    ovstage_write_data_t w;
    memset(&w, 0, sizeof(w));
    w.tensors = &t;
    w.tensor_count = 1;
    w.is_array = false;
    // Match the existing USD attribute's metadata. Updating physics:velocity
    // stamps no new semantic.
    w.semantic = OVSTAGE_SEMANTIC_NONE;

    int ok = 0;
    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(inst, (ovx_primpath_list_t)g->prims.list, &q) == OVSTAGE_OK
        && q != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ok = ovstage_sample_wait(inst, ovstage_write_attribute(inst, q, name_str("physics:velocity"), ordinal, w,
                                                               OVSTAGE_PRIM_MODE_UPSERT));
        // Released either way. A failed write still owns the query.
        ok = ovstage_sample_wait(inst, ovstage_release_query(inst, q)) && ok;
    }
    else
    {
        fprintf(stderr, "ovstage_query_from_path_list failed for physics:velocity\n");
    }
    free(values);
    return ok;
}

// One pinned write-back, held until its ordinal is sealed so it can be read back.
//
// The output ordinal is above the effective write floor until advance_floor() seals it, and
// ovstage refuses to read state above the floor with OVSTAGE_ERROR_WRITE_FLOOR_VIOLATION, for
// both snapshot and range reads. The round trip therefore straddles the seal: pin during the
// write, read after advance_floor().
//
// The prim list is borrowed from the read group and dies with it, so a reference is taken here
// and released after the read back. That is the path_dictionary ownership rule for handles that
// outlive the producing op.
typedef struct
{
    int armed;
    ovx_primpath_list_t list;
    ovx_token_t attr;
    ovstage_ordinal_t ordinal;
    path_dictionary_instance_t* dict;
} pending_readback_t;

// Read the pinned sim:<name> column back out of ovstage after its ordinal has been sealed.
//
// An accepted write only proves acceptance. An accepted write whose value cannot be retrieved
// is exactly the failure this sample has to catch, so the loop is only closed by this read.
// The prim list the write used is reused, so the prims are identical by construction and a
// mismatch can only be the write-back.
//
// Returns 1 if a group carrying data came back. Always clears the pin.
static int verify_pending_readback(ovstage_instance_t* inst, pending_readback_t* pending)
{
    if (!pending->armed)
        return 1; // nothing pinned this frame. run() checks that one was verified overall

    int ok = 0;
    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(inst, pending->list, &q) == OVSTAGE_OK
        && q != OVSTAGE_INVALID_QUERY_HANDLE)
    {
        ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;
        // A latest snapshot, not a range pinned to the write. After the seal the column's latest
        // committed payload is the value just written, and pinning the ordinal would trip the
        // write-floor check.
        ovstage_ordinal_range_t range;
        memset(&range, 0, sizeof(range));
        range.end_ordinal = pending->ordinal;
        range.has_start_ordinal = false;

        const ovstage_enqueue_result_t rr =
            ovstage_read_attributes(inst, q, &pending->attr, 1, range, &read);
        if (rr.status != OVSTAGE_OK)
        {
            fprintf(stderr, "read-back: ovstage_read_attributes failed (status %d)\n", (int)rr.status);
        }
        else
        {
            int seen_values = 0;
            for (;;)
            {
                ovstage_read_group_t g;
                memset(&g, 0, sizeof(g));
                const ovstage_api_status_t fs =
                    ovstage_fetch_read_next(inst, read, OVSTAGE_TIMEOUT_INFINITE, &g);
                if (fs == OVSTAGE_ERROR_END_OF_ITERATION)
                    break;
                if (fs != OVSTAGE_OK)
                {
                    fprintf(stderr, "read-back: ovstage_fetch_read_next failed (status %d)\n", (int)fs);
                    break;
                }
                // A delete row carries no values. Counting it would let an erased column read as a
                // successful round trip.
                if (!g.is_delete && g.data.tensor_count > 0 && g.data.tensors)
                    ++seen_values;
                ovstage_release_group(inst, &g);
            }
            ok = ovstage_sample_wait(inst, rr) && seen_values > 0;
            ok = ovstage_sample_wait(inst, ovstage_release_read(inst, read)) && ok;
            if (seen_values == 0)
                fprintf(stderr, "read-back: no sim column came back at ordinal %llu\n",
                        (unsigned long long)pending->ordinal);
        }
        ovstage_sample_wait(inst, ovstage_release_query(inst, q));
    }
    else
    {
        fprintf(stderr, "read-back: ovstage_query_from_path_list failed\n");
    }

    // Drop the pin either way. A failed read must not leak the reference.
    if (pending->dict)
        path_dictionary_release_path_list_reference(pending->dict, pending->list);
    memset(pending, 0, sizeof(*pending));
    return ok;
}

// physics -> app: write one read-output group back into ovstage under
// sim:<attribute> at `ordinal`, reusing the group's interned prim_list and its
// borrowed ovstage column tensors verbatim. The destination name is the cached
// interned "sim:<name>" token (see sim_token_for), so the write passes a token
// with no per-call string work. These output ordinals are never drained by
// ovphysx_update_from_ovstage. Returns 1 if the group was written back or there
// was nothing to write, 0 on failure.
static int dump_group(ovstage_instance_t* inst,
                      sim_token_cache_t* tokens,
                      path_dictionary_instance_t* dict,
                      const ovstage_read_group_t* g,
                      ovstage_ordinal_t ordinal,
                      int* out_written,
                      pending_readback_t* out_pending)
{
    if (g->is_delete || g->data.tensor_count == 0 || !g->data.tensors || g->prims.list == 0)
        return 1; // nothing to write back. Not a failure and not a dumped group either

    // The interned "sim:<name>" token for this emitted attribute, resolved once and
    // then cached. It comes from the same shared dictionary as the group, so it is
    // valid as the ovstage write target with no re-intern. A literal name is the
    // fallback when the token could not be resolved.
    const ovx_token_t sim = sim_token_for(tokens, dict, (ovx_token_t)g->attribute);
    const ovx_string_or_token_t attr = sim ? name_tok(sim) : name_str("sim:value");

    // Reuse the group's data block verbatim. The borrowed tensors are already in
    // ovstage column layout (tuple width in dtype.lanes), so they assign straight
    // into the write payload without reshape or copy.
    ovstage_write_data_t w;
    memset(&w, 0, sizeof(w));
    w.tensors = g->data.tensors;
    w.tensor_count = g->data.tensor_count;
    w.semantic = g->semantic;
    w.is_array = g->is_array;

    ovstage_query_handle_t q = OVSTAGE_INVALID_QUERY_HANDLE;
    if (ovstage_query_from_path_list(inst, (ovx_primpath_list_t)g->prims.list, &q) != OVSTAGE_OK
        || q == OVSTAGE_INVALID_QUERY_HANDLE)
    {
        fprintf(stderr, "ovstage_query_from_path_list failed for a read group\n");
        return 0;
    }

    int ok = ovstage_sample_wait(inst, ovstage_write_attribute(inst, q, attr, ordinal, w, OVSTAGE_PRIM_MODE_UPSERT));

    // Pin one write per run for the post-seal read-back. It is pinned rather than read here
    // because the ordinal is still above the write floor (see pending_readback_t). One per run
    // is enough, since the round trip has the same shape for every attribute. The pin is skipped
    // when the sim token could not be interned, because the read is by token. `run()` fails if
    // nothing was ever verified, so skipping cannot quietly become never.
    if (ok && out_pending && !out_pending->armed && sim && dict
        && path_dictionary_add_path_list_reference(dict, (ovx_primpath_list_t)g->prims.list).status == OVX_API_SUCCESS)
    {
        out_pending->armed = 1;
        out_pending->list = (ovx_primpath_list_t)g->prims.list;
        out_pending->attr = sim;
        out_pending->ordinal = ordinal;
        out_pending->dict = dict;
    }

    // Released either way. A failed write still owns the query.
    ok = ovstage_sample_wait(inst, ovstage_release_query(inst, q)) && ok;
    if (ok)
        ++(*out_written);
    return ok;
}

// app -> physics: nudge every rigid body sideways. Read RIGID_BODY and reuse each
// fixed group's prim_list to author physics:velocity.
//
// Returns the number of groups authored, or -1 if any step failed. Success and failure are
// tracked separately so that a fetch or release failure after a successfully authored group
// is not reported as a healthy control update that the caller then drains.
static int author_rigid_body_velocity(
    ovphysx_handle_t handle, ovstage_instance_t* inst, ovstage_ordinal_t ordinal, const float vel[3])
{
    int authored = 0;
    int failed = 0;
    ovphysx_query_handle_t query = 0;
    const ovphysx_result_t qr = ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query);
    if (qr.status != OVPHYSX_API_SUCCESS)
    {
        // A query error is not "no rigid bodies". Conflating them would let a broken read path
        // pass as an empty scene.
        fprintf(stderr, "ovphysx_query(RIGID_BODY) failed (status %d)\n", (int)qr.status);
        return -1;
    }
    if (query == 0)
        return 0; // genuinely no rigid bodies

    const ovx_string_or_token_t names[] = { name_str(OVPHYSX_ATTR_POSITION) };
    ovphysx_read_handle_t read = 0;
    const ovphysx_result_t rr = ovphysx_read(handle, query, names, 1, &read);
    if (rr.status != OVPHYSX_API_SUCCESS || read == 0)
    {
        fprintf(stderr, "ovphysx_read(RIGID_BODY) failed (status %d)\n", (int)rr.status);
        failed = 1;
    }
    else
    {
        for (;;)
        {
            // Producer-owned: fetch hands back a borrowed ovstage_read_group_t*.
            const ovstage_read_group_t* g = NULL;
            const ovphysx_result_t r = ovphysx_fetch_read_next(handle, read, &g);
            if (r.status == OVPHYSX_API_END_OF_ITERATION)
                break; // exhausted. The only exit that is not a failure
            if (r.status != OVPHYSX_API_SUCCESS)
            {
                fprintf(stderr, "ovphysx_fetch_read_next(RIGID_BODY) failed (status %d)\n", (int)r.status);
                failed = 1;
                break;
            }
            if (!g->is_array) // standalone bodies carry a writable physics:velocity
            {
                // Counts writes that succeeded, not writes attempted. The caller drains the control
                // ordinal based on this count, and draining after a failed write would make the
                // loop look closed when it is not.
                if (author_velocity(inst, g, ordinal, vel))
                    ++authored;
                else
                    failed = 1;
            }
            if (ovphysx_release_group(handle, read, g->read_group_id).status != OVPHYSX_API_SUCCESS)
            {
                fprintf(stderr, "ovphysx_release_group(RIGID_BODY) failed\n");
                failed = 1;
            }
            if (failed)
                break;
        }
        if (ovphysx_release_read(handle, read).status != OVPHYSX_API_SUCCESS)
        {
            fprintf(stderr, "ovphysx_release_read(RIGID_BODY) failed\n");
            failed = 1;
        }
    }
    if (ovphysx_release_query(handle, query).status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "ovphysx_release_query(RIGID_BODY) failed\n");
        failed = 1;
    }
    return failed ? -1 : authored;
}

typedef struct
{
    uint32_t count;
    double sum_x;
    float min_x;
    float max_x;
} vector_stats_t;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static double absolute_double(double value)
{
    return value < 0.0 ? -value : value;
}

// Read one fixed-width rigid-body vector column and summarize its x component.
// The sample runs in CPU mode, so the borrowed DLTensor data is directly readable.
static int read_rigid_body_vector_stats(ovphysx_handle_t handle, const char* attribute, vector_stats_t* stats)
{
    memset(stats, 0, sizeof(*stats));
    stats->min_x = FLT_MAX;
    stats->max_x = -FLT_MAX;

    ovphysx_query_handle_t query = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &query).status
            != OVPHYSX_API_SUCCESS
        || query == 0)
        return 0;

    int valid = 1;
    const ovx_string_or_token_t name = name_str(attribute);
    ovphysx_read_handle_t read = 0;
    if (ovphysx_read(handle, query, &name, 1, &read).status != OVPHYSX_API_SUCCESS || read == 0)
    {
        ovphysx_release_query(handle, query);
        return 0;
    }

    for (;;)
    {
        const ovstage_read_group_t* g = NULL;
        const ovphysx_result_t r = ovphysx_fetch_read_next(handle, read, &g);
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
                || tensor->dtype.bits != 32 || tensor->dtype.lanes < 1 || tensor->ndim != 1 || !tensor->shape)
            {
                valid = 0;
            }
            else
            {
                const unsigned char* base = (const unsigned char*)tensor->data + tensor->byte_offset;
                const int64_t stride = tensor->strides ? tensor->strides[0] : 1;
                const size_t element_bytes = sizeof(float) * tensor->dtype.lanes;
                for (int64_t i = 0; i < tensor->shape[0]; ++i)
                {
                    const float* element = (const float*)(base + (size_t)(i * stride) * element_bytes);
                    const float x = element[0];
                    stats->sum_x += x;
                    stats->min_x = x < stats->min_x ? x : stats->min_x;
                    stats->max_x = x > stats->max_x ? x : stats->max_x;
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
    return valid && stats->count > 0;
}

// physics -> app: read every type in OUTPUT_SPECS and dump each group to ovstage at
// `ordinal`. `tokens` caches the interned "sim:<name>" write-back tokens across
// types and frames. Returns the number of groups written back, or -1 if any step
// failed. The count is incremented on a successful write rather than per fetched
// group, so a frame whose writes all failed cannot report a healthy number.
static int read_and_dump_all(ovphysx_handle_t handle,
                             ovstage_instance_t* inst,
                             sim_token_cache_t* tokens,
                             ovstage_ordinal_t ordinal,
                             pending_readback_t* out_pending)
{
    int dumped = 0;
    for (int s = 0; s < NUM_OUTPUT_SPECS; ++s)
    {
        const output_spec_t* spec = &OUTPUT_SPECS[s];
        ovphysx_query_handle_t query = 0;
        const ovphysx_result_t qr = ovphysx_query(handle, spec->type, OVPHYSX_SCOPE_ALL, &query);
        if (qr.status != OVPHYSX_API_SUCCESS)
        {
            // A query error is not "no objects of this type". Conflating them would let a broken
            // read path pass as an empty scene.
            fprintf(stderr, "ovphysx_query failed for output spec %d (status %d)\n", s, (int)qr.status);
            return -1;
        }
        if (query == 0)
            continue; // genuinely no objects of this type in the scene

        // ovstage's own shared source dictionary, the same one that interned this
        // query's prim lists and attribute tokens and that the ovstage write path
        // interns into. It is not an ovphysx-private dictionary.
        void* dictVoid = NULL;
        ovphysx_query_shared_dictionary(handle, query, &dictVoid);
        path_dictionary_instance_t* dict = (path_dictionary_instance_t*)dictVoid;

        ovx_string_or_token_t attrs[4];
        for (size_t i = 0; i < spec->attr_count; ++i)
            attrs[i] = name_str(spec->attrs[i]);

        ovphysx_read_handle_t read = 0;
        const ovphysx_result_t rr = ovphysx_read(handle, query, attrs, spec->attr_count, &read);
        if (rr.status != OVPHYSX_API_SUCCESS || read == 0)
        {
            fprintf(stderr, "ovphysx_read failed for output spec %d (status %d)\n", s, (int)rr.status);
            ovphysx_release_query(handle, query);
            return -1;
        }

        int failed = 0;
        for (;;)
        {
            // Producer-owned: fetch hands back a borrowed ovstage_read_group_t*.
            const ovstage_read_group_t* g = NULL;
            const ovphysx_result_t r = ovphysx_fetch_read_next(handle, read, &g);
            if (r.status == OVPHYSX_API_END_OF_ITERATION)
                break; // exhausted. The normal exit, not an error
            if (r.status != OVPHYSX_API_SUCCESS)
            {
                fprintf(stderr, "ovphysx_fetch_read_next failed (status %d)\n", (int)r.status);
                failed = 1;
                break;
            }
            if (!dump_group(inst, tokens, dict, g, ordinal, &dumped, out_pending))
                failed = 1;
            if (ovphysx_release_group(handle, read, g->read_group_id).status != OVPHYSX_API_SUCCESS)
            {
                fprintf(stderr, "ovphysx_release_group failed\n");
                failed = 1;
            }
            if (failed)
                break;
        }

        if (ovphysx_release_read(handle, read).status != OVPHYSX_API_SUCCESS)
        {
            fprintf(stderr, "ovphysx_release_read failed\n");
            failed = 1;
        }
        if (ovphysx_release_query(handle, query).status != OVPHYSX_API_SUCCESS)
        {
            fprintf(stderr, "ovphysx_release_query failed\n");
            failed = 1;
        }
        if (failed)
            return -1;
    }
    return dumped;
}

static int run(void)
{
    printf("=== ovphysx Output Read Example (C API) ===\n\n");

    if (ovphysx_initialize().status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "Failed to initialize ovphysx\n");
        return 1;
    }
    // CPU mode keeps the read deterministic, with no GPU warmup step before the read. If the
    // process is already in GPU mode this call fails, and every later read would then need a
    // warmup step the sample never takes, so the failure is not ignored.
    if (ovphysx_set_cpu_mode(true).status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "Failed to enter CPU-only mode\n");
        ovphysx_shutdown();
        return 1;
    }

    ovphysx_create_args create_args = OVPHYSX_CREATE_ARGS_DEFAULT;
    ovphysx_handle_t handle = 0;
    if (ovphysx_create_instance(&create_args, &handle).status != OVPHYSX_API_SUCCESS)
    {
        fprintf(stderr, "Failed to create PhysX instance\n");
        ovphysx_shutdown();
        return 1;
    }

    printf("Loading USD scene through ovstage...\n");
    ovphysx_sample_stage_attachment_t attachment = { 0 };
    if (!ovphysx_sample_attach_usd_with_ovstage(
            handle, OVPHYSX_TEST_DATA "/boxes_falling_on_groundplane.usda", &attachment))
    {
        fprintf(stderr, "ovstage attach/update failed\n");
        ovphysx_destroy_instance(handle);
        ovphysx_shutdown();
        return 1;
    }
    printf("  [OK] USD scene loaded (parsed at ordinal %llu)\n\n", (unsigned long long)attachment.ordinal);

    ovstage_instance_t* stage = attachment.stage;

    // Two interleaved ordinal lanes past the parse ordinal (1). Even ordinals
    // carry control edits and are drained. Odd ordinals carry physics output and
    // are never drained.
    ovstage_ordinal_t control_ordinal = 2;
    ovstage_ordinal_t output_ordinal = 3;
    const float velocity[3] = { 2.0f, 0.0f, 0.0f };
    const float dt = 1.0f / 60.0f;

    // Interned "sim:<name>" write-back tokens, populated lazily on first sight and
    // reused for the rest of the run (see sim_token_for).
    sim_token_cache_t sim_tokens = { 0 };
    // Set once a written-back sim column has been read back out again. `run()` fails if it is
    // still clear at the end, so a run that never reached the verification cannot pass quietly.
    int readback_verified = 0;
    pending_readback_t pending;
    memset(&pending, 0, sizeof(pending));
    int failed = 0;

    for (int frame = 0; frame < 5; ++frame)
    {
        // 1. app -> physics: alternate the sideways velocity so every frame
        //    demonstrates an observable change. Reuse each group's interned
        //    prim_list, seal the control ordinal, then drain it.
        float target_velocity[3] = { frame % 2 == 0 ? velocity[0] : -velocity[0], 0.0f, 0.0f };
        vector_stats_t velocity_before;
        vector_stats_t position_before;
        if (!read_rigid_body_vector_stats(handle, OVPHYSX_ATTR_LINEAR_VELOCITY, &velocity_before)
            || !read_rigid_body_vector_stats(handle, OVPHYSX_ATTR_POSITION, &position_before))
        {
            fprintf(stderr, "Failed to read rigid-body state before frame %d\n", frame);
            failed = 1;
            break;
        }
        // The two results mean different things. -1 is a broken control lane. 0 is a scene with
        // no rigid bodies, which is equally a failure here because this scene has them by
        // construction, but it is not the same diagnosis.
        const int controlAuthored = author_rigid_body_velocity(handle, stage, control_ordinal, target_velocity);
        if (controlAuthored < 0)
        {
            fprintf(stderr, "  frame %d: the velocity control write failed\n", frame);
            failed = 1;
            break;
        }
        if (controlAuthored == 0)
        {
            fprintf(stderr, "No rigid bodies were available for the velocity control write\n");
            failed = 1;
            break;
        }

        advance_floor(stage, control_ordinal);
        ovstage_ordinal_range_t ctrl_range = { control_ordinal, control_ordinal, true };
        if (ovphysx_update_from_ovstage(handle, ctrl_range).status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_string_t err = ovphysx_get_last_error();
            fprintf(stderr, "update_from_ovstage failed: %.*s\n", (int)err.length, err.ptr ? err.ptr : "");
            failed = 1;
            break;
        }

        vector_stats_t velocity_after;
        if (!read_rigid_body_vector_stats(handle, OVPHYSX_ATTR_LINEAR_VELOCITY, &velocity_after)
            || velocity_after.count != velocity_before.count
            || absolute_float(velocity_after.min_x - target_velocity[0]) > 1.0e-5f
            || absolute_float(velocity_after.max_x - target_velocity[0]) > 1.0e-5f)
        {
            fprintf(stderr,
                    "physics:velocity was not applied at frame %d: expected x=%.3f, observed [%.3f, %.3f]\n",
                    frame, target_velocity[0], velocity_after.min_x, velocity_after.max_x);
            failed = 1;
            break;
        }
        if (absolute_double(velocity_after.sum_x - velocity_before.sum_x) <= 1.0e-5)
        {
            fprintf(stderr, "physics:velocity did not change any rigid-body x velocity at frame %d\n", frame);
            failed = 1;
            break;
        }

        // 2. step.
        if (ovphysx_step_sync(handle, dt).status != OVPHYSX_API_SUCCESS)
        {
            fprintf(stderr, "step failed at frame %d\n", frame);
            failed = 1;
            break;
        }

        vector_stats_t position_after;
        const double expected_displacement = (double)target_velocity[0] * dt;
        if (!read_rigid_body_vector_stats(handle, OVPHYSX_ATTR_POSITION, &position_after)
            || position_after.count != position_before.count)
        {
            fprintf(stderr, "Failed to read rigid-body positions after frame %d\n", frame);
            failed = 1;
            break;
        }
        const double mean_displacement =
            (position_after.sum_x - position_before.sum_x) / (double)position_after.count;
        if (absolute_double(mean_displacement - expected_displacement) > 1.0e-4)
        {
            fprintf(stderr, "Unexpected mean x displacement at frame %d: expected %.6f, observed %.6f\n", frame,
                    expected_displacement, mean_displacement);
            failed = 1;
            break;
        }
        printf("  frame %d: velocity x changed %.3f -> %.3f for %u body(s); mean dx=%.6f\n", frame,
               velocity_before.sum_x / velocity_before.count, velocity_after.sum_x / velocity_after.count,
               velocity_after.count, mean_displacement);

        // 3. physics -> app: read every type in OUTPUT_SPECS and dump it to ovstage at
        //    the output ordinal (sealed but never drained).
        const int dumped = read_and_dump_all(handle, stage, &sim_tokens, output_ordinal, &pending);
        if (dumped < 0)
        {
            fprintf(stderr, "  frame %d: output read/write-back failed\n", frame);
            failed = 1;
            break;
        }
        if (!advance_floor(stage, output_ordinal))
        {
            fprintf(stderr, "  frame %d: advancing the output write floor failed\n", frame);
            failed = 1;
            break;
        }

        // The read back happens after the seal. The write sat above the effective write floor
        // until this advance, and ovstage refuses to read above the floor.
        if (pending.armed)
        {
            if (!verify_pending_readback(stage, &pending))
            {
                fprintf(stderr, "  frame %d: the sim column written back could not be read back\n", frame);
                failed = 1;
                break;
            }
            readback_verified = 1;
        }
        // A frame that wrote nothing back is a failure of the closed loop, not a quiet success.
        // The scene has rigid bodies by construction, so zero groups means the read produced
        // nothing or every write-back was rejected.
        if (dumped == 0)
        {
            fprintf(stderr, "  frame %d: no output column groups were written back\n", frame);
            failed = 1;
            break;
        }
        printf("  frame %d: dumped %d output column group(s) to ovstage\n", frame, dumped);

        // 4. advance both lanes. The output ordinal just written is never drained.
        control_ordinal += 2;
        output_ordinal += 2;
    }

    // A frame that broke out early may still hold a pin. Drop it rather than leak the reference
    // into teardown. This is not a verification, `readback_verified` decides that below.
    if (pending.armed && pending.dict)
    {
        path_dictionary_release_path_list_reference(pending.dict, pending.list);
        memset(&pending, 0, sizeof(pending));
    }

    // The closed loop is only closed if a sim column was read back. Without this the sample could
    // still declare success having written values nothing can retrieve.
    if (!failed && !readback_verified)
    {
        fprintf(stderr, "no sim column was ever read back; the closed loop was never verified\n");
        failed = 1;
    }

    // Forced write failure. The write floor was advanced to the last output ordinal, so a write at
    // that ordinal is a floor violation, a real rejection from the public API rather than a test
    // hook. This checks that the failure propagates end to end: write_attribute -> wait ->
    // dump_group -> read_and_dump_all -> run. A non-negative return means a failed frame is being
    // reported as a healthy one.
    if (!failed)
    {
        const ovstage_ordinal_t floored = output_ordinal - 2;
        printf("\nChecking a rejected write is reported (writing at floored ordinal %llu)...\n",
               (unsigned long long)floored);
        if (read_and_dump_all(handle, stage, &sim_tokens, floored, NULL) >= 0)
        {
            fprintf(stderr, "a write below the write floor was NOT reported as a failure\n");
            failed = 1;
        }
        else
        {
            printf("  rejected write correctly reported as a failure\n");
        }
    }

    if (!failed)
        printf("\n=== Output Read Example Completed Successfully ===\n");

    ovphysx_sample_destroy_stage(handle, &attachment);
    ovphysx_destroy_instance(handle);
    ovphysx_shutdown();
    printf("Cleanup complete\n");
    return failed ? 1 : 0;
}

int main(void)
{
    return run();
}
// [tutorial-end]
