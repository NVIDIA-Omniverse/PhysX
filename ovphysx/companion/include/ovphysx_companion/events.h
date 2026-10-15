// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-EVENTS-001
// @covers AC-1 AC-2 AC-3 AC-4
#ifndef OVPHYSX_COMPANION_EVENTS_H
#define OVPHYSX_COMPANION_EVENTS_H
#include "companion.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef enum ovphysx_companion_event_kind {
    OVPHYSX_COMPANION_EVENT_PRE_STEP=1, OVPHYSX_COMPANION_EVENT_SUBSTEP_BEGIN=2,
    OVPHYSX_COMPANION_EVENT_SUBSTEP_END=3, OVPHYSX_COMPANION_EVENT_POST_STEP=4,
    OVPHYSX_COMPANION_EVENT_COMPLETE=5, OVPHYSX_COMPANION_EVENT_CONTACT=6,
    OVPHYSX_COMPANION_EVENT_TRIGGER=7, OVPHYSX_COMPANION_EVENT_OBJECT_CREATED=8,
    OVPHYSX_COMPANION_EVENT_OBJECT_REMOVED=9, OVPHYSX_COMPANION_EVENT_RESET=10,
    OVPHYSX_COMPANION_EVENT_ERROR=11, OVPHYSX_COMPANION_EVENT_TRIGGER_STAY=12
} ovphysx_companion_event_kind_t;
typedef enum ovphysx_companion_event_mask {
    OVPHYSX_COMPANION_EVENTS_PRE_STEP=1, OVPHYSX_COMPANION_EVENTS_POST_STEP=2,
    OVPHYSX_COMPANION_EVENTS_SUBSTEPS=4, OVPHYSX_COMPANION_EVENTS_CONTACTS=8,
    OVPHYSX_COMPANION_EVENTS_TRIGGERS=16, OVPHYSX_COMPANION_EVENTS_LIFECYCLE=32,
    OVPHYSX_COMPANION_EVENTS_ERRORS=64
} ovphysx_companion_event_mask_t;
typedef enum ovphysx_companion_contact_state {
    OVPHYSX_COMPANION_CONTACT_FOUND=0, OVPHYSX_COMPANION_CONTACT_LOST=1,
    OVPHYSX_COMPANION_CONTACT_PERSISTS=2
} ovphysx_companion_contact_state_t;
typedef enum ovphysx_companion_trigger_state {
    OVPHYSX_COMPANION_TRIGGER_ENTER=0, OVPHYSX_COMPANION_TRIGGER_LEAVE=1
} ovphysx_companion_trigger_state_t;
typedef enum ovphysx_companion_error_category {
    OVPHYSX_COMPANION_ERROR_USD_LOAD=0, OVPHYSX_COMPANION_ERROR_PHYSX=1,
    OVPHYSX_COMPANION_ERROR_CUDA=2, OVPHYSX_COMPANION_ERROR_LIMIT=3
} ovphysx_companion_error_category_t;
#define OVPHYSX_COMPANION_EVENT_HISTORY_LOST 8u
/** Copied event. kind: 1 pre-step, 2/3 substep begin/end, 4 post-step,
 * 5 simulation complete, 6 contact, 7 trigger enter/leave, 8/9 object created/removed,
 * 10 all objects reset, 11 typed process error, 12 derived trigger stay.
 * phase: 0 pre-step, 1 substep, 2 post-step, 3 source/lifecycle, 4 process.
 * type: contact 0 found/1 lost/2 persists; trigger 0 enter/1 leave;
 * object native PhysXType; error 0 USD load/1 PhysX/2 CUDA/3 too many errors.
 * generation is the conservative owner record epoch. UINT32_MAX instance/face
 * means unspecified. No native pointer escapes. Contact and trigger indices identify
 * point-instancer instances; distinct instances retain distinct trigger stay state.
 * Lifecycle object_id is the native record index qualified by attachment and
 * conservative epoch; it is not a pointer and must never be dereferenced.
 * Coordinates are world stage units, seconds and stage mass units. Contact normal
 * follows the native collider1-to-collider0 convention. point_count/index group
 * records by pair_sequence. Lost/empty reports still carry a pair record.
 * Errors have attachment=0 (process scope), not an invented per-owner origin;
 * message is UTF-8 bytes bounded to 255 bytes, flags bit0 signals truncation.
 */
typedef struct ovphysx_companion_event {
    uint64_t sequence,attachment,generation,step,substep,pair_sequence;
    uint64_t actor0,actor1,collider0,collider1,material0,material1,object_id;
    uint32_t kind,phase,type,flags;
    uint32_t instance0,instance1,point_index,point_count;
    uint32_t face0,face1;
    float dt,separation,position[3],normal[3],impulse[3];
    char message[256];
} ovphysx_companion_event_t;
typedef struct ovphysx_companion_event_batch {
    uint64_t attachment,dropped,completed_step;
    uint32_t count,overflow,owner_live,reserved;
} ovphysx_companion_event_batch_t;
/** Open mask: 1 pre-step,2 post-step,4 substeps/status,8 contacts,16 triggers,
 * 32 lifecycle,64 process errors. Capacity 1..65536. Optional absolute filter path
 * matches either contact body/collider or either trigger collider. Reporting APIs
 * must be authored already. GPU/DirectGPU contact/trigger subscriptions are unavailable. Subscribe before stepping to observe initial enters.
 * Poll only after owner completion; it never fetches outstanding tasks. Returned
 * phase describes native production; all Python/host delivery is post-results.
 * FIFO ordering reflects callback admission, not deterministic parallel scene order.
 * Trigger stay derives only from enters observed by this subscription. Reset and
 * object deletion discard known pairs. A late subscriber cannot reconstruct prior
 * overlaps. Start before the first step or explicitly rebuild/reset the scene.
 * Poll returns EVENT_HISTORY_LOST with a valid diagnostic batch on overflow;
 * other failures preserve output. This status never means a recoverable history.
 * Overflow is sticky and stops further enqueue, counts lost records, and invalidates
 * pair caches. Retained records are diagnostic only. Close, reset/rebuild the owning
 * scene and resubscribe before stepping to recover complete enter/leave history.
 * Close discards queued data and prevents further delivery, including callbacks
 * already admitted by native dispatch. It does not reset/step/edit the owner.
 */
typedef struct ovphysx_companion_events_api {
    uint32_t size,version;
    uint64_t capabilities;
    ovphysx_companion_status_t (*open)(uint64_t attachment,uint32_t mask,uint32_t capacity,
                                     const char* filter_path,uint64_t* queue);
    ovphysx_companion_status_t (*poll)(uint64_t queue,uint32_t event_size,ovphysx_companion_event_t* events,
                                     uint32_t capacity,uint32_t batch_size,ovphysx_companion_event_batch_t* batch);
    ovphysx_companion_status_t (*close)(uint64_t queue);
} ovphysx_companion_events_api_t;
OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_events_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_events_api_t* output);
#ifdef __cplusplus
}
#endif
#endif
