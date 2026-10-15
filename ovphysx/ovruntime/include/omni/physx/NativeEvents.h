// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-EVENT-QUEUE-001
// @covers AC-1 AC-2 AC-3 AC-4
#pragma once
#include <cstdint>
#include <memory>
namespace omni::physx::native
{
enum class EventResult : uint32_t { Success, Invalid, Stale, Busy, Unavailable, Error };
enum class EventKind : uint32_t
{
    PreStep=1, SubstepBegin, SubstepEnd, PostStep, SimulationComplete, Contact,
    Trigger, ObjectCreated, ObjectRemoved, ObjectsReset, RuntimeError, TriggerStay
};
// All numeric positions/impulses use scene length/mass units and seconds. Contact
// normal points from collider1 toward collider0, matching the native report.
// generation is the conservative owner object-record epoch, not a pointer. Instance
// indices identify point-instancer instances for contacts and triggers; UINT32_MAX
// means not supplied. Lifecycle objectId is an epoch-qualified native record index.
struct QueuedEvent
{
    uint64_t sequence=0, attachment=0, generation=0, step=0, substep=0, pairSequence=0;
    uint64_t actor0=0, actor1=0, collider0=0, collider1=0, material0=0, material1=0, objectId=0;
    uint32_t kind=0, phase=0, type=0, flags=0;
    uint32_t instance0=UINT32_MAX, instance1=UINT32_MAX, pointIndex=0, pointCount=0;
    uint32_t face0=UINT32_MAX, face1=UINT32_MAX;
    float dt=0, separation=0, position[3]{}, normal[3]{}, impulse[3]{};
    char message[256]{};
};
struct EventBatch
{
    uint64_t attachment=0, dropped=0, completedStep=0;
    uint32_t count=0, overflow=0, ownerLive=0, reserved=0;
};
class EventQueue;
// Mask: 1 pre-step, 2 post-step, 4 substep/status, 8 contacts, 16 triggers,
// 32 object lifecycle, 64 process errors. Capacity 1..65536. An optional absolute
// path filters contacts by either body/collider and triggers by either collider.
// GPU/DirectGPU contact/trigger subscriptions are unavailable on this surface.
// Native contact/trigger reporting must already be authored; open never enables it.
EventResult openEventQueue(uint64_t attachment, uint32_t mask, uint32_t capacity,
                          const char* filterPath, std::shared_ptr<EventQueue>& output) noexcept;
// Poll is post-results delivery: idle owner only. Retired owners may drain copied
// history without dereferencing their objects. FIFO order is observed callback
// enqueue order, not a deterministic ordering of concurrent scene workers.
// Pre/substep phase identifies production, never permits solver reentry from Python.
EventResult pollEventQueue(const std::shared_ptr<EventQueue>& queue, QueuedEvent* events,
                          uint32_t capacity, EventBatch& output) noexcept;
// Cancel and discard. Dispatcher snapshots hold weak queue references; an admitted
// callback takes a temporary strong reference and cannot enqueue after close's
// locked barrier, even during owner retirement. The native caller must retain the
// runtime through close; the companion bridge manages this lease automatically.
EventResult closeEventQueue(const std::shared_ptr<EventQueue>& queue) noexcept;
}
