// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// A step whose simulation blocks inside PhysX until the test opens a gate: an
// onAdvance callback on a pose-integration-preview actor waits for the gate, so the
// step's task stays unfinished while the test observes it. Install the scope while
// the instance runs no work; a watchdog opens the gate if the test never does.

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <PxRigidDynamic.h>
#include <PxScene.h>
#include <PxSimulationEventCallback.h>
#include "ovphysx/ovphysx.h"

namespace test_utils {

struct AdvanceGate
{
    std::mutex mutex;
    std::condition_variable condition;
    bool entered = false;
    bool open = false;
    bool watchdog_fired = false;
};

class GatedSimulationCallback final : public physx::PxSimulationEventCallback
{
public:
    GatedSimulationCallback(
        physx::PxSimulationEventCallback* original,
        std::shared_ptr<AdvanceGate> gate)
        : m_original(original), m_gate(std::move(gate))
    {
    }

    void onConstraintBreak(physx::PxConstraintInfo* constraints, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onConstraintBreak(constraints, count);
    }

    void onWake(physx::PxActor** actors, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onWake(actors, count);
    }

    void onSleep(physx::PxActor** actors, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onSleep(actors, count);
    }

    void onContact(
        const physx::PxContactPairHeader& pair_header,
        const physx::PxContactPair* pairs,
        physx::PxU32 pair_count) override
    {
        if (m_original != nullptr)
            m_original->onContact(pair_header, pairs, pair_count);
    }

    void onTrigger(physx::PxTriggerPair* pairs, physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onTrigger(pairs, count);
    }

    void onAdvance(
        const physx::PxRigidBody* const* bodies,
        const physx::PxTransform* poses,
        physx::PxU32 count) override
    {
        if (m_original != nullptr)
            m_original->onAdvance(bodies, poses, count);

        std::unique_lock<std::mutex> lock(m_gate->mutex);
        m_gate->entered = true;
        m_gate->condition.notify_all();
        m_gate->condition.wait(lock, [this] { return m_gate->open; });
    }

private:
    physx::PxSimulationEventCallback* m_original;
    std::shared_ptr<AdvanceGate> m_gate;
};

inline void open_advance_gate(const std::shared_ptr<AdvanceGate>& gate)
{
    {
        std::lock_guard<std::mutex> lock(gate->mutex);
        gate->open = true;
    }
    gate->condition.notify_all();
}

inline bool wait_for_advance_gate(const std::shared_ptr<AdvanceGate>& gate)
{
    std::unique_lock<std::mutex> lock(gate->mutex);
    return gate->condition.wait_for(
        lock,
        std::chrono::seconds(5),
        [gate] { return gate->entered; });
}

class GatedSimulationScope final
{
public:
    GatedSimulationScope(
        ovphysx_instance_handle_t handle,
        physx::PxScene* scene,
        physx::PxRigidDynamic* actor)
        : m_handle(handle),
          m_scene(scene),
          m_actor(actor),
          m_originalCallback(scene->getSimulationEventCallback()),
          m_previewWasSet(actor->getRigidBodyFlags().isSet(
              physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW)),
          m_gate(std::make_shared<AdvanceGate>()),
          m_callback(m_originalCallback, m_gate)
    {
        m_actor->setRigidBodyFlag(
            physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW,
            true);
        m_actor->wakeUp();
        m_scene->setSimulationEventCallback(&m_callback);
    }

    ~GatedSimulationScope()
    {
        finish();
    }

    GatedSimulationScope(const GatedSimulationScope&) = delete;
    GatedSimulationScope& operator=(const GatedSimulationScope&) = delete;

    void set_pending_task(ovphysx_enqueue_result_t task)
    {
        m_pendingTask = task;
    }

    bool wait_until_entered()
    {
        return wait_for_advance_gate(m_gate);
    }

    void start_watchdog()
    {
        std::shared_ptr<AdvanceGate> gate = m_gate;
        m_watchdog = std::thread([gate] {
            std::unique_lock<std::mutex> lock(gate->mutex);
            if (!gate->condition.wait_for(
                    lock,
                    std::chrono::seconds(5),
                    [gate] { return gate->open; }))
            {
                gate->watchdog_fired = true;
                gate->open = true;
                lock.unlock();
                gate->condition.notify_all();
            }
        });
    }

    bool open_and_join_watchdog() noexcept
    {
        open_advance_gate(m_gate);
        if (m_watchdog.joinable())
            m_watchdog.join();

        std::lock_guard<std::mutex> lock(m_gate->mutex);
        return m_gate->watchdog_fired;
    }

    ovphysx_api_status_t finish() noexcept
    {
        if (m_restored)
            return m_finalStatus;

        open_and_join_watchdog();

        if (m_pendingTask.op_index != 0)
        {
            ovphysx_wait_task_result_t wait_result{};
            const ovphysx_result_t result = ovphysx_wait_task(
                m_handle,
                m_pendingTask,
                OVPHYSX_TIMEOUT_INFINITE,
                &wait_result);
            ovphysx_destroy_wait_task_result(&wait_result);
            m_finalStatus = result.status;
            m_pendingTask = {};
        }

        m_scene->setSimulationEventCallback(m_originalCallback);
        m_actor->setRigidBodyFlag(
            physx::PxRigidBodyFlag::eENABLE_POSE_INTEGRATION_PREVIEW,
            m_previewWasSet);
        m_restored = true;
        return m_finalStatus;
    }

private:
    ovphysx_instance_handle_t m_handle;
    physx::PxScene* m_scene;
    physx::PxRigidDynamic* m_actor;
    physx::PxSimulationEventCallback* m_originalCallback;
    bool m_previewWasSet;
    std::shared_ptr<AdvanceGate> m_gate;
    GatedSimulationCallback m_callback;
    std::thread m_watchdog;
    ovphysx_enqueue_result_t m_pendingTask{};
    ovphysx_api_status_t m_finalStatus = OVPHYSX_API_SUCCESS;
    bool m_restored = false;
};

} // namespace test_utils
