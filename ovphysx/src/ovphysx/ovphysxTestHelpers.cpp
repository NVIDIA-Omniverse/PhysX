// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-EXECUTOR-001
 * @covers AC-8
 *
 * @implements REQ-CAPI-ASYNC-002
 * @covers AC-16
 */
// @implements REQ-CAPI-OMNIPVD-CAPTURE-001
// @covers AC-6

#include "ovphysxTestHelpers.h"
#include "CarboniteLoader/CarboniteLoader.hpp"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/ovphysx_config.h"

#include <omni/physx/IPhysx.h>
#include <omni/physx/PhysXRuntime.h>
#include <omni/physx/RuntimeError.h>

#include <carb/Framework.h>
#include <carb/settings/ISettings.h>
#include <carb/tasking/ITasking.h>
#include <carb/tasking/TaskingUtils.h>
#include <carb/thread/Util.h>
#include <omni/physx/IOptionalCuda.h>
#include <omni/physics/tensors/ISimulationView.h>
#include <PxPhysicsAPI.h>

#include <array>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <vector>

#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sidecar/ovphysxInternalInterop.h"


namespace {

struct TaskGuardTestState
{
    carb::tasking::MutexWrapper* gate = nullptr;
    carb::tasking::ConditionVariableWrapper* cv = nullptr;
    std::atomic<bool>* holding = nullptr;
    std::atomic<bool>* release = nullptr;
};

TaskGuardTestState g_taskGuardTest;

struct PhysXExtensionsFixture
{
    physx::PxCustomGeometryExt::CylinderCallbacks cylinder{ 1.0f, 0.25f, 1, 0.01f };
    physx::PxCustomGeometryExt::ConeCallbacks cone{ 1.0f, 0.25f, 1, 0.01f };
    physx::PxMaterial* material{ nullptr };
    std::array<physx::PxRigidDynamic*, 2> actors{};
    std::array<physx::PxShape*, 2> shapes{};
    std::array<physx::PxJoint*, 8> joints{};

    bool create(physx::PxPhysics& physics)
    {
        using namespace physx;
        material = physics.createMaterial(0.5f, 0.5f, 0.0f);
        actors = {
            physics.createRigidDynamic(PxTransform(PxVec3(-0.5f, 0.0f, 0.0f))),
            physics.createRigidDynamic(PxTransform(PxVec3(0.5f, 0.0f, 0.0f))),
        };
        if (!material || !actors[0] || !actors[1])
            return false;

        shapes = {
            physics.createShape(PxCustomGeometry(cylinder), *material),
            physics.createShape(PxCustomGeometry(cone), *material),
        };
        if (!shapes[0] || !shapes[1])
            return false;

        const PxTransform frame(PxIdentity);
        joints = {
            PxFixedJointCreate(physics, actors[0], frame, actors[1], frame),
            PxPrismaticJointCreate(physics, actors[0], frame, actors[1], frame),
            PxRevoluteJointCreate(physics, actors[0], frame, actors[1], frame),
            PxSphericalJointCreate(physics, actors[0], frame, actors[1], frame),
            PxDistanceJointCreate(physics, actors[0], frame, actors[1], frame),
            PxD6JointCreate(physics, actors[0], frame, actors[1], frame),
            PxGearJointCreate(physics, actors[0], frame, actors[1], frame),
            PxRackAndPinionJointCreate(physics, actors[0], frame, actors[1], frame),
        };
        for (PxJoint* joint : joints)
        {
            if (!joint)
                return false;
        }

        auto* prismatic = static_cast<PxPrismaticJoint*>(joints[1]);
        auto* revolute = static_cast<PxRevoluteJoint*>(joints[2]);
        auto* d6 = static_cast<PxD6Joint*>(joints[5]);
        auto* gear = static_cast<PxGearJoint*>(joints[6]);
        auto* rackAndPinion = static_cast<PxRackAndPinionJoint*>(joints[7]);
        return gear->setHinges(revolute, d6) && rackAndPinion->setJoints(revolute, prismatic);
    }

    ~PhysXExtensionsFixture()
    {
        for (auto it = joints.rbegin(); it != joints.rend(); ++it)
        {
            if (*it)
                (*it)->release();
        }
        for (physx::PxShape* shape : shapes)
        {
            if (shape)
                shape->release();
        }
        for (physx::PxRigidDynamic* actor : actors)
        {
            if (actor)
                actor->release();
        }
        if (material)
            material->release();
    }
};

} // namespace

extern "C" {

OVPHYSX_API bool ovphysx_get_tensor_binding_cuda_context_internal(
    ovphysx_instance_handle_t handle,
    ovphysx_tensor_binding_handle_t binding,
    uintptr_t* out_cuda_ctx)
{
    if (!out_cuda_ctx) return false;
    *out_cuda_ctx = 0;

    auto instance = get_instance(handle);
    if (!instance) return false;

    // NOTE: tensor_bindings is guarded by InstanceData::tensor_binding_mutex.
    std::lock_guard<std::mutex> lock(instance->tensor_binding_mutex);
    const auto it = instance->tensor_bindings.find(binding);
    if (it == instance->tensor_bindings.end()) return false;

    const auto& b = it->second;
    if (!b.simView) return true;  // binding exists but has no simView (ctx remains 0)

    *out_cuda_ctx = reinterpret_cast<uintptr_t>(b.simView->getCudaContext());
    return true;
}

OVPHYSX_API void* ovphysx_get_optional_cuda_internal(void)
{
    return static_cast<void*>(omni::physx::runtime::tryGetOptionalCudaInterface());
}

OVPHYSX_API void* ovphysx_get_tensor_api_internal(void)
{
    return static_cast<void*>(omni::physx::runtime::tryGetTensorApiInterface());
}

OVPHYSX_API bool ovphysx_get_attach_cuda_selector_for_test_internal(int32_t* out_value)
{
    if (!out_value)
        return false;

    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    constexpr const char* cudaDevicePath = "/physics/cudaDevice";
    if (!settings || settings->getItemType(cudaDevicePath) != carb::dictionary::ItemType::eInt)
        return false;

    *out_value = settings->getAsInt(cudaDevicePath);
    return true;
}

OVPHYSX_API bool ovphysx_set_raw_string_setting_for_test_internal(
    const char* path,
    const char* value,
    size_t length)
{
    if (!path || (!value && length != 0))
        return false;
    // Seed a host setting before PhysX runtime startup, including after full SDK shutdown.
    ovphysx::CarboniteLoader loader;
    if (!loader.initialize())
        return false;
    carb::Framework* framework = carb::getFramework();
    carb::settings::ISettings* settings =
        framework ? framework->tryAcquireInterface<carb::settings::ISettings>() : nullptr;
    if (!settings)
        return false;
    settings->setString(path, value, length);
    return true;
}

OVPHYSX_API ovphysx_test_set_viz_scope_tokens_fn
ovphysx_exchange_set_viz_scope_tokens_internal(ovphysx_test_set_viz_scope_tokens_fn replacement)
{
    return g_sidecarSetVizScopeTokens.exchange(replacement, std::memory_order_acq_rel);
}

OVPHYSX_API bool ovphysx_set_ovstage_attachment_state_internal(
    ovphysx_instance_handle_t handle,
    bool attached,
    int64_t stage_id)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return false;
    instance->ovstage_attached = attached;
    instance->attachedStageId = stage_id;
    // Attach identity is separate from the stage id (ADR-0013) and is what the
    // binding staleness checks compare, so a faked attach has to set it too.
    // Any nonzero value works here: the runtime never resolves this one.
    instance->attachHandle =
        attached ? omni::physics::tensors::AttachHandle(1) : omni::physics::tensors::kNoAttach;
    return true;
}

OVPHYSX_API bool ovphysx_create_extensions_fixture_for_test_internal(
    ovphysx_instance_handle_t handle,
    void** out_fixture)
{
    if (!out_fixture)
        return false;
    *out_fixture = nullptr;

    void* physicsPtr = nullptr;
    ovphysx_query_handle_t query = 0;
    ovphysx_enqueue_result_t opened =
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_PHYSICS, OVPHYSX_SCOPE_ALL, &query, nullptr);
    auto waitOk = [&](ovphysx_enqueue_result_t task) {
        ovphysx_wait_task_result_t waited{};
        const bool ok = task.status == OVPHYSX_API_SUCCESS &&
                        ovphysx_wait_task(handle, task, OVPHYSX_TIMEOUT_INFINITE, &waited).status ==
                            OVPHYSX_API_SUCCESS &&
                        waited.num_errors == 0;
        ovphysx_destroy_wait_task_result(&waited);
        return ok;
    };
    if (!waitOk(opened))
        return false;

    ovx_token_or_string_t name{};
    name.string.ptr = OVPHYSX_ATTR_PHYSX_POINTER;
    name.string.length = sizeof(OVPHYSX_ATTR_PHYSX_POINTER) - 1;
    ovphysx_read_handle_t read = 0;
    if (waitOk(ovphysx_read(handle, query, &name, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)))
    {
        const ovstage_read_group_t* group = nullptr;
        ovphysx_enqueue_result_t fetched = ovphysx_fetch_read_next(handle, read, &group);
        if (waitOk(fetched) && group)
        {
            const ovstage_data_t& data = group->columns[0].data;
            if (data.tensors && data.tensors[0].data)
                physicsPtr = reinterpret_cast<void*>(static_cast<const uint64_t*>(data.tensors[0].data)[0]);
            waitOk(ovphysx_release_group(handle, read, group->read_group_id));
        }
        waitOk(ovphysx_release_read(handle, read));
    }
    waitOk(ovphysx_release_query(handle, query));
    if (!physicsPtr)
        return false;

    PhysXExtensionsFixture* fixture = new (std::nothrow) PhysXExtensionsFixture;
    if (!fixture || !fixture->create(*static_cast<physx::PxPhysics*>(physicsPtr)))
    {
        delete fixture;
        return false;
    }
    *out_fixture = fixture;
    return true;
}

OVPHYSX_API void ovphysx_destroy_extensions_fixture_for_test_internal(void* fixture)
{
    delete static_cast<PhysXExtensionsFixture*>(fixture);
}

OVPHYSX_API void* ovphysx_create_d6_joint_at_path_for_test_internal(
    ovphysx_instance_handle_t handle, const char* joint_path, const char* body0_path, const char* body1_path)
{
    const std::shared_ptr<InstanceData> instance = get_instance(handle);
    omni::physx::IPhysx* runtime = omni::physx::runtime::tryGetPhysxInterface();
    if (!instance || !runtime || !runtime->createD6JointAtPath || !runtime->resolveObjectKey)
        return nullptr;
    const auto body0 = runtime->resolveObjectKey(instance->attachHandle, body0_path);
    const auto body1 = runtime->resolveObjectKey(instance->attachHandle, body1_path);
    return const_cast<void*>(runtime->createD6JointAtPath(instance->attachHandle, joint_path, body0, body1));
}

OVPHYSX_API bool ovphysx_get_worker_pool_counts_for_test_internal(
    uint32_t* out_tasking_threads,
    int32_t* out_tasking_setting,
    int32_t* out_tbb_setting,
    uint32_t* out_hardware_concurrency)
{
    if (!out_tasking_threads || !out_tasking_setting || !out_tbb_setting || !out_hardware_concurrency)
        return false;

    carb::Framework* framework = carb::getFramework();
    if (!framework)
        return false;

    // tryAcquireExistingInterface does not start the plugin, so this reports the
    // pool as it is rather than creating one the product did not ask for.
    carb::tasking::ITasking* tasking = framework->tryAcquireExistingInterface<carb::tasking::ITasking>();
    carb::settings::ISettings* settings = framework->tryAcquireInterface<carb::settings::ISettings>();
    if (!tasking || !settings)
        return false;

    *out_tasking_threads = tasking->getDesc().threadCount;
    *out_tasking_setting = settings->getAsInt("/plugins/carb.tasking.plugin/threadCount");
    *out_tbb_setting = settings->getAsInt("/plugins/omni.tbb.globalcontrol/maxThreadCount");
    *out_hardware_concurrency = carb::thread::hardware_concurrency();
    return true;
}

OVPHYSX_API void ovphysx_wait_on_task_guard_for_test_internal(void)
{
    carb::tasking::MutexWrapper& gate = *g_taskGuardTest.gate;
    gate.lock();
    g_taskGuardTest.holding->store(true, std::memory_order_release);
    g_taskGuardTest.cv->wait(gate, []() {
        return g_taskGuardTest.release->load(std::memory_order_acquire);
    });
    gate.unlock();
}

OVPHYSX_API bool ovphysx_with_suspended_carbonite_task_for_test_internal(
    ovphysx_test_task_fn body, ovphysx_test_task_fn probe, void* user)
{
    if (body == nullptr || probe == nullptr)
        return false;

    carb::Framework* framework = carb::getFramework();
    if (framework == nullptr)
        return false;
    carb::tasking::ITasking* tasking = framework->tryAcquireExistingInterface<carb::tasking::ITasking>();
    if (tasking == nullptr)
        return false;

    carb::tasking::MutexWrapper gate;
    carb::tasking::ConditionVariableWrapper cv;
    std::atomic<bool> holding{false};
    std::atomic<bool> release{false};
    g_taskGuardTest.gate = &gate;
    g_taskGuardTest.cv = &cv;
    g_taskGuardTest.holding = &holding;
    g_taskGuardTest.release = &release;
    struct ClearGuard
    {
        ~ClearGuard() { g_taskGuardTest = {}; }
    } clear;

    std::vector<carb::tasking::Future<void>> tasks;
    struct JoinTasks
    {
        std::vector<carb::tasking::Future<void>>& tasks;
        ~JoinTasks()
        {
            for (carb::tasking::Future<void>& task : tasks)
            {
                if (task.valid())
                    task.wait();
            }
        }
    } join{tasks};

    tasks.push_back(tasking->addTask(carb::tasking::Priority::eDefault, {}, [body, user]() { body(user); }));

    const std::chrono::steady_clock::time_point holdingDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (!holding.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < holdingDeadline)
        std::this_thread::yield();
    if (!holding.load(std::memory_order_acquire))
    {
        release.store(true, std::memory_order_release);
        cv.notify_all();
        return false;
    }

    // Acquiring the mutex blocks until the body task is inside the condition
    // wait, which has released the mutex and suspended that task.
    gate.lock();
    struct UnlockGate
    {
        carb::tasking::MutexWrapper& gate;
        bool locked = true;
        ~UnlockGate()
        {
            if (locked)
                gate.unlock();
        }
    } unlock{gate};

    const unsigned dispatchCount = 2 * tasking->getDesc().threadCount + 8;
    for (unsigned index = 0; index < dispatchCount; ++index)
    {
        tasks.push_back(tasking->addTask(
            carb::tasking::Priority::eDefault, {}, [probe, user]() { probe(user); }));
    }

    bool probesFinished = true;
    for (unsigned index = 1; index < tasks.size(); ++index)
        probesFinished = tasks[index].wait_for(std::chrono::seconds(10)) && probesFinished;

    release.store(true, std::memory_order_release);
    cv.notify_all();
    unlock.locked = false;
    gate.unlock();

    const bool bodyFinished = tasks[0].wait_for(std::chrono::seconds(10));
    return probesFinished && bodyFinished;
}

OVPHYSX_API bool ovphysx_set_worker_hold_for_test_internal(ovphysx_instance_handle_t handle, bool hold)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !instance->executor)
        return false;
    instance->executor->setHold(hold);
    return true;
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_submit_test_job_for_test_internal(
    ovphysx_instance_handle_t handle,
    const char* failure,
    uint32_t sleep_ms)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance)
        return {OVPHYSX_API_NOT_FOUND, 0, 0};
    std::string cause;
    const bool fails = failure != nullptr;
    if (fails)
        cause = failure;
    InstanceData& owner = *instance;
    std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
        "test_job", handle, std::move(instance),
        [cause, fails, sleep_ms](ovphysx::InstanceJob& self) {
            if (sleep_ms)
                std::this_thread::sleep_for(std::chrono::milliseconds(sleep_ms));
            if (fails)
                self.fail(cause);
        }));
    return ovphysx::submitInstanceJob(owner, handle, std::move(job));
}

OVPHYSX_API ovphysx_enqueue_result_t ovphysx_submit_test_callback_job_for_test_internal(
    ovphysx_instance_handle_t handle,
    ovphysx_test_task_fn fn,
    void* user)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !fn)
        return {OVPHYSX_API_NOT_FOUND, 0, 0};
    InstanceData& owner = *instance;
    std::unique_ptr<ovphysx::InstanceJob> job(new ovphysx::InstanceJob(
        "test_callback_job", handle, std::move(instance), [fn, user](ovphysx::InstanceJob&) { fn(user); }));
    return ovphysx::submitInstanceJob(owner, handle, std::move(job));
}

OVPHYSX_API uint64_t ovphysx_subscribe_step_event_for_test_internal(
    ovphysx_instance_handle_t handle,
    bool pre_step,
    ovphysx_test_step_event_fn fn,
    void* user)
{
    const std::shared_ptr<InstanceData> instance = get_instance(handle);
    omni::physx::IPhysx* physx = omni::physx::runtime::tryGetPhysxInterface();
    if (!instance || !physx || !fn || instance->attachHandle == omni::physics::kNoAttach)
        return 0;
    const omni::physx::SubscriptionId subscription =
        physx->subscribePhysicsOnStepEvents(instance->attachHandle, pre_step, 0, fn, user);
    return subscription == omni::physx::kInvalidSubscriptionId ? 0 : static_cast<uint64_t>(subscription) + 1;
}

OVPHYSX_API bool ovphysx_unsubscribe_step_event_for_test_internal(uint64_t subscription)
{
    omni::physx::IPhysx* physx = omni::physx::runtime::tryGetPhysxInterface();
    if (!physx || subscription == 0)
        return false;
    physx->unsubscribePhysicsOnStepEvents(static_cast<omni::physx::SubscriptionId>(subscription - 1));
    return true;
}

OVPHYSX_API void ovphysx_record_runtime_error_for_test_internal(const char* message)
{
    omni::physx::recordRuntimeError(message);
}

OVPHYSX_API bool ovphysx_hold_worker_after_next_submit_for_test_internal(ovphysx_instance_handle_t handle)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !instance->executor)
        return false;
    instance->executor->holdAfterNextSubmit();
    return true;
}

OVPHYSX_API bool ovphysx_worker_exited_for_test_internal(ovphysx_instance_handle_t handle, bool* out_exited)
{
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !instance->executor || !out_exited)
        return false;
    *out_exited = instance->executor->workerExited();
    return true;
}

OVPHYSX_API void ovphysx_with_lifecycle_lock_for_test_internal(ovphysx_test_task_fn fn, void* user)
{
    if (fn)
        ovphysx_run_under_lifecycle_lock(fn, user);
}

OVPHYSX_API bool ovphysx_lane_waits_and_destroy_from_saturated_carbonite_pool_for_test_internal(
    ovphysx_instance_handle_t handle,
    ovphysx_instance_handle_t victim,
    uint32_t* out_waiters)
{
    if (out_waiters)
        *out_waiters = 0;
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !instance->executor)
        return false;
    {
        std::shared_ptr<InstanceData> doomed = get_instance(victim);
        if (!doomed || !doomed->executor)
            return false;
        doomed->executor->setHold(true);
    }
    if (ovphysx_submit_test_job_for_test_internal(victim, nullptr, 0).status != OVPHYSX_API_SUCCESS)
        return false;
    carb::Framework* framework = carb::getFramework();
    carb::tasking::ITasking* tasking =
        framework ? framework->tryAcquireExistingInterface<carb::tasking::ITasking>() : nullptr;
    if (!tasking)
        return false;

    const uint32_t waiterCount = 2 * tasking->getDesc().threadCount + 2;
    struct Shared
    {
        std::unique_ptr<std::atomic<int>[]> statuses;
        std::atomic<int> destroyStatus{-1};
    };
    // Tasks capture the shared state, so an early return stays memory safe.
    std::shared_ptr<Shared> shared = std::make_shared<Shared>();
    shared->statuses.reset(new std::atomic<int>[waiterCount]);
    for (uint32_t index = 0; index < waiterCount; ++index)
        shared->statuses[index].store(-1);

    // A held worker with a queued job keeps the lane busy until the hold is released.
    instance->executor->setHold(true);
    struct ReleaseHold
    {
        ovphysx::InstanceExecutor& executor;
        ~ReleaseHold() { executor.setHold(false); }
    } releaseHold{*instance->executor};
    const ovphysx_enqueue_result_t held = ovphysx_submit_test_job_for_test_internal(handle, nullptr, 0);
    if (held.status != OVPHYSX_API_SUCCESS)
        return false;

    std::vector<carb::tasking::Future<void>> waiters;
    waiters.reserve(waiterCount);
    for (uint32_t index = 0; index < waiterCount; ++index)
    {
        waiters.push_back(tasking->addTask(carb::tasking::Priority::eDefault, {}, [handle, shared, index]() {
            const ovphysx_result_t result = ovphysx_set_config(
                handle, OVPHYSX_CONFIG_ACTIVE_CUDA_GPUS, ovphysx_config_value_string(OVPHYSX_LITERAL("")));
            shared->statuses[index].store(static_cast<int>(result.status));
        }));
    }

    // Every waiter is suspended as a task inside the executor; one that blocked its OS
    // thread would never be counted.
    const std::chrono::steady_clock::time_point suspendDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (instance->executor->waitingTaskCount() < waiterCount &&
           std::chrono::steady_clock::now() < suspendDeadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool allSuspended = instance->executor->waitingTaskCount() == waiterCount &&
                              instance->executor->waitingThreadCount() == 0;

    // With more waiters than workers, the destroying task runs only if waiting tasks
    // released their threads, and it must finish while they still wait.
    carb::tasking::Future<void> destroyer = tasking->addTask(carb::tasking::Priority::eDefault, {}, [victim, shared]() {
        shared->destroyStatus.store(static_cast<int>(ovphysx_destroy_instance(victim).status));
    });
    const bool destroyed = destroyer.wait_for(std::chrono::seconds(10)) &&
                           shared->destroyStatus.load() == static_cast<int>(OVPHYSX_API_SUCCESS);
    bool stillWaiting = true;
    for (uint32_t index = 0; index < waiterCount; ++index)
        stillWaiting = stillWaiting && shared->statuses[index].load() == -1;

    instance->executor->setHold(false);
    bool waitersFinished = true;
    for (carb::tasking::Future<void>& waiter : waiters)
        waitersFinished = waiter.wait_for(std::chrono::seconds(30)) && waitersFinished;
    bool allSucceeded = waitersFinished;
    for (uint32_t index = 0; allSucceeded && index < waiterCount; ++index)
        allSucceeded = shared->statuses[index].load() == static_cast<int>(OVPHYSX_API_SUCCESS);
    ovphysx_wait_task_result_t collected{};
    const bool heldJobSucceeded =
        ovphysx_wait_task(handle, held, OVPHYSX_TIMEOUT_INFINITE, &collected).status == OVPHYSX_API_SUCCESS;
    ovphysx_destroy_wait_task_result(&collected);

    if (out_waiters)
        *out_waiters = waiterCount;
    return allSuspended && destroyed && stillWaiting && allSucceeded && heldJobSucceeded;
}

OVPHYSX_API bool ovphysx_wait_from_saturated_carbonite_pool_for_test_internal(
    ovphysx_instance_handle_t handle,
    uint32_t* out_waiters)
{
    if (out_waiters)
        *out_waiters = 0;
    std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !instance->executor)
        return false;
    carb::Framework* framework = carb::getFramework();
    carb::tasking::ITasking* tasking =
        framework ? framework->tryAcquireExistingInterface<carb::tasking::ITasking>() : nullptr;
    if (!tasking)
        return false;

    const uint32_t waiterCount = 2 * tasking->getDesc().threadCount + 2;
    struct Shared
    {
        std::vector<ovphysx_enqueue_result_t> receipts;
        std::unique_ptr<std::atomic<int>[]> statuses;
    };
    // Waiting tasks capture the shared state, so an early return stays memory safe.
    std::shared_ptr<Shared> shared = std::make_shared<Shared>();
    shared->statuses.reset(new std::atomic<int>[waiterCount]);
    for (uint32_t index = 0; index < waiterCount; ++index)
        shared->statuses[index].store(-1);

    instance->executor->setHold(true);
    struct ReleaseHold
    {
        ovphysx::InstanceExecutor& executor;
        ~ReleaseHold() { executor.setHold(false); }
    } releaseHold{*instance->executor};

    for (uint32_t index = 0; index < waiterCount; ++index)
    {
        const ovphysx_enqueue_result_t receipt = ovphysx_submit_test_job_for_test_internal(handle, nullptr, 0);
        if (receipt.status != OVPHYSX_API_SUCCESS)
            return false;
        shared->receipts.push_back(receipt);
    }

    std::vector<carb::tasking::Future<void>> waiters;
    waiters.reserve(waiterCount);
    for (uint32_t index = 0; index < waiterCount; ++index)
    {
        waiters.push_back(tasking->addTask(carb::tasking::Priority::eDefault, {}, [handle, shared, index]() {
            ovphysx_wait_task_result_t result{};
            const ovphysx_result_t waited =
                ovphysx_wait_task(handle, shared->receipts[index], OVPHYSX_TIMEOUT_INFINITE, &result);
            ovphysx_destroy_wait_task_result(&result);
            shared->statuses[index].store(static_cast<int>(waited.status));
        }));
    }

    // Every job is still held. Wait until each waiter is suspended as a task inside
    // the executor; a waiter that blocked its OS thread would never be counted.
    const std::chrono::steady_clock::time_point suspendDeadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (instance->executor->waitingTaskCount() < waiterCount &&
           std::chrono::steady_clock::now() < suspendDeadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const bool allSuspended = instance->executor->waitingTaskCount() == waiterCount &&
                              instance->executor->waitingThreadCount() == 0;
    // With more waiters than workers, a probe can run only if waiting tasks released
    // their worker threads.
    carb::tasking::Future<void> probe = tasking->addTask(carb::tasking::Priority::eDefault, {}, []() {});
    const bool probeRan = probe.wait_for(std::chrono::seconds(10));
    bool stillWaiting = true;
    for (uint32_t index = 0; index < waiterCount; ++index)
        stillWaiting = stillWaiting && shared->statuses[index].load() == -1;

    instance->executor->setHold(false);
    bool waitersFinished = true;
    for (carb::tasking::Future<void>& waiter : waiters)
        waitersFinished = waiter.wait_for(std::chrono::seconds(30)) && waitersFinished;
    bool allSucceeded = waitersFinished;
    for (uint32_t index = 0; allSucceeded && index < waiterCount; ++index)
        allSucceeded = shared->statuses[index].load() == static_cast<int>(OVPHYSX_API_SUCCESS);

    if (out_waiters)
        *out_waiters = waiterCount;
    return allSuspended && probeRan && stillWaiting && allSucceeded;
}

OVPHYSX_API uint64_t ovphysx_worker_cuda_calls_for_test_internal(void)
{
    return ovphysx_worker_cuda_calls();
}

OVPHYSX_API uint64_t ovphysx_exchange_next_output_handle_for_test_internal(uint64_t next)
{
    return ovphysx::internal::exchangeNextOpaqueObjectHandle(next);
}

OVPHYSX_API bool ovphysx_set_test_execution_for_test_internal(ovphysx_test_execution_t mode)
{
    switch (mode)
    {
    case OVPHYSX_TEST_EXECUTION_NATURAL:
        ovphysx_set_test_execution(ovphysx::TestExecution::kNatural);
        return true;
    case OVPHYSX_TEST_EXECUTION_QUEUE:
        ovphysx_set_test_execution(ovphysx::TestExecution::kQueue);
        return true;
    }
    return false;
}

OVPHYSX_API bool ovphysx_read_step_bookkeeping_for_test_internal(
    ovphysx_instance_handle_t handle, float* simulation_time, float* last_step_dt)
{
    const std::shared_ptr<InstanceData> instance = get_instance(handle);
    if (!instance || !simulation_time || !last_step_dt)
        return false;
    *simulation_time = instance->sim_time;
    *last_step_dt = instance->last_step_dt;
    return true;
}

} // extern "C"
