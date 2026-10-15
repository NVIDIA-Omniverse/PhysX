// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "ovphysxTestHelpers.h"
#include "CarboniteLoader/CarboniteLoader.hpp"
#include "ovphysx/ovphysx_attributes.h"

#include <omni/physx/PhysXRuntime.h>

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
#include <mutex>
#include <new>
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

    ovx_string_or_token_t name{};
    name.string.ptr = OVPHYSX_ATTR_PHYSX_POINTER;
    name.string.length = sizeof(OVPHYSX_ATTR_PHYSX_POINTER) - 1;
    ovphysx_read_handle_t read = 0;
    if (waitOk(ovphysx_read(handle, query, &name, 1, &read, nullptr)))
    {
        const ovstage_read_group_t* group = nullptr;
        ovphysx_enqueue_result_t fetched = ovphysx_fetch_read_next(handle, read, &group);
        if (waitOk(fetched) && group)
        {
            if (group->data.tensors && group->data.tensors[0].data)
                physicsPtr = reinterpret_cast<void*>(static_cast<const uint64_t*>(group->data.tensors[0].data)[0]);
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

} // extern "C"
