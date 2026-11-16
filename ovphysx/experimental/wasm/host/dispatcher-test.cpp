// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
#include "host.h"
#include <carb/Framework.h>
#include <carb/InterfaceUtils.h>
#include "PhysXCarbCPUDispatcher.h"
#include <cstdio>
#include <thread>
#include <unordered_set>
#include <vector>

CARB_FRAMEWORK_GLOBALS("ovphysx.wasm.dispatcher-test")

class ProbeTask final : public physx::PxBaseTask
{
public:
    unsigned phase = 0;
    unsigned runs = 0;
    unsigned releases = 0;
    const std::thread::id caller = std::this_thread::get_id();
    bool valid = true;

    void run() override
    {
        valid = valid && phase == 0 && std::this_thread::get_id() == caller;
        phase = 1;
        ++runs;
    }
    void release() override
    {
        valid = valid && phase == 1 && std::this_thread::get_id() == caller;
        phase = 2;
        ++releases;
    }
    const char* getName() const override { return "dispatcher-probe"; }
    void addReference() override {}
    void removeReference() override {}
    int32_t getReference() const override { return 1; }
};

unsigned taskDestructors = 0;
void taskStorageDestroyed(void*) { ++taskDestructors; }

int main()
{
    g_carbFramework = wasmFabricFramework();
    carb::tasking::ITasking* tasking = carb::getCachedInterface<carb::tasking::ITasking>();
    if (!tasking || tasking->getDesc().threadCount != 0)
        return 1;
    for (uint32_t requested : {0u, 1u, 8u})
    {
        PhysXCarbCpuDispatcher dispatcher(requested);
        if (dispatcher.getWorkerCount() != 0)
            return 2;
        ProbeTask task;
        dispatcher.submitTask(task);
        if (!task.valid || task.phase != 2 || task.runs != 1 || task.releases != 1)
            return 3;
        dispatcher.release();
    }
    std::vector<carb::tasking::TaskStorageKey> keys;
    std::unordered_set<carb::tasking::TaskStorageKey> unique;
    int value = 7;
    for (;;)
    {
        const carb::tasking::TaskStorageKey key = tasking->allocTaskStorage(taskStorageDestroyed);
        if (key == carb::tasking::kInvalidTaskStorageKey)
            break;
        if (!unique.insert(key).second || keys.size() == 4096)
            return 4;
        keys.push_back(key);
        if (tasking->setTaskStorage(key, &value) || tasking->getTaskStorage(key) != nullptr)
            return 5;
    }
    if (keys.empty())
        return 6;
    for (carb::tasking::TaskStorageKey key : keys)
        tasking->freeTaskStorage(key);
    const carb::tasking::TaskStorageKey reused = tasking->allocTaskStorage(nullptr);
    if (reused == carb::tasking::kInvalidTaskStorageKey || taskDestructors != 0)
        return 7;
    tasking->freeTaskStorage(reused);
    wasmFabricHostReset();
    std::puts("PASS official CPU dispatcher and task-storage allocation/lifetime; zero host workers");
    return 0;
}
