// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
// Included in the copied OVStage host after its unsupported-operation helper.

struct WasmTaskStorageSlot
{
    carb::tasking::TaskStorageKey key = carb::tasking::kInvalidTaskStorageKey;
    carb::tasking::TaskStorageDestructorFn destructor = nullptr;
};

struct WasmTaskStorageRegistry
{
    std::mutex mutex;
    carb::tasking::TaskStorageKey nextKey = 0;
    WasmTaskStorageSlot slots[64];
};

WasmTaskStorageRegistry& taskStorageRegistry()
{
    static WasmTaskStorageRegistry registry;
    return registry;
}

carb::tasking::TaskStorageKey allocateTaskStorage(carb::tasking::TaskStorageDestructorFn destructor)
{
    WasmTaskStorageRegistry& registry = taskStorageRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    WasmTaskStorageSlot* available = nullptr;
    for (WasmTaskStorageSlot& slot : registry.slots)
    {
        if (slot.key == carb::tasking::kInvalidTaskStorageKey)
        {
            available = &slot;
            break;
        }
    }
    if (!available)
        return carb::tasking::kInvalidTaskStorageKey;

    // Keys remain distinct while live, including after the counter wraps.
    for (;;)
    {
        const carb::tasking::TaskStorageKey key = registry.nextKey++;
        if (key == carb::tasking::kInvalidTaskStorageKey)
            continue;
        bool inUse = false;
        for (const WasmTaskStorageSlot& slot : registry.slots)
            if (slot.key == key)
                inUse = true;
        if (!inUse)
        {
            available->key = key;
            available->destructor = destructor;
            return key;
        }
    }
}

void freeTaskStorage(carb::tasking::TaskStorageKey key)
{
    WasmTaskStorageRegistry& registry = taskStorageRegistry();
    std::lock_guard<std::mutex> lock(registry.mutex);
    if (key != carb::tasking::kInvalidTaskStorageKey)
    {
        for (WasmTaskStorageSlot& slot : registry.slots)
        {
            if (slot.key == key)
            {
                slot.key = carb::tasking::kInvalidTaskStorageKey;
                // The API explicitly excludes destructor calls when freeing a key.
                slot.destructor = nullptr;
                return;
            }
        }
    }
    unsupported("freeing an invalid or already freed task storage key");
}

void installTaskStorage(carb::tasking::ITasking& table)
{
    table.allocTaskStorage = allocateTaskStorage;
    table.freeTaskStorage = freeTaskStorage;
    // This host has no task scheduler. The documented outcomes outside task
    // context are false/null; values are never stored and no task is fabricated.
    table.setTaskStorage = [](carb::tasking::TaskStorageKey, void*) { return false; };
    table.getTaskStorage = [](carb::tasking::TaskStorageKey) -> void* { return nullptr; };
}
