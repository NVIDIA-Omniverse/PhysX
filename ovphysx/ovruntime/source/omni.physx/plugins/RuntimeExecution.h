// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CUSTOM-PARAMETERS-001
// @covers AC-1

// @implements REQ-ATTACH-EXECUTION-001
// @covers AC-3 AC-4
#pragma once

#include <omni/physics/AttachHandle.h>
#include <atomic>
#include <algorithm>
#include <vector>
#include <memory>
#include <shared_mutex>

namespace omni::physx
{
namespace usdparser { class AttachedStage; }

struct RuntimeExecutionState
{
    std::atomic_flag entered = ATOMIC_FLAG_INIT;
    AttachHandle pendingOwner = omni::physics::kNoAttach;
    std::vector<uint64_t> pendingScenes;
};

RuntimeExecutionState& getRuntimeExecutionState();
std::atomic<uint32_t>& runtimeOperationCount();

// Runtime mutex waits may migrate a Carbonite task between workers. Callback
// state follows that task; native SDK worker callbacks use thread storage.
void initializeRuntimeCallbacks();
void shutdownRuntimeCallbacks();
uint32_t getRuntimeCallbackDepth();
AttachHandle preStepCallbackOwner();
const usdparser::AttachedStage* customCreationOwner();
struct RuntimeCallbackState
{
    uint32_t depth = 0;
    AttachHandle preStepOwner = omni::physics::kNoAttach;
    const usdparser::AttachedStage* customOwner = nullptr;
};
class RuntimeCallbackScope
{
public:
    RuntimeCallbackScope(AttachHandle attach = omni::physics::kNoAttach, bool preStep = false);
    RuntimeCallbackScope(AttachHandle attach, bool preStep, const usdparser::AttachedStage* customOwner);
    ~RuntimeCallbackScope();
    RuntimeCallbackScope(const RuntimeCallbackScope&) = delete;
    RuntimeCallbackScope& operator=(const RuntimeCallbackScope&) = delete;
private:
    RuntimeCallbackState mState;
    RuntimeCallbackState* mPrevious = nullptr;

};


// One operation at a time per attachment. The lease keeps the selected owner alive
// throughout the call. Logical pending state lasts until the final fetch, even
// after SDK tasks finish. kNoAttach serializes runtime configuration calls only.
// Internal helpers execute under their public entry and do not enter recursively.
class RuntimeExecutionScope
{
public:
    explicit RuntimeExecutionScope(AttachHandle attach, bool allowPendingOwner = false, bool preStepForce = false);
    ~RuntimeExecutionScope();
    RuntimeExecutionScope(const RuntimeExecutionScope&) = delete;
    RuntimeExecutionScope& operator=(const RuntimeExecutionScope&) = delete;
    explicit operator bool() const { return mAllowed; }
    bool hasPending() const { return mState->pendingOwner != omni::physics::kNoAttach; }
    bool hasPendingScene(uint64_t scene) const { return std::find(mState->pendingScenes.begin(), mState->pendingScenes.end(), scene) != mState->pendingScenes.end(); }
    void begin(AttachHandle attach) { mState->pendingOwner = attach; }
    void addScene(uint64_t scene) { mState->pendingScenes.push_back(scene); }
    void complete(uint64_t scene = 0)
    {
        if (scene == 0)
            mState->pendingScenes.clear();
        else
        {
            const auto found = std::find(mState->pendingScenes.begin(), mState->pendingScenes.end(), scene);
            if (found != mState->pendingScenes.end())
                mState->pendingScenes.erase(found);
        }
        if (mState->pendingScenes.empty())
            mState->pendingOwner = omni::physics::kNoAttach;
    }
private:
    std::shared_ptr<usdparser::AttachedStage> mOwner;
    RuntimeExecutionState mInvalidOwnerState;
    RuntimeExecutionState* mState = nullptr;
    bool mOwnsEntry = false;
    bool mAllowed = false;
};

// Pure copied-data kernels share the SDK but neither own nor enter a scene.
// Only recording transport changes exclude them; playback and attach stay usable.
std::shared_mutex& runtimeAuthoringMutex();
class RuntimeAuthoringScope
{
public:
    RuntimeAuthoringScope();
    ~RuntimeAuthoringScope();
    RuntimeAuthoringScope(const RuntimeAuthoringScope&) = delete;
    RuntimeAuthoringScope& operator=(const RuntimeAuthoringScope&) = delete;
    explicit operator bool() const { return mLock.owns_lock(); }
private:
    std::shared_lock<std::shared_mutex> mLock;
};

// Called while holding configuration admission, which excludes attachment
// publication and native retirement, including property-query temporary owners.
bool admitIdleRecordingOwners(std::vector<std::unique_ptr<RuntimeExecutionScope>>& owners);

// Internal retirement primitive, called after the owner is fully drained and
// destroyed under the lifecycle caller's external serialization.
void retireRuntimeExecution(AttachHandle attach);
}
