// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-PREVIEW-001
// @covers AC-1 AC-2 AC-4 AC-5 AC-6 AC-7
// @implements REQ-CAPI-COMPANION-001
// @covers AC-4
#include "ovphysx_companion/preview.h"
#include "internal/sdk/ovphysxSDK.hpp"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxSimulation.h>
#include <omni/physx/IPhysxStatistics.h>
#include <omni/physx/PhysXRuntime.h>
#include <cmath>
#include <limits>

namespace ovphysx::companion::previews
{
using Status = ovphysx_companion_status_t;
using namespace omni::physx;
struct Preview {
    std::mutex mutex;
    CompanionServiceLease lease{companionRuntime()};
    std::shared_ptr<ovphysx::CarboniteLoader> services;
    AttachHandle attachment = 0;
    const void* stage = nullptr;
    uint64_t ordinal = 0;
    float time = 0;
    float pendingDt = 0;
    bool pending = false;
    bool closed = false;
    explicit Preview(std::shared_ptr<ovphysx::CarboniteLoader> value) : services(std::move(value))
    {}
    ~Preview() { services.reset(); }
};

Status convert(RuntimeResult result) {
    switch (result) {
    case RuntimeResult::eSuccess: return OVPHYSX_COMPANION_OK;
    case RuntimeResult::eBusy: return OVPHYSX_COMPANION_BUSY;
    case RuntimeResult::eInvalidAttachment: return OVPHYSX_COMPANION_STALE;
    case RuntimeResult::eInvalidScene: return OVPHYSX_COMPANION_NOT_FOUND;
    case RuntimeResult::eInvalidArgument: return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    }
    return OVPHYSX_COMPANION_ERROR;
}

// No registry/instance lock is held while entering runtime code or callbacks.
// try_lock prevents callback reentry and same-owner conflicts from deadlocking.
template<class Fn> Status withPreview(uint64_t handle, bool allowPending, Fn&& fn) noexcept {
    try {
        CompanionOperationGuard transition(companionRuntime());
        if (!transition) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<Preview> p;
        {
            std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex);
            const std::map<uint64_t, std::shared_ptr<Preview>>::iterator it = companionRuntime().previews().entries.find(handle);
            if (it == companionRuntime().previews().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
            p = it->second;
        }
        std::unique_lock<std::mutex> owner(p->mutex, std::try_to_lock);
        if (!owner.owns_lock()) return OVPHYSX_COMPANION_BUSY;
        if (p->closed) return OVPHYSX_COMPANION_NOT_FOUND;
        if ((!allowPending && p->pending) || runtime::isRuntimeBusy(p->attachment, allowPending))
            return OVPHYSX_COMPANION_BUSY;
        return fn(*p);
    } catch (...) { return OVPHYSX_COMPANION_ERROR; }
}

bool validPreviewOverrides(const ovphysx_companion_preview_desc_t& desc) noexcept {
    constexpr uint32_t known = OVPHYSX_COMPANION_PREVIEW_THREADS | OVPHYSX_COMPANION_PREVIEW_DEVICE |
        OVPHYSX_COMPANION_PREVIEW_PIPELINE | OVPHYSX_COMPANION_PREVIEW_SOLVER;
    if (desc.overrides & ~known) return false;
    if ((desc.overrides & OVPHYSX_COMPANION_PREVIEW_THREADS) && (!desc.threads || desc.threads>255)) return false;
    if ((desc.overrides & OVPHYSX_COMPANION_PREVIEW_DEVICE) && desc.cuda_device < -1) return false;
    if ((desc.overrides & OVPHYSX_COMPANION_PREVIEW_PIPELINE) && (desc.gpu_pipeline < -1 || desc.gpu_pipeline>1)) return false;
    if ((desc.overrides & OVPHYSX_COMPANION_PREVIEW_SOLVER) && (desc.solver < -1 || desc.solver>1)) return false;
    return true;
}
Status openPreview(const ovphysx_companion_preview_desc_t* d, uint64_t* output) noexcept {
    if (!d || !output || d->size != sizeof(*d) || d->version != 1 || !d->stage || !d->read_ordinal ||
        !validPreviewOverrides(*d) || (d->excluded_path_count && !d->excluded_paths)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    for (uint32_t i = 0; i < d->excluded_path_count; ++i)
        if (!d->excluded_paths[i] || d->excluded_paths[i][0] != '/') return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    uint64_t handle = 0;
    try {
        CompanionOperationGuard transition(companionRuntime());
        if (!transition) return OVPHYSX_COMPANION_BUSY;
        std::shared_ptr<Preview> p;
        PhysxAttachmentSettings settings;
        PhysxSharedSettings shared;
        AttachHandle source = 0;
        {
            std::unique_lock<std::shared_mutex> lock(g_instances_mutex);
            const std::unordered_map<uint64_t, std::shared_ptr<InstanceData>>::iterator it = g_instances.find(d->source_instance);
            if (it == g_instances.end()) return OVPHYSX_COMPANION_NOT_FOUND;
            if (!companionRuntime().admissionOpen() || it->second->companionDestroying) return OVPHYSX_COMPANION_BUSY;
            for (const std::pair<const uint64_t, std::shared_ptr<InstanceData>> & entry : g_instances)
                if (entry.second->ovstage_attached && entry.second->ovstage_attach_payload.instance == d->stage)
                    return OVPHYSX_COMPANION_INVALID_ARGUMENT;
            source = it->second->attachHandle;
            p = std::make_shared<Preview>(it->second->carbonite);
        }
        if (!p->services) return OVPHYSX_COMPANION_UNAVAILABLE;
        if (!(source ? runtime::snapshotAttachedSettings(source, settings, shared) :
                       runtime::snapshotAttachDefaults(settings, shared))) return OVPHYSX_COMPANION_BUSY;
        if ((d->overrides & OVPHYSX_COMPANION_PREVIEW_PIPELINE) && d->gpu_pipeline == 1) {
            bool cpuOnly = false;
            if (ovphysx_get_cpu_mode(&cpuOnly).status != OVPHYSX_API_SUCCESS)
                return OVPHYSX_COMPANION_ERROR;
            if (cpuOnly) return OVPHYSX_COMPANION_UNAVAILABLE;
        }
        if (d->overrides & OVPHYSX_COMPANION_PREVIEW_THREADS) shared.threadCount = d->threads;
        if (d->overrides & OVPHYSX_COMPANION_PREVIEW_DEVICE) shared.cudaDevice = d->cuda_device;
        if (d->overrides & OVPHYSX_COMPANION_PREVIEW_PIPELINE) settings.gpuPipelineOverride = d->gpu_pipeline;
        if (d->overrides & OVPHYSX_COMPANION_PREVIEW_SOLVER) settings.solverTypeOverride = d->solver;
        // Preview output is read explicitly, never written to the source.
        settings.updateToUsd = false;
        settings.updateParticlesToUsd = false;
        settings.updateVelocitiesToUsd = false;
        std::lock_guard<std::mutex> candidateLock(p->mutex);
        {
            std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex);
            for (std::pair<const uint64_t, std::shared_ptr<Preview>> const & entry : companionRuntime().previews().entries)
                if (entry.second->stage == d->stage) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
            if (companionRuntime().previews().nextHandle == UINT64_MAX) return OVPHYSX_COMPANION_ERROR;
            handle = companionRuntime().previews().nextHandle++;
            p->stage = d->stage;
            p->ordinal = d->read_ordinal;
            companionRuntime().previews().entries.emplace(handle, p); // reserve before native attach: no throwing allocation after it
        }
        OvstageAttachPayload payload{d->stage, d->dictionary, 0};
        PhysxAttachOptions options{d->excluded_paths, d->excluded_path_count, &settings, &shared};
        bool busy = false;
        // A preview has no public recording owner, so it must not start the
        // core's configured startup recording.
        const AttachHandle attachment = runtime::attachOvstageWithAdmission(&payload, d->read_ordinal, options,
                                                                            /*startConfiguredRecording=*/false, busy);
        {
            std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex);
            p->attachment = attachment;
        }
        if (!attachment) {
            std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex);
            companionRuntime().previews().entries.erase(handle);
            return busy ? OVPHYSX_COMPANION_BUSY : OVPHYSX_COMPANION_ERROR;
        }
        *output = handle;
        return OVPHYSX_COMPANION_OK;
    } catch (...) {
        if (handle) { std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex); companionRuntime().previews().entries.erase(handle); }
        return OVPHYSX_COMPANION_ERROR;
    }
}
Status closePreview(uint64_t handle) noexcept {
    return withPreview(handle, true, [handle](Preview& p) -> Status {
        const Status result = convert(p.services->getPhysxSimulation()->detachStage(p.attachment));
        if (result != OVPHYSX_COMPANION_OK) return result;
        p.pending = false;
        p.closed = true;
        std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex);
        companionRuntime().previews().entries.erase(handle);
        return OVPHYSX_COMPANION_OK;
    });
}
Status identity(uint64_t h, ovphysx_companion_identity_t* output) noexcept {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withPreview(h, true, [=](Preview& p) -> Status {
        *output = {0, p.attachment}; // independent service lease, no core instance owner
        return OVPHYSX_COMPANION_OK;
    });
}
Status simulate(uint64_t h, float dt) noexcept {
    if (!std::isfinite(dt) || dt <= 0 || dt > 1) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withPreview(h, false, [=](Preview& p) -> Status {
        if (!std::isfinite(p.time + dt)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        const Status status = convert(p.services->getPhysxSimulation()->simulate(p.attachment, dt, p.time));
        if (status == OVPHYSX_COMPANION_OK) {
            p.pending = true; p.pendingDt = dt;
        }
        return status;
    });
}
Status fetch(uint64_t h) noexcept {
    return withPreview(h, true, [](Preview& p) -> Status {
        if (!p.pending) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        const Status status = convert(p.services->getPhysxSimulation()->fetchResults(p.attachment));
        if (status == OVPHYSX_COMPANION_OK) {
            p.pending = false; p.time += p.pendingDt;
        }
        return status;
    });
}
Status statistics(uint64_t h, const char* path, uint32_t size, ovphysx_companion_scene_statistics_t* output) noexcept {
    if (!path || path[0] != '/' || !output || size != sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withPreview(h, false, [=](Preview& p) -> Status {
        omni::physx::IPhysx * physics = runtime::tryGetPhysxInterface();
        omni::physx::IPhysxStatistics * stats = runtime::tryGetPhysxStatisticsInterface();
        const omni::physics::parse::ObjectKey key = physics->resolveObjectKey(p.attachment, path);
        if (!key.valid()) return OVPHYSX_COMPANION_NOT_FOUND;
        PhysicsSceneStats counters;
        if (!stats->getPhysXSceneStatistics(p.attachment, key.handle, counters)) return OVPHYSX_COMPANION_ERROR;
        *output = {sizeof(*output), 1, {0, p.attachment}, counters.nbDynamicRigids, counters.nbActiveDynamicRigids,
            counters.nbStaticRigids, counters.nbKinematicBodies, counters.nbArticulations, counters.nbActiveConstraints};
        return OVPHYSX_COMPANION_OK;
    });
}
Status rigidPose(uint64_t h, const char* path, ovphysx_companion_pose_t* output) noexcept {
    if (!path || path[0] != '/' || !output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withPreview(h, false, [=](Preview& p) -> Status {
        omni::physx::IPhysx * physics = runtime::tryGetPhysxInterface();
        const omni::physics::parse::ObjectKey key = physics->resolveObjectKey(p.attachment, path);
        if (!key.valid()) return OVPHYSX_COMPANION_NOT_FOUND;
        carb::Float3 position; carb::Float4 rotation;
        if (!physics->getRigidBodyTransformation(p.attachment, key, position, rotation)) return OVPHYSX_COMPANION_NOT_FOUND;
        *output = {{position.x, position.y, position.z}, {rotation.x, rotation.y, rotation.z, rotation.w}};
        return OVPHYSX_COMPANION_OK;
    });
}
Status update(uint64_t h, uint64_t from, uint64_t to) noexcept {
    if (!from || from > to) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withPreview(h, false, [=](Preview& p) -> Status {
        if (from <= p.ordinal) return OVPHYSX_COMPANION_STALE;
        if (!p.services->getPhysxSimulation()->updateFromOvStage(p.attachment, from, to)) return OVPHYSX_COMPANION_ERROR;
        p.ordinal = to;
        return OVPHYSX_COMPANION_OK;
    });
}
}
bool CompanionRuntime::hasPendingPreviews()
{
    std::lock_guard<std::mutex> lock(m_previews.mutex);
    for (const std::pair<const uint64_t, std::shared_ptr<ovphysx::companion::previews::Preview>>& entry : m_previews.entries)
        if (entry.second->pending)
            return true;
    return false;
}
std::shared_ptr<ovphysx::CarboniteLoader> companionRuntimeForAttachment(uint64_t attachment)
{
    if (!attachment) return {};
    {
        std::shared_lock<std::shared_mutex> lock(g_instances_mutex);
        for (const std::pair<const uint64_t, std::shared_ptr<InstanceData>> & entry:g_instances)
            if (entry.second->attachHandle==attachment && !entry.second->companionDestroying)
                return entry.second->carbonite;
    }
    std::lock_guard<std::mutex> lock(companionRuntime().previews().mutex);
    for (const std::pair<const uint64_t, std::shared_ptr<ovphysx::companion::previews::Preview>>& entry:companionRuntime().previews().entries)
        if (entry.second->attachment==attachment) return entry.second->services;
    return {};
}
using namespace ovphysx::companion::previews;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_preview_acquire(
    uint32_t version, uint32_t size, ovphysx_companion_preview_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version != 1 || size != sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output = {sizeof(*output), 1, openPreview, closePreview, identity, simulate, fetch, statistics, rigidPose, update};
    return OVPHYSX_COMPANION_OK;
}
