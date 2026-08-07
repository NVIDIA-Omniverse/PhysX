// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//


// Implementation of C++ wrapper for ovphysx C API
// See: ovphysx/experimental/ovphysx.hpp

#include "AsyncEventManager/AsyncEventManager.hpp"
#include "ovphysx/experimental/ovphysx.hpp"
#include "ovphysx/ovphysx_types.h"
#include "carb/logging/Log.h"

namespace ovphysx {

using ovphysx::async::async_event_handle_t;

namespace {
inline bool ensureHandle(ovphysx_handle_t h, const char* fn) {
    if (!h) {
        CARB_LOG_ERROR("%s: invalid ovphysx_handle (null). Did PhysX::create(...) succeed?", fn);
        return false;
    }
    return true;
}
}

//------------------------------------------------------------------------------------------------------------
// Constructors / Destructor / Move
//------------------------------------------------------------------------------------------------------------

PhysX::PhysX(ovphysx_handle_t h) : m_handle(h) {}

PhysX::PhysX() : m_handle(0) {}

PhysX::~PhysX() {
    if (m_handle) {
        ovphysx_destroy_instance(m_handle);
    }
}

PhysX::PhysX(PhysX&& other) noexcept : m_handle(other.m_handle) {
    other.m_handle = 0;
}

PhysX& PhysX::operator=(PhysX&& other) noexcept {
    if (this != &other) {
        if (m_handle) {
            ovphysx_destroy_instance(m_handle);
        }
        m_handle = other.m_handle;
        other.m_handle = 0;
    }
    return *this;
}

ovphysx_handle_t PhysX::release() {
    ovphysx_handle_t h = m_handle;
    m_handle = 0;
    return h;
}

void PhysX::reset(ovphysx_handle_t h) {
    if (m_handle) {
        ovphysx_destroy_instance(m_handle);
    }
    m_handle = h;
}

//------------------------------------------------------------------------------------------------------------
// Stage Management
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t PhysX::reset_stage() {
    if (!ensureHandle(m_handle, "PhysX::reset_stage")) return OVPHYSX_API_ERROR;
    ovphysx_enqueue_result_t r = ovphysx_reset_stage(m_handle);
    return r.status;
}

ovphysx_api_status_t PhysX::attachOvstage(ovstage_instance_t* stage, ovstage_ordinal_t read_ordinal) {
    if (!ensureHandle(m_handle, "PhysX::attachOvstage")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_attach_ovstage(m_handle, stage, read_ordinal);
    return r.status;
}

ovphysx_api_status_t PhysX::updateFromOvstage(ovstage_ordinal_range_t range) {
    if (!ensureHandle(m_handle, "PhysX::updateFromOvstage")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_update_from_ovstage(m_handle, range);
    return r.status;
}

//------------------------------------------------------------------------------------------------------------
// Simulation
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t PhysX::step(float step_dt) {
    if (!ensureHandle(m_handle, "PhysX::step")) return OVPHYSX_API_ERROR;
    ovphysx_enqueue_result_t r = ovphysx_step(m_handle, step_dt);
    return r.status;
}

ovphysx_api_status_t PhysX::updateArticulationsKinematic() {
    if (!ensureHandle(m_handle, "PhysX::updateArticulationsKinematic")) return OVPHYSX_API_ERROR;
    ovphysx_result_t r = ovphysx_update_articulations_kinematic(m_handle);
    return r.status;
}

//------------------------------------------------------------------------------------------------------------
// User Tasks
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t PhysX::addUserTask(const ovphysx_user_task_desc_t& desc,
                                         ovphysx_op_index_t& out_op_index) {
    if (!ensureHandle(m_handle, "PhysX::addUserTask")) return OVPHYSX_API_ERROR;
    ovphysx_enqueue_result_t r = ovphysx_add_user_task(m_handle, &desc);
    out_op_index = r.op_index;
    return r.status;
}

//------------------------------------------------------------------------------------------------------------
// Synchronization
//------------------------------------------------------------------------------------------------------------

physx::WaitResult PhysX::waitOp(ovphysx_op_index_t op_index, uint64_t timeout_ns) {
    if (!ensureHandle(m_handle, "PhysX::waitOp")) {
        return physx::WaitResult();
    }
    physx::WaitResult w;
    ovphysx_wait_op(m_handle, op_index, timeout_ns, w.get());
    return w;
}

physx::WaitResult PhysX::waitAll(uint64_t timeout_ns) {
    if (!ensureHandle(m_handle, "PhysX::waitAll")) {
        return physx::WaitResult();
    }
    physx::WaitResult w;
    ovphysx_wait_op(m_handle, OVPHYSX_OP_INDEX_ALL, timeout_ns, w.get());
    return w;
}

//------------------------------------------------------------------------------------------------------------
// CreateArgs
//------------------------------------------------------------------------------------------------------------

CreateArgs::CreateArgs() : m_args(OVPHYSX_CREATE_ARGS_DEFAULT) {}

// Helper to update the ovphysx_string_t pointer after a string member changes.
static ovphysx_string_t _str_ref(const std::string& s)
{
    return { s.empty() ? nullptr : s.c_str(), s.size() };
}

CreateArgs::CreateArgs(const CreateArgs& other)
    : m_activeCudaGpus(other.m_activeCudaGpus)
    , m_bundledDepsPath(other.m_bundledDepsPath)
    , m_args(other.m_args)
{
    // Fix up pointers to point into our own string copies, not other's.
    m_args.active_cuda_gpus = _str_ref(m_activeCudaGpus);
    m_args.bundled_deps_path = _str_ref(m_bundledDepsPath);
}

CreateArgs& CreateArgs::operator=(const CreateArgs& other)
{
    if (this != &other) {
        m_activeCudaGpus = other.m_activeCudaGpus;
        m_bundledDepsPath = other.m_bundledDepsPath;
        m_args = other.m_args;
        m_args.active_cuda_gpus = _str_ref(m_activeCudaGpus);
        m_args.bundled_deps_path = _str_ref(m_bundledDepsPath);
    }
    return *this;
}

CreateArgs::CreateArgs(CreateArgs&& other) noexcept
    : m_activeCudaGpus(std::move(other.m_activeCudaGpus))
    , m_bundledDepsPath(std::move(other.m_bundledDepsPath))
    , m_args(other.m_args)
{
    m_args.active_cuda_gpus = _str_ref(m_activeCudaGpus);
    m_args.bundled_deps_path = _str_ref(m_bundledDepsPath);
    // Null out other's pointers so the moved-from state is unambiguously safe.
    other.m_args.active_cuda_gpus = {nullptr, 0};
    other.m_args.bundled_deps_path = {nullptr, 0};
}

CreateArgs& CreateArgs::operator=(CreateArgs&& other) noexcept
{
    if (this != &other) {
        m_activeCudaGpus = std::move(other.m_activeCudaGpus);
        m_bundledDepsPath = std::move(other.m_bundledDepsPath);
        m_args = other.m_args;
        m_args.active_cuda_gpus = _str_ref(m_activeCudaGpus);
        m_args.bundled_deps_path = _str_ref(m_bundledDepsPath);
        other.m_args.active_cuda_gpus = {nullptr, 0};
        other.m_args.bundled_deps_path = {nullptr, 0};
    }
    return *this;
}


void CreateArgs::setActiveCudaGpus(const std::string& gpus)
{
    m_activeCudaGpus = gpus;
    m_args.active_cuda_gpus = _str_ref(m_activeCudaGpus);
}

void CreateArgs::setBundledDepsPath(const std::string& path)
{
    m_bundledDepsPath = path;
    m_args.bundled_deps_path = _str_ref(m_bundledDepsPath);
}

void CreateArgs::setConfigEntries(const ovphysx_config_entry_t* entries, uint32_t count)
{
    m_args.config_entries = entries;
    m_args.config_entry_count = count;
}

const ovphysx_create_args& CreateArgs::cArgs() const { return m_args; }

//------------------------------------------------------------------------------------------------------------
// Factory
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t PhysX::create(
    PhysX& out_instance,
    const CreateArgs& args)
{
    const ovphysx_create_args& cArgs = args.cArgs();

    if (cArgs.config_entry_count > 0 && cArgs.config_entries == nullptr) {
        CARB_LOG_ERROR("PhysX::create: config_entry_count > 0 but config_entries is null");
        return OVPHYSX_API_INVALID_ARGUMENT;
    }
    if (cArgs.bundled_deps_path.length > 0 && cArgs.bundled_deps_path.ptr == nullptr) {
        CARB_LOG_ERROR("PhysX::create: bundled_deps_path.length > 0 but ptr is null");
        return OVPHYSX_API_INVALID_ARGUMENT;
    }

    ovphysx_handle_t handle = 0;
    ovphysx_result_t result = ovphysx_create_instance(&cArgs, &handle);

    if (result.status == OVPHYSX_API_SUCCESS) {
        out_instance = PhysX(handle);
    }
    return result.status;
}

ovphysx_api_status_t PhysX::setCpuMode(bool cpuOnly)
{
    return ovphysx_set_cpu_mode(cpuOnly).status;
}

//------------------------------------------------------------------------------------------------------------
// Tensor Bindings
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t PhysX::createTensorBinding(
    TensorBinding& out_binding,
    const std::string& pattern,
    ovphysx_tensor_type_t tensor_type)
{
    if (!ensureHandle(m_handle, "PhysX::createTensorBinding")) return OVPHYSX_API_ERROR;

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = {pattern.c_str(), pattern.size()};
    desc.tensor_type = tensor_type;
    desc.prim_paths = nullptr;
    desc.prim_paths_count = 0;

    ovphysx_tensor_binding_handle_t bindingHandle = 0;
    ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &bindingHandle);

    if (r.status == OVPHYSX_API_SUCCESS) {
        out_binding = TensorBinding(m_handle, bindingHandle);
    }
    return r.status;
}

//------------------------------------------------------------------------------------------------------------
// Clone
//------------------------------------------------------------------------------------------------------------

ovphysx_api_status_t PhysX::clone(const std::string& sourcePath, const std::vector<std::string>& targetPaths,
                                  const float* parentTransforms, const uint32_t* envIds,
                                  ovphysx_op_index_t* outOpIndex)
{
    if (!ensureHandle(m_handle, "PhysX::clone")) return OVPHYSX_API_ERROR;

    if (sourcePath.empty()) {
        CARB_LOG_ERROR("Source path must be a non-empty string");
        return OVPHYSX_API_INVALID_ARGUMENT;
    }

    if (targetPaths.empty()) {
        CARB_LOG_ERROR("Target paths must be a non-empty vector");
        return OVPHYSX_API_INVALID_ARGUMENT;
    }

    for (const auto& targetPath : targetPaths) {
        if (targetPath == sourcePath) {
            CARB_LOG_ERROR("Target path '%s' cannot be the same as source path", targetPath.c_str());
            return OVPHYSX_API_INVALID_ARGUMENT;
        }
    }

    std::vector<ovphysx_string_t> targetStrings;
    targetStrings.reserve(targetPaths.size());
    for (const auto& path : targetPaths) {
        targetStrings.push_back({path.c_str(), path.size()});
    }

    ovphysx_enqueue_result_t status = ovphysx_clone(
        m_handle,
        {sourcePath.c_str(), sourcePath.size()},
        targetStrings.data(),
        static_cast<uint32_t>(targetStrings.size()),
        parentTransforms,
        envIds
    );

    if (status.status != OVPHYSX_API_SUCCESS) {
        ovphysx_string_t err = ovphysx_get_last_error();
        if (err.ptr && err.length > 0) {
            CARB_LOG_ERROR(
                "Clone failed: status=%d, source=\"%s\", targets=%zu, error=\"%.*s\"",
                status.status,
                sourcePath.c_str(),
                targetPaths.size(),
                static_cast<int>(err.length),
                err.ptr
            );
        } else {
            CARB_LOG_ERROR("Clone failed: status=%d, source=\"%s\", targets=%zu",
                           status.status, sourcePath.c_str(), targetPaths.size());
        }
        return OVPHYSX_API_ERROR;
    }

    // Check clone operation result (operation completes before ovphysx_clone returns)
    async_event_handle_t cloneEvent = ovphysx::async::get_event_for_op(m_handle, status.op_index);
    if (ovphysx::async::AsyncEventManager::poll_event(cloneEvent) != ASYNC_STATUS_COMPLETED) {
        CARB_LOG_ERROR("Clone operation failed");
        return OVPHYSX_API_ERROR;
    }

    // Surface the op index for API uniformity with the C/Python forms (already complete here).
    if (outOpIndex) {
        *outOpIndex = status.op_index;
    }
    return OVPHYSX_API_SUCCESS;
}

} // namespace ovphysx
