// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-TASK-001
// @covers AC-1 AC-4 AC-5

/**
 * @implements REQ-CAPI-ASYNC-001
 * @covers AC-2
 *
 * @implements REQ-CAPI-SDF-001
 * @covers AC-1
 */


// @implements REQ-CAPI-COMPANION-001
// @covers AC-2

// C++ wrapper over the ovphysx C API, declared in ovphysx/experimental/ovphysx.hpp.

#include "ovphysx/experimental/ovphysx.hpp"
#include "ovphysx/ovphysx_geometry_query.h"
#include "ovphysx/ovphysx_types.h"
#include "carb/logging/Log.h"
#include <exception>
#include <stdexcept>
#include <utility>

namespace ovphysx {

namespace {
inline bool ensureHandle(ovphysx_instance_handle_t h, const char* fn) {
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

PhysX::PhysX(ovphysx_instance_handle_t h) : m_handle(h) {}

PhysX::PhysX() : m_handle(0) {}

PhysX::~PhysX() {
    if (m_handle) {
        const ovphysx_result_t result = ovphysx_destroy_instance(m_handle);
        if (result.status == OVPHYSX_API_INVALID_STATE) {
            CARB_LOG_ERROR("PhysX destructor requires closed leases and an idle runtime; call reset() and retry before leaving scope");
            std::terminate(); // A noexcept destructor cannot transfer a still-live owner to its caller.
        }
    }
}

PhysX::PhysX(PhysX&& other) noexcept : m_handle(other.m_handle) {
    other.m_handle = 0;
}

PhysX& PhysX::operator=(PhysX&& other) noexcept {
    // Swap ownership: the moved-from object retains our prior handle, so no
    // fallible destruction is hidden inside a noexcept assignment.
    std::swap(m_handle, other.m_handle);
    return *this;
}

ovphysx_instance_handle_t PhysX::release() {
    ovphysx_instance_handle_t h = m_handle;
    m_handle = 0;
    return h;
}

void PhysX::reset(ovphysx_instance_handle_t h) {
    if (h == m_handle) return;
    if (m_handle) {
        const ovphysx_result_t result = ovphysx_destroy_instance(m_handle);
        if (result.status == OVPHYSX_API_INVALID_STATE)
            throw std::runtime_error("PhysX reset refused: close outstanding leases or finish concurrent operations, then retry");
    }
    m_handle = h;
}

//------------------------------------------------------------------------------------------------------------
// Stage Management
//------------------------------------------------------------------------------------------------------------

ovphysx_enqueue_result_t PhysX::detachOvstage() {
    return ovphysx_detach_ovstage(m_handle);
}

ovphysx_enqueue_result_t PhysX::attachOvstage(ovstage_instance_t* stage, ovstage_ordinal_t read_ordinal) {
    return ovphysx_attach_ovstage(m_handle, stage, read_ordinal);
}

ovphysx_enqueue_result_t PhysX::updateFromOvstage(ovstage_ordinal_range_t range) {
    return ovphysx_update_from_ovstage(m_handle, range);
}

//------------------------------------------------------------------------------------------------------------
// Simulation
//------------------------------------------------------------------------------------------------------------

ovphysx_enqueue_result_t PhysX::step(float step_dt) {
    return ovphysx_step(m_handle, step_dt);
}

ovphysx_enqueue_result_t PhysX::updateArticulationsKinematic(ovphysx_query_handle_t query, uint32_t flags) {
    return ovphysx_update_articulations_kinematic(m_handle, query, flags);
}

ovphysx_enqueue_result_t PhysX::evaluateSdfShapes(ovphysx_query_handle_t query,
                                                  const DLTensor* points,
                                                  DLTensor* out,
                                                  ovstage_cuda_sync_t pointsReady) {
    return ovphysx_evaluate_sdf_shapes(m_handle, query, points, out, pointsReady);
}

//------------------------------------------------------------------------------------------------------------
// Synchronization
//------------------------------------------------------------------------------------------------------------

physx::WaitResult PhysX::waitTask(ovphysx_enqueue_result_t task, ovphysx_timeout_t timeout_ns) {
    physx::WaitResult result;
    result.waitTask(m_handle, task, timeout_ns);
    return result;
}

physx::WaitResult PhysX::waitAll(ovphysx_timeout_t timeout_ns) {
    physx::WaitResult result;
    result.waitAll(m_handle, timeout_ns);
    return result;
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
    // Point the string views at this object's copies, not other's.
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

ovphysx_api_status_t PhysX::create(PhysX& out_instance)
{
    ovphysx_instance_handle_t handle = 0;
    const ovphysx_result_t result = ovphysx_create_instance(&handle);
    if (result.status == OVPHYSX_API_SUCCESS)
        out_instance = PhysX(handle);
    return result.status;
}

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

    ovphysx_instance_handle_t handle = 0;
    ovphysx_result_t result = ovphysx_create_instance_with_args(&cArgs, &handle);

    if (result.status == OVPHYSX_API_SUCCESS) {
        out_instance = PhysX(handle);
    }
    return result.status;
}

ovphysx_api_status_t PhysX::setCpuMode(bool cpuOnly)
{
    return ovphysx_set_global_config(
        OVPHYSX_GLOBAL_CONFIG_CPU_ONLY,
        ovphysx_config_value_bool(cpuOnly)).status;
}

ovphysx_api_status_t PhysX::getCpuMode(bool& outCpuOnly)
{
    ovphysx_config_value_t value = {};
    const ovphysx_result_t result =
        ovphysx_get_global_config(OVPHYSX_GLOBAL_CONFIG_CPU_ONLY, &value);
    if (result.status == OVPHYSX_API_SUCCESS)
        outCpuOnly = value.value.bool_value;
    return result.status;
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

ovphysx_enqueue_result_t PhysX::clone(const std::string& sourcePath, const std::vector<std::string>& targetPaths,
                                      const float* anchorTransforms, const uint32_t* envIds)
{
    std::vector<ovphysx_string_t> targetStrings;
    targetStrings.reserve(targetPaths.size());
    for (const std::string& path : targetPaths) {
        targetStrings.push_back({path.c_str(), path.size()});
    }
    return ovphysx_clone(
        m_handle,
        {sourcePath.c_str(), sourcePath.size()},
        targetStrings.data(),
        static_cast<uint32_t>(targetStrings.size()),
        anchorTransforms,
        envIds);
}

} // namespace ovphysx
