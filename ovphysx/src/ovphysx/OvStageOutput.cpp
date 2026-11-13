// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 */

#include <ovphysx/experimental/OvStageOutput.hpp>
#include <ovphysx/experimental/Helpers.hpp>
#include <ovphysx/ovphysx_attributes.h>

#include "OvStageOutputCuda.h"
#include "OvStageOutputMath.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <carb/logging/Log.h>
#include <ovstage/ovstage.h>
#include <ovstage/ovstage_api/ovstage_api.h>
#include <ovx/path_dictionary/path_dictionary.h>

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace ovphysx::utils
{
namespace
{
struct OutputFailure : std::runtime_error
{
    ovphysx_api_status_t status;
    OutputFailure(ovphysx_api_status_t value, const std::string& message) : std::runtime_error(message), status(value)
    {
    }
};

void require(bool condition, const char* message)
{
    if (!condition)
        throw OutputFailure(OVPHYSX_API_INVALID_ARGUMENT, message);
}

void checkPhysics(ovphysx_result_t result)
{
    if (result.status != OVPHYSX_API_SUCCESS)
    {
        const ovphysx_string_t error = ovphysx_get_last_error();
        throw OutputFailure(result.status, std::string(error.ptr, error.length));
    }
}

void waitPhysics(ovphysx_instance_handle_t instance, ovphysx_enqueue_result_t task)
{
    checkPhysics({ task.status });
    ovphysx::physx::WaitResult result;
    const ovphysx_api_status_t status = result.waitTask(instance, task);
    if (status == OVPHYSX_API_SUCCESS)
        return;
    const ovphysx_string_t error = result.hasErrors() ?
        ovphysx_get_last_task_error(instance, task) : ovphysx_get_last_error();
    throw OutputFailure(status, std::string(error.ptr ? error.ptr : "", error.length));
}

void cleanupPhysics(ovphysx_instance_handle_t instance, ovphysx_enqueue_result_t task) noexcept
{
    ovphysx::physx::WaitResult result;
    const ovphysx_api_status_t status = task.status == OVPHYSX_API_SUCCESS ?
        result.waitTask(instance, task) : task.status;
    if (status == OVPHYSX_API_SUCCESS)
        return;
    const ovphysx_string_t error = result.hasErrors() ?
        ovphysx_get_last_task_error(instance, task) : ovphysx_get_last_error();
    CARB_LOG_ERROR("Physics output cleanup failed (%d): %.*s", static_cast<int>(status),
                   static_cast<int>(error.length), error.ptr ? error.ptr : "");
}

void finishPhysicsRelease(ovphysx_instance_handle_t instance, uint64_t& resource, ovphysx_enqueue_result_t task)
{
    // Accepted releases transfer cleanup responsibility even when they fail.
    if (task.status == OVPHYSX_API_SUCCESS)
        resource = 0;
    waitPhysics(instance, task);
}

void checkDictionary(path_dictionary_instance_t* dictionary, ovx_api_result_t result)
{
    if (result.status != OVX_API_SUCCESS)
    {
        const std::string message(result.error.ptr ? result.error.ptr : "", result.error.length);
        path_dictionary_release_error(dictionary, result.error);
        throw OutputFailure(OVPHYSX_API_ERROR, message);
    }
}

void checkStage(ovstage_instance_t* stage, ovstage_api_status_t status)
{
    if (status != OVSTAGE_OK)
        throw OutputFailure(OVPHYSX_API_ERROR, ovstage_get_error_string(stage, status));
}

void waitStage(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    checkStage(stage, enqueue.status);
    ovstage_op_wait_result_t result{};
    const ovstage_api_status_t status = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
    std::string message;
    for (size_t i = 0; i < result.error_op_id_count; ++i)
    {
        const ovx_string_t error = ovstage_get_last_op_error(stage, result.error_op_ids[i]);
        if (!message.empty())
            message += "; ";
        message.append(error.ptr, error.length);
    }
    const ovstage_api_status_t releaseStatus = ovstage_release_op(stage, enqueue.op_index);
    if (result.error_op_id_count)
        throw OutputFailure(OVPHYSX_API_ERROR, message.empty() ? "OVStage operation failed" : message);
    checkStage(stage, status);
    checkStage(stage, releaseStatus);
}

// Cleanup must finish before borrowed descriptors or GPU buffers can disappear.
void cleanupStage(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue) noexcept
{
    if (enqueue.status == OVSTAGE_OK)
    {
        ovstage_op_wait_result_t result{};
        ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &result);
        ovstage_release_op(stage, enqueue.op_index);
    }
}

struct StageAccess
{
    ovstage_instance_t* stage;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;

    explicit StageAccess(ovstage_instance_t* value) : stage(value)
    {
    }
    ~StageAccess()
    {
        if (read != OVSTAGE_INVALID_READ_HANDLE)
            cleanupStage(stage, ovstage_release_read(stage, read));
        if (query != OVSTAGE_INVALID_QUERY_HANDLE)
            cleanupStage(stage, ovstage_release_query(stage, query));
    }
    void finish()
    {
        if (read != OVSTAGE_INVALID_READ_HANDLE)
            waitStage(stage, ovstage_release_read(stage, std::exchange(read, OVSTAGE_INVALID_READ_HANDLE)));
        if (query != OVSTAGE_INVALID_QUERY_HANDLE)
            waitStage(stage, ovstage_release_query(stage, std::exchange(query, OVSTAGE_INVALID_QUERY_HANDLE)));
    }
};

// Physics output groups carry one attribute column.
const ovstage_read_column_t& onlyColumn(const ovstage_read_group_t& group)
{
    require(group.column_count == 1 && group.columns, "Expected a single-column read group");
    return group.columns[0];
}

struct PhysicsRead
{
    ovphysx_instance_handle_t instance;
    ovphysx_query_handle_t query = 0;
    ovphysx_read_handle_t read = 0;

    explicit PhysicsRead(ovphysx_instance_handle_t value) : instance(value)
    {
    }
    ~PhysicsRead()
    {
        if (read)
            cleanupPhysics(instance, ovphysx_release_read(instance, read));
        if (query)
            cleanupPhysics(instance, ovphysx_release_query(instance, query));
    }
    void finish()
    {
        if (read)
            finishPhysicsRelease(instance, read, ovphysx_release_read(instance, read));
        if (query)
            finishPhysicsRelease(instance, query, ovphysx_release_query(instance, query));
    }
};

struct CacheEntry
{
    std::vector<ovx_primpath_t> paths;
    DLDevice device{};
    std::vector<double> matrices;
    std::unique_ptr<detail::OutputCudaBuffers> cuda;
};

// The three physics columns one world matrix is composed from, for one prim group.
struct PoseColumns
{
    const ovstage_read_group_t* position = nullptr;
    const ovstage_read_group_t* orientation = nullptr;
    const ovstage_read_group_t* scale = nullptr;
    ovphysx_query_target_t type;
    CacheEntry* entry = nullptr;
};

// Token order of the physics read and of OvStageOutputCache::Impl::tokens.
enum OutputToken
{
    kPositionToken,
    kOrientationToken,
    kWorldScaleToken,
    kWorldMatrixToken,
    kOutputTokenCount
};

void getPaths(path_dictionary_instance_t* dictionary, ovx_primpath_list_t list, std::vector<ovx_primpath_t>& paths)
{
    size_t count = 0;
    checkDictionary(dictionary, path_dictionary_get_num_paths_from_path_list(dictionary, list, &count));
    paths.resize(count);
    size_t copied = 0;
    checkDictionary(
        dictionary, path_dictionary_get_paths_from_path_list(dictionary, list, 0, count, paths.data(), &copied));
    require(copied == count, "Incomplete prim path list");
}

ovstage_ordinal_t writeFloor(ovstage_instance_t* stage, ovx_token_t attribute)
{
    ovstage_ordinal_query_handle_t query = OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE;
    ovstage_ordinal_t floor = 0;
    try
    {
        waitStage(stage, ovstage_get_attribute_write_floor(stage, { attribute, {} }, &query));
        checkStage(stage, ovstage_fetch_ordinal(stage, query, OVSTAGE_TIMEOUT_INFINITE, &floor));
        waitStage(stage,
                  ovstage_release_ordinal_query(stage, std::exchange(query, OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)));
    }
    catch (...)
    {
        if (query != OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE)
            cleanupStage(stage, ovstage_release_ordinal_query(stage, query));
        throw;
    }
    return floor;
}

void validateExtent(const DLTensor& tensor)
{
    const uint64_t scalarBytes = tensor.dtype.bits / 8;
    const uint64_t rowBytes = scalarBytes * tensor.dtype.lanes;
    const uint64_t stride = tensor.strides ? static_cast<uint64_t>(tensor.strides[0]) : 1;
    const uintptr_t address = reinterpret_cast<uintptr_t>(tensor.data);
    const uint64_t maxBytes =
        std::min<uint64_t>(std::numeric_limits<ptrdiff_t>::max(), std::numeric_limits<uintptr_t>::max() - address);
    require(address % scalarBytes == 0 && tensor.byte_offset % scalarBytes == 0 && tensor.byte_offset <= maxBytes,
            "Misaligned or unaddressable tensor storage");
    const uint64_t rows = (maxBytes - tensor.byte_offset) / rowBytes;
    require(rows && stride <= rows && static_cast<uint64_t>(tensor.shape[0] - 1) <= (rows - 1) / stride,
            "Tensor row extent exceeds addressable storage");
}

const DLTensor& columnTensor(const ovstage_read_group_t& group, uint16_t lanes)
{
    const ovstage_read_column_t& column = onlyColumn(group);
    require(!column.is_array && !column.is_delete && group.prims.list != OVX_INVALID_PRIMPATH_LIST && group.prims.count,
            "Expected a nonempty fixed pose or scale column");
    require(!group.prims.offset && !group.prims.index_map && !column.data.index_map && !column.data.mask,
            "Fixed pose and scale columns must have dense prim and data coverage");
    require(column.data.tensor_count == 1 && column.data.tensors, "Expected one fixed pose or scale tensor");
    const DLTensor& tensor = column.data.tensors[0];
    require(tensor.data && tensor.ndim == 1 && tensor.shape && tensor.shape[0] == group.prims.count &&
                tensor.dtype.code == kDLFloat && tensor.dtype.bits == 32 && tensor.dtype.lanes == lanes &&
                (!tensor.strides || tensor.strides[0] > 0),
            "Invalid fixed pose or scale tensor layout");
    validateExtent(tensor);
    require(tensor.device.device_type == kDLCPU || tensor.device.device_type == kDLCUDA,
            "Pose and scale columns require CPU or CUDA storage");
    return tensor;
}

const float* columnData(const DLTensor& tensor, size_t row)
{
    const int64_t stride = tensor.strides ? tensor.strides[0] : 1;
    return reinterpret_cast<const float*>(static_cast<const char*>(tensor.data) + tensor.byte_offset) +
           row * stride * tensor.dtype.lanes;
}

void collectPoseGroups(PhysicsRead& session,
                       ovphysx_query_target_t type,
                       const ovx_token_t* tokens,
                       std::vector<PoseColumns>& groups,
                       OvStageOutputResult& result)
{
    waitPhysics(session.instance, ovphysx_query(session.instance, type, OVPHYSX_SCOPE_ALL, &session.query, nullptr));
    if (!session.query)
        return;
    const ovx_token_or_string_t attributes[] = { { tokens[kPositionToken], {} },
                                                 { tokens[kOrientationToken], {} },
                                                 { tokens[kWorldScaleToken], {} } };
    waitPhysics(session.instance, ovphysx_read(session.instance, session.query, attributes, 3,
                                               ovstage_cuda_sync_t{ 0, 0 }, &session.read, nullptr));
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        waitPhysics(session.instance, ovphysx_fetch_read_next(session.instance, session.read, &group));
        if (!group)
            break;
        const ovstage_read_column_t& groupColumn = onlyColumn(*group);
        if (groupColumn.is_delete || !group->prims.count)
            continue;
        // worldScale has no array form, so an instancer contributes its position and orientation arrays.
        if (groupColumn.is_array)
        {
            ++result.instancerGroupsSkipped;
            continue;
        }
        static constexpr const ovstage_read_group_t* PoseColumns::*kMembers[] = { &PoseColumns::position,
                                                                                  &PoseColumns::orientation,
                                                                                  &PoseColumns::scale };
        const ovstage_read_group_t* PoseColumns::*member = nullptr;
        for (int token = kPositionToken; token <= kWorldScaleToken; ++token)
        {
            if (groupColumn.attribute == tokens[token])
                member = kMembers[token];
        }
        require(member != nullptr, "Unexpected physics output attribute");
        // The physics read emits every column of one body set under the same prim list.
        std::vector<PoseColumns>::iterator columns =
            std::find_if(groups.begin(), groups.end(), [&](const PoseColumns& value) {
                const ovstage_read_group_t* first =
                    value.position ? value.position : value.orientation ? value.orientation : value.scale;
                return value.type == type && first->prims.list == group->prims.list;
            });
        if (columns == groups.end())
        {
            groups.push_back({ nullptr, nullptr, nullptr, type, nullptr });
            columns = groups.end() - 1;
        }
        const ovstage_read_group_t*& column = (*columns).*member;
        require(!column, "Duplicate pose or scale column for a prim group");
        // Later fetches preserve this borrowed group and its CPU or CUDA tensor storage.
        // The caller keeps the read session alive until pose processing is finished.
        column = group;
    }
}

} // namespace

struct OvStageOutputCache::Impl
{
    ovphysx_instance_handle_t instance = 0;
    ovstage_instance_t* stage = nullptr;
    uint64_t attachment = 0;
    ovx_token_t tokens[kOutputTokenCount]{};
    std::vector<std::unique_ptr<CacheEntry>> entries;
    std::vector<ovx_primpath_t> paths;
};

OvStageOutputCache::OvStageOutputCache() : m_impl(new Impl)
{
}
OvStageOutputCache::~OvStageOutputCache() = default;

void OvStageOutputCache::refresh()
{
    m_impl->entries.clear();
}

OvStageOutputResult writeWorldTransformsToOvstage(ovphysx_instance_handle_t instance,
                                                  ovstage_instance_t* stage,
                                                  ovstage_ordinal_t outputOrdinal,
                                                  OvStageOutputCache* cache)
{
    OvStageOutputResult result;
    try
    {
        require(instance && stage, "An ovphysx instance and its attached stage are required");
        const std::shared_ptr<InstanceData> instanceData = get_instance(instance);
        require(instanceData != nullptr, "Invalid ovphysx instance");
        const uint64_t attachment = static_cast<uint64_t>(instanceData->attachHandle);
        require(attachment != 0, "The ovphysx instance has no attached stage");
        require(instanceData->ovstage_attach_payload.instance == stage,
                "The supplied stage is not attached to this ovphysx instance");
        OvStageOutputCache::Impl local;
        OvStageOutputCache::Impl& state = cache ? *cache->m_impl : local;
        path_dictionary_instance_t* dictionary = ovstage_get_path_dictionary(stage);
        require(dictionary != nullptr, "The stage has no path dictionary");
        if (state.attachment)
            require(state.instance == instance && state.stage == stage && state.attachment == attachment,
                    "Output cache cannot be reused with another instance, stage, or attachment");
        else
        {
            const ovx_string_t names[] = { literal_to_ovx_string(OVPHYSX_ATTR_POSITION),
                                           literal_to_ovx_string(OVPHYSX_ATTR_ORIENTATION),
                                           literal_to_ovx_string(OVPHYSX_ATTR_WORLD_SCALE),
                                           literal_to_ovx_string("omni:fabric:worldMatrix") };
            checkDictionary(dictionary, path_dictionary_create_tokens_from_strings(
                                            dictionary, names, kOutputTokenCount, state.tokens));
            state.instance = instance;
            state.stage = stage;
            state.attachment = attachment;
        }
        const ovstage_ordinal_t floor = writeFloor(stage, state.tokens[kWorldMatrixToken]);
        require(outputOrdinal > floor, "Output ordinal must be above the worldMatrix write floor");
        PhysicsRead rigid(instance);
        PhysicsRead links(instance);
        std::vector<PoseColumns> groups;
        collectPoseGroups(rigid, OVPHYSX_QUERY_TARGET_RIGID_BODY, state.tokens, groups, result);
        collectPoseGroups(links, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, state.tokens, groups, result);

        // Compose and validate every group before the first write, so invalid physics data publishes nothing.
        for (PoseColumns& group : groups)
        {
            require(group.position && group.orientation && group.scale,
                    "Missing position, orientation or worldScale column");
            const DLTensor& positions = columnTensor(*group.position, 3);
            const DLTensor& orientations = columnTensor(*group.orientation, 4);
            const DLTensor& scales = columnTensor(*group.scale, 3);
            for (const DLTensor* other : { &orientations, &scales })
                require(other->shape[0] == positions.shape[0] &&
                            other->device.device_type == positions.device.device_type &&
                            other->device.device_id == positions.device.device_id,
                        "Pose and scale columns disagree on rows or device");
            getPaths(dictionary, group.position->prims.list, state.paths);
            require(state.paths.size() == group.position->prims.count, "Pose column does not cover its full prim list");
            for (const std::unique_ptr<CacheEntry>& entry : state.entries)
            {
                if (entry->device.device_type == positions.device.device_type &&
                    entry->device.device_id == positions.device.device_id && entry->paths == state.paths)
                {
                    group.entry = entry.get();
                    break;
                }
            }
            if (!group.entry)
            {
                std::unique_ptr<CacheEntry> entry(new CacheEntry);
                entry->paths = state.paths;
                entry->device = positions.device;
                if (entry->device.device_type == kDLCUDA)
                {
                    std::string error;
                    entry->cuda.reset(new detail::OutputCudaBuffers);
                    if (!entry->cuda->initialize(entry->device.device_id, entry->paths.size(), error))
                        throw OutputFailure(OVPHYSX_API_ERROR, error);
                }
                else
                    entry->matrices.resize(entry->paths.size() * 16);
                group.entry = entry.get();
                state.entries.push_back(std::move(entry));
            }
            CacheEntry& entry = *group.entry;
            if (entry.cuda)
            {
                std::string error;
                if (!entry.cuda->compose(onlyColumn(*group.position), onlyColumn(*group.orientation),
                                         onlyColumn(*group.scale), error))
                    throw OutputFailure(OVPHYSX_API_ERROR, error);
            }
            else
            {
                for (size_t i = 0; i < entry.paths.size(); ++i)
                    require(detail::composeWorldMatrix(columnData(positions, i), columnData(orientations, i),
                                                       columnData(scales, i), entry.matrices.data() + i * 16),
                            "Pose or scale contains non-finite values, or a zero quaternion");
            }
        }
        for (const PoseColumns& group : groups)
        {
            CacheEntry& entry = *group.entry;
            // OVStage can invoke CUDA inline even when CPU data overwrites a GPU-backed column.
            detail::OutputCudaContextScope context(entry.cuda.get());
            if (!context.active())
                throw OutputFailure(OVPHYSX_API_ERROR, "Could not acquire the CUDA context for world-matrix publication");
            StageAccess access(stage);
            checkStage(stage, ovstage_query_from_path_list(stage, group.position->prims.list, &access.query));
            int64_t count = static_cast<int64_t>(entry.paths.size());
            DLTensor tensor{};
            tensor.data = entry.cuda ? entry.cuda->matrices() : entry.matrices.data();
            tensor.device = entry.device;
            tensor.ndim = 1;
            tensor.dtype = { kDLFloat, 64, 16 };
            tensor.shape = &count;
            ovstage_write_data_t data{};
            data.tensors = &tensor;
            data.tensor_count = 1;
            data.semantic = OVSTAGE_SEMANTIC_MATRIX;
            waitStage(stage, ovstage_write_attribute(stage, access.query, { state.tokens[kWorldMatrixToken], {} },
                                                     outputOrdinal, data, OVSTAGE_PRIM_MODE_UPSERT));
            result.matricesWritten += entry.paths.size();
            access.finish();
            std::string error;
            if (!context.restore(error))
                throw OutputFailure(OVPHYSX_API_ERROR, error);
        }
        links.finish();
        rigid.finish();
        if (result.matricesWritten)
        {
            // The CPU hierarchy model folds a direct world write into the local transform at its next
            // computation, against the parents of that time. Fold it now, so a later parent edit moves the body
            // with it. The GPU models would instead rederive the world matrices from the old local transforms.
            detail::OutputCudaContextScope context;
            if (!context.active())
                throw OutputFailure(OVPHYSX_API_ERROR, "Could not acquire the CUDA context for hierarchy computation");
            waitStage(stage, ovstage_compute_hierarchy(
                                 stage, OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU, outputOrdinal, outputOrdinal));
            std::string error;
            if (!context.restore(error))
                throw OutputFailure(OVPHYSX_API_ERROR, error);
        }
    }
    catch (const OutputFailure& error)
    {
        result.status = error.status;
        result.message = error.what();
    }
    catch (const std::exception& error)
    {
        result.status = OVPHYSX_API_ERROR;
        result.message = error.what();
    }
    return result;
}

} // namespace ovphysx::utils
