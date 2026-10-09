// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

#include <ovphysx/experimental/OvStageOutput.hpp>

#include "OvStageOutputCuda.h"
#include "OvStageOutputMath.h"
#include "internal/sdk/ovphysxSDK.hpp"

#include <common/foundation/MatrixTools.h>
#include <ovstage/ovstage_api/ovstage_api.h>
#include <ovx/path_dictionary/path_dictionary.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>
#include <unordered_map>
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
    ovstage_query_handle_t query = 0;
    ovstage_read_handle_t read = 0;

    explicit StageAccess(ovstage_instance_t* value) : stage(value)
    {
    }
    ~StageAccess()
    {
        if (read)
            cleanupStage(stage, ovstage_release_read(stage, read));
        if (query)
            cleanupStage(stage, ovstage_release_query(stage, query));
    }
    void finish()
    {
        if (read)
            waitStage(stage, ovstage_release_read(stage, std::exchange(read, 0)));
        if (query)
            waitStage(stage, ovstage_release_query(stage, std::exchange(query, 0)));
    }
};

struct StageGroup
{
    ovstage_instance_t* stage;
    ovstage_read_group_t value{};
    ~StageGroup()
    {
        if (value.read_group_id)
            ovstage_release_group(stage, &value);
    }
};

struct PhysicsRead
{
    ovphysx_handle_t instance;
    ovphysx_query_handle_t query = 0;
    ovphysx_read_handle_t read = 0;

    explicit PhysicsRead(ovphysx_handle_t value) : instance(value)
    {
    }
    ~PhysicsRead()
    {
        if (read)
            ovphysx_release_read(instance, read);
        if (query)
            ovphysx_release_query(instance, query);
    }
    void finish()
    {
        if (read)
            checkPhysics(ovphysx_release_read(instance, std::exchange(read, 0)));
        if (query)
            checkPhysics(ovphysx_release_query(instance, std::exchange(query, 0)));
    }
};

struct CacheEntry
{
    std::vector<ovx_primpath_t> paths;
    DLDevice device{};
    std::vector<double> scales;
    std::vector<double> matrices;
    std::unique_ptr<detail::OutputCudaBuffers> cuda;
};

struct PosePair
{
    const ovstage_read_group_t* position = nullptr;
    const ovstage_read_group_t* orientation = nullptr;
    ovphysx_sim_object_type_t type;
    CacheEntry* entry = nullptr;
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
    ovstage_ordinal_query_handle_t query = 0;
    ovstage_ordinal_t floor = 0;
    try
    {
        waitStage(stage, ovstage_get_attribute_write_floor(stage, { attribute, {} }, &query));
        checkStage(stage, ovstage_fetch_ordinal(stage, query, OVSTAGE_TIMEOUT_INFINITE, &floor));
        waitStage(stage, ovstage_release_ordinal_query(stage, std::exchange(query, 0)));
    }
    catch (...)
    {
        if (query)
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

const DLTensor& poseTensor(const ovstage_read_group_t& group, uint16_t lanes)
{
    require(!group.is_array && !group.is_delete && group.prims.list && group.prims.count,
            "Expected a nonempty fixed pose column");
    require(!group.prims.offset && !group.prims.index_map && !group.data.index_map && !group.data.mask,
            "Fixed pose columns must have dense prim and data coverage");
    require(group.data.tensor_count == 1 && group.data.tensors, "Expected one fixed pose tensor");
    const DLTensor& tensor = group.data.tensors[0];
    require(tensor.data && tensor.ndim == 1 && tensor.shape && tensor.shape[0] == group.prims.count &&
                tensor.dtype.code == kDLFloat && tensor.dtype.bits == 32 && tensor.dtype.lanes == lanes &&
                (!tensor.strides || tensor.strides[0] > 0),
            "Invalid fixed pose tensor layout");
    validateExtent(tensor);
    require(tensor.device.device_type == kDLCPU || tensor.device.device_type == kDLCUDA,
            "Pose columns require CPU or CUDA storage");
    return tensor;
}

void gather(PhysicsRead& session,
            ovphysx_sim_object_type_t type,
            ovx_token_t positionToken,
            ovx_token_t orientationToken,
            std::vector<PosePair>& pairs,
            OvStageOutputResult& result)
{
    checkPhysics(ovphysx_query(session.instance, type, OVPHYSX_SCOPE_ALL, &session.query));
    if (!session.query)
        return;
    const ovx_string_or_token_t attributes[] = { { positionToken, {} }, { orientationToken, {} } };
    checkPhysics(ovphysx_read(session.instance, session.query, attributes, 2, &session.read));
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t fetch = ovphysx_fetch_read_next(session.instance, session.read, &group);
        if (fetch.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        checkPhysics(fetch);
        require(group != nullptr, "Missing physics output group");
        if (group->is_delete || !group->prims.count)
            continue;
        if (group->is_array)
        {
            ++result.instancerGroupsSkipped;
            continue;
        }
        require(group->attribute == positionToken || group->attribute == orientationToken,
                "Unexpected physics output attribute");
        std::vector<PosePair>::iterator pair = std::find_if(pairs.begin(), pairs.end(), [&](const PosePair& value) {
            const ovstage_read_group_t* first = value.position ? value.position : value.orientation;
            return value.type == type && first->prims.list == group->prims.list;
        });
        if (pair == pairs.end())
        {
            pairs.push_back({ nullptr, nullptr, type, nullptr });
            pair = pairs.end() - 1;
        }
        const ovstage_read_group_t*& column = group->attribute == positionToken ? pair->position : pair->orientation;
        require(!column, "Duplicate pose column for a prim group");
        column = group;
    }
}

void captureScales(ovstage_instance_t* stage,
                   path_dictionary_instance_t* dictionary,
                   ovx_token_t matrixToken,
                   ovstage_ordinal_t floor,
                   ovx_primpath_list_t paths,
                   CacheEntry& entry)
{
    StageAccess access(stage);
    checkStage(stage, ovstage_query_from_path_list(stage, paths, &access.query));
    waitStage(stage, ovstage_read_attributes(stage, access.query, &matrixToken, 1, { 0, floor, false }, &access.read));
    std::vector<bool> found(entry.paths.size(), false);
    std::unordered_map<ovx_primpath_t, size_t> rows;
    rows.reserve(entry.paths.size());
    for (size_t i = 0; i < entry.paths.size(); ++i)
        rows.emplace(entry.paths[i], i);
    std::vector<ovx_primpath_t> groupPaths;
    entry.scales.resize(entry.paths.size() * 3);
    for (;;)
    {
        StageGroup group{ stage, {} };
        const ovstage_api_status_t status =
            ovstage_fetch_read_next(stage, access.read, OVSTAGE_TIMEOUT_INFINITE, &group.value);
        if (status == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        checkStage(stage, status);
        const ovstage_read_group_t& value = group.value;
        if (value.is_delete || !value.prims.count)
            continue;
        require(!value.is_array && value.data.tensor_count == 1 && value.data.tensors,
                "worldMatrix must be a fixed matrix column");
        const DLTensor& tensor = value.data.tensors[0];
        require(tensor.data && tensor.ndim == 1 && tensor.shape && tensor.shape[0] > 0 && tensor.dtype.code == kDLFloat &&
                    tensor.dtype.bits == 64 && tensor.dtype.lanes == 16 && (!tensor.strides || tensor.strides[0] > 0),
                "worldMatrix must contain float64 matrices with 16 lanes");
        validateExtent(tensor);
        getPaths(dictionary, value.prims.list, groupPaths);
        // The public OVStage read API returns CPU tensors, including GPU-backed columns.
        require(tensor.device.device_type == kDLCPU, "Expected a CPU worldMatrix read");
        const double* data = reinterpret_cast<const double*>(static_cast<const char*>(tensor.data) + tensor.byte_offset);
        const int64_t stride = tensor.strides ? tensor.strides[0] : 1;
        for (size_t i = 0; i < value.prims.count; ++i)
        {
            if (value.data.mask && !(value.data.mask[i / 64] & (uint64_t(1) << (i % 64))))
                continue;
            const size_t pathIndex = value.prims.index_map ? value.prims.index_map[i] : value.prims.offset + i;
            const size_t row = value.data.index_map ? value.data.index_map[i] : i;
            require(pathIndex < groupPaths.size() && row < static_cast<size_t>(tensor.shape[0]),
                    "Invalid worldMatrix row coverage");
            const std::unordered_map<ovx_primpath_t, size_t>::const_iterator target = rows.find(groupPaths[pathIndex]);
            require(target != rows.end(), "Unexpected worldMatrix prim");
            const size_t index = target->second;
            require(!found[index], "Duplicate worldMatrix prim");
            const double* matrix = data + row * stride * 16;
            for (size_t component = 0; component < 16; ++component)
                require(std::isfinite(matrix[component]), "worldMatrix contains a non-finite value");
            ::physx::PxTransform pose;
            ::physx::PxVec3 scale;
            omni::physx::decomposeMatrix(pose, scale, matrix);
            entry.scales[index * 3] = scale.x;
            entry.scales[index * 3 + 1] = scale.y;
            entry.scales[index * 3 + 2] = scale.z;
            found[index] = true;
        }
    }
    require(std::all_of(found.begin(), found.end(), [](bool value) { return value; }),
            "Missing worldMatrix; compute and seal the current world transforms before publishing physics output");
    access.finish();
}

} // namespace

struct OvStageOutputCache::Impl
{
    ovphysx_handle_t instance = 0;
    ovstage_instance_t* stage = nullptr;
    uint64_t attachment = 0;
    ovx_token_t tokens[3]{};
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

OvStageOutputResult writeWorldTransformsToOvstage(ovphysx_handle_t instance,
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
                                           literal_to_ovx_string("omni:fabric:worldMatrix") };
            checkDictionary(dictionary, path_dictionary_create_tokens_from_strings(dictionary, names, 3, state.tokens));
            state.instance = instance;
            state.stage = stage;
            state.attachment = attachment;
        }
        const ovstage_ordinal_t floor = writeFloor(stage, state.tokens[2]);
        require(outputOrdinal > floor, "Output ordinal must be above the worldMatrix write floor");
        PhysicsRead rigid(instance);
        PhysicsRead links(instance);
        std::vector<PosePair> pairs;
        gather(rigid, OVPHYSX_OBJECT_RIGID_BODY, state.tokens[0], state.tokens[1], pairs, result);
        gather(links, OVPHYSX_OBJECT_ARTICULATION_LINK, state.tokens[0], state.tokens[1], pairs, result);

        // Capture every missing scale and validate every pose before the first write.
        // Stage read views cannot remain live while writes address the same rows.
        for (PosePair& pair : pairs)
        {
            require(pair.position && pair.orientation, "Missing position or orientation column");
            const DLTensor& positions = poseTensor(*pair.position, 3);
            const DLTensor& orientations = poseTensor(*pair.orientation, 4);
            require(positions.shape[0] == orientations.shape[0] &&
                        positions.device.device_type == orientations.device.device_type &&
                        positions.device.device_id == orientations.device.device_id,
                    "Pose columns disagree on rows or device");
            getPaths(dictionary, pair.position->prims.list, state.paths);
            require(state.paths.size() == pair.position->prims.count, "Pose column does not cover its full prim list");
            for (const std::unique_ptr<CacheEntry>& entry : state.entries)
            {
                if (entry->device.device_type == positions.device.device_type &&
                    entry->device.device_id == positions.device.device_id && entry->paths == state.paths)
                {
                    pair.entry = entry.get();
                    break;
                }
            }
            if (!pair.entry)
            {
                std::unique_ptr<CacheEntry> entry(new CacheEntry);
                entry->paths = state.paths;
                entry->device = positions.device;
                // Reading a previously published GPU column can also run inline.
                detail::OutputCudaContextScope captureContext;
                if (!captureContext.active())
                    throw OutputFailure(OVPHYSX_API_ERROR, "Could not acquire the CUDA context for world-scale capture");
                std::string error;
                captureScales(stage, dictionary, state.tokens[2], floor, pair.position->prims.list, *entry);
                if (entry->device.device_type == kDLCUDA)
                {
                    entry->cuda.reset(new detail::OutputCudaBuffers);
                    if (!entry->cuda->initialize(
                            entry->device.device_id, entry->paths.size(), entry->scales.data(), error))
                        throw OutputFailure(OVPHYSX_API_ERROR, error);
                }
                else
                    entry->matrices.resize(entry->paths.size() * 16);
                if (!captureContext.restore(error))
                    throw OutputFailure(OVPHYSX_API_ERROR, error);
                pair.entry = entry.get();
                state.entries.push_back(std::move(entry));
            }
            CacheEntry& entry = *pair.entry;
            if (entry.cuda)
            {
                std::string error;
                if (!entry.cuda->compose(positions, orientations, pair.position->data.cuda_sync,
                                         pair.orientation->data.cuda_sync, error))
                    throw OutputFailure(OVPHYSX_API_ERROR, error);
            }
            else
            {
                const float* positionData =
                    reinterpret_cast<const float*>(static_cast<const char*>(positions.data) + positions.byte_offset);
                const float* orientationData = reinterpret_cast<const float*>(
                    static_cast<const char*>(orientations.data) + orientations.byte_offset);
                const int64_t positionStride = positions.strides ? positions.strides[0] : 1;
                const int64_t orientationStride = orientations.strides ? orientations.strides[0] : 1;
                for (size_t i = 0; i < entry.paths.size(); ++i)
                    require(detail::composeWorldMatrix(positionData + i * positionStride * 3,
                                                       orientationData + i * orientationStride * 4,
                                                       entry.scales.data() + i * 3, entry.matrices.data() + i * 16),
                            "Pose contains non-finite values or a zero quaternion");
            }
        }
        for (const PosePair& pair : pairs)
        {
            CacheEntry& entry = *pair.entry;
            // OVStage can execute GPU writes inline on this thread.
            detail::OutputCudaContextScope context(entry.cuda.get());
            if (!context.active())
                throw OutputFailure(OVPHYSX_API_ERROR, "Could not acquire the CUDA context for world-matrix publication");
            StageAccess access(stage);
            checkStage(stage, ovstage_query_from_path_list(stage, pair.position->prims.list, &access.query));
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
            waitStage(stage, ovstage_write_attribute(stage, access.query, { state.tokens[2], {} }, outputOrdinal, data,
                                                     OVSTAGE_PRIM_MODE_UPSERT));
            result.matricesWritten += entry.paths.size();
            access.finish();
            std::string error;
            if (!context.restore(error))
                throw OutputFailure(OVPHYSX_API_ERROR, error);
        }
        links.finish();
        rigid.finish();
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
