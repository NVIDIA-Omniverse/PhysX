// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

// Session read/write helpers for the C unit tests: one attribute of one object type, over the whole set or
// a pattern selection, keyed by prim path, on host or device memory. A fixed group contributes each row's
// components; an array group (one tensor per prim, e.g. per-axis joint columns) contributes each prim's
// tensor. Values travel as double whatever the column's dtype.

#include "cuda_test_helpers.h"
#include "test_utilities.h"

#include "ovphysx/ovphysx.h"

#include <ovstage/ovx_path_dictionary.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContextManager.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace session_test
{

using Column = std::map<std::string, std::vector<double>>;

// Expected scalar element type (code + bits) for readColumn's optional dtype check: the per-attribute C-API
// dtype contract the value-only read would otherwise miss (an index column regressed to float32 still decodes
// to the same doubles). Lanes/shape are representation-dependent (array columns pack width in lanes) and not checked.
inline constexpr DLDataType kDtypeFloat32{ kDLFloat, 32, 1 };
inline constexpr DLDataType kDtypeInt32{ kDLInt, 32, 1 };
inline constexpr DLDataType kDtypeUInt8{ kDLUInt, 8, 1 };

// The CUDA context of the simulation owning the scene, actor, articulation or link at `path`, needed to copy
// a DirectGPU column; 0 for a CPU scene or an unknown object.
inline uintptr_t cudaContextOf(ovphysx_instance_handle_t handle, const char* path, ovphysx_physx_type_t type)
{
    ovphysx_query_target_t objectType = OVPHYSX_QUERY_TARGET_COUNT;
    if (type == OVPHYSX_PHYSX_TYPE_SCENE)
        objectType = OVPHYSX_QUERY_TARGET_SCENE;
    else if (type == OVPHYSX_PHYSX_TYPE_ACTOR)
        objectType = OVPHYSX_QUERY_TARGET_RIGID_ACTOR;
    else if (type == OVPHYSX_PHYSX_TYPE_LINK)
        objectType = OVPHYSX_QUERY_TARGET_ARTICULATION_LINK;
    else if (type == OVPHYSX_PHYSX_TYPE_ARTICULATION)
        objectType = OVPHYSX_QUERY_TARGET_ARTICULATION;
    else
        return 0;
    void* pointer = test_utils::read_physx_pointer(handle, objectType, path);
    if (!pointer)
        return 0;
    physx::PxScene* scene = nullptr;
    if (type == OVPHYSX_PHYSX_TYPE_SCENE)
        scene = static_cast<physx::PxScene*>(pointer);
    else if (type == OVPHYSX_PHYSX_TYPE_ACTOR || type == OVPHYSX_PHYSX_TYPE_LINK)
        scene = static_cast<physx::PxActor*>(pointer)->getScene();
    else if (type == OVPHYSX_PHYSX_TYPE_ARTICULATION)
        scene = static_cast<physx::PxArticulationReducedCoordinate*>(pointer)->getScene();
    physx::PxCudaContextManager* manager = scene ? scene->getCudaContextManager() : nullptr;
    return manager ? reinterpret_cast<uintptr_t>(manager->getContext()) : 0;
}

namespace detail
{

inline size_t componentBytes(const DLDataType& dtype)
{
    return static_cast<size_t>(dtype.bits) / 8u;
}

// Widen one component to double. The session read/write path emits exactly float32, uint8 and int32 (the
// ovphysx_types.h dtype contract); any other dtype is a regression the double-normalizing helpers would
// otherwise hide, so fail the test loudly rather than decode it wrong.
inline double decode(const DLDataType& dtype, const unsigned char* p)
{
    if (dtype.code == kDLFloat && dtype.bits == 32)
    {
        float v;
        std::memcpy(&v, p, sizeof(v));
        return v;
    }
    if (dtype.code == kDLInt && dtype.bits == 32)
    {
        int32_t v;
        std::memcpy(&v, p, sizeof(v));
        return static_cast<double>(v);
    }
    if (dtype.code == kDLUInt && dtype.bits == 8)
        return *p;
    ADD_FAILURE() << "session_test::decode: unsupported column dtype code=" << int(dtype.code)
                  << " bits=" << int(dtype.bits) << " (expected float32, int32 or uint8)";
    return 0.0;
}

inline void encode(const DLDataType& dtype, double value, unsigned char* p)
{
    if (dtype.code == kDLFloat && dtype.bits == 32)
    {
        const float v = static_cast<float>(value);
        std::memcpy(p, &v, sizeof(v));
        return;
    }
    if (dtype.code == kDLInt && dtype.bits == 32)
    {
        const int32_t v = static_cast<int32_t>(value);
        std::memcpy(p, &v, sizeof(v));
        return;
    }
    if (dtype.code == kDLUInt && dtype.bits == 8)
    {
        *p = static_cast<unsigned char>(value);
        return;
    }
    ADD_FAILURE() << "session_test::encode: unsupported column dtype code=" << int(dtype.code)
                  << " bits=" << int(dtype.bits) << " (expected float32, int32 or uint8)";
}

// Components per row of a fixed column (every dimension after the first, times the lanes).
inline size_t rowComponents(const DLTensor& tensor)
{
    size_t components = tensor.dtype.lanes ? tensor.dtype.lanes : 1;
    for (int d = 1; d < tensor.ndim; ++d)
        components *= static_cast<size_t>(tensor.shape[d]);
    return components;
}

inline size_t elementCount(const DLTensor& tensor)
{
    size_t count = tensor.dtype.lanes ? tensor.dtype.lanes : 1;
    for (int d = 0; d < tensor.ndim; ++d)
        count *= static_cast<size_t>(tensor.shape[d]);
    return count;
}

// The tensor's bytes on the host; a device tensor is copied under the simulation context.
inline bool hostBytes(const DLTensor& tensor, const ovstage_cuda_sync_t& sync, uintptr_t cudaContext,
                      std::vector<unsigned char>& out)
{
    out.resize(elementCount(tensor) * componentBytes(tensor.dtype));
    const unsigned char* source = static_cast<const unsigned char*>(tensor.data) + tensor.byte_offset;
    if (out.empty())
        return true;
    if (tensor.device.device_type == kDLCPU)
    {
        std::memcpy(out.data(), source, out.size());
        return true;
    }
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ovphysx::test_cuda::ScopedCudaContextPush context(cuda, cudaContext);
    if (!context.ok())
        return false;
    if (sync.wait_event && !cuda->eventSynchronize(sync.wait_event, nullptr))
        return false;
    if (sync.stream && !cuda->streamSynchronize(sync.stream, nullptr))
        return false;
    return cuda->memcpyDtoH(out.data(), reinterpret_cast<uintptr_t>(source), out.size(), nullptr);
}

inline bool deviceUpload(const DLTensor& tensor, const std::vector<unsigned char>& bytes, uintptr_t cudaContext)
{
    unsigned char* destination = static_cast<unsigned char*>(tensor.data) + tensor.byte_offset;
    if (bytes.empty())
        return true;
    if (tensor.device.device_type == kDLCPU)
    {
        std::memcpy(destination, bytes.data(), bytes.size());
        return true;
    }
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ovphysx::test_cuda::ScopedCudaContextPush context(cuda, cudaContext);
    return context.ok() && cuda->memcpyHtoD(reinterpret_cast<uintptr_t>(destination), bytes.data(), bytes.size(), nullptr);
}

inline ovx_string_or_token_t attributeName(const char* attribute)
{
    ovx_string_or_token_t name{};
    name.string = { attribute, std::strlen(attribute) };
    return name;
}

// Opens a query over `type`: the whole set, or the objects `pattern` selects.
inline bool openQuery(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, const char* pattern,
                      ovphysx_query_handle_t& query)
{
    query = 0;
    if (!pattern)
        return test_utils::complete_task(handle, ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)).status ==
                   OVPHYSX_API_SUCCESS &&
               query != 0;
    ovphysx_query_selection_t selection{};
    selection.pattern = ovphysx_cstr(pattern);
    return test_utils::complete_task(handle, ovphysx_query_selection(handle, type, &selection, &query, nullptr)).status ==
               OVPHYSX_API_SUCCESS &&
           query != 0;
}

// The prim path of row `row` of a group.
inline bool rowPath(ovx_path_dictionary_t* dictionary, const ovstage_prim_group_t& prims, uint32_t row,
                    std::string& out)
{
    const ovx_primpath_t* paths = nullptr;
    size_t count = 0;
    if (ovx_path_dictionary_get_paths(dictionary, prims.list, &paths, &count) != OVX_OK || !paths)
        return false;
    const uint32_t index = prims.index_map ? prims.index_map[row] : prims.offset + row;
    ovx_string_t text{};
    if (index >= count || ovx_path_dictionary_path_to_string(dictionary, paths[index], &text) != OVX_OK)
        return false;
    out.assign(text.ptr, text.length);
    return true;
}

} // namespace detail

// Reads `attribute` of the `type` objects (every one, or those `pattern` selects). `cudaContext` is the
// simulation's context for a DirectGPU scene (see cudaContextOf), 0 on CPU. Returns false on any failure.
inline bool readColumn(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, const char* attribute,
                       Column& out, const char* pattern = nullptr, uintptr_t cudaContext = 0,
                       const DLDataType* expectedDtype = nullptr)
{
    out.clear();
    ovphysx_query_handle_t query = 0;
    if (!detail::openQuery(handle, type, pattern, query))
        return false;
    path_dictionary_instance_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    bool ok = dictionary != nullptr;
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictionary);
    const ovx_string_or_token_t name = detail::attributeName(attribute);
    ovphysx_read_handle_t read = 0;
    ok = ok && test_utils::complete_task(handle, ovphysx_read(handle, query, &name, 1, &read, nullptr)).status ==
                   OVPHYSX_API_SUCCESS;
    std::vector<unsigned char> bytes;
    while (ok)
    {
        const ovstage_read_group_t* group = nullptr;
        ok = test_utils::complete_task(handle, ovphysx_fetch_read_next(handle, read, &group)).status ==
             OVPHYSX_API_SUCCESS;
        if (!ok || !group)
            break;
        for (uint32_t row = 0; ok && !group->is_delete && row < group->prims.count; ++row)
        {
            std::string path;
            ok = detail::rowPath(dict, group->prims, row, path);
            if (!ok)
                break;
            std::vector<double>& values = out[path];
            values.clear();
            if (group->is_array)
            {
                const uint32_t index = group->data.index_map ? group->data.index_map[row] : row;
                ok = index < group->data.tensor_count;
                if (!ok)
                    break;
                const DLTensor& tensor = group->data.tensors[index];
                if (expectedDtype)
                {
                    EXPECT_EQ(tensor.dtype.code, expectedDtype->code) << attribute << " dtype.code";
                    EXPECT_EQ(tensor.dtype.bits, expectedDtype->bits) << attribute << " dtype.bits";
                }
                ok = ok && detail::hostBytes(tensor, group->data.cuda_sync, cudaContext, bytes);
                const size_t width = detail::componentBytes(tensor.dtype);
                for (size_t i = 0; ok && i < bytes.size() / width; ++i)
                    values.push_back(detail::decode(tensor.dtype, bytes.data() + i * width));
            }
            else
            {
                ok = group->data.tensor_count >= 1;
                const DLTensor& tensor = group->data.tensors[0];
                if (ok && row == 0 && expectedDtype)
                {
                    EXPECT_EQ(tensor.dtype.code, expectedDtype->code) << attribute << " dtype.code";
                    EXPECT_EQ(tensor.dtype.bits, expectedDtype->bits) << attribute << " dtype.bits";
                }
                if (ok && row == 0)
                    ok = detail::hostBytes(tensor, group->data.cuda_sync, cudaContext, bytes);
                const size_t components = detail::rowComponents(tensor);
                const size_t width = detail::componentBytes(tensor.dtype);
                const size_t dataRow = group->data.index_map ? group->data.index_map[row] : row;
                for (size_t i = 0; ok && i < components; ++i)
                {
                    const size_t offset = (dataRow * components + i) * width;
                    ok = offset + width <= bytes.size();
                    if (ok)
                        values.push_back(detail::decode(tensor.dtype, bytes.data() + offset));
                }
            }
        }
        ok = test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)).status ==
                 OVPHYSX_API_SUCCESS &&
             ok;
    }
    if (read != 0)
        ok = test_utils::complete_task(handle, ovphysx_release_read(handle, read)).status == OVPHYSX_API_SUCCESS && ok;
    ok = test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status == OVPHYSX_API_SUCCESS && ok;
    return ok;
}

// Fills one prim's values for a write: `values` is the prim's slice of the write buffer, sized to its component
// count. It does not hold the column's current value (read first to write relative to it), so the callback sets
// every component.
using RowFill = std::function<void(const std::string& path, std::vector<double>& values)>;

// Writes `attribute` of the `type` objects (every one, or those `pattern` selects), committing every group.
// Returns the number of prims written, or -1 on failure.
inline int writeColumn(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, const char* attribute,
                       const RowFill& fill, const char* pattern = nullptr, uintptr_t cudaContext = 0)
{
    ovphysx_query_handle_t query = 0;
    if (!detail::openQuery(handle, type, pattern, query))
        return -1;
    path_dictionary_instance_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    bool ok = dictionary != nullptr;
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictionary);
    const ovx_string_or_token_t name = detail::attributeName(attribute);
    ovphysx_write_handle_t write = 0;
    ok = ok && test_utils::complete_task(handle, ovphysx_write(handle, query, &name, 1, &write, nullptr)).status ==
                   OVPHYSX_API_SUCCESS;
    int written = 0;
    std::vector<unsigned char> bytes;
    std::vector<double> values;
    while (ok)
    {
        ovphysx_fetched_write_group_t fetched{};
        ok = test_utils::complete_task(handle, ovphysx_fetch_write_next(handle, write, &fetched)).status ==
             OVPHYSX_API_SUCCESS;
        const ovstage_map_group_t* group = fetched.group;
        if (!ok || !group)
            break;
        const uint32_t tensors = group->data.tensor_count;
        // One tensor per prim (array transport) versus one stacked tensor; a single prim reads the same
        // either way.
        const bool perPrim = tensors > 1 || (tensors == 1 && group->prims.count == 1);
        if (perPrim)
        {
            for (uint32_t row = 0; ok && row < group->prims.count; ++row)
            {
                std::string path;
                const uint32_t index = group->data.index_map ? group->data.index_map[row] : row;
                ok = index < tensors;
                if (!ok)
                    break;
                const DLTensor& tensor = group->data.tensors[index];
                ok = detail::rowPath(dict, group->prims, row, path) &&
                     detail::hostBytes(tensor, ovstage_cuda_sync_t{}, cudaContext, bytes);
                const size_t width = detail::componentBytes(tensor.dtype);
                values.assign(bytes.size() / width, 0.0);
                for (size_t i = 0; ok && i < values.size(); ++i)
                    values[i] = detail::decode(tensor.dtype, bytes.data() + i * width);
                if (ok)
                    fill(path, values);
                for (size_t i = 0; ok && i < values.size(); ++i)
                    detail::encode(tensor.dtype, values[i], bytes.data() + i * width);
                ok = ok && detail::deviceUpload(tensor, bytes, cudaContext);
                written += ok ? 1 : 0;
            }
        }
        else if (tensors >= 1)
        {
            const DLTensor& tensor = group->data.tensors[0];
            ok = detail::hostBytes(tensor, ovstage_cuda_sync_t{}, cudaContext, bytes);
            const size_t components = detail::rowComponents(tensor);
            const size_t width = detail::componentBytes(tensor.dtype);
            for (uint32_t row = 0; ok && row < group->prims.count; ++row)
            {
                std::string path;
                ok = detail::rowPath(dict, group->prims, row, path);
                const size_t dataRow = group->data.index_map ? group->data.index_map[row] : row;
                values.assign(components, 0.0);
                for (size_t i = 0; ok && i < components; ++i)
                {
                    const size_t offset = (dataRow * components + i) * width;
                    ok = offset + width <= bytes.size();
                    if (ok)
                        values[i] = detail::decode(tensor.dtype, bytes.data() + offset);
                }
                if (ok)
                    fill(path, values);
                for (size_t i = 0; ok && i < components; ++i)
                    detail::encode(tensor.dtype, values[i], bytes.data() + (dataRow * components + i) * width);
                written += ok ? 1 : 0;
            }
            ok = ok && detail::deviceUpload(tensor, bytes, cudaContext);
        }
        ok = ok && test_utils::complete_task(handle, ovphysx_commit_group(handle, write, group, ovstage_cuda_sync_t{}))
                           .status == OVPHYSX_API_SUCCESS;
    }
    if (write != 0)
        ok = test_utils::complete_task(handle, ovphysx_release_write(handle, write)).status == OVPHYSX_API_SUCCESS &&
             ok;
    ok = test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status == OVPHYSX_API_SUCCESS && ok;
    return ok ? written : -1;
}

// Writes the same values to every selected prim (each prim gets `values`, truncated or zero-padded to its
// component count).
inline int writeUniform(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, const char* attribute,
                        const std::vector<double>& values, const char* pattern = nullptr, uintptr_t cudaContext = 0)
{
    return writeColumn(
        handle, type, attribute,
        [&](const std::string&, std::vector<double>& row)
        {
            for (size_t i = 0; i < row.size(); ++i)
                row[i] = i < values.size() ? values[i] : 0.0;
        },
        pattern, cudaContext);
}

// The completion status of opening a write of `attribute` over the `type` objects (every one, or those
// `pattern` selects): OVPHYSX_API_SUCCESS when the attribute is writable there.
inline ovphysx_api_status_t writeOpenStatus(ovphysx_instance_handle_t handle, ovphysx_query_target_t type,
                                            const char* attribute, const char* pattern = nullptr)
{
    ovphysx_query_handle_t query = 0;
    if (!detail::openQuery(handle, type, pattern, query))
        return OVPHYSX_API_ERROR;
    const ovx_string_or_token_t name = detail::attributeName(attribute);
    ovphysx_write_handle_t write = 0;
    const ovphysx_api_status_t status =
        test_utils::complete_task(handle, ovphysx_write(handle, query, &name, 1, &write, nullptr)).status;
    if (write != 0)
        test_utils::complete_task(handle, ovphysx_release_write(handle, write));
    test_utils::complete_task(handle, ovphysx_release_query(handle, query));
    return status;
}

// Advances the simulation `steps` times by `dt`, waiting on each step.
inline bool stepN(ovphysx_instance_handle_t handle, int steps, float dt = 1.0f / 60.0f)
{
    for (int i = 0; i < steps; ++i)
        if (ovphysx_step_sync(handle, dt).status != OVPHYSX_API_SUCCESS)
            return false;
    return true;
}

} // namespace session_test
