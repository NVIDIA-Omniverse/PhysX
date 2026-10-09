// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6
 */

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/experimental/OvStageOutput.hpp>
#include <ovphysx/ovphysx_config.h>
#include <ovstage/ovx_path_dictionary.h>
#include <common/foundation/MatrixTools.h>
#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContextManager.h>

#include <array>
#include <cmath>
#include <cstring>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#if OVPHYSX_ENABLE_GPU_TESTS
#    if defined(_WIN32)
#        include <windows.h>
#    else
#        include <dlfcn.h>
#    endif
#endif

namespace
{
using Matrix = std::array<double, 16>;
using Matrices = std::map<std::string, Matrix>;
using Poses = std::map<std::string, physx::PxTransform>;
using ovphysx::utils::OvStageOutputCache;
using ovphysx::utils::OvStageOutputResult;
using ovphysx::utils::writeWorldTransformsToOvstage;

constexpr const char* kWorldMatrix = "omni:fabric:worldMatrix";

ovx_string_or_token_t attribute(const char* name)
{
    return { 0, { name, std::strlen(name) } };
}

bool complete(ovstage_instance_t* stage, ovstage_enqueue_result_t enqueue)
{
    if (enqueue.status != OVSTAGE_OK)
        return false;
    if (enqueue.op_index == OVSTAGE_INVALID_OP_ID)
        return true;
    ovstage_op_wait_result_t wait{};
    const ovstage_api_status_t status = ovstage_wait_op(stage, enqueue.op_index, OVSTAGE_TIMEOUT_INFINITE, &wait);
    const bool ok = status == OVSTAGE_OK && wait.error_op_id_count == 0;
    if (!ok)
    {
        for (size_t i = 0; i < wait.error_op_id_count; ++i)
        {
            const ovx_string_t error = ovstage_get_last_op_error(stage, wait.error_op_ids[i]);
            ADD_FAILURE() << std::string(error.ptr ? error.ptr : "", error.length);
        }
    }
    return ovstage_release_op(stage, enqueue.op_index) == OVSTAGE_OK && ok;
}

bool seal(ovstage_instance_t* stage, ovstage_ordinal_t ordinal)
{
    ovstage_write_floor_desc_t floor{};
    floor.ordinal = ordinal;
    floor.scope = OVSTAGE_SCOPE_ALL;
    return complete(stage, ovstage_advance_write_floor(stage, &floor));
}

ovstage_ordinal_t writeFloor(ovstage_instance_t* stage)
{
    ovstage_ordinal_query_handle_t query = 0;
    EXPECT_TRUE(complete(stage, ovstage_get_attribute_write_floor(stage, {}, &query)));
    ovstage_ordinal_t ordinal = 0;
    EXPECT_EQ(ovstage_fetch_ordinal(stage, query, OVSTAGE_TIMEOUT_INFINITE, &ordinal), OVSTAGE_OK);
    EXPECT_TRUE(complete(stage, ovstage_release_ordinal_query(stage, query)));
    return ordinal;
}

ovstage_instance_t* attachedStage(ovphysx_handle_t handle)
{
    std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
    return test_utils::ovstage_test_attachments().at(handle).back().stage;
}

struct StageQuery
{
    ovstage_instance_t* stage;
    ovstage_query_handle_t query = 0;
    ovstage_read_handle_t read = 0;

    StageQuery(ovstage_instance_t* value, const std::vector<std::string>& paths) : stage(value)
    {
        std::vector<ovx_string_t> names;
        for (const std::string& path : paths)
            names.push_back({ path.data(), path.size() });
        ovx_primpath_list_t list = 0;
        ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
        EXPECT_EQ(
            ovx_path_dictionary_create_path_list_from_strings(dictionary, names.data(), names.size(), &list), OVX_OK);
        EXPECT_EQ(ovstage_query_from_path_list(stage, list, &query), OVSTAGE_OK);
        EXPECT_EQ(ovx_path_dictionary_destroy_path_list(dictionary, list), OVX_OK);
    }

    ~StageQuery()
    {
        if (read)
        {
            EXPECT_TRUE(complete(stage, ovstage_release_read(stage, read)));
        }
        if (query)
        {
            EXPECT_TRUE(complete(stage, ovstage_release_query(stage, query)));
        }
    }
};

std::vector<std::string> groupPaths(ovx_path_dictionary_t* dictionary, const ovstage_read_group_t& group)
{
    const ovx_primpath_t* paths = nullptr;
    size_t count = 0;
    EXPECT_EQ(ovx_path_dictionary_get_paths(dictionary, group.prims.list, &paths, &count), OVX_OK);
    std::vector<std::string> result;
    for (uint32_t row = 0; row < group.prims.count; ++row)
    {
        const size_t index = group.prims.index_map ? group.prims.index_map[row] : group.prims.offset + row;
        if (!paths || index >= count)
        {
            ADD_FAILURE() << "Invalid prim coverage in test read";
            break;
        }
        ovx_string_t name{};
        EXPECT_EQ(ovx_path_dictionary_path_to_string(dictionary, paths[index], &name), OVX_OK);
        result.emplace_back(name.ptr, name.length);
    }
    return result;
}

void copyRow(const ovstage_read_group_t& group, size_t row, void* destination, size_t bytes, uintptr_t cudaContext)
{
    const DLTensor& tensor = group.data.tensors[0];
    const size_t index = group.data.index_map ? group.data.index_map[row] : row;
    const int64_t stride = tensor.strides ? tensor.strides[0] : 1;
    const char* source = static_cast<const char*>(tensor.data) + tensor.byte_offset + index * stride * bytes;
    if (tensor.device.device_type == kDLCPU)
    {
        std::memcpy(destination, source, bytes);
        return;
    }
    ASSERT_EQ(tensor.device.device_type, kDLCUDA);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ovphysx::test_cuda::ScopedCudaContextPush context(cuda, cudaContext);
    ASSERT_TRUE(context.ok());
    if (group.data.cuda_sync.wait_event)
    {
        ASSERT_TRUE(cuda->eventSynchronize(group.data.cuda_sync.wait_event, nullptr));
    }
    if (group.data.cuda_sync.stream)
    {
        ASSERT_TRUE(cuda->streamSynchronize(group.data.cuda_sync.stream, nullptr));
    }
    ASSERT_TRUE(cuda->memcpyDtoH(destination, reinterpret_cast<uintptr_t>(source), bytes, nullptr));
}

Matrices readMatrices(ovstage_instance_t* stage,
                      const std::vector<std::string>& paths,
                      ovstage_ordinal_t ordinal,
                      uintptr_t cudaContext = 0)
{
    // Readback is test-side work. Keep ovstage's device selection inside a
    // balanced context scope so it cannot affect the helper's caller contract.
    std::optional<ovphysx::test_cuda::ScopedCudaContextPush> readContext;
    if (cudaContext)
    {
        readContext.emplace(ovphysx::test_cuda::getCuda(), cudaContext);
        EXPECT_TRUE(readContext->ok());
    }
    Matrices result;
    StageQuery query(stage, paths);
    ovx_token_t token = 0;
    EXPECT_EQ(
        ovx_path_dictionary_intern_token(ovstage_get_path_dictionary(stage), attribute(kWorldMatrix).string, &token),
        OVX_OK);
    if (!complete(stage, ovstage_read_attributes(stage, query.query, &token, 1, { 0, ordinal, false }, &query.read)))
        return result;
    for (;;)
    {
        ovstage_read_group_t group{};
        const ovstage_api_status_t status = ovstage_fetch_read_next(stage, query.read, OVSTAGE_TIMEOUT_INFINITE, &group);
        if (status == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        EXPECT_EQ(status, OVSTAGE_OK);
        if (status != OVSTAGE_OK)
            break;
        if (!group.is_delete)
        {
            EXPECT_FALSE(group.is_array);
            EXPECT_EQ(group.data.tensor_count, 1u);
            const DLTensor& tensor = group.data.tensors[0];
            EXPECT_EQ(tensor.dtype.code, kDLFloat);
            EXPECT_EQ(tensor.dtype.bits, 64);
            EXPECT_EQ(tensor.dtype.lanes, 16);
            EXPECT_EQ(group.semantic, OVSTAGE_SEMANTIC_MATRIX);
            const std::vector<std::string> names = groupPaths(ovstage_get_path_dictionary(stage), group);
            for (size_t row = 0; row < names.size(); ++row)
            {
                Matrix matrix{};
                copyRow(group, row, matrix.data(), sizeof(Matrix), cudaContext);
                result[names[row]] = matrix;
            }
        }
        EXPECT_EQ(ovstage_release_group(stage, &group), OVSTAGE_OK);
    }
    return result;
}

bool writeMatrix(ovstage_instance_t* stage,
                 const char* path,
                 const Matrix& matrix,
                 ovstage_ordinal_t ordinal,
                 uintptr_t cudaData = 0,
                 int device = 0)
{
    StageQuery query(stage, { path });
    int64_t shape = 1;
    DLTensor tensor{};
    tensor.data = cudaData ? reinterpret_cast<void*>(cudaData) : const_cast<double*>(matrix.data());
    tensor.device = { cudaData ? kDLCUDA : kDLCPU, device };
    tensor.ndim = 1;
    tensor.shape = &shape;
    tensor.dtype = { kDLFloat, 64, 16 };
    ovstage_write_data_t write{};
    write.tensors = &tensor;
    write.tensor_count = 1;
    write.semantic = OVSTAGE_SEMANTIC_MATRIX;
    return complete(stage, ovstage_write_attribute(
                               stage, query.query, attribute(kWorldMatrix), ordinal, write, OVSTAGE_PRIM_MODE_UPSERT));
}

Poses readPoses(ovphysx_handle_t handle, ovphysx_sim_object_type_t type, uintptr_t cudaContext = 0)
{
    Poses result;
    ovphysx_query_handle_t query = 0;
    EXPECT_EQ(ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query).status, OVPHYSX_API_SUCCESS);
    void* dictionary = nullptr;
    EXPECT_EQ(ovphysx_query_shared_dictionary(handle, query, &dictionary).status, OVPHYSX_API_SUCCESS);
    const ovx_string_or_token_t names[] = { attribute(OVPHYSX_ATTR_POSITION), attribute(OVPHYSX_ATTR_ORIENTATION) };
    ovphysx_read_handle_t read = 0;
    EXPECT_EQ(ovphysx_read(handle, query, names, 2, &read).status, OVPHYSX_API_SUCCESS);
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t status = ovphysx_fetch_read_next(handle, read, &group);
        if (status.status == OVPHYSX_API_END_OF_ITERATION)
            break;
        EXPECT_EQ(status.status, OVPHYSX_API_SUCCESS);
        if (status.status != OVPHYSX_API_SUCCESS)
            break;
        if (!group->is_array && !group->is_delete)
        {
            const DLTensor& tensor = group->data.tensors[0];
            EXPECT_EQ(tensor.device.device_type, cudaContext ? kDLCUDA : kDLCPU);
            const std::vector<std::string> paths = groupPaths(static_cast<ovx_path_dictionary_t*>(dictionary), *group);
            for (size_t row = 0; row < paths.size(); ++row)
            {
                result.try_emplace(paths[row], physx::PxIdentity);
                float values[4]{};
                copyRow(*group, row, values, tensor.dtype.lanes * sizeof(float), cudaContext);
                if (tensor.dtype.lanes == 3)
                    result.at(paths[row]).p = physx::PxVec3(values[0], values[1], values[2]);
                else
                    result.at(paths[row]).q = physx::PxQuat(values[0], values[1], values[2], values[3]);
            }
        }
        EXPECT_EQ(ovphysx_release_group(handle, read, group->read_group_id).status, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(ovphysx_release_read(handle, read).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_release_query(handle, query).status, OVPHYSX_API_SUCCESS);
    return result;
}

std::vector<std::string> posePaths(const Poses& poses)
{
    std::vector<std::string> paths;
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        paths.push_back(item.first);
    return paths;
}

void expectPoseAndScale(const Matrix& actual, const physx::PxTransform& pose, const Matrix& baseline)
{
    physx::PxTransform ignored;
    physx::PxVec3 scale;
    omni::physx::decomposeMatrix(ignored, scale, baseline.data());
    const physx::PxMat44 rotation(pose);
    for (size_t i = 0; i < 16; ++i)
    {
        const double expected = rotation.front()[i] * (i < 12 ? scale[static_cast<uint32_t>(i / 4)] : 1.0);
        EXPECT_NEAR(actual[i], expected, 2e-5) << "matrix element " << i;
    }
}

} // namespace

class OvStageOutputTest : public PhysXTestFixture
{
};

TEST_F(OvStageOutputTest, RigidBodiesAcrossScenesPreserveSignedScaleAndNestedWorldScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/ovstage_output_bodies.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_TRUE(complete(stage, ovstage_compute_hierarchy(stage, OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU, 1, 2)));
    ASSERT_TRUE(seal(stage, 2));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const Poses poses = readPoses(m_handle, OVPHYSX_OBJECT_RIGID_BODY);
    ASSERT_EQ(poses.size(), 3u);
    const Matrices baseline = readMatrices(stage, posePaths(poses), 2);
    ASSERT_EQ(baseline.size(), poses.size());
    EXPECT_DOUBLE_EQ(baseline.at("/World/Parent/Body")[0], 1.0);
    EXPECT_DOUBLE_EQ(baseline.at("/World/Parent/Body")[5], 6.0);
    EXPECT_DOUBLE_EQ(baseline.at("/World/Parent/Body")[10], 1.0);

    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    EXPECT_EQ(result.instancerGroupsSkipped, 0u);
    EXPECT_EQ(writeFloor(stage), 2u);
    ASSERT_TRUE(seal(stage, 3));
    const Matrices output = readMatrices(stage, posePaths(poses), 3);
    ASSERT_EQ(output.size(), poses.size());
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        expectPoseAndScale(output.at(item.first), item.second, baseline.at(item.first));

    // Publishing does not advance the simulation.
    const Poses after = readPoses(m_handle, OVPHYSX_OBJECT_RIGID_BODY);
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        EXPECT_EQ(after.at(item.first).p, item.second.p);
}

TEST_F(OvStageOutputTest, ArticulationLinksMatchCurrentPhysicsPoses)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const Poses poses = readPoses(m_handle, OVPHYSX_OBJECT_ARTICULATION_LINK);
    ASSERT_EQ(poses.size(), 6u);
    const Matrices baseline = readMatrices(stage, posePaths(poses), 1);
    ASSERT_EQ(baseline.size(), poses.size());
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 2);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    ASSERT_TRUE(seal(stage, 2));
    const Matrices output = readMatrices(stage, posePaths(poses), 2);
    ASSERT_EQ(output.size(), poses.size());
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        expectPoseAndScale(output.at(item.first), item.second, baseline.at(item.first));
}

TEST_F(OvStageOutputTest, CacheRetainsScaleUntilRefreshAndDefaultReadsCurrentScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const Poses poses = readPoses(m_handle, OVPHYSX_OBJECT_RIGID_BODY);
    const char* path = "/World/Cube1";
    const Matrix original = readMatrices(stage, { path }, 1).at(path);
    Matrix edited = original;
    edited[0] = -2;
    edited[5] = 3;
    edited[10] = 4;
    OvStageOutputCache cache;
    OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 2, &cache);
    ASSERT_TRUE(result.ok()) << result.message;
    ASSERT_TRUE(seal(stage, 2));
    ASSERT_TRUE(writeMatrix(stage, path, edited, 3));
    ASSERT_TRUE(seal(stage, 3));
    result = writeWorldTransformsToOvstage(m_handle, stage, 4, &cache);
    ASSERT_TRUE(result.ok()) << result.message;
    ASSERT_TRUE(seal(stage, 4));
    expectPoseAndScale(readMatrices(stage, { path }, 4).at(path), poses.at(path), original);

    ASSERT_TRUE(writeMatrix(stage, path, edited, 5));
    ASSERT_TRUE(seal(stage, 5));
    result = writeWorldTransformsToOvstage(m_handle, stage, 6);
    ASSERT_TRUE(result.ok()) << result.message;
    ASSERT_TRUE(seal(stage, 6));
    expectPoseAndScale(readMatrices(stage, { path }, 6).at(path), poses.at(path), edited);

    cache.refresh();
    result = writeWorldTransformsToOvstage(m_handle, stage, 7, &cache);
    ASSERT_TRUE(result.ok()) << result.message;
    ASSERT_TRUE(seal(stage, 7));
    expectPoseAndScale(readMatrices(stage, { path }, 7).at(path), poses.at(path), edited);
}

TEST_F(OvStageOutputTest, InvalidArgumentsAndReattachmentRejectBoundCache)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(writeWorldTransformsToOvstage(m_handle, nullptr, 2).ok());
    EXPECT_FALSE(writeWorldTransformsToOvstage(OVPHYSX_INVALID_HANDLE, stage, 2).ok());
    EXPECT_FALSE(writeWorldTransformsToOvstage(m_handle, stage, 0).ok());
    OvStageOutputCache cache;

    ovstage_instance_desc_t desc{};
    desc.name = "ovphysx-foreign-output-stage";
    ovstage_instance_t* foreignStage = nullptr;
    const ovstage_api_status_t createStatus = ovstage_create_instance(&desc, &foreignStage);
    std::unique_ptr<ovstage_instance_t, decltype(&ovstage_destroy_instance)> foreign(
        foreignStage, ovstage_destroy_instance);
    ASSERT_EQ(createStatus, OVSTAGE_OK);
    ASSERT_NE(foreign.get(), nullptr);
    const std::vector<std::string> paths = posePaths(readPoses(m_handle, OVPHYSX_OBJECT_RIGID_BODY));
    const Matrices baseline = readMatrices(stage, paths, 1);
    ASSERT_EQ(baseline.size(), paths.size());
    ASSERT_FALSE(baseline.empty());
    for (const auto& item : baseline)
    {
        ASSERT_TRUE(writeMatrix(foreign.get(), item.first.c_str(), item.second, 1));
    }
    ASSERT_TRUE(seal(foreign.get(), 1));
    ovstage_ordinal_t ordinal = 2;
    for (OvStageOutputCache* outputCache : { static_cast<OvStageOutputCache*>(nullptr), &cache })
    {
        const OvStageOutputResult rejected = writeWorldTransformsToOvstage(m_handle, foreign.get(), ordinal, outputCache);
        EXPECT_EQ(rejected.status, OVPHYSX_API_INVALID_ARGUMENT);
        EXPECT_EQ(rejected.matricesWritten, 0u);
        EXPECT_FALSE(rejected.message.empty());
        EXPECT_EQ(writeFloor(foreign.get()), ordinal - 1);
        ASSERT_TRUE(seal(foreign.get(), ordinal));
        EXPECT_EQ(readMatrices(foreign.get(), paths, ordinal), baseline);
        ++ordinal;
    }

    OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 2, &cache);
    ASSERT_TRUE(result.ok()) << result.message;
    ASSERT_TRUE(seal(stage, 2));
    ASSERT_EQ(ovphysx_detach_ovstage(m_handle).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(writeWorldTransformsToOvstage(m_handle, stage, 3, &cache).ok());
    ASSERT_EQ(ovphysx_attach_ovstage(m_handle, stage, 2).status, OVPHYSX_API_SUCCESS);
    cache.refresh();
    result = writeWorldTransformsToOvstage(m_handle, stage, 3, &cache);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.matricesWritten, 0u);
    EXPECT_FALSE(result.message.empty());
    OvStageOutputCache fresh;
    result = writeWorldTransformsToOvstage(m_handle, stage, 3, &fresh);
    EXPECT_TRUE(result.ok()) << result.message;
}

TEST_F(OvStageOutputTest, MissingWorldMatrixFailsBeforePublication)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    {
        StageQuery query(stage, { "/World/Cube1" });
        const ovx_string_or_token_t name = attribute(kWorldMatrix);
        ASSERT_TRUE(complete(stage, ovstage_delete_attributes(stage, query.query, &name, 1, 2)));
    }
    ASSERT_TRUE(seal(stage, 2));
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.matricesWritten, 0u);
    EXPECT_FALSE(result.message.empty());
    EXPECT_EQ(writeFloor(stage), 2u);
}

TEST_F(OvStageOutputTest, UnsealedWorldMatrixWriteIsReported)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const OvStageOutputResult first = writeWorldTransformsToOvstage(m_handle, stage, 2);
    ASSERT_TRUE(first.ok()) << first.message;
    const OvStageOutputResult second = writeWorldTransformsToOvstage(m_handle, stage, 3);
    EXPECT_FALSE(second.ok());
    EXPECT_EQ(second.matricesWritten, 0u);
    EXPECT_FALSE(second.message.empty());
    EXPECT_EQ(writeFloor(stage), 1u);
}

TEST_F(OvStageOutputTest, PointInstancerGroupsAreCountedWhileFixedBodyPublishes)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/ovstage_output_instancer.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 2);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, 1u);
    EXPECT_GT(result.instancerGroupsSkipped, 0u);
    ASSERT_TRUE(seal(stage, 2));
    const Matrices output = readMatrices(stage, { "/World/Fixed" }, 2);
    ASSERT_EQ(output.size(), 1u);
    EXPECT_LT(output.at("/World/Fixed")[14], 20.0);
}

TEST_F(OvStageOutputTest, EmptyOutputIsSuccess)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/ovstage_output_empty.usda"));
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, attachedStage(m_handle), 2);
    EXPECT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, 0u);
    EXPECT_EQ(result.instancerGroupsSkipped, 0u);
}

#if OVPHYSX_ENABLE_GPU_TESTS

namespace
{
// Follow test_cpu_no_cuda_context.cpp: inspect the driver dynamically so the
// test binary remains loadable on CPU-only hosts without CUDA SDK linkage.
class ForeignCudaContext
{
public:
    ForeignCudaContext()
    {
#    if defined(_WIN32)
        m_library = LoadLibraryA("nvcuda.dll");
#    else
        m_library = dlopen("libcuda.so.1", RTLD_NOW | RTLD_LOCAL);
#    endif
        if (!m_library)
            return;
        auto symbol = [this](const char* name) -> void* {
#    if defined(_WIN32)
            return reinterpret_cast<void*>(GetProcAddress(m_library, name));
#    else
            return dlsym(m_library, name);
#    endif
        };
        using Create = int (*)(void**, unsigned int, int);
        const Create create = reinterpret_cast<Create>(symbol("cuCtxCreate_v2"));
        m_destroy = reinterpret_cast<Destroy>(symbol("cuCtxDestroy_v2"));
        int device = 0;
        if (create && m_destroy && ovphysx::test_cuda::getCuda()->deviceGet(&device, 0, nullptr))
            m_status = create(&m_context, 0, device);
    }

    ~ForeignCudaContext()
    {
        if (m_context)
        {
            EXPECT_EQ(m_destroy(m_context), 0);
        }
        if (m_library)
        {
#    if defined(_WIN32)
            FreeLibrary(m_library);
#    else
            dlclose(m_library);
#    endif
        }
    }

    uintptr_t context() const
    {
        return reinterpret_cast<uintptr_t>(m_context);
    }
    int status() const
    {
        return m_status;
    }

private:
    using Destroy = int (*)(void*);
#    if defined(_WIN32)
    HMODULE m_library = nullptr;
#    else
    void* m_library = nullptr;
#    endif
    void* m_context = nullptr;
    Destroy m_destroy = nullptr;
    int m_status = -1;
};
} // namespace

class OvStageOutputGpuTest : public ::testing::Test
{
    static ovphysx_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t config[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true")),
        };
        args.config_entries = config;
        args.config_entry_count = 1;
        const ovphysx_result_t created = ovphysx_create_instance(&args, &s_handle);
        if (created.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            s_skipReason.assign(error.ptr ? error.ptr : "", error.length);
            s_handle = 0;
        }
        else if (!ovphysx::test_cuda::cudaAvailable())
        {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            s_skipReason = "CUDA not available";
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle)
            ovphysx_destroy_instance(s_handle);
        s_handle = 0;
    }

    void SetUp() override
    {
        if (!s_handle)
        {
            if (ovphysxTestRequireCuda())
            {
                FAIL() << "GPU required: " << s_skipReason;
            }
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        if (m_handle)
        {
            const ovphysx_enqueue_result_t reset = ovphysx_reset_stage(m_handle);
            ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
            if (reset.op_index)
            {
                EXPECT_TRUE(waitForOperationSuccess(m_handle, reset.op_index));
            }
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
        }
    }
};

ovphysx_handle_t OvStageOutputGpuTest::s_handle = 0;
std::string OvStageOutputGpuTest::s_skipReason;

TEST_F(OvStageOutputGpuTest, DirectGpuCachedAndUncachedMatricesPreserveScaleAndCallerContext)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* actorPointer = nullptr;
    ASSERT_EQ(
        ovphysx_get_physx_ptr(m_handle, ovphysx_cstr("/World/Cube1"), OVPHYSX_PHYSX_TYPE_ACTOR, &actorPointer).status,
        OVPHYSX_API_SUCCESS);
    physx::PxRigidActor* actor = static_cast<physx::PxRigidActor*>(actorPointer);
    ASSERT_NE(actor, nullptr);
    physx::PxCudaContextManager* manager = actor->getScene()->getCudaContextManager();
    ASSERT_NE(manager, nullptr);
    const uintptr_t context = reinterpret_cast<uintptr_t>(manager->getContext());
    ASSERT_FALSE(readPoses(m_handle, OVPHYSX_OBJECT_RIGID_BODY, context).empty());
    const char* path = "/World/Cube1";
    Matrix baseline = readMatrices(stage, { path }, 1, context).at(path);
    baseline[0] = -2;
    baseline[5] = 3;
    baseline[10] = 4;
    ASSERT_TRUE(writeMatrix(stage, path, baseline, 2));
    ASSERT_TRUE(seal(stage, 2));

    ovphysx::test_cuda::ScopedCudaContextDetach detached(ovphysx::test_cuda::getCuda());
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(ovphysx::test_cuda::getCuda()));
    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal = 3; ordinal <= 6; ++ordinal)
    {
        if (ordinal == 6)
            cache.refresh();
        const OvStageOutputResult result =
            writeWorldTransformsToOvstage(m_handle, stage, ordinal, ordinal == 5 ? nullptr : &cache);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_GT(result.matricesWritten, 0u);
        EXPECT_EQ(result.instancerGroupsSkipped, 0u);
        EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(ovphysx::test_cuda::getCuda()));
        ASSERT_TRUE(seal(stage, ordinal));
        const Matrices output = readMatrices(stage, { path }, ordinal, context);
        ASSERT_EQ(output.size(), 1u);
        // Boxes have no angular velocity: mirrored source scale follows canonical
        // decomposition while the current DirectGPU translation reflects gravity.
        const Matrix& matrix = output.at(path);
        EXPECT_NEAR(matrix[0], -2.0, 1e-5);
        EXPECT_NEAR(matrix[5], -3.0, 1e-5);
        EXPECT_NEAR(matrix[10], -4.0, 1e-5);
        EXPECT_LT(matrix[14], baseline[14]);
        EXPECT_DOUBLE_EQ(matrix[15], 1.0);
        EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(ovphysx::test_cuda::getCuda()));
    }
}

TEST_F(OvStageOutputGpuTest, DirectGpuArticulationLinksMatchPoseAndSignedScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const char* path = "/World/articulation/articulationLink0";
    void* pointer = nullptr;
    ASSERT_EQ(ovphysx_get_physx_ptr(m_handle, ovphysx_cstr(path), OVPHYSX_PHYSX_TYPE_LINK, &pointer).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(pointer, nullptr);
    physx::PxArticulationLink* link = static_cast<physx::PxArticulationLink*>(pointer);
    physx::PxCudaContextManager* manager = link->getScene()->getCudaContextManager();
    ASSERT_NE(manager, nullptr);
    const uintptr_t context = reinterpret_cast<uintptr_t>(manager->getContext());
    const Poses poses = readPoses(m_handle, OVPHYSX_OBJECT_ARTICULATION_LINK, context);
    ASSERT_EQ(poses.size(), 6u);
    Matrices baseline = readMatrices(stage, posePaths(poses), 1, context);
    ASSERT_EQ(baseline.size(), poses.size());
    baseline.at(path)[0] = -2;
    baseline.at(path)[5] = 3;
    baseline.at(path)[10] = 4;
    ASSERT_TRUE(writeMatrix(stage, path, baseline.at(path), 2));
    ASSERT_TRUE(seal(stage, 2));

    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal = 3; ordinal <= 4; ++ordinal)
    {
        const OvStageOutputResult result =
            writeWorldTransformsToOvstage(m_handle, stage, ordinal, ordinal == 3 ? &cache : nullptr);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_EQ(result.matricesWritten, poses.size());
        ASSERT_TRUE(seal(stage, ordinal));
        const Matrices output = readMatrices(stage, posePaths(poses), ordinal, context);
        ASSERT_EQ(output.size(), poses.size());
        for (const std::pair<const std::string, physx::PxTransform>& item : poses)
            expectPoseAndScale(output.at(item.first), item.second, baseline.at(item.first));
    }
}

TEST_F(OvStageOutputGpuTest, DirectGpuPublicationRestoresForeignCallerContext)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ForeignCudaContext foreign;
    ASSERT_EQ(foreign.status(), 0);
    ASSERT_NE(foreign.context(), 0u);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal = 2; ordinal <= 3; ++ordinal)
    {
        const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, ordinal, &cache);
        ASSERT_TRUE(result.ok()) << result.message;
        uintptr_t current = 0;
        ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
        EXPECT_EQ(current, foreign.context());
        ASSERT_TRUE(seal(stage, ordinal));
    }
    cache.refresh();
    uintptr_t current = 0;
    ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
    EXPECT_EQ(current, foreign.context());
}

TEST_F(OvStageOutputGpuTest, CpuPosesCaptureGpuAuthoredScaleAndRestoreCallerContext)
{
    // Retain a real primary-context manager for test-side GPU authoring even
    // after switching to a scene whose native physics pose columns are on CPU.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* pointer = nullptr;
    ASSERT_EQ(ovphysx_get_physx_ptr(m_handle, ovphysx_cstr("/World/Cube1"), OVPHYSX_PHYSX_TYPE_ACTOR, &pointer).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_NE(pointer, nullptr);
    physx::PxCudaContextManager* manager =
        static_cast<physx::PxRigidActor*>(pointer)->getScene()->getCudaContextManager();
    ASSERT_NE(manager, nullptr);
    manager->acquireReference();
    std::unique_ptr<physx::PxCudaContextManager, void (*)(physx::PxCudaContextManager*)> retained(
        manager, [](physx::PxCudaContextManager* value) { value->release(); });
    const uintptr_t context = reinterpret_cast<uintptr_t>(manager->getContext());
    ASSERT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));

    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/simple_physics_scene_cpu.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    // readPoses without a CUDA context asserts that every pose tensor is CPU.
    const Poses poses = readPoses(m_handle, OVPHYSX_OBJECT_RIGID_BODY);
    ASSERT_EQ(poses.size(), 4u);
    const char* path = "/World/Cube1";
    Matrix baseline = readMatrices(stage, { path }, 1).at(path);
    baseline[0] = -2;
    baseline[5] = 3;
    baseline[10] = 4;

    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ovphysx::test_cuda::CudaOps ops;
    ops.reset(cuda, context);
    struct DeviceMatrix
    {
        const ovphysx::test_cuda::CudaOps& ops;
        uintptr_t data = 0;
        ~DeviceMatrix()
        {
            EXPECT_TRUE(ops.memFree(data));
        }
    } deviceMatrix{ ops };
    int status = 0;
    ASSERT_TRUE(ops.memAlloc(sizeof(Matrix), &deviceMatrix.data, &status)) << status;
    ASSERT_NE(deviceMatrix.data, 0u);
    ASSERT_TRUE(ops.memcpyHtoD(deviceMatrix.data, baseline.data(), sizeof(Matrix)));
    auto authorGpuMatrix = [&](ovstage_ordinal_t ordinal) {
        ovphysx::test_cuda::ScopedCudaContextPush authorContext(cuda, context);
        int device = 0;
        return authorContext.ok() && cuda->ctxGetDevice(&device, nullptr) &&
               writeMatrix(stage, path, baseline, ordinal, deviceMatrix.data, device) && seal(stage, ordinal);
    };

    ovphysx::test_cuda::ScopedCudaContextDetach detached(cuda);
    ASSERT_TRUE(authorGpuMatrix(2));
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    // No stage read occurs between GPU authoring and the helper's scale capture.
    OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    ASSERT_TRUE(seal(stage, 3));
    expectPoseAndScale(readMatrices(stage, { path }, 3, context).at(path), poses.at(path), baseline);

    ForeignCudaContext foreign;
    ASSERT_EQ(foreign.status(), 0);
    ASSERT_NE(foreign.context(), 0u);
    ASSERT_TRUE(authorGpuMatrix(4));
    result = writeWorldTransformsToOvstage(m_handle, stage, 5);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    uintptr_t current = 0;
    ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
    EXPECT_EQ(current, foreign.context());
    ASSERT_TRUE(seal(stage, 5));
    expectPoseAndScale(readMatrices(stage, { path }, 5, context).at(path), poses.at(path), baseline);
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
