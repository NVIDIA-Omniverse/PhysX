// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-OVSTAGE-OUTPUT-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5 AC-6 AC-7
 */

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/experimental/OvStageOutput.hpp>
#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/ovphysx_config.h>
#include <ovstage/ovx_path_dictionary.h>
#include <ovstage/ovstage_api/ovstage_tensor_utils.h>
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
using Scales = std::map<std::string, physx::PxVec3>;
using ovphysx::utils::OvStageOutputCache;
using ovphysx::utils::OvStageOutputResult;
using ovphysx::utils::writeWorldTransformsToOvstage;

constexpr const char* kWorldMatrix = "omni:fabric:worldMatrix";
constexpr const char* kLocalMatrix = "omni:xform";

ovx_token_or_string_t attribute(const char* name)
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
    ovstage_ordinal_query_handle_t query = OVSTAGE_INVALID_ORDINAL_QUERY_HANDLE;
    EXPECT_TRUE(complete(stage, ovstage_get_attribute_write_floor(stage, {}, &query)));
    ovstage_ordinal_t ordinal = 0;
    EXPECT_EQ(ovstage_fetch_ordinal(stage, query, OVSTAGE_TIMEOUT_INFINITE, &ordinal), OVSTAGE_OK);
    EXPECT_TRUE(complete(stage, ovstage_release_ordinal_query(stage, query)));
    return ordinal;
}

ovstage_instance_t* attachedStage(ovphysx_instance_handle_t handle)
{
    std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
    return test_utils::ovstage_test_attachments().at(handle).back().stage;
}

struct StageQuery
{
    ovstage_instance_t* stage;
    ovstage_query_handle_t query = OVSTAGE_INVALID_QUERY_HANDLE;
    ovstage_read_handle_t read = OVSTAGE_INVALID_READ_HANDLE;

    StageQuery(ovstage_instance_t* value, const std::vector<std::string>& paths) : stage(value)
    {
        std::vector<ovx_string_t> names;
        for (const std::string& path : paths)
            names.push_back({ path.data(), path.size() });
        ovx_primpath_list_t list = OVX_INVALID_PRIMPATH_LIST;
        ovx_path_dictionary_t* dictionary = ovstage_get_path_dictionary(stage);
        EXPECT_EQ(
            ovx_path_dictionary_create_path_list_from_strings(dictionary, names.data(), names.size(), &list), OVX_OK);
        EXPECT_EQ(ovstage_query_from_path_list(stage, list, &query), OVSTAGE_OK);
        EXPECT_EQ(ovx_path_dictionary_destroy_path_list(dictionary, list), OVX_OK);
    }

    ~StageQuery()
    {
        if (read != OVSTAGE_INVALID_READ_HANDLE)
        {
            EXPECT_TRUE(complete(stage, ovstage_release_read(stage, read)));
        }
        if (query != OVSTAGE_INVALID_QUERY_HANDLE)
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
    const DLTensor& tensor = group.columns[0].data.tensors[0];
    const size_t index = group.columns[0].data.index_map ? group.columns[0].data.index_map[row] : row;
    const size_t elementBytes = tensor.dtype.bits / 8 * tensor.dtype.lanes;
    const size_t strideBytes = tensor.strides ? tensor.strides[0] * elementBytes : bytes;
    const char* source = static_cast<const char*>(tensor.data) + tensor.byte_offset + index * strideBytes;
    if (tensor.device.device_type == kDLCPU)
    {
        std::memcpy(destination, source, bytes);
        return;
    }
    ASSERT_EQ(tensor.device.device_type, kDLCUDA);
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ovphysx::test_cuda::ScopedCudaContextPush context(cuda, cudaContext);
    ASSERT_TRUE(context.ok());
    if (group.columns[0].data.cuda_sync.wait_event)
    {
        ASSERT_TRUE(cuda->eventSynchronize(group.columns[0].data.cuda_sync.wait_event, nullptr));
    }
    if (group.columns[0].data.cuda_sync.stream)
    {
        ASSERT_TRUE(cuda->streamSynchronize(group.columns[0].data.cuda_sync.stream, nullptr));
    }
    ASSERT_TRUE(cuda->memcpyDtoH(destination, reinterpret_cast<uintptr_t>(source), bytes, nullptr));
}

Matrices readMatrices(ovstage_instance_t* stage,
                      const std::vector<std::string>& paths,
                      ovstage_ordinal_t ordinal,
                      uintptr_t cudaContext = 0,
                      const char* name = kWorldMatrix)
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
    ovx_token_t token{0};
    EXPECT_EQ(
        ovx_path_dictionary_intern_token(ovstage_get_path_dictionary(stage), attribute(name).string, &token), OVX_OK);
    if (!complete(stage, ovstage_read_attributes(stage, query.query, &token, 1, { 0, ordinal, false }, &query.read)))
        return result;
    for (;;)
    {
        ovstage_read_group_t group{};
        size_t fetched = 0;
        const ovstage_api_status_t status =
            ovstage_fetch_read_next(stage, query.read, OVSTAGE_TIMEOUT_INFINITE, &group, 1, &fetched);
        if (status == OVSTAGE_ERROR_END_OF_ITERATION)
            break;
        EXPECT_EQ(status, OVSTAGE_OK);
        if (status != OVSTAGE_OK)
            break;
        EXPECT_EQ(group.column_count, 1u);
        if (!group.columns[0].is_delete)
        {
            EXPECT_FALSE(group.columns[0].is_array);
            EXPECT_EQ(group.columns[0].data.tensor_count, 1u);
            const DLTensor& tensor = group.columns[0].data.tensors[0];
            EXPECT_EQ(tensor.dtype.code, kDLFloat);
            EXPECT_EQ(tensor.dtype.bits, 64);
            DLTensor matrixTensor{};
            int64_t layout[2]{};
            EXPECT_EQ(ovstage_tensor_to_write(&tensor, &matrixTensor, layout), OVSTAGE_OK);
            EXPECT_EQ(matrixTensor.dtype.lanes, 16);
            EXPECT_EQ(group.columns[0].semantic, OVSTAGE_SEMANTIC_MATRIX);
            const std::vector<std::string> names = groupPaths(ovstage_get_path_dictionary(stage), group);
            for (size_t row = 0; row < names.size(); ++row)
            {
                Matrix matrix{};
                copyRow(group, row, matrix.data(), sizeof(Matrix), cudaContext);
                result[names[row]] = matrix;
            }
        }
        EXPECT_EQ(ovstage_release_groups(stage, &group, 1), OVSTAGE_OK);
    }
    return result;
}

bool writeMatrix(ovstage_instance_t* stage,
                 const char* path,
                 const Matrix& matrix,
                 ovstage_ordinal_t ordinal,
                 uintptr_t cudaData = 0,
                 int device = 0,
                 const char* name = kWorldMatrix)
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
    return complete(
        stage, ovstage_write_attribute(stage, query.query, attribute(name), ordinal, write, OVSTAGE_PRIM_MODE_UPSERT));
}

Poses readPoses(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, uintptr_t cudaContext = 0)
{
    Poses result;
    ovphysx_query_handle_t query = 0;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)).status, OVPHYSX_API_SUCCESS);
    ovx_path_dictionary_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    EXPECT_NE(dictionary, nullptr);
    const ovx_token_or_string_t names[] = { attribute(OVPHYSX_ATTR_POSITION), attribute(OVPHYSX_ATTR_ORIENTATION) };
    ovphysx_read_handle_t read = 0;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_read(handle, query, names, 2, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t status = test_utils::complete_task(handle, ovphysx_fetch_read_next(handle, read, &group));
        if (status.status == OVPHYSX_API_SUCCESS && !group)
            break;
        EXPECT_EQ(status.status, OVPHYSX_API_SUCCESS);
        if (status.status != OVPHYSX_API_SUCCESS)
            break;
        if (!group->columns[0].is_array && !group->columns[0].is_delete)
        {
            const DLTensor& tensor = group->columns[0].data.tensors[0];
            EXPECT_EQ(tensor.device.device_type, cudaContext ? kDLCUDA : kDLCPU);
            const std::vector<std::string> paths = groupPaths(dictionary, *group);
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
        EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)).status, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_read(handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status, OVPHYSX_API_SUCCESS);
    return result;
}

// The stored creation scale ovphysx serves as OVPHYSX_ATTR_WORLD_SCALE, in the pose's residency.
Scales readWorldScales(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, uintptr_t cudaContext = 0)
{
    Scales result;
    ovphysx_query_handle_t query = 0;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_query(handle, type, OVPHYSX_SCOPE_ALL, &query, nullptr)).status, OVPHYSX_API_SUCCESS);
    ovx_path_dictionary_t* dictionary = test_utils::attached_ovstage_dictionary(handle);
    EXPECT_NE(dictionary, nullptr);
    const ovx_token_or_string_t name = attribute(OVPHYSX_ATTR_WORLD_SCALE);
    ovphysx_read_handle_t read = 0;
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_read(handle, query, &name, 1, ovstage_cuda_sync_t{ 0, 0 }, &read, nullptr)).status, OVPHYSX_API_SUCCESS);
    for (;;)
    {
        const ovstage_read_group_t* group = nullptr;
        const ovphysx_result_t status = test_utils::complete_task(handle, ovphysx_fetch_read_next(handle, read, &group));
        if (status.status == OVPHYSX_API_SUCCESS && !group)
            break;
        EXPECT_EQ(status.status, OVPHYSX_API_SUCCESS);
        if (status.status != OVPHYSX_API_SUCCESS)
            break;
        // worldScale has no point-instancer array form.
        EXPECT_FALSE(group->columns[0].is_array);
        if (!group->columns[0].is_delete)
        {
            const DLTensor& tensor = group->columns[0].data.tensors[0];
            EXPECT_EQ(tensor.device.device_type, cudaContext ? kDLCUDA : kDLCPU);
            EXPECT_EQ(tensor.dtype.code, kDLFloat);
            EXPECT_EQ(tensor.dtype.bits, 32);
            EXPECT_EQ(tensor.dtype.lanes, 3);
            const std::vector<std::string> paths = groupPaths(dictionary, *group);
            for (size_t row = 0; row < paths.size(); ++row)
            {
                float values[3]{};
                copyRow(*group, row, values, sizeof(values), cudaContext);
                result[paths[row]] = physx::PxVec3(values[0], values[1], values[2]);
            }
        }
        EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_group(handle, read, group->read_group_id)).status, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_read(handle, read)).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(test_utils::complete_task(handle, ovphysx_release_query(handle, query)).status, OVPHYSX_API_SUCCESS);
    return result;
}

std::vector<std::string> posePaths(const Poses& poses)
{
    std::vector<std::string> paths;
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        paths.push_back(item.first);
    return paths;
}

physx::PxVec3 decomposedScale(const Matrix& matrix)
{
    physx::PxTransform ignored;
    physx::PxVec3 scale;
    omni::physx::decomposeMatrix(ignored, scale, matrix.data());
    return scale;
}

void expectScale(const physx::PxVec3& actual, const physx::PxVec3& expected)
{
    for (uint32_t axis = 0; axis < 3; ++axis)
        EXPECT_NEAR(actual[axis], expected[axis], 1e-5f) << "scale axis " << axis;
}

void expectPoseAndScale(const Matrix& actual, const physx::PxTransform& pose, const physx::PxVec3& scale)
{
    const physx::PxMat44 rotation(pose);
    for (size_t i = 0; i < 16; ++i)
    {
        const double expected = rotation.front()[i] * (i < 12 ? scale[static_cast<uint32_t>(i / 4)] : 1.0);
        EXPECT_NEAR(actual[i], expected, 2e-5) << "matrix element " << i;
    }
}

// The stored scale of every body is the parser's decomposition of its authored world matrix.
void expectStoredScalesMatch(const Scales& scales, const Matrices& baseline)
{
    ASSERT_EQ(scales.size(), baseline.size());
    for (const std::pair<const std::string, physx::PxVec3>& item : scales)
    {
        SCOPED_TRACE(item.first);
        expectScale(item.second, decomposedScale(baseline.at(item.first)));
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
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    ASSERT_EQ(poses.size(), 3u);
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    const Matrices baseline = readMatrices(stage, posePaths(poses), 2);
    ASSERT_EQ(baseline.size(), poses.size());
    EXPECT_DOUBLE_EQ(baseline.at("/World/Parent/Body")[0], 1.0);
    EXPECT_DOUBLE_EQ(baseline.at("/World/Parent/Body")[5], 6.0);
    EXPECT_DOUBLE_EQ(baseline.at("/World/Parent/Body")[10], 1.0);
    // Stored at creation: a mirror reads all-negative, a nested body its parent-composed scale.
    expectStoredScalesMatch(scales, baseline);
    expectScale(scales.at("/World/Mirrored"), physx::PxVec3(-2.0f, -3.0f, -4.0f));
    expectScale(scales.at("/World/Parent/Body"), physx::PxVec3(1.0f, 6.0f, 1.0f));

    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    EXPECT_EQ(result.instancerGroupsSkipped, 0u);
    EXPECT_EQ(writeFloor(stage), 2u);
    ASSERT_TRUE(seal(stage, 3));
    const Matrices output = readMatrices(stage, posePaths(poses), 3);
    ASSERT_EQ(output.size(), poses.size());
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        expectPoseAndScale(output.at(item.first), item.second, scales.at(item.first));

    // Publishing does not advance the simulation.
    const Poses after = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        EXPECT_EQ(after.at(item.first).p, item.second.p);
}

TEST_F(OvStageOutputTest, CharacterControllersPublishPrimOrientation)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/character_controller.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    const std::vector<std::string> paths = { "/World/Controller", "/World/Other" };
    const Matrices baseline = readMatrices(stage, paths, 1);
    ASSERT_EQ(baseline.size(), paths.size());
    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal = 2; ordinal <= 3; ++ordinal)
    {
        ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        const OvStageOutputResult result =
            writeWorldTransformsToOvstage(m_handle, stage, ordinal, ordinal == 2 ? nullptr : &cache);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_EQ(result.matricesWritten, paths.size());
        ASSERT_TRUE(seal(stage, ordinal));
        const Matrices output = readMatrices(stage, paths, ordinal);
        ASSERT_EQ(output.size(), paths.size());
        for (const std::string& path : paths)
            for (size_t component = 0; component < 16; ++component)
                EXPECT_NEAR(output.at(path)[component], baseline.at(path)[component], 1e-5);
    }
}

// OMPE-120176: an edit of the parent after publication moves the child with it, in OVStage and in PhysX.
TEST_F(OvStageOutputTest, ParentEditAfterPublicationMovesKinematicChild)
{
    const std::string parent = "/World/Outer/Parent";
    const std::string body = "/World/Outer/Parent/Body";
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/ovstage_output_parent_edit.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_TRUE(complete(stage, ovstage_compute_hierarchy(stage, OVSTAGE_HIERARCHY_COMPUTATION_MODEL_DEFAULT_CPU, 1, 2)));
    ASSERT_TRUE(seal(stage, 2));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, 1u);
    EXPECT_EQ(writeFloor(stage), 2u);
    ASSERT_TRUE(seal(stage, 3));

    // Outer turns the parent's (3, 4, 0) translation into (-8, 6, 0) in world space.
    const Matrix translation = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 3, 4, 0, 1 };
    ASSERT_TRUE(writeMatrix(stage, parent.c_str(), translation, 4, 0, 0, kLocalMatrix));
    ASSERT_TRUE(seal(stage, 4));
    const Matrices world = readMatrices(stage, { parent, body }, 4);
    const Matrices local = readMatrices(stage, { body }, 4, 0, kLocalMatrix);
    ASSERT_EQ(world.size(), 2u);
    ASSERT_EQ(local.size(), 1u);
    const auto expectTranslation = [](const Matrix& matrix, double x, double y, double z) {
        EXPECT_NEAR(matrix[12], x, 1e-4);
        EXPECT_NEAR(matrix[13], y, 1e-4);
        EXPECT_NEAR(matrix[14], z, 1e-4);
    };
    expectTranslation(world.at(parent), 42, 6, 0);
    expectTranslation(world.at(body), 42, 6, 8);
    expectTranslation(local.at(body), 0, 0, 4);

    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_update_from_ovstage(m_handle, { 4, 4, true })).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const physx::PxVec3 position = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY).at(body).p;
    EXPECT_NEAR(position.x, 42.0f, 1e-4f);
    EXPECT_NEAR(position.y, 6.0f, 1e-4f);
    EXPECT_NEAR(position.z, 8.0f, 1e-4f);
}

TEST_F(OvStageOutputTest, ArticulationLinksMatchCurrentPhysicsPosesAndStoredScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations_scaled.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK);
    ASSERT_EQ(poses.size(), 6u);
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK);
    const Matrices baseline = readMatrices(stage, posePaths(poses), 1);
    ASSERT_EQ(baseline.size(), poses.size());
    expectStoredScalesMatch(scales, baseline);
    expectScale(scales.at("/World/articulation/articulationLink0"), physx::PxVec3(2.0f));
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 2);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    ASSERT_TRUE(seal(stage, 2));
    const Matrices output = readMatrices(stage, posePaths(poses), 2);
    ASSERT_EQ(output.size(), poses.size());
    for (const std::pair<const std::string, physx::PxTransform>& item : poses)
        expectPoseAndScale(output.at(item.first), item.second, scales.at(item.first));
}

TEST_F(OvStageOutputTest, StageWorldMatrixEditsDoNotChangePublishedScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    const char* path = "/World/Cube1";
    const Matrix original = readMatrices(stage, { path }, 1).at(path);
    expectScale(scales.at(path), decomposedScale(original));
    Matrix edited = original;
    edited[0] = -2;
    edited[5] = 3;
    edited[10] = 4;

    // The helper reads no stage matrix, so neither a cached nor an uncached call, nor one after
    // refresh(), picks up a direct world-matrix edit: the scale is the one ovphysx stored.
    OvStageOutputCache cache;
    ovstage_ordinal_t ordinal = 2;
    for (OvStageOutputCache* outputCache : { &cache, static_cast<OvStageOutputCache*>(nullptr), &cache })
    {
        if (ordinal == 6)
            cache.refresh();
        ASSERT_TRUE(writeMatrix(stage, path, edited, ordinal));
        ASSERT_TRUE(seal(stage, ordinal));
        const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, ordinal + 1, outputCache);
        ASSERT_TRUE(result.ok()) << result.message;
        ASSERT_TRUE(seal(stage, ordinal + 1));
        expectPoseAndScale(readMatrices(stage, { path }, ordinal + 1).at(path), poses.at(path), scales.at(path));
        ordinal += 2;
    }
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
    const std::vector<std::string> paths = posePaths(readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY));
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
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(writeWorldTransformsToOvstage(m_handle, stage, 3, &cache).ok());
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_attach_ovstage(m_handle, stage, 2)).status, OVPHYSX_API_SUCCESS);
    cache.refresh();
    result = writeWorldTransformsToOvstage(m_handle, stage, 3, &cache);
    EXPECT_FALSE(result.ok());
    EXPECT_EQ(result.matricesWritten, 0u);
    EXPECT_FALSE(result.message.empty());
    OvStageOutputCache fresh;
    result = writeWorldTransformsToOvstage(m_handle, stage, 3, &fresh);
    EXPECT_TRUE(result.ok()) << result.message;
}

TEST_F(OvStageOutputTest, MissingWorldMatrixIsPublishedFromStoredScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    const char* path = "/World/Cube1";
    {
        StageQuery query(stage, { path });
        const ovx_token_or_string_t name = attribute(kWorldMatrix);
        ASSERT_TRUE(complete(stage, ovstage_delete_attributes(stage, query.query, &name, 1, 2)));
    }
    ASSERT_TRUE(seal(stage, 2));
    EXPECT_EQ(readMatrices(stage, { path }, 2).count(path), 0u);
    const OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    ASSERT_TRUE(seal(stage, 3));
    expectPoseAndScale(readMatrices(stage, { path }, 3).at(path), poses.at(path), scales.at(path));
}

TEST_F(OvStageOutputTest, OutputOrdinalAtOrBelowWriteFloorIsRejected)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const OvStageOutputResult first = writeWorldTransformsToOvstage(m_handle, stage, 2);
    ASSERT_TRUE(first.ok()) << first.message;
    // The helper never seals its own output.
    EXPECT_EQ(writeFloor(stage), 1u);
    ASSERT_TRUE(seal(stage, 2));
    for (ovstage_ordinal_t ordinal : { 1, 2 })
    {
        const OvStageOutputResult rejected = writeWorldTransformsToOvstage(m_handle, stage, ordinal);
        EXPECT_FALSE(rejected.ok());
        EXPECT_EQ(rejected.matricesWritten, 0u);
        EXPECT_FALSE(rejected.message.empty());
    }
    EXPECT_EQ(writeFloor(stage), 2u);
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
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t config[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true")),
        };
        args.config_entries = config;
        args.config_entry_count = 1;
        const ovphysx_result_t created = ovphysx_create_instance_with_args(&args, &s_handle);
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
            const ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
            ASSERT_EQ(reset.status, OVPHYSX_API_SUCCESS);
            if (reset.op_index)
            {
                EXPECT_TRUE(waitForTaskSuccess(m_handle, reset));
            }
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
        }
    }
};

ovphysx_instance_handle_t OvStageOutputGpuTest::s_handle = 0;
std::string OvStageOutputGpuTest::s_skipReason;

TEST_F(OvStageOutputGpuTest, DirectGpuCachedAndUncachedMatricesUseDeviceScaleAndCallerContext)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/ovstage_output_bodies_gpu.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* actorPointer = test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Mirrored");
    physx::PxRigidActor* actor = static_cast<physx::PxRigidActor*>(actorPointer);
    ASSERT_NE(actor, nullptr);
    physx::PxCudaContextManager* manager = actor->getScene()->getCudaContextManager();
    ASSERT_NE(manager, nullptr);
    const uintptr_t context = reinterpret_cast<uintptr_t>(manager->getContext());
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, context);
    ASSERT_EQ(poses.size(), 2u);
    // readWorldScales asserts kDLCUDA: the stored scale is served from its device copy.
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, context);
    const Matrices baseline = readMatrices(stage, posePaths(poses), 1, context);
    expectStoredScalesMatch(scales, baseline);
    expectScale(scales.at("/World/Mirrored"), physx::PxVec3(-2.0f, -3.0f, -4.0f));
    expectScale(scales.at("/World/Parent/Body"), physx::PxVec3(1.0f, 6.0f, 1.0f));

    ovphysx::test_cuda::ScopedCudaContextDetach detached(ovphysx::test_cuda::getCuda());
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(ovphysx::test_cuda::getCuda()));
    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal = 2; ordinal <= 5; ++ordinal)
    {
        if (ordinal == 5)
            cache.refresh();
        const OvStageOutputResult result =
            writeWorldTransformsToOvstage(m_handle, stage, ordinal, ordinal == 4 ? nullptr : &cache);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_EQ(result.matricesWritten, poses.size());
        EXPECT_EQ(result.instancerGroupsSkipped, 0u);
        EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(ovphysx::test_cuda::getCuda()));
        ASSERT_TRUE(seal(stage, ordinal));
        const Matrices output = readMatrices(stage, posePaths(poses), ordinal, context);
        ASSERT_EQ(output.size(), poses.size());
        for (const std::pair<const std::string, physx::PxTransform>& item : poses)
            expectPoseAndScale(output.at(item.first), item.second, scales.at(item.first));
        EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(ovphysx::test_cuda::getCuda()));
    }
}

TEST_F(OvStageOutputGpuTest, DirectGpuArticulationLinksMatchPoseAndStoredScale)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations_scaled.usda"));
    ovstage_instance_t* stage = attachedStage(m_handle);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    const char* path = "/World/articulation/articulationLink0";
    void* pointer = test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, path);
    ASSERT_NE(pointer, nullptr);
    physx::PxArticulationLink* link = static_cast<physx::PxArticulationLink*>(pointer);
    physx::PxCudaContextManager* manager = link->getScene()->getCudaContextManager();
    ASSERT_NE(manager, nullptr);
    const uintptr_t context = reinterpret_cast<uintptr_t>(manager->getContext());
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, context);
    ASSERT_EQ(poses.size(), 6u);
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, context);
    const Matrices baseline = readMatrices(stage, posePaths(poses), 1, context);
    expectStoredScalesMatch(scales, baseline);
    expectScale(scales.at(path), physx::PxVec3(2.0f));

    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal = 2; ordinal <= 3; ++ordinal)
    {
        const OvStageOutputResult result =
            writeWorldTransformsToOvstage(m_handle, stage, ordinal, ordinal == 2 ? &cache : nullptr);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_EQ(result.matricesWritten, poses.size());
        ASSERT_TRUE(seal(stage, ordinal));
        const Matrices output = readMatrices(stage, posePaths(poses), ordinal, context);
        ASSERT_EQ(output.size(), poses.size());
        for (const std::pair<const std::string, physx::PxTransform>& item : poses)
            expectPoseAndScale(output.at(item.first), item.second, scales.at(item.first));
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

TEST_F(OvStageOutputGpuTest, CpuPublicationOverGpuAuthoredMatricesKeepsStoredScaleAndCallerContext)
{
    // Retain a real primary-context manager for test-side GPU authoring even
    // after switching to a scene whose native physics pose columns are on CPU.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    void* pointer = test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_RIGID_ACTOR, "/World/Cube1");
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
    // readPoses and readWorldScales without a CUDA context assert that every tensor is CPU.
    const Poses poses = readPoses(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    ASSERT_EQ(poses.size(), 4u);
    const Scales scales = readWorldScales(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY);
    const char* path = "/World/Cube1";
    // Authored on the device so the stage column the CPU output overwrites is GPU-backed. Its edited
    // scale must not reach the output: the helper publishes the stored physics scale.
    Matrix baseline = readMatrices(stage, { path }, 1).at(path);
    expectScale(scales.at(path), decomposedScale(baseline));
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
    // The helper reads no stage matrix, so its CPU publication is the only stage access that can
    // touch the GPU-backed column.
    OvStageOutputResult result = writeWorldTransformsToOvstage(m_handle, stage, 3);
    ASSERT_TRUE(result.ok()) << result.message;
    EXPECT_EQ(result.matricesWritten, poses.size());
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
    ASSERT_TRUE(seal(stage, 3));
    expectPoseAndScale(readMatrices(stage, { path }, 3, context).at(path), poses.at(path), scales.at(path));

    OvStageOutputCache cache;
    for (ovstage_ordinal_t ordinal : { 4, 6 })
    {
        // The second call reuses cached buffers, so it tests CPU publication into
        // GPU-backed storage with nothing but the write under the context guard.
        ASSERT_TRUE(authorGpuMatrix(ordinal));
        ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
        result = writeWorldTransformsToOvstage(m_handle, stage, ordinal + 1, &cache);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_EQ(result.matricesWritten, poses.size());
        EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));
        ASSERT_TRUE(seal(stage, ordinal + 1));
        expectPoseAndScale(
            readMatrices(stage, { path }, ordinal + 1, context).at(path), poses.at(path), scales.at(path));
    }

    ForeignCudaContext foreign;
    ASSERT_EQ(foreign.status(), 0);
    ASSERT_NE(foreign.context(), 0u);
    for (ovstage_ordinal_t ordinal : { 8, 10 })
    {
        ASSERT_TRUE(authorGpuMatrix(ordinal));
        result = writeWorldTransformsToOvstage(m_handle, stage, ordinal + 1, ordinal == 8 ? nullptr : &cache);
        ASSERT_TRUE(result.ok()) << result.message;
        EXPECT_EQ(result.matricesWritten, poses.size());
        uintptr_t current = 0;
        ASSERT_TRUE(cuda->ctxGetCurrent(&current, nullptr));
        EXPECT_EQ(current, foreign.context());
        ASSERT_TRUE(seal(stage, ordinal + 1));
        expectPoseAndScale(
            readMatrices(stage, { path }, ordinal + 1, context).at(path), poses.at(path), scales.at(path));
    }
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
