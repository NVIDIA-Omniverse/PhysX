// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-READ-CORE-001
 * @covers AC-8
 * @implements REQ-INPUT-CORE-001
 * @covers AC-10
 *
 * DirectGPU disabled-body behavior through the official ovstage disableSimulation write path,
 * exercised at the ovphysx C-API surface. Cross-tree by design: the contract lives in these
 * ovruntime REQs and spans the interface, and this verifies its ovphysx face -- CPU reads keep the
 * disabled body while DirectGPU omits it (REQ-READ-CORE-001 AC-8) and writes land on the enabled
 * peers (REQ-INPUT-CORE-001 AC-10). REQ-CAPI-PHYSXPTR-001 AC-6 (the raw-pointer-unsupported
 * documentation) is verified by inspection, not by this test.
 */

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

#include <ovstage/ovx_path_dictionary.h>

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <vector>

using namespace test_utils;

namespace
{

ovx_string_or_token_t makeAttr(const char* name)
{
    ovx_string_or_token_t a{};
    a.token = 0;
    a.string.ptr = name;
    a.string.length = std::strlen(name);
    return a;
}

ovstage_cuda_sync_t noSync()
{
    ovstage_cuda_sync_t s{};
    s.stream = 0;
    s.wait_event = 0;
    return s;
}

bool stepOnce(ovphysx_handle_t handle)
{
    return ovphysx_step_sync(handle, 1.0f / 60.0f).status == OVPHYSX_API_SUCCESS;
}

std::map<std::string, float> readMassByPath(ovphysx_handle_t handle)
{
    std::map<std::string, float> out;
    ovphysx_query_handle_t q = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status != OVPHYSX_API_SUCCESS ||
        q == 0)
        return out;

    void* dictVoid = nullptr;
    if (ovphysx_query_shared_dictionary(handle, q, &dictVoid).status != OVPHYSX_API_SUCCESS || !dictVoid)
    {
        ovphysx_release_query(handle, q);
        return out;
    }
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictVoid);

    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_MASS);
    ovphysx_read_handle_t rh = 0;
    if (ovphysx_read(handle, q, &attr, 1, &rh).status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_release_query(handle, q);
        return out;
    }

    const ovstage_read_group_t* g = nullptr;
    while (ovphysx_fetch_read_next(handle, rh, &g).status == OVPHYSX_API_SUCCESS)
    {
        if (!g || g->data.tensor_count == 0 || !g->data.tensors)
        {
            ovphysx_release_group(handle, rh, g ? g->read_group_id : 0);
            continue;
        }
        const DLTensor& t = g->data.tensors[0];
        if (t.device.device_type != kDLCPU || !t.data)
        {
            ovphysx_release_group(handle, rh, g->read_group_id);
            continue;
        }

        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (ovx_path_dictionary_get_paths(dict, g->prims.list, &paths, &pathCount) != OVX_OK || !paths)
        {
            ovphysx_release_group(handle, rh, g->read_group_id);
            continue;
        }

        const float* values = static_cast<const float*>(t.data);
        const uint32_t n = g->prims.count;
        for (uint32_t i = 0; i < n && i < pathCount; ++i)
        {
            ovx_string_t ps{};
            if (ovx_path_dictionary_path_to_string(dict, paths[i], &ps) != OVX_OK || !ps.ptr)
                continue;
            out[std::string(ps.ptr, ps.length)] = values[i];
        }
        ovphysx_release_group(handle, rh, g->read_group_id);
    }

    ovphysx_release_read(handle, rh);
    ovphysx_release_query(handle, q);
    return out;
}

uint32_t queryPrimCount(ovphysx_handle_t handle)
{
    ovphysx_query_handle_t q = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status != OVPHYSX_API_SUCCESS)
        return 0;
    ovstage_query_result_t qr{};
    const ovphysx_api_status_t st = ovphysx_fetch_query_result(handle, q, &qr).status;
    ovphysx_release_query(handle, q);
    if (st != OVPHYSX_API_SUCCESS)
        return 0;
    return qr.total_prim_count;
}

bool writeMassByPath(ovphysx_handle_t handle, const std::map<std::string, float>& masses)
{
    std::map<std::string, float> merged = readMassByPath(handle);
    for (const std::pair<const std::string, float>& item : masses)
        merged[item.first] = item.second;
    ovphysx_query_handle_t q = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status != OVPHYSX_API_SUCCESS)
        return false;

    void* dictVoid = nullptr;
    if (ovphysx_query_shared_dictionary(handle, q, &dictVoid).status != OVPHYSX_API_SUCCESS || !dictVoid)
    {
        ovphysx_release_query(handle, q);
        return false;
    }
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictVoid);

    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_MASS);
    ovphysx_write_handle_t w = 0;
    if (ovphysx_write(handle, q, &attr, &w).status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_release_query(handle, q);
        return false;
    }

    bool ok = true;
    const ovstage_map_group_t* g = nullptr;
    while (ovphysx_fetch_write_next(handle, w, &g).status == OVPHYSX_API_SUCCESS)
    {
        if (!g || g->data.tensor_count == 0 || !g->data.tensors)
        {
            ok = false;
            break;
        }
        const DLTensor& t = g->data.tensors[0];
        if (t.device.device_type != kDLCPU || !t.data)
        {
            ok = false;
            break;
        }

        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (ovx_path_dictionary_get_paths(dict, g->prims.list, &paths, &pathCount) != OVX_OK || !paths)
        {
            ok = false;
            break;
        }

        float* values = static_cast<float*>(t.data);
        for (uint32_t i = 0; i < g->prims.count && i < pathCount; ++i)
        {
            ovx_string_t ps{};
            if (ovx_path_dictionary_path_to_string(dict, paths[i], &ps) != OVX_OK || !ps.ptr)
                continue;
            const std::map<std::string, float>::const_iterator it = merged.find(std::string(ps.ptr, ps.length));
            if (it != merged.end())
                values[i] = it->second;
        }
        if (ovphysx_commit_group(handle, w, g, noSync()).status != OVPHYSX_API_SUCCESS)
        {
            ok = false;
            break;
        }
    }

    ovphysx_release_write(handle, w);
    ovphysx_release_query(handle, q);
    return ok;
}

bool writeDisableSimulationByPath(ovphysx_handle_t handle, const std::map<std::string, uint8_t>& flags)
{
    ovphysx_query_handle_t q = 0;
    if (ovphysx_query(handle, OVPHYSX_OBJECT_RIGID_BODY, OVPHYSX_SCOPE_ALL, &q).status != OVPHYSX_API_SUCCESS)
        return false;

    void* dictVoid = nullptr;
    if (ovphysx_query_shared_dictionary(handle, q, &dictVoid).status != OVPHYSX_API_SUCCESS || !dictVoid)
    {
        ovphysx_release_query(handle, q);
        return false;
    }
    ovx_path_dictionary_t* dict = static_cast<ovx_path_dictionary_t*>(dictVoid);

    const ovx_string_or_token_t attr = makeAttr(OVPHYSX_ATTR_DISABLE_SIMULATION);
    ovphysx_write_handle_t w = 0;
    if (ovphysx_write(handle, q, &attr, &w).status != OVPHYSX_API_SUCCESS)
    {
        ovphysx_release_query(handle, q);
        return false;
    }

    bool ok = true;
    const ovstage_map_group_t* g = nullptr;
    while (ovphysx_fetch_write_next(handle, w, &g).status == OVPHYSX_API_SUCCESS)
    {
        if (!g || g->data.tensor_count == 0 || !g->data.tensors)
        {
            ok = false;
            break;
        }
        const DLTensor& t = g->data.tensors[0];
        if (t.device.device_type != kDLCPU || !t.data)
        {
            ok = false;
            break;
        }

        const ovx_primpath_t* paths = nullptr;
        size_t pathCount = 0;
        if (ovx_path_dictionary_get_paths(dict, g->prims.list, &paths, &pathCount) != OVX_OK || !paths)
        {
            ok = false;
            break;
        }

        uint8_t* values = static_cast<uint8_t*>(t.data);
        for (uint32_t i = 0; i < g->prims.count && i < pathCount; ++i)
        {
            ovx_string_t ps{};
            if (ovx_path_dictionary_path_to_string(dict, paths[i], &ps) != OVX_OK || !ps.ptr)
                continue;
            const std::map<std::string, uint8_t>::const_iterator it =
                flags.find(std::string(ps.ptr, ps.length));
            if (it != flags.end())
                values[i] = it->second;
        }
        if (ovphysx_commit_group(handle, w, g, noSync()).status != OVPHYSX_API_SUCCESS)
        {
            ok = false;
            break;
        }
    }

    ovphysx_release_write(handle, w);
    ovphysx_release_query(handle, q);
    return ok;
}

} // namespace

TEST_F(PhysXTestFixture, CpuOfficialDisableKeepsDisabledBodyInReadColumns)
{
    ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));
    ASSERT_TRUE(stepOnce(m_handle));

    const std::map<std::string, float> before = readMassByPath(m_handle);
    ASSERT_GE(before.size(), 2u);
    ASSERT_EQ(before.count("/World/Cube1"), 1u);

    std::map<std::string, uint8_t> disableCube1;
    disableCube1["/World/Cube1"] = 1u;
    ASSERT_TRUE(writeDisableSimulationByPath(m_handle, disableCube1));

    const std::map<std::string, float> whileDisabled = readMassByPath(m_handle);
    EXPECT_EQ(whileDisabled.size(), before.size());
    EXPECT_EQ(whileDisabled.count("/World/Cube1"), 1u);

    std::map<std::string, uint8_t> enableCube1;
    enableCube1["/World/Cube1"] = 0u;
    ASSERT_TRUE(writeDisableSimulationByPath(m_handle, enableCube1));

    const std::map<std::string, float> after = readMassByPath(m_handle);
    EXPECT_EQ(after.size(), before.size());
    EXPECT_EQ(after.count("/World/Cube1"), 1u);
}

#if OVPHYSX_ENABLE_GPU_TESTS

class DirectGpuDisabledBodyGpuTest : public ::testing::Test
{
    static ovphysx_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t directGpu[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"),
                                           OVPHYSX_LITERAL("true")),
        };
        args.config_entries = directGpu;
        args.config_entry_count = sizeof(directGpu) / sizeof(directGpu[0]);

        ovphysx_result_t created = ovphysx_create_instance(&args, &s_handle);
        if (created.status != OVPHYSX_API_SUCCESS)
        {
            ovphysx_string_t err = ovphysx_get_last_error();
            s_skipReason = err.length > 0 ? std::string(err.ptr, err.length) : std::string("Failed to create GPU instance");
            s_handle = 0;
            return;
        }

        if (!ovphysx::test_cuda::cudaAvailable())
        {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            s_skipReason = "CUDA not available";
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle != 0)
        {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    void SetUp() override
    {
        if (s_handle == 0)
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "GPU/CUDA not available (OVPHYSX_TEST_REQUIRE_CUDA=1): " << s_skipReason;
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        if (s_handle != 0)
        {
            ovphysx_enqueue_result_t reset = ovphysx_reset_stage(s_handle);
            if (reset.status == OVPHYSX_API_SUCCESS && reset.op_index != 0)
            {
                ovphysx_op_wait_result_t wait{};
                ovphysx_wait_op(s_handle, reset.op_index, 10'000'000'000ULL, &wait);
                ovphysx_destroy_wait_result(&wait);
            }
            test_utils::destroy_ovstage_test_attachments(s_handle);
        }
        m_handle = 0;
    }

    void attachGpuBoxes()
    {
        ASSERT_TRUE(attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
        ASSERT_TRUE(stepOnce(m_handle));
    }
};

ovphysx_handle_t DirectGpuDisabledBodyGpuTest::s_handle = 0;
std::string DirectGpuDisabledBodyGpuTest::s_skipReason;

TEST_F(DirectGpuDisabledBodyGpuTest, OfficialDisableOmitsBodyAndWriteLandsOnNamedPeers)
{
    attachGpuBoxes();

    const std::map<std::string, float> before = readMassByPath(m_handle);
    ASSERT_GE(before.size(), 2u);
    ASSERT_EQ(before.count("/World/Cube1"), 1u);
    const uint32_t discovery = queryPrimCount(m_handle);
    ASSERT_EQ(discovery, static_cast<uint32_t>(before.size()));

    std::map<std::string, uint8_t> disableCube1;
    disableCube1["/World/Cube1"] = 1u;
    ASSERT_TRUE(writeDisableSimulationByPath(m_handle, disableCube1));

    const std::map<std::string, float> whileDisabled = readMassByPath(m_handle);
    EXPECT_EQ(queryPrimCount(m_handle), discovery);
    EXPECT_EQ(whileDisabled.size(), before.size() - 1);
    EXPECT_EQ(whileDisabled.count("/World/Cube1"), 0u);
    EXPECT_EQ(whileDisabled.count("/World/Cube2"), 1u);

    std::map<std::string, float> written;
    float stamp = 11.0f;
    for (const std::pair<const std::string, float>& item : whileDisabled)
    {
        written[item.first] = stamp;
        stamp += 1.0f;
    }
    ASSERT_TRUE(writeMassByPath(m_handle, written));

    const std::map<std::string, float> afterWrite = readMassByPath(m_handle);
    ASSERT_EQ(afterWrite.size(), written.size());
    for (const std::pair<const std::string, float>& item : written)
    {
        ASSERT_EQ(afterWrite.count(item.first), 1u);
        EXPECT_FLOAT_EQ(afterWrite.at(item.first), item.second);
    }
    EXPECT_EQ(afterWrite.count("/World/Cube1"), 0u);

    std::map<std::string, uint8_t> enableCube1;
    enableCube1["/World/Cube1"] = 0u;
    ASSERT_TRUE(writeDisableSimulationByPath(m_handle, enableCube1));

    const std::map<std::string, float> reenabled = readMassByPath(m_handle);
    EXPECT_EQ(reenabled.size(), before.size());
    EXPECT_EQ(reenabled.count("/World/Cube1"), 1u);

    std::map<std::string, float> cube1Only;
    cube1Only["/World/Cube1"] = 42.0f;
    ASSERT_TRUE(writeMassByPath(m_handle, cube1Only));
    const std::map<std::string, float> afterCube1Write = readMassByPath(m_handle);
    ASSERT_EQ(afterCube1Write.count("/World/Cube1"), 1u);
    EXPECT_FLOAT_EQ(afterCube1Write.at("/World/Cube1"), 42.0f);
    ASSERT_EQ(afterCube1Write.count("/World/Cube2"), 1u);
    EXPECT_FLOAT_EQ(afterCube1Write.at("/World/Cube2"), written.at("/World/Cube2"));
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
