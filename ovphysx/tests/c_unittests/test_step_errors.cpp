// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-STEP-001
 * @covers AC-1 AC-2
 * @implements REQ-CAPI-TASK-001
 * @covers AC-5 AC-6
 */

#include <gtest/gtest.h>
#include <PxScene.h>
#include <cudamanager/PxCudaContext.h>
#include <cudamanager/PxCudaContextManager.h>
#include <string>

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysx_test_utils.h"

#if OVPHYSX_ENABLE_GPU_TESTS

namespace
{

class GpuStepErrorGpuTest : public ::testing::TestWithParam<bool>
{
protected:
    void SetUp() override
    {
        m_logLevel = ovphysx_get_log_level();
        ASSERT_EQ(ovphysx_create_instance(&m_handle).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_test_get_settings_bool("/physics/suppressReadback", &m_suppressReadback).status,
                  OVPHYSX_API_SUCCESS);
        m_restoreReadback = true;
        const ovphysx_config_entry_t readback = ovphysx_config_entry_carbonite(
            OVPHYSX_LITERAL("/physics/suppressReadback"),
            GetParam() ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false"));
        ASSERT_EQ(ovphysx_set_global_config_entry(readback).status, OVPHYSX_API_SUCCESS);
        if (!ovphysx::test_cuda::cudaAvailable())
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "CUDA is required for GPU step error coverage";
            GTEST_SKIP() << "CUDA is unavailable";
        }

        ASSERT_TRUE(test_utils::attach_usd_with_ovstage(
            m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
        ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

        void* scenePointer = test_utils::read_physx_pointer(
            m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/physicsScene");
        ASSERT_NE(scenePointer, nullptr);
        physx::PxScene* scene = static_cast<physx::PxScene*>(scenePointer);
        ASSERT_TRUE(scene->getFlags().isSet(physx::PxSceneFlag::eENABLE_GPU_DYNAMICS));
        ASSERT_EQ(scene->getFlags().isSet(physx::PxSceneFlag::eENABLE_DIRECT_GPU_API), GetParam());
        ASSERT_NE(scene->getCudaContextManager(), nullptr);
        m_cudaContext = scene->getCudaContextManager()->getCudaContext();
        ASSERT_NE(m_cudaContext, nullptr);
        ASSERT_EQ(m_cudaContext->getLastError(), 0);
        ASSERT_EQ(ovphysx_set_log_level(OVPHYSX_LOG_NONE).status, OVPHYSX_API_SUCCESS);

        // Refuse simulation without damaging the CUDA driver context. PhysX may
        // still mark this scene corrupted, so each case creates a fresh scene.
        m_cudaContext->setAbortMode(true);
    }

    void TearDown() override
    {
        if (m_cudaContext)
            m_cudaContext->setAbortMode(false);
        if (m_handle)
        {
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
            EXPECT_EQ(ovphysx_destroy_instance(m_handle).status, OVPHYSX_API_SUCCESS);
        }
        if (m_restoreReadback)
        {
            const ovphysx_config_entry_t readback = ovphysx_config_entry_carbonite(
                OVPHYSX_LITERAL("/physics/suppressReadback"),
                m_suppressReadback ? OVPHYSX_LITERAL("true") : OVPHYSX_LITERAL("false"));
            EXPECT_EQ(ovphysx_set_global_config_entry(readback).status, OVPHYSX_API_SUCCESS);
        }
        EXPECT_EQ(ovphysx_set_log_level(m_logLevel).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_instance_handle_t m_handle = 0;
    physx::PxCudaContext* m_cudaContext = nullptr;
    uint32_t m_logLevel = OVPHYSX_LOG_WARNING;
    bool m_suppressReadback = false;
    bool m_restoreReadback = false;
};

TEST_P(GpuStepErrorGpuTest, SyncFailureSurvivesDisabledLogging)
{
    for (int step = 0; step < 2; ++step)
    {
        EXPECT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_ERROR);
        const ovphysx_string_t error = ovphysx_get_last_error();
        EXPECT_NE(std::string(error.ptr ? error.ptr : "", error.length).find("CUDA error 2"), std::string::npos);
    }
}

TEST_P(GpuStepErrorGpuTest, AsyncFailureSurvivesDisabledLogging)
{
    for (int step = 0; step < 2; ++step)
    {
        const ovphysx_enqueue_result_t task = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(task.status, OVPHYSX_API_SUCCESS);
        ovphysx_wait_task_result_t wait{};
        EXPECT_EQ(ovphysx_wait_task(m_handle, task, OVPHYSX_TIMEOUT_INFINITE, &wait).status, OVPHYSX_API_ERROR);
        EXPECT_EQ(wait.num_errors, 1u);
        if (wait.num_errors == 1)
        {
            EXPECT_EQ(wait.error_tasks[0].op_index, task.op_index);
            EXPECT_EQ(wait.error_tasks[0].instance, task.instance);
        }
        const ovphysx_string_t error = ovphysx_get_last_task_error(m_handle, task);
        EXPECT_NE(std::string(error.ptr ? error.ptr : "", error.length).find("CUDA error 2"), std::string::npos);
        ovphysx_destroy_wait_task_result(&wait);
    }
}

INSTANTIATE_TEST_SUITE_P(ReadbackModes, GpuStepErrorGpuTest, ::testing::Bool());

} // namespace

#endif
