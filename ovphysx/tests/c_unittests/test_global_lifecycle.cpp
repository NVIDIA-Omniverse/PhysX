// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

#include <gtest/gtest.h>

#include "ovphysx/ovphysx.h"

namespace
{
void expectSuccess(ovphysx_result_t result)
{
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}
}

TEST(GlobalLifecycle, SecondInitializeBeforeShutdownReturnsError)
{
    expectSuccess(ovphysx_initialize());
    EXPECT_EQ(ovphysx_initialize().status, OVPHYSX_API_ERROR);
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, ShutdownClearsInitializeState)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);
}

TEST(GlobalLifecycle, ShutdownWithoutInitializeReturnsError)
{
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);
}

TEST(GlobalLifecycle, ExtraShutdownAfterBalancedInitializeReturnsError)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);
}

TEST(GlobalLifecycle, CreateInstanceBeforeInitializeReturnsError)
{
    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(handle, OVPHYSX_INVALID_HANDLE);
}

TEST(GlobalLifecycle, InitializeShutdownInitializeAgain)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, CreateInstanceAfterShutdownRequiresReinitialize)
{
    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_shutdown());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    EXPECT_EQ(ovphysx_create_instance(&args, &handle).status, OVPHYSX_API_ERROR);
    EXPECT_EQ(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_initialize());
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, InitializeDoesNotCreateInstance)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(handle));
    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, RecreateInstanceAfterLastDestroySucceeds)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t firstHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &firstHandle));
    ASSERT_NE(firstHandle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_destroy_instance(firstHandle));

    ovphysx_handle_t secondHandle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &secondHandle));
    ASSERT_NE(secondHandle, OVPHYSX_INVALID_HANDLE);
    expectSuccess(ovphysx_destroy_instance(secondHandle));

    expectSuccess(ovphysx_shutdown());
}

TEST(GlobalLifecycle, ShutdownOnlyClearsInitializeStateWhileInstanceIsLive)
{
    expectSuccess(ovphysx_initialize());

    ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

    ovphysx_handle_t handle = OVPHYSX_INVALID_HANDLE;
    expectSuccess(ovphysx_create_instance(&args, &handle));
    ASSERT_NE(handle, OVPHYSX_INVALID_HANDLE);

    expectSuccess(ovphysx_shutdown());
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_ERROR);

    expectSuccess(ovphysx_destroy_instance(handle));
}
