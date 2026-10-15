// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-CAPI-COMPANION-001
// @covers AC-1 AC-2 AC-3
// @implements REQ-COMP-API-001
// @covers AC-1 AC-2 AC-3
// @implements REQ-COMP-OWNER-001
// @covers AC-1 AC-2 AC-3 AC-4
// @implements REQ-COMP-DEBUG-001
// @covers AC-2

// @implements REQ-COMP-MASS-001
// @covers AC-1 AC-2 AC-3

#include <ovphysx_companion/companion.h>
#include <ovphysx/experimental/ovphysx.hpp>
#include <stdexcept>
#include <future>
#include <thread>
#include "internal/sdk/CompanionRuntime.hpp"
#include "global_test_environment.h"
#include <cstring>

namespace
{
struct CompanionBridge : PhysXTestFixture
{
    ovphysx_companion_api_t api{};
    std::vector<ovphysx_companion_session_t> sessions;

    void SetUp() override
    {
        PhysXTestFixture::SetUp();
        ASSERT_EQ(ovphysx_companion_acquire(1, sizeof(api), &api), OVPHYSX_COMPANION_OK);
    }
    void TearDown() override
    {
        for (uint64_t const session : sessions)
            api.close_session(session);
        PhysXTestFixture::TearDown();
    }
    uint64_t open()
    {
        uint64_t session = 0;
        EXPECT_EQ(api.open_session(m_handle, &session), OVPHYSX_COMPANION_OK);
        if (session)
            sessions.push_back(session);
        return session;
    }
    bool attach()
    {
        return test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda");
    }
};
}

TEST(CompanionDiscovery, VersionAndSizeFailureLeaveOutputUntouched)
{
    ovphysx_companion_api_t output;
    std::memset(&output, 0xa5, sizeof(output));
    unsigned char before[sizeof(output)];
    std::memcpy(before, &output, sizeof(output));
    EXPECT_EQ(ovphysx_companion_acquire(2, sizeof(output), &output), OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before, &output, sizeof(output)), 0);
    EXPECT_EQ(ovphysx_companion_acquire(1, sizeof(output) - 1, &output), OVPHYSX_COMPANION_ABI_MISMATCH);
    EXPECT_EQ(std::memcmp(before, &output, sizeof(output)), 0);
    EXPECT_EQ(ovphysx_companion_acquire(1, sizeof(output), nullptr), OVPHYSX_COMPANION_INVALID_ARGUMENT);
    ASSERT_EQ(ovphysx_companion_acquire(1, sizeof(output), &output), OVPHYSX_COMPANION_OK);
    EXPECT_EQ(output.size, sizeof(output));
    EXPECT_EQ(output.version, 1u);
    EXPECT_NE(output.capabilities & OVPHYSX_COMPANION_CAP_BORROWED_SESSION, 0u);
    EXPECT_NE(output.open_session, nullptr);
}

TEST_F(CompanionBridge, OpeningDoesNotAttachAndInvalidHandlesAreRejected)
{
    uint64_t output = 123;
    EXPECT_EQ(api.open_session(0, &output), OVPHYSX_COMPANION_NOT_FOUND);
    EXPECT_EQ(output, 123u);
    EXPECT_EQ(api.open_session(m_handle, &output), OVPHYSX_COMPANION_STALE);
    EXPECT_EQ(output, 123u);
    EXPECT_EQ(api.open_session(m_handle, nullptr), OVPHYSX_COMPANION_INVALID_ARGUMENT);
    EXPECT_EQ(api.close_session(0), OVPHYSX_COMPANION_NOT_FOUND);
}

TEST_F(CompanionBridge, LeasesRejectDestroyAndShutdownWithoutConsumingOwnership)
{
    ASSERT_TRUE(attach());
    const uint64_t session = open();
    ASSERT_NE(session, 0u);
    EXPECT_EQ(ovphysx_destroy_instance(m_handle).status, OVPHYSX_API_INVALID_STATE);
    EXPECT_EQ(ovphysx_shutdown().status, OVPHYSX_API_INVALID_STATE);
    ovphysx_companion_identity_t identity{};
    ASSERT_EQ(api.get_identity(session, &identity), OVPHYSX_COMPANION_OK);
    EXPECT_EQ(identity.instance, m_handle);
    EXPECT_NE(identity.attachment, 0u);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_step(m_handle, 1.0f / 60.0f)));
    EXPECT_EQ(api.close_session(session), OVPHYSX_COMPANION_OK);
    EXPECT_EQ(api.close_session(session), OVPHYSX_COMPANION_NOT_FOUND);
    EXPECT_EQ(api.get_identity(session, &identity), OVPHYSX_COMPANION_NOT_FOUND);
}

TEST_F(CompanionBridge, DetachReattachInvalidatesGenerationWithoutReusingSession)
{
    ASSERT_TRUE(attach());
    const uint64_t first = open();
    ovphysx_companion_identity_t initial{};
    ASSERT_EQ(api.get_identity(first, &initial), OVPHYSX_COMPANION_OK);
    ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_detach_ovstage(m_handle)).status, OVPHYSX_API_SUCCESS);
    ovphysx_companion_identity_t stale{123, 456};
    EXPECT_EQ(api.get_identity(first, &stale), OVPHYSX_COMPANION_STALE);
    EXPECT_EQ(stale.attachment, 456u);
    ASSERT_TRUE(attach());
    const uint64_t second = open();
    ASSERT_NE(first, second);
    ovphysx_companion_identity_t current{};
    ASSERT_EQ(api.get_identity(second, &current), OVPHYSX_COMPANION_OK);
    EXPECT_NE(initial.attachment, current.attachment);
    EXPECT_EQ(api.get_identity(first, &stale), OVPHYSX_COMPANION_STALE);
    EXPECT_EQ(api.close_session(first), OVPHYSX_COMPANION_OK);
    EXPECT_EQ(api.get_identity(second, &current), OVPHYSX_COMPANION_OK);
}

TEST_F(CompanionBridge, SceneStatisticsAreCopiedAndOwnerScoped)
{
    ASSERT_TRUE(attach());
    ASSERT_TRUE(waitForTaskSuccess(m_handle, ovphysx_step(m_handle, 1.0f / 60.0f)));
    const uint64_t session = open();
    ovphysx_companion_scene_statistics_t stats{};
    ASSERT_EQ(api.scene_statistics(session, "/World/physicsScene", sizeof(stats), &stats), OVPHYSX_COMPANION_OK);
    EXPECT_EQ(stats.owner.instance, m_handle);
    EXPECT_GT(stats.dynamic_bodies, 0u);
    EXPECT_GT(stats.static_bodies, 0u);
    ovphysx_companion_scene_statistics_t saved = stats;
    EXPECT_EQ(api.scene_statistics(session, "/missing", sizeof(stats), &stats), OVPHYSX_COMPANION_NOT_FOUND);
    EXPECT_EQ(std::memcmp(&saved, &stats, sizeof(stats)), 0);
    EXPECT_EQ(api.scene_statistics(session, "/World/physicsScene", sizeof(stats) - 1, &stats),
              OVPHYSX_COMPANION_INVALID_ARGUMENT);
    EXPECT_EQ(std::memcmp(&saved, &stats, sizeof(stats)), 0);
}

TEST(CompanionDiscovery, PrimitiveMassDoesNotRequireAnOwner)
{
    ovphysx_companion_api_t api{};
    ASSERT_EQ(ovphysx_companion_acquire(1, sizeof(api), &api), OVPHYSX_COMPANION_OK);
    ovphysx_companion_mass_shape_t shape{};
    shape.kind = OVPHYSX_COMPANION_BOX;
    shape.dimensions[0] = shape.dimensions[1] = shape.dimensions[2] = 1;
    shape.rotation[3] = 1;
    shape.density = 2;
    ovphysx_companion_mass_result_t output{};
    uint32_t invalid = 0;
    ASSERT_EQ(api.compute_primitive_mass(&shape, 1, sizeof(output), &output, &invalid), OVPHYSX_COMPANION_OK);
    EXPECT_EQ(invalid, UINT32_MAX);
    EXPECT_FLOAT_EQ(output.mass, 16);
    EXPECT_FLOAT_EQ(output.volume, 8);
    for (float inertia : output.diagonal_inertia) EXPECT_NEAR(inertia, 32.0f/3, 1e-5f);
    ovphysx_companion_mass_result_t before = output;
    shape.rotation[3] = 0;
    EXPECT_EQ(api.compute_primitive_mass(&shape, 1, sizeof(output), &output, &invalid),
              OVPHYSX_COMPANION_INVALID_ARGUMENT);
    EXPECT_EQ(invalid, 0u);
    EXPECT_EQ(std::memcmp(&output, &before, sizeof(output)), 0);
}

// @implements REQ-CAPI-COMPANION-001
// @covers AC-2
TEST_F(CompanionBridge, CppWrapperRetainsBusyOwnerAcrossResetAndMove)
{
    ASSERT_TRUE(attach());
    ovphysx_companion_session_t session = 0;
    ASSERT_EQ(api.open_session(m_handle, &session), OVPHYSX_COMPANION_OK);
    sessions.push_back(session);
    ovphysx::PhysX owner(m_handle);
    // Keep fixture ownership even if an assertion fails.
    struct Restore { ovphysx::PhysX& value; ~Restore() { value.release(); } } restore{owner};
    EXPECT_THROW(owner.reset(), std::runtime_error);
    EXPECT_EQ(owner.handle(), m_handle);
    ovphysx::PhysX other;
    owner = std::move(other);
    EXPECT_FALSE(owner);
    EXPECT_EQ(other.handle(), m_handle);
    owner = std::move(other);
    EXPECT_EQ(owner.handle(), m_handle);
    EXPECT_EQ(api.close_session(session), OVPHYSX_COMPANION_OK);
    sessions.pop_back();
}

// @implements REQ-CAPI-COMPANION-001
// @covers AC-4 AC-5
TEST(CompanionCoordinator, IndependentOwnersAndScopedReentry)
{
    CompanionRuntime first;
    CompanionRuntime second;
    {
        CompanionOperationGuard operation(first);
        ASSERT_TRUE(operation);
        CompanionOperationGuard recursive(first);
        EXPECT_FALSE(recursive);
        CompanionTransitionGuard conflicting(first);
        EXPECT_FALSE(conflicting);
        CompanionTransitionGuard independent(second);
        EXPECT_TRUE(independent);
        {
            CompanionNativeAccessScope borrow(first);
            EXPECT_TRUE(first.nativeAccessOnThisThread());
            EXPECT_FALSE(second.nativeAccessOnThisThread());
        }
        EXPECT_FALSE(first.nativeAccessOnThisThread());
    }
    {
        CompanionTransitionGuard outer(first);
        ASSERT_TRUE(outer);
        CompanionTransitionGuard nested(first);
        EXPECT_TRUE(nested);
        CompanionOperationGuard callback(first);
        EXPECT_FALSE(callback);
    }
    CompanionOperationGuard after(first);
    EXPECT_TRUE(after);
    EXPECT_FALSE(first.hasServiceLeases());
    {
        CompanionServiceLease lease(first);
        EXPECT_TRUE(first.hasServiceLeases());
        EXPECT_FALSE(second.hasServiceLeases());
    }
    EXPECT_FALSE(first.hasServiceLeases());
}

TEST(CompanionCoordinator, ConcurrentOperationsExcludeTransitionsUntilBothReturn)
{
    CompanionRuntime runtime;
    std::promise<bool> entered;
    std::promise<void> finish;
    std::future<void> finished = finish.get_future();
    std::thread worker([&] {
        CompanionOperationGuard operation(runtime);
        entered.set_value(bool(operation));
        finished.wait();
    });
    const bool workerAdmitted = entered.get_future().get();
    {
        CompanionOperationGuard peer(runtime);
        EXPECT_TRUE(workerAdmitted);
        EXPECT_TRUE(peer);
        CompanionTransitionGuard rejected(runtime);
        EXPECT_FALSE(rejected);
    }
    {
        CompanionTransitionGuard stillRejected(runtime);
        EXPECT_FALSE(stillRejected);
    }
    finish.set_value();
    worker.join();
    CompanionTransitionGuard idle(runtime);
    EXPECT_TRUE(idle);
}
