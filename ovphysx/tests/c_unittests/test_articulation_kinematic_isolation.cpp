// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-FK-001
 * @covers AC-1 AC-3 AC-4 AC-5
 * @maps_to TEST-TENSOR-FK-001
 *
 * CPU isolation for a selected query: a kinematic update over a query selecting some
 * articulations leaves the others' link poses unchanged.
 *
 * DirectGPU isolation for an articulation a selection leaves out, in a scene shared with
 * repeated-key query rows: the update leaves its link pose and last-step acceleration /
 * incoming joint force unchanged and does not advance the scene timestamp. A held selected
 * query refreshes on every call, and an unselected update refreshes every repeated-key root.
 */

#include "cuda_test_helpers.h"
#include "global_test_environment.h"
#include "session_test_helpers.h"
#include "test_utilities.h"

#include <ovphysx/ovphysx.h>
#include <ovphysx/ovphysx_config.h>

#include <PxPhysicsAPI.h>
#include <cudamanager/PxCudaContext.h>
#include <cudamanager/PxCudaContextManager.h>

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace
{

// On CPU a cache write (the session write) already teleports the links, so these cases leave the links
// stale through PhysX's non-cache joint API instead, which only a kinematic update propagates.
physx::PxArticulationReducedCoordinate* articulationAt(ovphysx_instance_handle_t handle, const char* path)
{
    void* pointer = test_utils::read_physx_pointer(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, path);
    EXPECT_NE(pointer, nullptr) << path;
    return static_cast<physx::PxArticulationReducedCoordinate*>(pointer);
}

std::vector<physx::PxArticulationLink*> linksOf(physx::PxArticulationReducedCoordinate* articulation)
{
    std::vector<physx::PxArticulationLink*> links(articulation->getNbLinks());
    articulation->getLinks(links.data(), static_cast<physx::PxU32>(links.size()));
    return links;
}

std::vector<physx::PxVec3> linkPositions(physx::PxArticulationReducedCoordinate* articulation)
{
    std::vector<physx::PxVec3> positions;
    for (physx::PxArticulationLink* link : linksOf(articulation))
        positions.push_back(link->getGlobalPose().p);
    return positions;
}

void setFreeJointPositions(physx::PxArticulationReducedCoordinate* articulation, float position)
{
    for (physx::PxArticulationLink* link : linksOf(articulation))
    {
        physx::PxArticulationJointReducedCoordinate* joint = link->getInboundJoint();
        if (joint == nullptr)
            continue;
        for (int axis = 0; axis < physx::PxArticulationAxis::eCOUNT; ++axis)
        {
            const physx::PxArticulationAxis::Enum a = static_cast<physx::PxArticulationAxis::Enum>(axis);
            if (joint->getMotion(a) != physx::PxArticulationMotion::eLOCKED)
                joint->setJointPosition(a, position);
        }
    }
}

} // namespace

class SelectedKinematicUpdateCpuTest : public PhysXTestFixture
{
protected:
    // Moves every joint of both articulations in two_articulations.usda, runs the kinematic update over a
    // query selected by `pattern`, and reports whether the links of each articulation moved.
    std::pair<bool, bool> linksMovedAfterSelectedUpdate(const char* pattern)
    {
        EXPECT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"));
        EXPECT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
        physx::PxArticulationReducedCoordinate* a = articulationAt(m_handle, "/World/articulation");
        physx::PxArticulationReducedCoordinate* b = articulationAt(m_handle, "/World/articulation2");
        if (a == nullptr || b == nullptr)
            return { false, false };

        const std::vector<physx::PxVec3> beforeA = linkPositions(a);
        const std::vector<physx::PxVec3> beforeB = linkPositions(b);
        setFreeJointPositions(a, 0.3f);
        setFreeJointPositions(b, 0.3f);
        EXPECT_EQ(linkPositions(a), beforeA) << "the non-cache joint write must leave the links stale";
        EXPECT_EQ(linkPositions(b), beforeB) << "the non-cache joint write must leave the links stale";

        ovphysx_query_selection_t selection{};
        selection.pattern = ovphysx_cstr(pattern);
        ovphysx_query_handle_t query = 0;
        const ovphysx_enqueue_result_t opened =
            ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, &selection, &query, nullptr);
        EXPECT_TRUE(waitForTaskSuccess(m_handle, opened));
        const ovphysx_enqueue_result_t updated =
            ovphysx_update_articulations_kinematic(m_handle, query, OVPHYSX_ARTICULATION_KINEMATIC_POSITION, ovstage_cuda_sync_t{ 0, 0 });
        EXPECT_TRUE(waitForTaskSuccess(m_handle, updated));
        test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query));

        return { linkPositions(a) != beforeA, linkPositions(b) != beforeB };
    }
};

TEST_F(SelectedKinematicUpdateCpuTest, RootSelectionUpdatesOnlyThatArticulation)
{
    EXPECT_EQ(linksMovedAfterSelectedUpdate("/World/articulation"), std::make_pair(true, false));
}

// A joint names its articulation (REQ-READ-SELECT-001 AC-7).
TEST_F(SelectedKinematicUpdateCpuTest, JointSelectionUpdatesItsArticulation)
{
    EXPECT_EQ(linksMovedAfterSelectedUpdate("/World/articulation2/articulatedRevoluteJoint1"),
              std::make_pair(false, true));
}

TEST_F(SelectedKinematicUpdateCpuTest, SelectionOfBothUpdatesBoth)
{
    EXPECT_EQ(linksMovedAfterSelectedUpdate("/World/articulation*"), std::make_pair(true, true));
}

#if OVPHYSX_ENABLE_GPU_TESTS

namespace
{

// The repeated-key /World/Shared articulations (two in scene A, one in scene B) and /World/Other, an articulation of
// its own in scene A. Every articulation is two 1 kg links joined rigidly 1 m apart along x, with no gravity.
constexpr const char* kFixture = "tests/data/shared_root_multiscene_with_peer.usda";
constexpr const char* kOther = "/World/Other";
constexpr const char* kOtherLinks = "/World/Other/*";
constexpr const char* kOtherBase = "/World/Other/base";
constexpr const char* kOtherTip = "/World/Other/tip";
constexpr physx::PxArticulationGPUIndex kInvalidGpuIndex = 0xffffffffu;
constexpr float kOtherBaseX = 100.0f;
constexpr float kWrittenRootX = 200.0f;
constexpr float kTipForceX = 20.0f;

struct LiveQuery
{
    ovphysx_instance_handle_t handle = 0;
    ovphysx_query_handle_t query = 0;

    ~LiveQuery()
    {
        if (handle != 0 && query != 0)
            test_utils::complete_task(handle, ovphysx_release_query(handle, query));
    }
};

struct DeviceWords
{
    physx::PxCudaContext* cuda = nullptr;
    CUdeviceptr address = 0;

    ~DeviceWords()
    {
        if (cuda != nullptr && address != 0)
            cuda->memFree(address);
    }
};

uint64_t articulationQueryCount(ovphysx_instance_handle_t handle)
{
    LiveQuery live;
    live.handle = handle;
    ovstage_query_result_t summary{};
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &live.query, &summary);
    EXPECT_EQ(opened.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(waitForTaskSuccess(handle, opened));
    return summary.total_prim_count;
}

bool allocDevice(physx::PxCudaContext* cuda, DeviceWords& words, size_t bytes)
{
    words.cuda = cuda;
    words.address = 0;
    const physx::PxCUresult allocated = cuda->memAlloc(&words.address, bytes);
    if (allocated != 0)
        words.address = 0;
    EXPECT_EQ(allocated, 0);
    return allocated == 0 && words.address != 0;
}

// A pageable memcpyHtoD can return before its DMA lands, and DirectGPU calls with a null
// startEvent dispatch on PhysX's own stream without waiting for it.
bool uploadToDevice(physx::PxCudaContext* cuda, CUdeviceptr address, const void* source, size_t bytes)
{
    return cuda->memcpyHtoD(address, source, bytes) == 0 && cuda->streamSynchronize(0) == 0;
}

// Reads the root and child link x of `articulation` through the DirectGPU API, first writing its root pose to
// `writtenRootX` when `writeRoot` is set. Only the repeated-key roots need this: a path resolves one of them, so the
// session API cannot address each. A root-pose write does not propagate until forward kinematics runs.
bool readOrWriteLinkX(physx::PxArticulationReducedCoordinate* articulation,
                      bool writeRoot,
                      float& rootX,
                      float& childX,
                      float writtenRootX)
{
    physx::PxScene* scene = articulation->getScene();
    physx::PxCudaContextManager* manager = scene->getCudaContextManager();
    EXPECT_NE(manager, nullptr);
    if (manager == nullptr)
        return false;
    physx::PxScopedCudaLock lock(*manager);
    physx::PxCudaContext* cuda = manager->getCudaContext();
    EXPECT_NE(cuda, nullptr);
    const physx::PxArticulationGPUIndex gpuIndex = articulation->getGPUIndex();
    EXPECT_NE(gpuIndex, kInvalidGpuIndex);
    if (cuda == nullptr || gpuIndex == kInvalidGpuIndex)
        return false;

    const physx::PxU32 linkCount = scene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts().maxLinks;
    EXPECT_GE(linkCount, 2u);
    if (linkCount < 2u)
        return false;

    std::vector<physx::PxTransform> poses(linkCount, physx::PxTransform(physx::PxIdentity));
    DeviceWords indices;
    DeviceWords poseBuffer;
    if (!allocDevice(cuda, indices, sizeof(gpuIndex)) || !allocDevice(cuda, poseBuffer, linkCount * sizeof(physx::PxTransform)))
        return false;
    if (!uploadToDevice(cuda, indices.address, &gpuIndex, sizeof(gpuIndex)))
        return false;

    if (writeRoot)
    {
        poses[0] = physx::PxTransform(physx::PxVec3(writtenRootX, 0.0f, 0.0f));
        if (!uploadToDevice(cuda, poseBuffer.address, poses.data(), sizeof(physx::PxTransform)))
            return false;
        if (!scene->getDirectGPUAPI().setArticulationData(
                reinterpret_cast<const void*>(poseBuffer.address),
                reinterpret_cast<const physx::PxArticulationGPUIndex*>(indices.address),
                physx::PxArticulationGPUAPIWriteType::eROOT_GLOBAL_POSE,
                1))
            return false;
    }

    if (!scene->getDirectGPUAPI().getArticulationData(
            reinterpret_cast<void*>(poseBuffer.address),
            reinterpret_cast<const physx::PxArticulationGPUIndex*>(indices.address),
            physx::PxArticulationGPUAPIReadType::eLINK_GLOBAL_POSE,
            1))
        return false;
    if (cuda->memcpyDtoH(poses.data(), poseBuffer.address, linkCount * sizeof(physx::PxTransform)) != 0)
        return false;
    rootX = poses[0].p.x;
    childX = poses[1].p.x;
    return true;
}

// The x of /World/Other's base and tip, read through the session API.
struct OtherX
{
    float base = 0.0f;
    float tip = 0.0f;
};

bool readOtherX(ovphysx_instance_handle_t handle, uintptr_t ctx, OtherX& out)
{
    session_test::Column positions;
    if (!session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION, positions,
                                  kOtherLinks, ctx))
        return false;
    EXPECT_EQ(positions.size(), 2u);
    if (positions.count(kOtherBase) == 0 || positions.count(kOtherTip) == 0)
        return false;
    out.base = static_cast<float>(positions.at(kOtherBase)[0]);
    out.tip = static_cast<float>(positions.at(kOtherTip)[0]);
    return true;
}

// Moves /World/Other's root to `rootX` on the x axis through the session API, leaving its tip stale.
bool writeOtherRootX(ovphysx_instance_handle_t handle, uintptr_t ctx, float rootX)
{
    return session_test::writeUniform(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION,
                                      { rootX, 0.0, 0.0 }, kOther, ctx) == 1;
}

// /World/Other's last-step link accelerations and the x of the force each link's inbound joint exerts on it.
struct ForceState
{
    float baseAccelerationX = 0.0f;
    float tipAccelerationX = 0.0f;
    float baseIncomingForceX = 0.0f;
    float tipIncomingForceX = 0.0f;
};

bool readForceState(ovphysx_instance_handle_t handle, uintptr_t ctx, ForceState& state)
{
    session_test::Column acceleration;
    session_test::Column incoming;
    if (!session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_LINEAR_ACCELERATION,
                                  acceleration, kOtherLinks, ctx) ||
        !session_test::readColumn(handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK,
                                  OVPHYSX_ATTR_LINK_INCOMING_JOINT_FORCE, incoming, kOtherLinks, ctx))
        return false;
    if (acceleration.count(kOtherBase) == 0 || acceleration.count(kOtherTip) == 0 ||
        incoming.count(kOtherBase) == 0 || incoming.count(kOtherTip) == 0)
        return false;
    state.baseAccelerationX = static_cast<float>(acceleration.at(kOtherBase)[0]);
    state.tipAccelerationX = static_cast<float>(acceleration.at(kOtherTip)[0]);
    state.baseIncomingForceX = static_cast<float>(incoming.at(kOtherBase)[0]);
    state.tipIncomingForceX = static_cast<float>(incoming.at(kOtherTip)[0]);
    return true;
}

void expectForceUnchanged(const ForceState& before, const ForceState& after)
{
    EXPECT_FLOAT_EQ(after.baseAccelerationX, before.baseAccelerationX);
    EXPECT_FLOAT_EQ(after.tipAccelerationX, before.tipAccelerationX);
    EXPECT_FLOAT_EQ(after.baseIncomingForceX, before.baseIncomingForceX);
    EXPECT_FLOAT_EQ(after.tipIncomingForceX, before.tipIncomingForceX);
}

// Runs the kinematic update over every articulation, or over those `pattern` selects.
bool refreshQuery(ovphysx_instance_handle_t handle, const char* pattern = nullptr)
{
    LiveQuery live;
    live.handle = handle;
    ovphysx_enqueue_result_t opened{};
    if (pattern == nullptr)
    {
        opened = ovphysx_query(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &live.query, nullptr);
    }
    else
    {
        ovphysx_query_selection_t selection{};
        selection.pattern = ovphysx_cstr(pattern);
        opened = ovphysx_query_selection(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, &selection, &live.query, nullptr);
    }
    EXPECT_EQ(opened.status, OVPHYSX_API_SUCCESS);
    const ::testing::AssertionResult openedWait = waitForTaskSuccess(handle, opened);
    EXPECT_TRUE(openedWait);
    if (opened.status != OVPHYSX_API_SUCCESS || !openedWait)
        return false;
    const ovphysx_enqueue_result_t updated = ovphysx_update_articulations_kinematic(
        handle,
        live.query,
        OVPHYSX_ARTICULATION_KINEMATIC_POSITION | OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY, ovstage_cuda_sync_t{ 0, 0 });
    EXPECT_EQ(updated.status, OVPHYSX_API_SUCCESS);
    const ::testing::AssertionResult updatedWait = waitForTaskSuccess(handle, updated);
    EXPECT_TRUE(updatedWait);
    return updated.status == OVPHYSX_API_SUCCESS && static_cast<bool>(updatedWait);
}

} // namespace

class SelectedKinematicUpdateGpuTest : public ::testing::Test
{
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;
    uintptr_t m_ctx = 0;

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
        if (s_handle != 0)
            ovphysx_destroy_instance(s_handle);
        s_handle = 0;
    }

    void SetUp() override
    {
        if (s_handle == 0)
        {
            if (ovphysxTestRequireCuda())
                FAIL() << "GPU required: " << s_skipReason;
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override
    {
        if (m_handle != 0)
        {
            const ovphysx_enqueue_result_t reset = ovphysx_detach_ovstage(m_handle);
            EXPECT_EQ(reset.status, OVPHYSX_API_SUCCESS);
            if (reset.op_index != 0)
            {
                EXPECT_TRUE(waitForTaskSuccess(m_handle, reset));
            }
            EXPECT_TRUE(test_utils::destroy_ovstage_test_attachments(m_handle));
        }
    }

    // Attaches the fixture and steps once. Returns scene A, read through its PhysX pointer only for its timestamp
    // and articulation list, or null on failure.
    physx::PxScene* prepareScene()
    {
        if (!test_utils::attach_usd_with_ovstage(m_handle, kFixture))
        {
            ADD_FAILURE() << "failed to attach " << kFixture;
            return nullptr;
        }
        if (ovphysx_step_sync(m_handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
        {
            ADD_FAILURE() << "initial step failed";
            return nullptr;
        }
        // Three runtime articulations share /World/Shared; discovery counts distinct prims.
        EXPECT_EQ(articulationQueryCount(m_handle), 2u);

        physx::PxScene* sceneA =
            static_cast<physx::PxScene*>(test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/sceneA"));
        EXPECT_NE(sceneA, nullptr);
        if (sceneA == nullptr)
            return nullptr;
        EXPECT_EQ(sceneA->getNbArticulations(), 3u);
        m_ctx = session_test::cudaContextOf(m_handle, "/World/sceneA", OVPHYSX_PHYSX_TYPE_SCENE);
        EXPECT_NE(m_ctx, 0u);
        return m_ctx != 0 ? sceneA : nullptr;
    }
};

ovphysx_instance_handle_t SelectedKinematicUpdateGpuTest::s_handle = 0;
std::string SelectedKinematicUpdateGpuTest::s_skipReason;

// A selection of the repeated-key roots leaves /World/Other out of the update: its tip stays where the root write left
// it, and the update does not step the scene.
TEST_F(SelectedKinematicUpdateGpuTest, OutsideQueryChildPoseStaysAtAuthoredX)
{
    physx::PxScene* sceneA = prepareScene();
    ASSERT_NE(sceneA, nullptr);

    OtherX other;
    ASSERT_TRUE(readOtherX(m_handle, m_ctx, other));
    EXPECT_NEAR(other.base, kOtherBaseX, 1.0e-3f);
    EXPECT_NEAR(other.tip, kOtherBaseX + 1.0f, 1.0e-3f);

    ASSERT_TRUE(writeOtherRootX(m_handle, m_ctx, kWrittenRootX));
    ASSERT_TRUE(readOtherX(m_handle, m_ctx, other));
    EXPECT_NEAR(other.base, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(other.tip, kOtherBaseX + 1.0f, 1.0e-3f);

    const physx::PxU32 timestampBefore = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle, "/World/Shared"));
    EXPECT_EQ(sceneA->getTimestamp(), timestampBefore);

    ASSERT_TRUE(readOtherX(m_handle, m_ctx, other));
    EXPECT_NEAR(other.base, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(other.tip, kOtherBaseX + 1.0f, 1.0e-3f);
}

// The same selection leaves /World/Other's last-step acceleration and incoming joint force as the step left them.
TEST_F(SelectedKinematicUpdateGpuTest, OutsideQueryAccelerationAndIncomingForceStay)
{
    physx::PxScene* sceneA = prepareScene();
    ASSERT_NE(sceneA, nullptr);

    ASSERT_EQ(session_test::writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_FORCE,
                                         { kTipForceX, 0.0, 0.0 }, kOtherTip, m_ctx),
              1);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    // The tip force drives the two rigidly joined 1 kg links at kTipForceX / 2 along x.
    ForceState simulated;
    ASSERT_TRUE(readForceState(m_handle, m_ctx, simulated));
    EXPECT_GT(simulated.tipAccelerationX, 0.5f * kTipForceX / 2.0f);

    OtherX before;
    ASSERT_TRUE(readOtherX(m_handle, m_ctx, before));
    ASSERT_TRUE(writeOtherRootX(m_handle, m_ctx, kWrittenRootX));
    OtherX written;
    ASSERT_TRUE(readOtherX(m_handle, m_ctx, written));
    EXPECT_NEAR(written.base, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(written.tip, before.tip, 1.0e-3f);

    ForceState afterWrite;
    ASSERT_TRUE(readForceState(m_handle, m_ctx, afterWrite));
    expectForceUnchanged(simulated, afterWrite);

    const physx::PxU32 timestampBefore = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle, "/World/Shared"));
    EXPECT_EQ(sceneA->getTimestamp(), timestampBefore);

    ForceState afterRefresh;
    ASSERT_TRUE(readForceState(m_handle, m_ctx, afterRefresh));
    expectForceUnchanged(afterWrite, afterRefresh);
}

// The indexed update over a held query reuses its device index list from call to call. Every call must
// still refresh the selected articulation from its current root pose, so each is checked against its own write.
TEST_F(SelectedKinematicUpdateGpuTest, HeldQueryRefreshesOnEveryCall)
{
    ASSERT_NE(prepareScene(), nullptr);

    LiveQuery live;
    live.handle = m_handle;
    ovphysx_query_selection_t selection{};
    selection.pattern = ovphysx_cstr(kOther);
    const ovphysx_enqueue_result_t opened =
        ovphysx_query_selection(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, &selection, &live.query, nullptr);
    ASSERT_EQ(opened.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(waitForTaskSuccess(m_handle, opened));

    for (const float writtenRootX : { kWrittenRootX, kWrittenRootX + 50.0f })
    {
        ASSERT_TRUE(writeOtherRootX(m_handle, m_ctx, writtenRootX));
        OtherX other;
        ASSERT_TRUE(readOtherX(m_handle, m_ctx, other));
        EXPECT_NEAR(other.base, writtenRootX, 1.0e-3f);
        EXPECT_GT(std::fabs(other.tip - (writtenRootX + 1.0f)), 1.0f); // stale until the update runs

        const ovphysx_enqueue_result_t updated = ovphysx_update_articulations_kinematic(
            m_handle,
            live.query,
            OVPHYSX_ARTICULATION_KINEMATIC_POSITION | OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY, ovstage_cuda_sync_t{ 0, 0 });
        ASSERT_EQ(updated.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(waitForTaskSuccess(m_handle, updated));

        ASSERT_TRUE(readOtherX(m_handle, m_ctx, other));
        EXPECT_NEAR(other.base, writtenRootX, 1.0e-3f);
        EXPECT_NEAR(other.tip, writtenRootX + 1.0f, 1.0e-3f);
    }
}

// An unselected update refreshes every articulation of both scenes, including the repeated-key roots that a path
// cannot address one by one. Their roots are written through PhysX for that reason.
TEST_F(SelectedKinematicUpdateGpuTest, RepeatedKeyRootsFollowWholeSceneRefresh)
{
    physx::PxScene* sceneA = prepareScene();
    ASSERT_NE(sceneA, nullptr);
    physx::PxScene* sceneB =
        static_cast<physx::PxScene*>(test_utils::read_physx_pointer(m_handle, OVPHYSX_QUERY_TARGET_SCENE, "/World/sceneB"));
    ASSERT_NE(sceneB, nullptr);

    std::vector<physx::PxArticulationReducedCoordinate*> articulations(sceneA->getNbArticulations() +
                                                                       sceneB->getNbArticulations());
    ASSERT_EQ(articulations.size(), 4u);
    sceneA->getArticulations(articulations.data(), sceneA->getNbArticulations());
    sceneB->getArticulations(articulations.data() + sceneA->getNbArticulations(), sceneB->getNbArticulations());

    std::vector<float> writtenRootX;
    for (physx::PxArticulationReducedCoordinate* articulation : articulations)
    {
        float rootX = 0.0f;
        float childX = 0.0f;
        ASSERT_TRUE(readOrWriteLinkX(articulation, false, rootX, childX, 0.0f));
        writtenRootX.push_back(rootX + kWrittenRootX);
        ASSERT_TRUE(readOrWriteLinkX(articulation, true, rootX, childX, writtenRootX.back()));
        EXPECT_NEAR(rootX, writtenRootX.back(), 1.0e-3f);
        EXPECT_GT(std::fabs(childX - (writtenRootX.back() + 1.0f)), 1.0f); // stale until the update runs
    }

    const physx::PxU32 timestampBefore = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle));
    EXPECT_EQ(sceneA->getTimestamp(), timestampBefore);

    for (size_t i = 0; i < articulations.size(); ++i)
    {
        float rootX = 0.0f;
        float childX = 0.0f;
        ASSERT_TRUE(readOrWriteLinkX(articulations[i], false, rootX, childX, 0.0f));
        EXPECT_NEAR(rootX, writtenRootX[i], 1.0e-3f) << "articulation " << i;
        EXPECT_NEAR(childX, writtenRootX[i] + 1.0f, 1.0e-3f) << "articulation " << i;
    }
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
