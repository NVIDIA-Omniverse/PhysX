// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-FK-001
 * @covers AC-1 AC-3 AC-4
 * @maps_to TEST-TENSOR-FK-001
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-5
 * @maps_to TEST-INPUT-CORE-001
 *
 * CPU isolation for a selected query: a kinematic update over a query selecting some
 * articulations leaves the others' link poses unchanged.
 *
 * DirectGPU isolation for a caller-created articulation that shares a scene with
 * repeated-key query rows but is absent from the query and from the structural
 * cache. The public kinematic update must leave that articulation's link pose
 * and last-step acceleration / incoming joint force unchanged, and must not
 * advance the scene timestamp.
 *
 * The same isolation for an unselected link-wrench write: the attachment does not
 * own the caller-created articulation, so the write must not clear the link force
 * applied to it.
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

#include <algorithm>
#include <cmath>
#include <cstdint>
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
            ovphysx_update_articulations_kinematic(m_handle, query, OVPHYSX_ARTICULATION_KINEMATIC_POSITION);
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

constexpr const char* kFixture = "tests/data/shared_root_multiscene.usda";
constexpr const char* kWrenchFixture = "tests/data/two_articulations_gpu.usda";
constexpr physx::PxArticulationGPUIndex kInvalidGpuIndex = 0xffffffffu;
constexpr float kRootX = 100.0f;
constexpr float kChildX = 101.0f;
constexpr float kWrittenRootX = 200.0f;
constexpr float kChildForceX = 20.0f;

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

void* physxPointer(ovphysx_instance_handle_t handle, const char* path, ovphysx_query_target_t type)
{
    void* pointer = type == OVPHYSX_QUERY_TARGET_PHYSICS
                        ? test_utils::read_physics_singleton(handle)
                        : test_utils::read_physx_pointer(handle, type, path);
    EXPECT_NE(pointer, nullptr) << (path ? path : "") << " type " << static_cast<int>(type);
    return pointer;
}

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

physx::PxArticulationReducedCoordinate* createNativeArticulation(physx::PxPhysics* physics, physx::PxScene* scene)
{
    physx::PxArticulationReducedCoordinate* articulation = physics->createArticulationReducedCoordinate();
    EXPECT_NE(articulation, nullptr);
    if (articulation == nullptr)
        return nullptr;

    physx::PxArticulationLink* root = articulation->createLink(nullptr, physx::PxTransform(physx::PxVec3(kRootX, 0.0f, 0.0f)));
    physx::PxArticulationLink* child = articulation->createLink(root, physx::PxTransform(physx::PxVec3(kChildX, 0.0f, 0.0f)));
    EXPECT_NE(root, nullptr);
    EXPECT_NE(child, nullptr);
    if (root == nullptr || child == nullptr)
    {
        articulation->release();
        return nullptr;
    }
    root->setMass(1.0f);
    root->setMassSpaceInertiaTensor(physx::PxVec3(1.0f));
    child->setMass(1.0f);
    child->setMassSpaceInertiaTensor(physx::PxVec3(1.0f));
    physx::PxArticulationJointReducedCoordinate* joint = child->getInboundJoint();
    EXPECT_NE(joint, nullptr);
    if (joint == nullptr)
    {
        articulation->release();
        return nullptr;
    }
    joint->setJointType(physx::PxArticulationJointType::eFIX);
    joint->setParentPose(physx::PxTransform(physx::PxVec3(1.0f, 0.0f, 0.0f)));
    joint->setChildPose(physx::PxTransform(physx::PxIdentity));
    EXPECT_TRUE(scene->addArticulation(*articulation));
    return articulation;
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

// Link 0 is the native root and link 1 is its child. A root-pose write does not
// propagate until forward kinematics runs.
bool readOrWriteLinkX(physx::PxArticulationReducedCoordinate* articulation, bool writeRoot, float& rootX, float& childX)
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
        poses[0] = physx::PxTransform(physx::PxVec3(kWrittenRootX, 0.0f, 0.0f));
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

// Incoming joint force is two PxVec3 values per link (force, then torque).
// Child force is therefore index 2, not index 1.
struct ForceState
{
    float rootAccelerationX = 0.0f;
    float childAccelerationX = 0.0f;
    float rootIncomingForceX = 0.0f;
    float childIncomingForceX = 0.0f;
};

bool applyChildForce(physx::PxArticulationReducedCoordinate* articulation)
{
    physx::PxScene* scene = articulation->getScene();
    physx::PxCudaContextManager* manager = scene->getCudaContextManager();
    EXPECT_NE(manager, nullptr);
    if (manager == nullptr)
        return false;
    physx::PxScopedCudaLock lock(*manager);
    physx::PxCudaContext* cuda = manager->getCudaContext();
    const physx::PxArticulationGPUIndex gpuIndex = articulation->getGPUIndex();
    EXPECT_NE(gpuIndex, kInvalidGpuIndex);
    const physx::PxU32 linkCount = scene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts().maxLinks;
    EXPECT_GE(linkCount, 2u);
    if (cuda == nullptr || gpuIndex == kInvalidGpuIndex || linkCount < 2u)
        return false;

    std::vector<physx::PxVec3> forces(linkCount, physx::PxVec3(0.0f));
    forces[1].x = kChildForceX;
    DeviceWords indices;
    DeviceWords data;
    if (!allocDevice(cuda, indices, sizeof(gpuIndex)) || !allocDevice(cuda, data, forces.size() * sizeof(physx::PxVec3)))
        return false;
    if (!uploadToDevice(cuda, indices.address, &gpuIndex, sizeof(gpuIndex)))
        return false;
    if (!uploadToDevice(cuda, data.address, forces.data(), forces.size() * sizeof(physx::PxVec3)))
        return false;
    return scene->getDirectGPUAPI().setArticulationData(
        reinterpret_cast<const void*>(data.address),
        reinterpret_cast<const physx::PxArticulationGPUIndex*>(indices.address),
        physx::PxArticulationGPUAPIWriteType::eLINK_FORCE,
        1);
}

bool readForceState(physx::PxArticulationReducedCoordinate* articulation, ForceState& state)
{
    physx::PxScene* scene = articulation->getScene();
    physx::PxCudaContextManager* manager = scene->getCudaContextManager();
    EXPECT_NE(manager, nullptr);
    if (manager == nullptr)
        return false;
    physx::PxScopedCudaLock lock(*manager);
    physx::PxCudaContext* cuda = manager->getCudaContext();
    const physx::PxArticulationGPUIndex gpuIndex = articulation->getGPUIndex();
    const physx::PxU32 linkCount = scene->getDirectGPUAPI().getArticulationGPUAPIMaxCounts().maxLinks;
    EXPECT_GE(linkCount, 2u);
    if (cuda == nullptr || gpuIndex == kInvalidGpuIndex || linkCount < 2u)
        return false;

    const physx::PxU32 spatialCount = linkCount * 2u;
    std::vector<physx::PxVec3> values(spatialCount, physx::PxVec3(0.0f));
    DeviceWords indices;
    DeviceWords data;
    if (!allocDevice(cuda, indices, sizeof(gpuIndex)) || !allocDevice(cuda, data, values.size() * sizeof(physx::PxVec3)))
        return false;
    if (!uploadToDevice(cuda, indices.address, &gpuIndex, sizeof(gpuIndex)))
        return false;

    if (!scene->getDirectGPUAPI().getArticulationData(
            reinterpret_cast<void*>(data.address),
            reinterpret_cast<const physx::PxArticulationGPUIndex*>(indices.address),
            physx::PxArticulationGPUAPIReadType::eLINK_LINEAR_ACCELERATION,
            1))
        return false;
    if (cuda->memcpyDtoH(values.data(), data.address, linkCount * sizeof(physx::PxVec3)) != 0)
        return false;
    state.rootAccelerationX = values[0].x;
    state.childAccelerationX = values[1].x;

    if (!scene->getDirectGPUAPI().getArticulationData(
            reinterpret_cast<void*>(data.address),
            reinterpret_cast<const physx::PxArticulationGPUIndex*>(indices.address),
            physx::PxArticulationGPUAPIReadType::eLINK_INCOMING_JOINT_FORCE,
            1))
        return false;
    if (cuda->memcpyDtoH(values.data(), data.address, values.size() * sizeof(physx::PxVec3)) != 0)
        return false;
    state.rootIncomingForceX = values[0].x;
    state.childIncomingForceX = values[2].x;
    return true;
}

void expectForceUnchanged(const ForceState& before, const ForceState& after)
{
    EXPECT_FLOAT_EQ(after.rootAccelerationX, before.rootAccelerationX);
    EXPECT_FLOAT_EQ(after.childAccelerationX, before.childAccelerationX);
    EXPECT_FLOAT_EQ(after.rootIncomingForceX, before.rootIncomingForceX);
    EXPECT_FLOAT_EQ(after.childIncomingForceX, before.childIncomingForceX);
}

bool refreshQuery(ovphysx_instance_handle_t handle)
{
    LiveQuery live;
    live.handle = handle;
    const ovphysx_enqueue_result_t opened =
        ovphysx_query(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_SCOPE_ALL, &live.query, nullptr);
    EXPECT_EQ(opened.status, OVPHYSX_API_SUCCESS);
    const ::testing::AssertionResult openedWait = waitForTaskSuccess(handle, opened);
    EXPECT_TRUE(openedWait);
    if (opened.status != OVPHYSX_API_SUCCESS || !openedWait)
        return false;
    const ovphysx_enqueue_result_t updated = ovphysx_update_articulations_kinematic(
        handle,
        live.query,
        OVPHYSX_ARTICULATION_KINEMATIC_POSITION | OVPHYSX_ARTICULATION_KINEMATIC_VELOCITY);
    EXPECT_EQ(updated.status, OVPHYSX_API_SUCCESS);
    const ::testing::AssertionResult updatedWait = waitForTaskSuccess(handle, updated);
    EXPECT_TRUE(updatedWait);
    return updated.status == OVPHYSX_API_SUCCESS && static_cast<bool>(updatedWait);
}

} // namespace

class NativeArticulationIsolationGpuTest : public ::testing::Test
{
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;
    physx::PxArticulationReducedCoordinate* m_native = nullptr;
    physx::PxArticulationReducedCoordinate* m_displaced = nullptr;
    physx::PxScene* m_displacedScene = nullptr;

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
        if (m_native != nullptr)
        {
            m_native->release();
            m_native = nullptr;
        }
        if (m_displaced != nullptr)
        {
            EXPECT_TRUE(m_displacedScene->addArticulation(*m_displaced));
            m_displaced = nullptr;
        }
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

    physx::PxScene* prepareSharedRootScene(bool replaceRuntime = false)
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
        EXPECT_EQ(articulationQueryCount(m_handle), 1u);

        physx::PxScene* sceneA = static_cast<physx::PxScene*>(physxPointer(m_handle, "/World/sceneA", OVPHYSX_QUERY_TARGET_SCENE));
        physx::PxScene* sceneB = static_cast<physx::PxScene*>(physxPointer(m_handle, "/World/sceneB", OVPHYSX_QUERY_TARGET_SCENE));
        physx::PxPhysics* physics = static_cast<physx::PxPhysics*>(physxPointer(m_handle, nullptr, OVPHYSX_QUERY_TARGET_PHYSICS));
        physx::PxArticulationReducedCoordinate* selected = static_cast<physx::PxArticulationReducedCoordinate*>(
            physxPointer(m_handle, "/World/Shared", OVPHYSX_QUERY_TARGET_ARTICULATION));
        EXPECT_NE(sceneA, nullptr);
        EXPECT_NE(sceneB, nullptr);
        EXPECT_NE(physics, nullptr);
        EXPECT_NE(selected, nullptr);
        if (sceneA == nullptr || sceneB == nullptr || physics == nullptr || selected == nullptr)
            return nullptr;

        EXPECT_EQ(selected->getScene(), sceneA);
        EXPECT_EQ(sceneA->getNbArticulations(), 2u);
        EXPECT_EQ(sceneB->getNbArticulations(), 1u);
        EXPECT_NE(sceneA->getCudaContextManager(), nullptr);

        // Prove complete coverage, then exercise its reuse before native membership changes.
        if (!refreshQuery(m_handle) || !refreshQuery(m_handle))
            return nullptr;
        if (replaceRuntime)
        {
            physx::PxArticulationReducedCoordinate* roots[2] = {};
            if (sceneA->getArticulations(roots, 2) != 2u)
                return nullptr;
            // Keep the path-resolved root live so the indexed fallback remains valid.
            m_displaced = roots[0] == selected ? roots[1] : roots[0];
            m_displacedScene = sceneA;
            sceneA->removeArticulation(*m_displaced);
        }
        m_native = createNativeArticulation(physics, sceneA);
        EXPECT_NE(m_native, nullptr);
        EXPECT_EQ(sceneA->getNbArticulations(), replaceRuntime ? 2u : 3u);
        EXPECT_EQ(sceneB->getNbArticulations(), 1u);
        if (m_native == nullptr)
            return nullptr;

        if (ovphysx_step_sync(m_handle, 1.0f / 60.0f).status != OVPHYSX_API_SUCCESS)
        {
            ADD_FAILURE() << "step after adding the native articulation failed";
            return nullptr;
        }
        EXPECT_NE(m_native->getGPUIndex(), kInvalidGpuIndex);
        EXPECT_EQ(articulationQueryCount(m_handle), 1u);
        return sceneA;
    }
};

ovphysx_instance_handle_t NativeArticulationIsolationGpuTest::s_handle = 0;
std::string NativeArticulationIsolationGpuTest::s_skipReason;

TEST_F(NativeArticulationIsolationGpuTest, OutsideQueryChildPoseStaysAtAuthoredX)
{
    physx::PxScene* sceneA = prepareSharedRootScene();
    ASSERT_NE(sceneA, nullptr);
    ASSERT_NE(m_native, nullptr);

    float rootX = 0.0f;
    float childX = 0.0f;
    ASSERT_TRUE(readOrWriteLinkX(m_native, false, rootX, childX));
    EXPECT_NEAR(rootX, kRootX, 1.0e-3f);
    EXPECT_NEAR(childX, kChildX, 1.0e-3f);

    float writtenRootX = 0.0f;
    float writtenChildX = 0.0f;
    ASSERT_TRUE(readOrWriteLinkX(m_native, true, writtenRootX, writtenChildX));
    EXPECT_NEAR(writtenRootX, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(writtenChildX, kChildX, 1.0e-3f);

    const physx::PxU32 timestampBefore = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle));
    EXPECT_EQ(sceneA->getTimestamp(), timestampBefore);

    float afterRootX = 0.0f;
    float afterChildX = 0.0f;
    ASSERT_TRUE(readOrWriteLinkX(m_native, false, afterRootX, afterChildX));
    EXPECT_NEAR(afterRootX, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(afterChildX, kChildX, 1.0e-3f);
}

TEST_F(NativeArticulationIsolationGpuTest, OutsideQueryAccelerationAndIncomingForceStay)
{
    physx::PxScene* sceneA = prepareSharedRootScene();
    ASSERT_NE(sceneA, nullptr);
    ASSERT_NE(m_native, nullptr);

    ASSERT_TRUE(applyChildForce(m_native));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(articulationQueryCount(m_handle), 1u);

    ForceState simulated;
    ASSERT_TRUE(readForceState(m_native, simulated));
    const float peak = std::max(
        std::max(std::fabs(simulated.rootAccelerationX), std::fabs(simulated.childAccelerationX)),
        std::max(std::fabs(simulated.rootIncomingForceX), std::fabs(simulated.childIncomingForceX)));
    EXPECT_GT(peak, 1.0f);

    float beforeRootX = 0.0f;
    float beforeChildX = 0.0f;
    ASSERT_TRUE(readOrWriteLinkX(m_native, false, beforeRootX, beforeChildX));
    float writtenRootX = 0.0f;
    float writtenChildX = 0.0f;
    ASSERT_TRUE(readOrWriteLinkX(m_native, true, writtenRootX, writtenChildX));
    EXPECT_NEAR(writtenRootX, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(writtenChildX, beforeChildX, 1.0e-3f);

    ForceState afterWrite;
    ASSERT_TRUE(readForceState(m_native, afterWrite));
    expectForceUnchanged(simulated, afterWrite);

    const physx::PxU32 timestampBefore = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle));
    EXPECT_EQ(sceneA->getTimestamp(), timestampBefore);

    ForceState afterRefresh;
    ASSERT_TRUE(readForceState(m_native, afterRefresh));
    expectForceUnchanged(afterWrite, afterRefresh);
}

TEST_F(NativeArticulationIsolationGpuTest, SameCountReplacementInvalidatesWholeSceneCoverage)
{
    physx::PxScene* sceneA = prepareSharedRootScene(/* replaceRuntime = */ true);
    ASSERT_NE(sceneA, nullptr);
    ASSERT_NE(m_native, nullptr);
    ASSERT_NE(m_displaced, nullptr);
    EXPECT_EQ(sceneA->getNbArticulations(), 2u);
    EXPECT_EQ(m_displaced->getScene(), nullptr);

    float rootX = 0.0f;
    float childX = 0.0f;
    ASSERT_TRUE(readOrWriteLinkX(m_native, true, rootX, childX));
    EXPECT_NEAR(rootX, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(childX, kChildX, 1.0e-3f);

    const physx::PxU32 timestampBefore = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle));
    EXPECT_EQ(sceneA->getTimestamp(), timestampBefore);
    ASSERT_TRUE(readOrWriteLinkX(m_native, false, rootX, childX));
    EXPECT_NEAR(rootX, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(childX, kChildX, 1.0e-3f);

    // Restoring the runtime root must make every repeated-key articulation eligible again.
    m_native->release();
    m_native = nullptr;
    physx::PxArticulationReducedCoordinate* restored = m_displaced;
    ASSERT_TRUE(sceneA->addArticulation(*restored));
    m_displaced = nullptr;
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(readOrWriteLinkX(restored, true, rootX, childX));
    EXPECT_NEAR(rootX, kWrittenRootX, 1.0e-3f);
    EXPECT_GT(std::fabs(childX - (kWrittenRootX + 1.0f)), 1.0f);

    const physx::PxU32 restoredTimestamp = sceneA->getTimestamp();
    ASSERT_TRUE(refreshQuery(m_handle));
    EXPECT_EQ(sceneA->getTimestamp(), restoredTimestamp);
    ASSERT_TRUE(readOrWriteLinkX(restored, false, rootX, childX));
    EXPECT_NEAR(rootX, kWrittenRootX, 1.0e-3f);
    EXPECT_NEAR(childX, kWrittenRootX + 1.0f, 1.0e-3f);
}

// An unselected write covers every link the attachment owns, but not the caller-created articulation that
// shares its scene. Applying the write to the whole native view would zero that articulation's link force
// before the step.
TEST_F(NativeArticulationIsolationGpuTest, OutsideQueryLinkForceSurvivesUnselectedWrenchWrite)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, kWrenchFixture));
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    physx::PxScene* scene =
        static_cast<physx::PxScene*>(physxPointer(m_handle, "/World/physicsScene", OVPHYSX_QUERY_TARGET_SCENE));
    physx::PxPhysics* physics = static_cast<physx::PxPhysics*>(physxPointer(m_handle, nullptr, OVPHYSX_QUERY_TARGET_PHYSICS));
    ASSERT_NE(scene, nullptr);
    ASSERT_NE(physics, nullptr);
    m_native = createNativeArticulation(physics, scene);
    ASSERT_NE(m_native, nullptr);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);
    ASSERT_NE(m_native->getGPUIndex(), kInvalidGpuIndex);

    const uintptr_t ctx = session_test::cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    ASSERT_TRUE(applyChildForce(m_native));
    ASSERT_GT(session_test::writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_WRENCH, { 0.0 },
                                         nullptr, ctx),
              0);
    ASSERT_EQ(ovphysx_step_sync(m_handle, 1.0f / 60.0f).status, OVPHYSX_API_SUCCESS);

    // The child force drives the two rigidly joined 1 kg links at kChildForceX / 2 along x; zeroed, it is ~0.
    ForceState state;
    ASSERT_TRUE(readForceState(m_native, state));
    EXPECT_GT(state.childAccelerationX, 0.5f * kChildForceX / 2.0f);
}

#endif // OVPHYSX_ENABLE_GPU_TESTS
