// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-INPUT-COVERAGE-001
 * @covers AC-1
 * @maps_to TEST-INPUT-COVERAGE-001
 *
 * @implements REQ-INPUT-CORE-001
 * @covers AC-4
 * @maps_to TEST-INPUT-CORE-001
 *
 * @implements REQ-CAPI-WRITE-001
 * @covers AC-1
 * @maps_to TEST-CAPI-WRITE-001
 *
 * @implements REQ-READ-SELECT-001
 * @covers AC-4 AC-8
 * @maps_to TEST-READ-SELECT-001
 *
 * @implements REQ-READ-ATTRS-001
 * @covers AC-1 AC-21
 * @maps_to TEST-READ-ATTRS-001
 *
 * @implements REQ-READ-ARTICULATION-001
 * @covers AC-4
 * @maps_to TEST-READ-ARTICULATION-001
 *
 * @implements REQ-READ-INVDYN-001
 * @covers AC-1 AC-5
 * @maps_to TEST-READ-INVDYN-001
 *
 * @implements REQ-READ-MATERIAL-001
 * @covers AC-1
 * @maps_to TEST-READ-MATERIAL-001
 */
// Session read/write physics tests: the successor coverage for the physics facts the deprecated tensor-binding
// tests checked (disable gravity/simulation, wake, forces and wrenches, drive type, projected joint force,
// centroidal momentum, deformables, selected writes). The binding versions live in test_tensor_binding.cpp and
// retire with the binding; these go through ovphysx_read / ovphysx_write and stay.

#include "global_test_environment.h"
#include "test_utilities.h"
#include "ovphysx/ovphysx.h"
#include "ovphysx/ovphysx_attributes.h"
#include "ovphysx/ovphysx_config.h"
#include "ovphysx_test_utils.h"
#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include <cstring>
#include <cmath>
#include <limits>

#include <carb/Framework.h>
#include <omni/physics/tensors/TensorApi.h>
#include <omni/physx/IOptionalCuda.h>
#include "ovphysxTestHelpers.h"

#include "cuda_test_helpers.h"
#include "session_test_helpers.h"

#include "WrenchConversion.h"

using test_utils::make_ovx_string;
using test_utils::make_ovx_string_bytes;



static bool wait_task_success(ovphysx_instance_handle_t handle, ovphysx_enqueue_result_t task, uint64_t timeout_ns = 10'000'000'000ULL) {
    ovphysx_wait_task_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_task(handle, task, timeout_ns, &wait_result);
    const bool task_had_errors = wait_result.error_tasks && wait_result.num_errors > 0;
    if (task_had_errors) {
        for (size_t i = 0; i < wait_result.num_errors; i++) {
            ovphysx_string_t op_err = ovphysx_get_last_task_error(handle, wait_result.error_tasks[i]);
            std::cerr << "Op error: " << std::string(op_err.ptr, op_err.length) << std::endl;
        }
    }
    ovphysx_destroy_wait_task_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS && !task_had_errors;
}



// A pattern selecting exactly the prims `paths` name under one parent: an alternation over their leaf names.
static std::string subsetPattern(const std::string& parent, const std::vector<std::string>& paths)
{
    std::string pattern = parent + "/(";
    for (size_t i = 0; i < paths.size(); ++i)
        pattern += (i ? "|" : "") + paths[i].substr(paths[i].rfind('/') + 1);
    return pattern + ")";
}

// Splits the prims of a column into alternating halves in prim-path order: the first, third, ... prim and
// the second, fourth, ... prim.
static void alternateHalves(const session_test::Column& column, std::vector<std::string>& even,
                            std::vector<std::string>& odd)
{
    even.clear();
    odd.clear();
    for (const std::pair<const std::string, std::vector<double>>& prim : column)
        (even.size() == odd.size() ? even : odd).push_back(prim.first);
}

namespace
{

// Count per-stage resets without changing the public binding API.
class ScopedTensorApiProbe
{
public:
    explicit ScopedTensorApiProbe(omni::physics::tensors::TensorApi& tensorApi)
        : mTensorApi(tensorApi), mOriginal(tensorApi.resetStage)
    {
        sOriginal = mOriginal;
        sCallCount = 0;
        sLastStageId = 0;
        mTensorApi.resetStage = &intercept;
    }

    ~ScopedTensorApiProbe()
    {
        mTensorApi.resetStage = mOriginal;
        sOriginal = nullptr;
    }

    ScopedTensorApiProbe(const ScopedTensorApiProbe&) = delete;
    ScopedTensorApiProbe& operator=(const ScopedTensorApiProbe&) = delete;

    int resetCount() const
    {
        return sCallCount;
    }

    omni::physics::AttachHandle lastStageId() const
    {
        return sLastStageId;
    }

private:
    static bool CARB_ABI intercept(omni::physics::AttachHandle stageId)
    {
        ++sCallCount;
        sLastStageId = stageId;
        return sOriginal && sOriginal(stageId);
    }

    omni::physics::tensors::TensorApi& mTensorApi;
    bool(CARB_ABI* mOriginal)(omni::physics::AttachHandle) = nullptr;

    static inline bool(CARB_ABI* sOriginal)(omni::physics::AttachHandle) = nullptr;
    static inline int sCallCount = 0;
    static inline omni::physics::AttachHandle sLastStageId = 0;
};

} // namespace

// ============================================================================
// CPU MODE TESTS
// ============================================================================

class SessionPhysicsCpuTest : public ::testing::Test {
protected:
    ovphysx_instance_handle_t m_handle = 0;

    void SetUp() override {
        ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create shared CPU PhysX instance";
        m_handle = sharedCpuInstance();
    }

    void TearDown() override {
        if (m_handle != 0) {
            ovphysx_enqueue_result_t reset_result = ovphysx_detach_ovstage(m_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                EXPECT_TRUE(waitForTaskSuccess(m_handle, reset_result))
                    << "SessionPhysicsCpuTest reset failed during teardown";
            }
            test_utils::destroy_ovstage_test_attachments(m_handle);
            m_handle = 0;
        }
    }
};



// ovphysx_detach_ovstage must release the tensor SimulationBackend's per-stage data, or stale
// views persist across reset and reattach. The reset path calls resetStage(stageId) in
// detach_ovstage / unload_usd.
TEST_F(SessionPhysicsCpuTest, ResetStageReleasesTensorBackendStage)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

    // A session read builds the backend's per-stage views.
    session_test::Column velocity;
    ASSERT_TRUE(session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, velocity,
                                         "/World/Cube1"));
    ASSERT_EQ(velocity.size(), 1u);

    omni::physics::tensors::TensorApi* tensorApi =
        static_cast<omni::physics::tensors::TensorApi*>(ovphysx_get_tensor_api_internal());
    ASSERT_NE(tensorApi, nullptr);
    ASSERT_NE(tensorApi->resetStage, nullptr);
    ScopedTensorApiProbe probe(*tensorApi);

    ovphysx_enqueue_result_t resetResult = ovphysx_detach_ovstage(m_handle);
    ASSERT_EQ(resetResult.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_task_success(m_handle, resetResult));

    EXPECT_EQ(probe.resetCount(), 1);
    EXPECT_GT(probe.lastStageId(), 0);
}

// Session counterpart of the read-only checks in CpuArticulationMassCenterReadOnly, the centroidal-momentum
// tests above and test_tensor_types_extended.cpp's link write rejections: the session refuses to open a
// write of a read-only attribute, and the mass center stays readable.
TEST_F(SessionPhysicsCpuTest, SessionReadOnlyArticulationAttributesRefuseWrites) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample.usda"))
        << "Failed to load USD";
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    for (const char* attribute : { OVPHYSX_ATTR_POSITION, OVPHYSX_ATTR_ORIENTATION, OVPHYSX_ATTR_INVERSE_MASS,
                                   OVPHYSX_ATTR_INVERSE_INERTIA })
        EXPECT_NE(writeOpenStatus(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, attribute), OVPHYSX_API_SUCCESS)
            << "link " << attribute << " must be read-only";
    for (const char* attribute : { OVPHYSX_ATTR_CENTER_OF_MASS_WORLD, OVPHYSX_ATTR_CENTER_OF_MASS_LOCAL,
                                   OVPHYSX_ATTR_CENTROIDAL_MOMENTUM })
        EXPECT_NE(writeOpenStatus(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, attribute), OVPHYSX_API_SUCCESS)
            << "articulation " << attribute << " must be read-only";
    EXPECT_NE(
        writeOpenStatus(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE),
        OVPHYSX_API_SUCCESS)
        << "the projected joint force must be read-only";

    // One world and one local mass-center point per articulation, with no fixed/floating-base restriction.
    for (const char* attribute : { OVPHYSX_ATTR_CENTER_OF_MASS_WORLD, OVPHYSX_ATTR_CENTER_OF_MASS_LOCAL })
    {
        Column com;
        ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, attribute, com, "/World/articulation"))
            << attribute;
        ASSERT_EQ(com.size(), 1u) << attribute;
        EXPECT_EQ(com.begin()->second.size(), 3u) << attribute;
    }
}


// Session counterpart of the three CpuArticulationCentroidalMomentum* binding tests. The session read
// SERVES centroidal momentum for a floating-base articulation (one flat 6*(maxDofs+7) row per articulation),
// and, unlike the binding, expresses "undefined for a fixed base" by OMITTING the fixed-base cohort from the
// read rather than rejecting the whole request. mixed_base_articulations.usda holds both a fixed-base
// (/World/articulation) and a floating-base (/World/articulation2) articulation, so one read shows the
// floating one served with the right shape and the fixed one omitted.
TEST_F(SessionPhysicsCpuTest, SessionArticulationCentroidalMomentumServedFloatingOmittedFixed) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/mixed_base_articulations.usda"))
        << "Failed to load mixed_base_articulations.usda";
    ASSERT_TRUE(stepN(m_handle, 1)); // inverse-dynamics columns are sized on the first simulate

    Column momentum;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CENTROIDAL_MOMENTUM, momentum));

    // Only the floating-base articulation is served; the fixed-base one is omitted, not an error.
    ASSERT_EQ(momentum.size(), 1u) << "only the floating-base articulation serves centroidal momentum";
    EXPECT_EQ(momentum.count("/World/articulation2"), 1u) << "the floating-base articulation must be served";
    EXPECT_EQ(momentum.count("/World/articulation"), 0u) << "the fixed-base articulation must be omitted";

    // The served row is a flat 6*(maxDofs+7) momentum matrix: 6 rows, at least 7 columns.
    const std::vector<double>& row = momentum.at("/World/articulation2");
    ASSERT_GT(row.size(), 0u);
    EXPECT_EQ(row.size() % 6u, 0u) << "centroidal momentum is 6 rows of maxDofs+7 columns";
    EXPECT_GE(row.size() / 6u, 7u) << "column count is maxDofs + 7, at least 7";
}

// Projected joint forces read before the first full step must be about 0. The session serves an
// articulation once the first simulate has inserted it, so the read follows the warmup, a 1 ns step.
TEST_F(SessionPhysicsCpuTest, CpuCartPoleProjectedJointForcePreStepZero) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/CartPole.usda"))
        << "Failed to load CartPole.usda";

    // Articulations become readable once the first simulate has inserted them (the session's
    // step-first precondition); warmup is that first simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    Column dof_forces;
    ASSERT_TRUE(
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, dof_forces));
    ASSERT_EQ(dof_forces.size(), 2u) << "CartPole has 2 DOFs (cart prismatic + pole revolute)";

    for (const std::pair<const std::string, std::vector<double>>& joint : dof_forces)
        for (double force : joint.second)
            EXPECT_LT(std::abs(force), 1e-3) << joint.first << " projection before the first full step: expected ~0, got " << force;
}

// Drive both DOFs with a 10N/10N.m actuation, step at the umbrella's
// timestep, and read the projected joint forces. Used as the baseline
// for the cart-prismatic step-1 readback comparison with the GPU twin.
TEST_F(SessionPhysicsCpuTest, CpuCartPoleProjectedJointForceMatchesActuation) {
    using namespace session_test;
    // CartPoleForcesOncePerStep.usda is CartPole.usda with external forces applied once per step,
    // which this test expects.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/CartPoleForcesOncePerStep.usda"))
        << "Failed to load CartPoleForcesOncePerStep.usda";

    // Articulations become readable once the first simulate has inserted them (the session's
    // step-first precondition); warmup is that first simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ACTUATION_FORCE, { 10.0 }),
              2);

    ASSERT_TRUE(stepN(m_handle, 1, 1.0f / 1000.0f));

    Column projected;
    ASSERT_TRUE(
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, projected));
    const double cart = projected["/cartpole/cartJoint"].at(0);
    const double pole = projected["/cartpole/poleJoint"].at(0);

    std::cerr << "[CPU cartpole step-1] cart-prismatic=" << cart << " pole-revolute=" << pole << "\n";

    // Tolerance matches the umbrella's np.allclose(rtol=0.04, atol=0.5).
    EXPECT_NEAR(cart, 10.0, 0.9) << "cart-prismatic projection diverged";
    EXPECT_NEAR(pole, 10.0, 0.9) << "pole-revolute projection diverged";
}

// Per-body disableSimulation flag round trip (OMPE-94459). The write toggles
// PxActorFlag::eDISABLE_SIMULATION on the PxRigidActor, so the flag takes effect on the
// next solver step.
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableSimulationRoundtrip) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    // Alternate 1/0 in prim-path order to exercise both set and cleared flags.
    std::map<std::string, double> written;
    ASSERT_GT(writeColumn(
                  m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values[0] = written.size() % 2 == 0 ? 1.0 : 0.0;
                      written[path] = values[0];
                  },
                  "/World/Cube*"),
              1);

    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, readback, "/World/Cube*", 0, &kDtypeUInt8));
    ASSERT_EQ(readback.size(), written.size());
    for (const std::pair<const std::string, double>& flag : written)
        EXPECT_EQ(readback[flag.first][0], flag.second) << flag.first << " disable flag round-trip";
}

TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableGravityRoundtrip) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    // Alternate 1/0 in prim-path order to exercise both set and cleared flags.
    std::map<std::string, double> written;
    ASSERT_GT(writeColumn(
                  m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values[0] = written.size() % 2 == 0 ? 1.0 : 0.0;
                      written[path] = values[0];
                  },
                  "/World/Cube*"),
              1);

    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, readback, "/World/Cube*", 0, &kDtypeUInt8));
    ASSERT_EQ(readback.size(), written.size());
    for (const std::pair<const std::string, double>& flag : written)
        EXPECT_EQ(readback[flag.first][0], flag.second) << flag.first << " disable-gravity round-trip";
}

TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableGravitySuppressesFall) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    const char* cubes = "/World/Cube*";
    Column poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    ASSERT_FALSE(poses.empty());
    const std::string first = poses.begin()->first;
    const double z_initial = poses[first][2];

    // Let bodies fall briefly so disabling gravity can be checked without a
    // separate solver-refreshing write.
    ASSERT_TRUE(stepN(m_handle, 5));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    const double z_falling = poses[first][2];
    Column before_vels;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, before_vels, cubes));

    // Every other body, in prim-path order, has gravity disabled; the rest are the controls.
    std::map<std::string, bool> disabled;
    bool next = true;
    for (const std::pair<const std::string, std::vector<double>>& body : before_vels)
    {
        disabled[body.first] = next;
        next = !next;
    }
    ASSERT_GT(writeColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY,
                          [&](const std::string& path, std::vector<double>& row) { row[0] = disabled[path] ? 1 : 0; },
                          cubes),
              0);

    ASSERT_TRUE(stepN(m_handle, 1));
    Column vels;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, vels, cubes));
    Column disabled_poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, disabled_poses, cubes));
    for (const std::pair<const std::string, bool>& body : disabled)
    {
        const double before_vz = before_vels[body.first][2];
        const double after_vz = vels[body.first][2];
        if (body.second)
            EXPECT_NEAR(after_vz, before_vz, 0.02) << "Gravity-disabled body " << body.first << " did not coast";
        else
            EXPECT_LT(after_vz, before_vz - 0.1) << "Gravity-enabled control body " << body.first
                                                 << " did not accelerate";
    }

    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, { 0 }, cubes), 0);
    ASSERT_TRUE(stepN(m_handle, 10));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, vels, cubes));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    EXPECT_LT(vels[first][2], -0.5) << "Body should fall after re-enabling gravity";
    EXPECT_LT(poses[first][2], disabled_poses[first][2] - 0.05) << "Body should fall after re-enabling gravity";
    EXPECT_LT(z_falling, z_initial) << "Sanity: body fell before gravity was disabled";
}

TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableGravitySleepingBodiesWakeOnReenable)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

    auto active_count = [&]() -> uint64_t {
        ovphysx_query_handle_t query = 0;
        ovstage_query_result_t result{};
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_SCOPE_ACTIVE, &query, &result)).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status, OVPHYSX_API_SUCCESS);
        return result.total_prim_count;
    };

    const char* cubes = "/World/Cube*";
    const int count = writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, { 0 }, cubes);
    ASSERT_GT(count, 0);
    EXPECT_EQ(active_count(), 0u);

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, { 1 }, cubes), count);
    ASSERT_TRUE(stepN(m_handle, 1));
    EXPECT_EQ(active_count(), 0u) << "disabling gravity must not wake a sleeping body";

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, { 0 }, cubes), count);
    ASSERT_TRUE(stepN(m_handle, 1));
    EXPECT_EQ(active_count(), static_cast<uint64_t>(count))
        << "re-enabling gravity must wake sleeping bodies without a wake write";
}

TEST_F(SessionPhysicsCpuTest, CpuArticulationDisableGravityRoundtrip) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"))
        << "Failed to load USD";

    // Articulations become readable once the first simulate has inserted them (the session's
    // step-first precondition); warmup is that first simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    // Alternate 1/0 in prim-path order to exercise both set and cleared flags.
    std::map<std::string, double> written;
    ASSERT_GT(writeColumn(
                  m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values[0] = written.size() % 2 == 0 ? 1.0 : 0.0;
                      written[path] = values[0];
                  },
                  nullptr),
              1);

    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, readback, nullptr, 0, &kDtypeUInt8));
    ASSERT_EQ(readback.size(), written.size());
    for (const std::pair<const std::string, double>& flag : written)
        EXPECT_EQ(readback[flag.first][0], flag.second) << flag.first << " link disable-gravity round-trip";
}

// Mixed link-count sentinel: [N, L] articulation disable-gravity reads must
// zero-pad columns beyond each articulation's numLinks (L = maxLinks). Prefill
// the dst with 0xff so an uncleared pad is distinguishable from a real 0 flag.


TEST_F(SessionPhysicsCpuTest, CpuArticulationDriveTypeRead) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations.usda"))
        << "Failed to load USD";

    // Articulations become readable once the first simulate has inserted them (the session's
    // step-first precondition); warmup is that first simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    Column drive_types;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DRIVE_TYPE, drive_types));
    ASSERT_FALSE(drive_types.empty());

    // DofDriveType: 0 = none, 1 = force, 2 = acceleration. The fixture authors
    // PhysicsDriveAPI on its angular DOFs, so at least one DOF must report a
    // driven type. The exact enum mapping is pinned separately by
    // CpuArticulationDriveTypeExactMapping, which authors each type explicitly.
    int drivenCount = 0;
    for (const std::pair<const std::string, std::vector<double>>& joint : drive_types)
        for (double value : joint.second)
        {
            EXPECT_LE(value, 2.0) << "drive type of " << joint.first << " is outside {0,1,2}";
            if (value != 0.0)
                ++drivenCount;
        }
    EXPECT_GT(drivenCount, 0) << "fixture authors drives, so some DOF should report a driven type";
}

// Pins the exact DofDriveType mapping. The fixture authors one DOF per drive
// type, so an off-by-one or swapped force/acceleration mapping fails here rather
// than passing an in-range {0,1,2} check.
TEST_F(SessionPhysicsCpuTest, CpuArticulationDriveTypeExactMapping) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/drive_type_articulation.usda"))
        << "Failed to load USD";

    // Articulations become readable once the first simulate has inserted them (the session's
    // step-first precondition); warmup is that first simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    Column drive_types;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_DRIVE_TYPE, drive_types,
                           "/World/articulation_drives/joint*"));
    ASSERT_EQ(drive_types.size(), 3u) << "joint1 + joint2 + joint3 = 3 revolute DOFs";
    for (const std::pair<const std::string, std::vector<double>>& joint : drive_types)
        ASSERT_EQ(joint.second.size(), 1u) << joint.first << " is a single-axis revolute joint";

    EXPECT_EQ(drive_types["/World/articulation_drives/joint1"][0], 1.0) << "joint1 authors type=force -> eForce";
    EXPECT_EQ(drive_types["/World/articulation_drives/joint2"][0], 2.0)
        << "joint2 authors type=acceleration -> eAcceleration";
    EXPECT_EQ(drive_types["/World/articulation_drives/joint3"][0], 0.0) << "joint3 authors no drive -> eNone";
}

// Mixed DOF-count sentinel: [N, D] drive-type reads must zero-pad columns beyond
// each articulation's numDofs (D = maxDofs). The two articulations author
// different leading drive types, so row order is observable and a row swap fails
// here rather than satisfying the per-row checks. Prefill with 0xff so an
// uncleared pad is distinguishable from a real eNone(0).


TEST_F(SessionPhysicsCpuTest, CpuArticulationDisableGravitySuppressesFall) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/mixed_base_articulations.usda"))
        << "Failed to load USD";

    const char* articulation = "/World/articulation2";
    const char* links = "/World/articulation2/*";
    Column root_pose, root_vel;

    // Disable gravity before the first full step. Stepping under gravity first would
    // leave the root coasting downward, so a stationary-height assertion could
    // be satisfied by that momentum rather than by gravity suppression. The
    // articulation is readable only after the first simulate, so a warmup (a 1 ns
    // step, too short to build momentum) precedes the write.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, { 1 }, links), 0);

    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, root_pose, articulation));
    ASSERT_EQ(root_pose.size(), 1u);
    const double z0 = root_pose.begin()->second[2];

    ASSERT_TRUE(stepN(m_handle, 20));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, root_pose, articulation));
    ASSERT_TRUE(
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY, root_vel, articulation));
    const double z_disabled = root_pose.begin()->second[2];
    EXPECT_NEAR(root_vel.begin()->second[2], 0.0, 1e-3)
        << "Gravity-disabled root should not accumulate downward velocity";
    EXPECT_NEAR(z_disabled, z0, 0.01) << "Floating-base root Z should stay stable with gravity disabled";

    // Re-enable and compare against the height reached while disabled, so the
    // fall cannot be credited to drift accumulated before gravity was restored.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, { 0 }, links), 0);
    ASSERT_TRUE(stepN(m_handle, 20));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, root_pose, articulation));
    ASSERT_TRUE(
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY, root_vel, articulation));
    EXPECT_LT(root_vel.begin()->second[2], -0.5) << "Root should gain downward velocity after re-enabling gravity";
    EXPECT_LT(root_pose.begin()->second[2], z_disabled - 0.05)
        << "Floating-base root should fall after re-enabling gravity";
}

// Writing an articulation's root pose must propagate to the links across a step (OMPE-94459).
// applyCache(eROOT_TRANSFORM) is equivalent to setRootGlobalPose followed by
// updateKinematic(POSITION), so every link shifts with the root.
TEST_F(SessionPhysicsCpuTest, CpuArticulationRootTransformPropagates) {
    using namespace session_test;
    // Ant.usda is a multi-branch articulation, matching the umbrella
    // TestArticulationRootTransforms scenario.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/Ant.usda"))
        << "Failed to load USD";

    // Settling step. PhysX expects a settled articulation before
    // applyCache(eROOT_TRANSFORM) propagates correctly.
    ASSERT_TRUE(stepN(m_handle, 1));

    // The shift is expressed relative to the current root pose.
    Column root_initial, link_initial;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, root_initial));
    ASSERT_EQ(root_initial.size(), 1u);
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION, link_initial));

    // Root pose shifted by kShiftX in X with identity rotation.
    constexpr double kShiftX = 0.5; // Matches the umbrella's Z linspace(0, 1).
    ASSERT_EQ(writeColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION,
                          [&](const std::string& path, std::vector<double>& values)
                          {
                              values = root_initial[path];
                              values[0] += kShiftX;
                          }),
              1);
    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_ORIENTATION, { 0, 0, 0, 1 }), 1);

    // Read the root back before stepping to confirm the write took effect.
    Column root_after_write;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, root_after_write));
    const double initial_x = root_initial.begin()->second[0];
    const double read_x = root_after_write.begin()->second[0];
    std::cerr << "[root-prop trace] initial=" << initial_x << "  set=" << initial_x + kShiftX
              << "  read_after_write=" << read_x << "  (delta to set: " << (read_x - initial_x - kShiftX) << ")\n";

    ASSERT_TRUE(stepN(m_handle, 1));

    // Every link's X must shift by kShiftX.
    Column link_post;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION, link_post));
    int checked = 0;
    for (const std::pair<const std::string, std::vector<double>>& link : link_initial)
    {
        const double dx = link_post[link.first][0] - link.second[0];
        EXPECT_NEAR(dx, kShiftX, 0.001) // Matches the umbrella's rtol=1e-3 and atol=1e-4.
            << link.first << ": expected x-shift " << kShiftX << ", got " << dx;
        ++checked;
    }
    EXPECT_GT(checked, 0) << "should have checked at least one link";
}

// Multi-env prismatic DOF velocity write/step/read on a cart-rail grid, mirroring the
// umbrella LinearDofVelocities scenario (OMPE-94459). With do_settle false the write
// happens before the first step, as in the umbrella. With do_settle true one step runs
// before the write, as in the passing single-articulation reference.
static void RunMultiCartRailPrismaticDofVelocity(ovphysx_instance_handle_t m_handle,
                                                 bool do_settle,
                                                 const char* usd_asset = "tests/data/MultiCartRail.usda") {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, usd_asset))
        << "Failed to load " << usd_asset;

    // The carts are readable only after the first simulate: warmup (a 1 ns step) keeps the
    // no-settle write ahead of any real step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    if (do_settle) {
        ASSERT_TRUE(stepN(m_handle, 1));
    }

    // One prismatic cart joint per env: the stage holds nothing else with a DOF.
    Column joints;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY, joints));
    const size_t N = joints.size();
    ASSERT_GE(N, 1u) << "expected at least one cart joint";
    for (const std::pair<const std::string, std::vector<double>>& joint : joints)
        ASSERT_EQ(joint.second.size(), 1u) << "CartRailNoPole has 1 DOF (prismatic cart joint)";

    // Values vary per env over -2.0..2.0 like the umbrella scenario. Uniform
    // values would mask per-articulation index-mapping bugs.
    std::map<std::string, double> set_vel;
    for (const std::pair<const std::string, std::vector<double>>& joint : joints)
        set_vel[joint.first] = N > 1 ? -2.0 + 4.0 * double(set_vel.size()) / double(N - 1) : -2.0;
    ASSERT_EQ(writeColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY,
                          [&](const std::string& path, std::vector<double>& values) { values[0] = set_vel[path]; }),
              static_cast<int>(N));

    ASSERT_TRUE(stepN(m_handle, 1));

    Column read_vel;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY, read_vel));

    double max_err = 0.0;
    int over_tol_count = 0;
    for (const std::pair<const std::string, double>& joint : set_vel) {
        const double err = std::abs(read_vel[joint.first][0] - joint.second);
        if (err > max_err) max_err = err;
        if (err > 1e-3) {
            std::cerr << "[multi-cart-rail vel settle=" << (do_settle ? 1 : 0)
                      << "]   " << joint.first << " set=" << joint.second
                      << " read=" << read_vel[joint.first][0] << " err=" << err << "\n";
            ++over_tol_count;
        }
    }
    std::cerr << "[multi-cart-rail vel settle=" << (do_settle ? 1 : 0)
              << "] max_err=" << max_err
              << " over_tol_count=" << over_tol_count << " / " << N << "\n";
}

TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_NoSettle) {
    RunMultiCartRailPrismaticDofVelocity(m_handle, /*do_settle=*/false);
}

TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_WithSettle) {
    RunMultiCartRailPrismaticDofVelocity(m_handle, /*do_settle=*/true);
}

// Variant with the PhysxSceneAPI attributes the umbrella applies on /physicsScene
// (OMPE-94459): enableGpuDynamics=0, broadPhaseType="MBP",
// enableSceneQuerySupport=0, timeStepsPerSecond=60. Isolates whether the scene
// settings trigger the umbrella's prismatic velocity decay.
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaSceneAPI) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false, "tests/data/MultiCartRailUmbrella.usda");
}

// 42-env variant matching the umbrella's scenario count, with gravity direction
// (0,0,0) instead of (0,0,-1) at magnitude 0. Rules out count dependence and the
// zero-vector gravity direction.
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_42EnvUmbrella) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false, "tests/data/MultiCartRail42Umbrella.usda");
}

// USD class-prim inheritance variant. `/envTemplate` is a class prim whose
// `railcart` child references the asset, and each `/envs/env_N` inherits it.
// Isolates composition by inheritance from the direct per-env references used
// by the `_NoSettle` and `_UmbrellaSceneAPI` variants.
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_InheritUmbrella) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/MultiCartRailInheritUmbrella.usda");
}

// Loads the USDA exported from a failing umbrella LinearDofVelocities run, which is
// the exact stage the ovstage population path sees there. Separates a cause in the
// USDA content from one above the C ABI (adapter indices, step path, or process-wide
// PhysX state from earlier tests).
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExport) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export.usda");
}

// Same as _UmbrellaExport with the groundPlane stripped, to check whether the
// static collider at z=0 affects the cart's prismatic DOF integration.
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportNoGround) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_noground.usda");
}

// Same as _UmbrellaExport with 50m env spacing on both axes. Rails extend +/-3 in
// Y, so this removes the inter-env rail overlap present at the umbrella's
// row_spacing=2 / col_spacing=6.5 layout and tests whether that overlap is the trigger.
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportWideSpacing) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_widespacing.usda");
}

// Same as _UmbrellaExport with row_spacing widened to 8m, the smallest
// non-overlapping spacing for rails extending +/-3 in Y. col_spacing stays at the
// umbrella's 6.5m, so only the row-direction overlap is removed.
TEST_F(SessionPhysicsCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportNoOverlap) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_nooverlap.usda");
}

// Linear drive PD settling, mirroring the umbrella LinearDofPositionTargets scenario
// (OMPE-94459). CartRailDriveLinear.usda uses the umbrella's PD parameters (k=2000,
// d=250, maxForce=4000). With a 10 kg cart (density 1000 kg/m^3, scale 0.2 x 0.25 x 0.2)
// the natural frequency is about 14 rad/s and the damping ratio about 0.88, so the
// cart settles in roughly 0.32 s. 60 steps at 60 Hz leave ample headroom.
TEST_F(SessionPhysicsCpuTest, CpuLinearDrivePD_SettlesIn60Steps) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/CartRailDriveLinear.usda"))
        << "Failed to load CartRailDriveLinear.usda";

    // Settle step, matching the umbrella flow where the target is set before the
    // first step.
    ASSERT_TRUE(stepN(m_handle, 1));

    const double target = 1.0;
    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION_TARGET, { target }),
              1);

    double final_pos = 0.0;
    Column position;
    for (int s = 0; s < 60; ++s) {
        ASSERT_TRUE(stepN(m_handle, 1));
        ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, position));
        ASSERT_EQ(position.size(), 1u);
        final_pos = position.begin()->second[0];
        if (s < 5 || s == 19 || s == 29 || s == 59) {
            std::cerr << "[linear-drive PD] step " << (s + 1) << " pos=" << final_pos << "\n";
        }
    }

    const double err = std::abs(final_pos - target);
    std::cerr << "[linear-drive PD] final_pos=" << final_pos << " target=" << target
              << " err=" << err << "\n";
    EXPECT_LT(err, 1e-3) << "linear-drive PD did not settle to target within 60 steps (err=" << err << ")";
}

// For a single CartRailNoPole prismatic articulation, a joint velocity
// write, step and read preserves the joint velocity exactly (OMPE-94459). The drift
// seen by the umbrella LinearDofVelocities test only appears in its 42-env cloned grid
// and is engine-side, prismatic-specific behavior rather than an ovphysx defect.
TEST_F(SessionPhysicsCpuTest, CpuPrismaticDofVelocity_NoDriftOnSingleArticulation) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/CartRailNoPole.usda"))
        << "Failed to load CartRailNoPole.usda";

    // Articulations become readable once the first simulate has inserted them (the session's
    // step-first precondition); warmup is that first simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const float set_vel = 2.0f;
    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY, { set_vel }), 1);

    ASSERT_TRUE(stepN(m_handle, 1));

    Column read_vel;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY, read_vel));
    ASSERT_EQ(read_vel.size(), 1u);
    ASSERT_EQ(read_vel.begin()->second.size(), 1u);
    // The ovphysx path preserves the velocity exactly on a single articulation.
    EXPECT_NEAR(read_vel.begin()->second[0], set_vel, 1e-6);
}


// Writing 1 to disableSimulation must freeze the body mid-simulation (OMPE-94459).
// Catches the case where the flag round-trips (CpuRigidBodyDisableSimulationRoundtrip)
// but the engine does not toggle PxActorFlag::eDISABLE_SIMULATION.
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableSimulationStopsSimulation) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    const char* cubes = "/World/Cube*";
    ASSERT_TRUE(stepN(m_handle, 1));
    Column pose_before_disable;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_before_disable, cubes));
    ASSERT_FALSE(pose_before_disable.empty());

    // One more step without disabling. The pose must change under gravity.
    ASSERT_TRUE(stepN(m_handle, 1));
    Column poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    bool moved_before_disable = false;
    for (const std::pair<const std::string, std::vector<double>>& body : poses)
        moved_before_disable |= std::abs(body.second[2] - pose_before_disable[body.first][2]) > 1e-5;
    ASSERT_TRUE(moved_before_disable) << "Sanity check: bodies should fall under gravity before disable.";

    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 }, cubes), 0);
    Column pose_after_disable;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_after_disable, cubes));

    // Disabled bodies must not move over further steps.
    ASSERT_TRUE(stepN(m_handle, 5));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    for (const std::pair<const std::string, std::vector<double>>& body : pose_after_disable)
        for (int axis = 0; axis < 3; ++axis)
            EXPECT_NEAR(poses[body.first][axis], body.second[axis], 1e-4)
                << body.first << " axis " << axis << " drifted while disabled";
}

// disableSimulation=1 written before any explicit step must be honoured by the next
// simulate (OMPE-94459, umbrella RigidBodyEnableDisablePhysics). Differs from
// CpuRigidBodyDisableSimulationStopsSimulation only in the missing step before the write.
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableSimulationInOnStart) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    // Starting pose after the initial parse and before any explicit step.
    const char* cubes = "/World/Cube*";
    Column pose_initial;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_initial, cubes));
    ASSERT_FALSE(pose_initial.empty());

    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 }, cubes), 0);
    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, readback, cubes));
    for (const std::pair<const std::string, std::vector<double>>& body : readback)
        EXPECT_EQ(body.second[0], 1.0) << body.first << " disable flag did not persist";

    ASSERT_TRUE(stepN(m_handle, 3));
    Column poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    int moved = 0;
    for (const std::pair<const std::string, std::vector<double>>& body : pose_initial)
    {
        const double dz = std::abs(poses[body.first][2] - body.second[2]);
        if (dz > 1e-4)
        {
            std::cerr << "[disable-in-on-start] " << body.first << " z drifted " << dz << "\n";
            ++moved;
        }
    }
    // None of the bodies may drift. This assert is what catches the regression
    // where a disable written before the first step is not re-read by PhysX.
    EXPECT_EQ(moved, 0) << "disabled bodies drifted after an on_start disable write";
}

// Same write-before-step flow as CpuRigidBodyDisableSimulationInOnStart, but with a
// selected write that disables only every other body (OMPE-94459). Mirrors the
// umbrella RigidBodyEnableDisablePhysics scenario, which writes a uniform 1 with
// indices [0, 2, 4, ...].
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableSimulationIndexedInOnStart) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    const char* cubes = "/World/Cube*";
    Column bodies;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, bodies, cubes));
    ASSERT_GT(bodies.size(), 1u);
    std::vector<std::string> disabled_paths, enabled_paths;
    alternateHalves(bodies, disabled_paths, enabled_paths);

    // Disable every other body before the first step, through a write selected by those bodies.
    const std::string disabled_pattern = subsetPattern("/World", disabled_paths);
    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 },
                           disabled_pattern.c_str()),
              static_cast<int>(disabled_paths.size()));
    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, readback, cubes));
    for (const std::string& path : disabled_paths)
        EXPECT_EQ(readback[path][0], 1.0) << "disabled body " << path << " not flagged";
    for (const std::string& path : enabled_paths)
        EXPECT_EQ(readback[path][0], 0.0) << "enabled body " << path << " incorrectly flagged";

    // Two steps match the umbrella's stepno=1, which is reached after two simulates.
    ASSERT_TRUE(stepN(m_handle, 2));
    Column velocities;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, velocities, cubes));
    int disabled_falling = 0, enabled_static = 0;
    for (const std::string& path : disabled_paths)
        disabled_falling += std::abs(velocities[path][2]) > 1e-3 ? 1 : 0;
    for (const std::string& path : enabled_paths)
        enabled_static += std::abs(velocities[path][2]) < 1e-3 ? 1 : 0;
    // The disabled half must read zero z-velocity while the half left enabled must still be falling.
    EXPECT_EQ(disabled_falling, 0) << "disabled bodies still falling";
    EXPECT_EQ(enabled_static, 0) << "enabled bodies were not falling";
}

// CPU reference for the umbrella's enable/disable flip sequence (OMPE-94459), which
// the GPU path is measured against: disable a moving body, then flip and wake in one
// window and step. A disabled body reads zero velocity and a re-enabled one resumes.
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyDisableFlipWakeSequence) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"));

    const char* cubes = "/World/Cube*";
    Column bodies;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, bodies, cubes));
    ASSERT_GT(bodies.size(), 1u);
    std::vector<std::string> idx_A, idx_B; // A starts disabled, B starts enabled.
    alternateHalves(bodies, idx_A, idx_B);
    const std::string pattern_A = subsetPattern("/World", idx_A);
    const std::string pattern_B = subsetPattern("/World", idx_B);

    // Disable A before the first step, then step twice. The umbrella's stepno=1 sees -2*dt*g.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 },
                           pattern_A.c_str()),
              0);
    ASSERT_TRUE(stepN(m_handle, 2));

    // Flip in one window: disable B, enable A, wake all.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 },
                           pattern_B.c_str()),
              0);
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 0 },
                           pattern_A.c_str()),
              0);
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, { 1 }, cubes), 0);
    ASSERT_TRUE(stepN(m_handle, 1));
    Column velocities;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, velocities, cubes));

    // A body disabled mid-motion reads 0. A re-enabled and woken body resumes.
    for (const std::string& path : idx_B)
        EXPECT_NEAR(velocities[path][2], 0.0, 1e-3) << "cpu: now-disabled body " << path << " should read 0";
    for (const std::string& path : idx_A)
        EXPECT_LT(velocities[path][2], -0.01) << "cpu: re-enabled+woken body " << path << " should be falling";
}

// A wake write after a disable/re-enable cycle (OMPE-94459). Re-enabled bodies come
// back asleep, so they only move again once woken. Bodies still flagged disabled are
// skipped by the wake write.
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyViewWakeUpAfterReEnable) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    const char* cubes = "/World/Cube*";
    Column pose_after_disable, poses;

    // Disabled bodies must not move across steps.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 }, cubes), 0);
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_after_disable, cubes));
    ASSERT_FALSE(pose_after_disable.empty());
    ASSERT_TRUE(stepN(m_handle, 3));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    for (const std::pair<const std::string, std::vector<double>>& body : pose_after_disable)
        EXPECT_NEAR(poses[body.first][2], body.second[2], 1e-4) << body.first << " z drifted while disabled";

    // Waking a still-disabled body is a per-body no-op. The write itself succeeds.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, { 1 }, cubes), 0);
    ASSERT_TRUE(stepN(m_handle, 2));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    for (const std::pair<const std::string, std::vector<double>>& body : pose_after_disable)
        EXPECT_NEAR(poses[body.first][2], body.second[2], 1e-4)
            << body.first << " z drifted after no-op wake on disabled body";

    // PhysX puts the re-introduced actor in sleep state. Without a wake, stepping
    // leaves the bodies asleep with zero velocity.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 0 }, cubes), 0);
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, { 1 }, cubes), 0);
    Column pose_before_wake_steps;
    ASSERT_TRUE(
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_before_wake_steps, cubes));
    ASSERT_TRUE(stepN(m_handle, 3));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, cubes));
    size_t moved = 0;
    for (const std::pair<const std::string, std::vector<double>>& body : pose_before_wake_steps)
        if (body.second[2] - poses[body.first][2] > 1e-3) // Positive means the body fell under gravity.
            ++moved;
    EXPECT_EQ(moved, pose_before_wake_steps.size()) << "all bodies should fall under gravity after re-enable + wake";
}

// A wake write selected by a subset: only that subset should wake.
TEST_F(SessionPhysicsCpuTest, CpuRigidBodyViewWakeUpIndexed) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    const char* cubes = "/World/Cube*";
    Column bodies;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, bodies, cubes));
    ASSERT_GT(bodies.size(), 1u);

    // Disable all, then re-enable all. Bodies are now in sleep state.
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 1 }, cubes), 0);
    ASSERT_TRUE(stepN(m_handle, 1));
    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_SIMULATION, { 0 }, cubes), 0);

    // Wake only every other body, through a write selected by those bodies.
    std::vector<std::string> woken, sleeping;
    alternateHalves(bodies, woken, sleeping);
    const std::string woken_pattern = subsetPattern("/World", woken);
    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, { 1 }, woken_pattern.c_str()),
              static_cast<int>(woken.size()));

    Column pose_before, pose_after;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_before, cubes));
    ASSERT_TRUE(stepN(m_handle, 3));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose_after, cubes));

    int woke_moved = 0, slept_moved = 0;
    for (const std::string& path : woken)
        woke_moved += pose_before[path][2] - pose_after[path][2] > 1e-3 ? 1 : 0;
    for (const std::string& path : sleeping)
        slept_moved += pose_before[path][2] - pose_after[path][2] > 1e-3 ? 1 : 0;
    EXPECT_GT(woke_moved, 0) << "woken bodies should fall under gravity";
    EXPECT_EQ(slept_moved, 0) << "non-woken bodies should remain at rest";
}

// Adds a one-cube rigid body under `path` through the ovstage population at `ordinal`, and brings the
// simulation up to that ordinal.
static bool addRigidBodyThroughStage(ovphysx_instance_handle_t handle, ovstage_instance_t* stage, const char* path,
                                     uint64_t ordinal)
{
    constexpr const char* body = R"(#usda 1.0
(defaultPrim = "Body")
def Xform "Body" (prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"])
{
    float physics:mass = 1.0
    def Cube "Collider" (prepend apiSchemas = ["PhysicsCollisionAPI"])
    {
        double size = 0.5
    }
}
)";
    constexpr ovstage_timeout_ns_t waitTimeout = 10'000'000'000ULL;
    const auto waitPopulation = [&](ovstage_population_enqueue_result_t op)
    {
        if (op.status != OVSTAGE_OK)
            return false;
        ovstage_population_op_wait_result_t result{};
        return ovstage_population_wait_op(stage, op.op_index, waitTimeout, &result) == OVSTAGE_OK &&
               result.error_op_id_count == 0;
    };
    if (!waitPopulation(ovstage_population_add_usd_reference_from_string(
            stage, { body, std::strlen(body) }, { path, std::strlen(path) }, nullptr)) ||
        !waitPopulation(ovstage_population_apply_usd_changes(stage, ordinal)))
        return false;
    ovstage_write_floor_desc_t floorDesc{};
    floorDesc.ordinal = ordinal;
    floorDesc.scope = OVSTAGE_SCOPE_ALL;
    const ovstage_enqueue_result_t floor = ovstage_advance_write_floor(stage, &floorDesc);
    if (floor.status != OVSTAGE_OK)
        return false;
    ovstage_op_wait_result_t floorResult{};
    const ovstage_api_status_t waitStatus = ovstage_wait_op(stage, floor.op_index, waitTimeout, &floorResult);
    if (waitStatus != OVSTAGE_OK && waitStatus != OVSTAGE_ERROR_OP_FAILED)
        return false;
    const ovstage_api_status_t releaseStatus = ovstage_release_op(stage, floor.op_index);
    if (waitStatus != OVSTAGE_OK || floorResult.error_op_id_count != 0 || releaseStatus != OVSTAGE_OK)
        return false;
    ovstage_ordinal_range_t range{};
    range.has_start_ordinal = true;
    range.start_ordinal = ordinal;
    range.end_ordinal = ordinal;
    return test_utils::complete_task(handle, ovphysx_update_from_ovstage(handle, range)).status == OVPHYSX_API_SUCCESS;
}

// Session counterpart of NewBindingIncludesAddedBody's non-binding check: a query opened after a body is
// added through the stage covers it, alongside the bodies already there.
TEST_F(SessionPhysicsCpuTest, SessionQueryIncludesAddedBody)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/empty_dynamic_boxes_cpu.usda"));
    ovstage_instance_t* stage = nullptr;
    {
        std::lock_guard<std::mutex> lock(test_utils::ovstage_test_attachments_mutex());
        stage = test_utils::ovstage_test_attachments().at(m_handle).back().stage;
    }

    const char* boxes = "/World/DynamicBoxes/*";
    ASSERT_TRUE(addRigidBodyThroughStage(m_handle, stage, "/World/DynamicBoxes/box_0", 2));
    Column poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, boxes));
    ASSERT_EQ(poses.size(), 1u);

    ASSERT_TRUE(addRigidBodyThroughStage(m_handle, stage, "/World/DynamicBoxes/box_1", 3));
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, poses, boxes));
    EXPECT_EQ(poses.size(), 2u);
    EXPECT_EQ(poses.count("/World/DynamicBoxes/box_0"), 1u);
    EXPECT_EQ(poses.count("/World/DynamicBoxes/box_1"), 1u);
    for (const std::pair<const std::string, std::vector<double>>& pose : poses)
        for (double value : pose.second)
            EXPECT_TRUE(std::isfinite(value)) << pose.first;
}





// ============================================================================
// FORCE / WRENCH EFFECT TESTS
// ============================================================================
// Written forces must produce observable physical effects, not just API success.
// boxes_falling_on_groundplane.usda has 11 cubes at Z=10.

TEST_F(SessionPhysicsCpuTest, ForceWriteEffect_RigidBodyDisplacement) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    // One step to initialize physics.
    ASSERT_TRUE(stepN(m_handle, 1));

    const char* cubes = "/World/Cube*";
    Column initial_poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, initial_poses, cubes));
    ASSERT_GE(initial_poses.size(), 2u) << "Need at least 2 rigid bodies";

    // The first body in path order is pushed; the last one is the reference.
    const std::string pushed = initial_poses.begin()->first;
    const std::string control = initial_poses.rbegin()->first;

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(writeColumn(
                      m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_FORCE,
                      [&](const std::string& path, std::vector<double>& values)
                      {
                          ASSERT_EQ(values.size(), 3u);
                          std::fill(values.begin(), values.end(), 0.0);
                          if (path == pushed)
                              values[0] = 50000.0;
                      },
                      cubes),
                  static_cast<int>(initial_poses.size()))
            << "Force write failed on step " << i;
        ASSERT_TRUE(stepN(m_handle, 1));
    }

    Column final_poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, final_poses, cubes));

    const double dx_pushed = std::abs(final_poses[pushed][0] - initial_poses[pushed][0]);
    const double dx_control = std::abs(final_poses[control][0] - initial_poses[control][0]);

    // 50000 N over 5 steps at 1/60 s moves even a 1000 kg body measurably.
    std::cout << "  Force test: " << pushed << " X " << initial_poses[pushed][0] << " -> " << final_poses[pushed][0]
              << " (dx=" << dx_pushed << ")" << std::endl;
    std::cout << "  Force test: control " << control << " X " << initial_poses[control][0] << " -> "
              << final_poses[control][0] << " (dx=" << dx_control << ")" << std::endl;

    EXPECT_GT(dx_pushed, 0.1)
        << "The pushed body should show significant X displacement from 50000N force";
    EXPECT_LT(dx_control, 0.01)
        << "Control body should have negligible X displacement (no X force applied)";
}

TEST_F(SessionPhysicsCpuTest, WrenchWriteEffect_RigidBodyDisplacement) {
    using namespace session_test;
    // Same as the force test, through the 9-wide wrench [fx,fy,fz, tx,ty,tz, px,py,pz]. Torque and
    // position stay zero so the force acts at the center of mass.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane.usda"))
        << "Failed to load USD";

    // One step to initialize physics.
    ASSERT_TRUE(stepN(m_handle, 1));

    const char* cubes = "/World/Cube*";
    Column initial_poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, initial_poses, cubes));
    ASSERT_GE(initial_poses.size(), 2u) << "Need at least 2 rigid bodies";

    // The first body in path order is pushed; the last one is the reference.
    const std::string pushed = initial_poses.begin()->first;
    const std::string control = initial_poses.rbegin()->first;

    for (int i = 0; i < 5; ++i) {
        ASSERT_EQ(writeColumn(
                      m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WRENCH,
                      [&](const std::string& path, std::vector<double>& values)
                      {
                          ASSERT_EQ(values.size(), 9u);
                          std::fill(values.begin(), values.end(), 0.0);
                          if (path == pushed)
                              values[0] = 50000.0;
                      },
                      cubes),
                  static_cast<int>(initial_poses.size()))
            << "Wrench write failed on step " << i;
        ASSERT_TRUE(stepN(m_handle, 1));
    }

    Column final_poses;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, final_poses, cubes));

    const double dx_pushed = std::abs(final_poses[pushed][0] - initial_poses[pushed][0]);
    const double dx_control = std::abs(final_poses[control][0] - initial_poses[control][0]);

    // 50000 N over 5 steps at 1/60 s moves even a 1000 kg body measurably.
    std::cout << "  Wrench test: " << pushed << " X " << initial_poses[pushed][0] << " -> " << final_poses[pushed][0]
              << " (dx=" << dx_pushed << ")" << std::endl;
    std::cout << "  Wrench test: control " << control << " X " << initial_poses[control][0] << " -> "
              << final_poses[control][0] << " (dx=" << dx_control << ")" << std::endl;

    EXPECT_GT(dx_pushed, 0.1)
        << "The pushed body should show significant X displacement from 50000N wrench";
    EXPECT_LT(dx_control, 0.01)
        << "Control body should have negligible X displacement (no X force applied)";
}

// ============================================================================
// CPU-ONLY MODE SAFETY TESTS
// ovphysx must run without crashes on systems without a GPU. These tests load
// stages that select CPU mode (no physxScene:enableGPUDynamics) and exercise the
// full lifecycle so no code path dereferences a null CUDA handle or calls a cu*
// function.
// ============================================================================

TEST_F(SessionPhysicsCpuTest, CpuOnlyLifecycle_NoCrash) {
    using namespace session_test;
    // Core lifecycle in CPU mode: load USD, step, read, write, step. Not crashing
    // is the point, value correctness is secondary.

    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample.usda"))
        << "Failed to load USD in CPU mode";

    ASSERT_TRUE(stepN(m_handle, 3)) << "CPU step failed";

    // warmup() runs in CPU mode too: a 1ns step for lazy init, and it disables Fabric sync.
    ovphysx_result_t result = ovphysx_warmup(m_handle);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS) << "warmup should succeed in CPU mode";

    Column positions;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, positions))
        << "CPU read failed";
    EXPECT_GT(positions.size(), 0u);

    ASSERT_GT(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, { 0.05 }), 0)
        << "CPU write failed";

    // Stepping after the write must not touch a CUDA handle the CPU scene never created.
    ASSERT_TRUE(stepN(m_handle, 1));
}



// ============================================================================
// DEFAULT GPU-DYNAMICS TESTS (NO DIRECTGPU)
// ============================================================================

class SessionGpuDynamicsTest : public ::testing::Test
{
    static ovphysx_instance_handle_t s_handle;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;

    static void SetUpTestSuite()
    {
#if !OVPHYSX_ENABLE_GPU_TESTS
        s_skipReason = "GPU tests disabled at compile time";
        return;
#endif
        const ovphysx_result_t createResult = ovphysx_create_instance(&s_handle);
        if (createResult.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            s_skipReason = error.ptr ? std::string(error.ptr, error.length) : "Failed to create default GPU instance";
            s_handle = 0;
        }
    }

    static void TearDownTestSuite()
    {
        if (s_handle == 0)
            return;
        const ovphysx_enqueue_result_t resetResult = ovphysx_detach_ovstage(s_handle);
        if (resetResult.status == OVPHYSX_API_SUCCESS && resetResult.op_index != 0)
            EXPECT_TRUE(wait_task_success(s_handle, resetResult));
        else
            EXPECT_EQ(resetResult.status, OVPHYSX_API_SUCCESS);
        test_utils::destroy_ovstage_test_attachments(s_handle);
        ovphysx_destroy_instance(s_handle);
        s_handle = 0;
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
        if (m_handle == 0)
            return;
        const ovphysx_enqueue_result_t resetResult = ovphysx_detach_ovstage(m_handle);
        if (resetResult.status == OVPHYSX_API_SUCCESS && resetResult.op_index != 0)
            EXPECT_TRUE(wait_task_success(m_handle, resetResult));
        else
            EXPECT_EQ(resetResult.status, OVPHYSX_API_SUCCESS);
        test_utils::destroy_ovstage_test_attachments(m_handle);
        m_handle = 0;
    }

    void step(int count)
    {
        for (int i = 0; i < count; ++i)
        {
            const ovphysx_enqueue_result_t result = ovphysx_step(m_handle, 1.0f / 60.0f);
            ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
            ASSERT_TRUE(wait_task_success(m_handle, result));
        }
    }

    void warmupAndStep()
    {
        ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
        step(1);
    }

    uint64_t activeCount(const ovphysx_query_target_t objectType)
    {
        ovphysx_query_handle_t query = 0;
        ovstage_query_result_t result{};
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, objectType, OVPHYSX_SCOPE_ACTIVE, &query, &result)).status,
                  OVPHYSX_API_SUCCESS);
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, query)).status, OVPHYSX_API_SUCCESS);
        return result.total_prim_count;
    }

    void writeUniformU8(const ovphysx_query_target_t objectType,
                        const char* attributeName,
                        const uint8_t value)
    {
        writeUniform(objectType, attributeName, kDLUInt, 8, &value, sizeof(value));
    }

    void writeDisableGravity(const ovphysx_query_target_t objectType, const uint8_t value)
    {
        writeUniformU8(objectType, OVPHYSX_ATTR_DISABLE_GRAVITY, value);
    }

    void writeUniform(const ovphysx_query_target_t objectType,
                      const char* attributeName,
                      const uint8_t dtypeCode,
                      const uint8_t dtypeBits,
                      const void* fillValue,
                      const size_t fillBytes)
    {
        struct WriteSessionGuard
        {
            explicit WriteSessionGuard(const ovphysx_instance_handle_t instance) : handle(instance)
            {
            }

            ~WriteSessionGuard()
            {
                if (write)
                    test_utils::complete_task(handle, ovphysx_release_write(handle, write));
                if (query)
                    test_utils::complete_task(handle, ovphysx_release_query(handle, query));
            }

            ovphysx_instance_handle_t handle;
            ovphysx_query_handle_t query = 0;
            ovphysx_write_handle_t write = 0;
        } guard(m_handle);

        ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_query(m_handle, objectType, OVPHYSX_SCOPE_ALL, &guard.query, nullptr)).status,
                  OVPHYSX_API_SUCCESS);

        const ovx_string_or_token_t attribute = {
            0,
            { attributeName, std::strlen(attributeName) }
        };
        ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_write(m_handle, guard.query, &attribute, 1, &guard.write, nullptr)).status, OVPHYSX_API_SUCCESS);

        ovphysx_fetched_write_group_t fetched{};
        ovphysx_result_t fetch{};
        while ((fetch = test_utils::complete_task(m_handle, ovphysx_fetch_write_next(m_handle, guard.write, &fetched))).status == OVPHYSX_API_SUCCESS && fetched.group)
        {
            const ovstage_map_group_t* group = fetched.group;
            ASSERT_NE(group, nullptr);
            ASSERT_EQ(group->data.tensor_count, 1u);
            const DLTensor& tensor = group->data.tensors[0];
            ASSERT_EQ(tensor.device.device_type, kDLCPU);
            ASSERT_EQ(tensor.dtype.code, dtypeCode);
            ASSERT_EQ(tensor.dtype.bits, dtypeBits);
            size_t elementCount = tensor.dtype.lanes ? tensor.dtype.lanes : 1;
            for (int32_t dim = 0; dim < tensor.ndim; ++dim)
                elementCount *= static_cast<size_t>(tensor.shape[dim]);
            uint8_t* dst = static_cast<uint8_t*>(tensor.data);
            for (size_t i = 0; i < elementCount; ++i)
                std::memcpy(dst + i * fillBytes, fillValue, fillBytes);
            ASSERT_EQ(test_utils::complete_task(m_handle, ovphysx_commit_group(m_handle, guard.write, group, ovstage_cuda_sync_t{})).status,
                      OVPHYSX_API_SUCCESS);
        }
        EXPECT_EQ(fetch.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_write(m_handle, guard.write)).status, OVPHYSX_API_SUCCESS);
        guard.write = 0;
        EXPECT_EQ(test_utils::complete_task(m_handle, ovphysx_release_query(m_handle, guard.query)).status, OVPHYSX_API_SUCCESS);
        guard.query = 0;
    }
};

ovphysx_instance_handle_t SessionGpuDynamicsTest::s_handle = 0;
std::string SessionGpuDynamicsTest::s_skipReason;



TEST_F(SessionGpuDynamicsTest, SessionWriteRigidBodyDisableGravityAffectsSimulation)
{
    using session_test::Column;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));
    warmupAndStep();

    const uintptr_t ctx = session_test::cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    const char* cubes = "/World/Cube*";
    Column beforeVel, movingVel, disabledVel, held, released;
    ASSERT_TRUE(session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, beforeVel,
                                         cubes, ctx));
    ASSERT_GT(beforeVel.size(), 1u);

    step(1);
    ASSERT_TRUE(session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, movingVel,
                                         cubes, ctx));
    for (const std::pair<const std::string, std::vector<double>>& body : beforeVel)
        EXPECT_LT(movingVel[body.first][2], body.second[2] - 0.1)
            << "gravity-enabled control body " << body.first << " did not accelerate";

    ASSERT_NO_FATAL_FAILURE(writeDisableGravity(OVPHYSX_QUERY_TARGET_RIGID_BODY, 1));
    step(1);
    ASSERT_TRUE(session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY,
                                         disabledVel, cubes, ctx));
    for (const std::pair<const std::string, std::vector<double>>& body : movingVel)
        EXPECT_NEAR(disabledVel[body.first][2], body.second[2], 0.02)
            << "gravity-disabled body " << body.first << " did not coast";

    ASSERT_TRUE(
        session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, held, cubes, ctx));

    ASSERT_NO_FATAL_FAILURE(writeDisableGravity(OVPHYSX_QUERY_TARGET_RIGID_BODY, 0));
    step(20);
    ASSERT_TRUE(
        session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, released, cubes, ctx));
    for (const std::pair<const std::string, std::vector<double>>& body : held)
        EXPECT_LT(released[body.first][2], body.second[2] - 0.05) << body.first;
}

TEST_F(SessionGpuDynamicsTest, SleepingRigidBodiesWakeOnlyWhenGravityIsReenabled)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"));

    session_test::Column bodies;
    const uintptr_t ctx = session_test::cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    ASSERT_TRUE(
        session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, bodies, nullptr, ctx));
    const uint64_t count = bodies.size();
    ASSERT_GT(count, 0u);

    ASSERT_NO_FATAL_FAILURE(writeUniformU8(OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WAKE, 0));
    EXPECT_EQ(activeCount(OVPHYSX_QUERY_TARGET_RIGID_BODY), 0u);

    ASSERT_NO_FATAL_FAILURE(writeDisableGravity(OVPHYSX_QUERY_TARGET_RIGID_BODY, 1));
    step(1);
    EXPECT_EQ(activeCount(OVPHYSX_QUERY_TARGET_RIGID_BODY), 0u)
        << "disabling gravity must not wake a sleeping body";

    ASSERT_NO_FATAL_FAILURE(writeDisableGravity(OVPHYSX_QUERY_TARGET_RIGID_BODY, 0));
    step(1);
    EXPECT_EQ(activeCount(OVPHYSX_QUERY_TARGET_RIGID_BODY), count)
        << "re-enabling gravity must wake sleeping bodies for the next step";
}



TEST_F(SessionGpuDynamicsTest, SessionWriteArticulationLinkDisableGravityAffectsSimulation)
{
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/mixed_base_articulations_gpu.usda"));
    warmupAndStep();

    const uintptr_t ctx = session_test::cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    // The root linear velocity's z of the floating-base articulation.
    const auto rootVz = [&](double& vz)
    {
        session_test::Column vel;
        ASSERT_TRUE(session_test::readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION,
                                             OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY, vel, "/World/articulation2", ctx));
        ASSERT_EQ(vel.size(), 1u);
        vz = vel.begin()->second[2];
    };
    double beforeVz = 0.0, movingVz = 0.0, disabledVz = 0.0, releasedVz = 0.0;
    ASSERT_NO_FATAL_FAILURE(rootVz(beforeVz));

    step(1);
    ASSERT_NO_FATAL_FAILURE(rootVz(movingVz));
    EXPECT_LT(movingVz, beforeVz - 0.1) << "gravity-enabled articulation control did not accelerate";

    ASSERT_NO_FATAL_FAILURE(writeDisableGravity(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, 1));
    step(1);
    ASSERT_NO_FATAL_FAILURE(rootVz(disabledVz));
    EXPECT_NEAR(disabledVz, movingVz, 0.02) << "gravity-disabled articulation did not coast";

    ASSERT_NO_FATAL_FAILURE(writeDisableGravity(OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, 0));
    step(20);
    ASSERT_NO_FATAL_FAILURE(rootVz(releasedVz));
    EXPECT_LT(releasedVz, disabledVz - 0.5);
}

// ============================================================================
// DIRECTGPU TESTS
// ============================================================================

// Shared GPU instance for all SessionPhysicsGpuTest tests.
//
// Carbonite and the Python interpreter cannot be cleanly finalized and
// re-initialized in the same process (DLL init routines fail on reload).
// A single long-lived instance avoids the destroy/recreate cycle while
// still resetting simulation state (USD, bindings) between tests.
class SessionPhysicsGpuTest : public ::testing::Test {
    static ovphysx_instance_handle_t s_handle;
    static ovphysx::test_cuda::CudaOps s_cudaOps;
    static std::string s_skipReason;

protected:
    ovphysx_instance_handle_t m_handle = 0;

    static void SetUpTestSuite() {
#if !OVPHYSX_ENABLE_GPU_TESTS
        s_skipReason = "GPU tests disabled at compile time";
        return;
#endif
        // DirectGPU pipeline (GPU-resident state via PxDirectGPUAPI): opt into
        // /physics/suppressReadback explicitly, as ovphysx does not enable it by default.
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;
        ovphysx_config_entry_t direct_gpu_entries[] = {
            ovphysx_config_entry_carbonite(OVPHYSX_LITERAL("/physics/suppressReadback"), OVPHYSX_LITERAL("true")),
        };
        args.config_entries = direct_gpu_entries;
        args.config_entry_count = sizeof(direct_gpu_entries) / sizeof(direct_gpu_entries[0]);

        ovphysx_result_t create_result = ovphysx_create_instance_with_args(&args, &s_handle);
        if (create_result.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t last_err = ovphysx_get_last_error();
            s_skipReason = last_err.length > 0 ? std::string(last_err.ptr, last_err.length) : "Failed to create GPU instance";
            s_handle = 0;
            return;
        }
        // Session GPU reads copy device columns through this CUDA shim; gate on its availability.
        s_cudaOps.reset(ovphysx::test_cuda::getCuda(), 0);
        if (!s_cudaOps.available()) {
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
            s_skipReason = "CUDA not available";
        }
    }

    static void TearDownTestSuite() {
        s_cudaOps.reset(nullptr, 0);
        if (s_handle != 0) {
            ovphysx_enqueue_result_t reset_result = ovphysx_detach_ovstage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                EXPECT_TRUE(wait_task_success(s_handle, reset_result));
            } else {
                EXPECT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);
            }
            ovphysx_destroy_instance(s_handle);
            s_handle = 0;
        }
    }

    void SetUp() override {
        if (s_handle == 0) {
            if (ovphysxTestRequireCuda())
                FAIL() << "GPU/CUDA not available (OVPHYSX_TEST_REQUIRE_CUDA=1): " << s_skipReason;
            GTEST_SKIP() << s_skipReason;
        }
        m_handle = s_handle;
    }

    void TearDown() override {
        if (s_handle != 0) {
            ovphysx_enqueue_result_t reset_result = ovphysx_detach_ovstage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                EXPECT_TRUE(wait_task_success(s_handle, reset_result));
            } else {
                EXPECT_EQ(reset_result.status, OVPHYSX_API_SUCCESS);
            }
            test_utils::destroy_ovstage_test_attachments(s_handle);
        }
        m_handle = 0;
    }
};

ovphysx_instance_handle_t SessionPhysicsGpuTest::s_handle = 0;
ovphysx::test_cuda::CudaOps SessionPhysicsGpuTest::s_cudaOps{};
std::string SessionPhysicsGpuTest::s_skipReason;



TEST_F(SessionPhysicsGpuTest, GpuArticulationDofReadWrite) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample_gpu.usda"))
        << "Failed to load USD";

    ovphysx_result_t result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    Column positions;
    ASSERT_TRUE(readColumn(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, positions, nullptr, ctx))
        << "GPU read failed";
    ASSERT_GT(positions.size(), 0u);

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, { 0.1 }, nullptr,
                           ctx),
              static_cast<int>(positions.size()))
        << "GPU write failed";

    Column readback;
    ASSERT_TRUE(readColumn(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION, readback, nullptr, ctx));
    ASSERT_EQ(readback.size(), positions.size()) << "readback dropped joints";
    for (const std::pair<const std::string, std::vector<double>>& joint : positions)
    {
        ASSERT_EQ(readback.count(joint.first), 1u) << "readback missing joint " << joint.first;
        const std::vector<double>& row = readback[joint.first];
        ASSERT_EQ(row.size(), joint.second.size()) << "component count changed for " << joint.first;
        for (double value : row)
            EXPECT_NEAR(value, 0.1, 0.01) << "DOF position mismatch at " << joint.first;
    }
}

TEST_F(SessionPhysicsGpuTest, GpuDeformableElementIndicesReadInt32) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/volume_deformable_simple.usda"))
        << "Failed to load deformable USD";

    ovphysx_result_t result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/PhysicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    Column elements;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, elements,
                           "/World/DeformableBody", ctx, &kDtypeInt32))
        << "GPU int32 element-index read failed";
    ASSERT_EQ(elements.size(), 1u);
    const std::vector<double> expected = { 0, 1, 2, 3, 1, 2, 3, 4 };
    EXPECT_EQ(elements.begin()->second, expected);
}

TEST_F(SessionPhysicsGpuTest, GpuDeformableBodyReadWriteAndReadOnly) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/volume_deformable_multi.usda"))
        << "Failed to load multi-deformable USD";

    ovphysx_result_t result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/PhysicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    const char* bodies = "/World/DeformableBody_*";
    const char* body0 = "/World/DeformableBody_0";
    const char* body1 = "/World/DeformableBody_1";
    constexpr ovphysx_query_target_t kVolume = OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME;

    Column original;
    ASSERT_TRUE(readColumn(m_handle, kVolume, OVPHYSX_ATTR_POINTS, original, bodies, ctx));
    ASSERT_EQ(original.size(), 2u);
    ASSERT_EQ(original[body0].size(), 15u) << "5 nodes x 3";
    ASSERT_EQ(original[body1].size(), 15u) << "5 nodes x 3";
    ASSERT_NEAR(original[body0][0], 0.0, 1.0e-4);
    ASSERT_NEAR(original[body1][0], 2.0, 1.0e-4);

    // A write selected by the second body leaves the first body untouched.
    std::vector<double> updated = original[body1];
    for (size_t vertex = 0; vertex < 5; ++vertex)
        updated[vertex * 3 + 1] += 0.125;
    ASSERT_EQ(writeColumn(
                  m_handle, kVolume, OVPHYSX_ATTR_POINTS,
                  [&](const std::string&, std::vector<double>& values) { values = updated; }, body1, ctx),
              1);
    Column readback;
    ASSERT_TRUE(readColumn(m_handle, kVolume, OVPHYSX_ATTR_POINTS, readback, bodies, ctx));
    for (size_t i = 0; i < 15; ++i)
    {
        EXPECT_NEAR(readback[body0][i], original[body0][i], 1.0e-4) << "body 0 position mismatch at " << i;
        EXPECT_NEAR(readback[body1][i], updated[i], 1.0e-4) << "body 1 position mismatch at " << i;
    }

    std::vector<double> velocities(15, 0.0);
    for (size_t i = 0; i < 15; i += 3)
        velocities[i] = 0.25;
    ASSERT_EQ(writeColumn(
                  m_handle, kVolume, OVPHYSX_ATTR_VELOCITIES,
                  [&](const std::string&, std::vector<double>& values) { values = velocities; }, body1, ctx),
              1);
    ASSERT_TRUE(readColumn(m_handle, kVolume, OVPHYSX_ATTR_VELOCITIES, readback, bodies, ctx));
    for (size_t i = 0; i < 15; ++i)
    {
        EXPECT_NEAR(readback[body0][i], 0.0, 1.0e-4) << "body 0 velocity mismatch at " << i;
        EXPECT_NEAR(readback[body1][i], velocities[i], 1.0e-4) << "body 1 velocity mismatch at " << i;
    }

    EXPECT_NE(writeOpenStatus(m_handle, kVolume, OVPHYSX_ATTR_REST_POINTS, bodies), OVPHYSX_API_SUCCESS)
        << "rest nodal positions are read-only";

    // Kinematic targets: xyz plus w per node, where w = 0 drives the node. The read serves the targets of
    // a driven body only, so every node is driven here.
    std::vector<double> targets(20, 0.0);
    for (size_t vertex = 0; vertex < 5; ++vertex)
    {
        targets[vertex * 4 + 0] = updated[vertex * 3 + 0];
        targets[vertex * 4 + 1] = updated[vertex * 3 + 1] + 0.05;
        targets[vertex * 4 + 2] = updated[vertex * 3 + 2];
        targets[vertex * 4 + 3] = 0.0;
    }
    ASSERT_EQ(writeColumn(
                  m_handle, kVolume, OVPHYSX_ATTR_KINEMATIC_TARGET,
                  [&](const std::string&, std::vector<double>& values)
                  {
                      ASSERT_EQ(values.size(), targets.size());
                      values = targets;
                  },
                  body1, ctx),
              1);
    ASSERT_TRUE(readColumn(m_handle, kVolume, OVPHYSX_ATTR_KINEMATIC_TARGET, readback, body1, ctx));
    ASSERT_EQ(readback[body1].size(), targets.size());
    for (size_t i = 0; i < targets.size(); ++i)
        EXPECT_NEAR(readback[body1][i], targets[i], 1.0e-4) << "target mismatch at " << i;
}

// Session counterpart of the caller-context checks in CrossDeviceStagingWithoutCallerCudaContext and
// ActiveCudaGpusAttachTest.DirectGpuHostReadOnNonZeroOrdinal: a DirectGPU session write and read, run with
// no CUDA context current on the caller's thread, leave none behind.
TEST_F(SessionPhysicsGpuTest, SessionReadWriteLeaveNoCallerCudaContext) {
    using namespace session_test;
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ASSERT_NE(cuda, nullptr);
    ASSERT_TRUE(cuda->cudaAvailable());

    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample_gpu.usda"));
    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    ASSERT_NE(ctx, 0u);

    ovphysx::test_cuda::ScopedCudaContextDetach detach(cuda);
    ASSERT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda));

    const int written = writeUniform(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_TARGET, { 0.25 }, nullptr, ctx);
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda))
        << "a session write left a CUDA context pushed on the caller's thread";
    Column targets, links;
    const bool readTargets = readColumn(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_VELOCITY_TARGET, targets, nullptr, ctx);
    // Link poses are device-resident on DirectGPU.
    const bool readLinks =
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_POSITION, links, nullptr, ctx);
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda))
        << "a session read left a CUDA context pushed on the caller's thread";

    EXPECT_TRUE(detach.restore()) << "failed to restore the caller's CUDA context stack";

    ASSERT_GT(written, 0);
    ASSERT_TRUE(readTargets);
    ASSERT_TRUE(readLinks);
    EXPECT_GT(links.size(), 0u);
    for (const std::pair<const std::string, std::vector<double>>& joint : targets)
        for (double value : joint.second)
            EXPECT_FLOAT_EQ(static_cast<float>(value), 0.25f) << joint.first;
}

// Session counterparts of the masked and indexed subset writes in test_tensor_binding.cpp
// (MaskedWrite*_ArticulationRootPose_*, MaskedWrite*_DofPositionTargets_*, MaskedWrite*_RigidBody*):
// a write selected by some prims changes exactly those prims and leaves the others as they were.
// `scene` is the physics scene whose CUDA context a DirectGPU column is copied under; a CPU scene has none.
namespace
{
void run_selected_articulation_writes(ovphysx_instance_handle_t handle)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, "tests/data/MultiCartPoleSpaced.usda"));
    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    // Root position of the middle cart-pole only.
    const std::string selected = "/envs/env1";
    Column roots_before, roots_after;
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, roots_before, nullptr, ctx));
    ASSERT_EQ(roots_before.size(), 3u);
    ASSERT_EQ(roots_before.count(selected), 1u);
    ASSERT_EQ(writeColumn(
                  handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values = roots_before[path];
                      values[0] += 0.5;
                  },
                  selected.c_str(), ctx),
              1);
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, roots_after, nullptr, ctx));
    for (const std::pair<const std::string, std::vector<double>>& root : roots_before)
        for (size_t axis = 0; axis < 3; ++axis)
            EXPECT_NEAR(roots_after[root.first][axis], root.second[axis] + (root.first == selected && axis == 0 ? 0.5 : 0.0),
                        1e-4)
                << root.first << " axis " << axis;

    // Position targets of the middle cart-pole's two joints only.
    Column targets_before, targets_after;
    ASSERT_TRUE(readColumn(
        handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION_TARGET, targets_before, nullptr, ctx));
    ASSERT_EQ(targets_before.size(), 6u) << "3 cart-poles x (cart prismatic + pole revolute)";
    ASSERT_EQ(writeUniform(handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION_TARGET, { 0.3 },
                           (selected + "/*").c_str(), ctx),
              2);
    ASSERT_TRUE(readColumn(
        handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_POSITION_TARGET, targets_after, nullptr, ctx));
    for (const std::pair<const std::string, std::vector<double>>& joint : targets_before)
    {
        const bool inSelection = joint.first.rfind(selected + "/", 0) == 0;
        EXPECT_NEAR(targets_after[joint.first].at(0), inSelection ? 0.3 : joint.second.at(0), 1e-5) << joint.first;
    }
}

void run_selected_rigid_body_writes(ovphysx_instance_handle_t handle, const char* usd, const char* scene)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, usd));
    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(handle, scene, OVPHYSX_PHYSX_TYPE_SCENE);

    const char* cubes = "/World/Cube*";
    Column before, velocities_before;
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, before, cubes, ctx));
    ASSERT_TRUE(
        readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, velocities_before, cubes, ctx));
    ASSERT_GT(before.size(), 2u);
    const std::string selected = before.begin()->first;

    // Position and linear velocity of one cube.
    ASSERT_EQ(writeColumn(
                  handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values = before[path];
                      values[2] += 1.0;
                  },
                  selected.c_str(), ctx),
              1);
    ASSERT_EQ(writeUniform(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, { 1.0, 2.0, 3.0 },
                           selected.c_str(), ctx),
              1);

    Column after, velocities;
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, after, cubes, ctx));
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, velocities, cubes, ctx));
    for (const std::pair<const std::string, std::vector<double>>& body : before)
    {
        const bool isSelected = body.first == selected;
        for (size_t axis = 0; axis < 3; ++axis)
            EXPECT_NEAR(after[body.first][axis], body.second[axis] + (isSelected && axis == 2 ? 1.0 : 0.0), 1e-4)
                << body.first << " position axis " << axis;
        for (size_t axis = 0; axis < 3; ++axis)
            EXPECT_NEAR(velocities[body.first][axis],
                        isSelected ? 1.0 + double(axis) : velocities_before[body.first][axis], 1e-4)
                << body.first << " velocity axis " << axis;
    }
}

// A per-shape material write (static friction) to one selected object must give that object's shape its
// own material, leaving a sibling that shares the authored material unchanged -- the isolation AC-4
// promises, for the one attribute family whose leak is through a SHARED PxMaterial rather than a stray
// row (REQ-READ-SELECT-001 AC-8; the binding's lost regression, NVBugs 6489465). Checked for rigid bodies
// and articulation links, which share the one per-shape material write path and pool.
void run_material_write_isolation(ovphysx_instance_handle_t handle, ovphysx_query_target_t type,
                                  const char* usd, const char* scene, const char* pattern)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, usd));
    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(handle, scene, OVPHYSX_PHYSX_TYPE_SCENE);

    Column before;
    ASSERT_TRUE(readColumn(handle, type, OVPHYSX_ATTR_STATIC_FRICTION, before, pattern, ctx));
    ASSERT_GT(before.size(), 2u);
    const std::string selected = before.begin()->first;
    const double baseline = before[selected].at(0);
    // Precondition and the point of the test: the objects share one authored material, so a write that
    // aliased it would change every sibling.
    for (const std::pair<const std::string, std::vector<double>>& body : before)
        ASSERT_NEAR(body.second.at(0), baseline, 1e-6)
            << "the objects must share one material for this to test isolation (" << body.first << ")";

    // A value distinct from the shared baseline, so a leak onto a sibling is observable.
    const double written = baseline + 0.5;
    ASSERT_EQ(writeUniform(handle, type, OVPHYSX_ATTR_STATIC_FRICTION, { written }, selected.c_str(), ctx), 1);

    Column after;
    ASSERT_TRUE(readColumn(handle, type, OVPHYSX_ATTR_STATIC_FRICTION, after, pattern, ctx));
    for (const std::pair<const std::string, std::vector<double>>& body : before)
    {
        // An untouched sibling reads back exactly the shared material's value, so its check is exact; the
        // written shape is an f32 round-trip of baseline + 0.5, so it keeps a small tolerance.
        if (body.first == selected)
            EXPECT_NEAR(after[body.first].at(0), written, 1e-5) << body.first << " (written shape)";
        else
            EXPECT_EQ(after[body.first].at(0), body.second.at(0)) << body.first << " (sibling must keep its own value)";
    }
}

// Session port of test_material_properties_pool_reuse (NVBugs 6489465 / OMPE-102536), the recycle half
// of the lost coverage. Per-shape material writes intern PxMaterials in a refcounted, value-keyed pool
// (BaseSimulationView::mMaterials) shared by the rigid-body and articulation-link write paths. The
// A -> B -> C -> A sequence drives tuple A's material refcount to zero after B (recycled), C repurposes it,
// and re-requesting A must return A's values -- not the recycled C's, which a prior key-drift bug served.
// The session write reaches the same pool as the retired binding, so this is the recycle guard that must
// survive binding removal. writeUniform zero-pads beyond the value list, so the fill vector is wide enough
// to cover a multi-shape prim; the write applies only to real shapes.
void run_material_pool_recycle(ovphysx_instance_handle_t handle, ovphysx_query_target_t type, const char* usd,
                               const char* scene, const char* pattern)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, usd));
    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(handle, scene, OVPHYSX_PHYSX_TYPE_SCENE);

    const auto writeTuple = [&](double sf, double df, double r)
    {
        ASSERT_GT(writeUniform(handle, type, OVPHYSX_ATTR_STATIC_FRICTION, std::vector<double>(64, sf), pattern, ctx), 0);
        ASSERT_GT(writeUniform(handle, type, OVPHYSX_ATTR_DYNAMIC_FRICTION, std::vector<double>(64, df), pattern, ctx), 0);
        ASSERT_GT(writeUniform(handle, type, OVPHYSX_ATTR_RESTITUTION, std::vector<double>(64, r), pattern, ctx), 0);
    };

    writeTuple(0.1, 0.2, 0.3); // A
    writeTuple(0.4, 0.5, 0.6); // B -- every shape leaves tuple A; A's material refcount hits 0 and is recycled
    writeTuple(0.7, 0.8, 0.9); // C -- repurposes the recycled material
    writeTuple(0.1, 0.2, 0.3); // A again -- must return A's values, not the recycled C's

    Column sf, df, r;
    ASSERT_TRUE(readColumn(handle, type, OVPHYSX_ATTR_STATIC_FRICTION, sf, pattern, ctx));
    ASSERT_TRUE(readColumn(handle, type, OVPHYSX_ATTR_DYNAMIC_FRICTION, df, pattern, ctx));
    ASSERT_TRUE(readColumn(handle, type, OVPHYSX_ATTR_RESTITUTION, r, pattern, ctx));
    ASSERT_GT(sf.size(), 0u);
    for (const std::pair<const std::string, std::vector<double>>& prim : sf)
    {
        EXPECT_NEAR(sf[prim.first].at(0), 0.1, 1e-5) << prim.first << " static friction after A->B->C->A (recycle)";
        EXPECT_NEAR(df[prim.first].at(0), 0.2, 1e-5) << prim.first << " dynamic friction after A->B->C->A (recycle)";
        EXPECT_NEAR(r[prim.first].at(0), 0.3, 1e-5) << prim.first << " restitution after A->B->C->A (recycle)";
    }
}
} // namespace

TEST_F(SessionPhysicsCpuTest, SessionSelectedArticulationWritesChangeOnlyTheSelection) {
    run_selected_articulation_writes(m_handle);
}
TEST_F(SessionPhysicsGpuTest, SessionSelectedArticulationWritesChangeOnlyTheSelection) {
    run_selected_articulation_writes(m_handle);
}
TEST_F(SessionPhysicsCpuTest, SessionSelectedRigidBodyWritesChangeOnlyTheSelection) {
    run_selected_rigid_body_writes(m_handle, "tests/data/boxes_falling_on_groundplane.usda", "/World/physicsScene");
}
TEST_F(SessionPhysicsGpuTest, SessionSelectedRigidBodyWritesChangeOnlyTheSelection) {
    run_selected_rigid_body_writes(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda",
                                   "/World/physicsScene");
}
TEST_F(SessionPhysicsCpuTest, SessionSelectedRigidBodyMaterialWriteIsolatedToTheSelection) {
    run_material_write_isolation(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, "tests/data/boxes_falling_on_groundplane.usda",
                                 "/World/physicsScene", "/World/Cube*");
}
TEST_F(SessionPhysicsGpuTest, SessionSelectedRigidBodyMaterialWriteIsolatedToTheSelection) {
    run_material_write_isolation(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY,
                                 "tests/data/boxes_falling_on_groundplane_gpu.usda", "/World/physicsScene",
                                 "/World/Cube*");
}
TEST_F(SessionPhysicsCpuTest, SessionSelectedArticulationLinkMaterialWriteIsolatedToTheSelection) {
    run_material_write_isolation(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, "tests/data/MultiCartPoleSpaced.usda",
                                 "/physicsScene", nullptr);
}
TEST_F(SessionPhysicsGpuTest, SessionSelectedArticulationLinkMaterialWriteIsolatedToTheSelection) {
    run_material_write_isolation(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, "tests/data/MultiCartPoleSpaced.usda",
                                 "/physicsScene", nullptr);
}
TEST_F(SessionPhysicsCpuTest, RigidBodyMaterialWritePoolRecycle) {
    run_material_pool_recycle(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, "tests/data/boxes_falling_on_groundplane.usda",
                              "/World/physicsScene", "/World/Cube*");
}
TEST_F(SessionPhysicsGpuTest, RigidBodyMaterialWritePoolRecycle) {
    run_material_pool_recycle(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, "tests/data/boxes_falling_on_groundplane_gpu.usda",
                              "/World/physicsScene", "/World/Cube*");
}
TEST_F(SessionPhysicsCpuTest, ArticulationLinkMaterialWritePoolRecycle) {
    run_material_pool_recycle(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, "tests/data/MultiCartPoleSpaced.usda",
                              "/physicsScene", nullptr);
}
TEST_F(SessionPhysicsGpuTest, ArticulationLinkMaterialWritePoolRecycle) {
    run_material_pool_recycle(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, "tests/data/MultiCartPoleSpaced.usda",
                              "/physicsScene", nullptr);
}

TEST_F(SessionPhysicsGpuTest, GpuDeformableMaterialElasticityDampingReadWrite) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/volume_deformable_simple.usda"))
        << "Failed to load deformable USD";

    const char* material = "/World/DeformableMaterial";
    Column value;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING,
                           value, material));
    ASSERT_EQ(value.size(), 1u);
    EXPECT_NEAR(value.begin()->second.at(0), 0.01, 1.0e-4); // Value authored in the fixture.

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING,
                           { 0.05 }, material),
              1);

    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_ELASTICITY_DAMPING,
                           value, material));
    EXPECT_NEAR(value.begin()->second.at(0), 0.05, 1.0e-4);
}

TEST_F(SessionPhysicsGpuTest, GpuSurfaceDeformableMaterialBendingPropertiesReadWrite) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/surface_deformable_material.usda"))
        << "Failed to load surface deformable material USD";

    const char* material = "/World/SurfaceDeformableMaterial";
    const auto read = [&](const char* attribute, double& out)
    {
        Column value;
        ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, attribute, value, material)) << attribute;
        ASSERT_EQ(value.size(), 1u) << attribute;
        out = value.begin()->second.at(0);
    };

    // Values authored in the fixture.
    double value = 0.0;
    ASSERT_NO_FATAL_FAILURE(read(OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS, value));
    EXPECT_NEAR(value, 100.0, 1.0e-2);
    ASSERT_NO_FATAL_FAILURE(read(OVPHYSX_ATTR_DEFORMABLE_THICKNESS, value));
    EXPECT_NEAR(value, 0.01, 1.0e-5);
    ASSERT_NO_FATAL_FAILURE(read(OVPHYSX_ATTR_DEFORMABLE_BENDING_DAMPING, value));
    EXPECT_NEAR(value, 0.05, 1.0e-4);

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_MATERIAL, OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS,
                           { 200.0 }, material),
              1);
    ASSERT_NO_FATAL_FAILURE(read(OVPHYSX_ATTR_DEFORMABLE_BENDING_STIFFNESS, value));
    EXPECT_NEAR(value, 200.0, 1.0e-2);
}

TEST_F(SessionPhysicsGpuTest, GpuVolumeDeformableCollisionElementIndicesRead) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/volume_deformable_simple.usda"))
        << "Failed to load deformable USD";

    ovphysx_result_t result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/PhysicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    const char* body = "/World/DeformableBody";
    Column elements;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_COLLISION_ELEMENT_INDICES,
                           elements, body, ctx, &kDtypeInt32))
        << "Collision element index read failed";
    ASSERT_EQ(elements.size(), 1u);
    const std::vector<double>& indices = elements.begin()->second;
    ASSERT_GT(indices.size(), 0u);
    EXPECT_EQ(indices.size() % 4, 0u) << "4 nodes per tetrahedron";
    for (double index : indices)
        EXPECT_GE(index, 0.0);

    // Element indices are read-only.
    EXPECT_NE(writeOpenStatus(m_handle, OVPHYSX_QUERY_TARGET_DEFORMABLE_VOLUME, OVPHYSX_ATTR_COLLISION_ELEMENT_INDICES, body),
              OVPHYSX_API_SUCCESS);
}

TEST_F(SessionPhysicsGpuTest, GpuSurfaceDeformableBodyReadWriteAndReadOnly) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/surface_deformable_simple.usda"))
        << "Failed to load surface deformable USD";

    ovphysx_result_t result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/PhysicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    const char* body = "/World/DeformableBody";
    constexpr ovphysx_query_target_t kSurface = OVPHYSX_QUERY_TARGET_DEFORMABLE_SURFACE;

    Column positions;
    ASSERT_TRUE(readColumn(m_handle, kSurface, OVPHYSX_ATTR_POINTS, positions, body, ctx));
    ASSERT_EQ(positions.size(), 1u);
    const std::vector<double> original = positions.begin()->second;
    ASSERT_EQ(original.size(), 12u); // Square cloth with four simulation nodes.
    // The fixture places the first node at the origin.
    EXPECT_NEAR(original[0], 0.0, 1.0e-4);
    EXPECT_NEAR(original[1], 0.0, 1.0e-4);
    EXPECT_NEAR(original[2], 0.0, 1.0e-4);

    std::vector<double> moved = original;
    for (size_t vertex = 0; vertex < 4; ++vertex)
        moved[vertex * 3 + 1] += 0.05;
    ASSERT_EQ(writeColumn(
                  m_handle, kSurface, OVPHYSX_ATTR_POINTS,
                  [&](const std::string&, std::vector<double>& values) { values = moved; }, body, ctx),
              1);

    Column elements;
    ASSERT_TRUE(readColumn(m_handle, kSurface, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, elements, body, ctx, &kDtypeInt32));
    ASSERT_EQ(elements.size(), 1u);
    EXPECT_EQ(elements.begin()->second.size(), 6u) << "2 triangles x 3 nodes";
    for (double index : elements.begin()->second)
        EXPECT_GE(index, 0.0);

    // Rest positions and element indices are read-only.
    EXPECT_NE(writeOpenStatus(m_handle, kSurface, OVPHYSX_ATTR_REST_POINTS, body), OVPHYSX_API_SUCCESS);
    EXPECT_NE(writeOpenStatus(m_handle, kSurface, OVPHYSX_ATTR_SIM_ELEMENT_INDICES, body), OVPHYSX_API_SUCCESS);
}

// OMPE-94459 / OMPE-103213: DISABLE_SIMULATION_BOOL is CPU-only even on a DirectGPU
// scene. Host all-enabled tensors round-trip. Writing any disable=1 invalidates the
// binding because DirectGPU has no row for the body. CUDA tensors are refused.


TEST_F(SessionPhysicsGpuTest, GpuRigidBodyDisableGravityRoundtrip) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"))
        << "Failed to load USD";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    // Alternate 1/0 in prim-path order to exercise both set and cleared flags.
    std::map<std::string, double> written;
    ASSERT_GT(writeColumn(
                  m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values[0] = written.size() % 2 == 0 ? 1.0 : 0.0;
                      written[path] = values[0];
                  },
                  "/World/Cube*", ctx),
              1);

    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY, readback, "/World/Cube*", ctx, &kDtypeUInt8));
    ASSERT_EQ(readback.size(), written.size());
    for (const std::pair<const std::string, double>& flag : written)
        EXPECT_EQ(readback[flag.first][0], flag.second) << flag.first << " disable-gravity round-trip";
}

// Companion to CpuRigidBodyDisableGravitySuppressesFall. The umbrella sets disable
// flags before the first physics step and checks vz on step 1, with no mid-flight swap.
TEST_F(SessionPhysicsGpuTest, GpuRigidBodyDisableGravitySuppressesFall) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda"))
        << "Failed to load USD";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    // Gravity is disabled on every other body (prim-path order) before the first step, as in the umbrella.
    const char* cubes = "/World/Cube*";
    std::map<std::string, bool> disabled;
    ASSERT_GT(writeColumn(
                  m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_DISABLE_GRAVITY,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      disabled[path] = disabled.size() % 2 == 0;
                      values[0] = disabled[path] ? 1.0 : 0.0;
                  },
                  cubes, ctx),
              0);

    ASSERT_TRUE(stepN(m_handle, 1));

    Column vels;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_LINEAR_VELOCITY, vels, cubes, ctx));
    const double dt = 1.0 / 60.0;
    const double expected_fall_vz = -dt * 9.81;
    for (const std::pair<const std::string, bool>& body : disabled) {
        const double vz = vels[body.first][2];
        if (body.second)
            EXPECT_NEAR(vz, 0.0, 1e-2) << body.first << " should not fall with gravity disabled";
        else
            EXPECT_NEAR(vz, expected_fall_vz, 0.05) << body.first << " should fall under gravity";
    }
}

// OMPE-103213: articulation disable-gravity is CPU-only on DirectGPU scenes too.
TEST_F(SessionPhysicsGpuTest, GpuArticulationDisableGravityRoundtrip) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/two_articulations_gpu.usda"))
        << "Failed to load USD";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    // Alternate 1/0 in prim-path order to exercise both set and cleared flags.
    std::map<std::string, double> written;
    ASSERT_GT(writeColumn(
                  m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY,
                  [&](const std::string& path, std::vector<double>& values)
                  {
                      values[0] = written.size() % 2 == 0 ? 1.0 : 0.0;
                      written[path] = values[0];
                  },
                  nullptr, ctx),
              1);

    Column readback;
    ASSERT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, readback, nullptr, ctx, &kDtypeUInt8));
    ASSERT_EQ(readback.size(), written.size());
    for (const std::pair<const std::string, double>& flag : written)
        EXPECT_EQ(readback[flag.first][0], flag.second) << flag.first << " link disable-gravity round-trip";
}

TEST_F(SessionPhysicsGpuTest, GpuArticulationDisableGravitySuppressesFall) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/mixed_base_articulations_gpu.usda"))
        << "Failed to load USD";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    const char* articulation = "/World/articulation2";
    const char* links = "/World/articulation2/*";
    const auto read_root_z = [&](double& out_z)
    {
        Column root;
        ASSERT_TRUE(
            readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_ROOT_POSITION, root, articulation, ctx));
        ASSERT_EQ(root.size(), 1u);
        out_z = root.begin()->second[2];
    };

    // Disable gravity before the first step. Stepping under gravity first would
    // leave the root coasting downward, so a stationary-height assertion could
    // be satisfied by that momentum rather than by gravity suppression.
    ASSERT_GT(
        writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, { 1 }, links, ctx), 0);

    double z0 = 0.0;
    ASSERT_NO_FATAL_FAILURE(read_root_z(z0));

    ASSERT_TRUE(stepN(m_handle, 20));
    double z_disabled = 0.0;
    ASSERT_NO_FATAL_FAILURE(read_root_z(z_disabled));
    EXPECT_NEAR(z_disabled, z0, 0.01) << "Floating-base root Z should stay stable with gravity disabled";

    // Re-enable and compare against the height reached while disabled, so the
    // fall cannot be credited to drift accumulated before gravity was restored.
    ASSERT_GT(
        writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_DISABLE_GRAVITY, { 0 }, links, ctx), 0);
    ASSERT_TRUE(stepN(m_handle, 20));
    double z_reenabled = 0.0;
    ASSERT_NO_FATAL_FAILURE(read_root_z(z_reenabled));
    EXPECT_LT(z_reenabled, z_disabled - 0.05) << "Floating-base root should fall after re-enabling gravity";
}

// Articulation world mass-center read on a DirectGPU scene (OMPE-94459). Only the shape
// and read-success contract is checked. Numeric correctness lives in the umbrella
// tensor suite.
TEST_F(SessionPhysicsGpuTest, GpuArticulationMassCenterRead) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample_gpu.usda"))
        << "Failed to load USD";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    Column com;
    EXPECT_TRUE(readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION, OVPHYSX_ATTR_CENTER_OF_MASS_WORLD, com,
                           "/World/articulation", ctx));
    ASSERT_EQ(com.size(), 1u);
    EXPECT_EQ(com.begin()->second.size(), 3u);
}

// OMPE-103213: shape properties such as the rest offset are CPU-only PhysX APIs, and
// still round-trip on a DirectGPU scene.
TEST_F(SessionPhysicsGpuTest, GpuArticulationShapePropertyWrite_B9) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/links_chain_sample_gpu.usda"))
        << "Failed to load USD";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/World/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    // One per-shape row per link, as wide as the link's shape count.
    const int links = writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_REST_OFFSET,
                                   std::vector<double>(1024, 0.0005), nullptr, ctx);
    ASSERT_GT(links, 0);

    Column rest_offsets;
    ASSERT_TRUE(
        readColumn(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_LINK, OVPHYSX_ATTR_REST_OFFSET, rest_offsets, nullptr, ctx));
    ASSERT_EQ(rest_offsets.size(), static_cast<size_t>(links));
    int matched = 0;
    for (const std::pair<const std::string, std::vector<double>>& link : rest_offsets)
        for (double value : link.second)
            if (std::abs(value - 0.0005) < 1e-4)
                ++matched;
    EXPECT_GT(matched, 0) << "expected at least one shape to roundtrip the rest-offset write";
}

// Multi-articulation CartPole on GPU with envs spaced so the 6m rails do not penetrate
// adjacent rails. The read must return clean per-env projected joint forces on a
// well-formed scene. A scene with overlapping rails failing this points at the scene,
// not at the engine.
TEST_F(SessionPhysicsGpuTest, GpuMultiCartPoleSpacedProjectedJointForceMatchesActuation) {
    using namespace session_test;
    // MultiCartPoleSpacedForcesOncePerStep.usda is MultiCartPoleSpaced.usda with external forces applied once per step,
    // which this test expects.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/MultiCartPoleSpacedForcesOncePerStep.usda"))
        << "Failed to load MultiCartPoleSpacedForcesOncePerStep.usda";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    const int written = writeUniform(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ACTUATION_FORCE, { 10.0 }, nullptr, ctx);
    ASSERT_GT(written, 0);

    ASSERT_TRUE(stepN(m_handle, 1, 1.0f / 1000.0f));

    Column projected;
    ASSERT_TRUE(readColumn(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, projected, nullptr, ctx));
    ASSERT_EQ(projected.size(), static_cast<size_t>(written));

    for (const std::pair<const std::string, std::vector<double>>& joint : projected)
        std::cerr << "[GPU multi-cartpole-spaced step-1] " << joint.first << "=" << joint.second.at(0) << "\n";
    for (const std::pair<const std::string, std::vector<double>>& joint : projected)
        EXPECT_NEAR(joint.second.at(0), 10.0, 0.9) << joint.first << " projection diverged";
}

// GPU counterpart of CpuCartPoleProjectedJointForceMatchesActuation. The projected
// joint forces must land within tolerance of the applied actuation.
TEST_F(SessionPhysicsGpuTest, GpuCartPoleProjectedJointForceMatchesActuation) {
    using namespace session_test;
    // CartPoleForcesOncePerStep.usda is CartPole.usda with external forces applied once per step,
    // which this test expects.
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/CartPoleForcesOncePerStep.usda"))
        << "Failed to load CartPoleForcesOncePerStep.usda";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    ASSERT_EQ(writeUniform(m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_ACTUATION_FORCE, { 10.0 },
                           nullptr, ctx),
              2);

    ASSERT_TRUE(stepN(m_handle, 1, 1.0f / 1000.0f));

    Column projected;
    ASSERT_TRUE(readColumn(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, projected, nullptr, ctx));
    const double cart = projected["/cartpole/cartJoint"].at(0);
    const double pole = projected["/cartpole/poleJoint"].at(0);

    std::cerr << "[GPU cartpole step-1] cart-prismatic=" << cart << " pole-revolute=" << pole << "\n";

    EXPECT_NEAR(cart, 10.0, 0.9) << "cart-prismatic projection diverged";
    EXPECT_NEAR(pole, 10.0, 0.9) << "pole-revolute projection diverged";
}

// After warmup, the GPU projected joint forces must hold the real projection. Gravity
// along Z projects to zero on the cart-rail prismatic Y axis, so the expected value is
// about 0.
TEST_F(SessionPhysicsGpuTest, GpuCartPoleProjectedJointForceWarmStartZero) {
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(m_handle, "tests/data/CartPole.usda"))
        << "Failed to load CartPole.usda";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);
    const uintptr_t ctx = cudaContextOf(m_handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);

    Column projected;
    ASSERT_TRUE(readColumn(
        m_handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, projected, nullptr, ctx));
    ASSERT_EQ(projected.size(), 2u) << "CartPole has 2 DOFs (cart prismatic + pole revolute)";

    for (const std::pair<const std::string, std::vector<double>>& joint : projected)
        for (double force : joint.second)
            EXPECT_LT(std::abs(force), 1e-3) << "GPU " << joint.first << " warm-start projection: expected ~0, got "
                                             << force;
}

// Standalone rigid ball with no articulation in the scene (OMPE-94459). Applies 150N
// +Z at world (ball.xy, ball.z+1), the same magnitude and anchor as the multi-body
// reproducer, so the lift here isolates direct rigid-body force application from
// articulation-ball contact.
namespace
{
void run_ball_only_lift_probe(ovphysx_instance_handle_t handle, const char* usda_relpath, float& out_dz)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, usda_relpath))
        << "Failed to load " << usda_relpath;

    // 0 on a CPU scene.
    const uintptr_t ctx = cudaContextOf(handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    const char* ball = "/envs/env0/right_ball";

    // A DirectGPU read serves state after the first simulate.
    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
    Column pose;
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose, ball, ctx));
    ASSERT_EQ(pose.size(), 1u);
    const std::vector<double> pose_init = pose.begin()->second;

    const std::vector<double> wrench = { 0, 0, 150, 0, 0, 0, pose_init[0], pose_init[1], pose_init[2] + 1 };
    ASSERT_EQ(writeUniform(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_WRENCH, wrench, ball, ctx), 1);

    ASSERT_TRUE(stepN(handle, 10));

    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose, ball, ctx));
    const std::vector<double>& pose_post = pose.begin()->second;

    const float dz = static_cast<float>(pose_post[2] - pose_init[2]);
    std::cerr << "  right_ball init=(" << pose_init[0] << "," << pose_init[1] << "," << pose_init[2]
              << ") post=(" << pose_post[0] << "," << pose_post[1] << "," << pose_post[2]
              << ") dz=" << dz << "\n";

    out_dz = dz;
}
} // namespace

// Standalone rigid-body force-at-position must be translation-invariant and produce
// the expected lift on CPU and GPU at both world positions (OMPE-94459). Over 10 steps
// at dt=1/60s with mass=0.5kg, gravity=20m/s^2 and a single F=150N impulse the lift is
// about 0.528m. All four variants agree to within 1mm.
TEST_F(SessionPhysicsCpuTest, CpuBallOnly_AtOrigin) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyAtOrigin.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}
TEST_F(SessionPhysicsCpuTest, CpuBallOnly_Far) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyFar.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}
TEST_F(SessionPhysicsGpuTest, GpuBallOnly_AtOrigin) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyAtOrigin.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}
TEST_F(SessionPhysicsGpuTest, GpuBallOnly_Far) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyFar.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}

// CartPole projected joint force before the first full step must be about 0 regardless
// of world position (OMPE-94459). CpuCartPoleProjectedJointForcePreStepZero covers the world
// origin. These variants cover world (100, 100, 0), which gates the Rw seed precision
// for prismatic and revolute chains far from the origin.
namespace
{
void run_cartpole_pre_step_zero(ovphysx_instance_handle_t handle, const char* usda_relpath)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, usda_relpath))
        << "Failed to load " << usda_relpath;

    // The articulation is readable only after the first simulate; warmup is that simulate, a 1 ns step.
    ASSERT_EQ(ovphysx_warmup(handle).status, OVPHYSX_API_SUCCESS);
    // 0 on a CPU scene.
    const uintptr_t ctx = cudaContextOf(handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    Column dof_forces;
    ASSERT_TRUE(readColumn(
        handle, OVPHYSX_QUERY_TARGET_ARTICULATION_JOINT, OVPHYSX_ATTR_JOINT_PROJECTED_FORCE, dof_forces, nullptr, ctx));
    ASSERT_EQ(dof_forces.size(), 2u) << "CartPole has 2 DOFs (cart prismatic + pole revolute)";

    for (const std::pair<const std::string, std::vector<double>>& joint : dof_forces)
        for (double force : joint.second)
            EXPECT_LT(std::abs(force), 1e-3)
                << joint.first << " projection before the first full step at " << usda_relpath << ": expected ~0, got " << force;
}
} // namespace

TEST_F(SessionPhysicsCpuTest, CpuCartPolePreStepZero_AtOrigin) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleAtOrigin.usda");
}
TEST_F(SessionPhysicsCpuTest, CpuCartPolePreStepZero_Far) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleFar.usda");
}
TEST_F(SessionPhysicsGpuTest, GpuCartPolePreStepZero_AtOrigin) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleAtOrigin.usda");
}
TEST_F(SessionPhysicsGpuTest, GpuCartPolePreStepZero_Far) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleFar.usda");
}

// A PhysicsSphericalJoint between a kinematic base and a free child under an articulation
// root, loaded at the world origin and at (100, 100, 0) (OMPE-94459). PhysX refuses an
// articulation with a kinematic link, so the pair simulates as two rigid bodies and a
// spherical joint. The child settles to the joint constraint at the same env-relative
// position in both.
namespace
{
void run_spherical_arti_settle_probe(ovphysx_instance_handle_t handle,
                                     const char* usda_relpath,
                                     float env_x, float env_y,
                                     float& out_child_local_z)
{
    using namespace session_test;
    ASSERT_TRUE(test_utils::attach_usd_with_ovstage(handle, usda_relpath))
        << "Failed to load " << usda_relpath;

    ASSERT_TRUE(stepN(handle, 20));

    // 0 on a CPU scene.
    const uintptr_t ctx = cudaContextOf(handle, "/physicsScene", OVPHYSX_PHYSX_TYPE_SCENE);
    Column pose;
    // The child is a rigid body, not an articulation link.
    ASSERT_TRUE(readColumn(handle, OVPHYSX_QUERY_TARGET_RIGID_BODY, OVPHYSX_ATTR_POSITION, pose,
                           "/envs/env0/spherical_arti/child", ctx));
    ASSERT_EQ(pose.size(), 1u);
    const std::vector<double>& pose_post = pose.begin()->second;

    // With aligned anchors and gravity only there is no swing, so the child ends at
    // world X = env_x and Y = env_y. Both variants have a zero env Z offset, so the
    // returned world Z equals the env-local Z.
    EXPECT_NEAR(pose_post[0], env_x, 0.01) << "child x should stay at env origin under pure-gravity hang";
    EXPECT_NEAR(pose_post[1], env_y, 0.01) << "child y should stay at env origin under pure-gravity hang";
    out_child_local_z = static_cast<float>(pose_post[2]);

    std::cerr << "  spherical child post=(" << pose_post[0] << "," << pose_post[1] << "," << pose_post[2]
              << ") env=(" << env_x << "," << env_y << ") local_z=" << out_child_local_z << "\n";
}
} // namespace

TEST_F(SessionPhysicsCpuTest, CpuSphericalArtiSettle_AtOrigin) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiAtOrigin.usda", 0.f, 0.f, z);
    // The base anchor is at z=1.9 (kinematic base at z=2 minus 0.1) and the child
    // anchor is 0.2 above the child center, so the settled child center is at
    // world z=1.7. The loose tolerance covers the iterative settle.
    EXPECT_NEAR(z, 1.7f, 0.05f) << "spherical settle: child center should reach world z=1.7";
}
TEST_F(SessionPhysicsCpuTest, CpuSphericalArtiSettle_Far) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiFar.usda", 100.f, 100.f, z);
    EXPECT_NEAR(z, 1.7f, 0.05f) << "translation invariance: spherical settle world z same at Far";
}
TEST_F(SessionPhysicsGpuTest, GpuSphericalArtiSettle_AtOrigin) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiAtOrigin.usda", 0.f, 0.f, z);
    EXPECT_NEAR(z, 1.7f, 0.05f);
}
TEST_F(SessionPhysicsGpuTest, GpuSphericalArtiSettle_Far) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiFar.usda", 100.f, 100.f, z);
    EXPECT_NEAR(z, 1.7f, 0.05f) << "translation invariance: GPU spherical settle world z same at Far";
}
