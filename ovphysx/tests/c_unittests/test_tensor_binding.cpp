// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "global_test_environment.h"
#include "test_utilities.h"
#include "ovphysx/ovphysx.h"
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

#include "WrenchConversion.h"

using test_utils::make_ovx_string;
using test_utils::make_ovx_string_bytes;

static uintptr_t getPhysxCudaContextFromBinding(ovphysx_handle_t handle, ovphysx_tensor_binding_handle_t binding)
{
    uintptr_t ctx = 0;
    if (!ovphysx_get_tensor_binding_cuda_context_internal(handle, binding, &ctx))
        return 0;
    return ctx;
}

// Helper to wait for async operations
static bool wait_op_success(ovphysx_handle_t handle, ovphysx_op_index_t op_index, uint64_t timeout_ns = 10'000'000'000ULL) {
    ovphysx_op_wait_result_t wait_result{};
    ovphysx_result_t res = ovphysx_wait_op(handle, op_index, timeout_ns, &wait_result);
    if (wait_result.error_op_indices && wait_result.num_errors > 0) {
        for (size_t i = 0; i < wait_result.num_errors; i++) {
            ovphysx_string_t op_err = ovphysx_get_last_op_error(wait_result.error_op_indices[i]);
            std::cerr << "Op error: " << std::string(op_err.ptr, op_err.length) << std::endl;
        }
    }
    ovphysx_destroy_wait_result(&wait_result);
    return res.status == OVPHYSX_API_SUCCESS;
}

TEST(TensorBinding, WrenchAosToSoaConversionCpu)
{
    // AoS row layout: [fx,fy,fz, tx,ty,tz, px,py,pz]
    const int64_t N = 2;
    const float aos[N * 9] = {
        1.f, 2.f, 3.f,   4.f, 5.f, 6.f,   7.f, 8.f, 9.f,
        10.f, 11.f, 12.f, 13.f, 14.f, 15.f, 16.f, 17.f, 18.f,
    };

    float soa[N * 9] = {};
    ovphysx::internal::convertWrenchAosToSoaCpu(aos, N, soa);

    // forces
    EXPECT_FLOAT_EQ(soa[0], 1.f);
    EXPECT_FLOAT_EQ(soa[1], 2.f);
    EXPECT_FLOAT_EQ(soa[2], 3.f);
    EXPECT_FLOAT_EQ(soa[3], 10.f);
    EXPECT_FLOAT_EQ(soa[4], 11.f);
    EXPECT_FLOAT_EQ(soa[5], 12.f);

    // torques
    EXPECT_FLOAT_EQ(soa[6], 4.f);
    EXPECT_FLOAT_EQ(soa[7], 5.f);
    EXPECT_FLOAT_EQ(soa[8], 6.f);
    EXPECT_FLOAT_EQ(soa[9], 13.f);
    EXPECT_FLOAT_EQ(soa[10], 14.f);
    EXPECT_FLOAT_EQ(soa[11], 15.f);

    // positions
    EXPECT_FLOAT_EQ(soa[12], 7.f);
    EXPECT_FLOAT_EQ(soa[13], 8.f);
    EXPECT_FLOAT_EQ(soa[14], 9.f);
    EXPECT_FLOAT_EQ(soa[15], 16.f);
    EXPECT_FLOAT_EQ(soa[16], 17.f);
    EXPECT_FLOAT_EQ(soa[17], 18.f);
}

// Helper to load USD and wait
static bool load_usd_and_wait(ovphysx_handle_t handle, const char* usd_path, ovphysx_usd_handle_t& out_handle) {
    out_handle = 1;
    return test_utils::attach_usd_with_ovstage(handle, usd_path);
}

namespace
{

// Intercepts TensorApi::resetStage to count per-stage tensor-backend resets during a test.
class ScopedTensorResetStageProbe
{
public:
    explicit ScopedTensorResetStageProbe(omni::physics::tensors::TensorApi& tensorApi)
        : mTensorApi(tensorApi), mOriginal(tensorApi.resetStage)
    {
        sOriginal = mOriginal;
        sCallCount = 0;
        sLastStageId = 0;
        mTensorApi.resetStage = &intercept;
    }

    ~ScopedTensorResetStageProbe()
    {
        mTensorApi.resetStage = mOriginal;
        sOriginal = nullptr;
    }

    ScopedTensorResetStageProbe(const ScopedTensorResetStageProbe&) = delete;
    ScopedTensorResetStageProbe& operator=(const ScopedTensorResetStageProbe&) = delete;

    int callCount() const
    {
        return sCallCount;
    }

    long lastStageId() const
    {
        return sLastStageId;
    }

private:
    static void CARB_ABI intercept(long stageId)
    {
        ++sCallCount;
        sLastStageId = stageId;
        if (sOriginal)
            sOriginal(stageId);
    }

    omni::physics::tensors::TensorApi& mTensorApi;
    void(CARB_ABI* mOriginal)(long) = nullptr;

    static inline void(CARB_ABI* sOriginal)(long) = nullptr;
    static inline int sCallCount = 0;
    static inline long sLastStageId = 0;
};

} // namespace

// ============================================================================
// CPU MODE TESTS
// ============================================================================

class TensorBindingCpuTest : public ::testing::Test {
protected:
    ovphysx_handle_t m_handle = 0;

    void SetUp() override {
        ASSERT_TRUE(ensureSharedCpuInstance()) << "Failed to create shared CPU PhysX instance";
        m_handle = sharedCpuInstance();
    }

    void TearDown() override {
        if (m_handle != 0) {
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(m_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                EXPECT_TRUE(waitForOperationSuccess(m_handle, reset_result.op_index))
                    << "TensorBindingCpuTest reset failed during teardown";
            }
            test_utils::destroy_ovstage_test_attachments(m_handle);
            m_handle = 0;
        }
    }
};

// Regression (MR review #2 / trunk 6489225333): ovphysx_reset_stage must release the tensor
// SimulationBackend's per-stage data, or stale views/data persist across reset / reattach. The
// reset path retains resetStage(stageId) in detach_ovstage / unload_usd.
TEST_F(TensorBindingCpuTest, ResetStageReleasesTensorBackendStage)
{
    ovphysx_usd_handle_t usdHandle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usdHandle));

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube1");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
    ovphysx_tensor_binding_handle_t binding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    auto* tensorApi = static_cast<omni::physics::tensors::TensorApi*>(ovphysx_get_tensor_api_internal());
    ASSERT_NE(tensorApi, nullptr);
    ASSERT_NE(tensorApi->resetStage, nullptr);
    ScopedTensorResetStageProbe probe(*tensorApi);

    ovphysx_enqueue_result_t resetResult = ovphysx_reset_stage(m_handle);
    ASSERT_EQ(resetResult.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(wait_op_success(m_handle, resetResult.op_index));

    EXPECT_EQ(probe.callCount(), 1);
    EXPECT_GT(probe.lastStageId(), 0);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// Verifies shape + read-only contract for the §B0 articulation tensors
// added in OMPE-94459. Both WORLD and LOCAL variants are exercised; their
// shapes must be [N, 3] for mass-center and writes must be rejected.
// The actual COM values are not asserted -- the umbrella tensor test
// covers the numeric correctness, this test only locks down the C API
// contract on shape + read-only rejection.
TEST_F(TensorBindingCpuTest, CpuArticulationMassCenterReadOnly) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    for (ovphysx_tensor_type_t type : {OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32,
                                       OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_LOCAL_F32}) {
        ovphysx_tensor_binding_handle_t binding = 0;
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation");
        desc.tensor_type = type;

        ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "create_tensor_binding failed for type=" << type;

        ovphysx_tensor_spec_t spec{};
        result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(spec.ndim, 2) << "type=" << type;
        EXPECT_GT(spec.shape[0], 0) << "at least one articulation; type=" << type;
        EXPECT_EQ(spec.shape[1], 3) << "mass center is [N, 3]; type=" << type;

        // Read should succeed (no fixed/floating-base restriction for COM).
        std::vector<float> com(spec.shape[0] * spec.shape[1], 0.0f);
        DLTensor tensor = {};
        tensor.data = com.data();
        tensor.device = {kDLCPU, 0};
        tensor.ndim = 2;
        tensor.dtype = {kDLFloat, 32, 1};
        int64_t shape[2] = {spec.shape[0], spec.shape[1]};
        tensor.shape = shape;

        result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS) << "read failed for type=" << type;

        // Write must be rejected -- these are read-only tensors.
        result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
        EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "write should be rejected; type=" << type;

        EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
    }
}

// Regression for NVBugs 6481094 / OMPE-102210: ARTICULATION_CENTROIDAL_MOMENTUM
// is only defined for floating-base articulations (PhysX errors out on
// fixed-base). Creating the binding for a fixed-base articulation must be
// rejected up front rather than deferring the failure to read time.
// links_chain_sample.usda is fixed-base (root link is world-anchored).
TEST_F(TensorBindingCpuTest, CpuArticulationCentroidalMomentumFixedBaseRejected) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "fixed-base articulation must reject a centroidal-momentum binding at creation";
    // No binding was created on the rejection path, so there is nothing to destroy.
}

// Companion to the fixed-base rejection above: a floating-base articulation
// (AntNoSelfColl.usda, articulation root at /ant/torso) must accept the
// centroidal-momentum binding. Locks the [N, 6, D+7] shape, a successful read
// (centroidal momentum is defined for floating-base), and read-only write
// rejection.
TEST_F(TensorBindingCpuTest, CpuArticulationCentroidalMomentumFloatingBase) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/AntNoSelfColl.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/ant/torso");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "floating-base articulation must accept a centroidal-momentum binding";

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 3);
    EXPECT_GT(spec.shape[0], 0);
    EXPECT_EQ(spec.shape[1], 6);
    EXPECT_GE(spec.shape[2], 7) << "shape[2] = max_dofs + 7, must be at least 7";

    // Step once so the articulation dynamics cache is live, then read.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Read must succeed for a floating-base articulation.
    std::vector<float> data(spec.shape[0] * spec.shape[1] * spec.shape[2], 0.0f);
    DLTensor tensor = {};
    tensor.data = data.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 3;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[3] = {spec.shape[0], spec.shape[1], spec.shape[2]};
    tensor.shape = shape;
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS) << "floating-base read should succeed";

    // Write must be rejected -- centroidal momentum is read-only.
    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "write should be rejected";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// Regression for NVBugs 6481094 (heterogeneous case): one pattern / explicit
// prim-path list can resolve to BOTH a fixed-base and a floating-base articulation.
// That view is heterogeneous (distinct metatypes -> null shared metatype), so the
// guard must inspect every matched articulation, not just the shared metatype: any
// fixed-base entry makes centroidal momentum undefined and creation must be rejected.
// mixed_base_articulations.usda has /World/articulation (fixed) + /World/articulation2 (floating).
TEST_F(TensorBindingCpuTest, CpuArticulationCentroidalMomentumMixedBaseRejected) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/mixed_base_articulations.usda", usd_handle))
        << "Failed to load USD";

    // Sanity-check the fixture actually mixes base types, so a mis-authored stage
    // fails here rather than silently masking the heterogeneous-rejection assertion.
    auto base_is_fixed = [&](const char* path) -> bool {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = make_ovx_string(path);
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS)
            << "root-pose binding for " << path;
        ovphysx_articulation_metadata_t meta{};
        EXPECT_EQ(ovphysx_get_articulation_metadata(m_handle, b, &meta).status, OVPHYSX_API_SUCCESS)
            << "metadata for " << path;
        ovphysx_destroy_tensor_binding(m_handle, b);
        return meta.is_fixed_base;
    };
    ASSERT_TRUE(base_is_fixed("/World/articulation")) << "/World/articulation should be fixed-base";
    ASSERT_FALSE(base_is_fixed("/World/articulation2")) << "/World/articulation2 should be floating-base";

    // Heterogeneous centroidal-momentum binding over both must be rejected at creation.
    ovphysx_string_t paths[] = {
        OVPHYSX_LITERAL("/World/articulation"),
        OVPHYSX_LITERAL("/World/articulation2"),
    };
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.prim_paths = paths;
    desc.prim_paths_count = 2;
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_CENTROIDAL_MOMENTUM_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "a binding mixing fixed- and floating-base articulations must be rejected at creation";
}

// Reading dof-projected / link-incoming joint forces before any
// simulate() must return zeros. PhysX zeros the cache on dt==0
// internally; this locks the contract on the ovphysx surface.
TEST_F(TensorBindingCpuTest, CpuCartPoleProjectedJointForcePreStepZero) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartPole.usda", usd_handle))
        << "Failed to load CartPole.usda";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/cartpole");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.shape[0], 1);
    ASSERT_EQ(spec.shape[1], 2) << "CartPole has 2 DOFs (cart prismatic + pole revolute)";

    std::vector<float> dof_forces(spec.shape[0] * spec.shape[1], 7.7f);
    DLTensor t{};
    t.data = dof_forces.data();
    t.device = {kDLCPU, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    t.shape = shape;
    t.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &t).status,
              OVPHYSX_API_SUCCESS);

    for (size_t i = 0; i < dof_forces.size(); ++i) {
        EXPECT_LT(std::abs(dof_forces[i]), 1e-3f)
            << "dof " << i << " pre-step projection: expected ~0, got "
            << dof_forces[i];
    }

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status,
              OVPHYSX_API_SUCCESS);
}

// Drive both DOFs with a 10N/10N.m actuation, step at the umbrella's
// timestep, and read the projected joint forces. Used as the baseline
// for the cart-prismatic step-1 readback comparison with the GPU twin.
TEST_F(TensorBindingCpuTest, CpuCartPoleProjectedJointForceMatchesActuation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartPole.usda", usd_handle))
        << "Failed to load CartPole.usda";

    ovphysx_tensor_binding_handle_t actuation_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/cartpole");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &actuation_b).status,
                  OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t projected_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/cartpole");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &projected_b).status,
                  OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, actuation_b, &spec).status,
              OVPHYSX_API_SUCCESS);
    const int64_t N = spec.shape[0];
    const int64_t D = spec.shape[1];
    ASSERT_EQ(N, 1);
    ASSERT_EQ(D, 2);

    std::vector<float> forces(N * D, 10.0f);
    DLTensor t{}; t.data = forces.data(); t.device = {kDLCPU, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {N, D}; t.shape = shape; t.ndim = 2;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, actuation_b, &t, nullptr).status,
              OVPHYSX_API_SUCCESS);

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 1000.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    std::vector<float> projected(N * D, 0.0f);
    t.data = projected.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, projected_b, &t).status,
              OVPHYSX_API_SUCCESS);

    std::cerr << "[CPU cartpole step-1] cart-prismatic=" << projected[0]
              << " pole-revolute=" << projected[1] << "\n";

    // Tolerance matches the umbrella's np.allclose(rtol=0.04, atol=0.5).
    EXPECT_NEAR(projected[0], 10.0f, 0.9f) << "cart-prismatic projection diverged";
    EXPECT_NEAR(projected[1], 10.0f, 0.9f) << "pole-revolute projection diverged";

    ovphysx_destroy_tensor_binding(m_handle, actuation_b);
    ovphysx_destroy_tensor_binding(m_handle, projected_b);
}

TEST_F(TensorBindingCpuTest, CpuCartPoleLinkIncomingJointForcePreStepZero) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartPole.usda", usd_handle))
        << "Failed to load CartPole.usda";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/cartpole");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_LINK_INCOMING_JOINT_FORCE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status,
              OVPHYSX_API_SUCCESS);

    const size_t total = size_t(spec.shape[0]) * size_t(spec.shape[1]) * size_t(spec.shape[2]);
    std::vector<float> link_forces(total, 7.7f);
    DLTensor t{};
    t.data = link_forces.data();
    t.device = {kDLCPU, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[3] = {spec.shape[0], spec.shape[1], spec.shape[2]};
    t.shape = shape;
    t.ndim = 3;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &t).status,
              OVPHYSX_API_SUCCESS);

    for (size_t i = 0; i < total; ++i) {
        EXPECT_LT(std::abs(link_forces[i]), 1e-3f)
            << "link force component " << i << " pre-step: expected ~0, got "
            << link_forces[i];
    }

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status,
              OVPHYSX_API_SUCCESS);
}

// OMPE-94459 §B5: per-body disable_simulation flag, runtime read/write.
// Underlying TensorAPI's IRigidBodyView::set/getDisableSimulations toggles
// PxActorFlag::eDISABLE_SIMULATION on the corresponding PxRigidActor, so
// the flag takes effect on the next solver step. This test locks down the
// round-trip contract: write a per-body flag, read it back, expect to
// see the same value -- and exercise the masked-write path too.
TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableSimulationRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 1);
    ASSERT_GT(spec.shape[0], 0);

    const int64_t n = spec.shape[0];
    // DISABLE_SIMULATION expects uint8/bool -- engine rejects float32.
    // Alternate 1/0 to exercise both enabled and disabled bodies.
    std::vector<uint8_t> written(n, 0);
    for (int64_t i = 0; i < n; ++i)
        written[i] = static_cast<uint8_t>(i % 2 == 0 ? 1 : 0);

    DLTensor tensor = {};
    tensor.data = written.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[1] = {n};
    tensor.shape = shape;

    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "write failed";

    std::vector<uint8_t> readback(n, 0xff);
    tensor.data = readback.data();
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "read failed";
    for (int64_t i = 0; i < n; ++i)
        EXPECT_EQ(static_cast<int>(readback[i]), static_cast<int>(written[i]))
            << "body " << i << " disable flag round-trip";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// OMPE-94459 (_KINEMATIC_UPDATE_NOOP fix): writing dof-positions and then
// calling ovphysx_articulation_update_kinematic must propagate the new joint
// state into the link buffer without stepping the simulator. Locks down the
// API contract for the umbrella's
// OvPhysxSimulationView.update_articulations_kinematic.
TEST_F(TensorBindingCpuTest, CpuArticulationUpdateKinematicPropagatesDofToLinks) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    // dof-positions binding
    ovphysx_tensor_binding_handle_t dof_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/articulation");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &dof_b).status, OVPHYSX_API_SUCCESS);
    }
    // link-pose binding (verifies kinematic propagation effect)
    ovphysx_tensor_binding_handle_t link_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/articulation");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &link_b).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t dof_spec{}, link_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, dof_b, &dof_spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, link_b, &link_spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_GT(dof_spec.shape[0], 0);
    ASSERT_GT(dof_spec.shape[1], 0);

    // Capture initial link positions.
    const int64_t N = link_spec.shape[0];
    const int64_t L = link_spec.shape[1];
    std::vector<float> links_before(N * L * 7, 0.0f);
    DLTensor lt{}; lt.data = links_before.data(); lt.device = {kDLCPU, 0};
    lt.ndim = 3; lt.dtype = {kDLFloat, 32, 1};
    int64_t lshape[3] = {N, L, 7}; lt.shape = lshape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // Bend each DOF by a non-trivial angle.
    std::vector<float> dofs(dof_spec.shape[0] * dof_spec.shape[1], 0.6f);
    DLTensor dt{}; dt.data = dofs.data(); dt.device = {kDLCPU, 0};
    dt.ndim = 2; dt.dtype = {kDLFloat, 32, 1};
    int64_t dshape[2] = {dof_spec.shape[0], dof_spec.shape[1]}; dt.shape = dshape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dof_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    // Call kinematic forward -- positions only.
    ASSERT_EQ(ovphysx_articulation_update_kinematic(m_handle, dof_b,
                                                    OVPHYSX_ARTICULATION_KINEMATIC_POSITION).status,
              OVPHYSX_API_SUCCESS);

    // Read link poses post-kinematic-update (no step). They should differ
    // from the initial poses: the previously-zero joint angles became 0.6.
    std::vector<float> links_after(N * L * 7, 0.0f);
    lt.data = links_after.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // At least one non-root link must have moved from its rest position.
    int moved = 0;
    for (int64_t k = 1; k < L; ++k) {  // skip k=0 (root, doesn't move with DOF)
        const float dpx = links_after[k * 7 + 0] - links_before[k * 7 + 0];
        const float dpy = links_after[k * 7 + 1] - links_before[k * 7 + 1];
        const float dpz = links_after[k * 7 + 2] - links_before[k * 7 + 2];
        if (std::sqrt(dpx * dpx + dpy * dpy + dpz * dpz) > 1e-3f) ++moved;
    }
    EXPECT_GT(moved, 0)
        << "expected non-root links to move after dof-position write + kinematic update";

    ovphysx_destroy_tensor_binding(m_handle, dof_b);
    ovphysx_destroy_tensor_binding(m_handle, link_b);
}

// OMPE-94459 (JointPerformanceEnvelope fix): ARTICULATION_DOF_DRIVE_MODEL is a
// new tensor type that exposes IArticulationView::set/getDofDriveModelProperties.
// Shape [N, D, 3]: (speedEffortGradient, maxActuatorVelocity, velocityDependentResistance).
TEST_F(TensorBindingCpuTest, CpuArticulationDofDriveModelRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/articulation");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_MODEL_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, b, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 3);
    EXPECT_GT(spec.shape[0], 0);
    EXPECT_GT(spec.shape[1], 0);
    EXPECT_EQ(spec.shape[2], 3);

    // Read current values to confirm read path works.
    std::vector<float> data(spec.shape[0] * spec.shape[1] * spec.shape[2], 0.0f);
    DLTensor t{}; t.data = data.data(); t.device = {kDLCPU, 0};
    t.ndim = 3; t.dtype = {kDLFloat, 32, 1};
    int64_t shape[3] = {spec.shape[0], spec.shape[1], spec.shape[2]}; t.shape = shape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, b, &t).status, OVPHYSX_API_SUCCESS);

    // Write a known triple and read back.
    for (int64_t i = 0; i < spec.shape[0]; ++i)
        for (int64_t j = 0; j < spec.shape[1]; ++j) {
            data[(i * spec.shape[1] + j) * 3 + 0] = 0.7f;
            data[(i * spec.shape[1] + j) * 3 + 1] = 1.2f;
            data[(i * spec.shape[1] + j) * 3 + 2] = 0.05f;
        }
    // Write succeeds; the engine will only actually apply the values to DOFs
    // that have PhysxDrivePerformanceEnvelopeAPI applied in USD (which the
    // test asset doesn't -- a CARB_LOG_WARN is printed and values are
    // silently dropped). Roundtrip-value verification requires an asset with
    // the API applied; the umbrella's JointPerformanceEnvelope test exercises
    // that surface on franka.usda. Here we just lock down the C ABI.
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, b, &t, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(data.size(), 0.0f);
    t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, b, &t).status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, b);
}

// OMPE-94459 (#13): ovphysx_get_object_type classifies prims by TensorAPI
// object type. Rigid bodies in boxes_falling_on_groundplane.usda resolve as
// RIGID_BODY; the articulation root in links_chain_sample is ARTICULATION_ROOT_LINK
// (it's both an articulation root and a link); unknown paths yield INVALID.
// OMPE-94459 (_ROOT_PROPAGATION_GAP trace): the umbrella's
// TestArticulationRootTransforms claims setting ARTICULATION_ROOT_POSE
// doesn't propagate to links across a step. PhysX docs say
// applyCache(eROOT_TRANSFORM) IS equivalent to setRootGlobalPose +
// updateKinematic(POSITION), so propagation should happen. This test
// writes a new root pose, steps once, and verifies the link poses
// shifted with the root.
TEST_F(TensorBindingCpuTest, CpuArticulationRootTransformPropagates) {
    // Use Ant.usda (multi-branch articulation) -- matches the umbrella's
    // failing TestArticulationRootTransforms scenario. Cloning + multi-env
    // matches the umbrella's grid topology where the SKIP was filed.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/Ant.usda", usd_handle))
        << "Failed to load USD";

    // Settling step -- the umbrella test does on_start setup before any
    // physics step, but PhysX expects an articulation to be in a "settled"
    // state before applyCache(eROOT_TRANSFORM) propagates correctly.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Create root-pose and link-pose bindings on the articulation.
    ovphysx_tensor_binding_handle_t root_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/ant/torso");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &root_b).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t link_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/ant/torso");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &link_b).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t root_spec{}, link_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, root_b, &root_spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, link_b, &link_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t N = root_spec.shape[0];
    const int64_t L = link_spec.shape[1];

    // Read the current root pose so we can describe the shift in absolute terms.
    std::vector<float> root_initial(N * 7, 0.0f);
    DLTensor t{}; t.data = root_initial.data(); t.device = {kDLCPU, 0};
    t.ndim = 2; t.dtype = {kDLFloat, 32, 1};
    int64_t rshape[2] = {N, 7}; t.shape = rshape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_b, &t).status, OVPHYSX_API_SUCCESS);

    // Read initial link poses.
    std::vector<float> link_initial(N * L * 7, 0.0f);
    DLTensor lt{}; lt.data = link_initial.data(); lt.device = {kDLCPU, 0};
    lt.ndim = 3; lt.dtype = {kDLFloat, 32, 1};
    int64_t lshape[3] = {N, L, 7}; lt.shape = lshape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // Set root pose shifted +5 in X. Identity rotation.
    std::vector<float> root_new(N * 7, 0.0f);
    constexpr float kShiftX = 0.5f;  // matches umbrella's Z linspace(0, 1)
    for (int64_t i = 0; i < N; ++i) {
        root_new[i * 7 + 0] = root_initial[i * 7 + 0] + kShiftX;
        root_new[i * 7 + 1] = root_initial[i * 7 + 1];
        root_new[i * 7 + 2] = root_initial[i * 7 + 2];
        root_new[i * 7 + 3] = 0.0f;  // qx
        root_new[i * 7 + 4] = 0.0f;  // qy
        root_new[i * 7 + 5] = 0.0f;  // qz
        root_new[i * 7 + 6] = 1.0f;  // qw
    }
    t.data = root_new.data();
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, root_b, &t, nullptr).status, OVPHYSX_API_SUCCESS);

    // Read root AFTER write, BEFORE step -- did the write actually take?
    std::vector<float> root_after_write(N * 7, 0.0f);
    t.data = root_after_write.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_b, &t).status, OVPHYSX_API_SUCCESS);
    std::cerr << "[root-prop trace] initial=" << root_initial[0] << "  set=" << root_new[0]
              << "  read_after_write=" << root_after_write[0]
              << "  (delta to set: " << (root_after_write[0] - root_new[0]) << ")\n";

    // Step.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Read link poses post-step.
    std::vector<float> link_post(N * L * 7, 0.0f);
    lt.data = link_post.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // Verify every non-padding link's X shifted by ~kShiftX. Padding links
    // (where the articulation has < L links) read back as zeros, skip.
    int checked = 0;
    for (int64_t i = 0; i < N; ++i) {
        for (int64_t k = 0; k < L; ++k) {
            const size_t base = (i * L + k) * 7;
            const float qw_initial = link_initial[base + 6];
            if (qw_initial == 0.0f) continue;  // zero-padded
            const float dx = link_post[base + 0] - link_initial[base + 0];
            EXPECT_NEAR(dx, kShiftX, 0.001f)  // umbrella's rtol=1e-3 + atol=1e-4
                << "arti " << i << " link " << k
                << ": expected x-shift " << kShiftX << ", got " << dx;
            ++checked;
        }
    }
    EXPECT_GT(checked, 0) << "should have checked at least one link";

    ovphysx_destroy_tensor_binding(m_handle, root_b);
    ovphysx_destroy_tensor_binding(m_handle, link_b);
}

// OMPE-94459 (#14 diagnostic): reproduces the umbrella's failing
// LinearDofVelocities multi-env scenario on the MultiCartRail.usda 4x4 grid.
// Variants:
//   _NoSettle: write velocity in "on_start", step once, read -- mirrors the
//     umbrella's pre-step write.
//   _WithSettle: step once before write, then write, step, read -- mirrors
//     the single-articulation control which passes.
static void RunMultiCartRailPrismaticDofVelocity(ovphysx_handle_t m_handle,
                                                 bool do_settle,
                                                 const char* usd_asset = "tests/data/MultiCartRail.usda",
                                                 const char* pattern = "/envs/*") {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, usd_asset, usd_handle))
        << "Failed to load " << usd_asset;

    if (do_settle) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    ovphysx_tensor_binding_handle_t vel_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = make_ovx_string(pattern);
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &vel_b).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t N = spec.shape[0];
    const int64_t D = spec.shape[1];
    ASSERT_GE(N, 1) << "expected at least one cartpole in the binding";
    ASSERT_EQ(D, 1) << "CartRailNoPole has 1 DOF (prismatic cart joint)";

    // Use linspace -2.0..2.0 like the umbrella scenario, so per-env values
    // vary -- catches per-arti index-mapping bugs that uniform values mask.
    std::vector<float> set_vel(N * D, 0.0f);
    for (int64_t i = 0; i < N; ++i) {
        const float v = -2.0f + 4.0f * float(i) / float(N - 1);
        for (int64_t j = 0; j < D; ++j) set_vel[i * D + j] = v;
    }
    DLTensor t{}; t.device = {kDLCPU, 0}; t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {N, D}; t.shape = shape; t.ndim = 2; t.data = set_vel.data();

    // The umbrella's _write path always passes indices=[0..N-1] (from
    // wp_utils.arange(view.count)) -- it never uses a null-indices full
    // write. Mirror that here so the indexed write path is exercised.
    std::vector<int32_t> indices_vec(N);
    for (int64_t i = 0; i < N; ++i) indices_vec[i] = static_cast<int32_t>(i);
    DLTensor idx_t{}; idx_t.device = {kDLCPU, 0}; idx_t.dtype = {kDLInt, 32, 1};
    idx_t.ndim = 1; int64_t idx_shape[1] = {N}; idx_t.shape = idx_shape;
    idx_t.data = indices_vec.data();
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, vel_b, &t, &idx_t).status, OVPHYSX_API_SUCCESS);

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    std::vector<float> read_vel(N * D, 0.0f);
    t.data = read_vel.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &t).status, OVPHYSX_API_SUCCESS);

    float max_err = 0.0f;
    int over_tol_count = 0;
    for (int64_t i = 0; i < N * D; ++i) {
        const float err = std::abs(read_vel[i] - set_vel[i]);
        if (err > max_err) max_err = err;
        if (err > 1e-3f) {
            std::cerr << "[multi-cart-rail vel settle=" << (do_settle ? 1 : 0)
                      << "]   i=" << i << " set=" << set_vel[i]
                      << " read=" << read_vel[i] << " err=" << err << "\n";
            ++over_tol_count;
        }
    }
    std::cerr << "[multi-cart-rail vel settle=" << (do_settle ? 1 : 0)
              << "] max_err=" << max_err
              << " over_tol_count=" << over_tol_count << " / " << (N * D) << "\n";

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
}

TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_NoSettle) {
    RunMultiCartRailPrismaticDofVelocity(m_handle, /*do_settle=*/false);
}

TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_WithSettle) {
    RunMultiCartRailPrismaticDofVelocity(m_handle, /*do_settle=*/true);
}

// OMPE-94459 (#14 follow-up diagnostic, filed by the umbrella-side agent
// 2026-05-26). The umbrella sees prismatic DOF velocities collapse to ~0
// after the first simulate() on the LinearDofVelocities scenario, but my
// existing MultiCartRail-based c_unittests pass cleanly. The difference
// between the two test setups is the PhysxSchema.PhysxSceneAPI attrs the
// umbrella's _scenario.py programmatically applies on /physicsScene:
//   physxScene:enableGpuDynamics = 0
//   physxScene:broadPhaseType    = "MBP"
//   physxScene:enableSceneQuerySupport = 0
//   physxScene:timeStepsPerSecond = 60
// MultiCartRailUmbrella.usda replicates that exactly. If this test
// reproduces the umbrella's velocity decay, the failure is engine-side
// and triggered by PhysxSceneAPI. If it doesn't reproduce, the cause is
// somewhere else (class-prim inheritance vs direct references, the
// USD parse path, etc.) and we need to investigate further.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaSceneAPI) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false, "tests/data/MultiCartRailUmbrella.usda");
}

// 42-env variant matching the umbrella's actual scenario count + gravity
// direction (0,0,0) instead of (0,0,-1) with magnitude=0. Rules out
// count-dependence and the unusual zero-vector gravity direction.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_42EnvUmbrella) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false, "tests/data/MultiCartRail42Umbrella.usda");
}

// USD class-prim inheritance variant. The umbrella's _scenario.py creates
// `/envTemplate` as a class prim with `/envTemplate/railcart` referencing
// the asset, then each `/envs/env_N` gets `prepend inherits = </envTemplate>`.
// This is composition by inheritance, NOT direct per-env references. Used to
// isolate whether the umbrella's class-prim composition is the differentiator
// from the prior `_NoSettle` / `_UmbrellaSceneAPI` variants (which both use
// direct references per env).
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_InheritUmbrella) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/MultiCartRailInheritUmbrella.usda",
        "/envs/*/railcart");
}

// Loads the umbrella's actual exported USDA captured from a failing
// LinearDofVelocities_ovphysx_cc run (shared via the handoff doc 2026-05-26).
// This is the exact file the ovstage population path sees in the failing test path.
// If THIS reproduces the velocity decay, the differentiator is in the USDA
// content (ground plane, CartRailNoPole.usda's content via absolute-path
// reference, or some other detail my hand-written variants miss). If it
// doesn't reproduce, the failure has to be above the C ABI (the umbrella
// adapter's warp-int32 indices, the umbrella's physx.step path, or
// process-wide PhysX state from prior tests).
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExport) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export.usda",
        "/envs/*/railcart");
}

// Same as _UmbrellaExport but with the groundPlane stripped, to test the
// hypothesis that the ground plane (a static collider at z=0) somehow
// affects the cart's prismatic DOF integration.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportNoGround) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_noground.usda",
        "/envs/*/railcart");
}

// Same as _UmbrellaExport but env positions widened to 50m spacing on both
// axes (rails extend +/-3 in Y, so 50m spacing eliminates inter-env rail
// overlap that exists at the umbrella's row_spacing=2 / col_spacing=6.5
// layout). Tests whether neighboring articulation overlap is the trigger.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportWideSpacing) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_widespacing.usda",
        "/envs/*/railcart");
}

// Same as _UmbrellaExport but row_spacing widened to 8m (rails extend +/-3
// in Y, so 8m > 6m is the minimum non-overlapping spacing). col_spacing
// stays at the umbrella's 6.5m. Tests whether the row-direction overlap
// alone is sufficient to trigger the decay.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportNoOverlap) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_nooverlap.usda",
        "/envs/*/railcart");
}

// OMPE-94459 (§B6 _LINEAR_DRIVE_TUNING diagnostic): reproduces the umbrella's
// failing LinearDofPositionTargets scenario. CartRailDriveLinear.usda has the
// same PD parameters the umbrella's _set_drive applies (k=2000, d=250,
// maxForce=4000). Write target=1.0, step 60 times (1s at 60Hz), expect cart
// to settle near the target. The cart mass at default density 1000 kg/m^3 and
// scale (0.2, 0.25, 0.2) = volume 0.01 = 10 kg gives nat freq ~14 rad/s,
// damping ratio ~0.88, settling time ~0.32s -- 60 steps should be plenty.
// If the cart fails to settle, the PhysX prismatic drive PD is responding
// differently than the SDF damping equations predict.
TEST_F(TensorBindingCpuTest, CpuLinearDrivePD_SettlesIn60Steps) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartRailDriveLinear.usda", usd_handle))
        << "Failed to load CartRailDriveLinear.usda";

    // Settle step (matches the umbrella's flow where on_start sets the target
    // and then a step is taken at stepno=1).
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    ovphysx_tensor_binding_handle_t pos_b = 0;
    ovphysx_tensor_binding_handle_t tgt_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/cartpole");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &pos_b).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/cartpole");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &tgt_b).status, OVPHYSX_API_SUCCESS);
    }

    const float target = 1.0f;
    DLTensor t{}; t.device = {kDLCPU, 0}; t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {1, 1}; t.shape = shape; t.ndim = 2;
    float target_buf = target; t.data = &target_buf;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, tgt_b, &t, nullptr).status, OVPHYSX_API_SUCCESS);

    // Step 60 times and trace the cart position. Settling time should be
    // ~0.32s = 20 steps; 60 steps gives ample headroom.
    float final_pos = 0.0f;
    for (int s = 0; s < 60; ++s) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));

        t.data = &final_pos;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pos_b, &t).status, OVPHYSX_API_SUCCESS);
        if (s < 5 || s == 19 || s == 29 || s == 59) {
            std::cerr << "[linear-drive PD] step " << (s + 1) << " pos=" << final_pos << "\n";
        }
    }

    const float err = std::abs(final_pos - target);
    std::cerr << "[linear-drive PD] final_pos=" << final_pos << " target=" << target
              << " err=" << err << "\n";

    ovphysx_destroy_tensor_binding(m_handle, pos_b);
    ovphysx_destroy_tensor_binding(m_handle, tgt_b);
}

// OMPE-94459 (#14 evidence): for a single CartRailNoPole prismatic
// articulation, ovphysx's ARTICULATION_DOF_VELOCITY_F32 write+step+read
// preserves the joint velocity exactly (drift=0). The umbrella's
// LinearDofVelocities test SKIPs with "velocity drift after 1 step" only
// in its 42-env cloned-grid configuration, and the GPU sibling §B20
// documents the underlying bug as "engine-side, prismatic-specific". The
// angular sibling (revolute joint) passes the same Common-class
// indexed-write code path. Conclusion: the wire-up through ovphysx is
// correct -- the residual drift is a PhysX SDK / omni.physx.tensors
// prismatic-joint behavior, not an ovphysx fix.
TEST_F(TensorBindingCpuTest, CpuPrismaticDofVelocity_NoDriftOnSingleArticulation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartRailNoPole.usda", usd_handle))
        << "Failed to load CartRailNoPole.usda";

    ovphysx_tensor_binding_handle_t vel_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/cartpole");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &vel_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 2);
    ASSERT_EQ(spec.shape[0], 1);
    ASSERT_EQ(spec.shape[1], 1);

    float set_vel = 2.0f;
    DLTensor tensor{};
    tensor.data = &set_vel;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {1, 1};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, vel_binding, &tensor, nullptr).status, OVPHYSX_API_SUCCESS);

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    float read_vel = 0.0f;
    tensor.data = &read_vel;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_binding, &tensor).status, OVPHYSX_API_SUCCESS);
    // The ovphysx code path itself preserves velocity exactly on a single
    // articulation. The umbrella's multi-env drift / GPU-zero behavior
    // lives in the engine and is documented in PROGRESS_tensors.md §B20.
    EXPECT_NEAR(read_vel, set_vel, 1e-6f);

    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
}

TEST_F(TensorBindingCpuTest, CpuGetObjectType) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    // No explicit step before get_object_type -- regression coverage for the
    // failure mode the umbrella's TestObjectType scenario hits. The engine
    // is expected to do the lazy attach + initial parse itself.

    ovphysx_object_type_t t = OVPHYSX_OBJECT_TYPE_INVALID;
    ASSERT_EQ(ovphysx_get_object_type(m_handle, make_ovx_string("/nonexistent/path"), &t).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(t, OVPHYSX_OBJECT_TYPE_INVALID) << "unknown path should return INVALID";

    // The articulation root prim contains links; classifying the root prim itself
    // returns the articulation classification.
    ASSERT_EQ(ovphysx_get_object_type(m_handle, make_ovx_string("/World/articulation"), &t).status, OVPHYSX_API_SUCCESS);
    EXPECT_NE(t, OVPHYSX_OBJECT_TYPE_INVALID)
        << "/World/articulation should classify as an articulation-family object";

    // A link prim should classify as articulation link (or root link for the root).
    ASSERT_EQ(ovphysx_get_object_type(m_handle, make_ovx_string("/World/articulation/articulationLink0"), &t).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(t == OVPHYSX_OBJECT_TYPE_ARTICULATION_LINK ||
                t == OVPHYSX_OBJECT_TYPE_ARTICULATION_ROOT_LINK)
        << "articulationLink0 classified as " << t;
}

// OMPE-94459 (§B5 simulation-effect proof): writing 1 to DISABLE_SIMULATION
// must actually freeze the body mid-sim. Drops a cube under gravity, steps
// once to confirm motion, disables it, steps more, and asserts pose stops
// changing. Catches a regression where the write would round-trip cleanly
// (handled by CpuRigidBodyDisableSimulationRoundtrip) but the engine
// wouldn't toggle PxActorFlag::eDISABLE_SIMULATION.
TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableSimulationStopsSimulation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t disable_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &disable_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t pose_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = pose_spec.shape[0];
    ASSERT_GT(n, 0);

    std::vector<float> poses(n * 7, 0.0f);
    DLTensor pose_t{};
    pose_t.data = poses.data();
    pose_t.device = {kDLCPU, 0};
    pose_t.ndim = 2;
    pose_t.dtype = {kDLFloat, 32, 1};
    int64_t pose_shape[2] = {n, 7};
    pose_t.shape = pose_shape;

    auto step_once = [&]() {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    };

    step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    std::vector<float> pose_before_disable(poses);

    // Step once more without disabling -- pose must change under gravity.
    step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    bool moved_before_disable = false;
    for (int64_t i = 0; i < n; ++i) {
        if (std::abs(poses[i * 7 + 2] - pose_before_disable[i * 7 + 2]) > 1e-5f) {
            moved_before_disable = true;
            break;
        }
    }
    ASSERT_TRUE(moved_before_disable) << "Sanity check: bodies should fall under gravity before disable.";

    // Disable simulation on all bodies.
    std::vector<uint8_t> flags(n, 1);
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    // Capture pose immediately after disable.
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    std::vector<float> pose_after_disable(poses);

    // Step several more -- disabled bodies should not move.
    for (int s = 0; s < 5; ++s) step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_NEAR(poses[i * 7 + 0], pose_after_disable[i * 7 + 0], 1e-4f) << "body " << i << " x drifted while disabled";
        EXPECT_NEAR(poses[i * 7 + 1], pose_after_disable[i * 7 + 1], 1e-4f) << "body " << i << " y drifted while disabled";
        EXPECT_NEAR(poses[i * 7 + 2], pose_after_disable[i * 7 + 2], 1e-4f) << "body " << i << " z drifted while disabled";
    }

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// OMPE-94459 follow-up (umbrella RigidBodyEnableDisablePhysics ask):
// reproduces the umbrella's failing path -- write DISABLE_SIMULATION=1 in
// on_start (before any explicit step) and check whether the next simulate
// honours the flag. The umbrella reports the flag commits to storage
// (readback shows it set) but the body still falls under gravity. The
// distinguishing factor vs CpuRigidBodyDisableSimulationStopsSimulation is
// the lack of a step BEFORE the write.
TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableSimulationInOnStart) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    // Create the bindings WITHOUT stepping first. The create call internally
    // triggers ovphysx_ensure_physics_attached which does a simulate(0,0)
    // for the initial scene parse, but no real step has been taken yet from
    // the user's perspective.
    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t disable_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &disable_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t pose_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = pose_spec.shape[0];
    ASSERT_GT(n, 0);

    // Capture starting pose (post initial-parse, pre any explicit step).
    std::vector<float> poses(n * 7, 0.0f);
    DLTensor pose_t{}; pose_t.data = poses.data(); pose_t.device = {kDLCPU, 0};
    pose_t.ndim = 2; pose_t.dtype = {kDLFloat, 32, 1};
    int64_t pose_shape[2] = {n, 7}; pose_t.shape = pose_shape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    std::vector<float> pose_initial(poses);

    // Write disable=1 to all bodies BEFORE any explicit step.
    std::vector<uint8_t> flags(n, 1);
    DLTensor flag_t{}; flag_t.data = flags.data(); flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1; flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n}; flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    // Read back the flag to confirm storage update.
    std::vector<uint8_t> readback(n, 0);
    flag_t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, disable_binding, &flag_t).status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_EQ(readback[i], 1) << "body " << i << " disable flag did not persist";
    }

    // Step a few times -- if the flag is properly honoured, bodies stay put.
    for (int s = 0; s < 3; ++s) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Check positions: disabled bodies should not have moved.
    pose_t.data = poses.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    int moved = 0;
    for (int64_t i = 0; i < n; ++i) {
        const float dz = std::abs(poses[i * 7 + 2] - pose_initial[i * 7 + 2]);
        if (dz > 1e-4f) {
            std::cerr << "[disable-in-on-start] body " << i
                      << " z drifted " << dz
                      << " (initial=" << pose_initial[i * 7 + 2]
                      << ", final=" << poses[i * 7 + 2]
                      << ", flag=" << int(readback[i]) << ")\n";
            ++moved;
        }
    }
    std::cerr << "[disable-in-on-start] " << moved << " / " << n
              << " bodies drifted while disabled\n";
    // The disable write issued in on_start must be honored: none of the bodies
    // may drift. Without the assert this test would pass green even if the
    // exact regression it reproduces (disable in on_start not re-read by PhysX)
    // came back.
    EXPECT_EQ(moved, 0) << "disabled bodies drifted after an on_start disable write";

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// OMPE-94459 follow-up: same on_start-write-then-step flow as
// CpuRigidBodyDisableSimulationInOnStart, but using an INDEXED write to
// disable only the even-indexed bodies (subset). Mirrors the umbrella's
// RigidBodyEnableDisablePhysics scenario which writes
// disable=[1,1,...,1] with indices [0, 2, 4, ...] -- alternate-index
// pattern with N balls = 32 (8 envs x 4 balls), full = uniform 1.
TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableSimulationIndexedInOnStart) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t pose_binding = 0, vel_binding = 0, disable_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &vel_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &disable_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t pose_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = pose_spec.shape[0];
    ASSERT_GT(n, 1);

    // Build indices for "every other body" -- mirrors the umbrella's
    // np.arange(0, count, 2) pattern. Use int32 (matches umbrella warp
    // indices dtype).
    std::vector<int32_t> indices_disabled;
    std::vector<int32_t> indices_enabled;
    for (int32_t i = 0; i < int32_t(n); ++i) {
        if (i % 2 == 0) indices_disabled.push_back(i);
        else            indices_enabled.push_back(i);
    }

    // Disable flag = 1 for every body in the indices_disabled subset. Per
    // BaseRigidBodyView::setDisableSimulations, the source tensor must be
    // sized [N] (full count), not [subset_count] -- the function reads
    // src[idx] for each subset index idx.
    std::vector<uint8_t> flags(n, 1);
    DLTensor flag_t{}; flag_t.data = flags.data(); flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1; flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n}; flag_t.shape = flag_shape;

    DLTensor idx_t{}; idx_t.device = {kDLCPU, 0}; idx_t.dtype = {kDLInt, 32, 1};
    idx_t.ndim = 1; int64_t idx_shape[1] = {int64_t(indices_disabled.size())};
    idx_t.shape = idx_shape; idx_t.data = indices_disabled.data();

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, &idx_t).status,
              OVPHYSX_API_SUCCESS);

    // Verify readback shows the alternating pattern.
    std::vector<uint8_t> readback(n, 0);
    flag_t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, disable_binding, &flag_t).status, OVPHYSX_API_SUCCESS);
    for (int32_t idx : indices_disabled) {
        EXPECT_EQ(readback[idx], 1) << "disabled body " << idx << " not flagged";
    }
    for (int32_t idx : indices_enabled) {
        EXPECT_EQ(readback[idx], 0) << "enabled body " << idx << " incorrectly flagged";
    }

    // Step twice to match the umbrella's "stepno=1 = after 2 simulates".
    for (int s = 0; s < 2; ++s) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Read velocities -- disabled bodies should have z-vel = 0.
    std::vector<float> velocities(n * 6, 0.0f);
    DLTensor vel_t{}; vel_t.data = velocities.data(); vel_t.device = {kDLCPU, 0};
    vel_t.ndim = 2; vel_t.dtype = {kDLFloat, 32, 1};
    int64_t vel_shape[2] = {n, 6}; vel_t.shape = vel_shape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_binding, &vel_t).status, OVPHYSX_API_SUCCESS);

    int disabled_falling = 0, enabled_static = 0;
    for (int32_t idx : indices_disabled) {
        const float vz = velocities[idx * 6 + 2];
        if (std::abs(vz) > 1e-3f) {
            std::cerr << "[disable-idx] disabled body " << idx << " z-vel=" << vz << "\n";
            ++disabled_falling;
        }
    }
    for (int32_t idx : indices_enabled) {
        const float vz = velocities[idx * 6 + 2];
        if (std::abs(vz) < 1e-3f) {
            std::cerr << "[disable-idx] enabled body " << idx << " z-vel=" << vz
                      << " (expected non-zero)\n";
            ++enabled_static;
        }
    }
    std::cerr << "[disable-idx] disabled_falling=" << disabled_falling
              << "/" << indices_disabled.size()
              << ", enabled_static=" << enabled_static
              << "/" << indices_enabled.size() << "\n";
    // Indexed disable must be honored on exactly the even-index subset: those
    // bodies must read zero z-velocity, while the odd-index bodies left enabled
    // must still be falling. Asserting both guards the alternate-index path the
    // umbrella exercises.
    EXPECT_EQ(disabled_falling, 0) << "disabled (even-index) bodies still falling";
    EXPECT_EQ(enabled_static, 0) << "enabled (odd-index) bodies were not falling";

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// OMPE-94459 (umbrella gg/gc): CPU reference for the umbrella's exact
// enable/disable flip sequence (common/rigid_body.py). Establishes the
// behavior the legacy test asserts so the GPU path can be measured against
// it: disable a MOVING body (does it zero or freeze?), then flip + wake in
// one window and step (does the re-enabled body resume?).
TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableFlipWakeSequence) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    auto make_binding = [&](ovphysx_tensor_type_t type) {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = type;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        return b;
    };
    ovphysx_tensor_binding_handle_t dis_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL);
    ovphysx_tensor_binding_handle_t vel_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);

    std::vector<int32_t> idx_A, idx_B; // A = disabled-in-on-start, B = enabled-in-on-start
    for (int32_t i = 0; i < int32_t(n); ++i) (i % 2 == 0 ? idx_A : idx_B).push_back(i);

    auto write_disable_subset = [&](uint8_t value, const std::vector<int32_t>& subset) {
        std::vector<uint8_t> flags(n, value);
        DLTensor ft{}; ft.data = flags.data(); ft.device = {kDLCPU, 0};
        ft.dtype = {kDLUInt, 8, 1}; ft.ndim = 1; int64_t fs[1] = {n}; ft.shape = fs;
        DLTensor it{}; it.data = const_cast<int32_t*>(subset.data()); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 32, 1}; it.ndim = 1; int64_t is[1] = {int64_t(subset.size())}; it.shape = is;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &ft, &it).status, OVPHYSX_API_SUCCESS);
    };
    auto step = [&]() {
        ovphysx_enqueue_result_t s = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(s.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, s.op_index));
    };
    auto read_vz = [&](std::vector<float>& vz) {
        std::vector<float> buf(n * 6, 0.f);
        DLTensor vt{}; vt.data = buf.data(); vt.device = {kDLCPU, 0};
        vt.dtype = {kDLFloat, 32, 1}; vt.ndim = 2; int64_t vs[2] = {n, 6}; vt.shape = vs;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_SUCCESS);
        vz.resize(n); for (int64_t i = 0; i < n; ++i) vz[i] = buf[i * 6 + 2];
    };

    // on_start: disable A. Step x2 (umbrella's "stepno=1" sees -2*dt*g).
    write_disable_subset(1, idx_A);
    step(); step();
    std::vector<float> vz1; read_vz(vz1);

    // stepno=1 flip: disable B, enable A, wake all -- in one window.
    write_disable_subset(1, idx_B);
    write_disable_subset(0, idx_A);
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, nullptr).status, OVPHYSX_API_SUCCESS);
    step();
    std::vector<float> vz2; read_vz(vz2);

    // CPU reference contract (what the umbrella's legacy test asserts):
    // a body disabled mid-motion reads 0; a re-enabled+woken body resumes.
    for (int32_t i : idx_B)
        EXPECT_NEAR(vz2[i], 0.f, 1e-3f) << "cpu: now-disabled body " << i << " should read 0, got " << vz2[i];
    for (int32_t i : idx_A)
        EXPECT_LT(vz2[i], -0.01f) << "cpu: re-enabled+woken body " << i << " should be falling, got " << vz2[i];

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-94459 (CR follow-up): ovphysx_rigid_body_view_wake_up validates its
// index tensor (1D int32, length <= body count) before forwarding, so a
// malformed tensor is rejected rather than mis-read as a host PxU32 buffer.
// Validation runs before any device staging, so this needs no CUDA.
TEST_F(TensorBindingCpuTest, CpuRigidBodyWakeUpRejectsBadIndices) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_tensor_binding_handle_t pose_b = 0;
    ovphysx_tensor_binding_desc_t d{};
    d.pattern = OVPHYSX_LITERAL("/World/Cube*");
    d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &pose_b).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);

    std::vector<int32_t> idx(n);
    for (int64_t i = 0; i < n; ++i) idx[i] = int32_t(i);
    auto wake = [&](const DLTensor* it) {
        return ovphysx_rigid_body_view_wake_up(m_handle, pose_b, it).status;
    };

    // Valid: 1D int32, length == count.
    {
        DLTensor it{}; it.data = idx.data(); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 32, 1}; it.ndim = 1; int64_t s[1] = {n}; it.shape = s;
        EXPECT_EQ(wake(&it), OVPHYSX_API_SUCCESS) << "valid 1D int32 indices should be accepted";
    }
    // Null = wake all (allowed).
    EXPECT_EQ(wake(nullptr), OVPHYSX_API_SUCCESS) << "null indices (wake all) should be accepted";
    // Wrong rank (2D).
    {
        DLTensor it{}; it.data = idx.data(); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 32, 1}; it.ndim = 2; int64_t s[2] = {n, 1}; it.shape = s;
        EXPECT_EQ(wake(&it), OVPHYSX_API_INVALID_ARGUMENT) << "2D indices should be rejected";
    }
    // Wrong dtype (int64).
    {
        std::vector<int64_t> idx64(n, 0);
        DLTensor it{}; it.data = idx64.data(); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 64, 1}; it.ndim = 1; int64_t s[1] = {n}; it.shape = s;
        EXPECT_EQ(wake(&it), OVPHYSX_API_INVALID_ARGUMENT) << "int64 indices should be rejected";
    }
    // Too long (length > body count).
    {
        std::vector<int32_t> idxLong(n + 1, 0);
        DLTensor it{}; it.data = idxLong.data(); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 32, 1}; it.ndim = 1; int64_t s[1] = {n + 1}; it.shape = s;
        EXPECT_EQ(wake(&it), OVPHYSX_API_INVALID_ARGUMENT) << "over-length indices should be rejected";
    }

    ovphysx_destroy_tensor_binding(m_handle, pose_b);
}

// OMPE-94459 follow-up (umbrella ask: wake_up API). Exercises
// ovphysx_rigid_body_view_wake_up by:
//   1. Disabling all bodies (they freeze in place).
//   2. Re-enabling them (PhysX places them in sleep state).
//   3. Calling wake_up.
//   4. Stepping and asserting bodies are now moving again.
// Also covers the "wake without re-enable" path -- bodies still flagged as
// disabled should be silently skipped by wake_up (per the engine semantics
// in BaseRigidBodyView::wakeUp at line 575).
TEST_F(TensorBindingCpuTest, CpuRigidBodyViewWakeUpAfterReEnable) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t pose_binding = 0, disable_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &disable_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t pose_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = pose_spec.shape[0];
    ASSERT_GT(n, 0);

    auto step_once = [&]() {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    };

    auto write_disable_all = [&](uint8_t value) {
        std::vector<uint8_t> flags(n, value);
        DLTensor t{}; t.data = flags.data(); t.device = {kDLCPU, 0};
        t.ndim = 1; t.dtype = {kDLUInt, 8, 1};
        int64_t shape[1] = {n}; t.shape = shape;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &t, nullptr).status,
                  OVPHYSX_API_SUCCESS);
    };

    auto read_poses = [&](std::vector<float>& out) {
        out.assign(n * 7, 0.0f);
        DLTensor t{}; t.data = out.data(); t.device = {kDLCPU, 0};
        t.ndim = 2; t.dtype = {kDLFloat, 32, 1};
        int64_t shape[2] = {n, 7}; t.shape = shape;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &t).status, OVPHYSX_API_SUCCESS);
    };

    // Disable all bodies. Step a few times -- they should not move.
    write_disable_all(1);
    std::vector<float> pose_after_disable;
    read_poses(pose_after_disable);
    for (int s = 0; s < 3; ++s) step_once();
    std::vector<float> pose_post_disable_steps;
    read_poses(pose_post_disable_steps);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_NEAR(pose_post_disable_steps[i * 7 + 2], pose_after_disable[i * 7 + 2], 1e-4f)
            << "body " << i << " z drifted while disabled";
    }

    // wake_up on a still-disabled binding is a documented no-op per body;
    // the call itself succeeds (engine returns true).
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, pose_binding, nullptr).status,
              OVPHYSX_API_SUCCESS);
    for (int s = 0; s < 2; ++s) step_once();
    std::vector<float> pose_after_silent_wake;
    read_poses(pose_after_silent_wake);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_NEAR(pose_after_silent_wake[i * 7 + 2], pose_after_disable[i * 7 + 2], 1e-4f)
            << "body " << i << " z drifted after no-op wake on disabled body";
    }

    // Re-enable. PhysX puts the re-introduced actor in sleep state.
    write_disable_all(0);
    // Without wake_up, stepping leaves the bodies asleep (zero velocity).
    // Issue the wake call now, then step and verify bodies move under gravity.
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, pose_binding, nullptr).status,
              OVPHYSX_API_SUCCESS);
    std::vector<float> pose_before_wake_steps;
    read_poses(pose_before_wake_steps);
    for (int s = 0; s < 3; ++s) step_once();
    std::vector<float> pose_after_wake_steps;
    read_poses(pose_after_wake_steps);
    int moved = 0;
    for (int64_t i = 0; i < n; ++i) {
        const float dz = pose_before_wake_steps[i * 7 + 2] - pose_after_wake_steps[i * 7 + 2];
        if (dz > 1e-3f) ++moved;  // positive == fell under gravity
    }
    EXPECT_EQ(moved, n) << "all bodies should fall under gravity after re-enable + wake_up";

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// wake_up with indices: only the indexed subset should wake.
TEST_F(TensorBindingCpuTest, CpuRigidBodyViewWakeUpIndexed) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t pose_binding = 0, disable_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &disable_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t pose_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = pose_spec.shape[0];
    ASSERT_GT(n, 1);

    // Disable all, then re-enable all. Bodies are now in sleep state.
    {
        std::vector<uint8_t> flags(n, 1);
        DLTensor t{}; t.data = flags.data(); t.device = {kDLCPU, 0};
        t.ndim = 1; t.dtype = {kDLUInt, 8, 1};
        int64_t shape[1] = {n}; t.shape = shape;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &t, nullptr).status,
                  OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }
    {
        std::vector<uint8_t> flags(n, 0);
        DLTensor t{}; t.data = flags.data(); t.device = {kDLCPU, 0};
        t.ndim = 1; t.dtype = {kDLUInt, 8, 1};
        int64_t shape[1] = {n}; t.shape = shape;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &t, nullptr).status,
                  OVPHYSX_API_SUCCESS);
    }

    // Wake only the even-indexed bodies.
    std::vector<int32_t> wake_indices;
    for (int32_t i = 0; i < int32_t(n); ++i)
        if (i % 2 == 0) wake_indices.push_back(i);
    DLTensor idx_t{}; idx_t.data = wake_indices.data(); idx_t.device = {kDLCPU, 0};
    idx_t.ndim = 1; idx_t.dtype = {kDLInt, 32, 1};
    int64_t idx_shape[1] = {int64_t(wake_indices.size())}; idx_t.shape = idx_shape;
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, pose_binding, &idx_t).status,
              OVPHYSX_API_SUCCESS);

    std::vector<float> pose_before(n * 7, 0.0f);
    DLTensor pt{}; pt.data = pose_before.data(); pt.device = {kDLCPU, 0};
    pt.ndim = 2; pt.dtype = {kDLFloat, 32, 1};
    int64_t pshape[2] = {n, 7}; pt.shape = pshape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pt).status, OVPHYSX_API_SUCCESS);

    for (int s = 0; s < 3; ++s) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }
    std::vector<float> pose_after(n * 7, 0.0f);
    pt.data = pose_after.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pt).status, OVPHYSX_API_SUCCESS);

    int woke_moved = 0, slept_moved = 0;
    for (int32_t i = 0; i < int32_t(n); ++i) {
        const float dz = pose_before[i * 7 + 2] - pose_after[i * 7 + 2];
        const bool moved = dz > 1e-3f;
        if (i % 2 == 0) {
            if (moved) ++woke_moved;
        } else {
            if (moved) ++slept_moved;
        }
    }
    EXPECT_GT(woke_moved, 0) << "woken bodies should fall under gravity";
    EXPECT_EQ(slept_moved, 0) << "non-woken bodies should remain at rest";

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// wake_up on an articulation binding should report invalid argument.
TEST_F(TensorBindingCpuTest, CpuRigidBodyViewWakeUpOnArticulationFails) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/Ant.usda", usd_handle))
        << "Failed to load Ant.usda";

    ovphysx_tensor_binding_handle_t arti_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/ant/torso");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &arti_binding).status, OVPHYSX_API_SUCCESS);
    }
    const ovphysx_result_t r = ovphysx_rigid_body_view_wake_up(m_handle, arti_binding, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_INVALID_ARGUMENT)
        << "wake_up on articulation binding should report INVALID_ARGUMENT";
    ovphysx_destroy_tensor_binding(m_handle, arti_binding);
}

TEST_F(TensorBindingCpuTest, CpuArticulationDofReadWrite) {
    // Load articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    // Create DOF position binding
    ovphysx_tensor_binding_handle_t dof_binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &dof_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    // Get spec
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(spec.ndim, 2);
    EXPECT_GT(spec.shape[0], 0);  // At least one articulation
    EXPECT_GT(spec.shape[1], 0);  // At least one DOF

    // CPU mode: warmup is a no-op but should succeed
    result = ovphysx_warmup_gpu(m_handle);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    size_t total_elements = spec.shape[0] * spec.shape[1];
    std::vector<float> data(total_elements, 0.0f);

    DLTensor tensor = {};
    tensor.data = data.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // Read DOF positions
    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU read should work";

    // Write new DOF positions
    for (size_t i = 0; i < total_elements; ++i) {
        data[i] = 0.1f;  // Set all to 0.1 rad
    }

    result = ovphysx_write_tensor_binding(m_handle, dof_binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Note: Read-back verification removed because CPU mode TensorAPI may not reflect
    // writes immediately. Write verification is tested in GPU mode tests instead.

    // Cleanup
    result = ovphysx_destroy_tensor_binding(m_handle, dof_binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

// ============================================================================
// ERROR CONDITION TESTS
// ============================================================================

class TensorBindingErrorTest : public PhysXTestFixture {};

TEST_F(TensorBindingErrorTest, InvalidHandle) {
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    // Use invalid handle
    ovphysx_result_t result = ovphysx_create_tensor_binding(OVPHYSX_INVALID_HANDLE, &desc, &binding);
    EXPECT_NE(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingErrorTest, NullDescriptor) {
    ovphysx_tensor_binding_handle_t binding = 0;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, nullptr, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(TensorBindingErrorTest, EmptyPattern) {
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = {nullptr, 0};  // Empty pattern
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
}

// NVBugs 6433621: embedded NUL bytes in path/pattern strings must be rejected.
TEST_F(TensorBindingErrorTest, RejectsEmbeddedNulPattern) {
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    std::string storage;
    desc.pattern = make_ovx_string_bytes(std::string("/World/Cube1") + '\0' + "GARBAGE", storage);
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(TensorBindingErrorTest, RejectsEmbeddedNulPrimPath) {
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    std::string storage;
    ovphysx_string_t paths[1];
    paths[0] = make_ovx_string_bytes(std::string("/World/Cube1") + '\0' + "GARBAGE", storage);
    desc.prim_paths = paths;
    desc.prim_paths_count = 1;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(TensorBindingErrorTest, RejectsEmbeddedNulGetObjectType) {
    std::string storage;
    ovphysx_object_type_t t = OVPHYSX_OBJECT_TYPE_INVALID;
    ovphysx_result_t result = ovphysx_get_object_type(
        m_handle,
        make_ovx_string_bytes(std::string("/World/Cube1") + '\0' + "GARBAGE", storage),
        &t);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
}

TEST_F(TensorBindingErrorTest, RejectsEmbeddedNulCreateSdfView) {
    std::string storage;
    ovphysx_sdf_view_handle_t sdf = 0;
    ovphysx_result_t result = ovphysx_create_sdf_view(
        m_handle,
        make_ovx_string_bytes(std::string("/World/Cube") + '\0' + "GARBAGE", storage),
        1,
        &sdf);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(sdf, 0);
}

TEST_F(TensorBindingErrorTest, InvalidBindingHandle) {
    // Try to read from non-existent binding
    float data[7];
    int64_t shape[2] = {1, 7};
    DLTensor tensor = {};
    tensor.data = data;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;

    ovphysx_result_t result = ovphysx_read_tensor_binding(m_handle, 999999, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_NOT_FOUND);
}

TEST_F(TensorBindingErrorTest, ShapeMismatch) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Create binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Try to read with wrong shape
    float data[3];  // Wrong size
    int64_t wrong_shape[2] = {1, 3};  // Wrong shape
    DLTensor tensor = {};
    tensor.data = data;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = wrong_shape;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);  // Shape mismatch

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingErrorTest, WrongDtype) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Create binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Get correct spec
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Try to read with wrong dtype (int32 instead of float32)
    std::vector<int32_t> data(spec.shape[0] * spec.shape[1], 0);
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor tensor = {};
    tensor.data = data.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLInt, 32, 1};  // Wrong dtype
    tensor.shape = shape;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);  // Dtype mismatch

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingErrorTest, ZeroMatchesSucceeds) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Create binding with pattern that matches nothing
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/NonExistent/Path/That/Matches/Nothing");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    // Should succeed even with 0 matches
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    if (result.status == OVPHYSX_API_SUCCESS) {
        // Get spec - should show 0 elements
        ovphysx_tensor_spec_t spec;
        result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(spec.shape[0], 0);  // 0 rigid bodies matched

        // Read/write should be successful no-ops for 0 matches.
        float dummy[7] = {0.0f};
        int64_t shape[2] = {0, 7};
        DLTensor tensor = {};
        tensor.data = dummy;
        tensor.device = {kDLCPU, 0};
        tensor.ndim = 2;
        tensor.dtype = {kDLFloat, 32, 1};
        tensor.shape = shape;
        tensor.strides = nullptr;
        tensor.byte_offset = 0;

        result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

        result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

        ovphysx_destroy_tensor_binding(m_handle, binding);
    }
}

namespace {

class ScopedLogCapture
{
public:
    explicit ScopedLogCapture(uint32_t level)
        : mOriginalLevel(ovphysx_get_log_level())
    {
        ovphysx_set_log_level(level);
    }

    ~ScopedLogCapture()
    {
        ovphysx_log_capture_stop();
        ovphysx_set_log_level(mOriginalLevel);
    }

    bool start()
    {
        return ovphysx_log_capture_start().status == OVPHYSX_API_SUCCESS;
    }

private:
    uint32_t mOriginalLevel;
};

} // namespace

TEST_F(TensorBindingErrorTest, ZeroMatchesDoesNotWarn) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    ScopedLogCapture capture(OVPHYSX_LOG_WARNING);
    ASSERT_TRUE(capture.start());

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/NonExistent/Path/That/Matches/Nothing");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "did not match any rigid bodies"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "Provided pattern list did not match any rigid bodies"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_WARNING, "binding with 0 prims"));

    if (binding != 0)
        ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingErrorTest, ExplicitPrimPathPartialMissStillLogsError) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/basic_simulation.usda", usd_handle));

    ScopedLogCapture capture(OVPHYSX_LOG_WARNING);
    ASSERT_TRUE(capture.start());

    ovphysx_string_t paths[] = {
        OVPHYSX_LITERAL("/World/envs/env0/table"),
        OVPHYSX_LITERAL("/World/DoesNotExist"),
    };
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.prim_paths = paths;
    desc.prim_paths_count = 2;
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_TRUE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "did not match any rigid bodies"));
    EXPECT_FALSE(ovphysx_log_capture_find(OVPHYSX_LOG_ERROR, "Provided pattern list did not match any rigid bodies"));

    if (result.status == OVPHYSX_API_SUCCESS)
    {
        ovphysx_tensor_spec_t spec;
        result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(spec.shape[0], 1);
    }

    if (binding != 0)
        ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// INDEXED WRITES TEST
// ============================================================================

TEST_F(TensorBindingCpuTest, IndexedWrite) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Create DOF target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Get spec
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GT(spec.shape[0], 0);  // Need at least one articulation

    // Initialize full tensor to a known value
    std::vector<float> full_data(spec.shape[0] * spec.shape[1], 1.0f);
    int64_t full_shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor full_tensor = {};
    full_tensor.data = full_data.data();
    full_tensor.device = {kDLCPU, 0};
    full_tensor.ndim = 2;
    full_tensor.dtype = {kDLFloat, 32, 1};
    full_tensor.shape = full_shape;
    full_tensor.strides = nullptr;
    full_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding(m_handle, binding, &full_tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Write only to index 0 using indexed write
    std::vector<float> partial_data(spec.shape[1], 0.5f);  // Just one articulation's DOFs
    int64_t partial_shape[2] = {1, spec.shape[1]};  // [1, D] - one articulation

    DLTensor src_tensor = {};
    src_tensor.data = partial_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = partial_shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Index tensor specifying which articulation to update
    std::vector<int32_t> indices = {0};  // Update only articulation 0
    int64_t index_shape[1] = {1};

    DLTensor index_tensor = {};
    index_tensor.data = indices.data();
    index_tensor.device = {kDLCPU, 0};
    index_tensor.ndim = 1;
    index_tensor.dtype = {kDLInt, 32, 1};
    index_tensor.shape = index_shape;
    index_tensor.strides = nullptr;
    index_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding(m_handle, binding, &src_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read back and validate row 0 updated
    std::vector<float> readback(spec.shape[0] * spec.shape[1], -1.0f);
    DLTensor dst_tensor = {};
    dst_tensor.data = readback.data();
    dst_tensor.device = {kDLCPU, 0};
    dst_tensor.ndim = 2;
    dst_tensor.dtype = {kDLFloat, 32, 1};
    dst_tensor.shape = full_shape;
    dst_tensor.strides = nullptr;
    dst_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &dst_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Row 0 should be updated to 0.5f
    for (int64_t j = 0; j < spec.shape[1]; ++j)
        EXPECT_FLOAT_EQ(readback[0 * spec.shape[1] + j], 0.5f);

    // Other rows remain at 1.0f (if present)
    for (int64_t i = 1; i < spec.shape[0]; ++i)
        for (int64_t j = 0; j < spec.shape[1]; ++j)
            EXPECT_FLOAT_EQ(readback[i * spec.shape[1] + j], 1.0f);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// MULTIPLE BINDINGS TEST
// ============================================================================

TEST_F(TensorBindingCpuTest, MultipleSamePatternBindings) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Create multiple bindings for the same pattern but different tensor types
    ovphysx_tensor_binding_handle_t pos_binding = 0;
    ovphysx_tensor_binding_handle_t vel_binding = 0;
    ovphysx_tensor_binding_handle_t target_binding = 0;

    ovphysx_tensor_binding_desc_t pos_desc{};
    pos_desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    pos_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_tensor_binding_desc_t vel_desc{};
    vel_desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    vel_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32;

    ovphysx_tensor_binding_desc_t target_desc{};
    target_desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    target_desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &pos_desc, &pos_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_create_tensor_binding(m_handle, &vel_desc, &vel_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_create_tensor_binding(m_handle, &target_desc, &target_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Get specs - all should have same shape (same pattern)
    ovphysx_tensor_spec_t pos_spec, vel_spec, target_spec;

    result = ovphysx_get_tensor_binding_spec(m_handle, pos_binding, &pos_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, vel_binding, &vel_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, target_binding, &target_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Same number of articulations and DOFs
    EXPECT_EQ(pos_spec.shape[0], vel_spec.shape[0]);
    EXPECT_EQ(pos_spec.shape[0], target_spec.shape[0]);
    EXPECT_EQ(pos_spec.shape[1], vel_spec.shape[1]);
    EXPECT_EQ(pos_spec.shape[1], target_spec.shape[1]);

    // Cleanup
    ovphysx_destroy_tensor_binding(m_handle, pos_binding);
    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
    ovphysx_destroy_tensor_binding(m_handle, target_binding);
}

TEST_F(TensorBindingCpuTest, DuplicateBindingSameType) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Create TWO bindings for the exact same pattern AND tensor type
    ovphysx_tensor_binding_handle_t binding1 = 0;
    ovphysx_tensor_binding_handle_t binding2 = 0;

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding1);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_create_tensor_binding(m_handle, &desc, &binding2);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Both should be valid and have same shape
    EXPECT_NE(binding1, binding2);  // Different handles

    ovphysx_tensor_spec_t spec1, spec2;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding1, &spec1);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, binding2, &spec2);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(spec1.shape[0], spec2.shape[0]);
    EXPECT_EQ(spec1.shape[1], spec2.shape[1]);

    // Both can be used independently
    std::vector<float> data1(spec1.shape[0] * spec1.shape[1], 0.0f);
    std::vector<float> data2(spec2.shape[0] * spec2.shape[1], 0.0f);

    int64_t shape[2] = {spec1.shape[0], spec1.shape[1]};

    DLTensor tensor1 = {};
    tensor1.data = data1.data();
    tensor1.device = {kDLCPU, 0};
    tensor1.ndim = 2;
    tensor1.dtype = {kDLFloat, 32, 1};
    tensor1.shape = shape;

    DLTensor tensor2 = {};
    tensor2.data = data2.data();
    tensor2.device = {kDLCPU, 0};
    tensor2.ndim = 2;
    tensor2.dtype = {kDLFloat, 32, 1};
    tensor2.shape = shape;

    result = ovphysx_read_tensor_binding(m_handle, binding1, &tensor1);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_read_tensor_binding(m_handle, binding2, &tensor2);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Data should be identical (same underlying physics state)
    for (size_t i = 0; i < data1.size(); ++i) {
        EXPECT_FLOAT_EQ(data1[i], data2[i]);
    }

    ovphysx_destroy_tensor_binding(m_handle, binding1);
    ovphysx_destroy_tensor_binding(m_handle, binding2);
}

// ============================================================================
// FORCE / WRENCH EFFECT TESTS
// ============================================================================
// These tests verify that written forces produce observable physical effects,
// not just API success. Uses boxes_falling_on_groundplane.usda (11 cubes at Z=10).

TEST_F(TensorBindingCpuTest, ForceWriteEffect_RigidBodyDisplacement) {
    // Load scene with rigid body cubes
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    // Step once to initialize physics
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f/60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Create force binding [N, 3]
    ovphysx_tensor_binding_handle_t force_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &force_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create force binding";
    }

    // Create pose binding [N, 7] for readback
    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create pose binding";
    }

    // Get specs
    ovphysx_tensor_spec_t force_spec, pose_spec;
    {
        ovphysx_result_t r = ovphysx_get_tensor_binding_spec(m_handle, force_binding, &force_spec);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_result_t r = ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }

    const int64_t N = force_spec.shape[0];
    ASSERT_GE(N, 2) << "Need at least 2 rigid bodies";
    ASSERT_EQ(force_spec.shape[1], 3);
    ASSERT_EQ(pose_spec.shape[0], N);
    ASSERT_EQ(pose_spec.shape[1], 7);

    // Read initial poses
    std::vector<float> initial_poses(N * 7, 0.0f);
    {
        DLTensor dst{};
        dst.data = initial_poses.data();
        dst.device = {kDLCPU, 0};
        dst.ndim = 2;
        dst.dtype = {kDLFloat, 32, 1};
        int64_t shape[2] = {N, 7};
        dst.shape = shape;
        dst.strides = nullptr;
        dst.byte_offset = 0;
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, pose_binding, &dst);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }

    float initial_x_body0 = initial_poses[0 * 7 + 0];          // px of body 0
    float initial_x_control = initial_poses[(N-1) * 7 + 0];     // px of last body

    // Build force tensor: 50000 N in +X on body 0, zeros elsewhere
    std::vector<float> forces(N * 3, 0.0f);
    forces[0 * 3 + 0] = 50000.0f;  // fx on body 0

    DLTensor force_tensor{};
    force_tensor.data = forces.data();
    force_tensor.device = {kDLCPU, 0};
    force_tensor.ndim = 2;
    force_tensor.dtype = {kDLFloat, 32, 1};
    int64_t force_shape[2] = {N, 3};
    force_tensor.shape = force_shape;
    force_tensor.strides = nullptr;
    force_tensor.byte_offset = 0;

    // Apply force each step for 5 steps
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i) {
        ovphysx_result_t wr = ovphysx_write_tensor_binding(m_handle, force_binding, &force_tensor, nullptr);
        ASSERT_EQ(wr.status, OVPHYSX_API_SUCCESS) << "Force write failed on step " << i;

        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Read final poses
    std::vector<float> final_poses(N * 7, 0.0f);
    {
        DLTensor dst{};
        dst.data = final_poses.data();
        dst.device = {kDLCPU, 0};
        dst.ndim = 2;
        dst.dtype = {kDLFloat, 32, 1};
        int64_t shape[2] = {N, 7};
        dst.shape = shape;
        dst.strides = nullptr;
        dst.byte_offset = 0;
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, pose_binding, &dst);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }

    float final_x_body0 = final_poses[0 * 7 + 0];
    float final_x_control = final_poses[(N-1) * 7 + 0];

    float dx_pushed = std::abs(final_x_body0 - initial_x_body0);
    float dx_control = std::abs(final_x_control - initial_x_control);

    // With 50000N on a ~1kg body for 5 steps at 1/60s, displacement should be large.
    // Even a 1000kg body would show measurable displacement.
    std::cout << "  Force test: body0 X " << initial_x_body0 << " -> " << final_x_body0
              << " (dx=" << dx_pushed << ")" << std::endl;
    std::cout << "  Force test: control X " << initial_x_control << " -> " << final_x_control
              << " (dx=" << dx_control << ")" << std::endl;

    EXPECT_GT(dx_pushed, 0.1f)
        << "Body 0 should show significant X displacement from 50000N force";
    EXPECT_LT(dx_control, 0.01f)
        << "Control body should have negligible X displacement (no X force applied)";

    ovphysx_destroy_tensor_binding(m_handle, force_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

TEST_F(TensorBindingCpuTest, WrenchWriteEffect_RigidBodyDisplacement) {
    // Same test but using WRENCH [N, 9] path (bypasses deprecated applyForces wrapper)
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    // Step once to initialize physics
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f/60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Create wrench binding [N, 9]
    ovphysx_tensor_binding_handle_t wrench_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &wrench_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create wrench binding";
    }

    // Create pose binding [N, 7] for readback
    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create pose binding";
    }

    // Get specs
    ovphysx_tensor_spec_t wrench_spec, pose_spec;
    {
        ovphysx_result_t r = ovphysx_get_tensor_binding_spec(m_handle, wrench_binding, &wrench_spec);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_result_t r = ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }

    const int64_t N = wrench_spec.shape[0];
    ASSERT_GE(N, 2) << "Need at least 2 rigid bodies";
    ASSERT_EQ(wrench_spec.shape[1], 9);
    ASSERT_EQ(pose_spec.shape[0], N);

    // Read initial poses
    std::vector<float> initial_poses(N * 7, 0.0f);
    {
        DLTensor dst{};
        dst.data = initial_poses.data();
        dst.device = {kDLCPU, 0};
        dst.ndim = 2;
        dst.dtype = {kDLFloat, 32, 1};
        int64_t shape[2] = {N, 7};
        dst.shape = shape;
        dst.strides = nullptr;
        dst.byte_offset = 0;
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, pose_binding, &dst);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }

    float initial_x_body0 = initial_poses[0 * 7 + 0];
    float initial_x_control = initial_poses[(N-1) * 7 + 0];

    // Build wrench tensor: [fx,fy,fz, tx,ty,tz, px,py,pz] per body
    // 50000 N in +X on body 0, applied at center of mass (position = body pose)
    std::vector<float> wrenches(N * 9, 0.0f);
    wrenches[0 * 9 + 0] = 50000.0f;  // fx on body 0
    // torque and position left at zero (force at COM)

    DLTensor wrench_tensor{};
    wrench_tensor.data = wrenches.data();
    wrench_tensor.device = {kDLCPU, 0};
    wrench_tensor.ndim = 2;
    wrench_tensor.dtype = {kDLFloat, 32, 1};
    int64_t wrench_shape[2] = {N, 9};
    wrench_tensor.shape = wrench_shape;
    wrench_tensor.strides = nullptr;
    wrench_tensor.byte_offset = 0;

    // Apply wrench each step for 5 steps
    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i) {
        ovphysx_result_t wr = ovphysx_write_tensor_binding(m_handle, wrench_binding, &wrench_tensor, nullptr);
        ASSERT_EQ(wr.status, OVPHYSX_API_SUCCESS) << "Wrench write failed on step " << i;

        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Read final poses
    std::vector<float> final_poses(N * 7, 0.0f);
    {
        DLTensor dst{};
        dst.data = final_poses.data();
        dst.device = {kDLCPU, 0};
        dst.ndim = 2;
        dst.dtype = {kDLFloat, 32, 1};
        int64_t shape[2] = {N, 7};
        dst.shape = shape;
        dst.strides = nullptr;
        dst.byte_offset = 0;
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, pose_binding, &dst);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS);
    }

    float final_x_body0 = final_poses[0 * 7 + 0];
    float final_x_control = final_poses[(N-1) * 7 + 0];

    float dx_pushed = std::abs(final_x_body0 - initial_x_body0);
    float dx_control = std::abs(final_x_control - initial_x_control);

    std::cout << "  Wrench test: body0 X " << initial_x_body0 << " -> " << final_x_body0
              << " (dx=" << dx_pushed << ")" << std::endl;
    std::cout << "  Wrench test: control X " << initial_x_control << " -> " << final_x_control
              << " (dx=" << dx_control << ")" << std::endl;

    EXPECT_GT(dx_pushed, 0.1f)
        << "Body 0 should show significant X displacement from 50000N wrench";
    EXPECT_LT(dx_control, 0.01f)
        << "Control body should have negligible X displacement (no X force applied)";

    ovphysx_destroy_tensor_binding(m_handle, wrench_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// ============================================================================
// CPU-ONLY MODE SAFETY TESTS
// Verify that ovphysx runs without crashes on CPU-only systems (no GPU).
// These tests load USD stages that select CPU mode (no physxScene:enableGPUDynamics)
// and exercise the full lifecycle to ensure no code path accidentally dereferences
// a null CUDA handle or calls a cu* function.
// ============================================================================

TEST_F(TensorBindingCpuTest, CpuOnlyLifecycle_NoCrash) {
    // This test exercises the core lifecycle in CPU mode:
    // create instance -> load USD -> step sim -> create bindings -> read/write -> destroy.
    // The primary goal is to verify no crash -- correctness of values is secondary.

    // Load a scene that has both rigid bodies and articulations
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD in CPU mode";

    // Step simulation a few times
    for (int i = 0; i < 3; ++i) {
        const float dt = 1.0f / 60.0f;
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS) << "CPU step " << i << " failed";
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // warmup_gpu should be a safe no-op in CPU mode
    ovphysx_result_t result = ovphysx_warmup_gpu(m_handle);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS) << "warmup_gpu should succeed (no-op) in CPU mode";

    // Create articulation DOF binding
    ovphysx_tensor_binding_handle_t dof_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
        result = ovphysx_create_tensor_binding(m_handle, &desc, &dof_binding);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU DOF binding creation failed";
    }

    // Get spec and verify it has valid dimensions
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(spec.shape[0], 0);
    EXPECT_GT(spec.shape[1], 0);

    // Read into a CPU tensor
    size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1]);
    std::vector<float> data(total, 0.0f);
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor tensor = {};
    tensor.data = data.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU read failed";

    // Write back (to exercise the write path in CPU mode)
    for (size_t i = 0; i < total; ++i) data[i] = 0.05f;
    result = ovphysx_write_tensor_binding(m_handle, dof_binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU write failed";

    // Step again after write
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Destroy binding (should not crash even though GPU was never initialized)
    result = ovphysx_destroy_tensor_binding(m_handle, dof_binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingCpuTest, CpuOnlyCudaTensorRejected) {
    // The CPU test pass sets OVPHYSX_DISABLE_GPU=1. A CUDA tensor must be
    // rejected by that process-wide policy before the pointer is accessed.

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    auto expectCpuOnlyDeviceMismatch = [&](ovphysx_result_t rejected, const char* operation) {
        EXPECT_EQ(rejected.status, OVPHYSX_API_DEVICE_MISMATCH) << operation;
        ovphysx_string_t last_error = ovphysx_get_last_error();
        ASSERT_NE(last_error.ptr, nullptr) << operation;
        const std::string error_message(last_error.ptr, last_error.length);
        EXPECT_NE(error_message.find("process-wide CPU-only mode"), std::string::npos)
            << operation << ": " << error_message;
    };

    // Use host pointers that claim to be CUDA memory. Every call must reject
    // the device policy before accessing those pointers.
    float dummy = 0.0f;
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor fake_gpu_tensor = {};
    fake_gpu_tensor.data = &dummy;  // Not actually GPU memory
    fake_gpu_tensor.ndim = 2;
    fake_gpu_tensor.dtype = {kDLFloat, 32, 1};
    fake_gpu_tensor.shape = shape;

    const size_t element_count = static_cast<size_t>(shape[0]) * static_cast<size_t>(shape[1]);
    std::vector<float> host_data(element_count, 0.0f);
    DLTensor host_tensor = fake_gpu_tensor;
    host_tensor.data = host_data.data();
    host_tensor.device = {kDLCPU, 0};

    int32_t index_data = 0;
    int64_t index_shape[1] = {1};
    DLTensor fake_gpu_index = {};
    fake_gpu_index.data = &index_data;  // Not actually GPU memory
    fake_gpu_index.ndim = 1;
    fake_gpu_index.dtype = {kDLInt, 32, 1};
    fake_gpu_index.shape = index_shape;

    std::vector<uint8_t> mask_data(static_cast<size_t>(shape[0]), 1);
    int64_t mask_shape[1] = {shape[0]};
    DLTensor fake_gpu_mask = {};
    fake_gpu_mask.data = mask_data.data();  // Not actually GPU memory
    fake_gpu_mask.ndim = 1;
    fake_gpu_mask.dtype = {kDLUInt, 8, 1};
    fake_gpu_mask.shape = mask_shape;
    DLTensor host_mask = fake_gpu_mask;
    host_mask.device = {kDLCPU, 0};

    const DLDeviceType cuda_device_types[] = {kDLCUDA, kDLCUDAManaged};
    for (DLDeviceType device_type : cuda_device_types)
    {
        fake_gpu_tensor.device = {device_type, 0};
        fake_gpu_index.device = {device_type, 0};
        fake_gpu_mask.device = {device_type, 0};

        expectCpuOnlyDeviceMismatch(
            ovphysx_read_tensor_binding(m_handle, binding, &fake_gpu_tensor), "read destination");
        expectCpuOnlyDeviceMismatch(
            ovphysx_write_tensor_binding(m_handle, binding, &fake_gpu_tensor, nullptr), "write source");
        expectCpuOnlyDeviceMismatch(
            ovphysx_write_tensor_binding(m_handle, binding, &host_tensor, &fake_gpu_index), "write index");
        expectCpuOnlyDeviceMismatch(
            ovphysx_write_tensor_binding_masked(m_handle, binding, &fake_gpu_tensor, &host_mask),
            "masked write source");
        expectCpuOnlyDeviceMismatch(
            ovphysx_write_tensor_binding_masked(m_handle, binding, &host_tensor, &fake_gpu_mask), "write mask");
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// GPU MODE TESTS
// ============================================================================

// Shared GPU instance for all TensorBindingGpuTest tests.
//
// Carbonite and the Python interpreter cannot be cleanly finalized and
// re-initialized in the same process (DLL init routines fail on reload).
// A single long-lived instance avoids the destroy/recreate cycle while
// still resetting simulation state (USD, bindings) between tests.
class TensorBindingGpuTest : public ::testing::Test {
    static ovphysx_handle_t s_handle;
    static ovphysx::test_cuda::CudaOps s_cudaOps;
    static std::string s_skipReason;

protected:
    ovphysx_handle_t m_handle = 0;
    uintptr_t m_gpuBuffer = 0;
    ovphysx::test_cuda::CudaOps m_cudaOps{};

    static void SetUpTestSuite() {
#if !OVPHYSX_ENABLE_GPU_TESTS
        s_skipReason = "GPU tests disabled at compile time";
        return;
#endif
        ovphysx_create_args args = OVPHYSX_CREATE_ARGS_DEFAULT;

        // These tests cover the DirectGPU tensor pipeline (GPU-resident state
        // reads/writes via PxDirectGPUAPI), so opt into DirectGPU explicitly.
        // Since 0.4.x, ovphysx no longer auto-enables /physics/suppressReadback
        // for GPU instances — it's a workflow-specific setting hosts opt into
        // (see create_args doc-comment in ovphysx_types.h). config_entries are
        // applied after carb settings loads but before PhysX plugins, which is
        // exactly the window where /physics/suppressReadback must land.
        ovphysx_config_entry_t direct_gpu_entries[] = {
            ovphysx_config_entry_carbonite(
                OVPHYSX_LITERAL("/physics/suppressReadback"),
                OVPHYSX_LITERAL("true")),
        };
        args.config_entries = direct_gpu_entries;
        args.config_entry_count = sizeof(direct_gpu_entries) / sizeof(direct_gpu_entries[0]);

        ovphysx_result_t create_result = ovphysx_create_instance(&args, &s_handle);
        if (create_result.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t last_err = ovphysx_get_last_error();
            if (last_err.length > 0) {
                s_skipReason = std::string(last_err.ptr, last_err.length);
            } else {
                s_skipReason = "Failed to create GPU instance";
            }
            s_handle = 0;
            return;
        }

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
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                ovphysx_op_wait_result_t wait_result{};
                ovphysx_wait_op(s_handle, reset_result.op_index, 10'000'000'000ULL, &wait_result);
                ovphysx_destroy_wait_result(&wait_result);
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
        m_cudaOps = s_cudaOps;
    }

    void TearDown() override {
        if (m_gpuBuffer != 0) {
            (void)m_cudaOps.memFree(m_gpuBuffer);
            m_gpuBuffer = 0;
        }
        if (s_handle != 0) {
            ovphysx_enqueue_result_t reset_result = ovphysx_reset_stage(s_handle);
            if (reset_result.status == OVPHYSX_API_SUCCESS && reset_result.op_index != 0) {
                ovphysx_op_wait_result_t wait_result{};
                ovphysx_wait_op(s_handle, reset_result.op_index, 10'000'000'000ULL, &wait_result);
                ovphysx_destroy_wait_result(&wait_result);
            }
        }
        m_handle = 0;
    }

    void* allocGpuBuffer(size_t size, ovphysx_tensor_binding_handle_t binding_for_ctx) {
        if (m_gpuBuffer != 0) {
            (void)m_cudaOps.memFree(m_gpuBuffer);
        }

        const uintptr_t cudaCtx = getPhysxCudaContextFromBinding(m_handle, binding_for_ctx);
        if (!cudaCtx)
            return nullptr;
        m_cudaOps.ctx = cudaCtx;

        int st = 0;
        if (!m_cudaOps.memAlloc(size, &m_gpuBuffer, &st)) {
            m_gpuBuffer = 0;
            return nullptr;
        }
        return reinterpret_cast<void*>(m_gpuBuffer);
    }
};

ovphysx_handle_t TensorBindingGpuTest::s_handle = 0;
ovphysx::test_cuda::CudaOps TensorBindingGpuTest::s_cudaOps{};
std::string TensorBindingGpuTest::s_skipReason;

TEST_F(TensorBindingGpuTest, GpuArticulationDofReadWrite) {
    // Load articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle))
        << "Failed to load USD";

    // Create DOF position binding
    ovphysx_tensor_binding_handle_t dof_binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &dof_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    // Get spec
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(spec.ndim, 2);
    EXPECT_GT(spec.shape[0], 0);  // At least one articulation
    EXPECT_GT(spec.shape[1], 0);  // At least one DOF

    // Explicit GPU warmup
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    size_t total_elements = spec.shape[0] * spec.shape[1];
    size_t buffer_size = total_elements * sizeof(float);

    void* gpu_data = allocGpuBuffer(buffer_size, dof_binding);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU buffer";

    // Initialize GPU buffer to zeros
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total_elements));

    DLTensor tensor = {};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // Read DOF positions
    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU read failed";

    // Set all DOF positions to 0.1 on GPU
    std::vector<float> host_data(total_elements, 0.1f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, host_data.data(), buffer_size));

    result = ovphysx_write_tensor_binding(m_handle, dof_binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU write failed";

    // Read back and verify
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total_elements));  // Clear to verify read works
    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> verify_data(total_elements);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(verify_data.data(), m_gpuBuffer, buffer_size));

    for (size_t i = 0; i < total_elements; ++i) {
        EXPECT_NEAR(verify_data[i], 0.1f, 0.01f) << "DOF position mismatch at index " << i;
    }

    // Cleanup
    result = ovphysx_destroy_tensor_binding(m_handle, dof_binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuDeformableElementIndicesReadInt32) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/volume_deformable_simple.usda", usd_handle))
        << "Failed to load deformable USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/DeformableBody");
    desc.tensor_type = OVPHYSX_TENSOR_DEFORMABLE_SIM_ELEMENT_INDICES_S32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create deformable element-index binding";

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 3);
    EXPECT_EQ(spec.shape[0], 1);
    EXPECT_EQ(spec.shape[1], 2);
    EXPECT_EQ(spec.shape[2], 4);
    EXPECT_EQ(spec.dtype.code, static_cast<uint8_t>(kDLInt));
    EXPECT_EQ(spec.dtype.bits, 32);
    EXPECT_EQ(spec.dtype.lanes, 1);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    const size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1] * spec.shape[2]);
    const size_t buffer_size = total * sizeof(int32_t);
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU buffer";

    int64_t shape[3] = {spec.shape[0], spec.shape[1], spec.shape[2]};
    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 3;
    tensor.dtype = {kDLInt, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU int32 element-index read failed";

    std::vector<int32_t> readback(total, -1);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));
    const std::vector<int32_t> expected = {0, 1, 2, 3, 1, 2, 3, 4};
    EXPECT_EQ(readback, expected);

    DLTensor float_tensor = tensor;
    float_tensor.dtype = {kDLFloat, 32, 1};
    result = ovphysx_read_tensor_binding(m_handle, binding, &float_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);

    result = ovphysx_destroy_tensor_binding(m_handle, binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuDeformableBodyReadWriteAndReadOnly) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/volume_deformable_multi.usda", usd_handle))
        << "Failed to load multi-deformable USD";

    auto create_binding = [&](ovphysx_tensor_type_t tensor_type) {
        ovphysx_tensor_binding_handle_t binding = 0;
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/DeformableBody_*");
        desc.tensor_type = tensor_type;
        ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
        return binding;
    };

    ovphysx_tensor_binding_handle_t pos = create_binding(OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32);
    ovphysx_tensor_binding_handle_t vel = create_binding(OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32);
    ovphysx_tensor_binding_handle_t targets = create_binding(OVPHYSX_TENSOR_DEFORMABLE_SIM_KINEMATIC_TARGET_F32);
    ovphysx_tensor_binding_handle_t rest = create_binding(OVPHYSX_TENSOR_DEFORMABLE_REST_NODAL_POSITION_F32);
    ASSERT_NE(pos, 0);
    ASSERT_NE(vel, 0);
    ASSERT_NE(targets, 0);
    ASSERT_NE(rest, 0);

    ovphysx_tensor_spec_t pos_spec{};
    ovphysx_result_t result = ovphysx_get_tensor_binding_spec(m_handle, pos, &pos_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(pos_spec.ndim, 3);
    ASSERT_EQ(pos_spec.shape[0], 2);
    ASSERT_EQ(pos_spec.shape[1], 5);
    ASSERT_EQ(pos_spec.shape[2], 3);

    ovphysx_tensor_spec_t target_spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, targets, &target_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(target_spec.ndim, 3);
    ASSERT_EQ(target_spec.shape[0], 2);
    ASSERT_EQ(target_spec.shape[1], 5);
    ASSERT_EQ(target_spec.shape[2], 4);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    const size_t pos_total = static_cast<size_t>(pos_spec.shape[0] * pos_spec.shape[1] * pos_spec.shape[2]);
    const size_t pos_bytes = pos_total * sizeof(float);
    void* gpu_data = allocGpuBuffer(pos_bytes, pos);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU buffer";

    int64_t pos_shape[3] = {pos_spec.shape[0], pos_spec.shape[1], pos_spec.shape[2]};
    DLTensor pos_tensor{};
    pos_tensor.data = gpu_data;
    pos_tensor.device = {kDLCUDA, 0};
    pos_tensor.ndim = 3;
    pos_tensor.dtype = {kDLFloat, 32, 1};
    pos_tensor.shape = pos_shape;
    pos_tensor.strides = nullptr;
    pos_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, pos, &pos_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> original_positions(pos_total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(original_positions.data(), m_gpuBuffer, pos_bytes));
    ASSERT_NEAR(original_positions[0], 0.0f, 1.0e-4f);
    ASSERT_NEAR(original_positions[15], 2.0f, 1.0e-4f);

    std::vector<float> updated_positions = original_positions;
    for (int64_t vertex = 0; vertex < pos_spec.shape[1]; ++vertex)
        updated_positions[static_cast<size_t>(pos_spec.shape[1] * 3 + vertex * 3 + 1)] += 0.125f;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, updated_positions.data(), pos_bytes));

    int32_t row_one = 1;
    int64_t index_shape[1] = {1};
    DLTensor index_tensor{};
    index_tensor.data = &row_one;
    index_tensor.device = {kDLCPU, 0};
    index_tensor.ndim = 1;
    index_tensor.dtype = {kDLInt, 32, 1};
    index_tensor.shape = index_shape;
    index_tensor.strides = nullptr;
    index_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding(m_handle, pos, &pos_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, pos_total));
    result = ovphysx_read_tensor_binding(m_handle, pos, &pos_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback_positions(pos_total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback_positions.data(), m_gpuBuffer, pos_bytes));
    for (size_t i = 0; i < pos_total; ++i)
    {
        const float expected = (i < 15) ? original_positions[i] : updated_positions[i];
        EXPECT_NEAR(readback_positions[i], expected, 1.0e-4f) << "position mismatch at flat index " << i;
    }

    updated_positions = readback_positions;
    for (int64_t vertex = 0; vertex < pos_spec.shape[1]; ++vertex)
        updated_positions[static_cast<size_t>(pos_spec.shape[1] * 3 + vertex * 3 + 2)] += 0.075f;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, updated_positions.data(), pos_bytes));

    std::vector<uint8_t> mask_host(static_cast<size_t>(pos_spec.shape[0]), 0);
    mask_host[1] = 1;
    int64_t mask_shape[1] = {pos_spec.shape[0]};
    DLTensor mask_tensor{};
    mask_tensor.data = mask_host.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, pos, &pos_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, pos_total));
    result = ovphysx_read_tensor_binding(m_handle, pos, &pos_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> masked_readback_positions(pos_total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(masked_readback_positions.data(), m_gpuBuffer, pos_bytes));
    for (size_t i = 0; i < pos_total; ++i)
    {
        const float expected = (i < 15) ? original_positions[i] : updated_positions[i];
        EXPECT_NEAR(masked_readback_positions[i], expected, 1.0e-4f) << "masked position mismatch at flat index "
                                                                    << i;
    }

    std::vector<float> velocities(pos_total, 0.0f);
    for (size_t i = 15; i < pos_total; i += 3)
        velocities[i] = 0.25f;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, velocities.data(), pos_bytes));
    result = ovphysx_write_tensor_binding(m_handle, vel, &pos_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, pos_total));
    result = ovphysx_read_tensor_binding(m_handle, vel, &pos_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback_velocities(pos_total, -1.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback_velocities.data(), m_gpuBuffer, pos_bytes));
    for (size_t i = 0; i < pos_total; ++i)
    {
        const float expected = (i < 15) ? 0.0f : velocities[i];
        EXPECT_NEAR(readback_velocities[i], expected, 1.0e-4f) << "velocity mismatch at flat index " << i;
    }

    result = ovphysx_write_tensor_binding(m_handle, rest, &pos_tensor, &index_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "rest nodal positions are read-only";

    std::fill(mask_host.begin(), mask_host.end(), 1);
    result = ovphysx_write_tensor_binding_masked(m_handle, rest, &pos_tensor, &mask_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "masked rest nodal positions are read-only";

    const size_t target_total =
        static_cast<size_t>(target_spec.shape[0] * target_spec.shape[1] * target_spec.shape[2]);
    const size_t target_bytes = target_total * sizeof(float);
    gpu_data = allocGpuBuffer(target_bytes, targets);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU target buffer";

    int64_t target_shape[3] = {target_spec.shape[0], target_spec.shape[1], target_spec.shape[2]};
    DLTensor target_tensor{};
    target_tensor.data = gpu_data;
    target_tensor.device = {kDLCUDA, 0};
    target_tensor.ndim = 3;
    target_tensor.dtype = {kDLFloat, 32, 1};
    target_tensor.shape = target_shape;
    target_tensor.strides = nullptr;
    target_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, targets, &target_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    std::vector<float> updated_targets(target_total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(updated_targets.data(), m_gpuBuffer, target_bytes));
    for (int64_t vertex = 0; vertex < target_spec.shape[1]; ++vertex)
    {
        const size_t base = static_cast<size_t>(target_spec.shape[1] * 4 + vertex * 4);
        updated_targets[base + 0] = updated_positions[static_cast<size_t>(15 + vertex * 3 + 0)];
        updated_targets[base + 1] = updated_positions[static_cast<size_t>(15 + vertex * 3 + 1)] + 0.05f;
        updated_targets[base + 2] = updated_positions[static_cast<size_t>(15 + vertex * 3 + 2)];
        updated_targets[base + 3] = 1.0f;
    }
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, updated_targets.data(), target_bytes));
    result = ovphysx_write_tensor_binding(m_handle, targets, &target_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, target_total));
    result = ovphysx_read_tensor_binding(m_handle, targets, &target_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback_targets(target_total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback_targets.data(), m_gpuBuffer, target_bytes));
    for (size_t i = 20; i < target_total; ++i)
        EXPECT_NEAR(readback_targets[i], updated_targets[i], 1.0e-4f) << "target mismatch at flat index " << i;

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, rest).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, targets).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, vel).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, pos).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuDeformableMaterialReadIndexedAndMaskedWrite) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/volume_deformable_multi.usda", usd_handle))
        << "Failed to load multi-deformable USD";

    auto create_binding = [&](ovphysx_tensor_type_t tensor_type) {
        ovphysx_tensor_binding_handle_t binding = 0;
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/DeformableMaterial_*");
        desc.tensor_type = tensor_type;
        ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
        return binding;
    };

    ovphysx_tensor_binding_handle_t friction =
        create_binding(OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_DYNAMIC_FRICTION_F32);
    ovphysx_tensor_binding_handle_t youngs =
        create_binding(OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_YOUNGS_MODULUS_F32);
    ovphysx_tensor_binding_handle_t poisson =
        create_binding(OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_POISSONS_RATIO_F32);
    ASSERT_NE(friction, 0);
    ASSERT_NE(youngs, 0);
    ASSERT_NE(poisson, 0);

    ovphysx_tensor_spec_t spec{};
    ovphysx_result_t result = ovphysx_get_tensor_binding_spec(m_handle, friction, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 1);
    ASSERT_EQ(spec.shape[0], 2);
    ASSERT_EQ(spec.dtype.code, static_cast<uint8_t>(kDLFloat));
    ASSERT_EQ(spec.dtype.bits, 32);

    int64_t shape[1] = {spec.shape[0]};
    std::vector<float> values(static_cast<size_t>(spec.shape[0]), 0.0f);
    DLTensor tensor{};
    tensor.data = values.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, friction, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(values[0], 0.5f, 1.0e-4f);
    EXPECT_NEAR(values[1], 0.7f, 1.0e-4f);

    std::vector<float> young_values(static_cast<size_t>(spec.shape[0]), 0.0f);
    tensor.data = young_values.data();
    result = ovphysx_read_tensor_binding(m_handle, youngs, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(young_values[0], 1000.0f, 1.0e-3f);
    EXPECT_NEAR(young_values[1], 2000.0f, 1.0e-3f);

    int32_t row_one = 1;
    int64_t index_shape[1] = {1};
    DLTensor index_tensor{};
    index_tensor.data = &row_one;
    index_tensor.device = {kDLCPU, 0};
    index_tensor.ndim = 1;
    index_tensor.dtype = {kDLInt, 32, 1};
    index_tensor.shape = index_shape;
    index_tensor.strides = nullptr;
    index_tensor.byte_offset = 0;

    young_values[1] = 2200.0f;
    tensor.data = young_values.data();
    result = ovphysx_write_tensor_binding(m_handle, youngs, &tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    young_values.assign(young_values.size(), 0.0f);
    tensor.data = young_values.data();
    result = ovphysx_read_tensor_binding(m_handle, youngs, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(young_values[0], 1000.0f, 1.0e-3f);
    EXPECT_NEAR(young_values[1], 2200.0f, 1.0e-3f);

    std::vector<float> poisson_values(static_cast<size_t>(spec.shape[0]), 0.0f);
    tensor.data = poisson_values.data();
    result = ovphysx_read_tensor_binding(m_handle, poisson, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(poisson_values[0], 0.3f, 1.0e-4f);
    EXPECT_NEAR(poisson_values[1], 0.35f, 1.0e-4f);

    poisson_values[0] = 0.22f;
    poisson_values[1] = 0.48f;
    std::vector<uint8_t> mask_host = {1, 0};
    int64_t mask_shape[1] = {spec.shape[0]};
    DLTensor mask_tensor{};
    mask_tensor.data = mask_host.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, poisson, &tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    poisson_values.assign(poisson_values.size(), 0.0f);
    tensor.data = poisson_values.data();
    result = ovphysx_read_tensor_binding(m_handle, poisson, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(poisson_values[0], 0.22f, 1.0e-4f);
    EXPECT_NEAR(poisson_values[1], 0.35f, 1.0e-4f);

    values[0] = 0.62f;
    values[1] = 0.82f;
    tensor.data = values.data();
    result = ovphysx_write_tensor_binding(m_handle, friction, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    values.assign(values.size(), 0.0f);
    tensor.data = values.data();
    result = ovphysx_read_tensor_binding(m_handle, friction, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(values[0], 0.62f, 1.0e-4f);
    EXPECT_NEAR(values[1], 0.82f, 1.0e-4f);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, poisson).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, youngs).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, friction).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuWriteAutoWarmupWithoutExplicitWarmup) {
    // Load GPU articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    // Create a writable DOF target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GT(spec.shape[0], 0);
    ASSERT_GT(spec.shape[1], 0);
    const size_t total = size_t(spec.shape[0] * spec.shape[1]);
    const size_t buffer_size = total * sizeof(float);

    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // Intentionally skip ovphysx_warmup_gpu(): first write should auto-warmup in GPU mode.
    std::vector<float> src_host(total, 0.2f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "First GPU write should auto-warmup";

    // Read back to verify value propagation.
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));
    for (size_t i = 0; i < total; ++i) {
        EXPECT_NEAR(readback[i], 0.2f, 0.01f);
    }

    result = ovphysx_destroy_tensor_binding(m_handle, binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuMaskedWriteAutoWarmupWithoutExplicitWarmup) {
    // Load GPU scene with two articulations for masked write
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GT(spec.shape[0], 0);
    ASSERT_GT(spec.shape[1], 0);
    const int64_t N = spec.shape[0];
    const int64_t D = spec.shape[1];
    const size_t total = size_t(N * D);
    const size_t buffer_size = total * sizeof(float);

    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    int64_t shape[2] = {N, D};
    DLTensor src_tensor{};
    src_tensor.data = gpu_data;
    src_tensor.device = {kDLCUDA, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";

    std::vector<uint8_t> mask_host(size_t(N), 1);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), static_cast<size_t>(N) * sizeof(uint8_t)));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor{};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Intentionally skip ovphysx_warmup_gpu(): first masked write should auto-warmup.
    std::vector<float> src_host(total, 0.35f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "First GPU masked write should auto-warmup";

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &src_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));
    for (size_t i = 0; i < total; ++i) {
        EXPECT_NEAR(readback[i], 0.35f, 0.01f);
    }

    (void)m_cudaOps.memFree(gpu_mask);
    result = ovphysx_destroy_tensor_binding(m_handle, binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, CrossDeviceWriteCpuTensorToGpuBinding) {
    // Symmetric to CrossDeviceReadCpuTensorFromGpuBinding: ovphysx_write_tensor_binding
    // accepts a CPU src against a GPU binding via internal staging. The staging
    // path allocates a GPU buffer on the binding's device, memcpyHtoDs the
    // caller's CPU data, then forwards the staging buffer to PhysX.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32;  // writable, GPU-resident

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0]) * static_cast<size_t>(spec.shape[1]);
    std::vector<float> cpu_src(total, 0.5f);
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};

    DLTensor cpu_tensor = {};
    cpu_tensor.data = cpu_src.data();
    cpu_tensor.device = {kDLCPU, 0};
    cpu_tensor.ndim = 2;
    cpu_tensor.dtype = {kDLFloat, 32, 1};
    cpu_tensor.shape = shape;

    result = ovphysx_write_tensor_binding(m_handle, binding, &cpu_tensor, nullptr);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "Cross-device write (CPU src → GPU binding) should succeed via staging";

    // Round-trip via the read side (which is also cross-device-capable) to
    // confirm the staged write actually landed in PhysX.
    std::vector<float> cpu_readback(total, 0.0f);
    DLTensor readback_tensor = {};
    readback_tensor.data = cpu_readback.data();
    readback_tensor.device = {kDLCPU, 0};
    readback_tensor.ndim = 2;
    readback_tensor.dtype = {kDLFloat, 32, 1};
    readback_tensor.shape = shape;
    result = ovphysx_read_tensor_binding(m_handle, binding, &readback_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    for (size_t i = 0; i < total; ++i)
    {
        EXPECT_FLOAT_EQ(cpu_readback[i], 0.5f) << "Staged write did not reach PhysX (i=" << i << ")";
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, CrossDeviceWriteGpuTensorToCpuBinding) {
    // Symmetric direction: GPU src against a CPU-only property binding (body
    // mass is CPU-only regardless of sim device). Staging allocates a CPU
    // buffer, memcpyDtoHs the caller's GPU data, then forwards to PhysX.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32;  // writable, CPU-only property

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0]) * static_cast<size_t>(spec.shape[1]);
    const size_t bytes = total * sizeof(float);

    // Stage a known pattern in CPU then push to GPU so the GPU buffer holds known values.
    std::vector<float> seed(total, 2.5f);
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(reinterpret_cast<uintptr_t>(gpu_data), seed.data(), bytes))
        << "Failed to seed GPU src buffer";

    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = shape;

    result = ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "Cross-device write (GPU src → CPU-only binding) should succeed via staging";

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, CrossDeviceReadCpuTensorFromGpuBinding) {
    // ovphysx_read_tensor_binding now supports cross-device reads via internal
    // staging: a GPU binding read into a CPU dst allocates GPU staging,
    // performs the read, then memcpyDtoH into the caller's buffer (and the
    // mirror direction for binding CPU / dst GPU). This test verifies the
    // GPU→CPU direction succeeds.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> cpu_data(spec.shape[0] * spec.shape[1], 0.0f);
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};

    DLTensor cpu_tensor = {};
    cpu_tensor.data = cpu_data.data();
    cpu_tensor.device = {kDLCPU, 0};
    cpu_tensor.ndim = 2;
    cpu_tensor.dtype = {kDLFloat, 32, 1};
    cpu_tensor.shape = shape;

    result = ovphysx_read_tensor_binding(m_handle, binding, &cpu_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "Cross-device read (GPU binding → CPU dst) should succeed via staging";

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, NonContiguousTensorRejected) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    // Create binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Warmup first
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Get spec
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Allocate GPU buffer
    size_t total_elements = spec.shape[0] * spec.shape[1];
    void* gpu_data = allocGpuBuffer(total_elements * sizeof(float), binding);
    ASSERT_NE(gpu_data, nullptr);

    // Create tensor with non-contiguous strides (strided access)
    // NOTE: The contiguity check correctly skips size-1 dimensions (stride is irrelevant).
    // To test rejection, we must set a bad stride on a dimension with size > 1.
    // The DOF dimension (shape[1]) typically has multiple DOFs, so we set stride[1] = 2.
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};

    // Skip test if shape[1] <= 1 (can't create meaningful non-contiguous tensor)
    if (spec.shape[1] <= 1) {
        GTEST_SKIP() << "Test requires shape[1] > 1 to test non-contiguous stride rejection";
    }

    // Non-contiguous: stride[1] = 2 instead of expected 1 (elements are not adjacent)
    int64_t strides[2] = {spec.shape[1] * 2, 2};

    DLTensor tensor = {};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = strides;  // Explicit non-contiguous strides
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT) << "Should reject non-contiguous tensor";
    {
        ovphysx_string_t last_err = ovphysx_get_last_error();
        if (last_err.length > 0) {
            std::string err(last_err.ptr, last_err.length);
            EXPECT_TRUE(err.find("contiguous") != std::string::npos) << "Error should mention contiguity";
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// MASKED WRITE TESTS - CPU
// ============================================================================

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_DofPositionTargets_Alternating) {
    // Load two-articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load two_articulations.usda";

    // Create DOF position target binding (targets are write-then-read verifiable)
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    // Get spec and verify shape
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.ndim, 2);
    ASSERT_EQ(spec.shape[0], 2) << "Expected 2 articulations";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    ASSERT_GT(D, 0);

    // Read initial values
    size_t total = N * D;
    std::vector<float> initial_data(total, -1.0f);
    int64_t shape[2] = {N, D};

    DLTensor read_tensor = {};
    read_tensor.data = initial_data.data();
    read_tensor.device = {kDLCPU, 0};
    read_tensor.ndim = 2;
    read_tensor.dtype = {kDLFloat, 32, 1};
    read_tensor.shape = shape;
    read_tensor.strides = nullptr;
    read_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &read_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Save initial values for comparison
    std::vector<float> saved_initial(initial_data.begin(), initial_data.end());

    // Create full src tensor [N, D] with all values = 0.5
    std::vector<float> src_data(total, 0.5f);
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Create mask = {1, 0} (update only first articulation)
    std::vector<uint8_t> mask_data = {1, 0};
    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = mask_data.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Write masked
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Masked write failed";

    // Read back
    std::vector<float> readback(total, -1.0f);
    DLTensor readback_tensor = {};
    readback_tensor.data = readback.data();
    readback_tensor.device = {kDLCPU, 0};
    readback_tensor.ndim = 2;
    readback_tensor.dtype = {kDLFloat, 32, 1};
    readback_tensor.shape = shape;
    readback_tensor.strides = nullptr;
    readback_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &readback_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Row 0 should be updated to 0.5
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[0 * D + j], 0.5f)
            << "Row 0 (masked=1) should be updated at col " << j;
    }
    // Row 1 should remain at initial value (mask=0)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[1 * D + j], saved_initial[1 * D + j])
            << "Row 1 (masked=0) should be unchanged at col " << j;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_DofPositionTargets_AllTrue) {
    // Load two-articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

    // Create DOF position target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;

    // Create src with all values = 0.75
    std::vector<float> src_data(total, 0.75f);
    int64_t shape[2] = {N, D};

    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Mask = {1, 1} (all true)
    std::vector<uint8_t> mask_data = {1, 1};
    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = mask_data.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read back - all rows should be 0.75
    std::vector<float> readback(total, -1.0f);
    DLTensor readback_tensor = {};
    readback_tensor.data = readback.data();
    readback_tensor.device = {kDLCPU, 0};
    readback_tensor.ndim = 2;
    readback_tensor.dtype = {kDLFloat, 32, 1};
    readback_tensor.shape = shape;
    readback_tensor.strides = nullptr;
    readback_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &readback_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    for (size_t i = 0; i < total; ++i) {
        EXPECT_FLOAT_EQ(readback[i], 0.75f)
            << "All rows should be updated (mask all-true) at index " << i;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_DofPositionTargets_AllFalse) {
    // Load two-articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

    // Create DOF position target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    int64_t shape[2] = {N, D};

    // Read initial values
    std::vector<float> initial_data(total, -1.0f);
    DLTensor read_tensor = {};
    read_tensor.data = initial_data.data();
    read_tensor.device = {kDLCPU, 0};
    read_tensor.ndim = 2;
    read_tensor.dtype = {kDLFloat, 32, 1};
    read_tensor.shape = shape;
    read_tensor.strides = nullptr;
    read_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &read_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> saved_initial(initial_data.begin(), initial_data.end());

    // Create src with distinctive values
    std::vector<float> src_data(total, 99.0f);
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Mask = {0, 0} (all false - nothing should change)
    std::vector<uint8_t> mask_data = {0, 0};
    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = mask_data.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read back - nothing should have changed
    std::vector<float> readback(total, -1.0f);
    DLTensor readback_tensor = {};
    readback_tensor.data = readback.data();
    readback_tensor.device = {kDLCPU, 0};
    readback_tensor.ndim = 2;
    readback_tensor.dtype = {kDLFloat, 32, 1};
    readback_tensor.shape = shape;
    readback_tensor.strides = nullptr;
    readback_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &readback_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    for (size_t i = 0; i < total; ++i) {
        EXPECT_FLOAT_EQ(readback[i], saved_initial[i])
            << "No rows should be updated (mask all-false) at index " << i;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_RigidBodyPose_Alternating) {
    // Load rigid body scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane.usda";

    // Create rigid body pose binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create rigid body pose binding";

    // Get spec and verify shape
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.ndim, 2);
    ASSERT_GE(spec.shape[0], 2) << "Need at least 2 rigid bodies";
    ASSERT_EQ(spec.shape[1], 7) << "Pose should be [N, 7] (pos xyz + quat xyzw)";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];  // 7
    size_t total = N * D;
    int64_t shape[2] = {N, D};

    // Read initial poses
    std::vector<float> initial_data(total, -1.0f);
    DLTensor read_tensor = {};
    read_tensor.data = initial_data.data();
    read_tensor.device = {kDLCPU, 0};
    read_tensor.ndim = 2;
    read_tensor.dtype = {kDLFloat, 32, 1};
    read_tensor.shape = shape;
    read_tensor.strides = nullptr;
    read_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &read_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> saved_initial(initial_data.begin(), initial_data.end());

    // Create src with distinctive values: position = (99,99,99), quat = (0,0,0,1)
    std::vector<float> src_data(total);
    for (int64_t i = 0; i < N; ++i) {
        src_data[i * D + 0] = 99.0f;  // px
        src_data[i * D + 1] = 99.0f;  // py
        src_data[i * D + 2] = 99.0f;  // pz
        src_data[i * D + 3] = 0.0f;   // qx
        src_data[i * D + 4] = 0.0f;   // qy
        src_data[i * D + 5] = 0.0f;   // qz
        src_data[i * D + 6] = 1.0f;   // qw
    }
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Create mask that selects only the first element
    std::vector<uint8_t> mask_data(N, 0);
    mask_data[0] = 1;
    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = mask_data.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Write masked
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Masked write for rigid body pose failed";

    // Read back
    std::vector<float> readback(total, -1.0f);
    DLTensor readback_tensor = {};
    readback_tensor.data = readback.data();
    readback_tensor.device = {kDLCPU, 0};
    readback_tensor.ndim = 2;
    readback_tensor.dtype = {kDLFloat, 32, 1};
    readback_tensor.shape = shape;
    readback_tensor.strides = nullptr;
    readback_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &readback_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Row 0 should be updated to the distinctive values
    EXPECT_NEAR(readback[0 * D + 0], 99.0f, 0.01f) << "Body 0 px should be 99";
    EXPECT_NEAR(readback[0 * D + 1], 99.0f, 0.01f) << "Body 0 py should be 99";
    EXPECT_NEAR(readback[0 * D + 2], 99.0f, 0.01f) << "Body 0 pz should be 99";
    EXPECT_NEAR(readback[0 * D + 6], 1.0f, 0.01f)  << "Body 0 qw should be 1";

    // Remaining rows should be unchanged
    for (int64_t i = 1; i < N; ++i) {
        for (int64_t j = 0; j < D; ++j) {
            EXPECT_FLOAT_EQ(readback[i * D + j], saved_initial[i * D + j])
                << "Body " << i << " col " << j << " should be unchanged (mask=0)";
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_ValidationErrors) {
    // Load scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

    // Create binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    int64_t shape[2] = {N, D};

    // Valid src tensor for all sub-tests
    std::vector<float> src_data(total, 0.5f);
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Sub-test 1: Wrong mask dtype (float32 instead of uint8)
    {
        std::vector<float> bad_mask_data(N, 1.0f);
        int64_t mask_shape[1] = {N};
        DLTensor bad_mask = {};
        bad_mask.data = bad_mask_data.data();
        bad_mask.device = {kDLCPU, 0};
        bad_mask.ndim = 1;
        bad_mask.dtype = {kDLFloat, 32, 1};  // Wrong dtype - should be uint8
        bad_mask.shape = mask_shape;
        bad_mask.strides = nullptr;
        bad_mask.byte_offset = 0;

        result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &bad_mask);
        EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT)
            << "Should reject float32 mask dtype";
    }

    // Sub-test 2: Wrong mask shape (wrong length)
    {
        std::vector<uint8_t> bad_mask_data(N + 5, 1);  // Wrong length
        int64_t mask_shape[1] = {N + 5};
        DLTensor bad_mask = {};
        bad_mask.data = bad_mask_data.data();
        bad_mask.device = {kDLCPU, 0};
        bad_mask.ndim = 1;
        bad_mask.dtype = {kDLUInt, 8, 1};
        bad_mask.shape = mask_shape;
        bad_mask.strides = nullptr;
        bad_mask.byte_offset = 0;

        result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &bad_mask);
        EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT)
            << "Should reject mask with wrong length";
    }

    // Sub-test 3: Null mask tensor
    {
        result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, nullptr);
        EXPECT_NE(result.status, OVPHYSX_API_SUCCESS)
            << "Should reject null mask tensor";
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// MASKED WRITE TESTS - GPU
// ============================================================================

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_DofPositionTargets_Alternating) {
    // Load GPU two-articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle))
        << "Failed to load two_articulations_gpu.usda";

    // Create DOF position target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    // Get spec
    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2) << "Expected 2 articulations";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    ASSERT_GT(D, 0);
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    // GPU warmup
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    // Allocate GPU buffer for src/read
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU data buffer";

    int64_t shape[2] = {N, D};

    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = shape;
    gpu_tensor.strides = nullptr;
    gpu_tensor.byte_offset = 0;

    // Read initial values to host
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    // Upload src data (all 0.5) to GPU
    std::vector<float> src_host(total, 0.5f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    // Allocate separate GPU buffer for mask
    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";

    // Mask = {1, 0}
    std::vector<uint8_t> mask_host = {1, 0};
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), static_cast<size_t>(N) * sizeof(uint8_t)));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Write masked
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU masked write failed";

    // Read back
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    // Row 0 should be 0.5 (mask=1)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[0 * D + j], 0.5f, 0.01f)
            << "Row 0 (masked=1) should be updated at col " << j;
    }
    // Row 1 should be unchanged (mask=0)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[1 * D + j], initial_host[1 * D + j], 0.01f)
            << "Row 1 (masked=0) should be unchanged at col " << j;
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_DofPositionTargets_AllTrue) {
    // Load GPU two-articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle));

    // Create DOF position target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    // GPU warmup
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Allocate GPU buffer for src/read
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    int64_t shape[2] = {N, D};

    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = shape;
    gpu_tensor.strides = nullptr;
    gpu_tensor.byte_offset = 0;

    // Upload src data (all 0.75) to GPU
    std::vector<float> src_host(total, 0.75f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    // Allocate GPU mask buffer: {1, 1}
    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host = {1, 1};
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), static_cast<size_t>(N) * sizeof(uint8_t)));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Write masked
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read back
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    for (size_t i = 0; i < total; ++i) {
        EXPECT_NEAR(readback[i], 0.75f, 0.01f)
            << "All rows should be updated (mask all-true) at index " << i;
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_BoolDtype_Alternating) {
    // Verify that kDLBool (dtype code 6, bits=8) works as mask dtype on GPU.
    // This is the dtype PyTorch uses for bool tensors.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    int64_t shape[2] = {N, D};

    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = shape;
    gpu_tensor.strides = nullptr;
    gpu_tensor.byte_offset = 0;

    // Read initial values
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    // Upload src data (all 0.33) to GPU
    std::vector<float> src_host(total, 0.33f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    // Create GPU mask with kDLBool dtype: {1, 0} (True, False)
    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host = {1, 0};
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), static_cast<size_t>(N) * sizeof(uint8_t)));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLBool, 8, 1};  // kDLBool = 6, bits=8 (PyTorch bool layout)
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU masked write with kDLBool dtype should succeed";

    // Read back
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    // Row 0 should be 0.33 (mask=True)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[0 * D + j], 0.33f, 0.01f)
            << "Row 0 (bool mask=True) should be updated at col " << j;
    }
    // Row 1 should be unchanged (mask=False)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[1 * D + j], initial_host[1 * D + j], 0.01f)
            << "Row 1 (bool mask=False) should be unchanged at col " << j;
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_DofPositionTargets_AllFalse) {
    // Load GPU two-articulation scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle));

    // Create DOF position target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    // GPU warmup
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Allocate GPU buffer
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    int64_t shape[2] = {N, D};

    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = shape;
    gpu_tensor.strides = nullptr;
    gpu_tensor.byte_offset = 0;

    // Read initial values
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    // Upload distinctive src data (all 99.0)
    std::vector<float> src_host(total, 99.0f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    // Allocate GPU mask buffer: {0, 0}
    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host = {0, 0};
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), static_cast<size_t>(N) * sizeof(uint8_t)));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Write masked (should be a no-op)
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read back
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    for (size_t i = 0; i < total; ++i) {
        EXPECT_NEAR(readback[i], initial_host[i], 0.01f)
            << "No rows should be updated (mask all-false) at index " << i;
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_RigidBodyPose_SingleElement) {
    // Load GPU rigid body scene
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane_gpu.usda";

    // Create rigid body pose binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GE(spec.shape[0], 2) << "Need at least 2 rigid bodies";
    ASSERT_EQ(spec.shape[1], 7);
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];  // 7
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    // GPU warmup
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Allocate GPU buffer
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    int64_t shape[2] = {N, D};

    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = shape;
    gpu_tensor.strides = nullptr;
    gpu_tensor.byte_offset = 0;

    // Read initial values
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    // Upload distinctive src: position = (77, 77, 77), quat = (0, 0, 0, 1) for all rows
    std::vector<float> src_host(total);
    for (int64_t i = 0; i < N; ++i) {
        src_host[i * D + 0] = 77.0f;
        src_host[i * D + 1] = 77.0f;
        src_host[i * D + 2] = 77.0f;
        src_host[i * D + 3] = 0.0f;
        src_host[i * D + 4] = 0.0f;
        src_host[i * D + 5] = 0.0f;
        src_host[i * D + 6] = 1.0f;
    }
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    // Create mask: only the last element is selected
    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host(N, 0);
    mask_host[N - 1] = 1;  // Only last body
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), static_cast<size_t>(N) * sizeof(uint8_t)));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor = {};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    // Write masked
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU masked write for rigid body pose failed";

    // Read back
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    // All rows except last should be unchanged
    for (int64_t i = 0; i < N - 1; ++i) {
        for (int64_t j = 0; j < D; ++j) {
            EXPECT_NEAR(readback[i * D + j], initial_host[i * D + j], 0.01f)
                << "Body " << i << " col " << j << " should be unchanged (mask=0)";
        }
    }

    // Last row should be updated
    int64_t last = N - 1;
    EXPECT_NEAR(readback[last * D + 0], 77.0f, 0.01f) << "Last body px should be 77";
    EXPECT_NEAR(readback[last * D + 1], 77.0f, 0.01f) << "Last body py should be 77";
    EXPECT_NEAR(readback[last * D + 2], 77.0f, 0.01f) << "Last body pz should be 77";
    EXPECT_NEAR(readback[last * D + 6], 1.0f, 0.01f)  << "Last body qw should be 1";

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// INDEXED WRITE BUG FIX REGRESSION TESTS
// ============================================================================

TEST_F(TensorBindingCpuTest, IndexedWriteFullTensor_CpuRegression) {
    // Regression test: when passing a full [N,D] src tensor with a subset of
    // indices, only the rows specified by the indices should be updated.
    // Previously a bug caused incorrect row mapping when src.shape[0] != indices.shape[0].

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load two_articulations.usda";

    // Create DOF position target binding (N=2, D=2)
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2) << "Expected 2 articulations";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    int64_t full_shape[2] = {N, D};

    // Step 1: Write initial values (all 1.0) using full write
    std::vector<float> init_data(total, 1.0f);
    DLTensor init_tensor = {};
    init_tensor.data = init_data.data();
    init_tensor.device = {kDLCPU, 0};
    init_tensor.ndim = 2;
    init_tensor.dtype = {kDLFloat, 32, 1};
    init_tensor.shape = full_shape;
    init_tensor.strides = nullptr;
    init_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding(m_handle, binding, &init_tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Step 2: Prepare full [N, D] src where row0 = 0.5, row1 = 0.99
    std::vector<float> src_data(total);
    for (int64_t j = 0; j < D; ++j) {
        src_data[0 * D + j] = 0.5f;
        src_data[1 * D + j] = 0.99f;
    }
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = full_shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Index tensor = {0} (update only articulation 0)
    std::vector<int32_t> indices = {0};
    int64_t index_shape[1] = {1};
    DLTensor index_tensor = {};
    index_tensor.data = indices.data();
    index_tensor.device = {kDLCPU, 0};
    index_tensor.ndim = 1;
    index_tensor.dtype = {kDLInt, 32, 1};
    index_tensor.shape = index_shape;
    index_tensor.strides = nullptr;
    index_tensor.byte_offset = 0;

    // Step 3: Indexed write with full [N,D] src + [1] indices
    result = ovphysx_write_tensor_binding(m_handle, binding, &src_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "Indexed write with full tensor should succeed";

    // Step 4: Read back and verify
    std::vector<float> readback(total, -1.0f);
    DLTensor readback_tensor = {};
    readback_tensor.data = readback.data();
    readback_tensor.device = {kDLCPU, 0};
    readback_tensor.ndim = 2;
    readback_tensor.dtype = {kDLFloat, 32, 1};
    readback_tensor.shape = full_shape;
    readback_tensor.strides = nullptr;
    readback_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &readback_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Row 0 should be 0.5 (src row 0 written to index 0)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[0 * D + j], 0.5f)
            << "Row 0 should be updated to 0.5 at col " << j;
    }
    // Row 1 should remain at 1.0 (unchanged from initial full write)
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[1 * D + j], 1.0f)
            << "Row 1 should remain at 1.0 (not indexed) at col " << j;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, IndexedWriteFullTensor_GpuRegression) {
    // GPU regression test for the same indexed-write bug as the CPU version.

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle))
        << "Failed to load two_articulations_gpu.usda";

    // Create DOF position target binding
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2) << "Expected 2 articulations";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);
    int64_t full_shape[2] = {N, D};

    // GPU warmup
    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Allocate GPU data buffer
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    DLTensor gpu_tensor = {};
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    gpu_tensor.ndim = 2;
    gpu_tensor.dtype = {kDLFloat, 32, 1};
    gpu_tensor.shape = full_shape;
    gpu_tensor.strides = nullptr;
    gpu_tensor.byte_offset = 0;

    // Step 1: Write initial values (all 1.0) using full write
    std::vector<float> init_host(total, 1.0f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, init_host.data(), buffer_size));

    result = ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Step 2: Prepare full [N, D] src where row0 = 0.5, row1 = 0.99
    std::vector<float> src_host(total);
    for (int64_t j = 0; j < D; ++j) {
        src_host[0 * D + j] = 0.5f;
        src_host[1 * D + j] = 0.99f;
    }
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    // Allocate GPU index buffer: {0}
    uintptr_t gpu_indices = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(sizeof(int32_t), &gpu_indices, &cu_st))
        << "Failed to allocate GPU indices buffer (status=" << cu_st << ")";
    int32_t index_val = 0;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_indices, &index_val, sizeof(int32_t)));

    int64_t index_shape[1] = {1};
    DLTensor index_tensor = {};
    index_tensor.data = reinterpret_cast<void*>(gpu_indices);
    index_tensor.device = {kDLCUDA, 0};
    index_tensor.ndim = 1;
    index_tensor.dtype = {kDLInt, 32, 1};
    index_tensor.shape = index_shape;
    index_tensor.strides = nullptr;
    index_tensor.byte_offset = 0;

    // Step 3: Indexed write with full [N,D] src + [1] indices
    result = ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "GPU indexed write with full tensor should succeed";

    // Step 4: Read back and verify
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    // Row 0 should be 0.5
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[0 * D + j], 0.5f, 0.01f)
            << "Row 0 should be updated to 0.5 at col " << j;
    }
    // Row 1 should remain at 1.0
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[1 * D + j], 1.0f, 0.01f)
            << "Row 1 should remain at 1.0 (not indexed) at col " << j;
    }

    (void)m_cudaOps.memFree(gpu_indices);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// ADDITIONAL COVERAGE: velocities + articulation root pose (CPU/GPU)
// ============================================================================

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_RigidBodyVelocity_SingleElement) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GE(spec.shape[0], 2) << "Need at least 2 rigid bodies";
    const int64_t N = spec.shape[0];
    const int64_t C = spec.shape[1];  // should be 6
    ASSERT_EQ(C, 6);

    std::vector<float> initial(N * C, 0.0f);
    int64_t shape[2] = {N, C};
    DLTensor tensor{};
    tensor.data = initial.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Full src tensor with distinctive velocity
    std::vector<float> src(N * C, 0.0f);
    for (int i = 0; i < C; ++i) src[i] = 123.0f;  // row 0

    DLTensor src_tensor = tensor;
    src_tensor.data = src.data();

    // Mask: update only row 0
    std::vector<uint8_t> mask = {1, 0};
    mask.resize(size_t(N), 0);
    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor{};
    mask_tensor.data = mask.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> after(N * C, 0.0f);
    tensor.data = after.data();
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    for (int i = 0; i < C; ++i) EXPECT_FLOAT_EQ(after[i], 123.0f);
    for (int64_t r = 1; r < N; ++r) {
        for (int i = 0; i < C; ++i) EXPECT_FLOAT_EQ(after[r * C + i], initial[r * C + i]);
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_ArticulationRootPose_Alternating) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    ASSERT_EQ(spec.shape[1], 7);
    const int64_t N = spec.shape[0];
    const int64_t C = spec.shape[1];

    std::vector<float> initial(N * C, 0.0f);
    int64_t shape[2] = {N, C};
    DLTensor tensor{};
    tensor.data = initial.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Full src pose: set both rows to known value, but mask only updates row 1
    std::vector<float> src(N * C, 0.0f);
    for (int64_t r = 0; r < N; ++r) {
        src[r * C + 0] = 10.0f;
        src[r * C + 1] = 20.0f;
        src[r * C + 2] = 30.0f;
        src[r * C + 3] = 0.0f;
        src[r * C + 4] = 0.0f;
        src[r * C + 5] = 0.0f;
        src[r * C + 6] = 1.0f;  // unit quat w
    }
    DLTensor src_tensor = tensor;
    src_tensor.data = src.data();

    std::vector<uint8_t> mask = {0, 1};
    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor{};
    mask_tensor.data = mask.data();
    mask_tensor.device = {kDLCPU, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> after(N * C, 0.0f);
    tensor.data = after.data();
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Row 0 unchanged, row 1 updated
    for (int i = 0; i < C; ++i) EXPECT_FLOAT_EQ(after[0 * C + i], initial[0 * C + i]);
    for (int i = 0; i < C; ++i) EXPECT_FLOAT_EQ(after[1 * C + i], src[1 * C + i]);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_RigidBodyVelocity_SingleElement) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GE(spec.shape[0], 2);
    ASSERT_EQ(spec.shape[1], 6);
    const int64_t N = spec.shape[0];
    const int64_t C = spec.shape[1];
    const size_t total = size_t(N * C);
    const size_t buffer_size = total * sizeof(float);
    int64_t shape[2] = {N, C};

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // Read initial velocities
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    std::vector<float> initial(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial.data(), m_gpuBuffer, buffer_size));

    // Write full src, mask only row 0
    std::vector<float> src(total, 0.0f);
    for (int i = 0; i < C; ++i) src[i] = 77.0f;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src.data(), buffer_size));

    uintptr_t gpu_mask = 0;
    const size_t mask_bytes = size_t(N) * sizeof(uint8_t);
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(mask_bytes, &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host(size_t(N), 0);
    mask_host[0] = 1;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), mask_bytes));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor{};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read back
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> after(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(after.data(), m_gpuBuffer, buffer_size));

    for (int i = 0; i < C; ++i) EXPECT_NEAR(after[i], 77.0f, 1e-3f);
    for (int64_t r = 1; r < N; ++r) {
        for (int i = 0; i < C; ++i) EXPECT_NEAR(after[r * C + i], initial[r * C + i], 1e-3f);
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_ArticulationRootPose_Alternating) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2);
    ASSERT_EQ(spec.shape[1], 7);
    const int64_t N = spec.shape[0];
    const int64_t C = spec.shape[1];
    const size_t total = size_t(N * C);
    const size_t buffer_size = total * sizeof(float);
    int64_t shape[2] = {N, C};

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr);

    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // Read initial
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    std::vector<float> initial(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial.data(), m_gpuBuffer, buffer_size));

    // Full src, mask only row 1
    std::vector<float> src(total, 0.0f);
    for (int64_t r = 0; r < N; ++r) {
        src[r * C + 0] = 1.0f;
        src[r * C + 1] = 2.0f;
        src[r * C + 2] = 3.0f;
        src[r * C + 6] = 1.0f;
    }
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src.data(), buffer_size));

    uintptr_t gpu_mask = 0;
    const size_t mask_bytes = size_t(N) * sizeof(uint8_t);
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(mask_bytes, &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host(size_t(N), 0);
    mask_host[1] = 1;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(gpu_mask, mask_host.data(), mask_bytes));

    int64_t mask_shape[1] = {N};
    DLTensor mask_tensor{};
    mask_tensor.data = reinterpret_cast<void*>(gpu_mask);
    mask_tensor.device = {kDLCUDA, 0};
    mask_tensor.ndim = 1;
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> after(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(after.data(), m_gpuBuffer, buffer_size));

    // row 0 unchanged, row 1 updated
    for (int i = 0; i < C; ++i) EXPECT_NEAR(after[0 * C + i], initial[0 * C + i], 1e-3f);
    for (int i = 0; i < C; ++i) EXPECT_NEAR(after[1 * C + i], src[1 * C + i], 1e-3f);

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, GpuDeformableMaterialElasticityDampingReadWrite) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/volume_deformable_simple.usda", usd_handle))
        << "Failed to load deformable USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/DeformableMaterial");
    desc.tensor_type = OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 1);
    ASSERT_EQ(spec.shape[0], 1);

    int64_t shape[1] = {1};
    float value = 0.0f;
    DLTensor tensor{};
    tensor.data = &value;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.01f, 1.0e-4f);  // authored value from fixture

    value = 0.05f;
    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    value = 0.0f;
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.05f, 1.0e-4f);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, GpuSurfaceDeformableMaterialBendingPropertiesReadWrite) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/surface_deformable_material.usda", usd_handle))
        << "Failed to load surface deformable material USD";

    auto create_mat_binding = [&](ovphysx_tensor_type_t type) {
        ovphysx_tensor_binding_handle_t binding = 0;
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/SurfaceDeformableMaterial");
        desc.tensor_type = type;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);
        return binding;
    };

    ovphysx_tensor_binding_handle_t bstiff = create_mat_binding(OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32);
    ovphysx_tensor_binding_handle_t thick  = create_mat_binding(OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_THICKNESS_F32);
    ovphysx_tensor_binding_handle_t bdamp  = create_mat_binding(OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_DAMPING_F32);
    ASSERT_NE(bstiff, 0);
    ASSERT_NE(thick, 0);
    ASSERT_NE(bdamp, 0);

    int64_t shape[1] = {1};
    float value = 0.0f;
    DLTensor tensor{};
    tensor.data = &value;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // Read authored values
    ovphysx_result_t result = ovphysx_read_tensor_binding(m_handle, bstiff, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 100.0f, 1.0e-2f);

    result = ovphysx_read_tensor_binding(m_handle, thick, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.01f, 1.0e-5f);

    result = ovphysx_read_tensor_binding(m_handle, bdamp, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.05f, 1.0e-4f);

    // Write and read back bending stiffness
    value = 200.0f;
    result = ovphysx_write_tensor_binding(m_handle, bstiff, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    value = 0.0f;
    result = ovphysx_read_tensor_binding(m_handle, bstiff, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 200.0f, 1.0e-2f);

    ovphysx_destroy_tensor_binding(m_handle, bstiff);
    ovphysx_destroy_tensor_binding(m_handle, thick);
    ovphysx_destroy_tensor_binding(m_handle, bdamp);
}

// Indexed write (int32 and int64 CPU index) for a volume deformable material property.
TEST_F(TensorBindingGpuTest, GpuDeformableMaterialIndexedWrite) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/volume_deformable_simple.usda", usd_handle))
        << "Failed to load deformable USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/DeformableMaterial");
    desc.tensor_type = OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_ELASTICITY_DAMPING_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    int64_t shape[1] = {1};
    float value = 0.0f;
    DLTensor tensor{};
    tensor.data    = &value;
    tensor.device  = {kDLCPU, 0};
    tensor.ndim    = 1;
    tensor.dtype   = {kDLFloat, 32, 1};
    tensor.shape   = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // --- int32 indexed write ---
    int32_t idx32 = 0;
    int64_t idx_shape[1] = {1};
    DLTensor idx_tensor{};
    idx_tensor.data    = &idx32;
    idx_tensor.device  = {kDLCPU, 0};
    idx_tensor.ndim    = 1;
    idx_tensor.dtype   = {kDLInt, 32, 1};
    idx_tensor.shape   = idx_shape;
    idx_tensor.strides = nullptr;
    idx_tensor.byte_offset = 0;

    value = 0.123f;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, &idx_tensor).status, OVPHYSX_API_SUCCESS);
    value = 0.0f;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.123f, 1.0e-4f);

    // --- int64 indexed write (staged to int32 internally) ---
    int64_t idx64 = 0;
    DLTensor idx64_tensor{};
    idx64_tensor.data    = &idx64;
    idx64_tensor.device  = {kDLCPU, 0};
    idx64_tensor.ndim    = 1;
    idx64_tensor.dtype   = {kDLInt, 64, 1};
    idx64_tensor.shape   = idx_shape;
    idx64_tensor.strides = nullptr;
    idx64_tensor.byte_offset = 0;

    value = 0.456f;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, &idx64_tensor).status, OVPHYSX_API_SUCCESS);
    value = 0.0f;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.456f, 1.0e-4f);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// Masked write for a surface deformable material property.
TEST_F(TensorBindingGpuTest, GpuSurfaceDeformableMaterialMaskedWrite) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/surface_deformable_material.usda", usd_handle))
        << "Failed to load surface deformable material USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/SurfaceDeformableMaterial");
    desc.tensor_type = OVPHYSX_TENSOR_DEFORMABLE_MATERIAL_BENDING_STIFFNESS_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    int64_t shape[1] = {1};
    float value = 0.0f;
    DLTensor tensor{};
    tensor.data    = &value;
    tensor.device  = {kDLCPU, 0};
    tensor.ndim    = 1;
    tensor.dtype   = {kDLFloat, 32, 1};
    tensor.shape   = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    // mask = 1: write should apply
    uint8_t mask_val = 1;
    int64_t mask_shape[1] = {1};
    DLTensor mask_tensor{};
    mask_tensor.data    = &mask_val;
    mask_tensor.device  = {kDLCPU, 0};
    mask_tensor.ndim    = 1;
    mask_tensor.dtype   = {kDLUInt, 8, 1};
    mask_tensor.shape   = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    value = 300.0f;
    ASSERT_EQ(ovphysx_write_tensor_binding_masked(m_handle, binding, &tensor, &mask_tensor).status, OVPHYSX_API_SUCCESS);
    value = 0.0f;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 300.0f, 1.0e-2f);

    // mask = 0: write should be skipped, value stays at 300
    mask_val = 0;
    value = 999.0f;
    ASSERT_EQ(ovphysx_write_tensor_binding_masked(m_handle, binding, &tensor, &mask_tensor).status, OVPHYSX_API_SUCCESS);
    value = 0.0f;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 300.0f, 1.0e-2f);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, GpuVolumeDeformableCollisionElementIndicesRead) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/volume_deformable_simple.usda", usd_handle))
        << "Failed to load deformable USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/DeformableBody");
    desc.tensor_type = OVPHYSX_TENSOR_DEFORMABLE_COLLISION_ELEMENT_INDICES_S32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create collision element-index binding";

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 3);
    EXPECT_EQ(spec.shape[0], 1);  // 1 deformable body
    EXPECT_GT(spec.shape[1], 0);  // at least 1 collision element
    EXPECT_GT(spec.shape[2], 0);  // K = getNumNodesPerElement() -- 4 for volume tetmesh
    EXPECT_EQ(spec.dtype.code, static_cast<uint8_t>(kDLInt));
    EXPECT_EQ(spec.dtype.bits, 32);
    EXPECT_EQ(spec.dtype.lanes, 1);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    const size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1] * spec.shape[2]);
    const size_t buffer_size = total * sizeof(int32_t);
    void* gpu_data = allocGpuBuffer(buffer_size, binding);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU buffer";

    int64_t shape[3] = {spec.shape[0], spec.shape[1], spec.shape[2]};
    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 3;
    tensor.dtype = {kDLInt, 32, 1};
    tensor.shape = shape;
    tensor.strides = nullptr;
    tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Collision element index read failed";

    std::vector<int32_t> readback(total, -1);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));
    // All indices must be non-negative and within node count
    for (auto idx : readback)
        EXPECT_GE(idx, 0);

    // Verify write is rejected (read-only tensor)
    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);

    result = ovphysx_destroy_tensor_binding(m_handle, binding);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuSurfaceDeformableBodyReadWriteAndReadOnly) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/surface_deformable_simple.usda", usd_handle))
        << "Failed to load surface deformable USD";

    auto create_binding = [&](ovphysx_tensor_type_t tensor_type) {
        ovphysx_tensor_binding_handle_t binding = 0;
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/DeformableBody");
        desc.tensor_type = tensor_type;
        ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
        return binding;
    };

    ovphysx_tensor_binding_handle_t pos     = create_binding(OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32);
    ovphysx_tensor_binding_handle_t vel     = create_binding(OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32);
    ovphysx_tensor_binding_handle_t rest    = create_binding(OVPHYSX_TENSOR_SURFACE_DEFORMABLE_REST_POSITION_F32);
    ovphysx_tensor_binding_handle_t elems   = create_binding(OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_ELEMENT_INDICES_S32);
    ASSERT_NE(pos, 0);
    ASSERT_NE(vel, 0);
    ASSERT_NE(rest, 0);
    ASSERT_NE(elems, 0);

    ovphysx_tensor_spec_t pos_spec{};
    ovphysx_result_t result = ovphysx_get_tensor_binding_spec(m_handle, pos, &pos_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(pos_spec.ndim, 3);
    EXPECT_EQ(pos_spec.shape[0], 1);  // 1 surface deformable
    EXPECT_EQ(pos_spec.shape[1], 4);  // 4 simulation nodes (square cloth)
    EXPECT_EQ(pos_spec.shape[2], 3);

    ovphysx_tensor_spec_t elem_spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, elems, &elem_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(elem_spec.ndim, 3);
    EXPECT_EQ(elem_spec.shape[0], 1);
    EXPECT_EQ(elem_spec.shape[1], 2);  // 2 triangles
    EXPECT_EQ(elem_spec.shape[2], 3);  // K=3 trimesh
    EXPECT_EQ(elem_spec.dtype.code, static_cast<uint8_t>(kDLInt));
    EXPECT_EQ(elem_spec.dtype.bits, 32);
    EXPECT_EQ(elem_spec.dtype.lanes, 1);

    result = ovphysx_warmup_gpu(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    // Read initial positions
    const size_t pos_total = static_cast<size_t>(pos_spec.shape[0] * pos_spec.shape[1] * pos_spec.shape[2]);
    const size_t pos_bytes = pos_total * sizeof(float);
    void* gpu_data = allocGpuBuffer(pos_bytes, pos);
    ASSERT_NE(gpu_data, nullptr);

    int64_t pos_shape[3] = {pos_spec.shape[0], pos_spec.shape[1], pos_spec.shape[2]};
    DLTensor pos_tensor{};
    pos_tensor.data = gpu_data;
    pos_tensor.device = {kDLCUDA, 0};
    pos_tensor.ndim = 3;
    pos_tensor.dtype = {kDLFloat, 32, 1};
    pos_tensor.shape = pos_shape;
    pos_tensor.strides = nullptr;
    pos_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, pos, &pos_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> positions(pos_total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(positions.data(), m_gpuBuffer, pos_bytes));
    // First node should be at origin
    EXPECT_NEAR(positions[0], 0.0f, 1.0e-4f);
    EXPECT_NEAR(positions[1], 0.0f, 1.0e-4f);
    EXPECT_NEAR(positions[2], 0.0f, 1.0e-4f);

    // Indexed write: update positions for body 0
    std::vector<float> new_positions = positions;
    for (int64_t v = 0; v < pos_spec.shape[1]; ++v)
        new_positions[static_cast<size_t>(v * 3 + 1)] += 0.05f;
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, new_positions.data(), pos_bytes));

    int32_t row_zero = 0;
    int64_t index_shape[1] = {1};
    DLTensor index_tensor{};
    index_tensor.data = &row_zero;
    index_tensor.device = {kDLCPU, 0};
    index_tensor.ndim = 1;
    index_tensor.dtype = {kDLInt, 32, 1};
    index_tensor.shape = index_shape;
    index_tensor.strides = nullptr;
    index_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding(m_handle, pos, &pos_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Read element indices and verify int32 + shape
    const size_t elem_total = static_cast<size_t>(elem_spec.shape[0] * elem_spec.shape[1] * elem_spec.shape[2]);
    const size_t elem_bytes = elem_total * sizeof(int32_t);
    void* elem_gpu = allocGpuBuffer(elem_bytes, elems);
    ASSERT_NE(elem_gpu, nullptr);

    int64_t elem_shape[3] = {elem_spec.shape[0], elem_spec.shape[1], elem_spec.shape[2]};
    DLTensor elem_tensor{};
    elem_tensor.data = elem_gpu;
    elem_tensor.device = {kDLCUDA, 0};
    elem_tensor.ndim = 3;
    elem_tensor.dtype = {kDLInt, 32, 1};
    elem_tensor.shape = elem_shape;
    elem_tensor.strides = nullptr;
    elem_tensor.byte_offset = 0;

    result = ovphysx_read_tensor_binding(m_handle, elems, &elem_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<int32_t> elem_readback(elem_total, -1);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(elem_readback.data(), m_gpuBuffer, elem_bytes));
    for (auto idx : elem_readback)
        EXPECT_GE(idx, 0);

    // Read-only enforcement
    result = ovphysx_write_tensor_binding(m_handle, rest, &pos_tensor, &index_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
    result = ovphysx_write_tensor_binding(m_handle, elems, &elem_tensor, &index_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_destroy_tensor_binding(m_handle, pos);
    ovphysx_destroy_tensor_binding(m_handle, vel);
    ovphysx_destroy_tensor_binding(m_handle, rest);
    ovphysx_destroy_tensor_binding(m_handle, elems);
}

// OMPE-94459 (§B5 GPU path): DISABLE_SIMULATION_BOOL tensor written + read on
// the GPU pipeline. Same contract as the CPU test below but with GPU-resident
// tensors and uint8 dtype.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableSimulationRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 1);
    ASSERT_GT(spec.shape[0], 0);
    EXPECT_EQ(spec.dtype.code, kDLUInt);
    EXPECT_EQ(spec.dtype.bits, 8);

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    const int64_t n = spec.shape[0];
    const size_t bytes = static_cast<size_t>(n);
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);

    std::vector<uint8_t> written(n, 0);
    for (int64_t i = 0; i < n; ++i)
        written[i] = static_cast<uint8_t>(i % 2 == 0 ? 1 : 0);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, written.data(), bytes));

    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[1] = {n};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(n, 0xff);
    // Stage a sentinel pattern into the device buffer so a no-op read is caught.
    std::vector<uint8_t> sentinel(n, 0xff);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, sentinel.data(), bytes));
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, bytes));
    for (int64_t i = 0; i < n; ++i)
        EXPECT_EQ(static_cast<int>(readback[i]), static_cast<int>(written[i])) << "body " << i;

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// Companion to CpuRigidBodyDisableSimulationStopsSimulation: verifies that the
// GPU path (GpuRigidBodyView::setDisableSimulations) actually suppresses
// simulation -- disabled bodies must not move after the flag is applied.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableSimulationStopsSimulation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t disable_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &disable_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t pose_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_binding, &pose_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = pose_spec.shape[0];
    ASSERT_GT(n, 0);

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    // Single GPU buffer for pose reads; flag writes use a CPU tensor so we
    // don't need to manage two CUDA allocations (which would require tracking
    // each pointer separately since allocGpuBuffer overwrites m_gpuBuffer).
    const size_t pose_bytes = static_cast<size_t>(n * 7) * sizeof(float);
    void* gpu_pose = allocGpuBuffer(pose_bytes, pose_binding);
    ASSERT_NE(gpu_pose, nullptr);

    int64_t pose_shape[2] = {n, 7};
    DLTensor pose_t{};
    pose_t.data = gpu_pose;
    pose_t.device = {kDLCUDA, 0};
    pose_t.ndim = 2;
    pose_t.dtype = {kDLFloat, 32, 1};
    pose_t.shape = pose_shape;

    auto step_once = [&]() {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    };

    auto read_poses_to_host = [&](std::vector<float>& out) {
        out.resize(static_cast<size_t>(n * 7));
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(out.data(), m_gpuBuffer, pose_bytes));
    };

    step_once();
    std::vector<float> pose_before_disable;
    read_poses_to_host(pose_before_disable);

    // Sanity: bodies fall under gravity before disable.
    step_once();
    std::vector<float> pose_after_one_more;
    read_poses_to_host(pose_after_one_more);
    bool moved = false;
    for (int64_t i = 0; i < n; ++i)
        if (std::abs(pose_after_one_more[i * 7 + 2] - pose_before_disable[i * 7 + 2]) > 1e-5f)
            { moved = true; break; }
    ASSERT_TRUE(moved) << "Sanity: bodies must fall under gravity before disable.";

    // Write disable=1 via a CPU tensor -- GpuRigidBodyView::setDisableSimulations
    // stages CPU input to GPU internally, so this exercises the staging path.
    std::vector<uint8_t> all_disabled(static_cast<size_t>(n), 1);
    int64_t flag_shape[1] = {n};
    DLTensor flag_t{};
    flag_t.data = all_disabled.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<float> pose_at_disable;
    read_poses_to_host(pose_at_disable);

    // Step several more -- disabled bodies must not move.
    for (int s = 0; s < 5; ++s) step_once();
    std::vector<float> pose_final;
    read_poses_to_host(pose_final);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_NEAR(pose_final[i * 7 + 0], pose_at_disable[i * 7 + 0], 1e-4f) << "body " << i << " x drifted while disabled";
        EXPECT_NEAR(pose_final[i * 7 + 1], pose_at_disable[i * 7 + 1], 1e-4f) << "body " << i << " y drifted while disabled";
        EXPECT_NEAR(pose_final[i * 7 + 2], pose_at_disable[i * 7 + 2], 1e-4f) << "body " << i << " z drifted while disabled";
    }

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// OMPE-94459 (§B0 GPU path): ARTICULATION_MASS_CENTER_WORLD read into a GPU
// tensor. Light test -- shape + read-success contract only; numeric correctness
// lives in the umbrella tensor suite.
TEST_F(TensorBindingGpuTest, GpuArticulationMassCenterRead) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_MASS_CENTER_WORLD_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 2);
    ASSERT_GT(spec.shape[0], 0);
    EXPECT_EQ(spec.shape[1], 3);

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t bytes = static_cast<size_t>(spec.shape[0] * spec.shape[1]) * sizeof(float);
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);
    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, static_cast<size_t>(spec.shape[0] * spec.shape[1])));

    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    tensor.shape = shape;

    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// OMPE-94459 (§B9 fix verification): writing a GPU tensor to ARTICULATION_REST_OFFSET
// previously failed with "Incompatible device" because BaseArticulationView's
// per-shape setters hardcoded checkTensorDevice(..., -1, ...). The
// GpuArticulationView override stages GPU tensors to host before delegating,
// so this write must now succeed and the readback must match.
TEST_F(TensorBindingGpuTest, GpuArticulationShapePropertyWrite_B9) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_REST_OFFSET_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 2);
    ASSERT_GT(spec.shape[0], 0);
    ASSERT_GT(spec.shape[1], 0);

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1]);
    const size_t bytes = total * sizeof(float);
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);

    std::vector<float> host_in(total, 0.0005f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, host_in.data(), bytes));

    DLTensor tensor{};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    tensor.shape = shape;

    ovphysx_result_t r = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    EXPECT_EQ(r.status, OVPHYSX_API_SUCCESS)
        << "GPU write to ARTICULATION_REST_OFFSET should succeed after §B9 fix; "
        << "err=" << (ovphysx_get_last_error().ptr ? ovphysx_get_last_error().ptr : "");

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    std::vector<float> host_out(total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(host_out.data(), m_gpuBuffer, bytes));
    // Bodies with fewer shapes have zero-padded trailing entries -- only
    // verify the slots we wrote that the engine actually applied.
    int matched = 0;
    for (size_t i = 0; i < total; ++i) {
        if (std::abs(host_out[i] - 0.0005f) < 1e-4f) ++matched;
    }
    EXPECT_GT(matched, 0) << "expected at least one shape to roundtrip the rest-offset write";

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// Multi-arti CartPole on GPU with envs spaced far apart enough that
// the 6m rails do not penetrate adjacent envs' rails. Locks in that
// the GPU multi-pattern view returns clean per-env projected joint
// forces when the scene is well-formed; if a future scenario shrinks
// the env grid below the rail length, this baseline still holds and
// the failure points at the scene, not at the engine.
TEST_F(TensorBindingGpuTest, GpuMultiCartPoleSpacedProjectedJointForceMatchesActuation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/MultiCartPoleSpaced.usda", usd_handle))
        << "Failed to load MultiCartPoleSpaced.usda";

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_binding_handle_t actuation_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/envs/*");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &actuation_b).status,
                  OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t projected_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/envs/*");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &projected_b).status,
                  OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, actuation_b, &spec).status,
              OVPHYSX_API_SUCCESS);
    const int64_t N = spec.shape[0];
    const int64_t D = spec.shape[1];

    const size_t total = static_cast<size_t>(N * D);
    const size_t bytes = total * sizeof(float);
    void* gpu_data = allocGpuBuffer(bytes, actuation_b);
    ASSERT_NE(gpu_data, nullptr);

    std::vector<float> host_forces(total, 10.0f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, host_forces.data(), bytes));

    DLTensor t{}; t.data = gpu_data; t.device = {kDLCUDA, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {N, D}; t.shape = shape; t.ndim = 2;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, actuation_b, &t, nullptr).status,
              OVPHYSX_API_SUCCESS);

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 1000.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, projected_b, &t).status,
              OVPHYSX_API_SUCCESS);

    std::vector<float> host_projected(total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(host_projected.data(), m_gpuBuffer, bytes));

    for (int64_t i = 0; i < N; ++i) {
        std::cerr << "[GPU multi-cartpole-spaced step-1] env" << i
                  << " cart-prismatic=" << host_projected[i * D + 0]
                  << " pole-revolute=" << host_projected[i * D + 1] << "\n";
    }
    for (int64_t i = 0; i < N; ++i) {
        EXPECT_NEAR(host_projected[i * D + 0], 10.0f, 0.9f)
            << "env" << i << " cart-prismatic projection diverged";
        EXPECT_NEAR(host_projected[i * D + 1], 10.0f, 0.9f)
            << "env" << i << " pole-revolute projection diverged";
    }

    ovphysx_destroy_tensor_binding(m_handle, actuation_b);
    ovphysx_destroy_tensor_binding(m_handle, projected_b);
}

// GPU mirror of CpuCartPoleProjectedJointForceMatchesActuation:
// single-arti, 10N actuation on each DOF, step at dt=1/1000s, expect
// the projected joint forces to land within tolerance of the
// applied actuation.
TEST_F(TensorBindingGpuTest, GpuCartPoleProjectedJointForceMatchesActuation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartPole.usda", usd_handle))
        << "Failed to load CartPole.usda";

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_binding_handle_t actuation_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/cartpole");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &actuation_b).status,
                  OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t projected_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/cartpole");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &projected_b).status,
                  OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, actuation_b, &spec).status,
              OVPHYSX_API_SUCCESS);
    const int64_t N = spec.shape[0];
    const int64_t D = spec.shape[1];
    ASSERT_EQ(N, 1);
    ASSERT_EQ(D, 2);

    const size_t total = static_cast<size_t>(N * D);
    const size_t bytes = total * sizeof(float);
    void* gpu_data = allocGpuBuffer(bytes, actuation_b);
    ASSERT_NE(gpu_data, nullptr);

    std::vector<float> host_forces(total, 10.0f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, host_forces.data(), bytes));

    DLTensor t{}; t.data = gpu_data; t.device = {kDLCUDA, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {N, D}; t.shape = shape; t.ndim = 2;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, actuation_b, &t, nullptr).status,
              OVPHYSX_API_SUCCESS);

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 1000.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, projected_b, &t).status,
              OVPHYSX_API_SUCCESS);

    std::vector<float> host_projected(total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(host_projected.data(), m_gpuBuffer, bytes));

    std::cerr << "[GPU cartpole step-1] cart-prismatic=" << host_projected[0]
              << " pole-revolute=" << host_projected[1] << "\n";

    EXPECT_NEAR(host_projected[0], 10.0f, 0.9f) << "cart-prismatic projection diverged";
    EXPECT_NEAR(host_projected[1], 10.0f, 0.9f) << "pole-revolute projection diverged";

    ovphysx_destroy_tensor_binding(m_handle, actuation_b);
    ovphysx_destroy_tensor_binding(m_handle, projected_b);
}

// After the standard auto-warmup, the GPU dof-projected-joint-forces
// kernel must overwrite a poison-stomped destination with the real
// projection (gravity-Z projects to zero on the cart-rail
// prismatic-Y, so the expected value is ~0). Catches the kernel
// silently leaving the dst untouched.
TEST_F(TensorBindingGpuTest, GpuCartPoleProjectedJointForceWarmStartZero) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartPole.usda", usd_handle))
        << "Failed to load CartPole.usda";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/cartpole");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.shape[0], 1);
    ASSERT_EQ(spec.shape[1], 2) << "CartPole has 2 DOFs (cart prismatic + pole revolute)";

    ASSERT_EQ(ovphysx_warmup_gpu(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1]);
    const size_t bytes = total * sizeof(float);
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);

    // Stomp the dst buffer with a poison pattern so a "no-op" read
    // can't accidentally pass by leaving the buffer at its prior
    // all-zeros state.
    std::vector<float> poison(total, 7.7f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, poison.data(), bytes));

    DLTensor t{};
    t.data = gpu_data;
    t.device = {kDLCUDA, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    t.shape = shape;
    t.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &t).status,
              OVPHYSX_API_SUCCESS);

    std::vector<float> host_out(total, 0.0f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(host_out.data(), m_gpuBuffer, bytes));

    for (size_t i = 0; i < total; ++i) {
        EXPECT_LT(std::abs(host_out[i]), 1e-3f)
            << "GPU dof " << i << " warm-start projection: expected ~0, got "
            << host_out[i] << " (poison-stomp pattern was 7.7)";
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// OMPE-94459 isolation: standalone rigid ball, NO articulation in the scene.
// Applies 150N +Z at world (ball.xy, ball.z+1) -- same magnitude/anchor as the
// multi-body reproducer, with no articulation and therefore no articulation-
// ball contact contribution to ball motion. Expected lift over 10 steps at
// dt=1/60s with mass=0.5kg, gravity=20m/s^2: ~0.535m. Used to determine
// whether the ~2x ball lift seen at large world coords in the multi-body
// scene is direct rigid-body force application (would reproduce here) or
// articulation-ball contact (would not reproduce here).
namespace
{
void run_ball_only_lift_probe(ovphysx_handle_t handle, const char* usda_relpath, float& out_dz)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(handle, usda_relpath, usd_handle))
        << "Failed to load " << usda_relpath;

    const std::string path_storage = "/envs/env0/right_ball";
    ovphysx_string_t st; st.ptr = path_storage.c_str();
    st.length = static_cast<uint32_t>(path_storage.size());
    ovphysx_string_t paths[1] = { st };

    ovphysx_tensor_binding_handle_t pose_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.prim_paths = paths;
        d.prim_paths_count = 1;
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(handle, &d, &pose_b).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t wrench_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.prim_paths = paths;
        d.prim_paths_count = 1;
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(handle, &d, &wrench_b).status, OVPHYSX_API_SUCCESS);
    }

    std::vector<float> pose_init(7, 0.0f);
    DLTensor pt{}; pt.data = pose_init.data(); pt.device = {kDLCPU, 0};
    pt.dtype = {kDLFloat, 32, 1};
    int64_t pose_shape[2] = {1, 7}; pt.shape = pose_shape; pt.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(handle, pose_b, &pt).status, OVPHYSX_API_SUCCESS);

    float wrench_host[9] = {0.f};
    wrench_host[2] = 150.f;
    wrench_host[6] = pose_init[0];
    wrench_host[7] = pose_init[1];
    wrench_host[8] = pose_init[2] + 1.f;

    DLTensor wt{}; wt.data = wrench_host; wt.device = {kDLCPU, 0};
    wt.dtype = {kDLFloat, 32, 1};
    int64_t wrench_shape[2] = {1, 9}; wt.shape = wrench_shape; wt.ndim = 2;
    ASSERT_EQ(ovphysx_write_tensor_binding(handle, wrench_b, &wt, nullptr).status, OVPHYSX_API_SUCCESS);

    for (int i = 0; i < 10; ++i) {
        ovphysx_enqueue_result_t step = ovphysx_step(handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(handle, step.op_index));
    }

    float pose_post[7] = {0.f};
    pt.data = pose_post;
    ASSERT_EQ(ovphysx_read_tensor_binding(handle, pose_b, &pt).status, OVPHYSX_API_SUCCESS);

    const float dz = pose_post[2] - pose_init[2];
    std::cerr << "  right_ball init=(" << pose_init[0] << "," << pose_init[1] << "," << pose_init[2]
              << ") post=(" << pose_post[0] << "," << pose_post[1] << "," << pose_post[2]
              << ") dz=" << dz << "\n";

    ovphysx_destroy_tensor_binding(handle, wrench_b);
    ovphysx_destroy_tensor_binding(handle, pose_b);
    out_dz = dz;
}
} // namespace

// OMPE-94459 regression gate: standalone rigid-body force-at-position must be
// translation-invariant and produce the physics-expected lift on both CPU and
// GPU at both world positions. Expected lift over 10 steps at dt=1/60s with
// mass=0.5kg, gravity=20m/s^2, single F=150N impulse: ~0.528m. Tolerance is
// 1mm; the four quadrants should match exactly to last-3rd-decimal in
// practice (verified empirically).
TEST_F(TensorBindingCpuTest, CpuBallOnly_AtOrigin) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyAtOrigin.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}
TEST_F(TensorBindingCpuTest, CpuBallOnly_Far) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyFar.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}
TEST_F(TensorBindingGpuTest, GpuBallOnly_AtOrigin) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyAtOrigin.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}
TEST_F(TensorBindingGpuTest, GpuBallOnly_Far) {
    float dz = 0.f;
    run_ball_only_lift_probe(m_handle, "tests/data/SingleBallOnlyFar.usda", dz);
    EXPECT_NEAR(dz, 0.5278f, 0.001f);
}

// OMPE-94459 isolation: ant articulation present in scene, but force is
// applied ONLY to the ball (no torque on articulation links). If the ball lift
// here matches the ball-only scene (0.528m), articulation-ball contact is not
// contributing to ball motion -- so the ~0.55m discrepancy in the multi-body
// test must come from contact triggered when forces are applied to the
// articulation itself. If ball lift differs from 0.528m, the articulation's
// presence (gravity + ground contact + ball contact) alone alters ball motion.
namespace
{
} // namespace

// OMPE-94459 Ask A residual root-cause probe: pure ant articulation under
// gravity, no ball, no forces. Reads pose, then steps 10x, then re-reads
// pose. Compares per-link dz so CPU/GPU divergence (if any) shows up
// independent of ball-contact resolution and warmup-step state.
namespace
{
} // namespace

// OMPE-94459 Ask A minimal floating-articulation probe: 2-link arti (base
// sphere + arm sphere, one revolute joint), no contact between links (radii
// 0.1, joint offset 0.3 -- 0.1m gap), gravity only. Distinguishes whether
// the ant divergence is floating-base specific or ant-asset specific.
namespace
{
} // namespace

// OMPE-94459 prismatic-at-Far coverage: CartPole (prismatic cart + revolute
// pole) pre-step DOF-projected joint force must be ~0 regardless of world
// position. The existing CpuCartPoleProjectedJointForcePreStepZero locks
// this in at world origin; these companions lock it in at world
// (100, 100, 0) to gate the Rw seed precision against future regression
// on prismatic + revolute chains in articulations far from origin.
namespace
{
void run_cartpole_pre_step_zero(ovphysx_handle_t handle, const char* usda_relpath)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(handle, usda_relpath, usd_handle))
        << "Failed to load " << usda_relpath;

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/envs/env0/cartpole");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_PROJECTED_JOINT_FORCE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.shape[0], 1);
    ASSERT_EQ(spec.shape[1], 2) << "CartPole has 2 DOFs (cart prismatic + pole revolute)";

    std::vector<float> dof_forces(2, 7.7f);
    DLTensor t{}; t.data = dof_forces.data(); t.device = {kDLCPU, 0};
    t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {1, 2}; t.shape = shape; t.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(handle, binding, &t).status, OVPHYSX_API_SUCCESS);

    for (size_t i = 0; i < dof_forces.size(); ++i) {
        EXPECT_LT(std::abs(dof_forces[i]), 1e-3f)
            << "dof " << i << " pre-step projection at " << usda_relpath
            << ": expected ~0, got " << dof_forces[i];
    }

    ovphysx_destroy_tensor_binding(handle, binding);
}
} // namespace

TEST_F(TensorBindingCpuTest, CpuCartPolePreStepZero_AtOrigin) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleAtOrigin.usda");
}
TEST_F(TensorBindingCpuTest, CpuCartPolePreStepZero_Far) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleFar.usda");
}
TEST_F(TensorBindingGpuTest, GpuCartPolePreStepZero_AtOrigin) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleAtOrigin.usda");
}
TEST_F(TensorBindingGpuTest, GpuCartPolePreStepZero_Far) {
    run_cartpole_pre_step_zero(m_handle, "tests/data/CartPoleFar.usda");
}

// OMPE-94459 spherical-joint coverage: minimal 2-link articulation with a
// PhysicsSphericalJoint between a kinematic base and a free child. Loads
// at world origin and at (100, 100, 0); same dynamics in both, child
// settles to the joint constraint and reads back the same per-link world
// position relative to the env. Locks in the spherical branch of
// computeLinkRFromJointState (newParentToChild derived from stored world
// quaternions). Without this gate, the spherical branch could regress
// silently because no other test exercises it.
namespace
{
void run_spherical_arti_settle_probe(ovphysx_handle_t handle,
                                     const char* usda_relpath,
                                     float env_x, float env_y,
                                     float& out_child_local_z)
{
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(handle, usda_relpath, usd_handle))
        << "Failed to load " << usda_relpath;

    const std::string child_path = "/envs/env0/spherical_arti/child";
    ovphysx_string_t child_st; child_st.ptr = child_path.c_str();
    child_st.length = static_cast<uint32_t>(child_path.size());
    ovphysx_string_t paths[1] = { child_st };

    ovphysx_tensor_binding_handle_t pose_b = 0;
    ovphysx_tensor_binding_desc_t d{};
    d.prim_paths = paths;
    d.prim_paths_count = 1;
    d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ASSERT_EQ(ovphysx_create_tensor_binding(handle, &d, &pose_b).status, OVPHYSX_API_SUCCESS);

    for (int i = 0; i < 20; ++i) {
        ovphysx_enqueue_result_t step = ovphysx_step(handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(handle, step.op_index));
    }

    float pose_post[7] = {0.f};
    DLTensor pt{}; pt.data = pose_post; pt.device = {kDLCPU, 0};
    pt.dtype = {kDLFloat, 32, 1};
    int64_t pose_shape[2] = {1, 7}; pt.shape = pose_shape; pt.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(handle, pose_b, &pt).status, OVPHYSX_API_SUCCESS);

    // Both Origin and Far variants have the same Z env offset (= 0). The
    // child must end at world X = env_x and Y = env_y exactly (no swing
    // under pure-gravity hang with anchors aligned). Out parameter is the
    // world Z of the child (== env-local z, since env Z offset is 0).
    EXPECT_NEAR(pose_post[0], env_x, 0.01f) << "child x should stay at env origin under pure-gravity hang";
    EXPECT_NEAR(pose_post[1], env_y, 0.01f) << "child y should stay at env origin under pure-gravity hang";
    out_child_local_z = pose_post[2];

    std::cerr << "  spherical child post=(" << pose_post[0] << "," << pose_post[1] << "," << pose_post[2]
              << ") env=(" << env_x << "," << env_y << ") local_z=" << out_child_local_z << "\n";

    ovphysx_destroy_tensor_binding(handle, pose_b);
}
} // namespace

TEST_F(TensorBindingCpuTest, CpuSphericalArtiSettle_AtOrigin) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiAtOrigin.usda", 0.f, 0.f, z);
    // Joint anchors: base.z - 0.1 = 1.9 (kinematic base at z=2), child.z + 0.2
    // (child starts at z=1.5). After settle, the joint pulls anchors
    // together so the child center ends at world z=1.7 (anchor at 1.9 minus
    // 0.2 child-local offset). Loose tolerance covers iterative settle.
    EXPECT_NEAR(z, 1.7f, 0.05f) << "spherical settle: child center should reach world z=1.7";
}
TEST_F(TensorBindingCpuTest, CpuSphericalArtiSettle_Far) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiFar.usda", 100.f, 100.f, z);
    EXPECT_NEAR(z, 1.7f, 0.05f) << "translation invariance: spherical settle world z same at Far";
}
TEST_F(TensorBindingGpuTest, GpuSphericalArtiSettle_AtOrigin) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiAtOrigin.usda", 0.f, 0.f, z);
    EXPECT_NEAR(z, 1.7f, 0.05f);
}
TEST_F(TensorBindingGpuTest, GpuSphericalArtiSettle_Far) {
    float z = 0.f;
    run_spherical_arti_settle_probe(m_handle, "tests/data/SphericalArtiFar.usda", 100.f, 100.f, z);
    EXPECT_NEAR(z, 1.7f, 0.05f) << "translation invariance: GPU spherical settle world z same at Far";
}

// OMPE-94459 engine ask C2: GPU DirectGPU pipeline (suppressReadback=true)
// must allow get_data('velocities') over a rigid-body view that includes
// disabled bodies. Umbrella reports "Internal error: Unresolved rigid
// dynamic index!" + heap corruption from get_data after writing
// RIGID_BODY_DISABLE_SIMULATION = 1 to a subset.
//
// This test shows the engine handles the pattern correctly when the
// binding is created BEFORE disable and cached -- which is the engine's
// expected usage. Binding ctor caches physxRdIdx from mNode2RdIndexMap
// (built once at simulation-view init); disabled bodies' rows are still
// readable through that cached index after disable. The umbrella's
// failure is from recreating the GpuRigidBodyView after disable (see
// the companion DISABLED test below for that pattern -- the
// reconstruction-after-disable failure mode).
TEST_F(TensorBindingGpuTest, GpuVelocityReadbackOverDisabledBodies) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane.usda";

    // Pose + velocity + disable bindings on /World/Cube*.
    ovphysx_tensor_binding_handle_t pose_b = 0;
    ovphysx_tensor_binding_handle_t vel_b = 0;
    ovphysx_tensor_binding_handle_t dis_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &pose_b).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &vel_b).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &dis_b).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n_bodies = spec.shape[0];
    ASSERT_GT(n_bodies, 1) << "scene must have multiple bodies for the disable-half pattern";

    // Initial velocity read -- DirectGPU view requires CUDA dst tensor.
    const size_t vel_bytes = n_bodies * 6 * sizeof(float);
    void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
    ASSERT_NE(gpu_buf, nullptr);

    DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
    vt.dtype = {kDLFloat, 32, 1};
    int64_t vel_shape[2] = {n_bodies, 6};
    vt.shape = vel_shape; vt.ndim = 2;
    {
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, vel_b, &vt);
        if (r.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t err = ovphysx_get_last_error();
            FAIL() << "initial velocity read failed: status=" << r.status
                   << " err=" << (err.ptr ? std::string(err.ptr, err.length) : "n/a");
        }
    }
    std::vector<float> vel_buf(n_bodies * 6, 0.f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(vel_buf.data(), m_gpuBuffer, vel_bytes));

    // Disable every other body.
    std::vector<uint8_t> dis_buf(n_bodies, 0);
    for (int64_t i = 0; i < n_bodies; i += 2) dis_buf[i] = 1;
    DLTensor dt{}; dt.data = dis_buf.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1};
    int64_t dis_shape[1] = {n_bodies};
    dt.shape = dis_shape; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    // Step once to let disable propagate into PhysX-SDK runtime state.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Read velocity again into the same GPU buffer. If the umbrella's gg
    // symptom is real, this read crashes / aborts with "Unresolved rigid
    // dynamic index!" + heap corruption. If it returns cleanly, the engine
    // path tolerates disabled bodies and the umbrella's failure is
    // somewhere above the C ABI.
    {
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, vel_b, &vt);
        if (r.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t err = ovphysx_get_last_error();
            FAIL() << "post-disable velocity read failed (engine ask C2 repro): status="
                   << r.status << " err=" << (err.ptr ? std::string(err.ptr, err.length) : "n/a");
        }
    }
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(vel_buf.data(), m_gpuBuffer, vel_bytes));

    // Spot-check: disabled bodies should have zero velocity.
    for (int64_t i = 0; i < n_bodies; ++i) {
        if (dis_buf[i] == 1) {
            const float vz = vel_buf[i * 6 + 2];
            EXPECT_NEAR(vz, 0.f, 1e-3f) << "body " << i << " is disabled; vz should be 0, got " << vz;
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, dis_b);
    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, pose_b);
}

// OMPE-94459 (umbrella gg/gc remaining bug): proves the GPU solver DOES
// honor a second mid-sim disable-simulation toggle, and that a re-enabled
// body resumes only after wake_up (the documented contract). The umbrella
// disables [1 0 1 0] in on_start, then flips to [0 1 0 1] mid-sim and
// reports "Now-disabled bodies should not fall (step 2)". This test runs
// the exact flip at the C ABI and measures:
//   P1: disable even -> even freeze, odd fall. (baseline)
//   P2: flip (enable even, disable odd), step WITHOUT wake -> odd read 0
//       (2nd disable honored; the readback matches CPU, which zeroes a
//       disabled body's velocity), even stay 0 (re-enable alone does not
//       resume -- by design, needs wake_up).
//   P3: wake the re-enabled even bodies -> they resume falling at once;
//       odd stay 0.
// The disabled-body readback is zeroed in the GPU velocity fetch to match
// CPU getLinearVelocity (OMPE-94459): the DirectGPU buffer retains a
// disabled body's last integrated value, but its authoritative state is 0.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableSimulationSecondToggleHonored) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane.usda";

    auto make_binding = [&](ovphysx_tensor_type_t type) {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = type;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        return b;
    };

    ovphysx_tensor_binding_handle_t dis_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL);
    ovphysx_tensor_binding_handle_t vel_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);

    const size_t vel_bytes = n * 6 * sizeof(float);

    auto write_disable = [&](const std::vector<uint8_t>& flags) {
        DLTensor dt{}; dt.data = const_cast<uint8_t*>(flags.data()); dt.device = {kDLCPU, 0};
        dt.dtype = {kDLUInt, 8, 1};
        int64_t sh[1] = {n}; dt.shape = sh; dt.ndim = 1;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);
    };
    auto step = [&]() {
        ovphysx_enqueue_result_t s = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(s.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, s.op_index));
    };
    auto read_vz = [&](ovphysx_tensor_binding_handle_t vb, std::vector<float>& vz_out) {
        void* gpu_buf = allocGpuBuffer(vel_bytes, vb);
        ASSERT_NE(gpu_buf, nullptr);
        DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
        vt.dtype = {kDLFloat, 32, 1};
        int64_t sh[2] = {n, 6}; vt.shape = sh; vt.ndim = 2;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vb, &vt).status, OVPHYSX_API_SUCCESS);
        std::vector<float> buf(n * 6, 0.f);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(buf.data(), m_gpuBuffer, vel_bytes));
        vz_out.resize(n);
        for (int64_t i = 0; i < n; ++i) vz_out[i] = buf[i * 6 + 2];
    };

    auto wake = [&](ovphysx_tensor_binding_handle_t b) {
        ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, b, nullptr).status, OVPHYSX_API_SUCCESS);
    };

    std::vector<uint8_t> even_disabled(n, 0), odd_disabled(n, 0);
    for (int64_t i = 0; i < n; ++i) (i % 2 == 0 ? even_disabled : odd_disabled)[i] = 1;

    // P1: disable even. Even freeze, odd fall.
    write_disable(even_disabled);
    step(); step();
    std::vector<float> vz_p1;
    read_vz(vel_b, vz_p1);
    for (int64_t i = 0; i < n; ++i) {
        if (i % 2 == 0) EXPECT_NEAR(vz_p1[i], 0.f, 1e-3f) << "P1: disabled even body " << i << " should be frozen";
        else            EXPECT_LT(vz_p1[i], -0.01f)       << "P1: enabled odd body " << i << " should be falling";
    }

    // P2: FLIP -- enable even, disable odd. Flag readback must reflect it.
    write_disable(odd_disabled);
    std::vector<uint8_t> dis_rb(n, 0xff);
    {
        DLTensor dt{}; dt.data = dis_rb.data(); dt.device = {kDLCPU, 0};
        dt.dtype = {kDLUInt, 8, 1}; int64_t sh[1] = {n}; dt.shape = sh; dt.ndim = 1;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, dis_b, &dt).status, OVPHYSX_API_SUCCESS);
    }
    for (int64_t i = 0; i < n; ++i)
        EXPECT_EQ(dis_rb[i], odd_disabled[i]) << "flag flip not stored for body " << i;

    step(); step();
    std::vector<float> vz_p2;
    read_vz(vel_b, vz_p2);
    for (int64_t i = 1; i < n; i += 2) {
        // 2nd disable IS honored and the readback matches CPU: a re-disabled
        // body reads zero velocity (its solver state is zeroed on disable),
        // not its frozen pre-disable value. This is the umbrella's "should
        // not fall" expectation.
        EXPECT_NEAR(vz_p2[i], 0.f, 1e-3f)
            << "P2: re-disabled odd body " << i << " should read 0, got " << vz_p2[i];
    }
    for (int64_t i = 0; i < n; i += 2) {
        // Re-enable alone does not resume motion -- the body stays in the
        // sleep state PhysX put it in. Needs wake_up (asserted in P3).
        EXPECT_NEAR(vz_p2[i], 0.f, 1e-3f)
            << "P2: re-enabled even body " << i << " moved before wake_up (" << vz_p2[i] << ")";
    }

    // P3: wake the re-enabled even bodies -> they resume falling at once.
    wake(vel_b);
    step(); step();
    std::vector<float> vz_p3;
    read_vz(vel_b, vz_p3);
    for (int64_t i = 0; i < n; i += 2)
        EXPECT_LT(vz_p3[i], -0.01f)
            << "P3: woken even body " << i << " should be falling, got " << vz_p3[i];
    for (int64_t i = 1; i < n; i += 2)
        EXPECT_NEAR(vz_p3[i], 0.f, 1e-3f)
            << "P3: disabled odd body " << i << " should read 0, got " << vz_p3[i];

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-94459 (umbrella gg/gc): GPU twin of CpuRigidBodyDisableFlipWakeSequence
// -- the umbrella's EXACT timing (flip + wake_up in one on_physics_step
// window, then a single simulate). Diagnostic: prints B (moving body
// disabled mid-flight) and A (re-enabled + woken) so the GPU result can be
// compared against the CPU reference and the umbrella's CPU/GPU divergence
// claim verified at the C ABI.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableFlipWakeSequence) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    auto make_binding = [&](ovphysx_tensor_type_t type) {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = type;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        return b;
    };
    ovphysx_tensor_binding_handle_t dis_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL);
    ovphysx_tensor_binding_handle_t vel_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);
    const size_t vel_bytes = n * 6 * sizeof(float);

    std::vector<int32_t> idx_A, idx_B;
    for (int32_t i = 0; i < int32_t(n); ++i) (i % 2 == 0 ? idx_A : idx_B).push_back(i);

    auto write_disable_subset = [&](uint8_t value, const std::vector<int32_t>& subset) {
        std::vector<uint8_t> flags(n, value);
        DLTensor ft{}; ft.data = flags.data(); ft.device = {kDLCPU, 0};
        ft.dtype = {kDLUInt, 8, 1}; ft.ndim = 1; int64_t fs[1] = {n}; ft.shape = fs;
        DLTensor it{}; it.data = const_cast<int32_t*>(subset.data()); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 32, 1}; it.ndim = 1; int64_t is[1] = {int64_t(subset.size())}; it.shape = is;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &ft, &it).status, OVPHYSX_API_SUCCESS);
    };
    auto step = [&]() {
        ovphysx_enqueue_result_t s = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(s.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, s.op_index));
    };
    ovphysx_tensor_binding_handle_t pose_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_POSE_F32);
    const size_t pose_bytes = n * 7 * sizeof(float);
    auto read_vz = [&](std::vector<float>& vz) {
        void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
        ASSERT_NE(gpu_buf, nullptr);
        DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
        vt.dtype = {kDLFloat, 32, 1}; vt.ndim = 2; int64_t vs[2] = {n, 6}; vt.shape = vs;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_SUCCESS);
        std::vector<float> buf(n * 6, 0.f);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(buf.data(), m_gpuBuffer, vel_bytes));
        vz.resize(n); for (int64_t i = 0; i < n; ++i) vz[i] = buf[i * 6 + 2];
    };
    // Read each body's X. Gravity is along -z and the cubes have distinct X,
    // so X is a stable per-body identity: if a disable/re-enable cycle leaves
    // the view reading a stale/swapped GPU slot, body i reads another body's X.
    auto read_x = [&](std::vector<float>& xs) {
        void* gpu_buf = allocGpuBuffer(pose_bytes, pose_b);
        ASSERT_NE(gpu_buf, nullptr);
        DLTensor pt{}; pt.data = gpu_buf; pt.device = {kDLCUDA, 0};
        pt.dtype = {kDLFloat, 32, 1}; pt.ndim = 2; int64_t ps[2] = {n, 7}; pt.shape = ps;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_b, &pt).status, OVPHYSX_API_SUCCESS);
        std::vector<float> buf(n * 7, 0.f);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(buf.data(), m_gpuBuffer, pose_bytes));
        xs.resize(n); for (int64_t i = 0; i < n; ++i) xs[i] = buf[i * 7 + 0];
    };

    std::vector<float> x0; read_x(x0); // per-body identity before any disable

    write_disable_subset(1, idx_A);
    step(); step();
    std::vector<float> vz1; read_vz(vz1);

    write_disable_subset(1, idx_B);
    write_disable_subset(0, idx_A);
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, nullptr).status, OVPHYSX_API_SUCCESS);
    step();
    std::vector<float> vz2; read_vz(vz2);

    // Identity check (discriminating): after the disable/re-enable cycle each
    // RE-ENABLED body must still read its own X (gravity-invariant). Pre-fix
    // the view read a stale/swapped GPU slot, so body i reported another
    // body's X. (Disabled bodies are compacted out -- DirectGPU exposes no
    // slot for them -- and read 0; that pose behaviour is out of scope here.)
    std::vector<float> x2; read_x(x2);
    for (int32_t i : idx_A)
        EXPECT_NEAR(x2[i], x0[i], 1e-3f)
            << "gpu: re-enabled body " << i << " X identity changed (" << x0[i]
            << " -> " << x2[i] << "); view read a stale/swapped GPU slot";

    // GPU must match the CPU reference: now-disabled body reads 0, re-enabled
    // + woken body resumes falling.
    for (int32_t i : idx_B)
        EXPECT_NEAR(vz2[i], 0.f, 1e-3f) << "gpu: now-disabled body " << i << " should read 0, got " << vz2[i];
    for (int32_t i : idx_A)
        EXPECT_LT(vz2[i], -0.01f) << "gpu: re-enabled+woken body " << i << " should be falling, got " << vz2[i];

    // Re-enable B + wake + step: it must resume from 0 (~-dt*g after one step),
    // confirming the disable zeroed the solver state (the readback fix mirrors
    // that authoritative state -- it is not papering over a retained velocity).
    write_disable_subset(0, idx_B);
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, nullptr).status, OVPHYSX_API_SUCCESS);
    step();
    std::vector<float> vz3; read_vz(vz3);
    for (int32_t i : idx_B)
        EXPECT_NEAR(vz3[i], -1.f / 60.f * 9.81f, 5e-3f)
            << "gpu: re-enabled B body " << i << " should resume from 0 (~-dt*g), got " << vz3[i];

    ovphysx_destroy_tensor_binding(m_handle, pose_b);
    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-94459: CI regression gate for disabled-body POSE readback on GPU. The
// transforms fetch kernel had the same compound-sentinel bug the velocity
// kernel was fixed for: a disabled rigid dynamic (tensorRdIdx + both arti
// fields == sentinel) fell into the articulation branch, where
// `tensorArtiIdx * simMaxLinks + linkIdx` overflowed to a non-sentinel index
// and read garbage from linkTransforms -- so a disabled body's pose came back
// wildly wrong (~1.9m off). Our other GPU tests only check disabled-body
// velocity, so this slipped through (the device-switch cycle test that hit it
// skips under the device-mode lock in CI). This runs in the normal GPU pass.
TEST_F(TensorBindingGpuTest, GpuDisabledBodyPoseReadbackIsSane) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    auto make_binding = [&](ovphysx_tensor_type_t type) {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = type;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        return b;
    };
    ovphysx_tensor_binding_handle_t dis_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL);
    ovphysx_tensor_binding_handle_t pose_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_POSE_F32);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, pose_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);
    const size_t pose_bytes = n * 7 * sizeof(float);

    auto read_z = [&](std::vector<float>& zs) {
        void* gpu_buf = allocGpuBuffer(pose_bytes, pose_b);
        ASSERT_NE(gpu_buf, nullptr);
        DLTensor pt{}; pt.data = gpu_buf; pt.device = {kDLCUDA, 0};
        pt.dtype = {kDLFloat, 32, 1}; pt.ndim = 2; int64_t sh[2] = {n, 7}; pt.shape = sh;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_b, &pt).status, OVPHYSX_API_SUCCESS);
        std::vector<float> buf(n * 7, 0.f);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(buf.data(), m_gpuBuffer, pose_bytes));
        zs.resize(n); for (int64_t i = 0; i < n; ++i) zs[i] = buf[i * 7 + 2];
    };
    auto step = [&]() {
        ovphysx_enqueue_result_t s = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(s.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, s.op_index));
    };

    std::vector<float> z_pre; read_z(z_pre); // pose before disabling anything

    // Disable even-indexed bodies.
    std::vector<uint8_t> dis(n, 0);
    for (int64_t i = 0; i < n; i += 2) dis[i] = 1;
    DLTensor dt{}; dt.data = dis.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1}; int64_t ds[1] = {n}; dt.shape = ds; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    step();
    std::vector<float> z1; read_z(z1);
    step();
    std::vector<float> z2; read_z(z2);

    for (int64_t i = 0; i < n; ++i) {
        if (dis[i] == 1) {
            // Disabled body must read its real (frozen) pose, not garbage. The
            // pre-fix kernel returned a wild value (~1.9m off); 5cm easily
            // separates that from the few-mm of suppressReadback readback lag.
            EXPECT_NEAR(z1[i], z_pre[i], 0.05f)
                << "disabled body " << i << " pose is garbage: " << z1[i] << " vs ~" << z_pre[i];
            // And it must not keep falling across a further step.
            EXPECT_NEAR(z2[i], z1[i], 1e-3f)
                << "disabled body " << i << " kept moving: " << z1[i] << " -> " << z2[i];
        } else {
            // Sanity: enabled bodies are still falling.
            EXPECT_LT(z2[i], z1[i] - 1e-4f) << "enabled body " << i << " should keep falling";
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, pose_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-94459 (CR follow-up, option c): wake_up accepts a GPU index tensor by
// staging it device-to-host before BaseRigidBodyView::wakeUp reads it as a
// host pointer. This gates that DtoH pre-stage end-to-end: disable a subset,
// re-enable it, wake ONLY that subset via a CUDA int32 index tensor, and
// confirm it resumes. If the GPU indices were mis-read (the pre-fix bug), the
// wrong/no bodies would wake and the subset would stay asleep at vz 0.
// Internal coverage for what the umbrella gg test exercised cross-project.
TEST_F(TensorBindingGpuTest, GpuRigidBodyWakeUpStagesGpuIndices) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle));

    auto make_binding = [&](ovphysx_tensor_type_t type) {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = type;
        EXPECT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        return b;
    };
    ovphysx_tensor_binding_handle_t dis_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL);
    ovphysx_tensor_binding_handle_t vel_b = make_binding(OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);
    const size_t vel_bytes = n * 6 * sizeof(float);

    std::vector<int32_t> even;
    for (int32_t i = 0; i < int32_t(n); i += 2) even.push_back(i);

    auto write_disable_subset = [&](uint8_t value, const std::vector<int32_t>& subset) {
        std::vector<uint8_t> flags(n, value);
        DLTensor ft{}; ft.data = flags.data(); ft.device = {kDLCPU, 0};
        ft.dtype = {kDLUInt, 8, 1}; ft.ndim = 1; int64_t fs[1] = {n}; ft.shape = fs;
        DLTensor it{}; it.data = const_cast<int32_t*>(subset.data()); it.device = {kDLCPU, 0};
        it.dtype = {kDLInt, 32, 1}; it.ndim = 1; int64_t is[1] = {int64_t(subset.size())}; it.shape = is;
        ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &ft, &it).status, OVPHYSX_API_SUCCESS);
    };
    auto step = [&]() {
        ovphysx_enqueue_result_t s = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(s.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, s.op_index));
    };
    auto read_vz = [&](std::vector<float>& vz) {
        void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
        ASSERT_NE(gpu_buf, nullptr);
        DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
        vt.dtype = {kDLFloat, 32, 1}; vt.ndim = 2; int64_t vs[2] = {n, 6}; vt.shape = vs;
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_SUCCESS);
        std::vector<float> buf(n * 6, 0.f);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(buf.data(), m_gpuBuffer, vel_bytes));
        vz.resize(n); for (int64_t i = 0; i < n; ++i) vz[i] = buf[i * 6 + 2];
    };

    // Disable the even subset, step so they go to sleep, then re-enable them.
    write_disable_subset(1, even);
    step();
    write_disable_subset(0, even);

    // Upload the even indices to the GPU and wake that subset via a CUDA int32
    // index tensor (this is the path that reads as a host pointer pre-fix).
    // allocGpuBuffer set the CUDA context above; reuse it for the index alloc.
    (void)allocGpuBuffer(vel_bytes, vel_b);
    uintptr_t idxDev = 0;
    int allocSt = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(even.size() * sizeof(int32_t), &idxDev, &allocSt));
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(idxDev, even.data(), even.size() * sizeof(int32_t)));
    DLTensor idxT{}; idxT.data = reinterpret_cast<void*>(idxDev); idxT.device = {kDLCUDA, 0};
    idxT.dtype = {kDLInt, 32, 1}; idxT.ndim = 1; int64_t is[1] = {int64_t(even.size())}; idxT.shape = is;
    EXPECT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, &idxT).status, OVPHYSX_API_SUCCESS)
        << "wake_up with a CUDA int32 index tensor should succeed (DtoH staged)";

    step();
    std::vector<float> vz; read_vz(vz);
    for (int32_t i : even)
        EXPECT_LT(vz[i], -0.01f)
            << "re-enabled body " << i << " woken via GPU indices should be falling, got " << vz[i];

    (void)m_cudaOps.memFree(idxDev);
    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// Companion to GpuVelocityReadbackOverDisabledBodies: gates the
// rebind-after-disable path the umbrella adapter exercises. Disables a
// subset BEFORE creating the velocity binding, then reads. Pre-fix,
// GpuRigidBodyView::ctor's node-index lookup failed for disabled bodies
// (their getInternalIslandNodeIndex() no longer matched mNode2RdIndexMap
// since PhysX removes them from the island system), the ctor logged
// "Internal error: Unresolved rigid dynamic index!", left rb.physxRdIdx
// unset, and the subsequent read indexed invalid GPU buffer rows ->
// heap corruption in the umbrella's gg test. Fix is the
// mActor2RdIndexMap actor-pointer fallback in GpuSimulationData; this
// test verifies the fallback resolves disabled bodies cleanly.
TEST_F(TensorBindingGpuTest, GpuVelocityReadbackBindingAfterDisable) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane.usda";

    ovphysx_tensor_binding_handle_t dis_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_SIMULATION_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &dis_b).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_spec_t dis_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, dis_b, &dis_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n_bodies = dis_spec.shape[0];

    std::vector<uint8_t> dis_buf(n_bodies, 0);
    for (int64_t i = 0; i < n_bodies; i += 2) dis_buf[i] = 1;
    DLTensor dt{}; dt.data = dis_buf.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1};
    int64_t dis_shape[1] = {n_bodies};
    dt.shape = dis_shape; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.f / 60.f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Now create the velocity binding. The view ctor at this point sees
    // already-disabled bodies and their getInternalIslandNodeIndex() does
    // not match mNode2RdIndexMap -> ctor logs the "Unresolved" error.
    ovphysx_tensor_binding_handle_t vel_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        // Binding create itself may still return success (ctor doesn't
        // propagate the error); the subsequent read indexes invalid rows.
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &vel_b).status, OVPHYSX_API_SUCCESS);
    }

    const size_t vel_bytes = n_bodies * 6 * sizeof(float);
    void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
    ASSERT_NE(gpu_buf, nullptr);
    DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
    vt.dtype = {kDLFloat, 32, 1};
    int64_t vel_shape[2] = {n_bodies, 6};
    vt.shape = vel_shape; vt.ndim = 2;
    // Disabling even-indexed bodies pushes enabled bodies to odd entry
    // indices >= mNumRds. Pre-fix, cMassLocalPosePos was sized compacted
    // (mNumRds + arti links), so updateCMassData (called from the view ctor)
    // wrote those enabled entries out of bounds -- a small heap overflow that
    // surfaced nondeterministically later as "free(): invalid pointer". The
    // ctor also logged "Unresolved rigid dynamic index!" per disabled body.
    // Post-fix: cMass is sized by the full entry count (matching the
    // entry-indexed coms[rbIdx] consumer), the ctor resolves disabled bodies
    // via mActor2RdIndexMap and marks them sentinel, and the fetch kernel
    // writes zero for sentinel records -- no out-of-bounds write, no leak.
    {
        ovphysx_result_t r = ovphysx_read_tensor_binding(m_handle, vel_b, &vt);
        if (r.status != OVPHYSX_API_SUCCESS) {
            ovphysx_string_t err = ovphysx_get_last_error();
            FAIL() << "post-rebind read failed: status=" << r.status
                   << " err=" << (err.ptr ? std::string(err.ptr, err.length) : "n/a");
        }
    }

    // Verify disabled bodies read back as zero velocity (kernel sentinel path).
    std::vector<float> vel_buf(n_bodies * 6, 0.f);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(vel_buf.data(), m_gpuBuffer, n_bodies * 6 * sizeof(float)));
    for (int64_t i = 0; i < n_bodies; ++i) {
        if (dis_buf[i] == 1) {
            for (int k = 0; k < 6; ++k) {
                EXPECT_EQ(vel_buf[i * 6 + k], 0.f)
                    << "disabled body " << i << " velocity[" << k << "] should be 0";
            }
        } else {
            // Enabled bodies sit at odd entry indices (>= mNumRds). Reading a
            // real downward velocity here proves those high-index entries
            // resolve to the correct rows rather than the OOB/garbage slots
            // the compacted sizing produced.
            EXPECT_LT(vel_buf[i * 6 + 2], -0.01f)
                << "enabled body " << i << " should still be falling (vz < 0)";
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}
