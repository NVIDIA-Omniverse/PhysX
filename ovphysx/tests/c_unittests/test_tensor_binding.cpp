// SPDX-FileCopyrightText: Copyright (c) 2025-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BINDING-DEVICE-001
 * @covers AC-1 AC-2 AC-3 AC-4 AC-5
 *
 * @implements REQ-CAPI-OBJECTTYPE-001
 * @covers AC-2 AC-3
 * @maps_to TEST-CAPI-OBJECTTYPE-001
 */
// DEPRECATED (tensor-binding-deprecation): a deprecated tensor-binding test; removed with the binding.

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

    omni::physics::AttachHandle lastStageId() const
    {
        return sLastStageId;
    }

private:
    static void CARB_ABI intercept(omni::physics::AttachHandle stageId)
    {
        ++sCallCount;
        sLastStageId = stageId;
        if (sOriginal)
            sOriginal(stageId);
    }

    omni::physics::tensors::TensorApi& mTensorApi;
    void(CARB_ABI* mOriginal)(omni::physics::AttachHandle) = nullptr;

    static inline void(CARB_ABI* sOriginal)(omni::physics::AttachHandle) = nullptr;
    static inline int sCallCount = 0;
    static inline omni::physics::AttachHandle sLastStageId = 0;
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

TEST_F(TensorBindingCpuTest, NativeDeviceReportsCpu)
{
    ovphysx_usd_handle_t usdHandle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usdHandle));

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube1");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
    ovphysx_tensor_binding_handle_t binding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    DLDevice device{ kDLExtDev, -1 };
    EXPECT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, binding, &device).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(device.device_type, kDLCPU);
    EXPECT_EQ(device.device_id, 0);

    EXPECT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, binding, nullptr).status,
              OVPHYSX_API_INVALID_ARGUMENT);
    EXPECT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, OVPHYSX_INVALID_HANDLE, &device).status,
              OVPHYSX_API_NOT_FOUND);

    DLDevice invalidHandleDevice{ kDLExtDev, 77 };
    EXPECT_EQ(ovphysx_get_tensor_binding_native_device(OVPHYSX_INVALID_HANDLE, binding, &invalidHandleDevice).status,
              OVPHYSX_API_ERROR);
    EXPECT_EQ(invalidHandleDevice.device_type, kDLExtDev);
    EXPECT_EQ(invalidHandleDevice.device_id, 77);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// ovphysx_reset_stage must release the tensor SimulationBackend's per-stage data, or stale
// views persist across reset and reattach. The reset path calls resetStage(stageId) in
// detach_ovstage / unload_usd.
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

// Shape and read-only contract of the articulation mass-center tensors (OMPE-94459).
// Both WORLD and LOCAL variants must have shape [N, 3] and reject writes. The COM
// values themselves are covered by the umbrella tensor test.
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

        // These tensors are read-only, so the write must be rejected.
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

    // Step once so the articulation inverse dynamics cache is live, then read.
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

    // Centroidal momentum is read-only, so the write must be rejected.
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

// Per-body disable_simulation flag round trip (OMPE-94459). IRigidBodyView::set/getDisableSimulations
// toggles PxActorFlag::eDISABLE_SIMULATION on the PxRigidActor, so the flag takes effect on the
// next solver step.
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
    // DISABLE_SIMULATION expects uint8/bool and the engine rejects float32.
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

TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableGravityRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 1);
    EXPECT_EQ(spec.dtype.code, kDLUInt);
    EXPECT_EQ(spec.dtype.bits, 8);
    ASSERT_GT(spec.shape[0], 0);

    const int64_t n = spec.shape[0];
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
            << "body " << i << " disable-gravity round-trip";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableGravitySuppressesFall) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t pose_binding = 0;
    ovphysx_tensor_binding_handle_t grav_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &grav_binding).status, OVPHYSX_API_SUCCESS);
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

    std::vector<float> poses(n * 7, 0.0f);
    DLTensor pose_t{};
    pose_t.data = poses.data();
    pose_t.device = {kDLCPU, 0};
    pose_t.ndim = 2;
    pose_t.dtype = {kDLFloat, 32, 1};
    int64_t pose_shape[2] = {n, 7};
    pose_t.shape = pose_shape;

    std::vector<float> vels(n * 6, 0.0f);
    DLTensor vel_t{};
    vel_t.data = vels.data();
    vel_t.device = {kDLCPU, 0};
    vel_t.ndim = 2;
    vel_t.dtype = {kDLFloat, 32, 1};
    int64_t vel_shape[2] = {n, 6};
    vel_t.shape = vel_shape;

    ovphysx_tensor_binding_handle_t vel_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &vel_binding).status, OVPHYSX_API_SUCCESS);
    }

    // Capture starting pose before any explicit step.
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    const float z_initial = poses[2];

    // Let bodies fall briefly, then disable gravity and zero velocity (PhysX keeps
    // coasting with existing velocity unless it is cleared).
    for (int i = 0; i < 5; ++i)
        step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    const float z_falling = poses[2];

    std::vector<uint8_t> flags(static_cast<size_t>(n), 1);
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    std::fill(vels.begin(), vels.end(), 0.0f);
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, vel_binding, &vel_t, nullptr).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    const float z_disabled = poses[2];

    for (int i = 0; i < 20; ++i)
        step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_binding, &vel_t).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(vels[2], 0.0f, 1e-3f) << "Gravity-disabled body should stay at zero velocity";
    EXPECT_NEAR(poses[2], z_disabled, 0.08f) << "Gravity-disabled body should not drift after velocity zeroed";

    std::fill(flags.begin(), flags.end(), 0);
    flag_t.data = flags.data();
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, pose_binding, nullptr).status, OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 10; ++i)
        step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_binding, &vel_t).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    EXPECT_LT(vels[2], -0.5f) << "Body should fall after re-enabling gravity";
    EXPECT_LT(poses[2], z_disabled - 0.05f) << "Body should fall after re-enabling gravity";
    EXPECT_LT(z_falling, z_initial) << "Sanity: body fell before gravity was disabled";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, vel_binding).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, pose_binding).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, grav_binding).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingCpuTest, CpuArticulationDisableGravityRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 2);
    EXPECT_EQ(spec.dtype.code, kDLUInt);
    EXPECT_EQ(spec.dtype.bits, 8);
    ASSERT_GT(spec.shape[0], 0);
    ASSERT_GT(spec.shape[1], 0);

    const int64_t n = spec.shape[0];
    const int64_t l = spec.shape[1];
    const int64_t total = n * l;
    std::vector<uint8_t> written(static_cast<size_t>(total), 0);
    for (int64_t i = 0; i < total; ++i)
        written[static_cast<size_t>(i)] = static_cast<uint8_t>(i % 2 == 0 ? 1 : 0);

    DLTensor tensor = {};
    tensor.data = written.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {n, l};
    tensor.shape = shape;

    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(static_cast<size_t>(total), 0xff);
    tensor.data = readback.data();
    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < total; ++i)
        EXPECT_EQ(static_cast<int>(readback[static_cast<size_t>(i)]),
                  static_cast<int>(written[static_cast<size_t>(i)]))
            << "link flag " << i;

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// Mixed link-count sentinel: [N, L] articulation disable-gravity reads must
// zero-pad columns beyond each articulation's numLinks (L = maxLinks). Prefill
// the dst with 0xff so an uncleared pad is distinguishable from a real 0 flag.
TEST_F(TensorBindingCpuTest, CpuArticulationDisableGravityZeroPadsShortRows) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/mixed_link_count_articulations.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_string_t paths[] = {
        OVPHYSX_LITERAL("/World/articulation_short"),
        OVPHYSX_LITERAL("/World/articulation_long"),
    };
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.prim_paths = paths;
    desc.prim_paths_count = 2;
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 2);
    ASSERT_EQ(spec.shape[0], 2) << "Expected short + long articulations";
    ASSERT_EQ(spec.shape[1], 3) << "Expected L = maxLinks = 3";

    ovphysx_articulation_metadata_t meta_short{};
    ovphysx_articulation_metadata_t meta_long{};
    {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/articulation_short");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_get_articulation_metadata(m_handle, b, &meta_short).status, OVPHYSX_API_SUCCESS);
        ovphysx_destroy_tensor_binding(m_handle, b);
    }
    {
        ovphysx_tensor_binding_handle_t b = 0;
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/articulation_long");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &b).status, OVPHYSX_API_SUCCESS);
        ASSERT_EQ(ovphysx_get_articulation_metadata(m_handle, b, &meta_long).status, OVPHYSX_API_SUCCESS);
        ovphysx_destroy_tensor_binding(m_handle, b);
    }
    ASSERT_EQ(meta_short.body_count, 2);
    ASSERT_EQ(meta_long.body_count, 3);

    const int64_t n = spec.shape[0];
    const int64_t l = spec.shape[1];
    std::vector<uint8_t> readback(static_cast<size_t>(n * l), 0xff);
    DLTensor tensor{};
    tensor.data = readback.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {n, l};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    // Row 0 = short (2 links): columns 0..1 written; column 2 must be 0 (not 0xff).
    EXPECT_LE(static_cast<int>(readback[0]), 1) << "short link0 should be a 0/1 flag";
    EXPECT_LE(static_cast<int>(readback[1]), 1) << "short link1 should be a 0/1 flag";
    EXPECT_EQ(static_cast<int>(readback[2]), 0) << "short row pad column must be zero, not sentinel";
    // Row 1 = long (3 links): all three columns written.
    EXPECT_LE(static_cast<int>(readback[3]), 1);
    EXPECT_LE(static_cast<int>(readback[4]), 1);
    EXPECT_LE(static_cast<int>(readback[5]), 1);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingCpuTest, CpuArticulationDriveTypeRead) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8;

    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 2);
    EXPECT_EQ(spec.dtype.code, kDLUInt);
    EXPECT_EQ(spec.dtype.bits, 8);
    ASSERT_GT(spec.shape[0], 0);
    ASSERT_GT(spec.shape[1], 0);

    const int64_t n = spec.shape[0];
    const int64_t d = spec.shape[1];
    // Sentinel-fill so an unwritten byte is distinguishable from a real eNone(0).
    std::vector<uint8_t> readback(static_cast<size_t>(n * d), 0xff);
    DLTensor tensor{};
    tensor.data = readback.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {n, d};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    // DofDriveType: 0 = none, 1 = force, 2 = acceleration. The fixture authors
    // PhysicsDriveAPI on its angular DOFs, so at least one DOF must report a
    // driven type. The exact enum mapping is pinned separately by
    // CpuArticulationDriveTypeExactMapping, which authors each type explicitly.
    int drivenCount = 0;
    for (int64_t i = 0; i < n * d; ++i) {
        const int value = static_cast<int>(readback[static_cast<size_t>(i)]);
        EXPECT_LE(value, 2) << "drive type at " << i << " is outside {0,1,2}";
        if (value != 0)
            ++drivenCount;
    }
    EXPECT_GT(drivenCount, 0) << "fixture authors drives, so some DOF should report a driven type";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// Pins the exact DofDriveType mapping. The fixture authors one DOF per drive
// type, so an off-by-one or swapped force/acceleration mapping fails here rather
// than passing an in-range {0,1,2} check.
TEST_F(TensorBindingCpuTest, CpuArticulationDriveTypeExactMapping) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/drive_type_articulation.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation_drives");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 2);
    ASSERT_EQ(spec.shape[0], 1) << "one articulation";
    ASSERT_EQ(spec.shape[1], 3) << "joint1 + joint2 + joint3 = 3 revolute DOFs";

    std::vector<uint8_t> readback(3, 0xff);
    DLTensor tensor{};
    tensor.data = readback.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {1, 3};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    // Solver DOF order follows the joint chain: joint1, joint2, joint3.
    EXPECT_EQ(static_cast<int>(readback[0]), 1) << "joint1 authors type=force -> eForce";
    EXPECT_EQ(static_cast<int>(readback[1]), 2) << "joint2 authors type=acceleration -> eAcceleration";
    EXPECT_EQ(static_cast<int>(readback[2]), 0) << "joint3 authors no drive -> eNone";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// A read-only tensor type must reject writes with the documented "read-only"
// error rather than falling through to "unsupported tensor type", on both the
// indexed and masked write paths.
TEST_F(TensorBindingCpuTest, CpuArticulationDriveTypeRejectsWrites) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    const int64_t d = spec.shape[1];

    // A well-formed uint8 source: rejection must come from the read-only
    // classification, not from a dtype or shape mismatch.
    std::vector<uint8_t> src(static_cast<size_t>(n * d), 0);
    DLTensor tensor{};
    tensor.data = src.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {n, d};
    tensor.shape = shape;

    ovphysx_result_t writeResult = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    EXPECT_EQ(writeResult.status, OVPHYSX_API_INVALID_ARGUMENT) << "write must be rejected";
    EXPECT_NE(std::string(ovphysx_get_last_error().ptr).find("read-only"), std::string::npos)
        << "rejection should name the read-only contract, not 'unsupported tensor type'";

    std::vector<uint8_t> mask(static_cast<size_t>(n), 1);
    DLTensor maskTensor{};
    maskTensor.data = mask.data();
    maskTensor.device = {kDLCPU, 0};
    maskTensor.ndim = 1;
    maskTensor.dtype = {kDLUInt, 8, 1};
    int64_t maskShape[1] = {n};
    maskTensor.shape = maskShape;

    ovphysx_result_t maskedResult = ovphysx_write_tensor_binding_masked(m_handle, binding, &tensor, &maskTensor);
    EXPECT_EQ(maskedResult.status, OVPHYSX_API_INVALID_ARGUMENT) << "masked write must be rejected";
    EXPECT_NE(std::string(ovphysx_get_last_error().ptr).find("read-only"), std::string::npos)
        << "masked rejection should name the read-only contract";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

// Mixed DOF-count sentinel: [N, D] drive-type reads must zero-pad columns beyond
// each articulation's numDofs (D = maxDofs). The two articulations author
// different leading drive types, so row order is observable and a row swap fails
// here rather than satisfying the per-row checks. Prefill with 0xff so an
// uncleared pad is distinguishable from a real eNone(0).
TEST_F(TensorBindingCpuTest, CpuArticulationDriveTypeZeroPadsShortRows) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/drive_type_articulation.usda", usd_handle))
        << "Failed to load USD";

    // Short row first, so the padded row is row 0.
    ovphysx_string_t paths[] = {
        OVPHYSX_LITERAL("/World/articulation_one_dof"),
        OVPHYSX_LITERAL("/World/articulation_drives"),
    };
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.prim_paths = paths;
    desc.prim_paths_count = 2;
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_DRIVE_TYPE_U8;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(spec.ndim, 2);
    ASSERT_EQ(spec.shape[0], 2) << "one_dof + drives articulations";
    ASSERT_EQ(spec.shape[1], 3) << "D = maxDofs = 3 (the drives articulation)";

    std::vector<uint8_t> readback(6, 0xff);
    DLTensor tensor{};
    tensor.data = readback.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {2, 3};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    // Row 0 = articulation_one_dof: 1 real DOF authored "acceleration", then 2 pad
    // columns. The leading 2 also proves this row is not the 3-DOF articulation,
    // whose first DOF is "force" (1).
    EXPECT_EQ(static_cast<int>(readback[0]), 2) << "row 0 must be one_dof (acceleration), not a swapped row";
    EXPECT_EQ(static_cast<int>(readback[1]), 0) << "row 0 pad col 1 must be zero, not sentinel";
    EXPECT_EQ(static_cast<int>(readback[2]), 0) << "row 0 pad col 2 must be zero, not sentinel";

    // Row 1 = articulation_drives: force, acceleration, none. No padding.
    EXPECT_EQ(static_cast<int>(readback[3]), 1) << "row 1 joint1 -> eForce";
    EXPECT_EQ(static_cast<int>(readback[4]), 2) << "row 1 joint2 -> eAcceleration";
    EXPECT_EQ(static_cast<int>(readback[5]), 0) << "row 1 joint3 -> eNone";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, binding).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingCpuTest, CpuArticulationDisableGravitySuppressesFall) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/mixed_base_articulations.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t root_binding = 0;
    ovphysx_tensor_binding_handle_t grav_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation2");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &root_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation2");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &grav_binding).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_binding_handle_t rootvel_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation2");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &rootvel_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t grav_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, grav_binding, &grav_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = grav_spec.shape[0];
    const int64_t l = grav_spec.shape[1];
    ASSERT_EQ(n, 1);
    ASSERT_GT(l, 0);

    auto step_once = [&]() {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    };

    std::vector<float> root_pose(7, 0.0f);
    DLTensor root_t{};
    root_t.data = root_pose.data();
    root_t.device = {kDLCPU, 0};
    root_t.ndim = 2;
    root_t.dtype = {kDLFloat, 32, 1};
    int64_t root_shape[2] = {1, 7};
    root_t.shape = root_shape;

    std::vector<float> root_vel(6, 0.0f);
    DLTensor rootvel_t{};
    rootvel_t.data = root_vel.data();
    rootvel_t.device = {kDLCPU, 0};
    rootvel_t.ndim = 2;
    rootvel_t.dtype = {kDLFloat, 32, 1};
    int64_t rootvel_shape[2] = {1, 6};
    rootvel_t.shape = rootvel_shape;

    // Disable gravity before the first step. Stepping under gravity first would
    // leave the root coasting downward, so a stationary-height assertion could
    // be satisfied by that momentum rather than by gravity suppression.
    std::vector<uint8_t> flags(static_cast<size_t>(n * l), 1);
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 2;
    flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[2] = {n, l};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_binding, &root_t).status, OVPHYSX_API_SUCCESS);
    const float z0 = root_pose[2];

    for (int i = 0; i < 20; ++i)
        step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_binding, &root_t).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, rootvel_binding, &rootvel_t).status, OVPHYSX_API_SUCCESS);
    const float z_disabled = root_pose[2];
    EXPECT_NEAR(root_vel[2], 0.0f, 1e-3f) << "Gravity-disabled root should not accumulate downward velocity";
    EXPECT_NEAR(z_disabled, z0, 0.01f) << "Floating-base root Z should stay stable with gravity disabled";

    // Re-enable and compare against the height reached while disabled, so the
    // fall cannot be credited to drift accumulated before gravity was restored.
    std::fill(flags.begin(), flags.end(), 0);
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 20; ++i)
        step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_binding, &root_t).status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, rootvel_binding, &rootvel_t).status, OVPHYSX_API_SUCCESS);
    EXPECT_LT(root_vel[2], -0.5f) << "Root should gain downward velocity after re-enabling gravity";
    EXPECT_LT(root_pose[2], z_disabled - 0.05f) << "Floating-base root should fall after re-enabling gravity";

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, rootvel_binding).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, root_binding).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, grav_binding).status, OVPHYSX_API_SUCCESS);
}

// Writing dof-positions and then calling ovphysx_articulation_update_kinematic must
// propagate the new joint state into the link buffer without stepping the simulator
// (OMPE-94459).
TEST_F(TensorBindingCpuTest, CpuArticulationUpdateKinematicPropagatesDofToLinks) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t dof_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/articulation");
        d.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &dof_b).status, OVPHYSX_API_SUCCESS);
    }
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

    ASSERT_EQ(ovphysx_articulation_update_kinematic(m_handle, dof_b,
                                                    OVPHYSX_ARTICULATION_KINEMATIC_POSITION).status,
              OVPHYSX_API_SUCCESS);

    // Link poses read without a step must differ from the initial poses, since the
    // joint angles went from zero to 0.6.
    std::vector<float> links_after(N * L * 7, 0.0f);
    lt.data = links_after.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // At least one non-root link must have moved from its rest position.
    int moved = 0;
    for (int64_t k = 1; k < L; ++k) {  // Skip k=0, the root link does not move with the DOFs.
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

// ARTICULATION_DOF_DRIVE_MODEL exposes IArticulationView::set/getDofDriveModelProperties
// (OMPE-94459). Shape [N, D, 3]: (speedEffortGradient, maxActuatorVelocity,
// velocityDependentResistance).
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

    std::vector<float> data(spec.shape[0] * spec.shape[1] * spec.shape[2], 0.0f);
    DLTensor t{}; t.data = data.data(); t.device = {kDLCPU, 0};
    t.ndim = 3; t.dtype = {kDLFloat, 32, 1};
    int64_t shape[3] = {spec.shape[0], spec.shape[1], spec.shape[2]}; t.shape = shape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, b, &t).status, OVPHYSX_API_SUCCESS);

    for (int64_t i = 0; i < spec.shape[0]; ++i)
        for (int64_t j = 0; j < spec.shape[1]; ++j) {
            data[(i * spec.shape[1] + j) * 3 + 0] = 0.7f;
            data[(i * spec.shape[1] + j) * 3 + 1] = 1.2f;
            data[(i * spec.shape[1] + j) * 3 + 2] = 0.05f;
        }
    // The write succeeds, but the engine only applies values to DOFs with
    // PhysxDrivePerformanceEnvelopeAPI applied in USD. This asset has none, so the
    // values are dropped with a warning and only the C ABI is checked here. Value
    // round trips are covered by the umbrella JointPerformanceEnvelope test.
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, b, &t, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(data.size(), 0.0f);
    t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, b, &t).status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, b);
}

// Writing ARTICULATION_ROOT_POSE must propagate to the links across a step (OMPE-94459).
// applyCache(eROOT_TRANSFORM) is equivalent to setRootGlobalPose followed by
// updateKinematic(POSITION), so every link shifts with the root.
TEST_F(TensorBindingCpuTest, CpuArticulationRootTransformPropagates) {
    // Ant.usda is a multi-branch articulation, matching the umbrella
    // TestArticulationRootTransforms scenario.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/Ant.usda", usd_handle))
        << "Failed to load USD";

    // Settling step. PhysX expects a settled articulation before
    // applyCache(eROOT_TRANSFORM) propagates correctly.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

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

    // The shift is expressed relative to the current root pose.
    std::vector<float> root_initial(N * 7, 0.0f);
    DLTensor t{}; t.data = root_initial.data(); t.device = {kDLCPU, 0};
    t.ndim = 2; t.dtype = {kDLFloat, 32, 1};
    int64_t rshape[2] = {N, 7}; t.shape = rshape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_b, &t).status, OVPHYSX_API_SUCCESS);

    std::vector<float> link_initial(N * L * 7, 0.0f);
    DLTensor lt{}; lt.data = link_initial.data(); lt.device = {kDLCPU, 0};
    lt.ndim = 3; lt.dtype = {kDLFloat, 32, 1};
    int64_t lshape[3] = {N, L, 7}; lt.shape = lshape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // Root pose shifted by kShiftX in X with identity rotation.
    std::vector<float> root_new(N * 7, 0.0f);
    constexpr float kShiftX = 0.5f;  // Matches the umbrella's Z linspace(0, 1).
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

    // Read the root back before stepping to confirm the write took effect.
    std::vector<float> root_after_write(N * 7, 0.0f);
    t.data = root_after_write.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_b, &t).status, OVPHYSX_API_SUCCESS);
    std::cerr << "[root-prop trace] initial=" << root_initial[0] << "  set=" << root_new[0]
              << "  read_after_write=" << root_after_write[0]
              << "  (delta to set: " << (root_after_write[0] - root_new[0]) << ")\n";

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    std::vector<float> link_post(N * L * 7, 0.0f);
    lt.data = link_post.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, link_b, &lt).status, OVPHYSX_API_SUCCESS);

    // Every non-padding link's X must shift by kShiftX. Padding links (articulations
    // with fewer than L links) read back as zeros and are skipped.
    int checked = 0;
    for (int64_t i = 0; i < N; ++i) {
        for (int64_t k = 0; k < L; ++k) {
            const size_t base = (i * L + k) * 7;
            const float qw_initial = link_initial[base + 6];
            if (qw_initial == 0.0f) continue;  // zero-padded
            const float dx = link_post[base + 0] - link_initial[base + 0];
            EXPECT_NEAR(dx, kShiftX, 0.001f)  // Matches the umbrella's rtol=1e-3 and atol=1e-4.
                << "arti " << i << " link " << k
                << ": expected x-shift " << kShiftX << ", got " << dx;
            ++checked;
        }
    }
    EXPECT_GT(checked, 0) << "should have checked at least one link";

    ovphysx_destroy_tensor_binding(m_handle, root_b);
    ovphysx_destroy_tensor_binding(m_handle, link_b);
}

// Multi-env prismatic DOF velocity write/step/read on a cart-rail grid, mirroring the
// umbrella LinearDofVelocities scenario (OMPE-94459). With do_settle false the write
// happens before the first step, as in the umbrella. With do_settle true one step runs
// before the write, as in the passing single-articulation reference.
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

    // Values vary per env over -2.0..2.0 like the umbrella scenario. Uniform
    // values would mask per-articulation index-mapping bugs.
    std::vector<float> set_vel(N * D, 0.0f);
    for (int64_t i = 0; i < N; ++i) {
        const float v = -2.0f + 4.0f * float(i) / float(N - 1);
        for (int64_t j = 0; j < D; ++j) set_vel[i * D + j] = v;
    }
    DLTensor t{}; t.device = {kDLCPU, 0}; t.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {N, D}; t.shape = shape; t.ndim = 2; t.data = set_vel.data();

    // The umbrella write path always passes indices=[0..N-1] and never a
    // null-indices full write, so the indexed write path is exercised here.
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

// Variant with the PhysxSceneAPI attributes the umbrella applies on /physicsScene
// (OMPE-94459): enableGpuDynamics=0, broadPhaseType="MBP",
// enableSceneQuerySupport=0, timeStepsPerSecond=60. Isolates whether the scene
// settings trigger the umbrella's prismatic velocity decay.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaSceneAPI) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false, "tests/data/MultiCartRailUmbrella.usda");
}

// 42-env variant matching the umbrella's scenario count, with gravity direction
// (0,0,0) instead of (0,0,-1) at magnitude 0. Rules out count dependence and the
// zero-vector gravity direction.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_42EnvUmbrella) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false, "tests/data/MultiCartRail42Umbrella.usda");
}

// USD class-prim inheritance variant. `/envTemplate` is a class prim whose
// `railcart` child references the asset, and each `/envs/env_N` inherits it.
// Isolates composition by inheritance from the direct per-env references used
// by the `_NoSettle` and `_UmbrellaSceneAPI` variants.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_InheritUmbrella) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/MultiCartRailInheritUmbrella.usda",
        "/envs/*/railcart");
}

// Loads the USDA exported from a failing umbrella LinearDofVelocities run, which is
// the exact stage the ovstage population path sees there. Separates a cause in the
// USDA content from one above the C ABI (adapter indices, step path, or process-wide
// PhysX state from earlier tests).
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExport) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export.usda",
        "/envs/*/railcart");
}

// Same as _UmbrellaExport with the groundPlane stripped, to check whether the
// static collider at z=0 affects the cart's prismatic DOF integration.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportNoGround) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_noground.usda",
        "/envs/*/railcart");
}

// Same as _UmbrellaExport with 50m env spacing on both axes. Rails extend +/-3 in
// Y, so this removes the inter-env rail overlap present at the umbrella's
// row_spacing=2 / col_spacing=6.5 layout and tests whether that overlap is the trigger.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportWideSpacing) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_widespacing.usda",
        "/envs/*/railcart");
}

// Same as _UmbrellaExport with row_spacing widened to 8m, the smallest
// non-overlapping spacing for rails extending +/-3 in Y. col_spacing stays at the
// umbrella's 6.5m, so only the row-direction overlap is removed.
TEST_F(TensorBindingCpuTest, CpuMultiCartRailPrismaticDofVelocity_UmbrellaExportNoOverlap) {
    RunMultiCartRailPrismaticDofVelocity(
        m_handle, /*do_settle=*/false,
        "tests/data/umbrella_lindof_export_nooverlap.usda",
        "/envs/*/railcart");
}

// Linear drive PD settling, mirroring the umbrella LinearDofPositionTargets scenario
// (OMPE-94459). CartRailDriveLinear.usda uses the umbrella's PD parameters (k=2000,
// d=250, maxForce=4000). With a 10 kg cart (density 1000 kg/m^3, scale 0.2 x 0.25 x 0.2)
// the natural frequency is about 14 rad/s and the damping ratio about 0.88, so the
// cart settles in roughly 0.32 s. 60 steps at 60 Hz leave ample headroom.
TEST_F(TensorBindingCpuTest, CpuLinearDrivePD_SettlesIn60Steps) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartRailDriveLinear.usda", usd_handle))
        << "Failed to load CartRailDriveLinear.usda";

    // Settle step, matching the umbrella flow where the target is set before the
    // first step.
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

// For a single CartRailNoPole prismatic articulation, an ARTICULATION_DOF_VELOCITY_F32
// write, step and read preserves the joint velocity exactly (OMPE-94459). The drift
// seen by the umbrella LinearDofVelocities test only appears in its 42-env cloned grid
// and is engine-side, prismatic-specific behavior rather than an ovphysx defect.
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
    // The ovphysx path preserves the velocity exactly on a single articulation.
    EXPECT_NEAR(read_vel, set_vel, 1e-6f);

    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
}

// OMPE-94459 (#13) / NVBugs 6560084: ovphysx_get_object_type classifies prims by
// TensorAPI object type. Articulation joints stay ARTICULATION_JOINT, distinct
// from the standalone JOINT cases below.
TEST_F(TensorBindingCpuTest, CpuGetObjectType) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    // No explicit step before get_object_type. The engine is expected to do the
    // lazy attach and initial parse itself, which is what the umbrella's
    // TestObjectType scenario relies on.

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

    // Articulation joints stay distinct from standalone joints (NVBug 6560084).
    ASSERT_EQ(
        ovphysx_get_object_type(m_handle, make_ovx_string("/World/articulation/articulatedRevoluteJoint1"), &t).status,
        OVPHYSX_API_SUCCESS);
    EXPECT_EQ(t, OVPHYSX_OBJECT_TYPE_ARTICULATION_JOINT);
}

// The classification is schema-independent, so each standalone-joint scene runs
// the same checks: the joint path is JOINT, an absent path is INVALID.
static void expectStandaloneJointClassified(ovphysx_handle_t handle, const char* usda, const char* joint_path) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(handle, usda, usd_handle)) << "Failed to load " << usda;

    ovphysx_object_type_t t = OVPHYSX_OBJECT_TYPE_INVALID;
    ASSERT_EQ(ovphysx_get_object_type(handle, make_ovx_string(joint_path), &t).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(t, OVPHYSX_OBJECT_TYPE_JOINT) << joint_path << " should classify as JOINT";
    ASSERT_EQ(ovphysx_get_object_type(handle, make_ovx_string("/World/Does_Not_Exist"), &t).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(t, OVPHYSX_OBJECT_TYPE_INVALID);
}

TEST_F(TensorBindingCpuTest, CpuGetObjectTypeStandalonePrismaticJoint) {
    expectStandaloneJointClassified(m_handle, "tests/data/standalone_prismatic_joint.usda", "/World/Anchor_Slide");
}

TEST_F(TensorBindingCpuTest, CpuGetObjectTypeStandaloneRevoluteJoint) {
    expectStandaloneJointClassified(m_handle, "tests/data/revolute_joint_scene.usda", "/World/revoluteJoint");
}

// Writing 1 to DISABLE_SIMULATION must freeze the body mid-simulation (OMPE-94459).
// Catches the case where the flag round-trips (CpuRigidBodyDisableSimulationRoundtrip)
// but the engine does not toggle PxActorFlag::eDISABLE_SIMULATION.
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

    // One more step without disabling. The pose must change under gravity.
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

    std::vector<uint8_t> flags(n, 1);
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    std::vector<float> pose_after_disable(poses);

    // Disabled bodies must not move over further steps.
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

// DISABLE_SIMULATION=1 written before any explicit step must be honoured by the next
// simulate (OMPE-94459, umbrella RigidBodyEnableDisablePhysics). Differs from
// CpuRigidBodyDisableSimulationStopsSimulation only in the missing step before the write.
TEST_F(TensorBindingCpuTest, CpuRigidBodyDisableSimulationInOnStart) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    // Bindings are created without stepping first. The create call triggers
    // ovphysx_ensure_physics_attached, which runs a simulate(0,0) for the initial
    // scene parse, but no real step has been taken from the user's perspective.
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

    // Starting pose after the initial parse and before any explicit step.
    std::vector<float> poses(n * 7, 0.0f);
    DLTensor pose_t{}; pose_t.data = poses.data(); pose_t.device = {kDLCPU, 0};
    pose_t.ndim = 2; pose_t.dtype = {kDLFloat, 32, 1};
    int64_t pose_shape[2] = {n, 7}; pose_t.shape = pose_shape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);
    std::vector<float> pose_initial(poses);

    std::vector<uint8_t> flags(n, 1);
    DLTensor flag_t{}; flag_t.data = flags.data(); flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1; flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n}; flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(n, 0);
    flag_t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, disable_binding, &flag_t).status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_EQ(readback[i], 1) << "body " << i << " disable flag did not persist";
    }

    for (int s = 0; s < 3; ++s) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

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
    // None of the bodies may drift. This assert is what catches the regression
    // where a disable written before the first step is not re-read by PhysX.
    EXPECT_EQ(moved, 0) << "disabled bodies drifted after an on_start disable write";

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// Same write-before-step flow as CpuRigidBodyDisableSimulationInOnStart, but with an
// indexed write that disables only the even-indexed bodies (OMPE-94459). Mirrors the
// umbrella RigidBodyEnableDisablePhysics scenario, which writes a uniform 1 with
// indices [0, 2, 4, ...].
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

    // Every other body, as int32 to match the umbrella's warp index dtype.
    std::vector<int32_t> indices_disabled;
    std::vector<int32_t> indices_enabled;
    for (int32_t i = 0; i < int32_t(n); ++i) {
        if (i % 2 == 0) indices_disabled.push_back(i);
        else            indices_enabled.push_back(i);
    }

    // BaseRigidBodyView::setDisableSimulations reads src[idx] for each subset index,
    // so the source tensor must be sized [N], not [subset_count].
    std::vector<uint8_t> flags(n, 1);
    DLTensor flag_t{}; flag_t.data = flags.data(); flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1; flag_t.dtype = {kDLUInt, 8, 1};
    int64_t flag_shape[1] = {n}; flag_t.shape = flag_shape;

    DLTensor idx_t{}; idx_t.device = {kDLCPU, 0}; idx_t.dtype = {kDLInt, 32, 1};
    idx_t.ndim = 1; int64_t idx_shape[1] = {int64_t(indices_disabled.size())};
    idx_t.shape = idx_shape; idx_t.data = indices_disabled.data();

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, &idx_t).status,
              OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(n, 0);
    flag_t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, disable_binding, &flag_t).status, OVPHYSX_API_SUCCESS);
    for (int32_t idx : indices_disabled) {
        EXPECT_EQ(readback[idx], 1) << "disabled body " << idx << " not flagged";
    }
    for (int32_t idx : indices_enabled) {
        EXPECT_EQ(readback[idx], 0) << "enabled body " << idx << " incorrectly flagged";
    }

    // Two steps match the umbrella's stepno=1, which is reached after two simulates.
    for (int s = 0; s < 2; ++s) {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

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
    // The even-index subset must read zero z-velocity while the odd-index bodies
    // left enabled must still be falling. Both checks guard the alternate-index path.
    EXPECT_EQ(disabled_falling, 0) << "disabled (even-index) bodies still falling";
    EXPECT_EQ(enabled_static, 0) << "enabled (odd-index) bodies were not falling";

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);
}

// CPU reference for the umbrella's enable/disable flip sequence (OMPE-94459), which
// the GPU path is measured against: disable a moving body, then flip and wake in one
// window and step. A disabled body reads zero velocity and a re-enabled one resumes.
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

    std::vector<int32_t> idx_A, idx_B; // A starts disabled, B starts enabled.
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

    // Disable A before the first step, then step twice. The umbrella's stepno=1 sees -2*dt*g.
    write_disable_subset(1, idx_A);
    step(); step();
    std::vector<float> vz1; read_vz(vz1);

    // Flip in one window: disable B, enable A, wake all.
    write_disable_subset(1, idx_B);
    write_disable_subset(0, idx_A);
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, nullptr).status, OVPHYSX_API_SUCCESS);
    step();
    std::vector<float> vz2; read_vz(vz2);

    // A body disabled mid-motion reads 0. A re-enabled and woken body resumes.
    for (int32_t i : idx_B)
        EXPECT_NEAR(vz2[i], 0.f, 1e-3f) << "cpu: now-disabled body " << i << " should read 0, got " << vz2[i];
    for (int32_t i : idx_A)
        EXPECT_LT(vz2[i], -0.01f) << "cpu: re-enabled+woken body " << i << " should be falling, got " << vz2[i];

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// ovphysx_rigid_body_view_wake_up validates its index tensor (1D int32, length <= body
// count) before forwarding, so a malformed tensor is rejected rather than misread as a
// host PxU32 buffer (OMPE-94459). Validation runs before any device staging, so no CUDA
// is needed.
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
    // Null indices wake all bodies.
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

// ovphysx_rigid_body_view_wake_up after a disable/re-enable cycle (OMPE-94459).
// Re-enabled bodies come back asleep, so they only move again after wake_up. Bodies
// still flagged disabled are skipped by wake_up, as in BaseRigidBodyView::wakeUp.
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

    // Disabled bodies must not move across steps.
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

    // wake_up on still-disabled bodies is a documented per-body no-op. The call
    // itself succeeds.
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, pose_binding, nullptr).status,
              OVPHYSX_API_SUCCESS);
    for (int s = 0; s < 2; ++s) step_once();
    std::vector<float> pose_after_silent_wake;
    read_poses(pose_after_silent_wake);
    for (int64_t i = 0; i < n; ++i) {
        EXPECT_NEAR(pose_after_silent_wake[i * 7 + 2], pose_after_disable[i * 7 + 2], 1e-4f)
            << "body " << i << " z drifted after no-op wake on disabled body";
    }

    // PhysX puts the re-introduced actor in sleep state. Without wake_up, stepping
    // leaves the bodies asleep with zero velocity.
    write_disable_all(0);
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
        if (dz > 1e-3f) ++moved;  // Positive means the body fell under gravity.
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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t dof_binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &dof_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(spec.ndim, 2);
    EXPECT_GT(spec.shape[0], 0);
    EXPECT_GT(spec.shape[1], 0);

    // In CPU mode warmup is a no-op but must still succeed.
    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU read should work";

    for (size_t i = 0; i < total_elements; ++i) {
        data[i] = 0.1f;
    }

    result = ovphysx_write_tensor_binding(m_handle, dof_binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // No read-back check here. The CPU TensorAPI may not reflect writes
    // immediately, so write verification lives in the GPU tests.

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
    desc.pattern = {nullptr, 0};
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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    float data[3];
    int64_t wrong_shape[2] = {1, 3};
    DLTensor tensor = {};
    tensor.data = data;
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = wrong_shape;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingErrorTest, WrongDtype) {
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

    std::vector<int32_t> data(spec.shape[0] * spec.shape[1], 0);
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor tensor = {};
    tensor.data = data.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLInt, 32, 1};
    tensor.shape = shape;

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingErrorTest, ZeroMatchesSucceeds) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/NonExistent/Path/That/Matches/Nothing");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    // Creation succeeds even with zero matches.
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);

    if (result.status == OVPHYSX_API_SUCCESS) {
        ovphysx_tensor_spec_t spec;
        result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
        EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS);
        EXPECT_EQ(spec.shape[0], 0);

        // Read and write are successful no-ops for zero matches.
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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_GT(spec.shape[0], 0);

    // Full write of a known value first, so the indexed write below is observable.
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

    // Indexed write of one articulation's DOFs, shape [1, D].
    std::vector<float> partial_data(spec.shape[1], 0.5f);
    int64_t partial_shape[2] = {1, spec.shape[1]};

    DLTensor src_tensor = {};
    src_tensor.data = partial_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = partial_shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

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

    result = ovphysx_write_tensor_binding(m_handle, binding, &src_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

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

    // Row 0 carries the indexed value, all other rows keep the full-write value.
    for (int64_t j = 0; j < spec.shape[1]; ++j)
        EXPECT_FLOAT_EQ(readback[0 * spec.shape[1] + j], 0.5f);

    for (int64_t i = 1; i < spec.shape[0]; ++i)
        for (int64_t j = 0; j < spec.shape[1]; ++j)
            EXPECT_FLOAT_EQ(readback[i * spec.shape[1] + j], 1.0f);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// ============================================================================
// MULTIPLE BINDINGS TEST
// ============================================================================

TEST_F(TensorBindingCpuTest, MultipleSamePatternBindings) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Several bindings on the same pattern with different tensor types.
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

    // The same pattern yields the same shape for every binding.
    ovphysx_tensor_spec_t pos_spec, vel_spec, target_spec;

    result = ovphysx_get_tensor_binding_spec(m_handle, pos_binding, &pos_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, vel_binding, &vel_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, target_binding, &target_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(pos_spec.shape[0], vel_spec.shape[0]);
    EXPECT_EQ(pos_spec.shape[0], target_spec.shape[0]);
    EXPECT_EQ(pos_spec.shape[1], vel_spec.shape[1]);
    EXPECT_EQ(pos_spec.shape[1], target_spec.shape[1]);

    ovphysx_destroy_tensor_binding(m_handle, pos_binding);
    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
    ovphysx_destroy_tensor_binding(m_handle, target_binding);
}

TEST_F(TensorBindingCpuTest, DuplicateBindingSameType) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle));

    // Two bindings with the same pattern and the same tensor type.
    ovphysx_tensor_binding_handle_t binding1 = 0;
    ovphysx_tensor_binding_handle_t binding2 = 0;

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding1);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_create_tensor_binding(m_handle, &desc, &binding2);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_NE(binding1, binding2);

    ovphysx_tensor_spec_t spec1, spec2;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding1, &spec1);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_get_tensor_binding_spec(m_handle, binding2, &spec2);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(spec1.shape[0], spec2.shape[0]);
    EXPECT_EQ(spec1.shape[1], spec2.shape[1]);

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

    // Both bindings read the same underlying physics state.
    for (size_t i = 0; i < data1.size(); ++i) {
        EXPECT_FLOAT_EQ(data1[i], data2[i]);
    }

    ovphysx_destroy_tensor_binding(m_handle, binding1);
    ovphysx_destroy_tensor_binding(m_handle, binding2);
}

// ============================================================================
// FORCE / WRENCH EFFECT TESTS
// ============================================================================
// Written forces must produce observable physical effects, not just API success.
// boxes_falling_on_groundplane.usda has 11 cubes at Z=10.

TEST_F(TensorBindingCpuTest, ForceWriteEffect_RigidBodyDisplacement) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    // One step to initialize physics.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f/60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    ovphysx_tensor_binding_handle_t force_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &force_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create force binding";
    }

    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create pose binding";
    }

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

    // 50000 N in +X on body 0, zero on every other body. The last body is the reference.
    std::vector<float> forces(N * 3, 0.0f);
    forces[0 * 3 + 0] = 50000.0f;

    DLTensor force_tensor{};
    force_tensor.data = forces.data();
    force_tensor.device = {kDLCPU, 0};
    force_tensor.ndim = 2;
    force_tensor.dtype = {kDLFloat, 32, 1};
    int64_t force_shape[2] = {N, 3};
    force_tensor.shape = force_shape;
    force_tensor.strides = nullptr;
    force_tensor.byte_offset = 0;

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i) {
        ovphysx_result_t wr = ovphysx_write_tensor_binding(m_handle, force_binding, &force_tensor, nullptr);
        ASSERT_EQ(wr.status, OVPHYSX_API_SUCCESS) << "Force write failed on step " << i;

        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

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

    // 50000 N over 5 steps at 1/60 s moves even a 1000 kg body measurably.
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
    // Same as the force test, through the WRENCH [N, 9] path that bypasses the
    // deprecated applyForces wrapper.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load USD";

    // One step to initialize physics.
    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f/60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    ovphysx_tensor_binding_handle_t wrench_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &wrench_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create wrench binding";
    }

    ovphysx_tensor_binding_handle_t pose_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ovphysx_result_t r = ovphysx_create_tensor_binding(m_handle, &desc, &pose_binding);
        ASSERT_EQ(r.status, OVPHYSX_API_SUCCESS) << "Failed to create pose binding";
    }

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

    // Row layout is [fx,fy,fz, tx,ty,tz, px,py,pz]. 50000 N in +X on body 0, with
    // torque and position zero so the force acts at the center of mass.
    std::vector<float> wrenches(N * 9, 0.0f);
    wrenches[0 * 9 + 0] = 50000.0f;

    DLTensor wrench_tensor{};
    wrench_tensor.data = wrenches.data();
    wrench_tensor.device = {kDLCPU, 0};
    wrench_tensor.ndim = 2;
    wrench_tensor.dtype = {kDLFloat, 32, 1};
    int64_t wrench_shape[2] = {N, 9};
    wrench_tensor.shape = wrench_shape;
    wrench_tensor.strides = nullptr;
    wrench_tensor.byte_offset = 0;

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 5; ++i) {
        ovphysx_result_t wr = ovphysx_write_tensor_binding(m_handle, wrench_binding, &wrench_tensor, nullptr);
        ASSERT_EQ(wr.status, OVPHYSX_API_SUCCESS) << "Wrench write failed on step " << i;

        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

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
// ovphysx must run without crashes on systems without a GPU. These tests load
// stages that select CPU mode (no physxScene:enableGPUDynamics) and exercise the
// full lifecycle so no code path dereferences a null CUDA handle or calls a cu*
// function.
// ============================================================================

TEST_F(TensorBindingCpuTest, CpuOnlyLifecycle_NoCrash) {
    // Core lifecycle in CPU mode: load USD, step, create bindings, read, write,
    // destroy. Not crashing is the point, value correctness is secondary.

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample.usda", usd_handle))
        << "Failed to load USD in CPU mode";

    for (int i = 0; i < 3; ++i) {
        const float dt = 1.0f / 60.0f;
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, dt);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS) << "CPU step " << i << " failed";
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // warmup() runs in CPU mode too: a 1ns step for lazy init, and it disables Fabric sync.
    ovphysx_result_t result = ovphysx_warmup(m_handle);
    EXPECT_EQ(result.status, OVPHYSX_API_SUCCESS) << "warmup should succeed in CPU mode";

    ovphysx_tensor_binding_handle_t dof_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
        result = ovphysx_create_tensor_binding(m_handle, &desc, &dof_binding);
        ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU DOF binding creation failed";
    }

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_GT(spec.shape[0], 0);
    EXPECT_GT(spec.shape[1], 0);

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

    for (size_t i = 0; i < total; ++i) data[i] = 0.05f;
    result = ovphysx_write_tensor_binding(m_handle, dof_binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "CPU write failed";

    {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    }

    // Destroying the binding must not crash even though the GPU was never initialized.
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

    // Host pointers that claim to be CUDA memory. Every call must reject the
    // device before accessing those pointers.
    float dummy = 0.0f;
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    DLTensor fake_gpu_tensor = {};
    fake_gpu_tensor.data = &dummy;
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
    fake_gpu_index.data = &index_data;
    fake_gpu_index.ndim = 1;
    fake_gpu_index.dtype = {kDLInt, 32, 1};
    fake_gpu_index.shape = index_shape;

    std::vector<uint8_t> mask_data(static_cast<size_t>(shape[0]), 1);
    int64_t mask_shape[1] = {shape[0]};
    DLTensor fake_gpu_mask = {};
    fake_gpu_mask.data = mask_data.data();
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

        // These tests cover the DirectGPU tensor pipeline (GPU-resident state via
        // PxDirectGPUAPI), so /physics/suppressReadback is opted into explicitly.
        // ovphysx does not enable it for GPU instances by default, see the
        // create_args documentation in ovphysx_types.h. config_entries are applied
        // after carb settings load and before the PhysX plugins, which is the
        // window where the setting has to land.
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

TEST_F(TensorBindingGpuTest, NativeDeviceReportsCudaStateAndCpuProperty)
{
    ovphysx_usd_handle_t usdHandle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usdHandle));

    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");

    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
    ovphysx_tensor_binding_handle_t stateBinding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &stateBinding).status, OVPHYSX_API_SUCCESS);

    DLDevice stateDevice{ kDLExtDev, -1 };
    ASSERT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, stateBinding, &stateDevice).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(stateDevice.device_type, kDLCUDA);
    EXPECT_EQ(stateDevice.device_id, 0);

    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_STIFFNESS_F32;
    ovphysx_tensor_binding_handle_t propertyBinding = 0;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &propertyBinding).status, OVPHYSX_API_SUCCESS);

    DLDevice propertyDevice{ kDLExtDev, -1 };
    ASSERT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, propertyBinding, &propertyDevice).status,
              OVPHYSX_API_SUCCESS);
    EXPECT_EQ(propertyDevice.device_type, kDLCPU);
    EXPECT_EQ(propertyDevice.device_id, 0);

    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, propertyBinding).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(ovphysx_destroy_tensor_binding(m_handle, stateBinding).status, OVPHYSX_API_SUCCESS);
}

TEST_F(TensorBindingGpuTest, GpuArticulationDofReadWrite) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t dof_binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &dof_binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, dof_binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(spec.ndim, 2);
    EXPECT_GT(spec.shape[0], 0);
    EXPECT_GT(spec.shape[1], 0);

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    size_t total_elements = spec.shape[0] * spec.shape[1];
    size_t buffer_size = total_elements * sizeof(float);

    void* gpu_data = allocGpuBuffer(buffer_size, dof_binding);
    ASSERT_NE(gpu_data, nullptr) << "Failed to allocate GPU buffer";

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

    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU read failed";

    std::vector<float> host_data(total_elements, 0.1f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, host_data.data(), buffer_size));

    result = ovphysx_write_tensor_binding(m_handle, dof_binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU write failed";

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total_elements));  // Cleared so the read is observable.
    result = ovphysx_read_tensor_binding(m_handle, dof_binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> verify_data(total_elements);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(verify_data.data(), m_gpuBuffer, buffer_size));

    for (size_t i = 0; i < total_elements; ++i) {
        EXPECT_NEAR(verify_data[i], 0.1f, 0.01f) << "DOF position mismatch at index " << i;
    }

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

    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_warmup(m_handle);
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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

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

    // No ovphysx_warmup() call. The first write must warm up on its own in GPU mode.
    std::vector<float> src_host(total, 0.2f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

    result = ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "First GPU write should auto-warmup";

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

    // No ovphysx_warmup() call. The first masked write must warm up on its own.
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
    // Counterpart of CrossDeviceReadCpuTensorFromGpuBinding. ovphysx_write_tensor_binding
    // accepts a CPU source against a GPU binding by staging it in a GPU buffer on the
    // binding's device before forwarding to PhysX.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32;  // Writable and GPU-resident.

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup(m_handle);
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

    // The read side is also cross-device capable, so a round trip confirms the
    // staged write reached PhysX.
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
    // OMPE-103213: CPU-only property bindings such as body mass refuse GPU sources
    // rather than silently staging device to host. Callers must supply host tensors.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_MASS_F32;  // Writable, CPU-only property.

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0]) * static_cast<size_t>(spec.shape[1]);
    const size_t bytes = total * sizeof(float);

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
    EXPECT_EQ(result.status, OVPHYSX_API_DEVICE_MISMATCH)
        << "GPU src against a CPU-only binding must be refused (no silent DtoH staging)";

    // A host tensor still writes successfully.
    DLTensor cpu_tensor = gpu_tensor;
    cpu_tensor.data = seed.data();
    cpu_tensor.device = {kDLCPU, 0};
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &cpu_tensor, nullptr).status,
              OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, CrossDeviceReadCpuTensorFromGpuBinding) {
    // ovphysx_read_tensor_binding supports cross-device reads through internal staging.
    // A GPU binding read into a CPU destination goes through a GPU staging buffer and a
    // device-to-host copy. This test covers the GPU to CPU direction.
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup(m_handle);
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

TEST_F(TensorBindingGpuTest, CrossDeviceStagingWithoutCallerCudaContext) {
    // Cross-device staging must push the binding's CUDA context instead of inheriting
    // the caller's.
    omni::physx::IOptionalCuda* cuda = ovphysx::test_cuda::getCuda();
    ASSERT_NE(cuda, nullptr);
    ASSERT_TRUE(cuda->cudaAvailable());

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32;  // Writable and GPU-resident.

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0]) * static_cast<size_t>(spec.shape[1]);
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};

    std::vector<float> host_src(total, 0.25f);
    DLTensor src_tensor = {};
    src_tensor.data = host_src.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;

    std::vector<float> host_dst(total, 0.0f);
    DLTensor dst_tensor = src_tensor;
    dst_tensor.data = host_dst.data();

    ovphysx::test_cuda::ScopedCudaContextDetach detach(cuda);

    ovphysx_result_t write_result = ovphysx_write_tensor_binding(m_handle, binding, &src_tensor, nullptr);
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda))
        << "write_tensor_binding left a CUDA context pushed on the caller's thread";

    ovphysx_result_t read_result = ovphysx_read_tensor_binding(m_handle, binding, &dst_tensor);
    EXPECT_TRUE(ovphysx::test_cuda::noCudaContextCurrent(cuda))
        << "read_tensor_binding left a CUDA context pushed on the caller's thread";

    EXPECT_TRUE(detach.restore()) << "failed to restore the caller's CUDA context stack";

    EXPECT_EQ(write_result.status, OVPHYSX_API_SUCCESS)
        << "Host source write into a GPU binding must stage in the binding's CUDA context";
    EXPECT_EQ(read_result.status, OVPHYSX_API_SUCCESS)
        << "GPU binding read into a host destination must stage in the binding's CUDA context";

    if (write_result.status == OVPHYSX_API_SUCCESS && read_result.status == OVPHYSX_API_SUCCESS) {
        for (size_t i = 0; i < total; ++i) {
            EXPECT_FLOAT_EQ(host_dst[i], 0.25f) << "staged round-trip lost data (i=" << i << ")";
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, NonContiguousTensorRejected) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/links_chain_sample_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    size_t total_elements = spec.shape[0] * spec.shape[1];
    void* gpu_data = allocGpuBuffer(total_elements * sizeof(float), binding);
    ASSERT_NE(gpu_data, nullptr);

    // The contiguity check skips size-1 dimensions, where the stride is irrelevant, so
    // the bad stride has to go on a dimension with size > 1. The DOF dimension is used.
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};

    if (spec.shape[1] <= 1) {
        GTEST_SKIP() << "Test requires shape[1] > 1 to test non-contiguous stride rejection";
    }

    // stride[1] = 2 instead of 1, so elements are not adjacent.
    int64_t strides[2] = {spec.shape[1] * 2, 2};

    DLTensor tensor = {};
    tensor.data = gpu_data;
    tensor.device = {kDLCUDA, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    tensor.shape = shape;
    tensor.strides = strides;
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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load two_articulations.usda";

    // DOF position targets read back exactly what was written, so they suit a
    // masked-write check.
    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.ndim, 2);
    ASSERT_EQ(spec.shape[0], 2) << "Expected 2 articulations";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    ASSERT_GT(D, 0);

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

    std::vector<float> saved_initial(initial_data.begin(), initial_data.end());

    std::vector<float> src_data(total, 0.5f);
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Only the first articulation is selected.
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

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Masked write failed";

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

    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[0 * D + j], 0.5f)
            << "Row 0 (masked=1) should be updated at col " << j;
    }
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[1 * D + j], saved_initial[1 * D + j])
            << "Row 1 (masked=0) should be unchanged at col " << j;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_DofPositionTargets_AllTrue) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

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

    // Distinctive source values, so any leak through the all-false mask is visible.
    std::vector<float> src_data(total, 99.0f);
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane.usda";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create rigid body pose binding";

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.ndim, 2);
    ASSERT_GE(spec.shape[0], 2) << "Need at least 2 rigid bodies";
    ASSERT_EQ(spec.shape[1], 7) << "Pose should be [N, 7] (pos xyz + quat xyzw)";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    size_t total = N * D;
    int64_t shape[2] = {N, D};

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

    // Distinctive source: position (99,99,99) with identity rotation.
    std::vector<float> src_data(total);
    for (int64_t i = 0; i < N; ++i) {
        src_data[i * D + 0] = 99.0f;
        src_data[i * D + 1] = 99.0f;
        src_data[i * D + 2] = 99.0f;
        src_data[i * D + 3] = 0.0f;
        src_data[i * D + 4] = 0.0f;
        src_data[i * D + 5] = 0.0f;
        src_data[i * D + 6] = 1.0f;
    }
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

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

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Masked write for rigid body pose failed";

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

    EXPECT_NEAR(readback[0 * D + 0], 99.0f, 0.01f) << "Body 0 px should be 99";
    EXPECT_NEAR(readback[0 * D + 1], 99.0f, 0.01f) << "Body 0 py should be 99";
    EXPECT_NEAR(readback[0 * D + 2], 99.0f, 0.01f) << "Body 0 pz should be 99";
    EXPECT_NEAR(readback[0 * D + 6], 1.0f, 0.01f)  << "Body 0 qw should be 1";

    for (int64_t i = 1; i < N; ++i) {
        for (int64_t j = 0; j < D; ++j) {
            EXPECT_FLOAT_EQ(readback[i * D + j], saved_initial[i * D + j])
                << "Body " << i << " col " << j << " should be unchanged (mask=0)";
        }
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingCpuTest, MaskedWriteCpu_ValidationErrors) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle));

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

    // The source tensor is valid throughout, so every rejection comes from the mask.
    std::vector<float> src_data(total, 0.5f);
    DLTensor src_tensor = {};
    src_tensor.data = src_data.data();
    src_tensor.device = {kDLCPU, 0};
    src_tensor.ndim = 2;
    src_tensor.dtype = {kDLFloat, 32, 1};
    src_tensor.shape = shape;
    src_tensor.strides = nullptr;
    src_tensor.byte_offset = 0;

    // Wrong mask dtype: float32 instead of uint8.
    {
        std::vector<float> bad_mask_data(N, 1.0f);
        int64_t mask_shape[1] = {N};
        DLTensor bad_mask = {};
        bad_mask.data = bad_mask_data.data();
        bad_mask.device = {kDLCPU, 0};
        bad_mask.ndim = 1;
        bad_mask.dtype = {kDLFloat, 32, 1};
        bad_mask.shape = mask_shape;
        bad_mask.strides = nullptr;
        bad_mask.byte_offset = 0;

        result = ovphysx_write_tensor_binding_masked(m_handle, binding, &src_tensor, &bad_mask);
        EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT)
            << "Should reject float32 mask dtype";
    }

    // Wrong mask length.
    {
        std::vector<uint8_t> bad_mask_data(N + 5, 1);
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

    // Null mask tensor.
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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle))
        << "Failed to load two_articulations_gpu.usda";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32;

    ovphysx_result_t result = ovphysx_create_tensor_binding(m_handle, &desc, &binding);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "Failed to create binding";

    ovphysx_tensor_spec_t spec;
    result = ovphysx_get_tensor_binding_spec(m_handle, binding, &spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    ASSERT_EQ(spec.shape[0], 2) << "Expected 2 articulations";
    int64_t N = spec.shape[0];
    int64_t D = spec.shape[1];
    ASSERT_GT(D, 0);
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

    // One GPU buffer serves as write source and read destination.
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

    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    std::vector<float> src_host(total, 0.5f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

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
    mask_tensor.dtype = {kDLUInt, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU masked write failed";

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[0 * D + j], 0.5f, 0.01f)
            << "Row 0 (masked=1) should be updated at col " << j;
    }
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[1 * D + j], initial_host[1 * D + j], 0.01f)
            << "Row 1 (masked=0) should be unchanged at col " << j;
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_DofPositionTargets_AllTrue) {
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

    result = ovphysx_warmup(m_handle);
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

    std::vector<float> src_host(total, 0.75f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

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

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

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
    // kDLBool with bits=8 is the dtype PyTorch uses for bool tensors, and it must be
    // accepted as a GPU mask.
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

    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    std::vector<float> src_host(total, 0.33f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

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
    mask_tensor.dtype = {kDLBool, 8, 1};
    mask_tensor.shape = mask_shape;
    mask_tensor.strides = nullptr;
    mask_tensor.byte_offset = 0;

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU masked write with kDLBool dtype should succeed";

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[0 * D + j], 0.33f, 0.01f)
            << "Row 0 (bool mask=True) should be updated at col " << j;
    }
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[1 * D + j], initial_host[1 * D + j], 0.01f)
            << "Row 1 (bool mask=False) should be unchanged at col " << j;
    }

    (void)m_cudaOps.memFree(gpu_mask);
    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, MaskedWriteGpu_DofPositionTargets_AllFalse) {
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

    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    // Distinctive source values, so any leak through the all-false mask is visible.
    std::vector<float> src_host(total, 99.0f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

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

    // An all-false mask makes the write a no-op.
    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

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
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane_gpu.usda";

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
    int64_t D = spec.shape[1];
    size_t total = N * D;
    size_t buffer_size = total * sizeof(float);

    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> initial_host(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial_host.data(), m_gpuBuffer, buffer_size));

    // Distinctive source: position (77,77,77) with identity rotation on every row.
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

    uintptr_t gpu_mask = 0;
    int cu_st = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(static_cast<size_t>(N) * sizeof(uint8_t), &gpu_mask, &cu_st))
        << "Failed to allocate GPU mask buffer (status=" << cu_st << ")";
    std::vector<uint8_t> mask_host(N, 0);
    mask_host[N - 1] = 1;
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

    result = ovphysx_write_tensor_binding_masked(m_handle, binding, &gpu_tensor, &mask_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU masked write for rigid body pose failed";

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    for (int64_t i = 0; i < N - 1; ++i) {
        for (int64_t j = 0; j < D; ++j) {
            EXPECT_NEAR(readback[i * D + j], initial_host[i * D + j], 0.01f)
                << "Body " << i << " col " << j << " should be unchanged (mask=0)";
        }
    }

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
    // A full [N,D] source with a subset of indices must update only the indexed rows.
    // The row mapping has to hold when src.shape[0] != indices.shape[0].

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations.usda", usd_handle))
        << "Failed to load two_articulations.usda";

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

    // Full write of a known value first, so the indexed write below is observable.
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

    // Full [N, D] source with distinct rows, so a row-mapping error is visible.
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

    result = ovphysx_write_tensor_binding(m_handle, binding, &src_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "Indexed write with full tensor should succeed";

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

    // Source row 0 lands on index 0, row 1 keeps the initial full-write value.
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[0 * D + j], 0.5f)
            << "Row 0 should be updated to 0.5 at col " << j;
    }
    for (int64_t j = 0; j < D; ++j) {
        EXPECT_FLOAT_EQ(readback[1 * D + j], 1.0f)
            << "Row 1 should remain at 1.0 (not indexed) at col " << j;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, IndexedWriteFullTensor_GpuRegression) {
    // GPU counterpart of IndexedWriteFullTensor_CpuRegression.

    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle))
        << "Failed to load two_articulations_gpu.usda";

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

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

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

    // Full write of a known value first, so the indexed write below is observable.
    std::vector<float> init_host(total, 1.0f);
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, init_host.data(), buffer_size));

    result = ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    // Full [N, D] source with distinct rows, so a row-mapping error is visible.
    std::vector<float> src_host(total);
    for (int64_t j = 0; j < D; ++j) {
        src_host[0 * D + j] = 0.5f;
        src_host[1 * D + j] = 0.99f;
    }
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(m_gpuBuffer, src_host.data(), buffer_size));

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

    result = ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, &index_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS)
        << "GPU indexed write with full tensor should succeed";

    ASSERT_TRUE(m_cudaOps.memsetD32(m_gpuBuffer, 0u, total));
    result = ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);

    std::vector<float> readback(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(readback.data(), m_gpuBuffer, buffer_size));

    for (int64_t j = 0; j < D; ++j) {
        EXPECT_NEAR(readback[0 * D + j], 0.5f, 0.01f)
            << "Row 0 should be updated to 0.5 at col " << j;
    }
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
    const int64_t C = spec.shape[1];
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

    // Distinctive velocity on row 0 only.
    std::vector<float> src(N * C, 0.0f);
    for (int i = 0; i < C; ++i) src[i] = 123.0f;

    DLTensor src_tensor = tensor;
    src_tensor.data = src.data();

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

    // Both source rows carry the same known pose. The mask selects only row 1.
    std::vector<float> src(N * C, 0.0f);
    for (int64_t r = 0; r < N; ++r) {
        src[r * C + 0] = 10.0f;
        src[r * C + 1] = 20.0f;
        src[r * C + 2] = 30.0f;
        src[r * C + 3] = 0.0f;
        src[r * C + 4] = 0.0f;
        src[r * C + 5] = 0.0f;
        src[r * C + 6] = 1.0f;
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

    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    std::vector<float> initial(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial.data(), m_gpuBuffer, buffer_size));

    // Distinctive velocity on row 0 only.
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

    result = ovphysx_warmup(m_handle);
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

    result = ovphysx_read_tensor_binding(m_handle, binding, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    std::vector<float> initial(total);
    ASSERT_TRUE(m_cudaOps.memcpyDtoH(initial.data(), m_gpuBuffer, buffer_size));

    // Both source rows carry the same known pose. The mask selects only row 1.
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
    EXPECT_NEAR(value, 0.01f, 1.0e-4f);  // Value authored in the fixture.

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

    // Values authored in the fixture.
    ovphysx_result_t result = ovphysx_read_tensor_binding(m_handle, bstiff, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 100.0f, 1.0e-2f);

    result = ovphysx_read_tensor_binding(m_handle, thick, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.01f, 1.0e-5f);

    result = ovphysx_read_tensor_binding(m_handle, bdamp, &tensor);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    EXPECT_NEAR(value, 0.05f, 1.0e-4f);

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

    // int32 index.
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

    // int64 index, staged to int32 internally.
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

    // With mask = 1 the write applies.
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

    // With mask = 0 the write is skipped and the value stays at 300.
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
    EXPECT_EQ(spec.shape[0], 1);
    EXPECT_GT(spec.shape[1], 0);
    EXPECT_GT(spec.shape[2], 0);  // K = getNumNodesPerElement(), 4 for a volume tetmesh.
    EXPECT_EQ(spec.dtype.code, static_cast<uint8_t>(kDLInt));
    EXPECT_EQ(spec.dtype.bits, 32);
    EXPECT_EQ(spec.dtype.lanes, 1);

    result = ovphysx_warmup(m_handle);
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
    for (auto idx : readback)
        EXPECT_GE(idx, 0);

    // Element indices are read-only.
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
    EXPECT_EQ(pos_spec.shape[0], 1);
    EXPECT_EQ(pos_spec.shape[1], 4);  // Square cloth with four simulation nodes.
    EXPECT_EQ(pos_spec.shape[2], 3);

    ovphysx_tensor_spec_t elem_spec{};
    result = ovphysx_get_tensor_binding_spec(m_handle, elems, &elem_spec);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS);
    ASSERT_EQ(elem_spec.ndim, 3);
    EXPECT_EQ(elem_spec.shape[0], 1);
    EXPECT_EQ(elem_spec.shape[1], 2);
    EXPECT_EQ(elem_spec.shape[2], 3);  // K = 3 for a trimesh.
    EXPECT_EQ(elem_spec.dtype.code, static_cast<uint8_t>(kDLInt));
    EXPECT_EQ(elem_spec.dtype.bits, 32);
    EXPECT_EQ(elem_spec.dtype.lanes, 1);

    result = ovphysx_warmup(m_handle);
    ASSERT_EQ(result.status, OVPHYSX_API_SUCCESS) << "GPU warmup failed";

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
    // The fixture places the first node at the origin.
    EXPECT_NEAR(positions[0], 0.0f, 1.0e-4f);
    EXPECT_NEAR(positions[1], 0.0f, 1.0e-4f);
    EXPECT_NEAR(positions[2], 0.0f, 1.0e-4f);

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

    // Rest positions and element indices are read-only.
    result = ovphysx_write_tensor_binding(m_handle, rest, &pos_tensor, &index_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);
    result = ovphysx_write_tensor_binding(m_handle, elems, &elem_tensor, &index_tensor);
    EXPECT_EQ(result.status, OVPHYSX_API_INVALID_ARGUMENT);

    ovphysx_destroy_tensor_binding(m_handle, pos);
    ovphysx_destroy_tensor_binding(m_handle, vel);
    ovphysx_destroy_tensor_binding(m_handle, rest);
    ovphysx_destroy_tensor_binding(m_handle, elems);
}

// OMPE-94459 / OMPE-103213: DISABLE_SIMULATION_BOOL is CPU-only even on a DirectGPU
// scene. Host all-enabled tensors round-trip. Writing any disable=1 invalidates the
// binding because DirectGPU has no row for the body. CUDA tensors are refused.
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

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const int64_t n = spec.shape[0];
    const size_t bytes = static_cast<size_t>(n);

    // All-enabled round trip. No membership change, so the binding stays valid.
    std::vector<uint8_t> all_enabled(n, 0);
    DLTensor tensor{};
    tensor.data = all_enabled.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[1] = {n};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(n, 0xff);
    tensor.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < n; ++i)
        EXPECT_EQ(static_cast<int>(readback[i]), 0) << "body " << i;

    // CUDA buffers must be refused with DEVICE_MISMATCH, not silently staged.
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);
    DLTensor gpu_tensor = tensor;
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr).status,
              OVPHYSX_API_DEVICE_MISMATCH);
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor).status,
              OVPHYSX_API_DEVICE_MISMATCH);

    // Writing disable=1 invalidates the DirectGPU mapping.
    std::vector<uint8_t> with_disable(n, 0);
    with_disable[0] = 1;
    tensor.data = with_disable.data();
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr).status, OVPHYSX_API_SUCCESS);
    tensor.data = readback.data();
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_NOT_FOUND);
    DLDevice staleDevice{};
    EXPECT_EQ(ovphysx_get_tensor_binding_native_device(m_handle, binding, &staleDevice).status,
              OVPHYSX_API_NOT_FOUND);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableGravityRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 1);
    EXPECT_EQ(spec.dtype.code, kDLUInt);
    EXPECT_EQ(spec.dtype.bits, 8);
    ASSERT_GT(spec.shape[0], 0);

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const int64_t n = spec.shape[0];
    const size_t bytes = static_cast<size_t>(n);

    std::vector<uint8_t> written(n, 0);
    for (int64_t i = 0; i < n; ++i)
        written[i] = static_cast<uint8_t>(i % 2 == 0 ? 1 : 0);

    DLTensor tensor{};
    tensor.data = written.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 1;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[1] = {n};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(n, 0xff);
    tensor.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < n; ++i)
        EXPECT_EQ(static_cast<int>(readback[i]), static_cast<int>(written[i])) << "body " << i;

    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);
    DLTensor gpu_tensor = tensor;
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr).status,
              OVPHYSX_API_DEVICE_MISMATCH);
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor).status,
              OVPHYSX_API_DEVICE_MISMATCH);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// Companion to CpuRigidBodyDisableGravitySuppressesFall. The umbrella sets disable
// flags before the first physics step and checks vz on step 1, with no mid-flight swap.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableGravitySuppressesFall) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t vel_binding = 0;
    ovphysx_tensor_binding_handle_t grav_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &vel_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &grav_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t vel_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_binding, &vel_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = vel_spec.shape[0];
    ASSERT_GT(n, 0);

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t vel_bytes = static_cast<size_t>(n * 6) * sizeof(float);
    void* gpu_vel = allocGpuBuffer(vel_bytes, vel_binding);
    ASSERT_NE(gpu_vel, nullptr);

    int64_t vel_shape[2] = {n, 6};
    DLTensor vel_t{};
    vel_t.data = gpu_vel;
    vel_t.device = {kDLCUDA, 0};
    vel_t.ndim = 2;
    vel_t.dtype = {kDLFloat, 32, 1};
    vel_t.shape = vel_shape;

    auto step_once = [&]() {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    };

    auto read_vels_to_host = [&](std::vector<float>& out) {
        out.resize(static_cast<size_t>(n * 6));
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_binding, &vel_t).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(out.data(), m_gpuBuffer, vel_bytes));
    };

    // Gravity is disabled on the even-indexed bodies before the first step, as in the umbrella.
    std::vector<uint8_t> flags(static_cast<size_t>(n), 0);
    for (int64_t i = 0; i < n; i += 2)
        flags[static_cast<size_t>(i)] = 1;
    int64_t flag_shape[1] = {n};
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    step_once();

    std::vector<float> vels;
    read_vels_to_host(vels);
    const float dt = 1.0f / 60.0f;
    const float expected_fall_vz = -dt * 9.81f;
    for (int64_t i = 0; i < n; ++i) {
        const float vz = vels[static_cast<size_t>(i * 6 + 2)];
        if (i % 2 == 0)
            EXPECT_NEAR(vz, 0.0f, 1e-2f) << "body " << i << " should not fall with gravity disabled";
        else
            EXPECT_NEAR(vz, expected_fall_vz, 0.05f) << "body " << i << " should fall under gravity";
    }

    ovphysx_destroy_tensor_binding(m_handle, grav_binding);
    ovphysx_destroy_tensor_binding(m_handle, vel_binding);
}

// Partial mask with a CPU-resident source on a DirectGPU scene. disable-gravity is a
// CPU-only tensor type, so the masked write scans the mask on the host, builds a CPU
// index and forwards it to the indexed setter. Empty and full masks bypass the
// indexed path and are not covered here.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableGravityPartialMaskCpuSource) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/Cube*");
    desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_DISABLE_GRAVITY_BOOL;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> flags(static_cast<size_t>(n), 0);
    std::vector<uint8_t> mask(static_cast<size_t>(n), 0);
    for (int64_t i = 0; i < n; i += 2) {
        flags[static_cast<size_t>(i)] = 1;
        mask[static_cast<size_t>(i)] = 1;
    }

    int64_t flag_shape[1] = {n};
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    flag_t.shape = flag_shape;

    int64_t mask_shape[1] = {n};
    DLTensor mask_t{};
    mask_t.data = mask.data();
    mask_t.device = {kDLCPU, 0};
    mask_t.ndim = 1;
    mask_t.dtype = {kDLUInt, 8, 1};
    mask_t.shape = mask_shape;

    ASSERT_EQ(ovphysx_write_tensor_binding_masked(m_handle, binding, &flag_t, &mask_t).status,
              OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(static_cast<size_t>(n), 0xff);
    DLTensor read_t = flag_t;
    read_t.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &read_t).status, OVPHYSX_API_SUCCESS);
    for (int64_t i = 0; i < n; ++i) {
        const uint8_t expected = (i % 2 == 0) ? 1 : 0;
        EXPECT_EQ(static_cast<int>(readback[static_cast<size_t>(i)]), static_cast<int>(expected))
            << "body " << i;
    }

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// Companion to CpuRigidBodyDisableSimulationStopsSimulation. On GPU (OMPE-103213)
// writing DISABLE_SIMULATION=1 invalidates the DirectGPU mapping, so later pose reads
// on the same binding return OVPHYSX_API_NOT_FOUND. A binding recreated over the
// still-enabled prims works.
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
    ASSERT_GT(n, 1);

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

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

    step_once();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_SUCCESS);

    // Prim paths are captured before the invalidation for the recreate below.
    std::vector<ovphysx_string_t> path_views(static_cast<size_t>(n));
    uint32_t path_count = 0;
    ASSERT_EQ(ovphysx_tensor_binding_get_prim_paths(m_handle, pose_binding, path_views.data(),
                                                   static_cast<uint32_t>(n), &path_count)
                  .status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(path_count, static_cast<uint32_t>(n));
    std::vector<std::string> all_paths;
    all_paths.reserve(path_count);
    for (uint32_t i = 0; i < path_count; ++i)
        all_paths.emplace_back(path_views[i].ptr ? path_views[i].ptr : "",
                               path_views[i].ptr ? path_views[i].length : 0);

    // Disabling the even-indexed bodies invalidates the GPU mapping.
    std::vector<uint8_t> flags(static_cast<size_t>(n), 0);
    for (int64_t i = 0; i < n; i += 2)
        flags[static_cast<size_t>(i)] = 1;
    int64_t flag_shape[1] = {n};
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 1;
    flag_t.dtype = {kDLUInt, 8, 1};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    // The disable binding's SimulationView is invalidated immediately.
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, disable_binding, &flag_t, nullptr).status, OVPHYSX_API_NOT_FOUND);
    // The sibling pose binding invalidates on the next DirectGPU op, when the refresh
    // detects the missing rows. The first read may surface ERROR, later reads NOT_FOUND.
    {
        ovphysx_api_status_t st = ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status;
        EXPECT_NE(st, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, pose_binding, &pose_t).status, OVPHYSX_API_NOT_FOUND);

    ovphysx_destroy_tensor_binding(m_handle, disable_binding);
    ovphysx_destroy_tensor_binding(m_handle, pose_binding);

    // A pose binding over the still-enabled odd prims must succeed.
    std::vector<std::string> enabled_paths;
    for (int64_t i = 1; i < n; i += 2)
        enabled_paths.push_back(all_paths[static_cast<size_t>(i)]);
    ASSERT_FALSE(enabled_paths.empty());
    std::vector<ovphysx_string_t> enabled_views(enabled_paths.size());
    for (size_t i = 0; i < enabled_paths.size(); ++i)
        enabled_views[i] = ovphysx_string_t{ enabled_paths[i].c_str(), enabled_paths[i].size() };

    ovphysx_tensor_binding_handle_t fresh_pose = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.prim_paths = enabled_views.data();
        desc.prim_paths_count = static_cast<uint32_t>(enabled_views.size());
        desc.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &fresh_pose).status, OVPHYSX_API_SUCCESS);
    }
    const int64_t n_en = static_cast<int64_t>(enabled_paths.size());
    const size_t fresh_bytes = static_cast<size_t>(n_en * 7) * sizeof(float);
    void* fresh_gpu = allocGpuBuffer(fresh_bytes, fresh_pose);
    ASSERT_NE(fresh_gpu, nullptr);
    int64_t fresh_shape[2] = {n_en, 7};
    DLTensor fresh_t{};
    fresh_t.data = fresh_gpu;
    fresh_t.device = {kDLCUDA, 0};
    fresh_t.ndim = 2;
    fresh_t.dtype = {kDLFloat, 32, 1};
    fresh_t.shape = fresh_shape;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, fresh_pose, &fresh_t).status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, fresh_pose);
}

// OMPE-103213: articulation disable-gravity is CPU-only on DirectGPU scenes too.
TEST_F(TensorBindingGpuTest, GpuArticulationDisableGravityRoundtrip) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/two_articulations_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t binding = 0;
    ovphysx_tensor_binding_desc_t desc{};
    desc.pattern = OVPHYSX_LITERAL("/World/articulation*");
    desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL;
    ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &binding).status, OVPHYSX_API_SUCCESS);

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, binding, &spec).status, OVPHYSX_API_SUCCESS);
    EXPECT_EQ(spec.ndim, 2);
    EXPECT_EQ(spec.dtype.code, kDLUInt);
    EXPECT_EQ(spec.dtype.bits, 8);
    ASSERT_GT(spec.shape[0], 0);
    ASSERT_GT(spec.shape[1], 0);

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const int64_t n = spec.shape[0];
    const int64_t l = spec.shape[1];
    const size_t bytes = static_cast<size_t>(n * l);

    std::vector<uint8_t> written(bytes, 0);
    for (size_t i = 0; i < bytes; ++i)
        written[i] = static_cast<uint8_t>(i % 2 == 0 ? 1 : 0);

    DLTensor tensor{};
    tensor.data = written.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLUInt, 8, 1};
    int64_t shape[2] = {n, l};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> readback(bytes, 0xff);
    tensor.data = readback.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);
    for (size_t i = 0; i < bytes; ++i)
        EXPECT_EQ(static_cast<int>(readback[i]), static_cast<int>(written[i])) << "link flag " << i;

    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);
    DLTensor gpu_tensor = tensor;
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr).status,
              OVPHYSX_API_DEVICE_MISMATCH);
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor).status,
              OVPHYSX_API_DEVICE_MISMATCH);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

TEST_F(TensorBindingGpuTest, GpuArticulationDisableGravitySuppressesFall) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/mixed_base_articulations_gpu.usda", usd_handle))
        << "Failed to load USD";

    ovphysx_tensor_binding_handle_t root_binding = 0;
    ovphysx_tensor_binding_handle_t grav_binding = 0;
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation2");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &root_binding).status, OVPHYSX_API_SUCCESS);
    }
    {
        ovphysx_tensor_binding_desc_t desc{};
        desc.pattern = OVPHYSX_LITERAL("/World/articulation2");
        desc.tensor_type = OVPHYSX_TENSOR_ARTICULATION_BODY_DISABLE_GRAVITY_BOOL;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &desc, &grav_binding).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t grav_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, grav_binding, &grav_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = grav_spec.shape[0];
    const int64_t l = grav_spec.shape[1];
    ASSERT_EQ(n, 1);
    ASSERT_GT(l, 0);

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t root_bytes = static_cast<size_t>(n * 7) * sizeof(float);
    void* gpu_root = allocGpuBuffer(root_bytes, root_binding);
    ASSERT_NE(gpu_root, nullptr);

    int64_t root_shape[2] = {n, 7};
    DLTensor root_t{};
    root_t.data = gpu_root;
    root_t.device = {kDLCUDA, 0};
    root_t.ndim = 2;
    root_t.dtype = {kDLFloat, 32, 1};
    root_t.shape = root_shape;

    auto step_once = [&]() {
        ovphysx_enqueue_result_t step = ovphysx_step(m_handle, 1.0f / 60.0f);
        ASSERT_EQ(step.status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(wait_op_success(m_handle, step.op_index));
    };

    auto read_root_z = [&](float& out_z) {
        std::vector<float> root_pose(static_cast<size_t>(n * 7), 0.0f);
        ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, root_binding, &root_t).status, OVPHYSX_API_SUCCESS);
        ASSERT_TRUE(m_cudaOps.memcpyDtoH(root_pose.data(), m_gpuBuffer, root_bytes));
        out_z = root_pose[2];
    };

    // Disable gravity before the first step. Stepping under gravity first would
    // leave the root coasting downward, so a stationary-height assertion could
    // be satisfied by that momentum rather than by gravity suppression.
    std::vector<uint8_t> flags(static_cast<size_t>(n * l), 1);
    int64_t flag_shape[2] = {n, l};
    DLTensor flag_t{};
    flag_t.data = flags.data();
    flag_t.device = {kDLCPU, 0};
    flag_t.ndim = 2;
    flag_t.dtype = {kDLUInt, 8, 1};
    flag_t.shape = flag_shape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);

    float z0 = 0.0f;
    read_root_z(z0);

    for (int i = 0; i < 20; ++i)
        step_once();
    float z_disabled = 0.0f;
    read_root_z(z_disabled);
    EXPECT_NEAR(z_disabled, z0, 0.01f) << "Floating-base root Z should stay stable with gravity disabled";

    // Re-enable and compare against the height reached while disabled, so the
    // fall cannot be credited to drift accumulated before gravity was restored.
    std::fill(flags.begin(), flags.end(), 0);
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, grav_binding, &flag_t, nullptr).status, OVPHYSX_API_SUCCESS);
    for (int i = 0; i < 20; ++i)
        step_once();
    float z_reenabled = 0.0f;
    read_root_z(z_reenabled);
    EXPECT_LT(z_reenabled, z_disabled - 0.05f) << "Floating-base root should fall after re-enabling gravity";

    ovphysx_destroy_tensor_binding(m_handle, root_binding);
    ovphysx_destroy_tensor_binding(m_handle, grav_binding);
}

// ARTICULATION_MASS_CENTER_WORLD read into a GPU tensor (OMPE-94459). Only the shape
// and read-success contract is checked. Numeric correctness lives in the umbrella
// tensor suite.
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

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

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

// OMPE-103213: shape properties such as ARTICULATION_REST_OFFSET are CPU-only PhysX
// APIs. Host tensors round-trip on a DirectGPU scene. CUDA tensors are refused with
// DEVICE_MISMATCH rather than silently staged to host.
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

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1]);
    const size_t bytes = total * sizeof(float);

    std::vector<float> host_in(total, 0.0005f);
    DLTensor tensor{};
    tensor.data = host_in.data();
    tensor.device = {kDLCPU, 0};
    tensor.ndim = 2;
    tensor.dtype = {kDLFloat, 32, 1};
    int64_t shape[2] = {spec.shape[0], spec.shape[1]};
    tensor.shape = shape;

    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &tensor, nullptr).status,
              OVPHYSX_API_SUCCESS);

    std::vector<float> host_out(total, 0.0f);
    tensor.data = host_out.data();
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &tensor).status, OVPHYSX_API_SUCCESS);

    // Bodies with fewer shapes have zero-padded trailing entries, so only the slots
    // the engine applied are counted.
    int matched = 0;
    for (size_t i = 0; i < total; ++i) {
        if (std::abs(host_out[i] - 0.0005f) < 1e-4f) ++matched;
    }
    EXPECT_GT(matched, 0) << "expected at least one shape to roundtrip the rest-offset write";

    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);
    DLTensor gpu_tensor = tensor;
    gpu_tensor.data = gpu_data;
    gpu_tensor.device = {kDLCUDA, 0};
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, binding, &gpu_tensor, nullptr).status,
              OVPHYSX_API_DEVICE_MISMATCH);
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, binding, &gpu_tensor).status,
              OVPHYSX_API_DEVICE_MISMATCH);

    ovphysx_destroy_tensor_binding(m_handle, binding);
}

// Multi-articulation CartPole on GPU with envs spaced so the 6m rails do not penetrate
// adjacent rails. The GPU multi-pattern view must return clean per-env projected joint
// forces on a well-formed scene. A scene with overlapping rails failing this points at
// the scene, not at the engine.
TEST_F(TensorBindingGpuTest, GpuMultiCartPoleSpacedProjectedJointForceMatchesActuation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/MultiCartPoleSpaced.usda", usd_handle))
        << "Failed to load MultiCartPoleSpaced.usda";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

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

// GPU counterpart of CpuCartPoleProjectedJointForceMatchesActuation. The projected
// joint forces must land within tolerance of the applied actuation.
TEST_F(TensorBindingGpuTest, GpuCartPoleProjectedJointForceMatchesActuation) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/CartPole.usda", usd_handle))
        << "Failed to load CartPole.usda";

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

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

// After warmup, the GPU dof-projected-joint-forces kernel must overwrite a poisoned
// destination with the real projection. Gravity along Z projects to zero on the
// cart-rail prismatic Y axis, so the expected value is about 0. Catches the kernel
// leaving the destination untouched.
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

    ASSERT_EQ(ovphysx_warmup(m_handle).status, OVPHYSX_API_SUCCESS);

    const size_t total = static_cast<size_t>(spec.shape[0] * spec.shape[1]);
    const size_t bytes = total * sizeof(float);
    void* gpu_data = allocGpuBuffer(bytes, binding);
    ASSERT_NE(gpu_data, nullptr);

    // Poison the destination so a no-op read cannot pass on a buffer that was
    // already all zeros.
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

// Standalone rigid ball with no articulation in the scene (OMPE-94459). Applies 150N
// +Z at world (ball.xy, ball.z+1), the same magnitude and anchor as the multi-body
// reproducer, so the lift here isolates direct rigid-body force application from
// articulation-ball contact.
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

// Standalone rigid-body force-at-position must be translation-invariant and produce
// the expected lift on CPU and GPU at both world positions (OMPE-94459). Over 10 steps
// at dt=1/60s with mass=0.5kg, gravity=20m/s^2 and a single F=150N impulse the lift is
// about 0.528m. All four variants agree to within 1mm.
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

namespace
{
} // namespace

namespace
{
} // namespace

namespace
{
} // namespace

// CartPole pre-step DOF-projected joint force must be about 0 regardless of world
// position (OMPE-94459). CpuCartPoleProjectedJointForcePreStepZero covers the world
// origin. These variants cover world (100, 100, 0), which gates the Rw seed precision
// for prismatic and revolute chains far from the origin.
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

// Minimal 2-link articulation with a PhysicsSphericalJoint between a kinematic base
// and a free child, loaded at the world origin and at (100, 100, 0) (OMPE-94459). The
// child settles to the joint constraint at the same env-relative position in both.
// This is the only coverage of the spherical branch of computeLinkRFromJointState.
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

    // With aligned anchors and gravity only there is no swing, so the child ends at
    // world X = env_x and Y = env_y. Both variants have a zero env Z offset, so the
    // returned world Z equals the env-local Z.
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
    // The base anchor is at z=1.9 (kinematic base at z=2 minus 0.1) and the child
    // anchor is 0.2 above the child center, so the settled child center is at
    // world z=1.7. The loose tolerance covers the iterative settle.
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

// OMPE-103213: after DISABLE_SIMULATION on GPU, velocity reads on the same binding
// return OVPHYSX_API_NOT_FOUND because the DirectGPU mapping is invalidated.
TEST_F(TensorBindingGpuTest, GpuVelocityReadbackOverDisabledBodies) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane_gpu.usda";

    ovphysx_tensor_binding_handle_t vel_b = 0;
    ovphysx_tensor_binding_handle_t dis_b = 0;
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
    ASSERT_GT(n_bodies, 1);

    const size_t vel_bytes = n_bodies * 6 * sizeof(float);
    void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
    ASSERT_NE(gpu_buf, nullptr);

    DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
    vt.dtype = {kDLFloat, 32, 1};
    int64_t vel_shape[2] = {n_bodies, 6};
    vt.shape = vel_shape; vt.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> dis_buf(n_bodies, 0);
    for (int64_t i = 0; i < n_bodies; i += 2) dis_buf[i] = 1;
    DLTensor dt{}; dt.data = dis_buf.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1};
    int64_t dis_shape[1] = {n_bodies};
    dt.shape = dis_shape; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    // The disable binding is stale immediately. The velocity sibling fails on the
    // next DirectGPU op.
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_NOT_FOUND);
    {
        ovphysx_api_status_t st = ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status;
        EXPECT_NE(st, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_NOT_FOUND);

    ovphysx_destroy_tensor_binding(m_handle, dis_b);
    ovphysx_destroy_tensor_binding(m_handle, vel_b);
}

// OMPE-103213: a second disable toggle cannot run on the same GPU binding after
// the first disable invalidated the mapping. Subsequent writes/reads return
// OVPHYSX_API_NOT_FOUND.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableSimulationSecondToggleHonored) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane_gpu.usda";

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

    std::vector<uint8_t> even_disabled(n, 0), odd_disabled(n, 0);
    for (int64_t i = 0; i < n; ++i) (i % 2 == 0 ? even_disabled : odd_disabled)[i] = 1;

    DLTensor dt{}; dt.data = even_disabled.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1};
    int64_t sh[1] = {n}; dt.shape = sh; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    // Second toggle on the stale disable binding must fail.
    dt.data = odd_disabled.data();
    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_NOT_FOUND);

    const size_t vel_bytes = n * 6 * sizeof(float);
    void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
    ASSERT_NE(gpu_buf, nullptr);
    DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
    vt.dtype = {kDLFloat, 32, 1};
    int64_t vsh[2] = {n, 6}; vt.shape = vsh; vt.ndim = 2;
    {
        ovphysx_api_status_t st = ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status;
        EXPECT_NE(st, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_NOT_FOUND);

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-103213: flip+wake sequence on the same GPU binding is not supported after
// the first disable invalidates the DirectGPU mapping.
TEST_F(TensorBindingGpuTest, GpuRigidBodyDisableFlipWakeSequence) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle));

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

    std::vector<int32_t> idx_A;
    for (int32_t i = 0; i < int32_t(n); i += 2) idx_A.push_back(i);

    std::vector<uint8_t> flags(n, 1);
    DLTensor ft{}; ft.data = flags.data(); ft.device = {kDLCPU, 0};
    ft.dtype = {kDLUInt, 8, 1}; ft.ndim = 1; int64_t fs[1] = {n}; ft.shape = fs;
    DLTensor it{}; it.data = idx_A.data(); it.device = {kDLCPU, 0};
    it.dtype = {kDLInt, 32, 1}; it.ndim = 1; int64_t ishape[1] = {int64_t(idx_A.size())}; it.shape = ishape;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &ft, &it).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &ft, &it).status, OVPHYSX_API_NOT_FOUND);

    const size_t vel_bytes = n * 6 * sizeof(float);
    void* gpu_buf = allocGpuBuffer(vel_bytes, vel_b);
    ASSERT_NE(gpu_buf, nullptr);
    DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
    vt.dtype = {kDLFloat, 32, 1}; vt.ndim = 2; int64_t vs[2] = {n, 6}; vt.shape = vs;
    {
        ovphysx_api_status_t st = ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status;
        EXPECT_NE(st, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, vel_b, &vt).status, OVPHYSX_API_NOT_FOUND);

    ovphysx_destroy_tensor_binding(m_handle, vel_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-103213: disabled-body pose readback on the same GPU binding is refused because
// the mapping is stale. Callers must recreate the binding for the enabled set.
TEST_F(TensorBindingGpuTest, GpuDisabledBodyPoseReadbackIsSane) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle));

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

    void* gpu_buf = allocGpuBuffer(pose_bytes, pose_b);
    ASSERT_NE(gpu_buf, nullptr);
    DLTensor pt{}; pt.data = gpu_buf; pt.device = {kDLCUDA, 0};
    pt.dtype = {kDLFloat, 32, 1}; pt.ndim = 2; int64_t sh[2] = {n, 7}; pt.shape = sh;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, pose_b, &pt).status, OVPHYSX_API_SUCCESS);

    std::vector<uint8_t> dis(n, 0);
    for (int64_t i = 0; i < n; i += 2) dis[i] = 1;
    DLTensor dt{}; dt.data = dis.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1}; int64_t ds[1] = {n}; dt.shape = ds; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    EXPECT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_NOT_FOUND);
    {
        ovphysx_api_status_t st = ovphysx_read_tensor_binding(m_handle, pose_b, &pt).status;
        EXPECT_NE(st, OVPHYSX_API_SUCCESS);
    }
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, pose_b, &pt).status, OVPHYSX_API_NOT_FOUND);

    ovphysx_destroy_tensor_binding(m_handle, pose_b);
    ovphysx_destroy_tensor_binding(m_handle, dis_b);
}

// OMPE-103213: wake_up is CPU-only, so a GPU index tensor is refused rather than
// silently staged to host. Host indices still wake the selected subset. No disable
// is used here because it would invalidate the GPU binding.
TEST_F(TensorBindingGpuTest, GpuRigidBodyWakeUpRequiresHostIndices) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle));

    ovphysx_tensor_binding_handle_t vel_b = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &vel_b).status, OVPHYSX_API_SUCCESS);
    }

    ovphysx_tensor_spec_t spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_b, &spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n = spec.shape[0];
    ASSERT_GT(n, 1);
    const size_t vel_bytes = n * 6 * sizeof(float);

    std::vector<int32_t> even;
    for (int32_t i = 0; i < int32_t(n); i += 2) even.push_back(i);

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

    // All bodies are put to sleep, then a subset is woken via host indices.
    ASSERT_EQ(ovphysx_rigid_body_view_sleep(m_handle, vel_b, nullptr).status, OVPHYSX_API_SUCCESS);

    (void)allocGpuBuffer(vel_bytes, vel_b);
    uintptr_t idxDev = 0;
    int allocSt = 0;
    ASSERT_TRUE(m_cudaOps.memAlloc(even.size() * sizeof(int32_t), &idxDev, &allocSt));
    ASSERT_TRUE(m_cudaOps.memcpyHtoD(idxDev, even.data(), even.size() * sizeof(int32_t)));
    DLTensor idxGpu{}; idxGpu.data = reinterpret_cast<void*>(idxDev); idxGpu.device = {kDLCUDA, 0};
    idxGpu.dtype = {kDLInt, 32, 1}; idxGpu.ndim = 1; int64_t isGpu[1] = {int64_t(even.size())}; idxGpu.shape = isGpu;
    EXPECT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, &idxGpu).status, OVPHYSX_API_INVALID_ARGUMENT)
        << "wake_up with a CUDA int32 index tensor must be refused (CPU-only API)";

    DLTensor idxHost{}; idxHost.data = even.data(); idxHost.device = {kDLCPU, 0};
    idxHost.dtype = {kDLInt, 32, 1}; idxHost.ndim = 1; int64_t isHost[1] = {int64_t(even.size())}; idxHost.shape = isHost;
    ASSERT_EQ(ovphysx_rigid_body_view_wake_up(m_handle, vel_b, &idxHost).status, OVPHYSX_API_SUCCESS);

    step();
    std::vector<float> vz; read_vz(vz);
    for (int32_t i : even)
        EXPECT_LT(vz[i], -0.01f)
            << "body " << i << " woken via host indices should be falling, got " << vz[i];

    (void)m_cudaOps.memFree(idxDev);
    ovphysx_destroy_tensor_binding(m_handle, vel_b);
}

// OMPE-103213: creating a velocity binding whose pattern also matches disabled
// rigid dynamics omits them and yields a valid enabled-only binding.
TEST_F(TensorBindingGpuTest, GpuVelocityReadbackBindingAfterDisable) {
    ovphysx_usd_handle_t usd_handle = 0;
    ASSERT_TRUE(load_usd_and_wait(m_handle, "tests/data/boxes_falling_on_groundplane_gpu.usda", usd_handle))
        << "Failed to load boxes_falling_on_groundplane_gpu.usda";

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
    ASSERT_GT(n_bodies, 1);

    std::vector<ovphysx_string_t> path_views(static_cast<size_t>(n_bodies));
    uint32_t path_count = 0;
    ASSERT_EQ(ovphysx_tensor_binding_get_prim_paths(m_handle, dis_b, path_views.data(),
                                                   static_cast<uint32_t>(n_bodies), &path_count)
                  .status,
              OVPHYSX_API_SUCCESS);
    ASSERT_EQ(path_count, static_cast<uint32_t>(n_bodies));
    std::vector<std::string> all_paths;
    for (uint32_t i = 0; i < path_count; ++i)
        all_paths.emplace_back(path_views[i].ptr ? path_views[i].ptr : "",
                               path_views[i].ptr ? path_views[i].length : 0);

    std::vector<uint8_t> dis_buf(n_bodies, 0);
    for (int64_t i = 0; i < n_bodies; i += 2) dis_buf[i] = 1;
    DLTensor dt{}; dt.data = dis_buf.data(); dt.device = {kDLCPU, 0};
    dt.dtype = {kDLUInt, 8, 1};
    int64_t dis_shape[1] = {n_bodies};
    dt.shape = dis_shape; dt.ndim = 1;
    ASSERT_EQ(ovphysx_write_tensor_binding(m_handle, dis_b, &dt, nullptr).status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, dis_b);

    // Recreate with the original wildcard. DirectGPU create omits the disabled
    // bodies, so the binding remains valid and reports the enabled-only shape.
    ovphysx_tensor_binding_handle_t vel_all = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.pattern = OVPHYSX_LITERAL("/World/Cube*");
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &vel_all).status, OVPHYSX_API_SUCCESS);
    }
    ovphysx_tensor_spec_t vel_spec{};
    ASSERT_EQ(ovphysx_get_tensor_binding_spec(m_handle, vel_all, &vel_spec).status, OVPHYSX_API_SUCCESS);
    const int64_t n_enabled = n_bodies / 2;
    ASSERT_EQ(vel_spec.shape[0], n_enabled);

    const size_t vel_bytes = static_cast<size_t>(n_enabled) * 6 * sizeof(float);
    void* gpu_buf = allocGpuBuffer(vel_bytes, vel_all);
    ASSERT_NE(gpu_buf, nullptr);
    DLTensor vt{}; vt.data = gpu_buf; vt.device = {kDLCUDA, 0};
    vt.dtype = {kDLFloat, 32, 1};
    int64_t vel_shape[2] = {n_enabled, 6};
    vt.shape = vel_shape; vt.ndim = 2;
    EXPECT_EQ(ovphysx_read_tensor_binding(m_handle, vel_all, &vt).status, OVPHYSX_API_SUCCESS);
    ovphysx_destroy_tensor_binding(m_handle, vel_all);

    // Enabled-only recreate must work.
    std::vector<std::string> enabled_paths;
    for (int64_t i = 1; i < n_bodies; i += 2)
        enabled_paths.push_back(all_paths[static_cast<size_t>(i)]);
    std::vector<ovphysx_string_t> enabled_views(enabled_paths.size());
    for (size_t i = 0; i < enabled_paths.size(); ++i)
        enabled_views[i] = ovphysx_string_t{ enabled_paths[i].c_str(), enabled_paths[i].size() };

    ovphysx_tensor_binding_handle_t vel_en = 0;
    {
        ovphysx_tensor_binding_desc_t d{};
        d.prim_paths = enabled_views.data();
        d.prim_paths_count = static_cast<uint32_t>(enabled_views.size());
        d.tensor_type = OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32;
        ASSERT_EQ(ovphysx_create_tensor_binding(m_handle, &d, &vel_en).status, OVPHYSX_API_SUCCESS);
    }
    const int64_t n_en = static_cast<int64_t>(enabled_paths.size());
    const size_t en_bytes = n_en * 6 * sizeof(float);
    void* en_gpu = allocGpuBuffer(en_bytes, vel_en);
    ASSERT_NE(en_gpu, nullptr);
    DLTensor vt_en{}; vt_en.data = en_gpu; vt_en.device = {kDLCUDA, 0};
    vt_en.dtype = {kDLFloat, 32, 1};
    int64_t en_shape[2] = {n_en, 6};
    vt_en.shape = en_shape; vt_en.ndim = 2;
    ASSERT_EQ(ovphysx_read_tensor_binding(m_handle, vel_en, &vt_en).status, OVPHYSX_API_SUCCESS);

    ovphysx_destroy_tensor_binding(m_handle, vel_en);
}
