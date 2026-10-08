// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the control-step/create probes use the binding. They retire with it.

/**
 * @implements REQ-CAPI-BENCHMARK-003
 * @covers AC-1 AC-2
 */

// Physics-owned Cartpole-4096 OVPhysX backend probes. The control row measures
// two control/simulation substeps followed by joint-state reads. The binding
// row measures creation and spec lookup for its five persistent bindings.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../OvstageLoad.h"

#include "ovphysx/dlpack/dlpack.h"
#include "ovphysx/experimental/TensorBinding.hpp"
#include <ovphysx/experimental/ovphysx.hpp>

#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
#include <cuda_runtime_api.h>
#endif


void initLabCartpole()
{
}

namespace
{

constexpr uint32_t kEnvCount = 4096;
constexpr uint32_t kDofCount = 2;
constexpr uint32_t kGridWidth = 64;
constexpr float kPhysicsDt = 1.0f / 120.0f;
constexpr int32_t kCudaDevice = 0;
constexpr const char* kArticulationPattern = "/World/envs/env_*/Robot";

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)

std::string lastError()
{
    const ovphysx_string_t error = ovphysx_get_last_error();
    if (error.ptr == nullptr || error.length == 0)
    {
        return std::string();
    }
    return std::string(error.ptr, error.length);
}

void requireSuccess(ovphysx_api_status_t status, const char* operation)
{
    if (status == OVPHYSX_API_SUCCESS)
    {
        return;
    }

    const std::string detail = lastError();
    throw std::runtime_error(
        std::string(operation) + " failed with status " + std::to_string(static_cast<int>(status)) +
        (detail.empty() ? std::string() : std::string(": ") + detail));
}

void requireSuccess(const ovphysx_result_t& result, const char* operation)
{
    requireSuccess(result.status, operation);
}

void requireCuda(cudaError_t status, const char* operation)
{
    if (status != cudaSuccess)
    {
        throw std::runtime_error(std::string(operation) + " failed: " + cudaGetErrorString(status));
    }
}

void validateDofSpec(const ovphysx_tensor_spec_t& spec, const char* bindingName)
{
    const bool isFloat32 = spec.dtype.code == static_cast<uint8_t>(kDLFloat) && spec.dtype.bits == 32 &&
                           spec.dtype.lanes == 1;
    if (!isFloat32 || spec.ndim != 2)
    {
        throw std::runtime_error(std::string(bindingName) + " must resolve to rank-2 float32[4096,2]");
    }
    if (spec.shape[0] != kEnvCount || spec.shape[1] != kDofCount)
    {
        throw std::runtime_error(
            std::string(bindingName) + " must resolve to float32[4096,2], got ndim=" +
            std::to_string(spec.ndim) + " shape=[" + std::to_string(spec.shape[0]) + "," +
            std::to_string(spec.shape[1]) + "]");
    }
}

class CudaTensor
{
public:
    CudaTensor() = default;
    CudaTensor(const CudaTensor&) = delete;
    CudaTensor& operator=(const CudaTensor&) = delete;

    ~CudaTensor()
    {
        if (mData != nullptr)
        {
            (void)cudaSetDevice(mDeviceId);
            (void)cudaFree(mData);
        }
    }

    void allocate(const ovphysx_tensor_spec_t& spec, int32_t deviceId)
    {
        mShape.assign(spec.shape, spec.shape + spec.ndim);
        mDtype = spec.dtype;
        mDeviceId = deviceId;

        size_t elementCount = 1;
        for (int64_t dimension : mShape)
        {
            elementCount *= static_cast<size_t>(dimension);
        }
        mByteCount = elementCount * static_cast<size_t>(mDtype.bits / 8) * mDtype.lanes;

        requireCuda(cudaSetDevice(mDeviceId), "cudaSetDevice");
        requireCuda(cudaMalloc(&mData, mByteCount), "cudaMalloc");
    }

    void copyFromHost(const void* source, size_t byteCount)
    {
        if (byteCount != mByteCount)
        {
            throw std::runtime_error("CUDA tensor host upload has the wrong byte count");
        }
        requireCuda(cudaMemcpy(mData, source, mByteCount, cudaMemcpyHostToDevice), "cudaMemcpy host to device");
    }

    void copyToHost(void* destination, size_t byteCount) const
    {
        if (byteCount != mByteCount)
        {
            throw std::runtime_error("CUDA tensor host readback has the wrong byte count");
        }
        requireCuda(cudaMemcpy(destination, mData, mByteCount, cudaMemcpyDeviceToHost), "cudaMemcpy device to host");
    }

    void zero()
    {
        requireCuda(cudaMemset(mData, 0, mByteCount), "cudaMemset");
    }

    DLTensor view()
    {
        DLTensor tensor{};
        tensor.data = mData;
        tensor.device = DLDevice{kDLCUDA, mDeviceId};
        tensor.ndim = static_cast<int32_t>(mShape.size());
        tensor.dtype = mDtype;
        tensor.shape = mShape.data();
        tensor.strides = nullptr;
        tensor.byte_offset = 0;
        return tensor;
    }

private:
    void* mData = nullptr;
    size_t mByteCount = 0;
    int32_t mDeviceId = 0;
    DLDataType mDtype{};
    std::vector<int64_t> mShape;
};

class CartpoleControlStep : public BmBenchmark
{
public:
    ~CartpoleControlStep() override
    {
        mEffortBinding.destroy();
        mPositionTargetBinding.destroy();
        mVelocityTargetBinding.destroy();
        mPositionBinding.destroy();
        mVelocityBinding.destroy();
        if (mStageLoaded && mPhysX != nullptr)
        {
            benchmarkClearOvstage(mPhysX, mStageAttachment);
        }
        if (mLogCallbackRegistered)
        {
            (void)ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
        }
    }

    bool isValid() const override
    {
        const BmGlobals& globals = BmGlobals::getInstance();
        if (!globals.forceGpu())
        {
            throw std::runtime_error("Cartpole probe requires --forceGpu");
        }
        if (!globals.directGpu())
        {
            throw std::runtime_error("Cartpole probe requires --directGpu");
        }
        if (globals.getPhysX() == nullptr)
        {
            throw std::runtime_error("Cartpole probe failed to initialize OVPhysX");
        }
        return true;
    }

    uint32_t getNbSteps() const override
    {
        return 20;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
        if (!mInitialized)
        {
            setup();
            mInitialized = true;
        }
        resetDofState();
    }

    void preStep() override
    {
    }

    void endRun() override
    {
    }

protected:
    void step() override
    {
        runControlStep();
    }

private:
    void setup()
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (mPhysX == nullptr)
        {
            throw std::runtime_error("Cartpole probe has no OVPhysX instance");
        }
        requireSuccess(
            ovphysx_set_log_callback(OVPHYSX_LOG_ERROR, nullptr, logCallback, this),
            "set Cartpole error callback");
        mLogCallbackRegistered = true;

        const std::string fixturePath =
            BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/cartpole_probe.usda";
        if (!benchmarkLoadUsdWithOvstage(mPhysX, fixturePath, mStageAttachment))
        {
            throw std::runtime_error("Cartpole probe failed to load " + fixturePath);
        }
        mStageLoaded = true;

        std::vector<std::string> targets;
        targets.reserve(kEnvCount - 1);
        std::vector<float> transforms(static_cast<size_t>(kEnvCount - 1) * 7, 0.0f);
        for (uint32_t envIndex = 1; envIndex < kEnvCount; ++envIndex)
        {
            targets.emplace_back("/World/envs/env_" + std::to_string(envIndex));
            float* transform = transforms.data() + static_cast<size_t>(envIndex - 1) * 7;
            transform[0] = static_cast<float>(envIndex % kGridWidth) * 4.0f;
            transform[2] = static_cast<float>(envIndex / kGridWidth) * 4.0f;
            transform[6] = 1.0f;
        }

        requireSuccess(mPhysX->clone("/World/envs/env_0", targets, transforms.data()), "clone Cartpole environments");
        ovphysx::physx::WaitResult cloneWait = mPhysX->waitAll();
        if (cloneWait.hasErrors())
        {
            throw std::runtime_error("Cartpole environment clone completed with asynchronous errors");
        }

        mCudaDevice = kCudaDevice;
        requireCuda(cudaSetDevice(mCudaDevice), "cudaSetDevice");
        requireSuccess(ovphysx_warmup(mPhysX->handle()), "warm up Cartpole GPU scene");

        createGpuBinding(
            OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32,
            "actuation force",
            mEffortBinding,
            mEffortBuffer,
            mEffortTensor);
        ovphysx_articulation_metadata_t metadata{};
        requireSuccess(mEffortBinding.metadata(metadata), "query Cartpole articulation topology");
        if (metadata.dof_count != static_cast<int32_t>(kDofCount) || metadata.body_count != 3 ||
            !metadata.is_fixed_base)
        {
            throw std::runtime_error("Cartpole articulation must be fixed-base with 3 bodies and 2 DOFs");
        }
        createGpuBinding(
            OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32,
            "position target",
            mPositionTargetBinding,
            mPositionTargetBuffer,
            mPositionTargetTensor);
        createGpuBinding(
            OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32,
            "velocity target",
            mVelocityTargetBinding,
            mVelocityTargetBuffer,
            mVelocityTargetTensor);
        createGpuBinding(
            OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32,
            "DOF position",
            mPositionBinding,
            mPositionBuffer,
            mPositionTensor);
        createGpuBinding(
            OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32,
            "DOF velocity",
            mVelocityBinding,
            mVelocityBuffer,
            mVelocityTensor);

        validateDriveProperty(OVPHYSX_TENSOR_ARTICULATION_DOF_DAMPING_F32, 10.0f, 0.0f, "drive damping");
        validateDriveProperty(OVPHYSX_TENSOR_ARTICULATION_DOF_MAX_FORCE_F32, 400.0f, 400.0f, "drive max force");

        std::vector<float> efforts(static_cast<size_t>(kEnvCount) * kDofCount, 0.0f);
        std::vector<float> reverseEfforts(static_cast<size_t>(kEnvCount) * kDofCount, 0.0f);
        for (uint32_t envIndex = 0; envIndex < kEnvCount; ++envIndex)
        {
            const size_t offset = static_cast<size_t>(envIndex) * kDofCount;
            const float effort = (envIndex % 2 == 0) ? 100.0f : -100.0f;
            efforts[offset] = effort;
            reverseEfforts[offset] = -effort;
        }
        mEffortBuffer.copyFromHost(efforts.data(), efforts.size() * sizeof(float));
        ovphysx_tensor_spec_t effortSpec{};
        requireSuccess(mEffortBinding.spec(effortSpec), "query actuation force spec");
        mReverseEffortBuffer.allocate(effortSpec, mCudaDevice);
        mReverseEffortBuffer.copyFromHost(reverseEfforts.data(), reverseEfforts.size() * sizeof(float));
        mReverseEffortTensor = mReverseEffortBuffer.view();
        mPositionTargetBuffer.zero();
        mVelocityTargetBuffer.zero();

        validateControlResponse();
        failOnLoggedError("Cartpole setup");
    }

    void createGpuBinding(ovphysx_tensor_type_t tensorType,
                          const char* bindingName,
                          ovphysx::TensorBinding& binding,
                          CudaTensor& buffer,
                          DLTensor& tensor)
    {
        requireSuccess(mPhysX->createTensorBinding(binding, kArticulationPattern, tensorType), bindingName);
        ovphysx_tensor_spec_t spec{};
        requireSuccess(binding.spec(spec), bindingName);
        validateDofSpec(spec, bindingName);
        buffer.allocate(spec, mCudaDevice);
        tensor = buffer.view();
    }

    void validateDriveProperty(ovphysx_tensor_type_t tensorType,
                               float expectedCart,
                               float expectedPole,
                               const char* bindingName)
    {
        ovphysx::TensorBinding binding;
        requireSuccess(mPhysX->createTensorBinding(binding, kArticulationPattern, tensorType), bindingName);

        ovphysx_tensor_spec_t spec{};
        requireSuccess(binding.spec(spec), bindingName);
        validateDofSpec(spec, bindingName);

        std::vector<float> values(static_cast<size_t>(kEnvCount) * kDofCount, 0.0f);
        int64_t shape[2] = {kEnvCount, kDofCount};
        DLTensor tensor{};
        tensor.data = values.data();
        tensor.device = DLDevice{kDLCPU, 0};
        tensor.ndim = 2;
        tensor.dtype = spec.dtype;
        tensor.shape = shape;
        tensor.strides = nullptr;
        tensor.byte_offset = 0;
        requireSuccess(binding.read(tensor), bindingName);

        for (uint32_t envIndex = 0; envIndex < kEnvCount; ++envIndex)
        {
            const size_t offset = static_cast<size_t>(envIndex) * kDofCount;
            if (std::fabs(values[offset] - expectedCart) > 1.0e-4f ||
                std::fabs(values[offset + 1] - expectedPole) > 1.0e-4f)
            {
                throw std::runtime_error(std::string(bindingName) + " does not match the authored Cartpole drives");
            }
        }
    }

    void resetDofState()
    {
        mPositionBuffer.zero();
        mVelocityBuffer.zero();
        requireSuccess(mPositionBinding.write(mPositionTensor), "reset DOF position");
        requireSuccess(mVelocityBinding.write(mVelocityTensor), "reset DOF velocity");
        requireSuccess(mPhysX->updateArticulationsKinematic(), "refresh Cartpole kinematics after reset");
        mUseReverseEffort = false;
        failOnLoggedError("Cartpole reset");
    }

    void runControlStep()
    {
        DLTensor& effortTensor = mUseReverseEffort ? mReverseEffortTensor : mEffortTensor;
        for (uint32_t substep = 0; substep < 2; ++substep)
        {
            requireSuccess(mEffortBinding.write(effortTensor), "write Cartpole effort");
            requireSuccess(mPositionTargetBinding.write(mPositionTargetTensor), "write Cartpole position target");
            requireSuccess(mVelocityTargetBinding.write(mVelocityTargetTensor), "write Cartpole velocity target");
            requireSuccess(ovphysx_step_sync(mPhysX->handle(), kPhysicsDt), "step Cartpole simulation");
            requireSuccess(mPhysX->updateArticulationsKinematic(), "refresh Cartpole kinematics");
        }

        requireSuccess(mPositionBinding.read(mPositionTensor), "read Cartpole DOF position");
        requireSuccess(mVelocityBinding.read(mVelocityTensor), "read Cartpole DOF velocity");
        mUseReverseEffort = !mUseReverseEffort;
        failOnLoggedError("Cartpole control step");
    }

    static void logCallback(
        ovphysx_log_level_t level,
        ovphysx_string_t,
        ovphysx_string_t,
        double,
        void* userData)
    {
        if (level == OVPHYSX_LOG_ERROR && userData != nullptr)
        {
            static_cast<CartpoleControlStep*>(userData)->mLoggedError.store(true, std::memory_order_relaxed);
        }
    }

    void failOnLoggedError(const char* operation) const
    {
        if (mLoggedError.load(std::memory_order_relaxed))
        {
            throw std::runtime_error(std::string(operation) + " emitted an OVPhysX error");
        }
    }

    void validateControlResponse()
    {
        resetDofState();
        runControlStep();

        std::vector<float> positions(static_cast<size_t>(kEnvCount) * kDofCount, 0.0f);
        std::vector<float> velocities(static_cast<size_t>(kEnvCount) * kDofCount, 0.0f);
        mPositionBuffer.copyToHost(positions.data(), positions.size() * sizeof(float));
        mVelocityBuffer.copyToHost(velocities.data(), velocities.size() * sizeof(float));

        for (uint32_t envIndex = 0; envIndex < kEnvCount; ++envIndex)
        {
            const size_t offset = static_cast<size_t>(envIndex) * kDofCount;
            for (uint32_t dofIndex = 0; dofIndex < kDofCount; ++dofIndex)
            {
                if (!std::isfinite(positions[offset + dofIndex]) || !std::isfinite(velocities[offset + dofIndex]))
                {
                    throw std::runtime_error("Cartpole preflight produced non-finite joint state");
                }
            }

            const float cartVelocity = velocities[offset];
            const bool expectedPositive = envIndex % 2 == 0;
            if (std::fabs(cartVelocity) < 1.0e-4f || (expectedPositive && cartVelocity <= 0.0f) ||
                (!expectedPositive && cartVelocity >= 0.0f))
            {
                throw std::runtime_error("Cartpole preflight produced an invalid cart response");
            }
        }
    }

    bool mInitialized = false;
    bool mStageLoaded = false;
    bool mLogCallbackRegistered = false;
    bool mUseReverseEffort = false;
    std::atomic<bool> mLoggedError{false};
    int mCudaDevice = 0;
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};

    ovphysx::TensorBinding mEffortBinding;
    ovphysx::TensorBinding mPositionTargetBinding;
    ovphysx::TensorBinding mVelocityTargetBinding;
    ovphysx::TensorBinding mPositionBinding;
    ovphysx::TensorBinding mVelocityBinding;

    CudaTensor mEffortBuffer;
    CudaTensor mReverseEffortBuffer;
    CudaTensor mPositionTargetBuffer;
    CudaTensor mVelocityTargetBuffer;
    CudaTensor mPositionBuffer;
    CudaTensor mVelocityBuffer;

    DLTensor mEffortTensor{};
    DLTensor mReverseEffortTensor{};
    DLTensor mPositionTargetTensor{};
    DLTensor mVelocityTargetTensor{};
    DLTensor mPositionTensor{};
    DLTensor mVelocityTensor{};
};

struct CartpoleBindingCase
{
    ovphysx_tensor_type_t tensorType;
    const char* name;
};

constexpr std::array<CartpoleBindingCase, 5> kCartpoleBindingCases = { {
    { OVPHYSX_TENSOR_ARTICULATION_DOF_ACTUATION_FORCE_F32, "actuation force" },
    { OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_TARGET_F32, "position target" },
    { OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_TARGET_F32, "velocity target" },
    { OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32, "DOF position" },
    { OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32, "DOF velocity" },
} };

class CartpoleTensorBindingCreate : public BmBenchmark
{
public:
    ~CartpoleTensorBindingCreate() override
    {
        destroyBindings(false);
        if (mStageLoaded && mPhysX != nullptr)
        {
            benchmarkClearOvstage(mPhysX, mStageAttachment);
        }
        if (mLogCallbackRegistered)
        {
            (void)ovphysx_set_log_callback(OVPHYSX_LOG_DEFAULT, nullptr, nullptr, nullptr);
        }
    }

    bool isValid() const override
    {
        const BmGlobals& globals = BmGlobals::getInstance();
        if (!globals.forceGpu())
        {
            throw std::runtime_error("Cartpole TensorBinding creation probe requires --forceGpu");
        }
        if (!globals.directGpu())
        {
            throw std::runtime_error("Cartpole TensorBinding creation probe requires --directGpu");
        }
        if (globals.getPhysX() == nullptr)
        {
            throw std::runtime_error("Cartpole TensorBinding creation probe failed to initialize OVPhysX");
        }
        return true;
    }

    uint32_t getNbSteps() const override
    {
        return 1;
    }

    uint32_t getNbRuns() const override
    {
        // The harness skips its dummy run when this is zero, preserving first-use creation.
        return 0;
    }

    void startRun() override
    {
        if (!mInitialized)
        {
            setup();
            mInitialized = true;
        }
    }

    void preStep() override
    {
    }

    Time::Second timedStep() override
    {
        if (mMeasured)
        {
            throw std::runtime_error("Cartpole TensorBinding creation probe must run once per process");
        }
        mMeasured = true;

        Time timer;
        for (size_t bindingIndex = 0; bindingIndex < kCartpoleBindingCases.size(); ++bindingIndex)
        {
            const CartpoleBindingCase& bindingCase = kCartpoleBindingCases[bindingIndex];
            requireSuccess(
                mPhysX->createTensorBinding(mBindings[bindingIndex], kArticulationPattern, bindingCase.tensorType),
                bindingCase.name);
            requireSuccess(mBindings[bindingIndex].spec(mSpecs[bindingIndex]), bindingCase.name);
        }
        const Time::Second elapsed = timer.getElapsedSeconds();

        validateBindings();
        return elapsed;
    }

    void endRun() override
    {
        destroyBindings(true);
        failOnLoggedError("Cartpole TensorBinding creation teardown");
    }

protected:
    void step() override
    {
    }

private:
    void setup()
    {
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (mPhysX == nullptr)
        {
            throw std::runtime_error("Cartpole TensorBinding creation probe has no OVPhysX instance");
        }
        requireSuccess(ovphysx_set_log_callback(OVPHYSX_LOG_ERROR, nullptr, logCallback, this),
                       "set Cartpole TensorBinding creation error callback");
        mLogCallbackRegistered = true;

        const std::string fixturePath =
            BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/cartpole_probe.usda";
        if (!ovphysx_sample_attach_usd_with_ovstage(mPhysX->handle(), fixturePath.c_str(), &mStageAttachment))
        {
            throw std::runtime_error("Cartpole TensorBinding creation probe failed to load " + fixturePath);
        }
        mStageLoaded = true;
        waitForAll("complete Cartpole stage attach");

        std::vector<std::string> targets;
        targets.reserve(kEnvCount - 1);
        std::vector<float> transforms(static_cast<size_t>(kEnvCount - 1) * 7, 0.0f);
        for (uint32_t envIndex = 1; envIndex < kEnvCount; ++envIndex)
        {
            targets.emplace_back("/World/envs/env_" + std::to_string(envIndex));
            float* transform = transforms.data() + static_cast<size_t>(envIndex - 1) * 7;
            transform[0] = static_cast<float>(envIndex % kGridWidth) * 4.0f;
            transform[2] = static_cast<float>(envIndex / kGridWidth) * 4.0f;
            transform[6] = 1.0f;
        }

        requireSuccess(mPhysX->clone("/World/envs/env_0", targets, transforms.data()), "clone Cartpole environments");
        waitForAll("complete Cartpole environment clone");

        requireCuda(cudaSetDevice(kCudaDevice), "cudaSetDevice");
        requireSuccess(ovphysx_warmup(mPhysX->handle()), "warm up Cartpole GPU scene");
        failOnLoggedError("Cartpole TensorBinding creation setup");
    }

    void validateBindings()
    {
        for (size_t bindingIndex = 0; bindingIndex < kCartpoleBindingCases.size(); ++bindingIndex)
        {
            validateDofSpec(mSpecs[bindingIndex], kCartpoleBindingCases[bindingIndex].name);
        }

        ovphysx_articulation_metadata_t metadata{};
        requireSuccess(mBindings[0].metadata(metadata), "query Cartpole articulation topology");
        const int64_t instanceCount = mSpecs[0].shape[0];
        if (instanceCount != static_cast<int64_t>(kEnvCount) || metadata.dof_count != static_cast<int32_t>(kDofCount) ||
            metadata.body_count != 3 || !metadata.is_fixed_base)
        {
            throw std::runtime_error(
                "Cartpole binding must contain exactly 4096 fixed-base articulations with 3 bodies and 2 DOFs");
        }
        failOnLoggedError("Cartpole TensorBinding creation");
    }

    static void logCallback(ovphysx_log_level_t level, ovphysx_string_t, ovphysx_string_t, double, void* userData)
    {
        if (level == OVPHYSX_LOG_ERROR && userData != nullptr)
        {
            static_cast<CartpoleTensorBindingCreate*>(userData)->mLoggedError.store(true, std::memory_order_relaxed);
        }
    }

    void failOnLoggedError(const char* operation) const
    {
        requireSuccess(
            ovphysx_flush_log(OVPHYSX_TIMEOUT_INFINITE), "flush Cartpole TensorBinding creation error callback");
        if (mLoggedError.load(std::memory_order_relaxed))
        {
            throw std::runtime_error(std::string(operation) + " emitted an OVPhysX error");
        }
    }

    void waitForAll(const char* operation)
    {
        ovphysx::physx::WaitResult waitResult;
        requireSuccess(
            ovphysx_wait_op(mPhysX->handle(), OVPHYSX_OP_INDEX_ALL, OVPHYSX_TIMEOUT_INFINITE, waitResult.get()),
            operation);
        if (waitResult.hasErrors())
        {
            throw std::runtime_error(std::string(operation) + " completed with asynchronous errors");
        }
    }

    void destroyBindings(bool checkErrors)
    {
        std::string firstError;
        for (size_t bindingIndex = 0; bindingIndex < mBindings.size(); ++bindingIndex)
        {
            if (!mBindings[bindingIndex])
            {
                continue;
            }
            mBindings[bindingIndex].destroy();
            if (checkErrors)
            {
                // TensorBinding::destroy() is void. Its C call leaves an empty last error on success.
                const std::string detail = lastError();
                if (!detail.empty() && firstError.empty())
                {
                    firstError = std::string(kCartpoleBindingCases[bindingIndex].name) + ": " + detail;
                }
            }
        }
        if (!firstError.empty())
        {
            throw std::runtime_error("destroy Cartpole TensorBinding failed: " + firstError);
        }
    }

    bool mInitialized = false;
    bool mStageLoaded = false;
    bool mLogCallbackRegistered = false;
    bool mMeasured = false;
    std::atomic<bool> mLoggedError{ false };
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
    std::array<ovphysx::TensorBinding, kCartpoleBindingCases.size()> mBindings;
    std::array<ovphysx_tensor_spec_t, kCartpoleBindingCases.size()> mSpecs{};
};

#else

class CartpoleControlStep : public BmBenchmark
{
public:
    bool isValid() const override
    {
        throw std::runtime_error("Cartpole probe requires a CUDAToolkit-enabled benchmark build");
    }

    uint32_t getNbSteps() const override
    {
        return 20;
    }

    uint32_t getNbRuns() const override
    {
        return 5;
    }

    void startRun() override
    {
    }

    void preStep() override
    {
    }

    void endRun() override
    {
    }

protected:
    void step() override
    {
    }
};

class CartpoleTensorBindingCreate : public BmBenchmark
{
public:
    bool isValid() const override
    {
        throw std::runtime_error(
            "Cartpole TensorBinding creation probe requires a CUDAToolkit-enabled benchmark build");
    }

    uint32_t getNbSteps() const override
    {
        return 1;
    }

    uint32_t getNbRuns() const override
    {
        return 0;
    }

    void startRun() override
    {
    }

    void preStep() override
    {
    }

    void endRun() override
    {
    }

protected:
    void step() override
    {
    }
};

#endif

Register<CartpoleControlStep, true> sCartpoleControlStep("Probe.cartpole_4096_control_step");
Register<CartpoleTensorBindingCreate, true> sCartpoleTensorBindingCreate("Probe.cartpole_4096_tensor_binding_create");

} // namespace
