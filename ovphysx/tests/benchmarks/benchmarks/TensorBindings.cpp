// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// Tensor binding I/O throughput across scaled scenes. CPU-only.
//
// Per review on MR !7247: the previous version measured against
// /World/envs/env0/* on basic_simulation.usda, which exposes only ~32
// objects to the binding. That's well below the regime where pose I/O
// throughput is meaningful. We now clone cubes20 into 1024 / 8192 env
// targets so the binding's spec yields N×21 prims (20 dynamic cubes +
// ground per env) and the timed read/write op covers a realistic
// IsaacLab-scale payload.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include "ovphysx/dlpack/dlpack.h"
#include "ovphysx/experimental/TensorBinding.hpp"
#include <ovphysx/experimental/ovphysx.hpp>

#include <cmath>
#include <cstdlib>
#include <string>
#include <vector>


void initTensorBindings()
{
}

namespace
{

constexpr DLDataType kFloat32 = {static_cast<uint8_t>(kDLFloat), 32, 1};

class CpuTensor
{
public:
    CpuTensor() = default;
    CpuTensor(const CpuTensor&) = delete;
    CpuTensor& operator=(const CpuTensor&) = delete;

    ~CpuTensor() { std::free(mData); }

    void resize(const std::vector<int64_t>& shape)
    {
        mShape = shape;
        mStrides.assign(shape.size(), 1);
        for (int i = static_cast<int>(shape.size()) - 2; i >= 0; --i)
        {
            mStrides[i] = mStrides[i + 1] * shape[i + 1];
        }
        size_t n = 1;
        for (int64_t d : shape) n *= static_cast<size_t>(d);
        std::free(mData);
        mData = std::malloc(n * sizeof(float));
    }

    DLTensor view()
    {
        DLTensor t{};
        t.data = mData;
        t.device = DLDevice{kDLCPU, 0};
        t.ndim = static_cast<int32_t>(mShape.size());
        t.dtype = kFloat32;
        t.shape = mShape.data();
        t.strides = mStrides.data();
        t.byte_offset = 0;
        return t;
    }

private:
    void* mData = nullptr;
    std::vector<int64_t> mShape;
    std::vector<int64_t> mStrides;
};


class TensorIoBase : public BmBenchmark
{
public:
    enum class Op { Create, Read, Write };

    TensorIoBase(Op op, uint32_t envCount) : mOp(op), mEnvCount(envCount) {}

    bool isValid() const override
    {
        if (BmGlobals::getInstance().forceGpu()) return false;
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override
    {
        // Create is heavier per call (full bind/spec/destroy cycle).
        if (mOp == Op::Create) return 10;
        // Read/Write at 8192 envs touch ~170k prims; cap steps so wall
        // time stays bounded.
        return mEnvCount >= 8192 ? 50 : 100;
    }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        // Cache PhysX* + DLTensor view once so step() doesn't pay for a
        // global lookup or rebuild per measured iteration (MR !7247).
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        // Load cubes20 (template at /World/envs/template under wrapper) and
        // clone to N envs so the binding pattern matches a realistic
        // IsaacLab-scale payload.
        const std::string path =
            BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/cubes20_envs.usda";
        if (!benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment))
        {
            printFormatted("TensorIo: ovstage load(cubes20_envs) failed");
            return;
        }

        std::vector<std::string> targets;
        targets.reserve(mEnvCount);
        for (uint32_t i = 0; i < mEnvCount; ++i)
            targets.emplace_back("/World/envs/env" + std::to_string(i + 1));
        constexpr float kSpacing = 4.0f;
        const uint32_t side = static_cast<uint32_t>(
            std::ceil(std::sqrt(static_cast<float>(mEnvCount))));
        std::vector<float> transforms(static_cast<size_t>(mEnvCount) * 7, 0.0f);
        for (uint32_t i = 0; i < mEnvCount; ++i)
        {
            const uint32_t row = i / side;
            const uint32_t col = i % side;
            float* t = transforms.data() + static_cast<size_t>(i) * 7;
            t[0] = static_cast<float>(col) * kSpacing;
            t[2] = static_cast<float>(row) * kSpacing;
            t[6] = 1.0f;
        }
        ovphysx_api_status_t st = mPhysX->clone("/World/envs/template", targets, transforms.data());
        if (st != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorIo: clone(N=%u) failed: %d",
                           mEnvCount, static_cast<int>(st));
            return;
        }
        mPhysX->waitAll();

        mPattern = "/World/envs/env*/*";

        if (mOp == Op::Create) return;

        // Persistent binding for Read/Write so each timed op is just the
        // read or write call, not bind creation.
        st = mPhysX->createTensorBinding(mBinding, mPattern,
                                         OVPHYSX_TENSOR_RIGID_BODY_POSE_F32);
        if (st != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorIo: createTensorBinding(%s) failed: status=%d",
                           mPattern.c_str(), static_cast<int>(st));
            return;
        }
        ovphysx_tensor_spec_t spec{};
        if (mBinding.spec(spec) != OVPHYSX_API_SUCCESS || spec.ndim < 1)
        {
            printFormatted("TensorIo: binding spec query failed");
            return;
        }
        std::vector<int64_t> shape(static_cast<size_t>(spec.ndim));
        for (int i = 0; i < spec.ndim; ++i) shape[i] = spec.shape[i];
        mTensor.resize(shape);
        // Prime with the binding's current state — see read/write rationale.
        DLTensor t = mTensor.view();
        if (mBinding.read(t) != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorIo: priming read() failed");
            return;
        }
        mPhysX->waitAll();
        mView = mTensor.view();
    }

    void endRun() override
    {
        if (mOp != Op::Create) mBinding.destroy();
        if (!mPhysX) return;
        benchmarkClearOvstage(mPhysX, mStageAttachment);
    }

    void preStep() override {}

protected:
    void step() override
    {
        if (mOp == Op::Create)
        {
            ovphysx::TensorBinding binding;
            (void)mPhysX->createTensorBinding(binding, mPattern,
                                              OVPHYSX_TENSOR_RIGID_BODY_POSE_F32);
            ovphysx_tensor_spec_t spec{};
            (void)binding.spec(spec);
            binding.destroy();
        }
        else if (mOp == Op::Read)
        {
            (void)mBinding.read(mView);
            mPhysX->waitAll();
        }
        else
        {
            (void)mBinding.write(mView);
            mPhysX->waitAll();
        }
    }

private:
    Op mOp;
    uint32_t mEnvCount;
    std::string mPattern;
    ovphysx::TensorBinding mBinding;
    CpuTensor mTensor;
    DLTensor mView{};
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


#define DEFINE_TENSOR_IO(opName, OpEnum, N)                                  \
    class TensorIo_##opName##_##N : public TensorIoBase                      \
    {                                                                        \
    public:                                                                  \
        TensorIo_##opName##_##N() : TensorIoBase(TensorIoBase::OpEnum, N) {} \
    };                                                                       \
    Register<TensorIo_##opName##_##N> sTensorIo_##opName##_##N(              \
        "TensorIo.pose_" #opName "_" #N "_cpu");

DEFINE_TENSOR_IO(create, Op::Create, 1024)
DEFINE_TENSOR_IO(create, Op::Create, 8192)
DEFINE_TENSOR_IO(read,   Op::Read,   1024)
DEFINE_TENSOR_IO(read,   Op::Read,   8192)
DEFINE_TENSOR_IO(write,  Op::Write,  1024)
DEFINE_TENSOR_IO(write,  Op::Write,  8192)

#undef DEFINE_TENSOR_IO

} // namespace
