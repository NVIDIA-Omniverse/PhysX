// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

// IsaacLab-style cartpole benchmarks. One cartpole articulation per env,
// cloned into N envs via the same clone() path that Clone.envs_* uses.
// Four ops per size: step / reset / tensor_read / tensor_write. All GPU
// (matching how RL workloads actually run). See OMPE-94463 Track 3.

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


void initLabCartpole()
{
}

namespace
{

constexpr DLDataType kFloat32 = {static_cast<uint8_t>(kDLFloat), 32, 1};


// Same shape as TensorBindings.cpp's CpuTensor — local to keep this
// translation unit self-contained.
class LabTensor
{
public:
    LabTensor() = default;
    LabTensor(const LabTensor&) = delete;
    LabTensor& operator=(const LabTensor&) = delete;
    ~LabTensor() { std::free(mData); }

    void resize(const std::vector<int64_t>& shape)
    {
        mShape = shape;
        mStrides.assign(shape.size(), 1);
        for (int i = static_cast<int>(shape.size()) - 2; i >= 0; --i)
            mStrides[i] = mStrides[i + 1] * shape[i + 1];
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


class LabCartpoleBase : public BmBenchmark
{
public:
    enum class Op { Step, Reset, TensorRead, TensorWrite };

    LabCartpoleBase(Op op, uint32_t envCount) : mOp(op), mEnvCount(envCount) {}

    bool isValid() const override
    {
        if (!BmGlobals::getInstance().forceGpu()) return false;
        return BmGlobals::getInstance().getPhysX() != nullptr;
    }

    uint32_t getNbSteps() const override
    {
        // Bound wall-time at large N. Step is heaviest per call; reset
        // also requires re-setup per timed step (see preStep) so it gets
        // even fewer measured iterations.
        if (mOp == Op::Step) return mEnvCount >= 8192 ? 10 : 20;
        if (mOp == Op::Reset) return mEnvCount >= 8192 ? 3 : 5;
        return 50;
    }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        // Cache PhysX* + tensor view once so step() doesn't pay for a
        // global lookup / view-rebuild / status-print branch per measured
        // iteration. Defensive coding has no place in the hot path.
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        // For Op::Reset the load+clone is done per-step in preStep() so
        // each timed reset acts on a freshly-cloned scene (the previous
        // step's reset wiped the stage). For all other ops, set up once
        // per run here and time the per-call op.
        if (mOp == Op::Reset) return;

        loadAndCloneOnce();

        if (mOp == Op::TensorRead || mOp == Op::TensorWrite)
        {
            // Pose tensor over all clone envs' cartpole rigid bodies.
            // The binding pattern is "/World/envs/env*/cartpole/*", which
            // matches env1..envN (clones) but NOT /World/envs/template
            // (the source) — exactly N envs, not N+1.
            ovphysx_api_status_t st = mPhysX->createTensorBinding(
                mBinding, "/World/envs/env*/cartpole/*",
                OVPHYSX_TENSOR_RIGID_BODY_POSE_F32);
            if (st != OVPHYSX_API_SUCCESS)
            {
                printFormatted("LabCartpole: createTensorBinding failed: %d",
                               static_cast<int>(st));
                return;
            }
            ovphysx_tensor_spec_t spec{};
            if (mBinding.spec(spec) != OVPHYSX_API_SUCCESS || spec.ndim < 1)
            {
                printFormatted("LabCartpole: binding spec query failed");
                return;
            }
            std::vector<int64_t> shape(static_cast<size_t>(spec.ndim));
            for (int i = 0; i < spec.ndim; ++i) shape[i] = spec.shape[i];
            mTensor.resize(shape);
            // Prime so write benches don't push uninitialized quaternions.
            // Also validate read() here so a failure surfaces in setup
            // rather than as silent garbage during the timed loop.
            DLTensor t = mTensor.view();
            if (mBinding.read(t) != OVPHYSX_API_SUCCESS)
            {
                printFormatted("LabCartpole: priming read() failed");
                return;
            }
            mPhysX->waitAll();
            // Cache the DLTensor view so step() doesn't rebuild it.
            mView = mTensor.view();
        }
    }

    void endRun() override
    {
        if (mOp == Op::TensorRead || mOp == Op::TensorWrite) mBinding.destroy();
        if (!mPhysX) return;
        benchmarkClearOvstage(mPhysX, mStageAttachment);
    }

    void preStep() override
    {
        // For Op::Reset, the previous step's reset wiped the stage, so
        // re-load and re-clone here (untimed) before each timed reset.
        if (mOp == Op::Reset) loadAndCloneOnce();
    }

protected:
    void step() override
    {
        switch (mOp)
        {
            case Op::Step:
                mPhysX->step(1.0f / 60.0f);
                mPhysX->waitAll();
                break;
            case Op::Reset:
                (void)ovphysx_reset_stage(mPhysX->handle());
                mPhysX->waitAll();
                break;
            case Op::TensorRead:
                (void)mBinding.read(mView);
                mPhysX->waitAll();
                break;
            case Op::TensorWrite:
                (void)mBinding.write(mView);
                mPhysX->waitAll();
                break;
        }
    }

private:
    void loadAndCloneOnce()
    {
        ovphysx::PhysX* physx = mPhysX;
        if (!physx) return;

        const std::string path =
            BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/cartpole.usda";
        if (!benchmarkLoadUsdWithOvstage(physx, path, mStageAttachment))
        {
            printFormatted("LabCartpole: ovstage load failed");
            return;
        }

        mTargets.clear();
        mTargets.reserve(mEnvCount);
        for (uint32_t i = 0; i < mEnvCount; ++i)
            mTargets.emplace_back("/World/envs/env" + std::to_string(i + 1));

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
        // cartpole.usda's source articulation lives at /World/envs/template
        // (NOT env0) so the binding pattern matches exactly N clones, not N+1.
        ovphysx_api_status_t st = physx->clone("/World/envs/template", mTargets, transforms.data());
        if (st != OVPHYSX_API_SUCCESS)
        {
            printFormatted("LabCartpole: clone(N=%u) failed: %d",
                           mEnvCount, static_cast<int>(st));
            return;
        }
        physx->waitAll();
    }

    Op mOp;
    uint32_t mEnvCount;
    std::vector<std::string> mTargets;
    ovphysx::TensorBinding mBinding;
    LabTensor mTensor;
    DLTensor mView{};
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


// Macro to keep the 16 (op × size) registrations terse. We register the
// op-first naming (`Lab.cartpole_step_64`) to match how the existing benches
// group by op-name then qualifier; downstream FrameCore dashboards stay
// sorted naturally.
#define DEFINE_LAB_CARTPOLE(opName, OpEnum, N)                                       \
    class LabCartpole_##opName##_##N : public LabCartpoleBase                        \
    {                                                                                \
    public:                                                                          \
        LabCartpole_##opName##_##N() : LabCartpoleBase(LabCartpoleBase::OpEnum, N) {}\
    };                                                                               \
    Register<LabCartpole_##opName##_##N> sLabCartpole_##opName##_##N(                \
        "Lab.cartpole_" #N "_" #opName);

// Sizes 4k/8k/16k. Smaller sizes (64/256/1024)
// were below the regime where Lab-style cloning + per-step ops show
// meaningful scaling.
DEFINE_LAB_CARTPOLE(step,         Op::Step,        4096)
DEFINE_LAB_CARTPOLE(step,         Op::Step,        8192)
DEFINE_LAB_CARTPOLE(step,         Op::Step,        16384)
DEFINE_LAB_CARTPOLE(reset,        Op::Reset,       4096)
DEFINE_LAB_CARTPOLE(reset,        Op::Reset,       8192)
DEFINE_LAB_CARTPOLE(reset,        Op::Reset,       16384)
DEFINE_LAB_CARTPOLE(tensor_read,  Op::TensorRead,  4096)
DEFINE_LAB_CARTPOLE(tensor_read,  Op::TensorRead,  8192)
DEFINE_LAB_CARTPOLE(tensor_read,  Op::TensorRead,  16384)
DEFINE_LAB_CARTPOLE(tensor_write, Op::TensorWrite, 4096)
DEFINE_LAB_CARTPOLE(tensor_write, Op::TensorWrite, 8192)
DEFINE_LAB_CARTPOLE(tensor_write, Op::TensorWrite, 16384)

#undef DEFINE_LAB_CARTPOLE

} // namespace
