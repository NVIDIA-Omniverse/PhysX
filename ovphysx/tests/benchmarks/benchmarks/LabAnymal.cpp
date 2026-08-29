// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// IsaacLab-style Anymal benchmarks. Quadruped articulation per env, cloned
// via clone() into N envs. Four ops per size: step / reset / tensor_read /
// tensor_write. All GPU.
//
// Asset state (OMPE-94463): the real Anymal asset is not currently
// published to packman. Until it is, these benches skip cleanly via
// isValid() returning false when the asset directory is absent. The
// expected layout when the asset lands is:
//     tests/benchmarks/data/anymal/anymal.usd
// with the articulation rooted at /World/envs/env0/anymal — see
// docs/internal/benchmark_suite_notes.md item 14 for follow-up details.

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
#include <fstream>
#include <string>
#include <vector>


void initLabAnymal()
{
}

namespace
{

constexpr DLDataType kFloat32 = {static_cast<uint8_t>(kDLFloat), 32, 1};


inline bool anymalAssetPresent()
{
    // Gate on BOTH the root articulation USD and the meshes it references.
    // Without instanceable_meshes.usd, PhysX falls back to bounding-sphere
    // collision and bench numbers are skewed — skip cleanly on partial
    // fetch instead of running with the wrong measurement.
    const std::string base =
        BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/anymal";
    const std::string root = base + "/anymal.usd";
    const std::string meshes = base + "/Props/instanceable_meshes.usd";
    std::ifstream fr(root.c_str());
    std::ifstream fm(meshes.c_str());
    return fr.good() && fm.good();
}


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


class LabAnymalBase : public BmBenchmark
{
public:
    enum class Op { Step, Reset, TensorRead, TensorWrite, Clone };

    LabAnymalBase(Op op, uint32_t envCount) : mOp(op), mEnvCount(envCount) {}

    bool isValid() const override
    {
        if (!BmGlobals::getInstance().forceGpu()) return false;
        if (BmGlobals::getInstance().getPhysX() == nullptr) return false;
        if (!anymalAssetPresent())
        {
            printFormatted("LabAnymal: anymal.usd or instanceable_meshes.usd "
                           "not present — run scripts/fetch_anymal_asset.py, "
                           "skipping");
            return false;
        }
        return true;
    }

    uint32_t getNbSteps() const override
    {
        // Anymal is much heavier per env than cartpole — smaller budgets.
        // Reset/Clone re-load+re-clone per timed step (see preStep) so
        // fewer steps at large N to bound wall-time.
        if (mOp == Op::Step) return mEnvCount >= 8192 ? 5 : 10;
        if (mOp == Op::Reset || mOp == Op::Clone) return mEnvCount >= 8192 ? 2 : 5;
        return 30;
    }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        // Cache PhysX* + tensor view once so step() doesn't pay for a
        // global lookup or rebuild per measured iteration.
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        // Build the target paths + grid transforms once per run for any
        // bench that calls clone() — these depend only on mEnvCount (const
        // for the bench instance) and at N=8192 allocating them inside the
        // timed Op::Clone step() would dwarf the actual clone() cost.
        buildCloneInputs();

        // For Op::Reset / Op::Clone, load+clone is done per-step in
        // preStep() so each timed iteration acts on a fresh scene.
        if (mOp == Op::Reset) return;
        if (mOp == Op::Clone)
        {
            // Clone bench: load the wrapper once; clone happens per-step.
            loadOnly();
            return;
        }

        loadAndCloneOnce();

        if (mOp == Op::TensorRead || mOp == Op::TensorWrite)
        {
            ovphysx_api_status_t st = mPhysX->createTensorBinding(
                mBinding, "/World/envs/env*/anymal/*",
                OVPHYSX_TENSOR_RIGID_BODY_POSE_F32);
            if (st != OVPHYSX_API_SUCCESS)
            {
                printFormatted("LabAnymal: createTensorBinding failed: %d",
                               static_cast<int>(st));
                return;
            }
            ovphysx_tensor_spec_t spec{};
            if (mBinding.spec(spec) != OVPHYSX_API_SUCCESS || spec.ndim < 1)
            {
                printFormatted("LabAnymal: binding spec query failed");
                return;
            }
            std::vector<int64_t> shape(static_cast<size_t>(spec.ndim));
            for (int i = 0; i < spec.ndim; ++i) shape[i] = spec.shape[i];
            mTensor.resize(shape);
            DLTensor t = mTensor.view();
            if (mBinding.read(t) != OVPHYSX_API_SUCCESS)
            {
                printFormatted("LabAnymal: priming read() failed");
                return;
            }
            mPhysX->waitAll();
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
        if (mOp == Op::Reset) loadAndCloneOnce();
        else if (mOp == Op::Clone)
        {
            // Reload the wrapper before each timed clone so the source
            // template exists again after the previous step's reset.
            benchmarkClearOvstage(mPhysX, mStageAttachment);
            loadOnly();
        }
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
            case Op::Clone:
                cloneOnly();
                mPhysX->waitAll();
                break;
        }
    }

private:
    void loadOnly()
    {
        ovphysx::PhysX* physx = mPhysX;
        if (!physx) return;
        const std::string path =
            BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/anymal/anymal_envs.usda";
        if (!benchmarkLoadUsdWithOvstage(physx, path, mStageAttachment))
        {
            printFormatted("LabAnymal: ovstage load failed");
            return;
        }
    }

    void buildCloneInputs()
    {
        // Deterministic for fixed mEnvCount — built once per run, reused by
        // every cloneOnly() call inside the timed step() loop.
        if (mTargets.size() == mEnvCount) return;
        mTargets.clear();
        mTargets.reserve(mEnvCount);
        for (uint32_t i = 0; i < mEnvCount; ++i)
            mTargets.emplace_back("/World/envs/env" + std::to_string(i + 1));
        constexpr float kSpacing = 8.0f;
        const uint32_t side = mEnvCount == 0 ? 1u : static_cast<uint32_t>(
            std::ceil(std::sqrt(static_cast<float>(mEnvCount))));
        mTransforms.assign(static_cast<size_t>(mEnvCount) * 7, 0.0f);
        for (uint32_t i = 0; i < mEnvCount; ++i)
        {
            const uint32_t row = i / side;
            const uint32_t col = i % side;
            float* t = mTransforms.data() + static_cast<size_t>(i) * 7;
            t[0] = static_cast<float>(col) * kSpacing;
            t[2] = static_cast<float>(row) * kSpacing;
            t[6] = 1.0f;
        }
    }

    void cloneOnly()
    {
        // Hot path for Op::Clone — mTargets / mTransforms were built once
        // in startRun() via buildCloneInputs(), so this is just the timed
        // clone() call plus its waitAll.
        ovphysx_api_status_t st = mPhysX->clone("/World/envs/template",
                                                mTargets, mTransforms.data());
        if (st != OVPHYSX_API_SUCCESS)
        {
            printFormatted("LabAnymal: clone(N=%u) failed: %d",
                           mEnvCount, static_cast<int>(st));
        }
    }

    void loadAndCloneOnce()
    {
        ovphysx::PhysX* physx = mPhysX;
        if (!physx) return;

        // Load the wrapper USDA (committed) which references the downloaded
        // anymal.usd under /World/envs/template/anymal. The wrapper exists
        // so we can clone /World/envs/template into env1..envN and have the
        // binding pattern /World/envs/env*/anymal/* match exactly N envs.
        const std::string path =
            BmGlobals::getInstance().getDataFolder() + "/../benchmarks/data/anymal/anymal_envs.usda";
        if (!benchmarkLoadUsdWithOvstage(physx, path, mStageAttachment))
        {
            printFormatted("LabAnymal: ovstage load failed");
            return;
        }

        // Reuse the cached targets/transforms built once in startRun().
        // Wrapper places the source under /World/envs/template/anymal so the
        // binding pattern /World/envs/env*/anymal/* matches the N clones
        // (env1..envN), not the source — exactly N envs, not N+1.
        buildCloneInputs();
        ovphysx_api_status_t st = physx->clone("/World/envs/template", mTargets, mTransforms.data());
        if (st != OVPHYSX_API_SUCCESS)
        {
            printFormatted("LabAnymal: clone(N=%u) failed: %d",
                           mEnvCount, static_cast<int>(st));
            return;
        }
        physx->waitAll();
    }

    Op mOp;
    uint32_t mEnvCount;
    std::vector<std::string> mTargets;
    std::vector<float> mTransforms;
    ovphysx::TensorBinding mBinding;
    LabTensor mTensor;
    DLTensor mView{};
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


#define DEFINE_LAB_ANYMAL(opName, OpEnum, N)                                      \
    class LabAnymal_##opName##_##N : public LabAnymalBase                         \
    {                                                                             \
    public:                                                                       \
        LabAnymal_##opName##_##N() : LabAnymalBase(LabAnymalBase::OpEnum, N) {}   \
    };                                                                            \
    Register<LabAnymal_##opName##_##N> sLabAnymal_##opName##_##N(                 \
        "Lab.anymal_" #N "_" #opName);

// Sizes: 1024/8192. Smaller sizes were below the regime where the harness
// signal-to-noise is meaningful.
DEFINE_LAB_ANYMAL(step,         Op::Step,        1024)
DEFINE_LAB_ANYMAL(step,         Op::Step,        8192)
DEFINE_LAB_ANYMAL(reset,        Op::Reset,       1024)
DEFINE_LAB_ANYMAL(reset,        Op::Reset,       8192)
DEFINE_LAB_ANYMAL(tensor_read,  Op::TensorRead,  1024)
DEFINE_LAB_ANYMAL(tensor_read,  Op::TensorRead,  8192)
DEFINE_LAB_ANYMAL(tensor_write, Op::TensorWrite, 1024)
DEFINE_LAB_ANYMAL(tensor_write, Op::TensorWrite, 8192)
// Clone benchmark -- measures cost of clone() itself. Cloning is on the hot
// path for IsaacLab-style RL setup, worth measuring directly.
DEFINE_LAB_ANYMAL(clone,        Op::Clone,       1024)
DEFINE_LAB_ANYMAL(clone,        Op::Clone,       8192)

#undef DEFINE_LAB_ANYMAL

} // namespace
