// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause

// Clone scaling benchmarks: replicate /World/envs/env0 from
// basic_simulation.usda into N target paths. GPU-only.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <cmath>
#include <string>
#include <vector>


void initClone()
{
}

namespace
{

class CloneScalingBase : public BmBenchmark
{
public:
    explicit CloneScalingBase(uint32_t n) : mN(n) {}

    bool isValid() const override
    {
        return BmGlobals::getInstance().forceGpu();
    }

    uint32_t getNbSteps() const override { return 1; }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        ovphysx::PhysX* physx = BmGlobals::getInstance().getPhysX();
        if (!physx) return;

        const std::string path = BmGlobals::getInstance().getDataFolder() + "/basic_simulation.usda";
        if (!benchmarkLoadUsdWithOvstage(physx, path, mStageAttachment))
        {
            printFormatted("Clone: ovstage load(basic_simulation.usda) failed");
            return;
        }

        // Pre-build the target paths and per-target parent transforms here
        // (untimed setup) so step() measures only the clone() call itself,
        // not the cost of constructing N strings and a 7N float array.
        mTargets.clear();
        mTargets.reserve(mN);
        for (uint32_t i = 0; i < mN; ++i)
        {
            // env0 already exists in the fixture; clone into env1..envN.
            mTargets.emplace_back("/World/envs/env" + std::to_string(i + 1));
        }

        // sqrt(N) x sqrt(N) grid placement with identity rotation.
        // Matches IsaacLab's GridCloner shape: 7 floats per target
        // (px, py, pz, qx, qy, qz, qw), imaginary-first quaternion.
        constexpr float kSpacing = 4.0f;
        const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<float>(mN))));
        mParentTransforms.assign(static_cast<size_t>(mN) * 7, 0.0f);
        for (uint32_t i = 0; i < mN; ++i)
        {
            const uint32_t row = i / side;
            const uint32_t col = i % side;
            float* t = mParentTransforms.data() + static_cast<size_t>(i) * 7;
            t[0] = static_cast<float>(col) * kSpacing; // px
            t[1] = 0.0f;                                // py (ground plane)
            t[2] = static_cast<float>(row) * kSpacing; // pz
            t[3] = 0.0f;                                // qx
            t[4] = 0.0f;                                // qy
            t[5] = 0.0f;                                // qz
            t[6] = 1.0f;                                // qw (identity)
        }
    }

    void endRun() override
    {
        ovphysx::PhysX* physx = BmGlobals::getInstance().getPhysX();
        if (!physx) return;
        benchmarkClearOvstage(physx, mStageAttachment);
    }

    void preStep() override {}

protected:
    void step() override
    {
        ovphysx::PhysX* physx = BmGlobals::getInstance().getPhysX();
        if (!physx) return;

        ovphysx_api_status_t st = physx->clone("/World/envs/env0", mTargets, mParentTransforms.data());
        if (st != OVPHYSX_API_SUCCESS)
        {
            printFormatted("Clone: clone(N=%u) failed: status=%d", mN, static_cast<int>(st));
            return;
        }
        physx->waitAll();
    }

private:
    uint32_t mN;
    std::vector<std::string> mTargets;
    std::vector<float> mParentTransforms;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


class Clone_64   : public CloneScalingBase { public: Clone_64()   : CloneScalingBase(64)   {} };
class Clone_256  : public CloneScalingBase { public: Clone_256()  : CloneScalingBase(256)  {} };
class Clone_1024 : public CloneScalingBase { public: Clone_1024() : CloneScalingBase(1024) {} };


Register<Clone_64>   sClone64("Clone.envs_64");
Register<Clone_256>  sClone256("Clone.envs_256");
Register<Clone_1024> sClone1024("Clone.envs_1024");

} // namespace
