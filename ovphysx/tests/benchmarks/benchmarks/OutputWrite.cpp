// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the binding comparison lanes retire with the binding. Session lanes stay.

// App -> physics write (ADR-0012) at scale, against the equivalent tensor binding.
//
// The mirror of OutputRead.cpp, and deliberately its twin: same scene, same clone counts, same
// warm-up and the same per-sample step, so a write number can be read directly against the read
// number beside it.
//
// Names are OutputWrite.<operation>_<objects>_<envs>_<device>:
//
//   operation  querywrite    -> query + session + fill + commit + release: what a stateless
//                               caller pays per frame.
//              writeonly     -> session + fill + commit, query opened once and held across steps.
//              tensorbinding -> the same payload written through a binding already built.
//
//   objects    rb            -> rigid bodies, cubes20 scene (~164k bodies at 8192 envs).
//
//              arti_root     -> the articulation's ROOT pose, cartpole scene, mirroring
//                               OutputRead's arti_link / arti_dof naming.
//
//              tendon_fixed  -> fixed tendons, cartpole_tendon scene, tendonStiffness.
//              tendon_spatial-> spatial tendons, same scene, same column.
//              rb_instancer  -> the same bodies authored as point-instancer INSTANCES, `position`:
//                               the only family where the write does more work than the read.
//              rb_instancer_vel -> the same instances, `linearVelocity`: no reframe, so the
//                               difference from rb_instancer is the reframe inverse plus the pose
//                               pre-read.
//              force_rb      -> rigid bodies, `force`: WRITE-ONLY, no read lane to face.
//              wrench_rb     -> rigid bodies, `wrench` [N,9]: adds a pose read and the
//                               torque-about-COM conversion over the same bodies.
//              wrench_link   -> articulation links, `wrench`: the DirectGPU (articulation, link)
//                               route, a different code path rather than other rows.
//              arti_dof      -> articulation joint DOFs, jointPosition: the per-axis ARRAY path,
//                               which no rigid lane has an equivalent of. Its setup PRIMES the
//                               joint cache with one read, because the joint write derives its
//                               joint set from the cache the read fills.
//
// WHAT IS TIMED, and why the fill is inside it. A write session hands the caller the destination
// buffer, so filling it IS the caller's copy. There is no second one. The binding takes a
// caller-owned tensor and copies out of it. Each side therefore performs exactly one copy of the
// payload, and excluding the fill here would hide the ovphysx side's half of that while still
// charging the binding for its own. One attribute per lane (position), because a session carries
// one attribute. The binding lane writes the pose tensor, which is the smallest unit it can write.
//
// So these lanes are NOT a like-for-like payload comparison the way the read's are: the binding
// writes [N, 7] where this writes [N, 3]. Read them as "what does each API cost to move one
// attribute", not as a throughput ratio. The pose write also pays a read-modify-write the binding
// does not. PxRigidDynamicGPUAPIWriteType carries one eGLOBAL_POSE, so setting position alone
// must first read the current pose to keep the orientation. That cost is real and belongs in the
// number.
//
// The _cpu variants run on the default pass. The _gpu ones require DirectGPU and are hidden from
// wildcard runs, for the same reason OutputRead's are: without suppressReadback the scene never
// raises PxSceneFlag::eENABLE_DIRECT_GPU_API and both APIs fall back to their host paths. Run them
// in a dedicated process (DirectGPU changes scene behavior for other GPU benchmarks):
//
//   ovphysx_benchmarks --forceGpu --directGpu --hidden --filter=OutputWrite.*_gpu
//
// NO VEHICLE LANE, deliberately (ADR-0012). vehicle_probe.usda is a DRIVE vehicle, and the per-wheel
// controls the write serves are refused on exactly that, so a lane on it would time a refusal. Beyond
// the fixture: the tensor binding has no vehicle tensor types, so there is no lane to face, and
// vehicles are CPU-only so there is no GPU number either. The reason is recorded in the write API rather
// than left as an absence.
//
// The binding lanes inherit objectType() from OutputWriteBase as RIGID_BODY, and the base's
// startRun does warm-up ovphysx WRITES before the binding is ever exercised. A rigid session asking
// for tendonStiffness is refused, the warm-up fails, and the lane reports Skipped, which reads
// like "not applicable here" rather than "this lane is broken". Any new binding lane whose attribute
// is not a rigid one must override objectType() to match.
//
#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "../OvstageLoad.h"

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
#include <cuda_runtime_api.h>
#endif

#include "ovphysx/dlpack/dlpack.h"
#include "ovphysx/experimental/TensorBinding.hpp"
#include <ovphysx/experimental/ovphysx.hpp>

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>


void initOutputWrite()
{
}

namespace
{

// The same selector the read benchmark binds, so the binding lane covers the bodies the write
// lanes cover.
constexpr const char* kBodyPattern = "/World/envs/*/*";
// The cartpole asset puts one articulation per env at env_*/Robot, the same pattern OutputRead's
// articulation binding lanes use, so the two files bind the same set.
constexpr const char* kArticulationPattern = "/World/envs/env_*/Robot";

class OutputWriteBase : public BmBenchmark
{
public:
    OutputWriteBase(uint32_t envCount, bool gpu) : mEnvCount(envCount), mGpu(gpu) {}

    bool isValid() const override
    {
        const BmGlobals& globals = BmGlobals::getInstance();
        if (!mGpu)
        {
            if (globals.forceGpu()) return false;
            return globals.getPhysX() != nullptr;
        }
        // The write branches on PxSceneFlag::eENABLE_DIRECT_GPU_API, which is raised only when
        // suppressReadback is set, so --forceGpu alone would measure the host scatter.
        if (!globals.forceGpu())
        {
            throw std::runtime_error("OutputWrite GPU variants require --forceGpu");
        }
        if (!globals.directGpu())
        {
            throw std::runtime_error("OutputWrite GPU variants require --directGpu");
        }
        if (globals.getPhysX() == nullptr)
        {
            throw std::runtime_error("OutputWrite failed to initialize OVPhysX");
        }
        return true;
    }

    static constexpr int kWarmUpWrites = 3;

    uint32_t getNbSteps() const override { return mEnvCount >= 8192 ? 20 : 50; }
    uint32_t getNbRuns() const override { return 5; }

    void startRun() override
    {
        mSetupOk = false;
        // Re-armed per run, not once per process: the payload gates (including the non-empty
        // shape check) only run while this is true, so leaving it false after run 1 would let
        // runs 2+ warm up on an empty write and snapshot the expectation as zero.
        mCountPayload = true;
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        const std::string path = BmGlobals::getInstance().getDataFolder() + sceneAsset();
        if (!benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment))
        {
            printFormatted("OutputWrite: ovstage load failed (%s)", path.c_str());
            return;
        }

        // An authored-env asset already holds every env, so cloning it would double the scene. Only
        // the CLONE is skipped. The step, afterSceneSetup() and the warm-up below all still have
        // to run, so this is a guarded block and not an early return. Same hook and same name as the
        // read's AuthoredEnvSceneT, so the two files read alike.
        if (clonesEnvs())
        {
            std::vector<std::string> targets;
            targets.reserve(mEnvCount);
            for (uint32_t i = 0; i < mEnvCount; ++i)
                targets.emplace_back(cloneTarget(i));

            const float kSpacing = cloneSpacing();
            const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<float>(mEnvCount))));
            std::vector<float> transforms(static_cast<size_t>(mEnvCount) * 7, 0.0f);
            for (uint32_t i = 0; i < mEnvCount; ++i)
            {
                const uint32_t row = i / side;
                const uint32_t col = i % side;
                float* t = transforms.data() + static_cast<size_t>(i) * 7;
                t[0] = static_cast<float>(col) * kSpacing;
                t[2] = static_cast<float>(row) * kSpacing;
                t[6] = 1.0f; // identity quaternion (qw)
            }
            if (mPhysX->clone(cloneSource(), targets, transforms.data()) != OVPHYSX_API_SUCCESS)
            {
                printFormatted("OutputWrite: clone(N=%u) failed", mEnvCount);
                return;
            }
            mPhysX->waitAll();
        }

        // DirectGPU refuses reads and writes before the first step.
        mPhysX->step(1.0f / 60.0f);
        mPhysX->waitAll();

        // Here, before ANY write, including the warm-up below. A lane whose write has a
        // precondition (the joint write needs the read to have enumerated the scene's joints) must
        // satisfy it before the first write is attempted, or the warm-up fails and the run reports
        // no timings. It also has to be after the step above, since a read before the first step
        // enumerates nothing.
        if (!afterSceneSetup())
            return;

        // The first session against a fresh scene builds the backend's superset view and its row
        // map, which a warm one reuses. Keep those out of the timed set.
        for (int warm = 0; warm < kWarmUpWrites; ++warm)
        {
            if (!writeOnce())
            {
                printFormatted("OutputWrite: warm-up write failed -- run will report no timings");
                return;
            }
        }
        mSetupOk = true;
        mCountPayload = false; // warm-up is over, from here the gates stay out of the timed region
        mPhases = Phases{}; // discard warm-up: it builds the caches the timed samples then hit

        if (!mReportedShape)
        {
            mReportedShape = true;
            // comp is printed, not assumed. For a per-shape attribute it is the widest body in the
            // session, so a lane that silently loaded the one-collider fixture would report 1 here,
            // which is the difference between measuring the padded per-shape path and measuring
            // nothing. The read's lanes carry the same warning. This makes it checkable.
            printFormatted("OutputWrite: envs=%u groups=%u rows=%u comp=%u", mEnvCount, mLastGroupCount,
                           mLastRowCount, mLastComp);
        }
    }

    // Both gates a lane can fail, run after the timing and before teardown. Subclasses that
    // override endRun() call finishRun() rather than either half, so neither can be forgotten.
    void finishRun()
    {
        checkTimedShape();
        reportFailures();
    }

    void endRun() override
    {
        finishRun();
        reportPhases();
        teardown();
    }

    // Advance the simulation between timed writes, so each one runs against state that actually
    // moved, which is what a caller does, and what forces the row re-resolution the write does
    // per commit. Outside timedStep(), so the step is not in the number. What it changes is what
    // the write has to cope with. Same reasoning as OutputRead's preStep, and it matters more
    // here: without it every session would resolve rows against a scene that never changed.
    void preStep() override
    {
        if (!mSetupOk || !mPhysX)
            return;
        mPhysX->step(1.0f / 60.0f);
        mPhysX->waitAll();
    }

protected:
    // Per-phase wall clock across the timed samples, printed once per run. Measured at the API
    // boundary, which is what a caller actually pays. No phase is skipped for looking like
    // bookkeeping, because on this path the dominant cost has often been in one of those.
    struct Phases
    {
        double query = 0, open = 0, fetch = 0, fill = 0, commit = 0, releaseWrite = 0, releaseQuery = 0;
        uint32_t samples = 0;
    };
    Phases mPhases;

    static double nowMs()
    {
        return std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    void reportPhases()
    {
        if (mPhases.samples == 0)
            return;
        const double n = double(mPhases.samples);
        const double total = mPhases.query + mPhases.open + mPhases.fetch + mPhases.fill + mPhases.commit +
                             mPhases.releaseWrite + mPhases.releaseQuery;
        printFormatted("%s_%u_%s PHASES (ms/sample over %u): query=%.3f open=%.3f fetch=%.3f fill=%.3f "
                       "commit=%.3f releaseWrite=%.3f releaseQuery=%.3f total=%.3f",
                       variantName(), mEnvCount, mGpu ? "gpu" : "cpu", mPhases.samples, mPhases.query / n,
                       mPhases.open / n, mPhases.fetch / n, mPhases.fill / n, mPhases.commit / n,
                       mPhases.releaseWrite / n, mPhases.releaseQuery / n, total / n);
        mPhases = Phases{};
    }

    void teardown()
    {
        if (!mPhysX) return;
        benchmarkClearOvstage(mPhysX, mStageAttachment);
    }

    void step() override
    {
        // Counted here rather than taken from getNbSteps(): --steps=N overrides that.
        ++mAttempts;
        if (!mSetupOk)
        {
            ++mFailedWrites;
            return;
        }
        if (!timedWrite())
            ++mFailedWrites;
    }

    // The exact name the registration macro handed to Register<>, rebuilt from the same three
    // tokens. bmRecordFailure() matches on it, so a failure filed under any other spelling would
    // leave the row publishing its timing.
    std::string registeredRow() const
    {
        return std::string(variantName()) + "_" + std::to_string(mEnvCount) + (mGpu ? "_gpu" : "_cpu");
    }

    // One failure per row, whichever gate trips first: the harness only needs to know the row is
    // bad, and repeating it across runs would bury the rest of the summary.
    bool recordRowFailure()
    {
        if (mFailureRecorded)
            return false;
        mFailureRecorded = true;
        return true;
    }

    // A failed write returns early, so it times as near-zero and makes the run look FASTER. The
    // harness cannot tell that from a genuinely quick write, so route it through the failure
    // channel: the row's sample is discarded and the process exits non-zero.
    void reportFailures()
    {
        const uint32_t failed = mFailedWrites;
        const uint32_t attempts = mAttempts;
        mFailedWrites = 0;
        mAttempts = 0;
        if (failed == 0 || !recordRowFailure())
            return;
        bmRecordFailure(registeredRow().c_str(),
                        "%u of %u writes failed -- the reported time is not a measurement", failed, attempts);
    }

    virtual const char* variantName() const { return "OutputWrite"; }

    // The measured operation. Overridden to compare a different API over the SAME scene.
    virtual bool timedWrite() { return writeOnce(); }

    virtual const char* sceneAsset() const
    {
        return mGpu ? "/../benchmarks/data/cubes20_envs_gpu.usda" : "/../benchmarks/data/cubes20_envs.usda";
    }
    virtual bool clonesEnvs() const { return true; }
    virtual const char* cloneSource() const { return "/World/envs/template"; }
    virtual std::string cloneTarget(uint32_t i) const { return "/World/envs/env" + std::to_string(i + 1); }
    virtual float cloneSpacing() const { return 4.0f; }
    virtual ovphysx_sim_object_type_t objectType() const { return OVPHYSX_OBJECT_RIGID_BODY; }

    // Runs on the loaded, cloned, stepped scene before the first write of any kind, so nothing it
    // does lands inside a timed sample. Returning false fails the run rather than reporting untimed
    // zeros.
    virtual bool afterSceneSetup() { return true; }

    // The single attribute this session writes. A session carries exactly one, so unlike the read's
    // attribute array this is one name. That is also why the write's property lanes measure ONE
    // attribute where the read's measure four: a four-attribute write is four sessions, and timing
    // them as one would hide the per-session cost the split is there to expose.
    virtual ovx_string_or_token_t writeAttr() const
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } };
    }

    bool openQuery(ovphysx_query_handle_t& query)
    {
        query = 0;
        if (ovphysx_query(physx()->handle(), objectType(), OVPHYSX_SCOPE_ALL, &query).status !=
                OVPHYSX_API_SUCCESS ||
            query == 0)
        {
            printFormatted("OutputWrite: ovphysx_query failed");
            return false;
        }
        return true;
    }

    // Write session over an existing query: open -> drain, filling and committing each group ->
    // release the session, not the query.
    bool writeWithQuery(ovphysx_query_handle_t query)
    {
        const ovx_string_or_token_t attr = writeAttr();
        ovphysx_write_handle_t write = 0;
        uint64_t floatCount = 0; // elements the session actually handed over, for the paired-lane check
        const double tOpen = nowMs();
        const bool openOk =
            ovphysx_write(physx()->handle(), query, &attr, &write).status == OVPHYSX_API_SUCCESS && write != 0;
        mPhases.open += nowMs() - tOpen;
        if (!openOk)
        {
            printFormatted("OutputWrite: ovphysx_write failed");
            return false;
        }

        const ovstage_map_group_t* group = nullptr;
        uint32_t groups = 0;
        int64_t rows = 0;
        bool drained = false;
        for (;;)
        {
            const double tFetch = nowMs();
            const ovphysx_api_status_t status = ovphysx_fetch_write_next(physx()->handle(), write, &group).status;
            mPhases.fetch += nowMs() - tFetch;
            if (status == OVPHYSX_API_END_OF_ITERATION)
            {
                drained = true;
                break;
            }
            if (status != OVPHYSX_API_SUCCESS || group == nullptr)
            {
                printFormatted("OutputWrite: ovphysx_fetch_write_next failed after %u groups", groups);
                break;
            }
            if (groups == 0 && group->data.tensors && group->data.tensor_count > 0 &&
                group->data.tensors[0].shape)
            {
                rows = group->data.tensors[0].shape[0];
                const DLDataType dt = group->data.tensors[0].dtype;
                mLastComp = dt.lanes ? dt.lanes : 1;
            }
            ++groups;
            // Counted while warming only: this is what validateBindingLayout() compares against the
            // binding, and it must not add a loop to the measured region.
            if (mCountPayload && group->data.tensors)
            {
                for (size_t t = 0; t < group->data.tensor_count; ++t)
                {
                    const DLTensor& dt = group->data.tensors[t];
                    if (!dt.shape)
                        continue;
                    uint64_t elements = dt.dtype.lanes ? dt.dtype.lanes : 1;
                    for (int32_t d = 0; d < dt.ndim; ++d)
                        elements *= static_cast<uint64_t>(dt.shape[d]);
                    floatCount += elements;
                }
            }

            const double tFill = nowMs();
            const bool filled = fillGroup(*group);
            mPhases.fill += nowMs() - tFill;
            if (!filled)
            {
                printFormatted("OutputWrite: fill failed after %u groups", groups);
                break;
            }
            // {0,0}: the fill above is already complete on this thread's stream by the time commit
            // is called, so there is no producer handoff to declare. A caller filling from its own
            // async stream would pass it here instead.
            const ovstage_cuda_sync_t noSync{ 0, 0 };
            const double tCommit = nowMs();
            const bool committed =
                ovphysx_commit_group(physx()->handle(), write, group, noSync).status == OVPHYSX_API_SUCCESS;
            mPhases.commit += nowMs() - tCommit;
            if (!committed)
            {
                printFormatted("OutputWrite: ovphysx_commit_group failed after %u groups", groups);
                break;
            }
            group = nullptr;
        }

        const double tRel = nowMs();
        ovphysx_release_write(physx()->handle(), write);
        mPhases.releaseWrite += nowMs() - tRel;
        ++mPhases.samples;
        if (!drained)
            return false;

        mLastGroupCount = groups;
        mLastRowCount = static_cast<uint32_t>(rows);
        // EVERY timed write contributes its shape to these extremes. The comparison against the
        // warmed shape happens once, after the timing, in checkTimedShape(), so the measured region
        // gains four min/max updates beside assignments that already happen, and the branching is
        // deferred to a point where nothing is being timed. Recording only the last write's shape
        // would miss an iteration that drifted and drifted back.
        if (!mCountPayload)
        {
            const uint32_t rowsU = static_cast<uint32_t>(rows);
            mTimedGroupMin = mTimedGroupMin < groups ? mTimedGroupMin : groups;
            mTimedGroupMax = mTimedGroupMax > groups ? mTimedGroupMax : groups;
            mTimedRowMin = mTimedRowMin < rowsU ? mTimedRowMin : rowsU;
            mTimedRowMax = mTimedRowMax > rowsU ? mTimedRowMax : rowsU;
        }
        mTimedShapeSeen = !mCountPayload;
        if (mCountPayload)
        {
            mLastFloatCount = floatCount;
            // A session that drains cleanly but reached nothing is measuring an empty write, which
            // times fast and means nothing. Checked while warming rather than per timed write, to
            // keep the branch out of the measured region.
            if (groups == 0 || rows <= 0)
            {
                printFormatted(
                    "OutputWrite: drained successfully but covered groups=%u rows=%lld -- invalid benchmark run",
                    groups, static_cast<long long>(rows));
                return false;
            }
            mExpectedGroupCount = groups;
            mExpectedRowCount = static_cast<uint32_t>(rows);
        }
        return true;
    }

    // The payload gate runs during warm-up only, to keep the branch out of the measurement. A lane
    // that stopped covering anything partway through the run would otherwise drain clean and time
    // fast, so the shape every timed write reached is compared against the warmed one here, once
    // the timing is over.
    //
    // This assumes a lane's shape is fixed across a run. One whose row count legitimately varies
    // (a scope filter that reaches only what moved, say) would have to opt out of it.
    void checkTimedShape()
    {
        if (!mTimedShapeSeen)
            return;
        mTimedShapeSeen = false;
        const bool groupsSteady = mTimedGroupMin == mExpectedGroupCount && mTimedGroupMax == mExpectedGroupCount;
        const bool rowsSteady = mTimedRowMin == mExpectedRowCount && mTimedRowMax == mExpectedRowCount;
        if (groupsSteady && rowsSteady)
            return;
        if (!recordRowFailure())
            return;
        bmRecordFailure(registeredRow().c_str(),
                        "write covered groups=%u..%u rows=%u..%u after warming at groups=%u rows=%u",
                        mTimedGroupMin, mTimedGroupMax, mTimedRowMin, mTimedRowMax, mExpectedGroupCount,
                        mExpectedRowCount);
    }

    // Fill one group's column with the payload the caller would supply. This IS the caller's copy:
    // the session handed over the destination, so there is no second one to skip.
    bool fillGroup(const ovstage_map_group_t& group)
    {
        if (!group.data.tensors || group.data.tensor_count == 0)
            return true;
        const DLTensor& t = group.data.tensors[0];
        if (!t.data || !t.shape)
            return true;
        const size_t lanes = t.dtype.lanes ? t.dtype.lanes : 1;
        const size_t floats = static_cast<size_t>(t.shape[0]) * lanes;
        if (floats == 0)
            return true;

        if (t.device.device_type == kDLCUDA)
        {
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
            // Host->device, which is what a caller with a host-side policy pays. A caller already
            // producing on device would write in place and pay nothing here (the case this API
            // exists to enable), so this lane is the PESSIMISTIC one, not the flattering one.
            if (mHostPayload.size() < floats)
                mHostPayload.assign(floats, 0.5f);
            if (cudaMemcpy(t.data, mHostPayload.data(), floats * sizeof(float), cudaMemcpyHostToDevice) !=
                cudaSuccess)
                return false;
            return true;
#else
            return false;
#endif
        }
        float* dst = static_cast<float*>(t.data);
        for (size_t i = 0; i < floats; ++i)
            dst[i] = 0.5f;
        return true;
    }

private:
    // Query + session: what a caller pays carrying no state between frames.
    bool writeOnce()
    {
        ovphysx_query_handle_t query = 0;
        const double tQuery = nowMs();
        const bool opened = openQuery(query);
        mPhases.query += nowMs() - tQuery;
        if (!opened)
            return false;
        const bool ok = writeWithQuery(query);
        const double tRq = nowMs();
        ovphysx_release_query(physx()->handle(), query);
        mPhases.releaseQuery += nowMs() - tRq;
        return ok;
    }

protected:
    bool mSetupOk = false;

    uint32_t mEnvCount;
    bool mGpu;
    ovphysx::PhysX* physx() const { return mPhysX; }
    // True only while startRun() is warming. It keeps payload accounting out of the timed region.
    // Protected because a paired lane closes its own warm-up window after binding.
    bool mCountPayload = true;
    // Elements the warmed write session handed over, which a paired lane compares its binding to.
    uint64_t lastFloatCount() const { return mLastFloatCount; }

private:
    std::vector<float> mHostPayload; // staging for the device fill; grown once
    uint32_t mFailedWrites = 0;
    uint32_t mAttempts = 0;
    uint32_t mLastGroupCount = 0;
    uint32_t mLastRowCount = 0;
    uint32_t mLastComp = 0;
    uint64_t mLastFloatCount = 0;
    // The shape the warm-up settled on, and whether a timed write has produced one to compare.
    uint32_t mExpectedGroupCount = 0;
    uint32_t mExpectedRowCount = 0;
    bool mTimedShapeSeen = false;
    // Seeded so the first timed write sets both ends. checkTimedShape() only consults them when
    // mTimedShapeSeen says a timed write actually happened.
    uint32_t mTimedGroupMin = UINT32_MAX;
    uint32_t mTimedGroupMax = 0;
    uint32_t mTimedRowMin = UINT32_MAX;
    uint32_t mTimedRowMax = 0;
    bool mFailureRecorded = false;
    bool mReportedShape = false;
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


// Write with the query opened once and held across steps. A query is a reusable selector resolved
// lazily, so this is legitimate usage. Against querywrite_* the difference is what re-opening the
// selector costs every frame.
class WriteOnlyBase : public OutputWriteBase
{
public:
    WriteOnlyBase(uint32_t envCount, bool gpu) : OutputWriteBase(envCount, gpu) {}

    void startRun() override
    {
        OutputWriteBase::startRun();
        if (!mSetupOk) return;
        mSetupOk = false;
        if (!physx()) return;
        if (!openQuery(mQuery))
            return;
        for (int warm = 0; warm < kWarmUpWrites; ++warm)
        {
            if (!writeWithQuery(mQuery))
            {
                printFormatted("OutputWriteOnly: warm-up write failed -- run will report no timings");
                return;
            }
        }
        mSetupOk = true;
        mPhases = Phases{}; // discard warm-up
    }

    void endRun() override
    {
        finishRun();
        // This override replaces the base's endRun, and with it the phase report, so the report is
        // re-issued here. Otherwise every writeonly-derived lane would print no phase split while
        // querywrite, which keeps the base's endRun, still would.
        reportPhases();
        if (mQuery)
        {
            ovphysx_release_query(physx()->handle(), mQuery);
            mQuery = 0;
        }
        teardown();
    }

protected:
    bool timedWrite() override { return writeWithQuery(mQuery); }

private:
    ovphysx_query_handle_t mQuery = 0;
};


// The same direction through the tensor-binding API: one pose write of [N, 7].
//
// Not a like-for-like payload. The binding's smallest pose write carries orientation too, where
// the session writes position alone. What it answers is the question that actually matters when
// choosing between them: what does one attribute update cost through each API, including the copy
// each one forces on the caller.
//
// The source matches the scene: device memory on DirectGPU, host on CPU. Binding a host tensor on
// a DirectGPU scene would force a host-to-device copy per write and measure the API in the mode
// that defeats it.
class TensorBindingWriteBase : public OutputWriteBase
{
public:
    TensorBindingWriteBase(uint32_t envCount, bool gpu) : OutputWriteBase(envCount, gpu) {}

    bool isValid() const override
    {
#if !defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        // Without CUDA the source silently falls back to host memory, which turns this into a
        // host-backed write reported under a name that says device.
        if (mGpu)
        {
            throw std::runtime_error(
                "OutputWrite tensorbinding GPU variant requires a CUDAToolkit-enabled benchmark build");
        }
#endif
        return OutputWriteBase::isValid();
    }

    // A binding write is well below the session the base is tuned for, so it needs more samples to
    // converge. Matches TensorBindingRead's reasoning.
    uint32_t getNbSteps() const override { return mEnvCount >= 8192 ? 50 : 100; }

    void startRun() override
    {
        OutputWriteBase::startRun();
        if (!mSetupOk) return;

        mSetupOk = false;
        if (!physx()) return;
        if (!bind(mPose, mPoseStorage, mPoseDev, mPoseShape, mPoseView, bindingTensorType(), bindingName()))
            return;
        for (int warm = 0; warm < kWarmUpWrites; ++warm)
        {
            if (!timedWrite())
            {
                printFormatted("TensorBindingWrite: warm-up write failed -- run will report no timings");
                return;
            }
        }
        if (!validateBindingLayout())
            return;
        mSetupOk = true;
        mCountPayload = false;
        mPhases = Phases{}; // discard warm-up, as the base does

        if (!mReportedBinding)
        {
            mReportedBinding = true;
            long long elems = mPoseShape.empty() ? 0 : 1;
            for (int64_t d : mPoseShape)
                elems *= static_cast<long long>(d);
            std::string shape;
            for (size_t i = 0; i < mPoseShape.size(); ++i)
                shape += (i ? "x" : "") + std::to_string(mPoseShape[i]);
            printFormatted("TensorBindingWrite: envs=%u src=%s pose=%s (%lld floats)", mEnvCount,
                           mGpu ? "device" : "host", shape.empty() ? "-" : shape.c_str(), elems);
        }
    }

    void endRun() override
    {
        finishRun();
        mPose.destroy();
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        (void)cudaFree(mPoseDev);
#endif
        mPoseDev = nullptr;
        teardown();
    }

protected:
    virtual const char* bindingPattern() const { return kBodyPattern; }
    // Which tensor this lane writes. A seam rather than a constant so the property lane can face the
    // property write over the same fixture. The allocation, seeding and timed loop are identical.
    virtual ovphysx_tensor_type_t bindingTensorType() const { return OVPHYSX_TENSOR_RIGID_BODY_POSE_F32; }
    virtual const char* bindingName() const { return "pose"; }
    // Stride of the value the seed has to make VALID, and which lane of it carries the 1. A pose is
    // 7 floats with qw last. An inertia is 9 with a positive diagonal. Seeding zeros would hand PhysX
    // a degenerate quaternion or a singular inertia and change what the solver does between samples.
    virtual uint32_t seedStride() const { return 7; }
    virtual void seedRow(float* row) const { row[6] = 1.0f; } // qw

    bool timedWrite() override
    {
        if (mPose.write(mPoseView) != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorBindingWrite: %s write failed", bindingName());
            return false;
        }
        physx()->waitAll();
        return true;
    }

private:
    bool bind(ovphysx::TensorBinding& binding,
              std::vector<float>& storage,
              void*& deviceStorage,
              std::vector<int64_t>& shape,
              DLTensor& view,
              ovphysx_tensor_type_t type,
              const char* what)
    {
        if (physx()->createTensorBinding(binding, bindingPattern(), type) != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorBindingWrite: createTensorBinding(%s) failed", what);
            return false;
        }
        ovphysx_tensor_spec_t spec{};
        if (binding.spec(spec) != OVPHYSX_API_SUCCESS || spec.ndim < 1)
        {
            printFormatted("TensorBindingWrite: spec(%s) failed", what);
            return false;
        }
        size_t elems = 1;
        shape.assign(static_cast<size_t>(spec.ndim), 0);
        for (int i = 0; i < spec.ndim; ++i)
        {
            shape[i] = spec.shape[i];
            elems *= static_cast<size_t>(spec.shape[i]);
        }
        view = DLTensor{};
        (void)deviceStorage; // only the CUDA build reaches it, isValid() refuses _gpu without it
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        if (mGpu)
        {
            if (cudaMalloc(&deviceStorage, elems * sizeof(float)) != cudaSuccess)
            {
                printFormatted("TensorBindingWrite: cudaMalloc(%s) failed", what);
                return false;
            }
            // Seeded once with a valid pose (identity quaternion per row): writing uninitialized
            // device memory would hand PhysX denormal or NaN quaternions and change what the
            // solver does between samples.
            std::vector<float> seed(elems, 0.0f);
            for (size_t i = 0; i + seedStride() <= elems; i += seedStride())
                seedRow(seed.data() + i);
            if (cudaMemcpy(deviceStorage, seed.data(), elems * sizeof(float), cudaMemcpyHostToDevice) !=
                cudaSuccess)
            {
                printFormatted("TensorBindingWrite: cudaMemcpy(%s) failed", what);
                return false;
            }
            view.data = deviceStorage;
            view.device = DLDevice{ kDLCUDA, 0 };
        }
        else
#endif
        {
            storage.assign(elems, 0.0f);
            for (size_t i = 0; i + seedStride() <= elems; i += seedStride())
                seedRow(storage.data() + i);
            view.data = storage.data();
            view.device = DLDevice{ kDLCPU, 0 };
        }
        view.ndim = static_cast<int32_t>(shape.size());
        view.dtype = DLDataType{ static_cast<uint8_t>(kDLFloat), 32, 1 };
        view.shape = shape.data();
        view.strides = nullptr;
        view.byte_offset = 0;
        return true;
    }

    ovphysx::TensorBinding mPose;
    std::vector<float> mPoseStorage; // CPU scenes
    void* mPoseDev = nullptr;        // DirectGPU scenes
    // Total elements, not shape[0]: the two APIs disagree about what a row is (a binding may
    // return [envs, links, comps] where the write session hands over one row per prim), so the
    // leading dimension differs while the payload covered is identical.
    //
    // A lane whose binding legitimately covers a different payload has to override this and say
    // why. It cannot skip the check by saying nothing.
    virtual bool validateBindingLayout() const
    {
        const uint64_t bound = bindingFloatCount();
        const uint64_t written = lastFloatCount();
        if (bound == written)
            return true;
        printFormatted("TensorBindingWrite: the binding covers %llu floats but the write session covers %llu -- "
                       "the two APIs did not reach the same payload, so no ratio is reported",
                       static_cast<unsigned long long>(bound), static_cast<unsigned long long>(written));
        return false;
    }

    uint64_t bindingFloatCount() const
    {
        uint64_t count = 0;
        for (const std::vector<int64_t>* shape : bindingShapes())
        {
            if (shape->empty())
                continue;
            uint64_t elements = 1;
            for (int64_t d : *shape)
                elements *= static_cast<uint64_t>(d);
            count += elements;
        }
        return count;
    }

    // Every shape this lane binds. Overridden by lanes that bind more than the one column.
    virtual std::vector<const std::vector<int64_t>*> bindingShapes() const
    {
        return { &mPoseShape };
    }

    std::vector<int64_t> mPoseShape;
    DLTensor mPoseView{};
    bool mReportedBinding = false;
};


// The body-PROPERTY write. `inertia` on purpose rather than `mass`: it is the widest of the set
// (9 floats), it reconstructs a diagonalisation per body on the way in, and it is the one whose
// write has a documented side effect (it overwrites the COM orientation). A lane that measured
// `mass` would be timing one setter call per body and would say nothing about the rest.
//
// CPU-REGISTERED ONLY, deliberately and for the same reason the read's property lanes are: these
// attributes have no device destination, so a GPU lane would time the identical host loop against a
// binding that cannot take a device tensor either. There is no comparison to make and the ratio
// would be meaningless. The scene branch still honours mGpu so the trap is not left for whoever
// adds one.
class PropertyWriteBase : public WriteOnlyBase
{
public:
    PropertyWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_INERTIA, sizeof(OVPHYSX_ATTR_INERTIA) - 1 } };
    }
};

// The same payload through the binding, for the ratio. setInertias is the binding's equivalent and
// carries the same diagonalisation, so the two sides do the same work per body.
class PropertyBindingWriteBase : public TensorBindingWriteBase
{
public:
    PropertyBindingWriteBase(uint32_t envCount, bool gpu) : TensorBindingWriteBase(envCount, gpu) {}

protected:
    ovphysx_tensor_type_t bindingTensorType() const override { return OVPHYSX_TENSOR_RIGID_BODY_INERTIA_F32; }
    const char* bindingName() const override { return "inertia"; }
    uint32_t seedStride() const override { return 9; }
    // A positive diagonal: a singular or zero inertia is not a value PhysX accepts as a body's, and
    // seeding one would change what is being measured into error handling.
    void seedRow(float* row) const override { row[0] = 1.0f; row[4] = 1.0f; row[8] = 1.0f; }
};

// Per-shape, on the COMPOUND fixture. cubes20_envs.usda is one collider per body, which is the
// degenerate width for a padded per-shape column: no padding, no bodies x shapes depth, and a lane
// that would look identical to a per-body one. cubes20_compound_envs.usda is the same twenty bodies
// with four colliders each, and it is the read's fixture too, which is what keeps the two
// directions' per-shape numbers comparable.
class ShapeWriteBase : public WriteOnlyBase
{
public:
    ShapeWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    // `staticFriction` rather than `contactOffset`: it carries the material indirection
    // (getMaterials per shape, one hop more than a shape getter), which is the cost that makes a
    // per-shape write different from a per-body one.
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_STATIC_FRICTION, sizeof(OVPHYSX_ATTR_STATIC_FRICTION) - 1 } };
    }
    // Branches on mGpu as the base does, so a GPU registration cannot silently load the CPU scene.
    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cubes20_compound_envs_gpu.usda" :
                      "/../benchmarks/data/cubes20_compound_envs.usda";
    }
};


// The articulation's ROOT state, through OVPHYSX_OBJECT_ARTICULATION.
//
// Cartpole rather than the cubes scene, matching OutputRead's articulation lanes exactly (same
// asset, same clone source, same 4.0 spacing), so a root-write number can be read against the
// arti_link and arti_dof read numbers beside it. One articulation per env, so this lane moves ~8192
// rows where the rigid lanes move ~164k: the per-row cost is comparable, the totals are not.
//
// `rootPosition` rather than `rootLinearVelocity`, deliberately. Root pose is the one that pays the
// read-modify-write. PxArticulationGPUAPIWriteType carries a single eROOT_GLOBAL_POSE, so writing
// position alone must first read the block back to keep the orientation. That is the cost worth
// measuring. A velocity lane would measure the cheap path and read as though the RMW were free.
//
// Both devices, unlike the property lanes: a root write HAS a device destination, so a GPU
// lane measures the DirectGPU path rather than the same host loop under a different name.
class ArticulationRootWriteBase : public WriteOnlyBase
{
public:
    ArticulationRootWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{
            0, { OVPHYSX_ATTR_ROOT_POSITION, sizeof(OVPHYSX_ATTR_ROOT_POSITION) - 1 }
        };
    }
    // Branches on mGpu as the rigid base does. cartpole_probe.usda authors enableGPUDynamics + GPU
    // broadphase. Running it from a CPU lane would measure the GPU pipeline under a name that says
    // cpu, which is what the _cpu asset exists to prevent.
    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                      "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    // env_0 is the template AND stays in the scene, so N clones cover N+1 articulations.
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
};


// The same root pose through the binding, which is what gives the lane above something to be read
// against. ARTICULATION_ROOT_POSE_F32 is [N, 7], where the session writes [N, 3], the same
// payload asymmetry the rigid pair carries, and for the same reason: the binding's smallest root
// write includes the orientation. Read it as "what does moving a root cost through each API".
class ArticulationRootBindingWriteBase : public TensorBindingWriteBase
{
public:
    ArticulationRootBindingWriteBase(uint32_t envCount, bool gpu) : TensorBindingWriteBase(envCount, gpu) {}

protected:
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t bindingTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32;
    }
    const char* bindingName() const override { return "root pose"; }
    // 7 floats with qw last, same as the rigid pose: seeding zeros would hand PhysX a degenerate
    // quaternion and change what the solver does between samples.
    uint32_t seedStride() const override { return 7; }
    void seedRow(float* row) const override { row[6] = 1.0f; }

    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                      "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
};


// Articulation joint DOFs, the per-axis ARRAY path, one tensor per joint rather than one stacking
// every prim. No rigid lane has that shape, so this has nothing to be compared against on the
// ovphysx side. The binding lane below is what gives it a facing number.
//
// THIS LANE MUST PRIME THE JOINT CACHE, and that is a property of the API rather than of the
// benchmark. The joint write derives its joint set from the cache the READ fills, and refuses
// outright when nothing has read the scene's joints yet. The write deliberately does not
// re-implement the enumeration, so a second copy cannot drift from the first. A caller round-tripping
// joint state reads first anyway, so priming here measures the same order a real one uses. The prime
// is in startRun, before the warm-up writes, so it is never inside a timed sample.
class ArticulationDofWriteBase : public WriteOnlyBase
{
public:
    ArticulationDofWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    // The prime, through the hook rather than by bracketing startRun: the scene has to be the one
    // the warm-up and the timed writes use. The joint cache is keyed by PxScene, so priming a scene
    // that is then reloaded caches an entry for a scene nothing will write to.
    bool afterSceneSetup() override
    {
        if (primeJointCache())
            return true;
        printFormatted("ArticulationDofWrite: joint-cache prime failed -- run will report no timings");
        return false;
    }

    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_JOINT; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0,
                                      { OVPHYSX_ATTR_JOINT_POSITION, sizeof(OVPHYSX_ATTR_JOINT_POSITION) - 1 } };
    }
    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                      "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }

private:
    // One read of jointPosition, fully drained. Not timed.
    bool primeJointCache()
    {
        if (!physx())
            return false;
        ovphysx_query_handle_t query = 0;
        if (!openQuery(query))
            return false;

        const ovx_string_or_token_t attr = writeAttr();
        ovphysx_read_handle_t read = 0;
        bool ok = ovphysx_read(physx()->handle(), query, &attr, 1, &read).status == OVPHYSX_API_SUCCESS &&
                  read != 0;
        if (ok)
        {
            const ovstage_read_group_t* group = nullptr;
            for (;;)
            {
                const ovphysx_api_status_t status =
                    ovphysx_fetch_read_next(physx()->handle(), read, &group).status;
                if (status == OVPHYSX_API_END_OF_ITERATION)
                    break;
                if (status != OVPHYSX_API_SUCCESS || group == nullptr)
                {
                    ok = false;
                    break;
                }
                const ovstage_read_group_id_t groupId = group->read_group_id;
                group = nullptr;
                if (ovphysx_release_group(physx()->handle(), read, groupId).status != OVPHYSX_API_SUCCESS)
                {
                    ok = false;
                    break;
                }
            }
            ovphysx_release_read(physx()->handle(), read);
        }
        ovphysx_release_query(physx()->handle(), query);
        return ok;
    }
};


// The DOF write through the binding: ARTICULATION_DOF_POSITION_F32, [N, D]. Unlike the root pair
// this IS close to like-for-like (both sides move one scalar per DOF), so the ratio here is
// readable as API overhead rather than as a payload difference.
class ArticulationDofBindingWriteBase : public TensorBindingWriteBase
{
public:
    ArticulationDofBindingWriteBase(uint32_t envCount, bool gpu) : TensorBindingWriteBase(envCount, gpu) {}

protected:
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t bindingTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32;
    }
    const char* bindingName() const override { return "dof position"; }
    // A DOF position is one scalar with no validity constraint (no quaternion to keep normalized
    // and no matrix to keep non-singular), so a zeroed row is a legitimate value here, unlike the
    // pose and inertia lanes.
    uint32_t seedStride() const override { return 1; }
    void seedRow(float*) const override {}

    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                      "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
};


// Tendons (ADR-0012), on the cartpole_tendon scene OutputRead's tendon lanes use (same asset, same
// clone source, same spacing), so a write number reads directly against the read number beside it.
//
// `tendonStiffness`, matching tendonReadAttrs' first column, so the fixed and spatial write lanes
// measure identical columns and can be compared with each other and with the read.
//
// ONE tendon of each kind per env, against 164k bodies in the rigid lanes: the smallest row count in
// this file. That is the point rather than a weakness. Tendon properties are authoring-time values,
// so what a lane answers here is what the SESSION costs when the payload is nearly nothing, which is
// where the fixed per-session cost shows up undiluted.
template <typename WriteBase>
class CartpoleTendonWriteSceneT : public WriteBase
{
public:
    CartpoleTendonWriteSceneT(uint32_t envCount, bool gpu) : WriteBase(envCount, gpu) {}

protected:
    const char* sceneAsset() const override
    {
        return this->mGpu ? "/../benchmarks/data/cartpole_tendon_probe.usda" :
                            "/../benchmarks/data/cartpole_tendon_probe_cpu.usda";
    }
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{
            0, { OVPHYSX_ATTR_TENDON_STIFFNESS, sizeof(OVPHYSX_ATTR_TENDON_STIFFNESS) - 1 }
        };
    }
};

class FixedTendonWriteBase : public CartpoleTendonWriteSceneT<WriteOnlyBase>
{
public:
    FixedTendonWriteBase(uint32_t envCount, bool gpu) : CartpoleTendonWriteSceneT<WriteOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_FIXED_TENDON; }
};

class SpatialTendonWriteBase : public CartpoleTendonWriteSceneT<WriteOnlyBase>
{
public:
    SpatialTendonWriteBase(uint32_t envCount, bool gpu) : CartpoleTendonWriteSceneT<WriteOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_SPATIAL_TENDON; }
};

// The binding lanes the two above face, matching OutputRead's tendon binding lanes.
class FixedTendonBindingWriteBase : public CartpoleTendonWriteSceneT<TensorBindingWriteBase>
{
public:
    FixedTendonBindingWriteBase(uint32_t envCount, bool gpu)
        : CartpoleTendonWriteSceneT<TensorBindingWriteBase>(envCount, gpu)
    {
    }

protected:
    // Inherited from OutputWriteBase as RIGID_BODY otherwise, which is not inert: the base's
    // startRun does warm-up ovphysx WRITES before the binding is exercised, so a rigid session
    // asking for tendonStiffness is refused and the whole lane reports Skipped. The timed work
    // still goes through the binding. This only makes the warm-up coherent.
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_FIXED_TENDON; }
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t bindingTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_FIXED_TENDON_STIFFNESS_F32;
    }
    const char* bindingName() const override { return "fixed tendon stiffness"; }
    // A stiffness is one scalar with no validity constraint (no quaternion to keep normalized and
    // no matrix to keep non-singular), so a zeroed row is a legitimate value here.
    uint32_t seedStride() const override { return 1; }
    void seedRow(float*) const override {}
};

class SpatialTendonBindingWriteBase : public CartpoleTendonWriteSceneT<TensorBindingWriteBase>
{
public:
    SpatialTendonBindingWriteBase(uint32_t envCount, bool gpu)
        : CartpoleTendonWriteSceneT<TensorBindingWriteBase>(envCount, gpu)
    {
    }

protected:
    // Inherited from OutputWriteBase as RIGID_BODY otherwise, which is not inert: the base's
    // startRun does warm-up ovphysx WRITES before the binding is exercised, so a rigid session
    // asking for tendonStiffness is refused and the whole lane reports Skipped. The timed work
    // still goes through the binding. This only makes the warm-up coherent.
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_SPATIAL_TENDON; }
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t bindingTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_SPATIAL_TENDON_STIFFNESS_F32;
    }
    const char* bindingName() const override { return "spatial tendon stiffness"; }
    uint32_t seedStride() const override { return 1; }
    void seedRow(float*) const override {}
};


// Force and wrench (ADR-0012), the write-only pair.
//
// NO READ LANE TO FACE, unlike every other family here: the read never emits these, so the tensor
// binding is the only comparison available. That makes the binding lanes load-bearing rather than
// supplementary.
//
// This is the comparison worth having: the binding's own path carries a host block that the
// ovstage path does not. applyForces flags every body, compacts with thrust::copy_if and takes the
// resulting COUNT off the device (fillRdFT returns an iterator difference), with
// cudaStreamSynchronize around it. The session knows its rows, so its count is numOutputs and no
// compaction runs.
class ForceWriteBase : public WriteOnlyBase
{
public:
    ForceWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_FORCE, sizeof(OVPHYSX_ATTR_FORCE) - 1 } };
    }
};

// The 9-wide one. Against `force` it isolates what the extra six floats cost: a pose read, the
// torque-about-COM conversion and a second DirectGPU write type, over the same bodies.
class WrenchWriteBase : public WriteOnlyBase
{
public:
    WrenchWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_WRENCH, sizeof(OVPHYSX_ATTR_WRENCH) - 1 } };
    }
};

// The DirectGPU LINK route, which is a different code path rather than the same one over other rows:
// (articulation, link) addressing, a whole-view block, and no packed rigid indices at all. On
// cartpole, so the row count is links rather than the cubes scene's bodies.
class LinkWrenchWriteBase : public WriteOnlyBase
{
public:
    LinkWrenchWriteBase(uint32_t envCount, bool gpu) : WriteOnlyBase(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_LINK; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_WRENCH, sizeof(OVPHYSX_ATTR_WRENCH) - 1 } };
    }
    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                      "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
};

// The binding lanes these three face. objectType() is overridden on each. The base runs warm-up
// ovphysx WRITES before the binding is exercised, and a mismatched (type, attribute) pair makes the
// whole lane report Skipped rather than fail.
class ForceBindingWriteBase : public TensorBindingWriteBase
{
public:
    ForceBindingWriteBase(uint32_t envCount, bool gpu) : TensorBindingWriteBase(envCount, gpu) {}

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_FORCE, sizeof(OVPHYSX_ATTR_FORCE) - 1 } };
    }
    ovphysx_tensor_type_t bindingTensorType() const override { return OVPHYSX_TENSOR_RIGID_BODY_FORCE_F32; }
    const char* bindingName() const override { return "force"; }
    // A force has no validity constraint (no quaternion to keep normalized, no matrix to keep
    // non-singular), so an all-zero seed is a legitimate value, unlike the pose and inertia lanes.
    uint32_t seedStride() const override { return 3; }
    void seedRow(float*) const override {}
};

class WrenchBindingWriteBase : public TensorBindingWriteBase
{
public:
    WrenchBindingWriteBase(uint32_t envCount, bool gpu) : TensorBindingWriteBase(envCount, gpu) {}

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_WRENCH, sizeof(OVPHYSX_ATTR_WRENCH) - 1 } };
    }
    ovphysx_tensor_type_t bindingTensorType() const override { return OVPHYSX_TENSOR_RIGID_BODY_WRENCH_F32; }
    const char* bindingName() const override { return "wrench"; }
    uint32_t seedStride() const override { return 9; }
    void seedRow(float*) const override {}
};

class LinkWrenchBindingWriteBase : public TensorBindingWriteBase
{
public:
    LinkWrenchBindingWriteBase(uint32_t envCount, bool gpu) : TensorBindingWriteBase(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_LINK; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_WRENCH, sizeof(OVPHYSX_ATTR_WRENCH) - 1 } };
    }
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t bindingTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_LINK_WRENCH_F32;
    }
    const char* bindingName() const override { return "link wrench"; }
    uint32_t seedStride() const override { return 9; }
    void seedRow(float*) const override {}
    const char* sceneAsset() const override
    {
        return mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                      "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
};


// Point-instancer instances (ADR-0012), on the same asset OutputRead's instancer lanes use, so a
// write number reads directly against the read number beside it.
//
// The one family where the write does MORE work than the read rather than the mirror of it:
//
//   read    world pose -> reframe to instancer-local -> publish
//   write   read the CURRENT world pose -> reframe to local -> substitute the caller's half ->
//           compose back out -> publish
//
// So the write carries the read's reframe PLUS an inverse composition PLUS a DirectGPU pose pre-read,
// per instance. Everywhere else the two directions do comparable work and the gap is session setup.
// Here there is a real asymmetry, and this lane is what says how much it costs.
//
// The read-modify-write is not avoidable: a session carries one attribute, and a pose needs both
// halves.
template <typename WriteBase>
class Cubes20InstancerWriteSceneT : public WriteBase
{
public:
    Cubes20InstancerWriteSceneT(uint32_t envCount, bool gpu) : WriteBase(envCount, gpu) {}

protected:
    const char* sceneAsset() const override
    {
        return this->mGpu ? "/../benchmarks/data/cubes20_instancer_envs_gpu.usda" :
                            "/../benchmarks/data/cubes20_instancer_envs.usda";
    }
    // cloneSource, cloneTarget and cloneSpacing are the base's, as on the read's mixin: the asset has
    // the same /World/envs/template layout as cubes20_envs.usda by construction.
};

// `position`, so the lane exercises the reframe inverse. A velocity column would skip it entirely
// (velocities are world-frame in both directions) and measure only the packed scatter.
class InstancerWriteBase : public Cubes20InstancerWriteSceneT<WriteOnlyBase>
{
public:
    InstancerWriteBase(uint32_t envCount, bool gpu) : Cubes20InstancerWriteSceneT<WriteOnlyBase>(envCount, gpu) {}
};

// The velocity column, against the pose one: the difference IS the reframe inverse plus the pose
// pre-read, over the same instances on the same scene. That subtraction is the number this pair
// exists to produce.
class InstancerVelocityWriteBase : public Cubes20InstancerWriteSceneT<WriteOnlyBase>
{
public:
    InstancerVelocityWriteBase(uint32_t envCount, bool gpu)
        : Cubes20InstancerWriteSceneT<WriteOnlyBase>(envCount, gpu)
    {
    }

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_LINEAR_VELOCITY, sizeof(OVPHYSX_ATTR_LINEAR_VELOCITY) - 1 } };
    }
};


// One macro per family, matching OutputRead's: the registered name is derived from the same tokens
// that build the class, so they cannot disagree.
#define DEFINE_OUTPUT_WRITE_VARIANT(base, cls, op, N, device, gpu, hide)       \
    class cls##_##N##_##device : public base                                   \
    {                                                                          \
    public:                                                                    \
        cls##_##N##_##device() : base(N, gpu) {}                               \
                                                                               \
    protected:                                                                 \
        const char* variantName() const override { return "OutputWrite." #op; } \
    };                                                                         \
    Register<cls##_##N##_##device, hide> s##cls##_##N##_##device(              \
        "OutputWrite." #op "_" #N "_" #device);

#define DEFINE_QUERY_WRITE(N, device, gpu, hide)                               \
    DEFINE_OUTPUT_WRITE_VARIANT(OutputWriteBase, OutputWrite, querywrite_rb, N, device, gpu, hide)

#define DEFINE_WRITE_ONLY(N, device, gpu, hide)                                \
    DEFINE_OUTPUT_WRITE_VARIANT(WriteOnlyBase, OutputWriteOnly, writeonly_rb, N, device, gpu, hide)

#define DEFINE_TENSOR_BINDING_WRITE(N, device, gpu, hide)                      \
    DEFINE_OUTPUT_WRITE_VARIANT(TensorBindingWriteBase, TensorBindingWrite, tensorbinding_rb, N, device, gpu, hide)

#define DEFINE_PROPERTY_WRITE(N, device, gpu, hide)                            \
    DEFINE_OUTPUT_WRITE_VARIANT(PropertyWriteBase, PropertyWrite, writeonly_property_rb, N, device, gpu, hide)

#define DEFINE_PROPERTY_BINDING_WRITE(N, device, gpu, hide)                    \
    DEFINE_OUTPUT_WRITE_VARIANT(PropertyBindingWriteBase, PropertyBindingWrite, tensorbinding_property, N, device, gpu, hide)

#define DEFINE_SHAPE_WRITE(N, device, gpu, hide)                               \
    DEFINE_OUTPUT_WRITE_VARIANT(ShapeWriteBase, ShapeWrite, writeonly_shape_rb, N, device, gpu, hide)

#define DEFINE_ARTI_ROOT_WRITE(N, device, gpu, hide)                           \
    DEFINE_OUTPUT_WRITE_VARIANT(ArticulationRootWriteBase, ArticulationRootWrite, writeonly_arti_root, N, device, gpu, hide)

#define DEFINE_ARTI_ROOT_BINDING_WRITE(N, device, gpu, hide)                   \
    DEFINE_OUTPUT_WRITE_VARIANT(ArticulationRootBindingWriteBase, ArticulationRootBindingWrite, tensorbinding_arti_root, N, device, gpu, hide)

// The DOF PROPERTY write (ADR-0012), against the DOF STATE lane beside it.
//
// Same scene and same object type as arti_dof, differing only in the attribute. That is the
// comparison worth having: state goes to a device scratch through PhysX's DirectGPU write types,
// while a property is a host loop over per-DOF setters. The pair isolates that difference with
// everything else held equal.
//
// CPU-registered only, for the reason PropertyWriteBase gives: a property has no device
// destination, so a GPU lane would time the identical host loop.
class ArticulationDofPropertyWriteBase : public ArticulationDofWriteBase
{
public:
    ArticulationDofPropertyWriteBase(uint32_t envCount, bool gpu) : ArticulationDofWriteBase(envCount, gpu) {}

protected:
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0,
                                      { OVPHYSX_ATTR_JOINT_STIFFNESS, sizeof(OVPHYSX_ATTR_JOINT_STIFFNESS) - 1 } };
    }
};


// The two authored-asset families. The asset holds every env already, so these do not clone. See
// clonesEnvs() on the base. One asset per size, because the env count is baked in.
template <typename WriteBase>
class AuthoredEnvWriteSceneT : public WriteBase
{
public:
    AuthoredEnvWriteSceneT(uint32_t envCount, bool gpu, const char* assetPrefix)
        : WriteBase(envCount, gpu),
          mAsset(std::string("/../benchmarks/data/") + assetPrefix + "_envs_" + std::to_string(envCount) +
                 ".usda")
    {
    }

protected:
    const char* sceneAsset() const override { return mAsset.c_str(); }
    bool clonesEnvs() const override { return false; }

private:
    std::string mAsset;
};


// Deformable MATERIAL properties (ADR-0012): one f32 per material prim, host-resident.
//
// GPU-registered despite being a host column, which is the exception to PropertyWriteBase's
// CPU-only rule rather than an oversight: deformables do not exist on a CPU scene at all, so there
// is no host scene to register against. What it measures is the host loop plus whatever the session
// costs on a DirectGPU scene, which is the only mode this attribute has.
class DeformableMaterialWriteBase : public AuthoredEnvWriteSceneT<WriteOnlyBase>
{
public:
    DeformableMaterialWriteBase(uint32_t envCount, bool gpu)
        : AuthoredEnvWriteSceneT<WriteOnlyBase>(envCount, gpu, "deformables")
    {
    }

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_DEFORMABLE_MATERIAL; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{
            0, { OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS, sizeof(OVPHYSX_ATTR_DEFORMABLE_YOUNGS_MODULUS) - 1 }
        };
    }
};


// Deformable SIM-MESH state (ADR-0012): the per-vertex point column, device-resident.
//
// The heaviest write in the file by element count. A group covers one body's whole sim mesh, so
// the payload scales with mesh resolution rather than with env count the way every other lane does.
// It is also the only lane whose scatter reframes on the device, which is what makes its cost worth
// separating from the material lane above.
class DeformableStateWriteBase : public AuthoredEnvWriteSceneT<WriteOnlyBase>
{
public:
    DeformableStateWriteBase(uint32_t envCount, bool gpu)
        : AuthoredEnvWriteSceneT<WriteOnlyBase>(envCount, gpu, "deformables")
    {
    }

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_DEFORMABLE_VOLUME; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_POINTS, sizeof(OVPHYSX_ATTR_POINTS) - 1 } };
    }
};


// Particle set points (ADR-0012): per-particle, and HOST-resident even here.
//
// The contrast with the deformable lane above is the point of measuring both: same array shape, same
// per-element payload, but this one lands in the set's pinned staging array for PhysX to upload at
// the next step, where the deformable one scatters straight into PhysX's device buffer. If the
// staging route costs materially more, this pair is where it shows.
class ParticleWriteBase : public AuthoredEnvWriteSceneT<WriteOnlyBase>
{
public:
    ParticleWriteBase(uint32_t envCount, bool gpu)
        : AuthoredEnvWriteSceneT<WriteOnlyBase>(envCount, gpu, "particles")
    {
    }

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_PARTICLE_SET; }
    ovx_string_or_token_t writeAttr() const override
    {
        return ovx_string_or_token_t{ 0, { OVPHYSX_ATTR_POINTS, sizeof(OVPHYSX_ATTR_POINTS) - 1 } };
    }
};


#define DEFINE_ARTI_DOF_WRITE(N, device, gpu, hide)                            \
    DEFINE_OUTPUT_WRITE_VARIANT(ArticulationDofWriteBase, ArticulationDofWrite, writeonly_arti_dof, N, device, gpu, hide)

#define DEFINE_ARTI_DOF_PROPERTY_WRITE(N, device, gpu, hide)                   \
    DEFINE_OUTPUT_WRITE_VARIANT(ArticulationDofPropertyWriteBase, ArticulationDofPropertyWrite, writeonly_arti_dof_prop, N, \
                                device, gpu, hide)

#define DEFINE_DEFORMABLE_MATERIAL_WRITE(N, device, gpu, hide)                 \
    DEFINE_OUTPUT_WRITE_VARIANT(DeformableMaterialWriteBase, DeformableMaterialWrite, writeonly_deformable_mat, N,   \
                                device, gpu, hide)

#define DEFINE_DEFORMABLE_STATE_WRITE(N, device, gpu, hide)                    \
    DEFINE_OUTPUT_WRITE_VARIANT(DeformableStateWriteBase, DeformableStateWrite, writeonly_deformable_points, N, \
                                device, gpu, hide)

#define DEFINE_PARTICLE_WRITE(N, device, gpu, hide)                            \
    DEFINE_OUTPUT_WRITE_VARIANT(ParticleWriteBase, ParticleWrite, writeonly_particle_points, N, device, gpu, hide)

#define DEFINE_ARTI_DOF_BINDING_WRITE(N, device, gpu, hide)                    \
    DEFINE_OUTPUT_WRITE_VARIANT(ArticulationDofBindingWriteBase, ArticulationDofBindingWrite, tensorbinding_arti_dof, N, device, gpu, hide)

#define DEFINE_FIXED_TENDON_WRITE(N, device, gpu, hide)                        \
    DEFINE_OUTPUT_WRITE_VARIANT(FixedTendonWriteBase, FixedTendonWrite, writeonly_tendon_fixed, N, device, gpu, hide)

#define DEFINE_SPATIAL_TENDON_WRITE(N, device, gpu, hide)                      \
    DEFINE_OUTPUT_WRITE_VARIANT(SpatialTendonWriteBase, SpatialTendonWrite, writeonly_tendon_spatial, N, device, gpu, hide)

#define DEFINE_FIXED_TENDON_BINDING_WRITE(N, device, gpu, hide)                \
    DEFINE_OUTPUT_WRITE_VARIANT(FixedTendonBindingWriteBase, FixedTendonBindingWrite, tensorbinding_tendon_fixed, N, device, gpu, hide)

#define DEFINE_SPATIAL_TENDON_BINDING_WRITE(N, device, gpu, hide)              \
    DEFINE_OUTPUT_WRITE_VARIANT(SpatialTendonBindingWriteBase, SpatialTendonBindingWrite, tensorbinding_tendon_spatial, N, device, gpu, hide)

#define DEFINE_FORCE_WRITE(N, device, gpu, hide)                               \
    DEFINE_OUTPUT_WRITE_VARIANT(ForceWriteBase, ForceWrite, writeonly_force_rb, N, device, gpu, hide)

#define DEFINE_WRENCH_WRITE(N, device, gpu, hide)                              \
    DEFINE_OUTPUT_WRITE_VARIANT(WrenchWriteBase, WrenchWrite, writeonly_wrench_rb, N, device, gpu, hide)

#define DEFINE_LINK_WRENCH_WRITE(N, device, gpu, hide)                         \
    DEFINE_OUTPUT_WRITE_VARIANT(LinkWrenchWriteBase, LinkWrenchWrite, writeonly_wrench_link, N, device, gpu, hide)

#define DEFINE_FORCE_BINDING_WRITE(N, device, gpu, hide)                       \
    DEFINE_OUTPUT_WRITE_VARIANT(ForceBindingWriteBase, ForceBindingWrite, tensorbinding_force_rb, N, device, gpu, hide)

#define DEFINE_WRENCH_BINDING_WRITE(N, device, gpu, hide)                      \
    DEFINE_OUTPUT_WRITE_VARIANT(WrenchBindingWriteBase, WrenchBindingWrite, tensorbinding_wrench_rb, N, device, gpu, hide)

#define DEFINE_LINK_WRENCH_BINDING_WRITE(N, device, gpu, hide)                 \
    DEFINE_OUTPUT_WRITE_VARIANT(LinkWrenchBindingWriteBase, LinkWrenchBindingWrite, tensorbinding_wrench_link, N, device, gpu, hide)

#define DEFINE_INSTANCER_WRITE(N, device, gpu, hide)                           \
    DEFINE_OUTPUT_WRITE_VARIANT(InstancerWriteBase, InstancerWrite, writeonly_rb_instancer, N, device, gpu, hide)

#define DEFINE_INSTANCER_VEL_WRITE(N, device, gpu, hide)                       \
    DEFINE_OUTPUT_WRITE_VARIANT(InstancerVelocityWriteBase, InstancerVelocityWrite, writeonly_rb_instancer_vel, N, device, gpu, hide)

DEFINE_QUERY_WRITE(1024, cpu, false, false)
DEFINE_QUERY_WRITE(8192, cpu, false, false)
DEFINE_QUERY_WRITE(1024, gpu, true, true)
DEFINE_QUERY_WRITE(8192, gpu, true, true)

// The lane to read next to the binding one: a held query, so neither side is charged for
// re-resolving a selector the other resolves once.
DEFINE_WRITE_ONLY(8192, cpu, false, false)
DEFINE_WRITE_ONLY(8192, gpu, true, true)

DEFINE_TENSOR_BINDING_WRITE(8192, cpu, false, false)
DEFINE_TENSOR_BINDING_WRITE(8192, gpu, true, true)

// CPU only, by the reasoning on PropertyWriteBase: no device destination exists, so a
// GPU lane would measure the same host loop under a name that says otherwise.
DEFINE_PROPERTY_WRITE(1024, cpu, false, false)
DEFINE_PROPERTY_WRITE(8192, cpu, false, false)
DEFINE_PROPERTY_BINDING_WRITE(1024, cpu, false, false)
DEFINE_PROPERTY_BINDING_WRITE(8192, cpu, false, false)

DEFINE_SHAPE_WRITE(1024, cpu, false, false)
DEFINE_SHAPE_WRITE(8192, cpu, false, false)

// Both devices, unlike the property lanes above: root state has a device destination.
DEFINE_ARTI_ROOT_WRITE(1024, cpu, false, false)
DEFINE_ARTI_ROOT_WRITE(8192, cpu, false, false)
DEFINE_ARTI_ROOT_WRITE(1024, gpu, true, true)
DEFINE_ARTI_ROOT_WRITE(8192, gpu, true, true)

DEFINE_ARTI_ROOT_BINDING_WRITE(8192, cpu, false, false)
DEFINE_ARTI_ROOT_BINDING_WRITE(8192, gpu, true, true)

DEFINE_ARTI_DOF_WRITE(1024, cpu, false, false)
DEFINE_ARTI_DOF_WRITE(8192, cpu, false, false)
DEFINE_ARTI_DOF_WRITE(1024, gpu, true, true)
DEFINE_ARTI_DOF_WRITE(8192, gpu, true, true)

DEFINE_ARTI_DOF_PROPERTY_WRITE(1024, cpu, false, false)
DEFINE_ARTI_DOF_PROPERTY_WRITE(8192, cpu, false, false)

DEFINE_ARTI_DOF_BINDING_WRITE(8192, cpu, false, false)
DEFINE_ARTI_DOF_BINDING_WRITE(8192, gpu, true, true)

// Tendon lanes, facing OutputRead's on the same asset.
DEFINE_FIXED_TENDON_WRITE(1024, cpu, false, false)
DEFINE_FIXED_TENDON_WRITE(8192, cpu, false, false)
DEFINE_FIXED_TENDON_WRITE(1024, gpu, true, true)
DEFINE_FIXED_TENDON_WRITE(8192, gpu, true, true)

DEFINE_SPATIAL_TENDON_WRITE(8192, cpu, false, false)
DEFINE_SPATIAL_TENDON_WRITE(8192, gpu, true, true)

DEFINE_FIXED_TENDON_BINDING_WRITE(8192, cpu, false, false)
DEFINE_FIXED_TENDON_BINDING_WRITE(8192, gpu, true, true)
DEFINE_SPATIAL_TENDON_BINDING_WRITE(8192, cpu, false, false)
DEFINE_SPATIAL_TENDON_BINDING_WRITE(8192, gpu, true, true)

// The write-only pair, plus the DirectGPU link route, which is a separate code path.
DEFINE_FORCE_WRITE(1024, cpu, false, false)
DEFINE_FORCE_WRITE(8192, cpu, false, false)
DEFINE_FORCE_WRITE(1024, gpu, true, true)
DEFINE_FORCE_WRITE(8192, gpu, true, true)

DEFINE_WRENCH_WRITE(8192, cpu, false, false)
DEFINE_WRENCH_WRITE(8192, gpu, true, true)

DEFINE_LINK_WRENCH_WRITE(8192, cpu, false, false)
DEFINE_LINK_WRENCH_WRITE(8192, gpu, true, true)

DEFINE_FORCE_BINDING_WRITE(8192, cpu, false, false)
DEFINE_FORCE_BINDING_WRITE(8192, gpu, true, true)
DEFINE_WRENCH_BINDING_WRITE(8192, cpu, false, false)
DEFINE_WRENCH_BINDING_WRITE(8192, gpu, true, true)
DEFINE_LINK_WRENCH_BINDING_WRITE(8192, cpu, false, false)
DEFINE_LINK_WRENCH_BINDING_WRITE(8192, gpu, true, true)

// Instancer lanes, facing OutputRead's readonly_rb_instancer lanes on the same asset.
DEFINE_INSTANCER_WRITE(1024, cpu, false, false)
DEFINE_INSTANCER_WRITE(8192, cpu, false, false)
DEFINE_INSTANCER_WRITE(1024, gpu, true, true)
DEFINE_INSTANCER_WRITE(8192, gpu, true, true)

DEFINE_INSTANCER_VEL_WRITE(8192, cpu, false, false)
DEFINE_INSTANCER_VEL_WRITE(8192, gpu, true, true)

// GPU only, and at the sizes the authored assets exist at: deformables and particles do not exist
// on a CPU scene, so there is no host lane to register. Hidden, matching the read's own lanes for
// these families.
DEFINE_DEFORMABLE_MATERIAL_WRITE(128, gpu, true, true)
DEFINE_DEFORMABLE_MATERIAL_WRITE(512, gpu, true, true)
DEFINE_DEFORMABLE_STATE_WRITE(128, gpu, true, true)
DEFINE_DEFORMABLE_STATE_WRITE(512, gpu, true, true)
// 128 only. The 512-set asset aborts on a 16 GB card (one PxPBDParticleSystem per env, and their
// grid buffers dominate), which is why the read's own ragged/flat particle lanes stop at 128 too.
DEFINE_PARTICLE_WRITE(128, gpu, true, true)

#undef DEFINE_PARTICLE_WRITE
#undef DEFINE_DEFORMABLE_STATE_WRITE
#undef DEFINE_DEFORMABLE_MATERIAL_WRITE
#undef DEFINE_ARTI_DOF_PROPERTY_WRITE
#undef DEFINE_INSTANCER_VEL_WRITE
#undef DEFINE_INSTANCER_WRITE
#undef DEFINE_LINK_WRENCH_BINDING_WRITE
#undef DEFINE_WRENCH_BINDING_WRITE
#undef DEFINE_FORCE_BINDING_WRITE
#undef DEFINE_LINK_WRENCH_WRITE
#undef DEFINE_WRENCH_WRITE
#undef DEFINE_FORCE_WRITE
#undef DEFINE_SPATIAL_TENDON_BINDING_WRITE
#undef DEFINE_FIXED_TENDON_BINDING_WRITE
#undef DEFINE_SPATIAL_TENDON_WRITE
#undef DEFINE_FIXED_TENDON_WRITE
#undef DEFINE_ARTI_DOF_BINDING_WRITE
#undef DEFINE_ARTI_DOF_WRITE
#undef DEFINE_ARTI_ROOT_BINDING_WRITE
#undef DEFINE_ARTI_ROOT_WRITE
#undef DEFINE_SHAPE_WRITE
#undef DEFINE_PROPERTY_BINDING_WRITE
#undef DEFINE_PROPERTY_WRITE
#undef DEFINE_TENSOR_BINDING_WRITE
#undef DEFINE_WRITE_ONLY
#undef DEFINE_QUERY_WRITE
#undef DEFINE_OUTPUT_WRITE_VARIANT

} // namespace
