// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// DEPRECATED (tensor-binding-deprecation): the binding comparison lanes retire with the binding. Session lanes stay.

// Physics-output read (ADR-0007 / ADR-0008) at scale, against the equivalent tensor binding.
//
// Names are OutputRead.<operation>_<objects>_<envs>_<device>.
//
//   operation  queryread     -> query + read + drain: what a stateless caller pays per frame.
//              readonly      -> read + drain, query opened once and held across steps.
//              tensorbinding -> read into preallocated tensors, view already built.
//              tensorcreate  -> view creation: create + spec + destroy.
//
//   objects    rb            -> rigid bodies, cubes20 scene (~164k bodies at 8192 envs). Reads
//                               position, orientation and linear/angular velocity, 13 floats
//                               per body, matching the binding's pose [N, 7] + velocity [N, 6].
//              arti_link     -> articulation links, cartpole scene, the same four attributes as
//                               the rigid lane so the rows are comparable.
//              arti_dof      -> articulation joints, jointPosition/jointVelocity: the per-axis
//                               array path, which has no rigid equivalent.
//              arti_root     -> whole articulations, cartpole scene: ONE row per articulation
//                               rather than one per link, reading the same four attributes.
//              arti_inverse_dynamics -> the whole-articulation INVERSE DYNAMICS columns, jacobian and mass matrix,
//                               the two the binding can face on equal payload. Their width comes
//                               from each articulation's topology, so the read emits one group per
//                               COHORT of identical articulations rather than one per attribute.
//                               A homogeneous fleet is one cohort, which is the case that matters.
//              arti_inverse_dynamics_mixed
//                            -> the same columns on articulation_pileup, sixteen articulations of
//                               increasing link count, so every row is its own cohort. An upper
//                               bound on per-cohort overhead rather than a workload: real scenes
//                               hold a handful of topologies. Read against arti_inverse_dynamics, it says
//                               what fragmenting a read into groups costs.
//              vehicle_wheel -> vehicle wheels, vehicle_probe scene, position and orientation.
//                               Host-only and CPU-registered: a vehicle cannot be attached to a
//                               DirectGPU scene at all, so there is no _gpu counterpart to register
//                               and no tensorbinding lane to face, since the tensor API has no
//                               vehicle view.
//              particles     -> particle sets, particles scene, points/velocities. GPU-registered
//                               only: a set cannot exist without a CUDA context. No tensorbinding
//                               lane either, since the binding serves no particle tensors.
//              deformable_vol -> volume deformable bodies, deformables scene, points/velocities.
//                               GPU-registered only, the mirror of the vehicle case: the read
//                               refuses to serve a deformable without a CUDA context and the
//                               binding refuses the tensor outside DirectGPU, so a _cpu lane
//                               would report nothing on either side.
//              deformable_surf-> surface deformable bodies, same scene and same columns.
//
// So readonly_rb_8192_cpu and tensorbinding_rb_8192_cpu face each other, as do
// readonly_arti_link_8192_cpu and tensorbinding_arti_link_8192_cpu.
//
// Compare a tensorbinding lane against the readonly one, not queryread: a binding resolves its
// selector once at bind time, so putting it next to a lane that re-opens a query every frame
// charges the read for work the binding never does.
//
// The vehicle lane is the one that does NOT clone: ovphysx_clone switches on PxConcreteType and has
// no vehicle case, so cloning a car yields one real vehicle and N-1 partial ones. Its asset authors
// all 1024 up front instead (gen_vehicle_probe.py), so it overrides clonesEnvs() to false and
// checks its own row count in startRun: with no clone step, nothing else ties mEnvCount to the
// scene the name claims.
//
// A read that does not complete returns early and therefore times FAST. Every such lane routes
// through bmRecordFailure(), so the harness discards its sample and the process exits non-zero
// rather than publishing a number no work produced.
//
// The _cpu variants run on the default pass. The _gpu ones require DirectGPU and are hidden from
// wildcard runs, like Probe.cartpole_4096_control_step: without suppressReadback the scene never
// raises PxSceneFlag::eENABLE_DIRECT_GPU_API and both APIs fall back to their host paths. Run them
// in a dedicated process (DirectGPU changes scene behavior for other GPU benchmarks):
//
//   ovphysx_benchmarks --forceGpu --directGpu --hidden --filter=OutputRead.*_gpu

#include "framework/UsdPCH.h"

#include "../BenchmarkFailure.h"
#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../OvstageLoad.h"

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
#include <cuda_runtime_api.h>
#endif

#include "ovphysx/dlpack/dlpack.h"
#include "ovphysx/experimental/TensorBinding.hpp"
#include <ovphysx/experimental/ovphysx.hpp>

#include <algorithm> // std::min / std::max for the per-group tensor extents
#include <cmath>
#include <stdexcept>
#include <string>
#include <deque>
#include <vector>


void initOutputRead()
{
}

namespace
{

// Every prim set the two APIs read: env1..envN plus the template they were cloned from, which
// OVPHYSX_SCOPE_ALL returns and a pattern rooted at env* would miss.
const char* const kBodyPattern = "/World/envs/*/*";

// The cartpole asset puts one articulation per env at env_*/Robot. Same pattern LabCartpole binds
// through, so the articulation lanes select the same set that suite already exercises.
const char* const kArticulationPattern = "/World/envs/env_*/Robot";

// The joint-state attribute set, shared by the read lane and the binding lane facing it so both
// describe the same columns. Static storage: handed to ovphysx_read by pointer, so it outlives
// the call.
const ovx_string_or_token_t* jointReadAttrs(size_t& count)
{
    static const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_JOINT_POSITION, sizeof(OVPHYSX_ATTR_JOINT_POSITION) - 1 } },
        { 0, { OVPHYSX_ATTR_JOINT_VELOCITY, sizeof(OVPHYSX_ATTR_JOINT_VELOCITY) - 1 } },
    };
    count = sizeof(attrs) / sizeof(attrs[0]);
    return attrs;
}

// The vehicle-wheel attribute set: the two the read serves for kOvxVehicleWheel.
const ovx_string_or_token_t* vehicleReadAttrs(size_t& count)
{
    static const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
        { 0, { OVPHYSX_ATTR_ORIENTATION, sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
    };
    count = sizeof(attrs) / sizeof(attrs[0]);
    return attrs;
}

// The deformable attribute set: `points` and `velocities`, which are also the two the binding serves
// as SIM_NODAL_POSITION / SIM_NODAL_VELOCITY, so neither side has to pick a subset of the other.
//
// Keep it to these two: the lane is the baseline both APIs are compared on. The body's host-only
// columns (`restPoints`, `simElementIndices`, `collisionElementIndices`) have no binding counterpart
// to face, and the material ones belong to their own object type (REQ-READ-MATERIAL-001).
const ovx_string_or_token_t* deformableReadAttrs(size_t& count)
{
    static const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_POINTS, sizeof(OVPHYSX_ATTR_POINTS) - 1 } },
        { 0, { OVPHYSX_ATTR_VELOCITIES, sizeof(OVPHYSX_ATTR_VELOCITIES) - 1 } },
    };
    count = sizeof(attrs) / sizeof(attrs[0]);
    return attrs;
}

// The particle attribute set: `points` and `velocities`, the only two the read serves for
// kOvxParticleSet. The binding serves no particle tensors, so this family has no partner lane to
// face.
const ovx_string_or_token_t* particleReadAttrs(size_t& count)
{
    static const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_POINTS, sizeof(OVPHYSX_ATTR_POINTS) - 1 } },
        { 0, { OVPHYSX_ATTR_VELOCITIES, sizeof(OVPHYSX_ATTR_VELOCITIES) - 1 } },
    };
    count = sizeof(attrs) / sizeof(attrs[0]);
    return attrs;
}

// The whole-articulation ROOT set. The same four quantities as the default rigid set, but the
// tokens are this type's own: its row is keyed by the articulation-root API prim, which need not be
// the root link, so the bare names would mean something else here (REQ-READ-ATTRS-001 AC-11).
// Both arti_root lanes need it, since the binding control reads it during warm-up.
const ovx_string_or_token_t* articulationRootReadAttrs(size_t& count)
{
    static const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_ROOT_POSITION, sizeof(OVPHYSX_ATTR_ROOT_POSITION) - 1 } },
        { 0, { OVPHYSX_ATTR_ROOT_ORIENTATION, sizeof(OVPHYSX_ATTR_ROOT_ORIENTATION) - 1 } },
        { 0, { OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY, sizeof(OVPHYSX_ATTR_ROOT_LINEAR_VELOCITY) - 1 } },
        { 0, { OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY, sizeof(OVPHYSX_ATTR_ROOT_ANGULAR_VELOCITY) - 1 } },
    };
    count = sizeof(attrs) / sizeof(attrs[0]);
    return attrs;
}

// The inverse dynamics set: the two columns whose width comes from the articulation's topology rather than
// from the attribute, so each is emitted one group per cohort of identical articulations. That is
// what these lanes are for: the gather is ordinary, the partitioning is not.
//
// Two rather than four, to face the binding on equal payload the way arti_root does. TensorBindings
// binds two tensors per lane, and these are the two that matter: on cartpole the jacobian is 24 of
// the 32 floats a row carries and the mass matrix another 4, so coriolis and gravity are 4 floats
// between them. They are covered by the tests, not by a lane whose only job is the comparison.
const ovx_string_or_token_t* articulationInverseDynamicsReadAttrs(size_t& count)
{
    static const ovx_string_or_token_t attrs[] = {
        { 0, { OVPHYSX_ATTR_JACOBIAN, sizeof(OVPHYSX_ATTR_JACOBIAN) - 1 } },
        { 0, { OVPHYSX_ATTR_MASS_MATRIX, sizeof(OVPHYSX_ATTR_MASS_MATRIX) - 1 } },
    };
    count = sizeof(attrs) / sizeof(attrs[0]);
    return attrs;
}

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
// A device-resident read records the session's completion event and hands it to the consumer rather
// than blocking on it (ADR-0008), so this wait is part of what a caller pays. Every group in one
// session carries the same completion event. Repeated waits are valid and are part of the public
// group-consumption pattern this lane measures.
bool waitForGroup(const ovstage_read_group_t& group)
{
    if (group.data.cuda_sync.wait_event != 0)
    {
        return cudaEventSynchronize(reinterpret_cast<cudaEvent_t>(group.data.cuda_sync.wait_event)) == cudaSuccess;
    }
    if (group.data.cuda_sync.stream != 0)
    {
        // 1 means the default stream, anything higher is a real cudaStream_t.
        const cudaStream_t stream = group.data.cuda_sync.stream == 1 ?
                                        cudaStream_t{} :
                                        reinterpret_cast<cudaStream_t>(group.data.cuda_sync.stream);
        return cudaStreamSynchronize(stream) == cudaSuccess;
    }
    return true;
}
#else
bool waitForGroup(const ovstage_read_group_t&)
{
    return true;
}
#endif


class OutputReadBase : public BmBenchmark
{
public:
    OutputReadBase(uint32_t envCount, bool gpu) : mEnvCount(envCount), mGpu(gpu) {}

    bool isValid() const override
    {
        const BmGlobals& globals = BmGlobals::getInstance();
        if (!mGpu)
        {
            if (globals.forceGpu()) return false;
            return globals.getPhysX() != nullptr;
        }
        // Both APIs branch on PxSceneFlag::eENABLE_DIRECT_GPU_API, which is raised only when
        // suppressReadback is set, so --forceGpu alone would measure their host paths.
        if (!globals.forceGpu())
        {
            throw std::runtime_error("OutputRead GPU variants require --forceGpu");
        }
        if (!globals.directGpu())
        {
            throw std::runtime_error("OutputRead GPU variants require --directGpu");
        }
        if (globals.getPhysX() == nullptr)
        {
            throw std::runtime_error("OutputRead failed to initialize OVPhysX");
        }
        return true;
    }

    static constexpr int kWarmUpReads = 3;

    // A warmed read converges in few samples, so more steps only lengthen the pass.
    uint32_t getNbSteps() const override { return mEnvCount >= 8192 ? 20 : 50; }
    uint32_t getNbRuns() const override { return 5; }

    // Replicate the scene's template into mEnvCount copies laid out on a grid. False if the clone
    // failed, in which case it has already reported why.
    //
    // A lane whose objects cannot be cloned authors them all up front instead and overrides
    // clonesEnvs() to false. This is then skipped entirely. Guarded rather than assumed, because
    // cloning something the replicator does not handle SUCCEEDS and silently produces one real
    // object and N-1 partial ones.
    bool cloneEnvs()
    {
        if (!clonesEnvs())
            return true;

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
            printFormatted("OutputRead: clone(N=%u) failed", mEnvCount);
            return false;
        }
        mPhysX->waitAll();
        return true;
    }

    void startRun() override
    {
        mSetupOk = false;
        // Re-armed per run, not once per process: the payload gates (including the non-empty
        // shape check) only run while this is true, so leaving it false after run 1 would let
        // runs 2+ warm up on an empty read and snapshot the expectation as zero.
        mCountPayload = true;
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!mPhysX) return;

        const std::string path = BmGlobals::getInstance().getDataFolder() + sceneAsset();
        if (!benchmarkLoadUsdWithOvstage(mPhysX, path, mStageAttachment))
        {
            printFormatted("OutputRead: ovstage load failed (%s)", path.c_str());
            return;
        }

        if (!cloneEnvs())
            return;

        // Warmup required by DirectGPU
        mPhysX->step(1.0f / 60.0f);
        mPhysX->waitAll();

        // The first reads against a fresh scene are far slower than a warm one, so keep them out of
        // the timed set. readOnce(), not timedRead(): a subclass binds in its own startRun, not yet
        // run.
        //
        // EVERY lane warms here, binding controls included. They do not measure the output read, but
        // they do share its scene, so warming on one side only would put the two APIs into their
        // timed regions with different cache and first-touch state.
        for (int warm = 0; warm < kWarmUpReads; ++warm)
        {
            if (!readOnce())
            {
                printFormatted("OutputRead: warm-up read failed -- run will report no timings");
                return;
            }
        }

        // Both APIs have to cover the same bodies for a comparison to mean anything, so report
        // the count and payload each one resolved.
        if (!mReportedShape)
        {
            mReportedShape = true;
            printFormatted("OutputRead: envs=%u groups=%u rows=%u floats=%llu", mEnvCount, mLastGroupCount,
                           mLastRowCount, static_cast<unsigned long long>(mLastFloatCount));
        }
        // Warm-up is over. From here readOnce() is the timed operation and carries no accounting.
        mExpectedGroupCount = mLastGroupCount;
        mExpectedRowCount = mLastRowCount;
        mExpectedArrayTensorsMin = mLastArrayTensorsMin;
        mExpectedArrayTensorsMax = mLastArrayTensorsMax;
        mCountPayload = false;
        mSetupOk = true;
    }

    void endRun() override
    {
        finishRun();
        teardown();
    }

    // Advance the simulation between timed reads, so each read runs against state that actually
    // changed, which is what a caller does. preStep() sits outside timedStep(), so the step is not
    // in the number. What it changes is what the read has to cope with.
    //
    // Without it the whole timed loop reads one frozen step. Every cache then hits forever, nothing
    // is ever invalidated, and refreshRdGpuIndices() early-outs after the first read, so the
    // measurement is the best case of the caching rather than its steady state. That flatters the
    // ovstage read specifically, since it is the side with caches to hit.
    void preStep() override
    {
        if (!mSetupOk || !mPhysX)
            return;
        mPhysX->step(1.0f / 60.0f);
        mPhysX->waitAll();
    }

protected:
    // Split from endRun so a subclass reporting under its own name can reuse it.
    void teardown()
    {
        if (!mPhysX) return;
        benchmarkClearOvstage(mPhysX, mStageAttachment);
    }

    void step() override
    {
        // Counted here rather than taken from getNbSteps(): --steps=N overrides that, so using it
        // as the denominator would report a total failure under --steps=10 as "10 of 50".
        ++mAttempts;
        if (!mSetupOk)
        {
            ++mFailedReads;
            return;
        }
        if (!timedRead())
            ++mFailedReads;
    }

    // Both gates a lane can fail, run after the timing and before teardown. Subclasses that
    // override endRun() call this rather than either half, so neither can be forgotten.
    void finishRun()
    {
        checkTimedShape();
        reportFailures();
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

    // A failed read returns early, so it times as near-zero and makes the run look FASTER. The
    // harness cannot tell that from a genuinely quick read, so route it through the failure channel:
    // the row's sample is discarded and the process exits non-zero.
    void reportFailures()
    {
        const uint32_t failed = mFailedReads;
        const uint32_t attempts = mAttempts;
        mFailedReads = 0;
        mAttempts = 0;
        if (failed == 0 || !recordRowFailure())
            return;
        bmRecordFailure(registeredRow().c_str(),
                        "%u of %u reads failed -- the reported time is not a measurement", failed, attempts);
    }

    // The payload gates run during warm-up only, to keep the branch out of the measurement. A lane
    // that stopped resolving anything partway through the run would otherwise drain clean and time
    // fast, so the last timed read's shape is compared against the warmed one here, once the timing
    // is over. Lanes whose timedRead() is not an output read never set the flag and are skipped.
    //
    // This assumes a lane's shape is fixed across a run. One whose row count legitimately varies
    // (a scope filter that returns only what moved, say) would have to opt out of it.
    void checkTimedShape()
    {
        if (!mTimedShapeSeen)
            return;
        mTimedShapeSeen = false;
        // The array-group extents are part of the shape, not extra: group and row counts both stay
        // right while one attribute quietly stops covering every prim, which is the truncation
        // setup rejects. Extremes across EVERY timed read, not the last one's shape: a run that
        // dipped in the middle and recovered by the end would otherwise drain clean, time fast and
        // publish a ratio.
        const bool groupsSteady = mTimedGroupMin == mExpectedGroupCount && mTimedGroupMax == mExpectedGroupCount;
        const bool rowsSteady = mTimedRowMin == mExpectedRowCount && mTimedRowMax == mExpectedRowCount;
        if (groupsSteady && rowsSteady && mLastArrayTensorsMin == mExpectedArrayTensorsMin &&
            mLastArrayTensorsMax == mExpectedArrayTensorsMax)
            return;
        if (!recordRowFailure())
            return;
        bmRecordFailure(registeredRow().c_str(),
                        "read resolved groups=%u rows=%u tensors=%u..%u after warming at "
                        "groups=%u rows=%u tensors=%u..%u",
                        mLastGroupCount, mLastRowCount, mLastArrayTensorsMin, mLastArrayTensorsMax,
                        mExpectedGroupCount, mExpectedRowCount, mExpectedArrayTensorsMin,
                        mExpectedArrayTensorsMax);
    }

    // The registered name minus the size/device suffix, supplied by the registration macro so it
    // cannot drift from what --list prints. The default only shows if a class is registered by
    // hand.
    virtual const char* variantName() const { return "OutputRead"; }

    // Whether mEnvCount is reached by cloning cloneSource(). False for a lane whose asset already
    // contains every object, which is the only option when the object type cannot be cloned.
    virtual bool clonesEnvs() const { return true; }

    // The measured operation. Overridden to compare a different API over the SAME scene.
    virtual bool timedRead() { return readOnce(); }

    // What the scene is and what gets read out of it. Defaults to the rigid-body workload. The
    // families below override these to point the same harness at another scene and object type.
    virtual const char* sceneAsset() const
    {
        return mGpu ? "/../benchmarks/data/cubes20_envs_gpu.usda" : "/../benchmarks/data/cubes20_envs.usda";
    }
    virtual const char* cloneSource() const { return "/World/envs/template"; }
    virtual std::string cloneTarget(uint32_t i) const { return "/World/envs/env" + std::to_string(i + 1); }
    virtual float cloneSpacing() const { return 4.0f; }
    virtual ovphysx_sim_object_type_t objectType() const { return OVPHYSX_OBJECT_RIGID_BODY; }
    virtual ovphysx_object_scope_t queryScope() const { return OVPHYSX_SCOPE_ALL; }

    // The attribute set the read requests. Static storage per override: the array is handed to
    // ovphysx_read by pointer and must outlive the call.
    virtual const ovx_string_or_token_t* readAttrs(size_t& count) const
    {
        static const ovx_string_or_token_t attrs[] = {
            { 0, { OVPHYSX_ATTR_POSITION, sizeof(OVPHYSX_ATTR_POSITION) - 1 } },
            { 0, { OVPHYSX_ATTR_ORIENTATION, sizeof(OVPHYSX_ATTR_ORIENTATION) - 1 } },
            { 0, { OVPHYSX_ATTR_LINEAR_VELOCITY, sizeof(OVPHYSX_ATTR_LINEAR_VELOCITY) - 1 } },
            { 0, { OVPHYSX_ATTR_ANGULAR_VELOCITY, sizeof(OVPHYSX_ATTR_ANGULAR_VELOCITY) - 1 } },
        };
        count = sizeof(attrs) / sizeof(attrs[0]);
        return attrs;
    }

protected:
    bool openQuery(ovphysx_query_handle_t& query)
    {
        query = 0;
        if (ovphysx_query(physx()->handle(), objectType(), queryScope(), &query).status !=
                OVPHYSX_API_SUCCESS ||
            query == 0)
        {
            printFormatted("OutputRead: ovphysx_query failed");
            return false;
        }
        return true;
    }

    // Read session over an existing query: read -> drain -> release the session, not the query.
    bool readWithQuery(ovphysx_query_handle_t query)
    {
        size_t attrCount = 0;
        const ovx_string_or_token_t* attrs = readAttrs(attrCount);

        ovphysx_read_handle_t read = 0;
        if (ovphysx_read(physx()->handle(), query, attrs, attrCount, &read).status != OVPHYSX_API_SUCCESS ||
            read == 0)
        {
            printFormatted("OutputRead: ovphysx_read failed");
            return false;
        }

        // Drain every group: a caller pays for the iteration too. END_OF_ITERATION is the only
        // non-error way out (ovphysx_types.h). Treating any non-SUCCESS as "done" would let a
        // partly-drained read time as a complete one.
        const ovstage_read_group_t* group = nullptr;
        uint32_t groups = 0;
        int64_t rows = 0;
        // Per ARRAY group, not just the first. A lane whose `points` group drains every prim while
        // `velocities` drains fewer is a TRUNCATED read, which recording one group's count would
        // let through setup. Min and max together say both "how many" and "did every group agree".
        uint32_t arrayTensorsMin = UINT32_MAX;
        uint32_t arrayTensorsMax = 0;
        uint32_t arrayGroups = 0;
        uint64_t floatCount = 0;
        bool drained = false;
        for (;;)
        {
            const ovphysx_api_status_t status = ovphysx_fetch_read_next(physx()->handle(), read, &group).status;
            if (status == OVPHYSX_API_END_OF_ITERATION)
            {
                drained = true;
                break;
            }
            if (status != OVPHYSX_API_SUCCESS || group == nullptr)
            {
                printFormatted("OutputRead: ovphysx_fetch_read_next failed after %u groups", groups);
                break;
            }
            // One group per emitted attribute, each stacking EVERY prim along shape[0], so the
            // body count is ONE group's rows, not the sum across groups.
            if (groups == 0 && group->data.tensors && group->data.tensor_count > 0 &&
                group->data.tensors[0].shape)
            {
                rows = group->data.tensors[0].shape[0];
            }
            // ARRAY groups only: a FIXED group carries one tensor by definition, so folding it in
            // would drive the minimum to 1 for every mixed read. For an array group the count IS
            // the prim count, which is the only thing tying a lane whose envs are authored rather
            // than cloned to the scene it actually loaded.
            if (group->is_array && group->data.tensors)
            {
                arrayTensorsMin = (std::min)(arrayTensorsMin, group->data.tensor_count);
                arrayTensorsMax = (std::max)(arrayTensorsMax, group->data.tensor_count);
                ++arrayGroups;
            }
            // Setup only. The payload is invariant across the reads of a lane (same objects, same
            // attributes), so counting it once during warm-up is enough, and readOnce() is the
            // default timedRead(): anything left in here is charged to EVERY lane's measurement.
            if (mCountPayload && group->data.tensors)
            {
                for (uint32_t tensorIdx = 0; tensorIdx < group->data.tensor_count; ++tensorIdx)
                {
                    const DLTensor& tensor = group->data.tensors[tensorIdx];
                    if (tensor.dtype.code != kDLFloat || tensor.dtype.bits != 32 || !tensor.shape)
                        continue;
                    uint64_t elements = tensor.dtype.lanes;
                    for (int32_t dim = 0; dim < tensor.ndim; ++dim)
                        elements *= static_cast<uint64_t>(tensor.shape[dim]);
                    floatCount += elements;
                }
            }
            ++groups;
            if (!waitForGroup(*group))
            {
                printFormatted("OutputRead: cuda_sync wait failed after %u groups", groups);
                break;
            }
            // Released per group, the loop the header prescribes and every other consumer writes.
            const ovstage_read_group_id_t groupId = group->read_group_id;
            group = nullptr;
            if (ovphysx_release_group(physx()->handle(), read, groupId).status != OVPHYSX_API_SUCCESS)
            {
                printFormatted("OutputRead: ovphysx_release_group failed after %u groups", groups);
                break;
            }
        }

        ovphysx_release_read(physx()->handle(), read);
        if (!drained)
            return false;

        mLastGroupCount = groups;
        mLastRowCount = static_cast<uint32_t>(rows);
        // EVERY timed read contributes its shape to these extremes. The comparison against the
        // warmed shape happens once, after the timing, in checkTimedShape(), so the validation
        // costs four min/max updates inside the measured region and the branching is deferred to a
        // point where nothing is being timed. Recording only the last read's shape would miss an
        // iteration that drifted and drifted back.
        if (!mCountPayload)
        {
            mTimedGroupMin = mTimedGroupMin < groups ? mTimedGroupMin : groups;
            mTimedGroupMax = mTimedGroupMax > groups ? mTimedGroupMax : groups;
            const uint32_t rowsU = static_cast<uint32_t>(rows);
            mTimedRowMin = mTimedRowMin < rowsU ? mTimedRowMin : rowsU;
            mTimedRowMax = mTimedRowMax > rowsU ? mTimedRowMax : rowsU;
        }
        // Normalised so a lane with no array groups reads 0/0 rather than UINT32_MAX/0, which would
        // make the timed comparison below trip on a shape that never existed.
        mLastArrayGroupCount = arrayGroups;
        mLastArrayTensorsMin = arrayGroups ? arrayTensorsMin : 0u;
        mLastArrayTensorsMax = arrayTensorsMax;
        mTimedShapeSeen = !mCountPayload;
        if (mCountPayload)
        {
            mLastFloatCount = floatCount;
            // A lane that drains cleanly but resolved nothing is measuring an empty read. Checked
            // during setup rather than per timed read, to keep the branch out of the measured region.
            if (groups == 0 || rows <= 0)
            {
                printFormatted(
                    "OutputRead: drained successfully but produced groups=%u rows=%lld -- invalid benchmark run",
                    groups, static_cast<long long>(rows));
                return false;
            }
        }
        return true;
    }

private:
    // Query + read: what a caller pays carrying no state between frames.
    bool readOnce()
    {
        ovphysx_query_handle_t query = 0;
        if (!openQuery(query))
            return false;
        const bool ok = readWithQuery(query);
        ovphysx_release_query(physx()->handle(), query);
        return ok;
    }

protected:
    // step() refuses to time while this is false, so a scene that never came up cannot report.
    bool mSetupOk = false;

    uint32_t mEnvCount;
    bool mGpu;

    // Rows the last read produced. Exposed so a lane whose scene comes from a committed asset rather
    // than from cloning can check the asset is the size its name claims. mEnvCount does not reach
    // such a scene, so nothing else ties the two together.
    uint32_t lastRowCount() const
    {
        return mLastRowCount;
    }
    uint32_t lastGroupCount() const
    {
        return mLastGroupCount;
    }
    uint64_t lastFloatCount() const
    {
        return mLastFloatCount;
    }
    // The smallest and largest tensor count across the read's ARRAY groups. Equal means every group
    // covered the same prims. A gap is a truncated read that one group would have hidden.
    uint32_t lastArrayTensorsMin() const
    {
        return mLastArrayTensorsMin;
    }
    uint32_t lastArrayTensorsMax() const
    {
        return mLastArrayTensorsMax;
    }
    uint32_t lastArrayGroupCount() const
    {
        return mLastArrayGroupCount;
    }
    ovphysx::PhysX* physx() const { return mPhysX; }

private:
    uint32_t mFailedReads = 0;
    uint32_t mAttempts = 0;
    uint32_t mLastGroupCount = 0;
    uint32_t mLastRowCount = 0;
    // The shape the warm-up settled on, and whether a timed read has produced one to compare.
    uint32_t mExpectedGroupCount = 0;
    uint32_t mExpectedRowCount = 0;
    uint32_t mExpectedArrayTensorsMin = 0;
    uint32_t mExpectedArrayTensorsMax = 0;
    bool mTimedShapeSeen = false;
    // Seeded so the first timed read sets both ends. checkTimedShape() only consults them when
    // mTimedShapeSeen says a timed read actually happened.
    uint32_t mTimedGroupMin = UINT32_MAX;
    uint32_t mTimedGroupMax = 0;
    uint32_t mTimedRowMin = UINT32_MAX;
    uint32_t mTimedRowMax = 0;
    bool mFailureRecorded = false;
    uint64_t mLastFloatCount = 0;
    uint32_t mLastArrayTensorsMin = 0;
    uint32_t mLastArrayTensorsMax = 0;
    uint32_t mLastArrayGroupCount = 0;
    // True only while startRun() is warming. It keeps payload accounting out of the timed region.
    bool mCountPayload = true;
    bool mReportedShape = false;
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mStageAttachment{};
};


// The same payload through the tensor-binding API: pose [N, 7] plus velocity [N, 6].
//
// The destination matches the scene: device memory on DirectGPU, host on CPU. That is the binding
// at its native residency. Binding it to host instead would force a device-to-host copy per read
// and measure the API in the mode that defeats it. The binding stamps kDLCUDA on the sim's device
// (ADR-0008) when mGpu is set.
//
// The output read (OutputReadBase::readWithQuery(), above) is NOT device-resident on a DirectGPU
// scene: ovphysx_read()/ovphysx_fetch_read_next() hand back host-backed ovstage_read_group_t data
// regardless of scene residency, so only the tensor-binding destination moves to device memory. The
// two GPU rows are therefore not like-for-like on residency. Read the CPU/GPU deltas on each row
// independently rather than comparing the read and binding rows to each other as a residency ratio.
//
// What still differs beyond residency is how each becomes readable. The output read publishes a
// session completion event and this harness waits on every returned group. The binding's read()
// calls are followed by waitAll(). Both lanes therefore measure completed work rather than
// comparing one submission with one completion.
class TensorBindingReadBase : public OutputReadBase
{
public:
    TensorBindingReadBase(uint32_t envCount, bool gpu) : OutputReadBase(envCount, gpu) {}

    bool isValid() const override
    {
#if !defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        // Without CUDA the destination silently falls back to host memory, which turns this into a
        // second host-backed read reported under a name that says device.
        if (mGpu)
        {
            throw std::runtime_error(
                "OutputRead tensorbinding GPU variant requires a CUDAToolkit-enabled benchmark build");
        }
#endif
        return OutputReadBase::isValid();
    }

    // A binding read is far shorter than the output read the base is tuned for, so it needs more
    // samples to converge. Matches TensorIo.pose_read_*, which measures the same operation.
    uint32_t getNbSteps() const override { return mEnvCount >= 8192 ? 50 : 100; }

    void startRun() override
    {
        OutputReadBase::startRun();
        if (!mSetupOk) return; // base setup already failed and has reported why

        // Not ready until THIS API's bindings exist: timedRead() dereferences mPose/mVel, which a
        // failed bind would leave unconstructed.
        mSetupOk = false;
        if (!physx()) return;
        if (!bind(mPose, mPoseStorage, mPoseDev, mPoseShape, mPoseView, firstTensorType(), firstName()))
            return;
        if (hasSecondTensor() &&
            !bind(mVel, mVelStorage, mVelDev, mVelShape, mVelView, secondTensorType(), secondName()))
            return;
        for (const ovphysx_tensor_type_t type : extraTensorTypes())
        {
            mExtra.emplace_back();
            ExtraBinding& e = mExtra.back();
            e.type = type;
            if (!bind(e.binding, e.storage, e.dev, e.shape, e.view, type, "extra tensor"))
                return;
        }
        if (!validateBindingLayout())
            return;
        // The base could only warm the output read. First reads here populate the binding's own
        // internal state, not the timed cost.
        for (int warm = 0; warm < kWarmUpReads; ++warm)
        {
            if (!timedRead())
            {
                printFormatted("TensorBindingRead: warm-up read failed -- run will report no timings");
                return;
            }
        }
        mSetupOk = true;

        // Element counts, not shape[0], against the OutputRead line above: the two APIs resolve the
        // object set independently AND disagree about what a row is. The articulation binding
        // returns [articulations, links, comps] while the read returns one row per link, so
        // comparing the leading dimension says they differ when they cover exactly the same data.
        // Total floats is the quantity that has to match for the timings to mean anything.
        if (!mReportedBinding)
        {
            mReportedBinding = true;
            auto elems = [](const std::vector<int64_t>& shape)
            {
                long long n = shape.empty() ? 0 : 1;
                for (int64_t d : shape)
                    n *= static_cast<long long>(d);
                return n;
            };
            auto describe = [](const std::vector<int64_t>& shape)
            {
                std::string out;
                for (size_t i = 0; i < shape.size(); ++i)
                    out += (i ? "x" : "") + std::to_string(shape[i]);
                return out.empty() ? std::string("-") : out;
            };
            printFormatted("TensorBindingRead: envs=%u dst=%s pose=%s (%lld floats) vel=%s (%lld floats) total=%lld",
                           mEnvCount, mGpu ? "device" : "host", describe(mPoseShape).c_str(), elems(mPoseShape),
                           describe(mVelShape).c_str(), elems(mVelShape), elems(mPoseShape) + elems(mVelShape));
        }
    }

    void endRun() override
    {
        finishRun();
        // Destroyed and CLEARED, not just destroyed: mExtra is a member, so a run that left its
        // entries behind would have the next run read bindings created against the previous stage.
        for (ExtraBinding& e : mExtra)
        {
            e.binding.destroy();
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
            (void)cudaFree(e.dev);
#endif
            e.dev = nullptr;
        }
        mExtra.clear();
        mVel.destroy();
        mPose.destroy();
#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        (void)cudaFree(mVelDev);
        (void)cudaFree(mPoseDev);
#endif
        mVelDev = nullptr;
        mPoseDev = nullptr;
        teardown();
    }

protected:
    // Which objects the binding covers and which two tensors it reads. Seams rather than constants
    // so the articulation lanes reuse this whole class. The allocation, the timed loop, the
    // warm-up and the row reporting are identical, only the selector and the tensor types differ.
    virtual const char* bindingPattern() const { return kBodyPattern; }
    virtual ovphysx_tensor_type_t firstTensorType() const { return OVPHYSX_TENSOR_RIGID_BODY_POSE_F32; }
    virtual ovphysx_tensor_type_t secondTensorType() const { return OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32; }
    virtual const char* firstName() const { return "pose"; }
    virtual const char* secondName() const { return "velocity"; }

    // Most lanes face a read of two columns. A lane whose read is ONE column overrides this to false
    // rather than binding a filler: a second tensor the read side never asks for would charge the
    // binding for work it does not do.
    virtual bool hasSecondTensor() const { return true; }

    // Tensors BEYOND the first two, for a lane whose read covers more columns than that pair. A ratio
    // lane has to bind exactly what the read side asks for: fewer charges the binding for less work
    // than the read does, more for work it never does, and either way the ratio means nothing.
    virtual std::vector<ovphysx_tensor_type_t> extraTensorTypes() const { return {}; }

    // Does the binding cover the SAME payload the output read covers? A ratio between two APIs is
    // meaningless unless both resolved the same objects and the same element counts, and nothing
    // else in this lane checks that.
    //
    // Enforced by default rather than opt-in, so a lane cannot skip the check by saying nothing. A
    // lane whose binding legitimately covers a different payload has to override this and say why.
    //
    // Total floats, not shape[0]: the two APIs disagree about what a row is. The articulation
    // binding returns [articulations, links, comps] where the read returns one row per link, so
    // the leading dimension differs while the data covered is identical.
    virtual bool validateBindingLayout() const
    {
        const uint64_t bound = bindingFloatCount();
        const uint64_t read = lastFloatCount();
        if (bound == read)
            return true;
        printFormatted("TensorBindingRead: the binding covers %llu floats but the output read covers %llu -- "
                       "the two APIs did not resolve the same payload, so no ratio is reported",
                       static_cast<unsigned long long>(bound), static_cast<unsigned long long>(read));
        return false;
    }

    const std::vector<int64_t>& firstShape() const
    {
        return mPoseShape;
    }
    const std::vector<int64_t>& secondShape() const
    {
        return mVelShape;
    }

    uint64_t bindingFloatCount() const
    {
        uint64_t count = 0;
        std::vector<const std::vector<int64_t>*> shapes{ &mPoseShape, &mVelShape };
        for (const ExtraBinding& e : mExtra)
            shapes.push_back(&e.shape);
        for (const std::vector<int64_t>* shape : shapes)
        {
            uint64_t elements = shape->empty() ? 0 : 1;
            for (int64_t dim : *shape)
                elements *= static_cast<uint64_t>(dim);
            count += elements;
        }
        return count;
    }

    bool timedRead() override
    {
        if (mPose.read(mPoseView) != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorBindingRead: %s read failed", firstName());
            return false;
        }
        if (hasSecondTensor() && mVel.read(mVelView) != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorBindingRead: %s read failed", secondName());
            return false;
        }
        for (ExtraBinding& e : mExtra)
        {
            if (e.binding.read(e.view) != OVPHYSX_API_SUCCESS)
            {
                const ovphysx_string_t err = ovphysx_get_last_error();
                printFormatted("TensorBindingRead: extra tensor (type %d) read failed: %.*s",
                               static_cast<int>(e.type), static_cast<int>(err.length),
                               err.ptr ? err.ptr : "");
                return false;
            }
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
            printFormatted("TensorBindingRead: createTensorBinding(%s) failed", what);
            return false;
        }
        ovphysx_tensor_spec_t spec{};
        if (binding.spec(spec) != OVPHYSX_API_SUCCESS || spec.ndim < 1)
        {
            printFormatted("TensorBindingRead: spec(%s) failed", what);
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
                printFormatted("TensorBindingRead: cudaMalloc(%s) failed", what);
                return false;
            }
            view.data = deviceStorage;
            view.device = DLDevice{ kDLCUDA, 0 };
        }
        else
#endif
        {
            storage.assign(elems, 0.0f);
            view.data = storage.data();
            view.device = DLDevice{ kDLCPU, 0 };
        }
        view.ndim = static_cast<int32_t>(shape.size());
        // From the spec, not assumed: ovphysx_types.h states "Always ... respect the returned dtype;
        // do not assume float32". The drive-type binding is uint8.
        view.dtype = spec.dtype;
        view.shape = shape.data();
        view.strides = nullptr;
        view.byte_offset = 0;
        return true;
    }

    ovphysx::TensorBinding mPose;
    ovphysx::TensorBinding mVel;
    std::vector<float> mPoseStorage, mVelStorage; // CPU scenes
    void* mPoseDev = nullptr;                     // DirectGPU scenes
    void* mVelDev = nullptr;
    std::vector<int64_t> mPoseShape, mVelShape;
    DLTensor mPoseView{}, mVelView{};

    // One per extraTensorTypes() entry. A deque so element addresses stay stable: bind() stores
    // pointers into an entry's storage while later entries are still being appended.
    struct ExtraBinding
    {
        ovphysx_tensor_type_t type{};
        ovphysx::TensorBinding binding;
        std::vector<float> storage;
        void* dev = nullptr;
        std::vector<int64_t> shape;
        DLTensor view{};
    };
    std::deque<ExtraBinding> mExtra;
    bool mReportedBinding = false;
};


// Read with the query opened once and held across steps. A query is a reusable selector resolved
// lazily at read time, so this is legitimate usage, not a benchmark shortcut. Against queryread_*
// the difference is what re-opening the selector costs on every frame.
class ReadOnlyBase : public OutputReadBase
{
public:
    ReadOnlyBase(uint32_t envCount, bool gpu) : OutputReadBase(envCount, gpu) {}

    void startRun() override
    {
        OutputReadBase::startRun();
        if (!mSetupOk) return; // base setup already failed and has reported why

        mSetupOk = false;
        if (!openQuery(mQuery)) return;
        for (int warm = 0; warm < kWarmUpReads; ++warm)
        {
            if (!readWithQuery(mQuery))
            {
                printFormatted("OutputReadOnly: warm-up read failed -- run will report no timings");
                return;
            }
        }
        mSetupOk = true;
    }

    void endRun() override
    {
        finishRun();
        if (mQuery != 0 && physx())
        {
            ovphysx_release_query(physx()->handle(), mQuery);
        }
        mQuery = 0;
        teardown();
    }

protected:
    bool timedRead() override { return readWithQuery(mQuery); }

private:
    ovphysx_query_handle_t mQuery = 0;
};

// What creating a view over the body set costs: the tensor binding pays it once and amortises it,
// the output read pays the equivalent on every call. Create and destroy are measured together
// because a step has to be repeatable.
class TensorBindingCreateBase : public OutputReadBase
{
public:
    TensorBindingCreateBase(uint32_t envCount, bool gpu) : OutputReadBase(envCount, gpu) {}

    // Creating a view over 163k bodies is far heavier than reading one.
    uint32_t getNbSteps() const override { return 10; }

    void startRun() override
    {
        OutputReadBase::startRun();
        if (!mSetupOk) return;

        mSetupOk = false;
        if (!physx()) return;
        for (int warm = 0; warm < kWarmUpReads; ++warm)
        {
            if (!timedRead())
            {
                printFormatted("TensorBindingCreate: warm-up failed -- run will report no timings");
                return;
            }
        }
        mSetupOk = true;
    }

    void endRun() override
    {
        finishRun();
        teardown();
    }

protected:
    // Same seams as TensorBindingReadBase, so a lane that changes what it binds changes it once.
    virtual const char* bindingPattern() const { return kBodyPattern; }
    virtual ovphysx_tensor_type_t firstTensorType() const { return OVPHYSX_TENSOR_RIGID_BODY_POSE_F32; }
    virtual ovphysx_tensor_type_t secondTensorType() const { return OVPHYSX_TENSOR_RIGID_BODY_VELOCITY_F32; }
    virtual const char* firstName() const { return "pose"; }
    virtual const char* secondName() const { return "velocity"; }

    bool timedRead() override
    {
        // Both views, because the matching read lane reads through both.
        ovphysx::TensorBinding first;
        ovphysx::TensorBinding second;
        if (!create(first, firstTensorType(), firstName()))
        {
            return false;
        }
        if (!create(second, secondTensorType(), secondName()))
        {
            first.destroy();
            return false;
        }
        second.destroy();
        first.destroy();
        return true;
    }

private:
    // spec() is part of creating a usable view: a caller needs the shape before it can allocate
    // the destination tensor.
    bool create(ovphysx::TensorBinding& binding, ovphysx_tensor_type_t type, const char* what)
    {
        if (physx()->createTensorBinding(binding, bindingPattern(), type) != OVPHYSX_API_SUCCESS)
        {
            printFormatted("TensorBindingCreate: createTensorBinding(%s) failed", what);
            return false;
        }
        ovphysx_tensor_spec_t spec{};
        if (binding.spec(spec) != OVPHYSX_API_SUCCESS || spec.ndim < 1)
        {
            printFormatted("TensorBindingCreate: spec(%s) failed", what);
            binding.destroy();
            return false;
        }
        return true;
    }
};

// Articulations through the same harness: same timed loop, same drain, same failure reporting.
// Only the scene and the object type change. Cartpole rather than the cubes scene because it is the
// articulation workload this suite already carries at scale (LabCartpole drives it), and it clones
// the same way: one articulation per env, three links and two DOFs each.
//
// Two families, because the two object types take different paths through the read:
//   * artilink -> ARTICULATION_LINK, the SAME four pose/velocity attributes as the rigid lane, so a
//     per-row comparison against queryread means something.
//   * artidof  -> ARTICULATION_JOINT, jointPosition/jointVelocity, the per-axis array path, which
//     has no rigid equivalent to compare against at all.
//
// The GPU/CPU split needs two assets for the same reason the cubes pair does: cartpole_probe.usda
// authors enableGPUDynamics + GPU broadphase, so a CPU lane running it would measure the GPU
// pipeline under a name that says cpu. cartpole_probe_cpu.usda is that file with those two scene
// attributes removed, and nothing else.
//
// A mixin rather than a base class: both reading APIs need this scene and it has to be identical
// for their numbers to be comparable, so parameterising on the read base keeps ONE definition of
// the asset, the clone source and the grid instead of one per API that can drift apart silently.
template <typename ReadBase>
class CartpoleSceneT : public ReadBase
{
public:
    CartpoleSceneT(uint32_t envCount, bool gpu) : ReadBase(envCount, gpu) {}

protected:
    const char* sceneAsset() const override
    {
        return this->mGpu ? "/../benchmarks/data/cartpole_probe.usda" :
                            "/../benchmarks/data/cartpole_probe_cpu.usda";
    }
    // env_0 is the template AND stays in the scene, so N clones cover N+1 articulations, the same
    // shape the rigid lane reports, where the cloned-from template is also read.
    const char* cloneSource() const override { return "/World/envs/env_0"; }
    std::string cloneTarget(uint32_t i) const override
    {
        return "/World/envs/env_" + std::to_string(i + 1);
    }
    // Matches LabCartpole's grid. The pole is 1.0 tall on a 0.4 cart, so 4.0 keeps neighbouring
    // envs from touching. Overlapping them would add contacts and measure a different scene.
    float cloneSpacing() const override { return 4.0f; }
};

using ArticulationReadBase = CartpoleSceneT<OutputReadBase>;


// The same two articulation workloads through the tensor binding: what artilink and artidof are
// compared against, on the same scene, clone grid and timed loop. The Lab* suites read articulations
// through the binding too, but on a different asset and harness, so they cannot answer what the same
// work costs through each API.
//
// The attributes line up with the read lanes they face: LINK_POSE + LINK_VELOCITY against
// artilink's pose/orientation/linear/angular columns, DOF_POSITION + DOF_VELOCITY against
// artidof's jointPosition/jointVelocity.
class ArticulationLinkBindingReadBase : public CartpoleSceneT<TensorBindingReadBase>
{
public:
    ArticulationLinkBindingReadBase(uint32_t envCount, bool gpu)
        : CartpoleSceneT<TensorBindingReadBase>(envCount, gpu)
    {
    }

protected:
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t firstTensorType() const override { return OVPHYSX_TENSOR_ARTICULATION_LINK_POSE_F32; }
    ovphysx_tensor_type_t secondTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_LINK_VELOCITY_F32;
    }
    const char* firstName() const override { return "link pose"; }
    const char* secondName() const override { return "link velocity"; }

    // Also set for the READ side, which this class inherits and which its base warms up with. The
    // OutputRead line it prints is how the two lanes are shown to cover the same set. Leaving the
    // rigid-body default would resolve nothing on a cartpole scene and report groups=0 rows=0,
    // silently dropping the only cross-check there is.
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_LINK; }
};


class ArticulationDofBindingReadBase : public CartpoleSceneT<TensorBindingReadBase>
{
public:
    ArticulationDofBindingReadBase(uint32_t envCount, bool gpu)
        : CartpoleSceneT<TensorBindingReadBase>(envCount, gpu)
    {
    }

protected:
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t firstTensorType() const override { return OVPHYSX_TENSOR_ARTICULATION_DOF_POSITION_F32; }
    ovphysx_tensor_type_t secondTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_DOF_VELOCITY_F32;
    }
    const char* firstName() const override { return "dof position"; }
    const char* secondName() const override { return "dof velocity"; }

    // Same reason as the link lane above: the inherited read has to describe this lane's objects.
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_JOINT; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return jointReadAttrs(count); }
};


// The articulation read with the query opened once and held, which is what makes the binding
// comparison honest: artilink/artidof re-open a selector every frame, while a binding resolves
// its selector once at bind time and never again. Comparing those directly charges the read for
// work the binding does not do. These are the lanes to put next to tensorlink / tensordof. The
// queryread-shaped ones above answer the different question of what a stateless caller pays.
class ArticulationLinkReadOnlyBase : public CartpoleSceneT<ReadOnlyBase>
{
public:
    ArticulationLinkReadOnlyBase(uint32_t envCount, bool gpu) : CartpoleSceneT<ReadOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_LINK; }
};


class ArticulationDofReadOnlyBase : public CartpoleSceneT<ReadOnlyBase>
{
public:
    ArticulationDofReadOnlyBase(uint32_t envCount, bool gpu) : CartpoleSceneT<ReadOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION_JOINT; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return jointReadAttrs(count); }
};


// Whole-articulation root pose + velocity. One row per articulation, so CartpoleSceneT's retained
// template plus N clones must produce exactly N+1 rows, and the four columns carry 3+4+3+3 = 13
// floats each. Checking all three keeps an empty or partial producer from reporting as a fast run.
class ArticulationRootReadOnlyBase : public CartpoleSceneT<ReadOnlyBase>
{
public:
    ArticulationRootReadOnlyBase(uint32_t envCount, bool gpu) : CartpoleSceneT<ReadOnlyBase>(envCount, gpu)
    {
    }

    void startRun() override
    {
        ReadOnlyBase::startRun();
        if (!mSetupOk)
            return;

        const uint32_t expectedRows = mEnvCount + 1;
        const uint64_t expectedFloats = uint64_t(expectedRows) * 13u;
        if (lastRowCount() != expectedRows || lastGroupCount() != 4u || lastFloatCount() != expectedFloats)
        {
            printFormatted(
                "OutputRead articulation root: expected rows=%u groups=4 floats=%llu, got rows=%u "
                "groups=%u floats=%llu -- run will report no timings",
                expectedRows, static_cast<unsigned long long>(expectedFloats), lastRowCount(), lastGroupCount(),
                static_cast<unsigned long long>(lastFloatCount()));
            mSetupOk = false;
        }
    }

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return articulationRootReadAttrs(count); }
};


// TensorBindings control for the exact same cartpole root payload.
class ArticulationRootBindingReadBase : public CartpoleSceneT<TensorBindingReadBase>
{
public:
    ArticulationRootBindingReadBase(uint32_t envCount, bool gpu) : CartpoleSceneT<TensorBindingReadBase>(envCount, gpu)
    {
    }

protected:
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t firstTensorType() const override { return OVPHYSX_TENSOR_ARTICULATION_ROOT_POSE_F32; }
    ovphysx_tensor_type_t secondTensorType() const override
    {
        return OVPHYSX_TENSOR_ARTICULATION_ROOT_VELOCITY_F32;
    }
    const char* firstName() const override { return "root pose"; }
    const char* secondName() const override { return "root velocity"; }
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION; }
    // Set for the READ side this class inherits and warms up with, exactly as the link and dof
    // binding lanes do: leaving the rigid default would resolve nothing on this type and report
    // groups=0 rows=0.
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return articulationRootReadAttrs(count); }
    bool validateBindingLayout() const override
    {
        const int64_t expectedRows = static_cast<int64_t>(mEnvCount) + 1;
        const uint64_t expectedFloats = static_cast<uint64_t>(expectedRows) * 13u;
        const std::vector<int64_t>& pose = firstShape();
        const std::vector<int64_t>& velocity = secondShape();
        const bool exact = pose.size() == 2u && pose[0] == expectedRows && pose[1] == 7 && velocity.size() == 2u &&
                           velocity[0] == expectedRows && velocity[1] == 6 && bindingFloatCount() == expectedFloats;
        if (!exact)
        {
            printFormatted(
                "TensorBinding articulation root: expected pose=%lldx7 velocity=%lldx6 floats=%llu -- "
                "run will report no timings",
                static_cast<long long>(expectedRows), static_cast<long long>(expectedRows),
                static_cast<unsigned long long>(expectedFloats));
        }
        return exact;
    }
};

// Vehicles. Three things make this lane unlike every other one here, all forced rather than chosen:
//
//   * It does NOT clone. ovphysx_clone cannot replicate a vehicle (PhysXReplicator switches on
//     PxConcreteType and has no vehicle case), so cloning would leave one real vehicle and N-1
//     chassis actors with no wheel attachments, and the lane would report N envs while reading
//     four wheels. The asset carries all 1024 cars instead.
//   * There is no _gpu variant, because there cannot be: a vehicle's suspension and sticky-tire
//     constraints are custom PxConstraints with a CPU solver-prep function, and PhysX refuses those
//     on a DirectGPU scene (REQ-READ-DEVICE-001 AC-8).
//   * mEnvCount is a LABEL here, not a parameter. With no clone step nothing carries it into the
//     scene, so the 1024 in the lane's name is only as true as the committed asset. That is why
//     startRun() checks the row count and fails the lane when the two disagree.
class VehicleWheelReadOnlyBase : public ReadOnlyBase
{
public:
    VehicleWheelReadOnlyBase(uint32_t envCount, bool gpu) : ReadOnlyBase(envCount, gpu) {}

protected:
    // Four wheels per car, which is what create4WheeledCarsScenario authors and what the generator
    // reproduces.
    static constexpr uint32_t kWheelsPerVehicle = 4;

    // No isValid() skip: a missing asset means a broken checkout, and the base already reports
    // "ovstage load failed" and produces no timings.
    const char* sceneAsset() const override { return "/../benchmarks/data/vehicle_probe.usda"; }
    bool clonesEnvs() const override { return false; } // the asset already holds every vehicle
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_VEHICLE_WHEEL; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return vehicleReadAttrs(count); }

    // The row count is the only thing tying this lane's name to the asset it loaded. Nothing else
    // does: clonesEnvs() is false, so mEnvCount never reaches the scene, and for this family it is a
    // label, not a parameter. A stale or differently-sized asset would otherwise produce a perfectly
    // plausible number under a name that says 1024.
    void startRun() override
    {
        ReadOnlyBase::startRun();
        if (!mSetupOk)
            return; // base setup already failed and has reported why

        const uint32_t expected = mEnvCount * kWheelsPerVehicle;
        if (lastRowCount() != expected)
        {
            printFormatted("OutputRead: vehicle asset holds %u wheel rows, this lane is named for %u "
                           "(%u vehicles x %u wheels) -- regenerate with "
                           "'python3 gen_vehicle_probe.py %u > vehicle_probe.usda'",
                           lastRowCount(), expected, mEnvCount, kWheelsPerVehicle, mEnvCount);
            mSetupOk = false;
        }
    }
};


// Deformables: the second family whose objects cannot be cloned, and the mirror image of the
// vehicle one above on every axis:
//
//   * GPU ONLY, where vehicles are CPU only. buildDeformableGroups warns-and-skips without a CUDA
//     context (deformables are device-only by design, ADR-0008 Decision 9) and the binding refuses
//     a deformable tensor outside DirectGPU, so a _cpu registration would report groups=0 rows=0 on
//     both sides. There is no CPU number for this type and there cannot be one.
//
//   * ovphysx_clone cannot replicate a deformable, for the same reason it cannot replicate a
//     vehicle: the replicator switches on PxConcreteType and has no case for PxDeformableVolume /
//     PxDeformableSurface, so cloning a subtree holding one SUCCEEDS and yields envs with no
//     deformable in them. clonesEnvs() is false and the asset authors every env, generated by
//     gen_deformables.py.
//
//   * ONE ASSET PER SIZE, unlike the vehicle lane's single file: the env count is baked in, so the
//     name is built from mEnvCount rather than fixed. Cached in a member because sceneAsset()
//     returns const char*.
//
// The size is 128, not the rigid lane's 8192: a deformable is far dearer per body both to
// cook at load and to step. Read a lane against its own binding partner at the SAME size. These
// are not comparable to readonly_rb.
//
// Each authored-asset lane checks its own shape in startRun(), like the vehicle lane and for the
// same reason: with no clone step nothing else ties mEnvCount to the scene that got loaded. That
// validation lives on AuthoredEnvSceneT below rather than per family. See there for what it pins.


// The authored-asset families, as data: both want the SAME two startRun() checks with different
// nouns, so what varies is a table rather than a second copy of the class.
struct AssetFamily
{
    const char* assetPrefix; // "deformables" -> deformables_envs_128.usda
    const char* name; // for the diagnostics
    const char* prim; // singular: "body", "set"
    const char* primPlural; // carried separately because "bodies" is irregular
    const char* generator; // the script that regenerates the asset
};

constexpr AssetFamily kDeformableFamily{ "deformables", "deformable", "body", "bodies", "gen_deformables.py" };
constexpr AssetFamily kParticleFamily{ "particles", "particle", "set", "sets", "gen_particles.py" };


// An env family authored in a .usda rather than produced by ovphysx_clone.
//
// Two numbers are checked, because neither alone pins the shape:
//
//   groups  == the number of ATTRIBUTES requested (one array group per attribute), and
//   tensors == mEnvCount (one tensor per prim, inside that group).
//
// `rows` is not usable for this: it is one prim's element count (a body's vertices, a set's
// particles) and says nothing about how many prims were found. The first check catches a read that
// regressed to one group per prim per attribute. The second catches a stale or wrongly sized asset
// reporting a plausible number under a name that says 128.
template <typename ReadBase>
class AuthoredEnvSceneT : public ReadBase
{
public:
    AuthoredEnvSceneT(uint32_t envCount, bool gpu, const AssetFamily& family)
        : ReadBase(envCount, gpu),
          mFamily(family),
          mAsset(std::string("/../benchmarks/data/") + family.assetPrefix + "_envs_" +
                 std::to_string(envCount) + ".usda")
    {
    }

protected:
    const char* sceneAsset() const override { return mAsset.c_str(); }
    bool clonesEnvs() const override { return false; } // the asset already holds every env

    void startRun() override
    {
        ReadBase::startRun();
        if (!this->mSetupOk)
            return; // base setup already failed and has reported why

        // Derived from readAttrs() rather than written down, so a lane requesting a different set
        // (a points-only diagnostic, or an attribute added later) needs no edit here.
        size_t attrs = 0;
        this->readAttrs(attrs);
        const uint32_t expectedGroups = static_cast<uint32_t>(attrs);
        if (this->lastGroupCount() != expectedGroups)
        {
            printFormatted("OutputRead: %s read emitted %u groups for %zu attributes -- one array "
                           "group per attribute is the contract, so this is a regression to the "
                           "group-per-%s shape, not an asset problem",
                           mFamily.name, this->lastGroupCount(), attrs, mFamily.prim);
            this->mSetupOk = false;
        }
        else if (this->lastArrayTensorsMin() != this->lastArrayTensorsMax())
        {
            // Checked before the count itself, because a ragged read has no single count to report.
            // One group covering every prim while another covers fewer is a TRUNCATED read.
            printFormatted("OutputRead: %s read is ragged across its %u array groups -- %u..%u "
                           "tensors, so at least one attribute covered fewer %s than another. That "
                           "is a truncated read, not an asset problem",
                           mFamily.name, this->lastArrayGroupCount(), this->lastArrayTensorsMin(),
                           this->lastArrayTensorsMax(), mFamily.primPlural);
            this->mSetupOk = false;
        }
        else if (this->lastArrayTensorsMin() != this->mEnvCount)
        {
            printFormatted("OutputRead: %s asset holds %u %s, this lane is named for %u -- "
                           "regenerate with 'python %s'",
                           mFamily.name, this->lastArrayTensorsMin(), mFamily.primPlural, this->mEnvCount,
                           mFamily.generator);
            this->mSetupOk = false;
        }
    }

private:
    const AssetFamily& mFamily;
    std::string mAsset;
};


template <typename ReadBase>
class DeformableSceneT : public AuthoredEnvSceneT<ReadBase>
{
public:
    DeformableSceneT(uint32_t envCount, bool gpu)
        : AuthoredEnvSceneT<ReadBase>(envCount, gpu, kDeformableFamily)
    {
    }
};


class VolumeDeformableReadOnlyBase : public DeformableSceneT<ReadOnlyBase>
{
public:
    VolumeDeformableReadOnlyBase(uint32_t envCount, bool gpu) : DeformableSceneT<ReadOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_DEFORMABLE_VOLUME; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return deformableReadAttrs(count); }
};


class SurfaceDeformableReadOnlyBase : public DeformableSceneT<ReadOnlyBase>
{
public:
    SurfaceDeformableReadOnlyBase(uint32_t envCount, bool gpu) : DeformableSceneT<ReadOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_DEFORMABLE_SURFACE; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return deformableReadAttrs(count); }
};


// The binding lanes the two read lanes face. Sim nodal position + velocity, matching
// deformableReadAttrs. The selector names the BODY prim inside each env rather than the env itself,
// because a deformable body is one prim, which is also the set the read resolves.
class VolumeDeformableBindingReadBase : public DeformableSceneT<TensorBindingReadBase>
{
public:
    VolumeDeformableBindingReadBase(uint32_t envCount, bool gpu)
        : DeformableSceneT<TensorBindingReadBase>(envCount, gpu)
    {
    }

protected:
    const char* bindingPattern() const override { return "/World/envs/env_*/Volume"; }
    ovphysx_tensor_type_t firstTensorType() const override
    {
        return OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_POSITION_F32;
    }
    ovphysx_tensor_type_t secondTensorType() const override
    {
        return OVPHYSX_TENSOR_DEFORMABLE_SIM_NODAL_VELOCITY_F32;
    }
    const char* firstName() const override { return "sim nodal position"; }
    const char* secondName() const override { return "sim nodal velocity"; }

    // Also set for the inherited READ side, as every other binding lane does: the base prints the
    // line showing both APIs cover the same set, so it has to request this lane's columns.
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_DEFORMABLE_VOLUME; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return deformableReadAttrs(count); }
};


class SurfaceDeformableBindingReadBase : public DeformableSceneT<TensorBindingReadBase>
{
public:
    SurfaceDeformableBindingReadBase(uint32_t envCount, bool gpu)
        : DeformableSceneT<TensorBindingReadBase>(envCount, gpu)
    {
    }

protected:
    const char* bindingPattern() const override { return "/World/envs/env_*/Surface"; }
    ovphysx_tensor_type_t firstTensorType() const override
    {
        return OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_POSITION_F32;
    }
    ovphysx_tensor_type_t secondTensorType() const override
    {
        return OVPHYSX_TENSOR_SURFACE_DEFORMABLE_SIM_VELOCITY_F32;
    }
    const char* firstName() const override { return "surface sim position"; }
    const char* secondName() const override { return "surface sim velocity"; }

    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_DEFORMABLE_SURFACE; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return deformableReadAttrs(count); }
};


// A particle scene, from an authored asset. Same bargain as DeformableSceneT and for the same
// reason: `ovphysx_clone` does not replicate a particle set (PhysXReplicator.cpp names neither
// ePTParticleSet nor ePTParticleSystem), so a cloned template would silently produce N envs
// holding no particles at all, since cloning still succeeds. The envs are in the file instead.
//
// GPU only. The runtime registers a particle system only on a GPU-capable scene, and the read
// refuses a particle column with no CUDA context (ADR-0008 Decision 9), so a _cpu lane would report
// zero groups rather than a slower number.
template <typename ReadBase>
class ParticleSceneT : public AuthoredEnvSceneT<ReadBase>
{
public:
    ParticleSceneT(uint32_t envCount, bool gpu)
        : AuthoredEnvSceneT<ReadBase>(envCount, gpu, kParticleFamily)
    {
    }
};


class ParticleReadOnlyBase : public ParticleSceneT<ReadOnlyBase>
{
public:
    ParticleReadOnlyBase(uint32_t envCount, bool gpu) : ParticleSceneT<ReadOnlyBase>(envCount, gpu) {}

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_PARTICLE_SET; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override { return particleReadAttrs(count); }
};


// One macro per family. The registered name is derived from the same tokens that build the class,
// so they cannot disagree. `gpu` and `hide` are separate parameters: they coincide today only
// because the DirectGPU variants are the ones worth hiding.
#define DEFINE_OUTPUT_READ_VARIANT(base, cls, op, N, device, gpu, hide)      \
    class cls##_##N##_##device : public base                                 \
    {                                                                        \
    public:                                                                  \
        cls##_##N##_##device() : base(N, gpu) {}                             \
                                                                             \
    protected:                                                               \
        const char* variantName() const override { return "OutputRead." #op; } \
    };                                                                       \
    Register<cls##_##N##_##device, hide> s##cls##_##N##_##device(            \
        "OutputRead." #op "_" #N "_" #device);

#define DEFINE_QUERY_READ(N, device, gpu, hide)                              \
    DEFINE_OUTPUT_READ_VARIANT(OutputReadBase, OutputRead, queryread_rb, N, device, gpu, hide)

#define DEFINE_READ_ONLY(N, device, gpu, hide)                               \
    DEFINE_OUTPUT_READ_VARIANT(ReadOnlyBase, OutputReadOnly, readonly_rb, N, device, gpu, hide)

#define DEFINE_TENSOR_BINDING_READ(N, device, gpu, hide)                     \
    DEFINE_OUTPUT_READ_VARIANT(TensorBindingReadBase, TensorBindingRead, tensorbinding_rb, N, device, gpu, hide)

#define DEFINE_TENSOR_BINDING_CREATE(N, device, gpu, hide)                   \
    DEFINE_OUTPUT_READ_VARIANT(TensorBindingCreateBase, TensorBindingCreate, tensorcreate_rb, N, device, gpu, hide)

#define DEFINE_ARTI_LINK_BINDING(N, device, gpu, hide)                       \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationLinkBindingReadBase, ArticulationLinkBindingRead, tensorbinding_arti_link, N, device, gpu, hide)

#define DEFINE_ARTI_DOF_BINDING(N, device, gpu, hide)                        \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationDofBindingReadBase, ArticulationDofBindingRead, tensorbinding_arti_dof, N, device, gpu, hide)

#define DEFINE_ARTI_LINK_READ_ONLY(N, device, gpu, hide)                     \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationLinkReadOnlyBase, ArticulationLinkReadOnly, readonly_arti_link, N, device, gpu, hide)

#define DEFINE_ARTI_DOF_READ_ONLY(N, device, gpu, hide)                      \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationDofReadOnlyBase, ArticulationDofReadOnly, readonly_arti_dof, N, device, gpu, hide)

// The inverse dynamics read on the homogeneous fleet: every cartpole shares a metatype, so this is ONE
// cohort and one group per attribute. That is the shape a real workload has, and the number that
// must not regress.
class ArticulationInverseDynamicsReadOnlyBase : public CartpoleSceneT<ReadOnlyBase>
{
public:
    ArticulationInverseDynamicsReadOnlyBase(uint32_t envCount, bool gpu) : CartpoleSceneT<ReadOnlyBase>(envCount, gpu)
    {
    }

protected:
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override
    {
        return articulationInverseDynamicsReadAttrs(count);
    }

    // Two attributes over one cohort is two groups. Asserting it is what stops a partitioning
    // regression. One that split the fleet per articulation would still produce a plausible time.
    void startRun() override
    {
        ReadOnlyBase::startRun();
        if (!mSetupOk)
            return;
        const uint32_t expectedRows = mEnvCount + 1;
        if (lastRowCount() != expectedRows || lastGroupCount() != 2u)
        {
            printFormatted("OutputRead articulation inverse dynamics: expected rows=%u groups=2 (one cohort), got "
                           "rows=%u groups=%u -- invalid benchmark run",
                           expectedRows, lastRowCount(), lastGroupCount());
            mSetupOk = false;
        }
    }
};

// The opposite end: articulation_pileup holds sixteen articulations of increasing link count, so
// every row is its own cohort and every group carries exactly one. Real scenes hold a handful of
// topologies rather than sixteen, so this is an upper bound on per-cohort overhead, not a workload.
// Against the lane above it says what fragmenting a read into groups costs.
class ArticulationInverseDynamicsMixedBase : public ReadOnlyBase
{
public:
    ArticulationInverseDynamicsMixedBase(uint32_t envCount, bool gpu) : ReadOnlyBase(envCount, gpu) {}

protected:
    const char* sceneAsset() const override { return "/../benchmarks/data/articulation_pileup.usda"; }
    bool clonesEnvs() const override { return false; } // the asset already holds every articulation
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION; }
    const ovx_string_or_token_t* readAttrs(size_t& count) const override
    {
        return articulationInverseDynamicsReadAttrs(count);
    }

    // As on the vehicle lane, mEnvCount is a label rather than a parameter here, so the row count is
    // the only thing tying the name to the asset. Groups are checked too: one per (attribute,
    // cohort), which is what makes this the fragmented case rather than just a small one.
    void startRun() override
    {
        ReadOnlyBase::startRun();
        if (!mSetupOk)
            return;
        // Checked on GROUPS, not rows. lastRowCount() is the first group's row count. The harness
        // assumes one group per attribute covering every prim, which is exactly the assumption a
        // cohort read breaks. Here each of the sixteen cohorts holds one articulation, so the two
        // attributes make sixteen single-row groups apiece.
        const uint32_t expectedGroups = mEnvCount * 2u;
        if (lastGroupCount() != expectedGroups || lastRowCount() != 1u)
        {
            printFormatted("OutputRead articulation inverse dynamics (mixed): asset produced %u groups of %u "
                           "rows, this lane is named for %u cohorts x 2 attributes = %u single-row "
                           "groups -- invalid benchmark run",
                           lastGroupCount(), lastRowCount(), mEnvCount, expectedGroups);
            mSetupOk = false;
        }
    }
};

// The TensorBindings control for the inverse dynamics read. Only the homogeneous lane can have one: the
// binding refuses these quantities on a non-homogeneous view (getJacobianShape and
// getGeneralizedMassMatrixShape both fail), which is the divergence the output read exists to close,
// so the mixed lane has no counterpart by construction rather than by omission.
class ArticulationInverseDynamicsBindingReadBase : public CartpoleSceneT<TensorBindingReadBase>
{
public:
    ArticulationInverseDynamicsBindingReadBase(uint32_t envCount, bool gpu)
        : CartpoleSceneT<TensorBindingReadBase>(envCount, gpu)
    {
    }

protected:
    const char* bindingPattern() const override { return kArticulationPattern; }
    ovphysx_tensor_type_t firstTensorType() const override { return OVPHYSX_TENSOR_ARTICULATION_JACOBIAN_F32; }
    ovphysx_tensor_type_t secondTensorType() const override { return OVPHYSX_TENSOR_ARTICULATION_MASS_MATRIX_F32; }
    const char* firstName() const override { return "jacobian"; }
    const char* secondName() const override { return "mass matrix"; }
    ovphysx_sim_object_type_t objectType() const override { return OVPHYSX_OBJECT_ARTICULATION; }

    // Set for the READ side this class inherits and warms up with, as the other binding lanes do.
    const ovx_string_or_token_t* readAttrs(size_t& count) const override
    {
        return articulationInverseDynamicsReadAttrs(count);
    }
};

#define DEFINE_ARTI_INVERSE_DYNAMICS(N, device, gpu, hide)                                                             \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationInverseDynamicsReadOnlyBase, ArticulationInverseDynamics,                   \
                               readonly_arti_inverse_dynamics, N, device, gpu, hide)

#define DEFINE_ARTI_INVERSE_DYNAMICS_BINDING(N, device, gpu, hide)                                                     \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationInverseDynamicsBindingReadBase, ArticulationInverseDynamicsBindingRead,     \
                               tensorbinding_arti_inverse_dynamics, N, device, gpu, hide)

#define DEFINE_ARTI_INVERSE_DYNAMICS_MIXED(N, device, gpu, hide)                                                       \
    DEFINE_OUTPUT_READ_VARIANT(ArticulationInverseDynamicsMixedBase, ArticulationInverseDynamicsMixed,                 \
                               readonly_arti_inverse_dynamics_mixed, N, device, gpu, hide)

#define DEFINE_ARTI_ROOT_READ_ONLY(N, device, gpu, hide)                                                               \
    DEFINE_OUTPUT_READ_VARIANT(                                                                                        \
        ArticulationRootReadOnlyBase, ArticulationRootReadOnly, readonly_arti_root, N, device, gpu, hide)

#define DEFINE_ARTI_ROOT_BINDING(N, device, gpu, hide)                                                                 \
    DEFINE_OUTPUT_READ_VARIANT(                                                                                        \
        ArticulationRootBindingReadBase, ArticulationRootBindingRead, tensorbinding_arti_root, N, device, gpu, hide)

#define DEFINE_VOLUME_DEFORMABLE_READ_ONLY(N, device, gpu, hide)             \
    DEFINE_OUTPUT_READ_VARIANT(VolumeDeformableReadOnlyBase, VolumeDeformableReadOnly, readonly_deformable_vol, N, device, gpu, hide)

#define DEFINE_SURFACE_DEFORMABLE_READ_ONLY(N, device, gpu, hide)            \
    DEFINE_OUTPUT_READ_VARIANT(SurfaceDeformableReadOnlyBase, SurfaceDeformableReadOnly, readonly_deformable_surf, N, device, gpu, hide)

#define DEFINE_PARTICLE_READ_ONLY(N, device, gpu, hide)                      \
    DEFINE_OUTPUT_READ_VARIANT(ParticleReadOnlyBase, ParticleReadOnly, readonly_particles, N, device, gpu, hide)


#define DEFINE_VOLUME_DEFORMABLE_BINDING(N, device, gpu, hide)               \
    DEFINE_OUTPUT_READ_VARIANT(VolumeDeformableBindingReadBase, VolumeDeformableBindingRead, tensorbinding_deformable_vol, N, device, gpu, hide)

#define DEFINE_SURFACE_DEFORMABLE_BINDING(N, device, gpu, hide)              \
    DEFINE_OUTPUT_READ_VARIANT(SurfaceDeformableBindingReadBase, SurfaceDeformableBindingRead, tensorbinding_deformable_surf, N, device, gpu, hide)


#define DEFINE_VEHICLE_WHEEL_READ_ONLY(N, device, gpu, hide)                 \
    DEFINE_OUTPUT_READ_VARIANT(VehicleWheelReadOnlyBase, VehicleWheelReadOnly, readonly_vehicle_wheel, N, device, gpu, hide)
// The rigid controls. queryread against readonly gives the per-frame query cost, readonly against
// tensorbinding gives the cross-API ratio, and tensorcreate prices the setup the binding amortises.
DEFINE_QUERY_READ(1024, cpu, false, false)
DEFINE_QUERY_READ(8192, cpu, false, false)
DEFINE_QUERY_READ(1024, gpu, true, true)
DEFINE_QUERY_READ(8192, gpu, true, true)

DEFINE_READ_ONLY(8192, cpu, false, false)
DEFINE_READ_ONLY(8192, gpu, true, true)

DEFINE_TENSOR_BINDING_READ(8192, cpu, false, false)
DEFINE_TENSOR_BINDING_READ(8192, gpu, true, true)

DEFINE_TENSOR_BINDING_CREATE(8192, cpu, false, false)
DEFINE_TENSOR_BINDING_CREATE(8192, gpu, true, true)

// One read-and-binding pair per articulation granularity: per link, per DOF, per articulation.
// Held-query on both sides, so neither is charged for resolving a selector the other resolves once.
// 8192 only: the ratio is what these are for, and it does not need a second size to be read.
DEFINE_ARTI_LINK_READ_ONLY(8192, cpu, false, false)
DEFINE_ARTI_LINK_READ_ONLY(8192, gpu, true, true)
DEFINE_ARTI_LINK_BINDING(8192, cpu, false, false)
DEFINE_ARTI_LINK_BINDING(8192, gpu, true, true)

DEFINE_ARTI_DOF_READ_ONLY(8192, cpu, false, false)
DEFINE_ARTI_DOF_READ_ONLY(8192, gpu, true, true)
DEFINE_ARTI_DOF_BINDING(8192, cpu, false, false)
DEFINE_ARTI_DOF_BINDING(8192, gpu, true, true)

DEFINE_ARTI_ROOT_READ_ONLY(8192, cpu, false, false)
DEFINE_ARTI_ROOT_READ_ONLY(8192, gpu, true, true)
DEFINE_ARTI_ROOT_BINDING(8192, cpu, false, false)
DEFINE_ARTI_ROOT_BINDING(8192, gpu, true, true)

// The inverse dynamics columns, whose width comes from each articulation's topology. The single-cohort
// lanes are the denominator: readonly_arti_inverse_dynamics_mixed reads sixteen distinct topologies, so
// against them it prices fragmenting one read into per-cohort groups. Sixteen is the asset's own
// articulation count, not a size knob.
DEFINE_ARTI_INVERSE_DYNAMICS(8192, cpu, false, false)
DEFINE_ARTI_INVERSE_DYNAMICS(8192, gpu, true, true)
DEFINE_ARTI_INVERSE_DYNAMICS_BINDING(8192, cpu, false, false)
DEFINE_ARTI_INVERSE_DYNAMICS_BINDING(8192, gpu, true, true)
DEFINE_ARTI_INVERSE_DYNAMICS_MIXED(16, cpu, false, false)
DEFINE_ARTI_INVERSE_DYNAMICS_MIXED(16, gpu, true, true)

// Vehicles: CPU only and un-cloned, for the reasons on VehicleWheelReadOnlyBase. No binding lane to
// face, since the tensor API has no vehicle view.
DEFINE_VEHICLE_WHEEL_READ_ONLY(1024, cpu, false, false)

// Particles and deformables: GPU only, the mirror of the vehicle case. The read refuses a
// deformable without a CUDA context and the binding refuses the tensor outside DirectGPU, so a
// _cpu lane would report nothing on either side. A particle set cannot exist without a CUDA
// context at all, and the binding serves no particle tensors, so that lane has no twin.
//
// 128 sets, not 512: a 512-set particle scene exhausts a 16 GB card.
DEFINE_PARTICLE_READ_ONLY(128, gpu, true, true)

DEFINE_VOLUME_DEFORMABLE_READ_ONLY(128, gpu, true, true)
DEFINE_SURFACE_DEFORMABLE_READ_ONLY(128, gpu, true, true)
DEFINE_VOLUME_DEFORMABLE_BINDING(128, gpu, true, true)
DEFINE_SURFACE_DEFORMABLE_BINDING(128, gpu, true, true)

#undef DEFINE_SURFACE_DEFORMABLE_BINDING
#undef DEFINE_VOLUME_DEFORMABLE_BINDING
#undef DEFINE_SURFACE_DEFORMABLE_READ_ONLY
#undef DEFINE_VOLUME_DEFORMABLE_READ_ONLY
#undef DEFINE_PARTICLE_READ_ONLY
#undef DEFINE_VEHICLE_WHEEL_READ_ONLY
#undef DEFINE_ARTI_INVERSE_DYNAMICS_MIXED
#undef DEFINE_ARTI_INVERSE_DYNAMICS_BINDING
#undef DEFINE_ARTI_INVERSE_DYNAMICS
#undef DEFINE_ARTI_ROOT_BINDING
#undef DEFINE_ARTI_ROOT_READ_ONLY
#undef DEFINE_ARTI_DOF_BINDING
#undef DEFINE_ARTI_DOF_READ_ONLY
#undef DEFINE_ARTI_LINK_BINDING
#undef DEFINE_ARTI_LINK_READ_ONLY
#undef DEFINE_TENSOR_BINDING_CREATE
#undef DEFINE_TENSOR_BINDING_READ
#undef DEFINE_READ_ONLY
#undef DEFINE_QUERY_READ
#undef DEFINE_OUTPUT_READ_VARIANT


} // namespace
