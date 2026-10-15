// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-004
 * @covers AC-2 AC-3 AC-4
 */

// Steady-state contact-sensor net-force read through the public session read API
// (replacing the old raw-contact benchmark rows). Raw event/detail consumers
// migrate through the session contact-event/detail attributes (ADR-0044), not
// these two aggregated net-force columns.
// Each measured sample times ovphysx_query(OVPHYSX_QUERY_TARGET_CONTACT_SENSOR) +
// ovphysx_read(netNormalContactForce, netFrictionContactForce) + drain over 512
// resting contact-sensor bodies. The simulation step itself is advanced in
// preStep(), outside the timed region, so the row measures the read path only.
// The CPU row reads host-backed groups; the GPU row runs under DirectGPU, where
// the net-force columns are device-resident -- modeled on OutputRead's session
// read/drain lane.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "../TaskCompletion.h"
#include "AuthoringCommon.h"

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
#include <cuda_runtime_api.h>
#endif

#include "ovphysx/dlpack/dlpack.h"
#include <ovphysx/ovphysx_attributes.h>
#include <ovphysx/experimental/ovphysx.hpp>

#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>


void initContactReport()
{
}


namespace
{

const uint32_t kSensorCount = 512;
const uint32_t kMeasuredSteps = 20;
const uint32_t kMeasuredRuns = 5;
const uint32_t kMaximumWarmupSteps = 48;
const uint32_t kStableReadsRequired = 3;
const float kStepDt = 1.0f / 60.0f;
const double kCellPitch = 2.0;
const double kBoxHalfExtent = 0.25;
// A resting unit-mass box under gravity carries a net normal force of ~9.81 N, so
// a warm-up read whose host net-force column stays below this is not generating
// contacts (dead scene / broken reduction).
const float kLiveNormalForceThreshold = 1.0f;
const uint32_t kPinnedGpuRigidContactCapacity = 524288;
const uint32_t kPinnedGpuRigidPatchCapacity = 81920;
const uint32_t kPinnedGpuFoundLostPairsCapacity = 262144;

const char* const kScenePrim = "def PhysicsScene \"PhysicsScene\"";
const char* const kSceneApi = "prepend apiSchemas = [\"PhysxSceneAPI\"]";
const char* const kCpuDynamics = "bool physxScene:enableGPUDynamics = false";
const char* const kCpuBroadphase = "uniform token physxScene:broadphaseType = \"MBP\"";
const char* const kGpuDynamics = "bool physxScene:enableGPUDynamics = true";
const char* const kGpuBroadphase = "uniform token physxScene:broadphaseType = \"GPU\"";
const char* const kSensorApis =
    "prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsCollisionAPI\", \"PhysicsMassAPI\", "
    "\"PhysxRigidBodyAPI\", \"PhysxContactReportAPI\"]";
const char* const kReportThreshold = "float physxContactReport:threshold = 0";
const char* const kSleepThreshold = "float physxRigidBody:sleepThreshold = 0";


// 512 dynamic boxes resting on a static ground slab, spaced so they never touch a
// neighbour. Each box carries PhysxContactReportAPI and sleepThreshold = 0 so a
// settled body keeps reporting its steady net contact force; the ground has no
// schema and is not a sensor. Under gravity each box is one contact sensor whose
// net normal force resists its weight.
std::string makeScene(bool gpu)
{
    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(kSensorCount))));
    const double span = static_cast<double>(side) * kCellPitch + 4.0;

    std::ostringstream stream;
    stream << "#usda 1.0\n"
              "(\n"
              "    defaultPrim = \"World\"\n"
              "    metersPerUnit = 1\n"
              "    kilogramsPerUnit = 1\n"
              "    upAxis = \"Y\"\n"
              ")\n\n"
              "def Xform \"World\"\n"
              "{\n"
              "    "
           << kScenePrim << " (\n"
           << "        " << kSceneApi << "\n"
           << "    )\n"
              "    {\n"
              "        vector3f physics:gravityDirection = (0, -1, 0)\n"
              "        float physics:gravityMagnitude = 9.81\n"
              "        uint physxScene:timeStepsPerSecond = 60\n"
           << "        " << (gpu ? kGpuDynamics : kCpuDynamics) << "\n"
           << "        " << (gpu ? kGpuBroadphase : kCpuBroadphase) << "\n";
    if (gpu)
    {
        stream << "        uint physxScene:gpuMaxRigidContactCount = " << kPinnedGpuRigidContactCapacity << "\n"
               << "        uint physxScene:gpuMaxRigidPatchCount = " << kPinnedGpuRigidPatchCapacity << "\n"
               << "        uint physxScene:gpuFoundLostPairsCapacity = " << kPinnedGpuFoundLostPairsCapacity << "\n";
    }
    stream << "    }\n\n";

    // Static ground slab: a thin, wide static collider centred under the grid,
    // top surface at y = 0. No physics body, no contact-report schema.
    stream << "    def Cube \"Ground\" (\n"
           << "        prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n"
           << "    )\n"
           << "    {\n"
           << "        double size = 1\n"
           << "        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n"
           << "        bool physics:collisionEnabled = true\n"
           << "        double3 xformOp:translate = (" << span * 0.5 << ", -0.5, " << span * 0.5 << ")\n"
           << "        float3 xformOp:scale = (" << span << ", 1, " << span << ")\n"
           << "        uniform token[] xformOpOrder = [\"xformOp:translate\", \"xformOp:scale\"]\n"
           << "    }\n\n";

    for (uint32_t index = 0; index < kSensorCount; ++index)
    {
        const uint32_t row = index / side;
        const uint32_t column = index % side;
        const double x = 2.0 + static_cast<double>(column) * kCellPitch;
        const double z = 2.0 + static_cast<double>(row) * kCellPitch;

        stream << "    def Cube \"Sensor_" << std::setw(6) << std::setfill('0') << index << "\" (\n"
               << "        " << kSensorApis << "\n"
               << "    )\n"
               << "    {\n"
               << "        double size = " << (kBoxHalfExtent * 2.0) << "\n"
               << "        float3[] extent = [(" << -kBoxHalfExtent << ", " << -kBoxHalfExtent << ", "
               << -kBoxHalfExtent << "), (" << kBoxHalfExtent << ", " << kBoxHalfExtent << ", " << kBoxHalfExtent
               << ")]\n"
               << "        bool physics:collisionEnabled = true\n"
               << "        bool physics:rigidBodyEnabled = true\n"
               << "        float physics:mass = 1\n"
               << "        " << kReportThreshold << "\n"
               << "        " << kSleepThreshold << "\n"
               << "        double3 xformOp:translate = (" << x << ", " << kBoxHalfExtent << ", " << z << ")\n"
               << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
               << "    }\n\n";
    }
    stream << "}\n";
    return stream.str();
}


class ContactReportBase : public BmBenchmark
{
public:
    ContactReportBase(const char* row, bool gpu) : mRow(row), mGpu(gpu)
    {
    }

    ~ContactReportBase() override
    {
        teardown();
    }

    bool isValid() const override
    {
        const BmGlobals& globals = BmGlobals::getInstance();
        if (globals.getPhysX() == nullptr)
        {
            return false;
        }
        if (!mGpu)
        {
            // The CPU row runs only on the default (host) pass.
            return !globals.forceGpu();
        }
#if !defined(OVPHYSX_BENCHMARK_HAS_CUDA)
        // Without CUDA a DirectGPU net-force read cannot be consumed; skip.
        return false;
#else
        // On a DirectGPU scene the net-force columns are device-resident, so the
        // GPU row requires --forceGpu AND --directGpu; otherwise it is selected
        // and skipped without publishing a metric.
        return globals.forceGpu() && globals.directGpu();
#endif
    }

    uint32_t getNbSteps() const override
    {
        return kMeasuredSteps;
    }

    uint32_t getNbRuns() const override
    {
        return kMeasuredRuns;
    }

    void startRun() override
    {
        if (mInitializationAttempted)
        {
            return;
        }
        mInitializationAttempted = true;
        setup();
    }

    void preStep() override
    {
        // Advance the simulation outside the timed region so the row measures the
        // contact-sensor read, not the step.
        if (!mSetupOk || !mStepOk)
        {
            return;
        }
        if (bmCompleteTask(handle(), mPhysX->step(kStepDt)).status != OVPHYSX_API_SUCCESS)
        {
            bmRecordFailure(mRow, "ovphysx_step failed before contact-sensor read");
            mStepOk = false;
        }
    }

    void endRun() override
    {
    }

protected:
    void step() override
    {
        // The timed operation: one query + read + drain of the net contact forces.
        if (!mSetupOk || !mStepOk)
        {
            return;
        }
        // A measured read treats empty or incomplete contact output as a failure;
        // record its deferred reason so the harness suppresses this row. The
        // timed path does not touch force values (warm-up already validated
        // the row count and device), so the measured number is a pure read latency.
        if (!readNetContactForces(/*checkLive=*/false))
        {
            if (!bmRowHasFailure(mRow))
            {
                bmRecordFailure(mRow, "%s", mLastReason.c_str());
            }
            mStepOk = false;
        }
    }

private:
    ovphysx_instance_handle_t handle() const
    {
        return mPhysX->handle();
    }

    void setup()
    {
        mSetupOk = false;
        mStepOk = false;
        mPhysX = BmGlobals::getInstance().getPhysX();
        if (!authoringbm::prepareStageAttachmentForRun(mAttachment, mRow, &mLeftoverAttachmentReported))
        {
            return;
        }
        mAttached = false;
        if (!mPhysX)
        {
            bmRecordFailure(mRow, "no ovphysx instance available");
            return;
        }

        const std::string usda = makeScene(mGpu);
        if (!authoringbm::populateAndAttachChecked(mPhysX, authoringbm::UsdSource::eString, usda,
                                                   "ovphysx-contact-report", mAttachment, mRow, &mAttached))
        {
            return;
        }
        if (!stabilize())
        {
            return;
        }
        mSetupOk = true;
        mStepOk = true;
    }

    void teardown()
    {
        if (!mInitializationAttempted)
        {
            return;
        }
        mSetupOk = false;
        mStepOk = false;
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mAttachment, mRow))
        {
            bmRecordFailure(mRow, "ovstage teardown failed; retaining the stage attachment");
            mLeftoverAttachmentReported = true;
        }
        else
        {
            mAttached = false;
        }
    }

    // Require three consecutive warm-up reads with the full 512-sensor set on the
    // expected device and, on a host read, a live net normal force. DirectGPU
    // force values are not checked. Runs outside any measured timing.
    bool stabilize()
    {
        uint32_t stableReads = 0;
        for (uint32_t warmup = 1; warmup <= kMaximumWarmupSteps; ++warmup)
        {
            if (bmCompleteTask(handle(), mPhysX->step(kStepDt)).status != OVPHYSX_API_SUCCESS)
            {
                bmRecordFailure(mRow, "ovphysx_step failed during contact-sensor warmup");
                return false;
            }
            if (readNetContactForces(/*checkLive=*/true))
            {
                if (++stableReads >= kStableReadsRequired)
                {
                    printFormatted("%s: warmup stabilized after %u steps (%u sensors)", mRow, warmup, kSensorCount);
                    return true;
                }
            }
            else
            {
                // Task, synchronization, device and cleanup failures are already
                // in the ledger. Successful reads with an incomplete or not-yet-live
                // contact set may be retried within the warm-up bound.
                if (bmRowHasFailure(mRow))
                {
                    return false;
                }
                stableReads = 0;
            }
        }
        bmRecordFailure(mRow,
                        "contact-sensor read did not reach %u consecutive complete reads within %u warmup steps "
                        "(last: %s)",
                        kStableReadsRequired, kMaximumWarmupSteps,
                        mLastReason.empty() ? "n/a" : mLastReason.c_str());
        return false;
    }

    // One query(CONTACT_SENSOR) + read(net normal, net friction) + full drain.
    // Operational failures immediately enter the row ledger. Contact readiness
    // misses return false with a deferred reason for the caller's retry policy.
    bool readNetContactForces(bool checkLive)
    {
        ovphysx_query_handle_t query = 0;
        if (bmCompleteTask(handle(), ovphysx_query(handle(), OVPHYSX_QUERY_TARGET_CONTACT_SENSOR, OVPHYSX_SCOPE_ALL,
                                                   &query, nullptr))
                    .status != OVPHYSX_API_SUCCESS ||
            query == 0)
        {
            return fail("ovphysx_query(CONTACT_SENSOR) failed");
        }

        static const ovx_string_or_token_t attrs[] = {
            { 0, { OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE, sizeof(OVPHYSX_ATTR_NET_NORMAL_CONTACT_FORCE) - 1 } },
            { 0, { OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE, sizeof(OVPHYSX_ATTR_NET_FRICTION_CONTACT_FORCE) - 1 } },
        };
        ovphysx_read_handle_t read = 0;
        const bool readOk =
            bmCompleteTask(handle(), ovphysx_read(handle(), query, attrs, sizeof(attrs) / sizeof(attrs[0]), &read,
                                                  nullptr))
                    .status == OVPHYSX_API_SUCCESS &&
            read != 0;

        bool ok = readOk ? drain(read, checkLive) : fail("ovphysx_read(netNormal/netFriction contact force) failed");

        // Always release in order (read then query); a release failure is a
        // fail-closed condition, so fold its status into the result.
        if (read != 0 &&
            bmCompleteTask(handle(), ovphysx_release_read(handle(), read)).status != OVPHYSX_API_SUCCESS)
        {
            ok = fail("ovphysx_release_read failed");
        }
        if (bmCompleteTask(handle(), ovphysx_release_query(handle(), query)).status != OVPHYSX_API_SUCCESS)
        {
            ok = fail("ovphysx_release_query failed");
        }
        return ok;
    }

    bool drain(ovphysx_read_handle_t read, bool checkLive)
    {
        const ovstage_read_group_t* group = nullptr;
        uint32_t groups = 0;
        int64_t groupRows = -1;
        bool rowsConsistent = true;
        float maxHostForce = 0.0f;
        bool sawHostForceColumn = false;
        bool drained = false;
        for (;;)
        {
            const ovphysx_api_status_t status =
                bmCompleteTask(handle(), ovphysx_fetch_read_next(handle(), read, &group)).status;
            if (status == OVPHYSX_API_SUCCESS && group == nullptr)
            {
                drained = true;
                break;
            }
            if (status != OVPHYSX_API_SUCCESS || group == nullptr)
            {
                return fail("ovphysx_fetch_read_next failed after %u groups", groups);
            }
            const DLTensor* tensor =
                (group->data.tensors && group->data.tensor_count > 0) ? &group->data.tensors[0] : nullptr;
            if (tensor && tensor->device.device_type != (mGpu ? kDLCUDA : kDLCPU))
            {
                return fail("expected %s contact-sensor tensor, got device type %d", mGpu ? "CUDA" : "CPU",
                            static_cast<int>(tensor->device.device_type));
            }
            if (tensor && tensor->shape)
            {
                // Every net-force group spans the full sensor set; require each to
                // carry exactly 512 rows, not merely the largest.
                const int64_t rows = tensor->shape[0];
                if (groupRows < 0)
                {
                    groupRows = rows;
                }
                else if (rows != groupRows)
                {
                    rowsConsistent = false;
                }
                // Warm-up only: check physical liveness on a host read, which
                // exposes the net-force column directly; a resting sensor's net
                // normal force is ~ its weight, so an all-zero column means no
                // contacts are being generated. Device (DirectGPU) columns are
                // checked for CUDA residency and row count, not force values,
                // and a strided column is skipped rather than walked unsafely.
                if (checkLive && tensor->data && tensor->strides == nullptr &&
                    tensor->device.device_type == kDLCPU &&
                    tensor->dtype.code == static_cast<uint8_t>(kDLFloat) && tensor->dtype.bits == 32)
                {
                    sawHostForceColumn = true;
                    size_t elems = tensor->dtype.lanes ? tensor->dtype.lanes : 1;
                    for (int32_t d = 0; d < tensor->ndim; ++d)
                    {
                        elems *= static_cast<size_t>(tensor->shape[d]);
                    }
                    const float* data =
                        reinterpret_cast<const float*>(static_cast<const char*>(tensor->data) + tensor->byte_offset);
                    for (size_t i = 0; i < elems; ++i)
                    {
                        // Plain comparison rather than std::max: on MSVC the USD
                        // precompiled header drags in the windows.h max() macro,
                        // which would mangle a std::max( call site.
                        const float mag = std::fabs(data[i]);
                        if (mag > maxHostForce)
                        {
                            maxHostForce = mag;
                        }
                    }
                }
            }
            // On a DirectGPU scene the net-force columns are device-resident; honor
            // the per-group CUDA sync before releasing. No-op on CPU / non-CUDA.
            if (!waitForGroup(*group))
            {
                return fail("cuda_sync wait failed after %u groups", groups);
            }
            ++groups;
            const ovstage_read_group_id_t groupId = group->read_group_id;
            group = nullptr;
            if (bmCompleteTask(handle(), ovphysx_release_group(handle(), read, groupId)).status != OVPHYSX_API_SUCCESS)
            {
                return fail("ovphysx_release_group failed after %u groups", groups);
            }
        }

        if (!drained || groups == 0)
        {
            return warmupMiss("contact-sensor read drained no groups");
        }
        if (groupRows != static_cast<int64_t>(kSensorCount) || !rowsConsistent)
        {
            return warmupMiss("expected %u contact-sensor rows in every group, got %lld (consistent=%d)", kSensorCount,
                              static_cast<long long>(groupRows), static_cast<int>(rowsConsistent));
        }
        if (checkLive && sawHostForceColumn && maxHostForce < kLiveNormalForceThreshold)
        {
            return warmupMiss("no contact sensor reported a live net force (max %.4g N); scene not generating contacts",
                              static_cast<double>(maxHostForce));
        }
        return true;
    }

#if defined(OVPHYSX_BENCHMARK_HAS_CUDA)
    bool waitForGroup(const ovstage_read_group_t& group)
    {
        if (group.data.cuda_sync.wait_event != 0)
        {
            return cudaEventSynchronize(reinterpret_cast<cudaEvent_t>(group.data.cuda_sync.wait_event)) == cudaSuccess;
        }
        if (group.data.cuda_sync.stream != 0)
        {
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

    // Operational failures are fatal even during warmup. Cleanup may retire the
    // session despite returning an error, so teardown cannot recover this signal.
    bool fail(const char* fmt, ...)
    {
        char buffer[256];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        mLastReason.assign(buffer);
        bmRecordFailure(mRow, "%s", buffer);
        return false;
    }

    // A successful read can observe an empty, incomplete or not-yet-live contact
    // set. Stabilization retries these misses within its existing step bound;
    // a measured read records empty or incomplete output as a failure.
    bool warmupMiss(const char* fmt, ...)
    {
        char buffer[256];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buffer, sizeof(buffer), fmt, args);
        va_end(args);
        mLastReason.assign(buffer);
        return false;
    }

    const char* mRow = nullptr;
    bool mGpu = false;
    ovphysx::PhysX* mPhysX = nullptr;
    ovphysx_sample_stage_attachment_t mAttachment{};
    bool mAttached = false;
    bool mLeftoverAttachmentReported = false;
    bool mInitializationAttempted = false;
    bool mSetupOk = false;
    bool mStepOk = false;
    std::string mLastReason;
};


struct ContactReportNetForce512Cpu : ContactReportBase
{
    ContactReportNetForce512Cpu() : ContactReportBase("ContactReport.net_force_512_read_cpu", false)
    {
    }
};


struct ContactReportNetForce512Gpu : ContactReportBase
{
    ContactReportNetForce512Gpu() : ContactReportBase("ContactReport.net_force_512_read_gpu", true)
    {
    }
};


Register<ContactReportNetForce512Cpu, true> sContactReportNetForce512Cpu("ContactReport.net_force_512_read_cpu");
Register<ContactReportNetForce512Gpu, true> sContactReportNetForce512Gpu("ContactReport.net_force_512_read_gpu");

} // namespace
