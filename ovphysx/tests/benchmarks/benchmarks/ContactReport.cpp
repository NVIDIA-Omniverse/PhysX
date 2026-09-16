// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-004
 * @covers AC-2 AC-3 AC-4
 */

// Steady-state raw contact-report performance for persistent dynamic/static pairs.

#include "framework/UsdPCH.h"

#include "../framework/BmBenchmark.h"
#include "../framework/BmGlobals.h"
#include "../framework/BmUtils.h"
#include "../BenchmarkFailure.h"
#include "AuthoringCommon.h"

#include <ovphysx/experimental/ovphysx.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>


void initContactReport()
{
}


namespace
{

const uint32_t kPairCount = 512;
const uint32_t kMeasuredSteps = 20;
const uint32_t kMeasuredRuns = 5;
const uint32_t kMaximumWarmupSteps = 32;
const uint32_t kStableReportsRequired = 3;
const float kStepDt = 1.0f / 60.0f;
const double kCellPitch = 4.0;
const double kPairOffset = 0.75;
const uint32_t kPinnedGpuRigidContactCapacity = 524288;
const uint32_t kPinnedGpuRigidPatchCapacity = 81920;
const uint32_t kPinnedGpuFoundLostPairsCapacity = 262144;
// ovphysx_contact_event_header_t documents these ABI values but does not expose
// a public named enum for them.
const int32_t kContactEventFound = 0;
const int32_t kContactEventPersist = 2;

const char* const kScenePrim = "def PhysicsScene \"PhysicsScene\"";
const char* const kSceneApi = "prepend apiSchemas = [\"PhysxSceneAPI\"]";
const char* const kSleepingOff = "uniform bool physxScene:disableSleeping = true";
const char* const kCpuDynamics = "bool physxScene:enableGPUDynamics = false";
const char* const kCpuBroadphase = "uniform token physxScene:broadphaseType = \"MBP\"";
const char* const kGpuDynamics = "bool physxScene:enableGPUDynamics = true";
const char* const kGpuBroadphase = "uniform token physxScene:broadphaseType = \"GPU\"";
const char* const kReporterApis =
    "prepend apiSchemas = [\"PhysicsRigidBodyAPI\", \"PhysicsCollisionAPI\", \"PhysicsMassAPI\", "
    "\"PhysxRigidBodyAPI\", \"PhysxContactReportAPI\"]";
const char* const kReportThreshold = "float physxContactReport:threshold = 0";
const char* const kKinematicOff = "bool physics:kinematicEnabled = false";
const char* const kSleepThreshold = "float physxRigidBody:sleepThreshold = 0";
const char* const kSolveContactOff = "bool physxRigidBody:solveContact = false";


std::string makeScene(bool gpu)
{
    const uint32_t side = static_cast<uint32_t>(std::ceil(std::sqrt(static_cast<double>(kPairCount))));

    std::ostringstream stream;
    stream << "#usda 1.0\n"
              "(\n"
              "    defaultPrim = \"World\"\n"
              "    metersPerUnit = 1\n"
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
              "        float physics:gravityMagnitude = 0\n"
              "        uint physxScene:timeStepsPerSecond = 60\n"
           << "        " << kSleepingOff << "\n"
           << "        " << (gpu ? kGpuDynamics : kCpuDynamics) << "\n"
           << "        " << (gpu ? kGpuBroadphase : kCpuBroadphase) << "\n";
    if (gpu)
    {
        // Pin the exact PhysxSceneAPI schema defaults. This fixture must not
        // scale or shrink these global GPU buffers with its pair count.
        stream << "        uint physxScene:gpuMaxRigidContactCount = " << kPinnedGpuRigidContactCapacity << "\n"
               << "        uint physxScene:gpuMaxRigidPatchCount = " << kPinnedGpuRigidPatchCapacity << "\n"
               << "        uint physxScene:gpuFoundLostPairsCapacity = " << kPinnedGpuFoundLostPairsCapacity << "\n";
    }
    stream << "    }\n\n";

    for (uint32_t index = 0; index < kPairCount; ++index)
    {
        const uint32_t row = index / side;
        const uint32_t column = index % side;
        const double x = static_cast<double>(column) * kCellPitch;
        const double z = static_cast<double>(row) * kCellPitch;

        stream << "    def Sphere \"Reporter_" << std::setw(6) << std::setfill('0') << index << "\" (\n"
               << "        " << kReporterApis << "\n"
               << "    )\n"
               << "    {\n"
               << "        double radius = 0.5\n"
               << "        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n"
               << "        bool physics:collisionEnabled = true\n"
               << "        bool physics:rigidBodyEnabled = true\n"
               << "        " << kKinematicOff << "\n"
               << "        float physics:mass = 1\n"
               << "        vector3f physics:velocity = (0, 0, 0)\n"
               << "        vector3f physics:angularVelocity = (0, 0, 0)\n"
               << "        " << kReportThreshold << "\n"
               << "        " << kSleepThreshold << "\n"
               << "        " << kSolveContactOff << "\n"
               << "        double3 xformOp:translate = (" << x << ", 0, " << z << ")\n"
               << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
               << "    }\n\n"
               << "    def Sphere \"Static_" << std::setw(6) << std::setfill('0') << index << "\" (\n"
               << "        prepend apiSchemas = [\"PhysicsCollisionAPI\"]\n"
               << "    )\n"
               << "    {\n"
               << "        double radius = 0.5\n"
               << "        float3[] extent = [(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)]\n"
               << "        bool physics:collisionEnabled = true\n"
               << "        double3 xformOp:translate = (" << x + kPairOffset << ", 0, " << z << ")\n"
               << "        uniform token[] xformOpOrder = [\"xformOp:translate\"]\n"
               << "    }\n\n";
    }
    stream << "}\n";
    return stream.str();
}


struct PairId
{
    uint64_t low = 0;
    uint64_t high = 0;

    bool operator==(const PairId& other) const
    {
        return low == other.low && high == other.high;
    }

    bool operator<(const PairId& other) const
    {
        return low < other.low || (low == other.low && high < other.high);
    }
};


struct ReportView
{
    const ovphysx_contact_event_header_t* headers = nullptr;
    uint32_t numHeaders = 0;
    const ovphysx_contact_point_t* points = nullptr;
    uint32_t numPoints = 0;
    const ovphysx_friction_anchor_t* anchors = nullptr;
    uint32_t numAnchors = 0;
};


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
        if (globals.directGpu())
        {
            // BmBenchmark fixes this override as const, so the process-wide
            // failure ledger records the invalid invocation, not object state.
            bmRecordFailure(mRow, "ContactReport rows reject DirectGPU; run the GPU row with --forceGpu only");
            return false;
        }
        if (globals.forceGpu() != mGpu)
        {
            return false;
        }
        return globals.getPhysX() != nullptr;
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
    }

    Time::Second timedStep() override
    {
        if (!mSetupOk || !mStepOk)
        {
            // A prior bmRecordFailure() makes the harness discard every timing
            // for this row and exit nonzero, so this value is never published.
            return 0.0f;
        }

        ReportView report;
        Time timer;
        const ovphysx_result_t stepResult = ovphysx_step_sync(mPhysX->handle(), kStepDt);
        if (stepResult.status != OVPHYSX_API_SUCCESS)
        {
            const Time::Second elapsed = timer.getElapsedSeconds();
            bmRecordFailure(mRow, "ovphysx_step_sync failed: status=%d", static_cast<int>(stepResult.status));
            mStepOk = false;
            return elapsed;
        }
        const ovphysx_result_t reportResult =
            ovphysx_get_contact_report(mPhysX->handle(), &report.headers, &report.numHeaders, &report.points,
                                       &report.numPoints, &report.anchors, &report.numAnchors);
        const Time::Second elapsed = timer.getElapsedSeconds();

        if (reportResult.status != OVPHYSX_API_SUCCESS)
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            bmRecordFailure(mRow, "ovphysx_get_contact_report failed: status=%d %.*s",
                            static_cast<int>(reportResult.status), static_cast<int>(error.length),
                            error.ptr ? error.ptr : "");
            mStepOk = false;
            return elapsed;
        }

        bool allPersist = false;
        if (!validateReport(report, allPersist))
        {
            mStepOk = false;
            return elapsed;
        }
        if (!allPersist)
        {
            bmRecordFailure(mRow, "measured report contains a non-persist event");
            mStepOk = false;
        }
        else if (!gpuEvidenceOk("measured step"))
        {
            mStepOk = false;
        }
        return elapsed;
    }

    void endRun() override
    {
        if (!gpuEvidenceOk("end of measured run"))
        {
            mStepOk = false;
        }
    }

protected:
    void step() override
    {
    }

private:
    void setup()
    {
        mSetupOk = false;
        mStepOk = false;
        mAttachHandle = 0;
        mIdentityResolved = false;
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

        // Destroy any prior owner before constructing a replacement: the log
        // API has one process-global callback slot.
        mGpuScope.reset();
        mGpuScope.reset(new authoringbm::GpuFallbackScope(mGpu, mRow));
        if (!mGpuScope->evidenceOk("detector setup"))
        {
            return;
        }
        const std::string usda = makeScene(mGpu);
        if (!authoringbm::populateAndAttachChecked(
                mPhysX, authoringbm::UsdSource::eString, usda, "ovphysx-contact-report", mAttachment, mRow, &mAttached))
        {
            return;
        }
        if (!warmupAndCheckDevice())
        {
            return;
        }
        if (ovphysx_get_attach_handle(mPhysX->handle(), &mAttachHandle).status != OVPHYSX_API_SUCCESS ||
            mAttachHandle == 0)
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            bmRecordFailure(mRow, "get_attach_handle failed or returned zero: %.*s", static_cast<int>(error.length),
                            error.ptr ? error.ptr : "");
            return;
        }
        if (!stabilizeReport())
        {
            return;
        }
        if (!gpuEvidenceOk("stabilization"))
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
        if (mGpuScope)
        {
            mGpuScope->evidenceOk("end of measured runs");
        }
        if (!authoringbm::clearOvstageChecked(mAttached ? mPhysX : nullptr, mAttachment, mRow))
        {
            bmRecordFailure(mRow, "ovstage teardown failed; retaining the stage attachment");
            mLeftoverAttachmentReported = true;
        }
        else
        {
            mAttached = false;
        }
        if (mGpuScope)
        {
            mGpuScope->finish("stage teardown");
            mGpuScope.reset();
        }
    }

    bool gpuEvidenceOk(const char* phase)
    {
        if (!mGpu)
        {
            return true;
        }
        if (!mGpuScope)
        {
            bmRecordFailure(mRow, "%s has no GPU fallback/capacity detector", phase);
            return false;
        }
        return mGpuScope->evidenceOk(phase);
    }

    bool warmupAndCheckDevice()
    {
        return authoringbm::checkDeviceEvidenceAndWarmup(mPhysX->handle(), mGpu, *mGpuScope, mRow);
    }

    // Split a resolved collider path into its Reporter/Static role and pair index.
    static bool parsePairPath(const std::string& path, bool& isReporter, uint32_t& index)
    {
        static const std::string kReporterPrefix = "/World/Reporter_";
        static const std::string kStaticPrefix = "/World/Static_";

        const std::string* prefix = nullptr;
        if (path.rfind(kReporterPrefix, 0) == 0)
        {
            isReporter = true;
            prefix = &kReporterPrefix;
        }
        else if (path.rfind(kStaticPrefix, 0) == 0)
        {
            isReporter = false;
            prefix = &kStaticPrefix;
        }
        else
        {
            return false;
        }

        const std::string digits = path.substr(prefix->size());
        if (digits.size() != 6 || digits.find_first_not_of("0123456789") != std::string::npos)
        {
            return false;
        }
        index = static_cast<uint32_t>(std::stoul(digits));
        return index < kPairCount;
    }

    // The header identity fields are opaque ObjectKey handles (ADR-0019), so the
    // expected set cannot be computed client-side from a path bit-cast. Resolve
    // the first stabilized report's handles to paths once, prove they exactly
    // cover the authored Reporter_N/Static_N pairs, and snapshot the handles so
    // every later report is checked by cheap integer comparison.
    bool resolveExpectedPairs(const ReportView& report)
    {
        if (report.numHeaders != kPairCount || !report.headers)
        {
            bmRecordFailure(mRow, "identity resolution expected %u headers, got %u%s", kPairCount, report.numHeaders,
                            report.headers ? "" : " (null header buffer)");
            return false;
        }

        std::array<uint64_t, static_cast<size_t>(kPairCount) * 2> ids{};
        for (uint32_t index = 0; index < kPairCount; ++index)
        {
            ids[static_cast<size_t>(index) * 2] = report.headers[index].collider0;
            ids[static_cast<size_t>(index) * 2 + 1] = report.headers[index].collider1;
        }

        std::array<ovphysx_string_t, static_cast<size_t>(kPairCount) * 2> paths{};
        uint32_t resolvedCount = 0;
        const ovphysx_result_t result =
            ovphysx_scene_query_get_paths_from_ids(mPhysX->handle(), ids.data(), static_cast<uint32_t>(ids.size()),
                                                   paths.data(), static_cast<uint32_t>(paths.size()), &resolvedCount);
        if (result.status != OVPHYSX_API_SUCCESS || resolvedCount != ids.size())
        {
            const ovphysx_string_t error = ovphysx_get_last_error();
            bmRecordFailure(mRow, "collider id resolution failed: status=%d count=%u %.*s",
                            static_cast<int>(result.status), resolvedCount, static_cast<int>(error.length),
                            error.ptr ? error.ptr : "");
            return false;
        }

        std::array<uint8_t, kPairCount> seen{};
        seen.fill(0);
        for (uint32_t index = 0; index < kPairCount; ++index)
        {
            uint32_t pairIndex[2] = { 0, 0 };
            bool isReporter[2] = { false, false };
            for (uint32_t side = 0; side < 2; ++side)
            {
                const ovphysx_string_t& entry = paths[static_cast<size_t>(index) * 2 + side];
                const std::string path(entry.ptr ? entry.ptr : "", entry.ptr ? entry.length : 0);
                if (!parsePairPath(path, isReporter[side], pairIndex[side]))
                {
                    bmRecordFailure(mRow, "header %u collider%u resolved to unexpected path '%s'", index, side,
                                    path.c_str());
                    return false;
                }
            }
            if (isReporter[0] == isReporter[1] || pairIndex[0] != pairIndex[1])
            {
                bmRecordFailure(mRow, "header %u is not a Reporter/Static pair with a shared index", index);
                return false;
            }
            if (seen[pairIndex[0]] != 0)
            {
                bmRecordFailure(mRow, "authored pair %u reported more than once", pairIndex[0]);
                return false;
            }
            seen[pairIndex[0]] = 1;

            const uint64_t collider0 = report.headers[index].collider0;
            const uint64_t collider1 = report.headers[index].collider1;
            if (collider0 == 0 || collider1 == 0 || collider0 == collider1)
            {
                bmRecordFailure(mRow, "invalid collider identity for authored pair %u", pairIndex[0]);
                return false;
            }
            // Parenthesized to stop the Windows min/max macros from expanding these.
            mExpectedPairs[index] = { (std::min)(collider0, collider1), (std::max)(collider0, collider1) };
        }

        std::sort(mExpectedPairs.begin(), mExpectedPairs.end());
        if (std::adjacent_find(mExpectedPairs.begin(), mExpectedPairs.end()) != mExpectedPairs.end())
        {
            bmRecordFailure(mRow, "expected Reporter/Static identities are not unique");
            return false;
        }
        mIdentityResolved = true;
        return true;
    }

    bool stabilizeReport()
    {
        uint32_t stableReports = 0;

        for (uint32_t warmup = 1; warmup <= kMaximumWarmupSteps; ++warmup)
        {
            ReportView report;
            const ovphysx_result_t stepResult = ovphysx_step_sync(mPhysX->handle(), kStepDt);
            if (stepResult.status != OVPHYSX_API_SUCCESS)
            {
                bmRecordFailure(
                    mRow, "warmup step failed at step %u: status=%d", warmup, static_cast<int>(stepResult.status));
                return false;
            }
            const ovphysx_result_t reportResult =
                ovphysx_get_contact_report(mPhysX->handle(), &report.headers, &report.numHeaders, &report.points,
                                           &report.numPoints, &report.anchors, &report.numAnchors);
            if (reportResult.status != OVPHYSX_API_SUCCESS)
            {
                bmRecordFailure(
                    mRow, "warmup getter failed at step %u: status=%d", warmup, static_cast<int>(reportResult.status));
                return false;
            }
            if (!gpuEvidenceOk("warmup step"))
            {
                return false;
            }
            if (!mIdentityResolved && !resolveExpectedPairs(report))
            {
                return false;
            }

            bool allPersist = false;
            if (!validateReport(report, allPersist))
            {
                return false;
            }

            if (allPersist)
            {
                ++stableReports;
                if (stableReports >= kStableReportsRequired)
                {
                    printFormatted("%s: warmup stabilized after %u steps (%u headers, %u points, %u anchors)", mRow,
                                   warmup, report.numHeaders, report.numPoints, report.numAnchors);
                    return true;
                }
            }
            else
            {
                stableReports = 0;
            }
        }

        bmRecordFailure(mRow, "contact report did not produce %u consecutive persistent reports within %u warmup steps",
                        kStableReportsRequired, kMaximumWarmupSteps);
        return false;
    }

    bool validateReport(const ReportView& report, bool& allPersist)
    {
        if (!mIdentityResolved)
        {
            bmRecordFailure(mRow, "report validated before the expected identities were resolved");
            return false;
        }
        if ((report.numHeaders != 0 && !report.headers) || (report.numPoints != 0 && !report.points) ||
            (report.numAnchors != 0 && !report.anchors))
        {
            bmRecordFailure(mRow, "contact getter returned a null buffer with a positive count");
            return false;
        }
        if (report.numHeaders != kPairCount || report.numPoints != kPairCount || report.numAnchors != 0)
        {
            bmRecordFailure(mRow, "unexpected cardinality: expected %u headers/points and zero anchors, got %u/%u/%u",
                            kPairCount, report.numHeaders, report.numPoints, report.numAnchors);
            return false;
        }

        mPointCoverage.fill(0);
        allPersist = true;

        for (uint32_t headerIndex = 0; headerIndex < report.numHeaders; ++headerIndex)
        {
            const ovphysx_contact_event_header_t& header = report.headers[headerIndex];
            if (header.type != kContactEventFound && header.type != kContactEventPersist)
            {
                bmRecordFailure(mRow, "invalid contact event type %d at header %u", header.type, headerIndex);
                return false;
            }
            allPersist = allPersist && header.type == kContactEventPersist;
            if (header.attachHandle != mAttachHandle || header.attachHandle == 0)
            {
                bmRecordFailure(mRow, "wrong attach handle at header %u: expected %llu, got %llu", headerIndex,
                                static_cast<unsigned long long>(mAttachHandle),
                                static_cast<unsigned long long>(header.attachHandle));
                return false;
            }
            if (header.actor0 == 0 || header.actor1 == 0 || header.collider0 == 0 || header.collider1 == 0 ||
                header.actor0 != header.collider0 || header.actor1 != header.collider1 ||
                header.actor0 == header.actor1 || header.protoIndex0 != (std::numeric_limits<uint32_t>::max)() ||
                header.protoIndex1 != (std::numeric_limits<uint32_t>::max)())
            {
                bmRecordFailure(mRow, "invalid actor/collider identity data at header %u", headerIndex);
                return false;
            }

            const PairId pair{ (std::min)(header.collider0, header.collider1),
                               (std::max)(header.collider0, header.collider1) };
            if (!std::binary_search(mExpectedPairs.begin(), mExpectedPairs.end(), pair))
            {
                bmRecordFailure(mRow, "unexpected Reporter/Static identity pair at header %u", headerIndex);
                return false;
            }
            mObservedPairs[headerIndex] = pair;

            if (header.numContactData != 1 || header.contactDataOffset > report.numPoints ||
                header.numContactData > report.numPoints - header.contactDataOffset ||
                header.contactDataOffset >= mPointCoverage.size())
            {
                bmRecordFailure(mRow, "invalid point slice at header %u: offset=%u count=%u total=%u", headerIndex,
                                header.contactDataOffset, header.numContactData, report.numPoints);
                return false;
            }
            if (header.frictionAnchorsDataOffset != 0 || header.numfrictionAnchorsData != 0)
            {
                bmRecordFailure(mRow, "unexpected friction anchors at header %u: offset=%u count=%u", headerIndex,
                                header.frictionAnchorsDataOffset, header.numfrictionAnchorsData);
                return false;
            }

            const uint32_t pointIndex = header.contactDataOffset;
            if (mPointCoverage[pointIndex] != 0)
            {
                bmRecordFailure(mRow, "overlapping point slices at point %u", pointIndex);
                return false;
            }
            mPointCoverage[pointIndex] = 1;

            const ovphysx_contact_point_t& point = report.points[pointIndex];
            for (uint32_t axis = 0; axis < 3; ++axis)
            {
                if (!std::isfinite(point.position[axis]) || !std::isfinite(point.normal[axis]) ||
                    !std::isfinite(point.impulse[axis]))
                {
                    bmRecordFailure(mRow, "non-finite point data at point %u", pointIndex);
                    return false;
                }
            }
            if (!std::isfinite(point.separation) || point.separation > 0.0f || point.separation < -1.0f)
            {
                bmRecordFailure(
                    mRow, "invalid separation at point %u: %.9g", pointIndex, static_cast<double>(point.separation));
                return false;
            }
            const float normalLengthSquared = point.normal[0] * point.normal[0] + point.normal[1] * point.normal[1] +
                                              point.normal[2] * point.normal[2];
            if (normalLengthSquared < 0.5f || normalLengthSquared > 1.5f)
            {
                bmRecordFailure(mRow, "invalid contact normal at point %u", pointIndex);
                return false;
            }
        }

        if (std::find(mPointCoverage.begin(), mPointCoverage.end(), uint8_t{ 0 }) != mPointCoverage.end())
        {
            bmRecordFailure(mRow, "contact report contains an incomplete point-slice partition");
            return false;
        }

        std::sort(mObservedPairs.begin(), mObservedPairs.end());
        if (std::adjacent_find(mObservedPairs.begin(), mObservedPairs.end()) != mObservedPairs.end())
        {
            bmRecordFailure(mRow, "contact report contains a duplicate Reporter/Static pair");
            return false;
        }
        if (mObservedPairs != mExpectedPairs)
        {
            bmRecordFailure(mRow, "contact report identity set does not exactly cover the expected pairs");
            return false;
        }
        return true;
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
    uint64_t mAttachHandle = 0;
    std::array<PairId, kPairCount> mExpectedPairs{};
    std::array<PairId, kPairCount> mObservedPairs{};
    std::array<uint8_t, kPairCount> mPointCoverage{};
    bool mIdentityResolved = false;
    std::unique_ptr<authoringbm::GpuFallbackScope> mGpuScope;
};


struct ContactReportPersistentPairs512Cpu : ContactReportBase
{
    ContactReportPersistentPairs512Cpu() : ContactReportBase("ContactReport.persistent_pairs_512_step_read_cpu", false)
    {
    }
};


struct ContactReportPersistentPairs512Gpu : ContactReportBase
{
    ContactReportPersistentPairs512Gpu() : ContactReportBase("ContactReport.persistent_pairs_512_step_read_gpu", true)
    {
    }
};


Register<ContactReportPersistentPairs512Cpu, true> sContactReportPersistentPairs512Cpu(
    "ContactReport.persistent_pairs_512_step_read_cpu");
Register<ContactReportPersistentPairs512Gpu, true> sContactReportPersistentPairs512Gpu(
    "ContactReport.persistent_pairs_512_step_read_gpu");

} // namespace
