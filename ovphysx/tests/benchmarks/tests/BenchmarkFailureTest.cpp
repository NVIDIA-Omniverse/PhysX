// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "../BenchmarkFailure.h"
#include "../benchmarks/AuthoringCommon.h"

#include <cstdint>
#include <cstdio>


namespace
{

bool expectMatch(const char* registeredName, bool expected)
{
    const bool actual = bmRowHasFailure(registeredName);
    if (actual == expected)
    {
        return true;
    }

    std::fprintf(stderr, "bmRowHasFailure(\"%s\") returned %s, expected %s\n", registeredName,
                 actual ? "true" : "false", expected ? "true" : "false");
    return false;
}

} // namespace


int main()
{
    const char* const row = "Authoring.population_add_drip_cpu";
    bmRecordFailure(row, "synthetic unit-test failure");

    bool ok = true;

    const char* const capacityWarnings[] = {
        "Contact buffer overflow detected, please increase its size in the scene desc!",
        "Patch buffer overflow detected, please increase its size in the scene desc!",
        "PxGpuDynamicsMemoryConfig::collisionStackSize buffer overflow detected; Contacts have been dropped.",
        "The application needs to increase PxGpuDynamicsMemoryConfig::foundLostPairsCapacity to 1024, otherwise, "
        "the simulation will miss interactions",
        "The application needs to increase PxGpuDynamicsMemoryConfig::foundLostAggregatePairsCapacity to 2048, "
        "otherwise, the simulation will miss interactions",
        "The application needs to increase PxGpuDynamicsMemoryConfig::totalAggregatePairsCapacity to 4096, "
        "otherwise, the simulation will miss interactions",
    };
    for (const char* warning : capacityWarnings)
    {
        if (!authoringbm::isGpuCapacityWarningText(warning))
        {
            std::fprintf(stderr, "source-backed capacity warning was not recognized: %s\n", warning);
            ok = false;
        }
    }
    const char* const benignCapacityMentions[] = {
        "Using authored physxScene:gpuMaxRigidContactCount",
        "maxRigidPatchCount was clamped to its schema default",
        "foundLostPairsCapacity is using its default value",
        "A later simulation may miss interactions for an unrelated reason",
    };
    for (const char* warning : benignCapacityMentions)
    {
        if (authoringbm::isGpuCapacityWarningText(warning))
        {
            std::fprintf(stderr, "benign capacity mention was misclassified: %s\n", warning);
            ok = false;
        }
    }

    ok = expectMatch(row, true) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_GPU", true) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_8T", true) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_8T_GPU", true) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_-2T", true) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_-2T_GPU", true) && ok;

    // One row's failure must not suppress a distinct neighboring row.
    ok = expectMatch("Authoring.population_add_packed_cpu", false) && ok;

    // Only the postfixes constructed by Harness.cpp are accepted.
    ok = expectMatch("Authoring.population_add_drip_cpu_extra", false) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_CPU", false) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_T", false) && ok;
    ok = expectMatch("Authoring.population_add_drip_cpu_8T_GPU_extra", false) && ok;

    // A caller that accidentally records only a family prefix must not
    // suppress a longer registered row in that family.
    bmRecordFailure("Authoring.teleport", "synthetic truncated-prefix failure");
    ok = expectMatch("Authoring.teleport_tensor_cpu", false) && ok;

    ovphysx_sample_stage_attachment_t cleanAttachment{};
    cleanAttachment.ordinal = 41;
    bool cleanAttachmentReported = false;
    if (!authoringbm::prepareStageAttachmentForRun(
            cleanAttachment, "Authoring.clean_attachment", &cleanAttachmentReported) ||
        cleanAttachment.stage != nullptr || cleanAttachment.ordinal != 0 || cleanAttachmentReported)
    {
        std::fprintf(stderr, "clean stage attachment was not reset for a new run\n");
        ok = false;
    }

    ovphysx_sample_stage_attachment_t leftoverAttachment{};
    leftoverAttachment.stage = reinterpret_cast<ovstage_instance_t*>(static_cast<uintptr_t>(1));
    leftoverAttachment.ordinal = 42;
    bool leftoverAttachmentReported = false;
    const uint32_t failuresBeforeLeftover = bmFailureCount();
    if (authoringbm::prepareStageAttachmentForRun(
            leftoverAttachment, "Authoring.leftover_attachment", &leftoverAttachmentReported) ||
        leftoverAttachment.stage != reinterpret_cast<ovstage_instance_t*>(static_cast<uintptr_t>(1)) ||
        leftoverAttachment.ordinal != 42 || !leftoverAttachmentReported ||
        bmFailureCount() != failuresBeforeLeftover + 1)
    {
        std::fprintf(stderr, "leftover stage attachment was not preserved and reported\n");
        ok = false;
    }
    if (authoringbm::prepareStageAttachmentForRun(
            leftoverAttachment, "Authoring.leftover_attachment", &leftoverAttachmentReported) ||
        bmFailureCount() != failuresBeforeLeftover + 1)
    {
        std::fprintf(stderr, "leftover stage attachment failure was reported more than once\n");
        ok = false;
    }

    bmRecordFailure("Authoring.already_failed", "synthetic prior failure");
    ovphysx_sample_stage_attachment_t failedAttachment{};
    failedAttachment.ordinal = 43;
    bool failedAttachmentReported = false;
    if (authoringbm::prepareStageAttachmentForRun(
            failedAttachment, "Authoring.already_failed", &failedAttachmentReported) ||
        failedAttachment.ordinal != 43 || failedAttachmentReported)
    {
        std::fprintf(stderr, "failed row reset or restarted its stage attachment\n");
        ok = false;
    }

    if (!authoringbm::isCompletedStageWaitStatus(OVSTAGE_OK) ||
        !authoringbm::isCompletedStageWaitStatus(OVSTAGE_ERROR_OP_FAILED) ||
        authoringbm::isCompletedStageWaitStatus(OVSTAGE_ERROR_TIMEOUT))
    {
        std::fprintf(stderr, "OVStage wait completion classification is incorrect\n");
        ok = false;
    }

    // queryPath()'s callers, churn in particular, read false as "the prim is
    // already gone", so a construction fault must not look like one. The list
    // is usable only when the call succeeded and handed back a real handle. A
    // success status carrying the invalid sentinel is a fault, not an empty
    // result.
    const ovx_primpath_list_t validList = static_cast<ovx_primpath_list_t>(1);
    if (!authoringbm::queryPathListIsUsable(OVX_API_SUCCESS, validList) ||
        authoringbm::queryPathListIsUsable(OVX_API_SUCCESS, OVX_INVALID_PRIMPATH_LIST) ||
        authoringbm::queryPathListIsUsable(OVX_API_ERROR, validList) ||
        authoringbm::queryPathListIsUsable(OVX_API_ERROR, OVX_INVALID_PRIMPATH_LIST))
    {
        std::fprintf(stderr, "query path-list usability rule is incorrect\n");
        ok = false;
    }

    // queryPath() creates a query and then releases a temporary path-list
    // reference. Only the both-succeeded outcome hands the caller a live
    // query. A created query whose path-list release failed still exists but
    // is reported as failure, so queryPath() has to retire it. The caller
    // reads false as "no handle" and cannot.
    if (authoringbm::queryPathMustDiscardQuery(/*queryCreated=*/true, /*pathListReleased=*/true) ||
        !authoringbm::queryPathMustDiscardQuery(/*queryCreated=*/true, /*pathListReleased=*/false) ||
        authoringbm::queryPathMustDiscardQuery(/*queryCreated=*/false, /*pathListReleased=*/true) ||
        authoringbm::queryPathMustDiscardQuery(/*queryCreated=*/false, /*pathListReleased=*/false))
    {
        std::fprintf(stderr, "queryPath orphaned-query ownership rule is incorrect\n");
        ok = false;
    }

    return ok ? 0 : 1;
}
