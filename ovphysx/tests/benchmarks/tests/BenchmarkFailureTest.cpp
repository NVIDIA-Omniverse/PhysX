// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-CAPI-BENCHMARK-008
 * @covers AC-4
 *
 * @implements REQ-CAPI-BENCHMARK-009
 * @covers AC-3 AC-4
 */

#include "../BenchmarkFailure.h"
#include "../benchmarks/AuthoringCommon.h"
#include "../benchmarks/AuthoringSpawnBatch.h"
#include "../benchmarks/AuthoringReadback.h"

#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <string_view>
#include <type_traits>


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


struct QueryReleaseProbe
{
    uint32_t calls = 0;
    ovstage_query_handle_t queries[2] = { OVSTAGE_INVALID_QUERY_HANDLE, OVSTAGE_INVALID_QUERY_HANDLE };
    ovstage_instance_t* stage = nullptr;
    const char* row = nullptr;
    bool result = true;
};

QueryReleaseProbe gQueryReleaseProbe;

bool recordQueryRelease(ovstage_instance_t* stage, ovstage_query_handle_t query, const char* row)
{
    if (gQueryReleaseProbe.calls < 2)
    {
        gQueryReleaseProbe.queries[gQueryReleaseProbe.calls] = query;
    }
    ++gQueryReleaseProbe.calls;
    gQueryReleaseProbe.stage = stage;
    gQueryReleaseProbe.row = row;
    return gQueryReleaseProbe.result;
}

// Model the ownership boundary after the first acquisition succeeded and a
// later acquisition or operation failed. No native API failure is injected.
bool abortWithOwnedQuery(ovstage_instance_t* stage, ovstage_query_handle_t query, const char* row)
{
    authoringbm::ScopedQuery owner(stage, query, row, recordQueryRelease);
    return false;
}

/**
 * @implements REQ-CAPI-BENCHMARK-008
 * @covers AC-4
 */
bool verifyScopedQueryOwnership()
{
    ovstage_instance_t* const stage = reinterpret_cast<ovstage_instance_t*>(static_cast<uintptr_t>(1));
    const char* const row = "QueryRetention.ownership_unit";
    const ovstage_query_handle_t first = ovstage_query_handle_t{ 11 };
    const ovstage_query_handle_t second = ovstage_query_handle_t{ 12 };
    bool ok = true;

    gQueryReleaseProbe = {};
    {
        authoringbm::ScopedQuery owner(stage, first, row, recordQueryRelease);
        // Borrowing the same selection for several operations never retires it.
        for (uint32_t operation = 0; operation < 22; ++operation)
        {
            if (owner.get() != first || gQueryReleaseProbe.calls != 0)
            {
                std::fprintf(stderr, "borrowing a retained query changed its ownership\n");
                ok = false;
                break;
            }
        }
        if (!owner.close() || owner.get() != OVSTAGE_INVALID_QUERY_HANDLE || !owner.close())
        {
            std::fprintf(stderr, "explicit query retirement did not consume ownership\n");
            ok = false;
        }
    }
    if (gQueryReleaseProbe.calls != 1 || gQueryReleaseProbe.queries[0] != first ||
        gQueryReleaseProbe.stage != stage || gQueryReleaseProbe.row != row)
    {
        std::fprintf(stderr, "retained query was not released exactly once against its owner stage\n");
        ok = false;
    }

    gQueryReleaseProbe = {};
    if (abortWithOwnedQuery(stage, first, row) || gQueryReleaseProbe.calls != 1 ||
        gQueryReleaseProbe.queries[0] != first)
    {
        std::fprintf(stderr, "early failure abandoned the previously acquired query\n");
        ok = false;
    }

    gQueryReleaseProbe = {};
    try
    {
        authoringbm::ScopedQuery body(stage, first, row, recordQueryRelease);
        authoringbm::ScopedQuery collider(stage, second, row, recordQueryRelease);
        throw std::runtime_error("simulated payload allocation failure");
    }
    catch (const std::runtime_error&)
    {
    }
    if (gQueryReleaseProbe.calls != 2 || gQueryReleaseProbe.queries[0] != second ||
        gQueryReleaseProbe.queries[1] != first)
    {
        std::fprintf(stderr, "exception unwinding did not release both acquired queries\n");
        ok = false;
    }

    gQueryReleaseProbe = {};
    gQueryReleaseProbe.result = false;
    {
        authoringbm::ScopedQuery owner(stage, first, row, recordQueryRelease);
        if (owner.close())
        {
            std::fprintf(stderr, "a failed checked query release was reported as successful\n");
            ok = false;
        }
    }
    if (gQueryReleaseProbe.calls != 1)
    {
        std::fprintf(stderr, "an ambiguous query release was retried during destruction\n");
        ok = false;
    }
    return ok;
}

// Check the values, not only acceptance: row maps, offsets and both stride
// units must select the same payload from packed and expanded storage.
bool verifySpawnTensorReadback()
{
    float storage[] = { -1, 1, 2, 3, -1, 4, 5, 6, -1 };
    int64_t shape[] = { 2, 3 };
    int64_t strides[] = { 4, 1 };
    DLTensor tensor{};
    tensor.data = storage;
    tensor.device.device_type = kDLCPU;
    tensor.dtype = { kDLFloat, 32, 1 };
    tensor.ndim = 2;
    tensor.shape = shape;
    tensor.strides = strides;
    tensor.byte_offset = sizeof(float);
    bool ok = true;
    auto check = [&](bool array, uint32_t row, uint16_t components, size_t count,
                     std::array<float, 4> expected, bool accepted = true)
    {
        std::array<float, 4> actual{};
        const bool read = authoringbm::readSpawnTensor(tensor, row, kDLFloat, components, array, count, actual);
        if (read != accepted || (read && actual != expected))
        {
            std::fprintf(stderr, "spawn tensor readback accepted=%d expected=%d ndim=%d lanes=%u\n",
                         int(read), int(accepted), tensor.ndim, unsigned(tensor.dtype.lanes));
            ok = false;
        }
    };
    check(false, 1, 3, 3, { 4, 5, 6, 0 }); // selected row, padded expanded layout, byte offset
    strides[1] = 2;
    check(false, 0, 3, 3, { 1, 3, 4, 0 }); // non-contiguous components
    tensor.strides = nullptr;
    check(false, 0, 3, 3, { 1, 2, 3, 0 }); // contiguous expanded layout
    tensor.ndim = 1;
    tensor.dtype.lanes = 3;
    check(false, 0, 3, 3, { 1, 2, 3, 0 }); // packed layout
    tensor.byte_offset = 0;
    tensor.strides = strides;
    strides[0] = 2;
    check(false, 1, 3, 3, { 5, 6, -1, 0 }); // packed strides count three-component elements
    check(false, 2, 3, 3, {}, false); // out-of-range selected row
    tensor.dtype.lanes = 4;
    shape[0] = 1;
    tensor.strides = nullptr;
    tensor.byte_offset = sizeof(float);
    check(true, 0, 4, 4, { 1, 2, 3, -1 }); // packed quaternion array
    tensor.ndim = 2;
    tensor.dtype.lanes = 1;
    shape[1] = 4;
    check(true, 0, 4, 4, { 1, 2, 3, -1 }); // expanded quaternion array
    shape[1] = 3;
    check(true, 0, 4, 4, {}, false); // wrong component extent
    shape[1] = 4;
    shape[0] = 2;
    check(true, 0, 4, 4, {}, false); // wrong array extent
    shape[0] = 1;
    tensor.dtype.bits = 64;
    check(false, 0, 4, 4, {}, false); // wrong scalar type
    tensor.dtype.bits = 32;
    tensor.dtype.code = kDLUInt;
    check(false, 0, 4, 4, {}, false);
    tensor.dtype.code = kDLFloat;
    tensor.device.device_type = kDLCUDA;
    check(false, 0, 4, 4, {}, false); // never dereference a device pointer
    tensor.device.device_type = kDLCPU;
    tensor.strides = strides;
    strides[0] = 0;
    check(false, 0, 4, 4, {}, false);
    strides[0] = 4;
    strides[1] = -1;
    check(false, 0, 4, 4, {}, false);
    return ok;
}

bool verifySpawnBatchAttributeLengths()
{
    authoringbm::SpawnBatchPayload payload;
    // The view covers only the name, without a terminator. The suffix must not
    // be scanned or included in the attribute passed to OVStage.
    const char storage[] = { 'm', 'a', 's', 's', 'X' };
    const std::string_view name(storage, 4);
    const float value = 1.0f;
    const uint64_t token = 7;
    if (!payload.addFloat(name, &value, 1, OVSTAGE_SEMANTIC_NONE) ||
        !payload.addDouble(name, 2.0) ||
        !payload.addTokens(name, &token, 1, false, OVSTAGE_SEMANTIC_NONE))
        return false;
    for (size_t i = 0; i < payload.size(); ++i)
    {
        const ovx_string_t& attribute = payload.writes()[i].attribute.string;
        if (attribute.ptr != storage || attribute.length != name.size() ||
            std::memcmp(attribute.ptr, "mass", name.size()) != 0)
            return false;
    }
    const size_t count = payload.size();
    return !payload.addFloat({}, &value, 1, OVSTAGE_SEMANTIC_NONE) &&
           !payload.addDouble({}, 2.0) &&
           !payload.addTokens({}, &token, 1, false, OVSTAGE_SEMANTIC_NONE) &&
           payload.size() == count;
}

bool verifySpawnBatchPayloadOwnership()
{
    static_assert(!std::is_copy_constructible<authoringbm::SpawnBatchPayload>::value,
                  "batch tensor pointers must remain attached to their storage");
    static_assert(!std::is_move_constructible<authoringbm::SpawnBatchPayload>::value,
                  "moving a live borrowed batch would invalidate its internal pointers");
    authoringbm::SpawnBatchPayload payload;
    float orientation[] = { 1.0f, 2.0f, 3.0f, 4.0f };
    float velocity[] = { 5.0f, 6.0f, 7.0f };
    uint64_t tokens[] = { 11, 12 };
    if (!payload.addFloat("orientation", orientation, 4, OVSTAGE_SEMANTIC_QUATERNION) ||
        !payload.addFloat("velocity", velocity, 3, OVSTAGE_SEMANTIC_VECTOR) ||
        !payload.addTokens("schemas", tokens, 2, true, OVSTAGE_SEMANTIC_NONE) ||
        !payload.addDouble("size", 1.25))
    {
        std::fprintf(stderr, "mixed spawn batch preparation failed\n");
        return false;
    }
    // The original caller arrays can change while all submitted column copies
    // retain their own values, shapes, dtypes and semantics.
    orientation[0] = 99.0f;
    velocity[0] = 98.0f;
    tokens[0] = 97;
    const ovstage_attribute_write_t* writes = payload.writes();
    const DLTensor& a = writes[0].data.tensors[0];
    const DLTensor& b = writes[1].data.tensors[0];
    const DLTensor& c = writes[2].data.tensors[0];
    const DLTensor& d = writes[3].data.tensors[0];
    bool ok = payload.size() == 4 &&
              static_cast<const float*>(a.data)[0] == 1.0f && static_cast<const float*>(a.data)[3] == 4.0f &&
              static_cast<const float*>(b.data)[0] == 5.0f && static_cast<const float*>(b.data)[2] == 7.0f &&
              static_cast<const uint64_t*>(c.data)[0] == 11 && static_cast<const uint64_t*>(c.data)[1] == 12 &&
              *static_cast<const double*>(d.data) == 1.25 &&
              a.dtype.code == kDLFloat && a.dtype.bits == 32 && a.dtype.lanes == 4 &&
              b.dtype.lanes == 3 && c.dtype.code == kDLUInt && c.dtype.bits == 64 && c.dtype.lanes == 1 &&
              d.dtype.code == kDLFloat && d.dtype.bits == 64 && d.dtype.lanes == 1 &&
              *a.shape == 1 && *b.shape == 1 && *c.shape == 2 && *d.shape == 1 &&
              !writes[0].data.is_array && writes[2].data.is_array &&
              writes[0].data.semantic == OVSTAGE_SEMANTIC_QUATERNION &&
              writes[1].data.semantic == OVSTAGE_SEMANTIC_VECTOR;
    for (size_t i = 0; i < payload.size(); ++i)
    {
        ok = ok && writes[i].data.tensor_count == 1 && !writes[i].data.managed_tensors;
        for (size_t j = i + 1; j < payload.size(); ++j)
        {
            ok = ok && writes[i].data.tensors != writes[j].data.tensors &&
                 writes[i].data.tensors[0].data != writes[j].data.tensors[0].data &&
                 writes[i].data.tensors[0].shape != writes[j].data.tensors[0].shape;
        }
    }
    const size_t beforeRejected = payload.size();
    ok = ok && !payload.addFloat("invalid", velocity, 0, OVSTAGE_SEMANTIC_NONE) &&
         !payload.addTokens("scalar", tokens, 2, false, OVSTAGE_SEMANTIC_NONE) && payload.size() == beforeRejected;
    for (size_t i = payload.size(); i < authoringbm::SpawnBatchPayload::kCapacity; ++i)
        ok = payload.addDouble("padding", 2.5) && ok;
    ok = ok && payload.size() == authoringbm::SpawnBatchPayload::kCapacity && !payload.addDouble("overflow", 3.5) &&
         static_cast<const float*>(a.data)[0] == 1.0f && *c.shape == 2;
    if (!ok)
        std::fprintf(stderr, "spawn batch payloads aliased, lost metadata or accepted invalid capacity\n");
    return ok;
}

} // namespace


int main()
{
    const char* const row = "Authoring.population_add_drip_cpu";
    bmRecordFailure(row, "synthetic unit-test failure");

    bool ok = verifyScopedQueryOwnership();
    ok = verifySpawnTensorReadback() && ok;
    ok = verifySpawnBatchAttributeLengths() && ok;
    ok = verifySpawnBatchPayloadOwnership() && ok;

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
    const ovx_primpath_list_t validList = ovx_primpath_list_t{1};
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
