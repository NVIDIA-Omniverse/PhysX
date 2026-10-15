// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-QUERY-001
// @covers AC-1 AC-2 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/queries.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/NativeQueries.h>
#include <algorithm>
#include <cstring>
namespace
{
using namespace omni::physx::native;
ObjectReference reference(const ovphysx_companion_object_t& r)
{return {r.attachment,r.object,r.key,r.generation,r.kind,r.instance};}
ovphysx_companion_status_t execute(uint64_t attachment,const ovphysx_companion_query_t* input,uint32_t inputSize,
    ovphysx_companion_query_hit_t* hits,uint32_t hitSize,uint32_t capacity,
    ovphysx_companion_query_result_t* output,uint32_t outputSize) noexcept
try
{
    if(!input || inputSize!=sizeof(*input) || !output || outputSize!=sizeof(*output) || !hits ||
        hitSize!=sizeof(*hits) || !input->capacity || capacity<input->capacity || input->capacity>65536 ||
        input->includeCount>256 || input->excludeCount>256 || (input->includeCount && !input->include) ||
        (input->excludeCount && !input->exclude))return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    CompanionOperationGuard guard(companionRuntime());if(!guard)return OVPHYSX_COMPANION_BUSY;
    SceneQuery query;
    query.kind=input->kind;query.mode=input->mode;query.geometry=input->geometry;query.flags=input->flags;
    query.capacity=input->capacity;query.distance=input->distance;query.shape=reference(input->shape);
    std::copy_n(input->pose,7,query.pose);std::copy_n(input->direction,3,query.direction);std::copy_n(input->dimensions,3,query.dimensions);
    for(uint32_t i=0;i<input->includeCount;++i)query.include.push_back(reference(input->include[i]));
    for(uint32_t i=0;i<input->excludeCount;++i)query.exclude.push_back(reference(input->exclude[i]));
    SceneQueryResult value;
    switch(queryScene(attachment,query,value)) {
    case InspectionResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case InspectionResult::Stale:return OVPHYSX_COMPANION_STALE;
    case InspectionResult::Busy:return OVPHYSX_COMPANION_BUSY;
    case InspectionResult::NotFound:return OVPHYSX_COMPANION_NOT_FOUND;
    case InspectionResult::Unavailable:return OVPHYSX_COMPANION_UNAVAILABLE;
    case InspectionResult::Error:return OVPHYSX_COMPANION_ERROR;
    case InspectionResult::Success:break;
    }
    static_assert(sizeof(SceneQueryHit)==sizeof(*hits) && alignof(SceneQueryHit)==alignof(ovphysx_companion_query_hit_t));
    if(!value.hits.empty())std::memcpy(hits,value.hits.data(),value.hits.size()*sizeof(*hits));
    *output={value.attachment,value.step,value.revision,value.total,uint32_t(value.hits.size()),uint32_t(value.truncated)};
    return OVPHYSX_COMPANION_OK;
}
catch(...){return OVPHYSX_COMPANION_ERROR;}
}
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_queries_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_queries_api_t* output)
{
    if(!output)return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if(version!=1 || size!=sizeof(*output))return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,7,execute};return OVPHYSX_COMPANION_OK;
}
