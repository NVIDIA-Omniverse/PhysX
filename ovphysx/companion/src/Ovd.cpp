// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-COMP-OVD-001
// @covers AC-2 AC-4
// @implements REQ-CAPI-COMPANION-001
// @covers AC-1
#include "ovphysx_companion/ovd.h"
#include "internal/sdk/CompanionRuntime.hpp"
#include <omni/physx/OvdDecode.h>
#include <map>
#include <memory>
#include <mutex>
#include <cstring>
#include <limits>
namespace ovphysx::companion::ovd
{
using namespace omni::physx::diagnostics;
using Status=ovphysx_companion_status_t;

Status convert(DecodeResult r) {
    switch(r) {
    case DecodeResult::Success:return OVPHYSX_COMPANION_OK;
    case DecodeResult::Invalid:return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    case DecodeResult::Unsupported:return OVPHYSX_COMPANION_UNAVAILABLE;
    case DecodeResult::Malformed:return OVPHYSX_COMPANION_OVD_MALFORMED;
    case DecodeResult::Limit:return OVPHYSX_COMPANION_OVD_LIMIT;
    case DecodeResult::Error:return OVPHYSX_COMPANION_ERROR;
    }
    return OVPHYSX_COMPANION_ERROR;
}
Status decode(const void* data,uint64_t bytes,const ovphysx_companion_ovd_limits_t* input,uint64_t* output,
              ovphysx_companion_ovd_diagnostic_t* diagnostic) noexcept {
    if (!input || input->size!=sizeof(*input) || input->version!=1 || input->reserved || !output || !diagnostic)
        return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    try {
        DecodeLimits limits{input->input_bytes,input->records,input->payload_bytes,input->string_bytes};
        std::shared_ptr<omni::physx::diagnostics::OvdDocument> document=std::make_shared<OvdDocument>();
        DecodeDiagnostic error;
        const Status status=convert(decodeOvd(data,bytes,limits,*document,error));
        ovphysx_companion_ovd_diagnostic_t message{};
        message.size=sizeof(message);message.version=1;message.offset=error.offset;
        std::memcpy(message.message,error.message,sizeof(message.message));
        *diagnostic=message;
        if (status!=OVPHYSX_COMPANION_OK) return status;
        std::lock_guard<std::mutex> lock(companionRuntime().ovdDocuments().mutex);
        if (companionRuntime().ovdDocuments().nextHandle==std::numeric_limits<uint64_t>::max()) return OVPHYSX_COMPANION_ERROR;
        const uint64_t handle=companionRuntime().ovdDocuments().nextHandle++;
        companionRuntime().ovdDocuments().entries.emplace(handle,std::move(document));
        *output=handle;
        return OVPHYSX_COMPANION_OK;
    } catch (...) {return OVPHYSX_COMPANION_ERROR;}
}
template<typename Fn> Status withDocument(uint64_t handle,Fn fn) noexcept {
    try {
        std::shared_ptr<const OvdDocument> document;
        {std::lock_guard<std::mutex> lock(companionRuntime().ovdDocuments().mutex);
         std::map<uint64_t, std::shared_ptr<const omni::physx::diagnostics::OvdDocument>>::iterator it=companionRuntime().ovdDocuments().entries.find(handle);if (it==companionRuntime().ovdDocuments().entries.end()) return OVPHYSX_COMPANION_NOT_FOUND;
         document=it->second;}
        return fn(*document);
    } catch(...) {return OVPHYSX_COMPANION_ERROR;}
}
Status describe(uint64_t handle,uint32_t size,ovphysx_companion_ovd_info_t* output) noexcept {
    if (!output || size!=sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withDocument(handle,[&](const OvdDocument& doc)->Status {
        *output={sizeof(*output),1,doc.major,doc.minor,doc.patch,0,doc.records.size()};
        return OVPHYSX_COMPANION_OK;
    });
}
Status record(uint64_t handle,uint64_t index,uint32_t size,ovphysx_companion_ovd_record_t* output) noexcept {
    if (!output || size!=sizeof(*output)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withDocument(handle,[&](const OvdDocument& doc)->Status {
        if (index>=doc.records.size()) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        const omni::physx::diagnostics::OvdRecord & r=doc.records[index];
        *output={sizeof(*output),1,r.command,0,r.offset,r.context,r.object,r.timestamp,
            r.classHandle,r.baseClass,r.attribute,r.dataType,r.elements,r.attributeClass,r.enumValue,r.line,r.messageType,
            uint32_t(r.name.size()),uint32_t(r.file.size()),uint32_t(r.attributePath.size()),uint32_t(r.payload.size())};
        return OVPHYSX_COMPANION_OK;
    });
}
Status copy(uint64_t handle,uint64_t index,uint32_t kind,void* data,uint64_t capacity,uint64_t* required) noexcept {
    if (!required || kind>3 || (!data && capacity)) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    return withDocument(handle,[&](const OvdDocument& doc)->Status {
        if (index>=doc.records.size()) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        const omni::physx::diagnostics::OvdRecord & r=doc.records[index];
        const void* source=nullptr;uint64_t size=0;
        switch(kind) {
        case 0:source=r.name.data();size=r.name.size();break;
        case 1:source=r.file.data();size=r.file.size();break;
        case 2:source=r.attributePath.data();size=r.attributePath.size()*sizeof(uint32_t);break;
        case 3:source=r.payload.data();size=r.payload.size();break;
        }
        *required=size;
        if (!data && !capacity) return OVPHYSX_COMPANION_OK;
        if (capacity<size) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
        if (size) std::memcpy(data,source,size);
        return OVPHYSX_COMPANION_OK;
    });
}
Status release(uint64_t handle) noexcept {
    try {std::lock_guard<std::mutex> lock(companionRuntime().ovdDocuments().mutex);return companionRuntime().ovdDocuments().entries.erase(handle)?OVPHYSX_COMPANION_OK:OVPHYSX_COMPANION_NOT_FOUND;}
    catch(...) {return OVPHYSX_COMPANION_ERROR;}
}
}
using namespace ovphysx::companion::ovd;
extern "C" OVPHYSX_API ovphysx_companion_status_t ovphysx_companion_ovd_acquire(
    uint32_t version,uint32_t size,ovphysx_companion_ovd_api_t* output) {
    if (!output) return OVPHYSX_COMPANION_INVALID_ARGUMENT;
    if (version!=1 || size!=sizeof(*output)) return OVPHYSX_COMPANION_ABI_MISMATCH;
    *output={sizeof(*output),1,1,decode,describe,record,copy,release};
    return OVPHYSX_COMPANION_OK;
}
