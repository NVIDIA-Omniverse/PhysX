// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

// @implements REQ-OVD-DECODE-001
// @covers AC-1 AC-2 AC-3
#include <omni/physx/OvdDecode.h>
#include <cstdio>
#include <cstring>
#include <utility>
namespace omni::physx::diagnostics
{
namespace
{
struct Failure { DecodeResult result; uint64_t offset; const char* message; };
struct Reader
{
    const uint8_t* data;
    uint64_t size, offset=0;
    const DecodeLimits& limits;
    [[noreturn]] void fail(DecodeResult code,const char* message) const {throw Failure{code,offset,message};}
    void require(uint64_t count) const {
        if (count>size-offset) fail(DecodeResult::Malformed,"Truncated command or payload");
    }
    template<typename T> T number() {
        require(sizeof(T));
        uint64_t value=0;
        for (unsigned i=0;i<sizeof(T);++i) value|=uint64_t(data[offset++])<<(i*8);
        return T(value);
    }
    std::string string() {
        const unsigned short length=number<uint16_t>();
        if (length>limits.stringBytes) fail(DecodeResult::Limit,"String exceeds configured limit");
        require(length);
        std::string value(reinterpret_cast<const char*>(data+offset),length);
        offset+=length;
        return value;
    }
    uint32_t type() {
        unsigned short value=number<uint16_t>();
        if (value>13) fail(DecodeResult::Unsupported,"Unknown attribute data type");
        return value;
    }
    void payload(OvdRecord& record) {
        const unsigned char depth=number<uint8_t>();
        if (!depth) fail(DecodeResult::Malformed,"Attribute path must be nonempty");
        require(uint64_t(depth)*sizeof(uint32_t));
        record.attributePath.reserve(depth);
        for (unsigned i=0;i<depth;++i) record.attributePath.push_back(number<uint32_t>());
        record.attribute=record.attributePath.back();
        const unsigned int count=number<uint32_t>();
        if (count>limits.payloadBytes) fail(DecodeResult::Limit,"Attribute payload exceeds configured limit");
        require(count);
        record.payload.assign(data+offset,data+offset+count);
        offset+=count;
    }
};
}
DecodeResult decodeOvd(const void* data,uint64_t size,const DecodeLimits& limits,
                      OvdDocument& output,DecodeDiagnostic& diagnostic) noexcept
{
    diagnostic={};
    try {
        if (!data || !size || !limits.inputBytes || limits.inputBytes>512ull*1024*1024 ||
            !limits.records || limits.records>1000000 || !limits.payloadBytes ||
            limits.payloadBytes>limits.inputBytes || !limits.stringBytes || limits.stringBytes>65535)
            throw Failure{DecodeResult::Invalid,0,"Invalid input or decoder limits"};
        if (size>limits.inputBytes) throw Failure{DecodeResult::Limit,0,"Input exceeds configured limit"};
        Reader reader{static_cast<const uint8_t*>(data),size,0,limits};
        OvdDocument result;
        result.major=reader.number<uint32_t>();
        result.minor=reader.number<uint32_t>();
        result.patch=reader.number<uint32_t>();
        if (result.major!=0 || result.minor<1 || result.minor>4 ||
            (result.minor==1 && result.patch<42) || (result.minor==4 && result.patch>0))
            reader.fail(DecodeResult::Unsupported,"Unsupported OVD stream version");
        while (reader.offset<size) {
            if (result.records.size()>=limits.records) reader.fail(DecodeResult::Limit,"Record count exceeds configured limit");
            OvdRecord r;
            r.offset=reader.offset;
            r.command=reader.number<uint8_t>();
            switch(r.command) {
            case 1: // REGISTER_CLASS
                r.classHandle=reader.number<uint32_t>();
                if (result.minor>=3) r.baseClass=reader.number<uint32_t>();
                r.name=reader.string();
                break;
            case 3: // REGISTER_ATTRIBUTE; enum values are registered here too.
                r.classHandle=reader.number<uint32_t>();r.attribute=reader.number<uint32_t>();
                r.dataType=reader.type();
                if (r.dataType==12) r.enumValue=reader.number<uint32_t>();
                else if (r.dataType==13) r.attributeClass=reader.number<uint32_t>();
                else r.elements=reader.number<uint32_t>();
                r.name=reader.string();
                break;
            case 4: // REGISTER_CLASS_ATTRIBUTE
                r.classHandle=reader.number<uint32_t>();r.attribute=reader.number<uint32_t>();
                r.attributeClass=reader.number<uint32_t>();r.name=reader.string();
                break;
            case 5: // REGISTER_UNIQUE_LIST_ATTRIBUTE
                r.classHandle=reader.number<uint32_t>();r.attribute=reader.number<uint32_t>();
                r.dataType=reader.type();r.name=reader.string();
                break;
            case 6: case 7: case 8: // SET, ADD, REMOVE_ATTRIBUTE
                r.context=reader.number<uint64_t>();r.object=reader.number<uint64_t>();reader.payload(r);
                break;
            case 9: // CREATE_OBJECT
                r.context=reader.number<uint64_t>();r.classHandle=reader.number<uint32_t>();
                r.object=reader.number<uint64_t>();r.name=reader.string();
                break;
            case 10: // DESTROY_OBJECT
                r.context=reader.number<uint64_t>();r.object=reader.number<uint64_t>();break;
            case 11: case 12: // START_FRAME, STOP_FRAME
                r.context=reader.number<uint64_t>();r.timestamp=reader.number<uint64_t>();break;
            case 13: // RECORD_MESSAGE
                if (result.minor<4) reader.fail(DecodeResult::Unsupported,"Message command requires OVD 0.4");
                r.context=reader.number<uint64_t>();r.name=reader.string();r.file=reader.string();
                r.line=reader.number<uint32_t>();r.messageType=reader.number<uint32_t>();
                r.classHandle=reader.number<uint32_t>();break;
            default:reader.fail(DecodeResult::Unsupported,"Unknown or reserved OVD command");
            }
            result.records.push_back(std::move(r));
        }
        output=std::move(result);
        return DecodeResult::Success;
    } catch(const Failure& error) {
        diagnostic.offset=error.offset;
        std::snprintf(diagnostic.message,sizeof(diagnostic.message),"%s",error.message);
        return error.result;
    } catch(...) {
        std::snprintf(diagnostic.message,sizeof(diagnostic.message),"OVD allocation or decoding failure");
        return DecodeResult::Error;
    }
}
}
