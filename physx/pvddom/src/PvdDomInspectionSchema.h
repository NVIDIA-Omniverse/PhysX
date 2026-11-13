// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once
#include "PvdDomInspection.h"

namespace PvdDomInspect
{
struct SchemaAttribute
{
    const OmniPvdClass* declaringClass;
    const OmniPvdAttributeDef* attribute;
    bool hasRecordedWrite;
};
// Internal bounded enumeration shared by CLI and Python. Pointers stay document-local.
inline bool schemaPage(const Document& document, uint64_t objectId, uint64_t offset, uint64_t limit,
                       std::vector<SchemaAttribute>& items, bool& more, std::string& error)
{
    const auto found = document.history().find(objectId);
    if (found == document.history().end() || !found->second.object || !found->second.object->mOmniPvdClass)
    { error = "Object schema is unavailable"; return false; }
    items.clear(); more = false;
    uint64_t seen = 0;
    const auto& record = found->second;
    const auto& chain = record.object->mOmniPvdClass->mInheritanceChain;
    for (size_t classIndex = 0; classIndex < chain.size(); ++classIndex)
    {
        const auto* cls = chain[classIndex];
        for (const auto* attribute : cls->mAttributeDefinitions)
        {
            if (seen++ < offset) continue;
            if (items.size() >= limit) { more = true; return true; }
            bool written = record.writes.count(attribute->mOmniAttributeHandle) != 0;
            if (attribute->mIsUniqueList && classIndex < record.object->mInheritedClassInstances.size())
            {
                const auto& lists = record.object->mInheritedClassInstances[classIndex].mClassAttributeLists;
                const auto slot = attribute->mClassOffset;
                written = slot < lists.size() && lists[slot] && lists[slot]->mFirst;
            }
            items.push_back({cls, attribute, written});
        }
    }
    return true;
}
inline const char* dataTypeName(uint16_t type)
{
    switch (type)
    {
    case OmniPvdDataType::eINT8: return "INT8";
    case OmniPvdDataType::eINT16: return "INT16";
    case OmniPvdDataType::eINT32: return "INT32";
    case OmniPvdDataType::eINT64: return "INT64";
    case OmniPvdDataType::eUINT8: return "UINT8";
    case OmniPvdDataType::eUINT16: return "UINT16";
    case OmniPvdDataType::eUINT32: return "UINT32";
    case OmniPvdDataType::eUINT64: return "UINT64";
    case OmniPvdDataType::eFLOAT32: return "FLOAT32";
    case OmniPvdDataType::eFLOAT64: return "FLOAT64";
    case OmniPvdDataType::eSTRING: return "STRING";
    case OmniPvdDataType::eOBJECT_HANDLE: return "OBJECT_HANDLE";
    case OmniPvdDataType::eENUM_VALUE: return "ENUM_VALUE";
    case OmniPvdDataType::eFLAGS_WORD: return "FLAGS_WORD";
    }
    return "UNKNOWN";
}
}
