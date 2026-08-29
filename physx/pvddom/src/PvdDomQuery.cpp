// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2026 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.


// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "PvdDomQuery.h"

#include <cstring>

// ---------------------------------------------------------------------------
// class / attribute lookup
// ---------------------------------------------------------------------------

OmniPvdClass* findClassByName(OmniPvdDOMState& domState, const char* className)
{
    if (!className)
    {
        return nullptr;
    }
    for (auto& kv : domState.mClassHandleToClassMap)
    {
        OmniPvdClass* cls = kv.second;
        if (cls && std::strcmp(cls->mClassName.c_str(), className) == 0)
        {
            return cls;
        }
    }
    return nullptr;
}

OmniPvdAttributeDef* findAttributeOnChain(OmniPvdClass* cls,
                                          const char* attrName,
                                          int32_t* outClassIndex,
                                          int32_t* outAttrIndex)
{
    if (outClassIndex) *outClassIndex = -1;
    if (outAttrIndex)  *outAttrIndex  = -1;

    if (!cls || !attrName)
    {
        return nullptr;
    }

    const int32_t chainLen = static_cast<int32_t>(cls->mInheritanceChain.size());
    for (int32_t c = 0; c < chainLen; ++c)
    {
        OmniPvdClass* ancestor = cls->mInheritanceChain[c];
        if (!ancestor) continue;
        const int32_t nbrAttrs = static_cast<int32_t>(ancestor->mAttributeDefinitions.size());
        for (int32_t a = 0; a < nbrAttrs; ++a)
        {
            OmniPvdAttributeDef* def = ancestor->mAttributeDefinitions[a];
            if (def && std::strcmp(def->mAttributeName.c_str(), attrName) == 0)
            {
                if (outClassIndex) *outClassIndex = c;
                if (outAttrIndex)  *outAttrIndex  = a;
                return def;
            }
        }
    }
    return nullptr;
}

std::vector<OmniPvdAttributeDef*> flattenInheritedAttributes(OmniPvdClass* cls)
{
    std::vector<OmniPvdAttributeDef*> out;
    if (!cls) return out;
    for (OmniPvdClass* ancestor : cls->mInheritanceChain)
    {
        if (!ancestor) continue;
        for (OmniPvdAttributeDef* def : ancestor->mAttributeDefinitions)
        {
            out.push_back(def);
        }
    }
    return out;
}

// ---------------------------------------------------------------------------
// lifespan / frame queries
// ---------------------------------------------------------------------------

bool isObjectAliveAtFrame(const OmniPvdObject* obj, uint64_t frame)
{
    // Half-open [mFrameStart, mFrameStop): the writer stamps mFrameStop with
    // the scene's mFrameId at destroy time, so that frame is the FIRST DEAD
    // frame, not the last alive one. Inclusive semantics leaked the actor
    // onto the pre-sim view of step N when removeActor() was called between
    // fetchResults(N-1) and simulate(N) -- a beta-tester report fixed here.
    // mFrameStop == 0 still encodes "never destroyed" / open-ended.
    if (!obj) return false;
    for (const OmniPvdObjectLifeSpan& span : obj->mLifeSpans)
    {
        const bool startsOnOrBefore = (span.mFrameStart <= frame);
        const bool stillOpen        = (span.mFrameStop == 0) || (frame < span.mFrameStop);
        if (startsOnOrBefore && stillOpen)
        {
            return true;
        }
    }
    return false;
}

std::vector<OmniPvdObject*> getAliveObjectsAtFrame(OmniPvdDOMState& domState, uint64_t frame)
{
    std::vector<OmniPvdObject*> out;
    for (OmniPvdObject* obj : domState.mObjectCreations)
    {
        if (isObjectAliveAtFrame(obj, frame))
        {
            out.push_back(obj);
        }
    }
    return out;
}

OmniPvdAttributeSample* getLatestSampleAtFrame(OmniPvdAttributeInstList* list, uint64_t frameTimestamp)
{
    if (!list || !list->mAttributeDef) return nullptr;
    // Unique-list attributes hold OmniPvdUniqueList instances, not samples.
    // Casting those to OmniPvdAttributeSample would be a wrong-subtype read.
    if (list->mAttributeDef->mIsUniqueList) return nullptr;
    OmniPvdAttributeSample* latest = nullptr;
    OmniPvdAttributeInst* inst = list->mFirst;
    while (inst)
    {
        if (inst->mTimeStamp <= frameTimestamp)
        {
            latest = static_cast<OmniPvdAttributeSample*>(inst);
        }
        else
        {
            // Early-break is sound only because the parser appends samples
            // strictly in monotonic, non-decreasing mTimeStamp order -- see
            // OmniPvdAttributeInstList::addAttribute callers in
            // PvdDomParser.cpp's setAttribute path, which only ever pushes
            // the newly constructed sample to `mLast`. Once we see a sample
            // past `frameTimestamp`, every successor is also past it, so
            // the current `latest` is the answer. If a future code path ever
            // inserts out of order this returns a stale result silently --
            // anyone adding non-append insertion must replace this loop with
            // a full scan or maintain a sorted-by-timestamp invariant.
            break;
        }
        inst = inst->mNextAttribute;
    }
    return latest;
}

OmniPvdAttributeSample* getLatestSampleAtFrame(OmniPvdObject* obj,
                                               const char* attrName,
                                               uint64_t frameTimestamp)
{
    if (!obj || !obj->mOmniPvdClass) return nullptr;

    int32_t classIndex = -1;
    int32_t attrIndex  = -1;
    if (!findAttributeOnChain(obj->mOmniPvdClass, attrName, &classIndex, &attrIndex))
    {
        return nullptr;
    }
    if (classIndex < 0 || attrIndex < 0) return nullptr;

    if (classIndex >= static_cast<int32_t>(obj->mInheritedClassInstances.size()))
    {
        return nullptr;
    }
    auto& attrLists = obj->mInheritedClassInstances[classIndex].mClassAttributeLists;
    if (attrIndex >= static_cast<int32_t>(attrLists.size()))
    {
        return nullptr;
    }
    return getLatestSampleAtFrame(attrLists[attrIndex], frameTimestamp);
}

// ---------------------------------------------------------------------------
// reference resolution
// ---------------------------------------------------------------------------

OmniPvdObject* resolveObjectReference(OmniPvdDOMState& domState, uint64_t apiHandle)
{
    if (apiHandle == 0) return nullptr;

    auto extIt = domState.mExternalToInternalHandleMap.find(apiHandle);
    if (extIt == domState.mExternalToInternalHandleMap.end())
    {
        return nullptr;
    }
    auto objIt = domState.mObjectHandleToObjectMap.find(extIt->second);
    if (objIt == domState.mObjectHandleToObjectMap.end())
    {
        return nullptr;
    }
    return objIt->second;
}

// ---------------------------------------------------------------------------
// typed sample reads
// ---------------------------------------------------------------------------

bool readVec3(const OmniPvdAttributeSample* sample, float out[3])
{
    if (!sample || !sample->mData || sample->mDataLen < sizeof(float) * 3)
    {
        return false;
    }
    std::memcpy(out, sample->mData, sizeof(float) * 3);
    return true;
}

bool readTransform(const OmniPvdAttributeSample* sample, float outPos[3], float outQuat[4])
{
    if (!sample || !sample->mData || sample->mDataLen < sizeof(float) * 7)
    {
        return false;
    }
    std::memcpy(outPos,  sample->mData,                      sizeof(float) * 3);
    std::memcpy(outQuat, sample->mData + sizeof(float) * 3,  sizeof(float) * 4);
    return true;
}
