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

#pragma once

#include "PvdDom.h"

#include <cstring>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
// PvdDomQuery -- read-side helpers for walking an OmniPvdDOMState.
//
// These utilities are layered on top of the raw DOM structures in PvdDom.h.
// They encapsulate the common lookup patterns needed when consuming an OVD
// capture: finding a class by name, resolving an attribute across an
// inheritance chain, enumerating alive objects at a given frame, fetching the
// latest attribute sample at/before a frame timestamp, resolving object
// references, and reading typed values out of a sample blob.
//
// All functions are non-owning and perform no allocation beyond the returned
// std::vector in the two enumeration helpers.
////////////////////////////////////////////////////////////////////////////////

// --- class / attribute lookup ------------------------------------------------

// Find a class by its schema name.  Returns nullptr if not registered.
OmniPvdClass* findClassByName(OmniPvdDOMState& domState, const char* className);

// Walk the inheritance chain of `cls` (root -> leaf) looking for an attribute
// whose name matches `attrName`.  On success returns the attribute def and
// (optionally) sets *outClassIndex / *outAttrIndex to its position in the
// chain / local attribute vector.  Returns nullptr when not found.
OmniPvdAttributeDef* findAttributeOnChain(OmniPvdClass* cls,
                                          const char* attrName,
                                          int32_t* outClassIndex = nullptr,
                                          int32_t* outAttrIndex = nullptr);

// Concatenate mAttributeDefinitions from every class in cls->mInheritanceChain
// (root first, leaf last).  Mirrors what PhysX's property browser shows when
// flattening inherited attributes.
std::vector<OmniPvdAttributeDef*> flattenInheritedAttributes(OmniPvdClass* cls);

// --- lifespan / frame queries ------------------------------------------------

// True iff `obj` has any lifespan covering `frame`.
// A span with mFrameStop == 0 is treated as open-ended (still alive).
bool isObjectAliveAtFrame(const OmniPvdObject* obj, uint64_t frame);

// All objects in `domState` alive at `frame`, in insertion order.
std::vector<OmniPvdObject*> getAliveObjectsAtFrame(OmniPvdDOMState& domState, uint64_t frame);

// Latest sample in `list` whose mTimeStamp is <= `frameTimestamp`.
// Returns nullptr if the list is empty or all samples post-date the frame.
OmniPvdAttributeSample* getLatestSampleAtFrame(OmniPvdAttributeInstList* list,
                                               uint64_t frameTimestamp);

// Convenience: find the attribute on the object's class chain, then fetch the
// latest sample at/before `frameTimestamp`.  Returns nullptr if either lookup
// fails.
OmniPvdAttributeSample* getLatestSampleAtFrame(OmniPvdObject* obj,
                                               const char* attrName,
                                               uint64_t frameTimestamp);

// --- reference resolution ----------------------------------------------------

// Resolve an eOBJECT_HANDLE payload (the external API handle -- typically a
// PhysX pointer value) to the DOM object it refers to, or nullptr.
OmniPvdObject* resolveObjectReference(OmniPvdDOMState& domState, uint64_t apiHandle);

// --- typed sample reads ------------------------------------------------------

// Copy a plain-old-data value out of `sample`.  Returns false if sample is
// null or its payload is smaller than sizeof(T).
template <typename T>
bool readAttributeAs(const OmniPvdAttributeSample* sample, T& out)
{
    if (!sample || !sample->mData || sample->mDataLen < sizeof(T))
    {
        return false;
    }
    std::memcpy(&out, sample->mData, sizeof(T));
    return true;
}

// Read a 3-float vector.  Returns false if sample payload is too short.
bool readVec3(const OmniPvdAttributeSample* sample, float out[3]);

// Read a PxTransform-shaped sample: 3 floats position, 4 floats quaternion.
// Layout matches how the pvdruntime writes PxTransform (pos then quat).
// Returns false if sample payload is too short.
bool readTransform(const OmniPvdAttributeSample* sample, float outPos[3], float outQuat[4]);
