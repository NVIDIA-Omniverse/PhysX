// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// SPDX-FileCopyrightText: Copyright (c) 2008-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0


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
