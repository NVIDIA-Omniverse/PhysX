// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-7
 *
 * @implements REQ-TENSOR-CONTACT-001
 * @covers AC-1 AC-2 AC-6 AC-7
 */

#include "TensorDesc.h"

#include <cstdint>
#include <string>
#include <vector>

namespace omni
{
namespace physics
{
namespace tensors
{

class IRigidContactView
{
public:
    virtual uint32_t getSensorCount() const = 0;
    virtual uint32_t getFilterCount() const = 0;
    virtual uint32_t getMaxContactDataCount() const = 0;

    // net contact force per sensor, shape (getSensorCount(), 3)
    virtual bool getNetContactForces(const TensorDesc* dstTensor, float dt) const = 0;

    // contact force per filter per sensor, shape (getSensorCount(), getFilterCount(), 3)
    virtual bool getContactForceMatrix(const TensorDesc* dstTensor, float dt) const = 0;

    // contact data per filter per sensor per patch
    virtual bool getContactData(const TensorDesc* contactForceTensor, //(getSensorCount(), getFilterCount(),
                                                                      // getMaxContactDataCount(), 1)
                                const TensorDesc* contactPointTensor, //(getSensorCount(), getFilterCount(),
                                                                      // getMaxContactDataCount(), 3)
                                const TensorDesc* contactNormalTensor, //(getSensorCount(), getFilterCount(),
                                                                       // getMaxContactDataCount(), 3)
                                const TensorDesc* contactSeparationTensor, //(getSensorCount(), getFilterCount(),
                                                                           // getMaxContactDataCount(), 1)
                                const TensorDesc* contactCountTensor, //(getSensorCount(), getFilterCount())
                                const TensorDesc* contactStartIndicesTensor, //(getSensorCount(), getFilterCount())
                                float dt) const = 0;

    // friction data per filter per sensor per patch
    virtual bool getFrictionData(const TensorDesc* FrictionForceTensor, //(getSensorCount(), getFilterCount(),
                                                                        // getMaxContactDataCount(), 4, 3)
                                 const TensorDesc* contactPointTensor, //(getSensorCount(), getFilterCount(),
                                                                       // getMaxContactDataCount(), 2, 3)
                                 const TensorDesc* contactCountTensor, //(getSensorCount(), getFilterCount())
                                 const TensorDesc* contactStartIndicesTensor, //(getSensorCount(), getFilterCount())
                                 float dt) const = 0;

    // raw contact data for all contacts per sensor (no filter required), with both
    // the sensor's own actor ID and the contacting body's actor ID, aligned 1:1 with
    // every contact entry.
    //
    // The two pairs that are only meaningful together travel together, as columns of
    // one tensor rather than as separate buffers the caller has to keep in step:
    //   sensorLayoutTensor[i] = { count, startIndex } for sensor i
    //   actorIdsTensor[k]     = { sensorActorId, otherActorId } for contact k
    // The four per-contact value tensors stay separate, matching getContactData's
    // shape convention -- they are distinct physical quantities, not a pair.
    //
    // Contacts are grouped by sensor: sensor i owns the flat range
    // [start, start + count) from its sensorLayoutTensor row. If the step produces
    // more contacts than getMaxContactDataCount(), the excess is dropped with a
    // logged warning -- counts report only what was written and start indices are
    // clamped to capacity, so the range stays valid (possibly empty) for every sensor.
    //
    // Both ID columns carry the runtime's opaque object-identity handle (ADR-0019:
    // that handle is the only way the public API names an object), which for contacts
    // is the raw ObjectKey handle in both build configurations -- see CommonTypes.h's
    // asInt()/keyFromLegacyId() and ContactReport.cpp's keyToLegacyPathInt. 0 means no
    // known actor. Resolve with getOtherActorPathsFromIds() rather than path-decoding
    // an id: it is a handle, not an encoded path.
    virtual bool getRawContactData(const TensorDesc* contactForceTensor,      // (getMaxContactDataCount(), 1)
                                   const TensorDesc* contactPointTensor,      // (getMaxContactDataCount(), 3)
                                   const TensorDesc* contactNormalTensor,     // (getMaxContactDataCount(), 3)
                                   const TensorDesc* contactSeparationTensor, // (getMaxContactDataCount(), 1)
                                   const TensorDesc* sensorLayoutTensor,      // (getSensorCount(), 2) count,start
                                   const TensorDesc* actorIdsTensor,          // (getMaxContactDataCount(), 2) uint64
                                   float dt) const = 0;

    // Batch convert actor IDs from getRawContactData to USD paths. Accepts either
    // ID tensor -- both use the same namespace. An ID of 0, or one the source does
    // not recognise, yields an empty string.
    //
    // Non-zero IDs are gated on IPhysicsSource::existsBatch first, so an actor that
    // has been removed resolves to empty rather than to the path it used to name --
    // neither textFor()'s interned storage nor this view's path cache expires on its
    // own. Precision follows the backend's exists(): see REQ-TENSOR-CONTACT-001 AC-7
    // for the USD deactivation caveat.
    virtual void getOtherActorPathsFromIds(const TensorDesc* otherActorIdsTensor, std::vector<std::string>& outPaths) const = 0;

    virtual bool check() const = 0;

    virtual void release() = 0;

    // Path and name accessors return nullptr for an out-of-range sensor or filter index.
    virtual const char* getUsdPrimPath(uint32_t sensorIdx) const = 0;
    virtual const char* getUsdPrimName(uint32_t sensorIdx) const = 0;
    virtual const char* getFilterUsdPrimPath(uint32_t sensorIdx, uint32_t filterIdx) const = 0;
    virtual const char* getFilterUsdPrimName(uint32_t sensorIdx, uint32_t filterIdx) const = 0;

protected:
    virtual ~IRigidContactView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
