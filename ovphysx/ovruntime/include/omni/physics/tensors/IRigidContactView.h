// SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

/**
 * @implements REQ-TENSOR-CONTACT-003
 * @covers AC-1 AC-2 AC-3 AC-4
 *
 * @implements REQ-TENSOR-PATH-001
 * @covers AC-7
 *
 * @implements REQ-TENSOR-CONTACT-001
 * @covers AC-1 AC-2 AC-3 AC-6 AC-7
 *
 * @implements REQ-TENSOR-CONTACT-002
 * @covers AC-1 AC-2 AC-3
 */

#pragma once

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

    // Deprecated aliases retain their original results.
    // Deprecated alias of getNetNormalContactForces; returns normal forces only.
    virtual bool getNetContactForces(const TensorDesc* dstTensor, float dt) const
    {
        return getNetNormalContactForces(dstTensor, dt);
    }

    // Deprecated alias of getNormalContactForceMatrix; returns normal forces only.
    virtual bool getContactForceMatrix(const TensorDesc* dstTensor, float dt) const
    {
        return getNormalContactForceMatrix(dstTensor, dt);
    }

    // Deprecated alias of getNormalContactData; forces contain only normal components.
    virtual bool getContactData(const TensorDesc* contactForceTensor,
                                const TensorDesc* contactPointTensor,
                                const TensorDesc* contactNormalTensor,
                                const TensorDesc* contactSeparationTensor,
                                const TensorDesc* contactCountTensor,
                                const TensorDesc* contactStartIndicesTensor,
                                uint32_t* outRequiredContactCount,
                                float dt) const
    {
        return getNormalContactData(contactForceTensor, contactPointTensor, contactNormalTensor,
                                    contactSeparationTensor, contactCountTensor, contactStartIndicesTensor,
                                    outRequiredContactCount, dt);
    }

    // Deprecated alias of getFrictionContactData.
    virtual bool getFrictionData(const TensorDesc* frictionForceTensor,
                                 const TensorDesc* contactPointTensor,
                                 const TensorDesc* contactCountTensor,
                                 const TensorDesc* contactStartIndicesTensor,
                                 uint32_t* outRequiredFrictionCount,
                                 float dt) const
    {
        return getFrictionContactData(frictionForceTensor, contactPointTensor, contactCountTensor,
                                      contactStartIndicesTensor, outRequiredFrictionCount, dt);
    }

    // raw contact data for all contacts per sensor (no filter required), with both
    // the sensor's own actor ID and the contacting body's actor ID, aligned 1:1 with
    // every contact entry.
    //
    // The two pairs that are only meaningful together travel together, as columns of
    // one tensor rather than as separate buffers the caller has to keep in step:
    //   sensorLayoutTensor[i] = { count, startIndex } for sensor i
    //   actorIdsTensor[k]     = { sensorActorId, otherActorId } for contact k
    // The four per-contact value tensors stay separate, matching getNormalContactData's
    // shape convention -- they are distinct physical quantities, not a pair.
    //
    // Contacts are grouped by sensor: sensor i owns the flat range
    // [start, start + count) from its sensorLayoutTensor row. If the step produces
    // more contacts than getMaxContactDataCount(), the excess is dropped with a
    // logged warning -- counts report only what was written and start indices are
    // clamped to capacity, so the range stays valid (possibly empty) for every sensor.
    // outRequiredContactCount receives the total number of contacts produced before
    // truncation, and therefore the capacity needed for a complete read.
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
                                   uint32_t* outRequiredContactCount,
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

    // World-space normal contact force per sensor, shape (getSensorCount(), 3).
    // Includes forces from all reported contacts with other bodies, including
    // bodies not matched by filters, independently of detailed-contact capacity.
    // Reported impulses are divided by dt, so dt = 1 returns impulses.
    virtual bool getNetNormalContactForces(const TensorDesc* dstTensor, float dt) const = 0;

    // World-space friction force per sensor, shape (getSensorCount(), 3).
    // Sums reported friction-anchor impulses from contacts with all other bodies,
    // including bodies not matched by filters, then divides by dt. Filters and
    // detailed-contact capacity are not required. Adding the normal and friction
    // results from the same step and dt yields the total contact force.
    // Opposing anchor impulses may cancel while producing torque.
    // This vector sum does not report friction torque.
    virtual bool getNetFrictionContactForces(const TensorDesc* dstTensor, float dt) const = 0;

    // World-space normal force per matching sensor/filter pair, shape
    // (getSensorCount(), getFilterCount(), 3), independent of detailed-contact
    // capacity, with the same dt scaling as getNetNormalContactForces.
    virtual bool getNormalContactForceMatrix(const TensorDesc* dstTensor, float dt) const = 0;

    // World-space friction force per matching sensor/filter pair, shape
    // (getSensorCount(), getFilterCount(), 3). Sums all reported friction-anchor
    // impulses for each pair and divides by dt, independently of detailed-contact
    // capacity. Adding the normal and friction matrices from the same step and
    // dt yields total contact forces for the configured pairs.
    virtual bool getFrictionContactForceMatrix(const TensorDesc* dstTensor, float dt) const = 0;

    // Normal contact data per filter per sensor. Multiply each scalar force by
    // its corresponding normal to obtain the world-space normal force vector.
    // Friction forces are returned separately by getFrictionContactData.
    // outRequiredContactCount receives the total contacts produced before truncation.
    // When that total exceeds getMaxContactDataCount(), the excess is dropped with a
    // logged warning -- counts report only what was written and start indices are
    // clamped to capacity, so [start, start + count) stays valid (possibly empty)
    // for every (sensor, filter) pair. Forces are scaled by 1/dt.
    virtual bool getNormalContactData(const TensorDesc* contactForceTensor,        // (getMaxContactDataCount(), 1)
                                      const TensorDesc* contactPointTensor,        // (getMaxContactDataCount(), 3)
                                      const TensorDesc* contactNormalTensor,       // (getMaxContactDataCount(), 3)
                                      const TensorDesc* contactSeparationTensor,   // (getMaxContactDataCount(), 1)
                                      const TensorDesc* contactCountTensor,        // (getSensorCount(), getFilterCount())
                                      const TensorDesc* contactStartIndicesTensor, // (getSensorCount(), getFilterCount())
                                      uint32_t* outRequiredContactCount,
                                      float dt) const = 0;

    // World-space friction force vectors and anchor points per filter per sensor.
    // Friction anchors are points used by the solver to apply friction impulses.
    // contactPointTensor[i] contains the position of anchor i; frictionForceTensor[i]
    // contains the force at that anchor (the reported impulse divided by dt).
    // Anchors need not correspond one-to-one with getNormalContactData's contact points.
    // contactCountTensor and contactStartIndicesTensor describe the anchor entries
    // belonging to each sensor/filter pair, independently of the normal-contact layout.
    // outRequiredFrictionCount receives the total friction anchors produced before
    // truncation, with the same in-range prefix contract as getNormalContactData.
    virtual bool getFrictionContactData(const TensorDesc* frictionForceTensor,       // (getMaxContactDataCount(), 3)
                                        const TensorDesc* contactPointTensor,        // (getMaxContactDataCount(), 3)
                                        const TensorDesc* contactCountTensor,        // (getSensorCount(), getFilterCount())
                                        const TensorDesc* contactStartIndicesTensor, // (getSensorCount(), getFilterCount())
                                        uint32_t* outRequiredFrictionCount,
                                        float dt) const = 0;

protected:
    virtual ~IRigidContactView() = default;
};

} // namespace tensors
} // namespace physics
} // namespace omni
