// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "DebugVisualizationVehicle.h"
#include "DebugVisualization.h"

#include <carb/Defines.h>
#include <carb/Framework.h>
#include <carb/profiler/Profile.h>

#include <private/omni/physx/PhysxUsd.h>


extern omni::physx::usdparser::IPhysxUsdLoad* gUsdLoad;


namespace omni
{
namespace physx
{
namespace ui
{

uint32_t convertColor(uint32_t inColor);


DebugVisualizationVehicle::DebugVisualizationVehicle(pxr::UsdGeomXformCache& xformcache)
    : mXformCache(xformcache)
    , mLineBuffer(::omni::renderer::IDebugDraw::eInvalidBuffer)
    , mLineRenderInstanceBuffer(::omni::renderer::IDebugDraw::eInvalidBuffer)
    , mLineIndex(0)
    , mMaxLineBufferSize(0)
    , mVisualizationFlags(0)
{
    mVehicleComponentTrackerHandle = gUsdLoad->createVehicleComponentTracker();
    mDebugDraw = carb::getCachedInterface<omni::renderer::IDebugDraw>();
}

DebugVisualizationVehicle::~DebugVisualizationVehicle()
{
    gUsdLoad->releaseVehicleComponentTracker(mVehicleComponentTrackerHandle);
    releaseLineBuffer();
    mDebugDraw = nullptr;
}

void DebugVisualizationVehicle::setVisualization(PhysXVehicleVisualizationParameter::Enum param, bool enable)
{
    CARB_ASSERT(param < 32);

    if (enable)
        mVisualizationFlags |= (1 << param);
    else
        mVisualizationFlags &= ~(1 << param);
}

bool DebugVisualizationVehicle::getVisualization(PhysXVehicleVisualizationParameter::Enum param) const
{
    return (mVisualizationFlags & (1 << param));
}

bool DebugVisualizationVehicle::isXformPathTracked(const pxr::SdfPath& path)
{
    PathSet::const_iterator it = mVehiclePathCache.find(path);
    if (it != mVehiclePathCache.end())
        return true;

    it = mVehicleWheelPathCache.find(path);
    if (it != mVehicleWheelPathCache.end())
        return true;

    return false;
}

void DebugVisualizationVehicle::clear(bool resetDrawBuffersInsteadOfRelease)
{
    if (!resetDrawBuffersInsteadOfRelease)
        releaseLineBuffer();
    else
    {
        mLineRenderInstanceBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
        mLineBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
        mLineIndex = 0;
    }

    mDescCache.release();
    mVehiclePathCache.clear();
    mVehicleWheelPathCache.clear();

    gUsdLoad->releaseVehicleComponentTracker(mVehicleComponentTrackerHandle);
    mVehicleComponentTrackerHandle = gUsdLoad->createVehicleComponentTracker();
}

void DebugVisualizationVehicle::releaseDrawBuffersIfEmpty()
{
    if (mLineIndex == 0)
    {
        releaseLineBuffer();
    }
}

void DebugVisualizationVehicle::updateVehicle(const pxr::UsdPrim& prim,
                                              pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "DebugVisualizationVehicle::updateVehicle");

    createLineBuffer();

    // Parse the vehicle descriptor if not cached
    const pxr::SdfPath path = prim.GetPath();
    usdparser::VehicleDesc* vehicleDesc = (usdparser::VehicleDesc*)mDescCache.getDesc(path, usdparser::eVehicle);
    if (!vehicleDesc)
    {
        const uint64_t stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
        vehicleDesc = gUsdLoad->parseVehicle(stageId, path, mVehicleComponentTrackerHandle);
        if (vehicleDesc)
        {
            CARB_ASSERT(mVehiclePathCache.find(path) == mVehiclePathCache.end());
            mVehiclePathCache.insert(path);
            for (const usdparser::WheelAttachmentDesc& wheelAttDesc : vehicleDesc->wheelAttachments)
            {
                if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eMANAGE_TRANSFORMS)
                {
                    CARB_ASSERT(mVehicleWheelPathCache.find(wheelAttDesc.path) == mVehicleWheelPathCache.end());
                    mVehicleWheelPathCache.insert(wheelAttDesc.path);
                }
            }

            // Deposit it to DescCache so we don't have to parse the USD again
            mDescCache.addDesc(prim.GetPrimPath(), usdparser::eVehicle, vehicleDesc);
        }
    }

    if (vehicleDesc)
    {
        // in the future, PhysXVehicleVisualizationParameter::eSuspension etc. will have
        // to be tested in here to only visualize what was requested but at the moment
        // it's the only property, so it does not make sense to do so

        CARB_ASSERT(prim.HasAPI<pxr::UsdPhysicsMassAPI>());  // note: once referenceFrameIsCenterOfMass gets removed, this
                                                             //       will likely not be required anymore
        CARB_ASSERT(prim.IsA<pxr::UsdGeomXformable>());

        const pxr::GfMatrix4d vehiclePoseWorld = mXformCache.GetLocalToWorldTransform(prim);
        const pxr::GfTransform tr(vehiclePoseWorld);
        const pxr::GfQuatf vehicleOrientWorld(tr.GetRotation().GetQuat());
        // note: GfMatrix4d::ExtractRotationQuat() etc. gives poor results when scale is involved,
        //       thus using GfTransform

        pxr::GfVec3f vehicleRefPosWorld;
        pxr::GfRotation vehicleRefOrientWorld;

        if (vehicleDesc->referenceFrameIsCenterOfMass)  // deprecated
        {
            pxr::UsdPhysicsMassAPI massAPI(prim);
            pxr::UsdAttribute centerOfMassAttr = massAPI.GetCenterOfMassAttr();
            pxr::GfVec3f centerOfMassPosLocal;
            centerOfMassAttr.Get(&centerOfMassPosLocal);

            if ((!isfinite(centerOfMassPosLocal[0])) || (!isfinite(centerOfMassPosLocal[1])) || (!isfinite(centerOfMassPosLocal[2])))
            {
                CARB_LOG_WARN(
                    "PhysX Vehicle Debug Visualization: prim \"%s\": attribute \"centerOfMass\" does not specify an actual "
                    "center of mass, (0, 0, 0) will be used\n",
                    prim.GetPath().GetText());

                centerOfMassPosLocal[0] = 0.0f;
                centerOfMassPosLocal[1] = 0.0f;
                centerOfMassPosLocal[2] = 0.0f;
            }

            // the center-of-mass frame is relative to the actor/vehicle frame (including scale),
            // thus scale needs to be applied (using TransformAffine(), the scale will get factored
            // in)
            vehicleRefPosWorld = vehiclePoseWorld.TransformAffine(centerOfMassPosLocal);

            pxr::UsdAttribute principalAxesAttr = massAPI.GetPrincipalAxesAttr();
            pxr::GfQuatf centerOfMassOrientLocal;
            principalAxesAttr.Get(&centerOfMassOrientLocal);
            const pxr::GfVec3f& imag = centerOfMassOrientLocal.GetImaginary();
            if ((centerOfMassOrientLocal.GetReal() != 0.0f) ||
                (imag[0] != 0.0f) ||
                (imag[1] != 0.0f) ||
                (imag[2] != 0.0f))
            {
                const pxr::GfQuatf centerOfMassOrientWorld = (vehicleOrientWorld * centerOfMassOrientLocal).GetNormalized();
                vehicleRefOrientWorld.SetQuat(centerOfMassOrientWorld);
            }
            else
            {
                CARB_LOG_WARN(
                    "PhysX Vehicle Debug Visualization: prim \"%s\": attribute \"principalAxes\" does not specify a valid "
                    "rotation, identity will be used\n",
                    prim.GetPath().GetText());

                // the default/special value for principalAxesAttr is now (0, 0, 0, 0), so special
                // code is needed to assume identity rotation in this case
                vehicleRefOrientWorld = tr.GetRotation();
            }
        }
        else
        {
            vehicleRefPosWorld = pxr::GfVec3f(tr.GetTranslation());
            vehicleRefOrientWorld = tr.GetRotation();
        }

        for (const usdparser::WheelAttachmentDesc& wheelAttDesc : vehicleDesc->wheelAttachments)
        {
            const pxr::GfVec3f suspDirLocal(wheelAttDesc.suspensionTravelDirection.x,
                wheelAttDesc.suspensionTravelDirection.y, wheelAttDesc.suspensionTravelDirection.z);

            pxr::GfVec3f suspFramePosLocal;
            if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_SUSPENSION_FRAME)
            {
                // the suspension frame is relative to the center-of-mass frame or vehicle frame
                // (including scale), thus scale needs to be applied
                suspFramePosLocal = pxr::GfVec3f(
                    wheelAttDesc.suspensionFramePosition.x * vehicleDesc->scale.x,
                    wheelAttDesc.suspensionFramePosition.y * vehicleDesc->scale.y,
                    wheelAttDesc.suspensionFramePosition.z * vehicleDesc->scale.z);
            }
            else
            {
                // deprecated code path
                CARB_ASSERT(wheelAttDesc.state & usdparser::WheelAttachmentDesc::eHAS_WHEEL_COM_OFFSET);

                // the wheel center of mass offset is relative to the center-of-mass frame or vehicle frame
                // (including scale), thus scale needs to be applied. Note that travelDistance, maxCompression
                // etc. are in world scale, thus only scaling this one here.
                const pxr::GfVec3f wheelCenterOfMassLocal(
                    wheelAttDesc.wheelCenterOfMassOffset.x * vehicleDesc->scale.x,
                    wheelAttDesc.wheelCenterOfMassOffset.y * vehicleDesc->scale.y,
                    wheelAttDesc.wheelCenterOfMassOffset.z * vehicleDesc->scale.z);

                if (wheelAttDesc.suspension->travelDistance > 0.0f)
                {
                    // can't really support this weird mix of deprecated and non-deprecated attributes
                    // -> choosing "random" point
                    suspFramePosLocal = wheelCenterOfMassLocal - (suspDirLocal * (wheelAttDesc.suspension->travelDistance * 0.5f));
                }
                else
                {
                    suspFramePosLocal = wheelCenterOfMassLocal - (suspDirLocal * wheelAttDesc.suspension->maxCompression);
                }
            }

            const pxr::GfVec3f suspFramePosWorld = vehicleRefPosWorld + (vehicleRefOrientWorld.TransformDir(suspFramePosLocal));

            const pxr::GfVec3f suspDirWorld = vehicleRefOrientWorld.TransformDir(suspDirLocal);

            float suspTravelDistance;
            if (wheelAttDesc.suspension->travelDistance > 0.0f)
                suspTravelDistance = wheelAttDesc.suspension->travelDistance;
            else
            {
                // deprecated code path. Note that the case with a negative maxDroop value (auto-compute)
                // can not really be supported here as we would need to know, for example, the vehicle mass
                // which is not necessarily available during authoring time.
                suspTravelDistance = wheelAttDesc.suspension->maxCompression + CARB_MAX(wheelAttDesc.suspension->maxDroop, 0.0f);
            }

            float currentSuspDelta;
            if (wheelAttDesc.state & usdparser::WheelAttachmentDesc::eMANAGE_TRANSFORMS)
            {
                // Mapping the current wheel attachment world transform to the suspension frame.
                // We have:
                // wheelToWorld = suspensionToWorld * wheelToSuspension
                // We want:
                // suspensionToWorld = wheelToWorld * inv(wheelToSuspension)

                // the wheel frame is relative to the suspension frame. The suspension frame does
                // not define scale, so no extra scale added and the center-of-mass frame scale
                // is used.
                const pxr::GfVec3f invWheelFramePosLocal(
                    -wheelAttDesc.wheelFramePosition.x * vehicleDesc->scale.x,
                    -wheelAttDesc.wheelFramePosition.y * vehicleDesc->scale.y,
                    -wheelAttDesc.wheelFramePosition.z * vehicleDesc->scale.z);

                pxr::UsdPrim wheelAttPrim = stage->GetPrimAtPath(wheelAttDesc.path);
                const pxr::GfMatrix4d wheelPoseWorld = mXformCache.GetLocalToWorldTransform(wheelAttPrim);
                const pxr::GfTransform trw(wheelPoseWorld);
                // note: GfMatrix4d::ExtractRotationQuat() etc. gives poor results when scale is involved,
                //       thus using GfTransform
                const pxr::GfVec3f wheelPosWorld(trw.GetTranslation());
                const pxr::GfRotation wheelRotWorld = trw.GetRotation();

                const pxr::GfVec3f suspPosWorld = wheelPosWorld + (wheelRotWorld.TransformDir(invWheelFramePosLocal));

                const float currentSuspDeltaUnclamped = (suspPosWorld - suspFramePosWorld) * suspDirWorld;
                currentSuspDelta = CARB_CLAMP(currentSuspDeltaUnclamped, 0.0f, suspTravelDistance);
            }
            else
                currentSuspDelta = suspTravelDistance;

            static const uint32_t gBaseColor = 0xff00ffff;
            static const uint32_t gJounceColor = 0xffff0000;
            static const float gLineWidth = 1.0f;

            const pxr::GfVec3f maxDroopPos = suspFramePosWorld + (suspDirWorld * suspTravelDistance);
            const pxr::GfVec3f suspPos = suspFramePosWorld + (suspDirWorld * currentSuspDelta);

            const carb::Float3 carbSuspFramePosWorld = {suspFramePosWorld[0], suspFramePosWorld[1], suspFramePosWorld[2]};
            const carb::Float3 carbMaxDroopPos = {maxDroopPos[0], maxDroopPos[1], maxDroopPos[2]};
            const carb::Float3 carbSuspPos = {suspPos[0], suspPos[1], suspPos[2]};

            mDebugDraw->setLine(mLineBuffer, mLineIndex, carbSuspFramePosWorld, gBaseColor, gLineWidth, carbSuspPos, gBaseColor, gLineWidth);
            mLineIndex++;
            mDebugDraw->setLine(mLineBuffer, mLineIndex, carbSuspPos, gJounceColor, gLineWidth, carbMaxDroopPos, gJounceColor, gLineWidth);
            mLineIndex++;
        }
    }
}

void DebugVisualizationVehicle::updateTrackedVehicles(pxr::UsdStageWeakPtr stage)
{
    CARB_PROFILE_ZONE(0, "DebugVisualization::vehicleUpdate");

    releaseLineBuffer();

    for (const pxr::SdfPath& path : mVehiclePathCache)
    {
        const pxr::UsdPrim prim = stage->GetPrimAtPath(path);
        updateVehicle(prim, stage);
    }
}

void DebugVisualizationVehicle::createLineBuffer()
{
    if (mLineBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer)
    {
        mLineBuffer = mDebugDraw->allocateLineBuffer(CARB_MAX(2048, mMaxLineBufferSize));
        mLineIndex = 0;

        CARB_ASSERT(mLineRenderInstanceBuffer == ::omni::renderer::IDebugDraw::eInvalidBuffer);
        mLineRenderInstanceBuffer = mDebugDraw->allocateRenderInstanceBuffer(mLineBuffer, 1);
        float transform[16] = {};
        transform[0] = 1.f;
        transform[1 + 4] = 1.f;
        transform[2 + 8] = 1.f;
        transform[3 + 12] = 1.f;
        mDebugDraw->setRenderInstance(mLineRenderInstanceBuffer, 0, transform, 0, 0);
    }
}

void DebugVisualizationVehicle::releaseLineBuffer()
{
    if (mLineRenderInstanceBuffer != ::omni::renderer::IDebugDraw::eInvalidBuffer)
    {
        mDebugDraw->deallocateRenderInstanceBuffer(mLineRenderInstanceBuffer);
        mLineRenderInstanceBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
    }

    if (mLineBuffer != ::omni::renderer::IDebugDraw::eInvalidBuffer)
    {
        mMaxLineBufferSize = CARB_MAX(mMaxLineBufferSize, mLineIndex);
        mLineIndex = 0;

        mDebugDraw->deallocateLineBuffer(mLineBuffer);
        mLineBuffer = ::omni::renderer::IDebugDraw::eInvalidBuffer;
    }
}


} // namespace ui
} // namespace physx
} // namespace omni
