// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#define CARB_EXPORTS

#include "VehicleManager.h"

#include <carb/Framework.h>
#include <carb/input/IInput.h>
#include <carb/events/EventsUtils.h>
#include <carb/PluginUtils.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxVehicle.h>
#include <private/omni/physx/PhysxUsd.h>

#include <omni/kit/IStageUpdate.h>
#include <omni/kit/KitUpdateOrder.h>

#include <PxPhysicsAPI.h>

#include <common/foundation/Allocator.h>
#include <common/foundation/TypeCast.h>
#include <common/utilities/PhysXErrorCallback.h>
#include <common/utilities/Utilities.h>


using namespace omni;
using namespace omni::physx;
using namespace pxr;


omni::physx::IPhysx* gPhysXInterface = nullptr;
UsdStageRefPtr gStage = nullptr;
omni::kit::StageUpdatePtr gStageUpdate;
omni::kit::StageUpdateNode* gStageUpdateNodePrePhysics = nullptr;
omni::kit::StageUpdateNode* gStageUpdateNodePostPhysics = nullptr;

omni::physx::VehicleManager* gVehicleManager = nullptr;

static carb::events::IEventStreamPtr gErrorEventStream;
static ::physx::PxDefaultAllocator gAllocator;
static CarbPhysXErrorCallback gErrorCallback;
static ::physx::PxFoundation* gFoundation = nullptr;
static SubscriptionId gPhysicsObjectChangeNotifySubscription;

struct RunState
{
    enum Enum
    {
        ePLAYING,
        ePAUSED,
        eSTOPPED
    };
};

static int gRunState = RunState::eSTOPPED;

typedef pxr::TfHashSet<pxr::SdfPath, pxr::SdfPath::Hash> PrimAddMap;


static void onPhysicsObjectCreation(const pxr::SdfPath& sdfPath, omni::physx::usdparser::ObjectId objectId,
    omni::physx::PhysXType type, void* userData)
{
    if (type == omni::physx::ePTVehicleController)
    {
        const pxr::UsdPrim& prim = gStage->GetPrimAtPath(sdfPath);

        CARB_ASSERT(userData);
        omni::physx::VehicleManager* vehicleManager = reinterpret_cast<omni::physx::VehicleManager*>(userData);

        vehicleManager->addController(prim, objectId);
    }
}

static void onPhysicsObjectDestruction(const pxr::SdfPath& sdfPath, omni::physx::usdparser::ObjectId objectId,
    omni::physx::PhysXType type, void* userData)
{
    if ((type == omni::physx::ePTVehicle) || (type == omni::physx::ePTVehicleController))
    {
        // checking for ePTVehicle too as removing that API invalidates the assumption that
        // every vehicle controller is a vehicle (used when switching between enabled/disabled,
        // for example). It is safe to call this multiple times for the same path since the
        // remove code will check if there is a matching controller in the list first.

        CARB_ASSERT(userData);
        omni::physx::VehicleManager* vehicleManager = reinterpret_cast<omni::physx::VehicleManager*>(userData);

        vehicleManager->removeController(sdfPath);
    }
}

static void onPhysicsAllObjectsDestruction(void* userData)
{
    CARB_ASSERT(userData);
    omni::physx::VehicleManager* vehicleManager = reinterpret_cast<omni::physx::VehicleManager*>(userData);

    vehicleManager->releaseControllers();
}


const struct carb::PluginImplDesc kPluginImpl = { "omni.physx.vehicle.plugin", "PhysX Vehicle", "NVIDIA",
                                                  carb::PluginHotReload::eDisabled, "dev" };

CARB_PLUGIN_IMPL(kPluginImpl, omni::physx::IPhysxVehicle)
CARB_PLUGIN_IMPL_DEPS(omni::kit::IStageUpdate,
                      omni::physx::IPhysx, 
                      carb::input::IInput)


struct UsdNoticeListener : public pxr::TfWeakBase
{
    UsdNoticeListener() = default;

    void handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged);
};

void updateControllers(float elapsedSecs)
{
    if (gVehicleManager)
    {
        gVehicleManager->update(elapsedSecs, pxr::UsdTimeCode::Default());
    }
}

void onAttach(long int stageId, double metersPerUnit, void*)
{
    gStage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
}

void onDetach(void*)
{
    if (gVehicleManager)
    {
        gVehicleManager->onStop();
        gVehicleManager->releaseVehicleSettings();
    }

    gStage = nullptr;
}

void onUpdatePrePhysics(float currentTime, float elapsedSecs, const omni::kit::StageUpdateSettings*, void*)
{
    if (gRunState == RunState::ePLAYING)
    {
        updateControllers(elapsedSecs);
    }
}

void onResume(float currentTime, void* userData)
{
    if (gVehicleManager)
    {
        gVehicleManager->onResume();
    }

    gRunState = RunState::ePLAYING;
}

void onPause(void* userData)
{
    gRunState = RunState::ePAUSED;
}

void onStop(void* userData)
{
    gRunState = RunState::eSTOPPED;

    if (gVehicleManager)
    {
        gVehicleManager->onStop();
    }
}

void UsdNoticeListener::handle(const class pxr::UsdNotice::ObjectsChanged& objectsChanged)
{
    CARB_PROFILE_ZONE(0, "PhysXVehicleUsdNoticeListener");
    TRACE_FUNCTION();

    if (!gVehicleManager ||
        gVehicleManager->hasSimulationStarted() == false)
    {
        return;
    }

    // This is an old callback, ignore it
    if (!gStage || gStage != objectsChanged.GetStage())
    {
        return;
    }

    // for now this is set to default
    const pxr::UsdTimeCode timeCode = pxr::UsdTimeCode::Default();

    if (gVehicleManager->hasUsdChangeListeners())
    {
        for (const SdfPath& path : objectsChanged.GetChangedInfoOnlyPaths())
        {
            gVehicleManager->onUsdObjectChange(path, timeCode);
        }
    }
}

void registerToStageUpdate()
{
    if (gStageUpdate)
    {
        {
            omni::kit::StageUpdateNodeDesc desc = { 0 };
            desc.displayName = "PhysXVehiclePrePhysics";
            desc.order = omni::kit::update::eIUsdStageUpdatePhysicsVehicle; // this makes sure it runs before omni.physx update
            desc.onAttach = onAttach;
            desc.onDetach = onDetach;
            desc.onUpdate = onUpdatePrePhysics;
            desc.onPause = onPause;
            desc.onStop = onStop;

            gStageUpdateNodePrePhysics = gStageUpdate->createStageUpdateNode(desc);
        }

        {
            omni::kit::StageUpdateNodeDesc desc = { 0 };
            desc.displayName = "PhysXVehiclePostPhysics";
            desc.order = omni::kit::update::eIUsdStageUpdateVehiclePostPhysics; // this makes sure it runs after omni.physx update
            desc.onResume = onResume;
            // omni.physx creates the PhysX objects that are needed in the vehicle controllers.
            // Thus, creation of controllers should happen afterwards.

            gStageUpdateNodePostPhysics = gStageUpdate->createStageUpdateNode(desc);
        }
    }
}

void unregisterFromStageUpdate()
{
    if (gStageUpdate)
    {
        if (gStageUpdateNodePrePhysics)
        {
            gStageUpdate->destroyStageUpdateNode(gStageUpdateNodePrePhysics);
            gStageUpdateNodePrePhysics = nullptr;
        }

        if (gStageUpdateNodePostPhysics)
        {
            gStageUpdate->destroyStageUpdateNode(gStageUpdateNodePostPhysics);
            gStageUpdateNodePostPhysics = nullptr;
        }
    }
}

pxr::TfNotice::Key gUsdNoticeListenerKey;
UsdNoticeListener* gUsdNoticeListener = nullptr;

CARB_EXPORT void carbOnPluginStartup()
{
    carb::Framework* framework = carb::getFramework();
    gPhysXInterface = carb::getCachedInterface<omni::physx::IPhysx>();

    gErrorEventStream = carb::events::getCachedEventsInterface()->createEventStream();
    gErrorCallback.setEventStream(gErrorEventStream);
    gErrorCallback.setErrorEventType(omni::physx::ePhysxError);
    // create PxFoundation since this extension is using some PhysX methods that can send error
    // messages etc.
    gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);

    gStageUpdate =  omni::kit::getStageUpdate();
    registerToStageUpdate();

    gUsdNoticeListener = ICE_PLACEMENT_NEW(UsdNoticeListener)();
    gUsdNoticeListenerKey = pxr::TfNotice::Register(pxr::TfCreateWeakPtr(gUsdNoticeListener), &UsdNoticeListener::handle);

    gVehicleManager = ICE_PLACEMENT_NEW(VehicleManager)();
    if (gVehicleManager)
    {
        IPhysicsObjectChangeCallback callback;
        callback.objectCreationNotifyFn = onPhysicsObjectCreation;
        callback.objectDestructionNotifyFn = onPhysicsObjectDestruction;
        callback.allObjectsDestructionNotifyFn = onPhysicsAllObjectsDestruction;
        callback.userData = gVehicleManager;
        gPhysicsObjectChangeNotifySubscription = gPhysXInterface->subscribeObjectChangeNotifications(callback);
    }
}

CARB_EXPORT void carbOnPluginShutdown()
{
    if (gVehicleManager)
    {
        gPhysXInterface->unsubscribeObjectChangeNotifications(gPhysicsObjectChangeNotifySubscription);
        ICE_PLACEMENT_DELETE(gVehicleManager, VehicleManager);
    }

    pxr::TfNotice::Revoke(gUsdNoticeListenerKey);
    ICE_PLACEMENT_DELETE(gUsdNoticeListener, UsdNoticeListener);
    gUsdNoticeListener = nullptr;

    // note: aquired interfaces are not released here as this causes warning messages "...already been released"

    unregisterFromStageUpdate();

    if (gFoundation)
    {
        gFoundation->release();
        gFoundation = nullptr;
    }
    gErrorCallback.invalidateEventStream();
    gErrorEventStream = nullptr;
}

bool getInputEnabled(const char* path)
{
    return gVehicleManager->getInputEnabled(pxr::SdfPath(path));
}

void setInputEnabled(const char* path, bool inputEnabled)
{
    gVehicleManager->setInputEnabled(pxr::SdfPath(path), inputEnabled);
}

bool getMouseEnabled(const char* path)
{
    return gVehicleManager->getMouseEnabled(pxr::SdfPath(path));
}

void setMouseEnabled(const char* path, bool mouseEnabled)
{
    gVehicleManager->setMouseEnabled(pxr::SdfPath(path), mouseEnabled);
}

bool getAutoReverseEnabled(const char* path)
{
    return gVehicleManager->getAutoReverseEnabled(pxr::SdfPath(path));
}

void setAutoReverseEnabled(const char* path, bool autoReverseEnabled)
{
    gVehicleManager->setAutoReverseEnabled(pxr::SdfPath(path), autoReverseEnabled);
}

float getSteeringSensitivity(const char* path)
{
    return gVehicleManager->getSteeringSensitivity(pxr::SdfPath(path));
}

void setSteeringSensitivity(const char* path, float steeringSensitivity)
{
    if (steeringSensitivity < 1.0f)
    {
        CARB_LOG_ERROR("IPhysxVehicle::setSteeringSensitivity: steeringSensitivity must be 1.0 or greater.");
        return;
    }

    gVehicleManager->setSteeringSensitivity(pxr::SdfPath(path), steeringSensitivity);
}

float getSteeringFilterTime(const char* path)
{
    return gVehicleManager->getSteeringFilterTime(pxr::SdfPath(path));
}

void setSteeringFilterTime(const char* path, float steeringFilterTime)
{
    if (steeringFilterTime < 0.0f)
    {
        CARB_LOG_ERROR("IPhysxVehicle::setSteeringFilterTime: steeringFilterTime must be 0.0 or greater.");
        return;
    }

    gVehicleManager->setSteeringFilterTime(pxr::SdfPath(path), steeringFilterTime);
}

float computeAckermannSteeringAngle(const float steerAngle,
                                    const float axleSeparation,
                                    const float axleWidth)
{
    CARB_ASSERT(steerAngle <= ::physx::PxHalfPi);
    CARB_ASSERT(steerAngle >= -::physx::PxHalfPi);

    /*
    //
    //                              axle width
    //                            |------------|
    //
    //
    //                    inner wheel      outer wheel
    //                   (steer angle)
    //
    //                         \\            \\
    //                         _ \\         _ \\        -
    //                      __/    \\  ____/   \\       |
    //                   __/      ____/                 |
    //                __/    ____/                      | axle separation
    //             __/  ____/                           |
    //            /____/         ||           ||        |
    //  ---------x---------------||-----------||------------
    //                           ||           ||
    //   center of
    //   turning circle
    //
    //           |---------------|
    //           turningCircleCenterToInnerWheel
    //
    */

    if (steerAngle != 0.0f)
    {
        // note: faster approximate functions for tan/atan could be used as results don't need to be too accurate here.

        const float turningCircleCenterToInnerWheel = axleSeparation / tanf(fabsf(steerAngle));
        const float turningCircleCenterToOuterWheel = turningCircleCenterToInnerWheel + axleWidth;
        const float outerWheelSteerAngle = atanf(axleSeparation / turningCircleCenterToOuterWheel);

        if (steerAngle > 0.0f)
            return outerWheelSteerAngle;
        else
            return -outerWheelSteerAngle;
    }
    else
        return 0.0f;
}

bool computeSprungMasses(float mass, uint32_t upAxisIndex, const carb::Float3* positions, uint32_t positionCount,
    float* sprungMasses)
{
    if (positionCount <= usdparser::VehicleDesc::maxNumberOfWheels)
    {
        static_assert(sizeof(::physx::PxVec3) == sizeof(carb::Float3), "Size of PxVec3 and Float3 is expected to match");
        const ::physx::PxVec3* sprungMassPositions = reinterpret_cast<const ::physx::PxVec3*>(positions);

        // to ensure the index can be mapped by doing a bit shift to the left (equivalent to multiplication by 2).
        // Note that for the purpose of computing the sprung mass values, ePosX and eNegX etc. are equivalent
        static_assert(::physx::vehicle2::PxVehicleAxes::ePosX == 0, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::eNegX == 1, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::ePosY == 2, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::eNegY == 3, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::ePosZ == 4, "");
        static_assert(::physx::vehicle2::PxVehicleAxes::eNegZ == 5, "");
        const ::physx::vehicle2::PxVehicleAxes::Enum upAxisEnum = static_cast<const ::physx::vehicle2::PxVehicleAxes::Enum>(
            upAxisIndex << 1);

        if (::physx::vehicle2::PxVehicleComputeSprungMasses(positionCount, sprungMassPositions, mass, upAxisEnum, sprungMasses))
            return true;
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: computeSprungMasses(): sprung mass computation failed. Setup might be ill-conditioned. "
                "Make sure the origin is enclosed by provided sprung mass positions.\n");
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: computeSprungMasses(): the maximum number of supported positions is %d.\n",
            usdparser::VehicleDesc::maxNumberOfWheels);
    }

    return false;
}

static UsdPrim getVehiclePrim(const pxr::UsdPrim& wheelAttachmentPrim)
{
    UsdPrim vehiclePrim = wheelAttachmentPrim.GetParent();

    while (!vehiclePrim.IsPseudoRoot())
    {
        if (vehiclePrim.HasAPI<PhysxSchemaPhysxVehicleAPI>())
        {
            break;
        }

        vehiclePrim = vehiclePrim.GetParent();
    }

    CARB_ASSERT(!vehiclePrim.IsPseudoRoot());
    CARB_ASSERT(vehiclePrim.HasAPI<PhysxSchemaPhysxVehicleAPI>()); // else, loading the stage should have failed

    return vehiclePrim;
}

static bool computeSuspensionFrameTransformsUSDInternal(pxr::UsdPrim& wheelAttachmentPrim,
                                                        const ::physx::PxTransform& vehicleRefTransformWorld,
                                                        const GfVec3f& vehicleScale,
                                                        UsdGeomXformCache& xfCache)
{
    CARB_ASSERT(wheelAttachmentPrim.HasAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>()); // else this method should not
                                                                                          // get called

    if (wheelAttachmentPrim.IsA<UsdGeomXformable>())
    {
        GfMatrix4d primToWorld = xfCache.GetLocalToWorldTransform(wheelAttachmentPrim);

        const pxr::GfTransform tr(primToWorld);
        // note: GfMatrix4d::ExtractRotationQuat() etc. gives poor results when scale is involved,
        //       thus using GfTransform
        ::physx::PxTransform wheelToWorld;
        wheelToWorld.p = toPhysX(tr.GetTranslation());
        wheelToWorld.q = toPhysX(tr.GetRotation().GetQuat());

        // the assumption is that the given USD wheel attachment pose includes the wheel frame.
        // We want to compute what is called the "suspension frame" in vehicle 2.0:
        // We have:
        // wheelToWorld = vehicleToWorld * suspensionToVehicle * wheelToSuspension
        // -> suspensionToVehicle = inv(vehicleToWorld) * wheelToWorld * inv(wheelToSuspension)

        PhysxSchemaPhysxVehicleWheelAttachmentAPI wheelAttachment(wheelAttachmentPrim);

        GfVec3f gfPosf;
        wheelAttachment.GetWheelFramePositionAttr().Get(&gfPosf);
        GfQuatf gfQuatf;
        wheelAttachment.GetWheelFrameOrientationAttr().Get(&gfQuatf);
        ::physx::PxTransform wheelToSuspension(toPhysX(gfPosf), toPhysX(gfQuatf));

        // the wheel frame is defined relative to the suspension frame. The suspension frame is
        // defined relative to the vehicle reference frame which includes scale. That scale
        // also gets applied here since the following calculations expect everything being in
        // the same "global scale" (note that the suspension frame itself does not define a
        // scale, so just the vehicle scale is used)
        wheelToSuspension.p.x *= vehicleScale[0];
        wheelToSuspension.p.y *= vehicleScale[1];
        wheelToSuspension.p.z *= vehicleScale[2];

        ::physx::PxTransform suspensionToVehicle = vehicleRefTransformWorld.getInverse() * wheelToWorld * wheelToSuspension.getInverse();

        // the scale has to be taken into account for the USD definition since the suspension frame is
        // relative to the vehicle frame (including scale)

        GfVec3f gfLocalPos(
            suspensionToVehicle.p.x / vehicleScale[0],
            suspensionToVehicle.p.y / vehicleScale[1],
            suspensionToVehicle.p.z / vehicleScale[2]
        );

        UsdAttribute attr;

        attr = wheelAttachment.GetSuspensionFramePositionAttr();
        if (attr.HasAuthoredValue())
            attr.Set(gfLocalPos);

        suspensionToVehicle.q.normalize();
        GfQuatf gfLocalOrient(suspensionToVehicle.q.w, suspensionToVehicle.q.x, suspensionToVehicle.q.y, suspensionToVehicle.q.z);
        attr = wheelAttachment.GetSuspensionFrameOrientationAttr();
        if (attr.HasValue())
            attr.Set(gfLocalOrient);

        return true;
    }
    else
    {
        CARB_LOG_ERROR(
            "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" has to inherit from \"Xformable\"\n",
            wheelAttachmentPrim.GetPath().GetText());

        return false;
    }
}

static bool computeSuspensionFrameTransformsUSDInternal(pxr::UsdPrim& vehiclePrim, pxr::UsdPrim* wheelAttachmentPrim)
{
    if (!vehiclePrim.IsA<UsdGeomXformable>())
    {
        CARB_LOG_ERROR(
            "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" must inherit from \"Xformable\"\n",
            vehiclePrim.GetPath().GetText());

        return false;
    }

    UsdGeomXformCache xfCache;
    GfMatrix4d primToWorld = xfCache.GetLocalToWorldTransform(vehiclePrim);

    const GfTransform tr(primToWorld);
    // note: GfMatrix4d::ExtractRotationQuat() etc. gives poor results when scale is involved,
    //       thus using GfTransform
    const GfQuatf vehicleOrientWorld(tr.GetRotation().GetQuat());
    const GfVec3f scale = GfVec3f(tr.GetScale());

    if ((scale[0] == 0.0f) || (scale[1] == 0.0f) || (scale[2] == 0.0f))
    {
        CARB_LOG_ERROR(
            "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" must have non zero total scale on each axis\n",
            vehiclePrim.GetPath().GetText());

        return false;
    }

    if (!scaleIsIdentity(scale[0], scale[1], scale[2]))
    {
        CARB_LOG_WARN(
            "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" has a gobal scale that is not identity. It is recommended to avoid "
            "such configurations since not all vehicle related attributes are scale aware.\n",
            vehiclePrim.GetPath().GetText());
    }

    VtValue referenceFrameIsCenterOfMassVal;
    bool referenceFrameIsCenterOfMass;
    if (vehiclePrim.GetMetadataByDictKey(pxr::SdfFieldKeys->CustomData, PhysxSchemaTokens->referenceFrameIsCenterOfMass, &referenceFrameIsCenterOfMassVal))
    {
        referenceFrameIsCenterOfMass = referenceFrameIsCenterOfMassVal.Get<bool>();

        if (referenceFrameIsCenterOfMass)
        {
            CARB_LOG_WARN("PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": the custom metadata attribute "
                "physxVehicle:referenceFrameIsCenterOfMass is set to \"True\". Using the center of mass frame as reference "
                "has been deprecated and in the future there will be no such option anymore.\n",
                vehiclePrim.GetName().GetText());
        }
    }
    else
    {
        CARB_LOG_WARN("PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": the custom metadata attribute "
            "physxVehicle:referenceFrameIsCenterOfMass is missing on this prim. A value of \"True\" is assumed for backwards "
            "compatibility reasons. Note that using the center of mass frame as reference has been deprecated and in the future "
            "there will be no such option anymore (vehicles will be treated as if this custom metadata has been set to \"False\").\n",
            vehiclePrim.GetName().GetText());

        referenceFrameIsCenterOfMass = true;
    }

    ::physx::PxTransform vehicleRefTransformWorld;

    if (referenceFrameIsCenterOfMass)  // deprecated
    {
        if (!vehiclePrim.HasAPI<UsdPhysicsMassAPI>())
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" must have API schema \"MassAPI\" applied\n",
                vehiclePrim.GetPath().GetText());

            return false;
        }

        UsdPhysicsMassAPI massAPI(vehiclePrim);

        UsdAttribute centerOfMassAttr = massAPI.GetCenterOfMassAttr();
        GfVec3f centerOfMassLocal;
        if (!centerOfMassAttr.Get(&centerOfMassLocal))
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": reading attribute \"centerOfMass\" failed\n",
                vehiclePrim.GetPath().GetText());

            return false;
        }

        if ((!isfinite(centerOfMassLocal[0])) || (!isfinite(centerOfMassLocal[1])) || (!isfinite(centerOfMassLocal[2])))
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": attribute \"centerOfMass\" must specify an actual "
                "center of mass to use this method\n",
                vehiclePrim.GetPath().GetText());

            return false;
        }

        UsdAttribute principalAxesAttr = massAPI.GetPrincipalAxesAttr();
        GfQuatf principalAxes;
        if (!principalAxesAttr.Get(&principalAxes))
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": reading attribute \"principalAxes\" failed\n",
                vehiclePrim.GetPath().GetText());

            return false;
        }

        const GfVec3f& imag = principalAxes.GetImaginary();
        if ((principalAxes.GetReal() == 0.0f) && (imag[0] == 0.0f) && (imag[1] == 0.0f) && (imag[2] == 0.0f))
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": attribute \"principalAxes\" must specify an actual "
                "rotation to use this method\n",
                vehiclePrim.GetPath().GetText());

            return false;
        }

        if ((!scaleIsUniform(scale[0], scale[1], scale[2])) &&
		    ((tr.GetScaleOrientation().GetQuaternion() != GfQuaternion::GetIdentity()) || (principalAxes != GfQuatf::GetIdentity())))
        {
            CARB_LOG_WARN(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": ScaleOrientation with respect to the vehicle prim "
                "or the center-of-mass frame is not supported. You may ignore this if the scale is close to uniform.\n",
                vehiclePrim.GetPath().GetText());
        }
    
        GfVec3f centerOfMassWorldPos = primToWorld.TransformAffine(centerOfMassLocal);  // note: takes scale into account
    
        GfQuatf rot((vehicleOrientWorld * principalAxes).GetNormalized());
        vehicleRefTransformWorld = ::physx::PxTransform(
            centerOfMassWorldPos[0], centerOfMassWorldPos[1], centerOfMassWorldPos[2],
            toPhysX(rot));
    }
    else
    {
        if ((!scaleIsUniform(scale[0], scale[1], scale[2])) &&
            (tr.GetScaleOrientation().GetQuaternion() != GfQuaternion::GetIdentity()))
        {
            CARB_LOG_WARN(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\": ScaleOrientation with respect to the vehicle prim "
                "is not supported. You may ignore this if the scale is close to uniform.\n",
                vehiclePrim.GetPath().GetText());
        }

        vehicleRefTransformWorld = ::physx::PxTransform(toPhysX(tr.GetTranslation()),
            toPhysX(vehicleOrientWorld));
    }

    bool success = true;
    if (!wheelAttachmentPrim)
    {
        UsdPrimSubtreeRange subPrims = vehiclePrim.GetDescendants();
        for (UsdPrim subPrim : subPrims)
        {
            if (subPrim.HasAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>())
            {
                success = success &&
                    computeSuspensionFrameTransformsUSDInternal(subPrim, vehicleRefTransformWorld, scale, xfCache);
            }
        }
    }
    else
    {
        success =
            computeSuspensionFrameTransformsUSDInternal(*wheelAttachmentPrim, vehicleRefTransformWorld, scale, xfCache);
    }

    return success;
}

bool computeSuspensionFrameTransformsUSD(const pxr::SdfPath& path)
{
    pxr::UsdPrim prim = gStage->GetPrimAtPath(path);
    if (prim)
    {
        if (prim.HasAPI<PhysxSchemaPhysxVehicleAPI>())
        {
            return computeSuspensionFrameTransformsUSDInternal(prim, nullptr);
        }
        else if (prim.HasAPI<PhysxSchemaPhysxVehicleWheelAttachmentAPI>())
        {
            pxr::UsdPrim vehiclePrim = getVehiclePrim(prim);
            return computeSuspensionFrameTransformsUSDInternal(vehiclePrim, &prim);
        }
        else
        {
            CARB_LOG_ERROR(
                "PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" needs to have API schema \"PhysxVehicleWheelAttachmentAPI\" or \"PhysxVehicleAPI\" applied\n",
                path.GetText());

            return false;
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: computeSuspensionFrameTransforms(): prim \"%s\" not found\n", path.GetText());

        return false;
    }
}

bool computeSuspensionFrameTransforms(const char* path)
{
    return computeSuspensionFrameTransformsUSD(pxr::SdfPath(path));
}

bool attachStage(long int stageId, bool unregFromStageUpdate)
{
    if (unregFromStageUpdate)
        unregisterFromStageUpdate();

    if (gStage)
    {
        onDetach(nullptr);
        CARB_ASSERT(!gStage);
    }

    if (stageId)
    {
        UsdStageRefPtr stage = UsdUtilsStageCache::Get().Find(UsdStageCache::Id::FromLongInt(stageId));
        
        if (stage)
        {
            gStage = stage;

            if (gVehicleManager)
            {
                gVehicleManager->onResume();
            }

            return true;
        }
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: attaching to stage failed. Fetching stage for ID %ld failed.\n", stageId);
        }
    }
    else
    {
        CARB_LOG_ERROR("PhysX Vehicle: attaching to stage failed. Invalid stage ID %ld.\n", stageId);
    }

    return false;
}

void detachStage(bool regToStageUpdate)
{
    UsdStageRefPtr stage;
    long stageId;
    if (regToStageUpdate)
    {
        stage = gStage;
        stageId = pxr::UsdUtilsStageCache::Get().GetId(stage).ToLongInt();
    }
    else
    {
        stageId = 0;
    }

    onDetach(nullptr);

    if (regToStageUpdate)
    {
        if (stage)
        {
            onAttach(stageId, pxr::UsdGeomGetStageMetersPerUnit(stage), nullptr);
        }
        registerToStageUpdate();
    }
}

void fillInterface(IPhysxVehicle& iface)
{
    iface.getInputEnabled = getInputEnabled;
    iface.setInputEnabled = setInputEnabled;

    iface.getMouseEnabled = getMouseEnabled;
    iface.setMouseEnabled = setMouseEnabled;

    iface.getAutoReverseEnabled = getAutoReverseEnabled;
    iface.setAutoReverseEnabled = setAutoReverseEnabled;

    iface.getSteeringSensitivity = getSteeringSensitivity;
    iface.setSteeringSensitivity = setSteeringSensitivity;

    iface.getSteeringFilterTime = getSteeringFilterTime;
    iface.setSteeringFilterTime = setSteeringFilterTime;

    iface.computeAckermannSteeringAngle = computeAckermannSteeringAngle;

    iface.attachStage = attachStage;
    iface.detachStage = detachStage;
    iface.updateControllers = updateControllers;

    iface.computeSprungMasses = computeSprungMasses;

    iface.computeSuspensionFrameTransforms = computeSuspensionFrameTransforms;
}
