// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CharacterController.h"

#include <carb/logging/Log.h>
#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxCct.h>

#include <carb/Framework.h>

extern omni::physx::IPhysx* gPhysXInterface;
extern pxr::UsdStageRefPtr gStage;
extern carb::events::IEventStreamPtr gCctEventStream;
extern carb::dictionary::IDictionary* gDictionary;

using namespace physx;
using namespace pxr;

namespace omni
{
namespace physx
{

inline uint64_t asInt(const pxr::SdfPath& path)
{
    static_assert(sizeof(pxr::SdfPath) == sizeof(uint64_t), "Change to make the same size as pxr::SdfPath");
    uint64_t ret;
    std::memcpy(&ret, &path, sizeof(pxr::SdfPath));
    return ret;
}

static void createEventFromSdfPath(
    carb::events::IEventPtr& eventPtr,
    const char* itemName,    
    const SdfPath& path
)
{
    const uint64_t ui64Path = asInt(path);

    carb::dictionary::Item* item = gDictionary->createItem(eventPtr->payload, itemName, carb::dictionary::ItemType::eDictionary);
    gDictionary->setArray<int32_t>(item, (const int32_t*)&ui64Path, 2);
}

template <typename... ValuesT>
carb::events::IEventPtr createEvent(CctEvent::Enum type, ValuesT... values)
{
    using namespace carb::events;
    carb::events::IEventPtr event = carb::stealObject(
        gCctEventStream->createEventPtr(static_cast<EventType>(type), kGlobalSenderId, values...));

    return event;
}

CharacterController::CharacterController(const pxr::SdfPath& path)
    : mUsdPath(path)
    , mGravityMode(GravityMode::DISABLED)
    , mDirty(true)
    , mWorldSpaceMove(false)
    , mMove(0.0f, 0.0f, 0.0f)
    , mTimeSinceFalling(0.0f)
    , mCurrentCollisionFlags(0)
{
    mUsdPrim = gStage->GetPrimAtPath(mUsdPath);
}

CharacterController::~CharacterController()
{
}

void CharacterController::readUsdControls()
{
    PhysxSchemaPhysxCharacterControllerAPI cctAPI = PhysxSchemaPhysxCharacterControllerAPI(mUsdPrim);
    if (cctAPI)
    {
        cctAPI.GetMoveTargetAttr().Get(&mMove);
    }
}

void CharacterController::update(float timeStep, const unsigned char stageUpAxisIndex)
{
    const void* cctPtr = gPhysXInterface->getPhysXPtr(mUsdPath, ePTCct);
    if (!cctPtr)
        return;

    if (mDirty)
        readUsdControls();

    PxController* cct = (PxController*)cctPtr;
    PxControllerFilters filters;
    PxVec3 moveDir;

    if (mWorldSpaceMove)
    {
        moveDir = PxVec3(mMove[0], mMove[1], mMove[2]);
    }
    else
    {
        PxVec3 fwd, right;
        getLocalMoveFrame(fwd, right, stageUpAxisIndex);
        moveDir = fwd * mMove[0] + right * mMove[1] + cct->getUpDirection() * mMove[2];
    }

    if (hasGravityEnabled())
    {
        mTimeSinceFalling += timeStep;

        // Euler integrated delta move is velocity * dt, velocity is gravity * timeSinceFalling.
        if (mGravityMode == GravityMode::APPLY_BASE)
        {
            moveDir += cct->getActor()->getScene()->getGravity() * mTimeSinceFalling * timeStep;
        }
        else
        {
            moveDir += mCustomGravity * mTimeSinceFalling * timeStep;
        }
    }

    const PxControllerCollisionFlags collisionFlags = cct->move(moveDir, 0.0f, timeStep, filters);
    sendCollisionFlagEvents(collisionFlags);
						
	// reset falling counter as long as we are standing on something
    if (collisionFlags & PxControllerCollisionFlag::eCOLLISION_DOWN)
        mTimeSinceFalling = 0.0f;
}

void checkFlag(const PxControllerCollisionFlags flags, const PxControllerCollisionFlags currentFlags, PxControllerCollisionFlag::Enum collisionFlag, const SdfPath& path, CctEvent::Enum event)
{    
    if ((flags & collisionFlag) && !(currentFlags & collisionFlag))
    {
        carb::events::IEventPtr eventPtr = createEvent(event, std::make_pair("collision", true));
        createEventFromSdfPath(eventPtr, "cctPath", path);
        gCctEventStream->push(eventPtr.get());
    }
    else if (!(flags & collisionFlag) && (currentFlags & collisionFlag))
    {
        carb::events::IEventPtr eventPtr = createEvent(event, std::make_pair("collision", false));
        createEventFromSdfPath(eventPtr, "cctPath", path);
        gCctEventStream->push(eventPtr.get());
    }
}

void CharacterController::sendCollisionFlagEvents(const PxControllerCollisionFlags flags)
{
    checkFlag(flags, mCurrentCollisionFlags, PxControllerCollisionFlag::eCOLLISION_DOWN, mUsdPath, CctEvent::eCollisionDown);
    checkFlag(flags, mCurrentCollisionFlags, PxControllerCollisionFlag::eCOLLISION_UP, mUsdPath, CctEvent::eCollisionUp);
    checkFlag(flags, mCurrentCollisionFlags, PxControllerCollisionFlag::eCOLLISION_SIDES, mUsdPath, CctEvent::eCollisionSides);

    mCurrentCollisionFlags = flags;
}

void CharacterController::getLocalMoveFrame(PxVec3& fwd, PxVec3& right, const unsigned char stageUpAxisIndex)
{
    if (isFirstPerson())
    {
        // from camera
        auto cameraPrim = gStage->GetPrimAtPath(mCameraPath);

        UsdGeomXformCache xfCache;
        const GfMatrix4d m = xfCache.GetLocalToWorldTransform(cameraPrim);

        auto rm = m.ExtractRotationMatrix();
        auto camFwd = -rm.GetRow(2);
        auto camRight = rm.GetRow(0);

        fwd = PxVec3(float(camFwd[0]), float(camFwd[1]), float(camFwd[2]));
        right = PxVec3(float(camRight[0]), float(camRight[1]), float(camRight[2]));
    }
    else
    {
        // from capsule
        UsdGeomXformCache xfCache;
        const GfMatrix4d tm = xfCache.GetLocalToWorldTransform(mUsdPrim);
        const pxr::GfMatrix3d rm = tm.GetOrthonormalized().ExtractRotation();

        pxr::GfVec3d gfForward(0), gfRight(0);

        if (stageUpAxisIndex == 1)
        {
            gfForward = -rm.GetColumn(2);
            gfRight = rm.GetColumn(0);
        }
        else if (stageUpAxisIndex == 2)
        {
            gfForward = -rm.GetColumn(0);
            gfRight = rm.GetColumn(1);
        }
        else
        {
            CARB_LOG_ERROR("Character Controller: Stage up axis is %d. Should be 1 or 2 (Y or Z)", stageUpAxisIndex);
        }

        fwd = PxVec3((float)(gfForward[0]), (float)(gfForward[1]), (float)(gfForward[2]));
        right = PxVec3((float)(gfRight[0]), (float)(gfRight[1]), (float)(gfRight[2]));
    }

    if (hasGravityEnabled())
    {
        // zero out potential vertical movement
        fwd[stageUpAxisIndex] = 0.f;
        right[stageUpAxisIndex] = 0.f;
        fwd.normalize();
        right.normalize();
    }
}

float CharacterController::getHeight()
{
    const void* cctPtr = gPhysXInterface->getPhysXPtr(mUsdPath, ePTCct);
    if (cctPtr)
    {
        PxCapsuleController* cct = (PxCapsuleController*)cctPtr;
        return cct->getHeight();
    }
    else
        return 1.0f;
}

void CharacterController::setHeight(float height)
{
    const void* cctPtr = gPhysXInterface->getPhysXPtr(mUsdPath, ePTCct);
    if (cctPtr)
    {
        PxCapsuleController* cct = (PxCapsuleController*)cctPtr;
        cct->setHeight(height);
    }
}

void CharacterController::cacheMoveTarget()
{
    PhysxSchemaPhysxCharacterControllerAPI cctAPI = PhysxSchemaPhysxCharacterControllerAPI::Get(gStage, mUsdPath);
    if (cctAPI)
    {
        cctAPI.GetMoveTargetAttr().Get(&mInitMove);
    }
}


void CharacterController::resetMoveTarget()
{
    PhysxSchemaPhysxCharacterControllerAPI cctAPI = PhysxSchemaPhysxCharacterControllerAPI::Get(gStage, mUsdPath);
    if (cctAPI)
    {
        cctAPI.GetMoveTargetAttr().Set(mInitMove);
    }
}

void CharacterController::setMove(const pxr::GfVec3f& displ)
{
    PhysxSchemaPhysxCharacterControllerAPI cctAPI = PhysxSchemaPhysxCharacterControllerAPI(mUsdPrim);
    if (cctAPI)
    {
        UsdEditContext ctx(gStage, gStage->GetSessionLayer());
        cctAPI.GetMoveTargetAttr().Set(displ);
    }
}

void CharacterController::switchPurposeToGuide()
{
    static pxr::TfToken tGuide("guide");
    UsdGeomCapsule capsule = UsdGeomCapsule::Get(gStage, mUsdPath);
    if (capsule)
    {
        if (!capsule.GetPurposeAttr().Get(&mInitPurpose))
        {
            mInitPurpose = pxr::TfToken();
        }
        capsule.GetPurposeAttr().Set(tGuide);
    }
}

void CharacterController::resetPurpose()
{
    if (!mInitPurpose.IsEmpty())
    {
        UsdGeomCapsule capsule = UsdGeomCapsule::Get(gStage, mUsdPath);
        if (capsule)
        {
            capsule.GetPurposeAttr().Set(mInitPurpose);
        }
    }
}

void CharacterController::enableWorldSpaceMove(bool enable)
{
    mWorldSpaceMove = enable;
}

void CharacterController::onResume()
{
    mTimeSinceFalling = 0.f;
    mUsdPrim = gStage->GetPrimAtPath(mUsdPath);
}

void CharacterController::onStop()
{
    if (isFirstPerson())
    {
        resetPurpose();
    }
}

void CharacterController::onTimelinePlay()
{
    if (isFirstPerson())
    {
        switchPurposeToGuide();
    }
}

void CharacterController::setPosition(const ::physx::PxExtendedVec3& position)
{
    const void* cctPtr = gPhysXInterface->getPhysXPtr(mUsdPath, ePTCct);
    if (cctPtr)
    {
        PxCapsuleController& cct = *(PxCapsuleController*)cctPtr;
        cct.setPosition(position);
    }
}

void CharacterController::enableGravity()
{
    mGravityMode = GravityMode::APPLY_BASE;
}

void CharacterController::enableCustomGravity(PxVec3 gravity)
{
    mGravityMode = GravityMode::APPLY_CUSTOM;
    mCustomGravity = gravity;
}

void CharacterController::disableGravity()
{
    mGravityMode = GravityMode::DISABLED;
}

bool CharacterController::hasGravityEnabled() const
{
    return mGravityMode != GravityMode::DISABLED;
}

} // namespace physx
} // namespace omni
