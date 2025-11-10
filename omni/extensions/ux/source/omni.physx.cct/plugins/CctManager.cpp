// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include "CctManager.h"

#include "CharacterController.h"

#include <carb/logging/Log.h>
#include <carb/windowing/IWindowing.h>
#include <omni/kit/IAppWindow.h>
#include <carb/input/IInput.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

extern carb::input::IInput* gInput;
extern pxr::UsdStageRefPtr gStage;

using namespace pxr;
using namespace physx;

namespace omni
{
namespace physx
{

CctManager::CctManager()
{
    mUpAxisIndex = 1;
    mInitialMoveStored = false;
    mHiddenCursor = false;
    mDirty = false;
}

CctManager::~CctManager()
{
    release();
}

CharacterController* CctManager::getCct(const pxr::SdfPath& path)
{
    CharacterControllerMap::iterator it = mCharacterControllerMap.find(path);
    if (it != mCharacterControllerMap.end())
        return it->second;

    CharacterController* cct = new CharacterController(path);
    mCharacterControllerMap[path] = cct;

    return cct;
}

void CctManager::updateUpAxisIndex(const TfToken& upAxis)
{
    if (upAxis == TfToken("Y"))
    {
        mUpAxisIndex = 1;
    }
    else if (upAxis == TfToken("Z"))
    {
        mUpAxisIndex = 2;
    }
}

void CctManager::forceRefresh()
{
    onStop();
    onResume();
    onTimelinePlay();
    mDirty = false;
}

void CctManager::activateCct(const pxr::SdfPath& path)
{
    getCct(path);
    mDirty = true;
}

void CctManager::removeCct(const pxr::SdfPath& path)
{
    mCharacterControllerMap.erase(path);
    mDirty = true;
}

void CctManager::enableGravity(const pxr::SdfPath& path)
{
    getCct(path)->enableGravity();
}

void CctManager::enableCustomGravity(const pxr::SdfPath& path, const carb::Double3& gravity)
{
    getCct(path)->enableCustomGravity(PxVec3((float)gravity.x, (float)gravity.y, (float)gravity.z));
}

void CctManager::disableGravity(const pxr::SdfPath& path)
{
    getCct(path)->disableGravity();
}

bool CctManager::hasGravityEnabled(const pxr::SdfPath& path)
{
    return getCct(path)->hasGravityEnabled();
}

float CctManager::getControllerHeight(const pxr::SdfPath& path)
{
    return getCct(path)->getHeight();
}

void CctManager::setControllerHeight(const pxr::SdfPath& path, float height)
{
    getCct(path)->setHeight(height);
}

void CctManager::enableFirstPerson(const pxr::SdfPath& path, const pxr::SdfPath& cameraPath)
{
    getCct(path)->enableFirstPerson(cameraPath);
    mDirty = true;
}

void CctManager::disableFirstPerson(const pxr::SdfPath& path)
{
    getCct(path)->disableFirstPerson();
    mDirty = true;
}

void CctManager::controllerDirty(const pxr::SdfPath& path)
{
    getCct(path)->setDirty();
}

void CctManager::onTimelinePlay()
{
    if (!mInitialMoveStored)
    {
        for (CharacterControllerMap::value_type& kv : mCharacterControllerMap)
        {
            kv.second->cacheMoveTarget();
        }
        mInitialMoveStored = true;
    }

    for (CharacterControllerMap::value_type& kv : mCharacterControllerMap)
    {
        kv.second->onTimelinePlay();
    }
}

void CctManager::onResume()
{
    updateUpAxisIndex(UsdGeomGetStageUpAxis(gStage));
    
    for (CharacterControllerMap::value_type& kv : mCharacterControllerMap)
    {
        CharacterController& cct = *kv.second;
        if (cct.isFirstPerson())
        {
            useHiddenCursor(true);
            break;
        }
    }

    for (CharacterControllerMap::value_type& kv : mCharacterControllerMap)
    {
        kv.second->onResume();
    }

    mDirty = false;
}

void CctManager::onPause()
{
    useHiddenCursor(false);
}

void CctManager::onStop()
{
    useHiddenCursor(false);

    for (CharacterControllerMap::value_type& kv : mCharacterControllerMap)
    {
        CharacterController& cct = *kv.second;

        if (mInitialMoveStored)
        {
            cct.resetMoveTarget();
        }

        cct.onStop();
    }

    mInitialMoveStored = false;
}

void CctManager::update(float timeStep)
{
    if (!gStage)
    {
        gStage = omni::usd::UsdContext::getContext()->getStage();
        updateUpAxisIndex(UsdGeomGetStageUpAxis(gStage));
    }

    if (mDirty)
    {
        forceRefresh();
    }

    for (const CharacterControllerMap::value_type& kv : mCharacterControllerMap)
    {
        kv.second->update(timeStep, mUpAxisIndex);
    }
}

void CctManager::setPosition(const pxr::SdfPath& path, const carb::Double3& pos)
{
    getCct(path)->setPosition(PxExtendedVec3(pos.x, pos.y, pos.z));
}

void CctManager::release()
{
    CharacterControllerMap::iterator it = mCharacterControllerMap.begin();
    while (it != mCharacterControllerMap.end())
    {
        delete it->second;
        it++;
    }

    mCharacterControllerMap.clear();
}

void CctManager::useHiddenCursor(bool enable)
{
    carb::windowing::IWindowing* windowing = carb::getCachedInterface<carb::windowing::IWindowing>();
    omni::kit::IAppWindow* appWindow = omni::kit::getDefaultAppWindow();

    if (!windowing || !appWindow)
    {
        return;
    }

    auto* window = appWindow ? appWindow->getWindow() : nullptr;

    if (!window)
    {
        return;
    }

    if (enable)
    {
        windowing->setCursorMode(window, carb::windowing::CursorMode::eDisabled);
        mHiddenCursor = true;       
    }
    else if (mHiddenCursor && !enable)
    {
        windowing->setCursorMode(window, carb::windowing::CursorMode::eNormal);
        mHiddenCursor = false;
    }
}

void CctManager::enableWorldSpaceMove(const pxr::SdfPath& path, bool enable)
{
    getCct(path)->enableWorldSpaceMove(enable);
}

void CctManager::setMove(const pxr::SdfPath& path, const carb::Float3& displacement)
{
    getCct(path)->setMove(pxr::GfVec3f(displacement.x, displacement.y, displacement.z));
}

} // namespace physx
} // namespace omni
