// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <carb/input/InputTypes.h>

#include <map>

namespace omni
{
namespace physx
{
class CharacterController;

typedef std::map<pxr::SdfPath, CharacterController*> CharacterControllerMap;

class CctManager
{
public:
    CctManager();
    ~CctManager();

    void update(float timeStep);

    void setPosition(const pxr::SdfPath& path, const carb::Double3& pos);

    void enableGravity(const pxr::SdfPath& path);

    void enableCustomGravity(const pxr::SdfPath& path, const carb::Double3& gravity);

    void disableGravity(const pxr::SdfPath& path);

    bool hasGravityEnabled(const pxr::SdfPath& path);

    float getControllerHeight(const pxr::SdfPath& path);

    void setControllerHeight(const pxr::SdfPath& path, float height);

    void enableFirstPerson(const pxr::SdfPath& path, const pxr::SdfPath& cameraPath);

    void disableFirstPerson(const pxr::SdfPath& path);

    void controllerDirty(const pxr::SdfPath& path);

    void onTimelinePlay();

    void onResume();

    void onPause();

    void onStop();

    void release();

    void activateCct(const pxr::SdfPath& path);

    void removeCct(const pxr::SdfPath& path);

    void useHiddenCursor(bool enable);

    void enableWorldSpaceMove(const pxr::SdfPath& path, bool enable);

    void setMove(const pxr::SdfPath& path, const carb::Float3& displacement);

private:
    CharacterController* getCct(const pxr::SdfPath& path);
    void updateUpAxisIndex(const pxr::TfToken& upAxis);
    void setDirty();
    void forceRefresh();

private:
    unsigned char mUpAxisIndex;
    bool mHiddenCursor;
    CharacterControllerMap mCharacterControllerMap;
    bool mInitialMoveStored;
    bool mDirty;
};
} // namespace physx
} // namespace omni
