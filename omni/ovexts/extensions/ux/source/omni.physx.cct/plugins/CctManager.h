// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

typedef std::map<PXR_NS::SdfPath, CharacterController*> CharacterControllerMap;

class CctManager
{
public:
    CctManager();
    ~CctManager();

    void update(float timeStep);

    void setPosition(const PXR_NS::SdfPath& path, const carb::Double3& pos);

    void enableGravity(const PXR_NS::SdfPath& path);

    void enableCustomGravity(const PXR_NS::SdfPath& path, const carb::Double3& gravity);

    void disableGravity(const PXR_NS::SdfPath& path);

    bool hasGravityEnabled(const PXR_NS::SdfPath& path);

    float getControllerHeight(const PXR_NS::SdfPath& path);

    void setControllerHeight(const PXR_NS::SdfPath& path, float height);

    void enableFirstPerson(const PXR_NS::SdfPath& path, const PXR_NS::SdfPath& cameraPath);

    void disableFirstPerson(const PXR_NS::SdfPath& path);

    void controllerDirty(const PXR_NS::SdfPath& path);

    void onTimelinePlay();

    void onResume();

    void onPause();

    void onStop();

    void release();

    void activateCct(const PXR_NS::SdfPath& path);

    void removeCct(const PXR_NS::SdfPath& path);

    void useHiddenCursor(bool enable);

    void enableWorldSpaceMove(const PXR_NS::SdfPath& path, bool enable);

    void setMove(const PXR_NS::SdfPath& path, const carb::Float3& displacement);

private:
    CharacterController* getCct(const PXR_NS::SdfPath& path);
    void updateUpAxisIndex(const PXR_NS::TfToken& upAxis);
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
