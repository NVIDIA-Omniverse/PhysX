// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#pragma once

#include "UsdPCH.h"

#include <carb/Defines.h>
#include <carb/Types.h>
#include <omni/physx/IPhysx.h>

#include <map>


namespace omni
{
namespace physx
{

class VehicleController;

typedef std::map<pxr::SdfPath, VehicleController*> VehicleControllerMap;
typedef std::set<VehicleController*> VehicleControllerSet;
typedef std::map<pxr::SdfPath, struct VehicleControllerSettings*> VehicleControllerSettingsMap;


class VehicleManager
{
public:
    VehicleManager();
    ~VehicleManager();

    bool getInputEnabled(const pxr::SdfPath& path);
    void setInputEnabled(const pxr::SdfPath& path, bool inputEnabled);

    bool getMouseEnabled(const pxr::SdfPath& path);
    void setMouseEnabled(const pxr::SdfPath& path, bool mouseEnabled);
    bool getAutoReverseEnabled(const pxr::SdfPath& path);
    void setAutoReverseEnabled(const pxr::SdfPath& path, bool autoReverseEnabled);

    float getSteeringSensitivity(const pxr::SdfPath& path);
    void setSteeringSensitivity(const pxr::SdfPath& path, float steeringSensitivity);
    float getSteeringFilterTime(const pxr::SdfPath& path);
    void setSteeringFilterTime(const pxr::SdfPath& path, float steeringFilterTime);

    void releaseControllers();
    void addController(const pxr::UsdPrim& vehiclePrim, usdparser::ObjectId);
    void removeController(const pxr::SdfPath& vehiclePath);

    bool hasSimulationStarted() const
    {
        return mSimulationStarted;
    }
    bool hasUsdChangeListeners() const
    {
        return (mUsdChangeListenerCount > 0);
    }

    void update(float timeStep, const pxr::UsdTimeCode&);

    void onResume();
    void onStop();

    void releaseVehicleSettings();

    void onUsdObjectChange(const pxr::SdfPath& path, const pxr::UsdTimeCode& timeCode);

    VehicleControllerSettings* getVehicleSettings(const pxr::SdfPath& path);

    VehicleController* getVehicle(const pxr::SdfPath& path);

private:
    void release();
    void createControllers();

    void vehicleParamDirty(const pxr::SdfPath&, const pxr::TfToken&, VehicleController&, const pxr::UsdTimeCode&);


private:
    bool mSimulationStarted;
    bool mSetupInputs;

    unsigned int mUsdChangeListenerCount;

    VehicleControllerMap mVehicleControllerMap;
    VehicleControllerSet mEnabledVehicleControllerSet;
    VehicleControllerSettingsMap mVehicleControllerSettingsMap;
};

} // namespace physx
} // namespace omni
