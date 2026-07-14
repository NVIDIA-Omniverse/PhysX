// SPDX-FileCopyrightText: Copyright (c) 2018-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

typedef std::map<PXR_NS::SdfPath, VehicleController*> VehicleControllerMap;
typedef std::set<VehicleController*> VehicleControllerSet;
typedef std::map<PXR_NS::SdfPath, struct VehicleControllerSettings*> VehicleControllerSettingsMap;


class VehicleManager
{
public:
    VehicleManager();
    ~VehicleManager();

    bool getInputEnabled(const PXR_NS::SdfPath& path);
    void setInputEnabled(const PXR_NS::SdfPath& path, bool inputEnabled);

    bool getMouseEnabled(const PXR_NS::SdfPath& path);
    void setMouseEnabled(const PXR_NS::SdfPath& path, bool mouseEnabled);
    bool getAutoReverseEnabled(const PXR_NS::SdfPath& path);
    void setAutoReverseEnabled(const PXR_NS::SdfPath& path, bool autoReverseEnabled);

    float getSteeringSensitivity(const PXR_NS::SdfPath& path);
    void setSteeringSensitivity(const PXR_NS::SdfPath& path, float steeringSensitivity);
    float getSteeringFilterTime(const PXR_NS::SdfPath& path);
    void setSteeringFilterTime(const PXR_NS::SdfPath& path, float steeringFilterTime);

    void releaseControllers();
    void addController(const PXR_NS::UsdPrim& vehiclePrim, usdparser::ObjectId);
    void removeController(const PXR_NS::SdfPath& vehiclePath);

    bool hasSimulationStarted() const
    {
        return mSimulationStarted;
    }
    bool hasUsdChangeListeners() const
    {
        return (mUsdChangeListenerCount > 0);
    }

    void update(float timeStep, const PXR_NS::UsdTimeCode&);

    void onResume();
    void onStop();

    void releaseVehicleSettings();

    void onUsdObjectChange(const PXR_NS::SdfPath& path, const PXR_NS::UsdTimeCode& timeCode);

    VehicleControllerSettings* getVehicleSettings(const PXR_NS::SdfPath& path);

    VehicleController* getVehicle(const PXR_NS::SdfPath& path);

private:
    void release();
    void createControllers();

    void vehicleParamDirty(const PXR_NS::SdfPath&, const PXR_NS::TfToken&, VehicleController&, const PXR_NS::UsdTimeCode&);


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
