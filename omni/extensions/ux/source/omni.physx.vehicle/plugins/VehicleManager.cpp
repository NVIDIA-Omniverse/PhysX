// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <math.h>

#include "VehicleManager.h"

#include "VehicleController.h"

#include <carb/logging/Log.h>
#include <carb/profiler/Profile.h>

#include <omni/usd/UsdContextIncludes.h>
#include <omni/usd/UsdContext.h>

#include <omni/physx/IPhysx.h>
#include <omni/physx/IPhysxVehicle.h>

#include <common/foundation/Allocator.h>

extern pxr::UsdStageRefPtr gStage;

extern omni::physx::IPhysx* gPhysXInterface;


using namespace pxr;
using namespace carb;
using namespace ::physx;


namespace omni
{
namespace physx
{

VehicleManager::VehicleManager()
{
    mSimulationStarted = false;
    mSetupInputs = true;

    mUsdChangeListenerCount = 0;
}

VehicleManager::~VehicleManager()
{
    release();
}

void VehicleManager::release()
{
    releaseControllers();
}

void VehicleManager::releaseVehicleSettings()
{
    mSetupInputs = true;

    // Delete Vehicle Controller Settings.
    VehicleControllerSettingsMap::iterator vehicleSettingsIt = mVehicleControllerSettingsMap.begin();

    while (vehicleSettingsIt != mVehicleControllerSettingsMap.end())
    {
        ICE_PLACEMENT_DELETE(vehicleSettingsIt->second, VehicleControllerSettings);
        vehicleSettingsIt++;
    }

    mVehicleControllerSettingsMap.clear();
}

VehicleControllerSettings* VehicleManager::getVehicleSettings(const pxr::SdfPath& path)
{
    VehicleControllerSettingsMap::iterator iter = mVehicleControllerSettingsMap.find(path);

    if (iter != mVehicleControllerSettingsMap.end())
    {
        return iter->second;
    }
    else
    {
        VehicleControllerSettings* vehicleSettings = ICE_PLACEMENT_NEW(VehicleControllerSettings)();

        if (vehicleSettings)
        {
            mVehicleControllerSettingsMap.emplace(path, vehicleSettings);
            return vehicleSettings;
        }
        else
        {
            CARB_LOG_ERROR("PhysX Vehicle: VehicleControllerSettings could not be allocated "
                "for controller at prim \"%s\".\n",
                path.GetText());

            return nullptr;
        }
    }
}

bool VehicleManager::getInputEnabled(const pxr::SdfPath& path)
{
    return getVehicleSettings(path)->mInputEnabled;
}

void VehicleManager::setInputEnabled(const pxr::SdfPath& path, bool inputEnabled)
{
    // Disable inputs to all of the vehicles.
    VehicleControllerSettingsMap::iterator vehicleSettingsIt = mVehicleControllerSettingsMap.begin();

    while (vehicleSettingsIt != mVehicleControllerSettingsMap.end())
    {
        vehicleSettingsIt->second->mInputEnabled = false;
        vehicleSettingsIt++;
    }

    VehicleControllerMap::iterator vehicleIt = mVehicleControllerMap.begin();

    while (vehicleIt != mVehicleControllerMap.end())
    {
        VehicleController* vehicleController = vehicleIt->second;
        vehicleController->unbindInputs();
        vehicleIt++;
    }

    VehicleControllerSettings* vehicleSettings = getVehicleSettings(path);
    vehicleSettings->mInputEnabled = inputEnabled;

    VehicleController* vehicleController = getVehicle(path);

    if (vehicleController)
    {
        if (inputEnabled)
        {
            vehicleController->bindInputs();
        }
        else
        {
            vehicleController->disableFiltering();
        }
    }

    // Once the inputs are manually set, do not override them.
    mSetupInputs = false;
}

bool VehicleManager::getMouseEnabled(const pxr::SdfPath& path)
{
    return getVehicleSettings(path)->mMouseEnabled;
}

void VehicleManager::setMouseEnabled(const pxr::SdfPath& path, bool mouseEnabled)
{
    getVehicleSettings(path)->mMouseEnabled = mouseEnabled;
}

bool VehicleManager::getAutoReverseEnabled(const pxr::SdfPath& path)
{
    return getVehicleSettings(path)->mAutoReverseEnabled;
}

void VehicleManager::setAutoReverseEnabled(const pxr::SdfPath& path, bool autoReverseEnabled)
{
    getVehicleSettings(path)->mAutoReverseEnabled = autoReverseEnabled;
}

float VehicleManager::getSteeringSensitivity(const pxr::SdfPath& path)
{
    return getVehicleSettings(path)->mSteeringSensitivity;
}

void VehicleManager::setSteeringSensitivity(const pxr::SdfPath& path, float steeringSensitivity)
{
    getVehicleSettings(path)->mSteeringSensitivity = steeringSensitivity;
}

float VehicleManager::getSteeringFilterTime(const pxr::SdfPath& path)
{
    return getVehicleSettings(path)->mSteeringFilterTime;
}

void VehicleManager::setSteeringFilterTime(const pxr::SdfPath& path, float steeringFilterTime)
{
    getVehicleSettings(path)->mSteeringFilterTime = steeringFilterTime;

    VehicleControllerMap::iterator it = mVehicleControllerMap.find(path);

    if (it != mVehicleControllerMap.end())
    {
        it->second->onSteeringFilterTimeChanged();
    }
}

VehicleController* VehicleManager::getVehicle(const pxr::SdfPath& path)
{
    VehicleControllerMap::iterator it = mVehicleControllerMap.find(path);

    if (it != mVehicleControllerMap.end())
    {
        return it->second;
    }

    return nullptr;
}

void VehicleManager::vehicleParamDirty(const pxr::SdfPath& vehiclePath, const pxr::TfToken& token,
    VehicleController& vehicleController, const pxr::UsdTimeCode& timeCode)
{
    if (token == PhysxSchemaTokens->physxVehicleVehicleEnabled)
    {
        const pxr::UsdPrim& vehPrim = gStage->GetPrimAtPath(vehiclePath);
        CARB_ASSERT(vehPrim.HasAPI<PhysxSchemaPhysxVehicleAPI>());
        PhysxSchemaPhysxVehicleAPI vehicleAPI(vehPrim);
        bool enabled;
        vehicleAPI.GetVehicleEnabledAttr().Get(&enabled, timeCode);

        const bool wasEnabled = vehicleController.getEnabled();
        if (enabled != wasEnabled)
        {
            if (enabled)
            {
                CARB_ASSERT(mEnabledVehicleControllerSet.find(&vehicleController) == mEnabledVehicleControllerSet.end());
                mEnabledVehicleControllerSet.insert(&vehicleController);
            }
            else
            {
                CARB_ASSERT(mEnabledVehicleControllerSet.find(&vehicleController) != mEnabledVehicleControllerSet.end());
                mEnabledVehicleControllerSet.erase(&vehicleController);
            }

            vehicleController.setEnabled(enabled);
        }
    }
}

void VehicleManager::onResume()
{
    if (!mSimulationStarted)
    {
        createControllers();
        mSimulationStarted = true;
    }
}

void VehicleManager::onStop()
{
    release();
    mSimulationStarted = false;
}

void VehicleManager::createControllers()
{
    if (gStage && gPhysXInterface)
    {
        // Find all of the vehicles with a controller API, add a controller for them if one does not already exist.
        pxr::UsdPrimRange range = gStage->Traverse();

        for (pxr::UsdPrimRange::const_iterator iter = range.begin(); iter != range.end(); ++iter)
        {
            const pxr::UsdPrim& prim = *iter;
            if (!prim)
                continue;

            if (prim.HasAPI<PhysxSchemaPhysxVehicleControllerAPI>())
            {
                addController(prim, usdparser::kInvalidObjectId);
            }
        }
    }
}

void VehicleManager::releaseControllers()
{
    // Delete Vehicle Controllers.
    VehicleControllerMap::iterator vehicleIt = mVehicleControllerMap.begin();

    while (vehicleIt != mVehicleControllerMap.end())
    {
        VehicleController::release(*vehicleIt->second);
        vehicleIt++;

        CARB_ASSERT(mUsdChangeListenerCount > 0);
        mUsdChangeListenerCount--;
    }

    CARB_ASSERT(mUsdChangeListenerCount == 0);
    // to ensure the count is adjusted properly when starting to allow creation/deletion
    // while playing
    mUsdChangeListenerCount = 0;  // don't want dangling counts to carry over to next simulation session

    mVehicleControllerMap.clear();
    mEnabledVehicleControllerSet.clear();
}

void VehicleManager::addController(const pxr::UsdPrim& vehiclePrim, usdparser::ObjectId objectId)
{
    if (!vehiclePrim.HasAPI<PhysxSchemaPhysxVehicleTankControllerAPI>())
    {
        const pxr::SdfPath& vehiclePath = vehiclePrim.GetPath();

        // Create a new vehicle controller if one has not yet been created.
        VehicleControllerSettings* settings = getVehicleSettings(vehiclePath);
        if (settings)
        {
            // Enable inputs for the first vehicle if it has not been set manually.
            if (mSetupInputs && mVehicleControllerMap.size() == 0)
            {
                settings->mInputEnabled = true;
            }

            VehicleController* vehicleController = VehicleController::create(vehiclePrim, *settings, *this, objectId);

            if (vehicleController)
            {
                mVehicleControllerMap[vehiclePath] = vehicleController;

                if (vehicleController->getEnabled())
                {
                    CARB_ASSERT(mEnabledVehicleControllerSet.find(vehicleController) == mEnabledVehicleControllerSet.end());
                    mEnabledVehicleControllerSet.insert(vehicleController);
                }

                mUsdChangeListenerCount++;
            }
        }
    }
}

void VehicleManager::removeController(const pxr::SdfPath& vehiclePath)
{
    VehicleControllerMap::iterator vehicleIter = mVehicleControllerMap.find(vehiclePath);

    if (vehicleIter != mVehicleControllerMap.end())
    {
        VehicleController* vehicleController = vehicleIter->second;

        if (vehicleController->getEnabled())
        {
            VehicleControllerSet::iterator vehicleSetIter = mEnabledVehicleControllerSet.find(vehicleController);
            CARB_ASSERT(vehicleSetIter != mEnabledVehicleControllerSet.end());
            mEnabledVehicleControllerSet.erase(vehicleSetIter);
        }

        VehicleController::release(*vehicleController);

        mVehicleControllerMap.erase(vehicleIter);
        mUsdChangeListenerCount--;
    }
}

void VehicleManager::update(float timeStep, const pxr::UsdTimeCode& timeCode)
{
    if (!gStage)
    {
        gStage = omni::usd::UsdContext::getContext()->getStage();
    }

    {
        CARB_PROFILE_ZONE(0, "PhysXVehicleControllerUpdate");

        VehicleControllerSet::iterator it;

        for (it = mEnabledVehicleControllerSet.begin(); it != mEnabledVehicleControllerSet.end(); it++)
        {
            // Update the vehicle inputs.
            (*it)->update(timeStep, timeCode);
        }
    }
}

void VehicleManager::onUsdObjectChange(const pxr::SdfPath& path, const pxr::UsdTimeCode& timeCode)
{
    CARB_ASSERT(mUsdChangeListenerCount);  // else it should not be called

    if (path.IsPropertyPath())
    {
        TfToken const& attrName = path.GetNameToken();

        if (!UsdGeomXformable::IsTransformationAffectedByAttrNamed(attrName) &&
            attrName != pxr::UsdPhysicsTokens.Get()->physicsVelocity &&
            attrName != pxr::UsdPhysicsTokens.Get()->physicsAngularVelocity)
        {
            // note: using UsdGeomXformable::IsTransformationAffectedByAttrNamed() is an optimization
            //       to filter out notification for transformation changes early.

            // note: fetching the prim from the stage is not that cheap. Querying the prim
            //       by schema API type (prim.HasAPI()) is really expensive. prim.IsA() is
            //       much cheaper than HasAPI() but not super cheap either. Thus, rather
            //       use the path map we have already.

            CARB_ASSERT(gStage->GetPseudoRoot().GetPath() != path);
            // since the path describes a property (see path.IsPropertyPath() check), there is no
            // need to check for root path and path.GetPrimPath() can be used directly as checking
            // for the root path is not free. Assert to make sure this is as assumed.

            const SdfPath primPath = path.GetPrimPath();

            VehicleControllerMap::iterator vehIter = mVehicleControllerMap.find(primPath);

            if (vehIter != mVehicleControllerMap.end())
            {
                VehicleController* vehController = vehIter->second;
                vehicleParamDirty(primPath, attrName, *vehController, timeCode);
            }
        }
    }
}

} // namespace physx
} // namespace omni
