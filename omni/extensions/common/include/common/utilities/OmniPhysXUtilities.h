// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#include <omni/physx/IPhysxSettings.h>
#include <pxr/usd/usd/prim.h>

#include <carb/settings/ISettings.h>


namespace omni
{
namespace physx
{

inline bool isPhysXDefaultSimulator()
{
    carb::settings::ISettings* settings = carb::getCachedInterface<carb::settings::ISettings>();
    return strcmp(settings->getStringBuffer(kSettingDefaultSimulator), "PhysX") == 0 ? true : false;
}

// Checks if physics scene can be processed by PhysX
// Optionally pass a cached bool already containing the value of isPhysXDefaultSimulator
inline bool canSceneBeProcessedByPhysX(const pxr::UsdPrim& scenePrim, const bool* optDefaultSimulator)
{
    // We process only scenes without any API or with PhysxSceneAPI
    bool processScene = scenePrim.HasAPI<pxr::PhysxSchemaPhysxSceneAPI>();
    const std::string allowedList[] = { "PhysxResidualReportingAPI", "PhysxSceneQuasistaticAPI", "MaterialBindingAPI",
                                        "CollectionAPI", "VehicleContextAPI" };
    const pxr::TfTokenVector& apis = scenePrim.GetPrimTypeInfo().GetAppliedAPISchemas();
    const bool isDefaultSimulator = optDefaultSimulator ? *optDefaultSimulator : isPhysXDefaultSimulator();
    if (apis.empty() && isDefaultSimulator)
    {
        processScene = true;
    }
    else if (!processScene && !apis.empty())
    {
        bool allowedApis = true;
        for (const pxr::TfToken& token : apis)
        {
            const std::string tokenString = token.GetString();
            bool allowed = false;
            for (const std::string& allowedToken : allowedList)
            {
                if (tokenString.find(allowedToken) != std::string::npos)
                {
                    allowed = true;
                    break;
                }
            }
            if (!allowed)
            {
                allowedApis = false;
                break;
            }
        }
        if (allowedApis)
            processScene = true;
    }
    return processScene;
}
} // namespace physx
} // namespace omni
