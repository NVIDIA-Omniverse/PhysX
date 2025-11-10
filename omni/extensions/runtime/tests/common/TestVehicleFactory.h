// SPDX-FileCopyrightText: Copyright (c) 2019-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//
#pragma once

#include "UsdPCH.h"

#include <carb/Types.h>

struct UnitScale {
    float lengthScale;
    float massScale;
};

class VehicleFactory
{
public:

    struct CollGroupId
    {
        enum Enum
        {
            eVEHICLE_CHASSIS,
            eVEHICLE_WHEEL,
            eVEHICLE_GROUND_QUERY,
            eGROUND_SURFACE,
            eCOUNT
        };
    };

    struct TireFrictionTableId
    {
        enum Enum
        {
            eWINTER_TIRE,
            eSUMMER_TIRE,
            eCOUNT
        };
    };

    struct MaterialId
    {
        enum Enum
        {
            eTARMAC,
            eGRAVEL,
            eCOUNT
        };
    };

    struct TireId
    {
        enum Enum
        {
            eFRONT,
            eREAR,
            eCOUNT
        };
    };

    struct SuspensionId
    {
        enum Enum
        {
            eFRONT,
            eREAR,
            eCOUNT
        };
    };

    struct WheelAttId
    {
        enum Enum
        {
            eFL,  // front left
            eFR,  // front right
            eRL,  // rear left
            eRR,  // rear right
            eCOUNT
        };
    };

    struct DriveMode
    {
        enum Enum
        {
            eNONE = 0,
            eBASIC = 1,
            eSTANDARD = 2
        };
    };

    struct AxesIndices
    {
        // x-axis = 0
        // y-axis = 1
        // z-axis = 2

        AxesIndices()
            : up(1)
            , forward(2)
            , side(0)
        {
        };

        AxesIndices(uint32_t upAxisIdx, uint32_t forwardAxisIdx, uint32_t sideAxisIdx)
            : up(upAxisIdx)
            , forward(forwardAxisIdx)
            , side(sideAxisIdx)
        {
        };


        uint32_t up;
        uint32_t forward;
        uint32_t side;
    };

    struct Car4WheelsParams
    {
        Car4WheelsParams()
            : wheelPaths(nullptr)
            , tirePaths(nullptr)
            , suspensionPaths(nullptr)
            , drivePath(nullptr)
            , wheelAttachmentPathsOut(nullptr)
            , enabled(true)
            , useRaycasts(true)
            , createCollisionShapesForWheels(false)
            , useMeshAsWheelCollisionShape(false)
            , createAutoGearBox(true)
            , omitControllers(false)
            , addChassisCollisionBox(false)
            , addChassisRenderMesh(false)
        {
        }

        AxesIndices axes;
        const pxr::SdfPath(*wheelPaths)[4];
        const pxr::SdfPath(*tirePaths)[4];
        const pxr::SdfPath(*suspensionPaths)[4];
        const pxr::SdfPath* drivePath;
        pxr::SdfPath(*wheelAttachmentPathsOut)[4];
        bool enabled;
        bool useRaycasts;
        bool createCollisionShapesForWheels;
        bool useMeshAsWheelCollisionShape;
        bool createAutoGearBox;
        bool omitControllers;
        bool addChassisCollisionBox;
        bool addChassisRenderMesh;
    };

    struct Car4WheelsScenarioParams
    {
        Car4WheelsScenarioParams()
            : driveMode(DriveMode::eNONE)
            , basePositionForwardDir(0.0f)
            , basePositionSideDir(0.0f)
            , vehiclePathsOut(nullptr)
            , wheelAttachmentPathsOut(nullptr)
            , tireFrictionTablePathsOut(nullptr)
            , collisionGroupPathsOut(nullptr)
            , vehicleEnabledList(nullptr)
            , useRaycastsList(nullptr)
            , useShareableComponentsList(nullptr)
            , timeStepsPerSecond(60)
            , createCollisionShapesForWheels(false)
            , useMeshAsWheelCollisionShape(false)
            , createAutoGearBox(true)
            , omitControllers(false)
            , addChassisCollisionBox(false)
            , addChassisRenderMesh(false)
        {
            vehicleDelta.x = -3.0f;
            vehicleDelta.y = 0.0f;
            vehicleDelta.z = 0.0f;
        }

        DriveMode::Enum driveMode;
        AxesIndices axes;
        carb::Float3 vehicleDelta;
        float basePositionForwardDir;
        float basePositionSideDir;
        pxr::SdfPath* vehiclePathsOut;
        pxr::SdfPath(*wheelAttachmentPathsOut)[WheelAttId::eCOUNT];
        pxr::SdfPath(*tireFrictionTablePathsOut)[TireFrictionTableId::eCOUNT];
        pxr::SdfPath(*collisionGroupPathsOut)[CollGroupId::eCOUNT];
        const bool* vehicleEnabledList;
        const bool* useRaycastsList;
        const bool* useShareableComponentsList;
        uint32_t timeStepsPerSecond;
        bool createCollisionShapesForWheels;
        bool useMeshAsWheelCollisionShape;
        bool createAutoGearBox;
        bool omitControllers;
        bool addChassisCollisionBox;
        bool addChassisRenderMesh;
    };


    static void createCollisionGroups(
        const pxr::UsdStageRefPtr&,
        pxr::SdfPath(*collisionGroupPathsOut)[CollGroupId::eCOUNT] = nullptr
    );

    static void createMaterialsAndTireFrictionTables(
        const pxr::UsdStageRefPtr&,
        pxr::SdfPath(*materialPathsOut)[MaterialId::eCOUNT] = nullptr,
        pxr::SdfPath(*tireFrictionTablePathsOut)[TireFrictionTableId::eCOUNT] = nullptr
    );

    static void createSceneBasics(
        const pxr::UsdStageRefPtr&,
        const UnitScale unitScale,
        const AxesIndices& axes = AxesIndices(),
        const uint32_t timeStepsPerSecond = 60
    );

    static void createGroundPlane(
        const pxr::UsdStageRefPtr&,
        const UnitScale unitScale,
        const pxr::SdfPath& planeCollisionGroupPath,
        const pxr::SdfPath& planeMaterialPath,
        const AxesIndices& axes = AxesIndices()
    );

    //
    // note:
    // - tireFrictionTablePath can be set to nullptr if tirePaths is not None
    // - chassisCollisionGroupPath can be set to nullptr if addChassisCollisionBox is false
    // - wheelCollisionGroupPath can be set to nullptr if createCollisionShapesForWheels is false
    // - driveMode and createAutoGearBox do not matter if drivePath is set with the exception
    //   that they will still impact whether target gear is set to first or automatic mode
    // - useMeshAsWheelCollisionShape has no effect if createCollisionShapesForWheels is false
    //
    static void create4WheeledCar(
        const pxr::UsdStageRefPtr&,
        const UnitScale unitScale,
        const DriveMode::Enum,
        const pxr::SdfPath& vehiclePath,
        const pxr::GfVec3f& vehiclePosition,
        const bool(&wheelIsDriven)[4],
        const pxr::SdfPath& groundQueryCollisionGroupPath,
        const pxr::SdfPath* tireFrictionTablePath = nullptr,
        const pxr::SdfPath* chassisCollisionGroupPath = nullptr,
        const pxr::SdfPath* wheelCollisionGroupPath = nullptr,
        const Car4WheelsParams& params = Car4WheelsParams()
    );

    static void create4WheeledCarsScenario(
        const pxr::UsdStageRefPtr&,
        const UnitScale unitScale,
        const uint32_t vehicleCount,
        const Car4WheelsScenarioParams&
    );

};
