// SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause
//

#include "UsdPCH.h"

#include <carb/Framework.h>

#include <omni/physx/IPhysxVehicle.h>
#include <omni/physx/IPhysx.h>
#include <private/omni/physx/PhysxUsd.h>

// include auto-generated header
#include <OgnVehicleGetWheelStateDatabase.h>

class OgnVehicleGetWheelState
{
public:

    static bool compute(OgnVehicleGetWheelStateDatabase& db)
    {
        omni::physx::IPhysx* physXInterface = carb::getCachedInterface<omni::physx::IPhysx>();
        if (!physXInterface)
        {
            db.logError("Could not retrieve omni::physx::IPhysxVehicle interface");
            return false;
        }

        //Length of node input and output arrays.
        const size_t nb = db.inputs.vehicleWheelAttachmentPaths().size();

        if (db.outputs.wheelRotationSpeeds().size() < nb)
        {
            db.outputs.wheelRotationSpeeds().resize(nb);
            db.outputs.wheelRotationAngles().resize(nb);
            db.outputs.wheelSteerAngles().resize(nb);
            db.outputs.groundContactStates().resize(nb);
            db.outputs.groundPlanes().resize(nb);
            db.outputs.groundHitPositions().resize(nb);
            db.outputs.groundPhysXActors().resize(nb);
            db.outputs.groundPhysXShapes().resize(nb);
            db.outputs.groundPhysXMaterials().resize(nb);
            db.outputs.suspensionJounces().resize(nb);
            db.outputs.suspensionForces().resize(nb);
            db.outputs.tireFrictions().resize(nb);
            db.outputs.tireLongitudinalSlips().resize(nb);
            db.outputs.tireLateralSlips().resize(nb);
            db.outputs.tireLongitudinalDirections().resize(nb);
            db.outputs.tireLateralDirections().resize(nb);
            db.outputs.tireForces().resize(nb);
        }

        //Node input array.
        const omni::graph::core::ogn::const_array<NameToken>& vehicleWheelAttachmentPathTokens = db.inputs.vehicleWheelAttachmentPaths();

        //Note output arrays.
        omni::graph::core::ogn::array<float>& wheelRotationSpeeds = db.outputs.wheelRotationSpeeds();
        omni::graph::core::ogn::array<float>& wheelRotationAngles = db.outputs.wheelRotationAngles();
        omni::graph::core::ogn::array<float>& wheelSteerAngles = db.outputs.wheelSteerAngles();

        omni::graph::core::ogn::array<bool>& groundContactStates = db.outputs.groundContactStates();
        omni::graph::core::ogn::array<pxr::GfVec4f>& groundPlanes = db.outputs.groundPlanes();
        omni::graph::core::ogn::array<pxr::GfVec3f>& groundPositions = db.outputs.groundHitPositions();
        omni::graph::core::ogn::array<NameToken>& groundActors = db.outputs.groundPhysXActors();
        omni::graph::core::ogn::array<NameToken>& groundShapes = db.outputs.groundPhysXShapes();
        omni::graph::core::ogn::array<NameToken>& groundMaterials = db.outputs.groundPhysXMaterials();

        omni::graph::core::ogn::array<float>& suspensionJounces = db.outputs.suspensionJounces();
        omni::graph::core::ogn::array<pxr::GfVec3f>& suspensionForces = db.outputs.suspensionForces();

        omni::graph::core::ogn::array<float>& tireFrictions = db.outputs.tireFrictions();
        omni::graph::core::ogn::array<float>& tireLongitudinalSlips = db.outputs.tireLongitudinalSlips();
        omni::graph::core::ogn::array<float>& tireLateralSlips = db.outputs.tireLateralSlips();
        omni::graph::core::ogn::array<pxr::GfVec3f>& tireLongitudinalDirections = db.outputs.tireLongitudinalDirections();
        omni::graph::core::ogn::array<pxr::GfVec3f>& tireLateralDirections = db.outputs.tireLateralDirections();
        omni::graph::core::ogn::array<pxr::GfVec3f>& tireForces = db.outputs.tireForces();

        //Gather the object ids for all wheels.
        std::vector<omni::physx::usdparser::ObjectId> wheelObjectIdsA;
        omni::physx::usdparser::ObjectId wheelObjectIdsB[128];
        omni::physx::usdparser::ObjectId* wheelObjectIds = NULL;
        {
            if (nb > 128)
            {
                wheelObjectIdsA.resize(nb);
                wheelObjectIds = wheelObjectIdsA.data();
            }
            else
            {
                wheelObjectIds = wheelObjectIdsB;
            }

            for (uint32_t i = 0; i < uint32_t(nb); i++)
            {
                const char* wheelAttachmentPath = db.tokenToString(vehicleWheelAttachmentPathTokens[i]);
                if (!wheelAttachmentPath)
                {
                    db.logError("The wheel attachment prim path is not defined");
                    return false;
                }

                const omni::physx::usdparser::ObjectId wheelObjectId = physXInterface->getObjectId(pxr::SdfPath(wheelAttachmentPath), omni::physx::PhysXType::ePTVehicleWheelAttachment);
                if (omni::physx::usdparser::kInvalidObjectId == wheelObjectId)
                {
                    db.logError("Computation of wheel attachment id for prim at \"%s\" failed", wheelAttachmentPath);
                    return false;
                }

                wheelObjectIds[i] = wheelObjectId;
            }
        }

        //Get the state of all wheels.
        std::vector<omni::physx::VehicleWheelState> wheelStatesA;
        omni::physx::VehicleWheelState wheelStatesB[128];
        omni::physx::VehicleWheelState* wheelStates = NULL;
        {
            if (nb > 128)
            {
                wheelStatesA.resize(nb);
                wheelStates = wheelStatesA.data();
            }
            else
            {
                wheelStates = wheelStatesB;
            }

            if (!physXInterface->getWheelState(wheelObjectIds, uint32_t(nb), wheelStates))
            {
                db.logError("omni::physx::IPhysx::getWheelState() failed");
                return false;
            }
        }

        //Copy the wheel states to the output arrays of the node.
        {
            for (uint32_t i = 0; i < uint32_t(nb); i++)
            {
                wheelRotationSpeeds[i] = wheelStates[i].rotationSpeed;

                wheelRotationAngles[i] = wheelStates[i].rotationAngle;
                wheelSteerAngles[i] = wheelStates[i].steerAngle;

                groundContactStates[i] = wheelStates[i].isOnGround;
                if (wheelStates[i].isOnGround)
                {
                    groundPlanes[i].Set(wheelStates[i].groundPlane.x, wheelStates[i].groundPlane.y, wheelStates[i].groundPlane.z, wheelStates[i].groundPlane.w);
                    groundPositions[i].Set(wheelStates[i].groundHitPosition.x, wheelStates[i].groundHitPosition.y, wheelStates[i].groundHitPosition.z);
                    const char* gact = physXInterface->getPhysXObjectUsdPath(wheelStates[i].groundActor).GetText();
                    groundActors[i] = db.stringToToken(physXInterface->getPhysXObjectUsdPath(wheelStates[i].groundActor).GetText());
                    groundShapes[i] = db.stringToToken(physXInterface->getPhysXObjectUsdPath(wheelStates[i].groundShape).GetText());
                    groundMaterials[i] = db.stringToToken(physXInterface->getPhysXObjectUsdPath(wheelStates[i].groundMaterial).GetText());
                }
                else
                {
                    groundPlanes[i].Set(0, 0, 0, 0);
                    groundPositions[i].Set(0, 0, 0);
                    groundActors[i] = NameToken();
                    groundShapes[i] = NameToken();
                    groundMaterials[i] = NameToken();
                }

                suspensionJounces[i] = wheelStates[i].suspensionJounce;
                suspensionForces[i].Set(wheelStates[i].suspensionForce.x, wheelStates[i].suspensionForce.y, wheelStates[i].suspensionForce.z);

                tireFrictions[i] = wheelStates[i].tireFriction;
                tireLongitudinalSlips[i] = wheelStates[i].tireLongitudinalSlip;
                tireLateralSlips[i] = wheelStates[i].tireLateralSlip;
                tireLongitudinalDirections[i].Set(wheelStates[i].tireLongitudinalDirection.x, wheelStates[i].tireLongitudinalDirection.y, wheelStates[i].tireLongitudinalDirection.z);
                tireLateralDirections[i].Set(wheelStates[i].tireLateralDirection.x, wheelStates[i].tireLateralDirection.y, wheelStates[i].tireLateralDirection.z);
                tireForces[i].Set(wheelStates[i].tireForce.x, wheelStates[i].tireForce.y, wheelStates[i].tireForce.z);
            }
        }

        return true;
    }
};

REGISTER_OGN_NODE()
