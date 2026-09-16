#!/usr/bin/env python3
# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Generate the vehicle OVPhysX probe fixture.

N four-wheeled cars authored directly, one per grid cell, rather than one car cloned N times like
every other probe here: ovphysx_clone cannot replicate a vehicle. PhysXReplicator has no vehicle
case, so cloning duplicates the chassis actor and leaves InternalVehicle's wheel attachments behind.

CPU only: a vehicle's suspension and sticky-tire constraints are custom PxConstraints with a CPU
solver-prep function, which PhysX refuses on a scene carrying PxSceneFlag::eENABLE_DIRECT_GPU_API
(asserted by TestOvstageOutputReadVehicles.cpp). There cannot be a _gpu variant of this asset.

The scene body below is captured from VehicleFactory::create4WheeledCarsScenario, not transliterated.

Usage:
    python3 tests/benchmarks/data/gen_vehicle_probe.py [N] > \
        tests/benchmarks/data/vehicle_probe.usda
"""

import sys

VEHICLE_SPACING = 6.0  # metres between cars. The chassis is ~4 long, so this leaves them clear


PREAMBLE = r"""#usda 1.0
(
    defaultPrim = "World"
    kilogramsPerUnit = 1
    metersPerUnit = 1
    upAxis = "Y"
)

def "World"
{
    def SphereLight "SphereLight"
    {
        float inputs:intensity = 30000
        float inputs:radius = 1.5
        float3 xformOp:translate = (6.5, 11.5, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def PhysicsScene "PhysicsScene" (
        prepend apiSchemas = ["PhysxSceneAPI", "PhysxVehicleContextAPI"]
    )
    {
        vector3f physics:gravityDirection = (0, -1, 0)
        float physics:gravityMagnitude = 10
        uint physxScene:timeStepsPerSecond = 60
        uniform token physxVehicleContext:longitudinalAxis = "posZ"
        uniform token physxVehicleContext:updateMode = "velocityChange"
        uniform token physxVehicleContext:verticalAxis = "posY"
    }

    def PhysicsCollisionGroup "VehicleChassisCollisionGroup"
    {
        prepend rel physics:filteredGroups = </World/VehicleChassisCollisionGroup>
    }

    def PhysicsCollisionGroup "VehicleWheelCollisionGroup"
    {
        prepend rel physics:filteredGroups = [
            </World/VehicleGroundQueryGroup>,
            </World/GroundSurfaceCollisionGroup>,
        ]
    }

    def PhysicsCollisionGroup "VehicleGroundQueryGroup"
    {
        prepend rel physics:filteredGroups = [
            </World/VehicleChassisCollisionGroup>,
            </World/VehicleWheelCollisionGroup>,
        ]
    }

    def PhysicsCollisionGroup "GroundSurfaceCollisionGroup"
    {
        prepend rel collection:colliders:includes = </World/CollisionPlane>
        prepend rel physics:filteredGroups = [
            </World/GroundSurfaceCollisionGroup>,
            </World/VehicleWheelCollisionGroup>,
        ]
    }

    def Material "TarmacMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {
        float physics:dynamicFriction = 0.7
        float physics:restitution = 0
        float physics:staticFriction = 0.9
    }

    def Material "GravelMaterial" (
        prepend apiSchemas = ["PhysicsMaterialAPI", "PhysxMaterialAPI"]
    )
    {
        float physics:dynamicFriction = 0.6
        float physics:restitution = 0
        float physics:staticFriction = 0.6
    }

    def PhysxVehicleTireFrictionTable "WinterTireFrictionTable"
    {
        float[] frictionValues = [0.75, 0.6]
        prepend rel groundMaterials = [
            </World/TarmacMaterial>,
            </World/GravelMaterial>,
        ]
    }

    def PhysxVehicleTireFrictionTable "SummerTireFrictionTable"
    {
        float[] frictionValues = [0.7, 0.6]
        prepend rel groundMaterials = [
            </World/TarmacMaterial>,
            </World/GravelMaterial>,
        ]
    }

    def Mesh "GroundPlane"
    {
        int[] faceVertexCounts = [4]
        int[] faceVertexIndices = [0, 1, 2, 3]
        normal3f[] normals = [(0, 1, 0), (0, 1, 0), (0, 1, 0), (0, 1, 0)]
        point3f[] points = [(-15, 0, -15), (15, 0, -15), (15, 0, 15), (-15, 0, 15)]
        color3f[] primvars:displayColor = [(0.5, 0.5, 0.5)]
        quatf xformOp:orient = (1, 0, 0, 0)
        float3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]
    }

    def Plane "CollisionPlane" (
        prepend apiSchemas = ["PhysicsCollisionAPI", "MaterialBindingAPI"]
    )
    {
        uniform token axis = "Y"
        rel material:binding:physics = </World/TarmacMaterial> (
            bindMaterialAs = "weakerThanDescendants"
        )
        uniform token purpose = "guide"
    }

    def Scope "Wheel" (
        prepend apiSchemas = ["PhysxVehicleWheelAPI"]
    )
    {
        float physxVehicleWheel:dampingRate = 0.25
        float physxVehicleWheel:mass = 20
        float physxVehicleWheel:moi = 1.225
        float physxVehicleWheel:radius = 0.35
        float physxVehicleWheel:width = 0.15
    }

    def Scope "FrontTire" (
        prepend apiSchemas = ["PhysxVehicleTireAPI"]
    )
    {
        float physxVehicleTire:camberStiffness = 0
        prepend rel physxVehicleTire:frictionTable = </World/WinterTireFrictionTable>
        float2[] physxVehicleTire:frictionVsSlipGraph = [(0, 1), (0.1, 1), (1, 1)]
        float2 physxVehicleTire:lateralStiffnessGraph = (2, 76500)
        float physxVehicleTire:longitudinalStiffness = 5000
    }

    def Scope "RearTire" (
        prepend apiSchemas = ["PhysxVehicleTireAPI"]
    )
    {
        float physxVehicleTire:camberStiffness = 0
        prepend rel physxVehicleTire:frictionTable = </World/WinterTireFrictionTable>
        float2[] physxVehicleTire:frictionVsSlipGraph = [(0, 1), (0.1, 1), (1, 1)]
        float2 physxVehicleTire:lateralStiffnessGraph = (2, 76500)
        float physxVehicleTire:longitudinalStiffness = 5000
    }

    def Scope "FrontSuspension" (
        prepend apiSchemas = ["PhysxVehicleSuspensionAPI"]
    )
    {
        float physxVehicleSuspension:springDamperRate = 4500
        float physxVehicleSuspension:springStrength = 45000
        float physxVehicleSuspension:travelDistance = 0.2
    }

    def Scope "RearSuspension" (
        prepend apiSchemas = ["PhysxVehicleSuspensionAPI"]
    )
    {
        float physxVehicleSuspension:springDamperRate = 4500
        float physxVehicleSuspension:springStrength = 45000
        float physxVehicleSuspension:travelDistance = 0.2
    }

    def Scope "DriveBasic" (
        prepend apiSchemas = ["PhysxVehicleDriveBasicAPI"]
    )
    {
        float physxVehicleDriveBasic:peakTorque = 500
    }

"""

CAR = r"""    def Xform "@NAME@" (
        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", "PhysxRigidBodyAPI", "PhysxVehicleAPI", "PhysxVehicleControllerAPI", "PhysxVehicleBrakesAPI:brakes0", "PhysxVehicleBrakesAPI:brakes1", "PhysxVehicleSteeringAPI", "PhysxVehicleMultiWheelDifferentialAPI"]
        customData = {
            dictionary physxVehicle = {
                bool referenceFrameIsCenterOfMass = 0
            }
        }
    )
    {
        point3f physics:centerOfMass = (0, -0.25, 0)
        float3 physics:diagonalInertia = (3606.0005, 3942, 636)
        float physics:mass = 1800
        quatf physics:principalAxes = (1, 0, 0, 0)
        bool physxRigidBody:disableGravity = 1
        float physxRigidBody:sleepThreshold = 0
        float physxRigidBody:stabilizationThreshold = 0
        prepend rel physxVehicle:drive = </World/DriveBasic>
        int physxVehicle:highForwardSpeedSubStepCount = 1
        int physxVehicle:lowForwardSpeedSubStepCount = 3
        float physxVehicle:minActiveLongitudinalSlipDenominator = 0.1
        float physxVehicle:minLateralSlipDenominator = 1
        float physxVehicle:minPassiveLongitudinalSlipDenominator = 4
        float physxVehicle:subStepThresholdLongitudinalSpeed = 5
        uniform token physxVehicle:suspensionLineQueryType = "raycast"
        bool physxVehicle:vehicleEnabled = 1
        float physxVehicleBrakes:brakes0:maxBrakeTorque = 3600
        float physxVehicleBrakes:brakes1:maxBrakeTorque = 3000
        int[] physxVehicleBrakes:brakes1:wheels = [2, 3]
        float physxVehicleController:accelerator = 0
        float physxVehicleController:brake0 = 0
        float physxVehicleController:brake1 = 0
        float physxVehicleController:steer = 0
        int physxVehicleController:targetGear = 1
        float[] physxVehicleMultiWheelDifferential:torqueRatios = [0.5, 0.5]
        int[] physxVehicleMultiWheelDifferential:wheels = [0, 1]
        float physxVehicleSteering:maxSteerAngle = 0.554264
        int[] physxVehicleSteering:wheels = [0, 1]
        quatf xformOp:orient = (1, 0, 0, 0)
        float3 xformOp:translate = (@X@, 1, @Z@)
        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]

        def Xform "FrontLeftWheel" (
            prepend apiSchemas = ["PhysxVehicleWheelAttachmentAPI", "PhysxVehicleSuspensionComplianceAPI"]
        )
        {
            float4[] physxVehicleSuspensionCompliance:suspensionForceAppPoint = [(0, 0, -0.100000024, 0)]
            float4[] physxVehicleSuspensionCompliance:tireForceAppPoint = [(0, 0, -0.100000024, 0)]
            float2[] physxVehicleSuspensionCompliance:wheelCamberAngle = []
            float2[] physxVehicleSuspensionCompliance:wheelToeAngle = []
            prepend rel physxVehicleWheelAttachment:collisionGroup = </World/VehicleGroundQueryGroup>
            int physxVehicleWheelAttachment:index = 0
            prepend rel physxVehicleWheelAttachment:suspension = </World/FrontSuspension>
            quatf physxVehicleWheelAttachment:suspensionFrameOrientation = (1, 0, 0, 0)
            point3f physxVehicleWheelAttachment:suspensionFramePosition = (0.8, -0.54999995, 1.6)
            vector3f physxVehicleWheelAttachment:suspensionTravelDirection = (0, -1, 0)
            prepend rel physxVehicleWheelAttachment:tire = </World/FrontTire>
            prepend rel physxVehicleWheelAttachment:wheel = </World/Wheel>
            float3 xformOp:translate = (0.8, -0.65, 1.6)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "FrontRightWheel" (
            prepend apiSchemas = ["PhysxVehicleWheelAttachmentAPI", "PhysxVehicleSuspensionComplianceAPI"]
        )
        {
            float4[] physxVehicleSuspensionCompliance:suspensionForceAppPoint = [(0, 0, -0.100000024, 0)]
            float4[] physxVehicleSuspensionCompliance:tireForceAppPoint = [(0, 0, -0.100000024, 0)]
            float2[] physxVehicleSuspensionCompliance:wheelCamberAngle = []
            float2[] physxVehicleSuspensionCompliance:wheelToeAngle = []
            prepend rel physxVehicleWheelAttachment:collisionGroup = </World/VehicleGroundQueryGroup>
            int physxVehicleWheelAttachment:index = 1
            prepend rel physxVehicleWheelAttachment:suspension = </World/FrontSuspension>
            quatf physxVehicleWheelAttachment:suspensionFrameOrientation = (1, 0, 0, 0)
            point3f physxVehicleWheelAttachment:suspensionFramePosition = (-0.8, -0.54999995, 1.6)
            vector3f physxVehicleWheelAttachment:suspensionTravelDirection = (0, -1, 0)
            prepend rel physxVehicleWheelAttachment:tire = </World/FrontTire>
            prepend rel physxVehicleWheelAttachment:wheel = </World/Wheel>
            float3 xformOp:translate = (-0.8, -0.65, 1.6)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "RearLeftWheel" (
            prepend apiSchemas = ["PhysxVehicleWheelAttachmentAPI", "PhysxVehicleSuspensionComplianceAPI"]
        )
        {
            float4[] physxVehicleSuspensionCompliance:suspensionForceAppPoint = [(0, 0, -0.100000024, 0)]
            float4[] physxVehicleSuspensionCompliance:tireForceAppPoint = [(0, 0, -0.100000024, 0)]
            float2[] physxVehicleSuspensionCompliance:wheelCamberAngle = []
            float2[] physxVehicleSuspensionCompliance:wheelToeAngle = []
            prepend rel physxVehicleWheelAttachment:collisionGroup = </World/VehicleGroundQueryGroup>
            int physxVehicleWheelAttachment:index = 2
            prepend rel physxVehicleWheelAttachment:suspension = </World/RearSuspension>
            quatf physxVehicleWheelAttachment:suspensionFrameOrientation = (1, 0, 0, 0)
            point3f physxVehicleWheelAttachment:suspensionFramePosition = (0.8, -0.54999995, -1.6)
            vector3f physxVehicleWheelAttachment:suspensionTravelDirection = (0, -1, 0)
            prepend rel physxVehicleWheelAttachment:tire = </World/RearTire>
            prepend rel physxVehicleWheelAttachment:wheel = </World/Wheel>
            float3 xformOp:translate = (0.8, -0.65, -1.6)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }

        def Xform "RearRightWheel" (
            prepend apiSchemas = ["PhysxVehicleWheelAttachmentAPI", "PhysxVehicleSuspensionComplianceAPI"]
        )
        {
            float4[] physxVehicleSuspensionCompliance:suspensionForceAppPoint = [(0, 0, -0.100000024, 0)]
            float4[] physxVehicleSuspensionCompliance:tireForceAppPoint = [(0, 0, -0.100000024, 0)]
            float2[] physxVehicleSuspensionCompliance:wheelCamberAngle = []
            float2[] physxVehicleSuspensionCompliance:wheelToeAngle = []
            prepend rel physxVehicleWheelAttachment:collisionGroup = </World/VehicleGroundQueryGroup>
            int physxVehicleWheelAttachment:index = 3
            prepend rel physxVehicleWheelAttachment:suspension = </World/RearSuspension>
            quatf physxVehicleWheelAttachment:suspensionFrameOrientation = (1, 0, 0, 0)
            point3f physxVehicleWheelAttachment:suspensionFramePosition = (-0.8, -0.54999995, -1.6)
            vector3f physxVehicleWheelAttachment:suspensionTravelDirection = (0, -1, 0)
            prepend rel physxVehicleWheelAttachment:tire = </World/RearTire>
            prepend rel physxVehicleWheelAttachment:wheel = </World/Wheel>
            float3 xformOp:translate = (-0.8, -0.65, -1.6)
            uniform token[] xformOpOrder = ["xformOp:translate"]
        }
    }
"""


def main() -> int:
    # Default matches the committed asset, and the size readonly_vehicle_wheel_1024_cpu is named for,
    # so regenerating with no argument reproduces that file rather than silently shrinking it.
    count = int(sys.argv[1]) if len(sys.argv) > 1 else 1024
    side = int(count ** 0.5) + 1

    # One prototype plus N internal references to it, rather than ~120 spelled-out lines per car, so
    # the generated asset stays committable. The prototype is a `class`, hence abstract: it composes
    # into the prims that reference it and not into the scene, so the vehicle count stays N, not N + 1.
    out = [PREAMBLE]
    out.append(CAR.replace("@NAME@", "CarProto")
                  .replace("@X@", "0").replace("@Z@", "0")
                  .replace('def Xform "CarProto"', 'class Xform "CarProto"', 1))
    for i in range(count):
        x = (i % side) * VEHICLE_SPACING
        z = (i // side) * VEHICLE_SPACING
        out.append(
            '    def Xform "Car_%d" (\n'
            '        prepend references = </World/CarProto>\n'
            '    )\n'
            '    {\n'
            '        float3 xformOp:translate = (%g, 1, %g)\n'
            '        uniform token[] xformOpOrder = ["xformOp:translate"]\n'
            '    }\n\n' % (i, x, z)
        )
    out.append("}\n")  # close /World
    sys.stdout.write("".join(out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
