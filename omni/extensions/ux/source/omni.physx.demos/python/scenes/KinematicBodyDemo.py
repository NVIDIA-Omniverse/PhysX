# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, PhysxSchema
import omni.physxdemos as demo
import omni


class KinematicBodyDemo(demo.Base):
    title = "Kinematic Body"
    category = demo.Categories.RIGID_BODIES
    short_description = "Basic demo showing a kinematic body feature"
    description = "Basic demo showing an animated kinematic rigid body that interacts with dynamic rigid bodies. Press play (space) to run the simulation."

    def set_contact_offset(self, stage, path, offsetAmount, restAmount):
        prim = stage.GetPrimAtPath(path)
        physxCollisionAPI = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        physxCollisionAPI.CreateContactOffsetAttr(offsetAmount)
        physxCollisionAPI.CreateRestOffsetAttr(restAmount)

    def create(self, stage, manual_animation=False):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        self.stage = stage

        self.scale = 0.5
        size = 25.0

        color = demo.get_primary_color()

        # Kinematic
        position0 = Gf.Vec3f(-200.0, 0.0, 100.0) * self.scale
        positionEnd0 = Gf.Vec3f(100.0, 0.0, 100.0) * self.scale
        boxActorPath = defaultPrimPath + "/kinematicActor0"
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        translateOp0 = cubeGeom.AddTranslateOp() 
        translateOp0.Set(position0)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0,8.0,8.0) * self.scale) 
        cubeGeom.CreateDisplayColorAttr().Set([color])
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateKinematicEnabledAttr().Set(True)


        position1 = Gf.Vec3f(100.0, 0.0, 100.0) * self.scale
        positionEnd1 = Gf.Vec3f(400.0, 0.0, 100.0) * self.scale
        boxActorPath = defaultPrimPath + "/kinematicActor1"
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        translateOp1 = cubeGeom.AddTranslateOp() 
        translateOp1.Set(position1)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0,8.0,8.0) * self.scale) 
        cubeGeom.CreateDisplayColorAttr().Set([color])
        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateKinematicEnabledAttr().Set(True)

        self._animationData = {
            "manualAnimation": manual_animation,
            "maxStep":100,
            "keyframes":[
                {"op":translateOp0, "times":[0,50,100], "values":[position0,positionEnd0,position0]},
                {"op":translateOp1, "times":[0,50,100], "values":[position1,positionEnd1,position1]}
            ]
        }

        demo.animate_attributes(self, stage)

        # dynamic actor
        size = 50.0
        boxActorPath = defaultPrimPath + "/boxActor"
        position = Gf.Vec3f(0.0, 0.0, 50.0)
        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)

        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        UsdPhysics.RigidBodyAPI.Apply(cubePrim)

        room = demo.get_demo_room(self, stage, zoom = 0.5)
        room.set_physics_material(boxActorPath, room._plasticMaterialPath, True)
        
        for i in range(2):
            path = defaultPrimPath + "/kinematicActor" + str(i)
            room.set_glass_material(path)
            room.set_physics_material(path, room._glassAudioMaterialPath, True)

        self.set_contact_offset(stage, defaultPrimPath + "/kinematicActor0", 3.0, 2.0)
        self.set_contact_offset(stage, defaultPrimPath + "/kinematicActor1", 3.0, 2.0)
