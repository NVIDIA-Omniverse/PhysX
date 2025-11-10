# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import UsdLux, UsdGeom, Gf, UsdPhysics, PhysxSchema, Sdf
import omni.physxdemos as demo
import omni.timeline
from omni.physx.scripts.physicsUtils import *
import omni.kit.commands
import omni.kit.app
import omni.usd


class ForceDemo(demo.Base):
    title = "Force"
    category = demo.Categories.RIGID_BODIES
    short_description = "Basic demo showing PhysX force API usage"
    description = "Basic demo showing PhysX force API usage. Press play (space) to run the simulation. Enabling flow flame requires the availability of the omni.flow extension."

    params = {
        "Enable_Flow_Flame": demo.CheckboxParam(False),
    }

    def create_pyramid(self, stage, pyramidSize, y_position):
        box_size = 12.0
        size = Gf.Vec3f(box_size)
        orientation = Gf.Quatf(1.0, 0.0, 0.0, 0.0)
        color = demo.get_primary_color()
        linVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        angularVelocity = Gf.Vec3f(0.0, 0.0, 0.0)
        density = 0.0001
        offset = 1
        
        pyramidPath = Sdf.Path(self._defaultPrimPath + "/pyramid")
        UsdGeom.Scope.Define(stage, pyramidPath)

        for i in range(pyramidSize):
            for j in range(pyramidSize - i):
                # Box
                boxActorPath = pyramidPath.AppendChild("boxActor" + str(i) + str(j))
                position = Gf.Vec3f((-box_size*pyramidSize/2) + i*box_size/2 + j * (box_size + offset), y_position, (box_size + offset) * (i + 0.5))
                add_rigid_box(stage, boxActorPath, size, position, orientation, color, density, linVelocity, angularVelocity)

                self._room.set_physics_material(boxActorPath, self._room._plasticMaterialPath, True)

    def create(self, stage, Enable_Flow_Flame, manual_animation=False):
        self._defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        self._room = demo.get_demo_room(self, stage, zoom = 0.5)
        self._flow_was_enabled = False

        # Shuttle body
        shuttleActorPath = self._defaultPrimPath + "/shuttleActor"
        xform = UsdGeom.Xform.Define(stage, shuttleActorPath)
        xformable = UsdGeom.Xformable(xform.GetPrim())
        xformable.AddTranslateOp().Set(Gf.Vec3d(0.0, 180.0, self._room._floorOffset))
        UsdPhysics.RigidBodyAPI.Apply(xform.GetPrim())

        # Shuttle main
        capsuleActorPath = shuttleActorPath + "/capsuleMain"
        rocketScale = 0.5
        position = Gf.Vec3f(0.0, 0.0, 60.0)*rocketScale
        color = demo.get_primary_color()

        capsuleGeom = UsdGeom.Capsule.Define(stage, capsuleActorPath)
        capsuleGeom.CreateRadiusAttr(20.0*rocketScale)
        capsuleGeom.CreateHeightAttr(50.0*rocketScale)
        capsuleGeom.AddTranslateOp().Set(position)
        capsuleGeom.CreateDisplayColorAttr().Set([color])
        UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())        
        
        # side bodies        
        positions = [ Gf.Vec3f(0.0, 30.0, 20.0)*rocketScale, Gf.Vec3f(0.0, -30.0, 20.0)*rocketScale, Gf.Vec3f(30.0, 0.0, 20.0)*rocketScale, Gf.Vec3f(-30.0, 0.0, 20.0)*rocketScale ]        

        i = 0
        for pos in positions:
            capsuleActorPath = shuttleActorPath + "/capsuleSide" + str(i)
            capsuleGeom = UsdGeom.Capsule.Define(stage, capsuleActorPath)
            capsuleGeom.CreateRadiusAttr(10.0*rocketScale)
            capsuleGeom.CreateHeightAttr(20.0*rocketScale)
            capsuleGeom.AddTranslateOp().Set(pos)
            capsuleGeom.CreateDisplayColorAttr().Set([color])

            UsdPhysics.CollisionAPI.Apply(capsuleGeom.GetPrim())
            i = i + 1
        
        self._room.set_physics_material(shuttleActorPath, self._room._plasticMaterialPath, True)

        # Shuttle rocket force
        shuttleForcePath = shuttleActorPath + "/shuttleForce"
        xform = UsdGeom.Xform.Define(stage, shuttleForcePath)
        forceApi = PhysxSchema.PhysxForceAPI.Apply(xform.GetPrim())        

        # program rocket force
        omni.timeline.get_timeline_interface().set_end_time(10000/24)
        
        forceAttr = forceApi.GetForceAttr()
        forceEnabledAttr = forceApi.GetForceEnabledAttr()
        xformable = UsdGeom.Xformable(xform.GetPrim())
        translateOp = xformable.AddTranslateOp()

        self._animationData = {
            "manualAnimation": manual_animation,
            "keyframes":[
                {"op":forceAttr, "times":[0,20,50], "values":[Gf.Vec3f(0.0, 0.0, 1700.0),Gf.Vec3f(0.0, 0.0, 1300.0),Gf.Vec3f(0.0, 0.0, 1400.0)]},
                {"op":forceEnabledAttr, "times":[0,50], "values":[True, False]},
                {"op":translateOp, "times":[0,20,25], "values":[Gf.Vec3d(0.0, 0.0, 0.0), Gf.Vec3d(0.0, 2.0, 0.0), Gf.Vec3d(0.0, 0.0, 0.0)]}
            ]
        }

        demo.animate_attributes(self, stage)
        
        # cube stack   
        self.create_pyramid(stage, 10, 0)                    
                
        # flow flame:
        if Enable_Flow_Flame:
            # make sure extension is on
            self._setup_flow()

            # create Fire preset
            omni.kit.commands.execute(
                "FlowCreatePresets",
                paths=[self._defaultPrimPath + "/shuttleActor"],
                preset_name="Fire",
                layer=1
            )
            
            # enable flow rendering
            omni.kit.commands.execute("ChangeSetting", path="rtx/flow/enabled", value=True)
            omni.kit.commands.execute("ChangeSetting", path="rtx/flow/rayTracedReflectionsEnabled", value=True)
            omni.kit.commands.execute("ChangeSetting", path="rtx/flow/rayTracedTranslucencyEnabled", value=True)
            omni.kit.commands.execute("ChangeSetting", path="rtx/flow/pathTracingEnabled", value=True)
        
        # clear selection
        prim_path_list = []
        omni.usd.get_context().get_selection().set_selected_prim_paths(prim_path_list, True)        

    def _setup_flow(self):
        self._flow_was_enabled = demo.utils.enable_extension_with_check("omni.flowusd")

    def on_shutdown(self):
        if not self._flow_was_enabled:
            demo.utils.disable_extension("omni.flowusd")
        super().on_shutdown()
