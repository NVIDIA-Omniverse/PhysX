# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
import omni.kit.commands
import omni.physxdemos as demo
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, UsdShade, PhysxSchema


class ParticleDemoBase:

    def on_shutdown(self):
        """
        Make sure to call this from your optional on_shutdown override with super().on_shutdown() for proper cleanup.
        """
        # disable/enable fabric ext if needed
        manager = omni.kit.app.get_app().get_extension_manager()
        manager.set_extension_enabled_immediate("omni.physx.fabric", self._fabric_was_enabled)

    def __init__(self, enable_fabric=False, fabric_compatible=True):
        """
        Use the constructor arguments to configure what additional extensions are loaded
        and made available to the demo. If fabric_compatible=False then enable_fabric=True is ignored.
        If fabric_compatible=True and enable_fabric=False, the fabric extension is not unloaded.

        Call from your derived class constructor, for example like this:
            def __init__(self):
                super().__init__(enable_fabric=True)
        """
        super().__init__()
        self._fabric_was_enabled = False

        if not fabric_compatible:
            enable_fabric = False

        self._enable_fabric(enable_fabric, fabric_compatible)

    # helpers:
    def setup_base_scene(self, stage, zoomAmount=0.25):
        defaultPrimPath, scene = demo.setup_physics_scene(self, stage, primPathAsString=False, upAxis=UsdGeom.Tokens.y)

        physxSceneAPI = PhysxSchema.PhysxSceneAPI.Apply(scene.GetPrim())
        physxSceneAPI.GetEnableExternalForcesEveryIterationAttr().Set(True)

        self._stage = stage
        self._scene = scene
        self._room = demo.get_demo_room(self, stage, zoom=zoomAmount, contactRestOffset=Gf.Vec2f(2.0, 1.0), enableCollisionAudio=False)
        return defaultPrimPath

    def create_pbd_material(self, mat_name: str, color_rgb: Gf.Vec3f = Gf.Vec3f(0.2, 0.2, 0.8)) -> Sdf.Path:
        # create material for extras
        create_list = []
        omni.kit.commands.execute(
            "CreateAndBindMdlMaterialFromLibrary",
            mdl_name="OmniPBR.mdl",
            mtl_name="OmniPBR",
            mtl_created_list=create_list,
            bind_selected_prims=False,
            select_new_prim=False,
        )
        target_path = "/World/Looks/" + mat_name
        if create_list[0] != target_path:
            omni.kit.commands.execute("MovePrims", paths_to_move={create_list[0]: target_path})
        shader = UsdShade.Shader.Get(self._stage, target_path + "/Shader")
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Color3f).Set(color_rgb)
        return Sdf.Path(target_path)

    def create_particle_box_collider(
        self,
        path: Sdf.Path,
        side_length: float = 100.0,
        height: float = 50.0,
        translate: Gf.Vec3f = Gf.Vec3f(0, 0, 0),
        thickness: float = 10.0,
        add_cylinder_top=True,
    ):
        """
        Creates an invisible collider box to catch particles. Opening is in y-up

        Args:
            path:           box path (xform with cube collider children that make up box)
            side_length:    inner side length of box
            height:         height of box
            translate:      location of box, w.r.t it's bottom center
            thickness:      thickness of the box walls
        """
        xform = UsdGeom.Xform.Define(self._stage, path)
        # xform.MakeInvisible()
        xform_path = xform.GetPath()
        physicsUtils.set_or_add_translate_op(xform, translate=translate)
        cube_width = side_length + 2.0 * thickness
        offset = side_length * 0.5 + thickness * 0.5
        # front and back (+/- x)
        cube = UsdGeom.Cube.Define(self._stage, xform_path.AppendChild("front"))
        cube.CreateSizeAttr().Set(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        physicsUtils.set_or_add_translate_op(cube, Gf.Vec3f(0, height * 0.5, offset))
        physicsUtils.set_or_add_scale_op(cube, Gf.Vec3f(cube_width, height, thickness))

        if add_cylinder_top:
            top_cylinder = UsdGeom.Cylinder.Define(self._stage, xform_path.AppendChild("front_top_cylinder"))
            top_cylinder.CreateRadiusAttr().Set(thickness * 0.5)
            top_cylinder.CreateHeightAttr().Set(cube_width)
            top_cylinder.CreateAxisAttr().Set("X")
            UsdPhysics.CollisionAPI.Apply(top_cylinder.GetPrim())
            physicsUtils.set_or_add_translate_op(top_cylinder, Gf.Vec3f(0, height, offset))

        cube = UsdGeom.Cube.Define(self._stage, xform_path.AppendChild("back"))
        cube.CreateSizeAttr().Set(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        physicsUtils.set_or_add_translate_op(cube, Gf.Vec3f(0, height * 0.5, -offset))
        physicsUtils.set_or_add_scale_op(cube, Gf.Vec3f(cube_width, height, thickness))

        if add_cylinder_top:
            top_cylinder = UsdGeom.Cylinder.Define(self._stage, xform_path.AppendChild("back_top_cylinder"))
            top_cylinder.CreateRadiusAttr().Set(thickness * 0.5)
            top_cylinder.CreateHeightAttr().Set(cube_width)
            top_cylinder.CreateAxisAttr().Set("X")
            UsdPhysics.CollisionAPI.Apply(top_cylinder.GetPrim())
            physicsUtils.set_or_add_translate_op(top_cylinder, Gf.Vec3f(0, height, -offset))

        # left and right:
        cube = UsdGeom.Cube.Define(self._stage, xform_path.AppendChild("left"))
        cube.CreateSizeAttr().Set(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        physicsUtils.set_or_add_translate_op(cube, Gf.Vec3f(-offset, height * 0.5, 0))
        physicsUtils.set_or_add_scale_op(cube, Gf.Vec3f(thickness, height, cube_width))

        if add_cylinder_top:
            top_cylinder = UsdGeom.Cylinder.Define(self._stage, xform_path.AppendChild("left_top_cylinder"))
            top_cylinder.CreateRadiusAttr().Set(thickness * 0.5)
            top_cylinder.CreateHeightAttr().Set(cube_width - thickness)
            top_cylinder.CreateAxisAttr().Set("Z")
            UsdPhysics.CollisionAPI.Apply(top_cylinder.GetPrim())
            physicsUtils.set_or_add_translate_op(top_cylinder, Gf.Vec3f(-offset, height, 0))

        cube = UsdGeom.Cube.Define(self._stage, xform_path.AppendChild("right"))
        cube.CreateSizeAttr().Set(1.0)
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        physicsUtils.set_or_add_translate_op(cube, Gf.Vec3f(offset, height * 0.5, 0))
        physicsUtils.set_or_add_scale_op(cube, Gf.Vec3f(thickness, height, cube_width))

        if add_cylinder_top:
            top_cylinder = UsdGeom.Cylinder.Define(self._stage, xform_path.AppendChild("right_top_cylinder"))
            top_cylinder.CreateRadiusAttr().Set(thickness * 0.5)
            top_cylinder.CreateHeightAttr().Set(cube_width - thickness)
            top_cylinder.CreateAxisAttr().Set("Z")
            UsdPhysics.CollisionAPI.Apply(top_cylinder.GetPrim())
            physicsUtils.set_or_add_translate_op(top_cylinder, Gf.Vec3f(offset, height, 0))

        xform_path_str = str(xform_path)

        paths = [
            xform_path_str + "/front",
            xform_path_str + "/back",
            xform_path_str + "/left",
            xform_path_str + "/right",
        ]
        for path in paths:
            self._room.set_glass_material(path)

    def _enable_fabric(self, enable_fabric, fabric_compatible):
        manager = omni.kit.app.get_app().get_extension_manager()
        self._fabric_was_enabled = manager.is_extension_enabled("omni.physx.fabric")
        if enable_fabric and not self._fabric_was_enabled:
            demo.utils.enable_extension("omni.physx.fabric")
        if not fabric_compatible and self._fabric_was_enabled:
            manager.set_extension_enabled_immediate("omni.physx.fabric", False)
