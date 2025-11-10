# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, UsdLux, Usd
import omni.kit.commands
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils


class TendonStabilizeDemo(demo.Base):
    title = "Spatial Tendon"
    category = demo.Categories.ARTICULATIONS
    short_description = "Snippet demonstrating the spatial tendon feature."
    description = (
        "The capsule link of the articulation is stabilized using a branching spatial tendon. "
        "Press play and interact with the capsule (shift-click it and drag). "
        "Disable the parents checkbox to remove all parent-child relationships of the tendon in order to explore "
        "rigging functionality."
    )

    params = {"Parents": demo.CheckboxParam(True)}

    def create(self, stage, Parents):
        self._setup_parents = Parents
        self._stage = stage

        self._default_prim_path, scene = demo.setup_physics_scene(self, stage, upAxis = UsdGeom.Tokens.y, primPathAsString = False)
        room = demo.get_demo_room(self, stage, zoom = 0.5, hasTable = False, camElevation = 100.0)

        # setup scene and stage:
        self._setup_articulation()
        self._setup_tendons()
        self._enable_tendon_debug_visibility()
        

    def _setup_articulation(self):
        self._cube = UsdGeom.Cube.Define(self._stage, self._default_prim_path.AppendChild("Cube"))
        self._cube.GetSizeAttr().Set(100)
        cubeExtent = UsdGeom.Cube.ComputeExtentFromPlugins(self._cube, Usd.TimeCode.Default())
        self._cube.CreateExtentAttr(cubeExtent)
        cubeTranslation = Gf.Vec3d(0, 50, 0)
        self._cube.AddTranslateOp().Set(Gf.Vec3d(0, 50, 0))
        self._cube.CreateDisplayColorAttr().Set([demo.get_primary_color()])
        omni.kit.commands.execute(
            "SetRigidBody", path=self._cube.GetPath(), approximationShape="convexHull", kinematic=False
        )
        fixedJoint = UsdPhysics.FixedJoint.Define(self._stage, self._default_prim_path.AppendChild("FixedRootJoint"))
        fixedJoint.CreateBody1Rel().SetTargets([self._cube.GetPath()])
        fixedJoint.CreateLocalPos1Attr().Set(-cubeTranslation)
        UsdPhysics.ArticulationRootAPI.Apply(fixedJoint.GetPrim())
        self._capsule = UsdGeom.Capsule.Define(self._stage, self._default_prim_path.AppendChild("Capsule"))
        self._capsule.GetRadiusAttr().Set(25.0)
        self._capsule.GetHeightAttr().Set(50.0)
        self._capsule.GetAxisAttr().Set("Y")
        capsuleExtent = UsdGeom.Capsule.ComputeExtentFromPlugins(self._capsule, Usd.TimeCode.Default())
        self._capsule.CreateExtentAttr(capsuleExtent)
        self._capsule.AddTranslateOp().Set(Gf.Vec3d(0, 160, 0))
        self._capsule.CreateDisplayColorAttr().Set([demo.get_primary_color()])
        omni.kit.commands.execute(
            "SetRigidBody", path=self._capsule.GetPath(), approximationShape="convexHull", kinematic=False
        )
        sphericalJoint = UsdPhysics.SphericalJoint.Define(
            self._stage, self._capsule.GetPath().AppendChild("sphericalJoint")
        )
        sphericalJoint.CreateAxisAttr("Y")
        sphericalJoint.CreateConeAngle0LimitAttr(45.0)
        sphericalJoint.CreateConeAngle1LimitAttr(45.0)
        sphericalJoint.CreateCollisionEnabledAttr().Set(True)
        sphericalJoint.CreateBody0Rel().SetTargets([self._cube.GetPath()])
        sphericalJoint.CreateBody1Rel().SetTargets([self._capsule.GetPath()])

        sphericalJoint.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        sphericalJoint.CreateLocalRot0Attr().Set(Gf.Quatf(0, Gf.Vec3f(1, 0, 0)))

        sphericalJoint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, -110.0, 0.0))
        sphericalJoint.CreateLocalRot1Attr().Set(Gf.Quatf(0, Gf.Vec3f(1, 0, 0)))

    def _setup_tendons(self):
        cubePrim = self._cube.GetPrim()
        capsulePrim = self._capsule.GetPrim()

        # setup cube
        rootApi = PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(cubePrim, "root")
        # center attachment above dynamic link
        PhysxSchema.PhysxTendonAttachmentAPI(rootApi, "root").CreateLocalPosAttr().Set(Gf.Vec3f(0.0))
        rootApi.CreateStiffnessAttr().Set(5000.0)
        rootApi.CreateDampingAttr().Set(1000.0)

        coordinates = [Gf.Vec3f(50, 50, 0), Gf.Vec3f(-50, 50, 0), Gf.Vec3f(0, 50, 50), Gf.Vec3f(0, 50, -50)]
        for i, c in enumerate(coordinates):
            attachmentApi = PhysxSchema.PhysxTendonAttachmentAPI.Apply(cubePrim, f"R{i}")
            if self._setup_parents:
                attachmentApi.CreateParentLinkRel().AddTarget(cubePrim.GetPath())
                attachmentApi.CreateParentAttachmentAttr().Set("root")
            attachmentApi.CreateLocalPosAttr().Set(c)

        coordinates = [Gf.Vec3f(25, 10, 0), Gf.Vec3f(-25, 10, 0), Gf.Vec3f(0, 10, 25), Gf.Vec3f(0, 10, -25)]
        for i, c in enumerate(coordinates):
            leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(capsulePrim, f"L{i}")
            if self._setup_parents:
                PhysxSchema.PhysxTendonAttachmentAPI(leafApi, f"L{i}").CreateParentLinkRel().AddTarget(cubePrim.GetPath())
                PhysxSchema.PhysxTendonAttachmentAPI(leafApi, f"L{i}").CreateParentAttachmentAttr().Set(f"R{i}")
            PhysxSchema.PhysxTendonAttachmentAPI(leafApi, f"L{i}").CreateLocalPosAttr().Set(c)

    def _enable_tendon_debug_visibility(self):
        isregistry = carb.settings.acquire_settings_interface()
        # setting to two forces debug viz of all tendons
        self._tendon_viz_mode = isregistry.get_as_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS)
        isregistry.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, 2)

    def on_shutdown(self):
        isregistry = carb.settings.acquire_settings_interface()
        isregistry.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, self._tendon_viz_mode)
