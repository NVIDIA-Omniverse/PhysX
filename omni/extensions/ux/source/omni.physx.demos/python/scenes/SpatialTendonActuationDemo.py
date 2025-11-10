# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import math
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Sdf, UsdLux
import omni.kit.commands
import omni.physxdemos as demo
from omni.physx.scripts import physicsUtils


class TendonScaleDemo(demo.Base):
    title = "Spatial Tendon Actuation"
    category = demo.Categories.ARTICULATIONS
    short_description = "Snippet demonstrating actuation using a spatial tendon."
    description = (
        "The two opposite moving links of the articulations are actuated by a spatial tendon that spans "
        "all three links with an attachment each. The root and leaf attachment are on the moving links and the "
        "leaf force and equal root reaction force are holding up and moving the links."
    )

    params = {"Enable_Motion": demo.CheckboxParam(True)}

    def create(self, stage, Enable_Motion):
        self._stage = stage
        self._default_prim_path, scene = demo.setup_physics_scene(self, stage, primPathAsString = False, upAxis = UsdGeom.Tokens.y, metersPerUnit = 1.0)
        self._gravity_magnitude = 9.81  # [m/s^2]
        self._offset_amplitude = 0.3  # [m]
        self._attachment_amplitude = 0.2  # [m]
        self._motion_frequency = 0.25  # [Hz]
        self._link_dimensions = Gf.Vec3f(0.45, 0.05, 0.05)  # [m]
        self._link_density = 1000.0  # [kg/m^3]

        # link colors
        self._staticColor = demo.get_static_color()
        self._dynamicColor = demo.get_primary_color()

        self._setup_articulations()
        self._enable_tendon_debug_visibility()
        if Enable_Motion:
            self._time = 0.0  # motion variable tracking sim time
            self._setup_callbacks()

        room = demo.get_demo_room(self, stage, zoom = 0.5, hasTable = False, camElevation = 100.0, overrideSphericalLightIntensity = 2.0e5)

    def _create_moving_link(
        self, link_path: Sdf.Path, parent_path: Sdf.Path, translation: Gf.Vec3f, parent_joint_offset: Gf.Vec3f
    ):
        left_link = UsdGeom.Cube.Define(self._stage, link_path)
        left_link.GetSizeAttr().Set(1.0)
        left_link.AddTranslateOp().Set(translation)
        left_link.AddScaleOp().Set(self._link_dimensions)
        left_link.CreateDisplayColorAttr().Set([self._dynamicColor])
        omni.kit.commands.execute(
            "SetRigidBodyCommand", path=link_path, approximationShape="convexHull", kinematic=False
        )
        mass_api = UsdPhysics.MassAPI.Apply(left_link.GetPrim())
        mass_api.CreateDensityAttr().Set(self._link_density)
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, link_path.AppendChild("revoluteJoint"))
        joint.CreateBody0Rel().SetTargets([parent_path])
        joint.CreateBody1Rel().SetTargets([link_path])
        joint.CreateAxisAttr("Z")
        joint.CreateLocalPos0Attr().Set(parent_joint_offset)
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(0.0))
        joint.CreateLocalPos1Attr().Set(-parent_joint_offset)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(0.0))

    def _create_articulation(self, articulation_name: str, offset: Gf.Vec3f):
        basePath = self._default_prim_path.AppendChild(articulation_name + "_BaseLink")
        base = UsdGeom.Cube.Define(self._stage, basePath)
        base.GetSizeAttr().Set(1.0)
        base.AddTranslateOp().Set(offset)
        base.AddScaleOp().Set(self._link_dimensions)
        base.CreateDisplayColorAttr().Set([self._staticColor])
        omni.kit.commands.execute(
            "SetRigidBodyCommand", path=base.GetPath(), approximationShape="convexHull", kinematic=False
        )
        fixedJoint = UsdPhysics.FixedJoint.Define(
            self._stage, self._default_prim_path.AppendChild(articulation_name + "_FixedRootJoint")
        )
        fixedJoint.CreateBody1Rel().SetTargets([base.GetPath()])
        UsdPhysics.ArticulationRootAPI.Apply(fixedJoint.GetPrim())

        link_offset = Gf.Vec3f(self._link_dimensions[0], 0, 0)
        # position is 0.5 because link shape is achieved via scaling that needs to be considered in the joint pos
        joint_offset = Gf.Vec3f(0.5, 0, 0)
        left_link_path = self._default_prim_path.AppendChild(articulation_name + "_leftLink")
        self._create_moving_link(link_path=left_link_path, parent_path=basePath, translation=offset - link_offset, parent_joint_offset=-joint_offset)
        right_link_path = self._default_prim_path.AppendChild(articulation_name + "_right_link")
        self._create_moving_link(link_path=right_link_path, parent_path=basePath, translation=offset + link_offset, parent_joint_offset=joint_offset)
        return self._setup_spatial_tendon(base_link_path=basePath, left_link_path=left_link_path, right_link_path=right_link_path)

    def _setup_articulations(self):
        apis = self._create_articulation("Offset_Actuation", Gf.Vec3f(0.0, 1.5, -0.5))
        self._offset_attachment = apis[0]
        apis = self._create_articulation("Attachment_Actuation", Gf.Vec3f(0.0, 1.5, 0.5))
        self._local_pos_attachment = apis[1]

    def _setup_spatial_tendon(self, base_link_path: Sdf.Path, left_link_path: Sdf.Path, right_link_path: Sdf.Path):
        """
            Sets up a spatial tendon with a root attachment on the left link, a regular attachment on the base, and a
            leaf attachment on the right link.

            The root and leaf attachment are at the center of gravity of the links, and the base link's attachment is
            a link half-length above the fixed-base's COG in the stage-up direction (+Y).

            The code sizes the tendon's stiffness based on the tendon geometry and link mass
        """
        base_prim = self._stage.GetPrimAtPath(base_link_path)
        left_link_prim = self._stage.GetPrimAtPath(left_link_path)
        right_link_prim = self._stage.GetPrimAtPath(right_link_path)

        half_length = self._link_dimensions[0] * 0.5

        # Tendon stiffness dimensioning
        # We could set the rest length to -1 to autocompute it but we calculate it here to size the tendon stiffness
        # based on a desired deviation from the rest length
        # the rest length is 2 * sqrt((2 * half_length)^2 + half_length^2) = 2 * half_length * sqrt(5)
        rest_length = 2.0 * half_length * math.sqrt(5.0)

        # The goal is to have the revolute joints deviate just a few degrees from the horizontal
        # and we compute the length of the tendon at that angle:
        deviationAngle = 3.0 * math.pi / 180.0
        # Distances from the base-link attachment to the root and leaf attachments
        vertical_distance = half_length * (1.0 + math.sin(deviationAngle))
        horizontal_distance = half_length * (1.0 + math.cos(deviationAngle))
        deviated_length = 2.0 * math.sqrt(vertical_distance ** 2.0 + horizontal_distance ** 2.0)

        # At rest, the force on the leaf attachment is (deviated_length - rest_length) * tendon_stiffness.
        # The force points from the moving links' COG to the fixed-base attachment.
        # The force needs to be equal to the gravity force acting on the link, projected onto the tendon.
        # An equal and opposing (in the direction of the tendon) force acts on the root-attachment link and will hold
        # that link up. In order to calculate the tendon stiffness that produces that force, we consider the forces
        # and attachment geometry at zero joint angles.
        link_mass = self._link_dimensions[0] * self._link_dimensions[1] * self._link_dimensions[2] * self._link_density
        gravity_force = self._gravity_magnitude * link_mass
        # Project onto tendon at rest length / with joints at zero angle
        tendon_force = gravity_force * math.sqrt(5.0)  # gravity_force * 0.5 * rest_length / half_length
        # and compute stiffness to get tendon force at deviated length to hold the link:
        tendon_stiffness = tendon_force / (deviated_length - rest_length)
        tendon_damping = 0.3 * tendon_stiffness

        rootApi = PhysxSchema.PhysxTendonAttachmentRootAPI.Apply(left_link_prim, "Root")
        # center attachment above dynamic link
        PhysxSchema.PhysxTendonAttachmentAPI(rootApi, "Root").CreateLocalPosAttr().Set(Gf.Vec3f(0.0))
        rootApi.CreateStiffnessAttr().Set(tendon_stiffness)
        rootApi.CreateDampingAttr().Set(tendon_damping)

        attachmentApi = PhysxSchema.PhysxTendonAttachmentAPI.Apply(base_prim, "Attachment")
        attachmentApi.CreateParentLinkRel().AddTarget(left_link_path)
        attachmentApi.CreateParentAttachmentAttr().Set("Root")
        # the attachment local pos takes the link scaling into account, just like the joint positions
        attachment_local_pos = half_length / self._link_dimensions[1]
        attachmentApi.CreateLocalPosAttr().Set(Gf.Vec3f(0.0, attachment_local_pos, 0.0))

        leafApi = PhysxSchema.PhysxTendonAttachmentLeafAPI.Apply(right_link_prim, "Leaf")
        tendonApi = PhysxSchema.PhysxTendonAttachmentAPI(leafApi, "Leaf") 
        tendonApi.CreateParentLinkRel().AddTarget(base_link_path)
        tendonApi.CreateParentAttachmentAttr().Set("Attachment")
        tendonApi.CreateLocalPosAttr().Set(Gf.Vec3f(0.0))
        leafApi.CreateRestLengthAttr().Set(rest_length)

        return (rootApi, attachmentApi, leafApi)

    def _enable_tendon_debug_visibility(self):
        isregistry = carb.settings.acquire_settings_interface()
        # setting to two forces debug viz of all tendons
        self._tendon_viz_mode = isregistry.get_as_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS)
        isregistry.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, 2)

    def on_shutdown(self):
        isregistry = carb.settings.acquire_settings_interface()
        isregistry.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, self._tendon_viz_mode)
        self._deregister_callbacks()

    def _setup_callbacks(self):
        # callbacks
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        # subscribe to Physics updates:
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self.on_physics_step
        )

    def _deregister_callbacks(self):
        self._timeline_subscription = None
        self._physics_update_subscription = None

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._time = 0
            # cannot call update tendons here to properly reset the demo, because on new stage load while running,
            # Kit crashes due to a fabric issue otherwise.

    def on_physics_step(self, dt):
        self._time += dt
        self._update_tendons()

    def _update_tendons(self):
        offsetPos = self._offset_amplitude * (1.0 + math.sin(self._time * 2.0 * math.pi * self._motion_frequency - math.pi * 0.5))
        attr = self._offset_attachment.CreateOffsetAttr()
        attr.Set(offsetPos)
        attachment_local_pos = 0.5 * self._link_dimensions[0] / self._link_dimensions[1]
        amp = self._attachment_amplitude / self._link_dimensions[1]
        offsetPos = attachment_local_pos + amp * (1.0 + math.sin(self._time * 2.0 * math.pi * self._motion_frequency - math.pi * 0.5))
        attr = self._local_pos_attachment.CreateLocalPosAttr()
        attr.Set(Gf.Vec3f(0.0, offsetPos, 0.0))
