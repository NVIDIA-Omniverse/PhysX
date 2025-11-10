# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import math
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, UsdLux, Sdf
import omni.kit.commands
import omni.physxdemos as demo


class FixedTendonDemo(demo.Base):
    title = "Fixed Tendon"
    category = demo.Categories.ARTICULATIONS
    short_description = "Snippet demonstrating the fixed tendon feature."
    description = (
        "The revolute joint between the two blue links is coupled to the base revolute joint by a fixed tendon. "
        "A drive on the base revolute joint toggles between two target positions. "
        "If the Use Limits checkbox is enabled, the fixed tendon is setup using tendon-length limits to achieve the "
        "same motion. Inspect the snippet source code or the revolute joint properties for the tendon API parameters."
    )

    params = {"Use_Limits": demo.CheckboxParam(False), "Disable_Motion": demo.CheckboxParam(False)}

    def create(self, stage, Use_Limits, Disable_Motion):
        self._stage = stage
        
        self._default_prim_path, scene = demo.setup_physics_scene(self, stage, upAxis = UsdGeom.Tokens.y, primPathAsString = False)
        self._room = demo.get_demo_room(self, self._stage, zoom = 0.5, hasTable = False, floorOffset = -200.0, camElevation = 200.0)
        self._staticColor = demo.get_static_color()
        self._dynamicColor = demo.get_primary_color()

        halfLength = 50.0
        self._linkGeometry = Gf.Vec3d(2.0 * halfLength, 10.0, 10.0)  # cm
        self._linkDensity = 0.001  # kg / cm^3
        self._gravityMagnitude = 981.0  # cm/s^2
        self._linkMass = self._linkGeometry[0] * self._linkGeometry[1] * self._linkGeometry[2] * self._linkDensity
        # calculate the torque due to gravity of the two moving links on the root drive joint
        dLink1 = halfLength  # distance drive joint to first link COG
        dLink2 = 3.0 * halfLength  # distance drive joint to second link COG
        gravityTorque = self._gravityMagnitude * self._linkMass * (dLink1 + dLink2)
        # size drive stiffness to allow a few deg deviation when the drive is holding the links horizontal
        # assuming no contribution of the fixed tendon joint at the drive joint
        deviationAngle = 2.0
        self._driveStiffness = gravityTorque / deviationAngle
        self._driveDamping = self._driveStiffness * 0.3

        # size fixed tendon to allow just a few degrees deviation as well
        tendonGravityTorque = halfLength * self._linkMass * self._gravityMagnitude
        self._tendonStiffness = tendonGravityTorque / deviationAngle
        self._tendonDamping = self._tendonStiffness * 0.3
        # tendon gearing such that the second joint angle mirrors the first ([1, -1] also works)
        self._tendonGearing = [1, -1]
        # setup the force coefficients such that the first, driven joint is not affected by the tendon, but the second
        # joint is driven to mirror the first by the tendon. The sign must be the same for the force (here torque) to
        # drive the joint in the correct direction.
        self._tendonForceCoefficients = [0, -1]

        # symmetric length-limit to use if limits are active
        self._symmetric_limit = 1.0  # = 1 deg with |gearing| = 1

        # set drive target position - the drive will toggle between zero and this value:
        self._drive_target = 45.0  # deg

        self._motion_period = 4.5  # seconds
        self._reset_time()

        self._setup_articulation()
        self._setup_tendons(Use_Limits)

        if not Disable_Motion:
            self._setup_callbacks()

        # force tendon debug visibility
        self._enable_tendon_debug_visibility()

    def _createMovingLink(
        self, linkName: str, jointName: str, parentLinkPath: Sdf.Path, offset: Gf.Vec3f, displayColor: Gf.Vec3f
    ):
        link = UsdGeom.Cube.Define(self._stage, self._default_prim_path.AppendChild(linkName))
        link.GetSizeAttr().Set(1.0)
        link.AddTranslateOp().Set(offset)
        link.AddScaleOp().Set(self._linkGeometry)
        link.CreateDisplayColorAttr().Set([displayColor])
        omni.kit.commands.execute(
            "SetRigidBody", path=link.GetPath(), approximationShape="convexHull", kinematic=False
        )
        joint = UsdPhysics.RevoluteJoint.Define(self._stage, link.GetPath().AppendChild(jointName))
        joint.CreateBody0Rel().SetTargets([parentLinkPath])
        joint.CreateBody1Rel().SetTargets([link.GetPath()])
        joint.CreateAxisAttr("Z")
        # position is 0.5 because link shape is achieved via scaling that needs to be considered in the joint pos
        jointPosition = Gf.Vec3f(0.5, 0, 0)
        joint.CreateLocalPos0Attr().Set(jointPosition)
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(0.0))
        joint.CreateLocalPos1Attr().Set(-jointPosition)
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(0.0))
        return (link, joint)

    def _setup_articulation(self):
        height = 0
        self._base = UsdGeom.Cube.Define(self._stage, self._default_prim_path.AppendChild("Base"))
        self._base.GetSizeAttr().Set(1.0)
        self._base.AddTranslateOp().Set(Gf.Vec3d(0, height, 0))
        self._base.AddScaleOp().Set(self._linkGeometry)
        self._base.CreateDisplayColorAttr().Set([self._staticColor])
        omni.kit.commands.execute(
            "SetRigidBody", path=self._base.GetPath(), approximationShape="convexHull", kinematic=False
        )
        fixedJoint = UsdPhysics.FixedJoint.Define(self._stage, self._default_prim_path.AppendChild("FixedRootJoint"))
        fixedJoint.CreateBody1Rel().SetTargets([self._base.GetPath()])
        UsdPhysics.ArticulationRootAPI.Apply(fixedJoint.GetPrim())
        self._midLink, self._midJoint = self._createMovingLink(
            "midLink", "midJoint", self._base.GetPath(), Gf.Vec3d(self._linkGeometry[0], height, 0), self._dynamicColor
        )
        self._drive = UsdPhysics.DriveAPI.Apply(self._midJoint.GetPrim(), "angular")
        self._drive.CreateDampingAttr(self._driveDamping)
        self._drive.CreateStiffnessAttr(self._driveStiffness)
        self._drive.CreateTargetPositionAttr().Set(0.0)
        self._distalLink, self._distalJoint = self._createMovingLink(
            "distalLink",
            "distalJoint",
            self._midLink.GetPath(),
            Gf.Vec3d(2 * self._linkGeometry[0], height, 0),
            self._dynamicColor,
        )

    def _setup_tendons(self, use_limits):
        midJointPrim = self._midJoint.GetPrim()
        distalJointPrim = self._distalJoint.GetPrim()

        # setup drive joint
        rootApi = PhysxSchema.PhysxTendonAxisRootAPI.Apply(midJointPrim, "fixedTendon")
        rootAxisApi = PhysxSchema.PhysxTendonAxisAPI(rootApi, "fixedTendon")
        if use_limits:
            rootApi.CreateLimitStiffnessAttr().Set(self._tendonStiffness)
            # limit is +/- 1 deg
            rootApi.CreateLowerLimitAttr().Set(-self._symmetric_limit)
            rootApi.CreateUpperLimitAttr().Set(self._symmetric_limit)
        else:
            rootApi.CreateStiffnessAttr().Set(self._tendonStiffness)
        rootApi.CreateDampingAttr().Set(self._tendonDamping)
        rootAxisApi.CreateGearingAttr().Set([self._tendonGearing[0]])
        rootAxisApi.CreateForceCoefficientAttr().Set([self._tendonForceCoefficients[0]])
        # setup second joint
        axisApi = PhysxSchema.PhysxTendonAxisAPI.Apply(distalJointPrim, "fixedTendon")
        axisApi.CreateGearingAttr().Set([self._tendonGearing[1]])
        axisApi.CreateForceCoefficientAttr().Set([self._tendonForceCoefficients[1]])

    def _reset_time(self):
        self._time = 0.25 * self._motion_period  # seconds - variable to track time in physics callbacks

    def on_shutdown(self):
        isregistry = carb.settings.acquire_settings_interface()
        isregistry.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, self._tendon_viz_mode)
        self._deregister_callbacks()
        super().on_shutdown()

    def _enable_tendon_debug_visibility(self):
        isregistry = carb.settings.acquire_settings_interface()
        # setting to two forces debug viz of all tendons
        self._tendon_viz_mode = isregistry.get_as_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS)
        isregistry.set_int(omni.physx.bindings._physx.SETTING_DISPLAY_TENDONS, 2)

    def _setup_callbacks(self):
        self._timeline = omni.timeline.get_timeline_interface()
        stream = self._timeline.get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)
        self._physics_update_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            self.on_physics_step
        )

    def _deregister_callbacks(self):
        self._timeline_subscription = None
        self._physics_update_subscription = None

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.STOP):
            self._reset_time()

    def on_physics_step(self, dt):
        self._time += dt
        self._set_drive_target_angle()

    def _set_drive_target_angle(self):
        periodPos = (self._time % self._motion_period) / self._motion_period
        if periodPos > 0.5:
            self._drive.GetTargetPositionAttr().Set(self._drive_target)
        else:
            self._drive.GetTargetPositionAttr().Set(0.0)
