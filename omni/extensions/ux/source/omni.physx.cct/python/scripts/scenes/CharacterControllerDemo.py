# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.physx.scripts import physicsUtils
from pxr import Usd, UsdGeom, Gf, UsdPhysics, PhysxSchema, Vt, Sdf, UsdUtils, PhysicsSchemaTools
import omni.physxdemos as demo
import carb.input
import omni.appwindow
import omni.timeline
import carb.windowing
from omni import ui
from omni.physx import get_physx_scene_query_interface, get_physx_simulation_interface
from carb.input import MouseEventType, EVENT_TYPE_ALL
import omni.physx.scripts.utils as core_utils
from omni.kit.viewport.utility.camera_state import ViewportCameraState
import omni.physxcct.scripts.utils as cct_utils
import omni.kit.viewport.utility as vp_utils


class CharacterControllerDemo(demo.Base):
    title = "Python"
    category = demo.Categories.CCT
    short_description = "Base demo showing character controller setup through Python utilities."
    description = "Play (space) to run the simulation. Use WASD to move the controller, E to jump, mouse move to look, left mouse-button to shoot the dynamic object or gamepad left-stick to move and right-stick to rotate the camera."
    tags = ["Python API", "Character Controller"]

    params = {
        "mouse_sensitivity": demo.IntParam(25, 0, 100, 1),
        "gamepad_sensitivity": demo.IntParam(25, 0, 100, 1),
        "first_person": demo.CheckboxParam(True),
    }

    def reset(self):
        self.shoot = False
        self.captured = False

    def createScene(self, stage, color):
        self.stage_id = UsdUtils.StageCache.Get().GetId(stage).ToLongInt()

        # Dynamic Box
        boxActorPath = self.defaultPrimPath + "/boxActor"

        size = 25.0
        position = Gf.Vec3f(0.0, 0.0, 500.0)
        orientation = Gf.Quatf(1.0)
        linVelocity = Gf.Vec3f(2.0, 1.0, 2.0)
        angularVelocity = Gf.Vec3f(180.0, 0.0, 0.0)

        cubeGeom = UsdGeom.Cube.Define(stage, boxActorPath)
        cubePrim = stage.GetPrimAtPath(boxActorPath)
        cubeGeom.CreateSizeAttr(size)
        half_extent = size / 2
        cubeGeom.CreateExtentAttr([(-half_extent, -half_extent, -half_extent), (half_extent, half_extent, half_extent)])
        cubeGeom.AddTranslateOp().Set(position)
        cubeGeom.AddOrientOp().Set(orientation)
        cubeGeom.AddScaleOp().Set(Gf.Vec3f(1.0))
        cubeGeom.CreateDisplayColorAttr().Set([color])

        UsdPhysics.CollisionAPI.Apply(cubePrim)
        physicsAPI = UsdPhysics.RigidBodyAPI.Apply(cubePrim)
        physicsAPI.CreateVelocityAttr().Set(linVelocity)
        physicsAPI.CreateAngularVelocityAttr().Set(angularVelocity)
        UsdPhysics.MassAPI.Apply(cubePrim)

    def create(self, stage, mouse_sensitivity, gamepad_sensitivity, first_person):
        self.reset()
        self.hit_boxes = []
        self.defaultPrimPath, scene = demo.setup_physics_scene(self, stage)
        room = demo.get_demo_room(self, stage, createCamera = False)

        self.createScene(stage, Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0))

        # Capsule
        cct_path = self.defaultPrimPath + "/capsuleActor"
        cct_utils.spawn_capsule(stage, cct_path, Gf.Vec3f(500, 0, 250), 100, 50)

        # Setup CCT
        camera_path = "/OmniverseKit_Persp" if first_person else None
        self._cct = cct_utils.CharacterController(cct_path, camera_path, True, 0.01)
        self._cct.activate(stage)

        stream = omni.timeline.get_timeline_interface().get_timeline_event_stream()
        self._timeline_subscription = stream.create_subscription_to_pop(self._on_timeline_event)

        self.appwindow = omni.appwindow.get_default_app_window()
        self._mouse = self.appwindow.get_mouse()
        input_iface = carb.input.acquire_input_interface()
        self._mouse_sub = input_iface.subscribe_to_input_events(self._mouse_info_event_cb, EVENT_TYPE_ALL, self._mouse, 0)
        self._viewport_overlay_frame = None
        self._first_person = first_person

        # Setup CCT controls
        self._cct.setup_controls(500, cct_utils.ControlFlag.DEFAULT)
        self._cct.control_state.mouse_sensitivity = mouse_sensitivity
        self._cct.control_state.gamepad_sensitivity = gamepad_sensitivity * 10

        # reset camera pos and view
        customLayerData = {
            "cameraSettings": {
                "Perspective": {"position": Gf.Vec3d(2407, 2407, 2638), "radius": 500, "target": Gf.Vec3d(0, 0, 0)},
                "boundCamera": "/OmniverseKit_Persp",
            }
        }
        stage.GetRootLayer().customLayerData = customLayerData

    def _cleanup_overlay(self):
        if self._viewport_overlay_frame:
            self._viewport_overlay_frame.visible = False
            self._viewport_overlay_frame = None
        self._viewport_window = None

    def refresh_crosshair(self, is_playing):
        if is_playing:
            if not self._viewport_overlay_frame:
                self._viewport_window = vp_utils.get_active_viewport_window()
                self._viewport_overlay_frame = self._viewport_window.get_frame("omni.physx.cct.crosshair_frame")
                self._viewport_overlay_frame.visible = True
                with self._viewport_overlay_frame:
                    with ui.Placer(offset_x=ui.Percent(50), offset_y=ui.Percent(50)):
                        self._crosshair = ui.Circle(width=10, height=10, radius=10, style={"background_color": 4283782485})
        elif self._viewport_overlay_frame:
            self._cleanup_overlay()

    def update(self, stage, dt, viewport, physxIFace):
        is_playing = omni.timeline.get_timeline_interface().is_playing()

        if self._first_person:
            self.refresh_crosshair(is_playing)

        if not is_playing:
            return

        with Sdf.ChangeBlock():
            origColor = Vt.Vec3fArray([Gf.Vec3f(71.0 / 255.0, 105.0 / 255.0, 1.0)])
            self.set_colors(origColor, True)

        kilogramsPerUnit = UsdPhysics.GetStageKilogramsPerUnit(stage)
        metersPerUnit = UsdGeom.GetStageMetersPerUnit(stage)

        camera_prim = stage.GetPrimAtPath(vp_utils.get_active_viewport_camera_path())
        camera_th = core_utils.CameraTransformHelper(camera_prim)
        cameraPos = camera_th.get_pos()
        cameraForward = camera_th.get_forward()

        origin = carb.Float3(cameraPos[0], cameraPos[1], cameraPos[2])
        rayDir = carb.Float3(cameraForward[0], cameraForward[1], cameraForward[2])

        hitInfo = get_physx_scene_query_interface().raycast_closest(origin, rayDir, 10000.0)
        if hitInfo["hit"]:
            usdGeom = UsdGeom.Mesh.Get(stage, hitInfo["rigidBody"])
            usdPrim = usdGeom.GetPrim()
            if usdPrim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.hit_boxes.append(usdGeom)
                if self.shoot:
                    impulseSize = 200.0 / (metersPerUnit * kilogramsPerUnit)
                    impulse = carb.Float3(
                        cameraForward[0] * impulseSize,
                        cameraForward[1] * impulseSize,
                        cameraForward[2] * impulseSize
                    )                    
                    rbo_encoded = PhysicsSchemaTools.sdfPathToInt(hitInfo["rigidBody"])
                    get_physx_simulation_interface().apply_force_at_pos(self.stage_id, rbo_encoded, impulse, hitInfo["position"], "Impulse")
        self.shoot = False

        with Sdf.ChangeBlock():
            hitColor = Vt.Vec3fArray([Gf.Vec3f(180.0 / 255.0, 16.0 / 255.0, 0.0)])
            self.set_colors(hitColor, False)

    def set_colors(self, color, reset_array):
        for usdGeom in self.hit_boxes:
            usdGeom.GetDisplayColorAttr().Set(color)

        if reset_array:
            self.hit_boxes = []
        return

    def _mouse_info_event_cb(self, event_in, *args, **kwargs):
        if self.captured:
            event = event_in.event
            if event.type == MouseEventType.LEFT_BUTTON_DOWN:
                self.shoot = True
        return not self.captured

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.PLAY):
            self.captured = True
        elif e.type == int(omni.timeline.TimelineEventType.STOP):
            self.reset()
        elif e.type == int(omni.timeline.TimelineEventType.PAUSE):
            self.captured = False

    def on_shutdown(self):
        carb.input.acquire_input_interface().unsubscribe_to_input_events(self._mouse_sub)
        self._mouse_sub = None
        self._timeline_subscription = None
        self._cleanup_overlay()
        self._cct.disable()
        self._cct.shutdown()
        self._cct = None
