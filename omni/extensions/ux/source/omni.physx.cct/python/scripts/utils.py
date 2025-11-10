# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb.input
import carb.windowing
from carb.eventdispatcher import get_eventdispatcher
import omni.appwindow
import omni.kit.app
import omni.timeline
import omni.usd
import omni.stageupdate
import omni.kit.commands
from pxr import Usd, UsdGeom, PhysxSchema, Gf
from omni.physxcct import get_physx_cct_interface, CctEvent
from omni.physx.scripts import utils
from functools import partial
from enum import IntFlag, IntEnum, auto
from carb.input import KeyboardInput, GamepadInput
import omni.kit.viewport.utility as vp_utils
from omni.kit.viewport.utility.camera_state import ViewportCameraState
from omni.physxui import get_input_manager


def register_stage_update_node(display_name, priority=8, **kwargs):
    stage_update = omni.stageupdate.get_stage_update_interface()
    stage_update_node = stage_update.create_stage_update_node(display_name, **kwargs)
    nodes = stage_update.get_stage_update_nodes()
    # defaults to 8 = pre-physics
    stage_update.set_stage_update_node_order(len(nodes) - 1, priority)

    return stage_update_node


def spawn_capsule(stage, path, pos, height=50, radius=25):
    """
    Capsule prim creation helper.
    """
    up_axis = UsdGeom.GetStageUpAxis(stage)
    capsule_geom = UsdGeom.Capsule.Define(stage, path)
    capsule_geom.CreateHeightAttr(height)
    capsule_geom.CreateRadiusAttr(radius)
    capsule_geom.CreateExtentAttr([(-radius, -radius, -height), (radius, radius, height)])
    capsule_geom.CreateAxisAttr(up_axis)
    capsule_geom.AddTranslateOp().Set(pos)
    capsule_geom.AddOrientOp().Set(Gf.Quatf(1.0))
    capsule_geom.AddScaleOp().Set(Gf.Vec3f(1.0))
    return capsule_geom


def gen_camera_paths(stage):
    if stage is None:
        return

    for prim in Usd.PrimRange(stage.GetPseudoRoot()):
        if prim.IsA(UsdGeom.Camera):
            yield prim.GetPath()


class ControlFlag(IntFlag):
    """ Flags for activating features in setup_controls. """

    VERTICAL_MOVEMENT = auto()
    """ Full vertical movement with gravity off, jumping with gravity on. """

    KBD_MOUSE = auto()
    """ Use keyboard and mouse controls """

    GAMEPAD = auto()
    """ Use gamepad controls """

    DEFAULT = KBD_MOUSE | GAMEPAD | VERTICAL_MOVEMENT
    """ Default setup with both keyboard/mouse and gamepad """

    KBD_MOUSE_DEFAULT = KBD_MOUSE | VERTICAL_MOVEMENT
    """ Default setup with keyboard/mouse only """

    GAMEPAD_DEFAULT = GAMEPAD | VERTICAL_MOVEMENT
    """ Default setup with gamepad only """


class ControlAction(IntEnum):
    FORWARD = auto()
    BACKWARD = auto()
    RIGHT = auto()
    LEFT = auto()
    UP = auto()
    DOWN = auto()
    STICK_RIGHT_X = auto()
    STICK_RIGHT_Y = auto()
    TOTAL = auto()


class ControlState:
    """
    Keeps track of cct controls internal state.
    """
    def __init__(self, speed, with_camera):
        self.inputs = [0.0 for _ in range(ControlAction.TOTAL)]

        self.action_subs = []
        self.action_callbacks = dict()
        self.update_sub = None
        self.cct_event_sub = None
        self.timeline_event_sub = None
        self.stage_update_node = None

        self.speed = speed
        self.jump_speed = speed
        self.jump_time = -1.0
        self.ground_contact = True
        self.first_frame = True
        self.paused = False

        app_window = omni.appwindow.get_default_app_window()
        action_mapping_set_path = app_window.get_action_mapping_set_path()
        self._input = carb.input.acquire_input_interface()
        self._action_mapping_set = self._input.get_action_mapping_set_by_path(action_mapping_set_path)

        if with_camera:
            self.min_pitch = -85.0
            self.max_pitch = 85.0
            self.mouse_sensitivity = 25
            self.gamepad_sensitivity = 25
            self.pitch = 0
            self.yaw = 0

    """
    Cleans up event subscriptions and unregisters actions
    """
    def shutdown(self):
        for sub in self.action_subs:
            self._input.unsubscribe_to_action_events(sub)

        for name in self.action_callbacks.keys():
            get_input_manager().unregister_action(name)

        self._input = None

        self.action_subs = []
        self.action_callbacks = None
        self.update_sub = None
        self.cct_event_sub = None
        self.timeline_event_sub = None
        self.stage_update_node = None

    """
    Helper to register a keyboard action
    """
    def register_keyboard_action(self, name, kb_input, modifier, callback_fn):
        get_input_manager().register_keyboard_action(name, kb_input, modifier)
        self.action_subs.append(self._input.subscribe_to_action_events(self._action_mapping_set, name, callback_fn))
        self.action_callbacks[name] = callback_fn

    """
    Helper to register a gamepad action
    """
    def register_gamepad_action(self, name, pad_input, pad_index, callback_fn):
        get_input_manager().register_gamepad_action(name, pad_input, pad_index)
        self.action_subs.append(self._input.subscribe_to_action_events(self._action_mapping_set, name, callback_fn))
        self.action_callbacks[name] = callback_fn

    """
    Refresh subscriptions. Done on Play to make sure all subs are in place.
    """
    def refresh_subscriptions(self):
        for sub in self.action_subs:
            self._input.unsubscribe_to_action_events(sub)

        self.action_subs = []

        for name, callback_fn in self.action_callbacks.items():
            self.action_subs.append(self._input.subscribe_to_action_events(self._action_mapping_set, name, callback_fn))



class CharacterController:
    def __init__(self, cct_path, fp_cam_path=None, enable_gravity=True, max_climbable_offset=0.5):
        self.path = cct_path
        self.first_person_camera = fp_cam_path
        self.gravity_enabled = enable_gravity
        self.control_state = None
        self.overriden_camera = None
        self.orig_camera_posrot = None
        self.max_climbable_offset = max_climbable_offset

    def activate(self, stage):
        """
        Apply the CCT API to a prim and sets up a CCT according to params while also activating it in the CCT manager that will then handle its movement updates.

        Args:
            stage: USD stage.
            fp_cam_path: Camera prim path to be used for first person mode. Disables first person mode when None.
            enable_gravity: Add PhysicsScene gravity to the CCT when True.
        """

        # setup capsule
        usdPrim = stage.GetPrimAtPath(self.path)
        cctPhysxAPI = PhysxSchema.PhysxCharacterControllerAPI.Apply(usdPrim)
        cctPhysxAPI.GetUpAxisAttr().Set(UsdGeom.GetStageUpAxis(stage))
        cctPhysxAPI.GetStepOffsetAttr().Set(self.max_climbable_offset)

        # setup cct
        physxCct = get_physx_cct_interface()

        if self.first_person_camera:
            physxCct.enable_first_person(self.path, self.first_person_camera)
        else:
            physxCct.disable_first_person(self.path)

        if self.gravity_enabled:
            physxCct.enable_gravity(self.path)
        else:
            physxCct.disable_gravity(self.path)

    def disable(self):
        """
        Explicitly remove a CCT from the CCT manager (it will still be simulated while having the CCT API applied though).
        """

        physxCct = get_physx_cct_interface()
        physxCct.remove_cct(self.path)

    def shutdown(self):
        if self.control_state is not None:
            self.control_state.shutdown()

    def setup_controls(self, speed, flags=ControlFlag.DEFAULT, rebind=None):
        """
        Setup and handle CCT controls for free-look and movement on mouse/keyboard and gamepad.

        Args:
            speed: Desired movement speed.
            flags: Activate control setup features. See ControlFlag docs.
            rebind: Dictionary string:carb.input.KeyboardInput for keyboard control rebinding. Use "Forward", "Backward", "Right", "Left", "Up" and "Down" strings.
        """

        stage = omni.usd.get_context().get_stage()
        up_axis = UsdGeom.GetStageUpAxis(stage)
        with_vertical = flags & ControlFlag.VERTICAL_MOVEMENT

        state = ControlState(speed, self.first_person_camera is not None)
        app_window = omni.appwindow.get_default_app_window()
        windowing = carb.windowing.acquire_windowing_interface()
        os_window = app_window.get_window()

        up, forward, right = utils.get_basis(up_axis)

        # default keyboard keys
        keybinds = {
            "Forward": KeyboardInput.W,
            "Backward": KeyboardInput.S,
            "Right": KeyboardInput.D,
            "Left": KeyboardInput.A,
            "Up": KeyboardInput.E,
            "Down": KeyboardInput.Q,
        }

        # custom rebind
        if rebind:
            for name in keybinds.keys():
                bind = rebind.get(name)
                if bind:
                    keybinds[name] = bind

        def on_button_action(key, inv, evt):
            if evt.flags & carb.input.BUTTON_FLAG_PRESSED or evt.flags & carb.input.BUTTON_FLAG_DOWN:
                state.inputs[key] = evt.value * inv
            elif evt.flags & carb.input.BUTTON_FLAG_RELEASED:
                state.inputs[key] = 0

        def on_stick_action(key, inv, evt):
            state.inputs[key] = evt.value * inv if evt.value > 0.2 else 0

        get_input_manager().clear()

        if flags & ControlFlag.KBD_MOUSE:
            state.register_keyboard_action("CctMoveForward", keybinds["Forward"], 0, partial(on_button_action, ControlAction.FORWARD, 1))
            state.register_keyboard_action("CctMoveBackward", keybinds["Backward"], 0, partial(on_button_action, ControlAction.BACKWARD, -1))
            state.register_keyboard_action("CctMoveRight", keybinds["Right"], 0, partial(on_button_action, ControlAction.RIGHT, 1))
            state.register_keyboard_action("CctMoveLeft", keybinds["Left"], 0, partial(on_button_action, ControlAction.LEFT, -1))

            if with_vertical:
                state.register_keyboard_action("CctMoveUp", keybinds["Up"], 0, partial(on_button_action, ControlAction.UP, 1))
                state.register_keyboard_action("CctMoveDown", keybinds["Down"], 0, partial(on_button_action, ControlAction.DOWN, -1))

        if flags & ControlFlag.GAMEPAD:
            state.register_gamepad_action("CctGamepadMoveForward", GamepadInput.LEFT_STICK_UP, 0, partial(on_stick_action, ControlAction.FORWARD, 1))
            state.register_gamepad_action("CctGamepadMoveBackward", GamepadInput.LEFT_STICK_DOWN, 0, partial(on_stick_action, ControlAction.BACKWARD, -1))
            state.register_gamepad_action("CctGamepadMoveRight", GamepadInput.LEFT_STICK_RIGHT, 0, partial(on_stick_action, ControlAction.RIGHT, 1))
            state.register_gamepad_action("CctGamepadMoveLeft", GamepadInput.LEFT_STICK_LEFT, 0, partial(on_stick_action, ControlAction.LEFT, -1))

            if with_vertical:
                state.register_gamepad_action("CctGamepadMoveUp", GamepadInput.RIGHT_SHOULDER, 0, partial(on_stick_action, ControlAction.UP, 1))
                state.register_gamepad_action("CctGamepadMoveDown", GamepadInput.LEFT_SHOULDER, 0, partial(on_stick_action, ControlAction.DOWN, -1))

            if self.first_person_camera:
                state.register_gamepad_action("CctGamepadPitchUp", GamepadInput.RIGHT_STICK_DOWN, 0, partial(on_stick_action, ControlAction.STICK_RIGHT_Y, 1))
                state.register_gamepad_action("CctGamepadPitchDown", GamepadInput.RIGHT_STICK_UP, 0, partial(on_stick_action, ControlAction.STICK_RIGHT_Y, -1))
                state.register_gamepad_action("CctGamepadYawRight", GamepadInput.RIGHT_STICK_RIGHT, 0, partial(on_stick_action, ControlAction.STICK_RIGHT_X, 1))
                state.register_gamepad_action("CctGamepadYawLeft", GamepadInput.RIGHT_STICK_LEFT, 0, partial(on_stick_action, ControlAction.STICK_RIGHT_X, -1))

        def update_movement(dt, state):
            speed = state.speed * dt

            # set horizontal movement, scale to 1 (diagonal with kbd controls can exceed that)
            x = state.inputs[ControlAction.FORWARD] + state.inputs[ControlAction.BACKWARD]
            y = state.inputs[ControlAction.RIGHT] + state.inputs[ControlAction.LEFT]
            move = Gf.Vec3f(x, y, 0).GetNormalized() * speed

            # set vertical movement
            if with_vertical:
                has_gravity_enabled = get_physx_cct_interface().has_gravity_enabled(self.path)
                if has_gravity_enabled:
                    if state.jump_time < 0.0 and state.inputs[ControlAction.UP] != 0 and state.ground_contact:
                        # can start jumping if not already jumping
                        state.jump_time = 0.0
                    elif state.jump_time > 0.0 and state.ground_contact:
                        # stop jump after we landed
                        state.jump_time = -1.0
                        move[2] = 0

                    if state.jump_time >= 0.0:
                        # already jumping (do first jump move immediately as the jump command comes in, without a frame delay)
                        # the falling due to gravity acceleration part is already done in the built-in controller code!
                        move[2] = state.jump_speed * dt
                        state.jump_time += dt
                else:
                    move[2] = (state.inputs[ControlAction.UP] + state.inputs[ControlAction.DOWN]) * speed

            get_physx_cct_interface().set_move(self.path, (move[0], move[1], move[2]))

        def recenter_cursor():
            cx = int(app_window.get_width() / 2)
            cy = int(app_window.get_height() / 2)
            windowing.set_cursor_position(os_window, (cx, cy))
            return cx, cy

        def update_camera(dt, state, cct_prim):
            # set camera position to capsule
            xfCache = UsdGeom.XformCache()
            tm = xfCache.GetLocalToWorldTransform(cct_prim)
            height = get_physx_cct_interface().get_controller_height(self.path)
            pos = tm.ExtractTranslation() + up * height

            # mouse and gamepad freelook
            delta_x = 0
            delta_y = 0

            # can't do freelook without an active window
            if os_window and (flags & ControlFlag.KBD_MOUSE):
                cursor_pos = windowing.get_cursor_position(os_window)
                cx, cy = recenter_cursor()
                delta_x = (cursor_pos.x - cx) * state.mouse_sensitivity
                delta_y = (cursor_pos.y - cy) * state.mouse_sensitivity

            if flags & ControlFlag.GAMEPAD:
                pad_delta_x = state.inputs[ControlAction.STICK_RIGHT_X]
                pad_delta_y = state.inputs[ControlAction.STICK_RIGHT_Y]

                if pad_delta_x != 0 or pad_delta_y != 0:
                    delta_x = pad_delta_x * state.gamepad_sensitivity
                    delta_y = pad_delta_y * state.gamepad_sensitivity

            r_pitch = state.pitch - delta_y * dt
            state.pitch = max(state.min_pitch, min(r_pitch, state.max_pitch))
            state.yaw += -delta_x * dt

            rot_yaw = Gf.Rotation(up, state.yaw)
            rot_pitch = Gf.Rotation(right, state.pitch)

            rel_t = (rot_pitch * rot_yaw).TransformDir(forward)
            abs_t = pos + rel_t

            # update camera target
            camera_state = ViewportCameraState()
            camera_prim = camera_state.usd_camera
            camera_path = camera_prim.GetPath()

            parent_xform = camera_prim.ComputeParentToWorldTransform(Usd.TimeCode.Default())
            iparent_xform = parent_xform.GetInverse()
            local_pos = iparent_xform.Transform(pos)
            local_target = iparent_xform.Transform(abs_t)
            local_xform = Gf.Matrix4d(1).SetLookAt(local_pos, local_target, up).GetInverse()

            omni.kit.commands.execute("TransformPrimCommand", path=camera_path, new_transform_matrix=local_xform)

        def on_cct_event(e):
            if e.type == int(CctEvent.COLLISION_DOWN):
                state.ground_contact = e.payload['collision']

        state.cct_event_sub = get_physx_cct_interface().get_cct_event_stream().create_subscription_to_pop(on_cct_event)

        def on_play():
            if self.first_person_camera:
                if state.paused:
                    recenter_cursor()
                else:
                    camera_state = ViewportCameraState()
                    self.orig_camera_posrot = (camera_state.position_world, camera_state.target_world)
            state.refresh_subscriptions()
            state.paused = False

        def on_pause():
            state.paused = True

        def on_timeline_event(e):
            if e.type == int(omni.timeline.TimelineEventType.PLAY):
                on_play()

            if e.type == int(omni.timeline.TimelineEventType.STOP):
                state.first_frame = True
                state.paused = False
                state.yaw = 0
                state.pitch = 0
                if self.overriden_camera:
                    vp_utils.get_active_viewport().set_active_camera(self.overriden_camera)
                    if self.orig_camera_posrot:
                        camera_state = ViewportCameraState()
                        camera_state.set_position_world(self.orig_camera_posrot[0], False)
                        camera_state.set_target_world(self.orig_camera_posrot[1], True)
                self.orig_camera_posrot = None
            if e.type == int(omni.timeline.TimelineEventType.PAUSE):
                on_pause()

        timeline = omni.timeline.get_timeline_interface()

        # setup base state as if the events already came
        if timeline.is_playing():
            on_play()
        elif not timeline.is_playing() and not timeline.is_stopped():
            on_play()
            on_pause()

        timeline_events = timeline.get_timeline_event_stream()
        state.timeline_event_sub = timeline_events.create_subscription_to_pop(on_timeline_event)

        def on_update(e):
            if self.path is None or not timeline.is_playing():
                return

            stage = omni.usd.get_context().get_stage()
            cct_prim = stage.GetPrimAtPath(self.path)
            if not cct_prim.IsValid():
                return

            dt = e.payload["dt"]

            if self.first_person_camera:
                if state.first_frame:
                    active_viewport = vp_utils.get_active_viewport()
                    if active_viewport:
                        state.first_frame = False
                        self.overriden_camera = active_viewport.get_active_camera()
                        active_viewport.set_active_camera(self.first_person_camera)
                        recenter_cursor()
                else:
                    update_camera(dt, state, cct_prim)

        state.update_sub = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=on_update,
            observer_name="omni.physxcct.CharacterController",
        )

        def on_stage_update(_, dt):
            if self.path is None or not omni.timeline.get_timeline_interface().is_playing():
                return

            update_movement(dt, state)

        state.stage_update_node = register_stage_update_node("cct_controls", on_update_fn=on_stage_update)
        self.control_state = state
