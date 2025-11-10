# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from pxr import Sdf
import omni.usd
import omni.graph.core as og
from omni.physxcct.scripts import utils as cct_utils
import carb
import carb.input
from omni.physxcct.ogn.OgnCharacterControllerDatabase import OgnCharacterControllerDatabase


# this needs to be in the same order as in carb.input.KeyboardInput ..
key_order = ["Unknown", "Space", "Apostrophe", "Comma", "Minus", "Period", "Slash", "Key0", "Key1", "Key2", "Key3", "Key4", "Key5", "Key6", "Key7", "Key8", "Key9", "Semicolon", "Equal", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "LeftBracket", "Backslash", "RightBracket", "GraveAccent", "Escape", "Tab", "Enter", "Backspace", "Insert", "Del", "Right", "Left", "Down", "Up", "PageUp", "PageDown", "Home", "End", "CapsLock", "ScrollLock", "NumLock", "PrintScreen", "Pause", "F1", "F2", "F3", "F4", "F5", "F6", "F7", "F8", "F9", "F10", "F11", "F12", "Numpad0", "Numpad1", "Numpad2", "Numpad3", "Numpad4", "Numpad5", "Numpad6", "Numpad7", "Numpad8", "Numpad9", "NumpadDel", "NumpadDivide", "NumpadMultiply", "NumpadSubtract", "NumpadAdd", "NumpadEnter", "NumpadEqual", "LeftShift", "LeftControl", "LeftAlt", "LeftSuper", "RightShift", "RightControl", "RightAlt", "RightSuper", "Menu"]
key_index = {}
i = 0
for k in key_order:
    key_index[k] = i
    i += 1


def convert_key(k):
    return carb.input.KeyboardInput(key_index.get(k, 0))


ACTION_NAMES = ["Forward", "Backward", "Right", "Left", "Up", "Down"]


class OgnCharacterControllerState:
    def __init__(self):
        self.cct_handle = None


class OgnCharacterController:
    @staticmethod
    def internal_state():
        return OgnCharacterControllerState()

    @staticmethod
    def release(node):
        state = OgnCharacterControllerDatabase.per_instance_internal_state(node)
        if state.cct_handle:
            state.cct_handle.disable()
            state.cct_handle.shutdown()
            state.cct_handle = None

    @staticmethod
    def activate(db, stage):
        if db.per_instance_state.cct_handle:
            carb.log_warn("OgnCharacterController: Activating an already activated CharacterController. Cleaning up previous instance.")
            db.per_instance_state.cct_handle.disable()
            db.per_instance_state.cct_handle.shutdown()
            db.per_instance_state.cct_handle = None

        cct_path = db.inputs.capsulePath
        if not Sdf.Path.IsValidPathString(cct_path):
            carb.log_error("OgnCharacterController: Invalid capsule path.")
            db.outputs.done = og.ExecutionAttributeState.ENABLED
            return False

        cct_camera = db.inputs.fpCameraPathToken
        if not Sdf.Path.IsValidPathString(cct_camera):
            cct_camera = None

        cct = cct_utils.CharacterController(cct_path, cct_camera, db.inputs.gravity, 0.01)
        cct.activate(stage)

        if db.inputs.setupControls == "Auto":
            if db.inputs.controlsSettings.attributes:
                rebind_attributes = [db.inputs.controlsSettings.attribute_by_name(name) for name in ACTION_NAMES]
                rebind = {a.name: convert_key(a.value) for a in rebind_attributes}

                cct.setup_controls(db.inputs.speed, cct_utils.ControlFlag.DEFAULT, rebind)
                cct.control_state.mouse_sensitivity = db.inputs.controlsSettings.attribute_by_name("MouseSensitivity").value
                cct.control_state.gamepad_sensitivity = db.inputs.controlsSettings.attribute_by_name("GamepadSensitivity").value
            else:
                cct.setup_controls(db.inputs.speed, cct_utils.ControlFlag.DEFAULT)

        db.per_instance_state.cct_handle = cct
        db.outputs.done = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def deactivate(db):
        cct = db.per_instance_state.cct_handle
        if not cct:
            db.outputs.done = og.ExecutionAttributeState.ENABLED
            return False

        cct.disable()
        cct.shutdown()
        db.per_instance_state.cct_handle = None

        db.outputs.done = og.ExecutionAttributeState.ENABLED
        return True

    @staticmethod
    def compute(db) -> bool:
        if db.inputs.activate:
            stage = omni.usd.get_context().get_stage()
            return OgnCharacterController.activate(db, stage)

        if db.inputs.deactivate:
            return OgnCharacterController.deactivate(db)

        return False
