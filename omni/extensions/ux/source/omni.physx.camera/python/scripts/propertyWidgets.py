# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.ui
import omni.usd
import omni.kit.undo
from omni.kit.window.property.templates import SimplePropertyWidget
from pxr import Usd


PROPERTY_WIDGET_STYLE = {
    "Label": {
        "font_size": 14
    }
}


class PropertyWidgetCameraBase(SimplePropertyWidget):
    name = "physx_camera_base"

    def __init__(self, title: str, schema: Usd.SchemaBase, isAPISchema: bool,
        undoCommandRedrawList = None):
        super().__init__(title, False)
        self._schema = schema
        self._isAPISchema = isAPISchema
        self._prim = None
        self._primPath = None
        self._undoCommandRedrawList = undoCommandRedrawList  # list of undo command names that should trigger a redraw
        self._undo_change_subs = False
        self._no_redraw_on_next_command_match = False
        self._change_info_path_subscription = None

    # { PropertyWidget

    def clean(self):
        self.on_hide()

        super().clean()

    def on_new_payload(self, payload):
        if (self._prim is not None):
            self.on_hide()
        self._prim = None
        self._primPath = None

        if not super().on_new_payload(payload):
            return False

        stage = omni.usd.get_context().get_stage()
        if (stage):
            for usdPath in payload:
                prim = stage.GetPrimAtPath(usdPath)

                if not prim:
                    continue

                if (self._isAPISchema):
                    if (prim.HasAPI(self._schema)):
                        self.on_rebuild(prim)
                        return True
                else:
                    if (prim.IsA(self._schema)):
                        self.on_rebuild(prim)
                        return True

        return False

    # } PropertyWidget

    def is_valid(self):
        # the property widget system has multiple delays:
        # - 2 frames from selecting a prim to on_new_payload() being called (might be reduced to 1 in the future)
        # - another 2 frames from on_new_payload() to build_items() being called
        #
        # as a consequence, it is possible to get a build_items() call even though the property widget has been
        # invalidated in the meantime (through another on_new_payload() call before pending build_items() did run).
        # This method is a workaround to detect such a case.
        #
        return (self._prim is not None)

    # override in subclass as needed
    def on_hide(self):
        if (self._undo_change_subs):
            omni.kit.undo.unsubscribe_on_change(self._on_undo_change)
            self._undo_change_subs = False
        self._no_redraw_on_next_command_match = False

        if (self._change_info_path_subscription is not None):
            self._change_info_path_subscription = None

    # override in subclass as needed
    def on_rebuild(self, prim: Usd.Prim):
        self._prim = prim
        self._primPath = prim.GetPath().pathString
        if (self._undoCommandRedrawList is not None):
            if (not self._undo_change_subs):
                omni.kit.undo.subscribe_on_change(self._on_undo_change)
                self._undo_change_subs = True

    def _on_undo_change(self, cmds):
        # some commands do not change USD data but might still require a redraw of the property window
        # -> listen for those commands

        recreate = False
        for commandString in self._undoCommandRedrawList:

            if (commandString in cmds):
                if (self._no_redraw_on_next_command_match):
                    self._no_redraw_on_next_command_match = False
                    return

                recreate = True
                break

        if (recreate):
            rebuild_property_window()

    def no_redraw_on_next_command_match(self):
        # should not redraw the UI the next time a matching command is detected.
        # This is to avoid redrawing when issueing a command explicitly (vs. undo/redo)
        self._no_redraw_on_next_command_match = True

    def register_change_info_path(self, usdPath, callback):
        self._change_info_path_subscription = omni.usd.get_watcher().subscribe_to_change_info_path(
            usdPath, callback)


# request to rebuild property window on next draw, it's ok to call multiple times
def rebuild_property_window():
    omni.kit.window.property.get_window()._window.frame.rebuild()

