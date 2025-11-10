# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from omni.kit.window.property.templates import SimplePropertyWidget
from omni.usd import get_context
from omni.timeline import get_timeline_interface
from pxr import Sdf
import omni.usd

from omni.physxpvd.scripts.property_widget.usd_helpers import (
    get_class_name
)

class PropertyWidgetOmniPvd(SimplePropertyWidget):
    name = "OmniPVD"
    _path = ""
    _prim_handlers = []
    _timeline_event_sub = None

    def __init__(self, prim_handlers):
        super().__init__(title=self.name, collapsed=False)
        self._prim_handlers = prim_handlers

    def _on_timeline_event(self, e):
        if e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_TICKED) or e.type == int(omni.timeline.TimelineEventType.CURRENT_TIME_CHANGED):
            current_time = e.payload["currentTime"]
            if current_time != self._current_time:
                self._current_time = current_time
                self.request_rebuild()            

    def on_startup(self):
        if self._timeline_event_sub is not None:
            return
        timeline = get_timeline_interface()
        timeline_events = timeline.get_timeline_event_stream()
        self._current_time = timeline.get_current_time()
        self._timeline_event_sub = timeline_events.create_subscription_to_pop(self._on_timeline_event)
    
    def on_shutdown(self):
        self._timeline_event_sub = None

    def build_items(self):
        #print("build_items property")
        stage = get_context().get_stage()
        if stage:
            if not Sdf.Path.IsValidPathString(self._path):
                return
            if self._path:
                prim = stage.GetPrimAtPath(self._path)
                if not prim:
                    return
                for prim_handler in self._prim_handlers:
                    if prim_handler(prim):
                        break

    def on_new_payload(self, payload, ignore_large_selection=False) -> bool:
        if not super().on_new_payload(payload):
            return False
        if not self._payload or len(self._payload) == 0:
            return False
        if not omni.usd.get_context():
            return False
        isOmniPvdPrimInSelection = False
        stage = omni.usd.get_context().get_stage()
        if stage:
            for path in payload:
                if path:
                    prim = stage.GetPrimAtPath(path)
                    if prim:
                        if prim.HasAttribute("omni:pvdi:class"):
                            if get_class_name(prim):
                                isOmniPvdPrimInSelection = True;
        if not isOmniPvdPrimInSelection:
            return False

        sdf_paths = payload.get_paths()
        self._path = str(sdf_paths[0]) if len(sdf_paths) > 0 else ""
        return True
