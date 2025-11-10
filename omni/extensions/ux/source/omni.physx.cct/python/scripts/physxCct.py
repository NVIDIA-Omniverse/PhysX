# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import carb
import carb.profiler
from carb.eventdispatcher import get_eventdispatcher
import omni.kit.usd
import omni.kit.utils
import omni.kit.app
import omni.usd
from omni import ui
from pxr import UsdGeom, UsdPhysics
from omni.physxcct import get_physx_cct_interface
from . import utils, externals
import omni.kit.viewport.utility
from omni.kit.viewport.utility.camera_state import ViewportCameraState


class PhysXCct(ui.Window):
    def __init__(self):
        super().__init__("PhysX Character Controller", ui.DockPreference.RIGHT_BOTTOM, width=480, height=480)
        self.deferred_dock_in("Property", ui.DockPolicy.CURRENT_WINDOW_IS_ACTIVE)

        self._timeline = omni.timeline.get_timeline_interface()
        self._update_sub = None
        self._cct = None
        self._gen_path = None

        events = omni.usd.get_context().get_stage_event_stream()
        self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
        self._build_ui()

    def on_shutdown(self):
        self._disable_cct()
        self._stage_event_sub = None
        self._release_ui()

    def _on_stage_event(self, event):
        if event.type == int(omni.usd.StageEventType.CLOSING):
            self._disable_cct()
        elif event.type == int(omni.usd.StageEventType.OPENED):
            self._camera_model.refresh_cameras()

    def _disable_cct(self):
        self._update_sub = None
        if self._cct:
            self._cct.disable()
            self._cct.shutdown()
            self._cct = None
        self._refresh_active_ui()

    def _on_update(self, e):
        if self._cct is None or not self._timeline.is_playing():
            return

        # FIXME: cleanup deleted capsule, remove after a notifier solution is done
        capsulePrim = omni.usd.get_context().get_stage().GetPrimAtPath(self._cct.path)
        if not capsulePrim.IsValid():
            self._disable_cct()

    class CameraItem(ui.AbstractItem):
        def __init__(self, model):
            super().__init__()
            self.model = model

    class CameraModel(ui.AbstractItemModel):
        def __init__(self, parent):
            super().__init__()

            self._index = ui.SimpleIntModel(0)
            self._index.add_value_changed_fn(self._index_changed)
            self._parent = parent

            self.refresh_cameras()

        @carb.profiler.profile
        def refresh_cameras(self):
            stage = omni.usd.get_context().get_stage()
            if stage:
                self.cameras = [PhysXCct.CameraItem(ui.SimpleStringModel(path.pathString)) for path in utils.gen_camera_paths(stage)]
            else:
                self.cameras = []

        def _index_changed(self, id):
            self._parent._on_camera_changed(self.get_selected_camera())
            self._item_changed(None)  # need to refresh manually

        def get_item_children(self, item):
            return self.cameras

        def get_item_value_model(self, item, column_id):
            if item is None:
                return self._index  # combobox expects index model on Item == None
            return item.model

        def get_selected_camera(self):
            return self.cameras[self._index.as_int].model.as_string

    def _build_ui(self):
        with self.frame:
            with ui.VStack():
                with ui.HStack(height=25):
                    self._active_label = ui.Label("", height=25)
                    ui.Button("Activate", width=100, height=25, tooltip="Activate CCT on a selected Capsule without a Rigid Body or activates on a spawned Capsule when no prim is selected").set_clicked_fn(self.on_activate_click)
                    self._disable_button = ui.Button("Disable", width=100, enabled=False, clicked_fn=self._disable_cct)
                with ui.CollapsableFrame("Settings", height=0):
                    with ui.VStack():
                        with ui.HStack(height=25):
                            ui.Label("Camera:", height=25, width=100)
                            self._camera_model = PhysXCct.CameraModel(self)
                            ui.ComboBox(self._camera_model, height=25)
                        with ui.HStack(height=25):
                            ui.Label("Gravity:", height=25, width=100)
                            self._gravity_model = ui.SimpleBoolModel(True)
                            ui.CheckBox(self._gravity_model, height=25)
                            self._gravity_model.add_value_changed_fn(self._on_gravity_changed)
                        with ui.HStack(height=25):
                            ui.Label("Speed:", height=25, width=100)
                            self._speed_model = ui.SimpleIntModel(500)
                            ui.IntDrag(self._speed_model, min=1, max=5000, step=1, height=25)
                            self._speed_model.add_value_changed_fn(self._on_speed_changed)
                        with ui.HStack(height=25):
                            ui.Label("Jump Speed:", height=25, width=100)
                            self._jump_speed_model = ui.SimpleIntModel(500)
                            ui.IntDrag(self._jump_speed_model, min=1, max=5000, step=1, height=25)
                            self._jump_speed_model.add_value_changed_fn(self._on_jump_speed_changed)

        self._refresh_active_ui()

    def _release_ui(self):
        self._active_label = None
        self._disable_button = None
        self._camera_model = None
        self._gravity_model = None
        self._speed_model = None
        self._jump_speed_model = None

    def _refresh_active_ui(self):
        path = self._cct.path if self._cct else "None"
        self._active_label.text = "Active CCT: " + str(path)
        self._disable_button.enabled = path != "None"

    def _on_camera_changed(self, camera_path):
        if self._cct:
            get_physx_cct_interface().enable_first_person(self._cct.path, camera_path)

    def _on_gravity_changed(self, model):
        if self._cct:
            cctiface = get_physx_cct_interface()
            if model.as_bool:
                cctiface.enable_gravity(self._cct.path)
            else:
                cctiface.disable_gravity(self._cct.path)

    def _on_speed_changed(self, model):
        if self._cct:
            self._cct.control_state.speed = model.as_int

    def _on_jump_speed_changed(self, model):
        if self._cct:
            self._cct.control_state.jump_speed = model.as_int

    def _get_camera_pos(self):
        camera = ViewportCameraState()
        return camera.position_world

    def _move_capsule_to_camera_pos(self, stage, path):
        pos = self._get_camera_pos()
        geom = UsdGeom.Capsule.Get(stage, path)
        for op in geom.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(pos)

    def _spawn_cct_capsule(self, stage, path):
        cam_pos = self._get_camera_pos()
        capsule = utils.spawn_capsule(stage, path, cam_pos)
        capsule.CreatePurposeAttr().Set("guide")

    def _activate_as_cct(self, stage, path):
        self._disable_cct()

        self._cct = utils.CharacterController(path, self._camera_model.get_selected_camera(), self._gravity_model.as_bool)
        self._cct.activate(stage)
        self._cct.setup_controls(self._speed_model.as_int)
        self._update_sub = get_eventdispatcher().observe_event(
            event_name=omni.kit.app.GLOBAL_EVENT_UPDATE,
            on_event=self._on_update,
            observer_name="omni.physxcct",
        )
        self._refresh_active_ui()

    def on_activate_click(self):
        stage = omni.usd.get_context().get_stage()
        paths = omni.usd.get_context().get_selection().get_selected_prim_paths()
        if len(paths) == 0:
            # check if the generated capsule was not deleted
            if self._gen_path is not None and not stage.GetPrimAtPath(self._gen_path).IsValid():
                self._gen_path = None
            # generate free path and try spawning a cct enabled capsule
            if self._gen_path is not None:
                self._move_capsule_to_camera_pos(stage, self._gen_path)
            else:
                self._gen_path = omni.usd.get_stage_next_free_path(stage, "/temp/capsule_cct", True)
                self._spawn_cct_capsule(stage, self._gen_path)
                self._activate_as_cct(stage, self._gen_path)
        else:
            # find first capsule in selection and activate it as cct
            for path in paths:
                prim = stage.GetPrimAtPath(path)
                if prim.IsA(UsdGeom.Capsule):
                    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                        prompt = externals.Prompt("Error", f"Cannot activate CCT on a Capsule with a Rigid Body at '{path}'! Please remove its Rigid Body and activate again or choose a different Capsule.", modal=True)
                        prompt.show()
                    else:
                        self._activate_as_cct(stage, path)
                    return
            prompt = externals.Prompt("Error", "No Capsule prim found among selected prims!", modal=True)
            prompt.show()
