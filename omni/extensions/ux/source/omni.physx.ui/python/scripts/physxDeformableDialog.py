# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, Usd
from omni.kit.property.usd.widgets import ICON_PATH
from omni.kit.property.usd.relationship import SelectionWatch
from omni.kit.widget.stage import StageWidget
from pathlib import Path
import omni.ui as ui
import omni.usd
from functools import partial
from enum import Enum, auto
import weakref

class Styles:
    DISABLED_COLOR = 0xFF606060

def create_wrapped_label(name, tooltipText, height, width, tooltipWidth, style):
    label = ui.Label(name, word_wrap = True, height=height, width = width, style=style)

    # tooltip could be set in omni.ui.Label directly but the text will have no padding
    # which does look somewhat crappy. Thus, creating custom tooltip widget.
    if (tooltipText):
        def create_tooltip():
            ui.Label(tooltipText, word_wrap = True, width = tooltipWidth,
                style = {"margin": 5})  # "padding" did not work

        label.set_tooltip_fn(create_tooltip)

    return label


class PrimPicker:
    def __init__(self, stage, on_select_fn, filter_type_list, filter_lambda=None, **kwargs):
        self._weak_stage = weakref.ref(stage)
        self._on_select_fn = on_select_fn
        self._selected_path = Sdf.Path()
        self._window_title = "Select a " + " or ".join([x.__name__ for x in filter_type_list])
        self._window_title = kwargs.pop("window_title", self._window_title)
        self._window_width = kwargs.pop("window_width", 400)
        self._window_height = kwargs.pop("window_height", 400)

        def on_window_visibility_changed(visible):
            if not visible:
                self._stage_widget.open_stage(None)
            else:
                # Only attach the stage when picker is open. Otherwise the Tf notice listener in StageWidget kills perf
                self._stage_widget.open_stage(self._weak_stage())

        self._window = ui.Window(
            self._window_title,
            width=self._window_width,
            height=self._window_height,
            visible=False,
            flags=ui.WINDOW_FLAGS_MODAL,
            visibility_changed_fn=on_window_visibility_changed,
        )
        with self._window.frame:
            with ui.VStack():
                with ui.Frame():
                    self._stage_widget = StageWidget(None, columns_enabled=["Type"])
                    self._selection_watch = SelectionWatch(
                        stage=stage,
                        on_selection_changed_fn=self._on_selection_changed,
                        filter_type_list=filter_type_list,
                        filter_lambda=filter_lambda,
                    )
                    self._stage_widget.set_selection_watch(self._selection_watch)

                    if len(filter_type_list):
                        self._stage_widget._filter_by_type(filter_type_list, True)
                    if filter_lambda is not None:
                        self._stage_widget._filter_by_lambda({"relpicker_filter": filter_lambda}, True)

                def on_select(weak_self):
                    weak_self = weak_self()
                    if not weak_self:
                        return
                    weak_self._window.visible = False
                    self._on_select_fn(self._selected_path)

                with ui.VStack(
                    height=0, style={"Button.Label:disabled": {"color": 0xFF606060}}
                ):  # TODO consolidate all styles
                    self._label = ui.Label("Selected Path:\n\tNone")
                    self._button = ui.Button(
                        "Select", height=10, clicked_fn=partial(on_select, weak_self=weakref.ref(self)), enabled=False
                    )

    def clean(self):
        self._window.set_visibility_changed_fn(None)
        self._window = None
        self._selection_watch = None
        self._stage_widget.open_stage(None)
        self._stage_widget.destroy()
        self._stage_widget = None

    def show(self):
        self._selection_watch.reset(1)
        self._window.visible = True

    def _on_selection_changed(self, paths):
        if len(paths) > 0:
            self._selected_path = paths[0]
        else:
            self._selected_path = None
        if self._button:
            self._button.enabled = self._selected_path is not None
        if self._label:
            if self._selected_path is not None:
                label_text = f"Selected Path: {self._selected_path}"
                self._label.text = label_text
            else:
                self._label.text = ""

class DeformableType(Enum):
    VOLUME = auto()
    SURFACE = auto()

class DeformableDialog:

    def __init__(self, root_path: Sdf.Path, deformable_type: DeformableType):
        self._usd_context = omni.usd.get_context()

        self._type = deformable_type
        self._usd_sim_type = UsdGeom.TetMesh if deformable_type == DeformableType.VOLUME else UsdGeom.Mesh

        self._root_path = root_path
        self._sim_hex_enabled = False
        self._cooking_src_simp_enabled = (deformable_type == DeformableType.VOLUME)
        self._simulation_path = Sdf.Path(self._get_default_simulation_path())
        self._collision_path = Sdf.Path(self._get_default_collision_path())
        self._cooking_src_path = Sdf.Path(self._get_default_cooking_src_path())

        self._build_dialog_window()


    def _get_subtree_meshes(self):
        mesh_paths = []
        stage = self._usd_context.get_stage()
        if stage:
            root_prim = stage.GetPrimAtPath(self._root_path)
            prim_range = Usd.PrimRange(root_prim, Usd.PrimAllPrimsPredicate)
            for prim in prim_range:
                # skip the root itself for the special transform check
                if prim != root_prim:
                    xformable = UsdGeom.Xformable(prim)
                    if xformable and xformable.GetResetXformStack():
                        # prune the subtree under this prim and move on
                        range.PruneChildren()
                        continue
                if prim.IsA(UsdGeom.Mesh):
                    mesh_paths.append(prim.GetPath())
        return mesh_paths

    def _get_child_meshes(self, api_type):
        mesh_paths = []
        stage = self._usd_context.get_stage()
        if stage:
            root_prim = stage.GetPrimAtPath(self._root_path)
            children = root_prim.GetChildren()
            for prim in children:
                xformable = UsdGeom.Xformable(prim)
                if xformable and xformable.GetResetXformStack():
                    continue
                if prim.HasAPI(api_type):
                    mesh_paths.append(prim.GetPath())
        return mesh_paths

    @staticmethod
    def _is_in_subtree(prim, root_prim):
        while prim and prim != root_prim:
            prim = prim.GetParent()
        return prim == root_prim

    def _make_subtree_checker(self, root_prim):
        return lambda prim: DeformableDialog._is_in_subtree(prim, root_prim)

    def _get_default_cooking_src_path(self):
        mesh_paths = self._get_subtree_meshes()
        if len(mesh_paths) > 0:
            return mesh_paths[0]
        else:
            return Sdf.Path()

    def _get_src_path_button_text(self):
        if self._cooking_src_path == Sdf.Path():
            return "Pick Source Mesh"
        else:
            return str(self._cooking_src_path)

    def _on_cooking_src_path_changed(self, path: Sdf.Path):
        self._cooking_src_path = Sdf.Path(path)
        self._cooking_src_path_button.text = self._get_src_path_button_text()
        self._update_valid()


    def _get_default_simulation_path(self):
        stage = self._usd_context.get_stage()
        if stage:
            sim_api_name = "OmniPhysicsVolumeDeformableSimAPI" if self._type == DeformableType.VOLUME else "OmniPhysicsSurfaceDeformableSimAPI"
            mesh_paths = self._get_child_meshes(Usd.SchemaRegistry().GetTypeFromSchemaTypeName(sim_api_name))
            if len(mesh_paths) > 0:
                return Sdf.Path(mesh_paths[0])
            else:
                root_prim = stage.GetPrimAtPath(self._root_path)
                sim_path = self._root_path.AppendChild("simulation_mesh")
                return Sdf.Path(omni.usd.get_stage_next_free_path(stage, sim_path, False))
        return Sdf.Path()

    def _get_simulation_path_button_text(self):
        if self._simulation_path == Sdf.Path():
            return ""
        else:
            return str(self._simulation_path)

    def _on_simulation_path_changed(self, path: Sdf.Path):
        self._simulation_path = Sdf.Path(path)
        self._simulation_path_button.text = self._get_simulation_path_button_text()
        self._on_collision_path_changed(self._get_default_collision_path())
        self._update_valid()



    def _get_default_collision_path(self):
        stage = self._usd_context.get_stage()
        if stage:
            if self._sim_hex_enabled:
                mesh_paths = self._get_child_meshes(UsdPhysics.CollisionAPI)
                if len(mesh_paths) > 0:
                    return Sdf.Path(mesh_paths[0])
                else:
                    root_prim = stage.GetPrimAtPath(self._root_path)
                    coll_path = self._root_path.AppendChild("collision_mesh")
                    return Sdf.Path(omni.usd.get_stage_next_free_path(stage, coll_path, False))
            else:
                return Sdf.Path(self._simulation_path)
        return Sdf.Path()

    def _get_collision_path_button_text(self):
        if self._collision_path == Sdf.Path():
            return ""
        else:
            return str(self._collision_path)

    def _on_collision_path_changed(self, path: Sdf.Path):
        self._collision_path = Sdf.Path(path)
        self._collision_path_button.text = self._get_collision_path_button_text()
        self._update_valid()


    def _on_sim_hex_enabled_changed(self, value):
        changed = self._sim_hex_enabled != value
        if changed:
            self._sim_hex_enabled = value
            self._on_collision_path_changed(self._get_default_collision_path())
            #self._update_valid() already checked in _on_collision_path_changed
        self._collision_path_button.enabled = self._sim_hex_enabled

    def _on_cooking_src_simp_enabled_changed(self, value):
        self._cooking_src_simp_enabled = value


    def _check_valid(self):
        stage = self._usd_context.get_stage()
        if not stage:
            return False

        #cooking source valid
        src_prim = stage.GetPrimAtPath(self._cooking_src_path)
        if not src_prim or not src_prim.IsA(UsdGeom.Mesh):
            return False

        #check simulation path
        if self._simulation_path == Sdf.Path():
            return False

        if not self._simulation_path.HasPrefix(self._root_path) or self._simulation_path == self._root_path:
            return False

        sim_prim = stage.GetPrimAtPath(self._simulation_path)
        if sim_prim and not sim_prim.IsA(UsdGeom.TetMesh):
            return False


        #check collision path
        if self._collision_path != self._simulation_path:
            if self._collision_path == Sdf.Path():
                return False

            if not self._collision_path.HasPrefix(self._root_path) or self._collision_path == self._root_path:
                return False

            coll_prim = stage.GetPrimAtPath(self._collision_path)
            if coll_prim and not coll_prim.IsA(UsdGeom.TetMesh):
                return False

        #check consistency with _sim_hex_enabled
        if self._sim_hex_enabled != (self._collision_path != self._simulation_path):
            return False

        return True

    def _update_valid(self):
        valid = self._check_valid()
        if self._create_button is not None:
            self._create_button.enabled = valid

    def _on_window_closed(self, visible):
        self._root_path = None
        self._usd_context = None

    def _hide(self):
        self._window.visible = False

    def _on_cancel(self):
        self._hide()

    def _on_create(self):
        stage = self._usd_context.get_stage()
        omni.kit.undo.begin_group()
        if self._type == DeformableType.VOLUME:
            omni.kit.commands.execute("CreateAutoVolumeDeformableHierarchy",
                root_prim_path = self._root_path,
                simulation_tetmesh_path = self._simulation_path,
                collision_tetmesh_path = self._collision_path,
                cooking_src_mesh_path = self._cooking_src_path,
                simulation_hex_mesh_enabled = self._sim_hex_enabled,
                cooking_src_simplification_enabled = self._cooking_src_simp_enabled
            )
        else:
            omni.kit.commands.execute("CreateAutoSurfaceDeformableHierarchy",
                root_prim_path = self._root_path,
                simulation_mesh_path = self._simulation_path,
                cooking_src_mesh_path = self._cooking_src_path,
                cooking_src_simplification_enabled = self._cooking_src_simp_enabled
            )
        omni.kit.undo.end_group()
        self._hide()


    def _build_dialog_window(self):
        stage = self._usd_context.get_stage()
        if not stage:
            return

        root_prim = stage.GetPrimAtPath(self._root_path)
        window_title = "Create Volume Deformable Body" if self._type == DeformableType.VOLUME else "Create Surface Deformable Body"

        self._window = ui.Window(window_title, dockPreference = omni.ui.DockPreference.DISABLED, width=800, height=0, padding_x = 15, padding_y = 15)
        self._window.set_visibility_changed_fn(self._on_window_closed)
        self._window.flags = (ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_SCROLLBAR)

        with self._window.frame:
            with ui.VStack(spacing=10, height=0, style={"Button.Label:disabled": {"color": Styles.DISABLED_COLOR}}):

                labelWidth = 200
                with ui.HStack(spacing=20, style={"Button.Label": {"alignment": ui.Alignment.LEFT_CENTER}}):
                    create_wrapped_label("Source Mesh",
                                        tooltipText = "Souce mesh used for simulation and collision mesh generation, may be outside deformable sub-tree.",
                                        height=20, width=labelWidth, tooltipWidth = 300, style={"margin_width": 5, "font_size": 16}
                    )
                    picker = PrimPicker(stage, self._on_cooking_src_path_changed, [UsdGeom.Mesh], filter_lambda=None)
                    self._cooking_src_path_button = omni.ui.Button(text=self._get_src_path_button_text(), clicked_fn=picker.show, height=20)

                with ui.HStack(spacing=20, style={"Button.Label": {"alignment": ui.Alignment.LEFT_CENTER}}):
                    create_wrapped_label("Simulation Mesh",
                                        tooltipText = "Path to simulation mesh being generated.",
                                        height=20, width=labelWidth, tooltipWidth = 300, style={"margin_width": 5, "font_size": 16}
                    )
                    picker = PrimPicker(stage, self._on_simulation_path_changed, [UsdGeom.TetMesh], filter_lambda=self._make_subtree_checker(root_prim))
                    self._simulation_path_button = omni.ui.Button(text=self._get_simulation_path_button_text(), clicked_fn=picker.show, height=20)

                if self._type == DeformableType.VOLUME:
                    with ui.HStack(spacing=20, style={"Button.Label": {"alignment": ui.Alignment.LEFT_CENTER}}):
                        create_wrapped_label("Collision Mesh",
                                            tooltipText = "Path to collision mesh being generated.",
                                            height=20, width=labelWidth, tooltipWidth = 300, style={"margin_width": 5, "font_size": 16}
                        )
                        picker = PrimPicker(stage, self._on_collision_path_changed, [UsdGeom.TetMesh], filter_lambda=self._make_subtree_checker(root_prim))
                        self._collision_path_button = omni.ui.Button(text=self._get_collision_path_button_text(), clicked_fn=picker.show, height=20)

                ui.Spacer()

                with ui.HStack(spacing=20):
                    create_wrapped_label("Source Mesh Simplification", tooltipText = "Enable source mesh simplification.",
                                        height=20, width=labelWidth, tooltipWidth = 300, style={"margin_width": 5, "font_size": 16}
                    )
                    model = ui.CheckBox().model
                    model.add_value_changed_fn(lambda value: self._on_cooking_src_simp_enabled_changed(value.as_bool))
                    model.set_value(self._cooking_src_simp_enabled)
                    self._on_cooking_src_simp_enabled_changed(self._cooking_src_simp_enabled)

                if self._type == DeformableType.VOLUME:
                    with ui.HStack(spacing=20):
                        create_wrapped_label("Hexahedral Simulation Mesh", tooltipText = "Generate separate hexahedral simulation mesh.",
                                            height=20, width=labelWidth, tooltipWidth = 300, style={"margin_width": 5, "font_size": 16}
                        )
                        model = ui.CheckBox().model
                        model.add_value_changed_fn(lambda value: self._on_sim_hex_enabled_changed(value.as_bool))
                        model.set_value(self._sim_hex_enabled)
                        self._on_sim_hex_enabled_changed(self._sim_hex_enabled)

                ui.Spacer()

                with ui.HStack(spacing=20):
                    self._create_button = ui.Button("Create", width=100, height=30)
                    self._create_button.set_clicked_fn(self._on_create)
                    self._update_valid()

                    ui.Button("Cancel", width=100, height=30).set_clicked_fn(self._on_cancel)
