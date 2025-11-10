# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
from pxr import UsdGeom, Gf, Sdf, UsdPhysics, PhysxSchema, Usd
from omni.kit.property.usd.widgets import ICON_PATH
from pathlib import Path
import omni.ui as ui
import omni.usd
from functools import partial

class Prompt:
    def __init__(
        self,
        title,
        text,
        ok_button_text="OK",
        cancel_button_text=None,
        middle_button_text=None,
        ok_button_fn=None,
        cancel_button_fn=None,
        middle_button_fn=None,
        modal=False,
    ):
        self._title = title
        self._text = text
        self._cancel_button_text = cancel_button_text
        self._cancel_button_fn = cancel_button_fn
        self._ok_button_fn = ok_button_fn
        self._ok_button_text = ok_button_text
        self._middle_button_text = middle_button_text
        self._middle_button_fn = middle_button_fn
        self._modal = modal
        self._build_ui()

    def __del__(self):
        self._cancel_button_fn = None
        self._ok_button_fn = None

    def __enter__(self):
        self._window.show()
        return self

    def __exit__(self, type, value, trace):
        self._window.hide()

    def show(self):
        self._window.visible = True

    def hide(self):
        self._window.visible = False

    def is_visible(self):
        return self._window.visible

    def set_text(self, text):
        self._text_label.text = text

    def set_confirm_fn(self, on_ok_button_clicked):
        self._ok_button_fn = on_ok_button_clicked

    def set_cancel_fn(self, on_cancel_button_clicked):
        self._cancel_button_fn = on_cancel_button_clicked

    def set_middle_button_fn(self, on_middle_button_clicked):
        self._middle_button_fn = on_middle_button_clicked

    def _on_ok_button_fn(self):
        self.hide()
        if self._ok_button_fn:
            self._ok_button_fn()

    def _on_cancel_button_fn(self):
        self.hide()
        if self._cancel_button_fn:
            self._cancel_button_fn()

    def _on_middle_button_fn(self):
        self.hide()
        if self._middle_button_fn:
            self._middle_button_fn()

    def _build_ui(self):
        self._window = ui.Window(
            self._title, visible=False, height=0, dockPreference=ui.DockPreference.DISABLED
        )
        self._window.flags = (
            ui.WINDOW_FLAGS_NO_COLLAPSE
            | ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_NO_RESIZE
            | ui.WINDOW_FLAGS_NO_MOVE
        )

        if self._modal:
            self._window.flags = self._window.flags | ui.WINDOW_FLAGS_MODAL

        with self._window.frame:
            with ui.VStack(height=0):
                ui.Spacer(width=0, height=10)
                with ui.HStack(height=0):
                    ui.Spacer()
                    self._text_label = ui.Label(self._text, word_wrap=True, width=self._window.width - 80, height=0)
                    ui.Spacer()
                ui.Spacer(width=0, height=10)
                with ui.HStack(height=0):
                    ui.Spacer(height=0)
                    if self._ok_button_text:
                        ok_button = ui.Button(self._ok_button_text, width=60, height=0)
                        ok_button.set_clicked_fn(self._on_ok_button_fn)
                    if self._middle_button_text:
                        middle_button = ui.Button(self._middle_button_text, width=60, height=0)
                        middle_button.set_clicked_fn(self._on_middle_button_fn)
                    if self._cancel_button_text:
                        cancel_button = ui.Button(self._cancel_button_text, width=60, height=0)
                        cancel_button.set_clicked_fn(self._on_cancel_button_fn)
                    ui.Spacer(height=0)
                ui.Spacer(width=0, height=10)

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

def extend_bbox(bbox: Gf.BBox3d, scale: float)-> Gf.BBox3d:
    bbox_range = bbox.GetRange()

    if scale < 0.0 or bbox_range.IsEmpty():
        return Gf.BBox3d(bbox)

    center = bbox_range.GetMidpoint()
    extent = bbox_range.GetSize() * scale
    bbox_range_extended = Gf.Range3d(center - 0.5 * extent, center + 0.5 * extent)
    return Gf.BBox3d(bbox_range_extended, bbox.GetMatrix())

def is_overlap(stage, deformable_path, actor_path, overlap_tolerance):
    deformable_prim = stage.GetPrimAtPath(deformable_path)
    actor_prim = stage.GetPrimAtPath(actor_path)

    deformable_purpose = UsdGeom.Imageable(deformable_prim).GetPurposeAttr().Get()
    actor_purpose = UsdGeom.Imageable(actor_prim).GetPurposeAttr().Get()

    bbox0 = UsdGeom.Xformable(deformable_prim).ComputeWorldBound(Usd.TimeCode.Default(), purpose1=deformable_purpose)
    bbox1 = UsdGeom.Xformable(actor_prim).ComputeWorldBound(Usd.TimeCode.Default(), purpose1=actor_purpose)
    bbox0 = extend_bbox(bbox0, 1.0 + overlap_tolerance)
    bbox1 = extend_bbox(bbox1, 1.0 + overlap_tolerance)
    range0 = bbox0.ComputeAlignedRange()
    range1 = bbox1.ComputeAlignedRange()

    # return true for xformables without extent
    if range0.IsEmpty() or range1.IsEmpty():
        return True

    return not range0.IntersectWith(range1).IsEmpty()

def is_pair_exist(path0, path1, actor1_paths):
    if path0 in actor1_paths:
        if path1 in actor1_paths[path0]:
            return True

    if path1 in actor1_paths:
        if path0 in actor1_paths[path1]:
            return True

    return False

# DEPRECATED
def is_particle_cloth(prim):
    return True if PhysxSchema.PhysxParticleClothAPI(prim) else False

def is_deformable_body(prim):
    return True if PhysxSchema.PhysxDeformableBodyAPI(prim) else False

def is_deformable_surface(prim):
    return True if PhysxSchema.PhysxDeformableSurfaceAPI(prim) else False

def is_rigid(prim):
    return True if (UsdPhysics.RigidBodyAPI(prim) or UsdPhysics.CollisionAPI(prim)) else False

def is_pair_valid_deprecated(stage, path0, path1):
    prim0 = stage.GetPrimAtPath(path0)
    prim1 = stage.GetPrimAtPath(path1)
    result = True

    if is_rigid(prim0) and is_rigid(prim1):
        result = False

    if is_particle_cloth(prim0) and is_particle_cloth(prim1):
        result = False

    if is_deformable_surface(prim0) and is_deformable_surface(prim1):
        result = False

    if is_particle_cloth(prim0) and is_deformable_surface(prim1):
        result = False

    if is_deformable_surface(prim0) and is_particle_cloth(prim1):
        result = False

    return result
#~DEPRECATED


def is_pair_valid(stage, deformable_path, actor_path):
    deformable_prim = stage.GetPrimAtPath(deformable_path)
    actor_prim = stage.GetPrimAtPath(actor_path)
    result = True

    if not deformable_prim.IsA(UsdGeom.Imageable) or not actor_prim.IsA(UsdGeom.Imageable):
        result = False
    elif not deformable_prim.HasAPI("OmniPhysicsDeformableBodyAPI"):
        result = False

    return result

def compute_overlapped_paths_pairs_from_two_actor_groups(stage, pair_dict, deformables, actors, overlap_tolerance):

    for deformable_path in deformables:
        for actor_path in actors:
            if deformable_path != actor_path and not is_pair_exist(deformable_path, actor_path, pair_dict) and is_pair_valid(stage, deformable_path, actor_path) and is_overlap(stage, deformable_path, actor_path, overlap_tolerance):
                if deformable_path not in pair_dict:
                    pair_dict[deformable_path] = list()
                pair_dict[deformable_path].append(actor_path)

# DEPRECATED
def extend_range_deprecated(bbox, percentage):
    bound_range = bbox.ComputeAlignedBox()
    center = bound_range.GetMidpoint()
    extent = bound_range.GetSize() * (1 + percentage)
    return Gf.Range3d(center - 0.5 * extent, center + 0.5 * extent)

def is_overlap_deprecated(stage, path0, path1, overlap_tolerance):
    prim0 = stage.GetPrimAtPath(path0)
    prim1 = stage.GetPrimAtPath(path1)
    bbox0 = UsdGeom.Xformable(prim0).ComputeWorldBound(Usd.TimeCode.Default(), purpose1=UsdGeom.Imageable(prim0).GetPurposeAttr().Get())
    bbox1 = UsdGeom.Xformable(prim1).ComputeWorldBound(Usd.TimeCode.Default(), purpose1=UsdGeom.Imageable(prim1).GetPurposeAttr().Get())

    range0 = extend_range_deprecated(bbox0, overlap_tolerance)
    range1 = extend_range_deprecated(bbox1, overlap_tolerance)

    for i in range(3):
        if range0.GetMax()[i] < range1.GetMin()[i] or range0.GetMin()[i] > range1.GetMax()[i]:
            return False
    return True     

def compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, actor0s, actor1s, overlap_tolerance):
    for path0 in actor0s:
        for path1 in actor1s:
            if path0 != path1 and not is_pair_exist(path0, path1, pair_dict) and is_pair_valid_deprecated(stage, path0, path1) and is_overlap_deprecated(stage, path0, path1, overlap_tolerance):
                if path0 not in pair_dict:
                    pair_dict[path0] = list()
                pair_dict[path0].append(path1)

def compute_overlapped_paths_pairs_deprecated(stage, particlecloths, deformablebodies, deformablesurfaces, rigids, overlap_tolerance):
    # Compute AABB overlap to determine potential actor pairs
    pair_dict = dict()
    compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, particlecloths, deformablebodies, overlap_tolerance)
    compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, particlecloths, rigids, overlap_tolerance)
    compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, deformablebodies, deformablebodies, overlap_tolerance)
    compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, deformablebodies, deformablesurfaces, overlap_tolerance)
    compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, deformablebodies, rigids, overlap_tolerance)
    compute_overlapped_paths_pairs_from_two_actor_groups_deprecated(stage, pair_dict, deformablesurfaces, rigids, overlap_tolerance)

    return pair_dict
#~DEPRECATED


def compute_overlapped_paths_pairs(stage, deformables, xformables, overlap_tolerance):
    # Compute AABB overlap to determine potential actor pairs
    pair_dict = dict()
    compute_overlapped_paths_pairs_from_two_actor_groups(stage, pair_dict, deformables, deformables, overlap_tolerance)
    compute_overlapped_paths_pairs_from_two_actor_groups(stage, pair_dict, deformables, xformables, overlap_tolerance)

    return pair_dict


class PairItem(ui.AbstractItem):
    def __init__(self, path0, path1):
        super().__init__()
        self.actor0_path_model = ui.SimpleStringModel(path0)
        self.actor1_path_model = ui.SimpleStringModel(path1)

    def __repr__(self):
        return f'"{self.actor0_path_model.as_string} {self.actor1_path_model.as_string}"'
        
class PairModel(ui.AbstractItemModel):
    def __init__(self, *args):
        super().__init__()
        self._children = [PairItem(*t) for t in args]

    def get_item_children(self, item):
        if item is not None:
            return []

        return self._children

    def get_item_value_model_count(self, item):
        return 2

    def get_item_value_model(self, item, column_id):
        return item.actor1_path_model if column_id == 1 else item.actor0_path_model

    def my_item_changed(self, *items):
        self._children.clear()
        self._item_changed(None)
        self._children = [PairItem(*t) for t in items]


class AttachmentsDialog:

    def __init__(self, deformables=None, xformables=None, particlecloths=None, deformablebodies=None, deformablesurfaces=None, rigids=None):
        self._deformables = deformables
        self._xformables = xformables
        
        # DEPRECATED
        self._particlecloths_deprecated = particlecloths
        self._deformablebodies_deprecated = deformablebodies
        self._deformablesurfaces_deprecated = deformablesurfaces
        self._rigids_deprecated = rigids

        self._deprecated = self._deformables is None and self._xformables is None

        self._usd_context = omni.usd.get_context()
        stage = self._usd_context.get_stage()

        self._overlap_tolerance = 0.01
        
        if self._deprecated:
            self._pair_dict = compute_overlapped_paths_pairs_deprecated(stage, self._particlecloths_deprecated, self._deformablebodies_deprecated, self._deformablesurfaces_deprecated, self._rigids_deprecated, self._overlap_tolerance)
        else:
        	self._pair_dict = compute_overlapped_paths_pairs(stage, self._deformables, self._xformables, self._overlap_tolerance)

        self._sorted_pairs = self.sort_pairs_by_islands(self._pair_dict)
        self._tree_views = dict()
        self._selected_items_dict = dict()
        self._display_pairs_models = dict()
        self._display_pairs = list() 
        for pairs in self._sorted_pairs.values():
            pairs = self._get_display_pairs(pairs)
            self._display_pairs.append(pairs)
        self._is_num_tree_views_changed = False

        # This is a workaround to solve the error "Tried to call pure virtual function "AbstractItemModel::get_item_children"
        # We need to keep the reference to the models somewhere rather than destroying them
        self._workaround_set = set()        

        self._style = {
            "TreeView": {"background_color": 0xFF444444, "background_selected_color": 0x663A3A3A},
            "TreeView:selected": {"background_color": 0x66FFFFFF},
            "TreeView.Item": {
                "margin": 3,
                "color": 0xFFCCCCCC,
                },
            "TreeView.Item:selected": {"color": 0xFFCCCCCC},
            "TreeView.Header": {"background_color": 0xFF000000},
            "ScrollingFrame": {"background_color": 0xFF444444},
        }

        self._build_dialog_window()

    def _on_window_closed(self, visible):
        self._overlap_tolerance = 0.01
        self._selected_items_dict = dict()
        self._is_num_tree_views_changed = False
        self._workaround_set = None

    @staticmethod
    def find_islands(overlapped_path_dict):
        visited = set()
        islands = []

        def dfs(node, island):
            # This function performs a Depth First Search starting from the given node.
            visited.add(node)
            island.append(node)
            if node in overlapped_path_dict:
                for neighbor in overlapped_path_dict[node]:
                    if neighbor not in visited:
                        dfs(neighbor, island)

        for node in overlapped_path_dict.keys():
            if node not in visited:
                island = []
                dfs(node, island)
                islands.append(island)
                
        return islands

    @staticmethod
    def sort_pairs_by_islands(overlapped_path_dict):
        sorted_pairs = []
        islands = AttachmentsDialog.find_islands(overlapped_path_dict)
        island_dict = {path: idx for idx, island in enumerate(islands) for path in island}
        for key in overlapped_path_dict:
            for value in overlapped_path_dict[key]:
                if value in island_dict and key in island_dict:
                    sorted_pairs.append((min(island_dict[value], island_dict[key]), key, value))
        sorted_pairs.sort(key=lambda x: (x[0], x[1], x[2]))

        result = {}
        current_island = None
        for idx, key, value in sorted_pairs:
            if idx != current_island:
                current_island = idx
                result[current_island] = []
            result[current_island].append((key, value))

        return result

    def _get_display_pairs(self, sorted_result):
        return [(str(key), str(value)) for key, value in sorted_result]

    def _update_display_pairs(self):
        self._display_pairs = list()
        for island_id, pairs in self._sorted_pairs.items():
            display_pairs = self._get_display_pairs(pairs)
            self._display_pairs.append(display_pairs)
            self._display_pairs_models[island_id].my_item_changed(*display_pairs) 

    def _create_pair_models(self):
        self._display_pairs_models = dict()
        for island_id, pairs in self._sorted_pairs.items():
            display_pairs = self._get_display_pairs(pairs)        
            model = PairModel(*display_pairs)
            self._display_pairs_models[island_id]=model           

    def _reset_selected_items(self):
        self._selected_items_dict = dict()
        self._usd_context.get_selection().set_selected_prim_paths([], True)

    def _on_tolerance_changed(self, value):
        self._overlap_tolerance = value.as_float
        stage = self._usd_context.get_stage()
        if self._deprecated:
            self._pair_dict = compute_overlapped_paths_pairs_deprecated(stage, self._particlecloths, self._deformablebodies, self._deformablesurfaces, self._rigids, self._overlap_tolerance)
        else:
            self._pair_dict = compute_overlapped_paths_pairs(stage, self._deformables, self._xformables, self._overlap_tolerance)
        
        self._sorted_pairs = self.sort_pairs_by_islands(self._pair_dict)

        if len(self._sorted_pairs) != len(self._tree_views):
            self._is_num_tree_views_changed = True

        if self._is_num_tree_views_changed:
            self._create_pair_models()
        
        self._update_display_pairs()
        self._reset_selected_items()

        # Do a refresh if treeviews get changed
        if self._is_num_tree_views_changed:
            self._scrolling_frame.rebuild()
            self._is_num_tree_views_changed = False

    def _hide(self):
        self._window.visible = False

    def _update_keys(self, empty_ids, old_dict):
        new_dict = dict()
        for key, value in old_dict.items():
            new_key = key - len([id for id in empty_ids if id < key])
            new_dict[new_key] = value

        return new_dict

    def _on_remove_clicked(self):
        empty_ids = list()
        for island_id, pairs in self._sorted_pairs.items():
            tree_view = self._tree_views[island_id]
            selected_items = list()
            for item in tree_view.selection:
                selected_items.append((item.actor0_path_model.as_string, item.actor1_path_model.as_string))
            
            self._sorted_pairs[island_id] = [pair for pair in pairs if pair not in selected_items]
            if not self._sorted_pairs[island_id]:
                empty_ids.append(island_id)

        for island_id in empty_ids:
            del self._sorted_pairs[island_id]

            model = self._display_pairs_models[island_id]
            self._workaround_set.add(model)

            del self._display_pairs_models[island_id]
            del self._tree_views[island_id]

        # Update keys accordingly if islands get removed
        if empty_ids:
            self._sorted_pairs = self._update_keys(empty_ids, self._sorted_pairs)
            self._display_pairs_models = self._update_keys(empty_ids, self._display_pairs_models)
            self._tree_views = self._update_keys(empty_ids, self._tree_views)

        self._update_display_pairs()
        self._reset_selected_items()

        # Do a refresh if treeviews get removed 
        if empty_ids:
            self._scrolling_frame.rebuild()

    def _on_remove_with_prompt(self):
        prompt = Prompt(
            "Remove Selected?",
            "Are you sure you want to remove the selected path pairs?",
            "Yes",
            "No",
            ok_button_fn=self._on_remove_clicked,
            modal=True,
        )
        prompt.show()

    def _get_selected_paths(self):
        paths = set()
        for pairs in self._selected_items_dict.values():
            for pair in pairs:
                paths.add(pair.actor0_path_model.as_string)
                paths.add(pair.actor1_path_model.as_string)

        return list(paths)

    def _on_selection_changed(self, island_id, selections):
        tree_view = self._tree_views[island_id]
        self._selected_items_dict[island_id] = selections
        selected_paths = self._get_selected_paths()
        self._usd_context.get_selection().set_selected_prim_paths(selected_paths, True)

    def _on_no(self):
        self._hide()

    def _on_yes(self):
        stage = self._usd_context.get_stage()
        omni.kit.undo.begin_group()
        for island_pairs in self._sorted_pairs.values():
            for pair in island_pairs:
                path0 = pair[0]
                path1 = pair[1]
                att_path = path0.AppendElementString("attachment")
                att_path = Sdf.Path(omni.usd.get_stage_next_free_path(stage, str(att_path), False))

                if self._deprecated:
                    omni.kit.commands.execute("CreatePhysicsAttachmentCommand", target_attachment_path=att_path, actor0_path=path0, actor1_path=path1)
                else:
                    omni.kit.commands.execute("CreateAutoDeformableAttachment", target_attachment_path=att_path,
                                              attachable0_path=path0, attachable1_path=path1)

        omni.kit.undo.end_group()
        self._hide()

    def _build_tree_views(self):
        self._tear_down_tree_views()
        self._tree_views = dict()
        self._create_pair_models()

        with self._scrolling_frame:
            with ui.VStack(spacing=10, height=100):          
                for i in range(len(self._sorted_pairs)):
                    with ui.ZStack():                             
                        ui.Rectangle()
                        tree_view = ui.TreeView(
                            self._display_pairs_models[i],
                            root_visible=False,
                            header_visible=False,
                            columns_resizable=True,
                            column_widths=[ui.Fraction(1), ui.Fraction(1)],
                            style=self._style,
                        )
                        self._tree_views[i] = tree_view                

                for i in range(len(self._tree_views)):
                    tree_view = self._tree_views[i]
                    callback = partial(self._on_selection_changed, i)
                    tree_view.set_selection_changed_fn(callback)

    def _tear_down_tree_views(self):
        for i in range(len(self._tree_views)):
            self._tree_views[i] = None
            if i in self._display_pairs_models:
                model = self._display_pairs_models[i]
                self._workaround_set.add(model)

    def _build_dialog_window(self):
        self._window = ui.Window("Create Attachments", dockPreference = omni.ui.DockPreference.DISABLED, width=1015, height=575, padding_x = 15, padding_y = 15)
        self._window.set_visibility_changed_fn(self._on_window_closed)        
        self._window.flags = (ui.WINDOW_FLAGS_NO_COLLAPSE | ui.WINDOW_FLAGS_NO_SCROLLBAR)

        with self._window.frame:            
            with ui.VStack(spacing=10, height=390):
                self._scrolling_frame = ui.ScrollingFrame(
                                        height=390,
                                        horizontal_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                        vertical_scrollbar_policy=ui.ScrollBarPolicy.SCROLLBAR_AS_NEEDED,
                                        style_type_name_override="TreeView",
                                        style=self._style,
                                    )
                self._scrolling_frame.set_build_fn(self._build_tree_views)

                ui.Spacer()
                with ui.HStack(spacing=20):
                    create_wrapped_label("Overlap Tolerance", tooltipText = "Relative margin added to object bounds used to populate pairs for attachment creation. A value of 0.0 indicates that the original bounds are used for generating the overlapping pairs.",
                                        height=20, width=10, tooltipWidth = 300, style={"margin_width": 5, "font_size": 16})
                    fd_tol = ui.FloatDrag(width=66, min=0.0, max=2.0, step=0.01)
                    fd_tol.model.set_value(self._overlap_tolerance)
                    fd_tol.model.add_value_changed_fn(self._on_tolerance_changed)
                with ui.HStack(spacing=20):
                    ui.Button("Remove Selected Pairs", width=220, height=30).set_mouse_pressed_fn(lambda *_: self._on_remove_with_prompt())
                with ui.HStack(spacing=20):
                    ui.Button("Create", width=100, height=30).set_clicked_fn(self._on_yes)
                    ui.Button("Cancel", width=100, height=30).set_clicked_fn(self._on_no)
