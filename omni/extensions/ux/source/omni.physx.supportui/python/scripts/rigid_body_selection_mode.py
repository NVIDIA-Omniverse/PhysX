# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import omni.usd
import omni.ui as ui
import omni.kit.widget.layers as layers
from .utils import get_rigid_body_parent_prim


class RigidBodySelectionMode:
    class HookInfo:
        def __init__(self):
            self.tree_view = None
            self.sel_watch = None
            self.orig_sel_changed_fn = None
            self.new_sel_changed_fn = None

        def hook(self, tree_view, sel_watch, current_sel_changed_fn, new_sel_changed_fn):
            if self.tree_view != tree_view or self.sel_watch != sel_watch:
                self.tree_view = tree_view
                self.sel_watch = sel_watch
                self.orig_sel_changed_fn = current_sel_changed_fn
                self.new_sel_changed_fn = new_sel_changed_fn
                tree_view.set_selection_changed_fn(new_sel_changed_fn)

        def unhook(self):
            if self.tree_view:
                self.tree_view.set_selection_changed_fn(self.orig_sel_changed_fn)
            self.tree_view = None
            self.sel_watch = None
            self.orig_sel_changed_fn = None
            self.new_sel_changed_fn = None

        def __del__(self):
            self.unhook()

    def __init__(self):
        self._enabled = False
        self._stage_wnd_in_selection = False
        self._layers_wnd_in_selection = False
        self._stage_wnd_hook = self.HookInfo()
        self._layers_wnd_hook = self.HookInfo()
        self._last_selected_prim_paths = []

    @property
    def enabled(self) -> bool:
        return self._enabled

    @enabled.setter
    def enabled(self, enable: bool):
        if self._enabled == enable:
            return

        self._enabled = enable
        self._enable_sel_op_main_toolbar_button(enable)

        if enable:
            self.check_external_hooks()
        else:
            self.restore_orig_external_hooks()

        self._stage_wnd_in_selection = False
        self._layers_wnd_in_selection = False

    def _enable_sel_op_main_toolbar_button(self, enable):
        import omni.kit.window.toolbar
        from omni.kit.window.toolbar.builtin_tools.select_button_group import SelectButtonGroup

        toolbar = omni.kit.window.toolbar.get_instance()

        widget_instance = toolbar

        if hasattr(toolbar, '_widget_instance') and toolbar._widget_instance is not None:
            widget_instance = toolbar._widget_instance

        if widget_instance._toolbar_widget_groups is None:
            carb.log_warn(
                f"Couldn't obtain main toolbar instance. "
                "Once rigid body selection mode is activated, "
                "it won't be able to hide selection button on the main toolbar."
            )
            return

        for _, widget_group in widget_instance._toolbar_widget_groups:
            if isinstance(widget_group, SelectButtonGroup):
                widget_group._select_mode_button.visible = not enable
                break

    def _get_stage_sel(self):
        stage_wnd = ui.Workspace.get_window("Stage")
        if stage_wnd:
            return stage_wnd._selection

    def _get_stage_tree_view(self):
        stage_sel = self._get_stage_sel()
        if stage_sel:
            return stage_sel._tree_view

    def _get_layers_sel(self):
        layers_ext = layers.get_instance()
        if layers_ext and layers_ext._window:
            return layers_ext._window._selection

    def _get_layers_tree_view(self):
        layers_sel = self._get_layers_sel()
        if layers_sel:
            return layers_sel._tree_view

    def _on_stage_wnd_selection_changed(self, selection):
        self._stage_wnd_hook.orig_sel_changed_fn(selection)
        self._stage_wnd_in_selection = not self._stage_wnd_hook.sel_watch._in_selection

    def _on_layers_wnd_selection_changed(self, selection):
        self._layers_wnd_hook.orig_sel_changed_fn(selection)
        self._layers_wnd_in_selection = not self._layers_wnd_hook.sel_watch._in_selection

    def check_external_hooks(self):
        stage_sel = self._get_stage_sel()
        if stage_sel:
            self._stage_wnd_hook.hook(
                self._get_stage_tree_view(),
                stage_sel,
                stage_sel._on_widget_selection_changed,
                self._on_stage_wnd_selection_changed
            )

        layers_sel = self._get_layers_sel()
        if layers_sel:
            self._layers_wnd_hook.hook(
                self._get_layers_tree_view(),
                layers_sel,
                layers_sel._on_widget_selection_changed,
                self._on_layers_wnd_selection_changed
            )

    def restore_orig_external_hooks(self):
        if (
            self._stage_wnd_hook.tree_view == self._get_stage_tree_view()
            and self._stage_wnd_hook.sel_watch == self._get_stage_sel()
        ):
            self._stage_wnd_hook.unhook()
        if (
            self._layers_wnd_hook.tree_view == self._get_layers_tree_view()
            and self._layers_wnd_hook.sel_watch == self._get_layers_sel()
        ):
            self._layers_wnd_hook.unhook()

    def try_select_rigid_body_parent(self):
        if not self.enabled:
            return

        ctx = omni.usd.get_context()
        stage = ctx.get_stage()
        selection = ctx.get_selection()

        # Make sure it's a new selection. It happens that omni.usd sends the same selection twice. No sorting because
        # the order of selection is important.
        prim_paths = selection.get_selected_prim_paths()
        if prim_paths == self._last_selected_prim_paths:
            return

        self._last_selected_prim_paths = prim_paths

        if self._stage_wnd_in_selection or self._layers_wnd_in_selection:
            self._stage_wnd_in_selection = False
            self._layers_wnd_in_selection = False
            return

        # this is only needed because of possible stage or layer extension reload
        self.check_external_hooks()

        rb_prims = set()

        for path in prim_paths:
            prim = stage.GetPrimAtPath(path)
            rb_prim = get_rigid_body_parent_prim(prim)
            if rb_prim is not None:
                rb_prims.add(rb_prim)

        if len(rb_prims) > 0:
            rb_prim_paths = [str(p.GetPath()) for p in rb_prims]
            # selection.set_selected_prim_paths() will trigger another StageEventType.SELECTION_CHANGED event!
            # The only reason why this does not end up in endless loop is when rb_prim_paths
            # equal the current selection. Then no change event is triggered.
            selection.set_selected_prim_paths(rb_prim_paths, True)
            self._last_selected_prim_paths = rb_prim_paths
