# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from omni.kit.property.physics import PhysicsWidget
from omni.kit.property.physics.utils import is_parent_api_instance, OverlayButton
from omni.kit.property.physics.utils import get_schema_name
from pxr import PhysxSchema
import omni.ui as ui
from omni.physx.scripts import utils
import asyncio
from functools import partial
from omni.physxui import get_physxui_interface
from omni.kit.property.physics.widgets import EYE_GLYPH
from omni.kit.property.physics.builders import GearingWidgetBuilder, CustomTokenComboBuilder


class APIInheritanceCheckWidget(PhysicsWidget):
    def __init__(self, title, schema, builders=None):
        super().__init__(title, schema, builders)
        self._any_visible = False

    def on_new_payload(self, payload):
        self._any_visible = False
        return super().on_new_payload(payload)

    def _build_frame(self):
        super()._build_frame()

        if not self._any_visible:
            self.show_frame(False)

    def _build_group_frame(self, name, props, stage):
        if is_parent_api_instance(get_schema_name(self._main_schema), props[0].instance_name, self._payload):
            return None

        self._any_visible = True

        return super()._build_group_frame(name, props, stage)


class FixedTendonWidget(APIInheritanceCheckWidget):
    instances = set()

    def __init__(self, title, schema):
        builders = {
            "gearing": [GearingWidgetBuilder],
            "forceCoefficient": [GearingWidgetBuilder],
        }
        super().__init__(title, schema, builders)
        self._obs = []
        self._switched = None

        FixedTendonWidget.instances.add(self)

    def clean(self):
        super().clean()
        self._cleanup()

    def _cleanup(self):
        self._obs = []
        self._switched = None
        FixedTendonWidget.instances.discard(self)

    def on_new_payload(self, payload):
        self._cleanup()
        ret = super().on_new_payload(payload)
        if ret:
            FixedTendonWidget.instances.add(self)
        return ret

    def _build_group_frame(self, name, props, stage):
        instance_name = props[0].instance_name

        def clicked(curr):
            if self._switched == curr:
                get_physxui_interface().set_tendon_visualization_filter(None)
                self._obs[curr].set_style({"color": ui.color.grey})
                self._switched = None
            else:
                get_physxui_interface().set_tendon_visualization_filter(instance_name)
                for instance in FixedTendonWidget.instances:
                    instance._switched = None
                    for ob in instance._obs:
                        ob.set_style({"color": ui.color.grey})
                self._obs[curr].set_style({"color": ui.color.white})
                self._switched = curr

        with ui.ZStack():
            frame = super()._build_group_frame(name, props, stage)
            if not frame:
                return
            with ui.HStack():
                ui.Spacer(width=ui.Fraction(0.5))
                with ui.VStack(width=0):
                    ui.Spacer(height=3)
                    ob = OverlayButton(EYE_GLYPH, frame, partial(clicked, len(self._obs)), height=16, width=16)
                    ob.set_style({"color": ui.color.grey})
                    self._obs.append(ob)
                ui.Spacer(width=5)


class SpatialTendonWidget(APIInheritanceCheckWidget):
    def _build_group_frame(self, name, props, stage):
        instance_name = props[0].instance_name
        body_path = self._payload[0].pathString

        def clicked(*_):
            get_physxui_interface().select_spatial_tendon_attachment_helper(body_path, instance_name)

        with ui.ZStack():
            frame = super()._build_group_frame(name, props, stage)
            if not frame:
                return
            with ui.HStack():
                ui.Spacer(width=ui.Fraction(0.5))
                with ui.VStack(width=0):
                    ui.Spacer(height=3)
                    OverlayButton(EYE_GLYPH, frame, clicked, height=16, width=16)
                ui.Spacer(width=5)

    def _filter_props_to_build(self, prim):
        filtered_props = super()._filter_props_to_build(prim)

        def remove_props(remove_list):
            nonlocal filtered_props
            filtered_props = [prop for prop in filtered_props if prop.base_name not in remove_list]

        if self._main_schema == PhysxSchema.PhysxTendonAttachmentRootAPI:
            remove_props(["parentAttachment", "parentLink", "gearing"])
        elif len(self._payload) != 1:
            remove_props(["parentAttachment"])

        return filtered_props

    def _build_property_item(self, stage, prop, prim_paths):
        if prop.base_name == "parentAttachment":
            prop.finalize(stage)
            prim = stage.GetPrimAtPath(prim_paths.get_paths()[0])
            parent = utils.get_spatial_tendon_parent_link(prim, prop.instance_name)
            custom_names = [""]
            if parent is not None:
                custom_names.extend(utils.get_spatial_tendon_attachment_candidates(stage.GetPrimAtPath(parent)))

            label_kwargs = self.get_default_label_kwargs()
            model = CustomTokenComboBuilder(stage, prop, prim_paths, label_kwargs, None, custom_names)
            return model
        elif prop.base_name == "parentLink":
            model = super()._build_property_item(stage, prop, prim_paths)

            async def delayed_request():
                await omni.kit.app.get_app().next_update_async()
                self.request_rebuild()

            model.add_item_changed_fn(lambda *_: asyncio.ensure_future(delayed_request()))
            return model
        else:
            return super()._build_property_item(stage, prop, prim_paths)
