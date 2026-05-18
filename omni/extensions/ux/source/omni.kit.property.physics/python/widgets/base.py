# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from .mainframe import MainFrameWidget
from .utils import filter_property_with_multiapply_schema, generate_prims, sort_props
from .utils import get_components, add_component, remove_component, REMOVE_BUTTON_STYLE, ADD_GLYPH, is_multi_api
from .utils import display_name_from_prop_name, display_name_from_name
from .uiprop import UiProp
from ..externals import Prompt
from ..utils import add_disabled_styles, get_schema_name, has_schema
from .. import database
from omni.kit.window.property.templates import build_frame_header
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
from omni.physx.scripts import utils
from omni.physx.scripts.utils import get_TfType_compatible, get_schema_instances
from omni.kit.property.usd.usd_attribute_model import GfVecAttributeModel, TfTokenAttributeModel
from omni.kit.property.usd.usd_model_base import UsdBase
from pxr import Usd, Sdf, Gf
from pxr.Sdf import Path as SdfPath
from pxr.Sdf import PropertySpec as Spec
from pxr.PhysicsSchemaTools import units
from functools import partial
import omni.ui as ui
import omni.usd
import carb
import asyncio
import math
from collections import defaultdict


context_menu = ui.Menu("Context menu", name="physx_context_menu")


class PhysicsWidget(UsdPropertiesWidget):
    def __init__(self, title, schema, builders=None):
        UsdPropertiesWidget.__init__(self, title, False)        

        self._schema_class = schema
        self._main_schema = get_TfType_compatible(schema)
        self._is_main_multi = is_multi_api(self._main_schema)
        self._button_frame = None
        self._allow_refresh = True
        self._property_utils = {}
        self._builders = builders

        self.refresh_apis()

    def refresh_apis(self):
        extras_apis = database.get_extras_apis(self._schema_class)
        internal_ext_apis = database.get_internal_extension_apis(self._schema_class)

        self._all_schema_keys = [self._schema_class] + internal_ext_apis + extras_apis
        self._all_schemas = [get_TfType_compatible(schema) for schema in self._all_schema_keys]

        self._single_schema_prop_names_set = set()
        self._single_schema_prop_name_to_api = {}
        self._multi_schema_prop_names = {}

        for schema in self._all_schemas:
            if is_multi_api(schema):
                self._multi_schema_prop_names[schema] = utils.getSchemaPropertyNames(schema)
            else:
                names = utils.getSchemaPropertyNames(schema)
                self._single_schema_prop_names_set.update(names)
                self._single_schema_prop_name_to_api.update({n: schema for n in names})

        self._int_ext_apis = [get_TfType_compatible(schema) for schema in internal_ext_apis]
        self._int_ext_prop_specs = {}

        for api in self._int_ext_apis:
            prim_def = utils.getSchemaPrimDef(api)
            self._int_ext_prop_specs[api] = [prim_def.GetSchemaPropertySpec(name) for name in prim_def.GetPropertyNames()]

    def clean(self):
        super().clean()
        self._button_frame = None

    def request_rebuild(self):
        if self._allow_refresh:
            super().request_rebuild()

    def _is_main_schema_present(self):
        if len(self._payload) == 0:
            return False

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)

            if not prim or not has_schema(prim, self._main_schema):
                return False

        if self._is_main_multi:
            instances = get_schema_instances(prim, Usd.SchemaRegistry().GetSchemaTypeName(self._main_schema))
            if not len(instances):
                return False

        return True

    def _on_usd_changed(self, notice, stage):
        # If the stage parameters change, it will trigger a notice for the absoluteRootPath.
        if SdfPath.absoluteRootPath in notice.GetChangedInfoOnlyPaths() and stage == self._units_stage:
            stage_info = database.StageInfo(stage)
            
            if stage_info != self._stage_info:
                self._stage_info = stage_info
                self.request_rebuild()


        super()._on_usd_changed(notice, stage)

    def on_new_payload(self, payload):
        if not UsdPropertiesWidget.on_new_payload(self, payload):
            return False

        if not self._payload or len(self._payload) == 0:
            return False

        return self._is_main_schema_present()

    def build_items(self):
        stage = omni.usd.get_context().get_stage()

        if stage:
            self._units_stage = stage
            self._stage_info = database.StageInfo(stage)

        super().build_items()

    def _customize_props_layout(self, props):
        return sort_props(props, self._all_schema_keys)

    def _get_shared_properties_from_selected_prims(self, anchor_prim):
        shared_props_dict = None
        for prim_path in self._payload:
            prim = self._get_prim(prim_path)
            props = self._filter_props_to_build(prim)
            prop_dict = {prop.prop_name: prop for prop in props}
            if shared_props_dict is None:
                shared_props_dict = prop_dict
            else:
                intersect_shared_props = {}
                for prop_name, prop_info in shared_props_dict.items():
                    prop = prop_dict.get(prop_name)
                    if prop is not None:
                        if prop.from_spec:
                            prop_info.apply_schema = prop.apply_schema
                            prop_info.apply_to_prims += prop.apply_to_prims
                        intersect_shared_props[prop_name] = prop_info
                if len(intersect_shared_props) == 0:
                    return

                shared_props_dict = intersect_shared_props
        shared_props = list(shared_props_dict.values())
        return shared_props

    def _filter_props_to_build(self, prim):
        filtered_props = []

        # prims and applied apis
        for prop in prim.GetProperties():
            prop_name = prop.GetName()
            prop_prim = prop.GetPrim()
            if prop_name in self._single_schema_prop_names_set:
                # basically a HasAPI check, but cheaper and needs to be got to create the prop anyway
                prop_spec = prop_prim.GetPrimDefinition().GetSchemaPropertySpec(prop_name)
                if prop_spec is None:
                    def get_name():
                        name = self._single_schema_prop_name_to_api.get(prop_name)
                        return f"a {get_schema_name(name)}" if name else "an unknown API"
                    carb.log_warn(f"{prop_prim.GetPath().pathString}:{prop_name} is authored, but {get_name()} is not applied. Simulation will default to a fallback value. Edit this value in the Property window to autoapply the correct API or manually edit the 'apiSchemas' list!\n")
                else:
                    filtered_props.append(UiProp().from_property(prop, prop_spec=prop_spec))
            else:
                for schema, prop_names in self._multi_schema_prop_names.items():
                    uiprop = filter_property_with_multiapply_schema(prop, schema, prop_names)
                    if uiprop is not None:
                        filtered_props.append(uiprop)

        # generate non-applied internal extension api properties from spec
        for api in self._int_ext_apis:
            condition = database.extension_api_conditions.get(api)
            if condition is not None:
                if not all([condition(prim) for prim in generate_prims(self._payload)]):
                    continue

            if is_multi_api(self._main_schema):
                main_schema_type_name = Usd.SchemaRegistry().GetSchemaTypeName(self._main_schema)
                ext_schema_type_name = Usd.SchemaRegistry().GetSchemaTypeName(api)

                main_instances = set(get_schema_instances(prim, main_schema_type_name))
                ext_instances = set(get_schema_instances(prim, ext_schema_type_name))
                instances_not_applied = main_instances - ext_instances

                prefix = database.ext_multi_api_prefixes[api]

                for instance in instances_not_applied:
                    instance_props = [UiProp().from_property_spec(prop_spec, prim, api, instance, prefix) for prop_spec in self._int_ext_prop_specs[api]]
                    filtered_props += instance_props
            else:
                if not prim.HasAPI(api):
                    for prop_spec in self._int_ext_prop_specs[api]:
                        prop = UiProp().from_property_spec(prop_spec, prim, api)
                        filtered_props.append(prop)

        return filtered_props

    def _remove_prompt_check(self, title, on_yes):
        prompt = Prompt(
            "Remove component?", f"Are you sure you want to remove the '{title}' component?", "Yes", "No",
            ok_button_fn=on_yes, modal=True)
        prompt.show()

    def _show_remove_button(self):
        return True

    def _build_impl_with_remove_button(self, component):
        # behold the greatness of my omni.ui-fu! yeah, just kidding, items in a zstack (or wherever, but
        # it's important here) cannot stop mouse events going through the whole stack so I have to revert
        # the underlying frame's collapse when the remove button is clicked ...
        revert_last = False

        def on_collapsed_changed(collapsed):
            nonlocal revert_last
            if revert_last and collapsed:
                self._collapsable_frame.collapsed = False
            revert_last = False

        def on_remove_clicked_mouse(*_):
            def on_yes():
                nonlocal revert_last
                revert_last = True
                remove_component(component, self._payload)

            self._remove_prompt_check(component.title, on_yes)

        self._button_frame = ui.Frame()
        with self._button_frame:
            with ui.ZStack():
                super().build_impl()
                with ui.HStack():
                    ui.Spacer(width=ui.Fraction(0.5))
                    with ui.VStack(width=0):
                        ui.Spacer(height=5)
                        ui.Button(style=REMOVE_BUTTON_STYLE, height=16, width=16, identifier=f"remove_component {component.title}").set_mouse_pressed_fn(on_remove_clicked_mouse)
                    ui.Spacer(width=5)
        self._collapsable_frame.set_collapsed_changed_fn(on_collapsed_changed)

    def build_impl(self):
        self._button_frame = None
        component = database.components.get(get_schema_name(self._main_schema))
        if component is not None and self._show_remove_button():
            self._build_impl_with_remove_button(component)
        else:
            super().build_impl()

    def is_visible(self):
        return self._is_main_schema_present()

    def show_frame(self, show):
        self._collapsable_frame.visible = show
        if self._button_frame is not None:
            self._button_frame.visible = show

    def _build_frame(self):
        # need to double check here - frame can be refreshed on underlying USD change
        # or a change might have happened between payload check and UI build
        if self._is_main_schema_present():
            super()._build_frame()
            self.show_frame(self._any_item_visible)
        else:
            ui.VStack(height=0)
            self.show_frame(False)

    def get_property_sentinels(self, base_name: str) -> dict:
        sentinels = {}
        for key, value in database.property_sentinels.get(base_name, {}).items():
            if callable(key):
                key = key(self._stage_info)
            if callable(value):
                value = value(self._stage_info)

            sentinels[key] = value
            
        return sentinels

    def build_property_item(self, stage, prop, prim_paths):
        def precompute_menu():
            sentinels_cpy = self.get_property_sentinels(prop.base_name)
            sentinels_inv = {v: k for k, v in sentinels_cpy.items()}
            property_utils = self._property_utils.get(prop.base_name, {})
            items = []

            def add_base(name, value):
                sentinel_name = sentinels_inv.get(value)
                if sentinel_name is not None:
                    items.append((f"{name} ({sentinel_name})", value))
                else:
                    items.append((name, value))

            if prop.info is not None:
                add_base("Set Minimum", prop.info.min)
                add_base("Set Maximum", prop.info.max)
            if len(sentinels_cpy) != 0 and prop.info is not None:
                items.append(("", None))
            for name, value in sentinels_cpy.items():
                items.append((f"Set {name}", value))
            for name, value in property_utils.items():
                items.append((f"{name}", value))

            return items

        def populate_menu(model, menu_items):
            def set_value(model, value):
                # remove property when the value is from an N/A sentinel
                if value == database.not_authored_token:
                    for path in model._object_paths:
                        prim = model._stage.GetPrimAtPath(path.GetPrimPath())
                        prim.RemoveProperty(prop.base_name)
                    return

                # temporarily disable limits to set sentinels outside of that range
                disable_limits = isinstance(model, UsdBase)
                if disable_limits:
                    tmp_min, tmp_max = model._min, model._max
                    model._min, model._max = None, None

                # set value either in a singlechannel or a multichannel mode
                if isinstance(model, GfVecAttributeModel):
                    for i, item in enumerate(model._items):
                        item.model.set_value(value[i])
                    model.set_value(value)
                else:
                    model.set_value(value)

                if disable_limits:
                    model._min, model._max = tmp_min, tmp_max

            ui.Separator()

            for item in menu_items:
                n, v = item
                if n == "":
                    ui.Separator()
                else:
                    ui.MenuItem(n).set_triggered_fn(lambda v=v: v() if callable(v) else set_value(model, v))

        model = self._build_property_item(stage, prop, prim_paths)
        menu_items = precompute_menu()
        if menu_items:
            MainFrameWidget.instance.register_popup_menu_populate_fn(prop.base_name, partial(populate_menu, model, menu_items))

    def get_default_label_kwargs(self):
        label_kwargs = {}
        if self._filter.name:
            label_kwargs["highlight"] = self._filter.name
        return label_kwargs

    def _build_property_item(self, stage, prop, prim_paths):
        prop.finalize(stage)

        if not prop.metadata.get(Spec.DisplayNameKey):
            prop.override_display_name(display_name_from_prop_name(prop))

        label_kwargs = self.get_default_label_kwargs()
        model = None

        def build_widget():
            # create widget
            available_builders = []

            # search local builders first
            if self._builders is not None:
                available_builders = [val for key, val in self._builders.items() if prop.prop_name.endswith(key)]
            
            # search global builders if local builders are not found
            if len(available_builders) == 0:
                available_builders = database.get_available_builders(prop.prop_name)

            if len(available_builders) != 0:
                builder_data = available_builders[0]
                models = builder_data[0](stage, prop, prim_paths, label_kwargs, None, *builder_data[1:])
            else:
                widget_kwargs = {"step": prop.step} if prop.step is not None else {}
                if UsdPropertiesWidgetBuilder._get_type_name(prop.metadata).type == Sdf.ValueTypeNames.Token.type:
                    widget_kwargs["model_cls"] = TfTokenAttributeModel

                units_text = units.get_attribute_units_as_text(prop.prop_name, stage)
                if units_text:
                    tooltip = widget_kwargs.get("tooltip", "")
                    widget_kwargs["tooltip"] =  tooltip + ("\n" if tooltip else "") + units_text[1]

                widget_kwargs["identifier"] = f"physprop_{prop.prop_name}"
                if MainFrameWidget.instance.is_a_omnijoint:
                    widget_kwargs["model_kwargs"] = {"auto_target_session_layer_def": False}
                models = UsdPropertiesWidgetBuilder.build(stage, prop.prop_name, prop.metadata, prop.property_type, prim_paths, label_kwargs, widget_kwargs)

            # register and return its model
            if not isinstance(models, list):
                model = models
                for prim_path in self._payload:
                    self._models[prim_path.AppendProperty(prop.prop_name)].append(model)

                # FIXME: fix uint min being overriden in UsdPropertiesWidgetBuilder.build
                if prop.metadata.get("typeName") == 'uint' and prop.info and prop.info_min:
                    model._min = prop.info_min

                return model, None
            else:
                for model in models:
                    for prim_path in prim_paths:
                        self._models[prim_path.AppendProperty(prop.prop_name)].append(model)
                # for per channel builders the last model is AbstractItemModel covering all
                return models[-1], models[:-1]

        sentinels = self.get_property_sentinels(prop.base_name)
        if len(sentinels.keys()) == 0:
            model, _ = build_widget()
        else:
            with ui.ZStack():
                model, models = build_widget()
                stc = ui.HStack()
                with stc:
                    ui.Spacer(width=173)
                    with ui.ZStack():
                        ui.Rectangle(name="value")
                        lbl = ui.Label("Overlay Text", name="label", alignment=ui.Alignment.CENTER)
                        add_disabled_styles(lbl)
                        setattr(model, "sentinel_widget", lbl)

                        sentinels_inverted = {v: k for k, v in sentinels.items()}
                        editing = False

                        def toggle_sentinel():
                            def get_sentinel():
                                na_sentinel = sentinels_inverted.get(database.not_authored_token)
                                if na_sentinel is not None:
                                    # NOTE: prop.is_authored is refreshed only when the widget is first loaded
                                    # so we have to query the usd attributes directly
                                    attributes = model._get_attributes()
                                    is_authored = any([hasattr(a, "IsAuthored") and a.IsAuthored() for a in attributes])

                                    if not is_authored:
                                        return na_sentinel

                                if isinstance(model.get_value(), Gf.Quatf):
                                    # Quatf is not hashable
                                    current_val = model.get_value()
                                    for k, v in sentinels.items():
                                        if v == current_val:
                                            return k
                                    return None

                                return sentinels_inverted.get(model.get_value())

                            sentinel = get_sentinel()
                            if sentinel is not None:
                                stc.visible = True
                                lbl.text = sentinel
                            else:
                                stc.visible = False

                        async def show_check():
                            await asyncio.sleep(0.3)  # matching ImGui double-click delay
                            if editing:
                                stc.visible = False

                        def begin_edit(*_):
                            nonlocal editing
                            editing = True
                            asyncio.ensure_future(show_check())

                        def end_edit(*_):
                            nonlocal editing
                            editing = False
                            toggle_sentinel()

                        def changed(*_):
                            if not editing:
                                toggle_sentinel()

                        model.add_begin_edit_fn(begin_edit)
                        model.add_end_edit_fn(end_edit)
                        if isinstance(model, ui.AbstractValueModel):
                            # singlechannel widget
                            model.add_value_changed_fn(changed)
                        else:
                            if models is not None:
                                # multichannel widget
                                for submodel in models:
                                    submodel.add_begin_edit_fn(begin_edit)
                                    submodel.add_end_edit_fn(end_edit)
                            else:
                                # single widget representing multiple channels
                                for item in model._items:
                                    item.model.add_begin_edit_fn(begin_edit)
                                    item.model.add_end_edit_fn(end_edit)
                            model.add_item_changed_fn(changed)

                        toggle_sentinel()
                    ui.Spacer(width=15)

        # FIXME: override change_on_edit_end (sdk might end up with this as a default someday ..)
        if hasattr(model, "_change_on_edit_end"):
            model._change_on_edit_end = database.property_change_on_edit_end_override.get(prop.base_name, True)

        # FIXME: override getters to return the multiplied default where neccessary
        if prop.info and (prop.info.multiply_meter_fn or prop.info.multiply_kg_fn) and prop.default:
            def get_default_value(default, property):
                return True, default

            def is_different_from_default(model, default):
                ret = not math.isclose(model.get_value(), default, abs_tol=1e-5)
                return ret

            def read_value(default, orig_fn, object, time_code):
                if isinstance(object, Usd.Attribute) and not object.IsAuthored():
                    return default
                return orig_fn(object, time_code)

            model._get_default_value = partial(get_default_value, prop.default)
            model.is_different_from_default = partial(is_different_from_default, model, prop.default)
            model._read_value = partial(read_value, prop.default, model._read_value)

            # refresh to update control state, model's inner variables and the widget
            model._update_value(force=True)
            model._value_changed()

        # catch change on ext non-applied API properties and apply them
        # this is better than the previous approach with change callbacks, we can apply the api
        # before the property is set, but still a FIXME, every function override should be removed
        # sooner or later
        if prop.apply_schema is not None:
            def create_placeholder_attributes(prop, model, orig_fn, attributes):
                # force skip refresh to prevent a visual glitch
                self._allow_refresh = False

                # apply new schema before the property is set
                for attribute in attributes:
                    if prop.instance_name is not None:
                        attribute.GetPrim().ApplyAPI(prop.apply_schema, prop.instance_name)
                    else:
                        attribute.GetPrim().ApplyAPI(prop.apply_schema)

                # write into the original list (so that the subsequent property change is not done
                # to the placeholder) and switch back the original method
                attributes[:] = model._get_attributes()
                model._create_placeholder_attributes = orig_fn
                self._allow_refresh = True

            model._create_placeholder_attributes = partial(
                create_placeholder_attributes, prop, model,
                model._create_placeholder_attributes
            )

        return model

    def _build_group_frame(self, name, props, stage):
        if len(props) == 0:
            return None

        is_instance = props[0].instance_name is not None

        def get_title():
            if not is_instance:
                return display_name_from_name(name)
            if self._main_schema in database.display_raw_instance_name_apis:
                return name
            return database.instance_display_names.get(name, display_name_from_name(name))

        def build_instance_frame(component=None):
            collapsed = not is_instance
            frame = ui.CollapsableFrame(title=get_title(), build_header_fn=build_frame_header, name="subFrame")
            with frame:
                with ui.HStack():
                    with ui.VStack(height=0, spacing=5, name="frame_v_stack"):
                        for prop in props:
                            self.build_property_item(stage, prop, self._payload)
                            collapsed |= prop.display_group_collapsed
                    if component is not None:
                        def on_remove_clicked():
                            def on_yes():
                                remove_component(component, self._payload, props[0].instance_name)
                            self._remove_prompt_check(component.title, on_yes)
                        ui.Button(style=REMOVE_BUTTON_STYLE, clicked_fn=on_remove_clicked, width=16, identifier=f"remove_component {component.title}")
            frame.collapsed = collapsed
            return frame

        if self._is_main_multi:
            instance_name = f"{get_schema_name(self._main_schema)}:{props[0].instance_name}"
            component = database.components.get(instance_name)
            if component is not None:
                return build_instance_frame(component)
            else:
                component = database.components.get(get_schema_name(self._main_schema))
                return build_instance_frame(component)
        else:
            return build_instance_frame()

    def build_nested_group_frames(self, stage, grouped_props):
        for prop in grouped_props.props:
            self.build_property_item(stage, prop, self._payload)

        instances = defaultdict(list)

        # init custom instance order if available
        custom_order = database.custom_instance_order_map.get(self._main_schema)
        if custom_order is not None:
            for inst_name in custom_order:
                instances[inst_name] = []

        for group in grouped_props.sub_groups.values():
            for prop in group.props:
                instances[prop.display_group].append(prop)

        for name, props in instances.items():
            self._build_group_frame(name, props, stage)

        if self._is_main_multi:
            schema_components = get_components(self._payload, self._main_schema)

            def on_click():
                context_menu.clear()
                with context_menu:
                    for c in schema_components:
                        ui.MenuItem(c.title).set_triggered_fn(partial(add_component, c, self._payload))
                context_menu.show()

            if len(schema_components) > 0 and self._main_schema in database.multi_api_add_button_text:
                ui.Button(f"{ADD_GLYPH} Add {database.multi_api_add_button_text[self._main_schema]}", clicked_fn=on_click)


class ExtensionSchemaWidget(PhysicsWidget):
    """Widget for extension APIs that shows when the parent schema is present,
    even if the extension API itself is not applied."""

    def __init__(self, title, schema, parent_schema, builders=None):
        self._parent_schema_class = parent_schema
        self._parent_main_schema = get_TfType_compatible(parent_schema)
        self._is_parent_multi = is_multi_api(self._parent_main_schema)

        # Cache property specs for generating from-spec properties when not applied
        ext_api = get_TfType_compatible(schema)
        prim_def = utils.getSchemaPrimDef(ext_api)
        self._ext_prop_specs = [prim_def.GetSchemaPropertySpec(name) for name in prim_def.GetPropertyNames()]

        super().__init__(title, schema, builders)

    def _is_main_schema_present(self):
        """Show when the parent schema is present (not this extension API)."""
        if len(self._payload) == 0:
            return False

        for prim_path in self._payload:
            prim = self._get_prim(prim_path)
            if not prim or not has_schema(prim, self._parent_main_schema):
                return False

            # Check extension API conditions (e.g., mesh approximation checks)
            condition = database.extension_api_conditions.get(self._main_schema)
            if condition is not None and not condition(prim):
                return False

        if self._is_parent_multi:
            instances = get_schema_instances(prim, Usd.SchemaRegistry().GetSchemaTypeName(self._parent_main_schema))
            if not len(instances):
                return False

        return True

    def _filter_props_to_build(self, prim):
        # Get applied properties using parent class logic
        filtered_props = super()._filter_props_to_build(prim)

        # Generate from-spec properties for non-applied cases
        if self._is_main_multi:
            parent_type_name = Usd.SchemaRegistry().GetSchemaTypeName(self._parent_main_schema)
            ext_type_name = Usd.SchemaRegistry().GetSchemaTypeName(self._main_schema)

            parent_instances = set(get_schema_instances(prim, parent_type_name))
            ext_instances = set(get_schema_instances(prim, ext_type_name))
            instances_not_applied = parent_instances - ext_instances

            prefix = database.ext_multi_api_prefixes.get(self._main_schema)

            for instance in instances_not_applied:
                instance_props = [UiProp().from_property_spec(prop_spec, prim, self._main_schema, instance, prefix) for prop_spec in self._ext_prop_specs]
                filtered_props += instance_props
        else:
            if not prim.HasAPI(self._main_schema):
                for prop_spec in self._ext_prop_specs:
                    filtered_props.append(UiProp().from_property_spec(prop_spec, prim, self._main_schema))

        return filtered_props
