# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
import math
from omni.kit.property.physx.models import CustomMetadataObjectModel
import omni.physxcommands as commands
from omni.kit.window.property.templates import HORIZONTAL_SPACING
from omni.kit.property.usd.usd_attribute_model import UsdAttributeModel, TfTokenAttributeModel
from omni.kit.property.usd.usd_attribute_model import GfQuatEulerAttributeModel
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder
import omni.ui as ui
import omni.physx.bindings._physx as pxb
from omni.physx.scripts.pythonUtils import ScopeGuard
from .utils import get_display_name, Limits
from pxr import Sdf, Vt
import carb
import carb.settings
import omni.kit.app
import omni.kit.undo


class LocalSpaceVelocitiesWidgetBuilder(CustomMetadataObjectModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs, name):
        omni.kit.undo.subscribe_on_change(self._undo_redo_on_change)

        self._velocities_in_local_space = carb.settings.get_settings().get_as_bool(pxb.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE)
        self._settings_subscription = omni.kit.app.SettingChangeSubscription(
            pxb.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE, self.on_velocities_settings_changed
        )

        super().__init__(stage, prop, prim_paths, name)

        self._info_str_default_value = "Default value (coming from Physics Settings)"
        self._info_str_same_value_mixed_sources = "This value comes partially from locally overridden value(s), partially from Physics Settings default"

        with ui.HStack(spacing=HORIZONTAL_SPACING):
            UsdPropertiesWidgetBuilder._create_label(prop.attr_name, prop.metadata, label_kwargs)
            with ui.VStack(width=10):
                ui.Spacer()
                with ui.ZStack():
                    with ui.Placer(offset_x=0, offset_y=-2):
                        self._check_box = ui.CheckBox(width=10, height=0, name="greenCheck",
                                                      identifier="LocalSpaceVelocitiesCheckBox")
                    with ui.Placer(offset_x=1, offset_y=-1):
                        self._mixed_overlay = ui.Rectangle(height=8, width=8, name="mixed_overlay",
                                                           identifier="LocalSpaceVelocitiesMixedOverlay",
                                                           alignment=ui.Alignment.CENTER, visible=False)
                ui.Spacer(width=10)
            ui.Spacer(width=ui.Fraction(1))
            self._reset_button = ui.Button("Reset to Default", width=0, alignment=ui.Alignment.RIGHT_CENTER,
                                           clicked_fn=self._reset_to_default, identifier="LocalSpaceVelocitiesResetButton",
                                           tooltip="Sets the value in accordance to Physics Settings")
            ui.Spacer(width=11)

        self._scope_guard = ScopeGuard()

        self._determine_value()
        self._check_box.model.set_value(self.get_value())
        self._update_ui_state()
        self._cb_value_changed_id = self._check_box.model.add_value_changed_fn(self._cb_value_changed_fn)
        self._user_initiated_change = True

    def clean(self):
        omni.kit.undo.unsubscribe_on_change(self._undo_redo_on_change)
        self._settings_subscription = None
        self._mixed_overlay = None
        self._check_box.model.remove_value_changed_fn(self._cb_value_changed_id)
        self._check_box = None
        self._reset_button = None
        super().clean()

    def _cb_value_changed_fn(self, value):
        if self._user_initiated_change:
            self.set_value(value.as_bool)
        self._update_ui_state()

    def _reset_to_default(self):
        self.set_value(self._get_default_value(), True)
        self._check_box.model.set_value(self.get_value())
        self._update_ui_state()

    def _get_default_value(self):
        return self._velocities_in_local_space

    def _update_ui_state(self):
        self._mixed_overlay.visible = self.is_ambiguous()
        self._reset_button.enabled = self.is_ambiguous() or self._info_str != self._info_str_default_value
        self._check_box.tooltip = self.get_info_str()

    def _reflect_current_state_in_ui(self):
        if self._scope_guard.is_guarded():
            return

        with self._scope_guard:
            self._determine_value()
            self._user_initiated_change = False
            self._check_box.model.set_value(self.get_value())
            self._user_initiated_change = True
            self._update_ui_state()

    def on_velocities_settings_changed(self, item, event_type):
        if event_type == carb.settings.ChangeEventType.CHANGED:
            self._velocities_in_local_space = carb.settings.get_settings().get_as_bool(pxb.SETTING_OUTPUT_VELOCITIES_LOCAL_SPACE)
            self._reflect_current_state_in_ui()

    def _undo_redo_on_change(self, cmds):
        if commands.SetCustomMetadataCommand.__name__ in cmds:
            self._reflect_current_state_in_ui()


class ReferenceFrameIsCenterOfMassWidgetBuilder(CustomMetadataObjectModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs, name):
        omni.kit.undo.subscribe_on_change(self._undo_redo_on_change)

        self._default_value = True

        super().__init__(stage, prop, prim_paths, name)

        self._info_str_default_value = "Default value"
        self._info_str_same_value_mixed_sources = "This value comes partially from locally overridden value(s), partially from default"

        with ui.HStack(spacing=HORIZONTAL_SPACING):
            UsdPropertiesWidgetBuilder._create_label(prop.attr_name, prop.metadata, label_kwargs)
            with ui.VStack(width=10):
                ui.Spacer()
                with ui.ZStack():
                    with ui.Placer(offset_x=0, offset_y=-2):
                        self._check_box = ui.CheckBox(width=10, height=0, name="greenCheck",
                                                      identifier="ReferenceFrameIsCenterOfMassCheckBox")
                    with ui.Placer(offset_x=1, offset_y=-1):
                        self._mixed_overlay = ui.Rectangle(height=8, width=8, name="mixed_overlay",
                                                           identifier="ReferenceFrameIsCenterOfMassMixedOverlay",
                                                           alignment=ui.Alignment.CENTER, visible=False)
                ui.Spacer(width=10)
            ui.Spacer(width=ui.Fraction(1))
            self._reset_button = ui.Button("Reset to Default", width=0, alignment=ui.Alignment.RIGHT_CENTER,
                                           clicked_fn=self._reset_to_default, identifier="ReferenceFrameIsCenterOfMassResetButton",
                                           tooltip="Sets the value to the default")
            ui.Spacer(width=11)

        self._scope_guard = ScopeGuard()

        self._determine_value()
        self._check_box.model.set_value(self.get_value())
        self._update_ui_state()
        self._cb_value_changed_id = self._check_box.model.add_value_changed_fn(self._cb_value_changed_fn)
        self._user_initiated_change = True

    def clean(self):
        omni.kit.undo.unsubscribe_on_change(self._undo_redo_on_change)
        self._mixed_overlay = None
        self._check_box.model.remove_value_changed_fn(self._cb_value_changed_id)
        self._check_box = None
        self._reset_button = None
        super().clean()

    def _cb_value_changed_fn(self, value):
        if self._user_initiated_change:
            self.set_value(value.as_bool)
        self._update_ui_state()

    def _reset_to_default(self):
        self.set_value(self._get_default_value(), True)
        self._check_box.model.set_value(self.get_value())
        self._update_ui_state()

    def _get_default_value(self):
        return self._default_value

    def _update_ui_state(self):
        self._mixed_overlay.visible = self.is_ambiguous()
        self._reset_button.enabled = self.is_ambiguous() or self._info_str != self._info_str_default_value
        self._check_box.tooltip = self.get_info_str()

    def _reflect_current_state_in_ui(self):
        if self._scope_guard.is_guarded():
            return

        with self._scope_guard:
            self._determine_value()
            self._user_initiated_change = False
            self._check_box.model.set_value(self.get_value())
            self._user_initiated_change = True
            self._update_ui_state()

    def _undo_redo_on_change(self, cmds):
        if commands.SetCustomMetadataCommand.__name__ in cmds:
            self._reflect_current_state_in_ui()


def RelationshipWidgetBuilder(stage, prop, prim_paths, label_kwargs, widget_kwargs, filter_types=[], targets_limit=0, filter_fn=None, target_name=None):
    models = None
    additional_widget_kwargs = {"target_picker_filter_type_list": filter_types, "targets_limit": targets_limit}

    if filter_fn is not None:
        additional_widget_kwargs["target_picker_filter_lambda"] = filter_fn

    if target_name is not None:
        additional_widget_kwargs["target_name"] = target_name

    with ui.HStack(spacing=HORIZONTAL_SPACING):
        models = UsdPropertiesWidgetBuilder._relationship_builder(stage, prop.attr_name, prop.metadata, prim_paths, label_kwargs, additional_widget_kwargs)

    return models


class BitfieldWidgetBuilder(UsdAttributeModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs, bits):
        super().__init__(stage, [path.AppendProperty(prop.attr_name) for path in prim_paths], True, prop.metadata)
        self.add_value_changed_fn(lambda x: self._scatter_value())
        self.item_models = []
        self._edit_guard = ScopeGuard()

        with ui.HStack(spacing=HORIZONTAL_SPACING):
            UsdPropertiesWidgetBuilder._create_label(prop.attr_name, prop.metadata, label_kwargs)
            for i in range(0, len(bits)):
                ui.Label(bits[i], name="label", width=0)
                ui.Spacer(width=5)
                with ui.VStack(width=10):
                    ui.Spacer()
                    cb = ui.CheckBox(width=10, height=0, name="greenCheck")
                    ui.Spacer()
                cb.model.add_value_changed_fn(lambda x: self._gather_value())
                self.item_models.append(cb.model)
                ui.Spacer(width=10)

        self._scatter_value()

    def _scatter_value(self):
        if self._edit_guard.is_guarded():
            return

        with self._edit_guard:
            for i in range(0, len(self.item_models)):
                curr_pos = (self.as_int >> i) & 1
                self.item_models[i].set_value(curr_pos)

    def _gather_value(self):
        if self._edit_guard.is_guarded():
            return

        with self._edit_guard:
            total = sum([m.as_int << i for i, m in enumerate(self.item_models)])
            self.set_value(total)


class GearingWidgetBuilder(UsdAttributeModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs):
        super().__init__(stage, [path.AppendProperty(prop.attr_name) for path in prim_paths], True, prop.metadata)

        def change_value(val):
            temp = Vt.FloatArray(1)
            temp[0] = val.as_float
            self.set_value(temp)
            pass

        with ui.HStack(spacing=HORIZONTAL_SPACING):
            UsdPropertiesWidgetBuilder._create_label(prop.attr_name, prop.metadata, label_kwargs)
            widget = ui.FloatDrag(min=Limits.FLT_NMAX, max=Limits.FLT_MAX, step=0.1)
            widget.model.set_value(self.get_first_array_item_as_float())
            widget.model.add_value_changed_fn(change_value)

    def get_first_array_item_as_float(self) -> float:
        self._update_value()
        if self._value is None:
            return 0.0
        else:
            return float(self._value[0])


class AngleWidgetBuilder(UsdAttributeModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs):
        super().__init__(stage, [path.AppendProperty(prop.attr_name) for path in prim_paths], True, prop.metadata)

        def change_value(val):
            self.set_value(math.radians(val.as_float))
            pass

        with ui.HStack():
            ui.Label(get_display_name(prop.metadata, prop.attr_name), name="label", width=0)
            fd = ui.FloatDrag(min=-360, max=360, step=1)
            fd.model.set_value(math.degrees(self.as_float))
            fd.model.add_value_changed_fn(change_value)


class HideWidgetBuilder(UsdAttributeModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs):
        super().__init__(stage, [path.AppendProperty(prop.attr_name) for path in prim_paths], False, prop.metadata)


class QuatEulerRotationBuilder(GfQuatEulerAttributeModel):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs):
        type_name = Sdf.ValueTypeNames.Find(prop.metadata.get(Sdf.PrimSpec.TypeNameKey))
        super().__init__(stage, [path.AppendProperty(prop.attr_name) for path in prim_paths], type_name.type, False, prop.metadata)

        with ui.HStack(spacing=HORIZONTAL_SPACING):
            label = UsdPropertiesWidgetBuilder._create_label(prop.attr_name, prop.metadata, label_kwargs)
            value_widget, mixed_overlay = UsdPropertiesWidgetBuilder._create_multi_float_drag_with_labels(
                model=self, labels=[("X", 0xFF5555AA), ("Y", 0xFF76A371), ("Z", 0xFFA07D4F)], comp_count=3, step=1)
            UsdPropertiesWidgetBuilder._create_control_state(self, value_widget=value_widget, mixed_overlay=mixed_overlay, label=label)
            value_widget.identifier = f"physprop_{prop.attr_name}"


class PrettyPrintTokenComboBuilder(TfTokenAttributeModel):
    class AllowedTokenItem(ui.AbstractItem):
        def __init__(self, token, name):
            super().__init__()
            self.token = token
            self.model = ui.SimpleStringModel(name)

        def __repr__(self):
            return f"{self.token}, {self.model.get_value_as_string()}"

    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs, pretty_names, additions=[]):
        self._names = pretty_names
        self._additions = additions
        super().__init__(stage, [path.AppendProperty(prop.attr_name) for path in prim_paths], False, prop.metadata)

        with ui.HStack(spacing=HORIZONTAL_SPACING):
            UsdPropertiesWidgetBuilder._create_label(prop.attr_name, prop.metadata, label_kwargs)
            with ui.ZStack():
                value_widget = ui.ComboBox(self)
                mixed_overlay = UsdPropertiesWidgetBuilder._create_mixed_text_overlay()
            UsdPropertiesWidgetBuilder._create_control_state(self, value_widget=value_widget, mixed_overlay=mixed_overlay)
            value_widget.identifier = f"physprop_{prop.attr_name}"

    def _update_allowed_token(self, token_item=AllowedTokenItem):
        self._allowed_tokens = []
        attributes = self._get_attributes()
        attr = attributes[0] if len(attributes) > 0 else None
        if not attr:
            return

        for n, t in enumerate(self._get_allowed_tokens(attr)):
            self._allowed_tokens.append(token_item(t, self._names[n]))

        for t, n in self._additions:
            self._allowed_tokens.append(token_item(t, n))

    def is_allowed_token(self, token):
        return token in [allowed_token.token for allowed_token in self._allowed_tokens]


class CustomTokenComboBuilder(PrettyPrintTokenComboBuilder):
    def __init__(self, stage, prop, prim_paths, label_kwargs, widget_kwargs, custom_names):
        super().__init__(stage, prop, prim_paths, label_kwargs, widget_kwargs, custom_names)

    def _get_allowed_tokens(self, _):
        return self._names


class ModelWithWidgetBuilder(UsdPropertiesWidgetBuilder):
    def __new__(cls, stage, prop, prim_paths, label_kwargs, widget_kwargs):
        model, widget = cls.build(stage, prop.prop_name, prop.metadata, prop.property_type, prim_paths, label_kwargs, widget_kwargs)
        setattr(model, "value_widget", widget)
        return model

    @classmethod
    def tftoken_builder(
        cls,
        stage,
        attr_name,
        type_name,
        metadata,
        prim_paths,
        additional_label_kwargs=None,
        additional_widget_kwargs=None,
    ):
        with ui.HStack(spacing=HORIZONTAL_SPACING):
            model = None
            cls._create_label(attr_name, metadata, additional_label_kwargs)
            tokens = metadata.get("allowedTokens")
            if tokens is not None and len(tokens) > 0:
                model = TfTokenAttributeModel(
                    stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata
                )
                widget_kwargs = {"name": "choices"}
                if additional_widget_kwargs:
                    widget_kwargs.update(additional_widget_kwargs)
                with ui.ZStack():
                    value_widget = ui.ComboBox(model, **widget_kwargs)
                    mixed_overlay = cls._create_mixed_text_overlay()
            else:
                model = UsdAttributeModel(
                    stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata
                )
                widget_kwargs = {"name": "models"}
                if additional_widget_kwargs:
                    widget_kwargs.update(additional_widget_kwargs)
                with ui.ZStack():
                    value_widget = ui.StringField(model, **widget_kwargs)
                    mixed_overlay = cls._create_mixed_text_overlay()
            cls._create_control_state(
                model=model, value_widget=value_widget, mixed_overlay=mixed_overlay, **widget_kwargs
            )
            return model, value_widget

    @classmethod
    def floating_point_builder(
        cls,
        stage,
        attr_name,
        type_name,
        metadata,
        prim_paths,
        additional_label_kwargs=None,
        additional_widget_kwargs=None,
    ):
        with ui.HStack(spacing=HORIZONTAL_SPACING):
            model_kwargs = cls._get_attr_value_range_kwargs(metadata)
            model = UsdAttributeModel(
                stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata, **model_kwargs
            )
            cls._create_label(attr_name, metadata, additional_label_kwargs)
            widget_kwargs = {"model": model}
            widget_kwargs.update(cls._get_attr_value_soft_range_kwargs(metadata))
            if additional_widget_kwargs:
                widget_kwargs.update(additional_widget_kwargs)
            with ui.ZStack():
                value_widget = cls._create_drag_or_slider(ui.FloatDrag, ui.FloatSlider, **widget_kwargs)
                mixed_overlay = cls._create_mixed_text_overlay()
            cls._create_control_state(value_widget=value_widget, mixed_overlay=mixed_overlay, **widget_kwargs)
            return model, value_widget

    @classmethod
    def integer_builder(
        cls,
        stage,
        attr_name,
        type_name,
        metadata,
        prim_paths,
        additional_label_kwargs=None,
        additional_widget_kwargs=None,
    ):
        with ui.HStack(spacing=HORIZONTAL_SPACING):
            cls._create_label(attr_name, metadata, additional_label_kwargs)
            model_kwargs = cls._get_attr_value_range_kwargs(metadata)
            if type_name.type == cls.tf_uint:
                model_kwargs["min"] = 0
            model = UsdAttributeModel(
                stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata, **model_kwargs
            )
            widget_kwargs = {"model": model}
            widget_kwargs.update(cls._get_attr_value_soft_range_kwargs(metadata))
            if additional_widget_kwargs:
                widget_kwargs.update(additional_widget_kwargs)
            with ui.ZStack():
                value_widget = cls._create_drag_or_slider(ui.IntDrag, ui.IntSlider, **widget_kwargs)
                mixed_overlay = cls._create_mixed_text_overlay()
            cls._create_control_state(value_widget=value_widget, mixed_overlay=mixed_overlay, **widget_kwargs)
            return model, value_widget

    @classmethod
    def bool_builder(
        cls,
        stage,
        attr_name,
        type_name,
        metadata,
        prim_paths,
        additional_label_kwargs=None,
        additional_widget_kwargs=None,
    ):
        with ui.HStack(spacing=HORIZONTAL_SPACING):
            model = UsdAttributeModel(stage, [path.AppendProperty(attr_name) for path in prim_paths], False, metadata)
            settings = carb.settings.get_settings()
            left_aligned = settings.get("ext/omni.kit.window.property/checkboxAlignment") == "left"

            if not left_aligned:
                if not additional_label_kwargs:
                    additional_label_kwargs = {}
                additional_label_kwargs["width"] = 0
            label = cls._create_label(attr_name, metadata, additional_label_kwargs)
            if not left_aligned:
                ui.Spacer(width=10)
                ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1))
                ui.Spacer(width=5)
            with ui.VStack(width=10):
                ui.Spacer()
                widget_kwargs = {"width": 10, "height": 0, "name": "greenCheck", "model": model}
                if additional_widget_kwargs:
                    widget_kwargs.update(additional_widget_kwargs)
                with ui.ZStack():
                    with ui.Placer(offset_x=0, offset_y=-2):
                        value_widget = ui.CheckBox(**widget_kwargs)
                    with ui.Placer(offset_x=1, offset_y=-1):
                        mixed_overlay = ui.Rectangle(
                            height=8, width=8, name="mixed_overlay", alignment=ui.Alignment.CENTER, visible=False
                        )
                ui.Spacer()
            if left_aligned:
                ui.Spacer(width=5)
                ui.Line(style={"color": 0x338A8777}, width=ui.Fraction(1))
            cls._create_control_state(
                value_widget=value_widget, mixed_overlay=mixed_overlay, **widget_kwargs, label=label
            )
            return model, value_widget


ModelWithWidgetBuilder.startup()
