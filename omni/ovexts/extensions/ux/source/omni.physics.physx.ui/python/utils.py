# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

import omni.ui as ui
from omni.kit.property.usd.usd_property_widget_builder import UsdPropertiesWidgetBuilder


def make_hide_cb(own_side: str, own_attr: str, other_attr):
    """Create a disable callback for a property covered by both Newton and PhysX schemas.

    Priority: PhysX authored > Newton authored > PhysX default.

    The callback returns ``(disabled, source_display_name, usd_attr_name)``:
    - ``disabled``: True if the own property is not currently providing the simulation value.
    - ``source_display_name``: human-readable name of the controlling side ("Newton" or "PhysX"),
      or "" if own side is in control.
    - ``usd_attr_name``: the USD attribute name that is controlling the value, or "".

    Args:
        own_side: ``"newton"`` or ``"physx"`` — identifies which side owns this property.
        own_attr: USD attribute name of this property (used by the "physx" side to check
            whether the PhysX attribute itself is authored before yielding to Newton).
        other_attr: USD attribute name(s) on the other side. Pass a list when multiple
            PhysX attributes map to one Newton property (e.g. hull vertex limit).
    """
    other_attrs = [other_attr] if isinstance(other_attr, str) else list(other_attr)

    def _cb(stage, prim_paths):
        if not prim_paths or stage is None:
            return False, "", ""

        prim = stage.GetPrimAtPath(str(prim_paths[0]))

        if own_side == "newton":
            # Newton property: disabled unless Newton is authored and no PhysX attr is authored.
            # Priority 1: PhysX authored → PhysX wins.
            for a in other_attrs:
                attr = prim.GetAttribute(a)
                if attr and attr.HasAuthoredValue():
                    return True, "PhysX", a
            # Priority 2: Newton authored, PhysX not → Newton wins.
            own = prim.GetAttribute(own_attr)
            if own and own.HasAuthoredValue():
                return False, "", ""
            # Priority 3: neither authored → PhysX default is used.
            return True, "PhysX", ""
        else:
            # PhysX property: disabled if own PhysX attr is NOT authored but Newton IS authored
            # (Newton is currently providing the fallback value).
            own = prim.GetAttribute(own_attr)
            if own and own.HasAuthoredValue():
                return False, "", ""
            for a in other_attrs:
                attr = prim.GetAttribute(a)
                if attr and attr.HasAuthoredValue():
                    return True, "Newton", a
            return False, "", ""

    _cb.watch_attrs = other_attrs
    return _cb


class DisableByCallbackBuilder(UsdPropertiesWidgetBuilder):
    """Property builder that renders a semi-transparent overlay when the property
    is not currently controlling simulation (determined by a priority callback).

    The builder wraps the standard property widget in a ZStack and places a
    black-with-transparency rectangle on top when ``disable_callback`` returns
    ``disabled=True``.

    A class-level registry maps (stage, prim_paths, attr_name) to each model as
    it is built.  When the linked property's builder has already run, both models
    are wired with bi-directional value-change subscriptions so either side
    updating causes both overlays to refresh.
    """

    # (id(stage), prim_paths_tuple, attr_name) -> (model, refresh_fn)
    _registry: dict = {}

    def __new__(cls, stage, prop, prim_paths, label_kwargs, widget_kwargs, disable_callback):
        def _tooltip(source_name, attr_name):
            if not source_name:
                return ""
            return f"Controlled by {attr_name}" if attr_name else f"Controlled by {source_name}"

        disabled, source_name, attr_name = disable_callback(stage, prim_paths)
        with ui.ZStack():
            model = cls.build(
                stage, prop.prop_name, prop.metadata, prop.property_type, prim_paths, label_kwargs, widget_kwargs
            )
            overlay = ui.Rectangle(
                identifier=f"newton_physx_overlay_{prop.prop_name.replace(':', '_')}",
                width=ui.Fraction(1),
                height=ui.Fraction(1),
                style={"background_color": ui.color(0, 0, 0, 64), "border_radius": 4},
                visible=disabled,
                tooltip=_tooltip(source_name, attr_name),
            )

        def _refresh(*_):
            disabled, source_name, attr_name = disable_callback(stage, prim_paths)
            overlay.visible = disabled
            overlay.set_tooltip(_tooltip(source_name, attr_name))

        if model is not None:
            model._remove_if_default = True

        if model is not None and hasattr(model, "subscribe_value_changed_fn"):
            if not hasattr(model, "_newton_physx_disable_subs"):
                model._newton_physx_disable_subs = []

            # Subscribe to own value changes.
            model._newton_physx_disable_subs.append(model.subscribe_value_changed_fn(_refresh))

            prim_key = tuple(str(p) for p in prim_paths)
            own_key = (id(stage), prim_key, prop.prop_name)
            cls._registry[own_key] = (model, _refresh)

            # Wire cross-side subscriptions with already-built linked models.
            for watch_attr in getattr(disable_callback, "watch_attrs", []):
                watch_key = (id(stage), prim_key, watch_attr)
                entry = cls._registry.get(watch_key)
                if entry is not None:
                    watch_model, watch_refresh = entry
                    # This model refreshes when the linked model changes.
                    model._newton_physx_disable_subs.append(
                        watch_model.subscribe_value_changed_fn(_refresh)
                    )
                    # Linked model refreshes when this model changes.
                    watch_model._newton_physx_disable_subs.append(
                        model.subscribe_value_changed_fn(watch_refresh)
                    )

        return model
