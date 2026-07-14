# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable

import carb


@dataclass(frozen=True)
class CustomPropertyDef:
    # Parameters matching UiProp.from_custom(name, display_name, display_group, type_name, default, doc)
    usd_name: str
    display_name: str
    display_group: str = ""
    type_name: str = "string"
    default: object = ""
    doc: str = ""


@dataclass
class CustomPropertyWidgetDef:
    """
    Definition of a "custom properties widget" (a collapsible section in the Physics property window)
    that displays a list of registered custom properties.

    The widget itself is owned/created by `omni.kit.property.physics` and is shown/hidden based on
    `visible_when` evaluated against the selected prim(s).
    """

    widget_title: str
    parent_schema: str | None
    visible_when: Callable[[object], bool]
    position: int = 1
    widget_name: str | None = None
    properties: list[CustomPropertyDef] = field(default_factory=list)


_custom_property_widgets: dict[str, CustomPropertyWidgetDef] = {}


def _sanitize_widget_name(widget_title: str) -> str:
    # Keep stable, avoid spaces/specials.
    safe = []
    for ch in widget_title.strip():
        if ch.isalnum():
            safe.append(ch.lower())
        else:
            safe.append("_")
    safe_s = "".join(safe).strip("_") or "custom"
    return f"physx_custom_properties_{safe_s}"


def register_custom_property_widget(
    widget_title: str,
    *,
    parent_schema: str | None = None,
    visible_when: Callable[[object], bool] | None = None,
    position: int = 1,
    widget_name: str | None = None,
) -> CustomPropertyWidgetDef:
    if not widget_title or not widget_title.strip():
        raise ValueError("widget_title must be a non-empty string")

    if visible_when is None:
        visible_when = lambda _prim: True

    if widget_name is None:
        widget_name = _sanitize_widget_name(widget_title)

    if widget_title in _custom_property_widgets:
        # Update in-place to preserve already registered properties.
        d = _custom_property_widgets[widget_title]
        d.parent_schema = parent_schema
        d.visible_when = visible_when
        d.position = position
        d.widget_name = widget_name
        return d

    d = CustomPropertyWidgetDef(
        widget_title=widget_title,
        parent_schema=parent_schema,
        visible_when=visible_when,
        position=position,
        widget_name=widget_name,
    )
    _custom_property_widgets[widget_title] = d
    return d


def unregister_custom_property_widget(widget_title: str) -> None:
    _custom_property_widgets.pop(widget_title, None)


def get_custom_property_widget(widget_title: str) -> CustomPropertyWidgetDef | None:
    return _custom_property_widgets.get(widget_title)


def iter_custom_property_widgets() -> list[CustomPropertyWidgetDef]:
    # deterministic ordering for UIs / tests
    return [d for _, d in sorted(_custom_property_widgets.items(), key=lambda kv: kv[0].lower())]


def register_custom_property(
    *,
    usd_name: str,
    display_name: str | None = None,
    display_group: str = "",
    type_name: str = "string",
    default: object = "",
    doc: str = "",
    widget_title: str,
) -> None:
    d = _custom_property_widgets.get(widget_title)
    if d is None:
        carb.log_error(
            f"[physx property] Custom property widget '{widget_title}' is not registered. "
            "Register the widget first, then add properties to it."
        )
        return

    if not usd_name or not usd_name.strip():
        raise ValueError("usd_name must be a non-empty string")
    if display_name is None or not str(display_name).strip():
        raise ValueError("display_name must be a non-empty string")

    usd_name = usd_name.strip()
    display_name = str(display_name).strip()
    display_group = str(display_group or "")
    type_name = str(type_name or "string")
    doc = str(doc or "")

    # Replace existing entry (same usd_name) to allow retitling.
    d.properties = [p for p in d.properties if p.usd_name != usd_name]
    d.properties.append(
        CustomPropertyDef(
            usd_name=usd_name,
            display_name=display_name,
            display_group=display_group,
            type_name=type_name,
            default=default,
            doc=doc,
        )
    )


def unregister_custom_property(*, usd_name: str, widget_title: str) -> None:
    d = _custom_property_widgets.get(widget_title)
    if d is None:
        return
    d.properties = [p for p in d.properties if p.usd_name != usd_name]


def get_custom_properties(widget_title: str) -> list[CustomPropertyDef]:
    d = _custom_property_widgets.get(widget_title)
    if d is None:
        return []
    # Deterministic ordering.
    return sorted(d.properties, key=lambda p: p.usd_name.lower())
