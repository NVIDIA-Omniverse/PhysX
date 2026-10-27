# SPDX-FileCopyrightText: Copyright (c) 2021-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

"""Fake Newton schema for the property-panel visual tests.

The Newton extension schemas (omni.usd.schema.newton) are not version-locked by
this repo, so their real property list can change independently of us. That makes
the property-panel visual goldens drift whenever the Newton schema changes, even
though nothing in our own code changed.

To insulate these tests from Newton schema changes we replace each Newton frame's
property list with a single fixed property that we author in an in-memory Sdf
layer here. The goldens then depend only on us, immune to Newton adding, removing,
or reordering properties.

We deliberately keep the *real* Newton property names (newton:selfCollisionEnabled
for the articulation frame, newton:timeStepsPerSecond for the scene frame) rather
than inventing fake names, for two reasons:
  * NewtonPhysXUiDefinitions.property_builders is keyed on these names, so the
    DisableByCallbackBuilder + make_hide_cb priority-overlay logic stays wired to
    our fake property and is exercised by these tests.
  * These names are part of the PhysX<->Newton integration contract; if Newton
    ever renames them our own make_hide_cb logic has to change too, and this test
    breaking is the correct signal that it did.

These are the two schemas that render a *separate* Newton frame
(NewtonPhysXUiDefinitions.extensions): NewtonArticulationRootAPI (Articulation
Root panel) and NewtonSceneAPI (Physics Scene panel).

Call install() from a test's setUp and restore() from its tearDown.
"""

import omni.kit.property.physics as kit_property_physics
from omni.kit.property.physics.widgets import ExtensionSchemaWidget
from omni.physics.physxui.newton_schemas import NewtonPhysXUiDefinitions
from pxr import Sdf


_FAKE_NEWTON_LAYER = Sdf.Layer.CreateAnonymous("fake_newton_schema.usda")


def _make_fake_newton_spec(prim_name, prop_name, display_name, type_name, default):
    """Author a single fake property spec on the in-memory layer and return it."""
    prim_spec = Sdf.PrimSpec(_FAKE_NEWTON_LAYER, prim_name, Sdf.SpecifierClass)
    attr_spec = Sdf.AttributeSpec(prim_spec, prop_name, type_name)
    attr_spec.default = default
    attr_spec.displayName = display_name
    return attr_spec


# The single fake property shown in each Newton frame, keyed by the Newton
# extension schema type name (Newton schemas have no python binding, so the manager
# keys widgets by type name). We use the real Newton property names so that
# NewtonPhysXUiDefinitions.property_builders (DisableByCallbackBuilder + make_hide_cb)
# stays wired to them.
_FAKE_NEWTON_PROPS = {
    "NewtonArticulationRootAPI": [
        _make_fake_newton_spec(
            "FakeNewtonArticulationRoot", "newton:selfCollisionEnabled",
            "Self Collision Enabled", Sdf.ValueTypeNames.Bool, False,
        )
    ],
    "NewtonSceneAPI": [
        _make_fake_newton_spec(
            "FakeNewtonScene", "newton:timeStepsPerSecond",
            "Time Steps Per Second", Sdf.ValueTypeNames.UInt, 60,
        )
    ],
}


# Frame titles, overridden to make it obvious in the golden that this is our fake
# Newton frame and not the real omni.usd.schema.newton one.
_FAKE_NEWTON_TITLES = {
    "NewtonArticulationRootAPI": "Fake Newton Articulation Root (PhysX)",
    "NewtonSceneAPI": "Fake Newton Scene (PhysX)",
}


class _FakeNewtonExtensionWidget(ExtensionSchemaWidget):
    """ExtensionSchemaWidget that renders a single fixed property (using the real
    Newton property name, so make_hide_cb stays wired) under a clearly-fake frame
    title, so the property-panel visual goldens are decoupled from the real
    omni.usd.schema.newton property list."""

    def __init__(self, title, schema, parent_schema, builders=None):
        # `schema` is the extension schema type name (a str) for Newton schemas.
        title = _FAKE_NEWTON_TITLES.get(schema, title)
        super().__init__(title, schema, parent_schema, builders)
        self._ext_prop_specs = list(_FAKE_NEWTON_PROPS.get(schema, []))


# Custom widgets for the two Newton frames.
_FAKE_NEWTON_FRAME_WIDGETS = {
    "NewtonArticulationRootAPI": _FakeNewtonExtensionWidget,
    "NewtonSceneAPI": _FakeNewtonExtensionWidget,
}


def _do_register(widgets):
    kit_property_physics.register_parent_schema(
        "newton",
        "PhysX",
        widgets,
        NewtonPhysXUiDefinitions.property_builders,
        NewtonPhysXUiDefinitions.property_order,
        NewtonPhysXUiDefinitions.extensions,
        NewtonPhysXUiDefinitions.extras,
        NewtonPhysXUiDefinitions.ignore,
        NewtonPhysXUiDefinitions.internal_extensions,
    )


def _register_newton_parent_schema(widgets):
    kit_property_physics.unregister_parent_schema("newton")
    try:
        _do_register(widgets)
    except Exception:
        # register failed after we already unregistered - don't leave "newton"
        # unregistered for the rest of the run; fall back to the production
        # registration so the remaining tests still see a valid Newton mapping.
        _do_register(NewtonPhysXUiDefinitions.widgets)
        raise


def install():
    """Replace the live Newton frames with the fake ones. Call from setUp()."""
    _register_newton_parent_schema(_FAKE_NEWTON_FRAME_WIDGETS)


def restore():
    """Restore the production Newton registration. Call from tearDown()."""
    _register_newton_parent_schema(NewtonPhysXUiDefinitions.widgets)
