# SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#

"""Newton schema discovery and UI definitions for the PhysX simulation engine."""

from typing import Set, Tuple


def _get_newton_schema_names(plugin_name: str) -> Tuple[Set[str], Set[str]]:
    from pxr import Plug, Usd, Tf

    prim_type_names: Set[str] = set()
    api_schema_names: Set[str] = set()

    plug_registry = Plug.Registry()
    schema_registry = Usd.SchemaRegistry()

    plugin = plug_registry.GetPluginWithName(plugin_name)
    if plugin is None:
        return prim_type_names, api_schema_names

    all_types = Tf.Type.Find(Usd.SchemaBase).GetAllDerivedTypes()
    for tf_type in all_types:
        if not plugin.DeclaresType(tf_type):
            continue
        schema_type_name = schema_registry.GetSchemaTypeName(tf_type)
        if not schema_type_name:
            continue
        schema_kind = schema_registry.GetSchemaKind(tf_type)
        if schema_kind in (Usd.SchemaKind.AbstractTyped, Usd.SchemaKind.ConcreteTyped):
            prim_type_names.add(schema_type_name)
        elif schema_kind in (Usd.SchemaKind.SingleApplyAPI, Usd.SchemaKind.MultipleApplyAPI):
            api_schema_names.add(schema_type_name)

    return prim_type_names, api_schema_names


def get_newton_schema_names() -> Tuple[Set[str], Set[str]]:
    """Return Newton schema type names that act as PhysX capabilities.

    Schemas not supported by the PhysX Newton compat layer are excluded from
    the capability sets so they are not advertised as active PhysX features.
    They are still visible in the property panel unless listed in
    NewtonPhysXUiDefinitions.ignore.
    """
    prim_type_names, api_schema_names = _get_newton_schema_names("newton")

    # These schemas are recognized by the property panel but do not indicate
    # PhysX simulation capability (either not mapped to PhysX or solver-internal).
    not_capabilities = {
        "NewtonSceneAPI",
        "NewtonXpbdSceneAPI",
        "NewtonKaminoSceneAPI",
        "NewtonCollisionAPI",
        "NewtonMeshCollisionAPI",
        "NewtonMaterialAPI",
    }
    for name in not_capabilities:
        prim_type_names.discard(name)
        api_schema_names.discard(name)

    return prim_type_names, api_schema_names


# ---------------------------------------------------------------------------
# UI definitions
# ---------------------------------------------------------------------------

from pxr import UsdPhysics

from .utils import DisableByCallbackBuilder, make_hide_cb


class NewtonPhysXUiDefinitions:
    """UI definitions (widgets, property builders, ordering) for Newton schemas
    in the context of the PhysX simulation engine.

    Priority rule (from NewtonCompat.h):
        1. PhysX attribute authored  → PhysX wins
        2. Newton attribute authored → Newton provides the fallback value
        3. PhysX default

    Properties covered by this priority logic carry a DisableByCallbackBuilder
    that overlays a semi-transparent rectangle when the property is not the
    active source of the simulation value.
    """

    ignore = {
        # Not mapped to PhysX — would confuse users into editing no-op properties.
        "NewtonMaterialAPI",
        # Solver-variant scene APIs — XPBD/Kamino parameters have no PhysX equivalents.
        "NewtonXpbdSceneAPI",
        "NewtonKaminoSceneAPI",
    }

    extensions = {
        UsdPhysics.Scene: ["NewtonSceneAPI"],
        UsdPhysics.ArticulationRootAPI: ["NewtonArticulationRootAPI"],
    }

    internal_extensions = {
        UsdPhysics.CollisionAPI: ["NewtonCollisionAPI"],
        UsdPhysics.MeshCollisionAPI: ["NewtonMeshCollisionAPI"],
    }

    widgets = {}

    property_builders = {
        # Newton properties — overlayed when the corresponding PhysX attribute is authored
        # (PhysX takes priority; the Newton value is not used in that case).
        # SCENE
        "newton:timeStepsPerSecond": [
            DisableByCallbackBuilder,
            make_hide_cb("newton", "newton:timeStepsPerSecond", "physxScene:timeStepsPerSecond"),
        ],
        # ARTICULATION
        "newton:selfCollisionEnabled": [
            DisableByCallbackBuilder,
            make_hide_cb("newton", "newton:selfCollisionEnabled", "physxArticulation:enabledSelfCollisions"),
        ],
        # SHAPE
        "newton:contactMargin": [
            DisableByCallbackBuilder,
            make_hide_cb("newton", "newton:contactMargin", "physxCollision:restOffset"),
        ],
        "newton:contactGap": [
            DisableByCallbackBuilder,
            make_hide_cb("newton", "newton:contactGap", "physxCollision:contactOffset"),
        ],
        "newton:maxHullVertices": [
            DisableByCallbackBuilder,
            make_hide_cb(
                "newton",
                "newton:maxHullVertices",
                ["physxConvexHullCollision:hullVertexLimit", "physxConvexDecompositionCollision:hullVertexLimit"],
            ),
        ],
    }

    property_order = {}
    extras = {}
