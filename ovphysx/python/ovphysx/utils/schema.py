# SPDX-FileCopyrightText: Copyright (c) 2020-2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# @implements REQ-PYTHON-UTILS-001
# @covers AC-1 AC-2 AC-3 AC-12 AC-15

"""USD schema introspection and API-schema property manipulation.

These helpers work against the USD schema registry, so they cover schemas that
were registered from a codeless plugin as well as compiled ones. That is what
makes them useful outside Kit: a caller that only has ``pxr`` can still
enumerate a PhysX API schema's properties, snapshot them, and restore them.

How properties are reached
^^^^^^^^^^^^^^^^^^^^^^^^^^
ovphysx ships the PhysX schemas codeless, so ``TfType.pythonClass`` is ``None``
for them and every property here is reached through the registry:

* :func:`get_schema_attribute` and :func:`get_schema_relationship` return the
  ``Sdf`` property spec.
* :func:`create_api_schema_property_cache` records the property's value type, and
  :func:`apply_api_schema_property_cache` replays it through the generic
  ``Usd.Prim`` API rather than a typed ``Create*Attr`` method.
"""

import logging
import typing

from pxr import Sdf, Usd, UsdGeom

from ._deprecation import deprecated_alias, deprecated_parameter

logger = logging.getLogger(__name__)

__all__ = [
    "get_tf_type_compatible",
    "get_schema_prim_def",
    "get_schema_property_names",
    "get_schema_attribute",
    "get_schema_relationship",
    "has_schema",
    "descendant_has_api",
    "ancestor_has_api",
    "get_schema_instances",
    "remove_api_schema_properties",
    "remove_multiple_api_schema_properties",
    "create_api_schema_property_cache",
    "apply_api_schema_property_cache",
    "create_multiple_api_schema_property_cache",
]


def get_tf_type_compatible(schema_type_or_type_name):
    """Resolve a schema identifier to a TfType, passing a TfType through.

    Accepts a schema type-name string (the only form available for a codeless
    schema), a typed schema class, or an existing ``Tf.Type``.

    Args:
        schema_type_or_type_name: A schema type-name string, schema class, or type.
    """
    if isinstance(schema_type_or_type_name, str):
        return Usd.SchemaRegistry().GetTypeFromSchemaTypeName(schema_type_or_type_name)
    # A typed schema class carries its own static TfType. Codeless schemas have
    # no class, so this branch only ever sees Usd/UsdPhysics schemas.
    get_static_type = getattr(schema_type_or_type_name, "_GetStaticTfType", None)
    if get_static_type is not None:
        return get_static_type()
    return schema_type_or_type_name


def get_schema_prim_def(schema):
    """Get the registry prim definition for an applied API or concrete schema.

    Args:
        schema: A schema type-name string or schema class.
    """
    schema_type = get_tf_type_compatible(schema)
    isApi = Usd.SchemaRegistry().IsAppliedAPISchema(schema_type)
    schemaToken = (
        Usd.SchemaRegistry().GetAPISchemaTypeName(schema_type)
        if isApi
        else Usd.SchemaRegistry().GetConcreteSchemaTypeName(schema_type)
    )
    return (
        Usd.SchemaRegistry().FindAppliedAPIPrimDefinition(schemaToken)
        if isApi
        else Usd.SchemaRegistry().FindConcretePrimDefinition(schemaToken)
    )


def _get_schema_registry_properties(schema):
    primDef = get_schema_prim_def(schema)
    if not primDef:
        raise ValueError(
            f"No prim definition for schema {schema!r}. The schema may not be "
            "registered; see ovphysx.utils.codeless.register_schemas()."
        )
    return [(name, primDef.GetSchemaPropertySpec(name)) for name in primDef.GetPropertyNames()]


def _get_schema_property_name(name):
    """Reduce a registry property name to its bare name.

    A multiple-apply schema registers its properties under the template
    ``prefix:__INSTANCE_NAME__:name``; everything else is already bare.
    """
    if Usd.SchemaRegistry.IsMultipleApplyNameTemplate(name):
        return Usd.SchemaRegistry.GetMultipleApplyNameTemplateBaseName(name)
    return name


def get_schema_property_names(schema):
    """List the property names a schema registers.

    Args:
        schema: A schema type-name string or schema class.
    """
    return [_get_schema_property_name(name) for name, _ in _get_schema_registry_properties(schema)]


def get_schema_attribute(schema) -> typing.List[typing.Tuple[str, Sdf.AttributeSpec]]:
    """List (name, spec) pairs for a schema's attributes.

    Args:
        schema: A schema type-name string or schema class.

    Returns:
        ``(bare_property_name, Sdf.AttributeSpec)`` pairs. Read
        ``spec.typeName`` for the value type.
    """
    return [
        (_get_schema_property_name(name), spec)
        for name, spec in _get_schema_registry_properties(schema)
        if isinstance(spec, Sdf.AttributeSpec)
    ]


def get_schema_relationship(schema) -> typing.List[typing.Tuple[str, Sdf.RelationshipSpec]]:
    """List (name, spec) pairs for a schema's relationships.

    Args:
        schema: A schema type-name string or schema class.

    Returns:
        ``(bare_property_name, Sdf.RelationshipSpec)`` pairs. See
        :func:`get_schema_attribute` for why this is a spec and not a class.
    """
    return [
        (_get_schema_property_name(name), spec)
        for name, spec in _get_schema_registry_properties(schema)
        if isinstance(spec, Sdf.RelationshipSpec)
    ]


@deprecated_parameter("schemaName", "schema_name")
def has_schema(prim, schema_name):
    """Check whether a prim has an applied schema, matching by name.

    Args:
        prim:       The Usd.Prim to check.
        schema_name: The applied schema name, e.g. "PhysicsRigidBodyAPI".
    """
    return schema_name in prim.GetAppliedSchemas()


def descendant_has_api(name, prim):
    """Check whether a prim or any descendant has an API schema applied.

    A prim that resets its xform stack terminates the descent, because its
    subtree no longer inherits the ancestor's transform. The boundary is read
    with ``UsdGeom.Xformable.GetResetXformStack()``.

    Args:
        name: The API schema to look for, as a schema identifier string, a
              typed schema class, or a ``Tf.Type``.
        prim: The Usd.Prim to start from.
    """
    children = prim.GetChildren()
    if prim.HasAPI(name):
        return True
    elif children:
        xformable = UsdGeom.Xformable(prim)
        if xformable and xformable.GetResetXformStack():
            return False
        for i in children:
            result = descendant_has_api(name, i)
            if result:
                return True
    return False


def ancestor_has_api(name, prim):
    """Check whether a prim or any ancestor has an API schema applied.

    A prim that resets its xform stack terminates the ascent, because it no
    longer inherits its ancestors' transform. The boundary is read with
    ``UsdGeom.Xformable.GetResetXformStack()``.

    Args:
        name: The API schema to look for, as a schema identifier string, a
              typed schema class, or a ``Tf.Type``.
        prim: The Usd.Prim to start from.
    """
    if prim.HasAPI(name):
        return True
    elif prim.GetParent():
        xformable = UsdGeom.Xformable(prim)
        if xformable and xformable.GetResetXformStack():
            return False
        return ancestor_has_api(name, prim.GetParent())
    return False


def get_schema_instances(prim, schema_type_name):
    """Get the instance names of a multiple-apply schema applied to a prim.

    Args:
        prim:             The Usd.Prim to inspect.
        schema_type_name: The multiple-apply schema type name.
    """
    return {s[len(schema_type_name) + 1 :] for s in prim.GetAppliedSchemas() if s.startswith(schema_type_name)}


def remove_api_schema_properties(api, prim):
    """Remove every property an API schema defines from a prim.

    Args:
        api:  The API schema type-name string or schema class.
        prim: The Usd.Prim to strip.
    """
    rbProp = get_schema_property_names(api)
    for prop in rbProp:
        prim.RemoveProperty(prop)


def remove_multiple_api_schema_properties(api, prim, api_prefix, multiple_token):
    """Remove one instance of a multiple-apply API schema's properties.

    Args:
        api:            The API schema type-name string or schema class.
        prim:           The Usd.Prim to strip.
        api_prefix:     The property namespace prefix, e.g. "physxCookedData".
        multiple_token: The instance name.
    """
    rbProp = get_schema_property_names(api)
    for prop in rbProp:
        name = _get_property_multiple_name(api_prefix, multiple_token, prop)
        prim.RemoveProperty(name)


def _get_property_multiple_name(api_prefix, multiple_token, name):
    return Usd.SchemaRegistry.MakeMultipleApplyNameInstance(
        Usd.SchemaRegistry.MakeMultipleApplyNameTemplate(api_prefix, name), multiple_token
    )


def _get_authored_value(prim, name):
    """The prim's authored value for an attribute, or None if it has none.

    A resolved schema fallback is not an authored value, and recording one
    would make restoring the cache author it explicitly.
    """
    attr = prim.GetAttribute(name)
    return attr.Get() if attr and attr.HasAuthoredValue() else None


def _get_authored_targets(prim, name):
    """The prim's authored targets for a relationship, or None if it has none.

    An empty return is not the same as ``None``: it means the relationship was
    authored with no targets, which is an opinion of its own and one the replay
    has to reproduce. USD composes an unauthored relationship to an empty target
    list too, so ``HasAuthoredTargets`` is the only thing that separates them.
    """
    rel = prim.GetRelationship(name)
    return rel.GetTargets() if rel and rel.HasAuthoredTargets() else None


def create_api_schema_property_cache(api, prim):
    """Snapshot an API schema's attribute and relationship values on a prim.

    The result is consumed by :func:`apply_api_schema_property_cache`, which makes
    it possible to remove an API schema and restore its authored values later.

    Only authored state is recorded. The snapshot holds one entry per property
    the API declares, and what that entry carries is the property's authored
    value or targets, or nothing at all: an attribute that has only its schema
    fallback, and a relationship with no authored targets, are both snapshotted
    as unset.

    "Authored" is USD's own sense of the word throughout: an opinion exists
    somewhere in the prim's composed property stack, as opposed to the value
    coming from a schema fallback. An opinion arriving over a reference, an
    inherit, a specialize, a variant or a weaker sublayer therefore counts as
    authored and is recorded. The snapshot does not tell a local opinion from a
    composed one; what it excludes is the fallback.

    A relationship needs ``Usd.Relationship.HasAuthoredTargets`` for that
    distinction, because an unauthored relationship and one authored with an
    empty target list both compose to an empty list. An unauthored one is stored
    with ``None`` targets, as an unauthored attribute is stored with a ``None``
    value; an authored one is stored with its target list whether or not that
    list is empty. :func:`apply_api_schema_property_cache` skips ``None``
    targets and not the empty list, so a relationship authored with no targets
    is replayed and one carrying no opinion is left alone.

    The cache exists for
    :func:`~ovphysx.utils.schema.remove_api_schema_properties`, not for the
    removal of the API schema: ``Usd.Prim.RemoveAPI`` and
    :func:`ovphysx.utils.codeless.remove_api` drop the prim's ``apiSchemas``
    entry and leave every property authored under it in place.

    :func:`~ovphysx.utils.authoring.remove_collider` strips the approximation
    and cooked-data APIs' properties itself and leaves ``PhysxCollisionAPI``'s,
    so removing those is the caller's to ask for and snapshotting them first is
    how the caller gets them back.

    Example::

        from pxr import UsdPhysics

        from ovphysx.utils import (
            apply_api_schema_property_cache,
            create_api_schema_property_cache,
            remove_api_schema_properties,
            remove_collider,
            set_collider,
        )

        # prim is a UsdGeom.Mesh set up as a convexHull collider, carrying
        # authored physxCollision:contactOffset and physxCollision:restOffset
        # and a physxConvexHullCollision:hullVertexLimit.
        tuning = create_api_schema_property_cache("PhysxCollisionAPI", prim)

        # Takes the hull API's own opinions with it; leaves the two offsets,
        # which is why they have to be removed explicitly here.
        remove_collider(prim)
        remove_api_schema_properties("PhysxCollisionAPI", prim)

        set_collider(prim, UsdPhysics.Tokens.convexDecomposition)
        apply_api_schema_property_cache(tuning, prim)

    The prim ends up approximated as a convex decomposition, carrying the two
    offsets it started with and no leftover ``physxConvexHullCollision``
    opinion. The other attributes ``PhysxCollisionAPI`` declares stay
    unauthored, because the snapshot recorded no value for them.

    Args:
        api:  The API schema type-name string or schema class.
        prim: The Usd.Prim to read.

    Returns:
        ``[attributes, relationships]``, where each attribute entry is
        ``(name, value_type, value)`` and each relationship entry is
        ``(name, targets)``. The value type is the ``Sdf.ValueTypeName`` needed
        to recreate the property, a codeless schema having no class to name.

        ``value`` and ``targets`` are both ``None`` for an unauthored property.
        ``targets`` is an empty list only for a relationship authored with no
        targets, which is what lets that opinion survive a restore.
    """
    return [
        [
            (name, spec.typeName, _get_authored_value(prim, name))
            for name, spec in get_schema_attribute(api)
        ],
        [(name, _get_authored_targets(prim, name)) for name, _ in get_schema_relationship(api)],
    ]


def apply_api_schema_property_cache(cache, prim, multiple_api_token=None):
    """Restore values captured by :func:`create_api_schema_property_cache`.

    Replays the cache through the generic ``Usd.Prim`` property API, so it works
    for codeless and compiled schemas alike. Apply the owning API schema to the
    prim first; this only authors properties.

    Unset properties -- a ``None`` attribute value, or ``None`` relationship
    targets -- are skipped so that restoring does not author an opinion the
    snapshot did not have. The relationship test is ``targets is None`` and not
    ``not targets``: an empty target list is how an authored-empty relationship
    is snapshotted. See :func:`create_api_schema_property_cache`.

    Args:
        cache:              The cache to replay.
        prim:               The Usd.Prim to author on.
        multiple_api_token: The instance name, for a multiple-apply schema.
    """
    def instanced(name):
        # Only a multiple-apply cache carries __INSTANCE_NAME__ templates; a
        # single-apply cache stores names that are already complete.
        if multiple_api_token is None or not Usd.SchemaRegistry.IsMultipleApplyNameTemplate(name):
            return name
        return Usd.SchemaRegistry.MakeMultipleApplyNameInstance(name, multiple_api_token)

    for name, value_type, value in cache[0]:
        if value is None:
            continue
        prim.CreateAttribute(instanced(name), value_type).Set(value)

    for name, targets in cache[1]:
        if targets is None:
            continue
        prim.CreateRelationship(instanced(name)).SetTargets(targets)


def create_multiple_api_schema_property_cache(api, prim, api_prefix, multiple_token):
    """Snapshot one instance of a multiple-apply API schema's properties.

    Args:
        api:            The API schema type-name string or schema class.
        prim:           The Usd.Prim to read.
        api_prefix:     The property namespace prefix, e.g. "physxCookedData".
        multiple_token: The instance name.

    Returns:
        The same shape as :func:`create_api_schema_property_cache`, except that names
        are recorded as ``prefix:__INSTANCE_NAME__:name`` templates.
        :func:`apply_api_schema_property_cache` substitutes its ``multiple_api_token``
        into them, so a snapshot can be replayed onto a different instance. It
        records authored state only, on the same terms -- including ``None``
        targets for a relationship instance with no authored ones, as against an
        empty list for one authored with none.
    """

    def template(name):
        return Usd.SchemaRegistry.MakeMultipleApplyNameTemplate(api_prefix, name)

    def instance(name):
        return _get_property_multiple_name(api_prefix, multiple_token, name)

    return [
        [
            (template(name), spec.typeName, _get_authored_value(prim, instance(name)))
            for name, spec in get_schema_attribute(api)
        ],
        [
            (template(name), _get_authored_targets(prim, instance(name)))
            for name, _ in get_schema_relationship(api)
        ],
    ]


# Deprecated alias; see the note beside authoring.py's for why it is not in
# __all__ (AC-15).
hasSchema = deprecated_alias(has_schema, "hasSchema")
